/*
 * EffectsCalculator.cpp
 *
 *  Created on: 27.01.21
 *      Author: Jon Lidgard, Yannick Richter, Vincent Manoukian
 *
 * ────── ODrive-FFB-Wheel fork ──────────────────────────────────────────────
 * Cópia forkada de OpenFFBoard upstream (v1.17.0, commit ac68fba).
 * Localização upstream: Firmware/FFBoard/Src/EffectsCalculator.cpp
 *
 * Sem modificações funcionais nesse .cpp por ora — modificações estão no .h:
 *   - Acessores públicos: getGainStruct(), getFilterStruct(),
 *     getFilterStructDefault(), getGlobalGain(), getFilterProfileId(),
 *     setFilterProfileToCustom(), applyFilterChanges()
 *
 * Pra atualizar contra upstream: diff aqui e em inc/EffectsCalculator.h.
 * ───────────────────────────────────────────────────────────────────────────
 */

#include <stdint.h>
#include <math.h>
#include "EffectsCalculator.h"
#include "Axis.h"
#include "ledEffects.h"
#include "eeprom_addresses.h"  // ADR_FFB_*
#include "utils.hpp"            // our_arm_sin_f32 table-interpolated sine


#define EFFECT_STATE_INACTIVE 0

ClassIdentifier EffectsCalculator::info = {
		  .name = "Effects" ,
		  .id	= CLSID_EFFECTSCALC,
		  .visibility = command_handler_stub::ClassVisibility::hidden
};
const ClassIdentifier EffectsCalculator::getInfo(){
	return info;
}

EffectsStateGuard::EffectsStateGuard(EffectsCalculator& calculator)
	: calculator_(calculator), locked_(calculator_.lockEffectState()) {}

EffectsStateGuard::~EffectsStateGuard() {
	if (locked_) calculator_.unlockEffectState();
}

bool EffectsCalculator::lockEffectState() {
	return effects_state_mutex_.Lock();
}

void EffectsCalculator::unlockEffectState() {
	effects_state_mutex_.Unlock();
}

EffectsCalculator::EffectsCalculator() : CommandHandler("fx", CLSID_EFFECTSCALC),
		Thread("FXCalc", EFFECT_THREAD_MEM, EFFECT_THREAD_PRIO)
{
	restoreFlash();

	CommandHandler::registerCommands();
	registerCommand("filterCfFreq", EffectsCalculator_commands::ffbfiltercf, "Constant force filter frequency", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("filterCfQ", EffectsCalculator_commands::ffbfiltercf_q, "Constant force filter Q-factor", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("spring", EffectsCalculator_commands::spring, "Spring gain", CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("friction", EffectsCalculator_commands::friction, "Friction gain", CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("damper", EffectsCalculator_commands::damper, "Damper gain", CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("inertia", EffectsCalculator_commands::inertia, "Inertia gain", CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("effects", EffectsCalculator_commands::effects, "USed effects since reset (Info print as str). set 0 to reset", CMDFLAG_GET | CMDFLAG_SET | CMDFLAG_INFOSTRING);
	registerCommand("effectsDetails", EffectsCalculator_commands::effectsDetails, "List effects details. set 0 to reset", CMDFLAG_GET | CMDFLAG_SET  | CMDFLAG_STR_ONLY | CMDFLAG_GETADR);
	registerCommand("effectsForces", EffectsCalculator_commands::effectsForces, "List actual effects forces.", CMDFLAG_GET | CMDFLAG_GETADR);

	registerCommand("damper_f", EffectsCalculator_commands::damper_f, "Damper biquad freq", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("damper_q", EffectsCalculator_commands::damper_q, "Damper biquad q", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("friction_f", EffectsCalculator_commands::friction_f, "Friction biquad freq", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("friction_q", EffectsCalculator_commands::friction_q, "Friction biquad q", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("inertia_f", EffectsCalculator_commands::inertia_f, "Inertia biquad freq", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("inertia_q", EffectsCalculator_commands::inertia_q, "Inertia biquad q", CMDFLAG_GET | CMDFLAG_SET);
	registerCommand("filterProfile_id", EffectsCalculator_commands::filterProfileId, "Conditional effects filter profile: 0 default; 1 custom", CMDFLAG_GET | CMDFLAG_SET);

	registerCommand("frictionPctSpeedToRampup", EffectsCalculator_commands::frictionPctSpeedToRampup, "% of max speed for gradual increase", CMDFLAG_GET | CMDFLAG_SET);
}

EffectsCalculator::~EffectsCalculator()
{

}


bool EffectsCalculator::isActive()
{
	return effects_active;
}
void EffectsCalculator::setActive(bool active)
{
	effects_active = active;
	for (uint8_t i = 0; i < effects_stats.size(); i++)
	{
		effects_stats[i].current = {0}; // Reset active effect forces
		effects_statslast[i].current = {0};
	}
	setClipLed(active);
}

void EffectsCalculator::setPaused(bool paused)
{
	if (effects_paused_ == paused) return;
	const uint32_t now_ms = HAL_GetTick();
	if (paused) {
		effects_paused_ = true;
		for (FFB_Effect& effect : effects) {
			if (effect.state != EFFECT_STATE_INACTIVE) {
				effect.pauseStartTime = now_ms;
				effect.pauseClockPending = true;
			}
		}
		return;
	}

	for (FFB_Effect& effect : effects) {
		if (effect.state != EFFECT_STATE_INACTIVE && effect.pauseClockPending) {
			const uint32_t paused_ms = now_ms - effect.pauseStartTime;
			effect.startTime += paused_ms;
			effect.phaseLastTick += paused_ms;
			effect.pauseClockPending = false;
		}
	}
	effects_paused_ = false;
}

void EffectsCalculator::noteEffectStartedWhilePaused(FFB_Effect* effect, uint32_t start_tick)
{
	if (effects_paused_ && effect != nullptr) {
		effect->pauseStartTime = start_tick;
		effect->pauseClockPending = true;
	}
}

void EffectsCalculator::updateSamplerate(float newSamplerate){
	EffectsStateGuard state_lock(*this);
	this->calcfrequency = newSamplerate;
	for(FFB_Effect &effect : this->effects){
		if(effect.filter[0]){ // Update filters if effect has filters
			setFilters(&effect);
		}
	}
}


/*
If the metric is less than CP Offset - Dead Band, then the resulting force is given by the following formula:
		force = Negative Coefficient * (q - (CP Offset – Dead Band))
Similarly, if the metric is greater than CP Offset + Dead Band, then the resulting force is given by the
following formula:
		force = Positive Coefficient * (q - (CP Offset + Dead Band))
A spring condition uses axis position as the metric.
A damper condition uses axis velocity as the metric.
An inertia condition uses axis acceleration as the metric.

 */

/**
 * Calculates the resulting torque for FFB effects
 * Takes current position input scaled from -0x7fff to 0x7fff
 * Outputs a torque value from -0x7fff to 0x7fff (not yet clipped)
 */
void EffectsCalculator::calculateEffects(std::vector<std::unique_ptr<Axis>> &axes)
{
	for (auto &axis : axes) {
		axis->calculateAxisEffects(isActive());
	}

	// Use one coherent time snapshot for every effect in this output sample.
	const uint32_t now_ms = HAL_GetTick();
	auto advanceEffectClock = [now_ms](FFB_Effect *effect) {
		if (effect->state == EFFECT_STATE_INACTIVE) return false;

		if (effect->startDelayPending) {
			const uint32_t delay_elapsed_ms = now_ms - effect->startTime;
			if (delay_elapsed_ms < effect->pendingStartDelay) return false;
			effect->startTime += effect->pendingStartDelay;
			effect->startDelayPending = false;
			// Periodic phase begins at the scheduled activation tick, even if
			// actuator output was disabled while the delay elapsed.
			effect->phaseLastTick = effect->startTime;
		}

		const bool periodic = effect->type == FFB_EFFECT_SQUARE ||
			effect->type == FFB_EFFECT_TRIANGLE ||
			effect->type == FFB_EFFECT_SAWTOOTHUP ||
			effect->type == FFB_EFFECT_SAWTOOTHDOWN ||
			effect->type == FFB_EFFECT_SINE;
		if (periodic && effect->duration == FFB_EFFECT_DURATION_INFINITE) {
			const uint32_t period_ms = effect->period;
			const uint32_t elapsed_since_phase_tick_ms = now_ms - effect->phaseLastTick;
			effect->phaseLastTick = now_ms;
			const uint32_t phase_advance_ms = elapsed_since_phase_tick_ms % period_ms;
			const uint32_t phase_sum_ms = effect->phaseElapsedMs + phase_advance_ms;
			effect->phaseElapsedMs = (uint16_t)(phase_sum_ms >= period_ms
				? phase_sum_ms - period_ms
				: phase_sum_ms);
		}
		return true;
	};

	if(!isActive()){
		// Actuator disable allows effect time to continue, but PID Pause freezes
		// finite durations, start delays and periodic phase until Continue.
		if (!effects_paused_) {
			for (FFB_Effect &effect : effects) {
				advanceEffectClock(&effect);
			}
		}
		for (auto &axis : axes) {
			axis->setEffectTorque(0);
		}
		return;
	}

	float force = 0.0f;
	int axisCount = axes.size();
	float forces[MAX_AXIS] = {0.0f};
	const float globalGainScale = (float)global_gain / 255.0f;

	for (uint8_t i = 0; i < effects_stats.size(); i++)
	{
		effects_stats[i].current = {0}; // Reset active effect forces
	}

	for (uint8_t fxi = 0; fxi < MAX_EFFECTS; fxi++)
	{
		FFB_Effect *effect = &effects[fxi];

		if (!advanceEffectClock(effect)) continue;
		if (effect->state != EFFECT_STATE_INACTIVE) {
			const uint32_t elapsed_ms = now_ms - effect->startTime;
			// Finite effects expire after their duration; infinite effects do not.
			if (effect->duration != FFB_EFFECT_DURATION_INFINITE && effect->duration != 0) {
				if (elapsed_ms >= effect->duration)
				{
					const uint32_t completed_cycles = elapsed_ms / effect->duration;
					if (effect->loopsRemaining == FFB_EFFECT_LOOP_INFINITE) {
						// Rebase to the current iteration without accumulating sample jitter.
						effect->startTime += completed_cycles * effect->duration;
					} else if (effect->loopsRemaining > completed_cycles) {
						effect->loopsRemaining -= (uint8_t)completed_cycles;
						effect->startTime += completed_cycles * effect->duration;
					} else {
						effect->state = EFFECT_STATE_INACTIVE;
						for(uint8_t axis=0 ; axis < axisCount ; axis++)
							calcStatsEffectType(effect->type, 0,axis); // record a 0 on the ended force
					}
				}
			}
		}

		// Filter out inactive effects
		if (effect->state == EFFECT_STATE_INACTIVE)
		{
			continue;
		}

		force = calcNonConditionEffectForce(effect, now_ms);

		for(uint8_t axis=0 ; axis < axisCount ; axis++) // Calculate effects for all axes
		{
			float axisforce = calcComponentForce(effect, force, axes, axis);
			const float statsForceScaled = axisforce * globalGainScale;
			const int32_t statsForce = (int32_t)statsForceScaled;
			calcStatsEffectType(effect->type, statsForce,axis);
			forces[axis] += statsForceScaled; // Keep gain-scaled fractions until effects are summed.
		}
	}

	// Clip the globally scaled sum after all effects have accumulated.
	for(uint8_t i=0 ; i < axisCount ; i++)
	{
		int32_t force = (int32_t)clip<float, float>(forces[i], -32767.0f, 32767.0f);
		axes[i]->setEffectTorque(force);
	}

	effects_statslast = effects_stats;
}

/**
 * Calculates forces from a non conditional effect
 * Periodic and constant effects
 */
float EffectsCalculator::calcNonConditionEffectForce(FFB_Effect *effect, uint32_t now_ms) {
	float force_vector = 0.0f;
	float magnitude = (float)effect->magnitude;

	// If using an envelope modulate the magnitude based on time
	if(effect->useEnvelope){
		magnitude = (float)getEnvelopeMagnitude(effect, now_ms);
	}
	switch (effect->type){

	case FFB_EFFECT_CONSTANT:
	{ // Constant force is just the force
		force_vector = magnitude;
		break;
	}

	case FFB_EFFECT_RAMP:
	{
		const uint32_t elapsed_ms = now_ms - effect->startTime;
		const int32_t duration = effect->duration;
		if (duration <= 0) {
			force_vector = effect->endLevel;
		} else if (effect->useEnvelope) {
			const uint32_t duration_ms = (uint32_t)duration;
			uint32_t attack_time = std::min(effect->attackTime, duration_ms);
			uint32_t fade_time = std::min(effect->fadeTime, duration_ms);
			const uint64_t envelope_time_sum = (uint64_t)attack_time + fade_time;
			if (envelope_time_sum > duration_ms) {
				attack_time = (uint32_t)((uint64_t)attack_time * duration_ms / envelope_time_sum);
				fade_time = duration_ms - attack_time;
			}

			const uint32_t ramp_elapsed = std::min(elapsed_ms, duration_ms);
			const float ramp_start = (float)effect->startLevel;
			const float ramp_end = (float)effect->endLevel;
			auto ramp_level_at = [ramp_start, ramp_end, duration_ms](uint32_t time_ms) -> float {
				const float progress = (float)time_ms / (float)duration_ms;
				return ramp_start + (ramp_end - ramp_start) * progress;
			};

			if (attack_time != 0 && ramp_elapsed < attack_time) {
				const float attack_level = (float)std::min<uint16_t>(effect->attackLevel, 0x7FFF);
				const float attack_start = ramp_start < 0.0f ? -attack_level : attack_level;
				const float attack_end = ramp_level_at(attack_time);
				const float progress = (float)ramp_elapsed / (float)attack_time;
				force_vector = attack_start + (attack_end - attack_start) * progress;
			} else if (fade_time != 0 && ramp_elapsed >= duration_ms - fade_time) {
				const uint32_t fade_start_time = duration_ms - fade_time;
				const uint32_t fade_elapsed = ramp_elapsed - fade_start_time;
				const float fade_start = ramp_level_at(fade_start_time);
				const float fade_level = (float)std::min<uint16_t>(effect->fadeLevel, 0x7FFF);
				const float fade_end = ramp_end < 0.0f ? -fade_level : fade_level;
				const float progress = (float)fade_elapsed / (float)fade_time;
				force_vector = fade_start + (fade_end - fade_start) * progress;
			} else {
				force_vector = ramp_level_at(ramp_elapsed);
			}
		} else if (elapsed_ms >= (uint32_t)duration) {
			force_vector = effect->endLevel;
		} else {
			const float progress = (float)elapsed_ms / (float)duration;
			force_vector = (float)effect->startLevel +
				((float)effect->endLevel - (float)effect->startLevel) * progress;
		}
		break;
	}

	case FFB_EFFECT_SQUARE:
	case FFB_EFFECT_TRIANGLE:
	case FFB_EFFECT_SAWTOOTHUP:
	case FFB_EFFECT_SAWTOOTHDOWN:
	case FFB_EFFECT_SINE:
	{
		// Period defaults to 1 ms and HID updates clamp it to [1, 32767].
		uint32_t period_ms = effect->period;
		uint32_t period_us = period_ms * 1000;
		// Infinite effects accumulate phase modulo the period so a full HAL tick
		// rollover cannot reset their waveform. Finite loops still rebase startTime.
		uint32_t elapsed_phase_us;
		if (effect->duration == FFB_EFFECT_DURATION_INFINITE) {
			elapsed_phase_us = (uint32_t)effect->phaseElapsedMs * 1000u;
		} else {
			const uint32_t elapsed_ms = now_ms - effect->startTime;
			elapsed_phase_us = (elapsed_ms % period_ms) * 1000u;
		}
		const uint32_t phase_centidegrees = effect->phase;
		// 1000 / 36000 reduces exactly to 1 / 36. The HID period is capped
		// at 32767 ms, so the maximum product is 35999 * 32767 < UINT32_MAX.
		uint32_t phase_us = (phase_centidegrees * period_ms) / 36u;
		uint32_t cycle_pos_us = elapsed_phase_us + phase_us;
		// Both terms are already within one period, so at most one fold is needed.
		if (cycle_pos_us >= period_us) cycle_pos_us -= period_us;
		float phase_norm = (float)cycle_pos_us * effect->periodScale; // [0.0, 1.0), scaled when the HID period changes

		if (effect->type == FFB_EFFECT_SQUARE) {
			int32_t sqr = (phase_norm < 0.5f) ? -magnitude : magnitude;
			force_vector = effect->offset + sqr;
		} else if (effect->type == FFB_EFFECT_TRIANGLE) {
			float tri = (phase_norm < 0.5f) ? (4.0f * phase_norm - 1.0f) : (3.0f - 4.0f * phase_norm);
			force_vector = effect->offset + tri * magnitude;
		} else if (effect->type == FFB_EFFECT_SAWTOOTHUP) {
			float saw = 2.0f * phase_norm - 1.0f;
			force_vector = effect->offset + saw * magnitude;
		} else if (effect->type == FFB_EFFECT_SAWTOOTHDOWN) {
			float saw = 1.0f - 2.0f * phase_norm;
			force_vector = effect->offset + saw * magnitude;
		} else if (effect->type == FFB_EFFECT_SINE) {
			// phase_norm is in [0, 1); table interpolation avoids libm range
			// reduction in this 1 kHz path with < 1 torque count of peak error.
			float sine = our_arm_sin_f32(2.0f * (float)M_PI * phase_norm) * magnitude;
			force_vector = effect->offset + sine;
		}
		break;
	}
	default:
		return 0;
		break;
	}

	return force_vector * ((float)effect->gain / 255.0f);
}



/**
 * Calculates the force of an effect
 */

float EffectsCalculator::calcComponentForce(FFB_Effect *effect, float forceVector, std::vector<std::unique_ptr<Axis>> &axes, uint8_t axis)
{
	float result_torque = 0.0f;
	uint8_t con_idx = effect->useSingleCondition? 0 : axis; // condition block index

	metric_t *metrics = axes[axis]->getMetrics();

	float angle_ratio = effect->axisMagnitudes[axis];
	// HID Set Effect Gain scales every effect's magnitude, including condition
	// effects. Waveform effects receive it in calcNonConditionEffectForce();
	// apply the same normalized gain here to spring/damper/inertia/friction.
	const float effectGainScale = (float)effect->gain / 255.0f;

	switch (effect->type)
	{
	case FFB_EFFECT_CONSTANT:
	{
		// Optional filtering to reduce spikes
		if(effect->filter[axis] != nullptr) {
			// if the filter is enabled we apply it
			if (effect->filter[axis]->getFc() < 0.5 && effect->filter[0]->getFc() != 0.0)
			{
				forceVector = effect->filter[axis]->process(forceVector);
			}
		}
	}
	// No break required here. The filter is a special preprocessing case for the constant force effect.
	case FFB_EFFECT_RAMP:
	case FFB_EFFECT_SQUARE:
	case FFB_EFFECT_TRIANGLE:
	case FFB_EFFECT_SAWTOOTHUP:
	case FFB_EFFECT_SAWTOOTHDOWN:
	case FFB_EFFECT_SINE:
	{
		result_torque = -forceVector * angle_ratio;
		break;
	}

	case FFB_EFFECT_SPRING:
	{
		float pos = metrics->pos_scaled_16b;
		result_torque -= effectGainScale * calcConditionEffectForce(effect, pos, gain.spring, con_idx, scaler.spring, angle_ratio);
		break;
	}


	/** 	      |	  (rampup is from 0..5% of max velocity)
	 * 			  |	  __________ (after use max coefficient)
	 * 			  |	 /
	 *			  |	/
	 *			  |-
	 * ------------------------  Velocity
	 * 			 -|
	 *			/ |
	 * 		   /  |
	 * 	-------   |
	 * 			  |
	 */
	case FFB_EFFECT_FRICTION: // TODO sometimes unstable.
	{
		float speed = metrics->speed * INTERNAL_SCALER_FRICTION;

		int16_t offset = effect->conditions[con_idx].cpOffset;
		int16_t deadBand = effect->conditions[con_idx].deadBand;
		float force = 0.0f;

		float speedRampupCeil = speedRampupPct();

		// Effect is only active outside deadband + offset
		if (fabsf(speed - (float)offset) > (float)deadBand){

			// remove offset/deadband from metric to compute force
			speed -= (offset + (deadBand * (speed < offset ? -1 : 1)) );

			// check if speed is in the 0..x% to rampup, if is this range, apply a sinusoidale function to smooth the torque (slow near 0, slow around the X% rampup
			float rampupFactor = 1.0;
			if (fabs (speed) < speedRampupCeil) {								// if speed in the range to rampup we apply a sinus curbe to ramup

				float phaseRad = M_PI * ((fabs (speed) / speedRampupCeil) - 0.5);// we start to compute the normalized angle (speed / normalizedSpeed@5%) and translate it of -1/2PI to translate sin on 1/2 periode
					rampupFactor = (1.0f + our_arm_sin_f32(phaseRad)) / 2.0f;				// Half-sine ramp from 0 to 1.

			}

			int8_t sign = speed >= 0 ? 1 : -1;
			int32_t coeff = speed < 0
				? (int32_t)effect->conditions[con_idx].negativeCoefficient
				: (int32_t)effect->conditions[con_idx].positiveCoefficient;
			force = (float)coeff * rampupFactor * (float)sign;

			//if there is a saturation, used it to clip result
			if (effect->conditions[con_idx].negativeSaturation !=0 || effect->conditions[con_idx].positiveSaturation !=0) {
				force = clip<float, float>(force,
					-(float)effect->conditions[con_idx].negativeSaturation,
					(float)effect->conditions[con_idx].positiveSaturation);
			}

			const float frictionGain = gain.friction == 0 ? 0.0f : (float)(gain.friction + 1) / 256.0f;
			float force_raw = frictionGain * force * angle_ratio * scaler.friction;
			if (effect->filter[axis] != nullptr) {
				result_torque -= effectGainScale * effect->filter[axis]->process(force_raw);
			} else {
				result_torque -= effectGainScale * force_raw;
			}
		}

		break;
	}
	case FFB_EFFECT_DAMPER:
	{

		float speed = metrics->speed * INTERNAL_SCALER_DAMPER;
		float force_raw = calcConditionEffectForce(effect, speed, gain.damper, con_idx, scaler.damper, angle_ratio);
		if (effect->filter[axis] != nullptr) {
			result_torque -= effectGainScale * effect->filter[axis]->process(force_raw);
		} else {
			result_torque -= effectGainScale * force_raw;
		}

		break;
	}

	case FFB_EFFECT_INERTIA:
	{
		float accel = metrics->accel * INTERNAL_SCALER_INERTIA;
		float force_raw = calcConditionEffectForce(effect, accel, gain.inertia, con_idx, scaler.inertia, angle_ratio); // Bump *60 the inertia feedback
		if (effect->filter[axis] != nullptr) {
			result_torque -= effectGainScale * effect->filter[axis]->process(force_raw);
		} else {
			result_torque -= effectGainScale * force_raw;
		}

		break;
	}

	default:
		// Unsupported effect
		break;
	}
	return result_torque;
}

float EffectsCalculator::speedRampupPct() {
	return (frictionPctSpeedToRampup / 100.0) * 32767;	// compute the normalizedSpeed of pctToRampup factor
}


/**
 * Calculates a conditional effect
 * Takes care of deadband and offsets and scalers
 * Gain of 255 = 1x. Prescale with scale factor
 */
float EffectsCalculator::calcConditionEffectForce(FFB_Effect *effect, float  metric, uint8_t gain,
										 uint8_t idx, float scale, float angle_ratio)
{
	int16_t offset = effect->conditions[idx].cpOffset;
	int16_t deadBand = effect->conditions[idx].deadBand;
	float force = 0.0f;
	float gainfactor = gain == 0 ? 0.0f : (float)(gain + 1) / 256.0f;

	// Effect is only active outside deadband + offset
	if (fabsf(metric - (float)offset) > (float)deadBand){
		float coefficient = effect->conditions[idx].negativeCoefficient;
		if(metric > offset){
			coefficient = effect->conditions[idx].positiveCoefficient;
		}
		coefficient /= 0x7fff; // rescale the coefficient of effect

		// remove offset/deadband from metric to compute force
		metric = metric - (offset + (deadBand * (metric < offset ? -1 : 1)) );

		int32_t posSat = effect->conditions[idx].positiveSaturation;
		int32_t negSat = effect->conditions[idx].negativeSaturation;
		if (posSat == 0 && effect->conditions[idx].positiveCoefficient != 0) posSat = 0x7FFF;
		if (negSat == 0 && effect->conditions[idx].negativeCoefficient != 0) negSat = 0x7FFF;

		force = clip<float, float>((coefficient * gainfactor * scale * metric),
										-(float)negSat, (float)posSat);
	}


	return force * angle_ratio;
}

/**
 * Modulates the magnitude of an effect based on time and attack/fade levels
 * During attack time the strength changes from the initial attack level to the normal magnitude which is sustained
 * until the fade time where the strength changes to the fade level until the stop time of the effect.
 * Infinite effects can't have an envelope and return the normal magnitude.
 */
float EffectsCalculator::getEnvelopeMagnitude(FFB_Effect *effect, uint32_t now_ms)
{
	if(effect->duration == FFB_EFFECT_DURATION_INFINITE || effect->duration == 0){
		return effect->magnitude; // Effect is infinite. envelope is invalid
	}
	const uint32_t duration = effect->duration;
	uint32_t attack_time = std::min(effect->attackTime, duration);
	uint32_t fade_time = std::min(effect->fadeTime, duration);
	const uint64_t envelope_time_sum = (uint64_t)attack_time + fade_time;
	// Keep attack and fade segments from overlapping when a host supplies
	// timings whose sum exceeds the finite effect duration.
	if (envelope_time_sum > duration) {
		attack_time = (uint32_t)((uint64_t)attack_time * duration / envelope_time_sum);
		fade_time = duration - attack_time;
	}

	const uint32_t elapsed_raw = now_ms - effect->startTime;
	const uint32_t elapsed_time = std::min(elapsed_raw, duration);
	const float magnitude = fabsf((float)effect->magnitude);
	const float attack_level = (float)std::min<uint16_t>(effect->attackLevel, 0x7FFF);
	const float fade_level = (float)std::min<uint16_t>(effect->fadeLevel, 0x7FFF);
	float scaler = magnitude;
	if (attack_time != 0 && elapsed_time < attack_time) {
		scaler = attack_level + (magnitude - attack_level) *
			((float)elapsed_time / (float)attack_time);
	}
	if (fade_time != 0 && elapsed_time >= duration - fade_time) {
		const uint32_t remaining = duration - elapsed_time;
		scaler = fade_level + (scaler - fade_level) *
			((float)remaining / (float)fade_time);
	}
	if (effect->magnitude < 0) scaler = -scaler;
	return scaler;
}

void EffectsCalculator::setFilters(FFB_Effect *effect){

	std::function<void(std::unique_ptr<Biquad> &)> fnptr = [=](std::unique_ptr<Biquad> &filter){};

	switch (effect->type)
	{
	case FFB_EFFECT_DAMPER:
		fnptr = [=, this](std::unique_ptr<Biquad> &filter){
			if (filter != nullptr)
				filter->setBiquad(BiquadType::lowpass, this->filter[filterProfileId].damper.freq/ (float)calcfrequency, this->filter[filterProfileId].damper.q * qfloatScaler , (float)0.0);
			else
				filter = std::make_unique<Biquad>(BiquadType::lowpass, this->filter[filterProfileId].damper.freq / (float)calcfrequency, this->filter[filterProfileId].damper.q * qfloatScaler, (float)0.0);
		};
		break;
	case FFB_EFFECT_FRICTION:
		fnptr = [=, this](std::unique_ptr<Biquad> &filter){
			if (filter != nullptr)
				filter->setBiquad(BiquadType::lowpass, this->filter[filterProfileId].friction.freq / (float)calcfrequency, this->filter[filterProfileId].friction.q * qfloatScaler, (float)0.0);
			else
				filter = std::make_unique<Biquad>(BiquadType::lowpass, this->filter[filterProfileId].friction.freq / (float)calcfrequency, this->filter[filterProfileId].friction.q * qfloatScaler, (float)0.0);
		};
		break;
	case FFB_EFFECT_INERTIA:
		fnptr = [=, this](std::unique_ptr<Biquad> &filter){
			if (filter != nullptr)
				filter->setBiquad(BiquadType::lowpass, this->filter[filterProfileId].inertia.freq / (float)calcfrequency, this->filter[filterProfileId].inertia.q * qfloatScaler, (float)0.0);
			else
				filter = std::make_unique<Biquad>(BiquadType::lowpass, this->filter[filterProfileId].inertia.freq / (float)calcfrequency, this->filter[filterProfileId].inertia.q * qfloatScaler, (float)0.0);
		};
		break;
	case FFB_EFFECT_CONSTANT:
		fnptr = [=, this](std::unique_ptr<Biquad> &filter){
			if (filter != nullptr)
				filter->setBiquad(BiquadType::lowpass, this->filter[0].constant.freq / (float)calcfrequency, this->filter[0].constant.q * qfloatScaler, (float)0.0);
			else
				filter = std::make_unique<Biquad>(BiquadType::lowpass, this->filter[0].constant.freq / (float)calcfrequency, this->filter[0].constant.q * qfloatScaler, (float)0.0);
		};
		break;
	}


	for (int i=0; i<MAX_AXIS; i++) {
		fnptr(effect->filter[i]);
	}
}


void EffectsCalculator::setGain(uint8_t gain)
{
	global_gain = gain;
}

uint8_t EffectsCalculator::getGain() { return global_gain; }



/*
 * Read parameters from flash and restore settings
 */
void EffectsCalculator::restoreFlash()
{
	uint16_t filterStorage;
	if (Flash_Read(ADR_FFB_CF_FILTER, &filterStorage))
	{
		uint32_t freq = filterStorage & 0x1FF;
		uint8_t q = (filterStorage >> 9) & 0x7F;
		checkFilterCoeff(&(this->filter[0].constant), freq, q);
		updateFilterSettingsForEffects(FFB_EFFECT_CONSTANT);
	}

	if (Flash_Read(ADR_FFB_FR_FILTER, &filterStorage))
	{
		uint32_t freq = filterStorage & 0x1FF;
		uint8_t q = (filterStorage >> 9) & 0x7F;
		checkFilterCoeff(&(this->filter[CUSTOM_PROFILE_ID].friction), freq, q);
		updateFilterSettingsForEffects(FFB_EFFECT_FRICTION);
	}

	if (Flash_Read(ADR_FFB_DA_FILTER, &filterStorage))
	{
		uint32_t freq = filterStorage & 0x1FF;
		uint8_t q = (filterStorage >> 9) & 0x7F;
		checkFilterCoeff(&(this->filter[CUSTOM_PROFILE_ID].damper), freq, q);
		updateFilterSettingsForEffects(FFB_EFFECT_DAMPER);
	}

	if (Flash_Read(ADR_FFB_IN_FILTER, &filterStorage))
	{
		uint32_t freq = filterStorage & 0x1FF;
		uint8_t q = (filterStorage >> 9) & 0x7F;
		checkFilterCoeff(&(this->filter[CUSTOM_PROFILE_ID].inertia), freq, q);
		updateFilterSettingsForEffects(FFB_EFFECT_INERTIA);
	}

	uint16_t effects = 0;
	if(Flash_Read(ADR_FFB_EFFECTS1, &effects)){
		gain.friction = (effects >> 8) & 0xff;
		gain.inertia = (effects & 0xff);
	}
	if(Flash_Read(ADR_FFB_EFFECTS2, &effects)){
		gain.damper = (effects >> 8) & 0xff;
		gain.spring = (effects & 0xff);
	}
	if(Flash_Read(ADR_FFB_EFFECTS3, &effects)){
		filterProfileId = clip<uint8_t, uint8_t>(
			(uint8_t)((effects >> 8) & 0x03), 0, CUSTOM_PROFILE_ID);
		frictionPctSpeedToRampup = clip<uint8_t, uint8_t>(effects & 0xff, 0, 100);
	}

}

// Saves parameters to flash
void EffectsCalculator::saveFlash(uint8_t* globalGainSnapshot)
{
	effect_gain_t gainSnapshot;
	effect_biquad_t filterSnapshot[2];
	uint8_t filterProfileSnapshot;
	uint8_t frictionRampupSnapshot;
	{
		EffectsStateGuard state_lock(*this);
		gainSnapshot = gain;
		filterSnapshot[0] = filter[0];
		filterSnapshot[1] = filter[CUSTOM_PROFILE_ID];
		filterProfileSnapshot = filterProfileId;
		frictionRampupSnapshot = frictionPctSpeedToRampup;
		if (globalGainSnapshot) *globalGainSnapshot = global_gain;
	}

	// Flash programming starts after the effect state snapshot is unlocked.
	uint16_t filterStorage;

	// save CF biquad
	filterStorage = (uint16_t)filterSnapshot[0].constant.freq & 0x1FF;
	filterStorage |= ( (uint16_t)filterSnapshot[0].constant.q & 0x7F ) << 9 ;
	Flash_Write(ADR_FFB_CF_FILTER, filterStorage);

	if(filterProfileSnapshot == CUSTOM_PROFILE_ID){ // Only attempt saving if custom profile active
		// save Friction biquad
		filterStorage = (uint16_t)filterSnapshot[CUSTOM_PROFILE_ID].friction.freq & 0x1FF;
		filterStorage |= ( (uint16_t)filterSnapshot[CUSTOM_PROFILE_ID].friction.q & 0x7F ) << 9 ;
		Flash_Write(ADR_FFB_FR_FILTER, filterStorage);

		// save Damper biquad
		filterStorage = (uint16_t)filterSnapshot[CUSTOM_PROFILE_ID].damper.freq & 0x1FF;
		filterStorage |= ( (uint16_t)filterSnapshot[CUSTOM_PROFILE_ID].damper.q & 0x7F ) << 9 ;
		Flash_Write(ADR_FFB_DA_FILTER, filterStorage);

		// save Inertia biquad
		filterStorage = (uint16_t)filterSnapshot[CUSTOM_PROFILE_ID].inertia.freq & 0x1FF;
		filterStorage |= ( (uint16_t)filterSnapshot[CUSTOM_PROFILE_ID].inertia.q & 0x7F ) << 9 ;
		Flash_Write(ADR_FFB_IN_FILTER, filterStorage);
	}

	// save the effect gain
	uint16_t effects = gainSnapshot.inertia | (gainSnapshot.friction << 8);
	Flash_Write(ADR_FFB_EFFECTS1, effects);

	effects = gainSnapshot.spring | (gainSnapshot.damper << 8);
	Flash_Write(ADR_FFB_EFFECTS2, effects);

	// save the friction rampup zone
	effects = frictionRampupSnapshot | (filterProfileSnapshot << 8);
	Flash_Write(ADR_FFB_EFFECTS3, effects);

}

void EffectsCalculator::checkFilterCoeff(biquad_constant_t *filter, uint32_t freq,uint8_t q)
{
	if(q == 0) {
		q = 1;
	}

	if(freq == 0){
		freq = calcfrequency / 2;
	}

	filter->freq = clip<uint32_t, uint32_t>(freq, 1, (calcfrequency / 2));
	filter->q = clip<uint8_t, uint8_t>(q,0,127);
}

void EffectsCalculator::updateFilterSettingsForEffects(uint8_t type_effect) {

	// loop on all effect in memory and setup new constant filter
	for (uint8_t i = 0; i < MAX_EFFECTS; i++)
	{
		if (effects[i].type == type_effect)
		{
			setFilters(&effects[i]);
		}
	}
}


void EffectsCalculator::logEffectType(uint8_t type,bool remove){
	if(type > 0 && type < 32){

		if(remove){
			if(effects_stats[type-1].nb > 0)
				effects_stats[type-1].nb--;

			if(!effects_stats[type-1].nb){
				effects_stats[type-1].current = {0};
			}
		}else{
			effects_used |= 1<<(type-1);
			if( effects_stats[type-1].nb < 65535 ) {
				effects_stats[type-1].nb ++;
			}
		}
	}
}

void EffectsCalculator::logEffectState(uint8_t type,uint8_t state){
	if(type > 0 && type < 32){
		if(!state){
			effects_stats[type-1].current = {0};
		}
	}
}


void EffectsCalculator::calcStatsEffectType(uint8_t type, int32_t force,uint8_t axis){
	if(axis >= MAX_AXIS)
		return;
	if(type > 0 && type < 13) {
		uint8_t arrayLocation = type - 1;
		effects_stats[arrayLocation].current[axis] = clip<int32_t,int32_t>(effects_stats[arrayLocation].current[axis] + force, -0x7fff, 0x7fff);
		effects_stats[arrayLocation].max[axis] = std::max(effects_stats[arrayLocation].max[axis], (int16_t)abs(force));
	}
}

/**
 * Prints a list of effects that were active at some point
 * Does not reset when an effect is deactivated
 * Axis only used in detail mode
 */
std::string EffectsCalculator::listEffectsUsed(bool details,uint8_t axis){
	std::string effects_list = "";
	if(axis >= MAX_AXIS)
		return "";

	if (!details) {
		if(effects_used == 0){
			return "None";
		}

		static const char *effects[12] = {"Constant,","Ramp,","Square,","Sine,","Triangle,","Sawtooth Up,","Sawtooth Down,","Spring,","Damper,","Inertia,","Friction,","Custom,"};

		for (int i=0;i < 12; i++) {
			if((effects_used >> i) & 1) {
				effects_list += effects[i];
			}
		}

		effects_list.pop_back();
	} else {

		bool firstItem = true;
		for (int i=0;i < 12; i++) {
			if (!firstItem) effects_list += ", ";
			effects_list += "{\"max\":" + std::to_string(effects_stats[i].max[axis]);
			effects_list += ", \"curr\":" + std::to_string(effects_stats[i].current[axis]);
			effects_list += ", \"nb\":" + std::to_string(effects_stats[i].nb) + "}";
			firstItem = false;
		}

	}

	return effects_list.c_str();
}

/**
 * Resets the effects_used flags
 * If reinit is true it will set the flag again if the currently active effect number is not 0
 */
void EffectsCalculator::resetLoggedActiveEffects(bool reinit){
	effects_used = 0;
	if(reinit){
		for (int i=0;i < 12; i++) {
			if(effects_stats[i].nb > 0) {
				effects_used |= 1<<(i);
			}
		}
	}
}

CommandStatus EffectsCalculator::command(const ParsedCommand& cmd,std::vector<CommandReply>& replies){
	EffectsStateGuard state_lock(*this);
	switch(static_cast<EffectsCalculator_commands>(cmd.cmdId)){

	case EffectsCalculator_commands::ffbfiltercf:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(filter[0].constant.freq);
		}
		else if (cmd.type == CMDtype::set)
		{
			checkFilterCoeff(&filter[0].constant, cmd.val, filter[0].constant.q);
			updateFilterSettingsForEffects(FFB_EFFECT_CONSTANT);
		}
		break;
	case EffectsCalculator_commands::ffbfiltercf_q:
		if(cmd.type == CMDtype::info){
			replies.emplace_back("scale:"+std::to_string(this->qfloatScaler));
		}
		else if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(filter[0].constant.q);
		}
		else if (cmd.type == CMDtype::set)
		{
			checkFilterCoeff(&filter[0].constant, filter[0].constant.freq, cmd.val);
			updateFilterSettingsForEffects(FFB_EFFECT_CONSTANT);
		}
		break;
	case EffectsCalculator_commands::effects:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(effects_used);
		}
		else if (cmd.type == CMDtype::set)
		{
			resetLoggedActiveEffects(cmd.val == 0);
		}
		else if (cmd.type == CMDtype::info)
		{
			replies.emplace_back(listEffectsUsed(false));
		}
		break;
	case EffectsCalculator_commands::effectsDetails:
		if (cmd.type == CMDtype::get || cmd.type == CMDtype::getat)
		{
			replies.emplace_back(listEffectsUsed(true,cmd.adr));
		}
		else if (cmd.type == CMDtype::set && cmd.val >= 0)
		{
			for (int i=0; i<12; i++) {
				effects_stats[i].max = {0};
				if(cmd.val > 0){
					effects_stats[i].current = {0};
					effects_stats[i].nb = 0;
				}
			}
			resetLoggedActiveEffects(true);
		}
		break;
	case EffectsCalculator_commands::effectsForces:
	{
		uint8_t axis = 0;
		if(cmd.type == CMDtype::getat){
			axis = std::min<uint8_t>(cmd.adr,MAX_AXIS);
		}
		if (cmd.type == CMDtype::get || cmd.type == CMDtype::getat)
		{
			for (size_t i=0; i < effects_statslast.size(); i++) {
				replies.emplace_back(effects_statslast[i].current[axis],effects_statslast[i].nb);
			}
		}
		break;
	}
	case EffectsCalculator_commands::spring:
		if(cmd.type == CMDtype::info){
			replies.emplace_back("scale:"+std::to_string(this->scaler.spring));
		}else
			return handleGetSet(cmd, replies, this->gain.spring);
		break;
	case EffectsCalculator_commands::friction:
		if(cmd.type == CMDtype::info){
			replies.emplace_back("scale:"+std::to_string(this->scaler.friction)+",factor:"+std::to_string(INTERNAL_SCALER_FRICTION));
		}else
			return handleGetSet(cmd, replies, this->gain.friction);
		break;
	case EffectsCalculator_commands::damper:
		if(cmd.type == CMDtype::info){
			replies.emplace_back("scale:"+std::to_string(this->scaler.damper)+",factor:"+std::to_string(INTERNAL_SCALER_DAMPER));
		}else
			return handleGetSet(cmd, replies, this->gain.damper);
		break;
	case EffectsCalculator_commands::inertia:
		if(cmd.type == CMDtype::info){
			replies.emplace_back("scale:"+std::to_string(this->scaler.inertia)+",factor:"+std::to_string(INTERNAL_SCALER_INERTIA));
		}else
			return handleGetSet(cmd, replies, this->gain.inertia);
		break;
	case EffectsCalculator_commands::damper_f:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(filter[filterProfileId].damper.freq);
		}
		else if (cmd.type == CMDtype::set)
		{
			checkFilterCoeff(&filter[CUSTOM_PROFILE_ID].damper, cmd.val, filter[CUSTOM_PROFILE_ID].damper.q);
			if (filterProfileId == CUSTOM_PROFILE_ID) updateFilterSettingsForEffects(FFB_EFFECT_DAMPER);
		}
		break;
	case EffectsCalculator_commands::damper_q:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(filter[filterProfileId].damper.q);
		}
		else if (cmd.type == CMDtype::set)
		{
			checkFilterCoeff(&filter[CUSTOM_PROFILE_ID].damper, filter[CUSTOM_PROFILE_ID].damper.freq, cmd.val);
			if (filterProfileId == CUSTOM_PROFILE_ID) updateFilterSettingsForEffects(FFB_EFFECT_DAMPER);
		}
		break;
	case EffectsCalculator_commands::friction_f:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(filter[filterProfileId].friction.freq);
		}
		else if (cmd.type == CMDtype::set)
		{
			checkFilterCoeff(&filter[CUSTOM_PROFILE_ID].friction, cmd.val, filter[CUSTOM_PROFILE_ID].friction.q);
			if (filterProfileId == CUSTOM_PROFILE_ID) updateFilterSettingsForEffects(FFB_EFFECT_FRICTION);
		}
		break;
	case EffectsCalculator_commands::friction_q:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(filter[filterProfileId].friction.q);
		}
		else if (cmd.type == CMDtype::set)
		{
			checkFilterCoeff(&filter[CUSTOM_PROFILE_ID].friction, filter[CUSTOM_PROFILE_ID].friction.freq, cmd.val);
			if (filterProfileId == CUSTOM_PROFILE_ID) updateFilterSettingsForEffects(FFB_EFFECT_FRICTION);
		}
		break;
	case EffectsCalculator_commands::inertia_f:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(filter[filterProfileId].inertia.freq);
		}
		else if (cmd.type == CMDtype::set)
		{
			checkFilterCoeff(&filter[CUSTOM_PROFILE_ID].inertia, cmd.val, filter[CUSTOM_PROFILE_ID].inertia.q);
			if (filterProfileId == CUSTOM_PROFILE_ID) updateFilterSettingsForEffects(FFB_EFFECT_INERTIA);
		}
		break;
	case EffectsCalculator_commands::inertia_q:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(filter[filterProfileId].inertia.q);
		}
		else if (cmd.type == CMDtype::set)
		{
			checkFilterCoeff(&filter[CUSTOM_PROFILE_ID].inertia, filter[CUSTOM_PROFILE_ID].inertia.freq, cmd.val);
			if (filterProfileId == CUSTOM_PROFILE_ID) updateFilterSettingsForEffects(FFB_EFFECT_INERTIA);
		}
		break;

	case EffectsCalculator_commands::frictionPctSpeedToRampup:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(frictionPctSpeedToRampup);
		}
		else if (cmd.type == CMDtype::set)
		{
			uint8_t pct = clip<uint8_t, uint8_t>(cmd.val, 0, 100);
			frictionPctSpeedToRampup = pct;
		}
		break;
	case EffectsCalculator_commands::filterProfileId:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(this->filterProfileId);
		}
		else if (cmd.type == CMDtype::set)
		{
			uint32_t value = clip<uint32_t, uint32_t>(cmd.val, 0, 1);
			this->filterProfileId = value;
			updateFilterSettingsForEffects(FFB_EFFECT_INERTIA);
			updateFilterSettingsForEffects(FFB_EFFECT_DAMPER);
			updateFilterSettingsForEffects(FFB_EFFECT_FRICTION);
		}
		break;
	case EffectsCalculator_commands::monitorEffect:
		if (cmd.type == CMDtype::get)
		{
			replies.emplace_back(isMonitorEffect);
		}
		else if (cmd.type == CMDtype::set)
		{
			isMonitorEffect = clip<uint8_t, uint8_t>(cmd.val, 0, 1);
		}
		break;

	default:
		return CommandStatus::NOT_FOUND;
	}
	return CommandStatus::OK;
}

/*
 *
 */
void EffectsCalculator::Run() {
	Delay(500);
	while (true) {
		Delay(3000);
	}
}

/**
 * Resets an effect and marks the effect as free
 */
void EffectsCalculator::free_effect(uint16_t idx){
	if(idx < this->effects.size()){
		logEffectType(effects[idx].type, true); // Effect off
		effects[idx] = FFB_Effect(); // Reset all settings
		for(int i=0; i< MAX_AXIS; i++) {
			if(effects[idx].filter[i] != nullptr){
				effects[idx].filter[i].reset(nullptr);
			}
		}
	}
}

/**
 * Will return the first effect index which is empty or -1 if none found
 */
int32_t EffectsCalculator::find_free_effect(uint8_t type){
	if(type > FFB_EFFECT_NONE && type < FFB_EFFECT_CUSTOM+1){ // Check if it is a valid effect type
		for(uint8_t i=0;i<effects.size();i++){
			if(effects[i].type == FFB_EFFECT_NONE){
				return(i);
			}
		}
	}
	return -1;
}




/**
 * Calculates the frequency of hid out reports
 */
uint32_t EffectsControlItf::getRate(){
	float periodAvg = fxPeriodAvg.getAverage();
	if((micros() - lastFxUpdate) > 1000000 || periodAvg == 0){
		// Reset average
		fxPeriodAvg.clear();
		return 0;
	}else{
		return (1000000.0/periodAvg);
	}
}

/**
 * Calculates the frequency of the CF effect only
 */
uint32_t EffectsControlItf::getConstantForceRate(){
	float periodAvg = cfUpdatePeriodAvg.getAverage();
	if((micros() - lastCfUpdate) > 1000000 || periodAvg == 0){
		// Reset average
		cfUpdatePeriodAvg.clear();
		return 0;
	}else{
		return (1000000.0/periodAvg);
	}
}


void EffectsControlItf::cfUpdateEvent(){
	cfUpdatePeriodAvg.addValue((uint32_t)(micros() - lastCfUpdate));
	lastCfUpdate = micros();
}

void EffectsControlItf::fxUpdateEvent(){
	fxPeriodAvg.addValue((uint32_t)(micros() - lastFxUpdate));
	lastFxUpdate = micros();
}
