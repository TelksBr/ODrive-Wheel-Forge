/*
 * HidFFB.cpp
 *
 *  Created on: 12.02.2020
 *      Author: Yannick
 *
 * ────── ODrive-FFB-Wheel fork ──────────────────────────────────────────────
 * Este arquivo é uma cópia forkada do OpenFFBoard upstream.
 *   Base: OpenFFBoard v1.17.0 (commit ac68fba)
 *   Localização upstream: Firmware/FFBoard/Src/HidFFB.cpp
 *
 * Modificações locais:
 *   - set_effect: fallback pra single-axis wheel quando axisMagnitudes[0]=0
 *     e axisMagnitudes[1]=0 simultaneamente (ex: directionX=0 polar). Usa
 *     cos(phaseX) ou 1.0 como default — força que jogos com direção north
 *     gerem torque no eixo X do volante.
 *
 * Pra atualizar contra upstream:
 *   diff Odrive-Wheel/src/HidFFB.cpp \
 *        OpenFFBoard-master/OpenFFBoard-master/Firmware/FFBoard/Src/HidFFB.cpp
 * ───────────────────────────────────────────────────────────────────────────
 */

#include <assert.h>
#include "HidFFB.h"
#include "flash_helpers.h"
#include "hid_device.h"
#include "cppmain.h"
#include <math.h>

template <typename T>
static bool copy_hid_report(const uint8_t* buffer, uint16_t size, T& report) {
	if (buffer == nullptr || size < sizeof(T)) {
		return false;
	}
	memcpy(&report, buffer, sizeof(T));
	return true;
}

template <typename T>
static bool copy_hid_report_with_id(uint8_t report_id, const uint8_t* buffer,
		uint16_t size, T& report) {
	if (buffer == nullptr || size < sizeof(T) - 1) return false;
	auto* bytes = reinterpret_cast<uint8_t*>(&report);
	bytes[0] = report_id;
	memcpy(bytes + 1, buffer, sizeof(T) - 1);
	return true;
}

HidFFB::HidFFB(std::shared_ptr<EffectsCalculator> ec,uint8_t axisCount) : effects_calc(ec), effects(ec->effects),axisCount(axisCount)
{
	// O descriptor anuncia sempre X e Y (MAX_AXIS=2), mesmo que o produto só
	// tenha um eixo físico. Portanto o bit de direção é fixo no bit 2 (0x04),
	// não pode ser inferido do número de eixos físicos.
	directionEnableMask = HID_DIRECTION_ENABLE;
	// Initialize reports
	blockLoad_report.effectBlockIndex = 1;
	blockLoad_report.ramPoolAvailable = (effects.size()-used_effects)*sizeof(FFB_Effect);
	blockLoad_report.loadStatus = 1;

	pool_report.ramPoolSize = effects.size()*sizeof(FFB_Effect);
	pool_report.maxSimultaneousEffects = effects.size();
	pool_report.memoryManagement = 1;


	this->registerHidCallback();
}

HidFFB::~HidFFB() {
}


/**
 * Sets the mask where the direction enable bit is in the effect
 */
void HidFFB::setDirectionEnableMask(uint8_t mask){
	this->directionEnableMask = mask;
}

void HidFFB::updateSamplerate(float newSamplerate){
	effects_calc->updateSamplerate(newSamplerate);
}


bool HidFFB::getFfbActive(){
	return this->ffb_active;
}

bool HidFFB::HID_SendReport(uint8_t *report,uint16_t len){
	return tud_hid_report(0, report, len); // ID 0 skips ID field
}


/**
 * Sends a status report for a specific effect
 */
void HidFFB::sendStatusReport(uint8_t effect){
	this->reportFFBStatus.status = HID_ACTUATOR_POWER;
	if(this->ffb_active){
		this->reportFFBStatus.status |= HID_ENABLE_ACTUATORS;
		this->reportFFBStatus.status |= HID_EFFECT_PLAYING;
	}else{
		this->reportFFBStatus.status |= HID_EFFECT_PAUSE;
	}
	HID_SendReport(reinterpret_cast<uint8_t*>(&this->reportFFBStatus), sizeof(reportFFB_status_t));
}


/**
 * Called when HID OUT data is received via USB
 */
void HidFFB::hidOut(uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize){
	(void)report_type;
	if (buffer == nullptr) return;
	EffectsStateGuard state_lock(*effects_calc);
	fxUpdateEvent(); // use uint16_t for timer overflow handling if micros timer is used

	// FFB Output Message
	const uint8_t* report = buffer;
	uint8_t event_idx = report_id - FFB_ID_OFFSET;

	// -------- Out Reports --------
	switch(event_idx)
	{

		case HID_ID_NEWEFREP: //add Effect Report. Feature
		{
			FFB_CreateNewEffect_Feature_Data_t data{};
			if (copy_hid_report(report, bufsize, data)) new_effect(&data);
			break;
		}
		case HID_ID_EFFREP: // Set Effect
		{
			FFB_SetEffect_t data{};
			if (copy_hid_report_with_id(report_id, report, bufsize, data)) set_effect(&data);
			break;
		}
		case HID_ID_CTRLREP: // Control report. 1=Enable Actuators, 2=Disable Actuators, 4=Stop All Effects, 8=Reset, 16=Pause, 32=Continue
			if (bufsize >= 1) ffb_control(report[0]);
			//sendStatusReport(0);
			break;
		case HID_ID_GAINREP: // Set global gain
			if (bufsize >= 1) set_gain(report[0]);
			break;
		case HID_ID_ENVREP: // Envelope
		{
			FFB_SetEnvelope_Data_t data{};
			if (copy_hid_report_with_id(report_id, report, bufsize, data)) set_envelope(&data);
			break;
		}
		case HID_ID_CONDREP: // Spring, Damper, Friction, Inertia
		{
			FFB_SetCondition_Data_t data{};
			if (copy_hid_report_with_id(report_id, report, bufsize, data)) set_condition(&data);
			break;
		}
		case HID_ID_PRIDREP: // Periodic
		{
			FFB_SetPeriodic_Data_t data{};
			if (copy_hid_report_with_id(report_id, report, bufsize, data)) set_periodic(&data);
			break;
		}
		case HID_ID_CONSTREP: // Constant
		{
			FFB_SetConstantForce_Data_t data{};
			if (copy_hid_report_with_id(report_id, report, bufsize, data)) set_constant_effect(&data);
			break;
		}
		case HID_ID_RAMPREP: // Ramp
		{
			FFB_SetRamp_Data_t data{};
			if (copy_hid_report_with_id(report_id, report, bufsize, data)) set_ramp(&data);
			break;
		}
		case HID_ID_CSTMREP: // Custom. pretty much never used
			break;
		case HID_ID_SMPLREP: // Download sample
			break;
		case HID_ID_EFOPREP: //Effect operation
		{
			FFB_EffOp_Data_t data{};
			if (copy_hid_report_with_id(report_id, report, bufsize, data)) set_effect_operation(&data);
			break;
		}
		case HID_ID_BLKFRREP: // Free a block (PID Block Free Report)
		{
			// Bug 5: Decrementa used_effects e atualiza ramPoolAvailable para evitar underflow
			if (bufsize < 1) break;
			uint8_t eid = report[0];
			if (eid > 0 && eid <= effects.size()) {
				effects_calc->free_effect(eid - 1);
				if (used_effects > 0) {
					used_effects--;
				}
				blockLoad_report.ramPoolAvailable = (effects.size() - used_effects) * sizeof(FFB_Effect);
			}
			break;
		}

		default:
		{
			break;
		}
	}

}


/**
 * Called on HID feature GET events
 * Any reply is assigned to the return buffer
 *
 * Handles block load reports and pool status which are requested after a new effect has been created
 */
uint16_t HidFFB::hidGet(uint8_t report_id, hid_report_type_t report_type,uint8_t* buffer, uint16_t reqlen){
	// Feature gets go here
	(void)report_type;
	if (buffer == nullptr || reqlen == 0) return 0;

	uint8_t id = report_id - FFB_ID_OFFSET;

	switch(id){
	case HID_ID_STATE:
		// Bug 2: DirectInput PID State Report (Report ID 2)
		// Chamado por jogos e utilitários (ex: Forza, GetForceFeedbackState) para
		// checar se atuadores e alimentação estão ativos (DIGFFS_ACTUATORSON | DIGFFS_POWERON).
		buffer[0] = this->reportFFBStatus.status;
		return 1;
	case HID_ID_BLKLDREP:
		// Notice: first byte ID is not present in the reply buffer because it is handled by tinyusb internally!
		{
			const uint16_t size = std::min<uint16_t>(reqlen, sizeof(FFB_BlockLoad_Feature_Data_t));
			memcpy(buffer, &this->blockLoad_report, size);
			return size;
		}
	case HID_ID_POOLREP:
		{
			const uint16_t size = std::min<uint16_t>(reqlen, sizeof(FFB_PIDPool_Feature_Data_t));
			memcpy(buffer, &this->pool_report, size);
			return size;
		}
	default:
		break;
	}
	return 0;
}

void HidFFB::start_FFB(){
#ifdef DEBUGLOG
	CommandHandler::logSerialDebug("FFB on");
#endif
	this->set_FFB(true);
}

void HidFFB::stop_FFB(){
#ifdef DEBUGLOG
	CommandHandler::logSerialDebug("FFB off");
#endif
	this->set_FFB(false);
}

void HidFFB::set_FFB(bool state)
{
	assert(effects_calc != nullptr);
	this->ffb_active = state;
	effects_calc->setActive(state);
}

void HidFFB::set_gain(uint8_t gain){
	assert(effects_calc != nullptr);
	effects_calc->setGain(gain);
}

void HidFFB::set_filters(FFB_Effect *effect){
	assert(effects_calc != nullptr);
	effects_calc->setFilters(effect);
}

void HidFFB::ffb_control(uint8_t cmd){

	if(cmd & 0x01){ // Enable Actuators
		start_FFB();
	}
	if(cmd & 0x02){ // Disable Actuators
		stop_FFB();
	}
	if(cmd & 0x04){ // Stop All Effects (Bug 6: para todos os efeitos ativos)
		for(auto& effect : effects){
			effect.state = 0;
		}
	}
	if(cmd & 0x08){ // Reset
		stop_FFB();
		reset_ffb();
	}
	if(cmd & 0x10){ // Pause
		stop_FFB();
	}
	if(cmd & 0x20){ // Continue
		start_FFB();
	}
}


void HidFFB::set_constant_effect(FFB_SetConstantForce_Data_t* data){
	if(data->effectBlockIndex == 0 || data->effectBlockIndex > effects.size()){
		return;
	}
	cfUpdateEvent();
	FFB_Effect& effect_p = effects[data->effectBlockIndex-1];

	effect_p.magnitude = data->magnitude;
}

void HidFFB::new_effect(FFB_CreateNewEffect_Feature_Data_t* effect){
	// Allocates a new effect
	if (effect->effectType == FFB_EFFECT_NONE || effect->effectType > FFB_EFFECT_CUSTOM) {
		blockLoad_report.effectBlockIndex = 0;
		blockLoad_report.loadStatus = 3; // Block Load Error: unsupported effect type
		return;
	}

	int32_t index = effects_calc->find_free_effect(effect->effectType); // next effect
	if(index == -1){
		blockLoad_report.effectBlockIndex = 0;
		blockLoad_report.loadStatus = 2;
#ifdef DEBUGLOG
		CommandHandler::logSerialDebug("Can't allocate a new effect");
#endif
		return;
	}
	FFB_Effect new_effect;
	new_effect.type = effect->effectType;

	this->effects_calc->logEffectType(effect->effectType,false);
#ifdef DEBUGLOG
	CommandHandler::logSerialDebug("New effect type:" + std::to_string(effect->effectType) + " idx: " + std::to_string(index-1));
#endif

	set_filters(&new_effect);

	effects[index] = std::move(new_effect);
	blockLoad_report.effectBlockIndex = index+1;
	used_effects++;
	blockLoad_report.ramPoolAvailable = (effects.size()-used_effects)*sizeof(FFB_Effect);
	blockLoad_report.loadStatus = 1;
	sendStatusReport(index+1);
	

}

/**
 * Sets up an effect
 * If the direction enable bit is set then only 1 condition block is used
 */
void HidFFB::set_effect(FFB_SetEffect_t* effect){
	uint8_t index = effect->effectBlockIndex;
	if(index > effects.size() || index == 0)
		return;

	FFB_Effect* effect_p = &effects[index-1];

	if (effect_p->type != effect->effectType){
		effect_p->startTime = 0;
		set_filters(effect_p);
	}

	effect_p->gain = effect->gain;
	effect_p->type = effect->effectType;
	effect_p->samplePeriod = effect->samplePeriod;

	bool directionEnable = (effect->enableAxis & this->directionEnableMask);
	bool overridesCondition = false;

//	if(effect_p->useSingleCondition){ // Only allow turning single condition off in case it was overridden by sending multiple conditions previously
//		effect_p->useSingleCondition = directionEnable; // If direction is used only a single parameter block is allowed. Somehow this is still set while 2 conditions are sent...
//	}
	// Conditional effects usually do not use directions
	if(!effect_p->useSingleCondition && (effect->effectType == FFB_EFFECT_SPRING || effect->effectType == FFB_EFFECT_DAMPER || effect->effectType == FFB_EFFECT_INERTIA || effect->effectType == FFB_EFFECT_FRICTION))
	{
		if(effect_p->conditions[0].isActive()){
			effect_p->axisMagnitudes[0] = 1.0f;
			overridesCondition = true;
		}
		if(effect_p->conditions[1].isActive() || effect_p->useSingleCondition){
			effect_p->axisMagnitudes[1] = 1.0f;
			overridesCondition = true;
		}
	}


	if(!overridesCondition){
		float phaseX = M_PI*2.0 * (effect->directionX/36000.0f);

		effect_p->axisMagnitudes[0] = directionEnable ? sin(phaseX) : (effect->enableAxis & X_AXIS_ENABLE ? (effect->directionX - 18000.0f) / 18000.0f : 0); // Angular vector if dirEnable used otherwise linear or 0 if axis enabled
		effect_p->axisMagnitudes[1] = directionEnable ? -cos(phaseX) : (effect->enableAxis & Y_AXIS_ENABLE ? -(effect->directionY - 18000.0f) / 18000.0f : 0);

		// Single-axis wheel fix (Bug 1): em volantes de 1 eixo (axisCount == 1),
		// se a direção polar enviada pelo jogo for Norte (0°) ou Sul (180°),
		// sin(phaseX) vale 0.0f. Sem um eixo Y físico para consumir -cos(phaseX),
		// toda a força do efeito era perdida (resultando em 0 Nm).
		// Redirecionamos a componente não-nula para o eixo 0.
		if (axisCount == 1) {
            // Polar directions at 180°/360° produce tiny nonzero sine values
            // in floating point, so detect a numerically-zero X component.
            if (fabsf(effect_p->axisMagnitudes[0]) < 1e-6f) {
				float fallback = directionEnable ? -cosf(phaseX) :
				                 (effect->enableAxis & Y_AXIS_ENABLE ? -(effect->directionY - 18000.0f) / 18000.0f : 1.0f);
				if (fallback == 0.0f) fallback = 1.0f;
				effect_p->axisMagnitudes[0] = fallback;
			}
		} else {
			if (effect_p->axisMagnitudes[0] == 0.0f && effect_p->axisMagnitudes[1] == 0.0f) {
				effect_p->axisMagnitudes[0] = 1.0f;
			}
		}
	}

#if MAX_AXIS == 3
	float phaseY = M_PI*2.0 * (effect->directionY/36000.0);
	effect_p->axisMagnitudes[3] = directionEnable ? sin(phaseY) : (effect->enableAxis & Z_AXIS_ENABLE ? (effect->directionZ - 18000.0f) / 18000.0f : 0);
#endif
	if(effect->duration == 0){ // Fix for games assuming 0 is infinite
		effect_p->duration = FFB_EFFECT_DURATION_INFINITE;
	}else{
		effect_p->duration = effect->duration;
	}
	effect_p->startDelay = effect->startDelay;
	if(!ffb_active)
		start_FFB();

	sendStatusReport(effect->effectBlockIndex);
	//CommandHandler::logSerialDebug("Setting Effect: " + std::to_string(effect->effectType) +  " at " + std::to_string(index) + "\n");
}

/**
	If the number of Condition report blocks is equal to the number of axes for the effect, then the first report
	block applies to the first axis, the second applies to the second axis, and so on. For example, a two-axis
	spring condition with CP Offset set to zero in both Condition report blocks would have the same effect as
	the joystick self-centering spring. When a condition is defined for each axis in this way, the effect must
	not be rotated.

	If there is a single Condition report block for an effect with more than one axis, then the direction along
	which the parameters of the Condition report block are in effect is determined by the direction parameters
	passed in the Direction field of the Effect report block. For example, a friction condition rotated 45
	degrees (in polar coordinates) would resist joystick motion in the northeast-southwest direction but would
	have no effect on joystick motion in the northwest-southeast direction.
 */
void HidFFB::set_condition(FFB_SetCondition_Data_t *cond){
	if(cond->effectBlockIndex == 0 || cond->effectBlockIndex > effects.size()){
		return;
	}
	// Parameter block offsets refer to the logical HID axes in the descriptor,
	// not the number of physical motor axes. Keep X and Y conditions in separate
	// slots even when only X is connected to a motor; otherwise Y can overwrite X.
	uint8_t axis = std::min<uint8_t>((uint8_t)(MAX_AXIS - 1),
			(uint8_t)(cond->parameterBlockOffset & 0x03));

	FFB_Effect *effect = &effects[cond->effectBlockIndex - 1];
	effect->conditions[axis].cpOffset = cond->cpOffset;
	effect->conditions[axis].negativeCoefficient = cond->negativeCoefficient;
	effect->conditions[axis].positiveCoefficient = cond->positiveCoefficient;
	effect->conditions[axis].negativeSaturation = cond->negativeSaturation;
	effect->conditions[axis].positiveSaturation = cond->positiveSaturation;
	effect->conditions[axis].deadBand = cond->deadBand;

	// Bug 3: DirectInput DICONDITION default saturation
	// Se o jogo envia coeficiente não-nulo mas saturação zero (ex: AC, rFactor 2, AMS 2),
	// no padrão DirectInput isso significa "sem limite de saturação / saturação máxima (100% = 0x7FFF)".
	if(effect->conditions[axis].positiveSaturation == 0 && effect->conditions[axis].positiveCoefficient != 0){
		effect->conditions[axis].positiveSaturation = 0x7FFF;
	}
	if(effect->conditions[axis].negativeSaturation == 0 && effect->conditions[axis].negativeCoefficient != 0){
		effect->conditions[axis].negativeSaturation = 0x7FFF;
	}

	if(axis>0 && axis < MAX_AXIS && effect->conditions[axis].isActive()){ // Workaround when direction enable is set but multiple conditions are defined... Resets direction and uses conditions again
		effect->useSingleCondition = false;
	}
	if((effect->conditions[axis].isActive() || (axis > 0 && effect->useSingleCondition))  && effect->axisMagnitudes[axis] == 0){
		effect->axisMagnitudes[axis] = 1.0;
	}
}

void HidFFB::set_effect_operation(FFB_EffOp_Data_t* report){
	if(report->effectBlockIndex == 0 || report->effectBlockIndex > effects.size()){
		return; // Invalid ID
	}
	// Start or stop effect
	uint8_t id = report->effectBlockIndex-1;
	if(report->state == 3){
		effects[id].state = 0; //Stop
		effects[id].loopsRemaining = 0;
#ifdef DEBUGLOG
		CommandHandler::logSerialDebug("Stop effect: " + std::to_string(id));
#endif

	}else{

		// 1 = start, 2 = start solo
		if(report->state == 2){
#ifdef DEBUGLOG
			CommandHandler::logSerialDebug("Start solo: " + std::to_string(id));
#endif
			for(FFB_Effect& effect : effects){
				effect.state = 0; // Stop all other effects
			}
		}
		if(effects[id].state != 1){
			set_filters(&effects[id]);
		}
#ifdef DEBUGLOG
		CommandHandler::logSerialDebug("Start effect: " + std::to_string(id));
#endif
		effects[id].startTime = HAL_GetTick() + effects[id].startDelay;
		effects[id].loopsRemaining = report->loopCount;
		effects[id].state = 1; //Start
	}
	this->effects_calc->logEffectState(effects[id].type,effects[id].state);
}


void HidFFB::set_envelope(FFB_SetEnvelope_Data_t *report){
	if(report->effectBlockIndex == 0 || report->effectBlockIndex > effects.size()){
		return;
	}
	FFB_Effect *effect = &effects[report->effectBlockIndex - 1];

	effect->attackLevel = report->attackLevel;
	effect->attackTime = report->attackTime;
	effect->fadeLevel = report->fadeLevel;
	effect->fadeTime = report->fadeTime;
	effect->useEnvelope = true;
}

void HidFFB::set_ramp(FFB_SetRamp_Data_t *report){
	if(report->effectBlockIndex == 0 || report->effectBlockIndex > effects.size()){
		return;
	}
	FFB_Effect *effect = &effects[report->effectBlockIndex - 1];
	effect->magnitude = 0x7fff; // Full magnitude for envelope calculation. This effect does not have a periodic report
	effect->startLevel = report->startLevel;
	effect->endLevel = report->endLevel;
}

void HidFFB::set_periodic(FFB_SetPeriodic_Data_t* report){
	if(report->effectBlockIndex == 0 || report->effectBlockIndex > effects.size()){
		return;
	}
	FFB_Effect* effect = &effects[report->effectBlockIndex-1];

	effect->period = clip<uint32_t,uint32_t>(report->period,1,0x7fff); // Period is never 0
	effect->periodScale = 1.0f / ((float)effect->period * 1000.0f);
	effect->magnitude = report->magnitude;
	effect->offset = report->offset;
	effect->phase = (uint16_t)(report->phase % 36000u);
}


void HidFFB::reset_ffb(){
	for(uint8_t i=0;i<effects.size();i++){
		effects_calc->free_effect(i);
	}
	this->reportFFBStatus.status = (HID_ACTUATOR_POWER) | (HID_ENABLE_ACTUATORS);
	used_effects = 0;
	blockLoad_report.ramPoolAvailable = effects.size() * sizeof(FFB_Effect);
}
