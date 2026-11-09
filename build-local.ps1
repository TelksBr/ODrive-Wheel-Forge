param(
  [string]$ToolchainBin = "",
  [string]$MsysBash = "C:\msys64\usr\bin\bash.exe",
  [switch]$NoClean,
  [switch]$SkipSubmoduleInit
)

$ErrorActionPreference = "Stop"

# Auto-discover ARM toolchain — try common install locations across versions.
# User can override via -ToolchainBin param.
if ([string]::IsNullOrEmpty($ToolchainBin)) {
  $candidates = @(
    # Modern Arm GNU Toolchain (post-2022 rename)
    "C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\*\bin",
    "C:\Program Files\Arm GNU Toolchain arm-none-eabi\*\bin",
    # Legacy GNU Arm Embedded Toolchain (pre-2022)
    "C:\Program Files (x86)\GNU Arm Embedded Toolchain\*\bin",
    "C:\Program Files\GNU Arm Embedded Toolchain\*\bin"
  )
  foreach ($pattern in $candidates) {
    $found = Get-ChildItem -Path $pattern -ErrorAction SilentlyContinue |
             Sort-Object FullName -Descending | Select-Object -First 1
    if ($found) {
      $ToolchainBin = $found.FullName
      Write-Host "Auto-detected ARM toolchain: $ToolchainBin"
      break
    }
  }
}

if (!(Test-Path $MsysBash)) {
  Write-Error "MSYS2 bash not found at $MsysBash. Install from https://www.msys2.org/ or override with -MsysBash."
  exit 1
}

if ([string]::IsNullOrEmpty($ToolchainBin) -or !(Test-Path $ToolchainBin)) {
  Write-Error "ARM toolchain not found. Install from https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads or override with -ToolchainBin."
  exit 1
}

# Submodule init — pulls OpenFFBoard-master sources which are referenced by
# the Makefile via include paths. Skipped via -SkipSubmoduleInit on repeated
# runs to save time.
if (-not $SkipSubmoduleInit) {
  Write-Host "Ensuring submodules are initialized..."
  & git submodule update --init --recursive
  if ($LASTEXITCODE -ne 0) {
    Write-Error "git submodule update failed. Run manually: git submodule update --init --recursive"
    exit 1
  }
}

function Convert-ToMsysPath {
  param([string]$Path)
  $full = (Resolve-Path $Path).Path
  $drive, $rest = $full -split ":", 2
  $drive = $drive.ToLower()
  $rest = $rest -replace "\\", "/"
  return "/$drive$rest"
}

$repoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$firmwareDir = Join-Path $repoRoot "Odrive-Wheel"
$odriveRoot = Join-Path $repoRoot "ODrive-fw-v0.5.6\ODrive-fw-v0.5.6"
$odriveFirmwareDir = Join-Path $odriveRoot "Firmware"
$autogenDir = Join-Path $odriveFirmwareDir "autogen"

$toolsRoot = Join-Path $odriveRoot "tools"
$odriveTools = Join-Path $toolsRoot "odrive"
$fibreTools = Join-Path $toolsRoot "fibre-tools"
$versionScript = Join-Path $odriveTools "version.py"
$interfaceGenerator = Join-Path $fibreTools "interface_generator.py"

function Ensure-ToolFile {
  param(
    [string]$Path,
    [string]$Url
  )
  if (!(Test-Path $Path)) {
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $Path) | Out-Null
    Invoke-WebRequest -Uri $Url -OutFile $Path
  }
}

Ensure-ToolFile -Path $versionScript -Url "https://raw.githubusercontent.com/odriverobotics/ODrive/master/tools/odrive/version.py"
Ensure-ToolFile -Path $interfaceGenerator -Url "https://raw.githubusercontent.com/odriverobotics/ODrive/master/tools/fibre-tools/interface_generator.py"

$venvPython = Join-Path $repoRoot ".venv\bin\python.exe"
$venvPythonWin = Join-Path $repoRoot ".venv\Scripts\python.exe"
if (Test-Path $venvPython) {
  $pythonExe = $venvPython
} elseif (Test-Path $venvPythonWin) {
  $pythonExe = $venvPythonWin
} else {
  $pythonCmd = Get-Command python -ErrorAction SilentlyContinue
  if (-not $pythonCmd) {
    $pythonCmd = Get-Command python3 -ErrorAction SilentlyContinue
  }

  if (-not $pythonCmd) {
    Write-Error "Python 3 not found. Install Python and pip packages: pyyaml, jinja2, jsonschema>=4.0"
    exit 1
  }

  $pythonExe = $pythonCmd.Source
}

& $pythonExe -c "import yaml, jinja2, jsonschema" 2>$null
if ($LASTEXITCODE -ne 0) {
  Write-Host "Installing Python deps..."
  & $pythonExe -m pip install pyyaml jinja2 jsonschema>=4.0
  if ($LASTEXITCODE -ne 0) {
    Write-Error "Failed to install Python deps. Run: pip install pyyaml jinja2 jsonschema>=4.0"
    exit 1
  }
}

Push-Location $odriveFirmwareDir
New-Item -ItemType Directory -Force -Path "autogen" | Out-Null

& $pythonExe $versionScript --output autogen/version.c
if ($LASTEXITCODE -ne 0) { Pop-Location; Write-Error "Failed to generate autogen/version.c"; exit 1 }

& $pythonExe interface_generator_stub.py --definitions odrive-interface.yaml --template fibre-cpp/interfaces_template.j2 --output autogen/interfaces.hpp
if ($LASTEXITCODE -ne 0) { Pop-Location; Write-Error "Failed to generate autogen/interfaces.hpp"; exit 1 }

& $pythonExe interface_generator_stub.py --definitions odrive-interface.yaml --template fibre-cpp/function_stubs_template.j2 --output autogen/function_stubs.hpp
if ($LASTEXITCODE -ne 0) { Pop-Location; Write-Error "Failed to generate autogen/function_stubs.hpp"; exit 1 }

& $pythonExe interface_generator_stub.py --definitions odrive-interface.yaml --generate-endpoints ODrive --template fibre-cpp/endpoints_template.j2 --output autogen/endpoints.hpp
if ($LASTEXITCODE -ne 0) { Pop-Location; Write-Error "Failed to generate autogen/endpoints.hpp"; exit 1 }

& $pythonExe interface_generator_stub.py --definitions odrive-interface.yaml --template fibre-cpp/type_info_template.j2 --output autogen/type_info.hpp
if ($LASTEXITCODE -ne 0) { Pop-Location; Write-Error "Failed to generate autogen/type_info.hpp"; exit 1 }

Pop-Location

$msysToolchain = Convert-ToMsysPath $ToolchainBin
$msysFirmware = Convert-ToMsysPath $firmwareDir

# Limpa binários antigos para garantir que nunca aprove binário defasado
$buildDir = Join-Path $firmwareDir "build"
$binPath = Join-Path $buildDir "odrive-wheel.bin"
$elfPath = Join-Path $buildDir "odrive-wheel.elf"
$hexPath = Join-Path $buildDir "odrive-wheel.hex"

if (Test-Path $binPath) { Remove-Item -Force $binPath }
if (Test-Path $elfPath) { Remove-Item -Force $elfPath }
if (Test-Path $hexPath) { Remove-Item -Force $hexPath }

$scriptContent = @(
  "set -euo pipefail",
  "exec 2>&1",
  ('export PATH="{0}:/mingw64/bin:/usr/bin:$PATH"' -f $msysToolchain),
  ('cd "{0}"' -f $msysFirmware)
)

if (-not $NoClean) {
  $scriptContent += "/mingw64/bin/mingw32-make clean"
}

$scriptContent += '/mingw64/bin/mingw32-make -j$(nproc)'

$tempScript = Join-Path $firmwareDir ".build_runner.sh"
$runId = [Guid]::NewGuid().ToString("N")
$stdoutPath = Join-Path $firmwareDir ".build-$runId.stdout"
$stderrPath = Join-Path $firmwareDir ".build-$runId.stderr"
[System.IO.File]::WriteAllLines($tempScript, $scriptContent)
$msysScript = Convert-ToMsysPath $tempScript

Write-Host "Running build in $firmwareDir"
try {
  # Capture bash output as text files so Windows PowerShell does not convert
  # compiler stderr diagnostics into NativeCommandError records.
  $buildProcess = Start-Process -FilePath $MsysBash `
    -ArgumentList @($msysScript) `
    -Wait -PassThru -WindowStyle Hidden `
    -RedirectStandardOutput $stdoutPath `
    -RedirectStandardError $stderrPath

  if (Test-Path $stdoutPath) {
    Get-Content -LiteralPath $stdoutPath | ForEach-Object { Write-Output $_ }
  }
  if (Test-Path $stderrPath) {
    Get-Content -LiteralPath $stderrPath | ForEach-Object { Write-Output $_ }
  }
  $buildExitCode = $buildProcess.ExitCode
} finally {
  if (Test-Path $tempScript) {
    Remove-Item -Force $tempScript
  }
  if (Test-Path $stdoutPath) {
    Remove-Item -Force $stdoutPath
  }
  if (Test-Path $stderrPath) {
    Remove-Item -Force $stderrPath
  }
}

if ($buildExitCode -ne 0) {
  Write-Error "Build failed with exit code $buildExitCode"
  exit $buildExitCode
}

if (!(Test-Path $binPath)) {
  Write-Error "Missing build output: $binPath"
  exit 1
}

Write-Host "Build OK: $binPath"
