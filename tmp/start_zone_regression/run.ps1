param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$taskProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
Push-Location -LiteralPath $taskProjectRoot
try {
    $taskTestExecutable = Join-Path $PSScriptRoot 'start_zone_regression.exe'
    & $Compiler -std=c11 -Wall -Wextra -Werror -O0 -g -ffunction-sections `
        -fdata-sections -Itmp/start_zone_regression -IBSP -IAPP -ICore/Inc `
        tmp/start_zone_regression/start_zone_regression.c `
        BSP/screen.c BSP/QR.c BSP/task_plan.c `
        APP/task1_start_qr.c APP/task10_return_home.c `
        '-Wl,--gc-sections' -lm -o $taskTestExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Start-zone regression compilation failed.' }
    & $taskTestExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Start-zone regression failed.' }
}
finally {
    Pop-Location
}
