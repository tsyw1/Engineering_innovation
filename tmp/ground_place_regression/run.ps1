param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$taskProjectRoot = (Resolve-Path (Join-Path $PSScriptRoot '../..')).Path
Push-Location -LiteralPath $taskProjectRoot
try {
    $taskTestExecutable = Join-Path $PSScriptRoot 'ground_place_regression.exe'
    & $Compiler -std=c11 -Wall -Wextra -Werror -O0 -g -ffunction-sections `
        -fdata-sections -Itmp/ground_place_regression -IBSP -IAPP -ICore/Inc `
        tmp/ground_place_regression/ground_place_regression.c `
        BSP/arm_controller.c BSP/vision_fine_adjust.c `
        APP/task3_coarse_ring.c APP/task5_temporary_ring.c APP/task7_coarse_ring.c `
        APP/task9_temporary_stack.c `
        '-Wl,--gc-sections' -lm -o $taskTestExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Host regression compilation failed.' }
    & $taskTestExecutable
    if ($LASTEXITCODE -ne 0) { throw 'Host regression failed.' }
}
finally {
    Pop-Location
}
