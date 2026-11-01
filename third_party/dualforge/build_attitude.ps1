param([string]$SdlSource = $env:DUALFORGE_SDL_SOURCE_DIR)
$ErrorActionPreference = 'Stop'
$projectRoot = $PSScriptRoot
$toolRoot = 'E:\CLion 2026.1.1\bin'
$cmake = Join-Path $toolRoot 'cmake\win\x64\bin\cmake.exe'
$ninja = Join-Path $toolRoot 'ninja\win\x64\ninja.exe'
$compiler = Join-Path $toolRoot 'mingw\bin'
if (!(Test-Path -LiteralPath $cmake)) { throw 'Edit toolRoot in build_attitude.ps1 to point to your CLion toolchain.' }
# Isolate CMake from unrelated executables/DLLs on the user's global PATH.
$env:PATH = "$compiler;$(Split-Path -Parent $ninja);$env:SystemRoot\System32;$env:SystemRoot"
$sourceAlias = Join-Path $env:TEMP 'dualforge-attitude-src'
$buildRoot = Join-Path $env:TEMP 'dualforge-attitude-build'
if (Test-Path -LiteralPath $sourceAlias) {
    $existing = Get-Item -LiteralPath $sourceAlias
    if ($existing.LinkType -ne 'Junction' -or $existing.Target -ne $projectRoot) { throw "Build alias belongs to another project: $sourceAlias" }
} else { New-Item -ItemType Junction -Path $sourceAlias -Target $projectRoot | Out-Null }
$config = @('-S', $sourceAlias, '-B', $buildRoot, '-G', 'Ninja', "-DCMAKE_MAKE_PROGRAM=$ninja",
    "-DCMAKE_C_COMPILER=$compiler\gcc.exe", "-DCMAKE_CXX_COMPILER=$compiler\g++.exe", '-DCMAKE_BUILD_TYPE=Release',
    '-DDUALFORGE_BUILD_EXAMPLES=OFF', '-DDUALFORGE_BUILD_TESTS=ON', '-DDUALFORGE_BUILD_ATTITUDE_WEB=ON')
if ($SdlSource) { $config += "-DDUALFORGE_SDL_SOURCE_DIR=$SdlSource" }
Push-Location -LiteralPath $env:TEMP
try {
    & $cmake @config
    if ($LASTEXITCODE -ne 0) { throw 'CMake configuration failed' }
    & $cmake --build $buildRoot --parallel 6
    if ($LASTEXITCODE -ne 0) { throw 'Build failed' }
    & (Join-Path (Split-Path -Parent $cmake) 'ctest.exe') --test-dir $buildRoot --output-on-failure
    if ($LASTEXITCODE -ne 0) { throw 'Tests failed' }
} finally { Pop-Location }
$destination = Join-Path $projectRoot 'build-attitude'
New-Item -ItemType Directory -Path $destination -Force | Out-Null
Copy-Item -LiteralPath (Join-Path $buildRoot 'dualforge_attitude_web.exe') -Destination $destination
Get-ChildItem -LiteralPath $buildRoot -Filter '*.dll' -File | Copy-Item -Destination $destination
Copy-Item -LiteralPath (Join-Path $buildRoot 'attitude_web') -Destination $destination -Recurse -Force
Write-Host "Ready: $destination\dualforge_attitude_web.exe"
