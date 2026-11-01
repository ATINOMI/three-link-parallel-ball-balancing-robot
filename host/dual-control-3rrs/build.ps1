param(
  [string]$DriverRoot='E:\工科学习资料\计算机\dualforge-cpp-main\dualforge-cpp-main',
  [string]$SdlSource='C:\Users\ATINOMI\Documents\Codex\2026-10-10\z-l\work\sdl-source\SDL3-3.4.18'
)
$ErrorActionPreference='Stop'
$projectRoot=$PSScriptRoot
$toolRoot='E:\CLion 2026.1.1\bin'
$cmake=Join-Path $toolRoot 'cmake\win\x64\bin\cmake.exe'
$ninja=Join-Path $toolRoot 'ninja\win\x64\ninja.exe'
$compiler=Join-Path $toolRoot 'mingw\bin'
$savedPath=$env:PATH
try {
$env:PATH="$compiler;$(Split-Path -Parent $ninja);$env:SystemRoot\System32;$env:SystemRoot"
function SourceAlias([string]$name,[string]$target) {
  $aliasPath=Join-Path $env:TEMP $name
  if(Test-Path -LiteralPath $aliasPath) {
    $item=Get-Item -LiteralPath $aliasPath
    if($item.LinkType -ne 'Junction' -or $item.Target -ne $target){throw "Alias belongs to another project: $aliasPath"}
  }else{New-Item -ItemType Junction -Path $aliasPath -Target $target | Out-Null}
  return $aliasPath
}
$sourceAlias=SourceAlias 'dual-control-3rrs-src' $projectRoot
$driverAlias=SourceAlias 'dual-control-3rrs-driver' $DriverRoot
$buildRoot=Join-Path $env:TEMP 'dual-control-3rrs-build'
Push-Location -LiteralPath $env:TEMP
try {
  & $cmake -S $sourceAlias -B $buildRoot -G Ninja "-DCMAKE_MAKE_PROGRAM=$ninja" "-DCMAKE_C_COMPILER=$compiler\gcc.exe" "-DCMAKE_CXX_COMPILER=$compiler\g++.exe" '-DCMAKE_BUILD_TYPE=Release' "-DDUALFORGE_ROOT=$driverAlias" "-DDUALFORGE_SDL_SOURCE_DIR=$SdlSource"
  if($LASTEXITCODE -ne 0){throw 'CMake configure failed'}
  & $cmake --build $buildRoot --parallel 6
  if($LASTEXITCODE -ne 0){throw 'Build failed'}
  & (Join-Path (Split-Path -Parent $cmake) 'ctest.exe') --test-dir $buildRoot --output-on-failure
  if($LASTEXITCODE -ne 0){throw 'Native tests failed'}
}finally{Pop-Location}
$destination=Join-Path $projectRoot 'build-input-v2'
New-Item -ItemType Directory -Path $destination -Force | Out-Null
Copy-Item -LiteralPath (Join-Path $buildRoot 'dual_control_3rrs.exe') -Destination $destination
Get-ChildItem -LiteralPath $buildRoot -Filter '*.dll' -File | Copy-Item -Destination $destination
Copy-Item -LiteralPath (Join-Path $projectRoot 'web') -Destination $destination -Recurse -Force
Write-Host "Ready: $destination\dual_control_3rrs.exe"
}finally{$env:PATH=$savedPath}
