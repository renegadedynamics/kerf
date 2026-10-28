<#
.SYNOPSIS
  Builds the b3docct shim against the OCCT in .deps/install/<rid> and installs it into native/out/<rid>.
#>
param([ValidateSet('win-x64', 'win-arm64')] [string] $Rid = 'win-x64')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
. (Join-Path $PSScriptRoot 'devenv.ps1') -Rid $Rid

$build = Join-Path $root ".deps/build/shim-$Rid"
cmake -S (Join-Path $PSScriptRoot 'shim') -B $build -G Ninja -DCMAKE_BUILD_TYPE=Release `
  "-DCMAKE_PREFIX_PATH=$(Join-Path $root ".deps/install/$Rid")" `
  "-DCMAKE_INSTALL_PREFIX=$(Join-Path $root "native/out/$Rid")"
if ($LASTEXITCODE) { throw "configure failed ($LASTEXITCODE)" }
cmake --build $build --target install
if ($LASTEXITCODE) { throw "build failed ($LASTEXITCODE)" }
