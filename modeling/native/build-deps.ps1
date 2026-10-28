<#
.SYNOPSIS
  Builds FreeType and OCCT for Windows into .deps/install/<rid>.
.PARAMETER Rid
  win-x64 (default) or win-arm64.
.PARAMETER OcctSource
  Optional existing OCCT checkout, to skip the download.
#>
param(
  [ValidateSet('win-x64', 'win-arm64')] [string] $Rid = 'win-x64',
  [string] $OcctSource = ''
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
. (Join-Path $PSScriptRoot 'devenv.ps1') -Rid $Rid

$build = Join-Path $root ".deps\build\$Rid"
$install = Join-Path $root ".deps\install\$Rid"
$cfg = @('-S', (Join-Path $PSScriptRoot 'occt'), '-B', $build, '-G', 'Ninja',
         '-DCMAKE_BUILD_TYPE=Release', "-DCMAKE_INSTALL_PREFIX=$install")
if ($OcctSource) { $cfg += "-DOCCT_SOURCE_DIR=$OcctSource" }

cmake @cfg
if ($LASTEXITCODE) { throw "configure failed ($LASTEXITCODE)" }
cmake --build $build
if ($LASTEXITCODE) { throw "build failed ($LASTEXITCODE)" }
