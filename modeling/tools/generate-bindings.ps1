<#
.SYNOPSIS
  Regenerates the OCCT bindings from bindings/occt.txt: the C++ shim sources (native/shim/generated),
  the C# wrappers (src/Build123d.Occt/Generated) and the skipped-member report.
.PARAMETER Rid
  Runtime whose installed OCCT headers are parsed. The output doesn't depend on it.
#>
param([string] $Rid = 'win-x64')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot

dotnet run --project (Join-Path $root 'tools/OcctGen') -c Release -- `
  (Join-Path $root ".deps/install/$Rid/include/opencascade") `
  (Join-Path $root 'bindings/occt.txt') `
  (Join-Path $root 'native/shim/generated') `
  (Join-Path $root 'src/Build123d.Occt/Generated') `
  (Join-Path $root 'bindings/occt.skipped.txt')
if ($LASTEXITCODE) { throw "OcctGen failed ($LASTEXITCODE)" }
