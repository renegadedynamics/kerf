<#
.SYNOPSIS
  Regenerates the OCCT.NET backend's wrappers (src/Build123d.OcctNet/Generated) from Build123d.Occt's public API and
  OCCT.NET's, and the report of what they can't forward (bindings/occtnet.unforwarded.txt).
.DESCRIPTION
  Builds Build123d.Occt (managed only: its natives aren't needed) and OCCT.NET's DataExchange (with the assemblies it
  references), then runs
  tools/OcctNetGen over both assemblies. Rerun after regenerating the native bindings or moving to a new OCCT.NET.
.PARAMETER OcctNetRoot
  The OCCT.NET checkout to forward to (default: ../OCCT.NET beside this repository).
#>
param([string] $OcctNetRoot = '')
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
if (-not $OcctNetRoot) { $OcctNetRoot = Join-Path (Split-Path -Parent $root) 'OCCT.NET' }
$OcctNetRoot = (Resolve-Path $OcctNetRoot).Path

dotnet build (Join-Path $root 'src/Build123d.Occt/Build123d.Occt.csproj') -c Release -nologo -v q
if ($LASTEXITCODE) { throw "Build123d.Occt failed to build ($LASTEXITCODE)" }
$exchange = Join-Path $OcctNetRoot 'src/OcctNet.DataExchange/OcctNet.DataExchange.csproj'
dotnet build $exchange -c Release -nologo -v q
if ($LASTEXITCODE) { throw "OCCT.NET failed to build ($LASTEXITCODE)" }

# OCCT.NET's Bento.Collections, for its signatures
$nuget = if ($env:NUGET_PACKAGES) { $env:NUGET_PACKAGES } else { Join-Path $HOME '.nuget/packages' }
$bento = Get-ChildItem (Join-Path $nuget 'bento.collections') -Directory | Sort-Object Name | Select-Object -Last 1
$bentoLib = Join-Path $bento.FullName 'lib/net8.0'

dotnet run --project (Join-Path $root 'tools/OcctNetGen') -c Release -- `
  (Join-Path $root 'src/Build123d.Occt/bin/Release/net10.0/Build123d.Occt.dll') `
  (Join-Path $OcctNetRoot 'src/OcctNet.DataExchange/bin/Release/net10.0') `
  (Join-Path $root 'src/Build123d.OcctNet/Generated') `
  (Join-Path $root 'bindings/occtnet.handwritten.txt') `
  (Join-Path $root 'bindings/occtnet.unforwarded.txt') `
  $bentoLib
if ($LASTEXITCODE) { throw "OcctNetGen failed ($LASTEXITCODE)" }
