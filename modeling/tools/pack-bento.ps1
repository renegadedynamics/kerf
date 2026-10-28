<#
.SYNOPSIS
  Packs Bento.Collections from a sibling checkout of the bento repository into the local NuGet feed
  .deps/nuget, which nuget.config lists. Temporary, until Bento.Collections is published to a feed.
.PARAMETER BentoRoot
  The bento checkout. Defaults to ../bento next to this repository.
#>
param([string] $BentoRoot = (Join-Path (Split-Path -Parent (Split-Path -Parent $PSScriptRoot)) 'bento'))
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$project = Join-Path $BentoRoot 'Bento.Collections/Bento.Collections.csproj'
if (-not (Test-Path $project)) { throw "Bento.Collections not found at $project" }

# Build output stays out of the bento checkout's own bin/obj, which its running app may be using
$feed = Join-Path $root '.deps/nuget'
dotnet pack $project -c Release --artifacts-path (Join-Path $root '.deps/bento-build') "-p:PackageOutputPath=$feed" -nologo
if ($LASTEXITCODE) { throw "pack failed ($LASTEXITCODE)" }
Get-ChildItem $feed -Filter 'Bento.Collections.*.nupkg' | ForEach-Object Name
