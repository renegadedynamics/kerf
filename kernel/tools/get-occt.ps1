<#
.SYNOPSIS
  Checks out the OCCT source this port follows (occt-version.json) into .deps/occt-src, and checks the commit.
.PARAMETER Source
  Where to clone from. Defaults to the repository in occt-version.json; a local checkout that has the tag
  (such as ../build123d.net/.deps/occt-src) avoids the download.
#>
param([string] $Source)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$version = Get-Content (Join-Path $root 'occt-version.json') -Raw | ConvertFrom-Json
if (-not $Source) { $Source = $version.repository }
$target = Join-Path $root '.deps/occt-src'

if (-not (Test-Path $target)) {
    if (Test-Path $Source) { $Source = 'file:///' + ((Resolve-Path $Source).Path -replace '\\', '/') }
    git -c advice.detachedHead=false clone --quiet --depth 1 --branch $version.tag $Source $target
    if ($LASTEXITCODE) { throw "git clone failed ($LASTEXITCODE)" }
}

$head = git -C $target rev-parse HEAD
if ($head -ne $version.commit) {
    throw "$target is at $head, not $($version.tag) ($($version.commit)). Delete it and run this again."
}
"OCCT $($version.tag) ($head) is in $target"
