<#
.SYNOPSIS
  Packs Build123d and Build123d.Occt into the local folder feed artifacts/packages, then checks the packages load
  through a collectible AssemblyLoadContext from a folder of their own (tests/Build123d.Packaging.Tests).
.DESCRIPTION
  Build123d.Occt carries the native libraries flat under runtimes/<rid>/native: the shim b3docct and only the OCCT
  and FreeType libraries it depends on, found by following the DLL imports from b3docct (dumpbin, from the MSVC
  environment devenv.ps1 sets up).
.PARAMETER Rid
  The runtime to pack natives for.
.PARAMETER VersionSuffix
  A SemVer pre-release suffix, e.g. 'preview.1'.
.PARAMETER SkipTest
  Don't run the load-context test after packing.
#>
param(
  [ValidateSet('win-x64')] [string] $Rid = 'win-x64',
  [string] $VersionSuffix = '',
  [switch] $SkipTest
)
$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
. (Join-Path $root 'native/devenv.ps1') -Rid $Rid | Out-Null

# The native closure of b3docct
$available = @{}
foreach ($dir in (Join-Path $root ".deps/install/$Rid/bin"), (Join-Path $root "native/out/$Rid/bin")) {
  Get-ChildItem $dir -Filter *.dll | ForEach-Object { $available[$_.Name.ToLowerInvariant()] = $_.FullName }
}
if (-not $available.ContainsKey('b3docct.dll')) { throw "b3docct.dll not built: run native/build-shim.ps1" }
$needed = [System.Collections.Generic.HashSet[string]]::new()
$queue = [System.Collections.Generic.Queue[string]]::new()
$queue.Enqueue('b3docct.dll')
while ($queue.Count) {
  $dll = $queue.Dequeue()
  if (-not $needed.Add($dll)) { continue }
  $imports = & dumpbin /nologo /dependents $available[$dll] | Select-String '^\s+(\S+\.dll)\s*$' |
    ForEach-Object { $_.Matches[0].Groups[1].Value.ToLowerInvariant() }
  if ($LASTEXITCODE) { throw "dumpbin failed on $dll" }
  $imports | Where-Object { $available.ContainsKey($_) } | ForEach-Object { $queue.Enqueue($_) }
}

$stage = Join-Path $root ".deps/stage/$Rid/native"
if (Test-Path $stage) { Remove-Item -Recurse -Force $stage }
New-Item -ItemType Directory -Force $stage | Out-Null
$needed | ForEach-Object { Copy-Item $available[$_] $stage }
$size = ($needed | ForEach-Object { (Get-Item $available[$_]).Length } | Measure-Object -Sum).Sum / 1MB
"Natives: {0} libraries, {1:N1} MB" -f $needed.Count, $size

# Pack
$feed = Join-Path $root 'artifacts/packages'
$props = @("-p:NativeRid=$Rid", "-p:NativeStageDir=$stage/", "-p:PackageOutputPath=$feed")
if ($VersionSuffix) { $props += "-p:VersionSuffix=$VersionSuffix" }
foreach ($project in 'src/Build123d.Occt/Build123d.Occt.csproj', 'src/Build123d/Build123d.csproj') {
  dotnet pack (Join-Path $root $project) -c Release --artifacts-path (Join-Path $root '.deps/pack-build') @props -nologo
  if ($LASTEXITCODE) { throw "pack of $project failed ($LASTEXITCODE)" }
}
Get-ChildItem $feed -Filter '*.nupkg' | Sort-Object LastWriteTime | Select-Object -Last 2 | ForEach-Object Name

if (-not $SkipTest) {
  $env:B3D_PACKAGES = $feed
  dotnet test (Join-Path $root 'tests/Build123d.Packaging.Tests') -nologo
  if ($LASTEXITCODE) { throw "the load-context test failed ($LASTEXITCODE)" }
}
