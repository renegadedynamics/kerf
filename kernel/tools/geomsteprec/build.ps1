<#
.SYNOPSIS
  Builds geomsteprec, the native recorder of unit 8D (StepToGeom and GeomToStep), and records:
    tools/geomsteprec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-OcctSourceDir .deps\occt-src]
                                [-OcctTestDataDir .deps\occt-testdata\opencascade-dataset-7.9.0] [-Out <dir>] [-Full]
  - cases.txt (committed beside this script): the constructed curves, 2D curves, surfaces (GeomTools' compact text)
    and gp values, written by "geomsteprec gen";
  - geomsteprec-s2g.txt.gz: StepToGeom on every geometric entity of the corpus's STEP files (the dataset's step/ and
    data/step, as StepRoundTripTests lists them), with the default factors and with inches and degrees;
  - geomsteprec-g2s.txt.gz: GeomToStep on the cases and on the geometry of data/occ's .brep files, with both factor
    sets, each result written as STEP entities and converted back.
  The tests embed the two recordings (tests/OcctNet.DataExchange.Tests/GeomToStep/Recordings). -Full writes every
  line (for diagnosis) instead of the hashes and samples.
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $OcctSourceDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-src'),
  [string] $OcctTestDataDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-testdata\opencascade-dataset-7.9.0'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'geomsteprec'),
  [switch] $Full
)
$ErrorActionPreference = 'Stop'
$inst = Join-Path $Build123dNetRoot '.deps\install\win-x64'
New-Item -ItemType Directory -Force $Out | Out-Null

# The STEP inputs, as StepRoundTripTests.CorpusFiles lists them: the dataset's step/ then data/step, each folder's
# .stp and .step files in ordinal order
$steps = [Collections.Generic.List[string]]::new()
foreach ($pair in @(@('testdata/step', (Join-Path $OcctTestDataDir 'step')), @('src/data/step', (Join-Path $OcctSourceDir 'data\step')))) {
  if (-not (Test-Path $pair[1])) { continue }
  $files = Get-ChildItem $pair[1] -File | Where-Object { $_.Extension -match '^\.(stp|step)$' } | ForEach-Object { $_.Name }
  $sorted = [string[]]$files
  [Array]::Sort($sorted, [StringComparer]::Ordinal)
  foreach ($n in $sorted) { $steps.Add("$($pair[0])/$n`t$(Join-Path $pair[1] $n)") }
}
[IO.File]::WriteAllLines((Join-Path $Out 'steps.txt'), $steps)
# The BRep inputs: data/occ's .brep files in ordinal order
$breps = [string[]](Get-ChildItem (Join-Path $OcctSourceDir 'data\occ') -Filter *.brep | ForEach-Object { $_.Name })
[Array]::Sort($breps, [StringComparer]::Ordinal)
[IO.File]::WriteAllLines((Join-Path $Out 'breps.txt'), [string[]]($breps | ForEach-Object { "src/data/occ/$_`t$(Join-Path $OcctSourceDir "data\occ\$_")" }))

if (-not $env:VCToolsInstallDir) {
  $installer = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer"
  $env:PATH = "$installer;$env:PATH"
  $vs = & (Join-Path $installer 'vswhere.exe') -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
  Import-Module (Join-Path $vs 'Common7\Tools\Microsoft.VisualStudio.DevShell.dll')
  Enter-VsDevShell -VsInstallPath $vs -SkipAutomaticLocation -DevCmdArguments '-arch=amd64 -host_arch=amd64' | Out-Null
}

function Compress([string] $from, [string] $to) {
  $bytes = [IO.File]::ReadAllBytes($from)
  $gz = [IO.File]::Create($to)
  $zip = New-Object IO.Compression.GZipStream($gz, [IO.Compression.CompressionLevel]::Optimal)
  $zip.Write($bytes, 0, $bytes.Length)
  $zip.Dispose(); $gz.Dispose()
}

Push-Location $Out
try {
  $libs = Get-ChildItem (Join-Path $inst 'lib\TK*.lib') | ForEach-Object { $_.FullName }
  & cl /nologo /O2 /EHa /MD /std:c++17 /bigobj /D_USE_MATH_DEFINES /DWNT /DNOMINMAX /D_CRT_SECURE_NO_WARNINGS `
    /I"$inst\include\opencascade" /Fe"geomsteprec.exe" (Join-Path $PSScriptRoot 'main.cpp') /link $libs bcrypt.lib
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  $cases = Join-Path $PSScriptRoot 'cases.txt'
  & .\geomsteprec.exe gen $cases
  if ($LASTEXITCODE -ne 0) { throw "geomsteprec gen failed ($LASTEXITCODE)" }
  $mode = if ($Full) { 'full' } else { 'sampled' }
  & .\geomsteprec.exe g2s $cases (Join-Path $Out 'breps.txt') (Join-Path $Out 'geomsteprec-g2s.txt') $mode
  if ($LASTEXITCODE -ne 0) { throw "geomsteprec g2s failed ($LASTEXITCODE)" }
  Compress (Join-Path $Out 'geomsteprec-g2s.txt') (Join-Path $Out 'geomsteprec-g2s.txt.gz')
  & .\geomsteprec.exe s2g (Join-Path $Out 'steps.txt') (Join-Path $Out 'geomsteprec-s2g.txt') $mode
  if ($LASTEXITCODE -ne 0) { throw "geomsteprec s2g failed ($LASTEXITCODE)" }
  Compress (Join-Path $Out 'geomsteprec-s2g.txt') (Join-Path $Out 'geomsteprec-s2g.txt.gz')
  Write-Host "Recordings in $Out ($($steps.Count) STEP files, $($breps.Count) BRep files)"
}
finally { Pop-Location }
