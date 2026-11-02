<#
.SYNOPSIS
  Builds stepctlrec, the native recorder of unit 8F (STEPControl's reader and writer), and records:
    tools/stepctlrec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-OcctSourceDir .deps\occt-src]
                               [-OcctTestDataDir .deps\occt-testdata\opencascade-dataset-7.9.0] [-Out <dir>] [-Dump]
  - stepctlrec-read.txt.gz: every STEP file of the corpus (the dataset's step/ and data/step, as StepRoundTripTests
    lists them) read with STEPControl_Reader and its roots transferred: the counts, the check lists, the transfer's
    messages;
  - stepctlrec-write.txt.gz: data/occ's .brep files and three assemblies written with STEPControl_Writer in each
    schema (AP203, AP214IS, AP242DIS) and model type, the masked STEP texts hashed.
  The tests embed the two recordings (tests/OcctNet.DataExchange.Tests/STEPControl/Recordings). -Dump also writes the
  masked STEP texts into <Out>\dump (for diagnosis).
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $OcctSourceDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-src'),
  [string] $OcctTestDataDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-testdata\opencascade-dataset-7.9.0'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'stepctlrec'),
  [switch] $Dump
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
    /I"$inst\include\opencascade" /Fe"stepctlrec.exe" (Join-Path $PSScriptRoot 'main.cpp') /link $libs bcrypt.lib
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  $extra = @()
  if ($Dump) { $dumpDir = Join-Path $Out 'dump'; New-Item -ItemType Directory -Force $dumpDir | Out-Null; $extra = @($dumpDir) }
  # the STEP reader reports a corrupted file's errors on stderr, which must not stop the script
  $ErrorActionPreference = 'Continue'
  & .\stepctlrec.exe write (Join-Path $Out 'breps.txt') (Join-Path $Out 'stepctlrec-write.txt') @extra
  if ($LASTEXITCODE -ne 0) { throw "stepctlrec write failed ($LASTEXITCODE)" }
  Compress (Join-Path $Out 'stepctlrec-write.txt') (Join-Path $Out 'stepctlrec-write.txt.gz')
  & .\stepctlrec.exe read (Join-Path $Out 'steps.txt') (Join-Path $Out 'stepctlrec-read.txt') > $null
  if ($LASTEXITCODE -ne 0) { throw "stepctlrec read failed ($LASTEXITCODE)" }
  Compress (Join-Path $Out 'stepctlrec-read.txt') (Join-Path $Out 'stepctlrec-read.txt.gz')
  Write-Host "Recordings in $Out ($($steps.Count) STEP files, $($breps.Count) BRep files)"
}
finally { Pop-Location }
