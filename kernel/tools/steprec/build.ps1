<#
.SYNOPSIS
  Builds steprec, the native recorder of OCCT's STEP parser (StepFile's flex scanner, bison parser and
  StepFile_ReadData, compiled from OCCT's source), and records the STEP files of OCCT's test dataset and of its source's
  data folder, and the hostile and mutated inputs of tests/OcctNet.DataExchange.Tests/StepFile/Recordings/cases.pack.gz
  (unit 8P):
    tools/steprec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-OcctSourceDir .deps\occt-src]
                            [-OcctTestDataDir .deps\occt-testdata\opencascade-dataset-7.9.0] [-Out <dir>] [-Bench]
  Writes steprec-corpus.txt.gz and steprec-cases.txt.gz to -Out, which the tests embed (StepFile/Recordings); with
  -Bench, also steprec-bench.txt: native's parse time per corpus file (the fastest of 5, from memory), which the
  benchmarks' "stepfile" suite reads.
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $OcctSourceDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-src'),
  [string] $OcctTestDataDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-testdata\opencascade-dataset-7.9.0'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'steprec'),
  [switch] $Bench
)
$ErrorActionPreference = 'Stop'
$inst = Join-Path $Build123dNetRoot '.deps\install\win-x64'
$stepFile = Join-Path $OcctSourceDir 'src\DataExchange\TKDESTEP\StepFile'
New-Item -ItemType Directory -Force $Out | Out-Null

# The inputs (StepFileCorpus.Inputs lists the same): every file of the dataset's step folder and every
# .stp/.step/.p21 file elsewhere in it, then data/step's, each "name<TAB>path", in ordinal order of the name
$list = [Collections.Generic.SortedDictionary[string, string]]::new([StringComparer]::Ordinal)
$root = (Resolve-Path $OcctTestDataDir).Path.TrimEnd('\')
Get-ChildItem $root -Recurse -File | ForEach-Object {
  $rel = $_.FullName.Substring($root.Length + 1).Replace('\', '/')
  if ($rel.StartsWith('step/') -or $_.Extension -match '^\.(stp|step|p21)$') { $list["testdata/$rel"] = $_.FullName }
}
Get-ChildItem (Join-Path $OcctSourceDir 'data\step') -File | ForEach-Object { $list["src/data/step/$($_.Name)"] = $_.FullName }
[IO.File]::WriteAllLines((Join-Path $Out 'inputs.txt'), [string[]]($list.GetEnumerator() | ForEach-Object { "$($_.Key)`t$($_.Value)" }))

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
  $libs = 'TKernel', 'TKXSBase' | ForEach-Object { Join-Path $inst "lib\$_.lib" }
  # OCCT's own scanner and parser, as TKDESTEP compiles them (they include <step.tab.hxx> and <FlexLexer.h>)
  & cl /nologo /O2 /EHa /MD /std:c++17 /DWNT /D_CRT_SECURE_NO_WARNINGS /I"$stepFile" /I"$inst\include\opencascade" `
    /Fe"steprec.exe" (Join-Path $PSScriptRoot 'main.cpp') (Join-Path $stepFile 'lex.step.cxx') `
    (Join-Path $stepFile 'step.tab.cxx') (Join-Path $stepFile 'StepFile_ReadData.cxx') /link $libs bcrypt.lib
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"

  & .\steprec.exe rec (Join-Path $Out 'inputs.txt') (Join-Path $Out 'steprec-corpus.txt')
  if ($LASTEXITCODE -ne 0) { throw "steprec rec failed ($LASTEXITCODE)" }
  Compress (Join-Path $Out 'steprec-corpus.txt') (Join-Path $Out 'steprec-corpus.txt.gz')

  # The hostile and mutated inputs, unpacked from the tests' pack
  $pack = Join-Path $PSScriptRoot '..\..\tests\OcctNet.DataExchange.Tests\StepFile\Recordings\cases.pack.gz'
  $in = [IO.File]::OpenRead($pack)
  $unzip = New-Object IO.Compression.GZipStream($in, [IO.Compression.CompressionMode]::Decompress)
  $raw = [IO.File]::Create((Join-Path $Out 'cases.pack'))
  $unzip.CopyTo($raw)
  $raw.Dispose(); $unzip.Dispose(); $in.Dispose()
  & .\steprec.exe pack (Join-Path $Out 'cases.pack') (Join-Path $Out 'steprec-cases.txt')
  if ($LASTEXITCODE -ne 0) { throw "steprec pack failed ($LASTEXITCODE)" }
  Compress (Join-Path $Out 'steprec-cases.txt') (Join-Path $Out 'steprec-cases.txt.gz')

  if ($Bench) {
    & .\steprec.exe bench (Join-Path $Out 'inputs.txt') (Join-Path $Out 'steprec-bench.txt') 5
    if ($LASTEXITCODE -ne 0) { throw "steprec bench failed ($LASTEXITCODE)" }
  }
  Write-Host "Recordings in $Out ($($list.Count) corpus inputs)"
}
finally { Pop-Location }
