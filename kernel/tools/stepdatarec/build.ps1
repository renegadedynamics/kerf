<#
.SYNOPSIS
  Builds stepdatarec, the native recorder of unit 8B (StepData: StepReaderData's text decoding, the reader tool, the
  model, the header and StepWriter; Resource_Unicode's code tables), and records:
    tools/stepdatarec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-OcctSourceDir .deps\occt-src]
                                [-OcctTestDataDir .deps\occt-testdata\opencascade-dataset-7.9.0] [-Out <dir>]
  - stepdatarec-corpus.txt.gz: the corpus's STEP files (the dataset's and data/step's, as tools/steprec lists them)
    read into a model with the header protocol only, dumped, and written back;
  - stepdatarec-text.txt.gz: the texts of text-cases.txt decoded under every code page;
  - stepdatarec-tables.gz: OCCT's code tables (Resource_CodePages, Resource_Shiftjis, Resource_GB2312,
    Resource_GBK, Resource_Big5), which the tests compare with .NET's encodings.
  The tests embed them (tests/OcctNet.DataExchange.Tests/StepData/Recordings).
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $OcctSourceDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-src'),
  [string] $OcctTestDataDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-testdata\opencascade-dataset-7.9.0'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'stepdatarec')
)
$ErrorActionPreference = 'Stop'
$inst = Join-Path $Build123dNetRoot '.deps\install\win-x64'
$stepFile = Join-Path $OcctSourceDir 'src\DataExchange\TKDESTEP\StepFile'
$resource = Join-Path $OcctSourceDir 'src\FoundationClasses\TKernel\Resource'
New-Item -ItemType Directory -Force $Out | Out-Null

# The inputs, as tools/steprec lists them (StepDataNativeTests reads the same)
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
  $libs = 'TKernel', 'TKXSBase', 'TKDESTEP' | ForEach-Object { Join-Path $inst "lib\$_.lib" }
  & cl /nologo /O2 /EHa /MD /std:c++17 /DWNT /D_CRT_SECURE_NO_WARNINGS /DNOMINMAX /I"$stepFile" /I"$resource" `
    /I"$inst\include\opencascade" /Fe"stepdatarec.exe" (Join-Path $PSScriptRoot 'main.cpp') `
    (Join-Path $stepFile 'lex.step.cxx') (Join-Path $stepFile 'step.tab.cxx') (Join-Path $stepFile 'StepFile_ReadData.cxx') `
    /link $libs bcrypt.lib
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"

  & .\stepdatarec.exe tables (Join-Path $Out 'stepdatarec-tables.dat')
  if ($LASTEXITCODE -ne 0) { throw "stepdatarec tables failed ($LASTEXITCODE)" }
  Compress (Join-Path $Out 'stepdatarec-tables.dat') (Join-Path $Out 'stepdatarec-tables.gz')

  & .\stepdatarec.exe text (Join-Path $PSScriptRoot 'text-cases.txt') (Join-Path $Out 'stepdatarec-text.txt')
  if ($LASTEXITCODE -ne 0) { throw "stepdatarec text failed ($LASTEXITCODE)" }
  Compress (Join-Path $Out 'stepdatarec-text.txt') (Join-Path $Out 'stepdatarec-text.txt.gz')

  & .\stepdatarec.exe rec (Join-Path $Out 'inputs.txt') (Join-Path $Out 'stepdatarec-corpus.txt')
  if ($LASTEXITCODE -ne 0) { throw "stepdatarec rec failed ($LASTEXITCODE)" }
  Compress (Join-Path $Out 'stepdatarec-corpus.txt') (Join-Path $Out 'stepdatarec-corpus.txt.gz')
  Write-Host "Recordings in $Out ($($list.Count) corpus inputs)"
}
finally { Pop-Location }
