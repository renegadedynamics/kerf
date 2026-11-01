<#
.SYNOPSIS
  Builds hlrrec, the native recorder of the foundations of the hidden-line removal (HLRAlgo, Intrv, Contap) (unit 7P)
  and of its data model (HLRTopoBRep, HLRBRep_Data) (unit 7Q1) and algorithms (HLRBRep_Algo, HLRBRep_PolyAlgo,
  HLRAppli_ReflectLines) (unit 7Q2), and records them for the tests
  (tests/OcctNet.ModelingAlgorithms.Tests/Contap/Recordings/hlrrec.txt.gz, HLRBRep/Recordings/hlrdata.txt.gz).
    tools/hlrrec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-Out <dir>] [-TestDataDir <opencascade-dataset>]
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'hlrrec'),
  [string] $TestDataDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-testdata\opencascade-dataset-7.9.0')
)
$ErrorActionPreference = 'Stop'
$inst = Join-Path $Build123dNetRoot '.deps\install\win-x64'
New-Item -ItemType Directory -Force $Out | Out-Null

if (-not $env:VCToolsInstallDir) {
  $installer = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer"
  $env:PATH = "$installer;$env:PATH"
  $vs = & (Join-Path $installer 'vswhere.exe') -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
  Import-Module (Join-Path $vs 'Common7\Tools\Microsoft.VisualStudio.DevShell.dll')
  Enter-VsDevShell -VsInstallPath $vs -SkipAutomaticLocation -DevCmdArguments '-arch=amd64 -host_arch=amd64' | Out-Null
}
Push-Location $Out
try {
  $libs = Get-ChildItem (Join-Path $inst 'lib\TK*.lib') | ForEach-Object { $_.FullName }
  & cl /nologo /O2 /EHa /MD /std:c++17 /D_USE_MATH_DEFINES /DWNT /I"$inst\include\opencascade" `
    /Fe"hlrrec.exe" (Join-Path $PSScriptRoot 'main.cpp') /link $libs
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  & .\hlrrec.exe (Join-Path $Out 'hlrrec.txt') (Join-Path $Out 'hlrdata.txt') $TestDataDir (Join-Path $Out 'hlralgo.txt')
  if ($LASTEXITCODE -ne 0) { throw "hlrrec failed ($LASTEXITCODE)" }
  # The tests embed them gzipped: tests/OcctNet.ModelingAlgorithms.Tests/Contap/Recordings/hlrrec.txt.gz (unit 7P) and
  # tests/OcctNet.ModelingAlgorithms.Tests/HLRBRep/Recordings/hlrdata.txt.gz (unit 7Q1) and hlralgo.txt.gz (unit 7Q2)
  foreach ($name in 'hlrrec', 'hlrdata', 'hlralgo') {
    $bytes = [IO.File]::ReadAllBytes((Join-Path $Out "$name.txt"))
    $gz = [IO.File]::Create((Join-Path $Out "$name.txt.gz"))
    $zip = New-Object IO.Compression.GZipStream($gz, [IO.Compression.CompressionLevel]::Optimal)
    $zip.Write($bytes, 0, $bytes.Length)
    $zip.Dispose(); $gz.Dispose()
  }
  Write-Host "Recordings in $Out"
}
finally { Pop-Location }
