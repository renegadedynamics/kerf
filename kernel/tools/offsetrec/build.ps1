<#
.SYNOPSIS
  Builds offsetrec, the native recorder of BRepOffset_Analyse, BRepOffset_Offset and BRepOffset_MakeSimpleOffset
  (unit 7E), and records its generated primitives, Boolean and fillet results and data/occ's shapes (up to 300 kB) for
  the tests (tests/OcctNet.ModelingAlgorithms.Tests/BRepOffset/Recordings/offsetrec.txt.gz).
    tools/offsetrec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-OcctSourceDir .deps\occt-src]
                                [-OcctTestDataDir .deps\occt-testdata\opencascade-dataset-7.9.0] [-Out <dir>]
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $OcctSourceDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-src'),
  [string] $OcctTestDataDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-testdata\opencascade-dataset-7.9.0'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'offsetrec'),
  [long] $MaxSize = 300000
)
$ErrorActionPreference = 'Stop'
$inst = Join-Path $Build123dNetRoot '.deps\install\win-x64'
New-Item -ItemType Directory -Force $Out | Out-Null

# The inputs: data/occ's .brep files in ordinal order (the generated shapes come first, made by the recorder)
$inputs = [Collections.Generic.List[string]]::new()
Get-ChildItem (Join-Path $OcctSourceDir 'data\occ') -Filter *.brep | Where-Object { $_.Length -le $MaxSize } |
  Sort-Object Name -CaseSensitive | ForEach-Object { $inputs.Add($_.FullName) }
[IO.File]::WriteAllLines((Join-Path $Out 'inputs.txt'), $inputs)

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
    /Fe"offsetrec.exe" (Join-Path $PSScriptRoot 'main.cpp') /link $libs
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  & .\offsetrec.exe (Join-Path $Out 'inputs.txt') (Join-Path $Out 'offsetrec.txt')
  if ($LASTEXITCODE -ne 0) { throw "offsetrec failed ($LASTEXITCODE)" }
  # The tests embed it gzipped: tests/OcctNet.ModelingAlgorithms.Tests/BRepOffset/Recordings/offsetrec.txt.gz
  $bytes = [IO.File]::ReadAllBytes((Join-Path $Out 'offsetrec.txt'))
  $gz = [IO.File]::Create((Join-Path $Out 'offsetrec.txt.gz'))
  $zip = New-Object IO.Compression.GZipStream($gz, [IO.Compression.CompressionLevel]::Optimal)
  $zip.Write($bytes, 0, $bytes.Length)
  $zip.Dispose(); $gz.Dispose()
  Write-Host "Recording in $Out\offsetrec.txt ($($inputs.Count) inputs)"
}
finally { Pop-Location }
