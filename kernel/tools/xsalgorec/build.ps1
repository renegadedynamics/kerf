<#
.SYNOPSIS
  Builds xsalgorec, the native recorder of unit 8A2's XSAlgo (the shape processor with STEP's default sequences, the
  merge of its history into transfer processes, the AlgoContainer, CheckPCurve), and records it on data/occ's shapes
  up to 600 kB for the tests (tests/OcctNet.DataExchange.Tests/XSAlgo/Recordings/xsalgorec.txt.gz).
    tools/xsalgorec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-OcctSourceDir .deps\occt-src] [-Out <dir>]
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $OcctSourceDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-src'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'xsalgorec'),
  [long] $MaxSize = 600000
)
$ErrorActionPreference = 'Stop'
$inst = Join-Path $Build123dNetRoot '.deps\install\win-x64'
New-Item -ItemType Directory -Force $Out | Out-Null

# The inputs: data/occ's .brep files, by name in ordinal order
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
    /Fe"xsalgorec.exe" (Join-Path $PSScriptRoot 'main.cpp') /link $libs
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  $src = (Resolve-Path $OcctSourceDir).Path.Replace('\', '/').TrimEnd('/')
  & .\xsalgorec.exe (Join-Path $Out 'inputs.txt') (Join-Path $Out 'xsalgorec.txt') $src
  if ($LASTEXITCODE -ne 0) { throw "xsalgorec failed ($LASTEXITCODE)" }
  $bytes = [IO.File]::ReadAllBytes((Join-Path $Out 'xsalgorec.txt'))
  $gz = [IO.File]::Create((Join-Path $Out 'xsalgorec.txt.gz'))
  $zip = New-Object IO.Compression.GZipStream($gz, [IO.Compression.CompressionLevel]::Optimal)
  $zip.Write($bytes, 0, $bytes.Length)
  $zip.Dispose(); $gz.Dispose()
  Write-Host "Recording in $Out\xsalgorec.txt ($($inputs.Count) inputs)"
}
finally { Pop-Location }
