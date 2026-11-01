<#
.SYNOPSIS
  Builds cksirec, a native run of BOPAlgo_CheckerSI on a translated shape (unit 7W, bugs/modalg_7/bug26034), and runs
  it on the case's shape at the translations of decision 8201:
    tools/cksirec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-TestData .deps\occt-testdata\opencascade-dataset-7.9.0] [-Out <dir>]
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $TestData = (Join-Path $PSScriptRoot '..\..\.deps\occt-testdata\opencascade-dataset-7.9.0'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'cksirec')
)
$ErrorActionPreference = 'Stop'
$inst = Join-Path $Build123dNetRoot '.deps\install\win-x64'
if (-not $env:VCToolsInstallDir) {
  $installer = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer"
  $env:PATH = "$installer;$env:PATH"
  $vs = & (Join-Path $installer 'vswhere.exe') -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
  Import-Module (Join-Path $vs 'Common7\Tools\Microsoft.VisualStudio.DevShell.dll')
  Enter-VsDevShell -VsInstallPath $vs -SkipAutomaticLocation -DevCmdArguments '-arch=amd64 -host_arch=amd64' | Out-Null
}
New-Item -ItemType Directory -Force $Out | Out-Null
Push-Location $Out
try {
  $libs = Get-ChildItem (Join-Path $inst 'lib\TK*.lib') | ForEach-Object { $_.FullName }
  & cl /nologo /O2 /EHsc /MD /std:c++17 /D_USE_MATH_DEFINES /DWNT /I"$inst\include\opencascade" `
    (Join-Path $PSScriptRoot 'main.cpp') /Fe"cksirec.exe" /link $libs
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  $brep = (Join-Path $TestData 'brep\bug26034_revolve-seg1-tcl-Revolve002.brep').Replace([char]92, [char]47)
  & .\cksirec.exe $brep 0 -0.5 0.5 -0.25
  if ($LASTEXITCODE -ne 0) { throw 'cksirec failed' }
}
finally { Pop-Location }
