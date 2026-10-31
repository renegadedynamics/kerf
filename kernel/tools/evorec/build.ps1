<#
.SYNOPSIS
  Builds evorec, the native recorder of BRepOffsetAPI_MakeEvolved (unit 5O2), and prints the values
  BRepFill_EvolvedTests.Vertical_profile_off_the_spine_follows_a_parallel checks (native MakeEvolved isn't bound).
    tools/evorec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-Out <dir>]
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs). Nothing is instrumented: the recorder links TKOffset itself.
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'evorec')
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
    (Join-Path $PSScriptRoot 'main.cpp') /Fe"evorec.exe" /link $libs
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  & .\evorec.exe
  if ($LASTEXITCODE -ne 0) { throw 'evorec failed' }
}
finally { Pop-Location }
