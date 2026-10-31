<#
.SYNOPSIS
  Builds walkrec, the native recorder of OCCT's walking steps (unit 5C, decision 687), and records the cases the
  tests compare with (tests/OcctNet.ModelingAlgorithms.Tests/IntWalk/Recordings; the prmprm-* cases, unit 5D's
  IntPatch_PrmPrmIntersection, go to tests/OcctNet.ModelingAlgorithms.Tests/IntPatch/Recordings).
    tools/walkrec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-Out <dir>]
  Needs Visual Studio's C++ tools, Python, OCCT's source (tools/get-occt.ps1) and build123d.NET's native OCCT build
  (its .deps/install/win-x64: headers, import libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $OcctSource = (Join-Path $PSScriptRoot '..\..\.deps\occt-src'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'walkrec')
)
$ErrorActionPreference = 'Stop'
$inst = Join-Path $Build123dNetRoot '.deps\install\win-x64'
python (Join-Path $PSScriptRoot 'instrument.py') $OcctSource (Join-Path $inst 'include\opencascade') $Out
if ($LASTEXITCODE -ne 0) { throw 'instrument.py failed' }
Copy-Item (Join-Path $PSScriptRoot 'main.cpp') (Join-Path $Out 'src')

if (-not $env:VCToolsInstallDir) {
  $installer = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer"
  $env:PATH = "$installer;$env:PATH"
  $vs = & (Join-Path $installer 'vswhere.exe') -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
  Import-Module (Join-Path $vs 'Common7\Tools\Microsoft.VisualStudio.DevShell.dll')
  Enter-VsDevShell -VsInstallPath $vs -SkipAutomaticLocation -DevCmdArguments '-arch=amd64 -host_arch=amd64' | Out-Null
}
Push-Location $Out
try {
  New-Item -ItemType Directory -Force obj, rec | Out-Null
  $libs = Get-ChildItem (Join-Path $inst 'lib\TK*.lib') | ForEach-Object { $_.FullName }
  & cl /nologo /O2 /EHsc /MD /std:c++17 /D_USE_MATH_DEFINES /DWNT /I"$Out\inc" /I"$Out\src" `
    /I"$inst\include\opencascade" /Fo"obj\\" /Fe"walkrec.exe" src\main.cpp src\IntPatch_TheIWalking_0.cxx `
    src\IntPatch_ImpPrmIntersection.cxx src\IntPatch_PrmPrmIntersection.cxx src\IntWalk_PWalking.cxx /link $libs
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  foreach ($case in 'cyl-bspline-closed', 'cyl-bspline-open', 'plane-torus', 'sphere-bspline', 'cone-bspline',
                    'bspline-plane', 'pwalk-cyl-bspline', 'pwalk-torus-bspline', 'pwalk-bspline-bspline',
                    'prmprm-bspline-bspline', 'prmprm-offset-bspline', 'prmprm-poly', 'prmprm-start', 'prmprm-voxels',
                    'prmprm-listpnts', 'prmprm-self') {
    $text = (& .\walkrec.exe $case | Out-String) -replace "`r`n", "`n"
    [IO.File]::WriteAllText((Join-Path $Out "rec\$case.txt"), $text)
  }
  Write-Host "Recordings in $Out\rec"
}
finally { Pop-Location }
