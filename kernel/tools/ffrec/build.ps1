<#
.SYNOPSIS
  Builds ffrec, the native recorder of IntTools_FaceFace (unit 6D), and records the face pairs the port compares with
  (tests/OcctNet.ModelingAlgorithms.Tests/IntTools/FaceFaceRecordings.txt.gz).
    tools/ffrec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-OcctSource .deps\occt-src] [-Out <dir>]
  Needs Visual Studio's C++ tools, OCCT's source (for data/occ) and build123d.NET's native OCCT build (its
  .deps/install/win-x64: headers, import libraries, DLLs). Nothing is instrumented: the recorder links TKBO itself.
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $OcctSource = (Join-Path $PSScriptRoot '..\..\.deps\occt-src'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'ffrec')
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
    (Join-Path $PSScriptRoot 'main.cpp') /Fe"ffrec.exe" /link $libs
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  $prim = (& .\ffrec.exe primitives | Out-String) -replace "`r`n", "`n"
  if ($LASTEXITCODE -ne 0) { throw 'ffrec primitives failed' }
  $occ = (& .\ffrec.exe ((Join-Path $OcctSource 'data\occ').Replace([char]92, [char]47)) | Out-String) -replace "`r`n", "`n"
  if ($LASTEXITCODE -ne 0) { throw 'ffrec data/occ failed' }
  $text = $prim + $occ
  [IO.File]::WriteAllText((Join-Path $Out 'FaceFaceRecordings.txt'), $text)
  $gz = Join-Path $Out 'FaceFaceRecordings.txt.gz'
  $in = [IO.File]::OpenRead((Join-Path $Out 'FaceFaceRecordings.txt'))
  $outs = [IO.File]::Create($gz)
  $z = New-Object IO.Compression.GZipStream($outs, [IO.Compression.CompressionLevel]::Optimal)
  $in.CopyTo($z); $z.Dispose(); $outs.Dispose(); $in.Dispose()
  Write-Host "Recording in $gz"
}
finally { Pop-Location }
