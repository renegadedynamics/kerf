<#
.SYNOPSIS
  Builds hlrrec, the native recorder of the foundations of the hidden-line removal (HLRAlgo, Intrv, Contap) (unit 7P), and
  records them for the tests (tests/OcctNet.ModelingAlgorithms.Tests/Contap/Recordings/hlrrec.txt.gz).
    tools/hlrrec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-Out <dir>]
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'hlrrec')
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
  & .\hlrrec.exe (Join-Path $Out 'hlrrec.txt')
  if ($LASTEXITCODE -ne 0) { throw "hlrrec failed ($LASTEXITCODE)" }
  # The tests embed it gzipped: tests/OcctNet.ModelingAlgorithms.Tests/Contap/Recordings/hlrrec.txt.gz
  $bytes = [IO.File]::ReadAllBytes((Join-Path $Out 'hlrrec.txt'))
  $gz = [IO.File]::Create((Join-Path $Out 'hlrrec.txt.gz'))
  $zip = New-Object IO.Compression.GZipStream($gz, [IO.Compression.CompressionLevel]::Optimal)
  $zip.Write($bytes, 0, $bytes.Length)
  $zip.Dispose(); $gz.Dispose()
  Write-Host "Recording in $Out\hlrrec.txt"
}
finally { Pop-Location }
