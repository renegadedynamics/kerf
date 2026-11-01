<#
.SYNOPSIS
  Builds helixrec, the native recorder of TKHelix (HelixBRep_BuilderHelix, HelixGeom_BuilderHelix,
  HelixGeom_BuilderHelixCoil, HelixGeom_Tools::ApprHelix; unit 7D), and records them on the helix grid's parameters
  (tools/helixrec/cases.txt) for the tests (tests/OcctNet.ModelingAlgorithms.Tests/HelixBRep/Recordings/helixrec.txt.gz).
    tools/helixrec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-Out <dir>]
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs; TKHelix included).

  cases.txt lists the helix commands (setaxis, comphelix, helix, spiral, comphelix2, helix2, spiral2) each case of
  OCCT's tests/helix calls, one per line, "<case id> <command> <arguments as Draw passes them>". It was made by running
  the grid's cases in Draw-lite with those commands replaced by one that logs its arguments.
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'helixrec')
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
    /Fe"helixrec.exe" (Join-Path $PSScriptRoot 'main.cpp') /link $libs
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  & .\helixrec.exe (Join-Path $PSScriptRoot 'cases.txt') (Join-Path $Out 'helixrec.txt')
  if ($LASTEXITCODE -ne 0) { throw "helixrec failed ($LASTEXITCODE)" }
  # The tests embed it gzipped: tests/OcctNet.ModelingAlgorithms.Tests/HelixBRep/Recordings/helixrec.txt.gz
  $bytes = [IO.File]::ReadAllBytes((Join-Path $Out 'helixrec.txt'))
  $gz = [IO.File]::Create((Join-Path $Out 'helixrec.txt.gz'))
  $zip = New-Object IO.Compression.GZipStream($gz, [IO.Compression.CompressionLevel]::Optimal)
  $zip.Write($bytes, 0, $bytes.Length)
  $zip.Dispose(); $gz.Dispose()
  Write-Host "Recording in $Out\helixrec.txt"
}
finally { Pop-Location }
