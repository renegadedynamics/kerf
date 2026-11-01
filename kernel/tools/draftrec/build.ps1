<#
.SYNOPSIS
  Builds draftrec, the native recorder of BRepOffsetAPI_MakeDraft, BiTgte_Blend and BRepOffsetAPI_MakeEvolved's
  volume mode (unit 7G), and records its generated inputs for the tests
  (tests/OcctNet.ModelingAlgorithms.Tests/Draft/Recordings/draftrec.txt.gz).
    tools/draftrec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-Out <dir>]
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'draftrec')
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
    /Fe"draftrec.exe" (Join-Path $PSScriptRoot 'main.cpp') /link $libs
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  & .\draftrec.exe (Join-Path $Out 'draftrec.txt')
  if ($LASTEXITCODE -ne 0) { throw "draftrec failed ($LASTEXITCODE)" }
  # The tests embed it gzipped: tests/OcctNet.ModelingAlgorithms.Tests/Draft/Recordings/draftrec.txt.gz
  $bytes = [IO.File]::ReadAllBytes((Join-Path $Out 'draftrec.txt'))
  $gz = [IO.File]::Create((Join-Path $Out 'draftrec.txt.gz'))
  $zip = New-Object IO.Compression.GZipStream($gz, [IO.Compression.CompressionLevel]::Optimal)
  $zip.Write($bytes, 0, $bytes.Length)
  $zip.Dispose(); $gz.Dispose()
  Write-Host "Recording in $Out\draftrec.txt"
}
finally { Pop-Location }
