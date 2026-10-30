<#
.SYNOPSIS
  Builds fillrec, the native recorder of OCCT's GeomFill fillings (unit 5L3), and records the cases
  the tests compare with (tests/OcctNet.ModelingAlgorithms.Tests/GeomFill/Recordings/fillrec.txt.gz).
    tools/fillrec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-Out <dir>] [-Cases 60]
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'fillrec'),
  [int] $Cases = 60
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
  & cl /nologo /O2 /EHsc /MD /std:c++17 /D_USE_MATH_DEFINES /DWNT /I"$inst\include\opencascade" `
    /Fe"fillrec.exe" (Join-Path $PSScriptRoot 'main.cpp') /link $libs
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  $text = ((& .\fillrec.exe $Cases) -join "`n") + "`n"
  if ($LASTEXITCODE -ne 0) { throw "fillrec failed ($LASTEXITCODE)" }
  [IO.File]::WriteAllText((Join-Path $Out 'fillrec.txt'), $text)
  # The tests embed it gzipped: tests/OcctNet.ModelingAlgorithms.Tests/GeomFill/Recordings/fillrec.txt.gz
  $gz = [IO.File]::Create((Join-Path $Out 'fillrec.txt.gz'))
  $zip = New-Object IO.Compression.GZipStream($gz, [IO.Compression.CompressionLevel]::Optimal)
  $bytes = [Text.Encoding]::ASCII.GetBytes($text)
  $zip.Write($bytes, 0, $bytes.Length)
  $zip.Dispose(); $gz.Dispose()
  Write-Host "Recording in $Out\fillrec.txt"
}
finally { Pop-Location }
