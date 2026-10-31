<#
.SYNOPSIS
  Builds borec, the native recorder of unit 6G's algorithms, and records an area's cases.
    tools/borec/build.ps1 -Area check|volume|connected|defeature [-Capture <capture file>] [-Out <dir>]
        [-Build123dNetRoot ..\build123d.net]
  The capture file holds the corpus calls Draw-lite captured (tests/OcctNet.DrawLite.Tests/DrawCapture.cs); without
  one, only the area's own random cases are recorded. The recording is written gzipped as <Out>/<area>.txt.gz, to be
  copied to tests/OcctNet.ModelingAlgorithms.Tests/BOPAlgo/Recordings/. Needs Visual Studio's C++ tools and
  build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import libraries, DLLs). Nothing is
  instrumented: the recorder links TKBO itself.
#>
param(
  [Parameter(Mandatory = $true)][ValidateSet('check', 'volume', 'connected', 'defeature')][string] $Area,
  [string] $Capture = '-',
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'borec')
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
  $sources = Get-ChildItem (Join-Path $PSScriptRoot '*.cpp') | ForEach-Object { $_.FullName }
  $cl = & cl /nologo /O2 /EHsc /MD /std:c++17 /D_USE_MATH_DEFINES /DWNT /I"$inst\include\opencascade" `
    $sources /Fe"borec.exe" /link $libs 2>&1
  if ($LASTEXITCODE -ne 0) { $cl | Select-String 'error' | ForEach-Object { Write-Host $_.Line }; throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  $text = Join-Path $Out "$Area.txt"
  & .\borec.exe $Area $Capture $text
  if ($LASTEXITCODE -ne 0) { throw 'borec failed' }
  $gz = Join-Path $Out "$Area.txt.gz"
  $in = [IO.File]::OpenRead($text)
  $outs = [IO.File]::Create($gz)
  $z = New-Object IO.Compression.GZipStream($outs, [IO.Compression.CompressionLevel]::Optimal)
  $in.CopyTo($z); $z.Dispose(); $outs.Dispose(); $in.Dispose()
  Write-Host "Recording in $gz"
}
finally { Pop-Location }
