<#
.SYNOPSIS
  Builds pfrec, the native recorder of BOPAlgo_PaveFiller (unit 6E), and records the cases the port compares with
  (tests/OcctNet.ModelingAlgorithms.Tests/BOPAlgo/PaveFillerRecordings.txt.gz).
    tools/pfrec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-OcctSource .deps\occt-src] [-Out <dir>]
  Needs Visual Studio's C++ tools, OCCT's source (for tests/boolean) and build123d.NET's native OCCT build (its
  .deps/install/win-x64: headers, import libraries, DLLs). Nothing is instrumented: the recorder links TKBO itself.
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $OcctSource = (Join-Path $PSScriptRoot '..\..\.deps\occt-src'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'pfrec')
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
    (Join-Path $PSScriptRoot 'main.cpp') /Fe"pfrec.exe" /link $libs
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  $boolean = Join-Path $OcctSource 'tests\boolean'
  $list = Join-Path $Out 'cases.txt'
  $names = foreach ($g in 'bopfuse_simple', 'bopcommon_simple', 'bopcut_simple', 'boptuc_simple') {
    Get-ChildItem (Join-Path $boolean $g) -File | Sort-Object Name | ForEach-Object { "$g/$($_.Name)" }
  }
  [IO.File]::WriteAllLines($list, [string[]]$names)
  $text = (& .\pfrec.exe $boolean.Replace([char]92, [char]47) $list | Out-String) -replace "`r`n", "`n"
  if ($LASTEXITCODE -ne 0) { throw 'pfrec failed' }
  [IO.File]::WriteAllText((Join-Path $Out 'PaveFillerRecordings.txt'), $text)
  $gz = Join-Path $Out 'PaveFillerRecordings.txt.gz'
  $in = [IO.File]::OpenRead((Join-Path $Out 'PaveFillerRecordings.txt'))
  $outs = [IO.File]::Create($gz)
  $z = New-Object IO.Compression.GZipStream($outs, [IO.Compression.CompressionLevel]::Optimal)
  $in.CopyTo($z); $z.Dispose(); $outs.Dispose(); $in.Dispose()
  Write-Host "Recording in $gz"
}
finally { Pop-Location }
