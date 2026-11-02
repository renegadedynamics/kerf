<#
.SYNOPSIS
  Builds xscontrolrec, the native recorder of unit 8A3 (IFSelect and XSControl's work session on STEP files read by
  STEPControl_Reader: roots, signatures, check lists, labels, selections, in "lite" and "full" modes), and records the
  corpus's STEP files up to 3 MB (the dataset's and data/step's, as tools/stepdatarec lists them):
    tools/xscontrolrec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-OcctSourceDir .deps\occt-src]
                                 [-OcctTestDataDir .deps\occt-testdata\opencascade-dataset-7.9.0] [-Out <dir>]
  The tests embed xscontrolrec.txt.gz (tests/OcctNet.DataExchange.Tests/XSControl/Recordings).
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $OcctSourceDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-src'),
  [string] $OcctTestDataDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-testdata\opencascade-dataset-7.9.0'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'xscontrolrec'),
  [long] $MaxSize = 3000000
)
$ErrorActionPreference = 'Stop'
$inst = Join-Path $Build123dNetRoot '.deps\install\win-x64'
New-Item -ItemType Directory -Force $Out | Out-Null

# The inputs, as tools/stepdatarec lists them, up to MaxSize bytes (XSControlNativeTests reads the same)
$list = [Collections.Generic.SortedDictionary[string, string]]::new([StringComparer]::Ordinal)
$root = (Resolve-Path $OcctTestDataDir).Path.TrimEnd('\')
Get-ChildItem $root -Recurse -File | Where-Object { $_.Length -le $MaxSize } | ForEach-Object {
  $rel = $_.FullName.Substring($root.Length + 1).Replace('\', '/')
  if ($rel.StartsWith('step/') -or $_.Extension -match '^\.(stp|step|p21)$') { $list["testdata/$rel"] = $_.FullName }
}
Get-ChildItem (Join-Path $OcctSourceDir 'data\step') -File | Where-Object { $_.Length -le $MaxSize } |
  ForEach-Object { $list["src/data/step/$($_.Name)"] = $_.FullName }
[IO.File]::WriteAllLines((Join-Path $Out 'inputs.txt'), [string[]]($list.GetEnumerator() | ForEach-Object { "$($_.Key)`t$($_.Value)" }))

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
  & cl /nologo /O2 /EHa /MD /std:c++17 /DWNT /D_CRT_SECURE_NO_WARNINGS /DNOMINMAX /I"$inst\include\opencascade" `
    /Fe"xscontrolrec.exe" (Join-Path $PSScriptRoot 'main.cpp') /link $libs
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  & .\xscontrolrec.exe (Join-Path $Out 'inputs.txt') (Join-Path $Out 'xscontrolrec.txt')
  if ($LASTEXITCODE -ne 0) { throw "xscontrolrec failed ($LASTEXITCODE)" }
  $bytes = [IO.File]::ReadAllBytes((Join-Path $Out 'xscontrolrec.txt'))
  $gz = [IO.File]::Create((Join-Path $Out 'xscontrolrec.txt.gz'))
  $zip = New-Object IO.Compression.GZipStream($gz, [IO.Compression.CompressionLevel]::Optimal)
  $zip.Write($bytes, 0, $bytes.Length)
  $zip.Dispose(); $gz.Dispose()
  Write-Host "Recording in $Out\xscontrolrec.txt ($($list.Count) inputs)"
}
finally { Pop-Location }
