<#
.SYNOPSIS
  Builds stlrec, the native recorder of OCCT's STL reading and writing (RWStl, StlAPI; unit 8H1), and records:
  data/occ's shapes (up to 600 kB) meshed and written as ASCII and binary STL; a hand-made triangulation written; and
  the STL files of OCCT's data/stl, of the test dataset and of the tests' own inputs
  (tests/OcctNet.DataExchange.Tests/RWStl/Inputs), with data derived from some of them, read every way. The tests
  embed the recording gzipped (tests/OcctNet.DataExchange.Tests/RWStl/Recordings/stlrec.txt.gz).
    tools/stlrec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-OcctSourceDir .deps\occt-src]
                           [-OcctTestDataDir .deps\occt-testdata\opencascade-dataset-7.9.0] [-Out <dir>]
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $OcctSourceDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-src'),
  [string] $OcctTestDataDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-testdata\opencascade-dataset-7.9.0'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'stlrec'),
  [long] $MaxBrepSize = 600000
)
$ErrorActionPreference = 'Stop'
$inst = Join-Path $Build123dNetRoot '.deps\install\win-x64'
New-Item -ItemType Directory -Force $Out | Out-Null
$scratch = Join-Path $Out 'scratch'
New-Item -ItemType Directory -Force $scratch | Out-Null

function SortedFiles([string] $dir, [string] $filter) {
  Get-ChildItem $dir -Filter $filter -File | Sort-Object Name -CaseSensitive
}

$lines = [Collections.Generic.List[string]]::new()
# Writing: data/occ's shapes
SortedFiles (Join-Path $OcctSourceDir 'data\occ') '*.brep' | Where-Object { $_.Length -le $MaxBrepSize } |
  ForEach-Object { $lines.Add("W $($_.FullName)") }
$lines.Add('M')
# Reading: the tests' inputs, the dataset's (others/), data/stl
$inputs = (Resolve-Path (Join-Path $PSScriptRoot '..\..\tests\OcctNet.DataExchange.Tests\RWStl\Inputs')).Path
$others = Join-Path $OcctTestDataDir 'others'
$datastl = Join-Path $OcctSourceDir 'data\stl'
SortedFiles $inputs '*.stl' | ForEach-Object { $lines.Add("R $($_.FullName)") }
SortedFiles $others '*.stl' | ForEach-Object { $lines.Add("R $($_.FullName)") }
SortedFiles $datastl '*.stl' | ForEach-Object { $lines.Add("R $($_.FullName)") }
# Derived data: the same list as RWStlNativeTests.Derived
$a102 = Join-Path $others 'bug30109_a102.stl'
$b132 = Join-Path $others 'bug30109_b132.stl'
$box = Join-Path $others 'OMF6391_box.stl'
$design = Join-Path $others 'bug27622_Design1.stl'
$two = Join-Path $inputs 'twosolids.stl'
$shape = Join-Path $datastl 'shape.stl'
@(
  "cat2 $design", "cat2 $box", "ws $box", "cat2 $a102", "ws $a102", "cat2 $shape", "cat2 $two", "ws $two",
  "trunc $a102 100", "trunc $a102 133", "trunc $a102 134", "trunc $a102 200", "trunc $a102 683", "trunc $box 1500",
  "solidhdr $a102", "solidhdr $b132", "lie $a102 5", "lie $a102 -3", "lie $a102 -1000", "crlf $box", "crlf $two"
) | ForEach-Object { $lines.Add("D $_") }
[IO.File]::WriteAllLines((Join-Path $Out 'inputs.txt'), $lines)

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
    /Fe"stlrec.exe" (Join-Path $PSScriptRoot 'main.cpp') /link $libs bcrypt.lib
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  & .\stlrec.exe (Join-Path $Out 'inputs.txt') (Join-Path $Out 'stlrec.txt') $scratch
  if ($LASTEXITCODE -ne 0) { throw "stlrec failed ($LASTEXITCODE)" }
  $bytes = [IO.File]::ReadAllBytes((Join-Path $Out 'stlrec.txt'))
  $gz = [IO.File]::Create((Join-Path $Out 'stlrec.txt.gz'))
  $zip = New-Object IO.Compression.GZipStream($gz, [IO.Compression.CompressionLevel]::Optimal)
  $zip.Write($bytes, 0, $bytes.Length)
  $zip.Dispose(); $gz.Dispose()
  Write-Host "Recording in $Out\stlrec.txt ($($lines.Count) inputs)"
}
finally { Pop-Location }
