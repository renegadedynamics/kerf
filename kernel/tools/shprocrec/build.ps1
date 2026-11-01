<#
.SYNOPSIS
  Builds shprocrec, the native recorder of ShapeProcess (unit 7C): OCCT's default sequences, the heal group's and each
  operator alone, on data/occ's shapes and the heal group's public inputs (up to 60 kB), for the tests
  (tests/OcctNet.ModelingAlgorithms.Tests/ShapeProcess/Recordings/shprocrec.txt.gz).
    tools/shprocrec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-OcctSourceDir .deps\occt-src]
                                [-OcctTestDataDir .deps\occt-testdata\opencascade-dataset-7.9.0] [-Out <dir>]
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $OcctSourceDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-src'),
  [string] $OcctTestDataDir = (Join-Path $PSScriptRoot '..\..\.deps\occt-testdata\opencascade-dataset-7.9.0'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'shprocrec'),
  [long] $MaxSize = 60000
)
$ErrorActionPreference = 'Stop'
$inst = Join-Path $Build123dNetRoot '.deps\install\win-x64'
New-Item -ItemType Directory -Force $Out | Out-Null

# The inputs: data/occ's .brep files, then the heal group's data files (.brep, .rle), each by name in ordinal order
$inputs = [Collections.Generic.List[string]]::new()
Get-ChildItem (Join-Path $OcctSourceDir 'data\occ') -Filter *.brep | Where-Object { $_.Length -le $MaxSize } |
  Sort-Object Name -CaseSensitive | ForEach-Object { $inputs.Add($_.FullName) }
$names = [Collections.Generic.SortedSet[string]]::new([StringComparer]::Ordinal)
Get-ChildItem (Join-Path $OcctSourceDir 'tests\heal') -Recurse -File | ForEach-Object {
  foreach ($m in [regex]::Matches([IO.File]::ReadAllText($_.FullName), 'locate_data_file\s+([^\]\s]+)')) { [void]$names.Add($m.Groups[1].Value) }
}
$byName = @{}
Get-ChildItem $OcctTestDataDir -Recurse -File | ForEach-Object { if (-not $byName.ContainsKey($_.Name)) { $byName[$_.Name] = $_ } }
foreach ($n in $names) {
  $f = $byName[$n]
  if ($f -and ($n -match '\.(brep|rle)$') -and $f.Length -le $MaxSize) { $inputs.Add($f.FullName) }
}
[IO.File]::WriteAllLines((Join-Path $Out 'inputs.txt'), $inputs)

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
    /Fe"shprocrec.exe" (Join-Path $PSScriptRoot 'main.cpp') /link $libs
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  $src = (Resolve-Path $OcctSourceDir).Path.Replace('\', '/').TrimEnd('/')
  $brep = (Resolve-Path (Join-Path $OcctTestDataDir 'brep')).Path.Replace('\', '/').TrimEnd('/')
  & .\shprocrec.exe (Join-Path $Out 'inputs.txt') (Join-Path $Out 'shprocrec.txt') $src $brep
  if ($LASTEXITCODE -ne 0) { throw "shprocrec failed ($LASTEXITCODE)" }
  # The tests embed it gzipped: tests/OcctNet.ModelingAlgorithms.Tests/ShapeProcess/Recordings/shprocrec.txt.gz
  $bytes = [IO.File]::ReadAllBytes((Join-Path $Out 'shprocrec.txt'))
  $gz = [IO.File]::Create((Join-Path $Out 'shprocrec.txt.gz'))
  $zip = New-Object IO.Compression.GZipStream($gz, [IO.Compression.CompressionLevel]::Optimal)
  $zip.Write($bytes, 0, $bytes.Length)
  $zip.Dispose(); $gz.Dispose()
  Write-Host "Recording in $Out\shprocrec.txt ($($inputs.Count) inputs)"
}
finally { Pop-Location }
