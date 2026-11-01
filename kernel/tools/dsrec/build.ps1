<#
.SYNOPSIS
  Builds dsrec, the native recorder of the legacy data structure (TopOpeBRepDS) that native fillets and chamfers fill (unit 7L),
  and records it for the tests (tests/OcctNet.ModelingAlgorithms.Tests/TopOpeBRepDS/Recordings/dsrec.txt.gz).
    tools/dsrec/build.ps1 [-Build123dNetRoot ..\build123d.net] [-Out <dir>]
  Needs Visual Studio's C++ tools and build123d.NET's native OCCT build (its .deps/install/win-x64: headers, import
  libraries, DLLs).
#>
param(
  [string] $Build123dNetRoot = (Join-Path $PSScriptRoot '..\..\..\build123d.net'),
  [string] $Out = (Join-Path ([IO.Path]::GetTempPath()) 'dsrec'),
  # OCCT's sources: kpart.cpp's cases call ChFi3d_Builder_0.cxx, compiled unchanged into dsrec (unit 7N1)
  [string] $OcctSrc = (Join-Path $PSScriptRoot '..\..\.deps\occt-src')
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
    /Fe"dsrec.exe" (Join-Path $PSScriptRoot 'main.cpp') (Join-Path $PSScriptRoot 'kpart.cpp') `
    (Join-Path $OcctSrc 'src\ModelingAlgorithms\TKFillet\ChFi3d\ChFi3d_Builder_0.cxx') /link $libs
  if ($LASTEXITCODE -ne 0) { throw 'cl failed' }
  $env:PATH = "$(Join-Path $inst 'bin');$env:PATH"
  & .\dsrec.exe (Join-Path $Out 'dsrec.txt')
  if ($LASTEXITCODE -ne 0) { throw "dsrec failed ($LASTEXITCODE)" }
  # The tests embed it gzipped: tests/OcctNet.ModelingAlgorithms.Tests/TopOpeBRepDS/Recordings/dsrec.txt.gz
  $bytes = [IO.File]::ReadAllBytes((Join-Path $Out 'dsrec.txt'))
  $gz = [IO.File]::Create((Join-Path $Out 'dsrec.txt.gz'))
  $zip = New-Object IO.Compression.GZipStream($gz, [IO.Compression.CompressionLevel]::Optimal)
  $zip.Write($bytes, 0, $bytes.Length)
  $zip.Dispose(); $gz.Dispose()
  Write-Host "Recording in $Out\dsrec.txt"
  # The analytic fillets and ChFi3d_Builder_0's functions (unit 7N1):
  # tests/OcctNet.ModelingAlgorithms.Tests/ChFiKPart/Recordings/kpartrec.txt.gz
  & .\dsrec.exe --kpart (Join-Path $Out 'kpartrec.txt')
  if ($LASTEXITCODE -ne 0) { throw "dsrec --kpart failed ($LASTEXITCODE)" }
  $bytes = [IO.File]::ReadAllBytes((Join-Path $Out 'kpartrec.txt'))
  $gz = [IO.File]::Create((Join-Path $Out 'kpartrec.txt.gz'))
  $zip = New-Object IO.Compression.GZipStream($gz, [IO.Compression.CompressionLevel]::Optimal)
  $zip.Write($bytes, 0, $bytes.Length)
  $zip.Dispose(); $gz.Dispose()
  Write-Host "Recording in $Out\kpartrec.txt"
}
finally { Pop-Location }
