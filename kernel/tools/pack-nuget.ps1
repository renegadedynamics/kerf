<#
.SYNOPSIS
  Packs Kerf: the kernel's four assemblies and the modelling layer over them, as NuGet packages, ready for nuget.org.
.DESCRIPTION
  Kerf.Kernel.Foundation, Kerf.Kernel.ModelingData, Kerf.Kernel.ModelingAlgorithms and Kerf.Kernel.DataExchange, then the modelling layer,
  Kerf.Modeling, which calls the kernel directly (unit 10B: no native code, no forwarding layer), in Release with symbol
  packages, each depending on the one below at the same version. The repository's committed HEAD is copied (git
  archive) into a work folder and built there, so nothing is written into the checkout.

  Each assembly is compiled ahead of time (ReadyToRun) for win-x64, as Bento runs, keeping its IL: elsewhere .NET ignores
  the precompiled code and compiles the IL as usual. It cut the opening of Bento's example workbook by about a third
  (unit 9E, docs/performance.md). Each project is built, published with PublishReadyToRun,
  and packed without building again from its build output with the ReadyToRun assembly in place of its own.

  Then the packages are checked as a consumer gets them: a new console project restores Kerf.Modeling from the output
  folder (and nuget.org), into a package folder of its own so no cached copy stands in, and runs a smoke test: a
  filleted, drilled box written to STEP and glTF and read back, with no native library in its output.
.PARAMETER Suffix
  A prerelease label after the projects' VersionPrefix (0.2.0): preview.1 gives 0.2.0-preview.1. Without it, the release.
.PARAMETER Out
  The folder for the packages.
.PARAMETER Work
  A folder for the copies and the build; a new temporary folder if not given.
.PARAMETER SkipVerify
  Don't run the smoke test.
.PARAMETER NoReadyToRun
  Pack the assemblies as built (IL only).
#>
param(
  [string] $Suffix = '',
  [string] $Out = (Join-Path $PSScriptRoot "..\..\..\nuget-release\0.2.0$(if ($Suffix) { "-$Suffix" })"),
  [string] $Work = (Join-Path ([IO.Path]::GetTempPath()) ('occtnet-pack-' + [guid]::NewGuid().ToString('N'))),
  [switch] $SkipVerify,
  [switch] $NoReadyToRun
)
$ErrorActionPreference = 'Stop'

function Export-Checkout([string] $checkout, [string] $to) {
  New-Item -ItemType Directory -Force $to | Out-Null
  $archive = "$to.tar"
  git -C $checkout archive --format=tar -o $archive HEAD
  if ($LASTEXITCODE) { throw "git archive of $checkout failed" }
  tar -xf $archive -C $to
  if ($LASTEXITCODE) { throw "extracting $archive failed" }
  (git -C $checkout rev-parse --short HEAD).Trim()
}

New-Item -ItemType Directory -Force $Work, $Out | Out-Null
$repo = Join-Path $Work 'kerf'
$commit = Export-Checkout (Join-Path $PSScriptRoot '..\..') $repo
$occt = Join-Path $repo 'kernel'
$b3d = Join-Path $repo 'modeling'
"Kerf $commit"

# Whether an assembly holds ReadyToRun code: its CLI header's ManagedNativeHeader directory is set (read by hand, as
# Windows PowerShell's .NET Framework has no PEReader)
function Test-ReadyToRun([string] $assembly) {
  $b = [IO.File]::ReadAllBytes($assembly)
  $pe = [BitConverter]::ToInt32($b, 0x3C)
  $sections = [BitConverter]::ToUInt16($b, $pe + 6)
  $optional = $pe + 24
  $optionalSize = [BitConverter]::ToUInt16($b, $pe + 20)
  $directories = $optional + $(if ([BitConverter]::ToUInt16($b, $optional) -eq 0x20B) { 112 } else { 96 })
  $cliRva = [BitConverter]::ToInt32($b, $directories + 14 * 8)
  for ($i = 0; $i -lt $sections; $i++) {
    $section = $optional + $optionalSize + 40 * $i
    $va = [BitConverter]::ToInt32($b, $section + 12)
    $size = [BitConverter]::ToInt32($b, $section + 8)
    if ($cliRva -ge $va -and $cliRva -lt $va + $size) {
      $cli = $cliRva - $va + [BitConverter]::ToInt32($b, $section + 20)
      return [BitConverter]::ToInt32($b, $cli + 68) -gt 0
    }
  }
  $false
}

$version = "0.2.0$(if ($Suffix) { "-$Suffix" })"
$commit = (git -C $PSScriptRoot rev-parse HEAD).Trim()
$properties = @('-c', 'Release', '-nologo', "-p:VersionSuffix=$Suffix", '-p:ContinuousIntegrationBuild=true', "-p:RepositoryCommit=$commit")
$b3dProperties = @("-p:KerfKernelRoot=$occt/")
$projects = @(
  foreach ($p in 'Kerf.Kernel.Foundation', 'Kerf.Kernel.ModelingData', 'Kerf.Kernel.ModelingAlgorithms', 'Kerf.Kernel.DataExchange') {
    @{ Name = $p; Path = (Join-Path $occt "src/$p/$p.csproj"); Extra = @() }
  }
  foreach ($p in 'Kerf.Modeling') {
    @{ Name = $p; Path = (Join-Path $b3d "src/$p/$p.csproj"); Extra = $b3dProperties }
  }
)
foreach ($p in $projects) {
  dotnet build $p.Path @properties @($p.Extra)
  if ($LASTEXITCODE) { throw "building $($p.Name) failed" }
}
if (-not $NoReadyToRun) {
  # All published before any is swapped in: publishing a project builds the ones it references again, which would copy
  # their own assemblies back over the ReadyToRun ones
  foreach ($p in $projects) {
    dotnet publish $p.Path @properties @($p.Extra) -r win-x64 --self-contained false -p:PublishReadyToRun=true -o (Join-Path $Work "readytorun/$($p.Name)")
    if ($LASTEXITCODE) { throw "compiling $($p.Name) ahead of time failed" }
  }
  foreach ($p in $projects) {
    $target = (dotnet msbuild $p.Path -nologo -getProperty:TargetPath -p:Configuration=Release "-p:VersionSuffix=$Suffix" @($p.Extra)).Trim()
    Copy-Item (Join-Path $Work "readytorun/$($p.Name)/$($p.Name).dll") $target -Force
    if (-not (Test-ReadyToRun $target)) { throw "$target isn't ReadyToRun" }
  }
}
foreach ($p in $projects) {
  dotnet pack $p.Path @properties @($p.Extra) --no-build -o $Out
  if ($LASTEXITCODE) { throw "packing $($p.Name) failed" }
}
$Out = (Resolve-Path $Out).Path

if (-not $SkipVerify) {
  $smoke = Join-Path $Work 'smoke'
  New-Item -ItemType Directory -Force $smoke | Out-Null
  @"
<?xml version="1.0" encoding="utf-8"?>
<configuration>
  <config><add key="globalPackagesFolder" value="packages" /></config>
  <packageSources>
    <clear />
    <add key="release" value="$Out" />
    <add key="nuget.org" value="https://api.nuget.org/v3/index.json" />
  </packageSources>
</configuration>
"@ | Set-Content -Encoding utf8 (Join-Path $smoke 'nuget.config')
  @"
<Project Sdk="Microsoft.NET.Sdk">
  <PropertyGroup>
    <OutputType>Exe</OutputType>
    <TargetFramework>net10.0</TargetFramework>
    <Nullable>enable</Nullable>
    <ImplicitUsings>enable</ImplicitUsings>
  </PropertyGroup>
  <ItemGroup>
    <PackageReference Include="Kerf.Modeling" Version="$version" />
  </ItemGroup>
</Project>
"@ | Set-Content -Encoding utf8 (Join-Path $smoke 'smoke.csproj')
  @'
using Kerf.Modeling;

Part rounded = Operations.Fillet(new Box(10, 10, 10).Edges(), 1).Value;
Part part = rounded - new Cylinder(2, 20);
var step = new MemoryStream();
if (!Exporters3D.ExportStep(part, step).IsSuccess) throw new Exception("STEP export failed");
step.Position = 0;
var fromStep = Importers.ImportStep(step).Value;
if (Math.Abs(fromStep.Volume - part.Volume) > 1e-6 * part.Volume) throw new Exception($"STEP volume {fromStep.Volume} for {part.Volume}");
var gltf = new MemoryStream();
if (!Exporters3D.ExportGltf(part, gltf, new GltfExportOptions { Binary = true }).IsSuccess) throw new Exception("glTF export failed");
gltf.Position = 0;
var fromGltf = Importers.ImportGltf(gltf).Value;
if (fromGltf.Faces().Count == 0) throw new Exception("glTF read back no faces");
Console.WriteLine($"Smoke test passed: volume {part.Volume:F6}, STEP {step.Length} bytes, glTF {gltf.Length} bytes");
'@ | Set-Content -Encoding utf8 (Join-Path $smoke 'Program.cs')
  dotnet run --project (Join-Path $smoke 'smoke.csproj') -c Release -nologo
  if ($LASTEXITCODE) { throw "the smoke test failed" }
  $native = Get-ChildItem -Recurse (Join-Path $smoke 'bin') -Include 'b3docct.dll', 'TK*.dll'
  if ($native) { throw "native libraries in the smoke test's output: $($native.Name -join ', ')" }
  if (-not $NoReadyToRun) {
    foreach ($p in $projects) {
      $restored = Get-ChildItem -Recurse (Join-Path $smoke 'bin') -Filter "$($p.Name).dll" | Select-Object -First 1
      if (-not (Test-ReadyToRun $restored.FullName)) { throw "$($p.Name) from the packages isn't ReadyToRun" }
    }
    'The five assemblies from the packages are ReadyToRun'
  }
}
"The packages are in $Out"
