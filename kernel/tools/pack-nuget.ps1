<#
.SYNOPSIS
  Packs OCCT.NET's four assemblies and build123d.NET on OCCT.NET as NuGet packages, ready for nuget.org.
.DESCRIPTION
  OcctNet.Foundation, OcctNet.ModelingData, OcctNet.ModelingAlgorithms and OcctNet.DataExchange, then build123d.NET's
  Build123d.OcctNet (its OCCT API over OCCT.NET) and Build123d built on it (Build123dBackend=OcctNet: no native code),
  in Release with symbol packages, each depending on the one below at the same version. Each checkout's committed HEAD
  is copied (git archive) into a work folder and built there, so nothing is written into the checkouts.
  Schematiq.Collections, which OcctNet.Foundation and Build123d depend on, comes from nuget.org.

  Then the packages are checked as a consumer gets them: a new console project restores Build123d from the output
  folder (and nuget.org), into a package folder of its own so no cached copy stands in, and runs a smoke test: a
  filleted, drilled box written to STEP and glTF and read back, with no native library in its output.
.PARAMETER Suffix
  The prerelease label after the projects' VersionPrefix (0.1.0): preview.1 gives 0.1.0-preview.1.
.PARAMETER Build123d
  The build123d.NET checkout.
.PARAMETER Out
  The folder for the packages.
.PARAMETER Work
  A folder for the copies and the build; a new temporary folder if not given.
.PARAMETER SkipVerify
  Don't run the smoke test.
#>
param(
  [Parameter(Mandatory)] [string] $Suffix,
  [string] $Build123d = (Join-Path $PSScriptRoot '..\..\build123d.net'),
  [string] $Out = (Join-Path $PSScriptRoot "..\..\nuget-release\0.1.0-$Suffix"),
  [string] $Work = (Join-Path ([IO.Path]::GetTempPath()) ('occtnet-pack-' + [guid]::NewGuid().ToString('N'))),
  [switch] $SkipVerify
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
$occt = Join-Path $Work 'OCCT.NET'
$b3d = Join-Path $Work 'build123d.net'
$occtCommit = Export-Checkout (Join-Path $PSScriptRoot '..') $occt
$b3dCommit = Export-Checkout $Build123d $b3d
"OCCT.NET $occtCommit, build123d.NET $b3dCommit"

$common = @('-c', 'Release', '-nologo', "-p:VersionSuffix=$Suffix", '-p:ContinuousIntegrationBuild=true', '-o', $Out)
foreach ($p in 'OcctNet.Foundation', 'OcctNet.ModelingData', 'OcctNet.ModelingAlgorithms', 'OcctNet.DataExchange') {
  dotnet pack (Join-Path $occt "src/$p/$p.csproj") @common
  if ($LASTEXITCODE) { throw "packing $p failed" }
}
foreach ($p in 'Build123d.OcctNet', 'Build123d') {
  dotnet pack (Join-Path $b3d "src/$p/$p.csproj") @common -p:Build123dBackend=OcctNet "-p:OcctNetRoot=$occt/"
  if ($LASTEXITCODE) { throw "packing $p failed" }
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
    <PackageReference Include="Build123d" Version="0.1.0-$Suffix" />
  </ItemGroup>
</Project>
"@ | Set-Content -Encoding utf8 (Join-Path $smoke 'smoke.csproj')
  @'
using Build123d;

var part = Operations.Fillet(new Box(10, 10, 10).Edges(), 1) - new Cylinder(2, 20);
var step = new MemoryStream();
if (!Exporters3D.ExportStep(part, step)) throw new Exception("STEP export failed");
step.Position = 0;
var fromStep = Importers.ImportStep(step);
if (Math.Abs(fromStep.Volume - part.Volume) > 1e-6 * part.Volume) throw new Exception($"STEP volume {fromStep.Volume} for {part.Volume}");
var gltf = new MemoryStream();
if (!Exporters3D.ExportGltf(part, gltf, binary: true)) throw new Exception("glTF export failed");
gltf.Position = 0;
var fromGltf = Importers.ImportGltf(gltf);
if (fromGltf.Faces().Count == 0) throw new Exception("glTF read back no faces");
Console.WriteLine($"Smoke test passed: volume {part.Volume:F6}, STEP {step.Length} bytes, glTF {gltf.Length} bytes");
'@ | Set-Content -Encoding utf8 (Join-Path $smoke 'Program.cs')
  dotnet run --project (Join-Path $smoke 'smoke.csproj') -c Release -nologo
  if ($LASTEXITCODE) { throw "the smoke test failed" }
  $native = Get-ChildItem -Recurse (Join-Path $smoke 'bin') -Include 'b3docct.dll', 'TK*.dll'
  if ($native) { throw "native libraries in the smoke test's output: $($native.Name -join ', ')" }
}
"The packages are in $Out"
