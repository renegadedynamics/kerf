<#
.SYNOPSIS
  Packs OCCT.NET's four assemblies and build123d.NET on OCCT.NET as NuGet packages, ready for nuget.org.
.DESCRIPTION
  OcctNet.Foundation, OcctNet.ModelingData, OcctNet.ModelingAlgorithms and OcctNet.DataExchange, then build123d.NET's
  Build123d.OcctNet (its OCCT API over OCCT.NET) and Build123d built on it (Build123dBackend=OcctNet: no native code),
  in Release with symbol packages, each depending on the one below at the same version. Each checkout's committed HEAD
  is copied (git archive) into a work folder and built there, so nothing is written into the checkouts.
  Bento.Collections, which OcctNet.Foundation and Build123d depend on, isn't on nuget.org yet: its package from
  .deps/nuget is copied beside the others, to be pushed first (decision 1296).
.PARAMETER Suffix
  The prerelease label after the projects' VersionPrefix (0.1.0): preview.1 gives 0.1.0-preview.1.
.PARAMETER Build123d
  The build123d.NET checkout.
.PARAMETER Out
  The folder for the packages.
.PARAMETER Work
  A folder for the copies and the build; a new temporary folder if not given.
#>
param(
  [Parameter(Mandatory)] [string] $Suffix,
  [string] $Build123d = (Join-Path $PSScriptRoot '..\..\build123d.net'),
  [string] $Out = (Join-Path $PSScriptRoot "..\..\nuget-release\0.1.0-$Suffix"),
  [string] $Work = (Join-Path ([IO.Path]::GetTempPath()) ('occtnet-pack-' + [guid]::NewGuid().ToString('N')))
)
$ErrorActionPreference = 'Stop'

function Export-Checkout([string] $checkout, [string] $to) {
  New-Item -ItemType Directory -Force $to | Out-Null
  $archive = "$to.tar"
  git -C $checkout archive --format=tar -o $archive HEAD
  if ($LASTEXITCODE) { throw "git archive of $checkout failed" }
  tar -xf $archive -C $to
  if ($LASTEXITCODE) { throw "extracting $archive failed" }
  # Bento.Collections, from the checkout's local feed (nuget.config)
  Copy-Item -Recurse (Join-Path $checkout '.deps/nuget') (Join-Path $to '.deps/nuget')
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
Copy-Item (Join-Path $occt '.deps/nuget/Bento.Collections.0.4.0.nupkg') $Out
"The packages are in $((Resolve-Path $Out).Path)"
