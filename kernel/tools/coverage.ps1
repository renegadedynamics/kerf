<#
.SYNOPSIS
  Line coverage of the kernel's and the modeling layer's assemblies under their tests (phase10-plan.md 3.2), written
  to docs/coverage.md, and the public members no test calls, written to docs/public-members.md.
.DESCRIPTION
  Runs, each with Microsoft's code coverage (part of Microsoft.NET.Test.Sdk) writing Cobertura: the kernel's unit tests
  (Foundation, ModelingData, ModelingAlgorithms, DataExchange), the public API's tests, the Draw-lite tests in their default form (the corpus
  classes sample their cases; OCCTNET_CORPUS_FULL isn't set) and the modeling layer's suite (Kerf.Modeling.Tests);
  with -WithOracle, the oracle tests too (replayed from their recordings). Then tools/coverage-report.cs merges them
  and writes the report: coverage per OCCT package (the folder under each kernel assembly) and per folder of the
  modeling layer, the packages under the floor listed with their reasons (docs/coverage-gaps.tsv); and
  tools/public-members.cs compares the public API with the same data.
.PARAMETER Results
  The folder for the Cobertura files (a temporary one by default).
.PARAMETER Report
  The report (docs/coverage.md by default).
.PARAMETER Only
  Run only the test projects whose names contain one of these (all by default); the report merges what's in Results.
.PARAMETER WithOracle
  Run the oracle tests too (they replay native OCCT's recorded side, decision 1336).
.PARAMETER FullCorpus
  Also run the Draw-lite corpus whole (OCCTNET_CORPUS_FULL=1, about 25 minutes uninstrumented), kept apart in
  <Results>-corpus: the report's gaps then show how far the corpus covers each package.
.PARAMETER BuildArgs
  Extra build properties, e.g. OcctSourceDir and OcctTestDataDir where kernel/.deps lacks them.
#>
param(
  [string] $Results = (Join-Path ([IO.Path]::GetTempPath()) 'kerf-coverage'),
  [string] $Report = (Join-Path $PSScriptRoot '..\docs\coverage.md'),
  [string] $PublicMembers = (Join-Path $PSScriptRoot '..\docs\public-members.md'),
  [string[]] $Only = @(),
  [switch] $WithOracle,
  [switch] $FullCorpus,
  [string[]] $BuildArgs = @()
)
$ErrorActionPreference = 'Stop'

$kernel = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$root = (Resolve-Path (Join-Path $kernel '..')).Path
$projects = @(
  'kernel/tests/Kerf.Kernel.Foundation.Tests',
  'kernel/tests/Kerf.Kernel.ModelingData.Tests',
  'kernel/tests/Kerf.Kernel.ModelingAlgorithms.Tests',
  'kernel/tests/Kerf.Kernel.DataExchange.Tests',
  'kernel/tests/Kerf.Kernel.PublicApi.Tests',
  'kernel/tests/Kerf.Kernel.DrawLite.Tests',
  'modeling/tests/Kerf.Modeling.Tests'
)
if ($WithOracle) { $projects += 'kernel/tests/Kerf.Kernel.Oracle.Tests' }
if ($Only.Count -gt 0) { $projects = $projects | Where-Object { $p = $_; $Only | Where-Object { $p -like "*$_*" } } }
New-Item -ItemType Directory -Force $Results | Out-Null

foreach ($project in $projects) {
  $name = Split-Path $project -Leaf
  $out = Join-Path $Results $name
  Remove-Item -Recurse -Force $out -ErrorAction SilentlyContinue
  Write-Host "== $name"
  # the full corpus is slow: the Draw-lite tests run their default samples
  Remove-Item Env:OCCTNET_CORPUS_FULL -ErrorAction SilentlyContinue
  dotnet test (Join-Path $root $project) -c Release @BuildArgs --collect 'Code Coverage;Format=cobertura' --results-directory $out
  if ($LASTEXITCODE) { Write-Warning "$name had failures: its coverage is still counted" }
}

$corpus = "$Results-corpus"
if ($FullCorpus) {
  Remove-Item -Recurse -Force $corpus -ErrorAction SilentlyContinue
  $env:OCCTNET_CORPUS_FULL = '1'
  dotnet test (Join-Path $root 'kernel/tests/Kerf.Kernel.DrawLite.Tests') -c Release @BuildArgs --filter 'FullyQualifiedName~DrawCorpusTests' --collect 'Code Coverage;Format=cobertura' --results-directory $corpus
  Remove-Item Env:OCCTNET_CORPUS_FULL
  if ($LASTEXITCODE) { Write-Warning "the whole corpus had failures: its coverage is still counted" }
}
dotnet run (Join-Path $kernel 'tools/coverage-report.cs') -- $Results $Report 80 $corpus
dotnet run (Join-Path $kernel 'tools/public-members.cs') -- $Results (Join-Path $root 'modeling/tests/Kerf.Modeling.Tests/bin/Release/net10.0') $PublicMembers
