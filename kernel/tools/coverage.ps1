<#
.SYNOPSIS
  Line coverage of the kernel's and the modeling layer's assemblies under the tests that don't need native OCCT
  (unit 10A; phase10-plan.md 3.2), written to docs/coverage.md.
.DESCRIPTION
  Runs, each with Microsoft's code coverage (part of Microsoft.NET.Test.Sdk) writing Cobertura: the kernel's unit tests
  (Foundation, ModelingData, ModelingAlgorithms, DataExchange), the Draw-lite tests in their default form (the corpus
  classes sample their cases; OCCTNET_CORPUS_FULL isn't set) and the modeling layer's suite (Build123d.Tests, on
  OCCT.NET); with -WithOracle, the oracle tests too (replayed from their recordings). Then tools/coverage-report.cs
  merges them and writes the report: coverage per OCCT package (the folder under each kernel assembly) and per folder
  of the modeling layer, the packages under the floor listed.
.PARAMETER Results
  The folder for the Cobertura files (a temporary one by default).
.PARAMETER Report
  The report (docs/coverage.md by default).
.PARAMETER Only
  Run only the test projects whose names contain one of these (all by default); the report merges what's in Results.
.PARAMETER WithOracle
  Run the oracle tests too (they replay native OCCT's recorded side, decision 1336).
.PARAMETER BuildArgs
  Extra build properties, e.g. OcctSourceDir and OcctTestDataDir where kernel/.deps lacks them.
#>
param(
  [string] $Results = (Join-Path ([IO.Path]::GetTempPath()) 'kerf-coverage'),
  [string] $Report = (Join-Path $PSScriptRoot '..\docs\coverage.md'),
  [string[]] $Only = @(),
  [switch] $WithOracle,
  [string[]] $BuildArgs = @()
)
$ErrorActionPreference = 'Stop'

$kernel = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$root = (Resolve-Path (Join-Path $kernel '..')).Path
$projects = @(
  'kernel/tests/OcctNet.Foundation.Tests',
  'kernel/tests/OcctNet.ModelingData.Tests',
  'kernel/tests/OcctNet.ModelingAlgorithms.Tests',
  'kernel/tests/OcctNet.DataExchange.Tests',
  'kernel/tests/OcctNet.DrawLite.Tests',
  'modeling/tests/Build123d.Tests'
)
if ($WithOracle) { $projects += 'kernel/tests/OcctNet.Oracle.Tests' }
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

dotnet run (Join-Path $kernel 'tools/coverage-report.cs') -- $Results $Report
