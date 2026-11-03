<#
.SYNOPSIS
  The differential run (unit 9D2): build123d.NET's whole test suite on OCCT.NET, every shape it makes logged as BRep
  text, then compared shape by shape with native OCCT's log as recorded (unit 10A, decision 1338).
.DESCRIPTION
  build123d.NET logs every new kernel shape its Shape class wraps when BUILD123D_SHAPE_LOG names a folder (ShapeLog.cs,
  one file per test). This script runs the modeling layer's Kerf.Modeling.Tests with the log on, one test at a time, on
  OCCT.NET (this repository's kernel); then OCCT.NET's oracle test Build123dDifferentialTests compares the log with
  native's, recorded in tests/Kerf.Kernel.Oracle.Tests/Recordings/Build123dDifferential.native.gz: BRep text, then with the
  Free and Checked flags masked, then canonically, and reports each test's first divergence in Logs/report
  (summary.txt, details.txt and the two texts of each first divergence). Native's log is frozen: native OCCT went in
  unit 10B, so it can't be recorded again.
.PARAMETER Modeling
  The modeling layer (with ShapeLog.cs): the repository's modeling folder by default.
.PARAMETER Logs
  The folder for the log (occtnet) and the report.
.PARAMETER OracleArgs
  The oracle project's build properties (OcctSourceDir, OcctTestDataDir) when not the defaults.
#>
param(
  [string] $Modeling = (Join-Path $PSScriptRoot '..\..\modeling'),
  [string] $Logs = (Join-Path ([IO.Path]::GetTempPath()) 'b3d-differential'),
  [string[]] $OracleArgs = @()
)
$ErrorActionPreference = 'Stop'

$Modeling = (Resolve-Path $Modeling).Path
$occt = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
New-Item -ItemType Directory -Force $Logs | Out-Null
$Logs = (Resolve-Path $Logs).Path
$tests = Join-Path $Modeling 'tests/Kerf.Modeling.Tests'
$occtnet = @("-p:KerfKernelRoot=$occt/")

function Invoke-Suite([string[]] $backend, [string] $log) {
  dotnet build $tests -c Release -nologo -v q @backend
  if ($LASTEXITCODE) { throw "building build123d.NET's tests failed" }
  Remove-Item -Recurse -Force (Join-Path $Logs $log) -ErrorAction SilentlyContinue
  $env:BUILD123D_SHAPE_LOG = Join-Path $Logs $log
  try {
    # one test at a time: the log keeps one test's registry of shapes at once
    dotnet test $tests -c Release --no-build @backend -- xUnit.ParallelizeTestCollections=false
  }
  finally {
    Remove-Item Env:BUILD123D_SHAPE_LOG
  }
  # a failing test logs what it made before it failed: the comparison shows where
}

Invoke-Suite $occtnet 'occtnet'

$env:B3D_DIFF_LOGS = $Logs
try {
  dotnet test (Join-Path $occt 'tests/Kerf.Kernel.Oracle.Tests') -c Release @OracleArgs --filter 'FullyQualifiedName~Build123dDifferentialTests'
}
finally {
  Remove-Item Env:B3D_DIFF_LOGS
}
Get-Content (Join-Path $Logs 'report/summary.txt') -TotalCount 12
