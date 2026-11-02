<#
.SYNOPSIS
  The differential run (unit 9D2): build123d.NET's whole test suite on native OCCT and on OCCT.NET, every shape it
  makes logged as BRep text, then compared shape by shape.
.DESCRIPTION
  build123d.NET logs every new kernel shape its Shape class wraps when BUILD123D_SHAPE_LOG names a folder (ShapeLog.cs,
  one file per test). This script runs build123d.NET's Build123d.Tests with the log on, one test at a time: on the
  native backend NativeRuns times (to see where native varies from run to run), then on OCCT.NET (this checkout). Then
  OCCT.NET's oracle test Build123dDifferentialTests compares the logs: BRep text, then with the Free and Checked flags
  masked, then canonically, and reports each test's first divergence in Logs/report (summary.txt, details.txt and the
  two texts of each first divergence).
.PARAMETER Build123d
  The build123d.NET checkout to test (with ShapeLog.cs).
.PARAMETER NativeRoot
  The build123d.NET checkout whose native build (.deps/install, native/out) the native backend loads; Build123d by
  default. A worktree without a native build can name the main checkout.
.PARAMETER Logs
  The folder for the logs (native1, native2, ..., occtnet) and the report.
.PARAMETER NativeRuns
  How many times the suite runs natively (2 by default).
.PARAMETER OracleArgs
  The oracle project's build properties (OcctSourceDir, OcctTestDataDir, Build123dNetRoot) when not the defaults.
#>
param(
  [string] $Build123d = (Join-Path $PSScriptRoot '..\..\build123d.net'),
  [string] $NativeRoot = $Build123d,
  [string] $Logs = (Join-Path ([IO.Path]::GetTempPath()) 'b3d-differential'),
  [int] $NativeRuns = 2,
  [string[]] $OracleArgs = @()
)
$ErrorActionPreference = 'Stop'

$Build123d = (Resolve-Path $Build123d).Path
$NativeRoot = (Resolve-Path $NativeRoot).Path
$occt = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
New-Item -ItemType Directory -Force $Logs | Out-Null
$Logs = (Resolve-Path $Logs).Path
$tests = Join-Path $Build123d 'tests/Build123d.Tests'
$rid = [System.Runtime.InteropServices.RuntimeInformation]::RuntimeIdentifier
$native = @("-p:OcctNativeDir=$NativeRoot/.deps/install/$rid/", "-p:ShimNativeDir=$NativeRoot/native/out/$rid/")
$occtnet = @('-p:Build123dBackend=OcctNet', "-p:OcctNetRoot=$occt/")

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

for ($i = 1; $i -le $NativeRuns; $i++) { Invoke-Suite $native "native$i" }
Invoke-Suite $occtnet 'occtnet'

$env:B3D_DIFF_LOGS = $Logs
try {
  dotnet test (Join-Path $occt 'tests/OcctNet.Oracle.Tests') -c Release @OracleArgs --filter 'FullyQualifiedName~Build123dDifferentialTests'
}
finally {
  Remove-Item Env:B3D_DIFF_LOGS
}
Get-Content (Join-Path $Logs 'report/summary.txt') -TotalCount 12
