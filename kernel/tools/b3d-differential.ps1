<#
.SYNOPSIS
  The differential run (unit 9D2): build123d.NET's whole test suite on OCCT.NET, every shape it makes logged as BRep
  text, then compared shape by shape with native OCCT's log as recorded (unit 10A, decision 15003).
.DESCRIPTION
  build123d.NET logs every new kernel shape its Shape class wraps when BUILD123D_SHAPE_LOG names a folder (ShapeLog.cs,
  one file per test). This script runs the modeling layer's Build123d.Tests with the log on, one test at a time, on
  OCCT.NET (this repository's kernel); then OCCT.NET's oracle test Build123dDifferentialTests compares the log with
  native's, recorded in tests/OcctNet.Oracle.Tests/Recordings/Build123dDifferential.native.gz: BRep text, then with the
  Free and Checked flags masked, then canonically, and reports each test's first divergence in Logs/report
  (summary.txt, details.txt and the two texts of each first divergence).

  With -Record (while native OCCT exists), the suite also runs on the native backend NativeRuns times (to see where
  native varies from run to run), and the comparison records native's log again before comparing.
.PARAMETER Modeling
  The modeling layer (with ShapeLog.cs): the repository's modeling folder by default.
.PARAMETER NativeRoot
  With -Record: the build123d.NET checkout whose native build (.deps/install, native/out) the native backend loads.
.PARAMETER Logs
  The folder for the logs (occtnet, and with -Record native1, native2, ...) and the report.
.PARAMETER NativeRuns
  With -Record: how many times the suite runs natively (6 by default: fewer leave some of native's variance unseen).
.PARAMETER Record
  Record native's log again (needs native OCCT).
.PARAMETER OracleArgs
  The oracle project's build properties (OcctSourceDir, OcctTestDataDir) when not the defaults.
#>
param(
  [string] $Modeling = (Join-Path $PSScriptRoot '..\..\modeling'),
  [string] $NativeRoot = '',
  [string] $Logs = (Join-Path ([IO.Path]::GetTempPath()) 'b3d-differential'),
  [int] $NativeRuns = 6,
  [switch] $Record,
  [string[]] $OracleArgs = @()
)
$ErrorActionPreference = 'Stop'

$Modeling = (Resolve-Path $Modeling).Path
$occt = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
New-Item -ItemType Directory -Force $Logs | Out-Null
$Logs = (Resolve-Path $Logs).Path
$tests = Join-Path $Modeling 'tests/Build123d.Tests'
$occtnet = @("-p:OcctNetRoot=$occt/")

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

if ($Record) {
  if (-not $NativeRoot) { throw '-Record needs -NativeRoot, a build123d.NET checkout with its native build' }
  $NativeRoot = (Resolve-Path $NativeRoot).Path
  $native = @('-p:Build123dBackend=Native', "-p:Build123dNativeRoot=$NativeRoot/")
  for ($i = 1; $i -le $NativeRuns; $i++) { Invoke-Suite $native "native$i" }
}
Invoke-Suite $occtnet 'occtnet'

$env:B3D_DIFF_LOGS = $Logs
if ($Record) { $env:B3D_DIFF_RECORD = '1' }
try {
  dotnet test (Join-Path $occt 'tests/OcctNet.Oracle.Tests') -c Release @OracleArgs --filter 'FullyQualifiedName~Build123dDifferentialTests'
}
finally {
  Remove-Item Env:B3D_DIFF_LOGS
  Remove-Item Env:B3D_DIFF_RECORD -ErrorAction SilentlyContinue
}
Get-Content (Join-Path $Logs 'report/summary.txt') -TotalCount 12
