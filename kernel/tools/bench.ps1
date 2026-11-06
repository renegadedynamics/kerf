<#
.SYNOPSIS
  Kerf's benchmark rig: a fixed set on a quiet machine, in fresh processes, in Release, compiled ahead of time as the
  packages are, written up as a dated table, and compared with an earlier run.
.DESCRIPTION
  Refuses a busy machine (see -BusyPercent). Publishes both benchmark projects ReadyToRun for win-x64, as the packages are
  built (tools/pack-nuget.ps1), checks that the five assemblies hold ReadyToRun code, and runs the suites below, each in
  fresh processes; a row's value is the median across a suite's processes.

  Writes <date>-<short commit>.md for people and a .tsv beside it for comparing, with the commit, the machine, whether it
  was quiet and whether ReadyToRun was on at the top. A full run goes to docs/benchmarks and takes about 15 minutes. -Quick
  runs a subset with the same row names, fewer times, in about 5 minutes, for a unit's before-and-after; it writes to the
  work folder unless -Out says otherwise.

  A comparison exits with 1 on a regression past the thresholds: docs/performance.md says how they were chosen from
  measured noise. A result (a volume, a face count, a number of failed runs) that differs at all is a regression too.
  Exit codes: 0 fine; 1 a regression, a failed build or a failed run; 2 the machine is busy.
.PARAMETER Quick
  The subset, for a unit's before-and-after.
.PARAMETER Compare
  An earlier run's .tsv: after the run, print each row's change from it and exit with 1 on a regression.
.PARAMETER Result
  With -Compare: compare this .tsv instead of running anything.
.PARAMETER Force
  Run on a busy machine anyway; the output says it was taken on one.
.PARAMETER BusyPercent
  The total CPU, over the 5 s sampled before starting, above which the machine counts as busy. The default is 10%, about
  2.4 of 24 logical processors: an idle machine with VS Code's C# extension in the background measured about 2%, one
  thread spinning adds about 4%, and a build or a test run is far over. Each suite samples before it starts too, for up to
  10 s to let the previous process's work settle, and the output says so if the machine stayed over.
.PARAMETER Only
  Run only the suites with these names (for working on the rig; the output is partial).
.PARAMETER NoReadyToRun
  Publish IL only, to see what ReadyToRun gives; the output says so.
.PARAMETER Out
  The folder for the .md and the .tsv.
.PARAMETER Work
  A folder for the build and the suites' logs, which is kept; a new temporary one, removed afterwards, if not given.
.PARAMETER HeadlinePercent
  The regression threshold on Bento's total recalculation and first pass, the owner's figures, in percent.
.PARAMETER TotalPercent
  The regression threshold on the times that add up many operations: Bento's totals and opening, the Booleans, the parallel
  round and STEP, in percent of the earlier run's.
.PARAMETER TimePercent
  The regression threshold on any other time, which is one short operation and so noisier.
.PARAMETER TimeFloor
  Any other time's change under this many milliseconds is never a regression.
.PARAMETER MicrosecondFloor
  The same for construction's times, which are in microseconds.
.PARAMETER AllocationPercent
  The regression threshold on allocation, in percent.
.PARAMETER AllocationFloor
  An allocation's change under this many of its unit (MB, or KB for construction's) is never a regression.
.PARAMETER SpeedUpPercent
  The regression threshold on a parallel speed-up (which has no floor), in percent.
.EXAMPLE
  kernel/tools/bench.ps1
.EXAMPLE
  kernel/tools/bench.ps1 -Quick -Compare kernel/docs/benchmarks/2026-11-05-a14650837.tsv
#>
param(
  [switch] $Quick,
  [string] $Compare = '',
  [string] $Result = '',
  [switch] $Force,
  [double] $BusyPercent = 10,
  [string[]] $Only = @(),
  [switch] $NoReadyToRun,
  [string] $Out = '',
  [string] $Work = '',
  [double] $HeadlinePercent = 5,
  [double] $TotalPercent = 15,
  [double] $TimePercent = 60,
  [double] $TimeFloor = 10,
  [double] $MicrosecondFloor = 20,
  [double] $AllocationPercent = 2,
  [double] $AllocationFloor = 0.1,
  [double] $SpeedUpPercent = 15
)
$ErrorActionPreference = 'Stop'
# powershell -File passes -Only a,b as one string
$Only = @($Only | ForEach-Object { $_ -split ',' })
$invariant = [Globalization.CultureInfo]::InvariantCulture
$utf8 = New-Object Text.UTF8Encoding($false)

# Native's frozen figures on Bento's examples (unit 9E's run of the benchmark's native project, 9 passes, 2 November 2026:
# first pass ms, median of passes 2 to 9 ms, allocated by the managed wrappers MB). Native can't be measured again.
$nativeBento = @{
  '2 plate with hole'     = 102.7, 5.4, 0.1
  '3 prismatic solid'     = 16.4, 4.7, 0.1
  '5 working point'       = 8.1, 7.8, 0.1
  '6 point lists'         = 12.8, 7.6, 0.1
  '9 chamfer, fillet'     = 51.7, 12.9, 0.1
  '10 select last, hole'  = 18.4, 17.4, 0.1
  '29 bottle'             = 143.4, 105.6, 0.6
  '31 nesting locations'  = 46.2, 34.2, 0.8
  '32 for-loop'           = 20.5, 17.5, 0.3
  '33 function, for-loop' = 14.8, 15.2, 0.2
  '36 extrude until'      = 486.0, 457.1, 0.2
  'total'                 = 920.9, 685.4, 3.0
}
# The owner's targets (phase11-plan.md section 2): recalculation within 1.2 times native's; allocation half of 11A's baseline
$targetRecalculationMs = 1.2 * 685.4
$targetAllocationMB = 225

function Parse-Number([string] $text) { [double]::Parse($text, $invariant) }
function Format-Number([double] $value) {
  $digits = if ([Math]::Abs($value) -ge 100) { 'F0' } elseif ([Math]::Abs($value) -ge 10) { 'F1' } else { 'F2' }
  $value.ToString($digits, $invariant)
}

function Get-Kind([string] $metric) {
  switch (($metric -split ' ')[-1]) {
    'ms' { 'time' }
    'us' { 'time' }
    'MB' { 'allocation' }
    'KB' { 'allocation' }
    'x' { 'speedup' }
    'count' { 'information' }
    'result' { 'result' }
    default { throw "a metric the rig doesn't know: $metric" }
  }
}

# A .tsv: its header (# key<tab>value) and rows
function Read-Results([string] $path) {
  $header = [ordered]@{}
  $rows = New-Object Collections.ArrayList
  foreach ($line in [IO.File]::ReadAllLines((Resolve-Path $path).Path)) { # .NET's own current folder isn't PowerShell's
    if ($line.StartsWith('# ')) {
      $p = $line.Substring(2).Split("`t")
      $header[$p[0]] = $p[1]
    }
    elseif ($line -and -not $line.StartsWith("group`t")) {
      $p = $line.Split("`t")
      [void] $rows.Add([pscustomobject]@{ Group = $p[0]; Name = $p[1]; Metric = $p[2]; Value = $p[3]; Runs = $p[4]; Spread = $p[5] })
    }
  }
  @{ Header = $header; Rows = $rows }
}

# Prints each row's change; returns the number of regressions
function Compare-Results([string] $beforePath, [string] $afterPath) {
  $before = Read-Results $beforePath
  $after = Read-Results $afterPath
  "Before: $beforePath ($($before.Header['commit']), quiet: $($before.Header['quiet']), ReadyToRun: $($before.Header['readytorun']))"
  "After:  $afterPath ($($after.Header['commit']), quiet: $($after.Header['quiet']), ReadyToRun: $($after.Header['readytorun']))"
  foreach ($key in 'quiet', 'readytorun', 'machine') {
    # Of 'quiet', whether it was, not how much it measured
    if (($before.Header[$key] -split ':')[0] -ne ($after.Header[$key] -split ':')[0]) { "WARNING: the runs differ in '$key', so their times aren't comparable" }
  }
  $afterByKey = @{}
  foreach ($r in $after.Rows) { $afterByKey["$($r.Group)`t$($r.Name)`t$($r.Metric)"] = $r }
  $regressions = 0
  $improvements = 0
  $absent = 0
  $seen = @{}
  foreach ($b in $before.Rows) {
    $key = "$($b.Group)`t$($b.Name)`t$($b.Metric)"
    $seen[$key] = $true
    $a = $afterByKey[$key]
    if (-not $a) { $absent++; continue }
    $kind = Get-Kind $b.Metric
    $label = "$($b.Group) | $($b.Name) | $($b.Metric)"
    if ($kind -eq 'result') {
      if ($a.Value -cne $b.Value) {
        $regressions++
        "REGRESSION  $label : '$($b.Value)' became '$($a.Value)'"
      }
      continue
    }
    $old = Parse-Number $b.Value
    $new = Parse-Number $a.Value
    $change = if ($old -ne 0) { 100 * ($new - $old) / $old } else { 0 }
    $difference = $new - $old
    $flag = '  '
    switch ($kind) {
      'time' {
        $isTotal = $b.Name -eq 'total' -or $b.Group -in 'opening', 'booleans', 'parallel', 'process' -or $b.Group -like 'step*'
        $isHeadline = $b.Group -eq 'bento' -and $b.Name -eq 'total' -and $b.Metric -in 'recalc ms', 'first ms'
        $percent = if ($isHeadline) { $HeadlinePercent } elseif ($isTotal) { $TotalPercent } else { $TimePercent }
        $floor = if ($isTotal) { 0 } elseif ($b.Metric -like '* us' -or $b.Metric -eq 'us') { $MicrosecondFloor } else { $TimeFloor }
        if ($change -gt $percent -and $difference -gt $floor) { $flag = '!!' }
        elseif ($change -lt - $percent -and $difference -lt - $floor) { $flag = '++' }
      }
      'allocation' {
        if ($change -gt $AllocationPercent -and $difference -gt $AllocationFloor) { $flag = '!!' }
        elseif ($change -lt - $AllocationPercent -and $difference -lt - $AllocationFloor) { $flag = '++' }
      }
      'speedup' {
        if ($change -lt - $SpeedUpPercent) { $flag = '!!' }
        elseif ($change -gt $SpeedUpPercent) { $flag = '++' }
      }
    }
    if ($flag -eq '!!') { $regressions++ }
    if ($flag -eq '++') { $improvements++ }
    '{0} {1,10} {2,10} {3,8}  {4}' -f $flag, (Format-Number $old), (Format-Number $new), ($change.ToString('+0.0;-0.0;0.0', $invariant) + '%'), $label
  }
  $new = @($after.Rows | Where-Object { -not $seen["$($_.Group)`t$($_.Name)`t$($_.Metric)"] }).Count
  ''
  "$regressions regressions past the thresholds (Bento's total $HeadlinePercent%, other totals $TotalPercent%, other times $TimePercent% and $TimeFloor ms or $MicrosecondFloor us, allocation $AllocationPercent% and $AllocationFloor MB, speed-up $SpeedUpPercent%), $improvements improvements past them; $absent rows only in the earlier run, $new only in the later"
  # The count last: the caller reads it from the end of the output
  $regressions
}

function Exit-AfterComparing([string] $beforePath, [string] $afterPath) {
  $lines = @(Compare-Results $beforePath $afterPath)
  $lines[0..($lines.Count - 2)] | ForEach-Object { $_ }
  if ($lines[-1] -gt 0) { exit 1 }
  exit 0
}

if ($Result) {
  if (-not $Compare) { throw '-Result needs -Compare' }
  Exit-AfterComparing $Compare $Result
}

# Total CPU over an interval: GetSystemTimes needs no performance counters, whose names depend on the language
Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class KerfBenchCpu {
  [DllImport("kernel32.dll", SetLastError = true)]
  private static extern bool GetSystemTimes(out long idle, out long kernel, out long user);
  public static double Percent(int milliseconds) {
    long i0, k0, u0, i1, k1, u1;
    GetSystemTimes(out i0, out k0, out u0);
    System.Threading.Thread.Sleep(milliseconds);
    GetSystemTimes(out i1, out k1, out u1);
    long total = (k1 - k0) + (u1 - u0); // kernel time includes idle
    return total > 0 ? 100.0 * (total - (i1 - i0)) / total : 0;
  }
}
'@

# Whether an assembly holds ReadyToRun code: as tools/pack-nuget.ps1
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

$startLoad = [KerfBenchCpu]::Percent(5000)
$startText = $startLoad.ToString('F1', $invariant)
$isBusy = $startLoad -gt $BusyPercent
if ($isBusy -and -not $Force) {
  Write-Host "The machine is busy: $startText% total CPU over 5 s, above the $BusyPercent% allowed. Wait for it, or use -Force to mark the output as taken on a busy machine."
  exit 2
}
"Total CPU before starting: $startText%"

$kernel = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$keepWork = [bool] $Work
if (-not $Work) { $Work = Join-Path ([IO.Path]::GetTempPath()) ('kerf-bench-' + [guid]::NewGuid().ToString('N')) }
New-Item -ItemType Directory -Force $Work | Out-Null
$Work = (Resolve-Path $Work).Path

$commit = (git -C $kernel rev-parse --short HEAD).Trim()
$isDirty = [bool] (git -C $kernel status --porcelain)
$date = (Get-Date).ToString('yyyy-MM-dd', $invariant)
$overallClock = [Diagnostics.Stopwatch]::StartNew()

# Build. Publishing an executable with ReadyToRun compiles everything it references, as the packages' assemblies are
$publishes = @(
  @{ Name = 'kernel'; Project = 'Kerf.Kernel.Benchmarks' },
  @{ Name = 'bento'; Project = 'Kerf.Kernel.Bento.Benchmarks' }
)
$readyToRun = if ($NoReadyToRun) { 'false' } else { 'true' }
$exes = @{}
foreach ($p in $publishes) {
  $to = Join-Path $Work "publish/$($p.Name)"
  $log = Join-Path $Work "publish-$($p.Name).log"
  "Publishing $($p.Project)"
  dotnet publish (Join-Path $kernel "benchmarks/$($p.Project)/$($p.Project).csproj") -c Release -nologo -v q -r win-x64 --self-contained false "-p:PublishReadyToRun=$readyToRun" -o $to *> $log
  if ($LASTEXITCODE) { Get-Content $log | Select-Object -Last 20; throw "publishing $($p.Project) failed: $log" }
  $exes[$p.Name] = Join-Path $to "$($p.Project).exe"
  if (-not $NoReadyToRun) {
    $assemblies = @('Kerf.Kernel.Foundation', 'Kerf.Kernel.ModelingData', 'Kerf.Kernel.ModelingAlgorithms')
    $assemblies += if ($p.Name -eq 'bento') { 'Kerf.Kernel.DataExchange', 'Kerf.Modeling' } else { 'Kerf.Kernel.DataExchange' }
    foreach ($a in $assemblies) {
      if (-not (Test-ReadyToRun (Join-Path $to "$a.dll"))) { throw "$a in $to isn't ReadyToRun" }
    }
  }
}
$buildSeconds = [int] $overallClock.Elapsed.TotalSeconds

# The fixed set: how many fresh processes in a full run, and in -Quick (0: not in it)
$suites = @(
  @{ Name = 'bento'; Exe = 'bento'; Args = @('20', '0'); Full = 5; Quick = 3 },
  @{ Name = 'parallel'; Exe = 'bento'; Args = @('9', "$([Environment]::ProcessorCount)"); Full = 3; Quick = 1 },
  # The same under the settings a host should use (docs/performance.md, unit 11D): its rows' groups start with "host "
  @{ Name = 'parallel-host'; Exe = 'bento'; Args = @('9', "$([Environment]::ProcessorCount)"); Full = 3; Quick = 1; Prefix = 'host '
    Env = @{ DOTNET_gcServer = '1'; DOTNET_TieredPGO = '0'; DOTNET_TC_CallCountingDelayMs = '0' } },
  @{ Name = 'images'; Exe = 'bento'; Args = @('images', '9'); Full = 3; Quick = 1 },
  @{ Name = 'meshing'; Exe = 'kernel'; Args = @('quick'); Full = 3; Quick = 1 },
  @{ Name = 'construction'; Exe = 'kernel'; Args = @('construction'); Full = 3; Quick = 1 },
  @{ Name = 'features'; Exe = 'kernel'; Args = @('features'); Full = 3; Quick = 1 },
  @{ Name = 'booleans-simple'; Exe = 'kernel'; Args = @('booleans', 'simple'); Full = 2; Quick = 1 },
  @{ Name = 'booleans-complex'; Exe = 'kernel'; Args = @('booleans', 'complex'); Full = 2; Quick = 0 },
  @{ Name = 'booleans-profile'; Exe = 'kernel'; Args = @('booleans-profile'); Full = 1; Quick = 0 },
  @{ Name = 'step-parser'; Exe = 'kernel'; Args = @('stepfile'); Full = 3; Quick = 1 },
  @{ Name = 'step-and-healing'; Exe = 'kernel'; Args = @('healing'); Full = 2; Quick = 0 }
)

$rows = New-Object Collections.ArrayList
$busySuites = New-Object Collections.ArrayList
$suiteNotes = New-Object Collections.ArrayList
$logs = Join-Path $Work 'logs'
New-Item -ItemType Directory -Force $logs | Out-Null
foreach ($suite in $suites) {
  if ($Only -and $Only -notcontains $suite.Name) { continue }
  $times = if ($Quick) { $suite.Quick } else { $suite.Full }
  if ($times -eq 0) { continue }
  # A build's servers and the last process's exit take a moment to settle
  $load = [KerfBenchCpu]::Percent(2000)
  for ($wait = 0; $wait -lt 4 -and $load -gt $BusyPercent; $wait++) { $load = [KerfBenchCpu]::Percent(2000) }
  if ($load -gt $BusyPercent) { [void] $busySuites.Add("$($suite.Name) ($($load.ToString('F0', $invariant))%)") }
  $clock = [Diagnostics.Stopwatch]::StartNew()
  for ($i = 1; $i -le $times; $i++) {
    "$($suite.Name) $i of $times"
    $rowFile = Join-Path $logs "$($suite.Name)-$i.tsv"
    if (Test-Path $rowFile) { Remove-Item $rowFile }
    $env:KERF_BENCH_ROWS = $rowFile
    if ($suite.Env) { foreach ($k in $suite.Env.Keys) { Set-Item "Env:\$k" $suite.Env[$k] } }
    $process = Start-Process -FilePath $exes[$suite.Exe] -ArgumentList $suite.Args -Wait -PassThru -NoNewWindow `
      -RedirectStandardOutput (Join-Path $logs "$($suite.Name)-$i.out") -RedirectStandardError (Join-Path $logs "$($suite.Name)-$i.err")
    Remove-Item Env:\KERF_BENCH_ROWS
    if ($suite.Env) { foreach ($k in $suite.Env.Keys) { Remove-Item "Env:\$k" } }
    if ($process.ExitCode) { throw "$($suite.Name) failed with exit code $($process.ExitCode): see $logs" }
    foreach ($line in [IO.File]::ReadAllLines($rowFile)) {
      $p = $line.Split("`t")
      [void] $rows.Add([pscustomobject]@{ Group = "$($suite.Prefix)$($p[0])"; Name = $p[1]; Metric = $p[2]; Value = $p[3] })
    }
  }
  [void] $suiteNotes.Add("$($suite.Name): $times x $(($suite.Args) -join ' '), $([int] $clock.Elapsed.TotalSeconds) s")
}

# Each row's median across its processes; a result has to be the same in all of them
$aggregated = New-Object Collections.ArrayList
$byKey = [ordered]@{}
foreach ($r in $rows) {
  $key = "$($r.Group)`t$($r.Name)`t$($r.Metric)"
  if (-not $byKey.Contains($key)) { $byKey[$key] = New-Object Collections.ArrayList }
  [void] $byKey[$key].Add($r.Value)
}
$unstable = New-Object Collections.ArrayList
foreach ($key in $byKey.Keys) {
  $p = $key.Split("`t")
  $values = $byKey[$key]
  if ((Get-Kind $p[2]) -eq 'result') {
    $distinct = @($values | Sort-Object -Unique)
    if ($distinct.Count -gt 1) { [void] $unstable.Add("$key : $($distinct -join ' / ')") }
    $value = $values[0]
    $spread = ''
  }
  else {
    $numbers = @($values | ForEach-Object { Parse-Number $_ } | Sort-Object)
    # The middle one, the lower of two: a figure that was measured, and noise only ever adds to a time
    $median = $numbers[[int][Math]::Floor(($numbers.Count - 1) / 2)]
    $value = $median.ToString('F3', $invariant)
    $spread = if ($median -ne 0) { (100 * ($numbers[-1] - $numbers[0]) / $median).ToString('F1', $invariant) } else { '0.0' }
  }
  [void] $aggregated.Add([pscustomobject]@{ Group = $p[0]; Name = $p[1]; Metric = $p[2]; Value = $value; Runs = $values.Count; Spread = $spread })
}

# The header
$cpu = (Get-CimInstance Win32_Processor | Select-Object -First 1).Name.Trim()
$memoryGB = [Math]::Round((Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory / 1GB)
$os = (Get-CimInstance Win32_OperatingSystem).Caption
$machine = "$cpu, $([Environment]::ProcessorCount) logical processors, $memoryGB GB, $os"
$quiet = if ($isBusy) { "no: $startText% total CPU at the start, run with -Force" }
elseif ($busySuites.Count) { "no: over $BusyPercent% before $($busySuites -join ', ')" }
else { "yes: $startText% total CPU at the start, none over $BusyPercent% before any suite" }
$header = [ordered]@{
  date       = $date
  commit     = $commit + $(if ($isDirty) { ' with uncommitted changes' } else { '' })
  machine    = $machine
  runtime    = "dotnet $((dotnet --version).Trim())"
  quiet      = $quiet
  readytorun = if ($NoReadyToRun) { 'no' } else { 'yes' }
  set        = $(if ($Only) { "only $($Only -join ', ')" } elseif ($Quick) { 'quick' } else { 'full' })
}

if (-not $Out) { $Out = if ($Quick -or $Only) { $Work } else { Join-Path $kernel 'docs/benchmarks' } }
New-Item -ItemType Directory -Force $Out | Out-Null
$Out = (Resolve-Path $Out).Path
$baseName = "$date-$commit" + $(if ($Quick) { '-quick' } else { '' })
$tsvPath = Join-Path $Out "$baseName.tsv"
$mdPath = Join-Path $Out "$baseName.md"

$tsv = New-Object Text.StringBuilder
foreach ($k in $header.Keys) { [void] $tsv.Append("# $k`t$($header[$k])`n") }
[void] $tsv.Append("group`tname`tmetric`tvalue`truns`tspread%`n")
foreach ($r in $aggregated) { [void] $tsv.Append("$($r.Group)`t$($r.Name)`t$($r.Metric)`t$($r.Value)`t$($r.Runs)`t$($r.Spread)`n") }
[IO.File]::WriteAllText($tsvPath, $tsv.ToString(), $utf8)

# For people: a table a group, a row a name, a column a metric, with native's frozen figures beside Bento's
function Get-Row([string] $group, [string] $name, [string] $metric) {
  $aggregated | Where-Object { $_.Group -eq $group -and $_.Name -eq $name -and $_.Metric -eq $metric } | Select-Object -First 1
}
$md = New-Object Text.StringBuilder
function Add-Line([string] $line = '') { [void] $md.Append("$line`n") }
Add-Line "# Benchmarks: $date, $commit"
Add-Line
Add-Line "Written by ``kernel/tools/bench.ps1``; [performance.md](../performance.md) has how to read it."
Add-Line
foreach ($k in $header.Keys) { Add-Line "- **${k}:** $($header[$k])" }
Add-Line "- **time:** $([int] $overallClock.Elapsed.TotalSeconds) s, of which $buildSeconds s building"
Add-Line "- **suites:** $($suiteNotes -join '; ')"
if ($unstable.Count) {
  Add-Line "- **RESULTS DIFFERED BETWEEN PROCESSES:** $($unstable -join '; ')"
}
Add-Line
$total = Get-Row 'bento' 'total' 'recalc ms'
$allocated = Get-Row 'bento' 'total' 'alloc MB'
$first = Get-Row 'bento' 'total' 'first ms'
if ($total) {
  $recalculation = Parse-Number $total.Value
  Add-Line '## Against the targets'
  Add-Line
  Add-Line '| | Native (frozen) | Kerf | Kerf / native | Target |'
  Add-Line '|---|---:|---:|---:|---:|'
  Add-Line "| Recalculation (median of passes 2 to 9, summed), ms | 685 | $(Format-Number $recalculation) | $((Format-Number ($recalculation / 685.4)))x | $(Format-Number $targetRecalculationMs) |"
  Add-Line "| Opening (first pass, summed), ms | 921 | $(Format-Number (Parse-Number $first.Value)) | $((Format-Number ((Parse-Number $first.Value) / 920.9)))x | |"
  $targetText = if ($targetAllocationMB -gt 0) { Format-Number $targetAllocationMB } else { 'half the baseline' }
  Add-Line "| Allocated per recalculation, MB | 3 | $(Format-Number (Parse-Number $allocated.Value)) | | $targetText |"
  Add-Line
}
$groups = @($aggregated | ForEach-Object { $_.Group } | Select-Object -Unique)
foreach ($group in $groups) {
  $inGroup = @($aggregated | Where-Object { $_.Group -eq $group })
  $metrics = @($inGroup | ForEach-Object { $_.Metric } | Select-Object -Unique)
  $names = @($inGroup | ForEach-Object { $_.Name } | Select-Object -Unique)
  Add-Line "## $group"
  Add-Line
  $isBento = $group -eq 'bento'
  $columns = @()
  foreach ($m in $metrics) {
    $columns += $m
    if ($isBento -and $m -eq 'first ms') { $columns += 'native first' }
    if ($isBento -and $m -eq 'recalc ms') { $columns += 'native recalc', 'ratio' }
    if ($isBento -and $m -eq 'alloc MB') { $columns += 'native MB' }
  }
  Add-Line "| | $($columns -join ' | ') |"
  Add-Line "|---|$(($columns | ForEach-Object { if ($_ -eq 'result') { '---' } else { '---:' } }) -join '|')|"
  foreach ($name in $names) {
    $cells = @()
    $native = if ($isBento) { $nativeBento[$name] } else { $null }
    foreach ($m in $metrics) {
      $r = $inGroup | Where-Object { $_.Name -eq $name -and $_.Metric -eq $m } | Select-Object -First 1
      $text = if (-not $r) { '' } elseif ((Get-Kind $m) -eq 'result') { $r.Value } else { Format-Number (Parse-Number $r.Value) }
      $cells += $text
      if ($isBento -and $m -eq 'first ms') { $cells += $(if ($native) { Format-Number $native[0] } else { '' }) }
      if ($isBento -and $m -eq 'recalc ms') {
        $cells += $(if ($native) { Format-Number $native[1] } else { '' })
        $cells += $(if ($native -and $r) { (Format-Number ((Parse-Number $r.Value) / $native[1])) + 'x' } else { '' })
      }
      if ($isBento -and $m -eq 'alloc MB') { $cells += $(if ($native) { Format-Number $native[2] } else { '' }) }
    }
    Add-Line "| $name | $($cells -join ' | ') |"
  }
  Add-Line
}
[IO.File]::WriteAllText($mdPath, $md.ToString(), $utf8)

"Written: $mdPath"
"Written: $tsvPath"
"Took $([int] $overallClock.Elapsed.TotalSeconds) s; quiet: $quiet"
if ($total) {
  "Recalculation $(Format-Number $recalculation) ms (native 685, target $(Format-Number $targetRecalculationMs)); opening $(Format-Number (Parse-Number $first.Value)) ms (native 921); allocation $(Format-Number (Parse-Number $allocated.Value)) MB"
}
if (-not $keepWork) {
  # The results are all that's kept, unless they were written there
  if ($Out -eq $Work) { Get-ChildItem $Work -Exclude "$baseName.*" | Remove-Item -Recurse -Force }
  else { Remove-Item $Work -Recurse -Force }
}
if ($unstable.Count) {
  Write-Host "A result differed between processes: $($unstable -join '; ')"
  exit 1
}
if ($Compare) {
  Exit-AfterComparing $Compare $tsvPath
}
exit 0
