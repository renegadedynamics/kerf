<#
.SYNOPSIS
  Unit 11D's scaling curve: the Bento benchmark's parallel round at each thread count, under each set of runtime settings.
.DESCRIPTION
  Takes the benchmark as tools/bench.ps1 publishes it (<work>/publish/bento), runs it in a fresh process for each cell with
  the settings as DOTNET_ environment variables, and writes one tab-separated row a cell. Each round does the same work, so
  the speed-up is the serial work over the round's time. A cell's figures are the medians of the process's three rounds.
.PARAMETER Exe
  The published benchmark.
.PARAMETER Out
  The .tsv to write.
.PARAMETER Threads
  The thread counts.
.PARAMETER Copies
  How many of each example a round runs.
.PARAMETER Configs
  name=VAR:value,VAR:value entries; the variables get the DOTNET_ prefix. An empty list of variables is the runtime's default.
.EXAMPLE
  kernel/tools/parallel-curve.ps1 -Exe work/publish/bento/Kerf.Kernel.Bento.Benchmarks.exe -Out curve.tsv -Threads 1,24 -Configs 'server=gcServer:1'
#>
param(
  [Parameter(Mandatory)] [string] $Exe,
  [Parameter(Mandatory)] [string] $Out,
  [int[]] $Threads = @(1, 2, 4, 8, 12, 16, 24),
  [int] $Copies = 24,
  [string[]] $Configs = @('workstation concurrent=', 'workstation=gcConcurrent:0', 'server concurrent, DATAS=gcServer:1', 'server, DATAS=gcServer:1,gcConcurrent:0', 'server concurrent, fixed heaps=gcServer:1,GCDynamicAdaptationMode:0', 'server, fixed heaps=gcServer:1,gcConcurrent:0,GCDynamicAdaptationMode:0')
)
$ErrorActionPreference = 'Stop'
"config`tthreads`tround ms`tserial ms`tspeed-up`tpause ms`tcpu ms`tcontended`tpeak MB`talloc MB`tgc0`tgc2`tdiffering" | Set-Content $Out -Encoding utf8
foreach ($config in $Configs) {
  $name, $vars = $config -split '=', 2
  foreach ($t in $Threads) {
    $rows = [IO.Path]::GetTempFileName()
    $info = [Diagnostics.ProcessStartInfo]::new((Resolve-Path $Exe).Path, "9 $Copies $t")
    $info.UseShellExecute = $false
    $info.RedirectStandardOutput = $true
    $info.Environment['KERF_BENCH_ROWS'] = $rows
    foreach ($pair in ($vars -split ',' | Where-Object { $_ })) {
      $k, $v = $pair -split ':', 2
      $info.Environment["DOTNET_$k"] = $v
    }
    $process = [Diagnostics.Process]::Start($info)
    $null = $process.StandardOutput.ReadToEnd()
    $process.WaitForExit()
    if ($process.ExitCode) { throw "$name, $t threads: exit code $($process.ExitCode)" }
    $values = @{}
    foreach ($line in Get-Content $rows) {
      $f = $line -split "`t"
      if ($f[0] -in 'parallel', 'process') { $values[$f[2]] = $f[3] }
    }
    Remove-Item $rows
    $row = @($name, $t, $values['round ms'], $values['serial work ms'], $values['speed-up x'], $values["collector's pause in a round ms"],
      $values['processor time in a round ms'], $values['contended locks in a round count'], $values['peak working set in MB, count'],
      $values['alloc MB'], $values['generation 0 count'], $values['generation 2 count'], $values['result']) -join "`t"
    $row
    Add-Content $Out $row -Encoding utf8
  }
}
