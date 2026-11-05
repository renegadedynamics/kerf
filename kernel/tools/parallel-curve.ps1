<#
.SYNOPSIS
  Unit 11D's scaling curve: the Bento benchmark's parallel round on each number of logical processors, under each set of
  runtime settings; or, with -Independent, separate processes, which share only the machine.
.DESCRIPTION
  Takes the benchmark as tools/bench.ps1 publishes it (<work>/publish/bento), runs it in a fresh process for each cell with
  the settings as DOTNET_ environment variables and an affinity mask (one logical processor of each core first, then their
  other halves: on this machine 2n and 2n+1 share a core), and writes one tab-separated row a cell. The mask limits the
  whole process, the kernel's inner loops and the collector as well, which MaxDegreeOfParallelism on the round's loop
  wouldn't. A round does the same work whatever the mask. A cell's figures are the medians of the process's three rounds.
.PARAMETER Exe
  The published benchmark.
.PARAMETER Out
  The .tsv to write.
.PARAMETER Threads
  The numbers of logical processors.
.PARAMETER Copies
  How many of each example a round runs.
.PARAMETER Independent
  Instead of the round: n processes at once, each pinned to one logical processor, doing 40 passes of the serial examples;
  each row gives the median settled recalculation across them.
.PARAMETER Configs
  name=VAR:value,VAR:value entries; the variables get the DOTNET_ prefix unless they start with KERF_. An empty list of
  variables is the runtime's default.
.EXAMPLE
  kernel/tools/parallel-curve.ps1 -Exe work/publish/bento/Kerf.Kernel.Bento.Benchmarks.exe -Out curve.tsv -Threads 1,24 -Configs 'server=gcServer:1'
#>
param(
  [Parameter(Mandatory)] [string] $Exe,
  [Parameter(Mandatory)] [string] $Out,
  [int[]] $Threads = @(1, 2, 4, 8, 12, 16, 24),
  [int] $Copies = 24,
  [switch] $Independent,
  [string[]] $Configs = @('workstation concurrent=', 'workstation=gcConcurrent:0', 'server concurrent, DATAS=gcServer:1', 'server, DATAS=gcServer:1,gcConcurrent:0', 'server concurrent, fixed heaps=gcServer:1,GCDynamicAdaptationMode:0', 'server, fixed heaps=gcServer:1,gcConcurrent:0,GCDynamicAdaptationMode:0')
)
$ErrorActionPreference = 'Stop'
$exePath = (Resolve-Path $Exe).Path

function Get-Bit([int] $i) {
  if ($i -lt 12) { 2 * $i } else { 2 * ($i - 12) + 1 }
}

function Start-Pinned([long] $mask, [string] $arguments, [string] $rows, [string] $vars) {
  $info = [Diagnostics.ProcessStartInfo]::new($env:ComSpec, "/c start `"`" /b /wait /affinity $($mask.ToString('X')) `"$exePath`" $arguments >nul")
  $info.UseShellExecute = $false
  $info.CreateNoWindow = $true
  $info.Environment['KERF_BENCH_ROWS'] = $rows
  foreach ($pair in ($vars -split ',' | Where-Object { $_ })) {
    $k, $v = $pair -split ':', 2
    $info.Environment[$(if ($k -like 'KERF_*') { $k } else { "DOTNET_$k" })] = $v
  }
  [Diagnostics.Process]::Start($info)
}

if ($Independent) {
  "processes`tsettled ms`tprocessor ms" | Set-Content $Out -Encoding utf8
  foreach ($t in $Threads) {
    $started = for ($i = 0; $i -lt $t; $i++) {
      $rows = [IO.Path]::GetTempFileName()
      [pscustomobject] @{ Rows = $rows; Process = Start-Pinned ([long] 1 -shl (Get-Bit $i)) '40 0' $rows 'gcConcurrent:0' }
    }
    $settled = @(); $cpu = @()
    foreach ($s in $started) {
      $s.Process.WaitForExit()
      foreach ($line in Get-Content $s.Rows) {
        $f = $line -split "`t"
        if ($f[1] -eq 'total' -and $f[2] -eq 'settled ms') { $settled += [double] $f[3] }
        if ($f[1] -eq 'total' -and $f[2] -like 'processor time*') { $cpu += [double] $f[3] }
      }
      Remove-Item $s.Rows
    }
    $row = "$t`t$(($settled | Sort-Object)[[int]($settled.Count / 2)])`t$(($cpu | Sort-Object)[[int]($cpu.Count / 2)])"
    $row
    Add-Content $Out $row -Encoding utf8
  }
  return
}

"config`tprocessors`tround ms`tserial ms`tspeed-up`tpause ms`tcpu ms`tcontended`tpeak MB`talloc MB`tgc0`tgc2`tserial cpu ms`tdiffering" | Set-Content $Out -Encoding utf8
foreach ($config in $Configs) {
  $name, $vars = $config -split '=', 2
  foreach ($t in $Threads) {
    [long] $mask = 0
    for ($i = 0; $i -lt $t; $i++) { $mask = $mask -bor ([long] 1 -shl (Get-Bit $i)) }
    $rows = [IO.Path]::GetTempFileName()
    $process = Start-Pinned $mask "9 $Copies" $rows $vars
    $process.WaitForExit()
    if ($process.ExitCode) { throw "$name, $t processors: exit code $($process.ExitCode)" }
    $values = @{}
    foreach ($line in Get-Content $rows) {
      $f = $line -split "`t"
      if ($f[0] -in 'parallel', 'process' -or $f[1] -eq 'total') { $values[$f[2]] = $f[3] }
    }
    Remove-Item $rows
    $row = @($name, $t, $values['round ms'], $values['serial work ms'], $values['speed-up x'], $values["collector's pause in a round ms"],
      $values['processor time in a round ms'], $values['contended locks in a round count'], $values['peak working set in MB, count'],
      $values['alloc MB'], $values['generation 0 count'], $values['generation 2 count'], $values['processor time in a recalculation ms'], $values['result']) -join "`t"
    $row
    Add-Content $Out $row -Encoding utf8
  }
}
