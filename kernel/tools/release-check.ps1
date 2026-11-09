<#
.SYNOPSIS
  The release check: users' cases on the last release's packages from nuget.org and on the candidate, warm, interleaved,
  median of 9. Exits non-zero, naming each case, on one more than 20% slower or giving a different result.
.DESCRIPTION
  Unit 13C (decision 2015). 0.4.0 shipped two slowdowns the rig couldn't see, because it had no part shaped like the
  user's: a small part fused onto a large one, 4 to 5 times 0.3.0's time, and a chamfer collapsing tight corners, 30 to
  80 times a plain one. The cases in benchmarks/Kerf.UserCases are users' parts, each with a fixed result; this script
  runs them on both sides and stops the release on any that is slower or different, until it's explained.

  Each side is the cases' program from this checkout, copied out of the tree and built against that side's Kerf:
  the last release's packages from nuget.org (-Last), and the candidate: its packages from a folder (-CandidatePackages,
  what tools/pack-nuget.ps1 wrote, which is what would ship), a version on nuget.org (-Candidate), or else this
  checkout's tree as it stands. Each package side restores into a package folder of its own, so no cached copy stands
  in, and the packages' SHA-512 hashes are printed. Both are published ReadyToRun for win-x64, as the packages are built.

  Both programs stay running for the whole check, so every run is warm. Each case is run -WarmUp times on each side, then
  -Runs times, the two sides alternating case by case and taking turns to go first, so a machine's load falls on both
  alike; a case's figure is the median of its runs (the lower middle). Its operands are read untimed on its first run.

  A case stops the release when, on the candidate:
  - its median time is more than -SlowerPercent over the last release's;
  - its result (each shape's volume to -VolumeTolerance relative, validity and face count; or its failure) differs from
    the last release's, or from the case's fixed result in the cases' Expected.cs, or from one run to the next.
  A case the last release can't run (it uses a later API) has no baseline: it's checked against its fixed result only,
  and named. -Explained lists cases whose stop has been explained (a decision says why): they're shown, and don't stop it.

  Exit codes: 0, nothing stops the release; 1, a case stops it, or the check couldn't run.
.PARAMETER Last
  The last release's version on nuget.org. By default the newest version there that isn't a prerelease.
.PARAMETER Candidate
  The candidate as a version on nuget.org (to check a published release against an earlier one).
.PARAMETER CandidatePackages
  The candidate as packages in a folder: tools/pack-nuget.ps1's output. Without this or -Candidate, this checkout's tree.
.PARAMETER Runs
  Timed runs of each case on each side.
.PARAMETER WarmUp
  Untimed runs of each case on each side first.
.PARAMETER SlowerPercent
  How much slower than the last release, in percent of its median, stops the release.
.PARAMETER VolumeTolerance
  How far a volume may move, relative, and still be the same result. The cases' own test uses 1e-6 too.
.PARAMETER Explained
  Cases whose stop is explained: reported, but not a reason to exit non-zero.
.PARAMETER Only
  Run only these cases.
.PARAMETER Work
  A folder for the copies, the packages and the builds, which is kept; a new temporary one, removed afterwards, if not given.
.PARAMETER Out
  Also write the report to this file.
.PARAMETER RunTimeoutSeconds
  How long one run of a case may take before the check stops it as a hang.
.EXAMPLE
  kernel/tools/release-check.ps1 -CandidatePackages ../nuget-release/0.5.0
.EXAMPLE
  kernel/tools/release-check.ps1 -Last 0.3.0 -Candidate 0.4.0
#>
param(
  [string] $Last = '',
  [string] $Candidate = '',
  [string] $CandidatePackages = '',
  [int] $Runs = 9,
  [int] $WarmUp = 2,
  [double] $SlowerPercent = 20,
  [double] $VolumeTolerance = 1e-6,
  [string[]] $Explained = @(),
  [string[]] $Only = @(),
  [string] $Work = '',
  [string] $Out = '',
  [int] $RunTimeoutSeconds = 600
)
$ErrorActionPreference = 'Stop'
# powershell -File passes -Only a,b as one string
$Explained = @($Explained | ForEach-Object { $_ -split ',' } | Where-Object { $_ })
$Only = @($Only | ForEach-Object { $_ -split ',' } | Where-Object { $_ })
$invariant = [Globalization.CultureInfo]::InvariantCulture
$nugetOrg = 'https://api.nuget.org/v3/index.json'
[Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12

$report = New-Object Collections.ArrayList
function Say([string] $line = '') {
  [void] $report.Add($line)
  Write-Host $line
}

# Total CPU over an interval, as tools/bench.ps1 reads it: the check runs on a loaded machine too, and says how loaded
Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class KerfReleaseCheckCpu {
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

if ($Candidate -and $CandidatePackages) { throw 'Give -Candidate or -CandidatePackages, not both' }
if (-not $Last) {
  $listed = Invoke-RestMethod 'https://api.nuget.org/v3-flatcontainer/kerf.modeling/index.json'
  $Last = @($listed.versions | Where-Object { $_ -notmatch '-' } | Sort-Object { [Version] $_ })[-1]
  if (-not $Last) { throw "nuget.org lists no release of Kerf.Modeling" }
}
$kernel = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$repo = (Resolve-Path (Join-Path $kernel '..')).Path
$cases = Join-Path $kernel 'benchmarks/Kerf.UserCases'
$keepWork = [bool] $Work
if (-not $Work) { $Work = Join-Path ([IO.Path]::GetTempPath()) ('kerf-release-check-' + [guid]::NewGuid().ToString('N')) }
New-Item -ItemType Directory -Force $Work | Out-Null
$Work = (Resolve-Path $Work).Path
# Nothing above the copies is imported into their builds
'<Project />' | Set-Content -Encoding utf8 (Join-Path $Work 'Directory.Build.props')
'<Project />' | Set-Content -Encoding utf8 (Join-Path $Work 'Directory.Build.targets')

$candidateText = if ($CandidatePackages) {
  $CandidatePackages = (Resolve-Path $CandidatePackages).Path
  $package = Get-ChildItem $CandidatePackages -Filter 'Kerf.Modeling.*.nupkg' | Where-Object { $_.Name -notmatch '\.symbols\.nupkg$' } | Select-Object -First 1
  if (-not $package) { throw "no Kerf.Modeling package in $CandidatePackages" }
  $candidateVersion = $package.Name.Substring('Kerf.Modeling.'.Length) -replace '\.nupkg$', ''
  "Kerf.Modeling $candidateVersion from $CandidatePackages"
}
elseif ($Candidate) { $candidateVersion = $Candidate; "Kerf.Modeling $Candidate from nuget.org" }
else {
  $commit = (git -C $repo rev-parse --short HEAD).Trim()
  $dirty = if (git -C $repo status --porcelain) { ' with uncommitted changes' } else { '' }
  "this checkout's tree, $commit$dirty"
}

# A package folder's paths can be longer than Windows PowerShell's Remove-Item manages
function Remove-Folder([string] $folder) {
  cmd /c rd /s /q "\\?\$folder"
  if (Test-Path $folder) { throw "couldn't remove $folder" }
}

# A side: the cases' program copied out of the tree, restored and published against that side's Kerf
function New-Side([string] $name, [string] $version, [string] $localPackages) {
  $dir = Join-Path $Work $name
  if (Test-Path $dir) { Remove-Folder $dir }
  New-Item -ItemType Directory -Force $dir | Out-Null
  Get-ChildItem $cases -Exclude 'bin', 'obj' | Copy-Item -Destination $dir -Recurse
  $packages = Join-Path $dir 'packages'
  $local = if ($localPackages) { "    <add key=`"candidate`" value=`"$localPackages`" />`n" } else { '' }
  # Kerf's packages from one source only: a local folder may hold a version nuget.org has too
  $mapping = if ($localPackages) {
    "  <packageSourceMapping>`n    <packageSource key=`"candidate`"><package pattern=`"Kerf.*`" /></packageSource>`n    <packageSource key=`"nuget.org`"><package pattern=`"*`" /></packageSource>`n  </packageSourceMapping>`n"
  } else { '' }
  $config = if ($version) { "  <config><add key=`"globalPackagesFolder`" value=`"$packages`" /></config>`n" } else { '' }
  "<?xml version=`"1.0`" encoding=`"utf-8`"?>`n<configuration>`n$config  <packageSources>`n    <clear />`n$local    <add key=`"nuget.org`" value=`"$nugetOrg`" />`n  </packageSources>`n$mapping</configuration>`n" |
    Set-Content -Encoding utf8 (Join-Path $dir 'nuget.config')
  $properties = @(if ($version) { "-p:KerfPackageVersion=$version" }
    else {
      # This tree, with the packages' crossgen settings (modeling/Directory.Build.props)
      "-p:KerfModelingRoot=$repo/modeling/"
      '-p:PublishReadyToRunCrossgen2ExtraArgs=--opt-cross-module:*%3B--Ot'
    })
  $bin = Join-Path $dir 'bin-published'
  $log = Join-Path $dir 'publish.log'
  dotnet publish (Join-Path $dir 'Kerf.UserCases.csproj') -c Release -nologo -v q -r win-x64 --self-contained false `
    -p:PublishReadyToRun=true @properties -o $bin *> $log
  if ($LASTEXITCODE) { Get-Content $log | Select-Object -Last 30; throw "building the cases for $name failed: $log" }
  foreach ($a in 'Kerf.Kernel.Foundation', 'Kerf.Kernel.ModelingData', 'Kerf.Kernel.ModelingAlgorithms', 'Kerf.Kernel.DataExchange', 'Kerf.Modeling') {
    if (-not (Test-ReadyToRun (Join-Path $bin "$a.dll"))) { throw "$a for $name isn't ReadyToRun" }
  }
  $hashes = @()
  if ($version) {
    foreach ($sha in Get-ChildItem (Join-Path $packages 'kerf.*') -Recurse -Filter '*.nupkg.sha512') {
      $hashes += "$($sha.Directory.Parent.Name) $($sha.Directory.Name): SHA-512 $((Get-Content $sha.FullName -Raw).Trim())"
    }
  }
  @{ Name = $name; Exe = (Join-Path $bin 'Kerf.UserCases.exe'); Hashes = $hashes }
}

# A running copy of a side's program, which answers "run <case>" with a line
function Start-Runner($side) {
  $info = New-Object Diagnostics.ProcessStartInfo
  $info.FileName = $side.Exe
  $info.Arguments = 'serve'
  $info.UseShellExecute = $false
  $info.RedirectStandardInput = $true
  $info.RedirectStandardOutput = $true
  $info.StandardOutputEncoding = New-Object Text.UTF8Encoding($false)
  $info.WorkingDirectory = Split-Path $side.Exe
  $process = [Diagnostics.Process]::Start($info)
  $side.Process = $process
  $ready = Read-Answer $side 120
  $p = $ready.Split("`t")
  if ($p[0] -ne 'ready') { throw "$($side.Name) didn't start: $ready" }
  $side.Version = $p[1]
  $side.Runtime = $p[2]
}

function Read-Answer($side, [int] $seconds) {
  $pending = $side.Process.StandardOutput.ReadLineAsync()
  if (-not $pending.Wait([TimeSpan]::FromSeconds($seconds))) {
    # The check started this process and holds it: stopping it can't touch anything else
    $side.Process.Kill()
    throw "$($side.Name) gave no answer in $seconds s: a hang, or a case far slower than it should be"
  }
  if ($null -eq $pending.Result) { throw "$($side.Name) stopped: exit code $($side.Process.ExitCode)" }
  $pending.Result
}

function Invoke-Case($side, [string] $case) {
  $side.Process.StandardInput.WriteLine("run $case")
  $side.Process.StandardInput.Flush()
  $p = (Read-Answer $side $RunTimeoutSeconds).Split("`t")
  if ($p[0] -ne $case) { throw "$($side.Name) answered for '$($p[0])', not $case" }
  [pscustomobject]@{ Ms = [double]::Parse($p[1], $invariant); Bytes = [double]::Parse($p[2], $invariant); IsFixed = $p[3] -eq 'yes'; Result = $p[4] }
}

# A result line as shapes: absent, failed: why, or label=volume,valid,faces;...
function Read-Result([string] $text) {
  if ($text -eq 'absent') { return @{ Kind = 'absent' } }
  if ($text.StartsWith('failed')) { return @{ Kind = 'failed'; Text = $text } }
  $shapes = @(foreach ($item in $text.Split(';')) {
      $label, $rest = $item.Split('=', 2)
      $v = $rest.Split(',')
      @{ Label = $label; Volume = [double]::Parse($v[0], $invariant); Valid = $v[1]; Faces = $v[2] }
    })
  @{ Kind = 'shapes'; Shapes = $shapes }
}

# Whether two results are the same: as the cases' CaseResult.Matches
function Test-SameResult([string] $a, [string] $b) {
  $x = Read-Result $a
  $y = Read-Result $b
  if ($x.Kind -ne $y.Kind) { return $false }
  if ($x.Kind -ne 'shapes') { return $true }
  if ($x.Shapes.Count -ne $y.Shapes.Count) { return $false }
  for ($i = 0; $i -lt $x.Shapes.Count; $i++) {
    $s = $x.Shapes[$i]
    $t = $y.Shapes[$i]
    if ($s.Label -ne $t.Label -or $s.Valid -ne $t.Valid -or $s.Faces -ne $t.Faces) { return $false }
    if ([Math]::Abs($s.Volume - $t.Volume) -gt $VolumeTolerance * [Math]::Max([Math]::Abs($s.Volume), [Math]::Abs($t.Volume))) { return $false }
  }
  $true
}

# A result in a few words: its total volume, faces and any invalid shape, or its failure
function Format-Result([string] $text) {
  $r = Read-Result $text
  switch ($r.Kind) {
    'absent' { 'not in this version' }
    'failed' { $text }
    default {
      $volume = ($r.Shapes | ForEach-Object { $_.Volume } | Measure-Object -Sum).Sum
      $faces = ($r.Shapes | ForEach-Object { [int] $_.Faces } | Measure-Object -Sum).Sum
      $invalid = @($r.Shapes | Where-Object { $_.Valid -ne 'valid' }).Count
      $shapes = if ($r.Shapes.Count -gt 1) { "$($r.Shapes.Count) shapes, " } else { '' }
      "$shapes" + 'volume ' + $volume.ToString('F3', $invariant) + ", $faces faces" + $(if ($invalid) { ", $invalid INVALID" } else { '' })
    }
  }
}

function Get-Median([double[]] $values) {
  $sorted = @($values | Sort-Object)
  $sorted[[int][Math]::Floor(($sorted.Count - 1) / 2)]
}

$sides = @()
try {
  $startLoad = [KerfReleaseCheckCpu]::Percent(5000)
  Say "Release check: users' cases, warm, interleaved, median of $Runs"
  Say "Last release: Kerf.Modeling $Last from nuget.org"
  Say "Candidate:    $candidateText"
  Say "Building both sides in $Work"
  $lastSide = New-Side 'last' $Last ''
  $candidateSide = if ($CandidatePackages) { New-Side 'candidate' $candidateVersion $CandidatePackages }
  elseif ($Candidate) { New-Side 'candidate' $Candidate '' }
  else { New-Side 'candidate' '' '' }
  $sides = @($lastSide, $candidateSide)
  foreach ($h in $lastSide.Hashes + $candidateSide.Hashes) { Say "  $h" }
  foreach ($side in $sides) { Start-Runner $side }
  Say "Loaded: last $($lastSide.Version), candidate $($candidateSide.Version), .NET $($candidateSide.Runtime)"
  Say ("Total CPU before starting: " + $startLoad.ToString('F1', $invariant) + "%, which falls on both sides alike")

  $names = @(& $candidateSide.Exe list | ForEach-Object { $_.Split("`t")[0] })
  if ($Only) {
    foreach ($o in $Only) { if ($names -notcontains $o) { throw "no case is called $o" } }
    $names = @($names | Where-Object { $Only -contains $_ })
  }
  $runsOf = @{}
  foreach ($n in $names) { $runsOf[$n] = @{ last = (New-Object Collections.ArrayList); candidate = (New-Object Collections.ArrayList) } }
  $clock = [Diagnostics.Stopwatch]::StartNew()
  for ($round = 0; $round -lt $WarmUp + $Runs; $round++) {
    foreach ($n in $names) {
      # Taking turns to go first, so neither side always runs just after the other
      $order = if (($round + [Array]::IndexOf($names, $n)) % 2 -eq 0) { $sides } else { @($sides[1], $sides[0]) }
      foreach ($side in $order) {
        $run = Invoke-Case $side $n
        if ($round -ge $WarmUp) { [void] $runsOf[$n][$side.Name].Add($run) }
      }
    }
    $what = if ($round -lt $WarmUp) { "warm-up $($round + 1) of $WarmUp" } else { "run $($round - $WarmUp + 1) of $Runs" }
    Write-Host "  $what done, $([int] $clock.Elapsed.TotalSeconds) s"
  }
  $endLoad = [KerfReleaseCheckCpu]::Percent(2000)

  Say
  Say ('{0,-20} {1,10} {2,10} {3,8} {4,10} {5,10}  {6}' -f 'case', 'last ms', 'cand. ms', 'change', 'last MB', 'cand. MB', 'verdict')
  $stops = New-Object Collections.ArrayList
  $notes = New-Object Collections.ArrayList
  foreach ($n in $names) {
    $l = $runsOf[$n].last
    $c = $runsOf[$n].candidate
    $reasons = New-Object Collections.ArrayList
    $lastResult = $l[0].Result
    $candidateResult = $c[0].Result
    $lastAbsent = $lastResult -eq 'absent'
    if ($candidateResult -eq 'absent') { [void] $reasons.Add('the candidate can''t run it') }
    if (@($c | Where-Object { -not (Test-SameResult $_.Result $candidateResult) }).Count) { [void] $reasons.Add('its result changed from one run to the next') }
    if (@($c | Where-Object { -not $_.IsFixed }).Count) { [void] $reasons.Add("not its fixed result: $(Format-Result $candidateResult)") }
    $lastMs = Get-Median @($l | ForEach-Object { $_.Ms })
    $candidateMs = Get-Median @($c | ForEach-Object { $_.Ms })
    $lastMB = (Get-Median @($l | ForEach-Object { $_.Bytes })) / 1e6
    $candidateMB = (Get-Median @($c | ForEach-Object { $_.Bytes })) / 1e6
    $change = ''
    if ($lastAbsent) { [void] $notes.Add("$n has no baseline: $Last can't run it, so it's checked against its fixed result only") }
    else {
      if (-not (Test-SameResult $lastResult $candidateResult)) {
        [void] $reasons.Add("a different result from ${Last}: $(Format-Result $lastResult) became $(Format-Result $candidateResult)")
      }
      $percent = if ($lastMs -gt 0) { 100 * ($candidateMs - $lastMs) / $lastMs } else { 0 }
      $change = $percent.ToString('+0;-0;0', $invariant) + '%'
      if ($percent -gt $SlowerPercent) {
        [void] $reasons.Add(('{0:F0}% slower than {1}: {2:F1} ms, against {3:F1} ms ({4:F1} times)' -f $percent, $Last, $candidateMs, $lastMs, ($candidateMs / $lastMs)))
      }
    }
    $verdict = if (-not $reasons.Count) { 'ok' } elseif ($Explained -contains $n) { 'explained' } else { 'STOPS THE RELEASE' }
    Say ('{0,-20} {1,10} {2,10} {3,8} {4,10} {5,10}  {6}' -f $n,
      $(if ($lastAbsent) { '-' } else { $lastMs.ToString('F1', $invariant) }), $candidateMs.ToString('F1', $invariant), $change,
      $(if ($lastAbsent) { '-' } else { $lastMB.ToString('F1', $invariant) }), $candidateMB.ToString('F1', $invariant), $verdict)
    foreach ($r in $reasons) {
      if ($verdict -eq 'STOPS THE RELEASE') { [void] $stops.Add("${n}: $r") } else { [void] $notes.Add("$n (explained): $r") }
    }
  }
  Say
  foreach ($note in $notes) { Say "Note: $note" }
  Say ("Total CPU at the end: " + $endLoad.ToString('F1', $invariant) + "%; times from a machine that busy are noisier, and the sides share it alike")
  if ($stops.Count) {
    Say
    Say "STOP: $($stops.Count) reasons on $(@($stops | ForEach-Object { ($_ -split ':')[0] } | Sort-Object -Unique).Count) cases, until each is explained (then -Explained):"
    foreach ($s in $stops) { Say "  $s" }
  }
  else { Say "Nothing stops the release: no case is more than $SlowerPercent% slower than $Last or gives a different result." }
  if ($Out) { [IO.File]::WriteAllLines($Out, [string[]] $report.ToArray(), (New-Object Text.UTF8Encoding($false))) }
  $exitCode = if ($stops.Count) { 1 } else { 0 }
}
finally {
  foreach ($side in $sides) {
    if ($side.Process -and -not $side.Process.HasExited) {
      $side.Process.StandardInput.WriteLine('quit')
      if (-not $side.Process.WaitForExit(10000)) { $side.Process.Kill() }
    }
  }
  if (-not $keepWork) { Remove-Folder $Work }
}
exit $exitCode
