// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Where Bento's recalculation goes, by the engine's operations: reads an EventPipe trace of the Bento benchmark and sorts
// its CPU samples and allocation ticks by the outermost operation on each stack, leaving out everything before a given
// time (the first pass). Unit 11A's ranking in docs/performance.md comes from it. A sample is a thread running managed code,
// on any thread, so these are shares of thread time, not of wall time; time in native code (the JIT, the collector's own
// threads) doesn't show, but a thread stopped for a collection does (PollGC).
//
// Take the trace on a ReadyToRun build, as tools/bench.ps1 leaves in its -Work folder, then read it:
//   $env:DOTNET_EnableEventPipe = 1; $env:DOTNET_EventPipeOutputPath = 'bento.nettrace'; $env:DOTNET_EventPipeOutputStreaming = 1
//   $env:DOTNET_EventPipeConfig = 'Microsoft-DotNETCore-SampleProfiler:0:5,Microsoft-Windows-DotNETRuntime:4c14fccbd:5'
//   <work>/publish/bento/Kerf.Kernel.Bento.Benchmarks.exe 20 0
//   dotnet run kernel/tools/profile-report.cs -- bento.nettrace 2500
#:package Microsoft.Diagnostics.Tracing.TraceEvent@3.1.21
#:property TreatWarningsAsErrors=false
#:property EnforceCodeStyleInBuild=false

using Microsoft.Diagnostics.Tracing;
using Microsoft.Diagnostics.Tracing.Etlx;
using Microsoft.Diagnostics.Tracing.Parsers.Clr;

string file = args[0];
double skipMs = args.Length > 1 ? double.Parse(args[1], System.Globalization.CultureInfo.InvariantCulture) : 0;

// Outermost first: the first of these on the way from the root to the leaf names the sample
(string Bucket, string Pattern)[] buckets =
[
    ("properties (volume)", "Measurement.Mass"),
    ("distance (extrude until's extrema)", "Measurement.Distance."),
    ("optimal bounding box (the mesh's tolerance, BoundBox)", "Measurement.Bounds.AddOptimal"),
    ("other bounding boxes", "Measurement.Bounds."),
    ("meshing", "Engine.Meshing."),
    ("clean-up after Booleans (UnifySameDomain)", "UnifySameDomain"),
    ("fillets and chamfers", "Engine.Fillets."),
    ("offsets and thick solids", "Engine.Offsets."),
    ("Booleans: builder", "Engine.Booleans."),
    ("construction (prisms, revolutions, faces)", "Engine.Construction."),
    ("healing", "Engine.Healing."),
    ("exploring shapes", "Engine.Topology."),
];

string etlx = TraceLog.CreateFromEventPipeDataFile(file);
using var log = new TraceLog(etlx);
var samples = new Dictionary<string, int>();
var samplesWithout = new Dictionary<string, int>();
var bytes = new Dictionary<string, long>();
var inclusive = new Dictionary<string, int>();
var others = new Dictionary<string, int>();
int total = 0, totalWithout = 0, stopped = 0, stoppedWithout = 0;
long totalBytes = 0;

static string Name(TraceCallStack s)
{
    var m = s.CodeAddress.Method;
    return m is not null ? m.FullMethodName : (s.CodeAddress.ModuleName ?? "?") + "!?";
}

(string Bucket, bool IsExtrudeUntil) Classify(TraceCallStack leaf)
{
    var frames = new List<string>();
    for (var c = leaf; c is not null; c = c.Caller)
    {
        frames.Add(Name(c));
    }
    frames.Reverse();
    // The eleventh example, in the order of Program.cs; a worker thread's stack doesn't reach it
    bool until = frames.Any(f => f.Contains("b__0_10()", StringComparison.Ordinal) || f.Contains("ExtrudeUntil", StringComparison.Ordinal));
    foreach (string f in frames)
    {
        foreach ((string bucket, string pattern) in buckets)
        {
            if (f.Contains(pattern, StringComparison.Ordinal))
            {
                // The pave filler sits under the Boolean's own frames, or on a worker thread
                bool isFiller = bucket == "Booleans: builder" && frames.Any(g => g.Contains("PaveFiller", StringComparison.Ordinal) || g.Contains("FaceFaceJob", StringComparison.Ordinal));
                return (isFiller ? "Booleans: pave filler (intersections)" : bucket, until);
            }
        }
    }
    return ("other", until);
}

foreach (var ev in log.Events)
{
    if (ev.TimeStampRelativeMSec < skipMs)
    {
        continue;
    }
    if (ev.ProviderName == "Microsoft-DotNETCore-SampleProfiler")
    {
        var st = ev.CallStack();
        if (st is null)
        {
            continue;
        }
        string leaf = Name(st);
        if (leaf.Contains("WaitForSignal", StringComparison.Ordinal) || leaf.Contains("WaitHandle.Wait", StringComparison.Ordinal)
            || leaf.Contains("Monitor.Wait", StringComparison.Ordinal) || leaf.Contains("Thread.Sleep", StringComparison.Ordinal)
            || leaf.Contains("ManualResetEventSlim.Wait", StringComparison.Ordinal) || leaf.Contains("GetQueuedCompletionStatus", StringComparison.Ordinal))
        {
            continue;
        }
        (string bucket, bool until) = Classify(st);
        total++;
        if (bucket == "other")
        {
            others[leaf] = others.GetValueOrDefault(leaf) + 1;
        }
        samples[bucket] = samples.GetValueOrDefault(bucket) + 1;
        if (!until)
        {
            totalWithout++;
            samplesWithout[bucket] = samplesWithout.GetValueOrDefault(bucket) + 1;
        }
        if (leaf.Contains("PollGC", StringComparison.Ordinal))
        {
            stopped++;
            stoppedWithout += until ? 0 : 1;
        }
        var seen = new HashSet<string>();
        for (var c = st; c is not null; c = c.Caller)
        {
            string n = Name(c);
            int p = n.IndexOf('(');
            n = p > 0 ? n[..p] : n;
            if (seen.Add(n))
            {
                inclusive[n] = inclusive.GetValueOrDefault(n) + 1;
            }
        }
    }
    else if (ev is GCAllocationTickTraceData tick)
    {
        var st = ev.CallStack();
        string bucket = st is null ? "other" : Classify(st).Bucket;
        bytes[bucket] = bytes.GetValueOrDefault(bucket) + tick.AllocationAmount64;
        totalBytes += tick.AllocationAmount64;
    }
}

Console.WriteLine($"{total} samples after {skipMs} ms, {totalWithout} of them outside extrude until; {totalBytes / 1048576} MB in allocation ticks");
Console.WriteLine();
Console.WriteLine("| Operation | Share of thread time | Without extrude until | Share of allocation |");
Console.WriteLine("|---|---:|---:|---:|");
foreach ((string bucket, int n) in samples.OrderByDescending(p => p.Value))
{
    Console.WriteLine($"| {bucket} | {100.0 * n / total:F1}% | {100.0 * samplesWithout.GetValueOrDefault(bucket) / Math.Max(1, totalWithout):F1}% | {100.0 * bytes.GetValueOrDefault(bucket) / Math.Max(1, totalBytes):F1}% |");
}
Console.WriteLine();
Console.WriteLine($"Threads stopped for a collection, across all of those: {100.0 * stopped / total:F1}% ({100.0 * stoppedWithout / Math.Max(1, totalWithout):F1}% without extrude until)");
Console.WriteLine();
Console.WriteLine("Where the rest is:");
foreach ((string n, int c) in others.OrderByDescending(p => p.Value).Take(25))
{
    Console.WriteLine($"{100.0 * c / total,6:F1}%  {n}");
}
Console.WriteLine();
Console.WriteLine("The engine's methods, inclusive:");
foreach ((string n, int c) in inclusive.Where(p => p.Key.StartsWith("Kerf.", StringComparison.Ordinal)).OrderByDescending(p => p.Value).Take(40))
{
    Console.WriteLine($"{100.0 * c / total,6:F1}%  {n}");
}
