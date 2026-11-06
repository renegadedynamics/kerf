// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Where opening's time goes (unit 11C, docs/performance.md): reads an EventPipe trace of the Bento benchmark's first pass
// and adds up, for the whole pass and for each example, the JIT's time on the thread doing the work and on the background
// compiler, type loading outside the JIT, and ReadyToRun entry points found; then says what the JIT compiled and why: which
// generic instantiations, which assemblies, which tiers.
//
// The benchmark marks each example when KERF_BENCH_MARKS is set, so a trace can be cut at them. Take the trace on a
// ReadyToRun build, as tools/bench.ps1 leaves in its -Work folder, then read it:
//   $env:KERF_BENCH_MARKS = 1; $env:DOTNET_EnableEventPipe = 1; $env:DOTNET_EventPipeOutputPath = 'opening.nettrace'
//   $env:DOTNET_EventPipeOutputStreaming = 1
//   $env:DOTNET_EventPipeConfig = 'Microsoft-Windows-DotNETRuntime:b00000003c:5,Kerf-Bench:1:5'
//   <work>/publish/bento/Kerf.Kernel.Bento.Benchmarks.exe 1 0
//   dotnet run kernel/tools/opening-report.cs -- opening.nettrace [how many of each list to print]
// Tracing at this level makes the process about half as slow again, so read the shares, not the times: JitInfo's figures in
// the rig are the untraced ones.
//
// With "samples" as the third argument, on a trace taken with the sample profiler as well (add
// 'Microsoft-DotNETCore-SampleProfiler:0:5' and the runtime's 4c14fccbd keywords, so methods have names), it instead sorts the
// working thread's samples in the first pass by the static constructor on the stack, the outermost one. Time in native code
// shows under the managed frame that called it, so a type's constructor includes what it took to load what it uses.
#:package Microsoft.Diagnostics.Tracing.TraceEvent@3.1.21
#:property TreatWarningsAsErrors=false
#:property EnforceCodeStyleInBuild=false

using System.Globalization;
using System.Text.RegularExpressions;
using Microsoft.Diagnostics.Tracing;
using Microsoft.Diagnostics.Tracing.Etlx;
using Microsoft.Diagnostics.Tracing.Parsers.Clr;

string file = args[0];
int top = args.Length > 1 ? int.Parse(args[1], CultureInfo.InvariantCulture) : 40;

if (args.Length > 2 && args[2] == "samples")
{
    Samples(file, top);
    return;
}

var marks = new List<(double Time, string Name)>();
var jits = new List<Jit>();
var jitStarts = new Dictionary<(int, long), double>();
var typeLoadDepth = new Dictionary<int, (int Depth, double Start, string Name)>();
var typeLoads = new List<(double Start, double End, int Thread, string Name)>();
var typeLoadStarts = new Dictionary<(int, int), double>();
int r2rEntryPoints = 0;
var r2rByExample = new List<double>();
var assemblies = new List<(double Time, string Name)>();
var jitDepth = new Dictionary<int, int>();

using (var source = new EventPipeEventSource(file))
{
    source.Dynamic.All += e =>
    {
        if (e.ProviderName == "Kerf-Bench" && e.EventName.StartsWith("Mark", StringComparison.Ordinal))
        {
            marks.Add((e.TimeStampRelativeMSec, (string)e.PayloadByName("name")));
        }
    };
    source.Clr.MethodJittingStarted += e => jitStarts[(e.ThreadID, e.MethodID)] = e.TimeStampRelativeMSec;
    source.Clr.MethodLoadVerbose += e =>
    {
        if (!e.IsJitted || !jitStarts.Remove((e.ThreadID, e.MethodID), out double start))
        {
            return;
        }
        jits.Add(new Jit(start, e.TimeStampRelativeMSec, e.ThreadID, e.MethodNamespace, e.MethodName, e.MethodSignature, e.OptimizationTier.ToString(), e.MethodSize));
    };
    source.Clr.TypeLoadStart += e => typeLoadStarts[(e.ThreadID, e.TypeLoadStartID)] = e.TimeStampRelativeMSec;
    source.Clr.TypeLoadStop += e =>
    {
        if (typeLoadStarts.Remove((e.ThreadID, e.TypeLoadStartID), out double start))
        {
            typeLoads.Add((start, e.TimeStampRelativeMSec, e.ThreadID, e.TypeName));
        }
    };
    source.Clr.MethodR2RGetEntryPoint += e => { r2rEntryPoints++; r2rByExample.Add(e.TimeStampRelativeMSec); };
    source.Clr.LoaderAssemblyLoad += e => assemblies.Add((e.TimeStampRelativeMSec, e.FullyQualifiedAssemblyName));
    source.Process();
}

// The thread that ran the marks does the work; every other thread's JIT is the background compiler's
double passStart = marks.Count > 0 ? marks[0].Time : 0;
double passEnd = marks.FirstOrDefault(m => m.Name == "end of first pass").Time;
if (passEnd == 0)
{
    passEnd = jits.Max(j => j.End);
}

var inPass = jits.Where(j => j.Start < passEnd).ToList();
int workThread = inPass.Where(j => j.Start >= passStart).GroupBy(j => j.Thread).OrderByDescending(g => g.Count()).First().Key;

// A type load's own time, outermost only: loads nest, and those inside a JIT are the JIT's
static double UnionMs(IEnumerable<(double Start, double End)> spans)
{
    double sum = 0, reach = double.NegativeInfinity;
    foreach (var s in spans.OrderBy(s => s.Start))
    {
        if (s.End <= reach)
        {
            continue;
        }
        sum += s.End - Math.Max(s.Start, reach);
        reach = s.End;
    }
    return sum;
}

var workJit = inPass.Where(j => j.Thread == workThread && j.Start >= passStart).ToList();
var typeOnWork = typeLoads.Where(t => t.Thread == workThread && t.Start >= passStart && t.Start < passEnd).ToList();
double typeLoadWholeMs = UnionMs(typeOnWork.Select(t => (t.Start, t.End)));
double jitWholeMs = UnionMs(workJit.Select(j => (j.Start, j.End)));
double bothMs = UnionMs(typeOnWork.Select(t => (t.Start, t.End)).Concat(workJit.Select(j => (j.Start, j.End))));

Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"Trace from start of session to first mark {passStart:F1} ms; the pass {passEnd - passStart:F1} ms (traced)"));
Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"On the working thread: JIT {jitWholeMs:F1} ms ({workJit.Count} methods), type loads {typeLoadWholeMs:F1} ms ({typeOnWork.Count} of them), the two together {bothMs:F1} ms, so type loads outside the JIT {bothMs - jitWholeMs:F1} ms"));
var background = inPass.Where(j => j.Thread != workThread).ToList();
Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"On other threads in the pass: JIT {background.Sum(j => j.End - j.Start):F1} ms ({background.Count} methods)"));
Console.WriteLine($"ReadyToRun entry points used in the trace: {r2rEntryPoints}");
Console.WriteLine($"Assemblies loaded: {assemblies.Count}, {assemblies.Count(a => a.Time >= passStart && a.Time < passEnd)} in the pass");

foreach (var a in assemblies)
{
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {a.Time - passStart,8:F1} ms  {a.Name.Split(',')[0]}"));
}

Console.WriteLine();
Console.WriteLine("By example                    wall ms  JIT ms  methods  types out of JIT ms  rest ms");
for (int m = 0; m + 1 < marks.Count; m++)
{
    double a = marks[m].Time, b = marks[m + 1].Time;
    if (a >= passEnd)
    {
        break;
    }
    var j = workJit.Where(x => x.Start >= a && x.Start < b).ToList();
    var t = typeOnWork.Where(x => x.Start >= a && x.Start < b).Select(x => (x.Start, x.End)).ToList();
    double jm = UnionMs(j.Select(x => (x.Start, x.End)));
    double bm = UnionMs(t.Concat(j.Select(x => (x.Start, x.End))));
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"{marks[m].Name,-28} {b - a,8:F1} {jm,7:F1} {j.Count,8} {bm - jm,20:F1} {b - a - bm,8:F1}"));
}

Console.WriteLine();
Console.WriteLine("By tier, on the working thread");
foreach (var g in workJit.GroupBy(j => j.Tier).OrderByDescending(g => g.Sum(j => j.Ms)))
{
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {g.Key,-28} {g.Sum(j => j.Ms),8:F1} ms {g.Count(),6} methods"));
}

Console.WriteLine();
Console.WriteLine("Why it was compiled, on the working thread, not counting tier 1 or on-stack replacement");
var first = workJit.Where(j => !j.Tier.Contains("Tier1", StringComparison.Ordinal)).ToList();
foreach (var g in first.GroupBy(Reason).OrderByDescending(g => g.Sum(j => j.Ms)))
{
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {g.Key,-60} {g.Sum(j => j.Ms),8:F1} ms {g.Count(),6} methods"));
}

Console.WriteLine();
Console.WriteLine($"The {top} types whose methods took the JIT longest");
foreach (var g in first.GroupBy(j => j.Namespace).OrderByDescending(g => g.Sum(j => j.Ms)).Take(top))
{
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {g.Sum(j => j.Ms),7:F1} ms {g.Count(),5}  {Short(g.Key)}"));
}

Console.WriteLine();
Console.WriteLine($"The {top} generic definitions whose instantiations took the JIT longest");
foreach (var g in first.Where(j => j.Namespace.Contains('`')).GroupBy(j => Definition(j.Namespace)).OrderByDescending(g => g.Sum(j => j.Ms)).Take(top))
{
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {g.Sum(j => j.Ms),7:F1} ms {g.Count(),5}  {g.Select(j => j.Namespace).Distinct().Count(),4} instantiations  {g.Key}"));
}

Console.WriteLine();
Console.WriteLine($"The {top} methods that took the JIT longest");
foreach (var j in first.OrderByDescending(j => j.Ms).Take(top))
{
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {j.Ms,7:F1} ms {j.Size,7} bytes {j.Tier,-16} {Short(j.Namespace)}.{j.Name}"));
}

Console.WriteLine();
Console.WriteLine($"The {top} non-generic Kerf methods that took the JIT longest: ReadyToRun could have held these");
foreach (var j in first.Where(j => Reason(j).StartsWith("not generic: Kerf", StringComparison.Ordinal)).OrderByDescending(j => j.Ms).Take(top))
{
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {j.Ms,7:F1} ms {j.Size,7} bytes {j.Tier,-16} {Short(j.Namespace)}.{j.Name} {Short(j.Signature)}"));
}

Console.WriteLine();
Console.WriteLine($"The {top} types taking longest to load, on the working thread, outermost");
foreach (var t in typeOnWork.GroupBy(t => t.Name).Select(g => (Name: g.Key, Ms: g.Max(x => x.End - x.Start))).OrderByDescending(t => t.Ms).Take(top))
{
    Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {t.Ms,7:F2} ms  {Short(t.Name)}"));
}

static void Samples(string file, int top)
{
    string etlx = TraceLog.CreateFromEventPipeDataFile(file);
    using var log = new TraceLog(etlx);
    double start = 0, end = double.MaxValue;
    int workThread = -1;
    foreach (var ev in log.Events)
    {
        if (ev.ProviderName == "Kerf-Bench" && ev.EventName.StartsWith("Mark", StringComparison.Ordinal))
        {
            workThread = ev.ThreadID;
            start = start == 0 ? ev.TimeStampRelativeMSec : start;
            if ((string)ev.PayloadByName("name") == "end of first pass")
            {
                end = ev.TimeStampRelativeMSec;
            }
        }
    }

    var inConstructor = new Dictionary<string, int>();
    var leaves = new Dictionary<string, int>();
    int total = 0, underConstructors = 0;
    foreach (var ev in log.Events)
    {
        if (ev.ProviderName != "Microsoft-DotNETCore-SampleProfiler" || ev.ThreadID != workThread || ev.TimeStampRelativeMSec < start || ev.TimeStampRelativeMSec >= end)
        {
            continue;
        }
        var stack = ev.CallStack();
        if (stack is null)
        {
            continue;
        }
        total++;
        string? outermost = null;
        string leaf = stack.CodeAddress.Method?.FullMethodName ?? "?";
        for (var c = stack; c is not null; c = c.Caller)
        {
            string name = c.CodeAddress.Method?.FullMethodName ?? "";
            if (name.Contains(".cctor", StringComparison.Ordinal))
            {
                outermost = name;
            }
        }
        leaves[leaf] = leaves.GetValueOrDefault(leaf) + 1;
        if (outermost is not null)
        {
            underConstructors++;
            inConstructor[outermost] = inConstructor.GetValueOrDefault(outermost) + 1;
        }
    }

    Console.WriteLine($"{total} samples on the working thread in the first pass, {underConstructors} under a static constructor");
    foreach (var kv in inConstructor.OrderByDescending(kv => kv.Value).Take(top))
    {
        Console.WriteLine($"  {kv.Value,5}  {Short(kv.Key)}");
    }
    Console.WriteLine("Leaves");
    foreach (var kv in leaves.OrderByDescending(kv => kv.Value).Take(top))
    {
        Console.WriteLine($"  {kv.Value,5}  {Short(kv.Key)}");
    }
}

static string Short(string s) => s.Length > 170 ? s[..170] + "..." : s;

static string Definition(string ns)
{
    int bracket = ns.IndexOf('[');
    return bracket > 0 ? ns[..bracket] : ns;
}

// Why ReadyToRun didn't hold this method: the generic's home against what it's instantiated over
static string Reason(Jit j)
{
    if (j.Namespace.StartsWith("dynamicClass", StringComparison.Ordinal) || j.Name.StartsWith("IL_STUB", StringComparison.Ordinal))
    {
        return "a stub (IL_STUB, a delegate or a marshalling thunk)";
    }

    string home = Home(Definition(j.Namespace));
    bool generic = j.Namespace.Contains('`') || j.Name.Contains('`') || j.Name.Contains('[') || j.Signature.Contains("!!", StringComparison.Ordinal);
    if (!generic)
    {
        return $"not generic: {home}";
    }

    int bracket = j.Namespace.IndexOf('[');
    string arguments = bracket > 0 ? j.Namespace[bracket..] : "";
    bool overStruct = arguments.Length > 0 && !arguments.Contains("__Canon", StringComparison.Ordinal);
    return arguments.Length == 0
        ? $"a generic method: {home}"
        : $"a generic type from {home} over {(arguments.Contains("Kerf.", StringComparison.Ordinal) ? "Kerf's types" : overStruct ? "others' types" : "__Canon")}";
}

static string Home(string ns) => ns.StartsWith("Kerf.", StringComparison.Ordinal) ? "Kerf" : ns.StartsWith("System.", StringComparison.Ordinal) || ns.StartsWith("Microsoft.", StringComparison.Ordinal) ? "the framework" : "others";

record Jit(double Start, double End, int Thread, string Namespace, string Name, string Signature, string Tier, int Size)
{
    public double Ms => End - Start;
}
