// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Where Bento's recalculation allocates: reads an EventPipe trace of the Bento benchmark, taken as tools/profile-report.cs says,
// and sorts its allocation ticks by type, by the first frame that isn't a collection, and by a chain of four frames from there.
// Unit 11B's sites in docs/performance.md come from it. A tick stands for about 100 kB, so small sites are rough.
//   dotnet run kernel/tools/alloc-report.cs -- bento.nettrace <ms to skip> [rows] [only stacks with this frame]
//   dotnet run kernel/tools/alloc-report.cs -- bento.nettrace <ms to skip> ticks ticks.tsv (every tick's type and stack, for tools/alloc-groups.py)
#:package Microsoft.Diagnostics.Tracing.TraceEvent@3.1.21
#:property TreatWarningsAsErrors=false
#:property EnforceCodeStyleInBuild=false

using Microsoft.Diagnostics.Tracing;
using Microsoft.Diagnostics.Tracing.Etlx;
using Microsoft.Diagnostics.Tracing.Parsers.Clr;

string file = args[0];
double skipMs = args.Length > 1 ? double.Parse(args[1], System.Globalization.CultureInfo.InvariantCulture) : 0;
bool dump = args.Length > 3 && args[2] == "ticks";
int top = args.Length > 2 && !dump ? int.Parse(args[2]) : 45;
string? filter = args.Length > 3 && !dump ? args[3] : null;
string etlx = TraceLog.CreateFromEventPipeDataFile(file);
using var log = new TraceLog(etlx);
if (dump)
{
    var ticks = new Dictionary<string, (long Bytes, int Count)>();
    foreach (var ev in log.Events)
    {
        if (ev.TimeStampRelativeMSec < skipMs || ev is not GCAllocationTickTraceData tick)
        {
            continue;
        }
        var stack = new List<string>();
        for (var c = ev.CallStack(); c is not null; c = c.Caller)
        {
            string n = c.CodeAddress.Method?.FullMethodName ?? (c.CodeAddress.ModuleName ?? "?") + "!?";
            int p = n.IndexOf('(');
            stack.Add(p > 0 ? n[..p] : n);
        }
        string key = (tick.TypeName ?? "?") + "	" + string.Join("|", stack);
        var had = ticks.GetValueOrDefault(key);
        ticks[key] = (had.Bytes + tick.AllocationAmount64, had.Count + 1);
    }
    File.WriteAllLines(args[3], ticks.OrderByDescending(k => k.Value.Bytes).Select(k => $"{k.Value.Bytes}	{k.Value.Count}	{k.Key}"));
    return;
}
var byType = new Dictionary<string, long>();
var byLeaf = new Dictionary<string, long>();
var bySite = new Dictionary<string, long>();
var bySiteType = new Dictionary<string, long>();
var inclusive = new Dictionary<string, long>();
var chains = new Dictionary<string, long>();
long total = 0;

static string Name(TraceCallStack s)
{
    var m = s.CodeAddress.Method;
    string n = m is not null ? m.FullMethodName : (s.CodeAddress.ModuleName ?? "?") + "!?";
    int p = n.IndexOf('(');
    return p > 0 ? n[..p] : n;
}

static string Short(string f) { var a = f.Split('.'); return a.Length > 1 ? a[^2] + "." + a[^1] : f; }

static bool Plumbing(string f) => f.Contains("NCollection_") || f.Contains("VectorBase") || f.Contains("math_Vector") || f.Contains("math_Matrix")
    || f.StartsWith("System.") || f.Contains("!?") || f.Contains("Kerf.Engine.Collections") || f.Contains("ListExtensions") || f.Contains("DictionaryExtensions");

foreach (var ev in log.Events)
{
    if (ev.TimeStampRelativeMSec < skipMs || ev is not GCAllocationTickTraceData t)
    {
        continue;
    }
    long b = t.AllocationAmount64;
    total += b;
    string type = t.TypeName ?? "?";
        var st = ev.CallStack();
    if (st is null)
    {
        continue;
    }
    var frames = new List<string>();
    for (var c = st; c is not null; c = c.Caller)
    {
        frames.Add(Name(c));
    }
    if (filter is not null && !frames.Any(f => f.Contains(filter)))
    {
        total -= b;
        continue;
    }
    byType[type] = byType.GetValueOrDefault(type) + b;
    byLeaf[frames[0]] = byLeaf.GetValueOrDefault(frames[0]) + b;
    string site = frames.FirstOrDefault(f => !Plumbing(f)) ?? frames[0];
    bySite[site] = bySite.GetValueOrDefault(site) + b;
    string st2 = site + "  <" + type + ">";
    bySiteType[st2] = bySiteType.GetValueOrDefault(st2) + b;
    int si = frames.IndexOf(site);
    string chain = string.Join(" < ", frames.Skip(si).Take(4).Select(Short)) + "  <" + type + ">";
    chains[chain] = chains.GetValueOrDefault(chain) + b;
    foreach (string f in new HashSet<string>(frames))
    {
        inclusive[f] = inclusive.GetValueOrDefault(f) + b;
    }
}

void Show(string title, Dictionary<string, long> d, int n)
{
    Console.WriteLine($"--- {title}");
    foreach (var kv in d.OrderByDescending(k => k.Value).Take(n))
    {
        Console.WriteLine($"{100.0 * kv.Value / total,6:F2}%  {kv.Value / 1048576.0,9:F1} MB  {kv.Key}");
    }
}

Console.WriteLine($"total sampled {total / 1048576.0:F0} MB");
Show("by type", byType, 30);
Show("by first non-plumbing frame", bySite, top);
Show("by site and type", bySiteType, top);
Show("chains", chains, top);
Show("inclusive", inclusive, top * 2);
