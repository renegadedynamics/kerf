// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
//
// Which locks are contended, and how long the waits are: reads an EventPipe trace of the Bento benchmark, taken as
// tools/profile-report.cs says (its keywords include the runtime's contention events), and sorts the Contention events by
// the first frame on the waiting stack that isn't the runtime's. Unit 11D's verdict on the locks in docs/performance.md and
// docs/state.md comes from it.
//   dotnet run kernel/tools/contention-report.cs -- bento.nettrace [ms to skip]
#:package Microsoft.Diagnostics.Tracing.TraceEvent@3.1.21
#:property TreatWarningsAsErrors=false
#:property EnforceCodeStyleInBuild=false

using Microsoft.Diagnostics.Tracing;
using Microsoft.Diagnostics.Tracing.Etlx;
using Microsoft.Diagnostics.Tracing.Parsers.Clr;

string file = args[0];
double skipMs = args.Length > 1 ? double.Parse(args[1], System.Globalization.CultureInfo.InvariantCulture) : 0;
string etlx = TraceLog.CreateFromEventPipeDataFile(file);
using var log = new TraceLog(etlx);
var counts = new Dictionary<string, int>();
var waits = new Dictionary<string, double>();
var started = new Dictionary<int, string>();
int total = 0;
double totalMs = 0;

static string Name(TraceCallStack s)
{
    var m = s.CodeAddress.Method;
    return m is not null ? m.FullMethodName : (s.CodeAddress.ModuleName ?? "?") + "!?";
}

foreach (var ev in log.Events)
{
    if (ev.TimeStampRelativeMSec < skipMs)
    {
        continue;
    }
    if (ev is ContentionStartTraceData)
    {
        string site = "?";
        var chain = new List<string>();
        for (var c = ev.CallStack(); c is not null; c = c.Caller)
        {
            chain.Add(Name(c));
        }
        // The first frame out of the runtime's own lock code, and the one above it, to tell callers of a shared helper apart
        int at = chain.FindIndex(f => !f.StartsWith("System.Threading.Lock") && !f.StartsWith("System.Threading.Monitor") && !f.Contains("!?"));
        if (at >= 0)
        {
            site = chain[at] + (at + 1 < chain.Count ? "  <-  " + chain[at + 1] : "");
        }
        started[ev.ThreadID] = site;
        counts[site] = counts.GetValueOrDefault(site) + 1;
        total++;
    }
    else if (ev is ContentionStopTraceData stop && started.Remove(ev.ThreadID, out string? from))
    {
        double ms = stop.DurationNs / 1e6;
        waits[from] = waits.GetValueOrDefault(from) + ms;
        totalMs += ms;
    }
}
Console.WriteLine($"{total} contentions, {totalMs:F3} ms waiting in all");
foreach (var (site, n) in counts.OrderByDescending(p => waits.GetValueOrDefault(p.Key)))
{
    Console.WriteLine($"{n,6} {waits.GetValueOrDefault(site),10:F3} ms  {site}");
}
