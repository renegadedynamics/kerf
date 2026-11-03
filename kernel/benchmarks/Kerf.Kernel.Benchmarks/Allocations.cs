// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using System.Diagnostics.Tracing;
using System.Globalization;
using Occt;

namespace Kerf.Kernel.Benchmarks;

/// <summary>
/// Which types the port's mesh and extraction allocate (argument "alloc"): the runtime's allocation ticks (one about
/// every 100 KB) by type, for a serial mesh-and-extract of each shape repeated a few times.
/// </summary>
internal sealed class Allocations : EventListener
{
    private readonly Dictionary<string, long> myBytes = new(StringComparer.Ordinal);
    private bool myIsOn;

    protected override void OnEventSourceCreated(EventSource theSource)
    {
        if (theSource.Name == "Microsoft-Windows-DotNETRuntime")
        {
            EnableEvents(theSource, EventLevel.Verbose, (EventKeywords)0x1); // GC keyword: AllocationTick
        }
    }

    protected override void OnEventWritten(EventWrittenEventArgs theEvent)
    {
        if (!myIsOn || !theEvent.EventName!.StartsWith("GCAllocationTick", StringComparison.Ordinal) || theEvent.Payload is null)
        {
            return;
        }
        int aType = theEvent.PayloadNames!.IndexOf("TypeName");
        int anAmount = theEvent.PayloadNames.IndexOf("AllocationAmount64");
        string aName = aType >= 0 ? theEvent.Payload[aType] as string ?? "?" : "?";
        long aBytes = anAmount >= 0 ? Convert.ToInt64(theEvent.Payload[anAmount], CultureInfo.InvariantCulture) : 100_000;
        lock (myBytes)
        {
            myBytes[aName] = myBytes.GetValueOrDefault(aName) + aBytes;
        }
    }

    public static void Run(string? theFilter)
    {
        using var aListener = new Allocations();
        foreach (Case aCase in Cases.All())
        {
            if (theFilter is not null && !aCase.Name.Contains(theFilter, StringComparison.OrdinalIgnoreCase))
            {
                continue;
            }
            Shape anInput = Cases.PortRead(aCase.F1);
            Cases.PortMeshAndExtract(anInput, aCase.Deflection, false);
            lock (aListener.myBytes)
            {
                aListener.myBytes.Clear();
            }
            aListener.myIsOn = true;
            long aBefore = GC.GetTotalAllocatedBytes(true);
            const int Runs = 5;
            for (int i = 0; i < Runs; i++)
            {
                Cases.PortMeshAndExtract(anInput, aCase.Deflection, false);
            }
            long aTotal = (GC.GetTotalAllocatedBytes(true) - aBefore) / Runs;
            aListener.myIsOn = false;
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"{aCase.Name}: {aTotal / 1048576.0:F1} MB per mesh and extract; sampled by type:"));
            lock (aListener.myBytes)
            {
                long aSampled = aListener.myBytes.Values.Sum();
                foreach ((string aName, long aBytes) in aListener.myBytes.OrderByDescending(p => p.Value).Take(25))
                {
                    Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {100.0 * aBytes / aSampled,5:F1}%  {aName}"));
                }
            }
        }
    }
}
