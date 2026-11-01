// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#if HAS_NATIVE
using System.Diagnostics;
using System.Globalization;
using Occt;
using OcctNet.Oracle.Tests;
using N = Build123d.Occt;

namespace OcctNet.Benchmarks;

/// <summary>
/// The phase 7 operations suite (plan 2.7, unit 7X): build123d.NET's offset, draft, feature and healing calls (the
/// oracle's capture, <see cref="Build123dPhase7Capture"/>: Hollow, Offset3D, Thicken, OffsetCore, OffsetTopodsFace,
/// Draft, ExtrudeTaper's draft, DPrism, SplitByPerimeter, ToSplines), the port against native OCCT, serially. Both sides
/// read the same text; each run reads its operands afresh outside the timing (native writes into its input), the port's
/// frozen as a caller's. The median of the call after a warm-up, and the port's allocation of one call. Run in Release:
/// <c>-- phase7 [filter]</c> (a filter on the operation or the name).
/// </summary>
public static class Phase7
{
    private static double Ms(long theTicks) => theTicks * 1000.0 / Stopwatch.Frequency;

    // The median of the timed part after a warm-up: at least 5 runs, up to 1 s or 21 runs
    private static double Measure(Func<Func<long>> theSetup)
    {
        theSetup()();
        var aTimes = new List<double>();
        var aClock = Stopwatch.StartNew();
        for (int i = 0; i < 21 && (i < 5 || aClock.Elapsed.TotalSeconds < 1.0); i++)
        {
            aTimes.Add(Ms(theSetup()()));
        }
        aTimes.Sort();
        int n = aTimes.Count;
        return n % 2 == 1 ? aTimes[n / 2] : 0.5 * (aTimes[n / 2 - 1] + aTimes[n / 2]);
    }

    private sealed class Totals
    {
        public int Cases;
        public double Port, Native, LogRatio, AllocatedMB;
        public readonly List<(string Name, double Port, double Native)> Worst = [];
    }

    public static void Run(string? theFilter)
    {
        Console.WriteLine($".NET {Environment.Version}, {Environment.ProcessorCount} logical processors, {(Debugger.IsAttached ? "debugger" : "no debugger")}");
        Console.WriteLine("Medians in ms of build123d.NET's phase 7 calls (serial), the port against native (build123d.NET's binding of OCCT 8.0.1, "
                          + "MSVC); operands read afresh before each run, outside the timing. The port's MB are allocated by one call.");
        var aTotals = new SortedDictionary<string, Totals>(StringComparer.Ordinal);
        foreach (P7Call aCall in Build123dPhase7Capture.Capture(f => Console.Error.WriteLine("scenario failed: " + f)))
        {
            string aLabel = Build123dPhase7Capture.Label(aCall);
            if (theFilter is not null && !aLabel.Contains(theFilter, StringComparison.OrdinalIgnoreCase)
                && !aCall.Name.Contains(theFilter, StringComparison.OrdinalIgnoreCase))
            {
                continue;
            }
            string aText = Build123dPhase7Capture.OperandsText(aCall.Operands);
            double aPort = Measure(() =>
            {
                List<TopoDS_Shape> anOps = Build123dPhase7Capture.PortOperands(aText);
                return () =>
                {
                    long t0 = Stopwatch.GetTimestamp();
                    Build123dPhase7Capture.RunPort(aCall, anOps, false);
                    return Stopwatch.GetTimestamp() - t0;
                };
            });
            double aNative = Measure(() =>
            {
                List<N.TopoDS_Shape> anOps = Build123dPhase7Capture.NativeOperands(aText);
                return () =>
                {
                    long t0 = Stopwatch.GetTimestamp();
                    Build123dPhase7Capture.RunNative(aCall, anOps, false).Result?.Dispose();
                    return Stopwatch.GetTimestamp() - t0;
                };
            });
            List<TopoDS_Shape> aPOps = Build123dPhase7Capture.PortOperands(aText);
            long aBytes = GC.GetTotalAllocatedBytes(true);
            Build123dPhase7Capture.RunPort(aCall, aPOps, false);
            double anAllocated = (GC.GetTotalAllocatedBytes(true) - aBytes) / 1048576.0;
            if (!aTotals.TryGetValue(aLabel, out Totals? aT))
            {
                aTotals[aLabel] = aT = new Totals();
            }
            aT.Cases++;
            aT.Port += aPort;
            aT.Native += aNative;
            aT.LogRatio += Math.Log(aPort / aNative);
            aT.AllocatedMB += anAllocated;
            aT.Worst.Add(($"{aCall.Test}: {aCall.Name}", aPort, aNative));
        }

        Console.WriteLine();
        Console.WriteLine("| operation | calls | port serial | native serial | port/native (sum) | geometric mean | port MB per call |");
        Console.WriteLine("|---|---:|---:|---:|---:|---:|---:|");
        var anAll = new Totals();
        foreach ((string aLabel, Totals aT) in aTotals)
        {
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"| {aLabel} | {aT.Cases} | {aT.Port:F1} | {aT.Native:F1} | {aT.Port / aT.Native:F2}× | {Math.Exp(aT.LogRatio / aT.Cases):F2}× | "
                + $"{aT.AllocatedMB / aT.Cases:F2} |"));
            anAll.Cases += aT.Cases;
            anAll.Port += aT.Port;
            anAll.Native += aT.Native;
            anAll.LogRatio += aT.LogRatio;
            anAll.AllocatedMB += aT.AllocatedMB;
        }
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"| all | {anAll.Cases} | {anAll.Port:F1} | {anAll.Native:F1} | {anAll.Port / anAll.Native:F2}× | {Math.Exp(anAll.LogRatio / anAll.Cases):F2}× | "
            + $"{anAll.AllocatedMB / anAll.Cases:F2} |"));
        Console.WriteLine();
        Console.WriteLine("The slowest port/native ratios (calls over 0.5 ms natively):");
        foreach ((string aName, double aPort, double aNative) in aTotals.Values.SelectMany(t => t.Worst).Where(w => w.Native > 0.5)
                                                                    .OrderByDescending(w => w.Port / w.Native).Take(10))
        {
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"  {aName}: port {aPort:F2} ms, native {aNative:F2} ms, {aPort / aNative:F2}×"));
        }
    }
}
#endif
