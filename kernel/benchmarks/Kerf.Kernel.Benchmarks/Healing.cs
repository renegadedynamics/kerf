// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Topology;
using System.Diagnostics;
using System.Globalization;
using System.Reflection;

namespace Kerf.Kernel.Benchmarks;

/// <summary>
/// Healing (Kerf unit 10H4): on the dataset's STEP files up to a size, the STEP import with its shape processing, then
/// ShapeFix_Shape, ShapeUpgrade_UnifySameDomain and the STEP export on the imported (frozen) shape: per operation the
/// total of each file's fastest of three runs, and the bytes allocated by that run, serially.
/// </summary>
internal static class Healing
{
    private static string Metadata(string theKey)
        => typeof(Healing).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private const int MaxBytes = 400_000;
    private const int Rounds = 3;

    public static void Run(string? theFilter)
    {
        var aFiles = new SortedDictionary<string, string>(StringComparer.Ordinal);
        string aData = Path.Combine(Metadata("OcctTestDataDir"), "step");
        foreach (string aPath in Directory.EnumerateFiles(aData, "*", SearchOption.AllDirectories))
        {
            string anExt = Path.GetExtension(aPath).ToLowerInvariant();
            if (anExt is ".stp" or ".step" && new FileInfo(aPath).Length <= MaxBytes
                && (theFilter is null || aPath.Contains(theFilter, StringComparison.OrdinalIgnoreCase)))
            {
                aFiles[Path.GetFileName(aPath)] = aPath;
            }
        }

        double aReadMs = 0, aFixMs = 0, aUnifyMs = 0, aWriteMs = 0;
        long aReadBytes = 0, aFixBytes = 0, aUnifyBytes = 0, aWriteBytes = 0;
        int aCount = 0;
        foreach ((string aName, string aPath) in aFiles)
        {
            byte[] anInput = File.ReadAllBytes(aPath);
            Shape aShape = new();
            (double, long) aRead = Best(() =>
            {
                var aReader = new StepReader();
                aReader.ReadStream(aName, new MemoryStream(anInput));
                aReader.TransferRoots();
                aShape = aReader.OneShape();
            });
            if (aShape.IsNull)
            {
                continue;
            }
            Shape anInputShape = aShape;
            (double, long) aFix = Best(() =>
            {
                var aFixer = new ShapeFixer(anInputShape);
                aFixer.Perform();
                aFixer.Shape();
            });
            (double, long) anUnify = Best(() =>
            {
                var anUnifier = new UnifySameDomain(anInputShape, true, true, false);
                anUnifier.Build();
                anUnifier.Shape();
            });
            (double, long) aWrite = Best(() =>
            {
                var aWriter = new Writer();
                aWriter.Transfer(anInputShape, global::Kerf.Engine.Exchange.Step.ModelType.AsIs);
                aWriter.WriteStream(new MemoryStream());
            });
            aCount++;
            aReadMs += aRead.Item1;
            aReadBytes += aRead.Item2;
            aFixMs += aFix.Item1;
            aFixBytes += aFix.Item2;
            aUnifyMs += anUnify.Item1;
            aUnifyBytes += anUnify.Item2;
            aWriteMs += aWrite.Item1;
            aWriteBytes += aWrite.Item2;
        }

        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"{aCount} STEP files up to {MaxBytes / 1000} kB, the fastest of {Rounds} runs each, serial:"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  STEP import with healing  {aReadMs,9:F0} ms  {aReadBytes / 1048576.0,9:F1} MB"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  ShapeFix_Shape            {aFixMs,9:F0} ms  {aFixBytes / 1048576.0,9:F1} MB"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  UnifySameDomain           {aUnifyMs,9:F0} ms  {aUnifyBytes / 1048576.0,9:F1} MB"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"  STEP export               {aWriteMs,9:F0} ms  {aWriteBytes / 1048576.0,9:F1} MB"));
        string aGroup = $"step, {aCount} files";
        foreach ((string aRow, double aMs, long aRowBytes) in new[]
                 {
                     ("import with healing", aReadMs, aReadBytes), ("ShapeFixer", aFixMs, aFixBytes),
                     ("UnifySameDomain", aUnifyMs, aUnifyBytes), ("export", aWriteMs, aWriteBytes),
                 })
        {
            BenchRows.Add(aGroup, aRow, "ms", aMs);
            BenchRows.Add(aGroup, aRow, "alloc MB", aRowBytes / 1048576.0);
        }
    }

    // The fastest of the runs, with the bytes the thread allocated in it (failures count as runs)
    private static (double Ms, long Bytes) Best(Action theRun)
    {
        double aBest = double.MaxValue;
        long aBytes = 0;
        for (int r = 0; r < Rounds; r++)
        {
            long a0 = GC.GetAllocatedBytesForCurrentThread();
            long t0 = Stopwatch.GetTimestamp();
            try
            {
                theRun();
            }
            catch (FailureException)
            {
            }
            double aMs = Stopwatch.GetElapsedTime(t0).TotalMilliseconds;
            if (aMs < aBest)
            {
                aBest = aMs;
                aBytes = GC.GetAllocatedBytesForCurrentThread() - a0;
            }
        }
        return (aBest, aBytes);
    }
}
