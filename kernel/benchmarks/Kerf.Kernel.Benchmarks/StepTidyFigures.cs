// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step;
using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Topology;
using System.Diagnostics;
using System.Globalization;
using System.Reflection;

namespace Kerf.Kernel.Benchmarks;

/// <summary>
/// What StepTidy does (Kerf unit 11N): on the files <c>-- healing</c> exports, each shape written with
/// <c>CleanDuplicates</c> off and on: the size, the bytes allocated and the time (the fastest of three), and the two files
/// read back, with their volumes and face counts compared. Prints a row per file, and the totals.
/// </summary>
internal static class StepTidyFigures
{
    private const int MaxBytes = 400_000;
    private const int Rounds = 3;
    private const double VolumeTolerance = 1e-9;

    private static string Metadata(string theKey)
        => typeof(StepTidyFigures).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    public static void Run(string? theFilter)
    {
        var aFiles = new SortedDictionary<string, string>(StringComparer.Ordinal);
        foreach (string aPath in Directory.EnumerateFiles(Path.Combine(Metadata("OcctTestDataDir"), "step"), "*", SearchOption.AllDirectories))
        {
            string anExt = Path.GetExtension(aPath).ToLowerInvariant();
            if (anExt is ".stp" or ".step" && new FileInfo(aPath).Length <= MaxBytes
                && (theFilter is null || aPath.Contains(theFilter, StringComparison.OrdinalIgnoreCase)))
            {
                aFiles[Path.GetFileName(aPath)] = aPath;
            }
        }

        long anOffSize = 0, anOnSize = 0, anOffBytes = 0, anOnBytes = 0;
        double anOffMs = 0, anOnMs = 0;
        int aCompared = 0, aNotRead = 0, aFailed = 0;
        var aReductions = new List<double>();
        var aDiffering = new List<string>();
        Console.WriteLine("file\toff bytes\ton bytes\treduction %\toff alloc\ton alloc\tfaces off\tfaces on\tvolume off\tvolume on");
        foreach ((string aName, string aPath) in aFiles)
        {
            if (Read(aName, File.ReadAllBytes(aPath)) is not { IsNull: false } aShape)
            {
                aNotRead++;
                continue;
            }

            try
            {
                (byte[] anOff, long anOffAlloc, double aMsOff) = Write(aShape, false);
                (byte[] anOn, long anOnAlloc, double aMsOn) = Write(aShape, true);
                anOffSize += anOff.Length;
                anOnSize += anOn.Length;
                anOffBytes += anOffAlloc;
                anOnBytes += anOnAlloc;
                anOffMs += aMsOff;
                anOnMs += aMsOn;
                double aReduction = 100.0 * (anOff.Length - anOn.Length) / anOff.Length;
                aReductions.Add(aReduction);

                (int aFacesOff, double aVolumeOff) = Measure(Read(aName, anOff));
                (int aFacesOn, double aVolumeOn) = Measure(Read(aName, anOn));
                aCompared++;
                bool isSame = aFacesOff == aFacesOn
                              && Math.Abs(aVolumeOff - aVolumeOn) <= VolumeTolerance * Math.Max(1.0, Math.Abs(aVolumeOff));
                if (!isSame)
                {
                    aDiffering.Add(string.Create(CultureInfo.InvariantCulture, $"{aName}: {aFacesOff} faces, volume {aVolumeOff:R}; {aFacesOn} faces, volume {aVolumeOn:R}"));
                }
                Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                    $"{aName}\t{anOff.Length}\t{anOn.Length}\t{aReduction:F1}\t{anOffAlloc}\t{anOnAlloc}\t{aFacesOff}\t{aFacesOn}\t{aVolumeOff:R}\t{aVolumeOn:R}"));
            }
            catch (Exception e) when (e is FailureException or InvalidOperationException or ArgumentException)
            {
                aFailed++;
                aDiffering.Add($"{aName}: failed: {e.Message}");
            }
        }

        aReductions.Sort();
        Console.WriteLine();
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"{aCompared} files compared of {aFiles.Count}; {aNotRead} not read to a shape; {aFailed} failed"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"size: off {anOffSize:N0} bytes, on {anOnSize:N0} bytes, {100.0 * (anOffSize - anOnSize) / anOffSize:F1}% less"));
        if (aReductions.Count > 0)
        {
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"reduction per file: smallest {aReductions[0]:F1}%, median {aReductions[aReductions.Count / 2]:F1}%, largest {aReductions[^1]:F1}%"));
        }
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"allocation: off {anOffBytes / 1048576.0:F1} MB, on {anOnBytes / 1048576.0:F1} MB"));
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture, $"time (only on a quiet machine): off {anOffMs:F0} ms, on {anOnMs:F0} ms"));
        Console.WriteLine($"differing or failing: {aDiffering.Count}");
        foreach (string aLine in aDiffering)
        {
            Console.WriteLine("  " + aLine);
        }
    }

    private static Shape? Read(string theName, byte[] theData)
    {
        try
        {
            var aReader = new StepReader();
            aReader.ReadStream(theName, new MemoryStream(theData));
            aReader.TransferRoots();
            return aReader.OneShape();
        }
        catch (Exception e) when (e is FailureException or InvalidOperationException or ArgumentException)
        {
            return null;
        }
    }

    private static (byte[] Data, long Bytes, double Ms) Write(in Shape theShape, bool theClean)
    {
        byte[] aData = [];
        long aBytes = 0;
        double aBest = double.MaxValue;
        for (int r = 0; r < Rounds; r++)
        {
            var aStream = new MemoryStream();
            long a0 = GC.GetAllocatedBytesForCurrentThread();
            long t0 = Stopwatch.GetTimestamp();
            var aWriter = new Writer();
            aWriter.Transfer(theShape, global::Kerf.Engine.Exchange.Step.ModelType.AsIs);
            if (theClean)
            {
                aWriter.CleanDuplicateEntities();
            }
            aWriter.WriteStream(aStream);
            double aMs = Stopwatch.GetElapsedTime(t0).TotalMilliseconds;
            if (aMs < aBest)
            {
                aBest = aMs;
                aBytes = GC.GetAllocatedBytesForCurrentThread() - a0;
                aData = aStream.ToArray();
            }
        }
        return (aData, aBytes, aBest);
    }

    private static (int Faces, double Volume) Measure(Shape? theShape)
    {
        if (theShape is not { IsNull: false } aShape)
        {
            return (-1, double.NaN);
        }
        int aFaces = 0;
        for (var anExplorer = new Explorer(aShape, ShapeType.Face); anExplorer.More(); anExplorer.Next())
        {
            aFaces++;
        }
        var aProperties = new Properties();
        MassPropertiesBuilder.VolumeProperties(aShape, aProperties);
        return (aFaces, aProperties.Mass());
    }
}
