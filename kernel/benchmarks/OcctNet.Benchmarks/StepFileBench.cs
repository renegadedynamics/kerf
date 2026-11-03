// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Diagnostics;
using System.Globalization;
using System.Reflection;
using Occt;

namespace OcctNet.Benchmarks;

/// <summary>
/// The STEP parser's throughput (unit 8P): the port's StepFile_Scanner and StepFile_Parser filling StepFile_ReadData
/// from memory on the files tools/steprec records (the dataset's STEP files and data/step's), the fastest of 5 per
/// file, against native OCCT's flex and bison parser timed the same way by <c>tools/steprec/build.ps1 -Bench</c>
/// (its steprec-bench.txt: "name bytes ms" per file).
/// </summary>
internal static class StepFileBench
{
    private static string Metadata(string theKey)
        => typeof(StepFileBench).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    public static void Run(string? theNativeTimes)
    {
        var aNative = new Dictionary<string, double>();
        if (theNativeTimes is not null)
        {
            foreach (string aLine in File.ReadAllLines(theNativeTimes))
            {
                string[] aParts = aLine.Split(' ');
                aNative[aParts[0]] = double.Parse(aParts[^1], CultureInfo.InvariantCulture);
            }
        }

        var aFiles = new SortedDictionary<string, string>(StringComparer.Ordinal);
        string aData = Metadata("OcctTestDataDir");
        if (Directory.Exists(aData))
        {
            foreach (string aPath in Directory.EnumerateFiles(aData, "*", SearchOption.AllDirectories))
            {
                string aRel = Path.GetRelativePath(aData, aPath).Replace('\\', '/');
                string anExt = Path.GetExtension(aPath).ToLowerInvariant();
                if (aRel.StartsWith("step/", StringComparison.Ordinal) || anExt is ".stp" or ".step" or ".p21")
                {
                    aFiles["testdata/" + aRel] = aPath;
                }
            }
        }

        foreach (string aPath in Directory.EnumerateFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "step")))
        {
            aFiles["src/data/step/" + Path.GetFileName(aPath)] = aPath;
        }

        double aPortTotal = 0, aNativeTotal = 0;
        long aBytes = 0, aNativeBytes = 0;
        foreach ((string aName, string aPath) in aFiles)
        {
            byte[] anInput = File.ReadAllBytes(aPath);
            double aBest = double.MaxValue;
            for (int r = 0; r < 5; r++)
            {
                long t0 = Stopwatch.GetTimestamp();
                try
                {
                    new StepFile_Parser(new StepFile_Scanner(new StepFile_ReadData(), new MemoryStream(anInput))).parse();
                }
                catch (FailureException)
                {
                }

                aBest = Math.Min(aBest, Stopwatch.GetElapsedTime(t0).TotalMilliseconds);
            }

            aPortTotal += aBest;
            aBytes += anInput.Length;
            if (aNative.TryGetValue(aName, out double aNativeMs))
            {
                aNativeTotal += aNativeMs;
                aNativeBytes += anInput.Length;
            }
        }

        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"STEP parser: {aFiles.Count} files, {aBytes / 1e6:F1} MB: the port {aPortTotal:F0} ms ({aBytes / 1e3 / aPortTotal:F1} MB/s)"));
        if (aNativeTotal > 0)
        {
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"native {aNativeTotal:F0} ms on {aNativeBytes / 1e6:F1} MB ({aNativeBytes / 1e3 / aNativeTotal:F1} MB/s): port/native {aPortTotal / aNativeTotal:F2}×"));
        }
    }
}
