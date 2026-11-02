// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using Xunit.Abstractions;

namespace OcctNet.DataExchange.Tests.StepGen;

/// <summary>
/// Round trips through the generated readers and writers (unit 8C1): every STEP file of OCCT's dataset and of
/// data/step is read with a test protocol recognising StepBasic, StepGeom, StepShape and StepRepr (the other records
/// stay unknown entities), written back with the files' own labels, and each record of those packages compared with
/// the input's (texts, enumerations and references exactly, reals to the writer's 13 significant digits).
/// </summary>
public sealed class StepRoundTripTests(ITestOutputHelper theOutput)
{
    internal static readonly string[] Packages = ["StepBasic", "StepGeom", "StepShape", "StepRepr"];

    private static string Metadata(string theKey) =>
        typeof(StepRoundTripTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    internal static readonly Lazy<StepTestProtocol?> TheProtocol = new(() =>
    {
        var aTable = StepTestTable.Load(Metadata("OcctSourceDir"), Packages);
        if (aTable is null)
        {
            return null;
        }

        var aProtocol = new StepTestProtocol(aTable);
        var aRw = new StepTestReadWriteModule(aTable);
        // the libraries' global lists (decision 1182): a protocol of its own, which no other test uses
        Interface_GeneralLib.SetGlobal(new StepTestGeneralModule(aTable), aProtocol);
        Interface_ReaderLib.SetGlobal(aRw, aProtocol);
        StepData_WriterLib.SetGlobal(aRw, aProtocol);
        return aProtocol;
    });

    internal static List<string> CorpusFiles()
    {
        var aFiles = new List<string>();
        foreach (string aDir in new[] { Path.Combine(Metadata("OcctTestDataDir"), "step"), Path.Combine(Metadata("OcctSourceDir"), "data", "step") })
        {
            if (Directory.Exists(aDir))
            {
                aFiles.AddRange(Directory.GetFiles(aDir).Where(f => f.EndsWith(".stp", StringComparison.OrdinalIgnoreCase) || f.EndsWith(".step", StringComparison.OrdinalIgnoreCase))
                                         .Order(StringComparer.Ordinal));
            }
        }

        return aFiles;
    }

    [Fact]
    public void CorpusRoundTrips()
    {
        var aProtocol = TheProtocol.Value;
        var aFiles = CorpusFiles();
        if (aProtocol is null || aFiles.Count == 0)
        {
            return; // OCCT's source or dataset not fetched
        }

        var aRun = new StepRoundTrip();
        foreach (string aPath in aFiles)
        {
            aRun.Run(aPath, aProtocol);
        }

        theOutput.WriteLine($"{aRun.Files} files ({aRun.Unread} not read), {aRun.Records} records, {aRun.Compared} of the four packages "
                            + $"({aRun.Types.Count} types): {aRun.Same} written back the same; {aRun.WrappedStrings} with a string wrapped by the writer");
        foreach (var (aCause, (aCount, anExample)) in aRun.Explained.Concat(aRun.Unexplained))
        {
            theOutput.WriteLine($"{aCount}: {aCause}\n  e.g. {anExample}");
        }

        Assert.Equal(0, aRun.Unread);
        Assert.True(aRun.Compared > 1_400_000, $"{aRun.Compared} records compared");
        Assert.True(aRun.Unexplained.Count == 0, string.Join("\n", aRun.Unexplained.Select(d => $"{d.Value.Count}: {d.Key}\n  {d.Value.Example}")));
    }
}
