// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using Xunit.Abstractions;

namespace OcctNet.DataExchange.Tests.StepGen;

/// <summary>
/// Round trips through the generated readers and writers (units 8C1 and 8C2): every STEP file of OCCT's dataset and of
/// data/step is read with StepAP214's protocol and modules (RWStepAP214.Init), written back with the files' own
/// labels, and each record the protocol recognises compared with the input's (texts, enumerations and references
/// exactly, reals to the writer's 13 significant digits).
/// </summary>
public sealed class StepRoundTripTests(ITestOutputHelper theOutput)
{
    private static string Metadata(string theKey) =>
        typeof(StepRoundTripTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    /// <summary>StepAP214's protocol, with its modules in the global libraries (once per process, decision 1182).</summary>
    internal static StepAP214_Protocol Protocol()
    {
        RWStepAP214.Init();
        return StepAP214.Protocol();
    }

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
        var aFiles = CorpusFiles();
        if (aFiles.Count == 0)
        {
            return; // OCCT's dataset not fetched
        }

        var aProtocol = Protocol();
        var aRun = new StepRoundTrip();
        foreach (string aPath in aFiles)
        {
            aRun.Run(aPath, aProtocol);
        }

        long anUnknown = aRun.Unknown.Values.Sum();
        theOutput.WriteLine($"{aRun.Files} files ({aRun.Unread} not read), {aRun.Records} records, {aRun.Compared} recognised ({aRun.Types.Count} types), "
                            + $"{anUnknown} not ({aRun.Unknown.Count} types): {aRun.Same} written back the same; {aRun.WrappedStrings} with a string wrapped by the writer");
        foreach (var (aCause, (aCount, anExample)) in aRun.Explained.Concat(aRun.Unexplained))
        {
            theOutput.WriteLine($"{aCount}: {aCause}\n  e.g. {anExample}");
        }

        foreach (var (aType, aCount) in aRun.Unknown)
        {
            theOutput.WriteLine($"not recognised: {aCount} {aType}");
        }

        Assert.Equal(0, aRun.Unread);
        Assert.True(aRun.Compared > 1_500_000, $"{aRun.Compared} records compared");
        Assert.Equal(KnownUnknown, anUnknown);
        Assert.True(aRun.Unexplained.Count == 0, string.Join("\n", aRun.Unexplained.Select(d => $"{d.Value.Count}: {d.Key}\n  {d.Value.Example}")));
    }

    /// <summary>
    /// The corpus's records StepAP214's protocol doesn't recognise, in OCCT either (decision 1219): 88 types, none in
    /// OCCT's type tables (simple ones) or complex type lists, read as unknown entities and written back as read.
    /// </summary>
    private const long KnownUnknown = 17_402;
}
