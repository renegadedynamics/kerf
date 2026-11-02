// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;

namespace OcctNet.DataExchange.Tests.StepGen;

/// <summary>
/// tools/stepgen (unit 8C1): the committed STEP entity classes are the generator's output, and the generator is
/// deterministic.
/// </summary>
public sealed class StepGenTests
{
    private static string Metadata(string theKey) =>
        typeof(StepGenTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static string? OcctSource()
    {
        string aDir = Metadata("OcctSourceDir");
        return Directory.Exists(Path.Combine(aDir, "src", "DataExchange", "TKDESTEP", "StepBasic")) ? aDir : null;
    }

    private static string RepoRoot()
    {
        for (var d = new DirectoryInfo(AppContext.BaseDirectory); d != null; d = d.Parent)
        {
            if (File.Exists(Path.Combine(d.FullName, "occt-version.json")))
            {
                return d.FullName;
            }
        }

        throw new InvalidOperationException("not inside the repository");
    }

    private static readonly string[] Packages =
        ["StepBasic", "StepGeom", "StepShape", "StepRepr", "RWStepBasic", "RWStepGeom", "RWStepShape", "RWStepRepr"];

    [Fact]
    public void CommittedSourcesAreTheGeneratorsOutput()
    {
        string? anOcct = OcctSource();
        if (anOcct is null)
        {
            return; // OCCT's source isn't fetched (tools/get-occt.ps1)
        }

        var anOut = new StringWriter();
        var anErr = new StringWriter();
        int aStatus = global::StepGen.Generator.Run(
            ["--occt", anOcct, "--out", Path.Combine(RepoRoot(), "src", "OcctNet.DataExchange"), "--packages", string.Join(",", Packages), "--check"],
            anOut, anErr);
        Assert.True(aStatus == 0, "regenerate with tools/stepgen:\n" + anErr);
    }

    [Fact]
    public void GeneratorIsDeterministic()
    {
        string? anOcct = OcctSource();
        if (anOcct is null)
        {
            return;
        }

        string aTemp = Path.Combine(Path.GetTempPath(), "stepgen-" + Guid.NewGuid().ToString("N"));
        try
        {
            string a = Path.Combine(aTemp, "a"), b = Path.Combine(aTemp, "b");
            foreach (string aDir in new[] { a, b })
            {
                int aStatus = global::StepGen.Generator.Run(["--occt", anOcct, "--out", aDir, "--packages", string.Join(",", Packages)],
                                                            TextWriter.Null, TextWriter.Null);
                Assert.Equal(0, aStatus);
            }

            var aFiles = Directory.GetFiles(a, "*.cs", SearchOption.AllDirectories).Select(f => Path.GetRelativePath(a, f)).Order(StringComparer.Ordinal).ToList();
            var bFiles = Directory.GetFiles(b, "*.cs", SearchOption.AllDirectories).Select(f => Path.GetRelativePath(b, f)).Order(StringComparer.Ordinal).ToList();
            Assert.Equal(aFiles, bFiles);
            Assert.True(aFiles.Count > 800);
            foreach (string f in aFiles)
            {
                Assert.True(File.ReadAllBytes(Path.Combine(a, f)).AsSpan().SequenceEqual(File.ReadAllBytes(Path.Combine(b, f))), f);
            }
        }
        finally
        {
            if (Directory.Exists(aTemp))
            {
                Directory.Delete(aTemp, true);
            }
        }
    }
}
