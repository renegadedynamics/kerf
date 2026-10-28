// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using OcctNet.DrawLite;
using Xunit.Abstractions;

namespace OcctNet.DrawLite.Tests;

/// <summary>Scans OCCT's own test corpus to show how much of it the port could run, and what to implement next.</summary>
public class DrawCorpusTests(ITestOutputHelper output)
{
    private static string TestsDirectory
        => Path.Combine(typeof(DrawCorpusTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
                                                  .Single(a => a.Key == "OcctSourceDir").Value!,
                        "tests");

    [Fact]
    public void Every_case_parses_and_the_report_lists_missing_commands()
    {
        Assert.True(Directory.Exists(TestsDirectory), $"No OCCT source at {TestsDirectory}: run tools/get-occt.ps1");

        var aCorpus = new DrawCorpus(TestsDirectory);
        var anInterpreter = new DrawInterpreter();
        var aScans = aCorpus.Cases().Select(c => aCorpus.Scan(c, anInterpreter)).ToList();

        var aMissing = aScans.SelectMany(s => s.Missing)
                             .GroupBy(c => c)
                             .Select(g => (Command: g.Key, Cases: g.Count()))
                             .OrderByDescending(x => x.Cases)
                             .ToList();

        output.WriteLine($"{aScans.Count} cases, {aScans.Count(s => s.IsRunnable)} runnable with the commands implemented so far");
        output.WriteLine("Most-needed commands (cases that use each):");
        foreach (var (aCommand, aCases) in aMissing.Take(40))
        {
            output.WriteLine($"  {aCommand,-24} {aCases}");
        }

        Assert.True(aScans.Count > 10000, "OCCT V8_0_1 has over 10,000 test cases");
        var anUnparsable = aScans.Where(s => s.Commands.Contains("<unparsable>")).Select(s => s.Case.Id).ToList();
        output.WriteLine($"{anUnparsable.Count} unparsable: {string.Join(", ", anUnparsable.Take(10))}");
        Assert.True(anUnparsable.Count < aScans.Count / 100, "the parser should read at least 99% of the corpus");
    }
}
