// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Checks.Full;
using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// The check on small shapes against the check as it was, over one of the corpus's groups: with <c>KERF_CHECK_COMPARE=file</c>
/// and <c>KERF_CHECK_COMPARE_GROUP=boolean</c> (or blend, heal, chamfer), each check any case makes is made both ways.
/// It does nothing without them, as it takes minutes. <c>KERF_CHECK_COMPARE_WORKERS</c> sets how many cases run at once (4).
/// Cases are only there to make shapes, so what they conclude isn't asserted: their limit can pass with each check made twice.
/// </summary>
[Collection(CheckComparisonCollection.Name)]
public class DrawCheckComparisonCorpusTests(ITestOutputHelper output)
{
    [Fact]
    public void Each_check_of_a_group_finds_the_same_as_the_check_as_it_was()
    {
        string? aGroup = Environment.GetEnvironmentVariable("KERF_CHECK_COMPARE_GROUP");
        if (string.IsNullOrEmpty(aGroup) || string.IsNullOrEmpty(Environment.GetEnvironmentVariable("KERF_CHECK_COMPARE")))
        {
            return;
        }

        int aWorkers = int.TryParse(Environment.GetEnvironmentVariable("KERF_CHECK_COMPARE_WORKERS"), out int aSet) ? aSet : 4;
        string aTests = Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");
        var aCorpus = new DrawCorpus(aTests);
        var anInterpreter = new DrawInterpreter();
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        List<DrawTestCase> aCases = aCorpus.Cases().Where(c => c.Group == aGroup).ToList();
        List<DrawTestCase> aRunnable = aCases.Where(c => aCorpus.Scan(c, anInterpreter).IsRunnable).ToList();
        var aRuns = aRunnable.AsParallel().WithDegreeOfParallelism(aWorkers)
                             .Select(c => aCorpus.RunAndCheck(c, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                              TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null))
                             .ToList();
        string aStatuses = string.Join(", ", aRuns.GroupBy(r => r.Status.Split(' ')[0]).OrderBy(g => g.Key, StringComparer.Ordinal).Select(g => $"{g.Key} {g.Count()}"));
        string aLine = $"{aGroup}: {aRunnable.Count} cases run of {aCases.Count}; {aStatuses}; {CheckComparison.Compared} checks compared, {CheckComparison.Differences} differences";
        output.WriteLine(aLine);
        File.AppendAllText(Environment.GetEnvironmentVariable("KERF_CHECK_COMPARE")!, aLine + Environment.NewLine);
        Assert.Equal(0, CheckComparison.Differences);
    }
}

/// <summary>
/// The check comparison's collection, run on its own after every other test: CheckComparison counts the process's
/// checks, which should be the group's alone (decision 27802).
/// </summary>
[CollectionDefinition(Name, DisableParallelization = true)]
public sealed class CheckComparisonCollection
{
    public const string Name = "Check comparison";
}
