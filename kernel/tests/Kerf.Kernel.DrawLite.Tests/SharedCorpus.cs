// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using System.Diagnostics;
using System.Reflection;
using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;

namespace Kerf.Kernel.DrawLite.Tests;

/// <summary>
/// OCCT's corpus as the corpus tests share it (decision 2082): one scan of every case with all of Draw-lite's commands,
/// and each case run once with the default settings however many tests run it. Both are what each test computed on its
/// own before: the scan is a function of the case and the interpreter's commands, and a run of the case's script in a new
/// interpreter.
/// </summary>
/// <remarks>
/// Each value is computed once, by whichever test asks first; a test asking for a run another test is computing waits
/// for it (<see cref="Lazy{T}"/>'s thread-safe mode), so no case runs twice and none waits for anything but a run in
/// progress. A run is kept for the rest of the test process.
/// </remarks>
internal static class SharedCorpus
{
    public static string TestsDirectory => Path.Combine(DrawResourcesTests.OcctSourceDirectory, "tests");

    /// <summary>Whether OCCT's source, with its tests directory, is there (tools/get-occt.ps1).</summary>
    public static bool Exists => Directory.Exists(TestsDirectory);

    private static readonly Lazy<DrawCorpus> s_corpus = new(() => new DrawCorpus(TestsDirectory));

    /// <summary>The corpus with the default settings.</summary>
    public static DrawCorpus Corpus => s_corpus.Value;

    /// <summary>A new interpreter with the DrawResources loaded: every command of Draw-lite.</summary>
    public static DrawInterpreter NewInterpreter()
    {
        var anInterpreter = new DrawInterpreter();
        anInterpreter.LoadDrawResources(DrawResourcesTests.ResourcesDirectory);
        return anInterpreter;
    }

    private static readonly Lazy<IReadOnlyList<DrawCaseScan>> s_scans = new(() =>
    {
        DrawInterpreter anInterpreter = NewInterpreter();
        return Corpus.Cases().AsParallel().AsOrdered().Select(c => Corpus.Scan(c, anInterpreter)).ToList();
    });

    /// <summary>
    /// Every case of the corpus scanned with all of Draw-lite's commands, in the corpus's order (as
    /// <c>Corpus.Cases().AsParallel().AsOrdered().Select(c => Corpus.Scan(c, NewInterpreter()))</c>).
    /// </summary>
    public static IReadOnlyList<DrawCaseScan> Scans => s_scans.Value;

    /// <summary>
    /// The cases runnable with the interpreter's commands (those whose scan by <paramref name="theCorpus"/>, the shared
    /// corpus by default, misses none), in the order given: scanned side by side.
    /// </summary>
    public static List<DrawTestCase> RunnableWith(IEnumerable<DrawTestCase> theCases, DrawInterpreter theInterpreter, DrawCorpus? theCorpus = null)
    {
        DrawCorpus aCorpus = theCorpus ?? Corpus;
        return [.. theCases.AsParallel().AsOrdered().Where(c => aCorpus.Scan(c, theInterpreter).IsRunnable)];
    }

    /// <summary>
    /// The runnable cases of the scans that weren't runnable before a unit (<paramref name="theBefore"/> takes its commands
    /// out of a new interpreter), in order; with <paramref name="theFilterVariable"/> set, only those whose id contains its value.
    /// </summary>
    public static List<DrawTestCase> NewlyRunnable(IEnumerable<DrawCaseScan> theScans, Action<DrawInterpreter> theBefore, string? theFilterVariable = null)
    {
        List<DrawCaseScan> aScans = [.. theScans];
        DrawInterpreter aBefore = NewInterpreter();
        theBefore(aBefore);
        var aRunnableBefore = new HashSet<string>(RunnableWith(aScans.Select(s => s.Case), aBefore).Select(c => c.Id), StringComparer.Ordinal);
        string? aFilter = theFilterVariable is null ? null : Environment.GetEnvironmentVariable(theFilterVariable);
        return [.. aScans.Where(s => s.IsRunnable && !aRunnableBefore.Contains(s.Case.Id)).Select(s => s.Case)
                         .Where(c => string.IsNullOrEmpty(aFilter) || c.Id.Contains(aFilter, StringComparison.Ordinal))];
    }

    private static readonly ConcurrentDictionary<string, Lazy<(DrawCaseRun Run, TimeSpan Time)>> s_runs = new(StringComparer.Ordinal);

    /// <summary>
    /// The case run as the corpus tests run it: <see cref="DrawCorpus.RunAndCheck"/> with OCCT's resources, the corpus's
    /// data path, <see cref="DrawCorpusTests.CorpusTimeLimitSeconds"/> and no setup; with the time it took.
    /// </summary>
    public static (DrawCaseRun Run, TimeSpan Time) Timed(DrawTestCase theCase)
        => s_runs.GetOrAdd(theCase.Id, _ => new Lazy<(DrawCaseRun, TimeSpan)>(() =>
        {
            var aClock = Stopwatch.StartNew();
            DrawCaseRun aRun = Corpus.RunAndCheck(theCase, DrawResourcesTests.ResourcesDirectory, DrawCorpusTests.DataPath,
                                                  TimeSpan.FromSeconds(DrawCorpusTests.CorpusTimeLimitSeconds), null);
            return (aRun, aClock.Elapsed);
        })).Value;

    /// <summary>The case run as the corpus tests run it (<see cref="Timed"/>).</summary>
    public static DrawCaseRun Run(DrawTestCase theCase) => Timed(theCase).Run;

    /// <summary>
    /// The cases run (<see cref="Run"/>), in the order given: first those no test has started, here, then the others,
    /// waiting for those still running elsewhere.
    /// </summary>
    public static List<DrawCaseRun> RunAll(IEnumerable<DrawTestCase> theCases)
    {
        List<DrawTestCase> aCases = [.. theCases];
        foreach (DrawTestCase aCase in aCases.Where(c => !s_runs.ContainsKey(c.Id)))
        {
            Run(aCase);
        }

        return [.. aCases.Select(Run)];
    }
}

/// <summary>
/// The cases a corpus test runs, split into <see cref="Shards"/> classes of <see cref="CorpusShardTests{TTest, TShard}"/>
/// so that xUnit, which runs a class's tests one after another, runs them side by side (decision 2082): case <c>i</c> of
/// <see cref="Cases"/> is a row of shard <c>i % Shards</c>.
/// </summary>
/// <param name="theShards">The number of shard classes.</param>
/// <param name="theSelect">The cases, in the test's order: computed once, when first needed.</param>
/// <param name="theCheck">What the test asks of each case's run on its own, if anything.</param>
public sealed class CorpusPlan(int theShards, Func<IEnumerable<DrawTestCase>> theSelect, Action<DrawCaseRun>? theCheck = null)
{
    private readonly Lazy<IReadOnlyList<DrawTestCase>> myCases = new(() => SharedCorpus.Exists ? [.. theSelect()] : []);

    public int Shards { get; } = theShards;

    /// <summary>The cases, in the test's order (none without OCCT's source).</summary>
    public IReadOnlyList<DrawTestCase> Cases => myCases.Value;

    /// <summary>The cases of one shard.</summary>
    public IEnumerable<DrawTestCase> Shard(int theIndex) => Cases.Where((_, i) => i % Shards == theIndex);

    /// <summary>The test's check of one case's run (nothing, if it asks nothing of each case alone).</summary>
    public void Check(DrawCaseRun theRun) => theCheck?.Invoke(theRun);
}

/// <summary>A corpus test whose cases run in shard classes as well (<see cref="CorpusShardTests{TTest, TShard}"/>).</summary>
public interface ICorpusTest
{
    static abstract CorpusPlan Corpus { get; }
}

/// <summary>The index of a shard class: <c>Shard0</c>, <c>Shard1</c>… (CorpusShards.cs).</summary>
public interface IShardIndex
{
    static abstract int Index { get; }
}

/// <summary>
/// One shard of a corpus test: a row per case, each run (<see cref="SharedCorpus.Timed"/>) and given the test's check of
/// a single case. The test itself keeps its checks of all its cases together, on the same runs.
/// </summary>
public abstract class CorpusShardTests<TTest, TShard>(ITestOutputHelper theOutput) where TTest : ICorpusTest where TShard : IShardIndex
{
    /// <summary>The shard's cases, by id.</summary>
    public static TheoryData<string> Cases => [.. TTest.Corpus.Shard(TShard.Index).Select(c => c.Id)];

    [Theory]
    [MemberData(nameof(Cases))]
    public void Case_runs(string theId)
    {
        DrawTestCase aCase = TTest.Corpus.Cases.Single(c => c.Id == theId);
        (DrawCaseRun aRun, TimeSpan aTime) = SharedCorpus.Timed(aCase);
        theOutput.WriteLine($"{aRun.Status} ({aTime.TotalSeconds:F1} s)");
        if (!aRun.Passed)
        {
            theOutput.WriteLine(aRun.Log);
        }

        TTest.Corpus.Check(aRun);
    }
}

/// <summary>Every corpus test's shard classes are there, each once: every case of its plan is a row of one shard.</summary>
public class CorpusShardsTests
{
    [Fact]
    public void Each_corpus_test_has_one_class_per_shard()
    {
        Type[] aTypes = typeof(CorpusShardsTests).Assembly.GetTypes();
        var aShards = aTypes.Where(t => !t.IsAbstract && t.BaseType is { IsGenericType: true } b
                                        && b.GetGenericTypeDefinition() == typeof(CorpusShardTests<,>))
                            .Select(t => t.BaseType!.GetGenericArguments())
                            .Select(a => (Test: a[0], Index: (int)Static(a[1], nameof(IShardIndex.Index))))
                            .ToList();
        List<Type> aTests = [.. aTypes.Where(t => t.GetInterfaces().Contains(typeof(ICorpusTest)))];
        Assert.NotEmpty(aTests);
        Assert.Equal(aTests.Order(Comparer<Type>.Create((x, y) => string.CompareOrdinal(x.Name, y.Name))),
                     aShards.Select(s => s.Test).Distinct().Order(Comparer<Type>.Create((x, y) => string.CompareOrdinal(x.Name, y.Name))));
        foreach (Type aTest in aTests)
        {
            var aPlan = (CorpusPlan)Static(aTest, nameof(ICorpusTest.Corpus));
            Assert.Equal(Enumerable.Range(0, aPlan.Shards), aShards.Where(s => s.Test == aTest).Select(s => s.Index).Order());
        }
    }

    private static object Static(Type theType, string theName)
        => theType.GetProperty(theName, BindingFlags.Public | BindingFlags.Static)!.GetValue(null)!;
}
