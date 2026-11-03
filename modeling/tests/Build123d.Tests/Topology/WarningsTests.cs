namespace Build123d.Tests.Topology;

public class WarningsTests
{
    [Fact]
    public void AScopeCollectsTheWarningsRaisedInIt()
    {
        Warnings.Warn("before");
        var outer = Warnings.Capture();
        Warnings.Warn("one");
        using (var inner = Warnings.Capture())
        {
            Warnings.Warn("two");
            Assert.Equal(["two"], inner.Messages);
        }
        Warnings.Warn("three");
        Assert.Equal(["one", "two", "three"], outer.Messages);
    }

    [Fact]
    public async Task AScopeFollowsItsAsyncFlow()
    {
        var scope = Warnings.Capture();
        await Task.Run(() => Warnings.Warn("from a task"));
        Assert.Equal(["from a task"], scope.Messages);
    }

    [Fact]
    public void ConcurrentScopesDontSeeEachOthersWarnings()
    {
        var results = new IReadOnlyList<string>[8];
        var barrier = new Barrier(results.Length);
        Parallel.For(0, results.Length, new ParallelOptions { MaxDegreeOfParallelism = results.Length }, i =>
        {
            var scope = Warnings.Capture();
            barrier.SignalAndWait();   // every scope is open at once
            Warnings.Warn($"cell {i}");
            barrier.SignalAndWait();
            results[i] = scope.Messages;
        });
        for (int i = 0; i < results.Length; i++)
            Assert.Equal([$"cell {i}"], results[i]);
    }

    [Fact]
    public void AWarningOutsideAnyScopeGoesOnlyToTheTrace() => Warnings.Warn("nobody is listening");
}
