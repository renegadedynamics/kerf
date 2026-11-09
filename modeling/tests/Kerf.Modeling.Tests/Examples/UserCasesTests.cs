using Cases = Kerf.UserCases.UserCases;

namespace Kerf.Modeling.Tests.Examples;

/// <summary>
/// Users' cases (kernel/benchmarks/Kerf.UserCases, unit 13C): each gives its fixed result on this tree, twice from the
/// same operands as the rig and the release check run them, so a change that moves one fails here before they time it.
/// </summary>
/// <remarks>
/// They run in this process. They once ran in a process of their own, as their thousands of operations warmed this one's
/// runtime until the kernel's refusals in other tests escaped as bare argument exceptions (decision 2017); the layer now
/// tells those by their type, wherever the JIT compiled the check (decision 25801).
/// </remarks>
public class UserCasesTests
{
    public static TheoryData<string> Names() => [.. Cases.All.Select(c => c.Name)];

    [Theory]
    [MemberData(nameof(Names))]
    public void Each_gives_its_fixed_result(string name)
    {
        var c = Cases.All.Single(c => c.Name == name);
        var operation = c.Prepare(Path.Combine(AppContext.BaseDirectory, "data"));
        Assert.NotNull(operation);
        // Twice from the same operands, as the program's check runs them: the second run builds its shapes again (the
        // enclosure keeps its parts in static fields) and gives the same result
        var first = operation();
        var second = operation();
        Assert.False(ReferenceEquals(first, second), "the second run gave the first run's object");
        if (first is List<(string, Part)> a && second is List<(string, Part)> b)
            Assert.DoesNotContain(a.Zip(b), p => ReferenceEquals(p.First.Item2, p.Second.Item2));
        Assert.True(c.Read(first).Matches(c.Expected), $"first run: {c.Read(first).Summary()}");
        Assert.True(c.Read(second).Matches(c.Expected), $"second run: {c.Read(second).Summary()}");
    }
}
