namespace Kerf.Modeling.Tests.Immutability;

using Kerf.Modeling;

/// <summary>
/// How the ambient scopes, <see cref="Cancellation.Use"/>, <see cref="Warnings.Capture"/> and <see cref="SkipClean"/>,
/// behave on many threads and in async code (unit 14G, for the guide's "Running operations in parallel"). Each keeps its
/// state in an <see cref="AsyncLocal{T}"/>: it follows its own thread and async flow, flows into the tasks and parallel
/// loops started inside it, and isn't seen by other callers. A scope must end on the flow that opened it, as a
/// <c>using</c> ends it: one ended on another flow stays open where it was opened (the last test).
/// </summary>
public class AmbientScopesTests
{
    // A Boolean the clean changes: two boxes side by side fused, 6 faces cleaned, 10 not
    private static int FusedFaces() => (new Box(10, 10, 10) + new Pos(10, 0, 0) * new Box(10, 10, 10)).Faces().Count;

    private static void Warn(string message) => Warnings.Warn(message);

    [Fact]
    public void CallersOnOtherThreadsDontSeeEachOthersScopes()
    {
        Assert.Equal(6, FusedFaces());
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        using var barrier = new Barrier(2);
        int faces = 0;
        IReadOnlyList<string>? warnings = null, otherWarnings = null;
        Exception? stopped = null;
        var skipping = new Thread(() =>
        {
            using var scope = Warnings.Capture();
            using (new SkipClean())
            using (Cancellation.Use(cancelled.Token))
            {
                barrier.SignalAndWait();   // both scopes open at once
                Warn("mine");
                try
                {
                    _ = new Box(1, 1, 1) + new Pos(1, 0, 0) * new Box(1, 1, 1);
                }
                catch (OperationCanceledException e)
                {
                    stopped = e;
                }
                barrier.SignalAndWait();
            }
            warnings = scope.Messages;
        });
        var other = new Thread(() =>
        {
            using var scope = Warnings.Capture();
            barrier.SignalAndWait();
            faces = FusedFaces();   // cleaned, and not cancelled
            Warn("theirs");
            barrier.SignalAndWait();
            otherWarnings = scope.Messages;
        });
        skipping.Start();
        other.Start();
        skipping.Join();
        other.Join();
        Assert.NotNull(stopped);
        Assert.Equal(6, faces);
        Assert.Equal(["mine"], warnings);
        Assert.Equal(["theirs"], otherWarnings);
        Assert.True(SkipClean.Clean);
        Assert.False(Cancellation.Current.CanBeCanceled);
    }

    [Fact]
    public void ScopesFlowIntoTheLoopsAndTasksStartedInside()
    {
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        using (var scope = Warnings.Capture())
        using (new SkipClean())
        {
            int[] faces = new int[4];
            Parallel.For(0, faces.Length, new ParallelOptions { MaxDegreeOfParallelism = 2 }, i =>
            {
                faces[i] = FusedFaces();
                Warn($"body {i}");
            });
            Assert.All(faces, f => Assert.Equal(10, f));   // not cleaned: the SkipClean flowed in
            Assert.Equal(4, scope.Messages.Count);   // every body's warning collected

            using (Cancellation.Use(cancelled.Token))
            {
                var stopped = Assert.Throws<AggregateException>(() => Parallel.For(0, 2, new ParallelOptions { MaxDegreeOfParallelism = 2 }, i =>
                {
                    Part box = new Box(5, 5, 5);
                    _ = Operations.Fillet(box, box.Edges(), 1);
                }));
                Assert.All(stopped.InnerExceptions, e => Assert.IsAssignableFrom<OperationCanceledException>(e));
            }
        }

        Assert.True(SkipClean.Clean);
    }

    [Fact]
    public void AnOperationsBodyOnTheCallingThreadLeavesTheCallerAsItWas()
    {
        // Parallel.For runs bodies on the calling thread too: a scope a body opens and doesn't end stays in the body
        Parallel.For(0, 4, new ParallelOptions { MaxDegreeOfParallelism = 2 }, i =>
        {
            _ = new SkipClean();
            _ = Warnings.Capture();
        });
        Assert.True(SkipClean.Clean);
        Assert.Equal(6, FusedFaces());
    }

    [Fact]
    public async Task ScopesFollowAwaitsAndDontLeakOutOfAnAsyncMethod()
    {
        using (var scope = Warnings.Capture())
        using (new SkipClean())
        {
            await Task.Yield();   // the rest may run on another thread
            Assert.False(SkipClean.Clean);
            Warn("after the await");
            await Task.Run(() => Warn("in a task"));
            Assert.Equal(["after the await", "in a task"], scope.Messages);
        }

        Assert.True(SkipClean.Clean);

        // An async method's scopes end with it, even unended: its changes to the flow don't reach its caller
        await OpenAndForget();
        Assert.True(SkipClean.Clean);

        static async Task OpenAndForget()
        {
            _ = new SkipClean();
            await Task.Yield();
        }
    }

    [Fact]
    public async Task AScopeEndedOnAnotherFlowStaysOpenOnItsOwn()
    {
        // Found by unit 14G, for 14F: Dispose ends a scope on the flow it runs on. Run on another flow (a task's), it
        // ends nothing where the scope was opened, and marks it ended, so the using there can't end it either: the
        // opening flow skips the clean, and collects warnings, until it finishes.
        var task = Task.Run(async () =>
        {
            var skip = new SkipClean();
            var scope = Warnings.Capture();
            await Task.Run(() =>
            {
                skip.Dispose();
                scope.Dispose();
            });
            skip.Dispose();   // already disposed: does nothing
            scope.Dispose();
            Warn("after both ended");
            return (SkipClean.Clean, scope.Messages);
        });
        var (clean, messages) = await task;
        Assert.False(clean);
        Assert.Equal(["after both ended"], messages);
    }

    [Fact]
    public async Task ATaskThatOutlivesItsScopeStillWarnsIntoIt()
    {
        // A task started inside a scope keeps the scope after it ends: its later warnings still go to it
        using var release = new SemaphoreSlim(0);
        Task late;
        WarningScope scope;
        using (scope = Warnings.Capture())
        {
            late = Task.Run(async () =>
            {
                await release.WaitAsync();
                Warn("after the scope ended");
            });
        }

        release.Release();
        await late;
        Assert.Equal(["after the scope ended"], scope.Messages);
    }
}
