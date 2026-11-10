namespace Kerf.Modeling.Tests.Immutability;

using Kerf.Modeling;

/// <summary>
/// How the ambient scopes, <see cref="Cancellation.Use"/>, <see cref="Warnings.Capture"/> and <see cref="SkipClean"/>,
/// behave on many threads and in async code (unit 14G, for the guide's "Running operations in parallel"). Each keeps its
/// state in an <see cref="AsyncLocal{T}"/>: it follows its own thread and async flow, flows into the tasks and parallel
/// loops started inside it, and isn't seen by other callers. A scope's lifetime is its own (unit 14F): disposed on any
/// flow, it ends on the flow that opened it and in every task started from it, and nothing reaches it after.
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
    public async Task AScopeEndedOnAnotherFlowEndsWhereItWasOpened()
    {
        // Found by unit 14G, fixed by 14F: Dispose ended a scope only on the flow it ran on. A scope's lifetime is its
        // own: disposed in a task, it ends on the flow that opened it too, and a second Dispose there does nothing
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        var task = Task.Run(async () =>
        {
            var skip = new SkipClean();
            var scope = Warnings.Capture();
            var cancel = Cancellation.Use(cancelled.Token);
            await Task.Run(() =>
            {
                skip.Dispose();
                scope.Dispose();
                cancel.Dispose();
            });
            bool clean = SkipClean.Clean;
            bool cancelling = Cancellation.Current.CanBeCanceled;
            Warn("after all three ended");
            skip.Dispose();   // already ended: does nothing
            scope.Dispose();
            cancel.Dispose();
            return (clean, cancelling, scope.Messages, FusedFaces());
        });
        var (clean, cancelling, messages, faces) = await task;
        Assert.True(clean);
        Assert.False(cancelling);
        Assert.Empty(messages);
        Assert.Equal(6, faces);
    }

    [Fact]
    public async Task AScopeEndedOnAnotherFlowLeavesTheScopesOutsideItOpen()
    {
        var task = Task.Run(async () =>
        {
            using var outerSkip = new SkipClean();
            using var outer = Warnings.Capture();
            var innerSkip = new SkipClean();
            var inner = Warnings.Capture();
            await Task.Run(() =>
            {
                innerSkip.Dispose();
                inner.Dispose();
            });
            Warn("after the inner ended");
            // The outer scopes are open: the clean skipped, the warning theirs
            return (SkipClean.Clean, outer.Messages, inner.Messages, FusedFaces());
        });
        var (clean, outerMessages, innerMessages, faces) = await task;
        Assert.False(clean);
        Assert.Equal(["after the inner ended"], outerMessages);
        Assert.Empty(innerMessages);
        Assert.Equal(10, faces);
    }

    [Fact]
    public void ScopesEndedOutOfOrderLeaveTheInnerOneOpen()
    {
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        var outerSkip = new SkipClean();
        var outer = Warnings.Capture();
        var outerCancel = Cancellation.Use(cancelled.Token);
        var innerSkip = new SkipClean();
        var inner = Warnings.Capture();
        var innerCancel = Cancellation.Use(CancellationToken.None);
        try
        {
            outerSkip.Dispose();
            outer.Dispose();
            outerCancel.Dispose();
            Warn("inner only");
            Assert.False(SkipClean.Clean);
            Assert.False(Cancellation.Current.CanBeCanceled);
            Assert.Equal(["inner only"], inner.Messages);
            Assert.Empty(outer.Messages);
        }
        finally
        {
            innerCancel.Dispose();
            inner.Dispose();
            innerSkip.Dispose();
        }

        Assert.True(SkipClean.Clean);
        Assert.False(Cancellation.Current.CanBeCanceled);
    }

    [Fact]
    public async Task ATaskThatOutlivesItsScopesNoLongerUsesThem()
    {
        // Fixed by 14F: a task started inside a scope that outlives it doesn't keep it; its later warnings go to the
        // enclosing scope still open, it cleans again, and the token in force is the one outside
        using var release = new SemaphoreSlim(0);
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        Task<(bool Clean, bool Cancelling, int Faces)> late;
        WarningScope scope;
        using var outer = Warnings.Capture();
        using (scope = Warnings.Capture())
        using (new SkipClean())
        using (Cancellation.Use(cancelled.Token))
        {
            Warn("inside");
            late = Task.Run(async () =>
            {
                await release.WaitAsync();
                Warn("after the scope ended");
                return (SkipClean.Clean, Cancellation.Current.CanBeCanceled, FusedFaces());
            });
        }

        release.Release();
        var (clean, cancelling, faces) = await late;
        Assert.Equal(["inside"], scope.Messages);
        Assert.Equal(["inside", "after the scope ended"], outer.Messages);
        Assert.True(clean);
        Assert.False(cancelling);
        Assert.Equal(6, faces);
    }

    [Fact]
    public async Task NothingReachesAScopeAfterItsDisposeReturns()
    {
        // A scope ended while other threads are warning into it: whatever it holds was raised before Dispose returned,
        // and nothing reaches it after
        for (int trial = 0; trial < 20; trial++)
        {
            var scope = Warnings.Capture();
            using var go = new ManualResetEventSlim();
            int raisedAfter = 0;
            bool ended = false;
            var warners = Enumerable.Range(0, 2).Select(_ => Task.Run(() =>
            {
                go.Wait();
                for (int i = 0; i < 500; i++)
                {
                    if (Volatile.Read(ref ended))
                    {
                        Warn("late");
                        Interlocked.Increment(ref raisedAfter);
                        if (raisedAfter > 50)
                            return;
                    }
                    else
                    {
                        Warn("early");
                    }
                }
            })).ToArray();
            go.Set();
            Thread.Sleep(1);
            scope.Dispose();
            Volatile.Write(ref ended, true);
            await Task.WhenAll(warners);
            Assert.DoesNotContain("late", scope.Messages);
        }

        Assert.True(SkipClean.Clean);
    }
}
