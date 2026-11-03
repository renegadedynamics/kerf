// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Tests of the per-operation context (unit 10I; docs/state.md).

using Kerf.Engine.Foundation;

namespace Kerf.Kernel.Foundation.Tests.Context;

public class OperationContextTests
{
    private static readonly ContextKey<int> Number = new("Tests.Number", 7);
    private static readonly ContextKey<string> Text = new("Tests.Text", "default");

    [Fact]
    public void A_setting_has_its_default_until_a_context_overrides_it()
    {
        Assert.Equal(7, Number.Value);
        Assert.Equal(7, OperationContext.Default.Get(Number));
        Assert.False(OperationContext.Default.TryGet(Number, out _));

        OperationContext context = OperationContext.Default.With(Number, 3).With(Text, "set");
        Assert.Equal(2, context.Count);
        Assert.Equal(3, context.Get(Number));
        Assert.Equal("set", context.Get(Text));
        Assert.Equal(9, context.With(Number, 9).Get(Number));
        Assert.Equal(3, context.Get(Number)); // contexts are immutable
        Assert.Equal(7, context.Without(Number).Get(Number));
        Assert.Same(OperationContext.Default, OperationContext.Default.With(Number, 1).Without(Number));
        Assert.Throws<ArgumentException>(() => new ContextKey<int>("", 0));
    }

    [Fact]
    public void A_scope_sets_and_restores()
    {
        using (OperationContext.Begin(Number, 1))
        {
            Assert.Equal(1, Number.Value);
            using (OperationContext.Begin(Text, "inner"))
            {
                Assert.Equal(1, Number.Value); // nested scopes keep the outer settings
                Assert.Equal("inner", Text.Value);
            }
            Assert.Equal("default", Text.Value);
        }
        Assert.Equal(7, Number.Value);
        Assert.Same(OperationContext.Default, OperationContext.Current);
    }

    [Fact]
    public async Task A_setting_flows_into_tasks_and_not_back()
    {
        using (OperationContext.Begin(Number, 2))
        {
            Assert.Equal(2, await Task.Run(() => Number.Value));
            int[] seen = new int[8];
            Parallel.For(0, seen.Length, i => seen[i] = Number.Value);
            Assert.All(seen, value => Assert.Equal(2, value));
        }

        await Task.Run(() =>
        {
            Assert.Equal(7, OperationContext.Set(Number, 5)); // the previous value
            Assert.Equal(5, Number.Value);
        });
        Assert.Equal(7, Number.Value);
    }

    [Fact]
    public void Flows_on_other_threads_have_their_own()
    {
        var failures = 0;
        Parallel.For(0, 64, k =>
        {
            using (OperationContext.Begin(Number, k))
            {
                Thread.Sleep(1);
                if (Number.Value != k)
                {
                    Interlocked.Increment(ref failures);
                }
            }
        });
        Assert.Equal(0, failures);
    }
}
