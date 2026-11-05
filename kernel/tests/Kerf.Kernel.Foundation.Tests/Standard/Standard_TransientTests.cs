// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Kernel.Foundation.Tests;

public class Standard_TransientTests
{
    private sealed class Counter : Transient
    {
        public int Value { get; private set; }

        internal void Increment()
        {
            CheckMutable();
            Value++;
        }
    }

    [Fact]
    public void A_frozen_object_refuses_its_mutators()
    {
        var aCounter = new Counter();
        aCounter.Increment();
        Assert.False(aCounter.IsFrozen);

        aCounter.Freeze();
        aCounter.Freeze();

        Assert.True(aCounter.IsFrozen);
        EngineAssert.Throws<ImmutableObjectException>(aCounter.Increment);
        Assert.Equal(1, aCounter.Value);
    }

    [Fact]
    public void Runtime_type_queries_follow_dotnet_types()
    {
        Transient anObject = new Counter();

        Assert.True(anObject.IsKind<Transient>());
        Assert.True(anObject.IsInstance(typeof(Counter)));
        Assert.False(anObject.IsInstance(typeof(Transient)));
        Assert.Equal(typeof(Counter), anObject.DynamicType());
    }
}
