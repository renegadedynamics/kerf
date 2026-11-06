// Copyright (c) 2026 The Kerf Contributors. Apache License 2.0 (see LICENSE).

using Kerf.Modeling;
using Xunit;

namespace Kerf.Modeling.Tests.Topology;

public class KeyComparerTests
{
    private static readonly double[] Values =
        [double.NaN, double.NegativeInfinity, -1.5, -0.0, 0.0, double.Epsilon, 1.5, double.PositiveInfinity];

    [Fact]
    public void DoublesCompareAsObjectsDo()
    {
        foreach (double x in Values)
        {
            foreach (double y in Values)
            {
                Assert.Equal(Comparer<object>.Default.Compare(x, y), KeyComparer<double>.Instance.Compare(x, y));
            }
        }
    }
}
