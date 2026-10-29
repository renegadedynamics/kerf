// Port of: src/FoundationClasses/TKMath/GTests/BSplSLib_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class BSplSLibTest
{
    [Fact]
    public void UnitWeights_SmallSurface_ReturnsNonOwning()
    {
        const int aNbU = 4;
        const int aNbV = 5;
        NCollection_Array2<double> aWeights = BSplSLib.UnitWeights(aNbU, aNbV);

        Assert.Equal(aNbU, aWeights.ColLength());
        Assert.Equal(aNbV, aWeights.RowLength());
        Assert.Equal(aNbU * aNbV, aWeights.Size());
        Assert.False(aWeights.IsDeletable());

        for (int i = 1; i <= aNbU; ++i)
        {
            for (int j = 1; j <= aNbV; ++j)
            {
                Assert.Equal(1.0, aWeights[i, j]);
            }
        }
    }

    [Fact]
    public void UnitWeights_MaxBezierSize_ReturnsNonOwning()
    {
        // Max Bezier: 26x26 = 676 poles, well within limit
        const int aNbU = 26;
        const int aNbV = 26;
        NCollection_Array2<double> aWeights = BSplSLib.UnitWeights(aNbU, aNbV);

        Assert.Equal(aNbU * aNbV, aWeights.Size());
        Assert.False(aWeights.IsDeletable());
        Assert.Equal(1.0, aWeights[1, 1]);
        Assert.Equal(1.0, aWeights[aNbU, aNbV]);
    }

    [Fact]
    public void UnitWeights_AtMaxLimit_ReturnsNonOwning()
    {
        // Exactly at the limit: 2049 total poles
        const int aNbU = 3;
        const int aNbV = 683;
        NCollection_Array2<double> aWeights = BSplSLib.UnitWeights(aNbU, aNbV);

        Assert.Equal(aNbU * aNbV, aWeights.Size());
        Assert.False(aWeights.IsDeletable());
        Assert.Equal(1.0, aWeights[1, 1]);
        Assert.Equal(1.0, aWeights[aNbU, aNbV]);
    }

    [Fact]
    public void UnitWeights_OverMaxLimit_ReturnsOwning()
    {
        // Over the limit: needs allocation
        const int aNbU = 50;
        const int aNbV = 50;
        NCollection_Array2<double> aWeights = BSplSLib.UnitWeights(aNbU, aNbV);

        Assert.Equal(aNbU, aWeights.ColLength());
        Assert.Equal(aNbV, aWeights.RowLength());
        Assert.Equal(aNbU * aNbV, aWeights.Size());
        Assert.True(aWeights.IsDeletable());

        for (int i = 1; i <= aNbU; ++i)
        {
            for (int j = 1; j <= aNbV; ++j)
            {
                Assert.Equal(1.0, aWeights[i, j]);
            }
        }
    }

    [Fact]
    public void UnitWeights_SingleElement()
    {
        NCollection_Array2<double> aWeights = BSplSLib.UnitWeights(1, 1);

        Assert.Equal(1, aWeights.Size());
        Assert.False(aWeights.IsDeletable());
        Assert.Equal(1.0, aWeights[1, 1]);
    }
}
