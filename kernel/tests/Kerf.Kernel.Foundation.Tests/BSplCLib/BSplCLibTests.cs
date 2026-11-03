// Port of: src/FoundationClasses/TKMath/GTests/BSplCLib_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.Foundation.Tests;

public class BSplCLibTest
{
    [Fact]
    public void UnitWeights_SmallSize_ReturnsNonOwning()
    {
        const int aNbElems = 10;
        NCollection_Array1<double> aWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(aNbElems);

        Assert.Equal(1, aWeights.Lower());
        Assert.Equal(aNbElems, aWeights.Upper());
        Assert.Equal(aNbElems, aWeights.Length());
        Assert.False(aWeights.IsDeletable());

        for (int i = 1; i <= aNbElems; ++i)
        {
            Assert.Equal(1.0, aWeights[i]);
        }
    }

    [Fact]
    public void UnitWeights_MaxSize_ReturnsNonOwning()
    {
        int aNbElems = global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxUnitWeightsSize();
        NCollection_Array1<double> aWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(aNbElems);

        Assert.Equal(aNbElems, aWeights.Length());
        Assert.False(aWeights.IsDeletable());

        Assert.Equal(1.0, aWeights[1]);
        Assert.Equal(1.0, aWeights[aNbElems]);
    }

    [Fact]
    public void UnitWeights_OverMaxSize_ReturnsOwning()
    {
        int aNbElems = global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxUnitWeightsSize() + 1;
        NCollection_Array1<double> aWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(aNbElems);

        Assert.Equal(1, aWeights.Lower());
        Assert.Equal(aNbElems, aWeights.Upper());
        Assert.Equal(aNbElems, aWeights.Length());
        Assert.True(aWeights.IsDeletable());

        for (int i = 1; i <= aNbElems; ++i)
        {
            Assert.Equal(1.0, aWeights[i]);
        }
    }

    [Fact]
    public void UnitWeights_SingleElement()
    {
        NCollection_Array1<double> aWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(1);

        Assert.Equal(1, aWeights.Length());
        Assert.False(aWeights.IsDeletable());
        Assert.Equal(1.0, aWeights[1]);
    }

    [Fact]
    public void MaxUnitWeightsSize_IsPositive()
    {
        Assert.True(global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxUnitWeightsSize() > 0);
        Assert.Equal(2049, global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxUnitWeightsSize());
    }
}
