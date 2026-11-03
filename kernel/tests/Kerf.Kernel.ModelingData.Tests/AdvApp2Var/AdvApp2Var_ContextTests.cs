// Port of: src/ModelingData/TKGeomBase/GTests/AdvApp2Var_Context_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation.Advanced;

namespace Kerf.Kernel.ModelingData.Tests;

public class AdvApp2Var_ContextTest
{
    [Fact]
    public void Tolerances_AreAggregatedAndRescaledForBorderConstraints()
    {
        NCollection_HArray1<double> aTol1D = new(1, 1);
        NCollection_HArray1<double> aTol2D = new(1, 1);
        NCollection_HArray1<double> aTol3D = new(1, 1);
        aTol1D.SetValue(1, 6.0);
        aTol2D.SetValue(1, 12.0);
        aTol3D.SetValue(1, 18.0);

        NCollection_HArray2<double> aTof1D = new(1, 1, 1, 4);
        NCollection_HArray2<double> aTof2D = new(1, 1, 1, 4);
        NCollection_HArray2<double> aTof3D = new(1, 1, 1, 4);
        aTof1D.Init(100.0);
        aTof2D.Init(100.0);
        aTof3D.Init(100.0);

        Context aContext = new(0, 0, 0, 2, 2, 0, 1, 1, 1, aTol1D, aTol2D, aTol3D, aTof1D, aTof2D, aTof3D);

        Assert.Equal(3, aContext.TotalNumberSSP());
        Assert.Equal(6, aContext.TotalDimension());

        NCollection_HArray1<double>? anITol = aContext.IToler();
        Assert.NotNull(anITol);
        Assert.Equal(3.0, anITol.Value(1));
        Assert.Equal(6.0, anITol.Value(2));
        Assert.Equal(9.0, anITol.Value(3));

        NCollection_HArray2<double>? aFTol = aContext.FToler();
        NCollection_HArray2<double>? aCTol = aContext.CToler();
        Assert.NotNull(aFTol);
        Assert.NotNull(aCTol);

        for (int aTolIndex = 1; aTolIndex <= 4; ++aTolIndex)
        {
            Assert.Equal(1.0, aFTol.Value(1, aTolIndex));
            Assert.Equal(2.0, aFTol.Value(2, aTolIndex));
            Assert.Equal(3.0, aFTol.Value(3, aTolIndex));

            Assert.Equal(1.0, aCTol.Value(1, aTolIndex));
            Assert.Equal(2.0, aCTol.Value(2, aTolIndex));
            Assert.Equal(3.0, aCTol.Value(3, aTolIndex));
        }
    }
}
