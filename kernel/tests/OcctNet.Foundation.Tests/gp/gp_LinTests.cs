// Port of: src/FoundationClasses/TKMath/GTests/gp_Lin_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class gp_LinTest
{
    [Fact]
    public void ConstructorFromAx1()
    {
        gp_Ax1 anAx1 = new(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(1.0, 0.0, 0.0));
        gp_Lin aLin = new(anAx1);
        Assert.True(aLin.Location().IsEqual(new gp_Pnt(1.0, 2.0, 3.0), Precision.Confusion()));
        Assert.True(aLin.Direction().IsEqual(new gp_Dir(1.0, 0.0, 0.0), Precision.Angular()));
    }

    [Fact]
    public void ConstructorFromPointAndDir()
    {
        gp_Lin aLin = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        Assert.True(aLin.Location().IsEqual(new gp_Pnt(0.0, 0.0, 0.0), Precision.Confusion()));
        Assert.True(aLin.Direction().IsEqual(new gp_Dir(0.0, 0.0, 1.0), Precision.Angular()));
    }

    [Fact]
    public void DistanceToPoint()
    {
        gp_Lin aLin = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        gp_Pnt aPnt = new(5.0, 3.0, 4.0);
        // Distance from point to X axis: sqrt(3^2 + 4^2) = 5
        Assert.Equal(5.0, aLin.Distance(aPnt), Precision.Confusion());
    }

    [Fact]
    public void DistanceToLine_Parallel()
    {
        gp_Lin aL1 = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        gp_Lin aL2 = new(new gp_Pnt(0.0, 3.0, 4.0), new gp_Dir(1.0, 0.0, 0.0));
        Assert.Equal(5.0, aL1.Distance(aL2), Precision.Confusion());
    }

    [Fact]
    public void DistanceToLine_Intersecting()
    {
        gp_Lin aL1 = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        gp_Lin aL2 = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 1.0, 0.0));
        Assert.Equal(0.0, aL1.Distance(aL2), Precision.Confusion());
    }

    [Fact]
    public void Contains()
    {
        gp_Lin aLin = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        gp_Pnt aOnLine = new(5.0, 0.0, 0.0);
        gp_Pnt aOffLine = new(5.0, 1.0, 0.0);
        Assert.True(aLin.Contains(aOnLine, Precision.Confusion()));
        Assert.False(aLin.Contains(aOffLine, Precision.Confusion()));
    }

    [Fact]
    public void Angle()
    {
        gp_Lin aL1 = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        gp_Lin aL2 = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 1.0, 0.0));
        Assert.Equal(M_PI_2, aL1.Angle(aL2), Precision.Angular());
    }

    [Fact]
    public void Translate()
    {
        gp_Lin aLin = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        gp_Vec aVec = new(0.0, 5.0, 0.0);
        gp_Lin aTranslated = aLin.Translated(aVec);
        Assert.Equal(5.0, aTranslated.Location().Y(), Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        gp_Lin aLin = new(new gp_Pnt(1.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        gp_Ax1 anAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Lin aRotated = aLin.Rotated(anAxis, M_PI_2);
        Assert.Equal(0.0, aRotated.Location().X(), Precision.Confusion());
        Assert.Equal(1.0, aRotated.Location().Y(), Precision.Confusion());
        Assert.True(aRotated.Direction().IsEqual(new gp_Dir(0.0, 1.0, 0.0), Precision.Angular()));
    }

    [Fact]
    public void Mirror()
    {
        gp_Lin aLin = new(new gp_Pnt(1.0, 1.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        gp_Ax1 anAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        gp_Lin aMirrored = aLin.Mirrored(anAxis);
        Assert.Equal(-1.0, aMirrored.Location().Y(), Precision.Confusion());
    }

    [Fact]
    public void Transform()
    {
        gp_Lin aLin = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        gp_Trsf aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec(0.0, 0.0, 10.0));
        gp_Lin aTransformed = aLin.Transformed(aTrsf);
        Assert.Equal(10.0, aTransformed.Location().Z(), Precision.Confusion());
    }
}

/// <summary>The gp_Lin2d tests that OCCT keeps in gp_Lin_Test.cxx (OCC15489).</summary>
public class gp_Lin2dTest
{
    [Fact]
    public void ConstructFromEquation_ZeroDirection_ThrowsException()
    {
        // A=0, B=0 -> direction vector is zero -> must throw
        Assert.ThrowsAny<Standard_ConstructionError>(() => new gp_Lin2d(0.0, 0.0, 1.0));
    }

    [Fact]
    public void ConstructFromEquation_ValidCoefficients_CorrectOrigin()
    {
        // A=1e-20, B=-1, C=2 gives the line   1e-20 * x - y + 2 = 0
        // Origin (closest point to global origin on the line) must be
        // X_0 ~= -1.9999999999999999e-20, Y_0 ~= 2
        gp_Lin2d aLin2d = new(1e-20, -1.0, 2.0);
        gp_Pnt2d anOrigin = aLin2d.Location();
        Assert.Equal(-1.9999999999999999e-20, anOrigin.X(), 1e-25);
        Assert.Equal(2.0, anOrigin.Y(), 0.001);
    }
}
