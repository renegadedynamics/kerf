// Port of: src/FoundationClasses/TKMath/GTests/gp_Pln_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class gp_PlnTest
{
    [Fact]
    public void ConstructorFromAx3()
    {
        gp_Ax3 anAx3 = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Pln aPln = new(anAx3);
        Assert.True(aPln.Location().IsEqual(new gp_Pnt(0.0, 0.0, 0.0), Precision.Confusion()));
        Assert.True(aPln.Axis().Direction().IsEqual(new gp_Dir(0.0, 0.0, 1.0), Precision.Angular()));
    }

    [Fact]
    public void ConstructorFromPointAndDir()
    {
        gp_Pnt aLoc = new(1.0, 2.0, 3.0);
        gp_Dir aNorm = new(0.0, 0.0, 1.0);
        gp_Pln aPln = new(aLoc, aNorm);
        Assert.True(aPln.Location().IsEqual(aLoc, Precision.Confusion()));
        Assert.True(aPln.Axis().Direction().IsEqual(aNorm, Precision.Angular()));
    }

    [Fact]
    public void ConstructorFromCoefficients()
    {
        // Plane Z = 5 => 0*X + 0*Y + 1*Z - 5 = 0
        gp_Pln aPln = new(0.0, 0.0, 1.0, -5.0);
        aPln.Coefficients(out double anA, out double aB, out double aC, out double aD);
        Assert.Equal(0.0, anA, Precision.Confusion());
        Assert.Equal(0.0, aB, Precision.Confusion());
        Assert.Equal(1.0, aC, Precision.Confusion());
        Assert.Equal(-5.0, aD, Precision.Confusion());
    }

    [Fact]
    public void DistanceToPoint()
    {
        gp_Pln aPln = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Pnt aPnt = new(3.0, 4.0, 5.0);
        Assert.Equal(5.0, aPln.Distance(aPnt), Precision.Confusion());
    }

    [Fact]
    public void Contains()
    {
        gp_Pln aPln = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Pnt aOnPlane = new(3.0, 4.0, 0.0);
        gp_Pnt aOffPlane = new(3.0, 4.0, 1.0);
        Assert.True(aPln.Contains(aOnPlane, Precision.Confusion()));
        Assert.False(aPln.Contains(aOffPlane, Precision.Confusion()));
    }

    [Fact]
    public void Coefficients()
    {
        gp_Pln aPln = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        aPln.Coefficients(out double anA, out double aB, out double aC, out double aD);
        Assert.Equal(0.0, anA, Precision.Confusion());
        Assert.Equal(0.0, aB, Precision.Confusion());
        Assert.Equal(1.0, aC, Precision.Confusion());
        Assert.Equal(0.0, aD, Precision.Confusion());
    }

    [Fact]
    public void Mirror()
    {
        gp_Pln aPln = new(new gp_Pnt(0.0, 0.0, 5.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Ax2 aMirrorPlane = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Pln aMirrored = aPln.Mirrored(aMirrorPlane);
        Assert.Equal(-5.0, aMirrored.Location().Z(), Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        gp_Pln aPln = new(new gp_Pnt(1.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Ax1 anAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Pln aRotated = aPln.Rotated(anAxis, M_PI_2);
        Assert.Equal(0.0, aRotated.Location().X(), Precision.Confusion());
        Assert.Equal(1.0, aRotated.Location().Y(), Precision.Confusion());
    }

    [Fact]
    public void Scale()
    {
        gp_Pln aPln = new(new gp_Pnt(1.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Pnt aCenter = new(0.0, 0.0, 0.0);
        gp_Pln aScaled = aPln.Scaled(aCenter, 3.0);
        Assert.Equal(3.0, aScaled.Location().X(), Precision.Confusion());
    }

    [Fact]
    public void Translate()
    {
        gp_Pln aPln = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Vec aVec = new(1.0, 2.0, 3.0);
        gp_Pln aTranslated = aPln.Translated(aVec);
        Assert.Equal(1.0, aTranslated.Location().X(), Precision.Confusion());
        Assert.Equal(2.0, aTranslated.Location().Y(), Precision.Confusion());
        Assert.Equal(3.0, aTranslated.Location().Z(), Precision.Confusion());
    }

    [Fact]
    public void Transform()
    {
        gp_Pln aPln = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Trsf aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec(0.0, 0.0, 10.0));
        gp_Pln aTransformed = aPln.Transformed(aTrsf);
        Assert.Equal(10.0, aTransformed.Location().Z(), Precision.Confusion());
    }
}
