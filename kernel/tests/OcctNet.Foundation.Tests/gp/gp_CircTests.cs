// Port of: src/FoundationClasses/TKMath/GTests/gp_Circ_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class gp_CircTest
{
    [Fact]
    public void Constructor()
    {
        gp_Ax2 anAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Circ aCirc = new(anAxis, 5.0);
        Assert.Equal(5.0, aCirc.Radius(), Precision.Confusion());
    }

    [Fact]
    public void Area()
    {
        gp_Ax2 anAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Circ aCirc = new(anAxis, 1.0);
        Assert.Equal(M_PI, aCirc.Area(), Precision.Confusion());
    }

    [Fact]
    public void Length()
    {
        gp_Ax2 anAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Circ aCirc = new(anAxis, 1.0);
        Assert.Equal(2.0 * M_PI, aCirc.Length(), Precision.Confusion());
    }

    [Fact]
    public void Center()
    {
        gp_Pnt aCenter = new(1.0, 2.0, 3.0);
        gp_Ax2 anAxis = new(aCenter, new gp_Dir(0.0, 0.0, 1.0));
        gp_Circ aCirc = new(anAxis, 5.0);
        Assert.True(aCirc.Location().IsEqual(aCenter, Precision.Confusion()));
    }

    [Fact]
    public void Translate()
    {
        gp_Ax2 anAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Circ aCirc = new(anAxis, 5.0);
        gp_Vec aVec = new(10.0, 20.0, 30.0);
        gp_Circ aTranslated = aCirc.Translated(aVec);
        Assert.Equal(10.0, aTranslated.Location().X(), Precision.Confusion());
        Assert.Equal(20.0, aTranslated.Location().Y(), Precision.Confusion());
        Assert.Equal(30.0, aTranslated.Location().Z(), Precision.Confusion());
        Assert.Equal(5.0, aTranslated.Radius(), Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        gp_Ax2 anAxis = new(new gp_Pnt(1.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Circ aCirc = new(anAxis, 2.0);
        gp_Ax1 aRotAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Circ aRotated = aCirc.Rotated(aRotAxis, M_PI_2);
        Assert.Equal(0.0, aRotated.Location().X(), Precision.Confusion());
        Assert.Equal(1.0, aRotated.Location().Y(), Precision.Confusion());
        Assert.Equal(2.0, aRotated.Radius(), Precision.Confusion());
    }

    [Fact]
    public void Mirror()
    {
        gp_Ax2 anAxis = new(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Circ aCirc = new(anAxis, 5.0);
        gp_Pnt aMirrorPnt = new(0.0, 0.0, 0.0);
        gp_Circ aMirrored = aCirc.Mirrored(aMirrorPnt);
        Assert.Equal(-1.0, aMirrored.Location().X(), Precision.Confusion());
        Assert.Equal(-2.0, aMirrored.Location().Y(), Precision.Confusion());
        Assert.Equal(-3.0, aMirrored.Location().Z(), Precision.Confusion());
    }

    [Fact]
    public void Transform()
    {
        gp_Ax2 anAxis = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Circ aCirc = new(anAxis, 3.0);
        gp_Trsf aTrsf = new();
        aTrsf.SetScale(new gp_Pnt(0.0, 0.0, 0.0), 2.0);
        gp_Circ aTransformed = aCirc.Transformed(aTrsf);
        Assert.Equal(6.0, aTransformed.Radius(), Precision.Confusion());
    }
}
