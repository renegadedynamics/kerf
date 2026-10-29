// Port of: src/ModelingData/TKG3d/GTests/Geom_Line_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom_LineTest
{
    [Fact]
    public void ConstructFromAx1()
    {
        gp_Ax1 anAx1 = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        Geom_Line aLine = new(anAx1);
        gp_Pnt aLoc = aLine.Lin().Location();
        Assert.True(aLoc.IsEqual(new gp_Pnt(0.0, 0.0, 0.0), Precision.Confusion()));
    }

    [Fact]
    public void ConstructFromPointAndDir()
    {
        Geom_Line aLine = new(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(0.0, 0.0, 1.0));
        gp_Pnt aLoc = aLine.Lin().Location();
        Assert.True(aLoc.IsEqual(new gp_Pnt(1.0, 2.0, 3.0), Precision.Confusion()));
        Assert.True(aLine.Lin().Direction().IsEqual(new gp_Dir(0.0, 0.0, 1.0), Precision.Angular()));
    }

    [Fact]
    public void D0Evaluation()
    {
        Geom_Line aLine = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        aLine.D0(5.0, out gp_Pnt aPnt);
        Assert.Equal(5.0, aPnt.X(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Y(), Precision.Confusion());
        Assert.Equal(0.0, aPnt.Z(), Precision.Confusion());
    }

    [Fact]
    public void D1Evaluation()
    {
        Geom_Line aLine = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 1.0, 0.0));
        aLine.D1(3.0, out gp_Pnt aPnt, out gp_Vec aV1);
        Assert.Equal(3.0, aPnt.Y(), Precision.Confusion());
        // First derivative is the direction vector (constant for a line)
        Assert.Equal(0.0, aV1.X(), Precision.Confusion());
        Assert.Equal(1.0, aV1.Y(), Precision.Confusion());
        Assert.Equal(0.0, aV1.Z(), Precision.Confusion());
    }

    [Fact]
    public void D2Evaluation()
    {
        Geom_Line aLine = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        aLine.D2(1.0, out _, out _, out gp_Vec aV2);
        // Second derivative of a line is zero
        Assert.Equal(0.0, aV2.Magnitude(), Precision.Confusion());
    }

    [Fact]
    public void InfiniteParameters()
    {
        Geom_Line aLine = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        Assert.Equal(-Precision.Infinite(), aLine.FirstParameter());
        Assert.Equal(Precision.Infinite(), aLine.LastParameter());
    }

    [Fact]
    public void Reverse()
    {
        Geom_Line aLine = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        aLine.Reverse();
        Assert.True(aLine.Lin().Direction().IsEqual(new gp_Dir(-1.0, 0.0, 0.0), Precision.Angular()));
    }

    [Fact]
    public void Reversed()
    {
        Geom_Line aLine = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        Geom_Curve aReversed = aLine.Reversed();
        Geom_Line? aRevLine = aReversed as Geom_Line;
        Assert.NotNull(aRevLine);
        Assert.True(aRevLine.Lin().Direction().IsEqual(new gp_Dir(-1.0, 0.0, 0.0), Precision.Angular()));
    }

    [Fact]
    public void Transform()
    {
        Geom_Line aLine = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        gp_Trsf aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec(0.0, 0.0, 5.0));
        aLine.Transform(aTrsf);
        Assert.Equal(5.0, aLine.Lin().Location().Z(), Precision.Confusion());
    }

    [Fact]
    public void Copy()
    {
        Geom_Line aLine = new(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(0.0, 0.0, 1.0));
        Geom_Geometry aCopy = aLine.Copy();
        Geom_Line? aCopyLine = aCopy as Geom_Line;
        Assert.NotNull(aCopyLine);
        Assert.True(aCopyLine.Lin().Location().IsEqual(new gp_Pnt(1.0, 2.0, 3.0), Precision.Confusion()));
    }
}
