// Port of: src/ModelingData/TKGeomBase/GTests/IntAna_IntQuadQuad_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntAna_ResultType;

namespace OcctNet.ModelingData.Tests;

// The tests of IntAna_IntQuadQuad_Test that exercise IntAna_QuadQuadGeo.
public partial class IntAna_IntQuadQuad_Test
{
    // Regression test for near-tangent parallel cylinders.
    // Very small chord between two theoretical lines must collapse to one line.
    [Fact]
    public void CylinderCylinderNearTangent_CollapsesToSingleLine()
    {
        gp_Pnt aCenter1 = new(0.0, 0.0, 0.0);
        gp_Pnt aCenter2 = new(2.0 - 1.0e-9, 0.0, 0.0);
        gp_Dir aZDir = new(gp_Dir.D.Z);
        gp_Dir anXDir = new(gp_Dir.D.X);
        gp_Ax3 anAxis1 = new(aCenter1, aZDir, anXDir);
        gp_Ax3 anAxis2 = new(aCenter2, aZDir, anXDir);

        gp_Cylinder aCyl1 = new(anAxis1, 1.0);
        gp_Cylinder aCyl2 = new(anAxis2, 1.0);

        IntAna_QuadQuadGeo anInter = new(aCyl1, aCyl2, 1.0e-4);

        Assert.True(anInter.IsDone());
        Assert.Equal(IntAna_Line, anInter.TypeInter());
        Assert.Equal(1, anInter.NbSolutions());
    }

    // Clear non-tangent case must keep two-line solution.
    [Fact]
    public void CylinderCylinderNonTangent_HasTwoLines()
    {
        gp_Pnt aCenter1 = new(0.0, 0.0, 0.0);
        gp_Pnt aCenter2 = new(1.8, 0.0, 0.0);
        gp_Dir aZDir = new(gp_Dir.D.Z);
        gp_Dir anXDir = new(gp_Dir.D.X);
        gp_Ax3 anAxis1 = new(aCenter1, aZDir, anXDir);
        gp_Ax3 anAxis2 = new(aCenter2, aZDir, anXDir);

        gp_Cylinder aCyl1 = new(anAxis1, 1.0);
        gp_Cylinder aCyl2 = new(anAxis2, 1.0);

        IntAna_QuadQuadGeo anInter = new(aCyl1, aCyl2, 1.0e-7);

        Assert.True(anInter.IsDone());
        Assert.Equal(IntAna_Line, anInter.TypeInter());
        Assert.Equal(2, anInter.NbSolutions());
    }

    // Sanity regression for the pre-existing skew external-tangent branch.
    [Fact]
    public void CylinderCylinderSkewExternallyTangent_HasPoint()
    {
        double aR1 = 3.0;
        double aR2 = 2.0;
        double aDist = aR1 + aR2;

        gp_Ax3 anAxis1 = new(new gp_Pnt(0.0, 0.0, 0.0), gp_Dir.D.Z, gp_Dir.D.X);
        gp_Ax3 anAxis2 = new(new gp_Pnt(0.0, aDist, 0.0), gp_Dir.D.X, gp_Dir.D.Y);

        gp_Cylinder aCyl1 = new(anAxis1, aR1);
        gp_Cylinder aCyl2 = new(anAxis2, aR2);

        IntAna_QuadQuadGeo anInter = new(aCyl1, aCyl2, 1.0e-7);

        Assert.True(anInter.IsDone());
        Assert.Equal(IntAna_Point, anInter.TypeInter());
        Assert.Equal(1, anInter.NbSolutions());

        gp_Pnt aP = anInter.Point(1);
        Assert.Equal(aR1, new gp_Lin(aCyl1.Axis()).Distance(aP), 1.0e-7);
        Assert.Equal(aR2, new gp_Lin(aCyl2.Axis()).Distance(aP), 1.0e-7);
    }
}
