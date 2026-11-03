// Port of: src/ModelingData/TKGeomBase/GTests/IntAna_IntQuadQuad_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.ResultType;

namespace Kerf.Kernel.ModelingData.Tests;

// The tests of IntAna_IntQuadQuad_Test that exercise IntAna_QuadQuadGeo.
public partial class IntAna_IntQuadQuad_Test
{
    // Regression test for near-tangent parallel cylinders.
    // Very small chord between two theoretical lines must collapse to one line.
    [Fact]
    public void CylinderCylinderNearTangent_CollapsesToSingleLine()
    {
        Point3D aCenter1 = new(0.0, 0.0, 0.0);
        Point3D aCenter2 = new(2.0 - 1.0e-9, 0.0, 0.0);
        Direction3D aZDir = new(Direction3D.D.Z);
        Direction3D anXDir = new(Direction3D.D.X);
        CoordinateSystem anAxis1 = new(aCenter1, aZDir, anXDir);
        CoordinateSystem anAxis2 = new(aCenter2, aZDir, anXDir);

        Cylinder aCyl1 = new(anAxis1, 1.0);
        Cylinder aCyl2 = new(anAxis2, 1.0);

        QuadQuadGeo anInter = new(aCyl1, aCyl2, 1.0e-4);

        Assert.True(anInter.IsDone);
        Assert.Equal(Line, anInter.TypeInter());
        Assert.Equal(1, anInter.SolutionCount);
    }

    // Clear non-tangent case must keep two-line solution.
    [Fact]
    public void CylinderCylinderNonTangent_HasTwoLines()
    {
        Point3D aCenter1 = new(0.0, 0.0, 0.0);
        Point3D aCenter2 = new(1.8, 0.0, 0.0);
        Direction3D aZDir = new(Direction3D.D.Z);
        Direction3D anXDir = new(Direction3D.D.X);
        CoordinateSystem anAxis1 = new(aCenter1, aZDir, anXDir);
        CoordinateSystem anAxis2 = new(aCenter2, aZDir, anXDir);

        Cylinder aCyl1 = new(anAxis1, 1.0);
        Cylinder aCyl2 = new(anAxis2, 1.0);

        QuadQuadGeo anInter = new(aCyl1, aCyl2, 1.0e-7);

        Assert.True(anInter.IsDone);
        Assert.Equal(Line, anInter.TypeInter());
        Assert.Equal(2, anInter.SolutionCount);
    }

    // Sanity regression for the pre-existing skew external-tangent branch.
    [Fact]
    public void CylinderCylinderSkewExternallyTangent_HasPoint()
    {
        double aR1 = 3.0;
        double aR2 = 2.0;
        double aDist = aR1 + aR2;

        CoordinateSystem anAxis1 = new(new Point3D(0.0, 0.0, 0.0), Direction3D.D.Z, Direction3D.D.X);
        CoordinateSystem anAxis2 = new(new Point3D(0.0, aDist, 0.0), Direction3D.D.X, Direction3D.D.Y);

        Cylinder aCyl1 = new(anAxis1, aR1);
        Cylinder aCyl2 = new(anAxis2, aR2);

        QuadQuadGeo anInter = new(aCyl1, aCyl2, 1.0e-7);

        Assert.True(anInter.IsDone);
        Assert.Equal(Point, anInter.TypeInter());
        Assert.Equal(1, anInter.SolutionCount);

        Point3D aP = anInter.Point(1);
        Assert.Equal(aR1, new Line3D(aCyl1.Axis()).Distance(aP), 1.0e-7);
        Assert.Equal(aR2, new Line3D(aCyl2.Axis()).Distance(aP), 1.0e-7);
    }
}
