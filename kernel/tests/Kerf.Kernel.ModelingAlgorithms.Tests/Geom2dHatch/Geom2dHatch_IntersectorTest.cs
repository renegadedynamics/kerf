// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/Geom2dHatch_Intersector_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification.Hatching;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's Geom2dHatch_Intersector GTest (the suite is named after the class, hence the suffix).</summary>
public class Geom2dHatch_IntersectorTest
{
    // Verify LocalGeometry produces valid outputs on a circle (well-defined tangent).
    [Fact]
    public void LocalGeometry_Circle_ValidOutputs()
    {
        Axis2D anAxis = new(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));
        CircleCurve2D aCircle = new(anAxis, 1.0);
        CurveAdaptor2D aCurveAdaptor = new(aCircle);

        Intersector2D anIntersector = new(1.0e-7, 1.0e-7);

        anIntersector.LocalGeometry(aCurveAdaptor, 0.0, out Direction2D aTang, out Direction2D _, out double aCurv);

        // Circle tangent at param=0 is (0,1), normal is (-1,0), curvature is 1.0.
        Assert.Equal(0.0, aTang.X, 1.0e-10);
        Assert.Equal(1.0, aTang.Y, 1.0e-10);
        Assert.Equal(1.0, aCurv, 1.0e-10);
    }

    // Verify LocalGeometry produces valid outputs on a straight line (zero curvature).
    [Fact]
    public void LocalGeometry_Line_ZeroCurvature()
    {
        Axis2D anAxis = new(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0));
        LineCurve2D aLine = new(anAxis);
        CurveAdaptor2D aCurveAdaptor = new(aLine);

        Intersector2D anIntersector = new(1.0e-7, 1.0e-7);

        anIntersector.LocalGeometry(aCurveAdaptor, 0.5, out Direction2D aTang, out Direction2D aNorm, out double aCurv);

        Assert.Equal(1.0, aTang.X, 1.0e-10);
        Assert.Equal(0.0, aTang.Y, 1.0e-10);
        Assert.Equal(0.0, aCurv, 1.0e-10);
        // Normal should be perpendicular to tangent.
        Assert.Equal(0.0, aNorm.X, 1.0e-10);
        Assert.Equal(1.0, Math.Abs(aNorm.Y), 1.0e-10);
    }

    // Regression test for bug #24: LocalGeometry with undefined tangent
    // must still produce initialized (deterministic) output values.
    [Fact]
    public void LocalGeometry_DegenerateCurve_InitializedOutputs()
    {
        // Construct a degenerate BSpline where all control points coincide,
        // causing IsTangentDefined() to return false.
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles[1] = new Point2D(1.0, 1.0);
        aPoles[2] = new Point2D(1.0, 1.0);
        aPoles[3] = new Point2D(1.0, 1.0);
        aPoles[4] = new Point2D(1.0, 1.0);

        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;

        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;

        BSplineCurve2D aDegenerateCurve = new(aPoles, aKnots, aMults, 3);
        CurveAdaptor2D aCurveAdaptor = new(aDegenerateCurve);

        Intersector2D anIntersector = new(1.0e-7, 1.0e-7);

        Direction2D aTang = new(), aNorm = new();
        double aCurv = -999.0;

        // Must not crash and must produce initialized outputs.
        Assert.Null(Record.Exception(() =>
            anIntersector.LocalGeometry(aCurveAdaptor, 0.5, out aTang, out aNorm, out aCurv)));

        // Curvature should be initialized to 0 for degenerate case.
        Assert.Equal(0.0, aCurv, 1.0e-10);
        // Tang and Norm should be valid unit directions (magnitude == 1).
        double aTangMag = Math.Sqrt(aTang.X * aTang.X + aTang.Y * aTang.Y);
        Assert.Equal(1.0, aTangMag, 1.0e-10);
        double aNormMag = Math.Sqrt(aNorm.X * aNorm.X + aNorm.Y * aNorm.Y);
        Assert.Equal(1.0, aNormMag, 1.0e-10);
    }
}
