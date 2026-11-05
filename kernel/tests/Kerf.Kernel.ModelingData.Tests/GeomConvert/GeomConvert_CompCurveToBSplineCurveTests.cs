// Port of: src/ModelingData/TKGeomBase/GTests/GeomConvert_CompCurveToBSplineCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class GeomConvert_CompCurveToBSplineCurveTest
{
    private static BSplineCurve3D Bezier3(Point3D theP1, Point3D theP2, Point3D theP3, Point3D theP4)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = theP1;
        aPoles[2] = theP2;
        aPoles[3] = theP3;
        aPoles[4] = theP4;
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        return new BSplineCurve3D(aPoles, aKnots, aMults, 3);
    }

    private static void ExpectNear(in Point3D theA, in Point3D theB)
        => Assert.Equal(0.0, theA.DistanceTo(theB), global::Kerf.Engine.Foundation.Precision.Confusion());

    [Fact]
    public void ConcatenateClampedBSplines()
    {
        // Two clamped B-spline curves that share an endpoint.
        BSplineCurve3D aCurve1 = Bezier3(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 1, 0), new Point3D(3, 0, 0));
        BSplineCurve3D aCurve2 = Bezier3(new Point3D(3, 0, 0), new Point3D(4, -1, 0), new Point3D(5, -1, 0), new Point3D(6, 0, 0));

        CompCurveToBSplineCurve aConcat = new(aCurve1);
        bool isAdded = aConcat.Add(aCurve2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(isAdded, "Should successfully concatenate clamped B-splines");

        BSplineCurve3D? aResult = aConcat.BSplineCurve();
        Assert.NotNull(aResult);
        ExpectNear(aResult.StartPoint, new Point3D(0, 0, 0));
        ExpectNear(aResult.EndPoint, new Point3D(6, 0, 0));
    }

    [Fact]
    public void ConcatenateTrimmedCircleArcs()
    {
        // Trimmed circle arcs convert to non-clamped B-splines whose poles aren't their end points (bug 0030007).
        Circle3D aCirc = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5.0);
        CircleCurve3D aGeomCircle = new(aCirc);
        TrimmedCurve3D aArc1 = new(aGeomCircle, 0.0, Math.PI / 2.0);
        TrimmedCurve3D aArc2 = new(aGeomCircle, Math.PI / 2.0, Math.PI);
        ExpectNear(aArc1.EndPoint, aArc2.StartPoint);

        CompCurveToBSplineCurve aConcat = new(aArc1);
        bool isAdded = aConcat.Add(aArc2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(isAdded, "Should successfully concatenate trimmed circle arcs");

        BSplineCurve3D? aResult = aConcat.BSplineCurve();
        Assert.NotNull(aResult);
        ExpectNear(aResult.StartPoint, new Point3D(5, 0, 0));
        ExpectNear(aResult.EndPoint, new Point3D(-5, 0, 0));
    }

    [Fact]
    public void ConcatenateWithReversal()
    {
        BSplineCurve3D aCurve1 = Bezier3(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 1, 0), new Point3D(3, 0, 0));
        // Ends at aCurve1's end: must be reversed
        BSplineCurve3D aCurve2 = Bezier3(new Point3D(6, 0, 0), new Point3D(5, -1, 0), new Point3D(4, -1, 0), new Point3D(3, 0, 0));

        CompCurveToBSplineCurve aConcat = new(aCurve1);
        bool isAdded = aConcat.Add(aCurve2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(isAdded, "Should successfully concatenate curves with reversal");

        BSplineCurve3D? aResult = aConcat.BSplineCurve();
        Assert.NotNull(aResult);
        ExpectNear(aResult.StartPoint, new Point3D(0, 0, 0));
        ExpectNear(aResult.EndPoint, new Point3D(6, 0, 0));
    }

    [Fact]
    public void FailsForDisjointCurves()
    {
        BSplineCurve3D aCurve1 = Bezier3(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 1, 0), new Point3D(3, 0, 0));
        BSplineCurve3D aCurve2 = Bezier3(new Point3D(10, 0, 0), new Point3D(11, 1, 0), new Point3D(12, 1, 0), new Point3D(13, 0, 0));

        CompCurveToBSplineCurve aConcat = new(aCurve1);
        bool isAdded = aConcat.Add(aCurve2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.False(isAdded, "Should fail to concatenate disjoint curves");
    }

    [Fact]
    public void ConcatenateNonClampedBSpline_Bug30007()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 6);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 2, 0);
        aPoles[3] = new Point3D(2, 2, 0);
        aPoles[4] = new Point3D(3, 2, 0);
        aPoles[5] = new Point3D(4, 2, 0);
        aPoles[6] = new Point3D(5, 0, 0);
        NCollection_Array1<double> aKnots = new(1, 4);
        aKnots[1] = 0.0;
        aKnots[2] = 0.33;
        aKnots[3] = 0.67;
        aKnots[4] = 1.0;
        NCollection_Array1<int> aMults = new(1, 4);
        aMults[1] = 4;
        aMults[2] = 1;
        aMults[3] = 1;
        aMults[4] = 4;
        BSplineCurve3D aCurve1 = new(aPoles, aKnots, aMults, 3);

        Point3D aCurve1End = aCurve1.EndPoint;
        BSplineCurve3D aCurve2 = Bezier3(aCurve1End, new Point3D(6, -1, 0), new Point3D(7, -1, 0), new Point3D(8, 0, 0));

        CompCurveToBSplineCurve aConcat = new(aCurve1);
        bool isAdded = aConcat.Add(aCurve2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(isAdded, "Should concatenate using actual endpoints, not poles");

        BSplineCurve3D? aResult = aConcat.BSplineCurve();
        Assert.NotNull(aResult);
        ExpectNear(aResult.StartPoint, aCurve1.StartPoint);
        ExpectNear(aResult.EndPoint, new Point3D(8, 0, 0));
    }

    [Fact]
    public void PrependCurve()
    {
        BSplineCurve3D aCurve1 = Bezier3(new Point3D(3, 0, 0), new Point3D(4, 1, 0), new Point3D(5, 1, 0), new Point3D(6, 0, 0));
        // Ends at aCurve1's start: prepended
        BSplineCurve3D aCurve2 = Bezier3(new Point3D(0, 0, 0), new Point3D(1, -1, 0), new Point3D(2, -1, 0), new Point3D(3, 0, 0));

        CompCurveToBSplineCurve aConcat = new(aCurve1);
        bool isAdded = aConcat.Add(aCurve2, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(isAdded, "Should successfully prepend curve");

        BSplineCurve3D? aResult = aConcat.BSplineCurve();
        Assert.NotNull(aResult);
        ExpectNear(aResult.StartPoint, new Point3D(0, 0, 0));
        ExpectNear(aResult.EndPoint, new Point3D(6, 0, 0));
    }

    [Fact]
    public void EmptyInitialCurve()
    {
        CompCurveToBSplineCurve aConcat = new();
        BSplineCurve3D aCurve = Bezier3(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 1, 0), new Point3D(3, 0, 0));

        bool isAdded = aConcat.Add(aCurve, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(isAdded, "Should successfully add to empty converter");
        Assert.NotNull(aConcat.BSplineCurve());
    }

    [Fact]
    public void ClearAndReuse()
    {
        BSplineCurve3D aCurve = Bezier3(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 1, 0), new Point3D(3, 0, 0));
        CompCurveToBSplineCurve aConcat = new(aCurve);
        Assert.NotNull(aConcat.BSplineCurve());

        aConcat.Clear();
        Assert.Null(aConcat.BSplineCurve());

        bool isAdded = aConcat.Add(aCurve, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(isAdded, "Should successfully add after Clear()");
        Assert.NotNull(aConcat.BSplineCurve());
    }
}
