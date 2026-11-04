// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Geom2dGcc_Lin2d2Tan: OCCT's GTests (src/ModelingAlgorithms/TKGeomAlgo/GTests/Geom2dGcc_Lin2d2Tan_Test.cxx, from
/// QABugs_17's OCC813 and OCC814), the analytic dispatch against GccAna_Lin2d2Tan, and tangency on free-form curves.
/// </summary>
public class Geom2dGcc_Lin2d2TanTests
{
    private static readonly Direction2D DX = new(Direction2D.D.X);

    private static Frame3D TestAx2() => new(new Point3D(1262.224429, 425.040878, 363.609716),
                                           new Direction3D(0.173648, 0.984808, 0.000000),
                                           new Direction3D(-0.932169, 0.164367, -0.322560));

    // Test OCC813: Geom2dGcc_Lin2d2Tan - tangent line between a 2D ellipse and a point.
    [Fact]
    public void OCC813_EllipseAndPoint()
    {
        // Construct 3D ellipse and projection plane
        Frame3D       anAx2  = TestAx2();
        EllipseCurve3D anEll  = new(anAx2, 150, 100);
        PlaneSurface   aPlane = new(new CoordinateSystem(anAx2));
        Plane3D       aPln   = aPlane.Plane;

        // Project ellipse onto the plane and build 2D qualified curve
        Curve2D             aCurve2d = Projections.To2d(anEll, aPln)!;
        CurveAdaptor2D      anAdapt  = new(aCurve2d);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQCurve  = new(anAdapt, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside);

        // Query tangent line from 2D point to the projected ellipse
        Point2D            aPnt2d  = new(200.0, 200.0);
        global::Kerf.Engine.Construction.Constraints.Lin2d2Tan aLinTan = new(aQCurve, aPnt2d, 0.1);

        Assert.True(aLinTan.SolutionCount > 0, "Expected at least one tangent line solution");
    }

    // Test OCC814: Geom2dGcc_Lin2d2Tan - tangent line between a 2D circle and a 2D ellipse.
    [Fact]
    public void OCC814_CircleAndEllipse()
    {
        Frame3D        anAx2 = TestAx2();
        CircleCurve3D   aCir  = new(new Frame3D(new Point3D(823.687192, 502.366825, 478.960440),
                                             new Direction3D(0.173648, 0.984808, 0.000000),
                                             new Direction3D(-0.932169, 0.164367, -0.322560)),
                                  50);
        EllipseCurve3D  anEll  = new(anAx2, 150, 100);
        PlaneSurface    aPlane = new(new CoordinateSystem(anAx2));
        Plane3D        aPln   = aPlane.Plane;

        // Project both curves onto the plane and build qualified curves
        Curve2D             aCurve2d     = Projections.To2d(anEll, aPln)!;
        Curve2D             aFromCurve2d = Projections.To2d(aCir, aPln)!;
        CurveAdaptor2D      anAdaptEll   = new(aCurve2d);
        CurveAdaptor2D      anAdaptCir   = new(aFromCurve2d);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQEll        = new(anAdaptEll, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside);
        global::Kerf.Engine.Construction.Constraints.QualifiedCurve aQCir        = new(anAdaptCir, global::Kerf.Engine.Construction.Constraints.Qualifier.Outside);

        global::Kerf.Engine.Construction.Constraints.Lin2d2Tan aLinTan = new(aQEll, aQCir, 0.1);

        Assert.True(aLinTan.SolutionCount > 0, "Expected at least one tangent line solution");
    }

    [Fact]
    public void TwoCircles_GiveGccAnasSolutions()
    {
        Circle2D c1 = new(new Axis2D(new Point2D(0, 0), DX), 1);
        Circle2D c2 = new(new Axis2D(new Point2D(5, 1), DX), 2);
        global::Kerf.Engine.Construction.Constraints.Lin2d2Tan aLin = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(new CurveAdaptor2D(new CircleCurve2D(c1))),
                                       global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(new CurveAdaptor2D(new CircleCurve2D(c2))), 1e-9);
        AnalyticLin2d2Tan anAna = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(c1), global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(c2), 1e-9);
        Assert.True(aLin.IsDone);
        Assert.Equal(anAna.SolutionCount, aLin.SolutionCount);
        Assert.Equal(4, aLin.SolutionCount);
        for (int i = 1; i <= aLin.SolutionCount; i++)
        {
            Assert.Equal(anAna.ThisSolution(i), aLin.ThisSolution(i));
            aLin.Tangency1(i, out double p1, out double a1, out Point2D t1);
            anAna.Tangency1(i, out double q1, out double b1, out Point2D u1);
            Assert.Equal((q1, b1, u1), (p1, a1, t1));
            aLin.Tangency2(i, out double p2, out double a2, out Point2D t2);
            anAna.Tangency2(i, out double q2, out double b2, out Point2D u2);
            Assert.Equal((q2, b2, u2), (p2, a2, t2));
            aLin.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
            anAna.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aR1, out global::Kerf.Engine.Construction.Constraints.Qualifier aR2);
            Assert.Equal((aR1, aR2), (aQ1, aQ2));
        }
    }

    // A line tangent to a curve at a parameter: the point is on the curve and the line, and parallel to the tangent.
    internal static void CheckTangent(in Line2D theLin, CurveAdaptor2D theC, double theParArg, in Point2D thePnt,
                                      double theTol)
    {
        theC.D1(theParArg, out Point2D aP, out Vector2D aV);
        Assert.True(aP.DistanceTo(thePnt) < theTol, $"{aP} != {thePnt}");
        Assert.True(theLin.DistanceTo(thePnt) < theTol);
        Assert.True(Math.Abs(theLin.Direction.Crossed(new Direction2D(aV))) < theTol);
    }

    [Fact]
    public void EllipseAndBSpline_GiveTangentLines()
    {
        CurveAdaptor2D anEll = new(new EllipseCurve2D(new Axis2D(new Point2D(0, 0), DX), 3, 1));
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles[1] = new Point2D(6, -2);
        aPoles[2] = new Point2D(8, 3);
        aPoles[3] = new Point2D(10, -3);
        aPoles[4] = new Point2D(12, 2);
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        CurveAdaptor2D aBS = new(new BSplineCurve2D(aPoles, aKnots, aMults, 3));

        global::Kerf.Engine.Construction.Constraints.Lin2d2Tan aLin = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(anEll), global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(aBS), 1e-6);
        Assert.True(aLin.IsDone);
        Assert.InRange(aLin.SolutionCount, 1, 4);
        for (int i = 1; i <= aLin.SolutionCount; i++)
        {
            Line2D aSol = aLin.ThisSolution(i);
            aLin.Tangency1(i, out _, out double a1, out Point2D t1);
            aLin.Tangency2(i, out _, out double a2, out Point2D t2);
            CheckTangent(aSol, anEll, a1, t1, 1e-6);
            CheckTangent(aSol, aBS, a2, t2, 1e-6);
        }

        // Through a point
        global::Kerf.Engine.Construction.Constraints.Lin2d2Tan aThrough = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(anEll), new Point2D(0, 5), 1e-6);
        Assert.Equal(2, aThrough.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Line2D aSol = aThrough.ThisSolution(i);
            aThrough.Tangency1(i, out _, out double a1, out Point2D t1);
            CheckTangent(aSol, anEll, a1, t1, 1e-6);
            Assert.True(aSol.DistanceTo(new Point2D(0, 5)) < 1e-9);
            aThrough.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ1, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ2);
            Assert.Equal((global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified, global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier), (aQ1, aQ2));
        }
    }

    [Fact]
    public void Accessors_ThrowAsOCCT()
    {
        CurveAdaptor2D anEll = new(new EllipseCurve2D(new Axis2D(new Point2D(0, 0), DX), 3, 1));
        // A point inside the ellipse: no tangent line
        global::Kerf.Engine.Construction.Constraints.Lin2d2Tan aNone = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(anEll), new Point2D(0.5, 0), 1e-6);
        Assert.False(aNone.IsDone);
        Assert.Equal(0, aNone.SolutionCount);
        EngineAssert.Throws<NotDoneException>(() => aNone.Tangency1(1, out _, out _, out _));
        EngineAssert.Throws<OutOfRangeException>(() => aNone.ThisSolution(1));

        global::Kerf.Engine.Construction.Constraints.Lin2d2Tan aLin = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(anEll), new Point2D(0, 5), 1e-6);
        EngineAssert.Throws<OutOfRangeException>(() => aLin.ThisSolution(0));
        EngineAssert.Throws<OutOfRangeException>(() => aLin.Tangency2(3, out _, out _, out _));
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(
            () => new global::Kerf.Engine.Construction.Constraints.Lin2d2Tan(new global::Kerf.Engine.Construction.Constraints.QualifiedCurve(anEll, global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier), new Point2D(0, 5), 1e-6));
    }
}
