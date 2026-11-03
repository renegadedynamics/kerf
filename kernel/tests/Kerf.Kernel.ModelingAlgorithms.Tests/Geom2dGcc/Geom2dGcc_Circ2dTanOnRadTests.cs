// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Geom2dGcc_Circ2dTanOnRad: the analytic dispatch against GccAna_Circ2dTanOnRad, the geometric solver on an ellipse
/// and a B-spline, and the accessors' exceptions. OCCT has no GTest for it.
/// </summary>
public class Geom2dGcc_Circ2dTanOnRadTests
{
    private static readonly Direction2D DX = new(Direction2D.D.X);

    [Fact]
    public void CircleOnLine_GivesGccAnasSolutions()
    {
        Circle2D                c1  = new(new Axis2D(new Point2D(1, 2), DX), 3);
        Line2D                 lon = new(new Point2D(-5, 0), new Direction2D(1, 0.2));
        global::Kerf.Engine.Construction.Constraints.Circ2dTanOnRad aCirc = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(new CurveAdaptor2D(new CircleCurve2D(c1))),
                                             new CurveAdaptor2D(new LineCurve2D(lon)), 1.5, 1e-9);
        AnalyticCirc2dTanOnRad    anAna = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(c1), lon, 1.5, 1e-9);
        Assert.True(aCirc.IsDone);
        Assert.Equal(anAna.SolutionCount, aCirc.SolutionCount);
        Assert.True(aCirc.SolutionCount > 0);
        for (int i = 1; i <= aCirc.SolutionCount; i++)
        {
            Assert.Equal(anAna.ThisSolution(i), aCirc.ThisSolution(i));
            aCirc.Tangency1(i, out double p1, out double a1, out Point2D t1);
            anAna.Tangency1(i, out double q1, out double b1, out Point2D u1);
            Assert.Equal((q1, b1, u1), (p1, a1, t1));
            aCirc.CenterOn3(i, out double c3, out Point2D pc3);
            anAna.CenterOn3(i, out double d3, out Point2D qc3);
            Assert.Equal((d3, qc3), (c3, pc3));
            aCirc.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ);
            anAna.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aR);
            Assert.Equal(aR, aQ);
        }
    }

    [Fact]
    public void CircleOnEllipse_GivesTangentCirclesCentredOnTheEllipse()
    {
        // A circle and an ellipse to centre on: Geo's qualified-circle constructor (through
        // Geom2dInt_TheIntConicCurveOfGInter)
        Circle2D                c1    = new(new Axis2D(new Point2D(1, 0.5), DX), 1);
        CurveAdaptor2D      anEll = new(new EllipseCurve2D(new Axis2D(new Point2D(0, 0), DX), 3, 1));
        global::Kerf.Engine.Construction.Constraints.Circ2dTanOnRad aCirc = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(new CurveAdaptor2D(new CircleCurve2D(c1))), anEll,
                                             0.4, 1e-9);
        Assert.True(aCirc.IsDone);
        Assert.True(aCirc.SolutionCount > 0);
        for (int i = 1; i <= aCirc.SolutionCount; i++)
        {
            Circle2D aSol = aCirc.ThisSolution(i);
            Assert.Equal(0.4, aSol.Radius);
            aCirc.CenterOn3(i, out double aParCen, out _);
            Assert.True(anEll.Value(aParCen).DistanceTo(aSol.Location) < 1e-7);
            aCirc.Tangency1(i, out _, out double aParArg, out Point2D aPnt);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(aParArg, c1).DistanceTo(aPnt) < 1e-9);
            double aD = aSol.Location.DistanceTo(c1.Location);
            Assert.True(Math.Abs(aD - 1.4) < 1e-7 || Math.Abs(aD - 0.6) < 1e-7);
            aCirc.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ);
            Assert.Equal(Math.Abs(aD - 1.4) < 1e-7 ? global::Kerf.Engine.Construction.Constraints.Qualifier.Outside : global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed, aQ);
        }
    }

    [Fact]
    public void EllipseOnLine_FindsNothing()
    {
        // OCCT: for a free-form argument, Geo's constructor wraps the curve to centre on in an Adaptor2d_OffsetCurve
        // made without an offset, whose parameter range is [0, 0], so it intersects the argument's offsets with a
        // single point of that curve: done, but no solution here.
        CurveAdaptor2D      anEll = new(new EllipseCurve2D(new Axis2D(new Point2D(0, 0), DX), 3, 1));
        Line2D                 lon   = new(new Point2D(0, 0), new Direction2D(1, 0.3));
        global::Kerf.Engine.Construction.Constraints.Circ2dTanOnRad aCirc = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(anEll), new CurveAdaptor2D(new LineCurve2D(lon)),
                                             0.2, 1e-9);
        Assert.True(aCirc.IsDone);
        Assert.Equal(0, aCirc.SolutionCount);
    }

    [Fact]
    public void PointOnBSpline_GivesCirclesThroughThePoint()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 4);
        aPoles[1] = new Point2D(-3, -1);
        aPoles[2] = new Point2D(-1, 2);
        aPoles[3] = new Point2D(1, -2);
        aPoles[4] = new Point2D(3, 1);
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        CurveAdaptor2D aBS = new(new BSplineCurve2D(aPoles, aKnots, aMults, 3));
        Point2D            aP  = new(0, 0.5);

        global::Kerf.Engine.Construction.Constraints.Circ2dTanOnRad aCirc = new(new CartesianPoint2(aP), aBS, 1.0, 1e-9);
        Assert.True(aCirc.IsDone);
        Assert.True(aCirc.SolutionCount > 0);
        for (int i = 1; i <= aCirc.SolutionCount; i++)
        {
            Circle2D aSol = aCirc.ThisSolution(i);
            Assert.Equal(1.0, aSol.Location.DistanceTo(aP), 1e-7);
            aCirc.Tangency1(i, out _, out _, out Point2D aPnt);
            Assert.Equal(aP, aPnt);
            aCirc.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ);
            Assert.Equal(global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier, aQ);
        }
    }

    [Fact]
    public void Accessors_ThrowAsOCCT()
    {
        CurveAdaptor2D anEll = new(new EllipseCurve2D(new Axis2D(new Point2D(0, 0), DX), 3, 1));
        CurveAdaptor2D aLine = new(new LineCurve2D(new Point2D(0, -2), new Direction2D(1, 0.3)));
        Assert.Throws<NegativeValueException>(() => new global::Kerf.Engine.Construction.Constraints.Circ2dTanOnRad(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(anEll), aLine, -1, 1e-9));
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(
            () => new global::Kerf.Engine.Construction.Constraints.Circ2dTanOnRad(new global::Kerf.Engine.Construction.Constraints.QualifiedCurve(anEll, global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier), aLine, 1, 1e-9));

        global::Kerf.Engine.Construction.Constraints.Circ2dTanOnRad aCirc = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(anEll), aLine, 0.5, 1e-9);
        Assert.Throws<OutOfRangeException>(() => aCirc.ThisSolution(0));
        Assert.Throws<OutOfRangeException>(() => aCirc.CenterOn3(aCirc.SolutionCount + 1, out _, out _));

        // Enclosing a free-form curve: no side to offset to, so no solution and not done
        global::Kerf.Engine.Construction.Constraints.Circ2dTanOnRad aNone = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Enclosing(anEll), aLine, 0.5, 1e-9);
        Assert.False(aNone.IsDone);
        Assert.Throws<NotDoneException>(() => aNone.ThisSolution(1));
    }
}
