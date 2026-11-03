// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Geom2dGcc_Lin2dTanObl: the analytic dispatch against GccAna_Lin2dTanObl, the iterative solver on an ellipse, OCCT's
/// parallel case, and the accessors' exceptions. OCCT has no GTest for it.
/// </summary>
public class Geom2dGcc_Lin2dTanOblTests
{
    private static readonly Direction2D DX = new(Direction2D.D.X);

    [Fact]
    public void Circle_GivesGccAnasSolutions()
    {
        Circle2D             c1    = new(new Axis2D(new Point2D(1, 2), DX), 3);
        Line2D              aLine = new(new Point2D(0, -4), new Direction2D(1, 0.5));
        global::Kerf.Engine.Construction.Constraints.Lin2dTanObl aLin  = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(new CurveAdaptor2D(new CircleCurve2D(c1))), aLine,
                                          1e-9, 0.7);
        AnalyticLin2dTanObl    anAna = new(global::Kerf.Engine.Construction.Constraints.GccEnt.Unqualified(c1), aLine, 0.7);
        Assert.True(aLin.IsDone);
        Assert.Equal(2, aLin.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(anAna.ThisSolution(i), aLin.ThisSolution(i));
            aLin.Tangency1(i, out double p1, out double a1, out Point2D t1);
            anAna.Tangency1(i, out double q1, out double b1, out Point2D u1);
            Assert.Equal((q1, b1, u1), (p1, a1, t1));
            aLin.Intersection2(i, out double p2, out double a2, out Point2D t2);
            anAna.Intersection2(i, out double q2, out double b2, out Point2D u2);
            Assert.Equal((q2, b2, u2), (p2, a2, t2));
            aLin.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aQ);
            anAna.WhichQualifier(i, out global::Kerf.Engine.Construction.Constraints.Qualifier aR);
            Assert.Equal(aR, aQ);
        }
    }

    [Fact]
    public void Ellipse_GivesTangentLinesAtTheAngle()
    {
        CurveAdaptor2D   anEll = new(new EllipseCurve2D(new Axis2D(new Point2D(0, 0), DX), 3, 1));
        Line2D              aLine = new(new Point2D(0, -4), DX);
        global::Kerf.Engine.Construction.Constraints.Lin2dTanObl aLin  = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(anEll), aLine, 1e-6, M_PI / 4);
        Assert.True(aLin.IsDone);
        Assert.Equal(2, aLin.SolutionCount);
        for (int i = 1; i <= 2; i++)
        {
            Line2D aSol = aLin.ThisSolution(i);
            Assert.Equal(M_PI / 4, Math.Abs(DX.Angle(aSol.Direction())), 1e-9);
            aLin.Tangency1(i, out _, out double a1, out Point2D t1);
            Geom2dGcc_Lin2d2TanTests.CheckTangent(aSol, anEll, a1, t1, 1e-6);
            aLin.Intersection2(i, out double p2, out double a2, out Point2D t2);
            Assert.True(aLine.Distance(t2) < 1e-9);
            Assert.True(aSol.Distance(t2) < 1e-9);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(a2, aLine).Distance(t2) < 1e-9);
            Assert.True(global::Kerf.Engine.Geometry.ElCLib.Value(p2, aSol).Distance(t2) < 1e-9);
        }
    }

    [Fact]
    public void Ellipse_ParallelToTheLine_ThrowsIsParallel()
    {
        // OCCT: with an angle of 0 (or pi) the iterative solver's Intersection2 throws, and Add calls it on the first
        // solution found, so the constructor throws
        CurveAdaptor2D anEll = new(new EllipseCurve2D(new Axis2D(new Point2D(0, 0), DX), 3, 1));
        Line2D            aLine = new(new Point2D(0, -4), DX);
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.IsParallelException>(() => new global::Kerf.Engine.Construction.Constraints.Lin2dTanObl(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(anEll), aLine, 1e-6, 0.0));
    }

    [Fact]
    public void Accessors_ThrowAsOCCT()
    {
        CurveAdaptor2D   anEll = new(new EllipseCurve2D(new Axis2D(new Point2D(0, 0), DX), 3, 1));
        Line2D              aLine = new(new Point2D(0, -4), DX);
        global::Kerf.Engine.Construction.Constraints.Lin2dTanObl aLin  = new(global::Kerf.Engine.Construction.Constraints.Geom2dGcc.Unqualified(anEll), aLine, 1e-6, M_PI / 4);
        Assert.Throws<OutOfRangeException>(() => aLin.ThisSolution(3));
        Assert.Throws<OutOfRangeException>(() => aLin.Tangency1(0, out _, out _, out _));
        Assert.Throws<OutOfRangeException>(() => aLin.Intersection2(3, out _, out _, out _));
        Assert.Throws<OutOfRangeException>(() => aLin.WhichQualifier(3, out _));
        Assert.Throws<global::Kerf.Engine.Construction.Constraints.BadQualifierException>(
            () => new global::Kerf.Engine.Construction.Constraints.Lin2dTanObl(new global::Kerf.Engine.Construction.Constraints.QualifiedCurve(anEll, global::Kerf.Engine.Construction.Constraints.Qualifier.Noqualifier), aLine, 1e-6, M_PI / 4));
    }
}
