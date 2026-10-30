// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Geom2dGcc_Lin2d2Tan: OCCT's GTests (src/ModelingAlgorithms/TKGeomAlgo/GTests/Geom2dGcc_Lin2d2Tan_Test.cxx, from
/// QABugs_17's OCC813 and OCC814), the analytic dispatch against GccAna_Lin2d2Tan, and tangency on free-form curves.
/// </summary>
public class Geom2dGcc_Lin2d2TanTests
{
    private static readonly gp_Dir2d DX = new(gp_Dir2d.D.X);

    private static gp_Ax2 TestAx2() => new(new gp_Pnt(1262.224429, 425.040878, 363.609716),
                                           new gp_Dir(0.173648, 0.984808, 0.000000),
                                           new gp_Dir(-0.932169, 0.164367, -0.322560));

    // Test OCC813: Geom2dGcc_Lin2d2Tan - tangent line between a 2D ellipse and a point.
    [Fact]
    public void OCC813_EllipseAndPoint()
    {
        // Construct 3D ellipse and projection plane
        gp_Ax2       anAx2  = TestAx2();
        Geom_Ellipse anEll  = new(anAx2, 150, 100);
        Geom_Plane   aPlane = new(new gp_Ax3(anAx2));
        gp_Pln       aPln   = aPlane.Pln();

        // Project ellipse onto the plane and build 2D qualified curve
        Geom2d_Curve             aCurve2d = GeomAPI.To2d(anEll, aPln)!;
        Geom2dAdaptor_Curve      anAdapt  = new(aCurve2d);
        Geom2dGcc_QualifiedCurve aQCurve  = new(anAdapt, GccEnt_outside);

        // Query tangent line from 2D point to the projected ellipse
        gp_Pnt2d            aPnt2d  = new(200.0, 200.0);
        Geom2dGcc_Lin2d2Tan aLinTan = new(aQCurve, aPnt2d, 0.1);

        Assert.True(aLinTan.NbSolutions() > 0, "Expected at least one tangent line solution");
    }

    // Test OCC814: Geom2dGcc_Lin2d2Tan - tangent line between a 2D circle and a 2D ellipse.
    [Fact]
    public void OCC814_CircleAndEllipse()
    {
        gp_Ax2        anAx2 = TestAx2();
        Geom_Circle   aCir  = new(new gp_Ax2(new gp_Pnt(823.687192, 502.366825, 478.960440),
                                             new gp_Dir(0.173648, 0.984808, 0.000000),
                                             new gp_Dir(-0.932169, 0.164367, -0.322560)),
                                  50);
        Geom_Ellipse  anEll  = new(anAx2, 150, 100);
        Geom_Plane    aPlane = new(new gp_Ax3(anAx2));
        gp_Pln        aPln   = aPlane.Pln();

        // Project both curves onto the plane and build qualified curves
        Geom2d_Curve             aCurve2d     = GeomAPI.To2d(anEll, aPln)!;
        Geom2d_Curve             aFromCurve2d = GeomAPI.To2d(aCir, aPln)!;
        Geom2dAdaptor_Curve      anAdaptEll   = new(aCurve2d);
        Geom2dAdaptor_Curve      anAdaptCir   = new(aFromCurve2d);
        Geom2dGcc_QualifiedCurve aQEll        = new(anAdaptEll, GccEnt_outside);
        Geom2dGcc_QualifiedCurve aQCir        = new(anAdaptCir, GccEnt_outside);

        Geom2dGcc_Lin2d2Tan aLinTan = new(aQEll, aQCir, 0.1);

        Assert.True(aLinTan.NbSolutions() > 0, "Expected at least one tangent line solution");
    }

    [Fact]
    public void TwoCircles_GiveGccAnasSolutions()
    {
        gp_Circ2d c1 = new(new gp_Ax2d(new gp_Pnt2d(0, 0), DX), 1);
        gp_Circ2d c2 = new(new gp_Ax2d(new gp_Pnt2d(5, 1), DX), 2);
        Geom2dGcc_Lin2d2Tan aLin = new(Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(new Geom2d_Circle(c1))),
                                       Geom2dGcc.Unqualified(new Geom2dAdaptor_Curve(new Geom2d_Circle(c2))), 1e-9);
        GccAna_Lin2d2Tan anAna = new(GccEnt.Unqualified(c1), GccEnt.Unqualified(c2), 1e-9);
        Assert.True(aLin.IsDone());
        Assert.Equal(anAna.NbSolutions(), aLin.NbSolutions());
        Assert.Equal(4, aLin.NbSolutions());
        for (int i = 1; i <= aLin.NbSolutions(); i++)
        {
            Assert.Equal(anAna.ThisSolution(i), aLin.ThisSolution(i));
            aLin.Tangency1(i, out double p1, out double a1, out gp_Pnt2d t1);
            anAna.Tangency1(i, out double q1, out double b1, out gp_Pnt2d u1);
            Assert.Equal((q1, b1, u1), (p1, a1, t1));
            aLin.Tangency2(i, out double p2, out double a2, out gp_Pnt2d t2);
            anAna.Tangency2(i, out double q2, out double b2, out gp_Pnt2d u2);
            Assert.Equal((q2, b2, u2), (p2, a2, t2));
            aLin.WhichQualifier(i, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
            anAna.WhichQualifier(i, out GccEnt_Position aR1, out GccEnt_Position aR2);
            Assert.Equal((aR1, aR2), (aQ1, aQ2));
        }
    }

    // A line tangent to a curve at a parameter: the point is on the curve and the line, and parallel to the tangent.
    internal static void CheckTangent(in gp_Lin2d theLin, Geom2dAdaptor_Curve theC, double theParArg, in gp_Pnt2d thePnt,
                                      double theTol)
    {
        theC.D1(theParArg, out gp_Pnt2d aP, out gp_Vec2d aV);
        Assert.True(aP.Distance(thePnt) < theTol, $"{aP} != {thePnt}");
        Assert.True(theLin.Distance(thePnt) < theTol);
        Assert.True(Math.Abs(theLin.Direction().Crossed(new gp_Dir2d(aV))) < theTol);
    }

    [Fact]
    public void EllipseAndBSpline_GiveTangentLines()
    {
        Geom2dAdaptor_Curve anEll = new(new Geom2d_Ellipse(new gp_Ax2d(new gp_Pnt2d(0, 0), DX), 3, 1));
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, 4);
        aPoles[1] = new gp_Pnt2d(6, -2);
        aPoles[2] = new gp_Pnt2d(8, 3);
        aPoles[3] = new gp_Pnt2d(10, -3);
        aPoles[4] = new gp_Pnt2d(12, 2);
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        Geom2dAdaptor_Curve aBS = new(new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 3));

        Geom2dGcc_Lin2d2Tan aLin = new(Geom2dGcc.Unqualified(anEll), Geom2dGcc.Unqualified(aBS), 1e-6);
        Assert.True(aLin.IsDone());
        Assert.InRange(aLin.NbSolutions(), 1, 4);
        for (int i = 1; i <= aLin.NbSolutions(); i++)
        {
            gp_Lin2d aSol = aLin.ThisSolution(i);
            aLin.Tangency1(i, out _, out double a1, out gp_Pnt2d t1);
            aLin.Tangency2(i, out _, out double a2, out gp_Pnt2d t2);
            CheckTangent(aSol, anEll, a1, t1, 1e-6);
            CheckTangent(aSol, aBS, a2, t2, 1e-6);
        }

        // Through a point
        Geom2dGcc_Lin2d2Tan aThrough = new(Geom2dGcc.Unqualified(anEll), new gp_Pnt2d(0, 5), 1e-6);
        Assert.Equal(2, aThrough.NbSolutions());
        for (int i = 1; i <= 2; i++)
        {
            gp_Lin2d aSol = aThrough.ThisSolution(i);
            aThrough.Tangency1(i, out _, out double a1, out gp_Pnt2d t1);
            CheckTangent(aSol, anEll, a1, t1, 1e-6);
            Assert.True(aSol.Distance(new gp_Pnt2d(0, 5)) < 1e-9);
            aThrough.WhichQualifier(i, out GccEnt_Position aQ1, out GccEnt_Position aQ2);
            Assert.Equal((GccEnt_unqualified, GccEnt_noqualifier), (aQ1, aQ2));
        }
    }

    [Fact]
    public void Accessors_ThrowAsOCCT()
    {
        Geom2dAdaptor_Curve anEll = new(new Geom2d_Ellipse(new gp_Ax2d(new gp_Pnt2d(0, 0), DX), 3, 1));
        // A point inside the ellipse: no tangent line
        Geom2dGcc_Lin2d2Tan aNone = new(Geom2dGcc.Unqualified(anEll), new gp_Pnt2d(0.5, 0), 1e-6);
        Assert.False(aNone.IsDone());
        Assert.Equal(0, aNone.NbSolutions());
        Assert.Throws<StdFail_NotDone>(() => aNone.Tangency1(1, out _, out _, out _));
        Assert.Throws<Standard_OutOfRange>(() => aNone.ThisSolution(1));

        Geom2dGcc_Lin2d2Tan aLin = new(Geom2dGcc.Unqualified(anEll), new gp_Pnt2d(0, 5), 1e-6);
        Assert.Throws<Standard_OutOfRange>(() => aLin.ThisSolution(0));
        Assert.Throws<Standard_OutOfRange>(() => aLin.Tangency2(3, out _, out _, out _));
        Assert.Throws<GccEnt_BadQualifier>(
            () => new Geom2dGcc_Lin2d2Tan(new Geom2dGcc_QualifiedCurve(anEll, GccEnt_noqualifier), new gp_Pnt2d(0, 5), 1e-6));
    }
}
