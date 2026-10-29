#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// GeomLib_CheckCurveOnSurface against native OCCT through BRepTools::EvalAndUpdateTol, its one bound caller. The
/// port's side mirrors EvalAndUpdateTol: the range clamped to the non-periodic curves', a GeomAdaptor_Curve and a
/// Geom2dAdaptor_Curve on it, an Adaptor3d_CurveOnSurface on a GeomAdaptor_Surface, then the sampling fallback when
/// the check fails, and the edge's tolerance as the floor. Inputs are built from the same raw numbers on both sides.
/// </summary>
public class GeomLibCheckCurveOnSurfaceOracleTests(ITestOutputHelper output)
{
    // The port's BRepTools::EvalAndUpdateTol, for an edge of tolerance theTol.
    private static double EvalAndUpdateTol(double theTol, Geom_Curve C3d, Geom2d_Curve C2d, Geom_Surface S, double f,
                                           double l, out int theStatus)
    {
        double newtol = 0.0;
        double first = f, last = l;
        if (!C3d.IsPeriodic())
        {
            first = Math.Max(first, C3d.FirstParameter());
            last = Math.Min(last, C3d.LastParameter());
        }
        if (!C2d.IsPeriodic())
        {
            first = Math.Max(first, C2d.FirstParameter());
            last = Math.Min(last, C2d.LastParameter());
        }
        GeomAdaptor_Curve aGeomAdaptorCurve = new(C3d, first, last);
        Geom2dAdaptor_Curve aGeom2dAdaptorCurve = new(C2d, first, last);
        GeomAdaptor_Surface aGeomAdaptorSurface = new(S);
        Adaptor3d_CurveOnSurface anAdaptor3dCurveOnSurface = new(aGeom2dAdaptorCurve, aGeomAdaptorSurface);

        GeomLib_CheckCurveOnSurface CT = new(aGeomAdaptorCurve);
        CT.Perform(anAdaptor3dCurveOnSurface);
        theStatus = CT.ErrorStatus();
        if (CT.IsDone())
        {
            newtol = CT.MaxDistance();
        }
        else if (CT.ErrorStatus() == 3 || (CT.ErrorStatus() == 2 && (C3d.IsPeriodic() || C2d.IsPeriodic())))
        {
            // Try to estimate by sample points
            int nbint = 22;
            double dt = (last - first) / nbint;
            dt = Math.Max(dt, Precision.Confusion());
            double d, dmax = 0.0;
            int cnt = 0;
            double t = first;
            for (; t <= last; t += dt)
            {
                cnt++;
                gp_Pnt2d aP2d = C2d.Value(t);
                gp_Pnt aPC = C3d.Value(t);
                gp_Pnt aPS = S.Value(aP2d.X(), aP2d.Y());
                d = aPS.SquareDistance(aPC);
                if (d > dmax)
                {
                    dmax = d;
                }
            }
            if (cnt < nbint + 1)
            {
                t = last;
                gp_Pnt2d aP2d = C2d.Value(t);
                gp_Pnt aPC = C3d.Value(t);
                gp_Pnt aPS = S.Value(aP2d.X(), aP2d.Y());
                d = aPS.SquareDistance(aPC);
                if (d > dmax)
                {
                    dmax = d;
                }
            }

            newtol = 1.2 * Math.Sqrt(dmax);
        }
        return newtol > theTol ? newtol : theTol;
    }

    [Fact]
    public void CheckCurveOnSurface_through_BRepTools_EvalAndUpdateTol()
    {
        var anOracle = new Oracle(output, 20261501);
        int aCases = 0, aAboveTol = 0, aFailed = 0;
        int[] aKinds = new int[4];
        for (int c = 0; c < 600; c++)
        {
            int aKind = c % 4;
            N.Geom_Surface aNS;
            Geom_Surface aPS;
            N.Geom_Curve aNC;
            Geom_Curve aPC;
            N.Geom2d_Curve aNC2;
            Geom2d_Curve aPC2;
            double aF, aL;
            (N.gp_Ax3 aNAx3, gp_Ax3 aAx3) = GeomOracleInputs.Ax3s(anOracle);
            using (aNAx3)
            {
                if (aKind == 1)
                {
                    double aR = anOracle.Real(0.5, 5.0);
                    aNS = new N.Geom_CylindricalSurface(aNAx3, aR);
                    aPS = new Geom_CylindricalSurface(aAx3, aR);
                }
                else
                {
                    aNS = new N.Geom_Plane(aNAx3);
                    aPS = new Geom_Plane(aAx3);
                }
            }

            if (aKind <= 1)
            {
                // A 2D B-spline, and a 3D B-spline with the same knots whose poles are the surface's images of the 2D
                // poles, moved by up to 0.5 (none a quarter of the time): the general PSO + Newton path
                BSplineSpec aSpec = Geom2dOracle.RandomBSpline(anOracle, false);
                if (aKind == 1)
                {
                    for (int i = 0; i < aSpec.Poles.Length; i++)
                    {
                        aSpec.Poles[i] = new gp_Pnt2d(aSpec.Poles[i].X() * 0.3, aSpec.Poles[i].Y());
                    }
                }
                double aNoise = anOracle.Int(0, 3) == 0 ? 0.0 : anOracle.Real(0.0, 0.5);
                NCollection_Array1<gp_Pnt> aPoles3d = new(1, aSpec.Poles.Length);
                for (int i = 0; i < aSpec.Poles.Length; i++)
                {
                    gp_Pnt aP = aPS.Value(aSpec.Poles[i].X(), aSpec.Poles[i].Y());
                    aPoles3d[i + 1] = new gp_Pnt(aP.X() + aNoise * anOracle.Real(-1, 1),
                                                 aP.Y() + aNoise * anOracle.Real(-1, 1),
                                                 aP.Z() + aNoise * anOracle.Real(-1, 1));
                }
                NCollection_Array1<double>? aWeights =
                    aSpec.Weights is null ? null : NCollection_Array1<double>.From(aSpec.Weights);
                NCollection_Array1<double> aKnots = NCollection_Array1<double>.From(aSpec.Knots);
                NCollection_Array1<int> aMults = NCollection_Array1<int>.From(aSpec.Mults);
                aNC = Oracle.ToNativeCurve(aPoles3d, aWeights, aKnots, aMults, aSpec.Degree, false);
                aPC = aWeights is null
                          ? new Geom_BSplineCurve(aPoles3d, aKnots, aMults, aSpec.Degree)
                          : new Geom_BSplineCurve(aPoles3d, aWeights!, aKnots, aMults, aSpec.Degree);
                aNC2 = Geom2dOracle.ToNative(aSpec);
                aPC2 = Geom2dOracle.ToPort(aSpec);
                aF = aPC.FirstParameter();
                aL = aPC.LastParameter();
                if (anOracle.Bool())
                {
                    double a = anOracle.Real(aF, aF + 0.4 * (aL - aF));
                    aL = anOracle.Real(aF + 0.6 * (aL - aF), aL);
                    aF = a;
                }
            }
            else if (aKind == 2)
            {
                // A 2D circle or ellipse against a 3D circle: the elementary (trigonometric) path
                (N.gp_Ax2d aNAx2d, gp_Ax2d aAx2d) = GeomOracleInputs.Ax2ds(anOracle);
                double aR = anOracle.Real(0.5, 5.0);
                using (aNAx2d)
                {
                    if (anOracle.Bool())
                    {
                        aNC2 = new N.Geom2d_Circle(aNAx2d, aR);
                        aPC2 = new Geom2d_Circle(aAx2d, aR);
                    }
                    else
                    {
                        double aMinor = aR * anOracle.Real(0.2, 1.0);
                        aNC2 = new N.Geom2d_Ellipse(aNAx2d, aR, aMinor);
                        aPC2 = new Geom2d_Ellipse(aAx2d, aR, aMinor);
                    }
                }
                (N.gp_Ax2 aNAx2, gp_Ax2 aAx2) = GeomOracleInputs.Ax2s(anOracle);
                double aR3 = anOracle.Real(0.5, 5.0);
                using (aNAx2)
                {
                    aNC = new N.Geom_Circle(aNAx2, aR3);
                    aPC = new Geom_Circle(aAx2, aR3);
                }
                aF = anOracle.Real(-Math.PI, Math.PI);
                aL = aF + anOracle.Real(0.5, 2 * Math.PI);
            }
            else
            {
                // Two lines: the end-point path
                (N.gp_Ax2d aNAx2d, gp_Ax2d aAx2d) = GeomOracleInputs.Ax2ds(anOracle);
                using (aNAx2d)
                {
                    aNC2 = new N.Geom2d_Line(aNAx2d);
                    aPC2 = new Geom2d_Line(aAx2d);
                }
                (N.gp_Ax1 aNAx1, gp_Ax1 aAx1) = GeomOracleInputs.Ax1s(anOracle);
                using (aNAx1)
                {
                    aNC = new N.Geom_Line(aNAx1);
                    aPC = new Geom_Line(aAx1);
                }
                aF = anOracle.Real(-20, 10);
                aL = aF + anOracle.Real(0.1, 20);
            }

            using (aNS)
            using (aNC)
            using (aNC2)
            using (var aNP1 = new N.gp_Pnt(0, 0, 0))
            using (var aNP2 = new N.gp_Pnt(1, 0, 0))
            using (var aME = new N.BRepBuilderAPI_MakeEdge(aNP1, aNP2))
            using (N.TopoDS_Edge aNE = aME.Edge())
            {
                double aTol = N.BRep_Tool.Tolerance(aNE);
                double aNative = N.BRepTools.EvalAndUpdateTol(aNE, aNC, aNC2, aNS, aF, aL);
                double aPort = EvalAndUpdateTol(aTol, aPC, aPC2, aPS, aF, aL, out int aStatus);
                anOracle.Same(aNative, aPort, $"kind {aKind} #{c} [{aF:R}, {aL:R}] status {aStatus}");

                // the port's own EvalAndUpdateTol on an unfrozen edge, and its functional twin on a frozen one
                var aB = new BRep_Builder();
                TopoDS_Edge anE = new();
                aB.MakeEdge(ref anE);
                aB.UpdateEdge(anE, aTol);
                TopoDS_Edge aFrozenE = new();
                aB.MakeEdge(ref aFrozenE);
                aB.UpdateEdge(aFrozenE, aTol);
                aFrozenE.TShape()!.FreezeGraph();
                anOracle.Same(aNative, BRepTools.EvalAndUpdateTol(anE, aPC, aPC2, aPS, aF, aL), "BRepTools.EvalAndUpdateTol");
                anOracle.Same(N.BRep_Tool.Tolerance(aNE), BRep_Tool.Tolerance(anE), "the edge's updated tolerance");
                TopoDS_Edge aTwinE = BRepTools.TolEvaluatedAndUpdated(aFrozenE, aPC, aPC2, aPS, aF, aL, out double aTwinTol);
                anOracle.Same(aNative, aTwinTol, "BRepTools.TolEvaluatedAndUpdated");
                anOracle.Same(N.BRep_Tool.Tolerance(aNE), BRep_Tool.Tolerance(aTwinE), "the twin's tolerance");
                Assert.Equal(aTol, BRep_Tool.Tolerance(aFrozenE));
                aCases++;
                aKinds[aKind]++;
                if (aNative > aTol)
                {
                    aAboveTol++;
                }
                if (aStatus != 0)
                {
                    aFailed++;
                }
            }
        }
        output.WriteLine($"{aCases} cases ({string.Join(", ", aKinds)} by kind), {aAboveTol} above the edge tolerance, " +
                         $"{aFailed} where the check failed");
        anOracle.Report("GeomLib_CheckCurveOnSurface through BRepTools::EvalAndUpdateTol");
    }
}
#endif
