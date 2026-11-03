// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit.Abstractions;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// GeomLib_CheckCurveOnSurface against native OCCT through BRepTools::EvalAndUpdateTol, its one bound caller. The
/// port's side mirrors EvalAndUpdateTol: the range clamped to the non-periodic curves', a GeomAdaptor_Curve and a
/// Geom2dAdaptor_Curve on it, an Adaptor3d_CurveOnSurface on a GeomAdaptor_Surface, then the sampling fallback when
/// the check fails, and the edge's tolerance as the floor. Inputs are built from the same raw numbers on both sides.
/// </summary>
public class GeomLibCheckCurveOnSurfaceOracleTests(ITestOutputHelper output)
{
    // The port's BRepTools::EvalAndUpdateTol, for an edge of tolerance theTol.
    private static double EvalAndUpdateTol(double theTol, Curve3D C3d, Curve2D C2d, Surface S, double f,
                                           double l, out int theStatus)
    {
        double newtol = 0.0;
        double first = f, last = l;
        if (!C3d.IsPeriodic)
        {
            first = Math.Max(first, C3d.FirstParameter());
            last = Math.Min(last, C3d.LastParameter());
        }
        if (!C2d.IsPeriodic)
        {
            first = Math.Max(first, C2d.FirstParameter());
            last = Math.Min(last, C2d.LastParameter());
        }
        CurveAdaptor aGeomAdaptorCurve = new(C3d, first, last);
        CurveAdaptor2D aGeom2dAdaptorCurve = new(C2d, first, last);
        SurfaceAdaptor aGeomAdaptorSurface = new(S);
        CurveOnSurface anAdaptor3dCurveOnSurface = new(aGeom2dAdaptorCurve, aGeomAdaptorSurface);

        CheckCurveOnSurface CT = new(aGeomAdaptorCurve);
        CT.Perform(anAdaptor3dCurveOnSurface);
        theStatus = CT.ErrorStatus();
        if (CT.IsDone)
        {
            newtol = CT.MaxDistance();
        }
        else if (CT.ErrorStatus() == 3 || (CT.ErrorStatus() == 2 && (C3d.IsPeriodic || C2d.IsPeriodic)))
        {
            // Try to estimate by sample points
            int nbint = 22;
            double dt = (last - first) / nbint;
            dt = Math.Max(dt, global::Kerf.Engine.Foundation.Precision.Confusion());
            double d, dmax = 0.0;
            int cnt = 0;
            double t = first;
            for (; t <= last; t += dt)
            {
                cnt++;
                Point2D aP2d = C2d.Value(t);
                Point3D aPC = C3d.Value(t);
                Point3D aPS = S.Value(aP2d.X(), aP2d.Y());
                d = aPS.SquareDistance(aPC);
                if (d > dmax)
                {
                    dmax = d;
                }
            }
            if (cnt < nbint + 1)
            {
                t = last;
                Point2D aP2d = C2d.Value(t);
                Point3D aPC = C3d.Value(t);
                Point3D aPS = S.Value(aP2d.X(), aP2d.Y());
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
            Surface aPS;
            N.Geom_Curve aNC;
            Curve3D aPC;
            N.Geom2d_Curve aNC2;
            Curve2D aPC2;
            double aF, aL;
            (N.gp_Ax3 aNAx3, CoordinateSystem aAx3) = GeomOracleInputs.Ax3s(anOracle);
            using (aNAx3)
            {
                if (aKind == 1)
                {
                    double aR = anOracle.Real(0.5, 5.0);
                    aNS = new N.Geom_CylindricalSurface(aNAx3, aR);
                    aPS = new CylindricalSurface(aAx3, aR);
                }
                else
                {
                    aNS = new N.Geom_Plane(aNAx3);
                    aPS = new PlaneSurface(aAx3);
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
                        aSpec.Poles[i] = new Point2D(aSpec.Poles[i].X() * 0.3, aSpec.Poles[i].Y());
                    }
                }
                double aNoise = anOracle.Int(0, 3) == 0 ? 0.0 : anOracle.Real(0.0, 0.5);
                NCollection_Array1<Point3D> aPoles3d = new(1, aSpec.Poles.Length);
                for (int i = 0; i < aSpec.Poles.Length; i++)
                {
                    Point3D aP = aPS.Value(aSpec.Poles[i].X(), aSpec.Poles[i].Y());
                    aPoles3d[i + 1] = new Point3D(aP.X() + aNoise * anOracle.Real(-1, 1),
                                                 aP.Y() + aNoise * anOracle.Real(-1, 1),
                                                 aP.Z() + aNoise * anOracle.Real(-1, 1));
                }
                NCollection_Array1<double>? aWeights =
                    aSpec.Weights is null ? null : NCollection_Array1<double>.From(aSpec.Weights);
                NCollection_Array1<double> aKnots = NCollection_Array1<double>.From(aSpec.Knots);
                NCollection_Array1<int> aMults = NCollection_Array1<int>.From(aSpec.Mults);
                aNC = Oracle.ToNativeCurve(aPoles3d, aWeights, aKnots, aMults, aSpec.Degree, false);
                aPC = aWeights is null
                          ? new BSplineCurve3D(aPoles3d, aKnots, aMults, aSpec.Degree)
                          : new BSplineCurve3D(aPoles3d, aWeights!, aKnots, aMults, aSpec.Degree);
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
                (N.gp_Ax2d aNAx2d, Axis2D aAx2d) = GeomOracleInputs.Ax2ds(anOracle);
                double aR = anOracle.Real(0.5, 5.0);
                using (aNAx2d)
                {
                    if (anOracle.Bool())
                    {
                        aNC2 = new N.Geom2d_Circle(aNAx2d, aR);
                        aPC2 = new CircleCurve2D(aAx2d, aR);
                    }
                    else
                    {
                        double aMinor = aR * anOracle.Real(0.2, 1.0);
                        aNC2 = new N.Geom2d_Ellipse(aNAx2d, aR, aMinor);
                        aPC2 = new EllipseCurve2D(aAx2d, aR, aMinor);
                    }
                }
                (N.gp_Ax2 aNAx2, Frame3D aAx2) = GeomOracleInputs.Ax2s(anOracle);
                double aR3 = anOracle.Real(0.5, 5.0);
                using (aNAx2)
                {
                    aNC = new N.Geom_Circle(aNAx2, aR3);
                    aPC = new CircleCurve3D(aAx2, aR3);
                }
                aF = anOracle.Real(-Math.PI, Math.PI);
                aL = aF + anOracle.Real(0.5, 2 * Math.PI);
            }
            else
            {
                // Two lines: the end-point path
                (N.gp_Ax2d aNAx2d, Axis2D aAx2d) = GeomOracleInputs.Ax2ds(anOracle);
                using (aNAx2d)
                {
                    aNC2 = new N.Geom2d_Line(aNAx2d);
                    aPC2 = new LineCurve2D(aAx2d);
                }
                (N.gp_Ax1 aNAx1, Axis3D aAx1) = GeomOracleInputs.Ax1s(anOracle);
                using (aNAx1)
                {
                    aNC = new N.Geom_Line(aNAx1);
                    aPC = new LineCurve3D(aAx1);
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
                var aB = new BrepBuilder();
                Edge anE = new();
                aB.MakeEdge(ref anE);
                aB.UpdateEdge(anE, aTol);
                Edge aFrozenE = new();
                aB.MakeEdge(ref aFrozenE);
                aB.UpdateEdge(aFrozenE, aTol);
                aFrozenE.TShape()!.FreezeGraph();
                anOracle.Same(aNative, BrepTools.EvalAndUpdateTol(anE, aPC, aPC2, aPS, aF, aL), "BRepTools.EvalAndUpdateTol");
                anOracle.Same(N.BRep_Tool.Tolerance(aNE), Tool.Tolerance(anE), "the edge's updated tolerance");
                Edge aTwinE = BrepTools.TolEvaluatedAndUpdated(aFrozenE, aPC, aPC2, aPS, aF, aL, out double aTwinTol);
                anOracle.Same(aNative, aTwinTol, "BRepTools.TolEvaluatedAndUpdated");
                anOracle.Same(N.BRep_Tool.Tolerance(aNE), Tool.Tolerance(aTwinE), "the twin's tolerance");
                Assert.Equal(aTol, Tool.Tolerance(aFrozenE));
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
