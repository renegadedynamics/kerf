// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// GeomLib against native OCCT through BRepLib, on edges built natively by BRep_Builder from the same raw numbers as
/// the port's inputs:
/// <list type="bullet">
/// <item>GeomLib::BuildCurve3d (with To3d, isIsoLine, buildC3dOnIsoLine and the AdvApprox path) through
/// BRepLib::BuildCurve3d on edges having only a pcurve. The port mirrors BRepLib's call: Geom2dAdaptor_Curve of the
/// pcurve on the edge's range, GeomAdaptor_Surface, Adaptor3d_CurveOnSurface, BRepLib's evaluateMaxSegment; on a plane
/// (or a trimmed plane) BRepLib calls GeomLib::To3d itself, which is GeomLib::BuildCurve3d's own plane branch.</item>
/// <item>GeomLib::SameRange through BRepLib::SameRange on edges whose pcurve has another range than the 3D curve.</item>
/// </list>
/// </summary>
public class GeomLibOracleTests(ITestOutputHelper output)
{
    // BRepLib.cxx's evaluateMaxSegment, as OCCT writes it.
    private static int evaluateMaxSegment(int aMaxSegment, Adaptor3d_CurveOnSurface aCurveOnSurface)
    {
        if (aMaxSegment != 0)
        {
            return aMaxSegment;
        }

        Adaptor3d_Surface aSurf = aCurveOnSurface.GetSurface()!;
        Adaptor2d_Curve2d aCurv2d = aCurveOnSurface.GetCurve()!;

        double aNbSKnots = 0, aNbC2dKnots = 0;

        if (aSurf.GetType() == GeomAbs_BSplineSurface)
        {
            Geom_BSplineSurface aBSpline = aSurf.BSpline();
            aNbSKnots = Math.Max(aBSpline.NbUKnots(), aBSpline.NbVKnots());
        }
        if (aCurv2d.GetType() == GeomAbs_BSplineCurve)
        {
            aNbC2dKnots = aCurv2d.NbKnots();
        }
        int aReturn = (int)(30 + Math.Max(aNbSKnots, aNbC2dKnots));
        return aReturn;
    }

    // A finite box inside the surface's bounds.
    private static (double U1, double U2, double V1, double V2) Box(Oracle theO, Geom_Surface theS)
    {
        theS.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        if (Precision.IsInfinite(aU1) || Precision.IsInfinite(aU2))
        {
            aU1 = -4;
            aU2 = 4;
        }
        if (Precision.IsInfinite(aV1) || Precision.IsInfinite(aV2))
        {
            aV1 = -4;
            aV2 = 4;
        }
        double aDU = aU2 - aU1, aDV = aV2 - aV1;
        double u1 = theO.Real(aU1 + 0.05 * aDU, aU1 + 0.4 * aDU), u2 = theO.Real(aU1 + 0.6 * aDU, aU2 - 0.05 * aDU);
        double v1 = theO.Real(aV1 + 0.05 * aDV, aV1 + 0.4 * aDV), v2 = theO.Real(aV1 + 0.6 * aDV, aV2 - 0.05 * aDV);
        return (u1, u2, v1, v2);
    }

    private sealed class PCurve
    {
        public required Geom2d_Curve Port;
        public required N.Geom2d_Curve Native;
        public double First, Last;
        public required string Kind;
    }

    // A pcurve inside the box: an isoline (a line or a degree-1 B-spline, either sense, any parameter origin), a
    // diagonal line, a circle arc or a free B-spline.
    private static PCurve RandomPCurve(Oracle theO, double u1, double u2, double v1, double v2)
    {
        double c = theO.Real(-3, 3);
        switch (theO.Int(0, 4))
        {
            case 0:
            case 1:
            {
                bool anIsU = theO.Bool(), aFwd = theO.Bool();
                double aFix = anIsU ? theO.Real(u1, u2) : theO.Real(v1, v2);
                gp_Pnt2d aS = anIsU ? new gp_Pnt2d(aFix, aFwd ? v1 : v2) : new gp_Pnt2d(aFwd ? u1 : u2, aFix);
                gp_Pnt2d aE = anIsU ? new gp_Pnt2d(aFix, aFwd ? v2 : v1) : new gp_Pnt2d(aFwd ? u2 : u1, aFix);
                double aLen = aS.Distance(aE);
                if (theO.Bool())
                {
                    double dx = anIsU ? 0.0 : (aFwd ? 1.0 : -1.0), dy = anIsU ? (aFwd ? 1.0 : -1.0) : 0.0;
                    gp_Pnt2d aP = new(aS.X() - c * dx, aS.Y() - c * dy);
                    using var aNP = Oracle.ToNative(aP);
                    using var aND = new N.gp_Dir2d(dx, dy);
                    return new PCurve { Port = new Geom2d_Line(aP, new gp_Dir2d(dx, dy)), Native = new N.Geom2d_Line(aNP, aND),
                        First = c, Last = c + aLen, Kind = "iso line" };
                }
                NCollection_Array1<gp_Pnt2d> aPoles = NCollection_Array1<gp_Pnt2d>.From([aS, aE]);
                NCollection_Array1<double> aKnots = NCollection_Array1<double>.From([c, c + aLen]);
                NCollection_Array1<int> aMults = NCollection_Array1<int>.From([2, 2]);
                return new PCurve { Port = new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 1),
                    Native = Oracle.ToNativeCurve(aPoles, null, aKnots, aMults, 1, false), First = c, Last = c + aLen,
                    Kind = "iso B-spline" };
            }
            case 2:
            {
                gp_Pnt2d aS = new(theO.Real(u1, u2), theO.Real(v1, v2));
                gp_Pnt2d aE = new(theO.Real(u1, u2), theO.Real(v1, v2));
                double dx = aE.X() - aS.X(), dy = aE.Y() - aS.Y();
                double aLen = Math.Sqrt(dx * dx + dy * dy);
                if (aLen < 1e-3)
                {
                    aE = new gp_Pnt2d(u2, v2);
                    aS = new gp_Pnt2d(u1, v1);
                    dx = aE.X() - aS.X();
                    dy = aE.Y() - aS.Y();
                    aLen = Math.Sqrt(dx * dx + dy * dy);
                }
                gp_Dir2d aD = new(dx, dy);
                using var aNP = Oracle.ToNative(aS);
                using var aND = new N.gp_Dir2d(dx, dy);
                return new PCurve { Port = new Geom2d_Line(aS, aD), Native = new N.Geom2d_Line(aNP, aND), First = 0.0,
                    Last = aLen, Kind = "line" };
            }
            case 3:
            {
                double r = 0.4 * Math.Min(u2 - u1, v2 - v1);
                gp_Pnt2d aC = new(0.5 * (u1 + u2), 0.5 * (v1 + v2));
                using var aNC = Oracle.ToNative(aC);
                using var aNX = new N.gp_Dir2d(1, 0);
                using var aNA = new N.gp_Ax2d(aNC, aNX);
                double a = theO.Real(0, 3), b = a + theO.Real(0.3, 3);
                return new PCurve { Port = new Geom2d_Circle(new gp_Ax2d(aC, new gp_Dir2d(1, 0)), r),
                    Native = new N.Geom2d_Circle(aNA, r), First = a, Last = b, Kind = "circle" };
            }
            default:
            {
                int aDeg = theO.Int(1, 4);
                int aNbKnots = theO.Int(2, 4);
                NCollection_Array1<double> aKnots = new(1, aNbKnots);
                NCollection_Array1<int> aMults = new(1, aNbKnots);
                double k = c;
                for (int i = 1; i <= aNbKnots; i++)
                {
                    aKnots[i] = k;
                    k += theO.Real(0.3, 2);
                    aMults[i] = (i == 1 || i == aNbKnots) ? aDeg + 1 : theO.Int(1, aDeg);
                }
                int aNbPoles = Occt.BSplCLib.NbPoles(aDeg, false, aMults);
                NCollection_Array1<gp_Pnt2d> aPoles = new(1, aNbPoles);
                for (int i = 1; i <= aNbPoles; i++)
                {
                    aPoles[i] = new gp_Pnt2d(theO.Real(u1, u2), theO.Real(v1, v2));
                }
                return new PCurve { Port = new Geom2d_BSplineCurve(aPoles, aKnots, aMults, aDeg),
                    Native = Oracle.ToNativeCurve(aPoles, null, aKnots, aMults, aDeg, false), First = aKnots[1],
                    Last = aKnots[aNbKnots], Kind = $"B-spline deg {aDeg}" };
            }
        }
    }

    // The 3D curves: B-splines by structure; every curve by its values and first derivatives on [theF, theL].
    private static void SameCurve(Oracle theO, N.Geom_Curve theN, Geom_Curve theP, double theF, double theL, string theWhat)
    {
        if (theP is Geom_BSplineCurve aBSpline)
        {
            using var aNBSpline = N.Geom_BSplineCurve.DownCast(theN);
            Assert.True(aNBSpline is not null, $"{theWhat}: native result is not a B-spline");
            theO.Same(aNBSpline!.Degree(), aBSpline.Degree(), theWhat + " degree");
            theO.Same(aNBSpline.IsRational(), aBSpline.IsRational(), theWhat + " IsRational");
            theO.Same(aNBSpline.IsPeriodic(), aBSpline.IsPeriodic(), theWhat + " IsPeriodic");
            theO.Same(aNBSpline.NbPoles(), aBSpline.NbPoles(), theWhat + " NbPoles");
            theO.Same(aNBSpline.NbKnots(), aBSpline.NbKnots(), theWhat + " NbKnots");
            for (int i = 1; i <= aBSpline.NbPoles(); i++)
            {
                using N.gp_Pnt aNPole = aNBSpline.Pole(i);
                theO.Same(aNPole, aBSpline.Pole(i), theWhat + " pole");
                theO.Same(aNBSpline.Weight(i), aBSpline.Weight(i), theWhat + " weight");
            }
            for (int i = 1; i <= aBSpline.NbKnots(); i++)
            {
                theO.Same(aNBSpline.Knot(i), aBSpline.Knot(i), theWhat + " knot");
                theO.Same(aNBSpline.Multiplicity(i), aBSpline.Multiplicity(i), theWhat + " multiplicity");
            }
        }
        for (int i = 0; i <= 10; i++)
        {
            double t = theF + (theL - theF) * i / 10.0;
            using var aNP = new N.gp_Pnt();
            using var aNV = new N.gp_Vec();
            theN.D1(t, aNP, aNV);
            theP.D1(t, out gp_Pnt aP, out gp_Vec aV);
            theO.Same(aNP, aP, $"{theWhat} D1({t}) point");
            theO.Same(aNV, aV, $"{theWhat} D1({t}) derivative");
        }
    }

    /// <summary>BuildCurve3d on random edges, compared as the class summary says.</summary>
    [Fact]
    public void BuildCurve3d_through_BRepLib_BuildCurve3d()
    {
        var anOracle = new Oracle(output, 20261106);
        int aBuilt = 0, aFailed = 0, aPlanes = 0, anIsos = 0;
        int[] aKinds = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10];
        for (int c = 0; c < 440; c++)
        {
            int aKind = aKinds[c % aKinds.Length];
            (N.Geom_Surface aNS, Geom_Surface aPS) = AdaptorOracleInputs.RandomSurface(anOracle, aKind);
            using var aNSOwner = aNS;
            PCurve aPC;
            if (aKind == 0)
            {
                // On a plane, any 2D curve (BRepLib calls GeomLib::To3d on the stored pcurve)
                (N.Geom2d_Curve aNC, Geom2d_Curve aC, string aCKind) = TestAdaptorOracleTests.RandomCurve2d(anOracle);
                (double f, double l) = TestAdaptorOracleTests.Bounds(anOracle, aC.FirstParameter(), aC.LastParameter(),
                                                                     aC.IsPeriodic());
                if (anOracle.Bool())
                {
                    aPC = new PCurve { Port = new Geom2d_TrimmedCurve(aC, f, l), Native = new N.Geom2d_TrimmedCurve(aNC, f, l),
                        First = f, Last = l, Kind = "trimmed " + aCKind };
                    aNC.Dispose();
                }
                else
                {
                    aPC = new PCurve { Port = aC, Native = aNC, First = f, Last = l, Kind = aCKind };
                }
                aPlanes++;
            }
            else
            {
                (double u1, double u2, double v1, double v2) = Box(anOracle, aPS);
                aPC = RandomPCurve(anOracle, u1, u2, v1, v2);
            }
            double aTol = anOracle.Int(0, 2) switch { 0 => 1e-7, 1 => 1e-5, _ => 1e-3 };
            GeomAbs_Shape aCont = (GeomAbs_Shape)(anOracle.Int(0, 2) * 2); // C0, C1, C2
            int aMaxDeg = anOracle.Int(8, 14);
            int aMaxSeg = anOracle.Int(0, 2) switch { 0 => 0, 1 => 10, _ => 30 };
            string aWhat = $"case {c} ({AdaptorOracleInputs.SurfaceKinds[aKind]}, {aPC.Kind}, tol {aTol}, {aCont}, deg {aMaxDeg}, seg {aMaxSeg})";

            // Native: an edge with only the pcurve, then BRepLib::BuildCurve3d
            using var aNPC = aPC.Native;
            using var aB = new N.BRep_Builder();
            using var aNE = new N.TopoDS_Edge();
            using var aLoc = new N.TopLoc_Location();
            aB.MakeEdge(aNE);
            aB.UpdateEdge(aNE, aNPC, aNS, aLoc, aTol);
            aB.Range(aNE, aPC.First, aPC.Last);
            bool aNOk = N.BRepLib.BuildCurve3d(aNE, aTol, (N.GeomAbs_Shape)(int)aCont, aMaxDeg, aMaxSeg);
            double aNF = 0, aNL = 0;
            using N.Geom_Curve? aNC3d = N.BRep_Tool.Curve(aNE, ref aNF, ref aNL);

            // Port: BRepLib's path
            Geom2dAdaptor_Curve AnAdaptor3dCurve2d = new(aPC.Port, aPC.First, aPC.Last);
            GeomAdaptor_Surface AnAdaptor3dSurface = new(aPS);
            Adaptor3d_CurveOnSurface CurveOnSurface = new(AnAdaptor3dCurve2d, AnAdaptor3dSurface);
            Geom_Curve? NewCurvePtr = null;
            double aMaxDev = -1.0;
            Geom_Plane? P = aPS is Geom_RectangularTrimmedSurface RT ? RT.BasisSurface() as Geom_Plane : aPS as Geom_Plane;
            if (P is not null)
            {
                // BRepLib's own plane branch: To3d of the stored pcurve (GeomLib::BuildCurve3d's plane branch would
                // read the adaptor's curve, which is the basis of a trimmed pcurve)
                NewCurvePtr = GeomLib.To3d(P.Position().Ax2(), aPC.Port);
            }
            else
            {
                GeomLib.BuildCurve3d(aTol, CurveOnSurface, aPC.First, aPC.Last, ref NewCurvePtr, out aMaxDev,
                                     out double anAvgDev, aCont, aMaxDeg, evaluateMaxSegment(aMaxSeg, CurveOnSurface));
            }

            anOracle.Same(aNOk, NewCurvePtr is not null, aWhat + " built");
            if (NewCurvePtr is null)
            {
                aFailed++;
                continue;
            }
            Assert.True(aNC3d is not null, aWhat + ": no native curve");
            SameCurve(anOracle, aNC3d!, NewCurvePtr, aPC.First, aPC.Last, aWhat);
            if (aKind != 0 && NewCurvePtr is Geom_BSplineCurve && aMaxDev == 0.0)
            {
                anIsos++;
            }
            aBuilt++;
        }
        output.WriteLine($"{aBuilt} curves built ({aPlanes} on planes, {anIsos} from isolines), {aFailed} not built");
        Assert.True(aBuilt > 350, "enough curves built");
        anOracle.Report("GeomLib::BuildCurve3d through BRepLib::BuildCurve3d");
    }

    /// <summary>
    /// SameRange: an edge with a 3D curve on [f, l] and a pcurve on a plane whose range is [f + d, l + d] (same length)
    /// or another range; BRepLib::SameRange gives the pcurve the 3D range through GeomLib::SameRange.
    /// </summary>
    [Fact]
    public void SameRange_through_BRepLib_SameRange()
    {
        var anOracle = new Oracle(output, 20261107);
        int aCompared = 0;
        for (int c = 0; c < 600; c++)
        {
            (N.Geom_Surface aNS, Geom_Surface aPS) = AdaptorOracleInputs.RandomSurface(anOracle, 0);
            using var aNSOwner = aNS;
            (N.Geom2d_Curve aNC, Geom2d_Curve aC, string aCKind) = TestAdaptorOracleTests.RandomCurve2d(anOracle);
            using var aNCOwner = aNC;
            (double f, double l) = TestAdaptorOracleTests.Bounds(anOracle, aC.FirstParameter(), aC.LastParameter(),
                                                                 aC.IsPeriodic());
            N.Geom2d_Curve aNPC = aNC;
            Geom2d_Curve aPPC = aC;
            string aKind = aCKind;
            if (anOracle.Int(0, 3) == 0)
            {
                aNPC = new N.Geom2d_TrimmedCurve(aNC, f, l);
                aPPC = new Geom2d_TrimmedCurve(aC, f, l);
                aKind = "trimmed " + aCKind;
            }
            // The 3D range: the same length moved (or not), or another length
            double aShift = anOracle.Int(0, 3) == 0 ? 0.0 : anOracle.Real(-3, 3);
            double aRF = f + aShift;
            double aRL = anOracle.Int(0, 2) == 0 ? aRF + (l - f) * anOracle.Real(0.3, 2.5) : l + aShift;
            double aTol = anOracle.Int(0, 2) switch { 0 => 1e-9, 1 => 1e-7, _ => 1e-4 };
            string aWhat = $"case {c} ({aKind}, [{f}, {l}] to [{aRF}, {aRL}], tol {aTol})";

            // Native: a 3D line on [aRF, aRL], the pcurve on [f, l]
            using var aNP0 = new N.gp_Pnt(0, 0, 0);
            using var aNDX = new N.gp_Dir(1, 0, 0);
            using var aNLine = new N.Geom_Line(aNP0, aNDX);
            using var aB = new N.BRep_Builder();
            using var aNE = new N.TopoDS_Edge();
            using var aLoc = new N.TopLoc_Location();
            aB.MakeEdge(aNE, aNLine, aTol);
            aB.Range(aNE, aRF, aRL, true);
            aB.UpdateEdge(aNE, aNPC, aNS, aLoc, aTol);
            aB.Range(aNE, aNS, aLoc, f, l);
            Exception? aNErr = AdaptorOracleInputs.Catch(() => N.BRepLib.SameRange(aNE, aTol));
            double aNF = 0, aNL = 0;
            using N.Geom2d_Curve? aNRes = aNErr is null ? N.BRep_Tool.CurveOnSurface(aNE, aNS, aLoc, ref aNF, ref aNL) : null;

            // Port: BRepLib::SameRange calls GeomLib::SameRange with the pcurve's range and the 3D range
            Geom2d_Curve? aPRes = null;
            Exception? aPErr = AdaptorOracleInputs.Catch(() => GeomLib.SameRange(aTol, aPPC, f, l, aRF, aRL, ref aPRes));
            if (!AdaptorOracleInputs.SameOutcome(aNErr, aPErr, aWhat))
            {
                continue;
            }
            // OCCT stores the pcurve OCCT leaves unchanged (a null handle) when no case applies: not reached here.
            Assert.True(aPRes is not null && aNRes is not null, aWhat + ": no pcurve");
            anOracle.Same(aNF, aRF, aWhat + " first");
            anOracle.Same(aNL, aRL, aWhat + " last");
            if (aPRes is Geom2d_BSplineCurve aPB)
            {
                using N.Geom2d_BSplineCurve? aNB = N.Geom2d_BSplineCurve.DownCast(aNRes);
                Assert.True(aNB is not null, $"{aWhat}: native pcurve is not a B-spline");
                Geom2dOracle.SameStructure(anOracle, aNB!, aPB, aWhat);
            }
            for (int i = 0; i <= 10; i++)
            {
                double t = aRF + (aRL - aRF) * i / 10.0;
                using N.gp_Pnt2d aNP = aNRes!.Value(t);
                anOracle.Same(aNP, aPRes!.Value(t), $"{aWhat} Value({t})");
            }
            aCompared++;
            if (!ReferenceEquals(aNPC, aNC))
            {
                aNPC.Dispose();
            }
        }
        output.WriteLine($"{aCompared} pcurves compared");
        anOracle.Report("GeomLib::SameRange through BRepLib::SameRange");
    }
}
