// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using static Occt.Standard;
using static Occt.Convert_ParameterisationType;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Approx_SameParameter against native OCCT through BRepLib::SameParameter(edge, tolerance, newTol, false). Each edge
/// has a 3D curve and one pcurve on a plane, a cylinder or a B-spline surface, built natively by BRep_Builder from the
/// same raw numbers as the port's inputs (same range, SameRange set, SameParameter cleared). The pcurves are not
/// same-parameter with the 3D curve: rational B-spline circles under an angular circle, lines and isolines run at a
/// non-uniform speed, B-spline images with perturbed poles (or, for a tenth of the cases, they are). The port mirrors
/// BRepLib's path for such edges (no C0 pcurve, so no C0BSplineToC1BSplineCurve): GeomAdaptor_Curve on the edge
/// range, Geom2dAdaptor_Curve of the pcurve on that range, GeomAdaptor_Surface, BRepLib's ComputeTol (22 samples),
/// then Approx_SameParameter and BRepLib's choice of the pcurve and the tolerance stored.
/// </summary>
public class ApproxSameParameterOracleTests(ITestOutputHelper output)
{
    private sealed class EdgeCase
    {
        public required Geom_Surface PortS;
        public required N.Geom_Surface NativeS;
        public required Geom_Curve PortC3d;
        public required N.Geom_Curve NativeC3d;
        public required Geom2d_Curve PortPC;
        public required N.Geom2d_Curve NativePC;
        public double First, Last;
        public required string Kind;
    }

    // BRepLib.cxx's ComputeTol, as OCCT writes it.
    private static double ComputeTol(Adaptor3d_Curve c3d, Adaptor2d_Curve2d c2d, Adaptor3d_Surface surf, int nbp)
    {
        NCollection_Array1<double> dist = new(1, nbp + 10);
        dist.Init(-1.0);

        double uf = surf.FirstUParameter(), ul = surf.LastUParameter(), vf = surf.FirstVParameter(),
               vl = surf.LastVParameter();
        double du = 0.01 * (ul - uf), dv = 0.01 * (vl - vf);
        bool isUPeriodic = surf.IsUPeriodic(), isVPeriodic = surf.IsVPeriodic();
        double DSdu = 1.0 / surf.UResolution(1.0), DSdv = 1.0 / surf.VResolution(1.0);
        double d2 = 0.0;
        double first = c3d.FirstParameter();
        double last = c3d.LastParameter();
        double dapp = -1.0;
        for (int i = 0; i <= nbp; ++i)
        {
            double t = IntToReal(i) / IntToReal(nbp);
            double u = first * (1.0 - t) + last * t;
            gp_Pnt Pc3d = c3d.Value(u);
            gp_Pnt2d Puv = c2d.Value(u);
            if (!isUPeriodic)
            {
                if (Puv.X() < uf - du)
                {
                    dapp = Max(dapp, DSdu * (uf - Puv.X()));
                    continue;
                }
                else if (Puv.X() > ul + du)
                {
                    dapp = Max(dapp, DSdu * (Puv.X() - ul));
                    continue;
                }
            }
            if (!isVPeriodic)
            {
                if (Puv.Y() < vf - dv)
                {
                    dapp = Max(dapp, DSdv * (vf - Puv.Y()));
                    continue;
                }
                else if (Puv.Y() > vl + dv)
                {
                    dapp = Max(dapp, DSdv * (Puv.Y() - vl));
                    continue;
                }
            }
            gp_Pnt Pcons = surf.Value(Puv.X(), Puv.Y());
            if (Precision.IsInfinite(Pcons.X()) || Precision.IsInfinite(Pcons.Y()) || Precision.IsInfinite(Pcons.Z()))
            {
                d2 = Precision.Infinite();
                break;
            }
            double temp = Pc3d.SquareDistance(Pcons);
            dist[i + 1] = temp;
            d2 = Max(d2, temp);
        }

        if (Precision.IsInfinite(d2))
        {
            return d2;
        }

        d2 = Math.Sqrt(d2);
        if (dapp > d2)
        {
            return dapp;
        }

        bool ana = false;
        double D2 = 0;
        int N1 = 0, N2 = 0, N3 = 0;
        for (int i = 1; i <= nbp + 10; ++i)
        {
            if (dist[i] > 0)
            {
                if (dist[i] < 1.0)
                {
                    ++N1;
                }
                else
                {
                    ++N2;
                }
            }
        }

        if (N1 > N2 && N2 != 0)
        {
            N3 = 100 * N2 / (N1 + N2);
        }
        if (N3 < 10 && N3 != 0)
        {
            ana = true;
            for (int i = 1; i <= nbp + 10; ++i)
            {
                if (dist[i] > 0 && dist[i] < 1.0)
                {
                    D2 = Max(D2, dist[i]);
                }
            }
        }

        d2 = (!ana) ? 1.5 * d2 : 1.5 * Math.Sqrt(D2);
        d2 = Max(d2, 1.0e-7);
        return d2;
    }

    // A 2D B-spline (degree 2 or 3, one span) from theP1 to theP2 on [theT1, theT2] whose speed varies: the inner
    // poles are on the segment at uneven fractions (all at the regular ones when theWarp is false), then moved by
    // theNoise across it.
    private static (Geom2d_BSplineCurve, N.Geom2d_BSplineCurve) WarpedSegment(Oracle theO, gp_Pnt2d theP1, gp_Pnt2d theP2,
        double theT1, double theT2, bool theWarp, double theNoise)
    {
        int aDeg = theO.Int(2, 3);
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, aDeg + 1);
        double aDX = theP2.X() - theP1.X(), aDY = theP2.Y() - theP1.Y();
        double aLen = Math.Sqrt(aDX * aDX + aDY * aDY);
        double aNX = -aDY / aLen, aNY = aDX / aLen;
        double aPrev = 0.0;
        for (int i = 1; i <= aDeg + 1; i++)
        {
            double aFrac;
            if (i == 1)
            {
                aFrac = 0.0;
            }
            else if (i == aDeg + 1)
            {
                aFrac = 1.0;
            }
            else
            {
                double aRegular = (i - 1) / (double)aDeg;
                aFrac = theWarp ? theO.Real(aPrev + 0.05, Math.Min(1.0, aRegular + 0.35) - 0.05 * (aDeg + 1 - i)) : aRegular;
                aPrev = aFrac;
            }
            double aOff = (i == 1 || i == aDeg + 1) ? 0.0 : theNoise * theO.Real(-1, 1);
            aPoles[i] = new gp_Pnt2d(theP1.X() + aFrac * aDX + aOff * aNX, theP1.Y() + aFrac * aDY + aOff * aNY);
        }
        NCollection_Array1<double> aKnots = NCollection_Array1<double>.From([theT1, theT2]);
        NCollection_Array1<int> aMults = NCollection_Array1<int>.From([aDeg + 1, aDeg + 1]);
        return (new Geom2d_BSplineCurve(aPoles, aKnots, aMults, aDeg),
                Oracle.ToNativeCurve(aPoles, null, aKnots, aMults, aDeg, false));
    }

    // The native copy of a port B-spline (its own arrays).
    private static N.Geom2d_BSplineCurve ToNative(Geom2d_BSplineCurve theC)
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new(1, theC.NbPoles());
        theC.Poles(aPoles);
        NCollection_Array1<double> aKnots = new(1, theC.NbKnots());
        theC.Knots(aKnots);
        NCollection_Array1<int> aMults = new(1, theC.NbKnots());
        theC.Multiplicities(aMults);
        NCollection_Array1<double>? aWeights = null;
        if (theC.IsRational())
        {
            aWeights = new NCollection_Array1<double>(1, theC.NbPoles());
            theC.Weights(aWeights);
        }
        return Oracle.ToNativeCurve(aPoles, aWeights, aKnots, aMults, theC.Degree(), theC.IsPeriodic());
    }

    private static N.Geom_BSplineCurve ToNative(Geom_BSplineCurve theC)
    {
        NCollection_Array1<gp_Pnt> aPoles = new(1, theC.NbPoles());
        theC.Poles(aPoles);
        NCollection_Array1<double> aKnots = new(1, theC.NbKnots());
        theC.Knots(aKnots);
        NCollection_Array1<int> aMults = new(1, theC.NbKnots());
        theC.Multiplicities(aMults);
        NCollection_Array1<double>? aWeights = null;
        if (theC.IsRational())
        {
            aWeights = new NCollection_Array1<double>(1, theC.NbPoles());
            theC.Weights(aWeights);
        }
        return Oracle.ToNativeCurve(aPoles, aWeights, aKnots, aMults, theC.Degree(), theC.IsPeriodic());
    }

    private static double Noise(Oracle theO, bool theSame) =>
        theSame ? 0.0 : (theO.Int(0, 2) == 0 ? 0.0 : Math.Pow(10.0, theO.Real(-7, -3)));

    private static EdgeCase PlaneCase(Oracle theO, bool theSame)
    {
        double ox = theO.Real(-5, 5), oy = theO.Real(-5, 5), oz = theO.Real(-5, 5);
        Geom_Plane aPS = new(new gp_Ax3(new gp_Pnt(ox, oy, oz), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)));
        using var aNO = new N.gp_Pnt(ox, oy, oz);
        using var aNZ = new N.gp_Dir(0, 0, 1);
        using var aNX = new N.gp_Dir(1, 0, 0);
        using var aNAx = new N.gp_Ax3(aNO, aNZ, aNX);
        N.Geom_Plane aNS = new(aNAx);

        switch (theO.Int(0, 2))
        {
            case 0:
            {
                // A circle by angle; the pcurve is the rational B-spline of the same circle (another speed)
                double cu = theO.Real(-3, 3), cv = theO.Real(-3, 3), r = theO.Real(0.5, 4);
                double a = theO.Real(0, 3), b = a + theO.Real(0.3, 3);
                Geom_Circle aPC3 = new(new gp_Ax2(new gp_Pnt(cu + ox, cv + oy, oz), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), r);
                using var aNC = new N.gp_Pnt(cu + ox, cv + oy, oz);
                using var aNAx2 = new N.gp_Ax2(aNC, aNZ, aNX);
                N.Geom_Circle aNC3 = new(aNAx2, r);
                Geom2d_Circle aCircle2d = new(new gp_Ax2d(new gp_Pnt2d(cu, cv), new gp_Dir2d(1, 0)), r);
                Geom2d_BSplineCurve aPC = Geom2dConvert.CurveToBSplineCurve(new Geom2d_TrimmedCurve(aCircle2d, a, b),
                                                                            theO.Bool() ? Convert_RationalC1 : Convert_QuasiAngular);
                if (!theSame)
                {
                    NCollection_Array1<double> aKnots = new(aPC.InternalKnots());
                    BSplCLib.Reparametrize(a, b, aKnots);
                    aPC.SetKnots(aKnots);
                }
                else
                {
                    // The angular parametrisation itself
                    return new EdgeCase { PortS = aPS, NativeS = aNS, PortC3d = aPC3, NativeC3d = aNC3,
                        PortPC = aCircle2d, NativePC = new N.Geom2d_Circle(Oracle.ToNative(aCircle2d.Position()), r),
                        First = a, Last = b, Kind = "plane circle (same)" };
                }
                return new EdgeCase { PortS = aPS, NativeS = aNS, PortC3d = aPC3, NativeC3d = aNC3, PortPC = aPC,
                    NativePC = ToNative(aPC), First = a, Last = b, Kind = "plane circle" };
            }
            case 1:
            {
                // A line by length; the pcurve runs along it at a varying speed
                gp_Pnt2d aP1 = new(theO.Real(-5, 5), theO.Real(-5, 5));
                double ang = theO.Angle(), len = theO.Real(1, 8);
                double dx = Math.Cos(ang), dy = Math.Sin(ang);
                Geom_Line aPC3 = new(new gp_Pnt(aP1.X() + ox, aP1.Y() + oy, oz), new gp_Dir(dx, dy, 0));
                using var aNP = new N.gp_Pnt(aP1.X() + ox, aP1.Y() + oy, oz);
                using var aND = new N.gp_Dir(dx, dy, 0);
                N.Geom_Line aNC3 = new(aNP, aND);
                double a = theO.Real(-2, 2), b = a + len;
                gp_Dir2d aD2 = new(dx, dy);
                gp_Pnt2d aS = new(aP1.X() + a * aD2.X(), aP1.Y() + a * aD2.Y());
                gp_Pnt2d aE = new(aP1.X() + b * aD2.X(), aP1.Y() + b * aD2.Y());
                (Geom2d_BSplineCurve aPC, N.Geom2d_BSplineCurve aNPC) =
                    WarpedSegment(theO, aS, aE, a, b, !theSame, Noise(theO, theSame));
                return new EdgeCase { PortS = aPS, NativeS = aNS, PortC3d = aPC3, NativeC3d = aNC3, PortPC = aPC,
                    NativePC = aNPC, First = a, Last = b, Kind = "plane line" };
            }
            default:
            {
                // The image of a smooth 2D B-spline; the pcurve has its poles perturbed
                int aDeg = theO.Int(2, 4);
                int aNbKnots = theO.Int(2, 4);
                NCollection_Array1<double> aKnots = new(1, aNbKnots);
                NCollection_Array1<int> aMults = new(1, aNbKnots);
                double k = theO.Real(-2, 2);
                for (int i = 1; i <= aNbKnots; i++)
                {
                    aKnots[i] = k;
                    k += theO.Real(0.5, 2);
                    aMults[i] = (i == 1 || i == aNbKnots) ? aDeg + 1 : 1;
                }
                int aNbPoles = aDeg + aNbKnots - 1;
                NCollection_Array1<gp_Pnt2d> aPoles = new(1, aNbPoles);
                NCollection_Array1<gp_Pnt2d> aPCPoles = new(1, aNbPoles);
                NCollection_Array1<gp_Pnt> a3dPoles = new(1, aNbPoles);
                double aNoise = theSame ? 0.0 : Math.Pow(10.0, theO.Real(-6, -2));
                double x = theO.Real(-5, 0), y = theO.Real(-5, 5);
                for (int i = 1; i <= aNbPoles; i++)
                {
                    x += theO.Real(0.5, 2);
                    y += theO.Real(-1.5, 1.5);
                    aPoles[i] = new gp_Pnt2d(x, y);
                    a3dPoles[i] = new gp_Pnt(x + ox, y + oy, oz);
                    aPCPoles[i] = new gp_Pnt2d(x + aNoise * theO.Real(-1, 1), y + aNoise * theO.Real(-1, 1));
                }
                Geom_BSplineCurve aPC3 = new(a3dPoles, aKnots, aMults, aDeg);
                Geom2d_BSplineCurve aPC = new(aPCPoles, aKnots, aMults, aDeg);
                return new EdgeCase { PortS = aPS, NativeS = aNS, PortC3d = aPC3, NativeC3d = ToNative(aPC3), PortPC = aPC,
                    NativePC = ToNative(aPC), First = aKnots[1], Last = aKnots[aNbKnots], Kind = "plane bspline" };
            }
        }
    }

    private static EdgeCase CylinderCase(Oracle theO, bool theSame)
    {
        double r = theO.Real(0.5, 5);
        Geom_CylindricalSurface aPS = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), r);
        using var aNO = new N.gp_Pnt(0, 0, 0);
        using var aNZ = new N.gp_Dir(0, 0, 1);
        using var aNX = new N.gp_Dir(1, 0, 0);
        using var aNAx = new N.gp_Ax3(aNO, aNZ, aNX);
        N.Geom_CylindricalSurface aNS = new(aNAx, r);

        if (theO.Bool())
        {
            // A horizontal circle by angle; the pcurve is V = h with U running at a varying speed
            double h = theO.Real(-5, 5);
            double a = theO.Real(0, 3), b = a + theO.Real(0.3, 3);
            Geom_Circle aPC3 = new(new gp_Ax2(new gp_Pnt(0, 0, h), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), r);
            using var aNC = new N.gp_Pnt(0, 0, h);
            using var aNAx2 = new N.gp_Ax2(aNC, aNZ, aNX);
            N.Geom_Circle aNC3 = new(aNAx2, r);
            (Geom2d_BSplineCurve aPC, N.Geom2d_BSplineCurve aNPC) =
                WarpedSegment(theO, new gp_Pnt2d(a, h), new gp_Pnt2d(b, h), a, b, !theSame, Noise(theO, theSame));
            return new EdgeCase { PortS = aPS, NativeS = aNS, PortC3d = aPC3, NativeC3d = aNC3, PortPC = aPC,
                NativePC = aNPC, First = a, Last = b, Kind = "cylinder circle" };
        }
        else
        {
            // A generator by height; the pcurve is U = u0 with V running at a varying speed
            double u0 = theO.Real(0, 6);
            double cx = r * Math.Cos(u0), cy = r * Math.Sin(u0);
            Geom_Line aPC3 = new(new gp_Pnt(cx, cy, 0), new gp_Dir(0, 0, 1));
            using var aNP = new N.gp_Pnt(cx, cy, 0);
            N.Geom_Line aNC3 = new(aNP, aNZ);
            double a = theO.Real(-5, 5), b = a + theO.Real(0.5, 6);
            (Geom2d_BSplineCurve aPC, N.Geom2d_BSplineCurve aNPC) =
                WarpedSegment(theO, new gp_Pnt2d(u0, a), new gp_Pnt2d(u0, b), a, b, !theSame, Noise(theO, theSame));
            return new EdgeCase { PortS = aPS, NativeS = aNS, PortC3d = aPC3, NativeC3d = aNC3, PortPC = aPC,
                NativePC = aNPC, First = a, Last = b, Kind = "cylinder line" };
        }
    }

    private static EdgeCase BSplineSurfaceCase(Oracle theO, bool theSame)
    {
        (N.Geom_BSplineSurface aNS, Geom_BSplineSurface aPS) =
            AdaptorOracleInputs.RandomBSplineSurface(theO, 1, false, 2);
        aPS.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        bool anIsU = theO.Bool();
        double aFix = anIsU ? theO.Real(aU1, aU2) : theO.Real(aV1, aV2);
        double aLo = anIsU ? aV1 : aU1, aHi = anIsU ? aV2 : aU2;
        double a = theO.Real(aLo, aLo + 0.3 * (aHi - aLo)), b = theO.Real(aLo + 0.7 * (aHi - aLo), aHi);
        Geom_BSplineCurve anIso = (Geom_BSplineCurve)(anIsU ? aPS.UIso(aFix) : aPS.VIso(aFix));
        gp_Pnt2d aS = anIsU ? new gp_Pnt2d(aFix, a) : new gp_Pnt2d(a, aFix);
        gp_Pnt2d aE = anIsU ? new gp_Pnt2d(aFix, b) : new gp_Pnt2d(b, aFix);
        (Geom2d_BSplineCurve aPC, N.Geom2d_BSplineCurve aNPC) =
            WarpedSegment(theO, aS, aE, a, b, !theSame, Noise(theO, theSame) * 0.1);
        return new EdgeCase { PortS = aPS, NativeS = aNS, PortC3d = anIso, NativeC3d = ToNative(anIso), PortPC = aPC,
            NativePC = aNPC, First = a, Last = b, Kind = anIsU ? "bspline UIso" : "bspline VIso" };
    }

    private static void SameCurve2d(Oracle theO, N.Geom2d_Curve theN, Geom2d_Curve theP, double theF, double theL,
                                    string theWhat)
    {
        if (theP is Geom2d_BSplineCurve aPB)
        {
            using N.Geom2d_BSplineCurve? aNB = N.Geom2d_BSplineCurve.DownCast(theN);
            Assert.True(aNB is not null, $"{theWhat}: native pcurve is not a B-spline");
            Geom2dOracle.SameStructure(theO, aNB!, aPB, theWhat);
        }
        for (int i = 0; i <= 10; i++)
        {
            double t = theF + (theL - theF) * i / 10.0;
            using N.gp_Pnt2d aNP = theN.Value(t);
            theO.Same(aNP, theP.Value(t), $"{theWhat} Value({t})");
        }
    }

    /// <summary>The cases described in the class summary: pcurve and new tolerance compared.</summary>
    [Fact]
    public void SameParameter_through_BRepLib_SameParameter()
    {
        var anOracle = new Oracle(output, 20261105);
        int aCompared = 0, aReparam = 0, aAlready = 0, aFailed = 0;
        for (int c = 0; c < 600; c++)
        {
            bool aSame = anOracle.Int(0, 9) == 0;
            EdgeCase aCase = (c % 3) switch
            {
                0 => PlaneCase(anOracle, aSame),
                1 => CylinderCase(anOracle, aSame),
                _ => BSplineSurfaceCase(anOracle, aSame),
            };
            double aTol = anOracle.Int(0, 2) switch { 0 => 1e-7, 1 => 1e-5, _ => 1e-3 };
            string aWhat = $"case {c} ({aCase.Kind}, tol {aTol})";

            // Native: the edge, then BRepLib::SameParameter
            using var aNS = aCase.NativeS;
            using var aNC3d = aCase.NativeC3d;
            using var aNPC = aCase.NativePC;
            using var aB = new N.BRep_Builder();
            using var aNE = new N.TopoDS_Edge();
            using var aLoc = new N.TopLoc_Location();
            aB.MakeEdge(aNE, aNC3d, aTol);
            aB.UpdateEdge(aNE, aNPC, aNS, aLoc, aTol);
            aB.Range(aNE, aCase.First, aCase.Last);
            aB.SameRange(aNE, true);
            aB.SameParameter(aNE, false);
            double aNewTol = -1.0;
            using N.TopoDS_Edge aNRes = N.BRepLib.SameParameter(aNE, aTol, ref aNewTol, false);
            double aNF = 0, aNL = 0;
            using N.Geom2d_Curve aNResPC = N.BRep_Tool.CurveOnSurface(aNRes, aNS, aLoc, ref aNF, ref aNL)!;

            // Port: BRepLib's path for this edge
            GeomAdaptor_Curve HC = new(aCase.PortC3d, aCase.First, aCase.Last);
            Geom2dAdaptor_Curve HC2d = new(aCase.PortPC, aCase.First, aCase.Last);
            GeomAdaptor_Surface HS = new(aCase.PortS);
            double error = ComputeTol(HC, HC2d, HS, 22);
            Assert.True(error <= 1.0e10, $"{aWhat}: BigError");
            Approx_SameParameter SameP = new(HC, HC2d, HS, aTol);
            Geom2d_Curve aPortPC = aCase.PortPC;
            double maxdist = 0.0;
            bool aTolKnown = true;
            if (SameP.IsSameParameter())
            {
                maxdist = Max(maxdist, SameP.TolReached());
                aAlready++;
            }
            else if (SameP.IsDone())
            {
                double tolreached = SameP.TolReached();
                if (tolreached <= error)
                {
                    aPortPC = SameP.Curve2d()!;
                    maxdist = Max(maxdist, tolreached);
                }
                else
                {
                    maxdist = Max(maxdist, error);
                }
                aReparam++;
            }
            else
            {
                // BRepLib then keeps the pcurve (through GeomLib::SameRange on an equal range) and decides the
                // tolerance with BRepCheck's precisions: not mirrored.
                aTolKnown = false;
                aFailed++;
            }

            SameCurve2d(anOracle, aNResPC, aPortPC, aCase.First, aCase.Last, $"{aWhat} pcurve");
            anOracle.Same(aNF, aCase.First, $"{aWhat} first");
            anOracle.Same(aNL, aCase.Last, $"{aWhat} last");
            if (aTolKnown)
            {
                anOracle.Same(aNewTol, Max(maxdist, Precision.Confusion()), $"{aWhat} new tolerance");
                anOracle.Same(N.BRep_Tool.Tolerance(aNRes), Max(maxdist, Precision.Confusion()), $"{aWhat} edge tolerance");
            }
            aCompared++;
        }
        output.WriteLine($"{aCompared} edges: {aAlready} already same parameter, {aReparam} reparametrised, {aFailed} failed");
        Assert.True(aReparam > 300, "enough reparametrised edges");
        anOracle.Report("Approx_SameParameter through BRepLib::SameParameter");
    }
}
