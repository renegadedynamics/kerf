// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Kernel.Oracle.Tests.AdaptorOracleInputs;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// GeomAdaptor_Surface against native OCCT, on every kind of surface it knows (elementary, revolution, extrusion,
/// Bezier, B-spline, offset and trimmed surfaces), over the whole surface or random bounds, with and without
/// snapping tolerances: parameters, continuity, the intervals of every continuity, resolutions, type and the
/// type's data, degrees, poles, knots, rationality, closure and periodicity, and evaluation (D0 to D3 and DN) in a
/// random sequence of parameters that crosses spans, lands on knots, on the bounds and within the tolerances of
/// the bounds (the cache, and the B-spline's local evaluation on the bounds); also through UTrim, VTrim,
/// ShallowCopy, BasisCurve and BasisSurface.
/// </summary>
public class GeomAdaptor_SurfaceOracleTests(ITestOutputHelper output)
{
    private static readonly int Cases = Tape.Cases(40, 4);

    private static readonly Continuity[] Shapes =
        [Continuity.C0, Continuity.G1, Continuity.C1, Continuity.G2,
         Continuity.C2, Continuity.C3, Continuity.CN];

    private static void SameIntervals(Oracle theO, N.GeomAdaptor_Surface theN, SurfaceAdaptor theP, bool theU,
                                      string theWhat)
    {
        string aDir = theU ? "U" : "V";
        int aNC = 0, aPC = 0;
        if (SameOutcome(Catch(() => aNC = (int)(theU ? theN.UContinuity() : theN.VContinuity())),
                        Catch(() => aPC = (int)(theU ? theP.UContinuity() : theP.VContinuity())), $"{theWhat} {aDir}Continuity"))
        {
            theO.Same(aNC, aPC, $"{theWhat} {aDir}Continuity");
        }
        foreach (Continuity aS in Shapes)
        {
            var aNS = (N.GeomAbs_Shape)(int)aS;
            int aNNb = 0, aPNb = 0;
            if (!SameOutcome(Catch(() => aNNb = theU ? theN.NbUIntervals(aNS) : theN.NbVIntervals(aNS)),
                             Catch(() => aPNb = theU ? theP.NbUIntervals(aS) : theP.NbVIntervals(aS)), $"{theWhat} Nb{aDir}Intervals({aS})"))
            {
                continue;
            }
            theO.Same(aNNb, aPNb, $"{theWhat} Nb{aDir}Intervals({aS})");
            using var aNT = new N.TColStd_Array1OfReal(1, aNNb + 1);
            var aPT = new NCollection_Array1<double>(1, aPNb + 1);
            if (SameOutcome(Catch(() =>
                            {
                                if (theU)
                                {
                                    theN.UIntervals(aNT, aNS);
                                }
                                else
                                {
                                    theN.VIntervals(aNT, aNS);
                                }
                            }),
                            Catch(() =>
                            {
                                if (theU)
                                {
                                    theP.UIntervals(aPT, aS);
                                }
                                else
                                {
                                    theP.VIntervals(aPT, aS);
                                }
                            }), $"{theWhat} {aDir}Intervals({aS})"))
            {
                for (int i = 1; i <= aPNb + 1; i++)
                {
                    theO.Same(aNT.Value(i), aPT[i], $"{theWhat} {aDir}Intervals({aS})[{i}]");
                }
            }
        }
    }

    private static void SameQueries(Oracle theO, N.GeomAdaptor_Surface theN, SurfaceAdaptor theP, string theWhat)
    {
        theO.Same(theN.FirstUParameter(), theP.FirstUParameter(), theWhat + " FirstUParameter");
        theO.Same(theN.LastUParameter(), theP.LastUParameter(), theWhat + " LastUParameter");
        theO.Same(theN.FirstVParameter(), theP.FirstVParameter(), theWhat + " FirstVParameter");
        theO.Same(theN.LastVParameter(), theP.LastVParameter(), theWhat + " LastVParameter");
        theO.Same(theN.ToleranceU(), theP.ToleranceU(), theWhat + " ToleranceU");
        theO.Same(theN.ToleranceV(), theP.ToleranceV(), theWhat + " ToleranceV");
        theO.Same((int)theN.GetType_(), (int)theP.GetType(), theWhat + " GetType");
        theO.Same(theN.IsUClosed(), theP.IsUClosed, theWhat + " IsUClosed");
        theO.Same(theN.IsVClosed(), theP.IsVClosed, theWhat + " IsVClosed");
        theO.Same(theN.IsUPeriodic(), theP.IsUPeriodic, theWhat + " IsUPeriodic");
        theO.Same(theN.IsVPeriodic(), theP.IsVPeriodic, theWhat + " IsVPeriodic");
        if (theP.IsUPeriodic)
        {
            theO.Same(theN.UPeriod(), theP.UPeriod(), theWhat + " UPeriod");
        }
        if (theP.IsVPeriodic)
        {
            theO.Same(theN.VPeriod(), theP.VPeriod(), theWhat + " VPeriod");
        }
        theO.Same(theN.IsURational(), theP.IsURational, theWhat + " IsURational");
        theO.Same(theN.IsVRational(), theP.IsVRational, theWhat + " IsVRational");
        foreach (double aR3d in new[] { 1e-7, 1e-3, 0.5, 30.0 })
        {
            double aNR = 0, aPR = 0;
            if (SameOutcome(Catch(() => aNR = theN.UResolution(aR3d)), Catch(() => aPR = theP.UResolution(aR3d)), theWhat + " UResolution"))
            {
                theO.SameRelative(aNR, aPR, theWhat + " UResolution");
            }
            if (SameOutcome(Catch(() => aNR = theN.VResolution(aR3d)), Catch(() => aPR = theP.VResolution(aR3d)), theWhat + " VResolution"))
            {
                theO.SameRelative(aNR, aPR, theWhat + " VResolution");
            }
        }

        // degrees, poles and knots: throwing on both sides or neither
        (Func<int> N, Func<int> P, string What)[] aCounts =
        [
            (theN.UDegree, theP.UDegree, "UDegree"), (theN.VDegree, theP.VDegree, "VDegree"),
            (theN.NbUPoles, () => theP.UPoleCount, "NbUPoles"), (theN.NbVPoles, () => theP.VPoleCount, "NbVPoles"),
            (theN.NbUKnots, () => theP.UKnotCount, "NbUKnots"), (theN.NbVKnots, () => theP.VKnotCount, "NbVKnots"),
        ];
        foreach ((Func<int> aNF, Func<int> aPF, string aCountWhat) in aCounts)
        {
            int aNV = 0, aPV = 0;
            if (SameOutcome(Catch(() => aNV = aNF()), Catch(() => aPV = aPF()), $"{theWhat} {aCountWhat}"))
            {
                theO.Same(aNV, aPV, $"{theWhat} {aCountWhat}");
            }
        }

        switch (theP.GetType())
        {
            case SurfaceKind.Plane:
            {
                using N.gp_Pln aN = theN.Plane();
                using N.gp_Ax3 aNA = aN.Position();
                SameAx3(theO, aNA, theP.Plane().Position(), theWhat + " Plane");
                break;
            }
            case SurfaceKind.Cylinder:
            {
                using N.gp_Cylinder aN = theN.Cylinder();
                using N.gp_Ax3 aNA = aN.Position();
                SameAx3(theO, aNA, theP.Cylinder().Position(), theWhat + " Cylinder");
                theO.Same(aN.Radius(), theP.Cylinder().Radius(), theWhat + " Cylinder radius");
                break;
            }
            case SurfaceKind.Cone:
            {
                using N.gp_Cone aN = theN.Cone();
                using N.gp_Ax3 aNA = aN.Position();
                SameAx3(theO, aNA, theP.Cone().Position(), theWhat + " Cone");
                theO.Same(aN.RefRadius(), theP.Cone().RefRadius(), theWhat + " Cone radius");
                theO.Same(aN.SemiAngle(), theP.Cone().SemiAngle(), theWhat + " Cone angle");
                break;
            }
            case SurfaceKind.Sphere:
            {
                using N.gp_Sphere aN = theN.Sphere();
                using N.gp_Ax3 aNA = aN.Position();
                SameAx3(theO, aNA, theP.Sphere().Position(), theWhat + " Sphere");
                theO.Same(aN.Radius(), theP.Sphere().Radius(), theWhat + " Sphere radius");
                break;
            }
            case SurfaceKind.Torus:
            {
                using N.gp_Torus aN = theN.Torus();
                using N.gp_Ax3 aNA = aN.Position();
                SameAx3(theO, aNA, theP.Torus().Position(), theWhat + " Torus");
                theO.Same(aN.MajorRadius(), theP.Torus().MajorRadius(), theWhat + " Torus major");
                theO.Same(aN.MinorRadius(), theP.Torus().MinorRadius(), theWhat + " Torus minor");
                break;
            }
            case SurfaceKind.BezierSurface:
            {
                using N.Geom_BezierSurface aN = theN.Bezier()!;
                BezierSurface aP = theP.Bezier();
                theO.Same(aN.NbUPoles(), aP.UPoleCount, theWhat + " Bezier() NbUPoles");
                SameSurfaceEval(theO, aN, aP, 0.3, 0.6, 2, theWhat + " Bezier()");
                break;
            }
            case SurfaceKind.BSplineSurface:
            {
                using N.Geom_BSplineSurface aN = theN.BSpline()!;
                BSplineSurface aP = theP.BSpline();
                theO.Same(aN.NbUPoles(), aP.UPoleCount, theWhat + " BSpline() NbUPoles");
                SameSurfaceEval(theO, aN, aP, aP.UKnot(1), aP.VKnot(aP.VKnotCount), 2, theWhat + " BSpline()");
                break;
            }
            case SurfaceKind.SurfaceOfRevolution:
            {
                using N.gp_Ax1 aNA = theN.AxeOfRevolution();
                SameAx1(theO, aNA, theP.AxeOfRevolution(), theWhat + " AxeOfRevolution");
                using N.Adaptor3d_Curve aNC = theN.BasisCurve()!;
                CurveAdaptorBase aPC = theP.BasisCurve();
                SameCurveEval(theO, aNC, aPC, Param(theO, aPC.FirstParameter(), aPC.LastParameter()), 2, theWhat + " BasisCurve");
                break;
            }
            case SurfaceKind.SurfaceOfExtrusion:
            {
                using N.gp_Dir aND = theN.Direction();
                SameDir(theO, aND, theP.Direction(), theWhat + " Direction");
                using N.Adaptor3d_Curve aNC = theN.BasisCurve()!;
                CurveAdaptorBase aPC = theP.BasisCurve();
                SameCurveEval(theO, aNC, aPC, Param(theO, aPC.FirstParameter(), aPC.LastParameter()), 2, theWhat + " BasisCurve");
                break;
            }
            case SurfaceKind.OffsetSurface:
            {
                theO.Same(theN.OffsetValue(), theP.OffsetValue(), theWhat + " OffsetValue");
                using N.Adaptor3d_Surface aNS = theN.BasisSurface()!;
                SurfaceAdaptorBase aPS = theP.BasisSurface();
                double U = Param(theO, aPS.FirstUParameter(), aPS.LastUParameter());
                double V = Param(theO, aPS.FirstVParameter(), aPS.LastVParameter());
                SameSurfaceEval(theO, aNS, aPS, U, V, 2, theWhat + " BasisSurface");
                break;
            }
        }
    }

    /// <summary>
    /// A sequence of (U, V): random ones, the bounds and corners exactly, points within the snapping tolerances of
    /// the bounds, the knots of a B-spline surface; shuffled.
    /// </summary>
    private static List<(double U, double V)> Params(Oracle theO, SurfaceAdaptor theP)
    {
        double aU1 = theP.FirstUParameter(), aU2 = theP.LastUParameter();
        double aV1 = theP.FirstVParameter(), aV2 = theP.LastVParameter();
        var aUs = new List<double>();
        var aVs = new List<double>();
        for (int k = 0; k < 4; k++)
        {
            aUs.Add(Param(theO, aU1, aU2));
            aVs.Add(Param(theO, aV1, aV2));
        }
        foreach ((List<double> aList, double aF, double aL, double aTol) in
                 new[] { (aUs, aU1, aU2, theP.ToleranceU()), (aVs, aV1, aV2, theP.ToleranceV()) })
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(aF))
            {
                aList.Add(aF);
                aList.Add(aF + 0.5 * aTol);
            }
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(aL))
            {
                aList.Add(aL);
                aList.Add(aL - 0.5 * aTol);
            }
        }
        if (theP.GetType() == SurfaceKind.BSplineSurface)
        {
            BSplineSurface aBS = theP.BSpline();
            for (int i = 1; i <= aBS.UKnotCount; i++)
            {
                if (aBS.UKnot(i) >= aU1 && aBS.UKnot(i) <= aU2)
                {
                    aUs.Add(aBS.UKnot(i));
                }
            }
            for (int i = 1; i <= aBS.VKnotCount; i++)
            {
                if (aBS.VKnot(i) >= aV1 && aBS.VKnot(i) <= aV2)
                {
                    aVs.Add(aBS.VKnot(i));
                }
            }
        }
        var aParams = new List<(double, double)>();
        int aNb = Math.Max(aUs.Count, aVs.Count) + 6;
        for (int k = 0; k < aNb; k++)
        {
            aParams.Add((aUs[theO.Int(0, aUs.Count - 1)], aVs[theO.Int(0, aVs.Count - 1)]));
        }
        return aParams;
    }

    private static void CompareAdaptors(Oracle theO, N.GeomAdaptor_Surface theN, SurfaceAdaptor theP, string theWhat)
    {
        SameQueries(theO, theN, theP, theWhat);
        SameIntervals(theO, theN, theP, true, theWhat);
        SameIntervals(theO, theN, theP, false, theWhat);
        foreach ((double U, double V) in Params(theO, theP))
        {
            SameSurfaceEval(theO, theN, theP, U, V, 3, theWhat);
        }

        double aU1 = theP.FirstUParameter(), aU2 = theP.LastUParameter();
        double aV1 = theP.FirstVParameter(), aV2 = theP.LastVParameter();
        double aT1 = Param(theO, aU1, aU2), aT2 = Param(theO, aU1, aU2);
        if (aT1 > aT2)
        {
            (aT1, aT2) = (aT2, aT1);
        }
        using (N.Adaptor3d_Surface aNT = theN.UTrim(aT1, aT2, 1e-6)!)
        {
            SurfaceAdaptorBase aPT = theP.UTrim(aT1, aT2, 1e-6);
            theO.Same(aNT.FirstUParameter(), aPT.FirstUParameter(), theWhat + " UTrim first");
            SameSurfaceEval(theO, aNT, aPT, aT1, Param(theO, aV1, aV2), 2, theWhat + " UTrim");
        }
        aT1 = Param(theO, aV1, aV2);
        aT2 = Param(theO, aV1, aV2);
        if (aT1 > aT2)
        {
            (aT1, aT2) = (aT2, aT1);
        }
        using (N.Adaptor3d_Surface aNT = theN.VTrim(aT1, aT2, 1e-6)!)
        {
            SurfaceAdaptorBase aPT = theP.VTrim(aT1, aT2, 1e-6);
            theO.Same(aNT.LastVParameter(), aPT.LastVParameter(), theWhat + " VTrim last");
            SameSurfaceEval(theO, aNT, aPT, Param(theO, aU1, aU2), aT2, 2, theWhat + " VTrim");
        }
        using (N.Adaptor3d_Surface aNS = theN.ShallowCopy()!)
        {
            SurfaceAdaptorBase aPS = theP.ShallowCopy();
            SameSurfaceEval(theO, aNS, aPS, Param(theO, aU1, aU2), Param(theO, aV1, aV2), 2, theWhat + " ShallowCopy");
        }
    }

    [Fact]
    public void Surfaces()
    {
        for (int aKind = 0; aKind < SurfaceKinds.Length; aKind++)
        {
            var anOracle = new Oracle(output, 20261111 + aKind);
            for (int i = 0; i < Cases; i++)
            {
                (N.Geom_Surface aNS, Surface aPS) = RandomSurface(anOracle, aKind);
                using (aNS)
                {
                    string aWhat = SurfaceKinds[aKind];
                    using var aNA = new N.GeomAdaptor_Surface(aNS);
                    var aPA = new SurfaceAdaptor(aPS);
                    CompareAdaptors(anOracle, aNA, aPA, aWhat);

                    // random bounds within the surface's, with and without snapping tolerances
                    aPS.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                    (double aUF, double aUL) = Bounds(anOracle, aU1, aU2);
                    (double aVF, double aVL) = Bounds(anOracle, aV1, aV2);
                    double aTolU = anOracle.Bool() ? 0.0 : anOracle.Real(1e-9, 1e-4);
                    double aTolV = anOracle.Bool() ? 0.0 : anOracle.Real(1e-9, 1e-4);
                    using var aNB = new N.GeomAdaptor_Surface(aNS, aUF, aUL, aVF, aVL, aTolU, aTolV);
                    var aPB = new SurfaceAdaptor(aPS, aUF, aUL, aVF, aVL, aTolU, aTolV);
                    CompareAdaptors(anOracle, aNB, aPB, aWhat + " bounded");

                    // Load on an existing adaptor: the same surface with other bounds
                    aNA.Load(aNS, aUF, aUL, aVF, aVL, aTolU, aTolV);
                    aPA.Load(aPS, aUF, aUL, aVF, aVL, aTolU, aTolV);
                    CompareAdaptors(anOracle, aNA, aPA, aWhat + " reloaded");
                }
            }
            anOracle.Report("GeomAdaptor_Surface " + SurfaceKinds[aKind]);
        }
    }

    private static (double, double) Bounds(Oracle theO, double theFirst, double theLast)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(theFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(theLast))
        {
            double aF = theO.Real(-10, 0);
            return (aF, aF + theO.Real(0.5, 10));
        }
        double aD = theLast - theFirst;
        return (theO.Real(theFirst, theFirst + 0.45 * aD), theO.Real(theFirst + 0.55 * aD, theLast));
    }
}
