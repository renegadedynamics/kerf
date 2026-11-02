// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using static OcctNet.Oracle.Tests.AdaptorOracleInputs;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Compares a trihedron law of the port with native OCCT's (any law the bindings give through
/// <c>GeomFill_TrihedronLaw</c>) on random spines: every kind of <see cref="AdaptorOracleInputs.RandomCurve"/>
/// (B-splines with C0 and C1 knots included) through GeomAdaptor_Curve, bounded where the curve is infinite.
/// </summary>
/// <remarks>
/// Compared, after SetCurve (and its result): D0, D1 and D2 (the result and every vector, or throwing on both sides)
/// at random parameters, at the curve's interval bounds of every continuity (its knots), at the law's C0 interval
/// bounds (Frenet's singular points) and 1e-7 beside them, and at the ends;
/// NbIntervals and Intervals for C0 to CN; GetAverageLaw, IsConstant, IsOnlyBy3dCurve, ErrorStatus; then
/// SetInterval on a random sub-range with GetInterval and D0 to D2 there (ends included), GetAverageLaw again; and
/// Copy() of the law evaluated on the whole curve.
/// </remarks>
internal static class TrihedronOracle
{
    private static readonly GeomAbs_Shape[] Shapes =
        [GeomAbs_Shape.GeomAbs_C0, GeomAbs_Shape.GeomAbs_C1, GeomAbs_Shape.GeomAbs_C2, GeomAbs_Shape.GeomAbs_C3,
         GeomAbs_Shape.GeomAbs_CN];

    /// <summary>Runs <paramref name="theCases"/> random spines of every curve kind through both laws.</summary>
    internal static void Run(ITestOutputHelper theOutput, string theName, Func<N.GeomFill_TrihedronLaw> theNative,
                             Func<GeomFill_TrihedronLaw> thePort, int theCases, int theSeed, int[]? theSkippedKinds = null)
    {
        for (int aKind = 0; aKind < CurveKinds.Length; aKind++)
        {
            if (theSkippedKinds is not null && Array.IndexOf(theSkippedKinds, aKind) >= 0)
            {
                continue;
            }
            var anOracle = new Oracle(theOutput, theSeed + aKind);
            for (int i = 0; i < theCases; i++)
            {
                (N.Geom_Curve aNC, Geom_Curve aPC, double aRange) = RandomCurve(anOracle, aKind);
                using (aNC)
                {
                    double aF = aPC.FirstParameter(), aL = aPC.LastParameter();
                    if (Precision.IsInfinite(aF) || Precision.IsInfinite(aL))
                    {
                        aF = anOracle.Real(-aRange, 0);
                        aL = aF + anOracle.Real(0.1, 2 * aRange);
                    }
                    using var aNA = new N.GeomAdaptor_Curve(aNC, aF, aL);
                    var aPA = new GeomAdaptor_Curve(aPC, aF, aL);
                    using N.GeomFill_TrihedronLaw aNLaw = theNative();
                    GeomFill_TrihedronLaw aPLaw = thePort();
                    Compare(anOracle, aNLaw, aPLaw, aNA, aPA, $"{theName} on {CurveKinds[aKind]} #{i}");
                }
            }
            anOracle.Report($"{theName} on {CurveKinds[aKind]}");
        }
    }

    internal static void Compare(Oracle theO, N.GeomFill_TrihedronLaw theN, GeomFill_TrihedronLaw theP,
                                 N.GeomAdaptor_Curve theNC, GeomAdaptor_Curve thePC, string theWhat)
    {
        bool aNOk = false, aPOk = false;
        if (!SameOutcome(Catch(() => aNOk = theN.SetCurve(theNC)), Catch(() => aPOk = theP.SetCurve(thePC)),
                         theWhat + " SetCurve"))
        {
            return;
        }
        theO.Same(aNOk, aPOk, theWhat + " SetCurve");

        double aF = thePC.FirstParameter(), aL = thePC.LastParameter();
        List<double> aParams = Params(theO, thePC, aF, aL);
        // the law's own interval bounds (Frenet's singular points among them), on and just beside them
        int aNbLaw = 0;
        if (Catch(() => aNbLaw = theP.NbIntervals(GeomAbs_Shape.GeomAbs_C0)) is null && aNbLaw > 1)
        {
            var aLawT = new NCollection_Array1<double>(1, aNbLaw + 1);
            theP.Intervals(aLawT, GeomAbs_Shape.GeomAbs_C0);
            for (int i = 2; i <= aNbLaw; i++)
            {
                aParams.Add(aLawT[i]);
                aParams.Add(aLawT[i] - 1.0e-7);
                aParams.Add(aLawT[i] + 1.0e-7);
            }
        }
        foreach (double u in aParams)
        {
            SameEval(theO, theN, theP, u, theWhat);
        }
        SameIntervals(theO, theN, theP, theWhat);
        SameQueries(theO, theN, theP, theWhat);

        // a sub-range
        double aT1 = theO.Real(aF, aF + 0.45 * (aL - aF)), aT2 = theO.Real(aF + 0.55 * (aL - aF), aL);
        if (SameOutcome(Catch(() => theN.SetInterval(aT1, aT2)), Catch(() => theP.SetInterval(aT1, aT2)),
                        theWhat + " SetInterval"))
        {
            double aNF = 0, aNL = 0;
            theN.GetInterval(ref aNF, ref aNL);
            theP.GetInterval(out double aPF, out double aPL);
            theO.Same(aNF, aPF, theWhat + " GetInterval first");
            theO.Same(aNL, aPL, theWhat + " GetInterval last");
            SameEval(theO, theN, theP, aT1, theWhat + " interval");
            SameEval(theO, theN, theP, aT2, theWhat + " interval");
            for (int k = 0; k < 4; k++)
            {
                SameEval(theO, theN, theP, theO.Real(aT1, aT2), theWhat + " interval");
            }
            SameAverage(theO, theN, theP, theWhat + " interval");
        }

        // a copy evaluates the whole curve
        using N.GeomFill_TrihedronLaw aNCopy = theN.Copy()!;
        GeomFill_TrihedronLaw aPCopy = theP.Copy();
        Assert.Equal(theP.GetType(), aPCopy.GetType());
        for (int k = 0; k < 3; k++)
        {
            SameEval(theO, aNCopy, aPCopy, aParams[theO.Int(0, aParams.Count - 1)], theWhat + " Copy");
        }
    }

    /// <summary>Random parameters, the bounds of the curve's intervals of every continuity, and the ends.</summary>
    private static List<double> Params(Oracle theO, GeomAdaptor_Curve theC, double theF, double theL)
    {
        var aParams = new List<double> { theF, theL };
        foreach (GeomAbs_Shape aS in Shapes)
        {
            int aNb = theC.NbIntervals(aS);
            if (aNb <= 1)
            {
                continue;
            }
            var aT = new NCollection_Array1<double>(1, aNb + 1);
            theC.Intervals(aT, aS);
            for (int i = 2; i <= aNb; i++)
            {
                if (!aParams.Contains(aT[i]))
                {
                    aParams.Add(aT[i]);
                }
            }
        }
        for (int i = 0; i < 8; i++)
        {
            aParams.Add(theO.Real(theF, theL));
        }
        return aParams;
    }

    internal static void SameEval(Oracle theO, N.GeomFill_TrihedronLaw theN, GeomFill_TrihedronLaw theP, double theU,
                                  string theWhat)
    {
        string aWhat = $"{theWhat} at {theU:R}";
        {
            using var nT = new N.gp_Vec();
            using var nN = new N.gp_Vec();
            using var nB = new N.gp_Vec();
            gp_Vec pT = new(), pN = new(), pB = new();
            bool aNR = false, aPR = false;
            if (SameOutcome(Catch(() => aNR = theN.D0(theU, nT, nN, nB)), Catch(() => aPR = theP.D0(theU, ref pT, ref pN, ref pB)),
                            aWhat + " D0"))
            {
                theO.Same(aNR, aPR, aWhat + " D0");
                theO.Same(nT, pT, aWhat + " D0 T");
                theO.Same(nN, pN, aWhat + " D0 N");
                theO.Same(nB, pB, aWhat + " D0 B");
            }
        }
        {
            using var nT = new N.gp_Vec();
            using var nDT = new N.gp_Vec();
            using var nN = new N.gp_Vec();
            using var nDN = new N.gp_Vec();
            using var nB = new N.gp_Vec();
            using var nDB = new N.gp_Vec();
            gp_Vec pT = new(), pDT = new(), pN = new(), pDN = new(), pB = new(), pDB = new();
            bool aNR = false, aPR = false;
            if (SameOutcome(Catch(() => aNR = theN.D1(theU, nT, nDT, nN, nDN, nB, nDB)),
                            Catch(() => aPR = theP.D1(theU, ref pT, ref pDT, ref pN, ref pDN, ref pB, ref pDB)),
                            aWhat + " D1"))
            {
                theO.Same(aNR, aPR, aWhat + " D1");
                theO.Same(nT, pT, aWhat + " D1 T");
                theO.Same(nDT, pDT, aWhat + " D1 DT");
                theO.Same(nN, pN, aWhat + " D1 N");
                theO.Same(nDN, pDN, aWhat + " D1 DN");
                theO.Same(nB, pB, aWhat + " D1 B");
                theO.Same(nDB, pDB, aWhat + " D1 DB");
            }
        }
        {
            using var nT = new N.gp_Vec();
            using var nDT = new N.gp_Vec();
            using var nD2T = new N.gp_Vec();
            using var nN = new N.gp_Vec();
            using var nDN = new N.gp_Vec();
            using var nD2N = new N.gp_Vec();
            using var nB = new N.gp_Vec();
            using var nDB = new N.gp_Vec();
            using var nD2B = new N.gp_Vec();
            gp_Vec pT = new(), pDT = new(), pD2T = new(), pN = new(), pDN = new(), pD2N = new(), pB = new(), pDB = new(),
                   pD2B = new();
            bool aNR = false, aPR = false;
            if (SameOutcome(Catch(() => aNR = theN.D2(theU, nT, nDT, nD2T, nN, nDN, nD2N, nB, nDB, nD2B)),
                            Catch(() => aPR = theP.D2(theU, ref pT, ref pDT, ref pD2T, ref pN, ref pDN, ref pD2N, ref pB,
                                                      ref pDB, ref pD2B)),
                            aWhat + " D2"))
            {
                theO.Same(aNR, aPR, aWhat + " D2");
                theO.Same(nT, pT, aWhat + " D2 T");
                theO.Same(nDT, pDT, aWhat + " D2 DT");
                theO.Same(nD2T, pD2T, aWhat + " D2 D2T");
                theO.Same(nN, pN, aWhat + " D2 N");
                theO.Same(nDN, pDN, aWhat + " D2 DN");
                theO.Same(nD2N, pD2N, aWhat + " D2 D2N");
                theO.Same(nB, pB, aWhat + " D2 B");
                theO.Same(nDB, pDB, aWhat + " D2 DB");
                theO.Same(nD2B, pD2B, aWhat + " D2 D2B");
            }
        }
    }

    private static void SameIntervals(Oracle theO, N.GeomFill_TrihedronLaw theN, GeomFill_TrihedronLaw theP, string theWhat)
    {
        foreach (GeomAbs_Shape aS in Shapes)
        {
            int aNNb = 0, aPNb = 0;
            if (!SameOutcome(Catch(() => aNNb = theN.NbIntervals((N.GeomAbs_Shape)(int)aS)), Catch(() => aPNb = theP.NbIntervals(aS)),
                             $"{theWhat} NbIntervals({aS})"))
            {
                continue;
            }
            theO.Same(aNNb, aPNb, $"{theWhat} NbIntervals({aS})");
            using var aNT = new N.TColStd_Array1OfReal(1, aNNb + 1);
            var aPT = new NCollection_Array1<double>(1, aPNb + 1);
            if (SameOutcome(Catch(() => theN.Intervals(aNT, (N.GeomAbs_Shape)(int)aS)), Catch(() => theP.Intervals(aPT, aS)),
                            $"{theWhat} Intervals({aS})"))
            {
                for (int i = 1; i <= aPNb + 1; i++)
                {
                    theO.Same(aNT.Value(i), aPT[i], $"{theWhat} Intervals({aS})[{i}]");
                }
            }
        }
    }

    private static void SameQueries(Oracle theO, N.GeomFill_TrihedronLaw theN, GeomFill_TrihedronLaw theP, string theWhat)
    {
        theO.Same(theN.IsConstant(), theP.IsConstant(), theWhat + " IsConstant");
        theO.Same(theN.IsOnlyBy3dCurve(), theP.IsOnlyBy3dCurve(), theWhat + " IsOnlyBy3dCurve");
        theO.Same((int)theN.ErrorStatus(), (int)theP.ErrorStatus(), theWhat + " ErrorStatus");
        SameAverage(theO, theN, theP, theWhat);
    }

    private static void SameAverage(Oracle theO, N.GeomFill_TrihedronLaw theN, GeomFill_TrihedronLaw theP, string theWhat)
    {
        using var nT = new N.gp_Vec();
        using var nN = new N.gp_Vec();
        using var nB = new N.gp_Vec();
        gp_Vec pT = new(), pN = new(), pB = new();
        if (SameOutcome(Catch(() => theN.GetAverageLaw(nT, nN, nB)), Catch(() => theP.GetAverageLaw(ref pT, ref pN, ref pB)),
                        theWhat + " GetAverageLaw"))
        {
            theO.Same(nT, pT, theWhat + " GetAverageLaw T");
            theO.Same(nN, pN, theWhat + " GetAverageLaw N");
            theO.Same(nB, pB, theWhat + " GetAverageLaw B");
        }
    }
}
