// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Kernel.Oracle.Tests.AdaptorOracleInputs;

namespace Kerf.Kernel.Oracle.Tests;

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
    private static readonly Continuity[] Shapes =
        [Continuity.C0, Continuity.C1, Continuity.C2, Continuity.C3,
         Continuity.CN];

    /// <summary>Runs <paramref name="theCases"/> random spines of every curve kind through both laws.</summary>
    internal static void Run(ITestOutputHelper theOutput, string theName, Func<N.GeomFill_TrihedronLaw> theNative,
                             Func<TrihedronLaw> thePort, int theCases, int theSeed, int[]? theSkippedKinds = null)
    {
        for (int aKind = 0; aKind < CurveKinds.Length; aKind++)
        {
            if (theSkippedKinds is not null && Array.IndexOf(theSkippedKinds, aKind) >= 0)
            {
                continue;
            }
            var anOracle = new Oracle(theOutput, theSeed + aKind);
            foreach (int i in Tape.Budget(theCases))
            {
                (N.Geom_Curve aNC, Curve3D aPC, double aRange) = RandomCurve(anOracle, aKind);
                using (aNC)
                {
                    double aF = aPC.FirstParameter, aL = aPC.LastParameter;
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aF) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aL))
                    {
                        aF = anOracle.Real(-aRange, 0);
                        aL = aF + anOracle.Real(0.1, 2 * aRange);
                    }
                    using var aNA = new N.GeomAdaptor_Curve(aNC, aF, aL);
                    var aPA = new CurveAdaptor(aPC, aF, aL);
                    using N.GeomFill_TrihedronLaw aNLaw = theNative();
                    TrihedronLaw aPLaw = thePort();
                    Compare(anOracle, aNLaw, aPLaw, aNA, aPA, $"{theName} on {CurveKinds[aKind]} #{i}");
                }
            }
            anOracle.Report($"{theName} on {CurveKinds[aKind]}");
        }
    }

    internal static void Compare(Oracle theO, N.GeomFill_TrihedronLaw theN, TrihedronLaw theP,
                                 N.GeomAdaptor_Curve theNC, CurveAdaptor thePC, string theWhat)
    {
        // Plan 3.2's third level on an offset spine (occt-issues.md, the offset curves' CalculateD3; Kerf decision
        // 16804): the laws' D1 and D2 use the spine's D3, whose D2 and D3 OCCT got wrong, so there only D0 is compared,
        // native's D1 and D2 read as recorded (GeomFill_TrihedronLawTests checks them against finite differences); and
        // CorrectedFrenet's angle law, built from them, turns its normal otherwise: D0's tangent is compared, and its
        // normal and binormal checked to complete it to an orthonormal frame.
        bool anOffset = thePC.GetType() == CurveKind.OffsetCurve;
        Mode aMode = !anOffset ? Mode.All : theP is CorrectedFrenet ? Mode.Tangent : Mode.Trihedron;
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
        if (Catch(() => aNbLaw = theP.NbIntervals(Continuity.C0)) is null && aNbLaw > 1)
        {
            var aLawT = new NCollection_Array1<double>(1, aNbLaw + 1);
            theP.Intervals(aLawT, Continuity.C0);
            for (int i = 2; i <= aNbLaw; i++)
            {
                aParams.Add(aLawT[i]);
                aParams.Add(aLawT[i] - 1.0e-7);
                aParams.Add(aLawT[i] + 1.0e-7);
            }
        }
        foreach (double u in aParams)
        {
            SameEval(theO, theN, theP, u, theWhat, aMode);
        }
        SameIntervals(theO, theN, theP, theWhat, aMode != Mode.Tangent);
        SameQueries(theO, theN, theP, theWhat, aMode != Mode.Tangent);

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
            SameEval(theO, theN, theP, aT1, theWhat + " interval", aMode);
            SameEval(theO, theN, theP, aT2, theWhat + " interval", aMode);
            for (int k = 0; k < 4; k++)
            {
                SameEval(theO, theN, theP, theO.Real(aT1, aT2), theWhat + " interval", aMode);
            }
            SameAverage(theO, theN, theP, theWhat + " interval", aMode != Mode.Tangent);
        }

        // a copy evaluates the whole curve
        using N.GeomFill_TrihedronLaw aNCopy = theN.Copy()!;
        TrihedronLaw aPCopy = theP.Copy();
        Assert.Equal(theP.GetType(), aPCopy.GetType());
        for (int k = 0; k < 3; k++)
        {
            SameEval(theO, aNCopy, aPCopy, aParams[theO.Int(0, aParams.Count - 1)], theWhat + " Copy", aMode);
        }
    }

    /// <summary>Random parameters, the bounds of the curve's intervals of every continuity, and the ends.</summary>
    private static List<double> Params(Oracle theO, CurveAdaptor theC, double theF, double theL)
    {
        var aParams = new List<double> { theF, theL };
        foreach (Continuity aS in Shapes)
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

    /// <summary>What is compared with native: everything, D0's trihedron, or D0's tangent (see Compare).</summary>
    internal enum Mode
    {
        All,
        Trihedron,
        Tangent,
    }

    internal static void SameEval(Oracle theO, N.GeomFill_TrihedronLaw theN, TrihedronLaw theP, double theU,
                                  string theWhat, Mode theMode = Mode.All)
    {
        string aWhat = $"{theWhat} at {theU:R}";
        bool all = theMode == Mode.All;

        // compared, or native's read as recorded
        void Same(N.gp_Vec theNative, in Vector3D thePort, string theName, bool theCompare)
        {
            if (theCompare)
            {
                theO.Same(theNative, thePort, theName);
            }
            else
            {
                _ = (theNative.X(), theNative.Y(), theNative.Z());
            }
        }

        {
            using var nT = new N.gp_Vec();
            using var nN = new N.gp_Vec();
            using var nB = new N.gp_Vec();
            Vector3D pT = new(), pN = new(), pB = new();
            bool aNR = false, aPR = false;
            if (SameOutcome(Catch(() => aNR = theN.D0(theU, nT, nN, nB)), Catch(() => aPR = theP.D0(theU, ref pT, ref pN, ref pB)),
                            aWhat + " D0"))
            {
                theO.Same(aNR, aPR, aWhat + " D0");
                theO.Same(nT, pT, aWhat + " D0 T");
                Same(nN, pN, aWhat + " D0 N", theMode != Mode.Tangent);
                Same(nB, pB, aWhat + " D0 B", theMode != Mode.Tangent);
                if (theMode == Mode.Tangent && aPR)
                {
                    Assert.True(Math.Abs(pN.Magnitude - 1) < 1e-9 && Math.Abs(pN.Dot(pT)) < 1e-9
                                && (pT.Crossed(pN) - pB).Magnitude < 1e-9, aWhat + " D0: not an orthonormal frame");
                }
            }
        }
        {
            using var nT = new N.gp_Vec();
            using var nDT = new N.gp_Vec();
            using var nN = new N.gp_Vec();
            using var nDN = new N.gp_Vec();
            using var nB = new N.gp_Vec();
            using var nDB = new N.gp_Vec();
            Vector3D pT = new(), pDT = new(), pN = new(), pDN = new(), pB = new(), pDB = new();
            bool aNR = false, aPR = false;
            if (SameOutcome(Catch(() => aNR = theN.D1(theU, nT, nDT, nN, nDN, nB, nDB)),
                            Catch(() => aPR = theP.D1(theU, ref pT, ref pDT, ref pN, ref pDN, ref pB, ref pDB)),
                            aWhat + " D1"))
            {
                theO.Same(aNR, aPR, aWhat + " D1");
                Same(nT, pT, aWhat + " D1 T", all);
                Same(nDT, pDT, aWhat + " D1 DT", all);
                Same(nN, pN, aWhat + " D1 N", all);
                Same(nDN, pDN, aWhat + " D1 DN", all);
                Same(nB, pB, aWhat + " D1 B", all);
                Same(nDB, pDB, aWhat + " D1 DB", all);
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
            Vector3D pT = new(), pDT = new(), pD2T = new(), pN = new(), pDN = new(), pD2N = new(), pB = new(), pDB = new(),
                   pD2B = new();
            bool aNR = false, aPR = false;
            if (SameOutcome(Catch(() => aNR = theN.D2(theU, nT, nDT, nD2T, nN, nDN, nD2N, nB, nDB, nD2B)),
                            Catch(() => aPR = theP.D2(theU, ref pT, ref pDT, ref pD2T, ref pN, ref pDN, ref pD2N, ref pB,
                                                      ref pDB, ref pD2B)),
                            aWhat + " D2"))
            {
                theO.Same(aNR, aPR, aWhat + " D2");
                Same(nT, pT, aWhat + " D2 T", all);
                Same(nDT, pDT, aWhat + " D2 DT", all);
                Same(nD2T, pD2T, aWhat + " D2 D2T", all);
                Same(nN, pN, aWhat + " D2 N", all);
                Same(nDN, pDN, aWhat + " D2 DN", all);
                Same(nD2N, pD2N, aWhat + " D2 D2N", all);
                Same(nB, pB, aWhat + " D2 B", all);
                Same(nDB, pDB, aWhat + " D2 DB", all);
                Same(nD2B, pD2B, aWhat + " D2 D2B", all);
            }
        }
    }

    /// <remarks>
    /// Without <paramref name="theCompare"/> (CorrectedFrenet's law on an offset spine, see Compare), native's intervals
    /// are read as recorded and the port's checked to cover the range in increasing order: OCCT's law, sampled with the
    /// spine's wrong derivatives, has many more.
    /// </remarks>
    private static void SameIntervals(Oracle theO, N.GeomFill_TrihedronLaw theN, TrihedronLaw theP, string theWhat,
                                      bool theCompare = true)
    {
        foreach (Continuity aS in Shapes)
        {
            int aNNb = 0, aPNb = 0;
            if (!SameOutcome(Catch(() => aNNb = theN.NbIntervals((N.GeomAbs_Shape)(int)aS)), Catch(() => aPNb = theP.NbIntervals(aS)),
                             $"{theWhat} NbIntervals({aS})"))
            {
                continue;
            }
            if (!theCompare)
            {
                using var aNT0 = new N.TColStd_Array1OfReal(1, aNNb + 1);
                var aPT0 = new NCollection_Array1<double>(1, aPNb + 1);
                if (SameOutcome(Catch(() => theN.Intervals(aNT0, (N.GeomAbs_Shape)(int)aS)), Catch(() => theP.Intervals(aPT0, aS)),
                                $"{theWhat} Intervals({aS})"))
                {
                    double aNFirst = 0, aNLast = 0;
                    for (int i = 1; i <= aNNb + 1; i++)
                    {
                        double aValue = aNT0.Value(i);
                        aNFirst = i == 1 ? aValue : aNFirst;
                        aNLast = aValue;
                    }
                    for (int i = 1; i <= aPNb; i++)
                    {
                        Assert.True(aPT0[i] < aPT0[i + 1], $"{theWhat} Intervals({aS}) not increasing");
                    }
                    theO.Same(aNFirst, aPT0[1], $"{theWhat} Intervals({aS}) first");
                    theO.Same(aNLast, aPT0[aPNb + 1], $"{theWhat} Intervals({aS}) last");
                }
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

    private static void SameQueries(Oracle theO, N.GeomFill_TrihedronLaw theN, TrihedronLaw theP, string theWhat,
                                    bool theCompareAverage = true)
    {
        theO.Same(theN.IsConstant(), theP.IsConstant, theWhat + " IsConstant");
        theO.Same(theN.IsOnlyBy3dCurve(), theP.IsOnlyBy3dCurve, theWhat + " IsOnlyBy3dCurve");
        theO.Same((int)theN.ErrorStatus(), (int)theP.ErrorStatus(), theWhat + " ErrorStatus");
        SameAverage(theO, theN, theP, theWhat, theCompareAverage);
    }

    /// <remarks>
    /// Without <paramref name="theCompare"/> (CorrectedFrenet's law on an offset spine, see Compare), native's average
    /// is read as recorded (an average of the law's frames at its samples, which differ) and the port's checked finite.
    /// </remarks>
    private static void SameAverage(Oracle theO, N.GeomFill_TrihedronLaw theN, TrihedronLaw theP, string theWhat,
                                    bool theCompare = true)
    {
        using var nT = new N.gp_Vec();
        using var nN = new N.gp_Vec();
        using var nB = new N.gp_Vec();
        Vector3D pT = new(), pN = new(), pB = new();
        if (SameOutcome(Catch(() => theN.GetAverageLaw(nT, nN, nB)), Catch(() => theP.GetAverageLaw(ref pT, ref pN, ref pB)),
                        theWhat + " GetAverageLaw"))
        {
            if (!theCompare)
            {
                _ = (nT.X(), nT.Y(), nT.Z(), nN.X(), nN.Y(), nN.Z(), nB.X(), nB.Y(), nB.Z());
                Assert.True(double.IsFinite(pT.Magnitude + pN.Magnitude + pB.Magnitude), theWhat + " GetAverageLaw: not finite");
                return;
            }
            theO.Same(nT, pT, theWhat + " GetAverageLaw T");
            theO.Same(nN, pN, theWhat + " GetAverageLaw N");
            theO.Same(nB, pB, theWhat + " GetAverageLaw B");
        }
    }
}
