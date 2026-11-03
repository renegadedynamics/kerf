// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Kernel.Oracle.Tests.AdaptorOracleInputs;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// GeomAdaptor_Curve against native OCCT, on every kind of curve it knows (line, conics, Bezier, B-spline,
/// periodic B-spline, offset and trimmed curves), over the whole curve or random bounds: parameters, continuity,
/// the intervals of every continuity, resolution, type and the type's data (gp curve, B-spline or Bezier curve,
/// offset curve), degree, poles, knots, rationality, closure and periodicity, and evaluation (D0 to D3 and DN) in
/// a random sequence of parameters that crosses spans, lands on knots and on the bounds (the cache and the
/// boundary evaluation), also through Trim and ShallowCopy.
/// </summary>
public class GeomAdaptor_CurveOracleTests(ITestOutputHelper output)
{
    private static readonly int Cases = Tape.Cases(60, 4);

    private static readonly Continuity[] Shapes =
        [Continuity.C0, Continuity.G1, Continuity.C1, Continuity.G2,
         Continuity.C2, Continuity.C3, Continuity.CN];

    /// <summary>Continuity, and NbIntervals and Intervals for every continuity (throwing on both sides, or neither).</summary>
    private static void SameIntervals(Oracle theO, N.GeomAdaptor_Curve theN, CurveAdaptor theP, string theWhat)
    {
        int aNC = 0, aPC = 0;
        if (SameOutcome(Catch(() => aNC = (int)theN.Continuity()), Catch(() => aPC = (int)theP.Continuity()), theWhat + " Continuity"))
        {
            theO.Same(aNC, aPC, theWhat + " Continuity");
        }
        foreach (Continuity aS in Shapes)
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

    /// <summary>The queries that don't evaluate: type, data of the type, degree, poles, knots, closure, periodicity, resolution.</summary>
    private static void SameQueries(Oracle theO, N.GeomAdaptor_Curve theN, CurveAdaptor theP, string theWhat)
    {
        theO.Same(theN.FirstParameter(), theP.FirstParameter(), theWhat + " FirstParameter");
        theO.Same(theN.LastParameter(), theP.LastParameter(), theWhat + " LastParameter");
        theO.Same((int)theN.GetType_(), (int)theP.GetType(), theWhat + " GetType");
        theO.Same(theN.IsClosed(), theP.IsClosed, theWhat + " IsClosed");
        theO.Same(theN.IsPeriodic(), theP.IsPeriodic, theWhat + " IsPeriodic");
        theO.Same(theN.Period(), theP.Period(), theWhat + " Period");
        theO.Same(theN.IsRational(), theP.IsRational, theWhat + " IsRational");
        foreach (double aR3d in new[] { 1e-7, 1e-3, 0.5, 30.0 })
        {
            theO.SameRelative(theN.Resolution(aR3d), theP.Resolution(aR3d), theWhat + " Resolution");
        }

        switch (theP.GetType())
        {
            case CurveKind.Line:
            {
                using N.gp_Lin aNL = theN.Line();
                using N.gp_Ax1 aNA = aNL.Position();
                SameAx1(theO, aNA, theP.Line().Position(), theWhat + " Line");
                break;
            }
            case CurveKind.Circle:
            {
                using N.gp_Circ aNC = theN.Circle();
                using N.gp_Ax2 aNA = aNC.Position();
                SameAx2(theO, aNA, theP.Circle().Position(), theWhat + " Circle");
                theO.Same(aNC.Radius(), theP.Circle().Radius(), theWhat + " Circle radius");
                break;
            }
            case CurveKind.Ellipse:
            {
                using N.gp_Elips aNE = theN.Ellipse();
                using N.gp_Ax2 aNA = aNE.Position();
                SameAx2(theO, aNA, theP.Ellipse().Position(), theWhat + " Ellipse");
                theO.Same(aNE.MajorRadius(), theP.Ellipse().MajorRadius(), theWhat + " Ellipse major");
                theO.Same(aNE.MinorRadius(), theP.Ellipse().MinorRadius(), theWhat + " Ellipse minor");
                break;
            }
            case CurveKind.Hyperbola:
            {
                using N.gp_Hypr aNH = theN.Hyperbola();
                using N.gp_Ax2 aNA = aNH.Position();
                SameAx2(theO, aNA, theP.Hyperbola().Position(), theWhat + " Hyperbola");
                theO.Same(aNH.MajorRadius(), theP.Hyperbola().MajorRadius(), theWhat + " Hyperbola major");
                theO.Same(aNH.MinorRadius(), theP.Hyperbola().MinorRadius(), theWhat + " Hyperbola minor");
                break;
            }
            case CurveKind.Parabola:
            {
                using N.gp_Parab aNPa = theN.Parabola();
                using N.gp_Ax2 aNA = aNPa.Position();
                SameAx2(theO, aNA, theP.Parabola().Position(), theWhat + " Parabola");
                theO.Same(aNPa.Focal(), theP.Parabola().Focal(), theWhat + " Parabola focal");
                break;
            }
            case CurveKind.BezierCurve:
            {
                theO.Same(theN.Degree(), theP.Degree(), theWhat + " Degree");
                theO.Same(theN.NbPoles(), theP.PoleCount, theWhat + " NbPoles");
                using N.Geom_BezierCurve aNB = theN.Bezier()!;
                GeomIsoCurveCompare.SameCurve(theO, aNB, theP.Bezier(), theWhat + " Bezier()");
                break;
            }
            case CurveKind.BSplineCurve:
            {
                theO.Same(theN.Degree(), theP.Degree(), theWhat + " Degree");
                theO.Same(theN.NbPoles(), theP.PoleCount, theWhat + " NbPoles");
                theO.Same(theN.NbKnots(), theP.KnotCount, theWhat + " NbKnots");
                using N.Geom_BSplineCurve aNB = theN.BSpline()!;
                GeomIsoCurveCompare.SameCurve(theO, aNB, theP.BSpline(), theWhat + " BSpline()");
                break;
            }
            case CurveKind.OffsetCurve:
            {
                using N.Geom_OffsetCurve aNO = theN.OffsetCurve()!;
                OffsetCurve3D aPO = theP.OffsetCurve();
                theO.Same(aNO.Offset(), aPO.Offset(), theWhat + " OffsetCurve offset");
                using N.gp_Dir aND = aNO.Direction();
                SameDir(theO, aND, aPO.Direction(), theWhat + " OffsetCurve direction");
                break;
            }
        }
    }

    /// <summary>
    /// A sequence of parameters in [theFirst, theLast]: random ones, the bounds (exactly), the knots of a B-spline
    /// curve, and points just inside the bounds; several in a row in the same span, so that the cache is reused.
    /// </summary>
    private static List<double> Params(Oracle theO, CurveAdaptor theP, double theRange)
    {
        double aF = theP.FirstParameter(), aL = theP.LastParameter();
        bool isInfinite = global::Kerf.Engine.Foundation.Precision.IsInfinite(aF) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aL);
        var aParams = new List<double>();
        for (int k = 0; k < 8; k++)
        {
            aParams.Add(Param(theO, aF, aL, theRange));
        }
        if (!isInfinite)
        {
            aParams.Add(aF);
            aParams.Add(aL);
            aParams.Add(aF + 1e-9 * (aL - aF));
            aParams.Add(aL);
            aParams.Add(aL - 1e-9 * (aL - aF));
            aParams.Add(aF);
        }
        if (theP.GetType() == CurveKind.BSplineCurve)
        {
            BSplineCurve3D aBS = theP.BSpline();
            for (int i = 1; i <= aBS.KnotCount; i++)
            {
                double aK = aBS.Knot(i);
                if (isInfinite || (aK >= aF && aK <= aL))
                {
                    aParams.Add(aK);
                }
            }
            if (aBS.IsPeriodic)
            {
                aParams.Add(Param(theO, aF, aL, theRange) + aBS.Period() * theO.Int(-2, 2));
            }
        }
        // shuffled, then a few repeated in place (same span)
        for (int i = aParams.Count - 1; i > 0; i--)
        {
            int j = theO.Int(0, i);
            (aParams[i], aParams[j]) = (aParams[j], aParams[i]);
        }
        return aParams;
    }

    [Fact]
    public void Curves()
    {
        for (int aKind = 0; aKind < CurveKinds.Length; aKind++)
        {
            var anOracle = new Oracle(output, 20261101 + aKind);
            for (int i = 0; i < Cases; i++)
            {
                (N.Geom_Curve aNC, Curve3D aPC, double aRange) = RandomCurve(anOracle, aKind);
                using (aNC)
                {
                    string aWhat = CurveKinds[aKind];

                    // the whole curve, and random bounds within it
                    using var aNA = new N.GeomAdaptor_Curve(aNC);
                    var aPA = new CurveAdaptor(aPC);
                    CompareAdaptors(anOracle, aNA, aPA, aRange, aWhat);

                    double aF = aPC.FirstParameter(), aL = aPC.LastParameter();
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aF) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aL) || aPC.IsPeriodic && anOracle.Bool())
                    {
                        aF = anOracle.Real(-aRange, 0);
                        aL = aF + anOracle.Real(0.1, 2 * aRange);
                    }
                    else
                    {
                        double aD = aL - aF;
                        (aF, aL) = (anOracle.Real(aF, aF + 0.45 * aD), anOracle.Real(aF + 0.55 * aD, aL));
                    }
                    using var aNB = new N.GeomAdaptor_Curve(aNC, aF, aL);
                    var aPB = new CurveAdaptor(aPC, aF, aL);
                    CompareAdaptors(anOracle, aNB, aPB, aRange, aWhat + " bounded");

                    // Load on an existing adaptor: the same curve with other bounds (the cache is dropped), then another curve
                    aNA.Load(aNC, aF, aL);
                    aPA.Load(aPC, aF, aL);
                    CompareAdaptors(anOracle, aNA, aPA, aRange, aWhat + " reloaded");
                }
            }
            anOracle.Report("GeomAdaptor_Curve " + CurveKinds[aKind]);
        }
    }

    private static void CompareAdaptors(Oracle theO, N.GeomAdaptor_Curve theN, CurveAdaptor theP, double theRange,
                                        string theWhat)
    {
        SameQueries(theO, theN, theP, theWhat);
        SameIntervals(theO, theN, theP, theWhat);
        foreach (double u in Params(theO, theP, theRange))
        {
            SameCurveEval(theO, theN, theP, u, 4, theWhat);
        }

        // Trim and ShallowCopy evaluate as the original
        double aF = theP.FirstParameter(), aL = theP.LastParameter();
        double aT1 = Param(theO, aF, aL, theRange), aT2 = Param(theO, aF, aL, theRange);
        if (aT1 > aT2)
        {
            (aT1, aT2) = (aT2, aT1);
        }
        using N.Adaptor3d_Curve aNT = theN.Trim(aT1, aT2, 1e-7)!;
        CurveAdaptorBase aPT = theP.Trim(aT1, aT2, 1e-7);
        theO.Same(aNT.FirstParameter(), aPT.FirstParameter(), theWhat + " Trim first");
        theO.Same(aNT.LastParameter(), aPT.LastParameter(), theWhat + " Trim last");
        SameCurveEval(theO, aNT, aPT, aT1, 3, theWhat + " Trim");
        SameCurveEval(theO, aNT, aPT, 0.5 * (aT1 + aT2), 3, theWhat + " Trim");

        using N.Adaptor3d_Curve aNS = theN.ShallowCopy()!;
        CurveAdaptorBase aPS = theP.ShallowCopy();
        SameCurveEval(theO, aNS, aPS, Param(theO, aF, aL, theRange), 3, theWhat + " ShallowCopy");
    }
}
