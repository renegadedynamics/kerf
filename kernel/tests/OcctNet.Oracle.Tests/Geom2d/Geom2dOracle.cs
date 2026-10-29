#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests.Geom2d;

/// <summary>A random B-spline curve definition, built identically on both sides.</summary>
internal sealed class BSplineSpec
{
    public required gp_Pnt2d[] Poles;
    public double[]? Weights;
    public required double[] Knots;
    public required int[] Mults;
    public int Degree;
    public bool Periodic;
}

/// <summary>Shared helpers of the Geom2d oracle tests: random inputs, native construction, comparisons.</summary>
internal static class Geom2dOracle
{
    /// <summary>
    /// A random transformation, built by the same calls on both sides (1 to 3 elementary ones composed).
    /// </summary>
    public static (gp_Trsf2d Port, N.gp_Trsf2d Native) Trsf(Oracle theOracle)
    {
        gp_Trsf2d aPort = new gp_Trsf2d();
        N.gp_Trsf2d aNative = new N.gp_Trsf2d();
        int aCount = theOracle.Int(1, 3);
        for (int i = 0; i < aCount; i++)
        {
            gp_Trsf2d aT = new gp_Trsf2d();
            using N.gp_Trsf2d aNT = new N.gp_Trsf2d();
            switch (theOracle.Int(0, 4))
            {
                case 0:
                {
                    gp_Pnt2d aP = theOracle.Pnt2d();
                    double anAng = theOracle.Angle();
                    using var aNP = Oracle.ToNative(aP);
                    aT.SetRotation(aP, anAng);
                    aNT.SetRotation(aNP, anAng);
                    break;
                }
                case 1:
                {
                    gp_Pnt2d aP = theOracle.Pnt2d();
                    double aS = theOracle.Real(0.2, 5.0) * (theOracle.Bool() ? 1.0 : -1.0);
                    using var aNP = Oracle.ToNative(aP);
                    aT.SetScale(aP, aS);
                    aNT.SetScale(aNP, aS);
                    break;
                }
                case 2:
                {
                    gp_Pnt2d aP = theOracle.Pnt2d();
                    using var aNP = Oracle.ToNative(aP);
                    aT.SetMirror(aP);
                    aNT.SetMirror(aNP);
                    break;
                }
                case 3:
                {
                    (N.gp_Ax2d aNA, gp_Ax2d anA) = GeomOracleInputs.Ax2ds(theOracle);
                    using var aNAOwner = aNA;
                    aT.SetMirror(anA);
                    aNT.SetMirror(aNA);
                    break;
                }
                default:
                {
                    gp_Vec2d aV = theOracle.Vec2d();
                    using var aNV = Oracle.ToNative(aV);
                    aT.SetTranslation(aV);
                    aNT.SetTranslation(aNV);
                    break;
                }
            }
            aPort.Multiply(aT);
            aNative.Multiply(aNT);
        }
        return (aPort, aNative);
    }

    /// <summary>Compares D0 to D3 and DN (orders 1 to theMaxN) at U.</summary>
    public static void SameEval(Oracle theOracle, N.Geom2d_Curve theNative, Geom2d_Curve thePort, double U, string theWhat,
                                int theMaxN = 4, double theTolerance = Oracle.Tolerance)
    {
        using var aNP = new N.gp_Pnt2d();
        using var aNV1 = new N.gp_Vec2d();
        using var aNV2 = new N.gp_Vec2d();
        using var aNV3 = new N.gp_Vec2d();

        theNative.D0(U, aNP);
        thePort.D0(U, out gp_Pnt2d aP);
        theOracle.Same(Oracle.FromNative(aNP), aP.XY(), $"{theWhat} D0({U})", theTolerance);

        theNative.D1(U, aNP, aNV1);
        thePort.D1(U, out aP, out gp_Vec2d aV1);
        theOracle.Same(Oracle.FromNative(aNP), aP.XY(), $"{theWhat} D1 P", theTolerance);
        theOracle.Same(Oracle.FromNative(aNV1), aV1.XY(), $"{theWhat} D1 V1", theTolerance);

        theNative.D2(U, aNP, aNV1, aNV2);
        thePort.D2(U, out aP, out aV1, out gp_Vec2d aV2);
        theOracle.Same(Oracle.FromNative(aNP), aP.XY(), $"{theWhat} D2 P", theTolerance);
        theOracle.Same(Oracle.FromNative(aNV1), aV1.XY(), $"{theWhat} D2 V1", theTolerance);
        theOracle.Same(Oracle.FromNative(aNV2), aV2.XY(), $"{theWhat} D2 V2", theTolerance);

        theNative.D3(U, aNP, aNV1, aNV2, aNV3);
        thePort.D3(U, out aP, out aV1, out aV2, out gp_Vec2d aV3);
        theOracle.Same(Oracle.FromNative(aNP), aP.XY(), $"{theWhat} D3 P", theTolerance);
        theOracle.Same(Oracle.FromNative(aNV1), aV1.XY(), $"{theWhat} D3 V1", theTolerance);
        theOracle.Same(Oracle.FromNative(aNV2), aV2.XY(), $"{theWhat} D3 V2", theTolerance);
        theOracle.Same(Oracle.FromNative(aNV3), aV3.XY(), $"{theWhat} D3 V3", theTolerance);

        for (int n = 1; n <= theMaxN; n++)
        {
            using var aNDN = theNative.DN(U, n);
            theOracle.Same(Oracle.FromNative(aNDN), thePort.DN(U, n).XY(), $"{theWhat} DN({n})", theTolerance);
        }
    }

    /// <summary>Compares the parameter range, closure and periodicity.</summary>
    public static void SameBounds(Oracle theOracle, N.Geom2d_Curve theNative, Geom2d_Curve thePort, string theWhat)
    {
        theOracle.Same(theNative.FirstParameter(), thePort.FirstParameter(), $"{theWhat} FirstParameter");
        theOracle.Same(theNative.LastParameter(), thePort.LastParameter(), $"{theWhat} LastParameter");
        theOracle.Same(theNative.IsClosed(), thePort.IsClosed(), $"{theWhat} IsClosed");
        theOracle.Same(theNative.IsPeriodic(), thePort.IsPeriodic(), $"{theWhat} IsPeriodic");
        theOracle.Same((int)theNative.Continuity(), (int)thePort.Continuity(), $"{theWhat} Continuity");
    }

    /// <summary>A random B-spline: degree 1 to 5, rational or not, periodic or not; a third have uniform knots.</summary>
    public static BSplineSpec RandomBSpline(Oracle theOracle, bool? thePeriodic = null, bool theUniform = false)
    {
        bool aPeriodic = thePeriodic ?? theOracle.Bool();
        bool aUniform = theUniform || theOracle.Int(0, 2) == 0;
        while (true)
        {
            // Degree and knot count drawn together, so that every draw can eventually succeed
            int aDeg = theOracle.Int(1, 5);
            int aNbKnots = theOracle.Int(2, aUniform ? 12 : 6);
            double[] aKnots = new double[aNbKnots];
            int[] aMults = new int[aNbKnots];
            double aK = theOracle.Real(-5, 5);
            double aStep = theOracle.Real(0.2, 3.0);
            for (int i = 0; i < aNbKnots; i++)
            {
                aKnots[i] = aK;
                aK += aUniform ? aStep : theOracle.Real(0.1, 3.0);
                aMults[i] = aUniform ? 1 : theOracle.Int(1, aDeg);
            }
            if (aPeriodic)
            {
                aMults[aNbKnots - 1] = aMults[0];
            }
            else if (!aUniform || theOracle.Bool())
            {
                aMults[0] = aMults[aNbKnots - 1] = aDeg + 1;
            }

            int aSum = 0;
            foreach (int m in aMults)
            {
                aSum += m;
            }
            int aNbPoles = aPeriodic ? aSum - aMults[aNbKnots - 1] : aSum - aDeg - 1;
            if (aNbPoles < 2 || aNbPoles < aDeg + 1)
            {
                continue;
            }

            gp_Pnt2d[] aPoles = new gp_Pnt2d[aNbPoles];
            for (int i = 0; i < aNbPoles; i++)
            {
                aPoles[i] = new gp_Pnt2d(theOracle.Real(-10, 10), theOracle.Real(-10, 10));
            }
            double[]? aWeights = null;
            if (theOracle.Bool())
            {
                aWeights = new double[aNbPoles];
                for (int i = 0; i < aNbPoles; i++)
                {
                    aWeights[i] = theOracle.Real(0.5, 2.0);
                }
            }
            return new BSplineSpec
            {
                Poles = aPoles, Weights = aWeights, Knots = aKnots, Mults = aMults, Degree = aDeg, Periodic = aPeriodic
            };
        }
    }

    public static Geom2d_BSplineCurve ToPort(BSplineSpec theSpec)
    {
        var aPoles = NCollection_Array1<gp_Pnt2d>.From(theSpec.Poles);
        var aKnots = NCollection_Array1<double>.From(theSpec.Knots);
        var aMults = NCollection_Array1<int>.From(theSpec.Mults);
        return theSpec.Weights is null
                   ? new Geom2d_BSplineCurve(aPoles, aKnots, aMults, theSpec.Degree, theSpec.Periodic)
                   : new Geom2d_BSplineCurve(aPoles, NCollection_Array1<double>.From(theSpec.Weights), aKnots, aMults,
                                             theSpec.Degree, theSpec.Periodic);
    }

    public static N.Geom2d_BSplineCurve ToNative(BSplineSpec theSpec) =>
        Oracle.ToNativeCurve(NCollection_Array1<gp_Pnt2d>.From(theSpec.Poles),
                             theSpec.Weights is null ? null : NCollection_Array1<double>.From(theSpec.Weights),
                             NCollection_Array1<double>.From(theSpec.Knots),
                             NCollection_Array1<int>.From(theSpec.Mults),
                             theSpec.Degree,
                             theSpec.Periodic);

    /// <summary>Compares the whole definition: degree, flags, poles, weights, knots, multiplicities, flat knots.</summary>
    public static void SameStructure(Oracle theOracle, N.Geom2d_BSplineCurve theNative, Geom2d_BSplineCurve thePort, string theWhat)
    {
        theOracle.Same(theNative.Degree(), thePort.Degree(), $"{theWhat} Degree");
        theOracle.Same(theNative.IsPeriodic(), thePort.IsPeriodic(), $"{theWhat} IsPeriodic");
        theOracle.Same(theNative.IsRational(), thePort.IsRational(), $"{theWhat} IsRational");
        theOracle.Same(theNative.NbPoles(), thePort.NbPoles(), $"{theWhat} NbPoles");
        theOracle.Same(theNative.NbKnots(), thePort.NbKnots(), $"{theWhat} NbKnots");
        theOracle.Same((int)theNative.KnotDistribution(), (int)thePort.KnotDistribution(), $"{theWhat} KnotDistribution");
        theOracle.Same((int)theNative.Continuity(), (int)thePort.Continuity(), $"{theWhat} Continuity");

        using (var aNPoles = theNative.Poles())
        using (var aNWeights = theNative.WeightsArray())
        {
            PackedList<gp_Pnt2d> aPoles = thePort.Poles();
            PackedList<double> aWeights = thePort.WeightsArray();
            for (int i = 1; i <= thePort.NbPoles(); i++)
            {
                using var aNP = aNPoles.Value(i);
                theOracle.Same(Oracle.FromNative(aNP), aPoles[i - 1].XY(), $"{theWhat} Pole({i})");
                using var aNPi = theNative.Pole(i);
                theOracle.Same(Oracle.FromNative(aNPi), thePort.Pole(i).XY(), $"{theWhat} Pole({i})");
                theOracle.Same(aNWeights.Value(i), aWeights[i - 1], $"{theWhat} WeightsArray({i})");
                theOracle.Same(theNative.Weight(i), thePort.Weight(i), $"{theWhat} Weight({i})");
            }
        }

        using (var aNKnots = theNative.Knots())
        using (var aNMults = theNative.Multiplicities())
        {
            PackedList<double> aKnots = thePort.Knots();
            PackedList<int> aMults = thePort.Multiplicities();
            for (int i = 1; i <= thePort.NbKnots(); i++)
            {
                theOracle.Same(aNKnots.Value(i), aKnots[i - 1], $"{theWhat} Knots({i})");
                theOracle.Same(theNative.Knot(i), thePort.Knot(i), $"{theWhat} Knot({i})");
                theOracle.Same(aNMults.Value(i), aMults[i - 1], $"{theWhat} Mults({i})");
                theOracle.Same(theNative.Multiplicity(i), thePort.Multiplicity(i), $"{theWhat} Multiplicity({i})");
            }
        }

        using (var aNSeq = theNative.KnotSequence())
        {
            PackedList<double> aSeq = thePort.KnotSequence();
            theOracle.Same(aNSeq.Length(), aSeq.Count, $"{theWhat} KnotSequence length");
            theOracle.Same(aNSeq.Lower(), thePort.InternalKnotSequence().Lower(), $"{theWhat} KnotSequence lower");
            for (int i = 0; i < aSeq.Count; i++)
            {
                theOracle.Same(aNSeq.Value(aNSeq.Lower() + i), aSeq[i], $"{theWhat} KnotSequence({i})");
            }
        }

        theOracle.Same(theNative.FirstUKnotIndex(), thePort.FirstUKnotIndex(), $"{theWhat} FirstUKnotIndex");
        theOracle.Same(theNative.LastUKnotIndex(), thePort.LastUKnotIndex(), $"{theWhat} LastUKnotIndex");
        theOracle.Same(theNative.FirstParameter(), thePort.FirstParameter(), $"{theWhat} FirstParameter");
        theOracle.Same(theNative.LastParameter(), thePort.LastParameter(), $"{theWhat} LastParameter");
        theOracle.Same(theNative.IsClosed(), thePort.IsClosed(), $"{theWhat} IsClosed");
        using var aNStart = theNative.StartPoint();
        using var aNEnd = theNative.EndPoint();
        theOracle.Same(Oracle.FromNative(aNStart), thePort.StartPoint().XY(), $"{theWhat} StartPoint");
        theOracle.Same(Oracle.FromNative(aNEnd), thePort.EndPoint().XY(), $"{theWhat} EndPoint");
    }

    /// <summary>A random parameter in the curve's range (or slightly outside, for extrapolation).</summary>
    public static double Param(Oracle theOracle, Geom2d_Curve theCurve, double theOutside = 0.0)
    {
        double aF = theCurve.FirstParameter(), aL = theCurve.LastParameter();
        double aD = (aL - aF) * theOutside;
        return theOracle.Real(aF - aD, aL + aD);
    }
}
#endif
