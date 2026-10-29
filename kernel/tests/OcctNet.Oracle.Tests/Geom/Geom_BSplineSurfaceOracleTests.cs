#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using N = Build123d.Occt;
using static OcctNet.Oracle.Tests.GeomOracleInputs;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Geom_BSplineSurface against native OCCT: random surfaces (degrees 1 to 5 in each direction, every combination
/// of U and V periodicity, rational half the time in one direction or both), compared field by field (poles,
/// weights, knots, multiplicities, flat knots, flags, bounds, closure, continuity) and by evaluation (D0 to D3, DN,
/// local evaluation, locate, resolution), then again after random edits applied to both.
/// </summary>
public class Geom_BSplineSurfaceOracleTests(ITestOutputHelper output)
{
    private const int Cases = 200;

    /// <summary>A random surface, built natively and in the port from the same data.</summary>
    private static (N.Geom_BSplineSurface Native, Geom_BSplineSurface Port) RandomSurface(Oracle theOracle)
    {
        int aUDeg = theOracle.Int(1, 5), aVDeg = theOracle.Int(1, 5);
        bool aUPer = theOracle.Int(0, 2) == 1, aVPer = theOracle.Int(0, 2) == 1;
        bool aRational = theOracle.Int(0, 1) == 1;
        theOracle.BSplineKnots(aUDeg, aUPer, out NCollection_Array1<double> aUKnots, out NCollection_Array1<int> aUMults);
        theOracle.BSplineKnots(aVDeg, aVPer, out NCollection_Array1<double> aVKnots, out NCollection_Array1<int> aVMults);
        int aNbU = Occt.BSplCLib.NbPoles(aUDeg, aUPer, aUMults);
        int aNbV = Occt.BSplCLib.NbPoles(aVDeg, aVPer, aVMults);
        var aPoles = new NCollection_Array2<gp_Pnt>(1, aNbU, 1, aNbV);
        var aWeights = new NCollection_Array2<double>(1, aNbU, 1, aNbV);
        NCollection_Array1<double> aUW = theOracle.Weights(aNbU, aRational);
        NCollection_Array1<double> aVW = theOracle.Weights(aNbV, aRational);
        int aMode = theOracle.Int(0, 2);
        for (int i = 1; i <= aNbU; i++)
        {
            for (int j = 1; j <= aNbV; j++)
            {
                aPoles[i, j] = new gp_Pnt(theOracle.Real(0, 10), theOracle.Real(0, 10), theOracle.Real(0, 10));
                aWeights[i, j] = !aRational ? 1.0 : aMode == 0 ? theOracle.Real(0.5, 1.5) : aMode == 1 ? aUW[i] : aVW[j];
            }
        }

        // Sometimes a closed (non-periodic) surface: the last row of poles repeats the first
        if (!aUPer && theOracle.Int(0, 4) == 0)
        {
            for (int j = 1; j <= aNbV; j++)
            {
                aPoles[aNbU, j] = aPoles[1, j];
                aWeights[aNbU, j] = aWeights[1, j];
            }
        }

        using N.TColgp_Array2OfPnt aNPoles = Oracle.ToNative(aPoles);
        using N.TColStd_Array1OfReal aNUKnots = Oracle.ToNative(aUKnots);
        using N.TColStd_Array1OfReal aNVKnots = Oracle.ToNative(aVKnots);
        using N.TColStd_Array1OfInteger aNUMults = Oracle.ToNative(aUMults);
        using N.TColStd_Array1OfInteger aNVMults = Oracle.ToNative(aVMults);
        if (!aRational)
        {
            return (new N.Geom_BSplineSurface(aNPoles, aNUKnots, aNVKnots, aNUMults, aNVMults, aUDeg, aVDeg, aUPer, aVPer),
                    new Geom_BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, aUDeg, aVDeg, aUPer, aVPer));
        }

        using N.TColStd_Array2OfReal aNWeights = Oracle.ToNative(aWeights);
        return (new N.Geom_BSplineSurface(aNPoles, aNWeights, aNUKnots, aNVKnots, aNUMults, aNVMults, aUDeg, aVDeg, aUPer, aVPer),
                new Geom_BSplineSurface(aPoles, aWeights, aUKnots, aVKnots, aUMults, aVMults, aUDeg, aVDeg, aUPer, aVPer));
    }

    /// <summary>Compares the data of the two surfaces.</summary>
    private static void CompareData(Oracle theOracle, N.Geom_BSplineSurface theN, Geom_BSplineSurface theP, string theWhat)
    {
        theOracle.Same(theN.UDegree(), theP.UDegree(), theWhat + " UDegree");
        theOracle.Same(theN.VDegree(), theP.VDegree(), theWhat + " VDegree");
        theOracle.Same(theN.NbUPoles(), theP.NbUPoles(), theWhat + " NbUPoles");
        theOracle.Same(theN.NbVPoles(), theP.NbVPoles(), theWhat + " NbVPoles");
        theOracle.Same(theN.NbUKnots(), theP.NbUKnots(), theWhat + " NbUKnots");
        theOracle.Same(theN.NbVKnots(), theP.NbVKnots(), theWhat + " NbVKnots");
        theOracle.Same(theN.IsUPeriodic(), theP.IsUPeriodic(), theWhat + " IsUPeriodic");
        theOracle.Same(theN.IsVPeriodic(), theP.IsVPeriodic(), theWhat + " IsVPeriodic");
        theOracle.Same(theN.IsURational(), theP.IsURational(), theWhat + " IsURational");
        theOracle.Same(theN.IsVRational(), theP.IsVRational(), theWhat + " IsVRational");
        theOracle.Same((int)theN.Continuity(), (int)theP.Continuity(), theWhat + " Continuity");
        theOracle.Same((int)theN.UKnotDistribution(), (int)theP.UKnotDistribution(), theWhat + " UKnotDistribution");
        theOracle.Same((int)theN.VKnotDistribution(), (int)theP.VKnotDistribution(), theWhat + " VKnotDistribution");
        theOracle.Same(theN.FirstUKnotIndex(), theP.FirstUKnotIndex(), theWhat + " FirstUKnotIndex");
        theOracle.Same(theN.LastUKnotIndex(), theP.LastUKnotIndex(), theWhat + " LastUKnotIndex");
        theOracle.Same(theN.FirstVKnotIndex(), theP.FirstVKnotIndex(), theWhat + " FirstVKnotIndex");
        theOracle.Same(theN.LastVKnotIndex(), theP.LastVKnotIndex(), theWhat + " LastVKnotIndex");
        for (int k = 0; k <= 4; k++)
        {
            theOracle.Same(theN.IsCNu(k), theP.IsCNu(k), theWhat + " IsCNu");
            theOracle.Same(theN.IsCNv(k), theP.IsCNv(k), theWhat + " IsCNv");
        }

        // Closure compares iso-curves, whose construction throws when a rational edit left a weight near zero
        bool aNC = false, aPC = false;
        if (Both(() => aNC = theN.IsUClosed(), () => aPC = theP.IsUClosed(), theWhat + " IsUClosed"))
        {
            theOracle.Same(aNC, aPC, theWhat + " IsUClosed");
        }

        if (Both(() => aNC = theN.IsVClosed(), () => aPC = theP.IsVClosed(), theWhat + " IsVClosed"))
        {
            theOracle.Same(aNC, aPC, theWhat + " IsVClosed");
        }

        double aNU1 = 0, aNU2 = 0, aNV1 = 0, aNV2 = 0;
        theN.Bounds(ref aNU1, ref aNU2, ref aNV1, ref aNV2);
        theP.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        theOracle.Same(aNU1, aU1, theWhat + " U1");
        theOracle.Same(aNU2, aU2, theWhat + " U2");
        theOracle.Same(aNV1, aV1, theWhat + " V1");
        theOracle.Same(aNV2, aV2, theWhat + " V2");

        for (int i = 1; i <= theP.NbUKnots(); i++)
        {
            theOracle.Same(theN.UKnot(i), theP.UKnot(i), theWhat + " UKnot");
            theOracle.Same(theN.UMultiplicity(i), theP.UMultiplicity(i), theWhat + " UMult");
        }

        for (int i = 1; i <= theP.NbVKnots(); i++)
        {
            theOracle.Same(theN.VKnot(i), theP.VKnot(i), theWhat + " VKnot");
            theOracle.Same(theN.VMultiplicity(i), theP.VMultiplicity(i), theWhat + " VMult");
        }

        using (N.TColStd_Array1OfReal aNSeq = theN.UKnotSequence())
        {
            PackedList<double> aSeq = theP.UKnotSequence();
            theOracle.Same(aNSeq.Length(), aSeq.Count, theWhat + " UKnotSequence length");
            for (int i = 0; i < aSeq.Count; i++)
            {
                theOracle.Same(aNSeq.Value(aNSeq.Lower() + i), aSeq[i], theWhat + " UKnotSequence");
            }
        }

        using (N.TColStd_Array1OfReal aNSeq = theN.VKnotSequence())
        {
            PackedList<double> aSeq = theP.VKnotSequence();
            theOracle.Same(aNSeq.Length(), aSeq.Count, theWhat + " VKnotSequence length");
            for (int i = 0; i < aSeq.Count; i++)
            {
                theOracle.Same(aNSeq.Value(aNSeq.Lower() + i), aSeq[i], theWhat + " VKnotSequence");
            }
        }

        for (int i = 1; i <= theP.NbUPoles(); i++)
        {
            for (int j = 1; j <= theP.NbVPoles(); j++)
            {
                using N.gp_Pnt aNP = theN.Pole(i, j);
                theOracle.Same(aNP, theP.Pole(i, j), theWhat + " Pole");
                theOracle.Same(theN.Weight(i, j), theP.Weight(i, j), theWhat + " Weight");
            }
        }
    }

    /// <summary>Compares the evaluations on a grid of parameters, some outside a periodic domain.</summary>
    private static void CompareEvaluation(Oracle theOracle, N.Geom_BSplineSurface theN, Geom_BSplineSurface theP, string theWhat)
    {
        theP.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        const int n = 4;
        for (int ju = 0; ju <= n; ju++)
        {
            for (int jv = 0; jv <= n; jv++)
            {
                double aU = aU1 + (aU2 - aU1) * ju / n + (theP.IsUPeriodic() && ju == 1 ? (aU2 - aU1) * 1.3 : 0.0);
                double aV = aV1 + (aV2 - aV1) * jv / n + (theP.IsVPeriodic() && jv == 3 ? -(aV2 - aV1) * 2.2 : 0.0);
                CompareAt(theOracle, theN, theP, aU, aV, theWhat);
            }
        }

        // Inside spans, away from the grid
        for (int k = 0; k < 4; k++)
        {
            CompareAt(theOracle, theN, theP, theOracle.Real(aU1, aU2), theOracle.Real(aV1, aV2), theWhat);
        }
    }

    private static void CompareAt(Oracle theOracle, N.Geom_BSplineSurface theN, Geom_BSplineSurface theP, double aU, double aV,
                                  string theWhat)
    {
        using (N.gp_Pnt aNP = theN.EvalD0(aU, aV))
        {
            theOracle.Same(aNP, theP.Value(aU, aV), theWhat + " D0");
        }

        using (var aNP = new N.gp_Pnt(0, 0, 0))
        using (var aNU = new N.gp_Vec(0, 0, 0))
        using (var aNV = new N.gp_Vec(0, 0, 0))
        {
            theN.D1(aU, aV, aNP, aNU, aNV);
            theP.D1(aU, aV, out gp_Pnt aP, out gp_Vec aD1U, out gp_Vec aD1V);
            theOracle.Same(aNP, aP, theWhat + " D1 P");
            theOracle.Same(aNU, aD1U, theWhat + " D1 U");
            theOracle.Same(aNV, aD1V, theWhat + " D1 V");
        }

        using (var aNP = new N.gp_Pnt(0, 0, 0))
        using (var aNU = new N.gp_Vec(0, 0, 0))
        using (var aNV = new N.gp_Vec(0, 0, 0))
        using (var aNUU = new N.gp_Vec(0, 0, 0))
        using (var aNVV = new N.gp_Vec(0, 0, 0))
        using (var aNUV = new N.gp_Vec(0, 0, 0))
        {
            theN.D2(aU, aV, aNP, aNU, aNV, aNUU, aNVV, aNUV);
            theP.D2(aU, aV, out gp_Pnt aP, out gp_Vec aD1U, out gp_Vec aD1V, out gp_Vec aD2U, out gp_Vec aD2V, out gp_Vec aD2UV);
            theOracle.Same(aNP, aP, theWhat + " D2 P");
            theOracle.Same(aNU, aD1U, theWhat + " D2 U");
            theOracle.Same(aNV, aD1V, theWhat + " D2 V");
            theOracle.Same(aNUU, aD2U, theWhat + " D2 UU");
            theOracle.Same(aNVV, aD2V, theWhat + " D2 VV");
            theOracle.Same(aNUV, aD2UV, theWhat + " D2 UV");
        }

        using (var aNP = new N.gp_Pnt(0, 0, 0))
        using (var aNU = new N.gp_Vec(0, 0, 0))
        using (var aNV = new N.gp_Vec(0, 0, 0))
        using (var aNUU = new N.gp_Vec(0, 0, 0))
        using (var aNVV = new N.gp_Vec(0, 0, 0))
        using (var aNUV = new N.gp_Vec(0, 0, 0))
        using (var aNUUU = new N.gp_Vec(0, 0, 0))
        using (var aNVVV = new N.gp_Vec(0, 0, 0))
        using (var aNUUV = new N.gp_Vec(0, 0, 0))
        using (var aNUVV = new N.gp_Vec(0, 0, 0))
        {
            theN.D3(aU, aV, aNP, aNU, aNV, aNUU, aNVV, aNUV, aNUUU, aNVVV, aNUUV, aNUVV);
            theP.D3(aU, aV, out gp_Pnt aP, out gp_Vec a1U, out gp_Vec a1V, out gp_Vec a2U, out gp_Vec a2V, out gp_Vec a2UV,
                    out gp_Vec a3U, out gp_Vec a3V, out gp_Vec a3UUV, out gp_Vec a3UVV);
            theOracle.Same(aNP, aP, theWhat + " D3 P");
            theOracle.Same(aNU, a1U, theWhat + " D3 U");
            theOracle.Same(aNV, a1V, theWhat + " D3 V");
            theOracle.Same(aNUU, a2U, theWhat + " D3 UU");
            theOracle.Same(aNVV, a2V, theWhat + " D3 VV");
            theOracle.Same(aNUV, a2UV, theWhat + " D3 UV");
            theOracle.Same(aNUUU, a3U, theWhat + " D3 UUU");
            theOracle.Same(aNVVV, a3V, theWhat + " D3 VVV");
            theOracle.Same(aNUUV, a3UUV, theWhat + " D3 UUV");
            theOracle.Same(aNUVV, a3UVV, theWhat + " D3 UVV");
        }

        for (int nu = 0; nu <= 2; nu++)
        {
            for (int nv = 0; nv <= 2; nv++)
            {
                if (nu + nv > 0)
                {
                    using N.gp_Vec aNDN = theN.EvalDN(aU, aV, nu, nv);
                    theOracle.Same(aNDN, theP.DN(aU, aV, nu, nv), $"{theWhat} DN({nu},{nv})");
                }
            }
        }
    }

    /// <summary>Runs an edit on both sides: both throw, or neither does.</summary>
    private static bool Both(Action theNative, Action thePort, string theWhat)
    {
        Exception? aNErr = null, aPErr = null;
        try
        {
            theNative();
        }
        catch (Exception e)
        {
            aNErr = e;
        }

        try
        {
            thePort();
        }
        catch (Exception e)
        {
            aPErr = e;
        }

        Assert.True((aNErr is null) == (aPErr is null), $"{theWhat}: native {aNErr?.Message ?? "ok"}, port {aPErr?.ToString() ?? "ok"}");
        return aNErr is null;
    }

    [Fact]
    public void Construction_and_evaluation()
    {
        var anOracle = new Oracle(output);
        for (int i = 0; i < Cases; i++)
        {
            (N.Geom_BSplineSurface aN, Geom_BSplineSurface aP) = RandomSurface(anOracle);
            using (aN)
            {
                CompareData(anOracle, aN, aP, "new");
                CompareEvaluation(anOracle, aN, aP, "new");

                // Reversed parameters, periodic normalisation, periods
                aP.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                double aU = anOracle.Real(aU1 - 3, aU2 + 3), aV = anOracle.Real(aV1 - 3, aV2 + 3);
                anOracle.Same(aN.UReversedParameter(aU), aP.UReversedParameter(aU), "UReversedParameter");
                anOracle.Same(aN.VReversedParameter(aV), aP.VReversedParameter(aV), "VReversedParameter");
                double aNU = aU, aNV = aV, aPU = aU, aPV = aV;
                aN.PeriodicNormalization(ref aNU, ref aNV);
                aP.PeriodicNormalization(ref aPU, ref aPV);
                anOracle.Same(aNU, aPU, "PeriodicNormalization U");
                anOracle.Same(aNV, aPV, "PeriodicNormalization V");
                if (aP.IsUPeriodic())
                {
                    anOracle.Same(aN.UPeriod(), aP.UPeriod(), "UPeriod");
                }

                // Locate
                foreach (bool aRep in new[] { false, true })
                {
                    double aTol = anOracle.Int(0, 1) == 0 ? 0.0 : 1e-3;
                    int aNI1 = 0, aNI2 = 0;
                    aN.LocateU(aU, aTol, ref aNI1, ref aNI2, aRep);
                    aP.LocateU(aU, aTol, out int aI1, out int aI2, aRep);
                    anOracle.Same(aNI1, aI1, "LocateU I1");
                    anOracle.Same(aNI2, aI2, "LocateU I2");
                    double aKnotV = aP.VKnot(anOracle.Int(1, aP.NbVKnots()));
                    aN.LocateV(aKnotV, aTol, ref aNI1, ref aNI2, aRep);
                    aP.LocateV(aKnotV, aTol, out aI1, out aI2, aRep);
                    anOracle.Same(aNI1, aI1, "LocateV I1");
                    anOracle.Same(aNI2, aI2, "LocateV I2");
                }

                // Resolution
                double aNUTol = 0, aNVTol = 0;
                double aTol3d = anOracle.Real(1e-7, 1e-2);
                aN.Resolution(aTol3d, ref aNUTol, ref aNVTol);
                aP.Resolution(aTol3d, out double aUTol, out double aVTol);
                anOracle.SameRelative(aNUTol, aUTol, "Resolution U");
                anOracle.SameRelative(aNVTol, aVTol, "Resolution V");

                // Local evaluation on a random span
                int aUK1 = anOracle.Int(1, aP.NbUKnots() - 1), aVK1 = anOracle.Int(1, aP.NbVKnots() - 1);
                using (N.gp_Pnt aNL = aN.LocalValue(aU, aV, aUK1, aUK1 + 1, aVK1, aVK1 + 1))
                {
                    anOracle.Same(aNL, aP.LocalValue(aU, aV, aUK1, aUK1 + 1, aVK1, aVK1 + 1), "LocalValue");
                }

                using (N.gp_Vec aNL = aN.LocalDN(aU, aV, aUK1, aUK1 + 1, aVK1, aVK1 + 1, 1, 1))
                {
                    anOracle.Same(aNL, aP.LocalDN(aU, aV, aUK1, aUK1 + 1, aVK1, aVK1 + 1, 1, 1), "LocalDN");
                }
            }
        }

        anOracle.Report("Geom_BSplineSurface construction and evaluation");
    }

    [Fact]
    public void Iso_curves()
    {
        var anOracle = new Oracle(output, 20261029 + 200);
        for (int i = 0; i < Cases; i++)
        {
            (N.Geom_BSplineSurface aN, Geom_BSplineSurface aP) = RandomSurface(anOracle);
            using (aN)
            {
                GeomIsoCurveCompare.SameIsos(anOracle, aN, aP, "B-spline surface");

                // With and without CheckRational
                aP.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                double U = anOracle.Real(aU1, aU2), V = anOracle.Real(aV1, aV2);
                foreach (bool aCheck in new[] { true, false })
                {
                    using (N.Geom_Curve aNIso = aN.UIso(U, aCheck)!)
                    {
                        GeomIsoCurveCompare.SameCurve(anOracle, aNIso, aP.UIso(U, aCheck), $"B-spline surface UIso({U}, {aCheck})");
                    }
                    using (N.Geom_Curve aNIso = aN.VIso(V, aCheck)!)
                    {
                        GeomIsoCurveCompare.SameCurve(anOracle, aNIso, aP.VIso(V, aCheck), $"B-spline surface VIso({V}, {aCheck})");
                    }
                }
            }
        }
        anOracle.Report("Geom_BSplineSurface UIso and VIso");
    }

    [Fact]
    public void Edits()
    {
        var anOracle = new Oracle(output);
        int[] aDone = new int[20];
        for (int i = 0; i < Cases * 3; i++)
        {
            (N.Geom_BSplineSurface aN, Geom_BSplineSurface aP) = RandomSurface(anOracle);
            using (aN)
            {
                for (int aStep = 0; aStep < 3; aStep++)
                {
                    int anEdit = anOracle.Int(0, 19);
                    if (Edit(anOracle, aN, aP, anEdit, out string aWhat))
                    {
                        aDone[anEdit]++;
                    }

                    CompareData(anOracle, aN, aP, aWhat);
                    CompareEvaluation(anOracle, aN, aP, aWhat);
                }
            }
        }

        output.WriteLine("Edits done: " + string.Join(", ", aDone));
        anOracle.Report("Geom_BSplineSurface edits");
    }

    /// <summary>Applies the same random edit natively and in the port; false if both threw.</summary>
    private static bool Edit(Oracle theOracle, N.Geom_BSplineSurface theN, Geom_BSplineSurface theP, int theEdit, out string theWhat)
    {
        theP.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        switch (theEdit)
        {
            case 0:
            case 1:
            {
                bool anIsU = theEdit == 0;
                theWhat = anIsU ? "InsertUKnots" : "InsertVKnots";
                int aDeg = anIsU ? theP.UDegree() : theP.VDegree();
                int aNb = theOracle.Int(1, 3);
                var aKnots = new NCollection_Array1<double>(1, aNb);
                var aMults = new NCollection_Array1<int>(1, aNb);
                double aFirst = anIsU ? aU1 : aV1, aLast = anIsU ? aU2 : aV2;
                for (int k = 1; k <= aNb; k++)
                {
                    // Increasing, sometimes on an existing knot
                    aKnots[k] = theOracle.Int(0, 3) == 0
                                    ? (anIsU ? theP.UKnot(theOracle.Int(1, theP.NbUKnots())) : theP.VKnot(theOracle.Int(1, theP.NbVKnots())))
                                    : aFirst + (aLast - aFirst) * (k - 1 + theOracle.Real(0.05, 0.95)) / aNb;
                    aMults[k] = theOracle.Int(1, aDeg);
                }

                // keep the knots increasing
                for (int k = 2; k <= aNb; k++)
                {
                    if (aKnots[k] <= aKnots[k - 1])
                    {
                        aKnots[k] = aKnots[k - 1] + 1e-3;
                    }
                }

                bool anAdd = theOracle.Int(0, 3) != 0;
                double aTol = theOracle.Int(0, 1) == 0 ? 0.0 : 1e-6;
                using N.TColStd_Array1OfReal aNKnots = Oracle.ToNative(aKnots);
                using N.TColStd_Array1OfInteger aNMults = Oracle.ToNative(aMults);
                return anIsU ? Both(() => theN.InsertUKnots(aNKnots, aNMults, aTol, anAdd), () => theP.InsertUKnots(aKnots, aMults, aTol, anAdd), theWhat)
                             : Both(() => theN.InsertVKnots(aNKnots, aNMults, aTol, anAdd), () => theP.InsertVKnots(aKnots, aMults, aTol, anAdd), theWhat);
            }
            case 2:
            case 3:
            {
                bool anIsU = theEdit == 2;
                theWhat = anIsU ? "RemoveUKnot" : "RemoveVKnot";
                int aFirst = anIsU ? theP.FirstUKnotIndex() : theP.FirstVKnotIndex();
                int aLast = anIsU ? theP.LastUKnotIndex() : theP.LastVKnotIndex();
                // Inner knots only: OCCT accepts the end knots of a periodic surface, and removing one corrupts the
                // knots (occt-issues.md)
                if (aLast - aFirst < 2)
                {
                    return false;
                }

                int anIndex = theOracle.Int(aFirst + 1, aLast - 1);
                int aMult = anIsU ? theP.UMultiplicity(anIndex) : theP.VMultiplicity(anIndex);
                int aM = theOracle.Int(0, Math.Max(0, aMult - 1));
                double aTol = theOracle.Int(0, 1) == 0 ? 1e-3 : 100.0;
                bool aNRes = false, aPRes = false;
                bool anOk = anIsU ? Both(() => aNRes = theN.RemoveUKnot(anIndex, aM, aTol), () => aPRes = theP.RemoveUKnot(anIndex, aM, aTol), theWhat)
                                  : Both(() => aNRes = theN.RemoveVKnot(anIndex, aM, aTol), () => aPRes = theP.RemoveVKnot(anIndex, aM, aTol), theWhat);
                theOracle.Same(aNRes, aPRes, theWhat + " result");
                return anOk;
            }
            case 4:
            {
                theWhat = "IncreaseDegree";
                int aUD = theP.UDegree() + theOracle.Int(0, 2), aVD = theP.VDegree() + theOracle.Int(0, 2);
                if ((theP.IsUPeriodic() && aUD != theP.UDegree()) || (theP.IsVPeriodic() && aVD != theP.VDegree()))
                {
                    // Periodic IncreaseDegree is broken in OCCT (decision 91): not compared
                    return false;
                }

                return Both(() => theN.IncreaseDegree(aUD, aVD), () => theP.IncreaseDegree(aUD, aVD), theWhat);
            }
            case 5:
            case 6:
            {
                theWhat = theEdit == 5 ? "Segment" : "CheckAndSegment";
                double aSU1 = theOracle.Real(aU1, aU2), aSU2 = theOracle.Real(aU1, aU2);
                double aSV1 = theOracle.Real(aV1, aV2), aSV2 = theOracle.Real(aV1, aV2);
                (aSU1, aSU2) = (Math.Min(aSU1, aSU2), Math.Max(aSU1, aSU2));
                (aSV1, aSV2) = (Math.Min(aSV1, aSV2), Math.Max(aSV1, aSV2));
                if (aSU2 - aSU1 < 0.1 || aSV2 - aSV1 < 0.1)
                {
                    if (theEdit == 6)
                    {
                        // whole range in one direction or both
                        aSU1 = aU1;
                        aSU2 = aU2;
                    }
                    else
                    {
                        return false;
                    }
                }

                if (aSV2 - aSV1 < 0.1)
                {
                    aSV1 = aV1;
                    aSV2 = aV2;
                }

                // CheckAndSegment over the whole range of a periodic direction breaks the surface (occt-issues.md)
                if (theEdit == 6 && ((theP.IsUPeriodic() && aSU1 == aU1 && aSU2 == aU2) || (theP.IsVPeriodic() && aSV1 == aV1 && aSV2 == aV2)))
                {
                    return false;
                }

                return theEdit == 5 ? Both(() => theN.Segment(aSU1, aSU2, aSV1, aSV2), () => theP.Segment(aSU1, aSU2, aSV1, aSV2), theWhat)
                                    : Both(() => theN.CheckAndSegment(aSU1, aSU2, aSV1, aSV2), () => theP.CheckAndSegment(aSU1, aSU2, aSV1, aSV2), theWhat);
            }
            case 7:
            {
                theWhat = "SetPole";
                int ii = theOracle.Int(1, theP.NbUPoles()), jj = theOracle.Int(1, theP.NbVPoles());
                gp_Pnt aPnt = theOracle.Pnt();
                using N.gp_Pnt aNPnt = Oracle.ToNative(aPnt);
                if (theOracle.Int(0, 1) == 0)
                {
                    return Both(() => theN.SetPole(ii, jj, aNPnt), () => theP.SetPole(ii, jj, aPnt), theWhat);
                }

                double aW = theOracle.Real(0.5, 2);
                return Both(() => theN.SetPole(ii, jj, aNPnt, aW), () => theP.SetPole(ii, jj, aPnt, aW), theWhat);
            }
            case 8:
            {
                theWhat = "SetWeight";
                int ii = theOracle.Int(1, theP.NbUPoles()), jj = theOracle.Int(1, theP.NbVPoles());
                double aW = theOracle.Int(0, 3) == 0 ? 1.0 : theOracle.Real(0.5, 2);
                return Both(() => theN.SetWeight(ii, jj, aW), () => theP.SetWeight(ii, jj, aW), theWhat);
            }
            case 9:
            {
                theWhat = "SetWeightRow/Col";
                bool aRow = theOracle.Int(0, 1) == 0;
                int aNb = aRow ? theP.NbVPoles() : theP.NbUPoles();
                NCollection_Array1<double> aWs = theOracle.Weights(aNb, true);
                using N.TColStd_Array1OfReal aNWs = Oracle.ToNative(aWs);
                int anIndex = theOracle.Int(1, aRow ? theP.NbUPoles() : theP.NbVPoles());
                return aRow ? Both(() => theN.SetWeightRow(anIndex, aNWs), () => theP.SetWeightRow(anIndex, aWs), theWhat)
                            : Both(() => theN.SetWeightCol(anIndex, aNWs), () => theP.SetWeightCol(anIndex, aWs), theWhat);
            }
            case 10:
            {
                theWhat = "SetPoleRow/Col";
                bool aRow = theOracle.Int(0, 1) == 0;
                int aNb = aRow ? theP.NbVPoles() : theP.NbUPoles();
                var aPs = new NCollection_Array1<gp_Pnt>(1, aNb);
                for (int k = 1; k <= aNb; k++)
                {
                    aPs[k] = theOracle.Pnt();
                }

                using N.TColgp_Array1OfPnt aNPs = Oracle.ToNative(aPs);
                int anIndex = theOracle.Int(1, aRow ? theP.NbUPoles() : theP.NbVPoles());
                return aRow ? Both(() => theN.SetPoleRow(anIndex, aNPs), () => theP.SetPoleRow(anIndex, aPs), theWhat)
                            : Both(() => theN.SetPoleCol(anIndex, aNPs), () => theP.SetPoleCol(anIndex, aPs), theWhat);
            }
            case 11:
            {
                theWhat = theP.IsUPeriodic() ? "SetUNotPeriodic" : "SetUPeriodic";
                return theP.IsUPeriodic() ? Both(() => theN.SetUNotPeriodic(), () => theP.SetUNotPeriodic(), theWhat)
                                          : Both(() => theN.SetUPeriodic(), () => theP.SetUPeriodic(), theWhat);
            }
            case 12:
            {
                theWhat = theP.IsVPeriodic() ? "SetVNotPeriodic" : "SetVPeriodic";
                return theP.IsVPeriodic() ? Both(() => theN.SetVNotPeriodic(), () => theP.SetVNotPeriodic(), theWhat)
                                          : Both(() => theN.SetVPeriodic(), () => theP.SetVPeriodic(), theWhat);
            }
            case 13:
            {
                theWhat = "SetUOrigin/SetVOrigin";
                if (theP.IsUPeriodic())
                {
                    int anIndex = theOracle.Int(theP.FirstUKnotIndex(), theP.LastUKnotIndex());
                    return Both(() => theN.SetUOrigin(anIndex), () => theP.SetUOrigin(anIndex), theWhat);
                }

                if (theP.IsVPeriodic())
                {
                    int anIndex = theOracle.Int(theP.FirstVKnotIndex(), theP.LastVKnotIndex());
                    return Both(() => theN.SetVOrigin(anIndex), () => theP.SetVOrigin(anIndex), theWhat);
                }

                return false;
            }
            case 14:
            {
                theWhat = "ExchangeUV";
                return Both(() => theN.ExchangeUV(), () => theP.ExchangeUV(), theWhat);
            }
            case 15:
            {
                theWhat = "MovePoint";
                // Not on periodic surfaces, where OCCT reads poles past the array (occt-issues.md), and with the whole
                // pole grid as window: a window away from the point reads past the basis matrix (occt-issues.md)
                if (theP.IsUPeriodic() || theP.IsVPeriodic())
                {
                    return false;
                }

                double aU = theOracle.Real(aU1, aU2), aV = theOracle.Real(aV1, aV2);
                gp_Pnt aTarget = theP.Value(aU, aV).Translated(new gp_Vec(theOracle.Real(-1, 1), theOracle.Real(-1, 1), theOracle.Real(-1, 1)));
                int aUI1 = 1, aUI2 = theP.NbUPoles();
                int aVI1 = 1, aVI2 = theP.NbVPoles();
                using N.gp_Pnt aNTarget = Oracle.ToNative(aTarget);
                int aNF1 = 0, aNL1 = 0, aNF2 = 0, aNL2 = 0, aF1 = 0, aL1 = 0, aF2 = 0, aL2 = 0;
                bool anOk = Both(() => theN.MovePoint(aU, aV, aNTarget, aUI1, aUI2, aVI1, aVI2, ref aNF1, ref aNL1, ref aNF2, ref aNL2),
                                 () => theP.MovePoint(aU, aV, aTarget, aUI1, aUI2, aVI1, aVI2, out aF1, out aL1, out aF2, out aL2),
                                 theWhat);
                theOracle.Same(aNF1, aF1, "MovePoint UFirst");
                theOracle.Same(aNL1, aL1, "MovePoint ULast");
                theOracle.Same(aNF2, aF2, "MovePoint VFirst");
                theOracle.Same(aNL2, aL2, "MovePoint VLast");
                return anOk;
            }
            case 16:
            {
                theWhat = "UReverse";
                return Both(() => theN.UReverse(), () => theP.UReverse(), theWhat);
            }
            case 17:
            {
                theWhat = "VReverse";
                return Both(() => theN.VReverse(), () => theP.VReverse(), theWhat);
            }
            case 18:
            {
                theWhat = "Transform";
                (N.gp_Trsf aNT, gp_Trsf aT) = RandomTrsf(theOracle);
                using (aNT)
                {
                    return Both(() => theN.Transform(aNT), () => theP.Transform(aT), theWhat);
                }
            }
            default:
            {
                theWhat = "Increase/IncrementMultiplicity";
                bool anIsU = theOracle.Int(0, 1) == 0;
                int aNbKnots = anIsU ? theP.NbUKnots() : theP.NbVKnots();
                int aDeg = anIsU ? theP.UDegree() : theP.VDegree();
                int anI1 = theOracle.Int(1, aNbKnots), anI2 = theOracle.Int(anI1, aNbKnots);
                int aKind = theOracle.Int(0, 2);
                // Never lower a multiplicity (an insertion of a negative multiplicity, unchecked by OCCT)
                int aMaxMult = 0;
                for (int k = anI1; k <= anI2; k++)
                {
                    aMaxMult = Math.Max(aMaxMult, anIsU ? theP.UMultiplicity(k) : theP.VMultiplicity(k));
                }

                if (aMaxMult >= aDeg)
                {
                    return false;
                }

                int aM = theOracle.Int(aMaxMult, aDeg);
                if (aKind == 0)
                {
                    return anIsU ? Both(() => theN.IncreaseUMultiplicity(anI1, aM), () => theP.IncreaseUMultiplicity(anI1, aM), theWhat)
                                 : Both(() => theN.IncreaseVMultiplicity(anI1, aM), () => theP.IncreaseVMultiplicity(anI1, aM), theWhat);
                }

                if (aKind == 1)
                {
                    return anIsU ? Both(() => theN.IncreaseUMultiplicity(anI1, anI2, aM), () => theP.IncreaseUMultiplicity(anI1, anI2, aM), theWhat)
                                 : Both(() => theN.IncreaseVMultiplicity(anI1, anI2, aM), () => theP.IncreaseVMultiplicity(anI1, anI2, aM), theWhat);
                }

                return anIsU ? Both(() => theN.IncrementUMultiplicity(anI1, anI2, 1), () => theP.IncrementUMultiplicity(anI1, anI2, 1), theWhat)
                             : Both(() => theN.IncrementVMultiplicity(anI1, anI2, 1), () => theP.IncrementVMultiplicity(anI1, anI2, 1), theWhat);
            }
        }
    }
}
#endif
