// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Geom_BezierSurface against native OCCT: random surfaces (degrees 1 to 8 in each direction, rational half the time,
/// sometimes closed), compared field by field (poles, weights, rationality, closure, knots of the equivalent
/// B-spline) and by evaluation (D0 to D3, DN, resolution), then again after random edits applied to both.
/// </summary>
public class Geom_BezierSurfaceOracleTests(ITestOutputHelper output)
{
    private static readonly int Cases = Tape.Cases(200, 6);

    private static (N.Geom_BezierSurface Native, Geom_BezierSurface Port) RandomSurface(Oracle theOracle)
    {
        int aNbU = theOracle.Int(2, 9), aNbV = theOracle.Int(2, 9);
        bool aRational = theOracle.Int(0, 1) == 1;
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

        if (theOracle.Int(0, 4) == 0)
        {
            // closed in U: the last row repeats the first
            for (int j = 1; j <= aNbV; j++)
            {
                aPoles[aNbU, j] = aPoles[1, j];
            }
        }

        using N.TColgp_Array2OfPnt aNPoles = Oracle.ToNative(aPoles);
        if (!aRational)
        {
            return (new N.Geom_BezierSurface(aNPoles), new Geom_BezierSurface(aPoles));
        }

        using N.TColStd_Array2OfReal aNWeights = Oracle.ToNative(aWeights);
        return (new N.Geom_BezierSurface(aNPoles, aNWeights), new Geom_BezierSurface(aPoles, aWeights));
    }

    private static void Compare(Oracle theOracle, N.Geom_BezierSurface theN, Geom_BezierSurface theP, string theWhat)
    {
        theOracle.Same(theN.NbUPoles(), theP.NbUPoles(), theWhat + " NbUPoles");
        theOracle.Same(theN.NbVPoles(), theP.NbVPoles(), theWhat + " NbVPoles");
        theOracle.Same(theN.UDegree(), theP.UDegree(), theWhat + " UDegree");
        theOracle.Same(theN.VDegree(), theP.VDegree(), theWhat + " VDegree");
        theOracle.Same(theN.IsURational(), theP.IsURational(), theWhat + " IsURational");
        theOracle.Same(theN.IsVRational(), theP.IsVRational(), theWhat + " IsVRational");
        theOracle.Same(theN.IsUClosed(), theP.IsUClosed(), theWhat + " IsUClosed");
        theOracle.Same(theN.IsVClosed(), theP.IsVClosed(), theWhat + " IsVClosed");
        theOracle.Same(theN.IsUPeriodic(), theP.IsUPeriodic(), theWhat + " IsUPeriodic");
        theOracle.Same((int)theN.Continuity(), (int)theP.Continuity(), theWhat + " Continuity");
        for (int i = 1; i <= theP.NbUPoles(); i++)
        {
            for (int j = 1; j <= theP.NbVPoles(); j++)
            {
                using N.gp_Pnt aNP = theN.Pole(i, j);
                theOracle.Same(aNP, theP.Pole(i, j), theWhat + " Pole");
                theOracle.Same(theN.Weight(i, j), theP.Weight(i, j), theWhat + " Weight");
            }
        }

        using (N.TColStd_Array1OfInteger aNM = theN.UMultiplicities())
        {
            theOracle.Same(aNM.Value(aNM.Lower()), theP.UMultiplicities()[0], theWhat + " UMultiplicities");
        }

        using (N.TColStd_Array1OfReal aNK = theN.VKnotSequence())
        {
            PackedList<double> aK = theP.VKnotSequence();
            theOracle.Same(aNK.Length(), aK.Count, theWhat + " VKnotSequence length");
            for (int k = 0; k < aK.Count; k++)
            {
                theOracle.Same(aNK.Value(aNK.Lower() + k), aK[k], theWhat + " VKnotSequence");
            }
        }

        const int n = 4;
        for (int ju = 0; ju <= n; ju++)
        {
            for (int jv = 0; jv <= n; jv++)
            {
                double aU = (double)ju / n, aV = (double)jv / n;
                if (ju == 1)
                {
                    aU = theOracle.Real(-0.2, 1.2);
                }

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
        }

        double aNUTol = 0, aNVTol = 0;
        theN.Resolution(1e-3, ref aNUTol, ref aNVTol);
        theP.Resolution(1e-3, out double aUTol, out double aVTol);
        theOracle.SameRelative(aNUTol, aUTol, theWhat + " Resolution U");
        theOracle.SameRelative(aNVTol, aVTol, theWhat + " Resolution V");
    }

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
    public void Construction_evaluation_and_edits()
    {
        var anOracle = new Oracle(output);
        int[] aDone = new int[14];
        for (int i = 0; i < Cases; i++)
        {
            (N.Geom_BezierSurface aN, Geom_BezierSurface aP) = RandomSurface(anOracle);
            using (aN)
            {
                Compare(anOracle, aN, aP, "new");
                anOracle.Same(aN.UReversedParameter(0.3), aP.UReversedParameter(0.3), "UReversedParameter");
                for (int aStep = 0; aStep < 4; aStep++)
                {
                    int anEdit = anOracle.Int(0, 13);
                    if (Edit(anOracle, aN, aP, anEdit, out string aWhat))
                    {
                        aDone[anEdit]++;
                    }

                    Compare(anOracle, aN, aP, aWhat);
                }
            }
        }

        output.WriteLine("Edits done: " + string.Join(", ", aDone));
        anOracle.Report("Geom_BezierSurface");
    }

    [Fact]
    public void Iso_curves()
    {
        var anOracle = new Oracle(output, 20261029 + 300);
        for (int i = 0; i < Cases; i++)
        {
            (N.Geom_BezierSurface aN, Geom_BezierSurface aP) = RandomSurface(anOracle);
            using (aN)
            {
                GeomIsoCurveCompare.SameIsos(anOracle, aN, aP, "Bezier surface");
            }
        }
        anOracle.Report("Geom_BezierSurface UIso and VIso");
    }

    private static bool Edit(Oracle theOracle, N.Geom_BezierSurface theN, Geom_BezierSurface theP, int theEdit, out string theWhat)
    {
        int aNbU = theP.NbUPoles(), aNbV = theP.NbVPoles();
        switch (theEdit)
        {
            case 0:
            case 1:
            {
                // insert a column (0) or a row (1), after or before, with or without weights
                bool aCol = theEdit == 0;
                bool anAfter = theOracle.Int(0, 1) == 0;
                bool aWithWeights = theOracle.Int(0, 1) == 0;
                theWhat = (aCol ? "InsertPoleCol" : "InsertPoleRow") + (anAfter ? "After" : "Before") + (aWithWeights ? "W" : "");
                if ((aCol ? aNbV : aNbU) >= 12)
                {
                    return false;
                }

                int aNb = aCol ? aNbU : aNbV;
                var aPs = new NCollection_Array1<gp_Pnt>(1, aNb);
                for (int k = 1; k <= aNb; k++)
                {
                    aPs[k] = theOracle.Pnt();
                }

                NCollection_Array1<double> aWs = theOracle.Weights(aNb, true);
                int anIndex = anAfter ? theOracle.Int(1, aCol ? aNbV : aNbU) : theOracle.Int(2, aCol ? aNbV : aNbU);
                using N.TColgp_Array1OfPnt aNPs = Oracle.ToNative(aPs);
                using N.TColStd_Array1OfReal aNWs = Oracle.ToNative(aWs);
                return (aCol, anAfter, aWithWeights) switch
                {
                    (true, true, false) => Both(() => theN.InsertPoleColAfter(anIndex, aNPs), () => theP.InsertPoleColAfter(anIndex, aPs), theWhat),
                    (true, true, true) => Both(() => theN.InsertPoleColAfter(anIndex, aNPs, aNWs), () => theP.InsertPoleColAfter(anIndex, aPs, aWs), theWhat),
                    (true, false, false) => Both(() => theN.InsertPoleColBefore(anIndex, aNPs), () => theP.InsertPoleColBefore(anIndex, aPs), theWhat),
                    (true, false, true) => Both(() => theN.InsertPoleColBefore(anIndex, aNPs, aNWs), () => theP.InsertPoleColBefore(anIndex, aPs, aWs), theWhat),
                    (false, true, false) => Both(() => theN.InsertPoleRowAfter(anIndex, aNPs), () => theP.InsertPoleRowAfter(anIndex, aPs), theWhat),
                    (false, true, true) => Both(() => theN.InsertPoleRowAfter(anIndex, aNPs, aNWs), () => theP.InsertPoleRowAfter(anIndex, aPs, aWs), theWhat),
                    (false, false, false) => Both(() => theN.InsertPoleRowBefore(anIndex, aNPs), () => theP.InsertPoleRowBefore(anIndex, aPs), theWhat),
                    _ => Both(() => theN.InsertPoleRowBefore(anIndex, aNPs, aNWs), () => theP.InsertPoleRowBefore(anIndex, aPs, aWs), theWhat),
                };
            }
            case 2:
            {
                theWhat = "RemovePoleCol";
                int anIndex = theOracle.Int(1, aNbV);
                return Both(() => theN.RemovePoleCol(anIndex), () => theP.RemovePoleCol(anIndex), theWhat);
            }
            case 3:
            {
                theWhat = "RemovePoleRow";
                int anIndex = theOracle.Int(1, aNbU);
                return Both(() => theN.RemovePoleRow(anIndex), () => theP.RemovePoleRow(anIndex), theWhat);
            }
            case 4:
            {
                theWhat = "Increase";
                int aUD = Math.Min(theP.UDegree() + theOracle.Int(0, 3), 12), aVD = Math.Min(theP.VDegree() + theOracle.Int(0, 3), 12);
                return Both(() => theN.Increase(aUD, aVD), () => theP.Increase(aUD, aVD), theWhat);
            }
            case 5:
            {
                theWhat = "Segment";
                double aU1 = theOracle.Real(-0.2, 1.2), aU2 = theOracle.Real(-0.2, 1.2);
                double aV1 = theOracle.Real(-0.2, 1.2), aV2 = theOracle.Real(-0.2, 1.2);
                return Both(() => theN.Segment(aU1, aU2, aV1, aV2), () => theP.Segment(aU1, aU2, aV1, aV2), theWhat);
            }
            case 6:
            {
                theWhat = "SetPole";
                int ii = theOracle.Int(1, aNbU), jj = theOracle.Int(1, aNbV);
                gp_Pnt aPnt = theOracle.Pnt();
                using N.gp_Pnt aNPnt = Oracle.ToNative(aPnt);
                if (theOracle.Int(0, 1) == 0)
                {
                    return Both(() => theN.SetPole(ii, jj, aNPnt), () => theP.SetPole(ii, jj, aPnt), theWhat);
                }

                double aW = theOracle.Int(0, 3) == 0 ? 1.0 : theOracle.Real(0.5, 2);
                return Both(() => theN.SetPole(ii, jj, aNPnt, aW), () => theP.SetPole(ii, jj, aPnt, aW), theWhat);
            }
            case 7:
            {
                theWhat = "SetWeight";
                int ii = theOracle.Int(1, aNbU), jj = theOracle.Int(1, aNbV);
                double aW = theOracle.Int(0, 3) == 0 ? 1.0 : theOracle.Real(0.5, 2);
                return Both(() => theN.SetWeight(ii, jj, aW), () => theP.SetWeight(ii, jj, aW), theWhat);
            }
            case 8:
            {
                bool aRow = theOracle.Int(0, 1) == 0;
                theWhat = aRow ? "SetWeightRow" : "SetWeightCol";
                NCollection_Array1<double> aWs = theOracle.Weights(aRow ? aNbV : aNbU, true);
                using N.TColStd_Array1OfReal aNWs = Oracle.ToNative(aWs);
                int anIndex = theOracle.Int(1, aRow ? aNbU : aNbV);
                return aRow ? Both(() => theN.SetWeightRow(anIndex, aNWs), () => theP.SetWeightRow(anIndex, aWs), theWhat)
                            : Both(() => theN.SetWeightCol(anIndex, aNWs), () => theP.SetWeightCol(anIndex, aWs), theWhat);
            }
            case 9:
            {
                bool aRow = theOracle.Int(0, 1) == 0;
                bool aWithWeights = theOracle.Int(0, 1) == 0;
                theWhat = (aRow ? "SetPoleRow" : "SetPoleCol") + (aWithWeights ? "W" : "");
                int aNb = aRow ? aNbV : aNbU;
                var aPs = new NCollection_Array1<gp_Pnt>(1, aNb);
                for (int k = 1; k <= aNb; k++)
                {
                    aPs[k] = theOracle.Pnt();
                }

                NCollection_Array1<double> aWs = theOracle.Weights(aNb, true);
                using N.TColgp_Array1OfPnt aNPs = Oracle.ToNative(aPs);
                using N.TColStd_Array1OfReal aNWs = Oracle.ToNative(aWs);
                int anIndex = theOracle.Int(1, aRow ? aNbU : aNbV);
                return (aRow, aWithWeights) switch
                {
                    (true, false) => Both(() => theN.SetPoleRow(anIndex, aNPs), () => theP.SetPoleRow(anIndex, aPs), theWhat),
                    (true, true) => Both(() => theN.SetPoleRow(anIndex, aNPs, aNWs), () => theP.SetPoleRow(anIndex, aPs, aWs), theWhat),
                    (false, false) => Both(() => theN.SetPoleCol(anIndex, aNPs), () => theP.SetPoleCol(anIndex, aPs), theWhat),
                    _ => Both(() => theN.SetPoleCol(anIndex, aNPs, aNWs), () => theP.SetPoleCol(anIndex, aPs, aWs), theWhat),
                };
            }
            case 10:
            {
                theWhat = "ExchangeUV";
                return Both(() => theN.ExchangeUV(), () => theP.ExchangeUV(), theWhat);
            }
            case 11:
            {
                theWhat = "UReverse";
                return Both(() => theN.UReverse(), () => theP.UReverse(), theWhat);
            }
            case 12:
            {
                theWhat = "VReverse";
                return Both(() => theN.VReverse(), () => theP.VReverse(), theWhat);
            }
            default:
            {
                theWhat = "Transform";
                gp_Vec aV = theOracle.Vec();
                gp_Pnt aC = theOracle.Pnt();
                double aS = theOracle.Real(0.2, 3);
                using var aNV = Oracle.ToNative(aV);
                using var aNC = Oracle.ToNative(aC);
                using var aNT = new N.gp_Trsf();
                gp_Trsf aT;
                if (theOracle.Int(0, 1) == 0)
                {
                    aNT.SetTranslation(aNV);
                    aT = gp_Trsf.Translation(aV);
                }
                else
                {
                    aNT.SetScale(aNC, aS);
                    aT = gp_Trsf.Scale(aC, aS);
                }

                return Both(() => theN.Transform(aNT), () => theP.Transform(aT), theWhat);
            }
        }
    }
}
