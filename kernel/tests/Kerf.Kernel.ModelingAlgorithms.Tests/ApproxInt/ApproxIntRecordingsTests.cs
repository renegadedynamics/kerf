// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Approximation;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Surfaces;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using System.Globalization;
using System.Text;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// GeomInt_WLApprox, BRepApprox_Approx and ApproxInt_KnotTools on native walking lines, against native OCCT's
/// results recorded on the same lines (<see cref="ApproxIntRecordings"/>, decision 680): done or not, the tolerances
/// reached, and the B-spline's degree, knots, multiplicities and poles, every double bit for bit.
/// </summary>
public class ApproxIntRecordingsTests(ITestOutputHelper output)
{
    private static string Bits(double x) => x.ToString("R", CultureInfo.InvariantCulture);

    // Compares the port's approximation with the recorded one; returns the differences (empty: bit-identical) and
    // the largest deviation of a value.
    private static List<string> Compare(ApproxIntRecordings.Approx theRec, bool theDone, double theTol3d,
                                        double theTol2d, int theNbMultiCurves, MultiBSpCurve? theCurve,
                                        ref double theMaxDev)
    {
        List<string> aDiffs = [];
        void Value(string theWhat, double theNative, double thePort, ref double theDev)
        {
            if (BitConverter.DoubleToInt64Bits(theNative) != BitConverter.DoubleToInt64Bits(thePort))
            {
                aDiffs.Add($"{theWhat}: native {Bits(theNative)}, port {Bits(thePort)}");
                theDev = Math.Max(theDev, Math.Abs(theNative - thePort));
            }
        }
        if (theDone != theRec.IsDone)
        {
            aDiffs.Add($"IsDone: native {theRec.IsDone}, port {theDone}");
        }
        Value("TolReached3d", theRec.TolReached3d, theTol3d, ref theMaxDev);
        Value("TolReached2d", theRec.TolReached2d, theTol2d, ref theMaxDev);
        if (theNbMultiCurves != theRec.NbMultiCurves)
        {
            aDiffs.Add($"NbMultiCurves: native {theRec.NbMultiCurves}, port {theNbMultiCurves}");
        }
        if (!theRec.IsDone || !theDone || theCurve is null)
        {
            return aDiffs;
        }
        if (theCurve.NbCurves() != theRec.NbCurves || theCurve.Degree() != theRec.Degree
            || theCurve.NbPoles() != theRec.NbPoles || theCurve.Knots().Length() != theRec.Knots.Length)
        {
            aDiffs.Add($"MBSP: native {theRec.NbCurves} {theRec.Degree} {theRec.NbPoles} {theRec.Knots.Length}, port "
                       + $"{theCurve.NbCurves()} {theCurve.Degree()} {theCurve.NbPoles()} {theCurve.Knots().Length()}");
            return aDiffs;
        }
        NCollection_Array1<double> aKnots = theCurve.Knots();
        NCollection_Array1<int> aMults = theCurve.Multiplicities();
        for (int i = 0; i < theRec.Knots.Length; i++)
        {
            Value($"knot {i + 1}", theRec.Knots[i], aKnots[aKnots.Lower() + i], ref theMaxDev);
            if (aMults[aMults.Lower() + i] != theRec.Mults[i])
            {
                aDiffs.Add($"mult {i + 1}: native {theRec.Mults[i]}, port {aMults[aMults.Lower() + i]}");
            }
        }
        int np = theRec.NbPoles;
        for (int c = 1; c <= theRec.NbCurves; c++)
        {
            (int aDim, double[] aCoords) = theRec.Poles[c - 1];
            if (theCurve.Dimension(c) != aDim)
            {
                aDiffs.Add($"curve {c}: dimension native {aDim}, port {theCurve.Dimension(c)}");
                continue;
            }
            if (aDim == 3)
            {
                NCollection_Array1<Point3D> P = new(1, np);
                theCurve.Curve(c, P);
                for (int i = 1; i <= np; i++)
                {
                    Value($"curve {c} pole {i} X", aCoords[3 * (i - 1)], P[i].X, ref theMaxDev);
                    Value($"curve {c} pole {i} Y", aCoords[3 * (i - 1) + 1], P[i].Y, ref theMaxDev);
                    Value($"curve {c} pole {i} Z", aCoords[3 * (i - 1) + 2], P[i].Z, ref theMaxDev);
                }
            }
            else
            {
                NCollection_Array1<Point2D> P = new(1, np);
                theCurve.Curve(c, P);
                for (int i = 1; i <= np; i++)
                {
                    Value($"curve {c} pole {i} X", aCoords[2 * (i - 1)], P[i].X, ref theMaxDev);
                    Value($"curve {c} pole {i} Y", aCoords[2 * (i - 1) + 1], P[i].Y, ref theMaxDev);
                }
            }
        }
        return aDiffs;
    }

    private static void Run<TS, TT, TL>(Approx<TS, TT, TL> theApp, ApproxIntRecordings.Approx theRec,
                                       TS theS1, TS theS2, TL theLine)
        where TT : IApproxSurfaceTool<TS>
        where TL : class, IWLine<TL>
    {
        theApp.SetParameters(theRec.Tol3d, theRec.Tol2d, theRec.DegMin, theRec.DegMax, theRec.NbIter, theRec.NbPntMax,
                             theRec.Tangency, theRec.ParType);
        if (theRec.OnSurfaces)
        {
            theApp.Perform(theS1, theS2, theLine, theRec.Xyz, theRec.U1V1, theRec.U2V2, theRec.IMin, theRec.IMax);
        }
        else
        {
            theApp.Perform(theLine, theRec.Xyz, theRec.U1V1, theRec.U2V2, theRec.IMin, theRec.IMax);
        }
    }

    [Fact]
    public void Approximations_MatchNativeRecordings()
    {
        ApproxIntRecordings.Recording aRec = ApproxIntRecordings.Load();
        int aNbApprox = 0, aNbIdentical = 0, aNbValues = 0;
        double aMaxDev = 0.0;
        StringBuilder aReport = new();
        foreach (ApproxIntRecordings.Case aCase in aRec.Cases)
        {
            Surface aS1 = ApproxIntRecordings.ReadSurface(aCase.Surface1);
            Surface aS2 = ApproxIntRecordings.ReadSurface(aCase.Surface2);
            SurfaceAdaptor aHS1 = new(aS1), aHS2 = new(aS2);
            FaceSurfaceAdaptor aBS1 = new(new MakeFace(aS1, 1e-7).Face());
            FaceSurfaceAdaptor aBS2 = new(new MakeFace(aS2, 1e-7).Face());
            foreach (ApproxIntRecordings.Line aLine in aCase.Lines)
            {
                WLine aWL = ApproxIntRecordings.MakeWLine(aLine);
                Assert.Equal(aLine.ParType,
                             KnotTools.DefineParType(aWL, 1, aWL.NbPnts(), aLine.PtXyz, aLine.PtU1V1,
                                                               aLine.PtU2V2));
                foreach (ApproxIntRecordings.Approx anA in aLine.Approxes)
                {
                    bool aDone;
                    double aTol3d, aTol2d;
                    int aNbMC;
                    MultiBSpCurve? aCurve = null;
                    if (anA.Algo == "GeomInt")
                    {
                        WLApprox anApp = new();
                        Run(anApp, anA, aHS1, aHS2, aWL);
                        (aDone, aTol3d, aTol2d, aNbMC) = (anApp.IsDone(), anApp.TolReached3d(), anApp.TolReached2d(),
                                                           anApp.MultiCurveCount);
                        if (aDone)
                        {
                            aCurve = anApp.Value(1);
                        }
                    }
                    else
                    {
                        ApproxLine aBL = anA.OnSurfaces
                                                        ? new ApproxLine(aWL.Curve(), false)
                                                        : ApproxIntRecordings.MakeCurvesLine(aLine);
                        ApproximationApprox anApp = new();
                        Run(anApp, anA, aBS1, aBS2, aBL);
                        (aDone, aTol3d, aTol2d, aNbMC) = (anApp.IsDone(), anApp.TolReached3d(), anApp.TolReached2d(),
                                                           anApp.MultiCurveCount);
                        if (aDone)
                        {
                            aCurve = anApp.Value(1);
                        }
                    }
                    aNbApprox++;
                    aNbValues += 2 + anA.Knots.Length + anA.Poles.Sum(p => p.Coords.Length);
                    double aDev = 0.0;
                    List<string> aDiffs = Compare(anA, aDone, aTol3d, aTol2d, aNbMC, aCurve, ref aDev);
                    aMaxDev = Math.Max(aMaxDev, aDev);
                    if (aDiffs.Count == 0)
                    {
                        aNbIdentical++;
                    }
                    else
                    {
                        aReport.AppendLine($"{aCase.Name} line {aLine.Index} {anA}: {aDiffs.Count} differences, "
                                           + $"largest {aDev:R}; first: {aDiffs[0]}");
                    }
                }
            }
        }
        output.WriteLine($"{aNbApprox} approximations, {aNbIdentical} bit-identical ({aNbValues} values); "
                         + $"largest deviation {aMaxDev:R}");
        output.WriteLine(aReport.ToString());
        Assert.True(aNbApprox > 100, "too few recorded approximations");
        Assert.True(aReport.Length == 0, aReport.ToString());
    }

    [Fact]
    public void KnotTools_BuildKnots_MatchesNativeRecordings()
    {
        ApproxIntRecordings.Recording aRec = ApproxIntRecordings.Load();
        ApproxIntRecordings.Line aLine = aRec.Cases.First(c => c.Name == "bspl_bspl").Lines[0];
        WLine aWL = ApproxIntRecordings.MakeWLine(aLine);
        int n = aWL.NbPnts();
        TheMultiLineOfWLApprox ml = new(aWL, 1, 2, true, true, 0, 0, 0, 0, 0, 0, 0, true, 1, n);
        math_Vector pars = new(1, n);
        WLApprox.Parameters(ml, 1, n, global::Kerf.Geometry.ParametrizationType.ChordLength, pars);
        NCollection_Array1<Point3D> p3 = new(1, n);
        NCollection_Array1<Point2D> pa = new(1, n), pb = new(1, n);
        for (int i = 1; i <= n; i++)
        {
            aWL.Point(i).Parameters(out double u1, out double v1, out double u2, out double v2);
            p3[i] = aWL.Point(i).Value();
            pa[i] = new Point2D(u1, v1);
            pb[i] = new Point2D(u2, v2);
        }
        Assert.Equal(4, aRec.KnotTools.Count);
        foreach (ApproxIntRecordings.KnotToolsRun aRun in aRec.KnotTools)
        {
            List<int> aKnots = new();
            KnotTools.BuildKnots(p3, pa, pb, pars, true, true, true, aRun.MinNbPnts, aKnots);
            Assert.Equal(aRun.Knots, aKnots.ToArray());
        }
    }
}
