// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using Xunit.Abstractions;
using static OcctNet.Oracle.Tests.AdaptorOracleInputs;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// GeomConvert against native OCCT: CurveToBSplineCurve and SurfaceToBSplineSurface on every curve and surface kind
/// (whole and trimmed; offsets through GeomConvert_ApproxCurve and GeomConvert_ApproxSurface), SplitBSplineCurve and
/// SplitBSplineSurface (every overload), GeomConvert_ApproxCurve (both constructors) and
/// GeomConvert_CompCurveToBSplineCurve. ConcatG1, ConcatC1, the C0BSplineTo* functions and the other classes have no
/// binding and are unit-tested only.
/// </summary>
public class GeomConvertOracleTests(ITestOutputHelper output)
{
    internal static void SameBSplineCurve(Oracle theO, N.Geom_BSplineCurve theN, BSplineCurve3D theP, string theWhat)
    {
        theO.Same(theN.Degree(), theP.Degree(), theWhat + " degree");
        theO.Same(theN.IsPeriodic(), theP.IsPeriodic, theWhat + " periodic");
        theO.Same(theN.IsRational(), theP.IsRational, theWhat + " rational");
        theO.Same(theN.NbPoles(), theP.PoleCount, theWhat + " NbPoles");
        theO.Same(theN.NbKnots(), theP.KnotCount, theWhat + " NbKnots");
        if (theN.NbPoles() != theP.PoleCount || theN.NbKnots() != theP.KnotCount)
        {
            return;
        }

        for (int i = 1; i <= theP.PoleCount; i++)
        {
            using N.gp_Pnt aNP = theN.Pole(i);
            theO.Same(aNP, theP.Pole(i), theWhat + " pole");
            theO.Same(theN.Weight(i), theP.Weight(i), theWhat + " weight");
        }

        for (int k = 1; k <= theP.KnotCount; k++)
        {
            theO.Same(theN.Knot(k), theP.Knot(k), theWhat + " knot");
            theO.Same(theN.Multiplicity(k), theP.Multiplicity(k), theWhat + " mult");
        }
    }

    internal static void SameBSplineSurface(Oracle theO, N.Geom_BSplineSurface theN, BSplineSurface theP, string theWhat)
    {
        theO.Same(theN.UDegree(), theP.UDegree(), theWhat + " UDegree");
        theO.Same(theN.VDegree(), theP.VDegree(), theWhat + " VDegree");
        theO.Same(theN.IsUPeriodic(), theP.IsUPeriodic, theWhat + " UPeriodic");
        theO.Same(theN.IsVPeriodic(), theP.IsVPeriodic, theWhat + " VPeriodic");
        theO.Same(theN.IsURational(), theP.IsURational, theWhat + " URational");
        theO.Same(theN.IsVRational(), theP.IsVRational, theWhat + " VRational");
        theO.Same(theN.NbUPoles(), theP.UPoleCount, theWhat + " NbUPoles");
        theO.Same(theN.NbVPoles(), theP.VPoleCount, theWhat + " NbVPoles");
        theO.Same(theN.NbUKnots(), theP.UKnotCount, theWhat + " NbUKnots");
        theO.Same(theN.NbVKnots(), theP.VKnotCount, theWhat + " NbVKnots");
        if (theN.NbUPoles() != theP.UPoleCount || theN.NbVPoles() != theP.VPoleCount
            || theN.NbUKnots() != theP.UKnotCount || theN.NbVKnots() != theP.VKnotCount)
        {
            return;
        }

        for (int i = 1; i <= theP.UPoleCount; i++)
        {
            for (int j = 1; j <= theP.VPoleCount; j++)
            {
                using N.gp_Pnt aNP = theN.Pole(i, j);
                theO.Same(aNP, theP.Pole(i, j), theWhat + " pole");
                theO.Same(theN.Weight(i, j), theP.Weight(i, j), theWhat + " weight");
            }
        }

        for (int k = 1; k <= theP.UKnotCount; k++)
        {
            theO.Same(theN.UKnot(k), theP.UKnot(k), theWhat + " UKnot");
            theO.Same(theN.UMultiplicity(k), theP.UMultiplicity(k), theWhat + " UMult");
        }

        for (int k = 1; k <= theP.VKnotCount; k++)
        {
            theO.Same(theN.VKnot(k), theP.VKnot(k), theWhat + " VKnot");
            theO.Same(theN.VMultiplicity(k), theP.VMultiplicity(k), theWhat + " VMult");
        }
    }

    /// <summary>A random curve, trimmed half the time (always, when it is unbounded and not periodic).</summary>
    private static (N.Geom_Curve Native, Curve3D Port, string Kind) RandomInputCurve(Oracle theO, int theMaxKind = 9)
    {
        int aKind = theO.Int(0, theMaxKind);
        (N.Geom_Curve aN, Curve3D aP, double aRange) = RandomCurve(theO, aKind);
        double aF = aP.FirstParameter(), aL = aP.LastParameter();
        bool isUnbounded = global::Kerf.Engine.Foundation.Precision.IsInfinite(aF) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aL);
        if (aKind == 9 || (!isUnbounded && theO.Bool()))
        {
            return (aN, aP, CurveKinds[aKind]);
        }

        using (aN)
        {
            double aU1, aU2;
            if (isUnbounded)
            {
                aU1 = theO.Real(-aRange, aRange);
                aU2 = aU1 + theO.Real(0.05, aRange);
            }
            else if (aP.IsPeriodic)
            {
                aU1 = theO.Real(aF - (aL - aF), aL);
                aU2 = aU1 + (theO.Int(0, 3) == 0 ? aL - aF : theO.Real(0.02, 1.0) * (aL - aF));
            }
            else
            {
                aU1 = theO.Real(aF, aF + 0.45 * (aL - aF));
                aU2 = theO.Real(aF + 0.55 * (aL - aF), aL);
            }

            bool aSense = theO.Int(0, 4) != 0;
            return (new N.Geom_TrimmedCurve(aN, aU1, aU2, aSense), new TrimmedCurve3D(aP, aU1, aU2, aSense),
                    "trimmed " + CurveKinds[aKind]);
        }
    }

    [Fact]
    public void Curves_to_BSpline_curves()
    {
        var anOracle = new Oracle(output);
        int aNbThrows = 0;
        foreach (int i in Tape.Budget(1500))
        {
            (N.Geom_Curve aNC, Curve3D aPC, string aKind) = RandomInputCurve(anOracle);
            using N.Geom_Curve aNCOwned = aNC;
            var aParam = (ParameterisationType)anOracle.Int(0, 7);
            N.Geom_BSplineCurve? aN = null;
            BSplineCurve3D? aP = null;
            Exception? aNErr = Catch(() => aN = N.GeomConvert.CurveToBSplineCurve(aNC, (N.Convert_ParameterisationType)(int)aParam));
            Exception? aPErr = Catch(() => aP = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aPC, aParam));
            string aWhat = $"{aKind} {aParam}";
            if (!SameOutcome(aNErr, aPErr, aWhat))
            {
                aNbThrows++;
                continue;
            }

            using N.Geom_BSplineCurve aNOwned = aN!;
            Assert.False(aP!.IsFrozen);
            SameBSplineCurve(anOracle, aN!, aP, aWhat);
        }

        output.WriteLine($"Native and port threw for the same {aNbThrows} curves");
        anOracle.Report("GeomConvert::CurveToBSplineCurve");
    }

    [Fact]
    public void Long_RationalC1_arcs_to_BSpline_curves()
    {
        // Arcs of 6 radians or more with Convert_RationalC1: OCCT converts two halves and joins them with
        // GeomConvert_CompCurveToBSplineCurve. Whole turns (exactly 2 Pi) included.
        var anOracle = new Oracle(output);
        foreach (int i in Tape.Budget(400))
        {
            (N.Geom_Curve aNB, Curve3D aPB, _) = RandomCurve(anOracle, anOracle.Int(1, 2));
            using N.Geom_Curve aNBOwned = aNB;
            double aU1 = anOracle.Real(-10, 10);
            double aU2 = aU1 + (anOracle.Int(0, 4) == 0 ? 2 * Math.PI : anOracle.Real(6.0, 2 * Math.PI));
            bool aSense = anOracle.Bool();
            using var aNTrimmed = new N.Geom_TrimmedCurve(aNB, aU1, aU2, aSense);
            var aTrimmed = new TrimmedCurve3D(aPB, aU1, aU2, aSense);
            using N.Geom_BSplineCurve aNative =
                N.GeomConvert.CurveToBSplineCurve(aNTrimmed, N.Convert_ParameterisationType.Convert_RationalC1)!;
            BSplineCurve3D aPort = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(aTrimmed, ParameterisationType.RationalC1);
            SameBSplineCurve(anOracle, aNative, aPort, $"{Kerf.Engine.Foundation.OcctTypeNames.Of(aPB.GetType())} {aU2 - aU1}");
        }

        anOracle.Report("GeomConvert::CurveToBSplineCurve, long RationalC1 arcs");
    }

    [Fact]
    public void Split_BSpline_curves()
    {
        var anOracle = new Oracle(output);
        int aNbThrows = 0;
        foreach (int i in Tape.Budget(800))
        {
            (N.Geom_BSplineCurve aNC, BSplineCurve3D aPC) = RandomBSplineCurve(anOracle);
            using N.Geom_BSplineCurve aNCOwned = aNC;
            bool aSame = anOracle.Bool();
            N.Geom_BSplineCurve? aN = null;
            BSplineCurve3D? aP = null;
            string aWhat;
            Exception? aNErr, aPErr;
            if (anOracle.Bool())
            {
                int aF = aPC.FirstUKnotIndex(), aL = aPC.LastUKnotIndex();
                int aK1 = anOracle.Int(aF - 1, aL), aK2 = anOracle.Int(aF, aL + (anOracle.Int(0, 9) == 0 ? 1 : 0));
                aWhat = $"knots {aK1} {aK2} in [{aF}, {aL}] periodic {aPC.IsPeriodic}";
                aNErr = Catch(() => aN = N.GeomConvert.SplitBSplineCurve(aNC, aK1, aK2, aSame));
                aPErr = Catch(() => aP = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aPC, aK1, aK2, aSame));
            }
            else
            {
                double aF = aPC.FirstParameter(), aL = aPC.LastParameter();
                double aU1 = anOracle.Real(aF, aL), aU2 = anOracle.Real(aF, aL);
                if (Math.Abs(aU2 - aU1) < 1e-3)
                {
                    aU2 = aU1 < 0.5 * (aF + aL) ? aL : aF;
                }

                aWhat = $"parameters {aU1} {aU2} periodic {aPC.IsPeriodic}";
                aNErr = Catch(() => aN = N.GeomConvert.SplitBSplineCurve(aNC, aU1, aU2, 1e-9, aSame));
                aPErr = Catch(() => aP = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineCurve(aPC, aU1, aU2, 1e-9, aSame));
            }

            if (!SameOutcome(aNErr, aPErr, aWhat))
            {
                aNbThrows++;
                continue;
            }

            using N.Geom_BSplineCurve aNOwned = aN!;
            SameBSplineCurve(anOracle, aN!, aP!, aWhat);
        }

        output.WriteLine($"Native and port threw for the same {aNbThrows} splits");
        anOracle.Report("GeomConvert::SplitBSplineCurve");
    }

    [Fact]
    public void Split_BSpline_surfaces()
    {
        var anOracle = new Oracle(output);
        int aNbThrows = 0;
        foreach (int i in Tape.Budget(600))
        {
            (N.Geom_BSplineSurface aNS, BSplineSurface aPS) = RandomBSplineSurface(anOracle, anOracle.Int(0, 2));
            using N.Geom_BSplineSurface aNSOwned = aNS;
            bool aSameU = anOracle.Bool(), aSameV = anOracle.Bool();
            N.Geom_BSplineSurface? aN = null;
            BSplineSurface? aP = null;
            string aWhat;
            Exception? aNErr, aPErr;
            int aUF = aPS.FirstUKnotIndex(), aUL = aPS.LastUKnotIndex(), aVF = aPS.FirstVKnotIndex(), aVL = aPS.LastVKnotIndex();
            aPS.Bounds(out double aU0, out double aU9, out double aV0, out double aV9);
            (double, double) Params(double theF, double theL)
            {
                double a1 = anOracle.Real(theF, theL), a2 = anOracle.Real(theF, theL);
                if (Math.Abs(a2 - a1) < 1e-3)
                {
                    a2 = a1 < 0.5 * (theF + theL) ? theL : theF;
                }

                return (a1, a2);
            }

            switch (anOracle.Int(0, 3))
            {
                case 0:
                {
                    int aK1 = anOracle.Int(aUF, aUL), aK2 = anOracle.Int(aUF, aUL);
                    int aK3 = anOracle.Int(aVF, aVL), aK4 = anOracle.Int(aVF, aVL);
                    aWhat = $"knots U {aK1} {aK2} V {aK3} {aK4}";
                    aNErr = Catch(() => aN = N.GeomConvert.SplitBSplineSurface(aNS, aK1, aK2, aK3, aK4, aSameU, aSameV));
                    aPErr = Catch(() => aP = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineSurface(aPS, aK1, aK2, aK3, aK4, aSameU, aSameV));
                    break;
                }
                case 1:
                {
                    bool isU = anOracle.Bool();
                    int aK1 = isU ? anOracle.Int(aUF, aUL) : anOracle.Int(aVF, aVL);
                    int aK2 = isU ? anOracle.Int(aUF, aUL) : anOracle.Int(aVF, aVL);
                    aWhat = $"knots {(isU ? "U" : "V")} {aK1} {aK2}";
                    aNErr = Catch(() => aN = N.GeomConvert.SplitBSplineSurface(aNS, aK1, aK2, isU, aSameU));
                    aPErr = Catch(() => aP = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineSurface(aPS, aK1, aK2, isU, aSameU));
                    break;
                }
                case 2:
                {
                    (double aU1, double aU2) = Params(aU0, aU9);
                    (double aV1, double aV2) = Params(aV0, aV9);
                    aWhat = $"parameters U {aU1} {aU2} V {aV1} {aV2}";
                    aNErr = Catch(() => aN = N.GeomConvert.SplitBSplineSurface(aNS, aU1, aU2, aV1, aV2, 1e-9, aSameU, aSameV));
                    aPErr = Catch(() => aP = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineSurface(aPS, aU1, aU2, aV1, aV2, 1e-9, aSameU, aSameV));
                    break;
                }
                default:
                {
                    bool isU = anOracle.Bool();
                    (double a1, double a2) = isU ? Params(aU0, aU9) : Params(aV0, aV9);
                    aWhat = $"parameters {(isU ? "U" : "V")} {a1} {a2}";
                    aNErr = Catch(() => aN = N.GeomConvert.SplitBSplineSurface(aNS, a1, a2, isU, 1e-9, aSameU));
                    aPErr = Catch(() => aP = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SplitBSplineSurface(aPS, a1, a2, isU, 1e-9, aSameU));
                    break;
                }
            }

            aWhat += $" periodic {aPS.IsUPeriodic} {aPS.IsVPeriodic}";
            if (!SameOutcome(aNErr, aPErr, aWhat))
            {
                aNbThrows++;
                continue;
            }

            using N.Geom_BSplineSurface aNOwned = aN!;
            SameBSplineSurface(anOracle, aN!, aP!, aWhat);
        }

        output.WriteLine($"Native and port threw for the same {aNbThrows} splits");
        anOracle.Report("GeomConvert::SplitBSplineSurface");
    }

    [Fact]
    public void Surfaces_to_BSpline_surfaces()
    {
        var anOracle = new Oracle(output);
        int aNbThrows = 0;
        var aCounts = new Dictionary<string, int>();
        foreach (int i in Tape.Budget(700))
        {
            int aKind = anOracle.Int(0, 10);
            // offset surfaces without an equivalent surface are approximated (AdvApp2Var): fewer of them
            if (aKind == 9 && anOracle.Int(0, 2) != 0)
            {
                aKind = 10;
            }

            (N.Geom_Surface aNS, Surface aPS) = RandomSurface(anOracle, aKind);
            using N.Geom_Surface aNSOwned = aNS;
            string aWhat = SurfaceKinds[aKind] + " " + NativeSurfaceKind(aNS);
            N.Geom_BSplineSurface? aN = null;
            BSplineSurface? aP = null;
            Exception? aNErr = Catch(() => aN = N.GeomConvert.SurfaceToBSplineSurface(aNS));
            Exception? aPErr = Catch(() => aP = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(aPS));
            if (!SameOutcome(aNErr, aPErr, aWhat))
            {
                aNbThrows++;
                continue;
            }

            aCounts[aWhat] = aCounts.GetValueOrDefault(aWhat) + 1;
            using N.Geom_BSplineSurface aNOwned = aN!;
            Assert.False(aP!.IsFrozen);
            SameBSplineSurface(anOracle, aN!, aP, aWhat);
        }

        foreach (KeyValuePair<string, int> aPair in aCounts.OrderBy(p => p.Key))
        {
            output.WriteLine($"{aPair.Key}: {aPair.Value}");
        }

        output.WriteLine($"Native and port threw for the same {aNbThrows} surfaces");
        anOracle.Report("GeomConvert::SurfaceToBSplineSurface");
    }

    [Fact]
    public void Offset_surfaces_to_BSpline_surfaces()
    {
        // Offsets without an equivalent surface go through GeomConvert_ApproxSurface (AdvApp2Var); trimmed or not
        var anOracle = new Oracle(output);
        int aNbThrows = 0, aNbApprox = 0;
        foreach (int i in Tape.Budget(80))
        {
            (N.Geom_Surface aNS, Surface aPS) = RandomOffsetSurface(anOracle);
            if (anOracle.Bool())
            {
                (N.Geom_Surface? aNT, Surface? aPT) = RandomTrim(anOracle, aNS, aPS);
                aNS.Dispose();
                if (aNT is null)
                {
                    continue;
                }

                (aNS, aPS) = (aNT, aPT!);
            }

            using N.Geom_Surface aNSOwned = aNS;
            Surface aBasis = aPS is TrimmedSurface aTrim ? aTrim.BasisSurface() : aPS;
            bool isApprox = aBasis is OffsetSurface anOffset && anOffset.Surface() is null;
            string aWhat = NativeSurfaceKind(aNS) + (isApprox ? " approximated" : " equivalent");
            N.Geom_BSplineSurface? aN = null;
            BSplineSurface? aP = null;
            Exception? aNErr = Catch(() => aN = N.GeomConvert.SurfaceToBSplineSurface(aNS));
            Exception? aPErr = Catch(() => aP = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(aPS));
            if (!SameOutcome(aNErr, aPErr, aWhat))
            {
                aNbThrows++;
                continue;
            }

            aNbApprox += isApprox ? 1 : 0;
            using N.Geom_BSplineSurface aNOwned = aN!;
            Assert.False(aP!.IsFrozen);
            SameBSplineSurface(anOracle, aN!, aP, aWhat);
        }

        output.WriteLine($"{aNbApprox} approximated; native and port threw for the same {aNbThrows} surfaces");
        Assert.True(aNbApprox > 0); // as recorded, a budgeted sample (decision 1337)
        anOracle.Report("GeomConvert::SurfaceToBSplineSurface of offset surfaces");
    }

    [Fact]
    public void ApproxCurve()
    {
        var anOracle = new Oracle(output);
        int aNbThrows = 0, aNbNoResult = 0, aNbDone = 0;
        var aReasons = new Dictionary<string, int>();
        foreach (int i in Tape.Budget(400))
        {
            (N.Geom_Curve aNC, Curve3D aPC, string aKind) = RandomInputCurve(anOracle);
            using N.Geom_Curve aNCOwned = aNC;
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aPC.FirstParameter()) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aPC.LastParameter()))
            {
                continue;
            }

            double aTol = Math.Pow(10, -anOracle.Real(2, 11));
            int anOrderIndex = anOracle.Int(0, 2);
            Continuity anOrder = anOrderIndex == 0 ? Continuity.C0
                                    : anOrderIndex == 1 ? Continuity.C1 : Continuity.C2;
            // mostly degrees of at least 2 * order + 1 (AdvApprox raises lower ones)
            int aMaxSeg = anOracle.Int(1, 12);
            int aMaxDeg = anOracle.Int(0, 9) == 0 ? anOracle.Int(1, 14) : anOracle.Int(2 * anOrderIndex + 1, 14);
            bool isAdaptor = anOracle.Int(0, 3) == 0;
            double aF = aPC.FirstParameter(), aL = aPC.LastParameter();
            double aU1 = aF, aU2 = aL;
            if (isAdaptor)
            {
                aU1 = anOracle.Real(aF, aF + 0.4 * (aL - aF));
                aU2 = anOracle.Real(aF + 0.6 * (aL - aF), aL);
            }

            string aWhat = $"{aKind} tol {aTol} {anOrder} seg {aMaxSeg} deg {aMaxDeg}{(isAdaptor ? " adaptor" : "")}";
            N.GeomConvert_ApproxCurve? aN = null;
            global::Kerf.Engine.Geometry.Conversion.ApproxCurve? aP = null;
            N.GeomAdaptor_Curve? aNAdaptor = isAdaptor ? new N.GeomAdaptor_Curve(aNC, aU1, aU2) : null;
            Exception? aNErr = Catch(() => aN = isAdaptor
                ? new N.GeomConvert_ApproxCurve(aNAdaptor!, aTol, (N.GeomAbs_Shape)(int)anOrder, aMaxSeg, aMaxDeg)
                : new N.GeomConvert_ApproxCurve(aNC, aTol, (N.GeomAbs_Shape)(int)anOrder, aMaxSeg, aMaxDeg));
            Exception? aPErr = Catch(() => aP = isAdaptor
                ? new global::Kerf.Engine.Geometry.Conversion.ApproxCurve(new CurveAdaptor(aPC, aU1, aU2), aTol, anOrder, aMaxSeg, aMaxDeg)
                : new global::Kerf.Engine.Geometry.Conversion.ApproxCurve(aPC, aTol, anOrder, aMaxSeg, aMaxDeg));
            aNAdaptor?.Dispose();
            if (!SameOutcome(aNErr, aPErr, aWhat))
            {
                aNbThrows++;
                string aReason = aPErr!.GetType().Name + " " + aPErr.Message;
                aReasons[aReason] = aReasons.GetValueOrDefault(aReason) + 1;
                continue;
            }

            using N.GeomConvert_ApproxCurve aNOwned = aN!;
            anOracle.Same(aN!.IsDone(), aP!.IsDone, aWhat + " IsDone");
            anOracle.Same(aN.HasResult(), aP.HasResult, aWhat + " HasResult");
            anOracle.Same(aN.MaxError(), aP.MaxError(), aWhat + " MaxError");
            aNbDone += aP.IsDone ? 1 : 0;
            if (!aP.HasResult)
            {
                aNbNoResult++;
                continue;
            }

            using N.Geom_BSplineCurve aNCurve = aN.Curve()!;
            BSplineCurve3D aPCurve = aP.Curve()!;
            Assert.True(aPCurve.IsFrozen);
            Assert.Same(aPCurve, aP.Curve());
            SameBSplineCurve(anOracle, aNCurve, aPCurve, aWhat);
        }

        output.WriteLine($"Native and port threw for the same {aNbThrows} curves; {aNbNoResult} without result; {aNbDone} done");
        foreach (KeyValuePair<string, int> aPair in aReasons)
        {
            output.WriteLine($"  {aPair.Value} times: {aPair.Key}");
        }

        anOracle.Report("GeomConvert_ApproxCurve");
    }

    [Fact]
    public void CompCurveToBSplineCurve()
    {
        var anOracle = new Oracle(output);
        int aNbAdded = 0, aNbRefused = 0, aNbThrows = 0;
        foreach (int i in Tape.Budget(500))
        {
            // A basis curve cut into consecutive pieces, some reversed, added in order or not
            int aKind = anOracle.Int(0, 7);
            if (aKind == 4 && anOracle.Bool())
            {
                aKind = 6;
            }

            (N.Geom_Curve aNB, Curve3D aPB, double aRange) = RandomCurve(anOracle, aKind);
            using N.Geom_Curve aNBOwned = aNB;
            double aF = aPB.FirstParameter(), aL = aPB.LastParameter();
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aF) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aL) || aPB.IsPeriodic)
            {
                aF = anOracle.Real(-aRange, aRange);
                aL = aF + anOracle.Real(0.2, aPB.IsPeriodic && aKind != 7 ? 2 * Math.PI : aRange);
            }

            int aNbPieces = anOracle.Int(2, 5);
            var aCuts = new double[aNbPieces + 1];
            aCuts[0] = aF;
            aCuts[aNbPieces] = aL;
            for (int k = 1; k < aNbPieces; k++)
            {
                aCuts[k] = aF + (aL - aF) * (k + anOracle.Real(-0.3, 0.3)) / aNbPieces;
            }

            var aParam = (ParameterisationType)anOracle.Int(0, 7);
            var aNPieces = new List<N.Geom_TrimmedCurve>();
            var aPPieces = new List<TrimmedCurve3D>();
            for (int k = 0; k < aNbPieces; k++)
            {
                bool aSense = anOracle.Int(0, 3) != 0;
                double aU1 = aSense ? aCuts[k] : aCuts[k + 1], aU2 = aSense ? aCuts[k + 1] : aCuts[k];
                aNPieces.Add(new N.Geom_TrimmedCurve(aNB, aU1, aU2, aSense));
                aPPieces.Add(new TrimmedCurve3D(aPB, aU1, aU2, aSense));
            }

            int aStart = anOracle.Int(0, aNbPieces - 1);
            string aWhat = $"{CurveKinds[aKind]} {aNbPieces} pieces from {aStart} {aParam}";
            N.GeomConvert_CompCurveToBSplineCurve? aN = null;
            global::Kerf.Engine.Geometry.Conversion.CompCurveToBSplineCurve? aP = null;
            Exception? aNErr = Catch(() => aN = new N.GeomConvert_CompCurveToBSplineCurve(aNPieces[aStart], (N.Convert_ParameterisationType)(int)aParam));
            Exception? aPErr = Catch(() => aP = new global::Kerf.Engine.Geometry.Conversion.CompCurveToBSplineCurve(aPPieces[aStart], aParam));
            if (!SameOutcome(aNErr, aPErr, aWhat))
            {
                aNbThrows++;
                aNPieces.ForEach(p => p.Dispose());
                continue;
            }

            using N.GeomConvert_CompCurveToBSplineCurve aNOwned = aN!;
            bool isHandedOut = false;
            for (int aStep = 0; aStep < 2 * aNbPieces; aStep++)
            {
                int k = anOracle.Int(0, aNbPieces - 1);
                double aTol = Math.Pow(10, -anOracle.Real(3, 8));
                bool anAfter = anOracle.Bool(), aWithRatio = anOracle.Int(0, 3) != 0;
                int aMinM = anOracle.Int(0, 2);
                bool aNRes = false, aPRes = false;
                aNErr = Catch(() => aNRes = aN!.Add(aNPieces[k], aTol, anAfter, aWithRatio, aMinM));
                aPErr = Catch(() => aPRes = aP!.Add(aPPieces[k], aTol, anAfter, aWithRatio, aMinM));
                string aStepWhat = $"{aWhat} add {k} tol {aTol} after {anAfter} ratio {aWithRatio} MinM {aMinM}";
                if (!SameOutcome(aNErr, aPErr, aStepWhat))
                {
                    aNbThrows++;
                    break;
                }

                anOracle.Same(aNRes, aPRes, aStepWhat + " Add");
                if (aPRes)
                {
                    aNbAdded++;
                }
                else
                {
                    aNbRefused++;
                }

                if (anOracle.Int(0, 2) == 0 || aStep == 2 * aNbPieces - 1)
                {
                    // BSplineCurve() hands out the current curve frozen; a later Add must not change it
                    BSplineCurve3D aPCurve = aP!.BSplineCurve()!;
                    Assert.True(aPCurve.IsFrozen);
                    using N.Geom_BSplineCurve aNCurve = aN!.BSplineCurve()!;
                    SameBSplineCurve(anOracle, aNCurve, aPCurve, aStepWhat);
                    isHandedOut = true;
                }
            }

            Assert.True(isHandedOut || aNbThrows > 0);
            aNPieces.ForEach(p => p.Dispose());
        }

        output.WriteLine($"{aNbAdded} curves added, {aNbRefused} refused; native and port threw for the same {aNbThrows}");
        anOracle.Report("GeomConvert_CompCurveToBSplineCurve");
    }
}
