#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using N = Build123d.Occt;
using static OcctNet.Oracle.Tests.AdaptorOracleInputs;
using static OcctNet.Oracle.Tests.Geom.GeomOracleInputs;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Geom_OffsetSurface against native OCCT: random offsets of every kind of basis (elementary surfaces with their
/// equivalent surfaces, degenerate and flipped radii included; swept, Bezier, B-spline, trimmed and offset
/// surfaces), and offsets at singular points (collapsed rows of B-spline and Bezier poles, with their osculating
/// surfaces; sphere-like surfaces of revolution and cone apexes, through CSLib's higher-order normal); compared:
/// construction (both throw or neither), offset, continuities, bounds, closure, periodicity, the equivalent
/// surface, evaluation (D0 to D3 and DN), reversal, transformation (with TransformParameters and
/// ParametricTransformation), the offset setter, iso-curves (AdvApprox approximations), trimmed offsets, and
/// GeomAdaptor_Surface on the same surfaces (its offset evaluation runs through the basis adaptor).
/// </summary>
public class Geom_OffsetSurfaceOracleTests(ITestOutputHelper output)
{
    private const int Cases = 150;

    private static void SameProperties(Oracle theO, N.Geom_OffsetSurface theN, Geom_OffsetSurface theP, string theWhat)
    {
        theO.Same(theN.Offset(), theP.Offset(), theWhat + " Offset");
        theO.Same((int)theN.GetBasisSurfContinuity(), (int)theP.GetBasisSurfContinuity(), theWhat + " GetBasisSurfContinuity");
        theO.Same(theN.HasEvalRepresentation(), theP.HasEvalRepresentation(), theWhat + " HasEvalRepresentation");
        double aNU1 = 0, aNU2 = 0, aNV1 = 0, aNV2 = 0;
        theN.Bounds(ref aNU1, ref aNU2, ref aNV1, ref aNV2);
        theP.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        theO.Same(aNU1, aU1, theWhat + " U1");
        theO.Same(aNU2, aU2, theWhat + " U2");
        theO.Same(aNV1, aV1, theWhat + " V1");
        theO.Same(aNV2, aV2, theWhat + " V2");
        theO.Same(theN.IsUClosed(), theP.IsUClosed(), theWhat + " IsUClosed");
        theO.Same(theN.IsVClosed(), theP.IsVClosed(), theWhat + " IsVClosed");
        theO.Same(theN.IsUPeriodic(), theP.IsUPeriodic(), theWhat + " IsUPeriodic");
        theO.Same(theN.IsVPeriodic(), theP.IsVPeriodic(), theWhat + " IsVPeriodic");
        if (theP.IsUPeriodic())
        {
            theO.Same(theN.UPeriod(), theP.UPeriod(), theWhat + " UPeriod");
        }
        if (theP.IsVPeriodic())
        {
            theO.Same(theN.VPeriod(), theP.VPeriod(), theWhat + " VPeriod");
        }
        theO.Same((int)theN.Continuity(), (int)theP.Continuity(), theWhat + " Continuity");
        for (int n = 0; n <= 3; n++)
        {
            theO.Same(theN.IsCNu(n), theP.IsCNu(n), theWhat + " IsCNu");
            theO.Same(theN.IsCNv(n), theP.IsCNv(n), theWhat + " IsCNv");
        }
        double U = Param(theO, aU1, aU2), V = Param(theO, aV1, aV2);
        theO.Same(theN.UReversedParameter(U), theP.UReversedParameter(U), theWhat + " UReversedParameter");
        theO.Same(theN.VReversedParameter(V), theP.VReversedParameter(V), theWhat + " VReversedParameter");

        // the equivalent surface: none on both sides, or the same surface
        using N.Geom_Surface? aNEq = theN.Surface();
        Geom_Surface? anEq = theP.Surface();
        Assert.True((aNEq is null) == (anEq is null), $"{theWhat} Surface(): native {(aNEq is null ? "null" : "a surface")}, port {anEq?.GetType().Name ?? "null"}");
        if (aNEq is not null)
        {
            Assert.Equal(NativeSurfaceKind(aNEq), anEq!.GetType().Name);
            SameSurfaceEval(theO, aNEq, anEq, U, V, 2, theWhat + " Surface()");
        }

        using N.Geom_Surface aNB = theN.BasisSurface()!;
        Geom_Surface aB = theP.BasisSurface();
        Assert.True(aB.IsFrozen(), theWhat + " BasisSurface() frozen");
        SameSurfaceEval(theO, aNB, aB, U, V, 2, theWhat + " BasisSurface()");
    }

    /// <summary>Evaluation at random parameters, at the bounds and corners, and theExtra ones.</summary>
    private static void SameEvaluation(Oracle theO, N.Geom_Surface theN, Geom_Surface theP, string theWhat, int theCount = 5,
                                       IEnumerable<(double U, double V)>? theExtra = null)
    {
        theP.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        var aParams = new List<(double, double)>();
        for (int k = 0; k < theCount; k++)
        {
            aParams.Add((Param(theO, aU1, aU2), Param(theO, aV1, aV2)));
        }
        if (!Precision.IsInfinite(aU1) && !Precision.IsInfinite(aU2) && !Precision.IsInfinite(aV1) && !Precision.IsInfinite(aV2))
        {
            aParams.Add((aU1, aV1));
            aParams.Add((aU2, aV2));
            aParams.Add((aU1, Param(theO, aV1, aV2)));
            aParams.Add((Param(theO, aU1, aU2), aV2));
        }
        if (theExtra is not null)
        {
            aParams.AddRange(theExtra);
        }
        foreach ((double U, double V) in aParams)
        {
            SameSurfaceEval(theO, theN, theP, U, V, 3, theWhat);
        }
    }

    [Fact]
    public void Offsets_of_every_basis()
    {
        var anOracle = new Oracle(output, 20261121);
        for (int i = 0; i < Cases; i++)
        {
            (N.Geom_Surface aNS, Geom_Surface aPS) = RandomOffsetSurface(anOracle);
            using (aNS)
            {
                var aN = N.Geom_OffsetSurface.DownCast(aNS)!;
                var aP = (Geom_OffsetSurface)aPS;
                using N.Geom_Surface aNBasisKind = aN.BasisSurface()!;
                string aWhat = "offset of " + aP.BasisSurface().GetType().Name;
                SameProperties(anOracle, aN, aP, aWhat);
                SameEvaluation(anOracle, aN, aP, aWhat);

                // reversal: OCCT drops the equivalent-surface representation
                foreach (bool aU in new[] { true, false })
                {
                    using var aNR = N.Geom_OffsetSurface.DownCast(aN.Copy())!;
                    if (aU)
                    {
                        aNR.UReverse();
                    }
                    else
                    {
                        aNR.VReverse();
                    }
                    var aPR = (Geom_OffsetSurface)(aU ? aP.UReversed() : aP.VReversed());
                    SameProperties(anOracle, aNR, aPR, aWhat + (aU ? " UReversed" : " VReversed"));
                    SameEvaluation(anOracle, aNR, aPR, aWhat + (aU ? " UReversed" : " VReversed"), 3);
                }

                // transformation; TransformParameters transforms twice where there is an equivalent surface (OCCT)
                (N.gp_Trsf aNT, gp_Trsf aT) = RandomTrsf(anOracle);
                using (aNT)
                {
                    aP.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                    double U = Param(anOracle, aU1, aU2), V = Param(anOracle, aV1, aV2);
                    double aNU = U, aNV = V, aPU = U, aPV = V;
                    aN.TransformParameters(ref aNU, ref aNV, aNT);
                    aP.TransformParameters(ref aPU, ref aPV, aT);
                    anOracle.Same(aNU, aPU, aWhat + " TransformParameters U");
                    anOracle.Same(aNV, aPV, aWhat + " TransformParameters V");
                    using (N.gp_GTrsf2d aNG = aN.ParametricTransformation(aNT))
                    {
                        gp_GTrsf2d aPG = aP.ParametricTransformation(aT);
                        for (int r = 1; r <= 2; r++)
                        {
                            for (int c = 1; c <= 3; c++)
                            {
                                anOracle.Same(aNG.Value(r, c), aPG.Value(r, c), $"{aWhat} ParametricTransformation({r}, {c})");
                            }
                        }
                    }
                    using var aNTr = N.Geom_OffsetSurface.DownCast(aN.Copy())!;
                    aNTr.Transform(aNT);
                    var aPTr = (Geom_OffsetSurface)aP.Transformed(aT);
                    SameProperties(anOracle, aNTr, aPTr, aWhat + " transformed");
                    SameEvaluation(anOracle, aNTr, aPTr, aWhat + " transformed", 3);
                }

                // the offset setter, through its twin: the equivalent surface is recomputed
                double anOffset = anOracle.Real(-3, 3);
                using var aNO = N.Geom_OffsetSurface.DownCast(aN.Copy())!;
                aNO.SetOffsetValue(anOffset);
                Geom_OffsetSurface aPO = aP.WithOffsetValue(anOffset);
                SameProperties(anOracle, aNO, aPO, aWhat + " SetOffsetValue");
                SameEvaluation(anOracle, aNO, aPO, aWhat + " SetOffsetValue", 3);

                // a trimmed offset surface: an offset of a trimmed basis
                (N.Geom_Surface? aNTrim, Geom_Surface? aPTrim) = RandomTrim(anOracle, aN, aP);
                if (aNTrim is not null)
                using (aNTrim)
                {
                    using N.Geom_Surface aNTB = N.Geom_RectangularTrimmedSurface.DownCast(aNTrim)!.BasisSurface()!;
                    Geom_Surface aPTB = ((Geom_RectangularTrimmedSurface)aPTrim!).BasisSurface();
                    Assert.Equal(NativeSurfaceKind(aNTB), aPTB.GetType().Name);
                    SameEvaluation(anOracle, aNTrim, aPTrim!, aWhat + " trimmed", 3);
                }

                // GeomAdaptor_Surface on the offset surface
                using var aNA = new N.GeomAdaptor_Surface(aN);
                var aPA = new GeomAdaptor_Surface(aP);
                aP.Bounds(out double aAU1, out double aAU2, out double aAV1, out double aAV2);
                for (int k = 0; k < 3; k++)
                {
                    SameSurfaceEval(anOracle, aNA, aPA, Param(anOracle, aAU1, aAU2), Param(anOracle, aAV1, aAV2), 3,
                                    aWhat + " GeomAdaptor_Surface");
                }
            }
        }
        anOracle.Report("Geom_OffsetSurface");
    }

    [Fact]
    public void Iso_curves()
    {
        // The equivalent surface's iso-curves, the translated iso-line of an extrusion, or AdvApprox's B-spline
        // approximation of the offset iso-curve.
        var anOracle = new Oracle(output, 20261122);
        for (int i = 0; i < 40; i++)
        {
            (N.Geom_Surface aNS, Geom_Surface aPS) = RandomOffsetSurface(anOracle);
            using (aNS)
            {
                aPS.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                if (Precision.IsInfinite(aU1) || Precision.IsInfinite(aU2) || Precision.IsInfinite(aV1) || Precision.IsInfinite(aV2))
                {
                    // AdvApprox on an unbounded range: skip (it can't approximate)
                    if (((Geom_OffsetSurface)aPS).Surface() is null)
                    {
                        continue;
                    }
                }
                string aWhat = "offset of " + ((Geom_OffsetSurface)aPS).BasisSurface().GetType().Name;
                Geom.GeomIsoCurveCompare.SameIso(anOracle, aNS, aPS, true, Param(anOracle, aU1, aU2), aWhat);
                Geom.GeomIsoCurveCompare.SameIso(anOracle, aNS, aPS, false, Param(anOracle, aV1, aV2), aWhat);
            }
        }
        anOracle.Report("Geom_OffsetSurface iso-curves");
    }

    /// <summary>A B-spline surface whose first column of poles (the iso-curve V = V1) collapses to a point.</summary>
    private static (N.Geom_Surface Native, Geom_Surface Port) CollapsedBSpline(Oracle theO, bool theBezier, bool theBothEnds,
                                                                             bool theUDirection)
    {
        int aUDeg = theO.Int(2, 4), aVDeg = theO.Int(2, 4);
        int aNbU, aNbV;
        NCollection_Array1<double>? aUKnots = null, aVKnots = null;
        NCollection_Array1<int>? aUMults = null, aVMults = null;
        if (theBezier)
        {
            aNbU = aUDeg + 1;
            aNbV = aVDeg + 1;
        }
        else
        {
            theO.BSplineKnots(aUDeg, false, out aUKnots, out aUMults);
            theO.BSplineKnots(aVDeg, false, out aVKnots, out aVMults);
            for (int k = aUMults.Lower() + 1; k < aUMults.Upper(); k++)
            {
                aUMults[k] = Math.Min(aUMults[k], aUDeg - 1);
            }
            for (int k = aVMults.Lower() + 1; k < aVMults.Upper(); k++)
            {
                aVMults[k] = Math.Min(aVMults[k], aVDeg - 1);
            }
            aNbU = Occt.BSplCLib.NbPoles(aUDeg, false, aUMults);
            aNbV = Occt.BSplCLib.NbPoles(aVDeg, false, aVMults);
        }
        var aPoles = new NCollection_Array2<gp_Pnt>(1, aNbU, 1, aNbV);
        // a dome: rings of poles around the apex, opening as the collapsed index grows away from it
        double aH = theO.Real(1, 5);
        gp_Pnt anApex = new gp_Pnt(theO.Real(-1, 1), theO.Real(-1, 1), aH);
        for (int i = 1; i <= aNbU; i++)
        {
            for (int j = 1; j <= aNbV; j++)
            {
                int aRing = theUDirection ? i - 1 : j - 1;
                int anAround = theUDirection ? j - 1 : i - 1;
                int aNbAround = theUDirection ? aNbV : aNbU;
                int aNbRings = theUDirection ? aNbU : aNbV;
                double aR = aRing * 2.0;
                double anAng = 0.2 + 2.5 * anAround / Math.Max(1, aNbAround - 1);
                if (theBothEnds && aRing == aNbRings - 1)
                {
                    aR = 0.0; // collapsed at the other end too
                }
                aPoles[i, j] = aRing == 0 || aR == 0.0
                    ? new gp_Pnt(anApex.X(), anApex.Y(), aRing == 0 ? aH : -aH)
                    : new gp_Pnt(aR * Math.Cos(anAng) + theO.Real(-0.2, 0.2), aR * Math.Sin(anAng) + theO.Real(-0.2, 0.2),
                                 aH - aRing * 0.8 + theO.Real(-0.2, 0.2));
            }
        }
        NCollection_Array2<double>? aWeights = null;
        if (theO.Bool())
        {
            aWeights = new NCollection_Array2<double>(1, aNbU, 1, aNbV);
            for (int i = 1; i <= aNbU; i++)
            {
                for (int j = 1; j <= aNbV; j++)
                {
                    aWeights[i, j] = theO.Real(0.7, 1.3);
                }
            }
        }
        if (theBezier)
        {
            return MakeBezierSurface(aPoles, aWeights);
        }
        return MakeBSplineSurface(aPoles, aWeights, aUKnots!, aVKnots!, aUMults!, aVMults!, aUDeg, aVDeg, false, false);
    }

    /// <summary>A surface of revolution of a B-spline curve whose ends lie on the axis (a sphere-like surface).</summary>
    private static (N.Geom_Surface Native, Geom_Surface Port) SphereLikeRevolution(Oracle theO)
    {
        int aDeg = theO.Int(2, 4);
        int aNbPoles = aDeg + 1 + theO.Int(0, 2);
        int aNbKnots = aNbPoles - aDeg + 1;
        var aKnots = new NCollection_Array1<double>(1, aNbKnots);
        var aMults = new NCollection_Array1<int>(1, aNbKnots);
        for (int k = 1; k <= aNbKnots; k++)
        {
            aKnots[k] = k - 1;
            aMults[k] = k == 1 || k == aNbKnots ? aDeg + 1 : 1;
        }
        var aPoles = new NCollection_Array1<gp_Pnt>(1, aNbPoles);
        for (int k = 1; k <= aNbPoles; k++)
        {
            double t = (k - 1.0) / (aNbPoles - 1);
            double x = k == 1 || k == aNbPoles ? 0.0 : theO.Real(1, 4);
            aPoles[k] = new gp_Pnt(x, 0, 4 * t - 2 + theO.Real(-0.1, 0.1));
        }
        using N.Geom_BSplineCurve aNC = Oracle.ToNativeCurve(aPoles, null, aKnots, aMults, aDeg, false);
        var aPC = new Geom_BSplineCurve(aPoles, aKnots, aMults, aDeg);
        using var aNAx = new N.gp_Ax1(new N.gp_Pnt(0, 0, 0), new N.gp_Dir(0, 0, 1));
        var anAx = new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        return (new N.Geom_SurfaceOfRevolution(aNC, aNAx), new Geom_SurfaceOfRevolution(aPC, anAx));
    }

    /// <summary>A surface of revolution of a line crossing the axis: a cone with its apex inside, not a Geom cone.</summary>
    private static (N.Geom_Surface Native, Geom_Surface Port) ApexRevolution(Oracle theO)
    {
        double anAng = theO.Real(0.2, 1.3);
        var aDir = new gp_Dir(Math.Sin(anAng), 0, Math.Cos(anAng));
        using var aNLine = new N.Geom_Line(new N.gp_Ax1(new N.gp_Pnt(0, 0, 0), new N.gp_Dir(Math.Sin(anAng), 0, Math.Cos(anAng))));
        var aLine = new Geom_Line(new gp_Ax1(new gp_Pnt(0, 0, 0), aDir));
        double aF = theO.Real(-3, -0.5), aL = theO.Real(0.5, 3);
        using var aNTrim = new N.Geom_TrimmedCurve(aNLine, aF, aL);
        var aTrim = new Geom_TrimmedCurve(aLine, aF, aL);
        using var aNAx = new N.gp_Ax1(new N.gp_Pnt(0, 0, 0), new N.gp_Dir(0, 0, 1));
        var anAx = new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        return (new N.Geom_SurfaceOfRevolution(aNTrim, aNAx), new Geom_SurfaceOfRevolution(aTrim, anAx));
    }

    [Fact]
    public void Offsets_at_singular_points()
    {
        string[] aKinds =
        [
            "collapsed B-spline (V)", "collapsed B-spline (U)", "collapsed Bezier (V)", "collapsed Bezier (U)",
            "collapsed at both ends", "sphere-like revolution", "apex revolution"
        ];
        for (int aKind = 0; aKind < aKinds.Length; aKind++)
        {
            var anOracle = new Oracle(output, 20261123 + aKind);
            for (int i = 0; i < 25; i++)
            {
                (N.Geom_Surface aNB, Geom_Surface aPB) = aKind switch
                {
                    0 => CollapsedBSpline(anOracle, false, false, false),
                    1 => CollapsedBSpline(anOracle, false, false, true),
                    2 => CollapsedBSpline(anOracle, true, false, false),
                    3 => CollapsedBSpline(anOracle, true, false, true),
                    4 => CollapsedBSpline(anOracle, anOracle.Bool(), true, anOracle.Bool()),
                    5 => SphereLikeRevolution(anOracle),
                    _ => ApexRevolution(anOracle),
                };
                using (aNB)
                {
                    double anOffset = anOracle.Real(0.1, 2) * (anOracle.Bool() ? 1 : -1);
                    N.Geom_OffsetSurface? aN = null;
                    Geom_OffsetSurface? aP = null;
                    if (!SameOutcome(Catch(() => aN = new N.Geom_OffsetSurface(aNB, anOffset, true)),
                                     Catch(() => aP = new Geom_OffsetSurface(aPB, anOffset, true)), aKinds[aKind] + " construction"))
                    {
                        continue;
                    }
                    using (aN)
                    {
                        string aWhat = "offset of " + aKinds[aKind];
                        SameProperties(anOracle, aN!, aP!, aWhat);
                        aP!.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                        var aSingular = new List<(double, double)>();
                        for (int k = 0; k < 4; k++)
                        {
                            double U = anOracle.Real(aU1, aU2), V = anOracle.Real(aV1, aV2);
                            aSingular.Add((U, aV1));
                            aSingular.Add((U, aV2));
                            aSingular.Add((aU1, V));
                            aSingular.Add((aU2, V));
                            aSingular.Add((U, aV1 + 1e-9 * (aV2 - aV1)));
                            aSingular.Add((aU1 + 1e-9 * (aU2 - aU1), V));
                        }
                        SameEvaluation(anOracle, aN!, aP!, aWhat, 4, aSingular);

                        using var aNA = new N.GeomAdaptor_Surface(aN!);
                        var aPA = new GeomAdaptor_Surface(aP!);
                        foreach ((double U, double V) in aSingular)
                        {
                            SameSurfaceEval(anOracle, aNA, aPA, U, V, 3, aWhat + " GeomAdaptor_Surface");
                        }

                        // reversed: the osculating surfaces are those of the reversed basis, opposite where k is odd
                        using var aNR = N.Geom_OffsetSurface.DownCast(aN!.Copy())!;
                        aNR.VReverse();
                        var aPR = (Geom_OffsetSurface)aP.VReversed();
                        SameEvaluation(anOracle, aNR, aPR, aWhat + " VReversed", 2, aSingular.Take(8));
                    }
                }
            }
            anOracle.Report("Geom_OffsetSurface at singular points: " + aKinds[aKind]);
        }
    }
}
#endif
