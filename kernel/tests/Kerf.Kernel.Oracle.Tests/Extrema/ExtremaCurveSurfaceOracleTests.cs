// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Kernel.Oracle.Tests.AdaptorOracleInputs;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// Extrema_ExtCS against native OCCT, through BRepExtrema_DistShapeShape between an edge and a face, whose
/// BRepExtrema_ExtCF calls it; the port's side mirrors BRepExtrema_ExtCF.
/// </summary>
/// <remarks>
/// BRepExtrema_ExtCF::Initialize loads Extrema_ExtCS with the face's surface (BRepAdaptor_Surface) on
/// BRepTools::UVBounds and the tolerances max(min(URes, VRes)(min(Tol(F), Confusion)), PConfusion) for the surface and
/// max(Resolution(min(Tol(E), Confusion)), PConfusion) for the curve; Perform runs it on the edge's curve over its
/// range and keeps the extrema the face classifier puts IN or ON the face. The native solutions with an edge and a
/// face as supports are those extrema, copied unchanged: each must be one of the port's. Parameters and the face's
/// UV bounds are read natively; the curves and surfaces are built identically on both sides.
/// <para>
/// Every pair is compared: those Extrema_ExtCS solves analytically through Extrema_ExtElCS (lines against spheres,
/// cylinders and parallel planes, circles against planes, cylinders and spheres, hyperbolas against planes) and those
/// it solves through Extrema_ExtPS and Extrema_GenExtCS (a line against a plane it crosses, and the rest).
/// </para>
/// </remarks>
public class ExtremaCurveSurfaceOracleTests(ITestOutputHelper output)
{
    private const int Cases = 300;

    // (curve kind, surface kind) in AdaptorOracleInputs' CurveKinds / SurfaceKinds: line 0, circle 1, hyperbola 3;
    // plane 0, cylinder 1, sphere 3
    private static readonly (int Curve, int Surface)[] Pairs =
        [(0, 3), (0, 1), (0, 0), (1, 0), (1, 1), (1, 3), (3, 0)];

    [Fact]
    public void ExtCS_through_BRepExtrema_DistShapeShape()
    {
        var anOracle = new Oracle(output, 20261029);
        int aCases = 0, aCompared = 0, aSkipped = 0, aParallel = 0;
        for (int c = 0; c < Cases; c++)
        {
            (int aCurveKind, int aSurfKind) = Pairs[c % Pairs.Length];
            (N.Geom_Curve aNC, Curve3D aPC, _) = RandomCurve(anOracle, aCurveKind);
            (N.Geom_Surface aNS, Surface aPS) = RandomSurface(anOracle, aSurfKind);
            using (aNC)
            using (aNS)
            {
                (double aF, double aL) = Range(anOracle, aPC.FirstParameter, aPC.LastParameter, 1e-3);
                aPS.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
                (double aU1, double aU2) = Range(anOracle, u1, u2, 1e-2);
                (double aV1, double aV2) = Range(anOracle, v1, v2, 1e-2);

                // Move the curve to the face, now and then onto it, so that extrema fall inside both
                if (anOracle.Int(0, 3) != 0)
                {
                    Point3D aOnC = aPC.Value(anOracle.Real(aF, aL));
                    Point3D aOnS = aPS.Value(anOracle.Real(aU1, aU2), anOracle.Real(aV1, aV2));
                    double aGap = anOracle.Bool() ? 0.0 : anOracle.Real(0.01, 2.0);
                    Vector3D aV = new(aOnC, aOnS);
                    aV += new Vector3D(aGap * anOracle.Real(-1, 1), aGap * anOracle.Real(-1, 1), aGap * anOracle.Real(-1, 1));
                    using N.gp_Vec aNV = Oracle.ToNative(aV);
                    aNC.Translate(aNV);
                    aPC.Translate(aV);
                }
                string aW = $"{CurveKinds[aCurveKind]} [{aF:R}, {aL:R}] / {SurfaceKinds[aSurfKind]} [{aU1:R}, {aU2:R}] x [{aV1:R}, {aV2:R}] #{c}";

                using var aME = new N.BRepBuilderAPI_MakeEdge(aNC, aF, aL);
                using var aMF = new N.BRepBuilderAPI_MakeFace(aNS, aU1, aU2, aV1, aV2, 1e-7);
                if (!aME.IsDone() || !aMF.IsDone())
                {
                    continue;
                }
                using N.TopoDS_Edge aNE = aME.Edge();
                using N.TopoDS_Face aNF = aMF.Face();

                // The port's side of BRepExtrema_ExtCF(E, F)
                double aEF = 0.0, aEL = 0.0;
                N.BRep_Tool.Range(aNE, ref aEF, ref aEL);
                double aFU1 = 0.0, aFU2 = 0.0, aFV1 = 0.0, aFV2 = 0.0;
                N.BRepTools.UVBounds(aNF, ref aFU1, ref aFU2, ref aFV1, ref aFV2);
                var aSurf = new SurfaceAdaptor(aPS, aFU1, aFU2, aFV1, aFV2);
                var aCurv = new CurveAdaptor(aPC, aEF, aEL);
                double aTolS = Math.Min(N.BRep_Tool.Tolerance(aNF), global::Kerf.Engine.Foundation.Precision.Confusion());
                aTolS = Math.Min(aSurf.UResolution(aTolS), aSurf.VResolution(aTolS));
                aTolS = Math.Max(aTolS, global::Kerf.Engine.Foundation.Precision.PConfusion());
                double aTolC = Math.Min(N.BRep_Tool.Tolerance(aNE), global::Kerf.Engine.Foundation.Precision.Confusion());
                aTolC = aCurv.Resolution(aTolC);
                aTolC = Math.Max(aTolC, global::Kerf.Engine.Foundation.Precision.PConfusion());

                var aPort = new ExtCS();
                aPort.Initialize(aSurf, aFU1, aFU2, aFV1, aFV2, aTolC, aTolS);
                aPort.Perform(aCurv, aEF, aEL);
                aCases++;

                using var aN = new N.BRepExtrema_DistShapeShape(aNE, aNF);
                if (!aN.IsDone() || !aPort.IsDone)
                {
                    continue;
                }
                if (aPort.IsParallel)
                {
                    aParallel++;
                    continue;
                }
                for (int i = 1; i <= aN.NbSolution(); i++)
                {
                    if (aN.SupportTypeShape1(i) != N.BRepExtrema_SupportType.BRepExtrema_IsOnEdge
                        || aN.SupportTypeShape2(i) != N.BRepExtrema_SupportType.BRepExtrema_IsInFace)
                    {
                        continue;
                    }
                    double aT = 0.0, aU = 0.0, aVv = 0.0;
                    aN.ParOnEdgeS1(i, ref aT);
                    aN.ParOnFaceS2(i, ref aU, ref aVv);

                    int aMatch = 0;
                    double aBest = double.MaxValue;
                    for (int j = 1; j <= aPort.ExtCount; j++)
                    {
                        aPort.Points(j, out POnCurv aP1, out POnSurf aP2);
                        aP2.Parameter(out double aPU, out double aPV);
                        double aD = Math.Abs(aP1.Parameter() - aT) + Math.Abs(aPU - aU) + Math.Abs(aPV - aVv);
                        if (aD < aBest)
                        {
                            aBest = aD;
                            aMatch = j;
                        }
                    }
                    if (aMatch == 0 || aBest > 1e-6)
                    {
                        // DistanceSS also projects the edge's vertices onto the face: such a solution has an edge
                        // parameter at a bound
                        Assert.True(Math.Abs(aT - aEF) < 1e-9 || Math.Abs(aT - aEL) < 1e-9,
                            $"{aW}: native solution ({aT:R}; {aU:R}, {aVv:R}) is not an extremum of the port's Extrema_ExtCS");
                        aSkipped++;
                        continue;
                    }
                    aPort.Points(aMatch, out POnCurv aQ1, out POnSurf aQ2);
                    aQ2.Parameter(out double aQU, out double aQV);
                    using N.gp_Pnt aNP1 = aN.PointOnShape1(i);
                    using N.gp_Pnt aNP2 = aN.PointOnShape2(i);
                    anOracle.Same(aT, aQ1.Parameter(), $"{aW} parameter on the curve");
                    anOracle.Same(aU, aQU, $"{aW} U");
                    anOracle.Same(aVv, aQV, $"{aW} V");
                    anOracle.Same(aNP1, aQ1.Value(), $"{aW} point on the curve");
                    anOracle.Same(aNP2, aQ2.Value(), $"{aW} point on the surface");
                    anOracle.Same(Oracle.FromNative(aNP1).SquareDistanceTo(Oracle.FromNative(aNP2)),
                        aPort.SquareDistance(aMatch), $"{aW} square distance");
                    aCompared++;
                }
            }
        }
        output.WriteLine($"{aCases} edge/face pairs, {aCompared} solutions compared, {aSkipped} vertex projections skipped, "
                         + $"{aParallel} parallel");
        Assert.True(aCompared > aCases / 3, "too few solutions compared");
        anOracle.Report("Extrema_ExtCS through BRepExtrema_DistShapeShape");
    }

    // A finite range inside [theFirst, theLast] (a random one where that is infinite), whole half the time.
    private static (double First, double Last) Range(Oracle theO, double theFirst, double theLast, double theMinPart)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(theFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(theLast))
        {
            double a = theO.Real(-5, 0), b = theO.Real(0.5, 5);
            return (a, a + b);
        }
        if (theO.Bool())
        {
            return (theFirst, theLast);
        }
        double u = theO.Real(theFirst, theLast), v = theO.Real(theFirst, theLast);
        if (Math.Abs(u - v) < theMinPart * (theLast - theFirst))
        {
            return (theFirst, theFirst + 0.5 * (theLast - theFirst));
        }
        return u < v ? (u, v) : (v, u);
    }
}
