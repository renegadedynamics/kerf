// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Api;
using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Kernel.Oracle.Tests.AdaptorOracleInputs;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// GeomAPI_IntCS (IntCurveSurface_HInter) against native OCCT, on random curves of every kind (lines, conics,
/// Bezier, B-spline, periodic B-spline, offset and trimmed curves) and surfaces of every kind (elementary, swept,
/// Bezier, B-spline, offset, trimmed), built from the same numbers on both sides. The curve is moved so that one of
/// its points lies on the surface, now and then slightly off it, so that most pairs intersect. Every path of
/// IntCurveSurface runs: IntAna for conics on quadrics and lines on tori, the roots of the quadric's equation for
/// other curves on quadrics, and the polygon/polyhedron interference refined by IntImp_IntCS for the rest (with
/// the bounds estimated on infinite swept and offset surfaces). The number of points must match, and each point's
/// parameters (W on the curve, U and V on the surface) and coordinates.
/// </summary>
public class GeomAPI_IntCSOracleTests(ITestOutputHelper output)
{
    private const int CasesPerPair = 6;

    private static (double, double) Range(Oracle theO, double theFirst, double theLast, double theHalfRange)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(theFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(theLast))
        {
            return (-theHalfRange, theHalfRange);
        }
        return (theFirst, theLast);
    }

    private sealed class Stats
    {
        public int Cases;
        public int Compared;
        public int Points;
    }

    private static void Run(Oracle theO, Stats theStats, int theCurveKind, int theSurfKind, int theCase)
    {
        (N.Geom_Curve aNC, Curve3D aPC, double aRange) = RandomCurve(theO, theCurveKind);
        (N.Geom_Surface aNS, Surface aPS) = RandomSurface(theO, theSurfKind);
        using (aNC)
        using (aNS)
        {
            (double aF, double aL) = Range(theO, aPC.FirstParameter, aPC.LastParameter, aRange);
            aPS.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
            (double aU1, double aU2) = Range(theO, u1, u2, 10);
            (double aV1, double aV2) = Range(theO, v1, v2, 10);

            // Move the curve so that it meets the surface, now and then near it only
            Point3D aOnC = aPC.Value(theO.Real(aF, aL));
            Point3D aOnS = aPS.Value(theO.Real(aU1, aU2), theO.Real(aV1, aV2));
            double aGap = theO.Int(0, 3) == 0 ? theO.Real(0.0, 1.0) : 0.0;
            Vector3D aV = new(aOnC, aOnS);
            aV += new Vector3D(aGap * theO.Real(-1, 1), aGap * theO.Real(-1, 1), aGap * theO.Real(-1, 1));
            using (N.gp_Vec aNV = Oracle.ToNative(aV))
            {
                aNC.Translate(aNV);
            }
            aPC.Translate(aV);

            string aW = $"{CurveKinds[theCurveKind]} / {SurfaceKinds[theSurfKind]} #{theCase}";
            theStats.Cases++;

            using var aN = new N.GeomAPI_IntCS();
            var aP = new CurveSurfaceIntersection();
            Exception? aNErr = Catch(() => aN.Perform(aNC, aNS));
            Exception? aPErr = Catch(() => aP.Perform(aPC, aPS));
            SameOutcome(aNErr, aPErr, $"{aW} Perform");
            if (aNErr is not null)
            {
                return;
            }

            theO.Same(aN.IsDone(), aP.IsDone, $"{aW} IsDone");
            if (!aP.IsDone)
            {
                return;
            }
            theStats.Compared++;
            int aNb = aP.PointCount;
            theO.Same(aN.NbPoints(), aNb, $"{aW} NbPoints");
            theO.Same(aN.NbSegments(), aP.SegmentCount, $"{aW} NbSegments");
            for (int i = 1; i <= aNb; i++)
            {
                double aNU = 0, aNV = 0, aNW = 0;
                aN.Parameters(i, ref aNU, ref aNV, ref aNW);
                aP.Parameters(i, out double aPU, out double aPV, out double aPW);
                theO.Same(aNW, aPW, $"{aW} point {i} W", Tolerance);
                theO.Same(aNU, aPU, $"{aW} point {i} U", Tolerance);
                theO.Same(aNV, aPV, $"{aW} point {i} V", Tolerance);
                using N.gp_Pnt aNP = aN.Point(i);
                theO.Same(aNP, aP.Point(i), $"{aW} point {i}", Tolerance);
                theStats.Points++;
            }
        }
    }

    // The analytic paths solve polynomials (math_DirectPolynomialRoots, whose cos, acos and cbrt differ in the last
    // bits between MSVC and .NET: decision 157), and Newton's iterations amplify last-bit differences of the inputs.
    private const double Tolerance = 1.0e-9;

    private void RunAll(int theFirstCurveKind, int theLastCurveKind, int theSeed, string theWhat)
    {
        var anOracle = new Oracle(output, theSeed);
        var aStats = new Stats();
        var aFailures = new List<string>();
        for (int ck = theFirstCurveKind; ck <= theLastCurveKind; ck++)
        {
            for (int sk = 0; sk < SurfaceKinds.Length; sk++)
            {
                for (int c = 0; c < CasesPerPair; c++)
                {
                    try
                    {
                        Run(anOracle, aStats, ck, sk, c);
                    }
                    catch (Xunit.Sdk.XunitException anE)
                    {
                        aFailures.Add(anE.Message);
                    }
                }
            }
        }
        output.WriteLine($"{aStats.Cases} cases, {aStats.Compared} computed on both sides, {aStats.Points} points");
        anOracle.Report($"GeomAPI_IntCS, {theWhat}");
        foreach (string aFailure in aFailures)
        {
            output.WriteLine(aFailure);
        }
        Assert.Empty(aFailures);
    }

    /// <summary>Lines and conics against every kind of surface.</summary>
    [Fact]
    public void IntCS_Conics() => RunAll(0, 4, 20261029, "conics");

    /// <summary>Bezier, B-spline (periodic or not), offset and trimmed curves against every kind of surface.</summary>
    [Fact]
    public void IntCS_FreeFormCurves() => RunAll(5, 9, 20261030, "free-form, offset and trimmed curves");
    /// <summary>
    /// Lines tangent to every kind of surface: through a random point of the surface, along a direction of its
    /// tangent plane. At a tangency the roots are double, and the points found are as ill-conditioned as the
    /// problem: a change of one ulp in the line's location changes how many points native OCCT itself returns (the
    /// cluster of Newton solutions around the tangency, and far points on infinite swept and offset surfaces), so
    /// that a last-bit difference anywhere (IntAna's polynomial solver, the libm functions) shows. The test compares
    /// what is well-conditioned: whether each side finds a point at the tangency, and every value where both sides
    /// return the same number of points, within 1e-5 (the error of a double root is about the square root of the
    /// rounding error, times the curvatures).
    /// </summary>
    [Fact]
    public void IntCS_TangentLines()
    {
        var anOracle = new Oracle(output, 20261031);
        var aFailures = new List<string>();
        int aCases = 0, aSameCount = 0, aPoints = 0, aTouching = 0;
        for (int sk = 0; sk < SurfaceKinds.Length; sk++)
        {
            for (int c = 0; c < 3 * CasesPerPair; c++)
            {
                try
                {
                    (N.Geom_Surface aNS, Surface aPS) = RandomSurface(anOracle, sk);
                    using (aNS)
                    {
                        aPS.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
                        (double aU1, double aU2) = Range(anOracle, u1, u2, 10);
                        (double aV1, double aV2) = Range(anOracle, v1, v2, 10);
                        double aU = anOracle.Real(aU1, aU2), aV = anOracle.Real(aV1, aV2);
                        aPS.D1(aU, aV, out Point3D aP, out Vector3D aD1U, out Vector3D aD1V);
                        Vector3D aT = anOracle.Real(-1, 1) * aD1U + anOracle.Real(-1, 1) * aD1V;
                        if (aT.Magnitude < 1e-6)
                        {
                            continue;
                        }
                        Direction3D aDir = new(aT);
                        using N.Geom_Line aNL = new(Oracle.ToNative(aP), Oracle.ToNative(aDir));
                        LineCurve3D aPL = new(aP, aDir);
                        string aW = $"tangent line / {SurfaceKinds[sk]} #{c}";
                        aCases++;

                        using var aN = new N.GeomAPI_IntCS();
                        var aPI = new CurveSurfaceIntersection();
                        Exception? aNErr = Catch(() => aN.Perform(aNL, aNS));
                        Exception? aPErr = Catch(() => aPI.Perform(aPL, aPS));
                        SameOutcome(aNErr, aPErr, $"{aW} Perform");
                        if (aNErr is not null)
                        {
                            continue;
                        }

                        // A point at the tangency on one side if and only if on the other
                        double aNear = 1e-5 * Math.Max(1.0, aP.XYZ().Modulus());
                        bool aNTouch = false, aPTouch = false;
                        for (int i = 1; i <= aN.NbPoints(); i++)
                        {
                            using N.gp_Pnt aNP = aN.Point(i);
                            aNTouch |= Oracle.FromNative(aNP).DistanceTo(aP) <= aNear;
                        }
                        for (int i = 1; i <= aPI.PointCount; i++)
                        {
                            aPTouch |= aPI.Point(i).DistanceTo(aP) <= aNear;
                        }
                        if (sk != 4)
                        {
                            // On a torus, IntAna_IntLinTorus decides a double root of a quartic by the sign of a
                            // value that is zero to the rounding error: native OCCT finds the tangency or not as the
                            // line moves by one ulp.
                            anOracle.Same(aNTouch, aPTouch, $"{aW} finds the tangency");
                        }
                        aTouching += aPTouch ? 1 : 0;

                        int aNb = aPI.PointCount;
                        if (aN.NbPoints() != aNb)
                        {
                            continue;
                        }
                        aSameCount++;
                        for (int i = 1; i <= aNb; i++)
                        {
                            double aNU = 0, aNV = 0, aNW = 0;
                            aN.Parameters(i, ref aNU, ref aNV, ref aNW);
                            aPI.Parameters(i, out double aPU, out double aPV, out double aPW);
                            anOracle.Same(aNW, aPW, $"{aW} point {i} W", TangentTolerance);
                            anOracle.Same(aNU, aPU, $"{aW} point {i} U", TangentTolerance);
                            anOracle.Same(aNV, aPV, $"{aW} point {i} V", TangentTolerance);
                            using N.gp_Pnt aNP = aN.Point(i);
                            anOracle.Same(aNP, aPI.Point(i), $"{aW} point {i}", TangentTolerance);
                            aPoints++;
                        }
                    }
                }
                catch (Xunit.Sdk.XunitException anE)
                {
                    aFailures.Add(anE.Message);
                }
            }
        }
        output.WriteLine($"{aCases} cases, {aTouching} touching, {aSameCount} with the same number of points ({aPoints} points)");
        anOracle.Report("GeomAPI_IntCS, tangent lines");
        foreach (string aFailure in aFailures)
        {
            output.WriteLine(aFailure);
        }
        Assert.Empty(aFailures);
    }

    private const double TangentTolerance = 1.0e-5;
}
