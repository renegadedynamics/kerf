// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_BzSyntaxes.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Geometry.Splines.CurveComputation;

namespace Kerf.Engine.Geometry.Splines;

// Bezier curves as B-splines: the knots {0, 1} with multiplicities {Degree + 1, Degree + 1}.
// Note: OCCT's file defines No_Standard_RangeError and No_Standard_OutOfRange; the port keeps its checks.

internal static partial class BSplCLib
{
    /// <summary>
    /// Raises the degree of the Bezier curve <paramref name="poles"/> (weights or null) to
    /// <paramref name="newDegree"/>; NewPoles has NewDegree + 1 poles.
    /// </summary>
    public static void IncreaseDegree(int newDegree,
                                      NCollection_Array1<Point3D> poles,
                                      NCollection_Array1<double>? weights,
                                      NCollection_Array1<Point3D> newPoles,
                                      NCollection_Array1<double>? newWeights)
    {
        BSplCLib_IncreaseDegree_Bezier<Point3D, Vector3D, CurveTraits3d>(newDegree,
                                                                               poles,
                                                                               weights,
                                                                               newPoles,
                                                                               newWeights);
    }

    /// <inheritdoc cref="IncreaseDegree(int, NCollection_Array1{Point3D}, NCollection_Array1{double}?, NCollection_Array1{Point3D}, NCollection_Array1{double}?)"/>
    public static void IncreaseDegree(int newDegree,
                                      NCollection_Array1<Point2D> poles,
                                      NCollection_Array1<double>? weights,
                                      NCollection_Array1<Point2D> newPoles,
                                      NCollection_Array1<double>? newWeights)
    {
        BSplCLib_IncreaseDegree_Bezier<Point2D, Vector2D, CurveTraits2d>(newDegree,
                                                                                   poles,
                                                                                   weights,
                                                                                   newPoles,
                                                                                   newWeights);
    }

    /// <summary>
    /// Computes the polynomial coefficients (the cache, see BuildCache) of the Bezier curve
    /// <paramref name="poles"/> on [0, 1]. Pass null weights for a non-rational curve.
    /// </summary>
    public static void PolesCoefficients(NCollection_Array1<Point3D> poles,
                                         NCollection_Array1<double>? weights,
                                         NCollection_Array1<Point3D> cachePoles,
                                         NCollection_Array1<double>? cacheWeights)
    {
        BSplCLib_PolesCoefficients_Bezier<Point3D, Vector3D, CurveTraits3d>(poles,
                                                                                  weights,
                                                                                  cachePoles,
                                                                                  cacheWeights);
    }

    /// <inheritdoc cref="PolesCoefficients(NCollection_Array1{Point3D}, NCollection_Array1{double}?, NCollection_Array1{Point3D}, NCollection_Array1{double}?)"/>
    public static void PolesCoefficients(NCollection_Array1<Point2D> poles,
                                         NCollection_Array1<double>? weights,
                                         NCollection_Array1<Point2D> cachePoles,
                                         NCollection_Array1<double>? cacheWeights)
    {
        BSplCLib_PolesCoefficients_Bezier<Point2D, Vector2D, CurveTraits2d>(
          poles,
          weights,
          cachePoles,
          cacheWeights);
    }

    /// <summary>The point at <paramref name="u"/> on the Bezier curve <paramref name="poles"/> (weights or null).</summary>
    public static void D0(double u,
                          NCollection_Array1<Point3D> poles,
                          NCollection_Array1<double>? weights,
                          out Point3D p)
    {
        int degree = poles.Length() - 1;
        BezierKnotArrays(degree, out NCollection_Array1<double> bezierKnots_Knot, out NCollection_Array1<int> bezierKnots_Mult);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 1, degree, false, poles, weights, bezierKnots_Knot, bezierKnots_Mult, out p);
    }

    /// <inheritdoc cref="D0(double, NCollection_Array1{Point3D}, NCollection_Array1{double}?, out Point3D)"/>
    public static void D0(double u,
                          NCollection_Array1<Point2D> poles,
                          NCollection_Array1<double>? weights,
                          out Point2D p)
    {
        int degree = poles.Length() - 1;
        BezierKnotArrays(degree, out NCollection_Array1<double> bezierKnots_Knot, out NCollection_Array1<int> bezierKnots_Mult);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u, 1, degree, false, poles, weights, bezierKnots_Knot, bezierKnots_Mult, out p);
    }

    /// <summary>The point and first derivative at <paramref name="u"/> on a Bezier curve.</summary>
    public static void D1(double u,
                          NCollection_Array1<Point3D> poles,
                          NCollection_Array1<double>? weights,
                          out Point3D p,
                          out Vector3D v)
    {
        int degree = poles.Length() - 1;
        BezierKnotArrays(degree, out NCollection_Array1<double> bezierKnots_Knot, out NCollection_Array1<int> bezierKnots_Mult);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, 1, degree, false, poles, weights, bezierKnots_Knot, bezierKnots_Mult, out p, out v);
    }

    /// <inheritdoc cref="D1(double, NCollection_Array1{Point3D}, NCollection_Array1{double}?, out Point3D, out Vector3D)"/>
    public static void D1(double u,
                          NCollection_Array1<Point2D> poles,
                          NCollection_Array1<double>? weights,
                          out Point2D p,
                          out Vector2D v)
    {
        int degree = poles.Length() - 1;
        BezierKnotArrays(degree, out NCollection_Array1<double> bezierKnots_Knot, out NCollection_Array1<int> bezierKnots_Mult);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u, 1, degree, false, poles, weights, bezierKnots_Knot, bezierKnots_Mult, out p, out v);
    }

    /// <summary>The point and first two derivatives at <paramref name="u"/> on a Bezier curve.</summary>
    public static void D2(double u,
                          NCollection_Array1<Point3D> poles,
                          NCollection_Array1<double>? weights,
                          out Point3D p,
                          out Vector3D v1,
                          out Vector3D v2)
    {
        int degree = poles.Length() - 1;
        BezierKnotArrays(degree, out NCollection_Array1<double> bezierKnots_Knot, out NCollection_Array1<int> bezierKnots_Mult);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u,
                    1,
                    degree,
                    false,
                    poles,
                    weights,
                    bezierKnots_Knot,
                    bezierKnots_Mult,
                    out p,
                    out v1,
                    out v2);
    }

    /// <inheritdoc cref="D2(double, NCollection_Array1{Point3D}, NCollection_Array1{double}?, out Point3D, out Vector3D, out Vector3D)"/>
    public static void D2(double u,
                          NCollection_Array1<Point2D> poles,
                          NCollection_Array1<double>? weights,
                          out Point2D p,
                          out Vector2D v1,
                          out Vector2D v2)
    {
        int degree = poles.Length() - 1;
        BezierKnotArrays(degree, out NCollection_Array1<double> bezierKnots_Knot, out NCollection_Array1<int> bezierKnots_Mult);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u,
                    1,
                    degree,
                    false,
                    poles,
                    weights,
                    bezierKnots_Knot,
                    bezierKnots_Mult,
                    out p,
                    out v1,
                    out v2);
    }

    /// <summary>The point and first three derivatives at <paramref name="u"/> on a Bezier curve.</summary>
    public static void D3(double u,
                          NCollection_Array1<Point3D> poles,
                          NCollection_Array1<double>? weights,
                          out Point3D p,
                          out Vector3D v1,
                          out Vector3D v2,
                          out Vector3D v3)
    {
        int degree = poles.Length() - 1;
        BezierKnotArrays(degree, out NCollection_Array1<double> bezierKnots_Knot, out NCollection_Array1<int> bezierKnots_Mult);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(u,
                    1,
                    degree,
                    false,
                    poles,
                    weights,
                    bezierKnots_Knot,
                    bezierKnots_Mult,
                    out p,
                    out v1,
                    out v2,
                    out v3);
    }

    /// <inheritdoc cref="D3(double, NCollection_Array1{Point3D}, NCollection_Array1{double}?, out Point3D, out Vector3D, out Vector3D, out Vector3D)"/>
    public static void D3(double u,
                          NCollection_Array1<Point2D> poles,
                          NCollection_Array1<double>? weights,
                          out Point2D p,
                          out Vector2D v1,
                          out Vector2D v2,
                          out Vector2D v3)
    {
        int degree = poles.Length() - 1;
        BezierKnotArrays(degree, out NCollection_Array1<double> bezierKnots_Knot, out NCollection_Array1<int> bezierKnots_Mult);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(u,
                    1,
                    degree,
                    false,
                    poles,
                    weights,
                    bezierKnots_Knot,
                    bezierKnots_Mult,
                    out p,
                    out v1,
                    out v2,
                    out v3);
    }
}
