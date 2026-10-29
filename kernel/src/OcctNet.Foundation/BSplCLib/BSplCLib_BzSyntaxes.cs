// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_BzSyntaxes.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BSplCLib_CurveComputation;

namespace Occt;

// Bezier curves as B-splines: the knots {0, 1} with multiplicities {Degree + 1, Degree + 1}.
// OCCT's file defines No_Standard_RangeError and No_Standard_OutOfRange; the port keeps its checks.

public static partial class BSplCLib
{
    /// <summary>
    /// Raises the degree of the Bezier curve <paramref name="Poles"/> (weights or null) to
    /// <paramref name="NewDegree"/>; NewPoles has NewDegree + 1 poles.
    /// </summary>
    public static void IncreaseDegree(int NewDegree,
                                      NCollection_Array1<gp_Pnt> Poles,
                                      NCollection_Array1<double>? Weights,
                                      NCollection_Array1<gp_Pnt> NewPoles,
                                      NCollection_Array1<double>? NewWeights)
    {
        BSplCLib_IncreaseDegree_Bezier<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(NewDegree,
                                                                               Poles,
                                                                               Weights,
                                                                               NewPoles,
                                                                               NewWeights);
    }

    /// <inheritdoc cref="IncreaseDegree(int, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?)"/>
    public static void IncreaseDegree(int theNewDegree,
                                      NCollection_Array1<gp_Pnt2d> thePoles,
                                      NCollection_Array1<double>? theWeights,
                                      NCollection_Array1<gp_Pnt2d> theNewPoles,
                                      NCollection_Array1<double>? theNewWeights)
    {
        BSplCLib_IncreaseDegree_Bezier<gp_Pnt2d, gp_Vec2d, BSplCLib_CurveTraits2d>(theNewDegree,
                                                                                   thePoles,
                                                                                   theWeights,
                                                                                   theNewPoles,
                                                                                   theNewWeights);
    }

    /// <summary>
    /// Computes the polynomial coefficients (the cache, see BuildCache) of the Bezier curve
    /// <paramref name="Poles"/> on [0, 1]. Pass null weights for a non-rational curve.
    /// </summary>
    public static void PolesCoefficients(NCollection_Array1<gp_Pnt> Poles,
                                         NCollection_Array1<double>? Weights,
                                         NCollection_Array1<gp_Pnt> CachePoles,
                                         NCollection_Array1<double>? CacheWeights)
    {
        BSplCLib_PolesCoefficients_Bezier<gp_Pnt, gp_Vec, BSplCLib_CurveTraits3d>(Poles,
                                                                                  Weights,
                                                                                  CachePoles,
                                                                                  CacheWeights);
    }

    /// <inheritdoc cref="PolesCoefficients(NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?)"/>
    public static void PolesCoefficients(NCollection_Array1<gp_Pnt2d> Poles,
                                         NCollection_Array1<double>? Weights,
                                         NCollection_Array1<gp_Pnt2d> CachePoles,
                                         NCollection_Array1<double>? CacheWeights)
    {
        BSplCLib_PolesCoefficients_Bezier<gp_Pnt2d, gp_Vec2d, BSplCLib_CurveTraits2d>(
          Poles,
          Weights,
          CachePoles,
          CacheWeights);
    }

    /// <summary>The point at <paramref name="U"/> on the Bezier curve <paramref name="Poles"/> (weights or null).</summary>
    public static void D0(double U,
                          NCollection_Array1<gp_Pnt> Poles,
                          NCollection_Array1<double>? Weights,
                          out gp_Pnt P)
    {
        int aDegree = Poles.Length() - 1;
        BezierKnotArrays(aDegree, out NCollection_Array1<double> aBezierKnots_Knot, out NCollection_Array1<int> aBezierKnots_Mult);
        BSplCLib.D0(U, 1, aDegree, false, Poles, Weights, aBezierKnots_Knot, aBezierKnots_Mult, out P);
    }

    /// <inheritdoc cref="D0(double, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?, out gp_Pnt)"/>
    public static void D0(double U,
                          NCollection_Array1<gp_Pnt2d> Poles,
                          NCollection_Array1<double>? Weights,
                          out gp_Pnt2d P)
    {
        int aDegree = Poles.Length() - 1;
        BezierKnotArrays(aDegree, out NCollection_Array1<double> aBezierKnots_Knot, out NCollection_Array1<int> aBezierKnots_Mult);
        BSplCLib.D0(U, 1, aDegree, false, Poles, Weights, aBezierKnots_Knot, aBezierKnots_Mult, out P);
    }

    /// <summary>The point and first derivative at <paramref name="U"/> on a Bezier curve.</summary>
    public static void D1(double U,
                          NCollection_Array1<gp_Pnt> Poles,
                          NCollection_Array1<double>? Weights,
                          out gp_Pnt P,
                          out gp_Vec V)
    {
        int aDegree = Poles.Length() - 1;
        BezierKnotArrays(aDegree, out NCollection_Array1<double> aBezierKnots_Knot, out NCollection_Array1<int> aBezierKnots_Mult);
        BSplCLib.D1(U, 1, aDegree, false, Poles, Weights, aBezierKnots_Knot, aBezierKnots_Mult, out P, out V);
    }

    /// <inheritdoc cref="D1(double, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?, out gp_Pnt, out gp_Vec)"/>
    public static void D1(double U,
                          NCollection_Array1<gp_Pnt2d> Poles,
                          NCollection_Array1<double>? Weights,
                          out gp_Pnt2d P,
                          out gp_Vec2d V)
    {
        int aDegree = Poles.Length() - 1;
        BezierKnotArrays(aDegree, out NCollection_Array1<double> aBezierKnots_Knot, out NCollection_Array1<int> aBezierKnots_Mult);
        BSplCLib.D1(U, 1, aDegree, false, Poles, Weights, aBezierKnots_Knot, aBezierKnots_Mult, out P, out V);
    }

    /// <summary>The point and first two derivatives at <paramref name="U"/> on a Bezier curve.</summary>
    public static void D2(double U,
                          NCollection_Array1<gp_Pnt> Poles,
                          NCollection_Array1<double>? Weights,
                          out gp_Pnt P,
                          out gp_Vec V1,
                          out gp_Vec V2)
    {
        int aDegree = Poles.Length() - 1;
        BezierKnotArrays(aDegree, out NCollection_Array1<double> aBezierKnots_Knot, out NCollection_Array1<int> aBezierKnots_Mult);
        BSplCLib.D2(U,
                    1,
                    aDegree,
                    false,
                    Poles,
                    Weights,
                    aBezierKnots_Knot,
                    aBezierKnots_Mult,
                    out P,
                    out V1,
                    out V2);
    }

    /// <inheritdoc cref="D2(double, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?, out gp_Pnt, out gp_Vec, out gp_Vec)"/>
    public static void D2(double U,
                          NCollection_Array1<gp_Pnt2d> Poles,
                          NCollection_Array1<double>? Weights,
                          out gp_Pnt2d P,
                          out gp_Vec2d V1,
                          out gp_Vec2d V2)
    {
        int aDegree = Poles.Length() - 1;
        BezierKnotArrays(aDegree, out NCollection_Array1<double> aBezierKnots_Knot, out NCollection_Array1<int> aBezierKnots_Mult);
        BSplCLib.D2(U,
                    1,
                    aDegree,
                    false,
                    Poles,
                    Weights,
                    aBezierKnots_Knot,
                    aBezierKnots_Mult,
                    out P,
                    out V1,
                    out V2);
    }

    /// <summary>The point and first three derivatives at <paramref name="U"/> on a Bezier curve.</summary>
    public static void D3(double U,
                          NCollection_Array1<gp_Pnt> Poles,
                          NCollection_Array1<double>? Weights,
                          out gp_Pnt P,
                          out gp_Vec V1,
                          out gp_Vec V2,
                          out gp_Vec V3)
    {
        int aDegree = Poles.Length() - 1;
        BezierKnotArrays(aDegree, out NCollection_Array1<double> aBezierKnots_Knot, out NCollection_Array1<int> aBezierKnots_Mult);
        BSplCLib.D3(U,
                    1,
                    aDegree,
                    false,
                    Poles,
                    Weights,
                    aBezierKnots_Knot,
                    aBezierKnots_Mult,
                    out P,
                    out V1,
                    out V2,
                    out V3);
    }

    /// <inheritdoc cref="D3(double, NCollection_Array1{gp_Pnt}, NCollection_Array1{double}?, out gp_Pnt, out gp_Vec, out gp_Vec, out gp_Vec)"/>
    public static void D3(double U,
                          NCollection_Array1<gp_Pnt2d> Poles,
                          NCollection_Array1<double>? Weights,
                          out gp_Pnt2d P,
                          out gp_Vec2d V1,
                          out gp_Vec2d V2,
                          out gp_Vec2d V3)
    {
        int aDegree = Poles.Length() - 1;
        BezierKnotArrays(aDegree, out NCollection_Array1<double> aBezierKnots_Knot, out NCollection_Array1<int> aBezierKnots_Mult);
        BSplCLib.D3(U,
                    1,
                    aDegree,
                    false,
                    Poles,
                    Weights,
                    aBezierKnots_Knot,
                    aBezierKnots_Mult,
                    out P,
                    out V1,
                    out V2,
                    out V3);
    }
}
