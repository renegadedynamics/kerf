// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_CurveComputation.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using System.Runtime.InteropServices;

namespace Kerf.Engine.Geometry.Splines;

// Note: OCCT's template over <Point, Vector, Array1OfPoints, Dimension> is ported once, as generic methods over the
// point and vector types and a traits type. The traits are static abstract members of an interface implemented by
// two empty structs (2D and 3D), so the JIT specialises each instantiation as the C++ compiler does. Besides
// Note: OCCT's traits operations, the interface carries the calls the template resolves by overload at instantiation
// (PLib::SetPoles/GetPoles, SetCoord, Translated). Arrays of points are read as arrays of doubles with
// MemoryMarshal.Cast, as OCCT casts them with (double*); a point or vector OCCT writes through a (double*) is
// computed in a stack buffer and then built from it.

/// <summary>The point and vector operations the B-spline curve templates need, for one dimension.</summary>
internal interface ICurveTraits<Point, Vector>
    where Point : struct
    where Vector : struct
{
    /// <summary>The dimension of the space: 2 or 3.</summary>
    static abstract int Dimension { get; }

    // Coordinate operations
    static abstract void PointToCoords(Span<double> carr, in Point pnt, double op);

    static abstract void PointToCoordsScaled(Span<double> carr, in Point pnt, double scale);

    static abstract void CoordsToPointMultiplied(out Point pnt, ReadOnlySpan<double> carr, double factor);

    static abstract void CoordsToPointScaled(out Point pnt, ReadOnlySpan<double> carr, double scale);

    static abstract void CoordsToPointDirect(out Point pnt, ReadOnlySpan<double> carr);

    static abstract void CoordsToVectorDirect(out Vector vec, ReadOnlySpan<double> carr);

    static abstract void NullifyPoint(out Point pnt);

    static abstract void NullifyVector(out Vector vec);

    static abstract void NullifyCoords(Span<double> carr);

    static abstract void CopyCoords(Span<double> carr, ReadOnlySpan<double> carr2);

    static abstract void ModifyCoordsScale(Span<double> carr, double scale);

    static abstract void ModifyCoordsDivide(Span<double> carr, double divisor);

    // Calls that C++ resolves by overload at instantiation

    /// <summary>The coordinates of a vector (OCCT reads them through <c>(double*)&amp;Delta</c>).</summary>
    static abstract void VectorToCoords(Span<double> carr, in Vector vec);

    /// <summary><c>pnt.SetCoord(theIndex, theXi)</c>.</summary>
    static abstract void SetCoord(ref Point pnt, int index, double xi);

    /// <summary><c>pnt.Translated(theScale * theVec)</c>.</summary>
    static abstract Point Translated(in Point pnt, double scale, in Vector vec);

    static abstract void SetPoles(NCollection_Array1<Point> poles, NCollection_Array1<double> fp);

    static abstract void SetPoles(NCollection_Array1<Point> poles, NCollection_Array1<double> weights, NCollection_Array1<double> fp);

    static abstract void GetPoles(NCollection_Array1<double> fp, NCollection_Array1<Point> poles);

    static abstract void GetPoles(NCollection_Array1<double> fp, NCollection_Array1<Point> poles, NCollection_Array1<double> weights);
}

/// <summary>The curve traits for 3D: gp_Pnt, gp_Vec, dimension 3.</summary>
internal readonly struct CurveTraits3d : ICurveTraits<Point3D, Vector3D>
{
    public static int Dimension => 3;

    public static void PointToCoords(Span<double> carr, in Point3D pnt, double op)
    {
        carr[0] = pnt.X() + op;
        carr[1] = pnt.Y() + op;
        carr[2] = pnt.Z() + op;
    }

    public static void PointToCoordsScaled(Span<double> carr, in Point3D pnt, double scale)
    {
        carr[0] = pnt.X() * scale;
        carr[1] = pnt.Y() * scale;
        carr[2] = pnt.Z() * scale;
    }

    public static void CoordsToPointMultiplied(out Point3D pnt, ReadOnlySpan<double> carr, double factor)
        => pnt = new Point3D(carr[0] * factor, carr[1] * factor, carr[2] * factor);

    public static void CoordsToPointScaled(out Point3D pnt, ReadOnlySpan<double> carr, double scale)
        => pnt = new Point3D(carr[0] / scale, carr[1] / scale, carr[2] / scale);

    public static void CoordsToPointDirect(out Point3D pnt, ReadOnlySpan<double> carr)
        => pnt = new Point3D(carr[0], carr[1], carr[2]);

    public static void CoordsToVectorDirect(out Vector3D vec, ReadOnlySpan<double> carr)
        => vec = new Vector3D(carr[0], carr[1], carr[2]);

    public static void NullifyPoint(out Point3D pnt) => pnt = new Point3D(0.0, 0.0, 0.0);

    public static void NullifyVector(out Vector3D vec) => vec = new Vector3D(0.0, 0.0, 0.0);

    public static void NullifyCoords(Span<double> carr)
    {
        carr[0] = carr[1] = carr[2] = 0.0;
    }

    public static void CopyCoords(Span<double> carr, ReadOnlySpan<double> carr2)
    {
        carr[0] = carr2[0];
        carr[1] = carr2[1];
        carr[2] = carr2[2];
    }

    public static void ModifyCoordsScale(Span<double> carr, double scale)
    {
        carr[0] *= scale;
        carr[1] *= scale;
        carr[2] *= scale;
    }

    public static void ModifyCoordsDivide(Span<double> carr, double divisor)
    {
        carr[0] /= divisor;
        carr[1] /= divisor;
        carr[2] /= divisor;
    }

    public static void VectorToCoords(Span<double> carr, in Vector3D vec)
    {
        carr[0] = vec.X();
        carr[1] = vec.Y();
        carr[2] = vec.Z();
    }

    public static void SetCoord(ref Point3D pnt, int index, double xi) => pnt = pnt.WithCoord(index, xi);

    public static Point3D Translated(in Point3D pnt, double scale, in Vector3D vec) => pnt.Translated(scale * vec);

    public static void SetPoles(NCollection_Array1<Point3D> poles, NCollection_Array1<double> fp) => global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(poles, fp);

    public static void SetPoles(NCollection_Array1<Point3D> poles, NCollection_Array1<double> weights, NCollection_Array1<double> fp)
        => global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(poles, weights, fp);

    public static void GetPoles(NCollection_Array1<double> fp, NCollection_Array1<Point3D> poles) => global::Kerf.Engine.Numerics.Polynomials.PLib.GetPoles(fp, poles);

    public static void GetPoles(NCollection_Array1<double> fp, NCollection_Array1<Point3D> poles, NCollection_Array1<double> weights)
        => global::Kerf.Engine.Numerics.Polynomials.PLib.GetPoles(fp, poles, weights);
}

/// <summary>The curve traits for 2D: gp_Pnt2d, gp_Vec2d, dimension 2.</summary>
internal readonly struct CurveTraits2d : ICurveTraits<Point2D, Vector2D>
{
    public static int Dimension => 2;

    public static void PointToCoords(Span<double> carr, in Point2D pnt, double op)
    {
        carr[0] = pnt.X() + op;
        carr[1] = pnt.Y() + op;
    }

    public static void PointToCoordsScaled(Span<double> carr, in Point2D pnt, double scale)
    {
        carr[0] = pnt.X() * scale;
        carr[1] = pnt.Y() * scale;
    }

    public static void CoordsToPointMultiplied(out Point2D pnt, ReadOnlySpan<double> carr, double factor)
        => pnt = new Point2D(carr[0] * factor, carr[1] * factor);

    public static void CoordsToPointScaled(out Point2D pnt, ReadOnlySpan<double> carr, double scale)
        => pnt = new Point2D(carr[0] / scale, carr[1] / scale);

    public static void CoordsToPointDirect(out Point2D pnt, ReadOnlySpan<double> carr)
        => pnt = new Point2D(carr[0], carr[1]);

    public static void CoordsToVectorDirect(out Vector2D vec, ReadOnlySpan<double> carr)
        => vec = new Vector2D(carr[0], carr[1]);

    public static void NullifyPoint(out Point2D pnt) => pnt = new Point2D(0.0, 0.0);

    public static void NullifyVector(out Vector2D vec) => vec = new Vector2D(0.0, 0.0);

    public static void NullifyCoords(Span<double> carr)
    {
        carr[0] = carr[1] = 0.0;
    }

    public static void CopyCoords(Span<double> carr, ReadOnlySpan<double> carr2)
    {
        carr[0] = carr2[0];
        carr[1] = carr2[1];
    }

    public static void ModifyCoordsScale(Span<double> carr, double scale)
    {
        carr[0] *= scale;
        carr[1] *= scale;
    }

    public static void ModifyCoordsDivide(Span<double> carr, double divisor)
    {
        carr[0] /= divisor;
        carr[1] /= divisor;
    }

    public static void VectorToCoords(Span<double> carr, in Vector2D vec)
    {
        carr[0] = vec.X();
        carr[1] = vec.Y();
    }

    public static void SetCoord(ref Point2D pnt, int index, double xi) => pnt = pnt.WithCoord(index, xi);

    public static Point2D Translated(in Point2D pnt, double scale, in Vector2D vec) => pnt.Translated(scale * vec);

    public static void SetPoles(NCollection_Array1<Point2D> poles, NCollection_Array1<double> fp) => global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(poles, fp);

    public static void SetPoles(NCollection_Array1<Point2D> poles, NCollection_Array1<double> weights, NCollection_Array1<double> fp)
        => global::Kerf.Engine.Numerics.Polynomials.PLib.SetPoles(poles, weights, fp);

    public static void GetPoles(NCollection_Array1<double> fp, NCollection_Array1<Point2D> poles) => global::Kerf.Engine.Numerics.Polynomials.PLib.GetPoles(fp, poles);

    public static void GetPoles(NCollection_Array1<double> fp, NCollection_Array1<Point2D> poles, NCollection_Array1<double> weights)
        => global::Kerf.Engine.Numerics.Polynomials.PLib.GetPoles(fp, poles, weights);
}

/// <summary>
/// The B-spline curve computations shared by the 2D and 3D overloads of <see cref="BSplCLib"/>
/// (OCCT's BSplCLib_CurveComputation.pxx templates).
/// </summary>
internal static class CurveComputation
{
    //! Maximum supported degree for B-spline curves.
    internal const int THE_MAX_DEGREE = 25; // BSplCLib::MaxDegree()

    //! Validates that the given degree does not exceed the maximum supported degree.
    //! @throws Standard_OutOfRange if degree exceeds MaxDegree()
    internal static void ValidateBSplineDegree(int degree)
    {
        Standard_OutOfRange_Raise_if(degree > THE_MAX_DEGREE,
                                     "BSplCLib: bspline degree is greater than maximum supported");
    }

    // Sizes of OCCT's BSplCLib_DataContainer_T<Dimension>: stack buffers for poles, knots and derivatives
    // used in evaluation. The port allocates them with stackalloc in each evaluating function.
    private static int DataContainerPolesSize(int dimension) => (THE_MAX_DEGREE + 1) * (dimension + 1);

    private const int DataContainerKnotsSize = 2 * THE_MAX_DEGREE;

    private static int DataContainerDersSize(int dimension) => dimension * 4;

    //! Arrays for knot/multiplicity pairs (OCCT's BSplCLib_KnotArrays, stack-allocated there).
    //! Size=1: single knot (knotValue, multiplicity)
    //! Size=2: Bezier knots {0,1} and mults {degree+1, degree+1}
    internal static void KnotArrays(double knotValue, int multiplicity,
                                    out NCollection_Array1<double> knot, out NCollection_Array1<int> mult)
    {
        knot = new NCollection_Array1<double>([knotValue], 0, 1, 1);
        mult = new NCollection_Array1<int>([multiplicity], 0, 1, 1);
    }

    /// <inheritdoc cref="KnotArrays(double, int, out NCollection_Array1{double}, out NCollection_Array1{int})"/>
    internal static void KnotArrays(int degree, out NCollection_Array1<double> knot, out NCollection_Array1<int> mult)
    {
        knot = new NCollection_Array1<double>([0.0, 1.0], 0, 1, 2);
        mult = new NCollection_Array1<int>([degree + 1, degree + 1], 0, 1, 2);
    }

    // Bezier knot arrays shared by the read-only uses (the Bezier D0..D3), one pair per degree, so that
    // evaluating a Bezier curve doesn't allocate where OCCT uses the stack. Never written.
    private static readonly NCollection_Array1<double>[] THE_BEZIER_KNOTS = BuildBezierKnots();
    private static readonly NCollection_Array1<int>[] THE_BEZIER_MULTS = BuildBezierMults();

    private static NCollection_Array1<double>[] BuildBezierKnots()
    {
        var knots = new NCollection_Array1<double>[THE_MAX_DEGREE + 1];
        for (int i = 0; i <= THE_MAX_DEGREE; i++)
        {
            knots[i] = new NCollection_Array1<double>([0.0, 1.0], 0, 1, 2);
        }
        return knots;
    }

    private static NCollection_Array1<int>[] BuildBezierMults()
    {
        var mults = new NCollection_Array1<int>[THE_MAX_DEGREE + 1];
        for (int i = 0; i <= THE_MAX_DEGREE; i++)
        {
            mults[i] = new NCollection_Array1<int>([i + 1, i + 1], 0, 1, 2);
        }
        return mults;
    }

    /// <summary>
    /// Bezier knots {0, 1} and multiplicities {Degree + 1, Degree + 1} for read-only use: shared arrays for
    /// degrees up to <see cref="THE_MAX_DEGREE"/>, new ones above (the evaluation then rejects the degree).
    /// </summary>
    internal static void BezierKnotArrays(int degree, out NCollection_Array1<double> knot, out NCollection_Array1<int> mult)
    {
        if (degree >= 0 && degree <= THE_MAX_DEGREE)
        {
            knot = THE_BEZIER_KNOTS[degree];
            mult = THE_BEZIER_MULTS[degree];
        }
        else
        {
            KnotArrays(degree, out knot, out mult);
        }
    }

    // Reverses the order of elements in the array.
    // Works with any NCollection_Array1.
    // @param[in,out] theArray - array to be reversed
    // @param[in] theL - length parameter for reversal
    internal static void BSplCLib_Reverse<T>(NCollection_Array1<T> array, int l)
    {
        int lower = array.Lower();
        int upper = array.Upper();
        int aL = lower + (l - lower) % array.Length();

        array.AsSpan().Slice(0, aL - lower + 1).Reverse();
        if (aL < upper)
        {
            array.AsSpan(aL + 1).Reverse();
        }
    }

    // Removes a knot from the B-spline curve
    // @returns true if knot removal was successful
    internal static bool BSplCLib_RemoveKnot<Point, Vector, Traits>(int index,
                                                                   int mult,
                                                                   int degree,
                                                                   bool periodic,
                                                                   NCollection_Array1<Point> Poles,
                                                                   NCollection_Array1<double>? weights,
                                                                   NCollection_Array1<double> knots,
                                                                   NCollection_Array1<int> mults,
                                                                   NCollection_Array1<Point> newPoles,
                                                                   NCollection_Array1<double>? newWeights,
                                                                   NCollection_Array1<double> newKnots,
                                                                   NCollection_Array1<int> newMults,
                                                                   double tolerance)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        bool rational = weights != null;
        int dim;
        dim = Traits.Dimension;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.Length());
        NCollection_Array1<double> newpoles = new(1, dim * newPoles.Length());

        if (rational)
        {
            Traits.SetPoles(Poles, weights!, poles);
        }
        else
        {
            Traits.SetPoles(Poles, poles);
        }

        if (!global::Kerf.Engine.Geometry.Splines.BSplCLib.RemoveKnot(index,
                                 mult,
                                 degree,
                                 periodic,
                                 dim,
                                 poles,
                                 knots,
                                 mults,
                                 newpoles,
                                 newKnots,
                                 newMults,
                                 tolerance))
        {
            return false;
        }

        if (rational)
        {
            Traits.GetPoles(newpoles, newPoles, newWeights!);
        }
        else
        {
            Traits.GetPoles(newpoles, newPoles);
        }
        return true;
    }

    // Inserts an array of knots and multiplicities into the B-spline curve
    internal static void BSplCLib_InsertKnots<Point, Vector, Traits>(int degree,
                                                                    bool periodic,
                                                                    NCollection_Array1<Point> Poles,
                                                                    NCollection_Array1<double>? weights,
                                                                    NCollection_Array1<double> knots,
                                                                    NCollection_Array1<int> mults,
                                                                    NCollection_Array1<double> addKnots,
                                                                    NCollection_Array1<int>? addMults,
                                                                    NCollection_Array1<Point> newPoles,
                                                                    NCollection_Array1<double>? newWeights,
                                                                    NCollection_Array1<double> newKnots,
                                                                    NCollection_Array1<int> newMults,
                                                                    double epsilon,
                                                                    bool add)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        bool rational = weights != null;
        int dim;
        dim = Traits.Dimension;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.Length());
        NCollection_Array1<double> newpoles = new(1, dim * newPoles.Length());

        if (rational)
        {
            Traits.SetPoles(Poles, weights!, poles);
        }
        else
        {
            Traits.SetPoles(Poles, poles);
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(degree,
                             periodic,
                             dim,
                             poles,
                             knots,
                             mults,
                             addKnots,
                             addMults,
                             newpoles,
                             newKnots,
                             newMults,
                             epsilon,
                             add);

        if (rational)
        {
            Traits.GetPoles(newpoles, newPoles, newWeights!);
        }
        else
        {
            Traits.GetPoles(newpoles, newPoles);
        }
    }

    // Inserts a single knot into the B-spline curve
    internal static void BSplCLib_InsertKnot<Point, Vector, Traits>(int uIndex,
                                                                   double u,
                                                                   int uMult,
                                                                   int degree,
                                                                   bool periodic,
                                                                   NCollection_Array1<Point> poles,
                                                                   NCollection_Array1<double>? weights,
                                                                   NCollection_Array1<double> knots,
                                                                   NCollection_Array1<int> mults,
                                                                   NCollection_Array1<Point> newPoles,
                                                                   NCollection_Array1<double>? newWeights)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        _ = uIndex; // unused, as in OCCT
        KnotArrays(u, uMult, out NCollection_Array1<double> singleKnot_Knot, out NCollection_Array1<int> singleKnot_Mult);
        NCollection_Array1<double> newKnots = new(1, knots.Length() + 1);
        NCollection_Array1<int> newMults = new(1, knots.Length() + 1);
        BSplCLib_InsertKnots<Point, Vector, Traits>(degree,
                                                    periodic,
                                                    poles,
                                                    weights,
                                                    knots,
                                                    mults,
                                                    singleKnot_Knot,
                                                    singleKnot_Mult,
                                                    newPoles,
                                                    newWeights,
                                                    newKnots,
                                                    newMults,
                                                    Epsilon(u),
                                                    true);
    }

    // Raises the multiplicity of a knot in the B-spline curve
    internal static void BSplCLib_RaiseMultiplicity<Point, Vector, Traits>(int knotIndex,
                                                                          int mult,
                                                                          int degree,
                                                                          bool periodic,
                                                                          NCollection_Array1<Point> poles,
                                                                          NCollection_Array1<double>? weights,
                                                                          NCollection_Array1<double> knots,
                                                                          NCollection_Array1<int> mults,
                                                                          NCollection_Array1<Point> newPoles,
                                                                          NCollection_Array1<double>? newWeights)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        KnotArrays(knots[knotIndex], mult - mults[knotIndex],
                   out NCollection_Array1<double> singleKnot_Knot, out NCollection_Array1<int> singleKnot_Mult);
        NCollection_Array1<double> newKnots = new(1, knots.Length());
        NCollection_Array1<int> newMults = new(1, knots.Length());
        BSplCLib_InsertKnots<Point, Vector, Traits>(degree,
                                                    periodic,
                                                    poles,
                                                    weights,
                                                    knots,
                                                    mults,
                                                    singleKnot_Knot,
                                                    singleKnot_Mult,
                                                    newPoles,
                                                    newWeights,
                                                    newKnots,
                                                    newMults,
                                                    Epsilon(knots[knotIndex]),
                                                    true);
    }

    // Increases the degree of the B-spline curve
    internal static void BSplCLib_IncreaseDegree<Point, Vector, Traits>(int degree,
                                                                       int newDegree,
                                                                       bool periodic,
                                                                       NCollection_Array1<Point> Poles,
                                                                       NCollection_Array1<double>? weights,
                                                                       NCollection_Array1<double> knots,
                                                                       NCollection_Array1<int> mults,
                                                                       NCollection_Array1<Point> newPoles,
                                                                       NCollection_Array1<double>? newWeights,
                                                                       NCollection_Array1<double> newKnots,
                                                                       NCollection_Array1<int> newMults)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        bool rational = weights != null;
        int dim;
        dim = Traits.Dimension;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.Length());
        NCollection_Array1<double> newpoles = new(1, dim * newPoles.Length());

        if (rational)
        {
            Traits.SetPoles(Poles, weights!, poles);
        }
        else
        {
            Traits.SetPoles(Poles, poles);
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(degree,
                                newDegree,
                                periodic,
                                dim,
                                poles,
                                knots,
                                mults,
                                newpoles,
                                newKnots,
                                newMults);

        if (rational)
        {
            Traits.GetPoles(newpoles, newPoles, newWeights!);
        }
        else
        {
            Traits.GetPoles(newpoles, newPoles);
        }
    }

    // Converts a periodic B-spline curve to non-periodic
    internal static void BSplCLib_Unperiodize<Point, Vector, Traits>(int degree,
                                                                    NCollection_Array1<int> mults,
                                                                    NCollection_Array1<double> knots,
                                                                    NCollection_Array1<Point> Poles,
                                                                    NCollection_Array1<double>? weights,
                                                                    NCollection_Array1<int> newMults,
                                                                    NCollection_Array1<double> newKnots,
                                                                    NCollection_Array1<Point> newPoles,
                                                                    NCollection_Array1<double>? newWeights)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        bool rational = weights != null;
        int dim;
        dim = Traits.Dimension;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.Length());
        NCollection_Array1<double> newpoles = new(1, dim * newPoles.Length());

        if (rational)
        {
            Traits.SetPoles(Poles, weights!, poles);
        }
        else
        {
            Traits.SetPoles(Poles, poles);
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Unperiodize(degree, dim, mults, knots, poles, newMults, newKnots, newpoles);

        if (rational)
        {
            Traits.GetPoles(newpoles, newPoles, newWeights!);
        }
        else
        {
            Traits.GetPoles(newpoles, newPoles);
        }
    }

    // Trims the B-spline curve to a parameter range
    internal static void BSplCLib_Trimming<Point, Vector, Traits>(int degree,
                                                                 bool periodic,
                                                                 NCollection_Array1<double> knots,
                                                                 NCollection_Array1<int> mults,
                                                                 NCollection_Array1<Point> Poles,
                                                                 NCollection_Array1<double>? weights,
                                                                 double u1,
                                                                 double u2,
                                                                 NCollection_Array1<double> newKnots,
                                                                 NCollection_Array1<int> newMults,
                                                                 NCollection_Array1<Point> newPoles,
                                                                 NCollection_Array1<double>? newWeights)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        bool rational = weights != null;
        int dim;
        dim = Traits.Dimension;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.Length());
        NCollection_Array1<double> newpoles = new(1, dim * newPoles.Length());

        if (rational)
        {
            Traits.SetPoles(Poles, weights!, poles);
        }
        else
        {
            Traits.SetPoles(Poles, poles);
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Trimming(degree,
                          periodic,
                          dim,
                          knots,
                          mults,
                          poles,
                          u1,
                          u2,
                          newKnots,
                          newMults,
                          newpoles);

        if (rational)
        {
            Traits.GetPoles(newpoles, newPoles, newWeights!);
        }
        else
        {
            Traits.GetPoles(newpoles, newPoles);
        }
    }

    // Builds the local array for B-spline evaluation
    // @param[out] LP - local poles array for evaluation
    internal static void BSplCLib_BuildEval<Point, Vector, Traits>(int degree,
                                                                  int index,
                                                                  NCollection_Array1<Point> poles,
                                                                  NCollection_Array1<double>? weights,
                                                                  Span<double> lp)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int dimension = Traits.Dimension;
        double w;
        int pole = 0;
        int pLower = poles.Lower();
        int pUpper = poles.Upper();
        int i;
        int ip = pLower + index - 1;
        if (weights == null)
        {
            for (i = 0; i <= degree; i++)
            {
                ip++;
                if (ip > pUpper)
                {
                    ip = pLower;
                }
                ref readonly Point p = ref poles[ip];
                Traits.PointToCoords(lp.Slice(pole), p, 0);
                pole += dimension;
            }
        }
        else
        {
            for (i = 0; i <= degree; i++)
            {
                ip++;
                if (ip > pUpper)
                {
                    ip = pLower;
                }
                ref readonly Point p = ref poles[ip];
                lp[pole + dimension] = w = weights[ip];
                Traits.PointToCoordsScaled(lp.Slice(pole), p, w);
                pole += dimension + 1;
            }
        }
    }

    // Prepares data for B-spline evaluation by storing poles and knots in local arrays
    // @param[in,out] u - parameter value
    // @param[in,out] index - knot span index
    // @param[out] dim - dimension of the curve
    // @param[out] rational - whether the curve is rational
    // dc_poles and dc_knots are the data container's buffers
    private static void PrepareEval_T<Point, Vector, Traits>(ref double u,
                                                            ref int index,
                                                            out int dim,
                                                            out bool rational,
                                                            int degree,
                                                            bool periodic,
                                                            NCollection_Array1<Point> poles,
                                                            NCollection_Array1<double>? weights,
                                                            NCollection_Array1<double> knots,
                                                            NCollection_Array1<int>? mults,
                                                            Span<double> dc_poles,
                                                            Span<double> dc_knots)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        // Set the Index
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(degree, knots, mults, u, periodic, ref index, out u);

        // make the knots
        global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildKnots(degree, index, periodic, knots, mults, dc_knots);
        if (mults == null)
        {
            index -= knots.Lower() + degree;
        }
        else
        {
            index = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(degree, index, periodic, mults);
        }

        // check truly rational
        rational = (weights != null);
        if (rational)
        {
            int wLower = weights!.Lower() + index;
            rational = global::Kerf.Engine.Geometry.Splines.BSplCLib.IsRational(weights, wLower, wLower + degree);
        }

        // make the poles
        if (rational)
        {
            dim = Traits.Dimension + 1;
            BSplCLib_BuildEval<Point, Vector, Traits>(degree,
                                                      index,
                                                      poles,
                                                      weights,
                                                      dc_poles);
        }
        else
        {
            dim = Traits.Dimension;
            BSplCLib_BuildEval<Point, Vector, Traits>(degree,
                                                      index,
                                                      poles,
                                                      global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                                      dc_poles);
        }
    }

    // Evaluates the point on the B-spline curve at parameter U
    internal static void BSplCLib_D0<Point, Vector, Traits>(double U,
                                                           int Index,
                                                           int degree,
                                                           bool periodic,
                                                           NCollection_Array1<Point> poles,
                                                           NCollection_Array1<double>? weights,
                                                           NCollection_Array1<double> knots,
                                                           NCollection_Array1<int>? mults,
                                                           out Point p)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int dimension = Traits.Dimension;
        int dim, index = Index;
        double u = U;
        bool rational;
        ValidateBSplineDegree(degree);
        Span<double> dc_poles = stackalloc double[DataContainerPolesSize(dimension)];
        Span<double> dc_knots = stackalloc double[DataContainerKnotsSize];
        PrepareEval_T<Point, Vector, Traits>(ref u,
                                             ref index,
                                             out dim,
                                             out rational,
                                             degree,
                                             periodic,
                                             poles,
                                             weights,
                                             knots,
                                             mults,
                                             dc_poles,
                                             dc_knots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(u, degree, dc_knots, dim, dc_poles);

        if (rational)
        {
            double w = dc_poles[dimension];
            Traits.CoordsToPointScaled(out p, dc_poles, w);
        }
        else
        {
            Traits.CoordsToPointDirect(out p, dc_poles);
        }
    }

    // Evaluates the point and first derivative on the B-spline curve at parameter U
    internal static void BSplCLib_D1<Point, Vector, Traits>(double U,
                                                           int Index,
                                                           int degree,
                                                           bool periodic,
                                                           NCollection_Array1<Point> poles,
                                                           NCollection_Array1<double>? weights,
                                                           NCollection_Array1<double> knots,
                                                           NCollection_Array1<int>? mults,
                                                           out Point p,
                                                           out Vector v)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int dimension = Traits.Dimension;
        int dim, index = Index;
        double u = U;
        bool rational;
        ValidateBSplineDegree(degree);
        Span<double> dc_poles = stackalloc double[DataContainerPolesSize(dimension)];
        Span<double> dc_knots = stackalloc double[DataContainerKnotsSize];
        Span<double> dc_ders = stackalloc double[DataContainerDersSize(dimension)];
        PrepareEval_T<Point, Vector, Traits>(ref u,
                                             ref index,
                                             out dim,
                                             out rational,
                                             degree,
                                             periodic,
                                             poles,
                                             weights,
                                             knots,
                                             mults,
                                             dc_poles,
                                             dc_knots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u, degree, 1, dc_knots, dim, dc_poles);
        ReadOnlySpan<double> result = dc_poles;
        if (rational)
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.RationalDerivative(degree, 1, dimension, dc_poles, dc_ders);
            result = dc_ders;
        }

        Traits.CoordsToPointDirect(out p, result);
        Traits.CoordsToVectorDirect(out v, result.Slice(dimension));
    }

    // Evaluates the point and first two derivatives on the B-spline curve at parameter U
    internal static void BSplCLib_D2<Point, Vector, Traits>(double U,
                                                           int Index,
                                                           int degree,
                                                           bool periodic,
                                                           NCollection_Array1<Point> poles,
                                                           NCollection_Array1<double>? weights,
                                                           NCollection_Array1<double> knots,
                                                           NCollection_Array1<int>? mults,
                                                           out Point p,
                                                           out Vector v1,
                                                           out Vector v2)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int dimension = Traits.Dimension;
        int dim, index = Index;
        double u = U;
        bool rational;
        ValidateBSplineDegree(degree);
        Span<double> dc_poles = stackalloc double[DataContainerPolesSize(dimension)];
        Span<double> dc_knots = stackalloc double[DataContainerKnotsSize];
        Span<double> dc_ders = stackalloc double[DataContainerDersSize(dimension)];
        PrepareEval_T<Point, Vector, Traits>(ref u,
                                             ref index,
                                             out dim,
                                             out rational,
                                             degree,
                                             periodic,
                                             poles,
                                             weights,
                                             knots,
                                             mults,
                                             dc_poles,
                                             dc_knots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u, degree, 2, dc_knots, dim, dc_poles);
        ReadOnlySpan<double> result = dc_poles;
        if (rational)
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.RationalDerivative(degree, 2, dimension, dc_poles, dc_ders);
            result = dc_ders;
        }

        Traits.CoordsToPointDirect(out p, result);
        Traits.CoordsToVectorDirect(out v1, result.Slice(dimension));
        if (!rational && (degree < 2))
        {
            Traits.NullifyVector(out v2);
        }
        else
        {
            Traits.CoordsToVectorDirect(out v2, result.Slice(2 * dimension));
        }
    }

    // Evaluates the point and first three derivatives on the B-spline curve at parameter U
    internal static void BSplCLib_D3<Point, Vector, Traits>(double U,
                                                           int Index,
                                                           int degree,
                                                           bool periodic,
                                                           NCollection_Array1<Point> poles,
                                                           NCollection_Array1<double>? weights,
                                                           NCollection_Array1<double> knots,
                                                           NCollection_Array1<int>? mults,
                                                           out Point p,
                                                           out Vector v1,
                                                           out Vector v2,
                                                           out Vector v3)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int dimension = Traits.Dimension;
        int dim, index = Index;
        double u = U;
        bool rational;
        ValidateBSplineDegree(degree);
        Span<double> dc_poles = stackalloc double[DataContainerPolesSize(dimension)];
        Span<double> dc_knots = stackalloc double[DataContainerKnotsSize];
        Span<double> dc_ders = stackalloc double[DataContainerDersSize(dimension)];
        PrepareEval_T<Point, Vector, Traits>(ref u,
                                             ref index,
                                             out dim,
                                             out rational,
                                             degree,
                                             periodic,
                                             poles,
                                             weights,
                                             knots,
                                             mults,
                                             dc_poles,
                                             dc_knots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u, degree, 3, dc_knots, dim, dc_poles);
        ReadOnlySpan<double> result = dc_poles;
        if (rational)
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.RationalDerivative(degree, 3, dimension, dc_poles, dc_ders);
            result = dc_ders;
        }

        Traits.CoordsToPointDirect(out p, result);
        Traits.CoordsToVectorDirect(out v1, result.Slice(dimension));
        if (!rational && (degree < 2))
        {
            Traits.NullifyVector(out v2);
        }
        else
        {
            Traits.CoordsToVectorDirect(out v2, result.Slice(2 * dimension));
        }
        if (!rational && (degree < 3))
        {
            Traits.NullifyVector(out v3);
        }
        else
        {
            Traits.CoordsToVectorDirect(out v3, result.Slice(3 * dimension));
        }
    }

    // Evaluates the Nth derivative on the B-spline curve at parameter U
    internal static void BSplCLib_DN<Point, Vector, Traits>(double U,
                                                           int n,
                                                           int Index,
                                                           int degree,
                                                           bool periodic,
                                                           NCollection_Array1<Point> poles,
                                                           NCollection_Array1<double>? weights,
                                                           NCollection_Array1<double> knots,
                                                           NCollection_Array1<int>? mults,
                                                           out Vector vn)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int dimension = Traits.Dimension;
        int dim, index = Index;
        double u = U;
        bool rational;
        ValidateBSplineDegree(degree);
        Span<double> dc_poles = stackalloc double[DataContainerPolesSize(dimension)];
        Span<double> dc_knots = stackalloc double[DataContainerKnotsSize];
        PrepareEval_T<Point, Vector, Traits>(ref u,
                                             ref index,
                                             out dim,
                                             out rational,
                                             degree,
                                             periodic,
                                             poles,
                                             weights,
                                             knots,
                                             mults,
                                             dc_poles,
                                             dc_knots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u, degree, n, dc_knots, dim, dc_poles);

        if (rational)
        {
            Span<double> v = stackalloc double[dimension];
            global::Kerf.Engine.Numerics.Polynomials.PLib.RationalDerivative(degree, n, dimension, dc_poles, v, false);
            Traits.CoordsToVectorDirect(out vn, v);
        }
        else
        {
            if (n > degree)
            {
                Traits.NullifyVector(out vn);
            }
            else
            {
                ReadOnlySpan<double> dn = dc_poles.Slice(n * dimension);
                Traits.CoordsToVectorDirect(out vn, dn);
            }
        }
    }

    // Solves a banded system of equations for B-spline poles
    // @param[in,out] PolesArray - poles array containing right-hand side and receiving solution
    // @returns error status
    internal static int BSplCLib_SolveBandedSystem<Point, Vector, Traits>(Matrix matrix,
                                                                         int upperBandWidth,
                                                                         int lowerBandWidth,
                                                                         NCollection_Array1<Point> polesArray)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        Span<double> pArray;
        pArray = MemoryMarshal.Cast<Point, double>(polesArray.AsSpan());

        return global::Kerf.Engine.Geometry.Splines.BSplCLib.SolveBandedSystem(matrix, upperBandWidth, lowerBandWidth, Traits.Dimension, pArray);
    }

    // Solves a banded system of equations for rational B-spline poles with weights
    // @returns error status
    internal static int BSplCLib_SolveBandedSystem<Point, Vector, Traits>(Matrix matrix,
                                                                         int upperBandWidth,
                                                                         int lowerBandWidth,
                                                                         bool homogeneousFlag,
                                                                         NCollection_Array1<Point> polesArray,
                                                                         NCollection_Array1<double> weightsArray)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        Span<double> pArray, wArray;
        pArray = MemoryMarshal.Cast<Point, double>(polesArray.AsSpan());
        wArray = weightsArray.AsSpan();
        return global::Kerf.Engine.Geometry.Splines.BSplCLib.SolveBandedSystem(matrix,
                                          upperBandWidth,
                                          lowerBandWidth,
                                          homogeneousFlag,
                                          Traits.Dimension,
                                          pArray,
                                          wArray);
    }

    // Evaluates the B-spline curve at a given parameter
    // @param[in,out] ExtrapMode - extrapolation mode
    // @param[out] aPoint - resulting point
    // @param[out] aWeight - resulting weight
    internal static void BSplCLib_Eval<Point, Vector, Traits>(double parameter,
                                                             bool periodicFlag,
                                                             bool homogeneousFlag,
                                                             ReadOnlySpan<int> extrapMode,
                                                             int degree,
                                                             NCollection_Array1<double> flatKnots,
                                                             NCollection_Array1<Point> polesArray,
                                                             NCollection_Array1<double> weightsArray,
                                                             out Point aPoint,
                                                             out double aWeight)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int dimension = Traits.Dimension;
        double inverse;
        Span<double> p = stackalloc double[dimension];
        Span<double> weightBuf = stackalloc double[1]; // Note: OCCT passes &aWeight as a one-value result array
        ReadOnlySpan<double> pArray, wArray;
        int kk;
        pArray = MemoryMarshal.Cast<Point, double>(polesArray.AsSpan());
        wArray = weightsArray.AsSpan();
        if (homogeneousFlag)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(parameter,
                          periodicFlag,
                          0,
                          extrapMode,
                          degree,
                          flatKnots,
                          dimension,
                          pArray,
                          p);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(parameter,
                          periodicFlag,
                          0,
                          extrapMode,
                          degree,
                          flatKnots,
                          1,
                          wArray,
                          weightBuf);
            aWeight = weightBuf[0];
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Eval(parameter,
                          periodicFlag,
                          0,
                          extrapMode,
                          degree,
                          flatKnots,
                          dimension,
                          pArray,
                          wArray,
                          p,
                          weightBuf);
            aWeight = weightBuf[0];
            inverse = 1.0e0 / aWeight;

            for (kk = 0; kk < dimension; kk++)
            {
                p[kk] *= inverse;
            }
        }

        // C++ sets each coordinate of the caller's point; every coordinate is written
        Traits.NullifyPoint(out aPoint);
        for (kk = 0; kk < dimension; kk++)
        {
            Traits.SetCoord(ref aPoint, kk + 1, p[kk]);
        }
    }

    // Evaluates the point on cached B-spline curve at parameter
    internal static void BSplCLib_CacheD0<Point, Vector, Traits>(double parameter,
                                                                int degree,
                                                                double cacheParameter,
                                                                double spanLenght,
                                                                NCollection_Array1<Point> polesArray,
                                                                NCollection_Array1<double>? weightsArray,
                                                                out Point aPoint)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int dimension = Traits.Dimension;
        double newParameter, Inverse;
        ReadOnlySpan<double> pArray = MemoryMarshal.Cast<Point, double>(polesArray.AsSpan());
        Span<double> myPoint = stackalloc double[dimension]; // Note: OCCT writes through (double*)&aPoint
        newParameter = (parameter - cacheParameter) / spanLenght;
        global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(newParameter,
                                        degree,
                                        dimension,
                                        degree * dimension,
                                        pArray,
                                        myPoint);
        if (weightsArray != null)
        {
            NCollection_Array1<double> refWeights = weightsArray;
            ReadOnlySpan<double> wArray = refWeights.AsSpan();
            Span<double> inverse = stackalloc double[1];
            global::Kerf.Engine.Numerics.Polynomials.PLib.NoDerivativeEvalPolynomial(newParameter, degree, 1, degree, wArray, inverse);
            Inverse = inverse[0];

            Inverse = 1.0e0 / Inverse;
            Traits.ModifyCoordsScale(myPoint, Inverse);
        }
        Traits.CoordsToPointDirect(out aPoint, myPoint);
    }

    // Evaluates the point and first derivative on cached B-spline curve at parameter
    internal static void BSplCLib_CacheD1<Point, Vector, Traits>(double parameter,
                                                                int degree,
                                                                double cacheParameter,
                                                                double spanLenght,
                                                                NCollection_Array1<Point> polesArray,
                                                                NCollection_Array1<double>? weightsArray,
                                                                out Point aPoint,
                                                                out Vector aVector)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int dimension = Traits.Dimension;
        Span<double> localPDerivatives = stackalloc double[dimension << 1];
        Span<double> localWDerivatives = stackalloc double[2];
        double newParameter;

        ReadOnlySpan<double> pArray = MemoryMarshal.Cast<Point, double>(polesArray.AsSpan());
        newParameter = (parameter - cacheParameter) / spanLenght;
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(newParameter, 1, degree, dimension, pArray, localPDerivatives);

        Traits.ModifyCoordsDivide(localPDerivatives.Slice(dimension), spanLenght);

        if (weightsArray != null)
        {
            NCollection_Array1<double> refWeights = weightsArray;
            ReadOnlySpan<double> wArray = refWeights.AsSpan();
            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(newParameter, 1, degree, 1, wArray, localWDerivatives);

            localWDerivatives[1] /= spanLenght;

            global::Kerf.Engine.Numerics.Polynomials.PLib.RationalDerivatives(1,
                                     dimension,
                                     localPDerivatives,
                                     localWDerivatives,
                                     localPDerivatives);
        }

        Traits.CoordsToPointDirect(out aPoint, localPDerivatives);
        Traits.CoordsToVectorDirect(out aVector, localPDerivatives.Slice(dimension));
    }

    // Evaluates the point and first two derivatives on cached B-spline curve at parameter
    internal static void BSplCLib_CacheD2<Point, Vector, Traits>(double parameter,
                                                                int degree,
                                                                double cacheParameter,
                                                                double spanLenght,
                                                                NCollection_Array1<Point> polesArray,
                                                                NCollection_Array1<double>? weightsArray,
                                                                out Point aPoint,
                                                                out Vector aVector1,
                                                                out Vector aVector2)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int dimension = Traits.Dimension;
        int ii, index, endIndex;
        Span<double> localPDerivatives = stackalloc double[(dimension << 1) + dimension];
        Span<double> localWDerivatives = stackalloc double[3];
        double newParameter, factor;
        ReadOnlySpan<double> pArray = MemoryMarshal.Cast<Point, double>(polesArray.AsSpan());
        newParameter = (parameter - cacheParameter) / spanLenght;
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(newParameter, 2, degree, dimension, pArray, localPDerivatives);

        factor = 1.0e0 / spanLenght;
        index = dimension;
        endIndex = Min(2, degree);

        for (ii = 1; ii <= endIndex; ii++)
        {
            Traits.ModifyCoordsScale(localPDerivatives.Slice(index), factor);
            factor /= spanLenght;
            index += dimension;
        }

        index = (degree + 1) * dimension;
        for (ii = degree; ii < 2; ii++)
        {
            Traits.NullifyCoords(localPDerivatives.Slice(index));
            index += dimension;
        }

        if (weightsArray != null)
        {
            NCollection_Array1<double> refWeights = weightsArray;
            ReadOnlySpan<double> wArray = refWeights.AsSpan();

            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(newParameter, 2, degree, 1, wArray, localWDerivatives);

            for (ii = degree + 1; ii <= 2; ii++)
            {
                localWDerivatives[ii] = 0.0e0;
            }

            factor = 1.0e0 / spanLenght;

            for (ii = 1; ii <= endIndex; ii++)
            {
                localWDerivatives[ii] *= factor;
                factor /= spanLenght;
            }
            global::Kerf.Engine.Numerics.Polynomials.PLib.RationalDerivatives(2,
                                     dimension,
                                     localPDerivatives,
                                     localWDerivatives,
                                     localPDerivatives);
        }

        Traits.CoordsToPointDirect(out aPoint, localPDerivatives);
        Traits.CoordsToVectorDirect(out aVector1, localPDerivatives.Slice(dimension));
        Traits.CoordsToVectorDirect(out aVector2, localPDerivatives.Slice(dimension * 2));
    }

    // Evaluates the point and first three derivatives on cached B-spline curve at parameter
    internal static void BSplCLib_CacheD3<Point, Vector, Traits>(double parameter,
                                                                int degree,
                                                                double cacheParameter,
                                                                double spanLenght,
                                                                NCollection_Array1<Point> polesArray,
                                                                NCollection_Array1<double>? weightsArray,
                                                                out Point aPoint,
                                                                out Vector aVector1,
                                                                out Vector aVector2,
                                                                out Vector aVector3)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int dimension = Traits.Dimension;
        int ii, index, endIndex;
        Span<double> localPDerivatives = stackalloc double[dimension << 2];
        Span<double> localWDerivatives = stackalloc double[4];
        double factor, newParameter;
        ReadOnlySpan<double> pArray = MemoryMarshal.Cast<Point, double>(polesArray.AsSpan());
        newParameter = (parameter - cacheParameter) / spanLenght;
        global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(newParameter, 3, degree, dimension, pArray, localPDerivatives);

        index = (degree + 1) * dimension;
        for (ii = degree; ii < 3; ii++)
        {
            Traits.NullifyCoords(localPDerivatives.Slice(index));
            index += dimension;
        }

        factor = 1.0e0 / spanLenght;
        index = dimension;
        endIndex = Min(3, degree);

        for (ii = 1; ii <= endIndex; ii++)
        {
            Traits.ModifyCoordsScale(localPDerivatives.Slice(index), factor);
            factor /= spanLenght;
            index += dimension;
        }

        if (weightsArray != null)
        {
            NCollection_Array1<double> refWeights = weightsArray;
            ReadOnlySpan<double> wArray = refWeights.AsSpan();

            global::Kerf.Engine.Numerics.Polynomials.PLib.EvalPolynomial(newParameter, 3, degree, 1, wArray, localWDerivatives);

            factor = 1.0e0 / spanLenght;

            for (ii = 1; ii <= endIndex; ii++)
            {
                localWDerivatives[ii] *= factor;
                factor /= spanLenght;
            }

            for (ii = (degree + 1); ii <= 3; ii++)
            {
                localWDerivatives[ii] = 0.0e0;
            }
            global::Kerf.Engine.Numerics.Polynomials.PLib.RationalDerivatives(3,
                                     dimension,
                                     localPDerivatives,
                                     localWDerivatives,
                                     localPDerivatives);
        }

        Traits.CoordsToPointDirect(out aPoint, localPDerivatives);
        Traits.CoordsToVectorDirect(out aVector1, localPDerivatives.Slice(dimension));
        Traits.CoordsToVectorDirect(out aVector2, localPDerivatives.Slice(dimension * 2));
        Traits.CoordsToVectorDirect(out aVector3, localPDerivatives.Slice(dimension * 3));
    }

    // Builds cache for efficient B-spline evaluation (Point version)
    internal static void BSplCLib_BuildCache<Point, Vector, Traits>(double U,
                                                                   double spanDomain,
                                                                   bool periodic,
                                                                   int degree,
                                                                   NCollection_Array1<double> flatKnots,
                                                                   NCollection_Array1<Point> poles,
                                                                   NCollection_Array1<double>? weights,
                                                                   NCollection_Array1<Point> cachePoles,
                                                                   NCollection_Array1<double>? cacheWeights)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int dimension = Traits.Dimension;
        int ii, localDimension, localIndex, index = 0;
        double u = U, localValue;
        bool rational;

        ValidateBSplineDegree(degree);
        Span<double> dc_poles = stackalloc double[DataContainerPolesSize(dimension)];
        Span<double> dc_knots = stackalloc double[DataContainerKnotsSize];
        PrepareEval_T<Point, Vector, Traits>(ref u,
                                             ref index,
                                             out localDimension,
                                             out rational,
                                             degree,
                                             periodic,
                                             poles,
                                             weights,
                                             flatKnots,
                                             (global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults()),
                                             dc_poles,
                                             dc_knots);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(u, degree, degree, dc_knots, localDimension, dc_poles);

        localValue = 1.0e0;
        localIndex = 0;

        if (rational)
        {

            for (ii = 1; ii <= degree + 1; ii++)
            {
                Traits.CoordsToPointMultiplied(out cachePoles[ii], dc_poles.Slice(localIndex), localValue);
                localIndex += dimension + 1;
                localValue *= spanDomain / (double)ii;
            }

            localIndex = dimension;
            localValue = 1.0e0;
            for (ii = 1; ii <= degree + 1; ii++)
            {
                cacheWeights![ii] = dc_poles[localIndex] * localValue;
                localIndex += dimension + 1;
                localValue *= spanDomain / (double)ii;
            }
        }
        else
        {

            for (ii = 1; ii <= degree + 1; ii++)
            {
                Traits.CoordsToPointMultiplied(out cachePoles[ii], dc_poles.Slice(localIndex), localValue);
                localIndex += dimension;
                localValue *= spanDomain / (double)ii;
            }

            if (weights != null)
            {
                for (ii = 1; ii <= degree + 1; ii++)
                {
                    cacheWeights![ii] = 0.0e0;
                }
                cacheWeights![1] = 1.0e0;
            }
        }
    }

    // Builds cache for efficient B-spline evaluation (Array2 version)
    // @param[out] theCacheArray - 2D cache array
    internal static void BSplCLib_BuildCache<Point, Vector, Traits>(double parameter,
                                                                   double spanDomain,
                                                                   bool periodicFlag,
                                                                   int degree,
                                                                   int spanIndex,
                                                                   NCollection_Array1<double> flatKnots,
                                                                   NCollection_Array1<Point> poles,
                                                                   NCollection_Array1<double>? weights,
                                                                   NCollection_Array2<double> cacheArray)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        double param = parameter;
        int index = spanIndex;
        int dimension;
        bool isRational;

        ValidateBSplineDegree(degree);
        Span<double> dc_poles = stackalloc double[DataContainerPolesSize(Traits.Dimension)];
        Span<double> dc_knots = stackalloc double[DataContainerKnotsSize];
        PrepareEval_T<Point, Vector, Traits>(ref param,
                                             ref index,
                                             out dimension,
                                             out isRational,
                                             degree,
                                             periodicFlag,
                                             poles,
                                             weights,
                                             flatKnots,
                                             (global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults()),
                                             dc_poles,
                                             dc_knots);

        int cacheShift = (weights != null && !isRational) ? dimension + 1 : dimension;

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Bohm(param, degree, degree, dc_knots, dimension, dc_poles);

        double coeff = 1.0;
        // the cache array's storage from (LowerRow, LowerCol), row after row
        Span<double> aCacheArray = cacheArray.AsSpan();
        int cache = 0;
        int polyCoeffs = 0;

        for (int i = 0; i <= degree; i++)
        {
            for (int j = 0; j < dimension; j++)
            {
                aCacheArray[cache + j] = dc_poles[polyCoeffs + j] * coeff;
            }
            coeff *= spanDomain / (i + 1);
            polyCoeffs += dimension;
            cache += dimension;
            if (cacheShift > dimension)
            {
                aCacheArray[cache] = 0.0;
                cache++;
            }
        }

        if (cacheShift > dimension)
        {
            cacheArray.SetValue(cacheArray.LowerRow(),
                                   cacheArray.LowerCol() + cacheShift - 1,
                                   1.0);
        }
    }

    // Interpolates points to create a B-spline curve
    // @param[in,out] Poles - poles array to be computed
    // @param[out] InversionProblem - status of matrix inversion
    internal static void BSplCLib_Interpolate<Point, Vector, Traits>(int degree,
                                                                    NCollection_Array1<double> flatKnots,
                                                                    NCollection_Array1<double> parameters,
                                                                    NCollection_Array1<int> contactOrderArray,
                                                                    NCollection_Array1<Point> poles,
                                                                    out int inversionProblem)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        Span<double> pArray;

        pArray = MemoryMarshal.Cast<Point, double>(poles.AsSpan());

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree,
                             flatKnots,
                             parameters,
                             contactOrderArray,
                             Traits.Dimension,
                             pArray,
                             out inversionProblem);
    }

    // Interpolates weighted points to create a rational B-spline curve
    internal static void BSplCLib_Interpolate<Point, Vector, Traits>(int degree,
                                                                    NCollection_Array1<double> flatKnots,
                                                                    NCollection_Array1<double> parameters,
                                                                    NCollection_Array1<int> contactOrderArray,
                                                                    NCollection_Array1<Point> poles,
                                                                    NCollection_Array1<double> weights,
                                                                    out int inversionProblem)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        Span<double> pArray, wArray;
        pArray = MemoryMarshal.Cast<Point, double>(poles.AsSpan());
        wArray = weights.AsSpan();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Interpolate(degree,
                             flatKnots,
                             parameters,
                             contactOrderArray,
                             Traits.Dimension,
                             pArray,
                             wArray,
                             out inversionProblem);
    }

    // Moves a point on the B-spline curve by modifying poles
    // @param[out] FirstIndex - first modified pole index
    // @param[out] LastIndex - last modified pole index
    internal static void BSplCLib_MovePoint<Point, Vector, Traits>(double u,
                                                                  in Vector displ,
                                                                  int index1,
                                                                  int index2,
                                                                  int degree,
                                                                  NCollection_Array1<Point> poles,
                                                                  NCollection_Array1<double>? weights,
                                                                  NCollection_Array1<double> flatKnots,
                                                                  out int firstIndex,
                                                                  out int lastIndex,
                                                                  NCollection_Array1<Point> newPoles)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        // calculate the BSplineBasis in the parameter U
        int firstNonZeroBsplineIndex;
        Matrix bSplineBasis = new(1, 1, 1, degree + 1);
        int errorCode =
          global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(0, degree + 1, flatKnots, u, out firstNonZeroBsplineIndex, bSplineBasis);
        if (errorCode != 0)
        {
            firstIndex = 0;
            lastIndex = 0;

            for (int k = poles.Lower(); k <= poles.Upper(); k++) // i in OCCT
            {
                newPoles[k] = poles[k];
            }
            return;
        }

        // find the span which is predominant for parameter U
        firstIndex = firstNonZeroBsplineIndex;
        lastIndex = firstNonZeroBsplineIndex + degree;
        if (firstIndex < index1)
        {
            firstIndex = index1;
        }
        if (lastIndex > index2)
        {
            lastIndex = index2;
        }

        double maxValue = 0.0;
        int i, kk1 = 0, kk2, ii;

        for (i = firstIndex - firstNonZeroBsplineIndex + 1; i <= lastIndex - firstNonZeroBsplineIndex + 1;
             i++)
        {
            if (bSplineBasis[1, i] > maxValue)
            {
                kk1 = i + firstNonZeroBsplineIndex - 1;
                maxValue = bSplineBasis[1, i];
            }
        }

        // find a kk2 if symmetry
        kk2 = kk1;
        i = kk1 - firstNonZeroBsplineIndex + 2;
        if ((kk1 + 1) <= lastIndex)
        {
            if (Math.Abs(bSplineBasis[1, kk1 - firstNonZeroBsplineIndex + 2] - maxValue) < 1.0e-10)
            {
                kk2 = kk1 + 1;
            }
        }

        // compute the vector of displacement
        double d1 = 0.0;
        double d2 = 0.0;
        double hN, coef, dval;

        for (i = 1; i <= degree + 1; i++)
        {
            ii = i + firstNonZeroBsplineIndex - 1;
            if (weights != null)
            {
                hN = weights.Value(ii) * bSplineBasis[1, i];
                d2 += hN;
            }
            else
            {
                hN = bSplineBasis[1, i];
            }
            if (ii >= firstIndex && ii <= lastIndex)
            {
                if (ii < kk1)
                {
                    dval = kk1 - ii;
                }
                else if (ii > kk2)
                {
                    dval = ii - kk2;
                }
                else
                {
                    dval = 0.0;
                }
                d1 += 1.0 / (dval + 1.0) * hN;
            }
        }

        if (weights != null)
        {
            coef = d2 / d1;
        }
        else
        {
            coef = 1.0 / d1;
        }

        // compute the new poles

        for (i = poles.Lower(); i <= poles.Upper(); i++)
        {
            if (i >= firstIndex && i <= lastIndex)
            {
                if (i < kk1)
                {
                    dval = kk1 - i;
                }
                else if (i > kk2)
                {
                    dval = i - kk2;
                }
                else
                {
                    dval = 0.0;
                }
                newPoles[i] = Traits.Translated(poles[i], (coef / (dval + 1.0)), displ);
            }
            else
            {
                newPoles[i] = poles[i];
            }
        }
    }

    // Moves a point and tangent on the B-spline curve by modifying poles
    // @param[out] ErrorStatus - error status of the operation
    internal static void BSplCLib_MovePointAndTangent<Point, Vector, Traits>(double u,
                                                                            in Vector delta,
                                                                            in Vector deltaDerivatives,
                                                                            double tolerance,
                                                                            int degree,
                                                                            int startingCondition,
                                                                            int endingCondition,
                                                                            NCollection_Array1<Point> poles,
                                                                            NCollection_Array1<double>? weights,
                                                                            NCollection_Array1<double> flatKnots,
                                                                            NCollection_Array1<Point> newPoles,
                                                                            out int errorStatus)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int dimension = Traits.Dimension;
        Span<double> delta_array = stackalloc double[dimension];
        Span<double> delta_derivative_array = stackalloc double[dimension];
        ReadOnlySpan<double> poles_array;
        Span<double> new_poles_array;

        int num_poles;
        num_poles = poles.Length();

        if (newPoles.Length() != num_poles)
        {
            throw new ConstructionErrorException();
        }
        Traits.VectorToCoords(delta_array, delta);
        Traits.VectorToCoords(delta_derivative_array, deltaDerivatives);
        poles_array = MemoryMarshal.Cast<Point, double>(poles.AsSpan());

        new_poles_array = MemoryMarshal.Cast<Point, double>(newPoles.AsSpan());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.MovePointAndTangent(u,
                                     dimension,
                                     delta_array,
                                     delta_derivative_array,
                                     tolerance,
                                     degree,
                                     startingCondition,
                                     endingCondition,
                                     poles_array,
                                     weights,
                                     flatKnots,
                                     new_poles_array,
                                     out errorStatus);
    }

    // Computes the parametric tolerance corresponding to a 3D tolerance
    // @param[out] UTolerance - computed parametric tolerance
    internal static void BSplCLib_Resolution<Point, Vector, Traits>(NCollection_Array1<Point> poles,
                                                                   NCollection_Array1<double>? weights,
                                                                   int numPoles,
                                                                   NCollection_Array1<double> flatKnots,
                                                                   int degree,
                                                                   double tolerance3D,
                                                                   out double uTolerance)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        ReadOnlySpan<double> polesArray;
        polesArray = MemoryMarshal.Cast<Point, double>(poles.AsSpan());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Resolution(polesArray,
                            Traits.Dimension,
                            numPoles,
                            weights,
                            flatKnots,
                            degree,
                            tolerance3D,
                            out uTolerance);
    }

    // Multiplies the B-spline curve by a function
    // @param[out] theStatus - status of the operation
    internal static void BSplCLib_FunctionMultiply<Point, Vector, Traits>(EvaluatorFunction functionPtr,
                                                                         int bSplineDegree,
                                                                         NCollection_Array1<double> bSplineFlatKnots,
                                                                         NCollection_Array1<Point> poles,
                                                                         NCollection_Array1<double> flatKnots,
                                                                         int newDegree,
                                                                         NCollection_Array1<Point> newPoles,
                                                                         out int status)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int num_bspline_poles = bSplineFlatKnots.Length() - bSplineDegree - 1;
        int num_new_poles = flatKnots.Length() - newDegree - 1;

        if (poles.Length() != num_bspline_poles || newPoles.Length() != num_new_poles)
        {
            throw new ConstructionErrorException();
        }
        ReadOnlySpan<double> array_of_poles = MemoryMarshal.Cast<Point, double>(poles.AsSpan());
        Span<double> array_of_new_poles = MemoryMarshal.Cast<Point, double>(newPoles.AsSpan());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionMultiply(functionPtr,
                                  bSplineDegree,
                                  bSplineFlatKnots,
                                  Traits.Dimension,
                                  array_of_poles,
                                  flatKnots,
                                  newDegree,
                                  array_of_new_poles,
                                  out status);
    }

    // Reparameterizes the B-spline curve using a function
    // @param[out] theStatus - status of the operation
    internal static void BSplCLib_FunctionReparameterise<Point, Vector, Traits>(EvaluatorFunction functionPtr,
                                                                               int bSplineDegree,
                                                                               NCollection_Array1<double> bSplineFlatKnots,
                                                                               NCollection_Array1<Point> poles,
                                                                               NCollection_Array1<double> flatKnots,
                                                                               int newDegree,
                                                                               NCollection_Array1<Point> newPoles,
                                                                               out int status)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int num_bspline_poles = bSplineFlatKnots.Length() - bSplineDegree - 1;
        int num_new_poles = flatKnots.Length() - newDegree - 1;

        if (poles.Length() != num_bspline_poles || newPoles.Length() != num_new_poles)
        {
            throw new ConstructionErrorException();
        }
        ReadOnlySpan<double> array_of_poles = MemoryMarshal.Cast<Point, double>(poles.AsSpan());
        Span<double> array_of_new_poles = MemoryMarshal.Cast<Point, double>(newPoles.AsSpan());
        global::Kerf.Engine.Geometry.Splines.BSplCLib.FunctionReparameterise(functionPtr,
                                        bSplineDegree,
                                        bSplineFlatKnots,
                                        Traits.Dimension,
                                        array_of_poles,
                                        flatKnots,
                                        newDegree,
                                        array_of_new_poles,
                                        out status);
    }

    // Computes coefficients of a Bezier curve from poles (Bezier syntax wrapper)
    internal static void BSplCLib_PolesCoefficients_Bezier<Point, Vector, Traits>(NCollection_Array1<Point> poles,
                                                                                 NCollection_Array1<double>? weights,
                                                                                 NCollection_Array1<Point> cachePoles,
                                                                                 NCollection_Array1<double>? cacheWeights)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int degree = poles.Length() - 1;
        // Note: OCCT wraps the static table without copying; the port copies the 2 * (aDegree + 1) knots
        NCollection_Array1<double> bidFlatKnots =
          NCollection_Array1<double>.From(global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatBezierKnots(degree).Slice(0, 2 * (degree + 1)), 1);
        BSplCLib_BuildCache<Point, Vector, Traits>(0.0,
                                                   1.0,
                                                   false,
                                                   degree,
                                                   bidFlatKnots,
                                                   poles,
                                                   weights,
                                                   cachePoles,
                                                   cacheWeights);
    }

    // Increases the degree of a Bezier curve (Bezier syntax wrapper)
    internal static void BSplCLib_IncreaseDegree_Bezier<Point, Vector, Traits>(int newDegree,
                                                                              NCollection_Array1<Point> poles,
                                                                              NCollection_Array1<double>? weights,
                                                                              NCollection_Array1<Point> newPoles,
                                                                              NCollection_Array1<double>? newWeights)
        where Point : struct
        where Vector : struct
        where Traits : struct, ICurveTraits<Point, Vector>
    {
        int degree = poles.Length() - 1;
        // fresh arrays: they are both the input and the output knots and multiplicities, as in OCCT
        KnotArrays(degree, out NCollection_Array1<double> bezierKnots_Knot, out NCollection_Array1<int> bezierKnots_Mult);
        BSplCLib_IncreaseDegree<Point, Vector, Traits>(degree,
                                                       newDegree,
                                                       false,
                                                       poles,
                                                       weights,
                                                       bezierKnots_Knot,
                                                       bezierKnots_Mult,
                                                       newPoles,
                                                       newWeights,
                                                       bezierKnots_Knot,
                                                       bezierKnots_Mult);
    }
}
