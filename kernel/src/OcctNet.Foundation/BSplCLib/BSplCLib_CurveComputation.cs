// Port of: src/FoundationClasses/TKMath/BSplCLib/BSplCLib_CurveComputation.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.InteropServices;

namespace Occt;

// OCCT's template over <Point, Vector, Array1OfPoints, Dimension> is ported once, as generic methods over the
// point and vector types and a traits type. The traits are static abstract members of an interface implemented by
// two empty structs (2D and 3D), so the JIT specialises each instantiation as the C++ compiler does. Besides
// OCCT's traits operations, the interface carries the calls the template resolves by overload at instantiation
// (PLib::SetPoles/GetPoles, SetCoord, Translated). Arrays of points are read as arrays of doubles with
// MemoryMarshal.Cast, as OCCT casts them with (double*); a point or vector OCCT writes through a (double*) is
// computed in a stack buffer and then built from it.

/// <summary>The point and vector operations the B-spline curve templates need, for one dimension.</summary>
internal interface IBSplCLib_CurveTraits<Point, Vector>
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
    static abstract void SetCoord(ref Point pnt, int theIndex, double theXi);

    /// <summary><c>pnt.Translated(theScale * theVec)</c>.</summary>
    static abstract Point Translated(in Point pnt, double theScale, in Vector theVec);

    static abstract void SetPoles(NCollection_Array1<Point> Poles, NCollection_Array1<double> FP);

    static abstract void SetPoles(NCollection_Array1<Point> Poles, NCollection_Array1<double> Weights, NCollection_Array1<double> FP);

    static abstract void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<Point> Poles);

    static abstract void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<Point> Poles, NCollection_Array1<double> Weights);
}

/// <summary>The curve traits for 3D: gp_Pnt, gp_Vec, dimension 3.</summary>
internal readonly struct BSplCLib_CurveTraits3d : IBSplCLib_CurveTraits<gp_Pnt, gp_Vec>
{
    public static int Dimension => 3;

    public static void PointToCoords(Span<double> carr, in gp_Pnt pnt, double op)
    {
        carr[0] = pnt.X() + op;
        carr[1] = pnt.Y() + op;
        carr[2] = pnt.Z() + op;
    }

    public static void PointToCoordsScaled(Span<double> carr, in gp_Pnt pnt, double scale)
    {
        carr[0] = pnt.X() * scale;
        carr[1] = pnt.Y() * scale;
        carr[2] = pnt.Z() * scale;
    }

    public static void CoordsToPointMultiplied(out gp_Pnt pnt, ReadOnlySpan<double> carr, double factor)
        => pnt = new gp_Pnt(carr[0] * factor, carr[1] * factor, carr[2] * factor);

    public static void CoordsToPointScaled(out gp_Pnt pnt, ReadOnlySpan<double> carr, double scale)
        => pnt = new gp_Pnt(carr[0] / scale, carr[1] / scale, carr[2] / scale);

    public static void CoordsToPointDirect(out gp_Pnt pnt, ReadOnlySpan<double> carr)
        => pnt = new gp_Pnt(carr[0], carr[1], carr[2]);

    public static void CoordsToVectorDirect(out gp_Vec vec, ReadOnlySpan<double> carr)
        => vec = new gp_Vec(carr[0], carr[1], carr[2]);

    public static void NullifyPoint(out gp_Pnt pnt) => pnt = new gp_Pnt(0.0, 0.0, 0.0);

    public static void NullifyVector(out gp_Vec vec) => vec = new gp_Vec(0.0, 0.0, 0.0);

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

    public static void VectorToCoords(Span<double> carr, in gp_Vec vec)
    {
        carr[0] = vec.X();
        carr[1] = vec.Y();
        carr[2] = vec.Z();
    }

    public static void SetCoord(ref gp_Pnt pnt, int theIndex, double theXi) => pnt.SetCoord(theIndex, theXi);

    public static gp_Pnt Translated(in gp_Pnt pnt, double theScale, in gp_Vec theVec) => pnt.Translated(theScale * theVec);

    public static void SetPoles(NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double> FP) => PLib.SetPoles(Poles, FP);

    public static void SetPoles(NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double> Weights, NCollection_Array1<double> FP)
        => PLib.SetPoles(Poles, Weights, FP);

    public static void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<gp_Pnt> Poles) => PLib.GetPoles(FP, Poles);

    public static void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double> Weights)
        => PLib.GetPoles(FP, Poles, Weights);
}

/// <summary>The curve traits for 2D: gp_Pnt2d, gp_Vec2d, dimension 2.</summary>
internal readonly struct BSplCLib_CurveTraits2d : IBSplCLib_CurveTraits<gp_Pnt2d, gp_Vec2d>
{
    public static int Dimension => 2;

    public static void PointToCoords(Span<double> carr, in gp_Pnt2d pnt, double op)
    {
        carr[0] = pnt.X() + op;
        carr[1] = pnt.Y() + op;
    }

    public static void PointToCoordsScaled(Span<double> carr, in gp_Pnt2d pnt, double scale)
    {
        carr[0] = pnt.X() * scale;
        carr[1] = pnt.Y() * scale;
    }

    public static void CoordsToPointMultiplied(out gp_Pnt2d pnt, ReadOnlySpan<double> carr, double factor)
        => pnt = new gp_Pnt2d(carr[0] * factor, carr[1] * factor);

    public static void CoordsToPointScaled(out gp_Pnt2d pnt, ReadOnlySpan<double> carr, double scale)
        => pnt = new gp_Pnt2d(carr[0] / scale, carr[1] / scale);

    public static void CoordsToPointDirect(out gp_Pnt2d pnt, ReadOnlySpan<double> carr)
        => pnt = new gp_Pnt2d(carr[0], carr[1]);

    public static void CoordsToVectorDirect(out gp_Vec2d vec, ReadOnlySpan<double> carr)
        => vec = new gp_Vec2d(carr[0], carr[1]);

    public static void NullifyPoint(out gp_Pnt2d pnt) => pnt = new gp_Pnt2d(0.0, 0.0);

    public static void NullifyVector(out gp_Vec2d vec) => vec = new gp_Vec2d(0.0, 0.0);

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

    public static void VectorToCoords(Span<double> carr, in gp_Vec2d vec)
    {
        carr[0] = vec.X();
        carr[1] = vec.Y();
    }

    public static void SetCoord(ref gp_Pnt2d pnt, int theIndex, double theXi) => pnt.SetCoord(theIndex, theXi);

    public static gp_Pnt2d Translated(in gp_Pnt2d pnt, double theScale, in gp_Vec2d theVec) => pnt.Translated(theScale * theVec);

    public static void SetPoles(NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double> FP) => PLib.SetPoles(Poles, FP);

    public static void SetPoles(NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double> Weights, NCollection_Array1<double> FP)
        => PLib.SetPoles(Poles, Weights, FP);

    public static void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<gp_Pnt2d> Poles) => PLib.GetPoles(FP, Poles);

    public static void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double> Weights)
        => PLib.GetPoles(FP, Poles, Weights);
}

/// <summary>
/// The B-spline curve computations shared by the 2D and 3D overloads of <see cref="BSplCLib"/>
/// (OCCT's BSplCLib_CurveComputation.pxx templates).
/// </summary>
internal static class BSplCLib_CurveComputation
{
    //! Maximum supported degree for B-spline curves.
    internal const int THE_MAX_DEGREE = 25; // BSplCLib::MaxDegree()

    //! Validates that the given degree does not exceed the maximum supported degree.
    //! @throws Standard_OutOfRange if degree exceeds MaxDegree()
    internal static void validateBSplineDegree(int theDegree)
    {
        Standard_OutOfRange_Raise_if(theDegree > THE_MAX_DEGREE,
                                     "BSplCLib: bspline degree is greater than maximum supported");
    }

    // Sizes of OCCT's BSplCLib_DataContainer_T<Dimension>: stack buffers for poles, knots and derivatives
    // used in evaluation. The port allocates them with stackalloc in each evaluating function.
    private static int DataContainerPolesSize(int theDimension) => (THE_MAX_DEGREE + 1) * (theDimension + 1);

    private const int DataContainerKnotsSize = 2 * THE_MAX_DEGREE;

    private static int DataContainerDersSize(int theDimension) => theDimension * 4;

    //! Arrays for knot/multiplicity pairs (OCCT's BSplCLib_KnotArrays, stack-allocated there).
    //! Size=1: single knot (knotValue, multiplicity)
    //! Size=2: Bezier knots {0,1} and mults {degree+1, degree+1}
    internal static void KnotArrays(double theKnotValue, int theMultiplicity,
                                    out NCollection_Array1<double> Knot, out NCollection_Array1<int> Mult)
    {
        Knot = new NCollection_Array1<double>([theKnotValue], 0, 1, 1);
        Mult = new NCollection_Array1<int>([theMultiplicity], 0, 1, 1);
    }

    /// <inheritdoc cref="KnotArrays(double, int, out NCollection_Array1{double}, out NCollection_Array1{int})"/>
    internal static void KnotArrays(int theDegree, out NCollection_Array1<double> Knot, out NCollection_Array1<int> Mult)
    {
        Knot = new NCollection_Array1<double>([0.0, 1.0], 0, 1, 2);
        Mult = new NCollection_Array1<int>([theDegree + 1, theDegree + 1], 0, 1, 2);
    }

    // Bezier knot arrays shared by the read-only uses (the Bezier D0..D3), one pair per degree, so that
    // evaluating a Bezier curve doesn't allocate where OCCT uses the stack. Never written.
    private static readonly NCollection_Array1<double>[] THE_BEZIER_KNOTS = BuildBezierKnots();
    private static readonly NCollection_Array1<int>[] THE_BEZIER_MULTS = BuildBezierMults();

    private static NCollection_Array1<double>[] BuildBezierKnots()
    {
        var aKnots = new NCollection_Array1<double>[THE_MAX_DEGREE + 1];
        for (int i = 0; i <= THE_MAX_DEGREE; i++)
        {
            aKnots[i] = new NCollection_Array1<double>([0.0, 1.0], 0, 1, 2);
        }
        return aKnots;
    }

    private static NCollection_Array1<int>[] BuildBezierMults()
    {
        var aMults = new NCollection_Array1<int>[THE_MAX_DEGREE + 1];
        for (int i = 0; i <= THE_MAX_DEGREE; i++)
        {
            aMults[i] = new NCollection_Array1<int>([i + 1, i + 1], 0, 1, 2);
        }
        return aMults;
    }

    /// <summary>
    /// Bezier knots {0, 1} and multiplicities {Degree + 1, Degree + 1} for read-only use: shared arrays for
    /// degrees up to <see cref="THE_MAX_DEGREE"/>, new ones above (the evaluation then rejects the degree).
    /// </summary>
    internal static void BezierKnotArrays(int theDegree, out NCollection_Array1<double> Knot, out NCollection_Array1<int> Mult)
    {
        if (theDegree >= 0 && theDegree <= THE_MAX_DEGREE)
        {
            Knot = THE_BEZIER_KNOTS[theDegree];
            Mult = THE_BEZIER_MULTS[theDegree];
        }
        else
        {
            KnotArrays(theDegree, out Knot, out Mult);
        }
    }

    // Reverses the order of elements in the array.
    // Works with any NCollection_Array1.
    // @param[in,out] theArray - array to be reversed
    // @param[in] theL - length parameter for reversal
    internal static void BSplCLib_Reverse<T>(NCollection_Array1<T> theArray, int theL)
    {
        int aLower = theArray.Lower();
        int aUpper = theArray.Upper();
        int aL = aLower + (theL - aLower) % theArray.Length();

        theArray.AsSpan().Slice(0, aL - aLower + 1).Reverse();
        if (aL < aUpper)
        {
            theArray.AsSpan(aL + 1).Reverse();
        }
    }

    // Removes a knot from the B-spline curve
    // @returns true if knot removal was successful
    internal static bool BSplCLib_RemoveKnot<Point, Vector, Traits>(int Index,
                                                                   int Mult,
                                                                   int Degree,
                                                                   bool Periodic,
                                                                   NCollection_Array1<Point> Poles,
                                                                   NCollection_Array1<double>? Weights,
                                                                   NCollection_Array1<double> Knots,
                                                                   NCollection_Array1<int> Mults,
                                                                   NCollection_Array1<Point> NewPoles,
                                                                   NCollection_Array1<double>? NewWeights,
                                                                   NCollection_Array1<double> NewKnots,
                                                                   NCollection_Array1<int> NewMults,
                                                                   double Tolerance)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        bool rational = Weights != null;
        int dim;
        dim = Traits.Dimension;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.Length());
        NCollection_Array1<double> newpoles = new(1, dim * NewPoles.Length());

        if (rational)
        {
            Traits.SetPoles(Poles, Weights!, poles);
        }
        else
        {
            Traits.SetPoles(Poles, poles);
        }

        if (!BSplCLib.RemoveKnot(Index,
                                 Mult,
                                 Degree,
                                 Periodic,
                                 dim,
                                 poles,
                                 Knots,
                                 Mults,
                                 newpoles,
                                 NewKnots,
                                 NewMults,
                                 Tolerance))
        {
            return false;
        }

        if (rational)
        {
            Traits.GetPoles(newpoles, NewPoles, NewWeights!);
        }
        else
        {
            Traits.GetPoles(newpoles, NewPoles);
        }
        return true;
    }

    // Inserts an array of knots and multiplicities into the B-spline curve
    internal static void BSplCLib_InsertKnots<Point, Vector, Traits>(int Degree,
                                                                    bool Periodic,
                                                                    NCollection_Array1<Point> Poles,
                                                                    NCollection_Array1<double>? Weights,
                                                                    NCollection_Array1<double> Knots,
                                                                    NCollection_Array1<int> Mults,
                                                                    NCollection_Array1<double> AddKnots,
                                                                    NCollection_Array1<int>? AddMults,
                                                                    NCollection_Array1<Point> NewPoles,
                                                                    NCollection_Array1<double>? NewWeights,
                                                                    NCollection_Array1<double> NewKnots,
                                                                    NCollection_Array1<int> NewMults,
                                                                    double Epsilon,
                                                                    bool Add)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        bool rational = Weights != null;
        int dim;
        dim = Traits.Dimension;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.Length());
        NCollection_Array1<double> newpoles = new(1, dim * NewPoles.Length());

        if (rational)
        {
            Traits.SetPoles(Poles, Weights!, poles);
        }
        else
        {
            Traits.SetPoles(Poles, poles);
        }

        BSplCLib.InsertKnots(Degree,
                             Periodic,
                             dim,
                             poles,
                             Knots,
                             Mults,
                             AddKnots,
                             AddMults,
                             newpoles,
                             NewKnots,
                             NewMults,
                             Epsilon,
                             Add);

        if (rational)
        {
            Traits.GetPoles(newpoles, NewPoles, NewWeights!);
        }
        else
        {
            Traits.GetPoles(newpoles, NewPoles);
        }
    }

    // Inserts a single knot into the B-spline curve
    internal static void BSplCLib_InsertKnot<Point, Vector, Traits>(int UIndex,
                                                                   double U,
                                                                   int UMult,
                                                                   int Degree,
                                                                   bool Periodic,
                                                                   NCollection_Array1<Point> Poles,
                                                                   NCollection_Array1<double>? Weights,
                                                                   NCollection_Array1<double> Knots,
                                                                   NCollection_Array1<int> Mults,
                                                                   NCollection_Array1<Point> NewPoles,
                                                                   NCollection_Array1<double>? NewWeights)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        _ = UIndex; // unused, as in OCCT
        KnotArrays(U, UMult, out NCollection_Array1<double> aSingleKnot_Knot, out NCollection_Array1<int> aSingleKnot_Mult);
        NCollection_Array1<double> aNewKnots = new(1, Knots.Length() + 1);
        NCollection_Array1<int> aNewMults = new(1, Knots.Length() + 1);
        BSplCLib_InsertKnots<Point, Vector, Traits>(Degree,
                                                    Periodic,
                                                    Poles,
                                                    Weights,
                                                    Knots,
                                                    Mults,
                                                    aSingleKnot_Knot,
                                                    aSingleKnot_Mult,
                                                    NewPoles,
                                                    NewWeights,
                                                    aNewKnots,
                                                    aNewMults,
                                                    Epsilon(U),
                                                    true);
    }

    // Raises the multiplicity of a knot in the B-spline curve
    internal static void BSplCLib_RaiseMultiplicity<Point, Vector, Traits>(int KnotIndex,
                                                                          int Mult,
                                                                          int Degree,
                                                                          bool Periodic,
                                                                          NCollection_Array1<Point> Poles,
                                                                          NCollection_Array1<double>? Weights,
                                                                          NCollection_Array1<double> Knots,
                                                                          NCollection_Array1<int> Mults,
                                                                          NCollection_Array1<Point> NewPoles,
                                                                          NCollection_Array1<double>? NewWeights)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        KnotArrays(Knots[KnotIndex], Mult - Mults[KnotIndex],
                   out NCollection_Array1<double> aSingleKnot_Knot, out NCollection_Array1<int> aSingleKnot_Mult);
        NCollection_Array1<double> aNewKnots = new(1, Knots.Length());
        NCollection_Array1<int> aNewMults = new(1, Knots.Length());
        BSplCLib_InsertKnots<Point, Vector, Traits>(Degree,
                                                    Periodic,
                                                    Poles,
                                                    Weights,
                                                    Knots,
                                                    Mults,
                                                    aSingleKnot_Knot,
                                                    aSingleKnot_Mult,
                                                    NewPoles,
                                                    NewWeights,
                                                    aNewKnots,
                                                    aNewMults,
                                                    Epsilon(Knots[KnotIndex]),
                                                    true);
    }

    // Increases the degree of the B-spline curve
    internal static void BSplCLib_IncreaseDegree<Point, Vector, Traits>(int Degree,
                                                                       int NewDegree,
                                                                       bool Periodic,
                                                                       NCollection_Array1<Point> Poles,
                                                                       NCollection_Array1<double>? Weights,
                                                                       NCollection_Array1<double> Knots,
                                                                       NCollection_Array1<int> Mults,
                                                                       NCollection_Array1<Point> NewPoles,
                                                                       NCollection_Array1<double>? NewWeights,
                                                                       NCollection_Array1<double> NewKnots,
                                                                       NCollection_Array1<int> NewMults)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        bool rational = Weights != null;
        int dim;
        dim = Traits.Dimension;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.Length());
        NCollection_Array1<double> newpoles = new(1, dim * NewPoles.Length());

        if (rational)
        {
            Traits.SetPoles(Poles, Weights!, poles);
        }
        else
        {
            Traits.SetPoles(Poles, poles);
        }

        BSplCLib.IncreaseDegree(Degree,
                                NewDegree,
                                Periodic,
                                dim,
                                poles,
                                Knots,
                                Mults,
                                newpoles,
                                NewKnots,
                                NewMults);

        if (rational)
        {
            Traits.GetPoles(newpoles, NewPoles, NewWeights!);
        }
        else
        {
            Traits.GetPoles(newpoles, NewPoles);
        }
    }

    // Converts a periodic B-spline curve to non-periodic
    internal static void BSplCLib_Unperiodize<Point, Vector, Traits>(int Degree,
                                                                    NCollection_Array1<int> Mults,
                                                                    NCollection_Array1<double> Knots,
                                                                    NCollection_Array1<Point> Poles,
                                                                    NCollection_Array1<double>? Weights,
                                                                    NCollection_Array1<int> NewMults,
                                                                    NCollection_Array1<double> NewKnots,
                                                                    NCollection_Array1<Point> NewPoles,
                                                                    NCollection_Array1<double>? NewWeights)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        bool rational = Weights != null;
        int dim;
        dim = Traits.Dimension;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.Length());
        NCollection_Array1<double> newpoles = new(1, dim * NewPoles.Length());

        if (rational)
        {
            Traits.SetPoles(Poles, Weights!, poles);
        }
        else
        {
            Traits.SetPoles(Poles, poles);
        }

        BSplCLib.Unperiodize(Degree, dim, Mults, Knots, poles, NewMults, NewKnots, newpoles);

        if (rational)
        {
            Traits.GetPoles(newpoles, NewPoles, NewWeights!);
        }
        else
        {
            Traits.GetPoles(newpoles, NewPoles);
        }
    }

    // Trims the B-spline curve to a parameter range
    internal static void BSplCLib_Trimming<Point, Vector, Traits>(int Degree,
                                                                 bool Periodic,
                                                                 NCollection_Array1<double> Knots,
                                                                 NCollection_Array1<int> Mults,
                                                                 NCollection_Array1<Point> Poles,
                                                                 NCollection_Array1<double>? Weights,
                                                                 double U1,
                                                                 double U2,
                                                                 NCollection_Array1<double> NewKnots,
                                                                 NCollection_Array1<int> NewMults,
                                                                 NCollection_Array1<Point> NewPoles,
                                                                 NCollection_Array1<double>? NewWeights)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        bool rational = Weights != null;
        int dim;
        dim = Traits.Dimension;
        if (rational)
        {
            dim++;
        }

        NCollection_Array1<double> poles = new(1, dim * Poles.Length());
        NCollection_Array1<double> newpoles = new(1, dim * NewPoles.Length());

        if (rational)
        {
            Traits.SetPoles(Poles, Weights!, poles);
        }
        else
        {
            Traits.SetPoles(Poles, poles);
        }

        BSplCLib.Trimming(Degree,
                          Periodic,
                          dim,
                          Knots,
                          Mults,
                          poles,
                          U1,
                          U2,
                          NewKnots,
                          NewMults,
                          newpoles);

        if (rational)
        {
            Traits.GetPoles(newpoles, NewPoles, NewWeights!);
        }
        else
        {
            Traits.GetPoles(newpoles, NewPoles);
        }
    }

    // Builds the local array for B-spline evaluation
    // @param[out] LP - local poles array for evaluation
    internal static void BSplCLib_BuildEval<Point, Vector, Traits>(int Degree,
                                                                  int Index,
                                                                  NCollection_Array1<Point> Poles,
                                                                  NCollection_Array1<double>? Weights,
                                                                  Span<double> LP)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int Dimension = Traits.Dimension;
        double w;
        int pole = 0;
        int PLower = Poles.Lower();
        int PUpper = Poles.Upper();
        int i;
        int ip = PLower + Index - 1;
        if (Weights == null)
        {
            for (i = 0; i <= Degree; i++)
            {
                ip++;
                if (ip > PUpper)
                {
                    ip = PLower;
                }
                ref readonly Point P = ref Poles[ip];
                Traits.PointToCoords(LP.Slice(pole), P, 0);
                pole += Dimension;
            }
        }
        else
        {
            for (i = 0; i <= Degree; i++)
            {
                ip++;
                if (ip > PUpper)
                {
                    ip = PLower;
                }
                ref readonly Point P = ref Poles[ip];
                LP[pole + Dimension] = w = Weights[ip];
                Traits.PointToCoordsScaled(LP.Slice(pole), P, w);
                pole += Dimension + 1;
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
                                                            int Degree,
                                                            bool Periodic,
                                                            NCollection_Array1<Point> Poles,
                                                            NCollection_Array1<double>? Weights,
                                                            NCollection_Array1<double> Knots,
                                                            NCollection_Array1<int>? Mults,
                                                            Span<double> dc_poles,
                                                            Span<double> dc_knots)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        // Set the Index
        BSplCLib.LocateParameter(Degree, Knots, Mults, u, Periodic, ref index, out u);

        // make the knots
        BSplCLib.BuildKnots(Degree, index, Periodic, Knots, Mults, dc_knots);
        if (Mults == null)
        {
            index -= Knots.Lower() + Degree;
        }
        else
        {
            index = BSplCLib.PoleIndex(Degree, index, Periodic, Mults);
        }

        // check truly rational
        rational = (Weights != null);
        if (rational)
        {
            int WLower = Weights!.Lower() + index;
            rational = BSplCLib.IsRational(Weights, WLower, WLower + Degree);
        }

        // make the poles
        if (rational)
        {
            dim = Traits.Dimension + 1;
            BSplCLib_BuildEval<Point, Vector, Traits>(Degree,
                                                      index,
                                                      Poles,
                                                      Weights,
                                                      dc_poles);
        }
        else
        {
            dim = Traits.Dimension;
            BSplCLib_BuildEval<Point, Vector, Traits>(Degree,
                                                      index,
                                                      Poles,
                                                      BSplCLib.NoWeights(),
                                                      dc_poles);
        }
    }

    // Evaluates the point on the B-spline curve at parameter U
    internal static void BSplCLib_D0<Point, Vector, Traits>(double U,
                                                           int Index,
                                                           int Degree,
                                                           bool Periodic,
                                                           NCollection_Array1<Point> Poles,
                                                           NCollection_Array1<double>? Weights,
                                                           NCollection_Array1<double> Knots,
                                                           NCollection_Array1<int>? Mults,
                                                           out Point P)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int Dimension = Traits.Dimension;
        int dim, index = Index;
        double u = U;
        bool rational;
        validateBSplineDegree(Degree);
        Span<double> dc_poles = stackalloc double[DataContainerPolesSize(Dimension)];
        Span<double> dc_knots = stackalloc double[DataContainerKnotsSize];
        PrepareEval_T<Point, Vector, Traits>(ref u,
                                             ref index,
                                             out dim,
                                             out rational,
                                             Degree,
                                             Periodic,
                                             Poles,
                                             Weights,
                                             Knots,
                                             Mults,
                                             dc_poles,
                                             dc_knots);
        BSplCLib.Eval(u, Degree, dc_knots, dim, dc_poles);

        if (rational)
        {
            double w = dc_poles[Dimension];
            Traits.CoordsToPointScaled(out P, dc_poles, w);
        }
        else
        {
            Traits.CoordsToPointDirect(out P, dc_poles);
        }
    }

    // Evaluates the point and first derivative on the B-spline curve at parameter U
    internal static void BSplCLib_D1<Point, Vector, Traits>(double U,
                                                           int Index,
                                                           int Degree,
                                                           bool Periodic,
                                                           NCollection_Array1<Point> Poles,
                                                           NCollection_Array1<double>? Weights,
                                                           NCollection_Array1<double> Knots,
                                                           NCollection_Array1<int>? Mults,
                                                           out Point P,
                                                           out Vector V)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int Dimension = Traits.Dimension;
        int dim, index = Index;
        double u = U;
        bool rational;
        validateBSplineDegree(Degree);
        Span<double> dc_poles = stackalloc double[DataContainerPolesSize(Dimension)];
        Span<double> dc_knots = stackalloc double[DataContainerKnotsSize];
        Span<double> dc_ders = stackalloc double[DataContainerDersSize(Dimension)];
        PrepareEval_T<Point, Vector, Traits>(ref u,
                                             ref index,
                                             out dim,
                                             out rational,
                                             Degree,
                                             Periodic,
                                             Poles,
                                             Weights,
                                             Knots,
                                             Mults,
                                             dc_poles,
                                             dc_knots);
        BSplCLib.Bohm(u, Degree, 1, dc_knots, dim, dc_poles);
        ReadOnlySpan<double> result = dc_poles;
        if (rational)
        {
            PLib.RationalDerivative(Degree, 1, Dimension, dc_poles, dc_ders);
            result = dc_ders;
        }

        Traits.CoordsToPointDirect(out P, result);
        Traits.CoordsToVectorDirect(out V, result.Slice(Dimension));
    }

    // Evaluates the point and first two derivatives on the B-spline curve at parameter U
    internal static void BSplCLib_D2<Point, Vector, Traits>(double U,
                                                           int Index,
                                                           int Degree,
                                                           bool Periodic,
                                                           NCollection_Array1<Point> Poles,
                                                           NCollection_Array1<double>? Weights,
                                                           NCollection_Array1<double> Knots,
                                                           NCollection_Array1<int>? Mults,
                                                           out Point P,
                                                           out Vector V1,
                                                           out Vector V2)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int Dimension = Traits.Dimension;
        int dim, index = Index;
        double u = U;
        bool rational;
        validateBSplineDegree(Degree);
        Span<double> dc_poles = stackalloc double[DataContainerPolesSize(Dimension)];
        Span<double> dc_knots = stackalloc double[DataContainerKnotsSize];
        Span<double> dc_ders = stackalloc double[DataContainerDersSize(Dimension)];
        PrepareEval_T<Point, Vector, Traits>(ref u,
                                             ref index,
                                             out dim,
                                             out rational,
                                             Degree,
                                             Periodic,
                                             Poles,
                                             Weights,
                                             Knots,
                                             Mults,
                                             dc_poles,
                                             dc_knots);
        BSplCLib.Bohm(u, Degree, 2, dc_knots, dim, dc_poles);
        ReadOnlySpan<double> result = dc_poles;
        if (rational)
        {
            PLib.RationalDerivative(Degree, 2, Dimension, dc_poles, dc_ders);
            result = dc_ders;
        }

        Traits.CoordsToPointDirect(out P, result);
        Traits.CoordsToVectorDirect(out V1, result.Slice(Dimension));
        if (!rational && (Degree < 2))
        {
            Traits.NullifyVector(out V2);
        }
        else
        {
            Traits.CoordsToVectorDirect(out V2, result.Slice(2 * Dimension));
        }
    }

    // Evaluates the point and first three derivatives on the B-spline curve at parameter U
    internal static void BSplCLib_D3<Point, Vector, Traits>(double U,
                                                           int Index,
                                                           int Degree,
                                                           bool Periodic,
                                                           NCollection_Array1<Point> Poles,
                                                           NCollection_Array1<double>? Weights,
                                                           NCollection_Array1<double> Knots,
                                                           NCollection_Array1<int>? Mults,
                                                           out Point P,
                                                           out Vector V1,
                                                           out Vector V2,
                                                           out Vector V3)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int Dimension = Traits.Dimension;
        int dim, index = Index;
        double u = U;
        bool rational;
        validateBSplineDegree(Degree);
        Span<double> dc_poles = stackalloc double[DataContainerPolesSize(Dimension)];
        Span<double> dc_knots = stackalloc double[DataContainerKnotsSize];
        Span<double> dc_ders = stackalloc double[DataContainerDersSize(Dimension)];
        PrepareEval_T<Point, Vector, Traits>(ref u,
                                             ref index,
                                             out dim,
                                             out rational,
                                             Degree,
                                             Periodic,
                                             Poles,
                                             Weights,
                                             Knots,
                                             Mults,
                                             dc_poles,
                                             dc_knots);
        BSplCLib.Bohm(u, Degree, 3, dc_knots, dim, dc_poles);
        ReadOnlySpan<double> result = dc_poles;
        if (rational)
        {
            PLib.RationalDerivative(Degree, 3, Dimension, dc_poles, dc_ders);
            result = dc_ders;
        }

        Traits.CoordsToPointDirect(out P, result);
        Traits.CoordsToVectorDirect(out V1, result.Slice(Dimension));
        if (!rational && (Degree < 2))
        {
            Traits.NullifyVector(out V2);
        }
        else
        {
            Traits.CoordsToVectorDirect(out V2, result.Slice(2 * Dimension));
        }
        if (!rational && (Degree < 3))
        {
            Traits.NullifyVector(out V3);
        }
        else
        {
            Traits.CoordsToVectorDirect(out V3, result.Slice(3 * Dimension));
        }
    }

    // Evaluates the Nth derivative on the B-spline curve at parameter U
    internal static void BSplCLib_DN<Point, Vector, Traits>(double U,
                                                           int N,
                                                           int Index,
                                                           int Degree,
                                                           bool Periodic,
                                                           NCollection_Array1<Point> Poles,
                                                           NCollection_Array1<double>? Weights,
                                                           NCollection_Array1<double> Knots,
                                                           NCollection_Array1<int>? Mults,
                                                           out Vector VN)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int Dimension = Traits.Dimension;
        int dim, index = Index;
        double u = U;
        bool rational;
        validateBSplineDegree(Degree);
        Span<double> dc_poles = stackalloc double[DataContainerPolesSize(Dimension)];
        Span<double> dc_knots = stackalloc double[DataContainerKnotsSize];
        PrepareEval_T<Point, Vector, Traits>(ref u,
                                             ref index,
                                             out dim,
                                             out rational,
                                             Degree,
                                             Periodic,
                                             Poles,
                                             Weights,
                                             Knots,
                                             Mults,
                                             dc_poles,
                                             dc_knots);
        BSplCLib.Bohm(u, Degree, N, dc_knots, dim, dc_poles);

        if (rational)
        {
            Span<double> v = stackalloc double[Dimension];
            PLib.RationalDerivative(Degree, N, Dimension, dc_poles, v, false);
            Traits.CoordsToVectorDirect(out VN, v);
        }
        else
        {
            if (N > Degree)
            {
                Traits.NullifyVector(out VN);
            }
            else
            {
                ReadOnlySpan<double> DN = dc_poles.Slice(N * Dimension);
                Traits.CoordsToVectorDirect(out VN, DN);
            }
        }
    }

    // Solves a banded system of equations for B-spline poles
    // @param[in,out] PolesArray - poles array containing right-hand side and receiving solution
    // @returns error status
    internal static int BSplCLib_SolveBandedSystem<Point, Vector, Traits>(math_Matrix Matrix,
                                                                         int UpperBandWidth,
                                                                         int LowerBandWidth,
                                                                         NCollection_Array1<Point> PolesArray)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        Span<double> PArray;
        PArray = MemoryMarshal.Cast<Point, double>(PolesArray.AsSpan());

        return BSplCLib.SolveBandedSystem(Matrix, UpperBandWidth, LowerBandWidth, Traits.Dimension, PArray);
    }

    // Solves a banded system of equations for rational B-spline poles with weights
    // @returns error status
    internal static int BSplCLib_SolveBandedSystem<Point, Vector, Traits>(math_Matrix Matrix,
                                                                         int UpperBandWidth,
                                                                         int LowerBandWidth,
                                                                         bool HomogeneousFlag,
                                                                         NCollection_Array1<Point> PolesArray,
                                                                         NCollection_Array1<double> WeightsArray)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        Span<double> PArray, WArray;
        PArray = MemoryMarshal.Cast<Point, double>(PolesArray.AsSpan());
        WArray = WeightsArray.AsSpan();
        return BSplCLib.SolveBandedSystem(Matrix,
                                          UpperBandWidth,
                                          LowerBandWidth,
                                          HomogeneousFlag,
                                          Traits.Dimension,
                                          PArray,
                                          WArray);
    }

    // Evaluates the B-spline curve at a given parameter
    // @param[in,out] ExtrapMode - extrapolation mode
    // @param[out] aPoint - resulting point
    // @param[out] aWeight - resulting weight
    internal static void BSplCLib_Eval<Point, Vector, Traits>(double Parameter,
                                                             bool PeriodicFlag,
                                                             bool HomogeneousFlag,
                                                             ReadOnlySpan<int> ExtrapMode,
                                                             int Degree,
                                                             NCollection_Array1<double> FlatKnots,
                                                             NCollection_Array1<Point> PolesArray,
                                                             NCollection_Array1<double> WeightsArray,
                                                             out Point aPoint,
                                                             out double aWeight)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int Dimension = Traits.Dimension;
        double Inverse;
        Span<double> P = stackalloc double[Dimension];
        Span<double> aWeightBuf = stackalloc double[1]; // OCCT passes &aWeight as a one-value result array
        ReadOnlySpan<double> PArray, WArray;
        int kk;
        PArray = MemoryMarshal.Cast<Point, double>(PolesArray.AsSpan());
        WArray = WeightsArray.AsSpan();
        if (HomogeneousFlag)
        {
            BSplCLib.Eval(Parameter,
                          PeriodicFlag,
                          0,
                          ExtrapMode,
                          Degree,
                          FlatKnots,
                          Dimension,
                          PArray,
                          P);
            BSplCLib.Eval(Parameter,
                          PeriodicFlag,
                          0,
                          ExtrapMode,
                          Degree,
                          FlatKnots,
                          1,
                          WArray,
                          aWeightBuf);
            aWeight = aWeightBuf[0];
        }
        else
        {
            BSplCLib.Eval(Parameter,
                          PeriodicFlag,
                          0,
                          ExtrapMode,
                          Degree,
                          FlatKnots,
                          Dimension,
                          PArray,
                          WArray,
                          P,
                          aWeightBuf);
            aWeight = aWeightBuf[0];
            Inverse = 1.0e0 / aWeight;

            for (kk = 0; kk < Dimension; kk++)
            {
                P[kk] *= Inverse;
            }
        }

        // C++ sets each coordinate of the caller's point; every coordinate is written
        Traits.NullifyPoint(out aPoint);
        for (kk = 0; kk < Dimension; kk++)
        {
            Traits.SetCoord(ref aPoint, kk + 1, P[kk]);
        }
    }

    // Evaluates the point on cached B-spline curve at parameter
    internal static void BSplCLib_CacheD0<Point, Vector, Traits>(double Parameter,
                                                                int Degree,
                                                                double CacheParameter,
                                                                double SpanLenght,
                                                                NCollection_Array1<Point> PolesArray,
                                                                NCollection_Array1<double>? WeightsArray,
                                                                out Point aPoint)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int Dimension = Traits.Dimension;
        double NewParameter, Inverse;
        ReadOnlySpan<double> PArray = MemoryMarshal.Cast<Point, double>(PolesArray.AsSpan());
        Span<double> myPoint = stackalloc double[Dimension]; // OCCT writes through (double*)&aPoint
        NewParameter = (Parameter - CacheParameter) / SpanLenght;
        PLib.NoDerivativeEvalPolynomial(NewParameter,
                                        Degree,
                                        Dimension,
                                        Degree * Dimension,
                                        PArray,
                                        myPoint);
        if (WeightsArray != null)
        {
            NCollection_Array1<double> refWeights = WeightsArray;
            ReadOnlySpan<double> WArray = refWeights.AsSpan();
            Span<double> anInverse = stackalloc double[1];
            PLib.NoDerivativeEvalPolynomial(NewParameter, Degree, 1, Degree, WArray, anInverse);
            Inverse = anInverse[0];

            Inverse = 1.0e0 / Inverse;
            Traits.ModifyCoordsScale(myPoint, Inverse);
        }
        Traits.CoordsToPointDirect(out aPoint, myPoint);
    }

    // Evaluates the point and first derivative on cached B-spline curve at parameter
    internal static void BSplCLib_CacheD1<Point, Vector, Traits>(double Parameter,
                                                                int Degree,
                                                                double CacheParameter,
                                                                double SpanLenght,
                                                                NCollection_Array1<Point> PolesArray,
                                                                NCollection_Array1<double>? WeightsArray,
                                                                out Point aPoint,
                                                                out Vector aVector)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int Dimension = Traits.Dimension;
        Span<double> LocalPDerivatives = stackalloc double[Dimension << 1];
        Span<double> LocalWDerivatives = stackalloc double[2];
        double NewParameter;

        ReadOnlySpan<double> PArray = MemoryMarshal.Cast<Point, double>(PolesArray.AsSpan());
        NewParameter = (Parameter - CacheParameter) / SpanLenght;
        PLib.EvalPolynomial(NewParameter, 1, Degree, Dimension, PArray, LocalPDerivatives);

        Traits.ModifyCoordsDivide(LocalPDerivatives.Slice(Dimension), SpanLenght);

        if (WeightsArray != null)
        {
            NCollection_Array1<double> refWeights = WeightsArray;
            ReadOnlySpan<double> WArray = refWeights.AsSpan();
            PLib.EvalPolynomial(NewParameter, 1, Degree, 1, WArray, LocalWDerivatives);

            LocalWDerivatives[1] /= SpanLenght;

            PLib.RationalDerivatives(1,
                                     Dimension,
                                     LocalPDerivatives,
                                     LocalWDerivatives,
                                     LocalPDerivatives);
        }

        Traits.CoordsToPointDirect(out aPoint, LocalPDerivatives);
        Traits.CoordsToVectorDirect(out aVector, LocalPDerivatives.Slice(Dimension));
    }

    // Evaluates the point and first two derivatives on cached B-spline curve at parameter
    internal static void BSplCLib_CacheD2<Point, Vector, Traits>(double Parameter,
                                                                int Degree,
                                                                double CacheParameter,
                                                                double SpanLenght,
                                                                NCollection_Array1<Point> PolesArray,
                                                                NCollection_Array1<double>? WeightsArray,
                                                                out Point aPoint,
                                                                out Vector aVector1,
                                                                out Vector aVector2)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int Dimension = Traits.Dimension;
        int ii, Index, EndIndex;
        Span<double> LocalPDerivatives = stackalloc double[(Dimension << 1) + Dimension];
        Span<double> LocalWDerivatives = stackalloc double[3];
        double NewParameter, Factor;
        ReadOnlySpan<double> PArray = MemoryMarshal.Cast<Point, double>(PolesArray.AsSpan());
        NewParameter = (Parameter - CacheParameter) / SpanLenght;
        PLib.EvalPolynomial(NewParameter, 2, Degree, Dimension, PArray, LocalPDerivatives);

        Factor = 1.0e0 / SpanLenght;
        Index = Dimension;
        EndIndex = Min(2, Degree);

        for (ii = 1; ii <= EndIndex; ii++)
        {
            Traits.ModifyCoordsScale(LocalPDerivatives.Slice(Index), Factor);
            Factor /= SpanLenght;
            Index += Dimension;
        }

        Index = (Degree + 1) * Dimension;
        for (ii = Degree; ii < 2; ii++)
        {
            Traits.NullifyCoords(LocalPDerivatives.Slice(Index));
            Index += Dimension;
        }

        if (WeightsArray != null)
        {
            NCollection_Array1<double> refWeights = WeightsArray;
            ReadOnlySpan<double> WArray = refWeights.AsSpan();

            PLib.EvalPolynomial(NewParameter, 2, Degree, 1, WArray, LocalWDerivatives);

            for (ii = Degree + 1; ii <= 2; ii++)
            {
                LocalWDerivatives[ii] = 0.0e0;
            }

            Factor = 1.0e0 / SpanLenght;

            for (ii = 1; ii <= EndIndex; ii++)
            {
                LocalWDerivatives[ii] *= Factor;
                Factor /= SpanLenght;
            }
            PLib.RationalDerivatives(2,
                                     Dimension,
                                     LocalPDerivatives,
                                     LocalWDerivatives,
                                     LocalPDerivatives);
        }

        Traits.CoordsToPointDirect(out aPoint, LocalPDerivatives);
        Traits.CoordsToVectorDirect(out aVector1, LocalPDerivatives.Slice(Dimension));
        Traits.CoordsToVectorDirect(out aVector2, LocalPDerivatives.Slice(Dimension * 2));
    }

    // Evaluates the point and first three derivatives on cached B-spline curve at parameter
    internal static void BSplCLib_CacheD3<Point, Vector, Traits>(double Parameter,
                                                                int Degree,
                                                                double CacheParameter,
                                                                double SpanLenght,
                                                                NCollection_Array1<Point> PolesArray,
                                                                NCollection_Array1<double>? WeightsArray,
                                                                out Point aPoint,
                                                                out Vector aVector1,
                                                                out Vector aVector2,
                                                                out Vector aVector3)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int Dimension = Traits.Dimension;
        int ii, Index, EndIndex;
        Span<double> LocalPDerivatives = stackalloc double[Dimension << 2];
        Span<double> LocalWDerivatives = stackalloc double[4];
        double Factor, NewParameter;
        ReadOnlySpan<double> PArray = MemoryMarshal.Cast<Point, double>(PolesArray.AsSpan());
        NewParameter = (Parameter - CacheParameter) / SpanLenght;
        PLib.EvalPolynomial(NewParameter, 3, Degree, Dimension, PArray, LocalPDerivatives);

        Index = (Degree + 1) * Dimension;
        for (ii = Degree; ii < 3; ii++)
        {
            Traits.NullifyCoords(LocalPDerivatives.Slice(Index));
            Index += Dimension;
        }

        Factor = 1.0e0 / SpanLenght;
        Index = Dimension;
        EndIndex = Min(3, Degree);

        for (ii = 1; ii <= EndIndex; ii++)
        {
            Traits.ModifyCoordsScale(LocalPDerivatives.Slice(Index), Factor);
            Factor /= SpanLenght;
            Index += Dimension;
        }

        if (WeightsArray != null)
        {
            NCollection_Array1<double> refWeights = WeightsArray;
            ReadOnlySpan<double> WArray = refWeights.AsSpan();

            PLib.EvalPolynomial(NewParameter, 3, Degree, 1, WArray, LocalWDerivatives);

            Factor = 1.0e0 / SpanLenght;

            for (ii = 1; ii <= EndIndex; ii++)
            {
                LocalWDerivatives[ii] *= Factor;
                Factor /= SpanLenght;
            }

            for (ii = (Degree + 1); ii <= 3; ii++)
            {
                LocalWDerivatives[ii] = 0.0e0;
            }
            PLib.RationalDerivatives(3,
                                     Dimension,
                                     LocalPDerivatives,
                                     LocalWDerivatives,
                                     LocalPDerivatives);
        }

        Traits.CoordsToPointDirect(out aPoint, LocalPDerivatives);
        Traits.CoordsToVectorDirect(out aVector1, LocalPDerivatives.Slice(Dimension));
        Traits.CoordsToVectorDirect(out aVector2, LocalPDerivatives.Slice(Dimension * 2));
        Traits.CoordsToVectorDirect(out aVector3, LocalPDerivatives.Slice(Dimension * 3));
    }

    // Builds cache for efficient B-spline evaluation (Point version)
    internal static void BSplCLib_BuildCache<Point, Vector, Traits>(double U,
                                                                   double SpanDomain,
                                                                   bool Periodic,
                                                                   int Degree,
                                                                   NCollection_Array1<double> FlatKnots,
                                                                   NCollection_Array1<Point> Poles,
                                                                   NCollection_Array1<double>? Weights,
                                                                   NCollection_Array1<Point> CachePoles,
                                                                   NCollection_Array1<double>? CacheWeights)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int Dimension = Traits.Dimension;
        int ii, LocalDimension, LocalIndex, index = 0;
        double u = U, LocalValue;
        bool rational;

        validateBSplineDegree(Degree);
        Span<double> dc_poles = stackalloc double[DataContainerPolesSize(Dimension)];
        Span<double> dc_knots = stackalloc double[DataContainerKnotsSize];
        PrepareEval_T<Point, Vector, Traits>(ref u,
                                             ref index,
                                             out LocalDimension,
                                             out rational,
                                             Degree,
                                             Periodic,
                                             Poles,
                                             Weights,
                                             FlatKnots,
                                             (BSplCLib.NoMults()),
                                             dc_poles,
                                             dc_knots);

        BSplCLib.Bohm(u, Degree, Degree, dc_knots, LocalDimension, dc_poles);

        LocalValue = 1.0e0;
        LocalIndex = 0;

        if (rational)
        {

            for (ii = 1; ii <= Degree + 1; ii++)
            {
                Traits.CoordsToPointMultiplied(out CachePoles[ii], dc_poles.Slice(LocalIndex), LocalValue);
                LocalIndex += Dimension + 1;
                LocalValue *= SpanDomain / (double)ii;
            }

            LocalIndex = Dimension;
            LocalValue = 1.0e0;
            for (ii = 1; ii <= Degree + 1; ii++)
            {
                CacheWeights![ii] = dc_poles[LocalIndex] * LocalValue;
                LocalIndex += Dimension + 1;
                LocalValue *= SpanDomain / (double)ii;
            }
        }
        else
        {

            for (ii = 1; ii <= Degree + 1; ii++)
            {
                Traits.CoordsToPointMultiplied(out CachePoles[ii], dc_poles.Slice(LocalIndex), LocalValue);
                LocalIndex += Dimension;
                LocalValue *= SpanDomain / (double)ii;
            }

            if (Weights != null)
            {
                for (ii = 1; ii <= Degree + 1; ii++)
                {
                    CacheWeights![ii] = 0.0e0;
                }
                CacheWeights![1] = 1.0e0;
            }
        }
    }

    // Builds cache for efficient B-spline evaluation (Array2 version)
    // @param[out] theCacheArray - 2D cache array
    internal static void BSplCLib_BuildCache<Point, Vector, Traits>(double theParameter,
                                                                   double theSpanDomain,
                                                                   bool thePeriodicFlag,
                                                                   int theDegree,
                                                                   int theSpanIndex,
                                                                   NCollection_Array1<double> theFlatKnots,
                                                                   NCollection_Array1<Point> thePoles,
                                                                   NCollection_Array1<double>? theWeights,
                                                                   NCollection_Array2<double> theCacheArray)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        double aParam = theParameter;
        int anIndex = theSpanIndex;
        int aDimension;
        bool isRational;

        validateBSplineDegree(theDegree);
        Span<double> dc_poles = stackalloc double[DataContainerPolesSize(Traits.Dimension)];
        Span<double> dc_knots = stackalloc double[DataContainerKnotsSize];
        PrepareEval_T<Point, Vector, Traits>(ref aParam,
                                             ref anIndex,
                                             out aDimension,
                                             out isRational,
                                             theDegree,
                                             thePeriodicFlag,
                                             thePoles,
                                             theWeights,
                                             theFlatKnots,
                                             (BSplCLib.NoMults()),
                                             dc_poles,
                                             dc_knots);

        int aCacheShift = (theWeights != null && !isRational) ? aDimension + 1 : aDimension;

        BSplCLib.Bohm(aParam, theDegree, theDegree, dc_knots, aDimension, dc_poles);

        double aCoeff = 1.0;
        // the cache array's storage from (LowerRow, LowerCol), row after row
        Span<double> aCacheArray = theCacheArray.AsSpan();
        int aCache = 0;
        int aPolyCoeffs = 0;

        for (int i = 0; i <= theDegree; i++)
        {
            for (int j = 0; j < aDimension; j++)
            {
                aCacheArray[aCache + j] = dc_poles[aPolyCoeffs + j] * aCoeff;
            }
            aCoeff *= theSpanDomain / (i + 1);
            aPolyCoeffs += aDimension;
            aCache += aDimension;
            if (aCacheShift > aDimension)
            {
                aCacheArray[aCache] = 0.0;
                aCache++;
            }
        }

        if (aCacheShift > aDimension)
        {
            theCacheArray.SetValue(theCacheArray.LowerRow(),
                                   theCacheArray.LowerCol() + aCacheShift - 1,
                                   1.0);
        }
    }

    // Interpolates points to create a B-spline curve
    // @param[in,out] Poles - poles array to be computed
    // @param[out] InversionProblem - status of matrix inversion
    internal static void BSplCLib_Interpolate<Point, Vector, Traits>(int Degree,
                                                                    NCollection_Array1<double> FlatKnots,
                                                                    NCollection_Array1<double> Parameters,
                                                                    NCollection_Array1<int> ContactOrderArray,
                                                                    NCollection_Array1<Point> Poles,
                                                                    out int InversionProblem)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        Span<double> PArray;

        PArray = MemoryMarshal.Cast<Point, double>(Poles.AsSpan());

        BSplCLib.Interpolate(Degree,
                             FlatKnots,
                             Parameters,
                             ContactOrderArray,
                             Traits.Dimension,
                             PArray,
                             out InversionProblem);
    }

    // Interpolates weighted points to create a rational B-spline curve
    internal static void BSplCLib_Interpolate<Point, Vector, Traits>(int Degree,
                                                                    NCollection_Array1<double> FlatKnots,
                                                                    NCollection_Array1<double> Parameters,
                                                                    NCollection_Array1<int> ContactOrderArray,
                                                                    NCollection_Array1<Point> Poles,
                                                                    NCollection_Array1<double> Weights,
                                                                    out int InversionProblem)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        Span<double> PArray, WArray;
        PArray = MemoryMarshal.Cast<Point, double>(Poles.AsSpan());
        WArray = Weights.AsSpan();
        BSplCLib.Interpolate(Degree,
                             FlatKnots,
                             Parameters,
                             ContactOrderArray,
                             Traits.Dimension,
                             PArray,
                             WArray,
                             out InversionProblem);
    }

    // Moves a point on the B-spline curve by modifying poles
    // @param[out] FirstIndex - first modified pole index
    // @param[out] LastIndex - last modified pole index
    internal static void BSplCLib_MovePoint<Point, Vector, Traits>(double U,
                                                                  in Vector Displ,
                                                                  int Index1,
                                                                  int Index2,
                                                                  int Degree,
                                                                  NCollection_Array1<Point> Poles,
                                                                  NCollection_Array1<double>? Weights,
                                                                  NCollection_Array1<double> FlatKnots,
                                                                  out int FirstIndex,
                                                                  out int LastIndex,
                                                                  NCollection_Array1<Point> NewPoles)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        // calculate the BSplineBasis in the parameter U
        int FirstNonZeroBsplineIndex;
        math_Matrix BSplineBasis = new(1, 1, 1, Degree + 1);
        int ErrorCode =
          BSplCLib.EvalBsplineBasis(0, Degree + 1, FlatKnots, U, out FirstNonZeroBsplineIndex, BSplineBasis);
        if (ErrorCode != 0)
        {
            FirstIndex = 0;
            LastIndex = 0;

            for (int k = Poles.Lower(); k <= Poles.Upper(); k++) // i in OCCT
            {
                NewPoles[k] = Poles[k];
            }
            return;
        }

        // find the span which is predominant for parameter U
        FirstIndex = FirstNonZeroBsplineIndex;
        LastIndex = FirstNonZeroBsplineIndex + Degree;
        if (FirstIndex < Index1)
        {
            FirstIndex = Index1;
        }
        if (LastIndex > Index2)
        {
            LastIndex = Index2;
        }

        double maxValue = 0.0;
        int i, kk1 = 0, kk2, ii;

        for (i = FirstIndex - FirstNonZeroBsplineIndex + 1; i <= LastIndex - FirstNonZeroBsplineIndex + 1;
             i++)
        {
            if (BSplineBasis[1, i] > maxValue)
            {
                kk1 = i + FirstNonZeroBsplineIndex - 1;
                maxValue = BSplineBasis[1, i];
            }
        }

        // find a kk2 if symmetry
        kk2 = kk1;
        i = kk1 - FirstNonZeroBsplineIndex + 2;
        if ((kk1 + 1) <= LastIndex)
        {
            if (Math.Abs(BSplineBasis[1, kk1 - FirstNonZeroBsplineIndex + 2] - maxValue) < 1.0e-10)
            {
                kk2 = kk1 + 1;
            }
        }

        // compute the vector of displacement
        double D1 = 0.0;
        double D2 = 0.0;
        double hN, Coef, Dval;

        for (i = 1; i <= Degree + 1; i++)
        {
            ii = i + FirstNonZeroBsplineIndex - 1;
            if (Weights != null)
            {
                hN = Weights.Value(ii) * BSplineBasis[1, i];
                D2 += hN;
            }
            else
            {
                hN = BSplineBasis[1, i];
            }
            if (ii >= FirstIndex && ii <= LastIndex)
            {
                if (ii < kk1)
                {
                    Dval = kk1 - ii;
                }
                else if (ii > kk2)
                {
                    Dval = ii - kk2;
                }
                else
                {
                    Dval = 0.0;
                }
                D1 += 1.0 / (Dval + 1.0) * hN;
            }
        }

        if (Weights != null)
        {
            Coef = D2 / D1;
        }
        else
        {
            Coef = 1.0 / D1;
        }

        // compute the new poles

        for (i = Poles.Lower(); i <= Poles.Upper(); i++)
        {
            if (i >= FirstIndex && i <= LastIndex)
            {
                if (i < kk1)
                {
                    Dval = kk1 - i;
                }
                else if (i > kk2)
                {
                    Dval = i - kk2;
                }
                else
                {
                    Dval = 0.0;
                }
                NewPoles[i] = Traits.Translated(Poles[i], (Coef / (Dval + 1.0)), Displ);
            }
            else
            {
                NewPoles[i] = Poles[i];
            }
        }
    }

    // Moves a point and tangent on the B-spline curve by modifying poles
    // @param[out] ErrorStatus - error status of the operation
    internal static void BSplCLib_MovePointAndTangent<Point, Vector, Traits>(double U,
                                                                            in Vector Delta,
                                                                            in Vector DeltaDerivatives,
                                                                            double Tolerance,
                                                                            int Degree,
                                                                            int StartingCondition,
                                                                            int EndingCondition,
                                                                            NCollection_Array1<Point> Poles,
                                                                            NCollection_Array1<double>? Weights,
                                                                            NCollection_Array1<double> FlatKnots,
                                                                            NCollection_Array1<Point> NewPoles,
                                                                            out int ErrorStatus)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int Dimension = Traits.Dimension;
        Span<double> delta_array = stackalloc double[Dimension];
        Span<double> delta_derivative_array = stackalloc double[Dimension];
        ReadOnlySpan<double> poles_array;
        Span<double> new_poles_array;

        int num_poles;
        num_poles = Poles.Length();

        if (NewPoles.Length() != num_poles)
        {
            throw new Standard_ConstructionError();
        }
        Traits.VectorToCoords(delta_array, Delta);
        Traits.VectorToCoords(delta_derivative_array, DeltaDerivatives);
        poles_array = MemoryMarshal.Cast<Point, double>(Poles.AsSpan());

        new_poles_array = MemoryMarshal.Cast<Point, double>(NewPoles.AsSpan());
        BSplCLib.MovePointAndTangent(U,
                                     Dimension,
                                     delta_array,
                                     delta_derivative_array,
                                     Tolerance,
                                     Degree,
                                     StartingCondition,
                                     EndingCondition,
                                     poles_array,
                                     Weights,
                                     FlatKnots,
                                     new_poles_array,
                                     out ErrorStatus);
    }

    // Computes the parametric tolerance corresponding to a 3D tolerance
    // @param[out] UTolerance - computed parametric tolerance
    internal static void BSplCLib_Resolution<Point, Vector, Traits>(NCollection_Array1<Point> Poles,
                                                                   NCollection_Array1<double>? Weights,
                                                                   int NumPoles,
                                                                   NCollection_Array1<double> FlatKnots,
                                                                   int Degree,
                                                                   double Tolerance3D,
                                                                   out double UTolerance)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        ReadOnlySpan<double> PolesArray;
        PolesArray = MemoryMarshal.Cast<Point, double>(Poles.AsSpan());
        BSplCLib.Resolution(PolesArray,
                            Traits.Dimension,
                            NumPoles,
                            Weights,
                            FlatKnots,
                            Degree,
                            Tolerance3D,
                            out UTolerance);
    }

    // Multiplies the B-spline curve by a function
    // @param[out] theStatus - status of the operation
    internal static void BSplCLib_FunctionMultiply<Point, Vector, Traits>(BSplCLib_EvaluatorFunction FunctionPtr,
                                                                         int BSplineDegree,
                                                                         NCollection_Array1<double> BSplineFlatKnots,
                                                                         NCollection_Array1<Point> Poles,
                                                                         NCollection_Array1<double> FlatKnots,
                                                                         int NewDegree,
                                                                         NCollection_Array1<Point> NewPoles,
                                                                         out int theStatus)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int num_bspline_poles = BSplineFlatKnots.Length() - BSplineDegree - 1;
        int num_new_poles = FlatKnots.Length() - NewDegree - 1;

        if (Poles.Length() != num_bspline_poles || NewPoles.Length() != num_new_poles)
        {
            throw new Standard_ConstructionError();
        }
        ReadOnlySpan<double> array_of_poles = MemoryMarshal.Cast<Point, double>(Poles.AsSpan());
        Span<double> array_of_new_poles = MemoryMarshal.Cast<Point, double>(NewPoles.AsSpan());
        BSplCLib.FunctionMultiply(FunctionPtr,
                                  BSplineDegree,
                                  BSplineFlatKnots,
                                  Traits.Dimension,
                                  array_of_poles,
                                  FlatKnots,
                                  NewDegree,
                                  array_of_new_poles,
                                  out theStatus);
    }

    // Reparameterizes the B-spline curve using a function
    // @param[out] theStatus - status of the operation
    internal static void BSplCLib_FunctionReparameterise<Point, Vector, Traits>(BSplCLib_EvaluatorFunction FunctionPtr,
                                                                               int BSplineDegree,
                                                                               NCollection_Array1<double> BSplineFlatKnots,
                                                                               NCollection_Array1<Point> Poles,
                                                                               NCollection_Array1<double> FlatKnots,
                                                                               int NewDegree,
                                                                               NCollection_Array1<Point> NewPoles,
                                                                               out int theStatus)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int num_bspline_poles = BSplineFlatKnots.Length() - BSplineDegree - 1;
        int num_new_poles = FlatKnots.Length() - NewDegree - 1;

        if (Poles.Length() != num_bspline_poles || NewPoles.Length() != num_new_poles)
        {
            throw new Standard_ConstructionError();
        }
        ReadOnlySpan<double> array_of_poles = MemoryMarshal.Cast<Point, double>(Poles.AsSpan());
        Span<double> array_of_new_poles = MemoryMarshal.Cast<Point, double>(NewPoles.AsSpan());
        BSplCLib.FunctionReparameterise(FunctionPtr,
                                        BSplineDegree,
                                        BSplineFlatKnots,
                                        Traits.Dimension,
                                        array_of_poles,
                                        FlatKnots,
                                        NewDegree,
                                        array_of_new_poles,
                                        out theStatus);
    }

    // Computes coefficients of a Bezier curve from poles (Bezier syntax wrapper)
    internal static void BSplCLib_PolesCoefficients_Bezier<Point, Vector, Traits>(NCollection_Array1<Point> Poles,
                                                                                 NCollection_Array1<double>? Weights,
                                                                                 NCollection_Array1<Point> CachePoles,
                                                                                 NCollection_Array1<double>? CacheWeights)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int aDegree = Poles.Length() - 1;
        // OCCT wraps the static table without copying; the port copies the 2 * (aDegree + 1) knots
        NCollection_Array1<double> aBidFlatKnots =
          NCollection_Array1<double>.From(BSplCLib.FlatBezierKnots(aDegree).Slice(0, 2 * (aDegree + 1)), 1);
        BSplCLib_BuildCache<Point, Vector, Traits>(0.0,
                                                   1.0,
                                                   false,
                                                   aDegree,
                                                   aBidFlatKnots,
                                                   Poles,
                                                   Weights,
                                                   CachePoles,
                                                   CacheWeights);
    }

    // Increases the degree of a Bezier curve (Bezier syntax wrapper)
    internal static void BSplCLib_IncreaseDegree_Bezier<Point, Vector, Traits>(int NewDegree,
                                                                              NCollection_Array1<Point> Poles,
                                                                              NCollection_Array1<double>? Weights,
                                                                              NCollection_Array1<Point> NewPoles,
                                                                              NCollection_Array1<double>? NewWeights)
        where Point : struct
        where Vector : struct
        where Traits : struct, IBSplCLib_CurveTraits<Point, Vector>
    {
        int aDegree = Poles.Length() - 1;
        // fresh arrays: they are both the input and the output knots and multiplicities, as in OCCT
        KnotArrays(aDegree, out NCollection_Array1<double> aBezierKnots_Knot, out NCollection_Array1<int> aBezierKnots_Mult);
        BSplCLib_IncreaseDegree<Point, Vector, Traits>(aDegree,
                                                       NewDegree,
                                                       false,
                                                       Poles,
                                                       Weights,
                                                       aBezierKnots_Knot,
                                                       aBezierKnots_Mult,
                                                       NewPoles,
                                                       NewWeights,
                                                       aBezierKnots_Knot,
                                                       aBezierKnots_Mult);
    }
}
