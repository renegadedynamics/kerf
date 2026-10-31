// Port of: src/ModelingAlgorithms/TKShHealing/ShapeConstruct/ShapeConstruct_Curve.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeConstruct/ShapeConstruct_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// Adjusts a curve to have its start and end points at given points (lines and B-splines only), converts curves to
/// B-splines and fixes B-spline knots.
/// </summary>
/// <remarks>
/// <para>
/// A working object without state (decision 21), as OCCT uses it by value.
/// </para>
/// <para>
/// <c>AdjustCurve</c>, <c>AdjustCurveSegment</c> and <c>AdjustCurve2d</c> change the given curve in place, as OCCT:
/// on a frozen curve (one taken from a shape) they throw <see cref="Standard_ImmutableObject"/>, so callers pass a
/// <c>Copy()</c>, or use the functional twins <see cref="CurveAdjusted"/>, <see cref="CurveSegmentAdjusted"/> and
/// <see cref="Curve2dAdjusted"/>, which adjust a copy (decision 506's pattern). <c>ConvertToBSpline</c> returns its
/// result frozen; ported code calls <c>InternalConvertToBSpline</c>, whose result it may go on editing. When the
/// curve is already a B-spline needing no segment, both return the curve itself.
/// </para>
/// </remarks>
public sealed class ShapeConstruct_Curve
{
    /// <summary>
    /// Moves the ends of the curve to <paramref name="P1"/> (if <paramref name="take1"/>) and <paramref name="P2"/>
    /// (if <paramref name="take2"/>): the end poles of a B-spline, the whole line of a line (both points). Returns
    /// false for other curves. In place: throws on a frozen curve. Doesn't check whether the curve should be reversed.
    /// </summary>
    public bool AdjustCurve(Geom_Curve C3D, in gp_Pnt P1, in gp_Pnt P2, bool take1 = true, bool take2 = true)
    {
        if (!take1 && !take2)
        {
            return true;
        }

        if (C3D is Geom_BSplineCurve BSPL)
        {
            if (take1)
            {
                BSPL.SetPole(1, P1);
            }
            if (take2)
            {
                BSPL.SetPole(BSPL.NbPoles(), P2);
            }
            return true;
        }

        if (C3D is Geom_Line L3D)
        {
            //   ATTENTION, P1 et P2 sont supposes tous deux pertinents ...
            gp_Vec avec = new(P1, P2);
            gp_Dir adir = new(avec);
            gp_Lin alin = new(P1, adir);
            double theParam = ElCLib.Parameter(alin, L3D.Lin().Location());
            alin.SetLocation(ElCLib.Value(theParam, alin));
            L3D.SetLin(alin);
            return true;
        }

        return false;
    }

    /// <summary>
    /// As <see cref="AdjustCurve"/>, a B-spline being first segmented to [<paramref name="U1"/>, <paramref name="U2"/>]
    /// (within its range). In place: throws on a frozen curve.
    /// </summary>
    public bool AdjustCurveSegment(Geom_Curve C3D, in gp_Pnt P1, in gp_Pnt P2, double U1, double U2)
    {
        if (C3D is Geom_BSplineCurve BSPL)
        {
            //    Forcer l extremite c est bien
            //    Propager sur le reste, c est pas mal non plus
            if (U1 >= U2)
            {
                return false;
            }
            double UU1 = Max(U1, BSPL.FirstParameter());
            double UU2 = Min(U2, BSPL.LastParameter());
            BSPL.Segment(UU1, UU2);
            BSPL.SetPole(1, P1);
            BSPL.SetPole(BSPL.NbPoles(), P2);
            return true;
        }

        if (C3D is Geom_Line L3D)
        {
            //   ATTENTION, P1 et P2 sont supposes tous deux pertinents ...
            //   NB : on ne s aide pas de U1 et U2
            gp_Vec avec = new(P1, P2);
            gp_Dir adir = new(avec);
            gp_Lin alin = new(P1, adir);
            double theParam = ElCLib.Parameter(alin, L3D.Lin().Location());
            alin.SetLocation(ElCLib.Value(theParam, alin));
            L3D.SetLin(alin);
            return true;
        }

        return false;
    }

    /// <summary>The 2D form of <see cref="AdjustCurve"/>. In place: throws on a frozen curve.</summary>
    public bool AdjustCurve2d(Geom2d_Curve C2D, in gp_Pnt2d P1, in gp_Pnt2d P2, bool take1 = true, bool take2 = true)
    {
        if (!take1 && !take2)
        {
            return true;
        }

        if (C2D is Geom2d_BSplineCurve BSPL)
        {
            if (take1)
            {
                BSPL.SetPole(1, P1);
            }
            if (take2)
            {
                BSPL.SetPole(BSPL.NbPoles(), P2);
            }
            return true;
        }

        if (C2D is Geom2d_Line L2D)
        {
            //   ATTENTION, P1 et P2 sont supposes tous deux pertinents ...
            gp_Vec2d avec = new(P1, P2);
            gp_Dir2d adir = new(avec);
            gp_Lin2d alin = new(P1, adir);
            double theParam = ElCLib.Parameter(alin, L2D.Lin2d().Location());
            alin.SetLocation(ElCLib.Value(theParam, alin));
            L2D.SetLin2d(alin);
            return true;
        }

        return false;
    }

    /// <summary>
    /// The part [<paramref name="first"/>, <paramref name="last"/>] of the curve as a B-spline (frozen): a B-spline is
    /// segmented (the curve itself if no segment is needed), a Bezier curve or a line converted exactly, other
    /// curves approximated within <paramref name="prec"/> (Approx_Curve3d, C1, degree 9, 1000 segments). Null on
    /// failure.
    /// </summary>
    public Geom_BSplineCurve? ConvertToBSpline(Geom_Curve C, double first, double last, double prec)
    {
        Geom_BSplineCurve? aResult = InternalConvertToBSpline(C, first, last, prec);
        aResult?.Freeze();
        return aResult;
    }

    /// <summary>
    /// <see cref="ConvertToBSpline(Geom_Curve, double, double, double)"/> without freezing (ported code): a new curve
    /// is unfrozen; the given B-spline itself is returned as it is when no segment is needed.
    /// </summary>
    internal Geom_BSplineCurve? InternalConvertToBSpline(Geom_Curve C, double first, double last, double prec)
    {
        Geom_BSplineCurve? bspl = null;

        if (C is Geom_BSplineCurve aBSpline)
        {
            bspl = aBSpline;
        }
        else if (C is Geom_BezierCurve || C is Geom_Line)
        {
            Geom_Curve tc = new Geom_TrimmedCurve(C, first, last);
            try
            {
                bspl = GeomConvert.CurveToBSplineCurve(tc);
            }
            catch (Standard_Failure)
            {
            }
        }

        if (bspl is not null)
        {
            // take segment if trim and range differ
            double fbsp = bspl.FirstParameter(), lbsp = bspl.LastParameter();
            bool segment = false;
            if (first > fbsp + Precision.PConfusion())
            {
                fbsp = first;
                segment = true;
            }
            if (last < lbsp - Precision.PConfusion())
            {
                lbsp = last;
                segment = true;
            }
            if (!segment)
            {
                return bspl;
            }
            try
            {
                bspl = (Geom_BSplineCurve)bspl.Copy();
                bspl.Segment(fbsp, lbsp);
                return bspl;
            }
            catch (Standard_Failure)
            {
            }
        }

        // Approx is used for conics and when ordinary methods fail
        Geom_Curve newc = C;
        if (bspl is not null)
        {
            newc = bspl;
            bspl = null;
        }
        try
        {
            Approx_Curve3d Conv = new(new GeomAdaptor_Curve(newc, first, last), prec, GeomAbs_C1, 9, 1000);
            if (Conv.IsDone() || Conv.HasResult())
            {
                bspl = Conv.InternalCurve();
            }
        }
        catch (Standard_Failure)
        {
        }
        return bspl;
    }

    /// <summary>
    /// The 2D form of <see cref="ConvertToBSpline(Geom_Curve, double, double, double)"/> (Approx_Curve2d for other
    /// curves). Frozen; null on failure.
    /// </summary>
    public Geom2d_BSplineCurve? ConvertToBSpline(Geom2d_Curve C, double first, double last, double prec)
    {
        Geom2d_BSplineCurve? aResult = InternalConvertToBSpline(C, first, last, prec);
        aResult?.Freeze();
        return aResult;
    }

    /// <summary>The 2D <c>ConvertToBSpline</c> without freezing (ported code).</summary>
    internal Geom2d_BSplineCurve? InternalConvertToBSpline(Geom2d_Curve C, double first, double last, double prec)
    {
        Geom2d_BSplineCurve? bspl = null;

        if (C is Geom2d_BSplineCurve aBSpline)
        {
            bspl = aBSpline;
        }
        else if (C is Geom2d_BezierCurve || C is Geom2d_Line)
        {
            Geom2d_Curve tc = new Geom2d_TrimmedCurve(C, first, last);
            try
            {
                bspl = Geom2dConvert.CurveToBSplineCurve(tc);
            }
            catch (Standard_Failure)
            {
            }
        }

        if (bspl is not null)
        {
            // take segment if trim and range differ
            double fbsp = bspl.FirstParameter(), lbsp = bspl.LastParameter();
            bool segment = false;
            if (first > fbsp + Precision.PConfusion())
            {
                fbsp = first;
                segment = true;
            }
            if (last < lbsp - Precision.PConfusion())
            {
                lbsp = last;
                segment = true;
            }
            if (!segment)
            {
                return bspl;
            }
            try
            {
                bspl = (Geom2d_BSplineCurve)bspl.Copy();
                bspl.Segment(fbsp, lbsp);
                return bspl;
            }
            catch (Standard_Failure)
            {
            }
        }

        // Approx is used for conics and when ordinary methods fail
        Geom2d_Curve newc = C;
        if (bspl is not null)
        {
            newc = bspl;
            bspl = null;
        }
        try
        {
            Approx_Curve2d Conv = new(new Geom2dAdaptor_Curve(newc, first, last), first, last, prec, prec, GeomAbs_C1, 9,
                                      1000);
            if (Conv.IsDone() || Conv.HasResult())
            {
                bspl = Conv.InternalCurve();
            }
        }
        catch (Standard_Failure)
        {
        }
        return bspl;
    }

    /// <summary>
    /// Shifts B-spline knots closer than Epsilon to the previous one by 2 * Epsilon, in place. True if one moved.
    /// </summary>
    public static bool FixKnots(NCollection_HArray1<double> knots)
    {
        bool Fixed = false;
        int nbKnots = knots.Length();
        double knotVal = knots.Value(1);
        for (int i = 2; i <= nbKnots; i++)
        {
            double knotNext = knots.Value(i);
            if (knotNext - knotVal <= Epsilon(knotVal))
            {
                knotNext = knotVal + 2.0 * Epsilon(knotVal);
                knots.SetValue(i, knotNext);
                Fixed = true;
            }
            knotVal = knotNext;
        }
        return Fixed;
    }

    /// <summary>
    /// Shifts B-spline knots closer than Epsilon to the previous one by 2 * Epsilon, in place. True if one moved.
    /// </summary>
    public static bool FixKnots(NCollection_Array1<double> knots)
    {
        bool Fixed = false;
        int nbKnots = knots.Length();
        double knotVal = knots.Value(1);
        for (int i = 2; i <= nbKnots; i++)
        {
            double knotNext = knots.Value(i);
            if (knotNext - knotVal <= Epsilon(knotVal))
            {
                knotNext = knotVal + 2.0 * Epsilon(knotVal);
                knots.SetValue(i, knotNext);
                Fixed = true;
            }
            knotVal = knotNext;
        }
        return Fixed;
    }

    //=================================================================================================
    // The functional twins of the in-place adjustments (OCCT.NET)
    //=================================================================================================

    /// <summary>
    /// The functional twin of <see cref="AdjustCurve"/>: an adjusted copy of the curve (frozen), or the curve itself
    /// when <paramref name="isDone"/> is false or nothing is taken.
    /// </summary>
    public Geom_Curve CurveAdjusted(Geom_Curve C3D, in gp_Pnt P1, in gp_Pnt P2, out bool isDone, bool take1 = true,
                                    bool take2 = true)
    {
        if (!take1 && !take2)
        {
            isDone = true;
            return C3D;
        }
        Geom_Curve aCopy = (Geom_Curve)C3D.Copy();
        isDone = AdjustCurve(aCopy, P1, P2, take1, take2);
        if (!isDone)
        {
            return C3D;
        }
        aCopy.Freeze();
        return aCopy;
    }

    /// <summary>
    /// The functional twin of <see cref="AdjustCurveSegment"/>: an adjusted copy (frozen), or the curve itself when
    /// <paramref name="isDone"/> is false.
    /// </summary>
    public Geom_Curve CurveSegmentAdjusted(Geom_Curve C3D, in gp_Pnt P1, in gp_Pnt P2, double U1, double U2, out bool isDone)
    {
        Geom_Curve aCopy = (Geom_Curve)C3D.Copy();
        isDone = AdjustCurveSegment(aCopy, P1, P2, U1, U2);
        if (!isDone)
        {
            return C3D;
        }
        aCopy.Freeze();
        return aCopy;
    }

    /// <summary>
    /// The functional twin of <see cref="AdjustCurve2d"/>: an adjusted copy (frozen), or the curve itself when
    /// <paramref name="isDone"/> is false or nothing is taken.
    /// </summary>
    public Geom2d_Curve Curve2dAdjusted(Geom2d_Curve C2D, in gp_Pnt2d P1, in gp_Pnt2d P2, out bool isDone, bool take1 = true,
                                        bool take2 = true)
    {
        if (!take1 && !take2)
        {
            isDone = true;
            return C2D;
        }
        Geom2d_Curve aCopy = (Geom2d_Curve)C2D.Copy();
        isDone = AdjustCurve2d(aCopy, P1, P2, take1, take2);
        if (!isDone)
        {
            return C2D;
        }
        aCopy.Freeze();
        return aCopy;
    }
}
