// Port of: src/ModelingAlgorithms/TKShHealing/ShapeConstruct/ShapeConstruct_Curve.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeConstruct/ShapeConstruct_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Healing.Construct;

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
/// on a frozen curve (one taken from a shape) they throw <see cref="ImmutableObjectException"/>, so callers pass a
/// <c>Copy()</c>, or use the functional twins <see cref="CurveAdjusted"/>, <see cref="CurveSegmentAdjusted"/> and
/// <see cref="Curve2dAdjusted"/>, which adjust a copy (decision 506's pattern). <c>ConvertToBSpline</c> returns its
/// result frozen; ported code calls <c>InternalConvertToBSpline</c>, whose result it may go on editing. When the
/// curve is already a B-spline needing no segment, both return the curve itself.
/// </para>
/// </remarks>
internal sealed class ConstructCurve
{
    /// <summary>
    /// Moves the ends of the curve to <paramref name="p1"/> (if <paramref name="take1"/>) and <paramref name="p2"/>
    /// (if <paramref name="take2"/>): the end poles of a B-spline, the whole line of a line (both points). Returns
    /// false for other curves. In place: throws on a frozen curve. Doesn't check whether the curve should be reversed.
    /// </summary>
    public bool AdjustCurve(Curve3D c3D, in Point3D p1, in Point3D p2, bool take1 = true, bool take2 = true)
    {
        if (!take1 && !take2)
        {
            return true;
        }

        if (c3D is BSplineCurve3D bspl)
        {
            if (take1)
            {
                bspl.SetPole(1, p1);
            }
            if (take2)
            {
                bspl.SetPole(bspl.PoleCount, p2);
            }
            return true;
        }

        if (c3D is LineCurve3D l3D)
        {
            //   ATTENTION, P1 et P2 sont supposes tous deux pertinents ...
            Vector3D avec = new(p1, p2);
            Direction3D adir = new(avec);
            Line3D alin = new(p1, adir);
            double theParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(alin, l3D.Line.Location);
            alin = alin.WithLocation(global::Kerf.Engine.Geometry.ElCLib.Value(theParam, alin));
            l3D.SetLin(alin);
            return true;
        }

        return false;
    }

    /// <summary>
    /// As <see cref="AdjustCurve"/>, a B-spline being first segmented to [<paramref name="u1"/>, <paramref name="u2"/>]
    /// (within its range). In place: throws on a frozen curve.
    /// </summary>
    public bool AdjustCurveSegment(Curve3D c3D, in Point3D p1, in Point3D p2, double u1, double u2)
    {
        if (c3D is BSplineCurve3D bspl)
        {
            //    Forcer l extremite c est bien
            //    Propager sur le reste, c est pas mal non plus
            if (u1 >= u2)
            {
                return false;
            }
            double uu1 = Max(u1, bspl.FirstParameter);
            double uu2 = Min(u2, bspl.LastParameter);
            bspl.Segment(uu1, uu2);
            bspl.SetPole(1, p1);
            bspl.SetPole(bspl.PoleCount, p2);
            return true;
        }

        if (c3D is LineCurve3D l3D)
        {
            //   ATTENTION, P1 et P2 sont supposes tous deux pertinents ...
            //   NB : on ne s aide pas de U1 et U2
            Vector3D avec = new(p1, p2);
            Direction3D adir = new(avec);
            Line3D alin = new(p1, adir);
            double theParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(alin, l3D.Line.Location);
            alin = alin.WithLocation(global::Kerf.Engine.Geometry.ElCLib.Value(theParam, alin));
            l3D.SetLin(alin);
            return true;
        }

        return false;
    }

    /// <summary>The 2D form of <see cref="AdjustCurve"/>. In place: throws on a frozen curve.</summary>
    public bool AdjustCurve2d(Curve2D c2D, in Point2D p1, in Point2D p2, bool take1 = true, bool take2 = true)
    {
        if (!take1 && !take2)
        {
            return true;
        }

        if (c2D is BSplineCurve2D bspl)
        {
            if (take1)
            {
                bspl.SetPole(1, p1);
            }
            if (take2)
            {
                bspl.SetPole(bspl.PoleCount, p2);
            }
            return true;
        }

        if (c2D is LineCurve2D l2D)
        {
            //   ATTENTION, P1 et P2 sont supposes tous deux pertinents ...
            Vector2D avec = new(p1, p2);
            Direction2D adir = new(avec);
            Line2D alin = new(p1, adir);
            double theParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(alin, l2D.Line.Location);
            alin = alin.WithLocation(global::Kerf.Engine.Geometry.ElCLib.Value(theParam, alin));
            l2D.SetLin2d(alin);
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
    public BSplineCurve3D? ConvertToBSpline(Curve3D c, double first, double last, double prec)
    {
        BSplineCurve3D? result = InternalConvertToBSpline(c, first, last, prec);
        result?.Freeze();
        return result;
    }

    /// <summary>
    /// <see cref="ConvertToBSpline(Curve3D, double, double, double)"/> without freezing (ported code): a new curve
    /// is unfrozen; the given B-spline itself is returned as it is when no segment is needed.
    /// </summary>
    internal BSplineCurve3D? InternalConvertToBSpline(Curve3D c, double first, double last, double prec)
    {
        BSplineCurve3D? bspl = null;

        if (c is BSplineCurve3D bSpline)
        {
            bspl = bSpline;
        }
        else if (c is BezierCurve3D || c is LineCurve3D)
        {
            Curve3D tc = new TrimmedCurve3D(c, first, last);
            try
            {
                bspl = global::Kerf.Engine.Geometry.Conversion.GeomConvert.CurveToBSplineCurve(tc);
            }
            catch (Exception caught) when (Refusal.IsFailure(caught))
            {
            }
        }

        if (bspl is not null)
        {
            // take segment if trim and range differ
            double fbsp = bspl.FirstParameter, lbsp = bspl.LastParameter;
            bool segment = false;
            if (first > fbsp + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                fbsp = first;
                segment = true;
            }
            if (last < lbsp - global::Kerf.Engine.Foundation.Precision.PConfusion())
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
                bspl = (BSplineCurve3D)bspl.Copy();
                bspl.Segment(fbsp, lbsp);
                return bspl;
            }
            catch (Exception caught) when (Refusal.IsFailure(caught))
            {
            }
        }

        // Approx is used for conics and when ordinary methods fail
        Curve3D newc = c;
        if (bspl is not null)
        {
            newc = bspl;
            bspl = null;
        }
        try
        {
            Curve3d conv = new(new CurveAdaptor(newc, first, last), prec, C1, 9, 1000);
            if (conv.IsDone || conv.HasResult)
            {
                bspl = conv.InternalCurve();
            }
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
        }
        return bspl;
    }

    /// <summary>
    /// The 2D form of <see cref="ConvertToBSpline(Curve3D, double, double, double)"/> (Approx_Curve2d for other
    /// curves). Frozen; null on failure.
    /// </summary>
    public BSplineCurve2D? ConvertToBSpline(Curve2D c, double first, double last, double prec)
    {
        BSplineCurve2D? result = InternalConvertToBSpline(c, first, last, prec);
        result?.Freeze();
        return result;
    }

    /// <summary>The 2D <c>ConvertToBSpline</c> without freezing (ported code).</summary>
    internal BSplineCurve2D? InternalConvertToBSpline(Curve2D c, double first, double last, double prec)
    {
        BSplineCurve2D? bspl = null;

        if (c is BSplineCurve2D bSpline)
        {
            bspl = bSpline;
        }
        else if (c is BezierCurve2D || c is LineCurve2D)
        {
            Curve2D tc = new TrimmedCurve2D(c, first, last);
            try
            {
                bspl = Geom2dConvert2D.CurveToBSplineCurve(tc);
            }
            catch (Exception caught) when (Refusal.IsFailure(caught))
            {
            }
        }

        if (bspl is not null)
        {
            // take segment if trim and range differ
            double fbsp = bspl.FirstParameter, lbsp = bspl.LastParameter;
            bool segment = false;
            if (first > fbsp + global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                fbsp = first;
                segment = true;
            }
            if (last < lbsp - global::Kerf.Engine.Foundation.Precision.PConfusion())
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
                bspl = (BSplineCurve2D)bspl.Copy();
                bspl.Segment(fbsp, lbsp);
                return bspl;
            }
            catch (Exception caught) when (Refusal.IsFailure(caught))
            {
            }
        }

        // Approx is used for conics and when ordinary methods fail
        Curve2D newc = c;
        if (bspl is not null)
        {
            newc = bspl;
            bspl = null;
        }
        try
        {
            Curve2d conv = new(new CurveAdaptor2D(newc, first, last), first, last, prec, prec, C1, 9,
                                      1000);
            if (conv.IsDone || conv.HasResult)
            {
                bspl = conv.InternalCurve();
            }
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
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
    // The functional twins of the in-place adjustments (Kerf)
    //=================================================================================================

    /// <summary>
    /// The functional twin of <see cref="AdjustCurve"/>: an adjusted copy of the curve (frozen), or the curve itself
    /// when <paramref name="isDone"/> is false or nothing is taken.
    /// </summary>
    public Curve3D CurveAdjusted(Curve3D c3D, in Point3D p1, in Point3D p2, out bool isDone, bool take1 = true,
                                    bool take2 = true)
    {
        if (!take1 && !take2)
        {
            isDone = true;
            return c3D;
        }
        Curve3D copy = (Curve3D)c3D.Copy();
        isDone = AdjustCurve(copy, p1, p2, take1, take2);
        if (!isDone)
        {
            return c3D;
        }
        copy.Freeze();
        return copy;
    }

    /// <summary>
    /// The functional twin of <see cref="AdjustCurveSegment"/>: an adjusted copy (frozen), or the curve itself when
    /// <paramref name="isDone"/> is false.
    /// </summary>
    public Curve3D CurveSegmentAdjusted(Curve3D c3D, in Point3D p1, in Point3D p2, double u1, double u2, out bool isDone)
    {
        Curve3D copy = (Curve3D)c3D.Copy();
        isDone = AdjustCurveSegment(copy, p1, p2, u1, u2);
        if (!isDone)
        {
            return c3D;
        }
        copy.Freeze();
        return copy;
    }

    /// <summary>
    /// The functional twin of <see cref="AdjustCurve2d"/>: an adjusted copy (frozen), or the curve itself when
    /// <paramref name="isDone"/> is false or nothing is taken.
    /// </summary>
    public Curve2D Curve2dAdjusted(Curve2D c2D, in Point2D p1, in Point2D p2, out bool isDone, bool take1 = true,
                                        bool take2 = true)
    {
        if (!take1 && !take2)
        {
            isDone = true;
            return c2D;
        }
        Curve2D copy = (Curve2D)c2D.Copy();
        isDone = AdjustCurve2d(copy, p1, p2, take1, take2);
        if (!isDone)
        {
            return c2D;
        }
        copy.Freeze();
        return copy;
    }
}
