// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_Bisec.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_Bisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Offsets.Bisectors;

/// <summary>
/// The bisecting line of two elements (curves or points), trimmed by a point P and contained in the sector defined
/// by two vectors V1, V2 and a sense: if Sense is positive the bisector is in the sector of -V1 and -V2 in the
/// indirect sense, else in the direct sense. Lines and circles give an analytic bisector (Bisector_BisecAna), other
/// curves Bisector_BisecPC or Bisector_BisecCC; Value() is the bisector trimmed at its start and end.
/// </summary>
/// <remarks>
/// Tolerance defines degenerate bisectors: a hyperbola with a radius, a parabola with a focal length or an ellipse
/// with a minor radius smaller than Tolerance is replaced by a line or half-line on one of its axes.
/// A working object of the medial axis (MAT2d_Tool2d keeps and re-trims its curve in place, as OCCT does): Value()
/// and ChangeValue() return the trimmed curve itself, not frozen; its mutators are internal.
/// </remarks>
internal sealed class Bisec
{
    private TrimmedCurve2D _thebisector = null!;

    /// <summary>An empty bisector.</summary>
    public Bisec()
    {
    }

    //===========================================================================
    //    calculate the bissectrice between two curves coming from a point.
    //
    //   afirstcurve   : \ curves between which the
    //   asecondcurve  : / bissectrice is calculated.
    //   apoint        :   point through which the bissectrice should pass.
    //   afirstvector  : \ vectors to determine the sector where
    //   asecondvector : / the bissectrice should be located.
    //   adirection    :   shows the side of the bissectrice to be preserved.
    //   tolerance     :   threshold starting from which the bisectrices are degenerated
    //===========================================================================
    /// <summary>
    /// Computes the bisecting line of the curves afirstcurve and asecondcurve from apoint. oncurve is true if apoint
    /// is common to the two curves.
    /// </summary>
    public void Perform(Curve2D afirstcurve, Curve2D asecondcurve, in Point2D apoint,
                        in Vector2D afirstvector, in Vector2D asecondvector, double adirection,
                        JoinKind ajointype, double tolerance, bool oncurve = true)
    {
        Type type1 = afirstcurve.GetType();
        Type type2 = asecondcurve.GetType();
        BisectorCurve bis;
        double uFirst, uLast;

        if (type1 == typeof(TrimmedCurve2D))
        {
            type1 = ((TrimmedCurve2D)afirstcurve).ChangeBasisCurve().GetType();
        }
        if (type2 == typeof(TrimmedCurve2D))
        {
            type2 = ((TrimmedCurve2D)asecondcurve).ChangeBasisCurve().GetType();
        }

        Curve2D afirstcurve1 = afirstcurve;
        Curve2D asecondcurve1 = asecondcurve;

        if (type1 == typeof(BSplineCurve2D))
        {
            BSplineCurve2D bs;
            if (afirstcurve.GetType() == typeof(TrimmedCurve2D))
            {
                bs = (BSplineCurve2D)((TrimmedCurve2D)afirstcurve).ChangeBasisCurve();
            }
            else
            {
                bs = (BSplineCurve2D)afirstcurve;
            }
            if (bs.Degree == 1 && bs.PoleCount == 2)
            {
                if (bs.Pole(1).DistanceTo(bs.Pole(2)) < 1.0e-4)
                {
                    afirstcurve1 = new MakeSegment2d(bs.Pole(1), bs.Pole(2)).Value();
                    type1 = typeof(LineCurve2D);
                }
            }
        }

        if (type2 == typeof(BSplineCurve2D))
        {
            BSplineCurve2D bs;
            if (asecondcurve.GetType() == typeof(TrimmedCurve2D))
            {
                bs = (BSplineCurve2D)((TrimmedCurve2D)asecondcurve).ChangeBasisCurve();
            }
            else
            {
                bs = (BSplineCurve2D)asecondcurve;
            }
            if (bs.Degree == 1 && bs.PoleCount == 2)
            {
                if (bs.Pole(1).DistanceTo(bs.Pole(2)) < 1.0e-4)
                {
                    asecondcurve1 = new MakeSegment2d(bs.Pole(1), bs.Pole(2)).Value();
                    type2 = typeof(LineCurve2D);
                }
            }
        }

        if ((type1 == typeof(CircleCurve2D) || type1 == typeof(LineCurve2D))
            && (type2 == typeof(CircleCurve2D) || type2 == typeof(LineCurve2D)))
        {
            //------------------------------------------------------------------
            // Analytic Bissectrice.
            //------------------------------------------------------------------
            BisecAna bisAna = new();
            bisAna.Perform(afirstcurve1, asecondcurve1, apoint, afirstvector, asecondvector, adirection, ajointype,
                           tolerance, oncurve);
            uFirst = bisAna.ParameterOfStartPoint();
            uLast = bisAna.ParameterOfEndPoint();
            bis = bisAna;
        }
        else
        {
            bool isLine = false;

            if (oncurve)
            {
                Direction2D fd = new(afirstvector);
                Direction2D sd = new(asecondvector);
                // if (Fd.Dot(Sd) < Precision::Angular() - 1.) {
                // if (Fd.Dot(Sd) < 10*Precision::Angular() - 1.) //patch
                if (fd.Dot(sd) < Math.Sqrt(2.0 * global::Kerf.Engine.Foundation.Precision.Angular()) - 1.0)
                {
                    isLine = true;
                }
            }
            if (isLine)
            {
                //------------------------------------------------------------------
                // Half-Staight.
                //------------------------------------------------------------------
                Direction2D n = new(-adirection * afirstvector.Y, adirection * afirstvector.X);
                LineCurve2D l = new(apoint, n);
                TrimmedCurve2D bisL = new(l, 0, global::Kerf.Engine.Foundation.Precision.Infinite());
                BisecAna bisAna = new();
                bisAna.Init(bisL);
                uFirst = bisAna.ParameterOfStartPoint();
                uLast = bisAna.ParameterOfEndPoint();
                bis = bisAna;
            }
            else
            {
                //-------------------------------------------------------------------
                // Bissectrice algo
                //-------------------------------------------------------------------
                BisecCC bisCC = new();
                bisCC.Perform(asecondcurve1, afirstcurve1, adirection, adirection, apoint);

                if (bisCC.IsEmpty())
                {
                    // bissectrice is empty. a point is projected at the end of the guide curve.
                    // Construction of a false bissectrice.
                    //  modified by NIZHNY-EAP Mon Feb 21 12:00:13 2000 ___BEGIN___
                    Point2D p1 = afirstcurve1.Value(afirstcurve1.LastParameter);
                    Point2D p2 = asecondcurve1.Value(asecondcurve1.FirstParameter);
                    Point2D pm = new(.5 * (p1.XY() + p2.XY()));
                    double nx, ny;
                    if (pm.DistanceTo(apoint) > 10.0 * global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        nx = apoint.X - pm.X;
                        ny = apoint.Y - pm.Y;
                        if (adirection < 0)
                        {
                            nx = -nx;
                            ny = -ny;
                        }
                    }
                    else
                    {
                        Direction2D dir1 = new(afirstvector), dir2 = new(asecondvector);
                        nx = -dir1.X - dir2.X;
                        ny = -dir1.Y - dir2.Y;
                        if (Math.Abs(nx) <= Constants.Resolution() && Math.Abs(ny) <= Constants.Resolution())
                        {
                            nx = -afirstvector.Y;
                            ny = afirstvector.X;
                        }
                    }
                    Direction2D n = new(adirection * nx, adirection * ny);
                    //  modified by NIZHNY-EAP Mon Feb 21 12:00:19 2000 ___END___

                    LineCurve2D l = new(apoint, n);
                    TrimmedCurve2D bisL = new(l, 0, global::Kerf.Engine.Foundation.Precision.Infinite());
                    BisecAna bisAna = new();
                    bisAna.Init(bisL);
                    uFirst = bisAna.ParameterOfStartPoint();
                    uLast = bisAna.ParameterOfEndPoint();
                    bis = bisAna;
                }
                else
                {
                    uFirst = bisCC.FirstParameter;
                    uLast = bisCC.LastParameter;
                    bis = bisCC;
                    ReplaceByLineIfIsToSmall(ref bis, ref uFirst, ref uLast);
                }
            }
        }
        uFirst = Max(uFirst, bis.FirstParameter);
        uLast = Min(uLast, bis.LastParameter);
        _thebisector = new TrimmedCurve2D(bis, uFirst, uLast);
    }

    //===========================================================================
    //  calculate the bissectrice between a curve and a point starting in a point.
    //
    //   afirstcurve   : \ curve and point the bissectrice between which is calculated.
    //   asecondpoint  : /
    //   apoint        :   point through which the bissectrice should pass.
    //   afirstvector  : \ vectors to find the sector where
    //   asecondvector : / the bissectrice should be located.
    //   adirection    :   shows the side of the bissectrice to be preserved.
    //   tolerance     :   threshold starting from which the bisectrices are degenerated
    //===========================================================================
    /// <summary>
    /// Computes the bisecting line of the curve afirstcurve and the point asecondpoint from apoint. oncurve is true if
    /// apoint is the point asecondpoint.
    /// </summary>
    public void Perform(Curve2D afirstcurve, Point2 asecondpoint, in Point2D apoint,
                        in Vector2D afirstvector, in Vector2D asecondvector, double adirection, double tolerance,
                        bool oncurve = true)
    {
        BisectorCurve bis;
        Type type1 = afirstcurve.GetType();
        double uFirst, uLast;

        if (type1 == typeof(TrimmedCurve2D))
        {
            type1 = ((TrimmedCurve2D)afirstcurve).ChangeBasisCurve().GetType();
        }

        if (type1 == typeof(CircleCurve2D) || type1 == typeof(LineCurve2D))
        {
            //------------------------------------------------------------------
            // Analytic Bissectrice.
            //------------------------------------------------------------------
            BisecAna bisAna = new();
            bisAna.Perform(afirstcurve, asecondpoint, apoint, afirstvector, asecondvector, adirection, tolerance,
                           oncurve);
            uFirst = bisAna.ParameterOfStartPoint();
            uLast = bisAna.ParameterOfEndPoint();
            bis = bisAna;
        }
        else
        {
            bool isLine = false;
            double rc = global::Kerf.Engine.Foundation.Precision.Infinite();

            if (oncurve)
            {
                if (global::Kerf.Engine.Offsets.Bisectors.Bisector.IsConvex(afirstcurve, adirection) || IsMaxRC(afirstcurve, afirstcurve.LastParameter, ref rc))
                {
                    isLine = true;
                }
            }
            if (isLine)
            {
                //------------------------------------------------------------------
                // Half-Right.
                //------------------------------------------------------------------
                Direction2D n = new(-adirection * afirstvector.Y, adirection * afirstvector.X);
                LineCurve2D l = new(apoint, n);
                TrimmedCurve2D bisL = new(l, 0, rc);
                BisecAna bisAna = new();
                bisAna.Init(bisL);
                uFirst = bisAna.ParameterOfStartPoint();
                uLast = bisAna.ParameterOfEndPoint();
                bis = bisAna;
            }
            else
            {
                //-------------------------------------------------------------------
                // Bissectrice algo
                //-------------------------------------------------------------------
                BisecPC bisPC = new();
                Curve2D afirstcurvereverse = afirstcurve.Reversed();

                bisPC.Perform(afirstcurvereverse, asecondpoint.Pnt2d(), -adirection);
                //  Modified by Sergey KHROMOV - Thu Feb 21 16:49:54 2002 Begin
                if (bisPC.IsEmpty)
                {
                    Direction2D dir1 = new(afirstvector), dir2 = new(asecondvector);
                    double nx = -dir1.X - dir2.X, ny = -dir1.Y - dir2.Y;
                    if (Math.Abs(nx) <= Constants.Resolution() && Math.Abs(ny) <= Constants.Resolution())
                    {
                        nx = -afirstvector.Y;
                        ny = afirstvector.X;
                    }
                    Direction2D n = new(adirection * nx, adirection * ny);
                    LineCurve2D l = new(apoint, n);
                    TrimmedCurve2D bisL = new(l, 0, rc);
                    BisecAna bisAna = new();
                    bisAna.Init(bisL);
                    uFirst = bisAna.ParameterOfStartPoint();
                    uLast = bisAna.ParameterOfEndPoint();
                    bis = bisAna;
                }
                else
                {
                    //  Modified by Sergey KHROMOV - Wed Mar  6 17:01:08 2002 End
                    uFirst = bisPC.Parameter(apoint);
                    uLast = bisPC.LastParameter;
                    if (uFirst >= uLast)
                    {
                        // Extrapolate by line
                        Vector2D v = new(bisPC.Value(bisPC.FirstParameter), bisPC.Value(uLast));
                        Direction2D n = new(v);
                        LineCurve2D l = new(apoint, n);
                        TrimmedCurve2D bisL = new(l, 0, rc);
                        BisecAna bisAna = new();
                        bisAna.Init(bisL);
                        uFirst = bisAna.ParameterOfStartPoint();
                        uLast = bisAna.ParameterOfEndPoint();
                        bis = bisAna;
                    }
                    else
                    {
                        bis = bisPC;
                    }
                }
            }
        }
        if (uFirst < bis.FirstParameter)
        {
            uFirst = bis.FirstParameter;
        }
        if (uLast > bis.LastParameter)
        {
            uLast = bis.LastParameter;
        }
        _thebisector = new TrimmedCurve2D(bis, uFirst, uLast);
    }

    //===========================================================================
    //   calculate the bissectrice between a curve and a point starting in a point.
    //
    //   afirstpoint   : \ curve and point the bissectrice between which is calculated.
    //   asecondcurve  : /
    //   apoint        :   point through which the bissectrice should pass.
    //   afirstvector  : \ vectors to find the sector where
    //   asecondvector : / the bissectrice should be located.
    //   adirection    :   shows the side of the bissectrice to be preserved.
    //   tolerance     :   threshold starting from which the bisectrices are degenerated
    //===========================================================================
    /// <summary>
    /// Computes the bisecting line of the point afirstpoint and the curve asecondcurve from apoint. oncurve is true
    /// if apoint is the point afirstpoint.
    /// </summary>
    public void Perform(Point2 afirstpoint, Curve2D asecondcurve, in Point2D apoint,
                        in Vector2D afirstvector, in Vector2D asecondvector, double adirection, double tolerance,
                        bool oncurve = true)
    {
        BisectorCurve bis;
        Type type1 = asecondcurve.GetType();
        double uFirst, uLast;

        if (type1 == typeof(TrimmedCurve2D))
        {
            type1 = ((TrimmedCurve2D)asecondcurve).ChangeBasisCurve().GetType();
        }

        if (type1 == typeof(CircleCurve2D) || type1 == typeof(LineCurve2D))
        {
            //------------------------------------------------------------------
            // Analytic Bissectrice.
            //------------------------------------------------------------------
            BisecAna bisAna = new();
            bisAna.Perform(afirstpoint, asecondcurve, apoint, afirstvector, asecondvector, adirection, tolerance,
                           oncurve);
            uFirst = bisAna.ParameterOfStartPoint();
            uLast = bisAna.ParameterOfEndPoint();
            bis = bisAna;
        }
        else
        {
            bool isLine = false;
            double rc = global::Kerf.Engine.Foundation.Precision.Infinite();

            if (oncurve)
            {
                if (global::Kerf.Engine.Offsets.Bisectors.Bisector.IsConvex(asecondcurve, adirection)
                    || IsMaxRC(asecondcurve, asecondcurve.FirstParameter, ref rc))
                {
                    isLine = true;
                }
            }
            if (isLine)
            {
                //------------------------------------------------------------------
                // Half-Staight.
                //------------------------------------------------------------------
                Direction2D n = new(-adirection * afirstvector.Y, adirection * afirstvector.X);
                LineCurve2D l = new(apoint, n);
                TrimmedCurve2D bisL = new(l, 0, rc);
                BisecAna bisAna = new();
                bisAna.Init(bisL);
                uFirst = bisAna.ParameterOfStartPoint();
                uLast = bisAna.ParameterOfEndPoint();
                bis = bisAna;
            }
            else
            {
                //-------------------------------------------------------------------
                // Bissectrice algo
                //-------------------------------------------------------------------
                BisecPC bisPC = new();
                bisPC.Perform(asecondcurve, afirstpoint.Pnt2d(), adirection);
                //  Modified by Sergey KHROMOV - Thu Feb 21 16:49:54 2002 Begin
                if (bisPC.IsEmpty)
                {
                    Direction2D dir1 = new(afirstvector), dir2 = new(asecondvector);
                    double nx = -dir1.X - dir2.X, ny = -dir1.Y - dir2.Y;
                    if (Math.Abs(nx) <= Constants.Resolution() && Math.Abs(ny) <= Constants.Resolution())
                    {
                        nx = -afirstvector.Y;
                        ny = afirstvector.X;
                    }
                    Direction2D n = new(adirection * nx, adirection * ny);
                    LineCurve2D l = new(apoint, n);
                    TrimmedCurve2D bisL = new(l, 0, rc);
                    BisecAna bisAna = new();
                    bisAna.Init(bisL);
                    uFirst = bisAna.ParameterOfStartPoint();
                    uLast = bisAna.ParameterOfEndPoint();
                    bis = bisAna;
                }
                else
                {
                    //  Modified by Sergey KHROMOV - Thu Feb 21 16:49:58 2002 End
                    uFirst = bisPC.Parameter(apoint);
                    uLast = bisPC.LastParameter;
                    if (uFirst >= uLast)
                    {
                        // Extrapolate by line
                        Vector2D v = new(bisPC.Value(bisPC.FirstParameter), bisPC.Value(uLast));
                        Direction2D n = new(v);
                        LineCurve2D l = new(apoint, n);
                        TrimmedCurve2D bisL = new(l, 0, rc);
                        BisecAna bisAna = new();
                        bisAna.Init(bisL);
                        uFirst = bisAna.ParameterOfStartPoint();
                        uLast = bisAna.ParameterOfEndPoint();
                        bis = bisAna;
                    }
                    else
                    {
                        bis = bisPC;
                    }
                }
            }
        }

        uFirst = Max(uFirst, bis.FirstParameter);
        uLast = Min(uLast, bis.LastParameter);
        _thebisector = new TrimmedCurve2D(bis, uFirst, uLast);
    }

    //===========================================================================
    //        calculate the bissectrice between two points starting in a point.
    //
    //   afirstpoint   : \ curves the bissectrice between which should be
    //   asecondpoint  : / calculated.
    //   apoint        :   point through which the bissectrice should pass.
    //   afirstvector  : \ vectors to find the sector where
    //   asecondvector : / the bissectrice should be located.
    //   adirection    :   shows the side of the bissectrice to be preserved.
    //===========================================================================
    /// <summary>Computes the bisecting line of the points afirstpoint and asecondpoint from apoint.</summary>
    /// <exception cref="NoSolutionException">The points coincide.</exception>
    public void Perform(Point2 afirstpoint, Point2 asecondpoint, in Point2D apoint,
                        in Vector2D afirstvector, in Vector2D asecondvector, double adirection, double tolerance = 0.0,
                        bool oncurve = true)
    {
        BisecAna bis = new();
        bis.Perform(afirstpoint, asecondpoint, apoint, afirstvector, asecondvector, adirection, tolerance, oncurve);
        _thebisector = new TrimmedCurve2D(bis, bis.ParameterOfStartPoint(), bis.ParameterOfEndPoint());
    }

    /// <summary>The bisector: a Geom2d_TrimmedCurve whose basis is a Bisector_Curve.</summary>
    public TrimmedCurve2D Value() => _thebisector;

    /// <summary>The bisector, for ported code that re-trims it in place (MAT2d_Tool2d).</summary>
    public TrimmedCurve2D ChangeValue() => _thebisector;

    //=============================================================================
    //           replaced by a half-straight.
    //=============================================================================
    private static void ReplaceByLineIfIsToSmall(ref BisectorCurve bis, ref double uFirst, ref double uLast)
    {
        if (Math.Abs(uLast - uFirst) > 2.0 * global::Kerf.Engine.Foundation.Precision.PConfusion() * 10.0)
        {
            return; // patch
        }

        Point2D pf = bis.Value(uFirst);
        Point2D pl = bis.Value(uLast);

        if (pf.DistanceTo(pl) > global::Kerf.Engine.Foundation.Precision.Confusion() * 10.0)
        {
            return;
        }

        Vector2D t1 = bis.DN(uFirst, 1);

        LineCurve2D l = new(pf, new Direction2D(t1));
        TrimmedCurve2D bisL = new(l, 0, global::Kerf.Engine.Foundation.Precision.Infinite());
        BisecAna bisAna = new();
        bisAna.Init(bisL);
        uFirst = bisAna.ParameterOfStartPoint();
        uLast = bisAna.ParameterOfEndPoint();
        bis = bisAna;
    }

    private static bool IsMaxRC(Curve2D c, double u, ref double r)
    {
        double kf, kl;
        double us = c.FirstParameter;
        double ul = c.LastParameter;
        double norm2;

        c.D2(us, out Point2D _, out Vector2D d1, out Vector2D d2);
        norm2 = d1.SquareMagnitude;
        if (norm2 < Constants.Resolution())
        {
            kf = 0.0;
        }
        else
        {
            kf = Math.Abs(d1 ^ d2) / (norm2 * Math.Sqrt(norm2));
        }

        c.D2(ul, out Point2D _, out d1, out d2);
        norm2 = d1.SquareMagnitude;
        if (norm2 < Constants.Resolution())
        {
            kl = 0.0;
        }
        else
        {
            kl = Math.Abs(d1 ^ d2) / (norm2 * Math.Sqrt(norm2));
        }

        bool isMax = false;
        if (u == ul)
        {
            if (kl < kf)
            {
                if (kl == 0.0)
                {
                    r = global::Kerf.Engine.Foundation.Precision.Infinite();
                }
                else
                {
                    r = 1 / kl;
                }
                isMax = true;
            }
        }
        else
        {
            if (kf < kl)
            {
                if (kf == 0.0)
                {
                    r = global::Kerf.Engine.Foundation.Precision.Infinite();
                }
                else
                {
                    r = 1 / kf;
                }
                isMax = true;
            }
        }
        return isMax;
    }
}
