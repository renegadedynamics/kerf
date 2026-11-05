// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_IntTraces to ChFi3d_BuildPCurve)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Fillets;

internal static partial class Builder_0
{
    //=================================================================================================

    /// <summary>
    /// The crossing of the traces on their common face of the side <paramref name="jf1"/> of <paramref name="fd1"/>
    /// and <paramref name="jf2"/> of <paramref name="fd2"/>: their parameters <paramref name="p1"/> and
    /// <paramref name="p2"/>, chosen among several by OCCT's criteria (closest to the references, to the left by the
    /// senses, or to <paramref name="refP2d"/>).
    /// </summary>
    internal static bool ChFi3d_IntTraces(SurfData fd1, double pref1, ref double p1, int jf1, int sens1,
                                          SurfData fd2, double pref2, ref double p2, int jf2, int sens2,
                                          in Point2D refP2d, bool check2dDistance = false, bool enlarge = false)
    {
        CurveAdaptor2D c1 = new();
        CurveAdaptor2D c2 = new();
        // pcurves are enlarged to be sure that there is intersection
        // additionally all periodic curves are taken and points on
        // them are filtered using a specific criterion.

        double first, last, delta = 0.0;
        first = fd1.Interference(jf1).FirstParameter();
        last = fd1.Interference(jf1).LastParameter();
        if ((last - first) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }
        if (enlarge)
        {
            delta = Math.Min(0.1, 0.05 * (last - first));
        }
        Curve2D? pcf1 = fd1.Interference(jf1).PCurveOnFace();
        if (pcf1 is null)
        {
            return false;
        }
        bool isper1 = pcf1.IsPeriodic;
        if (isper1)
        {
            if (pcf1 is TrimmedCurve2D tr1)
            {
                pcf1 = tr1.BasisCurve;
            }
            c1.Load(pcf1);
        }
        else
        {
            c1.Load(pcf1, first - delta, last + delta);
        }
        double first1 = pcf1.FirstParameter, last1 = pcf1.LastParameter;

        first = fd2.Interference(jf2).FirstParameter();
        last = fd2.Interference(jf2).LastParameter();
        if ((last - first) < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            return false;
        }
        if (enlarge)
        {
            delta = Math.Min(0.1, 0.05 * (last - first));
        }
        Curve2D? pcf2 = fd2.Interference(jf2).PCurveOnFace();
        if (pcf2 is null)
        {
            return false;
        }
        bool isper2 = pcf2.IsPeriodic;
        if (isper2)
        {
            if (pcf2 is TrimmedCurve2D tr2)
            {
                pcf2 = tr2.BasisCurve;
            }
            c2.Load(pcf2);
        }
        else
        {
            c2.Load(fd2.Interference(jf2).PCurveOnFace()!, first - delta, last + delta);
        }
        double first2 = pcf2.FirstParameter, last2 = pcf2.LastParameter;

        IntersectionPoint2D int2d;
        GInter intersection = new();
        int nbpt, nbseg;
        Point2D p2d;
        if (ReferenceEquals(fd1.Interference(jf1).PCurveOnFace(), fd2.Interference(jf2).PCurveOnFace()))
        {
            intersection.Perform(c1, global::Kerf.Engine.Foundation.Precision.PIntersection(), global::Kerf.Engine.Foundation.Precision.PIntersection());
        }
        else
        {
            intersection.Perform(c1, c2, global::Kerf.Engine.Foundation.Precision.PIntersection(), global::Kerf.Engine.Foundation.Precision.PIntersection());
        }
        if (intersection.IsDone)
        {
            if (!intersection.IsEmpty)
            {
                nbseg = intersection.SegmentCount;
                if (nbseg > 0)
                {
                }
                nbpt = intersection.PointCount;
                if (nbpt >= 1)
                {
                    // The criteria sets to filter the found points in a strict way
                    // are missing. Two different criterions chosen somewhat randomly
                    // are used :
                    // - periodic curves : closest to the border.
                    // - non-periodic curves : the closest to the left of 2 curves
                    //                             modulo sens1 and sens2
                    int2d = intersection.Point(1);
                    p2d = int2d.Value();
                    p1 = int2d.ParamOnFirst();
                    p2 = int2d.ParamOnSecond();
                    if (isper1)
                    {
                        p1 = Recadre(p1, pref1, sens1, first1, last1);
                    }
                    if (isper2)
                    {
                        p2 = Recadre(p2, pref2, sens2, first2, last2);
                    }
                    for (int i = 2; i <= nbpt; i++)
                    {
                        int2d = intersection.Point(i);
                        if (isper1)
                        {
                            double pp1 = int2d.ParamOnFirst();
                            pp1 = Recadre(pp1, pref1, sens1, first1, last1);
                            if ((Math.Abs(pp1 - pref1) < Math.Abs(p1 - pref1)))
                            {
                                p1 = pp1;
                                p2 = int2d.ParamOnSecond();
                                p2d = int2d.Value();
                            }
                            //  Modified by skv - Mon Jun 16 15:51:21 2003 OCC615 Begin
                            else if (check2dDistance && refP2d.DistanceTo(int2d.Value()) < refP2d.DistanceTo(p2d))
                            {
                                double pp2 = int2d.ParamOnSecond();

                                if (isper2)
                                {
                                    pp2 = Recadre(pp2, pref2, sens2, first2, last2);
                                }

                                p1 = pp1;
                                p2 = pp2;
                                p2d = int2d.Value();
                            }
                            //  Modified by skv - Mon Jun 16 15:51:22 2003 OCC615 End
                        }
                        else if (isper2)
                        {
                            double pp2 = int2d.ParamOnSecond();
                            pp2 = Recadre(pp2, pref2, sens2, first2, last2);
                            if ((Math.Abs(pp2 - pref2) < Math.Abs(p2 - pref2)))
                            {
                                p2 = pp2;
                                p1 = int2d.ParamOnFirst();
                                p2d = int2d.Value();
                            }
                            //  Modified by skv - Mon Jun 16 15:51:21 2003 OCC615 Begin
                            else if (check2dDistance && refP2d.DistanceTo(int2d.Value()) < refP2d.DistanceTo(p2d))
                            {
                                double pp1 = int2d.ParamOnFirst();

                                if (isper1)
                                {
                                    pp1 = Recadre(pp1, pref1, sens1, first1, last1);
                                }

                                p1 = pp1;
                                p2 = pp2;
                                p2d = int2d.Value();
                            }
                            //  Modified by skv - Mon Jun 16 15:51:22 2003 OCC615 End
                        }
                        else if (((int2d.ParamOnFirst() - p1) * sens1 < 0.0)
                                 && ((int2d.ParamOnSecond() - p2) * sens2 < 0.0))
                        {
                            p1 = int2d.ParamOnFirst();
                            p2 = int2d.ParamOnSecond();
                            p2d = int2d.Value();
                        }
                        else if ((Math.Abs(int2d.ParamOnFirst() - pref1) < Math.Abs(p1 - pref1))
                                 && (Math.Abs(int2d.ParamOnSecond() - pref2) < Math.Abs(p2 - pref2)))
                        {
                            p1 = int2d.ParamOnFirst();
                            p2 = int2d.ParamOnSecond();
                            p2d = int2d.Value();
                        }
                        else if (check2dDistance && refP2d.DistanceTo(int2d.Value()) < refP2d.DistanceTo(p2d))
                        {
                            p1 = int2d.ParamOnFirst();
                            p2 = int2d.ParamOnSecond();
                            p2d = int2d.Value();
                        }
                    }
                    return true;
                }
                return false;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    //=======================================================================
    // function : ReparamPcurv
    // purpose  : Dans le cas ou la pcurve est une BSpline on verifie
    //           ses parametres et on la reparametre eventuellement.
    //=======================================================================

    /// <summary>When <paramref name="pcurv"/> is a B-spline (or a trimmed one), segments it to its range and
    /// reparametrizes it on [<paramref name="uf"/>, <paramref name="ul"/>].</summary>
    /// <remarks>OCCT segments the B-spline in place; a frozen one (an intersection's result) is segmented on a copy
    /// (decision 1088).</remarks>
    internal static void ChFi3d_ReparamPcurv(double uf, double ul, ref Curve2D? pcurv)
    {
        if (pcurv is null)
        {
            return;
        }
        double upcf = pcurv.FirstParameter;
        double upcl = pcurv.LastParameter;
        Curve2D basis = pcurv;
        if (pcurv is TrimmedCurve2D trpc)
        {
            basis = trpc.BasisCurve;
        }
        if (basis is not BSplineCurve2D pc)
        {
            return;
        }
        if (Math.Abs(upcf - pc.FirstParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion()
            || Math.Abs(upcl - pc.LastParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            pc = Writable(pc); // Kerf: OCCT segments in place; a frozen curve is segmented on a copy (10204)
            pc.Segment(upcf, upcl);
        }
        if (Math.Abs(uf - pc.FirstParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion()
            || Math.Abs(ul - pc.LastParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            NCollection_Array1<Point2D> pol = pc.InternalPoles();
            NCollection_Array1<double> kn = new(pc.InternalKnots());
            NCollection_Array1<int> mu = pc.InternalMultiplicities();
            int deg = pc.Degree;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(uf, ul, kn);
            pc = new BSplineCurve2D(pol, kn, mu, deg);
        }
        pcurv = pc;
    }

    //=======================================================================
    // function : ProjectPCurv
    // purpose  : Calculation of the pcurve corresponding to a line of intersection
    //           3d. Should be called only in analytic cases.
    //=======================================================================

    /// <summary>The projection of <paramref name="hCg"/> on <paramref name="hSg"/> (not a Bézier or B-spline
    /// surface: <paramref name="pcurv"/> and <paramref name="tolreached"/> are then left unchanged).</summary>
    /// <exception cref="MethodNotImplementedException">The projection is not a conic, a Bézier or a B-spline.</exception>
    internal static void ChFi3d_ProjectPCurv(CurveAdaptorBase hCg, SurfaceAdaptorBase hSg, ref Curve2D? pcurv,
                                             double tol, ref double tolreached)
    {
        if (hSg.GetType() != SurfaceKind.BezierSurface && hSg.GetType() != SurfaceKind.BSplineSurface)
        {
            ProjectedCurve projc = new(hSg, hCg, tol);
            tolreached = projc.GetTolerance();
            switch (projc.GetType())
            {
                case Line:
                {
                    pcurv = new LineCurve2D(projc.Line());
                }
                    break;
                case Circle:
                {
                    pcurv = new CircleCurve2D(projc.Circle());
                }
                    break;
                case Ellipse:
                {
                    pcurv = new EllipseCurve2D(projc.Ellipse());
                }
                    break;
                case Hyperbola:
                {
                    pcurv = new HyperbolaCurve2D(projc.Hyperbola());
                }
                    break;
                case Parabola:
                {
                    pcurv = new ParabolaCurve2D(projc.Parabola());
                }
                    break;
                case BezierCurve:
                {
                    pcurv = projc.Bezier();
                }
                    break;
                case BSplineCurve:
                {
                    pcurv = projc.BSpline();
                }
                    break;
                default:
                    throw new MethodNotImplementedException("echec approximation de la pcurve ");
            }
        }
    }

    //=======================================================================
    // function : CheckSameParameter
    // purpose  : Controls a posteriori that sameparameter worked well
    //=======================================================================

    /// <summary>Whether <paramref name="pcurv"/> on <paramref name="s"/> follows <paramref name="c3d"/> within
    /// <paramref name="tol3d"/>, at 45 points; twice the deviation in <paramref name="tolreached"/>.</summary>
    internal static bool ChFi3d_CheckSameParameter(CurveAdaptorBase c3d, Curve2D pcurv, SurfaceAdaptorBase s,
                                                   double tol3d, out double tolreached)
    {
        tolreached = 0.0;
        double f = c3d.FirstParameter();
        double l = c3d.LastParameter();
        int nbp = 45;
        double step = 1.0 / (nbp - 1);
        for (int i = 0; i < nbp; i++)
        {
            double t, u, v;
            t = step * i;
            t = (1 - t) * f + t * l;
            pcurv.Value(t).Coord(out u, out v);
            Point3D pS = s.Value(u, v);
            Point3D pC = c3d.Value(t);
            double d2 = pS.SquareDistanceTo(pC);
            tolreached = Math.Max(tolreached, d2);
        }
        tolreached = Math.Sqrt(tolreached);
        if (tolreached > tol3d)
        {
            tolreached *= 2.0;
            return false;
        }
        tolreached *= 2.0;
        tolreached = Math.Max(tolreached, global::Kerf.Engine.Foundation.Precision.Confusion());
        return true;
    }

    //=======================================================================
    // function : SameParameter
    // purpose  : Encapsulation of Sameparameter
    //=======================================================================

    /// <summary>Makes <paramref name="pcurv"/> same parameter with <paramref name="c3d"/> on <paramref name="s"/>
    /// (Approx_SameParameter when the check fails).</summary>
    internal static bool ChFi3d_SameParameter(CurveAdaptorBase c3d, ref Curve2D pcurv, SurfaceAdaptorBase s,
                                              double tol3d, out double tolreached)
    {
        if (ChFi3d_CheckSameParameter(c3d, pcurv, s, tol3d, out tolreached))
        {
            return true;
        }
        SameParameter sp = new(c3d, pcurv, s, tol3d);
        if (sp.IsDone && !sp.IsSameParameter)
        {
            pcurv = sp.Curve2d()!;
        }
        else if (!sp.IsDone && !sp.IsSameParameter)
        {
            return false;
        }
        tolreached = sp.TolReached();
        return true;
    }

    //=======================================================================
    // function : SameParameter
    // purpose  : Encapsulation de Sameparameter
    //=======================================================================

    /// <summary>Makes <paramref name="pcurv"/> same parameter with <paramref name="c3d"/> on
    /// [<paramref name="pardeb"/>, <paramref name="parfin"/>] on <paramref name="s"/>.</summary>
    internal static bool ChFi3d_SameParameter(Curve3D c3d, ref Curve2D pcurv, Surface s, double pardeb,
                                              double parfin, double tol3d, out double tolreached)
    {
        /*szv:static*/
        SurfaceAdaptor hs = new(s);
        /*szv:static*/
        CurveAdaptor hc = new(c3d, pardeb, parfin);
        return ChFi3d_SameParameter(hc, ref pcurv, hs, tol3d, out tolreached);
    }

    //=======================================================================
    // function : ComputePCurv
    // purpose  : Calculates a straight line in form of BSpline
    //           to guarantee the same range and parameters as of the
    //           reference 3D curve.
    //=======================================================================

    /// <summary>The segment from <paramref name="uv1"/> to <paramref name="uv2"/> on [<paramref name="pardeb"/>,
    /// <paramref name="parfin"/>], made same parameter with <paramref name="c3d"/>.</summary>
    internal static void ChFi3d_ComputePCurv(CurveAdaptorBase c3d, in Point2D uv1, in Point2D uv2,
                                             out Curve2D pcurv, SurfaceAdaptorBase s, double pardeb,
                                             double parfin, double tol3d, out double tolreached, bool reverse = false)
    {
        ChFi3d_ComputePCurv(uv1, uv2, out pcurv, pardeb, parfin, reverse);
        ChFi3d_SameParameter(c3d, ref pcurv, s, tol3d, out tolreached);
    }

    //=======================================================================
    // function : ComputePCurv
    // purpose  : Calculates a straight line in form of BSpline
    //           to guarantee the same range and parameters as of the
    //           reference 3D curve.
    //=======================================================================

    /// <summary>The segment from <paramref name="uv1"/> to <paramref name="uv2"/> on [<paramref name="pardeb"/>,
    /// <paramref name="parfin"/>], made same parameter with <paramref name="c3d"/> on <paramref name="s"/>.</summary>
    internal static void ChFi3d_ComputePCurv(Curve3D c3d, in Point2D uv1, in Point2D uv2,
                                             out Curve2D pcurv, Surface s, double pardeb, double parfin,
                                             double tol3d, out double tolreached, bool reverse = false)
    {
        SurfaceAdaptorBase hs = new SurfaceAdaptor(s);
        CurveAdaptorBase hc = new CurveAdaptor(c3d, pardeb, parfin);
        ChFi3d_ComputePCurv(hc, uv1, uv2, out pcurv, hs, pardeb, parfin, tol3d, out tolreached, reverse);
    }

    //=======================================================================
    // function : ComputePCurv
    // purpose  : Calculates a straight line in form of BSpline
    //           to guarantee the same range.
    //=======================================================================

    /// <summary>The segment from <paramref name="uv1"/> to <paramref name="uv2"/> (reversed by
    /// <paramref name="reverse"/>) on [<paramref name="pardeb"/>, <paramref name="parfin"/>]: an isoparametric line
    /// when it is one with that length, else a degree 1 B-spline; trimmed.</summary>
    internal static void ChFi3d_ComputePCurv(in Point2D uv1, in Point2D uv2, out Curve2D pcurv,
                                             double pardeb, double parfin, bool reverse = false)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        Point2D p1, p2;
        if (!reverse)
        {
            p1 = uv1;
            p2 = uv2;
        }
        else
        {
            p1 = uv2;
            p2 = uv1;
        }

        if (Math.Abs(p1.X - p2.X) <= tol && Math.Abs((p2.Y - p1.Y) - (parfin - pardeb)) <= tol)
        {
            Point2D ppp = new(p1.X, p1.Y - pardeb);
            pcurv = new LineCurve2D(ppp, Constants.DY2d());
        }
        else if (Math.Abs(p1.X - p2.X) <= tol && Math.Abs((p1.Y - p2.Y) - (parfin - pardeb)) <= tol)
        {
            Point2D ppp = new(p1.X, p1.Y + pardeb);
            pcurv = new LineCurve2D(ppp, Constants.DY2d().Reversed());
        }
        else if (Math.Abs(p1.Y - p2.Y) <= tol && Math.Abs((p2.X - p1.X) - (parfin - pardeb)) <= tol)
        {
            Point2D ppp = new(p1.X - pardeb, p1.Y);
            pcurv = new LineCurve2D(ppp, Constants.DX2d());
        }
        else if (Math.Abs(p1.Y - p2.Y) <= tol && Math.Abs((p1.X - p2.X) - (parfin - pardeb)) <= tol)
        {
            Point2D ppp = new(p1.X + pardeb, p1.Y);
            pcurv = new LineCurve2D(ppp, Constants.DX2d().Reversed());
        }
        else
        {
            NCollection_Array1<Point2D> p = new(1, 2);
            NCollection_Array1<double> k = new(1, 2);
            NCollection_Array1<int> m = new(1, 2);
            m.Init(2);
            k[1] = pardeb;
            k[2] = parfin;
            p[1] = p1;
            p[2] = p2;
            pcurv = new BSplineCurve2D(p, k, m, 1);
        }
        pcurv = new TrimmedCurve2D(pcurv, pardeb, parfin);
    }

    //=================================================================================================

    /// <summary>The boundary on <paramref name="fac"/> from <paramref name="pfac1"/> to <paramref name="pfac2"/>
    /// with the given tangents (reversed by the senses); <paramref name="curv"/> its pcurve.</summary>
    internal static Boundary ChFi3d_mkbound(SurfaceAdaptorBase fac, out Curve2D curv, int sens1,
                                                     in Point2D pfac1, in Vector2D vfac1, int sens2,
                                                     in Point2D pfac2, in Vector2D vfac2, double t3d, double ta)
    {
        Direction2D v1 = new(vfac1);
        if (sens1 == 1)
        {
            v1 = v1.Reversed();
        }
        Direction2D v2 = new(vfac2);
        if (sens2 == 1)
        {
            v2 = v2.Reversed();
        }
        curv = ChFi3d_BuildPCurve(fac, pfac1, new Vector2D(v1), pfac2, new Vector2D(v2), false);
        return ChFi3d_mkbound(fac, curv, t3d, ta);
    }

    //=================================================================================================

    /// <summary>The boundary on <paramref name="surf"/> from <paramref name="p1"/> to <paramref name="p2"/> with
    /// the given 3D tangents (reversed in place by the senses); <paramref name="curv"/> its pcurve.</summary>
    internal static Boundary ChFi3d_mkbound(SurfaceAdaptorBase surf, out Curve2D curv, int sens1,
                                                     in Point2D p1, ref Vector3D v1, int sens2, in Point2D p2,
                                                     ref Vector3D v2, double t3d, double ta)
    {
        if (sens1 == 1)
        {
            v1 = v1.Reversed();
        }
        if (sens2 == 1)
        {
            v2 = v2.Reversed();
        }
        curv = ChFi3d_BuildPCurve(surf, p1, v1, p2, v2);
        return ChFi3d_mkbound(surf, curv, t3d, ta);
    }

    //=================================================================================================

    /// <summary>The boundary on <paramref name="s"/> along the segment from <paramref name="p1"/> to
    /// <paramref name="p2"/>.</summary>
    internal static Boundary ChFi3d_mkbound(Surface s, in Point2D p1, in Point2D p2, double t3d,
                                                     double ta, bool isfreeboundary = false)
    {
        SurfaceAdaptorBase hs = new SurfaceAdaptor(s);
        return ChFi3d_mkbound(hs, p1, p2, t3d, ta, isfreeboundary);
    }

    //=================================================================================================

    /// <summary>The boundary on <paramref name="hs"/> along the segment from <paramref name="p1"/> to
    /// <paramref name="p2"/>.</summary>
    internal static Boundary ChFi3d_mkbound(SurfaceAdaptorBase hs, in Point2D p1, in Point2D p2,
                                                     double t3d, double ta, bool isfreeboundary = false)
    {
        NCollection_Array1<Point2D> pol = new(1, 2);
        pol[1] = p1;
        pol[2] = p2;
        Curve2D curv = new BezierCurve2D(pol);
        return ChFi3d_mkbound(hs, curv, t3d, ta, isfreeboundary);
    }

    //=================================================================================================

    /// <summary>The boundary on <paramref name="hs"/> along <paramref name="curv"/>: with the surface's normals, or a
    /// simple boundary if it is free.</summary>
    internal static Boundary ChFi3d_mkbound(SurfaceAdaptorBase hs, Curve2D curv, double t3d, double ta,
                                                     bool isfreeboundary = false)
    {
        CurveAdaptor2D hc = new(curv);
        CurveOnSurface cOnS = new(hc, hs);
        if (isfreeboundary)
        {
            CurveOnSurface hcOnS = (CurveOnSurface)cOnS.ShallowCopy();
            return new SimpleBound(hcOnS, t3d, ta);
        }
        return new BoundWithSurf(cOnS, t3d, ta);
    }

    //=================================================================================================

    /// <summary>The boundary on <paramref name="fac"/> along the segment from <paramref name="p1"/> to
    /// <paramref name="p2"/>; <paramref name="curv"/> its pcurve.</summary>
    internal static Boundary ChFi3d_mkbound(SurfaceAdaptorBase fac, out Curve2D curv, in Point2D p1,
                                                     in Point2D p2, double t3d, double ta,
                                                     bool isfreeboundary = false)
    {
        NCollection_Array1<Point2D> pol = new(1, 2);
        pol[1] = p1;
        pol[2] = p2;
        curv = new BezierCurve2D(pol);
        return ChFi3d_mkbound(fac, curv, t3d, ta, isfreeboundary);
    }

    //=================================================================================================

    /// <summary>A cubic from <paramref name="p1"/> to <paramref name="p2"/> with the tangent directions
    /// <paramref name="d1"/> and <paramref name="d2"/> (turned towards each other first, by
    /// <paramref name="redresse"/>).</summary>
    internal static Curve2D ChFi3d_BuildPCurve(in Point2D p1, ref Direction2D d1, in Point2D p2, ref Direction2D d2,
                                                    bool redresse = true)
    {
        Vector2D vref = new(p1, p2);
        Direction2D dref = new(vref);
        double mref = vref.Magnitude;
        if (redresse)
        {
            if (d1.Dot(dref) < 0.0)
            {
                d1 = d1.Reversed();
            }
            if (d2.Dot(dref) > 0.0)
            {
                d2 = d2.Reversed();
            }
        }
        // On fait une cubique a la mords moi le noeud
        NCollection_Array1<Point2D> pol = new(1, 4);
        pol[1] = p1;
        pol[4] = p2;
        double lambda1 = Math.Max(Math.Abs(d2.Dot(d1)), Math.Abs(dref.Dot(d1)));
        lambda1 = Math.Max(0.5 * mref * lambda1, 1.0e-5);
        pol[2] = new Point2D(p1.XY().Added(d1.XY().Multiplied(lambda1)));
        double lambda2 = Math.Max(Math.Abs(d1.Dot(d2)), Math.Abs(dref.Dot(d2)));
        lambda2 = Math.Max(0.5 * mref * lambda2, 1.0e-5);
        pol[3] = new Point2D(p2.XY().Added(d2.XY().Multiplied(lambda2)));
        return new BezierCurve2D(pol);
    }

    //=================================================================================================

    /// <summary>The cubic of the 2D tangents <paramref name="v1"/> and <paramref name="v2"/> built in the space
    /// scaled by the surface's resolutions.</summary>
    internal static Curve2D ChFi3d_BuildPCurve(SurfaceAdaptorBase surf, in Point2D p1, in Vector2D v1,
                                                    in Point2D p2, in Vector2D v2, bool redresse = false)
    {
        Point2D pp1 = p1, pp2 = p2;
        Vector2D vv1 = v1, vv2 = v2;
        double ures = surf.UResolution(1.0);
        double vres = surf.VResolution(1.0);
        double invures = 1.0 / ures;
        double invvres = 1.0 / vres;
        pp1 = pp1.WithX(invures * pp1.X);
        pp1 = pp1.WithY(invvres * pp1.Y);
        pp2 = pp2.WithX(invures * pp2.X);
        pp2 = pp2.WithY(invvres * pp2.Y);
        vv1 = vv1.WithX(invures * vv1.X);
        vv1 = vv1.WithY(invvres * vv1.Y);
        vv2 = vv2.WithX(invures * vv2.X);
        vv2 = vv2.WithY(invvres * vv2.Y);
        Direction2D d1 = new(vv1), d2 = new(vv2);
        Curve2D g2dc = ChFi3d_BuildPCurve(pp1, ref d1, pp2, ref d2, redresse);
        BezierCurve2D pc = (BezierCurve2D)g2dc;
        int nbp = pc.PoleCount;
        for (int ip = 1; ip <= nbp; ip++)
        {
            Point2D pol = pc.Pole(ip);
            pol = pol.WithX(ures * pol.X);
            pol = pol.WithY(vres * pol.Y);
            pc.SetPole(ip, pol);
        }
        return pc;
    }

    //=================================================================================================

    /// <summary>The cubic on <paramref name="surf"/> of the 3D tangents <paramref name="v1"/> and
    /// <paramref name="v2"/> brought to the parametric space.</summary>
    internal static Curve2D ChFi3d_BuildPCurve(SurfaceAdaptorBase surf, in Point2D p1, in Vector3D v1,
                                                    in Point2D p2, in Vector3D v2, bool redresse = false)
    {
        Vector3D d1u, d1v;
        Point3D pp1, pp2;
        double du, dv;
        surf.D1(p1.X, p1.Y, out pp1, out d1u, out d1v);
        ChFi3d_Coefficient(v1, d1u, d1v, out du, out dv);
        Vector2D vv1 = new(du, dv);
        surf.D1(p2.X, p2.Y, out pp2, out d1u, out d1v);
        ChFi3d_Coefficient(v2, d1u, d1v, out du, out dv);
        Vector2D vv2 = new(du, dv);
        Vector3D vref = new(pp1, pp2);
        if (redresse)
        {
            if (vref.Dot(v1) < 0.0)
            {
                vv1 = vv1.Reversed();
            }
            if (vref.Dot(v2) > 0.0)
            {
                vv2 = vv2.Reversed();
            }
        }
        return ChFi3d_BuildPCurve(surf, p1, vv1, p2, vv2, false);
    }

    // Kerf: OCCT changes these curves in place; one frozen elsewhere (a projection's or an intersection's
    // result) is changed on a copy (decisions 974 and 1088).
    private static T Writable<T>(T curve) where T : Geometry2D =>
        curve.IsFrozen ? (T)curve.Copy() : curve;
}
