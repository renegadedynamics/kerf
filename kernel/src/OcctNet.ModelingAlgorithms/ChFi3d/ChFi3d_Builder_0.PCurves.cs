// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_IntTraces to ChFi3d_BuildPCurve)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Projection;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

internal static partial class ChFi3d_Builder_0
{
    //=================================================================================================

    /// <summary>
    /// The crossing of the traces on their common face of the side <paramref name="jf1"/> of <paramref name="fd1"/>
    /// and <paramref name="jf2"/> of <paramref name="fd2"/>: their parameters <paramref name="p1"/> and
    /// <paramref name="p2"/>, chosen among several by OCCT's criteria (closest to the references, to the left by the
    /// senses, or to <paramref name="RefP2d"/>).
    /// </summary>
    internal static bool ChFi3d_IntTraces(ChFiDS_SurfData fd1, double pref1, ref double p1, int jf1, int sens1,
                                          ChFiDS_SurfData fd2, double pref2, ref double p2, int jf2, int sens2,
                                          in Point2D RefP2d, bool Check2dDistance = false, bool enlarge = false)
    {
        CurveAdaptor2D C1 = new();
        CurveAdaptor2D C2 = new();
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
                pcf1 = tr1.BasisCurve();
            }
            C1.Load(pcf1);
        }
        else
        {
            C1.Load(pcf1, first - delta, last + delta);
        }
        double first1 = pcf1.FirstParameter(), last1 = pcf1.LastParameter();

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
                pcf2 = tr2.BasisCurve();
            }
            C2.Load(pcf2);
        }
        else
        {
            C2.Load(fd2.Interference(jf2).PCurveOnFace()!, first - delta, last + delta);
        }
        double first2 = pcf2.FirstParameter(), last2 = pcf2.LastParameter();

        IntRes2d_IntersectionPoint int2d;
        Geom2dInt_GInter Intersection = new();
        int nbpt, nbseg;
        Point2D p2d;
        if (ReferenceEquals(fd1.Interference(jf1).PCurveOnFace(), fd2.Interference(jf2).PCurveOnFace()))
        {
            Intersection.Perform(C1, global::Kerf.Engine.Foundation.Precision.PIntersection(), global::Kerf.Engine.Foundation.Precision.PIntersection());
        }
        else
        {
            Intersection.Perform(C1, C2, global::Kerf.Engine.Foundation.Precision.PIntersection(), global::Kerf.Engine.Foundation.Precision.PIntersection());
        }
        if (Intersection.IsDone())
        {
            if (!Intersection.IsEmpty())
            {
                nbseg = Intersection.NbSegments();
                if (nbseg > 0)
                {
                }
                nbpt = Intersection.NbPoints();
                if (nbpt >= 1)
                {
                    // The criteria sets to filter the found points in a strict way
                    // are missing. Two different criterions chosen somewhat randomly
                    // are used :
                    // - periodic curves : closest to the border.
                    // - non-periodic curves : the closest to the left of 2 curves
                    //                             modulo sens1 and sens2
                    int2d = Intersection.Point(1);
                    p2d = int2d.Value();
                    p1 = int2d.ParamOnFirst();
                    p2 = int2d.ParamOnSecond();
                    if (isper1)
                    {
                        p1 = recadre(p1, pref1, sens1, first1, last1);
                    }
                    if (isper2)
                    {
                        p2 = recadre(p2, pref2, sens2, first2, last2);
                    }
                    for (int i = 2; i <= nbpt; i++)
                    {
                        int2d = Intersection.Point(i);
                        if (isper1)
                        {
                            double pp1 = int2d.ParamOnFirst();
                            pp1 = recadre(pp1, pref1, sens1, first1, last1);
                            if ((Math.Abs(pp1 - pref1) < Math.Abs(p1 - pref1)))
                            {
                                p1 = pp1;
                                p2 = int2d.ParamOnSecond();
                                p2d = int2d.Value();
                            }
                            //  Modified by skv - Mon Jun 16 15:51:21 2003 OCC615 Begin
                            else if (Check2dDistance && RefP2d.Distance(int2d.Value()) < RefP2d.Distance(p2d))
                            {
                                double pp2 = int2d.ParamOnSecond();

                                if (isper2)
                                {
                                    pp2 = recadre(pp2, pref2, sens2, first2, last2);
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
                            pp2 = recadre(pp2, pref2, sens2, first2, last2);
                            if ((Math.Abs(pp2 - pref2) < Math.Abs(p2 - pref2)))
                            {
                                p2 = pp2;
                                p1 = int2d.ParamOnFirst();
                                p2d = int2d.Value();
                            }
                            //  Modified by skv - Mon Jun 16 15:51:21 2003 OCC615 Begin
                            else if (Check2dDistance && RefP2d.Distance(int2d.Value()) < RefP2d.Distance(p2d))
                            {
                                double pp1 = int2d.ParamOnFirst();

                                if (isper1)
                                {
                                    pp1 = recadre(pp1, pref1, sens1, first1, last1);
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
                        else if (Check2dDistance && RefP2d.Distance(int2d.Value()) < RefP2d.Distance(p2d))
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

    /// <summary>When <paramref name="Pcurv"/> is a B-spline (or a trimmed one), segments it to its range and
    /// reparametrizes it on [<paramref name="Uf"/>, <paramref name="Ul"/>].</summary>
    /// <remarks>OCCT segments the B-spline in place; a frozen one (an intersection's result) is segmented on a copy
    /// (decision 1088).</remarks>
    internal static void ChFi3d_ReparamPcurv(double Uf, double Ul, ref Curve2D? Pcurv)
    {
        if (Pcurv is null)
        {
            return;
        }
        double upcf = Pcurv.FirstParameter();
        double upcl = Pcurv.LastParameter();
        Curve2D basis = Pcurv;
        if (Pcurv is TrimmedCurve2D trpc)
        {
            basis = trpc.BasisCurve();
        }
        if (basis is not BSplineCurve2D pc)
        {
            return;
        }
        if (Math.Abs(upcf - pc.FirstParameter()) > global::Kerf.Engine.Foundation.Precision.PConfusion()
            || Math.Abs(upcl - pc.LastParameter()) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            pc = Writable(pc); // OCCT.NET: OCCT segments in place; a frozen curve is segmented on a copy (10204)
            pc.Segment(upcf, upcl);
        }
        if (Math.Abs(Uf - pc.FirstParameter()) > global::Kerf.Engine.Foundation.Precision.PConfusion()
            || Math.Abs(Ul - pc.LastParameter()) > global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            NCollection_Array1<Point2D> pol = pc.InternalPoles();
            NCollection_Array1<double> kn = new(pc.InternalKnots());
            NCollection_Array1<int> mu = pc.InternalMultiplicities();
            int deg = pc.Degree();
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(Uf, Ul, kn);
            pc = new BSplineCurve2D(pol, kn, mu, deg);
        }
        Pcurv = pc;
    }

    //=======================================================================
    // function : ProjectPCurv
    // purpose  : Calculation of the pcurve corresponding to a line of intersection
    //           3d. Should be called only in analytic cases.
    //=======================================================================

    /// <summary>The projection of <paramref name="HCg"/> on <paramref name="HSg"/> (not a Bézier or B-spline
    /// surface: <paramref name="Pcurv"/> and <paramref name="tolreached"/> are then left unchanged).</summary>
    /// <exception cref="MethodNotImplementedException">The projection is not a conic, a Bézier or a B-spline.</exception>
    internal static void ChFi3d_ProjectPCurv(CurveAdaptorBase HCg, SurfaceAdaptorBase HSg, ref Curve2D? Pcurv,
                                             double tol, ref double tolreached)
    {
        if (HSg.GetType() != SurfaceKind.BezierSurface && HSg.GetType() != SurfaceKind.BSplineSurface)
        {
            ProjectedCurve Projc = new(HSg, HCg, tol);
            tolreached = Projc.GetTolerance();
            switch (Projc.GetType())
            {
                case Line:
                {
                    Pcurv = new LineCurve2D(Projc.Line());
                }
                    break;
                case Circle:
                {
                    Pcurv = new CircleCurve2D(Projc.Circle());
                }
                    break;
                case Ellipse:
                {
                    Pcurv = new EllipseCurve2D(Projc.Ellipse());
                }
                    break;
                case Hyperbola:
                {
                    Pcurv = new HyperbolaCurve2D(Projc.Hyperbola());
                }
                    break;
                case Parabola:
                {
                    Pcurv = new ParabolaCurve2D(Projc.Parabola());
                }
                    break;
                case BezierCurve:
                {
                    Pcurv = Projc.Bezier();
                }
                    break;
                case BSplineCurve:
                {
                    Pcurv = Projc.BSpline();
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

    /// <summary>Whether <paramref name="Pcurv"/> on <paramref name="S"/> follows <paramref name="C3d"/> within
    /// <paramref name="tol3d"/>, at 45 points; twice the deviation in <paramref name="tolreached"/>.</summary>
    internal static bool ChFi3d_CheckSameParameter(CurveAdaptorBase C3d, Curve2D Pcurv, SurfaceAdaptorBase S,
                                                   double tol3d, out double tolreached)
    {
        tolreached = 0.0;
        double f = C3d.FirstParameter();
        double l = C3d.LastParameter();
        int nbp = 45;
        double step = 1.0 / (nbp - 1);
        for (int i = 0; i < nbp; i++)
        {
            double t, u, v;
            t = step * i;
            t = (1 - t) * f + t * l;
            Pcurv.Value(t).Coord(out u, out v);
            Point3D pS = S.Value(u, v);
            Point3D pC = C3d.Value(t);
            double d2 = pS.SquareDistance(pC);
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

    /// <summary>Makes <paramref name="Pcurv"/> same parameter with <paramref name="C3d"/> on <paramref name="S"/>
    /// (Approx_SameParameter when the check fails).</summary>
    internal static bool ChFi3d_SameParameter(CurveAdaptorBase C3d, ref Curve2D Pcurv, SurfaceAdaptorBase S,
                                              double tol3d, out double tolreached)
    {
        if (ChFi3d_CheckSameParameter(C3d, Pcurv, S, tol3d, out tolreached))
        {
            return true;
        }
        SameParameter sp = new(C3d, Pcurv, S, tol3d);
        if (sp.IsDone && !sp.IsSameParameter)
        {
            Pcurv = sp.Curve2d()!;
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

    /// <summary>Makes <paramref name="Pcurv"/> same parameter with <paramref name="C3d"/> on
    /// [<paramref name="Pardeb"/>, <paramref name="Parfin"/>] on <paramref name="S"/>.</summary>
    internal static bool ChFi3d_SameParameter(Curve3D C3d, ref Curve2D Pcurv, Surface S, double Pardeb,
                                              double Parfin, double tol3d, out double tolreached)
    {
        /*szv:static*/
        SurfaceAdaptor hs = new(S);
        /*szv:static*/
        CurveAdaptor hc = new(C3d, Pardeb, Parfin);
        return ChFi3d_SameParameter(hc, ref Pcurv, hs, tol3d, out tolreached);
    }

    //=======================================================================
    // function : ComputePCurv
    // purpose  : Calculates a straight line in form of BSpline
    //           to guarantee the same range and parameters as of the
    //           reference 3D curve.
    //=======================================================================

    /// <summary>The segment from <paramref name="UV1"/> to <paramref name="UV2"/> on [<paramref name="Pardeb"/>,
    /// <paramref name="Parfin"/>], made same parameter with <paramref name="C3d"/>.</summary>
    internal static void ChFi3d_ComputePCurv(CurveAdaptorBase C3d, in Point2D UV1, in Point2D UV2,
                                             out Curve2D Pcurv, SurfaceAdaptorBase S, double Pardeb,
                                             double Parfin, double tol3d, out double tolreached, bool reverse = false)
    {
        ChFi3d_ComputePCurv(UV1, UV2, out Pcurv, Pardeb, Parfin, reverse);
        ChFi3d_SameParameter(C3d, ref Pcurv, S, tol3d, out tolreached);
    }

    //=======================================================================
    // function : ComputePCurv
    // purpose  : Calculates a straight line in form of BSpline
    //           to guarantee the same range and parameters as of the
    //           reference 3D curve.
    //=======================================================================

    /// <summary>The segment from <paramref name="UV1"/> to <paramref name="UV2"/> on [<paramref name="Pardeb"/>,
    /// <paramref name="Parfin"/>], made same parameter with <paramref name="C3d"/> on <paramref name="S"/>.</summary>
    internal static void ChFi3d_ComputePCurv(Curve3D C3d, in Point2D UV1, in Point2D UV2,
                                             out Curve2D Pcurv, Surface S, double Pardeb, double Parfin,
                                             double tol3d, out double tolreached, bool reverse = false)
    {
        SurfaceAdaptorBase hs = new SurfaceAdaptor(S);
        CurveAdaptorBase hc = new CurveAdaptor(C3d, Pardeb, Parfin);
        ChFi3d_ComputePCurv(hc, UV1, UV2, out Pcurv, hs, Pardeb, Parfin, tol3d, out tolreached, reverse);
    }

    //=======================================================================
    // function : ComputePCurv
    // purpose  : Calculates a straight line in form of BSpline
    //           to guarantee the same range.
    //=======================================================================

    /// <summary>The segment from <paramref name="UV1"/> to <paramref name="UV2"/> (reversed by
    /// <paramref name="reverse"/>) on [<paramref name="Pardeb"/>, <paramref name="Parfin"/>]: an isoparametric line
    /// when it is one with that length, else a degree 1 B-spline; trimmed.</summary>
    internal static void ChFi3d_ComputePCurv(in Point2D UV1, in Point2D UV2, out Curve2D Pcurv,
                                             double Pardeb, double Parfin, bool reverse = false)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        Point2D p1, p2;
        if (!reverse)
        {
            p1 = UV1;
            p2 = UV2;
        }
        else
        {
            p1 = UV2;
            p2 = UV1;
        }

        if (Math.Abs(p1.X() - p2.X()) <= tol && Math.Abs((p2.Y() - p1.Y()) - (Parfin - Pardeb)) <= tol)
        {
            Point2D ppp = new(p1.X(), p1.Y() - Pardeb);
            Pcurv = new LineCurve2D(ppp, Constants.DY2d());
        }
        else if (Math.Abs(p1.X() - p2.X()) <= tol && Math.Abs((p1.Y() - p2.Y()) - (Parfin - Pardeb)) <= tol)
        {
            Point2D ppp = new(p1.X(), p1.Y() + Pardeb);
            Pcurv = new LineCurve2D(ppp, Constants.DY2d().Reversed());
        }
        else if (Math.Abs(p1.Y() - p2.Y()) <= tol && Math.Abs((p2.X() - p1.X()) - (Parfin - Pardeb)) <= tol)
        {
            Point2D ppp = new(p1.X() - Pardeb, p1.Y());
            Pcurv = new LineCurve2D(ppp, Constants.DX2d());
        }
        else if (Math.Abs(p1.Y() - p2.Y()) <= tol && Math.Abs((p1.X() - p2.X()) - (Parfin - Pardeb)) <= tol)
        {
            Point2D ppp = new(p1.X() + Pardeb, p1.Y());
            Pcurv = new LineCurve2D(ppp, Constants.DX2d().Reversed());
        }
        else
        {
            NCollection_Array1<Point2D> p = new(1, 2);
            NCollection_Array1<double> k = new(1, 2);
            NCollection_Array1<int> m = new(1, 2);
            m.Init(2);
            k[1] = Pardeb;
            k[2] = Parfin;
            p[1] = p1;
            p[2] = p2;
            Pcurv = new BSplineCurve2D(p, k, m, 1);
        }
        Pcurv = new TrimmedCurve2D(Pcurv, Pardeb, Parfin);
    }

    //=================================================================================================

    /// <summary>The boundary on <paramref name="Fac"/> from <paramref name="pfac1"/> to <paramref name="pfac2"/>
    /// with the given tangents (reversed by the senses); <paramref name="curv"/> its pcurve.</summary>
    internal static GeomFill_Boundary ChFi3d_mkbound(SurfaceAdaptorBase Fac, out Curve2D curv, int sens1,
                                                     in Point2D pfac1, in Vector2D vfac1, int sens2,
                                                     in Point2D pfac2, in Vector2D vfac2, double t3d, double ta)
    {
        Direction2D v1 = new(vfac1);
        if (sens1 == 1)
        {
            v1.Reverse();
        }
        Direction2D v2 = new(vfac2);
        if (sens2 == 1)
        {
            v2.Reverse();
        }
        curv = ChFi3d_BuildPCurve(Fac, pfac1, new Vector2D(v1), pfac2, new Vector2D(v2), false);
        return ChFi3d_mkbound(Fac, curv, t3d, ta);
    }

    //=================================================================================================

    /// <summary>The boundary on <paramref name="Surf"/> from <paramref name="p1"/> to <paramref name="p2"/> with
    /// the given 3D tangents (reversed in place by the senses); <paramref name="curv"/> its pcurve.</summary>
    internal static GeomFill_Boundary ChFi3d_mkbound(SurfaceAdaptorBase Surf, out Curve2D curv, int sens1,
                                                     in Point2D p1, ref Vector3D v1, int sens2, in Point2D p2,
                                                     ref Vector3D v2, double t3d, double ta)
    {
        if (sens1 == 1)
        {
            v1.Reverse();
        }
        if (sens2 == 1)
        {
            v2.Reverse();
        }
        curv = ChFi3d_BuildPCurve(Surf, p1, v1, p2, v2);
        return ChFi3d_mkbound(Surf, curv, t3d, ta);
    }

    //=================================================================================================

    /// <summary>The boundary on <paramref name="s"/> along the segment from <paramref name="p1"/> to
    /// <paramref name="p2"/>.</summary>
    internal static GeomFill_Boundary ChFi3d_mkbound(Surface s, in Point2D p1, in Point2D p2, double t3d,
                                                     double ta, bool isfreeboundary = false)
    {
        SurfaceAdaptorBase HS = new SurfaceAdaptor(s);
        return ChFi3d_mkbound(HS, p1, p2, t3d, ta, isfreeboundary);
    }

    //=================================================================================================

    /// <summary>The boundary on <paramref name="HS"/> along the segment from <paramref name="p1"/> to
    /// <paramref name="p2"/>.</summary>
    internal static GeomFill_Boundary ChFi3d_mkbound(SurfaceAdaptorBase HS, in Point2D p1, in Point2D p2,
                                                     double t3d, double ta, bool isfreeboundary = false)
    {
        NCollection_Array1<Point2D> pol = new(1, 2);
        pol[1] = p1;
        pol[2] = p2;
        Curve2D curv = new BezierCurve2D(pol);
        return ChFi3d_mkbound(HS, curv, t3d, ta, isfreeboundary);
    }

    //=================================================================================================

    /// <summary>The boundary on <paramref name="HS"/> along <paramref name="curv"/>: with the surface's normals, or a
    /// simple boundary if it is free.</summary>
    internal static GeomFill_Boundary ChFi3d_mkbound(SurfaceAdaptorBase HS, Curve2D curv, double t3d, double ta,
                                                     bool isfreeboundary = false)
    {
        CurveAdaptor2D HC = new(curv);
        CurveOnSurface COnS = new(HC, HS);
        if (isfreeboundary)
        {
            CurveOnSurface HCOnS = (CurveOnSurface)COnS.ShallowCopy();
            return new GeomFill_SimpleBound(HCOnS, t3d, ta);
        }
        return new GeomFill_BoundWithSurf(COnS, t3d, ta);
    }

    //=================================================================================================

    /// <summary>The boundary on <paramref name="Fac"/> along the segment from <paramref name="p1"/> to
    /// <paramref name="p2"/>; <paramref name="curv"/> its pcurve.</summary>
    internal static GeomFill_Boundary ChFi3d_mkbound(SurfaceAdaptorBase Fac, out Curve2D curv, in Point2D p1,
                                                     in Point2D p2, double t3d, double ta,
                                                     bool isfreeboundary = false)
    {
        NCollection_Array1<Point2D> pol = new(1, 2);
        pol[1] = p1;
        pol[2] = p2;
        curv = new BezierCurve2D(pol);
        return ChFi3d_mkbound(Fac, curv, t3d, ta, isfreeboundary);
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
        double mref = vref.Magnitude();
        if (redresse)
        {
            if (d1.Dot(dref) < 0.0)
            {
                d1.Reverse();
            }
            if (d2.Dot(dref) > 0.0)
            {
                d2.Reverse();
            }
        }
        // On fait une cubique a la mords moi le noeud
        NCollection_Array1<Point2D> pol = new(1, 4);
        pol[1] = p1;
        pol[4] = p2;
        double Lambda1 = Math.Max(Math.Abs(d2.Dot(d1)), Math.Abs(dref.Dot(d1)));
        Lambda1 = Math.Max(0.5 * mref * Lambda1, 1.0e-5);
        pol[2] = new Point2D(p1.XY().Added(d1.XY().Multiplied(Lambda1)));
        double Lambda2 = Math.Max(Math.Abs(d1.Dot(d2)), Math.Abs(dref.Dot(d2)));
        Lambda2 = Math.Max(0.5 * mref * Lambda2, 1.0e-5);
        pol[3] = new Point2D(p2.XY().Added(d2.XY().Multiplied(Lambda2)));
        return new BezierCurve2D(pol);
    }

    //=================================================================================================

    /// <summary>The cubic of the 2D tangents <paramref name="v1"/> and <paramref name="v2"/> built in the space
    /// scaled by the surface's resolutions.</summary>
    internal static Curve2D ChFi3d_BuildPCurve(SurfaceAdaptorBase Surf, in Point2D p1, in Vector2D v1,
                                                    in Point2D p2, in Vector2D v2, bool redresse = false)
    {
        Point2D pp1 = p1, pp2 = p2;
        Vector2D vv1 = v1, vv2 = v2;
        double ures = Surf.UResolution(1.0);
        double vres = Surf.VResolution(1.0);
        double invures = 1.0 / ures;
        double invvres = 1.0 / vres;
        pp1.SetX(invures * pp1.X());
        pp1.SetY(invvres * pp1.Y());
        pp2.SetX(invures * pp2.X());
        pp2.SetY(invvres * pp2.Y());
        vv1.SetX(invures * vv1.X());
        vv1.SetY(invvres * vv1.Y());
        vv2.SetX(invures * vv2.X());
        vv2.SetY(invvres * vv2.Y());
        Direction2D d1 = new(vv1), d2 = new(vv2);
        Curve2D g2dc = ChFi3d_BuildPCurve(pp1, ref d1, pp2, ref d2, redresse);
        BezierCurve2D pc = (BezierCurve2D)g2dc;
        int nbp = pc.PoleCount;
        for (int ip = 1; ip <= nbp; ip++)
        {
            Point2D pol = pc.Pole(ip);
            pol.SetX(ures * pol.X());
            pol.SetY(vres * pol.Y());
            pc.SetPole(ip, pol);
        }
        return pc;
    }

    //=================================================================================================

    /// <summary>The cubic on <paramref name="Surf"/> of the 3D tangents <paramref name="v1"/> and
    /// <paramref name="v2"/> brought to the parametric space.</summary>
    internal static Curve2D ChFi3d_BuildPCurve(SurfaceAdaptorBase Surf, in Point2D p1, in Vector3D v1,
                                                    in Point2D p2, in Vector3D v2, bool redresse = false)
    {
        Vector3D D1u, D1v;
        Point3D PP1, PP2;
        double DU, DV;
        Surf.D1(p1.X(), p1.Y(), out PP1, out D1u, out D1v);
        ChFi3d_Coefficient(v1, D1u, D1v, out DU, out DV);
        Vector2D vv1 = new(DU, DV);
        Surf.D1(p2.X(), p2.Y(), out PP2, out D1u, out D1v);
        ChFi3d_Coefficient(v2, D1u, D1v, out DU, out DV);
        Vector2D vv2 = new(DU, DV);
        Vector3D Vref = new(PP1, PP2);
        if (redresse)
        {
            if (Vref.Dot(v1) < 0.0)
            {
                vv1.Reverse();
            }
            if (Vref.Dot(v2) > 0.0)
            {
                vv2.Reverse();
            }
        }
        return ChFi3d_BuildPCurve(Surf, p1, vv1, p2, vv2, false);
    }

    // OCCT.NET: OCCT changes these curves in place; one frozen elsewhere (a projection's or an intersection's
    // result) is changed on a copy (decisions 974 and 1088).
    private static T Writable<T>(T theCurve) where T : Geometry2D =>
        theCurve.IsFrozen ? (T)theCurve.Copy() : theCurve;
}
