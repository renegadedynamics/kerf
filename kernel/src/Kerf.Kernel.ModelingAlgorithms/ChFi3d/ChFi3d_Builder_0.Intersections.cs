// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_IntCS to GoodExt)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Extrema;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Fillets;

internal static partial class Builder_0
{
    //=======================================================================
    // function : IntCS
    // purpose  : Fast calculation of the intersection curve surface.
    //
    //=======================================================================

    /// <summary>
    /// The intersection of <paramref name="c"/> with <paramref name="s"/> within their bounds: the first one
    /// (<paramref name="wc"/> &lt; -1e100), the last one (&gt; 1e100), or the closest to <paramref name="wc"/>; its
    /// parameters on the surface and the curve.
    /// </summary>
    internal static bool ChFi3d_IntCS(SurfaceAdaptorBase s, CurveAdaptorBase c, ref Point2D p2dS, ref double wc)
    {
        HInter intersection = new();

        double uf = c.FirstParameter(), ul = c.LastParameter();
        double u1 = s.FirstUParameter(), u2 = s.LastUParameter();
        double v1 = s.FirstVParameter(), v2 = s.LastVParameter();
        CurveSurfaceIntersectionPoint pint;
        intersection.Perform(c, s);
        bool keepfirst = (wc < -1.0e100), keeplast = (wc > 1.0e100);
        double temp = 0.0;
        if (keepfirst)
        {
            temp = 1.0e100;
        }
        if (keeplast)
        {
            temp = -1.0e100;
        }
        double dist = 2.0e100;
        if (intersection.IsDone)
        {
            int nbp = intersection.PointCount, i, isol = 0;
            for (i = 1; i <= nbp; i++)
            {
                pint = intersection.Point(i);
                double up = pint.U();
                double vp = pint.V();
                if (s.IsUPeriodic)
                {
                    up = ChFi3d_InPeriod(up, u1, u1 + s.UPeriod(), 1.0e-8);
                }
                if (s.IsVPeriodic)
                {
                    vp = ChFi3d_InPeriod(vp, v1, v1 + s.VPeriod(), 1.0e-8);
                }
                if (uf <= pint.W() && ul >= pint.W() && u1 <= up && u2 >= up && v1 <= vp && v2 >= vp)
                {
                    if (keepfirst && pint.W() < temp)
                    {
                        temp = pint.W();
                        isol = i;
                    }
                    else if (keeplast && pint.W() > temp)
                    {
                        temp = pint.W();
                        isol = i;
                    }
                    else if (Math.Abs(pint.W() - wc) < dist)
                    {
                        dist = Math.Abs(pint.W() - wc);
                        isol = i;
                    }
                }
            }
            if (isol == 0)
            {
                return false;
            }
            pint = intersection.Point(isol);
            double up2 = pint.U();
            double vp2 = pint.V();
            if (s.IsUPeriodic)
            {
                up2 = ChFi3d_InPeriod(up2, u1, u1 + s.UPeriod(), 1.0e-8);
            }
            if (s.IsVPeriodic)
            {
                vp2 = ChFi3d_InPeriod(vp2, v1, v1 + s.VPeriod(), 1.0e-8);
            }
            p2dS = p2dS.WithCoord(up2, vp2);
            wc = pint.W();
            return true;
        }
        return false;
    }

    //=======================================================================
    // function : ComputesIntPC
    // purpose  : Intersection of two PCurves of type FaceInterference
    //           the parameters of the pcurves at the solution point are
    //           UInt1,UInt2
    //=======================================================================

    /// <summary>Refines the crossing of the two traces on the fillets from the parameters
    /// <paramref name="uInt1"/> and <paramref name="uInt2"/>.</summary>
    internal static void ChFi3d_ComputesIntPC(in FaceInterference fi1, in FaceInterference fi2,
                                              SurfaceAdaptor hs1, SurfaceAdaptor hs2, ref double uInt1,
                                              ref double uInt2)
    {
        Point3D bid = new();
        ChFi3d_ComputesIntPC(fi1, fi2, hs1, hs2, ref uInt1, ref uInt2, ref bid);
    }

    //=================================================================================================

    /// <summary>Refines the crossing of the two traces on the fillets from the parameters
    /// <paramref name="uInt1"/> and <paramref name="uInt2"/>; <paramref name="p"/> the point between them.</summary>
    internal static void ChFi3d_ComputesIntPC(in FaceInterference fi1, in FaceInterference fi2,
                                              SurfaceAdaptor hs1, SurfaceAdaptor hs2, ref double uInt1,
                                              ref double uInt2, ref Point3D p)
    {
        // Only one intersection to be carried out, however, the effort
        // is taken to check the extremities by an extrema c3d/c3d
        // created on pcurveonsurf of fillets.

        double x, y, distref2;
        fi1.PCurveOnSurf()!.Value(uInt1).Coord(out x, out y);
        Point3D p3d1 = hs1.Value(x, y);
        fi2.PCurveOnSurf()!.Value(uInt2).Coord(out x, out y);
        Point3D p3d2 = hs2.Value(x, y);
        distref2 = p3d1.SquareDistanceTo(p3d2);
        p = p.WithXYZ(p3d1.XYZ().Added(p3d2.XYZ()).Multiplied(0.5));
        // recalculation of the extremums
        double delt1 = Math.Min(0.1, 0.05 * (fi1.LastParameter() - fi1.FirstParameter()));
        CurveAdaptor2D hc2d1 = new(fi1.PCurveOnSurf()!, uInt1 - delt1, uInt1 + delt1);
        CurveOnSurface cons1 = new(hc2d1, hs1);
        double delt2 = Math.Min(0.1, 0.05 * (fi2.LastParameter() - fi2.FirstParameter()));
        CurveAdaptor2D hc2d2 = new(fi2.PCurveOnSurf()!, uInt2 - delt2, uInt2 + delt2);
        CurveOnSurface cons2 = new(hc2d2, hs2);
        LocateExtCC ext = new(cons1, cons2, uInt1, uInt2);
        if (ext.IsDone)
        {
            double dist2 = ext.SquareDistance();
            if (dist2 < distref2)
            {
                ext.Point(out POnCurv ponc1, out POnCurv ponc2);
                uInt1 = ponc1.Parameter();
                uInt2 = ponc2.Parameter();
                Point3D pnt1 = ponc1.Value();
                Point3D pnt2 = ponc2.Value();
                p = p.WithXYZ(pnt1.XYZ().Added(pnt2.XYZ()).Multiplied(0.5));
            }
        }
    }

    //=======================================================================
    // function : BoundSurf
    // purpose  : computes a GeomAdaptor_Surface from the surface of the
    //           SurfData Fd1 and trims it to allow the intersection computation
    //=======================================================================

    /// <summary>The adaptor of the fillet surface of <paramref name="fd1"/>, bounded around its traces on the sides
    /// <paramref name="iFaCo1"/> and <paramref name="iFaArc1"/> (unbounded when either is 0).</summary>
    internal static SurfaceAdaptor ChFi3d_BoundSurf(DsDataStructure dStr, SurfData fd1,
                                                         int iFaCo1, int iFaArc1)
    {
        // rmq : as in fact 2 interferences of Fd1 serve only to set limits
        //       indexes IFaCo1 and IFaArc1 are not useful.
        //       They are preserver here as an option in case it will be necessary to set
        //       more restrictive limits (with intersection points as additional argument).

        SurfaceAdaptor hs1 = new();
        SurfaceAdaptor s1 = hs1;
        s1.Load(dStr.Surface(fd1.Surf()).Surface()!);

        if ((iFaCo1 == 0) || (iFaArc1 == 0))
        {
            return hs1;
        }

        FaceInterference fiCo1 = fd1.Interference(iFaCo1);
        FaceInterference fiArc1 = fd1.Interference(iFaArc1);

        double du, dv, mu, Mu, mv, Mv;
        Point2D uVf1, uVf2, uVl1, uVl2;

        uVf1 = fiCo1.PCurveOnSurf()!.Value(fiCo1.FirstParameter());
        uVl1 = fiCo1.PCurveOnSurf()!.Value(fiCo1.LastParameter());
        uVf2 = fiArc1.PCurveOnSurf()!.Value(fiArc1.FirstParameter());
        uVl2 = fiArc1.PCurveOnSurf()!.Value(fiArc1.LastParameter());
        ChFi3d_Boite(uVf1, uVf2, uVl1, uVl2, out du, out dv, out mu, out Mu, out mv, out Mv);
        SurfaceKind styp = s1.GetType();
        if (styp == SurfaceKind.Cylinder)
        {
            dv = Math.Max(0.5 * dv, 4.0 * s1.Cylinder().Radius);
            du = 0.0;
            s1.Load(dStr.Surface(fd1.Surf()).Surface()!, mu, Mu, mv - dv, Mv + dv);
        }
        // In the case of a torus or cone, it is not necessary that the bounds create a surface with
        // period more than 2PI.
        else if (styp == SurfaceKind.Torus || styp == SurfaceKind.Cone)
        {
            du = Math.Min(Math.PI - 0.5 * du, 0.1 * du);
            dv = 0.0;
            s1.Load(dStr.Surface(fd1.Surf()).Surface()!, mu - du, Mu + du, mv, Mv);
        }
        else if (styp == Plane)
        {
            du = Math.Max(0.5 * du, 4.0 * dv);
            dv = 0.0;
            s1.Load(dStr.Surface(fd1.Surf()).Surface()!, mu - du, Mu + du, mv, Mv);
        }
        return hs1;
    }

    //=================================================================================================

    /// <summary>The pivot of a corner of three fillets: the one whose sections do not cross on the opposite face;
    /// -1 if none.</summary>
    internal static int ChFi3d_SearchPivot(int[] s, double[,] u, double t)
    {
        //           This function finds as pivot a cd the sections which of
        //           do not cross on the opposite face.
        //         - probably there will be cases asymmetric to the point that
        //           none of tree fillets will match! To be SEEN.
        //         - in case when several fillets match the
        //           first one taken is not inevitably the best
        //           it should be refined by comparing the parameters on
        //           guide lines and (/or) radiuses.

        bool bondeb, bonfin;
        for (int i = 0; i <= 2; i++)
        {
            if (s[(i + 1) % 3] == 1)
            {
                bondeb = (u[(i + 1) % 3, i] - u[(i + 1) % 3, (i + 2) % 3] >= -t);
            }
            else
            {
                bondeb = (u[(i + 1) % 3, i] - u[(i + 1) % 3, (i + 2) % 3] <= t);
            }
            if (s[(i + 2) % 3] == 1)
            {
                bonfin = (u[(i + 2) % 3, i] - u[(i + 2) % 3, (i + 1) % 3] >= -t);
            }
            else
            {
                bonfin = (u[(i + 2) % 3, i] - u[(i + 2) % 3, (i + 1) % 3] <= t);
            }
            if (bondeb && bonfin)
            {
                return i;
            }
        }
        return -1;
    }

    //=================================================================================================

    /// <summary>
    /// Searches from the surface data <paramref name="ind1"/> and <paramref name="ind2"/>, moving by the senses,
    /// for a pair of the two stripes' surface data in front of each other (<see cref="ChFi3d_IsInFront"/>).
    /// </summary>
    internal static bool ChFi3d_SearchFD(DsDataStructure dStr, Stripe cd1, Stripe cd2,
                                         int sens1, int sens2, ref int i1, ref int i2, ref double p1, ref double p2,
                                         int ind1, int ind2, ref Face face, ref bool sameside, ref int jf1,
                                         ref int jf2)
    {
        bool found = false;
        int id1 = ind1, id2 = ind2;
        int if1 = ind1, if2 = ind2;
        int l1 = cd1.SetOfSurfData()!.Length();
        int l2 = cd2.SetOfSurfData()!.Length();
        int i;
        bool fini1 = false, fini2 = false;
        bool visavis = false, visavisok = false;
        Vertex vtx = new();
        while (!found)
        {
            for (i = id1; (i * sens1) <= (if1 * sens1) && !found && !fini2; i = i + sens1)
            {
                if (ChFi3d_IsInFront(dStr, cd1, cd2, i, if2, sens1, sens2, ref p1, ref p2, ref face, ref sameside,
                                     ref jf1, ref jf2, ref visavis, vtx, false, false))
                {
                    i1 = i;
                    i2 = if2;
                    found = true;
                }
                else if (visavis && !visavisok)
                {
                    visavisok = true;
                    i1 = i;
                    i2 = if2;
                }
            }
            if (!fini1)
            {
                if1 = if1 + sens1;
                if (if1 < 1 || if1 > l1)
                {
                    if1 = if1 - sens1;
                    fini1 = true;
                }
            }

            for (i = id2; (i * sens2) <= (if2 * sens2) && !found && !fini1; i = i + sens2)
            {
                if (ChFi3d_IsInFront(dStr, cd1, cd2, if1, i, sens1, sens2, ref p1, ref p2, ref face, ref sameside,
                                     ref jf1, ref jf2, ref visavis, vtx, false, false))
                {
                    i1 = if1;
                    i2 = i;
                    found = true;
                }
                else if (visavis && !visavisok)
                {
                    visavisok = true;
                    i1 = if1;
                    i2 = i;
                }
            }
            if (!fini2)
            {
                if2 = if2 + sens2;
                if (if2 < 1 || if2 > l2)
                {
                    if2 = if2 - sens2;
                    fini2 = true;
                }
            }
            if (fini1 && fini2)
            {
                break;
            }
        }
        return found;
    }

    //=======================================================================
    // function : Parameters
    // purpose  : compute the parameters <u> and <v> of the 3d point <p3d>
    //           on the surface <S> if it's an analytic surface
    //=======================================================================

    /// <summary>The parameters of <paramref name="p3d"/> on <paramref name="s"/>: analytic for the elementary
    /// surfaces, else by projection.</summary>
    /// <exception cref="NotDoneException">The projection does not give exactly one point.</exception>
    internal static void ChFi3d_Parameters(Surface s, in Point3D p3d, out double u, out double v)
    {
        SurfaceAdaptor gas = new(s);
        switch (gas.GetType())
        {
            case Plane:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(gas.Plane(), p3d, out u, out v);
                break;
            case SurfaceKind.Cylinder:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(gas.Cylinder(), p3d, out u, out v);
                break;
            case SurfaceKind.Cone:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(gas.Cone(), p3d, out u, out v);
                break;
            case SurfaceKind.Sphere:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(gas.Sphere(), p3d, out u, out v);
                break;
            case SurfaceKind.Torus:
                global::Kerf.Engine.Geometry.ElSLib.Parameters(gas.Torus(), p3d, out u, out v);
                break;
            case SurfaceKind.BezierSurface:
            case SurfaceKind.BSplineSurface:
            default:
            {
                ProjectPointOnSurface tool = new(p3d, s);
                if (tool.NbPoints() != 1)
                {
                    throw new NotDoneException("ChFi3d_Parameters() - no projection results");
                }
                else
                {
                    tool.Parameters(1, out u, out v);
                }
            }
                break;
        }
    }

    //=======================================================================
    // function : TrimCurve
    // purpose  : trims the curve <gc> between the points <FirstP> and
    //           <LastP>. The trimmed curve is <gtc>
    //=======================================================================

    /// <summary><paramref name="gc"/> trimmed between the parameters of <paramref name="firstP"/> and
    /// <paramref name="lastP"/>.</summary>
    /// <exception cref="NotDoneException">A projection does not give exactly one point.</exception>
    internal static void ChFi3d_TrimCurve(Curve3D gc, in Point3D firstP, in Point3D lastP,
                                          out TrimmedCurve3D gtc)
    {
        double uf = 0.0, ul = 0.0;
        CurveAdaptor gac = new(gc);
        switch (gac.GetType())
        {
            case Line:
            {
                uf = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Line(), firstP);
                ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Line(), lastP);
            }
                break;
            case Circle:
            {
                uf = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Circle(), firstP);
                ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Circle(), lastP);
            }
                break;
            case Ellipse:
            {
                uf = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Ellipse(), firstP);
                ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Ellipse(), lastP);
            }
                break;
            case Hyperbola:
            {
                uf = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Hyperbola(), firstP);
                ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Hyperbola(), lastP);
            }
                break;
            case Parabola:
            {
                uf = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Parabola(), firstP);
                ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Parabola(), lastP);
            }
                break;
            default:
            {
                ProjectPointOnCurve tool = new(firstP, gc);
                if (tool.NbPoints() != 1)
                {
                    throw new NotDoneException("ChFi3d_TrimCurve() - no projection results for the first point");
                }
                else
                {
                    uf = tool.Parameter(1);
                }
                tool.Init(lastP, gc);
                if (tool.NbPoints() != 1)
                {
                    throw new NotDoneException("ChFi3d_TrimCurve() - no projection results for the second point");
                }
                else
                {
                    ul = tool.Parameter(1);
                }
            }
                break;
        }
        gtc = new TrimmedCurve3D(gc, uf, ul);
    }

    //=================================================================================================

    private static bool GoodExt(Curve3D c, in Vector3D v, double f, double l, double a)
    {
        for (int i = 0; i < 6; i++)
        {
            double t = i * 0.2;
            c.D1(((1 - t) * f + t * l), out Point3D d0, out Vector3D d1);
            double ang = d1.AngleTo(v);
            double angref = a * t + 0.002;
            if (ang > angref)
            {
                return false;
            }
        }
        return true;
    }
}
