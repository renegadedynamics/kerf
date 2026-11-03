// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_IntCS to GoodExt)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

internal static partial class ChFi3d_Builder_0
{
    //=======================================================================
    // function : IntCS
    // purpose  : Fast calculation of the intersection curve surface.
    //
    //=======================================================================

    /// <summary>
    /// The intersection of <paramref name="C"/> with <paramref name="S"/> within their bounds: the first one
    /// (<paramref name="wc"/> &lt; -1e100), the last one (&gt; 1e100), or the closest to <paramref name="wc"/>; its
    /// parameters on the surface and the curve.
    /// </summary>
    internal static bool ChFi3d_IntCS(Adaptor3d_Surface S, Adaptor3d_Curve C, ref Point2D p2dS, ref double wc)
    {
        IntCurveSurface_HInter Intersection = new();

        double uf = C.FirstParameter(), ul = C.LastParameter();
        double u1 = S.FirstUParameter(), u2 = S.LastUParameter();
        double v1 = S.FirstVParameter(), v2 = S.LastVParameter();
        IntCurveSurface_IntersectionPoint pint;
        Intersection.Perform(C, S);
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
        if (Intersection.IsDone())
        {
            int nbp = Intersection.NbPoints(), i, isol = 0;
            for (i = 1; i <= nbp; i++)
            {
                pint = Intersection.Point(i);
                double up = pint.U();
                double vp = pint.V();
                if (S.IsUPeriodic())
                {
                    up = ChFi3d_InPeriod(up, u1, u1 + S.UPeriod(), 1.0e-8);
                }
                if (S.IsVPeriodic())
                {
                    vp = ChFi3d_InPeriod(vp, v1, v1 + S.VPeriod(), 1.0e-8);
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
            pint = Intersection.Point(isol);
            double up2 = pint.U();
            double vp2 = pint.V();
            if (S.IsUPeriodic())
            {
                up2 = ChFi3d_InPeriod(up2, u1, u1 + S.UPeriod(), 1.0e-8);
            }
            if (S.IsVPeriodic())
            {
                vp2 = ChFi3d_InPeriod(vp2, v1, v1 + S.VPeriod(), 1.0e-8);
            }
            p2dS.SetCoord(up2, vp2);
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
    /// <paramref name="UInt1"/> and <paramref name="UInt2"/>.</summary>
    internal static void ChFi3d_ComputesIntPC(in ChFiDS_FaceInterference Fi1, in ChFiDS_FaceInterference Fi2,
                                              GeomAdaptor_Surface HS1, GeomAdaptor_Surface HS2, ref double UInt1,
                                              ref double UInt2)
    {
        Point3D bid = new();
        ChFi3d_ComputesIntPC(Fi1, Fi2, HS1, HS2, ref UInt1, ref UInt2, ref bid);
    }

    //=================================================================================================

    /// <summary>Refines the crossing of the two traces on the fillets from the parameters
    /// <paramref name="UInt1"/> and <paramref name="UInt2"/>; <paramref name="P"/> the point between them.</summary>
    internal static void ChFi3d_ComputesIntPC(in ChFiDS_FaceInterference Fi1, in ChFiDS_FaceInterference Fi2,
                                              GeomAdaptor_Surface HS1, GeomAdaptor_Surface HS2, ref double UInt1,
                                              ref double UInt2, ref Point3D P)
    {
        // Only one intersection to be carried out, however, the effort
        // is taken to check the extremities by an extrema c3d/c3d
        // created on pcurveonsurf of fillets.

        double x, y, distref2;
        Fi1.PCurveOnSurf()!.Value(UInt1).Coord(out x, out y);
        Point3D p3d1 = HS1.Value(x, y);
        Fi2.PCurveOnSurf()!.Value(UInt2).Coord(out x, out y);
        Point3D p3d2 = HS2.Value(x, y);
        distref2 = p3d1.SquareDistance(p3d2);
        P.SetXYZ(p3d1.XYZ().Added(p3d2.XYZ()).Multiplied(0.5));
        // recalculation of the extremums
        double delt1 = Math.Min(0.1, 0.05 * (Fi1.LastParameter() - Fi1.FirstParameter()));
        Geom2dAdaptor_Curve hc2d1 = new(Fi1.PCurveOnSurf()!, UInt1 - delt1, UInt1 + delt1);
        Adaptor3d_CurveOnSurface cons1 = new(hc2d1, HS1);
        double delt2 = Math.Min(0.1, 0.05 * (Fi2.LastParameter() - Fi2.FirstParameter()));
        Geom2dAdaptor_Curve hc2d2 = new(Fi2.PCurveOnSurf()!, UInt2 - delt2, UInt2 + delt2);
        Adaptor3d_CurveOnSurface cons2 = new(hc2d2, HS2);
        Extrema_LocateExtCC ext = new(cons1, cons2, UInt1, UInt2);
        if (ext.IsDone())
        {
            double dist2 = ext.SquareDistance();
            if (dist2 < distref2)
            {
                ext.Point(out Extrema_POnCurv ponc1, out Extrema_POnCurv ponc2);
                UInt1 = ponc1.Parameter();
                UInt2 = ponc2.Parameter();
                Point3D Pnt1 = ponc1.Value();
                Point3D Pnt2 = ponc2.Value();
                P.SetXYZ(Pnt1.XYZ().Added(Pnt2.XYZ()).Multiplied(0.5));
            }
        }
    }

    //=======================================================================
    // function : BoundSurf
    // purpose  : computes a GeomAdaptor_Surface from the surface of the
    //           SurfData Fd1 and trims it to allow the intersection computation
    //=======================================================================

    /// <summary>The adaptor of the fillet surface of <paramref name="Fd1"/>, bounded around its traces on the sides
    /// <paramref name="IFaCo1"/> and <paramref name="IFaArc1"/> (unbounded when either is 0).</summary>
    internal static GeomAdaptor_Surface ChFi3d_BoundSurf(TopOpeBRepDS_DataStructure DStr, ChFiDS_SurfData Fd1,
                                                         int IFaCo1, int IFaArc1)
    {
        // rmq : as in fact 2 interferences of Fd1 serve only to set limits
        //       indexes IFaCo1 and IFaArc1 are not useful.
        //       They are preserver here as an option in case it will be necessary to set
        //       more restrictive limits (with intersection points as additional argument).

        GeomAdaptor_Surface HS1 = new();
        GeomAdaptor_Surface S1 = HS1;
        S1.Load(DStr.Surface(Fd1.Surf()).Surface()!);

        if ((IFaCo1 == 0) || (IFaArc1 == 0))
        {
            return HS1;
        }

        ChFiDS_FaceInterference FiCo1 = Fd1.Interference(IFaCo1);
        ChFiDS_FaceInterference FiArc1 = Fd1.Interference(IFaArc1);

        double Du, Dv, mu, Mu, mv, Mv;
        Point2D UVf1, UVf2, UVl1, UVl2;

        UVf1 = FiCo1.PCurveOnSurf()!.Value(FiCo1.FirstParameter());
        UVl1 = FiCo1.PCurveOnSurf()!.Value(FiCo1.LastParameter());
        UVf2 = FiArc1.PCurveOnSurf()!.Value(FiArc1.FirstParameter());
        UVl2 = FiArc1.PCurveOnSurf()!.Value(FiArc1.LastParameter());
        ChFi3d_Boite(UVf1, UVf2, UVl1, UVl2, out Du, out Dv, out mu, out Mu, out mv, out Mv);
        SurfaceKind styp = S1.GetType();
        if (styp == SurfaceKind.Cylinder)
        {
            Dv = Math.Max(0.5 * Dv, 4.0 * S1.Cylinder().Radius());
            Du = 0.0;
            S1.Load(DStr.Surface(Fd1.Surf()).Surface()!, mu, Mu, mv - Dv, Mv + Dv);
        }
        // In the case of a torus or cone, it is not necessary that the bounds create a surface with
        // period more than 2PI.
        else if (styp == SurfaceKind.Torus || styp == SurfaceKind.Cone)
        {
            Du = Math.Min(Math.PI - 0.5 * Du, 0.1 * Du);
            Dv = 0.0;
            S1.Load(DStr.Surface(Fd1.Surf()).Surface()!, mu - Du, Mu + Du, mv, Mv);
        }
        else if (styp == Plane)
        {
            Du = Math.Max(0.5 * Du, 4.0 * Dv);
            Dv = 0.0;
            S1.Load(DStr.Surface(Fd1.Surf()).Surface()!, mu - Du, Mu + Du, mv, Mv);
        }
        return HS1;
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
    internal static bool ChFi3d_SearchFD(TopOpeBRepDS_DataStructure DStr, ChFiDS_Stripe cd1, ChFiDS_Stripe cd2,
                                         int sens1, int sens2, ref int i1, ref int i2, ref double p1, ref double p2,
                                         int ind1, int ind2, ref TopoDS_Face face, ref bool sameside, ref int jf1,
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
        TopoDS_Vertex Vtx = new();
        while (!found)
        {
            for (i = id1; (i * sens1) <= (if1 * sens1) && !found && !fini2; i = i + sens1)
            {
                if (ChFi3d_IsInFront(DStr, cd1, cd2, i, if2, sens1, sens2, ref p1, ref p2, ref face, ref sameside,
                                     ref jf1, ref jf2, ref visavis, Vtx, false, false))
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
                if (ChFi3d_IsInFront(DStr, cd1, cd2, if1, i, sens1, sens2, ref p1, ref p2, ref face, ref sameside,
                                     ref jf1, ref jf2, ref visavis, Vtx, false, false))
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

    /// <summary>The parameters of <paramref name="p3d"/> on <paramref name="S"/>: analytic for the elementary
    /// surfaces, else by projection.</summary>
    /// <exception cref="NotDoneException">The projection does not give exactly one point.</exception>
    internal static void ChFi3d_Parameters(Geom_Surface S, in Point3D p3d, out double u, out double v)
    {
        GeomAdaptor_Surface gas = new(S);
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
            case BezierSurface:
            case BSplineSurface:
            default:
            {
                GeomAPI_ProjectPointOnSurf tool = new(p3d, S);
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

    /// <summary><paramref name="gc"/> trimmed between the parameters of <paramref name="FirstP"/> and
    /// <paramref name="LastP"/>.</summary>
    /// <exception cref="NotDoneException">A projection does not give exactly one point.</exception>
    internal static void ChFi3d_TrimCurve(Geom_Curve gc, in Point3D FirstP, in Point3D LastP,
                                          out Geom_TrimmedCurve gtc)
    {
        double uf = 0.0, ul = 0.0;
        GeomAdaptor_Curve gac = new(gc);
        switch (gac.GetType())
        {
            case Line:
            {
                uf = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Line(), FirstP);
                ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Line(), LastP);
            }
                break;
            case Circle:
            {
                uf = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Circle(), FirstP);
                ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Circle(), LastP);
            }
                break;
            case Ellipse:
            {
                uf = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Ellipse(), FirstP);
                ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Ellipse(), LastP);
            }
                break;
            case Hyperbola:
            {
                uf = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Hyperbola(), FirstP);
                ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Hyperbola(), LastP);
            }
                break;
            case Parabola:
            {
                uf = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Parabola(), FirstP);
                ul = global::Kerf.Engine.Geometry.ElCLib.Parameter(gac.Parabola(), LastP);
            }
                break;
            default:
            {
                GeomAPI_ProjectPointOnCurve tool = new(FirstP, gc);
                if (tool.NbPoints() != 1)
                {
                    throw new NotDoneException("ChFi3d_TrimCurve() - no projection results for the first point");
                }
                else
                {
                    uf = tool.Parameter(1);
                }
                tool.Init(LastP, gc);
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
        gtc = new Geom_TrimmedCurve(gc, uf, ul);
    }

    //=================================================================================================

    private static bool GoodExt(Geom_Curve C, in Vector3D V, double f, double l, double a)
    {
        for (int i = 0; i < 6; i++)
        {
            double t = i * 0.2;
            C.D1(((1 - t) * f + t * l), out Point3D d0, out Vector3D d1);
            double ang = d1.Angle(V);
            double angref = a * t + 0.002;
            if (ang > angref)
            {
                return false;
            }
        }
        return true;
    }
}
