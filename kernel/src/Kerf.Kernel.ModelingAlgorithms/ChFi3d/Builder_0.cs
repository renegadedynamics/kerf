// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (to ChFi3d_IsInFront; the rest in the other
//          ChFi3d_Builder_0.*.cs files)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Fillets.Data.State;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets;

/// <summary>The fillet builders' free functions (<c>ChFi3d_*</c>), as a static class named after the file.</summary>
/// <remarks>
/// The whole of <c>ChFi3d_Builder_0.cxx</c> (unit 7N1; <c>ChFi3d_Coefficient</c> came first with unit 7E). Its
/// <c>OCCT_DEBUG</c> and <c>DRAW</c> blocks (traces, chronometers, the <c>ChFi3d_Settrace*</c> flags) are left out.
/// The functions write only into the builder's own data: the data structure (<see cref="DsDataStructure"/>),
/// the stripes, surface data and spines, and the geometry they make. None writes into a shape of the input: the
/// vertices whose tolerance the builder raises are updated by the builder itself (<c>ChFi3d_Builder</c>, unit 7N2),
/// not here (decision 1087).
/// </remarks>
internal static partial class Builder_0
{
    //=================================================================================================

    /// <summary><paramref name="U"/> brought into [<paramref name="uFirst"/>, <paramref name="uLast"/>], up to
    /// <paramref name="eps"/>.</summary>
    internal static double ChFi3d_InPeriod(double U, double uFirst, double uLast, double eps)
    {
        double period = uLast - uFirst;
        double u = U;
        while (eps < (uFirst - u))
        {
            u += period;
        }
        while (eps > (uLast - u))
        {
            u -= period;
        }
        if (u < uFirst)
        {
            u = uFirst;
        }
        return u;
    }

    // namespace { ChFi3d_BaseSurface, ChFi3d_ApplyBounds, ChFi3d_BoundSrfImpl }: the template's two instances are
    // written out through the adaptor's own surface and loading.

    private static Surface? ChFi3d_BaseSurface(SurfaceAdaptor surface) => surface.Surface();

    private static Surface? ChFi3d_BaseSurface(FaceSurfaceAdaptor surface) =>
        surface.GeomSurfaceOriginal();

    private static void ChFi3d_ApplyBounds(SurfaceAdaptor surface, Surface geomSurface,
                                           double uFirst, double uLast, double vFirst, double vLast)
    {
        surface.Load(geomSurface, uFirst, uLast, vFirst, vLast, surface.ToleranceU(),
                        surface.ToleranceV());
    }

    private static void ChFi3d_ApplyBounds(FaceSurfaceAdaptor surface, Surface geomSurface,
                                           double uFirst, double uLast, double vFirst, double vLast)
    {
        surface.Load(geomSurface, uFirst, uLast, vFirst, vLast, surface.Trsf(),
                        surface.ToleranceU(), surface.ToleranceV());
    }

    private static void ChFi3d_BoundSrfImpl(SurfaceAdaptorBase surface, Surface? theBase, double umin,
                                            double umax, double vmin, double vmax,
                                            bool checkNaturalBounds, out Surface aSurface,
                                            out double aUU1, out double aUU2, out double aVV1, out double aVV2)
    {
        double aUmin = umin;
        double aUmax = umax;
        double aVmin = vmin;
        double aVmax = vmax;
        aSurface = theBase!;
        if (aSurface is TrimmedSurface trimmedSurface)
        {
            aSurface = trimmedSurface.BasisSurface;
        }

        aSurface.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        double periodU = 0.0;
        double periodV = 0.0;
        if (aSurface.IsUPeriodic)
        {
            periodU = aSurface.UPeriod;
        }
        if (aSurface.IsVPeriodic)
        {
            periodV = aSurface.VPeriod;
        }

        double stepU = aUmax - aUmin;
        double stepV = aVmax - aVmin;
        double scaleU = surface.UResolution(1.0);
        double scaleV = surface.VResolution(1.0);
        double step3dU = stepU / scaleU;
        double step3dV = stepV / scaleV;

        if (step3dU > step3dV)
        {
            stepV = step3dU * scaleV;
        }
        if (step3dV > step3dU)
        {
            stepU = step3dV * scaleU;
        }
        if (periodU > 0.0)
        {
            stepU = 0.1 * (periodU - (aUmax - aUmin));
        }
        if (periodV > 0.0)
        {
            stepV = 0.1 * (periodV - (aVmax - aVmin));
        }

        aUU1 = aUmin - stepU;
        aUU2 = aUmax + stepU;
        aVV1 = aVmin - stepV;
        aVV2 = aVmax + stepV;
        if (checkNaturalBounds)
        {
            if (!surface.IsUPeriodic)
            {
                aUU1 = Math.Max(aUU1, u1);
                aUU2 = Math.Min(aUU2, u2);
            }
            if (!surface.IsVPeriodic)
            {
                aVV1 = Math.Max(aVV1, v1);
                aVV2 = Math.Min(aVV2, v2);
            }
        }
    }

    //=======================================================================
    // function : Box
    // purpose  : Calculation of min/max uv of the fillet to intersect.
    //=======================================================================

    /// <summary>The bounds in u and v of the two points.</summary>
    internal static void ChFi3d_Boite(in Point2D p1, in Point2D p2, out double mu, out double Mu, out double mv,
                                      out double Mv)
    {
        mu = Math.Min(p1.X, p2.X);
        Mu = Math.Max(p1.X, p2.X);
        mv = Math.Min(p1.Y, p2.Y);
        Mv = Math.Max(p1.Y, p2.Y);
    }

    //=======================================================================
    // function : Box
    // purpose  : Calculation of min/max uv of the fillet to intersect.
    //=======================================================================

    /// <summary>The bounds in u and v of the four points, and their spans.</summary>
    internal static void ChFi3d_Boite(in Point2D p1, in Point2D p2, in Point2D p3, in Point2D p4, out double du,
                                      out double dv, out double mu, out double Mu, out double mv, out double Mv)
    {
        double a, b;
        a = Math.Min(p1.X, p2.X);
        b = Math.Min(p3.X, p4.X);
        mu = Math.Min(a, b);
        a = Math.Max(p1.X, p2.X);
        b = Math.Max(p3.X, p4.X);
        Mu = Math.Max(a, b);
        a = Math.Min(p1.Y, p2.Y);
        b = Math.Min(p3.Y, p4.Y);
        mv = Math.Min(a, b);
        a = Math.Max(p1.Y, p2.Y);
        b = Math.Max(p3.Y, p4.Y);
        Mv = Math.Max(a, b);
        du = Mu - mu;
        dv = Mv - mv;
    }

    //=================================================================================================

    private static SurfaceAdaptorBase? Geometry(DsDataStructure dStr, int ind)
    {
        if (ind == 0)
        {
            return null;
        }
        if (ind > 0)
        {
            Face f = ShapeCasts.Face(dStr.Shape(ind));
            if (f.IsNull)
            {
                return null;
            }
            FaceSurfaceAdaptor hs = new();
            hs.Initialize(f, false);
            return hs;
        }
        else
        {
            Surface? s = dStr.Surface(-ind).Surface();
            if (s is null)
            {
                return null;
            }
            return new SurfaceAdaptor(s);
        }
    }

    //=================================================================================================

    /// <summary>Sets the tolerance of the point <paramref name="ip"/> to half again the diagonal of
    /// <paramref name="box"/>.</summary>
    internal static void ChFi3d_SetPointTolerance(DsDataStructure dStr, in Box box, int ip)
    {
        double a, b, c, d, e, f, vtol;
        box.Get(out a, out b, out c, out d, out e, out f);
        d -= a;
        e -= b;
        f -= c;
        d *= d;
        e *= e;
        f *= f;
        vtol = Math.Sqrt(d + e + f) * 1.5; // on prend un petit rab.
        dStr.ChangePoint(ip).Tolerance(vtol);
    }

    //=================================================================================================

    /// <summary>Adds the points of <paramref name="c"/> at <paramref name="wd"/> and <paramref name="wf"/> to the
    /// boxes.</summary>
    internal static void ChFi3d_EnlargeBox(Curve3D c, double wd, double wf, ref Box box1, ref Box box2)
    {
        box1.Add(c.Value(wd));
        box2.Add(c.Value(wf));
    }

    //=================================================================================================

    /// <summary>Adds the points of <paramref name="pc"/> on <paramref name="s"/> at <paramref name="wd"/> and
    /// <paramref name="wf"/> to the boxes.</summary>
    internal static void ChFi3d_EnlargeBox(SurfaceAdaptorBase s, Curve2D pc, double wd, double wf,
                                           ref Box box1, ref Box box2)
    {
        double u, v;
        pc.Value(wd).Coord(out u, out v);
        box1.Add(s.Value(u, v));
        pc.Value(wf).Coord(out u, out v);
        box2.Add(s.Value(u, v));
    }

    //=================================================================================================

    /// <summary>Adds the point of <paramref name="e"/> at <paramref name="w"/>, by its 3D curve and its pcurves on
    /// the faces <paramref name="lf"/>, to the box.</summary>
    internal static void ChFi3d_EnlargeBox(in Edge e, NCollection_List<Shape> lf, double w,
                                           ref Box box)
    {
        EdgeCurveAdaptor bc = new(e);
        box.Add(bc.Value(w));
        foreach (Shape it in lf)
        {
            Face f = ShapeCasts.Face(it);
            if (!f.IsNull)
            {
                bc.Initialize(e, f);
                box.Add(bc.Value(w));
            }
        }
    }

    //=================================================================================================

    /// <summary>Adds the extremities of the surface data <paramref name="sd"/> (and of the stripe's end curve) to the
    /// boxes of its two sides.</summary>
    internal static void ChFi3d_EnlargeBox(DsDataStructure dStr, Stripe? st, SurfData sd,
                                           ref Box b1, ref Box b2, bool isfirst)
    {
        double u, v;
        CommonPoint cp1 = sd.Vertex(isfirst, 1);
        CommonPoint cp2 = sd.Vertex(isfirst, 2);
        b1.Add(cp1.Point());
        b2.Add(cp2.Point());
        FaceInterference fi1 = sd.InterferenceOnS1();
        FaceInterference fi2 = sd.InterferenceOnS2();
        Surface s = dStr.Surface(sd.Surf()).Surface()!;
        Curve2D? pcs1 = fi1.PCurveOnSurf();
        Curve2D? pcs2 = fi2.PCurveOnSurf();
        Curve3D? c3d1 = dStr.Curve(fi1.LineIndex()).Curve();
        Curve3D? c3d2 = dStr.Curve(fi2.LineIndex()).Curve();
        SurfaceAdaptorBase? f1 = Geometry(dStr, sd.IndexOfS1());
        SurfaceAdaptorBase? f2 = Geometry(dStr, sd.IndexOfS2());
        double p1 = fi1.Parameter(isfirst);
        if (c3d1 is not null)
        {
            b1.Add(c3d1.Value(p1));
        }
        if (pcs1 is not null)
        {
            pcs1.Value(p1).Coord(out u, out v);
            b1.Add(s.Value(u, v));
        }
        if (f1 is not null)
        {
            Curve2D? pcf1 = fi1.PCurveOnFace();
            if (pcf1 is not null)
            {
                pcf1.Value(p1).Coord(out u, out v);
                b1.Add(f1.Value(u, v));
            }
        }
        double p2 = fi2.Parameter(isfirst);
        if (c3d2 is not null)
        {
            b2.Add(c3d2.Value(p2));
        }
        if (pcs2 is not null)
        {
            pcs2.Value(p2).Coord(out u, out v);
            b2.Add(s.Value(u, v));
        }
        if (f2 is not null)
        {
            Curve2D? pcf2 = fi2.PCurveOnFace();
            if (pcf2 is not null)
            {
                pcf2.Value(p2).Coord(out u, out v);
                b2.Add(f2.Value(u, v));
            }
        }
        if (st is not null)
        {
            Curve3D? c3d = dStr.Curve(st.Curve(isfirst)).Curve();
            Curve2D? c2d = st.PCurve(isfirst);
            if (st.Orientation(isfirst) == Forward)
            {
                st.Parameters(isfirst, out p1, out p2);
            }
            else
            {
                st.Parameters(isfirst, out p2, out p1);
            }
            if (c3d is not null)
            {
                b1.Add(c3d.Value(p1));
                b2.Add(c3d.Value(p2));
            }
            if (c2d is not null)
            {
                c2d.Value(p1).Coord(out u, out v);
                b1.Add(s.Value(u, v));
                c2d.Value(p2).Coord(out u, out v);
                b2.Add(s.Value(u, v));
            }
        }
    }

    //=================================================================================================

    /// <summary>The faces of <paramref name="efMap"/> on either side of <paramref name="e"/> (the second one null
    /// for a free boundary).</summary>
    internal static void ChFi3d_conexfaces(in Edge e, out Face f1, out Face f2, Map efMap)
    {
        f1 = new Face();
        f2 = new Face();
        foreach (Shape it in efMap[e])
        {
            if (f1.IsNull)
            {
                f1 = ShapeCasts.Face(it);
            }
            else
            {
                f2 = ShapeCasts.Face(it);
                if (!f2.IsSame(f1) || Tool.IsClosed(e, f1))
                {
                    break;
                }
                else
                {
                    f2.Nullify();
                }
            }
        }
    }

    //=======================================================================
    // function : EdgeState
    // purpose  : check concavities for the tops with 3 edges.
    //=======================================================================

    /// <summary>The state of a vertex where the three edges <paramref name="e"/> meet.</summary>
    internal static State ChFi3d_EdgeState(Edge[] e, Map efMap)
    {
        State sst;
        int i, j;
        // TopoDS_Face F[3]; (and the commented-out choice of the three faces)
        ChFi3d_conexfaces(e[0], out Face f1, out Face f2, efMap);
        ChFi3d_conexfaces(e[1], out Face f3, out Face f4, efMap);
        ChFi3d_conexfaces(e[2], out Face f5, out Face f6, efMap);

        // if(F[0].IsNull() || F[1].IsNull() || F[2].IsNull()) sst = ChFiDS_FreeBoundary;
        if (f2.IsNull || f4.IsNull || f6.IsNull)
        {
            sst = State.FreeBoundary;
        }
        else
        {
            Orientation o01, o02, o11, o12, o21, o22;
            i = global::Kerf.Engine.Fillets.ChFi3d.ConcaveSide(new FaceSurfaceAdaptor(f1), new FaceSurfaceAdaptor(f2), e[0], out o01, out o02);
            i = global::Kerf.Engine.Fillets.ChFi3d.ConcaveSide(new FaceSurfaceAdaptor(f3), new FaceSurfaceAdaptor(f4), e[1], out o11, out o12);
            j = global::Kerf.Engine.Fillets.ChFi3d.ConcaveSide(new FaceSurfaceAdaptor(f5), new FaceSurfaceAdaptor(f6), e[2], out o21, out o22);

            if (o01 == o11 && o02 == o21 && o12 == o22)
            {
                sst = State.AllSame;
            }
            else if (o12 == o22 || i == 10 || j == 10)
            {
                sst = State.OnDiff;
            }
            else
            {
                sst = State.OnSame;
            }
        }
        return sst;
    }

    //=======================================================================
    // function : evalconti
    // purpose  : Method very fast to code regularities CN. It is necessary to
    //           refine the processing.
    //=======================================================================

    /// <summary>The continuity across an edge: CN on a seam of a cone, a sphere or a torus, else G1.</summary>
    internal static Continuity ChFi3d_evalconti(in Edge e, in Face f1, in Face f2)
    {
        Continuity cont = G1;
        if (!f1.IsSame(f2))
        {
            return cont;
        }
        Face f = f1;
        f.Orient(Forward);
        FaceSurfaceAdaptor s = new(f, false);
        SurfaceKind typ = s.GetType();
        if (typ != SurfaceKind.Cone && typ != SurfaceKind.Sphere && typ != SurfaceKind.Torus)
        {
            return cont;
        }
        return CN;
    }

    // modified by NIZNHY-PKV Wed Dec 15 11:22:35 2010f
    //=================================================================================================

    /// <summary>Whether the edge <paramref name="ie"/> of the spine is one of ChFiKPart's particular cases between
    /// <paramref name="s1"/> and <paramref name="s2"/>.</summary>
    internal static bool ChFi3d_KParticular(Spine spine, int ie, FaceSurfaceAdaptor s1,
                                            FaceSurfaceAdaptor s2)
    {
        bool bRet;
        //
        bRet = true;
        //
        if (spine is FilSpine fs && !fs.IsConstant(ie))
        {
            return !bRet;
        }
        //
        bool bIsPlane1, bIsPlane2;
        double pa;
        CurveKind ct;
        SurfaceKind st1, st2;
        //
        st1 = s1.GetType();
        st2 = s2.GetType();
        bIsPlane1 = (st1 == Plane);
        bIsPlane2 = (st2 == Plane);
        if (!(bIsPlane1 || bIsPlane2))
        {
            return !bRet;
        }
        //
        FaceSurfaceAdaptor aS1 = (bIsPlane1) ? s1 : s2;
        FaceSurfaceAdaptor aS2 = (bIsPlane1) ? s2 : s1;
        st1 = aS1.GetType();
        st2 = aS2.GetType();
        //
        if (st2 != Plane && st2 != SurfaceKind.Cylinder && st2 != SurfaceKind.Cone)
        {
            return !bRet;
        }
        //
        EdgeCurveAdaptor bc = spine.CurrentElementarySpine(ie);
        ct = bc.GetType();
        if (ct != Line && ct != Circle)
        {
            return !bRet;
        }
        //
        pa = global::Kerf.Engine.Foundation.Precision.Angular();
        //
        if (st2 == Plane)
        {
            if (ct == Line)
            {
                return bRet;
            }
        }
        else if (st2 == SurfaceKind.Cylinder)
        {
            Direction3D d1 = aS1.Plane().Axis.Direction;
            Direction3D d2 = aS2.Cylinder().Axis.Direction;
            //
            if (ct == Line && d1.IsNormal(d2, pa))
            {
                return bRet;
            }
            else if (ct == Circle && d1.IsParallel(d2, pa))
            {
                return bRet;
            }
        }
        else if (st2 == SurfaceKind.Cone)
        {
            Direction3D d1 = aS1.Plane().Axis.Direction;
            Direction3D d2 = aS2.Cone().Axis.Direction;
            if (ct == Circle && d1.IsParallel(d2, pa))
            {
                return bRet;
            }
        }
        return !bRet;
    }

    // modified by NIZNHY-PKV Wed Dec 15 11:22:43 2010t
    //=======================================================================
    // function : BoundFac
    // purpose  : Resize the limits of surface adjacent to the given box
    //            Useful for intersections with known extremities.
    //=======================================================================

    /// <summary>Reloads the face adaptor <paramref name="s"/> on the box widened in proportion to its size (or a
    /// tenth of the rest of the period).</summary>
    internal static void ChFi3d_BoundFac(FaceSurfaceAdaptor s, double uumin, double uumax, double vvmin,
                                         double vvmax, bool checknaturalbounds = true)
    {
        ChFi3d_BoundSrfImpl(s, ChFi3d_BaseSurface(s), uumin, uumax, vvmin, vvmax, checknaturalbounds,
                            out Surface surface, out double uu1, out double uu2, out double vv1,
                            out double vv2);
        ChFi3d_ApplyBounds(s, surface, uu1, uu2, vv1, vv2);
    }

    //=======================================================================
    // function : ChFi3d_BoundSrf
    // purpose  : Resize the limits of surface adjacent to the given box
    //           Useful for intersections with known extremities.
    //=======================================================================

    /// <summary>Reloads the surface adaptor <paramref name="s"/> on the box widened in proportion to its size (or a
    /// tenth of the rest of the period).</summary>
    internal static void ChFi3d_BoundSrf(SurfaceAdaptor s, double uumin, double uumax, double vvmin,
                                         double vvmax, bool checknaturalbounds = true)
    {
        ChFi3d_BoundSrfImpl(s, ChFi3d_BaseSurface(s), uumin, uumax, vvmin, vvmax, checknaturalbounds,
                            out Surface surface, out double uu1, out double uu2, out double vv1,
                            out double vv2);
        ChFi3d_ApplyBounds(s, surface, uu1, uu2, vv1, vv2);
    }

    //=================================================================================================

    /// <summary>The parameter on <paramref name="c"/> of its first (<paramref name="sens"/>) or last intersection
    /// with the plane.</summary>
    internal static bool ChFi3d_InterPlaneEdge(SurfaceAdaptorBase plan, CurveAdaptorBase c, ref double w, bool sens,
                                               double tolc)
    {
        HInter intersection = new();
        int isol = 0, nbp, iip;
        double uf = c.FirstParameter(), ul = c.LastParameter();
        double cw;

        intersection.Perform(c, plan);

        if (intersection.IsDone)
        {
            nbp = intersection.PointCount;
            for (iip = 1; iip <= nbp; iip++)
            {
                cw = intersection.Point(iip).W();
                if (c.IsPeriodic)
                {
                    cw = global::Kerf.Engine.Geometry.ElCLib.InPeriod(cw, uf - tolc, uf - tolc + c.Period());
                }
                if (uf - tolc <= cw && ul + tolc >= cw)
                {
                    if (isol == 0)
                    {
                        isol = iip;
                        w = cw;
                    }
                    else
                    {
                        if (sens && cw < w)
                        {
                            w = cw;
                            isol = iip;
                        }
                        else if (!sens && cw > w)
                        {
                            w = cw;
                            isol = iip;
                        }
                    }
                }
            }
        }
        if (isol == 0)
        {
            return false;
        }
        return true;
    }

    //=================================================================================================

    /// <summary>The point, the guide's tangent and the radius at the parameter <paramref name="p"/> of the side
    /// <paramref name="jf"/> of the surface data <paramref name="i"/> of the stripe.</summary>
    internal static void ChFi3d_ExtrSpineCarac(DsDataStructure dStr, Stripe cd, int i, double p,
                                               int jf, int sens, out Point3D P, out Vector3D v, out double r)
    {
        // check if it is necessary to add D1,D2 and DR
        // Attention for approximated surfaces it is assumed that e
        // the parameters of the pcurve are the same as of
        // elspine used for its construction.
        Surface fffil = dStr.Surface(cd.SetOfSurfData()!.Value(i).Surf()).Surface()!;
        Point2D pp = cd.SetOfSurfData()!.Value(i).Interference(jf).PCurveOnSurf()!.Value(p);
        SurfaceAdaptor gs = new(fffil);
        P = fffil.Value(pp.X, pp.Y);
        Point3D pbid;
        Vector3D vbid;
        switch (gs.GetType())
        {
            case SurfaceKind.Cylinder:
            {
                Cylinder3D cyl = gs.Cylinder();
                r = cyl.Radius;
                global::Kerf.Engine.Geometry.ElSLib.D1(pp.X, pp.Y, cyl, out pbid, out vbid, out v);
            }
                break;
            case SurfaceKind.Torus:
            {
                Torus3D tor = gs.Torus();
                r = tor.MinorRadius;
                global::Kerf.Engine.Geometry.ElSLib.D1(pp.X, pp.Y, tor, out pbid, out v, out vbid);
            }
                break;
            default:
            {
                int nbelspine;
                Spine sp = cd.Spine()!;
                FilSpine fsp = (FilSpine)sp;
                nbelspine = sp.EdgeCount;
                ElSpine hels;
                if (nbelspine == 1)
                {
                    hels = sp.ElSpine(1)!;
                }
                else
                {
                    hels = sp.ElSpine(p)!;
                }
                if (fsp.IsConstant())
                {
                    r = fsp.Radius();
                }
                else
                {
                    r = fsp.Law(hels)!.Value(p);
                }
                hels.D1(p, out pbid, out v);
            }
                break;
        }
        v = v.Normalized();
        if (sens == 1)
        {
            v = v.Reversed();
        }
    }

    //=======================================================================
    // function : ChFi3d_CircularSpine
    // purpose  : Calculate a circular guideline for the corner created from
    //           tangent points and vectors calculated at the extremities
    //           of guidelines of start and end fillets.
    //=======================================================================

    /// <summary>The circular guide of a corner from the points and tangents at the ends of the two fillets' guides,
    /// or null when they turn the same way.</summary>
    internal static CircleCurve3D? ChFi3d_CircularSpine(ref double wFirst, ref double wLast, in Point3D pdeb,
                                                      in Vector3D vdeb, in Point3D pfin, in Vector3D vfin, double rad)
    {
        Circle3D ccc = new();
        Plane3D pl1 = new(pdeb, new Direction3D(vdeb)), pl2 = new(pfin, new Direction3D(vfin));
        QuadQuadGeo lInt = new(pl1, pl2, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Line3D li;
        if (lInt.IsDone)
        {
            li = lInt.Line(1);
            Point3D cendeb = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(li, pdeb), li);
            Point3D cenfin = global::Kerf.Engine.Geometry.ElCLib.Value(global::Kerf.Engine.Geometry.ElCLib.Parameter(li, pfin), li);
            Vector3D vvdeb = new(cendeb, pdeb);
            Vector3D vvfin = new(cenfin, pfin);
            Direction3D dddeb = new(vvdeb);
            Direction3D ddfin = new(vvfin);
            if (vdeb.Crossed(vvdeb).Dot(vfin.Crossed(vvfin)) > 0.0)
            {
                return null;
            }
            Frame3D circax2 = new(cendeb, dddeb ^ ddfin, dddeb);
            ccc = ccc.WithPosition(circax2);
            ccc = ccc.WithRadius(rad);
            wFirst = 0.0;
            wLast = dddeb.AngleTo(ddfin);
            return new CircleCurve3D(ccc);
        }

        return null;
    }

    //=======================================================================
    // function : ChFi3d_Spine
    // purpose  : Calculates the poles of the guideline for the corner from
    //           tangent points and vectors calculated at the extremities of
    //           guidelines of start and end fillets.
    //=======================================================================

    /// <summary>The cubic Bézier guide of a corner from the points and tangents at the ends of the two fillets'
    /// guides; scales <paramref name="vd"/> and <paramref name="vf"/>.</summary>
    internal static BezierCurve3D ChFi3d_Spine(in Point3D pd, ref Vector3D vd, in Point3D pf, ref Vector3D vf, double r)
    {
        NCollection_Array1<Point3D> pol = new(1, 4);
        double fac = 0.5 * Math.Tan((Math.PI - vd.AngleTo(vf)) * 0.5);
        pol[1] = pd;
        vd = vd.Multiplied(fac * r);
        pol[2] = pol[2].WithCoord(pd.X + vd.X, pd.Y + vd.Y, pd.Z + vd.Z);
        pol[4] = pf;
        vf = vf.Multiplied(fac * r);
        pol[3] = pol[3].WithCoord(pf.X + vf.X, pf.Y + vf.Y, pf.Z + vf.Z);
        return new BezierCurve3D(pol);
    }

    //=======================================================================
    // function : IsInFront
    // purpose  : Checks if surfdata i1 and i2 are face to face
    //=======================================================================

    /// <summary>
    /// Whether the surface data <paramref name="i1"/> of <paramref name="cd1"/> and <paramref name="i2"/> of
    /// <paramref name="cd2"/> lean on a common face whose traces cross; their crossing parameters, the face, the sides
    /// and whether they are on the same side of it.
    /// </summary>
    /// <remarks>The outputs are <c>ref</c>: OCCT leaves them unchanged (or puts back the first crossing found) when
    /// a later pair of sides does not cross.</remarks>
    internal static bool ChFi3d_IsInFront(DsDataStructure dStr, Stripe cd1, Stripe cd2,
                                          int i1, int i2, int sens1, int sens2, ref double p1, ref double p2,
                                          ref Face face, ref bool sameside, ref int jf1, ref int jf2,
                                          ref bool visavis, in Vertex Vtx, bool check2dDistance = false,
                                          bool enlarge = false)
    {
        bool isf1 = (sens1 == 1), isf2 = (sens2 == 1);
        SurfData fd1 = cd1.SetOfSurfData()!.Value(i1);
        SurfData fd2 = cd2.SetOfSurfData()!.Value(i2);

        Orientation or, orSave1, orSave2, orFace1, orFace2;
        visavis = false;
        double u1 = 0.0, u2 = 0.0;
        bool ss = false, ok = false;
        int j1 = 0, j2 = 0;
        Face ff = new();
        Vertex vtx = Vtx; // a local function cannot capture an in parameter
        if (fd1.IndexOfS1() == fd2.IndexOfS1())
        {
            jf1 = 1;
            jf2 = 1;
            face = ShapeCasts.Face(dStr.Shape(fd1.Index(jf1)));
            if (face.IsNull)
            {
                throw new NullObjectException("ChFi3d_IsInFront : Trying to check orientation of NULL face");
            }
            orSave1 = cd1.Orientation(jf1);
            or = orFace1 = face.Orientation;
            orSave2 = cd2.Orientation(jf2);
            Shape shape2 = dStr.Shape(fd2.Index(jf2));
            if (shape2.IsNull)
            {
                throw new NullObjectException("ChFi3d_IsInFront : Trying to check orientation of NULL shape");
            }
            orFace2 = shape2.Orientation;
            visavis = true;
            sameside = global::Kerf.Engine.Fillets.ChFi3d.SameSide(or, orSave1, orSave2, orFace1, orFace2);
            // The parameters of the other side are not used for orientation. This would raise problems
            int kf1 = jf1, kf2 = jf2;
            double pref1 = fd1.Interference(kf1).Parameter(isf1);
            double pref2 = fd2.Interference(kf2).Parameter(isf2);
            Point2D p2d = new();
            if (check2dDistance)
            {
                p2d = Tool.Parameters(Vtx, face);
            }
            if (ChFi3d_IntTraces(fd1, pref1, ref p1, jf1, sens1, fd2, pref2, ref p2, jf2, sens2, p2d,
                                 check2dDistance, enlarge))
            {
                u1 = p1;
                u2 = p2;
                ss = sameside;
                j1 = jf1;
                j2 = jf2;
                ff = face;
                ok = true;
            }
        }
        if (fd1.IndexOfS2() == fd2.IndexOfS1())
        {
            OtherSides(2, 1, ref p1, ref p2, ref face, ref sameside, ref jf1, ref jf2, ref visavis);
        }
        if (fd1.IndexOfS1() == fd2.IndexOfS2())
        {
            OtherSides(1, 2, ref p1, ref p2, ref face, ref sameside, ref jf1, ref jf2, ref visavis);
        }
        if (fd1.IndexOfS2() == fd2.IndexOfS2())
        {
            OtherSides(2, 2, ref p1, ref p2, ref face, ref sameside, ref jf1, ref jf2, ref visavis);
        }
        return ok;

        // OCCT writes the next three cases out in full; they differ only by the sides compared.
        void OtherSides(int s1, int s2, ref double p1, ref double p2, ref Face face, ref bool sameside,
                        ref int jf1, ref int jf2, ref bool visavis)
        {
            jf1 = s1;
            jf2 = s2;
            face = ShapeCasts.Face(dStr.Shape(fd1.Index(jf1)));
            if (face.IsNull)
            {
                throw new NullObjectException("ChFi3d_IsInFront : Trying to check orientation of NULL face");
            }
            orSave1 = cd1.Orientation(jf1);
            or = orFace1 = face.Orientation;
            orSave2 = cd2.Orientation(jf2);
            Shape shape2 = dStr.Shape(fd2.Index(jf2));
            if (shape2.IsNull)
            {
                throw new NullObjectException("ChFi3d_IsInFront : Trying to check orientation of NULL shape");
            }
            orFace2 = shape2.Orientation;
            visavis = true;
            sameside = global::Kerf.Engine.Fillets.ChFi3d.SameSide(or, orSave1, orSave2, orFace1, orFace2);
            // The parameters of the other side are not used for orientation. This would raise problems
            int kf1 = jf1, kf2 = jf2;
            double pref1 = fd1.Interference(kf1).Parameter(isf1);
            double pref2 = fd2.Interference(kf2).Parameter(isf2);
            Point2D p2d = new();
            if (check2dDistance)
            {
                p2d = Tool.Parameters(vtx, face);
            }
            if (ChFi3d_IntTraces(fd1, pref1, ref p1, jf1, sens1, fd2, pref2, ref p2, jf2, sens2, p2d,
                                 check2dDistance, enlarge))
            {
                bool restore = ok
                               && ((j1 == jf1 && sens1 * (p1 - u1) > 0.0) || (j2 == jf2 && sens2 * (p2 - u2) > 0.0));
                ok = true;
                if (restore)
                {
                    p1 = u1;
                    p2 = u2;
                    sameside = ss;
                    jf1 = j1;
                    jf2 = j2;
                    face = ff;
                }
                else
                {
                    u1 = p1;
                    u2 = p2;
                    ss = sameside;
                    j1 = jf1;
                    j2 = jf2;
                    ff = face;
                }
            }
            // the re-initialization is added in case p1,... take wrong values
            else if (ok)
            {
                p1 = u1;
                p2 = u2;
                sameside = ss;
                jf1 = j1;
                jf2 = j2;
                face = ff;
            }
        }
    }

    //=================================================================================================

    private static double Recadre(double p, double @ref, int sens, double first, double last)
    {
        double pp = p + (sens > 0 ? (first - last) : (last - first));
        return ((Math.Abs(pp - @ref) < Math.Abs(p - @ref)) ? pp : p);
    }

    /// <summary>
    /// The components (<paramref name="du"/>, <paramref name="dv"/>) in the parametric space of the vector
    /// <paramref name="v3d"/> on the tangent plane spanned by <paramref name="d1u"/> and <paramref name="d1v"/>
    /// (least squares).
    /// </summary>
    internal static void ChFi3d_Coefficient(in Vector3D v3d, in Vector3D d1u, in Vector3D d1v, out double du, out double dv)
    {
        double aa = d1u.SquareMagnitude;
        double bb = d1u.Dot(d1v);
        double cc = d1v.SquareMagnitude;
        double dd = d1u.Dot(v3d);
        double ee = d1v.Dot(v3d);
        double delta = aa * cc - bb * bb;
        du = (dd * cc - ee * bb) / delta;
        dv = (aa * ee - bb * dd) / delta;
    }
}
