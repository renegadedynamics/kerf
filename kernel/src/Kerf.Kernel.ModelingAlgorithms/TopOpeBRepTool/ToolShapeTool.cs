// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_ShapeTool.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_ShapeTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Properties;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Legacy.Tools;

/// <summary>
/// The legacy Booleans' shape tools: tolerances and points of shapes, basis curves and surfaces, parameter bounds,
/// periodic adjustments, and whether two shapes are oriented the same way.
/// </summary>
/// <remarks>Ported whole with the data structure's core (unit 7L): the data structure's same-domain filling and its points from vertices use it.</remarks>
internal static class ToolShapeTool
{
    /// <summary>The tolerance of a face, edge or vertex (0 for a null shape).</summary>
    /// <exception cref="ProgramErrorException">The shape is of another type.</exception>
    public static double Tolerance(in Shape s)
    {
        if (s.IsNull)
        {
            return 0.0;
        }
        double tol = 0;
        switch (s.Type)
        {
            case ShapeType.Face:
                tol = Tool.Tolerance(ShapeCasts.Face(s));
                break;
            case ShapeType.Edge:
                tol = Tool.Tolerance(ShapeCasts.Edge(s));
                break;
            case ShapeType.Vertex:
                tol = Tool.Tolerance(ShapeCasts.Vertex(s));
                break;
            default:
                throw new ProgramErrorException("TopOpeBRepTool_ShapeTool : Shape has no tolerance");
        }
        return tol;
    }

    /// <summary>The point of a vertex.</summary>
    /// <exception cref="ProgramErrorException">The shape is not a vertex.</exception>
    public static Point3D Pnt(in Shape s)
    {
        if (s.Type != ShapeType.Vertex)
        {
            throw new ProgramErrorException("TopOpeBRepTool_ShapeTool::Pnt");
        }
        return Tool.Pnt(ShapeCasts.Vertex(s));
    }

    /// <summary>The curve under any offset and trimming of <paramref name="c"/>.</summary>
    public static Curve3D BASISCURVE(Curve3D c)
    {
        Type t = c.DynamicType();
        if (t == typeof(OffsetCurve3D))
        {
            return BASISCURVE(((OffsetCurve3D)c).BasisCurve);
        }
        else if (t == typeof(TrimmedCurve3D))
        {
            return BASISCURVE(((TrimmedCurve3D)c).BasisCurve);
        }
        else
        {
            return c;
        }
    }

    /// <summary>The basis curve of the edge's 3D curve, or null.</summary>
    public static Curve3D? BASISCURVE(in Edge e)
    {
        Curve3D? c = Tool.Curve(e, out double _, out double _);
        if (c is null)
        {
            return c;
        }
        return BASISCURVE(c);
    }

    /// <summary>The surface under any offset and rectangular trimming of <paramref name="s"/>.</summary>
    public static Surface BASISSURFACE(Surface s)
    {
        Type t = s.DynamicType();
        if (t == typeof(global::Kerf.Geometry.OffsetSurface))
        {
            return BASISSURFACE(((global::Kerf.Geometry.OffsetSurface)s).BasisSurface);
        }
        else if (t == typeof(TrimmedSurface))
        {
            return BASISSURFACE(((TrimmedSurface)s).BasisSurface);
        }
        else
        {
            return s;
        }
    }

    /// <summary>The basis surface of the face's surface.</summary>
    public static Surface BASISSURFACE(in Face f)
    {
        Surface s = Tool.Surface(f, out Placement _)!;
        return BASISSURFACE(s);
    }

    /// <summary>
    /// The periodicities and parameter bounds of the basis surface of <paramref name="s"/>; for a surface of revolution
    /// or extrusion of a periodic curve, only the periodic direction and its bounds are set (the others are left as
    /// given, as in OCCT).
    /// </summary>
    public static void UVBOUNDS(Surface s,
                                ref bool uPeriodic,
                                ref bool vPeriodic,
                                ref double umin,
                                ref double umax,
                                ref double vmin,
                                ref double vmax)
    {
        Surface bs = BASISSURFACE(s);
        Type t = bs.DynamicType();

        if (t == typeof(RevolutionSurface))
        {
            RevolutionSurface sr = (RevolutionSurface)bs;
            Curve3D c = BASISCURVE(sr.BasisCurve);
            if (c.IsPeriodic)
            {
                uPeriodic = false;
                vPeriodic = true;
                vmin = c.FirstParameter;
                vmax = c.LastParameter;
            }
        }
        else if (t == typeof(ExtrusionSurface))
        {
            ExtrusionSurface se = (ExtrusionSurface)bs;
            Curve3D c = BASISCURVE(se.BasisCurve);
            if (c.IsPeriodic)
            {
                uPeriodic = true;
                umin = c.FirstParameter;
                umax = c.LastParameter;
                vPeriodic = false;
            }
        }
        else
        {
            uPeriodic = bs.IsUPeriodic;
            vPeriodic = bs.IsVPeriodic;
            bs.ParameterBounds(out umin, out umax, out vmin, out vmax);
        }
    }

    /// <summary>As the other overload, for the face's surface.</summary>
    public static void UVBOUNDS(in Face f,
                                ref bool uPeriodic,
                                ref bool vPeriodic,
                                ref double umin,
                                ref double umax,
                                ref double vmin,
                                ref double vmax)
    {
        Surface s = Tool.Surface(f, out Placement _)!;
        UVBOUNDS(s, ref uPeriodic, ref vPeriodic, ref umin, ref umax, ref vmin, ref vmax);
    }

    /// <summary>Brings (<paramref name="u"/>, <paramref name="v"/>) into the face's period after its first bounds, on a periodic surface.</summary>
    public static void AdjustOnPeriodic(in Shape f, ref double u, ref double v)
    {
        Face ff = ShapeCasts.Face(f);
        Surface surf = Tool.Surface(ff, out Placement _)!;

        //  double Ufirst,Ulast,Vfirst,Vlast;
        bool isUperio, isVperio;
        isUperio = surf.IsUPeriodic;
        isVperio = surf.IsVPeriodic;

        // exit if surface supporting F is not periodic on U or V
        if (!isUperio && !isVperio)
        {
            return;
        }

        BrepTools.UVBounds(ff, out double uFfirst, out double uFlast, out double vFfirst, out double vFlast);
        _ = uFlast;
        _ = vFlast;
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();

        if (isUperio)
        {
            double uperiod = surf.UPeriod;

            //    double ubid = UFfirst;

            //    ElCLib::AdjustPeriodic(UFfirst,UFfirst + Uperiod,tol,ubid,u);
            if (Math.Abs(u - uFfirst - uperiod) > tol)
            {
                u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uFfirst, uFfirst + uperiod);
            }
        }
        if (isVperio)
        {
            double vperiod = surf.VPeriod;

            //    double vbid = VFfirst;

            //    ElCLib::AdjustPeriodic(VFfirst,VFfirst + Vperiod,tol,vbid,v);
            if (Math.Abs(v - vFfirst - vperiod) > tol)
            {
                v = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, vFfirst, vFfirst + vperiod);
            }
        }
    }

    /// <summary>Whether the edge <paramref name="s1"/> is closed on the face <paramref name="s2"/> and appears there twice.</summary>
    public static bool Closed(in Shape s1, in Shape s2)
    {
        Edge e = ShapeCasts.Edge(s1);
        Face f = ShapeCasts.Face(s2);
        bool brepclosed = Tool.IsClosed(e, f);
        if (brepclosed)
        {
            int n = 0;
            for (Explorer x = new(f, ShapeType.Edge); x.More(); x.Next())
            {
                if (x.Current().IsSame(e))
                {
                    n++;
                }
            }
            return n >= 2;
        }
        return false;
    }

    // OCCT_DEBUG traces (TopOpeBRepTool_GettraceVC, TopOpeBRepTool_GettraceNYI) left out

    private static bool PARINBOUNDS(double par, double first, double last, double tol)
    {
        bool b = (((first + tol) <= par) && (par <= (last - tol)));
        return b;
    }

    private static bool PARONBOUND(double par, double bound, double tol)
    {
        bool b = (((bound - tol) <= par) && (par <= (bound + tol)));
        return b;
    }

    private static double ADJUST(double par, double first, double last, double tol)
    {
        double period = last - first, periopar = par;

        if (PARINBOUNDS(par, first, last, tol))
        {
            periopar = par + period;
        }
        else if (PARONBOUND(par, first, tol))
        {
            periopar = par + period;
        }
        else if (PARONBOUND(par, last, tol))
        {
            periopar = par - period;
        }
        return periopar;
    }

    /// <summary>
    /// The parameter on the other side of the seam: for an edge closed on the face with an iso-line pcurve, the
    /// parameter shifted by the surface's period.
    /// </summary>
    /// <exception cref="ProgramErrorException">The edge has no pcurve on the face.</exception>
    public static double PeriodizeParameter(double par, in Shape ee, in Shape ff)
    {
        double periopar = par;
        if (!Closed(ee, ff))
        {
            return periopar;
        }

        Edge e = ShapeCasts.Edge(ee);
        Face f = ShapeCasts.Face(ff);

        Surface surf = Tool.Surface(f, out Placement _)!;
        bool isUperio = surf.IsUPeriodic;
        bool isVperio = surf.IsVPeriodic;
        if (!isUperio && !isVperio)
        {
            return periopar;
        }

        surf.ParameterBounds(out double ufirst, out double ulast, out double vfirst, out double vlast);

        Curve2D? pc = Legacy2D.FC2D_CurveOnSurface(e, f, out double _, out double _, out double _);
        if (pc is null)
        {
            throw new ProgramErrorException("ShapeTool::PeriodizeParameter : no 2d curve");
        }

        Type theType = pc.DynamicType();
        if (theType == typeof(LineCurve2D))
        {
            LineCurve2D hl = (LineCurve2D)pc;
            Direction2D d = hl.Direction;
            double tol = global::Kerf.Engine.Foundation.Precision.Angular();
            bool isoU = false, isoV = false;
            if (d.IsParallel(new Direction2D(Direction2D.D.Y), tol))
            {
                isoU = true;
            }
            else if (d.IsParallel(new Direction2D(Direction2D.D.X), tol))
            {
                isoV = true;
            }
            if (isoU)
            {
                periopar = ADJUST(par, ufirst, ulast, tol);
            }
            else if (isoV)
            {
                periopar = ADJUST(par, vfirst, vlast, tol);
            }

            // OCCT_DEBUG trace left out
        }

        // NYI : BSpline ...

        return periopar;
    }

    /// <summary>
    /// Whether <paramref name="s1"/> and <paramref name="s2"/> are oriented the same way: by their orientations for
    /// the same shape; by their normals (planes, cylinders) or tangents (lines) for faces or edges; true otherwise.
    /// </summary>
    public static bool ShapesSameOriented(in Shape s1, in Shape s2)
    {
        bool so = true;

        bool sam = s1.IsSame(s2);
        if (sam)
        {
            Orientation o1 = s1.Orientation;
            Orientation o2 = s2.Orientation;
            if ((o1 == Forward || o1 == Reversed)
                && (o2 == Forward || o2 == Reversed))
            {
                so = (o1 == o2);
                return so;
            }
        }

        ShapeType t1 = s1.Type, t2 = s2.Type;
        if ((t1 == ShapeType.Solid) && (t2 == ShapeType.Solid))
        {
            so = true;
        }
        else if ((t1 == ShapeType.Face) && (t2 == ShapeType.Face))
        {
            so = FacesSameOriented(s1, s2);
        }
        else if ((t1 == ShapeType.Edge) && (t2 == ShapeType.Edge))
        {
            so = EdgesSameOriented(s1, s2);
        }
        else if ((t1 == ShapeType.Vertex) && (t2 == ShapeType.Vertex))
        {
            Orientation o1 = s1.Orientation;
            Orientation o2 = s2.Orientation;
            if (o1 == External || o1 == Internal || o2 == External
                || o2 == Internal)
            {
                so = true;
            }
            else
            {
                so = (o1 == o2);
            }
        }

        return so;
    }

    /// <summary>
    /// Whether two planes (or two cylinders) have normals pointing the same way; true for other surfaces (not
    /// implemented in OCCT).
    /// </summary>
    public static bool SurfacesSameOriented(FaceSurfaceAdaptor s1, FaceSurfaceAdaptor sref)
    {
        FaceSurfaceAdaptor s2 = sref;
        SurfaceKind st1 = s1.GetType();
        SurfaceKind st2 = s2.GetType();

        bool so = true;

        if (st1 == Plane && st2 == Plane)
        {
            double u1 = s1.FirstUParameter();
            double v1 = s1.FirstVParameter();
            s1.D1(u1, v1, out Point3D p1, out Vector3D d1u, out Vector3D d1v);
            _ = p1;
            Vector3D n1 = d1u.Crossed(d1v);

            double u2 = s2.FirstUParameter();
            double v2 = s2.FirstVParameter();
            s2.D1(u2, v2, out Point3D p2, out Vector3D d2u, out Vector3D d2v);
            _ = p2;
            Vector3D n2 = d2u.Crossed(d2v);

            double d = n1.Dot(n2);
            so = (d > 0.0);
        }
        else if (st1 == SurfaceKind.Cylinder && st2 == SurfaceKind.Cylinder)
        {
            // On peut projeter n'importe quel point.
            // prenons donc l'origine
            double u1 = 0.0;
            double v1 = 0.0;
            s1.D1(u1, v1, out Point3D p1, out Vector3D d1u, out Vector3D d1v);
            Vector3D n1 = d1u.Crossed(d1v);

            Surface hs2 = s2.GeomSurfaceTransformed()!;
            Point2D p22d = new();
            double dp2 = 0.0;
            bool ok = Project.FUN_tool_projPonS(p1, hs2, ref p22d, ref dp2);
            if (!ok)
            {
                return so; // NYI : raise
            }

            double u2 = p22d.X;
            double v2 = p22d.Y;
            s2.D1(u2, v2, out Point3D p2, out Vector3D d2u, out Vector3D d2v);
            _ = p2;
            Vector3D n2 = d2u.Crossed(d2v);

            double d = n1.Dot(n2);
            so = (d > 0.0);
        }
        else
        {
            // prendre u1,v1 et projeter sur 2 pour calcul des normales
            // au meme point 3d.
            // OCCT_DEBUG trace left out
        }

        return so;
    }

    /// <summary>Whether two faces are oriented the same way (their surfaces' orientations combined with theirs).</summary>
    public static bool FacesSameOriented(in Shape s1, in Shape sref)
    {
        Shape s2 = sref;
        Face f1 = ShapeCasts.Face(s1);
        Face f2 = ShapeCasts.Face(s2);
        Orientation o1 = f1.Orientation;
        Orientation o2 = f2.Orientation;
        if (o1 == External || o1 == Internal || o2 == External || o2 == Internal)
        {
            return true;
        }

        bool computerestriction = false;
        FaceSurfaceAdaptor bas1 = new(f1, computerestriction);
        FaceSurfaceAdaptor bas2 = new(f2, computerestriction);
        bool so = f1.IsSame(f2) || SurfacesSameOriented(bas1, bas2);
        bool b = so;
        if (o1 != o2)
        {
            b = !so;
        }
        return b;
    }

    /// <summary>Whether two lines have tangents pointing the same way; true for other curves (not implemented in OCCT).</summary>
    public static bool CurvesSameOriented(EdgeCurveAdaptor C1, EdgeCurveAdaptor cref)
    {
        EdgeCurveAdaptor C2 = cref;
        CurveKind ct1 = C1.GetType();
        CurveKind ct2 = C2.GetType();
        bool so = true;

        if (ct1 == Line && ct2 == Line)
        {
            double p1 = C1.FirstParameter();
            Direction3D t1 = new(), n1 = new();
            double c1 = 0.0;
            EdgeData(C1, p1, ref t1, ref n1, ref c1);
            double p2 = C2.FirstParameter();
            Direction3D t2 = new(), n2 = new();
            double c2 = 0.0;
            EdgeData(C2, p2, ref t2, ref n2, ref c2);
            double d = t1.Dot(t2);
            so = (d > 0.0);
        }
        else
        {
            // prendre p1 et projeter sur 2 pour calcul des normales
            // au meme point 3d.
            // OCCT_DEBUG trace left out
        }

        return so;
    }

    /// <summary>Whether two edges are oriented the same way (their curves' orientations combined with theirs).</summary>
    public static bool EdgesSameOriented(in Shape s1, in Shape sref)
    {
        Shape s2 = sref;
        Edge e1 = ShapeCasts.Edge(s1);
        Edge e2 = ShapeCasts.Edge(s2);
        Orientation o1 = e1.Orientation;
        Orientation o2 = e2.Orientation;
        if (o1 == External || o1 == Internal || o2 == External || o2 == Internal)
        {
            return true;
        }
        EdgeCurveAdaptor bac1 = new(e1);
        EdgeCurveAdaptor bac2 = new(e2);
        bool so = CurvesSameOriented(bac1, bac2);
        bool b = so;
        if (o1 != o2)
        {
            b = !so;
        }
        return b;
    }

    /// <summary>
    /// The tangent <paramref name="t"/>, the normal <paramref name="n"/> (set only where the curvature is above the
    /// tolerance) and the curvature <paramref name="c"/> of the curve at <paramref name="p"/>; returns the angular tolerance.
    /// </summary>
    public static double EdgeData(EdgeCurveAdaptor bac, double p, ref Direction3D t, ref Direction3D n, ref double c)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.Angular();

        CLProps bl = new(bac, p, 2, tol);
        bl.Tangent(out t);
        c = bl.Curvature();

        // xpu150399 cto900R4
        double tol1 = Epsilon(0.0);
        double tol2 = RealLast();
        double tolm = Max(tol, Max(tol1, tol2));

        // OCCT: tolm is RealLast, so the normal is never computed
        if (Math.Abs(c) > tolm)
        {
            bl.Normal(out n);
        }
        return tol;
    }

    /// <summary>As the other overload, for the edge <paramref name="e"/>.</summary>
    public static double EdgeData(in Shape e, double p, ref Direction3D t, ref Direction3D n, ref double c)
    {
        EdgeCurveAdaptor bac = new(ShapeCasts.Edge(e));
        double d = EdgeData(bac, p, ref t, ref n, ref c);
        return d;
    }

    /// <summary>The 3D length matching the 2D tolerance <paramref name="tol2d"/> along U.</summary>
    public static double Resolution3dU(Surface su, double tol2d)
    {
        SurfaceAdaptor gas = new(su);
        double r3dunit = 0.00001; // petite valeur (1.0 -> RangeError sur un tore)
        double ru = gas.UResolution(r3dunit);
        double r3du = r3dunit * (tol2d / ru);
        return r3du;
    }

    /// <summary>The 3D length matching the 2D tolerance <paramref name="tol2d"/> along V.</summary>
    public static double Resolution3dV(Surface su, double tol2d)
    {
        SurfaceAdaptor gas = new(su);
        double r3dunit = 0.00001; // petite valeur (1.0 -> RangeError sur un tore)
        double rv = gas.VResolution(r3dunit);
        double r3dv = r3dunit * (tol2d / rv);
        return r3dv;
    }

    /// <summary>The larger of <see cref="Resolution3dU"/> and <see cref="Resolution3dV"/>.</summary>
    public static double Resolution3d(Surface su, double tol2d)
    {
        double ru = Resolution3dU(su, tol2d);
        double rv = Resolution3dV(su, tol2d);
        double r = Max(ru, rv);
        return r;
    }

    /// <summary>As the other overload, for the face's surface.</summary>
    public static double Resolution3d(in Face f, double tol2d)
    {
        Surface su = Tool.Surface(f, out Placement _)!;
        double r = Resolution3d(su, tol2d);
        return r;
    }
}
