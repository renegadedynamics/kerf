// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_ShapeTool.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_ShapeTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Properties;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The legacy Booleans' shape tools: tolerances and points of shapes, basis curves and surfaces, parameter bounds,
/// periodic adjustments, and whether two shapes are oriented the same way.
/// </summary>
/// <remarks>Ported whole with the data structure's core (unit 7L): the data structure's same-domain filling and its points from vertices use it.</remarks>
public static class TopOpeBRepTool_ShapeTool
{
    /// <summary>The tolerance of a face, edge or vertex (0 for a null shape).</summary>
    /// <exception cref="ProgramErrorException">The shape is of another type.</exception>
    public static double Tolerance(in Shape S)
    {
        if (S.IsNull)
        {
            return 0.0;
        }
        double tol = 0;
        switch (S.ShapeType())
        {
            case ShapeType.Face:
                tol = Tool.Tolerance(ShapeCasts.Face(S));
                break;
            case ShapeType.Edge:
                tol = Tool.Tolerance(ShapeCasts.Edge(S));
                break;
            case ShapeType.Vertex:
                tol = Tool.Tolerance(ShapeCasts.Vertex(S));
                break;
            default:
                throw new ProgramErrorException("TopOpeBRepTool_ShapeTool : Shape has no tolerance");
        }
        return tol;
    }

    /// <summary>The point of a vertex.</summary>
    /// <exception cref="ProgramErrorException">The shape is not a vertex.</exception>
    public static Point3D Pnt(in Shape S)
    {
        if (S.ShapeType() != ShapeType.Vertex)
        {
            throw new ProgramErrorException("TopOpeBRepTool_ShapeTool::Pnt");
        }
        return Tool.Pnt(ShapeCasts.Vertex(S));
    }

    /// <summary>The curve under any offset and trimming of <paramref name="C"/>.</summary>
    public static Curve3D BASISCURVE(Curve3D C)
    {
        Type T = C.DynamicType();
        if (T == typeof(OffsetCurve3D))
        {
            return BASISCURVE(((OffsetCurve3D)C).BasisCurve());
        }
        else if (T == typeof(TrimmedCurve3D))
        {
            return BASISCURVE(((TrimmedCurve3D)C).BasisCurve());
        }
        else
        {
            return C;
        }
    }

    /// <summary>The basis curve of the edge's 3D curve, or null.</summary>
    public static Curve3D? BASISCURVE(in Edge E)
    {
        Curve3D? C = Tool.Curve(E, out double _, out double _);
        if (C is null)
        {
            return C;
        }
        return BASISCURVE(C);
    }

    /// <summary>The surface under any offset and rectangular trimming of <paramref name="S"/>.</summary>
    public static Surface BASISSURFACE(Surface S)
    {
        Type T = S.DynamicType();
        if (T == typeof(global::Kerf.Geometry.OffsetSurface))
        {
            return BASISSURFACE(((global::Kerf.Geometry.OffsetSurface)S).BasisSurface());
        }
        else if (T == typeof(TrimmedSurface))
        {
            return BASISSURFACE(((TrimmedSurface)S).BasisSurface());
        }
        else
        {
            return S;
        }
    }

    /// <summary>The basis surface of the face's surface.</summary>
    public static Surface BASISSURFACE(in Face F)
    {
        Surface S = Tool.Surface(F, out Placement _)!;
        return BASISSURFACE(S);
    }

    /// <summary>
    /// The periodicities and parameter bounds of the basis surface of <paramref name="S"/>; for a surface of revolution
    /// or extrusion of a periodic curve, only the periodic direction and its bounds are set (the others are left as
    /// given, as in OCCT).
    /// </summary>
    public static void UVBOUNDS(Surface S,
                                ref bool UPeriodic,
                                ref bool VPeriodic,
                                ref double Umin,
                                ref double Umax,
                                ref double Vmin,
                                ref double Vmax)
    {
        Surface BS = BASISSURFACE(S);
        Type T = BS.DynamicType();

        if (T == typeof(RevolutionSurface))
        {
            RevolutionSurface SR = (RevolutionSurface)BS;
            Curve3D C = BASISCURVE(SR.BasisCurve());
            if (C.IsPeriodic)
            {
                UPeriodic = false;
                VPeriodic = true;
                Vmin = C.FirstParameter();
                Vmax = C.LastParameter();
            }
        }
        else if (T == typeof(ExtrusionSurface))
        {
            ExtrusionSurface SE = (ExtrusionSurface)BS;
            Curve3D C = BASISCURVE(SE.BasisCurve());
            if (C.IsPeriodic)
            {
                UPeriodic = true;
                Umin = C.FirstParameter();
                Umax = C.LastParameter();
                VPeriodic = false;
            }
        }
        else
        {
            UPeriodic = BS.IsUPeriodic;
            VPeriodic = BS.IsVPeriodic;
            BS.Bounds(out Umin, out Umax, out Vmin, out Vmax);
        }
    }

    /// <summary>As the other overload, for the face's surface.</summary>
    public static void UVBOUNDS(in Face F,
                                ref bool UPeriodic,
                                ref bool VPeriodic,
                                ref double Umin,
                                ref double Umax,
                                ref double Vmin,
                                ref double Vmax)
    {
        Surface S = Tool.Surface(F, out Placement _)!;
        UVBOUNDS(S, ref UPeriodic, ref VPeriodic, ref Umin, ref Umax, ref Vmin, ref Vmax);
    }

    /// <summary>Brings (<paramref name="u"/>, <paramref name="v"/>) into the face's period after its first bounds, on a periodic surface.</summary>
    public static void AdjustOnPeriodic(in Shape F, ref double u, ref double v)
    {
        Face FF = ShapeCasts.Face(F);
        Surface Surf = Tool.Surface(FF, out Placement _)!;

        //  double Ufirst,Ulast,Vfirst,Vlast;
        bool isUperio, isVperio;
        isUperio = Surf.IsUPeriodic;
        isVperio = Surf.IsVPeriodic;

        // exit if surface supporting F is not periodic on U or V
        if (!isUperio && !isVperio)
        {
            return;
        }

        BrepTools.UVBounds(FF, out double UFfirst, out double UFlast, out double VFfirst, out double VFlast);
        _ = UFlast;
        _ = VFlast;
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();

        if (isUperio)
        {
            double Uperiod = Surf.UPeriod();

            //    double ubid = UFfirst;

            //    ElCLib::AdjustPeriodic(UFfirst,UFfirst + Uperiod,tol,ubid,u);
            if (Math.Abs(u - UFfirst - Uperiod) > tol)
            {
                u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, UFfirst, UFfirst + Uperiod);
            }
        }
        if (isVperio)
        {
            double Vperiod = Surf.VPeriod();

            //    double vbid = VFfirst;

            //    ElCLib::AdjustPeriodic(VFfirst,VFfirst + Vperiod,tol,vbid,v);
            if (Math.Abs(v - VFfirst - Vperiod) > tol)
            {
                v = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, VFfirst, VFfirst + Vperiod);
            }
        }
    }

    /// <summary>Whether the edge <paramref name="S1"/> is closed on the face <paramref name="S2"/> and appears there twice.</summary>
    public static bool Closed(in Shape S1, in Shape S2)
    {
        Edge E = ShapeCasts.Edge(S1);
        Face F = ShapeCasts.Face(S2);
        bool brepclosed = Tool.IsClosed(E, F);
        if (brepclosed)
        {
            int n = 0;
            for (Explorer x = new(F, ShapeType.Edge); x.More(); x.Next())
            {
                if (x.Current().IsSame(E))
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
    public static double PeriodizeParameter(double par, in Shape EE, in Shape FF)
    {
        double periopar = par;
        if (!Closed(EE, FF))
        {
            return periopar;
        }

        Edge E = ShapeCasts.Edge(EE);
        Face F = ShapeCasts.Face(FF);

        Surface Surf = Tool.Surface(F, out Placement _)!;
        bool isUperio = Surf.IsUPeriodic;
        bool isVperio = Surf.IsVPeriodic;
        if (!isUperio && !isVperio)
        {
            return periopar;
        }

        Surf.Bounds(out double Ufirst, out double Ulast, out double Vfirst, out double Vlast);

        Curve2D? PC = TopOpeBRepTool_2d.FC2D_CurveOnSurface(E, F, out double _, out double _, out double _);
        if (PC is null)
        {
            throw new ProgramErrorException("ShapeTool::PeriodizeParameter : no 2d curve");
        }

        Type TheType = PC.DynamicType();
        if (TheType == typeof(LineCurve2D))
        {
            LineCurve2D HL = (LineCurve2D)PC;
            Direction2D D = HL.Direction();
            double tol = global::Kerf.Engine.Foundation.Precision.Angular();
            bool isoU = false, isoV = false;
            if (D.IsParallel(new Direction2D(Direction2D.D.Y), tol))
            {
                isoU = true;
            }
            else if (D.IsParallel(new Direction2D(Direction2D.D.X), tol))
            {
                isoV = true;
            }
            if (isoU)
            {
                periopar = ADJUST(par, Ufirst, Ulast, tol);
            }
            else if (isoV)
            {
                periopar = ADJUST(par, Vfirst, Vlast, tol);
            }

            // OCCT_DEBUG trace left out
        }

        // NYI : BSpline ...

        return periopar;
    }

    /// <summary>
    /// Whether <paramref name="S1"/> and <paramref name="S2"/> are oriented the same way: by their orientations for
    /// the same shape; by their normals (planes, cylinders) or tangents (lines) for faces or edges; true otherwise.
    /// </summary>
    public static bool ShapesSameOriented(in Shape S1, in Shape S2)
    {
        bool so = true;

        bool sam = S1.IsSame(S2);
        if (sam)
        {
            Orientation o1 = S1.Orientation();
            Orientation o2 = S2.Orientation();
            if ((o1 == Forward || o1 == Reversed)
                && (o2 == Forward || o2 == Reversed))
            {
                so = (o1 == o2);
                return so;
            }
        }

        ShapeType t1 = S1.ShapeType(), t2 = S2.ShapeType();
        if ((t1 == ShapeType.Solid) && (t2 == ShapeType.Solid))
        {
            so = true;
        }
        else if ((t1 == ShapeType.Face) && (t2 == ShapeType.Face))
        {
            so = FacesSameOriented(S1, S2);
        }
        else if ((t1 == ShapeType.Edge) && (t2 == ShapeType.Edge))
        {
            so = EdgesSameOriented(S1, S2);
        }
        else if ((t1 == ShapeType.Vertex) && (t2 == ShapeType.Vertex))
        {
            Orientation o1 = S1.Orientation();
            Orientation o2 = S2.Orientation();
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
    public static bool SurfacesSameOriented(FaceSurfaceAdaptor S1, FaceSurfaceAdaptor Sref)
    {
        FaceSurfaceAdaptor S2 = Sref;
        SurfaceKind ST1 = S1.GetType();
        SurfaceKind ST2 = S2.GetType();

        bool so = true;

        if (ST1 == Plane && ST2 == Plane)
        {
            double u1 = S1.FirstUParameter();
            double v1 = S1.FirstVParameter();
            S1.D1(u1, v1, out Point3D p1, out Vector3D d1u, out Vector3D d1v);
            _ = p1;
            Vector3D n1 = d1u.Crossed(d1v);

            double u2 = S2.FirstUParameter();
            double v2 = S2.FirstVParameter();
            S2.D1(u2, v2, out Point3D p2, out Vector3D d2u, out Vector3D d2v);
            _ = p2;
            Vector3D n2 = d2u.Crossed(d2v);

            double d = n1.Dot(n2);
            so = (d > 0.0);
        }
        else if (ST1 == SurfaceKind.Cylinder && ST2 == SurfaceKind.Cylinder)
        {
            // On peut projeter n'importe quel point.
            // prenons donc l'origine
            double u1 = 0.0;
            double v1 = 0.0;
            S1.D1(u1, v1, out Point3D p1, out Vector3D d1u, out Vector3D d1v);
            Vector3D n1 = d1u.Crossed(d1v);

            Surface HS2 = S2.GeomSurfaceTransformed()!;
            Point2D p22d = new();
            double dp2 = 0.0;
            bool ok = TopOpeBRepTool_PROJECT.FUN_tool_projPonS(p1, HS2, ref p22d, ref dp2);
            if (!ok)
            {
                return so; // NYI : raise
            }

            double u2 = p22d.X();
            double v2 = p22d.Y();
            S2.D1(u2, v2, out Point3D p2, out Vector3D d2u, out Vector3D d2v);
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
    public static bool FacesSameOriented(in Shape S1, in Shape Sref)
    {
        Shape S2 = Sref;
        Face F1 = ShapeCasts.Face(S1);
        Face F2 = ShapeCasts.Face(S2);
        Orientation o1 = F1.Orientation();
        Orientation o2 = F2.Orientation();
        if (o1 == External || o1 == Internal || o2 == External || o2 == Internal)
        {
            return true;
        }

        bool computerestriction = false;
        FaceSurfaceAdaptor BAS1 = new(F1, computerestriction);
        FaceSurfaceAdaptor BAS2 = new(F2, computerestriction);
        bool so = F1.IsSame(F2) || SurfacesSameOriented(BAS1, BAS2);
        bool b = so;
        if (o1 != o2)
        {
            b = !so;
        }
        return b;
    }

    /// <summary>Whether two lines have tangents pointing the same way; true for other curves (not implemented in OCCT).</summary>
    public static bool CurvesSameOriented(EdgeCurveAdaptor C1, EdgeCurveAdaptor Cref)
    {
        EdgeCurveAdaptor C2 = Cref;
        CurveKind CT1 = C1.GetType();
        CurveKind CT2 = C2.GetType();
        bool so = true;

        if (CT1 == Line && CT2 == Line)
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
    public static bool EdgesSameOriented(in Shape S1, in Shape Sref)
    {
        Shape S2 = Sref;
        Edge E1 = ShapeCasts.Edge(S1);
        Edge E2 = ShapeCasts.Edge(S2);
        Orientation o1 = E1.Orientation();
        Orientation o2 = E2.Orientation();
        if (o1 == External || o1 == Internal || o2 == External || o2 == Internal)
        {
            return true;
        }
        EdgeCurveAdaptor BAC1 = new(E1);
        EdgeCurveAdaptor BAC2 = new(E2);
        bool so = CurvesSameOriented(BAC1, BAC2);
        bool b = so;
        if (o1 != o2)
        {
            b = !so;
        }
        return b;
    }

    /// <summary>
    /// The tangent <paramref name="T"/>, the normal <paramref name="N"/> (set only where the curvature is above the
    /// tolerance) and the curvature <paramref name="C"/> of the curve at <paramref name="P"/>; returns the angular tolerance.
    /// </summary>
    public static double EdgeData(EdgeCurveAdaptor BAC, double P, ref Direction3D T, ref Direction3D N, ref double C)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.Angular();

        CLProps BL = new(BAC, P, 2, tol);
        BL.Tangent(out T);
        C = BL.Curvature();

        // xpu150399 cto900R4
        double tol1 = Epsilon(0.0);
        double tol2 = RealLast();
        double tolm = Max(tol, Max(tol1, tol2));

        // OCCT: tolm is RealLast, so the normal is never computed
        if (Math.Abs(C) > tolm)
        {
            BL.Normal(out N);
        }
        return tol;
    }

    /// <summary>As the other overload, for the edge <paramref name="E"/>.</summary>
    public static double EdgeData(in Shape E, double P, ref Direction3D T, ref Direction3D N, ref double C)
    {
        EdgeCurveAdaptor BAC = new(ShapeCasts.Edge(E));
        double d = EdgeData(BAC, P, ref T, ref N, ref C);
        return d;
    }

    /// <summary>The 3D length matching the 2D tolerance <paramref name="Tol2d"/> along U.</summary>
    public static double Resolution3dU(Surface SU, double Tol2d)
    {
        SurfaceAdaptor GAS = new(SU);
        double r3dunit = 0.00001; // petite valeur (1.0 -> RangeError sur un tore)
        double ru = GAS.UResolution(r3dunit);
        double r3du = r3dunit * (Tol2d / ru);
        return r3du;
    }

    /// <summary>The 3D length matching the 2D tolerance <paramref name="Tol2d"/> along V.</summary>
    public static double Resolution3dV(Surface SU, double Tol2d)
    {
        SurfaceAdaptor GAS = new(SU);
        double r3dunit = 0.00001; // petite valeur (1.0 -> RangeError sur un tore)
        double rv = GAS.VResolution(r3dunit);
        double r3dv = r3dunit * (Tol2d / rv);
        return r3dv;
    }

    /// <summary>The larger of <see cref="Resolution3dU"/> and <see cref="Resolution3dV"/>.</summary>
    public static double Resolution3d(Surface SU, double Tol2d)
    {
        double ru = Resolution3dU(SU, Tol2d);
        double rv = Resolution3dV(SU, Tol2d);
        double r = Max(ru, rv);
        return r;
    }

    /// <summary>As the other overload, for the face's surface.</summary>
    public static double Resolution3d(in Face F, double Tol2d)
    {
        Surface SU = Tool.Surface(F, out Placement _)!;
        double r = Resolution3d(SU, Tol2d);
        return r;
    }
}
