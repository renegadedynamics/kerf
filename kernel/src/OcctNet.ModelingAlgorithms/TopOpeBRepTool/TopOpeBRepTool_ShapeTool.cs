// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_ShapeTool.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_ShapeTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

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
    public static double Tolerance(in TopoDS_Shape S)
    {
        if (S.IsNull())
        {
            return 0.0;
        }
        double tol = 0;
        switch (S.ShapeType())
        {
            case TopAbs_FACE:
                tol = BRep_Tool.Tolerance(TopoDS.Face(S));
                break;
            case TopAbs_EDGE:
                tol = BRep_Tool.Tolerance(TopoDS.Edge(S));
                break;
            case TopAbs_VERTEX:
                tol = BRep_Tool.Tolerance(TopoDS.Vertex(S));
                break;
            default:
                throw new ProgramErrorException("TopOpeBRepTool_ShapeTool : Shape has no tolerance");
        }
        return tol;
    }

    /// <summary>The point of a vertex.</summary>
    /// <exception cref="ProgramErrorException">The shape is not a vertex.</exception>
    public static Point3D Pnt(in TopoDS_Shape S)
    {
        if (S.ShapeType() != TopAbs_VERTEX)
        {
            throw new ProgramErrorException("TopOpeBRepTool_ShapeTool::Pnt");
        }
        return BRep_Tool.Pnt(TopoDS.Vertex(S));
    }

    /// <summary>The curve under any offset and trimming of <paramref name="C"/>.</summary>
    public static Geom_Curve BASISCURVE(Geom_Curve C)
    {
        Type T = C.DynamicType();
        if (T == typeof(Geom_OffsetCurve))
        {
            return BASISCURVE(((Geom_OffsetCurve)C).BasisCurve());
        }
        else if (T == typeof(Geom_TrimmedCurve))
        {
            return BASISCURVE(((Geom_TrimmedCurve)C).BasisCurve());
        }
        else
        {
            return C;
        }
    }

    /// <summary>The basis curve of the edge's 3D curve, or null.</summary>
    public static Geom_Curve? BASISCURVE(in TopoDS_Edge E)
    {
        Geom_Curve? C = BRep_Tool.Curve(E, out double _, out double _);
        if (C is null)
        {
            return C;
        }
        return BASISCURVE(C);
    }

    /// <summary>The surface under any offset and rectangular trimming of <paramref name="S"/>.</summary>
    public static Geom_Surface BASISSURFACE(Geom_Surface S)
    {
        Type T = S.DynamicType();
        if (T == typeof(Geom_OffsetSurface))
        {
            return BASISSURFACE(((Geom_OffsetSurface)S).BasisSurface());
        }
        else if (T == typeof(Geom_RectangularTrimmedSurface))
        {
            return BASISSURFACE(((Geom_RectangularTrimmedSurface)S).BasisSurface());
        }
        else
        {
            return S;
        }
    }

    /// <summary>The basis surface of the face's surface.</summary>
    public static Geom_Surface BASISSURFACE(in TopoDS_Face F)
    {
        Geom_Surface S = BRep_Tool.Surface(F, out Placement _)!;
        return BASISSURFACE(S);
    }

    /// <summary>
    /// The periodicities and parameter bounds of the basis surface of <paramref name="S"/>; for a surface of revolution
    /// or extrusion of a periodic curve, only the periodic direction and its bounds are set (the others are left as
    /// given, as in OCCT).
    /// </summary>
    public static void UVBOUNDS(Geom_Surface S,
                                ref bool UPeriodic,
                                ref bool VPeriodic,
                                ref double Umin,
                                ref double Umax,
                                ref double Vmin,
                                ref double Vmax)
    {
        Geom_Surface BS = BASISSURFACE(S);
        Type T = BS.DynamicType();

        if (T == typeof(Geom_SurfaceOfRevolution))
        {
            Geom_SurfaceOfRevolution SR = (Geom_SurfaceOfRevolution)BS;
            Geom_Curve C = BASISCURVE(SR.BasisCurve());
            if (C.IsPeriodic())
            {
                UPeriodic = false;
                VPeriodic = true;
                Vmin = C.FirstParameter();
                Vmax = C.LastParameter();
            }
        }
        else if (T == typeof(Geom_SurfaceOfLinearExtrusion))
        {
            Geom_SurfaceOfLinearExtrusion SE = (Geom_SurfaceOfLinearExtrusion)BS;
            Geom_Curve C = BASISCURVE(SE.BasisCurve());
            if (C.IsPeriodic())
            {
                UPeriodic = true;
                Umin = C.FirstParameter();
                Umax = C.LastParameter();
                VPeriodic = false;
            }
        }
        else
        {
            UPeriodic = BS.IsUPeriodic();
            VPeriodic = BS.IsVPeriodic();
            BS.Bounds(out Umin, out Umax, out Vmin, out Vmax);
        }
    }

    /// <summary>As the other overload, for the face's surface.</summary>
    public static void UVBOUNDS(in TopoDS_Face F,
                                ref bool UPeriodic,
                                ref bool VPeriodic,
                                ref double Umin,
                                ref double Umax,
                                ref double Vmin,
                                ref double Vmax)
    {
        Geom_Surface S = BRep_Tool.Surface(F, out Placement _)!;
        UVBOUNDS(S, ref UPeriodic, ref VPeriodic, ref Umin, ref Umax, ref Vmin, ref Vmax);
    }

    /// <summary>Brings (<paramref name="u"/>, <paramref name="v"/>) into the face's period after its first bounds, on a periodic surface.</summary>
    public static void AdjustOnPeriodic(in TopoDS_Shape F, ref double u, ref double v)
    {
        TopoDS_Face FF = TopoDS.Face(F);
        Geom_Surface Surf = BRep_Tool.Surface(FF, out Placement _)!;

        //  double Ufirst,Ulast,Vfirst,Vlast;
        bool isUperio, isVperio;
        isUperio = Surf.IsUPeriodic();
        isVperio = Surf.IsVPeriodic();

        // exit if surface supporting F is not periodic on U or V
        if (!isUperio && !isVperio)
        {
            return;
        }

        BRepTools.UVBounds(FF, out double UFfirst, out double UFlast, out double VFfirst, out double VFlast);
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
    public static bool Closed(in TopoDS_Shape S1, in TopoDS_Shape S2)
    {
        TopoDS_Edge E = TopoDS.Edge(S1);
        TopoDS_Face F = TopoDS.Face(S2);
        bool brepclosed = BRep_Tool.IsClosed(E, F);
        if (brepclosed)
        {
            int n = 0;
            for (TopExp_Explorer x = new(F, TopAbs_EDGE); x.More(); x.Next())
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
    public static double PeriodizeParameter(double par, in TopoDS_Shape EE, in TopoDS_Shape FF)
    {
        double periopar = par;
        if (!Closed(EE, FF))
        {
            return periopar;
        }

        TopoDS_Edge E = TopoDS.Edge(EE);
        TopoDS_Face F = TopoDS.Face(FF);

        Geom_Surface Surf = BRep_Tool.Surface(F, out Placement _)!;
        bool isUperio = Surf.IsUPeriodic();
        bool isVperio = Surf.IsVPeriodic();
        if (!isUperio && !isVperio)
        {
            return periopar;
        }

        Surf.Bounds(out double Ufirst, out double Ulast, out double Vfirst, out double Vlast);

        Geom2d_Curve? PC = TopOpeBRepTool_2d.FC2D_CurveOnSurface(E, F, out double _, out double _, out double _);
        if (PC is null)
        {
            throw new ProgramErrorException("ShapeTool::PeriodizeParameter : no 2d curve");
        }

        Type TheType = PC.DynamicType();
        if (TheType == typeof(Geom2d_Line))
        {
            Geom2d_Line HL = (Geom2d_Line)PC;
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
    public static bool ShapesSameOriented(in TopoDS_Shape S1, in TopoDS_Shape S2)
    {
        bool so = true;

        bool sam = S1.IsSame(S2);
        if (sam)
        {
            TopAbs_Orientation o1 = S1.Orientation();
            TopAbs_Orientation o2 = S2.Orientation();
            if ((o1 == TopAbs_FORWARD || o1 == TopAbs_REVERSED)
                && (o2 == TopAbs_FORWARD || o2 == TopAbs_REVERSED))
            {
                so = (o1 == o2);
                return so;
            }
        }

        TopAbs_ShapeEnum t1 = S1.ShapeType(), t2 = S2.ShapeType();
        if ((t1 == TopAbs_SOLID) && (t2 == TopAbs_SOLID))
        {
            so = true;
        }
        else if ((t1 == TopAbs_FACE) && (t2 == TopAbs_FACE))
        {
            so = FacesSameOriented(S1, S2);
        }
        else if ((t1 == TopAbs_EDGE) && (t2 == TopAbs_EDGE))
        {
            so = EdgesSameOriented(S1, S2);
        }
        else if ((t1 == TopAbs_VERTEX) && (t2 == TopAbs_VERTEX))
        {
            TopAbs_Orientation o1 = S1.Orientation();
            TopAbs_Orientation o2 = S2.Orientation();
            if (o1 == TopAbs_EXTERNAL || o1 == TopAbs_INTERNAL || o2 == TopAbs_EXTERNAL
                || o2 == TopAbs_INTERNAL)
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
    public static bool SurfacesSameOriented(BRepAdaptor_Surface S1, BRepAdaptor_Surface Sref)
    {
        BRepAdaptor_Surface S2 = Sref;
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

            Geom_Surface HS2 = S2.GeomSurfaceTransformed()!;
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
    public static bool FacesSameOriented(in TopoDS_Shape S1, in TopoDS_Shape Sref)
    {
        TopoDS_Shape S2 = Sref;
        TopoDS_Face F1 = TopoDS.Face(S1);
        TopoDS_Face F2 = TopoDS.Face(S2);
        TopAbs_Orientation o1 = F1.Orientation();
        TopAbs_Orientation o2 = F2.Orientation();
        if (o1 == TopAbs_EXTERNAL || o1 == TopAbs_INTERNAL || o2 == TopAbs_EXTERNAL || o2 == TopAbs_INTERNAL)
        {
            return true;
        }

        bool computerestriction = false;
        BRepAdaptor_Surface BAS1 = new(F1, computerestriction);
        BRepAdaptor_Surface BAS2 = new(F2, computerestriction);
        bool so = F1.IsSame(F2) || SurfacesSameOriented(BAS1, BAS2);
        bool b = so;
        if (o1 != o2)
        {
            b = !so;
        }
        return b;
    }

    /// <summary>Whether two lines have tangents pointing the same way; true for other curves (not implemented in OCCT).</summary>
    public static bool CurvesSameOriented(BRepAdaptor_Curve C1, BRepAdaptor_Curve Cref)
    {
        BRepAdaptor_Curve C2 = Cref;
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
    public static bool EdgesSameOriented(in TopoDS_Shape S1, in TopoDS_Shape Sref)
    {
        TopoDS_Shape S2 = Sref;
        TopoDS_Edge E1 = TopoDS.Edge(S1);
        TopoDS_Edge E2 = TopoDS.Edge(S2);
        TopAbs_Orientation o1 = E1.Orientation();
        TopAbs_Orientation o2 = E2.Orientation();
        if (o1 == TopAbs_EXTERNAL || o1 == TopAbs_INTERNAL || o2 == TopAbs_EXTERNAL || o2 == TopAbs_INTERNAL)
        {
            return true;
        }
        BRepAdaptor_Curve BAC1 = new(E1);
        BRepAdaptor_Curve BAC2 = new(E2);
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
    public static double EdgeData(BRepAdaptor_Curve BAC, double P, ref Direction3D T, ref Direction3D N, ref double C)
    {
        double tol = global::Kerf.Engine.Foundation.Precision.Angular();

        BRepLProp_CLProps BL = new(BAC, P, 2, tol);
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
    public static double EdgeData(in TopoDS_Shape E, double P, ref Direction3D T, ref Direction3D N, ref double C)
    {
        BRepAdaptor_Curve BAC = new(TopoDS.Edge(E));
        double d = EdgeData(BAC, P, ref T, ref N, ref C);
        return d;
    }

    /// <summary>The 3D length matching the 2D tolerance <paramref name="Tol2d"/> along U.</summary>
    public static double Resolution3dU(Geom_Surface SU, double Tol2d)
    {
        GeomAdaptor_Surface GAS = new(SU);
        double r3dunit = 0.00001; // petite valeur (1.0 -> RangeError sur un tore)
        double ru = GAS.UResolution(r3dunit);
        double r3du = r3dunit * (Tol2d / ru);
        return r3du;
    }

    /// <summary>The 3D length matching the 2D tolerance <paramref name="Tol2d"/> along V.</summary>
    public static double Resolution3dV(Geom_Surface SU, double Tol2d)
    {
        GeomAdaptor_Surface GAS = new(SU);
        double r3dunit = 0.00001; // petite valeur (1.0 -> RangeError sur un tore)
        double rv = GAS.VResolution(r3dunit);
        double r3dv = r3dunit * (Tol2d / rv);
        return r3dv;
    }

    /// <summary>The larger of <see cref="Resolution3dU"/> and <see cref="Resolution3dV"/>.</summary>
    public static double Resolution3d(Geom_Surface SU, double Tol2d)
    {
        double ru = Resolution3dU(SU, Tol2d);
        double rv = Resolution3dV(SU, Tol2d);
        double r = Max(ru, rv);
        return r;
    }

    /// <summary>As the other overload, for the face's surface.</summary>
    public static double Resolution3d(in TopoDS_Face F, double Tol2d)
    {
        Geom_Surface SU = BRep_Tool.Surface(F, out Placement _)!;
        double r = Resolution3d(SU, Tol2d);
        return r;
    }
}
