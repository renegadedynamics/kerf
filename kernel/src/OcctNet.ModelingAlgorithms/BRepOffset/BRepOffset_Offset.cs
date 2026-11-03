// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Offset.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Offset.lxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Offset.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepOffset_Status;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Builds the offset of a face, or the pipe or sphere joining offsets along an edge or at a vertex.</summary>
/// <remarks>
/// <para>
/// The offset of a face (<see cref="Init(in TopoDS_Face, double, bool, JoinKind)"/>) only reads the face: its
/// edges and vertices are new, but for those taken from <c>Created</c>, the images of its edges and vertices in the
/// offsets of other faces. As in OCCT, the offset writes pcurves on the new face and ranges into those
/// <c>Created</c> edges, in place: they are the 3D offset's own working edges and must be unfrozen (a frozen one throws
/// <see cref="TopoDS_LockedShape"/> at the first write).
/// </para>
/// <para>
/// The pipe along an edge and the sphere at a vertex write pcurves, ranges and flags into the edges they are given
/// (<c>Edge1</c>, <c>Edge2</c>, <c>FirstEdge</c>, <c>LastEdge</c>, the edges of <c>LEdge</c>), so that the pipe or
/// sphere shares them with the faces' offsets. When none of these holds a frozen shape (the 3D offset's working
/// edges), they are written in place, as in OCCT. When one does (a caller's shapes), they are copied at the entry
/// (<see cref="BRepTools_CopyAtEntry"/>): the face is built on the copies, and the caller's sub-shapes the
/// construction left unchanged (vertices whose tolerance didn't grow) are given back in it.
/// </para>
/// <para>
/// <see cref="Face"/> and <see cref="Generated"/> freeze what they return (a publication point); ported code reads
/// <see cref="InternalFace"/> and <see cref="InternalGenerated"/>, which don't.
/// </para>
/// </remarks>
public sealed class BRepOffset_Offset
{
    private TopoDS_Shape myShape = new();
    private BRepOffset_Status myStatus = BRepOffset_Unknown;
    private TopoDS_Face myFace = new();
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myMap = new(TopTools_ShapeMapHasher.Instance);

    // OCCT_DEBUG statics (Affich, NbOFFSET) and their blocks are left out.

    private static Point3D GetFarestCorner(in TopoDS_Wire aWire)
    {
        var Vertices = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aWire, TopAbs_VERTEX, Vertices);

        double MaxDist = 0.0;
        Point3D thePoint = new();
        for (int i = 1; i <= Vertices.Extent(); i++)
        {
            for (int j = 1; j <= Vertices.Extent(); j++)
            {
                TopoDS_Vertex V1 = TopoDS.Vertex(Vertices[i]);
                TopoDS_Vertex V2 = TopoDS.Vertex(Vertices[j]);
                Point3D P1 = BRep_Tool.Pnt(V1);
                Point3D P2 = BRep_Tool.Pnt(V2);
                double aDist = P1.SquareDistance(P2);
                if (aDist > MaxDist)
                {
                    MaxDist = aDist;
                    thePoint = P1;
                }
            }
        }

        return thePoint;
    }

    //=================================================================================================

    private static void UpdateEdge(in TopoDS_Edge E, Geom_Curve? C, in Placement L, double Tol)
    {
        // Cut curves to avoid copies in the extensions.
        var B = new BRep_Builder();
        if (C is Geom_TrimmedCurve BC)
        {
            B.UpdateEdge(E, BC.BasisCurve(), L, Tol);
        }
        else
        {
            B.UpdateEdge(E, C, L, Tol);
        }
    }

    //=================================================================================================

    private static void UpdateEdge(in TopoDS_Edge E, Geom2d_Curve? C, in TopoDS_Face F, double Tol)
    {
        // Cut curves to avoid copies in the extensions.
        var B = new BRep_Builder();
        if (C is Geom2d_TrimmedCurve BC)
        {
            B.UpdateEdge(E, BC.BasisCurve(), F, Tol);
        }
        else
        {
            B.UpdateEdge(E, C, F, Tol);
        }
    }

    //=================================================================================================

    private static void UpdateEdge(in TopoDS_Edge E, Geom2d_Curve? C1, Geom2d_Curve? C2, in TopoDS_Face F, double Tol)
    {
        // Cut curves to avoid copies in the extensions.
        var B = new BRep_Builder();
        Geom2d_Curve? NC1, NC2;
        if (C1 is Geom2d_TrimmedCurve BC1)
        {
            NC1 = BC1.BasisCurve();
        }
        else
        {
            NC1 = C1;
        }
        if (C2 is Geom2d_TrimmedCurve BC2)
        {
            NC2 = BC2.BasisCurve();
        }
        else
        {
            NC2 = C2;
        }
        B.UpdateEdge(E, NC1, NC2, F, Tol);
    }

    //=======================================================================
    // function : ComputeCurve3d
    // purpose  : Particular case of Curve On Surface.
    //=======================================================================

    private static void ComputeCurve3d(in TopoDS_Edge Edge, Geom2d_Curve Curve, Geom_Surface Surf, in Placement Loc,
                                       double Tol)
    {
        // try to find the particular case
        // if not found call BRepLib::BuildCurve3d

        bool IsComputed = false;

        // Search only isos on analytic surfaces.
        var C = new Geom2dAdaptor_Curve(Curve);
        var S = new GeomAdaptor_Surface(Surf);
        CurveKind CTy = C.GetType();
        SurfaceKind STy = S.GetType();
        var TheBuilder = new BRep_Builder();

        if (STy != Plane)
        { // if plane buildcurve3d manage KPart
            if (CTy == Line)
            {
                Direction2D D = C.Line().Direction();
                if (D.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                { // Iso V.
                    if (STy == SurfaceKind.Sphere)
                    {
                        Point2D P = C.Line().Location();
                        if (Math.Abs(Math.Abs(P.Y()) - M_PI / 2.0) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            TheBuilder.Degenerated(Edge, true);
                        }
                        else
                        {
                            Sphere Sph = S.Sphere();
                            CoordinateSystem Axis = Sph.Position();
                            Circle3D Ci = global::Kerf.Engine.Geometry.ElSLib.SphereVIso(Axis, Sph.Radius(), P.Y());
                            Direction3D DRev = Axis.XDirection().Crossed(Axis.YDirection());
                            var AxeRev = new Axis3D(Axis.Location(), DRev);
                            Ci.Rotate(AxeRev, P.X());
                            Geom_Circle Circle = new Geom_Circle(Ci);
                            if (D.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                Circle.Reverse();
                            }
                            UpdateEdge(Edge, Circle, Loc, Tol);
                        }
                        IsComputed = true;
                    }
                    else if (STy == SurfaceKind.Cylinder)
                    {
                        Cylinder Cyl = S.Cylinder();
                        Point2D P = C.Line().Location();
                        CoordinateSystem Axis = Cyl.Position();
                        Circle3D Ci = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(Axis, Cyl.Radius(), P.Y());
                        Direction3D DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        var AxeRev = new Axis3D(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.X());
                        Geom_Circle Circle = new Geom_Circle(Ci);
                        if (D.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Circle.Reverse();
                        }
                        UpdateEdge(Edge, Circle, Loc, Tol);
                        IsComputed = true;
                    }
                    else if (STy == SurfaceKind.Cone)
                    {
                        Cone Cone = S.Cone();
                        Point2D P = C.Line().Location();
                        CoordinateSystem Axis = Cone.Position();
                        Circle3D Ci = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(Axis, Cone.RefRadius(), Cone.SemiAngle(), P.Y());
                        Direction3D DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        var AxeRev = new Axis3D(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.X());
                        Geom_Circle Circle = new Geom_Circle(Ci);
                        if (D.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Circle.Reverse();
                        }
                        UpdateEdge(Edge, Circle, Loc, Tol);
                        IsComputed = true;
                    }
                    else if (STy == SurfaceKind.Torus)
                    {
                        Torus Tore = S.Torus();
                        Point2D P = C.Line().Location();
                        CoordinateSystem Axis = Tore.Position();
                        Circle3D Ci = global::Kerf.Engine.Geometry.ElSLib.TorusVIso(Axis, Tore.MajorRadius(), Tore.MinorRadius(), P.Y());
                        Direction3D DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        var AxeRev = new Axis3D(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.X());
                        Geom_Circle Circle = new Geom_Circle(Ci);
                        if (D.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Circle.Reverse();
                        }
                        UpdateEdge(Edge, Circle, Loc, Tol);
                        IsComputed = true;
                    }
                }
                else if (D.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                { // Iso U.
                    if (STy == SurfaceKind.Sphere)
                    {
                        Sphere Sph = S.Sphere();
                        Point2D P = C.Line().Location();
                        CoordinateSystem Axis = Sph.Position();
                        // calculate iso 0.
                        Circle3D Ci = global::Kerf.Engine.Geometry.ElSLib.SphereUIso(Axis, Sph.Radius(), 0.0);

                        // set to sameparameter (rotation of circle - offset of Y)
                        Direction3D DRev = Axis.XDirection().Crossed(Axis.Direction());
                        var AxeRev = new Axis3D(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.Y());

                        // transformation en iso U ( = P.X())
                        DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        AxeRev = new Axis3D(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.X());
                        Geom_Circle Circle = new Geom_Circle(Ci);

                        if (D.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Circle.Reverse();
                        }
                        UpdateEdge(Edge, Circle, Loc, Tol);
                        IsComputed = true;
                    }
                    else if (STy == SurfaceKind.Cylinder)
                    {
                        Cylinder Cyl = S.Cylinder();
                        Point2D P = C.Line().Location();
                        Line3D L = global::Kerf.Engine.Geometry.ElSLib.CylinderUIso(Cyl.Position(), Cyl.Radius(), P.X());
                        var Tr = new Vector3D(L.Direction());
                        Tr.Multiply(P.Y());
                        L.Translate(Tr);
                        Geom_Line Line = new Geom_Line(L);
                        if (D.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Line.Reverse();
                        }
                        UpdateEdge(Edge, Line, Loc, Tol);
                        IsComputed = true;
                    }
                    else if (STy == SurfaceKind.Cone)
                    {
                        Cone Cone = S.Cone();
                        Point2D P = C.Line().Location();
                        Line3D L = global::Kerf.Engine.Geometry.ElSLib.ConeUIso(Cone.Position(), Cone.RefRadius(), Cone.SemiAngle(), P.X());
                        var Tr = new Vector3D(L.Direction());
                        Tr.Multiply(P.Y());
                        L.Translate(Tr);
                        Geom_Line Line = new Geom_Line(L);
                        if (D.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Line.Reverse();
                        }
                        UpdateEdge(Edge, Line, Loc, Tol);
                        IsComputed = true;
                    }
                    else if (STy == SurfaceKind.Torus)
                    {
                        Torus Tore = S.Torus();
                        Point2D P = C.Line().Location();
                        CoordinateSystem Axis = Tore.Position();
                        Circle3D Ci = global::Kerf.Engine.Geometry.ElSLib.TorusUIso(Axis, Tore.MajorRadius(), Tore.MinorRadius(), P.X());
                        Ci.Rotate(Ci.Axis(), P.Y());
                        Geom_Circle Circle = new Geom_Circle(Ci);

                        if (D.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Circle.Reverse();
                        }
                        UpdateEdge(Edge, Circle, Loc, Tol);
                        IsComputed = true;
                    }
                }
            }
        }
        else
        { // Cas Plan
            Geom_Curve C3d = GeomAPI.To3d(Curve, S.Plane());
            UpdateEdge(Edge, C3d, Loc, Tol);
            IsComputed = true;
        }
        if (!IsComputed)
        {
            // BRepLib::BuildCurves3d(Edge,Tol);
            // Les Courbes 3d des edges dans le cas general ne sont calcules que si
            //  necessaire
            // ie dans les tuyaux et les bouchons ..
            //  dans la derniere etapes de MakeShells on reconstruira les courbes3d
            //  des edges du resultat qui n en ont pas.
        }
    }

    //=================================================================================================

    /// <summary>An empty offset.</summary>
    public BRepOffset_Offset()
    {
    }

    //=================================================================================================

    /// <summary>The offset of <paramref name="Face"/> by <paramref name="Offset"/>.</summary>
    public BRepOffset_Offset(in TopoDS_Face Face, double Offset, bool OffsetOutside = true,
                             JoinKind JoinType = Arc) =>
        Init(Face, Offset, OffsetOutside, JoinType);

    //=================================================================================================

    /// <summary>
    /// The offset of <paramref name="Face"/>, sharing the edges already generated from other faces:
    /// <c>Created(E) = E'</c>, where E is an edge (or vertex) of the face and E' its image in the offset of another face
    /// sharing E with at least G1 continuity (so two tangent faces generate one edge, no pipe). The edges E' are written
    /// into and must be unfrozen.
    /// </summary>
    public BRepOffset_Offset(in TopoDS_Face Face, double Offset, NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> Created,
                             bool OffsetOutside = true, JoinKind JoinType = Arc) =>
        Init(Face, Offset, Created, OffsetOutside, JoinType);

    //=================================================================================================

    /// <summary>The pipe of radius <paramref name="Offset"/> along <paramref name="Path"/> between two edges.</summary>
    public BRepOffset_Offset(in TopoDS_Edge Path, in TopoDS_Edge Edge1, in TopoDS_Edge Edge2, double Offset,
                             bool Polynomial = false, double Tol = 1.0e-4, Continuity Conti = C1) =>
        Init(Path, Edge1, Edge2, Offset, Polynomial, Tol, Conti);

    //=================================================================================================

    /// <summary>The pipe along <paramref name="Path"/> between two edges, closed by the given end edges.</summary>
    public BRepOffset_Offset(in TopoDS_Edge Path, in TopoDS_Edge Edge1, in TopoDS_Edge Edge2, double Offset,
                             in TopoDS_Edge FirstEdge, in TopoDS_Edge LastEdge, bool Polynomial = false,
                             double Tol = 1.0e-4, Continuity Conti = C1) =>
        Init(Path, Edge1, Edge2, Offset, FirstEdge, LastEdge, Polynomial, Tol, Conti);

    //=================================================================================================

    /// <summary>
    /// The sphere of radius <paramref name="Offset"/> at <paramref name="Vertex"/> bounded by the edges
    /// <paramref name="LEdge"/>. <paramref name="Tol"/> and <paramref name="Conti"/> are used only if
    /// <paramref name="Polynomial"/> (for the approximation).
    /// </summary>
    public BRepOffset_Offset(in TopoDS_Vertex Vertex, NCollection_List<TopoDS_Shape> LEdge, double Offset,
                             bool Polynomial = false, double Tol = 1.0e-4, Continuity Conti = C1) =>
        Init(Vertex, LEdge, Offset, Polynomial, Tol, Conti);

    //=================================================================================================

    /// <summary>Builds the offset of <paramref name="Face"/> by <paramref name="Offset"/>.</summary>
    public void Init(in TopoDS_Face Face, double Offset, bool OffsetOutside = true,
                     JoinKind JoinType = Arc)
    {
        var Empty = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        Init(Face, Offset, Empty, OffsetOutside, JoinType);
    }

    //=================================================================================================

    /// <summary>
    /// Builds the offset of <paramref name="Face"/>, sharing the edges already generated from other faces (see the
    /// constructor). The edges of <paramref name="Created"/> are written into and must be unfrozen.
    /// </summary>
    public void Init(in TopoDS_Face Face, double Offset, NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> Created,
                     bool OffsetOutside = true, JoinKind JoinType = Arc)
    {
        myShape = Face;
        double myOffset = Offset;
        if (Face.Orientation() == TopAbs_REVERSED)
        {
            myOffset *= -1.0;
        }

        Geom_Surface S = BRep_Tool.Surface(Face, out Placement L)!;
        // On detrime les surfaces, evite des recopies dans les extensions.
        if (S is Geom_RectangularTrimmedSurface RT)
        {
            S = RT.BasisSurface();
        }
        bool IsTransformed = false;
        if ((S is Geom_BSplineSurface || S is Geom_SurfaceOfLinearExtrusion || S is Geom_SurfaceOfRevolution
             || S is Geom_OffsetSurface)
            && !L.IsIdentity)
        {
            S = (Geom_Surface)S.Copy();
            S.Transform(L.Transformation());
            IsTransformed = true;
        }
        // particular case of cone
        if (S is Geom_ConicalSurface Co)
        {
            Point3D Apex = Co.Apex();
            global::Kerf.Engine.Geometry.ElSLib.Parameters(Co.Cone(), Apex, out double Uc, out double Vc);
            BRepTools.UVBounds(Face, out double UU1, out double UU2, out double VV1, out double VV2);
            if (VV2 < Vc && Co.SemiAngle() > 0)
            {
                myOffset *= -1;
            }
            else if (VV1 > Vc && Co.SemiAngle() < 0)
            {
                myOffset *= -1;
            }
            if (!Co.Position().Direct())
            {
                myOffset *= -1;
            }
        }

        Geom_Surface TheSurf = BRepOffset.Surface(S, myOffset, out myStatus);

        // processing offsets of faces with possible degenerated edges
        bool UminDegen = false;
        bool UmaxDegen = false;
        bool VminDegen = false;
        bool VmaxDegen = false;
        bool UisoDegen = false;
        Point3D MinApex = new(), MaxApex = new();
        bool HasSingularity = false;
        double fpar, lpar;
        BRepTools.UVBounds(Face, out double uf1, out double uf2, out double vf1, out double vf2);
        if ((!OffsetOutside || JoinType != Arc)
            && (TheSurf.GetType() == typeof(Geom_ConicalSurface) || TheSurf.GetType() == typeof(Geom_OffsetSurface)))
        {
            var DegEdges = new NCollection_Sequence<TopoDS_Shape>();
            var Explo = new TopExp_Explorer(Face, TopAbs_EDGE);
            for (; Explo.More(); Explo.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(Explo.Current());

                if (BRep_Tool.Degenerated(anEdge))
                {
                    Geom2d_Curve c2d = BRep_Tool.CurveOnSurface(anEdge, Face, out double aF, out double aL)!;

                    Point2D aFPnt2d = c2d.Value(aF), aLPnt2d = c2d.Value(aL);

                    Point3D aFPnt = S.Value(aFPnt2d.X(), aFPnt2d.Y()),
                           aLPnt = S.Value(aLPnt2d.X(), aLPnt2d.Y());

                    //  aFPnt.SquareDistance(aLPnt) > Precision::SquareConfusion() -
                    // is a sufficient condition of troubles: non-singular case, but edge is degenerated.
                    // So, normal handling of degenerated edges is not applicable in case of non-singular point.
                    if (aFPnt.SquareDistance(aLPnt) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        DegEdges.Append(anEdge);
                    }
                }
            }
            if (!DegEdges.IsEmpty())
            {
                const double TolApex = 1.0e-5;
                // define the iso of singularity (u or v)
                TopoDS_Edge theDegEdge = TopoDS.Edge(DegEdges[1]);
                Geom2d_Curve aCurve = BRep_Tool.CurveOnSurface(theDegEdge, Face, out fpar, out lpar)!;
                Point2D fp2d = aCurve.Value(fpar);
                Point2D lp2d = aCurve.Value(lpar);
                if (Math.Abs(fp2d.X() - lp2d.X()) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    UisoDegen = true;
                }

                if (DegEdges.Length() == 2)
                {
                    if (UisoDegen)
                    {
                        UminDegen = true;
                        UmaxDegen = true;
                    }
                    else
                    {
                        VminDegen = true;
                        VmaxDegen = true;
                    }
                }
                else // DegEdges.Length() == 1
                {
                    if (UisoDegen)
                    {
                        if (Math.Abs(fp2d.X() - uf1) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            UminDegen = true;
                        }
                        else
                        {
                            UmaxDegen = true;
                        }
                    }
                    else
                    {
                        if (Math.Abs(fp2d.Y() - vf1) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            VminDegen = true;
                        }
                        else
                        {
                            VmaxDegen = true;
                        }
                    }
                }
                if (TheSurf.GetType() == typeof(Geom_ConicalSurface))
                {
                    Cone theCone = ((Geom_ConicalSurface)TheSurf).Cone();
                    Point3D apex = theCone.Apex();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(theCone, apex, out double Uapex, out double Vapex);
                    if (VminDegen)
                    {
                        TheSurf = new Geom_RectangularTrimmedSurface(TheSurf, uf1, uf2, Vapex, vf2);
                        MinApex = apex;
                        HasSingularity = true;
                    }
                    else if (VmaxDegen)
                    {
                        TheSurf = new Geom_RectangularTrimmedSurface(TheSurf, uf1, uf2, vf1, Vapex);
                        MaxApex = apex;
                        HasSingularity = true;
                    }
                }
                else // TheSurf->DynamicType() == STANDARD_TYPE(Geom_OffsetSurface)
                {
                    // OCCT.NET: the bounds of the extended surface are named aU1.. (C# forbids reusing u1.. below)
                    if (UminDegen)
                    {
                        Geom_Curve uiso = TheSurf.UIso(uf1);
                        if (BRepOffset_Tool.Gabarit(uiso) > TolApex)
                        {
                            Geom_Surface BasisSurf = ((Geom_OffsetSurface)TheSurf).BasisSurface();
                            Point3D Papex, Pfirst, Pquart, Pmid;
                            Papex = BasisSurf.Value(uf1, vf1);
                            Pfirst = TheSurf.Value(uf1, vf1);
                            Pquart = TheSurf.Value(uf1, 0.75 * vf1 + 0.25 * vf2);
                            Pmid = TheSurf.Value(uf1, 0.5 * (vf1 + vf2));
                            Vector3D DirApex = new Vector3D(Pfirst, Pquart) ^ new Vector3D(Pfirst, Pmid);
                            var LineApex = new Geom_Line(Papex, new Direction3D(DirApex));
                            Vector3D DirGeneratrix = BasisSurf.DN(uf1, vf1, 1, 0);
                            var LineGeneratrix = new Geom_Line(Pfirst, new Direction3D(DirGeneratrix));
                            var theExtrema = new GeomAPI_ExtremaCurveCurve(LineGeneratrix, LineApex);
                            theExtrema.NearestPoints(out Point3D Pint1, out Point3D Pint2);
                            double length = Pfirst.Distance(Pint1);
                            if (OffsetOutside)
                            {
                                Geom_BoundedSurface aSurf = new Geom_RectangularTrimmedSurface(TheSurf, uf1, uf2, vf1, vf2);
                                GeomLib.ExtendSurfByLength(ref aSurf, length, 1, true, false);
                                TheSurf = aSurf;
                                TheSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                                MinApex = TheSurf.Value(aU1, vf1);
                            }
                            else
                            {
                                Geom_Curve viso = TheSurf.VIso(vf1);
                                var Projector = new GeomAPI_ProjectPointOnCurve(Pint1, viso);
                                double NewFirstU = Projector.LowerDistanceParameter();
                                TheSurf = new Geom_RectangularTrimmedSurface(TheSurf, NewFirstU, uf2, vf1, vf2);
                                MinApex = TheSurf.Value(NewFirstU, vf1);
                            }
                            HasSingularity = true;
                        }
                    } // end of if (UminDegen)
                    if (UmaxDegen)
                    {
                        Geom_Curve uiso = TheSurf.UIso(uf2);
                        if (BRepOffset_Tool.Gabarit(uiso) > TolApex)
                        {
                            Geom_Surface BasisSurf = ((Geom_OffsetSurface)TheSurf).BasisSurface();
                            Point3D Papex, Pfirst, Pquart, Pmid;
                            Papex = BasisSurf.Value(uf2, vf1);
                            Pfirst = TheSurf.Value(uf2, vf1);
                            Pquart = TheSurf.Value(uf2, 0.75 * vf1 + 0.25 * vf2);
                            Pmid = TheSurf.Value(uf2, 0.5 * (vf1 + vf2));
                            Vector3D DirApex = new Vector3D(Pfirst, Pquart) ^ new Vector3D(Pfirst, Pmid);
                            var LineApex = new Geom_Line(Papex, new Direction3D(DirApex));
                            Vector3D DirGeneratrix = BasisSurf.DN(uf2, vf1, 1, 0);
                            var LineGeneratrix = new Geom_Line(Pfirst, new Direction3D(DirGeneratrix));
                            var theExtrema = new GeomAPI_ExtremaCurveCurve(LineGeneratrix, LineApex);
                            theExtrema.NearestPoints(out Point3D Pint1, out Point3D Pint2);
                            double length = Pfirst.Distance(Pint1);
                            if (OffsetOutside)
                            {
                                Geom_BoundedSurface aSurf = new Geom_RectangularTrimmedSurface(TheSurf, uf1, uf2, vf1, vf2);
                                GeomLib.ExtendSurfByLength(ref aSurf, length, 1, true, true);
                                TheSurf = aSurf;
                                TheSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                                MaxApex = TheSurf.Value(aU2, vf1);
                            }
                            else
                            {
                                Geom_Curve viso = TheSurf.VIso(vf1);
                                var Projector = new GeomAPI_ProjectPointOnCurve(Pint1, viso);
                                double NewLastU = Projector.LowerDistanceParameter();
                                TheSurf = new Geom_RectangularTrimmedSurface(TheSurf, uf1, NewLastU, vf1, vf2);
                                MaxApex = TheSurf.Value(NewLastU, vf1);
                            }
                            HasSingularity = true;
                        }
                    } // end of if (UmaxDegen)
                    if (VminDegen)
                    {
                        Geom_Curve viso = TheSurf.VIso(vf1);
                        if (BRepOffset_Tool.Gabarit(viso) > TolApex)
                        {
                            Geom_Surface BasisSurf = ((Geom_OffsetSurface)TheSurf).BasisSurface();
                            Point3D Papex, Pfirst, Pquart, Pmid;
                            Papex = BasisSurf.Value(uf1, vf1);
                            Pfirst = TheSurf.Value(uf1, vf1);
                            Pquart = TheSurf.Value(0.75 * uf1 + 0.25 * uf2, vf1);
                            Pmid = TheSurf.Value(0.5 * (uf1 + uf2), vf1);
                            Vector3D DirApex = new Vector3D(Pfirst, Pquart) ^ new Vector3D(Pfirst, Pmid);
                            var LineApex = new Geom_Line(Papex, new Direction3D(DirApex));
                            Vector3D DirGeneratrix = BasisSurf.DN(uf1, vf1, 0, 1);
                            var LineGeneratrix = new Geom_Line(Pfirst, new Direction3D(DirGeneratrix));
                            var theExtrema = new GeomAPI_ExtremaCurveCurve(LineGeneratrix, LineApex);
                            theExtrema.NearestPoints(out Point3D Pint1, out Point3D Pint2);
                            double length = Pfirst.Distance(Pint1);
                            if (OffsetOutside)
                            {
                                Geom_BoundedSurface aSurf = new Geom_RectangularTrimmedSurface(TheSurf, uf1, uf2, vf1, vf2);
                                GeomLib.ExtendSurfByLength(ref aSurf, length, 1, false, false);
                                TheSurf = aSurf;
                                TheSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                                MinApex = TheSurf.Value(uf1, aV1);
                            }
                            else
                            {
                                Geom_Curve uiso = TheSurf.UIso(uf1);
                                var Projector = new GeomAPI_ProjectPointOnCurve(Pint1, uiso);
                                double NewFirstV = Projector.LowerDistanceParameter();
                                TheSurf = new Geom_RectangularTrimmedSurface(TheSurf, uf1, uf2, NewFirstV, vf2);
                                MinApex = TheSurf.Value(uf1, NewFirstV);
                                // TheSurf = new Geom_RectangularTrimmedSurface(TheSurf, uf1, uf2, vf1+length, vf2);
                                // MinApex = TheSurf->Value( uf1, vf1+length );
                            }
                            HasSingularity = true;
                        }
                    } // end of if (VminDegen)
                    if (VmaxDegen)
                    {
                        Geom_Curve viso = TheSurf.VIso(vf2);
                        if (BRepOffset_Tool.Gabarit(viso) > TolApex)
                        {
                            Geom_Surface BasisSurf = ((Geom_OffsetSurface)TheSurf).BasisSurface();
                            Point3D Papex, Pfirst, Pquart, Pmid;
                            Papex = BasisSurf.Value(uf1, vf2);
                            Pfirst = TheSurf.Value(uf1, vf2);
                            Pquart = TheSurf.Value(0.75 * uf1 + 0.25 * uf2, vf2);
                            Pmid = TheSurf.Value(0.5 * (uf1 + uf2), vf2);
                            Vector3D DirApex = new Vector3D(Pfirst, Pquart) ^ new Vector3D(Pfirst, Pmid);
                            var LineApex = new Geom_Line(Papex, new Direction3D(DirApex));
                            Vector3D DirGeneratrix = BasisSurf.DN(uf1, vf2, 0, 1);
                            var LineGeneratrix = new Geom_Line(Pfirst, new Direction3D(DirGeneratrix));
                            var theExtrema = new GeomAPI_ExtremaCurveCurve(LineGeneratrix, LineApex);
                            theExtrema.NearestPoints(out Point3D Pint1, out Point3D Pint2);
                            double length = Pfirst.Distance(Pint1);
                            if (OffsetOutside)
                            {
                                Geom_BoundedSurface aSurf = new Geom_RectangularTrimmedSurface(TheSurf, uf1, uf2, vf1, vf2);
                                GeomLib.ExtendSurfByLength(ref aSurf, length, 1, false, true);
                                TheSurf = aSurf;
                                TheSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                                MaxApex = TheSurf.Value(uf1, aV2);
                            }
                            else
                            {
                                Geom_Curve uiso = TheSurf.UIso(uf1);
                                var Projector = new GeomAPI_ProjectPointOnCurve(Pint1, uiso);
                                double NewLastV = Projector.LowerDistanceParameter();
                                TheSurf = new Geom_RectangularTrimmedSurface(TheSurf, uf1, uf2, vf1, NewLastV);
                                MaxApex = TheSurf.Value(uf1, NewLastV);
                                // TheSurf = new Geom_RectangularTrimmedSurface(TheSurf, uf1, uf2, vf1, vf2-length);
                                // MaxApex = TheSurf->Value( uf1, vf2-length );
                            }
                            HasSingularity = true;
                        }
                    } // end of if (VmaxDegen)
                } // end of else (case of Geom_OffsetSurface)
            } // end of if (!DegEdges.IsEmpty())
        } // end of processing offsets of faces with possible degenerated edges

        // find the PCurves of the edges of <Faces>

        var myBuilder = new BRep_Builder();
        myBuilder.MakeFace(ref myFace);
        if (!IsTransformed)
        {
            myBuilder.UpdateFace(myFace, TheSurf, L, BRep_Tool.Tolerance(Face));
        }
        else
        {
            myBuilder.UpdateFace(myFace, TheSurf, new Placement(), BRep_Tool.Tolerance(Face));
        }

        var MapSS = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);

        // mise a jour de la map sur les vertex deja crees
        TopoDS_Shape aLocalShapeOriented = Face.Oriented(TopAbs_FORWARD);
        TopoDS_Face CurFace = TopoDS.Face(aLocalShapeOriented);
        //  TopoDS_Face CurFace = TopoDS::Face(Face.Oriented(TopAbs_FORWARD));

        var VonDegen = new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TheSurf.Bounds(out double u1, out double u2, out double v1, out double v2);

        var exp = new TopExp_Explorer(CurFace, TopAbs_EDGE);
        for (; exp.More(); exp.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(exp.Current());
            TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            if (HasSingularity && BRep_Tool.Degenerated(E))
            {
                VonDegen.Add(V1);
            }
            if (Created.IsBound(E))
            {
                TopoDS_Edge OE = TopoDS.Edge(Created[E]);
                TopExp.Vertices(OE, out TopoDS_Vertex OV1, out TopoDS_Vertex OV2);
                if (!MapSS.IsBound(V1))
                {
                    MapSS.Bind(V1, OV1);
                }
                if (!MapSS.IsBound(V2))
                {
                    MapSS.Bind(V2, OV2);
                }
            }
            if (Created.IsBound(V1))
            {
                if (!MapSS.IsBound(V1))
                {
                    MapSS.Bind(V1, Created[V1]);
                }
            }
            if (Created.IsBound(V2))
            {
                if (!MapSS.IsBound(V2))
                {
                    MapSS.Bind(V2, Created[V2]);
                }
            }
        }

        var expw = new TopExp_Explorer(CurFace, TopAbs_WIRE);
        for (; expw.More(); expw.Next())
        {
            TopoDS_Wire W = TopoDS.Wire(expw.Current());
            var expe = new TopExp_Explorer(W.Oriented(TopAbs_FORWARD), TopAbs_EDGE);
            TopoDS_Wire OW = new();
            myBuilder.MakeWire(ref OW);
            for (; expe.More(); expe.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(expe.Current());
                TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                Point2D P2d1 = new(), P2d2 = new();
                Point3D P1, P2;
                double vstart, vend;
                double f, l;
                Geom2d_Curve C2d = BRep_Tool.CurveOnSurface(E, CurFace, out f, out l)!;
                TopoDS_Edge OE = new();
                if (MapSS.IsBound(E) && !VonDegen.Contains(V1) && !VonDegen.Contains(V2))
                { // c`est un edge de couture
                    OE = TopoDS.Edge(MapSS[E]);
                    TopoDS_Shape aLocalShape = E.Reversed();
                    Geom2d_Curve? C2d_1 = BRep_Tool.CurveOnSurface(TopoDS.Edge(aLocalShape), CurFace, out f, out l);
                    //	occ::handle<Geom2d_Curve> C2d_1 =
                    //	  BRep_Tool::CurveOnSurface(TopoDS::Edge(E.Reversed()),CurFace,f,l);
                    if (E.Orientation() == TopAbs_FORWARD)
                    {
                        UpdateEdge(OE, C2d, C2d_1, myFace, BRep_Tool.Tolerance(E));
                    }
                    else
                    {
                        UpdateEdge(OE, C2d_1, C2d, myFace, BRep_Tool.Tolerance(E));
                    }
                    myBuilder.Range(OE, f, l);
                }
                else
                {
                    TopoDS_Shape aLocalShape = E.Oriented(TopAbs_FORWARD);
                    TopoDS_Edge Eforward = TopoDS.Edge(aLocalShape);
                    P2d1 = C2d.Value(BRep_Tool.Parameter(V1, Eforward, CurFace));
                    P2d2 = C2d.Value(BRep_Tool.Parameter(V2, Eforward, CurFace));
                    if (VonDegen.Contains(V1))
                    {
                        if (Math.Abs(P2d1.Y() - vf1) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            P1 = MinApex;
                            vstart = v1;
                        }
                        else
                        {
                            P1 = MaxApex;
                            vstart = v2;
                        }
                    }
                    else
                    {
                        TheSurf.D0(P2d1.X(), P2d1.Y(), out P1);
                        if (!L.IsIdentity && !IsTransformed)
                        {
                            P1.Transform(L.Transformation());
                        }
                        vstart = P2d1.Y();
                    }
                    if (VonDegen.Contains(V2))
                    {
                        if (Math.Abs(P2d2.Y() - vf1) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            P2 = MinApex;
                            vend = v1;
                        }
                        else
                        {
                            P2 = MaxApex;
                            vend = v2;
                        }
                    }
                    else
                    {
                        TheSurf.D0(P2d2.X(), P2d2.Y(), out P2);
                        if (!L.IsIdentity && !IsTransformed)
                        {
                            P2.Transform(L.Transformation());
                        }
                        vend = P2d2.Y();
                    }
                    // E a-t-il ume image dans la Map des Created ?
                    if (Created.IsBound(E))
                    {
                        OE = TopoDS.Edge(Created[E]);
                    }
                    else if (MapSS.IsBound(E))
                    { // seam edge
                        OE = TopoDS.Edge(MapSS[E]);
                    }
                    else
                    {
                        myBuilder.MakeEdge(ref OE);
                        TopoDS_Vertex OV1 = new(), OV2 = new();
                        if (MapSS.IsBound(V1))
                        {
                            OV1 = TopoDS.Vertex(MapSS[V1]);
                        }
                        else
                        {
                            myBuilder.MakeVertex(ref OV1);
                            myBuilder.UpdateVertex(OV1, P1, BRep_Tool.Tolerance(V1));
                            MapSS.Bind(V1, OV1);
                        }
                        if (MapSS.IsBound(V2))
                        {
                            OV2 = TopoDS.Vertex(MapSS[V2]);
                        }
                        else
                        {
                            myBuilder.MakeVertex(ref OV2);
                            myBuilder.UpdateVertex(OV2, P2, BRep_Tool.Tolerance(V2));
                            MapSS.Bind(V2, OV2);
                        }
                        myBuilder.Add(OE, OV1.Oriented(V1.Orientation()));
                        myBuilder.Add(OE, OV2.Oriented(V2.Orientation()));
                        if (BRep_Tool.Degenerated(E))
                        {
                            myBuilder.Degenerated(OE, true);
                            // OCCT_DEBUG check of the degenerated edge's ends left out.
                        }
                    }
                    if (VonDegen.Contains(V1) || VonDegen.Contains(V2))
                    {
                        if (VonDegen.Contains(V1))
                        {
                            P2d1.SetY(vstart);
                        }
                        if (VonDegen.Contains(V2))
                        {
                            P2d2.SetY(vend);
                        }
                        C2d = new Geom2d_Line(P2d1, new Direction2D(new Vector2D(P2d1, P2d2)));
                        f = 0.0;
                        l = P2d1.Distance(P2d2);
                        if (MapSS.IsBound(E)) // seam edge
                        {
                            Geom2d_Curve? C2d_1 = BRep_Tool.CurveOnSurface(OE, myFace, out f, out l);
                            if (E.Orientation() == TopAbs_FORWARD)
                            {
                                UpdateEdge(OE, C2d, C2d_1, myFace, BRep_Tool.Tolerance(E));
                            }
                            else
                            {
                                UpdateEdge(OE, C2d_1, C2d, myFace, BRep_Tool.Tolerance(E));
                            }
                        }
                        else
                        {
                            UpdateEdge(OE, C2d, myFace, BRep_Tool.Tolerance(E));
                        }
                        // myBuilder.Range(OE,f,l);
                        myBuilder.Range(OE, myFace, f, l);
                        if (!BRep_Tool.Degenerated(E) && TheSurf.IsUClosed())
                        {
                            TopoDS_Shape aLocalShapeReversedE = E.Reversed();
                            Geom2d_Curve C2d_1 =
                                BRep_Tool.CurveOnSurface(TopoDS.Edge(aLocalShapeReversedE), CurFace, out f, out l)!;
                            P2d1 = C2d_1.Value(BRep_Tool.Parameter(V1, E, CurFace));
                            P2d2 = C2d_1.Value(BRep_Tool.Parameter(V2, E, CurFace));
                            if (VonDegen.Contains(V1))
                            {
                                P2d1.SetY(vstart);
                            }
                            if (VonDegen.Contains(V2))
                            {
                                P2d2.SetY(vend);
                            }
                            C2d_1 = new Geom2d_Line(P2d1, new Direction2D(new Vector2D(P2d1, P2d2)));
                            if (E.Orientation() == TopAbs_FORWARD)
                            {
                                UpdateEdge(OE, C2d, C2d_1, myFace, BRep_Tool.Tolerance(E));
                            }
                            else
                            {
                                UpdateEdge(OE, C2d_1, C2d, myFace, BRep_Tool.Tolerance(E));
                            }
                        }
                        /*
                        if (!BRep_Tool::Degenerated(E))
                          {
                        occ::handle<Geom_Line> theLine = new Geom_Line( P1, gp_Vec(P1, P2) );
                        myBuilder.UpdateEdge( OE, theLine, BRep_Tool::Tolerance(E) );
                          }
                        */
                    }
                    else
                    {
                        UpdateEdge(OE, C2d, myFace, BRep_Tool.Tolerance(E));
                        myBuilder.Range(OE, f, l);
                        // ComputeCurve3d(OE,C2d,TheSurf,L,BRep_Tool::Tolerance(E));
                    }
                    if (!BRep_Tool.Degenerated(OE))
                    {
                        ComputeCurve3d(OE, C2d, TheSurf, L, BRep_Tool.Tolerance(E));
                    }
                    MapSS.Bind(E, OE);
                }
                myBuilder.Add(OW, OE.Oriented(E.Orientation()));
            }
            myBuilder.Add(myFace, OW.Oriented(W.Orientation()));
        }

        myFace.Orientation(Face.Orientation());

        BRepTools.Update(myFace);
    }

    //=================================================================================================

    /// <summary>
    /// Builds the pipe of radius <paramref name="Offset"/> along <paramref name="Path"/> between
    /// <paramref name="Edge1"/> and <paramref name="Edge2"/>, which get pcurves on it.
    /// </summary>
    public void Init(in TopoDS_Edge Path, in TopoDS_Edge Edge1, in TopoDS_Edge Edge2, double Offset,
                     bool Polynomial = false, double Tol = 1.0e-4, Continuity Conti = C1)
    {
        TopoDS_Edge FirstEdge = new(), LastEdge = new();
        Init(Path, Edge1, Edge2, Offset, FirstEdge, LastEdge, Polynomial, Tol, Conti);
    }

    //=================================================================================================

    /// <summary>
    /// Builds the pipe of radius <paramref name="Offset"/> along <paramref name="Path"/> between
    /// <paramref name="Edge1"/> and <paramref name="Edge2"/>, closed at its ends by <paramref name="FirstEdge"/> and
    /// <paramref name="LastEdge"/> (or new edges when null). The given edges get pcurves on the pipe: in place when
    /// they are unfrozen, else on copies (see the class remarks).
    /// </summary>
    public void Init(in TopoDS_Edge Path, in TopoDS_Edge Edge1, in TopoDS_Edge Edge2, double Offset,
                     in TopoDS_Edge FirstEdge, in TopoDS_Edge LastEdge, bool Polynomial = false, double Tol = 1.0e-4,
                     Continuity Conti = C1)
    {
        // OCCT.NET: copy at entry (plan 2.2) of the edges written below, when one of them holds a frozen shape.
        BRepTools_CopyAtEntry? anEntry = CopyAtEntry([Edge1, Edge2, FirstEdge, LastEdge]);
        TopoDS_Edge aEdge1 = Copied(anEntry, Edge1);
        TopoDS_Edge aEdge2 = Copied(anEntry, Edge2);
        TopoDS_Edge aFirstEdge = Copied(anEntry, FirstEdge);
        TopoDS_Edge aLastEdge = Copied(anEntry, LastEdge);
        initPipe(Path, aEdge1, aEdge2, Offset, aFirstEdge, aLastEdge, Polynomial, Tol, Conti);
        if (anEntry is not null)
        {
            myFace = TopoDS.Face(anEntry.Finish(myFace));
        }
    }

    // The body of OCCT's Init(Path, Edge1, Edge2, Offset, FirstEdge, LastEdge, ...), on the edges to write.
    private void initPipe(in TopoDS_Edge Path, in TopoDS_Edge Edge1, in TopoDS_Edge Edge2, double Offset,
                          in TopoDS_Edge FirstEdge, in TopoDS_Edge LastEdge, bool Polynomial, double Tol,
                          Continuity Conti)
    {
        bool C1Denerated = false;
        bool C2Denerated = false;
        myStatus = BRepOffset_Good;
        myShape = Path;

        Placement Loc;
        var f = new double[3];
        var l = new double[3];

        Geom_Curve CP = BRep_Tool.Curve(Path, out Loc, out f[0], out l[0])!;
        CP = new Geom_TrimmedCurve(CP, f[0], l[0]);
        CP.Transform(Loc.Transformation());
        var HCP = new GeomAdaptor_Curve(CP);

        Geom_Curve? C1 = BRep_Tool.Curve(Edge1, out Loc, out f[1], out l[1]);

        Adaptor3d_Curve HEdge1;
        bool C1is3D = true;
        if (C1 is null)
        {
            C1is3D = false;
            BRep_Tool.CurveOnSurface(Edge1, out Geom2d_Curve? C12d, out Geom_Surface? S1, out Loc, out f[1], out l[1]);
            S1 = (Geom_Surface)S1!.Transformed(Loc.Transformation());
            C12d = new Geom2d_TrimmedCurve(C12d!, f[1], l[1]);
            var HS1 = new GeomAdaptor_Surface(S1);
            var HC1 = new Geom2dAdaptor_Curve(C12d);
            // OCCT.NET: OCCT copies a local Adaptor3d_CurveOnSurface Cons(HC1, HS1)
            HEdge1 = new Adaptor3d_CurveOnSurface(HC1, HS1);
        }
        else
        {
            C1 = new Geom_TrimmedCurve(C1, f[1], l[1]);
            C1.Transform(Loc.Transformation());
            HEdge1 = new GeomAdaptor_Curve(C1);
            var AC1 = new GeomAdaptor_Curve(C1);
            if (AC1.GetType() == Circle)
            {
                C1Denerated = (AC1.Circle().Radius() < global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }

        Geom_Curve? C2 = BRep_Tool.Curve(Edge2, out Loc, out f[2], out l[2]);

        Adaptor3d_Curve HEdge2;
        bool C2is3D = true;
        if (C2 is null)
        {
            C2is3D = false;
            BRep_Tool.CurveOnSurface(Edge2, out Geom2d_Curve? C12d, out Geom_Surface? S1, out Loc, out f[2], out l[2]);
            S1 = (Geom_Surface)S1!.Transformed(Loc.Transformation());
            C12d = new Geom2d_TrimmedCurve(C12d!, f[2], l[2]);
            var HS1 = new GeomAdaptor_Surface(S1);
            var HC1 = new Geom2dAdaptor_Curve(C12d);
            // OCCT.NET: OCCT copies a local Adaptor3d_CurveOnSurface Cons(HC1, HS1)
            HEdge2 = new Adaptor3d_CurveOnSurface(HC1, HS1);
        }
        else
        {
            C2 = new Geom_TrimmedCurve(C2, f[2], l[2]);
            C2.Transform(Loc.Transformation());
            HEdge2 = new GeomAdaptor_Curve(C2);
            var AC2 = new GeomAdaptor_Curve(C2);
            if (AC2.GetType() == Circle)
            {
                C2Denerated = (AC2.Circle().Radius() < global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }

        // Calcul du tuyau
        var Pipe = new GeomFill_Pipe(HCP, HEdge1, HEdge2, Math.Abs(Offset));
        Pipe.Perform(Tol, Polynomial, Conti);
        if (!Pipe.IsDone())
        {
            throw new ConstructionErrorException("GeomFill_Pipe : Cannot make a surface");
        }
        double ErrorPipe = Pipe.ErrorOnSurf();

        Geom_Surface S = Pipe.InternalSurface()!;
        bool ExchUV = Pipe.ExchangeUV();
        S.Bounds(out double f1, out double l1, out double f2, out double l2);

        // Perform the face
        double PathTol = BRep_Tool.Tolerance(Path);
        double TheTol;
        var myBuilder = new BRep_Builder();
        myBuilder.MakeFace(ref myFace);
        var Id = new Placement();
        myBuilder.UpdateFace(myFace, S, Id, PathTol);

        // update de Edge1. (Rem : has already a 3d curve)
        double U, U1, U2;
        Geom2d_Curve PC;
        if (ExchUV)
        {
            PC = new Geom2d_Line(new Point2D(0, f2), new Direction2D(Direction2D.D.X));
            U1 = f1;
            U2 = l1;
            if (!C1is3D)
            {
                C1 = S.VIso(f2);
            }
        }
        else
        {
            PC = new Geom2d_Line(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y));
            U1 = f2;
            U2 = l2;
            if (!C1is3D)
            {
                C1 = S.UIso(f1);
            }
        }

        Geom_Curve? Dummy = null;
        if (!C1is3D)
        {
            UpdateEdge(Edge1, C1, Id, BRep_Tool.Tolerance(Edge1));
        }
        else if (C1Denerated)
        {
            UpdateEdge(Edge1, Dummy, Id, BRep_Tool.Tolerance(Edge1));
            myBuilder.Degenerated(Edge1, true);
        }

        TheTol = Max(PathTol, BRep_Tool.Tolerance(Edge1) + ErrorPipe);
        UpdateEdge(Edge1, PC, myFace, TheTol);

        // mise a same range de la nouvelle pcurve.
        if (!C1is3D && !C1Denerated)
        {
            myBuilder.SameRange(Edge1, false);
            myBuilder.Range(Edge1, U1, U2, true);
        }
        myBuilder.Range(Edge1, myFace, U1, U2);
        BRepLib.SameRange(Edge1);

        // mise a sameparameter pour les KPart
        if (ErrorPipe == 0)
        {
            TheTol = Max(TheTol, Tol);
            myBuilder.SameParameter(Edge1, false);
            BRepLib.SameParameter(Edge1, TheTol);
        }

        // Update de edge2. (Rem : has already a 3d curve)
        if (ExchUV)
        {
            PC = new Geom2d_Line(new Point2D(0, l2), new Direction2D(Direction2D.D.X));
            U1 = f1;
            U2 = l1;
            if (!C2is3D)
            {
                C2 = S.VIso(l2);
            }
        }
        else
        {
            PC = new Geom2d_Line(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y));
            U1 = f2;
            U2 = l2;
            if (!C2is3D)
            {
                C2 = S.UIso(l1);
            }
        }

        if (!C2is3D)
        {
            UpdateEdge(Edge2, C2, Id, BRep_Tool.Tolerance(Edge2));
        }
        else if (C2Denerated)
        {
            UpdateEdge(Edge2, Dummy, Id, BRep_Tool.Tolerance(Edge2));
            myBuilder.Degenerated(Edge2, true);
        }

        TheTol = Max(PathTol, BRep_Tool.Tolerance(Edge2) + ErrorPipe);
        UpdateEdge(Edge2, PC, myFace, TheTol);

        // mise a same range de la nouvelle pcurve.
        myBuilder.SameRange(Edge2, false);
        if (!C2is3D && !C2Denerated)
        {
            myBuilder.Range(Edge2, U1, U2, true);
        }
        myBuilder.Range(Edge2, myFace, U1, U2);
        BRepLib.SameRange(Edge2);

        // mise a sameparameter pour les KPart
        if (ErrorPipe == 0)
        {
            TheTol = Max(TheTol, Tol);
            myBuilder.SameParameter(Edge2, false);
            BRepLib.SameParameter(Edge2, TheTol);
        }

        TopoDS_Edge Edge3 = new(), Edge4 = new();
        // eval edge3
        TopExp.Vertices(Path, out TopoDS_Vertex V1f, out TopoDS_Vertex V1l);
        bool IsClosed = (V1f.IsSame(V1l));

        TopExp.Vertices(Edge1, out V1f, out V1l);
        TopExp.Vertices(Edge2, out TopoDS_Vertex V2f, out TopoDS_Vertex V2l);

        bool StartDegenerated = (V1f.IsSame(V2f));
        bool EndDegenerated = (V1l.IsSame(V2l));

        bool E3rev = false;
        bool E4rev = false;

        TopoDS_Vertex VVf, VVl;
        if (FirstEdge.IsNull())
        {
            myBuilder.MakeEdge(ref Edge3);
            myBuilder.Add(Edge3, V1f.Oriented(TopAbs_FORWARD));
            myBuilder.Add(Edge3, V2f.Oriented(TopAbs_REVERSED));
        }
        else
        {
            TopoDS_Shape aLocalEdge = FirstEdge.Oriented(TopAbs_FORWARD);
            Edge3 = TopoDS.Edge(aLocalEdge);
            //    Edge3 = TopoDS::Edge(FirstEdge.Oriented(TopAbs_FORWARD));
            TopExp.Vertices(Edge3, out VVf, out VVl);
            // OCCT_DEBUG warning on unshared vertices left out.
            if (!VVf.IsSame(V1f) && !VVf.IsSame(V2f))
            {
                // On fait vraisemblablement des conneries !!
                // On cree un autre edge, on appelle le Sewing apres.
                myBuilder.MakeEdge(ref Edge3);
                myBuilder.Add(Edge3, V1f.Oriented(TopAbs_FORWARD));
                myBuilder.Add(Edge3, V2f.Oriented(TopAbs_REVERSED));
            }
            else if (!VVf.IsSame(V1f))
            {
                Edge3.Reverse();
                E3rev = true;
            }
        }

        if (IsClosed)
        {
            Edge4 = Edge3;
        }

        double TolApp = global::Kerf.Engine.Foundation.Precision.Approximation();

        Geom2d_Line L1, L2;
        if (IsClosed)
        {
            if (ExchUV)
            {
                // rem : si ExchUv, il faut  reverser le Wire.
                // donc l'edge Forward dans la face sera E4 : d'ou L1 et L2
                L2 = new Geom2d_Line(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y));
                L1 = new Geom2d_Line(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y));
                U1 = f2;
                U2 = l2;
            }
            else
            {
                L1 = new Geom2d_Line(new Point2D(0, f2), new Direction2D(Direction2D.D.X));
                L2 = new Geom2d_Line(new Point2D(0, l2), new Direction2D(Direction2D.D.X));
                U1 = f1;
                U2 = l1;
            }
            if (E3rev)
            {
                L1.Reverse();
                L2.Reverse();
                U = -U1;
                U1 = -U2;
                U2 = U;
            }
            UpdateEdge(Edge3, L1, L2, myFace, PathTol);
            myBuilder.Range(Edge3, myFace, U1, U2);
            if (StartDegenerated)
            {
                myBuilder.Degenerated(Edge3, true);
            }
            else if (FirstEdge.IsNull())
            { // then the 3d curve has not been yet computed
                ComputeCurve3d(Edge3, L1, S, Id, TolApp);
            }
        }
        else
        {
            if (LastEdge.IsNull())
            {
                myBuilder.MakeEdge(ref Edge4);
                myBuilder.Add(Edge4, V1l.Oriented(TopAbs_FORWARD));
                myBuilder.Add(Edge4, V2l.Oriented(TopAbs_REVERSED));
            }
            else
            {
                TopoDS_Shape aLocalEdge = LastEdge.Oriented(TopAbs_FORWARD);
                Edge4 = TopoDS.Edge(aLocalEdge);
                //      Edge4 = TopoDS::Edge(LastEdge.Oriented(TopAbs_FORWARD));
                TopExp.Vertices(Edge4, out VVf, out VVl);
                // OCCT_DEBUG warning on unshared vertices left out.
                if (!VVf.IsSame(V1l) && !VVf.IsSame(V2l))
                {
                    // On fait vraisemblablement des conneries !!
                    // On cree un autre edge, on appelle le Sewing apres.
                        myBuilder.MakeEdge(ref Edge4);
                    myBuilder.Add(Edge4, V1l.Oriented(TopAbs_FORWARD));
                    myBuilder.Add(Edge4, V2l.Oriented(TopAbs_REVERSED));
                }
                else if (!VVf.IsSame(V1l))
                {
                    Edge4.Reverse();
                    E4rev = true;
                }
            }

            if (ExchUV)
            {
                L1 = new Geom2d_Line(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y));
                U1 = f2;
                U2 = l2;
            }
            else
            {
                L1 = new Geom2d_Line(new Point2D(0, f2), new Direction2D(Direction2D.D.X));
                U1 = f1;
                U2 = l1;
            }
            if (E3rev)
            {
                L1.Reverse();
                U = -U1;
                U1 = -U2;
                U2 = U;
            }
            UpdateEdge(Edge3, L1, myFace, PathTol);
            myBuilder.Range(Edge3, myFace, U1, U2);
            if (StartDegenerated)
            {
                myBuilder.Degenerated(Edge3, true);
            }
            else if (FirstEdge.IsNull())
            { // then the 3d curve has not been yet computed
                ComputeCurve3d(Edge3, L1, S, Id, TolApp);
            }

            if (ExchUV)
            {
                L2 = new Geom2d_Line(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y));
                U1 = f2;
                U2 = l2;
            }
            else
            {
                L2 = new Geom2d_Line(new Point2D(0, l2), new Direction2D(Direction2D.D.X));
                U1 = f1;
                U2 = l1;
            }
            if (E4rev)
            {
                L2.Reverse();
                U = -U1;
                U1 = -U2;
                U2 = U;
            }
            UpdateEdge(Edge4, L2, myFace, PathTol);
            myBuilder.Range(Edge4, myFace, U1, U2);
            if (EndDegenerated)
            {
                myBuilder.Degenerated(Edge4, true);
            }
            else if (LastEdge.IsNull())
            { // then the 3d curve has not been yet computed
                ComputeCurve3d(Edge4, L2, S, Id, TolApp);
            }
        }

        // SameParameter ??
        if (!FirstEdge.IsNull() && !StartDegenerated)
        {
            BRepLib.BuildCurve3d(Edge3, PathTol);
            myBuilder.SameRange(Edge3, false);
            myBuilder.SameParameter(Edge3, false);
            BRepLib.SameParameter(Edge3, Tol);
        }
        if (!LastEdge.IsNull() && !EndDegenerated)
        {
            BRepLib.BuildCurve3d(Edge4, PathTol);
            myBuilder.SameRange(Edge4, false);
            myBuilder.SameParameter(Edge4, false);
            BRepLib.SameParameter(Edge4, Tol);
        }

        TopoDS_Wire W = new();
        myBuilder.MakeWire(ref W);

        myBuilder.Add(W, Edge1.Oriented(TopAbs_REVERSED));
        myBuilder.Add(W, Edge2.Oriented(TopAbs_FORWARD));
        myBuilder.Add(W, Edge4.Reversed());
        myBuilder.Add(W, Edge3);

        if (ExchUV)
        {
            W.Reverse();
        }

        myBuilder.Add(myFace, W);
        if (ExchUV)
        {
            myFace.Reverse();
        }

        BRepTools.Update(myFace);

        if (Edge1.Orientation() == TopAbs_REVERSED)
        {
            myFace.Reverse();
        }
    }

    //=================================================================================================

    /// <summary>
    /// Builds the sphere of radius <paramref name="Offset"/> centred on <paramref name="Vertex"/>, bounded by the
    /// edges <paramref name="LEdge"/>, which get pcurves on it: in place when they are unfrozen, else on copies (see the
    /// class remarks). <paramref name="Tol"/> and <paramref name="Conti"/> are used only if
    /// <paramref name="Polynomial"/> (for the approximation).
    /// </summary>
    public void Init(in TopoDS_Vertex Vertex, NCollection_List<TopoDS_Shape> LEdge, double Offset,
                     bool Polynomial = false, double Tol = 1.0e-4, Continuity Conti = C1)
    {
        // OCCT.NET: copy at entry (plan 2.2) of the edges written below, when one of them holds a frozen shape.
        var aShapes = new List<TopoDS_Shape>();
        for (var anIt = new NCollection_List<TopoDS_Shape>.Iterator(LEdge); anIt.More(); anIt.Next())
        {
            aShapes.Add(anIt.Value());
        }
        BRepTools_CopyAtEntry? anEntry = CopyAtEntry(aShapes);
        NCollection_List<TopoDS_Shape> aLEdge = LEdge;
        if (anEntry is not null)
        {
            aLEdge = new NCollection_List<TopoDS_Shape>();
            foreach (TopoDS_Shape anEdge in aShapes)
            {
                aLEdge.Append(anEntry.Copied(anEdge));
            }
        }
        initSphere(Vertex, aLEdge, Offset, Polynomial, Tol, Conti);
        if (anEntry is not null)
        {
            myFace = TopoDS.Face(anEntry.Finish(myFace));
        }
    }

    // The body of OCCT's Init(Vertex, LEdge, Offset, Polynomial, TolApp, Conti), on the edges to write.
    private void initSphere(in TopoDS_Vertex Vertex, NCollection_List<TopoDS_Shape> LEdge, double Offset,
                            bool Polynomial, double TolApp, Continuity Conti)
    {
        myStatus = BRepOffset_Good;
        myShape = Vertex;

        // evaluate the Ax3 of the Sphere
        // find 3 different vertices in LEdge
        var it = new NCollection_List<TopoDS_Shape>.Iterator();

        // OCCT_DEBUG naming of the shapes (Affich) left out.

        Point3D Origin = BRep_Tool.Pnt(Vertex);

        //// Find the axis of the sphere to exclude
        //// degenerated and seam edges from the face under construction
        var MW = new BRepLib_MakeWire();
        MW.Add(LEdge);
        TopoDS_Wire theWire = MW.InternalWire();

        var Fixer = new ShapeFix_Shape(theWire);
        Fixer.Perform();
        theWire = TopoDS.Wire(Fixer.InternalShape());

        var GlobalProps = new GProp_GProps();
        BRepGProp.LinearProperties(theWire, GlobalProps);
        Point3D BaryCenter = GlobalProps.CentreOfMass();
        var Xdir = new Vector3D(BaryCenter, Origin);

        Point3D FarestCorner = GetFarestCorner(theWire);
        Plane3D thePlane = new gce_MakePln(Origin, BaryCenter, FarestCorner).Value();
        Direction3D Vdir = thePlane.Axis().Direction();

        var Axis = new CoordinateSystem(Origin, Vdir, new Direction3D(Xdir));

        Geom_Surface S = new Geom_SphericalSurface(Axis, Math.Abs(Offset));

        double f, l, Tol = BRep_Tool.Tolerance(Vertex);

        Placement Loc = new();
        var myBuilder = new BRep_Builder();
        myBuilder.MakeFace(ref myFace);
        Geom_Surface SS = S;

        // En polynomial, calcul de la surface par F(u,v).
        // Pas de changement de parametre, donc ProjLib sur la Sphere
        // reste OK.
        if (Polynomial)
        {
            var Approx = new GeomConvert_ApproxSurface(S, TolApp, Conti, Conti, 10, 10, 10, 1);
            if (Approx.IsDone())
            {
                SS = Approx.Surface()!;
            }
        }

        myBuilder.UpdateFace(myFace, SS, Loc, Tol);

        TopoDS_Wire W = new();
        myBuilder.MakeWire(ref W);

        for (it.Initialize(LEdge); it.More(); it.Next())
        {
            TopoDS_Edge E = TopoDS.Edge(it.Value());

            Geom_Curve? C = BRep_Tool.Curve(E, out Loc, out f, out l);
            if (C is null)
            {
                BRepLib.BuildCurve3d(E, BRep_Tool.Tolerance(E));
                C = BRep_Tool.Curve(E, out Loc, out f, out l);
            }
            C = new Geom_TrimmedCurve(C!, f, l);
            C.Transform(Loc.Transformation());

            Geom2d_Curve PCurve = GeomProjLib.Curve2d(C, S)!;
            // check if the first point of PCurve in is the canonical boundaries
            // of the sphere. Else move it.
            // the transformation is : U` = U + PI + 2 k  PI
            //                         V` = +/- PI + 2 k` PI
            Point2D P2d = PCurve.Value(f);
            bool IsToAdjust = false;
            if (P2d.Y() < -M_PI / 2.0)
            {
                IsToAdjust = true;
                PCurve.Mirror(new Axis2D(new Point2D(0.0, -M_PI / 2.0), Constants.DX2d()));
            }
            else if (P2d.Y() > M_PI / 2.0)
            {
                IsToAdjust = true;
                PCurve.Mirror(new Axis2D(new Point2D(0.0, M_PI / 2.0), Constants.DX2d()));
            }
            if (IsToAdjust)
            {
                // set the u firstpoint in [0,2*pi]
                var Tr = new Vector2D(M_PI, 0.0);
                if (P2d.X() > M_PI)
                {
                    Tr.Reverse();
                }
                PCurve.Translate(Tr);
            }

            UpdateEdge(E, PCurve, myFace, Tol);
            myBuilder.Range(E, myFace, f, l);
            myBuilder.Add(W, E);
        }
        if (Offset < 0.0)
        {
            myBuilder.Add(myFace, W.Oriented(TopAbs_REVERSED));
            myFace.Reverse();
        }
        else
        {
            myBuilder.Add(myFace, W);
        }

        BRepTools.Update(myFace);
    }

    //=================================================================================================

    /// <summary>Builds the pipe of radius |<paramref name="Offset"/>| around the free boundary
    /// <paramref name="Edge"/> (used only by the rolling ball).</summary>
    public void Init(in TopoDS_Edge Edge, double Offset)
    {
        myShape = Edge;
        double myOffset = Math.Abs(Offset);

        Geom_Curve CP = BRep_Tool.Curve(Edge, out Placement Loc, out double f, out double l)!;
        CP = new Geom_TrimmedCurve(CP, f, l);
        CP.Transform(Loc.Transformation());

        var Pipe = new GeomFill_Pipe(CP, myOffset);
        Pipe.Perform();
        if (!Pipe.IsDone())
        {
            throw new ConstructionErrorException("GeomFill_Pipe : Cannot make a surface");
        }

        var MF = new BRepLib_MakeFace(Pipe.InternalSurface()!, global::Kerf.Engine.Foundation.Precision.Confusion());
        myFace = MF.InternalFace();

        if (Offset < 0.0)
        {
            myFace.Reverse();
        }
    }

    //=================================================================================================

    /// <summary>The shape the offset was built from.</summary>
    public TopoDS_Shape InitialShape() => myShape;

    //=================================================================================================

    /// <summary>The offset face (frozen).</summary>
    public TopoDS_Face Face()
    {
        myFace.TShape()?.FreezeGraph();
        return myFace;
    }

    /// <summary><see cref="Face"/> without freezing, for ported code.</summary>
    internal TopoDS_Face InternalFace() => myFace;

    //=================================================================================================

    /// <summary>
    /// The shape generated from a sub-shape of the initial shape (frozen): for a face, the offset of one of its edges;
    /// for a path edge, the end edge of the pipe at one of its vertices.
    /// </summary>
    public TopoDS_Shape Generated(in TopoDS_Shape Shape)
    {
        TopoDS_Shape aShape = InternalGenerated(Shape);
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary><see cref="Generated"/> without freezing, for ported code.</summary>
    internal TopoDS_Shape InternalGenerated(in TopoDS_Shape Shape)
    {
        TopoDS_Shape aShape = new();

        switch (myShape.ShapeType())
        {
            case TopAbs_FACE:
            {
                var exp = new TopExp_Explorer(myShape.Oriented(TopAbs_FORWARD), TopAbs_EDGE);
                var expo = new TopExp_Explorer(myFace.Oriented(TopAbs_FORWARD), TopAbs_EDGE);
                for (; exp.More() && expo.More(); exp.Next(), expo.Next())
                {
                    if (Shape.IsSame(exp.Current()))
                    {
                        if (myShape.Orientation() == TopAbs_REVERSED)
                        {
                            aShape = expo.Current().Reversed();
                        }
                        else
                        {
                            aShape = expo.Current();
                        }
                        break;
                    }
                }
            }
            break;

            case TopAbs_EDGE:
                // have generate a pipe.
            {
                TopExp.Vertices(TopoDS.Edge(myShape), out TopoDS_Vertex V1, out TopoDS_Vertex V2);

                var expf = new TopExp_Explorer(myFace.Oriented(TopAbs_FORWARD), TopAbs_WIRE);
                var expo = new TopExp_Explorer(expf.Current().Oriented(TopAbs_FORWARD), TopAbs_EDGE);
                expo.Next();
                expo.Next();

                if (V2.IsSame(Shape))
                {
                    if (expf.Current().Orientation() == TopAbs_REVERSED)
                    {
                        aShape = expo.Current().Reversed();
                    }
                    else
                    {
                        aShape = expo.Current();
                    }
                }
                else
                {
                    expo.Next();
                    if (expf.Current().Orientation() == TopAbs_REVERSED)
                    {
                        aShape = expo.Current().Reversed();
                    }
                    else
                    {
                        aShape = expo.Current();
                    }
                }
                if (myFace.Orientation() == TopAbs_REVERSED)
                {
                    aShape.Reverse();
                }
            }
            break;
            default:
                break;
        }

        return aShape;
    }

    //=================================================================================================

    /// <summary>The status of the offset surface.</summary>
    public BRepOffset_Status Status() => myStatus;

    //=================================================================================================

    // OCCT.NET: the copy at entry of the edges a pipe or a sphere writes into: null (write in place, as OCCT) unless
    // one of them holds a frozen shape; else one exact copy of them all, so that the vertices they share stay shared.
    private static BRepTools_CopyAtEntry? CopyAtEntry(IReadOnlyList<TopoDS_Shape> theShapes)
    {
        bool hasFrozen = false;
        foreach (TopoDS_Shape aShape in theShapes)
        {
            if (!aShape.IsNull() && BRepTools.HoldsFrozen(aShape))
            {
                hasFrozen = true;
                break;
            }
        }
        if (!hasFrozen)
        {
            return null;
        }

        var aB = new BRep_Builder();
        TopoDS_Compound aCompound = new();
        aB.MakeCompound(ref aCompound);
        foreach (TopoDS_Shape aShape in theShapes)
        {
            if (!aShape.IsNull())
            {
                aB.Add(aCompound, aShape);
            }
        }
        return new BRepTools_CopyAtEntry(aCompound, null, true);
    }

    // The copy of an edge given to a pipe (the edge itself when nothing was copied, or when it is null).
    private static TopoDS_Edge Copied(BRepTools_CopyAtEntry? theEntry, in TopoDS_Edge theEdge)
    {
        if (theEntry is null || theEdge.IsNull())
        {
            return theEdge;
        }
        return TopoDS.Edge(theEntry.Copied(theEdge));
    }
}
