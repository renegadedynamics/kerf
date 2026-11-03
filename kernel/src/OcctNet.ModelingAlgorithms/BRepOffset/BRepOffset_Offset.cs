// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Offset.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Offset.lxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Offset.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BRepOffset_Status;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>Builds the offset of a face, or the pipe or sphere joining offsets along an edge or at a vertex.</summary>
/// <remarks>
/// <para>
/// The offset of a face (<see cref="Init(in Face, double, bool, JoinKind)"/>) only reads the face: its
/// edges and vertices are new, but for those taken from <c>Created</c>, the images of its edges and vertices in the
/// offsets of other faces. As in OCCT, the offset writes pcurves on the new face and ranges into those
/// <c>Created</c> edges, in place: they are the 3D offset's own working edges and must be unfrozen (a frozen one throws
/// <see cref="LockedShapeException"/> at the first write).
/// </para>
/// <para>
/// The pipe along an edge and the sphere at a vertex write pcurves, ranges and flags into the edges they are given
/// (<c>Edge1</c>, <c>Edge2</c>, <c>FirstEdge</c>, <c>LastEdge</c>, the edges of <c>LEdge</c>), so that the pipe or
/// sphere shares them with the faces' offsets. When none of these holds a frozen shape (the 3D offset's working
/// edges), they are written in place, as in OCCT. When one does (a caller's shapes), they are copied at the entry
/// (<see cref="CopyAtEntry"/>): the face is built on the copies, and the caller's sub-shapes the
/// construction left unchanged (vertices whose tolerance didn't grow) are given back in it.
/// </para>
/// <para>
/// <see cref="Face"/> and <see cref="Generated"/> freeze what they return (a publication point); ported code reads
/// <see cref="InternalFace"/> and <see cref="InternalGenerated"/>, which don't.
/// </para>
/// </remarks>
public sealed class BRepOffset_Offset
{
    private Shape myShape = new();
    private BRepOffset_Status myStatus = BRepOffset_Unknown;
    private Face myFace = new();
    private readonly NCollection_DataMap<Shape, Shape> myMap = new(ShapeHasher.Instance);

    // OCCT_DEBUG statics (Affich, NbOFFSET) and their blocks are left out.

    private static Point3D GetFarestCorner(in Wire aWire)
    {
        var Vertices = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aWire, ShapeType.Vertex, Vertices);

        double MaxDist = 0.0;
        Point3D thePoint = new();
        for (int i = 1; i <= Vertices.Extent(); i++)
        {
            for (int j = 1; j <= Vertices.Extent(); j++)
            {
                Vertex V1 = ShapeCasts.Vertex(Vertices[i]);
                Vertex V2 = ShapeCasts.Vertex(Vertices[j]);
                Point3D P1 = Tool.Pnt(V1);
                Point3D P2 = Tool.Pnt(V2);
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

    private static void UpdateEdge(in Edge E, Curve3D? C, in Placement L, double Tol)
    {
        // Cut curves to avoid copies in the extensions.
        var B = new BrepBuilder();
        if (C is TrimmedCurve3D BC)
        {
            B.UpdateEdge(E, BC.BasisCurve(), L, Tol);
        }
        else
        {
            B.UpdateEdge(E, C, L, Tol);
        }
    }

    //=================================================================================================

    private static void UpdateEdge(in Edge E, Curve2D? C, in Face F, double Tol)
    {
        // Cut curves to avoid copies in the extensions.
        var B = new BrepBuilder();
        if (C is TrimmedCurve2D BC)
        {
            B.UpdateEdge(E, BC.BasisCurve(), F, Tol);
        }
        else
        {
            B.UpdateEdge(E, C, F, Tol);
        }
    }

    //=================================================================================================

    private static void UpdateEdge(in Edge E, Curve2D? C1, Curve2D? C2, in Face F, double Tol)
    {
        // Cut curves to avoid copies in the extensions.
        var B = new BrepBuilder();
        Curve2D? NC1, NC2;
        if (C1 is TrimmedCurve2D BC1)
        {
            NC1 = BC1.BasisCurve();
        }
        else
        {
            NC1 = C1;
        }
        if (C2 is TrimmedCurve2D BC2)
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

    private static void ComputeCurve3d(in Edge Edge, Curve2D Curve, Surface Surf, in Placement Loc,
                                       double Tol)
    {
        // try to find the particular case
        // if not found call BRepLib::BuildCurve3d

        bool IsComputed = false;

        // Search only isos on analytic surfaces.
        var C = new CurveAdaptor2D(Curve);
        var S = new SurfaceAdaptor(Surf);
        CurveKind CTy = C.GetType();
        SurfaceKind STy = S.GetType();
        var TheBuilder = new BrepBuilder();

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
                            CircleCurve3D Circle = new CircleCurve3D(Ci);
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
                        CircleCurve3D Circle = new CircleCurve3D(Ci);
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
                        CircleCurve3D Circle = new CircleCurve3D(Ci);
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
                        CircleCurve3D Circle = new CircleCurve3D(Ci);
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
                        CircleCurve3D Circle = new CircleCurve3D(Ci);

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
                        LineCurve3D Line = new LineCurve3D(L);
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
                        LineCurve3D Line = new LineCurve3D(L);
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
                        CircleCurve3D Circle = new CircleCurve3D(Ci);

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
            Curve3D C3d = GeomAPI.To3d(Curve, S.Plane());
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
    public BRepOffset_Offset(in Face Face, double Offset, bool OffsetOutside = true,
                             JoinKind JoinType = Arc) =>
        Init(Face, Offset, OffsetOutside, JoinType);

    //=================================================================================================

    /// <summary>
    /// The offset of <paramref name="Face"/>, sharing the edges already generated from other faces:
    /// <c>Created(E) = E'</c>, where E is an edge (or vertex) of the face and E' its image in the offset of another face
    /// sharing E with at least G1 continuity (so two tangent faces generate one edge, no pipe). The edges E' are written
    /// into and must be unfrozen.
    /// </summary>
    public BRepOffset_Offset(in Face Face, double Offset, NCollection_DataMap<Shape, Shape> Created,
                             bool OffsetOutside = true, JoinKind JoinType = Arc) =>
        Init(Face, Offset, Created, OffsetOutside, JoinType);

    //=================================================================================================

    /// <summary>The pipe of radius <paramref name="Offset"/> along <paramref name="Path"/> between two edges.</summary>
    public BRepOffset_Offset(in Edge Path, in Edge Edge1, in Edge Edge2, double Offset,
                             bool Polynomial = false, double Tol = 1.0e-4, Continuity Conti = C1) =>
        Init(Path, Edge1, Edge2, Offset, Polynomial, Tol, Conti);

    //=================================================================================================

    /// <summary>The pipe along <paramref name="Path"/> between two edges, closed by the given end edges.</summary>
    public BRepOffset_Offset(in Edge Path, in Edge Edge1, in Edge Edge2, double Offset,
                             in Edge FirstEdge, in Edge LastEdge, bool Polynomial = false,
                             double Tol = 1.0e-4, Continuity Conti = C1) =>
        Init(Path, Edge1, Edge2, Offset, FirstEdge, LastEdge, Polynomial, Tol, Conti);

    //=================================================================================================

    /// <summary>
    /// The sphere of radius <paramref name="Offset"/> at <paramref name="Vertex"/> bounded by the edges
    /// <paramref name="LEdge"/>. <paramref name="Tol"/> and <paramref name="Conti"/> are used only if
    /// <paramref name="Polynomial"/> (for the approximation).
    /// </summary>
    public BRepOffset_Offset(in Vertex Vertex, NCollection_List<Shape> LEdge, double Offset,
                             bool Polynomial = false, double Tol = 1.0e-4, Continuity Conti = C1) =>
        Init(Vertex, LEdge, Offset, Polynomial, Tol, Conti);

    //=================================================================================================

    /// <summary>Builds the offset of <paramref name="Face"/> by <paramref name="Offset"/>.</summary>
    public void Init(in Face Face, double Offset, bool OffsetOutside = true,
                     JoinKind JoinType = Arc)
    {
        var Empty = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        Init(Face, Offset, Empty, OffsetOutside, JoinType);
    }

    //=================================================================================================

    /// <summary>
    /// Builds the offset of <paramref name="Face"/>, sharing the edges already generated from other faces (see the
    /// constructor). The edges of <paramref name="Created"/> are written into and must be unfrozen.
    /// </summary>
    public void Init(in Face Face, double Offset, NCollection_DataMap<Shape, Shape> Created,
                     bool OffsetOutside = true, JoinKind JoinType = Arc)
    {
        myShape = Face;
        double myOffset = Offset;
        if (Face.Orientation() == Reversed)
        {
            myOffset *= -1.0;
        }

        Surface S = Tool.Surface(Face, out Placement L)!;
        // On detrime les surfaces, evite des recopies dans les extensions.
        if (S is TrimmedSurface RT)
        {
            S = RT.BasisSurface();
        }
        bool IsTransformed = false;
        if ((S is global::Kerf.Geometry.BSplineSurface || S is ExtrusionSurface || S is RevolutionSurface
             || S is global::Kerf.Geometry.OffsetSurface)
            && !L.IsIdentity)
        {
            S = (Surface)S.Copy();
            S.Transform(L.Transformation());
            IsTransformed = true;
        }
        // particular case of cone
        if (S is ConicalSurface Co)
        {
            Point3D Apex = Co.Apex();
            global::Kerf.Engine.Geometry.ElSLib.Parameters(Co.Cone(), Apex, out double Uc, out double Vc);
            BrepTools.UVBounds(Face, out double UU1, out double UU2, out double VV1, out double VV2);
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

        Surface TheSurf = BRepOffset.Surface(S, myOffset, out myStatus);

        // processing offsets of faces with possible degenerated edges
        bool UminDegen = false;
        bool UmaxDegen = false;
        bool VminDegen = false;
        bool VmaxDegen = false;
        bool UisoDegen = false;
        Point3D MinApex = new(), MaxApex = new();
        bool HasSingularity = false;
        double fpar, lpar;
        BrepTools.UVBounds(Face, out double uf1, out double uf2, out double vf1, out double vf2);
        if ((!OffsetOutside || JoinType != Arc)
            && (TheSurf.GetType() == typeof(ConicalSurface) || TheSurf.GetType() == typeof(global::Kerf.Geometry.OffsetSurface)))
        {
            var DegEdges = new NCollection_Sequence<Shape>();
            var Explo = new Explorer(Face, ShapeType.Edge);
            for (; Explo.More(); Explo.Next())
            {
                Edge anEdge = ShapeCasts.Edge(Explo.Current());

                if (Tool.Degenerated(anEdge))
                {
                    Curve2D c2d = Tool.CurveOnSurface(anEdge, Face, out double aF, out double aL)!;

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
                Edge theDegEdge = ShapeCasts.Edge(DegEdges[1]);
                Curve2D aCurve = Tool.CurveOnSurface(theDegEdge, Face, out fpar, out lpar)!;
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
                if (TheSurf.GetType() == typeof(ConicalSurface))
                {
                    Cone theCone = ((ConicalSurface)TheSurf).Cone();
                    Point3D apex = theCone.Apex();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(theCone, apex, out double Uapex, out double Vapex);
                    if (VminDegen)
                    {
                        TheSurf = new TrimmedSurface(TheSurf, uf1, uf2, Vapex, vf2);
                        MinApex = apex;
                        HasSingularity = true;
                    }
                    else if (VmaxDegen)
                    {
                        TheSurf = new TrimmedSurface(TheSurf, uf1, uf2, vf1, Vapex);
                        MaxApex = apex;
                        HasSingularity = true;
                    }
                }
                else // TheSurf->DynamicType() == STANDARD_TYPE(Geom_OffsetSurface)
                {
                    // OCCT.NET: the bounds of the extended surface are named aU1.. (C# forbids reusing u1.. below)
                    if (UminDegen)
                    {
                        Curve3D uiso = TheSurf.UIso(uf1);
                        if (BRepOffset_Tool.Gabarit(uiso) > TolApex)
                        {
                            Surface BasisSurf = ((global::Kerf.Geometry.OffsetSurface)TheSurf).BasisSurface();
                            Point3D Papex, Pfirst, Pquart, Pmid;
                            Papex = BasisSurf.Value(uf1, vf1);
                            Pfirst = TheSurf.Value(uf1, vf1);
                            Pquart = TheSurf.Value(uf1, 0.75 * vf1 + 0.25 * vf2);
                            Pmid = TheSurf.Value(uf1, 0.5 * (vf1 + vf2));
                            Vector3D DirApex = new Vector3D(Pfirst, Pquart) ^ new Vector3D(Pfirst, Pmid);
                            var LineApex = new LineCurve3D(Papex, new Direction3D(DirApex));
                            Vector3D DirGeneratrix = BasisSurf.DN(uf1, vf1, 1, 0);
                            var LineGeneratrix = new LineCurve3D(Pfirst, new Direction3D(DirGeneratrix));
                            var theExtrema = new GeomAPI_ExtremaCurveCurve(LineGeneratrix, LineApex);
                            theExtrema.NearestPoints(out Point3D Pint1, out Point3D Pint2);
                            double length = Pfirst.Distance(Pint1);
                            if (OffsetOutside)
                            {
                                BoundedSurface aSurf = new TrimmedSurface(TheSurf, uf1, uf2, vf1, vf2);
                                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSurf, length, 1, true, false);
                                TheSurf = aSurf;
                                TheSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                                MinApex = TheSurf.Value(aU1, vf1);
                            }
                            else
                            {
                                Curve3D viso = TheSurf.VIso(vf1);
                                var Projector = new GeomAPI_ProjectPointOnCurve(Pint1, viso);
                                double NewFirstU = Projector.LowerDistanceParameter();
                                TheSurf = new TrimmedSurface(TheSurf, NewFirstU, uf2, vf1, vf2);
                                MinApex = TheSurf.Value(NewFirstU, vf1);
                            }
                            HasSingularity = true;
                        }
                    } // end of if (UminDegen)
                    if (UmaxDegen)
                    {
                        Curve3D uiso = TheSurf.UIso(uf2);
                        if (BRepOffset_Tool.Gabarit(uiso) > TolApex)
                        {
                            Surface BasisSurf = ((global::Kerf.Geometry.OffsetSurface)TheSurf).BasisSurface();
                            Point3D Papex, Pfirst, Pquart, Pmid;
                            Papex = BasisSurf.Value(uf2, vf1);
                            Pfirst = TheSurf.Value(uf2, vf1);
                            Pquart = TheSurf.Value(uf2, 0.75 * vf1 + 0.25 * vf2);
                            Pmid = TheSurf.Value(uf2, 0.5 * (vf1 + vf2));
                            Vector3D DirApex = new Vector3D(Pfirst, Pquart) ^ new Vector3D(Pfirst, Pmid);
                            var LineApex = new LineCurve3D(Papex, new Direction3D(DirApex));
                            Vector3D DirGeneratrix = BasisSurf.DN(uf2, vf1, 1, 0);
                            var LineGeneratrix = new LineCurve3D(Pfirst, new Direction3D(DirGeneratrix));
                            var theExtrema = new GeomAPI_ExtremaCurveCurve(LineGeneratrix, LineApex);
                            theExtrema.NearestPoints(out Point3D Pint1, out Point3D Pint2);
                            double length = Pfirst.Distance(Pint1);
                            if (OffsetOutside)
                            {
                                BoundedSurface aSurf = new TrimmedSurface(TheSurf, uf1, uf2, vf1, vf2);
                                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSurf, length, 1, true, true);
                                TheSurf = aSurf;
                                TheSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                                MaxApex = TheSurf.Value(aU2, vf1);
                            }
                            else
                            {
                                Curve3D viso = TheSurf.VIso(vf1);
                                var Projector = new GeomAPI_ProjectPointOnCurve(Pint1, viso);
                                double NewLastU = Projector.LowerDistanceParameter();
                                TheSurf = new TrimmedSurface(TheSurf, uf1, NewLastU, vf1, vf2);
                                MaxApex = TheSurf.Value(NewLastU, vf1);
                            }
                            HasSingularity = true;
                        }
                    } // end of if (UmaxDegen)
                    if (VminDegen)
                    {
                        Curve3D viso = TheSurf.VIso(vf1);
                        if (BRepOffset_Tool.Gabarit(viso) > TolApex)
                        {
                            Surface BasisSurf = ((global::Kerf.Geometry.OffsetSurface)TheSurf).BasisSurface();
                            Point3D Papex, Pfirst, Pquart, Pmid;
                            Papex = BasisSurf.Value(uf1, vf1);
                            Pfirst = TheSurf.Value(uf1, vf1);
                            Pquart = TheSurf.Value(0.75 * uf1 + 0.25 * uf2, vf1);
                            Pmid = TheSurf.Value(0.5 * (uf1 + uf2), vf1);
                            Vector3D DirApex = new Vector3D(Pfirst, Pquart) ^ new Vector3D(Pfirst, Pmid);
                            var LineApex = new LineCurve3D(Papex, new Direction3D(DirApex));
                            Vector3D DirGeneratrix = BasisSurf.DN(uf1, vf1, 0, 1);
                            var LineGeneratrix = new LineCurve3D(Pfirst, new Direction3D(DirGeneratrix));
                            var theExtrema = new GeomAPI_ExtremaCurveCurve(LineGeneratrix, LineApex);
                            theExtrema.NearestPoints(out Point3D Pint1, out Point3D Pint2);
                            double length = Pfirst.Distance(Pint1);
                            if (OffsetOutside)
                            {
                                BoundedSurface aSurf = new TrimmedSurface(TheSurf, uf1, uf2, vf1, vf2);
                                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSurf, length, 1, false, false);
                                TheSurf = aSurf;
                                TheSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                                MinApex = TheSurf.Value(uf1, aV1);
                            }
                            else
                            {
                                Curve3D uiso = TheSurf.UIso(uf1);
                                var Projector = new GeomAPI_ProjectPointOnCurve(Pint1, uiso);
                                double NewFirstV = Projector.LowerDistanceParameter();
                                TheSurf = new TrimmedSurface(TheSurf, uf1, uf2, NewFirstV, vf2);
                                MinApex = TheSurf.Value(uf1, NewFirstV);
                                // TheSurf = new Geom_RectangularTrimmedSurface(TheSurf, uf1, uf2, vf1+length, vf2);
                                // MinApex = TheSurf->Value( uf1, vf1+length );
                            }
                            HasSingularity = true;
                        }
                    } // end of if (VminDegen)
                    if (VmaxDegen)
                    {
                        Curve3D viso = TheSurf.VIso(vf2);
                        if (BRepOffset_Tool.Gabarit(viso) > TolApex)
                        {
                            Surface BasisSurf = ((global::Kerf.Geometry.OffsetSurface)TheSurf).BasisSurface();
                            Point3D Papex, Pfirst, Pquart, Pmid;
                            Papex = BasisSurf.Value(uf1, vf2);
                            Pfirst = TheSurf.Value(uf1, vf2);
                            Pquart = TheSurf.Value(0.75 * uf1 + 0.25 * uf2, vf2);
                            Pmid = TheSurf.Value(0.5 * (uf1 + uf2), vf2);
                            Vector3D DirApex = new Vector3D(Pfirst, Pquart) ^ new Vector3D(Pfirst, Pmid);
                            var LineApex = new LineCurve3D(Papex, new Direction3D(DirApex));
                            Vector3D DirGeneratrix = BasisSurf.DN(uf1, vf2, 0, 1);
                            var LineGeneratrix = new LineCurve3D(Pfirst, new Direction3D(DirGeneratrix));
                            var theExtrema = new GeomAPI_ExtremaCurveCurve(LineGeneratrix, LineApex);
                            theExtrema.NearestPoints(out Point3D Pint1, out Point3D Pint2);
                            double length = Pfirst.Distance(Pint1);
                            if (OffsetOutside)
                            {
                                BoundedSurface aSurf = new TrimmedSurface(TheSurf, uf1, uf2, vf1, vf2);
                                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aSurf, length, 1, false, true);
                                TheSurf = aSurf;
                                TheSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                                MaxApex = TheSurf.Value(uf1, aV2);
                            }
                            else
                            {
                                Curve3D uiso = TheSurf.UIso(uf1);
                                var Projector = new GeomAPI_ProjectPointOnCurve(Pint1, uiso);
                                double NewLastV = Projector.LowerDistanceParameter();
                                TheSurf = new TrimmedSurface(TheSurf, uf1, uf2, vf1, NewLastV);
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

        var myBuilder = new BrepBuilder();
        myBuilder.MakeFace(ref myFace);
        if (!IsTransformed)
        {
            myBuilder.UpdateFace(myFace, TheSurf, L, Tool.Tolerance(Face));
        }
        else
        {
            myBuilder.UpdateFace(myFace, TheSurf, new Placement(), Tool.Tolerance(Face));
        }

        var MapSS = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);

        // mise a jour de la map sur les vertex deja crees
        Shape aLocalShapeOriented = Face.Oriented(Forward);
        Face CurFace = ShapeCasts.Face(aLocalShapeOriented);
        //  TopoDS_Face CurFace = TopoDS::Face(Face.Oriented(TopAbs_FORWARD));

        var VonDegen = new NCollection_Map<Shape>(ShapeHasher.Instance);
        TheSurf.Bounds(out double u1, out double u2, out double v1, out double v2);

        var exp = new Explorer(CurFace, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge E = ShapeCasts.Edge(exp.Current());
            Exploration.Vertices(E, out Vertex V1, out Vertex V2);
            if (HasSingularity && Tool.Degenerated(E))
            {
                VonDegen.Add(V1);
            }
            if (Created.IsBound(E))
            {
                Edge OE = ShapeCasts.Edge(Created[E]);
                Exploration.Vertices(OE, out Vertex OV1, out Vertex OV2);
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

        var expw = new Explorer(CurFace, ShapeType.Wire);
        for (; expw.More(); expw.Next())
        {
            Wire W = ShapeCasts.Wire(expw.Current());
            var expe = new Explorer(W.Oriented(Forward), ShapeType.Edge);
            Wire OW = new();
            myBuilder.MakeWire(ref OW);
            for (; expe.More(); expe.Next())
            {
                Edge E = ShapeCasts.Edge(expe.Current());
                Exploration.Vertices(E, out Vertex V1, out Vertex V2);
                Point2D P2d1 = new(), P2d2 = new();
                Point3D P1, P2;
                double vstart, vend;
                double f, l;
                Curve2D C2d = Tool.CurveOnSurface(E, CurFace, out f, out l)!;
                Edge OE = new();
                if (MapSS.IsBound(E) && !VonDegen.Contains(V1) && !VonDegen.Contains(V2))
                { // c`est un edge de couture
                    OE = ShapeCasts.Edge(MapSS[E]);
                    Shape aLocalShape = E.Reversed();
                    Curve2D? C2d_1 = Tool.CurveOnSurface(ShapeCasts.Edge(aLocalShape), CurFace, out f, out l);
                    //	occ::handle<Geom2d_Curve> C2d_1 =
                    //	  BRep_Tool::CurveOnSurface(TopoDS::Edge(E.Reversed()),CurFace,f,l);
                    if (E.Orientation() == Forward)
                    {
                        UpdateEdge(OE, C2d, C2d_1, myFace, Tool.Tolerance(E));
                    }
                    else
                    {
                        UpdateEdge(OE, C2d_1, C2d, myFace, Tool.Tolerance(E));
                    }
                    myBuilder.Range(OE, f, l);
                }
                else
                {
                    Shape aLocalShape = E.Oriented(Forward);
                    Edge Eforward = ShapeCasts.Edge(aLocalShape);
                    P2d1 = C2d.Value(Tool.Parameter(V1, Eforward, CurFace));
                    P2d2 = C2d.Value(Tool.Parameter(V2, Eforward, CurFace));
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
                        OE = ShapeCasts.Edge(Created[E]);
                    }
                    else if (MapSS.IsBound(E))
                    { // seam edge
                        OE = ShapeCasts.Edge(MapSS[E]);
                    }
                    else
                    {
                        myBuilder.MakeEdge(ref OE);
                        Vertex OV1 = new(), OV2 = new();
                        if (MapSS.IsBound(V1))
                        {
                            OV1 = ShapeCasts.Vertex(MapSS[V1]);
                        }
                        else
                        {
                            myBuilder.MakeVertex(ref OV1);
                            myBuilder.UpdateVertex(OV1, P1, Tool.Tolerance(V1));
                            MapSS.Bind(V1, OV1);
                        }
                        if (MapSS.IsBound(V2))
                        {
                            OV2 = ShapeCasts.Vertex(MapSS[V2]);
                        }
                        else
                        {
                            myBuilder.MakeVertex(ref OV2);
                            myBuilder.UpdateVertex(OV2, P2, Tool.Tolerance(V2));
                            MapSS.Bind(V2, OV2);
                        }
                        myBuilder.Add(OE, OV1.Oriented(V1.Orientation()));
                        myBuilder.Add(OE, OV2.Oriented(V2.Orientation()));
                        if (Tool.Degenerated(E))
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
                        C2d = new LineCurve2D(P2d1, new Direction2D(new Vector2D(P2d1, P2d2)));
                        f = 0.0;
                        l = P2d1.Distance(P2d2);
                        if (MapSS.IsBound(E)) // seam edge
                        {
                            Curve2D? C2d_1 = Tool.CurveOnSurface(OE, myFace, out f, out l);
                            if (E.Orientation() == Forward)
                            {
                                UpdateEdge(OE, C2d, C2d_1, myFace, Tool.Tolerance(E));
                            }
                            else
                            {
                                UpdateEdge(OE, C2d_1, C2d, myFace, Tool.Tolerance(E));
                            }
                        }
                        else
                        {
                            UpdateEdge(OE, C2d, myFace, Tool.Tolerance(E));
                        }
                        // myBuilder.Range(OE,f,l);
                        myBuilder.Range(OE, myFace, f, l);
                        if (!Tool.Degenerated(E) && TheSurf.IsUClosed)
                        {
                            Shape aLocalShapeReversedE = E.Reversed();
                            Curve2D C2d_1 =
                                Tool.CurveOnSurface(ShapeCasts.Edge(aLocalShapeReversedE), CurFace, out f, out l)!;
                            P2d1 = C2d_1.Value(Tool.Parameter(V1, E, CurFace));
                            P2d2 = C2d_1.Value(Tool.Parameter(V2, E, CurFace));
                            if (VonDegen.Contains(V1))
                            {
                                P2d1.SetY(vstart);
                            }
                            if (VonDegen.Contains(V2))
                            {
                                P2d2.SetY(vend);
                            }
                            C2d_1 = new LineCurve2D(P2d1, new Direction2D(new Vector2D(P2d1, P2d2)));
                            if (E.Orientation() == Forward)
                            {
                                UpdateEdge(OE, C2d, C2d_1, myFace, Tool.Tolerance(E));
                            }
                            else
                            {
                                UpdateEdge(OE, C2d_1, C2d, myFace, Tool.Tolerance(E));
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
                        UpdateEdge(OE, C2d, myFace, Tool.Tolerance(E));
                        myBuilder.Range(OE, f, l);
                        // ComputeCurve3d(OE,C2d,TheSurf,L,BRep_Tool::Tolerance(E));
                    }
                    if (!Tool.Degenerated(OE))
                    {
                        ComputeCurve3d(OE, C2d, TheSurf, L, Tool.Tolerance(E));
                    }
                    MapSS.Bind(E, OE);
                }
                myBuilder.Add(OW, OE.Oriented(E.Orientation()));
            }
            myBuilder.Add(myFace, OW.Oriented(W.Orientation()));
        }

        myFace.Orientation(Face.Orientation());

        BrepTools.Update(myFace);
    }

    //=================================================================================================

    /// <summary>
    /// Builds the pipe of radius <paramref name="Offset"/> along <paramref name="Path"/> between
    /// <paramref name="Edge1"/> and <paramref name="Edge2"/>, which get pcurves on it.
    /// </summary>
    public void Init(in Edge Path, in Edge Edge1, in Edge Edge2, double Offset,
                     bool Polynomial = false, double Tol = 1.0e-4, Continuity Conti = C1)
    {
        Edge FirstEdge = new(), LastEdge = new();
        Init(Path, Edge1, Edge2, Offset, FirstEdge, LastEdge, Polynomial, Tol, Conti);
    }

    //=================================================================================================

    /// <summary>
    /// Builds the pipe of radius <paramref name="Offset"/> along <paramref name="Path"/> between
    /// <paramref name="Edge1"/> and <paramref name="Edge2"/>, closed at its ends by <paramref name="FirstEdge"/> and
    /// <paramref name="LastEdge"/> (or new edges when null). The given edges get pcurves on the pipe: in place when
    /// they are unfrozen, else on copies (see the class remarks).
    /// </summary>
    public void Init(in Edge Path, in Edge Edge1, in Edge Edge2, double Offset,
                     in Edge FirstEdge, in Edge LastEdge, bool Polynomial = false, double Tol = 1.0e-4,
                     Continuity Conti = C1)
    {
        // OCCT.NET: copy at entry (plan 2.2) of the edges written below, when one of them holds a frozen shape.
        CopyAtEntry? anEntry = CopyAtEntry([Edge1, Edge2, FirstEdge, LastEdge]);
        Edge aEdge1 = Copied(anEntry, Edge1);
        Edge aEdge2 = Copied(anEntry, Edge2);
        Edge aFirstEdge = Copied(anEntry, FirstEdge);
        Edge aLastEdge = Copied(anEntry, LastEdge);
        initPipe(Path, aEdge1, aEdge2, Offset, aFirstEdge, aLastEdge, Polynomial, Tol, Conti);
        if (anEntry is not null)
        {
            myFace = ShapeCasts.Face(anEntry.Finish(myFace));
        }
    }

    // The body of OCCT's Init(Path, Edge1, Edge2, Offset, FirstEdge, LastEdge, ...), on the edges to write.
    private void initPipe(in Edge Path, in Edge Edge1, in Edge Edge2, double Offset,
                          in Edge FirstEdge, in Edge LastEdge, bool Polynomial, double Tol,
                          Continuity Conti)
    {
        bool C1Denerated = false;
        bool C2Denerated = false;
        myStatus = BRepOffset_Good;
        myShape = Path;

        Placement Loc;
        var f = new double[3];
        var l = new double[3];

        Curve3D CP = Tool.Curve(Path, out Loc, out f[0], out l[0])!;
        CP = new TrimmedCurve3D(CP, f[0], l[0]);
        CP.Transform(Loc.Transformation());
        var HCP = new CurveAdaptor(CP);

        Curve3D? C1 = Tool.Curve(Edge1, out Loc, out f[1], out l[1]);

        CurveAdaptorBase HEdge1;
        bool C1is3D = true;
        if (C1 is null)
        {
            C1is3D = false;
            Tool.CurveOnSurface(Edge1, out Curve2D? C12d, out Surface? S1, out Loc, out f[1], out l[1]);
            S1 = (Surface)S1!.Transformed(Loc.Transformation());
            C12d = new TrimmedCurve2D(C12d!, f[1], l[1]);
            var HS1 = new SurfaceAdaptor(S1);
            var HC1 = new CurveAdaptor2D(C12d);
            // OCCT.NET: OCCT copies a local Adaptor3d_CurveOnSurface Cons(HC1, HS1)
            HEdge1 = new CurveOnSurface(HC1, HS1);
        }
        else
        {
            C1 = new TrimmedCurve3D(C1, f[1], l[1]);
            C1.Transform(Loc.Transformation());
            HEdge1 = new CurveAdaptor(C1);
            var AC1 = new CurveAdaptor(C1);
            if (AC1.GetType() == Circle)
            {
                C1Denerated = (AC1.Circle().Radius() < global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }

        Curve3D? C2 = Tool.Curve(Edge2, out Loc, out f[2], out l[2]);

        CurveAdaptorBase HEdge2;
        bool C2is3D = true;
        if (C2 is null)
        {
            C2is3D = false;
            Tool.CurveOnSurface(Edge2, out Curve2D? C12d, out Surface? S1, out Loc, out f[2], out l[2]);
            S1 = (Surface)S1!.Transformed(Loc.Transformation());
            C12d = new TrimmedCurve2D(C12d!, f[2], l[2]);
            var HS1 = new SurfaceAdaptor(S1);
            var HC1 = new CurveAdaptor2D(C12d);
            // OCCT.NET: OCCT copies a local Adaptor3d_CurveOnSurface Cons(HC1, HS1)
            HEdge2 = new CurveOnSurface(HC1, HS1);
        }
        else
        {
            C2 = new TrimmedCurve3D(C2, f[2], l[2]);
            C2.Transform(Loc.Transformation());
            HEdge2 = new CurveAdaptor(C2);
            var AC2 = new CurveAdaptor(C2);
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

        Surface S = Pipe.InternalSurface()!;
        bool ExchUV = Pipe.ExchangeUV();
        S.Bounds(out double f1, out double l1, out double f2, out double l2);

        // Perform the face
        double PathTol = Tool.Tolerance(Path);
        double TheTol;
        var myBuilder = new BrepBuilder();
        myBuilder.MakeFace(ref myFace);
        var Id = new Placement();
        myBuilder.UpdateFace(myFace, S, Id, PathTol);

        // update de Edge1. (Rem : has already a 3d curve)
        double U, U1, U2;
        Curve2D PC;
        if (ExchUV)
        {
            PC = new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.X));
            U1 = f1;
            U2 = l1;
            if (!C1is3D)
            {
                C1 = S.VIso(f2);
            }
        }
        else
        {
            PC = new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y));
            U1 = f2;
            U2 = l2;
            if (!C1is3D)
            {
                C1 = S.UIso(f1);
            }
        }

        Curve3D? Dummy = null;
        if (!C1is3D)
        {
            UpdateEdge(Edge1, C1, Id, Tool.Tolerance(Edge1));
        }
        else if (C1Denerated)
        {
            UpdateEdge(Edge1, Dummy, Id, Tool.Tolerance(Edge1));
            myBuilder.Degenerated(Edge1, true);
        }

        TheTol = Max(PathTol, Tool.Tolerance(Edge1) + ErrorPipe);
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
            PC = new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.X));
            U1 = f1;
            U2 = l1;
            if (!C2is3D)
            {
                C2 = S.VIso(l2);
            }
        }
        else
        {
            PC = new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y));
            U1 = f2;
            U2 = l2;
            if (!C2is3D)
            {
                C2 = S.UIso(l1);
            }
        }

        if (!C2is3D)
        {
            UpdateEdge(Edge2, C2, Id, Tool.Tolerance(Edge2));
        }
        else if (C2Denerated)
        {
            UpdateEdge(Edge2, Dummy, Id, Tool.Tolerance(Edge2));
            myBuilder.Degenerated(Edge2, true);
        }

        TheTol = Max(PathTol, Tool.Tolerance(Edge2) + ErrorPipe);
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

        Edge Edge3 = new(), Edge4 = new();
        // eval edge3
        Exploration.Vertices(Path, out Vertex V1f, out Vertex V1l);
        bool IsClosed = (V1f.IsSame(V1l));

        Exploration.Vertices(Edge1, out V1f, out V1l);
        Exploration.Vertices(Edge2, out Vertex V2f, out Vertex V2l);

        bool StartDegenerated = (V1f.IsSame(V2f));
        bool EndDegenerated = (V1l.IsSame(V2l));

        bool E3rev = false;
        bool E4rev = false;

        Vertex VVf, VVl;
        if (FirstEdge.IsNull)
        {
            myBuilder.MakeEdge(ref Edge3);
            myBuilder.Add(Edge3, V1f.Oriented(Forward));
            myBuilder.Add(Edge3, V2f.Oriented(Reversed));
        }
        else
        {
            Shape aLocalEdge = FirstEdge.Oriented(Forward);
            Edge3 = ShapeCasts.Edge(aLocalEdge);
            //    Edge3 = TopoDS::Edge(FirstEdge.Oriented(TopAbs_FORWARD));
            Exploration.Vertices(Edge3, out VVf, out VVl);
            // OCCT_DEBUG warning on unshared vertices left out.
            if (!VVf.IsSame(V1f) && !VVf.IsSame(V2f))
            {
                // On fait vraisemblablement des conneries !!
                // On cree un autre edge, on appelle le Sewing apres.
                myBuilder.MakeEdge(ref Edge3);
                myBuilder.Add(Edge3, V1f.Oriented(Forward));
                myBuilder.Add(Edge3, V2f.Oriented(Reversed));
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

        LineCurve2D L1, L2;
        if (IsClosed)
        {
            if (ExchUV)
            {
                // rem : si ExchUv, il faut  reverser le Wire.
                // donc l'edge Forward dans la face sera E4 : d'ou L1 et L2
                L2 = new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y));
                L1 = new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y));
                U1 = f2;
                U2 = l2;
            }
            else
            {
                L1 = new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.X));
                L2 = new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.X));
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
            else if (FirstEdge.IsNull)
            { // then the 3d curve has not been yet computed
                ComputeCurve3d(Edge3, L1, S, Id, TolApp);
            }
        }
        else
        {
            if (LastEdge.IsNull)
            {
                myBuilder.MakeEdge(ref Edge4);
                myBuilder.Add(Edge4, V1l.Oriented(Forward));
                myBuilder.Add(Edge4, V2l.Oriented(Reversed));
            }
            else
            {
                Shape aLocalEdge = LastEdge.Oriented(Forward);
                Edge4 = ShapeCasts.Edge(aLocalEdge);
                //      Edge4 = TopoDS::Edge(LastEdge.Oriented(TopAbs_FORWARD));
                Exploration.Vertices(Edge4, out VVf, out VVl);
                // OCCT_DEBUG warning on unshared vertices left out.
                if (!VVf.IsSame(V1l) && !VVf.IsSame(V2l))
                {
                    // On fait vraisemblablement des conneries !!
                    // On cree un autre edge, on appelle le Sewing apres.
                        myBuilder.MakeEdge(ref Edge4);
                    myBuilder.Add(Edge4, V1l.Oriented(Forward));
                    myBuilder.Add(Edge4, V2l.Oriented(Reversed));
                }
                else if (!VVf.IsSame(V1l))
                {
                    Edge4.Reverse();
                    E4rev = true;
                }
            }

            if (ExchUV)
            {
                L1 = new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y));
                U1 = f2;
                U2 = l2;
            }
            else
            {
                L1 = new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.X));
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
            else if (FirstEdge.IsNull)
            { // then the 3d curve has not been yet computed
                ComputeCurve3d(Edge3, L1, S, Id, TolApp);
            }

            if (ExchUV)
            {
                L2 = new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y));
                U1 = f2;
                U2 = l2;
            }
            else
            {
                L2 = new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.X));
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
            else if (LastEdge.IsNull)
            { // then the 3d curve has not been yet computed
                ComputeCurve3d(Edge4, L2, S, Id, TolApp);
            }
        }

        // SameParameter ??
        if (!FirstEdge.IsNull && !StartDegenerated)
        {
            BRepLib.BuildCurve3d(Edge3, PathTol);
            myBuilder.SameRange(Edge3, false);
            myBuilder.SameParameter(Edge3, false);
            BRepLib.SameParameter(Edge3, Tol);
        }
        if (!LastEdge.IsNull && !EndDegenerated)
        {
            BRepLib.BuildCurve3d(Edge4, PathTol);
            myBuilder.SameRange(Edge4, false);
            myBuilder.SameParameter(Edge4, false);
            BRepLib.SameParameter(Edge4, Tol);
        }

        Wire W = new();
        myBuilder.MakeWire(ref W);

        myBuilder.Add(W, Edge1.Oriented(Reversed));
        myBuilder.Add(W, Edge2.Oriented(Forward));
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

        BrepTools.Update(myFace);

        if (Edge1.Orientation() == Reversed)
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
    public void Init(in Vertex Vertex, NCollection_List<Shape> LEdge, double Offset,
                     bool Polynomial = false, double Tol = 1.0e-4, Continuity Conti = C1)
    {
        // OCCT.NET: copy at entry (plan 2.2) of the edges written below, when one of them holds a frozen shape.
        var aShapes = new List<Shape>();
        for (var anIt = new NCollection_List<Shape>.Iterator(LEdge); anIt.More(); anIt.Next())
        {
            aShapes.Add(anIt.Value());
        }
        CopyAtEntry? anEntry = CopyAtEntry(aShapes);
        NCollection_List<Shape> aLEdge = LEdge;
        if (anEntry is not null)
        {
            aLEdge = new NCollection_List<Shape>();
            foreach (Shape anEdge in aShapes)
            {
                aLEdge.Append(anEntry.Copied(anEdge));
            }
        }
        initSphere(Vertex, aLEdge, Offset, Polynomial, Tol, Conti);
        if (anEntry is not null)
        {
            myFace = ShapeCasts.Face(anEntry.Finish(myFace));
        }
    }

    // The body of OCCT's Init(Vertex, LEdge, Offset, Polynomial, TolApp, Conti), on the edges to write.
    private void initSphere(in Vertex Vertex, NCollection_List<Shape> LEdge, double Offset,
                            bool Polynomial, double TolApp, Continuity Conti)
    {
        myStatus = BRepOffset_Good;
        myShape = Vertex;

        // evaluate the Ax3 of the Sphere
        // find 3 different vertices in LEdge
        var it = new NCollection_List<Shape>.Iterator();

        // OCCT_DEBUG naming of the shapes (Affich) left out.

        Point3D Origin = Tool.Pnt(Vertex);

        //// Find the axis of the sphere to exclude
        //// degenerated and seam edges from the face under construction
        var MW = new BRepLib_MakeWire();
        MW.Add(LEdge);
        Wire theWire = MW.InternalWire();

        var Fixer = new ShapeFix_Shape(theWire);
        Fixer.Perform();
        theWire = ShapeCasts.Wire(Fixer.InternalShape());

        var GlobalProps = new Properties();
        BRepGProp.LinearProperties(theWire, GlobalProps);
        Point3D BaryCenter = GlobalProps.CentreOfMass();
        var Xdir = new Vector3D(BaryCenter, Origin);

        Point3D FarestCorner = GetFarestCorner(theWire);
        Plane3D thePlane = new MakePln(Origin, BaryCenter, FarestCorner).Value();
        Direction3D Vdir = thePlane.Axis().Direction();

        var Axis = new CoordinateSystem(Origin, Vdir, new Direction3D(Xdir));

        Surface S = new SphericalSurface(Axis, Math.Abs(Offset));

        double f, l, Tol = Tool.Tolerance(Vertex);

        Placement Loc = new();
        var myBuilder = new BrepBuilder();
        myBuilder.MakeFace(ref myFace);
        Surface SS = S;

        // En polynomial, calcul de la surface par F(u,v).
        // Pas de changement de parametre, donc ProjLib sur la Sphere
        // reste OK.
        if (Polynomial)
        {
            var Approx = new ApproxSurface(S, TolApp, Conti, Conti, 10, 10, 10, 1);
            if (Approx.IsDone)
            {
                SS = Approx.Surface()!;
            }
        }

        myBuilder.UpdateFace(myFace, SS, Loc, Tol);

        Wire W = new();
        myBuilder.MakeWire(ref W);

        for (it.Initialize(LEdge); it.More(); it.Next())
        {
            Edge E = ShapeCasts.Edge(it.Value());

            Curve3D? C = Tool.Curve(E, out Loc, out f, out l);
            if (C is null)
            {
                BRepLib.BuildCurve3d(E, Tool.Tolerance(E));
                C = Tool.Curve(E, out Loc, out f, out l);
            }
            C = new TrimmedCurve3D(C!, f, l);
            C.Transform(Loc.Transformation());

            Curve2D PCurve = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(C, S)!;
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
            myBuilder.Add(myFace, W.Oriented(Reversed));
            myFace.Reverse();
        }
        else
        {
            myBuilder.Add(myFace, W);
        }

        BrepTools.Update(myFace);
    }

    //=================================================================================================

    /// <summary>Builds the pipe of radius |<paramref name="Offset"/>| around the free boundary
    /// <paramref name="Edge"/> (used only by the rolling ball).</summary>
    public void Init(in Edge Edge, double Offset)
    {
        myShape = Edge;
        double myOffset = Math.Abs(Offset);

        Curve3D CP = Tool.Curve(Edge, out Placement Loc, out double f, out double l)!;
        CP = new TrimmedCurve3D(CP, f, l);
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
    public Shape InitialShape() => myShape;

    //=================================================================================================

    /// <summary>The offset face (frozen).</summary>
    public Face Face()
    {
        myFace.TShape()?.FreezeGraph();
        return myFace;
    }

    /// <summary><see cref="Face"/> without freezing, for ported code.</summary>
    internal Face InternalFace() => myFace;

    //=================================================================================================

    /// <summary>
    /// The shape generated from a sub-shape of the initial shape (frozen): for a face, the offset of one of its edges;
    /// for a path edge, the end edge of the pipe at one of its vertices.
    /// </summary>
    public Shape Generated(in Shape Shape)
    {
        Shape aShape = InternalGenerated(Shape);
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary><see cref="Generated"/> without freezing, for ported code.</summary>
    internal Shape InternalGenerated(in Shape Shape)
    {
        Shape aShape = new();

        switch (myShape.ShapeType())
        {
            case global::Kerf.Topology.ShapeType.Face:
            {
                var exp = new Explorer(myShape.Oriented(Forward), ShapeType.Edge);
                var expo = new Explorer(myFace.Oriented(Forward), ShapeType.Edge);
                for (; exp.More() && expo.More(); exp.Next(), expo.Next())
                {
                    if (Shape.IsSame(exp.Current()))
                    {
                        if (myShape.Orientation() == Reversed)
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

            case ShapeType.Edge:
                // have generate a pipe.
            {
                Exploration.Vertices(ShapeCasts.Edge(myShape), out Vertex V1, out Vertex V2);

                var expf = new Explorer(myFace.Oriented(Forward), ShapeType.Wire);
                var expo = new Explorer(expf.Current().Oriented(Forward), ShapeType.Edge);
                expo.Next();
                expo.Next();

                if (V2.IsSame(Shape))
                {
                    if (expf.Current().Orientation() == Reversed)
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
                    if (expf.Current().Orientation() == Reversed)
                    {
                        aShape = expo.Current().Reversed();
                    }
                    else
                    {
                        aShape = expo.Current();
                    }
                }
                if (myFace.Orientation() == Reversed)
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
    private static CopyAtEntry? CopyAtEntry(IReadOnlyList<Shape> theShapes)
    {
        bool hasFrozen = false;
        foreach (Shape aShape in theShapes)
        {
            if (!aShape.IsNull && BrepTools.HoldsFrozen(aShape))
            {
                hasFrozen = true;
                break;
            }
        }
        if (!hasFrozen)
        {
            return null;
        }

        var aB = new BrepBuilder();
        Compound aCompound = new();
        aB.MakeCompound(ref aCompound);
        foreach (Shape aShape in theShapes)
        {
            if (!aShape.IsNull)
            {
                aB.Add(aCompound, aShape);
            }
        }
        return new CopyAtEntry(aCompound, null, true);
    }

    // The copy of an edge given to a pipe (the edge itself when nothing was copied, or when it is null).
    private static Edge Copied(CopyAtEntry? theEntry, in Edge theEdge)
    {
        if (theEntry is null || theEdge.IsNull)
        {
            return theEdge;
        }
        return ShapeCasts.Edge(theEntry.Copied(theEdge));
    }
}
