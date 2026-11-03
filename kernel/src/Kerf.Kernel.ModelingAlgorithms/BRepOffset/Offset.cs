// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Offset.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Offset.lxx
// Port of: src/ModelingAlgorithms/TKOffset/BRepOffset/BRepOffset_Offset.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Offsets.OffsetStatus;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Offsets;

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
public sealed class Offset
{
    private Shape _shape = new();
    private OffsetStatus _status = OffsetStatus.Unknown;
    private Face _face = new();
    private readonly Dictionary<Shape, Shape> _map = new(ShapeHasher.Instance);

    // OCCT_DEBUG statics (Affich, NbOFFSET) and their blocks are left out.

    private static Point3D GetFarestCorner(in Wire aWire)
    {
        var vertices = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aWire, ShapeType.Vertex, vertices);

        double maxDist = 0.0;
        Point3D thePoint = new();
        for (int i = 0; i < vertices.Count; i++)
        {
            for (int j = 0; j < vertices.Count; j++)
            {
                Vertex v1 = ShapeCasts.Vertex(vertices[i]);
                Vertex v2 = ShapeCasts.Vertex(vertices[j]);
                Point3D p1 = Tool.Pnt(v1);
                Point3D p2 = Tool.Pnt(v2);
                double dist = p1.SquareDistance(p2);
                if (dist > maxDist)
                {
                    maxDist = dist;
                    thePoint = p1;
                }
            }
        }

        return thePoint;
    }

    //=================================================================================================

    private static void UpdateEdge(in Edge e, Curve3D? c, in Placement l, double tol)
    {
        // Cut curves to avoid copies in the extensions.
        var b = new BrepBuilder();
        if (c is TrimmedCurve3D bc)
        {
            b.UpdateEdge(e, bc.BasisCurve(), l, tol);
        }
        else
        {
            b.UpdateEdge(e, c, l, tol);
        }
    }

    //=================================================================================================

    private static void UpdateEdge(in Edge e, Curve2D? c, in Face f, double tol)
    {
        // Cut curves to avoid copies in the extensions.
        var b = new BrepBuilder();
        if (c is TrimmedCurve2D bc)
        {
            b.UpdateEdge(e, bc.BasisCurve(), f, tol);
        }
        else
        {
            b.UpdateEdge(e, c, f, tol);
        }
    }

    //=================================================================================================

    private static void UpdateEdge(in Edge e, Curve2D? c1, Curve2D? c2, in Face f, double tol)
    {
        // Cut curves to avoid copies in the extensions.
        var b = new BrepBuilder();
        Curve2D? nc1, nc2;
        if (c1 is TrimmedCurve2D bc1)
        {
            nc1 = bc1.BasisCurve();
        }
        else
        {
            nc1 = c1;
        }
        if (c2 is TrimmedCurve2D bc2)
        {
            nc2 = bc2.BasisCurve();
        }
        else
        {
            nc2 = c2;
        }
        b.UpdateEdge(e, nc1, nc2, f, tol);
    }

    //=======================================================================
    // function : ComputeCurve3d
    // purpose  : Particular case of Curve On Surface.
    //=======================================================================

    private static void ComputeCurve3d(in Edge edge, Curve2D curve, Surface surf, in Placement loc,
                                       double tol)
    {
        // try to find the particular case
        // if not found call BRepLib::BuildCurve3d

        bool isComputed = false;

        // Search only isos on analytic surfaces.
        var c = new CurveAdaptor2D(curve);
        var s = new SurfaceAdaptor(surf);
        CurveKind cTy = c.GetType();
        SurfaceKind sTy = s.GetType();
        var theBuilder = new BrepBuilder();

        if (sTy != Plane)
        { // if plane buildcurve3d manage KPart
            if (cTy == Line)
            {
                Direction2D d = c.Line().Direction();
                if (d.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                { // Iso V.
                    if (sTy == SurfaceKind.Sphere)
                    {
                        Point2D p = c.Line().Location();
                        if (Math.Abs(Math.Abs(p.Y()) - M_PI / 2.0) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            theBuilder.Degenerated(edge, true);
                        }
                        else
                        {
                            Sphere sph = s.Sphere();
                            CoordinateSystem axis = sph.Position();
                            Circle3D ci = global::Kerf.Engine.Geometry.ElSLib.SphereVIso(axis, sph.Radius(), p.Y());
                            Direction3D dRev = axis.XDirection().Crossed(axis.YDirection());
                            var axeRev = new Axis3D(axis.Location(), dRev);
                            ci = ci.Rotated(axeRev, p.X());
                            CircleCurve3D circle = new CircleCurve3D(ci);
                            if (d.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                circle.Reverse();
                            }
                            UpdateEdge(edge, circle, loc, tol);
                        }
                        isComputed = true;
                    }
                    else if (sTy == SurfaceKind.Cylinder)
                    {
                        Cylinder cyl = s.Cylinder();
                        Point2D p = c.Line().Location();
                        CoordinateSystem axis = cyl.Position();
                        Circle3D ci = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(axis, cyl.Radius(), p.Y());
                        Direction3D dRev = axis.XDirection().Crossed(axis.YDirection());
                        var axeRev = new Axis3D(axis.Location(), dRev);
                        ci = ci.Rotated(axeRev, p.X());
                        CircleCurve3D circle = new CircleCurve3D(ci);
                        if (d.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            circle.Reverse();
                        }
                        UpdateEdge(edge, circle, loc, tol);
                        isComputed = true;
                    }
                    else if (sTy == SurfaceKind.Cone)
                    {
                        Cone cone = s.Cone();
                        Point2D p = c.Line().Location();
                        CoordinateSystem axis = cone.Position();
                        Circle3D ci = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(axis, cone.RefRadius(), cone.SemiAngle(), p.Y());
                        Direction3D dRev = axis.XDirection().Crossed(axis.YDirection());
                        var axeRev = new Axis3D(axis.Location(), dRev);
                        ci = ci.Rotated(axeRev, p.X());
                        CircleCurve3D circle = new CircleCurve3D(ci);
                        if (d.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            circle.Reverse();
                        }
                        UpdateEdge(edge, circle, loc, tol);
                        isComputed = true;
                    }
                    else if (sTy == SurfaceKind.Torus)
                    {
                        Torus tore = s.Torus();
                        Point2D p = c.Line().Location();
                        CoordinateSystem axis = tore.Position();
                        Circle3D ci = global::Kerf.Engine.Geometry.ElSLib.TorusVIso(axis, tore.MajorRadius(), tore.MinorRadius(), p.Y());
                        Direction3D dRev = axis.XDirection().Crossed(axis.YDirection());
                        var axeRev = new Axis3D(axis.Location(), dRev);
                        ci = ci.Rotated(axeRev, p.X());
                        CircleCurve3D circle = new CircleCurve3D(ci);
                        if (d.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            circle.Reverse();
                        }
                        UpdateEdge(edge, circle, loc, tol);
                        isComputed = true;
                    }
                }
                else if (d.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                { // Iso U.
                    if (sTy == SurfaceKind.Sphere)
                    {
                        Sphere sph = s.Sphere();
                        Point2D p = c.Line().Location();
                        CoordinateSystem axis = sph.Position();
                        // calculate iso 0.
                        Circle3D ci = global::Kerf.Engine.Geometry.ElSLib.SphereUIso(axis, sph.Radius(), 0.0);

                        // set to sameparameter (rotation of circle - offset of Y)
                        Direction3D dRev = axis.XDirection().Crossed(axis.Direction());
                        var axeRev = new Axis3D(axis.Location(), dRev);
                        ci = ci.Rotated(axeRev, p.Y());

                        // transformation en iso U ( = P.X())
                        dRev = axis.XDirection().Crossed(axis.YDirection());
                        axeRev = new Axis3D(axis.Location(), dRev);
                        ci = ci.Rotated(axeRev, p.X());
                        CircleCurve3D circle = new CircleCurve3D(ci);

                        if (d.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            circle.Reverse();
                        }
                        UpdateEdge(edge, circle, loc, tol);
                        isComputed = true;
                    }
                    else if (sTy == SurfaceKind.Cylinder)
                    {
                        Cylinder cyl = s.Cylinder();
                        Point2D p = c.Line().Location();
                        Line3D l = global::Kerf.Engine.Geometry.ElSLib.CylinderUIso(cyl.Position(), cyl.Radius(), p.X());
                        var tr = new Vector3D(l.Direction());
                        tr = tr.Multiplied(p.Y());
                        l = l.Translated(tr);
                        LineCurve3D line = new LineCurve3D(l);
                        if (d.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            line.Reverse();
                        }
                        UpdateEdge(edge, line, loc, tol);
                        isComputed = true;
                    }
                    else if (sTy == SurfaceKind.Cone)
                    {
                        Cone cone = s.Cone();
                        Point2D p = c.Line().Location();
                        Line3D l = global::Kerf.Engine.Geometry.ElSLib.ConeUIso(cone.Position(), cone.RefRadius(), cone.SemiAngle(), p.X());
                        var tr = new Vector3D(l.Direction());
                        tr = tr.Multiplied(p.Y());
                        l = l.Translated(tr);
                        LineCurve3D line = new LineCurve3D(l);
                        if (d.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            line.Reverse();
                        }
                        UpdateEdge(edge, line, loc, tol);
                        isComputed = true;
                    }
                    else if (sTy == SurfaceKind.Torus)
                    {
                        Torus tore = s.Torus();
                        Point2D p = c.Line().Location();
                        CoordinateSystem axis = tore.Position();
                        Circle3D ci = global::Kerf.Engine.Geometry.ElSLib.TorusUIso(axis, tore.MajorRadius(), tore.MinorRadius(), p.X());
                        ci = ci.Rotated(ci.Axis(), p.Y());
                        CircleCurve3D circle = new CircleCurve3D(ci);

                        if (d.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            circle.Reverse();
                        }
                        UpdateEdge(edge, circle, loc, tol);
                        isComputed = true;
                    }
                }
            }
        }
        else
        { // Cas Plan
            Curve3D c3d = Projections.To3d(curve, s.Plane());
            UpdateEdge(edge, c3d, loc, tol);
            isComputed = true;
        }
        if (!isComputed)
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
    public Offset()
    {
    }

    //=================================================================================================

    /// <summary>The offset of <paramref name="face"/> by <paramref name="offset"/>.</summary>
    public Offset(in Face face, double offset, bool offsetOutside = true,
                             JoinKind joinType = Arc) =>
        Init(face, offset, offsetOutside, joinType);

    //=================================================================================================

    /// <summary>
    /// The offset of <paramref name="face"/>, sharing the edges already generated from other faces:
    /// <c>Created(E) = E'</c>, where E is an edge (or vertex) of the face and E' its image in the offset of another face
    /// sharing E with at least G1 continuity (so two tangent faces generate one edge, no pipe). The edges E' are written
    /// into and must be unfrozen.
    /// </summary>
    public Offset(in Face face, double offset, Dictionary<Shape, Shape> created,
                             bool offsetOutside = true, JoinKind joinType = Arc) =>
        Init(face, offset, created, offsetOutside, joinType);

    //=================================================================================================

    /// <summary>The pipe of radius <paramref name="offset"/> along <paramref name="path"/> between two edges.</summary>
    public Offset(in Edge path, in Edge edge1, in Edge edge2, double offset,
                             bool polynomial = false, double tol = 1.0e-4, Continuity conti = C1) =>
        Init(path, edge1, edge2, offset, polynomial, tol, conti);

    //=================================================================================================

    /// <summary>The pipe along <paramref name="path"/> between two edges, closed by the given end edges.</summary>
    public Offset(in Edge path, in Edge edge1, in Edge edge2, double offset,
                             in Edge firstEdge, in Edge lastEdge, bool polynomial = false,
                             double tol = 1.0e-4, Continuity conti = C1) =>
        Init(path, edge1, edge2, offset, firstEdge, lastEdge, polynomial, tol, conti);

    //=================================================================================================

    /// <summary>
    /// The sphere of radius <paramref name="offset"/> at <paramref name="vertex"/> bounded by the edges
    /// <paramref name="lEdge"/>. <paramref name="tol"/> and <paramref name="conti"/> are used only if
    /// <paramref name="polynomial"/> (for the approximation).
    /// </summary>
    public Offset(in Vertex vertex, NCollection_List<Shape> lEdge, double offset,
                             bool polynomial = false, double tol = 1.0e-4, Continuity conti = C1) =>
        Init(vertex, lEdge, offset, polynomial, tol, conti);

    //=================================================================================================

    /// <summary>Builds the offset of <paramref name="face"/> by <paramref name="offset"/>.</summary>
    public void Init(in Face face, double offset, bool offsetOutside = true,
                     JoinKind joinType = Arc)
    {
        var empty = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        Init(face, offset, empty, offsetOutside, joinType);
    }

    //=================================================================================================

    /// <summary>
    /// Builds the offset of <paramref name="face"/>, sharing the edges already generated from other faces (see the
    /// constructor). The edges of <paramref name="created"/> are written into and must be unfrozen.
    /// </summary>
    public void Init(in Face face, double offset, Dictionary<Shape, Shape> created,
                     bool offsetOutside = true, JoinKind joinType = Arc)
    {
        _shape = face;
        double myOffset = offset;
        if (face.Orientation() == global::Kerf.Topology.Orientation.Reversed)
        {
            myOffset *= -1.0;
        }

        Surface s = Tool.Surface(face, out Placement L)!;
        // On detrime les surfaces, evite des recopies dans les extensions.
        if (s is TrimmedSurface rt)
        {
            s = rt.BasisSurface();
        }
        bool isTransformed = false;
        if ((s is global::Kerf.Geometry.BSplineSurface || s is ExtrusionSurface || s is RevolutionSurface
             || s is global::Kerf.Geometry.OffsetSurface)
            && !L.IsIdentity)
        {
            s = (Surface)s.Copy();
            s.Transform(L.Transformation());
            isTransformed = true;
        }
        // particular case of cone
        if (s is ConicalSurface co)
        {
            Point3D Apex = co.Apex();
            global::Kerf.Engine.Geometry.ElSLib.Parameters(co.Cone(), Apex, out double uc, out double vc);
            BrepTools.UVBounds(face, out double uu1, out double uu2, out double vv1, out double vv2);
            if (vv2 < vc && co.SemiAngle() > 0)
            {
                myOffset *= -1;
            }
            else if (vv1 > vc && co.SemiAngle() < 0)
            {
                myOffset *= -1;
            }
            if (!co.Position().Direct())
            {
                myOffset *= -1;
            }
        }

        Surface theSurf = global::Kerf.Engine.Offsets.BRepOffset.Surface(s, myOffset, out _status);

        // processing offsets of faces with possible degenerated edges
        bool uminDegen = false;
        bool umaxDegen = false;
        bool vminDegen = false;
        bool vmaxDegen = false;
        bool uisoDegen = false;
        Point3D minApex = new(), maxApex = new();
        bool hasSingularity = false;
        double fpar, lpar;
        BrepTools.UVBounds(face, out double uf1, out double uf2, out double vf1, out double vf2);
        if ((!offsetOutside || joinType != Arc)
            && (theSurf.GetType() == typeof(ConicalSurface) || theSurf.GetType() == typeof(global::Kerf.Geometry.OffsetSurface)))
        {
            var degEdges = new NCollection_Sequence<Shape>();
            var explo = new Explorer(face, ShapeType.Edge);
            for (; explo.More(); explo.Next())
            {
                Edge edge = ShapeCasts.Edge(explo.Current());

                if (Tool.Degenerated(edge))
                {
                    Curve2D c2d = Tool.CurveOnSurface(edge, face, out double aF, out double aL)!;

                    Point2D fPnt2d = c2d.Value(aF), lPnt2d = c2d.Value(aL);

                    Point3D fPnt = s.Value(fPnt2d.X(), fPnt2d.Y()),
                           lPnt = s.Value(lPnt2d.X(), lPnt2d.Y());

                    //  aFPnt.SquareDistance(aLPnt) > Precision::SquareConfusion() -
                    // is a sufficient condition of troubles: non-singular case, but edge is degenerated.
                    // So, normal handling of degenerated edges is not applicable in case of non-singular point.
                    if (fPnt.SquareDistance(lPnt) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                    {
                        degEdges.Append(edge);
                    }
                }
            }
            if (!degEdges.IsEmpty())
            {
                const double tolApex = 1.0e-5;
                // define the iso of singularity (u or v)
                Edge theDegEdge = ShapeCasts.Edge(degEdges[1]);
                Curve2D curve = Tool.CurveOnSurface(theDegEdge, face, out fpar, out lpar)!;
                Point2D fp2d = curve.Value(fpar);
                Point2D lp2d = curve.Value(lpar);
                if (Math.Abs(fp2d.X() - lp2d.X()) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    uisoDegen = true;
                }

                if (degEdges.Length() == 2)
                {
                    if (uisoDegen)
                    {
                        uminDegen = true;
                        umaxDegen = true;
                    }
                    else
                    {
                        vminDegen = true;
                        vmaxDegen = true;
                    }
                }
                else // DegEdges.Length() == 1
                {
                    if (uisoDegen)
                    {
                        if (Math.Abs(fp2d.X() - uf1) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            uminDegen = true;
                        }
                        else
                        {
                            umaxDegen = true;
                        }
                    }
                    else
                    {
                        if (Math.Abs(fp2d.Y() - vf1) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            vminDegen = true;
                        }
                        else
                        {
                            vmaxDegen = true;
                        }
                    }
                }
                if (theSurf.GetType() == typeof(ConicalSurface))
                {
                    Cone theCone = ((ConicalSurface)theSurf).Cone();
                    Point3D apex = theCone.Apex();
                    global::Kerf.Engine.Geometry.ElSLib.Parameters(theCone, apex, out double uapex, out double vapex);
                    if (vminDegen)
                    {
                        theSurf = new TrimmedSurface(theSurf, uf1, uf2, vapex, vf2);
                        minApex = apex;
                        hasSingularity = true;
                    }
                    else if (vmaxDegen)
                    {
                        theSurf = new TrimmedSurface(theSurf, uf1, uf2, vf1, vapex);
                        maxApex = apex;
                        hasSingularity = true;
                    }
                }
                else // TheSurf->DynamicType() == STANDARD_TYPE(Geom_OffsetSurface)
                {
                    // OCCT.NET: the bounds of the extended surface are named aU1.. (C# forbids reusing u1.. below)
                    if (uminDegen)
                    {
                        Curve3D uiso = theSurf.UIso(uf1);
                        if (OffsetTool.Gabarit(uiso) > tolApex)
                        {
                            Surface basisSurf = ((global::Kerf.Geometry.OffsetSurface)theSurf).BasisSurface();
                            Point3D papex, pfirst, pquart, pmid;
                            papex = basisSurf.Value(uf1, vf1);
                            pfirst = theSurf.Value(uf1, vf1);
                            pquart = theSurf.Value(uf1, 0.75 * vf1 + 0.25 * vf2);
                            pmid = theSurf.Value(uf1, 0.5 * (vf1 + vf2));
                            Vector3D dirApex = new Vector3D(pfirst, pquart) ^ new Vector3D(pfirst, pmid);
                            var lineApex = new LineCurve3D(papex, new Direction3D(dirApex));
                            Vector3D dirGeneratrix = basisSurf.DN(uf1, vf1, 1, 0);
                            var lineGeneratrix = new LineCurve3D(pfirst, new Direction3D(dirGeneratrix));
                            var theExtrema = new ExtremaCurveCurve(lineGeneratrix, lineApex);
                            theExtrema.NearestPoints(out Point3D pint1, out Point3D pint2);
                            double length = pfirst.Distance(pint1);
                            if (offsetOutside)
                            {
                                BoundedSurface surf = new TrimmedSurface(theSurf, uf1, uf2, vf1, vf2);
                                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref surf, length, 1, true, false);
                                theSurf = surf;
                                theSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                                minApex = theSurf.Value(aU1, vf1);
                            }
                            else
                            {
                                Curve3D viso = theSurf.VIso(vf1);
                                var projector = new ProjectPointOnCurve(pint1, viso);
                                double newFirstU = projector.LowerDistanceParameter();
                                theSurf = new TrimmedSurface(theSurf, newFirstU, uf2, vf1, vf2);
                                minApex = theSurf.Value(newFirstU, vf1);
                            }
                            hasSingularity = true;
                        }
                    } // end of if (UminDegen)
                    if (umaxDegen)
                    {
                        Curve3D uiso = theSurf.UIso(uf2);
                        if (OffsetTool.Gabarit(uiso) > tolApex)
                        {
                            Surface basisSurf = ((global::Kerf.Geometry.OffsetSurface)theSurf).BasisSurface();
                            Point3D papex, pfirst, pquart, pmid;
                            papex = basisSurf.Value(uf2, vf1);
                            pfirst = theSurf.Value(uf2, vf1);
                            pquart = theSurf.Value(uf2, 0.75 * vf1 + 0.25 * vf2);
                            pmid = theSurf.Value(uf2, 0.5 * (vf1 + vf2));
                            Vector3D dirApex = new Vector3D(pfirst, pquart) ^ new Vector3D(pfirst, pmid);
                            var lineApex = new LineCurve3D(papex, new Direction3D(dirApex));
                            Vector3D dirGeneratrix = basisSurf.DN(uf2, vf1, 1, 0);
                            var lineGeneratrix = new LineCurve3D(pfirst, new Direction3D(dirGeneratrix));
                            var theExtrema = new ExtremaCurveCurve(lineGeneratrix, lineApex);
                            theExtrema.NearestPoints(out Point3D pint1, out Point3D pint2);
                            double length = pfirst.Distance(pint1);
                            if (offsetOutside)
                            {
                                BoundedSurface surf = new TrimmedSurface(theSurf, uf1, uf2, vf1, vf2);
                                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref surf, length, 1, true, true);
                                theSurf = surf;
                                theSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                                maxApex = theSurf.Value(aU2, vf1);
                            }
                            else
                            {
                                Curve3D viso = theSurf.VIso(vf1);
                                var projector = new ProjectPointOnCurve(pint1, viso);
                                double newLastU = projector.LowerDistanceParameter();
                                theSurf = new TrimmedSurface(theSurf, uf1, newLastU, vf1, vf2);
                                maxApex = theSurf.Value(newLastU, vf1);
                            }
                            hasSingularity = true;
                        }
                    } // end of if (UmaxDegen)
                    if (vminDegen)
                    {
                        Curve3D viso = theSurf.VIso(vf1);
                        if (OffsetTool.Gabarit(viso) > tolApex)
                        {
                            Surface basisSurf = ((global::Kerf.Geometry.OffsetSurface)theSurf).BasisSurface();
                            Point3D papex, pfirst, pquart, pmid;
                            papex = basisSurf.Value(uf1, vf1);
                            pfirst = theSurf.Value(uf1, vf1);
                            pquart = theSurf.Value(0.75 * uf1 + 0.25 * uf2, vf1);
                            pmid = theSurf.Value(0.5 * (uf1 + uf2), vf1);
                            Vector3D dirApex = new Vector3D(pfirst, pquart) ^ new Vector3D(pfirst, pmid);
                            var lineApex = new LineCurve3D(papex, new Direction3D(dirApex));
                            Vector3D dirGeneratrix = basisSurf.DN(uf1, vf1, 0, 1);
                            var lineGeneratrix = new LineCurve3D(pfirst, new Direction3D(dirGeneratrix));
                            var theExtrema = new ExtremaCurveCurve(lineGeneratrix, lineApex);
                            theExtrema.NearestPoints(out Point3D pint1, out Point3D pint2);
                            double length = pfirst.Distance(pint1);
                            if (offsetOutside)
                            {
                                BoundedSurface surf = new TrimmedSurface(theSurf, uf1, uf2, vf1, vf2);
                                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref surf, length, 1, false, false);
                                theSurf = surf;
                                theSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                                minApex = theSurf.Value(uf1, aV1);
                            }
                            else
                            {
                                Curve3D uiso = theSurf.UIso(uf1);
                                var projector = new ProjectPointOnCurve(pint1, uiso);
                                double newFirstV = projector.LowerDistanceParameter();
                                theSurf = new TrimmedSurface(theSurf, uf1, uf2, newFirstV, vf2);
                                minApex = theSurf.Value(uf1, newFirstV);
                                // TheSurf = new Geom_RectangularTrimmedSurface(TheSurf, uf1, uf2, vf1+length, vf2);
                                // MinApex = TheSurf->Value( uf1, vf1+length );
                            }
                            hasSingularity = true;
                        }
                    } // end of if (VminDegen)
                    if (vmaxDegen)
                    {
                        Curve3D viso = theSurf.VIso(vf2);
                        if (OffsetTool.Gabarit(viso) > tolApex)
                        {
                            Surface basisSurf = ((global::Kerf.Geometry.OffsetSurface)theSurf).BasisSurface();
                            Point3D papex, pfirst, pquart, pmid;
                            papex = basisSurf.Value(uf1, vf2);
                            pfirst = theSurf.Value(uf1, vf2);
                            pquart = theSurf.Value(0.75 * uf1 + 0.25 * uf2, vf2);
                            pmid = theSurf.Value(0.5 * (uf1 + uf2), vf2);
                            Vector3D dirApex = new Vector3D(pfirst, pquart) ^ new Vector3D(pfirst, pmid);
                            var lineApex = new LineCurve3D(papex, new Direction3D(dirApex));
                            Vector3D dirGeneratrix = basisSurf.DN(uf1, vf2, 0, 1);
                            var lineGeneratrix = new LineCurve3D(pfirst, new Direction3D(dirGeneratrix));
                            var theExtrema = new ExtremaCurveCurve(lineGeneratrix, lineApex);
                            theExtrema.NearestPoints(out Point3D pint1, out Point3D pint2);
                            double length = pfirst.Distance(pint1);
                            if (offsetOutside)
                            {
                                BoundedSurface surf = new TrimmedSurface(theSurf, uf1, uf2, vf1, vf2);
                                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref surf, length, 1, false, true);
                                theSurf = surf;
                                theSurf.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                                maxApex = theSurf.Value(uf1, aV2);
                            }
                            else
                            {
                                Curve3D uiso = theSurf.UIso(uf1);
                                var projector = new ProjectPointOnCurve(pint1, uiso);
                                double newLastV = projector.LowerDistanceParameter();
                                theSurf = new TrimmedSurface(theSurf, uf1, uf2, vf1, newLastV);
                                maxApex = theSurf.Value(uf1, newLastV);
                                // TheSurf = new Geom_RectangularTrimmedSurface(TheSurf, uf1, uf2, vf1, vf2-length);
                                // MaxApex = TheSurf->Value( uf1, vf2-length );
                            }
                            hasSingularity = true;
                        }
                    } // end of if (VmaxDegen)
                } // end of else (case of Geom_OffsetSurface)
            } // end of if (!DegEdges.IsEmpty())
        } // end of processing offsets of faces with possible degenerated edges

        // find the PCurves of the edges of <Faces>

        var myBuilder = new BrepBuilder();
        myBuilder.MakeFace(ref _face);
        if (!isTransformed)
        {
            myBuilder.UpdateFace(_face, theSurf, L, Tool.Tolerance(face));
        }
        else
        {
            myBuilder.UpdateFace(_face, theSurf, new Placement(), Tool.Tolerance(face));
        }

        var mapSS = new Dictionary<Shape, Shape>(ShapeHasher.Instance);

        // mise a jour de la map sur les vertex deja crees
        Shape localShapeOriented = face.Oriented(Forward);
        Face curFace = ShapeCasts.Face(localShapeOriented);
        //  TopoDS_Face CurFace = TopoDS::Face(Face.Oriented(TopAbs_FORWARD));

        var vonDegen = new HashSet<Shape>(ShapeHasher.Instance);
        theSurf.Bounds(out double u1, out double u2, out double v1, out double v2);

        var exp = new Explorer(curFace, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            Exploration.Vertices(e, out Vertex V1, out Vertex V2);
            if (hasSingularity && Tool.Degenerated(e))
            {
                vonDegen.Add(V1);
            }
            if (created.ContainsKey(e))
            {
                Edge oe = ShapeCasts.Edge(created.GetValue(e));
                Exploration.Vertices(oe, out Vertex ov1, out Vertex ov2);
                if (!mapSS.ContainsKey(V1))
                {
                    mapSS[V1] = ov1;
                }
                if (!mapSS.ContainsKey(V2))
                {
                    mapSS[V2] = ov2;
                }
            }
            if (created.ContainsKey(V1))
            {
                if (!mapSS.ContainsKey(V1))
                {
                    mapSS[V1] = created.GetValue(V1);
                }
            }
            if (created.ContainsKey(V2))
            {
                if (!mapSS.ContainsKey(V2))
                {
                    mapSS[V2] = created.GetValue(V2);
                }
            }
        }

        var expw = new Explorer(curFace, ShapeType.Wire);
        for (; expw.More(); expw.Next())
        {
            Wire w = ShapeCasts.Wire(expw.Current());
            var expe = new Explorer(w.Oriented(Forward), ShapeType.Edge);
            Wire ow = new();
            myBuilder.MakeWire(ref ow);
            for (; expe.More(); expe.Next())
            {
                Edge e = ShapeCasts.Edge(expe.Current());
                Exploration.Vertices(e, out Vertex V1, out Vertex V2);
                Point2D p2d1 = new(), p2d2 = new();
                Point3D p1, p2;
                double vstart, vend;
                double f, l;
                Curve2D C2d = Tool.CurveOnSurface(e, curFace, out f, out l)!;
                Edge oe = new();
                if (mapSS.ContainsKey(e) && !vonDegen.Contains(V1) && !vonDegen.Contains(V2))
                { // c`est un edge de couture
                    oe = ShapeCasts.Edge(mapSS.GetValue(e));
                    Shape localShape = e.Reversed();
                    Curve2D? C2d_1 = Tool.CurveOnSurface(ShapeCasts.Edge(localShape), curFace, out f, out l);
                    //	occ::handle<Geom2d_Curve> C2d_1 =
                    //	  BRep_Tool::CurveOnSurface(TopoDS::Edge(E.Reversed()),CurFace,f,l);
                    if (e.Orientation() == Forward)
                    {
                        UpdateEdge(oe, C2d, C2d_1, _face, Tool.Tolerance(e));
                    }
                    else
                    {
                        UpdateEdge(oe, C2d_1, C2d, _face, Tool.Tolerance(e));
                    }
                    myBuilder.Range(oe, f, l);
                }
                else
                {
                    Shape localShape = e.Oriented(Forward);
                    Edge eforward = ShapeCasts.Edge(localShape);
                    p2d1 = C2d.Value(Tool.Parameter(V1, eforward, curFace));
                    p2d2 = C2d.Value(Tool.Parameter(V2, eforward, curFace));
                    if (vonDegen.Contains(V1))
                    {
                        if (Math.Abs(p2d1.Y() - vf1) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            p1 = minApex;
                            vstart = v1;
                        }
                        else
                        {
                            p1 = maxApex;
                            vstart = v2;
                        }
                    }
                    else
                    {
                        theSurf.D0(p2d1.X(), p2d1.Y(), out p1);
                        if (!L.IsIdentity && !isTransformed)
                        {
                            p1 = p1.Transformed(L.Transformation());
                        }
                        vstart = p2d1.Y();
                    }
                    if (vonDegen.Contains(V2))
                    {
                        if (Math.Abs(p2d2.Y() - vf1) <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            p2 = minApex;
                            vend = v1;
                        }
                        else
                        {
                            p2 = maxApex;
                            vend = v2;
                        }
                    }
                    else
                    {
                        theSurf.D0(p2d2.X(), p2d2.Y(), out p2);
                        if (!L.IsIdentity && !isTransformed)
                        {
                            p2 = p2.Transformed(L.Transformation());
                        }
                        vend = p2d2.Y();
                    }
                    // E a-t-il ume image dans la Map des Created ?
                    if (created.ContainsKey(e))
                    {
                        oe = ShapeCasts.Edge(created.GetValue(e));
                    }
                    else if (mapSS.ContainsKey(e))
                    { // seam edge
                        oe = ShapeCasts.Edge(mapSS.GetValue(e));
                    }
                    else
                    {
                        myBuilder.MakeEdge(ref oe);
                        Vertex ov1 = new(), ov2 = new();
                        if (mapSS.ContainsKey(V1))
                        {
                            ov1 = ShapeCasts.Vertex(mapSS.GetValue(V1));
                        }
                        else
                        {
                            myBuilder.MakeVertex(ref ov1);
                            myBuilder.UpdateVertex(ov1, p1, Tool.Tolerance(V1));
                            mapSS[V1] = ov1;
                        }
                        if (mapSS.ContainsKey(V2))
                        {
                            ov2 = ShapeCasts.Vertex(mapSS.GetValue(V2));
                        }
                        else
                        {
                            myBuilder.MakeVertex(ref ov2);
                            myBuilder.UpdateVertex(ov2, p2, Tool.Tolerance(V2));
                            mapSS[V2] = ov2;
                        }
                        myBuilder.Add(oe, ov1.Oriented(V1.Orientation()));
                        myBuilder.Add(oe, ov2.Oriented(V2.Orientation()));
                        if (Tool.Degenerated(e))
                        {
                            myBuilder.Degenerated(oe, true);
                            // OCCT_DEBUG check of the degenerated edge's ends left out.
                        }
                    }
                    if (vonDegen.Contains(V1) || vonDegen.Contains(V2))
                    {
                        if (vonDegen.Contains(V1))
                        {
                            p2d1 = p2d1.WithY(vstart);
                        }
                        if (vonDegen.Contains(V2))
                        {
                            p2d2 = p2d2.WithY(vend);
                        }
                        C2d = new LineCurve2D(p2d1, new Direction2D(new Vector2D(p2d1, p2d2)));
                        f = 0.0;
                        l = p2d1.Distance(p2d2);
                        if (mapSS.ContainsKey(e)) // seam edge
                        {
                            Curve2D? C2d_1 = Tool.CurveOnSurface(oe, _face, out f, out l);
                            if (e.Orientation() == Forward)
                            {
                                UpdateEdge(oe, C2d, C2d_1, _face, Tool.Tolerance(e));
                            }
                            else
                            {
                                UpdateEdge(oe, C2d_1, C2d, _face, Tool.Tolerance(e));
                            }
                        }
                        else
                        {
                            UpdateEdge(oe, C2d, _face, Tool.Tolerance(e));
                        }
                        // myBuilder.Range(OE,f,l);
                        myBuilder.Range(oe, _face, f, l);
                        if (!Tool.Degenerated(e) && theSurf.IsUClosed)
                        {
                            Shape localShapeReversedE = e.Reversed();
                            Curve2D C2d_1 =
                                Tool.CurveOnSurface(ShapeCasts.Edge(localShapeReversedE), curFace, out f, out l)!;
                            p2d1 = C2d_1.Value(Tool.Parameter(V1, e, curFace));
                            p2d2 = C2d_1.Value(Tool.Parameter(V2, e, curFace));
                            if (vonDegen.Contains(V1))
                            {
                                p2d1 = p2d1.WithY(vstart);
                            }
                            if (vonDegen.Contains(V2))
                            {
                                p2d2 = p2d2.WithY(vend);
                            }
                            C2d_1 = new LineCurve2D(p2d1, new Direction2D(new Vector2D(p2d1, p2d2)));
                            if (e.Orientation() == Forward)
                            {
                                UpdateEdge(oe, C2d, C2d_1, _face, Tool.Tolerance(e));
                            }
                            else
                            {
                                UpdateEdge(oe, C2d_1, C2d, _face, Tool.Tolerance(e));
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
                        UpdateEdge(oe, C2d, _face, Tool.Tolerance(e));
                        myBuilder.Range(oe, f, l);
                        // ComputeCurve3d(OE,C2d,TheSurf,L,BRep_Tool::Tolerance(E));
                    }
                    if (!Tool.Degenerated(oe))
                    {
                        ComputeCurve3d(oe, C2d, theSurf, L, Tool.Tolerance(e));
                    }
                    mapSS[e] = oe;
                }
                myBuilder.Add(ow, oe.Oriented(e.Orientation()));
            }
            myBuilder.Add(_face, ow.Oriented(w.Orientation()));
        }

        _face.Orientation(face.Orientation());

        BrepTools.Update(_face);
    }

    //=================================================================================================

    /// <summary>
    /// Builds the pipe of radius <paramref name="offset"/> along <paramref name="path"/> between
    /// <paramref name="edge1"/> and <paramref name="edge2"/>, which get pcurves on it.
    /// </summary>
    public void Init(in Edge path, in Edge edge1, in Edge edge2, double offset,
                     bool polynomial = false, double tol = 1.0e-4, Continuity conti = C1)
    {
        Edge firstEdge = new(), lastEdge = new();
        Init(path, edge1, edge2, offset, firstEdge, lastEdge, polynomial, tol, conti);
    }

    //=================================================================================================

    /// <summary>
    /// Builds the pipe of radius <paramref name="offset"/> along <paramref name="path"/> between
    /// <paramref name="edge1"/> and <paramref name="edge2"/>, closed at its ends by <paramref name="firstEdge"/> and
    /// <paramref name="lastEdge"/> (or new edges when null). The given edges get pcurves on the pipe: in place when
    /// they are unfrozen, else on copies (see the class remarks).
    /// </summary>
    public void Init(in Edge path, in Edge edge1, in Edge edge2, double offset,
                     in Edge firstEdge, in Edge lastEdge, bool polynomial = false, double tol = 1.0e-4,
                     Continuity conti = C1)
    {
        // OCCT.NET: copy at entry (plan 2.2) of the edges written below, when one of them holds a frozen shape.
        CopyAtEntry? entry = CopyAtEntry([edge1, edge2, firstEdge, lastEdge]);
        Edge aEdge1 = Copied(entry, edge1);
        Edge aEdge2 = Copied(entry, edge2);
        Edge aFirstEdge = Copied(entry, firstEdge);
        Edge aLastEdge = Copied(entry, lastEdge);
        InitPipe(path, aEdge1, aEdge2, offset, aFirstEdge, aLastEdge, polynomial, tol, conti);
        if (entry is not null)
        {
            _face = ShapeCasts.Face(entry.Finish(_face));
        }
    }

    // The body of OCCT's Init(Path, Edge1, Edge2, Offset, FirstEdge, LastEdge, ...), on the edges to write.
    private void InitPipe(in Edge path, in Edge edge1, in Edge edge2, double offset,
                          in Edge firstEdge, in Edge lastEdge, bool polynomial, double tol,
                          Continuity conti)
    {
        bool c1Denerated = false;
        bool c2Denerated = false;
        _status = OffsetStatus.Good;
        _shape = path;

        Placement loc;
        var f = new double[3];
        var l = new double[3];

        Curve3D cp = Tool.Curve(path, out loc, out f[0], out l[0])!;
        cp = new TrimmedCurve3D(cp, f[0], l[0]);
        cp.Transform(loc.Transformation());
        var hcp = new CurveAdaptor(cp);

        Curve3D? c1 = Tool.Curve(edge1, out loc, out f[1], out l[1]);

        CurveAdaptorBase hEdge1;
        bool c1is3D = true;
        if (c1 is null)
        {
            c1is3D = false;
            Tool.CurveOnSurface(edge1, out Curve2D? c12d, out Surface? s1, out loc, out f[1], out l[1]);
            s1 = (Surface)s1!.Transformed(loc.Transformation());
            c12d = new TrimmedCurve2D(c12d!, f[1], l[1]);
            var hs1 = new SurfaceAdaptor(s1);
            var hc1 = new CurveAdaptor2D(c12d);
            // OCCT.NET: OCCT copies a local Adaptor3d_CurveOnSurface Cons(HC1, HS1)
            hEdge1 = new CurveOnSurface(hc1, hs1);
        }
        else
        {
            c1 = new TrimmedCurve3D(c1, f[1], l[1]);
            c1.Transform(loc.Transformation());
            hEdge1 = new CurveAdaptor(c1);
            var ac1 = new CurveAdaptor(c1);
            if (ac1.GetType() == Circle)
            {
                c1Denerated = (ac1.Circle().Radius() < global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }

        Curve3D? c2 = Tool.Curve(edge2, out loc, out f[2], out l[2]);

        CurveAdaptorBase hEdge2;
        bool c2is3D = true;
        if (c2 is null)
        {
            c2is3D = false;
            Tool.CurveOnSurface(edge2, out Curve2D? c12d, out Surface? s1, out loc, out f[2], out l[2]);
            s1 = (Surface)s1!.Transformed(loc.Transformation());
            c12d = new TrimmedCurve2D(c12d!, f[2], l[2]);
            var hs1 = new SurfaceAdaptor(s1);
            var hc1 = new CurveAdaptor2D(c12d);
            // OCCT.NET: OCCT copies a local Adaptor3d_CurveOnSurface Cons(HC1, HS1)
            hEdge2 = new CurveOnSurface(hc1, hs1);
        }
        else
        {
            c2 = new TrimmedCurve3D(c2, f[2], l[2]);
            c2.Transform(loc.Transformation());
            hEdge2 = new CurveAdaptor(c2);
            var ac2 = new CurveAdaptor(c2);
            if (ac2.GetType() == Circle)
            {
                c2Denerated = (ac2.Circle().Radius() < global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }

        // Calcul du tuyau
        var pipe = new SurfacePipe(hcp, hEdge1, hEdge2, Math.Abs(offset));
        pipe.Perform(tol, polynomial, conti);
        if (!pipe.IsDone)
        {
            throw new ConstructionErrorException("GeomFill_Pipe : Cannot make a surface");
        }
        double errorPipe = pipe.ErrorOnSurf();

        Surface s = pipe.InternalSurface()!;
        bool exchUV = pipe.ExchangeUV();
        s.Bounds(out double f1, out double l1, out double f2, out double l2);

        // Perform the face
        double pathTol = Tool.Tolerance(path);
        double theTol;
        var myBuilder = new BrepBuilder();
        myBuilder.MakeFace(ref _face);
        var id = new Placement();
        myBuilder.UpdateFace(_face, s, id, pathTol);

        // update de Edge1. (Rem : has already a 3d curve)
        double u, u1, u2;
        Curve2D pc;
        if (exchUV)
        {
            pc = new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.X));
            u1 = f1;
            u2 = l1;
            if (!c1is3D)
            {
                c1 = s.VIso(f2);
            }
        }
        else
        {
            pc = new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y));
            u1 = f2;
            u2 = l2;
            if (!c1is3D)
            {
                c1 = s.UIso(f1);
            }
        }

        Curve3D? dummy = null;
        if (!c1is3D)
        {
            UpdateEdge(edge1, c1, id, Tool.Tolerance(edge1));
        }
        else if (c1Denerated)
        {
            UpdateEdge(edge1, dummy, id, Tool.Tolerance(edge1));
            myBuilder.Degenerated(edge1, true);
        }

        theTol = Max(pathTol, Tool.Tolerance(edge1) + errorPipe);
        UpdateEdge(edge1, pc, _face, theTol);

        // mise a same range de la nouvelle pcurve.
        if (!c1is3D && !c1Denerated)
        {
            myBuilder.SameRange(edge1, false);
            myBuilder.Range(edge1, u1, u2, true);
        }
        myBuilder.Range(edge1, _face, u1, u2);
        BrepLibrary.SameRange(edge1);

        // mise a sameparameter pour les KPart
        if (errorPipe == 0)
        {
            theTol = Max(theTol, tol);
            myBuilder.SameParameter(edge1, false);
            BrepLibrary.SameParameter(edge1, theTol);
        }

        // Update de edge2. (Rem : has already a 3d curve)
        if (exchUV)
        {
            pc = new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.X));
            u1 = f1;
            u2 = l1;
            if (!c2is3D)
            {
                c2 = s.VIso(l2);
            }
        }
        else
        {
            pc = new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y));
            u1 = f2;
            u2 = l2;
            if (!c2is3D)
            {
                c2 = s.UIso(l1);
            }
        }

        if (!c2is3D)
        {
            UpdateEdge(edge2, c2, id, Tool.Tolerance(edge2));
        }
        else if (c2Denerated)
        {
            UpdateEdge(edge2, dummy, id, Tool.Tolerance(edge2));
            myBuilder.Degenerated(edge2, true);
        }

        theTol = Max(pathTol, Tool.Tolerance(edge2) + errorPipe);
        UpdateEdge(edge2, pc, _face, theTol);

        // mise a same range de la nouvelle pcurve.
        myBuilder.SameRange(edge2, false);
        if (!c2is3D && !c2Denerated)
        {
            myBuilder.Range(edge2, u1, u2, true);
        }
        myBuilder.Range(edge2, _face, u1, u2);
        BrepLibrary.SameRange(edge2);

        // mise a sameparameter pour les KPart
        if (errorPipe == 0)
        {
            theTol = Max(theTol, tol);
            myBuilder.SameParameter(edge2, false);
            BrepLibrary.SameParameter(edge2, theTol);
        }

        Edge edge3 = new(), edge4 = new();
        // eval edge3
        Exploration.Vertices(path, out Vertex v1f, out Vertex v1l);
        bool isClosed = (v1f.IsSame(v1l));

        Exploration.Vertices(edge1, out v1f, out v1l);
        Exploration.Vertices(edge2, out Vertex v2f, out Vertex v2l);

        bool startDegenerated = (v1f.IsSame(v2f));
        bool endDegenerated = (v1l.IsSame(v2l));

        bool e3rev = false;
        bool e4rev = false;

        Vertex vVf, vVl;
        if (firstEdge.IsNull)
        {
            myBuilder.MakeEdge(ref edge3);
            myBuilder.Add(edge3, v1f.Oriented(Forward));
            myBuilder.Add(edge3, v2f.Oriented(global::Kerf.Topology.Orientation.Reversed));
        }
        else
        {
            Shape localEdge = firstEdge.Oriented(Forward);
            edge3 = ShapeCasts.Edge(localEdge);
            //    Edge3 = TopoDS::Edge(FirstEdge.Oriented(TopAbs_FORWARD));
            Exploration.Vertices(edge3, out vVf, out vVl);
            // OCCT_DEBUG warning on unshared vertices left out.
            if (!vVf.IsSame(v1f) && !vVf.IsSame(v2f))
            {
                // On fait vraisemblablement des conneries !!
                // On cree un autre edge, on appelle le Sewing apres.
                myBuilder.MakeEdge(ref edge3);
                myBuilder.Add(edge3, v1f.Oriented(Forward));
                myBuilder.Add(edge3, v2f.Oriented(global::Kerf.Topology.Orientation.Reversed));
            }
            else if (!vVf.IsSame(v1f))
            {
                edge3.Reverse();
                e3rev = true;
            }
        }

        if (isClosed)
        {
            edge4 = edge3;
        }

        double tolApp = global::Kerf.Engine.Foundation.Precision.Approximation();

        LineCurve2D L1, L2;
        if (isClosed)
        {
            if (exchUV)
            {
                // rem : si ExchUv, il faut  reverser le Wire.
                // donc l'edge Forward dans la face sera E4 : d'ou L1 et L2
                L2 = new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y));
                L1 = new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y));
                u1 = f2;
                u2 = l2;
            }
            else
            {
                L1 = new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.X));
                L2 = new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.X));
                u1 = f1;
                u2 = l1;
            }
            if (e3rev)
            {
                L1.Reverse();
                L2.Reverse();
                u = -u1;
                u1 = -u2;
                u2 = u;
            }
            UpdateEdge(edge3, L1, L2, _face, pathTol);
            myBuilder.Range(edge3, _face, u1, u2);
            if (startDegenerated)
            {
                myBuilder.Degenerated(edge3, true);
            }
            else if (firstEdge.IsNull)
            { // then the 3d curve has not been yet computed
                ComputeCurve3d(edge3, L1, s, id, tolApp);
            }
        }
        else
        {
            if (lastEdge.IsNull)
            {
                myBuilder.MakeEdge(ref edge4);
                myBuilder.Add(edge4, v1l.Oriented(Forward));
                myBuilder.Add(edge4, v2l.Oriented(global::Kerf.Topology.Orientation.Reversed));
            }
            else
            {
                Shape localEdge = lastEdge.Oriented(Forward);
                edge4 = ShapeCasts.Edge(localEdge);
                //      Edge4 = TopoDS::Edge(LastEdge.Oriented(TopAbs_FORWARD));
                Exploration.Vertices(edge4, out vVf, out vVl);
                // OCCT_DEBUG warning on unshared vertices left out.
                if (!vVf.IsSame(v1l) && !vVf.IsSame(v2l))
                {
                    // On fait vraisemblablement des conneries !!
                    // On cree un autre edge, on appelle le Sewing apres.
                        myBuilder.MakeEdge(ref edge4);
                    myBuilder.Add(edge4, v1l.Oriented(Forward));
                    myBuilder.Add(edge4, v2l.Oriented(global::Kerf.Topology.Orientation.Reversed));
                }
                else if (!vVf.IsSame(v1l))
                {
                    edge4.Reverse();
                    e4rev = true;
                }
            }

            if (exchUV)
            {
                L1 = new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y));
                u1 = f2;
                u2 = l2;
            }
            else
            {
                L1 = new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.X));
                u1 = f1;
                u2 = l1;
            }
            if (e3rev)
            {
                L1.Reverse();
                u = -u1;
                u1 = -u2;
                u2 = u;
            }
            UpdateEdge(edge3, L1, _face, pathTol);
            myBuilder.Range(edge3, _face, u1, u2);
            if (startDegenerated)
            {
                myBuilder.Degenerated(edge3, true);
            }
            else if (firstEdge.IsNull)
            { // then the 3d curve has not been yet computed
                ComputeCurve3d(edge3, L1, s, id, tolApp);
            }

            if (exchUV)
            {
                L2 = new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y));
                u1 = f2;
                u2 = l2;
            }
            else
            {
                L2 = new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.X));
                u1 = f1;
                u2 = l1;
            }
            if (e4rev)
            {
                L2.Reverse();
                u = -u1;
                u1 = -u2;
                u2 = u;
            }
            UpdateEdge(edge4, L2, _face, pathTol);
            myBuilder.Range(edge4, _face, u1, u2);
            if (endDegenerated)
            {
                myBuilder.Degenerated(edge4, true);
            }
            else if (lastEdge.IsNull)
            { // then the 3d curve has not been yet computed
                ComputeCurve3d(edge4, L2, s, id, tolApp);
            }
        }

        // SameParameter ??
        if (!firstEdge.IsNull && !startDegenerated)
        {
            BrepLibrary.BuildCurve3d(edge3, pathTol);
            myBuilder.SameRange(edge3, false);
            myBuilder.SameParameter(edge3, false);
            BrepLibrary.SameParameter(edge3, tol);
        }
        if (!lastEdge.IsNull && !endDegenerated)
        {
            BrepLibrary.BuildCurve3d(edge4, pathTol);
            myBuilder.SameRange(edge4, false);
            myBuilder.SameParameter(edge4, false);
            BrepLibrary.SameParameter(edge4, tol);
        }

        Wire w = new();
        myBuilder.MakeWire(ref w);

        myBuilder.Add(w, edge1.Oriented(global::Kerf.Topology.Orientation.Reversed));
        myBuilder.Add(w, edge2.Oriented(Forward));
        myBuilder.Add(w, edge4.Reversed());
        myBuilder.Add(w, edge3);

        if (exchUV)
        {
            w.Reverse();
        }

        myBuilder.Add(_face, w);
        if (exchUV)
        {
            _face.Reverse();
        }

        BrepTools.Update(_face);

        if (edge1.Orientation() == global::Kerf.Topology.Orientation.Reversed)
        {
            _face.Reverse();
        }
    }

    //=================================================================================================

    /// <summary>
    /// Builds the sphere of radius <paramref name="offset"/> centred on <paramref name="vertex"/>, bounded by the
    /// edges <paramref name="lEdge"/>, which get pcurves on it: in place when they are unfrozen, else on copies (see the
    /// class remarks). <paramref name="tol"/> and <paramref name="conti"/> are used only if
    /// <paramref name="polynomial"/> (for the approximation).
    /// </summary>
    public void Init(in Vertex vertex, NCollection_List<Shape> lEdge, double offset,
                     bool polynomial = false, double tol = 1.0e-4, Continuity conti = C1)
    {
        // OCCT.NET: copy at entry (plan 2.2) of the edges written below, when one of them holds a frozen shape.
        var shapes = new List<Shape>();
        for (var it = new NCollection_List<Shape>.Iterator(lEdge); it.More(); it.Next())
        {
            shapes.Add(it.Value());
        }
        CopyAtEntry? entry = CopyAtEntry(shapes);
        NCollection_List<Shape> aLEdge = lEdge;
        if (entry is not null)
        {
            aLEdge = new NCollection_List<Shape>();
            foreach (Shape edge in shapes)
            {
                aLEdge.Append(entry.Copied(edge));
            }
        }
        InitSphere(vertex, aLEdge, offset, polynomial, tol, conti);
        if (entry is not null)
        {
            _face = ShapeCasts.Face(entry.Finish(_face));
        }
    }

    // The body of OCCT's Init(Vertex, LEdge, Offset, Polynomial, TolApp, Conti), on the edges to write.
    private void InitSphere(in Vertex vertex, NCollection_List<Shape> lEdge, double offset,
                            bool polynomial, double tolApp, Continuity conti)
    {
        _status = OffsetStatus.Good;
        _shape = vertex;

        // evaluate the Ax3 of the Sphere
        // find 3 different vertices in LEdge
        var it = new NCollection_List<Shape>.Iterator();

        // OCCT_DEBUG naming of the shapes (Affich) left out.

        Point3D origin = Tool.Pnt(vertex);

        //// Find the axis of the sphere to exclude
        //// degenerated and seam edges from the face under construction
        var mw = new LibraryMakeWire();
        mw.Add(lEdge);
        Wire theWire = mw.InternalWire();

        var fixer = new ShapeFixer(theWire);
        fixer.Perform();
        theWire = ShapeCasts.Wire(fixer.InternalShape());

        var globalProps = new Properties();
        MassPropertiesBuilder.LinearProperties(theWire, globalProps);
        Point3D baryCenter = globalProps.CentreOfMass();
        var xdir = new Vector3D(baryCenter, origin);

        Point3D farestCorner = GetFarestCorner(theWire);
        Plane3D thePlane = new MakePln(origin, baryCenter, farestCorner).Value();
        Direction3D vdir = thePlane.Axis().Direction();

        var axis = new CoordinateSystem(origin, vdir, new Direction3D(xdir));

        Surface s = new SphericalSurface(axis, Math.Abs(offset));

        double f, l, tol = Tool.Tolerance(vertex);

        Placement loc = new();
        var myBuilder = new BrepBuilder();
        myBuilder.MakeFace(ref _face);
        Surface ss = s;

        // En polynomial, calcul de la surface par F(u,v).
        // Pas de changement de parametre, donc ProjLib sur la Sphere
        // reste OK.
        if (polynomial)
        {
            var approx = new ApproxSurface(s, tolApp, conti, conti, 10, 10, 10, 1);
            if (approx.IsDone)
            {
                ss = approx.Surface()!;
            }
        }

        myBuilder.UpdateFace(_face, ss, loc, tol);

        Wire w = new();
        myBuilder.MakeWire(ref w);

        for (it.Initialize(lEdge); it.More(); it.Next())
        {
            Edge e = ShapeCasts.Edge(it.Value());

            Curve3D? c = Tool.Curve(e, out loc, out f, out l);
            if (c is null)
            {
                BrepLibrary.BuildCurve3d(e, Tool.Tolerance(e));
                c = Tool.Curve(e, out loc, out f, out l);
            }
            c = new TrimmedCurve3D(c!, f, l);
            c.Transform(loc.Transformation());

            Curve2D pCurve = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(c, s)!;
            // check if the first point of PCurve in is the canonical boundaries
            // of the sphere. Else move it.
            // the transformation is : U` = U + PI + 2 k  PI
            //                         V` = +/- PI + 2 k` PI
            Point2D p2d = pCurve.Value(f);
            bool isToAdjust = false;
            if (p2d.Y() < -M_PI / 2.0)
            {
                isToAdjust = true;
                pCurve.Mirror(new Axis2D(new Point2D(0.0, -M_PI / 2.0), Constants.DX2d()));
            }
            else if (p2d.Y() > M_PI / 2.0)
            {
                isToAdjust = true;
                pCurve.Mirror(new Axis2D(new Point2D(0.0, M_PI / 2.0), Constants.DX2d()));
            }
            if (isToAdjust)
            {
                // set the u firstpoint in [0,2*pi]
                var tr = new Vector2D(M_PI, 0.0);
                if (p2d.X() > M_PI)
                {
                    tr = tr.Reversed();
                }
                pCurve.Translate(tr);
            }

            UpdateEdge(e, pCurve, _face, tol);
            myBuilder.Range(e, _face, f, l);
            myBuilder.Add(w, e);
        }
        if (offset < 0.0)
        {
            myBuilder.Add(_face, w.Oriented(global::Kerf.Topology.Orientation.Reversed));
            _face.Reverse();
        }
        else
        {
            myBuilder.Add(_face, w);
        }

        BrepTools.Update(_face);
    }

    //=================================================================================================

    /// <summary>Builds the pipe of radius |<paramref name="offset"/>| around the free boundary
    /// <paramref name="edge"/> (used only by the rolling ball).</summary>
    public void Init(in Edge edge, double offset)
    {
        _shape = edge;
        double myOffset = Math.Abs(offset);

        Curve3D cp = Tool.Curve(edge, out Placement loc, out double f, out double l)!;
        cp = new TrimmedCurve3D(cp, f, l);
        cp.Transform(loc.Transformation());

        var pipe = new SurfacePipe(cp, myOffset);
        pipe.Perform();
        if (!pipe.IsDone)
        {
            throw new ConstructionErrorException("GeomFill_Pipe : Cannot make a surface");
        }

        var mf = new LibraryMakeFace(pipe.InternalSurface()!, global::Kerf.Engine.Foundation.Precision.Confusion());
        _face = mf.InternalFace();

        if (offset < 0.0)
        {
            _face.Reverse();
        }
    }

    //=================================================================================================

    /// <summary>The shape the offset was built from.</summary>
    public Shape InitialShape() => _shape;

    //=================================================================================================

    /// <summary>The offset face (frozen).</summary>
    public Face Face()
    {
        _face.TShape()?.FreezeGraph();
        return _face;
    }

    /// <summary><see cref="Face"/> without freezing, for ported code.</summary>
    internal Face InternalFace() => _face;

    //=================================================================================================

    /// <summary>
    /// The shape generated from a sub-shape of the initial shape (frozen): for a face, the offset of one of its edges;
    /// for a path edge, the end edge of the pipe at one of its vertices.
    /// </summary>
    public Shape Generated(in Shape shape)
    {
        Shape aShape = InternalGenerated(shape);
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary><see cref="Generated"/> without freezing, for ported code.</summary>
    internal Shape InternalGenerated(in Shape shape)
    {
        Shape aShape = new();

        switch (_shape.ShapeType())
        {
            case global::Kerf.Topology.ShapeType.Face:
            {
                var exp = new Explorer(_shape.Oriented(Forward), ShapeType.Edge);
                var expo = new Explorer(_face.Oriented(Forward), ShapeType.Edge);
                for (; exp.More() && expo.More(); exp.Next(), expo.Next())
                {
                    if (shape.IsSame(exp.Current()))
                    {
                        if (_shape.Orientation() == global::Kerf.Topology.Orientation.Reversed)
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
                Exploration.Vertices(ShapeCasts.Edge(_shape), out Vertex v1, out Vertex v2);

                var expf = new Explorer(_face.Oriented(Forward), ShapeType.Wire);
                var expo = new Explorer(expf.Current().Oriented(Forward), ShapeType.Edge);
                expo.Next();
                expo.Next();

                if (v2.IsSame(shape))
                {
                    if (expf.Current().Orientation() == global::Kerf.Topology.Orientation.Reversed)
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
                    if (expf.Current().Orientation() == global::Kerf.Topology.Orientation.Reversed)
                    {
                        aShape = expo.Current().Reversed();
                    }
                    else
                    {
                        aShape = expo.Current();
                    }
                }
                if (_face.Orientation() == global::Kerf.Topology.Orientation.Reversed)
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
    public OffsetStatus Status() => _status;

    //=================================================================================================

    // OCCT.NET: the copy at entry of the edges a pipe or a sphere writes into: null (write in place, as OCCT) unless
    // one of them holds a frozen shape; else one exact copy of them all, so that the vertices they share stay shared.
    private static CopyAtEntry? CopyAtEntry(IReadOnlyList<Shape> shapes)
    {
        bool hasFrozen = false;
        foreach (Shape shape in shapes)
        {
            if (!shape.IsNull && BrepTools.HoldsFrozen(shape))
            {
                hasFrozen = true;
                break;
            }
        }
        if (!hasFrozen)
        {
            return null;
        }

        var b = new BrepBuilder();
        Compound compound = new();
        b.MakeCompound(ref compound);
        foreach (Shape shape in shapes)
        {
            if (!shape.IsNull)
            {
                b.Add(compound, shape);
            }
        }
        return new CopyAtEntry(compound, null, true);
    }

    // The copy of an edge given to a pipe (the edge itself when nothing was copied, or when it is null).
    private static Edge Copied(CopyAtEntry? entry, in Edge edge)
    {
        if (entry is null || edge.IsNull)
        {
            return edge;
        }
        return ShapeCasts.Edge(entry.Copied(edge));
    }
}
