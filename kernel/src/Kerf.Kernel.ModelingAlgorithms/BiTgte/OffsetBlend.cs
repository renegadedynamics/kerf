// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_Blend.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_Blend.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Offsets.Blend.ContactType;
using static Kerf.Engine.Offsets.OffsetStatus;
using static Kerf.Engine.Fillets.Data.TypeOfConcavity;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Offsets.Blend;

// OCCT_DEBUG blocks (chronometers and traces) are left out; so is the file-scope t_mkcurve they fill.

/// <summary>
/// A rolling-ball blend of a shape: a ball of the given radius rolled between faces (or along edges), giving tubes
/// along the edges and spheres at the vertices where three tubes meet; optionally the shape rebuilt with them.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21), Draw's <c>rollingball</c>. It works with the offset's tools (BRepOffset_Inter3d,
/// Inter2d, MakeLoops, Offset) on its own working shapes, and OCCT writes into the shape it is given (3D curves,
/// same-parameter, parameters of the tube's vertices on the edges). So <see cref="Perform"/> runs on the caller's
/// shape in a working overlay (unit 10H5, <see cref="WorkingOverlay"/>; in place inside an operation's overlay that
/// takes writes), and the result holds the caller's faces it left unchanged (decision 1046).
/// </para>
/// <para>
/// <see cref="Shape"/>, <see cref="Face(int)"/> and <see cref="Face(in Shape)"/> freeze what they return;
/// <see cref="SupportShape1"/> and <see cref="SupportShape2"/> return the caller's shapes where they are unchanged.
/// </para>
/// </remarks>
public sealed class OffsetBlend
{
    private double _radius;
    private double _tol;
    private bool _nubs;
    private Shape _shape = new();
    private Shape _result = new();
    private bool _buildShape;
    private readonly IndexedMap<Shape, NCollection_List<Shape>> _ancestors =
        new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, Dictionary<Shape, NCollection_List<Shape>>>
        _created = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, NCollection_List<Shape>> _cutEdges =
        new(ShapeHasher.Instance);
    private IndexedSet<Shape> _faces = new(ShapeHasher.Instance);
    private IndexedSet<Shape> _edges = new(ShapeHasher.Instance);
    private HashSet<Shape> _stopFaces = new(ShapeHasher.Instance);
    private readonly Analyse _analyse = new();
    private IndexedSet<Shape> _centers = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, Offset> _mapSF = new(ShapeHasher.Instance);
    private readonly ShapeImage _initOffsetFace = new();
    private readonly ShapeImage _image = new();
    private readonly ShapeImage _imageOffset = new();
    private readonly AsDes _asDes;
    private int _nbBranches;
    private NCollection_HArray1<int>? _indices;
    private bool _done;

    // Kerf: the working overlay of the last Perform (unit 10H5); null for a run in place inside an operation's overlay.
    private WorkingOverlay? _overlay;

    //======================== START STATIC FUNCTIONS ============

    //=================================================================================================

    private static bool IsOnRestriction(in Vertex v, in Edge curE, in Face F, out Edge e)
    {
        // find if Vertex V of CurE is on a restriction of F.
        // if yes, store this restriction in E.

        // dub - 03 01 97
        // Method somewhat brutal : possible to really optimize by a
        // direct call the SD of intersections -> See LBR

        e = new Edge();
        Curve2D curC = Tool.CurveOnSurface(curE, F, out double f, out double l)!;
        double u = Tool.Parameter(v, curE, F);
        Point2D p = curC.Value(u);

        ProjectPointOnCurve2D proj = new();

        // The tolerance is exaggerated : it is better to construct too many
        // tubes than to miss intersections.
        // double Tol = 100 * BRep_Tool::Tolerance(V);
        double tol = Tool.Tolerance(v);
        Explorer exp = new(F, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            e = ShapeCasts.Edge(exp.Current());
            Curve2D pc = Tool.CurveOnSurface(e, F, out f, out l)!;
            proj.Init(p, pc, f, l);
            if (proj.NbPoints() > 0)
            {
                if (proj.LowerDistance() < tol)
                {
                    return true;
                }
            }
        }
        return false;
    }

    //=================================================================================================

    // If WarningSurBordLibre = TRUE, no propagation if the edge is open.
    private static void Add(in Edge e,
                            IndexedSet<Shape> map,
                            in Shape s,
                            Offset of,
                            Analyse analyse,
                            bool warningSurBordLibre)
    {
        ShapeType type = s.ShapeType();

        if (type == global::Kerf.Topology.ShapeType.Face)
        {
            Explorer exp = new(s, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge oriE = ShapeCasts.Edge(exp.Current());
                Shape localShape = of.InternalGenerated(oriE);
                Edge ie = ShapeCasts.Edge(localShape);
                //      const TopoDS_Edge& IE   = TopoDS::Edge(OF.Generated(OriE));
                if (e.IsEqual(ie))
                {
                    if (warningSurBordLibre)
                    {
                        // It is checked that the border is not free.
                        NCollection_List<Shape> l = analyse.Ancestors(oriE);
                        if (l.Extent() == 1)
                        {
                            break; // Nothing is done.
                        }
                    }
                    map.Add(exp.Current());
                    break;
                }
            }
        }
        else if (type == ShapeType.Edge)
        {
            Explorer exp = new(s, ShapeType.Vertex);
            for (; exp.More(); exp.Next())
            {
                Shape localShape = of.InternalGenerated(exp.Current());
                Edge ie = ShapeCasts.Edge(localShape);
                //      const TopoDS_Edge& IE = TopoDS::Edge(OF.Generated(exp.Current()));
                if (e.IsEqual(ie))
                {
                    NCollection_List<Shape> l = analyse.Ancestors(exp.Current());
                    NCollection_List<Shape>.Iterator it = new(l);
                    for (; it.More(); it.Next())
                    {
                        map.Add(it.Value());
                    }
                    break;
                }
            }
        }
    }

    //=================================================================================================

    private static bool IsInFace(in Edge e, in Face f)
    {
        Explorer exp = new(f, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            if (e.IsSame(exp.Current()))
            {
                return true;
            }
        }
        return false;
    }

    //=================================================================================================

    private static void KPartCurve3d(in Edge edge, Curve2D curve, Surface surf)
    {
        // try to find the particular case
        // if not found call BRepLib::BuildCurve3d

        Placement loc = new();
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        // Search only isos on analytical surfaces.
        CurveAdaptor2D c = new(curve);
        SurfaceAdaptor s = new(surf);
        CurveKind cTy = c.GetType();
        SurfaceKind sTy = s.GetType();
        BrepBuilder theBuilder = new();

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
                            Axis3D axeRev = new(axis.Location(), dRev);
                            ci = ci.Rotated(axeRev, p.X());
                            CircleCurve3D circle = new(ci);
                            if (d.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                circle.Reverse();
                            }
                            theBuilder.UpdateEdge(edge, circle, loc, tol);
                        }
                    }
                    else if (sTy == SurfaceKind.Cylinder)
                    {
                        Cylinder cyl = s.Cylinder();
                        Point2D p = c.Line().Location();
                        CoordinateSystem axis = cyl.Position();
                        Circle3D ci = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(axis, cyl.Radius(), p.Y());
                        Direction3D dRev = axis.XDirection().Crossed(axis.YDirection());
                        Axis3D axeRev = new(axis.Location(), dRev);
                        ci = ci.Rotated(axeRev, p.X());
                        CircleCurve3D circle = new(ci);
                        if (d.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            circle.Reverse();
                        }
                        theBuilder.UpdateEdge(edge, circle, loc, tol);
                    }
                    else if (sTy == SurfaceKind.Cone)
                    {
                        Cone cone = s.Cone();
                        Point2D p = c.Line().Location();
                        CoordinateSystem axis = cone.Position();
                        Circle3D ci = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(axis, cone.RefRadius(), cone.SemiAngle(), p.Y());
                        Direction3D dRev = axis.XDirection().Crossed(axis.YDirection());
                        Axis3D axeRev = new(axis.Location(), dRev);
                        ci = ci.Rotated(axeRev, p.X());
                        CircleCurve3D circle = new(ci);
                        if (d.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            circle.Reverse();
                        }
                        theBuilder.UpdateEdge(edge, circle, loc, tol);
                    }
                    else if (sTy == SurfaceKind.Torus)
                    {
                        Torus tore = s.Torus();
                        Point2D p = c.Line().Location();
                        CoordinateSystem axis = tore.Position();
                        Circle3D ci = global::Kerf.Engine.Geometry.ElSLib.TorusVIso(axis, tore.MajorRadius(), tore.MinorRadius(), p.Y());
                        Direction3D dRev = axis.XDirection().Crossed(axis.YDirection());
                        Axis3D axeRev = new(axis.Location(), dRev);
                        ci = ci.Rotated(axeRev, p.X());
                        CircleCurve3D circle = new(ci);
                        if (d.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            circle.Reverse();
                        }
                        theBuilder.UpdateEdge(edge, circle, loc, tol);
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

                        // set to sameparameter (rotation of the circle - offset from Y)
                        Direction3D dRev = axis.XDirection().Crossed(axis.Direction());
                        Axis3D axeRev = new(axis.Location(), dRev);
                        ci = ci.Rotated(axeRev, p.Y());

                        // transformation by iso U ( = P.X())
                        dRev = axis.XDirection().Crossed(axis.YDirection());
                        axeRev = new Axis3D(axis.Location(), dRev);
                        ci = ci.Rotated(axeRev, p.X());
                        CircleCurve3D circle = new(ci);

                        if (d.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            circle.Reverse();
                        }
                        theBuilder.UpdateEdge(edge, circle, loc, tol);
                    }
                    else if (sTy == SurfaceKind.Cylinder)
                    {
                        Cylinder cyl = s.Cylinder();
                        Point2D p = c.Line().Location();
                        Line3D l = global::Kerf.Engine.Geometry.ElSLib.CylinderUIso(cyl.Position(), cyl.Radius(), p.X());
                        Vector3D tr = new(l.Direction());
                        tr = tr.Multiplied(p.Y());
                        l = l.Translated(tr);
                        LineCurve3D line = new(l);
                        if (d.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            line.Reverse();
                        }
                        theBuilder.UpdateEdge(edge, line, loc, tol);
                    }
                    else if (sTy == SurfaceKind.Cone)
                    {
                        Cone cone = s.Cone();
                        Point2D p = c.Line().Location();
                        Line3D l = global::Kerf.Engine.Geometry.ElSLib.ConeUIso(cone.Position(), cone.RefRadius(), cone.SemiAngle(), p.X());
                        Vector3D tr = new(l.Direction());
                        tr = tr.Multiplied(p.Y());
                        l = l.Translated(tr);
                        LineCurve3D line = new(l);
                        if (d.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            line.Reverse();
                        }
                        theBuilder.UpdateEdge(edge, line, loc, tol);
                    }
                    else if (sTy == SurfaceKind.Torus)
                    {
                    }
                }
            }
        }
        else
        { // Case Plane
            Curve3D c3d = Projections.To3d(curve, s.Plane());
            theBuilder.UpdateEdge(edge, c3d, loc, tol);
        }
    }

    //=================================================================================================

    private sealed class MakeCurve_Function : Function
    {
        private readonly CurveOnEdge _curve;

        // OCCT copies the curve into the function; the port's function only reads it.
        public MakeCurve_Function(CurveOnEdge c)
        {
            _curve = c;
            _nbPnt = 1;
            _nbPnt2d = 0;
        }

        public override double FirstParameter() => _curve.FirstParameter();

        public override double LastParameter() => _curve.LastParameter();

        public override bool Value(double t, NCollection_Array1<Point2D> pnt2d, NCollection_Array1<Point3D> pnt)
        {
            pnt[1] = _curve.Value(t);
            return true;
        }

        public override bool D1(double t, NCollection_Array1<Vector2D> vec2d, NCollection_Array1<Vector3D> vec)
        {
            return false;
        }
    }

    private static Curve3D MakeCurve(CurveOnEdge hc)
    {
        Curve3D c;

        if (hc.GetType() == Circle)
        {
            c = new CircleCurve3D(hc.Circle());
            c = new TrimmedCurve3D(c, hc.FirstParameter(), hc.LastParameter());
        }
        else
        { // the approximation is done
            MakeCurve_Function f = new(hc);
            int deg1, deg2;
            deg1 = deg2 = 8;
            double tol = global::Kerf.Engine.Foundation.Precision.Approximation();
            FitAndDivide fit = new(f, deg1, deg2, tol, tol, true);
            int i;
            int nbCurves = fit.MultiCurveCount;
            // it is attempted to make the curve at least C1
            CompBezierCurvesToBSplineCurve conv = new();

            for (i = 1; i <= nbCurves; i++)
            {
                MultiCurve mc = fit.Value(i); // Load the Ith Curve
                NCollection_Array1<Point3D> poles = new(1, mc.Degree() + 1); // Return poles
                mc.Curve(1, poles);

                conv.AddCurve(poles);
            }

            conv.Perform();

            int nbPoles = conv.PoleCount;
            int nbKnots = conv.KnotCount;
            NCollection_Array1<Point3D> newPoles = new(1, nbPoles);
            NCollection_Array1<double> newKnots = new(1, nbKnots);
            NCollection_Array1<int> newMults = new(1, nbKnots);

            conv.KnotsAndMults(newKnots, newMults);
            conv.Poles(newPoles);

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(hc.FirstParameter(), hc.LastParameter(), newKnots);

            c = new BSplineCurve3D(newPoles, newKnots, newMults, conv.Degree());
        }

        return c;
    }

    //=======================================================================
    // function : Touched
    // purpose  : Only the faces connected with caps are given
    //=======================================================================

    private static void Touched(Analyse analyse,
                                HashSet<Shape> stopFaces,
                                in Shape shape,
                                HashSet<Shape> touchedByCork)
    {
        // currently nothing is done !!
    }

    //=================================================================================================

    private static Vertex FindVertex(in Point3D p, HashSet<Shape> map, double tol)
    {
        BrepBuilder b = new();
        // Find in <Map> a vertex which represent the point <P>.
        double tol2, dist;
        Vertex v = new();
        Vertex[] vv = new Vertex[2];
        double tolCarre = tol * tol;
        // OCCT iterates the map by the edges' addresses; the port in insertion order.
        foreach (Shape key in map)
        {
            Edge e = ShapeCasts.Edge(key);
            if (!e.IsNull)
            {
                Exploration.Vertices(e, out vv[0], out vv[1]);

                for (int i = 0; i < 2; i++)
                {
                    // if OK la Tolerance du Vertex
                    tol2 = Tool.Tolerance(vv[i]);
                    tol2 *= tol2;
                    Point3D p1 = Tool.Pnt(vv[i]);
                    dist = p.SquareDistance(p1);
                    if (dist <= tol2)
                    {
                        return vv[i];
                    }
                    // otherwise with the required tolerance.
                    if (tolCarre > tol2)
                    {
                        if (dist <= tolCarre)
                        {
                            // so it is necessary to update the tolerance of Vertex.
                            b.UpdateVertex(vv[i], tol);
                            return vv[i];
                        }
                    }
                }
            }
        }

        return v;
    }

    //=================================================================================================

    private static Edge MakeDegeneratedEdge(Curve3D cc, in Vertex vfOnE)
    {
        BrepBuilder b = new();
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        // kill trimmed curves
        Curve3D c = cc;
        TrimmedCurve3D? ct = c as TrimmedCurve3D;
        while (ct is not null)
        {
            c = ct.BasisCurve();
            ct = c as TrimmedCurve3D;
        }

        Vertex v1 = new(), v2;
        if (vfOnE.IsNull)
        {
            Point3D p = c.Value(c.FirstParameter());
            b.MakeVertex(ref v1, p, tol);
            v2 = v1;
        }
        else
        {
            v1 = v2 = vfOnE;
        }
        v1.Orientation(Forward);
        v2.Orientation(global::Kerf.Topology.Orientation.Reversed);

        Edge e = new();
        b.MakeEdge(ref e, c, tol);
        b.Add(e, v1);
        b.Add(e, v2);
        //  B.UpdateVertex(V1,C->FirstParameter(),E,Tol);
        //  B.UpdateVertex(V2,C->LastParameter(),E,Tol);
        b.Range(e, cc.FirstParameter(), cc.LastParameter());
        return e;
    }

    //=================================================================================================

    private static Orientation Orientation(in Edge e, in Face f, NCollection_List<Shape> l)
    {
        Orientation orien = Forward;
        NCollection_List<Shape>.Iterator itld = new();
        for (itld.Initialize(l); itld.More(); itld.Next())
        {
            if (itld.Value().IsSame(e))
            {
                orien = itld.Value().Orientation();
                break;
            }
        }
        if (f.Orientation() == global::Kerf.Topology.Orientation.Reversed)
        {
            orien = global::Kerf.Engine.Topology.TopAbs.Reverse(orien);
        }

        return orien;
    }

    //=================================================================================================

    private static Edge FindCreatedEdge(in Vertex v1,
                                               in Edge e,
                                               Dictionary<Shape, Offset> mapSF,
                                               HashSet<Shape> mapOnV,
                                               Analyse centerAnalyse,
                                               double radius,
                                               double tol)
    {
        Edge e1 = new();
        if (!centerAnalyse.HasAncestor(v1))
        {
            return e1; // return a Null Shape.
        }

        NCollection_List<Shape> tangE = new();
        centerAnalyse.TangentEdges(e, v1, tangE);

        NCollection_List<Shape>.Iterator itl = new(tangE);
        bool find = false;
        for (; itl.More() && !find; itl.Next())
        {
            Edge et = ShapeCasts.Edge(itl.Value());
            if (mapSF.ContainsKey(et))
            {
                Shape localShape = mapSF.GetValue(et).InternalGenerated(v1);
                e1 = ShapeCasts.Edge(localShape);
                //      E1 = TopoDS::Edge(MapSF(ET).Generated(V1));
                mapOnV.Add(e1);
                find = true;
            }
            else
            {
                // Find the sharing of vertices in case of tangent consecutive 3 edges
                // the second of which is the edge that degenerates the tube.
                Curve3D cet = Tool.Curve(et, out Placement cLoc, out double ff, out double ll)!;
                if (cet.GetType() == typeof(TrimmedCurve3D))
                {
                    cet = ((TrimmedCurve3D)cet).BasisCurve();
                }
                if (cet is not CircleCurve3D circ)
                {
                    continue;
                }
                if (Math.Abs(circ.Radius() - Math.Abs(radius)) > tol)
                {
                    continue;
                }

                Exploration.Vertices(et, out Vertex u1, out Vertex u2);
                if (u1.IsSame(v1))
                {
                    u1 = u2;
                }
                NCollection_List<Shape> tang2 = new();
                centerAnalyse.TangentEdges(et, u1, tang2);
                NCollection_List<Shape>.Iterator it2 = new(tang2);
                for (; it2.More(); it2.Next())
                {
                    Edge et2 = ShapeCasts.Edge(it2.Value());
                    if (mapSF.ContainsKey(et2))
                    {
                        Shape localShape = mapSF.GetValue(et2).InternalGenerated(u1);
                        mapOnV.Add(ShapeCasts.Edge(localShape));
                        //	  MapOnV.Add(TopoDS::Edge(MapSF(ET2).Generated(U1)));
                    }
                }
            }
        }
        if (!find)
        {
            tangE.Clear();
            //	CenterAnalyse.Edges(V1f, OT, TangE);
            if (centerAnalyse.HasAncestor(v1))
            {
                // OCCT assigns (copies) the ancestors' list.
                tangE = new NCollection_List<Shape>(centerAnalyse.Ancestors(v1));
                itl.Initialize(tangE);
                for (; itl.More() && !find; itl.Next())
                {
                    if (mapSF.ContainsKey(itl.Value()))
                    {
                        mapOnV.Add(mapSF.GetValue(itl.Value()).InternalGenerated(v1));
                    }
                }
            }
        }

        return e1;
    }

    //======================== END OF STATIC FUNCTIONS ============

    //=================================================================================================

    /// <summary>An empty blend; <see cref="Init"/> gives the shape.</summary>
    public OffsetBlend()
    {
        _asDes = new AsDes();
        _nbBranches = -1;
    }

    //=================================================================================================

    /// <summary>
    /// The blend of <paramref name="s"/> with the radius <paramref name="radius"/>; <paramref name="tol"/> is the 3D
    /// tolerance of the approximations, <paramref name="nubs"/> true makes only B-spline surfaces.
    /// </summary>
    public OffsetBlend(in Shape s, double radius, double tol, bool nubs)
    {
        _asDes = new AsDes();
        Init(s, radius, tol, nubs);
    }

    //=================================================================================================

    /// <summary>Sets the shape, radius, tolerance and surface kind, and clears the rest.</summary>
    public void Init(in Shape s, double radius, double tol, bool nubs)
    {
        Clear();
        _shape = s;
        _tol = tol;
        _nubs = nubs;
        _radius = radius;
        _nbBranches = -1;
        //  TopExp::MapShapesAndAncestors(S,TopAbs_EDGE,TopAbs_FACE,myAncestors);
    }

    //=================================================================================================

    /// <summary>Clears all the fields.</summary>
    public void Clear()
    {
        _initOffsetFace.Clear();
        _image.Clear();
        _imageOffset.Clear();
        _stopFaces.Clear();
        _analyse.Clear();
        _asDes.Clear();
        _nbBranches = -1;
        _done = false;
        _overlay = null;
    }

    //=================================================================================================

    /// <summary>Sets a face of the shape on which the fillet must stop.</summary>
    public void SetStoppingFace(in Face face)
    {
        _stopFaces.Add(face);
        //-------------
        // MAJ SD. -> To end loop, set faces of edges
        //-------------
        //  myInitOffsetFace.SetRoot(Face);
        //  myInitOffsetFace.Bind   (Face,Face);
        //  myImageOffset.SetRoot   (Face);
    }

    //=================================================================================================

    /// <summary>Sets two faces of the shape between which the ball must roll.</summary>
    public void SetFaces(in Face f1, in Face f2)
    {
        _faces.Add(f1);
        _faces.Add(f2);
    }

    //=================================================================================================

    /// <summary>Sets an edge of the shape to be rounded.</summary>
    public void SetEdge(in Edge edge)
    {
        _edges.Add(edge);
    }

    //=================================================================================================

    /// <summary>
    /// Computes the blending surfaces, and the resulting shape if <paramref name="buildShape"/>.
    /// </summary>
    /// <exception cref="FailureException">The sewing of the shape failed.</exception>
    public void Perform(bool buildShape = true)
    {
        _buildShape = buildShape;

        // Kerf: OCCT writes into the shape it is given: the work is done on the caller's shape in a working overlay
        // (unit 10H5).
        using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(_shape);
        _overlay = overlay;

        // Try cutting to avoid tubes on free borders
        // that are not actually free.
        Sewing sew = new(_tol);
        BrepLibrary.BuildCurves3d(_shape);
        Explorer expf = new(_shape, global::Kerf.Topology.ShapeType.Face);
        for (; expf.More(); expf.Next())
        {
            sew.Add(expf.Current());
        }
        sew.Perform();
        Shape sewedShape = sew.InternalSewedShape();
        if (sewedShape.IsNull)
        {
            throw new FailureException("Sewing aux fraises");
        }

        // Check if the sewing modified the orientation.
        expf.Init(_shape, global::Kerf.Topology.ShapeType.Face);
        Face faceRef = ShapeCasts.Face(expf.Current());
        Orientation oriRef = faceRef.Orientation();
        if (sew.IsModified(faceRef))
        {
            faceRef = ShapeCasts.Face(sew.InternalModified(faceRef));
        }
        expf.Init(sewedShape, global::Kerf.Topology.ShapeType.Face);
        for (; expf.More(); expf.Next())
        {
            Face ff = ShapeCasts.Face(expf.Current());
            if (faceRef.IsSame(ff) && (ff.Orientation() != oriRef))
            {
                sewedShape.Reverse();
                break;
            }
        }

        // Make SameParameter if Sew does not do it (Detect that edges
        // are not sameparameter but it does nothing.)
        expf.Init(sewedShape, ShapeType.Edge);
        for (; expf.More(); expf.Next())
        {
            Edge sec = ShapeCasts.Edge(expf.Current());
            BrepLibrary.SameParameter(sec, Tool.Tolerance(sec));
        }

        Exploration.MapShapesAndAncestors(sewedShape, ShapeType.Edge, global::Kerf.Topology.ShapeType.Face, _ancestors);

        // Extend myFaces with the faces of the sewed shape.
        expf.Init(_shape, global::Kerf.Topology.ShapeType.Face);
        for (; expf.More(); expf.Next())
        {
            Shape f = expf.Current();
            if (_faces.Contains(f) && sew.IsModified(f))
            {
                _faces.Remove(f);
                _faces.Add(sew.InternalModified(f));
            }
        }

        _shape = sewedShape;
        // end Sewing for false free borders.

        // ----------------------------------------------------------------
        // place faces with the proper orientation in the initial shape
        // ----------------------------------------------------------------
        Explorer exp = new(_shape, global::Kerf.Topology.ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            Shape f = exp.Current();
            if (_faces.Contains(f))
            {
                _faces.Remove(f);
                _faces.Add(f);
            }
            else if (_stopFaces.Contains(f))
            {
                _stopFaces.Remove(f);
                _stopFaces.Add(f);
            }
        }

        // ----------------------------------------------
        // Calculate lines of centers and of surfaces
        // ----------------------------------------------
        ComputeCenters();

        // -----------------------------
        // Calculate connection Surfaces
        // -----------------------------
        ComputeSurfaces();

        // ----------------------------------
        // Calculate the generated shape if required
        // ----------------------------------
        if (_buildShape)
        {
            ComputeShape();
        }

        // Finally construct curves 3d from edges to be transferred
        // since the partition is provided ( A Priori);
        BrepLibrary.BuildCurves3d(_result, global::Kerf.Engine.Foundation.Precision.Confusion());

        // Kerf: new TShapes for the caller's sub-shapes the blend changed; its own for the rest.
        if (overlay is not null)
        {
            _result = overlay.Finish(_result);
        }

        _done = true;
    }

    //=================================================================================================

    /// <summary>True if <see cref="Perform"/> was run.</summary>
    public bool IsDone => _done;

    //=================================================================================================

    /// <summary>The result (frozen).</summary>
    public Shape Shape()
    {
        _result.TShape()?.FreezeGraph();
        return _result;
    }

    /// <summary><see cref="Shape"/> without freezing, for ported code.</summary>
    internal Shape InternalShape() => _result;

    //=================================================================================================

    /// <summary>The number of blending surfaces.</summary>
    public int SurfaceCount => _centers.Count;

    //=================================================================================================

    /// <summary>The blending surface of index <paramref name="index"/>.</summary>
    public Surface? Surface(int index) => Surface(_centers[index - 1]);

    //=================================================================================================

    /// <summary>The blending face of index <paramref name="index"/> (frozen).</summary>
    public Face Face(int index) => Face(_centers[index - 1]);

    //=================================================================================================

    /// <summary>Sets in <paramref name="lc"/> all the center lines.</summary>
    public void CenterLines(NCollection_List<Shape> lc)
    {
        lc.Clear();
        int nb = SurfaceCount;
        for (int i = 1; i <= nb; i++)
        {
            lc.Append(_centers[i - 1]);
        }
    }

    //=================================================================================================

    /// <summary>
    /// The surface generated by the center line <paramref name="centerLine"/>: a pipe for an edge, a sphere for a
    /// vertex.
    /// </summary>
    public Surface? Surface(in Shape centerLine)
    {
        Face f = _mapSF.GetValue(centerLine).InternalFace();
        Surface? s = Tool.Surface(f);
        s?.Freeze();
        return s;
    }

    //=================================================================================================

    /// <summary>The face generated by the center line <paramref name="centerLine"/> (frozen).</summary>
    /// <exception cref="DomainErrorException">The center line generated no face.</exception>
    public Face Face(in Shape centerLine)
    {
        if (!_mapSF.ContainsKey(centerLine))
        {
            throw new DomainErrorException("BiTgte_Blend::Face");
        }

        Face face = _mapSF.GetValue(centerLine).InternalFace();
        face.TShape()?.FreezeGraph();
        return face;
    }

    //=================================================================================================

    /// <summary>The type of contact of the blending surface of index <paramref name="index"/>.</summary>
    public ContactType ContactType(int index)
    {
        Shape s1 = SupportShape1(index);
        Shape s2 = SupportShape2(index);

        ShapeType type1 = s1.ShapeType();
        ShapeType type2 = s2.ShapeType();

        if (type2 < type1)
        {
            ShapeType dummy = type1;
            type1 = type2;
            type2 = dummy;
        }
        ContactType type = global::Kerf.Engine.Offsets.Blend.ContactType.VertexVertex;

        switch (type1)
        {
            case ShapeType.Vertex:
                switch (type2)
                {
                    case ShapeType.Vertex:
                        type = global::Kerf.Engine.Offsets.Blend.ContactType.VertexVertex;
                        break;
                    case ShapeType.Edge:
                        type = global::Kerf.Engine.Offsets.Blend.ContactType.EdgeVertex;
                        break;
                    case global::Kerf.Topology.ShapeType.Face:
                        type = global::Kerf.Engine.Offsets.Blend.ContactType.FaceVertex;
                        break;
                    default:
                        break;
                }
                break;

            case ShapeType.Edge:
                switch (type2)
                {
                    case ShapeType.Edge:
                        type = global::Kerf.Engine.Offsets.Blend.ContactType.EdgeEdge;
                        break;
                    case global::Kerf.Topology.ShapeType.Face:
                        type = global::Kerf.Engine.Offsets.Blend.ContactType.FaceEdge;
                        break;
                    default:
                        break;
                }
                break;

            case global::Kerf.Topology.ShapeType.Face:
                switch (type2)
                {
                    case global::Kerf.Topology.ShapeType.Face:
                        type = global::Kerf.Engine.Offsets.Blend.ContactType.FaceEdge; // OCCT: FaceFace is never returned
                        break;
                    default:
                        break;
                }
                break;

            default:
                break;
        }

        return type;
    }

    //=================================================================================================

    /// <summary>The first support shape of the blending surface of index <paramref name="index"/>.</summary>
    public Shape SupportShape1(int index)
    {
        Edge curE = ShapeCasts.Edge(_centers[index - 1]);

        NCollection_List<Shape> l = _asDes.Ascendant(curE);

        // --------------------------------------------------------------
        // F1 and F2 = 2 parallel faces intersecting at CurE.
        // --------------------------------------------------------------
        Face f1 = ShapeCasts.Face(l.First());
        Shape or1 = _initOffsetFace.ImageFrom(f1);
        return Published(or1);
    }

    //=================================================================================================

    /// <summary>The second support shape of the blending surface of index <paramref name="index"/>.</summary>
    public Shape SupportShape2(int index)
    {
        Edge curE = ShapeCasts.Edge(_centers[index - 1]);

        NCollection_List<Shape> l = _asDes.Ascendant(curE);

        // --------------------------------------------------------------
        // F1 and F2 = 2 parallel faces intersecting at CurE.
        // --------------------------------------------------------------
        Face f2 = ShapeCasts.Face(l.Last());
        Shape or2 = _initOffsetFace.ImageFrom(f2);
        return Published(or2);
    }

    // Kerf: a shape of the run as it stands in the result (the caller's own where unchanged), frozen.
    private Shape Published(in Shape shape)
    {
        Shape aShape = _overlay is not null && _overlay.IsFinished ? _overlay.Final(shape) : shape;
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    //=================================================================================================

    /// <summary>The 3D curve of the blending surface of index <paramref name="index"/> on its first support.</summary>
    public Curve3D? CurveOnShape1(int index)
    {
        Edge curE = ShapeCasts.Edge(_centers[index - 1]);
        Shape F = _mapSF.GetValue(curE).InternalFace();

        // somewhat brutal method based ONLY on the construction of the fillet:
        // the first edge of the tube is exactly the edge on Shape1.

        Explorer exp = new(F, ShapeType.Edge);
        Edge e = ShapeCasts.Edge(exp.Current());
        Curve3D? c = null;
        if (!Tool.Degenerated(e))
        {
            c = Tool.Curve(e, out double f, out double l);
            c = new TrimmedCurve3D(c!, f, l);
        }
        return c;
    }

    //=================================================================================================

    /// <summary>The 3D curve of the blending surface of index <paramref name="index"/> on its second support.</summary>
    public Curve3D? CurveOnShape2(int index)
    {
        Edge curE = ShapeCasts.Edge(_centers[index - 1]);
        Shape F = _mapSF.GetValue(curE).InternalFace();

        // somewhat brutal method based ONLY on the construction of the fillet:
        // the first edge of the tube is exactly the edge on Shape2.

        Explorer exp = new(F, ShapeType.Edge);
        exp.Next();
        Edge e = ShapeCasts.Edge(exp.Current());
        Curve3D? c = null;
        if (!Tool.Degenerated(e))
        {
            c = Tool.Curve(e, out double f, out double l);
            c = new TrimmedCurve3D(c!, f, l);
        }
        return c;
    }

    //=================================================================================================

    /// <summary>Always null in OCCT.</summary>
    public Curve2D? PCurveOnFace1(int index) => null;

    /// <summary>Always null in OCCT.</summary>
    public Curve2D? PCurve1OnFillet(int index) => null;

    /// <summary>Always null in OCCT.</summary>
    public Curve2D? PCurveOnFace2(int index) => null;

    /// <summary>Always null in OCCT.</summary>
    public Curve2D? PCurve2OnFillet(int index) => null;

    //=================================================================================================

    /// <summary>The number of branches (sets of connected blending faces); reorders the center lines by branch.</summary>
    public int NbBranches()
    {
        if (_nbBranches != -1)
        {
            return _nbBranches;
        }

        // else, compute the Branches.
        Quilt glue = new();

        int nbFaces = _centers.Count;

        if (nbFaces == 0)
        {
            return 0;
        }

        int i;
        for (i = 1; i <= nbFaces; i++)
        {
            Shape centerLine = _centers[i - 1];
            glue.Add(_mapSF.GetValue(centerLine).InternalFace());
        }

        Shape shells = glue.Shells();

        // Reorder Map myCenters.
        // The method is brutal and unpolished,
        // it is possible to refine it.
        _nbBranches = 0;
        IndexedSet<Shape> tmpMap = new(ShapeHasher.Instance);

        Explorer exp = new(shells, ShapeType.Shell);
        for (; exp.More(); exp.Next())
        {
            _nbBranches++;
        }

        _indices = new NCollection_HArray1<int>(1, _nbBranches + 1);

        _indices.SetValue(1, 0);
        int count = 0;
        int index = 2;

        exp.Init(shells, ShapeType.Shell);
        for (; exp.More(); exp.Next())
        {
            // CurS = the current Shell.
            Shape curS = exp.Current();

            Explorer exp2 = new(curS, global::Kerf.Topology.ShapeType.Face);
            for (; exp2.More(); exp2.Next())
            {
                // CurF = the current face of the current Shell.
                Shape curF = exp2.Current();

                for (i = 1; i <= nbFaces; i++)
                {
                    Shape center = _centers[i - 1];
                    Shape rakk = _mapSF.GetValue(center).InternalFace();
                    // Rakk = the ith generated connection face
                    if (curF.IsEqual(rakk))
                    {
                        tmpMap.Add(center);
                        count++;
                        break;
                    }
                }
            }
            _indices.SetValue(index, count);
            index++;
        }

        _centers = tmpMap;
        return _nbBranches;
    }

    //=================================================================================================

    /// <summary>
    /// The indices of the faces of the branch <paramref name="index"/>: Face(From) to Face(To) (after
    /// <see cref="NbBranches"/>).
    /// </summary>
    public void IndicesOfBranche(int index, out int From, out int to)
    {
        // Attention to the ranking in myIndices:
        // If the branches are  1-4 5-9 10-12, it is ranked in myIndices:
        //                      0 4   9    12
        From = _indices!.Value(index) + 1;
        to = _indices.Value(index + 1);
    }

    //=================================================================================================

    // OCCT binds copies of an empty map and an empty list into myCreated; the port binds new ones.
    private NCollection_List<Shape> Created(in Shape key1, in Shape key2) =>
        _created.GetValue(key1).GetValue(key2);

    private void BindCreated(in Shape key) =>
        _created[key] = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);

    private void BindCreated(in Shape key1, in Shape key2) =>
        _created.GetValue(key1)[key2] = new NCollection_List<Shape>();

    //=================================================================================================

    /// <summary>Computes the center lines.</summary>
    public void ComputeCenters()
    {
        // ------------
        // Preanalyze.
        // ------------
        double tolAngle = 2 * Math.Asin(_tol / Math.Abs(_radius * 0.5));
        _analyse.Perform(_shape, tolAngle);

        // ------------------------------------------
        // calculate faces touched by caps
        // ------------------------------------------
        HashSet<Shape> touchedByCork = new(ShapeHasher.Instance);
        Touched(_analyse, _stopFaces, _shape, touchedByCork);

        // -----------------------
        // init of the intersector
        // -----------------------
        Containment side = In;
        if (_radius < 0.0)
        {
            side = Out;
        }
        Inter3d inter = new(_asDes, side, _tol);

        Dictionary<Shape, Box> mapSBox = new(ShapeHasher.Instance);
        HashSet<Shape> done = new(ShapeHasher.Instance);

        BrepBuilder b = new();
        Compound co = new(); // to only know on which edges the tubes are made
        b.MakeCompound(ref co);

        // ----------------------------------------
        // Calculate Sections Face/Face + Propagation
        // ----------------------------------------
        bool jenRajoute = true;
        int i;

        while (jenRajoute)
        {
            jenRajoute = false;

            bool fini = false;

            Dictionary<Shape, Shape> edgeTgt = new(ShapeHasher.Instance);

            while (!fini)
            {
                // -------------------------------------------------
                // locate in myFaces the Faces connected to myEdges.
                // -------------------------------------------------
                fini = true;
                // for (it.Initialize(myEdges); it.More(); it.Next()) {
                for (i = 1; i <= _edges.Count; i++)
                {
                    Edge e = ShapeCasts.Edge(_edges[i - 1]);
                    if (Tool.Degenerated(e))
                    {
                        continue;
                    }

                    NCollection_List<Shape> l = _ancestors.GetValue(e);
                    if (l.Extent() == 1)
                    {
                        // So this is a free border onwhich the ball should roll.
                        _faces.Add(e);

                        // set in myStopFaces to not propagate the tube on free border.
                        _stopFaces.Add(e);
                    }
                    else
                    {
                        NCollection_List<Shape>.Iterator itl = new();
                        for (itl.Initialize(l); itl.More(); itl.Next())
                        {
                            Shape sh = itl.Value();
                            if (!_stopFaces.Contains(sh))
                            {
                                _faces.Add(itl.Value());
                            }
                        }
                    }
                }
                _edges.Clear();

                // --------------------------------------------
                // Construction of Offsets of all faces.
                // --------------------------------------------
                // for (it.Initialize(myFaces); it.More(); it.Next()) {
                for (i = 1; i <= _faces.Count; i++)
                {
                    Shape AS = _faces[i - 1];
                    if (_mapSF.ContainsKey(AS))
                    {
                        continue;
                    }

                    Offset of1 = new();
                    Face bigF = new();

                    if (AS.ShapeType() == global::Kerf.Topology.ShapeType.Face)
                    {
                        Face f = ShapeCasts.Face(_faces[i - 1]);
                        if (touchedByCork.Contains(f))
                        {
                            OffsetTool.EnLargeFace(f, ref bigF, true);
                            of1.Init(bigF, _radius, edgeTgt);
                        }
                        else
                        {
                            of1.Init(f, _radius, edgeTgt);
                        }
                    }
                    else
                    { // So this is a Free Border edge on which the ball rolls.
                        of1.Init(ShapeCasts.Edge(AS), _radius);
                    }

                    // ------------------------------------
                    // Increment the map of created tangents
                    // ------------------------------------
                    NCollection_List<Shape> Let = new();
                    if (AS.ShapeType() == global::Kerf.Topology.ShapeType.Face)
                    {
                        _analyse.Edges(ShapeCasts.Face(AS), TypeOfConcavity.Tangential, Let);
                    }
                    NCollection_List<Shape>.Iterator itlet = new(Let);

                    for (; itlet.More(); itlet.Next())
                    {
                        Edge cur = ShapeCasts.Edge(itlet.Value());
                        if (!edgeTgt.ContainsKey(cur))
                        {
                            Shape localShape = of1.InternalGenerated(cur);
                            Edge ote = ShapeCasts.Edge(localShape);
                            //	    const TopoDS_Edge& OTE = TopoDS::Edge(OF1.Generated(Cur));
                            edgeTgt[cur] = of1.InternalGenerated(cur);
                            Exploration.Vertices(cur, out Vertex v1, out Vertex v2);
                            Exploration.Vertices(ote, out Vertex ov1, out Vertex ov2);
                            NCollection_List<Shape> le = new();
                            if (!edgeTgt.ContainsKey(v1))
                            {
                                _analyse.Edges(v1, TypeOfConcavity.Tangential, le);
                                NCollection_List<Shape> la = _analyse.Ancestors(v1);
                                if (le.Extent() == la.Extent())
                                {
                                    edgeTgt[v1] = ov1;
                                }
                            }
                            if (!edgeTgt.ContainsKey(v2))
                            {
                                le.Clear();
                                _analyse.Edges(v2, TypeOfConcavity.Tangential, le);
                                NCollection_List<Shape> la = _analyse.Ancestors(v2);
                                if (le.Extent() == la.Extent())
                                {
                                    edgeTgt[v2] = ov2;
                                }
                            }
                        }
                    }
                    // end of map created tangent

                    if (of1.Status() == OffsetStatus.Reversed || of1.Status() == OffsetStatus.Degenerated)
                    {
                        continue;
                    }

                    Face f1 = of1.InternalFace();

                    // increment S D
                    _initOffsetFace.SetRoot(AS);
                    _initOffsetFace.Bind(AS, f1);

                    Box box1 = new();
                    Bounds.Add(f1, ref box1);
                    mapSBox[f1] = box1;

                    // ---------------------------------------------
                    // intersection with all already created faces.
                    // ---------------------------------------------
                    fini = !Intersect(AS, f1, mapSBox, of1, inter);

                    if (AS.ShapeType() == global::Kerf.Topology.ShapeType.Face)
                    {
                        b.Add(co, AS);
                    }

                    _mapSF[AS] = of1;
                }
            } // end of : while ( !Fini)

            //--------------------------------------------------------
            // so the offsets were created and intersected.
            // now the tubes are constructed.
            //--------------------------------------------------------
            // Construction of tubes on edge.
            //--------------------------------------------------------
            TypeOfConcavity ot1 = TypeOfConcavity.Convex;
            if (_radius < 0.0)
            {
                ot1 = TypeOfConcavity.Concave;
            }

            IndexedMap<Shape, NCollection_List<Shape>> map =
                new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(co, ShapeType.Edge, global::Kerf.Topology.ShapeType.Face, map);
            Exploration.MapShapesAndAncestors(co, ShapeType.Vertex, ShapeType.Edge, map);

            Explorer exp = new(co, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                if (_mapSF.ContainsKey(e))
                {
                    continue;
                }

                NCollection_List<Shape> anc = map.GetValue(e);
                if (anc.Extent() == 2)
                {
                    NCollection_List<Interval> l = _analyse.Type(e);
                    if (!l.IsEmpty() && l.First().Type() == ot1)
                    {
                        Shape localShapeGen = _mapSF.GetValue(anc.First()).InternalGenerated(e);
                        Edge eOn1 = ShapeCasts.Edge(localShapeGen);
                        localShapeGen = _mapSF.GetValue(anc.Last()).InternalGenerated(e);
                        Edge eOn2 = ShapeCasts.Edge(localShapeGen);
                        //	  TopoDS_Edge EOn1 = TopoDS::Edge(myMapSF(Anc.First()).Generated(E));
                        //	  TopoDS_Edge EOn2 = TopoDS::Edge(myMapSF(Anc.Last()) .Generated(E));
                        // find if exits tangent edges in the original shape
                        Edge e1f = new(), e1l = new();
                        Exploration.Vertices(e, out Vertex v1f, out Vertex v1l);
                        NCollection_List<Shape> tangE = new();
                        _analyse.TangentEdges(e, v1f, tangE);
                        // find if the pipe on the tangent edges are soon created.
                        NCollection_List<Shape>.Iterator itl = new(tangE);
                        bool find = false;
                        for (; itl.More() && !find; itl.Next())
                        {
                            if (_mapSF.ContainsKey(itl.Value()))
                            {
                                Shape localShape = _mapSF.GetValue(itl.Value()).InternalGenerated(v1f);
                                e1f = ShapeCasts.Edge(localShape);
                                //	      E1f  = TopoDS::Edge(myMapSF(itl.Value()).Generated(V1f));
                                find = true;
                            }
                        }
                        tangE.Clear();
                        _analyse.TangentEdges(e, v1l, tangE);
                        // find if the pipe on the tangent edges are soon created.
                        itl.Initialize(tangE);
                        find = false;
                        for (; itl.More() && !find; itl.Next())
                        {
                            if (_mapSF.ContainsKey(itl.Value()))
                            {
                                Shape localShape = _mapSF.GetValue(itl.Value()).InternalGenerated(v1l);
                                e1l = ShapeCasts.Edge(localShape);
                                //	      E1l  = TopoDS::Edge(myMapSF(itl.Value()).Generated(V1l));
                                find = true;
                            }
                        }
                        Offset of1 = new(e, eOn1, eOn2, _radius, e1f, e1l);
                        Face f1 = of1.InternalFace();

                        // maj S D
                        _initOffsetFace.SetRoot(e);
                        _initOffsetFace.Bind(e, f1);

                        Box box1 = new();
                        Bounds.Add(f1, ref box1);
                        mapSBox[f1] = box1;

                        // ---------------------------------------------
                        // intersection with all already created faces.
                        // ---------------------------------------------
                        bool isOnRest = Intersect(e, f1, mapSBox, of1, inter);
                        jenRajoute = jenRajoute || isOnRest;

                        _mapSF[e] = of1;
                    }
                }
            }
        } // end while JenRajoute

        _edges.Clear();
        _edges = new IndexedSet<Shape>(inter.NewEdges());

        // -------------------------------------------------------------------
        // now it is necessary to limit edges on the neighbors (otherwise one
        // will go too far and will not be able to construct faces).
        // -------------------------------------------------------------------

        // Proceed with MakeLoops
        IndexedMap<Shape, NCollection_List<Shape>> dmvv =
            new(ShapeHasher.Instance);
        TypeOfConcavity ot = TypeOfConcavity.Concave;
        if (_radius < 0.0)
        {
            ot = TypeOfConcavity.Convex;
        }

        NCollection_List<Shape> lof = new();
        // it.Initialize(myFaces);
        for (i = 1; i <= _faces.Count; i++)
        {
            Shape curS = _faces[i - 1];

            // tube on free border, it is undesirable.
            if (_stopFaces.Contains(curS))
            {
                continue;
            }

            if (!_mapSF.ContainsKey(curS))
            {
                continue; // inverted or degenerated
            }

            Face curOF = _mapSF.GetValue(curS).InternalFace();
            lof.Append(curOF);

            if (curS.ShapeType() == global::Kerf.Topology.ShapeType.Face)
            {
                Face curF = ShapeCasts.Face(curS);
                Explorer expe = new(curF.Oriented(Forward), ShapeType.Edge);
                for (; expe.More(); expe.Next())
                {
                    // --------------------------------------------------------------
                    // set in myAsDes the edges generated by limitations of the
                    // initial square if the type is correct (The edges that will
                    // disappear are not set)
                    // --------------------------------------------------------------
                    Edge curE = ShapeCasts.Edge(expe.Current());
                    NCollection_List<Interval> l = _analyse.Type(curE);
                    if (!l.IsEmpty() && l.First().Type() != ot)
                    {
                        // a priori doe s not disappear, so it is set
                        Shape localShape = _mapSF.GetValue(curF).InternalGenerated(curE);
                        Edge curOE = ShapeCasts.Edge(localShape);
                        //	  const TopoDS_Edge& CurOE =
                        //	    TopoDS::Edge(myMapSF(CurF).Generated(CurE));
                        _asDes.Add(curOF, curOE.Oriented(curE.Orientation()));
                    }
                    else
                    {
                        NCollection_List<Shape> lanc = _analyse.Ancestors(curE);
                        if (!_faces.Contains(lanc.First()) || !_faces.Contains(lanc.Last())
                            || _stopFaces.Contains(lanc.First()) || _stopFaces.Contains(lanc.Last()))
                        {
                            Shape localShape = _mapSF.GetValue(curF).InternalGenerated(curE);
                            Edge curOE = ShapeCasts.Edge(localShape);
                            //	    const TopoDS_Edge& CurOE =
                            //	      TopoDS::Edge(myMapSF(CurF).Generated(CurE));
                            _asDes.Add(curOF, curOE.Oriented(curE.Orientation()));
                        }
                    }
                }
                Dictionary<Shape, NCollection_List<Shape>> emptyMap =
                    new(ShapeHasher.Instance);
                Inter2d.Compute(_asDes, curOF, _edges, _tol, emptyMap, dmvv, new ProgressRange());
            }
        }

        // ----------------------------------------------------------------
        // It is also required to make 2D intersections with generated tubes
        // (Useful for unwinding)
        // ----------------------------------------------------------------
        // OCCT iterates myMapSF by the shapes' addresses; the port in insertion order.
        foreach (KeyValuePair<Shape, Offset> entry in _mapSF)
        {
            Shape curS = entry.Key;
            if (curS.ShapeType() == global::Kerf.Topology.ShapeType.Face)
            {
                continue;
            }

            Face curOF = entry.Value.InternalFace();

            // no unwinding by tubes on free border.
            if (_stopFaces.Contains(curS))
            {
                continue;
            }

            lof.Append(curOF);

            // --------------------------------------------------------------
            // set in myAsDes the edge restrictions of the square
            // --------------------------------------------------------------
            Explorer expe = new(curOF.Oriented(Forward), ShapeType.Edge);
            for (; expe.More(); expe.Next())
            {
                Edge curOE = ShapeCasts.Edge(expe.Current());
                _asDes.Add(curOF, curOE);
            }

            Dictionary<Shape, NCollection_List<Shape>> emptyMap =
                new(ShapeHasher.Instance);
            Inter2d.Compute(_asDes, curOF, _edges, _tol, emptyMap, dmvv, new ProgressRange());
        }
        //
        // fuse vertices on edges stored in AsDes
        ShapeImage emptyImage = new();
        Inter2d.FuseVertices(dmvv, _asDes, emptyImage);
        // ------------
        // unwinding
        // ------------
        OffsetMakeLoops makeLoops = new();
        makeLoops.Build(lof, _asDes, _imageOffset, emptyImage, new ProgressRange());

        // ------------------------------------------------------------
        // It is possible to unwind edges at least one ancestor which of
        // is a face of the initial shape, so:
        // the edges generated by intersection tube-tube are missing
        // ------------------------------------------------------------

        // --------------------------------------------------------------
        // Currently set the unwinded surfaces in <myResult>
        // --------------------------------------------------------------
        Compound result = new();
        b.MakeCompound(ref result);
        _result = result;
        NCollection_List<Shape>.Iterator itLOF = new(lof);
        for (; itLOF.More(); itLOF.Next())
        {
            Shape curLOF = itLOF.Value();

            if (!_imageOffset.HasImage(curLOF))
            {
                continue;
            }

            NCollection_List<Shape> lim = new();
            _imageOffset.LastImage(curLOF, lim);
            NCollection_List<Shape>.Iterator itLim = new(lim);
            for (; itLim.More(); itLim.Next())
            {
                // If a face is its own image, it is not set
                Shape curLIM = itLim.Value();
                if (curLIM.IsSame(curLOF))
                {
                    break;
                }

                b.Add(_result, curLIM);
            }
        }
    }

    //=================================================================================================

    private void ComputeSurfaces()
    {
        // set in myFaces, the faces actually implied in the connection
        _faces.Clear();

        // construct
        // 1 - Tubes (True Fillets)
        // 2 - Spheres.

        Surface? gs1 = null, gs2 = null;
        Curve3D? gc1 = null, gc2 = null;

        double tolAngle = 2 * Math.Asin(_tol / Math.Abs(_radius * 0.5));
        Analyse centerAnalyse = new(_result, tolAngle);

        // -----------------------------------------------------
        // Construction of tubes in myResult
        // -----------------------------------------------------
        BrepBuilder b = new();
        Compound result = new();
        b.MakeCompound(ref result);
        _result = result;

        // --------------------------------------------------------------------
        // Dummy: for construction of spheres:
        // Set in Co the center line, then it there are at least 3
        // center lines sharing the same vertex, Sphere on this vertex.
        // --------------------------------------------------------------------
        Compound co = new();
        b.MakeCompound(ref co);

        // --------------------------------------------------------------------
        // Iteration on the edges lines of center
        // and their valid valid part is taken after cut and tube construction.
        // --------------------------------------------------------------------

        // NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher>::Iterator ic(myEdges);
        int i;
        for (i = 1; i <= _edges.Count; i++)
        {
            Edge curE = ShapeCasts.Edge(_edges[i - 1]);

            NCollection_List<Shape> l = _asDes.Ascendant(curE);
            if (l.Extent() != 2)
            {
                continue;
            }

            // --------------------------------------------------------------
            // F1 and F2 = 2 parallel faces intersecting in CurE.
            // --------------------------------------------------------------
            Face F1 = ShapeCasts.Face(l.First());
            Face F2 = ShapeCasts.Face(l.Last());

            // -----------------------------------------------------
            // find the orientation of edges of intersection
            // in the initial faces.
            // -----------------------------------------------------
            NCollection_List<Shape> ld1 = _asDes.Descendant(F1);
            NCollection_List<Shape> ld2 = _asDes.Descendant(F2);

            Orientation orien1 = Orientation(curE, F1, ld1);
            Orientation orien2 = Orientation(curE, F2, ld2);

            // ---------------------------------------------------------
            // Or1 and Or2 : the shapes generators of parallel faces
            // ---------------------------------------------------------
            Shape or1 = _initOffsetFace.ImageFrom(F1);
            Shape or2 = _initOffsetFace.ImageFrom(F2);

            _faces.Add(or1);
            _faces.Add(or2);

            Edge oe1 = new(), oe2 = new();
            Face of1 = new(), of2 = new();
            Placement loc;
            double f1, l1, f2, l2;

            bool of1isEdge = false;

            if (or1.ShapeType() == ShapeType.Edge)
            {
                of1isEdge = true;
                oe1 = ShapeCasts.Edge(or1);
                gc1 = Tool.Curve(oe1, out loc, out f1, out l1);
                gc1 = (Curve3D)gc1!.Transformed(loc.Transformation());
            }
            else if (or1.ShapeType() == global::Kerf.Topology.ShapeType.Face)
            {
                of1 = ShapeCasts.Face(or1);
                gs1 = Tool.Surface(of1);
            }

            // ----------------------------------------------------------------
            // If a vertex is used in contact, currently nothing is done
            // and the vertexes are not managed (Intersections with sphere);
            // ----------------------------------------------------------------
            if (of1.IsNull && oe1.IsNull)
            {
                continue;
            }

            bool of2isEdge = false;

            if (or2.ShapeType() == ShapeType.Edge)
            {
                of2isEdge = true;
                oe2 = ShapeCasts.Edge(or2);
                gc2 = Tool.Curve(oe2, out loc, out f2, out l2);
                gc2 = (Curve3D)gc2!.Transformed(loc.Transformation());
            }
            else if (or2.ShapeType() == global::Kerf.Topology.ShapeType.Face)
            {
                of2 = ShapeCasts.Face(or2);
                gs2 = Tool.Surface(of2);
            }
            // ----------------------------------------------------------------
            // If a vertex is used in contact, currently nothing is done
            // and the vertexes are not managed (Intersections with sphere);
            // ----------------------------------------------------------------
            if (of2.IsNull && oe2.IsNull)
            {
                continue;
            }

            NCollection_List<Shape> curL = new();

            if (!_imageOffset.HasImage(curE))
            { // the tubes are not unwinded
                if (of1isEdge && of2isEdge)
                { // if I don't have the image, possibly
                    curL.Append(curE); // I'm on intersection tube-tube
                } // See comment on the call to
                else
                { // MakeLoops
                    continue;
                }
            }
            else
            {
                _imageOffset.LastImage(curE, curL);
            }

            // ---------------------------------------------------------------
            // CurL = List of edges descending from CurE ( = Cuts of CurE)
            // ---------------------------------------------------------------
            NCollection_List<Shape>.Iterator itl = new(curL);
            for (; itl.More(); itl.Next())
            {
                Edge curCutE = ShapeCasts.Edge(itl.Value());

                Curve2D? pc1 = Tool.CurveOnSurface(curCutE, F1, out f1, out l1);
                Curve2D? pc2 = Tool.CurveOnSurface(curCutE, F2, out f2, out l2);
                if (pc1 is null || pc2 is null)
                {
                    continue;
                }

                Edge e1f, e1l;
                Vertex vfOnE1, vlOnE1, vfOnE2, vlOnE2;
                HashSet<Shape> mapOnV1f = new(ShapeHasher.Instance);
                HashSet<Shape> mapOnV1l = new(ShapeHasher.Instance);

                Exploration.Vertices(curCutE, out Vertex v1f, out Vertex v1l);

                // find if the pipe on the tangent edges are soon created.
                // edges generated by V1f and V1l + Maj MapOnV1f/l
                e1f = FindCreatedEdge(v1f, curCutE, _mapSF, mapOnV1f, centerAnalyse, _radius, _tol);

                e1l = FindCreatedEdge(v1l, curCutE, _mapSF, mapOnV1l, centerAnalyse, _radius, _tol);

                Edge e1, e2;
                if (of1isEdge)
                {
                    CurveOnEdge conE = new(curCutE, oe1);
                    Curve3D c = MakeCurve(conE);
                    Point3D p1 = c.Value(c.FirstParameter());
                    Point3D p2 = c.Value(c.LastParameter());
                    vfOnE1 = FindVertex(p1, mapOnV1f, _tol);
                    if (vfOnE1.IsNull)
                    {
                        vfOnE1 = FindVertex(p1, mapOnV1l, _tol);
                    }
                    vlOnE1 = FindVertex(p2, mapOnV1l, _tol);
                    if (vlOnE1.IsNull)
                    {
                        vlOnE1 = FindVertex(p2, mapOnV1f, _tol);
                    }
                    if (p1.SquareDistance(p2) < _tol * _tol)
                    {
                        // BRepOffset_Offset manages degenerated KPart
                        // It is REQUIRED that C should be a circle with ZERO radius
                        e1 = MakeDegeneratedEdge(c, vfOnE1);
                    }
                    else
                    {
                        e1 = new LibraryMakeEdge(c, vfOnE1, vlOnE1).InternalEdge();
                    }
                }
                else
                {
                    Point2D p2d;
                    p2d = pc1.Value(f1);
                    Point3D p1 = gs1!.Value(p2d.X(), p2d.Y());
                    p2d = pc1.Value(l1);
                    Point3D p2 = gs1.Value(p2d.X(), p2d.Y());
                    vfOnE1 = FindVertex(p1, mapOnV1f, _tol);
                    vlOnE1 = FindVertex(p2, mapOnV1l, _tol);
                    LibraryMakeEdge mke = new(pc1, gs1, vfOnE1, vlOnE1, f1, l1);
                    if (mke.IsDone)
                    {
                        e1 = mke.InternalEdge();
                    }
                    else
                    {
                        // OCCT writes "Edge Not Done" to std::cout here.
                        e1 = mke.InternalEdge();
                    }

                    KPartCurve3d(e1, pc1, gs1);
                }

                if (of2isEdge)
                {
                    CurveOnEdge conE = new(curCutE, oe2);
                    Curve3D c = MakeCurve(conE);
                    Point3D p1 = c.Value(c.FirstParameter());
                    Point3D p2 = c.Value(c.LastParameter());
                    vfOnE2 = FindVertex(p1, mapOnV1f, _tol);
                    if (vfOnE2.IsNull)
                    {
                        vfOnE2 = FindVertex(p1, mapOnV1l, _tol);
                    }
                    vlOnE2 = FindVertex(p2, mapOnV1l, _tol);
                    if (vlOnE2.IsNull)
                    {
                        vlOnE2 = FindVertex(p2, mapOnV1f, _tol);
                    }
                    if (p1.SquareDistance(p2) < _tol * _tol)
                    {
                        // BRepOffset_Offset manages degenerated KParts
                        // It is REQUIRED that C should be a circle with ZERO radius
                        e2 = MakeDegeneratedEdge(c, vfOnE2);
                    }
                    else
                    {
                        e2 = new LibraryMakeEdge(c, vfOnE2, vlOnE2).InternalEdge();
                    }
                }
                else
                {
                    Point2D p2d;
                    p2d = pc2.Value(f2);
                    Point3D p1 = gs2!.Value(p2d.X(), p2d.Y());
                    p2d = pc2.Value(l2);
                    Point3D p2 = gs2.Value(p2d.X(), p2d.Y());
                    vfOnE2 = FindVertex(p1, mapOnV1f, _tol);
                    vlOnE2 = FindVertex(p2, mapOnV1l, _tol);
                    LibraryMakeEdge mke = new(pc2, gs2, vfOnE2, vlOnE2, f2, l2);
                    if (mke.IsDone)
                    {
                        e2 = mke.InternalEdge();
                    }
                    else
                    {
                        // OCCT writes "edge not Done" to std::cout here.
                        e2 = mke.InternalEdge();
                    }
                    KPartCurve3d(e2, pc2, gs2);
                }
                // Increment of the Map of Created if reconstruction of the Shape is required
                if (_buildShape)
                {
                    BindCreated(curCutE);

                    BindCreated(curCutE, or1);
                    Created(curCutE, or1).Append(e1);

                    BindCreated(curCutE, or2);
                    Created(curCutE, or2).Append(e2);
                }

                // ----------------------------------------------------------
                // try to init E1f, E1l, if not found with Analysis.
                // Should happen only if the THEORETICALLY tangent edges
                // are not actually tangent ( Cf: Approximation of lines
                // of intersection that add noise.)
                // ----------------------------------------------------------
                Vertex vertex1, vertex2;
                if (e1f.IsNull && !vfOnE1.IsNull && !vfOnE2.IsNull)
                {
                    // OCCT iterates the map by the edges' addresses; the port in insertion order.
                    foreach (Shape key in mapOnV1f)
                    {
                        Edge e = ShapeCasts.Edge(key);
                        if (!e.IsNull)
                        {
                            Exploration.Vertices(e, out vertex1, out vertex2);
                            if ((vertex1.IsSame(vfOnE1) && vertex2.IsSame(vfOnE2))
                                || (vertex2.IsSame(vfOnE1) && vertex1.IsSame(vfOnE2)))
                            {
                                e1f = e;
                                break;
                            }
                        }
                    }
                }
                if (e1l.IsNull && !vlOnE1.IsNull && !vlOnE2.IsNull)
                {
                    foreach (Shape key in mapOnV1l)
                    {
                        Edge e = ShapeCasts.Edge(key);
                        if (!e.IsNull)
                        {
                            Exploration.Vertices(e, out vertex1, out vertex2);
                            if ((vertex1.IsSame(vlOnE1) && vertex2.IsSame(vlOnE2))
                                || (vertex2.IsSame(vlOnE1) && vertex1.IsSame(vlOnE2)))
                            {
                                e1l = e;
                                break;
                            }
                        }
                    }
                }

                e1.Orientation(orien1);
                e2.Orientation(orien2);

                Offset anOffset = new(curCutE, e1, e2, -_radius, e1f, e1l, _nubs, _tol, C2);
                _mapSF[curCutE] = anOffset;
                _centers.Add(curCutE);
                b.Add(co, curCutE);

                Face tuyo = anOffset.InternalFace();
                b.Add(_result, tuyo);

                if (_buildShape)
                {
                    // method based ONLY on the construction of fillet:
                    // the first edge of the tube is exactly on Shape1.
                    ProjectPointOnCurve projector = new();
                    Explorer exp = new(tuyo, ShapeType.Edge);
                    Vertex v1, v2;
                    if (of1isEdge)
                    { // Update CutEdges.
                        Edge eOnF1 = ShapeCasts.Edge(exp.Current());
                        Exploration.Vertices(eOnF1, out v1, out v2);

                        Point3D p1 = Tool.Pnt(v1);
                        projector.Init(p1, gc1!);
                        double u1 = projector.LowerDistanceParameter();

                        Point3D p2 = Tool.Pnt(v2);
                        projector.Init(p2, gc1!);
                        double u2 = projector.LowerDistanceParameter();

                        Shape localShape = v1.Oriented(Internal);
                        b.UpdateVertex(ShapeCasts.Vertex(localShape), u1, ShapeCasts.Edge(or1), _tol);
                        localShape = v2.Oriented(Internal);
                        b.UpdateVertex(ShapeCasts.Vertex(localShape), u2, ShapeCasts.Edge(or1), _tol);
                        //	  B.UpdateVertex(TopoDS::Vertex(V1.Oriented(TopAbs_INTERNAL)),U1,
                        //			 TopoDS::Edge(Or1),myTol);
                        //	  B.UpdateVertex(TopoDS::Vertex(V2.Oriented(TopAbs_INTERNAL)),U2,
                        //			 TopoDS::Edge(Or1),myTol);

                        if (!_cutEdges.ContainsKey(or1))
                        {
                            _cutEdges[or1] = new NCollection_List<Shape>();
                        }
                        NCollection_List<Shape> L1 = _cutEdges.GetValue(or1);
                        L1.Append(v1);
                        L1.Append(v2);
                    }
                    if (of2isEdge)
                    { // Update CutEdges.
                        exp.Next();
                        Edge eOnF2 = ShapeCasts.Edge(exp.Current());
                        Exploration.Vertices(eOnF2, out v1, out v2);

                        Point3D p1 = Tool.Pnt(v1);
                        projector.Init(p1, gc2!);
                        double u1 = projector.LowerDistanceParameter();

                        Point3D p2 = Tool.Pnt(v2);
                        projector.Init(p2, gc2!);
                        double u2 = projector.LowerDistanceParameter();

                        Shape localShape = v1.Oriented(Internal);
                        b.UpdateVertex(ShapeCasts.Vertex(localShape), u1, ShapeCasts.Edge(or2), _tol);
                        localShape = v2.Oriented(Internal);
                        b.UpdateVertex(ShapeCasts.Vertex(localShape), u2, ShapeCasts.Edge(or2), _tol);
                        //	  B.UpdateVertex(TopoDS::Vertex(V1.Oriented(TopAbs_INTERNAL)),U1,
                        //			 TopoDS::Edge(Or2),myTol);
                        //	  B.UpdateVertex(TopoDS::Vertex(V2.Oriented(TopAbs_INTERNAL)),U2,
                        //			 TopoDS::Edge(Or2),myTol);

                        if (!_cutEdges.ContainsKey(or2))
                        {
                            _cutEdges[or2] = new NCollection_List<Shape>();
                        }
                        NCollection_List<Shape> L2 = _cutEdges.GetValue(or2);
                        L2.Append(v1);
                        L2.Append(v2);
                    }
                }
            }
        }

        // ---------------------------------------------------
        // Construction of spheres,
        // if enough tubes arrive at the vertex
        // ---------------------------------------------------
        IndexedMap<Shape, NCollection_List<Shape>> map = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(co, ShapeType.Vertex, ShapeType.Edge, map);

        for (int j = 0; j < map.Count; j++)
        {
            Vertex v = ShapeCasts.Vertex(map.KeyAt(j));
            if (map.ValueAt(j).Extent() != 3)
            {
                continue;
            }

            NCollection_List<Shape> loe = new();
            NCollection_List<Shape>.Iterator it = new();

            for (it.Initialize(map.ValueAt(j)); it.More(); it.Next())
            {
                bool reverse = true;
                if (reverse)
                {
                    loe.Append(_mapSF.GetValue(it.Value()).InternalGenerated(v).Reversed());
                }
                else
                {
                    loe.Append(_mapSF.GetValue(it.Value()).InternalGenerated(v));
                }
            }

            Offset oft = new(v, loe, -_radius, _nubs, _tol, C2);
            _mapSF[v] = oft;
            _centers.Add(v);

            b.Add(_result, oft.InternalFace());
        }
    }

    //=================================================================================================

    private void ComputeShape()
    {
        // Find in the initial Shapel:
        //  - untouched Faces
        //  - generated tubes
        //  - the faces neighbors of tubes that should be reconstructed preserving sharing.

        // For Debug : Visualize edges of the initial shape that should be reconstructed.
        // end debug

        BrepBuilder b = new();

        // Maj of the Map of created.
        // Update edges that do not change in the resulting shape
        // i.e. invariant edges in the unwinding.
        Explorer exp = new(_shape, global::Kerf.Topology.ShapeType.Face);
        // int nbe = 1;
        for (; exp.More(); exp.Next())
        {
            Face curF = ShapeCasts.Face(exp.Current());

            if (!_faces.Contains(curF))
            {
                continue; // so the face is not touched
            }

            // so the faces are unwinded
            if (!_mapSF.ContainsKey(curF))
            {
                continue; // inverted or degenerated
            }

            Offset offset = _mapSF.GetValue(curF);
            Face curOF = _mapSF.GetValue(curF).InternalFace();

            if (!_imageOffset.HasImage(curOF))
            { // face disappears in unwinding
                continue;
            }

            Explorer exp2 = new(curF, ShapeType.Edge);
            for (; exp2.More(); exp2.Next())
            {
                Edge curE = ShapeCasts.Edge(exp2.Current());
                Shape localShape = offset.InternalGenerated(curE);
                Edge curOE = ShapeCasts.Edge(localShape);
                //      const TopoDS_Edge& CurOE = TopoDS::Edge(Offset.Generated(CurE));

                if (!_imageOffset.HasImage(curOE))
                {
                    continue;
                }
                // CurOE disappears

                Edge imE = ShapeCasts.Edge(_imageOffset.Image(curOE).First());
                if (imE.IsSame(curOE))
                {
                    BindCreated(curOE);
                    BindCreated(curOE, curF);
                    Created(curOE, curF).Append(curE);
                }
            }
        }

        // The connected faces are already in myResult.
        // So it is necessary to add faces:
        //    - non-touched (so not in myFaces)
        //    - issuing from the unwinding (non degenerated, non inverted, non disappeared)
        exp.Init(_shape, global::Kerf.Topology.ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            Face curF = ShapeCasts.Face(exp.Current());

            if (!_faces.Contains(curF))
            {
                // so the face is not touched
                b.Add(_result, curF);
            }
            else
            { // so the faces are unwindeds

                if (!_mapSF.ContainsKey(curF))
                {
                    continue; // inverted or degenerated
                }

                Face curOF = _mapSF.GetValue(curF).InternalFace();

                if (!_imageOffset.HasImage(curOF))
                { // face disappears in unwinding
                    continue;
                }

                // List of faces generated by a face in the unwinding
                NCollection_List<Shape> lim = new();
                _imageOffset.LastImage(curOF, lim);
                NCollection_List<Shape>.Iterator itLim = new(lim);
                for (; itLim.More(); itLim.Next())
                {
                    // DeboucFace = offset Face unwinded in "Debouc".
                    Face deboucFace = ShapeCasts.Face(itLim.Value());

                    Surface s = Tool.Surface(curF, out Placement L)!;

                    Face newF = new();
                    b.MakeFace(ref newF);
                    b.UpdateFace(newF, s, L, Tool.Tolerance(curF));

                    Dictionary<Shape, Shape> mapSS = new(ShapeHasher.Instance);

                    Shape localShape = deboucFace.Oriented(Forward);
                    Face face = ShapeCasts.Face(localShape);
                    //	const TopoDS_Face& Face =
                    //	  TopoDS::Face(DeboucFace.Oriented(TopAbs_FORWARD));
                    Explorer exp2 = new(face, ShapeType.Edge);
                    for (; exp2.More(); exp2.Next())
                    {
                        Edge e = ShapeCasts.Edge(exp2.Current());
                        Exploration.Vertices(e, out Vertex v1, out Vertex v2);
                        if (_created.ContainsKey(e))
                        {
                            if (_created.GetValue(e).ContainsKey(curF))
                            {
                                Edge oe = ShapeCasts.Edge(Created(e, curF).First());
                                Exploration.Vertices(oe, out Vertex ov1, out Vertex ov2);
                                if (!_created.ContainsKey(v1))
                                {
                                    BindCreated(v1);
                                }
                                if (!_created.ContainsKey(v2))
                                {
                                    BindCreated(v2);
                                }
                                if (!_created.GetValue(v1).ContainsKey(curF))
                                {
                                    BindCreated(v1, curF);
                                    Created(v1, curF).Append(ov1);
                                }
                                if (!_created.GetValue(v2).ContainsKey(curF))
                                {
                                    BindCreated(v2, curF);
                                    Created(v2, curF).Append(ov2);
                                }
                            }
                        }
                    }

                    Explorer expw = new(face, ShapeType.Wire);
                    for (; expw.More(); expw.Next())
                    {
                        Wire w = ShapeCasts.Wire(expw.Current());
                        Explorer expe = new(w.Oriented(Forward), ShapeType.Edge);
                        Wire ow = new();
                        b.MakeWire(ref ow);

                        for (; expe.More(); expe.Next())
                        {
                            Edge e = ShapeCasts.Edge(expe.Current());
                            Curve2D? c2d = Tool.CurveOnSurface(e, face, out double f, out double l);
                            Edge oe = new();
                            if (mapSS.ContainsKey(e))
                            { // this is an edge of cutting
                                oe = ShapeCasts.Edge(mapSS.GetValue(e));
                                Shape localShapeReversedE = e.Reversed();
                                Curve2D? C2d_1 =
                                    Tool.CurveOnSurface(ShapeCasts.Edge(localShapeReversedE), face, out f, out l);
                                //	      occ::handle<Geom2d_Curve> C2d_1 =
                                //		BRep_Tool::CurveOnSurface(TopoDS::Edge(E.Reversed()),
                                //					  Face,f,l);
                                if (e.Orientation() == Forward)
                                {
                                    b.UpdateEdge(oe, c2d, C2d_1, newF, Tool.Tolerance(e));
                                }
                                else
                                {
                                    b.UpdateEdge(oe, C2d_1, c2d, newF, Tool.Tolerance(e));
                                }
                                b.Range(oe, f, l);
                            }
                            else
                            {
                                // Is there an image in the Map of Created ?
                                if (_created.ContainsKey(e))
                                {
                                    if (_created.GetValue(e).ContainsKey(curF))
                                    {
                                        oe = ShapeCasts.Edge(Created(e, curF).First());
                                    }
                                }
                                else
                                {
                                    b.MakeEdge(ref oe);
                                    Vertex ov1 = new(), ov2 = new();
                                    Exploration.Vertices(e, out Vertex v1, out Vertex v2);
                                    if (_created.ContainsKey(v1) && _created.GetValue(v1).ContainsKey(curF))
                                    {
                                        ov1 = ShapeCasts.Vertex(Created(v1, curF).First());
                                    }
                                    else
                                    {
                                        b.MakeVertex(ref ov1);
                                        Point2D p2d = c2d!.Value(Tool.Parameter(v1, e, face));
                                        s.D0(p2d.X(), p2d.Y(), out Point3D p);
                                        p = p.Transformed(L.Transformation());
                                        b.UpdateVertex(ov1, p, Tool.Tolerance(v1));
                                        BindCreated(v1);
                                        BindCreated(v1, curF);
                                        Created(v1, curF).Append(ov1);
                                    }
                                    if (_created.ContainsKey(v2) && _created.GetValue(v2).ContainsKey(curF))
                                    {
                                        ov2 = ShapeCasts.Vertex(Created(v2, curF).First());
                                    }
                                    else
                                    {
                                        b.MakeVertex(ref ov2);
                                        Point2D p2d = c2d!.Value(Tool.Parameter(v2, e, face));
                                        s.D0(p2d.X(), p2d.Y(), out Point3D p);
                                        p = p.Transformed(L.Transformation());
                                        b.UpdateVertex(ov2, p, Tool.Tolerance(v2));
                                        BindCreated(v2);
                                        BindCreated(v2, curF);
                                        Created(v2, curF).Append(ov2);
                                    }
                                    b.Add(oe, ov1.Oriented(v1.Orientation()));
                                    b.Add(oe, ov2.Oriented(v2.Orientation()));
                                }
                                b.UpdateEdge(oe, c2d, newF, Tool.Tolerance(e));
                                b.Range(oe, f, l);
                                //	      ComputeCurve3d(OE,C2d,TheSurf,L,BRep_Tool::Tolerance(E));
                                mapSS[e] = oe;
                            }
                            b.Add(ow, oe.Oriented(e.Orientation()));
                        }
                        b.Add(newF, ow.Oriented(w.Orientation()));
                    }

                    newF.Orientation(deboucFace.Orientation());

                    BrepTools.Update(newF);
                    b.Add(_result, newF);
                }
            }
        }

        // non-regarding the cause, there always remain greeb borders on this Shape, so it is sewn.
        Sewing sew = new(_tol);

        BrepLibrary.BuildCurves3d(_result);

        exp.Init(_result, global::Kerf.Topology.ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            sew.Add(exp.Current());
        }

        sew.Perform();

        // SameParameter is done in case Sew does not do it (Detect that the edges
        // are not sameparameter but does nothing.)

        Shape sewedShape = sew.InternalSewedShape();
        if (!sewedShape.IsNull)
        {
            exp.Init(sew.InternalSewedShape(), ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge sec = ShapeCasts.Edge(exp.Current());
                BrepLibrary.SameParameter(sec, Tool.Tolerance(sec));
            }
            _result = sewedShape;
        }
    }

    //=================================================================================================

    private bool Intersect(in Shape init,
                           in Face face,
                           Dictionary<Shape, Box> mapSBox,
                           Offset of1,
                           Inter3d inter)
    {
        bool jenRajoute = false;

        Box box1 = mapSBox.GetValue(face);

        // -----------------------------------------------
        // intersection with all already created faces.
        // -----------------------------------------------
        Shape initShape1 = of1.InitialShape();
        bool f1surBordLibre = initShape1.ShapeType() == ShapeType.Edge && _stopFaces.Contains(initShape1);

        HashSet<Shape> done = new(ShapeHasher.Instance);
        // OCCT iterates myMapSF by the shapes' addresses; the port in insertion order.
        foreach (KeyValuePair<Shape, Offset> it in _mapSF)
        {
            Offset of2 = it.Value;
            Face f2 = of2.InternalFace();

            if (box1.IsOut(mapSBox.GetValue(f2)))
            {
                continue;
            }

            if (inter.IsDone(face, f2))
            {
                continue;
            }

            // 2 tubes created on free border are not intersected.
            Shape initShape2 = of2.InitialShape();
            bool f2surBordLibre = initShape2.ShapeType() == ShapeType.Edge && _stopFaces.Contains(initShape2);

            if (f1surBordLibre && f2surBordLibre)
            {
                continue;
            }

            // -------------------------------------------------------
            // Tubes are not intersected with neighbor faces.
            // -------------------------------------------------------
            Shape itKey = it.Key;

            if (init.ShapeType() == ShapeType.Edge)
            {
                if (itKey.ShapeType() == global::Kerf.Topology.ShapeType.Face && IsInFace(ShapeCasts.Edge(init), ShapeCasts.Face(itKey)))
                {
                    continue;
                }
            }

            inter.FaceInter(face, f2, _initOffsetFace);

            // ------------------------------------------
            // an edge of F1 or F2 has been touched ?
            // if yes, add faces in myFaces
            //   ==> JenRajoute = True
            // ------------------------------------------
            NCollection_List<Shape> lInt = new();
            done.Clear();
            if (_asDes.HasCommonDescendant(face, f2, lInt))
            {
                NCollection_List<Shape>.Iterator itl2 = new();
                for (itl2.Initialize(lInt); itl2.More(); itl2.Next())
                {
                    Edge curE = ShapeCasts.Edge(itl2.Value());
                    Exploration.Vertices(curE, out Vertex v1, out Vertex v2);

                    if (done.Add(v1))
                    {
                        bool isOnR1 = IsOnRestriction(v1, curE, face, out Edge e1);
                        bool isOnR2 = IsOnRestriction(v1, curE, f2, out Edge e2);
                        if (isOnR1)
                        {
                            if (!_stopFaces.Contains(init))
                            {
                                Add(e1, _edges, init, of1, _analyse, isOnR1 && isOnR2);
                                jenRajoute = true;
                            }
                        }
                        if (isOnR2)
                        {
                            if (!_stopFaces.Contains(itKey))
                            {
                                Add(e2, _edges, itKey, of2, _analyse, isOnR1 && isOnR2);
                                jenRajoute = true;
                            }
                        }
                    }

                    if (done.Add(v2))
                    {
                        bool isOnR1 = IsOnRestriction(v2, curE, face, out Edge e1);
                        bool isOnR2 = IsOnRestriction(v2, curE, f2, out Edge e2);

                        // If IsOnR1 && IsOnR2,
                        // Leave in the same tps on 2 faces, propagate only on
                        // free borders.
                        // A priori, only facet is closed.
                        if (isOnR1)
                        {
                            if (!_stopFaces.Contains(init))
                            {
                                Add(e1, _edges, init, of1, _analyse, isOnR1 && isOnR2);
                                jenRajoute = true;
                            }
                        }
                        if (isOnR2)
                        {
                            if (!_stopFaces.Contains(itKey))
                            {
                                Add(e2, _edges, itKey, of2, _analyse, isOnR1 && isOnR2);
                                jenRajoute = true;
                            }
                        }
                    }
                }
            }
        }

        return jenRajoute;
    }
}
