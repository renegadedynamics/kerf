// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Modifier.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Modifier.lxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_Modifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Topology.Tools;

/// <summary>
/// Applies a <see cref="Modification"/> to a shape: rebuilds every sub-shape whose geometry, or one of
/// whose sub-shapes, the modification changes, and shares the others.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). The input shape is never written into: the rebuilt shapes are new. OCCT's
/// "mutable input" mode, which updates the input's vertices in place, is internal (<c>BRepTools_Modifier(true)</c>,
/// <c>SetMutableInput</c>), since every shape a caller holds is frozen and would throw.
/// </para>
/// <para>
/// <see cref="ModifiedShape"/> is a publication point (docs/phase3-design.md, section 4.2): it freezes the shape it
/// returns. Ported code that goes on building on the result reads it through <see cref="InternalModifiedShape"/>.
/// </para>
/// </remarks>
public sealed class Modifier
{
    private struct NewCurveInfo
    {
        public Curve3D? myCurve;
        public Placement myLoc;
        public double Toler;
    }

    private struct NewSurfaceInfo
    {
        public Surface? mySurface;
        public Placement myLoc;
        public double Toler;
        public bool myRevWires;
        public bool myRevFace;
    }

    private readonly Dictionary<Shape, Shape> _map = new(ShapeHasher.Instance);
    private Shape _shape = new();
    private bool _done;
    private readonly Dictionary<Shape, NewCurveInfo> _ncInfo = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, NewSurfaceInfo> _nsInfo = new(ShapeHasher.Instance);
    private readonly HashSet<Shape> _nonUpdFace = new(ShapeHasher.Instance);
    private readonly HashSet<Shape> _hasNewGeom = new(ShapeHasher.Instance);
    private bool _mutableInput;

    /// <summary>An empty modifier.</summary>
    public Modifier()
        : this(false)
    {
    }

    /// <summary>An empty modifier; with <paramref name="mutableInput"/> the input's vertices may be updated in place.</summary>
    internal Modifier(bool mutableInput)
    {
        _done = false;
        _mutableInput = mutableInput;
    }

    /// <summary>A modifier on the shape.</summary>
    public Modifier(in Shape s)
    {
        _shape = s;
        _done = false;
        _mutableInput = false;
        Put(s);
    }

    /// <summary>A modifier on the shape, performing the modification.</summary>
    public Modifier(in Shape s, Modification m)
    {
        _shape = s;
        _done = false;
        _mutableInput = false;
        Put(s);
        Perform(m);
    }

    /// <summary>Initializes the modifier with the shape.</summary>
    public void Init(in Shape s)
    {
        _shape = s;
        _done = false;
        Put(s);
    }

    /// <summary>Performs the modification.</summary>
    /// <exception cref="NullObjectException">The modifier has no shape.</exception>
    public void Perform(Modification m, ProgressRange progress = default)
    {
        if (_shape.IsNull)
        {
            throw new NullObjectException();
        }

        using var ps = new ProgressScope(progress, "Converting Shape", 2);

        var mve = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        var mef = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(_shape, ShapeType.Vertex, ShapeType.Edge, mve);
        Exploration.MapShapesAndAncestors(_shape, ShapeType.Edge, ShapeType.Face, mef);

        CreateNewVertices(mve, m);

        FillNewCurveInfo(mef, m);

        FillNewSurfaceInfo(m);

        if (!_mutableInput)
        {
            CreateOtherVertices(mve, mef, m);
        }

        Rebuild(_shape, m, out _, ps.Next());

        if (!ps.More())
        {
            // The processing was broken
            return;
        }

        if (_shape.Type == ShapeType.Face)
        {
            if (_shape.Orientation == Reversed)
            {
                _map.GetValueRef(_shape).Reverse();
            }
            else
            {
                _map.GetValueRef(_shape).Orient(_shape.Orientation);
            }
        }
        else
        {
            _map.GetValueRef(_shape).Orient(_shape.Orientation);
        }

        // Update the continuities

        BrepBuilder bb = new BrepBuilder();

        for (int ii = 0; ii < mef.Count; ii++)
        {
            Edge curE = ShapeCasts.Edge(mef.KeyAt(ii));
            Edge newE = ShapeCasts.Edge(_map.GetValue(curE));
            if (!curE.IsSame(newE))
            {
                var it = new NCollection_List<Shape>.Iterator();
                it.Initialize(mef.GetValue(curE));
                Face f1 = new(), f2 = new();
                while (it.More() && f2.IsNull)
                {
                    if (f1.IsNull)
                    {
                        f1 = ShapeCasts.Face(it.Value());
                    }
                    else
                    {
                        f2 = ShapeCasts.Face(it.Value());
                    }
                    it.Next();
                }
                if (!f2.IsNull)
                {
                    Face newf1 = ShapeCasts.Face(_map.GetValue(f1));
                    Face newf2 = ShapeCasts.Face(_map.GetValue(f2));
                    Continuity newcont = m.Continuity(curE, f1, f2, newE, newf1, newf2);
                    // OCCT.NET: frozen: an edge rebuilt by an earlier Perform (Init keeps the images, occt-issues.md)
                    // may have been published since; OCCT sets its continuity again, which changes nothing when it
                    // already has it
                    if (newcont > C0
                        && !(newE.TShape()!.IsFrozen && Tool.HasContinuity(newE, newf1, newf2)
                             && Tool.Continuity(newE, newf1, newf2) == newcont))
                    {
                        bb.Continuity(newE, newf1, newf2, newcont);
                    }
                }
            }
        }

        _done = true;
    }

    /// <summary>True if the modification has been computed successfully.</summary>
    public bool IsDone => _done;

    /// <summary>True if the input shape may be modified (OCCT's mutable input mode).</summary>
    public bool IsMutableInput => _mutableInput;

    /// <summary>Sets the mutable input mode: the input's vertices are then updated in place.</summary>
    internal void SetMutableInput(bool mutableInput) => _mutableInput = mutableInput;

    /// <summary>The shape that the sub-shape S became, frozen (a publication point).</summary>
    /// <exception cref="NoSuchObjectException">S isn't a sub-shape of the modified shape.</exception>
    public Shape ModifiedShape(in Shape s)
    {
        Shape result = InternalModifiedShape(s);
        result.TShape()?.FreezeGraph();
        return result;
    }

    /// <summary>The shape that the sub-shape S became, unfrozen if it was rebuilt: for ported code that goes on building on it.</summary>
    /// <exception cref="NoSuchObjectException">S isn't a sub-shape of the modified shape.</exception>
    internal Shape InternalModifiedShape(in Shape s)
    {
        if (!_map.ContainsKey(s))
        {
            throw new NoSuchObjectException();
        }
        return _map.GetValue(s);
    }

    private void Put(in Shape s)
    {
        if (!_map.ContainsKey(s))
        {
            _map[s] = new Shape();
            for (var theIterator = new Iterator(s, false); theIterator.More(); theIterator.Next())
            {
                Put(theIterator.Value());
            }
        }
    }

    private bool Rebuild(in Shape s, Modification m, out bool newGeom, ProgressRange progress)
    {
        ShapeType ts = s.Type;
        // OCCT.NET: a reference into the map, as OCCT's TopoDS_Shape& result = myMap(S). The map gains no keys while
        // rebuilding (Put bound every sub-shape), so the reference stays valid through the recursion.
        ref Shape result = ref _map.GetValueRef(s);
        if (!result.IsNull)
        {
            newGeom = _hasNewGeom.Contains(s);
            return !s.IsSame(result);
        }
        bool rebuild = false, revWires = false;
        Orientation resOr = Forward;
        BrepBuilder b = new BrepBuilder();
        double tol = 0.0;
        bool no3DCurve = false; // in fact, if we have no
        // 3D geometry modification, it is necessary to test the existence of a 3D curve.

        // new geometry ?

        switch (ts)
        {
            case ShapeType.Face:
            {
                rebuild = _nsInfo.ContainsKey(ShapeCasts.Face(s));
                if (rebuild)
                {
                    NewSurfaceInfo nSinfo = _nsInfo.GetValue(ShapeCasts.Face(s));
                    revWires = nSinfo.myRevWires;
                    Face resultF = ShapeCasts.Face(result);
                    b.MakeFace(ref resultF,
                               nSinfo.mySurface,
                               nSinfo.myLoc.Predivided(s.Placement),
                               nSinfo.Toler);
                    result = resultF;
                    result.Location(s.Placement, false);
                    if (nSinfo.myRevFace)
                    {
                        resOr = Reversed;
                    }
                    // set specifics flags of a Face
                    b.NaturalRestriction(ShapeCasts.Face(result), Tool.NaturalRestriction(ShapeCasts.Face(s)));
                }

                // update triangulation on the copied face
                if (m.NewTriangulation(ShapeCasts.Face(s), out Triangulation? triangulation))
                {
                    if (rebuild)
                    { // the copied face already exists => update it
                        b.UpdateFace(ShapeCasts.Face(result), triangulation);
                    }
                    else
                    { // create new face with the new triangulation, on the face's own surface if it has one (a
                      // modification keeping the surface, as NurbsConvert does a B-spline or Bezier face's), bare
                      // otherwise (decision 1476; OCCT made it bare, so a meshed face lost its surface)
                        Face resultF = ShapeCasts.Face(result);
                        Face sF = ShapeCasts.Face(s);
                        Surface? aSurface = Tool.Surface(sF, out Placement aSurfaceLoc);
                        if (aSurface is not null)
                        {
                            b.MakeFace(ref resultF, aSurface, aSurfaceLoc.Predivided(s.Placement), Tool.Tolerance(sF));
                            b.UpdateFace(resultF, triangulation);
                            b.NaturalRestriction(resultF, Tool.NaturalRestriction(sF));
                        }
                        else
                        {
                            b.MakeFace(ref resultF, triangulation);
                        }
                        result = resultF;
                        result.Location(s.Placement, false);
                    }
                    rebuild = true;
                }
            }
                break;

            case ShapeType.Edge:
            {
                rebuild = _ncInfo.ContainsKey(ShapeCasts.Edge(s));
                if (rebuild)
                {
                    NewCurveInfo nCinfo = _ncInfo.GetValue(ShapeCasts.Edge(s));
                    Edge resultE = ShapeCasts.Edge(result);
                    if (nCinfo.myCurve is null)
                    {
                        b.MakeEdge(ref resultE);
                        b.Degenerated(resultE, Tool.Degenerated(ShapeCasts.Edge(s)));
                        b.UpdateEdge(resultE, nCinfo.Toler); // OCC217
                        no3DCurve = true;
                    }
                    else
                    {
                        b.MakeEdge(ref resultE,
                                   nCinfo.myCurve,
                                   nCinfo.myLoc.Predivided(s.Placement),
                                   nCinfo.Toler);
                        no3DCurve = false;
                    }
                    result = resultE;
                    result.Location(s.Placement, false);
                    //	result.Orientation(S.Orientation());

                    // set specifics flags of an Edge
                    b.SameParameter(ShapeCasts.Edge(result), Tool.SameParameter(ShapeCasts.Edge(s)));
                    b.SameRange(ShapeCasts.Edge(result), Tool.SameRange(ShapeCasts.Edge(s)));
                }

                // update polygonal structure on the edge
                if (m.NewPolygon(ShapeCasts.Edge(s), out Polygon3D? polygon))
                {
                    if (rebuild)
                    { // the copied edge already exists => update it
                        b.UpdateEdge(ShapeCasts.Edge(result), polygon, s.Placement);
                    }
                    else
                    { // create new edge with bare polygon
                        Edge resultE = ShapeCasts.Edge(result);
                        b.MakeEdge(ref resultE, polygon);
                        result = resultE;
                        result.Location(s.Placement, false);
                    }
                    rebuild = true;
                }
            }
                break;
            default:
                break;
        }

        // rebuild sub-shapes and test new sub-shape ?

        bool newgeom = rebuild;
        newGeom = rebuild;

        Iterator it = new Iterator();

        {
            int shapeCount = 0;
            {
                for (it.Initialize(s, false); it.More(); it.Next())
                {
                    ++shapeCount;
                }
            }

            using var ps = new ProgressScope(progress, "Converting SubShapes", shapeCount);
            //
            for (it.Initialize(s, false); it.More() && ps.More(); it.Next())
            {
                // always call Rebuild
                bool subrebuilt = Rebuild(it.Value(), m, out bool isSubNewGeom, ps.Next());
                rebuild = subrebuilt || rebuild;
                newGeom = newGeom || isSubNewGeom;
            }
            if (!ps.More())
            {
                // The processing was broken
                return false;
            }
        }
        if (newGeom)
        {
            _hasNewGeom.Add(s);
        }

        // make an empty copy
        if (rebuild && !newgeom)
        {
            result = s.EmptyCopied();
            result.Orient(Forward);
        }

        // copy the sub-elements

        if (rebuild)
        {
            Orientation orient;
            for (it.Initialize(s, false); it.More(); it.Next())
            {
                orient = it.Value().Orientation;
                if (revWires || _map.GetValue(it.Value()).Orientation == Reversed)
                {
                    orient = global::Kerf.Engine.Topology.TopAbs.Reverse(orient);
                }
                b.Add(result, _map.GetValue(it.Value()).Oriented(orient));
            }

            if (ts == ShapeType.Face)
            {
                // pcurves
                Face face = ShapeCasts.Face(s);
                Orientation fcor = face.Orientation;
                if (fcor != Reversed)
                {
                    fcor = Forward;
                }

                var ex = new Explorer(face.Oriented(fcor), ShapeType.Edge);
                for (; ex.More(); ex.Next())
                {
                    Edge edge = ShapeCasts.Edge(ex.Current());

                    Curve2D? curve2d = null;
                    if (newGeom
                        && m.NewCurve2d(edge,
                                        face,
                                        ShapeCasts.Edge(_map.GetValue(ex.Current())),
                                        ShapeCasts.Face(result),
                                        out curve2d,
                                        out tol))
                    {
                        // rem dub 16/09/97 : Make constant topology or not make at all.
                        // Do not make if CopySurface = 1
                        // Attention, TRUE sewing edges (ReallyClosed)
                        // stay even if  CopySurface is true.

                        // check that edge contains two pcurves on this surface:
                        // either it is true seam on the current face, or belongs to two faces
                        // built on that same surface (see OCC21772)
                        // Note: this check could be made separate method in BRepTools
                        bool isClosed = false;
                        if (Tool.IsClosed(edge, face))
                        {
                            isClosed = (!newgeom || BrepTools.IsReallyClosed(edge, face));
                            if (!isClosed)
                            {
                                Shape resface = (_map.ContainsKey(face) ? _map.GetValue(face) : face);
                                if (resface.IsNull)
                                {
                                    resface = face;
                                }
                                Surface? surf = Tool.Surface(ShapeCasts.Face(resface), out Placement loc);
                                // check other faces sharing the same surface
                                var expF = new Explorer(_shape, ShapeType.Face);
                                for (; expF.More() && !isClosed; expF.Next())
                                {
                                    Face other = ShapeCasts.Face(expF.Current());
                                    if (other.IsSame(face))
                                    {
                                        continue;
                                    }
                                    Shape resface2 = (_map.ContainsKey(other) ? _map.GetValue(other) : other);
                                    if (resface2.IsNull)
                                    {
                                        resface2 = other;
                                    }
                                    Surface? otherSurf =
                                        Tool.Surface(ShapeCasts.Face(resface2), out Placement otherLoc);
                                    if (ReferenceEquals(surf, otherSurf) && loc.IsEqual(otherLoc))
                                    {
                                        var expE = new Explorer(other, ShapeType.Edge);
                                        for (; expE.More() && !isClosed; expE.Next())
                                        {
                                            isClosed = edge.IsSame(expE.Current());
                                        }
                                    }
                                }
                            }
                        }
                        if (isClosed)
                        {
                            Edge curE = ShapeCasts.Edge(_map.GetValue(edge));
                            Shape localResult = result;
                            localResult.Orient(Forward);
                            Face curF = ShapeCasts.Face(localResult);
                            Curve2D? curve2d1;
                            double f, l;
                            if ((!revWires && fcor != edge.Orientation)
                                || (revWires && fcor == edge.Orientation))
                            {
                                curE.Orient(Forward);
                                curve2d1 = Tool.CurveOnSurface(curE, curF, out f, out l);
                                if (curve2d1 is null)
                                {
                                    curve2d1 = new LineCurve2D(Constants.OX2d());
                                }
                                b.UpdateEdge(curE, curve2d1, curve2d, curF, 0.0);
                            }
                            else
                            {
                                curE.Orient(Reversed);
                                curve2d1 = Tool.CurveOnSurface(curE, curF, out f, out l);
                                if (curve2d1 is null)
                                {
                                    curve2d1 = new LineCurve2D(Constants.OX2d());
                                }
                                b.UpdateEdge(curE, curve2d, curve2d1, curF, 0.0);
                            }
                            Tool.CurveOnSurface(curE, curF, out f, out l);
                            b.Range(curE, f, l);
                        }
                        else
                        {
                            b.UpdateEdge(ShapeCasts.Edge(_map.GetValue(ex.Current())), curve2d, ShapeCasts.Face(result), 0.0);
                        }

                        Curve3D? c3D =
                            Tool.Curve(ShapeCasts.Edge(_map.GetValue(ex.Current())), out _, out _, out _);
                        if (c3D is null)
                        { // Update vertices
                            double param = 0.0;
                            var ex2 = new Explorer(edge, ShapeType.Vertex);
                            while (ex2.More())
                            {
                                Vertex vertex = ShapeCasts.Vertex(ex2.Current());
                                if (!m.NewParameter(vertex, edge, ref param, ref tol))
                                {
                                    tol = Tool.Tolerance(vertex);
                                    param = Tool.Parameter(vertex, edge);
                                }

                                Orientation vtxrelat = vertex.Orientation;
                                if (edge.Orientation == Reversed)
                                {
                                    // Update considers the edge as FORWARD, and the vertex relatively
                                    vtxrelat = global::Kerf.Engine.Topology.TopAbs.Reverse(vtxrelat);
                                }
                                // if (myMap(edge).Orientation() == TopAbs_REVERSED) {
                                //   vtxrelat= TopAbs::Reverse(vtxrelat);
                                // }

                                Vertex localVertex = ShapeCasts.Vertex(_map.GetValue(vertex));
                                localVertex.Orient(vtxrelat);
                                // B.UpdateVertex(TopoDS::Vertex
                                //(myMap(vertex).Oriented(vtxrelat)),
                                b.UpdateVertex(localVertex, param, ShapeCasts.Edge(_map.GetValue(edge)), tol);
                                ex2.Next();
                            }
                        }
                    }

                    // Copy polygon on triangulation
                    bool newPonT = m.NewPolygonOnTriangulation(edge, face, out PolygonOnTriangulation? polyOnTria_1);
                    PolygonOnTriangulation? polyOnTria_2 = null;
                    if (BrepTools.IsReallyClosed(edge, face))
                    {
                        // Obtain triangulation on reversed edge
                        Edge edgeRev = edge;
                        edgeRev.Reverse();
                        newPonT = m.NewPolygonOnTriangulation(edgeRev, face, out polyOnTria_2) || newPonT;
                        // It there is only one polygon on triangulation, store it to aPolyOnTria_1
                        if (polyOnTria_1 is null && polyOnTria_2 is not null)
                        {
                            polyOnTria_1 = polyOnTria_2;
                            polyOnTria_2 = null;
                        }
                    }
                    if (newPonT)
                    {
                        Triangulation? newFaceTria =
                            Tool.Triangulation(ShapeCasts.Face(_map.GetValue(face)), out Placement location);
                        Edge newEdge = ShapeCasts.Edge(_map.GetValue(edge));
                        if (polyOnTria_2 is null)
                        {
                            b.UpdateEdge(newEdge, polyOnTria_1, newFaceTria, location);
                        }
                        else
                        {
                            if (edge.Orientation == Forward)
                            {
                                b.UpdateEdge(newEdge, polyOnTria_1, polyOnTria_2, newFaceTria, location);
                            }
                            else
                            {
                                b.UpdateEdge(newEdge, polyOnTria_2, polyOnTria_1, newFaceTria, location);
                            }
                        }
                    }
                }
            }

            //    else if (ts == TopAbs_EDGE) {
            else if (ts == ShapeType.Edge && !no3DCurve)
            {
                // Vertices
                double param = 0.0;
                Edge edge = ShapeCasts.Edge(s);
                Orientation edor = edge.Orientation;
                if (edor != Reversed)
                {
                    edor = Forward;
                }
                var ex = new Explorer(edge.Oriented(edor), ShapeType.Vertex);
                while (ex.More())
                {
                    Vertex vertex = ShapeCasts.Vertex(ex.Current());

                    if (!m.NewParameter(vertex, edge, ref param, ref tol))
                    {
                        tol = Tool.Tolerance(vertex);
                        param = Tool.Parameter(vertex, edge);
                    }

                    Orientation vtxrelat = vertex.Orientation;
                    if (edor == Reversed)
                    {
                        // Update considers the edge as FORWARD, and the vertex relatively
                        vtxrelat = global::Kerf.Engine.Topology.TopAbs.Reverse(vtxrelat);
                    }

                    // if (result.Orientation() == TopAbs_REVERSED) {
                    //   vtxrelat= TopAbs::Reverse(vtxrelat);
                    // }
                    Vertex localVertex = ShapeCasts.Vertex(_map.GetValue(vertex));
                    localVertex.Orient(vtxrelat);
                    // B.UpdateVertex(TopoDS::Vertex(myMap(vertex).Oriented(vtxrelat)),
                    if (_mutableInput || !localVertex.IsSame(vertex))
                    {
                        b.UpdateVertex(localVertex, param, ShapeCasts.Edge(result), tol);
                    }
                    ex.Next();
                }
            }

            // update flags

            result.Orientable(s.IsOrientable);
            result.Closed(s.IsClosed);
            result.Infinite(s.IsInfinite);
        }
        else
        {
            result = s;
        }

        // Set flag of the shape.
        result.Orient(resOr);

        // OCCT.NET: frozen: an unrebuilt shape is the input itself, whose flags SetShapeFlags copies onto themselves;
        // a frozen one can't be written, and keeps them.
        if (!(ReferenceEquals(result.TShape(), s.TShape()) && s.TShape()!.IsFrozen))
        {
            SetShapeFlags(s, result);
        }

        return rebuild;
    }

    private void CreateNewVertices(IndexedMap<Shape, NCollection_List<Shape>> mve,
                                   Modification m)
    {
        BrepBuilder bb = new BrepBuilder();
        for (int i = 0; i < mve.Count; i++)
        {
            // fill MyMap only with vertices with NewPoint == true
            Vertex v = ShapeCasts.Vertex(mve.KeyAt(i));
            bool isNewP = m.NewPoint(v, out Point3D pnt, out double toler);
            if (isNewP)
            {
                Vertex newV = new();
                bb.MakeVertex(ref newV, pnt, toler);
                SetShapeFlags(v, newV);
                _map.GetValueRef(v) = newV;
                _hasNewGeom.Add(v);
            }
            else if (_mutableInput)
            {
                _map.GetValueRef(v) = v.Oriented(Forward);
            }
        }
    }

    private void FillNewCurveInfo(IndexedMap<Shape, NCollection_List<Shape>> mef,
                                  Modification m)
    {
        NewCurveInfo nCinfo = new NewCurveInfo();
        for (int i = 0; i < mef.Count; i++)
        {
            Edge e = ShapeCasts.Edge(mef.KeyAt(i));
            bool isNewCur = m.NewCurve(e, out Curve3D? curve, out Placement location, out double toler);
            if (isNewCur)
            {
                nCinfo.myCurve = curve;
                nCinfo.myLoc = location;
                nCinfo.Toler = toler;
                _ncInfo[e] = nCinfo;
                _hasNewGeom.Add(e);
            }
        }
    }

    private void FillNewSurfaceInfo(Modification m)
    {
        var mf = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(_shape, ShapeType.Face, mf);
        NewSurfaceInfo nSinfo = new NewSurfaceInfo();
        for (int i = 0; i < mf.Count; i++)
        {
            Face f = ShapeCasts.Face(mf[i]);
            bool isNewSur = m.NewSurface(f,
                                         out Surface? surface,
                                         out Placement location,
                                         out double toler1,
                                         out bool revWires,
                                         out bool revFace);
            if (isNewSur)
            {
                nSinfo.mySurface = surface;
                nSinfo.myLoc = location;
                nSinfo.Toler = toler1;
                nSinfo.myRevWires = revWires;
                nSinfo.myRevFace = revFace;
                _nsInfo[f] = nSinfo;
                _hasNewGeom.Add(f);
            }
            else
            {
                // check if subshapes will be modified
                bool notRebuilded = true;
                var exE = new Explorer(f, ShapeType.Edge);
                while (exE.More() && notRebuilded)
                {
                    Edge ee = ShapeCasts.Edge(exE.Current());
                    if (_ncInfo.ContainsKey(ee))
                    {
                        notRebuilded = false;
                        break;
                    }
                    var exV = new Explorer(ee, ShapeType.Vertex);
                    while (exV.More() && notRebuilded)
                    {
                        Vertex vv = ShapeCasts.Vertex(exV.Current());
                        if (!_map.GetValue(vv).IsNull)
                        {
                            notRebuilded = false;
                            break;
                        }
                        exV.Next();
                    }
                    exE.Next();
                }
                if (notRebuilded)
                {
                    // subshapes is not going to be modified
                    _nonUpdFace.Add(f);
                }
            }
        }
    }

    private void CreateOtherVertices(IndexedMap<Shape, NCollection_List<Shape>> mve,
                                     IndexedMap<Shape, NCollection_List<Shape>> mef,
                                     Modification m)
    {
        // The following logic in some ways repeats the logic from the Rebuild() method.
        // If the face with its subshapes is not going to be modified
        //(i.e. NewSurface() for this face and NewCurve(), NewPoint() for its edges/vertices returns
        // false) then the calling of NewCurve2d() for this face with its edges is not performed.
        // Therefore, the updating of vertices will not present in such cases and
        // the EmptyCopied() operation for vertices from this face is not needed.

        for (int i = 0; i < mve.Count; i++)
        {
            Vertex v = ShapeCasts.Vertex(mve.KeyAt(i));
            Vertex newV = ShapeCasts.Vertex(_map.GetValue(v));
            if (newV.IsNull)
            {
                NCollection_List<Shape> lEdges = mve.ValueAt(i);
                bool toReplace = false;
                var it = new NCollection_List<Shape>.Iterator(lEdges);
                for (; it.More() && !toReplace; it.Next())
                {
                    Edge e = ShapeCasts.Edge(it.Value());
                    if (_ncInfo.ContainsKey(e) && _ncInfo.GetValue(e).myCurve is not null)
                    {
                        toReplace = true;
                    }

                    if (!toReplace)
                    {
                        NCollection_List<Shape> lFaces = mef.GetValue(e);
                        var it2 = new NCollection_List<Shape>.Iterator(lFaces);
                        for (; it2.More(); it2.Next())
                        {
                            Face f = ShapeCasts.Face(it2.Value());
                            if (!_nonUpdFace.Contains(f))
                            {
                                // some NewCurve2d()s may use NewE arg internally, so the
                                // null TShape as an arg may lead to the exceptions
                                Edge dummyE = ShapeCasts.Edge(e.EmptyCopied());
                                if (m.NewCurve2d(e, f, dummyE, new Face(), out _, out _))
                                {
                                    toReplace = true;
                                    break;
                                }
                            }
                        }
                    }
                }
                if (toReplace)
                {
                    newV = ShapeCasts.Vertex(v.EmptyCopied());
                }
                else
                {
                    newV = v;
                }
                newV.Orient(Forward);
                _map.GetValueRef(v) = newV;
            }
        }
    }

    private static void SetShapeFlags(in Shape inSh, in Shape outSh)
    {
        outSh.Modified(inSh.Modified());
        outSh.Checked(inSh.Checked());
        outSh.Orientable(inSh.IsOrientable);
        outSh.Closed(inSh.IsClosed);
        outSh.Infinite(inSh.IsInfinite);
        outSh.Convex(inSh.IsConvex);
    }
}
