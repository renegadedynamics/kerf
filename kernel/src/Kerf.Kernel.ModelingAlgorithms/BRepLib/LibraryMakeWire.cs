// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeWire.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeWire.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_MakeWire_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Construction.Library.LibraryWireError;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Library;

/// <summary>
/// Makes a wire from edges, added one at a time or as a list. An edge must share a vertex with the wire, or have a
/// vertex within the tolerance of one of the wire's (the edge is then copied onto the wire's vertex, which is moved
/// and enlarged to cover both); a list is connected whatever its order, coincident vertices merged.
/// </summary>
/// <remarks>
/// <para><see cref="Wire"/>, <see cref="Edge"/> and <see cref="Vertex"/> freeze the wire (design 4.2); ported code
/// reads the internal forms. Adding after a public read goes on with a new wire holding the same edges.</para>
/// <para>The caller's edges and vertices are frozen. Merging a vertex onto a wire vertex that must move or grow writes
/// into that vertex, as OCCT does; the change runs in a working overlay (unit 10H3), so the wire holds new TShapes for
/// what changed and the caller's own for the rest; <see cref="InternalHistory"/> records the caller's edges and
/// vertices replaced.</para>
/// </remarks>
public class LibraryMakeWire : LibraryMakeShape
{
    private LibraryWireError _error;
    private Edge _edge = new();
    private Vertex _vertex = new();
    private readonly NCollection_IndexedMap<Shape> _vertices = new(ShapeHasher.Instance);
    private Vertex _firstVertex = new();
    private Vertex _vf = new();
    private Vertex _vl = new();
    // Kerf: the history of the caller's sub-shapes the changes replaced (null if none).
    private History? _history;

    /// <summary>Makes an empty wire: <see cref="Error"/> is BRepLib_EmptyWire until an edge is added.</summary>
    public LibraryMakeWire()
    {
        _error = LibraryWireError.EmptyWire;
    }

    public LibraryMakeWire(in Edge e)
    {
        Add(e);
    }

    public LibraryMakeWire(in Edge e1, in Edge e2)
    {
        Add(e1);
        Add(e2);
    }

    public LibraryMakeWire(in Edge e1, in Edge e2, in Edge e3)
    {
        Add(e1);
        Add(e2);
        Add(e3);
    }

    public LibraryMakeWire(in Edge e1, in Edge e2, in Edge e3, in Edge e4)
    {
        Add(e1);
        Add(e2);
        Add(e3);
        Add(e4);
    }

    public LibraryMakeWire(in Wire w)
    {
        Add(w);
    }

    public LibraryMakeWire(in Wire w, in Edge e)
    {
        Add(w);
        Add(e);
    }

    /// <summary>Adds the edges of a wire, stopping at the first that can't be added.</summary>
    public void Add(in Wire w)
    {
        for (var it = new Iterator(w); it.More(); it.Next())
        {
            Add(ShapeCasts.Edge(it.Value()));
            if (_error != LibraryWireError.WireDone)
            {
                break;
            }
        }
    }

    /// <summary>
    /// Adds an edge connected to the wire by a shared vertex or a vertex within tolerance (the edge is then copied
    /// onto the wire's vertex).
    /// </summary>
    public void Add(in Edge e)
    {
        WorkingOverlay? overlay = BeginChange(IsMerging(e));
        try
        {
            Add(e, true);
        }
        finally
        {
            EndChange(overlay);
        }
    }

    // ------------------------------------------------------------ Kerf: a change of the wire in a working overlay

    // Kerf (unit 10H3, in place of decision 417's copies for write): a change writes, as OCCT's, into the caller's
    // vertices (a vertex merged onto another is moved and enlarged, parameters are transferred) and into a wire a public
    // accessor has published. It runs in a working overlay: when the wire is published, or when the edge may be merged
    // (isNeeded), and unless the change runs inside an operation's overlay, whose writes these then are. At the end the
    // wire holds new TShapes for what changed and the caller's own for the rest, the maker's state (its vertices, VF,
    // VL, the last edge and vertex) is brought up to date, and InternalHistory records what was replaced.
    private WorkingOverlay? BeginChange(bool isNeeded)
    {
        if (!_shape.IsNull && _shape.TShape()!.IsFrozen)
        {
            return WorkingOverlay.Begin(_shape);
        }
        return isNeeded ? WorkingOverlay.BeginUnlessWriting(new Shape()) : null;
    }

    // True if adding the edge may merge one of its vertices onto the wire's: a vertex of it isn't one of the wire's.
    private bool IsMerging(in Edge e)
    {
        if (_edge.IsNull)
        {
            return false;
        }
        for (var it = new Iterator(e); it.More(); it.Next())
        {
            if (!_vertices.Contains(it.Value()))
            {
                return true;
            }
        }
        return false;
    }

    private void EndChange(WorkingOverlay? overlay)
    {
        if (overlay is null)
        {
            return;
        }
        try
        {
            if (_shape.IsNull || overlay.WorkingCount == 0)
            {
                return; // nothing was written (a merge that moved and enlarged nothing)
            }
            // the wire's sub-shapes and the maker's state as the change left them
            var shapes = new List<Shape>();
            var subShapes = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(_shape, subShapes);
            for (int i = 1; i <= subShapes.Extent(); i++)
            {
                shapes.Add(subShapes[i]);
            }
            for (int i = 1; i <= _vertices.Extent(); i++)
            {
                shapes.Add(_vertices.FindKey(i));
            }
            shapes.Add(_vf);
            shapes.Add(_vl);
            shapes.Add(_firstVertex);
            shapes.Add(_vertex);
            shapes.Add(_edge);

            _shape = overlay.Finish(_shape, shapes);
            for (int i = 1; i <= _vertices.Extent(); i++)
            {
                _vertices.Substitute(i, overlay.Final(_vertices.FindKey(i)));
            }
            _vf = ShapeCasts.Vertex(overlay.Final(_vf));
            _vl = ShapeCasts.Vertex(overlay.Final(_vl));
            _firstVertex = ShapeCasts.Vertex(overlay.Final(_firstVertex));
            _vertex = ShapeCasts.Vertex(overlay.Final(_vertex));
            _edge = ShapeCasts.Edge(overlay.Final(_edge));

            History images = overlay.Images(shapes);
            if (!images.HasModified)
            {
                return;
            }
            if (_history is null)
            {
                _history = images;
            }
            else
            {
                _history.Merge(images);
            }
        }
        finally
        {
            overlay.Dispose();
        }
    }

    // PMN  19/03/1998  For the Problem of performance TopExp::Vertices are not used on wire
    // PMN  10/09/1998  In case if the wire is previously closed (or degenerated)
    //                  TopExp::Vertices is used to reduce the ambiguity.
    // IsCheckGeometryProximity flag : If true => check for the geometry proximity of vertices
    private void Add(in Edge e, bool isCheckGeometryProximity)
    {
        bool forward = false;
        // to tell if it has been decided to add forward
        bool reverse = false;
        // to tell if it has been decided to add reversed
        bool init = false;
        // To know if it is necessary to calculate VL, VF
        var b = new BrepBuilder();
        var it = new Iterator();

        if (_edge.IsNull)
        {
            init = true;
            // first edge, create the wire
            Wire w = new();
            b.MakeWire(ref w);
            _shape = w;

            // set the edge
            _edge = e;

            // add the vertices
            for (it.Initialize(_edge); it.More(); it.Next())
            {
                _vertices.Add(it.Value());
            }
        }

        else
        {
            init = _shape.Closed(); // If it is closed no control
            Shape localShape = e.Oriented(Forward);
            Edge ee = ShapeCasts.Edge(localShape);

            // test the vertices of the edge

            bool connected = false;
            bool copyedge = false;

            if (_error != LibraryWireError.NonManifoldWire)
            {
                if (_vf.IsNull || _vl.IsNull)
                {
                    _error = LibraryWireError.NonManifoldWire;
                }
            }

            for (it.Initialize(ee); it.More(); it.Next())
            {
                Vertex ve = ShapeCasts.Vertex(it.Value());

                // if the vertex is in the wire, ok for the connection
                if (_vertices.Contains(ve))
                {
                    connected = true;
                    _vertex = ve;
                    if (_error != LibraryWireError.NonManifoldWire)
                    {
                        // is it always so ?
                        if (_vf.IsSame(_vl))
                        {
                            // Orientation indetermined (in 3d) : Preserve the initial
                            if (!_vf.IsSame(ve))
                            {
                                _error = LibraryWireError.NonManifoldWire;
                            }
                        }
                        else
                        {
                            if (_vf.IsSame(ve))
                            {
                                if (ve.Orientation() == Forward)
                                {
                                    reverse = true;
                                }
                                else
                                {
                                    forward = true;
                                }
                            }
                            else if (_vl.IsSame(ve))
                            {
                                if (ve.Orientation() == Reversed)
                                {
                                    reverse = true;
                                }
                                else
                                {
                                    forward = true;
                                }
                            }
                            else
                            {
                                _error = LibraryWireError.NonManifoldWire;
                            }
                        }
                    }
                }
                else if (isCheckGeometryProximity)
                {
                    // search if there is a similar vertex in the edge
                    Point3D pe = Tool.Pnt(ve);

                    for (int i = 1; i <= _vertices.Extent(); i++)
                    {
                        Vertex vw = ShapeCasts.Vertex(_vertices.FindKey(i));
                        Point3D pw = Tool.Pnt(vw);
                        double l = pe.Distance(pw);

                        if ((l < Tool.Tolerance(ve)) || (l < Tool.Tolerance(vw)))
                        {
                            copyedge = true;
                            if (_error != LibraryWireError.NonManifoldWire)
                            {
                                // is it always so ?
                                if (_vf.IsSame(_vl))
                                {
                                    // Orientation indetermined (in 3d) : Preserve the initial
                                    if (!_vf.IsSame(vw))
                                    {
                                        _error = LibraryWireError.NonManifoldWire;
                                    }
                                }
                                else
                                {
                                    if (_vf.IsSame(vw))
                                    {
                                        if (ve.Orientation() == Forward)
                                        {
                                            reverse = true;
                                        }
                                        else
                                        {
                                            forward = true;
                                        }
                                    }
                                    else if (_vl.IsSame(vw))
                                    {
                                        if (ve.Orientation() == Reversed)
                                        {
                                            reverse = true;
                                        }
                                        else
                                        {
                                            forward = true;
                                        }
                                    }
                                    else
                                    {
                                        _error = LibraryWireError.NonManifoldWire;
                                    }
                                }
                            }
                            break;
                        }
                    }
                    if (copyedge)
                    {
                        connected = true;
                    }
                }
            }

            if (!connected)
            {
                _error = LibraryWireError.DisconnectedWire;
                NotDone();
                return;
            }
            else
            {
                if (!copyedge)
                {
                    _edge = ee;
                    for (it.Initialize(ee); it.More(); it.Next())
                    {
                        _vertices.Add(it.Value());
                    }
                }
                else
                {
                    // copy the edge
                    Shape dummy = ee.EmptyCopied();
                    _edge = ShapeCasts.Edge(dummy);

                    for (it.Initialize(ee); it.More(); it.Next())
                    {
                        Vertex ve = ShapeCasts.Vertex(it.Value());
                        Point3D pe = Tool.Pnt(ve);

                        bool newvertex = false;
                        for (int i = 1; i <= _vertices.Extent(); i++)
                        {
                            Vertex vw = ShapeCasts.Vertex(_vertices.FindKey(i));
                            Point3D pw = Tool.Pnt(vw);
                            double l = pe.Distance(pw), tolE, tolW;
                            tolW = Tool.Tolerance(vw);
                            tolE = Tool.Tolerance(ve);

                            if ((l < tolE) || (l < tolW))
                            {
                                MergedVertex(pw, tolW, pe, tolE, l, out Point3D pc, out double maxtol);

                                b.UpdateVertex(vw, pc, maxtol);

                                newvertex = true;
                                _vertex = vw;
                                _vertex.Orientation(ve.Orientation());
                                b.Add(_edge, _vertex);
                                b.Transfert(ee, _edge, ve, _vertex);
                                break;
                            }
                        }
                        if (!newvertex)
                        {
                            _vertices.Add(ve);
                            b.Add(_edge, ve);
                            b.Transfert(ee, _edge, ve, ve);
                        }
                    }
                }
            }
            // Make a decision about the orientation of the edge
            // If there is an ambiguity (in 3d) preserve the orientation given at input
            // Case of ambiguity :
            // reverse and forward are false as nothing has been decided :
            //       closed wire, internal vertex ...
            // reverse and forward are true : closed or degenerated edge
            if (((forward == reverse) && (e.Orientation() == Reversed)) || (reverse && !forward))
            {
                _edge.Reverse();
            }
        }

        // add myEdge to myShape
        b.Add(_shape, _edge);
        _shape.Closed(false);

        // Initialize VF, VL
        if (init)
        {
            Exploration.Vertices(ShapeCasts.Wire(_shape), out _vf, out _vl);
        }
        else
        {
            if (_error == LibraryWireError.WireDone)
            { // Update only
                Vertex vRef = new();
                Exploration.Vertices(_edge, out Vertex v1, out Vertex v2);
                if (v1.IsSame(_vertex))
                {
                    vRef = v2;
                }
                else if (v2.IsSame(_vertex))
                {
                    vRef = v1;
                }
                else
                {
                    _error = LibraryWireError.NonManifoldWire;
                }

                if (_vf.IsSame(_vl))
                {
                    // Particular case: it is required to control the orientation
                }
                else
                { // General case
                    if (_vf.IsSame(_vertex))
                    {
                        _vf = vRef;
                    }
                    else if (_vl.IsSame(_vertex))
                    {
                        _vl = vRef;
                    }
                    else
                    {
                        _error = LibraryWireError.NonManifoldWire;
                    }
                }
            }
            if (_error == LibraryWireError.NonManifoldWire)
            {
                _vf = _vl = new Vertex(); // nullify
            }
        }
        // Test myShape is closed
        if (!_vf.IsNull && !_vl.IsNull && _vf.IsSame(_vl))
        {
            _shape.Closed(true);
        }

        // OCCT resets the error here, so a NonManifoldWire found above is never reported (occt-issues.md); only VF
        // and VL, nullified, remember it.
        _error = LibraryWireError.WireDone;
        Done();
    }

    // The new point and tolerance of a wire vertex VW onto which an edge's vertex VE within tolerance is merged
    // (OCCT's code in Add, put in a function so that needsWritableVertices computes the same values).
    private static void MergedVertex(in Point3D pw, double tolW, in Point3D pe, double tolE, double l,
                                     out Point3D pc, out double maxtol)
    {
        double cW, cE;
        maxtol = .5 * (tolW + tolE + l);
        if (maxtol > tolW && maxtol > tolE)
        {
            cW = (maxtol - tolE) / l;
            cE = 1.0 - cW;
        }
        else if (maxtol > tolW)
        {
            maxtol = tolE;
            cW = 0.0;
            cE = 1.0;
        }
        else
        {
            maxtol = tolW;
            cW = 1.0;
            cE = 0.0;
        }

        pc = new Point3D(cW * pw.X() + cE * pe.X(),
                        cW * pw.Y() + cE * pe.Y(),
                        cW * pw.Z() + cE * pe.Z());
    }

    /// <summary>
    /// Adds the edges of a list, in any order: coincident vertices of the list and the wire are merged into one
    /// (an existing wire vertex enlarged, or a new vertex), and the edges are added by connection. Not done if some
    /// edges couldn't be connected.
    /// </summary>
    public void Add(NCollection_List<Shape> l)
    {
        _error = LibraryWireError.WireDone;
        int lSize = 0;
        int refSize = l.Size();
        WorkingOverlay? overlay = l.IsEmpty() ? null : BeginChange(true);
        try
        {
            AddList(l, ref lSize);
        }
        finally
        {
            EndChange(overlay);
        }
        if (lSize == refSize)
        {
            Done();
        }
        else
        {
            NotDone();
            _error = LibraryWireError.DisconnectedWire;
        }
    }

    private void AddList(NCollection_List<Shape> l, ref int lSize)
    {
        if (!l.IsEmpty())
        {
            //
            var grVL = new NCollection_List<NCollection_List<Vertex>>();

            var mapVE = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);

            CollectCoincidentVertices(l, grVL);

            var o2NV = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);

            CreateNewVertices(grVL, o2NV);

            var newEList = new NCollection_List<Shape>();

            CreateNewListOfEdges(l, o2NV, newEList);
            //

            Exploration.MapShapesAndAncestors(_shape, global::Kerf.Topology.ShapeType.Vertex, global::Kerf.Topology.ShapeType.Edge, mapVE);

            var processedEdges = new HashSet<Shape>(ShapeHasher.Instance);
            var exp = new Explorer();

            var actEdges = new NCollection_List<Shape>();
            var neighEdges = new NCollection_List<Shape>();

            if (_edge.IsNull)
            {
                // take the first edge from the list and add it
                Edge fe = ShapeCasts.Edge(newEList.First());
                Add(fe, true); // Kerf: OCCT's Add(fe), in this change's overlay
                processedEdges.Add(fe);
                actEdges.Append(fe);
                lSize++;
            }
            else
            {
                // existing edges are already connected
                for (exp.Init(_shape, global::Kerf.Topology.ShapeType.Edge); exp.More(); exp.Next())
                {
                    Shape cSh = exp.Current();
                    processedEdges.Add(cSh);
                    actEdges.Append(cSh);
                }
            }

            var itL1 = new NCollection_List<Shape>.Iterator();
            var itL2 = new NCollection_List<Shape>.Iterator();

            for (itL1.Initialize(newEList); itL1.More(); itL1.Next())
            {
                Exploration.MapShapesAndAncestors(itL1.Value(), global::Kerf.Topology.ShapeType.Vertex, global::Kerf.Topology.ShapeType.Edge, mapVE);
            }

            while (!actEdges.IsEmpty())
            {
                itL2.Initialize(actEdges);
                for (; itL2.More(); itL2.Next())
                {
                    Shape ce = itL2.Value();
                    exp.Init(ce, global::Kerf.Topology.ShapeType.Vertex);
                    for (; exp.More(); exp.Next())
                    {
                        Shape cv = exp.Current();
                        for (itL1.Initialize(mapVE.FindFromKey(cv)); itL1.More(); itL1.Next())
                        {
                            Shape ne = itL1.Value(); // neighbor edge
                            if (!processedEdges.Contains(ne))
                            {
                                Add(ShapeCasts.Edge(ne), false);
                                neighEdges.Append(ne);
                                processedEdges.Add(ne);
                                lSize++;
                            }
                        }
                    }
                }
                actEdges.Clear();
                actEdges.Append(neighEdges);
            }
        }
    }

    private sealed class BRepLib_BndBoxVertexSelector : SpatialTree<int, Box>.Selector
    {
        private readonly NCollection_IndexedMap<Shape> _mapOfShape; // vertices
        private Point3D _p;
        private double _tolP;
        private int _vInd;
        private Box _vBox = new();
        private readonly NCollection_List<int> _resultInd = new();

        public BRepLib_BndBoxVertexSelector(NCollection_IndexedMap<Shape> mapOfShape)
        {
            _mapOfShape = mapOfShape;
            _tolP = 0.0;
            _vInd = 0;
        }

        public override bool Reject(in Box box) => box.IsOut(_vBox);

        public override bool Accept(in int obj)
        {
            if (obj > _mapOfShape.Extent())
            {
                return false;
            }

            Vertex v = ShapeCasts.Vertex(_mapOfShape[obj]);

            if (obj == _vInd)
            {
                return false;
            }

            Point3D vPnt = Tool.Pnt(v);

            double tolV = Tool.Tolerance(v);

            double l = _p.SquareDistance(vPnt);
            double sTol = tolV + _tolP;
            sTol *= sTol;

            if (l <= sTol)
            {
                _resultInd.Append(obj);
                return true;
            }

            return false;
        }

        public void SetCurrentVertex(in Point3D p, double tol, int vInd)
        {
            _p = p;
            // OCCT never resets myVBox, so the box grows with every vertex (occt-issues.md): more boxes pass Reject,
            // but Accept checks the distance, so only the time changes.
            _vBox.Add(_p);
            _vBox.Enlarge(tol);
            _tolP = tol;
            _vInd = vInd;
        }

        public NCollection_List<int> GetResultInds() => _resultInd;

        public void ClearResInds() => _resultInd.Clear();
    }

    private void CollectCoincidentVertices(NCollection_List<Shape> l,
                                           NCollection_List<NCollection_List<Vertex>> grVL)
    {
        var allV = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);

        Exploration.MapShapes(_shape, global::Kerf.Topology.ShapeType.Vertex, allV);

        var itL = new NCollection_List<Shape>.Iterator(l);
        for (; itL.More(); itL.Next())
        {
            Exploration.MapShapes(itL.Value(), global::Kerf.Topology.ShapeType.Vertex, allV);
        }

        // aV2CV : vertex <-> its coincident vertices. OCCT's maps here use std::hash and operator== (IsEqual).
        var v2CV = new NCollection_DataMap<Vertex, HashSet<Vertex>>(EqualityComparer<Vertex>.Default);
        var tree = new SpatialTree<int, Box>();
        var treeFiller = new SpatialTreeFiller<int, Box>(tree);
        var nonGroupedV = new HashSet<Vertex>(EqualityComparer<Vertex>.Default);

        // add vertices from anAllV to treefiller
        for (int i = 1; i <= allV.Extent(); i++)
        {
            Shape sh = allV[i];
            var bb = new Box();
            Bounds.Add(sh, ref bb);
            treeFiller.Add(i, bb);
        }

        treeFiller.Fill();
        var selector = new BRepLib_BndBoxVertexSelector(allV);

        int nbColl = 0;
        var itI = new NCollection_List<int>.Iterator();
        for (int i = 1; i <= allV.Extent(); i++)
        {
            Vertex v = ShapeCasts.Vertex(allV[i]);
            if (_vertices.Contains(v))
            {
                continue;
            }
            selector.SetCurrentVertex(Tool.Pnt(v), Tool.Tolerance(v), i);
            nbColl = tree.Select(selector);
            if (nbColl > 0)
            {
                NCollection_List<int> resInds = selector.GetResultInds();
                HashSet<Vertex> vm = v2CV.Bound(v, new HashSet<Vertex>(EqualityComparer<Vertex>.Default));
                for (itI.Initialize(resInds); itI.More(); itI.Next())
                {
                    Vertex cv = ShapeCasts.Vertex(allV[itI.Value()]);
                    vm.Add(cv);
                    if (_vertices.Contains(cv))
                    {
                        if (v2CV.IsBound(cv))
                        {
                            v2CV[cv].Add(v);
                        }
                        else
                        {
                            v2CV.Bound(cv, new HashSet<Vertex>(EqualityComparer<Vertex>.Default)).Add(v);
                            nonGroupedV.Add(cv);
                        }
                    }
                }
                if (vm.Count != 0)
                {
                    nonGroupedV.Add(v); // vertexes to be grouped; store only coincident vertices
                }
            }
            selector.ClearResInds();
        }

        // group the coincident vertices
        var startV = new NCollection_List<Vertex>();
        var currentV = new NCollection_List<Vertex>();
        var oneGrV = new NCollection_List<Vertex>();
        var itLV = new NCollection_List<Vertex>.Iterator();
        bool isStartNewGroup = true;
        while (nonGroupedV.Count != 0 || !isStartNewGroup)
        // exit only if there are no nongrouped vertices
        // and the last group are fully are constructed
        {
            if (isStartNewGroup)
            {
                // start list of vertices is empty => append one from aNonGroupedV
                //  and remove it from it (i.e. mark as grouped)
                // OCCT takes the first vertex of its hash map (an order of addresses); the port takes its own map's
                // first (decision 18).
                Vertex curV = new();
                foreach (Vertex v in nonGroupedV)
                {
                    curV = v;
                    break;
                }
                startV.Append(curV);
                nonGroupedV.Remove(curV);
            }
            itLV.Initialize(startV);
            for (; itLV.More(); itLV.Next())
            {
                Vertex sv = itLV.Value();
                oneGrV.Append(sv);
                foreach (Vertex cv in v2CV[sv])
                {
                    if (nonGroupedV.Contains(cv))
                    {
                        currentV.Append(cv);
                        nonGroupedV.Remove(cv);
                    }
                }
            }
            startV.Clear();
            startV.Append(currentV);
            isStartNewGroup = startV.IsEmpty();
            if (isStartNewGroup && !oneGrV.IsEmpty())
            {
                // OCCT's Append copies the list here (the item overload), then clears anOneGrV.
                grVL.Append(new NCollection_List<Vertex>(oneGrV));
                oneGrV.Clear();
            }
        }
    }

    private void CreateNewVertices(NCollection_List<NCollection_List<Vertex>> grVL,
                                   NCollection_DataMap<Shape, Shape> o2NV)
    {
        // map [old vertex => new vertex]
        // note that already existing shape (i.e. the original ones)
        // shouldn't be modified on the topological level
        var itLLV = new NCollection_List<NCollection_List<Vertex>>.Iterator();
        var itLV = new NCollection_List<Vertex>.Iterator();
        var bb = new BrepBuilder();
        itLLV.Initialize(grVL);
        for (; itLLV.More(); itLLV.Next())
        {
            Vertex newV = new();
            var valList = new NCollection_List<Shape>();
            NCollection_List<Vertex> val = itLLV.Value();
            itLV.Initialize(val);
            double newTol = 0;
            Point3D newC = new();
            for (; itLV.More(); itLV.Next())
            {
                Vertex vv = itLV.Value();
                valList.Append(vv);
                if (_vertices.Contains(vv))
                {
                    newV = vv;
                }
            }
            BrepLibrary.BoundingVertex(valList, ref newC, ref newTol);

            if (newV.IsNull)
            {
                // vertices from the original shape isn't found in this group
                // create the new vertex
                newV = new LibraryMakeVertex(newC).InternalVertex();
                bb.UpdateVertex(newV, newTol);
            }
            else
            {
                // update already existing vertex
                bb.UpdateVertex(newV, new Point3D(newC.XYZ()), newTol);
            }

            // fill the map of old->new vertices
            itLV.Initialize(val);
            for (; itLV.More(); itLV.Next())
            {
                Vertex vv = itLV.Value();
                o2NV.Bind(vv, newV);
            }
        }
    }

    private static void CreateNewListOfEdges(NCollection_List<Shape> l,
                                             NCollection_DataMap<Shape, Shape> o2NV,
                                             NCollection_List<Shape> newEList)
    {
        // create the new list (theNewEList) from the input list L
        bool isNewEdge;
        var vList = new NCollection_List<Vertex>();
        var exp = new Explorer();
        var bb = new BrepBuilder();
        var itL = new NCollection_List<Shape>.Iterator();
        for (itL.Initialize(l); itL.More(); itL.Next())
        {
            isNewEdge = false;
            vList.Clear();
            Edge ce = ShapeCasts.Edge(itL.Value());
            exp.Init(ce, global::Kerf.Topology.ShapeType.Vertex);
            for (; exp.More(); exp.Next())
            {
                Vertex val = ShapeCasts.Vertex(exp.Current());
                if (o2NV.IsBound(val))
                {
                    isNewEdge = true;
                    // append the new vertex
                    vList.Append(ShapeCasts.Vertex(o2NV[val].Oriented(val.Orientation())));
                }
                else
                {
                    vList.Append(val);
                }
            }
            if (isNewEdge)
            {
                Shape newE = ce.EmptyCopied();
                var it = new NCollection_List<Vertex>.Iterator(vList);
                for (; it.More(); it.Next())
                {
                    bb.Add(newE, it.Value());
                }
                newEList.Append(ShapeCasts.Edge(newE));
            }
            else
            {
                newEList.Append(ce);
            }
        }
    }

    /// <summary>Returns the construction status.</summary>
    public LibraryWireError Error()
    {
        return _error;
    }

    /// <summary>The wire (frozen).</summary>
    public Wire Wire()
    {
        return ShapeCasts.Wire(Shape());
    }

    /// <summary><see cref="Wire"/> without freezing, for ported code.</summary>
    internal Wire InternalWire()
    {
        return ShapeCasts.Wire(InternalShape());
    }

    /// <summary>The wire (OCCT's conversion operator; frozen).</summary>
    public static explicit operator Wire(LibraryMakeWire maker) => maker.Wire();

    /// <summary>The last edge added (frozen with the wire): a copy of the given edge when it was connected by proximity.</summary>
    public Edge Edge() => ShapeCasts.Edge(Published(_edge));

    /// <summary><see cref="Edge"/> without freezing, for ported code.</summary>
    internal Edge InternalEdge() => _edge;

    /// <summary>The last connecting vertex (frozen with the wire).</summary>
    public Vertex Vertex() => ShapeCasts.Vertex(Published(_vertex));

    /// <summary><see cref="Vertex"/> without freezing, for ported code.</summary>
    internal Vertex InternalVertex() => _vertex;

    /// <summary>
    /// Kerf: the history of the changes that wrote into the caller's edges and vertices or into a published wire (unit
    /// 10H3): each one that changed, modified into the new TShape standing for it in the wire. Null if no change ran in
    /// a working overlay, in which case the wire holds the caller's edges themselves (or their copies made by OCCT's
    /// algorithm).
    /// </summary>
    internal History? InternalHistory() => _history;
}
