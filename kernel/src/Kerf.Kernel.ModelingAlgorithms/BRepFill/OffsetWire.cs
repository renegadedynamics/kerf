// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_OffsetWire.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_OffsetWire.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Properties;
using Kerf.Engine.Offsets.Bisectors;
using Kerf.Engine.Offsets.MedialAxis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;
using static Kerf.Engine.Offsets.MedialAxis.MedialSide;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// Constructs a planar parallel (offset) of the wires of a planar face, through the medial axis of the face: the
/// parallels of its edges and arcs around its vertices, trimmed where they meet on the bisectors.
/// </summary>
/// <remarks>
/// <para>A working object (decision 21), as OCCT's. <see cref="Shape"/>, <see cref="Spine"/> and
/// <see cref="GeneratedShapes"/> are publication points: they freeze what they return (and the whole result with a
/// generated shape). <see cref="InternalShape"/>, <see cref="InternalSpine"/> and <see cref="InternalGeneratedShapes"/>
/// don't, for ported callers that go on building on the result (BRepOffsetAPI_MakeOffset, BRepFill_Evolved).</para>
/// <para>The algorithm never writes into the face it is given: it works on BRepMAT2d_Explorer's new face and on a work
/// face of cut edges (copies), and builds the parallels from new edges and vertices. A zero offset gives the spine's
/// own wires back, moved by the altitude.</para>
/// </remarks>
internal sealed class OffsetWire
{
    private Face _spine = new();
    private Face _workSpine = new();
    private double _offset;
    private bool _isOpenResult;
    private Shape _shape = new();
    private bool _isDone;
    private global::Kerf.Geometry.JoinKind _joinType;
    private readonly IndexedMap<Shape, NCollection_List<Shape>> _map = new(EqualityComparer<Shape>.Default);
    private readonly BisectingLocus _bilo = new();
    private readonly LinkTopoBilo _link = new();
    private readonly Dictionary<Shape, Shape> _mapSpine = new(ShapeHasher.Instance);
    // OCCT leaves myCallGen uninitialised until Perform; PerformWithBiLo alone reads it in GeneratedShapes, where an
    // empty myMapSpine makes its value irrelevant (occt-issues.md, 5O2)
    private bool _callGen;

    private static void EdgeVertices(in Edge e, out Vertex v1, out Vertex v2)
    {
        if (e.Orientation == Reversed)
        {
            Exploration.Vertices(e, out v2, out v1);
        }
        else
        {
            Exploration.Vertices(e, out v1, out v2);
        }
    }

    // A face bounded by one closed circle (or any single edge for an open result): its parallel is computed directly.
    private static bool KPartCircle(in Face mySpine,
                                    double myOffset,
                                    bool isOpenResult,
                                    double alt,
                                    ref Shape myShape,
                                    IndexedMap<Shape, NCollection_List<Shape>> myMap,
                                    ref bool myIsDone)
    {
        Edge e = new();
        for (Explorer edgeIter = new(mySpine, ShapeType.Edge); edgeIter.More(); edgeIter.Next())
        {
            if (!e.IsNull)
            {
                return false;
            }
            e = ShapeCasts.Edge(edgeIter.Current());
        }
        if (e.IsNull)
        {
            return false;
        }

        Curve3D? c = Tool.Curve(e, out Placement L, out double f, out double l);
        if (c is null)
        {
            return false;
        }

        if (c is TrimmedCurve3D ct)
        {
            c = ct.BasisCurve;
        }

        if ((c is CircleCurve3D && Tool.IsClosed(e)) || // closed circle
            isOpenResult)
        {
            double offset = myOffset;

            Curve2D pCurve = Tool.CurveOnSurface(e, mySpine, out f, out l)!;
            CurveAdaptor2D ahc = new(pCurve, f, l);
            Curve2D oc;
            if (ahc.GetType() == Line)
            {
                if (e.Orientation == Forward)
                {
                    offset *= -1;
                }
                OffsetCurveAdaptor2D off = new(ahc, offset);
                oc = new LineCurve2D(off.Line());
            }
            else if (ahc.GetType() == Circle)
            {
                if (e.Orientation == Forward)
                {
                    offset *= -1;
                }
                if (!Tool.IsClosed(e))
                {
                    offset *= -1;
                }
                Circle2D theCirc = ahc.Circle();
                if (offset > 0.0 || Math.Abs(offset) < theCirc.Radius)
                {
                    oc = new CircleCurve2D(theCirc.Position, theCirc.Radius + offset);
                }
                else
                {
                    myIsDone = false;
                    return false;
                }
            }
            else
            {
                if (e.Orientation == Forward)
                {
                    offset *= -1;
                }
                TrimmedCurve2D g2dT = new(pCurve, f, l);
                oc = new OffsetCurve2D(g2dT, offset);
            }
            Surface surf = Tool.Surface(mySpine)!;
            PlaneSurface? plane = surf as PlaneSurface;
            myShape = new LibraryMakeEdge(oc, plane!, f, l).InternalEdge();
            BrepLibrary.BuildCurve3d(ShapeCasts.Edge(myShape));

            myShape.Orient(e.Orientation);
            myShape.Location(L);
            if (Math.Abs(alt) > Constants.Resolution())
            {
                FaceSurfaceAdaptor s = new(mySpine, false);
                Axis3D nor = s.Plane().Axis;
                Transform3D t = new();
                Vector3D trans = new(nor.Direction);
                trans = alt * trans;
                t = t.WithTranslation(trans);
                myShape.Move(new Placement(t));
            }

            NCollection_List<Shape> ll = new();
            ll.Append(myShape);
            myMap.Add(e, ll);

            Edge myEdge = ShapeCasts.Edge(myShape);
            myShape = new LibraryMakeWire(myEdge).InternalWire();

            myIsDone = true;
            return true;
        }

        return false;
    }

    /// <summary>An empty algorithm.</summary>
    public OffsetWire()
    {
        _isOpenResult = false;
        _isDone = false;
    }

    /// <summary>
    /// Prepares the offset of the wires of <paramref name="spine"/> (a planar face) with the join type
    /// <paramref name="Join"/>; <paramref name="isOpenResult"/> asks for open parallels of an open wire.
    /// </summary>
    public OffsetWire(in Face spine, global::Kerf.Geometry.JoinKind Join = Arc, bool isOpenResult = false)
    {
        Init(spine, Join, isOpenResult);
    }

    /// <summary>
    /// Prepares the offset of the wires of <paramref name="spine"/>: cuts its edges for the bisectors and computes its
    /// medial axis on the left.
    /// </summary>
    public void Init(in Face spine, global::Kerf.Geometry.JoinKind Join = Arc, bool isOpenResult = false)
    {
        _isDone = false;
        Shape localShape = spine.Oriented(Forward);
        _spine = ShapeCasts.Face(localShape);
        _joinType = Join;
        _isOpenResult = isOpenResult;

        _map.Clear();
        _mapSpine.Clear();

        //------------------------------------------------------------------
        // cut the spine for bissectors.
        //------------------------------------------------------------------
        MedialAxisExplorer exp = new();
        exp.Perform(_spine);
        _spine = ShapeCasts.Face(exp.ModifiedShape(_spine));
        PrepareSpine();

        Shape shape = new();
        IndexedMap<Shape, NCollection_List<Shape>> map = new(EqualityComparer<Shape>.Default);
        bool done = false;
        if (KPartCircle(_workSpine, 1.0, _isOpenResult, 0.0, ref shape, map, ref done))
        {
            return;
        }

        //-----------------------------------------------------
        // Calculate the map of bissectors to the left.
        // and Links Topology -> base elements of the map.
        //-----------------------------------------------------

        exp.Perform(_workSpine);
        _bilo.Compute(exp, 1, MedialSide.Left, _joinType, _isOpenResult);
        _link.Perform(exp, _bilo);
    }

    /// <summary>True if the last computation succeeded.</summary>
    public bool IsDone => _isDone;

    /// <summary>The face whose wires are offset (BRepMAT2d_Explorer's face; frozen).</summary>
    public Face Spine()
    {
        _spine.TShape()?.FreezeGraph();
        return _spine;
    }

    /// <summary><see cref="Spine"/> without freezing, for ported callers.</summary>
    internal Face InternalSpine() => _spine;

    /// <summary>The parallel: a wire, or a compound of wires (frozen).</summary>
    public Shape Shape()
    {
        _shape.TShape()?.FreezeGraph();
        return _shape;
    }

    /// <summary><see cref="Shape"/> without freezing, for ported callers.</summary>
    internal Shape InternalShape() => _shape;

    /// <summary>
    /// The shapes of the parallel generated by <paramref name="spineShape"/>, an edge or a vertex of the spine (frozen,
    /// with the whole result).
    /// </summary>
    public IReadOnlyList<Shape> GeneratedShapes(in Shape spineShape)
    {
        NCollection_List<Shape> list = InternalGeneratedShapes(spineShape);
        _shape.TShape()?.FreezeGraph();
        if (list.IsEmpty())
        {
            return Array.Empty<Shape>();
        }
        var shapes = new Shape[list.Extent()];
        int i = 0;
        foreach (Shape s in list)
        {
            s.TShape()?.FreezeGraph();
            shapes[i++] = s;
        }
        return ImmutableArrays.Own(shapes);
    }

    /// <summary>
    /// <see cref="GeneratedShapes"/> without freezing: the algorithm's own list (an empty new list if there is none).
    /// </summary>
    internal NCollection_List<Shape> InternalGeneratedShapes(in Shape spineShape)
    {
        if (!_callGen)
        {
            if (_mapSpine.Count != 0)
            {
                // myMapSpine can be empty if passed by PerformWithBilo.
                // OCCT iterates the map in address order; the port in insertion order (occt-issues.md, 5O2)
                foreach (KeyValuePair<Shape, Shape> it in _mapSpine)
                {
                    if (_map.ContainsKey(it.Key))
                    {
                        if (!_map.ContainsKey(it.Value))
                        {
                            NCollection_List<Shape> l = new();
                            _map.Add(it.Value, l);
                        }
                        if (!it.Value.IsSame(it.Key))
                        {
                            _map.GetValue(it.Value).Append(_map.GetValue(it.Key));
                            _map.Remove(it.Key);
                        }
                    }
                    if (_map.ContainsKey(it.Key.Reversed()))
                    {
                        if (!_map.ContainsKey(it.Value.Reversed()))
                        {
                            NCollection_List<Shape> l = new();
                            _map.Add(it.Value.Reversed(), l);
                        }
                        if (!it.Value.IsSame(it.Key))
                        {
                            _map.GetValue(it.Value.Reversed())
                                 .Append(_map.GetValue(it.Key.Reversed()));
                            _map.Remove(it.Key.Reversed());
                        }
                    }
                }
            }
            _callGen = true;
        }

        if (_map.ContainsKey(spineShape))
        {
            return _map.GetValue(spineShape);
        }
        else
        {
            // OCCT returns a static empty list
            return new NCollection_List<Shape>();
        }
    }

    /// <summary>The join type.</summary>
    public global::Kerf.Geometry.JoinKind JoinType() => _joinType;

    /// <summary>
    /// Computes the parallel at the distance <paramref name="offset"/>, at the altitude <paramref name="alt"/> along
    /// the normal of the spine's plane.
    /// </summary>
    /// <exception cref="ConstructionErrorException">A closed result is expected and a parallel wire isn't closed.</exception>
    public void Perform(double offset, double alt = 0.0)
    {
        try
        {
            _callGen = false;
            if (KPartCircle(_workSpine, offset, _isOpenResult, alt, ref _shape, _map, ref _isDone))
            {
                return;
            }

            Face oldWorkSpain = _workSpine;

            NCollection_List<Shape> badEdges = new();
            CheckBadEdges(_workSpine, offset, _bilo, _link, badEdges);

            if (!badEdges.IsEmpty())
            {
                // Modification of myWorkSpine;
                Substitution subst = new();
                NCollection_List<Shape>.Iterator it = new(badEdges);
                NCollection_List<Shape> aL = new();
                double defl = .01 * Math.Abs(offset);
                NCollection_Sequence<double> parameters = new();
                NCollection_Sequence<Point3D> points = new();

                for (; it.More(); it.Next())
                {
                    aL.Clear();
                    parameters.Clear();
                    points.Clear();
                    Shape e = it.Value();

                    Exploration.Vertices(ShapeCasts.Edge(e), out Vertex vf, out Vertex vl);

                    Curve3D? g3d = Tool.Curve(ShapeCasts.Edge(e), out double f, out double l);
                    CurveAdaptor ac = new(g3d!, f, l);

                    PerformCurve(parameters, points, ac, defl, f, l, global::Kerf.Engine.Foundation.Precision.Confusion(), 2);

                    int nPnts = points.Length();
                    if (nPnts > 2)
                    {
                        Vertex fv = vf;
                        Vertex lv;
                        Edge newE;
                        int np;
                        for (np = 2; np < nPnts; np++)
                        {
                            Point3D lp = points[np];
                            lv = new LibraryMakeVertex(lp).InternalVertex();
                            newE = new LibraryMakeEdge(fv, lv).InternalEdge();
                            aL.Append(newE);
                            fv = lv;
                        }
                        lv = vl;
                        newE = new LibraryMakeEdge(fv, lv).InternalEdge();
                        aL.Append(newE);
                    }
                    else
                    {
                        Edge newE = new LibraryMakeEdge(vf, vl).InternalEdge();
                        aL.Append(newE);
                    }
                    // Update myMapSpine
                    if (_mapSpine.ContainsKey(e))
                    {
                        NCollection_List<Shape>.Iterator newit = new(aL);
                        for (; newit.More(); newit.Next())
                        {
                            Edge newEdge = ShapeCasts.Edge(newit.Value());
                            _mapSpine[newEdge] = _mapSpine.GetValue(e);
                            EdgeVertices(newEdge, out Vertex newV1, out Vertex newV2);
                            if (!_mapSpine.ContainsKey(newV1))
                            {
                                _mapSpine[newV1] = _mapSpine.GetValue(e);
                            }
                            if (!_mapSpine.ContainsKey(newV2))
                            {
                                _mapSpine[newV2] = _mapSpine.GetValue(e);
                            }
                        }
                        _mapSpine.Remove(e);
                    }
                    ///////////////////
                    subst.Substitute(e, aL);
                }

                Dictionary<Shape, NCollection_List<Shape>> wwmap = new(ShapeHasher.Instance);
                Iterator itws = new(_workSpine);
                for (; itws.More(); itws.Next())
                {
                    Shape wire = itws.Value();
                    subst.Build(wire);
                    if (subst.IsCopied(wire))
                    {
                        Wire newWire = ShapeCasts.Wire(subst.InternalCopy(wire).First());
                        newWire.Closed(wire.ClosedFlag);
                        NCollection_List<Shape> lw = new();
                        lw.Append(newWire);
                        wwmap[wire] = lw;
                    }
                }
                subst.Clear();
                foreach (KeyValuePair<Shape, NCollection_List<Shape>> itmap in wwmap)
                {
                    subst.Substitute(itmap.Key, itmap.Value);
                }

                subst.Build(_workSpine);

                if (subst.IsCopied(_workSpine))
                {
                    _workSpine = ShapeCasts.Face(subst.InternalCopy(_workSpine).First());

                    MedialAxisExplorer newExp = new();
                    newExp.Perform(_workSpine);
                    BisectingLocus newBilo = new();
                    LinkTopoBilo newLink = new();
                    newBilo.Compute(newExp, 1, MedialSide.Left, _joinType, _isOpenResult);

                    if (!newBilo.IsDone)
                    {
                        _shape.Nullify();
                        _isDone = false;
                        return;
                    }

                    newLink.Perform(newExp, newBilo);
                    PerformWithBiLo(_workSpine, offset, newBilo, newLink, _joinType, alt);
                    _workSpine = oldWorkSpain;
                }
                else
                {
                    PerformWithBiLo(_workSpine, offset, _bilo, _link, _joinType, alt);
                }
            }
            else
            {
                PerformWithBiLo(_workSpine, offset, _bilo, _link, _joinType, alt);
            }
        }
        catch (Exception exception) when ((Refusal.IsFailure(exception) || exception is NullReferenceException)
                                            && exception is not ImmutableObjectException)
        {
            // OCC_CATCH_SIGNALS turns OCCT's access violations (a null bisector, occt-issues.md) into failures caught
            // here: the port's NullReferenceException (decision 917)
            _shape.Nullify();
            _isDone = false;

            return;
        }

        if (!_isOpenResult)
        {
            Explorer exp = new(_shape, ShapeType.Wire);

            for (; exp.More(); exp.Next())
            {
                Shape wire = exp.Current();

                if (!wire.ClosedFlag)
                {
                    _shape.Nullify();
                    _isDone = false;
                    throw new ConstructionErrorException("Offset wire is not closed.");
                }
            }
        }
    }

    // The parallel at distance 0: the wires of the spine, moved by the altitude.
    private static void Compute(in Face spine,
                                ref Shape aShape,
                                IndexedMap<Shape, NCollection_List<Shape>> map,
                                double alt)
    {
        BrepBuilder b = new();
        Compound compound = new();
        b.MakeCompound(ref compound);
        aShape = compound;
        double ALT = alt;
        if (spine.Orientation == Reversed)
        {
            ALT = -alt;
        }
        Transform3D t = new Transform3D().WithTranslation(new Vector3D(0.0, 0.0, ALT));
        Placement l = new(t);

        for (Explorer exp = new(spine, ShapeType.Wire); exp.More(); exp.Next())
        {
            Wire curW = ShapeCasts.Wire(exp.Current());
            Shape localShape = curW.Moved(l);
            Wire newW = ShapeCasts.Wire(localShape);
            b.Add(aShape, newW);
            // update Map.
            Iterator it1 = new(curW);
            Iterator it2 = new(newW);
            for (; it1.More(); it1.Next(), it2.Next())
            {
                NCollection_List<Shape> list = new();
                list.Append(it2.Value());
                map.Add(it1.Value(), list);
            }
        }
    }

    /// <summary>
    /// Computes the parallel at the distance <paramref name="offset"/> of the face <paramref name="spine"/> from its
    /// bisecting locus <paramref name="locus"/> and links <paramref name="link"/>, already computed.
    /// </summary>
    public void PerformWithBiLo(in Face spine,
                                double offset,
                                BisectingLocus locus,
                                LinkTopoBilo link,
                                global::Kerf.Geometry.JoinKind Join = Arc,
                                double alt = 0.0)
    {
        _isDone = false;
        Shape localShapeWork = spine.Oriented(Forward);
        _workSpine = ShapeCasts.Face(localShapeWork);
        _joinType = Join;
        _offset = offset;
        _shape.Nullify();

        if (_spine.IsNull)
        {
            Shape localShape = spine.Oriented(Forward);
            _spine = ShapeCasts.Face(localShape);
        }
        _map.Clear();

        if (Math.Abs(_offset) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            Compute(_spine, ref _shape, _map, alt);
            _isDone = true;
            return;
        }

        //********************************
        // Calculate for a non null offset
        //********************************
        if (KPartCircle(_workSpine, offset, _isOpenResult, alt, ref _shape, _map, ref _isDone))
        {
            return;
        }

        BrepBuilder myBuilder = new();

        //---------------------------------------------------------------------
        // MapNodeVertex : associate to each node of the map (key1) and to
        //                 each element of the profile (key2) a vertex (item).
        // MapBis        : all edges or vertices (item) generated by
        //                 a bisectrice on a face or an edge (key) of revolution tubes.
        // MapVerPar     : Map of parameters of vertices on parallel edges
        //                 the list contained in MapVerPar (E) corresponds to
        //                 parameters on E of vertices contained in MapBis(E);
        //---------------------------------------------------------------------

        // OCCT.NET: keyed by the node's identity, as OCCT's handle; only looked up
        Dictionary<MedialNode, Shape> mapNodeVertex = new(ReferenceEqualityComparer.Instance);
        Dictionary<Shape, NCollection_Sequence<Shape>> mapBis = new(ShapeHasher.Instance);
        Dictionary<Shape, NCollection_Sequence<double>> mapVerPar = new(ShapeHasher.Instance);

        // OCCT.NET: OCCT down-casts (a null plane on another surface, dereferenced below: an access violation natively)
        PlaneSurface refPlane = (Tool.Surface(_workSpine) as PlaneSurface)!;
        if (Math.Abs(alt) > Constants.Resolution())
        {
            double anAlt = alt;
            if (_workSpine.Orientation == Reversed)
            {
                anAlt = -alt;
            }
            refPlane = (PlaneSurface)refPlane.Translated(anAlt * new Vector3D(refPlane.Axis.Direction));
        }

        //---------------------------------------------------------------
        // Construction of Circles and OffsetCurves
        //---------------------------------------------------------------

        Vertex[] ends = [new(), new()];
        if (_isOpenResult)
        {
            Wire theWire;
            Iterator iter = new(_spine);
            theWire = ShapeCasts.Wire(iter.Value());
            Exploration.Vertices(theWire, out ends[0], out ends[1]);
        }

        if (locus.NumberOfContours() == 0)
        {
            return;
        }

        for (int ic = 1; ic <= locus.NumberOfContours(); ic++)
        {
            Shape pe = link.GeneratingShape(locus.BasicElt(ic, locus.NumberOfElts(ic)));
            for (int ie = 1; ie <= locus.NumberOfElts(ic); ie++)
            {
                Shape se = link.GeneratingShape(locus.BasicElt(ic, ie));
                if (se.Type == ShapeType.Vertex)
                {
                    if (!se.IsSame(ends[0]) && !se.IsSame(ends[1]))
                    {
                        MakeCircle(ShapeCasts.Edge(pe), ShapeCasts.Vertex(se), _workSpine, _offset, _map, refPlane);
                    }
                }
                else
                {
                    MakeOffset(ShapeCasts.Edge(se),
                               _workSpine,
                               _offset,
                               _map,
                               refPlane,
                               _isOpenResult,
                               _joinType,
                               ends);
                    pe = se;
                }
            }
        }

        // Remove possible hanging arcs on vertices
        if (_isOpenResult && _joinType == Arc)
        {
            if (_map.Count != 0 && _map.KeyAt(0).Type == ShapeType.Vertex)
            {
                _map.RemoveAtSwap(0);
            }
            if (_map.Count != 0 && _map.KeyAt(_map.Count - 1).Type == ShapeType.Vertex)
            {
                _map.RemoveLast();
            }
        }

        //---------------------------------------------------
        // Construction of offset vertices.
        //---------------------------------------------------
        Dictionary<Shape, NCollection_List<Shape>> detromp = new(EqualityComparer<Shape>.Default);
        MedialArc currentArc;
        Edge currentEdge;
        Shape[] S = [new(), new()];
        Edge[] e = [new(), new()];
        int j, k;

        for (int i = 1; i <= locus.Graph().NumberOfArcs(); i++)
        {
            currentArc = locus.Graph().Arc(i);
            Bisec bisec = locus.GeomBis(currentArc, out bool reverse);

            //-------------------------------------------------------------------
            // Return elements of the spine corresponding to separate basicElts.
            //-------------------------------------------------------------------
            S[0] = link.GeneratingShape(currentArc.FirstElement());
            S[1] = link.GeneratingShape(currentArc.SecondElement());

            NCollection_Sequence<Shape> vertices = new();
            NCollection_Sequence<Point3D> Params = new();

            //-----------------------------------------------------------
            // Return parallel edges on each face.
            // If no offset generated => move to the next bissectrice.
            //--------------------------------------------------------------
            if (_map.ContainsKey(S[0]) && _map.ContainsKey(S[1]))
            {
                e[0] = ShapeCasts.Edge(_map.GetValue(S[0]).First());
                e[1] = ShapeCasts.Edge(_map.GetValue(S[1]).First());
            }
            else
            {
                continue;
            }

            //-----------------------------------------------------------
            // Construction of vertices corresponding to the node of the map.
            // if they are on the offset.
            //-----------------------------------------------------------
            Vertex vs = new(), ve = new();
            MedialNode node1, node2;

            if (reverse)
            {
                node1 = currentArc.SecondNode();
                node2 = currentArc.FirstNode();
            }
            else
            {
                node1 = currentArc.FirstNode();
                node2 = currentArc.SecondNode();
            }

            bool startOnEdge = false, endOnEdge = false;

            if (!node1.Infinite())
            {
                Point2D localPnt2d = locus.GeomElt(node1);
                startOnEdge = VertexFromNode(node1, _offset, localPnt2d, mapNodeVertex, ref vs);
            }
            if (!node2.Infinite())
            {
                Point2D localPnt2d = locus.GeomElt(node2);
                endOnEdge = VertexFromNode(node2, _offset, localPnt2d, mapNodeVertex, ref ve);
            }

            if (_joinType == global::Kerf.Geometry.JoinKind.Intersection)
            {
                startOnEdge = endOnEdge = false;
            }

            //---------------------------------------------
            // Construction of geometries.
            //---------------------------------------------
            TrimEdgeTool trim = new(bisec,
                                             locus.GeomElt(currentArc.FirstElement()),
                                             locus.GeomElt(currentArc.SecondElement()),
                                             _offset);

            //-----------------------------------------------------------
            // Construction of vertices on edges parallel to the spine.
            //-----------------------------------------------------------

            trim.IntersectWith(e[0], e[1], S[0], S[1], ends[0], ends[1], _joinType, _isOpenResult, Params);

            for (int s = 1; s <= Params.Length(); s++)
            {
                Vertex vc = new();
                myBuilder.MakeVertex(ref vc);
                Point2D p2 = bisec.Value().Value(Params.Value(s).X);
                Point3D pvc = new(p2.X, p2.Y, 0.0);

                myBuilder.UpdateVertex(vc, pvc, global::Kerf.Engine.Foundation.Precision.Confusion());
                vertices.Append(vc);
            }
            if (startOnEdge)
            {
                bool start = true;
                trim.AddOrConfuse(start, e[0], e[1], Params);
                if (Params.Length() == vertices.Length() && Params.Length() != 0)
                {
                    vertices.SetValue(1, vs);
                }
                else
                {
                    // the point was not found by IntersectWith
                    vertices.Prepend(vs);
                }
            }
            if (endOnEdge)
            {
                bool start = false;
                trim.AddOrConfuse(start, e[0], e[1], Params);
                if (Params.Length() == vertices.Length() && Params.Length() != 0)
                {
                    vertices.SetValue(Params.Length(), ve);
                }
                else
                {
                    // the point was not found by IntersectWith
                    vertices.Append(ve);
                }
            }

            //------------------------------------------------------------
            // Update Detromp.
            // Detromp allows to remove vertices on the offset
            // corresponding to tangency zones
            // Detromp ranks the vertices that limit
            // the parts of the bissectrices located between the spine and the
            // offset.
            //------------------------------------------------------------
            if (!detromp.ContainsKey(S[0]))
            {
                detromp[S[0]] = new NCollection_List<Shape>();
            }
            if (!detromp.ContainsKey(S[1]))
            {
                detromp[S[1]] = new NCollection_List<Shape>();
            }

            UpdateDetromp(detromp, S[0], S[1], vertices, Params, bisec, startOnEdge, endOnEdge, trim);
            //----------------------------------------------
            // Storage of vertices on parallel edges.
            // fill MapBis and MapVerPar.
            //----------------------------------------------
            if (!vertices.IsEmpty() && Params.Length() == vertices.Length())
            {
                for (k = 0; k <= 1; k++)
                {
                    if (!mapBis.ContainsKey(e[k]))
                    {
                        mapBis[e[k]] = new NCollection_Sequence<Shape>();
                        mapVerPar[e[k]] = new NCollection_Sequence<double>();
                    }
                    for (int ii = 1; ii <= vertices.Length(); ii++)
                    {
                        mapBis.GetValue(e[k]).Append(vertices.Value(ii));
                        if (k == 0)
                        {
                            mapVerPar.GetValue(e[k]).Append(Params.Value(ii).Y);
                        }
                        else
                        {
                            mapVerPar.GetValue(e[k]).Append(Params.Value(ii).Z);
                        }
                    }
                }
            }
            else
            {
                //------------------------------------------------------------
                // FOR COMPLETE CIRCLES. the parallel line can be contained
                // in the zone without intersection with the border
                // no intersection
                // if myoffset is < distance of nodes the parallel can be valid.
                //-------------------------------------------------------------
                for (k = 0; k <= 1; k++)
                {
                    if (!mapBis.ContainsKey(e[k]))
                    {
                        if (node1.Distance() > _offset && node2.Distance() > _offset)
                        {
                            mapBis[e[k]] = new NCollection_Sequence<Shape>();
                            mapVerPar[e[k]] = new NCollection_Sequence<double>();
                        }
                    }
                }
            }
        }

        //----------------------------------
        // Construction of parallel edges.
        //----------------------------------
        IndexedMap<Shape, Shape> mapVV = new(ShapeHasher.Instance);

        Shape currentSpine;

        for (j = 1; j <= _map.Count; j++)
        {
            currentSpine = _map.KeyAt(j - 1);
            currentEdge = ShapeCasts.Edge(_map.ValueAt(j - 1).First());

            _map.ValueAt(j - 1).Clear();
            if (mapBis.ContainsKey(currentEdge))
            {
                NCollection_Sequence<Shape> seqOfShape = new();
                if (!mapBis.GetValue(currentEdge).IsEmpty())
                {
                    int indOfE = 0;
                    if (_isOpenResult)
                    {
                        if (j == 1)
                        {
                            indOfE = 1;
                        }
                        else if (j == _map.Count)
                        {
                            indOfE = -1;
                        }
                    }
                    TrimEdge(currentEdge,
                             currentSpine,
                             _spine,
                             detromp.GetValue(currentSpine),
                             mapBis.GetValue(currentEdge),
                             mapVerPar.GetValue(currentEdge),
                             seqOfShape,
                             mapVV,
                             indOfE);
                    for (k = 1; k <= seqOfShape.Length(); k++)
                    {
                        _map.ValueAt(j - 1).Append(seqOfShape.Value(k));
                    }
                }
                else
                {
                    //-----------------
                    // Complete circles
                    //-----------------
                    _map.ValueAt(j - 1).Append(currentEdge);
                }
            }
        }

        int ind;
        for (ind = 1; ind <= mapVV.Count; ind++)
        {
            Vertex oldV = ShapeCasts.Vertex(mapVV.KeyAt(ind - 1));
            Vertex newV = ShapeCasts.Vertex(mapVV.ValueAt(ind - 1));
            Point3D p1 = Tool.Pnt(oldV);
            Point3D p2 = Tool.Pnt(newV);
            myBuilder.UpdateVertex(newV, p1.DistanceTo(p2));
            NCollection_List<Shape> lv = new();
            lv.Append(newV.Oriented(Forward));
            Substitution subst = new();
            subst.Substitute(oldV, lv);
            for (j = 1; j <= _map.Count; j++)
            {
                NCollection_List<Shape>.Iterator itl = new(_map.ValueAt(j - 1));
                for (; itl.More(); itl.Next())
                {
                    subst.Build(itl.Value());
                    if (subst.IsCopied(itl.Value()))
                    {
                        NCollection_List<Shape> listSh = subst.InternalCopy(itl.Value());
                        Orientation saveOr = itl.Value().Orientation;
                        itl.ChangeValue() = listSh.First();
                        itl.ChangeValue().Orient(saveOr);
                    }
                }
            }
        }

        //----------------------------------
        // Construction of offset wires.
        //----------------------------------
        MakeWires();

        // Update vertices ( Constructed in the plane Z = 0) !!!
        HashSet<Shape> mapVertex = new(ShapeHasher.Instance);
        for (Explorer exp = new(_shape, ShapeType.Vertex); exp.More(); exp.Next())
        {
            Vertex v = ShapeCasts.Vertex(exp.Current());
            if (mapVertex.Add(v))
            {
                Point3D p = Tool.Pnt(v);
                p = refPlane.Value(p.X, p.Y);
                myBuilder.UpdateVertex(v, p, global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }

        // Construction of curves 3d.
        BrepLibrary.BuildCurves3d(_shape);
        mapVertex.Clear();
        Explorer explo = new(_shape, ShapeType.Edge);
        for (; explo.More(); explo.Next())
        {
            Edge edge = ShapeCasts.Edge(explo.Current());
            Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
            BrepTVertex tv1 = (BrepTVertex)v1.TShape()!;
            BrepTVertex tv2 = (BrepTVertex)v2.TShape()!;

            Curve3D theCurve = Tool.Curve(edge, out Placement loc, out double f, out double l)!;
            theCurve = (Curve3D)theCurve.Copy();
            theCurve.Transform(loc.Transformation);
            Point3D f3d = theCurve.Value(f);
            Point3D l3d = theCurve.Value(l);

            double dist1, dist2;
            dist1 = f3d.DistanceTo(tv1.Pnt());
            dist2 = l3d.DistanceTo(tv2.Pnt());
            if (!mapVertex.Contains(v1))
            {
                tv1.Pnt(f3d);
                mapVertex.Add(v1);
            }
            else
            {
                tv1.UpdateTolerance(1.5 * dist1);
            }
            if (!mapVertex.Contains(v2))
            {
                tv2.Pnt(l3d);
                mapVertex.Add(v2);
            }
            else
            {
                tv2.UpdateTolerance(1.5 * dist2);
            }
        }

        if (!_isOpenResult)
        {
            FixHoles();
        }

        _isDone = true;
    }

    // OCCT's private Generated() (the map of generated shapes) is never called: not ported.
    // OCCT's private Add(const BRepFill_OffsetWire&) is declared but never defined: not ported.

    // The work face: the spine's edges, cut at the extrema of curvature and inflexions (MAT2d_CutCurve).
    private void PrepareSpine()
    {
        BrepBuilder b = new();
        NCollection_List<Shape> cuts = new();
        NCollection_List<Shape>.Iterator iteCuts = new();
        Vertex V1, V2;

        _workSpine.Nullify();
        _mapSpine.Clear();

        Surface? s = Tool.Surface(_spine, out Placement l);
        double tolF = Tool.Tolerance(_spine);
        Face workSpine = new();
        b.MakeFace(ref workSpine, s, l, tolF);
        _workSpine = workSpine;

        for (Iterator iteF = new(_spine); iteF.More(); iteF.Next())
        {
            Wire nw = new();
            b.MakeWire(ref nw);

            int forcedCut = 0;
            int nbResEdges = -1;
            IndexedSet<Shape> edgeMap = new(ShapeHasher.Instance);

            Exploration.MapShapes(iteF.Value(), ShapeType.Edge, edgeMap);
            int nbEdges = edgeMap.Count;

            if (nbEdges == 1 && !_isOpenResult)
            { // in case of open wire there's no need to do it
                forcedCut = 2;
            }

            for (Iterator iteW = new(iteF.Value()); iteW.More(); iteW.Next())
            {
                Edge e = ShapeCasts.Edge(iteW.Value());
                EdgeVertices(e, out V1, out V2);
                _mapSpine[V1] = V1;
                _mapSpine[V2] = V2;
                cuts.Clear();

                // Cut
                Shape localShape = e.Oriented(Forward);
                BrepTEdge tEdge = (BrepTEdge)e.TShape()!;
                int numCurvesInEdge = tEdge.Curves().Extent();
                if (nbEdges == 2 && nbResEdges == 0 && numCurvesInEdge > 1)
                {
                    forcedCut = 1;
                }
                nbResEdges = CutEdge(ShapeCasts.Edge(localShape), _spine, forcedCut, cuts);

                if (cuts.IsEmpty())
                {
                    b.Add(nw, e);
                    _mapSpine[e] = e;
                }
                else
                {
                    for (iteCuts.Initialize(cuts); iteCuts.More(); iteCuts.Next())
                    {
                        Edge ne = ShapeCasts.Edge(iteCuts.Value());
                        ne.Orient(e.Orientation);
                        b.Add(nw, ne);
                        _mapSpine[ne] = e;
                        EdgeVertices(ne, out V1, out V2);
                        if (!_mapSpine.ContainsKey(V1))
                        {
                            _mapSpine[V1] = e;
                        }
                        if (!_mapSpine.ContainsKey(V2))
                        {
                            _mapSpine[V2] = e;
                        }
                    }
                }
            }
            Exploration.Vertices(nw, out Vertex v1, out Vertex v2);

            nw.Closed(v1.IsSame(v2));

            b.Add(_workSpine, nw);
        }
    }

    // For each interval on bissectrice defined by parameters test if the medium point is at a distance > offset; in
    // this case vertices corresponding to the extremities of the interval are ranked in the proofing.
    // => If the same vertex appears in the proofing, the border of the zone of proximity is tangent to the offset.
    private void UpdateDetromp(Dictionary<Shape, NCollection_List<Shape>> detromp,
                               in Shape shape1,
                               in Shape shape2,
                               NCollection_Sequence<Shape> vertices,
                               NCollection_Sequence<Point3D> Params,
                               Bisec bisec,
                               bool sOnE,
                               bool eOnE,
                               TrimEdgeTool trim)
    {
        int ii = 1;

        if (_joinType == global::Kerf.Geometry.JoinKind.Intersection)
        {
            for (; ii <= vertices.Length(); ii++)
            {
                Vertex vertex = ShapeCasts.Vertex(vertices.Value(ii));
                detromp.GetValue(shape1).Append(vertex);
                detromp.GetValue(shape2).Append(vertex);
            }
        }
        else // myJoinType == GeomAbs_Arc
        {
            double u1, u2;
            Vertex v1 = new(), v2;

            Curve2D bis = bisec.Value();
            bool forceAdd = false;
            if (bis is TrimmedCurve2D tc && tc.ChangeBasisCurve().IsPeriodic)
            {
                Point2D pf = bis.Value(bis.FirstParameter);
                Point2D pl = bis.Value(bis.LastParameter);
                forceAdd = pf.DistanceTo(pl) <= global::Kerf.Engine.Foundation.Precision.Confusion();
            }

            u1 = bis.FirstParameter;

            if (sOnE)
            {
                // the first point of the bissectrice is on the offset
                v1 = ShapeCasts.Vertex(vertices.Value(ii));
                ii++;
            }

            while (ii <= vertices.Length() && ii <= Params.Length())
            {
                u2 = Params.Value(ii).X;
                v2 = ShapeCasts.Vertex(vertices.Value(ii));

                Point2D p = bis.Value((u2 + u1) * 0.5);
                if (!trim.IsInside(p) || forceAdd)
                {
                    if (!v1.IsNull)
                    {
                        detromp.GetValue(shape1).Append(v1);
                        detromp.GetValue(shape2).Append(v1);
                    }
                    detromp.GetValue(shape1).Append(v2);
                    detromp.GetValue(shape2).Append(v2);
                }
                u1 = u2;
                v1 = v2;
                ii++;
            }

            // test medium point between the last parameter and the end of the bissectrice.
            u2 = bis.LastParameter;
            if (!eOnE)
            {
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(u2))
                {
                    Point2D p = bis.Value((u2 + u1) * 0.5);
                    if (!trim.IsInside(p) || forceAdd)
                    {
                        if (!v1.IsNull)
                        {
                            detromp.GetValue(shape1).Append(v1);
                            detromp.GetValue(shape2).Append(v1);
                        }
                    }
                }
                else
                {
                    if (!v1.IsNull)
                    {
                        detromp.GetValue(shape1).Append(v1);
                        detromp.GetValue(shape2).Append(v1);
                    }
                }
            }
        } // end of else (myJoinType==GeomAbs_Arc)
    }

    // Chains the trimmed parallel edges into wires.
    private void MakeWires()
    {
        //--------------------------------------------------------
        // creation of a single list of created parallel edges.
        //--------------------------------------------------------
        NCollection_List<Shape> theWires = new();
        NCollection_List<Shape>.Iterator itl = new();
        IndexedMap<Shape, NCollection_List<Shape>> mve = new(ShapeHasher.Instance);
        Vertex v1, v2, vf, cv;
        int i;

        for (i = 1; i <= _map.Count; i++)
        {
            for (itl.Initialize(_map.ValueAt(i - 1)); itl.More(); itl.Next())
            {
                Edge e = ShapeCasts.Edge(itl.Value());
                Exploration.Vertices(e, out v1, out v2);
                if (v1.IsSame(v2) && IsSmallClosedEdge(e, v1))
                {
                    continue; // remove small closed edges
                }
                if (!CheckSmallParamOnEdge(e))
                {
                    continue;
                }
                if (!mve.ContainsKey(v1))
                {
                    NCollection_List<Shape> empty = new();
                    mve.Add(v1, empty);
                }
                mve.GetValue(v1).Append(e);
                if (!mve.ContainsKey(v2))
                {
                    NCollection_List<Shape> empty = new();
                    mve.Add(v2, empty);
                }
                mve.GetValue(v2).Append(e);
            }
        }

        //--------------------------------------
        // Creation of parallel wires.
        //--------------------------------------
        BrepBuilder b = new();

        Wire nw = new();
        bool end;
        Edge ce;

        while (mve.Count != 0)
        {
            b.MakeWire(ref nw);

            for (i = 1; i <= mve.Count; i++)
            {
                if (mve.ValueAt(i - 1).Extent() == 1)
                {
                    break;
                }
            }

            if (i > mve.Count)
            {
                i = 1;
            }

            cv = vf = ShapeCasts.Vertex(mve.KeyAt(i - 1));
            ce = ShapeCasts.Edge(mve.ValueAt(i - 1).First());
            end = false;
            mve.GetValue(cv).RemoveFirst();

            if (_isOpenResult && mve.GetValue(cv).IsEmpty())
            {
                mve.Remove(cv);
            }

            bool isClosed = false;

            while (!end)
            {
                //-------------------------------
                // Construction of a wire.
                //-------------------------------
                Exploration.Vertices(ce, out v1, out v2);
                if (!cv.IsSame(v1))
                {
                    cv = v1;
                }
                else
                {
                    cv = v2;
                }

                b.Add(nw, ce);

                if (vf.IsSame(cv) || !mve.ContainsKey(cv))
                {
                    isClosed = vf.IsSame(cv);
                    end = true;
                    mve.Remove(vf);
                }

                if (!end)
                {
                    for (itl.Initialize(mve.GetValue(cv)); itl.More(); itl.Next())
                    {
                        if (itl.Value().IsSame(ce))
                        {
                            mve.GetValue(cv).Remove(itl);
                            break;
                        }
                    }
                    if (!mve.GetValue(cv).IsEmpty())
                    {
                        ce = ShapeCasts.Edge(mve.GetValue(cv).First());
                        mve.GetValue(cv).RemoveFirst();
                    }
                    else if (_isOpenResult)
                    { // CV was a vertex with one edge
                        end = true;
                    }

                    if (mve.GetValue(cv).IsEmpty())
                    {
                        mve.Remove(cv);
                    }
                }
            }
            nw.Closed(isClosed);
            theWires.Append(nw);
        }

        // update myShape :
        //      -- if only one wire : myShape is a Wire
        //      -- if several wires : myShape is a Compound.
        if (theWires.Extent() == 1)
        {
            _shape = theWires.First();
        }
        else
        {
            Compound r = new();
            b.MakeCompound(ref r);
            NCollection_List<Shape>.Iterator it = new(theWires);
            for (; it.More(); it.Next())
            {
                b.Add(r, it.Value());
            }
            _shape = r;
        }
    }

    // Closes the parallel wires that MakeWires left open, joining close ends.
    private void FixHoles()
    {
        NCollection_Sequence<Shape> closedWires = new(), unclosedWires = new(), isolatedWires = new();

        double maxTol = 0.0;
        BrepBuilder bb = new();

        Explorer explo = new(_spine, ShapeType.Vertex);
        for (; explo.More(); explo.Next())
        {
            Vertex vertex = ShapeCasts.Vertex(explo.Current());
            double tol = Tool.Tolerance(vertex);
            if (tol > maxTol)
            {
                maxTol = tol;
            }
        }
        maxTol *= 100.0;

        explo.Init(_shape, ShapeType.Wire);
        for (; explo.More(); explo.Next())
        {
            Shape wire = explo.Current();
            // Remove duplicated edges
            Dictionary<Shape, NCollection_List<Shape>> eEmap = new(ShapeHasher.Instance);
            Iterator it = new(wire);
            for (; it.More(); it.Next())
            {
                Shape edge = it.Value();
                if (!eEmap.ContainsKey(edge))
                {
                    NCollection_List<Shape> le = new();
                    eEmap[edge] = le;
                }
                else
                {
                    eEmap.GetValue(edge).Append(edge);
                }
            }
            wire.Free(true);
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> mapit in eEmap)
            {
                NCollection_List<Shape> le = mapit.Value;
                NCollection_List<Shape>.Iterator itl = new(le);
                for (; itl.More(); itl.Next())
                {
                    bb.Remove(wire, itl.Value());
                }
            }
            // Sorting
            if (wire.ClosedFlag)
            {
                closedWires.Append(wire);
            }
            else
            {
                unclosedWires.Append(wire);
            }
        }

        while (!unclosedWires.IsEmpty())
        {
            Wire Base = ShapeCasts.Wire(unclosedWires[1]);
            Exploration.Vertices(Base, out Vertex vf, out Vertex vl);
            if (vf.IsNull || vl.IsNull)
            {
                throw new FailureException("BRepFill_OffsetWire::FixHoles(): Wrong wire.");
            }
            Point3D pf, pl;
            pf = Tool.Pnt(vf);
            pl = Tool.Pnt(vl);
            double distF = RealLast(), distL = RealLast();
            int indexF = 0, indexL = 0;
            bool isFirstF = false, isFirstL = false;
            for (int i = 2; i <= unclosedWires.Length(); i++)
            {
                Wire wire = ShapeCasts.Wire(unclosedWires[i]);
                Exploration.Vertices(wire, out Vertex v1, out Vertex v2);

                if (v1.IsNull || v2.IsNull)
                {
                    throw new FailureException("BRepFill_OffsetWire::FixHoles(): Wrong wire.");
                }

                Point3D p1, p2;
                p1 = Tool.Pnt(v1);
                p2 = Tool.Pnt(v2);
                double dist = pf.DistanceTo(p1);
                if (dist < distF)
                {
                    distF = dist;
                    indexF = i;
                    isFirstF = true;
                }
                dist = pf.DistanceTo(p2);
                if (dist < distF)
                {
                    distF = dist;
                    indexF = i;
                    isFirstF = false;
                }
                dist = pl.DistanceTo(p1);
                if (dist < distL)
                {
                    distL = dist;
                    indexL = i;
                    isFirstL = true;
                }
                dist = pl.DistanceTo(p2);
                if (dist < distL)
                {
                    distL = dist;
                    indexL = i;
                    isFirstL = false;
                }
            }
            if (distF > maxTol)
            {
                indexF = 0;
            }
            if (distL > maxTol)
            {
                indexL = 0;
            }
            Wire theWire;
            Edge theEdge;
            Vertex theVertex;
            double commonTol;
            bool tryToClose = true;
            if (distF <= maxTol && distL <= maxTol && indexF == indexL && isFirstF == isFirstL)
            {
                if (distF < distL)
                {
                    distL = RealLast();
                    indexL++;
                }
                else
                {
                    distF = RealLast();
                    indexF++;
                }
                tryToClose = false;
            }
            if (distF <= maxTol)
            {
                theWire = ShapeCasts.Wire(unclosedWires[indexF]);
                Exploration.Vertices(theWire, out Vertex v1, out Vertex v2);
                IndexedMap<Shape, NCollection_List<Shape>> vEmap = new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(theWire, ShapeType.Vertex, ShapeType.Edge, vEmap);
                theEdge = isFirstF ? ShapeCasts.Edge(vEmap.GetValue(v1).First())
                                   : ShapeCasts.Edge(vEmap.GetValue(v2).First());
                Iterator it = new(theWire);
                for (; it.More(); it.Next())
                {
                    Edge edge = ShapeCasts.Edge(it.Value());
                    if (isFirstF)
                    {
                        edge.Reverse();
                    }
                    if (!edge.IsSame(theEdge))
                    {
                        bb.Add(Base, edge);
                    }
                }
                theVertex = isFirstF ? v1 : v2;
                commonTol = Max(Tool.Tolerance(vf), Tool.Tolerance(theVertex));
                if (distF <= commonTol)
                {
                    theEdge.Free(true);
                    vf.Orient(theVertex.Orientation);
                    bb.Remove(theEdge, theVertex);
                    bb.Add(theEdge, vf);
                    bb.UpdateVertex(vf, commonTol);
                    if (isFirstF)
                    {
                        theEdge.Reverse();
                    }
                    bb.Add(Base, theEdge);
                }
                else
                {
                    if (isFirstF)
                    {
                        theEdge.Reverse();
                    }
                    bb.Add(Base, theEdge);
                    // Creating new edge from theVertex to Vf
                    Edge newEdge = new LibraryMakeEdge(theVertex, vf).InternalEdge();
                    bb.Add(Base, newEdge);
                }
            }
            if (distL <= maxTol && indexL != indexF)
            {
                theWire = ShapeCasts.Wire(unclosedWires[indexL]);
                Exploration.Vertices(theWire, out Vertex v1, out Vertex v2);
                IndexedMap<Shape, NCollection_List<Shape>> vEmap = new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(theWire, ShapeType.Vertex, ShapeType.Edge, vEmap);
                theEdge = isFirstL ? ShapeCasts.Edge(vEmap.GetValue(v1).First())
                                   : ShapeCasts.Edge(vEmap.GetValue(v2).First());
                Iterator it = new(theWire);
                for (; it.More(); it.Next())
                {
                    Edge edge = ShapeCasts.Edge(it.Value());
                    if (!isFirstL)
                    {
                        edge.Reverse();
                    }
                    if (!edge.IsSame(theEdge))
                    {
                        bb.Add(Base, edge);
                    }
                }
                theVertex = isFirstL ? v1 : v2;
                commonTol = Max(Tool.Tolerance(vl), Tool.Tolerance(theVertex));
                if (distL <= commonTol)
                {
                    theEdge.Free(true);
                    vl.Orient(theVertex.Orientation);
                    bb.Remove(theEdge, theVertex);
                    bb.Add(theEdge, vl);
                    bb.UpdateVertex(vl, commonTol);
                    if (!isFirstL)
                    {
                        theEdge.Reverse();
                    }
                    bb.Add(Base, theEdge);
                }
                else
                {
                    if (!isFirstL)
                    {
                        theEdge.Reverse();
                    }
                    bb.Add(Base, theEdge);
                    // Creating new edge from Vl to theVertex
                    Edge newEdge = new LibraryMakeEdge(vl, theVertex).InternalEdge();
                    bb.Add(Base, newEdge);
                }
            }
            // Check if it is possible to close resulting wire
            if (tryToClose)
            {
                Exploration.Vertices(Base, out vf, out vl);
                commonTol = Max(Tool.Tolerance(vf), Tool.Tolerance(vl));
                IndexedMap<Shape, NCollection_List<Shape>> vEmap = new(ShapeHasher.Instance);
                Exploration.MapShapesAndAncestors(Base, ShapeType.Vertex, ShapeType.Edge, vEmap);
                Edge efirst, elast;
                efirst = ShapeCasts.Edge(vEmap.GetValue(vf).First());
                elast = ShapeCasts.Edge(vEmap.GetValue(vl).First());
                pf = Tool.Pnt(vf);
                pl = Tool.Pnt(vl);
                double Dist = pf.DistanceTo(pl);
                if (Dist <= commonTol)
                {
                    elast.Free(true);
                    vf.Orient(vl.Orientation);
                    bb.Remove(elast, vl);
                    bb.Add(elast, vf);
                    bb.UpdateVertex(vf, commonTol);
                    Base.Closed(true);
                }
                else if (Dist <= maxTol)
                {
                    // Creating new edge from Vl to Vf
                    Edge newEdge = new LibraryMakeEdge(vf, vl).InternalEdge();
                    bb.Add(Base, newEdge);
                    Base.Closed(true);
                }
                _ = efirst;
            }
            // Updating sequences ClosedWires and UnclosedWires
            if (distF <= maxTol)
            {
                unclosedWires.Remove(indexF);
            }
            if (distL <= maxTol && indexL != indexF)
            {
                if (distF <= maxTol && indexL > indexF)
                {
                    indexL--;
                }
                unclosedWires.Remove(indexL);
            }
            if (Base.ClosedFlag)
            {
                closedWires.Append(Base);
                unclosedWires.Remove(1);
            }
            else if (distF > maxTol && distL > maxTol)
            {
                isolatedWires.Append(Base);
                unclosedWires.Remove(1);
            }
        }

        // Updating myShape
        if (closedWires.Length() + isolatedWires.Length() == 1)
        {
            if (!closedWires.IsEmpty())
            {
                _shape = closedWires.First();
            }
            else
            {
                _shape = isolatedWires.First();
            }
        }
        else
        {
            Compound r = new();
            bb.MakeCompound(ref r);
            int i;
            for (i = 1; i <= closedWires.Length(); i++)
            {
                bb.Add(r, closedWires[i]);
            }
            for (i = 1; i <= isolatedWires.Length(); i++)
            {
                bb.Add(r, isolatedWires[i]);
            }
            _shape = r;
        }
    }

    // Cut edge at the extrema of curvatures and points of inflexion. So, closed circles are cut in two. If Cuts is
    // empty, the edge is not modified. The first and the last vertex of the initial edge belong to the first and the
    // last parts respectively.
    private static int CutEdge(in Edge e, in Face F, int forceCut, NCollection_List<Shape> cuts)
    {
        cuts.Clear();
        CutCurve cuter = new();
        NCollection_Sequence<Curve2D> theCurves = new();
        double f, l;
        Curve2D c2d;
        TrimmedCurve2D ct2d;
        double tol = Tool.Tolerance(e);
        Curve3D? c;

        Vertex vf, vl;
        Exploration.Vertices(e, out Vertex v1, out Vertex v2);
        BrepBuilder b = new();

        c2d = Tool.CurveOnSurface(e, F, out f, out l)!;
        c = Tool.Curve(e, out f, out l);
        ct2d = new TrimmedCurve2D(c2d, f, l);

        if (ct2d.ChangeBasisCurve() is CircleCurve2D && Math.Abs(f - l) >= Math.PI)
        {
            return 0;
        }

        //-------------------------
        // Cut curve.
        //-------------------------
        cuter.Perform(ct2d);

        if (forceCut == 0)
        {
            if (cuter.UnModified())
            {
                //-----------------------------
                // edge not modified => return.
                //-----------------------------
                return 0;
            }
            else
            {
                for (int k = 1; k <= cuter.CurveCount; k++)
                {
                    theCurves.Append(cuter.Value(k));
                }
            }
        }
        else if (forceCut == 1)
        {
            if (cuter.UnModified())
            {
                CutCurve(ct2d, 2, theCurves);
            }
            else
            {
                for (int k = 1; k <= cuter.CurveCount; k++)
                {
                    theCurves.Append(cuter.Value(k));
                }
            }
        }
        else if (forceCut == 2)
        {
            if (cuter.UnModified())
            {
                CutCurve(ct2d, 3, theCurves);
            }
            else
            {
                if (cuter.CurveCount == 2)
                {
                    TrimmedCurve2D cc = cuter.Value(1);

                    if (cc.LastParameter > (l + f) / 2.0)
                    {
                        CutCurve(cc, 2, theCurves);
                        theCurves.Append(cuter.Value(2));
                    }
                    else
                    {
                        theCurves.Append(cc);
                        CutCurve(cuter.Value(2), 2, theCurves);
                    }
                }
                else
                {
                    for (int k = 1; k <= cuter.CurveCount; k++)
                    {
                        theCurves.Append(cuter.Value(k));
                    }
                }
            }
        }

        //--------------------------------------
        // Creation of cut edges.
        //--------------------------------------
        vf = v1;

        for (int k = 1; k <= theCurves.Length(); k++)
        {
            TrimmedCurve2D cc = (TrimmedCurve2D)theCurves.Value(k);

            if (k == theCurves.Length())
            {
                vl = v2;
            }
            else
            {
                Point3D p = c!.Value(cc.LastParameter);

                vl = new LibraryMakeVertex(p).InternalVertex();
                b.UpdateVertex(vl, tol);
            }
            Shape localShape = e.EmptyCopied();
            Edge ne = ShapeCasts.Edge(localShape);
            ne.Orient(Forward);
            b.Add(ne, vf.Oriented(Forward));
            b.Add(ne, vl.Oriented(Reversed));
            b.Range(ne, cc.FirstParameter, cc.LastParameter);
            cuts.Append(ne.Oriented(e.Orientation));
            vf = vl;
        }

        return theCurves.Length();
    }

    // Cuts the curve in nbParts parts of equal parameter range (parts too small are merged).
    private static void CutCurve(TrimmedCurve2D c, int nbParts, NCollection_Sequence<Curve2D> curves)
    {
        TrimmedCurve2D trimC;
        double uf, ul, uc;
        double step;
        Point2D pf, pl, pc;
        double pTol = global::Kerf.Engine.Foundation.Precision.PConfusion() * 10;
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion() * 10;
        bool yaCut = false;

        uf = c.FirstParameter;
        ul = c.LastParameter;
        pf = c.Value(uf);
        pl = c.Value(ul);

        step = (ul - uf) / nbParts;

        for (int i = 1; i < nbParts; i++)
        {
            uc = uf + i * step;
            pc = c.Value(uc);

            if (uc - uf > pTol && pc.DistanceTo(pf) > tol)
            {
                if (ul - uc < pTol || pl.DistanceTo(pc) < tol)
                {
                    continue;
                }

                trimC = new TrimmedCurve2D(c, uf, uc);
                curves.Append(trimC);
                uf = uc;
                pf = pc;
                yaCut = true;
            }
        }
        if (yaCut)
        {
            trimC = new TrimmedCurve2D(c, uf, ul);
            curves.Append(trimC);
        }
        else
        {
            curves.Append(c);
        }
    }

    // The arc of the offset around the vertex V, which follows the edge E in the contour.
    private static void MakeCircle(in Edge e,
                                   in Vertex v,
                                   in Face F,
                                   double offset,
                                   IndexedMap<Shape, NCollection_List<Shape>> map,
                                   PlaneSurface refPlane)
    {
        // evaluate the Axis of the Circle.
        Curve2D gc = Tool.CurveOnSurface(e, F, out double f, out double l)!;
        Vector2D dx;
        Point2D p;

        if (e.Orientation == Forward)
        {
            gc.D1(l, out p, out dx);
            dx = dx.Reversed();
        }
        else
        {
            gc.D1(f, out p, out dx);
        }

        Axis2D axis = new(p, new Direction2D(dx));
        CircleCurve2D circ = new(axis, Math.Abs(offset), offset < 0.0);

        // Bind the edges in my Map.
        Edge oe = new LibraryMakeEdge(circ, refPlane).InternalEdge();
        NCollection_List<Shape> ll = new();

        ll.Append(oe);
        map.Add(v, ll);
    }

    // The parallel of the edge E, extended at its ends for the trimming.
    private static void MakeOffset(in Edge e,
                                   in Face F,
                                   double offset,
                                   IndexedMap<Shape, NCollection_List<Shape>> map,
                                   PlaneSurface refPlane,
                                   bool isOpenResult,
                                   global::Kerf.Geometry.JoinKind joinType,
                                   Vertex[] ends)
    {
        double f, l;
        double anOffset = offset;

        if (e.Orientation == Forward)
        {
            anOffset *= -1;
        }

        Curve2D g2d = Tool.CurveOnSurface(e, F, out f, out l)!;
        Curve2D? g2dOC = null;

        bool toExtendFirstPar = true;
        bool toExtendLastPar = true;
        if (isOpenResult)
        {
            Exploration.Vertices(e, out Vertex v1, out Vertex v2);
            if (v1.IsSame(ends[0]) || v1.IsSame(ends[1]))
            {
                toExtendFirstPar = false;
            }
            if (v2.IsSame(ends[0]) || v2.IsSame(ends[1]))
            {
                toExtendLastPar = false;
            }
        }

        CurveAdaptor2D ac = new(g2d, f, l);
        if (ac.GetType() == Circle)
        {
            // if the offset is greater otr equal to the radius and the side of the
            // concavity of the circle => edge null.
            Circle2D c1 = ac.Circle();
            Frame2D axes = c1.Axis;
            Direction2D xd = axes.XDirection;
            Direction2D yd = axes.YDirection;
            double crossed = xd.X * yd.Y - xd.Y * yd.X;
            double signe = (crossed > 0.0) ? -1.0 : 1.0;

            if (anOffset * signe < ac.Circle().Radius - global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                CurveAdaptor2D ahc = new(g2d);
                OffsetCurveAdaptor2D off = new(ahc, anOffset);
                CircleCurve2D cc = new(off.Circle());

                double delta = 2 * Math.PI - l + f;
                if (joinType == Arc)
                {
                    if (toExtendFirstPar)
                    {
                        f -= 0.2 * delta;
                    }
                    if (toExtendLastPar)
                    {
                        l += 0.2 * delta;
                    }
                }
                else // GeomAbs_Intersection
                {
                    if (toExtendFirstPar && toExtendLastPar)
                    {
                        double old_l = l;
                        f = old_l + delta / 2.0;
                        l = f + 2 * Math.PI;
                    }
                    else if (toExtendFirstPar)
                    {
                        f = l;
                        l = f + 2 * Math.PI;
                    }
                    else if (toExtendLastPar)
                    {
                        l = f + 2 * Math.PI;
                    }
                }
                g2dOC = new TrimmedCurve2D(cc, f, l);
            }
        }
        else if (ac.GetType() == Line)
        {
            CurveAdaptor2D ahc = new(g2d);
            OffsetCurveAdaptor2D off = new(ahc, anOffset);
            LineCurve2D cc = new(off.Line());
            double delta = l - f;
            if (toExtendFirstPar)
            {
                if (joinType == Arc)
                {
                    f -= delta;
                }
                else
                { // GeomAbs_Intersection
                    f = -global::Kerf.Engine.Foundation.Precision.Infinite();
                }
            }
            if (toExtendLastPar)
            {
                if (joinType == Arc)
                {
                    l += delta;
                }
                else
                { // GeomAbs_Intersection
                    l = global::Kerf.Engine.Foundation.Precision.Infinite();
                }
            }
            g2dOC = new TrimmedCurve2D(cc, f, l);
        }
        else
        {
            TrimmedCurve2D g2dT = new(g2d, f, l);
            g2dOC = new OffsetCurve2D(g2dT, anOffset);
        }

        // Bind the edges in my Map.
        if (g2dOC is not null)
        {
            Edge oe = new LibraryMakeEdge(g2dOC, refPlane).InternalEdge();
            oe.Orient(e.Orientation);
            NCollection_List<Shape> ll = new();
            ll.Append(oe);
            map.Add(e, ll);
        }
    }

    // The vertex of a node of the locus that lies on the offset (one per node).
    private static bool VertexFromNode(MedialNode aNode,
                                       double offset,
                                       in Point2D pn,
                                       Dictionary<MedialNode, Shape> mapNodeVertex,
                                       ref Vertex vn)
    {
        bool status;
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        BrepBuilder b = new();

        if (!aNode.Infinite() && Math.Abs(aNode.Distance() - offset) < tol)
        {
            //------------------------------------------------
            // the Node gives a vertex on the offset
            //------------------------------------------------
            if (mapNodeVertex.ContainsKey(aNode))
            {
                vn = ShapeCasts.Vertex(mapNodeVertex.GetValue(aNode));
            }
            else
            {
                Point3D p = new(pn.X, pn.Y, 0.0);
                b.MakeVertex(ref vn);
                b.UpdateVertex(vn, p, global::Kerf.Engine.Foundation.Precision.Confusion());
                mapNodeVertex[aNode] = vn;
            }
            status = true;
        }
        else
        {
            status = false;
        }

        return status;
    }

    private static void StoreInMap(in Shape v1, in Shape v2, IndexedMap<Shape, Shape> mapVV)
    {
        Shape newV = v2;
        int i;

        if (mapVV.ContainsKey(v2))
        {
            newV = mapVV.GetValue(v2);
        }

        if (mapVV.ContainsKey(v1))
        {
            mapVV.GetValueRef(v1) = newV;
        }

        for (i = 1; i <= mapVV.Count; i++)
        {
            if (mapVV.ValueAt(i - 1).IsSame(v1))
            {
                mapVV.ValueAt(i - 1) = newV;
            }
        }

        mapVV.Add(v1, newV);
    }

    // The parts of the parallel E between its vertices on the bisectors that are kept (the proofing Detromp).
    private static void TrimEdge(in Edge e,
                                 in Shape proE,
                                 in Face allSpine,
                                 NCollection_List<Shape> detromp,
                                 NCollection_Sequence<Shape> theVer,
                                 NCollection_Sequence<double> thePar,
                                 NCollection_Sequence<Shape> s,
                                 IndexedMap<Shape, Shape> mapVV,
                                 int indOfE)
    {
        bool change = true;
        BrepBuilder theBuilder = new();
        s.Clear();

        //-----------------------------------------------------------
        // Parse two sequences depending on the parameter on the edge.
        //-----------------------------------------------------------
        while (change)
        {
            change = false;
            for (int i = 1; i < thePar.Length(); i++)
            {
                if (thePar.Value(i) > thePar.Value(i + 1))
                {
                    thePar.Exchange(i, i + 1);
                    theVer.Exchange(i, i + 1);
                    change = true;
                }
            }
        }

        //----------------------------------------------------------
        // If a vertex is not in the proofing, it is eliminated.
        //----------------------------------------------------------
        if (!Tool.Degenerated(e))
        {
            for (int k = 1; k <= theVer.Length(); k++)
            {
                if (DoubleOrNotInside(detromp, ShapeCasts.Vertex(theVer.Value(k))))
                {
                    theVer.Remove(k);
                    thePar.Remove(k);
                    k--;
                }
            }
        }

        //----------------------------------------------------------
        // If a vertex_double appears twice in the proofing
        // the vertex is removed.
        // otherwise preserve only one of its representations.
        //----------------------------------------------------------
        if (!Tool.Degenerated(e))
        {
            double parTol = 2.0 * global::Kerf.Engine.Foundation.Precision.PConfusion();
            for (int k = 1; k < theVer.Length(); k++)
            {
                if (theVer.Value(k).IsSame(theVer.Value(k + 1))
                    || Math.Abs(thePar.Value(k) - thePar.Value(k + 1)) <= parTol)
                {
                    if (k + 1 == theVer.Length())
                    {
                        StoreInMap(theVer[k], theVer[k + 1], mapVV);
                        theVer.Remove(k);
                        thePar.Remove(k);
                    }
                    else
                    {
                        StoreInMap(theVer[k + 1], theVer[k], mapVV);
                        theVer.Remove(k + 1);
                        thePar.Remove(k + 1);
                    }
                    k--;
                }
            }
        }
        //-----------------------------------------------------------
        // Creation of edges.
        // the number of vertices should be even. The created edges
        // go from a vertex with uneven index i to vertex i+1;
        //-----------------------------------------------------------
        if (indOfE == 1 || indOfE == -1) // open result and extreme edges of result
        {
            Shape localShape = e.EmptyCopied();
            Edge newEdge = ShapeCasts.Edge(localShape);
            Exploration.Vertices(e, out Vertex v1, out Vertex v2);
            Tool.CurveOnSurface(e, out Curve2D? pCurve, out Surface? _, out Placement _, out double fpar, out double lpar);

            bool toTrimAsOrigin = IsInnerEdge(proE, allSpine, out double trPar1, out double trPar2);

            if (indOfE == 1) // first edge of open wire
            {
                if (newEdge.Orientation == Forward)
                {
                    if (toTrimAsOrigin)
                    {
                        fpar = trPar1;
                        Point2D trPnt2d = pCurve!.Value(fpar);
                        Point3D trPnt = new(trPnt2d.X, trPnt2d.Y, 0.0);
                        v1 = new LibraryMakeVertex(trPnt).InternalVertex();
                    }
                    theBuilder.Add(newEdge, v1.Oriented(Forward));
                    theBuilder.Add(newEdge, theVer.First().Oriented(Reversed));
                    theBuilder.Range(newEdge, fpar, thePar.First());
                }
                else
                {
                    if (toTrimAsOrigin)
                    {
                        lpar = trPar2;
                        Point2D trPnt2d = pCurve!.Value(lpar);
                        Point3D trPnt = new(trPnt2d.X, trPnt2d.Y, 0.0);
                        v2 = new LibraryMakeVertex(trPnt).InternalVertex();
                    }
                    theBuilder.Add(newEdge, theVer.First().Oriented(Reversed));
                    theBuilder.Add(newEdge, v2.Oriented(Forward));
                    theBuilder.Range(newEdge, thePar.First(), lpar);
                }
            }
            else // last edge of open wire
            {
                if (newEdge.Orientation == Forward)
                {
                    if (toTrimAsOrigin)
                    {
                        lpar = trPar2;
                        Point2D trPnt2d = pCurve!.Value(lpar);
                        Point3D trPnt = new(trPnt2d.X, trPnt2d.Y, 0.0);
                        v2 = new LibraryMakeVertex(trPnt).InternalVertex();
                    }
                    theBuilder.Add(newEdge, theVer.First().Oriented(Forward));
                    theBuilder.Add(newEdge, v2.Oriented(Reversed));
                    theBuilder.Range(newEdge, thePar.First(), lpar);
                }
                else
                {
                    if (toTrimAsOrigin)
                    {
                        fpar = trPar1;
                        Point2D trPnt2d = pCurve!.Value(fpar);
                        Point3D trPnt = new(trPnt2d.X, trPnt2d.Y, 0.0);
                        v1 = new LibraryMakeVertex(trPnt).InternalVertex();
                    }
                    theBuilder.Add(newEdge, v1.Oriented(Reversed));
                    theBuilder.Add(newEdge, theVer.First().Oriented(Forward));
                    theBuilder.Range(newEdge, fpar, thePar.First());
                }
            }
            s.Append(newEdge);
        }
        else
        {
            for (int k = 1; k < theVer.Length(); k = k + 2)
            {
                Shape localShape = e.EmptyCopied();
                Edge newEdge = ShapeCasts.Edge(localShape);

                if (newEdge.Orientation == Reversed)
                {
                    theBuilder.Add(newEdge, theVer.Value(k).Oriented(Reversed));
                    theBuilder.Add(newEdge, theVer.Value(k + 1).Oriented(Forward));
                }
                else
                {
                    theBuilder.Add(newEdge, theVer.Value(k).Oriented(Forward));
                    theBuilder.Add(newEdge, theVer.Value(k + 1).Oriented(Reversed));
                }

                theBuilder.Range(newEdge, thePar.Value(k), thePar.Value(k + 1));

                s.Append(newEdge);
            }
        }
    }

    private static bool IsInnerEdge(in Shape proE, in Face allSpine, out double trPar1, out double trPar2)
    {
        trPar1 = trPar2 = 0.0;
        if (proE.Type != ShapeType.Edge)
        {
            return false;
        }

        Edge edge = ShapeCasts.Edge(proE);

        IndexedMap<Shape, NCollection_List<Shape>> vEmap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(allSpine, ShapeType.Vertex, ShapeType.Edge, vEmap);
        for (int i = 0; i < vEmap.Count; i++)
        {
            NCollection_List<Shape> le = vEmap.ValueAt(i);
            if (le.Extent() == 1 && edge.IsSame(le.First()))
            {
                return false;
            }
        }

        Tool.Range(edge, out trPar1, out trPar2);
        return true;
    }

    // True if V appears twice in LV or is not inside.
    private static bool DoubleOrNotInside(NCollection_List<Shape> lv, in Vertex v)
    {
        bool vu = false;
        NCollection_List<Shape>.Iterator it = new(lv);

        for (; it.More(); it.Next())
        {
            if (v.IsSame(it.Value()))
            {
                if (vu)
                {
                    return true;
                }
                else
                {
                    vu = true;
                }
            }
        }
        return !vu;
    }

    private static bool IsSmallClosedEdge(in Edge anEdge, in Vertex aVertex)
    {
        Point3D pv = Tool.Pnt(aVertex);
        Point2D pv2d = new(), pfirst, plast, pmid;
        pv2d = pv2d.WithCoord(pv.X, pv.Y);

        Curve2D pCurve;
        CurveRepresentation curveRep = ((BrepTEdge)anEdge.TShape()!).Curves().First();
        pCurve = curveRep.PCurve()!;

        double fpar = ((GCurve)curveRep).First();
        double lpar = ((GCurve)curveRep).Last();
        pfirst = pCurve.Value(fpar);
        plast = pCurve.Value(lpar);
        pmid = pCurve.Value((fpar + lpar) * 0.5);

        double theTol = Tool.Tolerance(aVertex);
        theTol *= 1.5;

        double dist1 = pfirst.DistanceTo(pv2d);
        double dist2 = plast.DistanceTo(pv2d);
        double dist3 = pmid.DistanceTo(pv2d);

        return dist1 <= theTol && dist2 <= theTol && dist3 <= theTol;
    }

    // The edges, other than lines and circles, whose curvature at an end is at least 1/Offset on the side of the offset.
    private static void CheckBadEdges(in Face spine,
                                      double offset,
                                      BisectingLocus locus,
                                      LinkTopoBilo link,
                                      NCollection_List<Shape> badEdges)
    {
        Face F = ShapeCasts.Face(spine.Oriented(Forward));
        double eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        double limCurv = 1.0 / offset;

        HashSet<Shape> map = new(ShapeHasher.Instance);

        for (int ic = 1; ic <= locus.NumberOfContours(); ic++)
        {
            for (int ie = 1; ie <= locus.NumberOfElts(ic); ie++)
            {
                Shape se = link.GeneratingShape(locus.BasicElt(ic, ie));
                if (se.Type == ShapeType.Edge)
                {
                    if (map.Contains(se))
                    {
                        continue;
                    }

                    Edge e = ShapeCasts.Edge(se);

                    Curve2D g2d = Tool.CurveOnSurface(e, F, out double f, out double l)!;

                    CurveAdaptor2D ac = new(g2d, f, l);
                    CurveKind cType = ac.GetType();

                    if (cType != Line && cType != Circle)
                    {
                        bool reverse = false;
                        if (e.Orientation == Forward)
                        {
                            reverse = true;
                        }

                        Point2D p, pc;
                        Direction2D n;

                        CLProps2d clProps = new(g2d, 2, eps);

                        clProps.SetParameter(f);
                        if (!clProps.IsTangentDefined())
                        {
                            badEdges.Append(se);
                            map.Add(se);
                            continue;
                        }

                        p = clProps.Value();
                        double crv = clProps.Curvature();

                        if (crv >= eps)
                        {
                            clProps.Tangent(out n);
                            double x = n.Y, y = -n.X;
                            n = n.WithCoord(x, y);
                            if (reverse)
                            {
                                n = n.Reversed();
                            }
                            clProps.CentreOfCurvature(out pc);
                            Vector2D dir = new(p, pc);
                            if (n.Dot(new Direction2D(dir)) > 0.0) // C++ converts the vector to a direction implicitly
                            {
                                if (limCurv <= crv + eps)
                                {
                                    badEdges.Append(se);
                                    map.Add(se);
                                    continue;
                                }
                            }
                        }

                        clProps.SetParameter(l);
                        if (!clProps.IsTangentDefined())
                        {
                            badEdges.Append(se);
                            map.Add(se);
                            continue;
                        }

                        p = clProps.Value();
                        crv = clProps.Curvature();

                        if (crv >= eps)
                        {
                            clProps.Tangent(out n);
                            double x = n.Y, y = -n.X;
                            n = n.WithCoord(x, y);
                            if (reverse)
                            {
                                n = n.Reversed();
                            }
                            clProps.CentreOfCurvature(out pc);
                            Vector2D dir = new(p, pc);
                            if (n.Dot(new Direction2D(dir)) > 0.0) // C++ converts the vector to a direction implicitly
                            {
                                if (limCurv <= crv + eps)
                                {
                                    badEdges.Append(se);
                                    map.Add(se);
                                    continue;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    private static bool PerformCurve(NCollection_Sequence<double> parameters,
                                     NCollection_Sequence<Point3D> points,
                                     CurveAdaptorBase c,
                                     double deflection,
                                     double u1,
                                     double u2,
                                     double epsilon,
                                     int nbmin)
    {
        double uu1 = Min(u1, u2);
        double uu2 = Max(u1, u2);

        c.D1(uu1, out Point3D pdeb, out Vector3D ddeb);
        parameters.Append(uu1);
        points.Append(pdeb);

        c.D1(uu2, out Point3D pfin, out Vector3D dfin);

        double delta = uu2 - uu1;
        double dist = pdeb.DistanceTo(pfin);

        if ((delta / dist) > 5.0e-14)
        {
            QuasiFleche(c, deflection * deflection, uu1, pdeb, ddeb, uu2, pfin, dfin, nbmin, epsilon * epsilon, parameters, points);
        }

        return true;
    }

    private static void QuasiFleche(CurveAdaptorBase c,
                                    double deflection2,
                                    double udeb,
                                    in Point3D pdeb,
                                    in Vector3D vdeb,
                                    double ufin,
                                    in Point3D pfin,
                                    in Vector3D vfin,
                                    int nbmin,
                                    double eps,
                                    NCollection_Sequence<double> parameters,
                                    NCollection_Sequence<Point3D> points)
    {
        int ptslength = points.Length();
        double udelta = ufin - udeb;
        Point3D pdelta;
        Vector3D vdelta;
        if (nbmin > 2)
        {
            udelta /= (nbmin - 1);
            c.D1(udeb + udelta, out pdelta, out vdelta);
        }
        else
        {
            pdelta = pfin;
            vdelta = vfin;
        }

        double norme = new Vector3D(pdeb, pdelta).SquareMagnitude;
        double theFleche = 0;
        bool flecheok = false;
        if (norme > eps)
        {
            // Evaluation of the arrow by interpolation. See IntWalk_IWalking::TestDeflection
            double n1 = vdeb.SquareMagnitude;
            double n2 = vdelta.SquareMagnitude;
            if (n1 > eps && n2 > eps)
            {
                double normediff = (vdeb.Normalized().XYZ() - vdelta.Normalized().XYZ()).SquareModulus();
                if (normediff > eps)
                {
                    theFleche = normediff * norme / 64.0;
                    flecheok = true;
                }
            }
        }
        if (!flecheok)
        {
            Point3D pmid = new((pdeb.XYZ() + pdelta.XYZ()) / 2.0);
            Point3D pverif = c.Value(udeb + udelta / 2.0);
            theFleche = pmid.SquareDistanceTo(pverif);
        }

        if (theFleche < deflection2)
        {
            parameters.Append(udeb + udelta);
            points.Append(pdelta);
        }
        else
        {
            QuasiFleche(c, deflection2, udeb, pdeb, vdeb, udeb + udelta, pdelta, vdelta, 3, eps, parameters, points);
        }

        if (nbmin > 2)
        {
            QuasiFleche(c, deflection2, udeb + udelta, pdelta, vdelta, ufin, pfin, vfin,
                        nbmin - (points.Length() - ptslength), eps, parameters, points);
        }
    }

    private static bool CheckSmallParamOnEdge(in Edge anEdge)
    {
        NCollection_List<CurveRepresentation> list = ((BrepTEdge)anEdge.TShape()!).Curves();
        if (!list.IsEmpty())
        {
            CurveRepresentation cRep = ((BrepTEdge)anEdge.TShape()!).Curves().First();
            double f = ((GCurve)cRep).First();
            double l = ((GCurve)cRep).Last();
            if (Math.Abs(l - f) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return false;
            }
        }
        return true;
    }
}
