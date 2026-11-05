// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_FuseEdges.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepLib/BRepLib_FuseEdges.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Library;

/// <summary>
/// Fuses the connected edges of a shape that lie on the same curve (line, circle, ellipse, B-spline, Bezier): each
/// vertex shared by exactly two such edges, bounding the same faces, is removed and the edges become one.
/// </summary>
/// <remarks>
/// A working object (decision 21). It only reads the shape it is given: the fused edges, and the faces, wires and
/// parents that hold them, are new shapes (BRepTools_Substitution). <see cref="Shape"/>, <see cref="ResultEdges()"/>
/// and <see cref="Faces()"/> freeze what they return; ported code reads <see cref="InternalShape"/> and the internal
/// map forms.
/// </remarks>
internal sealed class FuseEdges
{
    private Shape _shape;
    private bool _shapeDone;
    private bool _edgesDone;
    private bool _resultEdgesDone;
    private readonly IndexedMap<Shape, NCollection_List<Shape>> _mapVerLstEdg =
        new(ShapeHasher.Instance);
    private readonly IndexedMap<Shape, NCollection_List<Shape>> _mapEdgLstFac =
        new(ShapeHasher.Instance);
    private readonly Dictionary<int, NCollection_List<Shape>> _mapLstEdg = new();
    private readonly Dictionary<int, Shape> _mapEdg = new();
    private readonly Dictionary<Shape, Shape> _mapFaces = new(ShapeHasher.Instance);
    private int _nbConnexEdge;
    private IndexedSet<Shape> _avoidEdg = new(ShapeHasher.Instance);
    private bool _concatBSpl;

    private static void BCSmoothing(BSplineCurve3D c, int cont, double tol)
    {
        int nbIter = 5;
        bool bContinue = true;
        int iter = 1;
        var knotIndex = new NCollection_Sequence<int>();
        var knotIns = new NCollection_Sequence<double>();

        while (bContinue && iter <= nbIter)
        {
            int nbKnots = c.KnotCount;
            NCollection_Array1<int> mults = new(c.InternalMultiplicities());
            NCollection_Array1<double> knots = new(c.InternalKnots());
            int i, m = c.Degree;
            m = m - cont;

            if (m < 1)
            {
                return;
            }

            knotIndex.Clear();

            for (i = 2; i < nbKnots; i++)
            {
                if (mults[i] > m)
                {
                    if (!(c.RemoveKnot(i, m, tol)))
                    {
                        knotIndex.Append(i);
                    }
                }
            }

            // Prepare knots for inserting;

            int nbAdd = knotIndex.Length();

            if (nbAdd == 0)
            {
                return;
            }

            knotIns.Clear();

            double lastKnot = knots[1];
            for (i = 1; i <= nbAdd; i++)
            {
                int ind = knotIndex[i];

                double k1 = 0.5 * (knots[ind] + knots[ind - 1]);
                if (Math.Abs(k1 - lastKnot) > 1.0e-3)
                {
                    knotIns.Append(k1);
                    lastKnot = k1;
                }

                double k2 = 0.5 * (knots[ind + 1] + knots[ind]);

                if (Math.Abs(k2 - lastKnot) > 1.0e-3)
                {
                    knotIns.Append(k2);
                    lastKnot = k2;
                }
            }

            nbAdd = knotIns.Length();

            for (i = 1; i <= nbAdd; i++)
            {
                c.InsertKnot(knotIns[i], m, 1.0e-3);
            }

            iter++;
        }

        if (iter > nbIter)
        {
            BCSmoothing(c, cont, 1.0e10);
        }
    }

    private static void MakeClosedCurve(ref Curve3D c, in Point3D pf, ref double f, ref double l)
    {
        BSplineCurve3D bc = (BSplineCurve3D)c;
        // OCCT.NET: frozen. OCCT makes the curve periodic and moves its origin in place, and the curve can be the old
        // edge's own (or the basis curve of its trimmed curve), shared with the caller's shape: the port works on a copy.
        if (bc.IsFrozen)
        {
            bc = (BSplineCurve3D)bc.Copy();
        }
        Continuity cont = bc.Continuity;
        // Find new origin
        bc.SetPeriodic();
        int fk = bc.FirstUKnotIndex();
        int lk = bc.LastUKnotIndex();
        int k;
        double eps = global::Kerf.Engine.Foundation.Precision.Confusion();
        eps *= eps;
        double porig = 2.0 * l;
        double dmin = 1.0e100, pmin = f;
        for (k = fk; k <= lk; k++)
        {
            Point3D p = bc.Value(bc.Knot(k));
            double d = pf.SquareDistanceTo(p);
            if (pf.SquareDistanceTo(p) > eps)
            {
                if (d < dmin)
                {
                    pmin = bc.Knot(k);
                    dmin = d;
                }
                continue;
            }

            porig = bc.Knot(k);
            break;
        }
        if (porig > l)
        {
            // try to project
            var gac = new CurveAdaptor(bc);
            var prj = new LocateExtPC(pf, gac, pmin, global::Kerf.Engine.Foundation.Precision.PConfusion());
            if (prj.IsDone)
            {
                porig = prj.Point().Parameter();
            }
            else
            {
                throw new ConstructionErrorException("FuseEdges : Projection failed for closed curve");
            }
        }

        bc.SetOrigin(porig, global::Kerf.Engine.Foundation.Precision.PConfusion());
        f = bc.FirstParameter;
        l = bc.LastParameter;
        bc.Segment(f, l);
        if (cont > Continuity.C0 && bc.Continuity == Continuity.C0)
        {
            BCSmoothing(bc, 1, global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        c = bc;
        f = c.FirstParameter;
        l = c.LastParameter;
    }

    /// <summary>
    /// Initialises the algorithm on <paramref name="shape"/>. As in OCCT, <paramref name="performNow"/> is ignored:
    /// the work is done on demand.
    /// </summary>
    /// <exception cref="NullObjectException">The shape is null.</exception>
    public FuseEdges(in Shape shape, bool performNow = false)
    {
        _ = performNow;
        _shape = shape;
        _shapeDone = false;
        _edgesDone = false;
        _resultEdgesDone = false;
        _nbConnexEdge = 0;
        _concatBSpl = false;
        //  if (theShape.ShapeType() != TopAbs_SHELL && theShape.ShapeType() != TopAbs_SOLID)
        //    throw Standard_ConstructionError("FuseEdges");
        Standard_NullObject_Raise_if(shape.IsNull, "FuseEdges");
        _mapFaces.Clear();
    }

    /// <summary>Sets the edges that must not be fused (a copy of the map is kept).</summary>
    public void AvoidEdges(IndexedSet<Shape> mapEdg)
    {
        _avoidEdg = new IndexedSet<Shape>(mapEdg);
    }

    /// <summary>Sets whether connected B-spline edges that are only G1 at their common vertex may be concatenated.</summary>
    public void SetConcatBSpl(bool concatBSpl = true)
    {
        _concatBSpl = concatBSpl;
    }

    /// <summary>
    /// The lists of connected edges to be fused, by index: each list is a set of connected edges, in the forward
    /// direction, that can be fused into one. The edges are the input shape's own.
    /// </summary>
    public IReadOnlyDictionary<int, IReadOnlyList<Shape>> Edges()
    {
        var map = new Dictionary<int, NCollection_List<Shape>>();
        Edges(map);
        var result = new Dictionary<int, IReadOnlyList<Shape>>();
        foreach (KeyValuePair<int, NCollection_List<Shape>> item in map)
        {
            result.Add(item.Key, ImmutableArrays.Own([.. item.Value]));
        }
        return result;
    }

    /// <summary>OCCT's form: copies the lists of edges to be fused into <paramref name="mapLstEdg"/> (ported code).</summary>
    internal void Edges(Dictionary<int, NCollection_List<Shape>> mapLstEdg)
    {
        if (!_edgesDone)
        {
            BuildListEdges();
        }

        // theMapLstEdg = myMapLstEdg: C++ copies the map and its lists
        mapLstEdg.Clear();
        foreach (KeyValuePair<int, NCollection_List<Shape>> item in _mapLstEdg)
        {
            mapLstEdg[item.Key] = new NCollection_List<Shape>(item.Value);
        }
    }

    /// <summary>The fused edges (frozen), by the index of the list of edges each replaces (see <see cref="Edges()"/>).</summary>
    public IReadOnlyDictionary<int, Shape> ResultEdges()
    {
        var map = new Dictionary<int, Shape>();
        ResultEdges(map);
        var result = new Dictionary<int, Shape>();
        foreach (KeyValuePair<int, Shape> item in map)
        {
            item.Value.TShape()?.FreezeGraph();
            result.Add(item.Key, item.Value);
        }
        return result;
    }

    /// <summary>OCCT's form: copies the fused edges into <paramref name="mapEdg"/>, unfrozen (ported code).</summary>
    internal void ResultEdges(Dictionary<int, Shape> mapEdg)
    {
        if (!_edgesDone)
        {
            BuildListEdges();
        }

        if (!_resultEdgesDone)
        {
            BuildListResultEdges();
        }

        mapEdg.Clear();
        foreach (KeyValuePair<int, Shape> item in _mapEdg)
        {
            mapEdg[item.Key] = item.Value;
        }
    }

    /// <summary>The faces modified by the fusion: each face of the input mapped to its new face (frozen).</summary>
    public IReadOnlyDictionary<Shape, Shape> Faces()
    {
        var map = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        Faces(map);
        var result = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        foreach (KeyValuePair<Shape, Shape> item in map)
        {
            item.Value.TShape()?.FreezeGraph();
            result.Add(item.Key, item.Value);
        }
        return result;
    }

    /// <summary>OCCT's form: copies the modified faces into <paramref name="mapFac"/>, unfrozen (ported code).</summary>
    internal void Faces(Dictionary<Shape, Shape> mapFac)
    {
        if (!_edgesDone)
        {
            BuildListEdges();
        }

        if (!_resultEdgesDone)
        {
            BuildListResultEdges();
        }

        if (!_shapeDone)
        {
            Perform();
        }

        mapFac.Clear();
        foreach (KeyValuePair<Shape, Shape> item in _mapFaces)
        {
            mapFac[item.Key] = item.Value;
        }
    }

    /// <summary>The number of vertices removed by the fusion.</summary>
    public int NbVertices()
    {
        Standard_NullObject_Raise_if(_shape.IsNull, "FuseEdges : No Shape");
        int nbedges, nbvertices = 0;

        if (!_edgesDone)
        {
            BuildListEdges();
        }

        if ((nbedges = _mapLstEdg.Count) > 0)
        {
            foreach (KeyValuePair<int, NCollection_List<Shape>> itEdg in _mapLstEdg)
            {
                int iLst = itEdg.Key;
                NCollection_List<Shape> lmapEdg = _mapLstEdg.GetValue(iLst);
                nbvertices += lmapEdg.Extent() - 1;
            }
        }

        return nbvertices;
    }

    /// <summary>The shape with its edges fused (performed on demand), frozen.</summary>
    public Shape Shape()
    {
        Shape shape = InternalShape();
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary>The shape with its edges fused, without freezing it (ported code).</summary>
    internal Shape InternalShape()
    {
        Standard_NullObject_Raise_if(_shape.IsNull, "FuseEdges : No Shape");

        if (!_edgesDone)
        {
            BuildListEdges();
        }

        if (!_resultEdgesDone)
        {
            BuildListResultEdges();
        }

        if (!_shapeDone)
        {
            Perform();
        }

        return _shape;
    }

    // Build the all the lists of edges that are to be fused
    private void BuildListEdges()
    {
        //--------------------------------------------------------
        // Step One : Build the map ancestors
        //--------------------------------------------------------

        // Clear the maps
        _mapLstEdg.Clear();
        _mapVerLstEdg.Clear();
        _mapEdgLstFac.Clear();

        Exploration.MapShapesAndUniqueAncestors(_shape, ShapeType.Vertex, ShapeType.Edge, _mapVerLstEdg);
        Exploration.MapShapesAndAncestors(_shape, ShapeType.Edge, ShapeType.Face, _mapEdgLstFac);

        int iEdg;
        var mapUniqEdg = new HashSet<Shape>(ShapeHasher.Instance);

        // for each edge of myMapEdgLstFac
        for (iEdg = 1; iEdg <= _mapEdgLstFac.Count; iEdg++)
        {
            Shape edgecur = _mapEdgLstFac.KeyAt(iEdg - 1);
            var lstEdg = new NCollection_List<Shape>();

            // if edge not already treated
            if (!mapUniqEdg.Contains(edgecur)
                && (edgecur.Orientation == Forward || edgecur.Orientation == Reversed))
            {
                if (_avoidEdg.Contains(edgecur))
                {
                    continue; // edge is not allowed to be fused
                }
                BuildListConnexEdge(edgecur, mapUniqEdg, lstEdg);
                if (lstEdg.Extent() > 1)
                {
                    _nbConnexEdge++;
                    _mapLstEdg[_nbConnexEdge] = lstEdg;
                }
            }
        }

        _edgesDone = true;
        _resultEdgesDone = false;
    }

    private void BuildListResultEdges()
    {
        // if we have edges to fuse
        if (_mapLstEdg.Count > 0)
        {
            Vertex vf, vl;
            Curve3D C;
            Placement loc;
            double f, l;
            Edge newEdge;

            _mapEdg.Clear();

            foreach (KeyValuePair<int, NCollection_List<Shape>> itLstEdg in _mapLstEdg)
            {
                int iLst = itLstEdg.Key;
                NCollection_List<Shape> lmapEdg = _mapLstEdg.GetValue(iLst);

                Edge oldEdge = ShapeCasts.Edge(lmapEdg.First());

                // the first edge of the list will be replaced by the result fusion edge
                if (oldEdge.Orientation == Reversed)
                {
                    vl = Exploration.FirstVertex(ShapeCasts.Edge(lmapEdg.First()), true);
                    vf = Exploration.LastVertex(ShapeCasts.Edge(lmapEdg.Last()), true);
                }
                else
                {
                    vf = Exploration.FirstVertex(ShapeCasts.Edge(lmapEdg.First()), true);
                    vl = Exploration.LastVertex(ShapeCasts.Edge(lmapEdg.Last()), true);
                }
                C = Tool.Curve(oldEdge, out loc, out f, out l)!;

                if (!loc.IsIdentity)
                {
                    C = (Curve3D)C.Transformed(loc.Transformation);
                }
                // if the curve is trimmed we get the basis curve to fit the new vertices
                // otherwise the makeedge will fail.
                if (C.GetType() == typeof(TrimmedCurve3D))
                {
                    C = ((TrimmedCurve3D)C).BasisCurve;
                }

                if (_concatBSpl)
                {
                    // Prepare common BSpline curve
                    if (C.GetType() == typeof(BSplineCurve3D))
                    {
                        var edgIter = new NCollection_List<Shape>.Iterator(lmapEdg);
                        var tc = new TrimmedCurve3D(C, f, l);
                        var concat = new CompCurveToBSplineCurve(tc);

                        edgIter.Next();
                        for (; edgIter.More(); edgIter.Next())
                        {
                            Curve3D c = Tool.Curve(ShapeCasts.Edge(edgIter.Value()), out f, out l)!;
                            tc = new TrimmedCurve3D(c, f, l);
                            if (!concat.Add(tc, global::Kerf.Engine.Foundation.Precision.Confusion()))
                            {
                                // cannot merge curves
                                throw new ConstructionErrorException("FuseEdges : Concatenation failed");
                            }
                        }
                        // the concatenation is discarded at once: read it unfrozen (MakeClosedCurve may change it)
                        C = concat.InternalBSplineCurve()!;
                    }
                }

                var me = new LibraryMakeEdge();

                bool isBSpline = C.GetType() == typeof(BSplineCurve3D);

                if (vf.IsSame(vl) && isBSpline)
                {
                    // closed edge
                    f = C.FirstParameter;
                    l = C.LastParameter;
                    Point3D aPf = C.Value(f);
                    Point3D aPl = C.Value(l);
                    if (aPf.DistanceTo(aPl) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        throw new ConstructionErrorException("FuseEdges : Curve must be closed");
                    }
                    Point3D pf = Tool.Pnt(vf);
                    if (pf.DistanceTo(aPf) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        MakeClosedCurve(ref C, pf, ref f, ref l);
                    }
                    //
                    me.Init(C, vf, vl, f, l);
                    if (!me.IsDone)
                    {
                        throw new ConstructionErrorException("FuseEdges : MakeEdge failed for closed curve");
                    }
                }
                else
                {
                    me.Init(C, vf, vl);
                }
                //      BRepLib_MakeEdge ME(C,VF,VL);

                if (!me.IsDone)
                {
                    // the MakeEdge has fails, one reason could be that the new Vertices are outside
                    // the curve which is not infinite and limited to old vertices
                    // we try to use ExtendCurveToPoint, then rebuild the NewEdge

                    BoundedCurve3D? extC = C.Copy() as BoundedCurve3D;
                    if (extC is not null)
                    {
                        Point3D pf = Tool.Pnt(vf);
                        Point3D pl = Tool.Pnt(vl);
                        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref extC, pf, 1, false);
                        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref extC, pl, 1, true);

                        me.Init(extC, vf, vl);
                        if (!me.IsDone)
                        {
                            throw new ConstructionErrorException("FuseEdges : Fusion failed");
                        }
                    }
                    else
                    {
                        throw new ConstructionErrorException("FuseEdges : Fusion failed");
                    }
                }

                newEdge = me.InternalEdge();

                if (UpdatePCurve(oldEdge, newEdge, lmapEdg))
                {
                    _mapEdg[iLst] = newEdge;
                }
            }

            _resultEdgesDone = true;
        }
    }

    /// <summary>Performs the fusion (on demand from the accessors).</summary>
    public void Perform()
    {
        if (!_resultEdgesDone)
        {
            BuildListResultEdges();
        }

        // if we have fused edges
        if (_mapEdg.Count > 0)
        {
            NCollection_List<Shape> emptyList = new(), edgeToSubs = new();
            var bsub = new Substitution();

            foreach (KeyValuePair<int, NCollection_List<Shape>> itLstEdg in _mapLstEdg)
            {
                int iLst = itLstEdg.Key;
                if (!_mapEdg.ContainsKey(iLst))
                {
                    continue;
                }
                NCollection_List<Shape> lmapEdg = _mapLstEdg.GetValue(iLst);
                var itEdg = new NCollection_List<Shape>.Iterator();

                edgeToSubs.Clear();
                Edge oldEdge = ShapeCasts.Edge(lmapEdg.First());

                edgeToSubs.Append(_mapEdg.GetValue(iLst));
                bsub.Substitute(oldEdge, edgeToSubs);

                itEdg.Initialize(lmapEdg);

                // the other edges of the list will be removed
                while (itEdg.More())
                {
                    if (!oldEdge.IsSame(ShapeCasts.Edge(itEdg.Value())))
                    {
                        bsub.Substitute(itEdg.Value(), emptyList);
                    }
                    itEdg.Next();
                }
            }

            // perform the effective substitution
            bsub.Build(_shape);

            // before copying the resulting shape, map the modified faces into myMapFaces
            var exp = new Explorer(_shape, ShapeType.Face);

            for (; exp.More(); exp.Next())
            {
                Shape facecur = exp.Current();
                if (bsub.IsCopied(facecur))
                {
                    _mapFaces[facecur] = bsub.InternalCopy(facecur).First();
                }
            }

            if (bsub.IsCopied(_shape))
            {
                _shape = bsub.InternalCopy(_shape).First();
            }
        }

        _shapeDone = true;
    }

    // giving one edge, build the list of connex edges which have vertices that have only two connex edges. All the
    // edges that are added to the list must be added also to the mapUniq, in order for the caller to not treat again
    // these edges. This list is always oriented in the "Forward" direction.
    private void BuildListConnexEdge(in Shape edge,
                                     HashSet<Shape> mapUniq,
                                     NCollection_List<Shape> lstEdg)
    {
        Vertex vf, vl;

        vl = Exploration.LastVertex(ShapeCasts.Edge(edge), true);
        Shape edgeconnex = new();
        Shape edgecur = edge;
        lstEdg.Clear();
        lstEdg.Append(edgecur);
        mapUniq.Add(edgecur);
        Orientation ori2;

        // we first build the list of edges connex to edgecur by looking from the last Vertex VL
        while (NextConnexEdge(vl, edgecur, ref edgeconnex))
        {
            if (mapUniq.Contains(edgeconnex))
            {
                break;
            }
            lstEdg.Append(edgeconnex);
            edgecur = edgeconnex;
            // here take care about internal or external edges. It is non-sense to build
            // the connex list with such edges.
            ori2 = edgecur.Orientation;
            if (ori2 == External || ori2 == Internal)
            {
                break;
            }
            vl = Exploration.LastVertex(ShapeCasts.Edge(edgecur), true);
            mapUniq.Add(edgecur);
        }

        edgecur = edge;
        vf = Exploration.FirstVertex(ShapeCasts.Edge(edge), true);

        // then we build the list of edges connex to edgecur by looking from the first Vertex VF
        while (NextConnexEdge(vf, edgecur, ref edgeconnex))
        {
            if (mapUniq.Contains(edgeconnex))
            {
                break;
            }
            lstEdg.Prepend(edgeconnex);
            edgecur = edgeconnex;
            // here take care about internal or external edges. It is non-sense to build
            // the connex list with such edges.
            ori2 = edgecur.Orientation;
            if (ori2 == External || ori2 == Internal)
            {
                break;
            }
            vf = Exploration.FirstVertex(ShapeCasts.Edge(edgecur), true);
            mapUniq.Add(edgecur);
        }
    }

    // Look for an edge connex to theEdge at theVertex. the connex edge must satisfies the following criteria :
    //   * theVertex must have exactly 2 connex edges.
    //   * the 2 connex edges must have exactly the 2 same connex faces
    //   * the 2 connex edges must lie on the same support.
    // theEdgeConnex is written only when theVertex has two edges, as in OCCT.
    private bool NextConnexEdge(in Vertex vertex, in Shape edge, ref Shape edgeConnex)
    {
        NCollection_List<Shape> lmapEdg = _mapVerLstEdg.GetValue(vertex);
        bool hasConnex = true;
        var itEdg = new NCollection_List<Shape>.Iterator();
        var itFac1 = new NCollection_List<Shape>.Iterator();
        var itFac2 = new NCollection_List<Shape>.Iterator();

        // 1st condition
        if (lmapEdg.Extent() == 2)
        {
            itEdg.Initialize(lmapEdg);
            edgeConnex = itEdg.Value();
            if (edge.IsSame(edgeConnex))
            {
                itEdg.Next();
                edgeConnex = itEdg.Value();
            }

            if (_avoidEdg.Contains(edgeConnex))
            {
                hasConnex = false; // edge is not allowed to be fused
            }

            // 2nd condition
            if (hasConnex)
            {
                NCollection_List<Shape> lmapFac1 = _mapEdgLstFac.GetValue(edge);
                NCollection_List<Shape> lmapFac2 = _mapEdgLstFac.GetValue(edgeConnex);

                if (lmapFac1.Extent() == lmapFac2.Extent() && lmapFac1.Extent() < 3)
                {
                    itFac1.Initialize(lmapFac1);

                    // for each face in LmapFac1 we look in LmapFac2 if it exists
                    while (itFac1.More() && hasConnex)
                    {
                        Shape face1 = itFac1.Value();
                        for (itFac2.Initialize(lmapFac2); itFac2.More(); itFac2.Next())
                        {
                            Shape face2 = itFac2.Value();
                            hasConnex = false;
                            if (face1.IsSame(face2))
                            {
                                hasConnex = true;
                                break;
                            }
                        }
                        itFac1.Next();
                    }

                    // 3rd condition : same support
                    if (hasConnex)
                    {
                        hasConnex = SameSupport(ShapeCasts.Edge(edge), ShapeCasts.Edge(edgeConnex));
                    }
                }
                else
                {
                    hasConnex = false;
                }
            }
        }
        else
        {
            hasConnex = false;
        }

        return hasConnex;
    }

    private bool SameSupport(in Edge e1, in Edge e2)
    {
        if (e1.IsNull || e2.IsNull)
        {
            return false;
        }

        Curve3D? c1, c2;
        Placement loc;
        double f1, l1, f2, l2;
        Type typC1, typC2;

        c1 = Tool.Curve(e1, out loc, out f1, out l1);
        // modified by NIZNHY-PKV Mon Nov 15 16:24:10 1999
        // degenerated edges has no 3D curve
        if (c1 is null)
        {
            return false;
        }

        if (!loc.IsIdentity)
        {
            Geometry3D gg1 = c1.Transformed(loc.Transformation);
            c1 = (Curve3D)gg1;
        }
        c2 = Tool.Curve(e2, out loc, out f2, out l2);
        // modified by NIZNHY-PKV Mon Nov 15 16:24:38 1999
        // degenerated edges has no 3D curve
        if (c2 is null)
        {
            return false;
        }

        if (!loc.IsIdentity)
        {
            Geometry3D gg2 = c2.Transformed(loc.Transformation);
            c2 = (Curve3D)gg2;
        }

        typC1 = c1.GetType();
        typC2 = c2.GetType();

        if (typC1 == typeof(TrimmedCurve3D))
        {
            c1 = ((TrimmedCurve3D)c1).BasisCurve;
            typC1 = c1.GetType();
        }

        if (typC2 == typeof(TrimmedCurve3D))
        {
            c2 = ((TrimmedCurve3D)c2).BasisCurve;
            typC2 = c2.GetType();
        }

        if (typC1 != typC2)
        {
            return false;
        }

        if (typC1 != typeof(LineCurve3D) && typC1 != typeof(CircleCurve3D)
            && typC1 != typeof(EllipseCurve3D) && typC1 != typeof(BSplineCurve3D)
            && typC1 != typeof(BezierCurve3D))
        {
            return false;
        }

        // On a presomption de confusion
        double tollin = global::Kerf.Engine.Foundation.Precision.Confusion();
        double tolang = global::Kerf.Engine.Foundation.Precision.Angular();
        if (typC1 == typeof(LineCurve3D))
        {
            Line3D li1 = ((LineCurve3D)c1).Line;
            Line3D li2 = ((LineCurve3D)c2).Line;
            Direction3D dir1 = li1.Direction;
            Direction3D dir2 = li2.Direction;

            if (dir1.IsParallel(dir2, tolang))
            {
                // on verifie que l'on n'a pas de cas degenere. Par exemple E1 et E2 connexes
                // mais bouclant l'un sur l'autre (cas tres rare)
                Point3D pf1 = Tool.Pnt(Exploration.FirstVertex(e1, true));
                Point3D pl1 = Tool.Pnt(Exploration.LastVertex(e1, true));
                Point3D pf2 = Tool.Pnt(Exploration.FirstVertex(e2, true));
                Point3D pl2 = Tool.Pnt(Exploration.LastVertex(e2, true));
                return pl1.DistanceTo(pf2) >= tollin || pl2.DistanceTo(pf1) >= tollin;
            }
            return false;
        }
        else if (typC1 == typeof(CircleCurve3D))
        {
            Circle3D ci1 = ((CircleCurve3D)c1).Circle;
            Circle3D ci2 = ((CircleCurve3D)c2).Circle;
            return Math.Abs(ci1.Radius - ci2.Radius) <= tollin
                   && ci1.Location.SquareDistanceTo(ci2.Location) <= tollin * tollin
                   && ci1.Axis.IsParallel(ci2.Axis, tolang);
        }
        else if (typC1 == typeof(EllipseCurve3D))
        {
            Ellipse3D ci1 = ((EllipseCurve3D)c1).Ellipse;
            Ellipse3D ci2 = ((EllipseCurve3D)c2).Ellipse;

            return Math.Abs(ci1.MajorRadius - ci2.MajorRadius) <= tollin
                   && Math.Abs(ci1.MinorRadius - ci2.MinorRadius) <= tollin
                   && ci1.Location.SquareDistanceTo(ci2.Location) <= tollin * tollin
                   && ci1.Axis.IsParallel(ci2.Axis, tolang);
        }
        else if (typC1 == typeof(BSplineCurve3D))
        {
            if (_concatBSpl)
            {
                // Check G1 continuity
                Point3D aPf1, aPl1, aPf2, aPl2;
                Vector3D df1, dl1, df2, dl2;

                c1.D1(f1, out aPf1, out df1);
                c1.D1(l1, out aPl1, out dl1);

                c2.D1(f2, out aPf2, out df2);
                c2.D1(l2, out aPl2, out dl2);

                if (aPl1.DistanceTo(aPf2) <= tollin && dl1.IsParallel(df2, tolang))
                {
                    return true;
                }
                if (aPl2.DistanceTo(aPf1) <= tollin && dl2.IsParallel(df1, tolang))
                {
                    return true;
                }
                if (aPf1.DistanceTo(aPf2) <= tollin && df1.IsParallel(df2, tolang))
                {
                    return true;
                }
                if (aPl1.DistanceTo(aPl2) <= tollin && dl1.IsParallel(dl2, tolang))
                {
                    return true;
                }
            }
            // we must ensure that before fuse two bsplines, the end of one curve does not
            // corresponds to the beginning of the second.
            // we could add a special treatment for periodic bspline. This is not done for the moment.
            if (Math.Abs(f2 - l1) > tollin && Math.Abs(f1 - l2) > tollin)
            {
                return false;
            }

            BSplineCurve3D b1 = (BSplineCurve3D)c1;
            BSplineCurve3D b2 = (BSplineCurve3D)c2;

            int nbpoles = b1.PoleCount;
            if (nbpoles != b2.PoleCount)
            {
                return false;
            }

            int nbknots = b1.KnotCount;
            if (nbknots != b2.KnotCount)
            {
                return false;
            }

            NCollection_Array1<Point3D> p1 = b1.InternalPoles();
            NCollection_Array1<Point3D> p2 = b2.InternalPoles();

            double tol3d = Tool.Tolerance(e1);
            for (int p = 1; p <= nbpoles; p++)
            {
                if ((p1[p]).DistanceTo(p2[p]) > tol3d)
                {
                    return false;
                }
            }

            NCollection_Array1<double> k1 = b1.InternalKnots();
            NCollection_Array1<double> k2 = b2.InternalKnots();

            NCollection_Array1<int> m1 = b1.InternalMultiplicities();
            NCollection_Array1<int> m2 = b2.InternalMultiplicities();

            for (int k = 1; k <= nbknots; k++)
            {
                // OCCT compares the signed difference only: a knot of B1 smaller than B2's by any amount passes.
                if ((k1[k] - k2[k]) > tollin)
                {
                    return false;
                }
                if (Math.Abs(m1[k] - m2[k]) > tollin)
                {
                    return false;
                }
            }

            if (!b1.IsRational)
            {
                if (b2.IsRational)
                {
                    return false;
                }
            }
            else
            {
                if (!b2.IsRational)
                {
                    return false;
                }
            }

            if (b1.IsRational)
            {
                NCollection_Array1<double> w1 = b1.InternalWeightsArray();
                NCollection_Array1<double> w2 = b2.InternalWeightsArray();

                for (int w = 1; w <= nbpoles; w++)
                {
                    if (Math.Abs(w1[w] - w2[w]) > tollin)
                    {
                        return false;
                    }
                }
            }
            return true;
        }
        else if (typC1 == typeof(BezierCurve3D))
        {
            // we must ensure that before fuse two bezier, the end of one curve does not
            // corresponds to the beginning of the second.
            if (Math.Abs(f2 - l1) > tollin && Math.Abs(f1 - l2) > tollin)
            {
                return false;
            }

            BezierCurve3D b1 = (BezierCurve3D)c1;
            BezierCurve3D b2 = (BezierCurve3D)c2;

            int nbpoles = b1.PoleCount;
            if (nbpoles != b2.PoleCount)
            {
                return false;
            }

            NCollection_Array1<Point3D> p1 = b1.InternalPoles();
            NCollection_Array1<Point3D> p2 = b2.InternalPoles();

            for (int p = 1; p <= nbpoles; p++)
            {
                if ((p1[p]).DistanceTo(p2[p]) > tollin)
                {
                    return false;
                }
            }

            if (!b1.IsRational)
            {
                if (b2.IsRational)
                {
                    return false;
                }
            }
            else
            {
                if (!b2.IsRational)
                {
                    return false;
                }
            }

            if (b1.IsRational)
            {
                NCollection_Array1<double> w1 = b1.InternalWeightsArray();
                NCollection_Array1<double> w2 = b2.InternalWeightsArray();

                for (int w = 1; w <= nbpoles; w++)
                {
                    if (Math.Abs(w1[w] - w2[w]) > tollin)
                    {
                        return false;
                    }
                }
            }
            return true;
        }
        return false;
    }

    private bool UpdatePCurve(in Edge oldEdge, in Edge newEdge, NCollection_List<Shape> lstEdg)
    {
        // get the pcurve of edge to substitute (theOldEdge)
        // using CurveOnSurface with Index syntax, so we can update the pcurve
        // on all the faces
        BrepBuilder b = new();
        Curve2D? curv2d = null;
        Surface? surf = null;
        Placement loc = new(), locbid;
        double ef, el, cf = 0.0, cl = 0.0;
        int iedg = 1;

        // take care that we want only Pcurve that maps on the surface where the 3D edges lies.
        NCollection_List<Shape> lmapFac = _mapEdgLstFac.GetValue(oldEdge);

        Tool.CurveOnSurface(oldEdge, ref curv2d, ref surf, ref loc, ref cf, ref cl, iedg);

        bool pcurveRebuilt = false;

        while (curv2d is not null)
        {
            // we look for a face that contains the same surface as the one that cames
            // from CurveOnSurface
            bool sameSurf = false;
            var itFac = new NCollection_List<Shape>.Iterator();

            for (itFac.Initialize(lmapFac); itFac.More(); itFac.Next())
            {
                Shape face = itFac.Value();
                Surface? s = Tool.Surface(ShapeCasts.Face(face), out locbid);
                if (ReferenceEquals(s, surf))
                {
                    sameSurf = true;
                    break;
                }
            }

            if (sameSurf)
            {
                Tool.Range(newEdge, out ef, out el);

                // modified by NIZNHY-PKV Mon Nov 15 14:59:48 1999 _from
                Edge fEdge = oldEdge;
                fEdge.Orient(Forward);

                // take care if the edge is on the closing curve of a closed surface. In that case
                // we get the second pcurve by reversing the edge and calling again CurveOnSurface method

                Tool.CurveOnSurface(fEdge, ref curv2d, ref surf, ref loc, ref cf, ref cl, iedg);
                if (Tool.IsClosed(oldEdge, surf, loc))
                {
                    fEdge.Reverse();
                    Face fFace = ShapeCasts.Face(itFac.Value());
                    fFace.Orient(Forward);
                    Curve2D curv2dR = Tool.CurveOnSurface(fEdge, fFace, out cf, out cl)!;
                    if (curv2d!.GetType() == typeof(TrimmedCurve2D))
                    {
                        curv2d = ((TrimmedCurve2D)curv2d).BasisCurve;
                    }
                    if (curv2dR.GetType() == typeof(TrimmedCurve2D))
                    {
                        curv2dR = ((TrimmedCurve2D)curv2dR).BasisCurve;
                    }

                    b.UpdateEdge(newEdge, curv2d, curv2dR, surf, loc, Tool.Tolerance(newEdge));
                }
                else
                {
                    // update the new edge
                    if (curv2d!.GetType() == typeof(TrimmedCurve2D))
                    {
                        curv2d = ((TrimmedCurve2D)curv2d).BasisCurve;
                    }
                    double f, l;
                    f = curv2d.FirstParameter;
                    l = curv2d.LastParameter;
                    if (l - f + 2.0 * Epsilon(l - f) < el - ef)
                    {
                        BoundedCurve2D? bcurve = curv2d as BoundedCurve2D;
                        if (bcurve is null)
                        {
                            bcurve = new TrimmedCurve2D(curv2d, cf, cl);
                        }
                        var concat = new CompCurveToBSplineCurve2D(bcurve);
                        var iter = new NCollection_List<Shape>.Iterator(lstEdg);
                        iter.Next();
                        for (; iter.More(); iter.Next())
                        {
                            Edge e = ShapeCasts.Edge(iter.Value());
                            Curve2D c = Tool.CurveOnSurface(e, surf, loc, out double first, out double last)!;
                            BoundedCurve2D? BC = c as BoundedCurve2D;
                            if (BC is null)
                            {
                                BC = new TrimmedCurve2D(c, first, last);
                            }
                            if (!concat.Add(BC, global::Kerf.Engine.Foundation.Precision.PConfusion()))
                            {
                                // cannot merge pcurves
                                return false;
                            }
                        }
                        // the concatenation is discarded at once: read it unfrozen (its knots may be reset below)
                        curv2d = concat.InternalBSplineCurve()!;

                        // check that new curve 2d is same range
                        double first2 = curv2d.FirstParameter;
                        double last2 = curv2d.LastParameter;
                        if (Math.Abs(first2 - ef) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                            || Math.Abs(last2 - el) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            BSplineCurve2D bc = (BSplineCurve2D)curv2d;
                            NCollection_Array1<double> knots = new(bc.InternalKnots());
                            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(ef, el, knots);
                            bc.SetKnots(knots);
                        }
                        pcurveRebuilt = true;
                    }

                    b.UpdateEdge(newEdge, curv2d, surf, loc, Tool.Tolerance(newEdge));
                }

                // the old pcurve range is cf,cl. The new 3d edge range is ef,el. if we want
                // the pcurve to be samerange we must adapt the parameter of the edge. In general
                // cases cf=ef and cl=el expect for periodic curve if the new edge is going over
                // the value 0.
                if (!pcurveRebuilt)
                {
                    if (oldEdge.Orientation == Reversed)
                    {
                        b.Range(newEdge, cl - el + ef, cl);
                    }
                    else
                    {
                        b.Range(newEdge, cf, cf + el - ef);
                    }
                }
            }

            // get next pcurve
            iedg++;
            Tool.CurveOnSurface(oldEdge, ref curv2d, ref surf, ref loc, ref cf, ref cl, iedg);
        }

        if (pcurveRebuilt)
        {
            // force same parameter
            b.SameParameter(newEdge, false);
            BrepLibrary.SameParameter(newEdge, Tool.Tolerance(newEdge));
        }

        return true;
    }
}
