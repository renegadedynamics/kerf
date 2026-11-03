// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_UnifySameDomain.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_UnifySameDomain.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// The .cxx is split by subject: this file holds the class, the unification of faces (UnifyFaces, IntUnifyFaces),
// UnifyEdges, Build and FillHistory; ShapeUpgrade_UnifySameDomain.Faces.cs the file-static functions of the faces;
// ShapeUpgrade_UnifySameDomain.Edges.cs MergeEdges, MergeSeq, MergeSubSeq, UnionPCurves and theirs.

// OCCT's typedefs of the class
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using DataMapOfFacePlane = System.Collections.Generic.Dictionary<Kerf.Topology.Shape, Kerf.Geometry.PlaneSurface>;
using DataMapOfShapeMapOfShape = System.Collections.Generic.Dictionary<Kerf.Topology.Shape, System.Collections.Generic.HashSet<Kerf.Topology.Shape>>;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Unifies the faces and edges of a shape that lie on the same geometry: a group of neighbouring faces (edges) lying
/// on coincident surfaces (curves) becomes one face (edge).
/// </summary>
/// <remarks>
/// <para>
/// Initialized by a shape and the options: UnifyFaces (try to unify all possible faces), UnifyEdges (all possible
/// edges), ConcatBSplines (merge neighbouring edges on B-spline or Bezier curves with C1 continuity at their common
/// vertices into one edge). The input may be any shape holding faces or edges; the structure of compsolids, solids,
/// shells and wires is kept: faces of two shells sharing an edge aren't unified, faces of different compounds are.
/// </para>
/// <para>
/// A working object (decision 21). <see cref="Build"/> edits the shape it works on in place, as OCCT's does (even in
/// the safe input mode OCCT adds pcurves to the input's edges), in a working overlay (Kerf unit 10H4) that takes the
/// writes into frozen shapes (any shape a caller holds), so the caller's shape never changes, whatever
/// <see cref="SetSafeInputMode"/> says; the algorithm runs with the mode given, so its results are OCCT's for that
/// mode. At the end the result holds new TShapes for what changed and the caller's own sub-shapes for the rest: a shape
/// needing nothing gives back the input itself. The operation's own TShapes (ported code's), and every shape inside
/// another operation's overlay, are worked on in place, as in OCCT.
/// </para>
/// <para>
/// <see cref="Shape"/> and <see cref="History()"/> freeze the result (the history lists sub-shapes of the input and of
/// the result); ported code that goes on building reads <see cref="InternalShape"/> and
/// <see cref="InternalHistory"/>. The history answers for the caller's sub-shapes, directly; a sub-shape changed in
/// place (a pcurve added) is modified into what stands for it now, where OCCT's history has no entry.
/// </para>
/// </remarks>
public partial class UnifySameDomain : Transient
{
    private Shape                  _initShape;
    private double                        _linTol;
    private double                        _angTol;
    private bool                          _unifyFaces;
    private bool                          _unifyEdges;
    private bool                          _concatBSplines;
    private bool                          _allowInternal;
    private bool                          _safeInputMode;
    private Shape                  _shape;
    private readonly ReShapeBuilder   _context;
    private readonly HashSet<Shape> _keepShapes = new(ShapeHasher.Instance);
    private readonly DataMapOfFacePlane   _facePlaneMap = new(ShapeHasher.Instance);
    private readonly IndexedMap<Shape, NCollection_List<Shape>> _eFmap =
      new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, Shape> _faceNewFace = new(ShapeHasher.Instance);

    private History? _history; //!< The history.

    // OCCT.NET: the history FillHistory made over the shape worked on, merged into myHistory by Build once its working
    // overlay is finished (composed with the changes made in place).
    private History? _usdHistory;

    /// <summary>Empty constructor.</summary>
    public UnifySameDomain()
    {
        _initShape      = new Shape();
        _linTol         = global::Kerf.Engine.Foundation.Precision.Confusion();
        _angTol         = global::Kerf.Engine.Foundation.Precision.Angular();
        _unifyFaces     = true;
        _unifyEdges     = true;
        _concatBSplines = false;
        _allowInternal  = false;
        _safeInputMode  = true;
        _shape          = new Shape();
        _history        = new History();
        _context        = new ReShapeBuilder();
    }

    /// <summary>Constructor defining the input shape and the flags. It does not perform unification.</summary>
    public UnifySameDomain(in Shape aShape,
                                        bool            unifyEdges     = true,
                                        bool            unifyFaces     = true,
                                        bool            concatBSplines = false)
    {
        _initShape      = aShape;
        _linTol         = global::Kerf.Engine.Foundation.Precision.Confusion();
        _angTol         = global::Kerf.Engine.Foundation.Precision.Angular();
        _unifyFaces     = unifyFaces;
        _unifyEdges     = unifyEdges;
        _concatBSplines = concatBSplines;
        _allowInternal  = false;
        _safeInputMode  = true;
        _shape          = aShape;
        _history        = new History();
        _context        = new ReShapeBuilder();
    }

    /// <summary>
    /// Initializes with a shape and the flags. It does not perform unification. If you intend to nullify the history
    /// place holder, do it after initialization.
    /// </summary>
    public void Initialize(in Shape aShape,
                           bool            unifyEdges     = true,
                           bool            unifyFaces     = true,
                           bool            concatBSplines = false)
    {
        _initShape      = aShape;
        _shape          = aShape;
        _unifyEdges     = unifyEdges;
        _unifyFaces     = unifyFaces;
        _concatBSplines = concatBSplines;

        _context.Clear();
        _keepShapes.Clear();
        _facePlaneMap.Clear();
        _eFmap.Clear();
        _faceNewFace.Clear();
        _history?.Clear();
    }

    /// <summary>
    /// Sets the flag defining whether it is allowed to create internal edges inside merged faces in the case of
    /// non-manifold topology. Without this flag merging through multi connected edge is forbidden. Default value is
    /// false.
    /// </summary>
    public void AllowInternalEdges(bool value)
    {
        _allowInternal = value;
    }

    /// <summary>
    /// Sets the shape for avoid merging of the faces/edges. This shape can be vertex or edge. If the shape is a vertex
    /// it forbids merging of connected edges. If the shape is a edge it forbids merging of connected faces. This
    /// method can be called several times to keep several shapes.
    /// </summary>
    public void KeepShape(in Shape shape)
    {
        if (shape.Type == ShapeType.Edge || shape.Type == ShapeType.Vertex)
        {
            _keepShapes.Add(shape);
        }
    }

    /// <summary>
    /// Sets the map of shapes for avoid merging of the faces/edges. It allows passing a ready to use map instead of
    /// calling many times the method KeepShape.
    /// </summary>
    public void KeepShapes(HashSet<Shape> shapes)
    {
        foreach (Shape shape in shapes)
        {
            if (shape.Type == ShapeType.Edge || shape.Type == ShapeType.Vertex)
            {
                _keepShapes.Add(shape);
            }
        }
    }

    /// <summary>
    /// Sets the flag defining the behavior of the algorithm regarding modification of input shape. If this flag is
    /// equal to True then the input (original) shape can't be modified during modification process. Default value is
    /// true.
    /// </summary>
    /// <remarks>
    /// A caller's shape is never modified whatever the flag (it is copied, see the class), which then only selects
    /// OCCT's code path on the copy: false builds pcurves on planes in place and lets ShapeFix_Face work without the
    /// context, as OCCT's does.
    /// </remarks>
    public void SetSafeInputMode(bool value)
    {
        _safeInputMode = value;
    }

    /// <summary>
    /// Sets the linear tolerance. It plays the role of chord error when taking decision about merging of shapes.
    /// Default value is Precision::Confusion().
    /// </summary>
    public void SetLinearTolerance(double value)
    {
        _linTol = value;
    }

    /// <summary>
    /// Sets the angular tolerance. If two shapes form a connection angle greater than this value they will not be
    /// merged. Default value is Precision::Angular().
    /// </summary>
    public void SetAngularTolerance(double value)
    {
        _angTol = (value < global::Kerf.Engine.Foundation.Precision.Angular() ? global::Kerf.Engine.Foundation.Precision.Angular() : value);
    }

    /// <summary>The resulting shape, frozen.</summary>
    public Shape Shape()
    {
        _shape.TShape()?.FreezeGraph();
        return _shape;
    }

    /// <summary>The resulting shape, as <see cref="Shape"/>, without freezing it (ported code).</summary>
    internal Shape InternalShape() => _shape;

    /// <summary>
    /// The history of the processed shapes, null if it isn't collected. The result is frozen first, so every shape
    /// the history lists is frozen.
    /// </summary>
    public History? History()
    {
        _shape.TShape()?.FreezeGraph();
        return _history;
    }

    /// <summary>
    /// Sets the place holder of the history (OCCT's assignment to the non-const <c>History()</c>): null to collect no
    /// history. The history of each <see cref="Build"/> is merged into it.
    /// </summary>
    public void History(History? history)
    {
        _history = history;
    }

    /// <summary>The history, as <see cref="History()"/>, without freezing the result (ported code).</summary>
    internal History? InternalHistory() => _history;

    /// <summary>Performs unification and builds the resulting shape.</summary>
    public void Build()
    {
        // Kerf: the working overlay (unit 10H4; decision 583): OCCT writes into the shape it works on (pcurves added to
        // the edges of the faces it unifies, even in the safe input mode)
        using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(_initShape, _context);
        BuildCore();

        if (overlay is not null)
        {
            _shape = overlay.Finish(_shape);
        }
        if (_usdHistory is not null)
        {
            // Merge the history of the operation into global history
            _history!.Merge(overlay is null ? _usdHistory : overlay.Composed(_usdHistory));
            _usdHistory = null;
        }
    }

    // OCCT's Build, on the shape worked on
    private void BuildCore()
    {
        Exploration.MapShapesAndAncestors(_initShape, ShapeType.Edge, ShapeType.Face, _eFmap);

        if (_unifyFaces)
        {
            UnifyFaces();
        }
        if (_unifyEdges)
        {
            UnifyEdges();
        }

        // Fill the history of modifications during the operation
        FillHistory();
    }

    /// <summary>This method makes if possible a common face from each group of faces lying on coincident surfaces.</summary>
    protected void UnifyFaces()
    {
        // creating map of edge faces for the whole shape
        var gMapEdgeFaces =
          new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);

        var faceMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(_shape, ShapeType.Face, faceMap);
        for (int i = 0; i < faceMap.Count; i++)
        {
            Exploration.MapShapesAndAncestors(faceMap[i], ShapeType.Edge, ShapeType.Face, gMapEdgeFaces);
        }

        // creating map of face shells for the whole shape to avoid
        // unification of faces belonging to the different shells
        var gMapFaceShells = new DataMapOfShapeMapOfShape(ShapeHasher.Instance);
        for (var exp = new Explorer(_shape, ShapeType.Shell); exp.More(); exp.Next())
        {
            Shape shell = exp.Current();
            for (var itF = new Iterator(shell); itF.More(); itF.Next())
            {
                Shape f = itF.Value();
                if (gMapFaceShells.TryGetValue(f, out HashSet<Shape>? pShells))
                {
                    pShells.Add(shell);
                }
                else
                {
                    gMapFaceShells.SetValueRef(f, new HashSet<Shape>(ShapeHasher.Instance)).Add(shell);
                }
            }
        }

        // creating map of free boundaries
        var freeBoundMap = new HashSet<Shape>(ShapeHasher.Instance);
        // look at only shells not belonging to solids
        var explo = new Explorer(_shape, ShapeType.Shell, ShapeType.Solid);
        for (; explo.More(); explo.Next())
        {
            Shape shell = explo.Current();
            var eFmap = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(shell, ShapeType.Edge, ShapeType.Face, eFmap);
            for (int ii = 0; ii < eFmap.Count; ii++)
            {
                Edge                    edge    = ShapeCasts.Edge(eFmap.KeyAt(ii));
                NCollection_List<Shape> faceList = eFmap.ValueAt(ii);
                if (!Tool.Degenerated(edge) && faceList.Extent() == 1)
                {
                    freeBoundMap.Add(edge);
                }
            }
        }

        // unify faces in each shell separately
        var exps = new Explorer();
        for (exps.Init(_shape, ShapeType.Shell); exps.More(); exps.Next())
        {
            IntUnifyFaces(exps.Current(), gMapEdgeFaces, gMapFaceShells, freeBoundMap);
        }

        // gather all faces out of shells in one compound and unify them at once
        var bb  = new BrepBuilder();
        var cmp = new Compound();
        bb.MakeCompound(ref cmp);
        int nbf = 0;
        for (exps.Init(_shape, ShapeType.Face, ShapeType.Shell); exps.More(); exps.Next(), nbf++)
        {
            bb.Add(cmp, exps.Current());
        }

        if (nbf > 0)
        {
            // No connection to shells, thus no need to pass the face-shell map
            IntUnifyFaces(cmp, gMapEdgeFaces, new DataMapOfShapeMapOfShape(ShapeHasher.Instance), freeBoundMap);
        }

        _shape = _context.InternalApply(_shape);
    }

    private void IntUnifyFaces(in Shape                                                          inpShape,
                               IndexedMap<Shape, NCollection_List<Shape>> gMapEdgeFaces,
                               DataMapOfShapeMapOfShape                                                 gMapFaceShells,
                               HashSet<Shape>                                            freeBoundMap)
    {
        // creating map of edge faces for the shape
        var mapEdgeFaces =
          new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(inpShape, ShapeType.Edge, ShapeType.Face, mapEdgeFaces);

        // map of processed shapes
        var processed = new HashSet<Shape>(ShapeHasher.Instance);

        // processing each face
        var exp = new Explorer();
        for (exp.Init(inpShape, ShapeType.Face); exp.More(); exp.Next())
        {
            Face face = ShapeCasts.Face(exp.Current());
            if (processed.Contains(face))
            {
                continue;
            }

            Surface? baseSurface = ClearRts(Tool.Surface(face));
            // Bug 33894: Prevent crash when face has no surface
            if (baseSurface is null)
            {
                continue;
            }

            // Boundary edges for the new face
            var edges        = new NCollection_Sequence<Shape>();
            var removedEdges = new NCollection_Sequence<Shape>();

            int dummy = 0;
            AddOrdinaryEdges(edges, face, ref dummy, removedEdges);

            // Faces to get unified with the current faces
            var faces = new NCollection_Sequence<Shape>();

            // Add the current face for unification
            faces.Append(face);

            // surface and location to construct result
            var                baseLocation      = new Placement();
            Orientation refFaceOrientation = face.Orientation;

            // Take original surface
            var refFace = new Face();
            var bb      = new BrepBuilder();
            bb.MakeFace(ref refFace, baseSurface, baseLocation, 0.0);
            refFace.Orient(refFaceOrientation);
            // map of edges not lying on RefFace; these edges may be updated by temporary pcurves
            var mapEdgesWithTemporaryPCurves = new HashSet<Shape>(ShapeHasher.Instance);

            double uperiod = (baseSurface.IsUPeriodic) ? baseSurface.UPeriod : 0.0;
            double vperiod = (baseSurface.IsVPeriodic) ? baseSurface.VPeriod : 0.0;

            // Get shells connected to the face (in normal cases should not be more than 2)
            HashSet<Shape>? pFShells1 =
              gMapFaceShells.TryGetValue(face, out HashSet<Shape>? fShells1) ? fShells1 : null;

            // find adjacent faces to union
            int i;
            for (i = 1; i <= edges.Length(); i++)
            {
                Edge edge = ShapeCasts.Edge(edges[i]);
                if (Tool.Degenerated(edge))
                {
                    continue;
                }

                // get connectivity of the edge in the global shape
                NCollection_List<Shape> gList = gMapEdgeFaces.GetValue(edge);
                if (!_allowInternal
                    && (gList.Extent() != 2 || _keepShapes.Contains(edge) || freeBoundMap.Contains(edge)))
                {
                    // non manifold case is not processed unless myAllowInternal
                    continue;
                }
                //
                // Get the faces connected through the edge in the current shape
                NCollection_List<Shape> list = mapEdgeFaces.GetValue(edge);
                if (list.Extent() < 2)
                {
                    continue;
                }

                // for a planar face create and store pcurve of edge on face
                // to speed up all operations
                if (!_safeInputMode && baseSurface is PlaneSurface)
                {
                    BrepLibrary.BuildPCurveForEdgeOnPlane(edge, face);
                }

                // get normal of the face to compare it with normals of other faces
                var dn1 = new Direction3D();
                //
                // take intermediate point on edge to compute the normal
                Tool.Range(edge, out double f, out double l);
                double tMid = (f + l) * .5;
                //
                bool bCheckNormals = GetNormalToSurface(face, edge, tMid, ref dn1);
                //
                // Process the faces
                var iter = new NCollection_List<Shape>.Iterator(list);
                for (; iter.More(); iter.Next())
                {
                    Face checkedFace = ShapeCasts.Face(iter.Value());
                    if (checkedFace.IsSame(face))
                    {
                        continue;
                    }

                    if (processed.Contains(checkedFace))
                    {
                        continue;
                    }

                    // Get shells connected to the checked face
                    HashSet<Shape>? pFShells2 =
                      gMapFaceShells.TryGetValue(checkedFace, out HashSet<Shape>? fShells2) ? fShells2 : null;
                    // Faces can be unified only if the shells of faces connected to
                    // these faces are the same. Otherwise, topology would be broken.
                    if (!IsSameSets(pFShells1, pFShells2))
                    {
                        continue;
                    }

                    if (bCheckNormals)
                    {
                        // get normal of checked face using the same parameter on edge
                        var dn2 = new Direction3D();
                        if (GetNormalToSurface(checkedFace, edge, tMid, ref dn2))
                        {
                            // and check if the adjacent faces are having approximately same normals
                            double angle = dn1.AngleTo(dn2);
                            if (angle > _angTol)
                            {
                                continue;
                            }
                        }
                    }
                    //
                    if (IsSameDomain(face, checkedFace, _linTol, _angTol, _facePlaneMap))
                    {
                        if (AddOrdinaryEdges(edges, checkedFace, ref dummy, removedEdges))
                        {
                            // sequence edges is modified
                            i = dummy;
                        }

                        faces.Append(checkedFace);
                        processed.Add(checkedFace);
                        break;
                    }
                }
            }

            if (faces.Length() > 1)
            {
                if (_facePlaneMap.ContainsKey(faces[1]))
                {
                    PlaneSurface plane = _facePlaneMap.GetValue(faces[1]);
                    var        loc   = new Placement();
                    bb.UpdateFace(refFace, plane, loc, global::Kerf.Engine.Foundation.Precision.Confusion());
                }
                // Add correct pcurves for the reference surface to the edges of other faces
                Face F_RefFace0 = refFace;
                F_RefFace0.Orient(Forward);
                AddPCurves(faces, F_RefFace0, mapEdgesWithTemporaryPCurves);

                // fill in the connectivity map for selected faces
                var mapEF0 =
                  new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
                for (i = 1; i <= faces.Length(); i++)
                {
                    Exploration.MapShapesAndAncestors(faces[i], ShapeType.Edge, ShapeType.Face, mapEF0);
                }
                // Collect keep edges and multi-connected edges, i.e. edges that are internal to
                // the set of selected faces and have connections to other faces.
                var keepEdges = new NCollection_List<Shape>();
                for (i = 1; i <= mapEF0.Count; i++)
                {
                    NCollection_List<Shape> lf = mapEF0.ValueAt(i - 1);
                    if (lf.Extent() == 2)
                    {
                        Shape                   e   = mapEF0.KeyAt(i - 1);
                        NCollection_List<Shape> glf = gMapEdgeFaces.GetValue(e);
                        if (glf.Extent() > 2 || _keepShapes.Contains(e) || freeBoundMap.Contains(e))
                        {
                            keepEdges.Append(e);
                        }
                    }
                }
                if (!keepEdges.IsEmpty())
                {
                    if (!_allowInternal)
                    {
                        // Remove from the selection the faces which have no other connect edges
                        // and contain multi-connected edges and/or keep edges.
                        var avoidFaces = new HashSet<Shape>(ShapeHasher.Instance);
                        var it           = new NCollection_List<Shape>.Iterator(keepEdges);
                        for (; it.More(); it.Next())
                        {
                            Shape                   e  = it.Value();
                            NCollection_List<Shape> lf = mapEF0.GetValue(e);
                            avoidFaces.Add(lf.First());
                            avoidFaces.Add(lf.Last());
                        }
                        for (i = 1; i <= faces.Length(); i++)
                        {
                            if (avoidFaces.Contains(faces[i]))
                            {
                                // update the boundaries of merged area, for that
                                // remove from 'edges' the edges of this face and add to 'edges'
                                // the edges of this face that were not present in 'edges' before
                                bool hasConnectAnotherFaces = false;
                                var  ex                     = new Explorer(faces[i], ShapeType.Edge);
                                for (; ex.More() && !hasConnectAnotherFaces; ex.Next())
                                {
                                    Shape                   e  = ex.Current();
                                    NCollection_List<Shape> lf = mapEF0.GetValue(e);
                                    if (lf.Extent() > 1)
                                    {
                                        for (it.Initialize(lf); it.More() && !hasConnectAnotherFaces; it.Next())
                                        {
                                            if (!avoidFaces.Contains(it.Value()))
                                            {
                                                hasConnectAnotherFaces = true;
                                            }
                                        }
                                    }
                                }
                                if (!hasConnectAnotherFaces)
                                {
                                    AddOrdinaryEdges(edges, faces[i], ref dummy, removedEdges);
                                    faces.Remove(i);
                                    i--;
                                }
                            }
                        }
                        // check if the faces with keep edges contained in
                        // already updated the boundaries of merged area
                        if (!faces.IsEmpty())
                        {
                            var mapFaces = new HashSet<Shape>(ShapeHasher.Instance);
                            for (i = 1; i <= faces.Length(); i++)
                            {
                                mapFaces.Add(faces[i]);
                            }
                            for (it.Initialize(keepEdges); it.More(); it.Next())
                            {
                                Shape                   e  = it.Value();
                                NCollection_List<Shape> lf = mapEF0.GetValue(e);
                                if (lf.Extent() < 2)
                                {
                                    continue;
                                }
                                if (mapFaces.Contains(lf.First()) && mapFaces.Contains(lf.Last()))
                                {
                                    for (i = 1; i <= faces.Length(); i++)
                                    {
                                        if (faces[i].IsEqual(lf.First()) || faces[i].IsEqual(lf.Last()))
                                        {
                                            AddOrdinaryEdges(edges, faces[i], ref dummy, removedEdges);
                                            faces.Remove(i);
                                            i--;
                                        }
                                    }
                                }
                            }
                        }
                    } // if (!myAllowInternal)
                    else
                    { // internal edges are allowed
                        // add multi-connected and keep edges as internal in new face
                        var it = new NCollection_List<Shape>.Iterator(keepEdges);
                        for (; it.More(); it.Next())
                        {
                            Shape e = it.Value();
                            edges.Append(e.Oriented(Internal));
                        }
                    }
                } // if (!aKeepEdges.IsEmpty())
            } // if (faces.Length() > 1)

            var mapEF =
              new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            for (i = 1; i <= faces.Length(); i++)
            {
                Exploration.MapShapesAndUniqueAncestors(faces[i], ShapeType.Edge, ShapeType.Face, mapEF);
            }

            // Correct orientation of edges
            for (int ii = 1; ii <= edges.Length(); ii++)
            {
                ref Shape               anEdge = ref edges[ii];
                int                            indE   = mapEF.IndexOf(anEdge) + 1;
                NCollection_List<Shape> lf    = mapEF.ValueAt(indE - 1);
                if (_allowInternal && _keepShapes.Contains(anEdge) && lf.Extent() == 2)
                {
                    edges[ii].Orient(Internal);
                }

                if (anEdge.Orientation != Internal)
                {
                    edges[ii] = mapEF.KeyAt(indE - 1);
                }
            }

            // Exclude internal edges
            var internalEdges = new IndexedSet<Shape>(ShapeHasher.Instance);
            int ind_e         = 1;
            while (ind_e <= edges.Length())
            {
                Shape anEdge = edges[ind_e];
                if (anEdge.Orientation == Internal)
                {
                    internalEdges.Add(anEdge);
                    edges.Remove(ind_e);
                }
                else
                {
                    ind_e++;
                }
            }

            if (refFaceOrientation == Reversed)
            {
                for (int ii = 1; ii <= edges.Length(); ii++)
                {
                    edges[ii].Reverse();
                }
            }
            Face F_RefFace = refFace;
            F_RefFace.Orient(Forward);

            // all faces collected in the sequence. Perform union of faces
            if (faces.Length() > 1)
            {
                double coordTol;
                var    edgesMap = new HashSet<Shape>(ShapeHasher.Instance);
                coordTol =  ComputeMinEdgeSize(edges, F_RefFace, edgesMap);
                coordTol /= 10.0;
                coordTol =  Max(coordTol, global::Kerf.Engine.Foundation.Precision.Confusion());

                var vEmap =
                  new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
                for (int ind = 1; ind <= edges.Length(); ind++)
                {
                    Exploration.MapShapesAndUniqueAncestors(edges[ind], ShapeType.Vertex, ShapeType.Edge, vEmap);
                }

                // Try to find seam edge and an edge that is not seam but has 2 pcurves on the surface
                bool seamFound, useamFound = false, vseamFound = false;
                var  edgeWith2pcurves = new Edge();
                for (int ii = 1; ii <= faces.Length(); ii++)
                {
                    Face face_ii     = ShapeCasts.Face(faces[ii]);
                    Wire outerWire = BrepTools.OuterWire(face_ii);
                    var         itw         = new Iterator(outerWire);
                    for (; itw.More(); itw.Next())
                    {
                        Edge anEdge = ShapeCasts.Edge(itw.Value());
                        if (Tool.IsClosed(anEdge, face_ii))
                        {
                            if (BrepTools.IsReallyClosed(anEdge, face_ii))
                            {
                                if (IsUiso(anEdge, face_ii))
                                {
                                    useamFound = true;
                                }
                                else
                                {
                                    vseamFound = true;
                                }
                            }
                            else
                            {
                                edgeWith2pcurves = anEdge;
                            }
                        }
                    }
                }
                seamFound = useamFound || vseamFound;

                bool isEdgeWith2pcurvesSmooth = false;
                if (_concatBSplines && !edgeWith2pcurves.IsNull && !seamFound)
                {
                    NCollection_List<Shape> faceList = gMapEdgeFaces.GetValue(edgeWith2pcurves);
                    Face                    face1    = ShapeCasts.Face(faceList.First());
                    Face                    face2    = ShapeCasts.Face(faceList.Last());
                    Continuity orderOfCont = BrepLibrary.ContinuityOfFaces(edgeWith2pcurves, face1, face2, _angTol);
                    isEdgeWith2pcurvesSmooth = (orderOfCont >= Continuity.G1);
                }

                if (isEdgeWith2pcurvesSmooth)
                {
                    Curve2D pc1 = Tool.CurveOnSurface(edgeWith2pcurves, F_RefFace, out double first, out double last)!;
                    edgeWith2pcurves.Reverse();
                    Curve2D pc2       = Tool.CurveOnSurface(edgeWith2pcurves, F_RefFace, out first, out last)!;
                    Point2D     pnt1      = pc1.Value(first);
                    Point2D     pnt2      = pc2.Value(first);
                    bool         isUclosed = (Math.Abs(pnt1.X - pnt2.X) > Math.Abs(pnt1.Y - pnt2.Y));
                    bool         toMakeUPeriodic = false, toMakeVPeriodic = false;
                    if (isUclosed && uperiod == 0.0)
                    {
                        toMakeUPeriodic = true;
                    }
                    if (!isUclosed && vperiod == 0.0)
                    {
                        toMakeVPeriodic = true;
                    }

                    if (toMakeUPeriodic || toMakeVPeriodic)
                    {
                        var  bSplineSurface = baseSurface as BSplineSurface;
                        bool isPeriodicCopy  = false;
                        if (bSplineSurface is null)
                        {
                            double        tol   = 1.0e-4;
                            Continuity uCont = Continuity.C1, vCont = Continuity.C1;
                            int           degU   = 14, degV = 14;
                            int           nmax   = 16;
                            int           prec  = 1;
                            var approximator = new ApproxSurface(baseSurface, tol, uCont, vCont, degU, degV, nmax, prec);
                            bSplineSurface = approximator.Surface()!;
                            if (bSplineSurface.IsFrozen)
                            {
                                // a new surface, but frozen by the approximation: its copy is the same new surface
                                bSplineSurface = (BSplineSurface)bSplineSurface.Copy();
                            }
                        }
                        else if (bSplineSurface.IsFrozen)
                        {
                            // OCCT: makes the face's own surface periodic in place.
                            // OCCT.NET: frozen. OCCT makes the face's own surface periodic in place, so every shape
                            // sharing it sees a periodic surface, the input's faces included (occt-issues.md). The
                            // port makes a periodic copy and puts it in the original's place in the shape it works on
                            // (its copy of the input, see Build), which is what OCCT's edit does to that shape.
                            bSplineSurface = (BSplineSurface)bSplineSurface.Copy();
                            isPeriodicCopy  = true;
                        }

                        if (toMakeUPeriodic)
                        {
                            bSplineSurface.SetUPeriodic();
                            uperiod = bSplineSurface.UPeriod;
                        }
                        if (toMakeVPeriodic)
                        {
                            bSplineSurface.SetVPeriodic();
                            vperiod = bSplineSurface.VPeriod;
                        }

                        if (isPeriodicCopy)
                        {
                            ReplaceSurface(baseSurface, bSplineSurface, [_shape, refFace, .. _context.InternalReplacements()]);
                            baseSurface = bSplineSurface;
                        }

                        // Update ref face and pcurves if the surface changed
                        if (!ReferenceEquals(bSplineSurface, baseSurface))
                        {
                            Face   oldRefFace = refFace;
                            Curve2D? nullPCurve = null;
                            refFace.Nullify();
                            bb.MakeFace(ref refFace, bSplineSurface, baseLocation, 0.0);
                            for (int ii = 1; ii <= edges.Length(); ii++)
                            {
                                Edge   anEdge  = ShapeCasts.Edge(edges[ii]);
                                Curve2D? pCurve = Tool.CurveOnSurface(anEdge, oldRefFace, out first, out last);
                                if (mapEdgesWithTemporaryPCurves.Contains(anEdge))
                                {
                                    bb.UpdateEdge(anEdge, nullPCurve, oldRefFace, 0.0);
                                }
                                bb.UpdateEdge(anEdge, pCurve, refFace, 0.0);
                            }
                            F_RefFace = refFace;
                            F_RefFace.Orient(Forward);
                        }
                    }
                } // if (myConcatBSplines && !EdgeWith2pcurves.IsNull() && !SeamFound)

                // Perform relocating to new U-origin
                // Define boundaries in 2d space of RefFace
                Span<double> periods      = [uperiod, vperiod];
                Span<bool>   isSeamFound = [useamFound, vseamFound];
                Span<double> surfMin      = [0.0, 0.0];
                Span<double> surfMax      = [0.0, 0.0];
                baseSurface.ParameterBounds(out surfMin[0], out surfMax[0], out surfMin[1], out surfMax[1]);

                for (int ii = 0; ii < 2; ii++)
                {
                    if (periods[ii] != 0.0)
                    {
                        // if seam edge exists, do nothing
                        if (!isSeamFound[ii])
                        {
                            // try to find the origin of U in 2d space
                            // so that all the faces are in [origin, origin + Uperiod]
                            double minCoord = 0.0, maxCoord = 0.0; // Umin, Umax;
                            int    numberOfIntervals = 0, i_face_max = 0;
                            if (!FindCoordBounds(faces,
                                                 F_RefFace,
                                                 mapEF,
                                                 edgesMap,
                                                 ii + 1,
                                                 periods[ii],
                                                 ref minCoord,
                                                 ref maxCoord,
                                                 ref numberOfIntervals,
                                                 ref i_face_max))
                            {
                                break;
                            }

                            if (maxCoord - minCoord > periods[ii] - 1.0e-5)
                            {
                                isSeamFound[ii] = true;
                            }
                            else if (numberOfIntervals == 2)
                            {
                                var usedEdges0    = new HashSet<Shape>(ShapeHasher.Instance);
                                var edgeNewPCurve = new Dictionary<Shape, Curve2D>(EqualityComparer<Shape>.Default);

                                // Relocate pcurves to new U-origin
                                if (!RelocatePCurvesToNewUorigin(edges,
                                                                 faces[i_face_max],
                                                                 F_RefFace,
                                                                 coordTol,
                                                                 ii + 1,
                                                                 periods[ii],
                                                                 vEmap,
                                                                 edgeNewPCurve,
                                                                 usedEdges0))
                                {
                                    vEmap.Clear();
                                    for (int ind = 1; ind <= edges.Length(); ind++)
                                    {
                                        Exploration.MapShapesAndUniqueAncestors(edges[ind], ShapeType.Vertex, ShapeType.Edge, vEmap);
                                    }
                                    break;
                                }

                                // PCurves from unused edges (may be degenerated edges)
                                for (int ind = 1; ind <= edges.Length(); ind++)
                                {
                                    Edge anEdge = ShapeCasts.Edge(edges[ind]);
                                    if (!usedEdges0.Contains(anEdge))
                                    {
                                        Curve2D? pCurve =
                                          Tool.CurveOnSurface(anEdge, F_RefFace, out double fpar, out double lpar);
                                        if (pCurve is null)
                                        {
                                            edgeNewPCurve.Clear();
                                            break;
                                        }
                                        pCurve = new TrimmedCurve2D(pCurve, fpar, lpar);
                                        edgeNewPCurve[anEdge] = pCurve;
                                    }
                                }
                                if (edgeNewPCurve.Count != edges.Length())
                                {
                                    break;
                                }

                                // Restore VEmap
                                vEmap.Clear();
                                for (int ind = 1; ind <= edges.Length(); ind++)
                                {
                                    Exploration.MapShapesAndUniqueAncestors(edges[ind], ShapeType.Vertex, ShapeType.Edge, vEmap);
                                }

                                // Find NewUmin and NewUmax
                                double newCoordMin = RealLast(), newCoordMax = RealFirst();
                                for (int jj = 1; jj <= edges.Length(); jj++)
                                {
                                    Curve2D pCurve = edgeNewPCurve.GetValue(edges[jj]);
                                    UpdateBoundaries(pCurve,
                                                     pCurve.FirstParameter,
                                                     pCurve.LastParameter,
                                                     ii + 1,
                                                     ref newCoordMin,
                                                     ref newCoordMax);
                                }

                                if (newCoordMax - newCoordMin < periods[ii] - coordTol
                                    && (-global::Kerf.Engine.Foundation.Precision.Confusion() >= newCoordMin
                                        || newCoordMin >= periods[ii] + global::Kerf.Engine.Foundation.Precision.Confusion()
                                        || -global::Kerf.Engine.Foundation.Precision.Confusion() >= newCoordMax
                                        || newCoordMax >= periods[ii] + global::Kerf.Engine.Foundation.Precision.Confusion()))
                                {
                                    // we can build a face without seam edge:
                                    // update the edges with earlier computed relocated pcurves
                                    // fitting into (NewUorigin, NewUorigin + Uperiod)
                                    double restSpaceInCoord = periods[ii] - (newCoordMax - newCoordMin);
                                    double newCoordOrigin   = newCoordMin - restSpaceInCoord / 2;
                                    if (newCoordOrigin < surfMin[ii])
                                    {
                                        newCoordOrigin = surfMin[ii];
                                    }
                                    Surface newSurf;
                                    bool         isInU = (ii == 0);
                                    if (newCoordOrigin == surfMin[ii])
                                    {
                                        newSurf = baseSurface;
                                    }
                                    else
                                    {
                                        newSurf = new TrimmedSurface(baseSurface,
                                                                                     newCoordOrigin,
                                                                                     newCoordOrigin + periods[ii],
                                                                                     isInU); // trim in U or V
                                    }
                                    Face   oldRefFace = refFace;
                                    Curve2D? nullPCurve = null;
                                    refFace.Nullify();
                                    bb.MakeFace(ref refFace, newSurf, baseLocation, 0.0);
                                    for (int jj = 1; jj <= edges.Length(); jj++)
                                    {
                                        Edge anEdge = ShapeCasts.Edge(edges[jj]);
                                        if (mapEdgesWithTemporaryPCurves.Contains(anEdge))
                                        {
                                            bb.UpdateEdge(anEdge, nullPCurve, oldRefFace, 0.0);
                                        }
                                        Curve2D pCurve = edgeNewPCurve.GetValue(anEdge);
                                        bb.UpdateEdge(anEdge, pCurve, refFace, 0.0);
                                    }
                                }
                            } // else (Umax - Umin < Uperiod - 1.e-5, no Useam)
                        } // if (!UseamFound)
                    } // if (Uperiod != 0.)
                }
                useamFound = isSeamFound[0];
                vseamFound = isSeamFound[1];
                ////////////////////////////////////
                F_RefFace = refFace;
                F_RefFace.Orient(Forward);

                var newFaces = new NCollection_Sequence<Shape>();
                var newWires = new NCollection_Sequence<Shape>();

                if (uperiod == 0 || vperiod == 0)
                {
                    // Set the "periods" for closed non-periodic surface
                    Surface surf = Tool.Surface(refFace, out Placement _)!;
                    if (surf is TrimmedSurface rts)
                    {
                        surf = rts.BasisSurface;
                    }
                    surf.ParameterBounds(out double ufirst, out double ulast, out double vfirst, out double vlast);
                    if (uperiod == 0 && surf.IsUClosed)
                    {
                        uperiod = ulast - ufirst;
                    }
                    if (vperiod == 0 && surf.IsVClosed)
                    {
                        vperiod = vlast - vfirst;
                    }
                }

                var  usedEdges            = new HashSet<Shape>(ShapeHasher.Instance);
                bool isWireBuildingFailed = false;

                double faceUmin = RealLast();
                double faceVmin = RealLast();
                for (int ii = 1; ii <= edges.Length(); ii++)
                {
                    Edge anEdge   = ShapeCasts.Edge(edges[ii]);
                    var         bAcurve = new EdgeCurve2DAdaptor(anEdge, F_RefFace);
                    if (bAcurve.Curve() is null)
                    {
                        continue;
                    }

                    Point2D firstPoint = bAcurve.Value(bAcurve.FirstParameter());
                    Point2D lastPoint  = bAcurve.Value(bAcurve.LastParameter());

                    if (firstPoint.X < faceUmin)
                    {
                        faceUmin = firstPoint.X;
                    }
                    if (lastPoint.X < faceUmin)
                    {
                        faceUmin = lastPoint.X;
                    }

                    if (firstPoint.Y < faceVmin)
                    {
                        faceVmin = firstPoint.Y;
                    }
                    if (lastPoint.Y < faceVmin)
                    {
                        faceVmin = lastPoint.Y;
                    }
                }

                // Building new wires from <edges>
                // and build faces
                while (!edges.IsEmpty())
                {
                    // try to find non-degenerated edge
                    Edge startEdge = ShapeCasts.Edge(edges[1]);
                    int         istart    = 1;
                    while (Tool.Degenerated(startEdge) && istart < edges.Length())
                    {
                        istart++;
                        startEdge = ShapeCasts.Edge(edges[istart]);
                    }

                    var newWire = new Wire();
                    bb.MakeWire(ref newWire);
                    bb.Add(newWire, startEdge);
                    RemoveEdgeFromMap(startEdge, vEmap);
                    var splittingVertices = new IndexedSet<Shape>(ShapeHasher.Instance);

                    double        fpar, lpar;
                    Curve2D? startPCurve = Tool.CurveOnSurface(startEdge, F_RefFace, out fpar, out lpar);
                    if (startPCurve is null)
                    {
                        edges.Remove(istart);
                        continue;
                    }
                    Exploration.Vertices(startEdge, out Vertex startVertex, out Vertex curVertex, true); // with orientation
                    double startParam, curParam;
                    if (startEdge.Orientation == Forward)
                    {
                        startParam = fpar;
                        curParam   = lpar;
                    }
                    else
                    {
                        startParam = lpar;
                        curParam   = fpar;
                    }
                    Point2D startPoint = startPCurve.Value(startParam);
                    Point2D curPoint   = startPCurve.Value(curParam);

                    Edge curEdge = startEdge;
                    for (;;) // loop till the end of current new wire
                    {
                        var nextEdge  = new Edge();
                        var nextPoint = new Point2D();

                        var elist = new NCollection_List<Shape>();
                        // const NCollection_List<TopoDS_Shape>& Elist = VEmap.FindFromKey(CurVertex);
                        if (vEmap.ContainsKey(curVertex))
                        {
                            elist = new NCollection_List<Shape>(vEmap.GetValue(curVertex));
                        }
                        var itl = new NCollection_List<Shape>.Iterator(elist);
                        if (elist.IsEmpty())
                        {
                            if (curVertex.IsSame(startVertex))
                            {
                                // Points of two vertices coincide in 3d but may be not in 2d
                                if ((uperiod != 0.0 && Math.Abs(startPoint.X - curPoint.X) > uperiod / 2)
                                    || (vperiod != 0.0
                                        && Math.Abs(startPoint.Y - curPoint.Y) > vperiod / 2)) // end of parametric space
                                {
                                    //<edges> do not contain seams => we must reconstruct the seam up to <NextEdge>
                                    ReconstructMissedSeam(removedEdges,
                                                          F_RefFace,
                                                          curEdge,
                                                          curVertex,
                                                          curPoint,
                                                          uperiod,
                                                          vperiod,
                                                          ref nextEdge,
                                                          ref nextPoint);
                                }
                                else
                                {
                                    break; // end of wire
                                }
                            }
                        }

                        if (nextEdge.IsNull)
                        {
                            bool endOfWire = false;

                            bool isOnSingularity = IsOnSingularity(elist);
                            if (!isOnSingularity && elist.Extent() > 1)
                            {
                                splittingVertices.Add(curVertex);
                            }

                            var tmpElist  = new NCollection_List<Shape>();
                            var trueElist = new NCollection_List<Shape>();
                            //<TrueElist> will be the list of candidates to become <NextEdge>
                            for (itl.Initialize(elist); itl.More(); itl.Next())
                            {
                                Edge anEdge = ShapeCasts.Edge(itl.Value());
                                if (usedEdges.Contains(anEdge))
                                {
                                    continue;
                                }
                                Vertex firstVertex = Exploration.FirstVertex(anEdge, true);
                                if (!firstVertex.IsSame(curVertex))
                                {
                                    continue;
                                }
                                tmpElist.Append(anEdge);
                            }
                            Curve2D? curPCurve = Tool.CurveOnSurface(curEdge, F_RefFace, out fpar, out lpar);
                            if (tmpElist.Extent() <= 1 || (uperiod != 0.0 || vperiod != 0) || curPCurve is null)
                            {
                                trueElist.Assign(tmpElist);
                            }
                            else
                            {
                                // we must choose the closest direction - the biggest angle
                                double maxAngle = RealFirst();
                                var    trueEdge = new Edge();
                                curParam = (curEdge.Orientation == Forward) ? lpar : fpar;
                                Curve2D.ResD1 curD1 = curPCurve.EvalD1(curParam);
                                curPoint = curD1.Point;
                                Vector2D curDir = curD1.D1;
                                curDir = curDir.Normalized();
                                if (curEdge.Orientation == Reversed)
                                {
                                    curDir = curDir.Reversed();
                                }
                                for (itl.Initialize(tmpElist); itl.More(); itl.Next())
                                {
                                    Edge   anEdge  = ShapeCasts.Edge(itl.Value());
                                    Curve2D? pCurve = Tool.CurveOnSurface(anEdge, F_RefFace, out fpar, out lpar);
                                    if (pCurve is null)
                                    {
                                        // edge has no pcurve on the reference face, skip it as a direction candidate
                                        continue;
                                    }
                                    double   param = (anEdge.Orientation == Forward) ? fpar : lpar;
                                    Vector2D dir   = pCurve.EvalD1(param).D1;
                                    dir = dir.Normalized();
                                    if (anEdge.Orientation == Reversed)
                                    {
                                        dir = dir.Reversed();
                                    }
                                    double angle = curDir.AngleTo(dir);
                                    if (angle > maxAngle)
                                    {
                                        maxAngle = angle;
                                        trueEdge = anEdge;
                                    }
                                }
                                if (!trueEdge.IsNull)
                                {
                                    trueElist.Append(trueEdge);
                                }
                            }

                            // Find next edge in TrueElist
                            for (itl.Initialize(trueElist); itl.More(); itl.Next())
                            {
                                Edge anEdge = ShapeCasts.Edge(itl.Value());

                                Curve2D? pCurve = Tool.CurveOnSurface(anEdge, F_RefFace, out fpar, out lpar);
                                if (pCurve is null)
                                {
                                    // edge has no pcurve on the reference face, skip it as a next-edge candidate
                                    continue;
                                }
                                double   param = (anEdge.Orientation == Forward) ? fpar : lpar;
                                Point2D point = pCurve.Value(param);
                                double   diffU  = Math.Abs(point.X - curPoint.X);
                                double   diffV  = Math.Abs(point.Y - curPoint.Y);
                                if (uperiod != 0.0 && diffU > coordTol && Math.Abs(diffU - uperiod) > coordTol)
                                { // may be it is a deg.vertex
                                    continue;
                                }
                                if (vperiod != 0.0 && diffV > coordTol && Math.Abs(diffV - vperiod) > coordTol)
                                { // may be it is a deg.vertex
                                    continue;
                                }

                                // Check: may be <CurPoint> and <aPoint> are on Period from each other
                                if ((uperiod != 0.0 && diffU > uperiod / 2)
                                    || (vperiod != 0.0 && diffV > vperiod / 2)) // end of parametric space
                                {
                                    //<edges> do not contain seams => we must reconstruct the seam up to <NextEdge>
                                    var startOfNextEdge  = new Point2D();
                                    var lastVertexOfSeam = new Vertex();
                                    ReconstructMissedSeam(removedEdges,
                                                          F_RefFace,
                                                          curEdge,
                                                          curVertex,
                                                          curPoint,
                                                          uperiod,
                                                          vperiod,
                                                          ref nextEdge,
                                                          ref nextPoint);

                                    // Check: may be it is the end
                                    // OCCT: LastVertexOfSeam and StartOfNextEdge are never set, so this never holds
                                    if (lastVertexOfSeam.IsSame(startVertex)
                                        && Math.Abs(startPoint.X - startOfNextEdge.X) < uperiod / 2)
                                    {
                                        endOfWire = true;
                                    }

                                    break;
                                }
                                else
                                {
                                    nextEdge = anEdge;
                                    double lastParam = (nextEdge.Orientation == Forward) ? lpar : fpar;
                                    nextPoint = pCurve.Value(lastParam);
                                    break;
                                }
                            } // for (itl.Initialize(TrueElist); itl.More(); itl.Next())

                            if (endOfWire)
                            {
                                break;
                            }
                        }

                        if (nextEdge.IsNull)
                        {
                            if (uperiod != 0.0 || vperiod != 0.0)
                            {
                                if (curVertex.IsSame(startVertex)
                                    && (uperiod == 0.0 || Math.Abs(startPoint.X - curPoint.X) < uperiod / 2)
                                    && (vperiod == 0.0 || Math.Abs(startPoint.Y - curPoint.Y) < vperiod / 2))
                                {
                                    break; // end of wire
                                }

                                ReconstructMissedSeam(removedEdges,
                                                      F_RefFace,
                                                      curEdge,
                                                      curVertex,
                                                      curPoint,
                                                      uperiod,
                                                      vperiod,
                                                      ref nextEdge,
                                                      ref nextPoint);
                                if (nextEdge.IsNull)
                                {
                                    isWireBuildingFailed = true;
                                    break;
                                }
                            }
                            else
                            {
                                isWireBuildingFailed = true;
                                break;
                            }
                        }
                        curPoint  = nextPoint;
                        curEdge   = nextEdge;
                        curVertex = Exploration.LastVertex(curEdge, true); // with orientation
                        bb.Add(newWire, curEdge);
                        usedEdges.Add(curEdge);
                        RemoveEdgeFromMap(curEdge, vEmap);
                    } // for (;;)

                    if (isWireBuildingFailed)
                    {
                        break;
                    }

                    newWire.Closed(true);
                    usedEdges.Add(startEdge);

                    // Remove used edges from sequence
                    int ind = 1;
                    while (ind <= edges.Length())
                    {
                        if (usedEdges.Contains(edges[ind]))
                        {
                            edges.Remove(ind);
                        }
                        else
                        {
                            ind++;
                        }
                    }

                    // add just built wire to current face or save it in the sequence of wires
                    bool edgeOnBoundOfSurfFound = false;
                    var  itw                    = new Iterator(newWire);
                    for (; itw.More(); itw.Next())
                    {
                        Edge anEdge = ShapeCasts.Edge(itw.Value());
                        if (Tool.IsClosed(anEdge, refFace))
                        {
                            edgeOnBoundOfSurfFound = true;
                            break;
                        }
                    }
                    if (edgeOnBoundOfSurfFound) // this wire can not be a hole
                    {
                        Surface? surf   = Tool.Surface(refFace, out Placement loc);
                        var           result = new Face();
                        bb.MakeFace(ref result, surf, loc, 0);
                        bb.Add(result, newWire);
                        result.Orient(refFaceOrientation);
                        newFaces.Append(result);
                    }
                    else // may be this wire is a hole
                    {
                        // split this wire if needed
                        if (splittingVertices.Count != 0)
                        {
                            SplitWire(newWire, F_RefFace, splittingVertices, newWires);
                        }
                        else
                        {
                            newWires.Append(newWire);
                        }
                    }
                } // while (!edges.IsEmpty())

                if (isWireBuildingFailed)
                {
                    continue;
                }

                // Build wires from internal edges
                var intVEmap =
                  new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
                for (int ii = 0; ii < internalEdges.Count; ii++)
                {
                    Exploration.MapShapesAndAncestors(internalEdges[ii], ShapeType.Vertex, ShapeType.Edge, intVEmap);
                }
                var internalWires = new NCollection_Sequence<Shape>();
                while (internalEdges.Count != 0)
                {
                    Edge firstEdge = ShapeCasts.Edge(internalEdges[0]);
                    internalEdges.RemoveAtSwap(0);
                    RemoveEdgeFromMap(firstEdge, intVEmap);
                    var internalWire = new Wire();
                    bb.MakeWire(ref internalWire);
                    bb.Add(internalWire, firstEdge);
                    var endEdges = new Edge[2];
                    endEdges[0] = endEdges[1] = firstEdge;
                    var vv = new Vertex[2];
                    Exploration.Vertices(firstEdge, out vv[0], out vv[1]);
                    for (;;)
                    {
                        if (vv[0].IsSame(vv[1]))
                        { // closed wire
                            break;
                        }
                        bool found = false;
                        for (int ii = 0; ii < 2; ii++)
                        {
                            NCollection_List<Shape> elist = intVEmap.GetValue(vv[ii]);
                            var                            itl   = new NCollection_List<Shape>.Iterator(elist);
                            for (; itl.More(); itl.Next())
                            {
                                Edge anEdge = ShapeCasts.Edge(itl.Value());
                                if (anEdge.IsSame(endEdges[ii]))
                                {
                                    continue;
                                }
                                found = true;
                                internalEdges.Remove(anEdge);
                                RemoveEdgeFromMap(anEdge, intVEmap);
                                bb.Add(internalWire, anEdge);
                                Exploration.Vertices(anEdge, out Vertex v1, out Vertex v2);
                                vv[ii]       = (v1.IsSame(vv[ii])) ? v2 : v1;
                                endEdges[ii] = anEdge;
                                break;
                            }
                        }
                        if (!found)
                        { // end of open wire
                            break;
                        }
                    }
                    internalWires.Append(internalWire);
                }

                // Insert new faces instead of old ones
                if (newFaces.IsEmpty())
                {
                    // one face without seam
                    Surface? surf   = Tool.Surface(refFace, out Placement loc);
                    var           result = new Face();
                    bb.MakeFace(ref result, surf, loc, 0.0);
                    for (int ii = 1; ii <= newWires.Length(); ii++)
                    {
                        bb.Add(result, newWires[ii]);
                    }
                    for (int ii = 1; ii <= internalWires.Length(); ii++)
                    {
                        bb.Add(result, internalWires[ii]);
                    }
                    result.Orient(refFaceOrientation);
                    _context.Merge(faces, result);
                    // Update the map Face-NewFace
                    for (int jj = 1; jj <= faces.Length(); jj++)
                    {
                        _faceNewFace[faces[jj]] = result;
                    }
                }
                else if (newFaces.Length() == 1)
                {
                    Shape newFace = newFaces[1].Oriented(Forward);
                    for (int ii = 1; ii <= newWires.Length(); ii++)
                    {
                        bb.Add(newFace, newWires[ii]);
                    }
                    for (int ii = 1; ii <= internalWires.Length(); ii++)
                    {
                        bb.Add(newFace, internalWires[ii]);
                    }
                    _context.Merge(faces, newFaces[1]);
                    // Update the map Face-NewFace
                    for (int jj = 1; jj <= faces.Length(); jj++)
                    {
                        _faceNewFace[faces[jj]] = newFaces[1];
                    }
                }
                else
                {
                    // Insert new wires and internal wires into correspondent faces
                    InsertWiresIntoFaces(newWires, newFaces, refFace);
                    InsertWiresIntoFaces(internalWires, newFaces, refFace);

                    var emaps = new NCollection_Sequence<HashSet<Shape>>();
                    for (int ii = 1; ii <= faces.Length(); ii++)
                    {
                        var emap = new HashSet<Shape>(ShapeHasher.Instance);
                        Exploration.MapShapes(faces[ii], emap);
                        emaps.Append(emap);
                    }
                    for (int ii = 1; ii <= newFaces.Length(); ii++)
                    {
                        var facesForThisFace = new NCollection_Sequence<Shape>();
                        var usedFaces        = new HashSet<Shape>(ShapeHasher.Instance);
                        var explo            = new Explorer(newFaces[ii], ShapeType.Edge);
                        for (; explo.More(); explo.Next())
                        {
                            Edge anEdge = ShapeCasts.Edge(explo.Current());
                            if (Tool.Degenerated(anEdge) || Tool.IsClosed(anEdge, refFace))
                            {
                                continue;
                            }
                            int jj;
                            for (jj = 1; jj <= emaps.Length(); jj++)
                            {
                                if (emaps[jj].Contains(anEdge))
                                {
                                    break;
                                }
                            }
                            if (usedFaces.Add(faces[jj]))
                            {
                                facesForThisFace.Append(faces[jj]);
                            }
                        }
                        _context.Merge(facesForThisFace, newFaces[ii]);
                        // Update the map Face-NewFace
                        for (int jj = 1; jj <= facesForThisFace.Length(); jj++)
                        {
                            _faceNewFace[facesForThisFace[jj]] = newFaces[ii];
                        }
                    }
                }
            } // if (faces.Length() > 1)
        } // end processing each face
    }

    /// <summary>
    /// This method makes if possible a common edge from each group of smothly connected edges, which are common for
    /// the same couple of faces.
    /// </summary>
    protected void UnifyEdges()
    {
        Shape res = _context.InternalApply(_shape);
        // creating map of edge faces
        var mapEdgeFaces =
          new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(res, ShapeType.Edge, ShapeType.Face, mapEdgeFaces);
        // creating map of vertex edges
        var mapEdgesVertex =
          new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndUniqueAncestors(res, ShapeType.Vertex, ShapeType.Edge, mapEdgesVertex);
        // creating map of vertex faces
        var vFmap =
          new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndUniqueAncestors(res, ShapeType.Vertex, ShapeType.Face, vFmap);

        if (_safeInputMode)
        {
            UpdateMapOfShapes(_keepShapes, _context);
        }

        // Sequence of the edges of the shape
        var seqEdges = new NCollection_Sequence<Shape>();
        int nbE      = mapEdgeFaces.Count;
        for (int i = 0; i < nbE; ++i)
        {
            seqEdges.Append(mapEdgeFaces.KeyAt(i));
        }

        // Prepare map of shared vertices (with the number of connected edges greater then 2)
        var sharedVert = new HashSet<Shape>(ShapeHasher.Instance);
        CheckSharedVertices(seqEdges, mapEdgesVertex, _keepShapes, sharedVert);
        // Merge the edges avoiding removal of the shared vertices
        bool isMerged = MergeSeq(seqEdges, vFmap, sharedVert);
        // Collect faces to rebuild
        var changedFaces = new IndexedSet<Shape>(ShapeHasher.Instance);
        if (isMerged)
        {
            for (int i = 0; i < nbE; ++i)
            {
                Shape e = mapEdgeFaces.KeyAt(i);
                if (_context.IsRecorded(e))
                {
                    var it = new NCollection_List<Shape>.Iterator(mapEdgeFaces.ValueAt(i));
                    for (; it.More(); it.Next())
                    {
                        changedFaces.Add(it.Value());
                    }
                }
            }
        }

        // fix changed faces and replace them in the local context
        double prec = global::Kerf.Engine.Foundation.Precision.Confusion();
        for (int i = 0; i < changedFaces.Count; i++)
        {
            Face face = ShapeCasts.Face(_context.InternalApply(changedFaces[i]));
            if (face.IsNull)
            {
                continue;
            }

            // for a planar face create and store pcurve of edge on face
            // to speed up all operations; but this is allowed only when non-safe mode in force
            if (!_safeInputMode)
            {
                Surface? surface = Tool.Surface(face, out Placement _);
                surface = ClearRts(surface);
                if (surface is PlaneSurface)
                {
                    var le = new NCollection_List<Shape>();
                    for (var ex = new Explorer(face, ShapeType.Edge); ex.More(); ex.Next())
                    {
                        le.Append(ex.Current());
                    }
                    BrepLibrary.BuildPCurveForEdgesOnPlane(le, face);
                }
            }

            var sff = new FaceFixer(face);
            if (_safeInputMode)
            {
                sff.SetContext(_context);
            }
            sff.SetPrecision(prec);
            sff.SetMinTolerance(prec);
            sff.SetMaxTolerance(Max(1.0, prec * 1000.0));
            sff.FixOrientationMode()         = 0;
            sff.FixAddNaturalBoundMode()     = 0;
            sff.FixIntersectingWiresMode()   = 0;
            sff.FixLoopWiresMode()           = 0;
            sff.FixSplitFaceMode()           = 0;
            sff.FixPeriodicDegeneratedMode() = 0;
            SetFixWireModes(sff);
            sff.Perform();
            Shape newFace = sff.InternalFace();
            _context.Replace(face, newFace);
        }

        if (changedFaces.Count > 0)
        {
            // fix changed shell and replace it in the local context
            Shape res1     = _context.InternalApply(res);
            bool         isChanged = false;
            var          expsh     = new Explorer();
            for (expsh.Init(res1, ShapeType.Shell); expsh.More(); expsh.Next())
            {
                Shell shell = ShapeCasts.Shell(expsh.Current());
                var          sfsh   = new ShellFixer();
                sfsh.FixFaceOrientation(shell);
                Shape newShell = sfsh.InternalShell();
                if (!newShell.IsSame(shell))
                {
                    _context.Replace(shell, newShell);
                    isChanged = true;
                }
            }
            if (isChanged)
            {
                res1 = _context.InternalApply(res1);
            }
            _context.Replace(_shape, res1);
        }

        _shape = _context.InternalApply(_shape);
    }

    /// <summary>Fills the history of the modifications during the operation.</summary>
    protected void FillHistory()
    {
        if (_history is null)
        {
            // History is not requested
            return;
        }

        // Only Vertices, Edges and Faces can be modified during unification.
        // Thus, only these kind of shapes should be checked.

        // Get history from the context.
        // It contains all modifications of the operation. Some of these
        // modifications become not relevant and should be filtered.
        History ctxHistory = _context.InternalHistory();

        // Explore the history of the context and fill
        // the history of UnifySameDomain algorithm
        var usdHistory = new History();

        // Map all Vertices, Edges, Faces and Solids in the input shape
        var mapInputShape = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(_initShape, ShapeType.Vertex, mapInputShape);
        Exploration.MapShapes(_initShape, ShapeType.Edge, mapInputShape);
        Exploration.MapShapes(_initShape, ShapeType.Face, mapInputShape);
        Exploration.MapShapes(_initShape, ShapeType.Solid, mapInputShape);

        // Map all Vertices, Edges, Faces and Solids in the result shape
        var mapResultShapes = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(_shape, ShapeType.Vertex, mapResultShapes);
        Exploration.MapShapes(_shape, ShapeType.Edge, mapResultShapes);
        Exploration.MapShapes(_shape, ShapeType.Face, mapResultShapes);
        Exploration.MapShapes(_shape, ShapeType.Solid, mapResultShapes);

        // Iterate on all input shapes and get their modifications
        int i, nb = mapInputShape.Count;
        for (i = 1; i <= nb; ++i)
        {
            Shape s = mapInputShape[i - 1];

            // Check the shape itself to be present in the result
            if (mapResultShapes.Contains(s))
            {
                // The shape is present in the result as is, thus has not been modified
                continue;
            }

            // Check if the shape has been modified during the operation
            var lsImages = ctxHistory.Modified(s);
            if (lsImages.Count == 0)
            {
                // The shape has not been modified and not present in the result,
                // thus it has been removed
                usdHistory.Remove(s);
                continue;
            }

            // Check the images of the shape to be present in the result
            bool bRemoved = true;
            foreach (Shape sIm in lsImages)
            {
                if (mapResultShapes.Contains(sIm))
                {
                    if (!sIm.IsSame(s))
                    {
                        // Image is found in the result, thus the shape has been modified
                        usdHistory.AddModified(s, sIm);
                    }
                    bRemoved = false;
                }
            }

            if (bRemoved)
            {
                // No images are found in the result, thus the shape has been removed
                usdHistory.Remove(s);
            }
        }

        // Merge the history of the operation into global history
        // OCCT.NET: Build merges it once its working overlay is finished, composed with the changes made in place
        // (WorkingOverlay.Composed); inside another operation's overlay that is OCCT's merge.
        _usdHistory = usdHistory;
    }
}
