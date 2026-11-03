// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_UnifySameDomain.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_UnifySameDomain.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// The .cxx is split by subject: this file holds the class, the unification of faces (UnifyFaces, IntUnifyFaces),
// UnifyEdges, Build and FillHistory; ShapeUpgrade_UnifySameDomain.Faces.cs the file-static functions of the faces;
// ShapeUpgrade_UnifySameDomain.Edges.cs MergeEdges, MergeSeq, MergeSubSeq, UnionPCurves and theirs.

// OCCT's typedefs of the class
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using DataMapOfFacePlane = Occt.NCollection_DataMap<Kerf.Topology.Shape, Kerf.Geometry.PlaneSurface>;
using DataMapOfShapeMapOfShape = Occt.NCollection_DataMap<Kerf.Topology.Shape, Occt.NCollection_Map<Kerf.Topology.Shape>>;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

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
/// the safe input mode OCCT adds pcurves to the input's edges): a shape holding frozen shapes (any shape a caller
/// holds) is copied first (<see cref="CopyAtEntry"/>, decision 583), so the caller's shape never changes,
/// whatever <see cref="SetSafeInputMode"/> says; the algorithm then runs on the copy with the mode given, so its
/// results are OCCT's for that mode. At the end the copied sub-shapes left unchanged are the caller's own again: a
/// shape needing nothing gives back the input itself. An unfrozen input (ported code's) is worked on in place, as in
/// OCCT.
/// </para>
/// <para>
/// <see cref="Shape"/> and <see cref="History()"/> freeze the result (the history lists sub-shapes of the input and of
/// the result); ported code that goes on building reads <see cref="InternalShape"/> and
/// <see cref="InternalHistory"/>. The history answers for the caller's sub-shapes (decision 642's composition); a
/// sub-shape changed in place (a pcurve added) is modified into its changed copy, where OCCT's history has no entry.
/// </para>
/// </remarks>
public partial class ShapeUpgrade_UnifySameDomain : Transient
{
    private Shape                  myInitShape;
    private double                        myLinTol;
    private double                        myAngTol;
    private bool                          myUnifyFaces;
    private bool                          myUnifyEdges;
    private bool                          myConcatBSplines;
    private bool                          myAllowInternal;
    private bool                          mySafeInputMode;
    private Shape                  myShape;
    private readonly ShapeBuild_ReShape   myContext;
    private NCollection_Map<Shape> myKeepShapes = new(ShapeHasher.Instance);
    private readonly DataMapOfFacePlane   myFacePlaneMap = new(ShapeHasher.Instance);
    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myEFmap =
      new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, Shape> myFaceNewFace = new(ShapeHasher.Instance);

    private History? myHistory; //!< The history.

    // OCCT.NET: the history FillHistory made over the shape worked on, merged into myHistory by Build once the copy at
    // the entry is finished (composed onto the caller's sub-shapes).
    private History? myUSDHistory;

    /// <summary>Empty constructor.</summary>
    public ShapeUpgrade_UnifySameDomain()
    {
        myInitShape      = new Shape();
        myLinTol         = global::Kerf.Engine.Foundation.Precision.Confusion();
        myAngTol         = global::Kerf.Engine.Foundation.Precision.Angular();
        myUnifyFaces     = true;
        myUnifyEdges     = true;
        myConcatBSplines = false;
        myAllowInternal  = false;
        mySafeInputMode  = true;
        myShape          = new Shape();
        myHistory        = new History();
        myContext        = new ShapeBuild_ReShape();
    }

    /// <summary>Constructor defining the input shape and the flags. It does not perform unification.</summary>
    public ShapeUpgrade_UnifySameDomain(in Shape aShape,
                                        bool            UnifyEdges     = true,
                                        bool            UnifyFaces     = true,
                                        bool            ConcatBSplines = false)
    {
        myInitShape      = aShape;
        myLinTol         = global::Kerf.Engine.Foundation.Precision.Confusion();
        myAngTol         = global::Kerf.Engine.Foundation.Precision.Angular();
        myUnifyFaces     = UnifyFaces;
        myUnifyEdges     = UnifyEdges;
        myConcatBSplines = ConcatBSplines;
        myAllowInternal  = false;
        mySafeInputMode  = true;
        myShape          = aShape;
        myHistory        = new History();
        myContext        = new ShapeBuild_ReShape();
    }

    /// <summary>
    /// Initializes with a shape and the flags. It does not perform unification. If you intend to nullify the history
    /// place holder, do it after initialization.
    /// </summary>
    public void Initialize(in Shape aShape,
                           bool            UnifyEdges     = true,
                           bool            UnifyFaces     = true,
                           bool            ConcatBSplines = false)
    {
        myInitShape      = aShape;
        myShape          = aShape;
        myUnifyEdges     = UnifyEdges;
        myUnifyFaces     = UnifyFaces;
        myConcatBSplines = ConcatBSplines;

        myContext.Clear();
        myKeepShapes.Clear();
        myFacePlaneMap.Clear();
        myEFmap.Clear();
        myFaceNewFace.Clear();
        myHistory?.Clear();
    }

    /// <summary>
    /// Sets the flag defining whether it is allowed to create internal edges inside merged faces in the case of
    /// non-manifold topology. Without this flag merging through multi connected edge is forbidden. Default value is
    /// false.
    /// </summary>
    public void AllowInternalEdges(bool theValue)
    {
        myAllowInternal = theValue;
    }

    /// <summary>
    /// Sets the shape for avoid merging of the faces/edges. This shape can be vertex or edge. If the shape is a vertex
    /// it forbids merging of connected edges. If the shape is a edge it forbids merging of connected faces. This
    /// method can be called several times to keep several shapes.
    /// </summary>
    public void KeepShape(in Shape theShape)
    {
        if (theShape.ShapeType() == ShapeType.Edge || theShape.ShapeType() == ShapeType.Vertex)
        {
            myKeepShapes.Add(theShape);
        }
    }

    /// <summary>
    /// Sets the map of shapes for avoid merging of the faces/edges. It allows passing a ready to use map instead of
    /// calling many times the method KeepShape.
    /// </summary>
    public void KeepShapes(NCollection_Map<Shape> theShapes)
    {
        foreach (Shape aShape in theShapes)
        {
            if (aShape.ShapeType() == ShapeType.Edge || aShape.ShapeType() == ShapeType.Vertex)
            {
                myKeepShapes.Add(aShape);
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
    public void SetSafeInputMode(bool theValue)
    {
        mySafeInputMode = theValue;
    }

    /// <summary>
    /// Sets the linear tolerance. It plays the role of chord error when taking decision about merging of shapes.
    /// Default value is Precision::Confusion().
    /// </summary>
    public void SetLinearTolerance(double theValue)
    {
        myLinTol = theValue;
    }

    /// <summary>
    /// Sets the angular tolerance. If two shapes form a connection angle greater than this value they will not be
    /// merged. Default value is Precision::Angular().
    /// </summary>
    public void SetAngularTolerance(double theValue)
    {
        myAngTol = (theValue < global::Kerf.Engine.Foundation.Precision.Angular() ? global::Kerf.Engine.Foundation.Precision.Angular() : theValue);
    }

    /// <summary>The resulting shape, frozen.</summary>
    public Shape Shape()
    {
        myShape.TShape()?.FreezeGraph();
        return myShape;
    }

    /// <summary>The resulting shape, as <see cref="Shape"/>, without freezing it (ported code).</summary>
    internal Shape InternalShape() => myShape;

    /// <summary>
    /// The history of the processed shapes, null if it isn't collected. The result is frozen first, so every shape
    /// the history lists is frozen.
    /// </summary>
    public History? History()
    {
        myShape.TShape()?.FreezeGraph();
        return myHistory;
    }

    /// <summary>
    /// Sets the place holder of the history (OCCT's assignment to the non-const <c>History()</c>): null to collect no
    /// history. The history of each <see cref="Build"/> is merged into it.
    /// </summary>
    public void History(History? theHistory)
    {
        myHistory = theHistory;
    }

    /// <summary>The history, as <see cref="History()"/>, without freezing the result (ported code).</summary>
    internal History? InternalHistory() => myHistory;

    /// <summary>Performs unification and builds the resulting shape.</summary>
    public void Build()
    {
        // OCCT.NET: frozen. The copy at the entry (decision 583): OCCT writes into the shape it works on (pcurves added
        // to the edges of the faces it unifies, even in the safe input mode)
        var          anEntry    = new CopyAtEntry(myInitShape, myContext);
        Shape anInput    = myInitShape;
        NCollection_Map<Shape> aKeepShapes = myKeepShapes;
        if (anEntry.IsCopied)
        {
            myInitShape  = anEntry.Shape();
            myShape      = anEntry.Shape();
            myKeepShapes = new NCollection_Map<Shape>(ShapeHasher.Instance);
            foreach (Shape aKeepShape in aKeepShapes)
            {
                myKeepShapes.Add(anEntry.Copied(aKeepShape));
            }
        }

        try
        {
            build();
        }
        finally
        {
            myInitShape  = anInput;
            myKeepShapes = aKeepShapes;
        }

        myShape = anEntry.Finish(myShape);
        if (myUSDHistory is not null)
        {
            // Merge the history of the operation into global history
            myHistory!.Merge(anEntry.Composed(myUSDHistory));
            myUSDHistory = null;
        }
    }

    // OCCT's Build, on the shape worked on
    private void build()
    {
        Exploration.MapShapesAndAncestors(myInitShape, ShapeType.Edge, ShapeType.Face, myEFmap);

        if (myUnifyFaces)
        {
            UnifyFaces();
        }
        if (myUnifyEdges)
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
        var aGMapEdgeFaces =
          new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);

        var aFaceMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(myShape, ShapeType.Face, aFaceMap);
        for (int i = 1; i <= aFaceMap.Extent(); i++)
        {
            Exploration.MapShapesAndAncestors(aFaceMap[i], ShapeType.Edge, ShapeType.Face, aGMapEdgeFaces);
        }

        // creating map of face shells for the whole shape to avoid
        // unification of faces belonging to the different shells
        var aGMapFaceShells = new DataMapOfShapeMapOfShape(ShapeHasher.Instance);
        for (var anExp = new Explorer(myShape, ShapeType.Shell); anExp.More(); anExp.Next())
        {
            Shape aShell = anExp.Current();
            for (var anItF = new Iterator(aShell); anItF.More(); anItF.Next())
            {
                Shape aF = anItF.Value();
                if (aGMapFaceShells.Find(aF, out NCollection_Map<Shape> pShells))
                {
                    pShells.Add(aShell);
                }
                else
                {
                    aGMapFaceShells.Bound(aF, new NCollection_Map<Shape>(ShapeHasher.Instance)).Add(aShell);
                }
            }
        }

        // creating map of free boundaries
        var aFreeBoundMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
        // look at only shells not belonging to solids
        var anExplo = new Explorer(myShape, ShapeType.Shell, ShapeType.Solid);
        for (; anExplo.More(); anExplo.Next())
        {
            Shape aShell = anExplo.Current();
            var aEFmap = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(aShell, ShapeType.Edge, ShapeType.Face, aEFmap);
            for (int ii = 1; ii <= aEFmap.Extent(); ii++)
            {
                Edge                    anEdge    = ShapeCasts.Edge(aEFmap.FindKey(ii));
                NCollection_List<Shape> aFaceList = aEFmap[ii];
                if (!Tool.Degenerated(anEdge) && aFaceList.Extent() == 1)
                {
                    aFreeBoundMap.Add(anEdge);
                }
            }
        }

        // unify faces in each shell separately
        var exps = new Explorer();
        for (exps.Init(myShape, ShapeType.Shell); exps.More(); exps.Next())
        {
            IntUnifyFaces(exps.Current(), aGMapEdgeFaces, aGMapFaceShells, aFreeBoundMap);
        }

        // gather all faces out of shells in one compound and unify them at once
        var aBB  = new BrepBuilder();
        var aCmp = new Compound();
        aBB.MakeCompound(ref aCmp);
        int nbf = 0;
        for (exps.Init(myShape, ShapeType.Face, ShapeType.Shell); exps.More(); exps.Next(), nbf++)
        {
            aBB.Add(aCmp, exps.Current());
        }

        if (nbf > 0)
        {
            // No connection to shells, thus no need to pass the face-shell map
            IntUnifyFaces(aCmp, aGMapEdgeFaces, new DataMapOfShapeMapOfShape(ShapeHasher.Instance), aFreeBoundMap);
        }

        myShape = myContext.InternalApply(myShape);
    }

    private void IntUnifyFaces(in Shape                                                          theInpShape,
                               NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theGMapEdgeFaces,
                               DataMapOfShapeMapOfShape                                                 theGMapFaceShells,
                               NCollection_Map<Shape>                                            theFreeBoundMap)
    {
        // creating map of edge faces for the shape
        var aMapEdgeFaces =
          new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theInpShape, ShapeType.Edge, ShapeType.Face, aMapEdgeFaces);

        // map of processed shapes
        var aProcessed = new NCollection_Map<Shape>(ShapeHasher.Instance);

        // processing each face
        var exp = new Explorer();
        for (exp.Init(theInpShape, ShapeType.Face); exp.More(); exp.Next())
        {
            Face aFace = ShapeCasts.Face(exp.Current());
            if (aProcessed.Contains(aFace))
            {
                continue;
            }

            Surface? aBaseSurface = ClearRts(Tool.Surface(aFace));
            // Bug 33894: Prevent crash when face has no surface
            if (aBaseSurface is null)
            {
                continue;
            }

            // Boundary edges for the new face
            var edges        = new NCollection_Sequence<Shape>();
            var RemovedEdges = new NCollection_Sequence<Shape>();

            int dummy = 0;
            AddOrdinaryEdges(edges, aFace, ref dummy, RemovedEdges);

            // Faces to get unified with the current faces
            var faces = new NCollection_Sequence<Shape>();

            // Add the current face for unification
            faces.Append(aFace);

            // surface and location to construct result
            var                aBaseLocation      = new Placement();
            Orientation RefFaceOrientation = aFace.Orientation();

            // Take original surface
            var RefFace = new Face();
            var BB      = new BrepBuilder();
            BB.MakeFace(ref RefFace, aBaseSurface, aBaseLocation, 0.0);
            RefFace.Orientation(RefFaceOrientation);
            // map of edges not lying on RefFace; these edges may be updated by temporary pcurves
            var MapEdgesWithTemporaryPCurves = new NCollection_Map<Shape>(ShapeHasher.Instance);

            double Uperiod = (aBaseSurface.IsUPeriodic) ? aBaseSurface.UPeriod() : 0.0;
            double Vperiod = (aBaseSurface.IsVPeriodic) ? aBaseSurface.VPeriod() : 0.0;

            // Get shells connected to the face (in normal cases should not be more than 2)
            NCollection_Map<Shape>? pFShells1 =
              theGMapFaceShells.Find(aFace, out NCollection_Map<Shape> aFShells1) ? aFShells1 : null;

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
                NCollection_List<Shape> aGList = theGMapEdgeFaces.FindFromKey(edge);
                if (!myAllowInternal
                    && (aGList.Extent() != 2 || myKeepShapes.Contains(edge) || theFreeBoundMap.Contains(edge)))
                {
                    // non manifold case is not processed unless myAllowInternal
                    continue;
                }
                //
                // Get the faces connected through the edge in the current shape
                NCollection_List<Shape> aList = aMapEdgeFaces.FindFromKey(edge);
                if (aList.Extent() < 2)
                {
                    continue;
                }

                // for a planar face create and store pcurve of edge on face
                // to speed up all operations
                if (!mySafeInputMode && aBaseSurface is PlaneSurface)
                {
                    BRepLib.BuildPCurveForEdgeOnPlane(edge, aFace);
                }

                // get normal of the face to compare it with normals of other faces
                var aDN1 = new Direction3D();
                //
                // take intermediate point on edge to compute the normal
                Tool.Range(edge, out double f, out double l);
                double aTMid = (f + l) * .5;
                //
                bool bCheckNormals = GetNormalToSurface(aFace, edge, aTMid, ref aDN1);
                //
                // Process the faces
                var anIter = new NCollection_List<Shape>.Iterator(aList);
                for (; anIter.More(); anIter.Next())
                {
                    Face aCheckedFace = ShapeCasts.Face(anIter.Value());
                    if (aCheckedFace.IsSame(aFace))
                    {
                        continue;
                    }

                    if (aProcessed.Contains(aCheckedFace))
                    {
                        continue;
                    }

                    // Get shells connected to the checked face
                    NCollection_Map<Shape>? pFShells2 =
                      theGMapFaceShells.Find(aCheckedFace, out NCollection_Map<Shape> aFShells2) ? aFShells2 : null;
                    // Faces can be unified only if the shells of faces connected to
                    // these faces are the same. Otherwise, topology would be broken.
                    if (!isSameSets(pFShells1, pFShells2))
                    {
                        continue;
                    }

                    if (bCheckNormals)
                    {
                        // get normal of checked face using the same parameter on edge
                        var aDN2 = new Direction3D();
                        if (GetNormalToSurface(aCheckedFace, edge, aTMid, ref aDN2))
                        {
                            // and check if the adjacent faces are having approximately same normals
                            double anAngle = aDN1.Angle(aDN2);
                            if (anAngle > myAngTol)
                            {
                                continue;
                            }
                        }
                    }
                    //
                    if (IsSameDomain(aFace, aCheckedFace, myLinTol, myAngTol, myFacePlaneMap))
                    {
                        if (AddOrdinaryEdges(edges, aCheckedFace, ref dummy, RemovedEdges))
                        {
                            // sequence edges is modified
                            i = dummy;
                        }

                        faces.Append(aCheckedFace);
                        aProcessed.Add(aCheckedFace);
                        break;
                    }
                }
            }

            if (faces.Length() > 1)
            {
                if (myFacePlaneMap.IsBound(faces[1]))
                {
                    PlaneSurface aPlane = myFacePlaneMap[faces[1]];
                    var        aLoc   = new Placement();
                    BB.UpdateFace(RefFace, aPlane, aLoc, global::Kerf.Engine.Foundation.Precision.Confusion());
                }
                // Add correct pcurves for the reference surface to the edges of other faces
                Face F_RefFace0 = RefFace;
                F_RefFace0.Orientation(Forward);
                AddPCurves(faces, F_RefFace0, MapEdgesWithTemporaryPCurves);

                // fill in the connectivity map for selected faces
                var aMapEF0 =
                  new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
                for (i = 1; i <= faces.Length(); i++)
                {
                    Exploration.MapShapesAndAncestors(faces[i], ShapeType.Edge, ShapeType.Face, aMapEF0);
                }
                // Collect keep edges and multi-connected edges, i.e. edges that are internal to
                // the set of selected faces and have connections to other faces.
                var aKeepEdges = new NCollection_List<Shape>();
                for (i = 1; i <= aMapEF0.Extent(); i++)
                {
                    NCollection_List<Shape> aLF = aMapEF0[i];
                    if (aLF.Extent() == 2)
                    {
                        Shape                   aE   = aMapEF0.FindKey(i);
                        NCollection_List<Shape> aGLF = theGMapEdgeFaces.FindFromKey(aE);
                        if (aGLF.Extent() > 2 || myKeepShapes.Contains(aE) || theFreeBoundMap.Contains(aE))
                        {
                            aKeepEdges.Append(aE);
                        }
                    }
                }
                if (!aKeepEdges.IsEmpty())
                {
                    if (!myAllowInternal)
                    {
                        // Remove from the selection the faces which have no other connect edges
                        // and contain multi-connected edges and/or keep edges.
                        var anAvoidFaces = new NCollection_Map<Shape>(ShapeHasher.Instance);
                        var it           = new NCollection_List<Shape>.Iterator(aKeepEdges);
                        for (; it.More(); it.Next())
                        {
                            Shape                   aE  = it.Value();
                            NCollection_List<Shape> aLF = aMapEF0.FindFromKey(aE);
                            anAvoidFaces.Add(aLF.First());
                            anAvoidFaces.Add(aLF.Last());
                        }
                        for (i = 1; i <= faces.Length(); i++)
                        {
                            if (anAvoidFaces.Contains(faces[i]))
                            {
                                // update the boundaries of merged area, for that
                                // remove from 'edges' the edges of this face and add to 'edges'
                                // the edges of this face that were not present in 'edges' before
                                bool hasConnectAnotherFaces = false;
                                var  ex                     = new Explorer(faces[i], ShapeType.Edge);
                                for (; ex.More() && !hasConnectAnotherFaces; ex.Next())
                                {
                                    Shape                   aE  = ex.Current();
                                    NCollection_List<Shape> aLF = aMapEF0.FindFromKey(aE);
                                    if (aLF.Extent() > 1)
                                    {
                                        for (it.Initialize(aLF); it.More() && !hasConnectAnotherFaces; it.Next())
                                        {
                                            if (!anAvoidFaces.Contains(it.Value()))
                                            {
                                                hasConnectAnotherFaces = true;
                                            }
                                        }
                                    }
                                }
                                if (!hasConnectAnotherFaces)
                                {
                                    AddOrdinaryEdges(edges, faces[i], ref dummy, RemovedEdges);
                                    faces.Remove(i);
                                    i--;
                                }
                            }
                        }
                        // check if the faces with keep edges contained in
                        // already updated the boundaries of merged area
                        if (!faces.IsEmpty())
                        {
                            var aMapFaces = new NCollection_Map<Shape>(ShapeHasher.Instance);
                            for (i = 1; i <= faces.Length(); i++)
                            {
                                aMapFaces.Add(faces[i]);
                            }
                            for (it.Initialize(aKeepEdges); it.More(); it.Next())
                            {
                                Shape                   aE  = it.Value();
                                NCollection_List<Shape> aLF = aMapEF0.FindFromKey(aE);
                                if (aLF.Extent() < 2)
                                {
                                    continue;
                                }
                                if (aMapFaces.Contains(aLF.First()) && aMapFaces.Contains(aLF.Last()))
                                {
                                    for (i = 1; i <= faces.Length(); i++)
                                    {
                                        if (faces[i].IsEqual(aLF.First()) || faces[i].IsEqual(aLF.Last()))
                                        {
                                            AddOrdinaryEdges(edges, faces[i], ref dummy, RemovedEdges);
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
                        var it = new NCollection_List<Shape>.Iterator(aKeepEdges);
                        for (; it.More(); it.Next())
                        {
                            Shape aE = it.Value();
                            edges.Append(aE.Oriented(Internal));
                        }
                    }
                } // if (!aKeepEdges.IsEmpty())
            } // if (faces.Length() > 1)

            var aMapEF =
              new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            for (i = 1; i <= faces.Length(); i++)
            {
                Exploration.MapShapesAndUniqueAncestors(faces[i], ShapeType.Edge, ShapeType.Face, aMapEF);
            }

            // Correct orientation of edges
            for (int ii = 1; ii <= edges.Length(); ii++)
            {
                ref Shape               anEdge = ref edges[ii];
                int                            indE   = aMapEF.FindIndex(anEdge);
                NCollection_List<Shape> aLF    = aMapEF[indE];
                if (myAllowInternal && myKeepShapes.Contains(anEdge) && aLF.Extent() == 2)
                {
                    edges[ii].Orientation(Internal);
                }

                if (anEdge.Orientation() != Internal)
                {
                    edges[ii] = aMapEF.FindKey(indE);
                }
            }

            // Exclude internal edges
            var InternalEdges = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
            int ind_e         = 1;
            while (ind_e <= edges.Length())
            {
                Shape anEdge = edges[ind_e];
                if (anEdge.Orientation() == Internal)
                {
                    InternalEdges.Add(anEdge);
                    edges.Remove(ind_e);
                }
                else
                {
                    ind_e++;
                }
            }

            if (RefFaceOrientation == Reversed)
            {
                for (int ii = 1; ii <= edges.Length(); ii++)
                {
                    edges[ii].Reverse();
                }
            }
            Face F_RefFace = RefFace;
            F_RefFace.Orientation(Forward);

            // all faces collected in the sequence. Perform union of faces
            if (faces.Length() > 1)
            {
                double CoordTol;
                var    edgesMap = new NCollection_Map<Shape>(ShapeHasher.Instance);
                CoordTol =  ComputeMinEdgeSize(edges, F_RefFace, edgesMap);
                CoordTol /= 10.0;
                CoordTol =  Max(CoordTol, global::Kerf.Engine.Foundation.Precision.Confusion());

                var VEmap =
                  new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
                for (int ind = 1; ind <= edges.Length(); ind++)
                {
                    Exploration.MapShapesAndUniqueAncestors(edges[ind], ShapeType.Vertex, ShapeType.Edge, VEmap);
                }

                // Try to find seam edge and an edge that is not seam but has 2 pcurves on the surface
                bool SeamFound, UseamFound = false, VseamFound = false;
                var  EdgeWith2pcurves = new Edge();
                for (int ii = 1; ii <= faces.Length(); ii++)
                {
                    Face face_ii     = ShapeCasts.Face(faces[ii]);
                    Wire anOuterWire = BrepTools.OuterWire(face_ii);
                    var         itw         = new Iterator(anOuterWire);
                    for (; itw.More(); itw.Next())
                    {
                        Edge anEdge = ShapeCasts.Edge(itw.Value());
                        if (Tool.IsClosed(anEdge, face_ii))
                        {
                            if (BrepTools.IsReallyClosed(anEdge, face_ii))
                            {
                                if (IsUiso(anEdge, face_ii))
                                {
                                    UseamFound = true;
                                }
                                else
                                {
                                    VseamFound = true;
                                }
                            }
                            else
                            {
                                EdgeWith2pcurves = anEdge;
                            }
                        }
                    }
                }
                SeamFound = UseamFound || VseamFound;

                bool aIsEdgeWith2pcurvesSmooth = false;
                if (myConcatBSplines && !EdgeWith2pcurves.IsNull && !SeamFound)
                {
                    NCollection_List<Shape> aFaceList = theGMapEdgeFaces.FindFromKey(EdgeWith2pcurves);
                    Face                    aFace1    = ShapeCasts.Face(aFaceList.First());
                    Face                    aFace2    = ShapeCasts.Face(aFaceList.Last());
                    Continuity anOrderOfCont = BRepLib.ContinuityOfFaces(EdgeWith2pcurves, aFace1, aFace2, myAngTol);
                    aIsEdgeWith2pcurvesSmooth = (anOrderOfCont >= Continuity.G1);
                }

                if (aIsEdgeWith2pcurvesSmooth)
                {
                    Curve2D aPC1 = Tool.CurveOnSurface(EdgeWith2pcurves, F_RefFace, out double aFirst, out double aLast)!;
                    EdgeWith2pcurves.Reverse();
                    Curve2D aPC2       = Tool.CurveOnSurface(EdgeWith2pcurves, F_RefFace, out aFirst, out aLast)!;
                    Point2D     aPnt1      = aPC1.Value(aFirst);
                    Point2D     aPnt2      = aPC2.Value(aFirst);
                    bool         anIsUclosed = (Math.Abs(aPnt1.X() - aPnt2.X()) > Math.Abs(aPnt1.Y() - aPnt2.Y()));
                    bool         aToMakeUPeriodic = false, aToMakeVPeriodic = false;
                    if (anIsUclosed && Uperiod == 0.0)
                    {
                        aToMakeUPeriodic = true;
                    }
                    if (!anIsUclosed && Vperiod == 0.0)
                    {
                        aToMakeVPeriodic = true;
                    }

                    if (aToMakeUPeriodic || aToMakeVPeriodic)
                    {
                        var  aBSplineSurface = aBaseSurface as BSplineSurface;
                        bool isPeriodicCopy  = false;
                        if (aBSplineSurface is null)
                        {
                            double        aTol   = 1.0e-4;
                            Continuity aUCont = Continuity.C1, aVCont = Continuity.C1;
                            int           degU   = 14, degV = 14;
                            int           nmax   = 16;
                            int           aPrec  = 1;
                            var Approximator = new ApproxSurface(aBaseSurface, aTol, aUCont, aVCont, degU, degV, nmax, aPrec);
                            aBSplineSurface = Approximator.Surface()!;
                            if (aBSplineSurface.IsFrozen)
                            {
                                // a new surface, but frozen by the approximation: its copy is the same new surface
                                aBSplineSurface = (BSplineSurface)aBSplineSurface.Copy();
                            }
                        }
                        else if (aBSplineSurface.IsFrozen)
                        {
                            // OCCT: makes the face's own surface periodic in place.
                            // OCCT.NET: frozen. OCCT makes the face's own surface periodic in place, so every shape
                            // sharing it sees a periodic surface, the input's faces included (occt-issues.md). The
                            // port makes a periodic copy and puts it in the original's place in the shape it works on
                            // (its copy of the input, see Build), which is what OCCT's edit does to that shape.
                            aBSplineSurface = (BSplineSurface)aBSplineSurface.Copy();
                            isPeriodicCopy  = true;
                        }

                        if (aToMakeUPeriodic)
                        {
                            aBSplineSurface.SetUPeriodic();
                            Uperiod = aBSplineSurface.UPeriod();
                        }
                        if (aToMakeVPeriodic)
                        {
                            aBSplineSurface.SetVPeriodic();
                            Vperiod = aBSplineSurface.VPeriod();
                        }

                        if (isPeriodicCopy)
                        {
                            ReplaceSurface(aBaseSurface, aBSplineSurface, [myShape, RefFace, .. myContext.InternalReplacements()]);
                            aBaseSurface = aBSplineSurface;
                        }

                        // Update ref face and pcurves if the surface changed
                        if (!ReferenceEquals(aBSplineSurface, aBaseSurface))
                        {
                            Face   OldRefFace = RefFace;
                            Curve2D? NullPCurve = null;
                            RefFace.Nullify();
                            BB.MakeFace(ref RefFace, aBSplineSurface, aBaseLocation, 0.0);
                            for (int ii = 1; ii <= edges.Length(); ii++)
                            {
                                Edge   anEdge  = ShapeCasts.Edge(edges[ii]);
                                Curve2D? aPCurve = Tool.CurveOnSurface(anEdge, OldRefFace, out aFirst, out aLast);
                                if (MapEdgesWithTemporaryPCurves.Contains(anEdge))
                                {
                                    BB.UpdateEdge(anEdge, NullPCurve, OldRefFace, 0.0);
                                }
                                BB.UpdateEdge(anEdge, aPCurve, RefFace, 0.0);
                            }
                            F_RefFace = RefFace;
                            F_RefFace.Orientation(Forward);
                        }
                    }
                } // if (myConcatBSplines && !EdgeWith2pcurves.IsNull() && !SeamFound)

                // Perform relocating to new U-origin
                // Define boundaries in 2d space of RefFace
                Span<double> aPeriods      = [Uperiod, Vperiod];
                Span<bool>   anIsSeamFound = [UseamFound, VseamFound];
                Span<double> aSurfMin      = [0.0, 0.0];
                Span<double> aSurfMax      = [0.0, 0.0];
                aBaseSurface.Bounds(out aSurfMin[0], out aSurfMax[0], out aSurfMin[1], out aSurfMax[1]);

                for (int ii = 0; ii < 2; ii++)
                {
                    if (aPeriods[ii] != 0.0)
                    {
                        // if seam edge exists, do nothing
                        if (!anIsSeamFound[ii])
                        {
                            // try to find the origin of U in 2d space
                            // so that all the faces are in [origin, origin + Uperiod]
                            double aMinCoord = 0.0, aMaxCoord = 0.0; // Umin, Umax;
                            int    aNumberOfIntervals = 0, i_face_max = 0;
                            if (!FindCoordBounds(faces,
                                                 F_RefFace,
                                                 aMapEF,
                                                 edgesMap,
                                                 ii + 1,
                                                 aPeriods[ii],
                                                 ref aMinCoord,
                                                 ref aMaxCoord,
                                                 ref aNumberOfIntervals,
                                                 ref i_face_max))
                            {
                                break;
                            }

                            if (aMaxCoord - aMinCoord > aPeriods[ii] - 1.0e-5)
                            {
                                anIsSeamFound[ii] = true;
                            }
                            else if (aNumberOfIntervals == 2)
                            {
                                var UsedEdges0    = new NCollection_Map<Shape>(ShapeHasher.Instance);
                                var EdgeNewPCurve = new NCollection_DataMap<Shape, Curve2D>(EqualityComparer<Shape>.Default);

                                // Relocate pcurves to new U-origin
                                if (!RelocatePCurvesToNewUorigin(edges,
                                                                 faces[i_face_max],
                                                                 F_RefFace,
                                                                 CoordTol,
                                                                 ii + 1,
                                                                 aPeriods[ii],
                                                                 VEmap,
                                                                 EdgeNewPCurve,
                                                                 UsedEdges0))
                                {
                                    VEmap.Clear();
                                    for (int ind = 1; ind <= edges.Length(); ind++)
                                    {
                                        Exploration.MapShapesAndUniqueAncestors(edges[ind], ShapeType.Vertex, ShapeType.Edge, VEmap);
                                    }
                                    break;
                                }

                                // PCurves from unused edges (may be degenerated edges)
                                for (int ind = 1; ind <= edges.Length(); ind++)
                                {
                                    Edge anEdge = ShapeCasts.Edge(edges[ind]);
                                    if (!UsedEdges0.Contains(anEdge))
                                    {
                                        Curve2D? aPCurve =
                                          Tool.CurveOnSurface(anEdge, F_RefFace, out double fpar, out double lpar);
                                        if (aPCurve is null)
                                        {
                                            EdgeNewPCurve.Clear();
                                            break;
                                        }
                                        aPCurve = new TrimmedCurve2D(aPCurve, fpar, lpar);
                                        EdgeNewPCurve.Bind(anEdge, aPCurve);
                                    }
                                }
                                if (EdgeNewPCurve.Extent() != edges.Length())
                                {
                                    break;
                                }

                                // Restore VEmap
                                VEmap.Clear();
                                for (int ind = 1; ind <= edges.Length(); ind++)
                                {
                                    Exploration.MapShapesAndUniqueAncestors(edges[ind], ShapeType.Vertex, ShapeType.Edge, VEmap);
                                }

                                // Find NewUmin and NewUmax
                                double NewCoordMin = RealLast(), NewCoordMax = RealFirst();
                                for (int jj = 1; jj <= edges.Length(); jj++)
                                {
                                    Curve2D aPCurve = EdgeNewPCurve[edges[jj]];
                                    UpdateBoundaries(aPCurve,
                                                     aPCurve.FirstParameter(),
                                                     aPCurve.LastParameter(),
                                                     ii + 1,
                                                     ref NewCoordMin,
                                                     ref NewCoordMax);
                                }

                                if (NewCoordMax - NewCoordMin < aPeriods[ii] - CoordTol
                                    && (-global::Kerf.Engine.Foundation.Precision.Confusion() >= NewCoordMin
                                        || NewCoordMin >= aPeriods[ii] + global::Kerf.Engine.Foundation.Precision.Confusion()
                                        || -global::Kerf.Engine.Foundation.Precision.Confusion() >= NewCoordMax
                                        || NewCoordMax >= aPeriods[ii] + global::Kerf.Engine.Foundation.Precision.Confusion()))
                                {
                                    // we can build a face without seam edge:
                                    // update the edges with earlier computed relocated pcurves
                                    // fitting into (NewUorigin, NewUorigin + Uperiod)
                                    double RestSpaceInCoord = aPeriods[ii] - (NewCoordMax - NewCoordMin);
                                    double NewCoordOrigin   = NewCoordMin - RestSpaceInCoord / 2;
                                    if (NewCoordOrigin < aSurfMin[ii])
                                    {
                                        NewCoordOrigin = aSurfMin[ii];
                                    }
                                    Surface NewSurf;
                                    bool         anIsInU = (ii == 0);
                                    if (NewCoordOrigin == aSurfMin[ii])
                                    {
                                        NewSurf = aBaseSurface;
                                    }
                                    else
                                    {
                                        NewSurf = new TrimmedSurface(aBaseSurface,
                                                                                     NewCoordOrigin,
                                                                                     NewCoordOrigin + aPeriods[ii],
                                                                                     anIsInU); // trim in U or V
                                    }
                                    Face   OldRefFace = RefFace;
                                    Curve2D? NullPCurve = null;
                                    RefFace.Nullify();
                                    BB.MakeFace(ref RefFace, NewSurf, aBaseLocation, 0.0);
                                    for (int jj = 1; jj <= edges.Length(); jj++)
                                    {
                                        Edge anEdge = ShapeCasts.Edge(edges[jj]);
                                        if (MapEdgesWithTemporaryPCurves.Contains(anEdge))
                                        {
                                            BB.UpdateEdge(anEdge, NullPCurve, OldRefFace, 0.0);
                                        }
                                        Curve2D aPCurve = EdgeNewPCurve[anEdge];
                                        BB.UpdateEdge(anEdge, aPCurve, RefFace, 0.0);
                                    }
                                }
                            } // else (Umax - Umin < Uperiod - 1.e-5, no Useam)
                        } // if (!UseamFound)
                    } // if (Uperiod != 0.)
                }
                UseamFound = anIsSeamFound[0];
                VseamFound = anIsSeamFound[1];
                ////////////////////////////////////
                F_RefFace = RefFace;
                F_RefFace.Orientation(Forward);

                var NewFaces = new NCollection_Sequence<Shape>();
                var NewWires = new NCollection_Sequence<Shape>();

                if (Uperiod == 0 || Vperiod == 0)
                {
                    // Set the "periods" for closed non-periodic surface
                    Surface aSurf = Tool.Surface(RefFace, out Placement _)!;
                    if (aSurf is TrimmedSurface aRTS)
                    {
                        aSurf = aRTS.BasisSurface();
                    }
                    aSurf.Bounds(out double Ufirst, out double Ulast, out double Vfirst, out double Vlast);
                    if (Uperiod == 0 && aSurf.IsUClosed)
                    {
                        Uperiod = Ulast - Ufirst;
                    }
                    if (Vperiod == 0 && aSurf.IsVClosed)
                    {
                        Vperiod = Vlast - Vfirst;
                    }
                }

                var  UsedEdges            = new NCollection_Map<Shape>(ShapeHasher.Instance);
                bool isWireBuildingFailed = false;

                double FaceUmin = RealLast();
                double FaceVmin = RealLast();
                for (int ii = 1; ii <= edges.Length(); ii++)
                {
                    Edge anEdge   = ShapeCasts.Edge(edges[ii]);
                    var         aBAcurve = new EdgeCurve2DAdaptor(anEdge, F_RefFace);
                    if (aBAcurve.Curve() is null)
                    {
                        continue;
                    }

                    Point2D aFirstPoint = aBAcurve.Value(aBAcurve.FirstParameter());
                    Point2D aLastPoint  = aBAcurve.Value(aBAcurve.LastParameter());

                    if (aFirstPoint.X() < FaceUmin)
                    {
                        FaceUmin = aFirstPoint.X();
                    }
                    if (aLastPoint.X() < FaceUmin)
                    {
                        FaceUmin = aLastPoint.X();
                    }

                    if (aFirstPoint.Y() < FaceVmin)
                    {
                        FaceVmin = aFirstPoint.Y();
                    }
                    if (aLastPoint.Y() < FaceVmin)
                    {
                        FaceVmin = aLastPoint.Y();
                    }
                }

                // Building new wires from <edges>
                // and build faces
                while (!edges.IsEmpty())
                {
                    // try to find non-degenerated edge
                    Edge StartEdge = ShapeCasts.Edge(edges[1]);
                    int         istart    = 1;
                    while (Tool.Degenerated(StartEdge) && istart < edges.Length())
                    {
                        istart++;
                        StartEdge = ShapeCasts.Edge(edges[istart]);
                    }

                    var aNewWire = new Wire();
                    BB.MakeWire(ref aNewWire);
                    BB.Add(aNewWire, StartEdge);
                    RemoveEdgeFromMap(StartEdge, VEmap);
                    var SplittingVertices = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);

                    double        fpar, lpar;
                    Curve2D? StartPCurve = Tool.CurveOnSurface(StartEdge, F_RefFace, out fpar, out lpar);
                    if (StartPCurve is null)
                    {
                        edges.Remove(istart);
                        continue;
                    }
                    Exploration.Vertices(StartEdge, out Vertex StartVertex, out Vertex CurVertex, true); // with orientation
                    double StartParam, CurParam;
                    if (StartEdge.Orientation() == Forward)
                    {
                        StartParam = fpar;
                        CurParam   = lpar;
                    }
                    else
                    {
                        StartParam = lpar;
                        CurParam   = fpar;
                    }
                    Point2D StartPoint = StartPCurve.Value(StartParam);
                    Point2D CurPoint   = StartPCurve.Value(CurParam);

                    Edge CurEdge = StartEdge;
                    for (;;) // loop till the end of current new wire
                    {
                        var NextEdge  = new Edge();
                        var NextPoint = new Point2D();

                        var Elist = new NCollection_List<Shape>();
                        // const NCollection_List<TopoDS_Shape>& Elist = VEmap.FindFromKey(CurVertex);
                        if (VEmap.Contains(CurVertex))
                        {
                            Elist = new NCollection_List<Shape>(VEmap.FindFromKey(CurVertex));
                        }
                        var itl = new NCollection_List<Shape>.Iterator(Elist);
                        if (Elist.IsEmpty())
                        {
                            if (CurVertex.IsSame(StartVertex))
                            {
                                // Points of two vertices coincide in 3d but may be not in 2d
                                if ((Uperiod != 0.0 && Math.Abs(StartPoint.X() - CurPoint.X()) > Uperiod / 2)
                                    || (Vperiod != 0.0
                                        && Math.Abs(StartPoint.Y() - CurPoint.Y()) > Vperiod / 2)) // end of parametric space
                                {
                                    //<edges> do not contain seams => we must reconstruct the seam up to <NextEdge>
                                    ReconstructMissedSeam(RemovedEdges,
                                                          F_RefFace,
                                                          CurEdge,
                                                          CurVertex,
                                                          CurPoint,
                                                          Uperiod,
                                                          Vperiod,
                                                          ref NextEdge,
                                                          ref NextPoint);
                                }
                                else
                                {
                                    break; // end of wire
                                }
                            }
                        }

                        if (NextEdge.IsNull)
                        {
                            bool EndOfWire = false;

                            bool anIsOnSingularity = IsOnSingularity(Elist);
                            if (!anIsOnSingularity && Elist.Extent() > 1)
                            {
                                SplittingVertices.Add(CurVertex);
                            }

                            var TmpElist  = new NCollection_List<Shape>();
                            var TrueElist = new NCollection_List<Shape>();
                            //<TrueElist> will be the list of candidates to become <NextEdge>
                            for (itl.Initialize(Elist); itl.More(); itl.Next())
                            {
                                Edge anEdge = ShapeCasts.Edge(itl.Value());
                                if (UsedEdges.Contains(anEdge))
                                {
                                    continue;
                                }
                                Vertex aFirstVertex = Exploration.FirstVertex(anEdge, true);
                                if (!aFirstVertex.IsSame(CurVertex))
                                {
                                    continue;
                                }
                                TmpElist.Append(anEdge);
                            }
                            Curve2D? CurPCurve = Tool.CurveOnSurface(CurEdge, F_RefFace, out fpar, out lpar);
                            if (TmpElist.Extent() <= 1 || (Uperiod != 0.0 || Vperiod != 0) || CurPCurve is null)
                            {
                                TrueElist.Assign(TmpElist);
                            }
                            else
                            {
                                // we must choose the closest direction - the biggest angle
                                double MaxAngle = RealFirst();
                                var    TrueEdge = new Edge();
                                CurParam = (CurEdge.Orientation() == Forward) ? lpar : fpar;
                                Curve2D.ResD1 aCurD1 = CurPCurve.EvalD1(CurParam);
                                CurPoint = aCurD1.Point;
                                Vector2D CurDir = aCurD1.D1;
                                CurDir.Normalize();
                                if (CurEdge.Orientation() == Reversed)
                                {
                                    CurDir.Reverse();
                                }
                                for (itl.Initialize(TmpElist); itl.More(); itl.Next())
                                {
                                    Edge   anEdge  = ShapeCasts.Edge(itl.Value());
                                    Curve2D? aPCurve = Tool.CurveOnSurface(anEdge, F_RefFace, out fpar, out lpar);
                                    if (aPCurve is null)
                                    {
                                        // edge has no pcurve on the reference face, skip it as a direction candidate
                                        continue;
                                    }
                                    double   aParam = (anEdge.Orientation() == Forward) ? fpar : lpar;
                                    Vector2D aDir   = aPCurve.EvalD1(aParam).D1;
                                    aDir.Normalize();
                                    if (anEdge.Orientation() == Reversed)
                                    {
                                        aDir.Reverse();
                                    }
                                    double anAngle = CurDir.Angle(aDir);
                                    if (anAngle > MaxAngle)
                                    {
                                        MaxAngle = anAngle;
                                        TrueEdge = anEdge;
                                    }
                                }
                                if (!TrueEdge.IsNull)
                                {
                                    TrueElist.Append(TrueEdge);
                                }
                            }

                            // Find next edge in TrueElist
                            for (itl.Initialize(TrueElist); itl.More(); itl.Next())
                            {
                                Edge anEdge = ShapeCasts.Edge(itl.Value());

                                Curve2D? aPCurve = Tool.CurveOnSurface(anEdge, F_RefFace, out fpar, out lpar);
                                if (aPCurve is null)
                                {
                                    // edge has no pcurve on the reference face, skip it as a next-edge candidate
                                    continue;
                                }
                                double   aParam = (anEdge.Orientation() == Forward) ? fpar : lpar;
                                Point2D aPoint = aPCurve.Value(aParam);
                                double   DiffU  = Math.Abs(aPoint.X() - CurPoint.X());
                                double   DiffV  = Math.Abs(aPoint.Y() - CurPoint.Y());
                                if (Uperiod != 0.0 && DiffU > CoordTol && Math.Abs(DiffU - Uperiod) > CoordTol)
                                { // may be it is a deg.vertex
                                    continue;
                                }
                                if (Vperiod != 0.0 && DiffV > CoordTol && Math.Abs(DiffV - Vperiod) > CoordTol)
                                { // may be it is a deg.vertex
                                    continue;
                                }

                                // Check: may be <CurPoint> and <aPoint> are on Period from each other
                                if ((Uperiod != 0.0 && DiffU > Uperiod / 2)
                                    || (Vperiod != 0.0 && DiffV > Vperiod / 2)) // end of parametric space
                                {
                                    //<edges> do not contain seams => we must reconstruct the seam up to <NextEdge>
                                    var StartOfNextEdge  = new Point2D();
                                    var LastVertexOfSeam = new Vertex();
                                    ReconstructMissedSeam(RemovedEdges,
                                                          F_RefFace,
                                                          CurEdge,
                                                          CurVertex,
                                                          CurPoint,
                                                          Uperiod,
                                                          Vperiod,
                                                          ref NextEdge,
                                                          ref NextPoint);

                                    // Check: may be it is the end
                                    // OCCT: LastVertexOfSeam and StartOfNextEdge are never set, so this never holds
                                    if (LastVertexOfSeam.IsSame(StartVertex)
                                        && Math.Abs(StartPoint.X() - StartOfNextEdge.X()) < Uperiod / 2)
                                    {
                                        EndOfWire = true;
                                    }

                                    break;
                                }
                                else
                                {
                                    NextEdge = anEdge;
                                    double LastParam = (NextEdge.Orientation() == Forward) ? lpar : fpar;
                                    NextPoint = aPCurve.Value(LastParam);
                                    break;
                                }
                            } // for (itl.Initialize(TrueElist); itl.More(); itl.Next())

                            if (EndOfWire)
                            {
                                break;
                            }
                        }

                        if (NextEdge.IsNull)
                        {
                            if (Uperiod != 0.0 || Vperiod != 0.0)
                            {
                                if (CurVertex.IsSame(StartVertex)
                                    && (Uperiod == 0.0 || Math.Abs(StartPoint.X() - CurPoint.X()) < Uperiod / 2)
                                    && (Vperiod == 0.0 || Math.Abs(StartPoint.Y() - CurPoint.Y()) < Vperiod / 2))
                                {
                                    break; // end of wire
                                }

                                ReconstructMissedSeam(RemovedEdges,
                                                      F_RefFace,
                                                      CurEdge,
                                                      CurVertex,
                                                      CurPoint,
                                                      Uperiod,
                                                      Vperiod,
                                                      ref NextEdge,
                                                      ref NextPoint);
                                if (NextEdge.IsNull)
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
                        CurPoint  = NextPoint;
                        CurEdge   = NextEdge;
                        CurVertex = Exploration.LastVertex(CurEdge, true); // with orientation
                        BB.Add(aNewWire, CurEdge);
                        UsedEdges.Add(CurEdge);
                        RemoveEdgeFromMap(CurEdge, VEmap);
                    } // for (;;)

                    if (isWireBuildingFailed)
                    {
                        break;
                    }

                    aNewWire.Closed(true);
                    UsedEdges.Add(StartEdge);

                    // Remove used edges from sequence
                    int ind = 1;
                    while (ind <= edges.Length())
                    {
                        if (UsedEdges.Contains(edges[ind]))
                        {
                            edges.Remove(ind);
                        }
                        else
                        {
                            ind++;
                        }
                    }

                    // add just built wire to current face or save it in the sequence of wires
                    bool EdgeOnBoundOfSurfFound = false;
                    var  itw                    = new Iterator(aNewWire);
                    for (; itw.More(); itw.Next())
                    {
                        Edge anEdge = ShapeCasts.Edge(itw.Value());
                        if (Tool.IsClosed(anEdge, RefFace))
                        {
                            EdgeOnBoundOfSurfFound = true;
                            break;
                        }
                    }
                    if (EdgeOnBoundOfSurfFound) // this wire can not be a hole
                    {
                        Surface? aSurf   = Tool.Surface(RefFace, out Placement aLoc);
                        var           aResult = new Face();
                        BB.MakeFace(ref aResult, aSurf, aLoc, 0);
                        BB.Add(aResult, aNewWire);
                        aResult.Orientation(RefFaceOrientation);
                        NewFaces.Append(aResult);
                    }
                    else // may be this wire is a hole
                    {
                        // split this wire if needed
                        if (!SplittingVertices.IsEmpty())
                        {
                            SplitWire(aNewWire, F_RefFace, SplittingVertices, NewWires);
                        }
                        else
                        {
                            NewWires.Append(aNewWire);
                        }
                    }
                } // while (!edges.IsEmpty())

                if (isWireBuildingFailed)
                {
                    continue;
                }

                // Build wires from internal edges
                var IntVEmap =
                  new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
                for (int ii = 1; ii <= InternalEdges.Extent(); ii++)
                {
                    Exploration.MapShapesAndAncestors(InternalEdges[ii], ShapeType.Vertex, ShapeType.Edge, IntVEmap);
                }
                var InternalWires = new NCollection_Sequence<Shape>();
                while (!InternalEdges.IsEmpty())
                {
                    Edge aFirstEdge = ShapeCasts.Edge(InternalEdges[1]);
                    InternalEdges.RemoveFromIndex(1);
                    RemoveEdgeFromMap(aFirstEdge, IntVEmap);
                    var anInternalWire = new Wire();
                    BB.MakeWire(ref anInternalWire);
                    BB.Add(anInternalWire, aFirstEdge);
                    var EndEdges = new Edge[2];
                    EndEdges[0] = EndEdges[1] = aFirstEdge;
                    var VV = new Vertex[2];
                    Exploration.Vertices(aFirstEdge, out VV[0], out VV[1]);
                    for (;;)
                    {
                        if (VV[0].IsSame(VV[1]))
                        { // closed wire
                            break;
                        }
                        bool found = false;
                        for (int ii = 0; ii < 2; ii++)
                        {
                            NCollection_List<Shape> Elist = IntVEmap.FindFromKey(VV[ii]);
                            var                            itl   = new NCollection_List<Shape>.Iterator(Elist);
                            for (; itl.More(); itl.Next())
                            {
                                Edge anEdge = ShapeCasts.Edge(itl.Value());
                                if (anEdge.IsSame(EndEdges[ii]))
                                {
                                    continue;
                                }
                                found = true;
                                InternalEdges.RemoveKey(anEdge);
                                RemoveEdgeFromMap(anEdge, IntVEmap);
                                BB.Add(anInternalWire, anEdge);
                                Exploration.Vertices(anEdge, out Vertex V1, out Vertex V2);
                                VV[ii]       = (V1.IsSame(VV[ii])) ? V2 : V1;
                                EndEdges[ii] = anEdge;
                                break;
                            }
                        }
                        if (!found)
                        { // end of open wire
                            break;
                        }
                    }
                    InternalWires.Append(anInternalWire);
                }

                // Insert new faces instead of old ones
                if (NewFaces.IsEmpty())
                {
                    // one face without seam
                    Surface? aSurf   = Tool.Surface(RefFace, out Placement aLoc);
                    var           aResult = new Face();
                    BB.MakeFace(ref aResult, aSurf, aLoc, 0.0);
                    for (int ii = 1; ii <= NewWires.Length(); ii++)
                    {
                        BB.Add(aResult, NewWires[ii]);
                    }
                    for (int ii = 1; ii <= InternalWires.Length(); ii++)
                    {
                        BB.Add(aResult, InternalWires[ii]);
                    }
                    aResult.Orientation(RefFaceOrientation);
                    myContext.Merge(faces, aResult);
                    // Update the map Face-NewFace
                    for (int jj = 1; jj <= faces.Length(); jj++)
                    {
                        myFaceNewFace.Bind(faces[jj], aResult);
                    }
                }
                else if (NewFaces.Length() == 1)
                {
                    Shape aNewFace = NewFaces[1].Oriented(Forward);
                    for (int ii = 1; ii <= NewWires.Length(); ii++)
                    {
                        BB.Add(aNewFace, NewWires[ii]);
                    }
                    for (int ii = 1; ii <= InternalWires.Length(); ii++)
                    {
                        BB.Add(aNewFace, InternalWires[ii]);
                    }
                    myContext.Merge(faces, NewFaces[1]);
                    // Update the map Face-NewFace
                    for (int jj = 1; jj <= faces.Length(); jj++)
                    {
                        myFaceNewFace.Bind(faces[jj], NewFaces[1]);
                    }
                }
                else
                {
                    // Insert new wires and internal wires into correspondent faces
                    InsertWiresIntoFaces(NewWires, NewFaces, RefFace);
                    InsertWiresIntoFaces(InternalWires, NewFaces, RefFace);

                    var Emaps = new NCollection_Sequence<NCollection_Map<Shape>>();
                    for (int ii = 1; ii <= faces.Length(); ii++)
                    {
                        var aEmap = new NCollection_Map<Shape>(ShapeHasher.Instance);
                        Exploration.MapShapes(faces[ii], aEmap);
                        Emaps.Append(aEmap);
                    }
                    for (int ii = 1; ii <= NewFaces.Length(); ii++)
                    {
                        var facesForThisFace = new NCollection_Sequence<Shape>();
                        var UsedFaces        = new NCollection_Map<Shape>(ShapeHasher.Instance);
                        var Explo            = new Explorer(NewFaces[ii], ShapeType.Edge);
                        for (; Explo.More(); Explo.Next())
                        {
                            Edge anEdge = ShapeCasts.Edge(Explo.Current());
                            if (Tool.Degenerated(anEdge) || Tool.IsClosed(anEdge, RefFace))
                            {
                                continue;
                            }
                            int jj;
                            for (jj = 1; jj <= Emaps.Length(); jj++)
                            {
                                if (Emaps[jj].Contains(anEdge))
                                {
                                    break;
                                }
                            }
                            if (UsedFaces.Add(faces[jj]))
                            {
                                facesForThisFace.Append(faces[jj]);
                            }
                        }
                        myContext.Merge(facesForThisFace, NewFaces[ii]);
                        // Update the map Face-NewFace
                        for (int jj = 1; jj <= facesForThisFace.Length(); jj++)
                        {
                            myFaceNewFace.Bind(facesForThisFace[jj], NewFaces[ii]);
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
        Shape aRes = myContext.InternalApply(myShape);
        // creating map of edge faces
        var aMapEdgeFaces =
          new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aRes, ShapeType.Edge, ShapeType.Face, aMapEdgeFaces);
        // creating map of vertex edges
        var aMapEdgesVertex =
          new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndUniqueAncestors(aRes, ShapeType.Vertex, ShapeType.Edge, aMapEdgesVertex);
        // creating map of vertex faces
        var aVFmap =
          new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndUniqueAncestors(aRes, ShapeType.Vertex, ShapeType.Face, aVFmap);

        if (mySafeInputMode)
        {
            UpdateMapOfShapes(myKeepShapes, myContext);
        }

        // Sequence of the edges of the shape
        var aSeqEdges = new NCollection_Sequence<Shape>();
        int aNbE      = aMapEdgeFaces.Extent();
        for (int i = 1; i <= aNbE; ++i)
        {
            aSeqEdges.Append(aMapEdgeFaces.FindKey(i));
        }

        // Prepare map of shared vertices (with the number of connected edges greater then 2)
        var aSharedVert = new NCollection_Map<Shape>(ShapeHasher.Instance);
        CheckSharedVertices(aSeqEdges, aMapEdgesVertex, myKeepShapes, aSharedVert);
        // Merge the edges avoiding removal of the shared vertices
        bool isMerged = MergeSeq(aSeqEdges, aVFmap, aSharedVert);
        // Collect faces to rebuild
        var aChangedFaces = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        if (isMerged)
        {
            for (int i = 1; i <= aNbE; ++i)
            {
                Shape aE = aMapEdgeFaces.FindKey(i);
                if (myContext.IsRecorded(aE))
                {
                    var it = new NCollection_List<Shape>.Iterator(aMapEdgeFaces[i]);
                    for (; it.More(); it.Next())
                    {
                        aChangedFaces.Add(it.Value());
                    }
                }
            }
        }

        // fix changed faces and replace them in the local context
        double aPrec = global::Kerf.Engine.Foundation.Precision.Confusion();
        for (int i = 1; i <= aChangedFaces.Extent(); i++)
        {
            Face aFace = ShapeCasts.Face(myContext.InternalApply(aChangedFaces.FindKey(i)));
            if (aFace.IsNull)
            {
                continue;
            }

            // for a planar face create and store pcurve of edge on face
            // to speed up all operations; but this is allowed only when non-safe mode in force
            if (!mySafeInputMode)
            {
                Surface? aSurface = Tool.Surface(aFace, out Placement _);
                aSurface = ClearRts(aSurface);
                if (aSurface is PlaneSurface)
                {
                    var aLE = new NCollection_List<Shape>();
                    for (var anEx = new Explorer(aFace, ShapeType.Edge); anEx.More(); anEx.Next())
                    {
                        aLE.Append(anEx.Current());
                    }
                    BRepLib.BuildPCurveForEdgesOnPlane(aLE, aFace);
                }
            }

            var sff = new ShapeFix_Face(aFace);
            if (mySafeInputMode)
            {
                sff.SetContext(myContext);
            }
            sff.SetPrecision(aPrec);
            sff.SetMinTolerance(aPrec);
            sff.SetMaxTolerance(Max(1.0, aPrec * 1000.0));
            sff.FixOrientationMode()         = 0;
            sff.FixAddNaturalBoundMode()     = 0;
            sff.FixIntersectingWiresMode()   = 0;
            sff.FixLoopWiresMode()           = 0;
            sff.FixSplitFaceMode()           = 0;
            sff.FixPeriodicDegeneratedMode() = 0;
            SetFixWireModes(sff);
            sff.Perform();
            Shape aNewFace = sff.InternalFace();
            myContext.Replace(aFace, aNewFace);
        }

        if (aChangedFaces.Extent() > 0)
        {
            // fix changed shell and replace it in the local context
            Shape aRes1     = myContext.InternalApply(aRes);
            bool         isChanged = false;
            var          expsh     = new Explorer();
            for (expsh.Init(aRes1, ShapeType.Shell); expsh.More(); expsh.Next())
            {
                Shell aShell = ShapeCasts.Shell(expsh.Current());
                var          sfsh   = new ShapeFix_Shell();
                sfsh.FixFaceOrientation(aShell);
                Shape aNewShell = sfsh.InternalShell();
                if (!aNewShell.IsSame(aShell))
                {
                    myContext.Replace(aShell, aNewShell);
                    isChanged = true;
                }
            }
            if (isChanged)
            {
                aRes1 = myContext.InternalApply(aRes1);
            }
            myContext.Replace(myShape, aRes1);
        }

        myShape = myContext.InternalApply(myShape);
    }

    /// <summary>Fills the history of the modifications during the operation.</summary>
    protected void FillHistory()
    {
        if (myHistory is null)
        {
            // History is not requested
            return;
        }

        // Only Vertices, Edges and Faces can be modified during unification.
        // Thus, only these kind of shapes should be checked.

        // Get history from the context.
        // It contains all modifications of the operation. Some of these
        // modifications become not relevant and should be filtered.
        History aCtxHistory = myContext.InternalHistory();

        // Explore the history of the context and fill
        // the history of UnifySameDomain algorithm
        var aUSDHistory = new History();

        // Map all Vertices, Edges, Faces and Solids in the input shape
        var aMapInputShape = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(myInitShape, ShapeType.Vertex, aMapInputShape);
        Exploration.MapShapes(myInitShape, ShapeType.Edge, aMapInputShape);
        Exploration.MapShapes(myInitShape, ShapeType.Face, aMapInputShape);
        Exploration.MapShapes(myInitShape, ShapeType.Solid, aMapInputShape);

        // Map all Vertices, Edges, Faces and Solids in the result shape
        var aMapResultShapes = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(myShape, ShapeType.Vertex, aMapResultShapes);
        Exploration.MapShapes(myShape, ShapeType.Edge, aMapResultShapes);
        Exploration.MapShapes(myShape, ShapeType.Face, aMapResultShapes);
        Exploration.MapShapes(myShape, ShapeType.Solid, aMapResultShapes);

        // Iterate on all input shapes and get their modifications
        int i, aNb = aMapInputShape.Extent();
        for (i = 1; i <= aNb; ++i)
        {
            Shape aS = aMapInputShape[i];

            // Check the shape itself to be present in the result
            if (aMapResultShapes.Contains(aS))
            {
                // The shape is present in the result as is, thus has not been modified
                continue;
            }

            // Check if the shape has been modified during the operation
            var aLSImages = aCtxHistory.Modified(aS);
            if (aLSImages.Count == 0)
            {
                // The shape has not been modified and not present in the result,
                // thus it has been removed
                aUSDHistory.Remove(aS);
                continue;
            }

            // Check the images of the shape to be present in the result
            bool bRemoved = true;
            foreach (Shape aSIm in aLSImages)
            {
                if (aMapResultShapes.Contains(aSIm))
                {
                    if (!aSIm.IsSame(aS))
                    {
                        // Image is found in the result, thus the shape has been modified
                        aUSDHistory.AddModified(aS, aSIm);
                    }
                    bRemoved = false;
                }
            }

            if (bRemoved)
            {
                // No images are found in the result, thus the shape has been removed
                aUSDHistory.Remove(aS);
            }
        }

        // Merge the history of the operation into global history
        // OCCT.NET: frozen. Build merges it, composed onto the caller's sub-shapes, once the copy at the entry is
        // finished (BRepTools_CopyAtEntry.Composed); for an input worked on in place that is OCCT's merge.
        myUSDHistory = aUSDHistory;
    }
}
