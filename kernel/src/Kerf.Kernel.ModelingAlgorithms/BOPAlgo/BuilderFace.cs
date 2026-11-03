// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderFace.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Classification;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans;

/// <summary>
/// Builds the faces bounded by a set of edges on the surface of a face (the split parts of the face): the loops are
/// made by BOPAlgo_WireSplitter, classified as growths and holes, the holes put into the growths that contain them, and
/// the edges left over added as internal wires.
/// </summary>
/// <remarks>
/// The new faces are built on the face's surface, location and tolerance. The given face is kept forward; its own
/// orientation is <see cref="Orientation"/>.
/// </remarks>
public class BuilderFace : BuilderArea
{
    protected Face _face = new();
    protected Orientation _orientation;

    public BuilderFace()
    {
        _orientation = External;
    }

    /// <summary>Sets the face generatrix (kept forward; its orientation is <see cref="Orientation"/>).</summary>
    public void SetFace(in Face face)
    {
        _orientation = face.Orientation();
        _face = face;
        _face.Orientation(Forward);
    }

    /// <summary>The face generatrix.</summary>
    public Face Face() => _face;

    /// <summary>The orientation of the face generatrix.</summary>
    public Orientation Orientation() => _orientation;

    protected override void CheckData()
    {
        if (_face.IsNull)
        {
            AddError(new AlertNullInputShapes());
            return;
        }
        if (_context is null)
        {
            _context = new IntersectionContext();
        }
    }

    /// <summary>Performs the algorithm.</summary>
    public override void Perform(ProgressRange range = default)
    {
        using var ps = new ProgressScope(range, null, 100);

        GetReport().Clear();
        //
        CheckData();
        if (HasErrors)
        {
            return;
        }
        //
        PerformShapesToAvoid(ps.Next(1));
        if (HasErrors)
        {
            return;
        }
        //
        PerformLoops(ps.Next(10));
        if (HasErrors)
        {
            return;
        }
        //
        PerformAreas(ps.Next(80));
        if (HasErrors)
        {
            return;
        }
        //
        PerformInternalShapes(ps.Next(9));
    }

    /// <summary>Collects the edges that can't be in a loop: free edges and edges given twice.</summary>
    protected override void PerformShapesToAvoid(ProgressRange range)
    {
        bool bFound;
        int i, nbV, nbE;
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mve = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator it = new(_shapes);
        //
        _shapesToAvoid.Clear();
        //
        using var ps = new ProgressScope(range, null, 1);
        //
        for (;;)
        {
            if (UserBreak(ps))
            {
                return;
            }
            bFound = false;
            //
            // 1. MEF
            mve.Clear();
            it.Initialize(_shapes);
            for (; it.More(); it.Next())
            {
                Shape e = it.Value();
                if (!_shapesToAvoid.Contains(e))
                {
                    Exploration.MapShapesAndAncestors(e, ShapeType.Vertex, ShapeType.Edge, mve);
                }
            }
            nbV = mve.Extent();
            //
            // 2. myEdgesToAvoid
            for (i = 1; i <= nbV; ++i)
            {
                Vertex v = ShapeCasts.Vertex(mve.FindKey(i));
                //
                NCollection_List<Shape> le = mve.ChangeFromKey(v);
                nbE = le.Extent();
                if (nbE == 0)
                {
                    continue;
                }
                //
                Edge e1 = ShapeCasts.Edge(le.First());
                if (nbE == 1)
                {
                    if (Tool.Degenerated(e1))
                    {
                        continue;
                    }
                    if (v.Orientation() == Internal)
                    {
                        continue;
                    }
                    bFound = true;
                    _shapesToAvoid.Add(e1);
                }
                else if (nbE == 2)
                {
                    Edge e2 = ShapeCasts.Edge(le.Last());
                    if (e2.IsSame(e1))
                    {
                        Exploration.Vertices(e1, out Vertex v1x, out Vertex v2x);
                        if (v1x.IsSame(v2x))
                        {
                            continue;
                        }
                        bFound = true;
                        _shapesToAvoid.Add(e1);
                        _shapesToAvoid.Add(e2);
                    }
                }
            } // for (i=1; i<=aNbE; ++i) {
            //
            if (!bFound)
            {
                break;
            }
        }
    }

    /// <summary>Makes the loops with the wire splitter, and the internal wires of the edges left over.</summary>
    protected override void PerformLoops(ProgressRange range)
    {
        bool bFlag;
        int i, nbEA;
        NCollection_List<Shape>.Iterator it = new(_shapes);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> veMap = new(ShapeHasher.Instance);
        // OCCT's NCollection_Map<TopoDS_Shape> with the default hasher (orientation included, decision 721)
        NCollection_Map<Shape> mAdded = new(EqualityComparer<Shape>.Default);
        Iterator itW = new();
        BrepBuilder bb = new();
        WireEdgeSet wes = new();
        WireSplitter wSp = new();
        //
        using var mainScope = new ProgressScope(range, "Making wires", 10);
        //
        // 1.
        _loops.Clear();
        wes.SetFace(_face);
        //
        it.Initialize(_shapes);
        for (; it.More(); it.Next())
        {
            Shape e = it.Value();
            if (!_shapesToAvoid.Contains(e))
            {
                wes.AddStartElement(e);
            }
        }
        //
        wSp.SetWES(wes);
        wSp.SetRunParallel(_runParallel);
        wSp.SetContext(_context);
        wSp.Perform(mainScope.Next(9));
        if (wSp.HasErrors)
        {
            return;
        }
        //
        NCollection_List<Shape> lw = wes.Shapes();
        it.Initialize(lw);
        for (; it.More(); it.Next())
        {
            Shape w = it.Value();
            _loops.Append(w);
        }
        // Post Treatment
        NCollection_Map<Shape> mep = new(EqualityComparer<Shape>.Default);
        //
        // a. collect all edges that are in loops
        it.Initialize(_loops);
        for (; it.More(); it.Next())
        {
            Shape w = it.Value();
            itW.Initialize(w);
            for (; itW.More(); itW.Next())
            {
                Shape e = itW.Value();
                mep.Add(e);
            }
        }
        if (UserBreak(mainScope))
        {
            return;
        }
        //
        // b. collect all edges that are to avoid
        nbEA = _shapesToAvoid.Extent();
        for (i = 1; i <= nbEA; ++i)
        {
            Shape e = _shapesToAvoid[i];
            mep.Add(e);
        }
        //
        // c. add all edges that are not processed to myShapesToAvoid
        it.Initialize(_shapes);
        for (; it.More(); it.Next())
        {
            Shape e = it.Value();
            if (!mep.Contains(e))
            {
                _shapesToAvoid.Add(e);
            }
        }
        //
        if (UserBreak(mainScope))
        {
            return;
        }
        // 2. Internal Wires
        _loopsInternal.Clear();
        //
        nbEA = _shapesToAvoid.Extent();
        for (i = 1; i <= nbEA; ++i)
        {
            Shape ee = _shapesToAvoid[i];
            Exploration.MapShapesAndAncestors(ee, ShapeType.Vertex, ShapeType.Edge, veMap);
        }
        //
        bFlag = true;
        for (i = 1; (i <= nbEA) && bFlag; ++i)
        {
            Shape ee = _shapesToAvoid[i];
            if (!mAdded.Add(ee))
            {
                continue;
            }
            //
            if (UserBreak(mainScope))
            {
                return;
            }
            // make new wire
            Wire w = new();
            bb.MakeWire(ref w);
            bb.Add(w, ee);
            //
            itW.Initialize(w);
            for (; itW.More() && bFlag; itW.Next())
            {
                Edge e = ShapeCasts.Edge(itW.Value());
                //
                Iterator itE = new(e);
                for (; itE.More() && bFlag; itE.Next())
                {
                    Vertex v = ShapeCasts.Vertex(itE.Value());
                    NCollection_List<Shape> le = veMap.FindFromKey(v);
                    it.Initialize(le);
                    for (; it.More() && bFlag; it.Next())
                    {
                        Shape ex = it.Value();
                        if (mAdded.Add(ex))
                        {
                            bb.Add(w, ex);
                            if (mAdded.Extent() == nbEA)
                            {
                                bFlag = !bFlag;
                            }
                        }
                    } // for (; aIt.More(); aIt.Next()) {
                } // for (; aItE.More(); aItE.Next()) {
            } // for (; aItW.More(); aItW.Next()) {
            w.Closed(Tool.IsClosed(w));
            _loopsInternal.Append(w);
        } // for (i = 1; (i <= aNbEA) && bFlag; ++i) {
    }

    /// <summary>Makes the faces of the loops: growths, with the holes inside them.</summary>
    protected override void PerformAreas(ProgressRange range)
    {
        _areas.Clear();
        BrepBuilder bb = new();
        // Location of the myFace
        // Get surface from myFace
        Surface? s = Tool.Surface(_face, out Placement loc);
        // Get tolerance of myFace
        double tol = Tool.Tolerance(_face);

        using var mainScope = new ProgressScope(range, null, 10);

        // Check if there are no loops at all
        if (_loops.IsEmpty())
        {
            if (_context!.IsInfiniteFace(_face))
            {
                Face face0 = new();
                bb.MakeFace(ref face0, s, loc, tol);
                if (Tool.NaturalRestriction(_face))
                {
                    bb.NaturalRestriction(face0, true);
                }
                _areas.Append(face0);
            }
            return;
        }

        // The new faces
        NCollection_List<Shape> newFaces = new();
        // The hole faces which has to be classified relatively new faces
        NCollection_IndexedMap<Shape> holeFaces = new(ShapeHasher.Instance);
        // Map of the edges of the hole faces for quick check of the growths.
        // If the analyzed wire contains any of the edges from the hole faces
        // it is considered as growth.
        NCollection_IndexedMap<Shape> mhe = new(ShapeHasher.Instance);

        // Analyze the new wires - classify them to be the holes and growths
        using (var psClass = new ProgressScope(mainScope.Next(5), "Making faces", _loops.Extent()))
        {
            NCollection_List<Shape>.Iterator itLL = new(_loops);
            for (; itLL.More(); itLL.Next(), psClass.Next())
            {
                if (UserBreak(psClass))
                {
                    return;
                }

                Shape wire = itLL.Value();

                Face face = new();
                bb.MakeFace(ref face, s, loc, tol);
                bb.Add(face, wire);

                bool bIsGrowth = IsGrowthWire(wire, mhe);
                if (!bIsGrowth)
                {
                    // Fast check did not give the result, run classification
                    FaceClassifier2D clsf = _context!.FClass2d(face);
                    bIsGrowth = !clsf.IsHole;
                }

                // Save the face
                if (bIsGrowth)
                {
                    newFaces.Append(face);
                }
                else
                {
                    holeFaces.Add(face);
                    Exploration.MapShapes(wire, ShapeType.Edge, mhe);
                }
            }
        }

        if (holeFaces.IsEmpty())
        {
            // No holes, stop the analysis
            _areas.Append(newFaces);
            return;
        }

        // Classify holes relatively faces

        // Prepare tree with the boxes of the hole faces
        Box2dTree boxTree = new();
        int i, nbH = holeFaces.Extent();
        boxTree.SetSize(nbH);
        for (i = 1; i <= nbH; ++i)
        {
            Face hFace = ShapeCasts.Face(holeFaces[i]);
            //
            Box2d box = new();
            BrepTools.AddUVBounds(hFace, ref box);
            boxTree.Add(i, global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(box));
        }

        // Build BVH
        boxTree.Build();

        // Find outer growth face that is most close to each hole face
        NCollection_IndexedDataMap<Shape, Shape> holeFaceMap = new(ShapeHasher.Instance);

        // Selector
        Box2dTreeSelector selector = new();
        selector.SetBVHSet(boxTree);

        NCollection_List<Shape>.Iterator itLS = new(newFaces);
        using (var psHoles = new ProgressScope(mainScope.Next(4), "Adding holes", newFaces.Extent()))
        {
            for (; itLS.More(); itLS.Next(), psHoles.Next())
            {
                if (UserBreak(psHoles))
                {
                    return;
                }
                Face face = ShapeCasts.Face(itLS.Value());

                // Build box
                Box2d box = new();
                BrepTools.AddUVBounds(face, ref box);

                selector.Clear();
                selector.SetBox(global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(box));
                selector.Select();

                NCollection_List<int> li = selector.Indices();
                NCollection_List<int>.Iterator itLI = new(li);
                for (; itLI.More(); itLI.Next())
                {
                    int k = itLI.Value();
                    Shape hole = holeFaces[k];
                    // Check if it is inside
                    if (!IsInside(hole, face, _context!))
                    {
                        continue;
                    }

                    // Save the relation
                    int indexWas = holeFaceMap.FindIndex(hole);
                    if (indexWas > 0)
                    {
                        if (IsInside(face, holeFaceMap[indexWas], _context!))
                        {
                            holeFaceMap[indexWas] = face;
                        }
                    }
                    else
                    {
                        holeFaceMap.Add(hole, face);
                    }
                }
            }
        }

        // Make the back map from faces to holes
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> faceHolesMap = new(ShapeHasher.Instance);

        nbH = holeFaceMap.Extent();
        for (i = 1; i <= nbH; ++i)
        {
            Shape hole = holeFaceMap.FindKey(i);
            Shape face = holeFaceMap[i];
            //
            int index = faceHolesMap.FindIndex(face);
            NCollection_List<Shape> pLHoles = index > 0
                                                     ? faceHolesMap[index]
                                                     : faceHolesMap[faceHolesMap.Add(face, new NCollection_List<Shape>())];
            pLHoles.Append(hole);
        }

        // Add unused holes to the original face
        if (holeFaces.Extent() != holeFaceMap.Extent())
        {
            Box boxF = new();
            Bounds.Add(_face, ref boxF);
            if (boxF.IsOpenXmin || boxF.IsOpenXmax || boxF.IsOpenYmin || boxF.IsOpenYmax
                || boxF.IsOpenZmin || boxF.IsOpenZmax)
            {
                Face face = new();
                bb.MakeFace(ref face, s, loc, tol);
                NCollection_List<Shape> unUsedHoles =
                  faceHolesMap[faceHolesMap.Add(face, new NCollection_List<Shape>())];
                nbH = holeFaces.Extent();
                for (i = 1; i <= nbH; ++i)
                {
                    Shape hole = holeFaces[i];
                    if (!holeFaceMap.Contains(hole))
                    {
                        unUsedHoles.Append(hole);
                    }
                }
                // Save it
                newFaces.Append(face);
            }
        }

        // Add Holes to Faces and add them to myAreas
        using var psu = new ProgressScope(mainScope.Next(), null, newFaces.Extent());
        itLS.Initialize(newFaces);
        for (; itLS.More(); itLS.Next(), psu.Next())
        {
            if (UserBreak(psu))
            {
                return;
            }

            Face face = ShapeCasts.Face(itLS.Value());
            if (faceHolesMap.FindFromKey(face, out NCollection_List<Shape> pLHoles))
            {
                // update faces with the holes
                NCollection_List<Shape>.Iterator itLH = new(pLHoles);
                for (; itLH.More(); itLH.Next())
                {
                    Shape fHole = itLH.Value();
                    // The hole face contains only one wire
                    Iterator itW = new(fHole);
                    bb.Add(face, itW.Value());
                }

                // update classifier
                _context!.FClass2d(face).Init(face, tol);
            }

            // The face is just a draft that does not contain any internal shapes
            _areas.Append(face);
        }
    }

    /// <summary>Adds the internal wires into the faces that contain them; warns about the edges left over.</summary>
    protected override void PerformInternalShapes(ProgressRange range)
    {
        if (_avoidInternalShapes)
        {
            // User-defined option to avoid internal edges
            // in the result is in force.
            return;
        }

        if (_loopsInternal.IsEmpty())
        {
            // No edges left for classification
            return;
        }

        // Prepare tree with the boxes of the edges to classify
        Box2dTree boxTree = new();

        // Map of edges to classify
        NCollection_IndexedMap<Shape> edgesMap = new(ShapeHasher.Instance);

        // Main progress scope
        using var mainScope = new ProgressScope(range, "Adding internal shapes", 3);

        // Fill the tree and the map
        NCollection_List<Shape>.Iterator itLE = new(_loopsInternal);
        for (; itLE.More(); itLE.Next())
        {
            if (UserBreak(mainScope))
            {
                return;
            }
            Iterator itE = new(itLE.Value());
            for (; itE.More(); itE.Next())
            {
                Edge e = ShapeCasts.Edge(itE.Value());
                if (!edgesMap.Contains(e))
                {
                    Box2d boxE = new();
                    BrepTools.AddUVBounds(_face, e, ref boxE);
                    // Make sure the index of edge in the map and
                    // of the box in the tree is the same
                    boxTree.Add(edgesMap.Add(e), global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(boxE));
                }
            }
        }

        // Build BVH
        boxTree.Build();

        mainScope.Next();

        // Fence map
        NCollection_IntMap meDone = new();

        // Classify edges relatively faces
        using (var psClass = new ProgressScope(mainScope.Next(), null, _areas.Extent()))
        {
            NCollection_List<Shape>.Iterator itLF = new(_areas);
            for (; itLF.More(); itLF.Next(), psClass.Next())
            {
                if (UserBreak(psClass))
                {
                    return;
                }
                Face f = ShapeCasts.Face(itLF.Value());

                // Build box
                Box2d boxF = new();
                BrepTools.AddUVBounds(f, ref boxF);

                // Select edges for the classification
                Box2dTreeSelector selector = new();
                selector.SetBVHSet(boxTree);
                selector.SetBox(global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(boxF));
                if (selector.Select() == 0)
                {
                    continue;
                }

                // Collect edges inside the face
                NCollection_IndexedMap<Shape> edgesInside = new(ShapeHasher.Instance);

                NCollection_List<int> li = selector.Indices();
                NCollection_List<int>.Iterator itLI = new(li);
                for (; itLI.More(); itLI.Next())
                {
                    int nE = itLI.Value();
                    if (meDone.Contains(nE))
                    {
                        continue;
                    }

                    Edge e = ShapeCasts.Edge(edgesMap[nE]);
                    if (IsInside(e, f, _context!))
                    {
                        edgesInside.Add(e);
                        meDone.Add(nE);
                    }
                }

                if (edgesInside.IsEmpty())
                {
                    continue;
                }

                // Make internal wires
                NCollection_List<Shape> lsi0 = new();
                MakeInternalWires(edgesInside, lsi0);

                // Add wires to a face
                NCollection_List<Shape>.Iterator itLSI = new(lsi0);
                for (; itLSI.More(); itLSI.Next())
                {
                    Shape wi = itLSI.Value();
                    new BrepBuilder().Add(f, wi);
                }

                // Condition of early exit
                if (meDone.Extent() == edgesMap.Extent())
                {
                    // All edges are classified and added into the faces
                    return;
                }
            }
        }

        // Some edges are left unclassified - warn user about them
        NCollection_IndexedMap<Shape> edgesUnUsed = new(ShapeHasher.Instance);
        for (int i = 1; i <= edgesMap.Extent(); ++i)
        {
            if (!meDone.Contains(i))
            {
                edgesUnUsed.Add(edgesMap[i]);
            }
        }

        // Make internal wires
        NCollection_List<Shape> lsi = new();
        MakeInternalWires(edgesUnUsed, lsi);

        // Make compound
        Compound wShape = new();
        new BrepBuilder().MakeCompound(ref wShape);
        new BrepBuilder().Add(wShape, _face);
        if (lsi.Extent() == 1)
        {
            new BrepBuilder().Add(wShape, lsi.First());
        }
        else
        {
            Compound ce = new();
            new BrepBuilder().MakeCompound(ref ce);
            for (NCollection_List<Shape>.Iterator it = new(lsi); it.More(); it.Next())
            {
                new BrepBuilder().Add(ce, it.Value());
            }
            new BrepBuilder().Add(wShape, ce);
        }

        // Add warning
        AddWarning(new AlertFaceBuilderUnusedEdges(wShape));
    }

    private static void MakeInternalWires(NCollection_IndexedMap<Shape> me, NCollection_List<Shape> wires)
    {
        int i, nbE;
        NCollection_Map<Shape> addedMap = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itE = new(wires);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> mve = new(ShapeHasher.Instance);
        BrepBuilder bb = new();
        //
        nbE = me.Extent();
        for (i = 1; i <= nbE; ++i)
        {
            Shape e = me[i];
            Exploration.MapShapesAndAncestors(e, ShapeType.Vertex, ShapeType.Edge, mve);
        }
        //
        for (i = 1; i <= nbE; ++i)
        {
            Shape ee = me[i];
            if (!addedMap.Add(ee))
            {
                continue;
            }
            //
            // make a new shell
            Wire w = new();
            bb.MakeWire(ref w);
            ee.Orientation(Internal);
            bb.Add(w, ee);
            //
            Iterator itAdded = new(w);
            for (; itAdded.More(); itAdded.Next())
            {
                Shape e = itAdded.Value();
                //
                Explorer exp = new(e, ShapeType.Vertex);
                for (; exp.More(); exp.Next())
                {
                    Shape v = exp.Current();
                    NCollection_List<Shape> le = mve.FindFromKey(v);
                    itE.Initialize(le);
                    for (; itE.More(); itE.Next())
                    {
                        Shape el = itE.Value();
                        if (addedMap.Add(el))
                        {
                            el.Orientation(Internal);
                            bb.Add(w, el);
                        }
                    }
                }
            }
            w.Closed(Tool.IsClosed(w));
            wires.Append(w);
        }
    }

    // Checks if the wire is located inside the face: takes a point of the wire and classifies it relatively the face.
    private static bool IsInside(in Shape wire, in Shape f, IntersectionContext context)
    {
        // Avoid edges of the face
        NCollection_IndexedMap<Shape> faceEdgesMap = new(ShapeHasher.Instance);
        Exploration.MapShapes(f, ShapeType.Edge, faceEdgesMap);

        // Get classification tool from the context
        Face aF = ShapeCasts.Face(f);
        FaceClassifier2D classifier = context.FClass2d(aF);

        bool isInside = false;

        // Iterate on wire edges until first classification is performed
        Explorer exp = new(wire, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            Edge e = ShapeCasts.Edge(exp.Current());
            if (Tool.Degenerated(e))
            {
                // Avoid checking degenerated edges.
                continue;
            }

            if (faceEdgesMap.Contains(e))
            {
                // Face contains the edge from the wire, thus the wire cannot be
                // inside that face.
                return isInside;
            }

            // Get 2d curve of the edge on the face
            Curve2D? c2D = Tool.CurveOnSurface(e, aF, out double t1, out double t2);
            if (c2D is null)
            {
                continue;
            }

            // Get middle point on the curve
            Point2D p2D = c2D.Value((t1 + t2) / 2.0);

            // Classify the point
            Containment state = classifier.Perform(p2D);
            isInside = state == In;
            break;
        }
        return isInside;
    }

    private static bool IsGrowthWire(in Shape wire, NCollection_IndexedMap<Shape> mhe)
    {
        if (mhe.Extent() != 0)
        {
            Iterator it = new(wire);
            for (; it.More(); it.Next())
            {
                if (mhe.Contains(it.Value()))
                {
                    return true;
                }
            }
        }
        return false;
    }
}
