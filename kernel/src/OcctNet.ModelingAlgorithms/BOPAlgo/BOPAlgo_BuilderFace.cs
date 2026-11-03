// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderFace.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_BuilderFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Builds the faces bounded by a set of edges on the surface of a face (the split parts of the face): the loops are
/// made by BOPAlgo_WireSplitter, classified as growths and holes, the holes put into the growths that contain them, and
/// the edges left over added as internal wires.
/// </summary>
/// <remarks>
/// The new faces are built on the face's surface, location and tolerance. The given face is kept forward; its own
/// orientation is <see cref="Orientation"/>.
/// </remarks>
public class BOPAlgo_BuilderFace : BOPAlgo_BuilderArea
{
    protected TopoDS_Face myFace = new();
    protected TopAbs_Orientation myOrientation;

    public BOPAlgo_BuilderFace()
    {
        myOrientation = TopAbs_EXTERNAL;
    }

    /// <summary>Sets the face generatrix (kept forward; its orientation is <see cref="Orientation"/>).</summary>
    public void SetFace(in TopoDS_Face theFace)
    {
        myOrientation = theFace.Orientation();
        myFace = theFace;
        myFace.Orientation(TopAbs_FORWARD);
    }

    /// <summary>The face generatrix.</summary>
    public TopoDS_Face Face() => myFace;

    /// <summary>The orientation of the face generatrix.</summary>
    public TopAbs_Orientation Orientation() => myOrientation;

    protected override void CheckData()
    {
        if (myFace.IsNull())
        {
            AddError(new BOPAlgo_AlertNullInputShapes());
            return;
        }
        if (myContext is null)
        {
            myContext = new IntTools_Context();
        }
    }

    /// <summary>Performs the algorithm.</summary>
    public override void Perform(ProgressRange theRange = default)
    {
        using var aPS = new ProgressScope(theRange, null, 100);

        GetReport().Clear();
        //
        CheckData();
        if (HasErrors())
        {
            return;
        }
        //
        PerformShapesToAvoid(aPS.Next(1));
        if (HasErrors())
        {
            return;
        }
        //
        PerformLoops(aPS.Next(10));
        if (HasErrors())
        {
            return;
        }
        //
        PerformAreas(aPS.Next(80));
        if (HasErrors())
        {
            return;
        }
        //
        PerformInternalShapes(aPS.Next(9));
    }

    /// <summary>Collects the edges that can't be in a loop: free edges and edges given twice.</summary>
    protected override void PerformShapesToAvoid(ProgressRange theRange)
    {
        bool bFound;
        int i, aNbV, aNbE;
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMVE = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aIt = new(myShapes);
        //
        myShapesToAvoid.Clear();
        //
        using var aPS = new ProgressScope(theRange, null, 1);
        //
        for (;;)
        {
            if (UserBreak(aPS))
            {
                return;
            }
            bFound = false;
            //
            // 1. MEF
            aMVE.Clear();
            aIt.Initialize(myShapes);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape aE = aIt.Value();
                if (!myShapesToAvoid.Contains(aE))
                {
                    TopExp.MapShapesAndAncestors(aE, TopAbs_VERTEX, TopAbs_EDGE, aMVE);
                }
            }
            aNbV = aMVE.Extent();
            //
            // 2. myEdgesToAvoid
            for (i = 1; i <= aNbV; ++i)
            {
                TopoDS_Vertex aV = TopoDS.Vertex(aMVE.FindKey(i));
                //
                NCollection_List<TopoDS_Shape> aLE = aMVE.ChangeFromKey(aV);
                aNbE = aLE.Extent();
                if (aNbE == 0)
                {
                    continue;
                }
                //
                TopoDS_Edge aE1 = TopoDS.Edge(aLE.First());
                if (aNbE == 1)
                {
                    if (BRep_Tool.Degenerated(aE1))
                    {
                        continue;
                    }
                    if (aV.Orientation() == TopAbs_INTERNAL)
                    {
                        continue;
                    }
                    bFound = true;
                    myShapesToAvoid.Add(aE1);
                }
                else if (aNbE == 2)
                {
                    TopoDS_Edge aE2 = TopoDS.Edge(aLE.Last());
                    if (aE2.IsSame(aE1))
                    {
                        TopExp.Vertices(aE1, out TopoDS_Vertex aV1x, out TopoDS_Vertex aV2x);
                        if (aV1x.IsSame(aV2x))
                        {
                            continue;
                        }
                        bFound = true;
                        myShapesToAvoid.Add(aE1);
                        myShapesToAvoid.Add(aE2);
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
    protected override void PerformLoops(ProgressRange theRange)
    {
        bool bFlag;
        int i, aNbEA;
        NCollection_List<TopoDS_Shape>.Iterator aIt = new(myShapes);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aVEMap = new(TopTools_ShapeMapHasher.Instance);
        // OCCT's NCollection_Map<TopoDS_Shape> with the default hasher (orientation included, decision 721)
        NCollection_Map<TopoDS_Shape> aMAdded = new(EqualityComparer<TopoDS_Shape>.Default);
        TopoDS_Iterator aItW = new();
        BRep_Builder aBB = new();
        BOPAlgo_WireEdgeSet aWES = new();
        BOPAlgo_WireSplitter aWSp = new();
        //
        using var aMainScope = new ProgressScope(theRange, "Making wires", 10);
        //
        // 1.
        myLoops.Clear();
        aWES.SetFace(myFace);
        //
        aIt.Initialize(myShapes);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aE = aIt.Value();
            if (!myShapesToAvoid.Contains(aE))
            {
                aWES.AddStartElement(aE);
            }
        }
        //
        aWSp.SetWES(aWES);
        aWSp.SetRunParallel(myRunParallel);
        aWSp.SetContext(myContext);
        aWSp.Perform(aMainScope.Next(9));
        if (aWSp.HasErrors())
        {
            return;
        }
        //
        NCollection_List<TopoDS_Shape> aLW = aWES.Shapes();
        aIt.Initialize(aLW);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aW = aIt.Value();
            myLoops.Append(aW);
        }
        // Post Treatment
        NCollection_Map<TopoDS_Shape> aMEP = new(EqualityComparer<TopoDS_Shape>.Default);
        //
        // a. collect all edges that are in loops
        aIt.Initialize(myLoops);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aW = aIt.Value();
            aItW.Initialize(aW);
            for (; aItW.More(); aItW.Next())
            {
                TopoDS_Shape aE = aItW.Value();
                aMEP.Add(aE);
            }
        }
        if (UserBreak(aMainScope))
        {
            return;
        }
        //
        // b. collect all edges that are to avoid
        aNbEA = myShapesToAvoid.Extent();
        for (i = 1; i <= aNbEA; ++i)
        {
            TopoDS_Shape aE = myShapesToAvoid[i];
            aMEP.Add(aE);
        }
        //
        // c. add all edges that are not processed to myShapesToAvoid
        aIt.Initialize(myShapes);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aE = aIt.Value();
            if (!aMEP.Contains(aE))
            {
                myShapesToAvoid.Add(aE);
            }
        }
        //
        if (UserBreak(aMainScope))
        {
            return;
        }
        // 2. Internal Wires
        myLoopsInternal.Clear();
        //
        aNbEA = myShapesToAvoid.Extent();
        for (i = 1; i <= aNbEA; ++i)
        {
            TopoDS_Shape aEE = myShapesToAvoid[i];
            TopExp.MapShapesAndAncestors(aEE, TopAbs_VERTEX, TopAbs_EDGE, aVEMap);
        }
        //
        bFlag = true;
        for (i = 1; (i <= aNbEA) && bFlag; ++i)
        {
            TopoDS_Shape aEE = myShapesToAvoid[i];
            if (!aMAdded.Add(aEE))
            {
                continue;
            }
            //
            if (UserBreak(aMainScope))
            {
                return;
            }
            // make new wire
            TopoDS_Wire aW = new();
            aBB.MakeWire(ref aW);
            aBB.Add(aW, aEE);
            //
            aItW.Initialize(aW);
            for (; aItW.More() && bFlag; aItW.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(aItW.Value());
                //
                TopoDS_Iterator aItE = new(aE);
                for (; aItE.More() && bFlag; aItE.Next())
                {
                    TopoDS_Vertex aV = TopoDS.Vertex(aItE.Value());
                    NCollection_List<TopoDS_Shape> aLE = aVEMap.FindFromKey(aV);
                    aIt.Initialize(aLE);
                    for (; aIt.More() && bFlag; aIt.Next())
                    {
                        TopoDS_Shape aEx = aIt.Value();
                        if (aMAdded.Add(aEx))
                        {
                            aBB.Add(aW, aEx);
                            if (aMAdded.Extent() == aNbEA)
                            {
                                bFlag = !bFlag;
                            }
                        }
                    } // for (; aIt.More(); aIt.Next()) {
                } // for (; aItE.More(); aItE.Next()) {
            } // for (; aItW.More(); aItW.Next()) {
            aW.Closed(BRep_Tool.IsClosed(aW));
            myLoopsInternal.Append(aW);
        } // for (i = 1; (i <= aNbEA) && bFlag; ++i) {
    }

    /// <summary>Makes the faces of the loops: growths, with the holes inside them.</summary>
    protected override void PerformAreas(ProgressRange theRange)
    {
        myAreas.Clear();
        BRep_Builder aBB = new();
        // Location of the myFace
        // Get surface from myFace
        Geom_Surface? aS = BRep_Tool.Surface(myFace, out Placement aLoc);
        // Get tolerance of myFace
        double aTol = BRep_Tool.Tolerance(myFace);

        using var aMainScope = new ProgressScope(theRange, null, 10);

        // Check if there are no loops at all
        if (myLoops.IsEmpty())
        {
            if (myContext!.IsInfiniteFace(myFace))
            {
                TopoDS_Face aFace0 = new();
                aBB.MakeFace(ref aFace0, aS, aLoc, aTol);
                if (BRep_Tool.NaturalRestriction(myFace))
                {
                    aBB.NaturalRestriction(aFace0, true);
                }
                myAreas.Append(aFace0);
            }
            return;
        }

        // The new faces
        NCollection_List<TopoDS_Shape> aNewFaces = new();
        // The hole faces which has to be classified relatively new faces
        NCollection_IndexedMap<TopoDS_Shape> aHoleFaces = new(TopTools_ShapeMapHasher.Instance);
        // Map of the edges of the hole faces for quick check of the growths.
        // If the analyzed wire contains any of the edges from the hole faces
        // it is considered as growth.
        NCollection_IndexedMap<TopoDS_Shape> aMHE = new(TopTools_ShapeMapHasher.Instance);

        // Analyze the new wires - classify them to be the holes and growths
        using (var aPSClass = new ProgressScope(aMainScope.Next(5), "Making faces", myLoops.Extent()))
        {
            NCollection_List<TopoDS_Shape>.Iterator aItLL = new(myLoops);
            for (; aItLL.More(); aItLL.Next(), aPSClass.Next())
            {
                if (UserBreak(aPSClass))
                {
                    return;
                }

                TopoDS_Shape aWire = aItLL.Value();

                TopoDS_Face aFace = new();
                aBB.MakeFace(ref aFace, aS, aLoc, aTol);
                aBB.Add(aFace, aWire);

                bool bIsGrowth = IsGrowthWire(aWire, aMHE);
                if (!bIsGrowth)
                {
                    // Fast check did not give the result, run classification
                    IntTools_FClass2d aClsf = myContext!.FClass2d(aFace);
                    bIsGrowth = !aClsf.IsHole();
                }

                // Save the face
                if (bIsGrowth)
                {
                    aNewFaces.Append(aFace);
                }
                else
                {
                    aHoleFaces.Add(aFace);
                    TopExp.MapShapes(aWire, TopAbs_EDGE, aMHE);
                }
            }
        }

        if (aHoleFaces.IsEmpty())
        {
            // No holes, stop the analysis
            myAreas.Append(aNewFaces);
            return;
        }

        // Classify holes relatively faces

        // Prepare tree with the boxes of the hole faces
        BOPTools_Box2dTree aBoxTree = new();
        int i, aNbH = aHoleFaces.Extent();
        aBoxTree.SetSize(aNbH);
        for (i = 1; i <= aNbH; ++i)
        {
            TopoDS_Face aHFace = TopoDS.Face(aHoleFaces[i]);
            //
            Box2d aBox = new();
            BRepTools.AddUVBounds(aHFace, ref aBox);
            aBoxTree.Add(i, Tools.Bnd2BVH(aBox));
        }

        // Build BVH
        aBoxTree.Build();

        // Find outer growth face that is most close to each hole face
        NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> aHoleFaceMap = new(TopTools_ShapeMapHasher.Instance);

        // Selector
        BOPTools_Box2dTreeSelector aSelector = new();
        aSelector.SetBVHSet(aBoxTree);

        NCollection_List<TopoDS_Shape>.Iterator aItLS = new(aNewFaces);
        using (var aPSHoles = new ProgressScope(aMainScope.Next(4), "Adding holes", aNewFaces.Extent()))
        {
            for (; aItLS.More(); aItLS.Next(), aPSHoles.Next())
            {
                if (UserBreak(aPSHoles))
                {
                    return;
                }
                TopoDS_Face aFace = TopoDS.Face(aItLS.Value());

                // Build box
                Box2d aBox = new();
                BRepTools.AddUVBounds(aFace, ref aBox);

                aSelector.Clear();
                aSelector.SetBox(Tools.Bnd2BVH(aBox));
                aSelector.Select();

                NCollection_List<int> aLI = aSelector.Indices();
                NCollection_List<int>.Iterator aItLI = new(aLI);
                for (; aItLI.More(); aItLI.Next())
                {
                    int k = aItLI.Value();
                    TopoDS_Shape aHole = aHoleFaces[k];
                    // Check if it is inside
                    if (!IsInside(aHole, aFace, myContext!))
                    {
                        continue;
                    }

                    // Save the relation
                    int anIndexWas = aHoleFaceMap.FindIndex(aHole);
                    if (anIndexWas > 0)
                    {
                        if (IsInside(aFace, aHoleFaceMap[anIndexWas], myContext!))
                        {
                            aHoleFaceMap[anIndexWas] = aFace;
                        }
                    }
                    else
                    {
                        aHoleFaceMap.Add(aHole, aFace);
                    }
                }
            }
        }

        // Make the back map from faces to holes
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aFaceHolesMap = new(TopTools_ShapeMapHasher.Instance);

        aNbH = aHoleFaceMap.Extent();
        for (i = 1; i <= aNbH; ++i)
        {
            TopoDS_Shape aHole = aHoleFaceMap.FindKey(i);
            TopoDS_Shape aFace = aHoleFaceMap[i];
            //
            int anIndex = aFaceHolesMap.FindIndex(aFace);
            NCollection_List<TopoDS_Shape> pLHoles = anIndex > 0
                                                     ? aFaceHolesMap[anIndex]
                                                     : aFaceHolesMap[aFaceHolesMap.Add(aFace, new NCollection_List<TopoDS_Shape>())];
            pLHoles.Append(aHole);
        }

        // Add unused holes to the original face
        if (aHoleFaces.Extent() != aHoleFaceMap.Extent())
        {
            Box aBoxF = new();
            BRepBndLib.Add(myFace, ref aBoxF);
            if (aBoxF.IsOpenXmin || aBoxF.IsOpenXmax || aBoxF.IsOpenYmin || aBoxF.IsOpenYmax
                || aBoxF.IsOpenZmin || aBoxF.IsOpenZmax)
            {
                TopoDS_Face aFace = new();
                aBB.MakeFace(ref aFace, aS, aLoc, aTol);
                NCollection_List<TopoDS_Shape> anUnUsedHoles =
                  aFaceHolesMap[aFaceHolesMap.Add(aFace, new NCollection_List<TopoDS_Shape>())];
                aNbH = aHoleFaces.Extent();
                for (i = 1; i <= aNbH; ++i)
                {
                    TopoDS_Shape aHole = aHoleFaces[i];
                    if (!aHoleFaceMap.Contains(aHole))
                    {
                        anUnUsedHoles.Append(aHole);
                    }
                }
                // Save it
                aNewFaces.Append(aFace);
            }
        }

        // Add Holes to Faces and add them to myAreas
        using var aPSU = new ProgressScope(aMainScope.Next(), null, aNewFaces.Extent());
        aItLS.Initialize(aNewFaces);
        for (; aItLS.More(); aItLS.Next(), aPSU.Next())
        {
            if (UserBreak(aPSU))
            {
                return;
            }

            TopoDS_Face aFace = TopoDS.Face(aItLS.Value());
            if (aFaceHolesMap.FindFromKey(aFace, out NCollection_List<TopoDS_Shape> pLHoles))
            {
                // update faces with the holes
                NCollection_List<TopoDS_Shape>.Iterator aItLH = new(pLHoles);
                for (; aItLH.More(); aItLH.Next())
                {
                    TopoDS_Shape aFHole = aItLH.Value();
                    // The hole face contains only one wire
                    TopoDS_Iterator aItW = new(aFHole);
                    aBB.Add(aFace, aItW.Value());
                }

                // update classifier
                myContext!.FClass2d(aFace).Init(aFace, aTol);
            }

            // The face is just a draft that does not contain any internal shapes
            myAreas.Append(aFace);
        }
    }

    /// <summary>Adds the internal wires into the faces that contain them; warns about the edges left over.</summary>
    protected override void PerformInternalShapes(ProgressRange theRange)
    {
        if (myAvoidInternalShapes)
        {
            // User-defined option to avoid internal edges
            // in the result is in force.
            return;
        }

        if (myLoopsInternal.IsEmpty())
        {
            // No edges left for classification
            return;
        }

        // Prepare tree with the boxes of the edges to classify
        BOPTools_Box2dTree aBoxTree = new();

        // Map of edges to classify
        NCollection_IndexedMap<TopoDS_Shape> anEdgesMap = new(TopTools_ShapeMapHasher.Instance);

        // Main progress scope
        using var aMainScope = new ProgressScope(theRange, "Adding internal shapes", 3);

        // Fill the tree and the map
        NCollection_List<TopoDS_Shape>.Iterator itLE = new(myLoopsInternal);
        for (; itLE.More(); itLE.Next())
        {
            if (UserBreak(aMainScope))
            {
                return;
            }
            TopoDS_Iterator itE = new(itLE.Value());
            for (; itE.More(); itE.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(itE.Value());
                if (!anEdgesMap.Contains(aE))
                {
                    Box2d aBoxE = new();
                    BRepTools.AddUVBounds(myFace, aE, ref aBoxE);
                    // Make sure the index of edge in the map and
                    // of the box in the tree is the same
                    aBoxTree.Add(anEdgesMap.Add(aE), Tools.Bnd2BVH(aBoxE));
                }
            }
        }

        // Build BVH
        aBoxTree.Build();

        aMainScope.Next();

        // Fence map
        NCollection_IntMap aMEDone = new();

        // Classify edges relatively faces
        using (var aPSClass = new ProgressScope(aMainScope.Next(), null, myAreas.Extent()))
        {
            NCollection_List<TopoDS_Shape>.Iterator itLF = new(myAreas);
            for (; itLF.More(); itLF.Next(), aPSClass.Next())
            {
                if (UserBreak(aPSClass))
                {
                    return;
                }
                TopoDS_Face aF = TopoDS.Face(itLF.Value());

                // Build box
                Box2d aBoxF = new();
                BRepTools.AddUVBounds(aF, ref aBoxF);

                // Select edges for the classification
                BOPTools_Box2dTreeSelector aSelector = new();
                aSelector.SetBVHSet(aBoxTree);
                aSelector.SetBox(Tools.Bnd2BVH(aBoxF));
                if (aSelector.Select() == 0)
                {
                    continue;
                }

                // Collect edges inside the face
                NCollection_IndexedMap<TopoDS_Shape> anEdgesInside = new(TopTools_ShapeMapHasher.Instance);

                NCollection_List<int> aLI = aSelector.Indices();
                NCollection_List<int>.Iterator itLI = new(aLI);
                for (; itLI.More(); itLI.Next())
                {
                    int nE = itLI.Value();
                    if (aMEDone.Contains(nE))
                    {
                        continue;
                    }

                    TopoDS_Edge aE = TopoDS.Edge(anEdgesMap[nE]);
                    if (IsInside(aE, aF, myContext!))
                    {
                        anEdgesInside.Add(aE);
                        aMEDone.Add(nE);
                    }
                }

                if (anEdgesInside.IsEmpty())
                {
                    continue;
                }

                // Make internal wires
                NCollection_List<TopoDS_Shape> aLSI0 = new();
                MakeInternalWires(anEdgesInside, aLSI0);

                // Add wires to a face
                NCollection_List<TopoDS_Shape>.Iterator itLSI = new(aLSI0);
                for (; itLSI.More(); itLSI.Next())
                {
                    TopoDS_Shape aWI = itLSI.Value();
                    new BRep_Builder().Add(aF, aWI);
                }

                // Condition of early exit
                if (aMEDone.Extent() == anEdgesMap.Extent())
                {
                    // All edges are classified and added into the faces
                    return;
                }
            }
        }

        // Some edges are left unclassified - warn user about them
        NCollection_IndexedMap<TopoDS_Shape> anEdgesUnUsed = new(TopTools_ShapeMapHasher.Instance);
        for (int i = 1; i <= anEdgesMap.Extent(); ++i)
        {
            if (!aMEDone.Contains(i))
            {
                anEdgesUnUsed.Add(anEdgesMap[i]);
            }
        }

        // Make internal wires
        NCollection_List<TopoDS_Shape> aLSI = new();
        MakeInternalWires(anEdgesUnUsed, aLSI);

        // Make compound
        TopoDS_Compound aWShape = new();
        new BRep_Builder().MakeCompound(ref aWShape);
        new BRep_Builder().Add(aWShape, myFace);
        if (aLSI.Extent() == 1)
        {
            new BRep_Builder().Add(aWShape, aLSI.First());
        }
        else
        {
            TopoDS_Compound aCE = new();
            new BRep_Builder().MakeCompound(ref aCE);
            for (NCollection_List<TopoDS_Shape>.Iterator it = new(aLSI); it.More(); it.Next())
            {
                new BRep_Builder().Add(aCE, it.Value());
            }
            new BRep_Builder().Add(aWShape, aCE);
        }

        // Add warning
        AddWarning(new BOPAlgo_AlertFaceBuilderUnusedEdges(aWShape));
    }

    private static void MakeInternalWires(NCollection_IndexedMap<TopoDS_Shape> theME, NCollection_List<TopoDS_Shape> theWires)
    {
        int i, aNbE;
        NCollection_Map<TopoDS_Shape> aAddedMap = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape>.Iterator aItE = new(theWires);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMVE = new(TopTools_ShapeMapHasher.Instance);
        BRep_Builder aBB = new();
        //
        aNbE = theME.Extent();
        for (i = 1; i <= aNbE; ++i)
        {
            TopoDS_Shape aE = theME[i];
            TopExp.MapShapesAndAncestors(aE, TopAbs_VERTEX, TopAbs_EDGE, aMVE);
        }
        //
        for (i = 1; i <= aNbE; ++i)
        {
            TopoDS_Shape aEE = theME[i];
            if (!aAddedMap.Add(aEE))
            {
                continue;
            }
            //
            // make a new shell
            TopoDS_Wire aW = new();
            aBB.MakeWire(ref aW);
            aEE.Orientation(TopAbs_INTERNAL);
            aBB.Add(aW, aEE);
            //
            TopoDS_Iterator aItAdded = new(aW);
            for (; aItAdded.More(); aItAdded.Next())
            {
                TopoDS_Shape aE = aItAdded.Value();
                //
                TopExp_Explorer aExp = new(aE, TopAbs_VERTEX);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aV = aExp.Current();
                    NCollection_List<TopoDS_Shape> aLE = aMVE.FindFromKey(aV);
                    aItE.Initialize(aLE);
                    for (; aItE.More(); aItE.Next())
                    {
                        TopoDS_Shape aEL = aItE.Value();
                        if (aAddedMap.Add(aEL))
                        {
                            aEL.Orientation(TopAbs_INTERNAL);
                            aBB.Add(aW, aEL);
                        }
                    }
                }
            }
            aW.Closed(BRep_Tool.IsClosed(aW));
            theWires.Append(aW);
        }
    }

    // Checks if the wire is located inside the face: takes a point of the wire and classifies it relatively the face.
    private static bool IsInside(in TopoDS_Shape theWire, in TopoDS_Shape theF, IntTools_Context theContext)
    {
        // Avoid edges of the face
        NCollection_IndexedMap<TopoDS_Shape> aFaceEdgesMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theF, TopAbs_EDGE, aFaceEdgesMap);

        // Get classification tool from the context
        TopoDS_Face aF = TopoDS.Face(theF);
        IntTools_FClass2d aClassifier = theContext.FClass2d(aF);

        bool isInside = false;

        // Iterate on wire edges until first classification is performed
        TopExp_Explorer anExp = new(theWire, TopAbs_EDGE);
        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(anExp.Current());
            if (BRep_Tool.Degenerated(aE))
            {
                // Avoid checking degenerated edges.
                continue;
            }

            if (aFaceEdgesMap.Contains(aE))
            {
                // Face contains the edge from the wire, thus the wire cannot be
                // inside that face.
                return isInside;
            }

            // Get 2d curve of the edge on the face
            Geom2d_Curve? aC2D = BRep_Tool.CurveOnSurface(aE, aF, out double aT1, out double aT2);
            if (aC2D is null)
            {
                continue;
            }

            // Get middle point on the curve
            Point2D aP2D = aC2D.Value((aT1 + aT2) / 2.0);

            // Classify the point
            TopAbs_State aState = aClassifier.Perform(aP2D);
            isInside = aState == TopAbs_IN;
            break;
        }
        return isInside;
    }

    private static bool IsGrowthWire(in TopoDS_Shape theWire, NCollection_IndexedMap<TopoDS_Shape> theMHE)
    {
        if (theMHE.Extent() != 0)
        {
            TopoDS_Iterator aIt = new(theWire);
            for (; aIt.More(); aIt.Next())
            {
                if (theMHE.Contains(aIt.Value()))
                {
                    return true;
                }
            }
        }
        return false;
    }
}
