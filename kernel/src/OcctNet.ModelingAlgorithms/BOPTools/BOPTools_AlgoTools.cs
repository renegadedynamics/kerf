// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Foundation.Gravity;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>The topological tools of the Boolean component.</summary>
/// <remarks>
/// <para>
/// A static partial class over four files, as OCCT's class is over four .cxx files: this one (BOPTools_AlgoTools.cxx),
/// BOPTools_AlgoTools_1.cs (the tolerance corrections) and BOPTools_AlgoTools_2.cs (vertices, edges, ranges,
/// dimensions). Decision 6440.
/// </para>
/// <para>
/// Context arguments are non-null where OCCT dereferences them unchecked, nullable where OCCT passes them on to
/// functions that accept a null context. Outputs OCCT leaves unwritten on some path are <c>ref</c>.
/// </para>
/// <para>
/// Some functions write into the shapes they are given, as in OCCT (plan 2.3): <see cref="MakeEdge"/> (the
/// tolerances of its two vertices), <see cref="MakePCurve"/> (pcurves, SameParameter on the edge) and the
/// <c>UpdateVertex</c> forms (the vertex's tolerance). Ported callers give them shapes they built or copied; a frozen
/// shape throws at the first write.
/// </para>
/// </remarks>
public static partial class BOPTools_AlgoTools
{
    /// <summary>The tolerance increment used when updating tolerances (1e-12).</summary>
    public static double DTolerance() => 1.0e-12;

    /// <summary>
    /// Makes the connexity blocks of the elements of <paramref name="theS"/> connected through sub-shapes of
    /// <paramref name="theConnectionType"/>, each as a list, and fills the connection map.
    /// </summary>
    public static void MakeConnexityBlocks(in TopoDS_Shape theS,
                                           TopAbs_ShapeEnum theConnectionType,
                                           TopAbs_ShapeEnum theElementType,
                                           NCollection_List<NCollection_List<TopoDS_Shape>> theLCB,
                                           NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theConnectionMap)
    {
        // Map shapes to find connected elements
        TopExp.MapShapesAndAncestors(theS, theConnectionType, theElementType, theConnectionMap);
        // Fence map
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);

        TopExp_Explorer aExp = new(theS, theElementType);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Shape aS = aExp.Current();
            if (!aMFence.Add(aS))
            {
                continue;
            }
            // The block
            NCollection_List<TopoDS_Shape> aLBlock = new();
            // Start the block
            aLBlock.Append(aS);
            // Look for connected parts
            NCollection_List<TopoDS_Shape>.Iterator aItB = new(aLBlock);
            for (; aItB.More(); aItB.Next())
            {
                TopoDS_Shape aS1 = aItB.Value();
                TopExp_Explorer aExpSS = new(aS1, theConnectionType);
                for (; aExpSS.More(); aExpSS.Next())
                {
                    TopoDS_Shape aSubS = aExpSS.Current();
                    NCollection_List<TopoDS_Shape> aLS = theConnectionMap.FindFromKey(aSubS);
                    NCollection_List<TopoDS_Shape>.Iterator aItLS = new(aLS);
                    for (; aItLS.More(); aItLS.Next())
                    {
                        TopoDS_Shape aS2 = aItLS.Value();
                        if (aMFence.Add(aS2))
                        {
                            aLBlock.Append(aS2);
                        }
                    }
                }
            }
            // Add the block into result
            theLCB.Append(aLBlock);
        }
    }

    /// <summary>
    /// Makes the connexity blocks of the elements of <paramref name="theS"/> connected through sub-shapes of
    /// <paramref name="theConnectionType"/>, each as a compound.
    /// </summary>
    public static void MakeConnexityBlocks(in TopoDS_Shape theS,
                                           TopAbs_ShapeEnum theConnectionType,
                                           TopAbs_ShapeEnum theElementType,
                                           NCollection_List<TopoDS_Shape> theLCB)
    {
        NCollection_List<NCollection_List<TopoDS_Shape>> aLBlocks = new();
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aCMap = new(TopTools_ShapeMapHasher.Instance);
        MakeConnexityBlocks(theS, theConnectionType, theElementType, aLBlocks, aCMap);

        // Make compound from each block
        NCollection_List<NCollection_List<TopoDS_Shape>>.Iterator aItB = new(aLBlocks);
        for (; aItB.More(); aItB.Next())
        {
            NCollection_List<TopoDS_Shape> aLB = aItB.Value();

            TopoDS_Compound aBlock = new();
            new BRep_Builder().MakeCompound(ref aBlock);
            for (NCollection_List<TopoDS_Shape>.Iterator it = new(aLB); it.More(); it.Next())
            {
                new BRep_Builder().Add(aBlock, it.Value());
            }

            theLCB.Append(aBlock);
        }
    }

    /// <summary>
    /// Makes the connexity blocks of the shapes of <paramref name="theLS"/> connected through sub-shapes of
    /// <paramref name="theConnectionType"/>, and tells whether each is regular (every connection shared by exactly two
    /// elements, no element given twice).
    /// </summary>
    public static void MakeConnexityBlocks(NCollection_List<TopoDS_Shape> theLS,
                                           TopAbs_ShapeEnum theConnectionType,
                                           TopAbs_ShapeEnum theElementType,
                                           NCollection_List<BOPTools_ConnexityBlock> theLCB)
    {
        BRep_Builder aBB = new();
        // Make connexity blocks from start elements
        TopoDS_Compound aCStart = new();
        aBB.MakeCompound(ref aCStart);

        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance), aMNRegular = new(TopTools_ShapeMapHasher.Instance);

        NCollection_List<TopoDS_Shape>.Iterator aItL = new(theLS);
        for (; aItL.More(); aItL.Next())
        {
            TopoDS_Shape aS = aItL.Value();
            if (aMFence.Add(aS))
            {
                aBB.Add(aCStart, aS);
            }
            else
            {
                aMNRegular.Add(aS);
            }
        }

        NCollection_List<NCollection_List<TopoDS_Shape>> aLCB = new();
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aCMap = new(TopTools_ShapeMapHasher.Instance);
        MakeConnexityBlocks(aCStart, theConnectionType, theElementType, aLCB, aCMap);

        // Save the blocks and check their regularity
        NCollection_List<NCollection_List<TopoDS_Shape>>.Iterator aItB = new(aLCB);
        for (; aItB.More(); aItB.Next())
        {
            NCollection_List<TopoDS_Shape> aBlock = aItB.Value();

            BOPTools_ConnexityBlock aCB = new();
            NCollection_List<TopoDS_Shape> aLCS = aCB.ChangeShapes();

            bool bRegular = true;
            for (NCollection_List<TopoDS_Shape>.Iterator it = new(aBlock); it.More(); it.Next())
            {
                TopoDS_Shape aS = it.Value();
                if (aMNRegular.Contains(aS))
                {
                    bRegular = false;
                    aS.Orientation(TopAbs_FORWARD);
                    aLCS.Append(aS);
                    aS.Orientation(TopAbs_REVERSED);
                    aLCS.Append(aS);
                }
                else
                {
                    aLCS.Append(aS);
                    if (bRegular)
                    {
                        // Check if there are no multi-connected shapes
                        for (TopExp_Explorer ex = new(aS, theConnectionType); ex.More() && bRegular; ex.Next())
                        {
                            bRegular = (aCMap.FindFromKey(ex.Current()).Extent() == 2);
                        }
                    }
                }
            }

            aCB.SetRegular(bRegular);
            // OCCT appends a copy of the local block; the local one is not used afterwards, so it is appended itself.
            theLCB.Append(aCB);
        }
    }

    /// <summary>Reorients the edges of the wire for a correct ordering (connected edges oriented head to tail).</summary>
    public static void OrientEdgesOnWire(ref TopoDS_Shape theWire)
    {
        // make vertex-edges connexity map
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aVEMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(theWire, TopAbs_VERTEX, TopAbs_EDGE, aVEMap);
        //
        if (aVEMap.IsEmpty())
        {
            return;
        }
        //
        BRep_Builder aBB = new();
        // new wire
        TopoDS_Wire aWire = new();
        aBB.MakeWire(ref aWire);
        // fence map (OCCT's default hasher: orientation included, decision 6433)
        NCollection_Map<TopoDS_Shape> aMFence = new(EqualityComparer<TopoDS_Shape>.Default);
        //
        TopoDS_Iterator aIt = new(theWire);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Edge aEC = TopoDS.Edge(aIt.Value());
            if (!aMFence.Add(aEC))
            {
                continue;
            }
            //
            // add edge to a wire as it is
            aBB.Add(aWire, aEC);
            //
            TopExp.Vertices(aEC, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2, true);
            //
            if (aV1.IsSame(aV2))
            {
                // closed edge, go to the next edge
                continue;
            }
            //
            // orient the adjacent edges
            for (int i = 0; i < 2; ++i)
            {
                TopoDS_Shape aVC = i == 0 ? aV1 : aV2;
                //
                for (; ; )
                {
                    NCollection_List<TopoDS_Shape> aLE = aVEMap.FindFromKey(aVC);
                    if (aLE.Extent() != 2)
                    {
                        // free vertex or multi-connexity, go to the next edge
                        break;
                    }
                    //
                    bool bStop = true;
                    //
                    NCollection_List<TopoDS_Shape>.Iterator aItLE = new(aLE);
                    for (; aItLE.More(); aItLE.Next())
                    {
                        TopoDS_Edge aEN = TopoDS.Edge(aItLE.Value());
                        if (aMFence.Contains(aEN))
                        {
                            continue;
                        }
                        //
                        TopExp.Vertices(aEN, out TopoDS_Vertex aVN1, out TopoDS_Vertex aVN2, true);
                        if (aVN1.IsSame(aVN2))
                        {
                            // closed edge, go to the next edge
                            break;
                        }
                        //
                        // change orientation if necessary and go to the next edges
                        if ((i == 0 && aVC.IsSame(aVN2)) || (i != 0 && aVC.IsSame(aVN1)))
                        {
                            aBB.Add(aWire, aEN);
                        }
                        else
                        {
                            aBB.Add(aWire, aEN.Reversed());
                        }
                        aMFence.Add(aEN);
                        aVC = aVC.IsSame(aVN1) ? aVN2 : aVN1;
                        bStop = false;
                        break;
                    }
                    //
                    if (bStop)
                    {
                        break;
                    }
                }
            }
        }
        //
        theWire = aWire;
    }

    /// <summary>
    /// Makes a new shell of the faces of <paramref name="aShell"/>, reversing faces so that each edge shared by two
    /// faces has opposite orientations in them.
    /// </summary>
    public static void OrientFacesOnShell(ref TopoDS_Shape aShell)
    {
        bool bIsProcessed1, bIsProcessed2;
        int i, aNbE, aNbF, j;
        TopAbs_Orientation anOrE1, anOrE2;
        TopoDS_Face aF1x, aF2x;
        TopoDS_Shape aShellNew = new();
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aEFMap = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedMap<TopoDS_Shape> aProcessedFaces = new(TopTools_ShapeMapHasher.Instance);
        BRep_Builder aBB = new();
        //
        MakeContainer(TopAbs_SHELL, ref aShellNew);
        //
        TopExp.MapShapesAndAncestors(aShell, TopAbs_EDGE, TopAbs_FACE, aEFMap);
        aNbE = aEFMap.Extent();
        //
        // One seam edge  in aEFMap contains  2 equivalent faces.
        for (i = 1; i <= aNbE; ++i)
        {
            NCollection_List<TopoDS_Shape> aLF = aEFMap.ChangeFromIndex(i);
            aNbF = aLF.Extent();
            if (aNbF > 1)
            {
                NCollection_List<TopoDS_Shape> aLFTmp = new();
                NCollection_IndexedMap<TopoDS_Shape> aFM = new(TopTools_ShapeMapHasher.Instance);
                //
                NCollection_List<TopoDS_Shape>.Iterator anIt = new(aLF);
                for (; anIt.More(); anIt.Next())
                {
                    TopoDS_Shape aF = anIt.Value();
                    if (!aFM.Contains(aF))
                    {
                        aFM.Add(aF);
                        aLFTmp.Append(aF);
                    }
                }
                aLF.Clear();
                aLF.Assign(aLFTmp);
            }
        }
        //
        // Do
        for (i = 1; i <= aNbE; ++i)
        {
            TopoDS_Edge aE = TopoDS.Edge(aEFMap.FindKey(i));
            if (BRep_Tool.Degenerated(aE))
            {
                continue;
            }
            //
            NCollection_List<TopoDS_Shape> aLF = aEFMap.FindFromIndex(i);
            aNbF = aLF.Extent();
            if (aNbF != 2)
            {
                continue;
            }
            //
            // OCCT holds references into the list and reverses its items in place.
            ref TopoDS_Shape aF1 = ref aLF.ChangeFirst();
            ref TopoDS_Shape aF2 = ref aLF.ChangeLast();
            //
            bIsProcessed1 = aProcessedFaces.Contains(aF1);
            bIsProcessed2 = aProcessedFaces.Contains(aF2);
            if (bIsProcessed1 && bIsProcessed2)
            {
                continue;
            }

            if (!bIsProcessed1 && !bIsProcessed2)
            {
                aProcessedFaces.Add(aF1);
                aBB.Add(aShellNew, aF1);
                bIsProcessed1 = !bIsProcessed1;
            }
            //
            aF1x = TopoDS.Face(aF1);
            if (bIsProcessed1)
            {
                j = aProcessedFaces.FindIndex(aF1);
                aF1x = TopoDS.Face(aProcessedFaces.FindKey(j));
            }
            //
            aF2x = TopoDS.Face(aF2);
            if (bIsProcessed2)
            {
                j = aProcessedFaces.FindIndex(aF2);
                aF2x = TopoDS.Face(aProcessedFaces.FindKey(j));
            }
            //
            anOrE1 = Orientation(aE, aF1x);
            anOrE2 = Orientation(aE, aF2x);
            //
            if (bIsProcessed1 && !bIsProcessed2)
            {
                if (anOrE1 == anOrE2)
                {
                    if (!BRep_Tool.IsClosed(aE, TopoDS.Face(aF1)) && !BRep_Tool.IsClosed(aE, TopoDS.Face(aF2)))
                    {
                        aF2.Reverse();
                    }
                }
                aProcessedFaces.Add(aF2);
                aBB.Add(aShellNew, aF2);
            }
            else if (!bIsProcessed1 && bIsProcessed2)
            {
                if (anOrE1 == anOrE2)
                {
                    if (!BRep_Tool.IsClosed(aE, TopoDS.Face(aF1)) && !BRep_Tool.IsClosed(aE, TopoDS.Face(aF2)))
                    {
                        aF1.Reverse();
                    }
                }
                aProcessedFaces.Add(aF1);
                aBB.Add(aShellNew, aF1);
            }
        }
        //
        //
        for (i = 1; i <= aNbE; ++i)
        {
            TopoDS_Edge aE = TopoDS.Edge(aEFMap.FindKey(i));
            if (BRep_Tool.Degenerated(aE))
            {
                continue;
            }
            //
            NCollection_List<TopoDS_Shape> aLF = aEFMap.FindFromIndex(i);
            aNbF = aLF.Extent();
            if (aNbF != 2)
            {
                NCollection_List<TopoDS_Shape>.Iterator anIt = new(aLF);
                for (; anIt.More(); anIt.Next())
                {
                    TopoDS_Shape aF = anIt.Value();
                    if (!aProcessedFaces.Contains(aF))
                    {
                        aProcessedFaces.Add(aF);
                        aBB.Add(aShellNew, aF);
                    }
                }
            }
        }
        aShell = aShellNew;
    }

    // The orientation of the edge in the face (the first same edge found), INTERNAL if the face hasn't it.
    private static TopAbs_Orientation Orientation(in TopoDS_Edge anE, in TopoDS_Face aF)
    {
        TopAbs_Orientation anOr = TopAbs_INTERNAL;

        TopExp_Explorer anExp = new();
        anExp.Init(aF, TopAbs_EDGE);
        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Shape anEF1 = anExp.Current();
            if (anEF1.IsSame(anE))
            {
                anOr = anEF1.Orientation();
                break;
            }
        }
        return anOr;
    }

    /// <summary>
    /// Makes the connexity block of faces of <paramref name="theLFIn"/> that contains its first face, through edges
    /// not in <paramref name="theMEAvoid"/>, and appends its faces to <paramref name="theLCB"/>.
    /// </summary>
    public static void MakeConnexityBlock(NCollection_List<TopoDS_Shape> theLFIn,
                                          NCollection_IndexedMap<TopoDS_Shape> theMEAvoid,
                                          NCollection_List<TopoDS_Shape> theLCB)
    {
        int aNbF, aNbAdd1, aNbAdd, i;
        TopExp_Explorer aExp = new();
        NCollection_List<TopoDS_Shape>.Iterator aIt = new();
        //
        NCollection_IndexedMap<TopoDS_Shape> aMCB = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedMap<TopoDS_Shape> aMAdd = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedMap<TopoDS_Shape> aMAdd1 = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMEF = new(TopTools_ShapeMapHasher.Instance);
        //
        // 1. aMEF
        aIt.Initialize(theLFIn);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aF = aIt.Value();
            TopExp.MapShapesAndAncestors(aF, TopAbs_EDGE, TopAbs_FACE, aMEF);
        }
        //
        // 2. aMCB
        TopoDS_Shape aF1 = theLFIn.First();
        aMAdd.Add(aF1);
        //
        for (; ; )
        {
            aMAdd1.Clear();
            aNbAdd = aMAdd.Extent();
            for (i = 1; i <= aNbAdd; ++i)
            {
                TopoDS_Shape aF = aMAdd[i];
                //
                // aMAdd1.Clear();
                aExp.Init(aF, TopAbs_EDGE);
                for (; aExp.More(); aExp.Next())
                {
                    TopoDS_Shape aE = aExp.Current();
                    if (theMEAvoid.Contains(aE))
                    {
                        continue;
                    }
                    //
                    NCollection_List<TopoDS_Shape> aLF = aMEF.FindFromKey(aE);
                    aIt.Initialize(aLF);
                    for (; aIt.More(); aIt.Next())
                    {
                        TopoDS_Shape aFx = aIt.Value();
                        if (aFx.IsSame(aF))
                        {
                            continue;
                        }
                        if (aMCB.Contains(aFx))
                        {
                            continue;
                        }
                        aMAdd1.Add(aFx);
                    }
                } // for (; aExp.More(); aExp.Next()){
                aMCB.Add(aF);
            } // for (i=1; i<=aNbAdd; ++i) {
            //
            aNbAdd1 = aMAdd1.Extent();
            if (aNbAdd1 == 0)
            {
                break;
            }
            //
            aMAdd.Clear();
            for (i = 1; i <= aNbAdd1; ++i)
            {
                TopoDS_Shape aFAdd = aMAdd1[i];
                aMAdd.Add(aFAdd);
            }
            //
        } // while(1) {

        //
        aNbF = aMCB.Extent();
        for (i = 1; i <= aNbF; ++i)
        {
            TopoDS_Shape aF = aMCB[i];
            theLCB.Append(aF);
        }
    }

    /// <summary>
    /// The state of the shape relative to the solid, from one point of it: a vertex's point, an edge's intermediate
    /// point, a face's point (see the face form of <see cref="ComputeState(in TopoDS_Face, in TopoDS_Solid, double,
    /// NCollection_IndexedMap{TopoDS_Shape}, IntTools_Context)"/>). UNKNOWN for other types.
    /// </summary>
    public static TopAbs_State ComputeStateByOnePoint(in TopoDS_Shape theS, in TopoDS_Solid theRef, double theTol,
                                                      IntTools_Context theContext)
    {
        TopAbs_State aState = TopAbs_UNKNOWN;
        TopAbs_ShapeEnum aType = theS.ShapeType();

        switch (aType)
        {
            case TopAbs_VERTEX:
                aState = ComputeState(TopoDS.Vertex(theS), theRef, theTol, theContext);
                break;
            case TopAbs_EDGE:
                aState = ComputeState(TopoDS.Edge(theS), theRef, theTol, theContext);
                break;
            case TopAbs_FACE:
                {
                    NCollection_IndexedMap<TopoDS_Shape> aBounds = new(TopTools_ShapeMapHasher.Instance);
                    TopExp.MapShapes(theRef, TopAbs_EDGE, aBounds);
                    aState = ComputeState(TopoDS.Face(theS), theRef, theTol, aBounds, theContext);
                    break;
                }
            default:
                {
                    TopoDS_Iterator it = new(theS);
                    if (it.More())
                    {
                        // OCCT discards the state of the first sub-shape, so a container gives UNKNOWN.
                        ComputeStateByOnePoint(it.Value(), theRef, theTol, theContext);
                    }
                    break;
                }
        }
        return aState;
    }

    /// <summary>
    /// The state of the face relative to the solid: the state of an edge of the face that isn't in
    /// <paramref name="theBounds"/> (the solid's edges), else of a point inside the face.
    /// </summary>
    public static TopAbs_State ComputeState(in TopoDS_Face theF, in TopoDS_Solid theRef, double theTol,
                                            NCollection_IndexedMap<TopoDS_Shape> theBounds, IntTools_Context theContext)
    {
        TopAbs_State aState = TopAbs_UNKNOWN;

        // Try to find the edge on the face which does not
        // belong to the solid and classify the middle point of that
        // edge relatively solid.
        TopExp_Explorer aExp = new(theF, TopAbs_EDGE);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Edge aSE = TopoDS.Edge(aExp.Current());
            if (BRep_Tool.Degenerated(aSE))
            {
                continue;
            }

            if (!theBounds.Contains(aSE))
            {
                aState = ComputeState(aSE, theRef, theTol, theContext);
                return aState;
            }
        }

        // All edges of the face are on the solid.
        // Get point inside the face and classify it relatively solid.
        Point3D aP3D = new();
        Point2D aP2D = new();
        int iErr = BOPTools_AlgoTools3D.PointInFace(theF, ref aP3D, ref aP2D, theContext);
        if (iErr != 0)
        {
            // Hatcher fails to find the point -> get point near some edge
            aExp.Init(theF, TopAbs_EDGE);
            for (; aExp.More() && iErr != 0; aExp.Next())
            {
                TopoDS_Edge aSE = TopoDS.Edge(aExp.Current());
                if (BRep_Tool.Degenerated(aSE))
                {
                    continue;
                }

                iErr = BOPTools_AlgoTools3D.PointNearEdge(aSE, theF, ref aP2D, ref aP3D, theContext);
            }
        }

        if (iErr == 0)
        {
            aState = ComputeState(aP3D, theRef, theTol, theContext);
        }

        return aState;
    }

    /// <summary>The state of the vertex's point relative to the solid.</summary>
    public static TopAbs_State ComputeState(in TopoDS_Vertex theV, in TopoDS_Solid theRef, double theTol, IntTools_Context theContext)
    {
        TopAbs_State aState;
        Point3D aP3D;
        //
        aP3D = BRep_Tool.Pnt(theV);
        aState = ComputeState(aP3D, theRef, theTol, theContext);
        return aState;
    }

    /// <summary>
    /// The state of the edge's intermediate point (or of its vertex, for an edge without a 3D curve) relative to the
    /// solid.
    /// </summary>
    public static TopAbs_State ComputeState(in TopoDS_Edge theE, in TopoDS_Solid theRef, double theTol, IntTools_Context theContext)
    {
        double aT = 0.0;
        TopAbs_State aState;
        Geom_Curve? aC3D;
        Point3D aP3D;
        //
        aC3D = BRep_Tool.Curve(theE, out double aT1, out double aT2);
        //
        if (aC3D is null)
        {
            // it means that we are in degenerated edge
            TopoDS_Vertex aV = TopExp.FirstVertex(theE);
            if (aV.IsNull())
            {
                return TopAbs_UNKNOWN;
            }
            aP3D = BRep_Tool.Pnt(aV);
        }
        else
        { // usual case
            bool bF2Inf, bL2Inf;
            double dT = 10.0;
            //
            bF2Inf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(aT1);
            bL2Inf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(aT2);
            //
            if (bF2Inf && !bL2Inf)
            {
                aT = aT2 - dT;
            }
            else if (!bF2Inf && bL2Inf)
            {
                aT = aT1 + dT;
            }
            else if (bF2Inf && bL2Inf)
            {
                aT = 0.0;
            }
            else
            {
                aT = IntTools_Tools.IntermediatePoint(aT1, aT2);
            }
            aC3D.D0(aT, out aP3D);
        }
        //
        aState = ComputeState(aP3D, theRef, theTol, theContext);
        //
        return aState;
    }

    /// <summary>The state of the point relative to the solid, by the context's solid classifier.</summary>
    public static TopAbs_State ComputeState(in Point3D theP, in TopoDS_Solid theRef, double theTol, IntTools_Context theContext)
    {
        TopAbs_State aState;
        //
        BRepClass3d_SolidClassifier aSC = theContext.SolidClassifier(theRef);
        aSC.Perform(theP, theTol);
        //
        aState = aSC.State();
        //
        return aState;
    }

    /// <summary>
    /// True if the face is inside the solid: by the angles between the faces at an edge of the face found in
    /// <paramref name="theMEF"/> (the solid's edge/face map), else by classifying a point of the face.
    /// </summary>
    public static bool IsInternalFace(in TopoDS_Face theFace,
                                      in TopoDS_Solid theSolid,
                                      NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> theMEF,
                                      double theTol,
                                      IntTools_Context theContext)
    {
        bool bDegenerated;
        TopAbs_Orientation aOr;
        TopoDS_Edge aE1 = new();
        TopExp_Explorer aExp = new();
        //
        // For all invoked functions: [::IsInternalFace(...)]
        // the returned value iRet means:
        // iRet=0;  - state is not IN
        // iRet=1;  - state is IN
        // iRet=2;  - state can not be found by the method of angles
        int iRet = 0;
        // 1 Try to find an edge from theFace in theMEF
        aExp.Init(theFace, TopAbs_EDGE);
        for (; aExp.More(); aExp.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(aExp.Current());
            if (!theMEF.Contains(aE))
            {
                continue;
            }
            //
            aOr = aE.Orientation();
            if (aOr == TopAbs_INTERNAL)
            {
                continue;
            }
            bDegenerated = BRep_Tool.Degenerated(aE);
            if (bDegenerated)
            {
                continue;
            }
            // aE
            NCollection_List<TopoDS_Shape> aLF = theMEF.ChangeFromKey(aE);
            int aNbF = aLF.Extent();
            if (aNbF == 1)
            {
                // aE is internal edge on aLF.First()
                TopoDS_Face aF1 = TopoDS.Face(aLF.First());
                GetEdgeOnFace(aE, aF1, ref aE1);
                if (aE1.Orientation() != TopAbs_INTERNAL)
                {
                    continue;
                }
                //
                iRet = IsInternalFace(theFace, aE, aF1, aF1, theContext);
                break;
            }
            //
            else if (aNbF == 2)
            {
                TopoDS_Face aF1 = TopoDS.Face(aLF.First());
                TopoDS_Face aF2 = TopoDS.Face(aLF.Last());
                iRet = IsInternalFace(theFace, aE, aF1, aF2, theContext);
                if (iRet != 2)
                {
                    break;
                }
            }
        } // for(; aExp.More(); aExp.Next()) {
        //
        if (aExp.More() && iRet != 2)
        {
            return iRet == 1;
        }
        //
        //========================================
        // 2. Classify face using classifier
        //
        TopAbs_State aState;
        NCollection_IndexedMap<TopoDS_Shape> aBounds = new(TopTools_ShapeMapHasher.Instance);
        //
        TopExp.MapShapes(theSolid, TopAbs_EDGE, aBounds);
        //
        aState = ComputeState(theFace, theSolid, theTol, aBounds, theContext);
        return aState == TopAbs_IN;
    }

    /// <summary>
    /// Whether the face is inside the faces <paramref name="theLF"/> sharing <paramref name="theEdge"/>: 1 if IN, 0 if
    /// not, 2 if the angles can't tell.
    /// </summary>
    public static int IsInternalFace(in TopoDS_Face theFace, in TopoDS_Edge theEdge, NCollection_List<TopoDS_Shape> theLF,
                                     IntTools_Context theContext)
    {
        int aNbF, iRet;
        //
        iRet = 0;
        //
        aNbF = theLF.Extent();
        if (aNbF == 2)
        {
            TopoDS_Face aF1 = TopoDS.Face(theLF.First());
            TopoDS_Face aF2 = TopoDS.Face(theLF.Last());
            iRet = IsInternalFace(theFace, theEdge, aF1, aF2, theContext);
            return iRet;
        }
        //
        else
        {
            NCollection_List<BOPTools_CoupleOfShape> aLCFF = new();
            NCollection_List<BOPTools_CoupleOfShape>.Iterator aIt = new();
            //
            FindFacePairs(theEdge, theLF, aLCFF, theContext);
            //
            aIt.Initialize(aLCFF);
            for (; aIt.More(); aIt.Next())
            {
                BOPTools_CoupleOfShape aCSFF = aIt.Value();
                //
                TopoDS_Face aF1 = TopoDS.Face(aCSFF.Shape1());
                TopoDS_Face aF2 = TopoDS.Face(aCSFF.Shape2());
                iRet = IsInternalFace(theFace, theEdge, aF1, aF2, theContext);
                if (iRet != 0)
                {
                    return iRet;
                }
            }
        }
        return iRet;
    }

    /// <summary>
    /// Whether the face is between <paramref name="theFace1"/> and <paramref name="theFace2"/> at
    /// <paramref name="theEdge"/>: 1 if IN, 0 if not, 2 if the angles can't tell.
    /// </summary>
    public static int IsInternalFace(in TopoDS_Face theFace, in TopoDS_Edge theEdge, in TopoDS_Face theFace1, in TopoDS_Face theFace2,
                                     IntTools_Context theContext)
    {
        TopoDS_Edge aE1 = new(), aE2 = new();
        TopoDS_Face aFOff = new();
        NCollection_List<BOPTools_CoupleOfShape> theLCSOff = new();
        BOPTools_CoupleOfShape aCS1 = new(), aCS2 = new();
        //
        GetEdgeOnFace(theEdge, theFace1, ref aE1);
        if (aE1.Orientation() == TopAbs_INTERNAL)
        {
            aE2 = aE1;
            aE1.Orientation(TopAbs_FORWARD);
            aE2.Orientation(TopAbs_REVERSED);
        }
        else if (theFace1 == theFace2)
        {
            aE2 = aE1;
            aE1.Orientation(TopAbs_FORWARD);
            aE2.Orientation(TopAbs_REVERSED);
        }
        else
        {
            GetEdgeOnFace(theEdge, theFace2, ref aE2);
        }
        //
        aCS1.SetShape1(theEdge);
        aCS1.SetShape2(theFace);
        theLCSOff.Append(aCS1);
        //
        aCS2.SetShape1(aE2);
        aCS2.SetShape2(theFace2);
        theLCSOff.Append(aCS2);
        //
        int iRet = 0; // theFace is not internal
        bool isDone = GetFaceOff(aE1, theFace1, theLCSOff, ref aFOff, theContext);
        if (!isDone)
        {
            // error, unable to classify face by this edge
            iRet = 2;
        }
        else if (theFace.IsEqual(aFOff))
        {
            // theFace is internal
            iRet = 1;
        }

        return iRet;
    }

    /// <summary>
    /// Among the edge/face couples <paramref name="theLCSOff"/>, finds the face <paramref name="theFOff"/> making the
    /// smallest angle with <paramref name="theF1"/> around the edge <paramref name="theE1"/>. False if the smallest
    /// angle can't be told apart (<paramref name="theFOff"/> may still be set).
    /// </summary>
    public static bool GetFaceOff(in TopoDS_Edge theE1, in TopoDS_Face theF1, NCollection_List<BOPTools_CoupleOfShape> theLCSOff,
                                  ref TopoDS_Face theFOff, IntTools_Context theContext)
    {
        bool bRet, bIsComputed;
        double aT, aAngle, aTwoPI, aAngleMin, aDt3D;
        Point3D aPx;
        Direction3D aDBF, aDBF2, aDTF;
        Vector3D aVTgt = new();
        TopAbs_Orientation aOr;
        Geom_Curve aC3D;
        Geom_Plane aPL;
        NCollection_List<BOPTools_CoupleOfShape>.Iterator aIt = new();
        GeomAPI_ProjectPointOnSurf aProjPL = new();
        //
        aAngleMin = 100.0;
        aTwoPI = M_PI + M_PI;
        aC3D = BRep_Tool.Curve(theE1, out double aT1, out double aT2)!;
        aT = BOPTools_AlgoTools2D.IntermediatePoint(aT1, aT2);
        aC3D.D0(aT, out aPx);
        //
        BOPTools_AlgoTools2D.EdgeTangent(theE1, aT, ref aVTgt);
        Direction3D aDTgt = new(aVTgt), aDTgt2;
        aOr = theE1.Orientation();
        //
        aPL = new Geom_Plane(aPx, aDTgt);
        aPL.Bounds(out double aUmin, out double aUsup, out double aVmin, out double aVsup);
        aProjPL.Init(aPL, aUmin, aUsup, aVmin, aVsup);
        //
        bool bSmallFaces = false;
        aDt3D = MinStep3D(theE1, theF1, theLCSOff, aPx, theContext, ref bSmallFaces);
        bIsComputed = GetFaceDir(theE1, theF1, aPx, aT, aDTgt, bSmallFaces, out Direction3D aDN1, out aDBF, theContext, aProjPL, aDt3D);
        // OCCT_DEBUG only: a message when the bi-normal direction is incorrect.
        //
        aDTF = aDN1 ^ aDBF;
        //
        // The difference between faces should be obvious enough
        // to guarantee the correctness of the classification
        double anAngleCriteria = global::Kerf.Engine.Foundation.Precision.Confusion();

        bRet = true;
        aIt.Initialize(theLCSOff);
        for (; aIt.More(); aIt.Next())
        {
            BOPTools_CoupleOfShape aCS = aIt.Value();
            TopoDS_Edge aE2 = TopoDS.Edge(aCS.Shape1());
            TopoDS_Face aF2 = TopoDS.Face(aCS.Shape2());
            //
            aDTgt2 = (aE2.Orientation() == aOr) ? aDTgt : aDTgt.Reversed();
            bIsComputed = GetFaceDir(aE2, aF2, aPx, aT, aDTgt2, bSmallFaces, out _, out aDBF2, theContext, aProjPL, aDt3D);
            // OCCT_DEBUG only: a message when the bi-normal direction is incorrect.
            // Angle
            aAngle = AngleWithRef(aDBF, aDBF2, aDTF);
            //
            if (Math.Abs(aAngle) < global::Kerf.Engine.Foundation.Precision.Angular())
            {
                if (aF2 == theF1)
                {
                    aAngle = M_PI;
                }
                else if (aF2.IsSame(theF1))
                {
                    aAngle = aTwoPI;
                }
                else if (!bIsComputed)
                {
                    // Near-zero angle between different faces where the bi-normal direction could not be
                    // reliably computed (near-degenerate geometry, e.g. very thin face producing
                    // nearly-coincident intersection curves). Treat as maximum angle (2*PI) to prevent
                    // incorrect selection of this unreliable face over a proper closing face.
                    aAngle = aTwoPI;
                }
            }
            //
            if (Math.Abs(aAngle) < anAngleCriteria || Math.Abs(aAngle - aAngleMin) < anAngleCriteria)
            {
                // the minimal angle can not be found
                bRet = false;
            }
            //
            if (aAngle < 0.0)
            {
                aAngle = aTwoPI + aAngle;
            }
            //
            if (aAngle < aAngleMin)
            {
                aAngleMin = aAngle;
                theFOff = aF2;
            }
        }
        return bRet;
    }

    /// <summary>Finds in the face the edge same as <paramref name="theE1"/> with the opposite orientation.</summary>
    public static bool GetEdgeOff(in TopoDS_Edge theE1, in TopoDS_Face theF2, ref TopoDS_Edge theE2)
    {
        bool bFound;
        TopAbs_Orientation aOr1, aOr1C, aOr2;
        TopExp_Explorer anExp = new();
        //
        bFound = false;
        aOr1 = theE1.Orientation();
        aOr1C = TopAbs.Reverse(aOr1);
        //
        anExp.Init(theF2, TopAbs_EDGE);
        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Edge aEF2 = TopoDS.Edge(anExp.Current());
            if (aEF2.IsSame(theE1))
            {
                aOr2 = aEF2.Orientation();
                if (aOr2 == aOr1C)
                {
                    theE2 = aEF2;
                    bFound = !bFound;
                    return bFound;
                }
            }
        }
        return bFound;
    }

    /// <summary>
    /// True if the faces coincide: a point inside the first face is valid for the second, within the faces' and their
    /// edges' tolerances and the fuzzy value.
    /// </summary>
    public static bool AreFacesSameDomain(in TopoDS_Face theF1, in TopoDS_Face theF2, IntTools_Context theContext) =>
        AreFacesSameDomain(theF1, theF2, theContext, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <inheritdoc cref="AreFacesSameDomain(in TopoDS_Face, in TopoDS_Face, IntTools_Context)"/>
    public static bool AreFacesSameDomain(in TopoDS_Face theF1, in TopoDS_Face theF2, IntTools_Context theContext, double theFuzz)
    {
        bool bFacesSD = false;

        // The idea is to find a point inside the first face
        // and check its validity for the second face.
        // If valid - the faces are same domain.

        Point3D aP1 = new();
        Point2D aP2D1 = new();
        // Find point inside the first face
        int iErr = BOPTools_AlgoTools3D.PointInFace(theF1, ref aP1, ref aP2D1, theContext);

        if (iErr != 0)
        {
            // unable to find the point
            return bFacesSD;
        }

        // Check validity of the point for second face

        // Compute the tolerance to check the validity -
        // sum of tolerance of faces and fuzzy tolerance

        // Compute the tolerance of the faces, taking into account the deviation
        // of the edges from the surfaces
        double aTolF1 = BRep_Tool.Tolerance(theF1), aTolF2 = BRep_Tool.Tolerance(theF2);

        // Find maximal tolerance of edges.
        // The faces should have the same boundaries, thus
        // it does not matter which face to explore.
        {
            double aTolEMax = -1.0;
            TopExp_Explorer anExpE = new(theF1, TopAbs_EDGE);
            for (; anExpE.More(); anExpE.Next())
            {
                TopoDS_Edge aE = TopoDS.Edge(anExpE.Current());
                if (!BRep_Tool.Degenerated(aE))
                {
                    double aTolE = BRep_Tool.Tolerance(aE);
                    if (aTolE > aTolEMax)
                    {
                        aTolEMax = aTolE;
                    }
                }
            }
            if (aTolEMax > aTolF1)
            {
                aTolF1 = aTolEMax;
            }
            if (aTolEMax > aTolF2)
            {
                aTolF2 = aTolEMax;
            }
        }

        // Checking criteria
        double aTol = aTolF1 + aTolF2 + Max(theFuzz, global::Kerf.Engine.Foundation.Precision.Confusion());

        // Project and classify the point on second face
        bFacesSD = theContext.IsValidPointForFace(aP1, theF2, aTol);

        return bFacesSD;
    }

    /// <summary>
    /// The relative sense of two faces sharing an edge: 0 if they share no (non-degenerated, non-seam) edge, else
    /// <see cref="BOPTools_AlgoTools3D.SenseFlag"/> of their normals at it.
    /// </summary>
    public static int Sense(in TopoDS_Face theF1, in TopoDS_Face theF2, IntTools_Context theContext)
    {
        int iSense = 0;
        Direction3D aDNF1, aDNF2;
        TopoDS_Edge aE1 = new(), aE2 = new();
        TopExp_Explorer aExp = new();
        //
        aExp.Init(theF1, TopAbs_EDGE);
        for (; aExp.More(); aExp.Next())
        {
            aE1 = TopoDS.Edge(aExp.Current());
            if (!BRep_Tool.Degenerated(aE1))
            {
                if (!BRep_Tool.IsClosed(aE1, theF1))
                {
                    break;
                }
            }
        }
        //
        aExp.Init(theF2, TopAbs_EDGE);
        for (; aExp.More(); aExp.Next())
        {
            aE2 = TopoDS.Edge(aExp.Current());
            if (!BRep_Tool.Degenerated(aE2))
            {
                if (!BRep_Tool.IsClosed(aE2, theF2))
                {
                    if (aE2.IsSame(aE1))
                    {
                        iSense = 1;
                        break;
                    }
                }
            }
        }
        //
        if (iSense == 0)
        {
            return iSense;
        }
        //
        BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(aE1, theF1, out aDNF1, theContext);
        BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(aE2, theF2, out aDNF2, theContext);
        //
        iSense = BOPTools_AlgoTools3D.SenseFlag(aDNF1, aDNF2);
        //
        return iSense;
    }

    /// <summary>
    /// True if the split (an edge or a face) must be reversed to have the orientation of the original shape it was
    /// split from. Other types give false.
    /// </summary>
    public static bool IsSplitToReverse(in TopoDS_Shape theSp, in TopoDS_Shape theSr, IntTools_Context? theContext) =>
        IsSplitToReverse(theSp, theSr, theContext, out _);

    /// <summary>
    /// As <see cref="IsSplitToReverse(in TopoDS_Shape, in TopoDS_Shape, IntTools_Context?)"/>, with the error status
    /// (0 if fine, 100 for an unsupported type; the face and edge forms give their own codes). OCCT's
    /// <c>int* theError</c>.
    /// </summary>
    public static bool IsSplitToReverse(in TopoDS_Shape theSp, in TopoDS_Shape theSr, IntTools_Context? theContext, out int theError)
    {
        bool bRet;
        TopAbs_ShapeEnum aType;
        //
        bRet = false;
        theError = 0; // the edge and face forms always set it
        //
        aType = theSp.ShapeType();
        switch (aType)
        {
            case TopAbs_EDGE:
                {
                    TopoDS_Edge aESp = TopoDS.Edge(theSp);
                    TopoDS_Edge aESr = TopoDS.Edge(theSr);
                    bRet = IsSplitToReverse(aESp, aESr, theContext, out theError);
                }
                break;
            //
            case TopAbs_FACE:
                {
                    TopoDS_Face aFSp = TopoDS.Face(theSp);
                    TopoDS_Face aFSr = TopoDS.Face(theSr);
                    bRet = IsSplitToReverse(aFSp, aFSr, theContext, out theError);
                }
                break;
            //
            default:
                theError = 100;
                break;
        }
        return bRet;
    }

    /// <summary>
    /// As <see cref="IsSplitToReverse(in TopoDS_Shape, in TopoDS_Shape, IntTools_Context?)"/>; when the check fails,
    /// adds a <see cref="BOPAlgo_AlertUnableToOrientTheShape"/> warning (with both shapes) to the report.
    /// </summary>
    public static bool IsSplitToReverseWithWarn(in TopoDS_Shape theSplit, in TopoDS_Shape theShape, IntTools_Context? theContext,
                                                Report? theReport = null)
    {
        bool isToReverse = IsSplitToReverse(theSplit, theShape, theContext, out int anErr);
        if (anErr != 0 && theReport is not null)
        {
            // The error occurred during the check.
            // Add warning to the report, storing the shapes into the warning.
            TopoDS_Compound aWC = new();
            new BRep_Builder().MakeCompound(ref aWC);
            new BRep_Builder().Add(aWC, theSplit);
            new BRep_Builder().Add(aWC, theShape);
            theReport.AddAlert(Warning, new BOPAlgo_AlertUnableToOrientTheShape(aWC));
        }
        return isToReverse;
    }

    /// <summary>
    /// True if the split face must be reversed: its normal at a point inside it points against the original face's
    /// normal at the projection of that point (orientations included).
    /// </summary>
    public static bool IsSplitToReverse(in TopoDS_Face theFSp, in TopoDS_Face theFSr, IntTools_Context? theContext) =>
        IsSplitToReverse(theFSp, theFSr, theContext, out _);

    /// <summary>
    /// As <see cref="IsSplitToReverse(in TopoDS_Face, in TopoDS_Face, IntTools_Context?)"/>, with the error status: 0
    /// fine, 1 no point found in the split, 2 no normal on the split, 3 the projection failed, 4 no normal on the
    /// original.
    /// </summary>
    public static bool IsSplitToReverse(in TopoDS_Face theFSp, in TopoDS_Face theFSr, IntTools_Context? theContext, out int theError)
    {
        // Set OK error status
        theError = 0;

        // Compare surfaces
        Geom_Surface aSFSp = BRep_Tool.Surface(theFSp)!;
        Geom_Surface aSFOr = BRep_Tool.Surface(theFSr)!;
        if (ReferenceEquals(aSFSp, aSFOr))
        {
            return theFSp.Orientation() != theFSr.Orientation();
        }
        //
        bool bDone = false;
        // Find the point inside the split face
        Point3D aPFSp = new();
        Point2D aP2DFSp = new();
        //
        // Error status
        int iErr;
        // Use the hatcher to find the point in the middle of the face
        iErr = BOPTools_AlgoTools3D.PointInFace(theFSp, ref aPFSp, ref aP2DFSp, theContext!);
        if (iErr != 0)
        {
            // Hatcher has failed to find a point.
            // Try to get the point near some not closed and
            // not degenerated edge on the split face.
            TopExp_Explorer anExp = new(theFSp, TopAbs_EDGE);
            for (; anExp.More(); anExp.Next())
            {
                TopoDS_Edge aESp = TopoDS.Edge(anExp.Current());
                if (!BRep_Tool.Degenerated(aESp) && !BRep_Tool.IsClosed(aESp, theFSp))
                {
                    iErr = BOPTools_AlgoTools3D.PointNearEdge(aESp, theFSp, ref aP2DFSp, ref aPFSp, theContext!);
                    if (iErr == 0)
                    {
                        break;
                    }
                }
            }
            //
            if (!anExp.More())
            {
                theError = 1;
                // The point has not been found.
                return bDone;
            }
        }
        //
        // Compute normal direction of the split face
        Direction3D aDNFSp = new();
        bDone = BOPTools_AlgoTools3D.GetNormalToSurface(aSFSp, aP2DFSp.X(), aP2DFSp.Y(), ref aDNFSp);
        if (!bDone)
        {
            theError = 2;
            return bDone;
        }
        //
        if (theFSp.Orientation() == TopAbs_REVERSED)
        {
            aDNFSp.Reverse();
        }
        //
        // Project the point from the split face on the original face
        // to find its UV coordinates
        GeomAPI_ProjectPointOnSurf aProjector = theContext!.ProjPS(theFSr);
        aProjector.Perform(aPFSp);
        bDone = (aProjector.NbPoints() > 0);
        if (!bDone)
        {
            theError = 3;
            return bDone;
        }
        // UV coordinates of the point on the original face
        aProjector.LowerDistanceParameters(out double aU, out double aV);
        //
        // Compute normal direction for the original face in this point
        Direction3D aDNFOr = new();
        bDone = BOPTools_AlgoTools3D.GetNormalToSurface(aSFOr, aU, aV, ref aDNFOr);
        if (!bDone)
        {
            theError = 4;
            return bDone;
        }
        //
        if (theFSr.Orientation() == TopAbs_REVERSED)
        {
            aDNFOr.Reverse();
        }
        //
        // compare the normals
        double aCos = aDNFSp * aDNFOr;
        return (aCos < 0.0);
    }

    /// <summary>
    /// True if the split edge must be reversed: its tangent at a point inside it points against the original edge's
    /// tangent at the projection of that point (orientations included).
    /// </summary>
    public static bool IsSplitToReverse(in TopoDS_Edge theESp, in TopoDS_Edge theEOr, IntTools_Context? theContext) =>
        IsSplitToReverse(theESp, theEOr, theContext, out _);

    /// <summary>
    /// As <see cref="IsSplitToReverse(in TopoDS_Edge, in TopoDS_Edge, IntTools_Context?)"/>, with the error status: 0
    /// fine, 1 a degenerated edge, else the last failure over the samples (2 no tangent on the split, 3 the projection
    /// failed, 4 no tangent on the original).
    /// </summary>
    public static bool IsSplitToReverse(in TopoDS_Edge theESp, in TopoDS_Edge theEOr, IntTools_Context? theContext, out int theError)
    {
        // The idea is to compare the tangent vectors of two edges computed in
        // the same point. Thus, we need to take the point on split edge (since it is
        // shorter) and project it onto original edge to find corresponding parameter.

        if (BRep_Tool.Degenerated(theESp) || BRep_Tool.Degenerated(theEOr))
        {
            theError = 1;
            return false;
        }

        // Set OK error status
        theError = 0;

        // Get the curves from the edges
        Geom_Curve? aCSp = BRep_Tool.Curve(theESp, out double f, out double l);
        Geom_Curve? aCOr = BRep_Tool.Curve(theEOr, out f, out l);

        // If the curves are the same, compare orientations only
        if (ReferenceEquals(aCSp, aCOr))
        {
            return theESp.Orientation() != theEOr.Orientation();
        }

        // Find valid range of the split edge, to ensure that the point for computing
        // tangent vectors will be inside both edges.
        if (!BRepLib.FindValidRange(theESp, ref f, ref l))
        {
            BRep_Tool.Range(theESp, out f, out l);
        }

        // Error code
        int anErr = 0;
        // Try a few sample points on the split edge until first valid found
        const int aNbP = 11;
        double aDT = (l - f) / aNbP;
        for (int i = 1; i < aNbP; ++i)
        {
            double aTm = f + i * aDT;
            // Compute tangent vector on split edge
            Vector3D aVSpTgt = new();
            if (!BOPTools_AlgoTools2D.EdgeTangent(theESp, aTm, ref aVSpTgt))
            {
                // Unable to compute the tangent vector on the split edge
                // in this point -> take the next point
                anErr = 2;
                continue;
            }

            // Find corresponding parameter on the original edge
            double aTmOr = 0.0;
            if (!theContext!.ProjectPointOnEdge(aCSp!.Value(aTm), theEOr, ref aTmOr))
            {
                // Unable to project the point inside the split edge
                // onto the original edge -> take the next point
                anErr = 3;
                continue;
            }

            // Compute tangent vector on original edge
            Vector3D aVOrTgt = new();
            if (!BOPTools_AlgoTools2D.EdgeTangent(theEOr, aTmOr, ref aVOrTgt))
            {
                // Unable to compute the tangent vector on the original edge
                // in this point -> take the next point
                anErr = 4;
                continue;
            }

            // Compute the Dot product
            double aCos = aVSpTgt.Dot(aVOrTgt);
            return (aCos < 0.0);
        }

        theError = anErr;

        return false;
    }

    /// <summary>
    /// True if the wire is a hole in the face: the signed area its pcurves sweep (sampled) is positive.
    /// </summary>
    public static bool IsHole(in TopoDS_Shape aW, in TopoDS_Shape aFace)
    {
        bool bIsHole;
        int i, aNbS;
        double aS;
        double aU1, aU, dU;
        double aX1, aY1, aX0, aY0;
        TopAbs_Orientation aOr;

        Point2D aP2D0, aP2D1;
        Geom2d_Curve? aC2D;
        TopoDS_Face aF, aFF;
        TopoDS_Iterator aItW = new();
        //
        bIsHole = false;
        //
        aF = TopoDS.Face(aFace);
        aFF = aF;
        aFF.Orientation(TopAbs_FORWARD);
        //
        aS = 0.0;
        aItW.Initialize(aW);
        for (; aItW.More(); aItW.Next())
        {
            TopoDS_Edge aE = TopoDS.Edge(aItW.Value());
            aOr = aE.Orientation();
            if (aOr != TopAbs_FORWARD && aOr != TopAbs_REVERSED)
            {
                continue;
            }
            //
            aC2D = BRep_Tool.CurveOnSurface(aE, aFF, out double aT1, out double aT2);
            if (aC2D is null)
            {
                break; // xx
            }
            //
            BRepAdaptor_Curve2d aBAC2D = new(aE, aFF);
            aNbS = Geom2dInt_Geom2dCurveTool.NbSamples(aBAC2D);
            if (aNbS > 2)
            {
                aNbS *= 4;
            }
            //
            dU = (aT2 - aT1) / (double)(aNbS - 1);
            aU = aT1;
            aU1 = aT1;
            if (aOr == TopAbs_REVERSED)
            {
                aU = aT2;
                aU1 = aT2;
                dU = -dU;
            }
            //
            aBAC2D.D0(aU, out aP2D0);
            for (i = 2; i <= aNbS; i++)
            {
                aU = aU1 + (i - 1) * dU;
                aBAC2D.D0(aU, out aP2D1);
                aP2D0.Coord(out aX0, out aY0);
                aP2D1.Coord(out aX1, out aY1);
                //
                aS = aS + (aY0 + aY1) * (aX1 - aX0);
                //
                aP2D0 = aP2D1;
            }
        } // for (; aItW.More(); aItW.Next()) {
        bIsHole = (aS > 0.0);
        return bIsHole;
    }

    /// <summary>
    /// Makes an empty container of the type (compound, compsolid, solid, shell or wire); other types leave
    /// <paramref name="theC"/> unchanged.
    /// </summary>
    public static void MakeContainer(TopAbs_ShapeEnum theType, ref TopoDS_Shape theC)
    {
        BRep_Builder aBB = new();
        //
        switch (theType)
        {
            case TopAbs_COMPOUND:
                {
                    TopoDS_Compound aC = new();
                    aBB.MakeCompound(ref aC);
                    theC = aC;
                }
                break;
            //
            case TopAbs_COMPSOLID:
                {
                    TopoDS_CompSolid aCS = new();
                    aBB.MakeCompSolid(ref aCS);
                    theC = aCS;
                }
                break;
            //
            case TopAbs_SOLID:
                {
                    TopoDS_Solid aSolid = new();
                    aBB.MakeSolid(ref aSolid);
                    theC = aSolid;
                }
                break;
            //
            //
            case TopAbs_SHELL:
                {
                    TopoDS_Shell aShell = new();
                    aBB.MakeShell(ref aShell);
                    theC = aShell;
                }
                break;
            //
            case TopAbs_WIRE:
                {
                    TopoDS_Wire aWire = new();
                    aBB.MakeWire(ref aWire);
                    theC = aWire;
                }
                break;
            //
            default:
                break;
        }
    }

    /// <summary>
    /// Adds to the edge its pcurves on the faces where asked: the intersection curve's own pcurve, or one built,
    /// adjusted to the face's periods; then makes the edge same-parameter. Writes into <paramref name="aE"/>.
    /// </summary>
    public static void MakePCurve(in TopoDS_Edge aE, in TopoDS_Face aF1, in TopoDS_Face aF2, in IntTools_Curve aIC, bool bPC1, bool bPC2,
                                  IntTools_Context? theContext = null)
    {
        int i;
        double aTolE;
        Geom2d_Curve? aC2D, aC2DA, aC2Dx1;
        TopoDS_Face aFFWD;
        BRep_Builder aBB = new();
        bool bPC;
        //
        aTolE = BRep_Tool.Tolerance(aE);
        //
        Geom_Curve aC3DE = BRep_Tool.Curve(aE, out double aT1, out double aT2)!;
        Geom_TrimmedCurve aC3DETrim = new(aC3DE, aT1, aT2);
        //
        for (i = 0; i < 2; ++i)
        {
            bPC = i == 0 ? bPC1 : bPC2;
            if (!bPC)
            {
                continue;
            }
            //
            if (i == 0)
            {
                aFFWD = aF1;
                aC2Dx1 = aIC.FirstCurve2d();
            }
            else
            {
                aFFWD = aF2;
                aC2Dx1 = aIC.SecondCurve2d();
            }
            //
            aFFWD.Orientation(TopAbs_FORWARD);
            //
            aC2D = aC2Dx1;
            if (aC2D is null)
            {
                BOPTools_AlgoTools2D.BuildPCurveForEdgeOnFace(aE, aFFWD, theContext);
                BOPTools_AlgoTools2D.CurveOnSurface(aE, aFFWD, out aC2D, out _, out _, out _, theContext);
            }
            //
            if (aC3DE.IsPeriodic())
            {
                BOPTools_AlgoTools2D.AdjustPCurveOnFace(aFFWD, aT1, aT2, aC2D!, out aC2DA, theContext);
            }
            else
            {
                BOPTools_AlgoTools2D.AdjustPCurveOnFace(aFFWD, aC3DETrim, aC2D!, out aC2DA, theContext);
            }
            //
            aBB.UpdateEdge(aE, aC2DA, aFFWD, aTolE);
            // BRepLib::SameParameter(aE);
        }
        BRepLib.SameParameter(aE);
    }

    /// <summary>
    /// Makes the section edge of the intersection curve between the vertices (<see cref="MakeSectEdge"/>) with the
    /// tolerance <paramref name="theTolR3D"/>, raising the vertices' tolerances to it first. Writes into the vertices.
    /// </summary>
    public static void MakeEdge(in IntTools_Curve theIC, in TopoDS_Vertex theV1, double theT1, in TopoDS_Vertex theV2, double theT2,
                                double theTolR3D, out TopoDS_Edge theE)
    {
        BRep_Builder aBB = new();
        double aNeedTol = theTolR3D + DTolerance();
        //
        aBB.UpdateVertex(theV1, aNeedTol);
        aBB.UpdateVertex(theV2, aNeedTol);
        //
        MakeSectEdge(theIC, theV1, theT1, theV2, theT2, out theE);
        //
        aBB.UpdateEdge(theE, theTolR3D);
    }

    /// <summary>0 if the point is within the vertex's tolerance plus <paramref name="aTolP2"/> (and Confusion), else 1.</summary>
    public static int ComputeVV(in TopoDS_Vertex aV1, in Point3D aP2, double aTolP2)
    {
        double aTolV1, aTolSum, aTolSum2, aD2;
        Point3D aP1;
        //
        aTolV1 = BRep_Tool.Tolerance(aV1);

        aTolSum = aTolV1 + aTolP2 + global::Kerf.Engine.Foundation.Precision.Confusion();
        aTolSum2 = aTolSum * aTolSum;
        //
        aP1 = BRep_Tool.Pnt(aV1);
        //
        aD2 = aP1.SquareDistance(aP2);
        if (aD2 > aTolSum2)
        {
            return 1;
        }
        return 0;
    }

    /// <summary>0 if the vertices are within the sum of their tolerances and the fuzzy value, else 1.</summary>
    public static int ComputeVV(in TopoDS_Vertex aV1, in TopoDS_Vertex aV2) => ComputeVV(aV1, aV2, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <inheritdoc cref="ComputeVV(in TopoDS_Vertex, in TopoDS_Vertex)"/>
    public static int ComputeVV(in TopoDS_Vertex aV1, in TopoDS_Vertex aV2, double aFuzz)
    {
        double aTolV1, aTolV2, aTolSum, aTolSum2, aD2;
        Point3D aP1, aP2;
        double aFuzz1 = (aFuzz > global::Kerf.Engine.Foundation.Precision.Confusion() ? aFuzz : global::Kerf.Engine.Foundation.Precision.Confusion());
        //
        aTolV1 = BRep_Tool.Tolerance(aV1);
        aTolV2 = BRep_Tool.Tolerance(aV2);
        aTolSum = aTolV1 + aTolV2 + aFuzz1;
        aTolSum2 = aTolSum * aTolSum;
        //
        aP1 = BRep_Tool.Pnt(aV1);
        aP2 = BRep_Tool.Pnt(aV2);
        //
        aD2 = aP1.SquareDistance(aP2);
        if (aD2 > aTolSum2)
        {
            return 1;
        }
        return 0;
    }

    /// <summary>
    /// A vertex covering the vertices of the list: the vertex itself for one, a new bounding vertex
    /// (<see cref="BRepLib.BoundingVertex"/>) for several; an empty list leaves <paramref name="aVnew"/> unchanged.
    /// </summary>
    public static void MakeVertex(NCollection_List<TopoDS_Shape> aLV, ref TopoDS_Vertex aVnew)
    {
        int aNb = aLV.Extent();
        if (aNb == 1)
        {
            aVnew = TopoDS.Vertex(aLV.First());
        }
        else if (aNb > 1)
        {
            double aNTol = 0.0;
            Point3D aNC = new();
            BRepLib.BoundingVertex(aLV, ref aNC, ref aNTol);
            BRep_Builder aBB = new();
            aBB.MakeVertex(ref aVnew, aNC, aNTol);
        }
    }

    /// <summary>Finds in the face the edge same as <paramref name="theE1"/> (any orientation), through its wires.</summary>
    public static bool GetEdgeOnFace(in TopoDS_Edge theE1, in TopoDS_Face theF2, ref TopoDS_Edge theE2)
    {
        bool bFound;
        TopoDS_Iterator aItF = new(), aItW = new();
        //
        bFound = false;
        //
        aItF.Initialize(theF2);
        for (; aItF.More(); aItF.Next())
        {
            TopoDS_Shape aW = aItF.Value();
            aItW.Initialize(aW);
            for (; aItW.More(); aItW.Next())
            {
                TopoDS_Shape aE = aItW.Value();
                if (aE.IsSame(theE1))
                {
                    theE2 = TopoDS.Edge(aE);
                    bFound = !bFound;
                    return bFound;
                }
            }
        }
        return bFound;
    }

    // Pairs the faces of thLF sharing theE (each with the face off it); false if the edge isn't on one of them.
    private static bool FindFacePairs(in TopoDS_Edge theE, NCollection_List<TopoDS_Shape> thLF,
                                      NCollection_List<BOPTools_CoupleOfShape> theLCFF, IntTools_Context theContext)
    {
        bool bFound;
        int i, aNbCEF;
        TopAbs_Orientation aOr, aOrC = TopAbs_FORWARD;
        NCollection_Map<TopoDS_Shape> aMFP = new(TopTools_ShapeMapHasher.Instance);
        TopoDS_Face aF1 = new(), aF2 = new();
        TopoDS_Edge aEL = new(), aE1 = new();
        NCollection_List<TopoDS_Shape>.Iterator aItLF = new();
        BOPTools_CoupleOfShape aCEF = new(), aCFF = new();
        NCollection_List<BOPTools_CoupleOfShape> aLCEF = new(), aLCEFx = new();
        NCollection_List<BOPTools_CoupleOfShape>.Iterator aIt = new();
        //
        bFound = true;
        //
        // Preface aLCEF
        aItLF.Initialize(thLF);
        for (; aItLF.More(); aItLF.Next())
        {
            TopoDS_Face aFL = TopoDS.Face(aItLF.Value());
            //
            bFound = GetEdgeOnFace(theE, aFL, ref aEL);
            if (!bFound)
            {
                return bFound; // it can not be so
            }
            //
            aCEF.SetShape1(aEL);
            aCEF.SetShape2(aFL);
            aLCEF.Append(aCEF);
        }
        //
        aNbCEF = aLCEF.Extent();
        while (aNbCEF != 0)
        {
            //
            // aLCEFx
            aLCEFx.Clear();
            aIt.Initialize(aLCEF);
            for (i = 0; aIt.More(); aIt.Next(), ++i)
            {
                BOPTools_CoupleOfShape aCSx = aIt.Value();
                TopoDS_Shape aEx = aCSx.Shape1();
                TopoDS_Shape aFx = aCSx.Shape2();
                //
                aOr = aEx.Orientation();
                //
                if (i == 0)
                {
                    aOrC = TopAbs.Reverse(aOr);
                    aE1 = TopoDS.Edge(aEx);
                    aF1 = TopoDS.Face(aFx);
                    aMFP.Add(aFx);
                    continue;
                }
                //
                if (aOr == aOrC)
                {
                    aLCEFx.Append(aCSx);
                    aMFP.Add(aFx);
                }
            }
            //
            // F2
            GetFaceOff(aE1, aF1, aLCEFx, ref aF2, theContext);
            //
            aCFF.SetShape1(aF1);
            aCFF.SetShape2(aF2);
            theLCFF.Append(aCFF);
            //
            aMFP.Add(aF1);
            aMFP.Add(aF2);
            //
            // refine aLCEF
            aLCEFx.Clear();
            aLCEFx.Assign(aLCEF);
            aLCEF.Clear();
            aIt.Initialize(aLCEFx);
            for (; aIt.More(); aIt.Next())
            {
                BOPTools_CoupleOfShape aCSx = aIt.Value();
                TopoDS_Shape aFx = aCSx.Shape2();
                if (!aMFP.Contains(aFx))
                {
                    aLCEF.Append(aCSx);
                }
            }
            //
            aNbCEF = aLCEF.Extent();
        } // while(aNbCEF) {
        //
        return bFound;
    }

    // The angle from theD1 to theD2, signed by the reference direction (a pseudo-angle: pi/2 (1 - cos)).
    private static double AngleWithRef(in Direction3D theD1, in Direction3D theD2, in Direction3D theDRef)
    {
        double aCosinus, aSinus, aBeta, aHalfPI, aScPr;
        Xyz aXYZ;
        //
        aHalfPI = 0.5 * M_PI;
        //
        Xyz aXYZ1 = theD1.XYZ();
        Xyz aXYZ2 = theD2.XYZ();
        aXYZ = aXYZ1.Crossed(aXYZ2);
        aSinus = aXYZ.Modulus();
        aCosinus = theD1 * theD2;
        //
        aBeta = 0.0;
        if (aSinus >= 0.0)
        {
            aBeta = aHalfPI * (1.0 - aCosinus);
        }
        else
        {
            aBeta = 2.0 * M_PI - aHalfPI * (3.0 + aCosinus);
        }
        //
        aScPr = aXYZ.Dot(theDRef.XYZ());
        if (aScPr < 0.0)
        {
            aBeta = -aBeta;
        }
        return aBeta;
    }

    /// <summary>
    /// True if the part of the edge in the range (shrunk by 0.75% at each end) lies in or on the face: its two end
    /// points and its intermediate point project in or on the face, the intermediate one within the tolerances.
    /// </summary>
    public static bool IsBlockInOnFace(in IntTools_Range aShrR, in TopoDS_Face aF, in TopoDS_Edge aE1, IntTools_Context aContext)
    {
        bool bFlag;
        double ULD, VLD;
        Point2D aP2D = new();
        //
        aShrR.Range(out double f1, out double l1);
        double dt = 0.0075, k; // dt=0.001,  k;
        k = dt * (l1 - f1);
        f1 = f1 + k;
        l1 = l1 - k;
        //
        // Treatment P11
        PointOnEdge(aE1, f1, out Point3D aP11);
        //
        GeomAPI_ProjectPointOnSurf aProjector = aContext.ProjPS(aF);
        aProjector.Perform(aP11);
        //
        bFlag = aProjector.IsDone();
        if (!bFlag)
        {
            return bFlag;
        }

        aProjector.LowerDistanceParameters(out ULD, out VLD);
        aP2D.SetCoord(ULD, VLD);
        //
        bFlag = aContext.IsPointInOnFace(aF, aP2D);
        //
        if (!bFlag)
        {
            return bFlag;
        }
        //
        // Treatment P12
        PointOnEdge(aE1, l1, out Point3D aP12);
        //
        aProjector.Perform(aP12);
        //
        bFlag = aProjector.IsDone();
        if (!bFlag)
        {
            return bFlag;
        }

        aProjector.LowerDistanceParameters(out ULD, out VLD);
        aP2D.SetCoord(ULD, VLD);
        //
        bFlag = aContext.IsPointInOnFace(aF, aP2D);
        //
        if (!bFlag)
        {
            return bFlag;
        }
        //

        // Treatment intermediate
        double m1, aTolF, aTolE, aTol, aDist;
        m1 = IntTools_Tools.IntermediatePoint(f1, l1);
        PointOnEdge(aE1, m1, out aP12);
        //
        aProjector.Perform(aP12);
        //
        bFlag = aProjector.IsDone();
        if (!bFlag)
        {
            return bFlag;
        }
        //
        aTolE = BRep_Tool.Tolerance(aE1);
        aTolF = BRep_Tool.Tolerance(aF);
        aTol = aTolE + aTolF;
        aDist = aProjector.LowerDistance();
        if (aDist > aTol)
        {
            return false;
        }

        aProjector.LowerDistanceParameters(out ULD, out VLD);
        aP2D.SetCoord(ULD, VLD);
        //
        bFlag = aContext.IsPointInOnFace(aF, aP2D);
        //
        if (!bFlag)
        {
            return bFlag;
        }
        return bFlag;
    }

    /// <summary>
    /// True if the edge is a micro edge: degenerated, without geometry, or too small for a shrunk range
    /// (<see cref="IntTools_ShrunkRange"/>), or, if <paramref name="bCheckSplittable"/>, not splittable.
    /// </summary>
    public static bool IsMicroEdge(in TopoDS_Edge aE, IntTools_Context? aCtx, bool bCheckSplittable = true)
    {
        bool bRet;
        double aT1, aT2, aTmp;
        //
        bRet = (BRep_Tool.Degenerated(aE) || !BRep_Tool.IsGeometric(aE));
        if (bRet)
        {
            return bRet;
        }
        //
        _ = BRep_Tool.Curve(aE, out aT1, out aT2);
        TopExp.Vertices(aE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        aT1 = BRep_Tool.Parameter(aV1, aE);
        aT2 = BRep_Tool.Parameter(aV2, aE);
        if (aT2 < aT1)
        {
            aTmp = aT1;
            aT1 = aT2;
            aT2 = aTmp;
        }
        //
        IntTools_ShrunkRange aSR = new();
        aSR.SetContext(aCtx);
        aSR.SetData(aE, aT1, aT2, aV1, aV2);
        aSR.Perform();
        bRet = !aSR.IsDone();
        if (!bRet && bCheckSplittable)
        {
            bRet = !aSR.IsSplittable();
        }
        //
        return bRet;
    }

    // The bi-normal direction aDB of the face at the point aP of the edge (and its normal aDN).
    private static bool GetFaceDir(in TopoDS_Edge aE,
                                   in TopoDS_Face aF,
                                   in Point3D aP,
                                   double aT,
                                   in Direction3D aDTgt,
                                   bool theSmallFaces,
                                   out Direction3D aDN,
                                   out Direction3D aDB,
                                   IntTools_Context theContext,
                                   GeomAPI_ProjectPointOnSurf aProjPL,
                                   double aDt)
    {
        double aTolE;
        Point3D aPx = new();
        //
        BOPTools_AlgoTools3D.GetNormalToFaceOnEdge(aE, aF, aT, out aDN, theContext);
        if (aF.Orientation() == TopAbs_REVERSED)
        {
            aDN.Reverse();
        }
        //
        aTolE = BRep_Tool.Tolerance(aE);
        aDB = aDN ^ aDTgt;
        //
        // do not try to look for the point in the small face by intersecting
        // it with the circle because, most likely, the intersection point will
        // be out of the face
        bool bFound = !theSmallFaces && FindPointInFace(aF, aP, ref aDB, ref aPx, theContext, aProjPL, aDt, aTolE);
        if (!bFound)
        {
            // if the first method did not succeed, try to use hatcher to find the point
            bFound = BOPTools_AlgoTools3D.GetApproxNormalToFaceOnEdge(aE, aF, aT, aDt, ref aPx, ref aDN, theContext);
            aProjPL.Perform(aPx);
            // Standard_ASSERT_RETURN: returns false in release builds.
            if (!aProjPL.IsDone())
            {
                return false;
            }
            aPx = aProjPL.NearestPoint();
            Vector3D aVec = new(aP, aPx);
            aDB.SetXYZ(aVec.XYZ());
        }
        //
        return bFound;
    }

    // Finds a point in the face in the direction aDB: intersects the face with the circle of radius aDt at aP in the
    // plane of aProjPL, iterating projections.
    private static bool FindPointInFace(in TopoDS_Face aF,
                                        in Point3D aP,
                                        ref Direction3D aDB,
                                        ref Point3D aPOut,
                                        IntTools_Context theContext,
                                        GeomAPI_ProjectPointOnSurf aProjPL,
                                        double aDt,
                                        double aTolE)
    {
        int aNbItMax;
        double aDist, aDTol, aPM, anEps;
        bool bRet;
        Point3D aP1 = new(), aPS;
        //
        aDTol = global::Kerf.Engine.Foundation.Precision.Angular();
        aPM = aP.XYZ().Modulus();
        if (aPM > 1000.0)
        {
            aDTol = 5.0e-16 * aPM;
        }
        bRet = false;
        aNbItMax = 15;
        anEps = global::Kerf.Engine.Foundation.Precision.SquareConfusion();
        //
        GeomAPI_ProjectPointOnSurf aProj = theContext.ProjPS(aF);
        //
        aPS = aP;
        aProj.Perform(aPS);
        if (!aProj.IsDone())
        {
            return bRet;
        }
        aPS = aProj.NearestPoint();
        aProjPL.Perform(aPS);
        aPS = aProjPL.NearestPoint();
        //
        aPS.SetXYZ(aPS.XYZ() + 2.0 * aTolE * aDB.XYZ());
        aProj.Perform(aPS);
        if (!aProj.IsDone())
        {
            return bRet;
        }
        aPS = aProj.NearestPoint();
        aProjPL.Perform(aPS);
        aPS = aProjPL.NearestPoint();
        //
        do
        {
            aP1.SetXYZ(aPS.XYZ() + aDt * aDB.XYZ());
            //
            aProj.Perform(aP1);
            if (!aProj.IsDone())
            {
                return bRet;
            }
            aPOut = aProj.NearestPoint();
            aDist = aProj.LowerDistance();
            //
            aProjPL.Perform(aPOut);
            aPOut = aProjPL.NearestPoint();
            //
            Vector3D aV = new(aPS, aPOut);
            if (aV.SquareMagnitude() < anEps)
            {
                return bRet;
            }
            aDB.SetXYZ(aV.XYZ());
        } while (aDist > aDTol && --aNbItMax != 0);
        //
        bRet = aDist < aDTol;
        return bRet;
    }

    // The 3D step for GetFaceDir, and whether some face is too small for it.
    private static double MinStep3D(in TopoDS_Edge theE1,
                                    in TopoDS_Face theF1,
                                    NCollection_List<BOPTools_CoupleOfShape> theLCS,
                                    in Point3D aP,
                                    IntTools_Context theContext,
                                    ref bool theSmallFaces)
    {
        double aDt, aTolE, aTolF, aDtMax, aDtMin;
        //
        // add the current pair of edge/face for checking as well
        BOPTools_CoupleOfShape aCS1 = new();
        aCS1.SetShape1(theE1);
        aCS1.SetShape2(theF1);
        //
        NCollection_List<BOPTools_CoupleOfShape> aLCS = new(theLCS);
        aLCS.Append(aCS1);
        //
        aTolE = BRep_Tool.Tolerance(theE1);
        aDtMax = -1.0;
        aDtMin = 5.0e-6;
        //
        NCollection_List<BOPTools_CoupleOfShape>.Iterator aIt = new(aLCS);
        for (; aIt.More(); aIt.Next())
        {
            BOPTools_CoupleOfShape aCS = aIt.Value();
            TopoDS_Face aF = TopoDS.Face(aCS.Shape2());
            //
            aTolF = BRep_Tool.Tolerance(aF);
            aDt = 2 * (aTolE + aTolF);
            if (aDt > aDtMax)
            {
                aDtMax = aDt;
            }
            //
            // try to compute the minimal 3D step
            BRepAdaptor_Surface aBAS = theContext.SurfaceAdaptor(aF);
            double aR = 0.0;
            SurfaceKind aSType = aBAS.GetType();
            switch (aSType)
            {
                case SurfaceKind.Cylinder:
                    {
                        aR = aBAS.Cylinder().Radius();
                        break;
                    }
                case SurfaceKind.Cone:
                    {
                        Line3D aL = new(aBAS.Cone().Axis());
                        aR = aL.Distance(aP);
                        break;
                    }
                case SurfaceKind.Sphere:
                    {
                        aDtMin = Max(aDtMin, 5.0e-4);
                        aR = aBAS.Sphere().Radius();
                        break;
                    }
                case SurfaceKind.Torus:
                    {
                        aR = aBAS.Torus().MajorRadius();
                        break;
                    }
                default:
                    aDtMin = Max(aDtMin, 5.0e-4);
                    break;
            }
            //
            if (aR > 100.0)
            {
                double d = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();
                aDtMin = Max(aDtMin, Math.Sqrt(d * d + 2 * d * aR));
            }
        }
        //
        if (aDtMax < aDtMin)
        {
            aDtMax = aDtMin;
        }
        //
        // check if the computed 3D step is too big for any of the faces in the list
        aIt.Initialize(aLCS);
        for (; aIt.More(); aIt.Next())
        {
            BOPTools_CoupleOfShape aCS = aIt.Value();
            TopoDS_Face aF = TopoDS.Face(aCS.Shape2());
            //
            BRepAdaptor_Surface aBAS = theContext.SurfaceAdaptor(aF);
            //
            theContext.UVBounds(aF, out double aUMin, out double aUMax, out double aVMin, out double aVMax);
            //
            double aDU = aUMax - aUMin;
            if (aDU > 0.0)
            {
                double aURes = aBAS.UResolution(aDtMax);
                if (2 * aURes > aDU)
                {
                    break;
                }
            }
            //
            double aDV = aVMax - aVMin;
            if (aDV > 0.0)
            {
                double aVRes = aBAS.VResolution(aDtMax);
                if (2 * aVRes > aDV)
                {
                    break;
                }
            }
        }
        //
        theSmallFaces = aIt.More();
        //
        return aDtMax;
    }

    /// <summary>True if the shell has a free edge: a non-degenerated edge in only one forward or reversed face.</summary>
    public static bool IsOpenShell(in TopoDS_Shell aSh)
    {
        bool bRet;
        int i, aNbE, aNbF;
        TopAbs_Orientation aOrF;
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMEF = new(TopTools_ShapeMapHasher.Instance);
        //
        bRet = false;
        //
        TopExp.MapShapesAndAncestors(aSh, TopAbs_EDGE, TopAbs_FACE, aMEF);
        //
        aNbE = aMEF.Extent();
        for (i = 1; i <= aNbE; ++i)
        {
            TopoDS_Edge aE = TopoDS.Edge(aMEF.FindKey(i));
            if (BRep_Tool.Degenerated(aE))
            {
                continue;
            }
            //
            aNbF = 0;
            NCollection_List<TopoDS_Shape> aLF = aMEF.FindFromIndex(i);
            for (NCollection_List<TopoDS_Shape>.Iterator aItLS = new(aLF); aItLS.More(); aItLS.Next())
            {
                TopoDS_Shape aF = aItLS.Value();
                aOrF = aF.Orientation();
                if (aOrF == TopAbs_INTERNAL || aOrF == TopAbs_EXTERNAL)
                {
                    continue;
                }
                ++aNbF;
            }
            //
            if (aNbF == 1)
            {
                bRet = !bRet; // True
                break;
            }
        }
        //
        return bRet;
    }

    /// <summary>True if the solid is inverted: the infinite point classifies inside it.</summary>
    public static bool IsInvertedSolid(in TopoDS_Solid aSolid)
    {
        double aTolS;
        TopAbs_State aState;
        BRepClass3d_SolidClassifier aSC = new(aSolid);
        //
        aTolS = 1.0e-7;
        aSC.PerformInfinitePoint(aTolS);
        aState = aSC.State();
        return aState == TopAbs_IN;
    }
}
