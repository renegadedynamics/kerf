// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

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
    public static void MakeConnexityBlocks(in Shape theS,
                                           ShapeType theConnectionType,
                                           ShapeType theElementType,
                                           NCollection_List<NCollection_List<Shape>> theLCB,
                                           NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theConnectionMap)
    {
        // Map shapes to find connected elements
        Exploration.MapShapesAndAncestors(theS, theConnectionType, theElementType, theConnectionMap);
        // Fence map
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);

        Explorer aExp = new(theS, theElementType);
        for (; aExp.More(); aExp.Next())
        {
            Shape aS = aExp.Current();
            if (!aMFence.Add(aS))
            {
                continue;
            }
            // The block
            NCollection_List<Shape> aLBlock = new();
            // Start the block
            aLBlock.Append(aS);
            // Look for connected parts
            NCollection_List<Shape>.Iterator aItB = new(aLBlock);
            for (; aItB.More(); aItB.Next())
            {
                Shape aS1 = aItB.Value();
                Explorer aExpSS = new(aS1, theConnectionType);
                for (; aExpSS.More(); aExpSS.Next())
                {
                    Shape aSubS = aExpSS.Current();
                    NCollection_List<Shape> aLS = theConnectionMap.FindFromKey(aSubS);
                    NCollection_List<Shape>.Iterator aItLS = new(aLS);
                    for (; aItLS.More(); aItLS.Next())
                    {
                        Shape aS2 = aItLS.Value();
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
    public static void MakeConnexityBlocks(in Shape theS,
                                           ShapeType theConnectionType,
                                           ShapeType theElementType,
                                           NCollection_List<Shape> theLCB)
    {
        NCollection_List<NCollection_List<Shape>> aLBlocks = new();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aCMap = new(ShapeHasher.Instance);
        MakeConnexityBlocks(theS, theConnectionType, theElementType, aLBlocks, aCMap);

        // Make compound from each block
        NCollection_List<NCollection_List<Shape>>.Iterator aItB = new(aLBlocks);
        for (; aItB.More(); aItB.Next())
        {
            NCollection_List<Shape> aLB = aItB.Value();

            Compound aBlock = new();
            new BrepBuilder().MakeCompound(ref aBlock);
            for (NCollection_List<Shape>.Iterator it = new(aLB); it.More(); it.Next())
            {
                new BrepBuilder().Add(aBlock, it.Value());
            }

            theLCB.Append(aBlock);
        }
    }

    /// <summary>
    /// Makes the connexity blocks of the shapes of <paramref name="theLS"/> connected through sub-shapes of
    /// <paramref name="theConnectionType"/>, and tells whether each is regular (every connection shared by exactly two
    /// elements, no element given twice).
    /// </summary>
    public static void MakeConnexityBlocks(NCollection_List<Shape> theLS,
                                           ShapeType theConnectionType,
                                           ShapeType theElementType,
                                           NCollection_List<BOPTools_ConnexityBlock> theLCB)
    {
        BrepBuilder aBB = new();
        // Make connexity blocks from start elements
        Compound aCStart = new();
        aBB.MakeCompound(ref aCStart);

        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance), aMNRegular = new(ShapeHasher.Instance);

        NCollection_List<Shape>.Iterator aItL = new(theLS);
        for (; aItL.More(); aItL.Next())
        {
            Shape aS = aItL.Value();
            if (aMFence.Add(aS))
            {
                aBB.Add(aCStart, aS);
            }
            else
            {
                aMNRegular.Add(aS);
            }
        }

        NCollection_List<NCollection_List<Shape>> aLCB = new();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aCMap = new(ShapeHasher.Instance);
        MakeConnexityBlocks(aCStart, theConnectionType, theElementType, aLCB, aCMap);

        // Save the blocks and check their regularity
        NCollection_List<NCollection_List<Shape>>.Iterator aItB = new(aLCB);
        for (; aItB.More(); aItB.Next())
        {
            NCollection_List<Shape> aBlock = aItB.Value();

            BOPTools_ConnexityBlock aCB = new();
            NCollection_List<Shape> aLCS = aCB.ChangeShapes();

            bool bRegular = true;
            for (NCollection_List<Shape>.Iterator it = new(aBlock); it.More(); it.Next())
            {
                Shape aS = it.Value();
                if (aMNRegular.Contains(aS))
                {
                    bRegular = false;
                    aS.Orientation(Forward);
                    aLCS.Append(aS);
                    aS.Orientation(Reversed);
                    aLCS.Append(aS);
                }
                else
                {
                    aLCS.Append(aS);
                    if (bRegular)
                    {
                        // Check if there are no multi-connected shapes
                        for (Explorer ex = new(aS, theConnectionType); ex.More() && bRegular; ex.Next())
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
    public static void OrientEdgesOnWire(ref Shape theWire)
    {
        // make vertex-edges connexity map
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aVEMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(theWire, ShapeType.Vertex, ShapeType.Edge, aVEMap);
        //
        if (aVEMap.IsEmpty())
        {
            return;
        }
        //
        BrepBuilder aBB = new();
        // new wire
        Wire aWire = new();
        aBB.MakeWire(ref aWire);
        // fence map (OCCT's default hasher: orientation included, decision 6433)
        NCollection_Map<Shape> aMFence = new(EqualityComparer<Shape>.Default);
        //
        Iterator aIt = new(theWire);
        for (; aIt.More(); aIt.Next())
        {
            Edge aEC = ShapeCasts.Edge(aIt.Value());
            if (!aMFence.Add(aEC))
            {
                continue;
            }
            //
            // add edge to a wire as it is
            aBB.Add(aWire, aEC);
            //
            Exploration.Vertices(aEC, out Vertex aV1, out Vertex aV2, true);
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
                Shape aVC = i == 0 ? aV1 : aV2;
                //
                for (; ; )
                {
                    NCollection_List<Shape> aLE = aVEMap.FindFromKey(aVC);
                    if (aLE.Extent() != 2)
                    {
                        // free vertex or multi-connexity, go to the next edge
                        break;
                    }
                    //
                    bool bStop = true;
                    //
                    NCollection_List<Shape>.Iterator aItLE = new(aLE);
                    for (; aItLE.More(); aItLE.Next())
                    {
                        Edge aEN = ShapeCasts.Edge(aItLE.Value());
                        if (aMFence.Contains(aEN))
                        {
                            continue;
                        }
                        //
                        Exploration.Vertices(aEN, out Vertex aVN1, out Vertex aVN2, true);
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
    public static void OrientFacesOnShell(ref Shape aShell)
    {
        bool bIsProcessed1, bIsProcessed2;
        int i, aNbE, aNbF, j;
        Orientation anOrE1, anOrE2;
        Face aF1x, aF2x;
        Shape aShellNew = new();
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aEFMap = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> aProcessedFaces = new(ShapeHasher.Instance);
        BrepBuilder aBB = new();
        //
        MakeContainer(ShapeType.Shell, ref aShellNew);
        //
        Exploration.MapShapesAndAncestors(aShell, ShapeType.Edge, ShapeType.Face, aEFMap);
        aNbE = aEFMap.Extent();
        //
        // One seam edge  in aEFMap contains  2 equivalent faces.
        for (i = 1; i <= aNbE; ++i)
        {
            NCollection_List<Shape> aLF = aEFMap.ChangeFromIndex(i);
            aNbF = aLF.Extent();
            if (aNbF > 1)
            {
                NCollection_List<Shape> aLFTmp = new();
                NCollection_IndexedMap<Shape> aFM = new(ShapeHasher.Instance);
                //
                NCollection_List<Shape>.Iterator anIt = new(aLF);
                for (; anIt.More(); anIt.Next())
                {
                    Shape aF = anIt.Value();
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
            Edge aE = ShapeCasts.Edge(aEFMap.FindKey(i));
            if (Tool.Degenerated(aE))
            {
                continue;
            }
            //
            NCollection_List<Shape> aLF = aEFMap.FindFromIndex(i);
            aNbF = aLF.Extent();
            if (aNbF != 2)
            {
                continue;
            }
            //
            // OCCT holds references into the list and reverses its items in place.
            ref Shape aF1 = ref aLF.ChangeFirst();
            ref Shape aF2 = ref aLF.ChangeLast();
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
            aF1x = ShapeCasts.Face(aF1);
            if (bIsProcessed1)
            {
                j = aProcessedFaces.FindIndex(aF1);
                aF1x = ShapeCasts.Face(aProcessedFaces.FindKey(j));
            }
            //
            aF2x = ShapeCasts.Face(aF2);
            if (bIsProcessed2)
            {
                j = aProcessedFaces.FindIndex(aF2);
                aF2x = ShapeCasts.Face(aProcessedFaces.FindKey(j));
            }
            //
            anOrE1 = Orientation(aE, aF1x);
            anOrE2 = Orientation(aE, aF2x);
            //
            if (bIsProcessed1 && !bIsProcessed2)
            {
                if (anOrE1 == anOrE2)
                {
                    if (!Tool.IsClosed(aE, ShapeCasts.Face(aF1)) && !Tool.IsClosed(aE, ShapeCasts.Face(aF2)))
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
                    if (!Tool.IsClosed(aE, ShapeCasts.Face(aF1)) && !Tool.IsClosed(aE, ShapeCasts.Face(aF2)))
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
            Edge aE = ShapeCasts.Edge(aEFMap.FindKey(i));
            if (Tool.Degenerated(aE))
            {
                continue;
            }
            //
            NCollection_List<Shape> aLF = aEFMap.FindFromIndex(i);
            aNbF = aLF.Extent();
            if (aNbF != 2)
            {
                NCollection_List<Shape>.Iterator anIt = new(aLF);
                for (; anIt.More(); anIt.Next())
                {
                    Shape aF = anIt.Value();
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
    private static Orientation Orientation(in Edge anE, in Face aF)
    {
        Orientation anOr = Internal;

        Explorer anExp = new();
        anExp.Init(aF, ShapeType.Edge);
        for (; anExp.More(); anExp.Next())
        {
            Shape anEF1 = anExp.Current();
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
    public static void MakeConnexityBlock(NCollection_List<Shape> theLFIn,
                                          NCollection_IndexedMap<Shape> theMEAvoid,
                                          NCollection_List<Shape> theLCB)
    {
        int aNbF, aNbAdd1, aNbAdd, i;
        Explorer aExp = new();
        NCollection_List<Shape>.Iterator aIt = new();
        //
        NCollection_IndexedMap<Shape> aMCB = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> aMAdd = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> aMAdd1 = new(ShapeHasher.Instance);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMEF = new(ShapeHasher.Instance);
        //
        // 1. aMEF
        aIt.Initialize(theLFIn);
        for (; aIt.More(); aIt.Next())
        {
            Shape aF = aIt.Value();
            Exploration.MapShapesAndAncestors(aF, ShapeType.Edge, ShapeType.Face, aMEF);
        }
        //
        // 2. aMCB
        Shape aF1 = theLFIn.First();
        aMAdd.Add(aF1);
        //
        for (; ; )
        {
            aMAdd1.Clear();
            aNbAdd = aMAdd.Extent();
            for (i = 1; i <= aNbAdd; ++i)
            {
                Shape aF = aMAdd[i];
                //
                // aMAdd1.Clear();
                aExp.Init(aF, ShapeType.Edge);
                for (; aExp.More(); aExp.Next())
                {
                    Shape aE = aExp.Current();
                    if (theMEAvoid.Contains(aE))
                    {
                        continue;
                    }
                    //
                    NCollection_List<Shape> aLF = aMEF.FindFromKey(aE);
                    aIt.Initialize(aLF);
                    for (; aIt.More(); aIt.Next())
                    {
                        Shape aFx = aIt.Value();
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
                Shape aFAdd = aMAdd1[i];
                aMAdd.Add(aFAdd);
            }
            //
        } // while(1) {

        //
        aNbF = aMCB.Extent();
        for (i = 1; i <= aNbF; ++i)
        {
            Shape aF = aMCB[i];
            theLCB.Append(aF);
        }
    }

    /// <summary>
    /// The state of the shape relative to the solid, from one point of it: a vertex's point, an edge's intermediate
    /// point, a face's point (see the face form of <see cref="ComputeState(in Face, in Solid, double,
    /// NCollection_IndexedMap{Shape}, IntTools_Context)"/>). UNKNOWN for other types.
    /// </summary>
    public static Containment ComputeStateByOnePoint(in Shape theS, in Solid theRef, double theTol,
                                                      IntTools_Context theContext)
    {
        Containment aState = Unknown;
        ShapeType aType = theS.ShapeType();

        switch (aType)
        {
            case ShapeType.Vertex:
                aState = ComputeState(ShapeCasts.Vertex(theS), theRef, theTol, theContext);
                break;
            case ShapeType.Edge:
                aState = ComputeState(ShapeCasts.Edge(theS), theRef, theTol, theContext);
                break;
            case ShapeType.Face:
                {
                    NCollection_IndexedMap<Shape> aBounds = new(ShapeHasher.Instance);
                    Exploration.MapShapes(theRef, ShapeType.Edge, aBounds);
                    aState = ComputeState(ShapeCasts.Face(theS), theRef, theTol, aBounds, theContext);
                    break;
                }
            default:
                {
                    Iterator it = new(theS);
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
    public static Containment ComputeState(in Face theF, in Solid theRef, double theTol,
                                            NCollection_IndexedMap<Shape> theBounds, IntTools_Context theContext)
    {
        Containment aState = Unknown;

        // Try to find the edge on the face which does not
        // belong to the solid and classify the middle point of that
        // edge relatively solid.
        Explorer aExp = new(theF, ShapeType.Edge);
        for (; aExp.More(); aExp.Next())
        {
            Edge aSE = ShapeCasts.Edge(aExp.Current());
            if (Tool.Degenerated(aSE))
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
            aExp.Init(theF, ShapeType.Edge);
            for (; aExp.More() && iErr != 0; aExp.Next())
            {
                Edge aSE = ShapeCasts.Edge(aExp.Current());
                if (Tool.Degenerated(aSE))
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
    public static Containment ComputeState(in Vertex theV, in Solid theRef, double theTol, IntTools_Context theContext)
    {
        Containment aState;
        Point3D aP3D;
        //
        aP3D = Tool.Pnt(theV);
        aState = ComputeState(aP3D, theRef, theTol, theContext);
        return aState;
    }

    /// <summary>
    /// The state of the edge's intermediate point (or of its vertex, for an edge without a 3D curve) relative to the
    /// solid.
    /// </summary>
    public static Containment ComputeState(in Edge theE, in Solid theRef, double theTol, IntTools_Context theContext)
    {
        double aT = 0.0;
        Containment aState;
        Curve3D? aC3D;
        Point3D aP3D;
        //
        aC3D = Tool.Curve(theE, out double aT1, out double aT2);
        //
        if (aC3D is null)
        {
            // it means that we are in degenerated edge
            Vertex aV = Exploration.FirstVertex(theE);
            if (aV.IsNull)
            {
                return Unknown;
            }
            aP3D = Tool.Pnt(aV);
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
    public static Containment ComputeState(in Point3D theP, in Solid theRef, double theTol, IntTools_Context theContext)
    {
        Containment aState;
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
    public static bool IsInternalFace(in Face theFace,
                                      in Solid theSolid,
                                      NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theMEF,
                                      double theTol,
                                      IntTools_Context theContext)
    {
        bool bDegenerated;
        Orientation aOr;
        Edge aE1 = new();
        Explorer aExp = new();
        //
        // For all invoked functions: [::IsInternalFace(...)]
        // the returned value iRet means:
        // iRet=0;  - state is not IN
        // iRet=1;  - state is IN
        // iRet=2;  - state can not be found by the method of angles
        int iRet = 0;
        // 1 Try to find an edge from theFace in theMEF
        aExp.Init(theFace, ShapeType.Edge);
        for (; aExp.More(); aExp.Next())
        {
            Edge aE = ShapeCasts.Edge(aExp.Current());
            if (!theMEF.Contains(aE))
            {
                continue;
            }
            //
            aOr = aE.Orientation();
            if (aOr == Internal)
            {
                continue;
            }
            bDegenerated = Tool.Degenerated(aE);
            if (bDegenerated)
            {
                continue;
            }
            // aE
            NCollection_List<Shape> aLF = theMEF.ChangeFromKey(aE);
            int aNbF = aLF.Extent();
            if (aNbF == 1)
            {
                // aE is internal edge on aLF.First()
                Face aF1 = ShapeCasts.Face(aLF.First());
                GetEdgeOnFace(aE, aF1, ref aE1);
                if (aE1.Orientation() != Internal)
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
                Face aF1 = ShapeCasts.Face(aLF.First());
                Face aF2 = ShapeCasts.Face(aLF.Last());
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
        Containment aState;
        NCollection_IndexedMap<Shape> aBounds = new(ShapeHasher.Instance);
        //
        Exploration.MapShapes(theSolid, ShapeType.Edge, aBounds);
        //
        aState = ComputeState(theFace, theSolid, theTol, aBounds, theContext);
        return aState == In;
    }

    /// <summary>
    /// Whether the face is inside the faces <paramref name="theLF"/> sharing <paramref name="theEdge"/>: 1 if IN, 0 if
    /// not, 2 if the angles can't tell.
    /// </summary>
    public static int IsInternalFace(in Face theFace, in Edge theEdge, NCollection_List<Shape> theLF,
                                     IntTools_Context theContext)
    {
        int aNbF, iRet;
        //
        iRet = 0;
        //
        aNbF = theLF.Extent();
        if (aNbF == 2)
        {
            Face aF1 = ShapeCasts.Face(theLF.First());
            Face aF2 = ShapeCasts.Face(theLF.Last());
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
                Face aF1 = ShapeCasts.Face(aCSFF.Shape1());
                Face aF2 = ShapeCasts.Face(aCSFF.Shape2());
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
    public static int IsInternalFace(in Face theFace, in Edge theEdge, in Face theFace1, in Face theFace2,
                                     IntTools_Context theContext)
    {
        Edge aE1 = new(), aE2 = new();
        Face aFOff = new();
        NCollection_List<BOPTools_CoupleOfShape> theLCSOff = new();
        BOPTools_CoupleOfShape aCS1 = new(), aCS2 = new();
        //
        GetEdgeOnFace(theEdge, theFace1, ref aE1);
        if (aE1.Orientation() == Internal)
        {
            aE2 = aE1;
            aE1.Orientation(Forward);
            aE2.Orientation(Reversed);
        }
        else if (theFace1 == theFace2)
        {
            aE2 = aE1;
            aE1.Orientation(Forward);
            aE2.Orientation(Reversed);
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
    public static bool GetFaceOff(in Edge theE1, in Face theF1, NCollection_List<BOPTools_CoupleOfShape> theLCSOff,
                                  ref Face theFOff, IntTools_Context theContext)
    {
        bool bRet, bIsComputed;
        double aT, aAngle, aTwoPI, aAngleMin, aDt3D;
        Point3D aPx;
        Direction3D aDBF, aDBF2, aDTF;
        Vector3D aVTgt = new();
        Orientation aOr;
        Curve3D aC3D;
        PlaneSurface aPL;
        NCollection_List<BOPTools_CoupleOfShape>.Iterator aIt = new();
        GeomAPI_ProjectPointOnSurf aProjPL = new();
        //
        aAngleMin = 100.0;
        aTwoPI = M_PI + M_PI;
        aC3D = Tool.Curve(theE1, out double aT1, out double aT2)!;
        aT = BOPTools_AlgoTools2D.IntermediatePoint(aT1, aT2);
        aC3D.D0(aT, out aPx);
        //
        BOPTools_AlgoTools2D.EdgeTangent(theE1, aT, ref aVTgt);
        Direction3D aDTgt = new(aVTgt), aDTgt2;
        aOr = theE1.Orientation();
        //
        aPL = new PlaneSurface(aPx, aDTgt);
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
            Edge aE2 = ShapeCasts.Edge(aCS.Shape1());
            Face aF2 = ShapeCasts.Face(aCS.Shape2());
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
    public static bool GetEdgeOff(in Edge theE1, in Face theF2, ref Edge theE2)
    {
        bool bFound;
        Orientation aOr1, aOr1C, aOr2;
        Explorer anExp = new();
        //
        bFound = false;
        aOr1 = theE1.Orientation();
        aOr1C = global::Kerf.Engine.Topology.TopAbs.Reverse(aOr1);
        //
        anExp.Init(theF2, ShapeType.Edge);
        for (; anExp.More(); anExp.Next())
        {
            Edge aEF2 = ShapeCasts.Edge(anExp.Current());
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
    public static bool AreFacesSameDomain(in Face theF1, in Face theF2, IntTools_Context theContext) =>
        AreFacesSameDomain(theF1, theF2, theContext, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <inheritdoc cref="AreFacesSameDomain(in Face, in Face, IntTools_Context)"/>
    public static bool AreFacesSameDomain(in Face theF1, in Face theF2, IntTools_Context theContext, double theFuzz)
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
        double aTolF1 = Tool.Tolerance(theF1), aTolF2 = Tool.Tolerance(theF2);

        // Find maximal tolerance of edges.
        // The faces should have the same boundaries, thus
        // it does not matter which face to explore.
        {
            double aTolEMax = -1.0;
            Explorer anExpE = new(theF1, ShapeType.Edge);
            for (; anExpE.More(); anExpE.Next())
            {
                Edge aE = ShapeCasts.Edge(anExpE.Current());
                if (!Tool.Degenerated(aE))
                {
                    double aTolE = Tool.Tolerance(aE);
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
    public static int Sense(in Face theF1, in Face theF2, IntTools_Context theContext)
    {
        int iSense = 0;
        Direction3D aDNF1, aDNF2;
        Edge aE1 = new(), aE2 = new();
        Explorer aExp = new();
        //
        aExp.Init(theF1, ShapeType.Edge);
        for (; aExp.More(); aExp.Next())
        {
            aE1 = ShapeCasts.Edge(aExp.Current());
            if (!Tool.Degenerated(aE1))
            {
                if (!Tool.IsClosed(aE1, theF1))
                {
                    break;
                }
            }
        }
        //
        aExp.Init(theF2, ShapeType.Edge);
        for (; aExp.More(); aExp.Next())
        {
            aE2 = ShapeCasts.Edge(aExp.Current());
            if (!Tool.Degenerated(aE2))
            {
                if (!Tool.IsClosed(aE2, theF2))
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
    public static bool IsSplitToReverse(in Shape theSp, in Shape theSr, IntTools_Context? theContext) =>
        IsSplitToReverse(theSp, theSr, theContext, out _);

    /// <summary>
    /// As <see cref="IsSplitToReverse(in Shape, in Shape, IntTools_Context?)"/>, with the error status
    /// (0 if fine, 100 for an unsupported type; the face and edge forms give their own codes). OCCT's
    /// <c>int* theError</c>.
    /// </summary>
    public static bool IsSplitToReverse(in Shape theSp, in Shape theSr, IntTools_Context? theContext, out int theError)
    {
        bool bRet;
        ShapeType aType;
        //
        bRet = false;
        theError = 0; // the edge and face forms always set it
        //
        aType = theSp.ShapeType();
        switch (aType)
        {
            case ShapeType.Edge:
                {
                    Edge aESp = ShapeCasts.Edge(theSp);
                    Edge aESr = ShapeCasts.Edge(theSr);
                    bRet = IsSplitToReverse(aESp, aESr, theContext, out theError);
                }
                break;
            //
            case ShapeType.Face:
                {
                    Face aFSp = ShapeCasts.Face(theSp);
                    Face aFSr = ShapeCasts.Face(theSr);
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
    /// As <see cref="IsSplitToReverse(in Shape, in Shape, IntTools_Context?)"/>; when the check fails,
    /// adds a <see cref="BOPAlgo_AlertUnableToOrientTheShape"/> warning (with both shapes) to the report.
    /// </summary>
    public static bool IsSplitToReverseWithWarn(in Shape theSplit, in Shape theShape, IntTools_Context? theContext,
                                                Report? theReport = null)
    {
        bool isToReverse = IsSplitToReverse(theSplit, theShape, theContext, out int anErr);
        if (anErr != 0 && theReport is not null)
        {
            // The error occurred during the check.
            // Add warning to the report, storing the shapes into the warning.
            Compound aWC = new();
            new BrepBuilder().MakeCompound(ref aWC);
            new BrepBuilder().Add(aWC, theSplit);
            new BrepBuilder().Add(aWC, theShape);
            theReport.AddAlert(Warning, new BOPAlgo_AlertUnableToOrientTheShape(aWC));
        }
        return isToReverse;
    }

    /// <summary>
    /// True if the split face must be reversed: its normal at a point inside it points against the original face's
    /// normal at the projection of that point (orientations included).
    /// </summary>
    public static bool IsSplitToReverse(in Face theFSp, in Face theFSr, IntTools_Context? theContext) =>
        IsSplitToReverse(theFSp, theFSr, theContext, out _);

    /// <summary>
    /// As <see cref="IsSplitToReverse(in Face, in Face, IntTools_Context?)"/>, with the error status: 0
    /// fine, 1 no point found in the split, 2 no normal on the split, 3 the projection failed, 4 no normal on the
    /// original.
    /// </summary>
    public static bool IsSplitToReverse(in Face theFSp, in Face theFSr, IntTools_Context? theContext, out int theError)
    {
        // Set OK error status
        theError = 0;

        // Compare surfaces
        Surface aSFSp = Tool.Surface(theFSp)!;
        Surface aSFOr = Tool.Surface(theFSr)!;
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
            Explorer anExp = new(theFSp, ShapeType.Edge);
            for (; anExp.More(); anExp.Next())
            {
                Edge aESp = ShapeCasts.Edge(anExp.Current());
                if (!Tool.Degenerated(aESp) && !Tool.IsClosed(aESp, theFSp))
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
        if (theFSp.Orientation() == Reversed)
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
        if (theFSr.Orientation() == Reversed)
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
    public static bool IsSplitToReverse(in Edge theESp, in Edge theEOr, IntTools_Context? theContext) =>
        IsSplitToReverse(theESp, theEOr, theContext, out _);

    /// <summary>
    /// As <see cref="IsSplitToReverse(in Edge, in Edge, IntTools_Context?)"/>, with the error status: 0
    /// fine, 1 a degenerated edge, else the last failure over the samples (2 no tangent on the split, 3 the projection
    /// failed, 4 no tangent on the original).
    /// </summary>
    public static bool IsSplitToReverse(in Edge theESp, in Edge theEOr, IntTools_Context? theContext, out int theError)
    {
        // The idea is to compare the tangent vectors of two edges computed in
        // the same point. Thus, we need to take the point on split edge (since it is
        // shorter) and project it onto original edge to find corresponding parameter.

        if (Tool.Degenerated(theESp) || Tool.Degenerated(theEOr))
        {
            theError = 1;
            return false;
        }

        // Set OK error status
        theError = 0;

        // Get the curves from the edges
        Curve3D? aCSp = Tool.Curve(theESp, out double f, out double l);
        Curve3D? aCOr = Tool.Curve(theEOr, out f, out l);

        // If the curves are the same, compare orientations only
        if (ReferenceEquals(aCSp, aCOr))
        {
            return theESp.Orientation() != theEOr.Orientation();
        }

        // Find valid range of the split edge, to ensure that the point for computing
        // tangent vectors will be inside both edges.
        if (!BRepLib.FindValidRange(theESp, ref f, ref l))
        {
            Tool.Range(theESp, out f, out l);
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
    public static bool IsHole(in Shape aW, in Shape aFace)
    {
        bool bIsHole;
        int i, aNbS;
        double aS;
        double aU1, aU, dU;
        double aX1, aY1, aX0, aY0;
        Orientation aOr;

        Point2D aP2D0, aP2D1;
        Curve2D? aC2D;
        Face aF, aFF;
        Iterator aItW = new();
        //
        bIsHole = false;
        //
        aF = ShapeCasts.Face(aFace);
        aFF = aF;
        aFF.Orientation(Forward);
        //
        aS = 0.0;
        aItW.Initialize(aW);
        for (; aItW.More(); aItW.Next())
        {
            Edge aE = ShapeCasts.Edge(aItW.Value());
            aOr = aE.Orientation();
            if (aOr != Forward && aOr != Reversed)
            {
                continue;
            }
            //
            aC2D = Tool.CurveOnSurface(aE, aFF, out double aT1, out double aT2);
            if (aC2D is null)
            {
                break; // xx
            }
            //
            EdgeCurve2DAdaptor aBAC2D = new(aE, aFF);
            aNbS = Geom2dInt_Geom2dCurveTool.NbSamples(aBAC2D);
            if (aNbS > 2)
            {
                aNbS *= 4;
            }
            //
            dU = (aT2 - aT1) / (double)(aNbS - 1);
            aU = aT1;
            aU1 = aT1;
            if (aOr == Reversed)
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
    public static void MakeContainer(ShapeType theType, ref Shape theC)
    {
        BrepBuilder aBB = new();
        //
        switch (theType)
        {
            case ShapeType.Compound:
                {
                    Compound aC = new();
                    aBB.MakeCompound(ref aC);
                    theC = aC;
                }
                break;
            //
            case ShapeType.CompSolid:
                {
                    CompSolid aCS = new();
                    aBB.MakeCompSolid(ref aCS);
                    theC = aCS;
                }
                break;
            //
            case ShapeType.Solid:
                {
                    Solid aSolid = new();
                    aBB.MakeSolid(ref aSolid);
                    theC = aSolid;
                }
                break;
            //
            //
            case ShapeType.Shell:
                {
                    Shell aShell = new();
                    aBB.MakeShell(ref aShell);
                    theC = aShell;
                }
                break;
            //
            case ShapeType.Wire:
                {
                    Wire aWire = new();
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
    public static void MakePCurve(in Edge aE, in Face aF1, in Face aF2, in IntTools_Curve aIC, bool bPC1, bool bPC2,
                                  IntTools_Context? theContext = null)
    {
        int i;
        double aTolE;
        Curve2D? aC2D, aC2DA, aC2Dx1;
        Face aFFWD;
        BrepBuilder aBB = new();
        bool bPC;
        //
        aTolE = Tool.Tolerance(aE);
        //
        Curve3D aC3DE = Tool.Curve(aE, out double aT1, out double aT2)!;
        TrimmedCurve3D aC3DETrim = new(aC3DE, aT1, aT2);
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
            aFFWD.Orientation(Forward);
            //
            aC2D = aC2Dx1;
            if (aC2D is null)
            {
                BOPTools_AlgoTools2D.BuildPCurveForEdgeOnFace(aE, aFFWD, theContext);
                BOPTools_AlgoTools2D.CurveOnSurface(aE, aFFWD, out aC2D, out _, out _, out _, theContext);
            }
            //
            if (aC3DE.IsPeriodic)
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
    public static void MakeEdge(in IntTools_Curve theIC, in Vertex theV1, double theT1, in Vertex theV2, double theT2,
                                double theTolR3D, out Edge theE)
    {
        BrepBuilder aBB = new();
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
    public static int ComputeVV(in Vertex aV1, in Point3D aP2, double aTolP2)
    {
        double aTolV1, aTolSum, aTolSum2, aD2;
        Point3D aP1;
        //
        aTolV1 = Tool.Tolerance(aV1);

        aTolSum = aTolV1 + aTolP2 + global::Kerf.Engine.Foundation.Precision.Confusion();
        aTolSum2 = aTolSum * aTolSum;
        //
        aP1 = Tool.Pnt(aV1);
        //
        aD2 = aP1.SquareDistance(aP2);
        if (aD2 > aTolSum2)
        {
            return 1;
        }
        return 0;
    }

    /// <summary>0 if the vertices are within the sum of their tolerances and the fuzzy value, else 1.</summary>
    public static int ComputeVV(in Vertex aV1, in Vertex aV2) => ComputeVV(aV1, aV2, global::Kerf.Engine.Foundation.Precision.Confusion());

    /// <inheritdoc cref="ComputeVV(in Vertex, in Vertex)"/>
    public static int ComputeVV(in Vertex aV1, in Vertex aV2, double aFuzz)
    {
        double aTolV1, aTolV2, aTolSum, aTolSum2, aD2;
        Point3D aP1, aP2;
        double aFuzz1 = (aFuzz > global::Kerf.Engine.Foundation.Precision.Confusion() ? aFuzz : global::Kerf.Engine.Foundation.Precision.Confusion());
        //
        aTolV1 = Tool.Tolerance(aV1);
        aTolV2 = Tool.Tolerance(aV2);
        aTolSum = aTolV1 + aTolV2 + aFuzz1;
        aTolSum2 = aTolSum * aTolSum;
        //
        aP1 = Tool.Pnt(aV1);
        aP2 = Tool.Pnt(aV2);
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
    public static void MakeVertex(NCollection_List<Shape> aLV, ref Vertex aVnew)
    {
        int aNb = aLV.Extent();
        if (aNb == 1)
        {
            aVnew = ShapeCasts.Vertex(aLV.First());
        }
        else if (aNb > 1)
        {
            double aNTol = 0.0;
            Point3D aNC = new();
            BRepLib.BoundingVertex(aLV, ref aNC, ref aNTol);
            BrepBuilder aBB = new();
            aBB.MakeVertex(ref aVnew, aNC, aNTol);
        }
    }

    /// <summary>Finds in the face the edge same as <paramref name="theE1"/> (any orientation), through its wires.</summary>
    public static bool GetEdgeOnFace(in Edge theE1, in Face theF2, ref Edge theE2)
    {
        bool bFound;
        Iterator aItF = new(), aItW = new();
        //
        bFound = false;
        //
        aItF.Initialize(theF2);
        for (; aItF.More(); aItF.Next())
        {
            Shape aW = aItF.Value();
            aItW.Initialize(aW);
            for (; aItW.More(); aItW.Next())
            {
                Shape aE = aItW.Value();
                if (aE.IsSame(theE1))
                {
                    theE2 = ShapeCasts.Edge(aE);
                    bFound = !bFound;
                    return bFound;
                }
            }
        }
        return bFound;
    }

    // Pairs the faces of thLF sharing theE (each with the face off it); false if the edge isn't on one of them.
    private static bool FindFacePairs(in Edge theE, NCollection_List<Shape> thLF,
                                      NCollection_List<BOPTools_CoupleOfShape> theLCFF, IntTools_Context theContext)
    {
        bool bFound;
        int i, aNbCEF;
        Orientation aOr, aOrC = Forward;
        NCollection_Map<Shape> aMFP = new(ShapeHasher.Instance);
        Face aF1 = new(), aF2 = new();
        Edge aEL = new(), aE1 = new();
        NCollection_List<Shape>.Iterator aItLF = new();
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
            Face aFL = ShapeCasts.Face(aItLF.Value());
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
                Shape aEx = aCSx.Shape1();
                Shape aFx = aCSx.Shape2();
                //
                aOr = aEx.Orientation();
                //
                if (i == 0)
                {
                    aOrC = global::Kerf.Engine.Topology.TopAbs.Reverse(aOr);
                    aE1 = ShapeCasts.Edge(aEx);
                    aF1 = ShapeCasts.Face(aFx);
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
                Shape aFx = aCSx.Shape2();
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
    public static bool IsBlockInOnFace(in IntTools_Range aShrR, in Face aF, in Edge aE1, IntTools_Context aContext)
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
        aTolE = Tool.Tolerance(aE1);
        aTolF = Tool.Tolerance(aF);
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
    public static bool IsMicroEdge(in Edge aE, IntTools_Context? aCtx, bool bCheckSplittable = true)
    {
        bool bRet;
        double aT1, aT2, aTmp;
        //
        bRet = (Tool.Degenerated(aE) || !Tool.IsGeometric(aE));
        if (bRet)
        {
            return bRet;
        }
        //
        _ = Tool.Curve(aE, out aT1, out aT2);
        Exploration.Vertices(aE, out Vertex aV1, out Vertex aV2);
        aT1 = Tool.Parameter(aV1, aE);
        aT2 = Tool.Parameter(aV2, aE);
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
    private static bool GetFaceDir(in Edge aE,
                                   in Face aF,
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
        if (aF.Orientation() == Reversed)
        {
            aDN.Reverse();
        }
        //
        aTolE = Tool.Tolerance(aE);
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
    private static bool FindPointInFace(in Face aF,
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
    private static double MinStep3D(in Edge theE1,
                                    in Face theF1,
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
        aTolE = Tool.Tolerance(theE1);
        aDtMax = -1.0;
        aDtMin = 5.0e-6;
        //
        NCollection_List<BOPTools_CoupleOfShape>.Iterator aIt = new(aLCS);
        for (; aIt.More(); aIt.Next())
        {
            BOPTools_CoupleOfShape aCS = aIt.Value();
            Face aF = ShapeCasts.Face(aCS.Shape2());
            //
            aTolF = Tool.Tolerance(aF);
            aDt = 2 * (aTolE + aTolF);
            if (aDt > aDtMax)
            {
                aDtMax = aDt;
            }
            //
            // try to compute the minimal 3D step
            FaceSurfaceAdaptor aBAS = theContext.SurfaceAdaptor(aF);
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
            Face aF = ShapeCasts.Face(aCS.Shape2());
            //
            FaceSurfaceAdaptor aBAS = theContext.SurfaceAdaptor(aF);
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
    public static bool IsOpenShell(in Shell aSh)
    {
        bool bRet;
        int i, aNbE, aNbF;
        Orientation aOrF;
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMEF = new(ShapeHasher.Instance);
        //
        bRet = false;
        //
        Exploration.MapShapesAndAncestors(aSh, ShapeType.Edge, ShapeType.Face, aMEF);
        //
        aNbE = aMEF.Extent();
        for (i = 1; i <= aNbE; ++i)
        {
            Edge aE = ShapeCasts.Edge(aMEF.FindKey(i));
            if (Tool.Degenerated(aE))
            {
                continue;
            }
            //
            aNbF = 0;
            NCollection_List<Shape> aLF = aMEF.FindFromIndex(i);
            for (NCollection_List<Shape>.Iterator aItLS = new(aLF); aItLS.More(); aItLS.Next())
            {
                Shape aF = aItLS.Value();
                aOrF = aF.Orientation();
                if (aOrF == Internal || aOrF == External)
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
    public static bool IsInvertedSolid(in Solid aSolid)
    {
        double aTolS;
        Containment aState;
        BRepClass3d_SolidClassifier aSC = new(aSolid);
        //
        aTolS = 1.0e-7;
        aSC.PerformInfinitePoint(aTolS);
        aState = aSC.State();
        return aState == In;
    }
}
