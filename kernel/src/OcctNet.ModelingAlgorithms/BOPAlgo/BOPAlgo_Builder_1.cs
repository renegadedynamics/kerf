// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

public partial class BOPAlgo_Builder
{
    /// <summary>Fills the images of the vertices: each vertex to its same-domain vertex.</summary>
    protected void FillImagesVertices(Message_ProgressRange theRange)
    {
        using var aPS = new Message_ProgressScope(theRange, "Filling splits of vertices", myDS!.ShapesSD().Extent());
        // the DS's integer map iterates in OCCT's order (decision 784)
        NCollection_IntDataMap<int>.Iterator aIt = new(myDS.ShapesSD());
        for (; aIt.More(); aIt.Next(), aPS.Next())
        {
            if (UserBreak(aPS))
            {
                return;
            }
            int nV = aIt.Key();
            int nVSD = aIt.Value();

            TopoDS_Shape aV = myDS.Shape(nV);
            TopoDS_Shape aVSD = myDS.Shape(nVSD);
            // Add to Images map
            myImages.Bound(aV, new NCollection_List<TopoDS_Shape>()).Append(aVSD);
            // Add to SD map
            myShapesSD.Bind(aV, aVSD);
            // Add to Origins map
            if (!myOrigins.Find(aVSD, out NCollection_List<TopoDS_Shape> pLOr))
            {
                pLOr = myOrigins.Bound(aVSD, new NCollection_List<TopoDS_Shape>());
            }
            pLOr.Append(aV);
        }
    }

    /// <summary>Fills the images of the edges: the edges of their (real) pave blocks.</summary>
    protected void FillImagesEdges(Message_ProgressRange theRange)
    {
        int i, aNbS = myDS!.NbSourceShapes();
        using var aPS = new Message_ProgressScope(theRange, "Filling splits of edges", aNbS);
        for (i = 0; i < aNbS; ++i, aPS.Next())
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != TopAbs_EDGE)
            {
                continue;
            }
            //
            // Check if the pave blocks for the edge have been initialized
            if (!aSI.HasReference())
            {
                continue;
            }
            //
            TopoDS_Shape aE = aSI.Shape();
            NCollection_List<BOPDS_PaveBlock> aLPB = myDS.PaveBlocks(i);
            //
            // Fill the images of the edge from the list of its pave blocks.
            // The small edges, having no pave blocks, will have the empty list
            // of images and, thus, will be avoided in the result.
            NCollection_List<TopoDS_Shape> pLS = myImages.Bound(aE, new NCollection_List<TopoDS_Shape>());
            //
            NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new(aLPB);
            for (; aItPB.More(); aItPB.Next())
            {
                BOPDS_PaveBlock aPB = aItPB.Value();
                BOPDS_PaveBlock aPBR = myDS.RealPaveBlock(aPB);
                //
                int nSpR = aPBR.Edge();
                TopoDS_Shape aSpR = myDS.Shape(nSpR);
                pLS.Append(aSpR);
                //
                if (!myOrigins.Find(aSpR, out NCollection_List<TopoDS_Shape> pLOr))
                {
                    pLOr = myOrigins.Bound(aSpR, new NCollection_List<TopoDS_Shape>());
                }
                pLOr.Append(aE);
                //
                if (myDS.IsCommonBlockOnEdge(aPB))
                {
                    int nSp = aPB.Edge();
                    TopoDS_Shape aSp = myDS.Shape(nSp);
                    myShapesSD.Bind(aSp, aSpR);
                }
            }
            if (UserBreak(aPS))
            {
                return;
            }
        }
    }

    /// <summary>Adds the images of the arguments of the given type (or the arguments themselves) into the result.</summary>
    protected virtual void BuildResult(TopAbs_ShapeEnum theType)
    {
        // Fence map
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
        // Iterate on all arguments of given type
        // and add their images into result
        NCollection_List<TopoDS_Shape>.Iterator aItA = new(myArguments);
        for (; aItA.More(); aItA.Next())
        {
            TopoDS_Shape aS = aItA.Value();
            if (aS.ShapeType() != theType)
            {
                continue;
            }
            // Get images
            if (!myImages.Find(aS, out NCollection_List<TopoDS_Shape> pLSIm))
            {
                // No images -> add the argument shape itself into result
                if (aMFence.Add(aS))
                {
                    new BRep_Builder().Add(myShape, aS);
                }
            }
            else
            {
                // Add images of the argument shape into result
                NCollection_List<TopoDS_Shape>.Iterator aItIm = new(pLSIm);
                for (; aItIm.More(); aItIm.Next())
                {
                    TopoDS_Shape aSIm = aItIm.Value();
                    if (aMFence.Add(aSIm))
                    {
                        new BRep_Builder().Add(myShape, aSIm);
                    }
                }
            }
        }
    }

    /// <summary>Fills the images of the containers (wires, shells, compsolids) of the given type.</summary>
    protected void FillImagesContainers(TopAbs_ShapeEnum theType, Message_ProgressRange theRange)
    {
        int i, aNbS;
        //
        aNbS = myDS!.NbSourceShapes();
        using var aPS = new Message_ProgressScope(theRange, "Building splits of containers", 1);
        for (i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() == theType)
            {
                TopoDS_Shape aC = aSI.Shape();
                FillImagesContainer(aC, theType);
            }
            if (UserBreak(aPS))
            {
                return;
            }
        } // for (; aItS.More(); aItS.Next()) {
    }

    /// <summary>Fills the images of the compounds.</summary>
    protected void FillImagesCompounds(Message_ProgressRange theRange)
    {
        int i, aNbS;
        NCollection_Map<TopoDS_Shape> aMFP = new(TopTools_ShapeMapHasher.Instance);
        //
        aNbS = myDS!.NbSourceShapes();
        using var aPS = new Message_ProgressScope(theRange, "Building splits of compounds", aNbS);
        for (i = 0; i < aNbS; ++i, aPS.Next())
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() == TopAbs_COMPOUND)
            {
                TopoDS_Shape aC = aSI.Shape();
                FillImagesCompound(aC, aMFP);
            }
            if (UserBreak(aPS))
            {
                return;
            }
        } // for (; aItS.More(); aItS.Next()) {
    }

    /// <summary>Makes the image of the container from the splits of its sub-shapes, if any of them is split.</summary>
    protected void FillImagesContainer(in TopoDS_Shape theS, TopAbs_ShapeEnum theType)
    {
        // Check if any of the sub-shapes of the container have been modified
        TopoDS_Iterator aIt = new(theS);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aSS = aIt.Value();
            if (myImages.Find(aSS, out NCollection_List<TopoDS_Shape> pLFIm)
                && ((pLFIm.Extent() != 1) || !pLFIm.First().IsSame(aSS)))
            {
                break;
            }
        }

        if (!aIt.More())
        {
            // Non of the sub-shapes have been modified.
            // No need to create the new container.
            return;
        }

        BRep_Builder aBB = new();
        // Make the new container of the splits of its sub-shapes
        TopoDS_Shape aCIm = new();
        BOPTools_AlgoTools.MakeContainer(theType, ref aCIm);

        aIt.Initialize(theS);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aSS = aIt.Value();

            if (!myImages.Find(aSS, out NCollection_List<TopoDS_Shape> pLSSIm))
            {
                // No splits, add the sub-shape itself
                aBB.Add(aCIm, aSS);
                continue;
            }

            // Add the splits
            NCollection_List<TopoDS_Shape>.Iterator aItIm = new(pLSSIm);
            for (; aItIm.More(); aItIm.Next())
            {
                TopoDS_Shape aSSIm = aItIm.Value();
                if (!aSSIm.IsEqual(aSS)
                    && BOPTools_AlgoTools.IsSplitToReverseWithWarn(aSSIm, aSS, myContext, myReport))
                {
                    aSSIm.Reverse();
                }
                aBB.Add(aCIm, aSSIm);
            }
        }

        aCIm.Closed(BRep_Tool.IsClosed(aCIm));
        myImages.Bound(theS, new NCollection_List<TopoDS_Shape>()).Append(aCIm);
    }

    /// <summary>Makes the image of the compound (and of its sub-compounds) from the images of its components.</summary>
    protected void FillImagesCompound(in TopoDS_Shape theS, NCollection_Map<TopoDS_Shape> theMFP)
    {
        bool bInterferred;
        TopAbs_Orientation aOrX;
        TopoDS_Iterator aIt = new();
        BRep_Builder aBB = new();
        NCollection_List<TopoDS_Shape>.Iterator aItIm = new();
        //
        if (!theMFP.Add(theS))
        {
            return;
        }
        //
        bInterferred = false;
        aIt.Initialize(theS);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aSx = aIt.Value();
            if (aSx.ShapeType() == TopAbs_COMPOUND)
            {
                FillImagesCompound(aSx, theMFP);
            }
            if (myImages.IsBound(aSx))
            {
                bInterferred = true;
            }
        }
        if (!bInterferred)
        {
            return;
        }
        //
        TopoDS_Shape aCIm = new();
        BOPTools_AlgoTools.MakeContainer(TopAbs_COMPOUND, ref aCIm);
        //
        aIt.Initialize(theS);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aSX = aIt.Value();
            aOrX = aSX.Orientation();
            if (myImages.IsBound(aSX))
            {
                NCollection_List<TopoDS_Shape> aLFIm = myImages.Find(aSX);
                aItIm.Initialize(aLFIm);
                for (; aItIm.More(); aItIm.Next())
                {
                    TopoDS_Shape aSXIm = aItIm.Value();
                    aSXIm.Orientation(aOrX);
                    aBB.Add(aCIm, aSXIm);
                }
            }
            else
            {
                aBB.Add(aCIm, aSX);
            }
        }
        //
        NCollection_List<TopoDS_Shape> aLSIm = new();
        aLSIm.Append(aCIm);
        myImages.Bind(theS, aLSIm);
    }
}
