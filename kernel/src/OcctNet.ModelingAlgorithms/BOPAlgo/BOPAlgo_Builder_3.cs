// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>Extends BOPAlgo_BuilderSolid with the solid to split and its own progress range, for the builder's parallel loop.</summary>
internal sealed class BOPAlgo_SplitSolid : BOPAlgo_BuilderSolid, BOPTools_Parallel.ISolver
{
    private Solid mySolid = new(); //!< Solid to split
    private ProgressRange myRange;

    //! Sets the solid
    public void SetSolid(in Solid theSolid) => mySolid = theSolid;

    //! Returns the solid
    public Solid Solid() => mySolid;

    //! Sets progress range
    public void SetProgressRange(ProgressRange theRange) => myRange = theRange;

    // New perform method, using own progress range
    public void Perform()
    {
        using var aPS = new ProgressScope(myRange, null, 1);
        if (!aPS.More())
        {
            return;
        }
        base.Perform(aPS.Next());
    }

    //! Disables the range enabled method (private in OCCT; C# can't narrow an override)
    public override void Perform(ProgressRange theRange = default)
    {
    }
}

public partial class BOPAlgo_Builder
{
    /// <summary>Fills the images of the solids: the split solids with their internal parts.</summary>
    protected void FillImagesSolids(ProgressRange theRange)
    {
        int i = 0, aNbS = myDS!.NbSourceShapes();
        for (i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() == ShapeType.Solid)
            {
                break;
            }
        }
        if (i >= aNbS)
        {
            return;
        }

        using var aPS = new ProgressScope(theRange, "Building splits of solids", 10);
        // Draft solids
        NCollection_DataMap<Shape, Shape> aDraftSolids = new(ShapeHasher.Instance);
        // Find all IN faces for all IN faces
        FillIn3DParts(aDraftSolids, aPS.Next(4));
        if (HasErrors())
        {
            return;
        }
        // Build split of the solids
        BuildSplitSolids(aDraftSolids, aPS.Next(5));
        if (HasErrors())
        {
            return;
        }
        // Fill solids with internal parts
        FillInternalShapes(aPS.Next());
    }

    /// <summary>Builds the draft solids and finds the faces IN each solid (its own internal ones and others').</summary>
    protected virtual void FillIn3DParts(NCollection_DataMap<Shape, Shape> theDraftSolids, ProgressRange theRange)
    {
        using var aPS = new ProgressScope(theRange, null, 2);

        // Find all faces that are IN solids

        // Store boxes of the shapes into a map
        NCollection_DataMap<Shape, Box> aShapeBoxMap = new(ShapeHasher.Instance);

        // Fence map
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);

        // Get all faces
        NCollection_List<Shape> aLFaces = new();

        int i, aNbS = myDS!.NbSourceShapes();
        for (i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != ShapeType.Face)
            {
                continue;
            }

            if (UserBreak(aPS))
            {
                return;
            }

            Shape aS = aSI.Shape();

            if (myImages.Find(aS, out NCollection_List<Shape> pLSIm))
            {
                NCollection_List<Shape>.Iterator aItLSIm = new(pLSIm);
                for (; aItLSIm.More(); aItLSIm.Next())
                {
                    Shape aSIm = aItLSIm.Value();
                    if (aMFence.Add(aSIm))
                    {
                        aLFaces.Append(aSIm);
                    }
                }
            }
            else
            {
                aLFaces.Append(aS);
                aShapeBoxMap.Bind(aS, aSI.Box());
            }
        }

        BrepBuilder aBB = new();

        // Get all solids
        NCollection_List<Shape> aLSolids = new();
        // Keep INTERNAL faces of the solids
        NCollection_DataMap<Shape, NCollection_List<Shape>> aSolidsIF = new(ShapeHasher.Instance);
        // Draft solids
        NCollection_IndexedDataMap<Shape, Shape> aDraftSolid = new(ShapeHasher.Instance);

        for (i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ChangeShapeInfo(i);
            if (aSI.ShapeType() != ShapeType.Solid)
            {
                continue;
            }
            if (UserBreak(aPS))
            {
                return;
            }
            Shape aS = aSI.Shape();
            Solid aSolid = ShapeCasts.Solid(aS);
            //
            // Bounding box for the solid aS
            ref Box aBoxS = ref aSI.ChangeBox();
            if (aBoxS.IsVoid)
            {
                myDS.BuildBndBoxSolid(i, ref aBoxS, myCheckInverted);
            }

            // Build Draft Solid
            NCollection_List<Shape> aLIF = new();
            Solid aSD0 = new();
            aBB.MakeSolid(ref aSD0);
            Shape aSD = aSD0;
            BuildDraftSolid(aSolid, ref aSD, aLIF);

            aLSolids.Append(aSD);
            aSolidsIF.Bind(aSD, aLIF);
            aShapeBoxMap.Bind(aSD, aBoxS);
            aDraftSolid.Add(aS, aSD);
        }

        // Perform classification of the faces
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anInParts = new(ShapeHasher.Instance);

        BOPAlgo_Tools.ClassifyFaces(aLFaces,
                                    aLSolids,
                                    myRunParallel,
                                    myContext!,
                                    anInParts,
                                    aShapeBoxMap,
                                    aSolidsIF,
                                    aPS.Next());

        // Analyze the results of classification
        int aNbSol = aDraftSolid.Extent();
        for (i = 1; i <= aNbSol; ++i)
        {
            if (UserBreak(aPS))
            {
                return;
            }
            Solid aSolid = ShapeCasts.Solid(aDraftSolid.FindKey(i));
            Solid aSDraft = ShapeCasts.Solid(aDraftSolid[i]);
            NCollection_List<Shape> aLInFaces = anInParts.FindFromKey(aSDraft);
            NCollection_List<Shape> aLInternal = aSolidsIF.Find(aSDraft);

            int aNbIN = aLInFaces.Extent();

            if (aNbIN == 0)
            {
                bool bHasImage = false;
                // Check if the shells of the solid have image
                for (Iterator it = new(aSolid); it.More() && !bHasImage; it.Next())
                {
                    bHasImage = myImages.IsBound(it.Value());
                }

                if (!bHasImage)
                {
                    // no need to split the solid
                    continue;
                }
            }

            theDraftSolids.Bind(aSolid, aSDraft);

            int aNbInt = aLInternal.Extent();
            if (aNbInt != 0 || aNbIN != 0)
            {
                // Combine the lists
                NCollection_List<Shape> pLIN = myInParts.Bound(aSolid, new NCollection_List<Shape>());

                NCollection_List<Shape>.Iterator aItLS = new(aLInFaces);
                for (; aItLS.More(); aItLS.Next())
                {
                    pLIN.Append(aItLS.Value());
                }

                aItLS.Initialize(aLInternal);
                for (; aItLS.More(); aItLS.Next())
                {
                    pLIN.Append(aItLS.Value());
                }
            }
        }
    }

    /// <summary>Builds the draft solid of the solid from the images of its faces, collecting its internal faces.</summary>
    protected void BuildDraftSolid(in Shape theSolid, ref Shape theDraftSolid, NCollection_List<Shape> theLIF)
    {
        bool bToReverse;
        int iFlag;
        Orientation aOrF, aOrSh, aOrSd;
        Iterator aIt1 = new(), aIt2 = new();
        Shell aShD = new();
        Shape aFx;
        BrepBuilder aBB = new();
        NCollection_List<Shape>.Iterator aItS = new();
        //
        aOrSd = theSolid.Orientation();
        theDraftSolid.Orientation(aOrSd);
        //
        aIt1.Initialize(theSolid);
        for (; aIt1.More(); aIt1.Next())
        {
            Shape aSh = aIt1.Value();
            if (aSh.ShapeType() != ShapeType.Shell)
            {
                continue; // mb internal edges,vertices
            }
            //
            aOrSh = aSh.Orientation();
            aBB.MakeShell(ref aShD);
            aShD.Orientation(aOrSh);
            iFlag = 0;
            //
            aIt2.Initialize(aSh);
            for (; aIt2.More(); aIt2.Next())
            {
                Shape aF = aIt2.Value();
                aOrF = aF.Orientation();
                //
                if (myImages.IsBound(aF))
                {
                    NCollection_List<Shape> aLSp = myImages.Find(aF);
                    aItS.Initialize(aLSp);
                    for (; aItS.More(); aItS.Next())
                    {
                        aFx = aItS.Value();
                        //
                        if (myShapesSD.IsBound(aFx))
                        {
                            //
                            if (aOrF == Internal)
                            {
                                aFx.Orientation(aOrF);
                                theLIF.Append(aFx);
                            }
                            else
                            {
                                bToReverse = BOPTools_AlgoTools.IsSplitToReverseWithWarn(aFx, aF, myContext, myReport);
                                if (bToReverse)
                                {
                                    aFx.Reverse();
                                }
                                //
                                iFlag = 1;
                                aBB.Add(aShD, aFx);
                            }
                        } // if (myShapesSD.IsBound(aFx)) {
                        else
                        {
                            aFx.Orientation(aOrF);
                            if (aOrF == Internal)
                            {
                                theLIF.Append(aFx);
                            }
                            else
                            {
                                iFlag = 1;
                                aBB.Add(aShD, aFx);
                            }
                        }
                    }
                } // if (myImages.IsBound(aF)) {
                //
                else
                {
                    if (aOrF == Internal)
                    {
                        theLIF.Append(aF);
                    }
                    else
                    {
                        iFlag = 1;
                        aBB.Add(aShD, aF);
                    }
                }
            } // for (; aIt2.More(); aIt2.Next()) {
            //
            if (iFlag != 0)
            {
                aShD.Closed(Tool.IsClosed(aShD));
                aBB.Add(theDraftSolid, aShD);
            }
        } // for (; aIt1.More(); aIt1.Next()) {
    }

    /// <summary>Builds the split solids from the draft solids and their IN faces.</summary>
    protected void BuildSplitSolids(NCollection_DataMap<Shape, Shape> theDraftSolids, ProgressRange theRange)
    {
        bool bFlagSD;
        int i, aNbS;
        Explorer aExp = new();
        NCollection_List<Shape>.Iterator aIt = new();
        //
        NCollection_List<Shape> aSFS = new();
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
        // OCCT's NCollection_Map<BOPTools_Set>, of which Added() returns the key in the map: an indexed map gives
        // the stored key back from its index, and the map is never iterated
        NCollection_IndexedMap<BOPTools_Set> aMST = new();
        NCollection_DynamicArray<BOPAlgo_SplitSolid> aVBS = new();
        //
        using var aPSOuter = new ProgressScope(theRange, null, 10);
        // 0. Find same domain solids for non-interfered solids
        aNbS = myDS!.NbSourceShapes();
        for (i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            //
            if (aSI.ShapeType() != ShapeType.Solid)
            {
                continue;
            }
            if (UserBreak(aPSOuter))
            {
                return;
            }
            //
            Shape aS = aSI.Shape();
            if (!aMFence.Add(aS))
            {
                continue;
            }
            if (theDraftSolids.IsBound(aS))
            {
                continue;
            }
            //
            BOPTools_Set aST = new();
            //
            aST.Add(aS, ShapeType.Face);
            aMST.Add(aST);
            //
        } // for (i=1; i<=aNbS; ++i)
        //
        // Build temporary map of solids images to avoid rebuilding
        // of the solids without internal faces
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aSolidsIm = new(ShapeHasher.Instance);
        // 1. Build solids for interfered source solids
        for (i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != ShapeType.Solid)
            {
                continue;
            }

            Shape aS = aSI.Shape();
            Solid aSolid = ShapeCasts.Solid(aS);
            if (!theDraftSolids.IsBound(aS))
            {
                continue;
            }

            Shape aSD = theDraftSolids.Find(aS);
            if (!myInParts.Find(aS, out NCollection_List<Shape> pLFIN) || pLFIN.IsEmpty())
            {
                aSolidsIm[aSolidsIm.Add(aS, new NCollection_List<Shape>())].Append(aSD);
                continue;
            }

            aSFS.Clear();
            //
            // 1.1 Fill Shell Faces Set
            aExp.Init(aSD, ShapeType.Face);
            for (; aExp.More(); aExp.Next())
            {
                Shape aF = aExp.Current();
                aSFS.Append(aF);
            }
            //
            // 1.2 Fill internal faces
            aIt.Initialize(pLFIN);
            for (; aIt.More(); aIt.Next())
            {
                Shape aF = aIt.Value();
                //
                aF.Orientation(Forward);
                aSFS.Append(aF);
                aF.Orientation(Reversed);
                aSFS.Append(aF);
            }
            //
            // 1.3 Build new solids
            BOPAlgo_SplitSolid aBS = aVBS.Append(new BOPAlgo_SplitSolid());
            aBS.SetSolid(aSolid);
            aBS.SetShapes(aSFS);
            aBS.SetRunParallel(myRunParallel);
        } // for (i=0; i<aNbS; ++i) {
        //
        int k, aNbBS;
        //
        aNbBS = aVBS.Length();
        // Set progress range for each task to be run in parallel
        using (var aPSParallel = new ProgressScope(aPSOuter.Next(9), "Splitting solids", aNbBS))
        {
            for (int iS = 0; iS < aNbBS; iS++)
            {
                BOPAlgo_SplitSolid aSplitSolid = aVBS.ChangeValue(iS);
                aSplitSolid.SetProgressRange(aPSParallel.Next());
            }
            //
            //===================================================
            BOPTools_Parallel.Perform(myRunParallel, aVBS);
            //===================================================
        }
        if (UserBreak(aPSOuter))
        {
            return;
        }
        //
        for (k = 0; k < aNbBS; ++k)
        {
            BOPAlgo_SplitSolid aBS = aVBS[k];
            aSolidsIm.Add(aBS.Solid(), new NCollection_List<Shape>(aBS.Areas()));

            // Merge BuilderSolid's report into main report,
            // assigning the solid with the warnings/errors which
            // have been generated for it.
            // Convert all errors of BuilderSolid into warnings for main report.
            Report aBSReport = aBS.GetReport();
            Span<Gravity> anAlertTypes = [Warning, Fail];
            for (int iGravity = 0; iGravity < 2; iGravity++)
            {
                NCollection_List<Alert> anLAlerts = aBSReport.GetAlerts(anAlertTypes[iGravity]);
                for (NCollection_List<Alert>.Iterator itA = new(anLAlerts); itA.More(); itA.Next())
                {
                    Alert anAlert = itA.Value();

                    if (itA.Value() is AlertWithShape anAlertWithShape)
                    {
                        Compound aWarnShape0 = new();
                        new BrepBuilder().MakeCompound(ref aWarnShape0);
                        Shape aWarnShape = aWarnShape0;
                        new BrepBuilder().Add(aWarnShape, aBS.Solid());
                        new BrepBuilder().Add(aWarnShape, anAlertWithShape.InternalGetShape());

                        anAlertWithShape.SetShape(aWarnShape);
                        AddWarning(anAlertWithShape);
                    }
                    else
                    {
                        AddWarning(anAlert);
                    }
                }
            }
        }
        //
        // Add new solids to images map
        aNbBS = aSolidsIm.Extent();
        for (k = 1; k <= aNbBS; ++k)
        {
            Shape aS = aSolidsIm.FindKey(k);
            NCollection_List<Shape> aLSR = aSolidsIm[k];
            //
            if (!myImages.IsBound(aS))
            {
                NCollection_List<Shape> pLSx = myImages.Bound(aS, new NCollection_List<Shape>());
                //
                aIt.Initialize(aLSR);
                for (; aIt.More(); aIt.Next())
                {
                    BOPTools_Set aST = new();
                    //
                    Shape aSR = aIt.Value();
                    aST.Add(aSR, ShapeType.Face);
                    //
                    bFlagSD = aMST.Contains(aST);
                    //
                    BOPTools_Set aSTx = aMST.FindKey(aMST.Add(aST)); // OCCT: aMST.Added(aST)
                    Shape aSx = aSTx.Shape();
                    pLSx.Append(aSx);
                    //
                    if (!myOrigins.Find(aSx, out NCollection_List<Shape> pLOr))
                    {
                        pLOr = myOrigins.Bound(aSx, new NCollection_List<Shape>());
                    }
                    pLOr.Append(aS);
                    //
                    if (bFlagSD)
                    {
                        myShapesSD.Bind(aSR, aSx);
                    }
                }
            }
        }
    }

    /// <summary>Settles the internal vertices and edges of the arguments into the split solids containing them.</summary>
    protected void FillInternalShapes(ProgressRange theRange)
    {
        int i, j, aNbS, aNbSI, aNbSx;
        ShapeType aType;
        Containment aState;
        Iterator aItS = new();
        BrepBuilder aBB = new();
        NCollection_List<Shape>.Iterator aIt = new(), aIt1 = new();
        //
        //-----------------------------------------------------scope f
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aMSx = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> aMx = new(ShapeHasher.Instance);
        NCollection_IndexedMap<Shape> aMSI = new(ShapeHasher.Instance);
        NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
        NCollection_Map<Shape> aMSOr = new(ShapeHasher.Instance);
        NCollection_List<Shape> aLSd = new();
        NCollection_List<Shape> aLArgs = new();
        NCollection_List<Shape> aLSC = new();
        NCollection_List<Shape> aLSI = new();

        using var aPS = new ProgressScope(theRange, null, 10);
        //
        // 1. Shapes to process
        //
        // 1.1 Shapes from pure arguments aMSI
        // 1.1.1 vertex, edge, wire
        //
        NCollection_List<Shape> aArguments = myDS!.Arguments();
        aIt.Initialize(aArguments);
        for (; aIt.More(); aIt.Next())
        {
            Shape aS = aIt.Value();
            BOPTools_AlgoTools.TreatCompound(aS, aLSC, aMFence);
        }
        aIt.Initialize(aLSC);
        for (; aIt.More(); aIt.Next())
        {
            Shape aS = aIt.Value();
            aType = aS.ShapeType();
            if (aType == ShapeType.Wire)
            {
                aItS.Initialize(aS);
                for (; aItS.More(); aItS.Next())
                {
                    Shape aE = aItS.Value();
                    if (aMFence.Add(aE))
                    {
                        aLArgs.Append(aE);
                    }
                }
            }
            else if (aType == ShapeType.Vertex || aType == ShapeType.Edge)
            {
                aLArgs.Append(aS);
            }
        }
        aMFence.Clear();
        //
        aIt.Initialize(aLArgs);
        for (; aIt.More(); aIt.Next())
        {
            Shape aS = aIt.Value();
            aType = aS.ShapeType();
            if (aType == ShapeType.Vertex || aType == ShapeType.Edge || aType == ShapeType.Wire)
            {
                if (aMFence.Add(aS))
                {
                    if (myImages.IsBound(aS))
                    {
                        NCollection_List<Shape> aLSp = myImages.Find(aS);
                        aIt1.Initialize(aLSp);
                        for (; aIt1.More(); aIt1.Next())
                        {
                            Shape aSp = aIt1.Value();
                            aMSI.Add(aSp);
                        }
                    }
                    else
                    {
                        aMSI.Add(aS);
                    }
                }
            }
        }
        if (UserBreak(aPS))
        {
            return;
        }

        //
        // 2. Internal vertices, edges from source solids
        aMFence.Clear();
        aLSd.Clear();
        //
        aNbS = myDS.NbSourceShapes();
        for (i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            //
            if (aSI.ShapeType() != ShapeType.Solid)
            {
                continue;
            }
            if (UserBreak(aPS))
            {
                return;
            }
            //
            Shape aS = aSI.Shape();
            //
            aMx.Clear();
            OwnInternalShapes(aS, aMx);
            //
            aNbSx = aMx.Extent();
            for (j = 1; j <= aNbSx; ++j)
            {
                Shape aSi = aMx[j];
                if (myImages.IsBound(aSi))
                {
                    NCollection_List<Shape> aLSp = myImages.Find(aSi);
                    aIt1.Initialize(aLSp);
                    for (; aIt1.More(); aIt1.Next())
                    {
                        Shape aSp = aIt1.Value();
                        aMSI.Add(aSp);
                    }
                }
                else
                {
                    aMSI.Add(aSi);
                }
            }
            //
            // build aux map from splits of solids
            if (myImages.IsBound(aS))
            {
                NCollection_List<Shape> aLSp = myImages.Find(aS);
                aIt.Initialize(aLSp);
                for (; aIt.More(); aIt.Next())
                {
                    Shape aSp = aIt.Value();
                    if (aMFence.Add(aSp))
                    {
                        Exploration.MapShapesAndAncestors(aSp, ShapeType.Vertex, ShapeType.Edge, aMSx);
                        Exploration.MapShapesAndAncestors(aSp, ShapeType.Vertex, ShapeType.Face, aMSx);
                        Exploration.MapShapesAndAncestors(aSp, ShapeType.Edge, ShapeType.Face, aMSx);
                        aLSd.Append(aSp);
                    }
                }
            }
            else
            {
                if (aMFence.Add(aS))
                {
                    Exploration.MapShapesAndAncestors(aS, ShapeType.Vertex, ShapeType.Edge, aMSx);
                    Exploration.MapShapesAndAncestors(aS, ShapeType.Vertex, ShapeType.Face, aMSx);
                    Exploration.MapShapesAndAncestors(aS, ShapeType.Edge, ShapeType.Face, aMSx);
                    aLSd.Append(aS);
                    aMSOr.Add(aS);
                }
            }
        } // for (i=0; i<aNbS; ++i) {
        //
        // 3. Some shapes of aMSI can be already tied with faces of
        //    split solids
        aNbSI = aMSI.Extent();
        for (i = 1; i <= aNbSI; ++i)
        {
            Shape aSI = aMSI[i];
            if (aMSx.Contains(aSI))
            {
                NCollection_List<Shape> aLSx = aMSx.FindFromKey(aSI);
                aNbSx = aLSx.Extent();
                if (aNbSx == 0)
                {
                    aLSI.Append(aSI);
                }
            }
            else
            {
                aLSI.Append(aSI);
            }
        }
        //
        // 4. Just check it
        aNbSI = aLSI.Extent();
        if (aNbSI == 0)
        {
            return;
        }

        aPS.Next();
        //
        // 5 Settle internal vertices and edges into solids
        aMx.Clear();

        using var aPSLoop = new ProgressScope(aPS.Next(9), "Looking for internal shapes", aLSd.Extent());

        aIt.Initialize(aLSd);
        for (; aIt.More(); aIt.Next(), aPSLoop.Next())
        {
            Solid aSd = ShapeCasts.Solid(aIt.Value());
            //
            aIt1.Initialize(aLSI);
            for (; aIt1.More();)
            {
                Shape aSI = aIt1.Value();
                aSI.Orientation(Internal);
                //
                aState = BOPTools_AlgoTools.ComputeStateByOnePoint(aSI, aSd, 1.0e-11, myContext!);
                //
                if (aState != In)
                {
                    aIt1.Next();
                    continue;
                }
                //
                if (aMSOr.Contains(aSd))
                {
                    // make new solid
                    Solid aSdx = new();
                    //
                    aBB.MakeSolid(ref aSdx);
                    aItS.Initialize(aSd);
                    for (; aItS.More(); aItS.Next())
                    {
                        Shape aSh = aItS.Value();
                        aBB.Add(aSdx, aSh);
                    }
                    //
                    aBB.Add(aSdx, aSI);
                    //
                    // no need to check for images of aSd as aMSOr contains only original solids
                    NCollection_List<Shape> pLS = myImages.Bound(aSd, new NCollection_List<Shape>());
                    pLS.Append(aSdx);
                    //
                    NCollection_List<Shape> pLOr = myOrigins.Bound(aSdx, new NCollection_List<Shape>());
                    pLOr.Append(aSd);
                    //
                    aMSOr.Remove(aSd);
                    aSd = aSdx;
                }
                else
                {
                    // OCCT adds the shape into the split solid, which may be an argument's solid kept as the
                    // same-domain solid of a split (BuildSplitSolids' aSx; a frozen argument solid then throws, unit 6F)
                    aBB.Add(aSd, aSI);
                }
                //
                aLSI.Remove(aIt1);
            } // for (; aIt1.More();) {
        } // for (; aIt.More(); aIt.Next()) {
        //
        //-----------------------------------------------------scope t
        aLArgs.Clear();
        aLSd.Clear();
        aMSOr.Clear();
        aMFence.Clear();
        aMSI.Clear();
        aMx.Clear();
        aMSx.Clear();
    }

    // The own internal parts (all but the shells) of the shape.
    private static void OwnInternalShapes(in Shape theS, NCollection_IndexedMap<Shape> theMx)
    {
        Iterator aIt = new();
        //
        aIt.Initialize(theS);
        for (; aIt.More(); aIt.Next())
        {
            Shape aSx = aIt.Value();
            if (aSx.ShapeType() != ShapeType.Shell)
            {
                theMx.Add(aSx);
            }
        }
    }
}
