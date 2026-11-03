// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using static Kerf.Engine.Foundation.Gravity;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>Extends BOPAlgo_BuilderSolid with the solid to split and its own progress range, for the builder's parallel loop.</summary>
internal sealed class BOPAlgo_SplitSolid : BOPAlgo_BuilderSolid, BOPTools_Parallel.ISolver
{
    private TopoDS_Solid mySolid = new(); //!< Solid to split
    private ProgressRange myRange;

    //! Sets the solid
    public void SetSolid(in TopoDS_Solid theSolid) => mySolid = theSolid;

    //! Returns the solid
    public TopoDS_Solid Solid() => mySolid;

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
            if (aSI.ShapeType() == TopAbs_SOLID)
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
        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> aDraftSolids = new(TopTools_ShapeMapHasher.Instance);
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
    protected virtual void FillIn3DParts(NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theDraftSolids, ProgressRange theRange)
    {
        using var aPS = new ProgressScope(theRange, null, 2);

        // Find all faces that are IN solids

        // Store boxes of the shapes into a map
        NCollection_DataMap<TopoDS_Shape, Box> aShapeBoxMap = new(TopTools_ShapeMapHasher.Instance);

        // Fence map
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);

        // Get all faces
        NCollection_List<TopoDS_Shape> aLFaces = new();

        int i, aNbS = myDS!.NbSourceShapes();
        for (i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != TopAbs_FACE)
            {
                continue;
            }

            if (UserBreak(aPS))
            {
                return;
            }

            TopoDS_Shape aS = aSI.Shape();

            if (myImages.Find(aS, out NCollection_List<TopoDS_Shape> pLSIm))
            {
                NCollection_List<TopoDS_Shape>.Iterator aItLSIm = new(pLSIm);
                for (; aItLSIm.More(); aItLSIm.Next())
                {
                    TopoDS_Shape aSIm = aItLSIm.Value();
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

        BRep_Builder aBB = new();

        // Get all solids
        NCollection_List<TopoDS_Shape> aLSolids = new();
        // Keep INTERNAL faces of the solids
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aSolidsIF = new(TopTools_ShapeMapHasher.Instance);
        // Draft solids
        NCollection_IndexedDataMap<TopoDS_Shape, TopoDS_Shape> aDraftSolid = new(TopTools_ShapeMapHasher.Instance);

        for (i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ChangeShapeInfo(i);
            if (aSI.ShapeType() != TopAbs_SOLID)
            {
                continue;
            }
            if (UserBreak(aPS))
            {
                return;
            }
            TopoDS_Shape aS = aSI.Shape();
            TopoDS_Solid aSolid = TopoDS.Solid(aS);
            //
            // Bounding box for the solid aS
            ref Box aBoxS = ref aSI.ChangeBox();
            if (aBoxS.IsVoid)
            {
                myDS.BuildBndBoxSolid(i, ref aBoxS, myCheckInverted);
            }

            // Build Draft Solid
            NCollection_List<TopoDS_Shape> aLIF = new();
            TopoDS_Solid aSD0 = new();
            aBB.MakeSolid(ref aSD0);
            TopoDS_Shape aSD = aSD0;
            BuildDraftSolid(aSolid, ref aSD, aLIF);

            aLSolids.Append(aSD);
            aSolidsIF.Bind(aSD, aLIF);
            aShapeBoxMap.Bind(aSD, aBoxS);
            aDraftSolid.Add(aS, aSD);
        }

        // Perform classification of the faces
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anInParts = new(TopTools_ShapeMapHasher.Instance);

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
            TopoDS_Solid aSolid = TopoDS.Solid(aDraftSolid.FindKey(i));
            TopoDS_Solid aSDraft = TopoDS.Solid(aDraftSolid[i]);
            NCollection_List<TopoDS_Shape> aLInFaces = anInParts.FindFromKey(aSDraft);
            NCollection_List<TopoDS_Shape> aLInternal = aSolidsIF.Find(aSDraft);

            int aNbIN = aLInFaces.Extent();

            if (aNbIN == 0)
            {
                bool bHasImage = false;
                // Check if the shells of the solid have image
                for (TopoDS_Iterator it = new(aSolid); it.More() && !bHasImage; it.Next())
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
                NCollection_List<TopoDS_Shape> pLIN = myInParts.Bound(aSolid, new NCollection_List<TopoDS_Shape>());

                NCollection_List<TopoDS_Shape>.Iterator aItLS = new(aLInFaces);
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
    protected void BuildDraftSolid(in TopoDS_Shape theSolid, ref TopoDS_Shape theDraftSolid, NCollection_List<TopoDS_Shape> theLIF)
    {
        bool bToReverse;
        int iFlag;
        TopAbs_Orientation aOrF, aOrSh, aOrSd;
        TopoDS_Iterator aIt1 = new(), aIt2 = new();
        TopoDS_Shell aShD = new();
        TopoDS_Shape aFx;
        BRep_Builder aBB = new();
        NCollection_List<TopoDS_Shape>.Iterator aItS = new();
        //
        aOrSd = theSolid.Orientation();
        theDraftSolid.Orientation(aOrSd);
        //
        aIt1.Initialize(theSolid);
        for (; aIt1.More(); aIt1.Next())
        {
            TopoDS_Shape aSh = aIt1.Value();
            if (aSh.ShapeType() != TopAbs_SHELL)
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
                TopoDS_Shape aF = aIt2.Value();
                aOrF = aF.Orientation();
                //
                if (myImages.IsBound(aF))
                {
                    NCollection_List<TopoDS_Shape> aLSp = myImages.Find(aF);
                    aItS.Initialize(aLSp);
                    for (; aItS.More(); aItS.Next())
                    {
                        aFx = aItS.Value();
                        //
                        if (myShapesSD.IsBound(aFx))
                        {
                            //
                            if (aOrF == TopAbs_INTERNAL)
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
                            if (aOrF == TopAbs_INTERNAL)
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
                    if (aOrF == TopAbs_INTERNAL)
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
                aShD.Closed(BRep_Tool.IsClosed(aShD));
                aBB.Add(theDraftSolid, aShD);
            }
        } // for (; aIt1.More(); aIt1.Next()) {
    }

    /// <summary>Builds the split solids from the draft solids and their IN faces.</summary>
    protected void BuildSplitSolids(NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theDraftSolids, ProgressRange theRange)
    {
        bool bFlagSD;
        int i, aNbS;
        TopExp_Explorer aExp = new();
        NCollection_List<TopoDS_Shape>.Iterator aIt = new();
        //
        NCollection_List<TopoDS_Shape> aSFS = new();
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
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
            if (aSI.ShapeType() != TopAbs_SOLID)
            {
                continue;
            }
            if (UserBreak(aPSOuter))
            {
                return;
            }
            //
            TopoDS_Shape aS = aSI.Shape();
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
            aST.Add(aS, TopAbs_FACE);
            aMST.Add(aST);
            //
        } // for (i=1; i<=aNbS; ++i)
        //
        // Build temporary map of solids images to avoid rebuilding
        // of the solids without internal faces
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aSolidsIm = new(TopTools_ShapeMapHasher.Instance);
        // 1. Build solids for interfered source solids
        for (i = 0; i < aNbS; ++i)
        {
            BOPDS_ShapeInfo aSI = myDS.ShapeInfo(i);
            if (aSI.ShapeType() != TopAbs_SOLID)
            {
                continue;
            }

            TopoDS_Shape aS = aSI.Shape();
            TopoDS_Solid aSolid = TopoDS.Solid(aS);
            if (!theDraftSolids.IsBound(aS))
            {
                continue;
            }

            TopoDS_Shape aSD = theDraftSolids.Find(aS);
            if (!myInParts.Find(aS, out NCollection_List<TopoDS_Shape> pLFIN) || pLFIN.IsEmpty())
            {
                aSolidsIm[aSolidsIm.Add(aS, new NCollection_List<TopoDS_Shape>())].Append(aSD);
                continue;
            }

            aSFS.Clear();
            //
            // 1.1 Fill Shell Faces Set
            aExp.Init(aSD, TopAbs_FACE);
            for (; aExp.More(); aExp.Next())
            {
                TopoDS_Shape aF = aExp.Current();
                aSFS.Append(aF);
            }
            //
            // 1.2 Fill internal faces
            aIt.Initialize(pLFIN);
            for (; aIt.More(); aIt.Next())
            {
                TopoDS_Shape aF = aIt.Value();
                //
                aF.Orientation(TopAbs_FORWARD);
                aSFS.Append(aF);
                aF.Orientation(TopAbs_REVERSED);
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
            aSolidsIm.Add(aBS.Solid(), new NCollection_List<TopoDS_Shape>(aBS.Areas()));

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

                    if (itA.Value() is TopoDS_AlertWithShape anAlertWithShape)
                    {
                        TopoDS_Compound aWarnShape0 = new();
                        new BRep_Builder().MakeCompound(ref aWarnShape0);
                        TopoDS_Shape aWarnShape = aWarnShape0;
                        new BRep_Builder().Add(aWarnShape, aBS.Solid());
                        new BRep_Builder().Add(aWarnShape, anAlertWithShape.InternalGetShape());

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
            TopoDS_Shape aS = aSolidsIm.FindKey(k);
            NCollection_List<TopoDS_Shape> aLSR = aSolidsIm[k];
            //
            if (!myImages.IsBound(aS))
            {
                NCollection_List<TopoDS_Shape> pLSx = myImages.Bound(aS, new NCollection_List<TopoDS_Shape>());
                //
                aIt.Initialize(aLSR);
                for (; aIt.More(); aIt.Next())
                {
                    BOPTools_Set aST = new();
                    //
                    TopoDS_Shape aSR = aIt.Value();
                    aST.Add(aSR, TopAbs_FACE);
                    //
                    bFlagSD = aMST.Contains(aST);
                    //
                    BOPTools_Set aSTx = aMST.FindKey(aMST.Add(aST)); // OCCT: aMST.Added(aST)
                    TopoDS_Shape aSx = aSTx.Shape();
                    pLSx.Append(aSx);
                    //
                    if (!myOrigins.Find(aSx, out NCollection_List<TopoDS_Shape> pLOr))
                    {
                        pLOr = myOrigins.Bound(aSx, new NCollection_List<TopoDS_Shape>());
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
        TopAbs_ShapeEnum aType;
        TopAbs_State aState;
        TopoDS_Iterator aItS = new();
        BRep_Builder aBB = new();
        NCollection_List<TopoDS_Shape>.Iterator aIt = new(), aIt1 = new();
        //
        //-----------------------------------------------------scope f
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMSx = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedMap<TopoDS_Shape> aMx = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedMap<TopoDS_Shape> aMSI = new(TopTools_ShapeMapHasher.Instance);
        NCollection_Map<TopoDS_Shape> aMFence = new(TopTools_ShapeMapHasher.Instance);
        NCollection_Map<TopoDS_Shape> aMSOr = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape> aLSd = new();
        NCollection_List<TopoDS_Shape> aLArgs = new();
        NCollection_List<TopoDS_Shape> aLSC = new();
        NCollection_List<TopoDS_Shape> aLSI = new();

        using var aPS = new ProgressScope(theRange, null, 10);
        //
        // 1. Shapes to process
        //
        // 1.1 Shapes from pure arguments aMSI
        // 1.1.1 vertex, edge, wire
        //
        NCollection_List<TopoDS_Shape> aArguments = myDS!.Arguments();
        aIt.Initialize(aArguments);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aS = aIt.Value();
            BOPTools_AlgoTools.TreatCompound(aS, aLSC, aMFence);
        }
        aIt.Initialize(aLSC);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aS = aIt.Value();
            aType = aS.ShapeType();
            if (aType == TopAbs_WIRE)
            {
                aItS.Initialize(aS);
                for (; aItS.More(); aItS.Next())
                {
                    TopoDS_Shape aE = aItS.Value();
                    if (aMFence.Add(aE))
                    {
                        aLArgs.Append(aE);
                    }
                }
            }
            else if (aType == TopAbs_VERTEX || aType == TopAbs_EDGE)
            {
                aLArgs.Append(aS);
            }
        }
        aMFence.Clear();
        //
        aIt.Initialize(aLArgs);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aS = aIt.Value();
            aType = aS.ShapeType();
            if (aType == TopAbs_VERTEX || aType == TopAbs_EDGE || aType == TopAbs_WIRE)
            {
                if (aMFence.Add(aS))
                {
                    if (myImages.IsBound(aS))
                    {
                        NCollection_List<TopoDS_Shape> aLSp = myImages.Find(aS);
                        aIt1.Initialize(aLSp);
                        for (; aIt1.More(); aIt1.Next())
                        {
                            TopoDS_Shape aSp = aIt1.Value();
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
            if (aSI.ShapeType() != TopAbs_SOLID)
            {
                continue;
            }
            if (UserBreak(aPS))
            {
                return;
            }
            //
            TopoDS_Shape aS = aSI.Shape();
            //
            aMx.Clear();
            OwnInternalShapes(aS, aMx);
            //
            aNbSx = aMx.Extent();
            for (j = 1; j <= aNbSx; ++j)
            {
                TopoDS_Shape aSi = aMx[j];
                if (myImages.IsBound(aSi))
                {
                    NCollection_List<TopoDS_Shape> aLSp = myImages.Find(aSi);
                    aIt1.Initialize(aLSp);
                    for (; aIt1.More(); aIt1.Next())
                    {
                        TopoDS_Shape aSp = aIt1.Value();
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
                NCollection_List<TopoDS_Shape> aLSp = myImages.Find(aS);
                aIt.Initialize(aLSp);
                for (; aIt.More(); aIt.Next())
                {
                    TopoDS_Shape aSp = aIt.Value();
                    if (aMFence.Add(aSp))
                    {
                        TopExp.MapShapesAndAncestors(aSp, TopAbs_VERTEX, TopAbs_EDGE, aMSx);
                        TopExp.MapShapesAndAncestors(aSp, TopAbs_VERTEX, TopAbs_FACE, aMSx);
                        TopExp.MapShapesAndAncestors(aSp, TopAbs_EDGE, TopAbs_FACE, aMSx);
                        aLSd.Append(aSp);
                    }
                }
            }
            else
            {
                if (aMFence.Add(aS))
                {
                    TopExp.MapShapesAndAncestors(aS, TopAbs_VERTEX, TopAbs_EDGE, aMSx);
                    TopExp.MapShapesAndAncestors(aS, TopAbs_VERTEX, TopAbs_FACE, aMSx);
                    TopExp.MapShapesAndAncestors(aS, TopAbs_EDGE, TopAbs_FACE, aMSx);
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
            TopoDS_Shape aSI = aMSI[i];
            if (aMSx.Contains(aSI))
            {
                NCollection_List<TopoDS_Shape> aLSx = aMSx.FindFromKey(aSI);
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
            TopoDS_Solid aSd = TopoDS.Solid(aIt.Value());
            //
            aIt1.Initialize(aLSI);
            for (; aIt1.More();)
            {
                TopoDS_Shape aSI = aIt1.Value();
                aSI.Orientation(TopAbs_INTERNAL);
                //
                aState = BOPTools_AlgoTools.ComputeStateByOnePoint(aSI, aSd, 1.0e-11, myContext!);
                //
                if (aState != TopAbs_IN)
                {
                    aIt1.Next();
                    continue;
                }
                //
                if (aMSOr.Contains(aSd))
                {
                    // make new solid
                    TopoDS_Solid aSdx = new();
                    //
                    aBB.MakeSolid(ref aSdx);
                    aItS.Initialize(aSd);
                    for (; aItS.More(); aItS.Next())
                    {
                        TopoDS_Shape aSh = aItS.Value();
                        aBB.Add(aSdx, aSh);
                    }
                    //
                    aBB.Add(aSdx, aSI);
                    //
                    // no need to check for images of aSd as aMSOr contains only original solids
                    NCollection_List<TopoDS_Shape> pLS = myImages.Bound(aSd, new NCollection_List<TopoDS_Shape>());
                    pLS.Append(aSdx);
                    //
                    NCollection_List<TopoDS_Shape> pLOr = myOrigins.Bound(aSdx, new NCollection_List<TopoDS_Shape>());
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
    private static void OwnInternalShapes(in TopoDS_Shape theS, NCollection_IndexedMap<TopoDS_Shape> theMx)
    {
        TopoDS_Iterator aIt = new();
        //
        aIt.Initialize(theS);
        for (; aIt.More(); aIt.Next())
        {
            TopoDS_Shape aSx = aIt.Value();
            if (aSx.ShapeType() != TopAbs_SHELL)
            {
                theMx.Add(aSx);
            }
        }
    }
}
