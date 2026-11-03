// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_RemoveFeatures.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_RemoveFeatures.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2018-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Removes features (connected sets of faces: holes, bosses, pockets, fillets, chamfers) from a solid, a compsolid
/// or a compound of solids, filling each gap by extending the faces adjacent to the feature and trimming them by each
/// other and by the original faces' bounds, then rebuilding the solids with <see cref="BOPAlgo_MakerVolume"/>.
/// </summary>
/// <remarks>
/// <para>
/// The features are removed one by one, so a feature that can't be removed (warning
/// <see cref="BOPAlgo_AlertUnableToRemoveTheFeature"/>) doesn't stop the others. The result is simplified by
/// <see cref="ShapeUpgrade_UnifySameDomain"/>, keeping the input's edges and vertices that are in the result. The
/// history (<see cref="BOPAlgo_BuilderShape.History"/>) answers for the input's sub-shapes.
/// </para>
/// <para>
/// The input is a published (frozen) shape and is never written. The result shares the input's untouched sub-shapes
/// by TShape, as OCCT's does. OCCT writes into a face of the solids it rebuilds when it removes the face's internal
/// wires; where that face is a frozen input face, the port replaces it by a copy without those wires (decisions,
/// unit 6G).
/// </para>
/// </remarks>
public class BOPAlgo_RemoveFeatures : BOPAlgo_BuilderShape
{
    // Inputs
    protected Shape myInputShape = new();                     //!< Input shape
    protected NCollection_List<Shape> myFacesToRemove = new(); //!< Faces to remove

    // Intermediate
    protected NCollection_List<Shape> myFeatures = new(); //!< List of not connected features to remove (each feature is a compound of faces)
    protected NCollection_IndexedMap<Shape> myInputsMap = new(ShapeHasher.Instance); //!< Map of all sub-shapes of the input shape

    /// <summary>Empty constructor.</summary>
    public BOPAlgo_RemoveFeatures()
    {
    }

    /// <summary>Sets the shape for processing: a SOLID, COMPSOLID or COMPOUND of solids.</summary>
    public void SetShape(in Shape theShape) => myInputShape = theShape;

    /// <summary>The input shape.</summary>
    public Shape InputShape() => myInputShape;

    /// <summary>Adds the faces of <paramref name="theFace"/> to the faces to remove.</summary>
    public void AddFaceToRemove(in Shape theFace) => myFacesToRemove.Append(theFace);

    /// <summary>Adds the faces of the shapes of <paramref name="theFaces"/> to the faces to remove.</summary>
    public void AddFacesToRemove(NCollection_List<Shape> theFaces)
    {
        NCollection_List<Shape>.Iterator it = new(theFaces);
        for (; it.More(); it.Next())
        {
            myFacesToRemove.Append(it.Value());
        }
    }

    /// <summary>The shapes requested for removal.</summary>
    public NCollection_List<Shape> FacesToRemove() => myFacesToRemove;

    /// <summary>Clears the contents of the algorithm from the previous run, allowing reusing it.</summary>
    public override void Clear()
    {
        base.Clear();
        myInputShape.Nullify();
        myShape.Nullify();
        myFacesToRemove.Clear();
        myFeatures.Clear();
        myInputsMap.Clear();
    }

    // The steps of the operation (OCCT's file-local enum).
    private enum BOPAlgo_PIOperation
    {
        PIOperation_PrepareFeatures = 0,
        PIOperation_RemoveFeatures,
        PIOperation_UpdateHistory,
        PIOperation_SimplifyResult,

        PIOperation_Last,
    }

    protected override void fillPIConstants(double theWhole, BOPAlgo_PISteps theSteps)
    {
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PrepareFeatures, 0.05 * theWhole);
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_RemoveFeatures, 0.8 * theWhole);
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_UpdateHistory, 0.05 * theWhole);
        theSteps.SetStep((int)BOPAlgo_PIOperation.PIOperation_SimplifyResult, 0.1 * theWhole);
    }

    /// <summary>Performs the removal of the requested faces from the input shape.</summary>
    public override void Perform(ProgressRange theRange = default)
    {
        try
        {
            if (HasHistory())
            {
                myHistory = new History();
            }

            // Check the input data
            CheckData();
            if (HasErrors())
            {
                return;
            }
            using var aPS = new ProgressScope(theRange, "Removing features", 100);
            BOPAlgo_PISteps aSteps = new((int)BOPAlgo_PIOperation.PIOperation_Last);
            analyzeProgress(100.0, aSteps);

            // Prepare the faces to remove.
            PrepareFeatures(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PrepareFeatures)));
            if (HasErrors())
            {
                return;
            }

            // Remove the features and fill the created gaps
            RemoveFeatures(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_RemoveFeatures)));
            if (HasErrors())
            {
                return;
            }

            // Update history with the removed features
            UpdateHistory(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_UpdateHistory)));
            if (HasErrors())
            {
                return;
            }
            // Simplify the result
            SimplifyResult(aPS.Next(aSteps.GetStep((int)BOPAlgo_PIOperation.PIOperation_SimplifyResult)));
            if (HasErrors())
            {
                return;
            }
            // Post treatment
            PostTreat();
        }
        catch (FailureException)
        {
            AddError(new BOPAlgo_AlertRemoveFeaturesFailed());
        }
    }

    /// <summary>
    /// Checks the input data: the input shape must be a SOLID, COMPSOLID or COMPOUND of solids; from a compound only
    /// the solids are kept (the other shapes are removed with a warning).
    /// </summary>
    protected override void CheckData()
    {
        // Prepare the shape to work with
        myShape = myInputShape;

        // Check the type of input shape
        ShapeType aType = myInputShape.ShapeType();

        if (aType == ShapeType.Solid || aType == ShapeType.CompSolid)
        {
            return; // OK
        }

        if (aType == ShapeType.Compound)
        {
            NCollection_List<Shape> aShapes = new();
            NCollection_Map<Shape> aMFence = new(ShapeHasher.Instance);
            // Extract all shapes from the compound
            BOPTools_AlgoTools.TreatCompound(myInputShape, aShapes, aMFence);
            if (aShapes.IsEmpty())
            {
                // Add error of empty input shape
                AddError(new BOPAlgo_AlertEmptyShape(myInputShape));
                return;
            }

            // Find all solids in the list of shapes
            NCollection_List<Shape> aSolids = new();
            NCollection_List<Shape> anOtherShapes = new();
            NCollection_List<Shape>.Iterator aIt = new(aShapes);
            for (; aIt.More(); aIt.Next())
            {
                Shape aS = aIt.Value();
                if (aS.ShapeType() == ShapeType.Solid || aS.ShapeType() == ShapeType.CompSolid)
                {
                    aSolids.Append(aS);
                }
                else
                {
                    anOtherShapes.Append(aS);
                }
            }

            if (aSolids.IsEmpty())
            {
                // No solids have been found for processing.
                // Add error of unsupported type of input shape
                AddError(new BOPAlgo_AlertTooFewArguments());
            }
            else if (anOtherShapes.Extent() > 0)
            {
                // Add warning of unsupported type of input shape for all
                // non-solid shapes, contained in the input shape
                for (aIt.Initialize(anOtherShapes); aIt.More(); aIt.Next())
                {
                    AddWarning(new BOPAlgo_AlertUnsupportedType(aIt.Value()));
                }

                // Collect all solids into compound and overwrite the shape to rebuild
                Compound aCS = new();
                new BrepBuilder().MakeCompound(ref aCS);
                for (aIt.Initialize(aSolids); aIt.More(); aIt.Next())
                {
                    new BrepBuilder().Add(aCS, aIt.Value());
                }

                myShape = aCS;

                if (HasHistory())
                {
                    // Make non solid shapes removed in the history
                    MakeRemoved(anOtherShapes, myHistory!);
                }
            }
        }
        else
        {
            // Add error of unsupported type of input shape
            AddError(new BOPAlgo_AlertTooFewArguments());
        }
    }

    /// <summary>Prepares the features: the faces to remove that belong to the input, in connected blocks.</summary>
    protected void PrepareFeatures(ProgressRange theRange)
    {
        // Map all sub-shapes of the input solids
        Exploration.MapShapes(myInputShape, myInputsMap);

        // Collect all faces of the input shape requested for removal
        NCollection_List<Shape> aFacesToRemove = new();
        NCollection_List<Shape>.Iterator aIt = new(myFacesToRemove);
        using var aPSOuter = new ProgressScope(theRange, "Preparing features", 2);
        using var aPS = new ProgressScope(aPSOuter.Next(), "Preparing the faces to remove", myFacesToRemove.Extent());
        for (; aIt.More(); aIt.Next(), aPS.Next())
        {
            Shape aS = aIt.Value();
            Explorer anExpF = new(aS, ShapeType.Face);
            for (; anExpF.More(); anExpF.Next())
            {
                if (UserBreak(aPS))
                {
                    return;
                }
                Shape aF = anExpF.Current();
                if (myInputsMap.Contains(aF))
                {
                    aFacesToRemove.Append(aF);
                }
            }
        }

        if (aFacesToRemove.IsEmpty())
        {
            // Add error, that no features to remove have been found
            AddError(new BOPAlgo_AlertNoFacesToRemove());
            return;
        }

        // Build connexity blocks of the faces to remove
        Compound aCFToRemove = new();
        new BrepBuilder().MakeCompound(ref aCFToRemove);
        for (aIt.Initialize(aFacesToRemove); aIt.More(); aIt.Next())
        {
            new BrepBuilder().Add(aCFToRemove, aIt.Value());
        }

        // Fill the list of features with connexity blocks of faces
        BOPTools_AlgoTools.MakeConnexityBlocks(aCFToRemove, ShapeType.Edge, ShapeType.Face, myFeatures);
    }

    //=======================================================================
    // Adjacent faces extension block

    /// <summary>
    /// OCCT's file-local class FillGap: makes the faces filling the gap left by one feature (the adjacent faces
    /// extended and trimmed). A solver of the parallel loop; it reads the shared connection maps only.
    /// </summary>
    private sealed class FillGap : BOPTools_Parallel.ISolver
    {
        private bool myRunParallel;                                                   //!< Defines the mode of processing of the single feature
        private Shape myFeature = new();                                       //!< Feature to remove
        private NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>? myEFMap; //!< EF Connection map to find adjacent faces
        private NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>? myFSMap; //!< FS Connection map to find solids participating in the feature removal
        private ProgressRange myRange;                                        //!< Indication of progress

        // Results
        private readonly NCollection_Map<Shape> myFeatureFacesMap = new(ShapeHasher.Instance); //!< Faces of the feature
        private bool myHasAdjacentFaces;                                                                         //!< Flag to show whether the adjacent faces have been found or not
        private readonly NCollection_IndexedMap<Shape> mySolids = new(ShapeHasher.Instance);  //!< Solids participating in the feature removal
        private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myFaces = new(ShapeHasher.Instance); //!< Reconstructed adjacent faces
        private History? myHistory;                                                                    //!< History of the adjacent faces reconstruction

        public FillGap()
        {
            myRunParallel = false;
            myHasAdjacentFaces = false;
        }

        public void SetFeature(in Shape theFeature) => myFeature = theFeature;

        public Shape Feature() => myFeature;

        public void SetEFConnectionMap(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theEFMap) => myEFMap = theEFMap;

        public void SetFSConnectionMap(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theFSMap) => myFSMap = theFSMap;

        public void SetRunParallel(bool bRunParallel) => myRunParallel = bRunParallel;

        public History? History() => myHistory;

        public void SetRange(ProgressRange theRange) => myRange = theRange;

        /// <summary>Extends the adjacent faces, then trims the extended faces to fill the gap.</summary>
        public void Perform()
        {
            try
            {
                using var aPS = new ProgressScope(myRange, null, 3);

                myHistory = new History();

                // Find the faces adjacent to the faces of the feature
                NCollection_IndexedMap<Shape> aMFAdjacent = new(ShapeHasher.Instance);
                FindAdjacentFaces(aMFAdjacent, aPS.Next());
                if (!aPS.More())
                {
                    return;
                }

                myHasAdjacentFaces = aMFAdjacent.Extent() > 0;
                if (!myHasAdjacentFaces)
                {
                    return;
                }

                // Extend the adjacent faces keeping the connection to the original faces
                NCollection_IndexedDataMap<Shape, Shape> aFaceExtFaceMap = new(ShapeHasher.Instance);
                ExtendAdjacentFaces(aMFAdjacent, aFaceExtFaceMap, aPS.Next());
                if (!aPS.More())
                {
                    return;
                }

                // Trim the extended faces
                TrimExtendedFaces(aFaceExtFaceMap, aPS.Next());
            }
            catch (FailureException)
            {
                // Make sure the warning will be given on the higher level
                myHasAdjacentFaces = true;
                myFaces.Clear();
            }
        }

        public NCollection_Map<Shape> FeatureFacesMap() => myFeatureFacesMap;

        public bool HasAdjacentFaces() => myHasAdjacentFaces;

        public NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> Faces() => myFaces;

        public NCollection_IndexedMap<Shape> Solids() => mySolids;

        // Finds the faces adjacent to the feature and stores them into outgoing map.
        private void FindAdjacentFaces(NCollection_IndexedMap<Shape> theMFAdjacent, ProgressRange theRange)
        {
            // Map the faces of the feature to avoid them in the map of adjacent faces
            Iterator aIt = new(myFeature);
            for (; aIt.More(); aIt.Next())
            {
                myFeatureFacesMap.Add(aIt.Value());
            }
            using var aPSOuter = new ProgressScope(theRange, null, 2);
            // Find faces adjacent to the feature using the connection map
            aIt.Initialize(myFeature);
            using var aPSF = new ProgressScope(aPSOuter.Next(), "Looking for adjacent faces", 1, true);
            for (; aIt.More(); aIt.Next(), aPSF.Next())
            {
                if (!aPSF.More())
                {
                    return;
                }
                Shape aF = aIt.Value();
                Explorer anExpE = new(aF, ShapeType.Edge);
                for (; anExpE.More(); anExpE.Next())
                {
                    Shape aE = anExpE.Current();
                    if (myEFMap!.FindFromKey(aE, out NCollection_List<Shape> pAdjacentFaces))
                    {
                        NCollection_List<Shape>.Iterator itLFA = new(pAdjacentFaces);
                        for (; itLFA.More(); itLFA.Next())
                        {
                            Shape anAF = itLFA.Value();
                            if (!myFeatureFacesMap.Contains(anAF))
                            {
                                theMFAdjacent.Add(anAF);
                            }
                        }
                    }
                }
                // Find solids containing the feature face
                if (myFSMap!.FindFromKey(aF, out NCollection_List<Shape> pLS))
                {
                    NCollection_List<Shape>.Iterator itLS = new(pLS);
                    for (; itLS.More(); itLS.Next())
                    {
                        mySolids.Add(itLS.Value());
                    }
                }
            }

            // Find solids containing the edges of adjacent faces
            int aNbFA = theMFAdjacent.Extent();
            using var aPSS = new ProgressScope(aPSOuter.Next(), "Looking for adjacent solids", aNbFA);
            for (int i = 1; i <= aNbFA; ++i, aPSS.Next())
            {
                if (!aPSS.More())
                {
                    return;
                }
                Explorer anExpEA = new(theMFAdjacent.FindKey(i), ShapeType.Edge);
                for (; anExpEA.More(); anExpEA.Next())
                {
                    // Faces adjacent to the face adjacent to the feature
                    if (myEFMap!.FindFromKey(anExpEA.Current(), out NCollection_List<Shape> pLFAA))
                    {
                        NCollection_List<Shape>.Iterator itLFAA = new(pLFAA);
                        for (; itLFAA.More(); itLFAA.Next())
                        {
                            // Solids containing the faces
                            if (myFSMap!.FindFromKey(itLFAA.Value(), out NCollection_List<Shape> pLS))
                            {
                                NCollection_List<Shape>.Iterator itLS = new(pLS);
                                for (; itLS.More(); itLS.Next())
                                {
                                    mySolids.Add(itLS.Value());
                                }
                            }
                        }
                    }
                }
            }
        }

        // Extends the found adjacent faces and binds them to the original faces.
        private void ExtendAdjacentFaces(NCollection_IndexedMap<Shape> theMFAdjacent,
                                         NCollection_IndexedDataMap<Shape, Shape> theFaceExtFaceMap,
                                         ProgressRange theRange)
        {
            // Get the extension value for the faces - half of the diagonal of bounding box of the feature
            Box aFeatureBox = new();
            BRepBndLib.Add(myFeature, ref aFeatureBox);

            double anExtLength = Math.Sqrt(aFeatureBox.SquareExtent());

            int aNbFA = theMFAdjacent.Extent();
            using var aPS = new ProgressScope(theRange, "Extending adjacent faces", aNbFA);
            for (int i = 1; i <= aNbFA && aPS.More(); ++i, aPS.Next())
            {
                Face aF = ShapeCasts.Face(theMFAdjacent.FindKey(i));
                // Extend the face (the port's unfrozen form: the trimming builds on it)
                BRepLib.InternalExtendFace(aF, anExtLength, true, true, true, true, out Face aFExt);
                theFaceExtFaceMap.Add(aF, aFExt);
                myHistory!.AddModified(aF, aFExt);
            }
        }

        // Trims the extended adjacent faces by intersection with each other
        // and following intersection with the bounds of original faces.
        private void TrimExtendedFaces(NCollection_IndexedDataMap<Shape, Shape> theFaceExtFaceMap,
                                       ProgressRange theRange)
        {
            // Intersect the extended faces first
            BOPAlgo_Builder aGFInter = new();
            // Add faces for intersection
            int aNbF = theFaceExtFaceMap.Extent();
            for (int i = 1; i <= aNbF; ++i)
            {
                aGFInter.AddArgument(theFaceExtFaceMap.FindFromIndex(i));
            }

            aGFInter.SetRunParallel(myRunParallel);

            // Intersection result
            Shape anIntResult;
            using var aPSOuter = new ProgressScope(theRange, null, (aGFInter.Arguments().Extent() > 1) ? 2 : 1);
            if (aGFInter.Arguments().Extent() > 1)
            {
                aGFInter.Perform(aPSOuter.Next());
                if (aGFInter.HasErrors())
                {
                    return;
                }

                anIntResult = aGFInter.InternalShape();

                myHistory!.Merge(aGFInter.InternalHistory());
            }
            else
            {
                anIntResult = aGFInter.Arguments().First();
            }

            // Prepare the EF map of the extended faces after intersection
            // to select from them only boundary edges
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anEFExtMap = new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(anIntResult, ShapeType.Edge, ShapeType.Face, anEFExtMap);

            // Get the splits of the extended faces after intersection
            // and trim them by the edges of the original faces

            // Map the edges of the Feature to avoid them during the trim
            NCollection_IndexedMap<Shape> aFeatureEdgesMap = new(ShapeHasher.Instance);
            Exploration.MapShapes(myFeature, ShapeType.Edge, aFeatureEdgesMap);

            using var aPS = new ProgressScope(aPSOuter.Next(), "Trimming faces", aNbF);
            for (int i = 1; i <= aNbF && aPS.More(); ++i, aPS.Next())
            {
                Face aFOriginal = ShapeCasts.Face(theFaceExtFaceMap.FindKey(i));
                Face aFExt = ShapeCasts.Face(theFaceExtFaceMap.FindFromIndex(i));
                TrimFace(aFExt, aFOriginal, aFeatureEdgesMap, anEFExtMap, aGFInter);
            }
        }

        // Trim the extended faces by the bounds of the original face,
        // except those contained in the feature to remove.
        private void TrimFace(in Face theFExt,
                              in Face theFOriginal,
                              NCollection_IndexedMap<Shape> theFeatureEdgesMap,
                              NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theEFExtMap,
                              BOPAlgo_Builder theGFInter)
        {
            // Map all edges of the extended face, to filter the result of trim
            // from the faces containing these edges
            NCollection_Map<Shape> aMExtEdges = new(ShapeHasher.Instance);
            Explorer anExpE = new(theFExt, ShapeType.Edge);
            for (; anExpE.More(); anExpE.Next())
            {
                Edge aE = ShapeCasts.Edge(anExpE.Current());
                // skip degenerated and seam edges
                if (Tool.Degenerated(aE) || Tool.IsClosed(aE, theFExt))
                {
                    continue;
                }
                NCollection_List<Shape> aLEIm = new();
                TakeModified(aE, theGFInter, aLEIm);
                NCollection_List<Shape>.Iterator itLEIm = new(aLEIm);
                for (; itLEIm.More(); itLEIm.Next())
                {
                    Shape aEIm = itLEIm.Value();
                    if (theEFExtMap.FindFromKey(aEIm).Extent() == 1)
                    {
                        aMExtEdges.Add(aEIm);
                    }
                }
            }

            // Trimming tool
            BOPAlgo_Builder aGFTrim = new();

            // Get the splits of the face and add them for trimming
            NCollection_List<Shape> anExtSplits = new();
            TakeModified(theFExt, theGFInter, anExtSplits);
            aGFTrim.SetArguments(anExtSplits);

            // Add edges of the original faces
            NCollection_Map<Shape> aMEdgesToCheckOri = new(ShapeHasher.Instance);
            anExpE.Init(theFOriginal, ShapeType.Edge);
            for (; anExpE.More(); anExpE.Next())
            {
                Edge aE = ShapeCasts.Edge(anExpE.Current());
                if (!theFeatureEdgesMap.Contains(aE))
                {
                    aGFTrim.AddArgument(aE);
                    if (!Tool.Degenerated(aE) && !Tool.IsClosed(aE, theFOriginal))
                    {
                        if (!aMEdgesToCheckOri.Add(aE))
                        {
                            aMEdgesToCheckOri.Remove(aE);
                        }
                    }
                }
            }

            // Avoid faces intersection
            aGFTrim.SetGlue(BOPAlgo_GlueEnum.BOPAlgo_GlueShift);
            aGFTrim.SetRunParallel(myRunParallel);
            aGFTrim.SetNonDestructive(true);

            aGFTrim.Perform();
            if (aGFTrim.HasErrors())
            {
                return;
            }

            // Get all splits
            Shape aSplits = aGFTrim.InternalShape();
            // Filter the trimmed faces and save the valid ones into result map
            NCollection_List<Shape> aLFTrimmed = new();

            Explorer anExpF = new(aSplits, ShapeType.Face);
            for (; anExpF.More(); anExpF.Next())
            {
                Shape aSp = anExpF.Current();
                anExpE.Init(aSp, ShapeType.Edge);
                for (; anExpE.More(); anExpE.Next())
                {
                    if (aMExtEdges.Contains(anExpE.Current()))
                    {
                        break;
                    }
                }
                if (!anExpE.More())
                {
                    aLFTrimmed.Append(aSp);
                }
            }

            if (aLFTrimmed.Extent() > 1)
            {
                // Chose the correct faces - the ones that contains edges with proper
                // bi-normal direction
                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anEFMap = new(ShapeHasher.Instance);
                NCollection_List<Shape>.Iterator itLF0 = new(aLFTrimmed);
                for (; itLF0.More(); itLF0.Next())
                {
                    Exploration.MapShapesAndAncestors(itLF0.Value(), ShapeType.Edge, ShapeType.Face, anEFMap);
                }

                // Check edges orientations
                NCollection_Map<Shape> aFacesToAvoid = new(ShapeHasher.Instance), aValidFaces = new(ShapeHasher.Instance);
                FindExtraShapes(anEFMap, aMEdgesToCheckOri, aGFTrim, aFacesToAvoid, aValidFaces);

                if (aLFTrimmed.Extent() - aFacesToAvoid.Extent() > 1)
                {
                    // It is possible that the splits are forming the different blocks.
                    // Take only those containing the valid faces.
                    Compound aCF = new();
                    new BrepBuilder().MakeCompound(ref aCF);
                    itLF0.Initialize(aLFTrimmed);
                    for (; itLF0.More(); itLF0.Next())
                    {
                        if (!aFacesToAvoid.Contains(itLF0.Value()))
                        {
                            new BrepBuilder().Add(aCF, itLF0.Value());
                        }
                    }

                    NCollection_List<Shape> aLCB = new();
                    BOPTools_AlgoTools.MakeConnexityBlocks(aCF, ShapeType.Edge, ShapeType.Face, aLCB);
                    if (aLCB.Extent() > 1)
                    {
                        NCollection_List<Shape>.Iterator itLCB = new(aLCB);
                        for (; itLCB.More(); itLCB.Next())
                        {
                            // Check if the block contains any valid faces
                            Shape aCB = itLCB.Value();
                            Iterator itF = new(aCB);
                            for (; itF.More(); itF.Next())
                            {
                                if (aValidFaces.Contains(itF.Value()))
                                {
                                    break;
                                }
                            }
                            if (!itF.More())
                            {
                                // Invalid block
                                for (itF.Initialize(aCB); itF.More(); itF.Next())
                                {
                                    aFacesToAvoid.Add(itF.Value());
                                }
                            }
                        }
                    }
                }

                itLF0.Initialize(aLFTrimmed);
                for (; itLF0.More();)
                {
                    if (aFacesToAvoid.Contains(itLF0.Value()))
                    {
                        aLFTrimmed.Remove(itLF0);
                    }
                    else
                    {
                        itLF0.Next();
                    }
                }
            }
            else if (aLFTrimmed.IsEmpty())
            {
                // Use all splits, including those having the bounds of extended face
                anExpF.ReInit();
                for (; anExpF.More(); anExpF.Next())
                {
                    aLFTrimmed.Append(anExpF.Current());
                }
            }

            if (aLFTrimmed.Extent() != 0)
            {
                // Remove the internal edges and vertices from the faces
                History aCopies = new();
                RemoveInternalWires(aLFTrimmed, null, aCopies);
                myHistory!.Merge(aCopies);

                myFaces.Add(theFOriginal, aLFTrimmed);
            }

            // Update history after intersection of the extended face with bounds
            myHistory!.Merge(aGFTrim.InternalHistory());

            // Update history with all removed shapes
            History aHistRem = new();

            // Map of the result splits
            NCollection_IndexedMap<Shape> aResMap = new(ShapeHasher.Instance);
            NCollection_List<Shape>.Iterator itLF = new(aLFTrimmed);
            for (; itLF.More(); itLF.Next())
            {
                Exploration.MapShapes(itLF.Value(), aResMap);
            }

            NCollection_List<Shape> aLSplits = new();
            aLSplits.Append(aSplits);

            // Update the history with removed shapes
            MakeRemoved(aLSplits, aHistRem, aResMap);
            myHistory.Merge(aHistRem);
        }
    }

    /// <summary>
    /// Removes the features one by one, filling each gap by extension of the adjacent faces.
    /// </summary>
    protected void RemoveFeatures(ProgressRange theRange)
    {
        // For each feature:
        // - Find the faces adjacent to the feature;
        // - Extend the adjacent faces;
        // - Trim the extended faces to fill the gap;
        // - Rebuild the solids with reconstructed adjacent faces
        //   and avoiding the feature faces.

        using var aPSOuter = new ProgressScope(theRange, "Removing features", 2);

        // Make Edge-Face connection map of the input
        // shape to find faces adjacent to the feature
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anEFMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(myInputShape, ShapeType.Edge, ShapeType.Face, anEFMap);

        // Make Face-Solid connection map to find the solids
        // participating in the removal of each feature
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anFSMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(myInputShape, ShapeType.Face, ShapeType.Solid, anFSMap);

        // Tool for reconstruction of the faces adjacent to the feature
        // in parallel threads if necessary.
        NCollection_DynamicArray<FillGap> aVFG = new();
        // Fill the vector
        NCollection_List<Shape>.Iterator itF = new(myFeatures);
        for (; itF.More(); itF.Next())
        {
            Shape aFeature = itF.Value();
            FillGap aFG = aVFG.Append(new FillGap());
            aFG.SetFeature(aFeature);
            aFG.SetEFConnectionMap(anEFMap);
            aFG.SetFSConnectionMap(anFSMap);
            aFG.SetRunParallel(myRunParallel);
        }

        int aNbF = aVFG.Length();
        using var aPS = new ProgressScope(aPSOuter.Next(), "Filling gaps", aNbF);
        for (int i = 0; i < aNbF; ++i)
        {
            FillGap aFG = aVFG.Value(i);
            aFG.SetRange(aPS.Next());
        }
        // Perform the reconstruction of the adjacent faces
        BOPTools_Parallel.Perform(myRunParallel, aVFG);
        if (UserBreak(aPSOuter))
        {
            return;
        }
        // Even if the history is not requested, it is necessary to track:
        // - The solids modification after each feature removal to find
        //   the necessary solids to rebuild on the next step.
        // - The faces modification after each feature removal to find the
        //   splits of the adjacent and feature faces for the next steps.
        myHistory ??= new History();

        // Remove the features one by one.
        // It will allow removing the features even if there were
        // some problems with removal of the previous features.
        using var aPSLoop = new ProgressScope(aPSOuter.Next(), "Removing features one by one", aNbF);
        for (int i = 0; i < aNbF; ++i)
        {
            if (UserBreak(aPSLoop))
            {
                return;
            }
            FillGap aFG = aVFG.Value(i);

            // No need to fill the history for solids if the history is not
            // requested and the current feature is the last one.
            bool isSolidsHistoryNeeded = HasHistory() || (i < (aNbF - 1));

            // Perform removal of the single feature
            RemoveFeature(aFG.Feature(),
                          aFG.Solids(),
                          aFG.FeatureFacesMap(),
                          aFG.HasAdjacentFaces(),
                          aFG.Faces(),
                          aFG.History(),
                          isSolidsHistoryNeeded,
                          aPSLoop.Next());
        }
    }

    /// <summary>Removes a single feature from the shape.</summary>
    /// <param name="theFeature">The feature to remove.</param>
    /// <param name="theSolids">The solids to be reconstructed after the feature removal.</param>
    /// <param name="theFeatureFacesMap">The map of the feature's faces.</param>
    /// <param name="theHasAdjacentFaces">Whether adjacent faces have been found for the feature.</param>
    /// <param name="theAdjFaces">The reconstructed adjacent faces covering the feature.</param>
    /// <param name="theAdjFacesHistory">The history of the adjacent faces' reconstruction.</param>
    /// <param name="theSolidsHistoryNeeded">Whether the history of the solids' modifications is tracked.</param>
    /// <param name="theRange">The progress range.</param>
    protected void RemoveFeature(in Shape theFeature,
                                 NCollection_IndexedMap<Shape> theSolids,
                                 NCollection_Map<Shape> theFeatureFacesMap,
                                 bool theHasAdjacentFaces,
                                 NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theAdjFaces,
                                 History? theAdjFacesHistory,
                                 bool theSolidsHistoryNeeded,
                                 ProgressRange theRange)
    {
        bool bFuseShapes = true;
        int aNbAF = theAdjFaces.Extent();
        if (aNbAF == 0)
        {
            if (theHasAdjacentFaces)
            {
                // The adjacent faces have been found for the feature,
                // but something went wrong during their rebuilding.
                // Add error
                AddWarning(new BOPAlgo_AlertUnableToRemoveTheFeature(theFeature));
                return;
            }

            // No adjacent faces have been found for the feature.
            // Most likely, the intention is to remove some internal part of the shape.
            // We just have to rebuild the shape without the feature, no fuse is necessary.
            bFuseShapes = false;
        }

        using var aPS = new ProgressScope(theRange, null, 100);
        // Rebuild the shape using MakerVolume algorithm avoiding the faces of the
        // feature and replacing the adjacent faces with their images

        BrepBuilder aBB = new();

        // From the faces of input shape build map of faces with which the result will be validated:
        // - all non-internal faces of the input shape (except for adjacent and feature faces)
        //   must have some trace in the result solids;
        // - all adjacent faces (if any) must have some trace in the result solids.
        NCollection_IndexedMap<Shape> aFacesToBeKept = new(ShapeHasher.Instance);
        // Build also the map of faces to compare orientation of the original and result faces
        NCollection_Map<Shape> aFacesToCheckOri = new(ShapeHasher.Instance);

        // Collect list of internal entities of the input shape to be avoided in result
        // and to make them removed in the history.
        NCollection_List<Shape> anInternalShapes = new();

        // In the feature removal will participate only the solids connected to the feature
        // or the faces adjacent to the feature.

        // Solids to be rebuilt  after the feature removal
        NCollection_IndexedMap<Shape> aSolidsToRebuild = new(ShapeHasher.Instance);
        // Find faces shared between solids
        NCollection_List<Shape> aSharedFaces = new();
        // Solids to be avoided in the feature removal and added into result directly
        NCollection_List<Shape> anUnTouchedSolids = new();

        // Prepare to the feature removal - fill all necessary containers
        GetOriginalFaces(myShape,
                         theSolids,
                         theFeatureFacesMap,
                         theAdjFaces,
                         myHistory!,
                         aFacesToBeKept,
                         anInternalShapes,
                         aFacesToCheckOri,
                         aSolidsToRebuild,
                         aSharedFaces,
                         anUnTouchedSolids);

        aPS.Next(3);
        // To avoid excessive intersection of the faces collect the faces
        // of the input shape into a compound
        Compound anOrigF = new();
        aBB.MakeCompound(ref anOrigF);
        int aNbFK = aFacesToBeKept.Extent();
        for (int i = 1; i <= aNbFK; ++i)
        {
            aBB.Add(anOrigF, aFacesToBeKept.FindKey(i));
        }

        // Tool for solids reconstruction
        BOPAlgo_MakerVolume aMV = new();
        aMV.SetRunParallel(myRunParallel);
        aMV.SetAvoidInternalShapes(true);
        aMV.SetIntersect(bFuseShapes);
        aMV.SetNonDestructive(true);
        // Add faces of the input shape
        aMV.AddArgument(anOrigF);

        // Add reconstructed adjacent faces
        for (int i = 1; i <= aNbAF; ++i)
        {
            NCollection_List<Shape> aLFA = theAdjFaces.FindFromIndex(i);
            if (aLFA.Extent() == 1)
            {
                Shape aFA = aLFA.First();
                aMV.AddArgument(aFA);
                aFacesToBeKept.Add(aFA);
            }
            else
            {
                // To avoid intersection among the images, collect them into compound
                Compound anAdjF = new();
                aBB.MakeCompound(ref anAdjF);
                NCollection_List<Shape>.Iterator itLFA = new(aLFA);
                for (; itLFA.More(); itLFA.Next())
                {
                    aBB.Add(anAdjF, itLFA.Value());
                }

                aMV.AddArgument(anAdjF);
                aFacesToBeKept.Add(anAdjF);
            }

            if (HasHistory())
            {
                // Look for internal edges in the original adjacent faces
                Shape aFOr = theAdjFaces.FindKey(i);
                FindInternals(aFOr, anInternalShapes);
            }
        }

        // Build solids
        aMV.Perform(aPS.Next(90));
        if (aMV.HasErrors())
        {
            // Add warning for the feature
            AddWarning(new BOPAlgo_AlertUnableToRemoveTheFeature(theFeature));
            return;
        }

        // Result of MV operation
        Shape aSolids = aMV.InternalShape();
        Explorer anExpS = new(aSolids, ShapeType.Solid);
        if (!anExpS.More())
        {
            // No solids have been built - add warning for the feature
            AddWarning(new BOPAlgo_AlertUnableToRemoveTheFeature(theFeature));
            return;
        }

        // Now, it is necessary to:
        // - Validate the result by checking faces of the map <aFacesToBeKept>
        //   to have some parts kept in the resulting solids;
        // - Remove the solids possibly filling the holes in the original shapes;
        // - Update history with the history of MakerVolume algorithm.

        // Splits of adjacent faces from previous runs
        NCollection_Map<Shape> anAdjFacesSplits = new(ShapeHasher.Instance);
        for (int i = 1; i <= aNbAF; ++i)
        {
            if (!aPS.More())
            {
                return;
            }
            Shape aF = theAdjFaces.FindKey(i);
            IReadOnlyList<Shape> aLFIm = myHistory!.Modified(aF);
            if (aLFIm.Count == 0)
            {
                anAdjFacesSplits.Add(aF);
            }
            else
            {
                foreach (Shape aFIm in aLFIm)
                {
                    anAdjFacesSplits.Add(aFIm);
                }
            }
        }

        // Validate the result
        bool bValid = true;
        aNbFK = aFacesToBeKept.Extent();
        for (int i = 1; i <= aNbFK && bValid; ++i)
        {
            Shape aS = aFacesToBeKept.FindKey(i);
            if (anAdjFacesSplits.Contains(aS))
            {
                continue;
            }
            if (!aPS.More())
            {
                return;
            }
            Explorer anExpF = new(aS, ShapeType.Face);
            for (; anExpF.More(); anExpF.Next())
            {
                Shape aF = anExpF.Current();
                if (!aMV.IsDeleted(aF))
                {
                    break;
                }
            }
            bValid = anExpF.More();
        }

        if (!bValid)
        {
            // Add warning for the feature
            AddWarning(new BOPAlgo_AlertUnableToRemoveTheFeature(theFeature));
            return;
        }

        // It is possible that the result of MakerVolume operation contains
        // some extra solids. Get only correct solids.
        NCollection_List<Shape> aLSRes = new();
        // Remember the removed shapes
        NCollection_List<Shape> aRemovedShapes = new();
        GetValidSolids(aMV,
                       aFacesToCheckOri,
                       aSharedFaces,
                       anOrigF,
                       theSolids.Extent(),
                       aLSRes,
                       aRemovedShapes);

        if (aLSRes.Extent() != theSolids.Extent())
        {
            // Add warning for the feature
            AddWarning(new BOPAlgo_AlertUnableToRemoveTheFeature(theFeature));
            return;
        }
        aPS.Next(3);
        // Remove internal wires from the faces, possibly appeared after intersection
        History aCopies = new();
        RemoveInternalWires(aLSRes, anInternalShapes, aCopies);

        // Update history with:
        // History of adjacent faces reconstruction
        myHistory!.Merge(theAdjFacesHistory);
        // History of intersection
        myHistory.Merge(aMV.InternalHistory());
        // OCCT.NET: frozen. The input faces RemoveInternalWires replaced by copies (OCCT edits them in place)
        myHistory.Merge(aCopies);

        if (HasHistory())
        {
            // Map the result to check if the shape is removed
            NCollection_IndexedMap<Shape> aMSRes = new(ShapeHasher.Instance);
            NCollection_List<Shape>.Iterator itLS0 = new(aLSRes);
            for (; itLS0.More(); itLS0.Next())
            {
                Exploration.MapShapes(itLS0.Value(), aMSRes);
            }

            // Remove internal shapes and extra faces
            History aRemHist = new();
            anInternalShapes.Append(aRemovedShapes);
            MakeRemoved(anInternalShapes, aRemHist, aMSRes);
            myHistory.Merge(aRemHist);
        }
        aPS.Next(3);
        // Fill the history for the solids
        if (theSolidsHistoryNeeded)
        {
            History aSolidsHistory = new();
            FillSolidsHistory(aSolidsToRebuild, aLSRes, theAdjFaces, aMV, aSolidsHistory);
            myHistory.Merge(aSolidsHistory);
        }

        Compound aCRes = new();
        aBB.MakeCompound(ref aCRes);
        // Add reconstructed solids
        NCollection_List<Shape>.Iterator itLS = new(aLSRes);
        for (; itLS.More(); itLS.Next())
        {
            aBB.Add(aCRes, itLS.Value());
        }

        // Add unmodified solids
        itLS.Initialize(anUnTouchedSolids);
        for (; itLS.More(); itLS.Next())
        {
            aBB.Add(aCRes, itLS.Value());
        }

        // Save the result
        myShape = aCRes;
    }

    /// <summary>Updates the history with the removed features.</summary>
    protected void UpdateHistory(ProgressRange theRange)
    {
        if (!HasHistory())
        {
            return;
        }

        // Map the result
        myMapShape.Clear();
        Exploration.MapShapes(myShape, myMapShape);

        // Update the history
        History aHistory = new();

        int aNbS = myInputsMap.Extent();
        using var aPS = new ProgressScope(theRange, "Updating history", aNbS);
        for (int i = 1; i <= aNbS; ++i, aPS.Next())
        {
            Shape aS = myInputsMap.FindKey(i);
            if (!global::Kerf.Engine.Topology.History.IsSupportedType(aS))
            {
                continue;
            }

            if (myHistory!.IsRemoved(aS))
            {
                continue;
            }

            if (UserBreak(aPS))
            {
                return;
            }
            // Check if the shape has any trace in the result
            IReadOnlyList<Shape> aLSIm = myHistory.Modified(aS);
            if (aLSIm.Count == 0)
            {
                if (!myMapShape.Contains(aS))
                {
                    aHistory.Remove(aS);
                }
            }

            foreach (Shape aSIm in aLSIm)
            {
                if (!myMapShape.Contains(aSIm))
                {
                    aHistory.Remove(aSIm);
                }
            }
        }

        myHistory!.Merge(aHistory);
    }

    /// <summary>Simplifies the result, removing the extra edges and vertices created by the operation.</summary>
    protected void SimplifyResult(ProgressRange theRange)
    {
        if (myShape.IsSame(myInputShape))
        {
            return;
        }
        using var aPSOuter = new ProgressScope(theRange, "Simplifyingthe result", 2);
        ShapeUpgrade_UnifySameDomain aSDTool = new();
        aSDTool.Initialize(myShape, true, true);
        // Do not allow producing internal edges
        aSDTool.AllowInternalEdges(false);
        // Avoid removal of the input edges and vertices
        if (myMapShape.IsEmpty())
        {
            Exploration.MapShapes(myShape, myMapShape);
        }

        int aNbS = myInputsMap.Extent();
        using var aPS = new ProgressScope(aPSOuter.Next(), null, aNbS);
        for (int i = 1; i <= aNbS; ++i, aPS.Next())
        {
            if (UserBreak(aPS))
            {
                return;
            }
            if (myMapShape.Contains(myInputsMap.FindKey(i)))
            {
                aSDTool.KeepShape(myInputsMap.FindKey(i));
            }
        }

        // Perform unification (on the result, which shares the input's frozen sub-shapes: UnifySameDomain copies at
        // its entry, decision 766, and its history answers for the result's own sub-shapes)
        aSDTool.Build();
        aPSOuter.Next();
        myShape = aSDTool.InternalShape();
        if (HasHistory())
        {
            myHistory!.Merge(aSDTool.InternalHistory());
        }
    }

    /// <summary>Post treatment: restores the type of the input shape.</summary>
    protected void PostTreat()
    {
        ShapeType anInputType = myInputShape.ShapeType();
        ShapeType aResType = myShape.ShapeType();
        if (aResType == anInputType)
        {
            return;
        }

        Explorer anExpS = new(myShape, ShapeType.Solid);

        if (anInputType == ShapeType.Solid)
        {
            myShape = anExpS.Current();
            return;
        }

        Shape aRes;
        if (anInputType == ShapeType.Compound)
        {
            Compound aC = new();
            new BrepBuilder().MakeCompound(ref aC);
            aRes = aC;
        }
        else
        {
            CompSolid aCS = new();
            new BrepBuilder().MakeCompSolid(ref aCS);
            aRes = aCS;
        }

        for (; anExpS.More(); anExpS.Next())
        {
            new BrepBuilder().Add(aRes, anExpS.Current());
        }

        myShape = aRes;
    }

    //=======================================================================
    // static methods definition
    //=======================================================================

    // Makes the shapes in the list removed in the history. Keeps the shapes contained in the map.
    private static void MakeRemoved(NCollection_List<Shape> theShapes,
                                    History theHistory,
                                    NCollection_IndexedMap<Shape>? theKeepShapes = null)
    {
        NCollection_IndexedMap<Shape> aShapesMap = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator it = new(theShapes);
        for (; it.More(); it.Next())
        {
            Exploration.MapShapes(it.Value(), aShapesMap);
        }

        int aNbS = aShapesMap.Extent();
        for (int i = 1; i <= aNbS; ++i)
        {
            Shape aS = aShapesMap.FindKey(i);
            if ((theKeepShapes is null || !theKeepShapes.Contains(aS)) && global::Kerf.Engine.Topology.History.IsSupportedType(aS))
            {
                theHistory.Remove(aS);
            }
        }
    }

    // Looks for internal shapes inside the face or solid.
    private static void FindInternals(in Shape theS, NCollection_List<Shape> theLInt)
    {
        Iterator itS = new(theS);
        for (; itS.More(); itS.Next())
        {
            Shape aSS = itS.Value();
            if (aSS.Orientation() == Internal)
            {
                theLInt.Append(aSS);
            }
            else
            {
                Iterator itSS = new(aSS);
                for (; itSS.More(); itSS.Next())
                {
                    if (itSS.Value().Orientation() == Internal)
                    {
                        theLInt.Append(aSS);
                        break;
                    }
                }
            }
        }
    }

    // Removes internal wires from the faces.
    // OCCT.NET: frozen. OCCT removes the wires from the face itself (BRep_Builder::Remove on the face reached through
    // the explorer), so every shape sharing the face sees the change. A frozen face (one of the input's) can't be
    // changed: it is replaced by a copy without those wires, in the containers of theShapes that hold it (rebuilt in
    // their order) or in the list itself, and the replacement is recorded in theCopies (Modified).
    private static void RemoveInternalWires(NCollection_List<Shape> theShapes,
                                            NCollection_List<Shape>? theRemoved,
                                            History theCopies)
    {
        NCollection_DataMap<Shape, Shape> aCopies = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itLS = new(theShapes);
        for (; itLS.More(); itLS.Next())
        {
            Shape aShape = itLS.Value();
            Explorer anExpF = new(aShape, ShapeType.Face);
            for (; anExpF.More(); anExpF.Next())
            {
                Shape aF = anExpF.Current();
                NCollection_List<Shape> aLWToRemove = new();
                FindInternals(aF, aLWToRemove);
                if (aLWToRemove.Extent() != 0)
                {
                    if (aF.TShape()!.IsFrozen)
                    {
                        if (aCopies.IsBound(aF))
                        {
                            continue;
                        }
                        // the copy: the face's TShape emptied, with the wires that stay
                        Shape aFCopy = aF.EmptyCopied();
                        Iterator itW = new(aF);
                        for (; itW.More(); itW.Next())
                        {
                            if (!aLWToRemove.Contains(itW.Value()))
                            {
                                new BrepBuilder().Add(aFCopy, itW.Value());
                            }
                        }
                        NCollection_List<Shape>.Iterator itR0 = new(aLWToRemove);
                        for (; itR0.More(); itR0.Next())
                        {
                            theRemoved?.Append(itR0.Value());
                        }
                        aCopies.Bind(aF, aFCopy);
                        theCopies.AddModified(aF, aFCopy);
                        continue;
                    }
                    aF.Free(true);
                    NCollection_List<Shape>.Iterator itR = new(aLWToRemove);
                    for (; itR.More(); itR.Next())
                    {
                        theRemoved?.Append(itR.Value());
                        new BrepBuilder().Remove(aF, itR.Value());
                    }
                    aF.Free(false);
                }
            }
        }

        if (aCopies.IsEmpty())
        {
            return;
        }
        // Put the copies in place of the frozen faces
        for (itLS.Initialize(theShapes); itLS.More(); itLS.Next())
        {
            Shape aShape = itLS.Value();
            if (aShape.ShapeType() == ShapeType.Face)
            {
                if (aCopies.Find(aShape, out Shape aCopy))
                {
                    itLS.ChangeValue() = WithTShape(aShape, aCopy);
                }
                continue;
            }
            ReplaceFaces(aShape, aCopies);
        }
    }

    // The shape with the TShape of theTShapeOf, keeping its own location and orientation.
    private static Shape WithTShape(in Shape theShape, in Shape theTShapeOf)
    {
        Shape aS = theShape;
        aS.TShape(theTShapeOf.TShape());
        return aS;
    }

    // Replaces the faces bound in theCopies by their copies in the (unfrozen) containers of theShape, keeping the
    // order of every container's children.
    private static bool ReplaceFaces(in Shape theShape, NCollection_DataMap<Shape, Shape> theCopies)
    {
        if (theShape.ShapeType() == ShapeType.Face)
        {
            return false;
        }
        List<Shape> aChildren = [];
        bool aChanged = false;
        for (Iterator it = new(theShape, false, false); it.More(); it.Next())
        {
            Shape aChild = it.Value();
            if (aChild.ShapeType() == ShapeType.Face && theCopies.Find(aChild, out Shape aCopy))
            {
                aChild = WithTShape(aChild, aCopy);
                aChanged = true;
            }
            else if (!aChild.TShape()!.IsFrozen && ReplaceFaces(aChild, theCopies))
            {
                aChanged = true;
            }
            aChildren.Add(aChild);
        }
        if (!aChanged)
        {
            return false;
        }
        // Rebuild the container's list in its order (TShape-relative children: no location or orientation composed)
        Shape aBare = theShape;
        aBare.Location(new Placement(), false);
        aBare.Orientation(Forward);
        bool aFree = aBare.Free();
        aBare.Free(true);
        BrepBuilder aBB = new();
        List<Shape> anOld = [];
        for (Iterator it = new(aBare, false, false); it.More(); it.Next())
        {
            anOld.Add(it.Value());
        }
        foreach (Shape aChild in anOld)
        {
            aBB.Remove(aBare, aChild);
        }
        foreach (Shape aChild in aChildren)
        {
            aBB.Add(aBare, aChild);
        }
        aBare.Free(aFree);
        return true;
    }

    // Get original faces from my face
    private static void GetOriginalFaces(in Shape theShape,
                                         NCollection_IndexedMap<Shape> theSolids,
                                         NCollection_Map<Shape> theFeatureFacesMap,
                                         NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theAdjFaces,
                                         History theHistory,
                                         NCollection_IndexedMap<Shape> theFacesToBeKept,
                                         NCollection_List<Shape> theInternalShapes,
                                         NCollection_Map<Shape> theFacesToCheckOri,
                                         NCollection_IndexedMap<Shape> theSolidsToRebuild,
                                         NCollection_List<Shape> theSharedFaces,
                                         NCollection_List<Shape> theUnTouchedSolids)
    {
        // Use only solids which has to be reconstructed by the feature.
        // All other solids should be avoided in the feature removal and added
        // into result directly.

        // Update solids after removal of previous features
        int aNbSols = theSolids.Extent();
        for (int i = 1; i <= aNbSols; ++i)
        {
            Shape aSol = theSolids.FindKey(i);
            IReadOnlyList<Shape> aLFIm = theHistory.Modified(aSol);
            if (aLFIm.Count == 0)
            {
                theSolidsToRebuild.Add(aSol);
            }
            else
            {
                theSolidsToRebuild.Add(aLFIm[0]);
            }
        }

        // Splits of the feature faces
        NCollection_Map<Shape> aFeatureFacesSplits = new(ShapeHasher.Instance);
        foreach (Shape aF in theFeatureFacesMap)
        {
            IReadOnlyList<Shape> aLFIm = theHistory.Modified(aF);
            if (aLFIm.Count == 0)
            {
                aFeatureFacesSplits.Add(aF);
            }
            else
            {
                foreach (Shape aFIm in aLFIm)
                {
                    aFeatureFacesSplits.Add(aFIm);
                }
            }
        }

        Explorer anExpS = new(theShape, ShapeType.Solid);
        for (; anExpS.More(); anExpS.Next())
        {
            Shape aSol = anExpS.Current();

            // Check if the solid has to be reconstructed
            if (!theSolidsToRebuild.Contains(aSol))
            {
                // untouched solid
                theUnTouchedSolids.Append(aSol);
                continue;
            }

            Iterator itSh = new(aSol);
            for (; itSh.More(); itSh.Next())
            {
                Shape aSh = itSh.Value();
                if (aSh.ShapeType() != ShapeType.Shell)
                {
                    theInternalShapes.Append(aSh);
                    continue;
                }

                Iterator itF = new(aSh);
                for (; itF.More(); itF.Next())
                {
                    Shape aF = itF.Value();
                    // Avoid the feature faces
                    if (aFeatureFacesSplits.Contains(aF))
                    {
                        continue;
                    }

                    // Avoid the adjacent faces
                    if (theAdjFaces.Contains(aF))
                    {
                        continue;
                    }

                    if (aF.Orientation() != Internal)
                    {
                        theFacesToBeKept.Add(aF);

                        if (!theFacesToCheckOri.Add(aF))
                        {
                            theFacesToCheckOri.Remove(aF);
                            theSharedFaces.Append(aF);
                        }
                    }
                    else
                    {
                        theInternalShapes.Append(aSh);
                    }
                }
            }
        }
    }

    // Find the shape in the other shape
    private static void FindShape(in Shape theSWhat, in Shape theSWhere, ref Shape theSFound)
    {
        Explorer anExp = new(theSWhere, theSWhat.ShapeType());
        for (; anExp.More(); anExp.Next())
        {
            Shape aS = anExp.Current();
            if (aS.IsSame(theSWhat))
            {
                theSFound = aS;
                break;
            }
        }
    }

    // Checks the validity of the solids and keeps only valid ones
    private static void GetValidSolids(BOPAlgo_MakerVolume theMV,
                                       NCollection_Map<Shape> theFacesToCheckOri,
                                       NCollection_List<Shape> aSharedFaces,
                                       in Shape theOrigFaces,
                                       int theNbSol,
                                       NCollection_List<Shape> theLSRes,
                                       NCollection_List<Shape> theRemovedShapes)
    {
        Explorer anExpS = new(theMV.InternalShape(), ShapeType.Solid);
        for (; anExpS.More(); anExpS.Next())
        {
            theLSRes.Append(anExpS.Current());
        }

        if (theLSRes.Extent() > theNbSol)
        {
            // Find Solids filling the holes in the initial shape
            NCollection_Map<Shape> aSolidsToAvoid = new(ShapeHasher.Instance);
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aFSMap = new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(theMV.InternalShape(), ShapeType.Face, ShapeType.Solid, aFSMap);
            FindExtraShapes(aFSMap, theFacesToCheckOri, theMV, aSolidsToAvoid);

            NCollection_List<Shape>.Iterator itLS = new(theLSRes);
            for (; itLS.More();)
            {
                if (aSolidsToAvoid.Contains(itLS.Value()))
                {
                    theLSRes.Remove(itLS);
                }
                else
                {
                    itLS.Next();
                }
            }
        }

        if (theLSRes.Extent() > theNbSol)
        {
            // Check if the splits of the adjacent faces split the solids
            AvoidExtraSharedFaces(theLSRes, aSharedFaces, theMV, theRemovedShapes);
        }

        if (theLSRes.Extent() > theNbSol)
        {
            // Remove solids containing only the adjacent faces
            NCollection_Map<Shape> anOrigFacesRes = new(ShapeHasher.Instance);
            Explorer anExpF = new(theOrigFaces, ShapeType.Face);
            for (; anExpF.More(); anExpF.Next())
            {
                TakeModified(anExpF.Current(), theMV, anOrigFacesRes);
            }

            NCollection_List<Shape>.Iterator itLS = new(theLSRes);
            for (; itLS.More();)
            {
                anExpF.Init(itLS.Value(), ShapeType.Face);
                for (; anExpF.More(); anExpF.Next())
                {
                    if (anOrigFacesRes.Contains(anExpF.Current()))
                    {
                        break;
                    }
                }
                if (!anExpF.More())
                {
                    theRemovedShapes.Append(itLS.Value());
                    theLSRes.Remove(itLS);
                }
                else
                {
                    itLS.Next();
                }
            }
        }
    }

    // Find shapes possibly filling the holes in the original shape.
    // The checks are made in the iteration order of theShapesToCheckOri (native: address order; the port: insertion
    // order), but the outcome doesn't depend on it: a shape is avoided only if no check found it of the same
    // orientation.
    private static void FindExtraShapes(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theConnectionMap,
                                        NCollection_Map<Shape> theShapesToCheckOri,
                                        BOPAlgo_Builder theBuilder,
                                        NCollection_Map<Shape> theShapesToAvoid,
                                        NCollection_Map<Shape>? theValidShapes = null)
    {
        IntTools_Context? aCtx = theBuilder.Context();
        NCollection_Map<Shape> pValidShapes = theValidShapes ?? new NCollection_Map<Shape>(ShapeHasher.Instance);
        foreach (Shape aSToCheckOri in theShapesToCheckOri)
        {
            // Check modification of the shape during intersection
            NCollection_List<Shape> aLSIm = new();
            TakeModified(aSToCheckOri, theBuilder, aLSIm);

            NCollection_List<Shape>.Iterator itLSIm = new(aLSIm);
            for (; itLSIm.More(); itLSIm.Next())
            {
                Shape aSIm = itLSIm.Value();

                if (!theConnectionMap.FindFromKey(aSIm, out NCollection_List<Shape> pShapesToValidate))
                {
                    continue;
                }

                NCollection_List<Shape>.Iterator itSV = new(pShapesToValidate);
                for (; itSV.More(); itSV.Next())
                {
                    Shape aShapeToValidate = itSV.Value();
                    if (pValidShapes.Contains(aShapeToValidate))
                    {
                        continue;
                    }

                    // OCCT finds the image into a TopoDS_Face, whatever its type (the edges of TrimFace's check
                    // are copied into it: TopoDS_Shape's assignment)
                    Shape aSInShape = new();
                    FindShape(aSIm, aShapeToValidate, ref aSInShape);

                    bool bSameOri = !BOPTools_AlgoTools.IsSplitToReverse(aSInShape, aSToCheckOri, aCtx);

                    if (bSameOri)
                    {
                        pValidShapes.Add(aShapeToValidate);
                    }
                    else
                    {
                        theShapesToAvoid.Add(aShapeToValidate);
                    }
                }
            }
        }

        foreach (Shape aValid in pValidShapes)
        {
            theShapesToAvoid.Remove(aValid);
        }
    }

    // Looks for the extra faces splitting the solids
    private static void AvoidExtraSharedFaces(NCollection_List<Shape> theLSolids,
                                              NCollection_List<Shape> theLFSharedToAvoid,
                                              BOPAlgo_Builder theBuilder,
                                              NCollection_List<Shape> theExtraFaces)
    {
        // Get all splits of shared faces to avoid in the check
        NCollection_Map<Shape> aMFSharedSp = new(ShapeHasher.Instance);
        {
            NCollection_List<Shape>.Iterator itLFS = new(theLFSharedToAvoid);
            for (; itLFS.More(); itLFS.Next())
            {
                TakeModified(itLFS.Value(), theBuilder, aMFSharedSp);
            }
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aFSMap = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itLS = new(theLSolids);
        for (; itLS.More(); itLS.Next())
        {
            Exploration.MapShapesAndAncestors(itLS.Value(), ShapeType.Face, ShapeType.Solid, aFSMap);
        }

        NCollection_List<Shape> anExtraFaces = new();
        NCollection_List<Shape> aLFArguments = new();
        itLS.Initialize(theLSolids);
        for (; itLS.More(); itLS.Next())
        {
            Shape aSol = itLS.Value();
            Explorer anExpF = new(aSol, ShapeType.Face);
            for (; anExpF.More(); anExpF.Next())
            {
                Shape aF = anExpF.Current();
                NCollection_List<Shape> aLSol = aFSMap.FindFromKey(aF);
                if (aLSol.Extent() != 2 || aMFSharedSp.Contains(aF))
                {
                    aLFArguments.Append(aF);
                }
                else
                {
                    anExtraFaces.Append(aF);
                }
            }
        }

        if (anExtraFaces.IsEmpty())
        {
            return;
        }

        // Rebuild the solids avoiding the extra faces
        BOPAlgo_BuilderSolid aBS = new();
        aBS.SetAvoidInternalShapes(true);
        aBS.SetShapes(aLFArguments);
        aBS.Perform();
        if (aBS.HasErrors())
        {
            return;
        }

        theLSolids.Assign(aBS.Areas());
        theExtraFaces.Append(anExtraFaces);
    }

    // Fills the history of solids modifications
    private static void FillSolidsHistory(NCollection_IndexedMap<Shape> theSolIn,
                                          NCollection_List<Shape> theSolidsOut,
                                          NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theAdjFaces,
                                          BOPAlgo_Builder theBuilder,
                                          History theSolidsHistory)
    {
        int aNbS = theSolIn.Extent();
        for (int i = 1; i <= aNbS; ++i)
        {
            Shape aSolIn = theSolIn.FindKey(i);

            Shape aSolOut = new();
            FindSolid(aSolIn, theSolidsOut, theAdjFaces, theBuilder, ref aSolOut);

            if (aSolOut.IsNull)
            {
                theSolidsHistory.Remove(aSolIn);
                continue;
            }

            // Check if the solid has really been modified
            BOPTools_Set aSTIn = new(), aSTOut = new();
            aSTIn.Add(aSolIn, ShapeType.Face);
            aSTOut.Add(aSolOut, ShapeType.Face);
            if (aSTIn.IsEqual(aSTOut))
            {
                // The solids are the same. Replace the resulting solid in the result list
                // with the initial solid.
                NCollection_List<Shape>.Iterator itLS = new(theSolidsOut);
                for (; itLS.More(); itLS.Next())
                {
                    if (itLS.Value().IsSame(aSolOut))
                    {
                        theSolidsOut.InsertBefore(aSolIn, itLS);
                        theSolidsOut.Remove(itLS);
                        break;
                    }
                }
            }
            else
            {
                theSolidsHistory.AddModified(aSolIn, aSolOut);
            }
        }
    }

    // Stores the modified object into the list
    private static void TakeModified(in Shape theS, BOPAlgo_Builder theBuilder, NCollection_List<Shape> theList)
    {
        IReadOnlyList<Shape> aModified = theBuilder.Modified(theS);
        if (aModified.Count == 0 && !theBuilder.IsDeleted(theS))
        {
            theList.Append(theS);
        }
        else
        {
            foreach (Shape aM in aModified)
            {
                theList.Append(aM);
            }
        }
    }

    // Stores the modified object into the map
    private static void TakeModified(in Shape theS, BOPAlgo_Builder theBuilder, NCollection_Map<Shape> theMap)
    {
        IReadOnlyList<Shape> aModified = theBuilder.Modified(theS);
        if (aModified.Count == 0 && !theBuilder.IsDeleted(theS))
        {
            theMap.Add(theS);
        }
        else
        {
            foreach (Shape aM in aModified)
            {
                theMap.Add(aM);
            }
        }
    }

    // Looks for the image of the solid in the list of resulting solids
    private static void FindSolid(in Shape theSolIn,
                                  NCollection_List<Shape> theSolidsRes,
                                  NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> theAdjFaces,
                                  BOPAlgo_Builder theBuilder,
                                  ref Shape theSolOut)
    {
        IntTools_Context? aCtx = theBuilder.Context();

        // Take the face in the IN solid, and find it in the OUT list
        Explorer anExpF = new(theSolIn, ShapeType.Face);
        for (; anExpF.More(); anExpF.Next())
        {
            Shape aFS = anExpF.Current();
            // Images of the face
            NCollection_Map<Shape> aMFSIm = new(ShapeHasher.Instance);
            if (theAdjFaces.FindFromKey(aFS, out NCollection_List<Shape> pLFA))
            {
                NCollection_List<Shape>.Iterator itLFA = new(pLFA);
                for (; itLFA.More(); itLFA.Next())
                {
                    TakeModified(itLFA.Value(), theBuilder, aMFSIm);
                }
            }
            else
            {
                TakeModified(aFS, theBuilder, aMFSIm);
            }

            // Find any of these faces in the list of solids
            NCollection_List<Shape>.Iterator itLS = new(theSolidsRes);
            for (; itLS.More(); itLS.Next())
            {
                Shape aSol = itLS.Value();
                Explorer anExpFOut = new(aSol, ShapeType.Face);
                for (; anExpFOut.More(); anExpFOut.Next())
                {
                    Shape aF = anExpFOut.Current();
                    if (aMFSIm.Contains(aF))
                    {
                        // check orientations
                        if (!BOPTools_AlgoTools.IsSplitToReverse(aF, aFS, aCtx))
                        {
                            theSolOut = aSol;
                            return;
                        }
                    }
                }
            }
        }
    }
}
