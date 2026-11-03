// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_RemoveFeatures.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_RemoveFeatures.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2018-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>
/// Removes features (connected sets of faces: holes, bosses, pockets, fillets, chamfers) from a solid, a compsolid
/// or a compound of solids, filling each gap by extending the faces adjacent to the feature and trimming them by each
/// other and by the original faces' bounds, then rebuilding the solids with <see cref="MakerVolume"/>.
/// </summary>
/// <remarks>
/// <para>
/// The features are removed one by one, so a feature that can't be removed (warning
/// <see cref="AlertUnableToRemoveTheFeature"/>) doesn't stop the others. The result is simplified by
/// <see cref="UnifySameDomain"/>, keeping the input's edges and vertices that are in the result. The
/// history (<see cref="BuilderShape.History"/>) answers for the input's sub-shapes.
/// </para>
/// <para>
/// The input is a published (frozen) shape and is never written. The result shares the input's untouched sub-shapes
/// by TShape, as OCCT's does. OCCT writes into a face of the solids it rebuilds when it removes the face's internal
/// wires; where that face is a frozen input face, the port replaces it by a copy without those wires (decisions,
/// unit 6G).
/// </para>
/// </remarks>
public class RemoveFeaturesAlgorithm : BuilderShape
{
    // Inputs
    protected Shape _inputShape = new();                     //!< Input shape
    protected NCollection_List<Shape> _facesToRemove = new(); //!< Faces to remove

    // Intermediate
    protected NCollection_List<Shape> _features = new(); //!< List of not connected features to remove (each feature is a compound of faces)
    protected NCollection_IndexedMap<Shape> _inputsMap = new(ShapeHasher.Instance); //!< Map of all sub-shapes of the input shape

    /// <summary>Empty constructor.</summary>
    public RemoveFeaturesAlgorithm()
    {
    }

    /// <summary>Sets the shape for processing: a SOLID, COMPSOLID or COMPOUND of solids.</summary>
    public void SetShape(in Shape shape) => _inputShape = shape;

    /// <summary>The input shape.</summary>
    public Shape InputShape() => _inputShape;

    /// <summary>Adds the faces of <paramref name="face"/> to the faces to remove.</summary>
    public void AddFaceToRemove(in Shape face) => _facesToRemove.Append(face);

    /// <summary>Adds the faces of the shapes of <paramref name="faces"/> to the faces to remove.</summary>
    public void AddFacesToRemove(NCollection_List<Shape> faces)
    {
        NCollection_List<Shape>.Iterator it = new(faces);
        for (; it.More(); it.Next())
        {
            _facesToRemove.Append(it.Value());
        }
    }

    /// <summary>The shapes requested for removal.</summary>
    public NCollection_List<Shape> FacesToRemove() => _facesToRemove;

    /// <summary>Clears the contents of the algorithm from the previous run, allowing reusing it.</summary>
    public override void Clear()
    {
        base.Clear();
        _inputShape.Nullify();
        _shape.Nullify();
        _facesToRemove.Clear();
        _features.Clear();
        _inputsMap.Clear();
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

    protected override void FillPIConstants(double whole, PISteps steps)
    {
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_PrepareFeatures, 0.05 * whole);
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_RemoveFeatures, 0.8 * whole);
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_UpdateHistory, 0.05 * whole);
        steps.SetStep((int)BOPAlgo_PIOperation.PIOperation_SimplifyResult, 0.1 * whole);
    }

    /// <summary>Performs the removal of the requested faces from the input shape.</summary>
    public override void Perform(ProgressRange range = default)
    {
        try
        {
            if (HasHistory)
            {
                _history = new History();
            }

            // Check the input data
            CheckData();
            if (HasErrors)
            {
                return;
            }
            using var ps = new ProgressScope(range, "Removing features", 100);
            PISteps steps = new((int)BOPAlgo_PIOperation.PIOperation_Last);
            AnalyzeProgress(100.0, steps);

            // Prepare the faces to remove.
            PrepareFeatures(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_PrepareFeatures)));
            if (HasErrors)
            {
                return;
            }

            // Remove the features and fill the created gaps
            RemoveFeatures(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_RemoveFeatures)));
            if (HasErrors)
            {
                return;
            }

            // Update history with the removed features
            UpdateHistory(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_UpdateHistory)));
            if (HasErrors)
            {
                return;
            }
            // Simplify the result
            SimplifyResult(ps.Next(steps.GetStep((int)BOPAlgo_PIOperation.PIOperation_SimplifyResult)));
            if (HasErrors)
            {
                return;
            }
            // Post treatment
            PostTreat();
        }
        catch (FailureException)
        {
            AddError(new AlertRemoveFeaturesFailed());
        }
    }

    /// <summary>
    /// Checks the input data: the input shape must be a SOLID, COMPSOLID or COMPOUND of solids; from a compound only
    /// the solids are kept (the other shapes are removed with a warning).
    /// </summary>
    protected override void CheckData()
    {
        // Prepare the shape to work with
        _shape = _inputShape;

        // Check the type of input shape
        ShapeType type = _inputShape.ShapeType();

        if (type == ShapeType.Solid || type == ShapeType.CompSolid)
        {
            return; // OK
        }

        if (type == ShapeType.Compound)
        {
            NCollection_List<Shape> shapes = new();
            NCollection_Map<Shape> mFence = new(ShapeHasher.Instance);
            // Extract all shapes from the compound
            AlgoTools.TreatCompound(_inputShape, shapes, mFence);
            if (shapes.IsEmpty())
            {
                // Add error of empty input shape
                AddError(new AlertEmptyShape(_inputShape));
                return;
            }

            // Find all solids in the list of shapes
            NCollection_List<Shape> solids = new();
            NCollection_List<Shape> otherShapes = new();
            NCollection_List<Shape>.Iterator it = new(shapes);
            for (; it.More(); it.Next())
            {
                Shape s = it.Value();
                if (s.ShapeType() == ShapeType.Solid || s.ShapeType() == ShapeType.CompSolid)
                {
                    solids.Append(s);
                }
                else
                {
                    otherShapes.Append(s);
                }
            }

            if (solids.IsEmpty())
            {
                // No solids have been found for processing.
                // Add error of unsupported type of input shape
                AddError(new AlertTooFewArguments());
            }
            else if (otherShapes.Extent() > 0)
            {
                // Add warning of unsupported type of input shape for all
                // non-solid shapes, contained in the input shape
                for (it.Initialize(otherShapes); it.More(); it.Next())
                {
                    AddWarning(new AlertUnsupportedType(it.Value()));
                }

                // Collect all solids into compound and overwrite the shape to rebuild
                Compound cs = new();
                new BrepBuilder().MakeCompound(ref cs);
                for (it.Initialize(solids); it.More(); it.Next())
                {
                    new BrepBuilder().Add(cs, it.Value());
                }

                _shape = cs;

                if (HasHistory)
                {
                    // Make non solid shapes removed in the history
                    MakeRemoved(otherShapes, _history!);
                }
            }
        }
        else
        {
            // Add error of unsupported type of input shape
            AddError(new AlertTooFewArguments());
        }
    }

    /// <summary>Prepares the features: the faces to remove that belong to the input, in connected blocks.</summary>
    protected void PrepareFeatures(ProgressRange range)
    {
        // Map all sub-shapes of the input solids
        Exploration.MapShapes(_inputShape, _inputsMap);

        // Collect all faces of the input shape requested for removal
        NCollection_List<Shape> facesToRemove = new();
        NCollection_List<Shape>.Iterator it = new(_facesToRemove);
        using var psOuter = new ProgressScope(range, "Preparing features", 2);
        using var ps = new ProgressScope(psOuter.Next(), "Preparing the faces to remove", _facesToRemove.Extent());
        for (; it.More(); it.Next(), ps.Next())
        {
            Shape s = it.Value();
            Explorer expF = new(s, ShapeType.Face);
            for (; expF.More(); expF.Next())
            {
                if (UserBreak(ps))
                {
                    return;
                }
                Shape f = expF.Current();
                if (_inputsMap.Contains(f))
                {
                    facesToRemove.Append(f);
                }
            }
        }

        if (facesToRemove.IsEmpty())
        {
            // Add error, that no features to remove have been found
            AddError(new AlertNoFacesToRemove());
            return;
        }

        // Build connexity blocks of the faces to remove
        Compound cfToRemove = new();
        new BrepBuilder().MakeCompound(ref cfToRemove);
        for (it.Initialize(facesToRemove); it.More(); it.Next())
        {
            new BrepBuilder().Add(cfToRemove, it.Value());
        }

        // Fill the list of features with connexity blocks of faces
        AlgoTools.MakeConnexityBlocks(cfToRemove, ShapeType.Edge, ShapeType.Face, _features);
    }

    //=======================================================================
    // Adjacent faces extension block

    /// <summary>
    /// OCCT's file-local class FillGap: makes the faces filling the gap left by one feature (the adjacent faces
    /// extended and trimmed). A solver of the parallel loop; it reads the shared connection maps only.
    /// </summary>
    private sealed class FillGap : BooleanParallel.ISolver
    {
        private bool _runParallel;                                                   //!< Defines the mode of processing of the single feature
        private Shape _feature = new();                                       //!< Feature to remove
        private NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>? _efMap; //!< EF Connection map to find adjacent faces
        private NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>? _fsMap; //!< FS Connection map to find solids participating in the feature removal
        private ProgressRange _range;                                        //!< Indication of progress

        // Results
        private readonly NCollection_Map<Shape> _featureFacesMap = new(ShapeHasher.Instance); //!< Faces of the feature
        private bool _hasAdjacentFaces;                                                                         //!< Flag to show whether the adjacent faces have been found or not
        private readonly NCollection_IndexedMap<Shape> _solids = new(ShapeHasher.Instance);  //!< Solids participating in the feature removal
        private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> _faces = new(ShapeHasher.Instance); //!< Reconstructed adjacent faces
        private History? _history;                                                                    //!< History of the adjacent faces reconstruction

        public FillGap()
        {
            _runParallel = false;
            _hasAdjacentFaces = false;
        }

        public void SetFeature(in Shape feature) => _feature = feature;

        public Shape Feature() => _feature;

        public void SetEFConnectionMap(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> efMap) => _efMap = efMap;

        public void SetFSConnectionMap(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> fsMap) => _fsMap = fsMap;

        public void SetRunParallel(bool bRunParallel) => _runParallel = bRunParallel;

        public History? History() => _history;

        public void SetRange(ProgressRange range) => _range = range;

        /// <summary>Extends the adjacent faces, then trims the extended faces to fill the gap.</summary>
        public void Perform()
        {
            try
            {
                using var ps = new ProgressScope(_range, null, 3);

                _history = new History();

                // Find the faces adjacent to the faces of the feature
                NCollection_IndexedMap<Shape> mfAdjacent = new(ShapeHasher.Instance);
                FindAdjacentFaces(mfAdjacent, ps.Next());
                if (!ps.More())
                {
                    return;
                }

                _hasAdjacentFaces = mfAdjacent.Extent() > 0;
                if (!_hasAdjacentFaces)
                {
                    return;
                }

                // Extend the adjacent faces keeping the connection to the original faces
                NCollection_IndexedDataMap<Shape, Shape> faceExtFaceMap = new(ShapeHasher.Instance);
                ExtendAdjacentFaces(mfAdjacent, faceExtFaceMap, ps.Next());
                if (!ps.More())
                {
                    return;
                }

                // Trim the extended faces
                TrimExtendedFaces(faceExtFaceMap, ps.Next());
            }
            catch (FailureException)
            {
                // Make sure the warning will be given on the higher level
                _hasAdjacentFaces = true;
                _faces.Clear();
            }
        }

        public NCollection_Map<Shape> FeatureFacesMap() => _featureFacesMap;

        public bool HasAdjacentFaces => _hasAdjacentFaces;

        public NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> Faces() => _faces;

        public NCollection_IndexedMap<Shape> Solids() => _solids;

        // Finds the faces adjacent to the feature and stores them into outgoing map.
        private void FindAdjacentFaces(NCollection_IndexedMap<Shape> mfAdjacent, ProgressRange range)
        {
            // Map the faces of the feature to avoid them in the map of adjacent faces
            Iterator it = new(_feature);
            for (; it.More(); it.Next())
            {
                _featureFacesMap.Add(it.Value());
            }
            using var psOuter = new ProgressScope(range, null, 2);
            // Find faces adjacent to the feature using the connection map
            it.Initialize(_feature);
            using var psf = new ProgressScope(psOuter.Next(), "Looking for adjacent faces", 1, true);
            for (; it.More(); it.Next(), psf.Next())
            {
                if (!psf.More())
                {
                    return;
                }
                Shape f = it.Value();
                Explorer expE = new(f, ShapeType.Edge);
                for (; expE.More(); expE.Next())
                {
                    Shape e = expE.Current();
                    if (_efMap!.FindFromKey(e, out NCollection_List<Shape> pAdjacentFaces))
                    {
                        NCollection_List<Shape>.Iterator itLFA = new(pAdjacentFaces);
                        for (; itLFA.More(); itLFA.Next())
                        {
                            Shape af = itLFA.Value();
                            if (!_featureFacesMap.Contains(af))
                            {
                                mfAdjacent.Add(af);
                            }
                        }
                    }
                }
                // Find solids containing the feature face
                if (_fsMap!.FindFromKey(f, out NCollection_List<Shape> pLS))
                {
                    NCollection_List<Shape>.Iterator itLS = new(pLS);
                    for (; itLS.More(); itLS.Next())
                    {
                        _solids.Add(itLS.Value());
                    }
                }
            }

            // Find solids containing the edges of adjacent faces
            int nbFA = mfAdjacent.Extent();
            using var pss = new ProgressScope(psOuter.Next(), "Looking for adjacent solids", nbFA);
            for (int i = 1; i <= nbFA; ++i, pss.Next())
            {
                if (!pss.More())
                {
                    return;
                }
                Explorer expEA = new(mfAdjacent.FindKey(i), ShapeType.Edge);
                for (; expEA.More(); expEA.Next())
                {
                    // Faces adjacent to the face adjacent to the feature
                    if (_efMap!.FindFromKey(expEA.Current(), out NCollection_List<Shape> pLFAA))
                    {
                        NCollection_List<Shape>.Iterator itLFAA = new(pLFAA);
                        for (; itLFAA.More(); itLFAA.Next())
                        {
                            // Solids containing the faces
                            if (_fsMap!.FindFromKey(itLFAA.Value(), out NCollection_List<Shape> pLS))
                            {
                                NCollection_List<Shape>.Iterator itLS = new(pLS);
                                for (; itLS.More(); itLS.Next())
                                {
                                    _solids.Add(itLS.Value());
                                }
                            }
                        }
                    }
                }
            }
        }

        // Extends the found adjacent faces and binds them to the original faces.
        private void ExtendAdjacentFaces(NCollection_IndexedMap<Shape> mfAdjacent,
                                         NCollection_IndexedDataMap<Shape, Shape> faceExtFaceMap,
                                         ProgressRange range)
        {
            // Get the extension value for the faces - half of the diagonal of bounding box of the feature
            Box featureBox = new();
            Bounds.Add(_feature, ref featureBox);

            double extLength = Math.Sqrt(featureBox.SquareExtent());

            int nbFA = mfAdjacent.Extent();
            using var ps = new ProgressScope(range, "Extending adjacent faces", nbFA);
            for (int i = 1; i <= nbFA && ps.More(); ++i, ps.Next())
            {
                Face f = ShapeCasts.Face(mfAdjacent.FindKey(i));
                // Extend the face (the port's unfrozen form: the trimming builds on it)
                BrepLibrary.InternalExtendFace(f, extLength, true, true, true, true, out Face fExt);
                faceExtFaceMap.Add(f, fExt);
                _history!.AddModified(f, fExt);
            }
        }

        // Trims the extended adjacent faces by intersection with each other
        // and following intersection with the bounds of original faces.
        private void TrimExtendedFaces(NCollection_IndexedDataMap<Shape, Shape> faceExtFaceMap,
                                       ProgressRange range)
        {
            // Intersect the extended faces first
            BooleanBuilder gfInter = new();
            // Add faces for intersection
            int nbF = faceExtFaceMap.Extent();
            for (int i = 1; i <= nbF; ++i)
            {
                gfInter.AddArgument(faceExtFaceMap.FindFromIndex(i));
            }

            gfInter.SetRunParallel(_runParallel);

            // Intersection result
            Shape intResult;
            using var psOuter = new ProgressScope(range, null, (gfInter.Arguments().Extent() > 1) ? 2 : 1);
            if (gfInter.Arguments().Extent() > 1)
            {
                gfInter.Perform(psOuter.Next());
                if (gfInter.HasErrors)
                {
                    return;
                }

                intResult = gfInter.InternalShape();

                _history!.Merge(gfInter.InternalHistory());
            }
            else
            {
                intResult = gfInter.Arguments().First();
            }

            // Prepare the EF map of the extended faces after intersection
            // to select from them only boundary edges
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> efExtMap = new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(intResult, ShapeType.Edge, ShapeType.Face, efExtMap);

            // Get the splits of the extended faces after intersection
            // and trim them by the edges of the original faces

            // Map the edges of the Feature to avoid them during the trim
            NCollection_IndexedMap<Shape> featureEdgesMap = new(ShapeHasher.Instance);
            Exploration.MapShapes(_feature, ShapeType.Edge, featureEdgesMap);

            using var ps = new ProgressScope(psOuter.Next(), "Trimming faces", nbF);
            for (int i = 1; i <= nbF && ps.More(); ++i, ps.Next())
            {
                Face fOriginal = ShapeCasts.Face(faceExtFaceMap.FindKey(i));
                Face fExt = ShapeCasts.Face(faceExtFaceMap.FindFromIndex(i));
                TrimFace(fExt, fOriginal, featureEdgesMap, efExtMap, gfInter);
            }
        }

        // Trim the extended faces by the bounds of the original face,
        // except those contained in the feature to remove.
        private void TrimFace(in Face fExt,
                              in Face fOriginal,
                              NCollection_IndexedMap<Shape> featureEdgesMap,
                              NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> efExtMap,
                              BooleanBuilder gfInter)
        {
            // Map all edges of the extended face, to filter the result of trim
            // from the faces containing these edges
            NCollection_Map<Shape> mExtEdges = new(ShapeHasher.Instance);
            Explorer expE = new(fExt, ShapeType.Edge);
            for (; expE.More(); expE.Next())
            {
                Edge e = ShapeCasts.Edge(expE.Current());
                // skip degenerated and seam edges
                if (Tool.Degenerated(e) || Tool.IsClosed(e, fExt))
                {
                    continue;
                }
                NCollection_List<Shape> leIm = new();
                TakeModified(e, gfInter, leIm);
                NCollection_List<Shape>.Iterator itLEIm = new(leIm);
                for (; itLEIm.More(); itLEIm.Next())
                {
                    Shape eIm = itLEIm.Value();
                    if (efExtMap.FindFromKey(eIm).Extent() == 1)
                    {
                        mExtEdges.Add(eIm);
                    }
                }
            }

            // Trimming tool
            BooleanBuilder gfTrim = new();

            // Get the splits of the face and add them for trimming
            NCollection_List<Shape> extSplits = new();
            TakeModified(fExt, gfInter, extSplits);
            gfTrim.SetArguments(extSplits);

            // Add edges of the original faces
            NCollection_Map<Shape> mEdgesToCheckOri = new(ShapeHasher.Instance);
            expE.Init(fOriginal, ShapeType.Edge);
            for (; expE.More(); expE.Next())
            {
                Edge e = ShapeCasts.Edge(expE.Current());
                if (!featureEdgesMap.Contains(e))
                {
                    gfTrim.AddArgument(e);
                    if (!Tool.Degenerated(e) && !Tool.IsClosed(e, fOriginal))
                    {
                        if (!mEdgesToCheckOri.Add(e))
                        {
                            mEdgesToCheckOri.Remove(e);
                        }
                    }
                }
            }

            // Avoid faces intersection
            gfTrim.SetGlue(GlueMode.GlueShift);
            gfTrim.SetRunParallel(_runParallel);
            gfTrim.SetNonDestructive(true);

            gfTrim.Perform();
            if (gfTrim.HasErrors)
            {
                return;
            }

            // Get all splits
            Shape splits = gfTrim.InternalShape();
            // Filter the trimmed faces and save the valid ones into result map
            NCollection_List<Shape> lfTrimmed = new();

            Explorer expF = new(splits, ShapeType.Face);
            for (; expF.More(); expF.Next())
            {
                Shape sp = expF.Current();
                expE.Init(sp, ShapeType.Edge);
                for (; expE.More(); expE.Next())
                {
                    if (mExtEdges.Contains(expE.Current()))
                    {
                        break;
                    }
                }
                if (!expE.More())
                {
                    lfTrimmed.Append(sp);
                }
            }

            if (lfTrimmed.Extent() > 1)
            {
                // Chose the correct faces - the ones that contains edges with proper
                // bi-normal direction
                NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> efMap = new(ShapeHasher.Instance);
                NCollection_List<Shape>.Iterator itLF0 = new(lfTrimmed);
                for (; itLF0.More(); itLF0.Next())
                {
                    Exploration.MapShapesAndAncestors(itLF0.Value(), ShapeType.Edge, ShapeType.Face, efMap);
                }

                // Check edges orientations
                NCollection_Map<Shape> facesToAvoid = new(ShapeHasher.Instance), validFaces = new(ShapeHasher.Instance);
                FindExtraShapes(efMap, mEdgesToCheckOri, gfTrim, facesToAvoid, validFaces);

                if (lfTrimmed.Extent() - facesToAvoid.Extent() > 1)
                {
                    // It is possible that the splits are forming the different blocks.
                    // Take only those containing the valid faces.
                    Compound cf = new();
                    new BrepBuilder().MakeCompound(ref cf);
                    itLF0.Initialize(lfTrimmed);
                    for (; itLF0.More(); itLF0.Next())
                    {
                        if (!facesToAvoid.Contains(itLF0.Value()))
                        {
                            new BrepBuilder().Add(cf, itLF0.Value());
                        }
                    }

                    NCollection_List<Shape> lcb = new();
                    AlgoTools.MakeConnexityBlocks(cf, ShapeType.Edge, ShapeType.Face, lcb);
                    if (lcb.Extent() > 1)
                    {
                        NCollection_List<Shape>.Iterator itLCB = new(lcb);
                        for (; itLCB.More(); itLCB.Next())
                        {
                            // Check if the block contains any valid faces
                            Shape cb = itLCB.Value();
                            Iterator itF = new(cb);
                            for (; itF.More(); itF.Next())
                            {
                                if (validFaces.Contains(itF.Value()))
                                {
                                    break;
                                }
                            }
                            if (!itF.More())
                            {
                                // Invalid block
                                for (itF.Initialize(cb); itF.More(); itF.Next())
                                {
                                    facesToAvoid.Add(itF.Value());
                                }
                            }
                        }
                    }
                }

                itLF0.Initialize(lfTrimmed);
                for (; itLF0.More();)
                {
                    if (facesToAvoid.Contains(itLF0.Value()))
                    {
                        lfTrimmed.Remove(itLF0);
                    }
                    else
                    {
                        itLF0.Next();
                    }
                }
            }
            else if (lfTrimmed.IsEmpty())
            {
                // Use all splits, including those having the bounds of extended face
                expF.ReInit();
                for (; expF.More(); expF.Next())
                {
                    lfTrimmed.Append(expF.Current());
                }
            }

            if (lfTrimmed.Extent() != 0)
            {
                // Remove the internal edges and vertices from the faces
                History copies = new();
                RemoveInternalWires(lfTrimmed, null, copies);
                _history!.Merge(copies);

                _faces.Add(fOriginal, lfTrimmed);
            }

            // Update history after intersection of the extended face with bounds
            _history!.Merge(gfTrim.InternalHistory());

            // Update history with all removed shapes
            History histRem = new();

            // Map of the result splits
            NCollection_IndexedMap<Shape> resMap = new(ShapeHasher.Instance);
            NCollection_List<Shape>.Iterator itLF = new(lfTrimmed);
            for (; itLF.More(); itLF.Next())
            {
                Exploration.MapShapes(itLF.Value(), resMap);
            }

            NCollection_List<Shape> lSplits = new();
            lSplits.Append(splits);

            // Update the history with removed shapes
            MakeRemoved(lSplits, histRem, resMap);
            _history.Merge(histRem);
        }
    }

    /// <summary>
    /// Removes the features one by one, filling each gap by extension of the adjacent faces.
    /// </summary>
    protected void RemoveFeatures(ProgressRange range)
    {
        // For each feature:
        // - Find the faces adjacent to the feature;
        // - Extend the adjacent faces;
        // - Trim the extended faces to fill the gap;
        // - Rebuild the solids with reconstructed adjacent faces
        //   and avoiding the feature faces.

        using var psOuter = new ProgressScope(range, "Removing features", 2);

        // Make Edge-Face connection map of the input
        // shape to find faces adjacent to the feature
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> efMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(_inputShape, ShapeType.Edge, ShapeType.Face, efMap);

        // Make Face-Solid connection map to find the solids
        // participating in the removal of each feature
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> fsMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(_inputShape, ShapeType.Face, ShapeType.Solid, fsMap);

        // Tool for reconstruction of the faces adjacent to the feature
        // in parallel threads if necessary.
        NCollection_DynamicArray<FillGap> vfg = new();
        // Fill the vector
        NCollection_List<Shape>.Iterator itF = new(_features);
        for (; itF.More(); itF.Next())
        {
            Shape feature = itF.Value();
            FillGap fg = vfg.Append(new FillGap());
            fg.SetFeature(feature);
            fg.SetEFConnectionMap(efMap);
            fg.SetFSConnectionMap(fsMap);
            fg.SetRunParallel(_runParallel);
        }

        int nbF = vfg.Length();
        using var ps = new ProgressScope(psOuter.Next(), "Filling gaps", nbF);
        for (int i = 0; i < nbF; ++i)
        {
            FillGap fg = vfg.Value(i);
            fg.SetRange(ps.Next());
        }
        // Perform the reconstruction of the adjacent faces
        BooleanParallel.Perform(_runParallel, vfg);
        if (UserBreak(psOuter))
        {
            return;
        }
        // Even if the history is not requested, it is necessary to track:
        // - The solids modification after each feature removal to find
        //   the necessary solids to rebuild on the next step.
        // - The faces modification after each feature removal to find the
        //   splits of the adjacent and feature faces for the next steps.
        _history ??= new History();

        // Remove the features one by one.
        // It will allow removing the features even if there were
        // some problems with removal of the previous features.
        using var psLoop = new ProgressScope(psOuter.Next(), "Removing features one by one", nbF);
        for (int i = 0; i < nbF; ++i)
        {
            if (UserBreak(psLoop))
            {
                return;
            }
            FillGap fg = vfg.Value(i);

            // No need to fill the history for solids if the history is not
            // requested and the current feature is the last one.
            bool isSolidsHistoryNeeded = HasHistory || (i < (nbF - 1));

            // Perform removal of the single feature
            RemoveFeature(fg.Feature(),
                          fg.Solids(),
                          fg.FeatureFacesMap(),
                          fg.HasAdjacentFaces,
                          fg.Faces(),
                          fg.History(),
                          isSolidsHistoryNeeded,
                          psLoop.Next());
        }
    }

    /// <summary>Removes a single feature from the shape.</summary>
    /// <param name="feature">The feature to remove.</param>
    /// <param name="solids">The solids to be reconstructed after the feature removal.</param>
    /// <param name="featureFacesMap">The map of the feature's faces.</param>
    /// <param name="hasAdjacentFaces">Whether adjacent faces have been found for the feature.</param>
    /// <param name="adjFaces">The reconstructed adjacent faces covering the feature.</param>
    /// <param name="adjFacesHistory">The history of the adjacent faces' reconstruction.</param>
    /// <param name="solidsHistoryNeeded">Whether the history of the solids' modifications is tracked.</param>
    /// <param name="range">The progress range.</param>
    protected void RemoveFeature(in Shape feature,
                                 NCollection_IndexedMap<Shape> solids,
                                 NCollection_Map<Shape> featureFacesMap,
                                 bool hasAdjacentFaces,
                                 NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> adjFaces,
                                 History? adjFacesHistory,
                                 bool solidsHistoryNeeded,
                                 ProgressRange range)
    {
        bool bFuseShapes = true;
        int nbAF = adjFaces.Extent();
        if (nbAF == 0)
        {
            if (hasAdjacentFaces)
            {
                // The adjacent faces have been found for the feature,
                // but something went wrong during their rebuilding.
                // Add error
                AddWarning(new AlertUnableToRemoveTheFeature(feature));
                return;
            }

            // No adjacent faces have been found for the feature.
            // Most likely, the intention is to remove some internal part of the shape.
            // We just have to rebuild the shape without the feature, no fuse is necessary.
            bFuseShapes = false;
        }

        using var ps = new ProgressScope(range, null, 100);
        // Rebuild the shape using MakerVolume algorithm avoiding the faces of the
        // feature and replacing the adjacent faces with their images

        BrepBuilder bb = new();

        // From the faces of input shape build map of faces with which the result will be validated:
        // - all non-internal faces of the input shape (except for adjacent and feature faces)
        //   must have some trace in the result solids;
        // - all adjacent faces (if any) must have some trace in the result solids.
        NCollection_IndexedMap<Shape> facesToBeKept = new(ShapeHasher.Instance);
        // Build also the map of faces to compare orientation of the original and result faces
        NCollection_Map<Shape> facesToCheckOri = new(ShapeHasher.Instance);

        // Collect list of internal entities of the input shape to be avoided in result
        // and to make them removed in the history.
        NCollection_List<Shape> internalShapes = new();

        // In the feature removal will participate only the solids connected to the feature
        // or the faces adjacent to the feature.

        // Solids to be rebuilt  after the feature removal
        NCollection_IndexedMap<Shape> solidsToRebuild = new(ShapeHasher.Instance);
        // Find faces shared between solids
        NCollection_List<Shape> sharedFaces = new();
        // Solids to be avoided in the feature removal and added into result directly
        NCollection_List<Shape> unTouchedSolids = new();

        // Prepare to the feature removal - fill all necessary containers
        GetOriginalFaces(_shape,
                         solids,
                         featureFacesMap,
                         adjFaces,
                         _history!,
                         facesToBeKept,
                         internalShapes,
                         facesToCheckOri,
                         solidsToRebuild,
                         sharedFaces,
                         unTouchedSolids);

        ps.Next(3);
        // To avoid excessive intersection of the faces collect the faces
        // of the input shape into a compound
        Compound origF = new();
        bb.MakeCompound(ref origF);
        int nbFK = facesToBeKept.Extent();
        for (int i = 1; i <= nbFK; ++i)
        {
            bb.Add(origF, facesToBeKept.FindKey(i));
        }

        // Tool for solids reconstruction
        MakerVolume mv = new();
        mv.SetRunParallel(_runParallel);
        mv.SetAvoidInternalShapes(true);
        mv.SetIntersect(bFuseShapes);
        mv.SetNonDestructive(true);
        // Add faces of the input shape
        mv.AddArgument(origF);

        // Add reconstructed adjacent faces
        for (int i = 1; i <= nbAF; ++i)
        {
            NCollection_List<Shape> lfa = adjFaces.FindFromIndex(i);
            if (lfa.Extent() == 1)
            {
                Shape fa = lfa.First();
                mv.AddArgument(fa);
                facesToBeKept.Add(fa);
            }
            else
            {
                // To avoid intersection among the images, collect them into compound
                Compound adjF = new();
                bb.MakeCompound(ref adjF);
                NCollection_List<Shape>.Iterator itLFA = new(lfa);
                for (; itLFA.More(); itLFA.Next())
                {
                    bb.Add(adjF, itLFA.Value());
                }

                mv.AddArgument(adjF);
                facesToBeKept.Add(adjF);
            }

            if (HasHistory)
            {
                // Look for internal edges in the original adjacent faces
                Shape fOr = adjFaces.FindKey(i);
                FindInternals(fOr, internalShapes);
            }
        }

        // Build solids
        mv.Perform(ps.Next(90));
        if (mv.HasErrors)
        {
            // Add warning for the feature
            AddWarning(new AlertUnableToRemoveTheFeature(feature));
            return;
        }

        // Result of MV operation
        Shape aSolids = mv.InternalShape();
        Explorer expS = new(aSolids, ShapeType.Solid);
        if (!expS.More())
        {
            // No solids have been built - add warning for the feature
            AddWarning(new AlertUnableToRemoveTheFeature(feature));
            return;
        }

        // Now, it is necessary to:
        // - Validate the result by checking faces of the map <aFacesToBeKept>
        //   to have some parts kept in the resulting solids;
        // - Remove the solids possibly filling the holes in the original shapes;
        // - Update history with the history of MakerVolume algorithm.

        // Splits of adjacent faces from previous runs
        NCollection_Map<Shape> adjFacesSplits = new(ShapeHasher.Instance);
        for (int i = 1; i <= nbAF; ++i)
        {
            if (!ps.More())
            {
                return;
            }
            Shape f = adjFaces.FindKey(i);
            IReadOnlyList<Shape> lfIm = _history!.Modified(f);
            if (lfIm.Count == 0)
            {
                adjFacesSplits.Add(f);
            }
            else
            {
                foreach (Shape fIm in lfIm)
                {
                    adjFacesSplits.Add(fIm);
                }
            }
        }

        // Validate the result
        bool bValid = true;
        nbFK = facesToBeKept.Extent();
        for (int i = 1; i <= nbFK && bValid; ++i)
        {
            Shape s = facesToBeKept.FindKey(i);
            if (adjFacesSplits.Contains(s))
            {
                continue;
            }
            if (!ps.More())
            {
                return;
            }
            Explorer expF = new(s, ShapeType.Face);
            for (; expF.More(); expF.Next())
            {
                Shape f = expF.Current();
                if (!mv.IsDeleted(f))
                {
                    break;
                }
            }
            bValid = expF.More();
        }

        if (!bValid)
        {
            // Add warning for the feature
            AddWarning(new AlertUnableToRemoveTheFeature(feature));
            return;
        }

        // It is possible that the result of MakerVolume operation contains
        // some extra solids. Get only correct solids.
        NCollection_List<Shape> lsRes = new();
        // Remember the removed shapes
        NCollection_List<Shape> removedShapes = new();
        GetValidSolids(mv,
                       facesToCheckOri,
                       sharedFaces,
                       origF,
                       solids.Extent(),
                       lsRes,
                       removedShapes);

        if (lsRes.Extent() != solids.Extent())
        {
            // Add warning for the feature
            AddWarning(new AlertUnableToRemoveTheFeature(feature));
            return;
        }
        ps.Next(3);
        // Remove internal wires from the faces, possibly appeared after intersection
        History copies = new();
        RemoveInternalWires(lsRes, internalShapes, copies);

        // Update history with:
        // History of adjacent faces reconstruction
        _history!.Merge(adjFacesHistory);
        // History of intersection
        _history.Merge(mv.InternalHistory());
        // OCCT.NET: frozen. The input faces RemoveInternalWires replaced by copies (OCCT edits them in place)
        _history.Merge(copies);

        if (HasHistory)
        {
            // Map the result to check if the shape is removed
            NCollection_IndexedMap<Shape> msRes = new(ShapeHasher.Instance);
            NCollection_List<Shape>.Iterator itLS0 = new(lsRes);
            for (; itLS0.More(); itLS0.Next())
            {
                Exploration.MapShapes(itLS0.Value(), msRes);
            }

            // Remove internal shapes and extra faces
            History remHist = new();
            internalShapes.Append(removedShapes);
            MakeRemoved(internalShapes, remHist, msRes);
            _history.Merge(remHist);
        }
        ps.Next(3);
        // Fill the history for the solids
        if (solidsHistoryNeeded)
        {
            History solidsHistory = new();
            FillSolidsHistory(solidsToRebuild, lsRes, adjFaces, mv, solidsHistory);
            _history.Merge(solidsHistory);
        }

        Compound cRes = new();
        bb.MakeCompound(ref cRes);
        // Add reconstructed solids
        NCollection_List<Shape>.Iterator itLS = new(lsRes);
        for (; itLS.More(); itLS.Next())
        {
            bb.Add(cRes, itLS.Value());
        }

        // Add unmodified solids
        itLS.Initialize(unTouchedSolids);
        for (; itLS.More(); itLS.Next())
        {
            bb.Add(cRes, itLS.Value());
        }

        // Save the result
        _shape = cRes;
    }

    /// <summary>Updates the history with the removed features.</summary>
    protected void UpdateHistory(ProgressRange range)
    {
        if (!HasHistory)
        {
            return;
        }

        // Map the result
        _mapShape.Clear();
        Exploration.MapShapes(_shape, _mapShape);

        // Update the history
        History history = new();

        int nbS = _inputsMap.Extent();
        using var ps = new ProgressScope(range, "Updating history", nbS);
        for (int i = 1; i <= nbS; ++i, ps.Next())
        {
            Shape s = _inputsMap.FindKey(i);
            if (!global::Kerf.Engine.Topology.History.IsSupportedType(s))
            {
                continue;
            }

            if (_history!.IsRemoved(s))
            {
                continue;
            }

            if (UserBreak(ps))
            {
                return;
            }
            // Check if the shape has any trace in the result
            IReadOnlyList<Shape> lsIm = _history.Modified(s);
            if (lsIm.Count == 0)
            {
                if (!_mapShape.Contains(s))
                {
                    history.Remove(s);
                }
            }

            foreach (Shape sIm in lsIm)
            {
                if (!_mapShape.Contains(sIm))
                {
                    history.Remove(sIm);
                }
            }
        }

        _history!.Merge(history);
    }

    /// <summary>Simplifies the result, removing the extra edges and vertices created by the operation.</summary>
    protected void SimplifyResult(ProgressRange range)
    {
        if (_shape.IsSame(_inputShape))
        {
            return;
        }
        using var psOuter = new ProgressScope(range, "Simplifyingthe result", 2);
        UnifySameDomain sdTool = new();
        sdTool.Initialize(_shape, true, true);
        // Do not allow producing internal edges
        sdTool.AllowInternalEdges(false);
        // Avoid removal of the input edges and vertices
        if (_mapShape.IsEmpty())
        {
            Exploration.MapShapes(_shape, _mapShape);
        }

        int nbS = _inputsMap.Extent();
        using var ps = new ProgressScope(psOuter.Next(), null, nbS);
        for (int i = 1; i <= nbS; ++i, ps.Next())
        {
            if (UserBreak(ps))
            {
                return;
            }
            if (_mapShape.Contains(_inputsMap.FindKey(i)))
            {
                sdTool.KeepShape(_inputsMap.FindKey(i));
            }
        }

        // Perform unification (on the result, which shares the input's frozen sub-shapes: UnifySameDomain copies at
        // its entry, decision 766, and its history answers for the result's own sub-shapes)
        sdTool.Build();
        psOuter.Next();
        _shape = sdTool.InternalShape();
        if (HasHistory)
        {
            _history!.Merge(sdTool.InternalHistory());
        }
    }

    /// <summary>Post treatment: restores the type of the input shape.</summary>
    protected void PostTreat()
    {
        ShapeType inputType = _inputShape.ShapeType();
        ShapeType resType = _shape.ShapeType();
        if (resType == inputType)
        {
            return;
        }

        Explorer expS = new(_shape, ShapeType.Solid);

        if (inputType == ShapeType.Solid)
        {
            _shape = expS.Current();
            return;
        }

        Shape res;
        if (inputType == ShapeType.Compound)
        {
            Compound c = new();
            new BrepBuilder().MakeCompound(ref c);
            res = c;
        }
        else
        {
            CompSolid cs = new();
            new BrepBuilder().MakeCompSolid(ref cs);
            res = cs;
        }

        for (; expS.More(); expS.Next())
        {
            new BrepBuilder().Add(res, expS.Current());
        }

        _shape = res;
    }

    //=======================================================================
    // static methods definition
    //=======================================================================

    // Makes the shapes in the list removed in the history. Keeps the shapes contained in the map.
    private static void MakeRemoved(NCollection_List<Shape> shapes,
                                    History history,
                                    NCollection_IndexedMap<Shape>? keepShapes = null)
    {
        NCollection_IndexedMap<Shape> shapesMap = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator it = new(shapes);
        for (; it.More(); it.Next())
        {
            Exploration.MapShapes(it.Value(), shapesMap);
        }

        int nbS = shapesMap.Extent();
        for (int i = 1; i <= nbS; ++i)
        {
            Shape s = shapesMap.FindKey(i);
            if ((keepShapes is null || !keepShapes.Contains(s)) && global::Kerf.Engine.Topology.History.IsSupportedType(s))
            {
                history.Remove(s);
            }
        }
    }

    // Looks for internal shapes inside the face or solid.
    private static void FindInternals(in Shape s, NCollection_List<Shape> lInt)
    {
        Iterator itS = new(s);
        for (; itS.More(); itS.Next())
        {
            Shape ss = itS.Value();
            if (ss.Orientation() == Internal)
            {
                lInt.Append(ss);
            }
            else
            {
                Iterator itSS = new(ss);
                for (; itSS.More(); itSS.Next())
                {
                    if (itSS.Value().Orientation() == Internal)
                    {
                        lInt.Append(ss);
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
    private static void RemoveInternalWires(NCollection_List<Shape> shapes,
                                            NCollection_List<Shape>? removed,
                                            History copies)
    {
        NCollection_DataMap<Shape, Shape> aCopies = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itLS = new(shapes);
        for (; itLS.More(); itLS.Next())
        {
            Shape shape = itLS.Value();
            Explorer expF = new(shape, ShapeType.Face);
            for (; expF.More(); expF.Next())
            {
                Shape f = expF.Current();
                NCollection_List<Shape> lwToRemove = new();
                FindInternals(f, lwToRemove);
                if (lwToRemove.Extent() != 0)
                {
                    if (f.TShape()!.IsFrozen)
                    {
                        if (aCopies.IsBound(f))
                        {
                            continue;
                        }
                        // the copy: the face's TShape emptied, with the wires that stay
                        Shape fCopy = f.EmptyCopied();
                        Iterator itW = new(f);
                        for (; itW.More(); itW.Next())
                        {
                            if (!lwToRemove.Contains(itW.Value()))
                            {
                                new BrepBuilder().Add(fCopy, itW.Value());
                            }
                        }
                        NCollection_List<Shape>.Iterator itR0 = new(lwToRemove);
                        for (; itR0.More(); itR0.Next())
                        {
                            removed?.Append(itR0.Value());
                        }
                        aCopies.Bind(f, fCopy);
                        copies.AddModified(f, fCopy);
                        continue;
                    }
                    f.Free(true);
                    NCollection_List<Shape>.Iterator itR = new(lwToRemove);
                    for (; itR.More(); itR.Next())
                    {
                        removed?.Append(itR.Value());
                        new BrepBuilder().Remove(f, itR.Value());
                    }
                    f.Free(false);
                }
            }
        }

        if (aCopies.IsEmpty())
        {
            return;
        }
        // Put the copies in place of the frozen faces
        for (itLS.Initialize(shapes); itLS.More(); itLS.Next())
        {
            Shape shape = itLS.Value();
            if (shape.ShapeType() == ShapeType.Face)
            {
                if (aCopies.Find(shape, out Shape copy))
                {
                    itLS.ChangeValue() = WithTShape(shape, copy);
                }
                continue;
            }
            ReplaceFaces(shape, aCopies);
        }
    }

    // The shape with the TShape of theTShapeOf, keeping its own location and orientation.
    private static Shape WithTShape(in Shape shape, in Shape tShapeOf)
    {
        Shape s = shape;
        s.TShape(tShapeOf.TShape());
        return s;
    }

    // Replaces the faces bound in theCopies by their copies in the (unfrozen) containers of theShape, keeping the
    // order of every container's children.
    private static bool ReplaceFaces(in Shape shape, NCollection_DataMap<Shape, Shape> copies)
    {
        if (shape.ShapeType() == ShapeType.Face)
        {
            return false;
        }
        List<Shape> children = [];
        bool changed = false;
        for (Iterator it = new(shape, false, false); it.More(); it.Next())
        {
            Shape child = it.Value();
            if (child.ShapeType() == ShapeType.Face && copies.Find(child, out Shape copy))
            {
                child = WithTShape(child, copy);
                changed = true;
            }
            else if (!child.TShape()!.IsFrozen && ReplaceFaces(child, copies))
            {
                changed = true;
            }
            children.Add(child);
        }
        if (!changed)
        {
            return false;
        }
        // Rebuild the container's list in its order (TShape-relative children: no location or orientation composed)
        Shape bare = shape;
        bare.Location(new Placement(), false);
        bare.Orientation(Forward);
        bool free = bare.Free();
        bare.Free(true);
        BrepBuilder bb = new();
        List<Shape> old = [];
        for (Iterator it = new(bare, false, false); it.More(); it.Next())
        {
            old.Add(it.Value());
        }
        foreach (Shape child in old)
        {
            bb.Remove(bare, child);
        }
        foreach (Shape child in children)
        {
            bb.Add(bare, child);
        }
        bare.Free(free);
        return true;
    }

    // Get original faces from my face
    private static void GetOriginalFaces(in Shape shape,
                                         NCollection_IndexedMap<Shape> solids,
                                         NCollection_Map<Shape> featureFacesMap,
                                         NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> adjFaces,
                                         History history,
                                         NCollection_IndexedMap<Shape> facesToBeKept,
                                         NCollection_List<Shape> internalShapes,
                                         NCollection_Map<Shape> facesToCheckOri,
                                         NCollection_IndexedMap<Shape> solidsToRebuild,
                                         NCollection_List<Shape> sharedFaces,
                                         NCollection_List<Shape> unTouchedSolids)
    {
        // Use only solids which has to be reconstructed by the feature.
        // All other solids should be avoided in the feature removal and added
        // into result directly.

        // Update solids after removal of previous features
        int nbSols = solids.Extent();
        for (int i = 1; i <= nbSols; ++i)
        {
            Shape sol = solids.FindKey(i);
            IReadOnlyList<Shape> lfIm = history.Modified(sol);
            if (lfIm.Count == 0)
            {
                solidsToRebuild.Add(sol);
            }
            else
            {
                solidsToRebuild.Add(lfIm[0]);
            }
        }

        // Splits of the feature faces
        NCollection_Map<Shape> featureFacesSplits = new(ShapeHasher.Instance);
        foreach (Shape f in featureFacesMap)
        {
            IReadOnlyList<Shape> lfIm = history.Modified(f);
            if (lfIm.Count == 0)
            {
                featureFacesSplits.Add(f);
            }
            else
            {
                foreach (Shape fIm in lfIm)
                {
                    featureFacesSplits.Add(fIm);
                }
            }
        }

        Explorer expS = new(shape, ShapeType.Solid);
        for (; expS.More(); expS.Next())
        {
            Shape sol = expS.Current();

            // Check if the solid has to be reconstructed
            if (!solidsToRebuild.Contains(sol))
            {
                // untouched solid
                unTouchedSolids.Append(sol);
                continue;
            }

            Iterator itSh = new(sol);
            for (; itSh.More(); itSh.Next())
            {
                Shape sh = itSh.Value();
                if (sh.ShapeType() != ShapeType.Shell)
                {
                    internalShapes.Append(sh);
                    continue;
                }

                Iterator itF = new(sh);
                for (; itF.More(); itF.Next())
                {
                    Shape f = itF.Value();
                    // Avoid the feature faces
                    if (featureFacesSplits.Contains(f))
                    {
                        continue;
                    }

                    // Avoid the adjacent faces
                    if (adjFaces.Contains(f))
                    {
                        continue;
                    }

                    if (f.Orientation() != Internal)
                    {
                        facesToBeKept.Add(f);

                        if (!facesToCheckOri.Add(f))
                        {
                            facesToCheckOri.Remove(f);
                            sharedFaces.Append(f);
                        }
                    }
                    else
                    {
                        internalShapes.Append(sh);
                    }
                }
            }
        }
    }

    // Find the shape in the other shape
    private static void FindShape(in Shape sWhat, in Shape sWhere, ref Shape sFound)
    {
        Explorer exp = new(sWhere, sWhat.ShapeType());
        for (; exp.More(); exp.Next())
        {
            Shape s = exp.Current();
            if (s.IsSame(sWhat))
            {
                sFound = s;
                break;
            }
        }
    }

    // Checks the validity of the solids and keeps only valid ones
    private static void GetValidSolids(MakerVolume mv,
                                       NCollection_Map<Shape> facesToCheckOri,
                                       NCollection_List<Shape> aSharedFaces,
                                       in Shape origFaces,
                                       int nbSol,
                                       NCollection_List<Shape> lsRes,
                                       NCollection_List<Shape> removedShapes)
    {
        Explorer expS = new(mv.InternalShape(), ShapeType.Solid);
        for (; expS.More(); expS.Next())
        {
            lsRes.Append(expS.Current());
        }

        if (lsRes.Extent() > nbSol)
        {
            // Find Solids filling the holes in the initial shape
            NCollection_Map<Shape> solidsToAvoid = new(ShapeHasher.Instance);
            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> fsMap = new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(mv.InternalShape(), ShapeType.Face, ShapeType.Solid, fsMap);
            FindExtraShapes(fsMap, facesToCheckOri, mv, solidsToAvoid);

            NCollection_List<Shape>.Iterator itLS = new(lsRes);
            for (; itLS.More();)
            {
                if (solidsToAvoid.Contains(itLS.Value()))
                {
                    lsRes.Remove(itLS);
                }
                else
                {
                    itLS.Next();
                }
            }
        }

        if (lsRes.Extent() > nbSol)
        {
            // Check if the splits of the adjacent faces split the solids
            AvoidExtraSharedFaces(lsRes, aSharedFaces, mv, removedShapes);
        }

        if (lsRes.Extent() > nbSol)
        {
            // Remove solids containing only the adjacent faces
            NCollection_Map<Shape> origFacesRes = new(ShapeHasher.Instance);
            Explorer expF = new(origFaces, ShapeType.Face);
            for (; expF.More(); expF.Next())
            {
                TakeModified(expF.Current(), mv, origFacesRes);
            }

            NCollection_List<Shape>.Iterator itLS = new(lsRes);
            for (; itLS.More();)
            {
                expF.Init(itLS.Value(), ShapeType.Face);
                for (; expF.More(); expF.Next())
                {
                    if (origFacesRes.Contains(expF.Current()))
                    {
                        break;
                    }
                }
                if (!expF.More())
                {
                    removedShapes.Append(itLS.Value());
                    lsRes.Remove(itLS);
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
    private static void FindExtraShapes(NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> connectionMap,
                                        NCollection_Map<Shape> shapesToCheckOri,
                                        BooleanBuilder builder,
                                        NCollection_Map<Shape> shapesToAvoid,
                                        NCollection_Map<Shape>? validShapes = null)
    {
        IntersectionContext? ctx = builder.Context();
        NCollection_Map<Shape> pValidShapes = validShapes ?? new NCollection_Map<Shape>(ShapeHasher.Instance);
        foreach (Shape sToCheckOri in shapesToCheckOri)
        {
            // Check modification of the shape during intersection
            NCollection_List<Shape> lsIm = new();
            TakeModified(sToCheckOri, builder, lsIm);

            NCollection_List<Shape>.Iterator itLSIm = new(lsIm);
            for (; itLSIm.More(); itLSIm.Next())
            {
                Shape sIm = itLSIm.Value();

                if (!connectionMap.FindFromKey(sIm, out NCollection_List<Shape> pShapesToValidate))
                {
                    continue;
                }

                NCollection_List<Shape>.Iterator itSV = new(pShapesToValidate);
                for (; itSV.More(); itSV.Next())
                {
                    Shape shapeToValidate = itSV.Value();
                    if (pValidShapes.Contains(shapeToValidate))
                    {
                        continue;
                    }

                    // OCCT finds the image into a TopoDS_Face, whatever its type (the edges of TrimFace's check
                    // are copied into it: TopoDS_Shape's assignment)
                    Shape sInShape = new();
                    FindShape(sIm, shapeToValidate, ref sInShape);

                    bool bSameOri = !AlgoTools.IsSplitToReverse(sInShape, sToCheckOri, ctx);

                    if (bSameOri)
                    {
                        pValidShapes.Add(shapeToValidate);
                    }
                    else
                    {
                        shapesToAvoid.Add(shapeToValidate);
                    }
                }
            }
        }

        foreach (Shape valid in pValidShapes)
        {
            shapesToAvoid.Remove(valid);
        }
    }

    // Looks for the extra faces splitting the solids
    private static void AvoidExtraSharedFaces(NCollection_List<Shape> lSolids,
                                              NCollection_List<Shape> lfSharedToAvoid,
                                              BooleanBuilder builder,
                                              NCollection_List<Shape> extraFaces)
    {
        // Get all splits of shared faces to avoid in the check
        NCollection_Map<Shape> mfSharedSp = new(ShapeHasher.Instance);
        {
            NCollection_List<Shape>.Iterator itLFS = new(lfSharedToAvoid);
            for (; itLFS.More(); itLFS.Next())
            {
                TakeModified(itLFS.Value(), builder, mfSharedSp);
            }
        }

        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> fsMap = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itLS = new(lSolids);
        for (; itLS.More(); itLS.Next())
        {
            Exploration.MapShapesAndAncestors(itLS.Value(), ShapeType.Face, ShapeType.Solid, fsMap);
        }

        NCollection_List<Shape> anExtraFaces = new();
        NCollection_List<Shape> lfArguments = new();
        itLS.Initialize(lSolids);
        for (; itLS.More(); itLS.Next())
        {
            Shape sol = itLS.Value();
            Explorer expF = new(sol, ShapeType.Face);
            for (; expF.More(); expF.Next())
            {
                Shape f = expF.Current();
                NCollection_List<Shape> lSol = fsMap.FindFromKey(f);
                if (lSol.Extent() != 2 || mfSharedSp.Contains(f))
                {
                    lfArguments.Append(f);
                }
                else
                {
                    anExtraFaces.Append(f);
                }
            }
        }

        if (anExtraFaces.IsEmpty())
        {
            return;
        }

        // Rebuild the solids avoiding the extra faces
        BuilderSolid bs = new();
        bs.SetAvoidInternalShapes(true);
        bs.SetShapes(lfArguments);
        bs.Perform();
        if (bs.HasErrors)
        {
            return;
        }

        lSolids.Assign(bs.Areas());
        extraFaces.Append(anExtraFaces);
    }

    // Fills the history of solids modifications
    private static void FillSolidsHistory(NCollection_IndexedMap<Shape> solIn,
                                          NCollection_List<Shape> solidsOut,
                                          NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> adjFaces,
                                          BooleanBuilder builder,
                                          History solidsHistory)
    {
        int nbS = solIn.Extent();
        for (int i = 1; i <= nbS; ++i)
        {
            Shape aSolIn = solIn.FindKey(i);

            Shape solOut = new();
            FindSolid(aSolIn, solidsOut, adjFaces, builder, ref solOut);

            if (solOut.IsNull)
            {
                solidsHistory.Remove(aSolIn);
                continue;
            }

            // Check if the solid has really been modified
            ToolSet stIn = new(), stOut = new();
            stIn.Add(aSolIn, ShapeType.Face);
            stOut.Add(solOut, ShapeType.Face);
            if (stIn.IsEqual(stOut))
            {
                // The solids are the same. Replace the resulting solid in the result list
                // with the initial solid.
                NCollection_List<Shape>.Iterator itLS = new(solidsOut);
                for (; itLS.More(); itLS.Next())
                {
                    if (itLS.Value().IsSame(solOut))
                    {
                        solidsOut.InsertBefore(aSolIn, itLS);
                        solidsOut.Remove(itLS);
                        break;
                    }
                }
            }
            else
            {
                solidsHistory.AddModified(aSolIn, solOut);
            }
        }
    }

    // Stores the modified object into the list
    private static void TakeModified(in Shape s, BooleanBuilder builder, NCollection_List<Shape> list)
    {
        IReadOnlyList<Shape> modified = builder.Modified(s);
        if (modified.Count == 0 && !builder.IsDeleted(s))
        {
            list.Append(s);
        }
        else
        {
            foreach (Shape m in modified)
            {
                list.Append(m);
            }
        }
    }

    // Stores the modified object into the map
    private static void TakeModified(in Shape s, BooleanBuilder builder, NCollection_Map<Shape> map)
    {
        IReadOnlyList<Shape> modified = builder.Modified(s);
        if (modified.Count == 0 && !builder.IsDeleted(s))
        {
            map.Add(s);
        }
        else
        {
            foreach (Shape m in modified)
            {
                map.Add(m);
            }
        }
    }

    // Looks for the image of the solid in the list of resulting solids
    private static void FindSolid(in Shape solIn,
                                  NCollection_List<Shape> solidsRes,
                                  NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> adjFaces,
                                  BooleanBuilder builder,
                                  ref Shape solOut)
    {
        IntersectionContext? ctx = builder.Context();

        // Take the face in the IN solid, and find it in the OUT list
        Explorer expF = new(solIn, ShapeType.Face);
        for (; expF.More(); expF.Next())
        {
            Shape fs = expF.Current();
            // Images of the face
            NCollection_Map<Shape> mfsIm = new(ShapeHasher.Instance);
            if (adjFaces.FindFromKey(fs, out NCollection_List<Shape> pLFA))
            {
                NCollection_List<Shape>.Iterator itLFA = new(pLFA);
                for (; itLFA.More(); itLFA.Next())
                {
                    TakeModified(itLFA.Value(), builder, mfsIm);
                }
            }
            else
            {
                TakeModified(fs, builder, mfsIm);
            }

            // Find any of these faces in the list of solids
            NCollection_List<Shape>.Iterator itLS = new(solidsRes);
            for (; itLS.More(); itLS.Next())
            {
                Shape sol = itLS.Value();
                Explorer expFOut = new(sol, ShapeType.Face);
                for (; expFOut.More(); expFOut.Next())
                {
                    Shape f = expFOut.Current();
                    if (mfsIm.Contains(f))
                    {
                        // check orientations
                        if (!AlgoTools.IsSplitToReverse(f, fs, ctx))
                        {
                            solOut = sol;
                            return;
                        }
                    }
                }
            }
        }
    }
}
