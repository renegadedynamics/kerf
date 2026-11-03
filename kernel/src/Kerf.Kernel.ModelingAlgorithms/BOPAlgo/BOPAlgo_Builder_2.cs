// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>The same-domain check of a pair of faces, in the builder's parallel loop.</summary>
internal sealed class PairOfShapeBoolean : ParallelAlgo, BooleanParallel.IContextSolver<IntersectionContext>
{
    private bool _flag;
    private Shape _shape1 = new();
    private Shape _shape2 = new();
    private IntersectionContext? _context;

    public PairOfShapeBoolean()
    {
        _flag = false;
    }

    //
    public ref Shape Shape1() => ref _shape1;

    //
    public ref Shape Shape2() => ref _shape2;

    //
    public ref bool Flag() => ref _flag;

    //
    public void SetContext(IntersectionContext aContext) => _context = aContext;

    //
    public IntersectionContext? Context() => _context;

    //
    public override void Perform()
    {
        using var ps = new ProgressScope(_progressRange, null, 1);
        if (UserBreak(ps))
        {
            return;
        }

        Face fj = ShapeCasts.Face(_shape1);
        Face fk = ShapeCasts.Face(_shape2);
        _flag = AlgoTools.AreFacesSameDomain(fj, fk, _context!, _fuzzyValue);
    }
}

/// <summary>Extends BOPAlgo_BuilderFace with its own progress range, for the builder's parallel loop.</summary>
internal sealed class SplitFace : BuilderFace, BooleanParallel.ISolver
{
    private ProgressRange _range;

    //! Sets progress range
    public void SetProgressRange(ProgressRange range) => _range = range;

    // New perform method, using own progress range
    public void Perform()
    {
        using var ps = new ProgressScope(_range, null, 1);
        if (!ps.More())
        {
            return;
        }
        base.Perform(ps.Next());
    }

    //! Disables the range enabled method (private in OCCT; C# can't narrow an override)
    public override void Perform(ProgressRange range = default)
    {
    }
}

/// <summary>The classification of a vertex relatively a face, in the builder's parallel loop.</summary>
internal sealed class VFI : ParallelAlgo, BooleanParallel.IContextSolver<IntersectionContext>
{
    private bool _isInternal;
    private Vertex _v = new();
    private Face _f = new();
    private IntersectionContext? _context;

    public VFI()
    {
        _isInternal = false;
    }

    //
    public void SetVertex(in Vertex aV) => _v = aV;

    //
    public ref Vertex Vertex() => ref _v;

    //
    public void SetFace(in Face aF) => _f = aF;

    //
    public ref Face Face() => ref _f;

    //
    public bool IsInternal => _isInternal;

    //
    public void SetContext(IntersectionContext aContext) => _context = aContext;

    //
    public IntersectionContext? Context() => _context;

    //
    public override void Perform()
    {
        using var ps = new ProgressScope(_progressRange, null, 1);
        if (UserBreak(ps))
        {
            return;
        }

        double t1 = 0, t2 = 0, dummy = 0;
        //
        int iFlag = _context!.ComputeVF(_v, _f, ref t1, ref t2, ref dummy, _fuzzyValue);
        _isInternal = iFlag == 0;
    }
}

public partial class BooleanBuilder
{
    /// <summary>Fills the images of the faces: the split faces, the same-domain faces and the internal vertices.</summary>
    protected void FillImagesFaces(ProgressRange range)
    {
        using var ps = new ProgressScope(range, "Filling splits of faces", 10);
        BuildSplitFaces(ps.Next(9));
        if (HasErrors)
        {
            return;
        }
        FillSameDomainFaces(ps.Next(0.5));
        if (HasErrors)
        {
            return;
        }
        FillInternalVertices(ps.Next(0.5));
    }

    /// <summary>Builds the split faces of the faces having IN, ON or section pave blocks, or alone vertices.</summary>
    protected virtual void BuildSplitFaces(ProgressRange range)
    {
        bool bHasFaceInfo, bIsClosed, bIsDegenerated, bToReverse;
        int i, j, k, nbS, nbPBIn, nbPBOn, nbPBSc, nbAV, nSp;
        Face ff;
        Edge sp, ee;
        Orientation oriF, oriE;
        Explorer exp = new();
        NCollection_List<Shape>.Iterator it = new();
        NCollection_List<int> liav = new();
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        List<SplitFace> vbf = new();
        //
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~scope f
        NCollection_List<Shape> le = new();
        // (OCCT's unused map aMDE is left out)
        //
        using var psOuter = new ProgressScope(range, null, 10);
        // Build temporary map of faces images to avoid rebuilding
        // of the faces without any IN or section edges
        NCollection_IndexedDataMap<int, NCollection_List<Shape>> facesIm = new();
        //
        nbS = _ds!.SourceShapeCount;
        //
        for (i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() != ShapeType.Face)
            {
                continue;
            }
            if (UserBreak(psOuter))
            {
                return;
            }
            //
            Face f = ShapeCasts.Face(si.Shape());
            bool isUClosed = false, isVClosed = false, isChecked = false;
            //
            bHasFaceInfo = _ds.HasFaceInfo(i);
            if (!bHasFaceInfo)
            {
                continue;
            }
            //
            FaceInfo fi = _ds.FaceInfo(i);
            //
            NCollection_IndexedMap<PaveBlock> mpbIn = fi.PaveBlocksIn();
            NCollection_IndexedMap<PaveBlock> mpbOn = fi.PaveBlocksOn();
            NCollection_IndexedMap<PaveBlock> mpbSc = fi.PaveBlocksSc();
            liav.Clear();
            _ds.AloneVertices(i, liav);

            nbPBIn = mpbIn.Extent();
            nbPBOn = mpbOn.Extent();
            nbPBSc = mpbSc.Extent();
            nbAV = liav.Extent();
            if (nbPBIn == 0 && nbPBOn == 0 && nbPBSc == 0 && nbAV == 0)
            { // not compete
                continue;
            }

            if (nbPBIn == 0 && nbPBSc == 0)
            {
                // If there are any alone vertices to be put in the face,
                // the new face has to be created even if the wires of the
                // face have not been modified.

                // It is also necessary to check if the face contains any internal edges,
                // as such edges may split the face on parts and it is better
                // to send the face be treated by the BuilderFace algorithm.
                // In case of alone vertices the check for internals will be performed
                // in the BuildDraftFace method.
                bool hasInternals = false;
                if (nbAV == 0)
                {
                    // Check if any wires of the face have been modified.
                    // If no modified and internal wires present in the face
                    // there is no need to create the new face.
                    bool hasModified = false;

                    Iterator itW = new(f);
                    for (; itW.More(); itW.Next())
                    {
                        Iterator itE = new(itW.Value());
                        hasInternals = itE.More() && (itE.Value().Orientation() == Internal);
                        if (hasInternals)
                        {
                            break;
                        }

                        hasModified |= _images.ContainsKey(itW.Value());
                    }

                    if (!hasInternals && !hasModified)
                    {
                        continue;
                    }
                }

                if (!hasInternals)
                {
                    // No internal parts for the face, so just build the draft face
                    // and keep it to pass directly into result.
                    // If the original face has any internal edges or multi-connected vertices,
                    // the draft face will be null, as such sub-shapes may split the face on parts
                    // (as in the case "bugs modalg_5 bug25245_1").
                    // The BuilderFace algorithm will be called in this case.
                    Face fd = BuildDraftFace(f, _images, _context!, _report);
                    if (!fd.IsNull)
                    {
                        facesIm[facesIm.Add(i, new NCollection_List<Shape>())].Append(fd);
                        continue;
                    }
                }
            }

            mFence.Clear();
            //
            oriF = f.Orientation();
            ff = f;
            ff.Orientation(Forward);
            //
            // 1. Fill the edges set for the face aFF -> LE
            le.Clear();

            // 1.1 Bounding edges
            exp.Init(ff, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                oriE = e.Orientation();
                //
                if (!_images.ContainsKey(e))
                {
                    if (oriE == Internal)
                    {
                        ee = e;
                        ee.Orientation(Forward);
                        le.Append(ee);
                        ee.Orientation(Reversed);
                        le.Append(ee);
                    }
                    else
                    {
                        le.Append(e);
                    }

                    continue;
                }

                if (!isChecked)
                {
                    Surface surf = Tool.Surface(f)!;
                    global::Kerf.Engine.Geometry.Library.GeomLib.IsClosed(surf, Tool.Tolerance(e), out isUClosed, out isVClosed);

                    isChecked = true;
                }

                bIsClosed = false;

                if ((isUClosed || isVClosed) && Tool.IsClosed(e, f))
                {
                    AlgoTools2D.IsEdgeIsoline(e, f, out bool isUIso, out bool isVIso);

                    bIsClosed = (isUClosed && isUIso) || (isVClosed && isVIso);
                }

                bIsDegenerated = Tool.Degenerated(e);

                NCollection_List<Shape> lie = _images.GetValue(e);
                it.Initialize(lie);
                for (; it.More(); it.Next())
                {
                    sp = ShapeCasts.Edge(it.Value());
                    if (bIsDegenerated)
                    {
                        sp.Orientation(oriE);
                        le.Append(sp);
                        continue;
                    }
                    //
                    if (oriE == Internal)
                    {
                        sp.Orientation(Forward);
                        le.Append(sp);
                        sp.Orientation(Reversed);
                        le.Append(sp);
                        continue;
                    }
                    //
                    if (bIsClosed)
                    {
                        if (mFence.Add(sp))
                        {
                            if (!Tool.IsClosed(sp, f))
                            {
                                if (!AlgoTools3D.DoSplitSEAMOnFace(sp, f))
                                {
                                    // try different approach
                                    if (!AlgoTools3D.DoSplitSEAMOnFace(e, sp, f))
                                    {
                                        Compound ws = new();
                                        new BrepBuilder().MakeCompound(ref ws);
                                        new BrepBuilder().Add(ws, f);
                                        new BrepBuilder().Add(ws, sp);
                                        AddWarning(new AlertUnableToMakeClosedEdgeOnFace(ws));
                                    }
                                }
                            }
                            //
                            sp.Orientation(Forward);
                            le.Append(sp);
                            sp.Orientation(Reversed);
                            le.Append(sp);
                        } // if (aMFence.Add(aSp))
                        continue;
                    } // if (bIsClosed){
                    //
                    sp.Orientation(oriE);
                    bToReverse = AlgoTools.IsSplitToReverseWithWarn(sp, e, _context, _report);
                    if (bToReverse)
                    {
                        sp.Reverse();
                    }
                    le.Append(sp);
                } // for (; aIt.More(); aIt.Next()) {
            } // for (; aExp.More(); aExp.Next()) {
            //
            //
            // 1.2 In edges
            for (j = 1; j <= nbPBIn; ++j)
            {
                PaveBlock pb = mpbIn[j];
                nSp = pb.Edge();
                if (nSp < 0)
                {
                    // Standard_ASSERT: "Face information is not up to date" (a debug report in OCCT)
                    continue;
                }
                sp = ShapeCasts.Edge(_ds.Shape(nSp));
                //
                sp.Orientation(Forward);
                le.Append(sp);
                sp.Orientation(Reversed);
                le.Append(sp);
            }
            //
            //
            // 1.3 Section edges
            for (j = 1; j <= nbPBSc; ++j)
            {
                PaveBlock pb = mpbSc[j];
                nSp = pb.Edge();
                sp = ShapeCasts.Edge(_ds.Shape(nSp));
                //
                sp.Orientation(Forward);
                le.Append(sp);
                sp.Orientation(Reversed);
                le.Append(sp);
            }
            //
            if (!_paveFiller!.NonDestructive())
            {
                // speed up for planar faces
                BrepLibrary.BuildPCurveForEdgesOnPlane(le, ff);
            }
            // 3 Build split faces
            SplitFace bf =  new SplitFace();
            vbf.Add(bf);
            bf.SetFace(f);
            bf.SetShapes(le);
            bf.SetRunParallel(_runParallel);
            //
        } // for (i=0; i<aNbS; ++i) {

        // close preparation task
        psOuter.Next();
        //
        int nbBF = vbf.Count;
        // Set progress range for each task to be run in parallel
        using var psParallel = new ProgressScope(psOuter.Next(9), "Splitting faces", nbBF);
        for (int iF = 0; iF < nbBF; iF++)
        {
            SplitFace bf = vbf[iF];
            bf.SetProgressRange(psParallel.Next());
        }
        //===================================================
        BooleanParallel.Perform(_runParallel, vbf);
        //===================================================
        if (UserBreak(psOuter))
        {
            return;
        }
        for (k = 0; k < nbBF; ++k)
        {
            BuilderFace bf = vbf[k];
            facesIm.Add(_ds.Index(bf.Face()), new NCollection_List<Shape>(bf.Areas()));
            _report.Merge(bf.GetReport());
        }

        nbBF = facesIm.Extent();
        for (k = 1; k <= nbBF; ++k)
        {
            Face f = ShapeCasts.Face(_ds.Shape(facesIm.FindKey(k)));
            oriF = f.Orientation();
            NCollection_List<Shape> lfr = facesIm[k];
            //
            NCollection_List<Shape> pLFIm = _images.SetValueRef(f, new NCollection_List<Shape>());
            it.Initialize(lfr);
            for (; it.More(); it.Next())
            {
                ref Shape fr = ref it.ChangeValue();
                if (oriF == Reversed)
                {
                    fr.Orientation(Reversed);
                }
                pLFIm.Append(fr);
            }
        }
        //
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~scope t
    }

    // Adds the set of the edges of the shape to the map, keeping the connection to the shape.
    private static void AddEdgeSet(in Shape s, NCollection_IndexedDataMap<ToolSet, NCollection_List<Shape>> map)
    {
        // Make set
        ToolSet se = new();
        se.Add(s, ShapeType.Edge);
        // Add set to the map, keeping connection to the shape
        int index = map.FindIndex(se);
        NCollection_List<Shape> pLF = index > 0
                                                 ? map[index]
                                                 : map[map.Add(se, new NCollection_List<Shape>())];
        pLF.Append(s);
    }

    /// <summary>Finds the same-domain faces among the faces having interfered, and makes one represent each group.</summary>
    protected void FillSameDomainFaces(ProgressRange range)
    {
        // It is necessary to analyze all Face/Face intersections
        // and find all faces with equal sets of edges
        List<InterfFF> fFs = _ds!.InterfFF();
        int nbFFs = fFs.Count;
        if (nbFFs == 0)
        {
            return;
        }

        using var psOuter = new ProgressScope(range, null, 10);

        // Two distinct faces of one solid cannot be Same-Domain: that would
        // imply a zero-thickness interior in a single operand.
        Dictionary<Shape, Shape> faceToParent = new(ShapeHasher.Instance);
        {
            int nbSrc = _ds.SourceShapeCount;
            for (int iSrc = 0; iSrc < nbSrc; ++iSrc)
            {
                ShapeInfo si = _ds.ShapeInfo(iSrc);
                if (si.ShapeType() != ShapeType.Solid)
                {
                    continue;
                }
                Shape solid = si.Shape();
                for (Explorer expF = new(solid, ShapeType.Face); expF.More(); expF.Next())
                {
                    Shape f = expF.Current();
                    if (!faceToParent.ContainsKey(f))
                    {
                        faceToParent[f] = solid;
                    }
                }
            }
            Dictionary<Shape, Shape> propagation = new(ShapeHasher.Instance);
            // OCCT iterates the images map, a shape-keyed map, in address order (the port: insertion order); a split
            // face belongs to one face's images, so the order isn't observable here
            foreach (KeyValuePair<Shape, NCollection_List<Shape>> itIm in _images)
            {
                if (!faceToParent.TryGetValue(itIm.Key, out Shape pParent))
                {
                    continue;
                }
                for (NCollection_List<Shape>.Iterator itPiece = new(itIm.Value); itPiece.More(); itPiece.Next())
                {
                    if (!faceToParent.ContainsKey(itPiece.Value()))
                    {
                        propagation[itPiece.Value()] = pParent;
                    }
                }
            }
            foreach (KeyValuePair<Shape, Shape> itProp in propagation)
            {
                faceToParent[itProp.Key] = itProp.Value;
            }
        }

        // Vector to store the indices of faces for future sorting
        // for making the SD face for the group from the face with
        // smallest index in Data structure
        List<int> fiVec = new();
        // Fence map to avoid repeated checks of the same face.
        NCollection_IntMap mFence = new(1);

        // Fill the vector with indices of faces
        Span<int> nF2 = stackalloc int[2]; // OCCT's nF[2], renamed: C# can't redeclare nF below
        for (int i = 0; i < nbFFs; ++i)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            InterfFF ff = fFs[i];
            // get indices
            ff.Indices(out nF2[0], out nF2[1]);
            // store indices to the vector
            for (int j = 0; j < 2; ++j)
            {
                if (!_ds.HasFaceInfo(nF2[j]))
                {
                    continue;
                }

                if (!mFence.Add(nF2[j]))
                {
                    continue;
                }

                fiVec.Add(nF2[j]);
            }
        }

        // Sort the indices (distinct, so any sort gives std::sort's order)
        int[] fiArr = [.. fiVec];
        Array.Sort(fiArr);
        for (int i = 0; i < fiArr.Length; ++i)
        {
            fiVec[i] = fiArr[i];
        }

        // Data map of set of edges with all faces having this set
        NCollection_IndexedDataMap<ToolSet, NCollection_List<Shape>> eSetFaces = new();
        // Map of planar bounded faces. If such faces have the same Edge set
        // they are considered Same domain, without additional check.
        HashSet<Shape> mfPlanar = new(ShapeHasher.Instance);

        int nbF = fiVec.Count;
        for (int i = 0; i < nbF; ++i)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            int nF = fiVec[i];
            ShapeInfo si = _ds.ShapeInfo(nF);
            Shape f = si.Shape();

            bool bCheckPlanar = false;
            {
                // At this stage, context should contain adaptor for all intersected faces,
                // so getting a type of the underlying surface should be done at no cost.
                if (_context!.SurfaceAdaptor(ShapeCasts.Face(f)).GetType() == Plane)
                {
                    // Check bounding box of the face - it should not be open in any side
                    Box box = si.Box();
                    bCheckPlanar = !(box.IsOpenXmin || box.IsOpenXmax || box.IsOpenYmin
                                     || box.IsOpenYmax || box.IsOpenZmin || box.IsOpenZmax);
                }
            }

            if (_images.TryGetValue(f, out NCollection_List<Shape>? pLFSp))
            {
                NCollection_List<Shape>.Iterator itLF = new(pLFSp);
                for (; itLF.More(); itLF.Next())
                {
                    AddEdgeSet(itLF.Value(), eSetFaces);
                    if (bCheckPlanar)
                    {
                        mfPlanar.Add(itLF.Value());
                    }
                }
            }
            else
            {
                AddEdgeSet(f, eSetFaces);
                if (bCheckPlanar)
                {
                    mfPlanar.Add(f);
                }
            }
        }

        // Store pairs of faces with equal set of edges to check if they are really Same Domain
        List<PairOfShapeBoolean> vpsb = new();

        // Back and forth map of SD faces to make the blocks
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> dmsls = new(ShapeHasher.Instance);

        int nbSets = eSetFaces.Extent();
        for (int i = 1; i <= nbSets; ++i)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            NCollection_List<Shape> lf = eSetFaces[i];
            if (lf.Extent() < 2)
            {
                continue;
            }

            // All possible pairs from <aLF> should be checked
            NCollection_List<Shape>.Iterator it1 = new(lf);
            for (; it1.More(); it1.Next())
            {
                Shape f1 = it1.Value();
                bool bCheckPlanar = mfPlanar.Contains(f1);
                bool hasParent1 = faceToParent.TryGetValue(f1, out Shape pParent1);

                NCollection_List<Shape>.Iterator it2 = new();
                it2.Reset(lf, it1.Index); // aIt2 = aIt1
                for (it2.Next(); it2.More(); it2.Next())
                {
                    Shape f2 = it2.Value();
                    bool hasParent2 = faceToParent.TryGetValue(f2, out Shape pParent2);
                    if (hasParent1 && hasParent2 && pParent1.IsSame(pParent2))
                    {
                        continue;
                    }
                    if (bCheckPlanar && mfPlanar.Contains(f2))
                    {
                        // Consider planar bounded faces as Same Domain without additional check
                        BuilderTools.FillMap(f1, f2, dmsls);
                        continue;
                    }
                    // Add pair for analysis
                    PairOfShapeBoolean psb =  new PairOfShapeBoolean();
                    vpsb.Add(psb);
                    psb.Shape1() = f1;
                    psb.Shape2() = f2;
                    psb.SetFuzzyValue(_fuzzyValue);
                }
            }
        }

        psOuter.Next();

        // Set progress range for each task to be run in parallel
        using (var psParallel = new ProgressScope(psOuter.Next(6), "Checking SD faces", vpsb.Count))
        {
            for (int iPSB = 0; iPSB < vpsb.Count; ++iPSB)
            {
                vpsb[iPSB].SetProgressRange(psParallel.Next());
            }
            //================================================================
            // Perform analysis
            BooleanParallel.Perform(_runParallel, vpsb, _context!);
            //================================================================
        }
        if (UserBreak(psOuter))
        {
            return;
        }

        NCollection_List<NCollection_List<Shape>> mBlocks = new();
        // Fill map with SD faces to make the blocks
        int nbPairs = vpsb.Count;
        for (int i = 0; i < nbPairs; ++i)
        {
            PairOfShapeBoolean psb = vpsb[i];
            if (psb.Flag())
            {
                BuilderTools.FillMap(psb.Shape1(), psb.Shape2(), dmsls);
            }
        }
        vpsb.Clear();

        // Make blocks of SD faces using the back and forth map
        BuilderTools.MakeBlocks(dmsls, mBlocks, ShapeHasher.Instance);

        using var ps = new ProgressScope(psOuter.Next(3), "Filling same domain faces map", mBlocks.Extent());
        // Fill same domain faces map
        NCollection_List<NCollection_List<Shape>>.Iterator itB = new(mBlocks);
        for (; itB.More(); itB.Next(), ps.Next())
        {
            if (UserBreak(ps))
            {
                return;
            }
            NCollection_List<Shape> lsd = itB.Value();
            // If the group contains some original faces, the one with minimal
            // index in the DS will be chosen as the SD for the whole group.
            // If there are no original faces in the group, the first face from
            // the group will be used as the SD face.
            // Such SD face will be representative of the whole group in the result.
            Shape? pFSD = null;
            int nFMin = IntegerLast();
            NCollection_List<Shape>.Iterator itLF = new(lsd);
            for (; itLF.More(); itLF.Next())
            {
                Shape f = itLF.Value();
                // Check the index of the face in DS
                int nF = _ds.Index(f);
                if (nF >= 0)
                {
                    // The fact that the face is found in the DS, means that
                    // the face has not been change, and thus it is original one.
                    //
                    // Such face does not have any splits, but have an SD face.
                    // Consider it being split.
                    _images.SetValueRef(f, new NCollection_List<Shape>()).Append(f);

                    // For the SD face chose the one with minimal index
                    if (nF < nFMin)
                    {
                        nFMin = nF;
                        pFSD = f;
                    }
                }
            }

            if (pFSD is null)
            {
                // No original faces in the group, take the first one
                pFSD = lsd.First();
            }

            // Save all SD connections
            itLF.Initialize(lsd);
            for (; itLF.More(); itLF.Next())
            {
                Shape f = itLF.Value();
                _shapesSD[f] = pFSD.Value;
            }
        }

        // Update the map of images with SD faces and
        // fill the map of origins.
        int nbS = _ds.SourceShapeCount;
        for (int i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() != ShapeType.Face)
            {
                continue;
            }

            Shape f = si.Shape();
            if (!_images.TryGetValue(f, out NCollection_List<Shape>? pLFIm))
            {
                continue;
            }

            NCollection_List<Shape>.Iterator itLFIm = new(pLFIm);
            for (; itLFIm.More(); itLFIm.Next())
            {
                ref Shape fIm = ref itLFIm.ChangeValue();
                if (_shapesSD.TryGetValue(fIm, out Shape pFSD))
                {
                    // Update image with SD face
                    fIm = pFSD;
                }

                // Fill the map of origins
                if (!_origins.TryGetValue(fIm, out NCollection_List<Shape>? pLFOr))
                {
                    pLFOr = _origins.SetValueRef(fIm, new NCollection_List<Shape>());
                }
                pLFOr.Append(f);
            }
        }

        mBlocks.Clear();
        dmsls.Clear();
    }

    /// <summary>Adds the alone vertices of the faces into their splits as internal vertices.</summary>
    protected void FillInternalVertices(ProgressRange range)
    {
        using var psOuter = new ProgressScope(range, null, 1);

        // Vector of pairs of Vertex/Face for classification of the vertices
        // relatively faces, and adding them as internal into the faces
        List<VFI> vvfi = new();

        int nbS = _ds!.SourceShapeCount;
        for (int i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() != ShapeType.Face)
            {
                continue;
            }

            if (UserBreak(psOuter))
            {
                return;
            }

            Shape f = si.Shape();
            if (!_images.TryGetValue(f, out NCollection_List<Shape>? pLFIm))
            {
                continue;
            }

            // Find vertices to add as internal into the splits
            NCollection_List<int> liav = new();
            _ds.AloneVertices(i, liav);

            // Add vertices and faces for classification
            NCollection_List<int>.Iterator itLV = new(liav);
            for (; itLV.More(); itLV.Next())
            {
                Vertex v = ShapeCasts.Vertex(_ds.Shape(itLV.Value()));
                v.Orientation(Internal);

                NCollection_List<Shape>.Iterator itLFIm = new(pLFIm);
                for (; itLFIm.More(); itLFIm.Next())
                {
                    Face fIm = ShapeCasts.Face(itLFIm.Value());
                    // Make the pair
                    VFI vfi =  new VFI();
                    vvfi.Add(vfi);
                    vfi.SetVertex(v);
                    vfi.SetFace(fIm);
                    vfi.SetFuzzyValue(_fuzzyValue);
                }
            }
        }

        // Set progress range for each task to be run in parallel
        using (var psParallel = new ProgressScope(psOuter.Next(), "Looking for internal shapes", vvfi.Count))
        {
            for (int iVFI = 0; iVFI < vvfi.Count; ++iVFI)
            {
                vvfi[iVFI].SetProgressRange(psParallel.Next());
            }
            // Perform classification
            //================================================================
            BooleanParallel.Perform(_runParallel, vvfi, _context!);
            //================================================================
        }
        if (UserBreak(psOuter))
        {
            return;
        }

        int nbVFI = vvfi.Count;
        for (int i = 0; i < nbVFI; ++i)
        {
            VFI vfi = vvfi[i];
            if (vfi.IsInternal)
            {
                Vertex v = vfi.Vertex();
                Face f = vfi.Face();
                // OCCT adds the vertex into the split face, which may be an argument's face kept as the
                // same-domain face of a group (a frozen argument face then throws, unit 6F)
                new BrepBuilder().Add(f, v);
            }
        }
    }

    // Checks if the edge has multi-connected vertices.
    private static bool HasMultiConnected(in Edge edge, Dictionary<Shape, NCollection_List<Shape>> map)
    {
        Iterator itV = new(edge);
        for (; itV.More(); itV.Next())
        {
            Shape v = itV.Value();
            if (!map.TryGetValue(v, out NCollection_List<Shape>? pList))
            {
                pList = map.SetValueRef(v, new NCollection_List<Shape>());
                pList.Append(edge);
            }
            else
            {
                // The list is expected to be 1-2 elements long,
                // thus using "Contains" is safe.
                if (!pList.Contains(edge))
                {
                    pList.Append(edge);
                }

                if (pList.Extent() > 2)
                {
                    return true;
                }
            }
        }
        return false;
    }

    // Builds the draft face, updating the bounding edges according to the information stored into the images map.
    private static Face BuildDraftFace(in Face face,
                                             Dictionary<Shape, NCollection_List<Shape>> images,
                                             IntersectionContext ctx,
                                             Report report)
    {
        BrepBuilder bb = new();
        // Take the information from the original face
        Surface? s = Tool.Surface(face, out Placement loc);
        double tol = Tool.Tolerance(face);
        // Make the new face, without any wires
        Face draftFace = new();
        bb.MakeFace(ref draftFace, s, loc, tol);

        // Check if the thin face can be split by a vertex - in this case
        // this vertex will be contained in more than two edges. Thus, count
        // the vertices appearance, and if the multi-connexity is met return
        // the null face to use the BuilderFace algorithm for checking the
        // possibility of split.
        Dictionary<Shape, NCollection_List<Shape>> verticesCounter = new(ShapeHasher.Instance);

        // Check that the edges of the initial face have not been unified during intersection.
        // Otherwise, it will be necessary to check validity of the new wires.
        HashSet<Shape> mEdges = new(ShapeHasher.Instance);

        // Update wires of the original face and add them to draft face
        Iterator itW = new(face.Oriented(Forward));
        for (; itW.More(); itW.Next())
        {
            Shape w = itW.Value();
            if (w.ShapeType() != ShapeType.Wire)
            {
                continue;
            }

            // Rebuild wire using images of edges
            Iterator itE = new(w.Oriented(Forward));
            if (!itE.More())
            {
                continue;
            }

            Wire newWire = new();
            bb.MakeWire(ref newWire);

            for (; itE.More(); itE.Next())
            {
                Edge e = ShapeCasts.Edge(itE.Value());

                Orientation oriE = e.Orientation();
                if (oriE == Internal)
                {
                    // The internal edges could split the original face on halves.
                    // Thus, use the BuilderFace algorithm to build the new face.
                    return new Face();
                }

                // Check if the original edge is degenerated
                bool bIsDegenerated = Tool.Degenerated(e);
                // Check if the original edge is closed on the face
                bool bIsClosed = Tool.IsClosed(e, face);

                // Check for the splits of the edge
                if (!images.TryGetValue(e, out NCollection_List<Shape>? pLEIm))
                {
                    // Check if the edge has multi-connected vertices
                    if (!bIsDegenerated && HasMultiConnected(e, verticesCounter))
                    {
                        return new Face();
                    }

                    // Check edges unification
                    if (!bIsClosed && !mEdges.Add(e))
                    {
                        return new Face();
                    }

                    bb.Add(newWire, e);
                    continue;
                }

                NCollection_List<Shape>.Iterator itLEIm = new(pLEIm);
                for (; itLEIm.More(); itLEIm.Next())
                {
                    // OCCT changes the orientation of the split in the images map itself (ChangeValue through the
                    // map's const reference); kept
                    ref Shape sp = ref itLEIm.ChangeValue();

                    // Check if the split has multi-connected vertices
                    if (!bIsDegenerated && HasMultiConnected(ShapeCasts.Edge(sp), verticesCounter))
                    {
                        return new Face();
                    }

                    // Check edges unification
                    if (!bIsClosed && !mEdges.Add(sp))
                    {
                        return new Face();
                    }

                    sp.Orientation(oriE);
                    if (bIsDegenerated)
                    {
                        bb.Add(newWire, sp);
                        continue;
                    }

                    // If the original edge is closed on the face check closeness
                    // of the split edge and if it is not closed make the second PCurve
                    if (bIsClosed && !Tool.IsClosed(ShapeCasts.Edge(sp), face))
                    {
                        AlgoTools3D.DoSplitSEAMOnFace(ShapeCasts.Edge(sp), face);
                    }

                    // Check if the split should be reversed
                    if (AlgoTools.IsSplitToReverseWithWarn(sp, e, ctx, report))
                    {
                        sp.Reverse();
                    }

                    bb.Add(newWire, sp);
                }
            }

            newWire.Orientation(w.Orientation());
            newWire.Closed(Tool.IsClosed(newWire));
            bb.Add(draftFace, newWire);
        }

        if (face.Orientation() == Reversed)
        {
            draftFace.Reverse();
        }

        return draftFace;
    }
}
