// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_Builder_3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans;

/// <summary>Extends BOPAlgo_BuilderSolid with the solid to split and its own progress range, for the builder's parallel loop.</summary>
internal sealed class SplitSolid : BuilderSolid, BooleanParallel.ISolver
{
    private Solid _solid = new(); //!< Solid to split
    private ProgressRange _range;

    //! Sets the solid
    public void SetSolid(in Solid solid) => _solid = solid;

    //! Returns the solid
    public Solid Solid() => _solid;

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

public partial class BooleanBuilder
{
    /// <summary>Fills the images of the solids: the split solids with their internal parts.</summary>
    protected void FillImagesSolids(ProgressRange range)
    {
        int i = 0, nbS = _ds!.SourceShapeCount;
        for (i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() == ShapeType.Solid)
            {
                break;
            }
        }
        if (i >= nbS)
        {
            return;
        }

        using var ps = new ProgressScope(range, "Building splits of solids", 10);
        // Draft solids
        Dictionary<Shape, Shape> draftSolids = new(ShapeHasher.Instance);
        // Find all IN faces for all IN faces
        FillIn3DParts(draftSolids, ps.Next(4));
        if (HasErrors)
        {
            return;
        }
        // Build split of the solids
        BuildSplitSolids(draftSolids, ps.Next(5));
        if (HasErrors)
        {
            return;
        }
        // Fill solids with internal parts
        FillInternalShapes(ps.Next());
    }

    /// <summary>Builds the draft solids and finds the faces IN each solid (its own internal ones and others').</summary>
    protected virtual void FillIn3DParts(Dictionary<Shape, Shape> draftSolids, ProgressRange range)
    {
        using var ps = new ProgressScope(range, null, 2);

        // Find all faces that are IN solids

        // Store boxes of the shapes into a map
        Dictionary<Shape, Box> shapeBoxMap = new(ShapeHasher.Instance);

        // Fence map
        HashSet<Shape> mFence = new(ShapeHasher.Instance);

        // Get all faces
        NCollection_List<Shape> lFaces = new();

        int i, nbS = _ds!.SourceShapeCount;
        for (i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() != ShapeType.Face)
            {
                continue;
            }

            if (UserBreak(ps))
            {
                return;
            }

            Shape s = si.Shape();

            if (_images.TryGetValue(s, out NCollection_List<Shape>? pLSIm))
            {
                NCollection_List<Shape>.Iterator itLSIm = new(pLSIm);
                for (; itLSIm.More(); itLSIm.Next())
                {
                    Shape sIm = itLSIm.Value();
                    if (mFence.Add(sIm))
                    {
                        lFaces.Append(sIm);
                    }
                }
            }
            else
            {
                lFaces.Append(s);
                shapeBoxMap[s] = si.Box();
            }
        }

        BrepBuilder bb = new();

        // Get all solids
        NCollection_List<Shape> lSolids = new();
        // Keep INTERNAL faces of the solids
        Dictionary<Shape, NCollection_List<Shape>> solidsIF = new(ShapeHasher.Instance);
        // Draft solids
        IndexedMap<Shape, Shape> draftSolid = new(ShapeHasher.Instance);

        for (i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ChangeShapeInfo(i);
            if (si.ShapeType() != ShapeType.Solid)
            {
                continue;
            }
            if (UserBreak(ps))
            {
                return;
            }
            Shape s = si.Shape();
            Solid solid = ShapeCasts.Solid(s);
            //
            // Bounding box for the solid aS
            ref Box boxS = ref si.ChangeBox();
            if (boxS.IsVoid)
            {
                _ds.BuildBndBoxSolid(i, ref boxS, _checkInverted);
            }

            // Build Draft Solid
            NCollection_List<Shape> lif = new();
            Solid sd0 = new();
            bb.MakeSolid(ref sd0);
            Shape sd = sd0;
            BuildDraftSolid(solid, ref sd, lif);

            lSolids.Append(sd);
            solidsIF[sd] = lif;
            shapeBoxMap[sd] = boxS;
            draftSolid.Add(s, sd);
        }

        // Perform classification of the faces
        IndexedMap<Shape, NCollection_List<Shape>> inParts = new(ShapeHasher.Instance);

        BuilderTools.ClassifyFaces(lFaces,
                                    lSolids,
                                    _runParallel,
                                    _context!,
                                    inParts,
                                    shapeBoxMap,
                                    solidsIF,
                                    ps.Next());

        // Analyze the results of classification
        int nbSol = draftSolid.Count;
        for (i = 1; i <= nbSol; ++i)
        {
            if (UserBreak(ps))
            {
                return;
            }
            Solid solid = ShapeCasts.Solid(draftSolid.KeyAt(i - 1));
            Solid sDraft = ShapeCasts.Solid(draftSolid.ValueAt(i - 1));
            NCollection_List<Shape> lInFaces = inParts.GetValue(sDraft);
            NCollection_List<Shape> lInternal = solidsIF.GetValue(sDraft);

            int nbIN = lInFaces.Extent();

            if (nbIN == 0)
            {
                bool bHasImage = false;
                // Check if the shells of the solid have image
                for (Iterator it = new(solid); it.More() && !bHasImage; it.Next())
                {
                    bHasImage = _images.ContainsKey(it.Value());
                }

                if (!bHasImage)
                {
                    // no need to split the solid
                    continue;
                }
            }

            draftSolids[solid] = sDraft;

            int nbInt = lInternal.Extent();
            if (nbInt != 0 || nbIN != 0)
            {
                // Combine the lists
                NCollection_List<Shape> pLIN = _inParts.SetValueRef(solid, new NCollection_List<Shape>());

                NCollection_List<Shape>.Iterator itLS = new(lInFaces);
                for (; itLS.More(); itLS.Next())
                {
                    pLIN.Append(itLS.Value());
                }

                itLS.Initialize(lInternal);
                for (; itLS.More(); itLS.Next())
                {
                    pLIN.Append(itLS.Value());
                }
            }
        }
    }

    /// <summary>Builds the draft solid of the solid from the images of its faces, collecting its internal faces.</summary>
    protected void BuildDraftSolid(in Shape solid, ref Shape draftSolid, NCollection_List<Shape> lif)
    {
        bool bToReverse;
        int iFlag;
        Orientation orF, orSh, orSd;
        Iterator it1 = new(), it2 = new();
        Shell shD = new();
        Shape fx;
        BrepBuilder bb = new();
        NCollection_List<Shape>.Iterator itS = new();
        //
        orSd = solid.Orientation;
        draftSolid.Orient(orSd);
        //
        it1.Initialize(solid);
        for (; it1.More(); it1.Next())
        {
            Shape sh = it1.Value();
            if (sh.Type != ShapeType.Shell)
            {
                continue; // mb internal edges,vertices
            }
            //
            orSh = sh.Orientation;
            bb.MakeShell(ref shD);
            shD.Orient(orSh);
            iFlag = 0;
            //
            it2.Initialize(sh);
            for (; it2.More(); it2.Next())
            {
                Shape f = it2.Value();
                orF = f.Orientation;
                //
                if (_images.ContainsKey(f))
                {
                    NCollection_List<Shape> lSp = _images.GetValue(f);
                    itS.Initialize(lSp);
                    for (; itS.More(); itS.Next())
                    {
                        fx = itS.Value();
                        //
                        if (_shapesSD.ContainsKey(fx))
                        {
                            //
                            if (orF == Internal)
                            {
                                fx.Orient(orF);
                                lif.Append(fx);
                            }
                            else
                            {
                                bToReverse = AlgoTools.IsSplitToReverseWithWarn(fx, f, _context, _report);
                                if (bToReverse)
                                {
                                    fx.Reverse();
                                }
                                //
                                iFlag = 1;
                                bb.Add(shD, fx);
                            }
                        } // if (myShapesSD.IsBound(aFx)) {
                        else
                        {
                            fx.Orient(orF);
                            if (orF == Internal)
                            {
                                lif.Append(fx);
                            }
                            else
                            {
                                iFlag = 1;
                                bb.Add(shD, fx);
                            }
                        }
                    }
                } // if (myImages.IsBound(aF)) {
                //
                else
                {
                    if (orF == Internal)
                    {
                        lif.Append(f);
                    }
                    else
                    {
                        iFlag = 1;
                        bb.Add(shD, f);
                    }
                }
            } // for (; aIt2.More(); aIt2.Next()) {
            //
            if (iFlag != 0)
            {
                shD.Closed(Tool.IsClosed(shD));
                bb.Add(draftSolid, shD);
            }
        } // for (; aIt1.More(); aIt1.Next()) {
    }

    /// <summary>Builds the split solids from the draft solids and their IN faces.</summary>
    protected void BuildSplitSolids(Dictionary<Shape, Shape> draftSolids, ProgressRange range)
    {
        bool bFlagSD;
        int i, nbS;
        Explorer exp = new();
        NCollection_List<Shape>.Iterator it = new();
        //
        NCollection_List<Shape> sfs = new();
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        // OCCT's NCollection_Map<BOPTools_Set>, of which Added() returns the key in the map: an indexed map gives
        // the stored key back from its index, and the map is never iterated
        IndexedSet<ToolSet> mst = new();
        List<SplitSolid> vbs = new();
        //
        using var psOuter = new ProgressScope(range, null, 10);
        // 0. Find same domain solids for non-interfered solids
        nbS = _ds!.SourceShapeCount;
        for (i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            //
            if (si.ShapeType() != ShapeType.Solid)
            {
                continue;
            }
            if (UserBreak(psOuter))
            {
                return;
            }
            //
            Shape s = si.Shape();
            if (!mFence.Add(s))
            {
                continue;
            }
            if (draftSolids.ContainsKey(s))
            {
                continue;
            }
            //
            ToolSet st = new();
            //
            st.Add(s, ShapeType.Face);
            mst.Add(st);
            //
        } // for (i=1; i<=aNbS; ++i)
        //
        // Build temporary map of solids images to avoid rebuilding
        // of the solids without internal faces
        IndexedMap<Shape, NCollection_List<Shape>> solidsIm = new(ShapeHasher.Instance);
        // 1. Build solids for interfered source solids
        for (i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() != ShapeType.Solid)
            {
                continue;
            }

            Shape s = si.Shape();
            Solid solid = ShapeCasts.Solid(s);
            if (!draftSolids.ContainsKey(s))
            {
                continue;
            }

            Shape sd = draftSolids.GetValue(s);
            if (!_inParts.TryGetValue(s, out NCollection_List<Shape>? pLFIN) || pLFIN.IsEmpty())
            {
                solidsIm.ValueAt(solidsIm.Add(s, new NCollection_List<Shape>())).Append(sd);
                continue;
            }

            sfs.Clear();
            //
            // 1.1 Fill Shell Faces Set
            exp.Init(sd, ShapeType.Face);
            for (; exp.More(); exp.Next())
            {
                Shape f = exp.Current();
                sfs.Append(f);
            }
            //
            // 1.2 Fill internal faces
            it.Initialize(pLFIN);
            for (; it.More(); it.Next())
            {
                Shape f = it.Value();
                //
                f.Orient(Forward);
                sfs.Append(f);
                f.Orient(Reversed);
                sfs.Append(f);
            }
            //
            // 1.3 Build new solids
            SplitSolid bs =  new SplitSolid();
            vbs.Add(bs);
            bs.SetSolid(solid);
            bs.SetShapes(sfs);
            bs.SetRunParallel(_runParallel);
        } // for (i=0; i<aNbS; ++i) {
        //
        int k, nbBS;
        //
        nbBS = vbs.Count;
        // Set progress range for each task to be run in parallel
        using (var psParallel = new ProgressScope(psOuter.Next(9), "Splitting solids", nbBS))
        {
            for (int iS = 0; iS < nbBS; iS++)
            {
                SplitSolid splitSolid = vbs[iS];
                splitSolid.SetProgressRange(psParallel.Next());
            }
            //
            //===================================================
            BooleanParallel.Perform(_runParallel, vbs);
            //===================================================
        }
        if (UserBreak(psOuter))
        {
            return;
        }
        //
        for (k = 0; k < nbBS; ++k)
        {
            SplitSolid bs = vbs[k];
            solidsIm.Add(bs.Solid(), new NCollection_List<Shape>(bs.Areas()));

            // Merge BuilderSolid's report into main report,
            // assigning the solid with the warnings/errors which
            // have been generated for it.
            // Convert all errors of BuilderSolid into warnings for main report.
            Report bsReport = bs.GetReport();
            Span<Gravity> alertTypes = [Warning, Fail];
            for (int iGravity = 0; iGravity < 2; iGravity++)
            {
                NCollection_List<Alert> lAlerts = bsReport.GetAlerts(alertTypes[iGravity]);
                for (NCollection_List<Alert>.Iterator itA = new(lAlerts); itA.More(); itA.Next())
                {
                    Alert alert = itA.Value();

                    if (itA.Value() is AlertWithShape alertWithShape)
                    {
                        Compound warnShape0 = new();
                        new BrepBuilder().MakeCompound(ref warnShape0);
                        Shape warnShape = warnShape0;
                        new BrepBuilder().Add(warnShape, bs.Solid());
                        new BrepBuilder().Add(warnShape, alertWithShape.InternalGetShape());

                        alertWithShape.SetShape(warnShape);
                        AddWarning(alertWithShape);
                    }
                    else
                    {
                        AddWarning(alert);
                    }
                }
            }
        }
        //
        // Add new solids to images map
        nbBS = solidsIm.Count;
        for (k = 1; k <= nbBS; ++k)
        {
            Shape s = solidsIm.KeyAt(k - 1);
            NCollection_List<Shape> lsr = solidsIm.ValueAt(k - 1);
            //
            if (!_images.ContainsKey(s))
            {
                NCollection_List<Shape> pLSx = _images.SetValueRef(s, new NCollection_List<Shape>());
                //
                it.Initialize(lsr);
                for (; it.More(); it.Next())
                {
                    ToolSet st = new();
                    //
                    Shape sr = it.Value();
                    st.Add(sr, ShapeType.Face);
                    //
                    bFlagSD = mst.Contains(st);
                    //
                    ToolSet sTx = mst[mst.Add(st)]; // OCCT: aMST.Added(aST)
                    Shape sx = sTx.Shape();
                    pLSx.Append(sx);
                    //
                    if (!_origins.TryGetValue(sx, out NCollection_List<Shape>? pLOr))
                    {
                        pLOr = _origins.SetValueRef(sx, new NCollection_List<Shape>());
                    }
                    pLOr.Append(s);
                    //
                    if (bFlagSD)
                    {
                        _shapesSD[sr] = sx;
                    }
                }
            }
        }
    }

    /// <summary>Settles the internal vertices and edges of the arguments into the split solids containing them.</summary>
    protected void FillInternalShapes(ProgressRange range)
    {
        int i, j, nbS, nbSI, nbSx;
        ShapeType type;
        Containment state;
        Iterator itS = new();
        BrepBuilder bb = new();
        NCollection_List<Shape>.Iterator it = new(), it1 = new();
        //
        //-----------------------------------------------------scope f
        IndexedMap<Shape, NCollection_List<Shape>> mSx = new(ShapeHasher.Instance);
        IndexedSet<Shape> mx = new(ShapeHasher.Instance);
        IndexedSet<Shape> msi = new(ShapeHasher.Instance);
        HashSet<Shape> mFence = new(ShapeHasher.Instance);
        HashSet<Shape> msOr = new(ShapeHasher.Instance);
        NCollection_List<Shape> lSd = new();
        NCollection_List<Shape> lArgs = new();
        NCollection_List<Shape> lsc = new();
        NCollection_List<Shape> lsi = new();

        using var ps = new ProgressScope(range, null, 10);
        //
        // 1. Shapes to process
        //
        // 1.1 Shapes from pure arguments aMSI
        // 1.1.1 vertex, edge, wire
        //
        NCollection_List<Shape> arguments = _ds!.Arguments();
        it.Initialize(arguments);
        for (; it.More(); it.Next())
        {
            Shape s = it.Value();
            AlgoTools.TreatCompound(s, lsc, mFence);
        }
        it.Initialize(lsc);
        for (; it.More(); it.Next())
        {
            Shape s = it.Value();
            type = s.Type;
            if (type == ShapeType.Wire)
            {
                itS.Initialize(s);
                for (; itS.More(); itS.Next())
                {
                    Shape e = itS.Value();
                    if (mFence.Add(e))
                    {
                        lArgs.Append(e);
                    }
                }
            }
            else if (type == ShapeType.Vertex || type == ShapeType.Edge)
            {
                lArgs.Append(s);
            }
        }
        mFence.Clear();
        //
        it.Initialize(lArgs);
        for (; it.More(); it.Next())
        {
            Shape s = it.Value();
            type = s.Type;
            if (type == ShapeType.Vertex || type == ShapeType.Edge || type == ShapeType.Wire)
            {
                if (mFence.Add(s))
                {
                    if (_images.ContainsKey(s))
                    {
                        NCollection_List<Shape> lSp = _images.GetValue(s);
                        it1.Initialize(lSp);
                        for (; it1.More(); it1.Next())
                        {
                            Shape sp = it1.Value();
                            msi.Add(sp);
                        }
                    }
                    else
                    {
                        msi.Add(s);
                    }
                }
            }
        }
        if (UserBreak(ps))
        {
            return;
        }

        //
        // 2. Internal vertices, edges from source solids
        mFence.Clear();
        lSd.Clear();
        //
        nbS = _ds.SourceShapeCount;
        for (i = 0; i < nbS; ++i)
        {
            ShapeInfo aSI = _ds.ShapeInfo(i);
            //
            if (aSI.ShapeType() != ShapeType.Solid)
            {
                continue;
            }
            if (UserBreak(ps))
            {
                return;
            }
            //
            Shape s = aSI.Shape();
            //
            mx.Clear();
            OwnInternalShapes(s, mx);
            //
            nbSx = mx.Count;
            for (j = 1; j <= nbSx; ++j)
            {
                Shape aSi = mx[j - 1];
                if (_images.ContainsKey(aSi))
                {
                    NCollection_List<Shape> lSp = _images.GetValue(aSi);
                    it1.Initialize(lSp);
                    for (; it1.More(); it1.Next())
                    {
                        Shape sp = it1.Value();
                        msi.Add(sp);
                    }
                }
                else
                {
                    msi.Add(aSi);
                }
            }
            //
            // build aux map from splits of solids
            if (_images.ContainsKey(s))
            {
                NCollection_List<Shape> lSp = _images.GetValue(s);
                it.Initialize(lSp);
                for (; it.More(); it.Next())
                {
                    Shape sp = it.Value();
                    if (mFence.Add(sp))
                    {
                        Exploration.MapShapesAndAncestors(sp, ShapeType.Vertex, ShapeType.Edge, mSx);
                        Exploration.MapShapesAndAncestors(sp, ShapeType.Vertex, ShapeType.Face, mSx);
                        Exploration.MapShapesAndAncestors(sp, ShapeType.Edge, ShapeType.Face, mSx);
                        lSd.Append(sp);
                    }
                }
            }
            else
            {
                if (mFence.Add(s))
                {
                    Exploration.MapShapesAndAncestors(s, ShapeType.Vertex, ShapeType.Edge, mSx);
                    Exploration.MapShapesAndAncestors(s, ShapeType.Vertex, ShapeType.Face, mSx);
                    Exploration.MapShapesAndAncestors(s, ShapeType.Edge, ShapeType.Face, mSx);
                    lSd.Append(s);
                    msOr.Add(s);
                }
            }
        } // for (i=0; i<aNbS; ++i) {
        //
        // 3. Some shapes of aMSI can be already tied with faces of
        //    split solids
        nbSI = msi.Count;
        for (i = 1; i <= nbSI; ++i)
        {
            Shape si = msi[i - 1];
            if (mSx.ContainsKey(si))
            {
                NCollection_List<Shape> lSx = mSx.GetValue(si);
                nbSx = lSx.Extent();
                if (nbSx == 0)
                {
                    lsi.Append(si);
                }
            }
            else
            {
                lsi.Append(si);
            }
        }
        //
        // 4. Just check it
        nbSI = lsi.Extent();
        if (nbSI == 0)
        {
            return;
        }

        ps.Next();
        //
        // 5 Settle internal vertices and edges into solids
        mx.Clear();

        using var psLoop = new ProgressScope(ps.Next(9), "Looking for internal shapes", lSd.Extent());

        it.Initialize(lSd);
        for (; it.More(); it.Next(), psLoop.Next())
        {
            Solid sd = ShapeCasts.Solid(it.Value());
            //
            it1.Initialize(lsi);
            for (; it1.More();)
            {
                Shape si = it1.Value();
                si.Orient(Internal);
                //
                state = AlgoTools.ComputeStateByOnePoint(si, sd, 1.0e-11, _context!);
                //
                if (state != In)
                {
                    it1.Next();
                    continue;
                }
                //
                if (msOr.Contains(sd))
                {
                    // make new solid
                    Solid sdx = new();
                    //
                    bb.MakeSolid(ref sdx);
                    itS.Initialize(sd);
                    for (; itS.More(); itS.Next())
                    {
                        Shape sh = itS.Value();
                        bb.Add(sdx, sh);
                    }
                    //
                    bb.Add(sdx, si);
                    //
                    // no need to check for images of aSd as aMSOr contains only original solids
                    NCollection_List<Shape> pLS = _images.SetValueRef(sd, new NCollection_List<Shape>());
                    pLS.Append(sdx);
                    //
                    NCollection_List<Shape> pLOr = _origins.SetValueRef(sdx, new NCollection_List<Shape>());
                    pLOr.Append(sd);
                    //
                    msOr.Remove(sd);
                    sd = sdx;
                }
                else
                {
                    // OCCT adds the shape into the split solid, which may be an argument's solid kept as the
                    // same-domain solid of a split (BuildSplitSolids' aSx; a frozen argument solid then throws, unit 6F)
                    bb.Add(sd, si);
                }
                //
                lsi.Remove(it1);
            } // for (; aIt1.More();) {
        } // for (; aIt.More(); aIt.Next()) {
        //
        //-----------------------------------------------------scope t
        lArgs.Clear();
        lSd.Clear();
        msOr.Clear();
        mFence.Clear();
        msi.Clear();
        mx.Clear();
        mSx.Clear();
    }

    // The own internal parts (all but the shells) of the shape.
    private static void OwnInternalShapes(in Shape s, IndexedSet<Shape> mx)
    {
        Iterator it = new();
        //
        it.Initialize(s);
        for (; it.More(); it.Next())
        {
            Shape sx = it.Value();
            if (sx.Type != ShapeType.Shell)
            {
                mx.Add(sx);
            }
        }
    }
}
