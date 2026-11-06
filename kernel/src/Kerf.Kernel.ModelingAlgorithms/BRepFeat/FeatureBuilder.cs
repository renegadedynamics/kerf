// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Builder.hxx
// Port of: src/ModelingAlgorithms/TKFeat/BRepFeat/BRepFeat_Builder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2012-2014 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Features;

/// <summary>
/// Provides a basic tool to implement features topological operations: a Boolean operation of the basis shape with a
/// tool, in two steps. First the tool's parts are computed (<see cref="PartsOfTool"/>); the caller keeps some of them
/// (<see cref="KeepParts"/>), and <see cref="PerformResult"/> builds the result from the kept parts only, removing the
/// others' faces from the result.
/// </summary>
/// <remarks>
/// A BOPAlgo_BOP: its arguments are read as the Boolean operations read theirs, never written (decision 274).
/// </remarks>
internal class FeatureBuilder : Bop
{
    protected HashSet<Shape> _shapes = new(ShapeHasher.Instance);
    protected HashSet<Shape> _removed = new(ShapeHasher.Instance);
    protected int _fuse;

    public FeatureBuilder()
    {
        Clear();
    }

    /// <summary>Clears internal fields and arguments.</summary>
    public override void Clear()
    {
        _shapes?.Clear();
        _removed?.Clear();
        base.Clear();
    }

    /// <summary>Initializes the object of local boolean operation.</summary>
    public void Init(in Shape shape)
    {
        Clear();
        //
        AddArgument(shape);
    }

    /// <summary>Initializes the arguments of local boolean operation.</summary>
    public void Init(in Shape shape, in Shape tool)
    {
        Clear();
        //
        AddArgument(shape);
        AddTool(tool);
    }

    /// <summary>Sets the operation of local boolean operation: a cut if <paramref name="fuse"/> is 0, else a fuse.</summary>
    public void SetOperation(int fuse)
    {
        _fuse = fuse;
        _operation = _fuse != 0 ? global::Kerf.Engine.Booleans.Operation.Fuse : global::Kerf.Engine.Booleans.Operation.Cut;
    }

    /// <summary>
    /// Sets the operation of local boolean operation. When <paramref name="flag"/> is true no selection of the tool's
    /// parts is needed (no second part): the operation is then COMMON if <paramref name="fuse"/> is 0, else CUT21.
    /// When false, as <see cref="SetOperation(int)"/>.
    /// </summary>
    public void SetOperation(int fuse, bool flag)
    {
        _fuse = fuse;
        if (!flag)
        {
            _operation = _fuse != 0 ? global::Kerf.Engine.Booleans.Operation.Fuse : global::Kerf.Engine.Booleans.Operation.Cut;
        }
        else
        {
            _operation = _fuse != 0 ? global::Kerf.Engine.Booleans.Operation.CUT21 : global::Kerf.Engine.Booleans.Operation.Common;
        }
    }

    /// <summary>Collects parts of the tool (the solids of the first step's result; not frozen).</summary>
    public void PartsOfTool(NCollection_List<Shape> aLT)
    {
        Explorer exp = new();
        //
        aLT.Clear();
        exp.Init(_shape, ShapeType.Solid);
        for (; exp.More(); exp.Next())
        {
            Shape s = exp.Current();
            aLT.Append(s);
        }
    }

    /// <summary>Initializes parts of the tool for the second step: collects the shapes and their sub-shapes.</summary>
    public void KeepParts(NCollection_List<Shape> im)
    {
        NCollection_List<Shape>.Iterator itT = new();
        itT.Initialize(im);
        for (; itT.More(); itT.Next())
        {
            Shape tIm = itT.Value();
            KeepPart(tIm);
        }
    }

    /// <summary>Adds the shape and all its sub-shapes into the kept shapes.</summary>
    public void KeepPart(in Shape part)
    {
        Exploration.MapShapes(part, _shapes);
    }

    /// <summary>Prepares builder of local operation.</summary>
    protected override void Prepare()
    {
        GetReport().Clear();
        //
        BrepBuilder bb = new();
        Compound c = new();
        bb.MakeCompound(ref c);
        _shape = c;
        //
        FillRemoved();
    }

    /// <summary>Collects the removed parts of the tool.</summary>
    public void FillRemoved()
    {
        Explorer exp = new();
        //
        Shape args0 = _arguments.First();
        Shape args1 = _tools.First();
        //
        exp.Init(args0, ShapeType.Solid);
        for (; exp.More(); exp.Next())
        {
            Shape s = exp.Current();
            _images.Remove(s);
        }
        //
        if (!_images.ContainsKey(args1))
        {
            return;
        }
        //
        NCollection_List<Shape>.Iterator itIm = new();
        //
        NCollection_List<Shape> ls = _images.GetValue(args1);
        itIm.Initialize(ls);
        for (; itIm.More(); itIm.Next())
        {
            Shape s = itIm.Value();
            FillRemoved(s, _removed);
        }
    }

    /// <summary>Main function to build the result of the local operation required.</summary>
    public void PerformResult(ProgressRange range = default)
    {
        _operation = _fuse != 0 ? global::Kerf.Engine.Booleans.Operation.Fuse : global::Kerf.Engine.Booleans.Operation.Cut;
        if (_shapes.Count == 0)
        {
            BuildShape(range);
            return;
        }

        double whole = 100.0;
        using var ps = new ProgressScope(range, "BRepFeat_Builder", whole);
        double bsPart = 15;
        whole -= bsPart;

        // Compute PI steps
        const int size = 4;
        NCollection_Array1<double> steps = new(0, size - 1);
        {
            for (int i = 0; i < size; ++i)
            {
                steps[i] = 0.0;
            }

            NbShapes nbShapes = GetNbShapes();
            double treatFaces = 5 * nbShapes.NbFaces();
            double treatShells = nbShapes.NbShells();
            double treatSolids = 20 * nbShapes.NbSolids();
            double treatCompounds = nbShapes.NbCompounds();

            double sum = treatFaces + treatShells + treatSolids + treatCompounds;
            if (sum > 0)
            {
                steps[0] = treatFaces * whole / sum;
                steps[1] = treatShells * whole / sum;
                steps[2] = treatSolids * whole / sum;
                steps[3] = treatCompounds * whole / sum;
            }
        }
        //
        Prepare();
        //
        RebuildFaces();
        ps.Next(steps[0]);
        //
        FillImagesContainers(ShapeType.Shell, ps.Next(steps[1]));
        if (HasErrors)
        {
            return;
        }
        //
        FillImagesSolids(ps.Next(steps[2]));
        if (HasErrors)
        {
            return;
        }
        //
        CheckSolidImages();
        //
        BuildResult(ShapeType.Solid);
        if (HasErrors)
        {
            return;
        }
        //
        FillImagesCompounds(ps.Next(steps[3]));
        if (HasErrors)
        {
            return;
        }
        //
        BuildResult(ShapeType.Compound);
        if (HasErrors)
        {
            return;
        }
        //
        BuildShape(ps.Next(bsPart));
    }

    /// <summary>Rebuilds faces in accordance with the kept parts of the tool.</summary>
    public void RebuildFaces()
    {
        int nbS, i, iRank, nSp, j;
        bool bIsClosed, bIsDegenerated, bToReverse, bRem, bIm, bFlagSD, bVInShapes;
        Orientation oriF, oriE;
        Face ff;
        Edge sp;
        Shape sx;
        Explorer exp = new(), expE = new();
        HashSet<Shape> me = new(ShapeHasher.Instance), meSplit = new(ShapeHasher.Instance);
        NCollection_List<Shape>.Iterator itIm = new();
        // OCCT's NCollection_Map<BOPTools_Set>, of which Added() returns the key in the map: an indexed map gives the
        // stored key back from its index, and the map is never iterated (as BOPAlgo_Builder_3)
        IndexedSet<ToolSet> mst = new();
        NCollection_List<Shape> le = new();
        //
        foreach (Shape s in _shapes)
        {
            if (s.Type == ShapeType.Face)
            {
                ToolSet st = new();
                st.Add(s, ShapeType.Edge);
                mst.Add(st);
            }
        }
        //
        nbS = _ds!.SourceShapeCount;
        for (i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            //
            iRank = _ds.Rank(i);
            if (iRank == 1)
            {
                Shape s = si.Shape();
                //
                if (_images.ContainsKey(s))
                {
                    NCollection_List<Shape> lIm = _images.GetValue(s);
                    itIm.Initialize(lIm);
                    for (; itIm.More();)
                    {
                        Shape sIm = itIm.Value();
                        if (!_shapes.Contains(sIm))
                        {
                            lIm.Remove(itIm);
                            continue;
                        }
                        itIm.Next();
                    }
                }
                continue;
            }
            //
            if (si.ShapeType() != ShapeType.Face)
            {
                continue;
            }
            //
            FaceInfo fi = _ds.FaceInfo(i);
            Face f = ShapeCasts.Face(si.Shape());
            //
            if (!_images.ContainsKey(f))
            {
                continue;
            }
            //
            oriF = f.Orientation;
            ff = f;
            ff.Orient(Forward);

            IndexedSet<PaveBlock> mpbIn = fi.PaveBlocksIn();
            IndexedSet<PaveBlock> mpbSc = fi.PaveBlocksSc();

            le.Clear();

            // bounding edges
            exp.Init(ff, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                oriE = e.Orientation;
                bIsDegenerated = Tool.Degenerated(e);
                bIsClosed = Tool.IsClosed(e, f);
                if (_images.ContainsKey(e))
                {
                    NCollection_List<Shape> leIm = _images.GetValue(e);
                    //
                    bRem = false;
                    bIm = false;
                    me.Clear();
                    NCollection_List<Shape> leImNew = new();
                    //
                    itIm.Initialize(leIm);
                    for (; itIm.More(); itIm.Next())
                    {
                        Shape s = itIm.Value();

                        bVInShapes = false;
                        if (_shapes.Contains(s))
                        {
                            bVInShapes = true;
                        }
                        else
                        {
                            expE.Init(s, ShapeType.Vertex);
                            for (; expE.More(); expE.Next())
                            {
                                Shape v = expE.Current();
                                if (_shapes.Contains(v))
                                {
                                    bVInShapes = true;
                                    break;
                                }
                            }
                        }
                        //
                        if (bVInShapes)
                        {
                            bIm = true;
                            leImNew.Append(s);
                        }
                        else
                        {
                            bRem = true;
                            me.Add(s);
                        }
                    }
                    //
                    if (!bIm)
                    {
                        le.Append(e);
                        continue;
                    }
                    //
                    if (bRem && bIm)
                    {
                        if (leIm.Extent() == 2)
                        {
                            le.Append(e);
                            continue;
                        }
                        if (meSplit.Add(e))
                        {
                            RebuildEdge(e, ff, me, leImNew);
                            leIm.Assign(leImNew);
                            if (leIm.Extent() == 1)
                            {
                                le.Append(e);
                                continue;
                            }
                        }
                    }
                    //
                    itIm.Initialize(leIm);
                    for (; itIm.More(); itIm.Next())
                    {
                        sp = ShapeCasts.Edge(itIm.Value());

                        if (bIsDegenerated)
                        {
                            sp.Orient(oriE);
                            le.Append(sp);
                            continue;
                        }
                        //
                        if (oriE == Internal)
                        {
                            sp.Orient(Forward);
                            le.Append(sp);
                            sp.Orient(Reversed);
                            le.Append(sp);
                            continue;
                        }
                        //
                        if (bIsClosed)
                        {
                            if (!Tool.IsClosed(sp, ff))
                            {
                                AlgoTools3D.DoSplitSEAMOnFace(sp, ff);
                            }
                            //
                            sp.Orient(Forward);
                            le.Append(sp);
                            sp.Orient(Reversed);
                            le.Append(sp);
                            continue;
                        } // if (bIsClosed){
                        //
                        sp.Orient(oriE);
                        bToReverse = AlgoTools.IsSplitToReverse(sp, e, _context);
                        if (bToReverse)
                        {
                            sp.Reverse();
                        }
                        le.Append(sp);
                    }
                }
                else
                {
                    le.Append(e);
                }
            }

            int nbPBIn, nbPBSc;
            nbPBIn = mpbIn.Count;
            nbPBSc = mpbSc.Count;
            //
            // in edges
            for (j = 1; j <= nbPBIn; ++j)
            {
                PaveBlock pb = mpbIn[j - 1];
                nSp = pb.Edge();
                sp = ShapeCasts.Edge(_ds.Shape(nSp));
                if (_removed.Contains(sp))
                {
                    continue;
                }
                //
                sp.Orient(Forward);
                le.Append(sp);
                sp.Orient(Reversed);
                le.Append(sp);
            }
            // section edges
            for (j = 1; j <= nbPBSc; ++j)
            {
                PaveBlock pb = mpbSc[j - 1];
                nSp = pb.Edge();
                sp = ShapeCasts.Edge(_ds.Shape(nSp));
                if (_removed.Contains(sp))
                {
                    continue;
                }
                //
                sp.Orient(Forward);
                le.Append(sp);
                sp.Orient(Reversed);
                le.Append(sp);
            }

            // build new faces
            BuilderFace bf = new();
            bf.SetFace(ff);
            bf.SetShapes(le);

            bf.Perform();

            NCollection_List<Shape> lfIm = _images.GetValue(f);
            lfIm.Clear();

            NCollection_List<Shape> lfr = bf.Areas();
            itIm.Initialize(lfr);
            for (; itIm.More(); itIm.Next())
            {
                Shape fr = itIm.Value();
                //
                ToolSet st = new();
                st.Add(fr, ShapeType.Edge);
                bFlagSD = mst.Contains(st);
                //
                ToolSet sTx = mst[mst.Add(st)];
                sx = sTx.Shape();
                sx.Orient(oriF);
                lfIm.Append(sx);
                //
                if (!_origins.TryGetValue(sx, out NCollection_List<Shape>? pLOr))
                {
                    pLOr = _origins.SetValueRef(sx, new NCollection_List<Shape>());
                }
                pLOr.Append(f);
                //
                if (bFlagSD)
                {
                    _shapesSD[fr] = sx;
                }
            }
            //
            if (lfIm.Extent() == 0)
            {
                _images.Remove(f);
            }
        }
    }

    /// <summary>Rebuilds edges in accordance with the kept parts of the tool.</summary>
    public void RebuildEdge(in Shape e, in Face f, HashSet<Shape> aME,
                            NCollection_List<Shape> aLIm)
    {
        int nE, nSp, nV1, nV2, nE1, nV, nVx, nVSD = 0;
        int nV11, nV21;
        bool bOld;
        double t11, t21;
        double t1, t2;
        Edge aE;
        ShapeInfo si = new();
        Vertex v1, v2;
        PaveBlock pbNew;
        HashSet<int> mi = new(), mAdd = new(), mv = new(), mvOr = new();
        NCollection_List<PaveBlock>.Iterator itPB = new();
        NCollection_List<int>.Iterator itLI = new();
        // OCCT iterates this address-keyed map to find the one block with the same bounds: the order is not observable
        HashSet<PaveBlock> mpb = new(ReferenceEqualityComparer.Instance);
        //
        si.SetShapeType(ShapeType.Edge);

        // 1. collect origin vertices to aMV map.
        nE = _ds!.Index(e);
        ShapeInfo sie = _ds.ShapeInfo(nE);
        NCollection_List<int> ls = sie.SubShapes();
        itLI.Initialize(ls);
        for (; itLI.More(); itLI.Next())
        {
            nV = itLI.Value();
            nVx = nV;
            if (_ds.HasShapeSD(nV, ref nVSD))
            {
                nVx = nVSD;
            }
            mv.Add(nVx);
            mvOr.Add(nVx);
        }
        //
        // 2. collect vertices that should be removed to aMI map.
        pbNew = new PaveBlock();
        NCollection_List<Pave> lpExt = pbNew.ChangeExtPaves();
        NCollection_List<PaveBlock> lpb = _ds.ChangePaveBlocks(nE);
        //
        for (itPB.Initialize(lpb); itPB.More(); itPB.Next())
        {
            PaveBlock pb = itPB.Value();
            nE1 = pb.Edge();
            Shape e1 = _ds.Shape(nE1);
            //
            if (aME.Contains(e1))
            {
                pb.Indices(out nV1, out nV2);
                mi.Add(nV1);
                mi.Add(nV2);
            }
            else
            {
                mpb.Add(pb);
            }
        }
        // 3. collect vertices that split the source shape.
        for (itPB.Initialize(lpb); itPB.More(); itPB.Next())
        {
            PaveBlock pb = itPB.Value();
            pb.Indices(out nV1, out nV2);
            //
            if (!mi.Contains(nV1))
            {
                mv.Add(nV1);
            }
            if (!mi.Contains(nV2))
            {
                mv.Add(nV2);
            }
        }
        // 4. collect ext paves.
        for (itPB.Initialize(lpb); itPB.More(); itPB.Next())
        {
            PaveBlock pb = itPB.Value();
            pb.Indices(out nV1, out nV2);
            //
            if (mv.Contains(nV1))
            {
                if (mAdd.Add(nV1) || mvOr.Contains(nV1))
                {
                    lpExt.Append(pb.Pave1());
                }
            }
            //
            if (mv.Contains(nV2))
            {
                if (mAdd.Add(nV2) || mvOr.Contains(nV2))
                {
                    lpExt.Append(pb.Pave2());
                }
            }
        }

        aE = ShapeCasts.Edge(e);
        aE.Orient(Forward);
        //
        aLIm.Clear();
        //
        // 5. split edge by new set of vertices.
        lpb.Clear();
        pbNew.SetOriginalEdge(nE);
        pbNew.Update(lpb, false);
        //
        for (itPB.Initialize(lpb); itPB.More(); itPB.Next())
        {
            PaveBlock pb = itPB.Value();
            Pave pave1 = pb.Pave1();
            pave1.Contents(out nV1, out t1);
            //
            Pave pave2 = pb.Pave2();
            pave2.Contents(out nV2, out t2);
            //
            // check if it is the old pave block.
            bOld = false;
            foreach (PaveBlock pb1 in mpb)
            {
                pb1.Indices(out nV11, out nV21);
                pb1.Range(out t11, out t21);
                if (nV1 == nV11 && nV2 == nV21 && t1 == t11 && t2 == t21)
                {
                    Shape eIm = _ds.Shape(pb1.Edge());
                    aLIm.Append(eIm);
                    bOld = true;
                    break;
                }
            }
            if (bOld)
            {
                continue;
            }
            //
            v1 = ShapeCasts.Vertex(_ds.Shape(nV1));
            v1.Orient(Forward);
            //
            v2 = ShapeCasts.Vertex(_ds.Shape(nV2));
            v2.Orient(Reversed);
            //
            AlgoTools.MakeSplitEdge(aE, v1, t1, v2, t2, out Edge sp);
            AlgoTools2D.BuildPCurveForEdgeOnFace(sp, f, _context);
            //
            si.SetShape(sp);
            //
            ref Box box = ref si.ChangeBox();
            Bounds.Add(sp, ref box);
            //
            nSp = _ds.Append(si);
            //
            pb.SetEdge(nSp);
            aLIm.Append(sp);
        }
    }

    /// <summary>Collects the images of the object, that contains in the images of the tool.</summary>
    public void CheckSolidImages()
    {
        IndexedSet<ToolSet> mst = new(); // OCCT's Map with Added(): see RebuildFaces
        NCollection_List<Shape> lsImNew = new();
        NCollection_List<Shape>.Iterator it = new();
        Explorer exp = new();
        bool bFlagSD;
        //
        Shape args0 = _arguments.First();
        Shape args1 = _tools.First();
        //
        NCollection_List<Shape> lsIm = _images.GetValue(args1);
        it.Initialize(lsIm);
        for (; it.More(); it.Next())
        {
            Shape solIm = it.Value();
            //
            ToolSet st = new();
            st.Add(solIm, ShapeType.Face);
            mst.Add(st);
        }
        //
        exp.Init(args0, ShapeType.Solid);
        for (; exp.More(); exp.Next())
        {
            Shape solid = exp.Current();
            if (_images.ContainsKey(solid))
            {
                NCollection_List<Shape> lsImSol = _images.GetValue(solid);
                it.Initialize(lsImSol);
                for (; it.More(); it.Next())
                {
                    Shape solIm = it.Value();
                    //
                    ToolSet st = new();
                    st.Add(solIm, ShapeType.Face);
                    bFlagSD = mst.Contains(st);
                    //
                    ToolSet sTx = mst[mst.Add(st)];
                    Shape sx = sTx.Shape();
                    lsImNew.Append(sx);
                    //
                    if (bFlagSD)
                    {
                        _shapesSD[solIm] = sx;
                    }
                }
                lsImSol.Assign(lsImNew);
            }
        }
    }

    /// <summary>Adds the shape <paramref name="s"/> and its sub-shapes not kept into the map <paramref name="m"/>.</summary>
    public void FillRemoved(in Shape s, HashSet<Shape> m)
    {
        if (_shapes.Contains(s))
        {
            return;
        }
        //
        m.Add(s);
        Iterator it = new(s);
        while (it.More())
        {
            FillRemoved(it.Value(), m);
            it.Next();
        }
    }

    /// <summary>Redefined to avoid the usage of removed faces.</summary>
    protected override void FillIn3DParts(Dictionary<Shape, Shape> draftSolids,
                                          ProgressRange range)
    {
        GetReport().Clear();

        base.FillIn3DParts(draftSolids, range);

        // Clear the IN parts of the solids from the removed faces
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> itM in _inParts)
        {
            NCollection_List<Shape> list = itM.Value;
            NCollection_List<Shape>.Iterator itL = new(list);
            for (; itL.More();)
            {
                if (_removed.Contains(itL.Value()))
                {
                    list.Remove(itL);
                }
                else
                {
                    itL.Next();
                }
            }
        }
    }

    /// <summary>Avoid the check for open solids and always use the splits of solids for building the result shape.</summary>
    protected override bool CheckArgsForOpenSolid() => false;
}
