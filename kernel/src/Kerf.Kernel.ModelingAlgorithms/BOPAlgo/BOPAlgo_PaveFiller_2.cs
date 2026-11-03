// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>The vertex/edge intersection solver of the pave filler's parallel loop.</summary>
internal sealed class VertexEdge : ParallelAlgo, BooleanParallel.IContextSolver<IntersectionContext>
{
    private int _iv;
    private int _ie;
    private int _flag;
    private double _t;
    private double _tolVNew;
    private Vertex _v = new();
    private Edge _e = new();
    private IntersectionContext? _context;
    private PaveBlock? _pb;

    public VertexEdge()
    {
        _iv = -1;
        _ie = -1;
        _flag = -1;
        _t = -1.0;
        _tolVNew = -1.0;
    }

    //
    public void SetIndices(int nV, int nE)
    {
        _iv = nV;
        _ie = nE;
    }

    //
    public void Indices(out int nV, out int nE)
    {
        nV = _iv;
        nE = _ie;
    }

    //
    public void SetVertex(in Vertex aV) => _v = aV;

    //
    public void SetEdge(in Edge aE) => _e = aE;

    //
    public Vertex Vertex() => _v;

    //
    public Edge Edge() => _e;

    //
    public int Flag() => _flag;

    //
    public double Parameter() => _t;

    //
    public double VertexNewTolerance() => _tolVNew;

    //
    public void SetContext(IntersectionContext aContext) => _context = aContext;

    //
    public IntersectionContext? Context() => _context;

    //
    public void SetPaveBlock(PaveBlock pb) => _pb = pb;

    //
    public PaveBlock? PaveBlock() => _pb;

    //
    public override void Perform()
    {
        using var ps = new ProgressScope(_progressRange, null, 1);
        if (UserBreak(ps))
        {
            return;
        }
        try
        {
            _flag = _context!.ComputeVE(_v, _e, ref _t, ref _tolVNew, _fuzzyValue);
        }
        catch (FailureException)
        {
            AddError(new AlertIntersectionFailed());
        }
    }
}

public partial class PaveFiller
{
    /// <summary>The vertex/edge interferences.</summary>
    protected virtual void PerformVE(ProgressRange range)
    {
        FillShrunkData(ShapeType.Vertex, ShapeType.Edge);
        //
        _iterator!.Initialize(ShapeType.Vertex, ShapeType.Edge);
        using var ps = new ProgressScope(range, null, 1);

        int iSize = _iterator.ExpectedLength();
        if (iSize == 0)
        {
            return;
        }
        //
        // Prepare pairs for intersection
        NCollection_IndexedDataMap<PaveBlock, NCollection_List<int>> mvePairs = new();
        for (; _iterator.More(); _iterator.Next())
        {
            if (UserBreak(ps))
            {
                return;
            }
            _iterator.Value(out int nV, out int nE);
            //
            ShapeInfo sie = _ds!.ShapeInfo(nE);
            if (sie.HasSubShape(nV))
            {
                continue;
            }
            //
            if (sie.HasFlag())
            {
                continue;
            }
            //
            if (_ds.HasInterf(nV, nE))
            {
                continue;
            }
            //
            if (_ds.HasInterfShapeSubShapes(nV, nE))
            {
                continue;
            }
            //
            NCollection_List<PaveBlock> lpb = _ds.PaveBlocks(nE);
            if (lpb.IsEmpty())
            {
                continue;
            }
            //
            PaveBlock pb = lpb.First();
            if (!pb.IsSplittable)
            {
                // this is a micro edge, ignore it
                continue;
            }
            //
            int index = mvePairs.FindIndex(pb);
            NCollection_List<int> pLV = index > 0 ? mvePairs[index] : mvePairs[mvePairs.Add(pb, new NCollection_List<int>())];
            pLV.Append(nV);
        }
        //
        IntersectVE(mvePairs, ps.Next());
    }

    /// <summary>
    /// Intersects the vertices with the pave blocks of <paramref name="vePairs"/>, adds the extra paves and splits
    /// the pave blocks; with <paramref name="addInterfs"/> the vertex/edge interferences are added to the DS.
    /// </summary>
    protected void IntersectVE(NCollection_IndexedDataMap<PaveBlock, NCollection_List<int>> vePairs,
                               ProgressRange range, bool addInterfs = true)
    {
        int i, nbVE = vePairs.Extent();
        if (nbVE == 0)
        {
            return;
        }
        //
        NCollection_DynamicArray<InterfVE> vEs = _ds!.InterfVE();
        //
        // Prepare for intersection.
        NCollection_DynamicArray<VertexEdge> vve = new();
        // Map to collect all SD connections to add interferences
        // for all vertices having the same SD vertex.
        // It will also be used as a Fence map to avoid repeated
        // intersection of the same SD vertex with edge
        Dictionary<Pair, NCollection_List<int>> dmvsd = new();
        //
        using var psOuter = new ProgressScope(range, null, 10);
        for (i = 1; i <= nbVE; ++i)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            PaveBlock pb = vePairs.FindKey(i);
            int nE = pb.OriginalEdge();
            //
            NCollection_IntMap mvpb = new();
            NCollection_List<PaveBlock> lpb = _ds.PaveBlocks(nE);
            for (NCollection_List<PaveBlock>.Iterator itPB = new(lpb); itPB.More(); itPB.Next())
            {
                mvpb.Add(itPB.Value().Pave1().Index());
                mvpb.Add(itPB.Value().Pave2().Index());
            }

            NCollection_List<int> lv = vePairs[i];
            NCollection_List<int>.Iterator itLV = new(lv);
            for (; itLV.More(); itLV.Next())
            {
                int nV = itLV.Value();
                //
                int nVSD = nV;
                _ds.HasShapeSD(nV, ref nVSD);
                //
                if (mvpb.Contains(nVSD))
                {
                    continue;
                }

                Pair pair = new(nVSD, nE);
                if (dmvsd.TryGetValue(pair, out NCollection_List<int>? pLI0))
                {
                    // Already added
                    pLI0.Append(nV);
                    continue;
                }
                // New pair
                NCollection_List<int> pLI = dmvsd.SetValueRef(pair, new NCollection_List<int>());
                pLI.Append(nV);
                //
                Vertex v = ShapeCasts.Vertex(_ds.Shape(nVSD));
                Edge e = ShapeCasts.Edge(_ds.Shape(nE));
                //
                VertexEdge veSolver = vve.Append(new VertexEdge());
                veSolver.SetIndices(nVSD, nE);
                veSolver.SetVertex(v);
                veSolver.SetEdge(e);
                veSolver.SetPaveBlock(pb);
                veSolver.SetFuzzyValue(_fuzzyValue);
            }
        }
        //
        nbVE = vve.Length();

        using (var ps = new ProgressScope(psOuter.Next(9), "Performing Vertex-Edge intersection", nbVE))
        {
            for (i = 0; i < nbVE; i++)
            {
                VertexEdge veSolver = vve.ChangeValue(i);
                veSolver.SetProgressRange(ps.Next());
            }
            // Perform intersection
            //=============================================================
            BooleanParallel.Perform(_runParallel, vve, _context!);
            //=============================================================
        }
        if (UserBreak(psOuter))
        {
            return;
        }
        //
        // Keep the modified edges for further update
        NCollection_IntMap mEdges = new();
        //
        // Analyze intersections
        for (i = 0; i < nbVE; ++i)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            VertexEdge veSolver = vve[i];
            if (veSolver.Flag() != 0)
            {
                if (veSolver.HasErrors)
                {
                    // Warn about failed intersection of sub-shapes
                    AddIntersectionFailedWarning(veSolver.Vertex(), veSolver.Edge());
                }
                continue;
            }
            //
            veSolver.Indices(out int nV, out int nE);
            // Parameter of vertex on edge
            double t = veSolver.Parameter();
            // 1. Update vertex V/E if necessary
            double tolVNew = veSolver.VertexNewTolerance();
            int nVx = UpdateVertex(nV, tolVNew);
            // 2. Create new pave and add it as extra pave to pave block
            //    for further splitting of the edge
            NCollection_List<PaveBlock> lpb = _ds.PaveBlocks(nE);
            // Find the appropriate one
            PaveBlock? pb = null;
            NCollection_List<PaveBlock>.Iterator itPB = new(lpb);
            for (; itPB.More(); itPB.Next())
            {
                pb = itPB.Value();
                pb.Range(out double t1, out double t2);
                if (t > t1 && t < t2)
                {
                    break;
                }
            }
            if (!itPB.More())
            {
                continue;
            }

            Pave pave = new();
            pave.SetIndex(nVx);
            pave.SetParameter(t);
            pb!.AppendExtPave(pave);
            mEdges.Add(nE);
            //
            if (addInterfs)
            {
                // Add interferences into DS
                Pair pair = new(nV, nE);
                NCollection_List<int> li = dmvsd.GetValue(pair);
                NCollection_List<int>.Iterator itLI = new(li);
                for (; itLI.More(); itLI.Next())
                {
                    int nVOld = itLI.Value();
                    // 3. Create interference V/E
                    InterfVE ve = vEs.Append(new InterfVE());
                    ve.SetIndices(nVOld, nE);
                    ve.SetParameter(t);
                    // 2. Add a pair in the whole table of interferences
                    _ds.AddInterf(nVOld, nE);
                    // 4. Set index of new vertex in the interference
                    if (_ds.IsNewShape(nVx))
                    {
                        ve.SetIndexNew(nVx);
                    }
                }
            }
        }
        //
        // Split pave blocks of the intersected edges with the extra paves.
        // At the same time compute shrunk data for the new pave blocks
        // and in case there is no valid range for the pave block,
        // the vertices of this pave block should be unified.
        SplitPaveBlocks(mEdges, addInterfs);
    }

    // Make new Common Block from the given list of Pave Blocks
    private static void MakeNewCommonBlock(NCollection_List<PaveBlock> lpb, NCollection_List<int> lFaces, DS ds)
    {
        // Make Common Block from the pave blocks in the list
        CommonBlock cbNew = new();
        cbNew.SetPaveBlocks(lpb);
        cbNew.SetFaces(lFaces);
        //
        NCollection_List<PaveBlock>.Iterator itLPB = new(lpb);
        for (; itLPB.More(); itLPB.Next())
        {
            ds.SetCommonBlock(itLPB.Value(), cbNew);
        }
    }

    /// <summary>
    /// Splits the pave blocks of the edges by their extra paves; the vertices of a new pave block without a valid range
    /// are unified.
    /// </summary>
    protected void SplitPaveBlocks(NCollection_IntMap mEdges, bool addInterfs)
    {
        // Fence map to avoid unification of the same vertices twice
        HashSet<Pair> mPairs = new();
        // Map to treat the Common Blocks
        NCollection_IndexedDataMap<CommonBlock, NCollection_List<PaveBlock>> mcbNewPB = new();
        //
        // Map of vertices to init the pave blocks for them
        NCollection_IntMap mVerticesToInitPB = new();

        // OCCT iterates the edges in the int map's bucket order (decision 784): it decides the order of the new
        // same-domain vertices.
        foreach (int nE in mEdges)
        {
            NCollection_List<PaveBlock> lpb = _ds!.ChangePaveBlocks(nE);
            //
            NCollection_List<PaveBlock>.Iterator itLPB = new(lpb);
            for (; itLPB.More();)
            {
                PaveBlock pb = itLPB.Value();
                //
                if (!pb.IsToUpdate)
                {
                    itLPB.Next();
                    continue;
                }
                //
                CommonBlock? cb = _ds.CommonBlock(pb);
                //
                // Compute new pave blocks
                NCollection_List<PaveBlock> lpbn = new();
                pb.Update(lpbn);
                //
                // Make sure that each new pave block has a valid range,
                // otherwise unify the vertices of the pave block
                NCollection_List<PaveBlock>.Iterator itLPBN = new(lpbn);
                for (; itLPBN.More(); itLPBN.Next())
                {
                    PaveBlock pbn = itLPBN.Value();
                    _ds.UpdatePaveBlockWithSDVertices(pbn);
                    FillShrunkData(pbn);
                    //
                    bool bHasValidRange = pbn.HasShrunkData;
                    // Take into account that the edge could have really small valid range,
                    // so that the Pave Block cannot be further split. In this case, check if
                    // the vertices of the Pave Block do not interfere. And if they are, unify them.
                    bool bCheckDist = (bHasValidRange && !pbn.IsSplittable);
                    if (!bHasValidRange || bCheckDist)
                    {
                        pbn.Indices(out int nV1, out int nV2);
                        if (nV1 == nV2)
                        {
                            // Same vertices -> no valid range, no need to unify vertices
                            continue;
                        }

                        // Decide whether to unify vertices or not
                        if (bCheckDist)
                        {
                            Vertex v1 = ShapeCasts.Vertex(_ds.Shape(nV1));
                            Vertex v2 = ShapeCasts.Vertex(_ds.Shape(nV2));
                            if (AlgoTools.ComputeVV(v1, v2, _fuzzyValue) == 0)
                            {
                                // vertices are interfering -> no valid range, unify vertices
                                bHasValidRange = false;
                            }
                        }

                        if (!bHasValidRange)
                        {
                            Pair pair = new();
                            pair.SetIndices(nV1, nV2);
                            if (mPairs.Add(pair))
                            {
                                NCollection_List<int> lv = new();
                                lv.Append(nV1);
                                lv.Append(nV2);
                                MakeSDVertices(lv, addInterfs);

                                // Save vertices to init pave blocks
                                mVerticesToInitPB.Add(nV1);
                                mVerticesToInitPB.Add(nV2);
                            }
                            continue;
                        }
                    }
                    //
                    // Update the list with new pave block
                    lpb.Append(pbn);
                    // Treat the common block
                    if (cb is not null)
                    {
                        // Store the new pave block to make new common block
                        int index = mcbNewPB.FindIndex(cb);
                        NCollection_List<PaveBlock> pLPBCB = index > 0
                            ? mcbNewPB[index]
                            : mcbNewPB[mcbNewPB.Add(cb, new NCollection_List<PaveBlock>())];
                        pLPBCB.Append(pbn);
                    }
                }
                // Remove old pave block
                lpb.Remove(itLPB);
            }
        }
        //
        // Make Common Blocks
        int i, nbCB = mcbNewPB.Extent();
        for (i = 1; i <= nbCB; ++i)
        {
            CommonBlock cb = mcbNewPB.FindKey(i);
            NCollection_List<PaveBlock> lpbn = mcbNewPB[i];
            //
            // For each group of pave blocks with the same vertices make new common block
            NCollection_IndexedDataMap<Pair, NCollection_List<PaveBlock>> mInds = new();
            NCollection_List<PaveBlock>.Iterator itLPB = new(lpbn);
            for (; itLPB.More(); itLPB.Next())
            {
                PaveBlock pb = itLPB.Value();
                //
                Pair pair = new();
                pair.SetIndices(pb.Pave1().Index(), pb.Pave2().Index());
                //
                int index = mInds.FindIndex(pair);
                NCollection_List<PaveBlock> pLPBx = index > 0
                    ? mInds[index]
                    : mInds[mInds.Add(pair, new NCollection_List<PaveBlock>())];
                pLPBx.Append(pb);
            }
            //
            cb.PaveBlock1().Indices(out int nV1, out int nV2);
            bool bIsClosed = (nV1 == nV2);
            //
            int j, nbPairs = mInds.Extent();
            for (j = 1; j <= nbPairs; ++j)
            {
                NCollection_List<PaveBlock> lpb = mInds[j];
                //
                if (!bIsClosed)
                {
                    // Make Common Block from the pave blocks in the list
                    MakeNewCommonBlock(lpb, cb.Faces(), _ds!);
                    continue;
                }
                //
                // Find coinciding pave blocks
                while (lpb.Extent() != 0)
                {
                    // Pave blocks forming the common block
                    NCollection_List<PaveBlock> lpbcb = new();
                    // Point in the middle of the first pave block in the common block
                    Point3D pmFirst = new(0.0, 0.0, 0.0);
                    // Tolerance of the first edge in the common block
                    double tolEFirst = 0.0;
                    //
                    itLPB.Initialize(lpb);
                    for (; itLPB.More();)
                    {
                        PaveBlock pb = itLPB.Value();
                        if (lpbcb.IsEmpty())
                        {
                            lpbcb.Append(pb);
                            Edge eFirst = ShapeCasts.Edge(_ds!.Shape(pb.OriginalEdge()));
                            tolEFirst = Tool.MaxTolerance(eFirst, ShapeType.Vertex);
                            //
                            double tmFirst = (pb.Pave1().Parameter() + pb.Pave2().Parameter()) / 2.0;
                            AlgoTools.PointOnEdge(eFirst, tmFirst, out pmFirst);
                            //
                            lpb.Remove(itLPB);
                            continue;
                        }
                        //
                        // Check pave blocks for coincidence
                        Edge e = ShapeCasts.Edge(_ds!.Shape(pb.OriginalEdge()));
                        double tolE = Tool.MaxTolerance(e, ShapeType.Vertex);
                        //
                        double tOut = 0.0, dist = 0.0; // OCCT: uninitialised, read only on success
                        int iErr = _context!.ComputePE(pmFirst, tolEFirst + tolE + _fuzzyValue, e, ref tOut, ref dist);
                        if (iErr == 0 && ((tOut > pb.Pave1().Parameter()) && (tOut < pb.Pave2().Parameter())))
                        {
                            lpbcb.Append(pb);
                            lpb.Remove(itLPB);
                            continue;
                        }
                        itLPB.Next();
                    }
                    //
                    // Make Common Block from the pave blocks in the list
                    MakeNewCommonBlock(lpbcb, cb.Faces(), _ds!);
                }
            }
        }

        // Init pave blocks for vertices which have acquired SD vertex
        foreach (int nV in mVerticesToInitPB)
        {
            _ds!.InitPaveBlocksForVertex(nV);
        }
    }

    /// <summary>Adds the warning <see cref="AlertIntersectionOfPairOfShapesFailed"/> for the two shapes.</summary>
    protected void AddIntersectionFailedWarning(in Shape s1, in Shape s2)
    {
        // Create the warn shape
        Compound wc = new();
        new BrepBuilder().MakeCompound(ref wc);
        new BrepBuilder().Add(wc, s1);
        new BrepBuilder().Add(wc, s2);
        // Add the warning
        AddWarning(new AlertIntersectionOfPairOfShapesFailed(wc));
    }
}
