// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_7.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Booleans;

/// <summary>The split of an edge by a pave block (and the tolerance of its common block), as a parallel solver.</summary>
internal sealed class SplitEdgeAlgorithm : ParallelAlgo, BooleanParallel.IContextSolver<IntersectionContext>
{
    // ->
    private Edge _e = new();
    private Vertex _v1 = new();
    private double _t1;
    private Vertex _v2 = new();
    private double _t2;
    // <->
    private PaveBlock? _pb;
    private CommonBlock? _cb;
    // <-
    private Edge _eSp = new();
    private Box _box = new();
    private double _tol;
    //
    private DS? _ds;
    private IntersectionContext? _context;

    public SplitEdgeAlgorithm()
    {
        _t1 = 0.0;
        _t2 = 0.0;
        _tol = 0.0;
    }

    public void SetData(in Edge aE, in Vertex aV1, double aT1, in Vertex aV2, double aT2)
    {
        _e = aE;
        _v1 = aV1;
        _t1 = aT1;
        _v2 = aV2;
        _t2 = aT2;
        _eSp = aE;
    }

    public void SetPaveBlock(PaveBlock aPB) => _pb = aPB;

    public PaveBlock PaveBlock() => _pb!;

    public void SetCommonBlock(CommonBlock? aCB) => _cb = aCB;

    public CommonBlock? CommonBlock() => _cb;

    public Edge SplitEdge() => _eSp;

    public Box Box() => _box;

    public double Tolerance() => _tol;

    public void SetDS(DS ds) => _ds = ds;

    public void SetContext(IntersectionContext aContext) => _context = aContext;

    public override void Perform()
    {
        using var ps = new ProgressScope(_progressRange, null, 1);
        if (UserBreak(ps))
        {
            return;
        }
        _tol = BuilderTools.ComputeToleranceOfCB(_cb, _ds!, _context);
        AlgoTools.MakeSplitEdge(_e, _v1, _t1, _v2, _t2, out _eSp);
        Bounds.Add(_eSp, ref _box);
        _box.SetGap(_box.GetGap() + global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}

/// <summary>The p-curve of an edge on a face, as a parallel solver.</summary>
/// <remarks>
/// The solver builds a new p-curve only on a copy of the edge (<c>CopyEdge</c>) or adjusts an existing one; the pave
/// filler stores the new p-curve afterwards, serially. The vertex tolerances of section edges (<c>myFlag</c>) are
/// updated in place by <c>UpdateVertices</c>, in parallel, as in OCCT, but each update under a lock (decision 789).
/// </remarks>
internal sealed class MPC : ParallelAlgo, BooleanParallel.IContextSolver<IntersectionContext>
{
    private bool _flag;
    private Edge _e = new();
    private Face _f = new();
    private Edge _ez = new();
    private Vertex _v1 = new();
    private double _t1;
    private Vertex _v2 = new();
    private double _t2;
    private Curve2D? _newC2d;
    private double _newTol;
    //
    private IntersectionContext? _context;

    private readonly Lock _verticesLock;

    // verticesLock: the filler's, guarding the tolerances of the vertices its solvers share
    public MPC(Lock verticesLock)
    {
        _flag = false;
        _verticesLock = verticesLock;
    }

    public void SetEdge(in Edge aE) => _e = aE;

    public Edge Edge() => _e;

    public void SetFace(in Face aF) => _f = aF;

    public Face Face() => _f;

    public void SetFlag(bool bFlag) => _flag = bFlag;

    public bool Flag() => _flag;

    public void SetData(in Edge aEz, in Vertex aV1, double aT1, in Vertex aV2, double aT2)
    {
        _ez = aEz;
        _v1 = aV1;
        _t1 = aT1;
        _v2 = aV2;
        _t2 = aT2;
    }

    public void SetContext(IntersectionContext aContext) => _context = aContext;

    public IntersectionContext? Context() => _context;

    public override void Perform()
    {
        using var ps = new ProgressScope(_progressRange, null, 1);
        if (UserBreak(ps))
        {
            return;
        }
        try
        {
            // Check if edge has pcurve. If no then make its copy to avoid data races,
            // and use it to build pcurve.
            Edge copyE = _e;
            double f, l;
            Curve2D? c2d = Tool.CurveOnSurface(copyE, _f, out f, out l);
            if (c2d is null)
            {
                copyE = AlgoTools.CopyEdge(copyE);

                int iErr = 1;
                if (!_ez.IsNull)
                {
                    // Attach pcurve from the original edge
                    AlgoTools.MakeSplitEdge(_ez, _v1, _t1, _v2, _t2, out Edge spz);
                    iErr = AlgoTools2D.AttachExistingPCurve(spz, copyE, _f, _context);
                }
                if (iErr != 0)
                {
                    AlgoTools2D.BuildPCurveForEdgeOnFace(copyE, _f, _context);
                }

                _newC2d = Tool.CurveOnSurface(copyE, _f, out f, out l);
                if (_newC2d is null)
                {
                    AddError(new AlertBuildingPCurveFailed(new Shape()));
                    return;
                }
                else
                {
                    _newTol = Tool.Tolerance(copyE);
                }
            }
            else
            {
                FaceSurfaceAdaptor bas = _context!.SurfaceAdaptor(_f);
                if (bas.IsUPeriodic || bas.IsVPeriodic)
                {
                    // The curve already exists. Adjust it for periodic cases.
                    AlgoTools2D.AdjustPCurveOnSurf(_context.SurfaceAdaptor(_f), f, l, c2d, out _newC2d);
                    if (_newC2d != c2d)
                    {
                        _newTol = Tool.Tolerance(copyE);
                    }
                    else
                    {
                        _newC2d = null;
                    }
                }
            }

            if (_flag)
            {
                // OCCT: updates the tolerances of the edge's vertices in place, from parallel solvers that may share
                // a vertex (unsynchronised read-compare-writes: the port takes a lock, decision 789), and in
                // non-destructive mode a section edge may have an argument vertex (see BOPAlgo_PaveFiller.UpdateVertices).
                PaveFiller.UpdateVertices(copyE, _f, _verticesLock);
            }
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            AddError(new AlertBuildingPCurveFailed(new Shape()));
        }
    }

    public Curve2D? GetNewPCurve() => _newC2d;

    public double GetNewTolerance() => _newTol;
}

/// <summary>The p-curve of an edge on a planar face (<c>BRepLib.BuildPCurveForEdgeOnPlane</c>), as a parallel solver.</summary>
internal sealed class BPC : BooleanParallel.ISolver
{
    private Edge _e = new();
    private Face _f = new();
    private Curve2D? _curve;
    private bool _toUpdate;
    private ProgressRange _range;

    public void SetFace(in Face aF) => _f = aF;

    public void SetEdge(in Edge aE) => _e = aE;

    public Edge GetEdge() => _e;

    public Face GetFace() => _f;

    public Curve2D? GetCurve2d() => _curve;

    public bool IsToUpdate => _toUpdate;

    public void SetRange(ProgressRange range) => _range = range;

    public void Perform()
    {
        using var ps = new ProgressScope(_range, null, 1);
        if (!ps.More())
        {
            return;
        }
        BrepLibrary.BuildPCurveForEdgeOnPlane(_e, _f, out _curve, out _toUpdate);
    }
}

internal partial class PaveFiller
{
    /// <summary>Makes the split edges of the pave blocks (one per common block).</summary>
    protected void MakeSplitEdges(ProgressRange range)
    {
        List<NCollection_List<PaveBlock>> pbp = _ds!.ChangePaveBlocksPool();
        int nbPBP = pbp.Count;
        using var psOuter = new ProgressScope(range, null, 1);
        if (nbPBP == 0)
        {
            return;
        }
        //
        int i, nE, nV1, nV2, nSp, nbVBSE, k;
        double t1, t2;
        NCollection_List<PaveBlock>.Iterator itPB = new();
        PaveBlock pb;
        HashSet<CommonBlock> mcb = new();
        Vertex v1, v2;
        Edge e;
        List<SplitEdgeAlgorithm> vbse = new();

        //
        UpdateCommonBlocksWithSDVertices();
        //
        nbPBP = pbp.Count;
        //
        for (i = 0; i < nbPBP; ++i)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            NCollection_List<PaveBlock> lpb = pbp[i];
            //
            itPB.Initialize(lpb);
            for (; itPB.More(); itPB.Next())
            {
                pb = itPB.Value();
                nE = pb.OriginalEdge();
                ShapeInfo sie = _ds.ShapeInfo(nE);
                if (sie.HasFlag())
                {
                    // Skip degenerated edges
                    continue;
                }

                CommonBlock? cb = _ds.CommonBlock(pb);
                bool bCB = cb is not null;
                if (bCB && !mcb.Add(cb!))
                {
                    continue;
                }

                pb.Indices(out nV1, out nV2);
                // Check if it is necessary to make the split of the edge
                {
                    bool bV1 = _ds.IsNewShape(nV1);
                    bool bV2 = _ds.IsNewShape(nV2);

                    bool bToSplit = true;
                    if (!bV1 && !bV2) // no new vertices here
                    {
                        if (!_nonDestructive || !bCB)
                        {
                            if (bCB)
                            {
                                // Find the edge with these vertices
                                NCollection_List<PaveBlock>.Iterator it = new(cb!.PaveBlocks());
                                for (; it.More(); it.Next())
                                {
                                    nE = it.Value().OriginalEdge();
                                    if (_ds.PaveBlocks(nE).Extent() == 1)
                                    {
                                        break;
                                    }
                                }
                                if (it.More())
                                {
                                    // The pave block is found
                                    bToSplit = false;
                                    cb.SetRealPaveBlock(it.Value());
                                    cb.SetEdge(nE);
                                    // Compute tolerance of the common block and update the edge
                                    double tol = BuilderTools.ComputeToleranceOfCB(cb, _ds, _context);
                                    UpdateEdgeTolerance(nE, tol);
                                }
                            }
                            else if (lpb.Extent() == 1)
                            {
                                bToSplit = false;
                                pb.SetEdge(nE);
                            }
                            if (!bToSplit)
                            {
                                continue;
                            }
                        }
                    }
                }

                // Split the edge
                if (bCB)
                {
                    pb = cb!.PaveBlock1();
                    nE = pb.OriginalEdge();
                    pb.Indices(out nV1, out nV2);
                }
                pb.Range(out t1, out t2);
                //
                e = ShapeCasts.Edge(_ds.Shape(nE));
                e.Orient(Forward);
                //
                v1 = ShapeCasts.Vertex(_ds.Shape(nV1));
                v1.Orient(Forward);
                //
                v2 = ShapeCasts.Vertex(_ds.Shape(nV2));
                v2.Orient(Reversed);
                //
                SplitEdgeAlgorithm bse =  new SplitEdgeAlgorithm();
                vbse.Add(bse);
                //
                bse.SetData(e, v1, t1, v2, t2);
                bse.SetPaveBlock(pb);
                if (bCB)
                {
                    bse.SetCommonBlock(cb);
                }
                bse.SetDS(_ds);
            } // for (; aItPB.More(); aItPB.Next()) {
        } // for (i=0; i<aNbPBP; ++i) {
        //
        nbVBSE = vbse.Count;
        using var ps = new ProgressScope(psOuter.Next(), "Splitting edges", nbVBSE);
        for (k = 0; k < nbVBSE; k++)
        {
            SplitEdgeAlgorithm bse = vbse[k];
            bse.SetProgressRange(ps.Next());
        }
        //======================================================
        BooleanParallel.Perform(_runParallel, vbse, _context!);
        //======================================================
        if (HasErrors)
        {
            return;
        }
        //
        for (k = 0; k < nbVBSE; ++k)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            SplitEdgeAlgorithm bse = vbse[k];
            //
            Edge sp = bse.SplitEdge();
            Box box = bse.Box();
            //
            PaveBlock pBk = bse.PaveBlock();
            CommonBlock? cBk = bse.CommonBlock();
            //
            ShapeInfo si = new();
            si.SetShapeType(ShapeType.Edge);
            si.SetShape(sp);
            si.ChangeBox() = box;
            NCollection_List<int> subShapes = si.ChangeSubShapes();
            subShapes.Append(pBk.Pave1().Index());
            subShapes.Append(pBk.Pave2().Index());
            //
            nSp = _ds.Append(si);
            //
            if (cBk is not null)
            {
                UpdateEdgeTolerance(nSp, bse.Tolerance());
                cBk.SetEdge(nSp);
            }
            else
            {
                pBk.SetEdge(nSp);
            }
        }
    }

    /// <summary>Splits the edge <paramref name="nE"/> between the two vertices and appends the split to the DS.</summary>
    protected int SplitEdge(int nE, int nV1, double aT1, int nV2, double aT2)
    {
        int nSp;
        Vertex v1, v2;
        Edge e, sp;
        ShapeInfo si = new();
        //
        si.SetShapeType(ShapeType.Edge);
        //
        e = ShapeCasts.Edge(_ds!.Shape(nE));
        e.Orient(Forward);
        //
        v1 = ShapeCasts.Vertex(_ds.Shape(nV1));
        v1.Orient(Forward);
        //
        v2 = ShapeCasts.Vertex(_ds.Shape(nV2));
        v2.Orient(Reversed);
        //
        AlgoTools.MakeSplitEdge(e, v1, aT1, v2, aT2, out sp);
        //
        si.SetShape(sp);
        //
        ref Box box = ref si.ChangeBox();
        Bounds.Add(sp, ref box);
        box.SetGap(box.GetGap() + global::Kerf.Engine.Foundation.Precision.Confusion());
        //
        nSp = _ds.Append(si);
        return nSp;
    }

    /// <summary>
    /// Builds the p-curves of the edges in and on the faces, and of the section edges on their faces when the section
    /// attribute asks for them.
    /// </summary>
    /// <remarks>
    /// The p-curves are computed in parallel on copies of the edges (BOPAlgo_MPC) and stored afterwards, serially,
    /// with <c>UpdateEdge</c> into the DS edge. In non-destructive mode that edge is a new one wherever a p-curve is
    /// missing: an argument edge in or on another argument's face is in a common block, which MakeSplitEdges then
    /// always splits (decisions 577 and 719).
    /// </remarks>
    protected void MakePCurves(ProgressRange range)
    {
        using var psOuter = new ProgressScope(range, null, 1);
        if (_avoidBuildPCurve || (!_sectionAttribute.PCurveOnS1() && !_sectionAttribute.PCurveOnS2()))
        {
            return;
        }
        bool bHasPC;
        int i, nF1, nbC, k, nE, nbFF, nbFI, nEx;
        int j, nbPBIn, nbPBOn;
        NCollection_List<PaveBlock>.Iterator itLPB = new();
        Face f1F;
        List<MPC> vmpc = new();
        //
        // 1. Process Common Blocks
        List<FaceInfo> fip = _ds!.FaceInfoPool();
        //
        nbFI = fip.Count;
        for (i = 0; i < nbFI; ++i)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            FaceInfo fi = fip[i];
            nF1 = fi.Index();
            //
            f1F = ShapeCasts.Face(_ds.Shape(nF1));
            f1F.Orient(Forward);
            // In
            IndexedSet<PaveBlock> mpbIn = fi.PaveBlocksIn();
            nbPBIn = mpbIn.Count;
            for (j = 1; j <= nbPBIn; ++j)
            {
                PaveBlock pb = mpbIn[j - 1];
                nE = pb.Edge();
                // OCCT's Standard_ASSERT(nE >= 0, ..., continue): in release builds it only continues.
                if (nE < 0)
                {
                    continue;
                }
                Edge e = ShapeCasts.Edge(_ds.Shape(nE));
                //
                MPC mpc =  new MPC(_verticesLock);
                vmpc.Add(mpc);
                mpc.SetEdge(e);
                mpc.SetFace(f1F);
            }
            //
            // On
            IndexedSet<PaveBlock> mpbOn = fi.PaveBlocksOn();
            nbPBOn = mpbOn.Count;
            for (j = 1; j <= nbPBOn; ++j)
            {
                PaveBlock pb = mpbOn[j - 1];
                nE = pb.Edge();
                Edge e = ShapeCasts.Edge(_ds.Shape(nE));
                bHasPC = AlgoTools2D.HasCurveOnSurface(e, f1F);
                if (bHasPC)
                {
                    continue;
                }
                //
                MPC mpc =  new MPC(_verticesLock);
                vmpc.Add(mpc);

                CommonBlock? cb = _ds.CommonBlock(pb);

                if (cb is not null)
                {
                    NCollection_List<PaveBlock> lpb = cb.PaveBlocks();
                    if (lpb.Extent() >= 2)
                    {
                        itLPB.Initialize(lpb);
                        for (; itLPB.More(); itLPB.Next())
                        {
                            PaveBlock pBx = itLPB.Value();
                            if (pBx == pb)
                            {
                                continue;
                            }
                            //
                            nEx = pBx.OriginalEdge();
                            Edge ex = ShapeCasts.Edge(_ds.Shape(nEx));
                            bHasPC = AlgoTools2D.HasCurveOnSurface(ex, f1F);
                            if (!bHasPC)
                            {
                                continue;
                            }
                            //
                            int nV1x, nV2x;
                            double t1x, t2x;
                            Vertex v1x, v2x;
                            Edge ez;
                            //
                            ez = ex;
                            ez.Orient(Forward);
                            //
                            pBx.Indices(out nV1x, out nV2x);
                            pBx.Range(out t1x, out t2x);
                            //
                            v1x = ShapeCasts.Vertex(_ds.Shape(nV1x));
                            v1x.Orient(Forward);
                            //
                            v2x = ShapeCasts.Vertex(_ds.Shape(nV2x));
                            v2x.Orient(Reversed);
                            //
                            mpc.SetData(ez, v1x, t1x, v2x, t2x);
                            //
                            break;
                        }
                    }
                }

                mpc.SetEdge(e);
                mpc.SetFace(f1F);
            }
        } // for (i=0; i<aNbFI; ++i) {
        //
        // 2. Process section edges. P-curves on them must already be computed.
        // However, we must provide the call to UpdateVertices.
        Span<bool> bPCurveOnS = stackalloc bool[2];
        bPCurveOnS[0] = _sectionAttribute.PCurveOnS1();
        bPCurveOnS[1] = _sectionAttribute.PCurveOnS2();
        //
        if (bPCurveOnS[0] || bPCurveOnS[1])
        {
            // container to remember already added edge-face pairs
            HashSet<Pair> efPairs = new();
            List<InterfFF> fFs = _ds.InterfFF();
            nbFF = fFs.Count;
            Span<int> nF = stackalloc int[2];
            Face[] aFf = new Face[2];
            for (i = 0; i < nbFF; ++i)
            {
                InterfFF ff = fFs[i];
                List<SectionCurve> vnc = ff.Curves();
                nbC = vnc.Count;
                if (nbC == 0)
                {
                    continue;
                }
                ff.Indices(out nF[0], out nF[1]);
                //
                aFf[0] = ShapeCasts.Face(_ds.Shape(nF[0]));
                aFf[0].Orient(Forward);
                //
                aFf[1] = ShapeCasts.Face(_ds.Shape(nF[1]));
                aFf[1].Orient(Forward);
                //
                for (k = 0; k < nbC; ++k)
                {
                    SectionCurve nc = vnc[k];
                    NCollection_List<PaveBlock> lpb = nc.PaveBlocks();
                    itLPB.Initialize(lpb);
                    for (; itLPB.More(); itLPB.Next())
                    {
                        PaveBlock pb = itLPB.Value();
                        nE = pb.Edge();
                        Edge e = ShapeCasts.Edge(_ds.Shape(nE));
                        //
                        for (int m = 0; m < 2; ++m)
                        {
                            if (bPCurveOnS[m] && efPairs.Add(new Pair(nE, nF[m])))
                            {
                                MPC mpc =  new MPC(_verticesLock);
                                vmpc.Add(mpc);
                                mpc.SetEdge(e);
                                mpc.SetFace(aFf[m]);
                                mpc.SetFlag(true);
                            }
                        }
                    }
                }
            } // for (i=0; i<aNbFF; ++i) {
        } // if (bPCurveOnS1 || bPCurveOnS2 ) {
        //
        using var ps = new ProgressScope(psOuter.Next(), "Projecting edges on faces", vmpc.Count);
        for (i = 0; i < vmpc.Count; i++)
        {
            MPC mpc = vmpc[i];
            mpc.SetProgressRange(ps.Next());
        }
        //======================================================
        BooleanParallel.Perform(_runParallel, vmpc, _context!);
        //======================================================
        if (HasErrors)
        {
            return;
        }

        // Add warnings of the failed projections and update edges with new pcurves
        int nb = vmpc.Count;
        for (i = 0; i < nb; ++i)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            MPC mpc = vmpc[i];
            if (mpc.HasErrors)
            {
                Compound wc = new();
                new BrepBuilder().MakeCompound(ref wc);
                new BrepBuilder().Add(wc, mpc.Edge());
                new BrepBuilder().Add(wc, mpc.Face());
                AddWarning(new AlertBuildingPCurveFailed(wc));
            }
            else
            {
                Curve2D? newPC = mpc.GetNewPCurve();
                // if aNewPC is null we do not need to update the edge because it already contains
                // valid p-curve, and only vertices have been updated.
                if (newPC is not null)
                {
                    new BrepBuilder().UpdateEdge(mpc.Edge(), newPC, mpc.Face(), mpc.GetNewTolerance());
                }
            }
        }
    }

    // This filler's solvers share vertices (decision 789); another operation's have their own working copies.
    private readonly Lock _verticesLock = new();

    /// <summary>
    /// Updates the tolerances of the edge's vertices comparing the extremities of its 3D curve and of its p-curve on
    /// the face (OCCT's file-static <c>UpdateVertices</c>; writes into the vertices in place).
    /// </summary>
    internal static void UpdateVertices(in Edge aE, in Face aF, Lock verticesLock)
    {
        // Kerf: the solvers of MakePCurves run this in parallel on section edges that share vertices. OCCT's
        // tolerance update (a read-compare-write in BRep_TVertex::UpdateTolerance) can lose a larger value to a
        // concurrent smaller one; under the lock every vertex ends with the largest value, which is the serial
        // result in any order (decision 789).
        int j;
        double ux, vx, tolV2, d2, d;
        Span<double> t = stackalloc double[2];
        Point3D p3D, p3Dx;
        Point2D p2Dx;
        Surface s;
        Curve3D c3D;
        Curve2D c2D;
        Edge ef;
        Vertex[] v = new Vertex[2];
        BrepBuilder bb = new();
        //
        ef = aE;
        ef.Orient(Forward);
        //
        Exploration.Vertices(ef, out v[0], out v[1]);
        //
        s = Tool.Surface(aF)!;
        c3D = Tool.Curve(ef, out t[0], out t[1])!;
        c2D = Tool.CurveOnSurface(ef, aF, out t[0], out t[1])!;
        //
        for (j = 0; j < 2; ++j)
        {
            tolV2 = Tool.Tolerance(v[j]);
            tolV2 = tolV2 * tolV2;
            //
            c3D.D0(t[j], out p3D);
            c2D.D0(t[j], out p2Dx);
            p2Dx.Coord(out ux, out vx);
            s.D0(ux, vx, out p3Dx);
            d2 = p3D.SquareDistanceTo(p3Dx);
            if (d2 > tolV2)
            {
                d = Math.Sqrt(d2);
                lock (verticesLock)
                {
                    bb.UpdateVertex(v[j], d + AlgoTools.DTolerance());
                }
            }
        }
    }

    /// <summary>
    /// Builds the p-curves of the argument edges on the planar faces that interfere (destructive mode only: the
    /// p-curves are stored in the argument edges).
    /// </summary>
    protected void Prepare(ProgressRange range)
    {
        if (_nonDestructive)
        {
            // do not allow storing pcurves in original edges if non-destructive mode is on
            return;
        }
        Span<ShapeType> type = [ShapeType.Vertex, ShapeType.Edge, ShapeType.Face];
        bool bIsBasedOnPlane;
        int i, nb, n1, nF, nbF;
        Explorer exp = new();
        IndexedSet<Shape> mf = new(ShapeHasher.Instance);
        //
        nb = 3;
        using var psOuter = new ProgressScope(range, null, 1);
        for (i = 0; i < nb; ++i)
        {
            _iterator!.Initialize(type[i], type[2]);
            for (; _iterator.More(); _iterator.Next())
            {
                _iterator.Value(out n1, out nF);
                Face f = ShapeCasts.Face(_ds!.Shape(nF));
                //
                bIsBasedOnPlane = IsBasedOnPlane(f);
                if (bIsBasedOnPlane)
                {
                    mf.Add(f);
                }
            }
        }
        //
        nbF = mf.Count;
        if (nbF == 0)
        {
            return;
        }
        //
        // Build pcurves of edges on planes; first collect pairs edge-face.
        List<BPC> vbpc = new();
        //
        for (i = 1; i <= nbF; ++i)
        {
            Face f = ShapeCasts.Face(mf[i - 1]);
            exp.Init(f, type[1]);
            for (; exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                BPC bpc =  new BPC();
                vbpc.Add(bpc);
                bpc.SetEdge(e);
                bpc.SetFace(f);
            }
        }
        //
        using var ps = new ProgressScope(psOuter.Next(), "Building 2d curves on planar faces", vbpc.Count);
        for (i = 0; i < vbpc.Count; i++)
        {
            BPC bpc = vbpc[i];
            bpc.SetRange(ps.Next());
        }
        //======================================================
        BooleanParallel.Perform(_runParallel, vbpc);
        //======================================================
        if (UserBreak(ps))
        {
            return;
        }
        // pcurves are built, and now update edges
        BrepBuilder bb = new();
        for (i = 0; i < vbpc.Count; i++)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            BPC bpc = vbpc[i];
            if (bpc.IsToUpdate)
            {
                double tolE = Tool.Tolerance(bpc.GetEdge());
                bb.UpdateEdge(bpc.GetEdge(), bpc.GetCurve2d(), bpc.GetFace(), tolE);
            }
        }
    }

    // Whether the face's surface is a plane (or a trimmed plane).
    private static bool IsBasedOnPlane(in Face aF)
    {
        TrimmedSurface? grts;
        PlaneSurface? gp;

        Surface? s = Tool.Surface(aF, out Placement _);
        grts = s as TrimmedSurface;
        if (grts is not null)
        {
            gp = grts.BasisSurface as PlaneSurface;
        }
        else
        {
            gp = s as PlaneSurface;
        }
        //
        return gp is not null;
    }

    /// <summary>
    /// Kerf (decision 32901): each section edge's curves end at its vertices. A section's curve is put on a vertex it passes
    /// within the tolerances of, so the edge's curve in space and its curves on the two faces can end short of the vertex or
    /// past it, by what the walk was out by there; near a touch that's far more than a tolerance, as the surfaces lie within
    /// one of each other over a band. Each face's boundary is then open in its parameters at that vertex. Here the edge
    /// takes its own piece of each B-spline, with its end pole moved to the vertex: the vertex's point in space, and the
    /// point of the face's parameters nearest it.
    /// </summary>
    protected void SectionsEndAtTheirVertices()
    {
        // Where each vertex is on a face's own edges: the point of the edge at the vertex's parameter. An edge given as a
        // line or a circle can't be moved to the vertex, so the sections' curves go to where it ends. A vertex that one
        // face's edges end at in different places is left out, and the curves go to its own point
        Dictionary<Pair, Point3D> onEdges = new();
        HashSet<Pair> apart = new();
        HashSet<Pair> seen = new();
        for (int nF = 0; nF < _ds!.SourceShapeCount; ++nF)
        {
            if (_ds.ShapeInfo(nF).ShapeType() != ShapeType.Face)
            {
                continue;
            }

            for (Explorer edges = new(_ds.Shape(nF), ShapeType.Edge); edges.More(); edges.Next())
            {
                int nE = _ds.Index(edges.Current());
                if (nE < 0 || !seen.Add(new Pair(nE, nF)) || !_ds.HasPaveBlocks(nE))
                {
                    continue;
                }

                Edge edge = ShapeCasts.Edge(_ds.Shape(nE));
                if (Tool.Degenerated(edge))
                {
                    continue;
                }

                EdgeCurveAdaptor curve = new(edge);
                for (NCollection_List<PaveBlock>.Iterator blocks = new(_ds.PaveBlocks(nE)); blocks.More(); blocks.Next())
                {
                    for (int end = 0; end < 2; ++end)
                    {
                        Pave pave = end == 0 ? blocks.Value().Pave1() : blocks.Value().Pave2();
                        Pair key = new(SameDomain(pave.Index()), nF);
                        Point3D point = curve.Value(pave.Parameter());
                        if (!onEdges.TryAdd(key, point) && onEdges[key].DistanceTo(point) > global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            apart.Add(key);
                        }
                    }
                }
            }
        }
        foreach (Pair key in apart)
        {
            onEdges.Remove(key);
        }

        HashSet<int> done = new();
        foreach (InterfFF ff in _ds.InterfFF())
        {
            ff.Indices(out int nF1, out int nF2);
            foreach (SectionCurve section in ff.Curves())
            {
                for (NCollection_List<PaveBlock>.Iterator blocks = new(section.PaveBlocks()); blocks.More(); blocks.Next())
                {
                    int nE = blocks.Value().Edge();
                    if (nE >= 0 && _ds.IsNewShape(nE) && done.Add(nE))
                    {
                        EndAtItsVertices(nE, nF1, nF2, onEdges);
                    }
                }
            }
        }
    }

    private int SameDomain(int nV)
    {
        int same = nV;
        return nV >= 0 && _ds!.HasShapeSD(nV, ref same) ? same : nV;
    }

    // Where a curve on this face that ends at a vertex ends: on the face's edge the vertex is on, or else at its own point
    private Point3D EndAt(in Vertex vertex, int nF, Dictionary<Pair, Point3D> onEdges)
    {
        Point3D point = Tool.Pnt(vertex);
        int nV = _ds!.Index(vertex);
        return nV >= 0 && onEdges.TryGetValue(new Pair(SameDomain(nV), nF), out Point3D onEdge)
               && onEdge.DistanceTo(point) <= Tool.Tolerance(vertex) + global::Kerf.Engine.Foundation.Precision.Confusion()
            ? onEdge
            : point;
    }

    private void EndAtItsVertices(int nE, int nF1, int nF2, Dictionary<Pair, Point3D> onEdges)
    {
        Edge edge = ShapeCasts.Edge(_ds!.Shape(nE));
        edge.Orient(Forward);
        if (Tool.Degenerated(edge))
        {
            return;
        }

        Exploration.Vertices(edge, out Vertex first, out Vertex last);
        if (first.IsNull || last.IsNull || first.IsSame(last))
        {
            return;
        }

        Curve3D? curve = Tool.Curve(edge, out Placement curvePlacement, out double from, out double to);
        if (curve is null || !curvePlacement.IsIdentity || !(to > from))
        {
            return;
        }

        // A curve in space that isn't a B-spline, as the ellipse two cylinders are taken to meet in, stays where it is, and
        // only the curves on the faces move
        BSplineCurve3D? spline = curve as BSplineCurve3D;
        double confusion = global::Kerf.Engine.Foundation.Precision.Confusion();
        // In space the curve goes to where the first face's edges end, which the vertex covers as it does the second's
        Point3D a = EndAt(first, nF1, onEdges), b = EndAt(last, nF1, onEdges);
        if (a.IsEqual(Tool.Pnt(first), 0.0))
        {
            a = EndAt(first, nF2, onEdges);
        }
        if (b.IsEqual(Tool.Pnt(last), 0.0))
        {
            b = EndAt(last, nF2, onEdges);
        }
        bool open = spline is not null && (spline.Value(from).DistanceTo(a) > confusion || spline.Value(to).DistanceTo(b) > confusion);

        // Each face's curve, where it's a B-spline over the edge's range, with where its ends should be
        Span<int> faces = [nF1, nF2];
        Face[] onFaces = new Face[2];
        BSplineCurve2D?[] onSurfaces = new BSplineCurve2D?[2];
        Span<Point2D> ends = stackalloc Point2D[4];
        for (int side = 0; side < 2; ++side)
        {
            Face face = ShapeCasts.Face(_ds.Shape(faces[side]));
            face.Orient(Forward);
            onFaces[side] = face;
            Surface? surface = Tool.Surface(face, out Placement placement);
            if (surface is null || (side == 1 && nF2 == nF1) || Tool.IsClosed(edge, face)
                || Tool.CurveOnSurface(edge, face, out double from2, out double to2) is not BSplineCurve2D onSurface || from2 != from || to2 != to)
            {
                continue;
            }

            Point3D aOn = EndAt(first, faces[side], onEdges), bOn = EndAt(last, faces[side], onEdges);
            if (!placement.IsIdentity)
            {
                aOn = aOn.Transformed(placement.Inverted().Transformation);
                bOn = bOn.Transformed(placement.Inverted().Transformation);
            }
            Point2D start = onSurface.Value(from), end = onSurface.Value(to);
            open |= surface.Value(start.X, start.Y).DistanceTo(aOn) > confusion || surface.Value(end.X, end.Y).DistanceTo(bOn) > confusion;
            onSurfaces[side] = onSurface;
            ends[2 * side] = Nearest(surface, start, aOn);
            ends[2 * side + 1] = Nearest(surface, end, bOn);
        }

        if (!open)
        {
            return;
        }

        // The edge's own piece of each curve. Cutting one to a range within a parametric confusion of a knot ends it at the
        // knot, so the edge's range is taken from the cut curve, and every curve has to agree on it
        try
        {
            double ownFrom = from, ownTo = to;
            BSplineCurve3D? own = null;
            if (spline is not null)
            {
                own = (BSplineCurve3D)spline.Copy();
                own.Segment(from, to);
                ownFrom = own.FirstParameter;
                ownTo = own.LastParameter;
                double near = 2.0 * global::Kerf.Engine.Foundation.Precision.PConfusion();
                if (Math.Abs(ownFrom - from) > near || Math.Abs(ownTo - to) > near)
                {
                    return;
                }

                own.SetPole(1, a);
                own.SetPole(own.PoleCount, b);
            }

            BSplineCurve2D?[] owns = new BSplineCurve2D?[2];
            for (int side = 0; side < 2; ++side)
            {
                if (onSurfaces[side] is not { } onSurface)
                {
                    continue;
                }

                BSplineCurve2D own2 = (BSplineCurve2D)onSurface.Copy();
                own2.Segment(from, to);
                if (own2.FirstParameter != ownFrom || own2.LastParameter != ownTo)
                {
                    return;
                }

                own2.SetPole(1, ends[2 * side]);
                own2.SetPole(own2.PoleCount, ends[2 * side + 1]);
                owns[side] = own2;
            }

            BrepBuilder builder = new();
            double tolerance = Tool.Tolerance(edge);
            if (own is not null)
            {
                builder.UpdateEdge(edge, own, tolerance);
            }
            for (int side = 0; side < 2; ++side)
            {
                if (owns[side] is { } own2)
                {
                    builder.UpdateEdge(edge, own2, onFaces[side], tolerance);
                }
            }
            if (own is not null)
            {
                builder.Range(edge, ownFrom, ownTo);
            }
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            return;
        }

        double worst = Tool.Tolerance(edge);
        for (int side = 0; side < 2; ++side)
        {
            if (onSurfaces[side] is not null && AlgoTools.ComputeTolerance(onFaces[side], edge, out double off, out _))
            {
                worst = Max(worst, off);
            }
        }
        if (worst > Tool.Tolerance(edge))
        {
            UpdateEdgeTolerance(nE, worst);
        }
    }

    // The point of the surface's parameters nearest a point in space, from one near it
    private static Point2D Nearest(Surface surface, in Point2D from, in Point3D to)
    {
        double u = from.X, v = from.Y;
        for (int i = 0; i < 4; ++i)
        {
            surface.D1(u, v, out Point3D at, out Vector3D du, out Vector3D dv);
            Vector3D off = new(at, to);
            double uu = du.Dot(du), uv = du.Dot(dv), vv = dv.Dot(dv), determinant = uu * vv - uv * uv;
            if (!(determinant > 1e-12 * uu * vv))
            {
                break;
            }

            double pu = off.Dot(du), pv = off.Dot(dv);
            u += (pu * vv - pv * uv) / determinant;
            v += (pv * uu - pu * uv) / determinant;
        }
        return new Point2D(u, v);
    }
}
