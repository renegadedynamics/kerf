// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_7.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
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
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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

    public MPC()
    {
        _flag = false;
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
                PaveFiller.UpdateVertices(copyE, _f);
            }
        }
        catch (FailureException)
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

public partial class PaveFiller
{
    /// <summary>Makes the split edges of the pave blocks (one per common block).</summary>
    protected void MakeSplitEdges(ProgressRange range)
    {
        NCollection_DynamicArray<NCollection_List<PaveBlock>> pbp = _ds!.ChangePaveBlocksPool();
        int nbPBP = pbp.Length();
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
        NCollection_Map<CommonBlock> mcb = new();
        Vertex v1, v2;
        Edge e;
        NCollection_DynamicArray<SplitEdgeAlgorithm> vbse = new();

        //
        UpdateCommonBlocksWithSDVertices();
        //
        nbPBP = pbp.Length();
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
                e.Orientation(Forward);
                //
                v1 = ShapeCasts.Vertex(_ds.Shape(nV1));
                v1.Orientation(Forward);
                //
                v2 = ShapeCasts.Vertex(_ds.Shape(nV2));
                v2.Orientation(Reversed);
                //
                SplitEdgeAlgorithm bse = vbse.Append(new SplitEdgeAlgorithm());
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
        nbVBSE = vbse.Length();
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
        e.Orientation(Forward);
        //
        v1 = ShapeCasts.Vertex(_ds.Shape(nV1));
        v1.Orientation(Forward);
        //
        v2 = ShapeCasts.Vertex(_ds.Shape(nV2));
        v2.Orientation(Reversed);
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
        NCollection_DynamicArray<MPC> vmpc = new();
        //
        // 1. Process Common Blocks
        NCollection_DynamicArray<FaceInfo> fip = _ds!.FaceInfoPool();
        //
        nbFI = fip.Length();
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
            f1F.Orientation(Forward);
            // In
            NCollection_IndexedMap<PaveBlock> mpbIn = fi.PaveBlocksIn();
            nbPBIn = mpbIn.Extent();
            for (j = 1; j <= nbPBIn; ++j)
            {
                PaveBlock pb = mpbIn[j];
                nE = pb.Edge();
                // OCCT's Standard_ASSERT(nE >= 0, ..., continue): in release builds it only continues.
                if (nE < 0)
                {
                    continue;
                }
                Edge e = ShapeCasts.Edge(_ds.Shape(nE));
                //
                MPC mpc = vmpc.Append(new MPC());
                mpc.SetEdge(e);
                mpc.SetFace(f1F);
            }
            //
            // On
            NCollection_IndexedMap<PaveBlock> mpbOn = fi.PaveBlocksOn();
            nbPBOn = mpbOn.Extent();
            for (j = 1; j <= nbPBOn; ++j)
            {
                PaveBlock pb = mpbOn[j];
                nE = pb.Edge();
                Edge e = ShapeCasts.Edge(_ds.Shape(nE));
                bHasPC = AlgoTools2D.HasCurveOnSurface(e, f1F);
                if (bHasPC)
                {
                    continue;
                }
                //
                MPC mpc = vmpc.Append(new MPC());

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
                            ez.Orientation(Forward);
                            //
                            pBx.Indices(out nV1x, out nV2x);
                            pBx.Range(out t1x, out t2x);
                            //
                            v1x = ShapeCasts.Vertex(_ds.Shape(nV1x));
                            v1x.Orientation(Forward);
                            //
                            v2x = ShapeCasts.Vertex(_ds.Shape(nV2x));
                            v2x.Orientation(Reversed);
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
            NCollection_Map<Pair> efPairs = new();
            NCollection_DynamicArray<InterfFF> fFs = _ds.InterfFF();
            nbFF = fFs.Length();
            Span<int> nF = stackalloc int[2];
            Face[] aFf = new Face[2];
            for (i = 0; i < nbFF; ++i)
            {
                InterfFF ff = fFs[i];
                NCollection_DynamicArray<SectionCurve> vnc = ff.Curves();
                nbC = vnc.Length();
                if (nbC == 0)
                {
                    continue;
                }
                ff.Indices(out nF[0], out nF[1]);
                //
                aFf[0] = ShapeCasts.Face(_ds.Shape(nF[0]));
                aFf[0].Orientation(Forward);
                //
                aFf[1] = ShapeCasts.Face(_ds.Shape(nF[1]));
                aFf[1].Orientation(Forward);
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
                                MPC mpc = vmpc.Append(new MPC());
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
        using var ps = new ProgressScope(psOuter.Next(), "Projecting edges on faces", vmpc.Length());
        for (i = 0; i < vmpc.Length(); i++)
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
        int nb = vmpc.Length();
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

    private static readonly Lock VerticesLock = new();

    /// <summary>
    /// Updates the tolerances of the edge's vertices comparing the extremities of its 3D curve and of its p-curve on
    /// the face (OCCT's file-static <c>UpdateVertices</c>; writes into the vertices in place).
    /// </summary>
    internal static void UpdateVertices(in Edge aE, in Face aF)
    {
        // OCCT.NET: the solvers of MakePCurves run this in parallel on section edges that share vertices. OCCT's
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
        ef.Orientation(Forward);
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
            d2 = p3D.SquareDistance(p3Dx);
            if (d2 > tolV2)
            {
                d = Math.Sqrt(d2);
                lock (VerticesLock)
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
        NCollection_IndexedMap<Shape> mf = new(ShapeHasher.Instance);
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
        nbF = mf.Extent();
        if (nbF == 0)
        {
            return;
        }
        //
        // Build pcurves of edges on planes; first collect pairs edge-face.
        NCollection_DynamicArray<BPC> vbpc = new();
        //
        for (i = 1; i <= nbF; ++i)
        {
            Face f = ShapeCasts.Face(mf[i]);
            exp.Init(f, type[1]);
            for (; exp.More(); exp.Next())
            {
                Edge e = ShapeCasts.Edge(exp.Current());
                BPC bpc = vbpc.Append(new BPC());
                bpc.SetEdge(e);
                bpc.SetFace(f);
            }
        }
        //
        using var ps = new ProgressScope(psOuter.Next(), "Building 2d curves on planar faces", vbpc.Length());
        for (i = 0; i < vbpc.Length(); i++)
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
        for (i = 0; i < vbpc.Length(); i++)
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
            gp = grts.BasisSurface() as PlaneSurface;
        }
        else
        {
            gp = s as PlaneSurface;
        }
        //
        return gp is not null;
    }
}
