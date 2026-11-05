// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_5.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Booleans.GlueMode;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>The intersection of a pave block of an edge with a face, as a parallel solver of the pave filler.</summary>
/// <remarks>
/// OCCT's class derives from both IntTools_EdgeFace and BOPAlgo_ParallelAlgo; here it derives from the first and holds
/// what the second gives (the progress range and a report, through a <see cref="Options"/>). Its
/// <see cref="Perform"/> hides the base's: when the shapes are far from the origin it moves the edge and the face near
/// it (<see cref="BuilderTools.TrsfToPoint(in Box, in Box, ref Transform3D)"/>), intersects, restores them and
/// gives the common parts back the unmoved edge. A moved face is a different key of the context (decision 756).
/// </remarks>
internal sealed class EdgeFaceJob : EdgeFace, BooleanParallel.IContextSolver<IntersectionContext>
{
    private readonly Options _options = new();
    private ProgressRange _progressRange;
    private int _ie;
    private int _if;
    private ParameterRange _newSR = new();
    private PaveBlock? _pb;
    private Box _box1 = new();
    private Box _box2 = new();

    public EdgeFaceJob()
    {
        _ie = -1;
        _if = -1;
    }

    public void SetIndices(int nE, int nF)
    {
        _ie = nE;
        _if = nF;
    }

    public void Indices(out int nE, out int nF)
    {
        nE = _ie;
        nF = _if;
    }

    public void SetNewSR(in ParameterRange aR) => _newSR = aR;

    public ParameterRange NewSR() => _newSR;

    public void SetPaveBlock(PaveBlock aPB) => _pb = aPB;

    public PaveBlock PaveBlock() => _pb!;

    public void SetBoxes(in Box box1, in Box box2)
    {
        _box1 = box1;
        _box2 = box2;
    }

    /// <summary>BOPAlgo_ParallelAlgo's range for the single run.</summary>
    public void SetProgressRange(ProgressRange range) => _progressRange = range;

    /// <summary>Whether the run failed (BOPAlgo_Options' errors).</summary>
    public bool HasErrors => _options.HasErrors;

    void BooleanParallel.IContextSolver<IntersectionContext>.SetContext(IntersectionContext context) => SetContext(context);

    public new void Perform()
    {
        using var ps = new ProgressScope(_progressRange, null, 1);
        if (_options.UserBreak(ps))
        {
            return;
        }
        Face face = _face;
        Edge edge = _edge;
        bool hasTrsf = false;
        try
        {
            Transform3D trsf = new();
            if (BuilderTools.TrsfToPoint(_box1, _box2, ref trsf))
            {
                // Shapes are located far from origin, move the shapes to the origin,
                // to increase the accuracy of intersection.
                Placement loc = new(trsf);
                _edge.Move(loc);
                _face.Move(loc);
                hasTrsf = true;
            }

            base.Perform();
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            _options.AddError(new AlertIntersectionFailed());
        }
        _face = face;
        _edge = edge;

        if (hasTrsf)
        {
            for (int i = 1; i <= _seqOfCommonPrts.Length(); ++i)
            {
                CommonPrt cPart = _seqOfCommonPrts[i];
                cPart.SetEdge1(_edge);
            }
        }
    }
}

internal partial class PaveFiller
{
    /// <summary>The edge/face interferences.</summary>
    protected virtual void PerformEF(ProgressRange range)
    {
        FillShrunkData(ShapeType.Edge, ShapeType.Face);
        //
        _iterator!.Initialize(ShapeType.Edge, ShapeType.Face);
        using var psOuter = new ProgressScope(range, null, 10);
        int iSize = _iterator.ExpectedLength();
        if (iSize == 0)
        {
            return;
        }
        //
        int nE, nF;
        //
        if (_glue == global::Kerf.Booleans.GlueMode.Full)
        {
            // there is no need to intersect edges with faces in this mode
            // just initialize FaceInfo for faces
            for (; _iterator.More(); _iterator.Next())
            {
                _iterator.Value(out nE, out nF);
                if (!_ds!.ShapeInfo(nE).HasFlag())
                {
                    _ds.ChangeFaceInfo(nF);
                }
            }
            return;
        }
        //
        Span<bool> bV = stackalloc bool[2];
        Span<bool> bIsOnPave = stackalloc bool[2];
        bool bIsPBSplittable;
        bool bV1, bV2, bExpressCompute;
        int nV1, nV2;
        int i, nbCPrts, iX;
        Span<int> nV = stackalloc int[2];
        int nbEdgeFace, k;
        double tolE, tolF, ts1 = 0.0, ts2 = 0.0, t1 = 0.0, t2 = 0.0;
        ShapeType type;
        NCollection_List<PaveBlock>.Iterator it = new();
        List<EdgeFaceJob> vEdgeFace = new();
        //-----------------------------------------------------scope f
        //
        NCollection_IntMap miefc = new(100);
        IndexedMap<Shape, CoupleOfPaveBlocks> mvcpb = new(ShapeHasher.Instance);
        IndexedMap<PaveBlock, NCollection_List<int>> mpbli = new();
        Dictionary<PaveBlock, Box> dmpbBox = new();
        //
        List<InterfEF> eFs = _ds!.InterfEF();
        //
        for (; _iterator.More(); _iterator.Next())
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            _iterator.Value(out nE, out nF);
            //
            ShapeInfo sie = _ds.ShapeInfo(nE);
            if (sie.HasFlag())
            { // degenerated
                continue;
            }
            //
            Edge e = ShapeCasts.Edge(sie.Shape());
            Face f = ShapeCasts.Face(_ds.Shape(nF));
            Box bbf = _ds.ShapeInfo(nF).Box();
            //
            FaceInfo fi = _ds.ChangeFaceInfo(nF);
            IndexedSet<PaveBlock> mpbf = fi.PaveBlocksOn();
            //
            NCollection_IntMap mvIn = fi.VerticesIn();
            NCollection_IntMap mvOn = fi.VerticesOn();
            //
            tolE = Tool.Tolerance(e);
            tolF = Tool.Tolerance(f);
            //
            NCollection_List<PaveBlock> lpb = _ds.ChangePaveBlocks(nE);
            it.Initialize(lpb);
            for (; it.More(); it.Next())
            {
                if (UserBreak(psOuter))
                {
                    return;
                }
                PaveBlock pb = it.Value();
                //
                PaveBlock pbr = _ds.RealPaveBlock(pb);
                if (mpbf.Contains(pbr))
                {
                    continue;
                }
                //
                Box bbe = new();
                if (!GetPBBox(e, pb, dmpbBox, ref t1, ref t2, ref ts1, ref ts2, ref bbe))
                {
                    continue;
                }
                //
                if (bbf.IsOut(bbe))
                {
                    continue;
                }
                //
                pbr.Indices(out nV1, out nV2);
                bV1 = mvIn.Contains(nV1) || mvOn.Contains(nV1);
                bV2 = mvIn.Contains(nV2) || mvOn.Contains(nV2);
                bExpressCompute = bV1 && bV2;
                //
                EdgeFaceJob edgeFace =  new EdgeFaceJob();
                vEdgeFace.Add(edgeFace);
                //
                edgeFace.SetIndices(nE, nF);
                edgeFace.SetPaveBlock(pb);
                //
                edgeFace.SetEdge(e);
                edgeFace.SetFace(f);
                edgeFace.SetBoxes(_ds.ShapeInfo(nE).Box(), _ds.ShapeInfo(nF).Box());
                edgeFace.SetFuzzyValue(_fuzzyValue);
                edgeFace.UseQuickCoincidenceCheck(bExpressCompute);

                ParameterRange sr = new(ts1, ts2);
                AlgoTools.CorrectRange(e, f, sr, out ParameterRange anewSR);
                edgeFace.SetNewSR(anewSR);
                //
                ParameterRange pbRange = new(t1, t2);
                sr = pbRange;
                AlgoTools.CorrectRange(e, f, sr, out pbRange);
                edgeFace.SetRange(pbRange);
                //
                // Save the pair to avoid their forced intersection
                if (!_fpbDone.Find(nF, out HashSet<PaveBlock> pMPB))
                {
                    pMPB = new HashSet<PaveBlock>();
                    _fpbDone.Bind(nF, pMPB);
                }
                pMPB.Add(pb);
            } // for (; aIt.More(); aIt.Next()) {
        } // for (; myIterator->More(); myIterator->Next()) {
        //
        nbEdgeFace = vEdgeFace.Count;
        using var ps = new ProgressScope(psOuter.Next(9), "Performing Edge-Face intersection", nbEdgeFace);
        for (int index = 0; index < nbEdgeFace; index++)
        {
            EdgeFaceJob edgeFace = vEdgeFace[index];
            edgeFace.SetProgressRange(ps.Next());
        }
        //=================================================================
        BooleanParallel.Perform(_runParallel, vEdgeFace, _context!);
        //=================================================================
        if (UserBreak(psOuter))
        {
            return;
        }
        //
        for (k = 0; k < nbEdgeFace; ++k)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            EdgeFaceJob edgeFace = vEdgeFace[k];
            if (!edgeFace.IsDone || edgeFace.HasErrors)
            {
                // Warn about failed intersection of sub-shapes
                AddIntersectionFailedWarning(edgeFace.Edge(), edgeFace.Face());
                continue;
            }
            //
            edgeFace.Indices(out nE, out nF);
            //
            Edge e = edgeFace.Edge();
            Face f = edgeFace.Face();
            //
            tolE = Tool.Tolerance(e);
            tolF = Tool.Tolerance(f);
            //
            NCollection_Sequence<CommonPrt> cPrts = edgeFace.CommonParts();
            nbCPrts = cPrts.Length();
            if (nbCPrts == 0)
            {
                // The RealLast() test relies on MinimalDistance() staying RealLast on the paths that don't compute it
                // (decision 756).
                if (edgeFace.MinimalDistance() < RealLast() && edgeFace.MinimalDistance() > tolE + tolF)
                {
                    PaveBlock pb0 = edgeFace.PaveBlock();
                    pb0.Range(out t1, out t2);
                    Pair pair = new(nE, nF);
                    if (!_distances.TryGetValue(pair, out NCollection_List<EdgeRangeDistance>? pList))
                    {
                        pList = new NCollection_List<EdgeRangeDistance>();
                        _distances[pair] = pList;
                    }
                    pList.Append(new EdgeRangeDistance(t1, t2, edgeFace.MinimalDistance()));
                }
                continue;
            }
            //
            ParameterRange anewSR = edgeFace.NewSR();
            PaveBlock pb = edgeFace.PaveBlock();
            //
            pb.Range(out t1, out t2);
            pb.Indices(out nV[0], out nV[1]);
            bIsPBSplittable = pb.IsSplittable;
            //
            anewSR.Range(out ts1, out ts2);
            //
            if (cPrts[1].Type() == ShapeType.Vertex)
            {
                // for the intersection type VERTEX
                // extend vertices ranges using Edge/Edge intersections
                // between the edge aE and the edges of the face aF.
                // thereby the edge's intersection range is reduced
                ReduceIntersectionRange(nV[0], nV[1], nE, nF, ref ts1, ref ts2);
            }
            //
            ParameterRange r1 = new(t1, ts1), r2 = new(ts2, t2);
            //
            FaceInfo fi = _ds.ChangeFaceInfo(nF);
            NCollection_IntMap mifOn = fi.VerticesOn();
            NCollection_IntMap mifIn = fi.VerticesIn();
            //
            bool bLinePlane = false;
            if (nbCPrts != 0)
            {
                EdgeCurveAdaptor bac = new(e);
                bLinePlane = (bac.GetType() == Line && _context!.SurfaceAdaptor(f).GetType() == Plane);
            }
            //
            for (i = 1; i <= nbCPrts; ++i)
            {
                if (UserBreak(psOuter))
                {
                    return;
                }
                CommonPrt cPart = cPrts[i];
                type = cPart.Type();
                switch (type)
                {
                    case ShapeType.Vertex:
                    {
                        int j;
                        double t, tolToDecide;
                        Vertex vnew;
                        //
                        IntersectionTools.VertexParameter(cPart, out t);
                        AlgoTools.MakeNewVertex(e, t, f, out vnew);
                        //
                        ParameterRange r = cPart.Range1();
                        tolToDecide = 5.0e-8;
                        //
                        bIsOnPave[0] = IntersectionTools.IsInRange(r1, r, tolToDecide);
                        bIsOnPave[1] = IntersectionTools.IsInRange(r2, r, tolToDecide);
                        //
                        if ((bIsOnPave[0] && bIsOnPave[1]) || (bLinePlane && (bIsOnPave[0] || bIsOnPave[1])))
                        {
                            bV[0] = CheckFacePaves(nV[0], mifOn, mifIn);
                            bV[1] = CheckFacePaves(nV[1], mifOn, mifIn);
                            if (bV[0] && bV[1])
                            {
                                CommonPrt cp = new(cPart);
                                cp.SetType(ShapeType.Edge);
                                InterfEF ef0 =  new InterfEF();
                                eFs.Add(ef0);
                                iX = eFs.Count - 1;
                                ef0.SetIndices(nE, nF);
                                ef0.SetCommonPart(cp);
                                _ds.AddInterf(nE, nF);
                                //
                                miefc.Add(nF);
                                //
                                BuilderTools.FillMap(pb, nF, mpbli);
                                break;
                            }
                        }
                        //
                        if (!bIsPBSplittable)
                        {
                            continue;
                        }
                        //
                        for (j = 0; j < 2; ++j)
                        {
                            if (bIsOnPave[j])
                            {
                                bV[j] = CheckFacePaves(nV[j], mifOn, mifIn);
                                if (!bV[j])
                                {
                                    bIsOnPave[j] = ForceInterfVF(nV[j], nF);
                                }
                            }
                        }

                        if (bIsOnPave[0] || bIsOnPave[1])
                        {
                            // The found intersection point is located closely to one of the pave block's
                            // bounds. So, do not create the new vertex in this point.
                            // Check if this point is a real intersection, or just a touching point.
                            // If it is a touching point, do nothing.
                            // If it is an intersection point, update the existing vertex to cover the
                            // intersection point.
                            ProjectPointOnSurface projPS = _context!.ProjPS(f);
                            Point3D pnew0 = Tool.Pnt(vnew);
                            projPS.Perform(pnew0);
                            double minDistEF = (projPS.IsDone && projPS.NbPoints() != 0) ? projPS.LowerDistance()
                                                                                             : global::Kerf.Engine.Foundation.Precision.Infinite();
                            bool hasRealIntersection = minDistEF < global::Kerf.Engine.Foundation.Precision.Intersection();

                            if (!hasRealIntersection)
                            {
                                // no intersection point
                                continue;
                            }

                            // Real intersection is present.
                            // Update the existing vertex to cover the intersection point.
                            for (j = 0; j < 2; ++j)
                            {
                                if (bIsOnPave[j])
                                {
                                    Vertex v = ShapeCasts.Vertex(_ds.Shape(nV[j]));
                                    Point3D p = Tool.Pnt(v);
                                    double distPP = p.DistanceTo(pnew0);
                                    double tol = Tool.Tolerance(v);
                                    double maxDist = 1.0e4 * tol;
                                    if (tol < .01)
                                    {
                                        maxDist = Min(maxDist, 0.1);
                                    }
                                    if (distPP < maxDist)
                                    {
                                        UpdateVertex(nV[j], distPP);
                                        _vertsToAvoidExtension.Add(nV[j]);
                                    }
                                }
                            }
                            continue;
                        }

                        if (CheckFacePaves(vnew, mifOn))
                        {
                            continue;
                        }
                        //
                        double tolVnew = Tool.Tolerance(vnew);
                        tolVnew = Max(tolVnew, Max(tolE, tolF));
                        new BrepBuilder().UpdateVertex(vnew, tolVnew);
                        if (bLinePlane)
                        {
                            // increase tolerance for Line/Plane intersection, but do not update
                            // the vertex till its intersection with some other shape
                            ParameterRange cr = cPart.Range1();
                            tolVnew = Max(tolVnew, (cr.Last() - cr.First()) / 2.0);
                        }
                        //
                        Point3D pnew = Tool.Pnt(vnew);
                        //
                        if (!_context!.IsPointInFace(pnew, f, tolVnew))
                        {
                            continue;
                        }
                        //
                        miefc.Add(nF);
                        // 1
                        InterfEF ef =  new InterfEF();
                        eFs.Add(ef);
                        iX = eFs.Count - 1;
                        ef.SetIndices(nE, nF);
                        ef.SetCommonPart(cPart);
                        // 2
                        _ds.AddInterf(nE, nF);
                        // 3
                        CoupleOfPaveBlocks cpb = new();
                        //
                        cpb.SetPaveBlocks(pb, pb);
                        cpb.SetIndexInterf(iX);
                        cpb.SetTolerance(tolVnew);
                        mvcpb.Add(vnew, cpb);
                    }
                    break;
                    case ShapeType.Edge:
                    {
                        miefc.Add(nF);
                        //
                        // 1
                        InterfEF ef =  new InterfEF();
                        eFs.Add(ef);
                        iX = eFs.Count - 1;
                        ef.SetIndices(nE, nF);
                        //
                        bV[0] = CheckFacePaves(nV[0], mifOn, mifIn);
                        bV[1] = CheckFacePaves(nV[1], mifOn, mifIn);
                        if (!bV[0] || !bV[1])
                        {
                            _ds.AddInterf(nE, nF);
                            break;
                        }
                        ef.SetCommonPart(cPart);
                        // 2
                        _ds.AddInterf(nE, nF);
                        // 3
                        BuilderTools.FillMap(pb, nF, mpbli);
                    }
                    break;
                    default:
                        break;
                } // switch (aType) {
            } // for (i=1; i<=aNbCPrts; ++i) {
        } // for (k=0; k < aNbEdgeEdge; ++k) {
        //
        //=========================================
        // post treatment
        //=========================================
        BuilderTools.PerformCommonBlocks(mpbli, _ds, _context);
        UpdateVerticesOfCB();
        PerformNewVertices(mvcpb, psOuter.Next(1), false);
        if (HasErrors)
        {
            return;
        }
        //
        // Update FaceInfoIn for all faces having EF common parts
        _ds.UpdateFaceInfoIn(miefc);

        //-----------------------------------------------------scope t
        miefc.Clear();
        mvcpb.Clear();
        mpbli.Clear();
    }

    /// <summary>Whether the vertex is on or in the face (its index in one of the two maps).</summary>
    protected static bool CheckFacePaves(int nVx, NCollection_IntMap aMIFOn, NCollection_IntMap aMIFIn)
    {
        return aMIFOn.Contains(nVx) || aMIFIn.Contains(nVx);
    }

    /// <summary>Whether the new vertex interferes with one of the vertices of the map.</summary>
    protected bool CheckFacePaves(in Vertex aVnew, NCollection_IntMap aMIF)
    {
        bool bRet;
        int iFlag;
        //
        bRet = true;
        //
        // OCCT iterates the map in its hash order (decision 784); the result is whether any vertex interferes.
        foreach (int nV in aMIF)
        {
            Vertex v = ShapeCasts.Vertex(_ds!.Shape(nV));
            iFlag = AlgoTools.ComputeVV(aVnew, v);
            if (iFlag == 0)
            {
                return bRet;
            }
        }
        //
        return !bRet;
    }

    /// <summary>Forces the vertex/face interference, adding it if the vertex is on the face.</summary>
    protected bool ForceInterfVF(int nV, int nF)
    {
        bool bRet;
        int iFlag, nVx;
        double u = 0.0, V = 0.0, tolVNew = 0.0;
        //
        bRet = false;
        Vertex v = ShapeCasts.Vertex(_ds!.Shape(nV));
        Face f = ShapeCasts.Face(_ds.Shape(nF));
        //
        iFlag = _context!.ComputeVF(v, f, ref u, ref V, ref tolVNew, _fuzzyValue);
        if (iFlag == 0 || iFlag == -2)
        {
            bRet = !bRet;
            //
            List<InterfVF> vFs = _ds.InterfVF();
            // 1
            InterfVF vf =  new InterfVF();
            vFs.Add(vf);
            //
            vf.SetIndices(nV, nF);
            vf.SetUV(u, V);
            // 2
            _ds.AddInterf(nV, nF);
            //
            // 3 update vertex V/F if necessary
            nVx = UpdateVertex(nV, tolVNew);
            // 4
            if (_ds.IsNewShape(nVx))
            {
                vf.SetIndexNew(nVx);
            }
            //
            FaceInfo fi = _ds.ChangeFaceInfo(nF);
            NCollection_IntMap mvIn = fi.ChangeVerticesIn();
            mvIn.Add(nVx);
            //
            // check for self-interference
            int iRV = _ds.Rank(nV);
            if (iRV >= 0 && iRV == _ds.Rank(nF))
            {
                // add warning status
                Compound wc = new();
                new BrepBuilder().MakeCompound(ref wc);
                new BrepBuilder().Add(wc, v);
                new BrepBuilder().Add(wc, f);
                AddWarning(new AlertSelfInterferingShape(wc));
            }
        }
        return bRet;
    }

    /// <summary>
    /// Reduces the intersection range of the edge by the edge/edge intersections of the edge with the face's edges
    /// that made its new vertices.
    /// </summary>
    protected void ReduceIntersectionRange(int v1, int v2, int e, int f, ref double ts1, ref double ts2)
    {
        if (!_ds!.IsNewShape(v1) && !_ds.IsNewShape(v2))
        {
            return;
        }
        //
        if (!_ds.HasInterfShapeSubShapes(e, f))
        {
            return;
        }
        //
        List<InterfEE> eEs = _ds.InterfEE();
        int nbEEs = eEs.Count;
        if (nbEEs == 0)
        {
            return;
        }
        //
        int i, nV, nE1, nE2;
        double tr1, tr2;
        //
        // get face's edges to check that E/E contains the edge from the face
        NCollection_IntMap mfe = new();
        NCollection_List<int> li = _ds.ShapeInfo(f).SubShapes();
        NCollection_List<int>.Iterator itLI = new(li);
        for (; itLI.More(); itLI.Next())
        {
            nE1 = itLI.Value();
            if (_ds.ShapeInfo(nE1).ShapeType() == ShapeType.Edge)
            {
                mfe.Add(nE1);
            }
        }
        //
        for (i = 0; i < nbEEs; ++i)
        {
            InterfEE ee = eEs[i];
            if (!ee.HasIndexNew())
            {
                continue;
            }
            //
            // check the vertex
            nV = ee.IndexNew();
            if (nV != v1 && nV != v2)
            {
                continue;
            }
            //
            // check that the intersection is between the edge
            // and one of the face's edge
            ee.Indices(out nE1, out nE2);
            if (((e != nE1) && (e != nE2)) || (!mfe.Contains(nE1) && !mfe.Contains(nE2)))
            {
                continue;
            }
            //
            // update the intersection range
            CommonPrt cPart = ee.CommonPart();
            ParameterRange cRange = (e == nE1) ? cPart.Range1() : cPart.Ranges2().First();
            cRange.Range(out tr1, out tr2);
            //
            if (nV == v1)
            {
                if (ts1 < tr2)
                {
                    ts1 = tr2;
                }
            }
            else
            {
                if (ts2 > tr1)
                {
                    ts2 = tr1;
                }
            }
        }
    }

    /// <summary>
    /// Looks for the edge/face common blocks among the pairs of pave blocks and faces with the same vertices, after
    /// the vertices have been increased and unified.
    /// </summary>
    protected void ForceInterfEF(ProgressRange range)
    {
        using var ps = new ProgressScope(range, null, 1);
        if (!_isPrimary)
        {
            return;
        }

        // Now that we have vertices increased and unified, try to find additional
        // edge/face common blocks among the pairs of edge/face.
        // Here, we are interested in common blocks only, as all real intersections
        // should have happened already. Thus, we need to check only those pairs
        // of edge/face which have the same vertices.

        // Collect all pave blocks
        IndexedSet<PaveBlock> mpb = new();
        int nbS = _ds!.SourceShapeCount;
        for (int nE = 0; nE < nbS; ++nE)
        {
            ShapeInfo si = _ds.ShapeInfo(nE);
            if (si.ShapeType() != ShapeType.Edge)
            {
                // Not an edge
                continue;
            }

            if (!si.HasReference)
            {
                // Edge has no pave blocks
                continue;
            }

            if (si.HasFlag())
            {
                // Degenerated edge
                continue;
            }

            if (UserBreak(ps))
            {
                return;
            }
            NCollection_List<PaveBlock> lpb = _ds.PaveBlocks(nE);
            NCollection_List<PaveBlock>.Iterator itLPB = new(lpb);
            for (; itLPB.More(); itLPB.Next())
            {
                PaveBlock pb = itLPB.Value();
                PaveBlock pbr = _ds.RealPaveBlock(pb);
                mpb.Add(pbr);
            }
        }

        // Perform intersection of collected pave blocks with faces

        ForceInterfEF(mpb, ps.Next(), true);
    }

    /// <summary>
    /// Intersects the pave blocks with the faces containing both their vertices, looking for edge/face common blocks
    /// only; <paramref name="addInterf"/> adds the interferences and makes the common blocks.
    /// </summary>
    protected void ForceInterfEF(IndexedSet<PaveBlock> mpb, ProgressRange range, bool addInterf)
    {
        // Split progress on preparation, intersection and post-treatment stages
        using var psOuter = new ProgressScope(range, null, 10);
        if (mpb.Count == 0)
        {
            return;
        }
        // Fill the tree with bounding boxes of the pave blocks
        BoxTree bbTree = new();

        IndexedSet<PaveBlock> pbMap = new();

        int nbPB = mpb.Count;
        for (int iPB = 0; iPB < nbPB; ++iPB)
        {
            PaveBlock pb = mpb[iPB];
            if (!pb.HasShrunkData || !_ds!.IsValidShrunkData(pb))
            {
                FillShrunkData(pb);
                if (!pb.HasShrunkData)
                {
                    continue;
                }
            }
            if (UserBreak(psOuter))
            {
                return;
            }

            pb.ShrunkData(out double f, out double l, out Box pbBox, out bool isSplit);

            bbTree.Add(pbMap.Add(pb) + 1, global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(pbBox));
        }

        // Shake the tree
        bbTree.Build();

        bool bSICheckMode = (_arguments.Extent() == 1);

        // Find pairs of Face/PaveBlock containing the same vertices
        // and prepare those pairs for intersection.
        List<EdgeFaceJob> vEdgeFace = new();

        Span<double> ts = stackalloc double[2];
        int nbS = _ds!.SourceShapeCount;
        for (int nF = 0; nF < nbS; ++nF)
        {
            ShapeInfo si = _ds.ShapeInfo(nF);
            if (si.ShapeType() != ShapeType.Face)
            {
                // Not a face
                continue;
            }

            if (!si.HasReference)
            {
                // Face has no face info
                continue;
            }

            if (UserBreak(psOuter))
            {
                return;
            }

            Box boxF = si.Box();
            BoxTreeSelector selector = new();
            selector.SetBox(global::Kerf.Engine.Geometry.Bounds.Tools.Bnd2BVH(boxF));
            selector.SetBVHSet(bbTree);
            if (selector.Select() == 0)
            {
                continue;
            }

            Face aF = ShapeCasts.Face(si.Shape());
            FaceInfo fi = _ds.FaceInfo(nF);
            // Vertices of the face
            NCollection_IntMap mvf = new();
            NCollection_IntMap[] pMVF = [fi.VerticesOn(), fi.VerticesIn(), fi.VerticesSc()];
            for (int iM = 0; iM < 3; ++iM)
            {
                foreach (int vIndex in pMVF[iM])
                {
                    mvf.Add(vIndex);
                }
            }

            // Pave Blocks of the face
            IndexedSet<PaveBlock>[] pMPBF = [fi.PaveBlocksOn(), fi.PaveBlocksIn(), fi.PaveBlocksSc()];
            for (int iM = 0; iM < 3; ++iM)
            {
                int nb = pMPBF[iM].Count;
                for (int iPB = 1; iPB <= nb; ++iPB)
                {
                    PaveBlock pb = pMPBF[iM][iPB - 1];
                    mvf.Add(pb.Pave1().Index());
                    mvf.Add(pb.Pave2().Index());
                }
            }

            // Projection tool
            ProjectPointOnSurface projPS = _context!.ProjPS(aF);
            FaceSurfaceAdaptor surfAdaptor = _context.SurfaceAdaptor(aF);

            // Iterate on pave blocks and combine pairs containing
            // the same vertices
            NCollection_List<int> lipb = selector.Indices();
            NCollection_List<int>.Iterator itLIPB = new(lipb);
            for (; itLIPB.More(); itLIPB.Next())
            {
                PaveBlock pb = pbMap[itLIPB.Value() - 1];
                if (pMPBF[0].Contains(pb) || pMPBF[1].Contains(pb) || pMPBF[2].Contains(pb))
                {
                    continue;
                }

                // Check if the face contains both vertices of the pave block
                pb.Indices(out int nV1, out int nV2);
                if (!mvf.Contains(nV1) || !mvf.Contains(nV2))
                {
                    // Face does not contain the vertices
                    continue;
                }

                // Get the edge
                if (!pb.HasEdge(out int nE))
                {
                    nE = pb.OriginalEdge();
                    if (nE < 0)
                    {
                        continue;
                    }

                    // Make sure that the edge and face came from different arguments
                    if (_ds.Rank(nF) == _ds.Rank(nE))
                    {
                        continue;
                    }
                }

                Edge e = ShapeCasts.Edge(_ds.Shape(nE));
                EdgeCurveAdaptor bac = new(e);

                // Check directions coincidence at middle point on the edge
                // and projection of that point on the face.
                // If the angle between tangent vector to the curve and normal
                // of the face is not in the range of 65 - 115 degrees, do not use the additional
                // tolerance, as it may lead to undesired unification of edge with the face.
                bool bUseAddTol = true;

                pb.ShrunkData(out ts[0], out ts[1], out Box pbBox, out bool isSplit);

                // Middle point
                // Tangent vector in the middle point
                bac.D1(AlgoTools2D.IntermediatePoint(ts[0], ts[1]), out Point3D pOnE, out Vector3D veTgt);
                if (veTgt.SquareMagnitude < Constants.Resolution())
                {
                    continue;
                }

                projPS.Perform(pOnE);
                if (projPS.NbPoints() == 0)
                {
                    continue;
                }

                // Check the distance in the middle point, using the max vertices
                // tolerance as the criteria.
                Vertex v1 = ShapeCasts.Vertex(_ds.Shape(nV1));
                Vertex v2 = ShapeCasts.Vertex(_ds.Shape(nV2));

                // In the Self-Interference check mode we are interested in real
                // intersections only, so use only the real tolerance of edges,
                // no need to use the extended tolerance.
                double tolCheck = (bSICheckMode ? _fuzzyValue
                                                 : 2 * Max(Tool.Tolerance(v1), Tool.Tolerance(v2)));

                if (projPS.LowerDistance() > tolCheck + _fuzzyValue)
                {
                    continue;
                }

                projPS.LowerDistanceParameters(out double u, out double v);
                if (!_context.IsPointInFace(aF, new Point2D(u, v)))
                {
                    continue;
                }

                if (surfAdaptor.GetType() != Plane || bac.GetType() != Line)
                {
                    Point3D pOnS = projPS.NearestPoint();
                    Vector3D vfNorm = new(pOnS, pOnE);
                    if (vfNorm.SquareMagnitude > Constants.Resolution())
                    {
                        // Angle between vectors should be close to 90 degrees.
                        // We allow deviation of 25 degrees.
                        double cos = vfNorm.Normalized().Dot(veTgt.Normalized());
                        if (Math.Abs(cos) > 0.4226)
                        {
                            bUseAddTol = false;
                        }
                    }
                }

                // Compute an addition to Fuzzy value
                double tolAdd = 0.0;
                if (bUseAddTol)
                {
                    // Compute the distance from the bounding points of the edge
                    // to the face and use the maximal of these distances as a
                    // fuzzy tolerance for the intersection.
                    // Use the maximal tolerance of the pave block's vertices
                    // as a max criteria for the computed distance.

                    for (int iP = 0; iP < 2; ++iP)
                    {
                        Point3D p = bac.Value(ts[iP]);
                        projPS.Perform(p);
                        if (projPS.NbPoints() != 0)
                        {
                            double distEF = projPS.LowerDistance();
                            if (distEF < tolCheck && distEF > tolAdd)
                            {
                                tolAdd = distEF;
                            }
                        }
                    }
                    if (tolAdd > 0.0)
                    {
                        tolAdd -= (Tool.Tolerance(e) + Tool.Tolerance(aF));
                        if (tolAdd < 0.0)
                        {
                            tolAdd = 0.0;
                        }
                    }
                }

                bool bIntersect = tolAdd > 0;
                if (!bIntersect)
                {
                    bIntersect = !_fpbDone.Find(nF, out HashSet<PaveBlock> pMPB) || !pMPB.Contains(pb);
                }

                if (bIntersect)
                {
                    // Prepare pair for intersection
                    EdgeFaceJob aEdgeFace =  new EdgeFaceJob();
                    vEdgeFace.Add(aEdgeFace);
                    aEdgeFace.SetIndices(nE, nF);
                    aEdgeFace.SetPaveBlock(pb);
                    aEdgeFace.SetEdge(e);
                    aEdgeFace.SetFace(aF);
                    aEdgeFace.SetBoxes(_ds.ShapeInfo(nE).Box(), _ds.ShapeInfo(nF).Box());
                    aEdgeFace.SetFuzzyValue(_fuzzyValue + tolAdd);
                    aEdgeFace.UseQuickCoincidenceCheck(true);
                    aEdgeFace.SetRange(new ParameterRange(pb.Pave1().Parameter(), pb.Pave2().Parameter()));
                }
            }
        }

        int nbEFs = vEdgeFace.Count;
        if (nbEFs == 0)
        {
            return;
        }

        // close preparation step
        psOuter.Next(0.7);

        pbMap.Clear();

        using var ps = new ProgressScope(psOuter.Next(9), "Checking for edges coinciding with faces", nbEFs);
        for (int i = 0; i < nbEFs; i++)
        {
            EdgeFaceJob aEdgeFace = vEdgeFace[i];
            aEdgeFace.SetProgressRange(ps.Next());
        }
        // Perform intersection of the found pairs
        BooleanParallel.Perform(_runParallel, vEdgeFace, _context!);
        if (UserBreak(psOuter))
        {
            return;
        }

        List<InterfEF> eFs = _ds.InterfEF();

        // Analyze the results of intersection looking for TopAbs_EDGE
        // intersection type only.

        // Collect all pairs for common block creation
        IndexedMap<PaveBlock, NCollection_List<int>> mpbli = new();
        for (int i = 0; i < nbEFs; ++i)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            EdgeFaceJob edgeFace = vEdgeFace[i];
            if (!edgeFace.IsDone || edgeFace.HasErrors)
            {
                // Warn about failed intersection of sub-shapes
                AddIntersectionFailedWarning(edgeFace.Edge(), edgeFace.Face());
                continue;
            }

            NCollection_Sequence<CommonPrt> cParts = edgeFace.CommonParts();
            if (cParts.Length() != 1)
            {
                continue;
            }

            CommonPrt cp = cParts[1];
            if (cp.Type() != ShapeType.Edge)
            {
                continue;
            }

            edgeFace.Indices(out int nE, out int nF);
            if (addInterf)
            {
                // Add interference
                InterfEF ef =  new InterfEF();
                eFs.Add(ef);
                ef.SetIndices(nE, nF);
                ef.SetCommonPart(cp);
                _ds.AddInterf(nE, nF);
            }

            PaveBlock pb = edgeFace.PaveBlock();
            // Update face information with new IN pave block
            _ds.ChangeFaceInfo(nF).ChangePaveBlocksIn().Add(pb);
            if (addInterf)
            {
                // Fill map for common blocks creation
                BuilderTools.FillMap(pb, nF, mpbli);
            }
        }

        if (mpbli.Count != 0)
        {
            // Create new common blocks for coinciding pairs
            BuilderTools.PerformCommonBlocks(mpbli, _ds);
        }
    }
}
