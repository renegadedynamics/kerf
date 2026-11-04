// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_3.cxx
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
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>The edge/edge intersection solver of the pave filler's parallel loops.</summary>
/// <remarks>
/// OCCT derives it from IntTools_EdgeEdge and BOPAlgo_ParallelAlgo. C# has one base class: it derives from
/// IntTools_EdgeEdge, implements <see cref="BooleanParallel.ISolver"/>, and holds what BOPAlgo_ParallelAlgo gives it
/// (the progress range, and a <see cref="Options"/> for the report) as members. <see cref="Perform"/> hides the
/// base's: it moves the edges near the origin when they are far from it (<see cref="BuilderTools.TrsfToPoint(in Box, in Box, ref Transform3D)"/>),
/// intersects them, restores them and sets the restored edges in the common parts.
/// </remarks>
internal sealed class EdgeEdgeJob : EdgeEdge, BooleanParallel.ISolver
{
    private readonly Options _options = new();
    private ProgressRange _progressRange;
    private PaveBlock? _pb1;
    private PaveBlock? _pb2;
    private Box _box1 = new();
    private Box _box2 = new();

    //
    public void SetPaveBlock1(PaveBlock aPB) => _pb1 = aPB;

    //
    public PaveBlock PaveBlock1() => _pb1!;

    //
    public void SetPaveBlock2(PaveBlock aPB) => _pb2 = aPB;

    //
    public PaveBlock PaveBlock2() => _pb2!;

    //
    public void SetBoxes(in Box box1, in Box box2)
    {
        _box1 = box1;
        _box2 = box2;
    }

    // BOPAlgo_ParallelAlgo's members
    public void SetProgressRange(ProgressRange range) => _progressRange = range;

    public bool HasErrors => _options.HasErrors;

    //
    public new void Perform()
    {
        using var ps = new ProgressScope(_progressRange, null, 1);
        if (_options.UserBreak(ps))
        {
            return;
        }
        Edge e1 = _edge1, e2 = _edge2;
        bool hasTrsf = false;
        try
        {
            Transform3D trsf = new();
            if (BuilderTools.TrsfToPoint(_box1, _box2, ref trsf))
            {
                // Shapes are located far from origin, move the shapes to the origin,
                // to increase the accuracy of intersection.
                Placement loc = new(trsf);
                _edge1.Move(loc);
                _edge2.Move(loc);
                hasTrsf = true;
            }

            base.Perform();
        }
        catch (FailureException)
        {
            _options.AddError(new AlertIntersectionFailed());
        }

        _edge1 = e1;
        _edge2 = e2;
        if (hasTrsf)
        {
            for (int i = 1; i <= _commonParts.Length(); ++i)
            {
                CommonPrt cPart = _commonParts[i];
                cPart.SetEdge1(_edge1);
                cPart.SetEdge2(_edge2);
            }
        }
    }
}

internal partial class PaveFiller
{
    /// <summary>The edge/edge interferences: new vertices and common blocks.</summary>
    protected virtual void PerformEE(ProgressRange range)
    {
        FillShrunkData(ShapeType.Edge, ShapeType.Edge);
        //
        _iterator!.Initialize(ShapeType.Edge, ShapeType.Edge);
        int iSize = _iterator.ExpectedLength();
        using var psOuter = new ProgressScope(range, null, 10);
        if (iSize == 0)
        {
            return;
        }
        //
        bool bExpressCompute, bIsPBSplittable1, bIsPBSplittable2;
        int i, iX, nE1, nE2, nbCPrts, k, nbEdgeEdge;
        int nV11, nV12, nV21, nV22;
        // OCCT: uninitialised, written by GetPBBox before they are read
        double ts11 = 0, ts12 = 0, ts21 = 0, ts22 = 0, t11 = 0, t12 = 0, t21 = 0, t22 = 0;
        ShapeType type;
        // The fixed arrays of the TopAbs_VERTEX case (bIsOnPave[4], nV[4], nVS[2])
        Span<bool> bIsOnPave = stackalloc bool[4];
        Span<int> nV = stackalloc int[4];
        Span<int> nVS = stackalloc int[2];
        NCollection_List<PaveBlock>.Iterator it1 = new(), it2 = new();
        List<EdgeEdgeJob> vEdgeEdge = new();
        // keep modified edges for further update
        NCollection_IntMap mEdges = new();
        //
        //-----------------------------------------------------scope f
        IndexedMap<PaveBlock, NCollection_List<PaveBlock>> mpblpb = new();
        IndexedMap<Shape, CoupleOfPaveBlocks> mvcpb = new(ShapeHasher.Instance);
        Dictionary<PaveBlock, Box> dmpbBox = new();
        //
        List<InterfEE> eEs = _ds!.InterfEE();
        //
        for (; _iterator.More(); _iterator.Next())
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            _iterator.Value(out nE1, out nE2);
            //
            ShapeInfo sie1 = _ds.ShapeInfo(nE1);
            if (sie1.HasFlag())
            {
                continue;
            }
            ShapeInfo sie2 = _ds.ShapeInfo(nE2);
            if (sie2.HasFlag())
            {
                continue;
            }
            //
            NCollection_List<PaveBlock> lpb1 = _ds.ChangePaveBlocks(nE1);
            if (lpb1.IsEmpty())
            {
                continue;
            }
            //
            NCollection_List<PaveBlock> lpb2 = _ds.ChangePaveBlocks(nE2);
            if (lpb2.IsEmpty())
            {
                continue;
            }
            //
            Edge e1 = ShapeCasts.Edge(sie1.Shape());
            Edge e2 = ShapeCasts.Edge(sie2.Shape());
            //
            it1.Initialize(lpb1);
            for (; it1.More(); it1.Next())
            {
                if (UserBreak(psOuter))
                {
                    return;
                }
                Box bb1 = new();
                //
                PaveBlock pb1 = it1.Value();
                //
                if (!GetPBBox(e1, pb1, dmpbBox, ref t11, ref t12, ref ts11, ref ts12, ref bb1))
                {
                    continue;
                }
                //
                pb1.Indices(out nV11, out nV12);
                //
                it2.Initialize(lpb2);
                for (; it2.More(); it2.Next())
                {
                    Box bb2 = new();
                    //
                    PaveBlock pb2 = it2.Value();
                    //
                    if (!GetPBBox(e2, pb2, dmpbBox, ref t21, ref t22, ref ts21, ref ts22, ref bb2))
                    {
                        continue;
                    }
                    //
                    if (bb1.IsOut(bb2))
                    {
                        continue;
                    }
                    //
                    pb2.Indices(out nV21, out nV22);
                    //
                    bExpressCompute = ((nV11 == nV21 && nV12 == nV22) || (nV12 == nV21 && nV11 == nV22));
                    //
                    EdgeEdgeJob edgeEdge =  new EdgeEdgeJob();
                    vEdgeEdge.Add(edgeEdge);
                    //
                    edgeEdge.UseQuickCoincidenceCheck(bExpressCompute);
                    //
                    edgeEdge.SetPaveBlock1(pb1);
                    edgeEdge.SetPaveBlock2(pb2);
                    //
                    edgeEdge.SetEdge1(e1, t11, t12);
                    edgeEdge.SetEdge2(e2, t21, t22);
                    edgeEdge.SetBoxes(bb1, bb2);
                    edgeEdge.SetFuzzyValue(_fuzzyValue);
                } // for (; aIt2.More(); aIt2.Next()) {
            } // for (; aIt1.More(); aIt1.Next()) {
        } // for (; myIterator->More(); myIterator->Next()) {
        //
        nbEdgeEdge = vEdgeEdge.Count;

        using (var ps = new ProgressScope(psOuter.Next(9), "Performing Edge-edge intersection", nbEdgeEdge))
        {
            for (k = 0; k < nbEdgeEdge; k++)
            {
                EdgeEdgeJob edgeEdge = vEdgeEdge[k];
                edgeEdge.SetProgressRange(ps.Next());
            }
            //======================================================
            BooleanParallel.Perform(_runParallel, vEdgeEdge);
            //======================================================
        }
        if (UserBreak(psOuter))
        {
            return;
        }
        //
        for (k = 0; k < nbEdgeEdge; ++k)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            Box bb1 = new(), bb2 = new();
            //
            EdgeEdgeJob edgeEdge = vEdgeEdge[k];
            if (!edgeEdge.IsDone || edgeEdge.HasErrors)
            {
                // Warn about failed intersection of sub-shapes
                Shape e1f = _ds.Shape(edgeEdge.PaveBlock1().OriginalEdge());
                Shape e2f = _ds.Shape(edgeEdge.PaveBlock2().OriginalEdge());
                AddIntersectionFailedWarning(e1f, e2f);
                continue;
            }
            //
            NCollection_Sequence<CommonPrt> cPrts = edgeEdge.CommonParts();
            nbCPrts = cPrts.Length();
            if (nbCPrts == 0)
            {
                continue;
            }
            //--------------------------------------------
            PaveBlock pb1 = edgeEdge.PaveBlock1();
            nE1 = pb1.OriginalEdge();
            pb1.Range(out t11, out t12);
            if (!pb1.HasShrunkData)
            {
                ts11 = t11;
                ts12 = t12;
                bIsPBSplittable1 = false;
            }
            else
            {
                pb1.ShrunkData(out ts11, out ts12, out bb1, out bIsPBSplittable1);
            }
            //
            PaveBlock pb2 = edgeEdge.PaveBlock2();
            nE2 = pb2.OriginalEdge();
            pb2.Range(out t21, out t22);
            if (!pb2.HasShrunkData)
            {
                ts21 = t21;
                ts22 = t22;
                bIsPBSplittable2 = false;
            }
            else
            {
                pb2.ShrunkData(out ts21, out ts22, out bb2, out bIsPBSplittable2);
            }
            //
            //--------------------------------------------
            ParameterRange r11 = new(t11, ts11), r12 = new(ts12, t12), r21 = new(t21, ts21), r22 = new(ts22, t22);
            //
            bool bAnalytical = false;
            {
                Edge oe1 = ShapeCasts.Edge(_ds.Shape(nE1));
                Edge oe2 = ShapeCasts.Edge(_ds.Shape(nE2));
                //
                EdgeCurveAdaptor bac1 = new(oe1), bac2 = new(oe2);
                //
                CurveKind type1 = bac1.GetType();
                CurveKind type2 = bac2.GetType();
                //
                bAnalytical = (type1 == Line && type2 == Circle)
                              || (type1 == Circle && type2 == Line);
            }
            //
            for (i = 1; i <= nbCPrts; ++i)
            {
                if (UserBreak(psOuter))
                {
                    return;
                }
                CommonPrt cPart = cPrts[i];
                //
                Edge e1 = cPart.Edge1();
                Edge e2 = cPart.Edge2();
                //
                type = cPart.Type();
                switch (type)
                {
                    case ShapeType.Vertex:
                    {
                        if (!bIsPBSplittable1 || !bIsPBSplittable2)
                        {
                            continue;
                        }
                        //
                        int j;
                        double t1, t2, tol;
                        Vertex vnew;
                        ParameterRange cr1, cr2;
                        //
                        IntersectionTools.VertexParameters(cPart, out t1, out t2);
                        tol = global::Kerf.Engine.Foundation.Precision.Confusion();
                        cr1 = cPart.Range1();
                        cr2 = cPart.Ranges2().Value(1);
                        //
                        // decide to keep the pave or not
                        bIsOnPave[0] = IntersectionTools.IsOnPave1(t1, r11, tol)
                                       || IntersectionTools.IsOnPave1(r11.First(), cr1, tol);
                        bIsOnPave[1] = IntersectionTools.IsOnPave1(t1, r12, tol)
                                       || IntersectionTools.IsOnPave1(r12.Last(), cr1, tol);
                        bIsOnPave[2] = IntersectionTools.IsOnPave1(t2, r21, tol)
                                       || IntersectionTools.IsOnPave1(r21.First(), cr2, tol);
                        bIsOnPave[3] = IntersectionTools.IsOnPave1(t2, r22, tol)
                                       || IntersectionTools.IsOnPave1(r22.Last(), cr2, tol);
                        //
                        pb1.Indices(out nV[0], out nV[1]);
                        pb2.Indices(out nV[2], out nV[3]);
                        //
                        if ((bIsOnPave[0] && bIsOnPave[2]) || (bIsOnPave[0] && bIsOnPave[3])
                            || (bIsOnPave[1] && bIsOnPave[2]) || (bIsOnPave[1] && bIsOnPave[3]))
                        {
                            continue;
                        }
                        //
                        bool isVExists = false;
                        for (j = 0; j < 4; ++j)
                        {
                            if (bIsOnPave[j])
                            {
                                PaveBlock pb = (j < 2) ? pb2 : pb1;
                                bIsOnPave[j] = ForceInterfVE(nV[j], pb, mEdges);
                                if (bIsOnPave[j])
                                {
                                    isVExists = true;
                                }
                            }
                        }

                        AlgoTools.MakeNewVertex(e1, t1, e2, t2, out vnew);
                        Point3D pnew = Tool.Pnt(vnew);

                        if (isVExists)
                        {
                            // The found intersection point is located closely to one of the
                            // pave blocks bounds. So, do not create the new vertex in this point.
                            // Check if this point is a real intersection point or just a touching point.
                            // If it is a touching point, do nothing.
                            // If it is an intersection point, update the existing vertex to cover the
                            // intersection point.
                            Point3D pOnE1 = new EdgeCurveAdaptor(e1).Value(t1);
                            Point3D pOnE2 = new EdgeCurveAdaptor(e2).Value(t2);
                            if (pOnE1.DistanceTo(pOnE2) > global::Kerf.Engine.Foundation.Precision.Intersection())
                            {
                                // No intersection point
                                continue;
                            }

                            // Real intersection is present.
                            // Update the existing vertex to cover the intersection point.
                            for (j = 0; j < 4; ++j)
                            {
                                if (bIsOnPave[j])
                                {
                                    Vertex v = ShapeCasts.Vertex(_ds.Shape(nV[j]));
                                    Point3D p = Tool.Pnt(v);
                                    double distPP = p.DistanceTo(pnew);
                                    // Just update the vertex
                                    UpdateVertex(nV[j], distPP);
                                    _vertsToAvoidExtension.Add(nV[j]);
                                }
                            }
                        }

                        double tolVnew = Tool.Tolerance(vnew);
                        if (bAnalytical)
                        {
                            // increase tolerance for Line/Circle intersection, but do not update
                            // the vertex till its intersection with some other shape
                            double tolMin = (new EdgeCurveAdaptor(e1).GetType() == Line)
                                ? (cr1.Last() - cr1.First()) / 2.0
                                : (cr2.Last() - cr2.First()) / 2.0;
                            if (tolMin > tolVnew)
                            {
                                tolVnew = tolMin;
                            }
                        }
                        // <-LXBR
                        {
                            int iFound;
                            double tolVx, d2, dt2;
                            NCollection_IntMap mv = new();
                            Point3D px;
                            //
                            iFound = 0;
                            j = -1;
                            mv.Add(nV[0]);
                            mv.Add(nV[1]);
                            //
                            if (mv.Contains(nV[2]))
                            {
                                ++j;
                                nVS[j] = nV[2];
                            }
                            if (mv.Contains(nV[3]))
                            {
                                ++j;
                                nVS[j] = nV[3];
                            }
                            //
                            for (int k1 = 0; k1 <= j; ++k1)
                            {
                                Vertex vx = ShapeCasts.Vertex(_ds.Shape(nVS[k1]));
                                tolVx = Tool.Tolerance(vx);
                                px = Tool.Pnt(vx);
                                d2 = pnew.SquareDistanceTo(px);
                                //
                                dt2 = 100.0 * (tolVnew + tolVx) * (tolVnew + tolVx);
                                //
                                if (d2 < dt2)
                                {
                                    iFound = 1;
                                    break;
                                }
                            }
                            //
                            if (iFound != 0)
                            {
                                continue;
                            }
                        }
                        //
                        // 1
                        InterfEE ee =  new InterfEE();
                        eEs.Add(ee);
                        iX = eEs.Count - 1;
                        ee.SetIndices(nE1, nE2);
                        ee.SetCommonPart(cPart);
                        // 2
                        _ds.AddInterf(nE1, nE2);
                        //
                        CoupleOfPaveBlocks cpb = new();
                        //
                        cpb.SetPaveBlocks(pb1, pb2);
                        cpb.SetIndexInterf(iX);
                        cpb.SetTolerance(tolVnew);
                        mvcpb.Add(vnew, cpb);
                    } // case TopAbs_VERTEX:
                        break;
                    //
                    case ShapeType.Edge:
                    {
                        if (nbCPrts > 1)
                        {
                            break;
                        }
                        //
                        bool bHasSameBounds;
                        bHasSameBounds = pb1.HasSameBounds(pb2);
                        if (!bHasSameBounds)
                        {
                            break;
                        }
                        // 1
                        InterfEE ee =  new InterfEE();
                        eEs.Add(ee);
                        iX = eEs.Count - 1;
                        ee.SetIndices(nE1, nE2);
                        ee.SetCommonPart(cPart);
                        // 2
                        _ds.AddInterf(nE1, nE2);
                        //
                        BuilderTools.FillMap(pb1, pb2, mpblpb);
                    } // case TopAbs_EDGE
                        break;
                    default:
                        break;
                } // switch (aType) {
            } // for (i=1; i<=aNbCPrts; i++) {
        } // for (k=0; k < aNbFdgeEdge; ++k) {
        //
        //=========================================
        // post treatment
        //=========================================
        BuilderTools.PerformCommonBlocks(mpblpb, _ds, _context);
        // Update vertices of common blocks with real CB tolerances
        UpdateVerticesOfCB();

        PerformNewVertices(mvcpb, psOuter.Next());
        if (HasErrors)
        {
            return;
        }
        //
        if (mEdges.Extent() != 0)
        {
            int nbV = mvcpb.Count;
            for (i = 1; i <= nbV; ++i)
            {
                CoupleOfPaveBlocks cpb = mvcpb.ValueAt(i - 1);
                PaveBlock pb1 = cpb.PaveBlock1()!;
                PaveBlock pb2 = cpb.PaveBlock2()!;
                //
                mEdges.Remove(pb1.OriginalEdge());
                mEdges.Remove(pb2.OriginalEdge());
            }
            //
            SplitPaveBlocks(mEdges, false);
        }
        //
        //-----------------------------------------------------scope t
        mpblpb.Clear();
        mvcpb.Clear();
    }

    /// <summary>
    /// Fuses the new vertices of the edge/edge (or edge/face) interferences, adds them to the DS and splits the pave
    /// blocks by them.
    /// </summary>
    protected void PerformNewVertices(IndexedMap<Shape, CoupleOfPaveBlocks> mvcpb,
                                      ProgressRange range, bool isEEIntersection = true)
    {
        int nbV = mvcpb.Count;
        if (nbV == 0)
        {
            return;
        }
        //
        double tolAdd = _fuzzyValue / 2.0;
        //
        // 1. Fuse the new vertices
        IndexedMap<Shape, NCollection_List<Shape>> images = new(ShapeHasher.Instance);
        TreatNewVertices(mvcpb, images);
        //
        // 2. Add new vertices to myDS and connect indices to CPB structure
        List<InterfEE> eEs = _ds!.InterfEE();
        List<InterfEF> eFs = _ds.InterfEF();
        //
        // 4. Compute Extra Paves and split Pave blocks by the Extra paves
        using var ps = new ProgressScope(range, null, 2);
        int i, nb = images.Count;
        using (var ps1 = new ProgressScope(ps.Next(), null, nb + nbV))
        {
            for (i = 1; i <= nb; ++i, ps1.Next())
            {
                if (UserBreak(ps))
                {
                    return;
                }
                Vertex v = ShapeCasts.Vertex(images.KeyAt(i - 1));
                NCollection_List<Shape> lvsd = images.ValueAt(i - 1);
                //
                ShapeInfo si = new();
                si.SetShapeType(ShapeType.Vertex);
                si.SetShape(v);
                int iV = _ds.Append(si);
                //
                ShapeInfo sids = _ds.ChangeShapeInfo(iV);
                ref Box box = ref sids.ChangeBox();
                box.Add(Tool.Pnt(v));
                box.SetGap(Tool.Tolerance(v) + tolAdd);
                //
                NCollection_List<Shape>.Iterator itLS = new(lvsd);
                for (; itLS.More(); itLS.Next())
                {
                    Shape vx = itLS.Value();
                    ref CoupleOfPaveBlocks cpb = ref mvcpb.GetValueRef(vx);
                    cpb.SetIndex(iV);
                    // update interference
                    int iX = cpb.IndexInterf();
                    Interf aInt = isEEIntersection ? eEs[iX] : eFs[iX];
                    aInt.SetIndexNew(iV);
                }
            }
            //
            // 3. Map PaveBlock/ListOfVertices to add to this PaveBlock ->aMPBLI
            IndexedMap<PaveBlock, NCollection_List<int>> mpbli = new();
            for (i = 1; i <= nbV; ++i, ps1.Next())
            {
                if (UserBreak(ps))
                {
                    return;
                }
                CoupleOfPaveBlocks cpb = mvcpb.ValueAt(i - 1);
                int iV = cpb.Index();
                //
                PaveBlock pb1 = cpb.PaveBlock1()!;
                PaveBlock pb2 = cpb.PaveBlock2()!;
                PaveBlock[] pb = [pb1, pb2];
                for (int j = 0; j < 2; ++j)
                {
                    int index = mpbli.IndexOf(pb[j]) + 1;
                    NCollection_List<int> pLI = index > 0 ? mpbli.ValueAt(index - 1) : mpbli.ValueAt(mpbli.Add(pb[j], new NCollection_List<int>()));
                    pLI.Append(iV);
                    //
                    if (pb[0] == pb[1])
                    {
                        break;
                    }
                }
            }
            // 4. Compute Extra Paves and split Pave blocks by the Extra paves
            IntersectVE(mpbli, ps.Next(), false);
        }
    }

    /// <summary>Fuses the coinciding new vertices: each chain of intersecting vertices becomes one vertex.</summary>
    protected void TreatNewVertices(IndexedMap<Shape, CoupleOfPaveBlocks> mvcpb,
                                    IndexedMap<Shape, NCollection_List<Shape>> images)
    {
        //
        // Prepare for intersection
        IndexedMap<Shape, double> verts = new(ShapeHasher.Instance);
        int i, nbV = mvcpb.Count;
        for (i = 1; i <= nbV; ++i)
        {
            Shape v = mvcpb.KeyAt(i - 1);
            double tol = mvcpb.ValueAt(i - 1).Tolerance();
            verts.Add(v, tol);
        }
        //
        // Perform intersection
        NCollection_List<NCollection_List<Shape>> chains = new();
        BuilderTools.IntersectVertices(verts, _fuzzyValue, chains);
        //
        // Treat the results - make new vertices for each chain
        NCollection_List<NCollection_List<Shape>>.Iterator itC = new(chains);
        for (; itC.More(); itC.Next())
        {
            NCollection_List<Shape> lvsd = itC.Value();
            //
            Vertex vNew = new();
            AlgoTools.MakeVertex(lvsd, ref vNew);
            images.Add(vNew, lvsd);
        }
    }

    /// <summary>Computes the shrunk data (valid range and box) of the pave block.</summary>
    protected void FillShrunkData(PaveBlock pb)
    {
        // Vertices
        pb.Indices(out int nV1, out int nV2);

        if (nV1 < 0 || nV2 < 0)
        {
            return;
        }

        Vertex v1 = ShapeCasts.Vertex(_ds!.Shape(nV1));
        Vertex v2 = ShapeCasts.Vertex(_ds.Shape(nV2));
        // Get the edge
        if (!pb.HasEdge(out int nE))
        {
            nE = pb.OriginalEdge();
            if (nE < 0)
            {
                return;
            }
        }

        Edge e = ShapeCasts.Edge(_ds.Shape(nE));
        // Range
        pb.Range(out double t1, out double t2);
        //
        EdgeShrunkRange sr = new();
        sr.SetContext(_context);
        sr.SetData(e, t1, t2, v1, v2);
        sr.Perform();
        // Analyze the results of computations
        AnalyzeShrunkData(pb, sr);
    }

    /// <summary>Sets the shrunk data of the pave block from the computation, with warnings where it failed.</summary>
    protected void AnalyzeShrunkData(PaveBlock pb, EdgeShrunkRange sr)
    {
        // in case of error treat the warning status
        bool bWholeEdge = false;
        Shape warnShape = new();
        //
        if (!sr.IsDone || !sr.IsSplittable)
        {
            Tool.Range(sr.Edge(), out double eFirst, out double eLast);
            pb.Range(out double pbFirst, out double pbLast);
            bWholeEdge = pbFirst <= eFirst && pbLast >= eLast;
            if (bWholeEdge && pb.OriginalEdge() >= 0)
            {
                warnShape = sr.Edge();
            }
            else
            {
                Shape v1 = _ds!.Shape(pb.Pave1().Index());
                Shape v2 = _ds.Shape(pb.Pave2().Index());
                Compound wc = new();
                new BrepBuilder().MakeCompound(ref wc);
                warnShape = wc;
                new BrepBuilder().Add(warnShape, sr.Edge());
                new BrepBuilder().Add(warnShape, v1);
                new BrepBuilder().Add(warnShape, v2);
            }
            //
            if (!sr.IsDone)
            {
                if (bWholeEdge)
                {
                    AddWarning(new AlertTooSmallEdge(warnShape));
                }
                else
                {
                    AddWarning(new AlertBadPositioning(warnShape));
                }
                sr.ShrunkRange(out double ts1f, out double ts2f);
                pb.SetShrunkData(ts1f, ts2f, new Box(), false);
                return;
            }
            //
            if (bWholeEdge)
            {
                AddWarning(new AlertNotSplittableEdge(warnShape));
            }
            else
            {
                AddWarning(new AlertBadPositioning(warnShape));
            }
        }
        //
        sr.ShrunkRange(out double ts1, out double ts2);
        Box box = sr.BndBox();
        box.SetGap(box.GetGap() + _fuzzyValue / 2.0);
        pb.SetShrunkData(ts1, ts2, box, sr.IsSplittable);
    }

    /// <summary>
    /// Forces the vertex/edge interference of the vertex with the pave block's edge: on success adds the interference
    /// and the extra pave, and the edge to <paramref name="mEdges"/>.
    /// </summary>
    protected bool ForceInterfVE(int nV, PaveBlock aPB, NCollection_IntMap mEdges)
    {
        int nE, nVx, nVSD = 0, iFlag;
        double t = 0.0, tolVNew = 0.0; // OCCT: uninitialised, read only on success
        //
        nE = aPB.OriginalEdge();
        //
        ShapeInfo sie = _ds!.ShapeInfo(nE);
        if (sie.HasSubShape(nV))
        {
            return true;
        }
        //
        if (_ds.HasInterf(nV, nE))
        {
            return true;
        }
        //
        if (_ds.HasInterfShapeSubShapes(nV, nE))
        {
            return true;
        }
        //
        if (aPB.Pave1().Index() == nV || aPB.Pave2().Index() == nV)
        {
            return true;
        }
        //
        nVx = nV;
        if (_ds.HasShapeSD(nV, ref nVSD))
        {
            nVx = nVSD;
        }
        //
        Vertex v = ShapeCasts.Vertex(_ds.Shape(nVx));
        Edge e = ShapeCasts.Edge(_ds.Shape(nE));
        //
        iFlag = _context!.ComputeVE(v, e, ref t, ref tolVNew, _fuzzyValue);
        if (iFlag == 0 || iFlag == -4)
        {
            Pave pave = new();
            //
            //
            List<InterfVE> vEs = _ds.InterfVE();
            // 1
            InterfVE ve =  new InterfVE();
            vEs.Add(ve);
            ve.SetIndices(nV, nE);
            ve.SetParameter(t);
            // 2
            _ds.AddInterf(nV, nE);
            //
            // 3 update vertex V/E if necessary
            nVx = UpdateVertex(nV, tolVNew);
            // 4
            if (_ds.IsNewShape(nVx))
            {
                ve.SetIndexNew(nVx);
            }
            // 5 append ext pave to pave block
            pave.SetIndex(nVx);
            pave.SetParameter(t);
            aPB.AppendExtPave(pave);
            //
            mEdges.Add(nE);
            //
            // check for self-interference
            int iRV = _ds.Rank(nV);
            if (iRV >= 0 && iRV == _ds.Rank(nE))
            {
                // add warning status
                Compound wc = new();
                new BrepBuilder().MakeCompound(ref wc);
                new BrepBuilder().Add(wc, v);
                new BrepBuilder().Add(wc, e);
                AddWarning(new AlertSelfInterferingShape(wc));
            }
            return true;
        }
        return false;
    }

    /// <summary>
    /// The range, shrunk range and box of the pave block (its shrunk box, or a box of its range kept in
    /// <paramref name="pbBox"/>). False if the range is too small.
    /// </summary>
    protected bool GetPBBox(in Edge e, PaveBlock pb, Dictionary<PaveBlock, Box> pbBox,
                            ref double first, ref double last, ref double sFirst, ref double sLast, ref Box box)
    {
        pb.Range(out first, out last);
        // check the validity of PB's range
        bool bValid = last - first > global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (!bValid)
        {
            return bValid;
        }
        //
        // check shrunk data
        if (pb.HasShrunkData)
        {
            pb.ShrunkData(out sFirst, out sLast, out box, out bool bIsSplittable);
            return bValid;
        }
        //
        sFirst = first;
        sLast = last;
        // check the map
        if (pbBox.ContainsKey(pb))
        {
            box = pbBox.GetValue(pb);
        }
        else
        {
            // build bounding box
            EdgeCurveAdaptor bac = new(e);
            double tol = Tool.Tolerance(e) + global::Kerf.Engine.Foundation.Precision.Confusion();
            Add3dCurve.Add(bac, sFirst, sLast, tol, ref box);
            pbBox[pb] = box;
        }
        return bValid;
    }

    /// <summary>Increases the tolerances of the common blocks' vertices to the blocks' tolerances.</summary>
    protected void UpdateVerticesOfCB()
    {
        // Fence map to avoid checking same Common block twice
        HashSet<PaveBlock> mpbFence = new();

        List<NCollection_List<PaveBlock>> pbp = _ds!.ChangePaveBlocksPool();
        int nbPBP = pbp.Count;
        for (int i = 0; i < nbPBP; ++i)
        {
            NCollection_List<PaveBlock> lpb = pbp[i];
            NCollection_List<PaveBlock>.Iterator itPB = new(lpb);
            for (; itPB.More(); itPB.Next())
            {
                CommonBlock? cb = _ds.CommonBlock(itPB.Value());
                if (cb is null)
                {
                    continue;
                }

                PaveBlock pbr = cb.PaveBlock1();
                if (!mpbFence.Add(pbr))
                {
                    continue;
                }

                double tolCB = cb.Tolerance();
                if (tolCB > 0.0)
                {
                    UpdateVertex(pbr.Pave1().Index(), tolCB);
                    UpdateVertex(pbr.Pave2().Index(), tolCB);
                }
            }
        }
    }

    /// <summary>
    /// Looks for more common blocks among the pave blocks with the same bounding vertices, now that the vertices have
    /// been increased and unified.
    /// </summary>
    protected void ForceInterfEE(ProgressRange range)
    {
        // Now that we have vertices increased and unified, try to find additional
        // common blocks among the pairs of edges.
        // Since all real intersections should have already happened, here we
        // are interested in common blocks only, thus we need to check only
        // those pairs of pave blocks with the same bounding vertices.

        using var psOuter = new ProgressScope(range, null, 10);
        // Initialize pave blocks for all vertices which participated in intersections
        int nbS = _ds!.SourceShapeCount;
        for (int i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
            if (si.ShapeType() == ShapeType.Vertex)
            {
                if (_ds.HasInterf(i))
                {
                    _ds.InitPaveBlocksForVertex(i);
                }
            }
            if (UserBreak(psOuter))
            {
                return;
            }
        }
        // Fill the connection map from bounding vertices to pave blocks
        // having those bounding vertices
        IndexedMap<Pair, NCollection_List<PaveBlock>> pbMap = new();
        // Fence map of pave blocks
        HashSet<PaveBlock> mpbFence = new();

        for (int i = 0; i < nbS; ++i)
        {
            ShapeInfo si = _ds.ShapeInfo(i);
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
            if (UserBreak(psOuter))
            {
                return;
            }
            NCollection_List<PaveBlock> lpb = _ds.PaveBlocks(i);
            NCollection_List<PaveBlock>.Iterator itLPB = new(lpb);
            for (; itLPB.More(); itLPB.Next())
            {
                PaveBlock pb = itLPB.Value();
                PaveBlock pbr = _ds.RealPaveBlock(pb);
                if (!mpbFence.Add(pbr))
                {
                    continue;
                }

                // Get indices
                pbr.Indices(out int nV1, out int nV2);

                // Add pave block to a map
                Pair pair = new(nV1, nV2);
                int index = pbMap.IndexOf(pair) + 1;
                NCollection_List<PaveBlock> pList = index > 0
                    ? pbMap.ValueAt(index - 1)
                    : pbMap.ValueAt(pbMap.Add(pair, new NCollection_List<PaveBlock>()));
                pList.Append(pbr);
            }
        }

        int nbPB = pbMap.Count;
        if (nbPB == 0)
        {
            return;
        }

        bool bSICheckMode = (_arguments.Extent() == 1);

        // Prepare pave blocks with the same vertices for intersection.
        List<EdgeEdgeJob> vEdgeEdge = new();

        for (int i = 0; i < nbPB; ++i)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            NCollection_List<PaveBlock> lpb = pbMap.ValueAt(i);
            if (lpb.Extent() < 2)
            {
                continue;
            }

            Pair pair = pbMap.KeyAt(i);
            pair.Indices(out int nV1, out int nV2);

            Vertex v1 = ShapeCasts.Vertex(_ds.Shape(nV1));
            Vertex v2 = ShapeCasts.Vertex(_ds.Shape(nV2));

            // Use the max tolerance of vertices as Fuzzy value for intersection of edges.
            // In the Self-Interference check mode we are interested in real
            // intersections only, so use only the real tolerance of edges,
            // no need to use the extended tolerance.
            double tolAdd = (bSICheckMode ? _fuzzyValue
                                           : 2 * Max(Tool.Tolerance(v1), Tool.Tolerance(v2)));

            // All possible pairs combined from the list <aLPB> should be checked
            NCollection_List<PaveBlock>.Iterator itLPB1 = new(lpb);
            for (; itLPB1.More(); itLPB1.Next())
            {
                PaveBlock pb1 = itLPB1.Value();
                CommonBlock? cb1 = _ds.CommonBlock(pb1);
                int nE1 = pb1.OriginalEdge();
                int iR1 = _ds.Rank(nE1);
                Edge e1 = ShapeCasts.Edge(_ds.Shape(nE1));
                pb1.Range(out double t11, out double t12);
                EdgeCurveAdaptor bac1 = new(e1);
                bac1.D1((t11 + t12) * 0.5, out Point3D pm, out Vector3D vTgt1);
                if (vTgt1.SquareMagnitude < Constants.Resolution())
                {
                    continue;
                }
                vTgt1 = vTgt1.Normalized();

                NCollection_List<PaveBlock>.Iterator itLPB2 = new();
                itLPB2.Reset(lpb, itLPB1.Index);
                for (itLPB2.Next(); itLPB2.More(); itLPB2.Next())
                {
                    PaveBlock pb2 = itLPB2.Value();
                    CommonBlock? cb2 = _ds.CommonBlock(pb2);
                    int nE2 = pb2.OriginalEdge();
                    int iR2 = _ds.Rank(nE2);

                    // Check that the edges came from different arguments
                    if (iR1 == iR2)
                    {
                        // If the sharing of the vertices is not original, but has been acquired
                        // during the operation, check the coincidence of the edges even if
                        // they came from the same argument
                        if ((!_ds.IsNewShape(nV1) && (_ds.Rank(nV1) == iR1))
                            || (!_ds.IsNewShape(nV2) && (_ds.Rank(nV2) == iR2)))
                        {
                            continue;
                        }
                    }

                    // Check that the Pave blocks do not form the Common block already
                    if (cb1 is not null && cb2 is not null)
                    {
                        if (cb1 == cb2)
                        {
                            continue;
                        }
                    }

                    Edge e2 = ShapeCasts.Edge(_ds.Shape(nE2));
                    pb2.Range(out double t21, out double t22);

                    // Check the angle between edges in the middle point.
                    // If the angle is more than 25 degrees, do not use the additional
                    // tolerance, as it may lead to undesired unification of edges
                    bool bUseAddTol = true;
                    {
                        EdgeCurveAdaptor bac2 = new(e2);
                        if (bac1.GetType() != Line || bac2.GetType() != Line)
                        {
                            ProjectPointOnCurve projPC = _context!.ProjPC(e2);
                            projPC.Perform(pm);
                            if (projPC.NbPoints() == 0)
                            {
                                continue;
                            }

                            bac2.D1(projPC.LowerDistanceParameter(), out Point3D pm2, out Vector3D vTgt2);
                            if (vTgt2.SquareMagnitude < Constants.Resolution())
                            {
                                continue;
                            }

                            // The angle should be close to zero
                            double cos = vTgt1.Dot(vTgt2.Normalized());
                            if (Math.Abs(cos) < 0.9063)
                            {
                                bUseAddTol = false;
                            }
                        }
                    }

                    // Add pair for intersection
                    EdgeEdgeJob edgeEdge =  new EdgeEdgeJob();
                    vEdgeEdge.Add(edgeEdge);
                    edgeEdge.UseQuickCoincidenceCheck(true);
                    edgeEdge.SetPaveBlock1(pb1);
                    edgeEdge.SetPaveBlock2(pb2);
                    edgeEdge.SetEdge1(e1, t11, t12);
                    edgeEdge.SetEdge2(e2, t21, t22);
                    edgeEdge.SetBoxes(_ds.ShapeInfo(nE1).Box(), _ds.ShapeInfo(nE2).Box());
                    if (bUseAddTol)
                    {
                        edgeEdge.SetFuzzyValue(_fuzzyValue + tolAdd);
                    }
                    else
                    {
                        edgeEdge.SetFuzzyValue(_fuzzyValue);
                    }
                }
            }
        }

        int nbPairs = vEdgeEdge.Count;
        if (nbPairs == 0)
        {
            return;
        }

        // close preparation step
        psOuter.Next(0.7);

        pbMap.Clear();
        mpbFence.Clear();

        using (var ps = new ProgressScope(psOuter.Next(9), "Checking for coinciding edges", nbPairs))
        {
            for (int i = 0; i < nbPairs; i++)
            {
                EdgeEdgeJob edgeEdge = vEdgeEdge[i];
                edgeEdge.SetProgressRange(ps.Next());
            }

            // Perform intersection of the found pairs
            BooleanParallel.Perform(_runParallel, vEdgeEdge);
        }
        if (UserBreak(psOuter))
        {
            return;
        }
        List<InterfEE> eEs = _ds.InterfEE();

        // Analyze the results of intersection looking for TopAbs_EDGE
        // intersection type only.

        IndexedMap<PaveBlock, NCollection_List<PaveBlock>> mpblpb = new();
        for (int i = 0; i < nbPairs; ++i)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            EdgeEdgeJob edgeEdge = vEdgeEdge[i];
            if (!edgeEdge.IsDone || edgeEdge.HasErrors)
            {
                // Warn about failed intersection of sub-shapes
                Shape e1 = _ds.Shape(edgeEdge.PaveBlock1().OriginalEdge());
                Shape e2 = _ds.Shape(edgeEdge.PaveBlock2().OriginalEdge());
                AddIntersectionFailedWarning(e1, e2);
                continue;
            }

            NCollection_Sequence<CommonPrt> cParts = edgeEdge.CommonParts();
            if (cParts.Length() != 1)
            {
                continue;
            }

            CommonPrt cp = cParts[1];
            if (cp.Type() != ShapeType.Edge)
            {
                continue;
            }

            PaveBlock[] pb = [edgeEdge.PaveBlock1(), edgeEdge.PaveBlock2()];
            int nE1 = pb[0].OriginalEdge();
            int nE2 = pb[1].OriginalEdge();

            if (_ds.Rank(nE1) == _ds.Rank(nE2))
            {
                // Add acquired self-interference warning
                Compound wc = new();
                new BrepBuilder().MakeCompound(ref wc);
                new BrepBuilder().Add(wc, _ds.Shape(nE1));
                new BrepBuilder().Add(wc, _ds.Shape(nE2));
                AddWarning(new AlertAcquiredSelfIntersection(wc));
            }

            InterfEE ee =  new InterfEE();
            eEs.Add(ee);
            ee.SetIndices(nE1, nE2);
            ee.SetCommonPart(cp);
            _ds.AddInterf(nE1, nE2);

            // Fill map for common blocks creation
            for (int j = 0; j < 2; ++j)
            {
                if (_ds.IsCommonBlock(pb[j]))
                {
                    NCollection_List<PaveBlock> lpbcb = _ds.CommonBlock(pb[j])!.PaveBlocks();
                    NCollection_List<PaveBlock>.Iterator itLPB = new(lpbcb);
                    for (; itLPB.More(); itLPB.Next())
                    {
                        BuilderTools.FillMap(pb[j], itLPB.Value(), mpblpb);
                    }
                }
            }
            BuilderTools.FillMap(pb[0], pb[1], mpblpb);
        }
        // Create new common blocks of coinciding pairs.
        BuilderTools.PerformCommonBlocks(mpblpb, _ds);
    }
}
