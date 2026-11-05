// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_8.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

internal partial class PaveFiller
{
    /// <summary>Treats the degenerated edges: splits them by the pave blocks passing through their vertex.</summary>
    protected void ProcessDE(ProgressRange range)
    {
        using var psOuter = new ProgressScope(range, null, 1);
        //
        // 1. Find degenerated edges
        //-----------------------------------------------------scope f
        //

        for (int edgeIndex = 0; edgeIndex < _ds!.SourceShapeCount; ++edgeIndex)
        {
            ShapeInfo edgeInfo = _ds.ShapeInfo(edgeIndex);
            if (edgeInfo.ShapeType() != ShapeType.Edge)
            {
                continue;
            }

            if (edgeInfo.HasFlag(out int nF))
            {
                ShapeInfo sif = _ds.ShapeInfo(nF);
                int nV = edgeInfo.SubShapes().First();
                int nVSD = 0;
                if (_ds.HasShapeSD(nV, ref nVSD))
                {
                    nV = nVSD;
                }

                if (sif.ShapeType() == ShapeType.Face)
                {
                    // 1. Find PaveBlocks that go through nV for nF
                    NCollection_List<PaveBlock> lpbOut = new();
                    FindPaveBlocks(nV, nF, lpbOut);
                    if (!lpbOut.IsEmpty())
                    {
                        //
                        // 2.
                        NCollection_List<PaveBlock> lpbd = _ds.ChangePaveBlocks(edgeIndex);
                        // OCCT's Standard_ASSERT_VOID(!aLPBD.IsEmpty(), ...) is compiled out in release builds.
                        if (lpbd.IsEmpty())
                        {
                            continue;
                        }
                        PaveBlock pbd = lpbd.First();
                        //
                        FillPaves(nV, edgeIndex, nF, lpbOut, pbd);
                        //
                        _ds.UpdatePaveBlock(pbd);
                    }
                    //
                    MakeSplitEdge(edgeIndex, nF);
                }
                if (sif.ShapeType() == ShapeType.Edge)
                {
                    Edge de = ShapeCasts.Edge(_ds.Shape(edgeIndex));
                    Vertex vn = ShapeCasts.Vertex(_ds.Shape(nV));
                    //
                    Edge e = de;
                    e.EmptyCopy();
                    BrepBuilder bb = new();
                    bb.Add(e, vn);
                    bb.Degenerated(e, true);
                    bb.UpdateEdge(e, global::Kerf.Engine.Foundation.Precision.Confusion());
                    ShapeInfo si = new();
                    si.SetShapeType(ShapeType.Edge);
                    si.SetShape(e);
                    int nEn = _ds.Append(si);
                    NCollection_List<PaveBlock> lpbd = _ds.ChangePaveBlocks(edgeIndex);
                    PaveBlock pbd = lpbd.First();
                    pbd.SetEdge(nEn);
                }
            }

            if (UserBreak(psOuter))
            {
                return;
            }
        }
    }

    /// <summary>Finds the pave blocks of the face (In, On and Sc) that have the vertex as a bound.</summary>
    protected void FindPaveBlocks(int paveIndex, int faceInfoIndex, NCollection_List<PaveBlock> foundBlocks)
    {
        void ProcessPaveBlocks(IndexedSet<PaveBlock> paveBlocksMap)
        {
            for (int blockIndex = 0; blockIndex < paveBlocksMap.Count; ++blockIndex)
            {
                PaveBlock paveBlock = paveBlocksMap[blockIndex];
                paveBlock.Indices(out int nV1, out int nV2);
                if (paveIndex == nV1 || paveIndex == nV2)
                {
                    foundBlocks.Append(paveBlock);
                }
            }
        }

        FaceInfo faceInfo = _ds!.ChangeFaceInfo(faceInfoIndex);
        ProcessPaveBlocks(faceInfo.PaveBlocksIn());
        ProcessPaveBlocks(faceInfo.PaveBlocksOn());
        ProcessPaveBlocks(faceInfo.PaveBlocksSc());
    }

    /// <summary>Makes the split edges of the degenerated edge on the face.</summary>
    protected void MakeSplitEdge(int nDE, int nDF)
    {
        int nSp, nV1, nV2, nbPB;
        double t1, t2;
        Edge de, sp = new();
        Vertex v1, v2;
        NCollection_List<PaveBlock>.Iterator itLPB = new();
        ShapeInfo si = new();
        //
        si.SetShapeType(ShapeType.Edge);
        //
        de = ShapeCasts.Edge(_ds!.Shape(nDE));
        de.Orient(Forward);
        //
        Face df = ShapeCasts.Face(_ds.Shape(nDF));
        //
        NCollection_List<PaveBlock> lpb = _ds.ChangePaveBlocks(nDE);
        nbPB = lpb.Extent();
        //
        itLPB.Initialize(lpb);
        for (; itLPB.More(); itLPB.Next())
        {
            PaveBlock pb = itLPB.Value();
            //
            Pave pave1 = pb.Pave1();
            pave1.Contents(out nV1, out t1);
            //
            Pave pave2 = pb.Pave2();
            pave2.Contents(out nV2, out t2);
            //
            if (_ds.IsNewShape(nV1) || nbPB > 1)
            {
                v1 = ShapeCasts.Vertex(_ds.Shape(nV1));
                v1.Orient(Forward);
                //
                v2 = ShapeCasts.Vertex(_ds.Shape(nV2));
                v2.Orient(Reversed);
                //
                MakeSplitEdge1(de, df, v1, t1, v2, t2, out sp);
                //
                // OCCT reuses one ShapeInfo, which Append copies (decision 631).
                si.SetShape(sp);
                nSp = _ds.Append(si);
                pb.SetEdge(nSp);
            }
            else
            {
                _ds.ChangeShapeInfo(nDE).SetReference(-1);
                lpb.Clear();
                break;
            }
        }
    }

    /// <summary>
    /// Finds all pave blocks passing through the vertex <paramref name="nVD"/> and intersecting the 2D curve of the
    /// degenerated edge somewhere in the middle, and saves the intersection points as extra paves of the degenerated
    /// edge's pave block for its later splitting.
    /// </summary>
    protected void FillPaves(int nVD, int nED, int nFD, NCollection_List<PaveBlock> aLPBOut, PaveBlock aPBD)
    {
        // Prepare pave to put to pave block as an Extra pave
        Pave pave = new();
        pave.SetIndex(nVD);
        //
        Vertex dv = ShapeCasts.Vertex(_ds!.Shape(nVD));
        Edge de = ShapeCasts.Edge(_ds.Shape(nED));
        Face df = ShapeCasts.Face(_ds.Shape(nFD));
        //
        double tolV = Tool.Tolerance(dv);
        FaceSurfaceAdaptor bas = _context!.SurfaceAdaptor(df);
        //
        // 2D intersection tolerance should be computed as a resolution
        // from the tolerance of vertex to resolve the touching cases
        double tolInt = global::Kerf.Engine.Foundation.Precision.PConfusion();
        // UResolution from the tolerance of the vertex
        double uRes = bas.UResolution(tolV);
        // VResolution from the tolerance of the vertex
        double vRes = bas.VResolution(tolV);
        //
        tolInt = Max(tolInt, Max(uRes, vRes));
        //
        // Parametric tolerance to compare intersection point with boundaries
        // should be computed as a resolution from the tolerance of vertex
        // in the direction of the 2D curve of degenerated edge
        double tolCmp = global::Kerf.Engine.Foundation.Precision.PConfusion();
        // Get 2D curve
        Curve2D? c2DDE = Tool.CurveOnSurface(de, df, out double td1, out double td2);
        if (c2DDE is null)
        {
            return;
        }

        // Get direction of the curve
        bool bUDir = Math.Abs(c2DDE.Value(td1).Y - c2DDE.Value(td2).Y) < global::Kerf.Engine.Foundation.Precision.PConfusion();
        //
        tolCmp = Max(tolCmp, (bUDir ? uRes : vRes));
        //
        // Prepare adaptor for the degenerated edge for intersection
        CurveAdaptor2D gac1 = new();
        gac1.Load(c2DDE, td1, td2);
        //
        NCollection_List<PaveBlock>.Iterator itLPB = new(aLPBOut);
        for (; itLPB.More(); itLPB.Next())
        {
            PaveBlock pb = itLPB.Value();
            int nE = pb.Edge();
            if (nE < 0)
            {
                continue;
            }
            Edge e = ShapeCasts.Edge(_ds.Shape(nE));
            Curve2D? c2D = Tool.CurveOnSurface(e, df, out double t1, out double t2);
            if (c2D is null)
            {
                continue;
            }
            //
            // Prepare adaptor for the passing edge for intersection
            CurveAdaptor2D gac2 = new();
            //
            LineCurve2D? l2D = c2D as LineCurve2D;
            if (l2D is not null)
            {
                gac2.Load(c2D);
            }
            else
            {
                gac2.Load(c2D, t1, t2);
            }
            // Intersection
            GInter gInter = new(gac1, gac2, tolInt, tolInt);
            if (gInter.IsDone && gInter.PointCount != 0)
            {
                // Analyze intersection points
                int i, nbPoints = gInter.PointCount;
                for (i = 1; i <= nbPoints; ++i)
                {
                    double x = gInter.Point(i).ParamOnFirst();
                    pave.SetParameter(x);
                    AddSplitPoint(aPBD, pave, tolCmp);
                }
            }
            else
            {
                // If the intersection did not succeed, try the projection of the end point
                // of the curve corresponding to the vertex of degenerated edge
                double t = (nVD == pb.Pave1().Index() ? pb.Pave1().Parameter() : pb.Pave2().Parameter());
                Point2D p2d = c2D.Value(t);
                ProjectPointOnCurve2D proj2d = new(p2d, c2DDE, td1, td2);
                if (proj2d.NbPoints() != 0)
                {
                    double x = proj2d.LowerDistanceParameter();
                    pave.SetParameter(x);
                    AddSplitPoint(aPBD, pave, tolCmp);
                }
            }
        }
    }

    // A split of the degenerated edge: an empty copy with the two vertices and the range on the face.
    private static void MakeSplitEdge1(in Edge aE, in Face aF, in Vertex aV1, double aP1,
                                       in Vertex aV2, double aP2, out Edge aNewEdge)
    {
        double tol = 1.0e-7;

        Edge e = aE;

        e.EmptyCopy();
        BrepBuilder bb = new();
        bb.Add(e, aV1);
        bb.Add(e, aV2);

        bb.Range(e, aF, aP1, aP2);

        bb.Degenerated(e, true);

        bb.UpdateEdge(e, tol);
        aNewEdge = e;
    }

    // Validates the point represented by the pave for the pave block: if it passes the checks, it is added as an
    // extra pave of the pave block for its later splitting. Returns true if the point is added.
    private static bool AddSplitPoint(PaveBlock pbd, in Pave pave, double tol)
    {
        pbd.Range(out double td1, out double td2);

        double t = pave.Parameter();
        // Check that the parameter is inside the Pave Block
        if (t - td1 < tol || td2 - t < tol)
        {
            return false;
        }

        // Check that the pave block does not contain the same parameter
        int ind = 0;
        if (pbd.ContainsParameter(t, tol, ref ind))
        {
            return false;
        }

        // Add the point as an Extra pave to the Pave Block for further
        // splitting of the latter
        pbd.AppendExtPave1(pave);
        return true;
    }
}
