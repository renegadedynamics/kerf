// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_11.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

public partial class PaveFiller
{
    /// <summary>
    /// Checks for the self-interferences acquired by the arguments during the operation: shared vertices, common
    /// blocks and section shapes connecting several sub-shapes of one argument
    /// (<see cref="AlertAcquiredSelfIntersection"/> warnings).
    /// </summary>
    protected void CheckSelfInterference()
    {
        if (_arguments.Extent() == 1)
        {
            // Self-interference mode
            return;
        }
        //
        BrepBuilder bb = new();
        Span<int> nVs = stackalloc int[2];
        //
        int i, nbR = _ds!.RangeCount;
        for (i = 0; i < nbR; ++i)
        {
            IndexRange r = _ds.Range(i);
            //
            // Map of connections of interfering shapes
            IndexedMap<Shape, IndexedSet<Shape>> mcsi =
                new(ShapeHasher.Instance);
            HashSet<CommonBlock> mcbFence = new();
            //
            int j = r.First(), rLast = r.Last();
            for (; j <= rLast; ++j)
            {
                ShapeInfo si = _ds.ShapeInfo(j);
                if (!si.HasReference)
                {
                    // No pave blocks and no face info
                    continue;
                }
                //
                Shape s = si.Shape();
                //
                if (si.ShapeType() == ShapeType.Edge)
                {
                    if (si.HasFlag())
                    {
                        continue;
                    }
                    //
                    // Analyze the shared vertices and common blocks
                    //
                    NCollection_IntMap mSubS = new();
                    NCollection_List<int>.Iterator itLI = new(si.SubShapes());
                    for (; itLI.More(); itLI.Next())
                    {
                        int nV = itLI.Value();
                        _ds.HasShapeSD(nV, ref nV);
                        mSubS.Add(nV);
                    }
                    //
                    NCollection_List<PaveBlock> lpb = _ds.PaveBlocks(j);
                    bool bAnalyzeV = lpb.Extent() > 1;
                    //
                    NCollection_List<PaveBlock>.Iterator it = new(lpb);
                    for (; it.More(); it.Next())
                    {
                        PaveBlock pb = it.Value();
                        //
                        // Check the vertices
                        if (bAnalyzeV)
                        {
                            pb.Indices(out nVs[0], out nVs[1]);
                            for (int k = 0; k < 2; ++k)
                            {
                                if (!r.Contains(nVs[k]) && !mSubS.Contains(nVs[k]))
                                {
                                    // Add connection
                                    Shape v = _ds.Shape(nVs[k]);
                                    AddConnection(mcsi, v, s);
                                }
                            }
                        }
                        //
                        // Check common blocks
                        if (_ds.IsCommonBlock(pb))
                        {
                            CommonBlock cb = _ds.CommonBlock(pb)!;
                            if (mcbFence.Add(cb))
                            {
                                NCollection_List<PaveBlock> lpbcb = cb.PaveBlocks();
                                //
                                NCollection_List<int> le = new();
                                NCollection_List<PaveBlock>.Iterator itCB = new(lpbcb);
                                for (; itCB.More(); itCB.Next())
                                {
                                    PaveBlock pbcb = itCB.Value();
                                    int nEOr = pbcb.OriginalEdge();
                                    if (r.Contains(nEOr))
                                    {
                                        le.Append(nEOr);
                                    }
                                }
                                //
                                if (le.Extent() > 1)
                                {
                                    // Add the acquired self-interference warning:
                                    // The same common block contains several edges from one argument
                                    Compound wc = new();
                                    bb.MakeCompound(ref wc);
                                    //
                                    NCollection_List<int>.Iterator itLE = new(le);
                                    for (; itLE.More(); itLE.Next())
                                    {
                                        Shape e1 = _ds.Shape(itLE.Value());
                                        bb.Add(wc, e1);
                                    }
                                    //
                                    AddWarning(new AlertAcquiredSelfIntersection(wc));
                                }
                            }
                        }
                    }
                }
                else if (si.ShapeType() == ShapeType.Face)
                {
                    // Analyze IN and Section vertices and edges of the faces
                    FaceInfo fi = _ds.FaceInfo(j);
                    //
                    for (int k = 0; k < 2; ++k)
                    {
                        NCollection_IntMap mvf = k == 0 ? fi.VerticesIn() : fi.VerticesSc();
                        foreach (int vIndex in mvf)
                        {
                            Shape v = _ds.Shape(vIndex);
                            // add connection
                            AddConnection(mcsi, v, s);
                        }
                    }
                    //
                    for (int k = 0; k < 2; ++k)
                    {
                        IndexedSet<PaveBlock> mpbf = k == 0 ? fi.PaveBlocksIn() : fi.PaveBlocksSc();
                        int iPB, nbPB = mpbf.Count;
                        for (iPB = 1; iPB <= nbPB; ++iPB)
                        {
                            PaveBlock pb = mpbf[iPB - 1];
                            // OCCT's Standard_ASSERT(aPB->HasEdge(), ..., continue): in release builds it only continues.
                            if (!pb.HasEdge())
                            {
                                continue;
                            }
                            Shape e = _ds.Shape(pb.Edge());
                            // add connection
                            AddConnection(mcsi, e, s);
                        }
                    }
                }
            }
            //
            // Analyze connections
            int nbC = mcsi.Count;
            for (j = 1; j <= nbC; ++j)
            {
                IndexedSet<Shape> mcs = mcsi.ValueAt(j - 1);
                if (mcs.Count > 1)
                {
                    // Add acquired self-interference warning:
                    // Several faces from one argument contain the same vertex or edge
                    Compound wc = new();
                    bb.MakeCompound(ref wc);
                    //
                    int iS, nbS = mcs.Count;
                    for (iS = 1; iS <= nbS; ++iS)
                    {
                        Shape sx = mcs[iS - 1];
                        bb.Add(wc, sx);
                    }
                    AddWarning(new AlertAcquiredSelfIntersection(wc));
                }
            }
        }
    }

    // OCCT's ChangeSeek-or-Add of the connection map, written once.
    private static void AddConnection(IndexedMap<Shape, IndexedSet<Shape>> mcsi,
                                      in Shape key, in Shape shape)
    {
        int index = mcsi.IndexOf(key) + 1;
        IndexedSet<Shape> pMSOr = index > 0
            ? mcsi.ValueAt(index - 1)
            : mcsi.ValueAt(mcsi.Add(key, new IndexedSet<Shape>(ShapeHasher.Instance)));
        pMSOr.Add(shape);
    }
}
