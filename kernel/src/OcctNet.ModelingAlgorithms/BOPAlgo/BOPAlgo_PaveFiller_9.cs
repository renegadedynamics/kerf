// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_9.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>The shrunk range of a pave block, as a parallel solver of the pave filler.</summary>
internal sealed class ShrunkRange : EdgeShrunkRange, BooleanParallel.IContextSolver<IntersectionContext>
{
    private PaveBlock? _pb;

    public void SetPaveBlock(PaveBlock aPB) => _pb = aPB;

    public PaveBlock PaveBlock() => _pb!;

    void BooleanParallel.IContextSolver<IntersectionContext>.SetContext(IntersectionContext context) => SetContext(context);
}

public partial class PaveFiller
{
    protected void FillShrunkData(ShapeType aType1, ShapeType aType2)
    {
        _iterator!.Initialize(aType1, aType2);
        int iSize = _iterator.ExpectedLength();
        if (iSize == 0)
        {
            return;
        }
        //
        int i, nE, nV1, nV2, nbVSD, k;
        double t1, t2;
        Span<int> nS = stackalloc int[2];
        NCollection_List<PaveBlock>.Iterator itLPB = new();
        NCollection_IntMap mi = new();
        NCollection_DynamicArray<ShrunkRange> vsd = new();
        Span<ShapeType> type = [aType1, aType2];
        //
        for (; _iterator.More(); _iterator.Next())
        {
            _iterator.Value(out nS[0], out nS[1]);
            //
            for (i = 0; i < 2; ++i)
            {
                nE = nS[i];
                if (type[i] != ShapeType.Edge || !mi.Add(nE))
                {
                    continue;
                }
                //
                ShapeInfo sie = _ds!.ShapeInfo(nE);
                if (sie.HasFlag())
                {
                    continue;
                }
                //
                NCollection_List<PaveBlock> lpb = _ds.ChangePaveBlocks(nE);
                itLPB.Initialize(lpb);
                for (; itLPB.More(); itLPB.Next())
                {
                    PaveBlock pb = itLPB.Value();
                    if (pb.HasShrunkData && _ds.IsValidShrunkData(pb))
                    {
                        continue;
                    }
                    //
                    // FillShrunkData(aPB);
                    pb.Indices(out nV1, out nV2);
                    pb.Range(out t1, out t2);
                    //
                    Edge e = ShapeCasts.Edge(sie.Shape());
                    //
                    Vertex v1 = ShapeCasts.Vertex(_ds.Shape(nV1));
                    //
                    Vertex v2 = ShapeCasts.Vertex(_ds.Shape(nV2));
                    //
                    ShrunkRange sd = vsd.Append(new ShrunkRange());
                    //
                    sd.SetPaveBlock(pb);
                    sd.SetData(e, t1, t2, v1, v2);
                }
            }
        }
        //
        nbVSD = vsd.Length();
        //=============================================================
        BooleanParallel.Perform(_runParallel, vsd, _context!);
        //=============================================================
        //
        for (k = 0; k < nbVSD; ++k)
        {
            ShrunkRange sd = vsd[k];
            AnalyzeShrunkData(sd.PaveBlock(), sd);
        }
    }
}
