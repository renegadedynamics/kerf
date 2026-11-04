// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_PaveFiller_4.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Booleans.GlueMode;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans;

/// <summary>The vertex/face intersection solver of the pave filler's parallel loop.</summary>
internal sealed class VertexFace : ParallelAlgo, BooleanParallel.IContextSolver<IntersectionContext>
{
    private int _iv;
    private int _if;
    private int _flag;
    private double _t1;
    private double _t2;
    private double _tolVNew;
    private Vertex _v = new();
    private Face _f = new();
    private IntersectionContext? _context;

    public VertexFace()
    {
        _iv = -1;
        _if = -1;
        _flag = -1;
        _t1 = -1.0;
        _t2 = -1.0;
        _tolVNew = -1.0;
    }

    //
    public void SetIndices(int nV, int nF)
    {
        _iv = nV;
        _if = nF;
    }

    //
    public void Indices(out int nV, out int nF)
    {
        nV = _iv;
        nF = _if;
    }

    //
    public void SetVertex(in Vertex aV) => _v = aV;

    //
    public Vertex Vertex() => _v;

    //
    public void SetFace(in Face aF) => _f = aF;

    //
    public Face Face() => _f;

    //
    public int Flag() => _flag;

    //
    public void Parameters(out double aT1, out double aT2)
    {
        aT1 = _t1;
        aT2 = _t2;
    }

    //
    public double VertexNewTolerance() => _tolVNew;

    //
    public void SetContext(IntersectionContext aContext) => _context = aContext;

    //
    public IntersectionContext? Context() => _context;

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
            _flag = _context!.ComputeVF(_v, _f, ref _t1, ref _t2, ref _tolVNew, _fuzzyValue);
        }
        catch (FailureException)
        {
            AddError(new AlertIntersectionFailed());
        }
    }
}

internal partial class PaveFiller
{
    /// <summary>The vertex/face interferences.</summary>
    protected virtual void PerformVF(ProgressRange range)
    {
        _iterator!.Initialize(ShapeType.Vertex, ShapeType.Face);
        int iSize = _iterator.ExpectedLength();
        //
        int nV, nF;
        //
        using var psOuter = new ProgressScope(range, null, 10);
        if (_glue == global::Kerf.Booleans.GlueMode.Full)
        {
            // there is no need to intersect vertices with faces in this mode
            // just initialize FaceInfo for all faces
            for (; _iterator.More(); _iterator.Next())
            {
                _iterator.Value(out nV, out nF);
                if (!_ds!.IsSubShape(nV, nF))
                {
                    _ds.ChangeFaceInfo(nF);
                }
            }
            return;
        }
        //
        List<InterfVF> vFs = _ds!.InterfVF();
        if (iSize == 0)
        {
            //
            TreatVerticesEE();
            return;
        }
        //
        int nVSD = 0, iFlag, nVx, nbVF, k;
        List<VertexFace> vvf = new();
        //
        // Avoid repeated intersection of the same vertex with face in case
        // the group of vertices formed a single SD vertex
        Dictionary<Pair, NCollection_IntMap> mvfPairs = new();
        for (; _iterator.More(); _iterator.Next())
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            _iterator.Value(out nV, out nF);
            //
            if (_ds.IsSubShape(nV, nF))
            {
                continue;
            }
            //
            if (_ds.HasInterf(nV, nF))
            {
                continue;
            }
            //
            _ds.ChangeFaceInfo(nF);
            if (_ds.HasInterfShapeSubShapes(nV, nF))
            {
                continue;
            }
            //
            nVx = nV;
            if (_ds.HasShapeSD(nV, ref nVSD))
            {
                nVx = nVSD;
            }
            //
            Pair vfPair = new(nVx, nF);
            if (mvfPairs.TryGetValue(vfPair, out NCollection_IntMap? pMV0))
            {
                pMV0.Add(nV);
                continue;
            }

            NCollection_IntMap pMV = mvfPairs.SetValueRef(vfPair, new NCollection_IntMap());
            pMV.Add(nV);

            Vertex v = ShapeCasts.Vertex(_ds.Shape(nVx));
            Face f = ShapeCasts.Face(_ds.Shape(nF));
            //
            VertexFace vertexFace =  new VertexFace();
            vvf.Add(vertexFace);
            //
            vertexFace.SetIndices(nVx, nF);
            vertexFace.SetVertex(v);
            vertexFace.SetFace(f);
            vertexFace.SetFuzzyValue(_fuzzyValue);

        } // for (; myIterator->More(); myIterator->Next()) {
        //
        nbVF = vvf.Count;
        using (var ps = new ProgressScope(psOuter.Next(9), "Performing Vertex-Face intersection", nbVF))
        {
            for (k = 0; k < nbVF; k++)
            {
                VertexFace vertexFace = vvf[k];
                vertexFace.SetProgressRange(ps.Next());
            }
            //================================================================
            BooleanParallel.Perform(_runParallel, vvf, _context!);
            //================================================================
        }
        if (UserBreak(psOuter))
        {
            return;
        }
        //
        for (k = 0; k < nbVF; ++k)
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            VertexFace vertexFace = vvf[k];
            //
            iFlag = vertexFace.Flag();
            if (iFlag != 0)
            {
                if (vertexFace.HasErrors)
                {
                    // Warn about failed intersection of sub-shapes
                    AddIntersectionFailedWarning(vertexFace.Vertex(), vertexFace.Face());
                }
                continue;
            }
            //
            vertexFace.Indices(out nVx, out nF);
            vertexFace.Parameters(out double t1, out double t2);
            double tolVNew = vertexFace.VertexNewTolerance();

            Pair vfPair = new(nVx, nF);
            NCollection_IntMap mv = mvfPairs.GetValue(vfPair);
            // OCCT iterates the vertices in the int map's bucket order (decision 784): it decides the order of the
            // interferences and which nVx is added to VerticesIn below (the last one).
            foreach (int nVi in mv)
            {
                nV = nVi;
                // 1
                InterfVF vf =  new InterfVF();
                vFs.Add(vf);
                vf.SetIndices(nV, nF);
                vf.SetUV(t1, t2);
                // 2
                _ds.AddInterf(nV, nF);
                //
                // 3 update vertex V/F if necessary
                nVx = UpdateVertex(nV, tolVNew);
                //
                // 4
                if (_ds.IsNewShape(nVx))
                {
                    vf.SetIndexNew(nVx);
                }
            }
            // 5 update FaceInfo
            FaceInfo fi = _ds.ChangeFaceInfo(nF);
            NCollection_IntMap mvIn = fi.ChangeVerticesIn();
            mvIn.Add(nVx);
        } // for (k=0; k < aNbVF; ++k) {
        //
        TreatVerticesEE();
    }

    /// <summary>Intersects the new vertices of the edge/edge interferences with the faces.</summary>
    protected void TreatVerticesEE()
    {
        int i, nbS, nbEEs, nF, nV, iFlag;
        double t1 = 0.0, t2 = 0.0, dummy = 0.0; // OCCT: uninitialised, read only on success
        //
        NCollection_List<int> liv = new(), lif = new();
        NCollection_IntMap mi = new(100);
        //
        nbS = _ds!.SourceShapeCount;
        //
        List<InterfEE> eEs = _ds.InterfEE();
        nbEEs = eEs.Count;
        for (i = 0; i < nbEEs; ++i)
        {
            InterfEE ee = eEs[i];
            if (ee.HasIndexNew())
            {
                nV = ee.IndexNew();
                if (mi.Add(nV))
                {
                    liv.Append(nV);
                }
            }
        }
        if (liv.Extent() == 0)
        {
            return;
        }
        //
        nbS = _ds.SourceShapeCount;
        for (nF = 0; nF < nbS; ++nF)
        {
            ShapeInfo si = _ds.ShapeInfo(nF);
            if (si.ShapeType() == ShapeType.Face)
            {
                lif.Append(nF);
            }
        }
        if (lif.Extent() == 0)
        {
            return;
        }
        //-------------------------------------------------------------
        List<InterfVF> vFs = _ds.InterfVF();
        //
        SubIterator it = new();
        //
        it.SetDS(_ds);
        it.SetSubSet1(lif);
        it.SetSubSet2(liv);
        it.Prepare();
        it.Initialize();
        for (; it.More(); it.Next())
        {
            it.Value(out nV, out nF);
            //
            FaceInfo fi = _ds.ChangeFaceInfo(nF);
            NCollection_IntMap mvOn = fi.VerticesOn();
            //
            if (!mvOn.Contains(nV))
            {
                Vertex v = ShapeCasts.Vertex(_ds.Shape(nV));
                Face f = ShapeCasts.Face(_ds.Shape(nF));
                iFlag = _context!.ComputeVF(v, f, ref t1, ref t2, ref dummy, _fuzzyValue);
                if (iFlag == 0)
                {
                    // 1
                    InterfVF vf =  new InterfVF();
                    vFs.Add(vf);
                    i = vFs.Count - 1;
                    vf.SetIndices(nV, nF);
                    vf.SetUV(t1, t2);
                    // 2
                    _ds.AddInterf(nV, nF);
                    //
                    NCollection_IntMap mvIn = fi.ChangeVerticesIn();
                    mvIn.Add(nV);
                }
            }
        }
    }
}
