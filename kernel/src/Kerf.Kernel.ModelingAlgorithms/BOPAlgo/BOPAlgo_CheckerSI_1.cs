// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_CheckerSI_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Classification;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Booleans;

/// <summary>A vertex classified against a solid (OCCT's file-local BOPAlgo_VertexSolid).</summary>
internal sealed class VertexSolid : BooleanParallel.IContextSolver<IntersectionContext>
{
    private int _iv;
    private int _iz;
    private Containment _state;
    private Vertex _v = new();
    private Solid _z = new();
    private IntersectionContext? _context;
    private ProgressRange _progressRange;

    public VertexSolid()
    {
        _iv = -1;
        _iz = -1;
        _state = Unknown;
    }

    //
    public void SetIndices(int nV, int nZ)
    {
        _iv = nV;
        _iz = nZ;
    }

    //
    public void Indices(out int nV, out int nZ)
    {
        nV = _iv;
        nZ = _iz;
    }

    //
    public void SetVertex(in Vertex aV) => _v = aV;

    //
    public Vertex Vertex() => _v;

    //
    public void SetSolid(in Solid aZ) => _z = aZ;

    //
    public Solid Solid() => _z;

    //
    public void SetContext(IntersectionContext aContext) => _context = aContext;

    //
    public IntersectionContext? Context() => _context;

    //
    public Containment State() => _state;

    //
    public void SetProgressRange(ProgressRange range) => _progressRange = range;

    //
    public void Perform()
    {
        using var ps = new ProgressScope(_progressRange, null, 1);
        if (!ps.More())
        {
            return;
        }

        double tol;
        Point3D pv;
        //
        SolidClassifier sc = _context!.SolidClassifier(_z);
        //
        pv = Tool.Pnt(_v);
        tol = Tool.Tolerance(_v);
        //
        sc.Perform(pv, tol);
        //
        _state = sc.State();
    }
}

/// <summary>An edge or face tested against a solid (OCCT's file-local BOPAlgo_ShapeSolid).</summary>
internal class ShapeSolid : BooleanParallel.ISolver
{
    protected int _ie;
    protected int _iz;
    protected bool _hasInterf;
    protected DS? _ds;
    protected ProgressRange _progressRange;

    public ShapeSolid()
    {
        _ie = -1;
        _iz = -1;
        _hasInterf = false;
        _ds = null;
    }

    //
    public void SetIndices(int nE, int nZ)
    {
        _ie = nE;
        _iz = nZ;
    }

    //
    public void Indices(out int nE, out int nZ)
    {
        nE = _ie;
        nZ = _iz;
    }

    //
    public void SetDS(DS pDS) => _ds = pDS;

    //
    public bool HasInterf => _hasInterf;

    //
    public void SetProgressRange(ProgressRange range) => _progressRange = range;

    //
    public virtual void Perform()
    {
        using var ps = new ProgressScope(_progressRange, null, 1);
        if (!ps.More())
        {
            return;
        }

        bool bHasInterf;
        //
        _hasInterf = false;
        //
        bHasInterf = _ds!.HasInterfShapeSubShapes(_ie, _iz);
        if (!bHasInterf)
        {
            _hasInterf = _ds.HasInterfShapeSubShapes(_iz, _ie);
        }
    }
}

/// <summary>Two solids tested against each other (OCCT's file-local BOPAlgo_SolidSolid).</summary>
internal sealed class SolidSolid : ShapeSolid
{
    //
    public override void Perform()
    {
        using var ps = new ProgressScope(_progressRange, null, 1);
        if (!ps.More())
        {
            return;
        }

        bool bFlag;
        //
        bFlag = false;
        _hasInterf = false;
        //
        _hasInterf = _ds!.HasInterfShapeSubShapes(_iz, _ie, bFlag);
        if (!_hasInterf)
        {
            _hasInterf = _ds.HasInterfShapeSubShapes(_ie, _iz, bFlag);
        }
    }
}

internal partial class CheckerSI
{
    /// <summary>Vertex/solid intersection: a vertex classified IN a solid that doesn't share it.</summary>
    protected virtual void PerformVZ(ProgressRange range)
    {
        using var psOuter = new ProgressScope(range, null, 1);

        int iSize, nV, nZ, k, nbVVS;
        Containment state;
        HashSet<Pair> mpk = new();
        //
        _iterator!.Initialize(ShapeType.Vertex, ShapeType.Solid);
        iSize = _iterator.ExpectedLength();
        if (iSize == 0)
        {
            return;
        }
        //
        List<InterfVZ> vZs = _ds!.InterfVZ();
        //
        List<VertexSolid> vvs = [];
        //
        for (; _iterator.More(); _iterator.Next())
        {
            if (UserBreak(psOuter))
            {
                return;
            }
            _iterator.Value(out nV, out nZ);
            //
            if (_ds.HasInterfShapeSubShapes(nV, nZ))
            {
                continue;
            }
            //
            int nVSD = nV;
            _ds.HasShapeSD(nV, ref nVSD);
            //
            Pair pk = new();
            pk.SetIndices(nVSD, nZ);
            if (!mpk.Add(pk))
            {
                continue;
            }
            //
            Vertex v = ShapeCasts.Vertex(_ds.Shape(nVSD));
            Solid z = ShapeCasts.Solid(_ds.Shape(nZ));
            //
            VertexSolid vertexSolid = new();
            vvs.Add(vertexSolid);
            vertexSolid.SetIndices(nV, nZ);
            vertexSolid.SetVertex(v);
            vertexSolid.SetSolid(z);
        }
        //
        nbVVS = vvs.Count;

        using var psParallel = new ProgressScope(psOuter.Next(), "Performing Vertex-Solid intersection", nbVVS);
        for (int iVS = 0; iVS < nbVVS; ++iVS)
        {
            vvs[iVS].SetProgressRange(psParallel.Next());
        }
        //=============================================================
        BooleanParallel.Perform(_runParallel, vvs, _context!);
        //=============================================================
        if (UserBreak(psOuter))
        {
            return;
        }
        for (k = 0; k < nbVVS; ++k)
        {
            VertexSolid vertexSolid = vvs[k];
            state = vertexSolid.State();
            if (state == In)
            {
                vertexSolid.Indices(out nV, out nZ);
                //
                InterfVZ vz =  new InterfVZ();
                vZs.Add(vz);
                vz.SetIndices(nV, nZ);
                //
                _ds.AddInterf(nV, nZ);
            }
        }
    }

    /// <summary>Edge/solid intersection.</summary>
    protected virtual void PerformEZ(ProgressRange range)
    {
        PerformSZ(ShapeType.Edge, range);
    }

    /// <summary>Face/solid intersection.</summary>
    protected virtual void PerformFZ(ProgressRange range)
    {
        PerformSZ(ShapeType.Face, range);
    }

    /// <summary>Solid/solid intersection.</summary>
    protected virtual void PerformZZ(ProgressRange range)
    {
        using var psOuter = new ProgressScope(range, null, 1);

        bool bHasInterf;
        int iSize, nZ1, nZ, k, nbSolidSolid;
        //
        _iterator!.Initialize(ShapeType.Solid, ShapeType.Solid);
        iSize = _iterator.ExpectedLength();
        if (iSize == 0)
        {
            return;
        }
        //
        List<SolidSolid> vSolidSolid = [];
        //
        for (; _iterator.More(); _iterator.Next())
        {
            _iterator.Value(out nZ1, out nZ);
            //
            SolidSolid solidSolid = new();
            vSolidSolid.Add(solidSolid);
            solidSolid.SetIndices(nZ1, nZ);
            solidSolid.SetDS(_ds!);
        }
        //
        nbSolidSolid = vSolidSolid.Count;

        using var psParallel = new ProgressScope(psOuter.Next(), "Performing Solid-Solid intersection", nbSolidSolid);
        for (int iSS = 0; iSS < nbSolidSolid; ++iSS)
        {
            vSolidSolid[iSS].SetProgressRange(psParallel.Next());
        }
        //======================================================
        BooleanParallel.Perform(_runParallel, vSolidSolid);
        //======================================================
        if (UserBreak(psOuter))
        {
            return;
        }
        //
        List<InterfZZ> zZs = _ds!.InterfZZ();
        //
        for (k = 0; k < nbSolidSolid; ++k)
        {
            SolidSolid solidSolid = vSolidSolid[k];
            bHasInterf = solidSolid.HasInterf;
            if (bHasInterf)
            {
                solidSolid.Indices(out nZ1, out nZ);
                //
                InterfZZ zz =  new InterfZZ();
                zZs.Add(zz);
                zz.SetIndices(nZ1, nZ);
                //
                _ds.AddInterf(nZ1, nZ);
            }
        }
    }

    /// <summary>Edge/solid or face/solid intersection: the shape and the solid share interfering sub-shapes.</summary>
    protected virtual void PerformSZ(ShapeType ts, ProgressRange range)
    {
        using var psOuter = new ProgressScope(range, null, 1);

        bool bHasInterf;
        int iSize, nS, nZ, k, nbShapeSolid;
        //
        _iterator!.Initialize(ts, ShapeType.Solid);
        iSize = _iterator.ExpectedLength();
        if (iSize == 0)
        {
            return;
        }
        //
        List<ShapeSolid> vShapeSolid = [];
        //
        for (; _iterator.More(); _iterator.Next())
        {
            _iterator.Value(out nS, out nZ);
            //
            ShapeSolid shapeSolid = new();
            vShapeSolid.Add(shapeSolid);
            shapeSolid.SetIndices(nS, nZ);
            shapeSolid.SetDS(_ds!);
        }
        //
        nbShapeSolid = vShapeSolid.Count;

        using var psParallel = new ProgressScope(psOuter.Next(),
                                                          ts == ShapeType.Edge ? "Performing Edge-Solid intersection"
                                                                               : "Performing Face-Solid intersection",
                                                          nbShapeSolid);
        for (int iSS = 0; iSS < nbShapeSolid; ++iSS)
        {
            vShapeSolid[iSS].SetProgressRange(psParallel.Next());
        }
        //======================================================
        BooleanParallel.Perform(_runParallel, vShapeSolid);
        //======================================================
        if (UserBreak(psOuter))
        {
            return;
        }
        //
        List<InterfEZ> eZs = _ds!.InterfEZ();
        List<InterfFZ> fZs = _ds.InterfFZ();
        //
        for (k = 0; k < nbShapeSolid; ++k)
        {
            ShapeSolid shapeSolid = vShapeSolid[k];
            bHasInterf = shapeSolid.HasInterf;
            if (bHasInterf)
            {
                shapeSolid.Indices(out nS, out nZ);
                //
                if (ts == ShapeType.Edge)
                {
                    InterfEZ ez =  new InterfEZ();
                    eZs.Add(ez);
                    ez.SetIndices(nS, nZ);
                }
                else
                { // if (aTS==TopAbs_FACE)
                    InterfFZ fz =  new InterfFZ();
                    fZs.Add(fz);
                    fz.SetIndices(nS, nZ);
                }
                //
                _ds.AddInterf(nS, nZ);
            }
        }
    }
}
