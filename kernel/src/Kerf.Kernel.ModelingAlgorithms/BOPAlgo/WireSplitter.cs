// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireSplitter.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireSplitter.lxx
// Port of: src/ModelingAlgorithms/TKBO/BOPAlgo/BOPAlgo_WireSplitter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;

namespace Kerf.Engine.Booleans;

/// <summary>
/// Builds the loops (wires) from the edges of a BOPAlgo_WireEdgeSet on its face: the connexity blocks that are
/// regular make one wire each, the others are split by following the smallest clockwise angle at each vertex.
/// </summary>
internal partial class WireSplitter : BooleanAlgorithm
{
    protected WireEdgeSet? _wes;
    protected NCollection_List<ConnexityBlock> _lcb = new();
    protected IntersectionContext? _context;

    public WireSplitter()
    {
        _wes = null;
    }

    /// <summary>Sets the edge set (held by reference, as OCCT's pointer: the wires are added to it).</summary>
    public void SetWES(WireEdgeSet wes) => _wes = wes;

    public WireEdgeSet WES() => _wes!;

    public void SetContext(IntersectionContext? context) => _context = context;

    public IntersectionContext? Context() => _context;

    protected override void CheckData()
    {
        if (_wes is null)
        {
            AddError(new AlertNullInputShapes());
            return;
        }
    }

    public override void Perform(ProgressRange range = default)
    {
        GetReport().Clear();
        using var ps = new ProgressScope(range, "Building wires", 1);
        //
        CheckData();
        if (HasErrors)
        {
            return;
        }
        //
        // create a context
        if (_context is null)
        {
            _context = new IntersectionContext();
        }
        //
        AlgoTools.MakeConnexityBlocks(_wes!.StartElements(), ShapeType.Vertex, ShapeType.Edge, _lcb);
        if (UserBreak(ps))
        {
            return;
        }

        MakeWires(ps.Next());
    }

    /// <summary>Makes a wire of the edges (the .lxx's inline function); the wire is closed if its edges are.</summary>
    public static void MakeWire(NCollection_List<Shape> aLE, out Wire aWire)
    {
        BrepBuilder bb = new();
        aWire = new Wire();
        bb.MakeWire(ref aWire);
        NCollection_List<Shape>.Iterator it = new(aLE);
        for (; it.More(); it.Next())
        {
            bb.Add(aWire, it.Value());
        }
        aWire.Closed(Tool.IsClosed(aWire));
    }

    protected void MakeWires(ProgressRange range)
    {
        bool bIsRegular;
        int nbVCB, k;
        Wire w;
        NCollection_List<ConnexityBlock>.Iterator itCB = new(_lcb);
        NCollection_List<Shape>.Iterator it;
        List<WS_ConnexityBlock> vcb = new();
        //
        using var psOuter = new ProgressScope(range, null, 1);
        //
        Face f = _wes!.Face();
        //
        itCB.Initialize(_lcb);
        for (; itCB.More(); itCB.Next())
        {
            if (UserBreak(psOuter))
            {
                return;
            }

            ConnexityBlock cb = itCB.Value();
            bIsRegular = cb.IsRegular;
            if (bIsRegular)
            {
                NCollection_List<Shape> le = cb.ChangeShapes();
                MakeWire(le, out w);
                _wes.AddShape(w);
            }
            else
            {
                WS_ConnexityBlock wscb = new();
                vcb.Add(wscb);
                wscb.SetFace(f);
                wscb.SetConnexityBlock(cb);
            }
        }
        nbVCB = vcb.Count;
        using var psParallel = new ProgressScope(psOuter.Next(), null, nbVCB);
        for (int iW = 0; iW < nbVCB; ++iW)
        {
            vcb[iW].SetProgressRange(psParallel.Next());
        }
        //===================================================
        BooleanParallel.Perform(_runParallel, vcb, _context!);
        //===================================================
        for (k = 0; k < nbVCB; ++k)
        {
            WS_ConnexityBlock cb = vcb[k];
            NCollection_List<Shape> lw = cb.ConnexityBlock().Loops();
            it = new NCollection_List<Shape>.Iterator(lw);
            for (; it.More(); it.Next())
            {
                Shape wx = it.Value();
                _wes.AddShape(wx);
            }
        }
    }
}

/// <summary>An edge at a vertex in the wire splitter: incoming or outgoing, its angle, whether it has been passed.</summary>
/// <remarks>A working object held in the splitter's lists and changed in place (OCCT's code keeps pointers to them).</remarks>
internal sealed class EdgeInfo
{
    private Edge _edge;
    private bool _passed;
    private bool _inFlag;
    private bool _isInside;
    private double _angle;

    public EdgeInfo()
    {
        _edge = new Edge();
        _passed = false;
        _inFlag = false;
        _isInside = false;
        _angle = -1.0;
    }

    public void SetEdge(in Edge e) => _edge = e;

    public Edge Edge() => _edge;

    public void SetPassed(bool flag) => _passed = flag;

    public bool Passed() => _passed;

    public void SetInFlag(bool flag) => _inFlag = flag;

    public bool IsIn => _inFlag;

    public void SetAngle(double angle) => _angle = angle;

    public double Angle() => _angle;

    public bool IsInside => _isInside;

    public void SetIsInside(bool isInside) => _isInside = isInside;
}

/// <summary>The solver splitting one irregular connexity block (OCCT's file-local <c>BOPAlgo_WS_ConnexityBlock</c>).</summary>
internal sealed class WS_ConnexityBlock : BooleanParallel.IContextSolver<IntersectionContext>
{
    private Face _face = new();
    private ConnexityBlock _cb = new();
    private IntersectionContext? _context;
    private ProgressRange _range;

    public void SetFace(in Face f) => _face = f;

    public Face Face() => _face;

    // OCCT copies the block by value
    public void SetConnexityBlock(ConnexityBlock cb) => _cb = new ConnexityBlock(cb);

    public ConnexityBlock ConnexityBlock() => _cb;

    public void SetContext(IntersectionContext aContext) => _context = aContext;

    public IntersectionContext? Context() => _context;

    public void SetProgressRange(ProgressRange range) => _range = range;

    public void Perform()
    {
        using var ps = new ProgressScope(_range, null, 1);
        if (!ps.More())
        {
            return;
        }
        WireSplitter.SplitBlock(_face, _cb, _context!);
    }
}
