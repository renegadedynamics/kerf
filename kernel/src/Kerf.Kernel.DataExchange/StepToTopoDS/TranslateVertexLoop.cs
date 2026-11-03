// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateVertexLoop.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateVertexLoop.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Exchange.Step.Import.TranslateVertexLoopError;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>Translates a vertex_loop into a wire of one degenerated edge.</summary>
/// <remarks><see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1247).</remarks>
public sealed class TranslateVertexLoop : ImportRoot
{
    private TranslateVertexLoopError _error;
    private Shape _result = new Shape();

    public TranslateVertexLoop()
    {
        _error = TranslateVertexLoopError.TranslateVertexLoopOther;
    }

    public TranslateVertexLoop(StepVertexLoop vl, ImportTool t, NMTool nmTool, in Factors localFactors)
    {
        Init(vl, t, nmTool, localFactors);
    }

    public TranslateVertexLoop(StepVertexLoop vl, ImportTool t, NMTool nmTool)
        : this(vl, t, nmTool, new Factors())
    {
    }

    public void Init(StepVertexLoop vl, ImportTool aTool, NMTool nmTool, in Factors localFactors)
    {
        // A Vertex Loop shall be mapped onto a Vertex + Edge + Wire;
        if (!aTool.IsBound(vl))
        {
            var b = new BrepBuilder();
            TransientProcess tp = aTool.TransientProcess();

            //: S4136    double preci = BRepAPI::Precision();
            StepVertex vtx;
            Vertex v1 = new Vertex(), v2 = new Vertex();
            Edge e = new Edge();
            Wire w = new Wire();
            vtx = vl.LoopVertex();
            var myTranVtx = new TranslateVertex(vtx, aTool, nmTool, localFactors);
            if (myTranVtx.IsDone)
            {
                v1 = ShapeCasts.Vertex(myTranVtx.InternalValue());
                v2 = ShapeCasts.Vertex(myTranVtx.InternalValue());
            }
            else
            {
                tp.AddWarning(vl, "VertexLoop not mapped to TopoDS ");
                _error = TranslateVertexLoopError.TranslateVertexLoopOther;
                _done = false;
                return;
            }

            v1.Orient(Forward);
            v2.Orient(Reversed);
            b.MakeEdge(ref e);
            b.Add(e, v1);
            b.Add(e, v2);
            b.Degenerated(e, true);

            b.MakeWire(ref w);
            w.Closed(true);
            b.Add(w, e);
            aTool.Bind(vl, w);
            _result = w;
            _error = TranslateVertexLoopError.TranslateVertexLoopDone;
            _done = true;
        }
        else
        {
            _result = ShapeCasts.Wire(aTool.Find(vl));
            _error = TranslateVertexLoopError.TranslateVertexLoopDone;
            _done = true;
        }
    }

    public void Init(StepVertexLoop vl, ImportTool aTool, NMTool nmTool)
        => Init(vl, aTool, nmTool, new Factors());

    /// <summary>The wire, frozen.</summary>
    /// <exception cref="NotDoneException">The translation wasn't done.</exception>
    public Shape Value()
    {
        Shape shape = InternalValue();
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!_done, "StepToTopoDS_TranslateVertexLoop::Value() - no result");
        return _result;
    }

    public TranslateVertexLoopError Error() => _error;
}
