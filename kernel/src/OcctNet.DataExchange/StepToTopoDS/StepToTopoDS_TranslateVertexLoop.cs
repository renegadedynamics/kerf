// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateVertexLoop.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateVertexLoop.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.StepToTopoDS_TranslateVertexLoopError;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>Translates a vertex_loop into a wire of one degenerated edge.</summary>
/// <remarks><see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 13004).</remarks>
public sealed class StepToTopoDS_TranslateVertexLoop : StepToTopoDS_Root
{
    private StepToTopoDS_TranslateVertexLoopError myError;
    private TopoDS_Shape myResult = new TopoDS_Shape();

    public StepToTopoDS_TranslateVertexLoop()
    {
        myError = StepToTopoDS_TranslateVertexLoopOther;
    }

    public StepToTopoDS_TranslateVertexLoop(StepShape_VertexLoop VL, StepToTopoDS_Tool T, StepToTopoDS_NMTool NMTool, in StepData_Factors theLocalFactors)
    {
        Init(VL, T, NMTool, theLocalFactors);
    }

    public StepToTopoDS_TranslateVertexLoop(StepShape_VertexLoop VL, StepToTopoDS_Tool T, StepToTopoDS_NMTool NMTool)
        : this(VL, T, NMTool, new StepData_Factors())
    {
    }

    public void Init(StepShape_VertexLoop VL, StepToTopoDS_Tool aTool, StepToTopoDS_NMTool NMTool, in StepData_Factors theLocalFactors)
    {
        // A Vertex Loop shall be mapped onto a Vertex + Edge + Wire;
        if (!aTool.IsBound(VL))
        {
            var B = new BRep_Builder();
            Transfer_TransientProcess TP = aTool.TransientProcess();

            //: S4136    double preci = BRepAPI::Precision();
            StepShape_Vertex Vtx;
            TopoDS_Vertex V1 = new TopoDS_Vertex(), V2 = new TopoDS_Vertex();
            TopoDS_Edge E = new TopoDS_Edge();
            TopoDS_Wire W = new TopoDS_Wire();
            Vtx = VL.LoopVertex();
            var myTranVtx = new StepToTopoDS_TranslateVertex(Vtx, aTool, NMTool, theLocalFactors);
            if (myTranVtx.IsDone())
            {
                V1 = TopoDS.Vertex(myTranVtx.InternalValue());
                V2 = TopoDS.Vertex(myTranVtx.InternalValue());
            }
            else
            {
                TP.AddWarning(VL, "VertexLoop not mapped to TopoDS ");
                myError = StepToTopoDS_TranslateVertexLoopOther;
                done = false;
                return;
            }

            V1.Orientation(TopAbs_FORWARD);
            V2.Orientation(TopAbs_REVERSED);
            B.MakeEdge(ref E);
            B.Add(E, V1);
            B.Add(E, V2);
            B.Degenerated(E, true);

            B.MakeWire(ref W);
            W.Closed(true);
            B.Add(W, E);
            aTool.Bind(VL, W);
            myResult = W;
            myError = StepToTopoDS_TranslateVertexLoopDone;
            done = true;
        }
        else
        {
            myResult = TopoDS.Wire(aTool.Find(VL));
            myError = StepToTopoDS_TranslateVertexLoopDone;
            done = true;
        }
    }

    public void Init(StepShape_VertexLoop VL, StepToTopoDS_Tool aTool, StepToTopoDS_NMTool NMTool)
        => Init(VL, aTool, NMTool, new StepData_Factors());

    /// <summary>The wire, frozen.</summary>
    /// <exception cref="StdFail_NotDone">The translation wasn't done.</exception>
    public TopoDS_Shape Value()
    {
        TopoDS_Shape aShape = InternalValue();
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal TopoDS_Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!done, "StepToTopoDS_TranslateVertexLoop::Value() - no result");
        return myResult;
    }

    public StepToTopoDS_TranslateVertexLoopError Error() => myError;
}
