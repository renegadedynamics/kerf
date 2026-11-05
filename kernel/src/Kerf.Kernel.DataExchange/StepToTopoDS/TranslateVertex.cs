// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateVertex.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Exchange.Step.Import.TranslateVertexError;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>Translates a STEP vertex (a vertex_point) into a TopoDS_Vertex.</summary>
/// <remarks>
/// <see cref="Value"/> freezes the result (decision 1247); the translators use <c>InternalValue</c>, the working
/// shape.
/// </remarks>
internal sealed class TranslateVertex : ImportRoot
{
    private TranslateVertexError _error;
    private Shape _result = new Shape();

    public TranslateVertex()
    {
        _error = TranslateVertexError.TranslateVertexOther;
        _done = false;
    }

    public TranslateVertex(StepVertex v, ImportTool t, NMTool nmTool, in Factors localFactors)
    {
        Init(v, t, nmTool, localFactors);
    }

    public TranslateVertex(StepVertex v, ImportTool t, NMTool nmTool)
        : this(v, t, nmTool, new Factors())
    {
    }

    public void Init(StepVertex aVertex, ImportTool aTool, NMTool nmTool, in Factors localFactors)
    {
        if (aVertex is null)
        {
            _error = TranslateVertexError.TranslateVertexOther;
            _done = false;
            return;
        }

        if (!aTool.IsBound(aVertex))
        {
            // [BEGIN] Proceed with non-manifold topology (ssv; 14.11.2010)
            if (nmTool.IsActive && nmTool.IsBound(aVertex))
            {
                _result = nmTool.Find(aVertex);
                _error = TranslateVertexError.TranslateVertexDone;
                _done = true;
                return;
            }
            // [END] Proceed with non-manifold topology (ssv; 14.11.2010)

            // [BEGIN] Proceed with I-DEAS-like STP (ssv; 15.11.2010)
            TCollection_HAsciiString vName = aVertex.Name();
            if (nmTool.IsActive && nmTool.IsIDEASCase && vName is not null && !vName.IsEmpty()
                && nmTool.IsBound(vName.String()))
            {
                _result = nmTool.Find(vName.String());
                _error = TranslateVertexError.TranslateVertexDone;
                _done = true;
                return;
            }
            // [END] Proceed with I-DEAS-like STP (ssv; 15.11.2010)

            //: S4136    double preci = BRepAPI::Precision();
            StepVertexPoint vp = aVertex as StepVertexPoint;
            StepPoint p = vp.VertexGeometry();
            if (p is null)
            {
                _error = TranslateVertexError.TranslateVertexOther;
                _done = false;
                return;
            }

            StepCartesianPoint p1 = p as StepCartesianPoint;
            CartesianPoint p2 = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCartesianPoint(p1, localFactors);
            var b = new BrepBuilder();
            Vertex v = new Vertex();
            b.MakeVertex(ref v, p2.Pnt(), Kerf.Engine.Foundation.Precision.Confusion()); //: S4136: preci
            aTool.Bind(aVertex, v);
            // Register Vertex in NM tool (ssv; 14.11.2010)
            if (nmTool.IsActive)
            {
                nmTool.Bind(aVertex, v);
                if (nmTool.IsIDEASCase && vName is not null && !vName.IsEmpty())
                {
                    nmTool.Bind(vName.String(), v);
                }
            }

            _result = v;
        }
        else
        {
            _result = ShapeCasts.Vertex(aTool.Find(aVertex));
        }

        _error = TranslateVertexError.TranslateVertexDone;
        _done = true;
    }

    public void Init(StepVertex aVertex, ImportTool aTool, NMTool nmTool)
        => Init(aVertex, aTool, nmTool, new Factors());

    /// <summary>The vertex, frozen.</summary>
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
        StdFail_NotDone_Raise_if(!_done, "StepToTopoDS_TranslateVertex::Value() - no result");
        return _result;
    }

    public TranslateVertexError Error() => _error;
}
