// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateVertex.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslateVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using static Occt.StepToTopoDS_TranslateVertexError;

namespace Occt;

/// <summary>Translates a STEP vertex (a vertex_point) into a TopoDS_Vertex.</summary>
/// <remarks>
/// <see cref="Value"/> freezes the result (decision 1247); the translators use <c>InternalValue</c>, the working
/// shape.
/// </remarks>
public sealed class StepToTopoDS_TranslateVertex : StepToTopoDS_Root
{
    private StepToTopoDS_TranslateVertexError myError;
    private Shape myResult = new Shape();

    public StepToTopoDS_TranslateVertex()
    {
        myError = StepToTopoDS_TranslateVertexError.StepToTopoDS_TranslateVertexOther;
        done = false;
    }

    public StepToTopoDS_TranslateVertex(StepShape_Vertex V, StepToTopoDS_Tool T, StepToTopoDS_NMTool NMTool, in StepData_Factors theLocalFactors)
    {
        Init(V, T, NMTool, theLocalFactors);
    }

    public StepToTopoDS_TranslateVertex(StepShape_Vertex V, StepToTopoDS_Tool T, StepToTopoDS_NMTool NMTool)
        : this(V, T, NMTool, new StepData_Factors())
    {
    }

    public void Init(StepShape_Vertex aVertex, StepToTopoDS_Tool aTool, StepToTopoDS_NMTool NMTool, in StepData_Factors theLocalFactors)
    {
        if (aVertex is null)
        {
            myError = StepToTopoDS_TranslateVertexError.StepToTopoDS_TranslateVertexOther;
            done = false;
            return;
        }

        if (!aTool.IsBound(aVertex))
        {
            // [BEGIN] Proceed with non-manifold topology (ssv; 14.11.2010)
            if (NMTool.IsActive() && NMTool.IsBound(aVertex))
            {
                myResult = NMTool.Find(aVertex);
                myError = StepToTopoDS_TranslateVertexError.StepToTopoDS_TranslateVertexDone;
                done = true;
                return;
            }
            // [END] Proceed with non-manifold topology (ssv; 14.11.2010)

            // [BEGIN] Proceed with I-DEAS-like STP (ssv; 15.11.2010)
            TCollection_HAsciiString aVName = aVertex.Name();
            if (NMTool.IsActive() && NMTool.IsIDEASCase() && aVName is not null && !aVName.IsEmpty()
                && NMTool.IsBound(aVName.String()))
            {
                myResult = NMTool.Find(aVName.String());
                myError = StepToTopoDS_TranslateVertexError.StepToTopoDS_TranslateVertexDone;
                done = true;
                return;
            }
            // [END] Proceed with I-DEAS-like STP (ssv; 15.11.2010)

            //: S4136    double preci = BRepAPI::Precision();
            StepShape_VertexPoint VP = aVertex as StepShape_VertexPoint;
            StepGeom_Point P = VP.VertexGeometry();
            if (P is null)
            {
                myError = StepToTopoDS_TranslateVertexError.StepToTopoDS_TranslateVertexOther;
                done = false;
                return;
            }

            StepGeom_CartesianPoint P1 = P as StepGeom_CartesianPoint;
            CartesianPoint P2 = StepToGeom.MakeCartesianPoint(P1, theLocalFactors);
            var B = new BrepBuilder();
            Vertex V = new Vertex();
            B.MakeVertex(ref V, P2.Pnt(), Kerf.Engine.Foundation.Precision.Confusion()); //: S4136: preci
            aTool.Bind(aVertex, V);
            // Register Vertex in NM tool (ssv; 14.11.2010)
            if (NMTool.IsActive())
            {
                NMTool.Bind(aVertex, V);
                if (NMTool.IsIDEASCase() && aVName is not null && !aVName.IsEmpty())
                {
                    NMTool.Bind(aVName.String(), V);
                }
            }

            myResult = V;
        }
        else
        {
            myResult = ShapeCasts.Vertex(aTool.Find(aVertex));
        }

        myError = StepToTopoDS_TranslateVertexError.StepToTopoDS_TranslateVertexDone;
        done = true;
    }

    public void Init(StepShape_Vertex aVertex, StepToTopoDS_Tool aTool, StepToTopoDS_NMTool NMTool)
        => Init(aVertex, aTool, NMTool, new StepData_Factors());

    /// <summary>The vertex, frozen.</summary>
    /// <exception cref="NotDoneException">The translation wasn't done.</exception>
    public Shape Value()
    {
        Shape aShape = InternalValue();
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!done, "StepToTopoDS_TranslateVertex::Value() - no result");
        return myResult;
    }

    public StepToTopoDS_TranslateVertexError Error() => myError;
}
