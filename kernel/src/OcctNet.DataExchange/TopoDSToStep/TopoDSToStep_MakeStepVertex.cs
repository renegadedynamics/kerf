// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepVertex.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.TopoDSToStep_MakeVertexError;

namespace Occt;

/// <summary>Maps a TopoDS_Vertex to a STEP vertex_point.</summary>
public sealed class TopoDSToStep_MakeStepVertex : TopoDSToStep_Root
{
    private StepShape_TopologicalRepresentationItem myResult;
    private TopoDSToStep_MakeVertexError myError;

    public TopoDSToStep_MakeStepVertex()
    {
        myError = TopoDSToStep_VertexOther;
        done = false;
    }

    public TopoDSToStep_MakeStepVertex(in TopoDS_Vertex V, TopoDSToStep_Tool T, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors)
    {
        done = false;
        Init(V, T, FP, theLocalFactors);
    }

    public TopoDSToStep_MakeStepVertex(in TopoDS_Vertex V, TopoDSToStep_Tool T, Transfer_FinderProcess FP)
        : this(V, T, FP, new StepData_Factors())
    {
    }

    public void Init(in TopoDS_Vertex aVertex, TopoDSToStep_Tool aTool, Transfer_FinderProcess FP, in StepData_Factors theLocalFactors)
    {
        aTool.SetCurrentVertex(aVertex);

        // [BEGIN] Processing non-manifold topology (ssv; 11.11.2010)
        bool isNMMode = (FP.Model() as StepData_StepModel).InternalParameters.WriteNonmanifold;
        if (isNMMode)
        {
            StepShape_VertexPoint aVP = null;
            TransferBRep_ShapeMapper aSTEPMapper = TransferBRep.ShapeMapper(FP, aVertex);
            if (FP.FindTypedTransient(aSTEPMapper, typeof(StepShape_VertexPoint), ref aVP))
            {
                // Non-manifold topology detected
                myError = TopoDSToStep_VertexOther;
                myResult = aVP;
                done = true;
                return;
            }
        }
        // [END] Processing non-manifold topology (ssv; 11.11.2010)

        if (aTool.IsBound(aVertex))
        {
            myError = TopoDSToStep_VertexOther;
            done = true;
            myResult = aTool.Find(aVertex);
            return;
        }

        gp_Pnt P;

        P = BRep_Tool.Pnt(aVertex);
        var MkPoint = new GeomToStep_MakeCartesianPoint(P, theLocalFactors.LengthFactor());
        StepGeom_CartesianPoint Gpms = MkPoint.Value();
        var Vpms = new StepShape_VertexPoint();
        var aName = new TCollection_HAsciiString("");

        Vpms.Init(aName, Gpms);

        aTool.Bind(aVertex, Vpms);
        myError = TopoDSToStep_VertexDone;
        done = true;
        myResult = Vpms;
    }

    public void Init(in TopoDS_Vertex aVertex, TopoDSToStep_Tool aTool, Transfer_FinderProcess FP)
        => Init(aVertex, aTool, FP, new StepData_Factors());

    /// <exception cref="StdFail_NotDone">The mapping wasn't done.</exception>
    public StepShape_TopologicalRepresentationItem Value()
    {
        StdFail_NotDone_Raise_if(!done, "TopoDSToStep_MakeStepVertex::Value() - no result");
        return myResult;
    }

    public TopoDSToStep_MakeVertexError Error() => myError;
}
