// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepVertex.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_MakeStepVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Export.Geometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Exchange.Step.Export.MakeVertexError;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>Maps a TopoDS_Vertex to a STEP vertex_point.</summary>
internal sealed class MakeStepVertex : ExportRoot
{
    private StepTopologicalRepresentationItem _result;
    private MakeVertexError _error;

    public MakeStepVertex()
    {
        _error = MakeVertexError.VertexOther;
        _done = false;
    }

    public MakeStepVertex(in Vertex v, ExportTool t, FinderProcess fp, in Factors localFactors)
    {
        _done = false;
        Init(v, t, fp, localFactors);
    }

    public MakeStepVertex(in Vertex v, ExportTool t, FinderProcess fp)
        : this(v, t, fp, new Factors())
    {
    }

    public void Init(in Vertex aVertex, ExportTool aTool, FinderProcess fp, in Factors localFactors)
    {
        aTool.SetCurrentVertex(aVertex);

        // [BEGIN] Processing non-manifold topology (ssv; 11.11.2010)
        bool isNMMode = (fp.Model() as StepModel).InternalParameters.WriteNonmanifold;
        if (isNMMode)
        {
            StepVertexPoint vp = null;
            ShapeMapper stepMapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(fp, aVertex);
            if (fp.FindTypedTransient(stepMapper, typeof(StepVertexPoint), ref vp))
            {
                // Non-manifold topology detected
                _error = MakeVertexError.VertexOther;
                _result = vp;
                _done = true;
                return;
            }
        }
        // [END] Processing non-manifold topology (ssv; 11.11.2010)

        if (aTool.IsBound(aVertex))
        {
            _error = MakeVertexError.VertexOther;
            _done = true;
            _result = aTool.Find(aVertex);
            return;
        }

        Point3D p;

        p = Tool.Pnt(aVertex);
        var mkPoint = new MakeCartesianPoint(p, localFactors.LengthFactor());
        StepCartesianPoint gpms = mkPoint.Value();
        var vpms = new StepVertexPoint();
        var name = new TCollection_HAsciiString("");

        vpms.Init(name, gpms);

        aTool.Bind(aVertex, vpms);
        _error = MakeVertexError.VertexDone;
        _done = true;
        _result = vpms;
    }

    public void Init(in Vertex aVertex, ExportTool aTool, FinderProcess fp)
        => Init(aVertex, aTool, fp, new Factors());

    /// <exception cref="NotDoneException">The mapping wasn't done.</exception>
    public StepTopologicalRepresentationItem Value()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_MakeStepVertex::Value() - no result");
        return _result;
    }

    public MakeVertexError Error() => _error;
}
