// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakePlane.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakePlane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.FromGeometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of Plane from Geom (and Pln from gp) to a STEP plane.</summary>
internal sealed class GeometryMakePlane : GeometryRoot
{
    private StepPlane _plane;

    //=============================================================================
    // Creation d' un plane de prostep a partir d' un Pln de gp
    //=============================================================================

    /// <summary>A plane, its location divided by the length factor.</summary>
    public GeometryMakePlane(in Plane3D p, in Factors localFactors)
    {
        StepPlane plan = new StepPlane();
        StepAxis2Placement3d position;

        Axis2Placement3dMaker mkAxis2 = new Axis2Placement3dMaker(p.Position, localFactors);
        position = mkAxis2.Value();
        plan.SetPosition(position);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        plan.SetName(name);
        _plane = plan;
        _done = true;
    }

    /// <summary>A plane (default factors).</summary>
    public GeometryMakePlane(in Plane3D p) : this(p, new Factors())
    {
    }

    //=============================================================================
    // Creation d' un plane de prostep a partir d' un Plane de Geom
    //=============================================================================

    /// <summary>A plane, its location divided by the length factor.</summary>
    public GeometryMakePlane(PlaneSurface gpln, in Factors localFactors)
    {
        Plane3D p;
        StepPlane plan = new StepPlane();
        StepAxis2Placement3d position;

        p = gpln.Plane;

        Axis2Placement3dMaker mkAxis2 = new Axis2Placement3dMaker(p.Position, localFactors);
        position = mkAxis2.Value();
        plan.SetPosition(position);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        plan.SetName(name);
        _plane = plan;
        _done = true;
    }

    /// <summary>A plane (default factors).</summary>
    public GeometryMakePlane(PlaneSurface gpln) : this(gpln, new Factors())
    {
    }

    /// <summary>The plane made.</summary>
    public StepPlane Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakePlane::Value() - no result");
        return _plane;
    }
}
