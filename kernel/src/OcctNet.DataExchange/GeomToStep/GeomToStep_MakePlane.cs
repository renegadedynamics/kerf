// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakePlane.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakePlane.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Geometry;

namespace Occt;

/// <summary>The mapping of Plane from Geom (and Pln from gp) to a STEP plane.</summary>
public sealed class GeomToStep_MakePlane : GeomToStep_Root
{
    private StepGeom_Plane thePlane;

    //=============================================================================
    // Creation d' un plane de prostep a partir d' un Pln de gp
    //=============================================================================

    /// <summary>A plane, its location divided by the length factor.</summary>
    public GeomToStep_MakePlane(in Plane3D P, in StepData_Factors theLocalFactors)
    {
        StepGeom_Plane Plan = new StepGeom_Plane();
        StepGeom_Axis2Placement3d aPosition;

        GeomToStep_MakeAxis2Placement3d MkAxis2 = new GeomToStep_MakeAxis2Placement3d(P.Position(), theLocalFactors);
        aPosition = MkAxis2.Value();
        Plan.SetPosition(aPosition);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Plan.SetName(name);
        thePlane = Plan;
        done = true;
    }

    /// <summary>A plane (default factors).</summary>
    public GeomToStep_MakePlane(in Plane3D P) : this(P, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' un plane de prostep a partir d' un Plane de Geom
    //=============================================================================

    /// <summary>A plane, its location divided by the length factor.</summary>
    public GeomToStep_MakePlane(Geom_Plane Gpln, in StepData_Factors theLocalFactors)
    {
        Plane3D P;
        StepGeom_Plane Plan = new StepGeom_Plane();
        StepGeom_Axis2Placement3d aPosition;

        P = Gpln.Pln();

        GeomToStep_MakeAxis2Placement3d MkAxis2 = new GeomToStep_MakeAxis2Placement3d(P.Position(), theLocalFactors);
        aPosition = MkAxis2.Value();
        Plan.SetPosition(aPosition);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Plan.SetName(name);
        thePlane = Plan;
        done = true;
    }

    /// <summary>A plane (default factors).</summary>
    public GeomToStep_MakePlane(Geom_Plane Gpln) : this(Gpln, new StepData_Factors())
    {
    }

    /// <summary>The plane made.</summary>
    public StepGeom_Plane Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakePlane::Value() - no result");
        return thePlane;
    }
}
