// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSurfaceOfRevolution.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSurfaceOfRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The mapping of SurfaceOfRevolution from Geom to a STEP surface_of_revolution.</summary>
public sealed class GeomToStep_MakeSurfaceOfRevolution : GeomToStep_Root
{
    private StepGeom_SurfaceOfRevolution theSurfaceOfRevolution;

    //=============================================================================
    // Creation d' une surface_of_revolution de prostep a partir d' une
    // SurfaceOfRevolution de Geom
    //=============================================================================

    /// <summary>A surface of revolution: its basis curve and its axis.</summary>
    public GeomToStep_MakeSurfaceOfRevolution(Geom_SurfaceOfRevolution S, in StepData_Factors theLocalFactors)
    {
        StepGeom_SurfaceOfRevolution Surf;
        StepGeom_Curve aSweptCurve;
        StepGeom_Axis1Placement aAxisPosition;

        GeomToStep_MakeCurve MkSwept = new GeomToStep_MakeCurve(S.BasisCurve(), theLocalFactors);
        GeomToStep_MakeAxis1Placement MkAxis1 = new GeomToStep_MakeAxis1Placement(S.Axis(), theLocalFactors);
        aSweptCurve = MkSwept.Value();
        aAxisPosition = MkAxis1.Value();
        Surf = new StepGeom_SurfaceOfRevolution();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Surf.Init(name, aSweptCurve, aAxisPosition);
        theSurfaceOfRevolution = Surf;
        done = true;
    }

    /// <summary>A surface of revolution (default factors).</summary>
    public GeomToStep_MakeSurfaceOfRevolution(Geom_SurfaceOfRevolution S) : this(S, new StepData_Factors())
    {
    }

    /// <summary>The surface_of_revolution made.</summary>
    public StepGeom_SurfaceOfRevolution Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeSurfaceOfRevolution::Value() - no result");
        return theSurfaceOfRevolution;
    }
}
