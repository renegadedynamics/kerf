// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSurfaceOfLinearExtrusion.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSurfaceOfLinearExtrusion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The mapping of SurfaceOfLinearExtrusion from Geom to a STEP surface_of_linear_extrusion.</summary>
public sealed class GeomToStep_MakeSurfaceOfLinearExtrusion : GeomToStep_Root
{
    private StepGeom_SurfaceOfLinearExtrusion theSurfaceOfLinearExtrusion;

    //=============================================================================
    // Creation d' une surface_of_linear_extrusion de prostep a partir d' une
    // SurfaceOfLinearExtrusion de Geom
    //=============================================================================

    /// <summary>
    /// An extrusion: its basis curve, and its unit direction as a vector (magnitude 1 divided by the length factor).
    /// </summary>
    public GeomToStep_MakeSurfaceOfLinearExtrusion(Geom_SurfaceOfLinearExtrusion S, in StepData_Factors theLocalFactors)
    {
        StepGeom_SurfaceOfLinearExtrusion Surf;
        StepGeom_Curve aSweptCurve;
        StepGeom_Vector aExtrusionAxis;

        GeomToStep_MakeCurve MkCurve = new GeomToStep_MakeCurve(S.BasisCurve(), theLocalFactors);
        GeomToStep_MakeVector MkVector = new GeomToStep_MakeVector(new gp_Vec(S.Direction()), theLocalFactors);

        aSweptCurve = MkCurve.Value();
        aExtrusionAxis = MkVector.Value();

        Surf = new StepGeom_SurfaceOfLinearExtrusion();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Surf.Init(name, aSweptCurve, aExtrusionAxis);
        theSurfaceOfLinearExtrusion = Surf;
        done = true;
    }

    /// <summary>An extrusion (default factors).</summary>
    public GeomToStep_MakeSurfaceOfLinearExtrusion(Geom_SurfaceOfLinearExtrusion S) : this(S, new StepData_Factors())
    {
    }

    /// <summary>The surface_of_linear_extrusion made.</summary>
    public StepGeom_SurfaceOfLinearExtrusion Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeSurfaceOfLinearExtrusion::Value() - no result");
        return theSurfaceOfLinearExtrusion;
    }
}
