// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSurfaceOfLinearExtrusion.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSurfaceOfLinearExtrusion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of SurfaceOfLinearExtrusion from Geom to a STEP surface_of_linear_extrusion.</summary>
internal sealed class MakeSurfaceOfLinearExtrusion : GeometryRoot
{
    private StepSurfaceOfLinearExtrusion _surfaceOfLinearExtrusion;

    //=============================================================================
    // Creation d' une surface_of_linear_extrusion de prostep a partir d' une
    // SurfaceOfLinearExtrusion de Geom
    //=============================================================================

    /// <summary>
    /// An extrusion: its basis curve, and its unit direction as a vector (magnitude 1 divided by the length factor).
    /// </summary>
    public MakeSurfaceOfLinearExtrusion(ExtrusionSurface s, in Factors localFactors)
    {
        StepSurfaceOfLinearExtrusion surf;
        StepCurve sweptCurve;
        StepVector extrusionAxis;

        MakeCurve mkCurve = new MakeCurve(s.BasisCurve, localFactors);
        MakeVector mkVector = new MakeVector(new Vector3D(s.Direction), localFactors);

        sweptCurve = mkCurve.Value();
        extrusionAxis = mkVector.Value();

        surf = new StepSurfaceOfLinearExtrusion();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        surf.Init(name, sweptCurve, extrusionAxis);
        _surfaceOfLinearExtrusion = surf;
        _done = true;
    }

    /// <summary>An extrusion (default factors).</summary>
    public MakeSurfaceOfLinearExtrusion(ExtrusionSurface s) : this(s, new Factors())
    {
    }

    /// <summary>The surface_of_linear_extrusion made.</summary>
    public StepSurfaceOfLinearExtrusion Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeSurfaceOfLinearExtrusion::Value() - no result");
        return _surfaceOfLinearExtrusion;
    }
}
