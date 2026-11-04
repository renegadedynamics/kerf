// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSurfaceOfRevolution.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSurfaceOfRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of SurfaceOfRevolution from Geom to a STEP surface_of_revolution.</summary>
internal sealed class MakeSurfaceOfRevolution : GeometryRoot
{
    private StepSurfaceOfRevolution _surfaceOfRevolution;

    //=============================================================================
    // Creation d' une surface_of_revolution de prostep a partir d' une
    // SurfaceOfRevolution de Geom
    //=============================================================================

    /// <summary>A surface of revolution: its basis curve and its axis.</summary>
    public MakeSurfaceOfRevolution(RevolutionSurface s, in Factors localFactors)
    {
        StepSurfaceOfRevolution surf;
        StepCurve sweptCurve;
        StepAxis1Placement axisPosition;

        MakeCurve mkSwept = new MakeCurve(s.BasisCurve, localFactors);
        MakeAxis1Placement mkAxis1 = new MakeAxis1Placement(s.Axis, localFactors);
        sweptCurve = mkSwept.Value();
        axisPosition = mkAxis1.Value();
        surf = new StepSurfaceOfRevolution();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        surf.Init(name, sweptCurve, axisPosition);
        _surfaceOfRevolution = surf;
        _done = true;
    }

    /// <summary>A surface of revolution (default factors).</summary>
    public MakeSurfaceOfRevolution(RevolutionSurface s) : this(s, new Factors())
    {
    }

    /// <summary>The surface_of_revolution made.</summary>
    public StepSurfaceOfRevolution Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeSurfaceOfRevolution::Value() - no result");
        return _surfaceOfRevolution;
    }
}
