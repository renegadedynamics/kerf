// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeAxis1Placement.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeAxis1Placement.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeAxis1Placement_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>
/// The mapping of Axis1Placement from Geom (and AxisPlacement from Geom2d, Ax1 and Ax2d from gp) to a STEP
/// axis1_placement.
/// </summary>
public sealed class MakeAxis1Placement : GeometryRoot
{
    private StepAxis1Placement _axis1Placement;

    //=============================================================================
    // Creation d' un axis1_placement de prostep a partir d' un Ax1 de gp
    //=============================================================================

    /// <summary>A 3D axis, its location divided by the length factor.</summary>
    public MakeAxis1Placement(in Axis3D a, in Factors localFactors)
    {
        Make(new MakeCartesianPoint(a.Location(), localFactors.LengthFactor()), new MakeDirection(a.Direction()));
    }

    /// <summary>A 3D axis (default factors).</summary>
    public MakeAxis1Placement(in Axis3D a) : this(a, new Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis1_placement de prostep a partir d' un Ax2d de gp
    //=============================================================================

    /// <summary>A 2D axis.</summary>
    public MakeAxis1Placement(in Axis2D a, in Factors localFactors)
    {
        Make(new MakeCartesianPoint(a.Location(), localFactors.LengthFactor()), new MakeDirection(a.Direction()));
    }

    /// <summary>A 2D axis.</summary>
    public MakeAxis1Placement(in Axis2D a) : this(a, new Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis1_placement de prostep a partir d' un Ax1Placement de
    // Geom
    //=============================================================================

    /// <summary>A 3D axis, its location divided by the length factor.</summary>
    public MakeAxis1Placement(Axis1Placement axis1, in Factors localFactors)
    {
        Axis3D a;
        a = axis1.Ax1();
        Make(new MakeCartesianPoint(a.Location(), localFactors.LengthFactor()), new MakeDirection(a.Direction()));
    }

    /// <summary>A 3D axis (default factors).</summary>
    public MakeAxis1Placement(Axis1Placement axis1) : this(axis1, new Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis1_placement de prostep a partir d' un AxPlacement de
    // Geom2d
    //=============================================================================

    /// <summary>A 2D axis.</summary>
    public MakeAxis1Placement(AxisPlacement2 axis1, in Factors localFactors)
    {
        Axis2D a;
        a = axis1.Ax2d();
        Make(new MakeCartesianPoint(a.Location(), localFactors.LengthFactor()), new MakeDirection(a.Direction()));
    }

    /// <summary>A 2D axis.</summary>
    public MakeAxis1Placement(AxisPlacement2 axis1) : this(axis1, new Factors())
    {
    }

    // GeomToStep_MakeAxis1Placement_gen.pxx, the body OCCT includes in each constructor (its point and direction
    // made first)
    private void Make(MakeCartesianPoint mkPoint, MakeDirection mkDir)
    {
        StepAxis1Placement axe = new StepAxis1Placement();
        StepCartesianPoint p;
        StepDirection d;

        p = mkPoint.Value();
        d = mkDir.Value();
        axe.SetLocation(p);
        axe.SetAxis(d);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        axe.SetName(name);
        _axis1Placement = axe;
        _done = true;
    }

    /// <summary>The axis1_placement made.</summary>
    public StepAxis1Placement Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeAxis1Placement::Value() - no result");
        return _axis1Placement;
    }
}
