// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeAxis2Placement2d.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeAxis2Placement2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of Ax2 and Ax22d from gp to a STEP axis2_placement_2d.</summary>
internal sealed class MakeAxis2Placement2d : GeometryRoot
{
    private StepAxis2Placement2d _axis2Placement2d;

    //=============================================================================
    // Creation d' un axis2_placement_2d de prostep a partir d' un Ax2 de gp
    //=============================================================================

    /// <summary>
    /// An Ax2: its location (3D, divided by the length factor) and its main direction (as OCCT writes them, a 3D
    /// placement in a 2D entity).
    /// </summary>
    public MakeAxis2Placement2d(in Frame3D a, in Factors localFactors)
    {
        StepAxis2Placement2d axe;
        StepCartesianPoint p;
        StepDirection d;

        // OCCT: the 3D location and main direction, in an axis2_placement_2d (occt-issues.md, 8D)
        MakeCartesianPoint mkPoint = new MakeCartesianPoint(a.Location, localFactors.LengthFactor());
        MakeDirection mkDir = new MakeDirection(a.Direction);

        p = mkPoint.Value();
        d = mkDir.Value();

        axe = new StepAxis2Placement2d();
        axe.SetLocation(p);
        axe.SetRefDirection(d);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        axe.SetName(name);
        _axis2Placement2d = axe;
        _done = true;
    }

    /// <summary>An Ax2 (default factors).</summary>
    public MakeAxis2Placement2d(in Frame3D a) : this(a, new Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis2_placement_2d de prostep a partir d' un Ax22d de gp
    //=============================================================================

    /// <summary>An Ax22d: its location and X direction.</summary>
    public MakeAxis2Placement2d(in Frame2D a, in Factors localFactors)
    {
        StepAxis2Placement2d axe;
        StepCartesianPoint p;
        StepDirection d1;

        MakeCartesianPoint mkPoint = new MakeCartesianPoint(a.Location, localFactors.LengthFactor());
        MakeDirection mkDir = new MakeDirection(a.XDirection);

        p = mkPoint.Value();
        d1 = mkDir.Value();

        axe = new StepAxis2Placement2d();
        axe.SetLocation(p);
        axe.SetRefDirection(d1);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        axe.SetName(name);
        _axis2Placement2d = axe;
        _done = true;
    }

    /// <summary>An Ax22d.</summary>
    public MakeAxis2Placement2d(in Frame2D a) : this(a, new Factors())
    {
    }

    // (OCCT's commented-out constructor from a Geom_Axis2Placement left out)

    /// <summary>The axis2_placement_2d made.</summary>
    public StepAxis2Placement2d Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeAxis2Placement2d::Value() - no result");
        return _axis2Placement2d;
    }
}
