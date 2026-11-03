// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeAxis1Placement.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeAxis1Placement.cxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeAxis1Placement_gen.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The mapping of Axis1Placement from Geom (and AxisPlacement from Geom2d, Ax1 and Ax2d from gp) to a STEP
/// axis1_placement.
/// </summary>
public sealed class GeomToStep_MakeAxis1Placement : GeomToStep_Root
{
    private StepGeom_Axis1Placement theAxis1Placement;

    //=============================================================================
    // Creation d' un axis1_placement de prostep a partir d' un Ax1 de gp
    //=============================================================================

    /// <summary>A 3D axis, its location divided by the length factor.</summary>
    public GeomToStep_MakeAxis1Placement(in Axis3D A, in StepData_Factors theLocalFactors)
    {
        Make(new GeomToStep_MakeCartesianPoint(A.Location(), theLocalFactors.LengthFactor()), new GeomToStep_MakeDirection(A.Direction()));
    }

    /// <summary>A 3D axis (default factors).</summary>
    public GeomToStep_MakeAxis1Placement(in Axis3D A) : this(A, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis1_placement de prostep a partir d' un Ax2d de gp
    //=============================================================================

    /// <summary>A 2D axis.</summary>
    public GeomToStep_MakeAxis1Placement(in Axis2D A, in StepData_Factors theLocalFactors)
    {
        Make(new GeomToStep_MakeCartesianPoint(A.Location(), theLocalFactors.LengthFactor()), new GeomToStep_MakeDirection(A.Direction()));
    }

    /// <summary>A 2D axis.</summary>
    public GeomToStep_MakeAxis1Placement(in Axis2D A) : this(A, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis1_placement de prostep a partir d' un Ax1Placement de
    // Geom
    //=============================================================================

    /// <summary>A 3D axis, its location divided by the length factor.</summary>
    public GeomToStep_MakeAxis1Placement(Axis1Placement Axis1, in StepData_Factors theLocalFactors)
    {
        Axis3D A;
        A = Axis1.Ax1();
        Make(new GeomToStep_MakeCartesianPoint(A.Location(), theLocalFactors.LengthFactor()), new GeomToStep_MakeDirection(A.Direction()));
    }

    /// <summary>A 3D axis (default factors).</summary>
    public GeomToStep_MakeAxis1Placement(Axis1Placement Axis1) : this(Axis1, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis1_placement de prostep a partir d' un AxPlacement de
    // Geom2d
    //=============================================================================

    /// <summary>A 2D axis.</summary>
    public GeomToStep_MakeAxis1Placement(AxisPlacement2 Axis1, in StepData_Factors theLocalFactors)
    {
        Axis2D A;
        A = Axis1.Ax2d();
        Make(new GeomToStep_MakeCartesianPoint(A.Location(), theLocalFactors.LengthFactor()), new GeomToStep_MakeDirection(A.Direction()));
    }

    /// <summary>A 2D axis.</summary>
    public GeomToStep_MakeAxis1Placement(AxisPlacement2 Axis1) : this(Axis1, new StepData_Factors())
    {
    }

    // GeomToStep_MakeAxis1Placement_gen.pxx, the body OCCT includes in each constructor (its point and direction
    // made first)
    private void Make(GeomToStep_MakeCartesianPoint MkPoint, GeomToStep_MakeDirection MkDir)
    {
        StepGeom_Axis1Placement Axe = new StepGeom_Axis1Placement();
        StepGeom_CartesianPoint P;
        StepGeom_Direction D;

        P = MkPoint.Value();
        D = MkDir.Value();
        Axe.SetLocation(P);
        Axe.SetAxis(D);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Axe.SetName(name);
        theAxis1Placement = Axe;
        done = true;
    }

    /// <summary>The axis1_placement made.</summary>
    public StepGeom_Axis1Placement Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeAxis1Placement::Value() - no result");
        return theAxis1Placement;
    }
}
