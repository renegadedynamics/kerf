// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeAxis2Placement2d.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeAxis2Placement2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Geometry;

namespace Occt;

/// <summary>The mapping of Ax2 and Ax22d from gp to a STEP axis2_placement_2d.</summary>
public sealed class GeomToStep_MakeAxis2Placement2d : GeomToStep_Root
{
    private StepGeom_Axis2Placement2d theAxis2Placement2d;

    //=============================================================================
    // Creation d' un axis2_placement_2d de prostep a partir d' un Ax2 de gp
    //=============================================================================

    /// <summary>
    /// An Ax2: its location (3D, divided by the length factor) and its main direction (as OCCT writes them, a 3D
    /// placement in a 2D entity).
    /// </summary>
    public GeomToStep_MakeAxis2Placement2d(in Frame3D A, in StepData_Factors theLocalFactors)
    {
        StepGeom_Axis2Placement2d Axe;
        StepGeom_CartesianPoint P;
        StepGeom_Direction D;

        // OCCT: the 3D location and main direction, in an axis2_placement_2d (occt-issues.md, 8D)
        GeomToStep_MakeCartesianPoint MkPoint = new GeomToStep_MakeCartesianPoint(A.Location(), theLocalFactors.LengthFactor());
        GeomToStep_MakeDirection MkDir = new GeomToStep_MakeDirection(A.Direction());

        P = MkPoint.Value();
        D = MkDir.Value();

        Axe = new StepGeom_Axis2Placement2d();
        Axe.SetLocation(P);
        Axe.SetRefDirection(D);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Axe.SetName(name);
        theAxis2Placement2d = Axe;
        done = true;
    }

    /// <summary>An Ax2 (default factors).</summary>
    public GeomToStep_MakeAxis2Placement2d(in Frame3D A) : this(A, new StepData_Factors())
    {
    }

    //=============================================================================
    // Creation d' un axis2_placement_2d de prostep a partir d' un Ax22d de gp
    //=============================================================================

    /// <summary>An Ax22d: its location and X direction.</summary>
    public GeomToStep_MakeAxis2Placement2d(in Frame2D A, in StepData_Factors theLocalFactors)
    {
        StepGeom_Axis2Placement2d Axe;
        StepGeom_CartesianPoint P;
        StepGeom_Direction D1;

        GeomToStep_MakeCartesianPoint MkPoint = new GeomToStep_MakeCartesianPoint(A.Location(), theLocalFactors.LengthFactor());
        GeomToStep_MakeDirection MkDir = new GeomToStep_MakeDirection(A.XDirection());

        P = MkPoint.Value();
        D1 = MkDir.Value();

        Axe = new StepGeom_Axis2Placement2d();
        Axe.SetLocation(P);
        Axe.SetRefDirection(D1);
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Axe.SetName(name);
        theAxis2Placement2d = Axe;
        done = true;
    }

    /// <summary>An Ax22d.</summary>
    public GeomToStep_MakeAxis2Placement2d(in Frame2D A) : this(A, new StepData_Factors())
    {
    }

    // (OCCT's commented-out constructor from a Geom_Axis2Placement left out)

    /// <summary>The axis2_placement_2d made.</summary>
    public StepGeom_Axis2Placement2d Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeAxis2Placement2d::Value() - no result");
        return theAxis2Placement2d;
    }
}
