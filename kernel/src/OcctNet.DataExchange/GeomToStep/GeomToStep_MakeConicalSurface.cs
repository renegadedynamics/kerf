// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeConicalSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeConicalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Geometry;

namespace Occt;

/// <summary>The mapping of ConicalSurface from Geom to a STEP conical_surface.</summary>
public sealed class GeomToStep_MakeConicalSurface : GeomToStep_Root
{
    private StepGeom_ConicalSurface theConicalSurface;

    //=============================================================================
    // Creation d' une conical_surface de prostep a partir d' une ConicalSurface
    // de Geom
    //=============================================================================

    /// <summary>
    /// A cone, its position and reference radius divided by the length factor (its semi-angle in radians); throws
    /// Standard_DomainError when the semi-angle is negative, which STEP doesn't allow.
    /// </summary>
    public GeomToStep_MakeConicalSurface(ConicalSurface CS, in StepData_Factors theLocalFactors)
    {
        StepGeom_ConicalSurface CSstep = new StepGeom_ConicalSurface();
        StepGeom_Axis2Placement3d aPosition;
        double aRadius, aSemiAngle;

        GeomToStep_MakeAxis2Placement3d MkAxis = new GeomToStep_MakeAxis2Placement3d(CS.Position(), theLocalFactors);
        aPosition = MkAxis.Value();
        aRadius = CS.RefRadius();
        aSemiAngle = CS.SemiAngle();
        if (aSemiAngle < 0.0 || aSemiAngle > M_PI / 2.0)
        {
            throw new DomainErrorException("Conicalsurface not STEP conformant");
        }

        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        // OCCT: the semi-angle in radians whatever the plane angle factor (occt-issues.md, 8D)
        CSstep.Init(name, aPosition, aRadius / theLocalFactors.LengthFactor(), aSemiAngle);
        theConicalSurface = CSstep;
        done = true;
    }

    /// <summary>A cone (default factors).</summary>
    public GeomToStep_MakeConicalSurface(ConicalSurface CS) : this(CS, new StepData_Factors())
    {
    }

    /// <summary>The conical_surface made.</summary>
    public StepGeom_ConicalSurface Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeConicalSurface::Value() - no result");
        return theConicalSurface;
    }
}
