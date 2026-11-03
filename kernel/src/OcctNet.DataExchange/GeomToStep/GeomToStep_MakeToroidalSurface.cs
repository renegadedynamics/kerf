// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeToroidalSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeToroidalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Geometry;

namespace Occt;

/// <summary>The mapping of ToroidalSurface from Geom to a STEP toroidal_surface.</summary>
public sealed class GeomToStep_MakeToroidalSurface : GeomToStep_Root
{
    private StepGeom_ToroidalSurface theToroidalSurface;

    //=============================================================================
    // Creation d' une toroidal_surface de prostep a partir d' une ToroidalSurface
    // de Geom
    //=============================================================================

    /// <summary>A torus, its position and radii divided by the length factor.</summary>
    public GeomToStep_MakeToroidalSurface(ToroidalSurface S, in StepData_Factors theLocalFactors)
    {
        StepGeom_ToroidalSurface Surf;
        StepGeom_Axis2Placement3d aPosition;
        double aMajorRadius, aMinorRadius;

        GeomToStep_MakeAxis2Placement3d MkAxis2 = new GeomToStep_MakeAxis2Placement3d(S.Position(), theLocalFactors);
        aPosition = MkAxis2.Value();
        aMajorRadius = S.MajorRadius();
        aMinorRadius = S.MinorRadius();
        Surf = new StepGeom_ToroidalSurface();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        double fact = theLocalFactors.LengthFactor();
        Surf.Init(name, aPosition, aMajorRadius / fact, aMinorRadius / fact);
        theToroidalSurface = Surf;
        done = true;
    }

    /// <summary>A torus (default factors).</summary>
    public GeomToStep_MakeToroidalSurface(ToroidalSurface S) : this(S, new StepData_Factors())
    {
    }

    /// <summary>The toroidal_surface made.</summary>
    public StepGeom_ToroidalSurface Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeToroidalSurface::Value() - no result");
        return theToroidalSurface;
    }
}
