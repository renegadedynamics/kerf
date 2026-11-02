// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSphericalSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSphericalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The mapping of SphericalSurface from Geom to a STEP spherical_surface.</summary>
public sealed class GeomToStep_MakeSphericalSurface : GeomToStep_Root
{
    private StepGeom_SphericalSurface theSphericalSurface;

    //=============================================================================
    // Creation d' une conical_surface de prostep a partir d' une SphericalSurface
    // de Geom
    //=============================================================================

    /// <summary>A sphere, its position and radius divided by the length factor.</summary>
    public GeomToStep_MakeSphericalSurface(Geom_SphericalSurface S, in StepData_Factors theLocalFactors)
    {
        StepGeom_SphericalSurface Surf;
        StepGeom_Axis2Placement3d aPosition;
        double aRadius;

        GeomToStep_MakeAxis2Placement3d MkAxis2 = new GeomToStep_MakeAxis2Placement3d(S.Position(), theLocalFactors);
        aPosition = MkAxis2.Value();
        aRadius = S.Radius();
        Surf = new StepGeom_SphericalSurface();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        Surf.Init(name, aPosition, aRadius / theLocalFactors.LengthFactor());
        theSphericalSurface = Surf;
        done = true;
    }

    /// <summary>A sphere (default factors).</summary>
    public GeomToStep_MakeSphericalSurface(Geom_SphericalSurface S) : this(S, new StepData_Factors())
    {
    }

    /// <summary>The spherical_surface made.</summary>
    public StepGeom_SphericalSurface Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeSphericalSurface::Value() - no result");
        return theSphericalSurface;
    }
}
