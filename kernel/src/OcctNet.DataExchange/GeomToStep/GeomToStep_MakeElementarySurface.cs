// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeElementarySurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeElementarySurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The mapping of ElementarySurface from Geom to a STEP elementary_surface (cylinder, cone, sphere, torus, plane).
/// </summary>
public sealed class GeomToStep_MakeElementarySurface : GeomToStep_Root
{
    private StepGeom_ElementarySurface theElementarySurface;

    //=============================================================================
    // Creation d' une ElementarySurface de prostep a partir d' une
    // ElementarySurface de Geom
    //=============================================================================

    /// <summary>An elementary surface.</summary>
    public GeomToStep_MakeElementarySurface(ElementarySurface S, in StepData_Factors theLocalFactors)
    {
        done = true;
        if (S is CylindricalSurface)
        {
            CylindricalSurface Sur = S as CylindricalSurface;
            GeomToStep_MakeCylindricalSurface MkCylindrical = new GeomToStep_MakeCylindricalSurface(Sur, theLocalFactors);
            theElementarySurface = MkCylindrical.Value();
        }
        else if (S is ConicalSurface)
        {
            ConicalSurface Sur = S as ConicalSurface;
            GeomToStep_MakeConicalSurface MkConical = new GeomToStep_MakeConicalSurface(Sur, theLocalFactors);
            theElementarySurface = MkConical.Value();
        }
        else if (S is SphericalSurface)
        {
            SphericalSurface Sur = S as SphericalSurface;
            GeomToStep_MakeSphericalSurface MkSpherical = new GeomToStep_MakeSphericalSurface(Sur, theLocalFactors);
            theElementarySurface = MkSpherical.Value();
        }
        else if (S is ToroidalSurface)
        {
            ToroidalSurface Sur = S as ToroidalSurface;
            GeomToStep_MakeToroidalSurface MkToroidal = new GeomToStep_MakeToroidalSurface(Sur, theLocalFactors);
            theElementarySurface = MkToroidal.Value();
        }
        else if (S is PlaneSurface)
        {
            PlaneSurface Sur = S as PlaneSurface;
            GeomToStep_MakePlane MkPlane = new GeomToStep_MakePlane(Sur, theLocalFactors);
            theElementarySurface = MkPlane.Value();
        }
        else
        {
            done = false;
        }
    }

    /// <summary>An elementary surface (default factors).</summary>
    public GeomToStep_MakeElementarySurface(ElementarySurface S) : this(S, new StepData_Factors())
    {
    }

    /// <summary>The elementary_surface made.</summary>
    public StepGeom_ElementarySurface Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeElementarySurface::Value() - no result");
        return theElementarySurface;
    }
}
