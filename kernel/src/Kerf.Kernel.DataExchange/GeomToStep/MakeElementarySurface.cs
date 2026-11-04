// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeElementarySurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeElementarySurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>
/// The mapping of ElementarySurface from Geom to a STEP elementary_surface (cylinder, cone, sphere, torus, plane).
/// </summary>
internal sealed class MakeElementarySurface : GeometryRoot
{
    private StepElementarySurface _elementarySurface;

    //=============================================================================
    // Creation d' une ElementarySurface de prostep a partir d' une
    // ElementarySurface de Geom
    //=============================================================================

    /// <summary>An elementary surface.</summary>
    public MakeElementarySurface(ElementarySurface s, in Factors localFactors)
    {
        _done = true;
        if (s is CylindricalSurface)
        {
            CylindricalSurface sur = s as CylindricalSurface;
            GeometryMakeCylindricalSurface mkCylindrical = new GeometryMakeCylindricalSurface(sur, localFactors);
            _elementarySurface = mkCylindrical.Value();
        }
        else if (s is ConicalSurface)
        {
            ConicalSurface sur = s as ConicalSurface;
            GeometryMakeConicalSurface mkConical = new GeometryMakeConicalSurface(sur, localFactors);
            _elementarySurface = mkConical.Value();
        }
        else if (s is SphericalSurface)
        {
            SphericalSurface sur = s as SphericalSurface;
            MakeSphericalSurface mkSpherical = new MakeSphericalSurface(sur, localFactors);
            _elementarySurface = mkSpherical.Value();
        }
        else if (s is ToroidalSurface)
        {
            ToroidalSurface sur = s as ToroidalSurface;
            MakeToroidalSurface mkToroidal = new MakeToroidalSurface(sur, localFactors);
            _elementarySurface = mkToroidal.Value();
        }
        else if (s is PlaneSurface)
        {
            PlaneSurface sur = s as PlaneSurface;
            GeometryMakePlane mkPlane = new GeometryMakePlane(sur, localFactors);
            _elementarySurface = mkPlane.Value();
        }
        else
        {
            _done = false;
        }
    }

    /// <summary>An elementary surface (default factors).</summary>
    public MakeElementarySurface(ElementarySurface s) : this(s, new Factors())
    {
    }

    /// <summary>The elementary_surface made.</summary>
    public StepElementarySurface Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeElementarySurface::Value() - no result");
        return _elementarySurface;
    }
}
