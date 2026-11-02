// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSweptSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSweptSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>The mapping of SweptSurface from Geom to a STEP swept_surface (linear extrusion or revolution).</summary>
public sealed class GeomToStep_MakeSweptSurface : GeomToStep_Root
{
    private StepGeom_SweptSurface theSweptSurface;

    //=============================================================================
    // Creation d' une SweptSurface de prostep a partir d' une
    // SweptSurface de Geom
    //=============================================================================

    /// <summary>A swept surface.</summary>
    public GeomToStep_MakeSweptSurface(Geom_SweptSurface S, in StepData_Factors theLocalFactors)
    {
        done = true;
        if (S is Geom_SurfaceOfLinearExtrusion)
        {
            Geom_SurfaceOfLinearExtrusion Sur = S as Geom_SurfaceOfLinearExtrusion;
            GeomToStep_MakeSurfaceOfLinearExtrusion MkLinear = new GeomToStep_MakeSurfaceOfLinearExtrusion(Sur, theLocalFactors);
            theSweptSurface = MkLinear.Value();
        }
        else if (S is Geom_SurfaceOfRevolution)
        {
            Geom_SurfaceOfRevolution Sur = S as Geom_SurfaceOfRevolution;
            GeomToStep_MakeSurfaceOfRevolution MkRevol = new GeomToStep_MakeSurfaceOfRevolution(Sur, theLocalFactors);
            theSweptSurface = MkRevol.Value();
        }
        else
        {
            done = false;
        }
    }

    /// <summary>A swept surface (default factors).</summary>
    public GeomToStep_MakeSweptSurface(Geom_SweptSurface S) : this(S, new StepData_Factors())
    {
    }

    /// <summary>The swept_surface made.</summary>
    public StepGeom_SweptSurface Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeSweptSurface::Value() - no result");
        return theSweptSurface;
    }
}
