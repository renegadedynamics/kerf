// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.StepData_Logical;

namespace Occt;

/// <summary>
/// The mapping of Surface from Geom to a STEP surface: bounded, elementary, swept and offset surfaces.
/// </summary>
public sealed class GeomToStep_MakeSurface : GeomToStep_Root
{
    private StepGeom_Surface theSurface;

    //=============================================================================
    // Creation d' une Surface de prostep a partir d' une Surface de Geom
    //=============================================================================

    /// <summary>A surface.</summary>
    public GeomToStep_MakeSurface(Geom_Surface S, in StepData_Factors theLocalFactors)
    {
        done = true;
        if (S is Geom_BoundedSurface)
        {
            Geom_BoundedSurface S1 = S as Geom_BoundedSurface;
            GeomToStep_MakeBoundedSurface MkBoundedS = new GeomToStep_MakeBoundedSurface(S1, theLocalFactors);
            theSurface = MkBoundedS.Value();
        }
        else if (S is Geom_ElementarySurface)
        {
            Geom_ElementarySurface S1 = S as Geom_ElementarySurface;
            GeomToStep_MakeElementarySurface MkElementaryS = new GeomToStep_MakeElementarySurface(S1, theLocalFactors);
            theSurface = MkElementaryS.Value();
        }
        else if (S is Geom_SweptSurface)
        {
            Geom_SweptSurface S1 = S as Geom_SweptSurface;
            GeomToStep_MakeSweptSurface MkSwept = new GeomToStep_MakeSweptSurface(S1, theLocalFactors);
            theSurface = MkSwept.Value();
        }
        else if (S is Geom_OffsetSurface)
        {
            Geom_OffsetSurface S1 = S as Geom_OffsetSurface;
            GeomToStep_MakeSurface MkBasis = new GeomToStep_MakeSurface(S1.BasisSurface(), theLocalFactors);
            done = MkBasis.IsDone();
            if (!done)
            {
                return;
            }
            StepGeom_OffsetSurface Surf = new StepGeom_OffsetSurface();
            Surf.Init(new TCollection_HAsciiString(""), MkBasis.Value(), S1.Offset() / theLocalFactors.LengthFactor(), StepData_LFalse);
            theSurface = Surf;
        }
        else
        {
            done = false;
            // OCCT_DEBUG output left out
        }
    }

    /// <summary>A surface (default factors).</summary>
    public GeomToStep_MakeSurface(Geom_Surface S) : this(S, new StepData_Factors())
    {
    }

    /// <summary>The surface made.</summary>
    public StepGeom_Surface Value()
    {
        StdFail_NotDone_Raise_if(!done, "GeomToStep_MakeSurface::Value() - no result");
        return theSurface;
    }
}
