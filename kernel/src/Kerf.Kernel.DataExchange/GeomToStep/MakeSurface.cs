// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Exchange.Step.Data.Logical;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>
/// The mapping of Surface from Geom to a STEP surface: bounded, elementary, swept and offset surfaces.
/// </summary>
internal sealed class MakeSurface : GeometryRoot
{
    private StepSurface _surface;

    //=============================================================================
    // Creation d' une Surface de prostep a partir d' une Surface de Geom
    //=============================================================================

    /// <summary>A surface.</summary>
    public MakeSurface(Surface s, in Factors localFactors)
    {
        _done = true;
        if (s is BoundedSurface)
        {
            BoundedSurface s1 = s as BoundedSurface;
            MakeBoundedSurface mkBoundedS = new MakeBoundedSurface(s1, localFactors);
            _surface = mkBoundedS.Value();
        }
        else if (s is ElementarySurface)
        {
            ElementarySurface s1 = s as ElementarySurface;
            MakeElementarySurface mkElementaryS = new MakeElementarySurface(s1, localFactors);
            _surface = mkElementaryS.Value();
        }
        else if (s is SweptSurface)
        {
            SweptSurface s1 = s as SweptSurface;
            MakeSweptSurface mkSwept = new MakeSweptSurface(s1, localFactors);
            _surface = mkSwept.Value();
        }
        else if (s is OffsetSurface)
        {
            OffsetSurface s1 = s as OffsetSurface;
            MakeSurface mkBasis = new MakeSurface(s1.BasisSurface, localFactors);
            _done = mkBasis.IsDone;
            if (!_done)
            {
                return;
            }
            StepOffsetSurface surf = new StepOffsetSurface();
            surf.Init(new TCollection_HAsciiString(""), mkBasis.Value(), s1.Offset / localFactors.LengthFactor(), Logical.LFalse);
            _surface = surf;
        }
        else
        {
            _done = false;
            // OCCT_DEBUG output left out
        }
    }

    /// <summary>A surface (default factors).</summary>
    public MakeSurface(Surface s) : this(s, new Factors())
    {
    }

    /// <summary>The surface made.</summary>
    public StepSurface Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeSurface::Value() - no result");
        return _surface;
    }
}
