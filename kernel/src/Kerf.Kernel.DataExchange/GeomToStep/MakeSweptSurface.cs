// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSweptSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSweptSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of SweptSurface from Geom to a STEP swept_surface (linear extrusion or revolution).</summary>
internal sealed class MakeSweptSurface : GeometryRoot
{
    private StepSweptSurface _sweptSurface;

    //=============================================================================
    // Creation d' une SweptSurface de prostep a partir d' une
    // SweptSurface de Geom
    //=============================================================================

    /// <summary>A swept surface.</summary>
    public MakeSweptSurface(SweptSurface s, in Factors localFactors)
    {
        _done = true;
        if (s is ExtrusionSurface)
        {
            ExtrusionSurface sur = s as ExtrusionSurface;
            MakeSurfaceOfLinearExtrusion mkLinear = new MakeSurfaceOfLinearExtrusion(sur, localFactors);
            _sweptSurface = mkLinear.Value();
        }
        else if (s is RevolutionSurface)
        {
            RevolutionSurface sur = s as RevolutionSurface;
            MakeSurfaceOfRevolution mkRevol = new MakeSurfaceOfRevolution(sur, localFactors);
            _sweptSurface = mkRevol.Value();
        }
        else
        {
            _done = false;
        }
    }

    /// <summary>A swept surface (default factors).</summary>
    public MakeSweptSurface(SweptSurface s) : this(s, new Factors())
    {
    }

    /// <summary>The swept_surface made.</summary>
    public StepSweptSurface Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeSweptSurface::Value() - no result");
        return _sweptSurface;
    }
}
