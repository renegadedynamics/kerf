// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSphericalSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeSphericalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.FromGeometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of SphericalSurface from Geom to a STEP spherical_surface.</summary>
internal sealed class MakeSphericalSurface : GeometryRoot
{
    private StepSphericalSurface _sphericalSurface;

    //=============================================================================
    // Creation d' une conical_surface de prostep a partir d' une SphericalSurface
    // de Geom
    //=============================================================================

    /// <summary>A sphere, its position and radius divided by the length factor.</summary>
    public MakeSphericalSurface(SphericalSurface s, in Factors localFactors)
    {
        StepSphericalSurface surf;
        StepAxis2Placement3d position;
        double radius;

        Axis2Placement3dMaker mkAxis2 = new Axis2Placement3dMaker(s.Position, localFactors);
        position = mkAxis2.Value();
        radius = s.Radius;
        surf = new StepSphericalSurface();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        surf.Init(name, position, radius / localFactors.LengthFactor());
        _sphericalSurface = surf;
        _done = true;
    }

    /// <summary>A sphere (default factors).</summary>
    public MakeSphericalSurface(SphericalSurface s) : this(s, new Factors())
    {
    }

    /// <summary>The spherical_surface made.</summary>
    public StepSphericalSurface Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeSphericalSurface::Value() - no result");
        return _sphericalSurface;
    }
}
