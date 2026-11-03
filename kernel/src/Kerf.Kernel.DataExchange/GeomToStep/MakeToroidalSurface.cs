// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeToroidalSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeToroidalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.FromGeometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of ToroidalSurface from Geom to a STEP toroidal_surface.</summary>
public sealed class MakeToroidalSurface : GeometryRoot
{
    private StepToroidalSurface _toroidalSurface;

    //=============================================================================
    // Creation d' une toroidal_surface de prostep a partir d' une ToroidalSurface
    // de Geom
    //=============================================================================

    /// <summary>A torus, its position and radii divided by the length factor.</summary>
    public MakeToroidalSurface(ToroidalSurface s, in Factors localFactors)
    {
        StepToroidalSurface surf;
        StepAxis2Placement3d position;
        double majorRadius, minorRadius;

        Axis2Placement3dMaker mkAxis2 = new Axis2Placement3dMaker(s.Position, localFactors);
        position = mkAxis2.Value();
        majorRadius = s.MajorRadius;
        minorRadius = s.MinorRadius;
        surf = new StepToroidalSurface();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        double fact = localFactors.LengthFactor();
        surf.Init(name, position, majorRadius / fact, minorRadius / fact);
        _toroidalSurface = surf;
        _done = true;
    }

    /// <summary>A torus (default factors).</summary>
    public MakeToroidalSurface(ToroidalSurface s) : this(s, new Factors())
    {
    }

    /// <summary>The toroidal_surface made.</summary>
    public StepToroidalSurface Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeToroidalSurface::Value() - no result");
        return _toroidalSurface;
    }
}
