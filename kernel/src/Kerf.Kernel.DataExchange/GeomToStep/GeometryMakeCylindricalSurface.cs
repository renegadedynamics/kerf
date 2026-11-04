// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCylindricalSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeCylindricalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.FromGeometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of CylindricalSurface from Geom to a STEP cylindrical_surface.</summary>
internal sealed class GeometryMakeCylindricalSurface : GeometryRoot
{
    private StepCylindricalSurface _cylindricalSurface;

    //=============================================================================
    // Creation d' une conical_surface de prostep a partir d' une
    // CylindricalSurface de Geom
    //=============================================================================

    /// <summary>A cylinder, its position and radius divided by the length factor.</summary>
    public GeometryMakeCylindricalSurface(CylindricalSurface cs, in Factors localFactors)
    {
        StepCylindricalSurface cSstep;
        StepAxis2Placement3d position;
        double radius;

        Axis2Placement3dMaker mkAxis2 = new Axis2Placement3dMaker(cs.Position, localFactors);
        position = mkAxis2.Value();
        radius = cs.Radius;
        cSstep = new StepCylindricalSurface();
        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        cSstep.Init(name, position, radius / localFactors.LengthFactor());
        _cylindricalSurface = cSstep;
        _done = true;
    }

    /// <summary>A cylinder (default factors).</summary>
    public GeometryMakeCylindricalSurface(CylindricalSurface cs) : this(cs, new Factors())
    {
    }

    /// <summary>The cylindrical_surface made.</summary>
    public StepCylindricalSurface Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeCylindricalSurface::Value() - no result");
        return _cylindricalSurface;
    }
}
