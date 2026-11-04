// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeConicalSurface.hxx
// Port of: src/DataExchange/TKDESTEP/GeomToStep/GeomToStep_MakeConicalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.FromGeometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export.Geometry;

/// <summary>The mapping of ConicalSurface from Geom to a STEP conical_surface.</summary>
internal sealed class GeometryMakeConicalSurface : GeometryRoot
{
    private StepConicalSurface _conicalSurface;

    //=============================================================================
    // Creation d' une conical_surface de prostep a partir d' une ConicalSurface
    // de Geom
    //=============================================================================

    /// <summary>
    /// A cone, its position and reference radius divided by the length factor, its semi-angle by the plane angle
    /// factor; throws Standard_DomainError when the semi-angle is negative, which STEP doesn't allow.
    /// </summary>
    public GeometryMakeConicalSurface(ConicalSurface cs, in Factors localFactors)
    {
        StepConicalSurface cSstep = new StepConicalSurface();
        StepAxis2Placement3d position;
        double radius, semiAngle;

        Axis2Placement3dMaker mkAxis = new Axis2Placement3dMaker(cs.Position, localFactors);
        position = mkAxis.Value();
        radius = cs.RefRadius;
        semiAngle = cs.SemiAngle;
        if (semiAngle < 0.0 || semiAngle > M_PI / 2.0)
        {
            throw new DomainErrorException("Conicalsurface not STEP conformant");
        }

        TCollection_HAsciiString name = new TCollection_HAsciiString("");
        // the semi-angle in the plane angle unit (OCCT wrote radians whatever the unit; Kerf decision 1510)
        cSstep.Init(name, position, radius / localFactors.LengthFactor(), semiAngle / localFactors.PlaneAngleFactor());
        _conicalSurface = cSstep;
        _done = true;
    }

    /// <summary>A cone (default factors).</summary>
    public GeometryMakeConicalSurface(ConicalSurface cs) : this(cs, new Factors())
    {
    }

    /// <summary>The conical_surface made.</summary>
    public StepConicalSurface Value()
    {
        StdFail_NotDone_Raise_if(!_done, "GeomToStep_MakeConicalSurface::Value() - no result");
        return _conicalSurface;
    }
}
