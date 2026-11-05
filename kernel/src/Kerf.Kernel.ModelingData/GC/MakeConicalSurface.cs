// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeConicalSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeConicalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a Geom_ConicalSurface: from an axis system, a semi-angle and a radius, a gp_Cone, four points, or
/// two points and two radii.
/// </summary>
internal sealed class MakeConicalSurface : Root
{
    private ConicalSurface? _theCone;

    /// <summary>
    /// A cone in the axis system A2 with semi-angle Ang and reference radius Radius; fails with gce_NegativeRadius
    /// if Radius is negative and with gce_BadAngle unless Resolution &lt; Ang &lt; π/2 − Resolution.
    /// </summary>
    public MakeConicalSurface(in Frame3D a2, double ang, double radius)
    {
        if (radius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else if (ang <= Constants.Resolution() || ang >= M_PI / 2.0 - Constants.Resolution())
        {
            _theError = BadAngle;
        }
        else
        {
            _theError = Done;
            _theCone = new ConicalSurface(new CoordinateSystem(a2), ang, radius);
        }
    }

    /// <summary>A cone from the gp_Cone C.</summary>
    public MakeConicalSurface(in Cone3D c)
    {
        _theError = Done;
        _theCone = new ConicalSurface(c);
    }

    /// <summary>A cone with axis P1P2 through P3 and P4; fails with the status of gce_MakeCone.</summary>
    public MakeConicalSurface(in Point3D p1, in Point3D p2, in Point3D p3, in Point3D p4)
    {
        MakeCone c = new MakeCone(p1, p2, p3, p4);
        _theError = c.Status();
        if (_theError == Done)
        {
            _theCone = new ConicalSurface(c.Value());
        }
    }

    /// <summary>A cone with axis P1P2 and radii R1 at P1 and R2 at P2; fails with the status of gce_MakeCone.</summary>
    public MakeConicalSurface(in Point3D p1, in Point3D p2, double r1, double r2)
    {
        MakeCone c = new MakeCone(p1, p2, r1, r2);
        _theError = c.Status();
        if (_theError == Done)
        {
            _theCone = new ConicalSurface(c.Value());
        }
    }

    /// <summary>The constructed cone (frozen).</summary>
    /// <exception cref="NotDoneException">No cone was constructed.</exception>
    public ConicalSurface Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeConicalSurface::Value() - no result");
        _theCone!.Freeze();
        return _theCone;
    }

    /// <summary>The constructed cone (OCCT's implicit conversion).</summary>
    public static explicit operator ConicalSurface(MakeConicalSurface maker) => maker.Value();
}
