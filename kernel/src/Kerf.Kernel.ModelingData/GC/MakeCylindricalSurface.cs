// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeCylindricalSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GC/GC_MakeCylindricalSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Curves;

/// <summary>
/// Constructions of a Geom_CylindricalSurface: from a gp_Cylinder, an axis (system) and a radius, a circle, three
/// points, or coaxial to a cylinder at a distance or through a point.
/// </summary>
internal sealed class MakeCylindricalSurface : Root
{
    private CylindricalSurface? _theCylinder;

    /// <summary>A cylinder from the gp_Cylinder C.</summary>
    public MakeCylindricalSurface(in Cylinder c)
    {
        _theError = Done;
        _theCylinder = new CylindricalSurface(c);
    }

    /// <summary>A cylinder in the axis system A2; fails with gce_NegativeRadius if Radius is negative.</summary>
    public MakeCylindricalSurface(in Frame3D a2, double radius)
    {
        if (radius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            _theError = Done;
            _theCylinder = new CylindricalSurface(new CoordinateSystem(a2), radius);
        }
    }

    /// <summary>A cylinder of axis A1; fails with the status of gce_MakeCylinder.</summary>
    public MakeCylindricalSurface(in Axis3D a1, double radius)
    {
        MakeCylinder cyl = new MakeCylinder(a1, radius);
        _theError = cyl.Status();
        if (_theError == Done)
        {
            _theCylinder = new CylindricalSurface(cyl.Value());
        }
    }

    /// <summary>The cylinder whose axis and radius are those of Circ.</summary>
    public MakeCylindricalSurface(in Circle3D circ)
    {
        // gce_MakeCylinder(Circ) always succeeds, so its implicit conversion can't throw.
        Cylinder cyl = new MakeCylinder(circ).Value();
        _theCylinder = new CylindricalSurface(cyl);
        _theError = Done;
    }

    /// <summary>A cylinder with axis P1P2 through P3; fails with the status of gce_MakeCylinder.</summary>
    public MakeCylindricalSurface(in Point3D p1, in Point3D p2, in Point3D p3)
    {
        MakeCylinder cyl = new MakeCylinder(p1, p2, p3);
        _theError = cyl.Status();
        if (_theError == Done)
        {
            _theCylinder = new CylindricalSurface(cyl.Value());
        }
    }

    /// <summary>A cylinder coaxial to Cyl with radius |Cyl.Radius() − Dist|.</summary>
    public MakeCylindricalSurface(in Cylinder cyl, double dist)
    {
        _theError = Done;
        double r = Math.Abs(cyl.Radius - dist);
        _theCylinder = new CylindricalSurface(cyl);
        _theCylinder.SetRadius(r);
    }

    /// <summary>A cylinder coaxial to Cyl through Point.</summary>
    public MakeCylindricalSurface(in Cylinder cyl, in Point3D point)
    {
        _theError = Done;
        Cylinder c = cyl;
        Line3D l = new Line3D(c.Axis);
        double r = l.DistanceTo(point);
        c = c.WithRadius(r);
        _theCylinder = new CylindricalSurface(c);
    }

    /// <summary>The constructed cylinder (frozen).</summary>
    /// <exception cref="NotDoneException">No cylinder was constructed.</exception>
    public CylindricalSurface Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "GC_MakeCylindricalSurface::Value() - no result");
        _theCylinder!.Freeze();
        return _theCylinder;
    }

    /// <summary>The constructed cylinder (OCCT's implicit conversion).</summary>
    public static explicit operator CylindricalSurface(MakeCylindricalSurface maker) => maker.Value();
}
