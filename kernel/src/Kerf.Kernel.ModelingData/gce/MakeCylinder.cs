// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCylinder.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>
/// Constructions of a gp_Cylinder: from an axis system and a radius, coaxial to a cylinder through a point or at a
/// signed offset, from three points, from an axis and a radius, or from a circular base.
/// </summary>
internal sealed class MakeCylinder : Root
{
    private Cylinder3D _theCylinder = new();

    /// <summary>A cylinder from an axis system and a radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public MakeCylinder(in Frame3D a2, double radius)
    {
        if (radius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            _theCylinder = new Cylinder3D(new CoordinateSystem(a2), radius);
            _theError = Done;
        }
    }

    /// <summary>A cylinder from its axis and a radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public MakeCylinder(in Axis3D axis, double radius)
    {
        if (radius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            Direction3D d = axis.Direction;
            Direction3D direc = new();
            double x = d.X;
            double y = d.Y;
            double z = d.Z;
            if (Math.Abs(x) > Constants.Resolution())
            {
                direc = new Direction3D(-y, x, 0.0);
            }
            else if (Math.Abs(y) > Constants.Resolution())
            {
                direc = new Direction3D(0.0, -z, y);
            }
            else if (Math.Abs(z) > Constants.Resolution())
            {
                direc = new Direction3D(z, 0.0, -x);
            }
            _theCylinder = new Cylinder3D(new CoordinateSystem(new Frame3D(axis.Location, d, direc)), radius);
            _theError = Done;
        }
    }

    /// <summary>A cylinder whose axis and radius are those of a circle.</summary>
    public MakeCylinder(in Circle3D circ)
    {
        _theCylinder = new Cylinder3D(new CoordinateSystem(circ.Position), circ.Radius);
        _theError = Done;
    }

    /// <summary>
    /// A cylinder whose axis passes through P1 and P2 and whose radius is the distance from P3 to it; fails with
    /// gce_ConfusedPoints if P1 and P2 coincide.
    /// </summary>
    public MakeCylinder(in Point3D p1, in Point3D p2, in Point3D p3)
    {
        // P1 and P2 define the cylinder axis, the distance from P3 to the axis
        // gives the cylinder radius.
        if (p1.DistanceTo(p2) < Constants.Resolution())
        {
            _theError = ConfusedPoints;
        }
        else
        {
            Direction3D d1 = new(p2.XYZ() - p1.XYZ());
            Direction3D d2 = new();
            double x = d1.X;
            double y = d1.Y;
            double z = d1.Z;
            if (Math.Abs(x) > Constants.Resolution())
            {
                d2 = new Direction3D(-y, x, 0.0);
            }
            else if (Math.Abs(y) > Constants.Resolution())
            {
                d2 = new Direction3D(0.0, -z, y);
            }
            else if (Math.Abs(z) > Constants.Resolution())
            {
                d2 = new Direction3D(z, 0.0, -x);
            }
            _theCylinder = new Cylinder3D(new CoordinateSystem(new Frame3D(p1, d1, d2)), new Line3D(p1, d1).DistanceTo(p3));
            _theError = Done;
        }
    }

    /// <summary>
    /// A cylinder coaxial to Cyl at the signed distance Dist; fails with gce_NegativeRadius if the radius would be
    /// negative.
    /// </summary>
    public MakeCylinder(in Cylinder3D cyl, double dist)
    {
        double rad = cyl.Radius + dist;
        if (rad < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            _theCylinder = cyl;
            _theCylinder = _theCylinder.WithRadius(rad);
            _theError = Done;
        }
    }

    /// <summary>A cylinder coaxial to Cyl through the point P.</summary>
    public MakeCylinder(in Cylinder3D cyl, in Point3D p)
    {
        Line3D l = new(cyl.Axis);
        double rad = l.DistanceTo(p);
        _theCylinder = cyl;
        _theCylinder = _theCylinder.WithRadius(rad);
        _theError = Done;
    }

    /// <summary>The constructed cylinder.</summary>
    /// <exception cref="NotDoneException">No cylinder was constructed.</exception>
    public Cylinder3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakeCylinder::Value() - no result");
        return _theCylinder;
    }

    /// <summary>Alias for Value().</summary>
    public Cylinder3D Operator() => Value();

    /// <summary>The constructed cylinder (OCCT's implicit conversion operator).</summary>
    public static explicit operator Cylinder3D(MakeCylinder maker) => maker.Operator();
}
