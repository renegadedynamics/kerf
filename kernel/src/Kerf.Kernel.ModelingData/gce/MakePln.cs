// Port of: src/ModelingData/TKGeomBase/gce/gce_MakePln.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakePln.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>
/// Constructions of a gp_Pln: from an axis system, a point and a normal, a cartesian equation, parallel to a plane
/// through a point or at a signed distance, through three points, through a point normal to the direction to a second
/// point, or through the location of an axis normal to it.
/// </summary>
internal sealed class MakePln : Root
{
    private Plane3D _thePln = new();

    /// <summary>
    /// A plane whose coordinate system is A2: its direction is the normal, its location the origin, and its X and Y
    /// directions parametrize the plane.
    /// </summary>
    public MakePln(in Frame3D a2)
    {
        _thePln = new Plane3D(new CoordinateSystem(a2));
        _theError = Done;
    }

    /// <summary>A plane through P (its location) with normal V.</summary>
    public MakePln(in Point3D p, in Direction3D v)
    {
        _thePln = new Plane3D(p, v);
        _theError = Done;
    }

    /// <summary>A plane through P1, normal to P1P2; fails with gce_ConfusedPoints if they coincide.</summary>
    public MakePln(in Point3D p1, in Point3D p2)
    {
        if (p1.DistanceTo(p2) <= Constants.Resolution())
        {
            _theError = ConfusedPoints;
        }
        else
        {
            Direction3D dir = new(p2.XYZ() - p1.XYZ());
            _thePln = new Plane3D(p1, dir);
            _theError = Done;
        }
    }

    /// <summary>
    /// The plane A * X + B * Y + C * Z + D = 0; fails with gce_BadEquation if A*A + B*B + C*C &lt;= gp::Resolution().
    /// </summary>
    public MakePln(double a, double b, double c, double d)
    {
        if (a * a + b * b + c * c <= Constants.Resolution())
        {
            _theError = BadEquation;
        }
        else
        {
            _thePln = new Plane3D(a, b, c, d);
            _theError = Done;
        }
    }

    /// <summary>A plane through three points; fails with gce_ColinearPoints if they are collinear.</summary>
    public MakePln(in Point3D p1, in Point3D p2, in Point3D p3)
    {
        Xyz v1 = p2.XYZ() - p1.XYZ();
        Xyz v2 = p3.XYZ() - p1.XYZ();
        Xyz norm = v1.Crossed(v2);
        if (norm.Modulus() < Constants.Resolution())
        {
            _theError = ColinearPoints;
        }
        else
        {
            Direction3D dNorm = new(norm);
            Direction3D dx = new(v1);
            _thePln = new Plane3D(new CoordinateSystem(p1, dNorm, dx));
            _theError = Done;
        }
    }

    /// <summary>A plane parallel to Pl at the signed distance Dist (along its normal when positive).</summary>
    public MakePln(in Plane3D pl, double dist)
    {
        Point3D center = new(pl.Location.XYZ() + dist * pl.Axis.Direction.XYZ());
        _thePln = new Plane3D(new CoordinateSystem(center, pl.Axis.Direction, pl.XAxis.Direction));
        _theError = Done;
    }

    /// <summary>A plane parallel to Pl through Point.</summary>
    public MakePln(in Plane3D pl, in Point3D point)
    {
        _thePln = new Plane3D(new CoordinateSystem(point, pl.Axis.Direction, pl.XAxis.Direction));
        _theError = Done;
    }

    /// <summary>A plane through the location of Axis, normal to its direction.</summary>
    public MakePln(in Axis3D axis)
    {
        _thePln = new Plane3D(axis.Location, axis.Direction);
        _theError = Done;
    }

    /// <summary>The constructed plane.</summary>
    /// <exception cref="NotDoneException">No plane was constructed.</exception>
    public Plane3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakePln::Value() - no result");
        return _thePln;
    }

    /// <summary>Alias for Value().</summary>
    public Plane3D Operator() => Value();

    /// <summary>The constructed plane (OCCT's implicit conversion operator).</summary>
    public static explicit operator Plane3D(MakePln maker) => maker.Operator();
}
