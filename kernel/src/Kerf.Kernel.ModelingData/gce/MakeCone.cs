// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCone.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>
/// Constructions of a gp_Cone: from an axis system, a semi-angle and a reference radius, coaxial to a cone through a
/// point or at a signed offset, from four points, from an axis and two points, or from two axis points and two
/// section radii.
/// </summary>
internal sealed class MakeCone : Root
{
    private Cone3D _theCone = new();

    /// <summary>
    /// A cone from an axis system, a semi-angle Ang (radians, in ]0, PI/2[) and a reference radius. Fails with
    /// gce_NegativeRadius if Radius is negative and gce_BadAngle if Ang is out of range.
    /// </summary>
    public MakeCone(in Frame3D a2, double ang, double radius)
    {
        if (radius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            if (ang <= Constants.Resolution() || M_PI / 2 - ang <= Constants.Resolution())
            {
                _theError = BadAngle;
            }
            else
            {
                _theError = Done;
                _theCone = new Cone3D(new CoordinateSystem(a2), ang, radius);
            }
        }
    }

    /// <summary>
    /// A cone from four points: P1 and P2 define the axis, the distance from P3 to it the base radius, and the
    /// distance from P4 to it the radius of the section through P4.
    /// </summary>
    public MakeCone(in Point3D p1, in Point3D p2, in Point3D p3, in Point3D p4)
    {
        // P1 and P2 define the cone axis. The distance from P3 to the axis gives
        // the base radius, and the distance from P4 to the axis gives the radius
        // of the section passing through P4.
        if (p1.DistanceTo(p2) < Constants.Resolution() || p3.DistanceTo(p4) < Constants.Resolution())
        {
            _theError = ConfusedPoints;
            return;
        }
        Direction3D d1 = new(p2.XYZ() - p1.XYZ());
        double cos = d1.Dot(new Direction3D(p4.XYZ() - p1.XYZ()));
        double dist = p1.DistanceTo(p4);
        Point3D pp4 = new(p1.XYZ() + cos * dist * d1.XYZ());
        cos = d1.Dot(new Direction3D(p3.XYZ() - p1.XYZ()));
        dist = p1.DistanceTo(p3);
        Point3D pp3 = new(p1.XYZ() + cos * dist * d1.XYZ());
        double dist13 = pp3.DistanceTo(p1);
        double dist14 = pp4.DistanceTo(p1);
        if (Math.Abs(dist13 - dist14) < Constants.Resolution())
        {
            _theError = NullAngle;
            return;
        }
        Line3D l1 = new(p1, d1);
        double dist3 = l1.DistanceTo(p3);
        double dist4 = l1.DistanceTo(p4);
        double difRad = dist3 - dist4;
        double angle = Math.Abs(Math.Atan(difRad / (dist13 - dist14)));
        if (Math.Abs(M_PI / 2.0 - angle) < Constants.Resolution() || Math.Abs(angle) < Constants.Resolution())
        {
            _theError = NullRadius;
            return;
        }
        double r1 = pp3.DistanceTo(p3);
        double r2 = pp4.DistanceTo(p4);
        if (r1 < 0.0 || r2 < 0.0)
        {
            _theError = NegativeRadius;
            return;
        }
        Direction3D dd1 = new(pp4.XYZ() - pp3.XYZ());
        Direction3D d2 = new();
        double x = dd1.X;
        double y = dd1.Y;
        double z = dd1.Z;
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
        if (r1 > r2)
        {
            angle *= -1;
        }
        _theCone = new Cone3D(new CoordinateSystem(new Frame3D(pp3, dd1, d2)), angle, r1);
        _theError = Done;
    }

    /// <summary>
    /// A cone from its axis and two points: the distance from P1 to the axis gives the base radius, from P2 the radius
    /// of the section through P2. The status is that of the four-point construction.
    /// </summary>
    public MakeCone(in Axis3D axis, in Point3D p1, in Point3D p2)
    {
        // The distance from P1 to the axis gives the base radius, and
        // the distance from P2 to the axis gives the section radius at P2.
        Point3D p3 = axis.Location;
        Point3D p4 = new(p3.XYZ() + axis.Direction.XYZ());
        MakeCone cone = new(p3, p4, p1, p2);
        if (cone.IsDone)
        {
            _theCone = cone.Value();
            _theError = Done;
        }
        else
        {
            _theError = cone.Status();
        }
    }

    /// <summary>
    /// A cone coaxial to Cone through the point P (the nearest such cone when there are two); fails with
    /// gce_NegativeRadius when no non-negative radius exists.
    /// </summary>
    public MakeCone(in Cone3D cone, in Point3D p)
    {
        Xyz vecToPoint = p.XYZ() - cone.Location.XYZ();
        double vParam = vecToPoint.Dot(cone.Axis.Direction.XYZ());
        double radius = new Line3D(cone.Axis).DistanceTo(p);
        double tan = Math.Tan(cone.SemiAngle);
        double cos = Math.Cos(cone.SemiAngle);
        double candidate1 = radius - vParam * tan;
        double candidate2 = -radius - vParam * tan;
        // Accept tiny negative values as zero (rounding noise from analytic formulas),
        // then clamp to 0.0 before constructing gp_Cone.
        static bool IsNonNegativeWithTol(double value) => value >= -Constants.Resolution();
        static double ClampToZero(double value) => value < 0.0 ? 0.0 : value;
        double resultRadius = -1.0;
        if (IsNonNegativeWithTol(candidate1) && IsNonNegativeWithTol(candidate2))
        {
            // Keep the nearest parallel cone to preserve predictable behavior.
            double cand1 = ClampToZero(candidate1);
            double cand2 = ClampToZero(candidate2);
            double dist1 = Math.Abs((cand1 - cone.RefRadius) * cos);
            double dist2 = Math.Abs((cand2 - cone.RefRadius) * cos);
            resultRadius = (dist1 <= dist2) ? cand1 : cand2;
        }
        else if (IsNonNegativeWithTol(candidate1))
        {
            resultRadius = ClampToZero(candidate1);
        }
        else if (IsNonNegativeWithTol(candidate2))
        {
            resultRadius = ClampToZero(candidate2);
        }
        if (resultRadius < 0.0)
        {
            _theError = NegativeRadius;
            return;
        }
        _theCone = new Cone3D(cone.Position, cone.SemiAngle, resultRadius);
        _theError = Done;
    }

    /// <summary>
    /// A cone coaxial to Cone at the signed distance Dist; fails with gce_NullAngle if the semi-angle's cosine is
    /// too small and gce_NegativeRadius if the radius would be negative.
    /// </summary>
    public MakeCone(in Cone3D cone, double dist)
    {
        // gp_Cone keeps |SemiAngle| in ]gp::Resolution(), PI/2 - gp::Resolution()[,
        // thus aCos is expected to be positive and not too small.
        double cos = Math.Cos(cone.SemiAngle);
        if (Math.Abs(cos) <= Constants.Resolution())
        {
            _theError = NullAngle;
            return;
        }
        double radius = cone.RefRadius + dist / cos;
        if (radius < 0.0)
        {
            _theError = NegativeRadius;
            return;
        }
        _theCone = new Cone3D(cone.Position, cone.SemiAngle, radius);
        _theError = Done;
    }

    /// <summary>
    /// A cone from the axis of a line and two points (as the gp_Ax1 overload). The status is that of the four-point
    /// construction.
    /// </summary>
    public MakeCone(in Line3D axis, in Point3D p1, in Point3D p2)
    {
        Point3D p3 = axis.Location;
        Point3D p4 = new(p3.XYZ() + axis.Direction.XYZ());
        MakeCone cone = new(p3, p4, p1, p2);
        if (cone.IsDone)
        {
            _theCone = cone.Value();
            _theError = Done;
        }
        else
        {
            _theError = cone.Status();
        }
    }

    /// <summary>
    /// A cone whose axis passes through P1 and P2, with section radius R1 at P1 and R2 at P2. Fails with gce_NullAxis
    /// if the points coincide, gce_NegativeRadius if a radius is negative and gce_NullAngle for a zero or right angle.
    /// </summary>
    public MakeCone(in Point3D p1, in Point3D p2, double r1, double r2)
    {
        // Cone defined by two points (axis) and two radii (section radii at each point).
        double dist = p1.DistanceTo(p2);
        if (dist < Constants.Resolution())
        {
            _theError = NullAxis;
        }
        else
        {
            if (r1 < 0.0 || r2 < 0.0)
            {
                _theError = NegativeRadius;
            }
            else
            {
                double angle = Math.Abs(Math.Atan((r1 - r2) / dist));
                if (Math.Abs(M_PI / 2.0 - angle) < Constants.Resolution() || Math.Abs(angle) < Constants.Resolution())
                {
                    _theError = NullAngle;
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
                    if (r1 > r2)
                    {
                        angle *= -1;
                    }
                    _theCone = new Cone3D(new CoordinateSystem(new Frame3D(p1, d1, d2)), angle, r1);
                    _theError = Done;
                }
            }
        }
    }

    /// <summary>The constructed cone.</summary>
    /// <exception cref="NotDoneException">No cone was constructed.</exception>
    public Cone3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakeCone::Value() - no result");
        return _theCone;
    }

    /// <summary>Alias for Value().</summary>
    public Cone3D Operator() => Value();

    /// <summary>The constructed cone (OCCT's implicit conversion operator).</summary>
    public static explicit operator Cone3D(MakeCone maker) => maker.Operator();
}
