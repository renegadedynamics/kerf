// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCirc.hxx
// Port of: src/ModelingData/TKGeomBase/gce/gce_MakeCirc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Values.ErrorType;

namespace Kerf.Engine.Construction.Values;

/// <summary>
/// Constructions of a gp_Circ: from an axis system and a radius, coaxial to a circle through a point or at a signed
/// offset, through three points, from a center and a normal or plane, from a center and a point on the axis, or from
/// an axis and a radius.
/// </summary>
internal sealed class MakeCirc : Root
{
    private Circle3D _theCirc = new();

    /// <summary>
    /// A circle through three points. If all three are coincident, the result is a circle of zero radius at P1; if
    /// two are, the construction fails with gce_ConfusedPoints; collinear points give gce_ColinearPoints.
    /// </summary>
    public MakeCirc(in Point3D p1, in Point3D p2, in Point3D p3)
    {
        // Three cases:
        // 1) All three points are coincident: result is a circle centered at P1 with zero radius.
        // 2) Two of the three points are coincident: no solution (error: ConfusedPoints).
        // 3) All three points are distinct: create the perpendicular bisector of P1P2 and
        //    the perpendicular bisector of P2P3. The center is their intersection and
        //    the radius is the distance from that center to any of the three points.
        double dist1, dist2, dist3, resolution;
        //
        resolution = Constants.Resolution();
        //
        dist1 = p1.DistanceTo(p2);
        dist2 = p1.DistanceTo(p3);
        dist3 = p2.DistanceTo(p3);
        //
        if ((dist1 < resolution) && (dist2 < resolution) && (dist3 < resolution))
        {
            Direction3D dirx = new(Direction3D.D.X);
            Direction3D dirz = new(Direction3D.D.Z);
            _theCirc = new Circle3D(new Frame3D(p1, dirx, dirz), 0.0);
            return; // OCCT: TheError is left unset (gce_Done in the port)
        }
        if (dist1 < resolution || dist2 < resolution)
        {
            _theError = ConfusedPoints;
            return;
        }
        //
        double x1, y1, z1, x2, y2, z2, x3, y3, z3;
        //
        p1.Coord(out x1, out y1, out z1);
        p2.Coord(out x2, out y2, out z2);
        p3.Coord(out x3, out y3, out z3);
        Direction3D dir1 = new(x2 - x1, y2 - y1, z2 - z1);
        Vector3D vDir2 = new(x3 - x2, y3 - y2, z3 - z2);
        //
        Axis3D ax1 = new(p1, dir1);
        Line3D l12 = new(ax1);
        if (l12.DistanceTo(p3) < resolution)
        {
            _theError = ColinearPoints;
            return;
        }
        //
        Vector3D vDir1 = new(dir1);
        Vector3D vDir3 = vDir1.Crossed(vDir2);
        if (vDir3.SquareMagnitude < resolution)
        {
            _theError = ColinearPoints;
            return;
        }
        //
        Direction3D dir3 = new(vDir3);
        Direction3D dir = dir1.Crossed(dir3);
        Line3D l1 = new(new Point3D((p1.XYZ() + p2.XYZ()) / 2.0), dir);
        dir = new Direction3D(vDir2.Crossed(new Vector3D(dir3)));
        Line3D l2 = new(new Point3D((p3.XYZ() + p2.XYZ()) / 2.0), dir);
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion();
        ExtElC distmin = new(l1, l2, tol);
        if (!distmin.IsDone)
        {
            _theError = IntersectionError;
        }
        else
        {
            int nbext;
            //
            //
            if (distmin.IsParallel)
            {
                _theError = IntersectionError;
                return;
            }
            nbext = distmin.ExtCount;
            //
            //
            if (nbext == 0)
            {
                _theError = IntersectionError;
            }
            else
            {
                double theDist = RealLast();
                Point3D pInt = new(), pon1, pon2;
                int i = 1;
                POnCurv Pon1, Pon2;
                while (i <= nbext)
                {
                    if (distmin.SquareDistance(i) < theDist)
                    {
                        theDist = distmin.SquareDistance(i);
                        distmin.Points(i, out Pon1, out Pon2);
                        pon1 = Pon1.Value();
                        pon2 = Pon2.Value();
                        pInt = new Point3D((pon1.XYZ() + pon2.XYZ()) / 2.0);
                    }
                    i++;
                }
                // modified by NIZNHY-PKV Thu Mar  3 11:30:34 2005f
                // Dir2.Cross(Dir1);
                // modified by NIZNHY-PKV Thu Mar  3 11:30:37 2005t
                dist1 = p1.DistanceTo(pInt);
                dist2 = p2.DistanceTo(pInt);
                dist3 = p3.DistanceTo(pInt);
                pInt.Coord(out x3, out y3, out z3);
                if (dist1 < resolution)
                {
                    Direction3D dirx = new(Direction3D.D.X);
                    Direction3D dirz = new(Direction3D.D.Z);
                    _theCirc = new Circle3D(new Frame3D(pInt, dirx, dirz), 0.0);
                    return; // OCCT: TheError is left unset (gce_Done in the port)
                }
                dir1 = new Direction3D(x1 - x3, y1 - y3, z1 - z3);
                // modified by NIZNHY-PKV Thu Mar  3 11:31:11 2005f
                // Dir2 = gp_Dir(x2-x3,y2-y3,z2-z3);
                // modified by NIZNHY-PKV Thu Mar  3 11:31:13 2005t
                //
                _theCirc = new Circle3D(new Frame3D(pInt, new Direction3D(vDir3), dir1), (dist1 + dist2 + dist3) / 3.0);
                _theError = Done;
            }
        }
    }

    /// <summary>A circle from an axis system and a radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public MakeCirc(in Frame3D a2, double radius)
    {
        if (radius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            _theError = Done;
            _theCirc = new Circle3D(a2, radius);
        }
    }

    /// <summary>A circle from its center, the plane it lies in and its radius.</summary>
    public MakeCirc(in Point3D center, in Plane3D plane, double radius)
    {
        MakeCirc c = new(center, plane.Position.Direction, radius);
        // OCCT calls C.Value(), whose check its release build (No_Exception) compiles out: a negative radius gives
        // the status gce_NegativeRadius there, and the port's always-on check would throw instead.
        _theCirc = c._theCirc;
        _theError = c.Status();
    }

    /// <summary>A circle from its center, the normal to its plane and its radius.</summary>
    public MakeCirc(in Point3D center, in Direction3D norm, double radius)
    {
        if (radius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            double a = norm.X;
            double b = norm.Y;
            double c = norm.Z;
            double aabs = Math.Abs(a);
            double babs = Math.Abs(b);
            double cabs = Math.Abs(c);
            Frame3D pos;
            // To determine the X axis:
            // the dot product Vx.Norm = 0.
            // find the max(A,B,C) for the division.
            // one of the vector coordinates is zero.
            if (babs <= aabs && babs <= cabs)
            {
                if (aabs > cabs)
                {
                    pos = new Frame3D(center, norm, new Direction3D(-c, 0.0, a));
                }
                else
                {
                    pos = new Frame3D(center, norm, new Direction3D(c, 0.0, -a));
                }
            }
            else if (aabs <= babs && aabs <= cabs)
            {
                if (babs > cabs)
                {
                    pos = new Frame3D(center, norm, new Direction3D(0.0, -c, b));
                }
                else
                {
                    pos = new Frame3D(center, norm, new Direction3D(0.0, c, -b));
                }
            }
            else
            {
                if (aabs > babs)
                {
                    pos = new Frame3D(center, norm, new Direction3D(-b, a, 0.0));
                }
                else
                {
                    pos = new Frame3D(center, norm, new Direction3D(b, -a, 0.0));
                }
            }
            _theCirc = new Circle3D(pos, radius);
            _theError = Done;
        }
    }

    /// <summary>
    /// A circle from its center, a point on its axis and its radius; fails with gce_NullAxis if the points coincide.
    /// </summary>
    public MakeCirc(in Point3D center, in Point3D ptaxis, double radius)
    {
        if (radius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            if (center.DistanceTo(ptaxis) <= Constants.Resolution())
            {
                _theError = NullAxis;
            }
            else
            {
                double a = ptaxis.X - center.X;
                double b = ptaxis.Y - center.Y;
                double c = ptaxis.Z - center.Z;
                double aabs = Math.Abs(a);
                double babs = Math.Abs(b);
                double cabs = Math.Abs(c);
                Frame3D pos;
                // To determine the X axis:
                // the dot product Vx.Norm = 0.
                // find the max(A,B,C) for the division.
                // one of the vector coordinates is zero.
                Direction3D norm = new MakeDir(center, ptaxis).Value();
                if (babs <= aabs && babs <= cabs)
                {
                    if (aabs > cabs)
                    {
                        pos = new Frame3D(center, norm, new Direction3D(-c, 0.0, a));
                    }
                    else
                    {
                        pos = new Frame3D(center, norm, new Direction3D(c, 0.0, -a));
                    }
                }
                else if (aabs <= babs && aabs <= cabs)
                {
                    if (babs > cabs)
                    {
                        pos = new Frame3D(center, norm, new Direction3D(0.0, -c, b));
                    }
                    else
                    {
                        pos = new Frame3D(center, norm, new Direction3D(0.0, c, -b));
                    }
                }
                else
                {
                    if (aabs > babs)
                    {
                        pos = new Frame3D(center, norm, new Direction3D(-b, a, 0.0));
                    }
                    else
                    {
                        pos = new Frame3D(center, norm, new Direction3D(b, -a, 0.0));
                    }
                }
                _theCirc = new Circle3D(pos, radius);
                _theError = Done;
            }
        }
    }

    /// <summary>A circle from its axis and its radius; fails with gce_NegativeRadius if Radius is negative.</summary>
    public MakeCirc(in Axis3D axis, double radius)
    {
        if (radius < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            Direction3D norm = axis.Direction;
            Point3D center = axis.Location;
            double a = norm.X;
            double b = norm.Y;
            double c = norm.Z;
            double aabs = Math.Abs(a);
            double babs = Math.Abs(b);
            double cabs = Math.Abs(c);
            Frame3D pos;
            // To determine the X axis:
            // the dot product Vx.Norm = 0.
            // find the max(A,B,C) for the division.
            // one of the vector coordinates is zero.
            if (babs <= aabs && babs <= cabs)
            {
                if (aabs > cabs)
                {
                    pos = new Frame3D(center, norm, new Direction3D(-c, 0.0, a));
                }
                else
                {
                    pos = new Frame3D(center, norm, new Direction3D(c, 0.0, -a));
                }
            }
            else if (aabs <= babs && aabs <= cabs)
            {
                if (babs > cabs)
                {
                    pos = new Frame3D(center, norm, new Direction3D(0.0, -c, b));
                }
                else
                {
                    pos = new Frame3D(center, norm, new Direction3D(0.0, c, -b));
                }
            }
            else
            {
                if (aabs > babs)
                {
                    pos = new Frame3D(center, norm, new Direction3D(-b, a, 0.0));
                }
                else
                {
                    pos = new Frame3D(center, norm, new Direction3D(b, -a, 0.0));
                }
            }
            _theCirc = new Circle3D(pos, radius);
            _theError = Done;
        }
    }

    /// <summary>
    /// A circle coaxial to Circ at the signed distance Dist (enclosing it when positive); fails with
    /// gce_NegativeRadius if the radius would be negative.
    /// </summary>
    public MakeCirc(in Circle3D circ, double dist)
    {
        double rad = circ.Radius + dist;
        if (rad < 0.0)
        {
            _theError = NegativeRadius;
        }
        else
        {
            _theCirc = new Circle3D(circ.Position, rad);
            _theError = Done;
        }
    }

    /// <summary>A circle coaxial to Circ through the point P.</summary>
    public MakeCirc(in Circle3D circ, in Point3D p)
    {
        double rad = new Line3D(circ.Axis).DistanceTo(p);
        _theCirc = new Circle3D(circ.Position, rad);
        _theError = Done;
    }

    /// <summary>The constructed circle.</summary>
    /// <exception cref="NotDoneException">No circle was constructed.</exception>
    public Circle3D Value()
    {
        StdFail_NotDone_Raise_if(_theError != Done, "gce_MakeCirc::Value() - no result");
        return _theCirc;
    }

    /// <summary>Alias for Value().</summary>
    public Circle3D Operator() => Value();

    /// <summary>The constructed circle (OCCT's implicit conversion operator).</summary>
    public static explicit operator Circle3D(MakeCirc maker) => maker.Operator();
}
