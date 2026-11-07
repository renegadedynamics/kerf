// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Measurement;

/// <summary>
/// What a planar face bounded by lines and circles adds to a volume, in closed form. It's what <see cref="Vinert"/> integrates
/// and no more: the same boundary, in the same order, against the same reference point, without the integration or its work
/// vectors (decision 21402).
/// </summary>
/// <remarks>
/// On a plane P(u, v) = O + uX + vY, the integrand (P - ref) . (X x Y) / 3 doesn't depend on u or v, so a face's part is
/// (O - ref) . (X x Y) / 3 times its signed area in UV. Green's theorem makes the area the integral of u dv around the boundary,
/// which is OCCT's own reduction to the edges, and for a line or a circle that has a closed form. The integration's error
/// is no longer in the figure: for lines it's rounding, about 1e-14 of the volume, and for circles at most its tolerance.
/// </remarks>
internal static class PlanarFaceMass
{
    // A boundary that closes to within this in UV, as a valid face's does. The integration measures u from the face's least u, so
    // an open boundary's result depends on that; such a face goes to the integration, for the same answer as before
    private const double ClosedWithin = 1e-5;

    /// <summary>
    /// A planar face's part of a volume. False for any other face: another surface, no boundary, an edge
    /// without a pcurve or with another curve, a boundary that doesn't close.
    /// </summary>
    public static bool TryVolume(in Face face, Domain domain, in Point3D reference, out double volume)
    {
        volume = 0;
        if (face.ChildCount == 0 || Tool.Surface(face, out Placement placement) is not PlaneSurface plane)
        {
            return false;
        }

        double area = 0, du = 0, dv = 0;
        for (domain.Init(face); domain.More(); domain.Next())
        {
            Edge edge = domain.Value();
            Curve2D? curve = Tool.CurveOnSurface(edge, plane, placement, out double first, out double last);
            double arcArea, arcU, arcV;
            switch (curve)
            {
                case LineCurve2D line:
                    (arcArea, arcU, arcV) = OfLine(line, first, last);
                    break;
                case CircleCurve2D circle:
                    (arcArea, arcU, arcV) = OfCircle(circle, first, last);
                    break;
                default:
                    return false;
            }

            double sign = edge.Orientation == Reversed ? -1 : 1;
            area += sign * arcArea;
            du += sign * arcU;
            dv += sign * arcV;
        }

        if (Math.Abs(du) > ClosedWithin || Math.Abs(dv) > ClosedWithin)
        {
            return false;
        }

        CoordinateSystem position = plane.Position;
        Point3D origin = position.Location;
        var x = new Vector3D(position.XDirection);
        var y = new Vector3D(position.YDirection);
        if (!placement.IsIdentity)
        {
            Transform3D transform = placement.Transformation;
            origin = origin.Transformed(transform);
            x = x.Transformed(transform);
            y = y.Transformed(transform);
        }

        Vector3D normal = x.Crossed(y);
        double height = (origin.X - reference.X) * normal.X + (origin.Y - reference.Y) * normal.Y + (origin.Z - reference.Z) * normal.Z;
        volume = height * area / 3.0;
        if (face.Orientation == Reversed)
        {
            volume = -volume;
        }

        return true;
    }

    // The integral of u dv from first to last, and how far u and v moved
    private static (double Area, double Du, double Dv) OfLine(LineCurve2D line, double first, double last)
    {
        Point2D p = line.Location;
        Direction2D d = line.Direction;
        double dt = last - first;
        return (d.Y * (p.X * dt + d.X * 0.5 * (last * last - first * first)), d.X * dt, d.Y * dt);
    }

    private static (double Area, double Du, double Dv) OfCircle(CircleCurve2D circle, double first, double last)
    {
        Frame2D frame = circle.Position;
        Point2D c = frame.Location;
        Direction2D x = frame.XDirection, y = frame.YDirection;
        double r = circle.Radius;
        double dt = last - first;
        // u = cu + r (xu cos t + yu sin t), dv = r (-xv sin t + yv cos t) dt
        double dCos = Math.Cos(last) - Math.Cos(first), dSin = Math.Sin(last) - Math.Sin(first);
        double du = r * (x.X * dCos + y.X * dSin), dv = r * (x.Y * dCos + y.Y * dSin);
        double sin2 = Math.Sin(2 * last) - Math.Sin(2 * first);
        double sinCos = 0.5 * (Math.Sin(last) * Math.Sin(last) - Math.Sin(first) * Math.Sin(first));
        double cosSquared = 0.5 * dt + 0.25 * sin2, sinSquared = 0.5 * dt - 0.25 * sin2;
        double area = c.X * r * (x.Y * dCos + y.Y * dSin)
                      + r * r * ((y.X * y.Y - x.X * x.Y) * sinCos + x.X * y.Y * cosSquared - y.X * x.Y * sinSquared);
        return (area, du, dv);
    }
}
