// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Measurement;

/// <summary>
/// What a planar face bounded by lines and circles adds to a volume, in closed form. It's what <see cref="Vinert"/> integrates
/// and no more: the same boundary, in the same order, against the same reference point, without the integration or its work
/// vectors (decision 1871).
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

    // A face whose area is less than this of the square of its boundary's length: a ratio of two near-cancellations, left to the integration
    private const double SliverArea = 1e-10;

    /// <summary>
    /// A planar face's part of a volume. False for any other face: another surface, no boundary, an edge
    /// without a pcurve or with another curve, a boundary that doesn't close.
    /// </summary>
    public static bool TryVolume(in Face face, Domain domain, in Point3D reference, out double volume)
    {
        volume = 0;
        if (!TryParts(face, domain, out double area, out Point3D origin, out Vector3D normal))
        {
            return false;
        }

        double height = (origin.X - reference.X) * normal.X + (origin.Y - reference.Y) * normal.Y + (origin.Z - reference.Z) * normal.Z;
        volume = height * area / 3.0;
        if (face.Orientation == Reversed)
        {
            volume = -volume;
        }

        return true;
    }

    /// <summary>
    /// A planar face's area, whichever way the face is turned, as <see cref="Sinert"/> integrates it. False as for
    /// <see cref="TryVolume"/>.
    /// </summary>
    public static bool TryArea(in Face face, Domain domain, out double area)
    {
        if (!TryParts(face, domain, out double inUV, out _, out Vector3D normal))
        {
            area = 0;
            return false;
        }

        area = inUV * normal.Magnitude;
        return true;
    }

    // The face's signed area in UV, and the plane's origin and X x Y as placed
    private static bool TryParts(in Face face, Domain domain, out double area, out Point3D origin, out Vector3D normal)
    {
        area = 0;
        origin = default;
        normal = default;
        if (face.ChildCount == 0 || Tool.Surface(face, out Placement placement) is not PlaneSurface plane)
        {
            return false;
        }

        double du = 0, dv = 0;
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
        origin = position.Location;
        var x = new Vector3D(position.XDirection);
        var y = new Vector3D(position.YDirection);
        if (!placement.IsIdentity)
        {
            Transform3D transform = placement.Transformation;
            origin = origin.Transformed(transform);
            x = x.Transformed(transform);
            y = y.Transformed(transform);
        }

        normal = x.Crossed(y);
        return true;
    }

    /// <summary>
    /// A planar face's centre of area, in closed form (decision 22001): Green's theorem gives the first moments as the integrals of
    /// u^2/2 dv and uv dv around the boundary. Measured from a point of the boundary, so a face far from its plane's origin loses
    /// nothing to cancellation. False as for <see cref="TryVolume"/>, and for a face with next to no area for the size of its
    /// boundary, where the ratio means little.
    /// </summary>
    public static bool TryCentre(in Face face, Domain domain, out Point3D centre)
    {
        centre = default;
        if (face.ChildCount == 0 || Tool.Surface(face, out Placement placement) is not PlaneSurface plane)
        {
            return false;
        }

        bool started = false;
        double u0 = 0, v0 = 0;
        double area = 0, momentU = 0, momentV = 0, du = 0, dv = 0, length = 0;
        for (domain.Init(face); domain.More(); domain.Next())
        {
            Edge edge = domain.Value();
            Curve2D? curve = Tool.CurveOnSurface(edge, plane, placement, out double first, out double last);
            ArcMoments arc;
            switch (curve)
            {
                case LineCurve2D line:
                    if (!started)
                    {
                        (u0, v0, started) = (line.Location.X + first * line.Direction.X, line.Location.Y + first * line.Direction.Y, true);
                    }

                    arc = MomentsOfLine(line, first, last, u0, v0);
                    break;
                case CircleCurve2D circle:
                    if (!started)
                    {
                        Frame2D frame = circle.Position;
                        (double sin, double cos) = Math.SinCos(first);
                        u0 = frame.Location.X + circle.Radius * (frame.XDirection.X * cos + frame.YDirection.X * sin);
                        v0 = frame.Location.Y + circle.Radius * (frame.XDirection.Y * cos + frame.YDirection.Y * sin);
                        started = true;
                    }

                    arc = MomentsOfCircle(circle, first, last, u0, v0);
                    break;
                default:
                    return false;
            }

            double sign = edge.Orientation == Reversed ? -1 : 1;
            area += sign * arc.Area;
            momentU += sign * arc.MomentU;
            momentV += sign * arc.MomentV;
            du += sign * arc.Du;
            dv += sign * arc.Dv;
            length += arc.Length;
        }

        if (Math.Abs(du) > ClosedWithin || Math.Abs(dv) > ClosedWithin || Math.Abs(area) <= SliverArea * length * length)
        {
            return false;
        }

        double u = u0 + momentU / area, v = v0 + momentV / area;
        CoordinateSystem position = plane.Position;
        Direction3D x = position.XDirection, y = position.YDirection;
        Point3D origin = position.Location;
        centre = new Point3D(origin.X + u * x.X + v * y.X, origin.Y + u * x.Y + v * y.Y, origin.Z + u * x.Z + v * y.Z);
        if (!placement.IsIdentity)
        {
            centre = centre.Transformed(placement.Transformation);
        }

        return true;
    }

    private readonly record struct ArcMoments(double Area, double MomentU, double MomentV, double Du, double Dv, double Length);

    // The integrals of u dv, u^2/2 dv and uv dv along a line, with u and v measured from (u0, v0)
    private static ArcMoments MomentsOfLine(LineCurve2D line, double first, double last, double u0, double v0)
    {
        Point2D p = line.Location;
        Direction2D d = line.Direction;
        double ua = p.X + first * d.X - u0, va = p.Y + first * d.Y - v0;
        double ub = p.X + last * d.X - u0, vb = p.Y + last * d.Y - v0;
        double run = vb - va;
        return new ArcMoments(run * (ua + ub) / 2,
                              run * (ua * ua + ua * ub + ub * ub) / 6,
                              run * (2 * ua * va + ua * vb + ub * va + 2 * ub * vb) / 6,
                              ub - ua, run, Math.Abs(last - first));
    }

    // The same along a circle: u = cu + r (a cos t + b sin t), v = cv + r (e cos t + f sin t), which leaves products of up to
    // three sines and cosines, each with a closed integral
    private static ArcMoments MomentsOfCircle(CircleCurve2D circle, double first, double last, double u0, double v0)
    {
        Frame2D frame = circle.Position;
        double cu = frame.Location.X - u0, cv = frame.Location.Y - v0, r = circle.Radius;
        double a = frame.XDirection.X, b = frame.YDirection.X, e = frame.XDirection.Y, f = frame.YDirection.Y;
        (double s0, double c0) = Math.SinCos(first);
        (double s1, double c1) = Math.SinCos(last);
        double dt = last - first;

        // The integrals from first to last of cos, sin, and their products
        double iC = s1 - s0, iS = c0 - c1;
        double iCS = (s1 * s1 - s0 * s0) / 2;
        double sin2 = (s1 * c1 - s0 * c0) / 2;
        double iCC = dt / 2 + sin2, iSS = dt / 2 - sin2;
        double iCCC = iC - (s1 * s1 * s1 - s0 * s0 * s0) / 3;
        double iSSS = iS + (c1 * c1 * c1 - c0 * c0 * c0) / 3;
        double iCCS = (c0 * c0 * c0 - c1 * c1 * c1) / 3;
        double iCSS = (s1 * s1 * s1 - s0 * s0 * s0) / 3;

        double run = r * (f * iC - e * iS);
        // (a cos + b sin)(f cos - e sin), which is the integral of (u - cu) dv over r^2
        double uDv = a * f * iCC + (b * f - a * e) * iCS - b * e * iSS;
        // (e cos + f sin)(f cos - e sin), of (v - cv) dv
        double vDv = e * f * (iCC - iSS) + (f * f - e * e) * iCS;
        // (a cos + b sin)^2 (f cos - e sin), of (u - cu)^2 dv
        double uuDv = a * a * f * iCCC + (2 * a * b * f - a * a * e) * iCCS + (b * b * f - 2 * a * b * e) * iCSS - b * b * e * iSSS;
        // (a cos + b sin)(e cos + f sin)(f cos - e sin), of (u - cu)(v - cv) dv
        double uvDv = a * e * f * iCCC + ((a * f + b * e) * f - a * e * e) * iCCS + (b * f * f - (a * f + b * e) * e) * iCSS - b * e * f * iSSS;

        double r2 = r * r, r3 = r2 * r;
        return new ArcMoments(cu * run + r2 * uDv,
                              (cu * cu * run + 2 * cu * r2 * uDv + r3 * uuDv) / 2,
                              cu * cv * run + cu * r2 * vDv + cv * r2 * uDv + r3 * uvDv,
                              r * (a * (c1 - c0) + b * (s1 - s0)), run, r * Math.Abs(dt));
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
