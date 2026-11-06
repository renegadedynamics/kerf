// Port of: src/FoundationClasses/TKMath/ElCLib/ElCLib.hxx
// Port of: src/FoundationClasses/TKMath/ElCLib/ElCLib.lxx
// Port of: src/FoundationClasses/TKMath/ElCLib/ElCLib.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry;

/// <summary>
/// Elementary Curves Library: points and derivatives of lines and conics (2D and 3D) at a parameter, the parameter
/// of a point on such a curve, placing parameters within the period of a curve, and building 3D curves from 2D
/// ones in a coordinate system. Each function exists for a gp curve and for the curve given by its position and
/// dimensions.
/// </summary>
/// <remarks>
/// Parametrisations, in the local coordinate system of the curve:
/// line <c>P(U) = O + U * Dir</c>;
/// circle <c>X(U) = Radius * cos(U), Y(U) = Radius * sin(U)</c>;
/// ellipse <c>X(U) = MajorRadius * cos(U), Y(U) = MinorRadius * sin(U)</c>;
/// hyperbola <c>X(U) = MajorRadius * ch(U), Y(U) = MinorRadius * sh(U)</c>;
/// parabola <c>X(U) = U * U / (4 * Focal), Y(U) = U</c>.
/// Output points and vectors (OCCT's <c>gp_Pnt&amp;</c>, <c>gp_Vec&amp;</c>) are <c>out</c> parameters.
/// </remarks>
internal static class ElCLib
{
    private const double Pipi = M_PI + M_PI;

    // Threshold for angle normalization to avoid discontinuity near zero
    private static readonly double NEGATIVE_RESOLUTION = -global::Kerf.Engine.Foundation.Precision.Computational();

    // Normalize angle to [0, 2*PI] range, with special handling
    // for values very close to zero to avoid discontinuity.
    // Preserves values at exactly 2*PI for proper seam handling.
    private static void NormalizeAngle(ref double angle)
    {
        while (angle < NEGATIVE_RESOLUTION)
        {
            angle += Pipi;
        }

        // Only normalize angles strictly greater than 2*PI (with small tolerance)
        // to preserve the closing seam value of exactly 2*PI
        while (angle > Pipi * (1.0 + Constants.Resolution()))
        {
            angle -= Pipi;
        }

        if (angle < 0.0)
        {
            angle = 0.0;
        }
    }

    /// <summary>
    /// U brought into [UFirst, ULast) by adding or removing the period ULast - UFirst. ULast is expected, but not
    /// checked, to be greater than UFirst.
    /// </summary>
    /// <remarks>
    /// ULast is never returned. Because the period is computed as ULast - UFirst, rounding can make it differ
    /// slightly from the curve's true period; use with care.
    /// </remarks>
    public static double InPeriod(double u, double uFirst, double uLast)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(u) || global::Kerf.Engine.Foundation.Precision.IsInfinite(uFirst)
            || global::Kerf.Engine.Foundation.Precision.IsInfinite(uLast))
        { // In order to avoid FLT_Overflow exception
            return u;
        }

        double period = uLast - uFirst;
        if (period < Epsilon(uLast))
        {
            return u;
        }

        return Max(uFirst, u + period * Math.Ceiling((uFirst - u) / period));
    }

    /// <summary>
    /// Adjusts U1 and U2 to the range [UFirst, ULast] of a periodic curve of period ULast - UFirst: U1 is brought
    /// into [UFirst, ULast], then U2 into [U1, U1 + period]. Preci is used to test the equalities.
    /// </summary>
    public static void AdjustPeriodic(double uFirst, double uLast, double preci, ref double u1, ref double u2)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(uFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(uLast))
        {
            u1 = uFirst;
            u2 = uLast;
            return;
        }

        double period = uLast - uFirst;
        if (period < Epsilon(uLast))
        {
            // In order to avoid FLT_Overflow exception
            // (test bugs moddata_1 bug22757)
            u1 = uFirst;
            u2 = uLast;
            return;
        }

        u1 -= Math.Floor((u1 - uFirst) / period) * period;
        if (uLast - u1 < preci)
        {
            u1 -= period;
        }

        u2 -= Math.Floor((u2 - u1) / period) * period;
        if (u2 - u1 < preci)
        {
            u2 += period;
        }
    }

    // ElCLib.lxx: the gp curve overloads

    /// <summary>The point of parameter U on a line or conic.</summary>
    public static Point3D Value(double u, in Line3D l) => LineValue(u, l.Position);

    public static Point3D Value(double u, in Circle3D c) => CircleValue(u, c.Position, c.Radius);

    public static Point3D Value(double u, in Ellipse3D e) => EllipseValue(u, e.Position, e.MajorRadius, e.MinorRadius);

    public static Point3D Value(double u, in Hyperbola3D h) => HyperbolaValue(u, h.Position, h.MajorRadius, h.MinorRadius);

    public static Point3D Value(double u, in Parabola3D prb) => ParabolaValue(u, prb.Position, prb.Focal);

    /// <summary>The point P of parameter U and the first derivative V1.</summary>
    public static void D1(double u, in Line3D l, out Point3D p, out Vector3D v1) => LineD1(u, l.Position, out p, out v1);

    public static void D1(double u, in Circle3D c, out Point3D p, out Vector3D v1)
        => CircleD1(u, c.Position, c.Radius, out p, out v1);

    public static void D1(double u, in Ellipse3D e, out Point3D p, out Vector3D v1)
        => EllipseD1(u, e.Position, e.MajorRadius, e.MinorRadius, out p, out v1);

    public static void D1(double u, in Hyperbola3D h, out Point3D p, out Vector3D v1)
        => HyperbolaD1(u, h.Position, h.MajorRadius, h.MinorRadius, out p, out v1);

    public static void D1(double u, in Parabola3D prb, out Point3D p, out Vector3D v1)
        => ParabolaD1(u, prb.Position, prb.Focal, out p, out v1);

    /// <summary>The point P of parameter U and the first and second derivatives V1 and V2.</summary>
    public static void D2(double u, in Circle3D c, out Point3D p, out Vector3D v1, out Vector3D v2)
        => CircleD2(u, c.Position, c.Radius, out p, out v1, out v2);

    public static void D2(double u, in Ellipse3D e, out Point3D p, out Vector3D v1, out Vector3D v2)
        => EllipseD2(u, e.Position, e.MajorRadius, e.MinorRadius, out p, out v1, out v2);

    public static void D2(double u, in Hyperbola3D h, out Point3D p, out Vector3D v1, out Vector3D v2)
        => HyperbolaD2(u, h.Position, h.MajorRadius, h.MinorRadius, out p, out v1, out v2);

    public static void D2(double u, in Parabola3D prb, out Point3D p, out Vector3D v1, out Vector3D v2)
        => ParabolaD2(u, prb.Position, prb.Focal, out p, out v1, out v2);

    /// <summary>The point P of parameter U and the first, second and third derivatives V1, V2 and V3.</summary>
    public static void D3(double u, in Circle3D c, out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3)
        => CircleD3(u, c.Position, c.Radius, out p, out v1, out v2, out v3);

    public static void D3(double u, in Ellipse3D e, out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3)
        => EllipseD3(u, e.Position, e.MajorRadius, e.MinorRadius, out p, out v1, out v2, out v3);

    public static void D3(double u, in Hyperbola3D h, out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3)
        => HyperbolaD3(u, h.Position, h.MajorRadius, h.MinorRadius, out p, out v1, out v2, out v3);

    /// <summary>The derivative of order N (greater than 0) at the point of parameter U.</summary>
    public static Vector3D DN(double u, in Line3D l, int n) => LineDN(u, l.Position, n);

    public static Vector3D DN(double u, in Circle3D c, int n) => CircleDN(u, c.Position, c.Radius, n);

    public static Vector3D DN(double u, in Ellipse3D e, int n)
        => EllipseDN(u, e.Position, e.MajorRadius, e.MinorRadius, n);

    public static Vector3D DN(double u, in Hyperbola3D h, int n)
        => HyperbolaDN(u, h.Position, h.MajorRadius, h.MinorRadius, n);

    public static Vector3D DN(double u, in Parabola3D prb, int n) => ParabolaDN(u, prb.Position, prb.Focal, n);

    public static Point2D Value(double u, in Line2D l) => LineValue(u, l.Position);

    public static Point2D Value(double u, in Circle2D c) => CircleValue(u, c.Axis, c.Radius);

    public static Point2D Value(double u, in Ellipse2D e) => EllipseValue(u, e.Axis, e.MajorRadius, e.MinorRadius);

    public static Point2D Value(double u, in Hyperbola2D h) => HyperbolaValue(u, h.Axis, h.MajorRadius, h.MinorRadius);

    public static Point2D Value(double u, in Parabola2D prb) => ParabolaValue(u, prb.Axis, prb.Focal);

    public static void D1(double u, in Line2D l, out Point2D p, out Vector2D v1) => LineD1(u, l.Position, out p, out v1);

    public static void D1(double u, in Circle2D c, out Point2D p, out Vector2D v1)
        => CircleD1(u, c.Axis, c.Radius, out p, out v1);

    public static void D1(double u, in Ellipse2D e, out Point2D p, out Vector2D v1)
        => EllipseD1(u, e.Axis, e.MajorRadius, e.MinorRadius, out p, out v1);

    public static void D1(double u, in Hyperbola2D h, out Point2D p, out Vector2D v1)
        => HyperbolaD1(u, h.Axis, h.MajorRadius, h.MinorRadius, out p, out v1);

    public static void D1(double u, in Parabola2D prb, out Point2D p, out Vector2D v1)
        => ParabolaD1(u, prb.Axis, prb.Focal, out p, out v1);

    public static void D2(double u, in Circle2D c, out Point2D p, out Vector2D v1, out Vector2D v2)
        => CircleD2(u, c.Axis, c.Radius, out p, out v1, out v2);

    public static void D2(double u, in Ellipse2D e, out Point2D p, out Vector2D v1, out Vector2D v2)
        => EllipseD2(u, e.Axis, e.MajorRadius, e.MinorRadius, out p, out v1, out v2);

    public static void D2(double u, in Hyperbola2D h, out Point2D p, out Vector2D v1, out Vector2D v2)
        => HyperbolaD2(u, h.Axis, h.MajorRadius, h.MinorRadius, out p, out v1, out v2);

    public static void D2(double u, in Parabola2D prb, out Point2D p, out Vector2D v1, out Vector2D v2)
        => ParabolaD2(u, prb.Axis, prb.Focal, out p, out v1, out v2);

    public static void D3(double u, in Circle2D c, out Point2D p, out Vector2D v1, out Vector2D v2, out Vector2D v3)
        => CircleD3(u, c.Axis, c.Radius, out p, out v1, out v2, out v3);

    public static void D3(double u, in Ellipse2D e, out Point2D p, out Vector2D v1, out Vector2D v2, out Vector2D v3)
        => EllipseD3(u, e.Axis, e.MajorRadius, e.MinorRadius, out p, out v1, out v2, out v3);

    public static void D3(double u, in Hyperbola2D h, out Point2D p, out Vector2D v1, out Vector2D v2, out Vector2D v3)
        => HyperbolaD3(u, h.Axis, h.MajorRadius, h.MinorRadius, out p, out v1, out v2, out v3);

    public static Vector2D DN(double u, in Line2D l, int n) => LineDN(u, l.Position, n);

    public static Vector2D DN(double u, in Circle2D c, int n) => CircleDN(u, c.Axis, c.Radius, n);

    public static Vector2D DN(double u, in Ellipse2D e, int n)
        => EllipseDN(u, e.Axis, e.MajorRadius, e.MinorRadius, n);

    public static Vector2D DN(double u, in Hyperbola2D h, int n)
        => HyperbolaDN(u, h.Axis, h.MajorRadius, h.MinorRadius, n);

    public static Vector2D DN(double u, in Parabola2D prb, int n) => ParabolaDN(u, prb.Axis, prb.Focal, n);

    /// <summary>
    /// The parameter of a point on a curve. The point must be on the curve; these functions don't check it, and
    /// may throw if it is not.
    /// </summary>
    public static double Parameter(in Line3D l, in Point3D p) => LineParameter(l.Position, p);

    public static double Parameter(in Line2D l, in Point2D p) => LineParameter(l.Position, p);

    public static double Parameter(in Circle3D c, in Point3D p) => CircleParameter(c.Position, p);

    public static double Parameter(in Circle2D c, in Point2D p) => CircleParameter(c.Axis, p);

    public static double Parameter(in Ellipse3D e, in Point3D p)
        => EllipseParameter(e.Position, e.MajorRadius, e.MinorRadius, p);

    public static double Parameter(in Ellipse2D e, in Point2D p)
        => EllipseParameter(e.Axis, e.MajorRadius, e.MinorRadius, p);

    public static double Parameter(in Hyperbola3D h, in Point3D p)
        => HyperbolaParameter(h.Position, h.MajorRadius, h.MinorRadius, p);

    public static double Parameter(in Hyperbola2D h, in Point2D p)
        => HyperbolaParameter(h.Axis, h.MajorRadius, h.MinorRadius, p);

    // A parabola of focal 0 evaluates as the line along its X direction, so its parameter is measured along it (OCCT #1617, decision 1781)
    public static double Parameter(in Parabola3D prb, in Point3D p)
        => prb.Focal > Constants.Resolution() ? ParabolaParameter(prb.Position, p) : new Vector3D(prb.Location, p).Dot(new Vector3D(prb.Position.XDirection));

    public static double Parameter(in Parabola2D prb, in Point2D p)
        => prb.Focal > Constants.Resolution() ? ParabolaParameter(prb.Axis, p) : new Vector2D(prb.Location, p).Dot(new Vector2D(prb.Axis.XAxis.Direction));

    // ElCLib.cxx: curves given by their geometric characteristics

    public static Point3D LineValue(double u, in Axis3D pos)
    {
        Xyz zDir = pos.Direction.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        return new Point3D(u * zDir.X() + pLoc.X(), u * zDir.Y() + pLoc.Y(), u * zDir.Z() + pLoc.Z());
    }

    public static Point3D CircleValue(double u, in Frame3D pos, double radius)
    {
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double a1 = radius * Math.Cos(u);
        double a2 = radius * Math.Sin(u);
        return new Point3D(a1 * xDir.X() + a2 * yDir.X() + pLoc.X(),
                          a1 * xDir.Y() + a2 * yDir.Y() + pLoc.Y(),
                          a1 * xDir.Z() + a2 * yDir.Z() + pLoc.Z());
    }

    public static Point3D EllipseValue(double u, in Frame3D pos, double majorRadius, double minorRadius)
    {
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double a1 = majorRadius * Math.Cos(u);
        double a2 = minorRadius * Math.Sin(u);
        return new Point3D(a1 * xDir.X() + a2 * yDir.X() + pLoc.X(),
                          a1 * xDir.Y() + a2 * yDir.Y() + pLoc.Y(),
                          a1 * xDir.Z() + a2 * yDir.Z() + pLoc.Z());
    }

    public static Point3D HyperbolaValue(double u, in Frame3D pos, double majorRadius, double minorRadius)
    {
        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double a1 = majorRadius * Math.Cosh(u);
        double a2 = minorRadius * Math.Sinh(u);
        return new Point3D(a1 * xDir.X() + a2 * yDir.X() + pLoc.X(),
                          a1 * xDir.Y() + a2 * yDir.Y() + pLoc.Y(),
                          a1 * xDir.Z() + a2 * yDir.Z() + pLoc.Z());
    }

    /// <summary>The point of parameter U on the parabola; a parabola of focal length below gp::Resolution() is a line.</summary>
    public static Point3D ParabolaValue(double u, in Frame3D pos, double focal)
    {
        if (Math.Abs(focal) <= Constants.Resolution())
        {
            Xyz xDir0 = pos.XDirection.XYZ();
            Xyz pLoc0 = pos.Location.XYZ();
            return new Point3D(u * xDir0.X() + pLoc0.X(), u * xDir0.Y() + pLoc0.Y(), u * xDir0.Z() + pLoc0.Z());
        }

        Xyz xDir = pos.XDirection.XYZ();
        Xyz yDir = pos.YDirection.XYZ();
        Xyz pLoc = pos.Location.XYZ();
        double a1 = u * u / (4.0 * focal);
        return new Point3D(a1 * xDir.X() + u * yDir.X() + pLoc.X(),
                          a1 * xDir.Y() + u * yDir.Y() + pLoc.Y(),
                          a1 * xDir.Z() + u * yDir.Z() + pLoc.Z());
    }

    public static void LineD1(double u, in Axis3D pos, out Point3D p, out Vector3D v1)
    {
        Xyz coord = pos.Direction.XYZ();
        v1 = new Vector3D(coord);
        coord.SetLinearForm(u, coord, pos.Location.XYZ());
        p = new Point3D(coord);
    }

    public static void CircleD1(double u, in Frame3D pos, double radius, out Point3D p, out Vector3D v1)
    {
        double xc = radius * Math.Cos(u);
        double yc = radius * Math.Sin(u);
        Xyz coord1 = pos.XDirection.XYZ();
        Xyz coord2 = pos.YDirection.XYZ();
        Xyz coord0 = new Xyz();
        // Point courant :
        coord0.SetLinearForm(xc, coord1, yc, coord2, pos.Location.XYZ());
        p = new Point3D(coord0);
        // D1 :
        coord0.SetLinearForm(-yc, coord1, xc, coord2);
        v1 = new Vector3D(coord0);
    }

    public static void EllipseD1(double u, in Frame3D pos, double majorRadius, double minorRadius,
                                 out Point3D p, out Vector3D v1)
    {
        double xc = Math.Cos(u);
        double yc = Math.Sin(u);
        Xyz coord1 = pos.XDirection.XYZ();
        Xyz coord2 = pos.YDirection.XYZ();
        Xyz coord0 = new Xyz();
        // Point courant :
        coord0.SetLinearForm(xc * majorRadius, coord1, yc * minorRadius, coord2, pos.Location.XYZ());
        p = new Point3D(coord0);
        // D1 :
        coord0.SetLinearForm(-yc * majorRadius, coord1, xc * minorRadius, coord2);
        v1 = new Vector3D(coord0);
    }

    public static void HyperbolaD1(double u, in Frame3D pos, double majorRadius, double minorRadius,
                                   out Point3D p, out Vector3D v1)
    {
        double xc = Math.Cosh(u);
        double yc = Math.Sinh(u);
        Xyz coord1 = pos.XDirection.XYZ();
        Xyz coord2 = pos.YDirection.XYZ();
        Xyz coord0 = new Xyz();
        // Point courant :
        coord0.SetLinearForm(xc * majorRadius, coord1, yc * minorRadius, coord2, pos.Location.XYZ());
        p = new Point3D(coord0);
        // D1 :
        coord0.SetLinearForm(yc * majorRadius, coord1, xc * minorRadius, coord2);
        v1 = new Vector3D(coord0);
    }

    public static void ParabolaD1(double u, in Frame3D pos, double focal, out Point3D p, out Vector3D v1)
    {
        Xyz coord1 = pos.XDirection.XYZ();
        if (Math.Abs(focal) <= Constants.Resolution())
        { // Parabole degenere en une droite
            v1 = new Vector3D(coord1);
            coord1.Multiply(u);
            coord1.Add(pos.Location.XYZ());
            p = new Point3D(coord1);
        }
        else
        {
            Xyz coord0 = new Xyz();
            Xyz coord2 = pos.YDirection.XYZ();
            coord0.SetLinearForm(u / (2.0 * focal), coord1, coord2);
            v1 = new Vector3D(coord0);
            coord0.SetLinearForm((u * u) / (4.0 * focal), coord1, u, coord2, pos.Location.XYZ());
            p = new Point3D(coord0);
        }
    }

    public static void CircleD2(double u, in Frame3D pos, double radius, out Point3D p, out Vector3D v1, out Vector3D v2)
    {
        double xc = radius * Math.Cos(u);
        double yc = radius * Math.Sin(u);
        Xyz coord1 = pos.XDirection.XYZ();
        Xyz coord2 = pos.YDirection.XYZ();
        Xyz coord0 = new Xyz();
        // Point courant :
        coord0.SetLinearForm(xc, coord1, yc, coord2, pos.Location.XYZ());
        p = new Point3D(coord0);
        // D1 :
        coord0.SetLinearForm(-yc, coord1, xc, coord2);
        v1 = new Vector3D(coord0);
        // D2 :
        coord0.SetLinearForm(-xc, coord1, -yc, coord2);
        v2 = new Vector3D(coord0);
    }

    public static void EllipseD2(double u, in Frame3D pos, double majorRadius, double minorRadius,
                                 out Point3D p, out Vector3D v1, out Vector3D v2)
    {
        double xc = Math.Cos(u);
        double yc = Math.Sin(u);
        Xyz coord1 = pos.XDirection.XYZ();
        Xyz coord2 = pos.YDirection.XYZ();
        Xyz coord0 = new Xyz();
        // Point courant :
        coord0.SetLinearForm(xc * majorRadius, coord1, yc * minorRadius, coord2, pos.Location.XYZ());
        p = new Point3D(coord0);
        // D1 :
        coord0.SetLinearForm(-yc * majorRadius, coord1, xc * minorRadius, coord2);
        v1 = new Vector3D(coord0);
        // D2 :
        coord0.SetLinearForm(-xc * majorRadius, coord1, -yc * minorRadius, coord2);
        v2 = new Vector3D(coord0);
    }

    public static void HyperbolaD2(double u, in Frame3D pos, double majorRadius, double minorRadius,
                                   out Point3D p, out Vector3D v1, out Vector3D v2)
    {
        double xc = Math.Cosh(u);
        double yc = Math.Sinh(u);
        Xyz coord1 = pos.XDirection.XYZ();
        Xyz coord2 = pos.YDirection.XYZ();
        Xyz coord0 = new Xyz();
        // Point courant et D2:
        coord0.SetLinearForm(xc * majorRadius, coord1, yc * minorRadius, coord2);
        v2 = new Vector3D(coord0);
        coord0.Add(pos.Location.XYZ());
        p = new Point3D(coord0);
        // D1 :
        coord0.SetLinearForm(yc * majorRadius, coord1, xc * minorRadius, coord2);
        v1 = new Vector3D(coord0);
    }

    public static void ParabolaD2(double u, in Frame3D pos, double focal, out Point3D p, out Vector3D v1, out Vector3D v2)
    {
        Xyz coord1 = pos.XDirection.XYZ();
        if (Math.Abs(focal) <= Constants.Resolution())
        {
            v2 = new Vector3D(0.0, 0.0, 0.0);
            v1 = new Vector3D(coord1);
            coord1.Multiply(u);
            coord1.Add(pos.Location.XYZ());
            p = new Point3D(coord1);
        }
        else
        {
            Xyz coord0 = new Xyz();
            Xyz coord2 = pos.YDirection.XYZ();
            coord0.SetLinearForm((u * u) / (4.0 * focal), coord1, u, coord2, pos.Location.XYZ());
            p = new Point3D(coord0);
            coord0.SetLinearForm(u / (2.0 * focal), coord1, coord2);
            v1 = new Vector3D(coord0);
            coord1.Multiply(1.0 / (2.0 * focal));
            v2 = new Vector3D(coord1);
        }
    }

    public static void CircleD3(double u, in Frame3D pos, double radius,
                                out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3)
    {
        double xc = radius * Math.Cos(u);
        double yc = radius * Math.Sin(u);
        Xyz coord1 = pos.XDirection.XYZ();
        Xyz coord2 = pos.YDirection.XYZ();
        Xyz coord0 = new Xyz();
        // Point Courant :
        coord0.SetLinearForm(xc, coord1, yc, coord2, pos.Location.XYZ());
        p = new Point3D(coord0);
        // D1 :
        coord0.SetLinearForm(-yc, coord1, xc, coord2);
        v1 = new Vector3D(coord0);
        // D2 :
        coord0.SetLinearForm(-xc, coord1, -yc, coord2);
        v2 = new Vector3D(coord0);
        // D3 :
        coord0.SetLinearForm(yc, coord1, -xc, coord2);
        v3 = new Vector3D(coord0);
    }

    public static void EllipseD3(double u, in Frame3D pos, double majorRadius, double minorRadius,
                                 out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3)
    {
        double xc = Math.Cos(u);
        double yc = Math.Sin(u);
        Xyz coord1 = pos.XDirection.XYZ();
        Xyz coord2 = pos.YDirection.XYZ();
        Xyz coord0 = new Xyz();
        // Point Courant :
        coord0.SetLinearForm(xc * majorRadius, coord1, yc * minorRadius, coord2, pos.Location.XYZ());
        p = new Point3D(coord0);
        // D1 :
        coord0.SetLinearForm(-yc * majorRadius, coord1, xc * minorRadius, coord2);
        v1 = new Vector3D(coord0);
        // D2 :
        coord0.SetLinearForm(-xc * majorRadius, coord1, -yc * minorRadius, coord2);
        v2 = new Vector3D(coord0);
        // D3
        coord0.SetLinearForm(yc * majorRadius, coord1, -xc * minorRadius, coord2);
        v3 = new Vector3D(coord0);
    }

    public static void HyperbolaD3(double u, in Frame3D pos, double majorRadius, double minorRadius,
                                   out Point3D p, out Vector3D v1, out Vector3D v2, out Vector3D v3)
    {
        double xc = Math.Cosh(u);
        double yc = Math.Sinh(u);
        Xyz coord1 = pos.XDirection.XYZ();
        Xyz coord2 = pos.YDirection.XYZ();
        Xyz coord0 = new Xyz();
        // Point courant et D2 :
        coord0.SetLinearForm(xc * majorRadius, coord1, yc * minorRadius, coord2);
        v2 = new Vector3D(coord0);
        coord0.Add(pos.Location.XYZ());
        p = new Point3D(coord0);
        // D1 et D3 :
        coord0.SetLinearForm(yc * majorRadius, coord1, xc * minorRadius, coord2);
        v1 = new Vector3D(coord0);
        v3 = new Vector3D(coord0);
    }

    public static Point2D LineValue(double u, in Axis2D pos)
    {
        Xy zDir = pos.Direction.XY();
        Xy pLoc = pos.Location.XY();
        return new Point2D(u * zDir.X() + pLoc.X(), u * zDir.Y() + pLoc.Y());
    }

    public static Point2D CircleValue(double u, in Frame2D pos, double radius)
    {
        Xy xDir = pos.XDirection.XY();
        Xy yDir = pos.YDirection.XY();
        Xy pLoc = pos.Location.XY();
        double a1 = radius * Math.Cos(u);
        double a2 = radius * Math.Sin(u);
        return new Point2D(a1 * xDir.X() + a2 * yDir.X() + pLoc.X(),
                            a1 * xDir.Y() + a2 * yDir.Y() + pLoc.Y());
    }

    public static Point2D EllipseValue(double u, in Frame2D pos, double majorRadius, double minorRadius)
    {
        Xy xDir = pos.XDirection.XY();
        Xy yDir = pos.YDirection.XY();
        Xy pLoc = pos.Location.XY();
        double a1 = majorRadius * Math.Cos(u);
        double a2 = minorRadius * Math.Sin(u);
        return new Point2D(a1 * xDir.X() + a2 * yDir.X() + pLoc.X(),
                            a1 * xDir.Y() + a2 * yDir.Y() + pLoc.Y());
    }

    public static Point2D HyperbolaValue(double u, in Frame2D pos, double majorRadius, double minorRadius)
    {
        Xy xDir = pos.XDirection.XY();
        Xy yDir = pos.YDirection.XY();
        Xy pLoc = pos.Location.XY();
        double a1 = majorRadius * Math.Cosh(u);
        double a2 = minorRadius * Math.Sinh(u);
        return new Point2D(a1 * xDir.X() + a2 * yDir.X() + pLoc.X(),
                            a1 * xDir.Y() + a2 * yDir.Y() + pLoc.Y());
    }

    public static Point2D ParabolaValue(double u, in Frame2D pos, double focal)
    {
        if (Math.Abs(focal) <= Constants.Resolution())
        {
            Xy xDir0 = pos.XDirection.XY();
            Xy pLoc0 = pos.Location.XY();
            return new Point2D(u * xDir0.X() + pLoc0.X(), u * xDir0.Y() + pLoc0.Y());
        }

        Xy xDir = pos.XDirection.XY();
        Xy yDir = pos.YDirection.XY();
        Xy pLoc = pos.Location.XY();
        double a1 = u * u / (4.0 * focal);
        return new Point2D(a1 * xDir.X() + u * yDir.X() + pLoc.X(), a1 * xDir.Y() + u * yDir.Y() + pLoc.Y());
    }

    public static void LineD1(double u, in Axis2D pos, out Point2D p, out Vector2D v1)
    {
        Xy coord = pos.Direction.XY();
        v1 = new Vector2D(coord);
        coord.SetLinearForm(u, coord, pos.Location.XY());
        p = new Point2D(coord);
    }

    public static void CircleD1(double u, in Frame2D pos, double radius, out Point2D p, out Vector2D v1)
    {
        double xc = radius * Math.Cos(u);
        double yc = radius * Math.Sin(u);
        Xy xdir = pos.XDirection.XY();
        Xy ydir = pos.YDirection.XY();
        Xy vxy = new Xy();
        // Point courant :
        vxy.SetLinearForm(xc, xdir, yc, ydir, pos.Location.XY());
        p = new Point2D(vxy);
        // V1 :
        vxy.SetLinearForm(-yc, xdir, xc, ydir);
        v1 = new Vector2D(vxy);
    }

    public static void EllipseD1(double u, in Frame2D pos, double majorRadius, double minorRadius,
                                 out Point2D p, out Vector2D v1)
    {
        double xc = Math.Cos(u);
        double yc = Math.Sin(u);
        Xy xdir = pos.XDirection.XY();
        Xy ydir = pos.YDirection.XY();
        Xy vxy = new Xy();
        // Point courant :
        vxy.SetLinearForm(xc * majorRadius, xdir, yc * minorRadius, ydir, pos.Location.XY());
        p = new Point2D(vxy);
        // V1 :
        vxy.SetLinearForm(-yc * majorRadius, xdir, xc * minorRadius, ydir);
        v1 = new Vector2D(vxy);
    }

    public static void HyperbolaD1(double u, in Frame2D pos, double majorRadius, double minorRadius,
                                   out Point2D p, out Vector2D v1)
    {
        double xc = Math.Cosh(u);
        double yc = Math.Sinh(u);
        Xy xdir = pos.XDirection.XY();
        Xy ydir = pos.YDirection.XY();
        Xy vxy = new Xy();
        // Point courant :
        vxy.SetLinearForm(xc * majorRadius, xdir, yc * minorRadius, ydir, pos.Location.XY());
        p = new Point2D(vxy);
        // V1 :
        vxy.SetLinearForm(yc * majorRadius, xdir, xc * minorRadius, ydir);
        v1 = new Vector2D(vxy);
    }

    public static void ParabolaD1(double u, in Frame2D pos, double focal, out Point2D p, out Vector2D v1)
    {
        Xy vxy = new Xy();
        Xy xdir = pos.XDirection.XY();
        if (Math.Abs(focal) <= Constants.Resolution())
        { // Parabole degenere en une droite
            v1 = new Vector2D(xdir);
            vxy.SetLinearForm(u, xdir, pos.Location.XY());
        }
        else
        {
            Xy ydir = pos.YDirection.XY();
            vxy.SetLinearForm(u / (2.0 * focal), xdir, ydir);
            v1 = new Vector2D(vxy);
            vxy.SetLinearForm((u * u) / (4.0 * focal), xdir, u, ydir, pos.Location.XY());
        }

        p = new Point2D(vxy);
    }

    public static void CircleD2(double u, in Frame2D pos, double radius,
                                out Point2D p, out Vector2D v1, out Vector2D v2)
    {
        Xy xdir = pos.XDirection.XY();
        Xy ydir = pos.YDirection.XY();
        double xc = radius * Math.Cos(u);
        double yc = radius * Math.Sin(u);
        Xy vxy = new Xy();
        // V2 :
        vxy.SetLinearForm(xc, xdir, yc, ydir);
        v2 = new Vector2D(vxy);
        v2 = v2.Reversed();
        vxy.Add(pos.Location.XY());
        p = new Point2D(vxy);
        // V1 :
        vxy.SetLinearForm(-yc, xdir, xc, ydir);
        v1 = new Vector2D(vxy);
    }

    public static void EllipseD2(double u, in Frame2D pos, double majorRadius, double minorRadius,
                                 out Point2D p, out Vector2D v1, out Vector2D v2)
    {
        Xy xdir = pos.XDirection.XY();
        Xy ydir = pos.YDirection.XY();
        double xc = Math.Cos(u);
        double yc = Math.Sin(u);
        Xy vxy = new Xy();
        // V2 :
        vxy.SetLinearForm(xc * majorRadius, xdir, yc * minorRadius, ydir);
        v2 = new Vector2D(vxy);
        v2 = v2.Reversed();
        // Point courant :
        vxy.Add(pos.Location.XY());
        p = new Point2D(vxy);
        // V1 :
        vxy.SetLinearForm(-yc * majorRadius, xdir, xc * minorRadius, ydir);
        v1 = new Vector2D(vxy);
    }

    public static void HyperbolaD2(double u, in Frame2D pos, double majorRadius, double minorRadius,
                                   out Point2D p, out Vector2D v1, out Vector2D v2)
    {
        Xy xdir = pos.XDirection.XY();
        Xy ydir = pos.YDirection.XY();
        double xc = Math.Cosh(u);
        double yc = Math.Sinh(u);
        Xy vxy = new Xy();
        // V2 :
        vxy.SetLinearForm(xc * majorRadius, xdir, yc * minorRadius, ydir);
        v2 = new Vector2D(vxy);
        // Point courant :
        vxy.Add(pos.Location.XY());
        p = new Point2D(vxy);
        // V1 :
        vxy.SetLinearForm(yc * majorRadius, xdir, xc * minorRadius, ydir);
        v1 = new Vector2D(vxy);
    }

    public static void ParabolaD2(double u, in Frame2D pos, double focal,
                                  out Point2D p, out Vector2D v1, out Vector2D v2)
    {
        Xy vxy = new Xy();
        Xy xdir = pos.XDirection.XY();
        if (Math.Abs(focal) <= Constants.Resolution())
        {
            v2 = new Vector2D(0.0, 0.0);
            v1 = new Vector2D(xdir);
            vxy.SetLinearForm(u, xdir, pos.Location.XY());
        }
        else
        {
            Xy ydir = pos.YDirection.XY();
            vxy = xdir.Multiplied(1.0 / (2.0 * focal));
            v2 = new Vector2D(vxy);
            vxy.SetLinearForm(u, vxy, ydir);
            v1 = new Vector2D(vxy);
            vxy.SetLinearForm(u * u / (4.0 * focal), xdir, u, ydir);
            vxy.Add(pos.Location.XY());
        }

        p = new Point2D(vxy);
    }

    public static void CircleD3(double u, in Frame2D pos, double radius,
                                out Point2D p, out Vector2D v1, out Vector2D v2, out Vector2D v3)
    {
        Xy vxy = new Xy();
        Xy xdir = pos.XDirection.XY();
        Xy ydir = pos.YDirection.XY();
        double xc = radius * Math.Cos(u);
        double yc = radius * Math.Sin(u);
        // V2 :
        vxy.SetLinearForm(xc, xdir, yc, ydir);
        v2 = new Vector2D(vxy);
        v2 = v2.Reversed();
        // Point courant :
        vxy.Add(pos.Location.XY());
        p = new Point2D(vxy);
        // V1 :
        vxy.SetLinearForm(-yc, xdir, xc, ydir);
        v1 = new Vector2D(vxy);
        // V3 :
        v3 = new Vector2D(vxy);
        v3 = v3.Reversed();
    }

    public static void EllipseD3(double u, in Frame2D pos, double majorRadius, double minorRadius,
                                 out Point2D p, out Vector2D v1, out Vector2D v2, out Vector2D v3)
    {
        Xy xdir = pos.XDirection.XY();
        Xy ydir = pos.YDirection.XY();
        double xc = Math.Cos(u);
        double yc = Math.Sin(u);
        Xy vxy = new Xy();
        // V2 :
        vxy.SetLinearForm(xc * majorRadius, xdir, yc * minorRadius, ydir);
        v2 = new Vector2D(vxy);
        v2 = v2.Reversed();
        // Point courant :
        vxy.Add(pos.Location.XY());
        p = new Point2D(vxy);
        // V1 :
        vxy.SetLinearForm(-yc * majorRadius, xdir, xc * minorRadius, ydir);
        v1 = new Vector2D(vxy);
        // V3 :
        v3 = new Vector2D(vxy);
        v3 = v3.Reversed();
    }

    public static void HyperbolaD3(double u, in Frame2D pos, double majorRadius, double minorRadius,
                                   out Point2D p, out Vector2D v1, out Vector2D v2, out Vector2D v3)
    {
        Xy xdir = pos.XDirection.XY();
        Xy ydir = pos.YDirection.XY();
        double xc = Math.Cosh(u);
        double yc = Math.Sinh(u);
        Xy vxy = new Xy();
        // V2 :
        vxy.SetLinearForm(xc * majorRadius, xdir, yc * minorRadius, ydir);
        v2 = new Vector2D(vxy);
        // Point courant :
        vxy.Add(pos.Location.XY());
        p = new Point2D(vxy);
        // V1 :
        vxy.SetLinearForm(yc * majorRadius, xdir, xc * minorRadius, ydir);
        v1 = new Vector2D(vxy);
        // V3 :
        v3 = new Vector2D(vxy);
    }

    /// <summary>The derivative of order N of the line: its direction for N = 1, zero otherwise.</summary>
    public static Vector3D LineDN(double u, in Axis3D pos, int n)
    {
        if (n == 1)
        {
            return new Vector3D(pos.Direction);
        }

        return new Vector3D(0.0, 0.0, 0.0);
    }

    public static Vector3D CircleDN(double u, in Frame3D pos, double radius, int n)
    {
        double xc = 0, yc = 0;
        if (n == 1)
        {
            xc = radius * -Math.Sin(u);
            yc = radius * Math.Cos(u);
        }
        else if ((n + 2) % 4 == 0)
        {
            xc = radius * -Math.Cos(u);
            yc = radius * -Math.Sin(u);
        }
        else if ((n + 1) % 4 == 0)
        {
            xc = radius * Math.Sin(u);
            yc = radius * -Math.Cos(u);
        }
        else if (n % 4 == 0)
        {
            xc = radius * Math.Cos(u);
            yc = radius * Math.Sin(u);
        }
        else if ((n - 1) % 4 == 0)
        {
            xc = radius * -Math.Sin(u);
            yc = radius * Math.Cos(u);
        }

        Xyz coord1 = pos.XDirection.XYZ();
        coord1.SetLinearForm(xc, coord1, yc, pos.YDirection.XYZ());
        return new Vector3D(coord1);
    }

    public static Vector3D EllipseDN(double u, in Frame3D pos, double majorRadius, double minorRadius, int n)
    {
        double xc = 0, yc = 0;
        if (n == 1)
        {
            xc = majorRadius * -Math.Sin(u);
            yc = minorRadius * Math.Cos(u);
        }
        else if ((n + 2) % 4 == 0)
        {
            xc = majorRadius * -Math.Cos(u);
            yc = minorRadius * -Math.Sin(u);
        }
        else if ((n + 1) % 4 == 0)
        {
            xc = majorRadius * Math.Sin(u);
            yc = minorRadius * -Math.Cos(u);
        }
        else if (n % 4 == 0)
        {
            xc = majorRadius * Math.Cos(u);
            yc = minorRadius * Math.Sin(u);
        }
        else if ((n - 1) % 4 == 0)
        {
            xc = majorRadius * -Math.Sin(u);
            yc = minorRadius * Math.Cos(u);
        }

        Xyz coord1 = pos.XDirection.XYZ();
        coord1.SetLinearForm(xc, coord1, yc, pos.YDirection.XYZ());
        return new Vector3D(coord1);
    }

    public static Vector3D HyperbolaDN(double u, in Frame3D pos, double majorRadius, double minorRadius, int n)
    {
        double xc = 0, yc = 0;
        if (IsOdd(n))
        {
            xc = majorRadius * Math.Sinh(u);
            yc = minorRadius * Math.Cosh(u);
        }
        else if (IsEven(n))
        {
            xc = majorRadius * Math.Cosh(u);
            yc = minorRadius * Math.Sinh(u);
        }

        Xyz coord1 = pos.XDirection.XYZ();
        coord1.SetLinearForm(xc, coord1, yc, pos.YDirection.XYZ());
        return new Vector3D(coord1);
    }

    public static Vector3D ParabolaDN(double u, in Frame3D pos, double focal, int n)
    {
        if (n > 2 || n <= 0)
        {
            return new Vector3D(0.0, 0.0, 0.0);
        }

        Xyz coord1 = pos.XDirection.XYZ();
        if (n == 1)
        {
            if (Math.Abs(focal) <= Constants.Resolution())
            {
                return new Vector3D(coord1);
            }

            coord1.SetLinearForm(u / (2.0 * focal), coord1, pos.YDirection.XYZ());
            return new Vector3D(coord1);
        }

        if (Math.Abs(focal) <= Constants.Resolution())
        {
            return new Vector3D(0.0, 0.0, 0.0);
        }

        coord1.Multiply(1.0 / (2.0 * focal));
        return new Vector3D(coord1);
    }

    public static Vector2D LineDN(double u, in Axis2D pos, int n)
    {
        if (n == 1)
        {
            return new Vector2D(pos.Direction);
        }

        return new Vector2D(0.0, 0.0);
    }

    public static Vector2D CircleDN(double u, in Frame2D pos, double radius, int n)
    {
        double xc = 0, yc = 0;
        if (n == 1)
        {
            xc = radius * -Math.Sin(u);
            yc = radius * Math.Cos(u);
        }
        else if ((n + 2) % 4 == 0)
        {
            xc = radius * -Math.Cos(u);
            yc = radius * -Math.Sin(u);
        }
        else if ((n + 1) % 4 == 0)
        {
            xc = radius * Math.Sin(u);
            yc = radius * -Math.Cos(u);
        }
        else if (n % 4 == 0)
        {
            xc = radius * Math.Cos(u);
            yc = radius * Math.Sin(u);
        }
        else if ((n - 1) % 4 == 0)
        {
            xc = radius * -Math.Sin(u);
            yc = radius * Math.Cos(u);
        }

        Xy xdir = pos.XDirection.XY();
        Xy ydir = pos.YDirection.XY();
        xdir.SetLinearForm(xc, xdir, yc, ydir);
        return new Vector2D(xdir);
    }

    public static Vector2D EllipseDN(double u, in Frame2D pos, double majorRadius, double minorRadius, int n)
    {
        double xc = 0, yc = 0;
        if (n == 1)
        {
            xc = majorRadius * -Math.Sin(u);
            yc = minorRadius * Math.Cos(u);
        }
        else if ((n + 2) % 4 == 0)
        {
            xc = majorRadius * -Math.Cos(u);
            yc = minorRadius * -Math.Sin(u);
        }
        else if ((n + 1) % 4 == 0)
        {
            xc = majorRadius * Math.Sin(u);
            yc = minorRadius * -Math.Cos(u);
        }
        else if (n % 4 == 0)
        {
            xc = majorRadius * Math.Cos(u);
            yc = minorRadius * Math.Sin(u);
        }
        else if ((n - 1) % 4 == 0)
        {
            xc = majorRadius * -Math.Sin(u);
            yc = minorRadius * Math.Cos(u);
        }

        Xy xdir = pos.XDirection.XY();
        Xy ydir = pos.YDirection.XY();
        xdir.SetLinearForm(xc, xdir, yc, ydir);
        return new Vector2D(xdir);
    }

    public static Vector2D HyperbolaDN(double u, in Frame2D pos, double majorRadius, double minorRadius, int n)
    {
        double xc = 0, yc = 0;
        if (IsOdd(n))
        {
            xc = majorRadius * Math.Sinh(u);
            yc = minorRadius * Math.Cosh(u);
        }
        else if (IsEven(n))
        {
            xc = majorRadius * Math.Cosh(u);
            yc = minorRadius * Math.Sinh(u);
        }

        Xy xdir = pos.XDirection.XY();
        Xy ydir = pos.YDirection.XY();
        xdir.SetLinearForm(xc, xdir, yc, ydir);
        return new Vector2D(xdir);
    }

    public static Vector2D ParabolaDN(double u, in Frame2D pos, double focal, int n)
    {
        if (n > 2 || n <= 0)
        {
            return new Vector2D(0.0, 0.0);
        }

        Xy xdir = pos.XDirection.XY();
        if (n == 1)
        {
            if (Math.Abs(focal) <= Constants.Resolution())
            {
                return new Vector2D(xdir);
            }

            Xy ydir = pos.YDirection.XY();
            xdir.SetLinearForm(u / (2.0 * focal), xdir, ydir);
            return new Vector2D(xdir);
        }

        if (Math.Abs(focal) <= Constants.Resolution())
        {
            return new Vector2D(0.0, 0.0);
        }

        xdir.Multiply(1.0 / (2.0 * focal));
        return new Vector2D(xdir);
    }

    /// <summary>The parameter of P on the line: P(U) = L.Location() + U * L.Direction().</summary>
    public static double LineParameter(in Axis3D l, in Point3D p)
        => (p.XYZ() - l.Location.XYZ()).Dot(l.Direction.XYZ());

    /// <summary>The parameter of P on the circle of axis Pos, in [0, 2*PI]; 0 when P is on the axis.</summary>
    public static double CircleParameter(in Frame3D pos, in Point3D p)
    {
        Vector3D vec = new Vector3D(pos.Location, p);
        if (vec.SquareMagnitude < Constants.Resolution())
        {
            // coinciding points -> infinite number of parameters
            return 0.0;
        }

        Direction3D dir = pos.Direction;
        // Project vector on circle's plane
        Xyz vProj = dir.XYZ().CrossCrossed(vec.XYZ(), dir.XYZ());
        if (vProj.SquareModulus() < Constants.Resolution())
        {
            return 0.0;
        }

        // Angle between X direction and projected vector
        double teta = pos.XDirection.AngleWithRef(new Direction3D(vProj), dir);
        NormalizeAngle(ref teta);
        return teta;
    }

    public static double EllipseParameter(in Frame3D pos, double majorRadius, double minorRadius, in Point3D p)
    {
        Xyz op = p.XYZ() - pos.Location.XYZ();
        Xyz xaxis = pos.XDirection.XYZ();
        Xyz yaxis = pos.YDirection.XYZ();
        double ny = op.Dot(yaxis);
        double nx = op.Dot(xaxis);
        if ((Math.Abs(nx) <= Constants.Resolution()) && (Math.Abs(ny) <= Constants.Resolution()))
        {
            //-- The point P is on the Axis of the Ellipse.
            return 0.0;
        }

        yaxis.Multiply(ny * (majorRadius / minorRadius));
        Xyz om = xaxis.Multiplied(nx);
        om.Add(yaxis);
        double teta = new Vector3D(xaxis).AngleWithRef(new Vector3D(om), new Vector3D(pos.Direction));
        NormalizeAngle(ref teta);
        return teta;
    }

    public static double HyperbolaParameter(in Frame3D pos, double majorRadius, double minorRadius, in Point3D p)
    {
        double sht = new Vector3D(pos.Location, p).Dot(new Vector3D(pos.YDirection)) / minorRadius;
        return Math.Asinh(sht);
    }

    public static double ParabolaParameter(in Frame3D pos, in Point3D p)
        => new Vector3D(pos.Location, p).Dot(new Vector3D(pos.YDirection));

    public static double LineParameter(in Axis2D l, in Point2D p)
    {
        Xy coord = p.XY();
        coord.Subtract(l.Location.XY());
        return coord.Dot(l.Direction.XY());
    }

    public static double CircleParameter(in Frame2D pos, in Point2D p)
    {
        double teta = pos.XDirection.AngleTo(new Direction2D(new Vector2D(pos.Location, p)));
        teta = ((pos.XDirection ^ pos.YDirection) >= 0.0) ? teta : -teta;
        NormalizeAngle(ref teta);
        return teta;
    }

    public static double EllipseParameter(in Frame2D pos, double majorRadius, double minorRadius, in Point2D p)
    {
        Xy op = p.XY();
        op.Subtract(pos.Location.XY());
        Xy xaxis = pos.XDirection.XY();
        Xy yaxis = pos.YDirection.XY();
        Xy om = xaxis.Multiplied(op.Dot(xaxis));
        yaxis.Multiply((op.Dot(yaxis)) * (majorRadius / minorRadius));
        om.Add(yaxis);
        double teta = new Vector2D(xaxis).AngleTo(new Vector2D(om));
        teta = ((pos.XDirection ^ pos.YDirection) >= 0.0) ? teta : -teta;
        NormalizeAngle(ref teta);
        return teta;
    }

    public static double HyperbolaParameter(in Frame2D pos, double majorRadius, double minorRadius, in Point2D p)
    {
        Vector2D v = new Vector2D(pos.YDirection.XY());
        double sht = new Vector2D(pos.Location, p).Dot(v) / minorRadius;
        return Math.Asinh(sht);
    }

    public static double ParabolaParameter(in Frame2D pos, in Point2D p)
    {
        Vector2D directrix = new Vector2D(pos.YDirection.XY());
        return new Vector2D(pos.Location, p).Dot(directrix);
    }

    // Building 3D entities from 2D ones: the X and Y axes of the 2D space are lined up with the X and Y
    // directions of Pos.

    public static Point3D To3d(in Frame3D pos, in Point2D p)
    {
        Xyz vxy = pos.XDirection.XYZ();
        vxy.SetLinearForm(p.X, vxy, p.Y, pos.YDirection.XYZ(), pos.Location.XYZ());
        return new Point3D(vxy);
    }

    public static Direction3D To3d(in Frame3D pos, in Direction2D v)
    {
        Vector3D vx = new Vector3D(pos.XDirection);
        Vector3D vy = new Vector3D(pos.YDirection);
        vx = vx.Multiplied(v.X);
        vy = vy.Multiplied(v.Y);
        vx = vx.Added(vy);
        return new Direction3D(vx);
    }

    public static Vector3D To3d(in Frame3D pos, in Vector2D v)
    {
        Vector3D vx = new Vector3D(pos.XDirection);
        Vector3D vy = new Vector3D(pos.YDirection);
        vx = vx.Multiplied(v.X);
        vy = vy.Multiplied(v.Y);
        vx = vx.Added(vy);
        return vx;
    }

    public static Axis3D To3d(in Frame3D pos, in Axis2D a)
    {
        Point3D p = To3d(pos, a.Location);
        Vector3D v = new Vector3D(To3d(pos, a.Direction));
        return new Axis3D(p, new Direction3D(v));
    }

    public static Frame3D To3d(in Frame3D pos, in Frame2D a)
    {
        Point3D p = To3d(pos, a.Location);
        Vector3D vx = new Vector3D(To3d(pos, a.XDirection));
        Vector3D vy = new Vector3D(To3d(pos, a.YDirection));
        return new Frame3D(p, new Direction3D(vx.Crossed(vy)), new Direction3D(vx));
    }

    public static Line3D To3d(in Frame3D pos, in Line2D l) => new Line3D(To3d(pos, l.Position));

    public static Circle3D To3d(in Frame3D pos, in Circle2D c) => new Circle3D(To3d(pos, c.Axis), c.Radius);

    public static Ellipse3D To3d(in Frame3D pos, in Ellipse2D e)
        => new Ellipse3D(To3d(pos, e.Axis), e.MajorRadius, e.MinorRadius);

    public static Hyperbola3D To3d(in Frame3D pos, in Hyperbola2D h)
        => new Hyperbola3D(To3d(pos, h.Axis), h.MajorRadius, h.MinorRadius);

    public static Parabola3D To3d(in Frame3D pos, in Parabola2D prb) => new Parabola3D(To3d(pos, prb.Axis), prb.Focal);
}
