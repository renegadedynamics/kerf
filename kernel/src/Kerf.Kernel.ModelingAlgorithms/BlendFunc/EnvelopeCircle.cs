// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>
/// The circle in which the surface swept by a rolling ball of changing radius meets the ball (Kerf, unit 15AK): the
/// envelope of the balls |x - c(w)| = r(w) meets each in the plane (x - c) . c' = -r r', square to the path of the
/// centre and r r' / |c'| from it, a circle of radius r sqrt(1 - r'^2 / |c'|^2) about c - (r r' / |c'|^2) c'. A point
/// the ball rests on (its contact with a face, or with an edge) moves square to the ball's radius there, so it lies
/// in that plane: the circle goes through the contacts, and the envelope is tangent to the faces along them. With a
/// constant radius it is the ball's great circle square to the path.
/// </summary>
internal struct EnvelopeCircle
{
    // Below this share of the ball's radius the circle's is taken to be nothing: the centre isn't advancing.
    private const double Least = 1.0e-9;

    /// <summary>The circle's centre and its derivatives along the walk.</summary>
    public Point3D Centre;
    public Vector3D DCentre, D2Centre;

    /// <summary>The circle's radius and its derivatives.</summary>
    public double Radius, DRadius, D2Radius;

    /// <summary>The unit vectors from the circle's centre to the two contacts, and their derivatives.</summary>
    public Vector3D N1, DN1, D2N1, N2, DN2, D2N2;

    /// <summary>The unit normal to the circle's plane, along the path of the ball's centre, and its derivatives.</summary>
    public Vector3D Plane, DPlane, D2Plane;

    /// <summary>
    /// The circle of the ball of centre <paramref name="c"/> and radius <paramref name="r"/> (either sign) through its
    /// contacts, to the order asked (0 to 2): order n takes the centre's and the radius's derivatives to n + 1, and the
    /// unit vectors <paramref name="u1"/> and <paramref name="u2"/> from the centre to the contacts with their
    /// derivatives to n. From the directions, not the contacts themselves, so that it holds as the radius runs down to
    /// nothing: the vector from the circle's centre to a contact is |r| (u + (|r|' / |c'|^2) c'). The plane's normal is
    /// on the side of <paramref name="reference"/>. False where the centre stands still along the walk, so the envelope
    /// has no circle there.
    /// </summary>
    public static bool Make(int order, in Point3D c, in Vector3D dc, in Vector3D d2c, in Vector3D d3c, double r, double dr,
                            double d2r, double d3r, in Vector3D u1, in Vector3D du1, in Vector3D d2u1, in Vector3D u2,
                            in Vector3D du2, in Vector3D d2u2, in Vector3D reference, out EnvelopeCircle circle)
    {
        circle = default;
        double g = dc.SquareMagnitude;
        if (g <= 0.0)
        {
            return false;
        }

        // The radius as a length, with its derivatives; m = |r|' / |c'|^2; the circle's radius is |r| s, s^2 = 1 - t
        double sign = r < 0.0 ? -1.0 : 1.0;
        double r0 = sign * r, r1 = sign * dr, r2 = sign * d2r, r3 = sign * d3r;
        double m = r1 / g;
        double t = r1 * m;
        if (1.0 - t <= Least * Least)
        {
            return false;
        }

        double s = Math.Sqrt(1.0 - t), norm = Math.Sqrt(g), invnorm = 1.0 / norm;
        double side = reference.Dot(dc) < 0.0 ? -1.0 : 1.0;
        Vector3D nn = invnorm * dc;
        double k = r0 * m;
        circle.Centre = c.Translated(-k * dc);
        circle.Radius = r0 * s;
        circle.Plane = side * nn;
        circle.N1 = InPlane(u1 + m * dc, nn);
        circle.N2 = InPlane(u2 + m * dc, nn);
        if (order == 0)
        {
            return true;
        }

        double g1 = 2 * dc.Dot(d2c);
        double m1 = (r2 - m * g1) / g;
        double t1 = (2 * r1 * r2 - t * g1) / g;
        double s1 = -t1 / (2 * s);
        double k1 = r1 * m + r0 * m1;
        circle.DCentre = dc - k1 * dc - k * d2c;
        circle.DRadius = r1 * s + r0 * s1;
        circle.DPlane = side * ConstRad.UnitNormalD1(nn, invnorm, d2c);
        circle.DN1 = (du1 + m1 * dc + m * d2c - s1 * circle.N1) / s;
        circle.DN2 = (du2 + m1 * dc + m * d2c - s1 * circle.N2) / s;
        if (order == 1)
        {
            return true;
        }

        double g2 = 2 * (d2c.SquareMagnitude + dc.Dot(d3c));
        double m2 = (r3 - 2 * m1 * g1 - m * g2) / g;
        double t2 = (2 * r2 * r2 + 2 * r1 * r3 - 2 * t1 * g1 - t * g2) / g;
        double s2 = -t2 / (2 * s) - s1 * s1 / s;
        double k2 = r2 * m + 2 * r1 * m1 + r0 * m2;
        circle.D2Centre = d2c - k2 * dc - 2 * k1 * d2c - k * d3c;
        circle.D2Radius = r2 * s + 2 * r1 * s1 + r0 * s2;
        circle.D2Plane = side * ConstRad.UnitNormalD2(nn, invnorm, d2c, d2c, d3c);
        Vector3D common = m2 * dc + 2 * m1 * d2c + m * d3c;
        circle.D2N1 = (d2u1 + common - 2 * s1 * circle.DN1 - s2 * circle.N1) / s;
        circle.D2N2 = (d2u2 + common - 2 * s1 * circle.DN2 - s2 * circle.N2) / s;
        return true;
    }

    // v's direction in the plane square to the unit vector n. A contact is on the ball, and so in the plane, only as
    // nearly as the walk's point solves its equations.
    private static Vector3D InPlane(in Vector3D v, in Vector3D n)
    {
        Vector3D p = v - v.Dot(n) * n;
        double norm = p.Magnitude;
        return norm > 0.0 ? p / norm : v;
    }

    /// <summary>
    /// The least, over the arc between the contacts, of how fast the envelope's point there moves on along the path of
    /// the ball's centre, as a share of the centre's own speed. With F(x, w) = |x - c|^2 - r^2, the envelope is where F
    /// and its derivative in w vanish, and it is a surface there only while the second derivative,
    /// 2 (|c'|^2 - (x - c) . c'' - r'^2 - r r''), keeps its sign: where it reaches nothing the circles of neighbouring
    /// balls cross, the swept surface folds back on itself, and what bounds the balls there is no longer this surface.
    /// At a contact it is the contact's own advance, p' . c'. It is linear in x, and taken at nine points of the arc.
    /// Where the radius isn't changing it is 1: a ball of constant radius on a path that turns more tightly than its
    /// radius folds its surface too, as ChFi3d's constant fillets do, and that is not this unit's to refuse.
    /// </summary>
    public readonly double LeastAdvance(in Point3D c, in Vector3D dc, in Vector3D d2c, double r, double dr, double d2r)
    {
        double g = dc.SquareMagnitude;
        if (dr * dr <= 1.0e-18 * g && Math.Abs(r * d2r) <= 1.0e-18 * g)
        {
            return 1.0;
        }

        double level = g - dr * dr - r * d2r;
        Vector3D across = Plane.Crossed(N1);
        double angle = Angle(), least = double.MaxValue;
        for (int i = 0; i <= 8; i++)
        {
            double at = angle * i / 8;
            Point3D x = Centre.Translated(Radius * (Math.Cos(at) * N1 + Math.Sin(at) * across));
            least = Math.Min(least, (level - new Vector3D(c, x).Dot(d2c)) / g);
        }

        return least;
    }

    /// <summary>The middle of the arc between the contacts; false where they're opposite or the circle is a point.</summary>
    public readonly bool ArcMiddle(out Point3D middle)
    {
        Vector3D towards = N1 + N2;
        double norm = towards.Magnitude;
        middle = Centre;
        if (norm < 1.0e-9)
        {
            return false;
        }

        middle = Centre.Translated(Radius / norm * towards);
        return true;
    }

    /// <summary>
    /// The angle of the arc from the first contact to the second about <see cref="Plane"/>, in ]-pi/2, 3pi/2] as
    /// GeomFill's circles take it.
    /// </summary>
    public readonly double Angle()
    {
        double cosa = Math.Clamp(N1.Dot(N2), -1.0, 1.0);
        double sina = Plane.Dot(N1.Crossed(N2));
        double angle = Math.Acos(cosa);
        if (sina < 0.0)
        {
            angle = cosa > 0.0 ? -angle : 2.0 * Math.PI - angle;
        }

        return angle;
    }
}
