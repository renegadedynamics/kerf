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
    /// <para>
    /// With a <paramref name="tilt"/> (and its derivatives along the walk) the circle is not the envelope's but the
    /// ball's circle through the same contacts that the centre's speed along the walk raised by the tilt would give:
    /// the centre's derivative with tilt t added, t the unit vector square to both contacts' directions, along which
    /// alone that derivative is free (unit 15AK2, <see cref="EnvelopeTilt"/>).
    /// </para>
    /// </summary>
    public static bool Make(int order, in Point3D c, in Vector3D dc, in Vector3D d2c, in Vector3D d3c, double r, double dr,
                            double d2r, double d3r, in Vector3D u1, in Vector3D du1, in Vector3D d2u1, in Vector3D u2,
                            in Vector3D du2, in Vector3D d2u2, in Vector3D reference, out EnvelopeCircle circle,
                            double tilt = 0.0, double dtilt = 0.0, double d2tilt = 0.0)
    {
        if (tilt == 0.0 && dtilt == 0.0 && d2tilt == 0.0)
        {
            return MakeEnvelope(order, c, dc, d2c, d3c, r, dr, d2r, d3r, u1, du1, d2u1, u2, du2, d2u2, reference, out circle);
        }

        Vector3D across = u1.Crossed(u2);
        double norm = across.Magnitude;
        if (norm < 1.0e-9)
        {
            return MakeEnvelope(order, c, dc, d2c, d3c, r, dr, d2r, d3r, u1, du1, d2u1, u2, du2, d2u2, reference, out circle);
        }

        double inv = 1.0 / norm;
        Vector3D t = inv * across, t1 = new(), t2 = new();
        if (order >= 1)
        {
            Vector3D across1 = du1.Crossed(u2) + u1.Crossed(du2);
            t1 = ConstRad.UnitNormalD1(t, inv, across1);
            if (order >= 2)
            {
                t2 = ConstRad.UnitNormalD2(t, inv, across1, across1, d2u1.Crossed(u2) + 2 * du1.Crossed(du2) + u1.Crossed(d2u2));
            }
        }

        return MakeEnvelope(order, c, dc + tilt * t, d2c + dtilt * t + tilt * t1, d3c + d2tilt * t + 2 * dtilt * t1 + tilt * t2, r, dr,
                            d2r, d3r, u1, du1, d2u1, u2, du2, d2u2, reference, out circle);
    }

    private static bool MakeEnvelope(int order, in Point3D c, in Vector3D dc, in Vector3D d2c, in Vector3D d3c, double r, double dr,
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

    /// <summary>
    /// How fast the envelope moves on at the point <paramref name="x"/> of the ball, as a share of the centre's speed:
    /// <see cref="LeastAdvance"/>'s quantity at one point, whatever the radius is doing.
    /// </summary>
    public static double Advance(in Point3D c, in Vector3D dc, in Vector3D d2c, double r, double dr, double d2r, in Point3D x)
    {
        double g = dc.SquareMagnitude;
        return (g - dr * dr - r * d2r - new Vector3D(c, x).Dot(d2c)) / g;
    }

    /// <summary>The point of the arc the share given of the way from the first contact to the second.</summary>
    public readonly Point3D ArcPoint(double share)
    {
        double at = Angle() * share;
        return Centre.Translated(Radius * (Math.Cos(at) * N1 + Math.Sin(at) * Plane.Crossed(N1)));
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

/// <summary>
/// The section at an end of a walked surface, with what the path of the ball's centre is doing there (Kerf, unit
/// 15AK2): what it takes to say how far the surfaces either side of a corner of that path leave the swept surface.
/// </summary>
internal readonly struct EnvelopeEnd(in EnvelopeCircle circle, in Point3D ball, in Vector3D dc, in Vector3D d2c, double r, double dr, double d2r)
{
    private readonly Point3D _ball = ball;
    private readonly Vector3D _dc = dc, _d2c = d2c;
    private readonly double _r = r, _dr = dr, _d2r = d2r;

    /// <summary>The envelope's circle on the ball.</summary>
    public EnvelopeCircle Circle { get; } = circle;

    /// <summary>The ball's radius, as a length.</summary>
    public double Radius => Math.Abs(_r);

    /// <summary>The derivative of the ball's centre along the walk.</summary>
    public Vector3D DCentre => _dc;

    /// <summary>
    /// The unit vector square to both contacts' directions from the ball's centre, the first's crossed with the
    /// second's as <see cref="EnvelopeCircle.Make"/> takes them; false where the contacts are in line with the centre.
    /// </summary>
    public bool Across(out Vector3D across)
    {
        Vector3D u1 = new(_ball, Circle.Centre.Translated(Circle.Radius * Circle.N1)), u2 = new(_ball, Circle.Centre.Translated(Circle.Radius * Circle.N2));
        across = u1.Crossed(u2);
        double norm = across.Magnitude;
        if (norm < 1.0e-9 * _r * _r)
        {
            return false;
        }

        across /= norm;
        return true;
    }

    /// <summary>The ball's centre.</summary>
    public Point3D Ball => _ball;

    /// <summary>
    /// Whether <paramref name="other"/>'s arc, a circle of the same ball, lies on the side of this circle where this
    /// side's own balls are (<paramref name="direction"/>: 1 for greater parameters, -1 for less): the two sides'
    /// surfaces then cross, and the swept surface has a crease there. Otherwise the circles leave a gap, and the swept
    /// surface has the piece of the ball between them. Two circles of a ball through the same two points don't cross
    /// between them, so the arc's seven inner points agree but for rounding; the greater share is taken.
    /// </summary>
    public bool Crosses(in EnvelopeEnd other, int direction)
    {
        double sum = 0.0;
        for (int i = 1; i <= 7; i++)
        {
            sum += new Vector3D(Circle.Centre, other.Circle.ArcPoint(i / 8.0)).Dot(_dc);
        }

        return (sum > 0.0) == (direction > 0);
    }

    /// <summary><see cref="EnvelopeCircle.LeastAdvance"/> of this section.</summary>
    public double LeastAdvance() => Circle.LeastAdvance(_ball, _dc, _d2c, _r, _dr, _d2r);

    /// <summary>
    /// How far inside this side's balls its surface is once it is made to end on <paramref name="other"/>'s arc, a
    /// circle of the same ball (<see cref="EnvelopeTilt"/>), to the second order. With F(x, w) = |x - c(w)|^2 - r(w)^2,
    /// a point x of the ball has F = 0, F' = -2 |c'| h, h its distance from this circle's plane along the centre's way,
    /// and F'' = 2 |c'|^2 a, a the advance there (<see cref="EnvelopeCircle.Advance"/>). So the ball a step d on along
    /// the walk holds x by (2 |c'| h d - |c'|^2 a d^2) / (2 r), at most h^2 / (2 r a), at d = h / (|c'| a).
    /// <para>
    /// <paramref name="direction"/> says which way this side's balls lie from the end (1 for greater parameters, -1 for
    /// less, 0 not known). Where the other's arc is on that side of this circle (the two sides' surfaces cross), the
    /// balls that hold x are there, and the surface's end is h^2 / (2 r a) inside them. Where it is on the far side
    /// (the circles leave a gap, and the swept surface has the piece of the ball between them) the end itself is on
    /// the swept surface, and it is the tilted sections beside it that are inside the balls behind them, of which there
    /// are only those back to the end: at s from the end, the tilt there being the ramp's share of h, by the greatest
    /// the balls up to s back allow. <paramref name="ramp"/> is the tilt's length, in the walk's parameter. Taken at
    /// nine points of the arc.
    /// </para>
    /// </summary>
    public double Depth(in EnvelopeEnd other, int direction, double ramp)
    {
        double worst = 0.0, speed = _dc.Magnitude;
        if (speed <= 0.0)
        {
            return double.MaxValue;
        }

        for (int i = 0; i <= 8; i++)
        {
            Point3D x = other.Circle.ArcPoint(i / 8.0);
            double h = new Vector3D(Circle.Centre, x).Dot(_dc) / speed;
            double advance = EnvelopeCircle.Advance(_ball, _dc, _d2c, _r, _dr, _d2r, x);
            if (h * h <= 1.0e-24 * _r * _r)
            {
                continue;
            }

            if (advance <= 0.0)
            {
                return double.MaxValue;
            }

            if (direction == 0 || ramp <= 0.0 || (h > 0.0) == (direction > 0))
            {
                worst = Math.Max(worst, h * h / (2.0 * Radius * advance));
                continue;
            }

            for (int k = 1; k <= 64; k++)
            {
                double share = k / 64.0, s = share * ramp * speed;
                double tilted = Math.Abs(h) * (1.0 - share * share * share * (10.0 - 15.0 * share + 6.0 * share * share));
                double step = Math.Min(tilted / advance, s);
                worst = Math.Max(worst, (2.0 * tilted * step - advance * step * step) / (2.0 * Radius));
            }
        }

        return worst;
    }
}

/// <summary>
/// The tilt of a walked surface's sections beside its ends (Kerf, unit 15AK2). Where the path of the ball's centre has
/// a corner, the one ball there has two circles through the same contacts, one for each side, and two surfaces that
/// each end on their own don't meet. The later of the two is made to end on the earlier's: its sections beside that
/// end are its balls' circles through their contacts for a speed of the centre that goes over, along a ramp, from its
/// own to the other side's at the corner. A circle of a ball off the envelope's is inside the neighbouring balls, by
/// what <see cref="EnvelopeEnd.Depth"/> gives, so the surface stays within the corner's measure of the swept one.
/// </summary>
internal sealed class EnvelopeTilt
{
    private readonly List<(double At, double Strength, double Length)> _ends = [];

    /// <summary>
    /// A tilt of the strength given (a speed of the centre, along <see cref="EnvelopeEnd.Across"/>) at the parameter
    /// <paramref name="at"/>, falling to nothing over <paramref name="length"/> either side of it.
    /// </summary>
    public void Add(double at, double strength, double length)
    {
        if (strength != 0.0 && length > 0.0)
        {
            _ends.Add((at, strength, length));
        }
    }

    /// <summary>The tilt at the parameter, and its first two derivatives: 1 - x^3 (10 - 15 x + 6 x^2) of each end's strength, x the share of its length away.</summary>
    public void At(double prm, out double tilt, out double dtilt, out double d2tilt)
    {
        tilt = dtilt = d2tilt = 0.0;
        if (double.IsNaN(prm))
        {
            return;
        }

        foreach ((double at, double strength, double length) in _ends)
        {
            double x = Math.Abs(prm - at) / length;
            if (x >= 1.0)
            {
                continue;
            }

            double sign = prm >= at ? 1.0 : -1.0;
            tilt += strength * (1.0 - x * x * x * (10.0 - 15.0 * x + 6.0 * x * x));
            dtilt += strength * sign * (-30.0 * x * x * (1.0 - x) * (1.0 - x)) / length;
            d2tilt += strength * (-60.0 * x * (1.0 - x) * (1.0 - 2.0 * x)) / (length * length);
        }
    }
}
