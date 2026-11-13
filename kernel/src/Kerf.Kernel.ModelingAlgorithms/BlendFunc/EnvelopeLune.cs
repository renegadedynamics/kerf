// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>
/// The piece of the ball at a corner of the path of an evolving fillet's ball (Kerf, unit 15AK3). Where the
/// derivative of the centre's path jumps, the one ball there has two circles through the same two contacts, one for
/// each side (<see cref="EnvelopeEnd"/>), and where they leave a gap the surface the balls sweep is the ball itself
/// between them: a lune, its two corners the contacts.
/// <para>
/// It is made exactly, as a rational surface of degree 2 by 2. The circles of a ball through two of its points p1 and
/// p2 are those whose tangents at the two points meet on the line the ball's tangent planes there share: through q0,
/// the pole of the chord, along t, square to both radii. The circle whose tangents meet at q0 + e tan(a) t, e the
/// distance from a contact to q0, is the arc of poles p1, q0 + e tan(a) t, p2 and weights 1, (d / e) cos(a), 1, d half
/// the chord. So in homogeneous coordinates its middle pole runs along a circle as a does, and a itself is an arc of
/// degree 2 from the first circle's angle to the second's. Across the surface (V) each line is one of the ball's
/// circles through the contacts, from the first contact to the second, with the weights the fillet's own sections
/// have; along it (U) it goes from the first circle to the second, and both its ends in V are single points.
/// </para>
/// </summary>
internal static class EnvelopeLune
{
    /// <summary>
    /// The lune of <paramref name="from"/>'s ball between its circle and <paramref name="to"/>'s, U from 0 on the first
    /// to 1 on the second and V from <paramref name="v1"/> at the contact nearest <paramref name="p1"/> to
    /// <paramref name="v2"/> at the other; with <paramref name="acrossU"/>, U and V the other way about, as the
    /// surfaces of a ball resting on an edge have them. Null where the contacts are too near each other or opposite,
    /// where either arc is more than some 150 degrees, or where the two circles are the same.
    /// </summary>
    public static BSplineSurface? Make(in EnvelopeEnd from, in EnvelopeEnd to, in Point3D p1, double v1, double v2, bool acrossU)
    {
        EnvelopeCircle a = from.Circle, b = to.Circle;
        Point3D c = from.Ball;
        double r = from.Radius;
        Point3D c1 = a.Centre.Translated(a.Radius * a.N1), c2 = a.Centre.Translated(a.Radius * a.N2);
        if (c2.DistanceTo(p1) < c1.DistanceTo(p1))
        {
            (c1, c2) = (c2, c1);
        }

        Vector3D u1 = new(c, c1), u2 = new(c, c2), t = u1.Crossed(u2);
        double norm = t.Magnitude;
        Point3D middle = new(0.5 * (c1.X + c2.X), 0.5 * (c1.Y + c2.Y), 0.5 * (c1.Z + c2.Z));
        Vector3D m = new(c, middle);
        double d = c1.DistanceTo(middle);
        if (r <= 0.0 || norm < 1.0e-6 * r * r || m.SquareMagnitude < 1.0e-12 * r * r || a.N1.Dot(a.N2) < -0.85 || b.N1.Dot(b.N2) < -0.85)
        {
            return null;
        }

        t /= norm;
        Point3D q0 = c.Translated(r * r / m.SquareMagnitude * m);
        double e = c1.DistanceTo(q0);
        double first = Math.Atan(new Vector3D(q0, Pole(a)).Dot(t) / e), second = Math.Atan(new Vector3D(q0, Pole(b)).Dot(t) / e);
        double half = 0.5 * (second - first);
        if (Math.Abs(half) < 1.0e-12 || Math.Abs(half) > 0.7)
        {
            return null;
        }

        Span<double> angle = [first, first + half, second];
        Span<double> along = [1.0, Math.Cos(half), 1.0];
        Point3D[] poles = new Point3D[9];
        double[] weights = new double[9];
        bool rising = v1 < v2;
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                int k = acrossU ? 3 * j + i : 3 * i + j;
                poles[k] = j == 1 ? q0.Translated(e * Math.Tan(angle[i]) * t) : (j == 0) == rising ? c1 : c2;
                weights[k] = j == 1 ? d / e * Math.Cos(angle[i]) : along[i];
            }
        }

        double[] ends = [0.0, 1.0], sides = [Math.Min(v1, v2), Math.Max(v1, v2)];
        return new BSplineSurface(new Grid<Point3D>(poles, 3, 3), acrossU ? sides : ends, acrossU ? ends : sides, [3, 3], [3, 3], 2, 2,
                                  false, false, new Grid<double>(weights, 3, 3));
    }

    // Where the tangents to the circle at its two contacts meet.
    private static Point3D Pole(in EnvelopeCircle circle) =>
        circle.Centre.Translated(circle.Radius / (1.0 + circle.N1.Dot(circle.N2)) * (circle.N1 + circle.N2));
}
