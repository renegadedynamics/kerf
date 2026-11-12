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
    /// The circle of the ball of centre <paramref name="c"/> and radius <paramref name="r"/> (either sign) through the
    /// contacts <paramref name="p1"/> and <paramref name="p2"/>, to the order asked (0 to 2): order n takes the centre's
    /// and the radius's derivatives to n + 1 and the contacts' to n. The plane's normal is on the side of
    /// <paramref name="reference"/>. False where the centre stands still along the walk, so the envelope has no circle
    /// there.
    /// </summary>
    public static bool Make(int order, in Point3D c, in Vector3D dc, in Vector3D d2c, in Vector3D d3c, double r, double dr,
                            double d2r, double d3r, in Point3D p1, in Vector3D dp1, in Vector3D d2p1, in Point3D p2,
                            in Vector3D dp2, in Vector3D d2p2, in Vector3D reference, out EnvelopeCircle circle)
    {
        circle = default;
        double g = dc.SquareMagnitude;
        if (g <= 0.0)
        {
            return false;
        }

        // k = r r' / |c'|^2; the circle's centre is c - k c', its radius squared r^2 - k r r'
        double a = r * dr;
        double k = a / g;
        double b = r * r - a * k;
        if (b <= Least * Least * r * r)
        {
            return false;
        }

        double rho = Math.Sqrt(b), norm = Math.Sqrt(g), invnorm = 1.0 / norm;
        double side = reference.Dot(dc) < 0.0 ? -1.0 : 1.0;
        Vector3D nn = invnorm * dc;
        Point3D q = c.Translated(-k * dc);
        circle.Centre = q;
        circle.Radius = rho;
        circle.Plane = side * nn;
        circle.N1 = new Vector3D(q, p1) / rho;
        circle.N2 = InPlane(new Vector3D(q, p2), nn);
        if (order == 0)
        {
            return true;
        }

        double dg = 2 * dc.Dot(d2c);
        double da = dr * dr + r * d2r;
        double dk = (da - k * dg) / g;
        double db = 2 * a - da * k - a * dk;
        double drho = db / (2 * rho);
        Vector3D dq = dc - dk * dc - k * d2c;
        circle.DCentre = dq;
        circle.DRadius = drho;
        circle.DPlane = side * ConstRad.UnitNormalD1(nn, invnorm, d2c);
        circle.DN1 = (dp1 - dq - drho * circle.N1) / rho;
        circle.DN2 = (dp2 - dq - drho * circle.N2) / rho;
        if (order == 1)
        {
            return true;
        }

        double d2g = 2 * (d2c.SquareMagnitude + dc.Dot(d3c));
        double d2a = 3 * dr * d2r + r * d3r;
        double d2k = (d2a - 2 * dk * dg - k * d2g) / g;
        double d2b = 2 * da - d2a * k - 2 * da * dk - a * d2k;
        double d2rho = (d2b - 2 * drho * drho) / (2 * rho);
        Vector3D d2q = d2c - d2k * dc - 2 * dk * d2c - k * d3c;
        circle.D2Centre = d2q;
        circle.D2Radius = d2rho;
        circle.D2Plane = side * ConstRad.UnitNormalD2(nn, invnorm, d2c, d2c, d3c);
        circle.D2N1 = (d2p1 - d2q - 2 * drho * circle.DN1 - d2rho * circle.N1) / rho;
        circle.D2N2 = (d2p2 - d2q - 2 * drho * circle.DN2 - d2rho * circle.N2) / rho;
        return true;
    }

    // v's direction in the plane square to the unit vector n. The second contact is on the ball, and so in the plane,
    // only as nearly as the walk's point solves its equations.
    private static Vector3D InPlane(in Vector3D v, in Vector3D n)
    {
        Vector3D p = v - v.Dot(n) * n;
        double norm = p.Magnitude;
        return norm > 0.0 ? p / norm : v;
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
