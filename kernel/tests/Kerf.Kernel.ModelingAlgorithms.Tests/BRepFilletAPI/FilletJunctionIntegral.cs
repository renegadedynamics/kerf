// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>
/// The volume a concave blend adds where two bodies meet, from the convention alone (decision 2139) and nothing of how a fillet is
/// built: a point outside both bodies is added if no ball of the radius clear of both reaches it. Each body is given by its
/// distance; the ball's centres that touch both lie on a closed curve, traced here by Newton's method, and the region is integrated
/// along rays from each centre, across the fan between its two contacts, out from the ball to the first of the bodies' surfaces
/// (unit 15T4). The first body is one rounded already, given by the distance to its core, the body less the radius all round.
/// </summary>
internal static class FilletJunctionIntegral
{
    internal readonly record struct V(double X, double Y, double Z)
    {
        public static V operator +(V a, V b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static V operator -(V a, V b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static V operator *(V a, double k) => new(a.X * k, a.Y * k, a.Z * k);
        public double Dot(V b) => X * b.X + Y * b.Y + Z * b.Z;
        public V Cross(V b) => new(Y * b.Z - Z * b.Y, Z * b.X - X * b.Z, X * b.Y - Y * b.X);
        public double Length => Math.Sqrt(Dot(this));
        public V Unit => this * (1 / Length);
    }

    // Gauss-Legendre's eight points
    private static readonly double[] Nodes = [-0.9602898564975363, -0.7966664774136267, -0.5255324099163290, -0.1834346424956498,
                                              0.1834346424956498, 0.5255324099163290, 0.7966664774136267, 0.9602898564975363];
    private static readonly double[] Weights = [0.1012285362903763, 0.2223810344533745, 0.3137066458778873, 0.3626837833783620,
                                                0.3626837833783620, 0.3137066458778873, 0.2223810344533745, 0.1012285362903763];

    internal static double Gauss(Func<double, double> f, double lo, double hi, int panels)
    {
        double sum = 0;
        for (int k = 0; k < panels; k++)
        {
            double a = lo + (hi - lo) * k / panels, b = lo + (hi - lo) * (k + 1) / panels;
            for (int i = 0; i < Nodes.Length; i++)
            {
                sum += Weights[i] * 0.5 * (b - a) * f(0.5 * (a + b) + 0.5 * (b - a) * Nodes[i]);
            }
        }
        return sum;
    }

    /// <summary>The distance from a cylinder about z of the given radius, running down from the given height without end; negative inside.</summary>
    internal static double FromCylinder(V p, double radius, double top)
    {
        double dr = Math.Sqrt(p.X * p.X + p.Y * p.Y) - radius, dz = p.Z - top;
        return dr <= 0 && dz <= 0 ? Math.Max(dr, dz) : Math.Sqrt(Math.Max(dr, 0) * Math.Max(dr, 0) + Math.Max(dz, 0) * Math.Max(dz, 0));
    }

    private static V Gradient(Func<V, double> f, V p)
    {
        const double e = 1e-6;
        return new V(f(p + new V(e, 0, 0)) - f(p - new V(e, 0, 0)), f(p + new V(0, e, 0)) - f(p - new V(0, e, 0)), f(p + new V(0, 0, e)) - f(p - new V(0, 0, e))) * (0.5 / e);
    }

    /// <summary>
    /// The volume added round one closed junction: <paramref name="core"/> the distance to the rounded body's core,
    /// <paramref name="other"/> to the other body, <paramref name="near"/> a point near where a ball touching both has its centre,
    /// <paramref name="step"/> how far apart the centres are taken.
    /// </summary>
    internal static double Added(Func<V, double> core, Func<V, double> other, double radius, V near, double step)
    {
        V OnCurve(V p)
        {
            for (int i = 0; i < 60; i++)
            {
                V a = Gradient(core, p), b = Gradient(other, p);
                double g1 = core(p) - 2 * radius, g2 = other(p) - radius;
                double a11 = a.Dot(a), a12 = a.Dot(b), a22 = b.Dot(b), det = a11 * a22 - a12 * a12;
                double m1 = (g1 * a22 - g2 * a12) / det, m2 = (a11 * g2 - a12 * g1) / det;
                V move = a * -m1 + b * -m2;
                p += move;
                if (move.Length < 1e-14)
                {
                    break;
                }
            }
            Assert.True(Math.Abs(core(p) - 2 * radius) < 1e-11 && Math.Abs(other(p) - radius) < 1e-11, "no centre found");
            return p;
        }

        // The centres, and at each the curve's direction
        var centres = new List<V> { OnCurve(near) };
        var along = new List<V> { Gradient(core, centres[0]).Cross(Gradient(other, centres[0])).Unit };
        while (true)
        {
            V next = OnCurve(centres[^1] + along[^1] * step);
            if (centres.Count > 3 && (next - centres[0]).Length < step)
            {
                if ((next - centres[0]).Length > 1e-3 * step)
                {
                    centres.Add(next);
                    V last = Gradient(core, next).Cross(Gradient(other, next)).Unit;
                    along.Add(last.Dot(along[^1]) < 0 ? last * -1 : last);
                }
                break;
            }
            V t = Gradient(core, next).Cross(Gradient(other, next)).Unit;
            centres.Add(next);
            along.Add(t.Dot(along[^1]) < 0 ? t * -1 : t);
            Assert.True(centres.Count < 1_000_000, "the centres' curve doesn't close");
        }
        int n = centres.Count;

        // Along a ray from a centre, how far the first of the two bodies is, and whether it's the rounded one
        (double Far, bool Rounded) Exit(V c, V d)
        {
            double F(double rho) => Math.Min(core(c + d * rho) - radius, other(c + d * rho));
            double lo = radius;
            if (F(lo) <= 0)
            {
                return (radius, true);
            }
            double hi = lo;
            do
            {
                lo = hi;
                hi += radius / 16;
                Assert.True(hi < 60 * radius, "a ray meets neither body");
            }
            while (F(hi) > 0);
            for (int i = 0; i < 52; i++)
            {
                double m = 0.5 * (lo + hi);
                if (F(m) > 0)
                {
                    lo = m;
                }
                else
                {
                    hi = m;
                }
            }
            double far = 0.5 * (lo + hi);
            V p = c + d * far;
            return (far, core(p) - radius < other(p));
        }

        var fans = new double[n];
        for (int i = 0; i < n; i++)
        {
            V c = centres[i];
            V before = centres[(i + n - 1) % n], after = centres[(i + 1) % n];
            V bend = (along[(i + 1) % n] - along[(i + n - 1) % n]) * (1 / ((after - c).Length + (c - before).Length));
            V na = Gradient(core, c).Unit * -1, nb = Gradient(other, c).Unit * -1;
            double angle = Math.Acos(Math.Clamp(na.Dot(nb), -1, 1));
            V Ray(double phi) => (na * Math.Sin(angle - phi) + nb * Math.Sin(phi)) * (1 / Math.Sin(angle));
            double Density(double phi)
            {
                V d = Ray(phi);
                double far = Exit(c, d).Far;
                return 0.5 * (far * far - radius * radius) - bend.Dot(d) * (far * far * far - radius * radius * radius) / 3;
            }

            // Where the rays stop ending on the rounded body and start ending on the other
            double from = 0, to = angle;
            for (int k = 0; k < 40; k++)
            {
                double m = 0.5 * (from + to);
                if (Exit(c, Ray(m)).Rounded)
                {
                    from = m;
                }
                else
                {
                    to = m;
                }
            }
            double turn = 0.5 * (from + to);
            fans[i] = Gauss(Density, 0, turn, 2) + Gauss(Density, turn, angle, 2);
        }
        double volume = 0;
        for (int i = 0; i < n; i++)
        {
            volume += 0.5 * (fans[i] + fans[(i + 1) % n]) * (centres[(i + 1) % n] - centres[i]).Length;
        }
        return volume;
    }

    /// <summary>
    /// What rounding the rim of a cylinder about z takes away where <paramref name="outside"/> is positive, the part of the rim's
    /// ring clear of the other body: each section of the ring along rays from its ball's centre, turned about the axis.
    /// </summary>
    internal static double Ring(double main, double top, double radius, Func<V, double> outside, int panels = 1440)
    {
        double Section(double theta)
        {
            double cos = Math.Cos(theta), sin = Math.Sin(theta);
            double Rays(double alpha)
            {
                double ca = Math.Cos(alpha), sa = Math.Sin(alpha), far = radius / Math.Max(ca, sa);
                double Out(double t)
                {
                    double u = main - radius + t * ca;
                    return outside(new V(u * cos, u * sin, top - radius + t * sa));
                }
                double Moment(double t) => (main - radius) * t * t / 2 + ca * t * t * t / 3;
                const int Pieces = 6;
                double sum = 0, a = radius;
                bool was = Out(a) > 0;
                for (int k = 1; k <= Pieces; k++)
                {
                    double b = radius + (far - radius) * k / Pieces;
                    bool now = Out(b) > 0;
                    if (now == was)
                    {
                        sum += was ? Moment(b) - Moment(a) : 0;
                    }
                    else
                    {
                        double lo = a, hi = b;
                        for (int i = 0; i < 52; i++)
                        {
                            double m = 0.5 * (lo + hi);
                            if (Out(m) > 0 == was)
                            {
                                lo = m;
                            }
                            else
                            {
                                hi = m;
                            }
                        }
                        double cut = 0.5 * (lo + hi);
                        sum += was ? Moment(cut) - Moment(a) : Moment(b) - Moment(cut);
                    }
                    (a, was) = (b, now);
                }
                return sum;
            }
            return Gauss(Rays, 0, Math.PI / 4, 3) + Gauss(Rays, Math.PI / 4, Math.PI / 2, 3);
        }
        return Gauss(Section, 0, 2 * Math.PI, panels);
    }
}
