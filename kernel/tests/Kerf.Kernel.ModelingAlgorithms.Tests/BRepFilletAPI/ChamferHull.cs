// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BRepFilletAPI;

/// <summary>
/// A chamfer past tight corners on a convex outline, worked out a way of its own (docs/chamfer-corners.md, section 1): the convex hull
/// of the outline at the chamfer's depth and its contact on the face. The contact is the outline's region eroded by the length on the
/// face, found as the intersection of the half-planes of the outline's tangents, each stepped in; the hull's section part way up is the
/// Minkowski combination of the two, found by merging their edges. Polygons of many sides stand for the curves, and two counts of sides
/// are extrapolated (Richardson), as the error goes with the square of the side. Nothing here reads how the chamfer was built.
/// </summary>
internal static class ChamferHull
{
    private readonly record struct P(double X, double Y)
    {
        public static P operator +(P a, P b) => new(a.X + b.X, a.Y + b.Y);

        public static P operator -(P a, P b) => new(a.X - b.X, a.Y - b.Y);

        public static P operator *(P a, double k) => new(a.X * k, a.Y * k);

        public double Cross(P b) => X * b.Y - Y * b.X;
    }

    /// <summary>A convex outline as its points and tangents, anticlockwise about the face's normal +z, from <paramref name="sample"/> at
    /// a given count of points along it.</summary>
    public delegate (double X, double Y, double Tx, double Ty)[] Outline(int count);

    /// <summary>The outer outline of a planar face square to z, from its edges' curves: points evenly along each edge by parameter, in
    /// proportion to its length, chained end to end and turned anticlockwise seen from +z.</summary>
    public static Outline OfFace(Face face) => count =>
    {
        Edge[] edges = [.. face.OuterWire.AllEdges()];
        double total = edges.Sum(e => Length(e));
        var runs = new List<List<(double X, double Y, double Tx, double Ty)>>();
        foreach (Edge edge in edges)
        {
            var curve = new EdgeCurveAdaptor(edge);
            double first = curve.FirstParameter(), last = curve.LastParameter();
            int n = Math.Max(4, (int)(count * Length(edge) / total));
            var run = new List<(double, double, double, double)>();
            for (int i = 0; i <= n; i++)
            {
                double t = first + (last - first) * i / n;
                curve.D1(t, out Point3D at, out Vector3D tangent);
                run.Add((at.X, at.Y, tangent.X, tangent.Y));
            }
            runs.Add(run);
        }

        // End to end: each run starts where the last ended, turned round if it runs the other way
        static List<(double X, double Y, double Tx, double Ty)> Reversed(List<(double X, double Y, double Tx, double Ty)> run) =>
            [.. Enumerable.Reverse(run).Select(p => (p.X, p.Y, -p.Tx, -p.Ty))];
        static double Gap((double X, double Y, double Tx, double Ty) p, (double X, double Y, double Tx, double Ty) q) => Math.Abs(p.X - q.X) + Math.Abs(p.Y - q.Y);
        var chain = new List<List<(double X, double Y, double Tx, double Ty)>> { runs[0] };
        runs.RemoveAt(0);
        while (runs.Count > 0)
        {
            var end = chain[^1][^1];
            int next = runs.FindIndex(r => Gap(r[0], end) < 1e-6 || Gap(r[^1], end) < 1e-6);
            chain.Add(Gap(runs[next][0], end) < 1e-6 ? runs[next] : Reversed(runs[next]));
            runs.RemoveAt(next);
        }
        var samples = chain.SelectMany(r => r.Take(r.Count - 1)).ToArray();
        double twice = 0;
        for (int i = 0; i < samples.Length; i++)
        {
            var (x0, y0, _, _) = samples[i];
            var (x1, y1, _, _) = samples[(i + 1) % samples.Length];
            twice += x0 * y1 - x1 * y0;
        }
        if (twice < 0)
        {
            // Clockwise: the same points the other way, each with its tangent turned round
            samples = [.. samples.Reverse().Select(s => (s.X, s.Y, -s.Tx, -s.Ty))];
        }
        return samples;
    };

    private static double Length(Edge edge)
    {
        var curve = new EdgeCurveAdaptor(edge);
        double first = curve.FirstParameter(), last = curve.LastParameter(), length = 0;
        Point3D previous = curve.Value(first);
        for (int i = 1; i <= 64; i++)
        {
            Point3D next = curve.Value(first + (last - first) * i / 64);
            length += previous.DistanceTo(next);
            previous = next;
        }
        return length;
    }

    /// <summary>What a chamfer a on the face and down sides drafted outwards by t over a depth h removes from a convex prism's end:
    /// the frustum of the sides, less the hull of the outline grown by t at depth h and the contact on the face. A drafted side's section
    /// is the outline grown, its tangents moved out, which is mitred at a sharp corner and round at an arc; its area is quadratic in the
    /// depth, so Simpson's rule is exact for the frustum as for the hull. With t = 0 the sides are square, and h is b.</summary>
    public static double Removed(Outline outline, double a, double h, double t = 0)
    {
        double At(int count)
        {
            (double X, double Y, double Tx, double Ty)[] samples = outline(count);
            P[] region = Polygon(samples), contact = Eroded(samples, a), low = Grown(samples, region, t);
            double Section(double lambda) => Area(Minkowski(Scaled(low, 1 - lambda), Scaled(contact, lambda)));
            double frustum = h / 6 * (Area(region) + 4 * Area(Grown(samples, region, t / 2)) + Area(low));
            double hull = h / 6 * (Section(0) + 4 * Section(0.5) + Section(1));
            return frustum - hull;
        }
        double coarse = At(8000), fine = At(16000);
        return (4 * fine - coarse) / 3;
    }

    /// <summary>The area of the chamfered prism's section at depth z below the face, between 0 and h: the hull's, (1 - λ) low + λ E
    /// with λ = 1 - z / h.</summary>
    public static double SectionArea(Outline outline, double a, double h, double z, double t = 0)
    {
        double At(int count)
        {
            (double X, double Y, double Tx, double Ty)[] samples = outline(count);
            P[] low = Grown(samples, Polygon(samples), t);
            double lambda = 1 - z / h;
            return Area(Minkowski(Scaled(low, 1 - lambda), Scaled(Eroded(samples, a), lambda)));
        }
        return (4 * At(16000) - At(8000)) / 3;
    }

    // The outline grown by t (shrunk where t is negative): the half-planes of its tangents moved out by t
    private static P[] Grown((double X, double Y, double Tx, double Ty)[] samples, P[] region, double t) => t == 0 ? region : Eroded(samples, -t);

    private static P[] Polygon((double X, double Y, double Tx, double Ty)[] samples) => [.. samples.Select(s => new P(s.X, s.Y))];

    private static P[] Scaled(P[] polygon, double k) => [.. polygon.Select(p => p * k)];

    private static double Area(P[] polygon)
    {
        double twice = 0;
        for (int i = 0; i < polygon.Length; i++)
        {
            twice += polygon[i].Cross(polygon[(i + 1) % polygon.Length]);
        }
        return twice / 2;
    }

    // The region eroded by a: the half-planes left of each tangent line, stepped in by a, intersected, by the sorted-lines method
    private static P[] Eroded((double X, double Y, double Tx, double Ty)[] samples, double a)
    {
        var lines = new List<(P Point, P Direction, double Angle)>();
        foreach ((double x, double y, double tx, double ty) in samples)
        {
            double length = Math.Sqrt(tx * tx + ty * ty);
            P d = new(tx / length, ty / length);
            lines.Add((new P(x, y) + new P(-d.Y, d.X) * a, d, Math.Atan2(d.Y, d.X)));
        }
        lines.Sort((u, v) => u.Angle.CompareTo(v.Angle));
        static bool Outside((P Point, P Direction, double Angle) line, P p) => line.Direction.Cross(p - line.Point) < -1e-15;
        static P Meet((P Point, P Direction, double Angle) u, (P Point, P Direction, double Angle) v)
        {
            double cross = u.Direction.Cross(v.Direction);
            return u.Point + u.Direction * ((v.Point - u.Point).Cross(v.Direction) / cross);
        }
        var deque = new (P Point, P Direction, double Angle)[lines.Count];
        var corners = new P[lines.Count];
        int head = 0, tail = 0;
        deque[0] = lines[0];
        for (int i = 1; i < lines.Count; i++)
        {
            var line = lines[i];
            while (tail > head && Outside(line, corners[tail - 1]))
            {
                tail--;
            }
            while (tail > head && Outside(line, corners[head]))
            {
                head++;
            }
            if (Math.Abs(line.Direction.Cross(deque[tail].Direction)) < 1e-15)
            {
                // Parallel and alike: keep the one further in
                if (!Outside(deque[tail], line.Point))
                {
                    deque[tail] = line;
                }
            }
            else
            {
                deque[++tail] = line;
            }
            if (tail > head)
            {
                corners[tail - 1] = Meet(deque[tail - 1], deque[tail]);
            }
        }
        while (tail > head && Outside(deque[head], corners[tail - 1]))
        {
            tail--;
        }
        while (tail > head && Outside(deque[tail], corners[head]))
        {
            head++;
        }
        corners[tail] = Meet(deque[tail], deque[head]);
        return [.. corners[head..(tail + 1)]];
    }

    // The Minkowski sum of two convex polygons, anticlockwise: their edges merged by direction
    private static P[] Minkowski(P[] one, P[] two)
    {
        static P[] FromLowest(P[] polygon)
        {
            int start = 0;
            for (int i = 1; i < polygon.Length; i++)
            {
                if (polygon[i].Y < polygon[start].Y || (polygon[i].Y == polygon[start].Y && polygon[i].X < polygon[start].X))
                {
                    start = i;
                }
            }
            return [.. polygon[start..], .. polygon[..start]];
        }
        P[] p = FromLowest(one), q = FromLowest(two);
        var sum = new List<P>(p.Length + q.Length);
        int i = 0, j = 0;
        while (i < p.Length || j < q.Length)
        {
            sum.Add(p[i % p.Length] + q[j % q.Length]);
            P ep = p[(i + 1) % p.Length] - p[i % p.Length], eq = q[(j + 1) % q.Length] - q[j % q.Length];
            double cross = ep.Cross(eq);
            if (j >= q.Length || (i < p.Length && cross > 0))
            {
                i++;
            }
            else if (i >= p.Length || cross < 0)
            {
                j++;
            }
            else
            {
                i++;
                j++;
            }
        }
        return [.. sum];
    }
}
