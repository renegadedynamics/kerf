// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

// Stage 4a (unit 15N; fillet-design.md, section 11): a rolling ball between a plane or a cylinder and a cylinder, along a chain of
// edges whose two faces turn tangent at both its ends, so that the fillet narrows to nothing there, as at an equal tee's saddle.
// ChFi3d's walk fails going towards such a point (decision 1925); here each section is solved where it lies, the ball's centre at
// the radius from both surfaces in the plane square to the edge, and the fillet is the arcs between its contacts. The faces keep
// their vertices, so each chain's edges give way to one face, and its two faces each to a face with the contact line in their
// place. What it makes is proved apart from how: the ball put back at points of the face it made touches both surfaces, the face
// is tangent to them along its edges, the ball doesn't reach into the result elsewhere, and the volume is the input's and the
// region between the surfaces and the ball, integrated on the exact sections.
internal static partial class FilletLimits
{
    // Counted for the tests and the sweeps, as EdgeCrossings counts its: blends built and returned, and those the ball test refused
    internal static long s_blendsBuilt;
    internal static long s_blendsRefused;

    // Sections per edge to start with, and the most
    private const int FirstSections = 16;
    private const int MostSections = 1024;

    private static EngineOutcome? BuildBlends(Shape shape, FilletSpec[] specs)
    {
        if (shape.AllSolids().Length != 1 || shape.Type is not (ShapeType.Solid or ShapeType.Compound) || specs.Length == 0)
        {
            return null;
        }
        double radius = specs[0].Radius;
        if (specs.Any(s => s.Radius != radius))
        {
            return null;
        }
        var neighbours = new Neighbours(shape);
        var wanted = new HashSet<Shape>(ShapeHasher.Instance);
        foreach (FilletSpec spec in specs)
        {
            IReadOnlyList<Face> faces = neighbours.Faces(spec.Edge);
            Edge forward = ShapeCasts.Edge(spec.Edge.Oriented(Orientation.Forward));

            // A seam has nothing to round, and ChFi3d passes over it
            if (faces.Count >= 1 && faces.All(f => f.IsSame(faces[0])))
            {
                continue;
            }
            if (faces.Count != 2 || Tool.Degenerated(forward))
            {
                return null;
            }
            wanted.Add(forward);
        }
        if (wanted.Count == 0)
        {
            return null;
        }
        if (Chains(neighbours, wanted, radius) is not { } chains)
        {
            return null;
        }
        Cancellation.ThrowIfRequested();
        var blends = new List<Blend>();
        foreach (Chain chain in chains)
        {
            if (Blend.Make(chain, radius) is not { } blend)
            {
                return null;
            }
            blends.Add(blend);
        }
        if (Assemble(shape, blends) is not { } assembled)
        {
            return null;
        }
        (Shape made, History history) = assembled;
        if (Validity.Problem(made, "The fillet") is not null || !BallHolds(shape, made, blends, radius))
        {
            Interlocked.Increment(ref s_blendsRefused);
            return null;
        }
        Interlocked.Increment(ref s_blendsBuilt);
        Shape built = made;
        return new EngineOutcome(true, () => built, history, Valid: true);
    }

    // A plane, a cylinder or (for unit 15X) a surface extruded from a curve, with the side the solid lies on: distances are signed
    // along the face's outward normal
    private sealed class Support
    {
        public required Face Face { get; init; }
        public required bool IsCylinder { get; init; }
        public required V Origin { get; init; }
        public required V Axis { get; init; } // a cylinder's axis, a plane's normal, an extrusion's direction
        public required V X { get; init; }
        public required V Y { get; init; }
        public required double Rho { get; init; }
        public double Sign { get; set; } = 1;

        // An extrusion's curve, and points of it seen along the direction, to start finding the nearest from
        public Curve3D? Profile { get; init; }
        private double[] _samples = [];
        private V[] _sampled = [];

        public bool IsExtrusion => Profile is not null;

        public static Support? Of(Face face, bool extrusions = false)
        {
            Surface? surface = Tool.Surface(face, out Placement placement);
            if (surface is null)
            {
                return null;
            }
            if (!placement.IsIdentity)
            {
                surface = (Surface)surface.Transformed(placement.Transformation);
            }
            if (extrusions && surface is ExtrusionSurface extrusion)
            {
                return Extruded(face, extrusion);
            }
            (bool cylinder, double rho, CoordinateSystem frame) = surface switch
            {
                CylindricalSurface c => (true, c.Radius, c.Position),
                PlaneSurface p => (false, 0.0, p.Position),
                _ => (false, -1.0, default),
            };
            if (rho < 0)
            {
                return null;
            }
            return new Support
            {
                Face = face, IsCylinder = cylinder, Rho = rho, Origin = V.Of(frame.Location), Axis = V.Of(frame.Direction),
                X = V.Of(frame.XDirection), Y = V.Of(frame.YDirection),
            };
        }

        private static Support? Extruded(Face face, ExtrusionSurface surface)
        {
            Curve3D profile = surface.BasisCurve;
            double first = profile.FirstParameter, last = profile.LastParameter;
            if (double.IsInfinity(first) || double.IsInfinity(last) || profile.IsPeriodic || last <= first)
            {
                return null;
            }
            V axis = V.Of(surface.Direction);
            const int count = 128;
            var samples = new double[count + 1];
            var sampled = new V[count + 1];
            for (int i = 0; i <= count; i++)
            {
                samples[i] = first + (last - first) * i / count;
                V c = V.Of(profile.EvalD0(samples[i]));
                sampled[i] = c - axis * c.Dot(axis);
            }
            return new Support
            {
                Face = face, IsCylinder = false, Rho = 0, Origin = default, Axis = axis, X = default, Y = default, Profile = profile,
                _samples = samples, _sampled = sampled,
            };
        }

        // An extrusion's nearest point of its curve, seen along the direction: the curve's parameter, point and tangent there
        private (double U, V Point, V Tangent) Nearest(V p)
        {
            V flat = p - Axis * p.Dot(Axis);
            int best = 0;
            double nearest = double.MaxValue;
            for (int i = 0; i < _sampled.Length; i++)
            {
                double d = (_sampled[i] - flat).Dot(_sampled[i] - flat);
                if (d < nearest)
                {
                    (best, nearest) = (i, d);
                }
            }
            double u = _samples[best];
            double first = _samples[0], last = _samples[^1], step = (last - first) / (_samples.Length - 1);
            for (int iteration = 0; iteration < 40; iteration++)
            {
                Curve3D.ResD2 d2 = Profile!.EvalD2(u);
                V c = V.Of(d2.Point), c1 = V.Of(d2.D1), c2 = V.Of(d2.D2);
                V gap = c - Axis * c.Dot(Axis) - flat;
                V t1 = c1 - Axis * c1.Dot(Axis), t2 = c2 - Axis * c2.Dot(Axis);
                double g = gap.Dot(t1), dg = t1.Dot(t1) + gap.Dot(t2);
                if (dg <= 0)
                {
                    break;
                }
                double du = Math.Clamp(g / dg, -step, step);
                u -= du;
                if (Math.Abs(du) <= 1e-15 * Math.Max(1, Math.Abs(u)))
                {
                    break;
                }
            }
            Curve3D.ResD1 d1 = Profile!.EvalD1(u);
            return (u, V.Of(d1.Point), V.Of(d1.D1));
        }

        private V Radial(V p)
        {
            V d = p - Origin;
            return (d - Axis * d.Dot(Axis)).Unit;
        }

        public double Distance(V p)
        {
            if (IsExtrusion)
            {
                (_, V c, V t) = Nearest(p);
                return Sign * (p - c).Dot(t.Cross(Axis).Unit);
            }
            V d = p - Origin;
            return Sign * (IsCylinder ? (d - Axis * d.Dot(Axis)).Length - Rho : d.Dot(Axis));
        }

        // The outward normal of the surface through p at the same distance
        public V Normal(V p)
        {
            if (IsExtrusion)
            {
                return Nearest(p).Tangent.Cross(Axis).Unit * Sign;
            }
            return (IsCylinder ? Radial(p) : Axis) * Sign;
        }

        public V Foot(V p)
        {
            if (IsExtrusion)
            {
                (_, V c, _) = Nearest(p);
                return c + Axis * (p - c).Dot(Axis);
            }
            V d = p - Origin;
            return IsCylinder ? Origin + Axis * d.Dot(Axis) + Radial(p) * Rho : p - Axis * d.Dot(Axis);
        }

        public (double U, double V) Parameters(V p)
        {
            if (IsExtrusion)
            {
                (double u, V c, _) = Nearest(p);
                return (u, (p - c).Dot(Axis));
            }
            V d = p - Origin;
            return IsCylinder ? (Math.Atan2(d.Dot(Y), d.Dot(X)), d.Dot(Axis)) : (d.Dot(X), d.Dot(Y));
        }

        // The point at parameters, and its derivative in u; the derivative in v is Along
        public V At(double u, double v, out V du)
        {
            if (IsExtrusion)
            {
                Curve3D.ResD1 d1 = Profile!.EvalD1(u);
                du = V.Of(d1.D1);
                return V.Of(d1.Point) + Axis * v;
            }
            if (IsCylinder)
            {
                du = (Y * Math.Cos(u) - X * Math.Sin(u)) * Rho;
                return Origin + (X * Math.Cos(u) + Y * Math.Sin(u)) * Rho + Axis * v;
            }
            du = X;
            return Origin + X * u + Y * v;
        }

        public V Along => IsCylinder || IsExtrusion ? Axis : Y;
    }

    // Edges in order between two vertices where the faces are tangent; Forward[i] when edge i's parameter runs along the chain
    private sealed record Chain(Edge[] Edges, bool[] Forward, Support One, Support Two, double Side, Vertex Start, Vertex End);

    // The selected edges as chains, or null if any isn't in one this can build
    private static List<Chain>? Chains(Neighbours neighbours, HashSet<Shape> wanted, double radius)
    {
        var chains = new List<Chain>();
        var used = new HashSet<Shape>(ShapeHasher.Instance);
        var supports = new Dictionary<Shape, Support>(ShapeHasher.Instance);
        Support? SupportOf(Face face)
        {
            if (!supports.TryGetValue(face, out Support? s))
            {
                if (Support.Of(face) is not { } made)
                {
                    return null;
                }
                supports[face] = s = made;
            }
            return s;
        }

        // Each edge's faces, the same two for the chain, and whether the faces are tangent at a vertex
        (Face, Face)? FacesOf(Edge e) => neighbours.Faces(e) is [var a, var b] && !a.IsSame(b) ? (a, b) : null;
        bool Pinched(Support one, Support two, Vertex v)
        {
            V p = V.Of(Tool.Pnt(v));
            return (one.Normal(one.Foot(p)) - two.Normal(two.Foot(p))).Length * radius <= Tool.Tolerance(v);
        }

        foreach (Shape item in wanted)
        {
            Edge first = ShapeCasts.Edge(item);
            if (used.Contains(first) || FacesOf(first) is not var (fa, fb))
            {
                continue;
            }
            if (SupportOf(fa) is not { } one || SupportOf(fb) is not { } two || !(one.IsCylinder || two.IsCylinder))
            {
                return null;
            }
            if (!Signed(one, first, fa) || !Signed(two, first, fb))
            {
                return null;
            }
            TypeOfConcavity kind = global::Kerf.Engine.Fillets.ChFi3d.DefineConnectType(first, fa, fb, 1.0e-5, false);
            if (kind is not (TypeOfConcavity.Convex or TypeOfConcavity.Concave))
            {
                return null;
            }
            double side = kind == TypeOfConcavity.Convex ? -1 : 1;

            // Back from this edge to a pinch, then forward from there to the other
            // A closed edge whose faces are tangent at its vertex is a chain of its own, as where a branch is flush with the side
            Exploration.Vertices(first, out Vertex v0, out Vertex v1);
            if (v0.IsSame(v1))
            {
                if (!Pinched(one, two, v0))
                {
                    return null;
                }
                used.Add(first);
                chains.Add(new Chain([first], [true], one, two, side, v0, v0));
                continue;
            }
            Edge current = first;
            Vertex back = v0;
            for (int guard = 0; !Pinched(one, two, back); guard++)
            {
                if (guard > wanted.Count || Across(current, back) is not { } previous)
                {
                    return null;
                }
                current = previous;
                back = Other(current, back);
            }
            Vertex start = back;
            var edges = new List<Edge>();
            var forward = new List<bool>();
            Vertex at = start;
            for (int guard = 0; ; guard++)
            {
                if (guard > wanted.Count || !used.Add(current))
                {
                    return null;
                }
                Exploration.Vertices(current, out Vertex a, out _);
                edges.Add(current);
                forward.Add(a.IsSame(at));
                at = Other(current, at);
                if (Pinched(one, two, at))
                {
                    break;
                }
                if (Across(current, at) is not { } next)
                {
                    return null;
                }
                current = next;
            }
            if (at.IsSame(start))
            {
                return null;
            }
            chains.Add(new Chain([.. edges], [.. forward], one, two, side, start, at));

            // Past a vertex of only two edges, both selected, of the same faces: the next of the chain
            Edge? Across(Edge e, Vertex v)
            {
                IReadOnlyList<Edge> there = neighbours.Edges(v);
                Edge[] others = [.. there.Where(x => !x.IsSame(e)).Select(x => ShapeCasts.Edge(x.Oriented(Orientation.Forward)))];
                if (there.Count(x => !Tool.Degenerated(x)) != 2 || others.Length != 1 || !wanted.Contains(others[0])
                    || FacesOf(others[0]) is not var (ga, gb) || !((ga.IsSame(fa) && gb.IsSame(fb)) || (ga.IsSame(fb) && gb.IsSame(fa))))
                {
                    return null;
                }
                return others[0];
            }
        }
        return used.Count == wanted.Count ? chains : null;
    }

    private static Vertex Other(Edge e, Vertex v)
    {
        Exploration.Vertices(e, out Vertex a, out Vertex b);
        return a.IsSame(v) ? b : a;
    }

    // Which way the face's outward normal points against the surface's own, at the middle of the edge
    private static bool Signed(Support support, Edge edge, Face face)
    {
        Tool.Range(edge, out double first, out double last);
        double t = 0.5 * (first + last);
        Point2D uv = new EdgeCurve2DAdaptor(edge, face).Value(t);
        var surface = new FaceSurfaceAdaptor(face);
        surface.D1(uv.X, uv.Y, out Point3D p, out Vector3D du, out Vector3D dv);
        V n = V.Of(du).Cross(V.Of(dv));
        if (n.Length < 1e-12)
        {
            return false;
        }
        if (face.Orientation == Orientation.Reversed)
        {
            n = -n;
        }
        double dot = n.Unit.Dot(support.Normal(V.Of(p)));
        if (Math.Abs(dot) < 0.5)
        {
            return false;
        }
        support.Sign = dot > 0 ? 1 : -1;
        return true;
    }
}
