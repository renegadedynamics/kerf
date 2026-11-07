// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

/// <summary>
/// Straight edges between planes that meet, or that end on a slant, with one radius, once ChFi3d has failed (decision 1908). Each edge
/// is a prism of the section, as in <see cref="FilletLimits"/>; where three selected edges meet there's a patch of a sphere, where two
/// meet with a sharp edge the third, their prisms run past each other, and where they meet round a re-entrant corner the ball pivots.
/// The answer is checked against the volume and area those give, worked out from the planes alone.
/// </summary>
internal static partial class FilletLimits
{
    private enum EndKind
    {
        /// <summary>On the third face, which it runs past.</summary>
        Free,

        /// <summary>On the side of another selected edge, sharp between them.</summary>
        Mitre,

        /// <summary>Square to the edge, through the centre of a ball, or at a pivot.</summary>
        Square,
    }

    /// <summary>Where a prism ends: on a plane through <paramref name="At"/> along the edge, of the normal given.</summary>
    private readonly record struct PrismEnd(EndKind Kind, double At, Direction3D Normal)
    {
        /// <summary>Where along the edge a point of the section at this offset from the edge meets this end.</summary>
        public double Along(in Vector3D offset, in Direction3D along) => At - new Vector3D(Normal).Dot(offset) / Normal.Dot(along);
    }

    /// <summary>One edge's section, and how far it runs.</summary>
    private sealed record Prism(Cutter Cut, PrismEnd First, PrismEnd Last)
    {
        private Corner Corner => Cut.Corner;

        private double R => Cut.Radius!.Value;

        private double On => Cut.On1;

        private double HalfTurn => (Math.PI - Corner.Angle) / 2;

        private Vector3D ToCentre => new(Corner.Start, Cut.BallCentre);

        private Vector3D TowardsEdge => ToCentre * (-1 / ToCentre.Magnitude);

        private double Between(in Vector3D offset) => Last.Along(offset, Corner.Along) - First.Along(offset, Corner.Along);

        /// <summary>Nothing of it is left: its two ends are one, as at a ball's plane.</summary>
        public bool IsEmpty => Between(new Vector3D(0, 0, 0)) <= Tolerance && Between(new Vector3D(Corner.One.Inward) * On) <= Tolerance && Between(new Vector3D(Corner.Two.Inward) * On) <= Tolerance;

        public double SectionArea => R * On - R * R * HalfTurn;

        /// <summary>A prism cut by two planes: the section's area by the length at its centroid.</summary>
        public double Volume
        {
            get
            {
                Vector3D p1 = new Vector3D(Corner.One.Inward) * On, p2 = new Vector3D(Corner.Two.Inward) * On, c = ToCentre;
                Vector3D moment = (p1 + c) * (R * On / 6) + (p2 + c) * (R * On / 6) - (c + TowardsEdge * (2 * R * Math.Sin(HalfTurn) / (3 * HalfTurn))) * (R * R * HalfTurn);
                return SectionArea * Between(moment * (1 / SectionArea));
            }
        }

        /// <summary>The new face's area, less the two strips it stands on, less or more what a free end's face loses or gains.</summary>
        public double AreaChange
        {
            get
            {
                Vector3D p1 = new Vector3D(Corner.One.Inward) * On, p2 = new Vector3D(Corner.Two.Inward) * On;
                double area = 2 * HalfTurn * R * Between(ToCentre + TowardsEdge * (R * Math.Sin(HalfTurn) / HalfTurn))
                              - On * Between(p1 * 0.5) - On * Between(p2 * 0.5);
                foreach (PrismEnd end in new[] { First, Last })
                {
                    if (end.Kind == EndKind.Free)
                    {
                        area += (Corner.IsConvex ? -1 : 1) * SectionArea / Math.Abs(end.Normal.Dot(Corner.Along));
                    }
                }
                return area;
            }
        }

        /// <summary>The tool: flush for what's added, standing out past the faces and the ends for what's cut.</summary>
        public Shape Tool()
        {
            if (!Corner.IsConvex)
            {
                return Cut.PrismTool(0, Corner.Length, 0);
            }
            Vector3D p1 = new Vector3D(Corner.One.Inward) * On, p2 = new Vector3D(Corner.Two.Inward) * On;
            double from = Reach(First, p1, p2, -1), to = Reach(Last, p1, p2, 1);
            return Cut.PrismTool(from, to, On);
        }

        // A square end is exact; another goes past its plane everywhere the section reaches, and only as far as it must
        private double Reach(in PrismEnd end, in Vector3D p1, in Vector3D p2, int sign)
        {
            if (end.Kind == EndKind.Square)
            {
                return end.At;
            }
            double a = end.Along(p1, Corner.Along), b = end.Along(p2, Corner.Along), c = end.At;
            return sign > 0 ? Math.Max(a, Math.Max(b, c)) + On : Math.Min(a, Math.Min(b, c)) - On;
        }
    }

    /// <summary>A patch of a sphere, where three selected edges meet at a convex corner.</summary>
    private sealed record BallCorner(Vertex Vertex, Point3D At, Point3D Centre, double R, Direction3D[] Normals, Point3D[] Feet)
    {
        // The feet on the edges, in the order of the normals' pairs: 0 between normals 0 and 1, 1 between 1 and 2, 2 between 2 and 0
        private double Quad(int normal)
        {
            Point3D contact = Centre.Translated(new Vector3D(Normals[normal]) * R);
            Point3D a = Feet[normal], b = Feet[(normal + 2) % 3];
            return Triangle(At, a, contact) + Triangle(At, contact, b);
        }

        private static double Triangle(in Point3D a, in Point3D b, in Point3D c) => new Vector3D(a, b).Crossed(new Vector3D(a, c)).Magnitude / 2;

        // The solid angle of the triangle of normals (Van Oosterom and Strackee)
        private double SolidAngle
        {
            get
            {
                Vector3D n1 = new(Normals[0]), n2 = new(Normals[1]), n3 = new(Normals[2]);
                double triple = Math.Abs(n1.Dot(n2.Crossed(n3)));
                return 2 * Math.Atan2(triple, 1 + n1.Dot(n2) + n2.Dot(n3) + n3.Dot(n1));
            }
        }

        private double Quads => Quad(0) + Quad(1) + Quad(2);

        /// <summary>The pyramids from the centre on each quad, less the sector of the sphere.</summary>
        public double Volume => R / 3 * Quads - R * R * R * SolidAngle / 3;

        public double AreaChange => R * R * SolidAngle - Quads;

        public bool IsOnBall(FaceSurfaceAdaptor surface)
        {
            if (surface.GetType() != SurfaceKind.Sphere)
            {
                return false;
            }
            Sphere3D sphere = surface.Sphere();
            return Math.Abs(sphere.Radius - R) <= Tolerance && sphere.Location.DistanceTo(Centre) <= Tolerance;
        }

        /// <summary>This corner's patch: on its ball, which others can share, as a capsule's four at an end do, and between its normals.</summary>
        public bool IsNewFace(Face face)
        {
            if (!IsOnBall(new FaceSurfaceAdaptor(face)))
            {
                return false;
            }
            Vector3D a = new(Normals[0]), b = new(Normals[1]), c = new(Normals[2]);
            double det = a.Dot(b.Crossed(c));
            foreach (Vertex vertex in face.AllVertices())
            {
                Vector3D p = new(Centre, Kerf.Engine.Topology.Brep.Tool.Pnt(vertex));
                if (p.Dot(b.Crossed(c)) / det < -1e-6 * R || p.Dot(c.Crossed(a)) / det < -1e-6 * R || p.Dot(a.Crossed(b)) / det < -1e-6 * R)
                {
                    return false;
                }
            }
            return true;
        }

        /// <summary>What's outside the ball, inside the rays from its centre along the normals, and out past the corner.</summary>
        public Shape Tool()
        {
            Vector3D toCorner = new(Centre, At);
            double reach = 2 * (toCorner.Magnitude + R);
            Point3D q0 = Centre.Translated(new Vector3D(Normals[0]) * reach), q1 = Centre.Translated(new Vector3D(Normals[1]) * reach), q2 = Centre.Translated(new Vector3D(Normals[2]) * reach);
            Shape block = Tetrahedron(Centre, q0, q1, q2);

            // With the sphere's poles and seam away from the patch
            Direction3D middle = new(new Vector3D(Normals[0]) + new Vector3D(Normals[1]) + new Vector3D(Normals[2]));
            Direction3D axis = new(new Vector3D(Normals[0]).Crossed(new Vector3D(Normals[1])) + new Vector3D(Normals[1]).Crossed(new Vector3D(Normals[2])) * 0.5);
            axis = new Direction3D(new Vector3D(axis) - new Vector3D(middle) * new Vector3D(axis).Dot(new Vector3D(middle)));
            Shape ball = new MakeSphere(new Frame3D(Centre, axis, middle.Reversed()), R).Shape();
            return Boolean(new Kerf.Engine.Booleans.Api.Cut(), block, [ball], out _) ?? throw new InvalidOperationException("The corner's tool couldn't be built.");
        }
    }

    private static Shape Tetrahedron(in Point3D apex, in Point3D a, in Point3D b, in Point3D c)
    {
        Point3D[] points = [apex, a, b, c];
        Vertex[] vertices = [.. points.Select(p => new MakeVertex(p).Vertex())];
        var edges = new Dictionary<(int, int), Edge>();
        Edge Line(int i, int j) => edges.TryGetValue((Math.Min(i, j), Math.Max(i, j)), out Edge e) ? e : edges[(Math.Min(i, j), Math.Max(i, j))] = new MakeEdge(vertices[Math.Min(i, j)], vertices[Math.Max(i, j)]).Edge();
        Point3D middle = new((apex.X + a.X + b.X + c.X) / 4, (apex.Y + a.Y + b.Y + c.Y) / 4, (apex.Z + a.Z + b.Z + c.Z) / 4);
        var builder = new BrepBuilder();
        Shell shell = default;
        builder.MakeShell(ref shell);
        foreach ((int i, int j, int k) in new[] { (0, 1, 2), (0, 2, 3), (0, 3, 1), (1, 2, 3) })
        {
            var wire = new MakeWire();
            wire.Add(Line(i, j));
            wire.Add(Line(j, k));
            wire.Add(Line(k, i));
            Vector3D normal = new Vector3D(points[i], points[j]).Crossed(new Vector3D(points[i], points[k]));
            if (normal.Dot(new Vector3D(middle, points[i])) < 0)
            {
                normal *= -1;
            }
            builder.Add(shell, new MakeFace(new Plane3D(points[i], new Direction3D(normal)), wire.Wire()).Face());
        }
        shell.Closed(true);
        return new MakeSolid(shell).Shape();
    }

    /// <summary>The ball pivoting on a re-entrant edge, between two selected edges whose sides are square to their common face: the
    /// section of either, turned about the edge through the angle outside the corner.</summary>
    private sealed record Pivot(Vertex Vertex, Point3D At, Direction3D Up, Direction3D StartOut, double Turn, double R)
    {
        // A section r by r, with the corner on the axis
        public double Volume => Turn * R * R * R * (5.0 / 6 - Math.PI / 4);

        public double AreaChange => Turn * R * R * (Math.PI / 2 - 1 - 0.5);

        private Point3D Centre => At.Translated(new Vector3D(StartOut) * R + new Vector3D(Up) * -R);

        public bool IsNewFace(FaceSurfaceAdaptor surface)
        {
            if (surface.GetType() != SurfaceKind.Torus)
            {
                return false;
            }
            Torus3D torus = surface.Torus();
            Point3D foot = At.Translated(new Vector3D(Up) * -R);
            return Math.Abs(torus.MinorRadius - R) <= Tolerance && Math.Abs(torus.MajorRadius - R) <= Tolerance && torus.Location.DistanceTo(foot) <= Tolerance;
        }

        public Shape Tool()
        {
            Vector3D up = new(Up), start = new(StartOut);
            Point3D below = At.Translated(up * -R), onTop = At.Translated(start * R), above = onTop.Translated(up * R), overAxis = At.Translated(up * R);
            Point3D centre = Centre;
            var wire = new MakeWire();
            wire.Add(ArcOnTorusFrame(centre, StartOut, Up, R, onTop, below));
            wire.Add(new MakeEdge(below, overAxis).Edge());
            wire.Add(new MakeEdge(overAxis, above).Edge());
            wire.Add(new MakeEdge(above, onTop).Edge());
            Direction3D normal = new(new Vector3D(Up).Crossed(start));
            Face section = new MakeFace(new Plane3D(At, normal), wire.Wire()).Face();
            var axis = new Axis3D(At, Up);
            Shape tool = new ApiMakeRevol(section, axis, Turn).Shape();
            return PutOnTorus(tool, below, Up, StartOut, R, R);
        }
    }

    /// <summary>A mitre between two prisms of one section's shape: what's counted twice.</summary>
    /// <param name="R">The radius.</param>
    /// <param name="On">The contact's distance on their common face.</param>
    /// <param name="Dihedral">The angle of each corner.</param>
    /// <param name="Crossing">The angle between the edges on that face.</param>
    private sealed record Mitre(double R, double On, double Dihedral, double Crossing)
    {
        // Gauss-Legendre on [-1, 1]. Strips of depth t overlap in a parallelogram of their two widths: a smooth integrand by the arc's angle
        private static readonly double[] Nodes = [0.1252334085114689, 0.3678314989981802, 0.5873179542866175, 0.7699026741943047, 0.9041172563704749, 0.9815606342467192];
        private static readonly double[] Weights = [0.2491470458134028, 0.2334925365383548, 0.2031674267230659, 0.1600783285433462, 0.1069393259953184, 0.0471753363865118];

        private double Width(double angle) => On - R * Math.Sin(angle) - R * (1 - Math.Cos(angle)) / Math.Tan(Dihedral);

        private double Integral(Func<double, double> f)
        {
            double top = Math.PI - Dihedral, sum = 0;
            for (int i = 0; i < Nodes.Length; i++)
            {
                sum += Weights[i] * (f((1 + Nodes[i]) * top / 2) + f((1 - Nodes[i]) * top / 2));
            }
            return sum * top / 2;
        }

        public double Volume
        {
            get
            {
                double self = R, across = Math.Sin(Crossing);
                return Integral(a => Width(a) * Width(a) / across * self * Math.Sin(a));
            }
        }

        public double AreaChange
        {
            get
            {
                double self = R, across = Math.Sin(Crossing);
                return On * On / across - 2 * Integral(a => Width(a) / across * self);
            }
        }
    }

    // A shape whose faces are split in one plane, or edges in one line, has vertices that stop nothing: ChFi3d's tangent chains run through
    // them. So the same, on the shape with them gone, each edge now being all of what it was part of
    private static EngineOutcome? BuildCornersOnUnified(Shape shape, FilletSpec[] specs)
    {
        var unifier = new UnifySameDomain(shape, true, true, true);
        unifier.SetSafeInputMode(true);
        unifier.AllowInternalEdges(false);
        unifier.Build();
        Shape unified = unifier.InternalShape();
        if (unified.AllFaces().Length == shape.AllFaces().Length && unified.AllEdges().Length == shape.AllEdges().Length)
        {
            return null;
        }
        History merged = unifier.InternalHistory()!;
        var own = new List<Edge>();
        foreach (FilletSpec spec in specs)
        {
            foreach (Edge edge in unified.AllEdges().Where(e => e.IsSame(spec.Edge) || merged.Modified(spec.Edge).Any(m => m.IsSame(e))))
            {
                if (!own.Any(e => e.IsSame(edge)))
                {
                    own.Add(edge);
                }
            }
        }
        if (own.Count == 0 || BuildCorners(unified, [.. own.Select(e => new FilletSpec(e, specs[0].Radius))]) is not { } outcome)
        {
            return null;
        }
        var history = new History(merged);
        history.Merge(outcome.History);

        // Each part of a merged edge made what the whole made
        foreach (FilletSpec spec in specs)
        {
            foreach (Edge whole in own.Where(e => e.IsSame(spec.Edge) || merged.Modified(spec.Edge).Any(m => m.IsSame(e))))
            {
                foreach (Shape face in outcome.History!.Generated(whole))
                {
                    history.AddGenerated(spec.Edge, face);
                }
            }
            history.Remove(spec.Edge);
        }
        return outcome with { History = history };
    }

    private static EngineOutcome? BuildCorners(Shape shape, FilletSpec[] specs)
    {
        if (shape.AllSolids().Length != 1 || shape.Type is not (ShapeType.Solid or ShapeType.Compound) || specs.Any(s => s.Radius != specs[0].Radius))
        {
            return null;
        }
        var neighbours = new Neighbours(shape);
        var corners = new Dictionary<Shape, Corner>(ShapeHasher.Instance);
        foreach (FilletSpec spec in specs)
        {
            if (Read(neighbours, spec.Edge) is not { } corner)
            {
                return null;
            }
            corners[spec.Edge] = corner;
        }

        // One radius, moved to the limit if a strip left would be too narrow for a tool to leave
        double r = specs[0].Radius;
        foreach ((Corner corner, Side side) in Sides(corners.Values))
        {
            if (side.Width is { } width)
            {
                double demand = corner.ContactPerRadius + (!side.Opposite.IsNull && corners.TryGetValue(side.Opposite, out Corner opposite) ? opposite.ContactPerRadius : 0);
                if (r * demand > width + side.Tolerance)
                {
                    return null;
                }
                if (r * demand >= width - TooNarrow * side.Tolerance)
                {
                    r = Math.Min(r, width / demand);
                }
            }
        }

        var balls = new Dictionary<Shape, BallCorner>(ShapeHasher.Instance);
        var pivots = new Dictionary<Shape, Pivot>(ShapeHasher.Instance);
        var mitres = new Dictionary<Shape, Mitre>(ShapeHasher.Instance);
        var prisms = new List<Prism>();
        foreach (Corner corner in corners.Values)
        {
            if (ReadEnd(neighbours, corners, corner, corner.Start, r, balls, pivots, mitres) is not { } first
                || ReadEnd(neighbours, corners, corner, corner.End, r, balls, pivots, mitres) is not { } last)
            {
                return null;
            }
            double on = r * corner.ContactPerRadius;
            prisms.Add(new Prism(new Cutter(corner, on, on, r), first, last));
        }

        double volume = prisms.Where(static p => !p.IsEmpty).Sum(static p => (p.Cut.Corner.IsConvex ? -1 : 1) * p.Volume)
                        - balls.Values.Sum(static b => b.Volume) - pivots.Values.Sum(static p => p.Volume) + mitres.Values.Sum(static m => m.Volume);
        double area = prisms.Sum(static p => p.AreaChange) // With no length left, there's still what a free end's face loses
                      + balls.Values.Sum(static b => b.AreaChange) + pivots.Values.Sum(static p => p.AreaChange) + mitres.Values.Sum(static m => m.AreaChange);

        // What each edge and corner made. An edge with no prism left made its balls
        var made = new List<(Shape Source, Func<Face, bool> IsNew)>();
        foreach (Prism prism in prisms)
        {
            Cutter cut = prism.Cut;
            if (!prism.IsEmpty)
            {
                made.Add((cut.Corner.Edge, f => cut.IsNewFace(f)));
            }
            else
            {
                BallCorner[] ends = [.. cut.Corner.Edge.AllVertices().Where(v => balls.ContainsKey(v)).Select(v => balls[v])];
                made.Add((cut.Corner.Edge, f => ends.Any(b => b.IsNewFace(f))));
            }
        }
        foreach (BallCorner ball in balls.Values)
        {
            made.Add((ball.Vertex, ball.IsNewFace));
        }
        foreach (Pivot pivot in pivots.Values)
        {
            made.Add((pivot.Vertex, f => pivot.IsNewFace(new FaceSurfaceAdaptor(f))));
        }

        // Nothing but one ball left: that's the answer, one face, with everything gone
        if (specs.Length == shape.AllEdges().Length && balls.Count > 0 && pivots.Count == 0 && mitres.Count == 0 && prisms.All(static p => p.IsEmpty)
            && balls.Values.All(b => b.Centre.DistanceTo(balls.Values.First().Centre) <= Tolerance))
        {
            History gone = new();
            foreach (Face face in shape.AllFaces())
            {
                gone.Remove(face);
            }
            foreach (Edge edge in shape.AllEdges())
            {
                gone.Remove(edge);
            }
            (Shape Source, Func<Face, bool> IsNew)[] all = [.. made.Select(static m => (m.Source, (Func<Face, bool>)(static _ => true)))];
            return Checked(shape, new MakeSphere(balls.Values.First().Centre, r).Shape().AllSolids()[0], gone, volume, area, all, ownSize: true);
        }

        // All that's cut, then all that's added, as they stand on what the other left
        Shape[] cuts = [.. prisms.Where(static p => !p.IsEmpty && p.Cut.Corner.IsConvex).Select(static p => p.Tool()), .. balls.Values.Select(static b => b.Tool()), .. pivots.Values.Select(static p => p.Tool())];
        Shape[] adds = [.. prisms.Where(static p => !p.IsEmpty && !p.Cut.Corner.IsConvex).Select(static p => p.Tool())];
        Shape result = shape;
        History history = new();
        if (cuts.Length > 0)
        {
            if (Boolean(new Kerf.Engine.Booleans.Api.Cut(), result, cuts, out History? cutting) is not { } after)
            {
                return null;
            }
            result = after;
            history.Merge(cutting);
        }
        if (adds.Length > 0)
        {
            if (Boolean(new Kerf.Engine.Booleans.Api.Fuse(), result, adds, out History? adding, simplify: true) is not { } after)
            {
                return null;
            }
            result = after;
            history.Merge(adding);
        }
        return result.AllSolids().Length != 1 ? null : Checked(shape, result.AllSolids()[0], history, volume, area, [.. made], ownSize: true);
    }

    // Reads one end of a selected edge. Null if it's a kind this doesn't build
    private static PrismEnd? ReadEnd(Neighbours neighbours, Dictionary<Shape, Corner> corners, in Corner corner, in Point3D end, double r,
                                     Dictionary<Shape, BallCorner> balls, Dictionary<Shape, Pivot> pivots, Dictionary<Shape, Mitre> mitres)
    {
        Direction3D along = corner.Along;
        double at = new Vector3D(corner.Start, end).Dot(new Vector3D(along));
        Point3D here = end;
        Vertex? found = null;
        foreach (Vertex v in corner.Edge.AllVertices())
        {
            if (Kerf.Engine.Topology.Brep.Tool.Pnt(v).DistanceTo(here) <= Math.Max(Tolerance, Kerf.Engine.Topology.Brep.Tool.Tolerance(v)))
            {
                found = v;
            }
        }
        if (found is not { } vertex)
        {
            return null;
        }
        IReadOnlyList<Edge> edges = neighbours.Edges(vertex);
        IReadOnlyList<Face> faces = neighbours.Faces(vertex);
        if (edges.Count != 3 || faces.Count != 3 || faces.Any(f => neighbours.Normal(f) is null))
        {
            return null;
        }
        Edge own = corner.Edge;
        Corner[] selected = [.. edges.Where(e => corners.ContainsKey(e)).Select(e => corners[e])];
        Face oneFace = corner.One.Face, twoFace = corner.Two.Face;
        switch (selected.Length)
        {
            case 1:
            {
                Direction3D normal = neighbours.Normal(faces.First(f => !f.IsSame(oneFace) && !f.IsSame(twoFace)))!.Value;
                if (Math.Abs(normal.Dot(along)) < 1e-6 || (!corner.IsConvex && Math.Abs(normal.Dot(along)) < 1 - 1e-9))
                {
                    return null;
                }
                return new PrismEnd(EndKind.Free, at, normal);
            }
            case 2:
            {
                Corner other = selected.First(c => !c.Edge.IsSame(own));
                if (!corner.IsConvex || !other.IsConvex)
                {
                    return null;
                }
                Side common = other.One.Face.IsSame(oneFace) || other.Two.Face.IsSame(oneFace) ? corner.One : corner.Two;
                Side beside = common.Face.IsSame(oneFace) ? corner.Two : corner.One;
                Side otherCommon = other.One.Face.IsSame(common.Face) ? other.One : other.Two;
                Side otherBeside = other.One.Face.IsSame(common.Face) ? other.Two : other.One;
                if (Math.Abs(corner.Angle - other.Angle) > 1e-9)
                {
                    return null;
                }

                // Which way the other edge runs from here: into this edge's strip, or out of it round a re-entrant corner
                Point3D otherFar = other.Start.DistanceTo(here) < other.End.DistanceTo(here) ? other.End : other.Start;
                Vector3D runs = new(here, otherFar);
                bool reEntrant = runs.Dot(new Vector3D(common.Inward)) < 0;
                if (reEntrant)
                {
                    if (Math.Abs(corner.Angle - Math.PI / 2) > 1e-9)
                    {
                        return null;
                    }
                    if (!pivots.ContainsKey(vertex))
                    {
                        // From the one edge's section to the other's, on the side the material is: the turn outside the corner
                        double turn = common.Inward.AngleTo(otherCommon.Inward);
                        Direction3D up = common.Normal;
                        Direction3D start = common.Inward;
                        Vector3D swung = new Vector3D(up).Crossed(new Vector3D(start));
                        if (swung.Dot(new Vector3D(otherCommon.Inward)) < 0)
                        {
                            start = otherCommon.Inward;
                        }
                        pivots[vertex] = new Pivot(vertex, here, up, start, turn, r);
                    }
                    return new PrismEnd(EndKind.Square, at, along);
                }
                if (!mitres.ContainsKey(vertex))
                {
                    double crossing = new Direction3D(runs).AngleTo(new Direction3D(new Vector3D(here, corner.Start.DistanceTo(here) < corner.End.DistanceTo(here) ? corner.End : corner.Start)));
                    mitres[vertex] = new Mitre(r, r * corner.ContactPerRadius, corner.Angle, crossing);
                }
                _ = beside;
                return new PrismEnd(EndKind.Mitre, at, otherBeside.Normal);
            }
            case 3:
            {
                if (selected.Any(static c => !c.IsConvex))
                {
                    return null;
                }
                Direction3D n0 = neighbours.Normal(faces[0])!.Value, n1 = neighbours.Normal(faces[1])!.Value, n2 = neighbours.Normal(faces[2])!.Value;
                Vector3D a = new(n0), b = new(n1), c = new(n2);
                double det = a.Dot(b.Crossed(c));
                if (Math.Abs(det) < 1e-6)
                {
                    return null;
                }

                // n . (V - C) = r on each face
                Vector3D fromCentre = (b.Crossed(c) + c.Crossed(a) + a.Crossed(b)) * (r / det);
                Point3D centre = here.Translated(fromCentre * -1);
                if (fromCentre.Dot(b.Crossed(c)) / det < 0 || fromCentre.Dot(c.Crossed(a)) / det < 0 || fromCentre.Dot(a.Crossed(b)) / det < 0)
                {
                    return null; // The corner isn't over the patch: the fillets would run past one another
                }
                if (!balls.ContainsKey(vertex))
                {
                    Point3D Foot(Face one, Face two)
                    {
                        Corner edge = selected.First(s => (s.One.Face.IsSame(one) || s.Two.Face.IsSame(one)) && (s.One.Face.IsSame(two) || s.Two.Face.IsSame(two)));
                        Direction3D d = edge.Along;
                        return edge.Start.Translated(new Vector3D(d) * new Vector3D(edge.Start, centre).Dot(new Vector3D(d)));
                    }
                    balls[vertex] = new BallCorner(vertex, here, centre, r, [n0, n1, n2], [Foot(faces[0], faces[1]), Foot(faces[1], faces[2]), Foot(faces[2], faces[0])]);
                }
                return new PrismEnd(EndKind.Square, new Vector3D(corner.Start, centre).Dot(new Vector3D(along)), along);
            }
            default:
                return null;
        }
    }
}
