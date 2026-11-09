// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Construction;

/// <summary>
/// Shapes from geometry (docs/api/construction.md; OCCT's <c>BRepBuilderAPI_MakeVertex</c>, <c>MakeEdge</c>,
/// <c>MakeWire</c>, <c>MakePolygon</c>, <c>MakeFace</c>, <c>MakeShell</c>, <c>MakeSolid</c> and <c>BRepLib_FindSurface</c>),
/// as static extension members of the shape types: <c>Edge.Line(p, q)</c>, <c>Wire.FromEdges(edges)</c>,
/// <c>Face.FromWire(wire)</c>. Unlike Topology's raw <c>Create</c>, they check and connect: a wire's edges must meet, a
/// face's wire must lie on its surface. A failure is the builder's status as a diagnostic (docs/api/diagnostics.md).
/// </summary>
public static class ShapeFactory
{
    extension(Vertex)
    {
        /// <summary>A vertex at <paramref name="point"/>.</summary>
        public static Vertex At(Point3D point) => new MakeVertex(point).Vertex();
    }

    extension(Edge)
    {
        /// <summary>The straight edge from <paramref name="start"/> to <paramref name="end"/>.</summary>
        public static Result<Edge> Line(Point3D start, Point3D end) => BuildEdge(() => new MakeEdge(start, end));

        /// <summary>An edge on <paramref name="curve"/>, over its whole range or <paramref name="range"/>.</summary>
        public static Result<Edge> FromCurve(Curve3D curve, Interval? range = null)
        {
            KernelArgumentNullException.ThrowIfNull(curve);
            return range is { } r ? BuildEdge(() => new MakeEdge(curve, r.Min, r.Max)) : BuildEdge(() => new MakeEdge(curve));
        }

        /// <summary>An edge on <paramref name="surface"/> along the 2D curve <paramref name="curve"/> of its parameters,
        /// with its 3D curve computed (OCCT's defaults), or, with <paramref name="computeCurve3D"/> false, without one, for
        /// <c>WithCurves3D</c> to compute with other settings.</summary>
        public static Result<Edge> FromCurve(Curve2D curve, Surface surface, Interval? range = null, bool computeCurve3D = true)
        {
            KernelArgumentNullException.ThrowIfNull(curve);
            KernelArgumentNullException.ThrowIfNull(surface);
            return BuildEdge(() => range is { } r ? new MakeEdge(curve, surface, r.Min, r.Max) : new MakeEdge(curve, surface),
                             buildCurve: computeCurve3D);
        }

        /// <summary>The closed edge of a whole circle.</summary>
        public static Result<Edge> Circle(in Circle3D circle)
        {
            Circle3D c = circle;
            return BuildEdge(() => new MakeEdge(c));
        }

        /// <summary>The edge of an arc of <paramref name="circle"/>, between two angles (radians).</summary>
        public static Result<Edge> Arc(in Circle3D circle, double startAngle, double endAngle)
        {
            Circle3D c = circle;
            return BuildEdge(() => new MakeEdge(c, startAngle, endAngle));
        }
    }

    extension(Wire)
    {
        /// <summary>A wire of <paramref name="edges"/>, connected in order (OCCT's <c>BRepBuilderAPI_MakeWire</c>: each edge
        /// must meet the wire so far; vertices within tolerance are merged).</summary>
        /// <exception cref="ArgumentException">An edge is null.</exception>
        public static Result<Wire> FromEdges(IEnumerable<Edge> edges)
        {
            KernelArgumentNullException.ThrowIfNull(edges);
            Edge[] list = [.. edges];
            foreach (Edge e in list)
            {
                MakerOutcome.NotNull(e, nameof(edges));
            }
            return Guarded(() =>
            {
                var wire = new MakeWire();
                foreach (Edge e in list)
                {
                    wire.Add(e);
                }
                return wire.IsDone ? wire.Wire() : Result<Wire>.Failure(StatusDiagnostics.ToDiagnostic(wire.Error()) ?? Failed("wire"));
            });
        }

        /// <summary>
        /// The wire the edges make in whatever order they are given, each connected where it meets the others (OCCT's
        /// <c>BRepBuilderAPI_MakeWire</c> given the list: edges meeting within their vertices' tolerances share a vertex).
        /// <c>construction.disconnected-wire</c> or <c>construction.non-manifold-wire</c> if they don't make one wire.
        /// </summary>
        /// <exception cref="ArgumentException">An edge is null.</exception>
        public static Result<Wire> Connect(IEnumerable<Edge> edges)
        {
            KernelArgumentNullException.ThrowIfNull(edges);
            var list = new Kerf.Engine.NCollection_List<Shape>();
            foreach (Edge e in edges)
            {
                MakerOutcome.NotNull(e, nameof(edges));
                list.Append(e);
            }
            return Guarded(() =>
            {
                var wire = new MakeWire();
                wire.Add(list);
                wire.Build();
                return wire.IsDone ? wire.Wire() : Result<Wire>.Failure(StatusDiagnostics.ToDiagnostic(wire.Error()) ?? Failed("wire"));
            });
        }

        /// <summary>The polygon through <paramref name="points"/>, closed back to the first if <paramref name="closed"/>.</summary>
        /// <exception cref="ArgumentException">Fewer than two points.</exception>
        public static Result<Wire> Polygon(ReadOnlySpan<Point3D> points, bool closed = false)
        {
            if (points.Length < 2)
            {
                throw new KernelArgumentException("A polygon needs at least two points.", nameof(points));
            }
            Point3D[] copy = points.ToArray();
            return Guarded(() =>
            {
                var polygon = new MakePolygon();
                foreach (Point3D p in copy)
                {
                    polygon.Add(p);
                }
                if (closed)
                {
                    polygon.Close();
                }
                return polygon.IsDone
                    ? polygon.Wire()
                    : Result<Wire>.Failure(Diagnostic.Error(new DiagnosticCode("construction.degenerate-polygon"), "The points don't make a polygon: they coincide."));
            });
        }
    }

    extension(Wire wire)
    {
        /// <summary>Whether the wire lies in a plane, and the plane if so (OCCT's <c>BRepLib_FindSurface</c>).</summary>
        public bool TryGetPlane(out Plane3D plane)
        {
            plane = default;
            if (wire.IsNull || !wire.AsShape().TryFindSurface(-1, out Surface? surface, onlyPlane: true) || surface is not PlaneSurface planar)
            {
                return false;
            }
            plane = planar.Plane;
            return true;
        }
    }

    /// <summary>Some edges' 3D curves couldn't be computed from their curves on faces: <c>construction.curves-3d-incomplete</c>.</summary>
    public static DiagnosticCode Curves3DIncomplete { get; } = new("construction.curves-3d-incomplete");

    extension<TShape>(TShape shape) where TShape : struct, IShape<TShape>
    {
        /// <summary>
        /// The shape with a 3D curve for each edge that has only curves on faces, approximated from them within
        /// <paramref name="tolerance"/> (OCCT's <c>BRepLib::BuildCurves3d</c>). An edge whose curve couldn't be computed is
        /// left without one, with a <c>construction.curves-3d-incomplete</c> warning. The shape isn't changed.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public Result<Built<Shape>> WithCurves3D(double tolerance = 1e-5, Continuity continuity = Continuity.C1, int maxDegree = 14,
                                                 int maxSegments = 0)
        {
            Shape self = shape.AsShape();
            MakerOutcome.NotNull(self, nameof(shape));
            bool done = true;
            Result<Built<Shape>> result = EngineOperation.Run<Shape>(_ =>
            {
                Shape built = BrepLibrary.Curves3dBuilt(self, tolerance, out done, out Engine.Topology.History history, continuity,
                                                        maxDegree, maxSegments);
                return new EngineOutcome(true, () => built, history);
            }, Diagnostic.Error(Primitives.Failed, "The edges' 3D curves couldn't be computed."));
            return result.TryGetValue(out Built<Shape>? value) && !done
                ? Result<Built<Shape>>.Success(value, [.. result.Diagnostics,
                    Diagnostic.Warning(Curves3DIncomplete, "Some edges' 3D curves couldn't be computed from their curves on faces.")])
                : result;
        }

        /// <summary>Whether the shape's edges lie on one surface within <paramref name="tolerance"/> (negative: their own
        /// tolerances), and the surface if so (OCCT's <c>BRepLib_FindSurface</c>), placed as the shape is.</summary>
        public bool TryFindSurface(double tolerance, [System.Diagnostics.CodeAnalysis.NotNullWhen(true)] out Surface? surface, bool onlyPlane = false)
        {
            surface = null;
            Shape self = shape.AsShape();
            if (self.IsNull)
            {
                return false;
            }
            try
            {
                var find = new FindSurface(self, tolerance, onlyPlane);
                if (!find.Found() || find.Surface() is not { } found)
                {
                    return false;
                }
                surface = find.Location().IsIdentity ? found : (Surface)found.Transformed(find.Location().Transformation);
                return true;
            }
            catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
            {
                return false;
            }
        }
    }

    extension(Face)
    {
        /// <summary>The face bounded by <paramref name="wire"/>, on the surface it lies on (a plane if it can be;
        /// only a plane if <paramref name="onlyPlane"/>).</summary>
        public static Result<Face> FromWire(Wire wire, bool onlyPlane = false, bool checkResult = true)
        {
            MakerOutcome.NotNull(wire, nameof(wire));
            return Validity.Checked(BuildFace(() => new MakeFace(wire, onlyPlane)), checkResult, "The face");
        }

        /// <summary>The face bounded by <paramref name="outer"/> with <paramref name="holes"/> cut from it. The holes must
        /// lie inside the outer wire and run the opposite way round, as OCCT requires. The face is on the surface the outer
        /// wire lies on (a plane if it can be; only a plane if <paramref name="onlyPlane"/>).</summary>
        public static Result<Face> FromWires(Wire outer, IEnumerable<Wire> holes, bool onlyPlane = false, bool checkResult = true)
        {
            MakerOutcome.NotNull(outer, nameof(outer));
            KernelArgumentNullException.ThrowIfNull(holes);
            Wire[] inner = [.. holes];
            foreach (Wire hole in inner)
            {
                MakerOutcome.NotNull(hole, nameof(holes));
            }
            return Validity.Checked(BuildFace(() =>
            {
                var face = new MakeFace(outer, onlyPlane);
                if (face.IsDone)
                {
                    foreach (Wire hole in inner)
                    {
                        face.Add(hole);
                    }
                }
                return face;
            }), checkResult, "The face");
        }

        /// <summary>A face on <paramref name="surface"/>: its natural bounds, <paramref name="bounds"/> of its parameters, or
        /// inside <paramref name="boundary"/>, a wire lying on it, with <paramref name="holes"/> (wires lying on it too) cut
        /// from it.</summary>
        public static Result<Face> FromSurface(Surface surface, (Interval U, Interval V)? bounds = null, Wire? boundary = null,
                                               IEnumerable<Wire>? holes = null, bool checkResult = true)
        {
            KernelArgumentNullException.ThrowIfNull(surface);
            if (bounds is not null && boundary is not null)
            {
                throw new KernelArgumentException("Give the bounds or a boundary, not both.", nameof(boundary));
            }
            Wire[] inner = [.. holes ?? []];
            if (inner.Length > 0 && boundary is null)
            {
                throw new KernelArgumentException("Holes need a boundary.", nameof(holes));
            }
            foreach (Wire hole in inner)
            {
                MakerOutcome.NotNull(hole, nameof(holes));
            }
            Result<Face> made = BuildFace(() => boundary is { } w
                ? WithHoles(new MakeFace(surface, w), inner)
                : bounds is { } b
                    ? new MakeFace(surface, b.U.Min, b.U.Max, b.V.Min, b.V.Max, Tolerances.Confusion)
                    : new MakeFace(surface, Tolerances.Confusion));
            // A surface's own bounds make a good face; bounds off the surface don't (mkface/after_extsurf_and_offset C6 to C9,
            // decision 1919), nor bounds so narrow that an edge is shorter than its tolerance (a rectangle 1e-9 wide,
            // decision 25802), and wires may not bound one
            return Validity.Checked(made, checkResult && (boundary is not null || (bounds is { } limits && (IsOff(surface, limits) || IsNarrow(limits)))),
                                    "The face");
        }

        // Bounds narrower than this, in the surface's parameters, are checked: from 2e-7 up a rectangle was valid as built
        private static bool IsNarrow((Interval U, Interval V) limits) =>
            limits.U.Max - limits.U.Min < 1e-5 || limits.V.Max - limits.V.Min < 1e-5;

        private static bool IsOff(Surface surface, (Interval U, Interval V) limits)
        {
            surface.ParameterBounds(out double uFirst, out double uLast, out double vFirst, out double vLast);
            double margin = Tolerances.Confusion;
            return (!surface.IsUPeriodic && (limits.U.Min < uFirst - margin || limits.U.Max > uLast + margin))
                   || (!surface.IsVPeriodic && (limits.V.Min < vFirst - margin || limits.V.Max > vLast + margin));
        }

        /// <summary>A face on <paramref name="plane"/>: infinite, or inside <paramref name="boundary"/>.</summary>
        public static Result<Face> FromPlane(in Plane3D plane, Wire? boundary = null)
        {
            Plane3D p = plane;
            return Validity.Checked(BuildFace(() => boundary is { } w ? new MakeFace(p, w) : new MakeFace(p)), boundary is not null, "The face");
        }
    }

    extension(Face face)
    {
        /// <summary>
        /// The face with <paramref name="holes"/> (wires lying in it, each running opposite its outer wire) cut from it: a
        /// copy of the face holding the holes as well (OCCT's <c>BRepBuilderAPI_MakeFace</c> of a face, then <c>Add</c>).
        /// The face isn't changed.
        /// </summary>
        /// <exception cref="ArgumentException">The face or a hole is null.</exception>
        public Result<Face> WithHoles(IEnumerable<Wire> holes, bool checkResult = true)
        {
            KernelArgumentNullException.ThrowIfNull(holes);
            Face f = face;
            MakerOutcome.NotNull(f, nameof(face));
            Wire[] inner = [.. holes];
            foreach (Wire hole in inner)
            {
                MakerOutcome.NotNull(hole, nameof(holes));
            }
            // BRepBuilderAPI_MakeFace of a face copies it without being done; each wire added makes it done
            return Validity.Checked(Guarded<Face>(() =>
            {
                var maker = new MakeFace(f);
                foreach (Wire hole in inner)
                {
                    maker.Add(hole);
                }

                return maker.Face();
            }), checkResult, "The face");
        }
    }

    extension(Shell)
    {
        /// <summary>A shell on <paramref name="surface"/>, over its natural bounds or <paramref name="bounds"/>.</summary>
        public static Result<Shell> FromSurface(Surface surface, (Interval U, Interval V)? bounds = null)
        {
            KernelArgumentNullException.ThrowIfNull(surface);
            return Guarded(() =>
            {
                MakeShell shell = bounds is { } b ? new MakeShell(surface, b.U.Min, b.U.Max, b.V.Min, b.V.Max) : new MakeShell(surface);
                return shell.IsDone ? shell.Shell() : Result<Shell>.Failure(StatusDiagnostics.ToDiagnostic(shell.Error()) ?? Failed("shell"));
            });
        }
    }

    extension(Shell)
    {
        /// <summary>
        /// The shell of planar polygons that share their corners (Kerf addition, decision 1995): one vertex for each
        /// point used, one straight edge for each pair of corners next to each other, used by every polygon that has
        /// it, so nothing is sewn. Each polygon lies on its plane, its corners in order anticlockwise about the plane's
        /// normal, which is the face's (taken as given: a polygon running the wrong way isn't turned round); corners within
        /// the edges' tolerance (1e-7) of the plane. The result is checked, so a polygon crossing itself fails.
        /// </summary>
        /// <param name="points">The corners, by index.</param>
        /// <param name="polygons">Each face's plane and its corners, as indices into <paramref name="points"/>.</param>
        /// <exception cref="ArgumentException">A polygon has fewer than three corners, or an index is out of range.</exception>
        public static Result<Shell> FromPolygons(IReadOnlyList<Point3D> points, IEnumerable<(Plane3D Plane, int[] Corners)> polygons)
        {
            KernelArgumentNullException.ThrowIfNull(points);
            KernelArgumentNullException.ThrowIfNull(polygons);
            var list = polygons.ToList();
            foreach (var (_, corners) in list)
            {
                if (corners is not { Length: >= 3 })
                {
                    throw new KernelArgumentException("A polygon needs at least three corners.", nameof(polygons));
                }
                if (corners.Any(i => i < 0 || i >= points.Count))
                {
                    throw new KernelArgumentException("A corner's index is out of range.", nameof(polygons));
                }
            }
            return Validity.Checked(Guarded<Shell>(() =>
            {
                var vertices = new Dictionary<int, Vertex>();
                Vertex VertexAt(int i) => vertices.TryGetValue(i, out Vertex v) ? v : vertices[i] = new MakeVertex(points[i]).Vertex();
                // Each edge once, from its lower index to its higher; a polygon running the other way uses it reversed
                var edges = new Dictionary<(int, int), Edge>();
                var faces = new List<Shape>(list.Count);
                foreach (var (plane, corners) in list)
                {
                    var wireEdges = new List<Shape>(corners.Length);
                    for (int k = 0; k < corners.Length; k++)
                    {
                        int a = corners[k], b = corners[(k + 1) % corners.Length];
                        var key = (Math.Min(a, b), Math.Max(a, b));
                        if (!edges.TryGetValue(key, out Edge edge))
                        {
                            var maker = new MakeEdge(VertexAt(key.Item1), VertexAt(key.Item2));
                            if (!maker.IsDone)
                            {
                                return Result<Shell>.Failure(StatusDiagnostics.ToDiagnostic(maker.Error()) ?? Failed("edge"));
                            }
                            edges[key] = edge = maker.Edge();
                        }
                        wireEdges.Add(a < b ? edge : edge.Reversed());
                    }
                    // The wire runs as given: OCCT's check of which side is inside can misjudge a thin polygon near its plane
                    var face = new MakeFace(plane, Builder.MakeWire(wireEdges), inside: false);
                    if (!face.IsDone)
                    {
                        return Result<Shell>.Failure(StatusDiagnostics.ToDiagnostic(face.Error()) ?? Failed("face"));
                    }
                    faces.Add(face.Face());
                }
                return Builder.MakeShell(faces);
            }), true, "The shell");
        }
    }

    extension(Solid)
    {
        /// <summary>The solid bounded by <paramref name="shells"/> (the first the outer, the others cavities).</summary>
        /// <exception cref="ArgumentException">No shells, or a null one.</exception>
        public static Result<Solid> FromShells(IEnumerable<Shell> shells, bool checkResult = true)
        {
            KernelArgumentNullException.ThrowIfNull(shells);
            Shell[] list = [.. shells];
            if (list.Length == 0)
            {
                throw new KernelArgumentException("A solid needs at least one shell.", nameof(shells));
            }
            foreach (Shell s in list)
            {
                MakerOutcome.NotNull(s, nameof(shells));
            }
            return Validity.Checked(Guarded(() =>
            {
                var solid = new MakeSolid();
                foreach (Shell s in list)
                {
                    solid.Add(s);
                }
                return solid.IsDone ? solid.Solid() : Result<Solid>.Failure(Failed("solid"));
            }), checkResult, "The solid");
        }
    }

    private static Result<Edge> BuildEdge(Func<MakeEdge> make, bool buildCurve = false) => Guarded(() =>
    {
        MakeEdge edge = make();
        if (!edge.IsDone)
        {
            return Result<Edge>.Failure(StatusDiagnostics.ToDiagnostic(edge.Error()) ?? Failed("edge"));
        }
        if (!buildCurve)
        {
            return edge.Edge();
        }

        // An edge on a surface gets its 3D curve before it is published
        Edge built = ShapeCasts.Edge(edge.InternalShape());
        BrepLibrary.BuildCurves3d(built);
        built.TShape()?.FreezeGraph();
        return built;
    });

    private static Result<Face> BuildFace(Func<MakeFace> make) => Guarded(() =>
    {
        MakeFace face = make();
        return face.IsDone ? face.Face() : Result<Face>.Failure(StatusDiagnostics.ToDiagnostic(face.Error()) ?? Failed("face"));
    });

    // A face maker given its holes (BRepBuilderAPI_MakeFace::Add), if it was made
    private static MakeFace WithHoles(MakeFace face, Wire[] holes)
    {
        if (face.IsDone)
        {
            foreach (Wire hole in holes)
            {
                face.Add(hole);
            }
        }
        return face;
    }

    private static Diagnostic Failed(string what) =>
        Diagnostic.Error(Primitives.Failed, $"The {what} couldn't be built.");

    // The engine's failures as a failed result
    private static Result<T> Guarded<T>(Func<Result<T>> run)
    {
        try
        {
            return run();
        }
        catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
        {
            return Result<T>.Failure(FailureDiagnostics.ToDiagnostic(e));
        }
    }
}
