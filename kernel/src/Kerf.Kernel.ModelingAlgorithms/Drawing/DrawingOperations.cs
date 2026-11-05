// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.HiddenLines;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Meshing;
using Kerf.Topology;
using Occt;

namespace Kerf.Drawing;

/// <summary>
/// Line drawings (docs/api/drawing.md): shapes projected by a <see cref="Camera"/> into visible and hidden 2D edges, by
/// OCCT's hidden-line removal, exact (<c>HLRBRep_Algo</c>) or polygonal (<c>HLRBRep_PolyAlgo</c>). Also a C# 14
/// extension member, so <c>DrawingOperations.ProjectLines(part, camera)</c> and <c>part.ProjectLines(camera)</c> both
/// work.
/// </summary>
public static class DrawingOperations
{
    /// <summary>The code of a projection that failed: <c>drawing.failed</c>.</summary>
    public static DiagnosticCode ProjectionFailed { get; } = new("drawing.failed");

    /// <summary>The code of a warning that the polygonal algorithm's mesh is missing faces:
    /// <c>drawing.faces-not-meshed</c>.</summary>
    public static DiagnosticCode FacesNotMeshed { get; } = new("drawing.faces-not-meshed");

    extension<TShape>(TShape shape) where TShape : struct, IShape<TShape>
    {
        /// <summary>
        /// The shape's line drawing for the camera: its edges and outlines projected, each part visible or hidden, as
        /// 2D curves with the sub-shapes they come from. The shape isn't changed.
        /// </summary>
        /// <returns>The drawing (empty for a shape with nothing to draw), or a failure (<see cref="ProjectionFailed"/>, or
        /// the engine's error).</returns>
        /// <exception cref="ArgumentException">The shape is null, the camera isn't valid (<see cref="Camera.ViewFrame"/>),
        /// or the polygonal algorithm is asked for with a perspective camera.</exception>
        /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
        public Result<LineDrawing> ProjectLines(Camera camera, DrawingOptions? options = null, CancellationToken cancellationToken = default)
            => ProjectLines([shape.AsShape()], camera, options, cancellationToken);
    }

    /// <summary>
    /// The line drawing of several shapes for the camera, each hiding the others' lines where it is in front of them.
    /// </summary>
    /// <returns>The drawing, or a failure (<see cref="ProjectionFailed"/>, or the engine's error).</returns>
    /// <exception cref="ArgumentNullException"><paramref name="shapes"/> or <paramref name="camera"/> is null.</exception>
    /// <exception cref="ArgumentException">A shape is null, the camera isn't valid, or the polygonal algorithm is asked
    /// for with a perspective camera.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result<LineDrawing> ProjectLines(IEnumerable<Shape> shapes, Camera camera, DrawingOptions? options = null,
                                                   CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(shapes);
        ArgumentNullException.ThrowIfNull(camera);
        List<Shape> inputs = [.. shapes];
        if (inputs.Any(s => s.IsNull))
        {
            throw new ArgumentException("A shape is null.", nameof(shapes));
        }

        options ??= DrawingOptions.Default;
        AlgorithmProjector projector = camera.ToProjector();
        if (options.Method == HiddenLineMethod.Polygonal && camera.Projection == Projection.Perspective)
        {
            throw new ArgumentException("The polygonal algorithm draws with orthographic cameras only.", nameof(options));
        }

        cancellationToken.ThrowIfCancellationRequested();
        double size = Size(inputs);
        try
        {
            if (options.Method == HiddenLineMethod.Exact)
            {
                return new LineDrawing(camera, Exact(inputs, projector, options, options.Tolerance ?? size / 2000, cancellationToken));
            }

            return Polygonal(inputs, camera, projector, options, size, cancellationToken);
        }
        catch (OperationCanceledException)
        {
            throw;
        }
        catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
        {
            cancellationToken.ThrowIfCancellationRequested();
            return Result<LineDrawing>.Failure(OperationDiagnostics.FromException(e));
        }
    }

    // The shapes' size: their bounding box's diagonal, or 1 for nothing to bound
    internal static double Size(List<Shape> shapes)
    {
        var box = new Box();
        foreach (Shape s in shapes)
        {
            Bounds.Add(s, ref box);
        }

        if (box.IsVoid)
        {
            return 1;
        }

        box.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
        double d = Math.Sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0) + (z1 - z0) * (z1 - z0));
        return d > 0 ? d : 1;
    }

    // OCCT's kinds as HLRBRep_HLRToShape numbers them, in the drawing's order
    private static readonly (LineKind Kind, int Type)[] Kinds =
        [(LineKind.Sharp, 5), (LineKind.Smooth, 3), (LineKind.Sewn, 4), (LineKind.Outline, 2), (LineKind.IsoLine, 1)];

    private static List<DrawnEdge> Exact(List<Shape> shapes, AlgorithmProjector projector, DrawingOptions options, double tolerance,
                                         CancellationToken cancellationToken)
    {
        var algo = new HiddenLineAlgo();
        foreach (Shape s in shapes)
        {
            algo.Add(s, options.IsoLineCount);
        }

        algo.Projector(projector);
        algo.Update();
        cancellationToken.ThrowIfCancellationRequested();
        algo.Hide();
        cancellationToken.ThrowIfCancellationRequested();
        var edges = new List<DrawnEdge>();
        HiddenLineData? ds = algo.DataStructure();
        if (ds is null)
        {
            return edges;
        }

        var collector = new ExactCollector(ds, new SubShapes(shapes, algo), tolerance, edges);
        ds.Projector().Scaled(true);
        try
        {
            foreach ((LineKind kind, int type) in Kinds)
            {
                collector.Collect(kind, type, visible: true);
                if (options.IncludeHidden)
                {
                    collector.Collect(kind, type, visible: false);
                }

                cancellationToken.ThrowIfCancellationRequested();
            }
        }
        finally
        {
            ds.Projector().Scaled(false);
        }

        return edges;
    }

    /// <summary>The caller's edges and faces, to tell a source the caller knows from one the outliner made, and the
    /// outliners' records of where theirs come from (a split edge from its edge, an outline or a rebuilt face from its
    /// face).</summary>
    private sealed class SubShapes
    {
        private readonly HashSet<Shape> _shapes = new(ShapeHasher.Instance);
        private readonly List<Engine.HiddenLines.Topology.TopologyData> _outliners = [];

        public SubShapes(List<Shape> shapes, HiddenLineAlgo? algo = null)
        {
            for (int i = 1; algo is not null && i <= algo.ShapeCount; i++)
            {
                if (algo.ShapeBounds(i).Shape() is { } outliner)
                {
                    _outliners.Add(outliner.DataStructure());
                }
            }

            foreach (Shape s in shapes)
            {
                var map = new IndexedSet<Shape>(ShapeHasher.Instance);
                Exploration.MapShapes(s, ShapeType.Edge, map);
                Exploration.MapShapes(s, ShapeType.Face, map);
                foreach (Shape sub in map)
                {
                    _shapes.Add(sub);
                }
            }
        }

        public Shape Known(in Shape s)
        {
            if (_shapes.TryGetValue(s, out Shape known))
            {
                return known;
            }

            foreach (Engine.HiddenLines.Topology.TopologyData data in _outliners)
            {
                Shape old = data.NewSOldS(s);
                if (!old.IsSame(s) && _shapes.TryGetValue(old, out known))
                {
                    return known;
                }
            }

            return default;
        }
    }

    /// <summary>HLRBRep_HLRToShape's selection of edges by kind, with each part's source and its curve as a public 2D
    /// curve rather than an edge.</summary>
    private sealed class ExactCollector(HiddenLineData ds, SubShapes known, double tolerance, List<DrawnEdge> edges)
    {
        public void Collect(LineKind kind, int type, bool visible)
        {
            NCollection_Array1<HiddenLineEdgeData> eData = ds.EDataArray();
            int edgeCount = ds.EdgeCount, faceCount = ds.FaceCount;
            for (int ie = 1; ie <= edgeCount; ie++)
            {
                HiddenLineEdgeData ed = eData[ie];
                if (ed.Selected() && !ed.Vertical())
                {
                    ed.Used(false);
                    ed.HideCount(0);
                }
                else
                {
                    ed.Used(true);
                }
            }

            for (int iface = 1; iface <= faceCount; iface++)
            {
                DrawFace(kind, type, visible, iface);
            }

            if (type >= 3)
            {
                for (int ie = 1; ie <= edgeCount; ie++)
                {
                    HiddenLineEdgeData ed = eData[ie];
                    if (!ed.Used())
                    {
                        DrawEdge(kind, type, visible, inFace: false, ie, 0, ed);
                        ed.Used(true);
                    }
                }
            }
        }

        private void DrawFace(LineKind kind, int type, bool visible, int iface)
        {
            var itf = new FaceIterator();
            for (itf.InitEdge(ds.FDataArray()[iface]); itf.MoreEdge(); itf.NextEdge())
            {
                int ie = itf.Edge();
                HiddenLineEdgeData edf = ds.EDataArray()[ie];
                if (edf.Used())
                {
                    continue;
                }

                bool todraw = type switch
                {
                    1 => itf.IsoLine(),
                    2 => itf.Internal(),
                    3 => edf.Rg1Line() && !edf.RgNLine() && !itf.OutLine(),
                    4 => edf.RgNLine() && !itf.OutLine(),
                    _ => !itf.IsoLine() && !itf.Internal() && (!edf.Rg1Line() || itf.OutLine()),
                };
                if (todraw)
                {
                    DrawEdge(kind, type, visible, inFace: true, ie, iface, edf);
                    edf.Used(true);
                }
                else if ((type > 4 || type == 2) && edf.Rg1Line() && !itf.OutLine())
                {
                    int hc = edf.HideCount();
                    if (hc > 0)
                    {
                        edf.Used(true);
                    }
                    else
                    {
                        edf.HideCount(hc + 1); // to try with another face
                    }
                }
                else
                {
                    edf.Used(true);
                }
            }
        }

        private void DrawEdge(LineKind kind, int type, bool visible, bool inFace, int ie, int iface, HiddenLineEdgeData ed)
        {
            bool todraw = inFace || type switch
            {
                3 => ed.Rg1Line() && !ed.RgNLine(),
                4 => ed.RgNLine(),
                _ => !ed.Rg1Line(),
            };
            if (!todraw)
            {
                return;
            }

            // An isoline or an internal outline is drawn on its face; the other lines are parts of edges
            Shape source = inFace && type <= 2 ? ds.FaceMap()[iface - 1] : ds.EdgeMap()[ie - 1];
            source = known.Known(source);
            var it = new EdgeIterator();
            if (visible)
            {
                for (it.InitVisible(ed.Status()); it.MoreVisible(); it.NextVisible())
                {
                    it.Visible(out double sta, out _, out double end, out _);
                    Add(ed.Geometry(), sta, end, kind, true, source);
                }
            }
            else
            {
                for (it.InitHidden(ed.Status()); it.MoreHidden(); it.NextHidden())
                {
                    it.Hidden(out double sta, out _, out double end, out _);
                    Add(ed.Geometry(), sta, end, kind, false, source);
                }
            }
        }

        private void Add(HiddenLineCurve ec, double u1, double u2, LineKind kind, bool visible, Shape source)
        {
            if (ProjectedCurves.Make(ec, u1, u2, tolerance) is { } curve)
            {
                edges.Add(new DrawnEdge(curve, kind, visible, source));
            }
        }
    }

    private static Result<LineDrawing> Polygonal(List<Shape> shapes, Camera camera, AlgorithmProjector projector, DrawingOptions options,
                                                 double size, CancellationToken cancellationToken)
    {
        MeshOptions meshOptions = options.Mesh ?? new MeshOptions { LinearDeflection = size / 1000, AngularDeflection = 0.2 };
        Shape input = shapes.Count == 1 ? shapes[0] : Compound.Create(shapes);
        var indicator = new ProgressIndicator(cancellationToken);
        List<DrawnEdge>? edges = IncrementalMesh.ReadMeshed(input, meshOptions.ToEngine(),
                                                             target => PolygonalOnMesh(input, target, projector, options, new SubShapes(shapes)),
                                                             indicator.Start(), out int status);
        cancellationToken.ThrowIfCancellationRequested();
        if (edges is null)
        {
            return Result<LineDrawing>.Failure(Diagnostic.Error(ProjectionFailed, "The shapes couldn't be meshed for the polygonal algorithm."));
        }

        var drawing = new LineDrawing(camera, edges);
        if ((status & (int)Engine.Meshing.Model.StatusModel.Failure) != 0)
        {
            return Result<LineDrawing>.Success(drawing, [Diagnostic.Warning(FacesNotMeshed, "Some faces couldn't be meshed: their lines may be missing.")]);
        }

        return drawing;
    }

    // HLRBRep_PolyHLRToShape's Update and selection, on the shapes meshed in the mesher's overlay; consecutive segments
    // of one source, kind and visibility are joined into polylines
    private static List<DrawnEdge> PolygonalOnMesh(in Shape input, in Shape target, AlgorithmProjector projector, DrawingOptions options,
                                                   SubShapes known)
    {
        var algo = new PolyAlgo(target);
        algo.Projector(projector);
        algo.Update();

        // The target is the input, or a copy of it of the same structure: its sources are the input's at the same index
        IndexedSet<Shape>? targetMap = null, inputMap = null;
        if (!ReferenceEquals(input.TShape(), target.TShape()))
        {
            targetMap = new IndexedSet<Shape>(ShapeHasher.Instance);
            inputMap = new IndexedSet<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(target, ShapeType.Edge, targetMap);
            Exploration.MapShapes(target, ShapeType.Face, targetMap);
            Exploration.MapShapes(input, ShapeType.Edge, inputMap);
            Exploration.MapShapes(input, ShapeType.Face, inputMap);
        }

        Transform3D t = algo.Projector().Transformation();
        var status = new EdgeStatus();
        var it = new EdgeIterator();
        var chains = new PolylineChains();
        for (algo.InitHide(); algo.MoreHide(); algo.NextHide())
        {
            ref AlgoBiPoint.PointsT points = ref algo.Hide(status, out Shape s, out bool reg1, out bool regn, out bool outl, out bool intl);
            Point3D a = new Point3D(points.Pnt1.X(), points.Pnt1.Y(), points.Pnt1.Z()).Transformed(t);
            Point3D b = new Point3D(points.Pnt2.X(), points.Pnt2.Y(), points.Pnt2.Z()).Transformed(t);
            double dx = b.X - a.X, dy = b.Y - a.Y;
            if (Math.Sqrt(dx * dx + dy * dy) <= 1.0e-10)
            {
                continue;
            }

            // HLRBRep_PolyHLRToShape's kinds, a segment taking the first that holds
            LineKind kind = intl ? LineKind.Outline
                : regn && !outl ? LineKind.Sewn
                : reg1 && !outl ? LineKind.Smooth
                : LineKind.Sharp;
            if (targetMap is not null && targetMap.IndexOf(s) is int index and >= 0)
            {
                s = inputMap![index];
            }

            Shape source = known.Known(s);
            for (it.InitVisible(status); it.MoreVisible(); it.NextVisible())
            {
                it.Visible(out double sta, out _, out double end, out _);
                chains.Add(new Point2D(a.X + sta * dx, a.Y + sta * dy), new Point2D(a.X + end * dx, a.Y + end * dy), kind, true, source);
            }

            if (options.IncludeHidden)
            {
                for (it.InitHidden(status); it.MoreHidden(); it.NextHidden())
                {
                    it.Hidden(out double sta, out _, out double end, out _);
                    chains.Add(new Point2D(a.X + sta * dx, a.Y + sta * dy), new Point2D(a.X + end * dx, a.Y + end * dy), kind, false, source);
                }
            }
        }

        return chains.Finish();
    }

    /// <summary>Segments joined into polylines: those of one source, kind and visibility, end to end.</summary>
    private sealed class PolylineChains
    {
        private readonly List<Group> _groups = [];
        private readonly Dictionary<(LineKind, bool), List<Group>> _byKind = [];

        private sealed class Group(LineKind kind, bool visible, Shape source)
        {
            public readonly LineKind Kind = kind;
            public readonly bool Visible = visible;
            public readonly Shape Source = source;
            public readonly List<(Point2D A, Point2D B)> Segments = [];
        }

        public void Add(Point2D p, Point2D q, LineKind kind, bool visible, Shape source)
        {
            if (p.SquareDistanceTo(q) <= 1e-20)
            {
                return;
            }

            if (!_byKind.TryGetValue((kind, visible), out List<Group>? groups))
            {
                _byKind[(kind, visible)] = groups = [];
            }

            Group? group = groups.Count > 0 && groups[^1].Source.IsSame(source) ? groups[^1] : groups.Find(g => g.Source.IsSame(source));
            if (group is null)
            {
                group = new Group(kind, visible, source);
                groups.Add(group);
                _groups.Add(group);
            }

            group.Segments.Add((p, q));
        }

        // In the exact algorithm's order: by kind, visible first, then as met
        public List<DrawnEdge> Finish()
        {
            var edges = new List<DrawnEdge>();
            foreach ((LineKind kind, _) in Kinds)
            {
                foreach (bool visible in (ReadOnlySpan<bool>)[true, false])
                {
                    foreach (Group group in _groups)
                    {
                        if (group.Kind == kind && group.Visible == visible)
                        {
                            foreach (List<Point2D> chain in Chains(group.Segments))
                            {
                                edges.Add(new DrawnEdge(ProjectedCurves.Polyline(chain), kind, visible, group.Source));
                            }
                        }
                    }
                }
            }

            return edges;
        }

        // Greedy chaining from each unused segment in turn, at its end and then at its start
        private static List<List<Point2D>> Chains(List<(Point2D A, Point2D B)> segments)
        {
            double size = 0;
            foreach ((Point2D a, Point2D b) in segments)
            {
                size = Math.Max(size, Math.Max(Math.Max(Math.Abs(a.X), Math.Abs(a.Y)), Math.Max(Math.Abs(b.X), Math.Abs(b.Y))));
            }

            double tolerance2 = Math.Pow(1e-9 * Math.Max(size, 1), 2);
            var used = new bool[segments.Count];
            var chains = new List<List<Point2D>>();
            for (int i = 0; i < segments.Count; i++)
            {
                if (used[i])
                {
                    continue;
                }

                used[i] = true;
                var chain = new List<Point2D> { segments[i].A, segments[i].B };
                while (Next(segments, used, chain[^1], tolerance2) is { } after)
                {
                    chain.Add(after);
                }

                while (Next(segments, used, chain[0], tolerance2) is { } before)
                {
                    chain.Insert(0, before);
                }

                chains.Add(chain);
            }

            return chains;
        }

        // The other end of an unused segment that has an end at p, marked used
        private static Point2D? Next(List<(Point2D A, Point2D B)> segments, bool[] used, Point2D p, double tolerance2)
        {
            for (int j = 0; j < segments.Count; j++)
            {
                if (used[j])
                {
                    continue;
                }

                if (segments[j].A.SquareDistanceTo(p) <= tolerance2)
                {
                    used[j] = true;
                    return segments[j].B;
                }

                if (segments[j].B.SquareDistanceTo(p) <= tolerance2)
                {
                    used[j] = true;
                    return segments[j].A;
                }
            }

            return null;
        }
    }
}
