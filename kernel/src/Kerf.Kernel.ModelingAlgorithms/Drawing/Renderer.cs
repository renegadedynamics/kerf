// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Buffers;
using System.Runtime.CompilerServices;
using Kerf.Geometry;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Drawing;

/// <summary>
/// The renderer of meshes (docs/api/drawing.md): a software rasteriser in managed code, with no native or GPU
/// dependency. Orthographic or perspective cameras, a depth buffer, flat or smooth shading with directional lights,
/// colours per mesh or per face, feature edges drawn over the faces with a depth test, supersampling, and a coloured or
/// transparent background. Also C# 14 extension members: <c>mesh.Render(camera)</c> and <c>part.Render(camera)</c>.
/// </summary>
/// <remarks>
/// The same scene, camera and options give the same pixels on every machine: the arithmetic is IEEE double precision
/// without library functions but square roots, and the bands of rows rasterised in parallel never share a pixel.
/// A triangle seen from behind is lit as if it faced the eye (open shells look right). Points at or behind a
/// perspective camera's eye are left out with their triangles.
/// <para>
/// A colour's opacity makes its faces see-through (decision 1998): each sample keeps every see-through face and edge in
/// front of the nearest opaque face, sorts them by depth and lays them over it from the back, so the picture doesn't
/// depend on the order of the items; a see-through item's edges take its opacity. A scene with nothing see-through is
/// drawn as before, pixel for pixel.
/// </para>
/// </remarks>
public static class Renderer
{
    /// <summary>The code of a picture that couldn't be made: <c>render.failed</c>.</summary>
    public static DiagnosticCode RenderFailed { get; } = new("render.failed");

    extension(ShapeMesh mesh)
    {
        /// <summary>The mesh rendered by the camera, in one colour (light grey by default).</summary>
        /// <returns>The picture, or a failure (<see cref="RenderFailed"/>).</returns>
        /// <exception cref="ArgumentNullException">The mesh or the camera is null.</exception>
        /// <exception cref="ArgumentException">The camera isn't valid (<see cref="Camera.ViewFrame"/>).</exception>
        /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
        public Result<RgbaImage> Render(Camera camera, RenderOptions? options = null, Color? color = null, CancellationToken cancellationToken = default)
            => Renderer.Render(new Scene(new SceneItem(mesh, color)), camera, options, cancellationToken);
    }

    extension<TShape>(TShape shape) where TShape : struct, IShape<TShape>
    {
        /// <summary>
        /// The shape meshed (with <paramref name="meshOptions"/>, or a deflection of a thousandth of its size) and
        /// rendered by the camera, in one colour (light grey by default).
        /// </summary>
        /// <returns>The picture, or the meshing's failure.</returns>
        /// <exception cref="ArgumentException">The shape is null, or the camera isn't valid.</exception>
        /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
        public Result<RgbaImage> Render(Camera camera, RenderOptions? options = null, Color? color = null, MeshOptions? meshOptions = null,
                                        CancellationToken cancellationToken = default)
        {
            KernelArgumentNullException.ThrowIfNull(camera);
            Shape s = shape.AsShape();
            if (s.IsNull)
            {
                throw new KernelArgumentException("The shape is null.", nameof(shape));
            }

            meshOptions ??= DefaultMeshOptions(s);
            return Mesher.Mesh(s, meshOptions, null, cancellationToken)
                         .Bind(mesh => Renderer.Render(new Scene(new SceneItem(mesh, color)), camera, options, cancellationToken));
        }
    }

    /// <summary>The mesh options for rendering a shape: a linear deflection of a thousandth of its size, an angle of
    /// 0.3 radians.</summary>
    /// <exception cref="ArgumentException">The shape is null.</exception>
    public static MeshOptions DefaultMeshOptions(Shape shape)
    {
        if (shape.IsNull)
        {
            throw new KernelArgumentException("The shape is null.", nameof(shape));
        }

        return new MeshOptions { LinearDeflection = DrawingOperations.Size([shape]) / 1000, AngularDeflection = 0.3 };
    }

    /// <summary>The scene rendered by the camera.</summary>
    /// <returns>The picture (the background alone for an empty scene), or a failure (<see cref="RenderFailed"/>).</returns>
    /// <exception cref="ArgumentNullException">The scene or the camera is null.</exception>
    /// <exception cref="ArgumentException">The camera isn't valid (<see cref="Camera.ViewFrame"/>).</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result<RgbaImage> Render(Scene scene, Camera camera, RenderOptions? options = null, CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(scene);
        KernelArgumentNullException.ThrowIfNull(camera);
        options ??= RenderOptions.Default;
        ViewTransform view = ViewTransform.Of(camera);
        cancellationToken.ThrowIfCancellationRequested();
        var rasteriser = new Rasteriser(view, options, cancellationToken);
        foreach (SceneItem item in scene.Items)
        {
            rasteriser.Add(item);
        }

        if (!rasteriser.Fit())
        {
            return Result<RgbaImage>.Failure(Diagnostic.Error(RenderFailed, "The scene's coordinates aren't finite."));
        }

        return rasteriser.Draw();
    }

    /// <summary>The rendering of one picture: the scene's nodes in view and screen coordinates, its triangles and
    /// edges' segments, rasterised by bands of rows into supersampled colour and depth buffers.</summary>
    private sealed class Rasteriser
    {
        private const int BandHeight = 16;

        private readonly ViewTransform _view;
        private readonly RenderOptions _options;
        private readonly CancellationToken _cancellationToken;
        private readonly int _width, _height, _samples;

        // Nodes: view coordinates, then screen coordinates and depth; unit normals in the view's frame
        private readonly List<double> _vx = [], _vy = [], _vz = [], _nx = [], _ny = [], _nz = [];
        private double[] _sx = [], _sy = [], _depth = [];
        private readonly List<bool> _valid = [];

        // Triangles: three nodes, a colour (with its opacity), whether it is seen from behind
        private readonly List<int> _triangles = [];
        private readonly List<(double R, double G, double B, double A)> _colors = [];
        private readonly List<bool> _back = [];

        // Edges' segments: two nodes each (edge points are nodes too, without normals), and the opacity and index of
        // the item they're from
        private readonly List<int> _segments = [];
        private readonly List<(double Alpha, int Item)> _segmentItems = [];
        private int _items;
        private bool _seeThrough;

        private readonly (double X, double Y, double Z, double Intensity)[] _lights;
        private double _depthBias;

        public Rasteriser(ViewTransform view, RenderOptions options, CancellationToken cancellationToken)
        {
            _view = view;
            _options = options;
            _cancellationToken = cancellationToken;
            _samples = options.Supersampling;
            _width = options.Width * _samples;
            _height = options.Height * _samples;
            _lights = [.. options.Lights.Where(l => l is not null && l.Direction.Magnitude > 0)
                                        .Select(l =>
                                        {
                                            double m = l.Direction.Magnitude;
                                            return (l.Direction.X / m, l.Direction.Y / m, l.Direction.Z / m, l.Intensity);
                                        })];
        }

        public void Add(SceneItem item)
        {
            _cancellationToken.ThrowIfCancellationRequested();
            ShapeMesh mesh = item.Mesh;
            Transform3D t = item.Transform;
            ReadOnlySpan<float> positions = mesh.Positions.Span, normals = mesh.Normals.Span;
            int first = _vx.Count;
            for (int i = 0; i < mesh.NodeCount; i++)
            {
                AddNode(positions[3 * i], positions[3 * i + 1], positions[3 * i + 2], t);
                Vector3D n = new Vector3D(normals[3 * i], normals[3 * i + 1], normals[3 * i + 2]).Transformed(t);
                _view.DirectionToView(n.X, n.Y, n.Z, out double x, out double y, out double z);
                double m = Math.Sqrt(x * x + y * y + z * z);
                if (m > 0)
                {
                    (x, y, z) = (x / m, y / m, z / m);
                }

                _nx.Add(x);
                _ny.Add(y);
                _nz.Add(z);
            }

            ReadOnlySpan<int> triangles = mesh.Triangles.Span;
            for (int f = 0; f < mesh.Faces.Count; f++)
            {
                FaceMesh face = mesh.Faces[f];
                Color color = item.FaceColors.TryGetValue(f, out Color c) ? c : item.Color;
                int colorIndex = _colors.Count;
                _colors.Add((color.R, color.G, color.B, color.A));
                int offset = first + face.FirstNode;
                for (int k = face.FirstTriangle; k < face.FirstTriangle + face.TriangleCount; k++)
                {
                    int a = offset + triangles[3 * k], b = offset + triangles[3 * k + 1], d = offset + triangles[3 * k + 2];
                    if (!_valid[a] || !_valid[b] || !_valid[d])
                    {
                        continue;
                    }

                    // Seen from behind: its normal (from its winding) facing away from the eye
                    double ux = _vx[b] - _vx[a], uy = _vy[b] - _vy[a], uz = _vz[b] - _vz[a];
                    double wx = _vx[d] - _vx[a], wy = _vy[d] - _vy[a], wz = _vz[d] - _vz[a];
                    double gx = uy * wz - uz * wy, gy = uz * wx - ux * wz, gz = ux * wy - uy * wx;
                    double ex = 0, ey = 0, ez = 1;
                    if (_view.IsPerspective)
                    {
                        ex = -_vx[a];
                        ey = -_vy[a];
                        ez = _view.Focus - _vz[a];
                    }

                    _triangles.Add(a);
                    _triangles.Add(b);
                    _triangles.Add(d);
                    _faceColor.Add(colorIndex);
                    _back.Add(gx * ex + gy * ey + gz * ez < 0);
                }
            }

            if (_options.Edges != EdgeDisplay.None)
            {
                AddEdges(mesh, t, item.Color.A, _items);
            }

            _items++;
        }

        private readonly List<int> _faceColor = [];

        private void AddNode(double x, double y, double z, Transform3D t)
        {
            Point3D p = new Point3D(x, y, z).Transformed(t);
            _view.ToView(p.X, p.Y, p.Z, out double vx, out double vy, out double vz);
            _vx.Add(vx);
            _vy.Add(vy);
            _vz.Add(vz);
            bool valid = double.IsFinite(vx) && double.IsFinite(vy) && double.IsFinite(vz) && (!_view.IsPerspective || vz < _view.Focus * (1 - 1e-6));
            _valid.Add(valid);
        }

        // The edges to draw, as segments between new nodes
        private void AddEdges(ShapeMesh mesh, Transform3D t, double alpha, int itemIndex)
        {
            Dictionary<(int, int, int), List<int>>? nodesAt = null;
            double cosFeature = Math.Cos(_options.FeatureAngle * Math.PI / 180);
            if (_options.Edges == EdgeDisplay.Feature)
            {
                nodesAt = NodesByPosition(mesh);
            }

            foreach (EdgePolyline edge in mesh.Edges)
            {
                ReadOnlySpan<float> points = edge.Points.Span;
                int count = edge.PointCount;
                if (count < 2 || (nodesAt is not null && !IsFeature(mesh, points, count, nodesAt, cosFeature)))
                {
                    continue;
                }

                int start = _vx.Count;
                for (int i = 0; i < count; i++)
                {
                    AddNode(points[3 * i], points[3 * i + 1], points[3 * i + 2], t);
                    _nx.Add(0);
                    _ny.Add(0);
                    _nz.Add(0);
                }

                for (int i = 0; i + 1 < count; i++)
                {
                    if (_valid[start + i] && _valid[start + i + 1])
                    {
                        _segments.Add(start + i);
                        _segments.Add(start + i + 1);
                        _segmentItems.Add((alpha, itemIndex));
                    }
                }
            }
        }

        private static (int, int, int) Key(ReadOnlySpan<float> xyz, int i)
        {
            static int Bits(float f) => f == 0 ? 0 : BitConverter.SingleToInt32Bits(f);
            return (Bits(xyz[3 * i]), Bits(xyz[3 * i + 1]), Bits(xyz[3 * i + 2]));
        }

        // The mesh's nodes by their exact position: where faces meet, and where a face meets itself (a seam)
        private static Dictionary<(int, int, int), List<int>> NodesByPosition(ShapeMesh mesh)
        {
            ReadOnlySpan<float> positions = mesh.Positions.Span;
            var nodes = new Dictionary<(int, int, int), List<int>>(mesh.NodeCount);
            for (int i = 0; i < mesh.NodeCount; i++)
            {
                (int, int, int) key = Key(positions, i);
                if (!nodes.TryGetValue(key, out List<int>? list))
                {
                    nodes[key] = list = new List<int>(2);
                }

                list.Add(i);
            }

            return nodes;
        }

        // An edge is drawn if it is free (one face's node at each point), unknown, or if the normals of the faces it
        // bounds differ somewhere along it by more than the feature angle
        private static bool IsFeature(ShapeMesh mesh, ReadOnlySpan<float> points, int count, Dictionary<(int, int, int), List<int>> nodesAt,
                                      double cosFeature)
        {
            // The faces having a node at every point of the edge
            HashSet<int>? faces = null;
            for (int i = 0; i < count; i++)
            {
                if (!nodesAt.TryGetValue(Key(points, i), out List<int>? nodes))
                {
                    return true;
                }

                var here = new HashSet<int>(nodes.Select(n => FaceOf(mesh, n)));
                if (faces is null)
                {
                    faces = here;
                }
                else
                {
                    faces.IntersectWith(here);
                }
            }

            if (faces is null || faces.Count == 0)
            {
                return true;
            }

            ReadOnlySpan<float> normals = mesh.Normals.Span;
            bool shared = false;
            for (int i = 0; i < count; i++)
            {
                List<int> nodes = nodesAt[Key(points, i)];
                for (int a = 0; a < nodes.Count; a++)
                {
                    if (!faces.Contains(FaceOf(mesh, nodes[a])))
                    {
                        continue;
                    }

                    for (int b = a + 1; b < nodes.Count; b++)
                    {
                        if (!faces.Contains(FaceOf(mesh, nodes[b])))
                        {
                            continue;
                        }

                        shared = true;
                        int na = 3 * nodes[a], nb = 3 * nodes[b];
                        double dot = (double)normals[na] * normals[nb] + (double)normals[na + 1] * normals[nb + 1] + (double)normals[na + 2] * normals[nb + 2];
                        if (dot < cosFeature)
                        {
                            return true;
                        }
                    }
                }
            }

            return !shared;
        }

        private static int FaceOf(ShapeMesh mesh, int node)
        {
            IReadOnlyList<FaceMesh> faces = mesh.Faces;
            int lo = 0, hi = faces.Count - 1;
            while (lo < hi)
            {
                int mid = (lo + hi + 1) / 2;
                if (faces[mid].FirstNode <= node)
                {
                    lo = mid;
                }
                else
                {
                    hi = mid - 1;
                }
            }

            // Faces without nodes share their first node with the next: the last face starting there owns it
            return lo;
        }

        // The screen coordinates: the projection scaled to fit (or at the given scale), centred
        public bool Fit()
        {
            int n = _vx.Count;
            var px = new double[n];
            var py = new double[n];
            double xMin = double.PositiveInfinity, xMax = double.NegativeInfinity, yMin = double.PositiveInfinity, yMax = double.NegativeInfinity;
            double dMin = double.PositiveInfinity, dMax = double.NegativeInfinity;
            _depth = new double[n];
            for (int i = 0; i < n; i++)
            {
                if (!_valid[i])
                {
                    continue;
                }

                _view.Projected(_vx[i], _vy[i], _vz[i], out px[i], out py[i]);
                xMin = Math.Min(xMin, px[i]);
                xMax = Math.Max(xMax, px[i]);
                yMin = Math.Min(yMin, py[i]);
                yMax = Math.Max(yMax, py[i]);
                _depth[i] = _view.IsPerspective ? 1 / (_view.Focus - _vz[i]) : _vz[i];
                dMin = Math.Min(dMin, _depth[i]);
                dMax = Math.Max(dMax, _depth[i]);
            }

            double scale, cx = 0, cy = 0;
            if (xMin > xMax)
            {
                scale = _options.Scale ?? 1;
                dMin = dMax = 0;
            }
            else
            {
                if (!double.IsFinite(xMax - xMin) || !double.IsFinite(yMax - yMin))
                {
                    return false;
                }

                if (_options.Scale is { } s)
                {
                    scale = s;
                }
                else
                {
                    cx = 0.5 * (xMin + xMax);
                    cy = 0.5 * (yMin + yMax);
                    double usable = 1 - 2 * _options.Margin;
                    double sx = xMax > xMin ? _options.Width * usable / (xMax - xMin) : double.PositiveInfinity;
                    double sy = yMax > yMin ? _options.Height * usable / (yMax - yMin) : double.PositiveInfinity;
                    scale = Math.Min(sx, sy);
                    if (double.IsPositiveInfinity(scale))
                    {
                        scale = 1;
                    }
                }
            }

            scale *= _samples;
            _sx = new double[n];
            _sy = new double[n];
            for (int i = 0; i < n; i++)
            {
                _sx[i] = 0.5 * _width + (px[i] - cx) * scale;
                _sy[i] = 0.5 * _height - (py[i] - cy) * scale;
            }

            _depthBias = 2e-3 * (dMax - dMin) + 1e-9 * Math.Max(Math.Abs(dMax), Math.Abs(dMin));
            return true;
        }

        public RgbaImage Draw()
        {
            int pixels = _width * _height;
            var color = new uint[pixels];
            var depth = new float[pixels];
            Array.Fill(depth, float.NegativeInfinity);
            Color background = _options.Background;
            uint backgroundPacked = Pack(Byte(background.R), Byte(background.G), Byte(background.B), Byte(background.A));
            Array.Fill(color, backgroundPacked);
            Color edge = _options.EdgeColor;
            uint edgePacked = Pack(Byte(edge.R), Byte(edge.G), Byte(edge.B), 255);

            int bands = (_height + BandHeight - 1) / BandHeight;
            var triangleBins = new List<int>[bands];
            var glassBins = new List<int>[bands];
            var segmentBins = new List<int>[bands];
            for (int b = 0; b < bands; b++)
            {
                triangleBins[b] = [];
                glassBins[b] = [];
                segmentBins[b] = [];
            }

            _seeThrough = _segmentItems.Any(s => s.Alpha < 1);
            for (int t = 0; t < _triangles.Count / 3; t++)
            {
                int a = _triangles[3 * t], b = _triangles[3 * t + 1], c = _triangles[3 * t + 2];
                bool glass = _colors[_faceColor[t]].A < 1;
                _seeThrough |= glass;
                Bin(glass ? glassBins : triangleBins, Math.Min(_sy[a], Math.Min(_sy[b], _sy[c])), Math.Max(_sy[a], Math.Max(_sy[b], _sy[c])), t);
            }

            double radius = Math.Max(0.5, 0.5 * _options.EdgeWidth * _samples);
            for (int s = 0; s < _segments.Count / 2; s++)
            {
                int a = _segments[2 * s], b = _segments[2 * s + 1];
                Bin(segmentBins, Math.Min(_sy[a], _sy[b]) - radius, Math.Max(_sy[a], _sy[b]) + radius, s);
            }

            void Band(int band)
            {
                _cancellationToken.ThrowIfCancellationRequested();
                int y0 = band * BandHeight, y1 = Math.Min(_height, y0 + BandHeight);
                foreach (int t in triangleBins[band])
                {
                    Triangle(t, y0, y1, color, depth, null);
                }

                if (!_seeThrough)
                {
                    foreach (int s in segmentBins[band])
                    {
                        Segment(s, y0, y1, radius, edgePacked, color, depth, null);
                    }

                    return;
                }

                // In front of the opaque faces, every see-through face and every edge, sorted at each sample
                using var fragments = new Fragments(y0 * _width, (y1 - y0) * _width);
                foreach (int s in segmentBins[band])
                {
                    Segment(s, y0, y1, radius, edgePacked, color, depth, fragments);
                }

                foreach (int t in glassBins[band])
                {
                    Triangle(t, y0, y1, color, depth, fragments);
                }

                fragments.Resolve(color);
            }

            if (_options.Parallel)
            {
                Parallel.For(0, bands, new ParallelOptions { CancellationToken = _cancellationToken }, Band);
            }
            else
            {
                for (int b = 0; b < bands; b++)
                {
                    Band(b);
                }
            }

            _cancellationToken.ThrowIfCancellationRequested();
            return new RgbaImage(_options.Width, _options.Height, Downsample(color));
        }

        private void Bin(List<int>[] bins, double yMin, double yMax, int index)
        {
            if (!(yMax >= 0) || !(yMin < _height))
            {
                return;
            }

            int first = Math.Max(0, (int)Math.Floor(yMin) / BandHeight), last = Math.Min(bins.Length - 1, (int)Math.Floor(Math.Min(yMax, _height - 1)) / BandHeight);
            for (int b = first; b <= last; b++)
            {
                bins[b].Add(index);
            }
        }

        // A see-through triangle (or, with fragments null, an opaque one)
        private void Triangle(int t, int y0, int y1, uint[] color, float[] depth, Fragments? fragments)
        {
            int ia = _triangles[3 * t], ib = _triangles[3 * t + 1], ic = _triangles[3 * t + 2];
            double ax = _sx[ia], ay = _sy[ia], bx = _sx[ib], by = _sy[ib], cx = _sx[ic], cy = _sy[ic];
            double area = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
            if (!(Math.Abs(area) > 1e-12))
            {
                return;
            }

            int xMin = Math.Max(0, (int)Math.Floor(Math.Min(ax, Math.Min(bx, cx)))), xMax = Math.Min(_width - 1, (int)Math.Ceiling(Math.Max(ax, Math.Max(bx, cx))));
            int yMin = Math.Max(y0, (int)Math.Floor(Math.Min(ay, Math.Min(by, cy)))), yMax = Math.Min(y1 - 1, (int)Math.Ceiling(Math.Max(ay, Math.Max(by, cy))));
            double da = _depth[ia], db = _depth[ib], dc = _depth[ic];
            (double r, double g, double b, double alpha) = _colors[_faceColor[t]];
            bool back = _back[t], smooth = _options.Shading == Shading.Smooth;
            uint flat = 0;
            if (!smooth)
            {
                // The triangle's own normal, in the view's frame, towards the eye
                double ux = _vx[ib] - _vx[ia], uy = _vy[ib] - _vy[ia], uz = _vz[ib] - _vz[ia];
                double wx = _vx[ic] - _vx[ia], wy = _vy[ic] - _vy[ia], wz = _vz[ic] - _vz[ia];
                double gx = uy * wz - uz * wy, gy = uz * wx - ux * wz, gz = ux * wy - uy * wx;
                double m = Math.Sqrt(gx * gx + gy * gy + gz * gz);
                double sign = back ? -1 : 1;
                flat = m > 0 ? Shade(sign * gx / m, sign * gy / m, sign * gz / m, r, g, b) : Shade(0, 0, 1, r, g, b);
            }

            double inverse = 1 / area;
            // A see-through triangle covers a sample on its border only from one side, or the two triangles meeting there
            // would both be laid over it: the edge function's direction, as the triangle winds anticlockwise, decides
            double sense = area > 0 ? 1 : -1;
            bool own0 = Owns(sense * (cx - bx), sense * (cy - by)), own1 = Owns(sense * (ax - cx), sense * (ay - cy)), own2 = Owns(sense * (bx - ax), sense * (by - ay));
            // A sliver's weights are mostly rounding, so which pixels pass is only known by asking each one
            bool useSpans = Math.Abs(area) >= 1;
            for (int y = yMin; y <= yMax; y++)
            {
                double py = y + 0.5;
                int row = y * _width;
                int xFrom = xMin, xTo = xMax;
                if (useSpans)
                {
                    // Only the pixels the triangle crosses on this row, and one either side so that rounding can't lose any.
                    // A long thin triangle's box is nearly all misses
                    double left = double.PositiveInfinity, right = double.NegativeInfinity;
                    Crossing(ax, ay, bx, by, py, ref left, ref right);
                    Crossing(bx, by, cx, cy, py, ref left, ref right);
                    Crossing(cx, cy, ax, ay, py, ref left, ref right);
                    if (left > right)
                    {
                        continue;
                    }

                    xFrom = Math.Max(xMin, (int)Math.Floor(left) - 1);
                    xTo = Math.Min(xMax, (int)Math.Ceiling(right) + 1);
                }

                for (int x = xFrom; x <= xTo; x++)
                {
                    double px = x + 0.5;
                    double w0 = ((bx - px) * (cy - py) - (by - py) * (cx - px)) * inverse;
                    double w1 = ((cx - px) * (ay - py) - (cy - py) * (ax - px)) * inverse;
                    double w2 = ((ax - px) * (by - py) - (ay - py) * (bx - px)) * inverse;
                    if (w0 < 0 || w1 < 0 || w2 < 0)
                    {
                        continue;
                    }

                    if (fragments is not null && ((w0 == 0 && !own0) || (w1 == 0 && !own1) || (w2 == 0 && !own2)))
                    {
                        continue;
                    }

                    float d = (float)(w0 * da + w1 * db + w2 * dc);
                    int i = row + x;
                    if (!(d > depth[i]))
                    {
                        continue;
                    }

                    if (fragments is null)
                    {
                        depth[i] = d;
                    }
                    if (smooth)
                    {
                        double nx = w0 * _nx[ia] + w1 * _nx[ib] + w2 * _nx[ic];
                        double ny = w0 * _ny[ia] + w1 * _ny[ib] + w2 * _ny[ic];
                        double nz = w0 * _nz[ia] + w1 * _nz[ib] + w2 * _nz[ic];
                        double m = Math.Sqrt(nx * nx + ny * ny + nz * nz);
                        uint shaded;
                        if (m > 0)
                        {
                            m = back ? -1 / m : 1 / m;
                            shaded = Shade(nx * m, ny * m, nz * m, r, g, b);
                        }
                        else
                        {
                            shaded = Shade(0, 0, 1, r, g, b);
                        }

                        if (fragments is null)
                        {
                            color[i] = shaded;
                        }
                        else
                        {
                            fragments.Add(i, d, shaded, alpha, -1);
                        }
                    }
                    else if (fragments is null)
                    {
                        color[i] = flat;
                    }
                    else
                    {
                        fragments.Add(i, d, flat, alpha, -1);
                    }
                }
            }
        }

        // The top-left rule: of two triangles sharing an edge, exactly one runs it with this direction
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool Owns(double dx, double dy) => dy > 0 || (dy == 0 && dx < 0);

        // Where an edge crosses a row, widening what's known of the row's span
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static void Crossing(double x0, double y0, double x1, double y1, double y, ref double left, ref double right)
        {
            const double margin = 1e-6;
            if (y < Math.Min(y0, y1) - margin || y > Math.Max(y0, y1) + margin)
            {
                return;
            }

            double dy = y1 - y0;
            double t = Math.Abs(dy) > margin ? Math.Clamp((y - y0) / dy, 0, 1) : 0;
            double xa = Math.Abs(dy) > margin ? x0 + t * (x1 - x0) : Math.Min(x0, x1);
            double xb = Math.Abs(dy) > margin ? xa : Math.Max(x0, x1);
            left = Math.Min(left, xa);
            right = Math.Max(right, xb);
        }

        // An edge's segment, drawn over the faces (or, in a scene with something see-through, kept to sort with them)
        private void Segment(int s, int y0, int y1, double radius, uint packed, uint[] color, float[] depth, Fragments? fragments)
        {
            (double alpha, int item) = _segmentItems[s];
            int ia = _segments[2 * s], ib = _segments[2 * s + 1];
            double ax = _sx[ia], ay = _sy[ia], bx = _sx[ib], by = _sy[ib], da = _depth[ia], db = _depth[ib];
            double dx = bx - ax, dy = by - ay, l2 = dx * dx + dy * dy, r2 = radius * radius;
            int xMin = Math.Max(0, (int)Math.Floor(Math.Min(ax, bx) - radius)), xMax = Math.Min(_width - 1, (int)Math.Ceiling(Math.Max(ax, bx) + radius));
            int yMin = Math.Max(y0, (int)Math.Floor(Math.Min(ay, by) - radius)), yMax = Math.Min(y1 - 1, (int)Math.Ceiling(Math.Max(ay, by) + radius));
            for (int y = yMin; y <= yMax; y++)
            {
                double py = y + 0.5;
                int row = y * _width;
                for (int x = xMin; x <= xMax; x++)
                {
                    double px = x + 0.5;
                    double t = l2 > 0 ? Math.Clamp(((px - ax) * dx + (py - ay) * dy) / l2, 0, 1) : 0;
                    double ex = ax + t * dx - px, ey = ay + t * dy - py;
                    if (ex * ex + ey * ey > r2)
                    {
                        continue;
                    }

                    int i = row + x;
                    float d = (float)(da + t * (db - da) + _depthBias);
                    if (d >= depth[i])
                    {
                        if (fragments is null)
                        {
                            color[i] = packed;
                        }
                        else
                        {
                            fragments.Add(i, d, packed, alpha, item);
                        }
                    }
                }
            }
        }

        // Ambient light plus each light's diffuse light on the unit normal (towards the eye), times the colour
        private uint Shade(double nx, double ny, double nz, double r, double g, double b)
        {
            double light = _options.Ambient;
            foreach ((double x, double y, double z, double intensity) in _lights)
            {
                double dot = nx * x + ny * y + nz * z;
                if (dot > 0)
                {
                    light += intensity * dot;
                }
            }

            return Pack(Byte(r * light), Byte(g * light), Byte(b * light), 255);
        }

        private static byte Byte(double c) => c >= 1 ? (byte)255 : c <= 0 ? (byte)0 : (byte)(int)(c * 255 + 0.5);

        private static uint Pack(byte r, byte g, byte b, byte a) => r | ((uint)g << 8) | ((uint)b << 16) | ((uint)a << 24);

        /// <summary>
        /// One band's see-through faces and edges, by sample: a list each, sorted by depth when the band is done and laid
        /// over the opaque picture from the back. An item's edges give a sample one fragment, the nearest, so the joins
        /// of its segments aren't laid twice. A band's lists are its own, so bands in parallel share nothing.
        /// </summary>
        private sealed class Fragments(int first, int count) : IDisposable
        {
            private struct Fragment
            {
                public float Depth;
                public uint Color;
                public float Alpha;
                public int Item;
                public int Next;
            }

            // From the shared pool, given back when the band is done: a band's buffers are its own while it has them
            private readonly int[] _head = Heads(count);
            private Fragment[] _all = ArrayPool<Fragment>.Shared.Rent(1024);
            private int _count;

            private static int[] Heads(int count)
            {
                int[] heads = ArrayPool<int>.Shared.Rent(count);
                Array.Fill(heads, -1, 0, count);
                return heads;
            }

            public void Add(int sample, float depth, uint color, double alpha, int item)
            {
                int slot = sample - first;
                if (item >= 0)
                {
                    for (int k = _head[slot]; k >= 0; k = _all[k].Next)
                    {
                        if (_all[k].Item == item)
                        {
                            if (depth > _all[k].Depth)
                            {
                                (_all[k].Depth, _all[k].Color, _all[k].Alpha) = (depth, color, (float)alpha);
                            }

                            return;
                        }
                    }
                }

                if (_count == _all.Length)
                {
                    Fragment[] larger = ArrayPool<Fragment>.Shared.Rent(2 * _all.Length);
                    Array.Copy(_all, larger, _count);
                    ArrayPool<Fragment>.Shared.Return(_all);
                    _all = larger;
                }

                _all[_count] = new Fragment { Depth = depth, Color = color, Alpha = (float)alpha, Item = item, Next = _head[slot] };
                _head[slot] = _count++;
            }

            // Each sample's fragments from the farthest (lowest depth) to the nearest, ties in the order they were
            // made, each laid over what's behind it
            public void Resolve(uint[] color)
            {
                var here = new List<int>();
                for (int slot = 0; slot < count; slot++)
                {
                    if (_head[slot] < 0)
                    {
                        continue;
                    }

                    here.Clear();
                    for (int k = _head[slot]; k >= 0; k = _all[k].Next)
                    {
                        here.Add(k);
                    }

                    here.Sort((p, q) => _all[p].Depth != _all[q].Depth ? _all[p].Depth.CompareTo(_all[q].Depth) : p.CompareTo(q));
                    int i = first + slot;
                    uint c = color[i];
                    double a = (c >> 24) / 255.0;
                    double r = (c & 0xFF) / 255.0 * a, g = ((c >> 8) & 0xFF) / 255.0 * a, b = ((c >> 16) & 0xFF) / 255.0 * a;
                    foreach (int k in here)
                    {
                        uint f = _all[k].Color;
                        double alpha = _all[k].Alpha;
                        r = alpha * (f & 0xFF) / 255.0 + (1 - alpha) * r;
                        g = alpha * ((f >> 8) & 0xFF) / 255.0 + (1 - alpha) * g;
                        b = alpha * ((f >> 16) & 0xFF) / 255.0 + (1 - alpha) * b;
                        a = alpha + (1 - alpha) * a;
                    }

                    color[i] = a > 0 ? Pack(Byte(r / a), Byte(g / a), Byte(b / a), Byte(a)) : 0;
                }
            }

            public void Dispose()
            {
                ArrayPool<int>.Shared.Return(_head);
                ArrayPool<Fragment>.Shared.Return(_all);
            }
        }

        // The supersampled buffer averaged to the picture's pixels, colours weighted by their opacity
        private byte[] Downsample(uint[] color)
        {
            int width = _options.Width, height = _options.Height, n = _samples * _samples;
            var pixels = new byte[4 * width * height];
            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    long r = 0, g = 0, b = 0, a = 0;
                    for (int j = 0; j < _samples; j++)
                    {
                        int row = (y * _samples + j) * _width + x * _samples;
                        for (int i = 0; i < _samples; i++)
                        {
                            uint c = color[row + i];
                            long alpha = c >> 24;
                            r += (c & 0xFF) * alpha;
                            g += ((c >> 8) & 0xFF) * alpha;
                            b += ((c >> 16) & 0xFF) * alpha;
                            a += alpha;
                        }
                    }

                    int p = 4 * (y * width + x);
                    if (a > 0)
                    {
                        pixels[p] = (byte)((r + a / 2) / a);
                        pixels[p + 1] = (byte)((g + a / 2) / a);
                        pixels[p + 2] = (byte)((b + a / 2) / a);
                    }

                    pixels[p + 3] = (byte)((a + n / 2) / n);
                }
            }

            return pixels;
        }
    }
}
