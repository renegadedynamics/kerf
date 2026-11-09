using System.Collections.Immutable;
using Kerf.Construction;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Meshing;

namespace Kerf.Modeling;

public static partial class ShapeExtensions
{
    // Any mesh on the shape is ignored, as it can affect the box, unless the shape has nothing else (faces holding only
    // triangles, as STL and glTF are read). The tolerance is accepted, as build123d accepts it, but unused.
    private static BoundingBox BoxOf(Shape shape, bool optimal)
    {
        if (shape.IsEmpty)
            return BoundingBox.Empty;
        var box = shape.BoundingBox(optimal);
        return box.IsEmpty ? shape.BoundingBox(optimal: false, useTriangulation: true) : box;
    }

    extension<TSelf>(TSelf self) where TSelf : IShape
    {
        // ---- Kind ----

        /// <summary>The type of the underlying curve (of an edge) or surface (of a face); <see cref="GeomType.Other"/> for other shapes.</summary>
        /// <exception cref="InvalidOperationException">The shape is empty.</exception>
        public GeomType GeomType
        {
            get
            {
                if (self.IsEmpty)
                    throw new InvalidOperationException("Cannot determine geometry type of an empty shape");
                switch (self.AsShape().Type)
                {
                    case ShapeType.Edge:
                        return ShapeExtensions.GeomTypeOf(((Edge)self.AsShape()).ToCurve().Kind);
                    case ShapeType.Face:
                        return ShapeExtensions.GeomTypeOf(((Face)self.AsShape()).ToSurface().Kind);
                    default:
                        return GeomType.Other;
                }
            }
        }

        // ---- Placement ----

        /// <summary>The shape's Location.</summary>
        /// <exception cref="InvalidOperationException">The shape is empty.</exception>
        public Location Location
        {
            get
            {
                self.NeedsGeometry("location");
                return new Location(self.AsShape().Placement);
            }
        }

        /// <summary>The position part of the shape's Location.</summary>
        public Point3D Position
        {
            get
            {
                self.NeedsGeometry("position");
                return self.Location.Position;
            }
        }

        /// <summary>The orientation part of the shape's Location, as rotations in degrees.</summary>
        public Vector3D OrientationAngles
        {
            get
            {
                self.NeedsGeometry("orientation");
                return self.Location.Orientation;
            }
        }

        // ---- Measures ----

        /// <summary>The surface area of all the faces in this shape.</summary>
        public double Area => self.IsEmpty ? 0.0 : TopoUtil.Area(self.AsShape());

        /// <summary>The measure OCCT computes for the shape's kind: length, area or volume (build123d's <c>compute_volume</c>).</summary>
        public double ComputeVolume()
        {
            if (self.IsEmpty || self.AsShape().IsVertex())
                return 0.0;
            return TopoUtil.ShapeTypeOf(self.AsShape()) switch
            {
                ShapeType.Edge or ShapeType.Wire => self.AsShape().IntegrateLength(),
                ShapeType.Face or ShapeType.Shell => self.AsShape().IntegrateArea(TopoUtil.Measured),
                _ => self.AsShape().IntegrateVolume(TopoUtil.Measured),
            };
        }

        /// <summary>The inertia tensor of the shape about its centre of mass, as rows.</summary>
        public ImmutableArray<ImmutableArray<double>> MatrixOfInertia
        {
            get
            {
                if (self.IsEmpty)
                    return [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
                Span<double> m = stackalloc double[9];
                self.AsShape().MassProperties(Dimension.Volume, TopoUtil.MeasuredMoments).Inertia.CopyTo(m);
                return [[m[0], m[1], m[2]], [m[3], m[4], m[5]], [m[6], m[7], m[8]]];
            }
        }

        /// <summary>The principal axes of inertia, each with its moment.</summary>
        public IReadOnlyList<(Direction3D Axis, double Moment)> PrincipalProperties
        {
            get
            {
                self.NeedsGeometry("principal axes");
                var props = self.AsShape().MassProperties(Dimension.Volume, TopoUtil.MeasuredMoments);
                var (m1, m2, m3) = props.PrincipalMoments;
                var (a1, a2, a3) = props.PrincipalAxes;
                return [(a1, m1), (a2, m2), (a3, m3)];
            }
        }

        /// <summary>The first moments of mass (Mx, My, Mz).</summary>
        public (double Mx, double My, double Mz) StaticMoments
        {
            get
            {
                if (self.IsEmpty)
                    return (0, 0, 0);
                var moments = self.AsShape().MassProperties(Dimension.Volume, TopoUtil.MeasuredMoments).StaticMoments;
                return (moments.X, moments.Y, moments.Z);
            }
        }

        /// <summary>The radius of gyration of the shape about an axis.</summary>
        public double RadiusOfGyration(Axis axis)
        {
            self.NeedsGeometry("radius of gyration");
            var ax1 = axis.ToAxis3D();
            return self.AsShape().MassProperties(Dimension.Volume, TopoUtil.MeasuredMoments).RadiusOfGyration(ax1);
        }

        // ---- Checks ----

        /// <summary>True if OCCT finds no defect in the shape or its sub-shapes (<c>BRepCheck_Analyzer</c>).</summary>
        public bool IsValid
        {
            get
            {
                if (self.IsEmpty)
                    return true;
                return self.AsShape().Validate().IsValid;
            }
        }

        /// <summary>
        /// True if each non-degenerate edge belongs to exactly two faces and the faces are consistently oriented: the shape
        /// is watertight. Faces that overlap or cross each other in space aren't detected; use <c>IsValid</c> for that.
        /// </summary>
        public bool IsManifold
        {
            get
            {
                if (self.IsEmpty)
                    return false;
                foreach (var shape in TopoUtil.TopLevelShapes(self.AsShape()))
                {
                    // The faces each edge is used by, once per use (TopExp::MapShapesAndAncestors): a seam counts its face twice
                    var uses = new Dictionary<Shape, int>(ShapeComparer.Same);
                    foreach (var face in shape.Explore<Face>())
                        foreach (var e in face.Explore<Edge>())
                            uses[e] = uses.GetValueOrDefault(e) + 1;
                    foreach (var (e, count) in uses)
                    {
                        // A degenerate edge (a sphere's pole, a cone's apex) belongs to one face only
                        if (((Edge)e).IsDegenerated)
                            continue;
                        if (count != 2)
                            return false;
                    }
                    // In a closed shell each edge is used once in each direction
                    if (shape.MisorientedEdges(checkInternalEdges: true).Length > 0)
                        return false;
                }
                return true;
            }
        }

        // ---- Bounding boxes ----

        /// <summary>
        /// A box aligned with the axes containing this shape: tight unless <paramref name="optimal"/> is false, falling back
        /// on a mesh for a shape with no geometry to bound, and empty for an empty shape. The kernel's <c>BoundingBox()</c>
        /// is the fast, loose one, and doesn't fall back.
        /// </summary>
        public BoundingBox Bounds(bool optimal = true) => BoxOf(self.AsShape(), optimal);

        /// <summary>The smallest box in any orientation containing this shape.</summary>
        public OrientedBoundBox OrientedBoundingBox()
        {
            return Kerf.Modeling.OrientedBoundBox.FromOrientedBox(self.IsEmpty ? Kerf.Geometry.OrientedBox.Empty : self.AsShape().OrientedBox());
        }

        // ---- Distances ----

        /// <summary>The minimal distance between this shape and another.</summary>
        /// <exception cref="InvalidOperationException">Either shape is empty.</exception>
        public double Distance(Shape other)
        {
            if (self.IsEmpty || other.IsEmpty)
                throw new InvalidOperationException("Cannot calculate distance to or from an empty shape");
            return Failures.Get(MeasureOperations.Distance(self.AsShape(), other), () => "distance").Value;
        }

        /// <summary>The minimal distance between this shape and another shape.</summary>
        public double DistanceTo(Shape other) => self.DistanceToWithClosestPoints(other).Distance;

        /// <summary>The minimal distance between this shape and a point.</summary>
        public double DistanceTo(Point3D point) => self.DistanceToWithClosestPoints(point).Distance;

        /// <summary>The points on this shape and another where they are closest.</summary>
        public (Point3D OnThis, Point3D OnOther) ClosestPoints(Shape other)
        {
            var (_, a, b) = self.DistanceToWithClosestPoints(other);
            return (a, b);
        }

        /// <summary>The points on this shape and at <paramref name="point"/> where they are closest.</summary>
        public (Point3D OnThis, Point3D OnOther) ClosestPoints(Point3D point)
        {
            var (_, a, b) = self.DistanceToWithClosestPoints(point);
            return (a, b);
        }

        /// <summary>The minimal distance to another shape and the points on each where it is found.</summary>
        public (double Distance, Point3D OnThis, Point3D OnOther) DistanceToWithClosestPoints(Shape other)
        {
            if (self.IsEmpty || other.IsEmpty)
                throw new InvalidOperationException("Cannot calculate distance to or from an empty shape");
            return self.DistanceToTopo(other);
        }

        /// <summary>The minimal distance to a point and the points on each where it is found.</summary>
        public (double Distance, Point3D OnThis, Point3D OnOther) DistanceToWithClosestPoints(Point3D point)
        {
            if (self.IsEmpty)
                throw new InvalidOperationException("Cannot calculate distance to or from an empty shape");
            var pnt = point;
            return self.DistanceToTopo(Vertex.At(pnt));
        }

        private (double, Point3D, Point3D) DistanceToTopo(Shape other)
        {
            var distance = Failures.Get(MeasureOperations.Distance(self.AsShape(), other), () => "distance");
            var nearest = distance.Solutions[0];
            return (distance.Value, nearest.OnA, nearest.OnB);
        }

        /// <summary>The minimal distances from this shape to each of <paramref name="others"/>.</summary>
        public IEnumerable<double> Distances(params IEnumerable<Shape> others)
        {
            if (self.IsEmpty)
                throw new InvalidOperationException("Cannot calculate distance to or from an empty shape");
            foreach (var other in others)
            {
                if (other.IsEmpty)
                    throw new InvalidOperationException("Cannot calculate distance to or from an empty shape");
                yield return Failures.Get(MeasureOperations.Distance(self.AsShape(), other), () => "distance").Value;
            }
        }

        /// <summary>The faces of this shape an axis passes through, nearest the axis's position first.</summary>
        public ShapeList<Face> FacesIntersectedByAxis(Axis axis, double tolerance = 1e-4)
        {
            if (self.IsEmpty)
                return [];
            var ax1 = axis.ToAxis3D();
            var line = new Line3D(ax1);
            var origin = axis.Position;
            var found = new List<(Face Face, double Distance)>();
            foreach (var hit in MeasureOperations.Intersect(self.AsShape(), in line, tolerance))
                found.Add((hit.Face, Math.Abs(origin.SquareDistanceTo(hit.Point))));
            return new ShapeList<Face>(found.OrderBy(f => f.Distance).Select(f => (Face)(f.Face)));
        }

        // ---- Meshing ----

        /// <summary>A triangulated approximation: the vertices and the triangles as index triples into them.</summary>
        public (IReadOnlyList<Point3D> Vertices, IReadOnlyList<(int, int, int)> Triangles) Tessellate(double tolerance, double angularTolerance = 0.1)
        {
            if (self.IsEmpty)
                throw new InvalidOperationException("Cannot tessellate an empty shape");
            // The kernel's mesher leaves the shape alone and hands the meshes back (unit 10L3)
            var options = new MeshOptions { LinearDeflection = tolerance, AngularDeflection = angularTolerance, Relative = true, Parallel = true };
            var mesh = Failures.GetWarnedWithCodes(self.AsShape().Mesh(options), () => "tessellate");
            var meshes = new Dictionary<Shape, global::Kerf.Meshing.FaceMesh>(ShapeComparer.Same);
            foreach (var faceMesh in mesh.Faces)
                meshes.TryAdd(faceMesh.Face, faceMesh);
            var vertices = new List<Point3D>();
            var triangles = new List<(int, int, int)>();
            int offset = 0;
            foreach (var face in self.Faces())
            {
                if (!meshes.TryGetValue(face, out var faceMesh) || faceMesh.TriangleCount == 0)
                    continue;
                bool reverse = face.Orientation == Orientation.Reversed;
                // The mesh's triangles follow the surface's normal, flipped again by a mirroring placement: this follows the face
                bool mirrored = faceMesh.Face.Placement.Transformation.IsNegative;
                var nodes = faceMesh.GetNodes();
                foreach (var node in nodes)
                    vertices.Add(new Point3D(node.X, node.Y, node.Z));
                foreach (var t in faceMesh.GetTriangles())
                {
                    int a = t.A + offset, b = t.B + offset, c = t.C + offset;
                    triangles.Add(mirrored ? (a, c, b) : (a, b, c));
                }
                offset += nodes.Length;
            }
            return (vertices, triangles);
        }
    }

    extension(Shape)
    {

        /// <summary>The centre of several shapes, weighted by their volume (mass) or by their bounding boxes.</summary>
        /// <exception cref="ArgumentException"><see cref="CenterOf.Geometry"/> isn't implemented.</exception>
        public static Point3D CombinedCenter<T>(IEnumerable<T> objects, CenterOf centerOf = CenterOf.Mass) where T : IShape
        {
            var list = objects.ToList();
            switch (centerOf)
            {
                case CenterOf.Mass:
                {
                    var totalMass = list.Sum(o => o.ComputeVolume());
                    var sum = list.Select(o => o.Center(CenterOf.Mass).ToVector() * o.ComputeVolume()).Aggregate((a, b) => a + b);
                    return (sum * (1.0 / totalMass)).ToPoint();
                }
                case CenterOf.BoundingBox:
                {
                    var sum = list.Select(o => o.Bounds().Center().ToVector()).Aggregate((a, b) => a + b);
                    return (sum * (1.0 / list.Count)).ToPoint();
                }
                default:
                    throw new ArgumentException("CenterOf.Geometry not implemented");
            }
        }
    }


    /// <summary>The geometry type of a kind of curve.</summary>
    internal static GeomType GeomTypeOf(CurveKind type) => type switch
    {
        CurveKind.Line => GeomType.Line,
        CurveKind.Circle => GeomType.Circle,
        CurveKind.Ellipse => GeomType.Ellipse,
        CurveKind.Hyperbola => GeomType.Hyperbola,
        CurveKind.Parabola => GeomType.Parabola,
        CurveKind.BezierCurve => GeomType.Bezier,
        CurveKind.BSplineCurve => GeomType.BSpline,
        CurveKind.OffsetCurve => GeomType.Offset,
        _ => GeomType.Other,
    };

    /// <summary>The geometry type of a kind of surface.</summary>
    internal static GeomType GeomTypeOf(SurfaceKind type) => type switch
    {
        SurfaceKind.Plane => GeomType.Plane,
        SurfaceKind.Cylinder => GeomType.Cylinder,
        SurfaceKind.Cone => GeomType.Cone,
        SurfaceKind.Sphere => GeomType.Sphere,
        SurfaceKind.Torus => GeomType.Torus,
        SurfaceKind.BezierSurface => GeomType.Bezier,
        SurfaceKind.BSplineSurface => GeomType.BSpline,
        SurfaceKind.SurfaceOfRevolution => GeomType.Revolution,
        SurfaceKind.SurfaceOfExtrusion => GeomType.Extrusion,
        SurfaceKind.OffsetSurface => GeomType.Offset,
        _ => GeomType.Other,
    };

    /// <summary>The mass properties for the shape's kind (length, area or volume); null for a vertex, which has none.</summary>
    internal static MassProperties? MassPropertiesOf(Shape shape) => TopoUtil.ShapeTypeOf(shape) switch
    {
        ShapeType.Vertex => null,
        ShapeType.Edge or ShapeType.Wire => shape.MassProperties(Dimension.Length),
        ShapeType.Face or ShapeType.Shell => shape.MassProperties(Dimension.Area, TopoUtil.MeasuredMoments),
        _ => shape.MassProperties(Dimension.Volume, TopoUtil.MeasuredMoments),
    };
}
