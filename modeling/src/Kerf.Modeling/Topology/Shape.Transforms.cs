using Kerf.Construction;
using Kerf.Healing;

namespace Kerf.Modeling;

/// <summary>The transformations both forms of a shape's operations share.</summary>
internal static class ShapeTransforms
{
    internal static Transform3D Mirror(Plane? mirrorPlane)
    {
        mirrorPlane ??= Plane.XY;
        var ax2 = new Frame3D(mirrorPlane.Origin, mirrorPlane.ZDir.ToDirection());
        return new Transform3D().WithMirror(ax2);
    }

    internal static Transform3D Rotation(Axis axis, double angle) =>
        new Transform3D().WithRotation(axis.ToAxis3D(), angle * Constants.DegToRad);

    internal static Transform3D Translation(Vector3D vector) => new Transform3D().WithTranslation(vector);

    internal static Transform3D ScaleAbout(Point3D point, double factor)
    {
        CheckScale(factor);
        return new Transform3D().WithScale(point, factor);
    }

    // A factor of zero collapses a shape to nothing that is still a shape of its kind
    private static void CheckScale(double factor)
    {
        if (factor == 0 || !double.IsFinite(factor))
            throw new ArgumentException($"A scale factor must be finite and not zero: this one is {factor}.");
    }

    // Zero is allowed only along an axis the shape has no extent on: a sketch in XY scaled by (2, 1, 0)
    private static void CheckScale(Shape shape, Vector3D factors)
    {
        if (factors.X == 0 || factors.Y == 0 || factors.Z == 0)
        {
            Vector3D size = shape.Bounds().Size;
            if (factors.X == 0 && size.X > Constants.Tolerance || factors.Y == 0 && size.Y > Constants.Tolerance || factors.Z == 0 && size.Z > Constants.Tolerance)
                throw new ArgumentException($"A scale factor of zero would collapse this shape: {factors.X}, {factors.Y}, {factors.Z} on a size of {size.X}, {size.Y}, {size.Z}.");
        }
        if (factors.X != 0)
            CheckScale(factors.X);
        if (factors.Y != 0)
            CheckScale(factors.Y);
        if (factors.Z != 0)
            CheckScale(factors.Z);
    }

    internal static GeneralTransform3D ScaleMatrix(Shape shape, Vector3D factors, Point3D p)
    {
        CheckScale(shape, factors);
        return ScaleValues(factors, p);
    }

    private static GeneralTransform3D ScaleValues(Vector3D factors, Point3D p) => GeneralTransform3D.FromValues(new double[,]
    {
        { factors.X, 0, 0, p.X * (1 - factors.X) },
        { 0, factors.Y, 0, p.Y * (1 - factors.Y) },
        { 0, 0, factors.Z, p.Z * (1 - factors.Z) },
        { 0, 0, 0, 1 },
    });

    /// <summary>The rotation in degrees about X, Y then Z, and then the move by <paramref name="offset"/>.</summary>
    internal static Transform3D Euler(Vector3D rotate, Vector3D offset)
    {
        var radians = rotate * Constants.DegToRad;
        var origin = new Point3D(0, 0, 0);
        Transform3D About(double x, double y, double z, double angle) =>
            new Transform3D().WithRotation(new Axis3D(origin, new Direction3D(x, y, z)), angle);
        var to = new Transform3D().WithTranslation(offset);
        return to.Multiplied(About(1, 0, 0, radians.X)).Multiplied(About(0, 1, 0, radians.Y)).Multiplied(About(0, 0, 1, radians.Z));
    }

    /// <summary>The geometry itself transformed (not just the location).</summary>
    internal static Built<Shape> Apply(Shape shape, Transform3D transformation) =>
        Failures.Get(shape.Transformed(transformation, copyGeometry: true), () => "transform");

    /// <summary>The shape with the matrix applied as its affine transform.</summary>
    internal static Shape TransformGeometry(Shape shape, GeneralTransform3D matrix)
    {
        if (matrix.Form == TransformForm.Other)
            // A general affine transform (stretch, shear): only GTransform can apply it, converting geometry to splines
            return Failures.Get(shape.Transformed(matrix, copyGeometry: true), () => "transform").Shape;
        // A similarity: Transform applies it exactly and keeps the geometry types; GTransform would drop its
        // scale factor, which a gp_GTrsf of any recognised form keeps apart from its matrix
        return Failures.Get(shape.Transformed(matrix.ToTransform(), copyGeometry: true), () => "transform").Shape;
    }

    /// <summary>The shape with the matrix applied by the kernel's <c>Transformed</c>, as a plane repositions shapes.</summary>
    internal static Shape TransformedBy(Shape shape, GeneralTransform3D matrix) =>
        TopoUtil.IsNothing(shape) ? shape : Failures.Get(shape.Transformed(matrix.ToTransform()), () => "transform").Shape;

    /// <summary>The shape with the matrix applied keeping every geometry type.</summary>
    internal static Shape TransformShape(Shape shape, GeneralTransform3D matrix) =>
        shape.TryAs(out Vertex vertex) && shape.IsVertex()
            ? Vertex.Make(matrix * vertex.Point)
            : Failures.Get(shape.Transformed(Rigid(matrix.ToTransform())), () => "transform").Shape;

    // The kernel's ToTransform takes a similarity given as a matrix since its decision 22607; keeping the geometry needs it rigid
    private static Transform3D Rigid(Transform3D transform) =>
        Math.Abs(Math.Abs(transform.ScaleFactor) - 1) <= 1e-12
            ? transform
            : throw new InvalidOperationException($"A transformation of the shape keeps its geometry, so it must be rigid; this one scales by {transform.ScaleFactor}.");

    internal static Shape ToSplines(Shape shape, int degree, double tolerance, bool nurbs)
    {
        if (TopoUtil.IsNothing(shape))
            throw new InvalidOperationException("Cannot approximate an empty shape");
        var options = new BSplineApproximationOptions
        {
            Tolerance3D = tolerance,
            Tolerance2D = tolerance,
            MaxDegree = degree,
            MaxSegments = 1,
            Continuity3D = Continuity.C0,
            Continuity2D = Continuity.C0,
            PreferDegree = true,
            Polynomial = !nurbs,
        };
        return Failures.Get(shape.ApproximateBSplines(options), () => "approximate as splines").Shape;
    }

    /// <summary>The shape cleaned (best effort: on failure the uncleaned shape is still usable), and the kernel's record.</summary>
    internal static (Shape Shape, ModelHistory? Record) Clean(Shape shape)
    {
        if (!shape.Unify(ShapeExtensions.CleanOptions).TryGetValue(out var built))
        {
            Warnings.Warn($"Unable to clean {shape.Describe()}");
            return (shape, null);
        }
        return (TopoUtil.Downcast(built.Shape), ModelHistory.FromHistory(built.History));
    }

    /// <summary>The record after a transformation: the one before, then what the transformation did.</summary>
    internal static ModelHistory ThroughTransform(ModelHistory record, Shape before, Built<Shape> built) =>
        new ModelHistory(record.Before, record.Brought)
            .Merge(record)
            .Merge(ModelHistory.FromAlgorithm(s => built.History.Modified(s), s => built.History.Generated(s), [before], built.Shape));
}

/// <summary>
/// The operations on the kernel's shapes that return a shape of the same type: <c>edge.Moved(loc)</c> is an
/// <see cref="Edge"/>. They stand for build123d's methods returning <c>Self</c>, and for its mutators (<c>locate</c>,
/// <c>move</c>, setters), which here return a changed copy instead.
/// </summary>
public static class ShapeOperations
{
    extension<T>(T shape) where T : struct, IShape<T>
    {
        private T Narrowed(Shape result) => TopoUtil.IsNothing(result) ? default : result.As<T>();

        /// <summary>A copy placed at <paramref name="location"/> in the absolute sense, sharing this shape's geometry.</summary>
        public T Located(Location location) => shape.IsEmpty ? shape : shape.Narrowed(shape.AsShape().Located(location.Wrapped));

        /// <summary>A copy moved by <paramref name="location"/> relative to where it is now, sharing this shape's geometry.</summary>
        public T Moved(Location location) => shape.IsEmpty ? shape : shape.Narrowed(shape.AsShape().Moved(location.Wrapped));

        /// <summary>A copy moved onto <paramref name="plane"/>, relative to where it is now.</summary>
        public T Moved(Plane plane) => shape.Moved(plane.Location);

        /// <summary>A copy with extraneous internal edges removed (the kernel's <c>Unify</c>).</summary>
        public T Clean()
        {
            if (shape.IsEmpty)
                return shape;
            var before = HistoryLog.Current?.Find(shape.AsShape());
            var (result, unified) = ShapeTransforms.Clean(shape.AsShape());
            if (unified is not null)
                result.MadeBy(before is null ? unified : before.Merge(unified));
            return shape.Narrowed(result);
        }

        /// <summary>The shape fixed by the kernel's healing if it isn't valid, else this shape.</summary>
        public T Fix() => shape.IsEmpty || shape.IsValid ? shape : shape.Narrowed(TopoUtil.Fix(shape.AsShape()));

        /// <summary>A copy mirrored about <paramref name="mirrorPlane"/> (default <see cref="Plane.XY"/>), without the original.</summary>
        public T Mirror(Plane? mirrorPlane = null) => shape.IsEmpty ? shape : shape.Narrowed(ShapeTransforms.Apply(shape.AsShape(), ShapeTransforms.Mirror(mirrorPlane)).Shape);

        /// <summary>
        /// A copy rotated about <paramref name="axis"/> by <paramref name="angle"/> degrees. With <paramref name="transform"/>
        /// the geometry is regenerated instead of just changing the location.
        /// </summary>
        public T Rotate(Axis axis, double angle, bool transform = false)
        {
            if (shape.IsEmpty)
                return shape;
            var t = ShapeTransforms.Rotation(axis, angle);
            return transform ? shape.Narrowed(ShapeTransforms.Apply(shape.AsShape(), t).Shape) : shape.Moved(new Location(t));
        }

        /// <summary>
        /// A copy moved by <paramref name="vector"/>. With <paramref name="transform"/> the geometry is regenerated instead
        /// of just changing the location.
        /// </summary>
        public T Translate(Vector3D vector, bool transform = false)
        {
            if (shape.IsEmpty)
                return shape;
            var t = ShapeTransforms.Translation(vector);
            return transform ? shape.Narrowed(ShapeTransforms.Apply(shape.AsShape(), t).Shape) : shape.Moved(new Location(t));
        }

        /// <summary>A copy scaled uniformly about <paramref name="about"/> (default the shape's position).</summary>
        public T Scale(double factor, Point3D? about = null) =>
            shape.IsEmpty ? shape : shape.Narrowed(ShapeTransforms.Apply(shape.AsShape(), ShapeTransforms.ScaleAbout(about ?? shape.Location.Position, factor)).Shape);

        /// <summary>
        /// A copy scaled by separate factors in X, Y and Z about <paramref name="about"/> (default the shape's position).
        /// Non-uniform scaling may turn the geometry into splines.
        /// </summary>
        public T Scale(Vector3D factors, Point3D? about = null) =>
            shape.IsEmpty ? shape : shape.Narrowed(ShapeTransforms.TransformGeometry(shape.AsShape(), ShapeTransforms.ScaleMatrix(shape.AsShape(), factors, about ?? shape.Location.Position)));

        /// <summary>
        /// A copy with an affine transform applied to its geometry. Handles stretch and skew, at the cost of sometimes
        /// turning lines and circles into splines; <see cref="TransformShape"/> keeps the geometry types.
        /// </summary>
        public T TransformGeometry(GeneralTransform3D matrix) => shape.IsEmpty ? shape : shape.Narrowed(ShapeTransforms.TransformGeometry(shape.AsShape(), matrix));

        /// <summary>A copy transformed (translation and rotation, chiefly) keeping every geometry type.</summary>
        public T TransformShape(GeneralTransform3D matrix) => shape.IsEmpty ? shape : shape.Narrowed(ShapeTransforms.TransformShape(shape.AsShape(), matrix));

        /// <summary>A copy rotated by the angles in degrees about X, Y then Z, and then moved by <paramref name="offset"/>.</summary>
        public T Transformed(Vector3D? rotate = null, Vector3D? offset = null) =>
            shape.IsEmpty ? shape : shape.Narrowed(ShapeTransforms.Apply(shape.AsShape(), ShapeTransforms.Euler(rotate ?? Vector3D.Zero, offset ?? Vector3D.Zero)).Shape);

        /// <summary>A copy with its free-form geometry made B-splines of at most <paramref name="degree"/>; planes and elementary surfaces stay as they are, by ShapeCustom's defaults.</summary>
        public T ToSplines(int degree = 3, double tolerance = 1e-3, bool nurbs = false) => shape.Narrowed(ShapeTransforms.ToSplines(shape.AsShape(), degree, tolerance, nurbs));

        /// <summary>A copy at <paramref name="location"/> (build123d's <c>location</c> setter).</summary>
        public T WithLocation(Location location) => shape.Located(location);

        /// <summary>A copy with its Location's position replaced (build123d's <c>position</c> setter).</summary>
        public T WithPosition(Point3D position) => shape.IsEmpty ? shape : shape.WithLocation(shape.Location.WithPosition(position));

        /// <summary>A copy with its Location's orientation replaced (build123d's <c>orientation</c> setter).</summary>
        public T WithOrientation(Vector3D orientation) => shape.IsEmpty ? shape : shape.WithLocation(shape.Location.WithOrientation(orientation));

        /// <summary>This shape as the object of its dimension (<see cref="ModelShape.For"/>), with a label: a kernel shape can't hold one.</summary>
        public ModelShape WithLabel(string label) => ModelShape.For(shape.AsShape()).WithLabel(label);

        /// <summary>This shape as the object of its dimension (<see cref="ModelShape.For"/>), with a colour.</summary>
        public ModelShape WithColor(Color? color) => ModelShape.For(shape.AsShape()).WithColor(color);

        /// <summary>This shape as the object of its dimension (<see cref="ModelShape.For"/>), marked (or not) as construction geometry.</summary>
        public ModelShape WithForConstruction(bool forConstruction = true) => ModelShape.For(shape.AsShape()).WithForConstruction(forConstruction);

        /// <summary>The shape moved by a location (build123d's <c>location * shape</c>).</summary>
        public static T operator *(Location location, T s) => s.Moved(location);

        /// <summary>The shape moved onto a plane (build123d's <c>plane * shape</c>).</summary>
        public static T operator *(Plane plane, T s) => s.Moved(plane);

        /// <summary>A copy of the shape moved by each location.</summary>
        public static IReadOnlyList<T> operator *(IEnumerable<Location> locations, T s) => [.. locations.Select(l => s.Moved(l))];

        /// <summary>A copy of the shape moved onto each plane.</summary>
        public static IReadOnlyList<T> operator *(IEnumerable<Plane> planes, T s) => [.. planes.Select(p => s.Moved(p))];
    }
}

/// <summary>
/// The operations on the modelling layer's objects that return an object of the caller's own class: <c>box.Moved(loc)</c>
/// is a Box, with its label, colour and history.
/// </summary>
public static class ModelShapeOperations
{
    extension<T>(T shape) where T : ModelShape
    {
        private T PlacedAs(Shape result) => (T)shape.Placed(result);

        // The geometry itself transformed, and the history carried through
        private T Applied(Transform3D transformation)
        {
            if (shape.IsEmpty)
                return shape;
            var built = ShapeTransforms.Apply(shape.Shape, transformation);
            var copy = shape.Rewrapped(built.Shape);
            if (shape.History is { } record)
                copy = copy.MadeBy(ShapeTransforms.ThroughTransform(record, shape.Shape, built));
            return (T)copy;
        }

        /// <summary>A copy placed at <paramref name="location"/> in the absolute sense, sharing this object's geometry.</summary>
        public T Located(Location location) =>
            // Nothing is nothing wherever it is put
            shape.PlacedAs(shape.IsEmpty ? TopoUtil.EmptyCompound() : shape.Shape.Located(location.Wrapped));

        /// <summary>A copy moved by <paramref name="location"/> relative to where it is now, sharing this object's geometry.</summary>
        public T Moved(Location location) =>
            shape.PlacedAs(shape.IsEmpty ? TopoUtil.EmptyCompound() : shape.Shape.Moved(location.Wrapped));

        /// <summary>A copy moved onto <paramref name="plane"/>, relative to where it is now.</summary>
        public T Moved(Plane plane) => shape.Moved(plane.Location);

        /// <summary>A copy with extraneous internal edges removed (the kernel's <c>Unify</c>).</summary>
        public T Clean()
        {
            if (shape.IsEmpty)
                return shape;
            var (cleaned, unified) = ShapeTransforms.Clean(shape.Shape);
            if (unified is null)
                return shape;
            var record = shape.HistoryHere();
            return (T)shape.Placed(cleaned).MadeBy(record is null ? unified : record.Merge(unified));
        }

        /// <summary>The object fixed by the kernel's healing if it isn't valid, else this object.</summary>
        public T Fix() => shape.IsEmpty || shape.IsValid ? shape : (T)shape.Rewrapped(TopoUtil.Fix(shape.Shape));

        /// <summary>A copy mirrored about <paramref name="mirrorPlane"/> (default <see cref="Plane.XY"/>), without the original.</summary>
        public T Mirror(Plane? mirrorPlane = null) => shape.Applied(ShapeTransforms.Mirror(mirrorPlane));

        /// <summary>
        /// A copy rotated about <paramref name="axis"/> by <paramref name="angle"/> degrees. With <paramref name="transform"/>
        /// the geometry is regenerated instead of just changing the location.
        /// </summary>
        public T Rotate(Axis axis, double angle, bool transform = false)
        {
            if (shape.IsEmpty)
                return shape;
            var t = ShapeTransforms.Rotation(axis, angle);
            return transform ? shape.Applied(t) : shape.Moved(new Location(t));
        }

        /// <summary>
        /// A copy moved by <paramref name="vector"/>. With <paramref name="transform"/> the geometry is regenerated instead
        /// of just changing the location.
        /// </summary>
        public T Translate(Vector3D vector, bool transform = false)
        {
            if (shape.IsEmpty)
                return shape;
            var t = ShapeTransforms.Translation(vector);
            return transform ? shape.Applied(t) : shape.Moved(new Location(t));
        }

        /// <summary>A copy scaled uniformly about <paramref name="about"/> (default the object's position).</summary>
        public T Scale(double factor, Point3D? about = null) =>
            shape.IsEmpty ? shape : shape.Applied(ShapeTransforms.ScaleAbout(about ?? shape.Location.Position, factor));

        /// <summary>
        /// A copy scaled by separate factors in X, Y and Z about <paramref name="about"/> (default the object's position).
        /// Non-uniform scaling may turn the geometry into splines.
        /// </summary>
        public T Scale(Vector3D factors, Point3D? about = null) =>
            shape.IsEmpty ? shape : shape.TransformGeometry(ShapeTransforms.ScaleMatrix(shape.Shape, factors, about ?? shape.Location.Position));

        /// <summary>
        /// A copy with an affine transform applied to its geometry. Handles stretch and skew, at the cost of sometimes
        /// turning lines and circles into splines; <see cref="TransformShape"/> keeps the geometry types.
        /// </summary>
        public T TransformGeometry(GeneralTransform3D matrix) => shape.IsEmpty ? shape : (T)shape.Rewrapped(ShapeTransforms.TransformGeometry(shape.Shape, matrix));

        /// <summary>A copy transformed (translation and rotation, chiefly) keeping every geometry type.</summary>
        public T TransformShape(GeneralTransform3D matrix) => shape.IsEmpty ? shape : (T)shape.Rewrapped(ShapeTransforms.TransformShape(shape.Shape, matrix));

        /// <summary>A copy rotated by the angles in degrees about X, Y then Z, and then moved by <paramref name="offset"/>.</summary>
        public T Transformed(Vector3D? rotate = null, Vector3D? offset = null) =>
            shape.Applied(ShapeTransforms.Euler(rotate ?? Vector3D.Zero, offset ?? Vector3D.Zero));

        /// <summary>A copy with its free-form geometry made B-splines of at most <paramref name="degree"/>; planes and elementary surfaces stay as they are, by ShapeCustom's defaults.</summary>
        public T ToSplines(int degree = 3, double tolerance = 1e-3, bool nurbs = false) =>
            (T)shape.Placed(ShapeTransforms.ToSplines(shape.Shape, degree, tolerance, nurbs));

        /// <summary>A copy at <paramref name="location"/> (build123d's <c>location</c> setter).</summary>
        public T WithLocation(Location location) => shape.IsEmpty ? shape : shape.PlacedAs(shape.Shape.Located(location.Wrapped));

        /// <summary>A copy with its Location's position replaced (build123d's <c>position</c> setter).</summary>
        public T WithPosition(Point3D position) => shape.IsEmpty ? shape : shape.WithLocation(shape.Location.WithPosition(position));

        /// <summary>A copy with its Location's orientation replaced (build123d's <c>orientation</c> setter).</summary>
        public T WithOrientation(Vector3D orientation) => shape.IsEmpty ? shape : shape.WithLocation(shape.Location.WithOrientation(orientation));

        /// <summary>A copy with another label.</summary>
        public T WithLabel(string label) => (T)shape.WithLabelCore(label);

        /// <summary>A copy with another colour (or none).</summary>
        public T WithColor(Color? color) => (T)shape.WithColorCore(color);

        /// <summary>A copy marked (or not) as construction geometry.</summary>
        public T WithForConstruction(bool forConstruction = true) => (T)shape.WithForConstructionCore(forConstruction);

        /// <summary>The object moved by a location (build123d's <c>location * shape</c>).</summary>
        public static T operator *(Location location, T s) => s.Moved(location);

        /// <summary>The object moved onto a plane (build123d's <c>plane * shape</c>).</summary>
        public static T operator *(Plane plane, T s) => s.Moved(plane);

        /// <summary>A copy of the object moved by each location.</summary>
        public static IReadOnlyList<T> operator *(IEnumerable<Location> locations, T s) => [.. locations.Select(l => s.Moved(l))];

        /// <summary>A copy of the object moved onto each plane.</summary>
        public static IReadOnlyList<T> operator *(IEnumerable<Plane> planes, T s) => [.. planes.Select(p => s.Moved(p))];
    }
}
