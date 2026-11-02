using Build123d.Occt;

namespace Build123d;

public abstract partial class Shape
{
    // The class-preserving operations. Each is reached through ShapeOperations, which casts the result back to the
    // caller's type; a subclass that must adjust its own state overrides the core method.

    internal Shape LocatedCore(Location location)
    {
        // Nothing is nothing wherever it is put
        if (IsEmpty)
            return Placed(TopoUtil.EmptyCompound());
        return Placed(TopoUtil.Downcast(_wrapped.Located(location.Wrapped)));
    }

    internal Shape MovedCore(Location location)
    {
        if (IsEmpty)
            return Placed(TopoUtil.EmptyCompound());
        return Placed(TopoUtil.Downcast(_wrapped.Moved(location.Wrapped)));
    }

    /// <summary>A copy with the transformation applied to the geometry (not just the location), and the history carried through.</summary>
    internal Shape ApplyTransform(gp_Trsf transformation)
    {
        if (IsEmpty)
            return this;
        using var builder = new BRepBuilderAPI_Transform(_wrapped, transformation, true);
        var transformed = builder.Shape();
        var copy = Rewrapped(transformed);
        if (History is not null)
        {
            var history = new ShapeHistory(History.Before, History.Brought)
                .Merge(History)
                .Merge(ShapeHistory.FromAlgorithm(builder, [_wrapped], transformed));
            copy = copy.MadeBy(history);
        }
        return copy;
    }

    /// <summary>A copy with the matrix applied by <c>BRepBuilderAPI_Transform</c>, as Plane repositions shapes.</summary>
    internal Shape TransformedBy(Matrix matrix)
    {
        if (IsEmpty)
            return this;
        using var trsf = matrix.GTrsf.Trsf();
        using var builder = new BRepBuilderAPI_Transform(_wrapped, trsf);
        return Rewrapped(builder.Shape());
    }

    internal Shape CleanCore()
    {
        if (IsEmpty)
            return this;
        using var upgrader = new ShapeUpgrade_UnifySameDomain(_wrapped, true, true, true);
        upgrader.AllowInternalEdges(false);
        // Cleaning is best effort: on failure the uncleaned shape is still usable
        try
        {
            upgrader.Build();
            var record = HistoryHere();
            var unified = ShapeHistory.FromUnify(upgrader);
            return Placed(TopoUtil.Downcast(upgrader.Shape())).MadeBy(record is null ? unified : record.Merge(unified));
        }
        catch (OcctException)
        {
            Warnings.Warn($"Unable to clean {this}");
            return this;
        }
    }

    internal Shape FixCore()
    {
        if (IsEmpty || IsValid)
            return this;
        return Rewrapped(TopoUtil.Fix(_wrapped));
    }

    internal Shape MirrorCore(Plane? mirrorPlane)
    {
        mirrorPlane ??= Plane.XY;
        if (IsEmpty)
            return this;
        using var trsf = new gp_Trsf();
        using var origin = mirrorPlane.Origin.ToGpPnt();
        using var normal = mirrorPlane.ZDir.ToGpDir();
        using var ax2 = new gp_Ax2(origin, normal);
        trsf.SetMirror(ax2);
        return ApplyTransform(trsf);
    }

    internal Shape RotateCore(Axis axis, double angle, bool transform)
    {
        if (IsEmpty)
            return this;
        using var trsf = new gp_Trsf();
        using var ax1 = axis.ToGpAx1();
        trsf.SetRotation(ax1, angle * Constants.DegToRad);
        return transform ? ApplyTransform(trsf) : MovedCore(new Location(trsf));
    }

    internal Shape TranslateCore(Vector vector, bool transform)
    {
        if (IsEmpty)
            return this;
        using var trsf = new gp_Trsf();
        using var vec = vector.ToGpVec();
        trsf.SetTranslation(vec);
        return transform ? ApplyTransform(trsf) : MovedCore(new Location(trsf));
    }

    internal Shape ScaleCore(double factor, Vector? about)
    {
        if (IsEmpty)
            return this;
        var point = about ?? Location.Position;
        using var trsf = new gp_Trsf();
        using var pnt = point.ToGpPnt();
        trsf.SetScale(pnt, factor);
        return ApplyTransform(trsf);
    }

    internal Shape ScaleCore(Vector factors, Vector? about)
    {
        if (IsEmpty)
            return this;
        var p = about ?? Location.Position;
        var matrix = new Matrix(new double[,]
        {
            { factors.X, 0, 0, p.X * (1 - factors.X) },
            { 0, factors.Y, 0, p.Y * (1 - factors.Y) },
            { 0, 0, factors.Z, p.Z * (1 - factors.Z) },
            { 0, 0, 0, 1 },
        });
        return TransformGeometryCore(matrix);
    }

    internal Shape TransformGeometryCore(Matrix matrix)
    {
        if (IsEmpty)
            return this;
        TopoDS_Shape result;
        if (matrix.GTrsf.Form() == gp_TrsfForm.gp_Other)
        {
            // A general affine transform (stretch, shear): only GTransform can apply it, converting geometry to splines
            using var builder = new BRepBuilderAPI_GTransform(_wrapped, matrix.GTrsf, true);
            result = builder.Shape();
        }
        else
        {
            // A similarity: Transform applies it exactly and keeps the geometry types; GTransform would drop its
            // scale factor, which a gp_GTrsf of any recognised form keeps apart from its matrix
            using var trsf = matrix.GTrsf.Trsf();
            using var builder = new BRepBuilderAPI_Transform(_wrapped, trsf, true);
            result = builder.Shape();
        }
        return Rewrapped(result);
    }

    internal virtual Shape TransformShapeCore(Matrix matrix)
    {
        if (IsEmpty)
            return this;
        using var trsf = matrix.GTrsf.Trsf();
        using var builder = new BRepBuilderAPI_Transform(_wrapped, trsf);
        return Rewrapped(builder.Shape());
    }

    internal Shape TransformedCore(Vector rotate, Vector offset)
    {
        var radians = rotate * Constants.DegToRad;
        using var origin = new gp_Pnt(0, 0, 0);
        gp_Trsf About(double x, double y, double z, double angle)
        {
            var t = new gp_Trsf();
            using var dir = new gp_Dir(x, y, z);
            using var ax1 = new gp_Ax1(origin, dir);
            t.SetRotation(ax1, angle);
            return t;
        }
        using var rx = About(1, 0, 0, radians.X);
        using var ry = About(0, 1, 0, radians.Y);
        using var rz = About(0, 0, 1, radians.Z);
        using var to = new gp_Trsf();
        using var vec = offset.ToGpVec();
        to.SetTranslation(vec);
        using var a = to.Multiplied(rx);
        using var b = a.Multiplied(ry);
        using var c = b.Multiplied(rz);
        return ApplyTransform(c);
    }

    internal Shape ToSplinesCore(int degree, double tolerance, bool nurbs)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Cannot approximate an empty shape");
        using var parameters = new ShapeCustom_RestrictionParameters();
        var result = ShapeCustom.BSplineRestriction(_wrapped, tolerance, tolerance, degree, 1,
                                                    GeomAbs_Shape.GeomAbs_C0, GeomAbs_Shape.GeomAbs_C0,
                                                    true, !nurbs, parameters);
        return CastLike(result);
    }

    internal Shape WithLocationCore(Location location)
    {
        if (IsEmpty)
            return this;
        return Placed(TopoUtil.Downcast(_wrapped.Located(location.Wrapped)));
    }
}

/// <summary>
/// The operations on shapes that return a shape of the caller's own class: <c>box.Moved(loc)</c> is a Box. They
/// stand for build123d's methods returning <c>Self</c>, and for its mutators (<c>locate</c>, <c>move</c>, setters),
/// which here return a changed copy instead.
/// </summary>
public static class ShapeOperations
{
    extension<T>(T shape) where T : Shape
    {
        /// <summary>A copy placed at <paramref name="location"/> in the absolute sense, sharing this shape's geometry.</summary>
        public T Located(Location location) => (T)shape.LocatedCore(location);

        /// <summary>A copy moved by <paramref name="location"/> relative to where it is now, sharing this shape's geometry.</summary>
        public T Moved(Location location) => (T)shape.MovedCore(location);

        /// <summary>A copy moved onto <paramref name="plane"/>, relative to where it is now.</summary>
        public T Moved(Plane plane) => (T)shape.MovedCore(plane.Location);

        /// <summary>A copy with extraneous internal edges removed (<c>ShapeUpgrade_UnifySameDomain</c>).</summary>
        public T Clean() => (T)shape.CleanCore();

        /// <summary>The shape fixed by <c>ShapeFix_Shape</c> if it isn't valid, else this shape.</summary>
        public T Fix() => (T)shape.FixCore();

        /// <summary>
        /// A copy carrying a triangulation within <paramref name="tolerance"/> (and <paramref name="angularTolerance"/>
        /// radians), for exporters that write meshes; build123d's <c>mesh</c> meshes the shape in place.
        /// </summary>
        public T Meshed(double tolerance, double angularTolerance = 0.1) => (T)shape.Rewrapped(shape.MeshedCopy(tolerance, angularTolerance));

        /// <summary>A copy mirrored about <paramref name="mirrorPlane"/> (default <see cref="Plane.XY"/>), without the original.</summary>
        public T Mirror(Plane? mirrorPlane = null) => (T)shape.MirrorCore(mirrorPlane);

        /// <summary>
        /// A copy rotated about <paramref name="axis"/> by <paramref name="angle"/> degrees. With <paramref name="transform"/>
        /// the geometry is regenerated instead of just changing the location.
        /// </summary>
        public T Rotate(Axis axis, double angle, bool transform = false) => (T)shape.RotateCore(axis, angle, transform);

        /// <summary>
        /// A copy moved by <paramref name="vector"/>. With <paramref name="transform"/> the geometry is regenerated instead
        /// of just changing the location.
        /// </summary>
        public T Translate(Vector vector, bool transform = false) => (T)shape.TranslateCore(vector, transform);

        /// <summary>A copy scaled uniformly about <paramref name="about"/> (default the shape's position).</summary>
        public T Scale(double factor, Vector? about = null) => (T)shape.ScaleCore(factor, about);

        /// <summary>
        /// A copy scaled by separate factors in X, Y and Z about <paramref name="about"/> (default the shape's position).
        /// Non-uniform scaling may turn the geometry into splines.
        /// </summary>
        public T Scale((double X, double Y, double Z) factors, Vector? about = null) =>
            (T)shape.ScaleCore(new Vector(factors.X, factors.Y, factors.Z), about);

        /// <summary>
        /// A copy with an affine transform applied to its geometry. Handles stretch and skew, at the cost of sometimes
        /// turning lines and circles into splines; <see cref="TransformShape"/> keeps the geometry types.
        /// </summary>
        public T TransformGeometry(Matrix matrix) => (T)shape.TransformGeometryCore(matrix);

        /// <summary>A copy transformed (translation and rotation, chiefly) keeping every geometry type.</summary>
        public T TransformShape(Matrix matrix) => (T)shape.TransformShapeCore(matrix);

        /// <summary>A copy rotated by the angles in degrees about X, Y then Z, and then moved by <paramref name="offset"/>.</summary>
        public T Transformed(Vector? rotate = null, Vector? offset = null) =>
            (T)shape.TransformedCore(rotate ?? new Vector(0, 0, 0), offset ?? new Vector(0, 0, 0));

        /// <summary>A copy with all geometry converted to B-splines of at most <paramref name="degree"/>.</summary>
        public T ToSplines(int degree = 3, double tolerance = 1e-3, bool nurbs = false) => (T)shape.ToSplinesCore(degree, tolerance, nurbs);

        /// <summary>A copy at <paramref name="location"/> (build123d's <c>location</c> setter).</summary>
        public T WithLocation(Location location) => (T)shape.WithLocationCore(location);

        /// <summary>A copy with its Location's position replaced (build123d's <c>position</c> setter).</summary>
        public T WithPosition(Vector position) => shape.IsEmpty ? shape : shape.WithLocation(shape.Location.WithPosition(position));

        /// <summary>A copy with its Location's orientation replaced (build123d's <c>orientation</c> setter).</summary>
        public T WithOrientation(Vector orientation) =>
            shape.IsEmpty ? shape : shape.WithLocation(shape.Location.WithOrientation(orientation));

        /// <summary>A copy with another label.</summary>
        public T WithLabel(string label) => (T)shape.WithLabelCore(label);

        /// <summary>A copy with another colour (or none).</summary>
        public T WithColor(Color? color) => (T)shape.WithColorCore(color);

        /// <summary>A copy marked (or not) as construction geometry.</summary>
        public T WithForConstruction(bool forConstruction = true) => (T)shape.WithForConstructionCore(forConstruction);

        /// <summary>The shape moved by a location (build123d's <c>location * shape</c>).</summary>
        public static T operator *(Location location, T s) => s.Moved(location);

        /// <summary>The shape moved onto a plane (build123d's <c>plane * shape</c>).</summary>
        public static T operator *(Plane plane, T s) => s.Moved(plane);

        /// <summary>A copy of the shape moved by each location.</summary>
        public static IReadOnlyList<T> operator *(IEnumerable<Location> locations, T s) => [.. locations.Select(s.Moved)];

        /// <summary>A copy of the shape moved onto each plane.</summary>
        public static IReadOnlyList<T> operator *(IEnumerable<Plane> planes, T s) => [.. planes.Select(p => s.Moved(p))];
    }
}
