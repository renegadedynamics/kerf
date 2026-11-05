using Kerf.Booleans;
using Kerf.Measurement;

namespace Kerf.Modeling;

/// <summary>
/// build123d's conveniences on a compound, and on the objects that hold one (<see cref="ModelShape"/>): its children, and
/// the volume, centre and contact of a group of shapes.
/// </summary>
public static partial class CompoundExtensions
{
    extension(Compound)
    {
        /// <summary>A compound of shapes; empty shapes add nothing, so a compound of only those is empty.</summary>
        public static Compound Make(IEnumerable<Shape> shapes) => TopoUtil.MakeCompoundOf(shapes);

        /// <summary>A compound of shapes of one type or objects; empty ones add nothing.</summary>
        public static Compound Make<T>(IEnumerable<T> shapes) where T : IShape => TopoUtil.MakeCompoundOf(shapes.Select(s => s.AsShape()));

        /// <summary>A shell extruded into a compound of solids.</summary>
        public static Compound Extrude(Shell obj, Vector3D direction) => (Compound)TopoUtil.Extrude(obj, direction);
    }

    extension<TSelf>(TSelf self) where TSelf : IShape3D
    {
        /// <summary>The shapes directly inside the compound (its kernel children).</summary>
        public IEnumerable<Shape> ChildShapes() => self.AsShape().ChildShapes();

        /// <summary>The number of shapes directly inside the compound.</summary>
        public int Count => self.IsEmpty ? 0 : self.AsShape().ChildCount;

        /// <summary>
        /// The compound without unnecessary compound wrappers: a compound of one shape becomes that shape (or, without
        /// <paramref name="fully"/>, the innermost compound around it).
        /// </summary>
        public Shape Unwrap(bool fully = true) => UnwrapCore(self.AsShape(), fully);
    }

    extension<TSelf>(TSelf self) where TSelf : IShape
    {
        /// <summary>This compound and the compounds directly inside it; none for any other shape.</summary>
        public ShapeList<Compound> Compounds()
        {
            var shape = self.AsShape();
            if (TopoUtil.IsNothing(shape) || !shape.IsCompound())
                return [];
            return [.. shape.ChildShapes().Where(c => c.IsCompound() && !TopoUtil.IsNothing(c)).Select(c => (Compound)c), (Compound)shape];
        }
    }

    extension(Shape self)
    {
        /// <summary>The shapes directly inside this one.</summary>
        internal List<Shape> ChildShapes()
        {
            var children = new List<Shape>();
            if (!self.IsNull)
                foreach (var child in self.Children)
                    children.Add(child);
            return children;
        }
    }

    private static Shape UnwrapCore(Shape shape, bool fully)
    {
        if (shape.IsNull || !shape.IsCompound())
            return shape;
        var children = shape.ChildShapes();
        if (children.Count != 1)
            return shape;
        var single = children[0];
        if (!single.IsCompound())
            return fully ? single : shape;
        return UnwrapCore(single, fully);
    }

    /// <summary>The volume of the solids and shells directly in the compound.</summary>
    internal static double VolumeOf(Shape compound) =>
        compound.ElementsOfTypeOf<Solid>().Sum(s => s.Volume) + compound.ElementsOfTypeOf<Shell>().Sum(s => s.Volume);

    /// <summary>
    /// The shapes of one kind directly inside this compound and the compounds directly inside it (build123d's
    /// <c>get_type</c>): not, as <c>Faces</c> does, the faces of its solids.
    /// </summary>
    public static List<T> ElementsOfType<T>(this IShape3D self) where T : struct, IShape<T> => ElementsOfTypeOf<T>(self.AsShape());

    private static List<T> ElementsOfTypeOf<T>(this Shape self) where T : struct, IShape<T>
    {
        if (T.Type == ShapeType.Shape)
            throw new ArgumentException($"Can't extract {typeof(T).Name}s from a compound");
        var results = new List<T>();
        foreach (var comp in self.Compounds())
            foreach (var child in ((Shape)comp).Children)
                if (child.Type == T.Type)
                    results.Add(child.As<T>());
        return results;
    }

    /// <summary>The centre of mass of the compound's contents (by default) or of its bounding box.</summary>
    internal static Point3D CenterOf(Shape self, CenterOf centerOf)
    {
        self.NeedsGeometry("center");
        switch (centerOf)
        {
            case Kerf.Modeling.CenterOf.Geometry:
                throw new ArgumentException("Center of GEOMETRY is not supported for this object");
            case Kerf.Modeling.CenterOf.Mass:
            {
                var dimension = TopoUtil.UnwrappedShapeType(self) switch
                {
                    ShapeType.Vertex =>
                        throw new NotSupportedException("The centre of mass of a compound of vertices isn't defined"),
                    ShapeType.Edge or ShapeType.Wire => Dimension.Length,
                    ShapeType.Face or ShapeType.Shell => Dimension.Area,
                    _ => Dimension.Volume,
                };
                return self.MassProperties(dimension).CentreOfMass;
            }
            default:
                return self.Bounds().Center();
        }
    }

    internal static ShapeList<Shape>? IntersectCore(Shape self, object other, double tolerance, bool includeTouched)
    {
        // Distributed over the elements: Compound([a, b]) ∩ s = (a ∩ s) ∪ (b ∩ s)
        var shape = ShapeExtensions.ShapeOf(other);
        var selfElements = self.ChildShapes();
        if (selfElements.Count == 0)
            return null;
        var otherElements = shape.IsCompound() ? shape.ChildShapes() : [shape];
        var results = new List<Shape>();
        foreach (var a in selfElements)
            foreach (var b in otherElements)
                if (ShapeExtensions.IntersectOne(a, b, tolerance, includeTouched) is { } found)
                    results.AddRange(found);
        var unique = results.Distinct(ShapeComparer.Same).ToList();
        return unique.Count > 0 ? new ShapeList<Shape>(unique) : null;
    }

    /// <summary>The boundary contacts of the compound's elements (in global coordinates) with another shape.</summary>
    internal static ShapeList<Shape> Touch(Shape self, Shape other, double tolerance) =>
        new(self.ChildShapes().SelectMany(e => e.Touch(other, tolerance)).Distinct(ShapeComparer.Same));
}
