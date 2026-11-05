using Kerf.Geometry;

namespace Kerf.Modeling;

// What was virtual on the old shape classes: the one member, deciding by the kind of shape held. An object (a Part, a Line)
// goes by what it holds, as the class it replaces did.

public static partial class ShapeExtensions
{
    extension<TSelf>(TSelf self) where TSelf : IShape
    {
        /// <summary>The volume of the shape: zero for shapes without one; of a compound, of the solids and shells directly in it.</summary>
        public double Volume => VolumeOf(self.AsShape());

        /// <summary>The centre of the shape, by the calculation its kind uses by default.</summary>
        public Vector Center() => CenterOfShape(self.AsShape());

        /// <summary>The centre of the shape by a chosen calculation.</summary>
        /// <exception cref="NotSupportedException">The kind of shape doesn't offer a choice.</exception>
        public Vector Center(CenterOf centerOf) => CenterOfShape(self.AsShape(), centerOf);

        /// <summary>Where this shape touches <paramref name="other"/> at its boundary: for faces, solids and compounds.</summary>
        public ShapeList<Shape> Touch(Shape other, double tolerance = 1e-6) => TouchOf(self.AsShape(), other, tolerance);
    }

    /// <summary>The dimension of a shape's kind: for the zero of a type, the type's.</summary>
    private static int? DimOfType(Type type) =>
        type == typeof(Vertex) ? 0
        : type == typeof(Edge) || type == typeof(Wire) ? 1
        : type == typeof(Face) || type == typeof(Shell) ? 2
        : type == typeof(Solid) ? 3
        : null;

    private static double VolumeOf(Shape s)
    {
        if (TopoUtil.IsNothing(s))
            return 0.0;
        return s.Type switch
        {
            ShapeType.Vertex or ShapeType.Edge or ShapeType.Wire or ShapeType.Face => 0.0,
            ShapeType.Shell => ((Shell)s).VolumeOfShell,
            ShapeType.Solid => ((Solid)s).ComputeVolume(),
            _ => CompoundExtensions.VolumeOf(s),
        };
    }

    private static Vector CenterOfShape(Shape s)
    {
        s.NeedsGeometry("center");
        return s.Type switch
        {
            ShapeType.Vertex => ((Vertex)s).CenterOfVertex(),
            ShapeType.Edge => ((Edge)s).CenterOfShape1D(CenterOf.Geometry),
            ShapeType.Wire => ((Wire)s).CenterOfShape1D(CenterOf.Geometry),
            ShapeType.Face => ((Face)s).CenterOfFace(CenterOf.Geometry),
            ShapeType.Shell => ((Shell)s).CenterOfShell(),
            ShapeType.Solid => ((Solid)s).CenterOfShape3D(CenterOf.Mass),
            _ => CompoundExtensions.CenterOf(s, CenterOf.Mass),
        };
    }

    private static Vector CenterOfShape(Shape s, CenterOf centerOf)
    {
        if (TopoUtil.IsNothing(s))
            s.NeedsGeometry("center");
        return s.Type switch
        {
            ShapeType.Edge => ((Edge)s).CenterOfShape1D(centerOf),
            ShapeType.Wire => ((Wire)s).CenterOfShape1D(centerOf),
            ShapeType.Face => ((Face)s).CenterOfFace(centerOf),
            ShapeType.Solid => ((Solid)s).CenterOfShape3D(centerOf),
            ShapeType.Compound or ShapeType.CompSolid => CompoundExtensions.CenterOf(s, centerOf),
            _ => throw new NotSupportedException($"{s.Type}.Center doesn't take a CenterOf"),
        };
    }

    private static ShapeList<Shape> TouchOf(Shape s, Shape other, double tolerance)
    {
        if (s.IsNull)
            return [];
        return s.Type switch
        {
            ShapeType.Face => ((Face)s).TouchOfShape2D(other, tolerance),
            ShapeType.Shell => ((Shell)s).TouchOfShape2D(other, tolerance),
            ShapeType.Solid => ((Solid)s).TouchOfSolid(other, tolerance),
            ShapeType.Compound or ShapeType.CompSolid => CompoundExtensions.Touch(s, other, tolerance),
            _ => [],
        };
    }

    /// <summary>
    /// The intersection of a shape with one shape or geometry object, or null if there is none. A shape as the other is
    /// boxed as its own type, as each kind's code matches on it.
    /// </summary>
    internal static ShapeList<Shape>? IntersectOne(Shape s, object other, double tolerance, bool includeTouched)
    {
        if (TopoUtil.IsNothing(s))
            return null;
        other = other is IShape held ? TopoUtil.Typed(held.AsShape()) : other;
        return s.Type switch
        {
            ShapeType.Vertex => ((Vertex)s).IntersectCoreOfVertex(other, tolerance, includeTouched),
            ShapeType.Edge => ((Edge)s).IntersectCoreOfShape1D(other, tolerance, includeTouched),
            ShapeType.Wire => ((Wire)s).IntersectCoreOfShape1D(other, tolerance, includeTouched),
            ShapeType.Face => ((Face)s).IntersectCoreOfShape2D(other, tolerance, includeTouched),
            ShapeType.Shell => ((Shell)s).IntersectCoreOfShape2D(other, tolerance, includeTouched),
            ShapeType.Solid => ((Solid)s).IntersectCoreOfShape3D(other, tolerance, includeTouched),
            _ => CompoundExtensions.IntersectCore(s, other, tolerance, includeTouched),
        };
    }
}

public static partial class Shape1DExtensions
{
    extension<TSelf>(TSelf self) where TSelf : IShape1D
    {
        /// <summary>The kernel's curve: an edge's <see cref="EdgeCurve"/>, or a wire's <see cref="WireCurve"/>.</summary>
        internal ShapeCurve Curve() => Dispatch(self, e => e.CurveOfEdge(), w => w.CurveOfWire());

        /// <summary>The edge (as its curve) and the kernel's curve parameter at a position, and whether that edge runs forward.</summary>
        internal (EdgeCurve Curve, double Param, bool Forward) OcctParamAt(double position, PositionMode mode = PositionMode.Parameter) =>
            Dispatch(self, e => e.OcctParamAtOfEdge(position, mode), w => w.OcctParamAtOfWire(position, mode));

        /// <summary>The kernel's curve parameter at a normalised arc-length position (0 at the start, 1 at the end).</summary>
        public double ParamAt(double position) => Dispatch(self, e => e.ParamAtOfEdge(position), w => w.ParamAtOfWire(position));

        /// <summary>
        /// The parameter of the point on this curve closest to <paramref name="point"/>: the arc-length fraction
        /// <see cref="PositionAt"/> uses, or with <paramref name="normalize"/> false the kernel's own parameter.
        /// </summary>
        public double ParamAtPoint(Vector point, bool normalize = true) =>
            Dispatch(self, e => e.ParamAtPointOfEdge(point, normalize), w => w.ParamAtPointOfWire(point, normalize));
    }

    /// <summary>What to do with an edge or a wire; a curve object of several pieces goes as a wire of them.</summary>
    internal static T Dispatch<TSelf, T>(TSelf self, Func<Edge, T> edge, Func<Wire, T> wire) where TSelf : IShape1D
    {
        var shape = self.AsShape();
        if (TopoUtil.IsNothing(shape))
            return self is Wire ? wire(default) : edge(default);
        return shape.Type switch
        {
            ShapeType.Edge => edge((Edge)shape),
            ShapeType.Wire => wire((Wire)shape),
            _ => wire(Wire.Make(shape.SelectEdges())),
        };
    }
}
