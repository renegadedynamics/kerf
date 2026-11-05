namespace Kerf.Modeling;

/// <summary>
/// build123d's conveniences on the kernel's shapes: extension members on <see cref="Shape"/>, <see cref="Vertex"/>,
/// <see cref="Edge"/>, <see cref="Wire"/>, <see cref="Face"/>, <see cref="Shell"/>, <see cref="Solid"/> and
/// <see cref="Compound"/>, and on the objects that hold one (<see cref="ModelShape"/>).
/// </summary>
/// <remarks>
/// <para>
/// A shape is immutable, and every operation returns a new one. A <see cref="ModelShape"/> keeps its class:
/// <c>box.Moved(loc)</c> is a <c>Box</c>, and a typed shape keeps its type: <c>edge.Moved(loc)</c> is an
/// <see cref="Edge"/>.
/// </para>
/// <para>
/// The zero of a type is its default value, <c>default(Solid)</c>: it <see cref="get_IsEmpty">is empty</see>, has no
/// sub-shapes, and adding it to another shape leaves that shape unchanged. An empty compound is empty too.
/// </para>
/// </remarks>
public static partial class ShapeExtensions
{
    extension<TSelf>(TSelf self) where TSelf : IShape
    {
        /// <summary>True for the zero of a shape type: a shape with nothing in it.</summary>
        public bool IsEmpty => TopoUtil.IsNothing(self.AsShape());

        /// <summary>Raises the one error for asking an empty shape for a place or direction, which it hasn't got.</summary>
        internal void NeedsGeometry(string askedFor)
        {
            if (self.IsEmpty)
                throw new InvalidOperationException($"An empty {typeof(TSelf).Name} has no {askedFor}");
        }

        /// <summary>The dimension of the shape (0 to 3), or null for a compound of mixed or no dimension.</summary>
        internal int? Dim => self is ModelShape model ? model.Dimension : TopoUtil.Dim(self.AsShape());

        /// <summary>The rank of the type in a boolean's result: Vertex 0 … Solid 3, Compound 4.</summary>
        internal double Order => OrderOf(self.AsShape());

        /// <summary>Whether two shapes are the same: the same data at the same place, either orientation. Two empty shapes are the same nothing.</summary>
        public bool SameAs<TOther>(TOther other) where TOther : IShape
        {
            Shape a = self.AsShape(), b = other.AsShape();
            bool emptyA = TopoUtil.IsNothing(a), emptyB = TopoUtil.IsNothing(b);
            return emptyA || emptyB ? emptyA && emptyB : a.IsSame(b);
        }

        /// <summary>The kind of shape and its place, for debugging and messages: <c>Solid at Location(…)</c>, and for a vertex <c>Vertex(1, 2, 3)</c>.</summary>
        public string Describe() => ShapeExtensions.DescribeOf(self.AsShape(), typeof(TSelf));
    }

    extension<TSelf>(TSelf self) where TSelf : struct, IShape<TSelf>
    {
        /// <summary>The kind of shape. An empty shape reports its own type's kind.</summary>
        public ShapeType ShapeType => self.AsShape().IsNull || TopoUtil.IsNothing(self.AsShape()) ? TSelf.Type : self.AsShape().Type;
    }

    /// <summary>The rank of a shape's type in a boolean's result.</summary>
    internal static double OrderOf(Shape shape)
    {
        if (shape.IsNull)
            return 4.0;
        return shape.Type switch
        {
            ShapeType.Vertex => 0.0,
            ShapeType.Edge => 1.0,
            ShapeType.Wire => 1.5,
            ShapeType.Face => 2.0,
            ShapeType.Shell => 2.5,
            ShapeType.Solid => 3.0,
            _ => 4.0,
        };
    }

    internal static string DescribeOf(Shape shape, Type type)
    {
        var name = shape.IsNull || TopoUtil.IsNothing(shape) ? (type == typeof(Shape) ? "Shape" : type.Name) : shape.Type.ToString();
        if (TopoUtil.IsNothing(shape))
            return $"{name}()";
        if (shape.TryAs(out Vertex vertex))
            return vertex.VertexText();
        return $"{name} at {new Location(shape.Placement)}";
    }

    /// <summary>The shape equivalent of a geometry object: a Vertex for a Vector or Location, an Edge for an Axis, a Face for a Plane.</summary>
    /// <exception cref="ArgumentException">The object has no shape equivalent.</exception>
    public static Shape ShapeOf(object obj) => obj switch
    {
        IShape s => s.AsShape(),
        Vector v => Vertex.Make(v),
        Location l => Vertex.Make(l.Position),
        Axis a => Edge.FromAxis(a),
        Plane p => Face.FromPlane(p),
        _ => throw new ArgumentException($"{obj.GetType().Name} has no shape equivalent"),
    };

    /// <summary>A compound of the shapes, keeping the record they share.</summary>
    internal static Shape MakeComposite(IEnumerable<Shape> shapes)
    {
        var list = shapes.ToList();
        var composite = TopoUtil.MakeCompound(list.Where(s => !TopoUtil.IsNothing(s)));
        // Pieces of one operation share its record; wrapping them keeps it
        var records = list.Select(s => s.History).OfType<ModelHistory>().Distinct().ToList();
        return records.Count == 1 ? ((Shape)composite).MadeBy(records[0]) : composite;
    }

    extension(Shape self)
    {
        /// <summary>
        /// The record of the operation that made this shape, while a <see cref="HistoryLog"/> is open; a struct holds none
        /// of its own. A <see cref="ModelShape"/> keeps it for good.
        /// </summary>
        internal ModelHistory? History => HistoryLog.Current?.Find(self);

        /// <summary>This shape, noting in the open <see cref="HistoryLog"/> how it was made.</summary>
        internal Shape MadeBy(ModelHistory? history)
        {
            HistoryLog.Current?.Note(self, history);
            return self;
        }
    }
}

/// <summary>
/// While open, the records of the operations run on this async flow, by result: how a <see cref="ModelShape"/>'s operation
/// reads back what the kernel's shapes can't carry. Records cost the same with no log open; they just aren't kept.
/// </summary>
internal sealed class HistoryLog : IDisposable
{
    private static readonly AsyncLocal<HistoryLog?> Scope = new();
    private readonly HistoryLog? _outer;
    private readonly Dictionary<Shape, ModelHistory> _records = new(ShapeComparer.Same);

    private HistoryLog()
    {
        _outer = Scope.Value;
        Scope.Value = this;
    }

    /// <summary>The open log, if any.</summary>
    internal static HistoryLog? Current => Scope.Value;

    /// <summary>Opens a log, closed when disposed.</summary>
    internal static HistoryLog Open() => new();

    internal void Note(Shape shape, ModelHistory? history)
    {
        if (shape.IsNull)
            return;
        if (history is null)
            _records.Remove(shape);
        else
            _records[shape] = history;
    }

    internal ModelHistory? Find(Shape shape) =>
        !shape.IsNull && _records.TryGetValue(shape, out var record) ? record : null;

    /// <summary>Closes the log.</summary>
    public void Dispose() => Scope.Value = _outer;
}
