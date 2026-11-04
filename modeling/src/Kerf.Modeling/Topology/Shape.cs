using Kerf.Topology;

namespace Kerf.Modeling;

/// <summary>
/// Base class of every CAD object: <see cref="Vertex"/>, <see cref="Edge"/>, <see cref="Wire"/>, <see cref="Face"/>,
/// <see cref="Shell"/>, <see cref="Solid"/> and <see cref="Compound"/>.
/// </summary>
/// <remarks>
/// <para>
/// A shape is immutable. It wraps an OCCT <c>TopoDS_Shape</c> that it never modifies, and every operation returns
/// a new shape. Operations that keep the shape's class (<c>Moved</c>, <c>Located</c>, <c>Rotate</c>, <c>Clean</c>, …)
/// are extension methods, so <c>box.Moved(loc)</c> is still a <c>Box</c>.
/// </para>
/// <para>
/// A shape made from nothing is the zero of its class: it holds the kernel's empty compound, is
/// <see cref="IsEmpty"/>, has no sub-shapes, and adding it to another shape leaves that shape unchanged.
/// </para>
/// </remarks>
// Properties and methods are found by name for selectors given plain values (ShapeProperty), trimmed or not
[System.Diagnostics.CodeAnalysis.DynamicallyAccessedMembers(
    System.Diagnostics.CodeAnalysis.DynamicallyAccessedMemberTypes.PublicProperties
    | System.Diagnostics.CodeAnalysis.DynamicallyAccessedMemberTypes.PublicMethods)]
public abstract partial class Shape : IEquatable<Shape>
{
    // The OCCT shape. Never modified: copies are made with Located/Moved/Reversed instead
    private global::Kerf.Topology.Shape _wrapped;
    private bool? _isEmpty;
    // The root Location the history describes: the shape's Location when it was recorded. A relocated copy shares
    // the record, so selection maps its sub-shapes back by the difference between this and its own.
    private Placement? _historyAt;

    private protected Shape(global::Kerf.Topology.Shape? obj, string label = "", Color? color = null)
    {
        _wrapped = obj is { } shape ? TopoUtil.Downcast(shape) : TopoUtil.EmptyCompound();
        Label = label;
        Color = color;
        if (ShapeLog.Folder is not null)
            ShapeLog.Record(this, _wrapped);
    }

    // ---- Metadata ----

    /// <summary>The user-assigned label.</summary>
    public string Label { get; private set; }

    /// <summary>The shape's colour, if it has one.</summary>
    public Color? Color { get; private set; }

    /// <summary>Whether the shape is construction geometry, used to place others rather than as part of the result.</summary>
    public bool ForConstruction { get; private set; }

    /// <summary>
    /// The shapes this one was extracted through, outermost first: an edge taken from a face of a solid records
    /// the solid, then the face.
    /// </summary>
    public IReadOnlyList<Shape> TopoPath { get; private set; } = [];

    /// <summary>The shape this one was ultimately taken out of: the outermost step of <see cref="TopoPath"/>.</summary>
    public Shape? TopoParent => TopoPath.Count > 0 ? TopoPath[0] : null;

    /// <summary>
    /// The shape this one was taken directly out of: the innermost step of <see cref="TopoPath"/>. An edge picked off
    /// a face knows that face, which says which side of the edge the face is on.
    /// </summary>
    public Shape? TopoOwner => TopoPath.Count > 0 ? TopoPath[^1] : null;

    /// <summary>
    /// What the operation that made this shape did to its inputs' sub-shapes, when the kernel reported it
    /// (booleans, clean, fillets and chamfers). Used by <see cref="Select.Last"/> and <see cref="Select.New"/>.
    /// </summary>
    public ShapeHistory? History { get; private set; }

    // ---- The OCCT shape ----

    /// <summary>The OCCT shape, for use inside the library. Never modify it.</summary>
    internal global::Kerf.Topology.Shape TopoShape => _wrapped;

    /// <summary>
    /// The OCCT shape (a <c>TopoDS_Edge</c> for an edge, and so on): a new handle sharing this shape's geometry,
    /// which may be modified without affecting this shape.
    /// </summary>
    public global::Kerf.Topology.Shape Wrapped => TopoUtil.Downcast(_wrapped);

    /// <summary>True for the zero of a shape class: a shape with nothing in it.</summary>
    public bool IsEmpty => _isEmpty ??= TopoUtil.IsEmptyCompound(_wrapped);

    /// <summary>Raises the one error for asking an empty shape for a place or direction, which it hasn't got.</summary>
    private protected void NeedsGeometry(string askedFor)
    {
        if (IsEmpty)
            throw new InvalidOperationException($"An empty {GetType().Name} has no {askedFor}");
    }

    // ---- Class properties ----

    /// <summary>The dimension of the shape (0 to 3), or null for a compound of mixed or no dimension.</summary>
    internal abstract int? Dim { get; }

    /// <summary>The rank of the class in a boolean's result: Vertex 0 … Solid 3, Compound 4.</summary>
    internal abstract double Order { get; }

    /// <summary>The zero of this class: nothing of a Box is a Part.</summary>
    internal abstract Shape MakeZero();

    /// <summary>The zero of the class an operation on these shapes would return: the highest-ranked one.</summary>
    internal static Shape ZeroOf(params IEnumerable<Shape> shapes) =>
        shapes.Select(s => s.MakeZero()).Aggregate((best, z) => z.Order > best.Order ? z : best);

    // ---- Copies ----

    private protected Shape Clone()
    {
        var copy = (Shape)MemberwiseClone();
        copy._isEmpty = null;
        return copy;
    }

    /// <summary>
    /// A copy of this shape around an already placed kernel shape: the same class and attributes, holding
    /// <paramref name="wrapped"/> rather than a copy of the geometry. The history is kept, as a relocated copy shares it.
    /// </summary>
    internal Shape Placed(global::Kerf.Topology.Shape wrapped)
    {
        var copy = Clone();
        copy._wrapped = wrapped;
        return copy;
    }

    /// <summary>A copy of this shape around new geometry: the same class and attributes, without a history.</summary>
    internal Shape Rewrapped(global::Kerf.Topology.Shape wrapped)
    {
        var copy = Clone();
        copy._wrapped = TopoUtil.Downcast(wrapped);
        copy.History = null;
        copy._historyAt = null;
        if (ShapeLog.Folder is not null)
            ShapeLog.Record(copy, copy._wrapped);
        return Result(copy);
    }

    /// <summary>A copy of this shape noting the record of how it was made.</summary>
    internal Shape MadeBy(ShapeHistory? history)
    {
        var copy = Clone();
        copy.History = history;
        copy._historyAt = copy._wrapped.Placement;
        return Result(copy);
    }

    /// <summary>
    /// A new result. The native backend told the GC here about the native memory a result held; OCCT.NET's memory is
    /// managed, so nothing is added (unit 10B).
    /// </summary>
    internal static T Result<T>(T shape) where T : Shape => shape;

    /// <summary>A copy of this shape noting that it was taken out of <paramref name="source"/>.</summary>
    internal Shape ExtractedFrom(Shape source)
    {
        var copy = Clone();
        copy.TopoPath = [.. source.TopoPath, source];
        return copy;
    }

    internal Shape WithTopoPath(IReadOnlyList<Shape> path)
    {
        var copy = Clone();
        copy.TopoPath = path;
        return copy;
    }

    internal Shape WithLabelCore(string label)
    {
        var copy = Clone();
        copy.Label = label;
        return copy;
    }

    internal Shape WithColorCore(Color? color)
    {
        var copy = Clone();
        copy.Color = color;
        return copy;
    }

    internal Shape WithForConstructionCore(bool forConstruction)
    {
        var copy = Clone();
        copy.ForConstruction = forConstruction;
        return copy;
    }

    /// <summary>
    /// <paramref name="target"/> with this shape's attributes (label, colour, construction flag and provenance) where
    /// its own are unset. The history isn't copied: it says how this shape was made, not the target.
    /// </summary>
    internal T CopyAttributesTo<T>(T target) where T : Shape
    {
        var copy = (T)target.Clone();
        if (copy.Label.Length == 0) copy.Label = Label;
        copy.Color ??= Color;
        copy.ForConstruction |= ForConstruction;
        if (copy.TopoPath.Count == 0) copy.TopoPath = TopoPath;
        CopySubclassAttributesTo(copy);
        return copy;
    }

    /// <summary>Copies attributes a subclass adds onto a target of the same class, where the target's are unset.</summary>
    private protected virtual void CopySubclassAttributesTo(Shape target) { }

    /// <summary>
    /// The record, brought to where the shape is now. A relocated shape shares the record of the shape it was
    /// placed from, which names the sub-shapes where that shape was; an operation that extends the record
    /// (<c>Clean</c>) needs a copy chained forward to here.
    /// </summary>
    internal ShapeHistory? HistoryHere()
    {
        if (History is null)
            return null;
        var here = _wrapped.Placement;
        if (_historyAt is not { } at || TopoUtil.IsSamePlacement(at, here))
            return History;
        var recorded = _wrapped.Located(at);
        return new ShapeHistory(History.Before, History.Brought)
            .Merge(History)
            .Merge(ShapeHistory.FromRelocation(recorded, _wrapped));
    }

    // ---- Casting ----

    /// <summary>The wrapper of the right class for an OCCT shape.</summary>
    public static Shape Cast(global::Kerf.Topology.Shape obj) => TopoUtil.ShapeTypeOf(obj) switch
    {
        global::Kerf.Topology.ShapeType.Vertex => new Vertex((global::Kerf.Topology.Vertex)obj),
        global::Kerf.Topology.ShapeType.Edge => new Edge((global::Kerf.Topology.Edge)obj),
        global::Kerf.Topology.ShapeType.Wire => new Wire((global::Kerf.Topology.Wire)obj),
        global::Kerf.Topology.ShapeType.Face => new Face((global::Kerf.Topology.Face)obj),
        global::Kerf.Topology.ShapeType.Shell => new Shell((global::Kerf.Topology.Shell)obj),
        global::Kerf.Topology.ShapeType.Solid => new Solid((global::Kerf.Topology.Solid)obj),
        global::Kerf.Topology.ShapeType.Compound or global::Kerf.Topology.ShapeType.CompSolid => new Compound(obj),
        var t => throw new ArgumentException($"Unable to cast {t}"),
    };

    /// <summary>
    /// <see cref="Cast(Kerf.Topology.Shape)"/> on behalf of this shape's class: the kernel's empty compound has no type of
    /// its own, so it becomes the zero of this class.
    /// </summary>
    internal Shape CastLike(global::Kerf.Topology.Shape obj) =>
        TopoUtil.IsEmptyCompound(obj) ? MakeZero() : Cast(obj);

    /// <summary>The shape equivalent of a geometry object: a Vertex for a Vector or Location, an Edge for an Axis, a Face for a Plane.</summary>
    public static Shape AsShape(object obj) => obj switch
    {
        Shape s => s,
        Vector v => new Vertex(v),
        Location l => new Vertex(l.Position),
        Axis a => Edge.FromAxis(a),
        Plane p => Face.FromPlane(p),
        _ => throw new ArgumentException($"{obj.GetType().Name} has no shape equivalent"),
    };

    /// <summary>The registered composite for a dimension: Curve, Sketch or Part, or a plain Compound.</summary>
    internal static Shape MakeComposite(IEnumerable<Shape> shapes, double? dimension = null)
    {
        var list = shapes.ToList();
        if (dimension is null && list.Count > 0)
            dimension = list.Select(s => s.Dim).Distinct().ToList() is [{ } d] ? d : null;
        Shape composite = dimension switch
        {
            1 => new Curve(list),
            2 => new Sketch(list),
            3 => new Part(list),
            _ => new Compound(list),
        };
        // Pieces of one operation share its record; wrapping them keeps it
        var records = list.Select(s => s.History).OfType<ShapeHistory>().Distinct().ToList();
        return records.Count == 1 ? composite.MadeBy(records[0]) : composite;
    }

    // ---- Equality ----

    /// <summary>
    /// Whether two shapes are the same: they share the same TShape at the same Location (orientations may differ).
    /// Empty shapes of one class are all the same nothing.
    /// </summary>
    public bool Equals(Shape? other)
    {
        if (other is null)
            return false;
        if (IsEmpty || other.IsEmpty)
            return IsEmpty && other.IsEmpty && GetType() == other.GetType();
        return _wrapped.IsSame(other._wrapped);
    }

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Shape s && Equals(s);

    /// <summary>A hash consistent with <see cref="Equals(Shape?)"/>.</summary>
    public override int GetHashCode() => IsEmpty ? GetType().GetHashCode() : _wrapped.StdHash().GetHashCode();

    /// <summary>Whether two shapes are the same (see <see cref="Equals(Shape?)"/>).</summary>
    public static bool operator ==(Shape? a, Shape? b) => a is null ? b is null : a.Equals(b);

    /// <summary>Whether two shapes aren't the same.</summary>
    public static bool operator !=(Shape? a, Shape? b) => !(a == b);

    /// <summary>True if the shapes share the same TShape with the same Location and orientation.</summary>
    public bool IsEqual(Shape other) => !IsEmpty && !other.IsEmpty && _wrapped.IsEqual(other._wrapped);

    /// <summary>True if the shapes share the same TShape with the same Location; orientations may differ.</summary>
    public bool IsSame(Shape other) => !IsEmpty && !other.IsEmpty && _wrapped.IsSame(other._wrapped);

    /// <summary>The class name, label and location, for debugging.</summary>
    public override string ToString() =>
        IsEmpty ? $"{GetType().Name}()" : $"{GetType().Name} at {Location}" + (Label.Length > 0 ? $", label({Label})" : "");
}
