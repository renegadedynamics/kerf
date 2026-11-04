
namespace Kerf.Modeling;

public abstract partial class Shape
{
    /// <summary>The distinct OCCT sub-shapes of a type.</summary>
    public IReadOnlyList<global::Kerf.Topology.Shape> Entities(ShapeType type) =>
        IsEmpty ? [] : TopoUtil.Entities(_wrapped, TopoUtil.ToTopAbs(type));

    /// <summary>The sub-shapes of a type, each noting that it was taken out of this shape.</summary>
    internal ShapeList<T> ShapeListOf<T>(ShapeType type) where T : Shape
    {
        if (IsEmpty)
            return [];
        return new ShapeList<T>(TopoUtil.Entities(_wrapped, TopoUtil.ToTopAbs(type))
                                    .Select(e => (T)CastLike(e).ExtractedFrom(this)));
    }

    /// <summary>The one sub-shape of a type.</summary>
    /// <exception cref="InvalidOperationException">There isn't exactly one.</exception>
    internal T SingleShapeOf<T>(ShapeType type) where T : Shape
    {
        var list = ShapeListOf<T>(type);
        if (list.Count != 1)
            throw new InvalidOperationException($"Expected exactly one {type.ToString().ToLowerInvariant()}, found {list.Count}");
        return list[0];
    }

    /// <summary>All the vertices in this shape, or those <paramref name="select"/> picks.</summary>
    public virtual ShapeList<Vertex> Vertices(Select select = Select.All) => SelectFrom(ShapeListOf<Vertex>(ShapeType.Vertex), select);

    /// <summary>All the non-degenerate edges in this shape, or those <paramref name="select"/> picks.</summary>
    public virtual ShapeList<Edge> Edges(Select select = Select.All) =>
        SelectFrom(ShapeListOf<Edge>(ShapeType.Edge).FilterBy(e => e.TopoEdge.IsDegenerated, reverse: true), select);

    /// <summary>All the wires in this shape, or those <paramref name="select"/> picks.</summary>
    public virtual ShapeList<Wire> Wires(Select select = Select.All) => SelectFrom(ShapeListOf<Wire>(ShapeType.Wire), select);

    /// <summary>All the faces in this shape, or those <paramref name="select"/> picks.</summary>
    public virtual ShapeList<Face> Faces(Select select = Select.All) => SelectFrom(ShapeListOf<Face>(ShapeType.Face), select);

    /// <summary>All the shells in this shape, or those <paramref name="select"/> picks.</summary>
    public virtual ShapeList<Shell> Shells(Select select = Select.All) => SelectFrom(ShapeListOf<Shell>(ShapeType.Shell), select);

    /// <summary>All the solids in this shape, or those <paramref name="select"/> picks.</summary>
    public virtual ShapeList<Solid> Solids(Select select = Select.All) => SelectFrom(ShapeListOf<Solid>(ShapeType.Solid), select);

    /// <summary>All the compounds in this shape (none, but for a compound).</summary>
    public virtual ShapeList<Compound> Compounds() => [];

    /// <summary>The one vertex in this shape (build123d's <c>vertex()</c>).</summary>
    /// <exception cref="InvalidOperationException">The shape doesn't have exactly one.</exception>
    public Vertex SingleVertex() => SingleShapeOf<Vertex>(ShapeType.Vertex);

    /// <summary>The one edge in this shape (build123d's <c>edge()</c>).</summary>
    public Edge SingleEdge() => SingleShapeOf<Edge>(ShapeType.Edge);

    /// <summary>The one wire in this shape (build123d's <c>wire()</c>).</summary>
    public Wire SingleWire() => SingleShapeOf<Wire>(ShapeType.Wire);

    /// <summary>The one face in this shape (build123d's <c>face()</c>).</summary>
    public Face SingleFace() => SingleShapeOf<Face>(ShapeType.Face);

    /// <summary>The one shell in this shape (build123d's <c>shell()</c>).</summary>
    public Shell SingleShell() => SingleShapeOf<Shell>(ShapeType.Shell);

    /// <summary>The one solid in this shape (build123d's <c>solid()</c>).</summary>
    public Solid SingleSolid() => SingleShapeOf<Solid>(ShapeType.Solid);

    /// <summary>The one compound in this shape (build123d's <c>compound()</c>).</summary>
    public Compound SingleCompound() => SingleShapeOf<Compound>(ShapeType.Compound);

    /// <summary>
    /// The non-compound shapes at this shape's first level: a compound's children, looking through nested compounds;
    /// any other shape is its own first level.
    /// </summary>
    public ShapeList<Shape> GetTopLevelShapes() =>
        IsEmpty ? [] : new ShapeList<Shape>(TopoUtil.TopLevelShapes(_wrapped).Select(CastLike));

    /// <summary>Narrows a list of this shape's sub-shapes by what the last operation did.</summary>
    private protected ShapeList<T> SelectFrom<T>(ShapeList<T> shapes, Select select) where T : Shape
    {
        if (select == Select.All)
            return shapes;
        var record = History ?? throw new InvalidOperationException(
            $"Select.{select} needs a record of how this shape was made, and this one has none - "
            + "it is the result of no recorded operation");
        // The record names the sub-shapes where the shape was when it was recorded. If the shape has been
        // relocated since, map each sub-shape back there to look it up.
        var here = _wrapped.Placement;
        Func<Shape, global::Kerf.Topology.Shape> key;
        if (_historyAt is not { } at || TopoUtil.IsSamePlacement(at, here))
            key = s => s.TopoShape;
        else
        {
            var back = at.Multiplied(here.Inverted());
            key = s => s.TopoShape.Moved(back);
        }
        return select switch
        {
            Select.Last => new ShapeList<T>(shapes.Where(s => record.IsLast(key(s)))),
            Select.New => new ShapeList<T>(shapes.Where(s => record.IsNew(key(s)))),
            _ => throw new ArgumentOutOfRangeException(nameof(select)),
        };
    }

    /// <summary>
    /// How the shape this was selected from sits around it: defined for a Vertex, Edge or Face relative to the shape it
    /// was taken out of (see <see cref="Kerf.Modeling.Convexity"/>).
    /// </summary>
    /// <exception cref="InvalidOperationException">The shape has no convexity.</exception>
    public virtual Convexity Convexity => throw new InvalidOperationException(
        $"a {GetType().Name} has no convexity - only a Vertex, Edge or Face selected from a larger shape does");
}
