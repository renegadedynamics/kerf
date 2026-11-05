using Kerf.Booleans;

namespace Kerf.Modeling;

// The algebra of parts, sketches and curves (build123d's algebra mode): each combines with shapes of its own dimension
// and gives a value of its own class, so a fuse of parts is a Part and a cut of sketches a Sketch. The operators read
// as build123d's do: + fuses, - cuts, & keeps what is common. Operands are flattened into their top-level shapes, and
// an empty one adds or removes nothing.
//
// Add, Subtract and Common return a Result: a Boolean that fails is a failure a caller should expect. The operators are
// the shorthand for code whose inputs are known to combine (unit 10M3): an operator can't return a Result and still
// chain, so it returns the value, and a failure throws InvalidOperationException with the Boolean's message.

public partial class Part
{
    private Part Run(Func<Shape> operation) => Finish<Part>(operation, r => new Part(r), () => new Part());

    /// <summary>
    /// This part fused with <paramref name="others"/>, solids or parts, within <see cref="Constants.Tolerance"/>.
    /// Adding nothing returns this part.
    /// </summary>
    /// <exception cref="ArgumentException">An operand isn't three-dimensional.</exception>
    public Result<Part> Add(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => Run(() => ShapeAlgebra.AddToCompound(Shape, 3, operands)));
    }

    /// <summary>This part with <paramref name="others"/>, solids or parts, cut away.</summary>
    /// <exception cref="ArgumentException">An operand isn't three-dimensional.</exception>
    public Result<Part> Subtract(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => Run(() => ShapeAlgebra.SubtractFromCompound(Shape, 3, operands)));
    }

    /// <summary>What this part has in common with <paramref name="others"/>: empty if nothing.</summary>
    public Result<Part> Common(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => Run(() => ShapeAlgebra.CommonOfCompound(Shape, operands)));
    }

    /// <inheritdoc cref="Add(IEnumerable{Shape})"/>
    public Result<Part> Add<T>(IEnumerable<T> others) where T : IShape => Add(others.Select(o => o.AsShape()));

    /// <inheritdoc cref="Subtract(IEnumerable{Shape})"/>
    public Result<Part> Subtract<T>(IEnumerable<T> others) where T : IShape => Subtract(others.Select(o => o.AsShape()));

    /// <inheritdoc cref="Common(IEnumerable{Shape})"/>
    public Result<Part> Common<T>(IEnumerable<T> others) where T : IShape => Common(others.Select(o => o.AsShape()));

    /// <summary>The part fused with a shape (see <see cref="Add"/>).</summary>
    public static Part operator +(Part a, Shape b) => a.Run(() => ShapeAlgebra.AddToCompound(a.Shape, 3, [b]));

    /// <summary>The part fused with shapes (see <see cref="Add"/>).</summary>
    public static Part operator +(Part a, IEnumerable<Shape> b) { var c = Checked(b); return a.Run(() => ShapeAlgebra.AddToCompound(a.Shape, 3, c)); }

    /// <summary>The part fused with objects (see <see cref="Add"/>).</summary>
    public static Part operator +(Part a, IEnumerable<ModelShape> b) => a + b.Select(o => o.Shape);

    /// <summary>The part with a shape cut away (see <see cref="Subtract"/>).</summary>
    public static Part operator -(Part a, Shape b) => a.Run(() => ShapeAlgebra.SubtractFromCompound(a.Shape, 3, [b]));

    /// <summary>The part with shapes cut away (see <see cref="Subtract"/>).</summary>
    public static Part operator -(Part a, IEnumerable<Shape> b) { var c = Checked(b); return a.Run(() => ShapeAlgebra.SubtractFromCompound(a.Shape, 3, c)); }

    /// <summary>The part with objects cut away (see <see cref="Subtract"/>).</summary>
    public static Part operator -(Part a, IEnumerable<ModelShape> b) => a - b.Select(o => o.Shape);

    /// <summary>What the part has in common with a shape (see <see cref="Common"/>).</summary>
    public static Part operator &(Part a, Shape b) => a.Run(() => ShapeAlgebra.CommonOfCompound(a.Shape, [b]));

    /// <summary>What the part has in common with shapes (see <see cref="Common"/>).</summary>
    public static Part operator &(Part a, IEnumerable<Shape> b) { var c = Checked(b); return a.Run(() => ShapeAlgebra.CommonOfCompound(a.Shape, c)); }
}

public partial class Sketch
{
    private Sketch Run(Func<Shape> operation) => Finish<Sketch>(operation, r => new Sketch(r), () => new Sketch());

    /// <summary>
    /// This sketch fused with <paramref name="others"/>, faces or sketches, within <see cref="Constants.Tolerance"/>.
    /// Adding nothing returns this sketch.
    /// </summary>
    /// <exception cref="ArgumentException">An operand isn't two-dimensional.</exception>
    public Result<Sketch> Add(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => Run(() => ShapeAlgebra.AddToCompound(Shape, 2, operands)));
    }

    /// <summary>This sketch with <paramref name="others"/> cut away.</summary>
    public Result<Sketch> Subtract(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => Run(() => ShapeAlgebra.SubtractFromCompound(Shape, 2, operands)));
    }

    /// <summary>What this sketch has in common with <paramref name="others"/>: empty if nothing.</summary>
    public Result<Sketch> Common(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => Run(() => ShapeAlgebra.CommonOfCompound(Shape, operands)));
    }

    /// <inheritdoc cref="Add(IEnumerable{Shape})"/>
    public Result<Sketch> Add<T>(IEnumerable<T> others) where T : IShape => Add(others.Select(o => o.AsShape()));

    /// <inheritdoc cref="Subtract(IEnumerable{Shape})"/>
    public Result<Sketch> Subtract<T>(IEnumerable<T> others) where T : IShape => Subtract(others.Select(o => o.AsShape()));

    /// <inheritdoc cref="Common(IEnumerable{Shape})"/>
    public Result<Sketch> Common<T>(IEnumerable<T> others) where T : IShape => Common(others.Select(o => o.AsShape()));

    /// <summary>The sketch fused with a shape (see <see cref="Add"/>).</summary>
    public static Sketch operator +(Sketch a, Shape b) => a.Run(() => ShapeAlgebra.AddToCompound(a.Shape, 2, [b]));

    /// <summary>The sketch fused with shapes (see <see cref="Add"/>).</summary>
    public static Sketch operator +(Sketch a, IEnumerable<Shape> b) { var c = Checked(b); return a.Run(() => ShapeAlgebra.AddToCompound(a.Shape, 2, c)); }

    /// <summary>The sketch fused with objects (see <see cref="Add"/>).</summary>
    public static Sketch operator +(Sketch a, IEnumerable<ModelShape> b) => a + b.Select(o => o.Shape);

    /// <summary>The sketch with a shape cut away (see <see cref="Subtract"/>).</summary>
    public static Sketch operator -(Sketch a, Shape b) => a.Run(() => ShapeAlgebra.SubtractFromCompound(a.Shape, 2, [b]));

    /// <summary>The sketch with shapes cut away (see <see cref="Subtract"/>).</summary>
    public static Sketch operator -(Sketch a, IEnumerable<Shape> b) { var c = Checked(b); return a.Run(() => ShapeAlgebra.SubtractFromCompound(a.Shape, 2, c)); }

    /// <summary>The sketch with objects cut away (see <see cref="Subtract"/>).</summary>
    public static Sketch operator -(Sketch a, IEnumerable<ModelShape> b) => a - b.Select(o => o.Shape);

    /// <summary>What the sketch has in common with a shape (see <see cref="Common"/>).</summary>
    public static Sketch operator &(Sketch a, Shape b) => a.Run(() => ShapeAlgebra.CommonOfCompound(a.Shape, [b]));

    /// <summary>What the sketch has in common with shapes (see <see cref="Common"/>).</summary>
    public static Sketch operator &(Sketch a, IEnumerable<Shape> b) { var c = Checked(b); return a.Run(() => ShapeAlgebra.CommonOfCompound(a.Shape, c)); }
}

public partial class Curve
{
    // A line or polyline holds its edge or wire, and combines into what the kernel gives: one edge or wire, or a compound
    private Curve Run(Func<Shape> operation) => Finish<Curve>(operation, r => new Curve(r), () => new Curve(), this is BaseEdgeObject or BaseLineObject);

    /// <summary>
    /// This curve joined with <paramref name="others"/>, edges, wires or curves: a wire where they connect, otherwise a
    /// compound. Adding nothing returns this curve.
    /// </summary>
    /// <exception cref="ArgumentException">An operand isn't one-dimensional.</exception>
    public Result<Curve> Add(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => Run(() => OneD.Add1D(Shape, operands)));
    }

    /// <summary>This curve with <paramref name="others"/> cut away.</summary>
    public Result<Curve> Subtract(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => Run(() => ShapeAlgebra.SubtractFromCompound(Shape, 1, operands)));
    }

    /// <summary>What this curve has in common with <paramref name="others"/>: empty if nothing.</summary>
    public Result<Curve> Common(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => Run(() => ShapeAlgebra.CommonOfCompound(Shape, operands)));
    }

    /// <inheritdoc cref="Add(IEnumerable{Shape})"/>
    public Result<Curve> Add<T>(IEnumerable<T> others) where T : IShape => Add(others.Select(o => o.AsShape()));

    /// <inheritdoc cref="Subtract(IEnumerable{Shape})"/>
    public Result<Curve> Subtract<T>(IEnumerable<T> others) where T : IShape => Subtract(others.Select(o => o.AsShape()));

    /// <inheritdoc cref="Common(IEnumerable{Shape})"/>
    public Result<Curve> Common<T>(IEnumerable<T> others) where T : IShape => Common(others.Select(o => o.AsShape()));

    /// <summary>The curve joined with a shape (see <see cref="Add"/>).</summary>
    public static Curve operator +(Curve a, Shape b) => a.Run(() => OneD.Add1D(a.Shape, [b]));

    /// <summary>The curve joined with shapes (see <see cref="Add"/>).</summary>
    public static Curve operator +(Curve a, IEnumerable<Shape> b) { var c = Checked(b); return a.Run(() => OneD.Add1D(a.Shape, c)); }

    /// <summary>The curve joined with objects (see <see cref="Add"/>).</summary>
    public static Curve operator +(Curve a, IEnumerable<ModelShape> b) => a + b.Select(o => o.Shape);

    /// <summary>The curve with a shape cut away (see <see cref="Subtract"/>).</summary>
    public static Curve operator -(Curve a, Shape b) => a.Run(() => ShapeAlgebra.SubtractFromCompound(a.Shape, 1, [b]));

    /// <summary>The curve with shapes cut away (see <see cref="Subtract"/>).</summary>
    public static Curve operator -(Curve a, IEnumerable<Shape> b) { var c = Checked(b); return a.Run(() => ShapeAlgebra.SubtractFromCompound(a.Shape, 1, c)); }

    /// <summary>The curve with objects cut away (see <see cref="Subtract"/>).</summary>
    public static Curve operator -(Curve a, IEnumerable<ModelShape> b) => a - b.Select(o => o.Shape);

    /// <summary>What the curve has in common with a shape (see <see cref="Common"/>).</summary>
    public static Curve operator &(Curve a, Shape b) => a.Run(() => ShapeAlgebra.CommonOfCompound(a.Shape, [b]));
}

public abstract partial class ModelShape
{
    /// <summary>The operands of an algebra operation, as a list.</summary>
    private protected static List<Shape> Checked(IEnumerable<Shape> others)
    {
        ArgumentNullException.ThrowIfNull(others);
        return others.ToList();
    }

    /// <summary>
    /// The result of an algebra operation as this object's algebra class: this object itself if nothing was done, nothing
    /// if the result is empty, else that class around the kernel's compound, with this object's attributes and the
    /// operation's record.
    /// </summary>
    private protected T Finish<T>(Func<Shape> operation, Func<Shape, T> make, Func<T> zero, bool holdResult = false) where T : ModelShape
    {
        using var log = HistoryLog.Open();
        var result = operation();
        var record = log.Find(result);
        if (this is T same && record is null && !result.IsNull && result.IsEqual(Shape))
            return same;
        if (TopoUtil.IsNothing(result))
            return CopyAttributesTo(zero());
        var held = holdResult || result.IsCompound() ? result : TopoUtil.MakeCompound([result]);
        return (T)CopyAttributesTo(make(held)).MadeBy(record);
    }
}

// The topological shapes (solids, faces, edges) keep build123d's operators, and their result is the kernel's: one solid
// or a compound of several pieces, a wire or a compound of edges. The type is decided by the result, as build123d's
// is and as the Python-reference tests pin it, so these return Shape; the algebra's classes above are typed.

/// <summary>build123d's operators on the kernel's shapes.</summary>
public static class ShapeOperators
{
    extension<T>(T) where T : struct, IShape<T>
    {
        /// <summary>
        /// This shape fused with another (build123d's <c>+</c>): the kernel's result, of whichever type it is. Only shapes
        /// of the same dimension can be added. A <see cref="Part"/>, <see cref="Sketch"/> or <see cref="Curve"/> has its own
        /// typed operator.
        /// </summary>
        public static Shape operator +(T a, Shape b) => ShapeAlgebra.Add(a.AsShape(), [b], T.Type);

        /// <summary>This shape fused with others.</summary>
        public static Shape operator +(T a, IEnumerable<Shape> b) => ShapeAlgebra.Add(a.AsShape(), b, T.Type);

        /// <summary>This shape fused with others of its type.</summary>
        public static Shape operator +(T a, IEnumerable<T> b) => ShapeAlgebra.Add(a.AsShape(), b.Select(s => s.AsShape()), T.Type);

        /// <summary>This shape with another cut away: only shapes of an equal or greater dimension can be subtracted.</summary>
        public static Shape operator -(T a, Shape b) => ShapeAlgebra.Subtract(a.AsShape(), [b]);

        /// <summary>This shape with others cut away.</summary>
        public static Shape operator -(T a, IEnumerable<Shape> b) => ShapeAlgebra.Subtract(a.AsShape(), b);

        /// <summary>This shape with others of its type cut away.</summary>
        public static Shape operator -(T a, IEnumerable<T> b) => ShapeAlgebra.Subtract(a.AsShape(), b.Select(s => s.AsShape()));

        /// <summary>What this shape has in common with another: one shape, a compound of several, or nothing.</summary>
        public static Shape operator &(T a, Shape b) => ShapeAlgebra.Common(a.AsShape(), [b]);

        /// <summary>What this shape has in common with others.</summary>
        public static Shape operator &(T a, IEnumerable<Shape> b) => ShapeAlgebra.Common(a.AsShape(), b);
    }
}

/// <summary>A vertex's coordinates add and subtract (build123d's <c>Vertex</c> operators).</summary>
public static class VertexOperators
{
    extension(Vertex)
    {
        /// <summary>A vertex at the sum of two vertices' coordinates.</summary>
        public static Vertex operator +(Vertex a, Vertex b) => Vertex.Make(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

        /// <summary>A vertex moved by a vector.</summary>
        public static Vertex operator +(Vertex a, Vector b) => Vertex.Make(a.X + b.X, a.Y + b.Y, a.Z + b.Z);

        /// <summary>A vertex at the difference of two vertices' coordinates.</summary>
        public static Vertex operator -(Vertex a, Vertex b) => Vertex.Make(a.X - b.X, a.Y - b.Y, a.Z - b.Z);

        /// <summary>A vertex moved back by a vector.</summary>
        public static Vertex operator -(Vertex a, Vector b) => Vertex.Make(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
    }
}

/// <summary>The algebra's rules on the kernel's shapes: what <c>+</c>, <c>-</c> and <c>&amp;</c> do with each kind of shape.</summary>
internal static class ShapeAlgebra
{
    /// <summary>Flattens boolean operands into their top-level shapes.</summary>
    internal static List<Shape> Operands(IEnumerable<Shape>? others) =>
        others is null ? [] : others.SelectMany(o => o.GetTopLevelShapes()).ToList();

    internal static Shape Add(Shape self, IEnumerable<Shape> other, ShapeType typeOfZero = ShapeType.Shape)
    {
        if (self.IsNull)
            // The zero of a type adds as that type does
            return typeOfZero switch
            {
                ShapeType.Edge or ShapeType.Wire => OneD.Add1D(self, other),
                ShapeType.Shell => default(Shell).AddCore_Shape2D(other),
                ShapeType.Compound => AddToCompound(self, null, other),
                _ => AddPlain(self, null, other),
            };
        return self.Type switch
        {
            ShapeType.Edge or ShapeType.Wire => OneD.Add1D(self, other),
            ShapeType.Face => ((Face)self).AddCore_Shape2D(other),
            ShapeType.Shell => ((Shell)self).AddCore_Shape2D(other),
            ShapeType.Compound or ShapeType.CompSolid => AddToCompound(self, null, other),
            _ => AddPlain(self, null, other),
        };
    }

    /// <summary>
    /// This shape fused with <paramref name="other"/> (build123d's <c>+</c>). Adding nothing returns this shape;
    /// only shapes of the same dimension can be added.
    /// </summary>
    internal static Shape AddPlain(Shape self, int? dim, IEnumerable<Shape> other)
    {
        var summands = Operands(other);
        if (summands.Count == 0)
            return self;
        var selfEmpty = TopoUtil.IsNothing(self);
        // An empty shape with no dimension of its own adds nothing to anything
        var addendDim = selfEmpty ? summands[0].Dim : dim ?? self.Dim;
        if (addendDim is null)
            throw new ArgumentException("Dimensions of objects to add to are inconsistent");
        if (summands.Any(s => s.Dim != addendDim))
            throw new ArgumentException("Only shapes with the same dimension can be added");
        if (selfEmpty)
            return summands.Count == 1 ? summands[0] : summands[0].Fuse(summands.Skip(1));
        return self.Fuse(summands);
    }

    /// <summary>
    /// This compound combined with others (build123d's <c>+</c>): connected 1D shapes join into a curve (or wire);
    /// otherwise the pieces are fused, within <see cref="Constants.Tolerance"/>, into a compound.
    /// </summary>
    internal static Shape AddToCompound(Shape self, int? dimension, IEnumerable<Shape> other)
    {
        var dim = dimension ?? TopoUtil.Dim(self);
        if (dim == 1)
        {
            var sum1d = OneD.Add1D(self, other);
            return sum1d.IsEdge() ? ShapeExtensions.MakeComposite([sum1d]).MadeBy(ModelHistory.Of([sum1d])) : sum1d;
        }
        var brought = Operands(other);
        if (brought.Count == 0)
            return self;
        var own = self.GetTopLevelShapes().ToList();
        var summands = own.Concat(brought).ToList();
        if (summands.Count <= 1)
            return ShapeExtensions.MakeComposite(summands.Take(1));
        var result = ShapeExtensions.BoolOp(summands.Take(1), summands.Skip(1), BooleanKind.Fuse, new BooleanOptions { FuzzyValue = Constants.Tolerance });
        // The fuse ran this compound's first piece against the rest; for the record, all of this compound was there
        // before, and what was added was brought in
        if (result.History is { } history)
            result = result.MadeBy(history.WithInputs(own, brought));
        return result.IsCompound() ? result : ShapeExtensions.MakeComposite([result]);
    }

    internal static Shape Subtract(Shape self, IEnumerable<Shape> other) =>
        !self.IsNull && self.IsCompound() ? SubtractFromCompound(self, null, other) : SubtractPlain(self, null, other);

    /// <summary>
    /// This shape with <paramref name="other"/> cut away (build123d's <c>-</c>). Only shapes of an equal or greater
    /// dimension can be subtracted.
    /// </summary>
    internal static Shape SubtractPlain(Shape self, int? dim, IEnumerable<Shape> other)
    {
        var list = other.ToList();
        var subtrahends = Operands(list);
        if (subtrahends.Count == 0)
            return self;
        if (TopoUtil.IsNothing(self))   // nothing less anything is still nothing
            return default;
        var minuendDim = dim ?? self.Dim;
        if (minuendDim is null || subtrahends.Any(s => s.Dim is null))
            throw new ArgumentException("Dimensions of objects to subtract from are inconsistent");
        var dims = subtrahends.Select(s => s.Dim!.Value).ToList();
        if (dims.Any(d => d < minuendDim))
            throw new ArgumentException(
                "Only shapes with equal or greater dimension can be subtracted: "
                + $"not {TopoUtil.KindName(self)} ({minuendDim}D) and {TopoUtil.KindName(list[0])} ({dims.Min()}D)");
        return self.Cut(subtrahends);
    }

    /// <summary>This compound with others cut away, always as a compound.</summary>
    internal static Shape SubtractFromCompound(Shape self, int? dim, IEnumerable<Shape> other)
    {
        var difference = SubtractPlain(self, dim ?? self.Dim, other);
        if (TopoUtil.IsNothing(difference) || (!difference.IsNull && difference.IsEqual(self)))
            return difference;
        return difference.IsCompound() ? difference : ShapeExtensions.MakeComposite([difference]);
    }

    internal static Shape Common(Shape self, IEnumerable<Shape> others)
    {
        if (!self.IsNull && self.IsVertex())
            throw new NotSupportedException("Vertices can't be intersected");
        return !self.IsNull && self.IsCompound() ? CommonOfCompound(self, others) : CommonPlain(self, others);
    }

    /// <summary>
    /// What this shape has in common with <paramref name="others"/> (build123d's <c>&amp;</c>): one shape, a compound
    /// of several, or nothing.
    /// </summary>
    internal static Shape CommonPlain(Shape self, IEnumerable<Shape> others)
    {
        var list = others.ToList();
        if (TopoUtil.IsNothing(self) || list.Any(TopoUtil.IsNothing))
            return default;
        var found = self.Intersect(list.Select(o => (object)o));
        if (found.Count == 0)
            return default;
        var result = found.Count == 1 ? found[0] : ShapeExtensions.MakeComposite(found);
        return SkipClean.Clean ? result.Clean() : result;
    }

    /// <summary>What this compound has in common with others, always as a compound.</summary>
    internal static Shape CommonOfCompound(Shape self, IEnumerable<Shape> others)
    {
        var intersection = CommonPlain(self, others);
        if (TopoUtil.IsNothing(intersection))
            return intersection;
        return intersection.IsCompound() ? intersection : ShapeExtensions.MakeComposite([intersection]);
    }
}
