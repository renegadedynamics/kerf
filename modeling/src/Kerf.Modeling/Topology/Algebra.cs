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
    /// <summary>
    /// This part fused with <paramref name="others"/>, solids or parts, within <see cref="Constants.Tolerance"/>.
    /// Adding nothing returns this part.
    /// </summary>
    /// <exception cref="ArgumentException">An operand isn't three-dimensional.</exception>
    public Result<Part> Add(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => AddCore(operands).As<Part>());
    }

    /// <summary>This part with <paramref name="others"/>, solids or parts, cut away.</summary>
    /// <exception cref="ArgumentException">An operand isn't three-dimensional.</exception>
    public Result<Part> Subtract(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => SubtractCore(operands).As<Part>());
    }

    /// <summary>What this part has in common with <paramref name="others"/>: empty if nothing.</summary>
    public Result<Part> Common(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => CommonCore(operands).As<Part>());
    }

    /// <summary>The part fused with a shape (see <see cref="Add"/>).</summary>
    public static Part operator +(Part a, Shape b) => a.AddCore(Checked([b])).As<Part>();

    /// <summary>The part fused with shapes (see <see cref="Add"/>).</summary>
    public static Part operator +(Part a, IEnumerable<Shape> b) => a.AddCore(Checked(b)).As<Part>();

    /// <summary>The part with a shape cut away (see <see cref="Subtract"/>).</summary>
    public static Part operator -(Part a, Shape b) => a.SubtractCore(Checked([b])).As<Part>();

    /// <summary>The part with shapes cut away (see <see cref="Subtract"/>).</summary>
    public static Part operator -(Part a, IEnumerable<Shape> b) => a.SubtractCore(Checked(b)).As<Part>();

    /// <summary>What the part has in common with a shape (see <see cref="Common"/>).</summary>
    public static Part operator &(Part a, Shape b) => a.CommonCore(Checked([b])).As<Part>();

    /// <summary>What the part has in common with shapes (see <see cref="Common"/>).</summary>
    public static Part operator &(Part a, IEnumerable<Shape> b) => a.CommonCore(Checked(b)).As<Part>();
}

public partial class Sketch
{
    /// <summary>
    /// This sketch fused with <paramref name="others"/>, faces or sketches, within <see cref="Constants.Tolerance"/>.
    /// Adding nothing returns this sketch.
    /// </summary>
    /// <exception cref="ArgumentException">An operand isn't two-dimensional.</exception>
    public Result<Sketch> Add(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => AddCore(operands).As<Sketch>());
    }

    /// <summary>This sketch with <paramref name="others"/>, faces, sketches or solids, cut away.</summary>
    /// <exception cref="ArgumentException">An operand has fewer than two dimensions.</exception>
    public Result<Sketch> Subtract(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => SubtractCore(operands).As<Sketch>());
    }

    /// <summary>What this sketch has in common with <paramref name="others"/>: empty if nothing.</summary>
    public Result<Sketch> Common(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => CommonCore(operands).As<Sketch>());
    }

    /// <summary>The sketch fused with a shape (see <see cref="Add"/>).</summary>
    public static Sketch operator +(Sketch a, Shape b) => a.AddCore(Checked([b])).As<Sketch>();

    /// <summary>The sketch fused with shapes (see <see cref="Add"/>).</summary>
    public static Sketch operator +(Sketch a, IEnumerable<Shape> b) => a.AddCore(Checked(b)).As<Sketch>();

    /// <summary>The sketch with a shape cut away (see <see cref="Subtract"/>).</summary>
    public static Sketch operator -(Sketch a, Shape b) => a.SubtractCore(Checked([b])).As<Sketch>();

    /// <summary>The sketch with shapes cut away (see <see cref="Subtract"/>).</summary>
    public static Sketch operator -(Sketch a, IEnumerable<Shape> b) => a.SubtractCore(Checked(b)).As<Sketch>();

    /// <summary>What the sketch has in common with a shape (see <see cref="Common"/>).</summary>
    public static Sketch operator &(Sketch a, Shape b) => a.CommonCore(Checked([b])).As<Sketch>();

    /// <summary>What the sketch has in common with shapes (see <see cref="Common"/>).</summary>
    public static Sketch operator &(Sketch a, IEnumerable<Shape> b) => a.CommonCore(Checked(b)).As<Sketch>();
}

public partial class Curve
{
    /// <summary>
    /// This curve joined with <paramref name="others"/>, edges, wires or curves: connected ones into a wire, the rest
    /// fused, in a curve. Adding nothing returns this curve.
    /// </summary>
    /// <exception cref="ArgumentException">An operand isn't one-dimensional.</exception>
    public Result<Curve> Add(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => AddCore(operands).As<Curve>());
    }

    /// <summary>The curve joined with a shape (see <see cref="Add"/>).</summary>
    public static Curve operator +(Curve a, Shape b) => a.AddCore(Checked([b])).As<Curve>();

    /// <summary>The curve joined with shapes (see <see cref="Add"/>).</summary>
    public static Curve operator +(Curve a, IEnumerable<Shape> b) => a.AddCore(Checked(b)).As<Curve>();

    /// <summary>This curve with <paramref name="others"/>, of any dimension, cut away.</summary>
    public Result<Curve> Subtract(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => SubtractCore(operands).As<Curve>());
    }

    /// <summary>What this curve has in common with <paramref name="others"/>: empty if nothing.</summary>
    public Result<Curve> Common(params IEnumerable<Shape> others)
    {
        var operands = Checked(others);
        return Failures.Try(() => CommonCore(operands).As<Curve>());
    }

    /// <summary>The curve with a shape cut away (see <see cref="Subtract"/>).</summary>
    public static Curve operator -(Curve a, Shape b) => a.SubtractCore(Checked([b])).As<Curve>();

    /// <summary>The curve with shapes cut away (see <see cref="Subtract"/>).</summary>
    public static Curve operator -(Curve a, IEnumerable<Shape> b) => a.SubtractCore(Checked(b)).As<Curve>();

    /// <summary>What the curve has in common with a shape (see <see cref="Common"/>).</summary>
    public static Curve operator &(Curve a, Shape b) => a.CommonCore(Checked([b])).As<Curve>();
}

// The topological shapes (solids, faces, edges) keep build123d's operators, and their result is the kernel's: one solid
// or a compound of several pieces, a wire or a compound of edges. The class is decided by the result, as build123d's
// is and as the Python-reference tests pin it, so these return Shape; the algebra's classes above are typed.

public abstract partial class Shape
{
    /// <summary>
    /// This shape fused with another (build123d's <c>+</c>): the kernel's result, of whichever class it is. Only shapes
    /// of the same dimension can be added. A <see cref="Part"/>, <see cref="Sketch"/> or <see cref="Curve"/> has its own
    /// typed operator.
    /// </summary>
    public static Shape operator +(Shape a, Shape b) => a.AddCore(Checked([b]));

    /// <summary>This shape fused with others (see <see cref="op_Addition(Shape, Shape)"/>).</summary>
    public static Shape operator +(Shape a, IEnumerable<Shape> b) => a.AddCore(Checked(b));

    /// <summary>This shape with another cut away: only shapes of an equal or greater dimension can be subtracted.</summary>
    public static Shape operator -(Shape a, Shape b) => a.SubtractCore(Checked([b]));

    /// <summary>This shape with others cut away.</summary>
    public static Shape operator -(Shape a, IEnumerable<Shape> b) => a.SubtractCore(Checked(b));

    /// <summary>What this shape has in common with another: one shape, a composite of several, or the zero of its class.</summary>
    public static Shape operator &(Shape a, Shape b) => a.CommonCore(Checked([b]));

    /// <summary>What this shape has in common with others.</summary>
    public static Shape operator &(Shape a, IEnumerable<Shape> b) => a.CommonCore(Checked(b));
}

public abstract partial class Shape
{
    /// <summary>The operands of an algebra operation, refused if any is null.</summary>
    private protected static IEnumerable<Shape> Checked(IEnumerable<Shape> others)
    {
        ArgumentNullException.ThrowIfNull(others);
        var list = others.ToList();
        if (list.Contains(null!))
            throw new ArgumentNullException(nameof(others), "An operand is null.");
        return list;
    }

    /// <summary>
    /// This shape as the algebra's class <typeparamref name="T"/> (a Part, Sketch or Curve): itself if it is one, else
    /// a compound of that class around it, with its attributes and history.
    /// </summary>
    internal T As<T>() where T : Compound
    {
        if (this is T typed)
            return typed;
        Shape wrapped;
        if (this is Compound && !IsEmpty && TopoShape.IsCompound())
        {
            // A plain compound of the right dimension: the same kernel compound, in the algebra's class
            wrapped = typeof(T) == typeof(Part) ? new Part(TopoShape)
                : typeof(T) == typeof(Sketch) ? new Sketch(TopoShape)
                : new Curve(TopoShape);
            wrapped = wrapped.MadeBy(History);
        }
        else if (IsEmpty)
            wrapped = typeof(T) == typeof(Part) ? new Part() : typeof(T) == typeof(Sketch) ? new Sketch() : new Curve();
        else
            wrapped = MakeComposite([this], typeof(T) == typeof(Part) ? 3 : typeof(T) == typeof(Sketch) ? 2 : 1);
        return (T)CopyAttributesTo(wrapped);
    }
}
