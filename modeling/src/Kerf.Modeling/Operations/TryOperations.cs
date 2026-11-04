namespace Kerf.Modeling;

// The Try forms of the operations that can fail in ways a caller should expect: each gives a Result<T> where its plain
// form throws a KerfException carrying the same diagnostics. A chain reads best with the plain forms
// (box.Fillet(…) - hole), and code that must not throw (a spreadsheet cell) takes the Try form at the step that may
// fail. Misuse, such as a null argument, throws in both.

public static partial class Operations
{
    /// <summary>The <see cref="Fillet(IEnumerable{Edge}, double)"/> of the edges, or its failure.</summary>
    public static Result<Part> TryFillet(IEnumerable<Edge> edges, double radius) => Failures.Try(() => Fillet(edges, radius));

    /// <summary>The <see cref="Chamfer(IEnumerable{Edge}, double, double?, double?, Face?)"/> of the edges, or its failure.</summary>
    public static Result<Part> TryChamfer(IEnumerable<Edge> edges, double length, double? length2 = null, double? angle = null,
                                          Face? reference = null) =>
        Failures.Try(() => Chamfer(edges, length, length2, angle, reference));

    /// <summary>The <see cref="Extrude(Shape, double, ExtrudeOptions?)"/> of a face or sketch, or its failure.</summary>
    public static Result<Part> TryExtrude(Shape toExtrude, double amount, ExtrudeOptions? options = null) =>
        Failures.Try(() => Extrude(toExtrude, amount, options));

    /// <summary>The <see cref="Extrude(Shape, Until, Shape, ExtrudeOptions?)"/> of a face or sketch, or its failure.</summary>
    public static Result<Part> TryExtrude(Shape toExtrude, Until until, Shape target, ExtrudeOptions? options = null) =>
        Failures.Try(() => Extrude(toExtrude, until, target, options));

    /// <summary>The <see cref="Loft(IEnumerable{Shape}, bool, bool)"/> through the sections, or its failure.</summary>
    public static Result<Part> TryLoft(IEnumerable<Shape> sections, bool ruled = false, bool clean = true) =>
        Failures.Try(() => Loft(sections, ruled, clean));

    /// <summary>The <see cref="Revolve(Shape, Axis?, double, bool)"/> of a profile, or its failure.</summary>
    public static Result<Part> TryRevolve(Shape profile, Axis? axis = null, double revolutionArc = 360.0, bool clean = true) =>
        Failures.Try(() => Revolve(profile, axis, revolutionArc, clean));

    /// <summary>The <see cref="Sweep(IEnumerable{Shape}, Shape, SweepOptions?)"/> of the sections, or its failure.</summary>
    public static Result<Compound> TrySweep(IEnumerable<Shape> sections, Shape path, SweepOptions? options = null) =>
        Failures.Try(() => Sweep(sections, path, options));

    /// <summary>The <see cref="Offset(Part, double, OffsetOptions?)"/> of a part, or its failure.</summary>
    public static Result<Part> TryOffset(Part obj, double amount, OffsetOptions? options = null) =>
        Failures.Try(() => Offset(obj, amount, options));

    /// <summary>The <see cref="Offset(Sketch, double, OffsetOptions?)"/> of a sketch, or its failure.</summary>
    public static Result<Sketch> TryOffset(Sketch obj, double amount, OffsetOptions? options = null) =>
        Failures.Try(() => Offset(obj, amount, options));

    /// <summary>The <see cref="Thicken(Shape, double, ThickenOptions?)"/> of faces, or its failure.</summary>
    public static Result<Part> TryThicken(Shape toThicken, double amount, ThickenOptions? options = null) =>
        Failures.Try(() => Thicken(toThicken, amount, options));

    /// <summary>The <see cref="Draft(IEnumerable{Face}, Plane, double)"/> of the faces, or its failure.</summary>
    public static Result<Part> TryDraft(IEnumerable<Face> faces, Plane neutralPlane, double angle) =>
        Failures.Try(() => Draft(faces, neutralPlane, angle));
}

public partial class Part
{
    /// <summary>The <see cref="Add"/> of the shapes, or the Boolean's failure.</summary>
    public Result<Part> TryAdd(params IEnumerable<Shape> others) => Failures.Try(() => Add(others));

    /// <summary>The <see cref="Subtract"/> of the shapes, or the Boolean's failure.</summary>
    public Result<Part> TrySubtract(params IEnumerable<Shape> others) => Failures.Try(() => Subtract(others));

    /// <summary>The <see cref="Common"/> of the shapes, or the Boolean's failure.</summary>
    public Result<Part> TryCommon(params IEnumerable<Shape> others) => Failures.Try(() => Common(others));
}

public partial class Sketch
{
    /// <summary>The <see cref="Add"/> of the shapes, or the Boolean's failure.</summary>
    public Result<Sketch> TryAdd(params IEnumerable<Shape> others) => Failures.Try(() => Add(others));

    /// <summary>The <see cref="Subtract"/> of the shapes, or the Boolean's failure.</summary>
    public Result<Sketch> TrySubtract(params IEnumerable<Shape> others) => Failures.Try(() => Subtract(others));

    /// <summary>The <see cref="Common"/> of the shapes, or the Boolean's failure.</summary>
    public Result<Sketch> TryCommon(params IEnumerable<Shape> others) => Failures.Try(() => Common(others));
}
