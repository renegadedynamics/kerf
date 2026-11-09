using System.Reflection;

namespace Kerf.Modeling.Tests.Objects;

using Kerf.Modeling;

/// <summary>
/// Decision 2008: an object's constructor that fails on values a caller would think valid throws one exception type,
/// <see cref="ModelingException"/>, with the error a <see cref="Result{T}"/> would carry, and has a <c>Make…</c> twin in
/// <see cref="Operations"/> returning that result. Every public constructor of every object is walked by reflection: each
/// must have a case here, with values that fail and values that don't.
/// </summary>
public class ConstructorFailureTests
{
    private static readonly object D = Type.Missing;   // the parameter's default

    private static Edge LineEdge(double x0, double y0, double x1, double y1) => Edge.MakeLine(new Point3D(x0, y0, 0), new Point3D(x1, y1, 0));

    private static readonly Point3D O = new(0, 0, 0);
    private static readonly Point3D X1 = new(1, 0, 0);
    private static readonly Point3D X2 = new(2, 0, 0);
    private static readonly Point3D X4 = new(4, 0, 0);
    private static readonly Point3D Y1 = new(0, 1, 0);
    private static readonly Point3D XY1 = new(1, 1, 0);
    private static readonly Vector3D Along = new(1, 0, 0);

    /// <summary>
    /// A constructor's case: values that fail as a modelling failure, with its code, and values that make the object. No
    /// failing values means none were found that look valid (a Bézier curve of points alone builds from any two or more).
    /// </summary>
    public sealed record Case(Type Type, string Code, object?[]? Failing, object?[] Valid)
    {
        public override string ToString() => $"{Type.Name}({string.Join(", ", Valid.Select(v => v == Type.Missing ? "default" : v?.GetType().Name ?? "null"))})";
    }

    private static Case C<T>(string code, object?[]? failing, object?[] valid) => new(typeof(T), code, failing, valid);

    private const string NoSolution = "modeling.no-solution";
    private const string NotReached = "modeling.limit-not-reached";
    private const string ObjectFailed = "modeling.object-failed";
    private const string Any = "";   // the kernel's code, whatever it is

    public static readonly Case[] Cases =
    [
        // Parts
        C<Box>(Any, [0.0, 1.0, 1.0, D, D], [1.0, 2.0, 3.0, D, D]),
        C<Cone>(Any, [0.0, 0.0, 1.0, D, D, D], [2.0, 1.0, 3.0, D, D, D]),
        C<Cylinder>(Any, [1.0, 0.0, D, D, D], [1.0, 2.0, D, D, D]),
        C<Sphere>(Any, [1.0, 10.0, 10.0, D, D, D], [1.0, D, D, D, D, D]),
        C<Torus>(Any, [1.0, 1.0, D, D, D, D, D], [3.0, 1.0, D, D, D, D, D]),
        C<Wedge>(Any, [10.0, 10.0, 10.0, 8.0, 2.0, 2.0, 8.0, D, D], [10.0, 10.0, 10.0, 2.0, 2.0, 8.0, 8.0, D, D]),
        C<ConvexPolyhedron>("modeling.hull-failed", [new Point3D[] { O, X1, Y1, XY1 }, D, D], [new Point3D[] { O, X1, Y1, new(0, 0, 1) }, D, D]),
        C<CounterBoreHole>(Any, [0.0, 2.0, 1.0, (double?)5], [1.0, 2.0, 1.0, (double?)5]),
        C<CounterSinkHole>(Any, [1.0, 2.0, (double?)5, 180.0], [1.0, 2.0, (double?)5, D]),
        C<Hole>(Any, [0.0, (double?)1], [1.0, (double?)1]),

        // Sketches
        C<Circle>(Any, [1.0, 0.0, D], [1.0, D, D]),
        C<Ellipse>(Any, [1.0, 0.0, D, D], [2.0, 1.0, D, D]),
        C<Polygon>(Any, [new List<Point3D> { O, X1, X2 }, D, D], [new List<Point3D> { O, X1, Y1 }, D, D]),
        C<Polygon>(Any, [new Point3D[] { O, X1, X2 }], [new Point3D[] { O, X1, Y1 }]),
        C<Rectangle>(Any, [0.0, 1.0, D, D], [2.0, 1.0, D, D]),
        C<RectangleRounded>(NoSolution, [10.0, 4.0, 2.0, D, D], [10.0, 4.0, 1.0, D, D]),
        C<RegularPolygon>(Any, [1e-8, 6, D, D, D], [1.0, 6, D, D, D]),
        C<SlotCurve>(Any, [(IShape1D)new Line(O, new Point3D(5, 0, 0)), 0.0, D], [(IShape1D)new Line(O, new Point3D(5, 0, 0)), 1.0, D]),
        C<SlotCenterPoint>(NoSolution, [O, O, 1.0, D], [O, X1, 1.0, D]),
        C<SlotCenterToCenter>(Any, [1e-9, 1.0, D], [2.0, 1.0, D]),
        C<SlotOverall>(NoSolution, [1.0, 2.0, D, D], [3.0, 1.0, D, D]),
        C<Superellipse>(Any, [10.0, 5.0, 0.5, D, D, D], [10.0, 5.0, D, D, D, D]),
        C<Trapezoid>(NoSolution, [10.0, 5.0, 45.0, D, D, D], [10.0, 2.0, 45.0, D, D, D]),
        C<Triangle>(NoSolution, [(double?)1, (double?)1, (double?)3, D, D, D, D, D], [(double?)3, (double?)4, (double?)5, D, D, D, D, D]),

        // Curves
        C<Airfoil>(Any, ["2412", 0, D], ["2412", D, D]),
        C<Bezier>(Any, [new List<Point3D> { O, X1, X2 }, new double[] { 1, -1, 1 }], [new List<Point3D> { O, XY1, X2 }, D]),
        C<Bezier>(Any, null, [new Point3D[] { O, XY1, X2 }]),
        C<BlendCurve>(NoSolution, [LineEdge(0, 0, 1, 0), LineEdge(2, 1, 3, 1), D, ((Point3D, Point3D)?)(new Point3D(1 + 2e-6, 0, 0), new Point3D(2, 1, 0)), D],
                                  [LineEdge(0, 0, 1, 0), LineEdge(2, 1, 3, 1), D, D, D]),
        C<BSpline>(Any, [new Point3D[] { O, XY1, X2 }, new double[] { 0, 0, 0, 1, 1 }, 2, D, D], [new Point3D[] { O, XY1, X2 }, new double[] { 0, 0, 0, 1, 1, 1 }, 2, D, D]),
        C<CenterArc>(Any, [O, -1.0, 0.0, 90.0], [O, 1.0, 0.0, 90.0]),
        C<CenterArc>(NotReached, [O, 1.0, 0.0, (CurveLimit)new Point3D(5, 5, 0)], [O, 1.0, 0.0, (CurveLimit)Axis.Y]),
        C<ConstrainedArcs>(NoSolution, [(Tangent)O, (Tangent)X4, 1.0, D, D], [(Tangent)O, (Tangent)X4, 3.0, D, D]),
        C<ConstrainedArcs>(NoSolution, [(Tangent)LineEdge(-5, 0, 5, 0), (Tangent)LineEdge(-5, 1, 5, 1), (Tangent)LineEdge(-5, 2, 5, 2), D, D],
                                       [(Tangent)LineEdge(-5, 0, 5, 0), (Tangent)LineEdge(-5, 2, 5, 2), (Tangent)LineEdge(0, -5, 0, 5), D, D]),
        C<ConstrainedArcs>(NoSolution, [(Tangent)O, O, D], [(Tangent)X1, O, D]),
        C<ConstrainedArcs>(NoSolution, [(Tangent)new Point3D(0, 10, 0), 1.0, (Tangent)Axis.X, D], [(Tangent)new Point3D(0, 1, 0), 1.0, (Tangent)Axis.X, D]),
        C<ConstrainedLines>(NoSolution, [(Tangent)new CenterArc(O, 2, 0, 360), (Tangent)O, D], [(Tangent)new CenterArc(O, 2, 0, 360), (Tangent)new Point3D(5, 0, 0), D]),
        C<ConstrainedLines>(NoSolution, [(Tangent)LineEdge(0, 0, 1, 0), Axis.X, 0.0, D], [(Tangent)Edge.MakeCircle(1.0, Plane.XY), Axis.X, 30.0, D]),
        C<ConstrainedLines>(NoSolution, [(Tangent)LineEdge(0, 0, 1, 0), Axis.X, Along, D], [(Tangent)Edge.MakeCircle(1.0, Plane.XY), Axis.X, new Vector3D(1, 1, 0), D]),
        C<DoubleTangentArc>(NoSolution, [O, Along, (IShape1D)new Line(X1, X2), D], [O, new Vector3D(0, 1, 0), (IShape1D)new Line(new Point3D(3, -1, 0), new Point3D(3, 5, 0)), D]),
        C<EllipticalCenterArc>(Any, [O, -1.0, 1.0, D, D, D], [O, 2.0, 1.0, D, D, D]),
        C<EllipticalCenterArc>(NotReached, [O, 2.0, 1.0, 0.0, (CurveLimit)new Point3D(9, 9, 0), D], [O, 2.0, 1.0, 0.0, (CurveLimit)Axis.Y, D]),
        C<EllipticalStartArc>(Any, [O, new Vector3D(0, 0, 1), 2.0, 1.0, 90.0, (double?)0, D], [O, Along, 2.0, 1.0, 90.0, (double?)0, D]),
        C<ParabolicCenterArc>(Any, [O, 1.0, 0.0, 0.0, D], [O, 1.0, D, D, D]),
        C<ParabolicCenterArc>(NotReached, [O, 0.5, 0.0, (CurveLimit)new Point3D(0, 5, 0), D], [O, 0.5, 0.0, (CurveLimit)LineEdge(0, 1, 5, 1), D]),
        C<HyperbolicCenterArc>(Any, [O, -1.0, 1.0, D, D, D], [O, 2.0, 1.0, D, D, D]),
        C<HyperbolicCenterArc>(NotReached, [O, 2.0, 1.0, 0.0, (CurveLimit)new Point3D(0, 5, 0), D], [O, 2.0, 1.0, 0.0, (CurveLimit)LineEdge(0, 1, 10, 1), D]),
        C<Helix>(Any, [0.0, 10.0, 1.0, D, D, D, D], [1.0, 10.0, 1.0, D, D, D, D]),
        C<FilletPolyline>(Any, [new Point3D[] { O, X1, XY1 }, 5.0, D], [new Point3D[] { O, X2, new(2, 2, 0) }, 0.5, D]),
        C<FilletPolyline>(Any, [new Point3D[] { O, X1, XY1 }, new double[] { 5 }, D], [new Point3D[] { O, X2, new(2, 2, 0) }, new double[] { 0.5 }, D]),
        C<JernArc>(Any, [O, Along, 1.0, 0.0], [O, Along, 1.0, 90.0]),
        C<JernArc>(NotReached, [O, Along, 1.0, (CurveLimit)new Point3D(9, 9, 0)], [O, Along, 1.0, (CurveLimit)Axis.Y]),
        C<Line>(Any, [O, O], [O, X1]),
        C<Line>(Any, [new Point3D[] { O, O }], [new Point3D[] { O, X1 }]),
        C<IntersectingLine>(NotReached, [O, Along, (Shape)LineEdge(0, 1, 1, 1)], [O, Along, (Shape)LineEdge(3, -1, 3, 1)]),
        C<PolarLine>(Any, [O, 0.0, (double?)0, D, D], [O, 2.0, (double?)30, D, D]),
        C<PolarLine>(NotReached, [O, (CurveLimit)LineEdge(0, 1, 1, 1), (double?)0, D], [O, (CurveLimit)LineEdge(3, -1, 3, 1), (double?)0, D]),
        C<Polyline>(Any, [new Point3D[] { O, X1, X1, XY1 }, D], [new Point3D[] { O, X1, XY1 }, D]),
        C<Polyline>(Any, [new Point3D[] { O, O }], [new Point3D[] { O, X1 }]),
        C<RadiusArc>(NoSolution, [O, X4, 1.0, D], [O, X4, 3.0, D]),
        C<SagittaArc>(Any, [O, X4, 0.0], [O, X4, 1.0]),
        C<Spline>(Any, [new Point3D[] { O, X1, new(1 + 1e-7, 0, 0), new(2, 1, 0) }, D, D, D], [new Point3D[] { O, X1, new(2, 1, 0) }, D, D, D]),
        C<Spline>(Any, [new Point3D[] { O, O }], [new Point3D[] { O, X1, new(2, 1, 0) }]),
        C<TangentArc>(Any, [O, X1, Along, D], [O, X1, new Vector3D(0, 1, 0), D]),
        C<TangentArc>(Any, [new Point3D[] { O, X1 }, Along, D], [new Point3D[] { O, X1 }, new Vector3D(0, 1, 0), D]),
        C<ThreePointArc>(Any, [O, X1, X2], [O, XY1, X2]),
        C<ThreePointArc>(Any, [new Point3D[] { O, X1, X2 }], [new Point3D[] { O, XY1, X2 }]),
    ];

    public static TheoryData<int> CaseIndices() => [.. Enumerable.Range(0, Cases.Length)];

    // The constructor a case's values fit: one, or the case is wrong
    private static ConstructorInfo ConstructorOf(Case c)
    {
        var fits = c.Type.GetConstructors().Where(k => Fits(k.GetParameters(), c.Valid) && (c.Failing is null || Fits(k.GetParameters(), c.Failing))).ToList();
        Assert.True(fits.Count == 1, $"{c}: {fits.Count} constructors fit");
        return fits[0];
    }

    private static bool Fits(ParameterInfo[] parameters, object?[] values) =>
        parameters.Length == values.Length && parameters.Zip(values).All(pair => pair.Second switch
        {
            null => !pair.First.ParameterType.IsValueType || Nullable.GetUnderlyingType(pair.First.ParameterType) is not null,
            var v when v == Type.Missing => pair.First.HasDefaultValue,
            var v => pair.First.ParameterType.IsInstanceOfType(v),
        });

    private static object Construct(ConstructorInfo constructor, object?[] values)
    {
        try
        {
            return constructor.Invoke(values);
        }
        catch (TargetInvocationException e) when (e.InnerException is not null)
        {
            throw e.InnerException;
        }
    }

    // The Make… operation beside a constructor: its name and parameters
    private static MethodInfo MakeOf(ConstructorInfo constructor)
    {
        var types = constructor.GetParameters().Select(p => p.ParameterType).ToArray();
        var make = typeof(Kerf.Modeling.Operations).GetMethod("Make" + constructor.DeclaringType!.Name, BindingFlags.Public | BindingFlags.Static, types);
        Assert.True(make is not null, $"No Kerf.Modeling.Operations.Make{constructor.DeclaringType!.Name} with the parameters of {constructor}");
        Assert.Equal(constructor.GetParameters().Select(p => (p.Name, p.HasDefaultValue ? p.DefaultValue : "none")),
                     make!.GetParameters().Select(p => (p.Name, p.HasDefaultValue ? p.DefaultValue : "none")));
        return make;
    }

    // A result's parts by reflection: Result<Part>, Result<Sketch> or Result<Curve>
    private static (bool IsSuccess, Diagnostic? Error, ModelShape? Value) Read(object result)
    {
        var type = result.GetType();
        var success = (bool)type.GetProperty("IsSuccess")!.GetValue(result)!;
        var error = (Diagnostic?)type.GetProperty("Error")!.GetValue(result);
        return (success, error, success ? (ModelShape)type.GetProperty("Value")!.GetValue(result)! : null);
    }

    // Two diagnostics alike: a diagnostic's shapes are compared by reference, so not by Equals
    private static (Severity, string, string) Said(Diagnostic d) => (d.Severity, d.Code.Value, d.Message);

    private static double Size(ModelShape shape) => shape switch
    {
        Part part => part.Volume,
        Sketch sketch => sketch.Faces().Sum(f => f.Area),
        _ => shape.Edges().Sum(e => e.Length),
    };

    [Fact]
    public void Every_public_constructor_of_an_object_has_a_case()
    {
        // The objects: the public classes deriving from Part, Sketch or Curve, not those three themselves (which hold a shape
        // they are given, and build nothing)
        var objects = typeof(ModelShape).Assembly.GetExportedTypes()
            .Where(t => t.IsClass && !t.IsAbstract && typeof(ModelShape).IsAssignableFrom(t))
            .Where(t => t != typeof(Part) && t != typeof(Sketch) && t != typeof(Curve) && t != typeof(Assembly))
            .ToList();
        var covered = Cases.Select(ConstructorOf).ToHashSet();
        var missing = objects.SelectMany(t => t.GetConstructors()).Where(k => !covered.Contains(k)).Select(k => $"{k.DeclaringType!.Name}{k}").ToList();
        Assert.True(missing.Count == 0, "No case for: " + string.Join("; ", missing));
        Assert.Equal(66, covered.Count);

        // Every Make… of an object is one of these constructors' twins
        var makes = typeof(Kerf.Modeling.Operations).GetMethods(BindingFlags.Public | BindingFlags.Static)
            .Where(m => m.Name.StartsWith("Make", StringComparison.Ordinal) && objects.Any(t => "Make" + t.Name == m.Name)).ToHashSet();
        Assert.Equal(makes.OrderBy(m => m.ToString()), covered.Select(MakeOf).OrderBy(m => m.ToString()));
    }

    [Theory]
    [MemberData(nameof(CaseIndices))]
    public void A_constructor_throws_only_a_ModelingException_and_its_Make_gives_the_same_error(int index)
    {
        var c = Cases[index];
        var constructor = ConstructorOf(c);
        if (c.Failing is null)
            return;
        var thrown = Record.Exception(() => Construct(constructor, c.Failing));
        Assert.True(thrown is not null, $"{c}: the failing values made an object");
        Assert.True(thrown.GetType() == typeof(ModelingException), $"{c}: {thrown.GetType().Name}: {thrown.Message}");
        var failure = (ModelingException)thrown;
        Assert.True(failure.Error.IsError);
        Assert.StartsWith(failure.Error.Message, failure.Message, StringComparison.Ordinal);
        if (c.Code != Any)
            Assert.Equal(c.Code, failure.Error.Code.Value);

        // The Make… version: the same error, as a result
        var make = MakeOf(constructor);
        var result = Read(make.Invoke(null, BindingFlags.OptionalParamBinding, null, c.Failing, null)!);
        Assert.False(result.IsSuccess);
        Assert.Equal(Said(failure.Error), Said(result.Error!));
    }

    [Theory]
    [MemberData(nameof(CaseIndices))]
    public void A_constructor_and_its_Make_build_the_same_object(int index)
    {
        var c = Cases[index];
        var constructor = ConstructorOf(c);
        var made = (ModelShape)Construct(constructor, c.Valid);
        Assert.True(made.IsValid, $"{c}: not valid");
        var result = Read(MakeOf(constructor).Invoke(null, BindingFlags.OptionalParamBinding, null, c.Valid, null)!);
        Assert.True(result.IsSuccess, $"{c}: {result.Error}");
        Assert.Equal(constructor.DeclaringType, result.Value!.GetType());
        Assert.Equal(Size(made), Size(result.Value), 9);
    }

    [Fact]
    public void ConstrainedArcs_CenteredOn_fails_as_its_constructors_do()
    {
        var failure = Assert.Throws<ModelingException>(() => ConstrainedArcs.CenteredOn(new Point3D(0, 1, 0), new Point3D(0, 1, 0), Axis.X));
        Assert.Equal(NoSolution, failure.Error.Code.Value);
        var result = Kerf.Modeling.Operations.MakeConstrainedArcsCenteredOn(new Point3D(0, 1, 0), new Point3D(0, 1, 0), Axis.X);
        Assert.Equal(Said(failure.Error), Said(result.Error!));
        var made = Kerf.Modeling.Operations.MakeConstrainedArcsCenteredOn(LineEdge(-5, 0, 5, 0), new Point3D(0, 2, 0), Axis.Y);
        Assert.Equal(ConstrainedArcs.CenteredOn(LineEdge(-5, 0, 5, 0), new Point3D(0, 2, 0), Axis.Y).Edges().Count, made.Value.Edges().Count);
    }

    [Fact]
    public void Arguments_wrong_on_their_own_are_still_argument_errors()
    {
        // Where the line is drawn (decision 2008): what the constructor checks before building, on the arguments alone
        Assert.Throws<ArgumentException>(() => new Wedge(-1, 10, 10, 2, 2, 8, 8));
        Assert.Throws<ArgumentException>(() => new RegularPolygon(1, 2));
        Assert.Throws<ArgumentException>(() => new Triangle(sideA: 1));
        Assert.Throws<ArgumentException>(() => new Hole(1));
        Assert.Throws<ArgumentException>(() => new Polyline(new Point3D(0, 0, 0)));
        Assert.Throws<ArgumentException>(() => new Spline(new Point3D(0, 0, 0)));
        Assert.Throws<ArgumentException>(() => new Bezier(new Point3D(0, 0, 0)));
        Assert.Throws<ArgumentException>(() => new FilletPolyline([new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0)], [1.0, 2.0]));
        Assert.Throws<ArgumentException>(() => new PolarLine(new Point3D(0, 0, 0), 1.0));
        Assert.Throws<ArgumentException>(() => new EllipticalStartArc(new Point3D(0, 0, 0), new Vector3D(1, 0, 0), 2, 1, 90));
        Assert.Throws<ArgumentException>(() => new RectangleRounded(10, 4, -1));
        Assert.Throws<ArgumentException>(() => new ConstrainedArcs(new Point3D(0, 0, 0), new Point3D(4, 0, 0), -1));
        Assert.Throws<ArgumentException>(() => new ConstrainedArcs(new Point3D(0, 0, 0), new Point3D(4, 0, 0), 3, selector: _ => []));
        Assert.Throws<ArgumentNullException>(() => new ConvexPolyhedron(null!));
        Assert.Throws<ArgumentException>(() => Kerf.Modeling.Operations.MakeWedge(-1, 10, 10, 2, 2, 8, 8));
    }

    [Fact]
    public void The_exception_carries_what_a_result_would()
    {
        Point3D[] flat = [new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(1, 1, 0)];
        var failure = Assert.Throws<ModelingException>(() => new ConvexPolyhedron(flat));
        var result = Kerf.Modeling.Operations.MakeConvexPolyhedron(flat);
        Assert.Equal(result.Diagnostics.Select(Said), failure.Diagnostics.Select(Said));
        Assert.Equal(Said(result.Error!), Said(failure.Error));
        Assert.False(failure.TryGetInvalid(out _));
        Assert.IsAssignableFrom<InvalidOperationException>(failure);

    }

    [Fact]
    public void A_rounded_rectangle_of_radius_0_is_the_rectangle()
    {
        // Decision 1999 found it threw: the 2D fillet refuses a radius of 0
        var rounded = new RectangleRounded(10, 4, 0);
        Assert.Equal(40, rounded.Faces().Sum(f => f.Area), 9);
        Assert.Equal(4, rounded.Edges().Count);
    }

    [Fact]
    public void A_large_triangle_finds_its_sides()
    {
        // Decision 1999: its sides were found by length to 1e-6, which a triangle this size misses
        var triangle = new Triangle(sideA: 3e10, sideB: 4e10, sideC: 5e10);
        Assert.Equal(3e10, triangle.EdgeA.Length, 3e10 * 1e-12);
        Assert.Equal(4e10, triangle.EdgeB.Length, 4e10 * 1e-12);
        Assert.Equal(5e10, triangle.EdgeC.Length, 5e10 * 1e-12);
    }
}
