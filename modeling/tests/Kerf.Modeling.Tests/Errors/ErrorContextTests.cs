using Ops = Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Errors;

using Kerf.Modeling;

/// <summary>
/// A kernel failure says what was being done, for someone reading it in a cell: the native backend's "box 0.0 × 1.0 ×
/// 1.0 failed: …", lost in unit 10B (decision 1342), restored in 10F through the kernel's diagnostics. Since unit 10M3 a
/// failure is a failed result's diagnostics (Failures.Try, as the public operations return them); the shorthands that
/// can't return one throw ModelingException (decision 2013) with the same message, the kernel's failure inside. The tests keep
/// their names, under which native OCCT's differential log is recorded.
/// </summary>
public class ErrorContextTests
{
    [Fact]
    public void APrimitiveSaysWhatItWasMaking()
    {
        var failed = Failures.Try(() => new Box(0, 1, 1));
        // The kernel's public API refuses the size before OCCT would (unit 10M1): the refusal is said in context
        Assert.Equal("kernel.invalid-argument", failed.Error!.Code.Value);
        Assert.StartsWith("box 0 × 1 × 1 failed: The value must be a positive number", failed.Error.Message);
        // A constructor can't return a result: it throws, with the kernel's failure inside
        var e = Assert.Throws<ModelingException>(() => new Box(0, 1, 1));
        Assert.True(Failures.IsKernelFailure(e.InnerException!));
        Assert.StartsWith("box 0 × 1 × 1 failed: The value must be a positive number", e.Message);
    }

    [Fact]
    public void A_kernel_refusal_is_the_kernel_s_whichever_frame_throws_it()
    {
        // Decision 2028: once a call is hot, the JIT inlines the kernel's small checks, and the public methods calling
        // them, into the layer's methods, so a refusal is thrown from a frame of the layer's. Each refusal here is thrown
        // again from this test's frame, as the inlined check throws it, with nothing of the kernel's on its stack: the layer
        // must still take it for the kernel's, said in context. When the layer told them by the frame that threw
        // (decisions 2014, 2017), every one escaped Failures.Try as a bare argument exception
        Exception[] refusals =
        [
            Assert.Throws<ModelingException>(() => new Box(0, 1, 1)).InnerException!,                     // ArgumentOutOfRangeException
            Assert.Throws<ModelingException>(() => new Cone(0, 0, 1)).InnerException!,                    // ArgumentException
            Assert.Throws<ModelingException>(() => Solid.Extrude(Face.MakeRect(1, 1), new Vector3D(0, 0, 0))).InnerException!,
            new Kerf.Engine.Foundation.KernelArgumentNullException("profile"),                            // ArgumentNullException
        ];
        foreach (var refusal in refusals)
        {
            Assert.True(Diagnostic.IsKernelArgumentError(refusal), refusal.GetType().Name);
            var tried = Failures.Try<int>(() => throw refusal);
            Assert.Equal("kernel.invalid-argument", tried.Error!.Code.Value);
            Assert.StartsWith("operation failed: ", tried.Error.Message);
            // Thrown from here: no frame on its stack is the kernel's
            Assert.DoesNotContain(new System.Diagnostics.StackTrace(refusal).GetFrames(),
                                  f => System.Diagnostics.DiagnosticMethodInfo.Create(f)?.DeclaringAssemblyName.StartsWith("Kerf.Kernel.", StringComparison.Ordinal) == true);
            var built = Assert.Throws<ModelingException>(() => Failures.Building<int>(() => "box", () => throw refusal));
            Assert.Equal("kernel.invalid-argument", built.Error.Code.Value);
            Assert.StartsWith("box failed: ", built.Message);
            var done = Assert.Throws<ModelingException>(() => Failures.Doing<int>(() => "extrude", () => throw refusal));
            Assert.Equal("kernel.invalid-argument", done.Error.Code.Value);
        }

        // And the layer's own misuse, an argument exception of .NET's, still passes through as it is
        var misuse = new ArgumentException("Locations doesn't accept type Int32");
        Assert.Same(misuse, Assert.Throws<ArgumentException>(() => Failures.Try<int>(() => throw misuse)));
    }

    [Fact]
    public void A_kernel_refusal_is_the_kernel_s_once_the_JIT_has_optimised_the_call()
    {
        // The refusals that became the layer's own once hot, before decision 2028: the box and the extrusion (decision 2014),
        // and the cone, sphere, torus, circle and Bezier curve that went on doing so, measured on main after 13G (each said
        // modeling.object-failed, without its context, after about 450 rounds of ConstructorFailureTests' failing values).
        // Run long enough to be optimised (tiering waits 100 ms after 30 calls), each must still be the kernel's, every time
        Func<object>[] refused =
        [
            () => new Box(0, 1, 1),
            () => new Cone(0, 0, 1),
            () => new Sphere(1, 10, 10),
            () => new Torus(1, 1),
            () => new Circle(1, 0),
            () => new Bezier([new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(2, 0, 0)], [1, -1, 1]),
            () => Solid.Extrude(Face.MakeRect(1, 1), new Vector3D(0, 0, 0)),
        ];
        var watch = System.Diagnostics.Stopwatch.StartNew();
        int runs = 0;
        while (watch.ElapsedMilliseconds < 1500 || runs < 200)
        {
            foreach (var make in refused)
            {
                var result = Failures.Try(make);
                Assert.Equal("kernel.invalid-argument", result.Error!.Code.Value);
                Assert.Contains(" failed: ", result.Error.Message);
            }
            runs++;
        }
    }

    [Theory]
    [InlineData("extrude")]
    [InlineData("loft")]
    [InlineData("revolve")]
    [InlineData("sweep")]
    public void AFeatureSaysWhichItWas(string feature)
    {
        Func<Shape> make = feature switch
        {
            "extrude" => () => Solid.Extrude(Face.MakeRect(1, 1), new Vector3D(0, 0, 0)),
            "loft" => () => Solid.MakeLoft([Wire.MakeRect(1, 1), Wire.MakeRect(1, 1)]),
            "revolve" => () => Solid.Revolve(Face.MakeRect(1, 1).Moved(new Location(new Point3D(5.0, 0.0, 0.0))), 0, Axis.Y),
            _ => () => Solid.Sweep(Wire.MakeCircle(1), Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(0, 0, 1e-9))),
        };
        var failed = Failures.Try(make);
        Assert.StartsWith($"{feature} failed: ", failed.Error!.Message);
        // The kernel's code: an engine failure's, a refused argument's or the operation's own (construction.sweep-failed)
        Assert.Matches(@"^[a-z0-9-]+\.[a-z0-9-]+$", failed.Error.Code.Value);
    }

    [Fact]
    public void ANestedOperationSaysBoth()
    {
        // An operation inside another says both, outermost first, with the kernel's failure inside
        var failed = Failures.Try<int>(() => Failures.Doing(() => "extrude", () =>
            Failures.Doing<int>(() => "fuse of a Solid with a Solid", () =>
                throw Failures.Failed([Kerf.Diagnostic.Error(new Kerf.DiagnosticCode("engine.not-done"),
                                                             "the algorithm couldn't complete: BOPAlgo_AlertBuilderFailed")]))));
        Assert.Equal("extrude / fuse of a Solid with a Solid failed: the algorithm couldn't complete: BOPAlgo_AlertBuilderFailed",
                     failed.Error!.Message);
        Assert.Equal("engine.not-done", failed.Error.Code.Value);
    }

    [Fact]
    public void TheLayersOwnErrorsPassThrough()
    {
        // The modelling layer's own errors pass through unchanged
        var e = Assert.Throws<ArgumentException>(() => Failures.Doing<int>(() => "outer", () => throw new ArgumentException("mine")));
        Assert.Equal("mine", e.Message);
    }

    [Fact]
    public void TheContextIsFormattedOnlyOnFailure()
    {
        // What was being done is only formatted on failure, and success returns the value
        bool formatted = false;
        Assert.Equal(5, Failures.Doing(() => { formatted = true; return "nothing"; }, () => 5));
        Assert.False(formatted);
    }

    [Fact]
    public void LibraryExceptionsKeepOcctsAsTheirCause()
    {
        // build123d's own messages stand, with OCCT's failure inside
        var fillet = Ops.Fillet(new Box(1, 1, 1).Edges(), 5);
        Assert.Contains("The fillet with a radius of 5 is too large", fillet.Error!.Message);
        Assert.Contains("It failed on 12 of 12 edges", fillet.Error!.Message);
        Assert.Equal(ModelingDiagnostics.FilletFailed, fillet.Error.Code);
        // The kernel's diagnostics follow the layer's own
        Assert.Contains(fillet.Diagnostics.Skip(1), d => d.IsError && !d.Code.Value.StartsWith("modeling.", StringComparison.Ordinal));
        // The shorthand throws with the same message, the kernel's failure inside
        var box = new Box(1, 1, 1);
        var e = Assert.Throws<ModelingException>(() => box.Fillet(5, box.Edges()));
        Assert.Contains("It failed on 12 of 12 edges", e.Message);
        Assert.True(Failures.IsKernelFailure(e.InnerException!));
    }

    [Fact]
    public void AFailedBooleanNamesOcctsReasons()
    {
        // OCCT reports why a boolean failed as alerts, which the kernel's public Booleans turn into diagnostics (unit
        // 10L1); the modelling layer reads them from the result. A Boolean without arguments is refused before OCCT
        // (its BOPAlgo_AlertTooFewArguments can't arise), and a good one has no errors
        KernelAssert.Throws<ArgumentException>(() => TopoUtil.Boolean(BooleanKind.Fuse, [], []));
        var box = new Box(1, 1, 1).Shape;
        var common = TopoUtil.Boolean(BooleanKind.Common, [box], [box]);
        Assert.True(common.IsSuccess);
        Assert.DoesNotContain(common.Diagnostics, d => d.IsError);
    }
}
