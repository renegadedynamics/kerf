using Ops = Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Errors;

using Kerf.Modeling;

/// <summary>
/// A kernel failure says what was being done, for someone reading it in a cell: the native backend's "box 0.0 × 1.0 ×
/// 1.0 failed: …", lost in unit 10B (decision 1342), restored in 10F through the kernel's diagnostics. Since unit 10M3 a
/// failure is a failed result's diagnostics (Failures.Try, as the public operations return them); the shorthands that
/// can't return one throw InvalidOperationException with the same message, the kernel's failure inside. The tests keep
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
        var e = Assert.Throws<InvalidOperationException>(() => new Box(0, 1, 1));
        Assert.True(Failures.IsKernelFailure(e.InnerException!));
        Assert.StartsWith("box 0 × 1 × 1 failed: The value must be a positive number", e.Message);
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
        Assert.Contains("The fillet with a radius of 5 on 12 of 12 edges", fillet.Error!.Message);
        Assert.Equal(ModelingDiagnostics.FilletFailed, fillet.Error.Code);
        // The kernel's diagnostics follow the layer's own
        Assert.Contains(fillet.Diagnostics.Skip(1), d => d.IsError && !d.Code.Value.StartsWith("modeling.", StringComparison.Ordinal));
        // The shorthand throws with the same message, the kernel's failure inside
        var box = new Box(1, 1, 1);
        var e = Assert.Throws<InvalidOperationException>(() => box.Fillet(5, box.Edges()));
        Assert.Contains("The fillet with a radius of 5 on 12 of 12 edges", e.Message);
        Assert.True(Failures.IsKernelFailure(e.InnerException!));
    }

    [Fact]
    public void AFailedBooleanNamesOcctsReasons()
    {
        // OCCT reports why a boolean failed as alerts, which the kernel's public Booleans turn into diagnostics (unit
        // 10L1); the modelling layer reads them from the result. A Boolean without arguments is refused before OCCT
        // (its BOPAlgo_AlertTooFewArguments can't arise), and a good one has no errors
        Assert.Throws<ArgumentException>(() => TopoUtil.Boolean(BooleanKind.Fuse, [], []));
        var box = new Box(1, 1, 1).Shape;
        var common = TopoUtil.Boolean(BooleanKind.Common, [box], [box]);
        Assert.True(common.IsSuccess);
        Assert.DoesNotContain(common.Diagnostics, d => d.IsError);
    }
}
