using Ops = Kerf.Modeling.Operations;

namespace Build123d.Tests.Errors;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>
/// A kernel failure says what was being done, for someone reading it in a cell: the native backend's "box 0.0 × 1.0 ×
/// 1.0 failed: …", lost in unit 10B (decision 1342), restored in 10F as a KerfException whose diagnostics are the kernel's
/// and whose inner exception is OCCT.NET's own. The tests keep their names, under which native OCCT's differential log is
/// recorded.
/// </summary>
public class ErrorContextTests
{
    [Fact]
    public void APrimitiveSaysWhatItWasMaking()
    {
        var e = Assert.Throws<Kerf.KerfException>(() => new Box(0, 1, 1));
        Assert.True(Failures.IsKernelFailure(e.InnerException!));
        Assert.Equal("engine.domain-error", e.Error.Code.Value);
        Assert.StartsWith("box 0.0 × 1.0 × 1.0 failed: a value is outside what the operation accepts", e.Message);
    }

    [Theory]
    [InlineData("extrude")]
    [InlineData("loft")]
    [InlineData("revolve")]
    [InlineData("sweep")]
    public void AFeatureSaysWhichItWas(string feature)
    {
        Action make = feature switch
        {
            "extrude" => () => Solid.Extrude(Face.MakeRect(1, 1), new Vector(0, 0, 0)),
            "loft" => () => Solid.MakeLoft([Wire.MakeRect(1, 1), Wire.MakeRect(1, 1)]),
            "revolve" => () => Solid.Revolve(Face.MakeRect(1, 1).Moved(new Location((5.0, 0.0, 0.0))), 0, Axis.Y),
            _ => () => Solid.Sweep(Wire.MakeCircle(1), Edge.MakeLine((0, 0, 0), (0, 0, 1e-9))),
        };
        var e = Assert.Throws<Kerf.KerfException>(make);
        Assert.StartsWith($"{feature} failed: ", e.Message);
        Assert.True(Failures.IsKernelFailure(e.InnerException!));
        Assert.StartsWith("engine.", e.Error.Code.Value);
    }

    [Fact]
    public void ANestedOperationSaysBoth()
    {
        // An operation inside another says both, outermost first, with the kernel's failure inside
        var e = Assert.Throws<Kerf.KerfException>(() => Failures.Doing(() => "extrude", () =>
            Failures.Doing(() => "fuse of a Solid with a Solid", () =>
                throw new Kerf.KerfException(Kerf.Diagnostic.Error(new Kerf.DiagnosticCode("engine.not-done"),
                                                                   "the algorithm couldn't complete: BOPAlgo_AlertBuilderFailed")))));
        Assert.Equal("extrude / fuse of a Solid with a Solid failed: the algorithm couldn't complete: BOPAlgo_AlertBuilderFailed", e.Message);
        Assert.IsType<Kerf.KerfException>(e.InnerException);
        Assert.Equal("engine.not-done", e.Error.Code.Value);
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
        var fillet = Assert.Throws<ArgumentException>(() => Ops.Fillet(new Box(1, 1, 1).Edges(), 5));
        Assert.Contains("Failed creating a fillet with radius of 5.0 on 12 of 12 edges", fillet.Message);
        Assert.True(Failures.IsKernelFailure(fillet.InnerException!));
    }

    [Fact]
    public void AFailedBooleanNamesOcctsReasons()
    {
        // OCCT reports why a boolean failed as alerts, which the kernel's public Booleans turn into diagnostics (unit
        // 10L1); the modelling layer reads them from the result. A Boolean without arguments is refused before OCCT
        // (its BOPAlgo_AlertTooFewArguments can't arise), and a good one has no errors
        Assert.Throws<ArgumentException>(() => TopoUtil.Boolean(BooleanKind.Fuse, [], []));
        var box = new Box(1, 1, 1).Wrapped;
        var common = TopoUtil.Boolean(BooleanKind.Common, [box], [box]);
        Assert.True(common.IsSuccess);
        Assert.DoesNotContain(common.Diagnostics, d => d.IsError);
    }
}
