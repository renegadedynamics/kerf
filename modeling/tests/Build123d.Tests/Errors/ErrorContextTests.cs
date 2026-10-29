using Build123d.Occt;
using Ops = Build123d.Operations;

namespace Build123d.Tests.Errors;

/// <summary>An OCCT failure says what was being done, for someone reading it in a cell.</summary>
public class ErrorContextTests
{
    [Fact]
    public void APrimitiveSaysWhatItWasMaking()
    {
        var e = Assert.Throws<OcctException>(() => new Box(0, 1, 1));
        Assert.Equal("Standard_DomainError", e.OcctType);
        Assert.Equal("box 0.0 × 1.0 × 1.0", e.Operation);
        Assert.StartsWith("box 0.0 × 1.0 × 1.0 failed: Standard_DomainError", e.Message);
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
        var e = Assert.Throws<OcctException>(make);
        Assert.Equal(feature, e.Operation);
        Assert.StartsWith($"{feature} failed: {e.OcctType}", e.Message);
    }

    [Fact]
    public void ScopesNestAndEnd()
    {
        Assert.Null(OcctContext.Current);
        using (OcctContext.Doing("cell A1"))
        {
            using (OcctContext.Doing("extrude"))
                Assert.Equal("cell A1 / extrude", OcctContext.Current);
            var e = Assert.Throws<OcctException>(() => new Box(-1, 1, 1));
            Assert.Equal("cell A1 / box -1.0 × 1.0 × 1.0", e.Operation);
            Assert.Equal("cell A1", OcctContext.Current);
        }
        Assert.Null(OcctContext.Current);
    }

    [Fact]
    public void AScopeLeftOpenEndsWithItsParent()
    {
        var outer = OcctContext.Doing("outer");
        _ = OcctContext.Doing("forgotten");
        outer.Dispose();
        Assert.Null(OcctContext.Current);
    }

    [Fact]
    public void ScopesBelongToTheirThread()
    {
        using (OcctContext.Doing("here"))
        {
            string? there = "unset";
            var thread = new Thread(() => there = OcctContext.Current);
            thread.Start();
            thread.Join();
            Assert.Null(there);
        }
    }

    [Fact]
    public void LibraryExceptionsKeepOcctsAsTheirCause()
    {
        // build123d's own messages stand, with OCCT's failure inside
        var fillet = Assert.Throws<ArgumentException>(() => Ops.Fillet(new Box(1, 1, 1).Edges(), 5));
        Assert.Contains("Failed creating a fillet with radius of 5.0 on 12 of 12 edges", fillet.Message);
        Assert.IsType<OcctException>(fillet.InnerException);
    }

    [Fact]
    public void AFailedBooleanNamesOcctsReasons()
    {
        // OCCT reports why a boolean failed as alerts, where Shape() says only StdFail_NotDone
        using var fuse = new BRepAlgoAPI_Fuse();
        fuse.Build();
        Assert.True(fuse.HasErrors());
        Assert.Equal("BOPAlgo_AlertTooFewArguments", Extras.AlgoErrors(fuse));
        using var box = new BRepPrimAPI_MakeBox(1, 1, 1);
        using var common = new BRepAlgoAPI_Common(box.Shape(), box.Shape());
        Assert.Null(Extras.AlgoErrors(common));
    }
}
