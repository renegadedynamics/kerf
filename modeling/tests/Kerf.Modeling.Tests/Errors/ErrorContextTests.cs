using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Occt;
using Ops = Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Errors;

/// <summary>
/// An OCCT failure reaches the caller as OCCT.NET's own exception, OCCT's type and message (unit 10B): the native
/// backend's OcctException, with what was being done, went with it, until results replace exceptions (unit 10F). The tests keep their names, under which native OCCT's
/// differential log is recorded.
/// </summary>
public class ErrorContextTests
{
    [Fact]
    public void APrimitiveSaysWhatItWasMaking()
    {
        var e = Assert.ThrowsAny<FailureException>(() => new Box(0, 1, 1));
        Assert.Equal("Standard_DomainError", e.OcctType);
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
        var e = Assert.ThrowsAny<FailureException>(make);
        Assert.StartsWith(e.OcctType, Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType()));
    }

    [Fact]
    public void LibraryExceptionsKeepOcctsAsTheirCause()
    {
        // build123d's own messages stand, with OCCT's failure inside
        var fillet = Assert.Throws<ArgumentException>(() => Ops.Fillet(new Box(1, 1, 1).Edges(), 5));
        Assert.Contains("Failed creating a fillet with radius of 5.0 on 12 of 12 edges", fillet.Message);
        Assert.IsAssignableFrom<FailureException>(fillet.InnerException);
    }

    [Fact]
    public void AFailedBooleanNamesOcctsReasons()
    {
        // OCCT reports why a boolean failed as alerts, where Shape() says only StdFail_NotDone
        var fuse = new Fuse();
        fuse.Build();
        Assert.True(fuse.HasErrors);
        Assert.Equal("BOPAlgo_AlertTooFewArguments", Extras.AlgoErrors(fuse));
        var box = new MakeBox(1, 1, 1);
        var common = new Common(box.Shape(), box.Shape());
        Assert.Null(Extras.AlgoErrors(common));
    }
}
