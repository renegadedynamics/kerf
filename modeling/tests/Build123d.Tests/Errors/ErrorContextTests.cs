using Occt;
using Ops = Build123d.Operations;

namespace Build123d.Tests.Errors;

/// <summary>
/// An OCCT failure reaches the caller as OCCT.NET's own exception, OCCT's type and message (unit 10B): the native
/// backend's OcctException, with what was being done, went with it, until results replace exceptions (unit 10F).
/// </summary>
public class ErrorContextTests
{
    [Fact]
    public void APrimitiveFailsWithOcctsException()
    {
        var e = Assert.ThrowsAny<Standard_Failure>(() => new Box(0, 1, 1));
        Assert.Equal("Standard_DomainError", e.OcctType);
    }

    [Theory]
    [InlineData("extrude")]
    [InlineData("loft")]
    [InlineData("revolve")]
    [InlineData("sweep")]
    public void AFeatureFailsWithOcctsException(string feature)
    {
        Action make = feature switch
        {
            "extrude" => () => Solid.Extrude(Face.MakeRect(1, 1), new Vector(0, 0, 0)),
            "loft" => () => Solid.MakeLoft([Wire.MakeRect(1, 1), Wire.MakeRect(1, 1)]),
            "revolve" => () => Solid.Revolve(Face.MakeRect(1, 1).Moved(new Location((5.0, 0.0, 0.0))), 0, Axis.Y),
            _ => () => Solid.Sweep(Wire.MakeCircle(1), Edge.MakeLine((0, 0, 0), (0, 0, 1e-9))),
        };
        var e = Assert.ThrowsAny<Standard_Failure>(make);
        Assert.StartsWith(e.OcctType, e.GetType().Name);
    }

    [Fact]
    public void LibraryExceptionsKeepOcctsAsTheirCause()
    {
        // build123d's own messages stand, with OCCT's failure inside
        var fillet = Assert.Throws<ArgumentException>(() => Ops.Fillet(new Box(1, 1, 1).Edges(), 5));
        Assert.Contains("Failed creating a fillet with radius of 5.0 on 12 of 12 edges", fillet.Message);
        Assert.IsAssignableFrom<Standard_Failure>(fillet.InnerException);
    }

    [Fact]
    public void AFailedBooleanNamesOcctsReasons()
    {
        // OCCT reports why a boolean failed as alerts, where Shape() says only StdFail_NotDone
        var fuse = new BRepAlgoAPI_Fuse();
        fuse.Build();
        Assert.True(fuse.HasErrors());
        Assert.Equal("BOPAlgo_AlertTooFewArguments", Extras.AlgoErrors(fuse));
        var box = new BRepPrimAPI_MakeBox(1, 1, 1);
        var common = new BRepAlgoAPI_Common(box.Shape(), box.Shape());
        Assert.Null(Extras.AlgoErrors(common));
    }
}
