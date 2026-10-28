using Build123d.Occt;

namespace Build123d.Occt.Tests;

/// <summary>Phase 0 spike: the generated bindings drive OCCT end to end.</summary>
public class SpikeTests
{
    const double BlockVolume = 1000.0 - Math.PI * 9.0 * 10.0;   // 10mm cube minus a r3 x 10 cylinder

    static TopoDS_Shape BoxMinusCylinder()
    {
        using var corner = new gp_Pnt(-5, -5, -5);
        using var box = new BRepPrimAPI_MakeBox(corner, 10, 10, 10);
        using var origin = new gp_Pnt(0, 0, -5);
        using var axis = new gp_Ax2(origin, gp.DZ());
        using var cylinder = new BRepPrimAPI_MakeCylinder(axis, 3, 10);
        using var cut = new BRepAlgoAPI_Cut(box.Shape(), cylinder.Shape());
        Assert.True(cut.IsDone());
        return cut.Shape();
    }

    static double Volume(TopoDS_Shape shape)
    {
        using var props = new GProp_GProps();
        BRepGProp.VolumeProperties(shape, props);
        return props.Mass();
    }

    [Fact]
    public void ReportsTheOcctVersion() => Assert.Equal("8.0.1", Occt.Version);

    [Fact]
    public void BoxMinusCylinderHasTheExpectedVolume()
    {
        using var block = BoxMinusCylinder();
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_COMPOUND, block.ShapeType());
        Assert.Equal(BlockVolume, Volume(block), 6);
    }

    [Fact]
    public void StepRoundTripKeepsTheVolume()
    {
        var path = Path.Combine(Path.GetTempPath(), $"b3d-spike-{Guid.NewGuid():N}.step");
        try
        {
            using (var block = BoxMinusCylinder())
            using (var writer = new STEPControl_Writer())
            {
                Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone,
                             writer.Transfer(block, STEPControl_StepModelType.STEPControl_AsIs));
                Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone, writer.Write(path));
            }

            using var reader = new STEPControl_Reader();
            Assert.Equal(IFSelect_ReturnStatus.IFSelect_RetDone, reader.ReadFile(path));
            Assert.True(reader.TransferRoots() > 0);
            using var read = reader.OneShape();
            Assert.Equal(BlockVolume, Volume(read), 6);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void OcctErrorsBecomeOcctExceptions()
    {
        // A box with no depth: BRepPrim refuses it with a Standard_DomainError
        var ex = Assert.Throws<OcctException>(() =>
        {
            using var box = new BRepPrimAPI_MakeBox(10, 10, 0);
            box.Shape();
        });
        Assert.Equal("Standard_DomainError", ex.OcctType);
    }

    [Fact]
    public void HandlesUpcastAndDowncast()
    {
        using var p = new gp_Pnt(1, 2, 3);
        using var line = new Geom_Line(p, gp.DX());

        // Use the line through its base class, then recover the derived type
        Geom_Curve curve = line;
        using var at2 = curve.Value(2);
        Assert.Equal((3.0, 2.0, 3.0), (at2.X(), at2.Y(), at2.Z()));

        using var back = Geom_Line.DownCast(curve);
        Assert.NotNull(back);
        Assert.Null(Geom_TrimmedCurve.DownCast(curve));
    }

    [Fact]
    public void DisposedObjectsThrowInsteadOfCrashing()
    {
        var p = new gp_Pnt(1, 2, 3);
        p.Dispose();
        Assert.True(p.IsDisposed);
        Assert.Throws<ObjectDisposedException>(() => p.X());
    }
}
