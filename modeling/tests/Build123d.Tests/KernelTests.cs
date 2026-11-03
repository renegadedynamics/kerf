namespace Build123d.Tests;

/// <summary>The kernel under build123d.NET: OCCT.NET, the managed port, with no native code.</summary>
public class KernelTests
{
    [Fact]
    public void FilletsAndChamfersRun()
    {
        // Phase 7 of OCCT.NET: 3D fillets and chamfers, offsets and drafts
        var filleted = Build123d.Operations.Fillet(new Box(10, 10, 10).Edges(), 1);
        Assert.True(filleted.IsValid);
        Assert.Equal(26, filleted.Faces().Count);
        // a box of 8 grown by 1: its faces, quarter cylinders along its edges and eighth spheres at its corners
        Check.AlmostEqual(512 + 384 + 24 * Math.PI + 4 * Math.PI / 3, filleted.Volume, 4);
        var chamfered = Build123d.Operations.Chamfer(new Box(10, 10, 10).Edges(), 1);
        Assert.True(chamfered.IsValid);
        Assert.Equal(26, chamfered.Faces().Count);
    }

    [Fact]
    public void NoNativeLibraryLoads()
    {
        _ = new Box(1, 1, 1).Triangulate(0.1);
        Assert.DoesNotContain(System.Diagnostics.Process.GetCurrentProcess().Modules.Cast<System.Diagnostics.ProcessModule>(),
                              m => m.ModuleName is "b3docct.dll" or "TKernel.dll");
    }

    [Fact]
    public void OcctFailuresAreOcctNetsExceptions()
    {
        var e = Assert.ThrowsAny<Kerf.Engine.Foundation.FailureException>(() => new Kerf.Geometry.Direction3D(0, 0, 0));
        Assert.Equal("Standard_ConstructionError", e.OcctType);
    }
}
