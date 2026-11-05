namespace Build123d.Tests;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

/// <summary>The kernel under build123d.NET: Kerf, the managed port, with no native code.</summary>
public class KernelTests
{
    [Fact]
    public void FilletsAndChamfersRun()
    {
        // Phase 7 of Kerf: 3D fillets and chamfers, offsets and drafts
        var filleted = Kerf.Modeling.Operations.Fillet(new Box(10, 10, 10).Edges(), 1).Value;
        Assert.True(filleted.IsValid);
        Assert.Equal(26, filleted.Faces().Count);
        // a box of 8 grown by 1: its faces, quarter cylinders along its edges and eighth spheres at its corners
        Check.AlmostEqual(512 + 384 + 24 * Math.PI + 4 * Math.PI / 3, filleted.Volume, 4);
        var chamfered = Kerf.Modeling.Operations.Chamfer(new Box(10, 10, 10).Edges(), 1).Value;
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
        // Through the kernel's public API: the exception is the engine's, its diagnostic Standard_ConstructionError's
        var e = Record.Exception(() => new Kerf.Geometry.Direction3D(0, 0, 0));
        Assert.True(Kerf.Diagnostic.IsKernelFailure(e!));
        Assert.Equal("engine.construction-error", Kerf.Diagnostic.FromException(e!).Code.Value);
    }
}
