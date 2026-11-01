namespace Build123d.Tests;

/// <summary>
/// A test of the native backend's shim (native faults, native memory), skipped under the OCCT.NET backend
/// (Build123dBackend=OcctNet), which runs no native code.
/// </summary>
public sealed class NativeFactAttribute : FactAttribute
{
    public NativeFactAttribute()
    {
#if OCCTNET
        Skip = "tests the native shim; the OCCT.NET backend runs no native code";
#endif
    }
}

#if OCCTNET
/// <summary>The OCCT.NET backend's own behaviour.</summary>
public class OcctNetBackendTests
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
    public void WhatIsntPortedSaysSo()
    {
        // STEP isn't ported to OCCT.NET: its classes throw, with no native fallback
        var e = Assert.Throws<NotSupportedException>(() => new Build123d.Occt.STEPControl_Writer());
        Assert.Equal("not yet ported to OCCT.NET: STEPControl_Writer", e.Message);
        // nor are members OCCT.NET doesn't make public, such as the phase 7 algorithms' copy constructors
        using var fillet = new Build123d.Occt.BRepFilletAPI_MakeFillet(new Box(10, 10, 10).TopoShape);
        e = Assert.Throws<NotSupportedException>(() => new Build123d.Occt.BRepFilletAPI_MakeFillet(fillet));
        Assert.Equal("not available through OCCT.NET's public API: BRepFilletAPI_MakeFillet::.ctor(BRepFilletAPI_MakeFillet)",
                     e.Message);
    }

    [Fact]
    public void NoNativeLibraryLoads()
    {
        _ = new Box(1, 1, 1).Triangulate(0.1);
        Assert.DoesNotContain(System.Diagnostics.Process.GetCurrentProcess().Modules.Cast<System.Diagnostics.ProcessModule>(),
                              m => m.ModuleName is "b3docct.dll" or "TKernel.dll");
    }

    [Fact]
    public void OcctFailuresComeBackAsOcctExceptions()
    {
        var e = Assert.Throws<Build123d.Occt.OcctException>(() => new Build123d.Occt.gp_Dir(0, 0, 0));
        Assert.Equal("Standard_ConstructionError", e.OcctType);
        Assert.IsAssignableFrom<global::Occt.Standard_Failure>(e.InnerException);
    }
}
#endif
