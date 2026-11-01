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
    public void WhatIsntPortedSaysSo()
    {
        // Phase 7 of OCCT.NET: 3D fillets, chamfers, offsets and drafts throw, with no native fallback
        var e = Assert.Throws<NotSupportedException>(() => Build123d.Operations.Fillet(new Box(10, 10, 10).Edges(), 1));
        Assert.Equal("not yet ported to OCCT.NET: BRepFilletAPI_MakeFillet", e.Message);
        e = Assert.Throws<NotSupportedException>(() => Build123d.Operations.Chamfer(new Box(10, 10, 10).Edges(), 1));
        Assert.Equal("not yet ported to OCCT.NET: BRepFilletAPI_MakeChamfer", e.Message);
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
