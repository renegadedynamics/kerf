// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Kernel.DataExchange.Tests;

/// <summary>The data-exchange assembly builds and loads (phase 8's first units fill it).</summary>
public class ProjectSmokeTests
{
    [Fact]
    public void The_assembly_loads() =>
        Assert.Equal("Kerf.Kernel.DataExchange", typeof(Kerf.Kernel.DataExchange.AssemblyMarker).Assembly.GetName().Name);
}
