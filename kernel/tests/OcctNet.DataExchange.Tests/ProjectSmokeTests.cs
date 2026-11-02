// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.DataExchange.Tests;

/// <summary>The data-exchange assembly builds and loads (phase 8's first units fill it).</summary>
public class ProjectSmokeTests
{
    [Fact]
    public void The_assembly_loads() =>
        Assert.Equal("OcctNet.DataExchange", typeof(OcctNet.DataExchange.AssemblyMarker).Assembly.GetName().Name);
}
