// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using Kerf.Geometry;
using Kerf.Tests.Shared;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// The kernel's public API against its baseline, docs/api/public-api.txt (unit 13P, decision 2043): any change to the
/// four assemblies' public surface fails here until the file is regenerated and committed with it, so every change to
/// the API is seen at review. Regenerate with <c>KERF_API_UPDATE=1</c>, running this test.
/// </summary>
public class PublicApiBaselineTests
{
    internal static IEnumerable<Assembly> Assemblies()
        => [typeof(Point3D).Assembly, typeof(Shape).Assembly, typeof(GeometryOperations).Assembly, typeof(Kerf.Exchange.Step).Assembly];

    [Fact]
    public void The_public_API_is_its_baseline()
    {
        string? difference = ApiSurface.CompareWithBaseline(
            Assemblies(), Path.Combine(ApiSurface.RepositoryRoot(), "kernel", "docs", "api", "public-api.txt"));
        Assert.True(difference is null, difference);
    }
}
