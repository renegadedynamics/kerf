// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Meshing;
using Kerf.Topology;
using Xunit;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary><see cref="MeshOptions.ControlAllSurfaces"/> on the shape that shows it: the bottle's long fillet tori (decision 1680).</summary>
public class MeshOptionsControlAllSurfacesTests
{
    [Fact]
    public void ALongFilletGetsMoreTrianglesOnlyWhenAskedFor()
    {
        Shape bottle = BRepMeshModelKit.ReadDataOcc("bottle.brep");
        var options = new MeshOptions { LinearDeflection = 0.094, AngularDeflection = 0.2 };
        Assert.False(options.ControlAllSurfaces);

        int plain = Mesher.Mesh(bottle, options).Value.TriangleCount;
        int controlled = Mesher.Mesh(bottle, options with { ControlAllSurfaces = true }).Value.TriangleCount;

        Assert.True(controlled > plain * 1.2, $"{controlled} triangles against {plain}");
    }
}
