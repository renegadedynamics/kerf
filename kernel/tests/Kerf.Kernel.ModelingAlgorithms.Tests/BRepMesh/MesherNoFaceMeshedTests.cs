// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Meshing;
using Kerf.Topology;
using Xunit;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>A mesh in which no face could be meshed is a failure, carrying what there is (decision 1742; unit 12O).</summary>
public class MesherNoFaceMeshedTests
{
    [Fact]
    public void A_shape_none_of_whose_faces_meshes_is_a_failure_carrying_its_edges()
    {
        // A planar face bounded by a bow tie, whose wire crosses itself: the mesher fails it
        Wire bowTie = new MakePolygon(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(1, 0, 0), new Point3D(0, 1, 0), true).Wire();
        var builder = new BrepBuilder();
        Face face = new();
        builder.MakeFace(ref face, new PlaneSurface(new Plane3D()), Engine.Foundation.Precision.Confusion());
        builder.Add(face, bowTie);

        Result<ShapeMesh> meshed = Mesher.Mesh(face, new MeshOptions { LinearDeflection = 0.01 });
        Assert.False(meshed.IsSuccess);
        Assert.Equal(Mesher.MeshFailed, meshed.Error!.Code);
        Assert.True(meshed.TryGetInvalid(out ShapeMesh? mesh));
        Assert.Equal(0, mesh.TriangleCount);
        Assert.Equal(4, mesh.Edges.Count);
    }

    [Fact]
    public void A_shape_some_of_whose_faces_mesh_is_a_success_with_a_warning()
    {
        Wire bowTie = new MakePolygon(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(1, 0, 0), new Point3D(0, 1, 0), true).Wire();
        var builder = new BrepBuilder();
        Face face = new();
        builder.MakeFace(ref face, new PlaneSurface(new Plane3D()), Engine.Foundation.Precision.Confusion());
        builder.Add(face, bowTie);
        Shape square = new MakeFace(new MakePolygon(new Point3D(2, 0, 0), new Point3D(3, 0, 0), new Point3D(3, 1, 0), new Point3D(2, 1, 0), true).Wire()).Face();

        Result<ShapeMesh> meshed = Mesher.Mesh((Shape)Engine.Topology.Builder.MakeCompound(face, square), new MeshOptions { LinearDeflection = 0.01 });
        Assert.True(meshed.IsSuccess);
        Assert.Equal(Mesher.FacesNotMeshed, Assert.Single(meshed.Diagnostics).Code);
        Assert.Equal(2, meshed.Value.TriangleCount);
    }
}
