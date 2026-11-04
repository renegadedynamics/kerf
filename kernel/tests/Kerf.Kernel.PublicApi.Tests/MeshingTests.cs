// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Exchange;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Kerf.Meshing (docs/api/meshing.md): a separate mesh, the shape unchanged.</summary>
public class MeshingTests
{
    [Fact]
    public void A_box_meshes_into_two_triangles_a_face()
    {
        Solid box = Fixtures.Box();
        Result<ShapeMesh> meshed = box.Mesh();
        Assert.True(meshed.IsSuccess);
        ShapeMesh mesh = meshed.Value;
        Assert.Equal(box.AsShape(), mesh.Shape);
        Assert.Equal(6, mesh.Faces.Count);
        Assert.Equal(12, mesh.Edges.Count);
        Assert.Equal(12, mesh.TriangleCount);
        Assert.Equal(24, mesh.NodeCount);
        Assert.Equal(3 * mesh.NodeCount, mesh.Positions.Length);
        Assert.Equal(3 * mesh.NodeCount, mesh.Normals.Length);
        Assert.Equal(3 * mesh.TriangleCount, mesh.Triangles.Length);
        int nodes = 0, triangles = 0;
        foreach (FaceMesh face in mesh.Faces)
        {
            Assert.Equal(nodes, face.FirstNode);
            Assert.Equal(triangles, face.FirstTriangle);
            Assert.Equal(SurfaceKind.Plane, face.Kind);
            Assert.Equal(box.Faces().Single(f => f.IsSame(face.Face)).Orientation, face.Face.Orientation);
            Assert.Equal(face.NodeCount, face.GetNodes().Length);
            Assert.Equal(face.NodeCount, face.GetSurfaceParameters().Length);
            nodes += face.NodeCount;
            triangles += face.TriangleCount;
        }

        Assert.All(mesh.Edges, e => Assert.Equal(2, e.PointCount));
    }

    [Fact]
    public void The_mesh_area_is_the_exact_area()
    {
        foreach (Solid solid in new[] { Fixtures.Box(), Fixtures.Cylinder() })
        {
            ShapeMesh mesh = solid.Mesh(new MeshOptions { LinearDeflection = 0.01, AngularDeflection = 0.1 }).Value;
            Assert.Equal(solid.Area(), MeshArea(mesh), 1e-2 * solid.Area());
        }
    }

    [Fact]
    public void Normals_are_unit_and_face_out()
    {
        Solid box = Fixtures.Box();
        ShapeMesh mesh = box.Mesh().Value;
        ReadOnlySpan<float> normals = mesh.Normals.Span, positions = mesh.Positions.Span;
        for (int i = 0; i < mesh.NodeCount; i++)
        {
            var n = new Vector3D(normals[3 * i], normals[3 * i + 1], normals[3 * i + 2]);
            Assert.Equal(1, n.Magnitude, 1e-6);
            var p = new Point3D(positions[3 * i], positions[3 * i + 1], positions[3 * i + 2]);
            Assert.True((p - new Point3D(5, 10, 15)).Dot(n) > 0);
        }
    }

    [Fact]
    public void Meshing_leaves_the_shape_alone()
    {
        Solid box = Fixtures.Box();
        byte[] before = BrepBytes(box);
        ShapeMesh first = box.Mesh().Value;
        Assert.Equal(before, BrepBytes(box));
        ShapeMesh second = Mesher.Mesh(box).Value;
        Assert.Equal(first.Positions.ToArray(), second.Positions.ToArray());
        Assert.Equal(first.Triangles.ToArray(), second.Triangles.ToArray());
    }

    [Fact]
    public void Options_are_checked_and_meshing_can_be_cancelled()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new MeshOptions { LinearDeflection = 0 });
        Assert.Throws<ArgumentOutOfRangeException>(() => new MeshOptions { AngularDeflection = -1 });
        Assert.Throws<ArgumentException>(() => default(Shape).Mesh());
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        Assert.ThrowsAny<OperationCanceledException>(() => Fixtures.Box().Mesh(cancellationToken: cancelled.Token));
    }

    private static byte[] BrepBytes(Shape shape)
    {
        var stream = new MemoryStream();
        Brep.Write(shape, stream);
        return stream.ToArray();
    }

    private static double MeshArea(ShapeMesh mesh)
    {
        double area = 0;
        foreach (FaceMesh face in mesh.Faces)
        {
            Point3D[] nodes = face.GetNodes();
            ReadOnlySpan<int> triangles = face.Triangles.Span;
            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3D u = nodes[triangles[i + 1]] - nodes[triangles[i]], v = nodes[triangles[i + 2]] - nodes[triangles[i]];
                area += 0.5 * u.Crossed(v).Magnitude;
            }
        }

        return area;
    }
}
