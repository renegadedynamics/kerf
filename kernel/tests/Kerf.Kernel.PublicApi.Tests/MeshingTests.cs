// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
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
    public void Triangles_come_as_records_per_face_and_for_the_whole_mesh()
    {
        ShapeMesh mesh = Fixtures.Box().Mesh().Value;
        Triangle[] all = mesh.GetTriangles();
        Assert.Equal(mesh.TriangleCount, all.Length);
        ReadOnlySpan<int> buffer = mesh.Triangles.Span;
        foreach (FaceMesh face in mesh.Faces)
        {
            Triangle[] own = face.GetTriangles();
            Assert.Equal(face.TriangleCount, own.Length);
            for (int i = 0; i < own.Length; i++)
            {
                int t = face.FirstTriangle + i;
                Assert.Equal(new Triangle(buffer[3 * t], buffer[3 * t + 1], buffer[3 * t + 2]), own[i]);
                Assert.Equal(own[i] with { A = own[i].A + face.FirstNode, B = own[i].B + face.FirstNode, C = own[i].C + face.FirstNode }, all[t]);
            }
        }
        Assert.All(all, t => Assert.InRange(Math.Max(t.A, Math.Max(t.B, t.C)), 0, mesh.NodeCount - 1));
    }

    // The worst of a mesh's triangles' centres and edges' midpoints, measured from the surface
    private static double Deviation(ShapeMesh mesh)
    {
        double worst = 0;
        foreach (FaceMesh face in mesh.Faces)
        {
            Point3D[] nodes = face.GetNodes();
            foreach (Triangle t in face.GetTriangles())
            {
                Point3D a = nodes[t.A], b = nodes[t.B], c = nodes[t.C];
                Point3D[] samples =
                [
                    new((a.X + b.X + c.X) / 3, (a.Y + b.Y + c.Y) / 3, (a.Z + b.Z + c.Z) / 3),
                    new((a.X + b.X) / 2, (a.Y + b.Y) / 2, (a.Z + b.Z) / 2),
                    new((b.X + c.X) / 2, (b.Y + c.Y) / 2, (b.Z + c.Z) / 2),
                    new((a.X + c.X) / 2, (a.Y + c.Y) / 2, (a.Z + c.Z) / 2),
                ];
                foreach (Point3D sample in samples)
                {
                    worst = Math.Max(worst, face.Face.ParametersOf(sample, 1e-6).Gap);
                }
            }
        }
        return worst;
    }

    [Fact]
    public void A_thin_toruss_mesh_keeps_to_the_deflection_asked()
    {
        // OCCT #1444: 1.42 from the surface for 0.45 asked, and nothing said
        Shape face = Brep.Read(new MemoryStream(Fixtures.Bytes("thin-torus-face"))).Value;
        ShapeMesh mesh = face.Mesh(new MeshOptions { LinearDeflection = 0.45, AngularDeflection = 0.27, MinSize = 0.2 }).Value;
        Assert.InRange(Deviation(mesh), 0, 0.45);

        // A torus whose grid meets it isn't refined: 26 by 26 quadrilaterals, two triangles each
        ShapeMesh plain = Solid.Torus(10, 2).Value.Shape.Mesh(new MeshOptions { LinearDeflection = 0.281, AngularDeflection = 0.5 }).Value;
        Assert.Equal(2 * 26 * 26, plain.TriangleCount);
        Assert.InRange(Deviation(plain), 0, 0.281);
    }

    [Fact]
    public void A_spheres_mesh_keeps_to_the_deflection_asked_with_the_grids_triangles()
    {
        // Decision 1953: 1.70 times the deflection beside the seam, and twice it at the equator at a finer one, where
        // Delaunay took a length along u as 0.64 of what it is; the same 306 and 2,022 triangles now keep within it
        Shape sphere = Solid.Sphere(10).Value.Shape;
        ShapeMesh coarse = sphere.Mesh(new MeshOptions { LinearDeflection = 0.354, AngularDeflection = 0.5 }).Value;
        Assert.Equal(306, coarse.TriangleCount);
        Assert.InRange(Deviation(coarse), 0, 0.354);

        ShapeMesh fine = sphere.Mesh(new MeshOptions { LinearDeflection = 0.05, AngularDeflection = 0.2 }).Value;
        Assert.Equal(2022, fine.TriangleCount);
        Assert.InRange(Deviation(fine), 0, 0.05);

        // A sphere's part on a box: 1.55 times out before
        Shape part = Solid.Box(20, 20, 20).Value.Shape.Fuse(Solid.Sphere(6, new Frame3D(new Point3D(10, 10, 10), new Direction3D(0, 0, 1))).Value.Shape).Value.Shape;
        Assert.InRange(Deviation(part.Mesh(new MeshOptions { LinearDeflection = 0.1, AngularDeflection = 0.5 }).Value), 0, 0.1);
    }

    [Fact]
    public void A_cone_whose_seam_is_a_little_off_vertical_has_its_triangles()
    {
        // OCCT #572, from its STEP file: its seam's pcurves lean by 2e-14, and it had no triangles and a warning
        Shape face = Brep.Read(new MemoryStream(Fixtures.Bytes("cone-leaning-seam"))).Value;
        Result<ShapeMesh> meshed = face.Mesh(new MeshOptions { LinearDeflection = 0.1, AngularDeflection = 0.5 });
        Assert.Empty(meshed.Diagnostics);
        ShapeMesh mesh = meshed.Value;
        Assert.True(mesh.TriangleCount > 0);

        double area = 0;
        Point3D[] nodes = mesh.Faces[0].GetNodes();
        foreach (Triangle t in mesh.Faces[0].GetTriangles())
        {
            area += new Vector3D(nodes[t.A], nodes[t.B]).Crossed(new Vector3D(nodes[t.A], nodes[t.C])).Magnitude / 2;
        }

        Assert.Equal(face.IntegrateArea(), area, face.IntegrateArea() * 0.01);
        Assert.InRange(Deviation(mesh), 0, 0.1);
    }

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
            Assert.Equal(box.AllFaces().Single(f => f.IsSame(face.Face)).Orientation, face.Face.Orientation);
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
            Assert.Equal(solid.IntegrateArea(), MeshArea(mesh), 1e-2 * solid.IntegrateArea());
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
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => new MeshOptions { LinearDeflection = 0 });
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => new MeshOptions { AngularDeflection = -1 });
        KernelAssert.Throws<ArgumentException>(() => default(Shape).Mesh());
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
