// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Drawing;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Meshing;
using Kerf.Topology;
using Xunit;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// A mesh whose sphere or torus faces their guards leave further from the surface than the deflection succeeds with a
/// warning, <c>meshing.deflection-exceeded</c>, naming the faces and giving the furthest (decision 2071). The mesh is
/// the one it was without the warning: the warning only reads what the guards measured.
/// </summary>
public class MeshDeflectionWarningTests
{
    // A cylinder of radius 10 and height 40, its top edge filleted at 0.3: at a relative deflection of 0.005 and an angle
    // of 0.5 its fillet, a torus, is 1.005 times its deflection by its guard's measure
    internal static Shape FilletedCylinder()
    {
        Shape cylinder = new ApiMakeCylinder(10, 40).Shape();
        var fillet = new MakeFillet(cylinder);
        foreach (Shape edge in MeshExtraction.Edges(cylinder))
        {
            var curve = new EdgeCurveAdaptor(ShapeCasts.Edge(edge));
            if (curve.GetType() == CurveKind.Circle && curve.Value(curve.FirstParameter()).Z > 39)
            {
                fillet.Add(0.3, ShapeCasts.Edge(edge));
            }
        }

        return fillet.Shape();
    }

    internal static MeshOptions Relative(double coefficient, double angle) =>
        new() { LinearDeflection = coefficient, AngularDeflection = angle, Relative = true };

    [Fact]
    public void A_fillet_torus_of_Bottom_over_its_deflection_gives_the_warning_with_its_figures()
    {
        // data/occ's Bottom at a relative deflection of 0.005: three fillet tori over, by 13O's survey and by their
        // guards; face 123 the furthest, 1.129 times its deflection
        Shape shape = BrepTools.Cleaned(BRepMeshModelKit.ReadDataOcc("Bottom.brep"));
        Result<ShapeMesh> meshed = Mesher.Mesh(shape, Relative(0.005, 0.5));

        Assert.True(meshed.IsSuccess);
        Diagnostic warning = Assert.Single(meshed.Diagnostics);
        Assert.Equal(Mesher.DeflectionExceeded, warning.Code);
        Assert.Equal(Severity.Warning, warning.Severity);
        Assert.Equal("3 faces (faces 49, 71, 123) are further from their surfaces than the deflection: the furthest, face 123, is "
                     + "0.1421 from it, 1.129 times the deflection of 0.1258 (relative mode's for that face). The mesher couldn't refine them further.",
                     warning.Message);

        // The faces named are the mesh's own, by index, and tori
        List<Shape> faces = MeshExtraction.Faces(shape);
        Assert.Equal([faces[49], faces[71], faces[123]], warning.Shapes.ToArray(), ShapeComparer.Same);
        Assert.All(warning.Shapes, f => Assert.Equal(SurfaceKind.Torus, new FaceSurfaceAdaptor(ShapeCasts.Face(f)).GetType()));

        // The figure is the mesh's: measured from the torus at 13 points a triangle, face 123 is as far as the warning says
        FaceMesh face = meshed.Value.Faces[123];
        Assert.True(face.Face.IsSame(faces[123]));
        Torus3D torus = new FaceSurfaceAdaptor(face.Face).Torus();
        double furthest = Furthest(torus, face);
        Assert.InRange(furthest, 0.1421 * 0.995, 0.1421 * 1.005);
    }

    [Fact]
    public void A_mesh_within_its_deflection_gives_no_warning()
    {
        // A torus and a sphere at relative deflections, and the filleted cylinder at an absolute one: all within
        Assert.Empty(Mesher.Mesh(new MakeTorus(10, 2).Shape(), Relative(0.01, 0.5)).Diagnostics);
        Assert.Empty(Mesher.Mesh(new MakeSphere(10).Shape(), Relative(0.001, 0.5)).Diagnostics);
        Assert.Empty(Mesher.Mesh(FilletedCylinder(), new MeshOptions { LinearDeflection = 0.05, AngularDeflection = 0.5 }).Diagnostics);
        Assert.Empty(Mesher.Mesh(FilletedCylinder(), Relative(0.01, 0.5)).Diagnostics);
    }

    [Fact]
    public void A_face_a_little_over_inside_its_triangles_is_refined_within_and_gives_no_warning()
    {
        // data/occ's MODERN_Cooker_1 at a relative deflection of 0.002: four fillet tori were 1.035 times their deflection
        // at the midpoints of their triangles' medians, where the guard measured nothing, with no warning (decision 2071's
        // 25 faces); the guard now measures there and refines them within (decision 2102)
        Shape shape = BrepTools.Cleaned(BRepMeshModelKit.ReadDataOcc("MODERN_Cooker_1.brep"));
        Result<ShapeMesh> meshed = Mesher.Mesh(shape, Relative(0.002, 0.5));

        Assert.True(meshed.IsSuccess);
        Assert.Empty(meshed.Diagnostics);
        ToolParameters parameters = BRepMeshModelKit.Params(0.002, 0.5, isRelative: true);
        ModelModel model = BRepMeshModelKit.Model(shape, parameters, BRepMeshModelKit.Stage.Healed);
        List<Shape> faces = MeshExtraction.Faces(shape);
        int tori = 0;
        for (int i = 0; i < model.FacesNb(); i++)
        {
            Face face = model.GetFace(i).GetFace();
            var surface = new FaceSurfaceAdaptor(face);
            if (surface.GetType() != SurfaceKind.Torus)
            {
                continue;
            }

            int index = faces.FindIndex(f => f.IsSame(face));
            double furthest = Furthest(surface.Torus(), meshed.Value.Faces[index]);
            Assert.True(furthest <= model.GetFace(i).GetDeflection() * 1.001, $"face {index}: {furthest:G4} for {model.GetFace(i).GetDeflection():G4}");
            tori++;
        }

        Assert.Equal(12, tori);
    }

    [Fact]
    public void Meshes_at_an_absolute_deflection_are_unchanged_by_the_measure_inside()
    {
        // The measure inside a triangle, taken once the control has nothing left to add, finds nothing on a mesh within
        // its deflection (decision 2102): the same triangles and nodes as before it, at an absolute deflection. Taken
        // beside the centre's measure instead, the pin's fillet, 0.84 of its deflection, got 1,300 triangles
        ShapeMesh torus = Mesher.Mesh(new MakeTorus(10, 2).Shape(), new MeshOptions { LinearDeflection = 0.05, AngularDeflection = 0.5 }).Value;
        Assert.Equal((2548, 1350), (torus.TriangleCount, torus.NodeCount));
        ShapeMesh pin = Mesher.Mesh(FilletedCylinder(), new MeshOptions { LinearDeflection = 0.05, AngularDeflection = 0.5 }).Value;
        Assert.Equal((1206, 749), (pin.TriangleCount, pin.NodeCount));
    }

    [Fact]
    public void The_filleted_cylinder_gives_the_warning_and_the_same_mesh_serially_and_in_parallel()
    {
        Result<ShapeMesh> parallel = Mesher.Mesh(FilletedCylinder(), Relative(0.005, 0.5));
        Result<ShapeMesh> serial = Mesher.Mesh(FilletedCylinder(), Relative(0.005, 0.5) with { Parallel = false });
        Diagnostic warning = Assert.Single(parallel.Diagnostics);
        Assert.Equal(Mesher.DeflectionExceeded, warning.Code);
        Assert.StartsWith("Face 2 is further from its surface than the deflection: 0.05438 from it, 1.005 times the deflection of 0.05412", warning.Message);
        Assert.Equal(warning.Message, Assert.Single(serial.Diagnostics).Message);
        Assert.Equal(parallel.Value.Triangles.ToArray(), serial.Value.Triangles.ToArray());
        Assert.Equal(parallel.Value.Positions.ToArray(), serial.Value.Positions.ToArray());
    }

    [Fact]
    public void Rendering_a_shape_passes_the_warning_on()
    {
        Result<RgbaImage> picture = FilletedCylinder().Render(Camera.Isometric, new RenderOptions { Width = 40, Height = 30 },
                                                              meshOptions: Relative(0.005, 0.5));
        Assert.True(picture.IsSuccess);
        Assert.Equal(Mesher.DeflectionExceeded, Assert.Single(picture.Diagnostics).Code);
    }

    // The furthest distance from the torus over each triangle's 12 points on a grid of quarters (its corners left out)
    // and its centroid
    private static double Furthest(Torus3D torus, FaceMesh face)
    {
        Point3D o = torus.Location;
        Direction3D z = torus.Axis.Direction;
        double Distance(double x, double y, double w)
        {
            x -= o.X; y -= o.Y; w -= o.Z;
            double h = x * z.X + y * z.Y + w * z.Z;
            double rx = x - h * z.X, ry = y - h * z.Y, rz = w - h * z.Z;
            double rho = Math.Sqrt(rx * rx + ry * ry + rz * rz) - torus.MajorRadius;
            return Math.Abs(Math.Sqrt(rho * rho + h * h) - torus.MinorRadius);
        }

        Point3D[] nodes = face.GetNodes();
        double worst = 0;
        const int n = 4;
        foreach (Kerf.Meshing.Triangle t in face.GetTriangles())
        {
            Point3D a = nodes[t.A], b = nodes[t.B], c = nodes[t.C];
            for (int i = 0; i <= n; i++)
            {
                for (int j = 0; i + j <= n; j++)
                {
                    int k = n - i - j;
                    if (i < n && j < n && k < n)
                    {
                        worst = Math.Max(worst, Distance((i * a.X + j * b.X + k * c.X) / n, (i * a.Y + j * b.Y + k * c.Y) / n, (i * a.Z + j * b.Z + k * c.Z) / n));
                    }
                }
            }

            worst = Math.Max(worst, Distance((a.X + b.X + c.X) / 3, (a.Y + b.Y + c.Y) / 3, (a.Z + b.Z + c.Z) / 3));
        }

        return worst;
    }
}
