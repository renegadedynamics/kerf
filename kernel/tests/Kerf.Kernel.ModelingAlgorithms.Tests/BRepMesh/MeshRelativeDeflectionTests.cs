// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Meshing;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// At a relative deflection a sphere's and a torus's meshes keep to the deflection relative mode works out for each face
/// (decision 26301), measured from the surface: each triangle at 13 points, exactly. Before it, they were OCCT's, which
/// missed it by the factors noted.
/// </summary>
public class MeshRelativeDeflectionTests
{
    public static TheoryData<string, double, double> Cases => new()
    {
        { "sphere", 0.001, 0.5 },        // 2.06 times out
        { "sphere", 0.05, 0.5 },         // 1.20
        { "torus", 0.01, 0.5 },          // within before, and after
        { "Pump_Nut.brep", 0.005, 0.5 }, // its tori 2.59
        { "Ball.brep", 0.002, 0.5 },     // its spheres 3.91, its tori 2.94
    };

    [Theory]
    [MemberData(nameof(Cases))]
    public void Spheres_and_tori_keep_to_their_faces_deflection(string name, double coefficient, double angle)
    {
        Shape shape = name switch
        {
            "sphere" => new MakeSphere(10).Shape(),
            "torus" => new MakeTorus(10, 2).Shape(),
            _ => BrepTools.Cleaned(BRepMeshModelKit.ReadDataOcc(name)),
        };
        ToolParameters parameters = BRepMeshModelKit.Params(coefficient, angle, isRelative: true);

        // Each face's deflection as relative mode works it out, from the model before meshing; and the faces in the same
        // order from the meshed shape, which may be a copy
        ModelModel before = BRepMeshModelKit.Model(shape, parameters, BRepMeshModelKit.Stage.Healed);
        Shape meshed = new IncrementalMesh(shape, coefficient, true, angle).Shape();
        ModelModel after = BRepMeshModelKit.Model(meshed, parameters, BRepMeshModelKit.Stage.Built);
        Assert.Equal(before.FacesNb(), after.FacesNb());

        int measured = 0;
        for (int i = 0; i < before.FacesNb(); i++)
        {
            Face face = after.GetFace(i).GetFace();
            var surface = new FaceSurfaceAdaptor(face);
            if (surface.GetType() is not (SurfaceKind.Sphere or SurfaceKind.Torus))
            {
                continue;
            }

            Triangulation triangulation = Tool.Triangulation(face, out Placement location)!;
            Assert.NotNull(triangulation);
            double target = before.GetFace(i).GetDeflection();
            double worst = Worst(surface, triangulation, location.Transformation);
            Assert.True(worst <= target * 1.001, $"face {i} ({surface.GetType()}): {worst:G4} from its surface for {target:G4}");
            measured++;
        }

        Assert.True(measured > 0);
    }

    // The largest distance from the surface over each triangle's 12 points on a grid of quarters (its corners left out) and
    // its centroid, exactly for a sphere and a torus
    private static double Worst(FaceSurfaceAdaptor surface, Triangulation triangulation, Transform3D transform)
    {
        Func<Point3D, double> distance;
        if (surface.GetType() == SurfaceKind.Sphere)
        {
            Sphere3D sphere = surface.Sphere();
            distance = p => Math.Abs(sphere.Location.DistanceTo(p) - sphere.Radius);
        }
        else
        {
            Torus3D torus = surface.Torus();
            Point3D o = torus.Location;
            Direction3D z = torus.Axis.Direction;
            distance = p =>
            {
                double x = p.X - o.X, y = p.Y - o.Y, w = p.Z - o.Z;
                double h = x * z.X + y * z.Y + w * z.Z;
                double rx = x - h * z.X, ry = y - h * z.Y, rz = w - h * z.Z;
                double rho = Math.Sqrt(rx * rx + ry * ry + rz * rz) - torus.MajorRadius;
                return Math.Abs(Math.Sqrt(rho * rho + h * h) - torus.MinorRadius);
            };
        }

        double worst = 0;
        const int n = 4;
        for (int t = 1; t <= triangulation.TriangleCount; t++)
        {
            triangulation.Triangle(t).Get(out int ia, out int ib, out int ic);
            Point3D a = triangulation.Node(ia).Transformed(transform), b = triangulation.Node(ib).Transformed(transform),
                    c = triangulation.Node(ic).Transformed(transform);
            for (int i = 0; i <= n; i++)
            {
                for (int j = 0; i + j <= n; j++)
                {
                    int k = n - i - j;
                    if (i < n && j < n && k < n)
                    {
                        worst = Math.Max(worst, distance(new Point3D((i * a.X + j * b.X + k * c.X) / n, (i * a.Y + j * b.Y + k * c.Y) / n,
                                                                     (i * a.Z + j * b.Z + k * c.Z) / n)));
                    }
                }
            }

            worst = Math.Max(worst, distance(new Point3D((a.X + b.X + c.X) / 3, (a.Y + b.Y + c.Y) / 3, (a.Z + b.Z + c.Z) / 3)));
        }

        return worst;
    }
}
