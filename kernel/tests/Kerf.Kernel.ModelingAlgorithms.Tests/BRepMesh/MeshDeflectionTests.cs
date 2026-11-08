// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Meshing;
using Kerf.Topology;
using Xunit;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The mesh keeps to the deflection asked, with no option for it, on the shapes that missed it: the bottle's long fillet tori
/// (decisions 1680, 1929) and <c>Ball.brep</c>'s spheres (decision 1953). <c>ControlAllSurfaces</c> went (decision 1954).
/// </summary>
public class MeshDeflectionTests
{
    [Theory]
    [InlineData("bottle.brep", 0.094, 0.2)]   // 3.7 times out at decision 1680, on its fillets' tori
    [InlineData("Ball.brep", 0.042, 0.2)]     // twice out, on its spheres, before decision 1953
    public void Every_curved_face_keeps_to_the_deflection_asked(string file, double deflection, double angle)
    {
        Shape shape = BRepMeshModelKit.ReadDataOcc(file);
        ShapeMesh mesh = Mesher.Mesh(shape, new MeshOptions { LinearDeflection = deflection, AngularDeflection = angle }).Value;

        // Each triangle's centre and edges' middles, measured from the surface
        foreach (FaceMesh face in mesh.Faces.Where(static f => f.Kind is SurfaceKind.Torus or SurfaceKind.Sphere or SurfaceKind.Cylinder or SurfaceKind.Cone))
        {
            Point3D[] nodes = face.GetNodes();
            foreach (Triangle t in face.GetTriangles())
            {
                Point3D a = nodes[t.A], b = nodes[t.B], c = nodes[t.C];
                foreach (Point3D sample in new[] { new Point3D((a.X + b.X + c.X) / 3, (a.Y + b.Y + c.Y) / 3, (a.Z + b.Z + c.Z) / 3),
                                                   new Point3D((a.X + b.X) / 2, (a.Y + b.Y) / 2, (a.Z + b.Z) / 2),
                                                   new Point3D((b.X + c.X) / 2, (b.Y + c.Y) / 2, (b.Z + c.Z) / 2),
                                                   new Point3D((a.X + c.X) / 2, (a.Y + c.Y) / 2, (a.Z + c.Z) / 2) })
                {
                    Assert.InRange(face.Face.ParametersOf(sample, 1e-6).Gap, 0, deflection * 1.01);
                }
            }
        }
    }
}
