// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Meshing;
using Kerf.Topology;
using Xunit;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary><see cref="MeshOptions.ControlAllSurfaces"/> on the shape that shows it: the bottle's long fillet tori (decision 1680).</summary>
public class MeshOptionsControlAllSurfacesTests
{
    [Fact]
    public void ALongFilletKeepsToTheDeflectionAskedWithOrWithoutTheControl()
    {
        // The bottle's fillets are tori, whose grid OCCT trusts; ControlAllSurfaces once added a fifth more triangles for them. Since
        // decision 22604 a torus is controlled against the deflection asked anyway, so the control has nothing to add here
        Shape bottle = BRepMeshModelKit.ReadDataOcc("bottle.brep");
        var options = new MeshOptions { LinearDeflection = 0.094, AngularDeflection = 0.2 };
        Assert.False(options.ControlAllSurfaces);

        ShapeMesh plain = Mesher.Mesh(bottle, options).Value;
        int controlled = Mesher.Mesh(bottle, options with { ControlAllSurfaces = true }).Value.TriangleCount;
        Assert.Equal(controlled, plain.TriangleCount);

        // Each torus's triangles keep within the deflection, measured from the surface at their centres and edges' middles
        foreach (FaceMesh face in plain.Faces.Where(static f => f.Kind == SurfaceKind.Torus))
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
                    Assert.InRange(face.Face.ParametersOf(sample, 1e-6).Gap, 0, 0.094 * 1.01);
                }
            }
        }
    }
}

