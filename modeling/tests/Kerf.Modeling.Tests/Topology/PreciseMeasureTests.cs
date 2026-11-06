namespace Kerf.Modeling.Tests.Topology;

using Kerf.Measurement;
using Kerf.Meshing;
using Kerf.Modeling;
using Kerf.Modeling.Tests.Examples;
using static Kerf.Modeling.Operations;

/// <summary>Area and volume are on the precise path: OCCT's default is off on swept and B-spline faces (decision 1802).</summary>
public class PreciseMeasureTests
{
    private static (double Volume, double Area) OfMesh(Shape shape, double deflection)
    {
        double volume = 0, area = 0;
        foreach (var face in shape.Mesh(new MeshOptions { LinearDeflection = deflection, AngularDeflection = 0.05, Parallel = false }).Value.Faces)
        {
            var nodes = face.Positions.Span;
            var triangles = face.Triangles.Span;
            for (int i = 0; i + 2 < triangles.Length; i += 3)
            {
                Vector3D a = Node(nodes, triangles[i]), b = Node(nodes, triangles[i + 1]), c = Node(nodes, triangles[i + 2]);
                volume += a.Dot(b.Cross(c)) / 6;
                area += (b - a).Cross(c - a).Magnitude / 2;
            }
        }
        return (volume, area);
    }

    private static Vector3D Node(ReadOnlySpan<float> nodes, int i) => new(nodes[3 * i], nodes[3 * i + 1], nodes[3 * i + 2]);

    [Fact]
    public void A_fine_mesh_sides_with_the_precise_path()
    {
        Shape example = IntroductoryExamplesTests.Build(12);
        var mesh = OfMesh(example, 0.002);
        Assert.True(Math.Abs(example.Volume - mesh.Volume) < Math.Abs(example.IntegrateVolume() - mesh.Volume) / 2);
        Assert.True(Math.Abs(example.Area - mesh.Area) < Math.Abs(example.IntegrateArea() - mesh.Area) / 2);
        Assert.Equal(mesh.Area, example.Area, 1e-5 * mesh.Area);
    }

    [Theory]
    [InlineData(0.01, 2e-3)]   // the sweep's own tolerance of 1e-4 is a tenth of this section's radius: the surface is that far out
    [InlineData(10, 1e-5)]
    [InlineData(10000, 1e-5)]
    public void A_part_of_any_size_gets_its_known_volume(double size, double within)
    {
        // Four turns of a helix: a tube of radius size / 10, by the section's area and the way round it travels
        Shape swept = Sweep(new Plane(new Point3D(size, 0, 0), zDir: new Vector3D(0, 1, 0)) * new Circle(size / 10),
                            new Helix(size / 2, size * 2, size), new SweepOptions { IsFrenet = true }).Value;
        double expected = Math.PI * Math.Pow(size / 10, 2) * (4 * 2 * Math.PI * size);
        Assert.Equal(expected, swept.Volume, expected * within);
        // And the integration itself is as close at every size
        double tighter = swept.IntegrateVolume(new MassOptions { Tolerance = 1e-9 });
        Assert.Equal(tighter, swept.Volume, tighter * 1e-5);
    }

    [Fact]
    public void Analytic_parts_keep_their_exact_figures()
    {
        Assert.Equal(48000, new Box(80, 60, 10).Volume, 1e-8);
        Assert.Equal(Math.PI * 4 * 5, new Cylinder(2, 5).Volume, 1e-9);
        Assert.Equal(4 * Math.PI * 4, new Sphere(2).Area, 1e-9);
    }

    [Fact]
    public void Moments_keep_to_their_own_precision()
    {
        // At the precision of a volume, a cylinder's two equal moments differ by 8e-6 of them
        var moments = new Cylinder(2, 5).PrincipalProperties.Select(p => p.Moment).OrderBy(m => m).ToList();
        Assert.Equal(moments[1], moments[2], 1e-9);
    }
}
