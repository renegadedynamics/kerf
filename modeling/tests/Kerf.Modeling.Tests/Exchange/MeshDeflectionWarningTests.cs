using Ops = Kerf.Modeling.Operations;

namespace Kerf.Modeling.Tests.Exchange;

using Kerf.Modeling;

/// <summary>
/// Where the layer meshes, it passes on the kernel's <c>meshing.deflection-exceeded</c> as it does other warnings: the
/// exporters in their results, <c>Tessellate</c> and <c>Triangulate</c> (and so a <c>MeshNode</c>) to <see cref="Warnings"/> (the kernel's decision 2071).
/// </summary>
public class MeshDeflectionWarningTests
{
    // A cylinder of radius 10 and height 40, its top edge filleted at 0.3: at a relative deflection of 0.005 and an angle
    // of 0.5 the fillet, a torus, is 1.005 times its deflection by its guard's measure
    private static Part FilletedCylinder()
    {
        var cylinder = new Cylinder(10, 40);
        return (Part)Ops.Fillet(cylinder.Edges().Where(GeomType.Circle).GroupBy(Axis.Z).Last(), 0.3).Value;
    }

    private const string Code = "meshing.deflection-exceeded";

    public static TheoryData<string> Exporters => ["stl", "glb", "gltf", "3mf"];

    private static Result Export(string format, IShape shape, double deflection, Stream stream) => format switch
    {
        "stl" => Exporters3D.ExportStl(shape, stream, new StlExportOptions { LinearDeflection = deflection, AngularDeflection = 0.5 }),
        "glb" => Exporters3D.ExportGltf(shape, stream, new GltfExportOptions { LinearDeflection = deflection, AngularDeflection = 0.5 }),
        "gltf" => Exporters3D.ExportGltf(shape, stream, new GltfExportOptions { LinearDeflection = deflection, AngularDeflection = 0.5, Binary = false }),
        _ => Exporters3D.Export3mf(shape, stream, new ThreeMfExportOptions { LinearDeflection = deflection, AngularDeflection = 0.5 }),
    };

    [Theory]
    [MemberData(nameof(Exporters))]
    public void AnExporterThatMeshesPassesTheWarningOn(string format)
    {
        var part = FilletedCylinder();
        var over = new MemoryStream();
        var written = Export(format, part, 0.005, over);
        Assert.True(written.IsSuccess);
        var warning = Assert.Single(written.Diagnostics);
        Assert.Equal(Code, warning.Code.Value);
        Assert.Contains("further from its surface than the deflection: 0.05438 from it, 1.005 times the deflection of 0.05412", warning.Message);

        // Within the deflection: nothing said
        Assert.Empty(Export(format, part, 0.01, new MemoryStream()).Diagnostics);
    }

    [Fact]
    public void ExportingToAFilePassesTheWarningOn()
    {
        var path = Path.Combine(Path.GetTempPath(), $"kerf-13s-{Guid.NewGuid():N}.stl");
        try
        {
            var written = Exporters3D.ExportStl(FilletedCylinder(), path, new StlExportOptions { LinearDeflection = 0.005, AngularDeflection = 0.5 });
            Assert.Equal(Code, Assert.Single(written.Diagnostics).Code.Value);
        }
        finally
        {
            File.Delete(path);
        }
    }

    [Fact]
    public void TessellateRaisesTheWarning()
    {
        var part = FilletedCylinder();
        using var warnings = Warnings.Capture();
        part.Tessellate(0.005, 0.5);
        var raised = Assert.Single(warnings.Raised);
        Assert.Equal(Code, raised.Code?.Value);
        Assert.Contains("1.005 times", raised.Message);
    }

    [Fact]
    public void TriangulateRaisesTheWarning()
    {
        var part = FilletedCylinder();
        using var warnings = Warnings.Capture();
        part.Triangulate(0.005, 0.5, relative: true);
        Assert.Equal(Code, Assert.Single(warnings.Raised).Code?.Value);
    }

    [Fact]
    public void AMeshWithinItsDeflectionRaisesNothing()
    {
        var part = FilletedCylinder();
        using var warnings = Warnings.Capture();
        part.Tessellate(0.01, 0.5);
        part.Triangulate(0.01, 0.5, relative: true);
        part.Triangulate(0.05, 0.5);
        Assert.Empty(warnings.Messages);
    }
}
