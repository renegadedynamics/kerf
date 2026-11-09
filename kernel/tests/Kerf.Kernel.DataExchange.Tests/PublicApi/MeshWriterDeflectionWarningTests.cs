// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Exchange;
using Kerf.Geometry;
using Kerf.Meshing;
using Kerf.Topology;
using Xunit;

namespace Kerf.Kernel.DataExchange.Tests.PublicApi;

/// <summary>
/// The writers that mesh a model (glTF, 3MF, OBJ) and a model's scene pass on <c>meshing.deflection-exceeded</c> for a part
/// whose faces are meshed further from their surfaces than the deflection, naming the part (decision 26901); the bytes
/// written are the ones written without it.
/// </summary>
public class MeshWriterDeflectionWarningTests
{
    // A cylinder of radius 10 and height 40, its top edge filleted at 0.3: at a relative deflection of 0.005 and an angle
    // of 0.5 its fillet is 1.005 times its deflection by its guard's measure
    private static Shape FilletedCylinder()
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

    private static readonly MeshOptions Over = new() { LinearDeflection = 0.005, AngularDeflection = 0.5, Relative = true };
    private static readonly MeshOptions Within = new() { LinearDeflection = 0.01, AngularDeflection = 0.5, Relative = true };

    public static TheoryData<string> Writers => ["glb", "gltf", "3mf", "obj"];

    private static Result Write(string writer, ModelNode model, MeshOptions mesh, Stream stream) => writer switch
    {
        "glb" => Gltf.Write(model, stream, new GltfWriteOptions { Mesh = mesh }),
        "gltf" => Gltf.Write(model, stream, new GltfWriteOptions { Mesh = mesh, Binary = false }),
        "3mf" => ThreeMf.Write(model, stream, new ThreeMfWriteOptions { Mesh = mesh }),
        _ => Obj.Write(model, stream, options: new ObjWriteOptions { Mesh = mesh }),
    };

    [Theory]
    [MemberData(nameof(Writers))]
    public void A_writer_passes_the_warning_on_naming_the_part(string writer)
    {
        Shape part = FilletedCylinder();
        ModelNode model = ModelNode.Assembly([ModelNode.Part(part, "knob"), ModelNode.Part(new MakeBox(10, 10, 10).Shape(), "block")]);
        Result written = Write(writer, model, Over, new MemoryStream());

        Assert.True(written.IsSuccess);
        Diagnostic warning = Assert.Single(written.Diagnostics);
        Assert.Equal(Mesher.DeflectionExceeded, warning.Code);
        Assert.StartsWith("Face 2 of knob is further from its surface than the deflection: 0.05438 from it, 1.005 times", warning.Message);
        Assert.True(Assert.Single(warning.Shapes).IsSame(MeshExtraction.Faces(part)[2]));

        Assert.Empty(Write(writer, model, Within, new MemoryStream()).Diagnostics);
    }

    [Fact]
    public void A_models_scene_passes_the_warning_on()
    {
        Result<Drawing.Scene> scene = ModelNode.Part(FilletedCylinder(), "knob").ToScene(Over);
        Assert.True(scene.IsSuccess);
        Assert.Equal(Mesher.DeflectionExceeded, Assert.Single(scene.Diagnostics).Code);
    }
}
