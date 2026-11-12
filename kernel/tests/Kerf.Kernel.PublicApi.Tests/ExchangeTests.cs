// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using Kerf.Exchange;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Meshing;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>Kerf.Exchange (docs/api/exchange.md): STEP, STL, glTF, 3MF, OBJ and BREP, ModelNode and Kerf.Color.</summary>
public class ExchangeTests
{
    [Fact]
    public void BREP_round_trips_as_text_and_binary()
    {
        Solid box = Fixtures.Box();
        foreach (bool binary in new[] { false, true })
        {
            var stream = new MemoryStream();
            Assert.True(Brep.Write(box, stream, new BrepWriteOptions { Binary = binary }).IsSuccess);
            stream.Position = 0;
            Result<Shape> read = Brep.Read(stream);
            Assert.True(read.IsSuccess, read.Error?.Message);
            Assert.Equal(6000, read.Value.IntegrateVolume(), 1e-6);
        }

        // The text written is the text read
        var text = new MemoryStream();
        Brep.Write(box, text, new BrepWriteOptions { WithTriangles = false });
        Assert.Equal(Encoding.ASCII.GetString(Fixtures.Bytes("box")), Encoding.ASCII.GetString(text.ToArray()));
    }

    [Fact]
    public void Bad_BREP_is_a_failure()
    {
        Result<Shape> garbage = Brep.Read(new MemoryStream("not a shape at all"u8.ToArray()));
        Assert.False(garbage.IsSuccess);
        Assert.Equal("exchange.malformed", garbage.Error!.Code.Value);
        Result<Shape> large = Brep.Read(new MemoryStream(Fixtures.Bytes("box")), new ReadLimits { MaxBytes = 100 });
        Assert.Equal("exchange.limit-exceeded", large.Error!.Code.Value);
    }

    [Fact]
    public void STEP_is_written_in_the_schema_asked_and_read_with_or_without_healing()
    {
        foreach ((StepSchema schema, string protocol) in new[] { (StepSchema.Ap214, "AUTOMOTIVE_DESIGN"), (StepSchema.Ap203, "CONFIG_CONTROL_DESIGN"),
                                                               (StepSchema.Ap242, "AP242_MANAGED_MODEL_BASED_3D_ENGINEERING") })
        {
            var stream = new MemoryStream();
            Assert.True(Step.Write(Fixtures.Box(), stream, new StepWriteOptions { Schema = schema }).IsSuccess);
            Assert.Contains(protocol, Encoding.ASCII.GetString(stream.ToArray()));
            foreach (bool heal in new[] { true, false })
            {
                stream.Position = 0;
                Result<ModelNode> read = Step.Read(stream, new StepReadOptions { Heal = heal });
                Assert.True(read.IsSuccess, read.Error?.Message);
                Assert.Equal(6000, read.Value.Shape!.Value.IntegrateVolume(), 1e-6);
            }
        }
    }

    /// <summary>
    /// OCCT's issue 382 (decision 33101): a band of a cylinder between a circle and a loop with a step in it, as a STEP
    /// file gives it, with no seam. The reader puts the seam in along the step's line; where the arc that leaves the
    /// step begins a few billionths of a radian before that line, the seam was never made and the band came back as two
    /// invalid faces, a success.
    /// </summary>
    [Theory]
    [InlineData(-3e-9)]
    [InlineData(-2e-8)]
    [InlineData(0.0)]
    public void STEP_band_of_a_cylinder_whose_lower_rim_has_a_step_is_read_as_one_valid_face(double offset)
    {
        double a1 = 39 * Math.PI / 180, a2 = 141 * Math.PI / 180;
        string At(double angle, double z) => FormattableString.Invariant($"({Math.Cos(angle):R},{Math.Sin(angle):R},{z:R})");
        string step = string.Join('\n',
            "ISO-10303-21;", "HEADER;", "FILE_DESCRIPTION((''),'2;1');", "FILE_NAME('','',(''),(''),'','','');",
            "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 3 1 1 1 }'));", "ENDSEC;", "DATA;",
            $"#1=CARTESIAN_POINT('',{At(a1, 0)});", $"#2=CARTESIAN_POINT('',{At(a1 + offset, 0.2)});",
            $"#3=CARTESIAN_POINT('',{At(a2, 0.2)});", $"#4=CARTESIAN_POINT('',{At(a2, 0)});", "#5=CARTESIAN_POINT('',(-1.,0.,2.));",
            "#11=VERTEX_POINT('',#1);", "#12=VERTEX_POINT('',#2);", "#13=VERTEX_POINT('',#3);", "#14=VERTEX_POINT('',#4);", "#15=VERTEX_POINT('',#5);",
            "#20=DIRECTION('',(0.,0.,1.));", "#21=DIRECTION('',(1.,0.,0.));",
            "#22=CARTESIAN_POINT('',(0.,0.,0.));", "#23=CARTESIAN_POINT('',(0.,0.,0.2));", "#24=CARTESIAN_POINT('',(0.,0.,2.));",
            "#25=AXIS2_PLACEMENT_3D('',#22,#20,#21);", "#26=AXIS2_PLACEMENT_3D('',#23,#20,#21);", "#27=AXIS2_PLACEMENT_3D('',#24,#20,#21);",
            "#30=CIRCLE('',#25,1.);", "#31=CIRCLE('',#26,1.);", "#32=CIRCLE('',#27,1.);",
            "#33=VECTOR('',#20,1.);", "#34=LINE('',#1,#33);", "#35=LINE('',#4,#33);", "#36=CYLINDRICAL_SURFACE('',#25,1.);",
            "#40=EDGE_CURVE('',#14,#11,#30,.T.);", "#41=EDGE_CURVE('',#11,#12,#34,.T.);", "#42=EDGE_CURVE('',#12,#13,#31,.T.);",
            "#43=EDGE_CURVE('',#14,#13,#35,.T.);", "#44=EDGE_CURVE('',#15,#15,#32,.T.);",
            "#50=ORIENTED_EDGE('',*,*,#40,.T.);", "#51=ORIENTED_EDGE('',*,*,#41,.T.);", "#52=ORIENTED_EDGE('',*,*,#42,.T.);",
            "#53=ORIENTED_EDGE('',*,*,#43,.F.);", "#54=ORIENTED_EDGE('',*,*,#44,.F.);",
            "#60=EDGE_LOOP('',(#50,#51,#52,#53));", "#61=EDGE_LOOP('',(#54));", "#62=FACE_BOUND('',#60,.T.);", "#63=FACE_BOUND('',#61,.T.);",
            "#64=ADVANCED_FACE('',(#62,#63),#36,.T.);", "#65=OPEN_SHELL('',(#64));", "#66=SHELL_BASED_SURFACE_MODEL('',(#65));",
            "#70=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));", "#71=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));",
            "#72=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());",
            "#73=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#70,'distance_accuracy_value','');",
            "#74=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#73))GLOBAL_UNIT_ASSIGNED_CONTEXT((#70,#71,#72))REPRESENTATION_CONTEXT('',''));",
            "#75=MANIFOLD_SURFACE_SHAPE_REPRESENTATION('',(#66),#74);",
            "#80=APPLICATION_CONTEXT('automotive design');", "#81=PRODUCT_CONTEXT('',#80,'mechanical');", "#82=PRODUCT('band','band','',(#81));",
            "#83=PRODUCT_DEFINITION_FORMATION('','',#82);", "#84=PRODUCT_DEFINITION_CONTEXT('part definition',#80,'design');",
            "#85=PRODUCT_DEFINITION('','',#83,#84);", "#86=PRODUCT_DEFINITION_SHAPE('','',#85);", "#87=SHAPE_DEFINITION_REPRESENTATION(#86,#75);",
            "ENDSEC;", "END-ISO-10303-21;", "");

        Result<ModelNode> read = Step.Read(new MemoryStream(Encoding.ASCII.GetBytes(step)));

        Assert.True(read.IsSuccess, read.Error?.Message);
        Shape band = read.Value.ToShape();
        Assert.Single(band.AllFaces());
        Assert.True(band.Validate().IsValid);
        // the long arc, 258 degrees, is 2 below the circle, and the short one, 102 degrees, 1.8 below
        Assert.Equal((258 * 2 + 102 * 1.8) * Math.PI / 180, band.IntegrateArea(), 1e-6);
    }

    /// <summary>
    /// OCCT's issue 349 (decision 33102): a strip of plane 0.001 wide written as two lines between the same two vertices,
    /// the vertices midway between them. OCCT moves a line that misses its vertices onto them; both lines moved so
    /// became one, and the strip a face of no area whose wire lay on itself, a success.
    /// </summary>
    [Fact]
    public void STEP_strip_between_two_lines_that_share_their_vertices_keeps_its_width()
    {
        Result<ModelNode> read = Step.Read(new MemoryStream(Encoding.ASCII.GetBytes(StripStep("#1"))));

        Assert.True(read.IsSuccess, read.Error?.Message);
        Assert.Empty(read.Diagnostics);
        Shape strip = read.Value.ToShape();
        Assert.Single(strip.AllFaces());
        Assert.True(strip.Validate().IsValid);
        // The width from the mesh: the area's integral reads the ends, which the vertices cover, as missing until
        // decision 2257 is on this branch
        ShapeMesh mesh = strip.Mesh().Value;
        Assert.Equal(2, mesh.TriangleCount);
        float[] across = [.. Enumerable.Range(0, mesh.NodeCount).Select(node => mesh.Positions.Span[3 * node + 1])];
        Assert.Equal(0.001, across.Max() - across.Min(), 1e-6);
    }

    /// <summary>
    /// OCCT's issue 1578 (decision 33104): a reference to an entity that isn't in the file was bound to whatever the file
    /// has at the record of that number, here another point, and the model read as if nothing were wrong. It is unresolved:
    /// what refers to it is left out, and the read says which entities.
    /// </summary>
    [Fact]
    public void STEP_reference_to_an_entity_that_isnt_there_is_not_bound_to_another_and_is_reported()
    {
        // no entity has the identifier 15; the fifteenth record of the file is the point #4
        Result<ModelNode> read = Step.Read(new MemoryStream(Encoding.ASCII.GetBytes(StripStep("#15"))));

        Assert.True(read.IsSuccess, read.Error?.Message);
        Diagnostic skipped = Assert.Single(read.Diagnostics);
        Assert.Equal("exchange.entity-skipped", skipped.Code.Value);
        Assert.Equal(Severity.Warning, skipped.Severity);
        Assert.Contains("#11", skipped.Message);
        Assert.Empty(read.Value.ToShape().AllFaces());
    }

    /// <summary>A strip of plane 0.001 wide and 1 long, its first vertex at the point <paramref name="firstVertexPoint"/>.</summary>
    private static string StripStep(string firstVertexPoint)
    {
        return string.Join('\n',
            "ISO-10303-21;", "HEADER;", "FILE_DESCRIPTION((''),'2;1');", "FILE_NAME('','',(''),(''),'','','');",
            "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 3 1 1 1 }'));", "ENDSEC;", "DATA;",
            "#1=CARTESIAN_POINT('',(0.,0.,0.));", "#2=CARTESIAN_POINT('',(1.,0.,0.));",
            "#3=CARTESIAN_POINT('',(0.,-0.0005,0.));", "#4=CARTESIAN_POINT('',(0.,0.0005,0.));",
            $"#11=VERTEX_POINT('',{firstVertexPoint});", "#12=VERTEX_POINT('',#2);",
            "#20=DIRECTION('',(0.,0.,1.));", "#21=DIRECTION('',(1.,0.,0.));", "#25=AXIS2_PLACEMENT_3D('',#1,#20,#21);",
            "#33=VECTOR('',#21,1.);", "#34=LINE('',#3,#33);", "#35=LINE('',#4,#33);", "#36=PLANE('',#25);",
            "#40=EDGE_CURVE('',#11,#12,#34,.T.);", "#41=EDGE_CURVE('',#11,#12,#35,.T.);",
            "#50=ORIENTED_EDGE('',*,*,#40,.T.);", "#51=ORIENTED_EDGE('',*,*,#41,.F.);",
            "#60=EDGE_LOOP('',(#50,#51));", "#62=FACE_OUTER_BOUND('',#60,.T.);",
            "#64=ADVANCED_FACE('',(#62),#36,.T.);", "#65=OPEN_SHELL('',(#64));", "#66=SHELL_BASED_SURFACE_MODEL('',(#65));",
            "#70=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));", "#71=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));",
            "#72=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());",
            "#73=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-05),#70,'distance_accuracy_value','');",
            "#74=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#73))GLOBAL_UNIT_ASSIGNED_CONTEXT((#70,#71,#72))REPRESENTATION_CONTEXT('',''));",
            "#75=MANIFOLD_SURFACE_SHAPE_REPRESENTATION('',(#66),#74);",
            "#80=APPLICATION_CONTEXT('automotive design');", "#81=PRODUCT_CONTEXT('',#80,'mechanical');", "#82=PRODUCT('strip','strip','',(#81));",
            "#83=PRODUCT_DEFINITION_FORMATION('','',#82);", "#84=PRODUCT_DEFINITION_CONTEXT('part definition',#80,'design');",
            "#85=PRODUCT_DEFINITION('','',#83,#84);", "#86=PRODUCT_DEFINITION_SHAPE('','',#85);", "#87=SHAPE_DEFINITION_REPRESENTATION(#86,#75);",
            "ENDSEC;", "END-ISO-10303-21;", "");
    }

    [Fact]
    public void STEP_keeps_names_colours_and_the_assembly()
    {
        var orange = new Color(1, 0.5, 0);
        ModelNode model = ModelNode.Assembly([ModelNode.Part(Fixtures.Box(), "Body", orange), ModelNode.Part(Fixtures.Cylinder(), "Lid")], "Box");
        var stream = new MemoryStream();
        Assert.True(Step.Write(model, stream).IsSuccess);
        Assert.StartsWith("ISO-10303-21;", Encoding.ASCII.GetString(stream.ToArray()));

        stream.Position = 0;
        Result<ModelNode> read = Step.Read(stream);
        Assert.True(read.IsSuccess);
        ModelNode root = read.Value;
        Assert.Equal("Box", root.Name);
        Assert.Equal(2, root.Children.Length);
        ModelNode body = root.Children.Single(c => c.Name == "Body");
        Assert.Equal(orange, body.Color);
        Assert.Equal(6000, body.Shape!.Value.IntegrateVolume(), 1e-3);
        Assert.Equal(6000 + Math.PI * 250, root.ToShape().IntegrateVolume(), 1e-3);
    }

    [Fact]
    public void STEP_of_a_shape_in_inches()
    {
        var stream = new MemoryStream();
        Assert.True(Step.Write(Fixtures.Box(), stream, new StepWriteOptions { Unit = LengthUnit.Inch, Precision = StepPrecision.Greatest }).IsSuccess);
        Assert.Contains("INCH", Encoding.ASCII.GetString(stream.ToArray()));
    }

    [Fact]
    public void Bad_STEP_is_a_failure()
    {
        Result<ModelNode> garbage = Step.Read(new MemoryStream("ISO-10303-21;\nHEADER;\ngarbage"u8.ToArray()));
        Assert.False(garbage.IsSuccess);
        Assert.NotNull(garbage.Error);
        using var cancelled = new CancellationTokenSource();
        cancelled.Cancel();
        Assert.ThrowsAny<OperationCanceledException>(() => Step.Read(new MemoryStream(), cancellationToken: cancelled.Token));
    }

    [Fact]
    public void STL_writes_a_mesh_and_reads_one()
    {
        ShapeMesh mesh = Fixtures.Box().Mesh().Value;
        var binary = new MemoryStream();
        Assert.True(Stl.Write(mesh, binary).IsSuccess);
        Assert.Equal(84 + 50 * 12, binary.Length);
        var ascii = new MemoryStream();
        Assert.True(Stl.Write(mesh, ascii, new StlWriteOptions { Ascii = true }).IsSuccess);
        Assert.StartsWith("solid", Encoding.ASCII.GetString(ascii.ToArray()));

        binary.Position = 0;
        Result<ShapeMesh> read = Stl.Read(binary, new StlReadOptions { Unit = LengthUnit.Centimetre });
        Assert.True(read.IsSuccess);
        Assert.True(read.Value.Shape.IsNull);
        Assert.Equal(12, read.Value.TriangleCount);
        Assert.Equal(8, read.Value.NodeCount);   // the coincident nodes merged
        Assert.Equal(300, read.Value.Positions.ToArray().Max(), 1e-3);   // centimetres scaled to millimetres
        Assert.False(Stl.Read(new MemoryStream("solid x\nfacet garbage"u8.ToArray())).IsSuccess);
    }

    [Fact]
    public void GlTF_writes_and_reads_a_model_of_meshes()
    {
        ModelNode model = ModelNode.Assembly([ModelNode.Part(Fixtures.Box(), "Body", new Color(0, 0, 1)), ModelNode.Part(Fixtures.Cylinder(), "Lid")]);
        var glb = new MemoryStream();
        Assert.True(Gltf.Write(model, glb).IsSuccess);
        Assert.Equal("glTF", Encoding.ASCII.GetString(glb.ToArray(), 0, 4));
        var json = new MemoryStream();
        Assert.True(Gltf.Write(model, json, new GltfWriteOptions { Binary = false }).IsSuccess);
        Assert.Contains("data:application/octet-stream;base64,", Encoding.UTF8.GetString(json.ToArray()));
        var separate = new MemoryStream();
        var bin = new MemoryStream();
        Assert.True(Gltf.Write(model, separate, bin, "model.bin").IsSuccess);
        Assert.True(bin.Length > 0);

        glb.Position = 0;
        Result<ModelNode> read = Gltf.Read(glb);
        Assert.True(read.IsSuccess);
        var parts = Parts(read.Value).ToList();
        Assert.Equal(2, parts.Count);
        Assert.All(parts, p => Assert.NotNull(p.Mesh));
        Assert.Contains(parts, p => p.Mesh!.TriangleCount == 12);

        // A mesh part is written as it is
        var again = new MemoryStream();
        Assert.True(Gltf.Write(ModelNode.Part(parts[0].Mesh!), again).IsSuccess);
        Assert.False(Gltf.Read(new MemoryStream("{ not json"u8.ToArray())).IsSuccess);
    }

    [Fact]
    public void ThreeMF_and_OBJ_are_written()
    {
        ModelNode model = ModelNode.Part(Fixtures.Box(), "Body", new Color(0.2, 0.4, 0.6));
        var package = new MemoryStream();
        Assert.True(ThreeMf.Write(model, package, new ThreeMfWriteOptions { Unit = LengthUnit.Millimetre }).IsSuccess);
        Assert.Equal("PK", Encoding.ASCII.GetString(package.ToArray(), 0, 2));
        var obj = new MemoryStream();
        var mtl = new MemoryStream();
        Assert.True(Obj.Write(model, obj, mtl, "body.mtl").IsSuccess);
        string text = Encoding.ASCII.GetString(obj.ToArray());
        Assert.Contains("mtllib body.mtl", text);
        Assert.Contains("\nv ", text);
        Assert.True(mtl.Length > 0);
    }

    [Fact]
    public void Colours_are_sRGB_and_convert_to_linear()
    {
        var grey = new Color(0.5, 0.5, 0.5);
        Assert.Equal(0.5, grey.R, 1e-6);
        Assert.Equal(1, grey.A);
        Assert.Equal(0.214, grey.ToLinear().R, 1e-3);
        Color linear = Color.FromLinear(0.214041, 0.214041, 0.214041, 0.5);
        Assert.Equal(0.5, linear.R, 1e-4);
        Assert.Equal((float)0.214041, (float)linear.ToLinear().R);
        Assert.Equal(0.5, linear.A);
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => new Color(1.5, 0, 0));
        KernelAssert.Throws<ArgumentOutOfRangeException>(() => Color.FromLinear(0, 0, double.NaN));
        Assert.NotEqual(new Color(1, 0, 0), new Color(1, 0, 0, 0.5));
    }

    [Fact]
    public void A_model_node_is_a_part_or_an_assembly()
    {
        ModelNode part = ModelNode.Part(Fixtures.Box(), "Body");
        Assert.True(part.IsPart);
        Assert.Equal(6000, part.ToShape().IntegrateVolume(), 1e-6);
        var placement = new Placement(Kerf.Geometry.Transform3D.Translation(new Kerf.Geometry.Vector3D(100, 0, 0)));
        ModelNode assembly = ModelNode.Assembly([part, part], "Pair", placement);
        Assert.False(assembly.IsPart);
        Shape all = assembly.ToShape();
        Assert.Equal(12000, all.IntegrateVolume(), 1e-6);
        Assert.Equal(100, all.BoundingBox(optimal: true).Min.X, 1e-6);
        KernelAssert.Throws<ArgumentException>(() => ModelNode.Part(default(Shape)));
        Assert.Equal(1L << 30, ReadLimits.Default.MaxBytes);
        Assert.Equal(25.4, LengthUnit.Inch.Millimetres());
    }

    private static IEnumerable<ModelNode> Parts(ModelNode node)
        => node.IsPart ? [node] : node.Children.SelectMany(Parts);
}
