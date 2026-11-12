// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using Kerf.Engine.Healing.Fixes;
using Kerf.Exchange;
using Kerf.Healing;
using Kerf.Topology;

namespace Kerf.Kernel.DataExchange.Tests.STEPControl;

/// <summary>
/// The STEP read's check of the faces it may have read wrongly (decision 33502; the owner's ruling, decision 33501): the
/// faces the healing reported on and those with a vertex off its edge's curve are checked, and an invalid one is a
/// warning, <c>exchange.invalid-face</c>, on a read that still succeeds. The invalid faces are 15E9's, made again by
/// reading with its two fixes left out (<see cref="UndoneFixes"/>): #382's band of a cylinder, which came back as two
/// invalid faces, and #349's strip, which came back as a face of no area.
/// </summary>
public class StepReadCheckTests
{
    [Theory]
    [InlineData(-3e-9)]
    [InlineData(-5e-8)]
    public void A_band_read_as_two_invalid_faces_is_a_success_with_a_warning_naming_its_face(double offset)
    {
        byte[] step = Encoding.ASCII.GetBytes(BandStep(offset));
        Result<ModelNode> read;
        using (UndoneFixes.Scope())
        {
            read = Step.Read(new MemoryStream(step));
        }

        Assert.True(read.IsSuccess, read.Error?.Message);
        Diagnostic warning = Assert.Single(read.Diagnostics);
        Assert.Equal(Step.InvalidFace, warning.Code);
        Assert.Equal("exchange.invalid-face", warning.Code.Value);
        Assert.Equal(Severity.Warning, warning.Severity);
        Assert.StartsWith("2 faces of the STEP data are invalid as read (from #64)", warning.Message);
        var faces = read.Value.ToShape().AllFaces();
        Assert.Equal(2, faces.Length);
        Assert.All(faces, face => Assert.False(face.Validate().IsValid));

        // With the fix, one valid face and nothing said
        Result<ModelNode> fixedRead = Step.Read(new MemoryStream(step));
        Assert.Empty(fixedRead.Diagnostics);
        Assert.True(Assert.Single(fixedRead.Value.ToShape().AllFaces()).Validate().IsValid);
    }

    [Fact]
    public void A_strip_read_as_a_face_of_no_area_is_a_success_with_a_warning_naming_it()
    {
        byte[] step = Encoding.ASCII.GetBytes(StripStep());
        Result<ModelNode> read;
        using (UndoneFixes.Scope())
        {
            read = Step.Read(new MemoryStream(step));
        }

        Assert.True(read.IsSuccess, read.Error?.Message);
        Diagnostic warning = Assert.Single(read.Diagnostics);
        Assert.Equal(Step.InvalidFace, warning.Code);
        Assert.StartsWith("1 face of the STEP data is invalid as read (from #64)", warning.Message);
        Assert.False(Assert.Single(read.Value.ToShape().AllFaces()).Validate().IsValid);

        Result<ModelNode> fixedRead = Step.Read(new MemoryStream(step));
        Assert.Empty(fixedRead.Diagnostics);
        Assert.True(Assert.Single(fixedRead.Value.ToShape().AllFaces()).Validate().IsValid);
    }

    /// <summary>
    /// The strip's lines are moved onto its vertices as the file is read, before any healing: with the healing off, the
    /// vertices off their curves still mark the face for the check.
    /// </summary>
    [Fact]
    public void Without_healing_a_face_with_a_vertex_off_its_curve_is_still_checked()
    {
        Result<ModelNode> read;
        using (UndoneFixes.Scope())
        {
            read = Step.Read(new MemoryStream(Encoding.ASCII.GetBytes(StripStep())), new StepReadOptions { Heal = false });
        }

        Assert.True(read.IsSuccess, read.Error?.Message);
        Assert.Equal(Step.InvalidFace, Assert.Single(read.Diagnostics).Code);
    }

    /// <summary>A caller who won't take a model with invalid faces refuses on the warning's code.</summary>
    [Fact]
    public void A_caller_can_refuse_a_model_with_invalid_faces_by_the_warnings_code()
    {
        static bool Accept(Result<ModelNode> read) => read.IsSuccess && !read.Diagnostics.Any(d => d.Code == Step.InvalidFace);

        using (UndoneFixes.Scope())
        {
            Assert.False(Accept(Step.Read(new MemoryStream(Encoding.ASCII.GetBytes(StripStep())))));
        }
        Assert.True(Accept(Step.Read(new MemoryStream(Encoding.ASCII.GetBytes(StripStep())))));
    }

    /// <summary>
    /// A band of a cylinder between a circle and a loop with a step in it, the arc after the step starting
    /// <paramref name="offset"/> radians from the step's line (OCCT's issue 382, decision 2305).
    /// </summary>
    private static string BandStep(double offset)
    {
        double a1 = 39 * Math.PI / 180, a2 = 141 * Math.PI / 180;
        string At(double angle, double z) => FormattableString.Invariant($"({Math.Cos(angle):R},{Math.Sin(angle):R},{z:R})");
        return string.Join('\n',
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
    }

    /// <summary>
    /// A strip of plane 0.001 wide and 1 long written as two lines between the same two vertices, the vertices midway
    /// between them (OCCT's issue 349, decision 2306).
    /// </summary>
    private static string StripStep()
    {
        return string.Join('\n',
            "ISO-10303-21;", "HEADER;", "FILE_DESCRIPTION((''),'2;1');", "FILE_NAME('','',(''),(''),'','','');",
            "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 3 1 1 1 }'));", "ENDSEC;", "DATA;",
            "#1=CARTESIAN_POINT('',(0.,0.,0.));", "#2=CARTESIAN_POINT('',(1.,0.,0.));",
            "#3=CARTESIAN_POINT('',(0.,-0.0005,0.));", "#4=CARTESIAN_POINT('',(0.,0.0005,0.));",
            "#11=VERTEX_POINT('',#1);", "#12=VERTEX_POINT('',#2);",
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
}
