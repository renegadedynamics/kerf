// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
using Kerf.Exchange;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.DataExchange.Tests.STEPControl;

/// <summary>
/// OCCT #1652 (decision 35002): a <c>CURVE_BOUNDED_SURFACE</c> on a sphere or a torus whose <c>OUTER_BOUNDARY_CURVE</c>
/// runs clockwise in the surface's parameters was read as the rest of the surface, a valid face and a success, as the
/// healing took the outer boundary for a hole and added the natural bounds. The file is the issue's: one patch on a
/// sphere of radius 20, u from 0 to 30 degrees and v from 10 to 40, area 98.26; the torus is the same patch on a torus
/// of radii 30 and 10, area 106.81.
/// </summary>
public class CurveBoundedSurfaceTests
{
    [Theory]
    [InlineData(false, false, 98.2562)]
    [InlineData(false, true, 98.2562)]
    [InlineData(true, false, 106.8108)]
    [InlineData(true, true, 106.8108)]
    public void An_outer_boundary_curve_bounds_its_patch_whichever_way_it_runs(bool torus, bool clockwise, double area)
    {
        Result<ModelNode> read = Step.Read(new MemoryStream(Encoding.ASCII.GetBytes(Patch(torus, clockwise))));

        Assert.True(read.IsSuccess, read.Error?.Message);
        Assert.Empty(read.Diagnostics);
        Face face = Assert.Single(read.Value.ToShape().AllFaces());
        Assert.True(face.Validate().IsValid);
        Assert.Equal(area, face.IntegrateArea(new MassOptions { Tolerance = 1e-7 }), 3);
    }

    // The issue's patch_cw.stp, generated as its script makes it, on a sphere or a torus, its boundary either way round
    private static string Patch(bool torus, bool clockwise)
    {
        const double R = 30, r = 10, rs = 20;
        double v1 = Math.PI / 18, v2 = 2 * Math.PI / 9, u2 = Math.PI / 6;
        static string F(double x) => x.ToString("E15", CultureInfo.InvariantCulture);

        // The surface, the two circles of constant v (their centres' z and radii) and the two of constant u (radius)
        string surface = torus ? $"TOROIDAL_SURFACE('',#10,{F(R)},{F(r)})" : $"SPHERICAL_SURFACE('',#10,{F(rs)})";
        double minor = torus ? r : rs, major = torus ? R : 0;
        double z1 = minor * Math.Sin(v1), z2 = minor * Math.Sin(v2);
        double c1 = major + minor * Math.Cos(v1), c2 = major + minor * Math.Cos(v2);
        string order = clockwise ? "#65,#67,#69,#71" : "#71,#69,#67,#65";
        string s1 = clockwise ? ".T." : ".F.", s2 = clockwise ? ".F." : ".T.";

        return $$"""
            ISO-10303-21;
            HEADER;
            FILE_DESCRIPTION((''),'2;1');
            FILE_NAME('patch','2026-10-20T00:00:00',(''),(''),'','','');
            FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));
            ENDSEC;
            DATA;
            #1=(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.));
            #2=(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.));
            #3=(NAMED_UNIT(*)SI_UNIT($,.STERADIAN.)SOLID_ANGLE_UNIT());
            #4=UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#1,'distance_accuracy_value','');
            #5=(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#4))GLOBAL_UNIT_ASSIGNED_CONTEXT((#1,#2,#3))REPRESENTATION_CONTEXT('ID1','3D'));
            #6=(GEOMETRIC_REPRESENTATION_CONTEXT(2)PARAMETRIC_REPRESENTATION_CONTEXT()REPRESENTATION_CONTEXT('ID2','2'));
            #7=CARTESIAN_POINT('',(0.,0.,0.));
            #8=DIRECTION('',(0.,0.,1.));
            #9=DIRECTION('',(1.,0.,0.));
            #10=AXIS2_PLACEMENT_3D('',#7,#8,#9);
            #11={{surface}};
            #12=CARTESIAN_POINT('',(0.,0.,{{F(z1)}}));
            #13=DIRECTION('',(0.,0.,1.));
            #14=DIRECTION('',(1.,0.,0.));
            #15=AXIS2_PLACEMENT_3D('',#12,#13,#14);
            #16=CIRCLE('',#15,{{F(c1)}});
            #17=TRIMMED_CURVE('',#16,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE({{F(u2)}})),.T.,.PARAMETER.);
            #18=CARTESIAN_POINT('',(0.,{{F(v1)}}));
            #19=DIRECTION('',(1.,0.));
            #20=VECTOR('',#19,{{F(u2)}});
            #21=LINE('',#18,#20);
            #22=TRIMMED_CURVE('',#21,(PARAMETER_VALUE(0.E0)),(PARAMETER_VALUE(1.E0)),.T.,.UNSPECIFIED.);
            #23=DEFINITIONAL_REPRESENTATION('',(#22),#6);
            #24=PCURVE('',#11,#23);
            #25=CARTESIAN_POINT('',({{F(major * Math.Cos(u2))}},{{F(major * Math.Sin(u2))}},0.));
            #26=DIRECTION('',({{F(Math.Sin(u2))}},{{F(-Math.Cos(u2))}},0.));
            #27=DIRECTION('',({{F(Math.Cos(u2))}},{{F(Math.Sin(u2))}},0.));
            #28=AXIS2_PLACEMENT_3D('',#25,#26,#27);
            #29=CIRCLE('',#28,{{F(minor)}});
            #30=TRIMMED_CURVE('',#29,(PARAMETER_VALUE({{F(v1)}})),(PARAMETER_VALUE({{F(v2)}})),.T.,.PARAMETER.);
            #31=CARTESIAN_POINT('',({{F(u2)}},{{F(v1)}}));
            #32=DIRECTION('',(0.,1.));
            #33=VECTOR('',#32,{{F(v2 - v1)}});
            #34=LINE('',#31,#33);
            #35=TRIMMED_CURVE('',#34,(PARAMETER_VALUE(0.E0)),(PARAMETER_VALUE(1.E0)),.T.,.UNSPECIFIED.);
            #36=DEFINITIONAL_REPRESENTATION('',(#35),#6);
            #37=PCURVE('',#11,#36);
            #38=CARTESIAN_POINT('',(0.,0.,{{F(z2)}}));
            #39=DIRECTION('',(0.,0.,1.));
            #40=DIRECTION('',(1.,0.,0.));
            #41=AXIS2_PLACEMENT_3D('',#38,#39,#40);
            #42=CIRCLE('',#41,{{F(c2)}});
            #43=TRIMMED_CURVE('',#42,(PARAMETER_VALUE(0.)),(PARAMETER_VALUE({{F(u2)}})),.T.,.PARAMETER.);
            #44=CARTESIAN_POINT('',(0.,{{F(v2)}}));
            #45=DIRECTION('',(1.,0.));
            #46=VECTOR('',#45,{{F(u2)}});
            #47=LINE('',#44,#46);
            #48=TRIMMED_CURVE('',#47,(PARAMETER_VALUE(0.E0)),(PARAMETER_VALUE(1.E0)),.T.,.UNSPECIFIED.);
            #49=DEFINITIONAL_REPRESENTATION('',(#48),#6);
            #50=PCURVE('',#11,#49);
            #51=CARTESIAN_POINT('',({{F(major)}},0.,0.));
            #52=DIRECTION('',(0.,-1.,0.));
            #53=DIRECTION('',(1.,0.,0.));
            #54=AXIS2_PLACEMENT_3D('',#51,#52,#53);
            #55=CIRCLE('',#54,{{F(minor)}});
            #56=TRIMMED_CURVE('',#55,(PARAMETER_VALUE({{F(v1)}})),(PARAMETER_VALUE({{F(v2)}})),.T.,.PARAMETER.);
            #57=CARTESIAN_POINT('',(0.,{{F(v1)}}));
            #58=DIRECTION('',(0.,1.));
            #59=VECTOR('',#58,{{F(v2 - v1)}});
            #60=LINE('',#57,#59);
            #61=TRIMMED_CURVE('',#60,(PARAMETER_VALUE(0.E0)),(PARAMETER_VALUE(1.E0)),.T.,.UNSPECIFIED.);
            #62=DEFINITIONAL_REPRESENTATION('',(#61),#6);
            #63=PCURVE('',#11,#62);
            #64=(BOUNDED_CURVE()CURVE()GEOMETRIC_REPRESENTATION_ITEM()REPRESENTATION_ITEM('')SURFACE_CURVE(#56,(#63),.PCURVE_S1.));
            #65=COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,{{s1}},#64);
            #66=(BOUNDED_CURVE()CURVE()GEOMETRIC_REPRESENTATION_ITEM()REPRESENTATION_ITEM('')SURFACE_CURVE(#43,(#50),.PCURVE_S1.));
            #67=COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,{{s1}},#66);
            #68=(BOUNDED_CURVE()CURVE()GEOMETRIC_REPRESENTATION_ITEM()REPRESENTATION_ITEM('')SURFACE_CURVE(#30,(#37),.PCURVE_S1.));
            #69=COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,{{s2}},#68);
            #70=(BOUNDED_CURVE()CURVE()GEOMETRIC_REPRESENTATION_ITEM()REPRESENTATION_ITEM('')SURFACE_CURVE(#17,(#24),.PCURVE_S1.));
            #71=COMPOSITE_CURVE_SEGMENT(.CONTINUOUS.,{{s2}},#70);
            #72=OUTER_BOUNDARY_CURVE('',({{order}}),.F.);
            #73=CURVE_BOUNDED_SURFACE('',#11,(#72),.F.);
            #74=GEOMETRIC_SET('',(#73));
            #75=GEOMETRICALLY_BOUNDED_SURFACE_SHAPE_REPRESENTATION('',(#74),#5);
            #76=APPLICATION_CONTEXT('automotive design');
            #77=APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#76);
            #78=PRODUCT_CONTEXT('',#76,'mechanical');
            #79=PRODUCT('patch','patch','',(#78));
            #80=PRODUCT_DEFINITION_FORMATION('','',#79);
            #81=PRODUCT_DEFINITION_CONTEXT('part definition',#76,'design');
            #82=PRODUCT_DEFINITION('design','',#80,#81);
            #83=PRODUCT_DEFINITION_SHAPE('','',#82);
            #84=SHAPE_DEFINITION_REPRESENTATION(#83,#75);
            ENDSEC;
            END-ISO-10303-21;
            """;
    }
}
