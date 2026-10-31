// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntSurf_TypeTrans;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// IntPatch_RstInt.PutVertexOnLine: a walking line across a square plane face gets vertices on the two edges it
/// crosses, with their parameters on the arcs and the transitions of the line and the arcs, then its end vertices.
/// </summary>
public class IntPatch_RstIntTests
{
    [Fact]
    public void PutVertexOnLine_puts_the_crossings_of_a_square_face_on_the_line()
    {
        // Surf: the square [0, 10] x [0, 10] of the plane z = 0 (u = x, v = y). OtherSurf: the plane x = 5.
        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(new gp_Pln(), 0, 10, 0, 10).Face();
        BRepAdaptor_Surface aSurf = new(aFace);
        BRepTopAdaptor_TopolTool aDomain = new(aSurf);
        gp_Pln aPln2 = new(new gp_Pnt(5, 0, 0), new gp_Dir(1, 0, 0));
        GeomAdaptor_Surface aOther = new(new Geom_Plane(aPln2));

        // The intersection line x = 5, z = 0, from y = -2.5 to 11.5 by steps of 1.
        IntSurf_LineOn2S aCurve = new();
        for (int k = 0; k <= 14; k++)
        {
            gp_Pnt aP = new(5, -2.5 + k, 0);
            ElSLib.Parameters(aPln2, aP, out double u2, out double v2);
            IntSurf_PntOn2S aPnt = new();
            aPnt.SetValue(aP, 5, aP.Y(), u2, v2);
            aCurve.Add(aPnt);
        }
        IntPatch_WLine aW = new(aCurve, false, IntSurf_In, IntSurf_Out);

        IntPatch_RstInt.PutVertexOnLine(aW, aSurf, aDomain, aOther, true, 1e-7);

        // The crossings at y = 0 and y = 10 are inserted into the line (points 4 and 15), then the ends are made.
        Assert.Equal(17, aW.NbPnts());
        Assert.Equal(new[] { 1.0, 4.0, 15.0, 17.0 },
                     Enumerable.Range(1, aW.NbVertex()).Select(i => aW.Vertex(i).ParameterOnLine()));
        Assert.True(aW.FirstPoint().Value().Distance(new gp_Pnt(5, -2.5, 0)) < 1e-12);
        Assert.True(aW.LastPoint().Value().Distance(new gp_Pnt(5, 11.5, 0)) < 1e-12);
        Assert.False(aW.FirstPoint().IsOnDomS1());

        double[] aY = [0.0, 10.0];
        for (int i = 2; i <= 3; i++)
        {
            IntPatch_Point aV = aW.Vertex(i);
            Assert.True(aV.Value().Distance(new gp_Pnt(5, aY[i - 2], 0)) < 1e-7);
            Assert.True(aW.Point((int)aV.ParameterOnLine()).Value().Distance(aV.Value()) < 1e-7);
            Assert.True(aV.IsOnDomS1());
            Assert.False(aV.IsOnDomS2());
            Assert.False(aV.IsVertexOnS1());

            // The arc's parameter gives the crossing.
            Adaptor2d_Curve2d anArc = aV.ArcOnS1();
            Assert.True(anArc.Value(aV.ParameterOnArc1()).Distance(new gp_Pnt2d(5, aY[i - 2])) < 1e-7);
            aV.ParametersOnS1(out double u1, out double v1);
            Assert.Equal(5.0, u1, 1e-7);
            Assert.Equal(aY[i - 2], v1, 1e-7);

            // The line goes along +Y; the plane's normal is +Z: the line goes in where the arc goes along +X.
            anArc.D1(aV.ParameterOnArc1(), out _, out gp_Vec2d aD);
            IntSurf_Transition aTLine = aV.TransitionLineArc1(), aTArc = aV.TransitionOnS1();
            Assert.False(aTLine.IsTangent());
            Assert.Equal(aD.X() > 0 ? IntSurf_In : IntSurf_Out, aTLine.TransitionType());
            Assert.Equal(aD.X() > 0 ? IntSurf_Out : IntSurf_In, aTArc.TransitionType());
        }
        Assert.NotSame(aW.Vertex(2).ArcOnS1(), aW.Vertex(3).ArcOnS1());
        Assert.False(aW.HasArcOnS1());
        Assert.Equal((0.0, 0.0, 0.0, 0.0), (aW.U1Period(), aW.V1Period(), aW.U2Period(), aW.V2Period()));
    }

    [Fact]
    public void PutVertexOnLine_needs_the_points_of_a_restriction_line()
    {
        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(new gp_Pln(), 0, 10, 0, 10).Face();
        BRepAdaptor_Surface aSurf = new(aFace);
        Assert.Throws<Standard_DomainError>(() => IntPatch_RstInt.PutVertexOnLine(
            new IntPatch_RLine(false), aSurf, new BRepTopAdaptor_TopolTool(aSurf), aSurf, true, 1e-7));
    }
}
