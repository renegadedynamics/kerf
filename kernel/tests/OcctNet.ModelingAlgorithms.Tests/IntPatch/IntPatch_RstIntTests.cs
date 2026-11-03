// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

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
        Face aFace = new MakeFace(new Plane3D(), 0, 10, 0, 10).Face();
        FaceSurfaceAdaptor aSurf = new(aFace);
        BrepTopolTool aDomain = new(aSurf);
        Plane3D aPln2 = new(new Point3D(5, 0, 0), new Direction3D(1, 0, 0));
        SurfaceAdaptor aOther = new(new PlaneSurface(aPln2));

        // The intersection line x = 5, z = 0, from y = -2.5 to 11.5 by steps of 1.
        LineOn2S aCurve = new();
        for (int k = 0; k <= 14; k++)
        {
            Point3D aP = new(5, -2.5 + k, 0);
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aPln2, aP, out double u2, out double v2);
            PntOn2S aPnt = new();
            aPnt.SetValue(aP, 5, aP.Y(), u2, v2);
            aCurve.Add(aPnt);
        }
        WLine aW = new(aCurve, false, WalkingTypeTrans.In, WalkingTypeTrans.Out);

        RstInt.PutVertexOnLine(aW, aSurf, aDomain, aOther, true, 1e-7);

        // The crossings at y = 0 and y = 10 are inserted into the line (points 4 and 15), then the ends are made.
        Assert.Equal(17, aW.NbPnts());
        Assert.Equal(new[] { 1.0, 4.0, 15.0, 17.0 },
                     Enumerable.Range(1, aW.VertexCount).Select(i => aW.Vertex(i).ParameterOnLine()));
        Assert.True(aW.FirstPoint().Value().Distance(new Point3D(5, -2.5, 0)) < 1e-12);
        Assert.True(aW.LastPoint().Value().Distance(new Point3D(5, 11.5, 0)) < 1e-12);
        Assert.False(aW.FirstPoint().IsOnDomS1);

        double[] aY = [0.0, 10.0];
        for (int i = 2; i <= 3; i++)
        {
            PatchPoint aV = aW.Vertex(i);
            Assert.True(aV.Value().Distance(new Point3D(5, aY[i - 2], 0)) < 1e-7);
            Assert.True(aW.Point((int)aV.ParameterOnLine()).Value().Distance(aV.Value()) < 1e-7);
            Assert.True(aV.IsOnDomS1);
            Assert.False(aV.IsOnDomS2);
            Assert.False(aV.IsVertexOnS1);

            // The arc's parameter gives the crossing.
            Curve2DAdaptorBase anArc = aV.ArcOnS1();
            Assert.True(anArc.Value(aV.ParameterOnArc1()).Distance(new Point2D(5, aY[i - 2])) < 1e-7);
            aV.ParametersOnS1(out double u1, out double v1);
            Assert.Equal(5.0, u1, 1e-7);
            Assert.Equal(aY[i - 2], v1, 1e-7);

            // The line goes along +Y; the plane's normal is +Z: the line goes in where the arc goes along +X.
            anArc.D1(aV.ParameterOnArc1(), out _, out Vector2D aD);
            WalkingTransition aTLine = aV.TransitionLineArc1(), aTArc = aV.TransitionOnS1();
            Assert.False(aTLine.IsTangent);
            Assert.Equal(aD.X() > 0 ? WalkingTypeTrans.In : WalkingTypeTrans.Out, aTLine.TransitionType());
            Assert.Equal(aD.X() > 0 ? WalkingTypeTrans.Out : WalkingTypeTrans.In, aTArc.TransitionType());
        }
        Assert.NotSame(aW.Vertex(2).ArcOnS1(), aW.Vertex(3).ArcOnS1());
        Assert.False(aW.HasArcOnS1);
        Assert.Equal((0.0, 0.0, 0.0, 0.0), (aW.U1Period(), aW.V1Period(), aW.U2Period(), aW.V2Period()));
    }

    [Fact]
    public void PutVertexOnLine_needs_the_points_of_a_restriction_line()
    {
        Face aFace = new MakeFace(new Plane3D(), 0, 10, 0, 10).Face();
        FaceSurfaceAdaptor aSurf = new(aFace);
        Assert.Throws<DomainErrorException>(() => RstInt.PutVertexOnLine(
            new RLine(false), aSurf, new BrepTopolTool(aSurf), aSurf, true, 1e-7));
    }
}
