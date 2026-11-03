// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of BRepFill_Filling, its constraint classes and BRepOffsetAPI_MakeFilling, on frozen inputs as callers
/// hold them.
/// </summary>
public class BRepFill_FillingTests
{
    private static string Text(in Shape theShape)
    {
        using var aWriter = new StringWriter();
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }

    private static Edge Segment(in Point3D P1, in Point3D P2) => new BRepBuilderAPI_MakeEdge(P1, P2).Edge();

    // A closed contour around the square [0, 10]^2: three segments and an arc bulging up (as build123d.NET's
    // MakeSurface takes them, on shared vertices).
    private static Edge[] Contour()
    {
        var aV1 = new BRepBuilderAPI_MakeVertex(new Point3D(0, 0, 0)).Vertex();
        var aV2 = new BRepBuilderAPI_MakeVertex(new Point3D(10, 0, 0)).Vertex();
        var aV3 = new BRepBuilderAPI_MakeVertex(new Point3D(10, 10, 0)).Vertex();
        var aV4 = new BRepBuilderAPI_MakeVertex(new Point3D(0, 10, 0)).Vertex();
        var anArc = new MakeArcOfCircle(new Point3D(10, 10, 0), new Point3D(5, 10, 3), new Point3D(0, 10, 0)).Value()!;
        return
        [
            new BRepBuilderAPI_MakeEdge(aV1, aV2).Edge(),
            new BRepBuilderAPI_MakeEdge(aV2, aV3).Edge(),
            new BRepBuilderAPI_MakeEdge(anArc, aV3, aV4).Edge(),
            new BRepBuilderAPI_MakeEdge(aV4, aV1).Edge(),
        ];
    }

    [Fact]
    public void A_contour_of_edges_is_filled_with_a_valid_frozen_face()
    {
        Edge[] anEdges = Contour();
        string[] aTexts = anEdges.Select(e => Text(e)).ToArray();
        var aFilling = new BRepOffsetAPI_MakeFilling(3, 15, 2, false, 0.00001, 0.0001, 0.01, 0.1, 8, 9);
        foreach (Edge anEdge in anEdges)
        {
            aFilling.Add(anEdge, Continuity.C0);
        }
        aFilling.Build();
        Assert.True(aFilling.IsDone());
        Shape aFace = aFilling.Shape();
        Assert.Equal(ShapeType.Face, aFace.ShapeType());
        Assert.True(aFace.TShape()!.IsFrozen);
        Assert.True(new BRepCheck_Analyzer(aFace).IsValid());
        Assert.True(aFilling.G0Error() < 1.0e-3, $"G0 {aFilling.G0Error()}");
        Assert.Equal(0.0, aFilling.G1Error());

        // four new edges on new vertices, each generated from a boundary edge
        var anExp = new Explorer(aFace, ShapeType.Edge);
        int aNb = 0;
        for (; anExp.More(); anExp.Next())
        {
            aNb++;
        }
        Assert.Equal(4, aNb);
        for (int i = 0; i < 4; i++)
        {
            var aGenerated = aFilling.Generated(anEdges[i]);
            Assert.Single(aGenerated);
            Assert.False(aGenerated[0].IsSame(anEdges[i]));
            Assert.True(aGenerated[0].TShape()!.IsFrozen);
            Assert.Equal(aTexts[i], Text(anEdges[i])); // the caller's edges never change
            Exploration.Vertices(anEdges[i], out Vertex aV1, out _);
            Assert.Single(aFilling.Generated(aV1));
        }
    }

    [Fact]
    public void A_built_filling_rebuilds_with_more_points_as_build123d_does()
    {
        Edge[] anEdges = Contour();
        var aFilling = new BRepOffsetAPI_MakeFilling(3, 15, 2, false, 0.00001, 0.0001, 0.01, 0.1, 8, 9);
        foreach (Edge anEdge in anEdges)
        {
            aFilling.Add(anEdge, Continuity.C0);
        }
        aFilling.Build();
        Shape aFirst = aFilling.Shape();
        string aFirstText = Text(aFirst);
        aFilling.Add(new Point3D(5, 5, 2));
        aFilling.Build();
        Shape aSecond = aFilling.Shape();
        Assert.False(aSecond.IsSame(aFirst));
        Assert.Equal(aFirstText, Text(aFirst)); // the face handed out doesn't change
        Assert.True(new BRepCheck_Analyzer(aSecond).IsValid());

        // the surface passes through the point
        Face aFace = ShapeCasts.Face(aSecond);
        var aProj = new GeomAPI_ProjectPointOnSurf(new Point3D(5, 5, 2), Tool.Surface(aFace)!);
        Assert.True(aProj.LowerDistance() < 1.0e-3, $"{aProj.LowerDistance()}");

        // as in OCCT, the second build's edges are empty copies of the first build's
        Assert.Single(aFilling.Generated(anEdges[0]));
    }

    [Fact]
    public void Edges_on_support_faces_give_a_G1_filling()
    {
        // two half cylinders' straight edges and the segments between their ends; the filling between the two
        // semicircular edges of a cylinder, tangent to it, is the cylinder
        Face aCylFace = new BRepPrimAPI_MakeCylinder(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 5.0, 10.0, M_PI)
            .Face();
        var aHalfCyl = aCylFace;
        Surface aSurf = Tool.Surface(aHalfCyl)!;
        // the two semicircles of the face (the edges along v = 0 and v = 10)
        var anExp = new Explorer(aHalfCyl, ShapeType.Edge);
        var aCircles = new List<Edge>();
        var aLines = new List<Edge>();
        for (; anExp.More(); anExp.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anExp.Current());
            var aCurve = new EdgeCurveAdaptor(anEdge);
            (aCurve.GetType() == CurveKind.Circle ? aCircles : aLines).Add(anEdge);
        }
        Assert.Equal(2, aCircles.Count);
        Assert.Equal(2, aLines.Count);
        string aFaceText = Text(aHalfCyl);

        var aFilling = new BRepOffsetAPI_MakeFilling();
        aFilling.Add(aCircles[0], aHalfCyl, Continuity.G1);
        aFilling.Add(aLines[0], Continuity.C0);
        aFilling.Add(aCircles[1], aHalfCyl, Continuity.G1);
        aFilling.Add(aLines[1], Continuity.C0);
        aFilling.Build();
        Assert.True(aFilling.IsDone());
        Shape aResult = aFilling.Shape();
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.True(aFilling.G0Error() < 1.0e-2, $"G0 {aFilling.G0Error()}");
        Assert.True(aFilling.G1Error() < 0.05, $"G1 {aFilling.G1Error()}");
        Assert.Equal(aFaceText, Text(aHalfCyl));
        _ = aSurf;
    }

    [Fact]
    public void An_empty_filling_is_not_done()
    {
        var aFilling = new BRepFill_Filling();
        aFilling.Build();
        Assert.False(aFilling.IsDone());
        Assert.True(aFilling.Face().IsNull);
        var aMaker = new BRepOffsetAPI_MakeFilling();
        Assert.Throws<NotDoneException>(() => aMaker.Shape());
    }

    [Fact]
    public void Add_returns_OCCTs_indices()
    {
        Edge[] anEdges = Contour();
        var aFilling = new BRepFill_Filling();
        Assert.Equal(1, aFilling.Add(anEdges[0], Continuity.C0));
        Assert.Equal(2, aFilling.Add(anEdges[1], Continuity.C0));
        Assert.Equal(3, aFilling.Add(anEdges[2], Continuity.C0, false));
        Assert.Equal(4, aFilling.Add(new Point3D(1, 2, 3)));
    }

    [Fact]
    public void The_constraint_values_compare_by_value()
    {
        Edge anEdge = Contour()[0];
        var aFace = new Face();
        Assert.Equal(new BRepFill_EdgeFaceAndOrder(anEdge, aFace, Continuity.G1),
                     new BRepFill_EdgeFaceAndOrder(anEdge, aFace, Continuity.G1));
        Assert.NotEqual(new BRepFill_EdgeFaceAndOrder(anEdge, aFace, Continuity.G1),
                        new BRepFill_EdgeFaceAndOrder(anEdge, aFace, Continuity.G2));
        Assert.Equal(new BRepFill_FaceAndOrder(aFace, Continuity.C0), new BRepFill_FaceAndOrder());
        Assert.Throws<FailureException>(() => new BRepFill_CurveConstraint((CurveOnSurface?)null, 1));
        Assert.Throws<FailureException>(() => new BRepFill_CurveConstraint(new EdgeCurveAdaptor(anEdge), 3));
    }
    // build123d.NET's MakeSurfacePatch: a spline edge with its extrusion as the support face. The port's prism sweeps a
    // copy of the caller's frozen edge, so the edge has no pcurve on the face; the filling takes the face's edge on the
    // same curve (decision 1103). It threw NullReferenceException in Geom2dAdaptor_Curve.EvalD1.
    [Fact]
    public void An_edge_on_the_face_extruded_from_it_is_a_constraint()
    {
        Edge Spline(params Point3D[] thePoints)
        {
            var aPnts = new NCollection_HArray1<Point3D>(1, thePoints.Length);
            for (int i = 0; i < thePoints.Length; i++)
            {
                aPnts.SetValue(i + 1, thePoints[i]);
            }
            var anI = new GeomAPI_Interpolate(aPnts, false, 1.0e-6);
            anI.Perform();
            return new BRepBuilderAPI_MakeEdge(anI.Curve()).Edge();
        }
        Edge m1 = Spline(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(10, 0, -10));
        Edge m2 = Spline(new Point3D(0, 0, 0), new Point3D(0, 1, 0), new Point3D(0, 10, -10));
        Edge m3 = Spline(new Point3D(10, 0, -10), new Point3D(7, 7, -10), new Point3D(0, 10, -10));
        Face f1 = ShapeCasts.Face(new BRepPrimAPI_MakePrism(m1, new Vector3D(0, -1, 0)).Shape());
        Assert.Null(Tool.CurveOnSurface(m1, f1, out _, out _));
        string aText = Text(m1);
        var aFill = new BRepOffsetAPI_MakeFilling();
        aFill.Add(m1, f1, Continuity.C1);
        aFill.Add(m2, Continuity.C0);
        aFill.Add(m3, Continuity.C0);
        aFill.Build();
        Assert.True(aFill.IsDone());
        Assert.True(new BRepCheck_Analyzer(aFill.Shape()).IsValid());
        Assert.Equal(aText, Text(m1));
    }
}
