// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of BRepFill_Filling, its constraint classes and BRepOffsetAPI_MakeFilling, on frozen inputs as callers
/// hold them.
/// </summary>
public class BRepFill_FillingTests
{
    private static string Text(in TopoDS_Shape theShape)
    {
        using var aWriter = new StringWriter();
        BRepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }

    private static TopoDS_Edge Segment(in gp_Pnt P1, in gp_Pnt P2) => new BRepBuilderAPI_MakeEdge(P1, P2).Edge();

    // A closed contour around the square [0, 10]^2: three segments and an arc bulging up (as build123d.NET's
    // MakeSurface takes them, on shared vertices).
    private static TopoDS_Edge[] Contour()
    {
        var aV1 = new BRepBuilderAPI_MakeVertex(new gp_Pnt(0, 0, 0)).Vertex();
        var aV2 = new BRepBuilderAPI_MakeVertex(new gp_Pnt(10, 0, 0)).Vertex();
        var aV3 = new BRepBuilderAPI_MakeVertex(new gp_Pnt(10, 10, 0)).Vertex();
        var aV4 = new BRepBuilderAPI_MakeVertex(new gp_Pnt(0, 10, 0)).Vertex();
        var anArc = new GC_MakeArcOfCircle(new gp_Pnt(10, 10, 0), new gp_Pnt(5, 10, 3), new gp_Pnt(0, 10, 0)).Value()!;
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
        TopoDS_Edge[] anEdges = Contour();
        string[] aTexts = anEdges.Select(e => Text(e)).ToArray();
        var aFilling = new BRepOffsetAPI_MakeFilling(3, 15, 2, false, 0.00001, 0.0001, 0.01, 0.1, 8, 9);
        foreach (TopoDS_Edge anEdge in anEdges)
        {
            aFilling.Add(anEdge, GeomAbs_Shape.GeomAbs_C0);
        }
        aFilling.Build();
        Assert.True(aFilling.IsDone());
        TopoDS_Shape aFace = aFilling.Shape();
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_FACE, aFace.ShapeType());
        Assert.True(aFace.TShape()!.IsFrozen());
        Assert.True(new BRepCheck_Analyzer(aFace).IsValid());
        Assert.True(aFilling.G0Error() < 1.0e-3, $"G0 {aFilling.G0Error()}");
        Assert.Equal(0.0, aFilling.G1Error());

        // four new edges on new vertices, each generated from a boundary edge
        var anExp = new TopExp_Explorer(aFace, TopAbs_ShapeEnum.TopAbs_EDGE);
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
            Assert.True(aGenerated[0].TShape()!.IsFrozen());
            Assert.Equal(aTexts[i], Text(anEdges[i])); // the caller's edges never change
            TopExp.Vertices(anEdges[i], out TopoDS_Vertex aV1, out _);
            Assert.Single(aFilling.Generated(aV1));
        }
    }

    [Fact]
    public void A_built_filling_rebuilds_with_more_points_as_build123d_does()
    {
        TopoDS_Edge[] anEdges = Contour();
        var aFilling = new BRepOffsetAPI_MakeFilling(3, 15, 2, false, 0.00001, 0.0001, 0.01, 0.1, 8, 9);
        foreach (TopoDS_Edge anEdge in anEdges)
        {
            aFilling.Add(anEdge, GeomAbs_Shape.GeomAbs_C0);
        }
        aFilling.Build();
        TopoDS_Shape aFirst = aFilling.Shape();
        string aFirstText = Text(aFirst);
        aFilling.Add(new gp_Pnt(5, 5, 2));
        aFilling.Build();
        TopoDS_Shape aSecond = aFilling.Shape();
        Assert.False(aSecond.IsSame(aFirst));
        Assert.Equal(aFirstText, Text(aFirst)); // the face handed out doesn't change
        Assert.True(new BRepCheck_Analyzer(aSecond).IsValid());

        // the surface passes through the point
        TopoDS_Face aFace = TopoDS.Face(aSecond);
        var aProj = new GeomAPI_ProjectPointOnSurf(new gp_Pnt(5, 5, 2), BRep_Tool.Surface(aFace)!);
        Assert.True(aProj.LowerDistance() < 1.0e-3, $"{aProj.LowerDistance()}");

        // as in OCCT, the second build's edges are empty copies of the first build's
        Assert.Single(aFilling.Generated(anEdges[0]));
    }

    [Fact]
    public void Edges_on_support_faces_give_a_G1_filling()
    {
        // two half cylinders' straight edges and the segments between their ends; the filling between the two
        // semicircular edges of a cylinder, tangent to it, is the cylinder
        TopoDS_Face aCylFace = new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 5.0, 10.0, M_PI)
            .Face();
        var aHalfCyl = aCylFace;
        Geom_Surface aSurf = BRep_Tool.Surface(aHalfCyl)!;
        // the two semicircles of the face (the edges along v = 0 and v = 10)
        var anExp = new TopExp_Explorer(aHalfCyl, TopAbs_ShapeEnum.TopAbs_EDGE);
        var aCircles = new List<TopoDS_Edge>();
        var aLines = new List<TopoDS_Edge>();
        for (; anExp.More(); anExp.Next())
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());
            var aCurve = new BRepAdaptor_Curve(anEdge);
            (aCurve.GetType() == GeomAbs_CurveType.GeomAbs_Circle ? aCircles : aLines).Add(anEdge);
        }
        Assert.Equal(2, aCircles.Count);
        Assert.Equal(2, aLines.Count);
        string aFaceText = Text(aHalfCyl);

        var aFilling = new BRepOffsetAPI_MakeFilling();
        aFilling.Add(aCircles[0], aHalfCyl, GeomAbs_Shape.GeomAbs_G1);
        aFilling.Add(aLines[0], GeomAbs_Shape.GeomAbs_C0);
        aFilling.Add(aCircles[1], aHalfCyl, GeomAbs_Shape.GeomAbs_G1);
        aFilling.Add(aLines[1], GeomAbs_Shape.GeomAbs_C0);
        aFilling.Build();
        Assert.True(aFilling.IsDone());
        TopoDS_Shape aResult = aFilling.Shape();
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
        Assert.True(aFilling.Face().IsNull());
        var aMaker = new BRepOffsetAPI_MakeFilling();
        Assert.Throws<StdFail_NotDone>(() => aMaker.Shape());
    }

    [Fact]
    public void Add_returns_OCCTs_indices()
    {
        TopoDS_Edge[] anEdges = Contour();
        var aFilling = new BRepFill_Filling();
        Assert.Equal(1, aFilling.Add(anEdges[0], GeomAbs_Shape.GeomAbs_C0));
        Assert.Equal(2, aFilling.Add(anEdges[1], GeomAbs_Shape.GeomAbs_C0));
        Assert.Equal(3, aFilling.Add(anEdges[2], GeomAbs_Shape.GeomAbs_C0, false));
        Assert.Equal(4, aFilling.Add(new gp_Pnt(1, 2, 3)));
    }

    [Fact]
    public void The_constraint_values_compare_by_value()
    {
        TopoDS_Edge anEdge = Contour()[0];
        var aFace = new TopoDS_Face();
        Assert.Equal(new BRepFill_EdgeFaceAndOrder(anEdge, aFace, GeomAbs_Shape.GeomAbs_G1),
                     new BRepFill_EdgeFaceAndOrder(anEdge, aFace, GeomAbs_Shape.GeomAbs_G1));
        Assert.NotEqual(new BRepFill_EdgeFaceAndOrder(anEdge, aFace, GeomAbs_Shape.GeomAbs_G1),
                        new BRepFill_EdgeFaceAndOrder(anEdge, aFace, GeomAbs_Shape.GeomAbs_G2));
        Assert.Equal(new BRepFill_FaceAndOrder(aFace, GeomAbs_Shape.GeomAbs_C0), new BRepFill_FaceAndOrder());
        Assert.Throws<Standard_Failure>(() => new BRepFill_CurveConstraint((Adaptor3d_CurveOnSurface?)null, 1));
        Assert.Throws<Standard_Failure>(() => new BRepFill_CurveConstraint(new BRepAdaptor_Curve(anEdge), 3));
    }
    // build123d.NET's MakeSurfacePatch: a spline edge with its extrusion as the support face. The port's prism sweeps a
    // copy of the caller's frozen edge, so the edge has no pcurve on the face; the filling takes the face's edge on the
    // same curve (decision 10403). It threw NullReferenceException in Geom2dAdaptor_Curve.EvalD1.
    [Fact]
    public void An_edge_on_the_face_extruded_from_it_is_a_constraint()
    {
        TopoDS_Edge Spline(params gp_Pnt[] thePoints)
        {
            var aPnts = new NCollection_HArray1<gp_Pnt>(1, thePoints.Length);
            for (int i = 0; i < thePoints.Length; i++)
            {
                aPnts.SetValue(i + 1, thePoints[i]);
            }
            var anI = new GeomAPI_Interpolate(aPnts, false, 1.0e-6);
            anI.Perform();
            return new BRepBuilderAPI_MakeEdge(anI.Curve()).Edge();
        }
        TopoDS_Edge m1 = Spline(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(10, 0, -10));
        TopoDS_Edge m2 = Spline(new gp_Pnt(0, 0, 0), new gp_Pnt(0, 1, 0), new gp_Pnt(0, 10, -10));
        TopoDS_Edge m3 = Spline(new gp_Pnt(10, 0, -10), new gp_Pnt(7, 7, -10), new gp_Pnt(0, 10, -10));
        TopoDS_Face f1 = TopoDS.Face(new BRepPrimAPI_MakePrism(m1, new gp_Vec(0, -1, 0)).Shape());
        Assert.Null(BRep_Tool.CurveOnSurface(m1, f1, out _, out _));
        string aText = Text(m1);
        var aFill = new BRepOffsetAPI_MakeFilling();
        aFill.Add(m1, f1, GeomAbs_Shape.GeomAbs_C1);
        aFill.Add(m2, GeomAbs_Shape.GeomAbs_C0);
        aFill.Add(m3, GeomAbs_Shape.GeomAbs_C0);
        aFill.Build();
        Assert.True(aFill.IsDone());
        Assert.True(new BRepCheck_Analyzer(aFill.Shape()).IsValid());
        Assert.Equal(aText, Text(m1));
    }
}
