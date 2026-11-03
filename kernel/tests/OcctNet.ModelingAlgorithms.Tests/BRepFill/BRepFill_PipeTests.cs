// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.GeomFill_Trihedron;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 5S1's BRepFill_Pipe and BRepOffsetAPI_MakePipe (decisions 909-911). The topological sweep (BRepFill_Sweep)
/// is unit 5S2's (decision 924). These tests check pipes of every kind of profile against their measures, what Perform
/// does before it sweeps (the location law, the placement of the profile, the first and last shapes, the solid check),
/// and that frozen inputs are never written.
/// </summary>
public class BRepFill_PipeTests
{
    private static TopoDS_Wire Polyline(params Point3D[] thePoints)
    {
        BRepBuilderAPI_MakePolygon aP = new();
        foreach (Point3D aQ in thePoints)
        {
            aP.Add(aQ);
        }
        return aP.Wire();
    }

    private static TopoDS_Wire Circle(double theRadius)
    {
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(new Circle3D(new Frame3D(), theRadius)).Edge();
        return new BRepBuilderAPI_MakeWire(anEdge).Wire();
    }

    private static string Text(in TopoDS_Shape theShape)
    {
        using StringWriter aW = new();
        BRepTools.Write(theShape, aW);
        return aW.ToString();
    }

    private static List<Point3D> Points(in TopoDS_Shape theShape)
    {
        List<Point3D> aPoints = [];
        for (TopExp_Explorer anExp = new(theShape, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            aPoints.Add(BRep_Tool.Pnt(TopoDS.Vertex(anExp.Current())));
        }
        return aPoints;
    }

    [Fact]
    public void MakePipe_sweeps_every_kind_of_profile_and_leaves_its_inputs_unchanged()
    {
        TopoDS_Wire aSpine = Polyline(new Point3D(0, 0, 0), new Point3D(0, 0, 10));
        (TopoDS_Shape Profile, TopAbs_ShapeEnum Type, double Measure)[] aCases =
        [
            (Circle(1), TopAbs_SHELL, 2 * Math.PI * 10),                                    // lateral area
            (new BRepBuilderAPI_MakeFace(Circle(2)).Face(), TopAbs_SOLID, Math.PI * 4 * 10), // volume
            (new BRepBuilderAPI_MakeVertex(new Point3D(1, 0, 0)).Vertex(), TopAbs_EDGE, 10),  // length
            (Polyline(new Point3D(1, 0, 0), new Point3D(2, 0, 0)), TopAbs_SHELL, 10),         // area
        ];
        string aSpineText = Text(aSpine);
        foreach ((TopoDS_Shape aProfile, TopAbs_ShapeEnum aType, double aMeasure) in aCases)
        {
            Assert.True(aProfile.TShape()!.IsFrozen);
            string aText = Text(aProfile);
            foreach (GeomFill_Trihedron aMode in new[] { GeomFill_IsCorrectedFrenet, GeomFill_IsFrenet, GeomFill_IsDiscreteTrihedron })
            {
                BRepOffsetAPI_MakePipe aPipe = new(aSpine, aProfile, aMode);
                Assert.True(aPipe.IsDone());
                TopoDS_Shape aResult = aPipe.Shape();
                Assert.True(aResult.TShape()!.IsFrozen);
                TopoDS_Shape aBody = aResult;
                if (aType != TopAbs_EDGE)
                {
                    TopExp_Explorer anExp = new(aResult, aType);
                    Assert.True(anExp.More());
                    aBody = anExp.Current();
                }
                GProp_GProps aProps = new();
                switch (aType)
                {
                    case TopAbs_SOLID:
                        BRepGProp.VolumeProperties(aBody, aProps);
                        break;
                    case TopAbs_SHELL:
                        BRepGProp.SurfaceProperties(aBody, aProps);
                        break;
                    default:
                        BRepGProp.LinearProperties(aBody, aProps);
                        break;
                }
                Assert.Equal(aMeasure, aProps.Mass(), 1e-6 * aMeasure);
                Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
                Assert.Equal(aText, Text(aProfile));
            }
            Assert.Equal(aText, Text(aProfile));
        }
        Assert.Equal(aSpineText, Text(aSpine));
    }

    [Fact]
    public void A_solid_profile_is_refused_after_the_first_and_last_shapes_are_placed()
    {
        // Perform places the profile and builds the first and last shapes before MakeShape refuses the solid, as OCCT.
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new Point3D(-1, -1, 0), 2, 2, 1).Shape();
        string aBoxText = Text(aBox);
        TopoDS_Wire aSpine = Polyline(new Point3D(0, 0, 0), new Point3D(0, 0, 4), new Point3D(0, 0, 10));

        BRepFill_Pipe aPipe = new();
        var anError = Assert.Throws<DomainErrorException>(() => aPipe.Perform(aSpine, aBox));
        Assert.Contains("profile contains solids", anError.Message, StringComparison.Ordinal);
        Assert.Equal(aBoxText, Text(aBox));

        // Along a straight spine the profile is only translated: the first shape is the box moved along the spine (by
        // the abscissa SectionPlacement found for it), the last one the first moved by the spine's length.
        TopoDS_Shape aFirst = aPipe.FirstShape();
        TopoDS_Shape aLast = aPipe.LastShape();
        Assert.True(aFirst.TShape()!.IsFrozen);
        Assert.Equal(TopAbs_SOLID, aFirst.ShapeType());
        Assert.True(aFirst.Location().IsIdentity); // RemoveLocations
        Assert.True(aLast.Location().IsIdentity);
        List<Point3D> aBoxPoints = Points(aBox), aFirstPoints = Points(aFirst), aLastPoints = Points(aLast);
        Assert.Equal(aBoxPoints.Count, aFirstPoints.Count);
        Vector3D aShift = new(aBoxPoints[0], aFirstPoints[0]);
        Assert.True(Math.Abs(aShift.X()) < 1e-9 && Math.Abs(aShift.Y()) < 1e-9);
        for (int i = 0; i < aBoxPoints.Count; i++)
        {
            Assert.True(aBoxPoints[i].Translated(aShift).Distance(aFirstPoints[i]) < 1e-9);
            Assert.True(aFirstPoints[i].Translated(new Vector3D(0, 0, 10)).Distance(aLastPoints[i]) < 1e-9);
        }
        Assert.Equal(aBoxText, Text(aBox));

        // and inside a compound
        BRep_Builder aB = new();
        TopoDS_Compound aCompound = new();
        aB.MakeCompound(ref aCompound);
        aB.Add(aCompound, aBox);
        Assert.Throws<DomainErrorException>(() => new BRepOffsetAPI_MakePipe(aSpine, aCompound));
        Assert.Equal(aBoxText, Text(aBox));
    }

    [Fact]
    public void Queries_of_an_unbuilt_pipe_throw_as_OCCT_does()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 1, 1).Shape();
        TopoDS_Wire aSpine = Polyline(new Point3D(0, 0, 0), new Point3D(0, 0, 3));
        BRepFill_Pipe aPipe = new();
        Assert.Throws<DomainErrorException>(() => aPipe.Perform(aSpine, aBox));
        Assert.True(aPipe.Shape().IsNull());
        Assert.Equal(TopAbs_WIRE, aPipe.Spine().ShapeType());

        TopoDS_Edge aSpineEdge = TopoDS.Edge(new TopExp_Explorer(aSpine, TopAbs_EDGE).Current());
        TopoDS_Edge aBoxEdge = TopoDS.Edge(new TopExp_Explorer(aBox, TopAbs_EDGE).Current());
        // the profile is searched first: a solid profile is refused
        Assert.Throws<DomainErrorException>(() => aPipe.Face(aSpineEdge, aBoxEdge));
        // a vertex not on the spine
        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(new Point3D(5, 5, 5)).Vertex();
        Assert.Throws<DomainErrorException>(() => aPipe.Section(aV));
        // a spine vertex: OCCT reads the sections the sweep would have made
        TopoDS_Vertex aSpineVertex = TopExp.FirstVertex(aSpineEdge);
        Assert.ThrowsAny<Exception>(() => aPipe.Section(aSpineVertex));
        // PipeLine sweeps a point along the spine (the location law was made before the profile was refused)
        TopoDS_Wire aLine = aPipe.PipeLine(new Point3D(0, 0, 0));
        Assert.Equal(TopAbs_WIRE, aLine.ShapeType());
        Assert.True(aLine.TShape()!.IsFrozen);

        NCollection_List<TopoDS_Shape> aList = new();
        aList.Append(aBox);
        aPipe.Generated(aBox, aList);
        Assert.True(aList.IsEmpty());
    }
}
