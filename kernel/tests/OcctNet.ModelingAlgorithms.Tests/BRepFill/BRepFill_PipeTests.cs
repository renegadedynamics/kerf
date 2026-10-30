// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomFill_Trihedron;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit 5S1's BRepFill_Pipe and BRepOffsetAPI_MakePipe (decisions 909-911). The topological sweep (BRepFill_Sweep)
/// is unit 5S2's: until then every pipe throws Standard_NotImplemented, which these tests pin, with what Perform does
/// before it sweeps (the location law, the placement of the profile, the first and last shapes, the solid check), and
/// that frozen inputs are never written.
/// </summary>
public class BRepFill_PipeTests
{
    private const string NotPorted = "BRepFill_Sweep is ported by unit 5S2";

    private static TopoDS_Wire Polyline(params gp_Pnt[] thePoints)
    {
        BRepBuilderAPI_MakePolygon aP = new();
        foreach (gp_Pnt aQ in thePoints)
        {
            aP.Add(aQ);
        }
        return aP.Wire();
    }

    private static TopoDS_Wire Circle(double theRadius)
    {
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(new gp_Circ(new gp_Ax2(), theRadius)).Edge();
        return new BRepBuilderAPI_MakeWire(anEdge).Wire();
    }

    private static string Text(in TopoDS_Shape theShape)
    {
        using StringWriter aW = new();
        BRepTools.Write(theShape, aW);
        return aW.ToString();
    }

    private static List<gp_Pnt> Points(in TopoDS_Shape theShape)
    {
        List<gp_Pnt> aPoints = [];
        for (TopExp_Explorer anExp = new(theShape, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            aPoints.Add(BRep_Tool.Pnt(TopoDS.Vertex(anExp.Current())));
        }
        return aPoints;
    }

    [Fact]
    public void MakePipe_waits_for_the_sweep_of_5S2_and_leaves_its_inputs_unchanged()
    {
        TopoDS_Wire aSpine = Polyline(new gp_Pnt(0, 0, 0), new gp_Pnt(0, 0, 10));
        TopoDS_Shape[] aProfiles =
        [
            Circle(1),
            new BRepBuilderAPI_MakeFace(Circle(2)).Face(),
            new BRepBuilderAPI_MakeVertex(new gp_Pnt(1, 0, 0)).Vertex(),
            Polyline(new gp_Pnt(1, 0, 0), new gp_Pnt(2, 0, 0)),
        ];
        string aSpineText = Text(aSpine);
        foreach (TopoDS_Shape aProfile in aProfiles)
        {
            Assert.True(aProfile.TShape()!.IsFrozen());
            string aText = Text(aProfile);
            foreach (GeomFill_Trihedron aMode in new[] { GeomFill_IsCorrectedFrenet, GeomFill_IsFrenet, GeomFill_IsDiscreteTrihedron })
            {
                var anError = Assert.Throws<Standard_NotImplemented>(() => new BRepOffsetAPI_MakePipe(aSpine, aProfile, aMode));
                Assert.Contains(NotPorted, anError.Message, StringComparison.Ordinal);
            }
            Assert.Throws<Standard_NotImplemented>(() => new BRepOffsetAPI_MakePipe(aSpine, aProfile));
            Assert.Throws<Standard_NotImplemented>(() => new BRepFill_Pipe(aSpine, aProfile));
            Assert.Equal(aText, Text(aProfile));
        }
        Assert.Equal(aSpineText, Text(aSpine));
    }

    [Fact]
    public void A_solid_profile_is_refused_after_the_first_and_last_shapes_are_placed()
    {
        // Perform places the profile and builds the first and last shapes before MakeShape refuses the solid, as OCCT.
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(-1, -1, 0), 2, 2, 1).Shape();
        string aBoxText = Text(aBox);
        TopoDS_Wire aSpine = Polyline(new gp_Pnt(0, 0, 0), new gp_Pnt(0, 0, 4), new gp_Pnt(0, 0, 10));

        BRepFill_Pipe aPipe = new();
        var anError = Assert.Throws<Standard_DomainError>(() => aPipe.Perform(aSpine, aBox));
        Assert.Contains("profile contains solids", anError.Message, StringComparison.Ordinal);
        Assert.Equal(aBoxText, Text(aBox));

        // Along a straight spine the profile is only translated: the first shape is the box moved along the spine (by
        // the abscissa SectionPlacement found for it), the last one the first moved by the spine's length.
        TopoDS_Shape aFirst = aPipe.FirstShape();
        TopoDS_Shape aLast = aPipe.LastShape();
        Assert.True(aFirst.TShape()!.IsFrozen());
        Assert.Equal(TopAbs_SOLID, aFirst.ShapeType());
        Assert.True(aFirst.Location().IsIdentity()); // RemoveLocations
        Assert.True(aLast.Location().IsIdentity());
        List<gp_Pnt> aBoxPoints = Points(aBox), aFirstPoints = Points(aFirst), aLastPoints = Points(aLast);
        Assert.Equal(aBoxPoints.Count, aFirstPoints.Count);
        gp_Vec aShift = new(aBoxPoints[0], aFirstPoints[0]);
        Assert.True(Math.Abs(aShift.X()) < 1e-9 && Math.Abs(aShift.Y()) < 1e-9);
        for (int i = 0; i < aBoxPoints.Count; i++)
        {
            Assert.True(aBoxPoints[i].Translated(aShift).Distance(aFirstPoints[i]) < 1e-9);
            Assert.True(aFirstPoints[i].Translated(new gp_Vec(0, 0, 10)).Distance(aLastPoints[i]) < 1e-9);
        }
        Assert.Equal(aBoxText, Text(aBox));

        // and inside a compound
        BRep_Builder aB = new();
        TopoDS_Compound aCompound = new();
        aB.MakeCompound(ref aCompound);
        aB.Add(aCompound, aBox);
        Assert.Throws<Standard_DomainError>(() => new BRepOffsetAPI_MakePipe(aSpine, aCompound));
        Assert.Equal(aBoxText, Text(aBox));
    }

    [Fact]
    public void Queries_of_an_unbuilt_pipe_throw_as_OCCT_does()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 1, 1).Shape();
        TopoDS_Wire aSpine = Polyline(new gp_Pnt(0, 0, 0), new gp_Pnt(0, 0, 3));
        BRepFill_Pipe aPipe = new();
        Assert.Throws<Standard_DomainError>(() => aPipe.Perform(aSpine, aBox));
        Assert.True(aPipe.Shape().IsNull());
        Assert.Equal(TopAbs_WIRE, aPipe.Spine().ShapeType());

        TopoDS_Edge aSpineEdge = TopoDS.Edge(new TopExp_Explorer(aSpine, TopAbs_EDGE).Current());
        TopoDS_Edge aBoxEdge = TopoDS.Edge(new TopExp_Explorer(aBox, TopAbs_EDGE).Current());
        // the profile is searched first: a solid profile is refused
        Assert.Throws<Standard_DomainError>(() => aPipe.Face(aSpineEdge, aBoxEdge));
        // a vertex not on the spine
        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(new gp_Pnt(5, 5, 5)).Vertex();
        Assert.Throws<Standard_DomainError>(() => aPipe.Section(aV));
        // a spine vertex: OCCT reads the sections the sweep would have made
        TopoDS_Vertex aSpineVertex = TopExp.FirstVertex(aSpineEdge);
        Assert.ThrowsAny<Exception>(() => aPipe.Section(aSpineVertex));
        // PipeLine sweeps a point
        Assert.Throws<Standard_NotImplemented>(() => aPipe.PipeLine(new gp_Pnt(0, 0, 0)));

        NCollection_List<TopoDS_Shape> aList = new();
        aList.Append(aBox);
        aPipe.Generated(aBox, aList);
        Assert.True(aList.IsEmpty());
    }
}
