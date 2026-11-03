// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of: src/ModelingAlgorithms/TKBool/GTests/BRepFill_PipeShell_Test.cxx (OCCT V8_0_1), with OCCT.NET's checks of
// frozen inputs and the history added.

using Kerf.Geometry;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>BRepFill_PipeShell (unit 5S2): OCCT's GTests, and the port's frozen inputs and history.</summary>
public class BRepFill_PipeShellTests
{
    private static TopoDS_Wire MakeSquareProfile(in Point3D theCenter, double theHalfSide)
    {
        BRepBuilderAPI_MakePolygon aPolygon = new();
        aPolygon.Add(new Point3D(theCenter.X() - theHalfSide, theCenter.Y() - theHalfSide, theCenter.Z()));
        aPolygon.Add(new Point3D(theCenter.X() + theHalfSide, theCenter.Y() - theHalfSide, theCenter.Z()));
        aPolygon.Add(new Point3D(theCenter.X() + theHalfSide, theCenter.Y() + theHalfSide, theCenter.Z()));
        aPolygon.Add(new Point3D(theCenter.X() - theHalfSide, theCenter.Y() + theHalfSide, theCenter.Z()));
        aPolygon.Close();
        return aPolygon.Wire();
    }

    private static string Text(in TopoDS_Shape theShape)
    {
        using StringWriter aW = new();
        BRepTools.Write(theShape, aW);
        return aW.ToString();
    }

    [Fact]
    public void MakeSolid_StraightSpineClosedProfile_ProducesClosedSolid()
    {
        TopoDS_Edge aSpineEdge = new BRepBuilderAPI_MakeEdge(new Point3D(0.0, 0.0, 0.0), new Point3D(0.0, 0.0, 10.0)).Edge();
        TopoDS_Wire aSpine = new BRepBuilderAPI_MakeWire(aSpineEdge).Wire();

        TopoDS_Wire aProfile = MakeSquareProfile(new Point3D(0.0, 0.0, 0.0), 1.0);

        BRepFill_PipeShell aPipe = new(aSpine);
        aPipe.Add(aProfile);
        Assert.True(aPipe.IsReady());
        Assert.True(aPipe.Build());
        Assert.True(aPipe.MakeSolid());

        TopoDS_Shape aResult = aPipe.Shape();
        Assert.False(aResult.IsNull());
        Assert.True(aResult.Closed());

        BRepCheck_Analyzer anAnalyzer = new(aResult);
        Assert.True(anAnalyzer.IsValid());
    }

    [Fact]
    public void MakeSolid_PolylineSpineClosedProfile_ProducesClosedSolid()
    {
        BRepBuilderAPI_MakePolygon aSpinePolygon = new();
        aSpinePolygon.Add(new Point3D(0.0, 0.0, 0.0));
        aSpinePolygon.Add(new Point3D(0.0, 0.0, 5.0));
        aSpinePolygon.Add(new Point3D(0.0, 5.0, 10.0));
        TopoDS_Wire aSpine = aSpinePolygon.Wire();

        TopoDS_Wire aProfile = MakeSquareProfile(new Point3D(0.0, 0.0, 0.0), 0.5);

        BRepFill_PipeShell aPipe = new(aSpine);
        aPipe.SetTransition(BRepFill_TransitionStyle.BRepFill_Right);
        aPipe.Add(aProfile);
        Assert.True(aPipe.IsReady());
        Assert.True(aPipe.Build());
        Assert.True(aPipe.MakeSolid());

        TopoDS_Shape aResult = aPipe.Shape();
        Assert.False(aResult.IsNull());
        Assert.True(aResult.Closed());
    }

    [Theory]
    [InlineData(BRepFill_TransitionStyle.BRepFill_Modified)]
    [InlineData(BRepFill_TransitionStyle.BRepFill_Right)]
    [InlineData(BRepFill_TransitionStyle.BRepFill_Round)]
    public void Frozen_inputs_are_unchanged_and_the_history_answers_for_them(BRepFill_TransitionStyle theTransition)
    {
        // OCCT.NET: a closed spine whose wire isn't flagged closed (the pipe closes a new wire of the caller's edges)
        BRepBuilderAPI_MakePolygon aSpinePolygon = new();
        aSpinePolygon.Add(new Point3D(0.0, 0.0, 0.0));
        aSpinePolygon.Add(new Point3D(0.0, 0.0, 5.0));
        aSpinePolygon.Add(new Point3D(0.0, 5.0, 10.0));
        TopoDS_Wire aSpine = aSpinePolygon.Wire();
        TopoDS_Wire aProfile = MakeSquareProfile(new Point3D(0.0, 0.0, 0.0), 0.5);
        Assert.True(aSpine.TShape()!.IsFrozen);
        Assert.True(aProfile.TShape()!.IsFrozen);
        string aSpineText = Text(aSpine), aProfileText = Text(aProfile);

        BRepFill_PipeShell aPipe = new(aSpine);
        aPipe.SetTransition(theTransition);
        aPipe.Add(aProfile);
        Assert.True(aPipe.Build());
        Assert.True(aPipe.MakeSolid());
        Assert.Equal(aSpineText, Text(aSpine));
        Assert.Equal(aProfileText, Text(aProfile));

        TopoDS_Shape aResult = aPipe.Shape();
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(TopAbs_SOLID, aResult.ShapeType());

        // every edge of the profile generates faces of the result, every edge of the spine too
        NCollection_IndexedMap<TopoDS_Shape> aFaces = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(aResult, TopAbs_FACE, aFaces);
        NCollection_List<TopoDS_Shape> aList = new();
        foreach (TopAbs_ShapeEnum aType in new[] { TopAbs_EDGE, TopAbs_VERTEX })
        {
            foreach (TopoDS_Shape aWire in new TopoDS_Shape[] { aProfile, aSpine })
            {
                for (TopExp_Explorer anExp = new(aWire, aType); anExp.More(); anExp.Next())
                {
                    aPipe.Generated(anExp.Current(), aList);
                    if (aType == TopAbs_EDGE)
                    {
                        Assert.False(aList.IsEmpty());
                    }
                    for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(aList); anIt.More(); anIt.Next())
                    {
                        Assert.True(anIt.Value().TShape()!.IsFrozen);
                        if (anIt.Value().ShapeType() == TopAbs_FACE)
                        {
                            Assert.True(aFaces.Contains(anIt.Value()));
                        }
                    }
                }
            }
        }
    }

    [Fact]
    public void A_spine_closed_but_not_flagged_is_closed_on_a_new_wire()
    {
        BRepBuilderAPI_MakePolygon aSpinePolygon = new();
        aSpinePolygon.Add(new Point3D(0.0, 0.0, 0.0));
        aSpinePolygon.Add(new Point3D(10.0, 0.0, 0.0));
        aSpinePolygon.Add(new Point3D(10.0, 10.0, 0.0));
        aSpinePolygon.Add(new Point3D(0.0, 0.0, 0.0));
        TopoDS_Wire aSpine = aSpinePolygon.Wire();
        BRep_Builder aB = new();
        TopoDS_Wire anOpen = new();
        aB.MakeWire(ref anOpen);
        for (TopoDS_Iterator anIt = new(aSpine); anIt.More(); anIt.Next())
        {
            aB.Add(anOpen, anIt.Value());
        }
        anOpen.TShape()!.FreezeGraph();
        Assert.False(anOpen.Closed());

        BRepFill_PipeShell aPipe = new(anOpen);
        Assert.False(anOpen.Closed());
        TopoDS_Wire aPipeSpine = aPipe.Spine();
        Assert.True(aPipeSpine.Closed());
        Assert.False(aPipeSpine.IsSame(anOpen));
        TopExp_Explorer anExp1 = new(anOpen, TopAbs_EDGE), anExp2 = new(aPipeSpine, TopAbs_EDGE);
        for (; anExp1.More(); anExp1.Next(), anExp2.Next())
        {
            Assert.True(anExp1.Current().IsSame(anExp2.Current()));
        }
    }
}
