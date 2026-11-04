// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of: src/ModelingAlgorithms/TKBool/GTests/BRepFill_PipeShell_Test.cxx (OCCT V8_0_1), with OCCT.NET's checks of
// frozen inputs and the history added.

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>BRepFill_PipeShell (unit 5S2): OCCT's GTests, and the port's frozen inputs and history.</summary>
public class BRepFill_PipeShellTests
{
    private static Wire MakeSquareProfile(in Point3D theCenter, double theHalfSide)
    {
        MakePolygon aPolygon = new();
        aPolygon.Add(new Point3D(theCenter.X - theHalfSide, theCenter.Y - theHalfSide, theCenter.Z));
        aPolygon.Add(new Point3D(theCenter.X + theHalfSide, theCenter.Y - theHalfSide, theCenter.Z));
        aPolygon.Add(new Point3D(theCenter.X + theHalfSide, theCenter.Y + theHalfSide, theCenter.Z));
        aPolygon.Add(new Point3D(theCenter.X - theHalfSide, theCenter.Y + theHalfSide, theCenter.Z));
        aPolygon.Close();
        return aPolygon.Wire();
    }

    private static string Text(in Shape theShape)
    {
        using StringWriter aW = new();
        BrepTools.Write(theShape, aW);
        return aW.ToString();
    }

    [Fact]
    public void MakeSolid_StraightSpineClosedProfile_ProducesClosedSolid()
    {
        Edge aSpineEdge = new MakeEdge(new Point3D(0.0, 0.0, 0.0), new Point3D(0.0, 0.0, 10.0)).Edge();
        Wire aSpine = new MakeWire(aSpineEdge).Wire();

        Wire aProfile = MakeSquareProfile(new Point3D(0.0, 0.0, 0.0), 1.0);

        PipeShell aPipe = new(aSpine);
        aPipe.Add(aProfile);
        Assert.True(aPipe.IsReady);
        Assert.True(aPipe.Build());
        Assert.True(aPipe.MakeSolid());

        Shape aResult = aPipe.Shape();
        Assert.False(aResult.IsNull);
        Assert.True(aResult.IsClosed);

        Analyzer anAnalyzer = new(aResult);
        Assert.True(anAnalyzer.IsValid());
    }

    [Fact]
    public void MakeSolid_PolylineSpineClosedProfile_ProducesClosedSolid()
    {
        MakePolygon aSpinePolygon = new();
        aSpinePolygon.Add(new Point3D(0.0, 0.0, 0.0));
        aSpinePolygon.Add(new Point3D(0.0, 0.0, 5.0));
        aSpinePolygon.Add(new Point3D(0.0, 5.0, 10.0));
        Wire aSpine = aSpinePolygon.Wire();

        Wire aProfile = MakeSquareProfile(new Point3D(0.0, 0.0, 0.0), 0.5);

        PipeShell aPipe = new(aSpine);
        aPipe.SetTransition(TransitionStyle.Right);
        aPipe.Add(aProfile);
        Assert.True(aPipe.IsReady);
        Assert.True(aPipe.Build());
        Assert.True(aPipe.MakeSolid());

        Shape aResult = aPipe.Shape();
        Assert.False(aResult.IsNull);
        Assert.True(aResult.IsClosed);
    }

    [Theory]
    [InlineData(TransitionStyle.Modified)]
    [InlineData(TransitionStyle.Right)]
    [InlineData(TransitionStyle.Round)]
    public void Frozen_inputs_are_unchanged_and_the_history_answers_for_them(object theTransitionValue)
    {
        var theTransition = (TransitionStyle)theTransitionValue; // the theory's data as object: the engine type is internal
        // OCCT.NET: a closed spine whose wire isn't flagged closed (the pipe closes a new wire of the caller's edges)
        MakePolygon aSpinePolygon = new();
        aSpinePolygon.Add(new Point3D(0.0, 0.0, 0.0));
        aSpinePolygon.Add(new Point3D(0.0, 0.0, 5.0));
        aSpinePolygon.Add(new Point3D(0.0, 5.0, 10.0));
        Wire aSpine = aSpinePolygon.Wire();
        Wire aProfile = MakeSquareProfile(new Point3D(0.0, 0.0, 0.0), 0.5);
        Assert.True(aSpine.TShape()!.IsFrozen);
        Assert.True(aProfile.TShape()!.IsFrozen);
        string aSpineText = Text(aSpine), aProfileText = Text(aProfile);

        PipeShell aPipe = new(aSpine);
        aPipe.SetTransition(theTransition);
        aPipe.Add(aProfile);
        Assert.True(aPipe.Build());
        Assert.True(aPipe.MakeSolid());
        Assert.Equal(aSpineText, Text(aSpine));
        Assert.Equal(aProfileText, Text(aProfile));

        Shape aResult = aPipe.Shape();
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(ShapeType.Solid, aResult.Type);

        // every edge of the profile generates faces of the result, every edge of the spine too
        IndexedSet<Shape> aFaces = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        global::Kerf.Engine.Topology.Exploration.MapShapes(aResult, ShapeType.Face, aFaces);
        NCollection_List<Shape> aList = new();
        foreach (ShapeType aType in new[] { ShapeType.Edge, ShapeType.Vertex })
        {
            foreach (Shape aWire in new Shape[] { aProfile, aSpine })
            {
                for (global::Kerf.Engine.Topology.Explorer anExp = new(aWire, aType); anExp.More(); anExp.Next())
                {
                    aPipe.Generated(anExp.Current(), aList);
                    if (aType == ShapeType.Edge)
                    {
                        Assert.False(aList.IsEmpty());
                    }
                    for (NCollection_List<Shape>.Iterator anIt = new(aList); anIt.More(); anIt.Next())
                    {
                        Assert.True(anIt.Value().TShape()!.IsFrozen);
                        if (anIt.Value().Type == ShapeType.Face)
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
        MakePolygon aSpinePolygon = new();
        aSpinePolygon.Add(new Point3D(0.0, 0.0, 0.0));
        aSpinePolygon.Add(new Point3D(10.0, 0.0, 0.0));
        aSpinePolygon.Add(new Point3D(10.0, 10.0, 0.0));
        aSpinePolygon.Add(new Point3D(0.0, 0.0, 0.0));
        Wire aSpine = aSpinePolygon.Wire();
        BrepBuilder aB = new();
        Wire anOpen = new();
        aB.MakeWire(ref anOpen);
        for (global::Kerf.Engine.Topology.Iterator anIt = new(aSpine); anIt.More(); anIt.Next())
        {
            aB.Add(anOpen, anIt.Value());
        }
        anOpen.TShape()!.FreezeGraph();
        Assert.False(anOpen.IsClosed);

        PipeShell aPipe = new(anOpen);
        Assert.False(anOpen.IsClosed);
        Wire aPipeSpine = aPipe.Spine();
        Assert.True(aPipeSpine.IsClosed);
        Assert.False(aPipeSpine.IsSame(anOpen));
        global::Kerf.Engine.Topology.Explorer anExp1 = new(anOpen, ShapeType.Edge), anExp2 = new(aPipeSpine, ShapeType.Edge);
        for (; anExp1.More(); anExp1.Next(), anExp2.Next())
        {
            Assert.True(anExp1.Current().IsSame(anExp2.Current()));
        }
    }
}
