// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The working overlay of a healing algorithm and the composition of its context and histories with the overlay's, on
/// a toy algorithm written as ShapeFix's are: it edits the shape in place and records replacements in a ShapeBuild_ReShape
/// context. Until unit 10H6 these tests ran the same toy on BRepTools_CopyAtEntry (decision 583), the copy at the entry
/// that the overlay replaced; the assertions on the results are theirs.
/// </summary>
public class WorkingOverlay_HealingPatternTests
{
    private static Shape Box() => new MakeBox(10, 20, 30).Shape();

    private static List<Shape> SubShapes(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        var aList = new List<Shape>();
        for (int i = 0; i < aMap.Count; i++)
        {
            aList.Add(aMap[i]);
        }
        return aList;
    }

    private static bool HasTShapeOf(in Shape theShape, in Shape theSub, ShapeType theType)
    {
        foreach (Shape aSub in SubShapes(theShape, theType))
        {
            if (ReferenceEquals(aSub.TShape(), theSub.TShape()))
            {
                return true;
            }
        }
        return false;
    }

    // A face not holding the vertex.
    private static Face FaceWithout(in Shape theShape, in Shape theVertex, int theSkip = 0)
    {
        foreach (Shape aF in SubShapes(theShape, ShapeType.Face))
        {
            if (!HasTShapeOf(aF, theVertex, ShapeType.Vertex) && theSkip-- == 0)
            {
                return ShapeCasts.Face(aF);
            }
        }
        throw new InvalidOperationException();
    }

    /// <summary>
    /// A toy healing algorithm, written as a ShapeFix class is: an overlay is opened on the shape it is given, then
    /// OCCT's code runs on the shape itself (an in-place tolerance raise on <paramref name="theVertex"/>, the
    /// replacement of <paramref name="theFace"/> by a copy recorded in the context, the context applied), and the end
    /// assembles the result, sharing the caller's unchanged sub-shapes.
    /// </summary>
    private static Shape ToyFix(in Shape theShape, ReShapeBuilder theContext, in Shape theVertex,
                                       in Shape theFace, out WorkingOverlay theOverlay)
    {
        // 1. the entry: the overlay; the shape and the arguments are the caller's own
        using var anOverlay = WorkingOverlay.Begin(theShape, theContext);
        Shape aWork = theShape;
        Vertex aV = ShapeCasts.Vertex(theVertex);
        Face aF = ShapeCasts.Face(theFace);

        // 2. OCCT's code, unchanged, on the caller's shape
        var aB = new BrepBuilder();
        if (!aV.IsNull)
        {
            aB.UpdateVertex(aV, 0.01); // in place, unrecorded (as ShapeFix_Edge's tolerance fixes)
        }
        if (!aF.IsNull)
        {
            Shape aNewF = aF.EmptyCopied();
            for (var anIt = new Iterator(aF, false, false); anIt.More(); anIt.Next())
            {
                aB.Add(aNewF, anIt.Value());
            }
            theContext.Replace(aF, aNewF); // recorded (as ShapeFix_Face's)
        }
        Shape aResult = theContext.InternalApply(aWork);

        // 3. the end: the result with the caller's unchanged sub-shapes, the context brought up to date
        aResult = anOverlay.Finish(aResult);
        theOverlay = anOverlay;
        return aResult; // the algorithm's public Shape() freezes it
    }

    [Fact]
    public void FrozenInput_IsUnchanged_AndUnchangedFacesAreTheCallers()
    {
        Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        Shape aVertex = SubShapes(aBox, ShapeType.Vertex)[0];
        Face aFace = FaceWithout(aBox, aVertex);
        var aContext = new ReShapeBuilder();

        Shape aResult = ToyFix(aBox, aContext, aVertex, aFace, out WorkingOverlay anOverlay);

        aSnap.AssertUnchanged(aBox);
        Assert.NotEqual(0, anOverlay.WorkingCount);
        Assert.False(ReferenceEquals(aResult.TShape(), aBox.TShape()));

        // of the 6 faces: 3 hold the vertex (changed in place), 1 was replaced; 2 are the caller's own
        int aNbKept = 0;
        foreach (Shape aF in SubShapes(aResult, ShapeType.Face))
        {
            if (HasTShapeOf(aBox, aF, ShapeType.Face))
            {
                aNbKept++;
            }
        }
        Assert.Equal(6, SubShapes(aResult, ShapeType.Face).Count);
        Assert.Equal(2, aNbKept);

        // the edges that don't touch the vertex are the caller's (12 - 3)
        int aNbKeptEdges = 0;
        foreach (Shape anE in SubShapes(aResult, ShapeType.Edge))
        {
            if (HasTShapeOf(aBox, anE, ShapeType.Edge))
            {
                aNbKeptEdges++;
            }
        }
        Assert.Equal(9, aNbKeptEdges);
        Assert.False(aResult.TShape()!.IsFrozen); // the overlay freezes nothing
    }

    [Fact]
    public void Context_AnswersForTheCallersSubShapes()
    {
        Shape aBox = Box();
        Shape aVertex = SubShapes(aBox, ShapeType.Vertex)[0];
        Face aFace = FaceWithout(aBox, aVertex);
        var aContext = new ReShapeBuilder();

        Shape aResult = ToyFix(aBox, aContext, aVertex, aFace, out _);

        // the changed vertex: recorded as replaced by its changed copy, which is in the result
        Shape aNewV = aContext.Value(aVertex);
        Assert.False(ReferenceEquals(aNewV.TShape(), aVertex.TShape()));
        Assert.Equal(0.01, Tool.Tolerance(ShapeCasts.Vertex(aNewV)));
        Assert.True(HasTShapeOf(aResult, aNewV, ShapeType.Vertex));

        // the replaced face: its replacement is in the result
        Shape aNewF = aContext.Value(aFace);
        Assert.False(ReferenceEquals(aNewF.TShape(), aFace.TShape()));
        Assert.True(HasTShapeOf(aResult, aNewF, ShapeType.Face));

        // a face that needed nothing: itself
        Face aKept = FaceWithout(aBox, aVertex, 1);
        Assert.True(ReferenceEquals(aContext.Value(aKept).TShape(), aKept.TShape()));
        Assert.False(aContext.IsRecorded(aKept));

        // Apply of the caller's shape gives the result
        Shape anApplied = aContext.Apply(aBox);
        Assert.True(ReferenceEquals(anApplied.TShape(), aResult.TShape()));

        // the context's history is over the caller's sub-shapes only
        History aHistory = aContext.History();
        Assert.Single(aHistory.Modified(aVertex));
        Assert.Single(aHistory.Modified(aFace));
        Assert.Empty(aHistory.Modified(aKept));
    }

    [Fact]
    public void History_AndComposedAlgorithmHistory()
    {
        Shape aBox = Box();
        Shape aVertex = SubShapes(aBox, ShapeType.Vertex)[0];
        Face aFace = FaceWithout(aBox, aVertex);
        var aContext = new ReShapeBuilder();

        // the algorithm's history is kept over the caller's sub-shapes, as an algorithm keeping its own would
        using var anOverlay = WorkingOverlay.Begin(aBox, aContext);
        Face aF = aFace;
        Shape aNewF = aF.EmptyCopied();
        var aB = new BrepBuilder();
        for (var anIt = new Iterator(aF, false, false); anIt.More(); anIt.Next())
        {
            aB.Add(aNewF, anIt.Value());
        }
        aB.UpdateVertex(ShapeCasts.Vertex(aVertex), 0.01);
        var anAlgoHistory = new History();
        anAlgoHistory.AddModified(aF, aNewF);
        Shape aRemovedE = SubShapes(aBox, ShapeType.Edge)[5];
        anAlgoHistory.Remove(aRemovedE);
        Shape aResult = anOverlay.Finish(new Shape(), [aNewF]);

        Assert.True(aResult.IsNull);
        History aComposed = anOverlay.Composed(anAlgoHistory);
        Assert.True(ReferenceEquals(aComposed.Modified(aFace)[0].TShape(), aNewF.TShape()));
        Assert.True(aComposed.IsRemoved(SubShapes(aBox, ShapeType.Edge)[5]));
        Assert.Single(aComposed.Modified(aVertex)); // changed in place: modified into what stands for it
        Assert.Single(anOverlay.History().Modified(aVertex));

        // the new face holds the caller's edges where they are unchanged
        int aNbCallers = 0;
        foreach (Shape anE in SubShapes(aNewF, ShapeType.Edge))
        {
            if (HasTShapeOf(aBox, anE, ShapeType.Edge))
            {
                aNbCallers++;
            }
        }
        Assert.Equal(4, aNbCallers);
    }

    [Fact]
    public void NothingChanged_GivesTheInputItself()
    {
        Shape aBox = Box();
        var aContext = new ReShapeBuilder();
        Shape aResult = ToyFix(aBox, aContext, new Vertex(), new Face(), out WorkingOverlay anOverlay);

        Assert.Equal(0, anOverlay.WorkingCount);
        Assert.True(ReferenceEquals(aResult.TShape(), aBox.TShape()));
        Assert.True(anOverlay.History().Modified(SubShapes(aBox, ShapeType.Face)[0]).Count == 0);
        Assert.False(aContext.History().HasModified);
    }

    [Fact]
    public void UnfrozenInput_RunsInPlace()
    {
        Shape aBox = TestCopies.ExactCopyForWrite(Box(), out _); // owned by ported code: unfrozen
        Shape aVertex = SubShapes(aBox, ShapeType.Vertex)[0];
        var aContext = new ReShapeBuilder();
        Shape aResult = ToyFix(aBox, aContext, aVertex, new Face(), out WorkingOverlay anOverlay);

        Assert.Equal(0, anOverlay.WorkingCount);
        Assert.True(ReferenceEquals(aResult.TShape(), aBox.TShape()));
        Assert.Equal(0.01, Tool.Tolerance(ShapeCasts.Vertex(aVertex))); // OCCT's in-place edit
    }

    [Fact]
    public void RequestsRecordedBeforeTheEntry_AreApplied()
    {
        Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        Shape aFace = SubShapes(aBox, ShapeType.Face)[2];
        var aContext = new ReShapeBuilder();
        aContext.Remove(aFace); // a caller's request on its own face

        Shape aResult = ToyFix(aBox, aContext, new Vertex(), new Face(), out _);

        aSnap.AssertUnchanged(aBox);
        Assert.Equal(5, SubShapes(aResult, ShapeType.Face).Count);
        Assert.True(aContext.IsRecorded(aFace));
        Assert.True(aContext.Value(aFace).IsNull);
        Assert.True(aContext.History().IsRemoved(aFace));
    }

    [Fact]
    public void Finish_IsCalledOnce()
    {
        Shape aBox = Box();
        using var anOverlay = WorkingOverlay.Begin(aBox);
        EngineAssert.ThrowsAny<ProgramErrorException>(() => anOverlay.History());
        anOverlay.Finish(aBox);
        EngineAssert.ThrowsAny<ProgramErrorException>(() => anOverlay.Finish(aBox));
    }
}
