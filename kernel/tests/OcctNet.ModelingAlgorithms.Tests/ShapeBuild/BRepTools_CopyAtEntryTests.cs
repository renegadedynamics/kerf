// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The copy at the entry of a healing algorithm (decision 5008) and the composition of its context and histories with
/// the copy's, on a toy algorithm written as ShapeFix's are: it edits the shape in place and records replacements in a
/// ShapeBuild_ReShape context. This is the pattern units 6H2, 6H3, 6S and 6U follow.
/// </summary>
public class BRepTools_CopyAtEntryTests
{
    private static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(10, 20, 30).Shape();

    private static List<TopoDS_Shape> SubShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        var aList = new List<TopoDS_Shape>();
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            aList.Add(aMap[i]);
        }
        return aList;
    }

    private static bool HasTShapeOf(in TopoDS_Shape theShape, in TopoDS_Shape theSub, TopAbs_ShapeEnum theType)
    {
        foreach (TopoDS_Shape aSub in SubShapes(theShape, theType))
        {
            if (ReferenceEquals(aSub.TShape(), theSub.TShape()))
            {
                return true;
            }
        }
        return false;
    }

    // A face not holding the vertex.
    private static TopoDS_Face FaceWithout(in TopoDS_Shape theShape, in TopoDS_Shape theVertex, int theSkip = 0)
    {
        foreach (TopoDS_Shape aF in SubShapes(theShape, TopAbs_FACE))
        {
            if (!HasTShapeOf(aF, theVertex, TopAbs_VERTEX) && theSkip-- == 0)
            {
                return TopoDS.Face(aF);
            }
        }
        throw new InvalidOperationException();
    }

    /// <summary>
    /// A toy healing algorithm, written as a ShapeFix class is: the shape it is given is copied at the entry if
    /// frozen, then OCCT's code runs on the copy (an in-place tolerance raise on <paramref name="theVertex"/>, the
    /// replacement of <paramref name="theFace"/> by a copy recorded in the context, the context applied), and the end
    /// gives the caller's sub-shapes back.
    /// </summary>
    private static TopoDS_Shape ToyFix(in TopoDS_Shape theShape, ShapeBuild_ReShape theContext, in TopoDS_Shape theVertex,
                                       in TopoDS_Shape theFace, out BRepTools_CopyAtEntry theEntry)
    {
        // 1. the entry: the shape to work on; arguments referring to the input's sub-shapes are mapped onto the copy
        var anEntry = new BRepTools_CopyAtEntry(theShape, theContext);
        TopoDS_Shape aWork = anEntry.Shape();
        TopoDS_Vertex aV = TopoDS.Vertex(anEntry.Copied(theVertex));
        TopoDS_Face aF = TopoDS.Face(anEntry.Copied(theFace));

        // 2. OCCT's code, unchanged, on the copy
        var aB = new BRep_Builder();
        if (!aV.IsNull())
        {
            aB.UpdateVertex(aV, 0.01); // in place, unrecorded (as ShapeFix_Edge's tolerance fixes)
        }
        if (!aF.IsNull())
        {
            TopoDS_Shape aNewF = aF.EmptyCopied();
            for (var anIt = new TopoDS_Iterator(aF, false, false); anIt.More(); anIt.Next())
            {
                aB.Add(aNewF, anIt.Value());
            }
            theContext.Replace(aF, aNewF); // recorded (as ShapeFix_Face's)
        }
        TopoDS_Shape aResult = theContext.InternalApply(aWork);

        // 3. the end: the result with the caller's unchanged sub-shapes, the context re-keyed to the caller's shapes
        aResult = anEntry.Finish(aResult);
        theEntry = anEntry;
        return aResult; // the algorithm's public Shape() freezes it
    }

    [Fact]
    public void FrozenInput_IsUnchanged_AndUnchangedFacesAreTheCallers()
    {
        TopoDS_Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        TopoDS_Shape aVertex = SubShapes(aBox, TopAbs_VERTEX)[0];
        TopoDS_Face aFace = FaceWithout(aBox, aVertex);
        var aContext = new ShapeBuild_ReShape();

        TopoDS_Shape aResult = ToyFix(aBox, aContext, aVertex, aFace, out BRepTools_CopyAtEntry anEntry);

        aSnap.AssertUnchanged(aBox);
        Assert.True(anEntry.IsCopied());
        Assert.False(ReferenceEquals(aResult.TShape(), aBox.TShape()));

        // of the 6 faces: 3 hold the vertex (changed in place), 1 was replaced; 2 are the caller's own
        int aNbKept = 0;
        foreach (TopoDS_Shape aF in SubShapes(aResult, TopAbs_FACE))
        {
            if (HasTShapeOf(aBox, aF, TopAbs_FACE))
            {
                aNbKept++;
            }
        }
        Assert.Equal(6, SubShapes(aResult, TopAbs_FACE).Count);
        Assert.Equal(2, aNbKept);

        // the edges that don't touch the vertex are the caller's (12 - 3)
        int aNbKeptEdges = 0;
        foreach (TopoDS_Shape anE in SubShapes(aResult, TopAbs_EDGE))
        {
            if (HasTShapeOf(aBox, anE, TopAbs_EDGE))
            {
                aNbKeptEdges++;
            }
        }
        Assert.Equal(9, aNbKeptEdges);
        Assert.False(aResult.TShape()!.IsFrozen()); // the helper freezes nothing
    }

    [Fact]
    public void Context_AnswersForTheCallersSubShapes()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Shape aVertex = SubShapes(aBox, TopAbs_VERTEX)[0];
        TopoDS_Face aFace = FaceWithout(aBox, aVertex);
        var aContext = new ShapeBuild_ReShape();

        TopoDS_Shape aResult = ToyFix(aBox, aContext, aVertex, aFace, out _);

        // the changed vertex: recorded as replaced by its changed copy, which is in the result
        TopoDS_Shape aNewV = aContext.Value(aVertex);
        Assert.False(ReferenceEquals(aNewV.TShape(), aVertex.TShape()));
        Assert.Equal(0.01, BRep_Tool.Tolerance(TopoDS.Vertex(aNewV)));
        Assert.True(HasTShapeOf(aResult, aNewV, TopAbs_VERTEX));

        // the replaced face: its replacement is in the result
        TopoDS_Shape aNewF = aContext.Value(aFace);
        Assert.False(ReferenceEquals(aNewF.TShape(), aFace.TShape()));
        Assert.True(HasTShapeOf(aResult, aNewF, TopAbs_FACE));

        // a face that needed nothing: itself
        TopoDS_Face aKept = FaceWithout(aBox, aVertex, 1);
        Assert.True(ReferenceEquals(aContext.Value(aKept).TShape(), aKept.TShape()));
        Assert.False(aContext.IsRecorded(aKept));

        // Apply of the caller's shape gives the result
        TopoDS_Shape anApplied = aContext.Apply(aBox);
        Assert.True(ReferenceEquals(anApplied.TShape(), aResult.TShape()));

        // the context's history is over the caller's sub-shapes only
        BRepTools_History aHistory = aContext.History();
        Assert.Single(aHistory.Modified(aVertex));
        Assert.Single(aHistory.Modified(aFace));
        Assert.Empty(aHistory.Modified(aKept));
    }

    [Fact]
    public void History_AndComposedAlgorithmHistory()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Shape aVertex = SubShapes(aBox, TopAbs_VERTEX)[0];
        TopoDS_Face aFace = FaceWithout(aBox, aVertex);
        var aContext = new ShapeBuild_ReShape();

        // the context's history is taken over the copy before Finish, as an algorithm keeping its own would
        var anEntry = new BRepTools_CopyAtEntry(aBox, aContext);
        TopoDS_Face aF = TopoDS.Face(anEntry.Copied(aFace));
        TopoDS_Shape aNewF = aF.EmptyCopied();
        var aB = new BRep_Builder();
        for (var anIt = new TopoDS_Iterator(aF, false, false); anIt.More(); anIt.Next())
        {
            aB.Add(aNewF, anIt.Value());
        }
        aB.UpdateVertex(TopoDS.Vertex(anEntry.Copied(aVertex)), 0.01);
        var anAlgoHistory = new BRepTools_History();
        anAlgoHistory.AddModified(aF, aNewF);
        TopoDS_Shape aRemovedE = anEntry.Copied(SubShapes(aBox, TopAbs_EDGE)[5]);
        anAlgoHistory.Remove(aRemovedE);
        TopoDS_Shape aResult = anEntry.Finish(new TopoDS_Shape(), [aNewF]);

        Assert.True(aResult.IsNull());
        BRepTools_History aComposed = anEntry.Composed(anAlgoHistory);
        Assert.True(ReferenceEquals(aComposed.Modified(aFace)[0].TShape(), aNewF.TShape()));
        Assert.True(aComposed.IsRemoved(SubShapes(aBox, TopAbs_EDGE)[5]));
        Assert.Single(aComposed.Modified(aVertex)); // changed in place: modified into its copy
        Assert.Single(anEntry.History().Modified(aVertex));

        // the new face holds the caller's edges where they are unchanged
        int aNbCallers = 0;
        foreach (TopoDS_Shape anE in SubShapes(aNewF, TopAbs_EDGE))
        {
            if (HasTShapeOf(aBox, anE, TopAbs_EDGE))
            {
                aNbCallers++;
            }
        }
        Assert.Equal(4, aNbCallers);
    }

    [Fact]
    public void NothingChanged_GivesTheInputItself()
    {
        TopoDS_Shape aBox = Box();
        var aContext = new ShapeBuild_ReShape();
        TopoDS_Shape aResult = ToyFix(aBox, aContext, new TopoDS_Vertex(), new TopoDS_Face(), out BRepTools_CopyAtEntry anEntry);

        Assert.True(anEntry.IsCopied());
        Assert.True(ReferenceEquals(aResult.TShape(), aBox.TShape()));
        Assert.True(anEntry.History().Modified(SubShapes(aBox, TopAbs_FACE)[0]).Count == 0);
        Assert.False(aContext.History().HasModified());
    }

    [Fact]
    public void UnfrozenInput_RunsInPlace()
    {
        TopoDS_Shape aBox = BRepTools.ExactCopyForWrite(Box(), out _); // owned by ported code: unfrozen
        TopoDS_Shape aVertex = SubShapes(aBox, TopAbs_VERTEX)[0];
        var aContext = new ShapeBuild_ReShape();
        TopoDS_Shape aResult = ToyFix(aBox, aContext, aVertex, new TopoDS_Face(), out BRepTools_CopyAtEntry anEntry);

        Assert.False(anEntry.IsCopied());
        Assert.True(ReferenceEquals(aResult.TShape(), aBox.TShape()));
        Assert.Equal(0.01, BRep_Tool.Tolerance(TopoDS.Vertex(aVertex))); // OCCT's in-place edit
    }

    [Fact]
    public void RequestsRecordedBeforeTheEntry_AreApplied()
    {
        TopoDS_Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        TopoDS_Shape aFace = SubShapes(aBox, TopAbs_FACE)[2];
        var aContext = new ShapeBuild_ReShape();
        aContext.Remove(aFace); // a caller's request on its own face

        TopoDS_Shape aResult = ToyFix(aBox, aContext, new TopoDS_Vertex(), new TopoDS_Face(), out _);

        aSnap.AssertUnchanged(aBox);
        Assert.Equal(5, SubShapes(aResult, TopAbs_FACE).Count);
        Assert.True(aContext.IsRecorded(aFace));
        Assert.True(aContext.Value(aFace).IsNull());
        Assert.True(aContext.History().IsRemoved(aFace));
    }

    [Fact]
    public void Finish_IsCalledOnce()
    {
        var anEntry = new BRepTools_CopyAtEntry(Box());
        Assert.ThrowsAny<Standard_ProgramError>(() => anEntry.History());
        anEntry.Finish(anEntry.Shape());
        Assert.ThrowsAny<Standard_ProgramError>(() => anEntry.Finish(anEntry.Shape()));
    }
}
