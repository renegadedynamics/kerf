using System.Collections.Concurrent;
using OcctNet.TestHarness;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests.Harness;

public class HarnessTests
{
    // ---- Shapes

    private static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();

    private static IEnumerable<TopoDS_Shape> Primitives()
    {
        yield return Box();
        yield return new BRepPrimAPI_MakeCylinder(5.0, 12.0).Shape();
        yield return new BRepPrimAPI_MakeSphere(7.0).Shape();
        yield return new BRepPrimAPI_MakeTorus(10.0, 3.0).Shape();
        yield return new BRepPrimAPI_MakeCone(6.0, 2.0, 9.0).Shape();
        yield return SharedCompound();
        yield return BoxShell(skipFace: -1);
    }

    // A compound of several solids, one of them twice at different locations (shared, located sub-shapes).
    private static TopoDS_Shape SharedCompound()
    {
        TopoDS_Shape aBox = Box();
        var aTrsf = new gp_Trsf();
        aTrsf.SetTranslation(new gp_Vec(100.0, 0.0, 0.0));
        var aBuilder = new BRep_Builder();
        var aCompound = new TopoDS_Compound();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aBox);
        aBuilder.Add(aCompound, aBox.Located(new TopLoc_Location(aTrsf)));
        aBuilder.Add(aCompound, new BRepPrimAPI_MakeCylinder(5.0, 12.0).Shape());
        aBuilder.Add(aCompound, new BRepPrimAPI_MakeSphere(4.0).Shape());
        return aCompound;
    }

    // A shell of a box's faces, without the face of the given index (-1: all six).
    private static TopoDS_Shape BoxShell(int skipFace)
    {
        var aBuilder = new BRep_Builder();
        var aShell = new TopoDS_Shell();
        aBuilder.MakeShell(ref aShell);
        int i = 0;
        foreach (TopoDS_Shape aFace in CanonicalShapeComparer.SubShapes(Box(), TopAbs_FACE))
        {
            if (i++ != skipFace)
            {
                aBuilder.Add(aShell, aFace);
            }
        }
        return aShell;
    }

    // The same shape with the children of every compound, compsolid, solid and shell in a random order.
    private static TopoDS_Shape Renumber(TopoDS_Shape theShape, Random theRandom)
    {
        TopAbs_ShapeEnum aType = theShape.ShapeType();
        if (aType is not (TopAbs_COMPOUND or TopAbs_COMPSOLID or TopAbs_SOLID or TopAbs_SHELL))
        {
            return theShape;
        }
        var aChildren = new List<TopoDS_Shape>();
        for (var anIt = new TopoDS_Iterator(theShape, false, false); anIt.More(); anIt.Next())
        {
            aChildren.Add(Renumber(anIt.Value(), theRandom));
        }
        if (aChildren.Count > 1)
        {
            int[] anOrder;
            do
            {
                anOrder = [.. Enumerable.Range(0, aChildren.Count).OrderBy(_ => theRandom.Next())];
            }
            while (anOrder.SequenceEqual(Enumerable.Range(0, aChildren.Count)));
            aChildren = [.. anOrder.Select(i => aChildren[i])];
        }

        var aBuilder = new BRep_Builder();
        TopoDS_Shape aNew;
        switch (aType)
        {
            case TopAbs_COMPOUND:
                var aCompound = new TopoDS_Compound();
                aBuilder.MakeCompound(ref aCompound);
                aNew = aCompound;
                break;
            case TopAbs_COMPSOLID:
                var aCompSolid = new TopoDS_CompSolid();
                aBuilder.MakeCompSolid(ref aCompSolid);
                aNew = aCompSolid;
                break;
            case TopAbs_SOLID:
                var aSolid = new TopoDS_Solid();
                aBuilder.MakeSolid(ref aSolid);
                aNew = aSolid;
                break;
            default:
                var aShell = new TopoDS_Shell();
                aBuilder.MakeShell(ref aShell);
                aNew = aShell;
                break;
        }
        foreach (TopoDS_Shape aChild in aChildren)
        {
            aBuilder.Add(aNew, aChild);
        }
        aNew.Closed(theShape.Closed());
        aNew.Orientable(theShape.Orientable());
        aNew.Convex(theShape.Convex());
        aNew.Infinite(theShape.Infinite());
        return aNew.Located(theShape.Location()).Oriented(theShape.Orientation());
    }

    // ---- BRep text comparison

    [Fact]
    public void BRepText_Identical()
    {
        string aText = BRepTextComparison.Write(Box());
        BRepTextDiff aDiff = BRepTextComparison.Compare(aText, BRepTextComparison.Write(Box()));
        Assert.True(aDiff.IsIdentical, aDiff.ToString());
        Assert.Equal(0, aDiff.FreeFlagDifferences);
    }

    [Fact]
    public void BRepText_FreeFlagMaskedAndCounted()
    {
        string aText = BRepTextComparison.Write(Box());
        string[] aLines = aText.Replace("\r\n", "\n", StringComparison.Ordinal).Split('\n');
        int aTShapes = Array.FindIndex(aLines, l => l.StartsWith("TShapes ", StringComparison.Ordinal));
        int aFlags = Array.FindIndex(aLines, aTShapes, l => l.Length == 7 && l.All(c => c is '0' or '1'));
        aLines[aFlags] = (aLines[aFlags][0] == '0' ? "1" : "0") + aLines[aFlags][1..];
        string aPort = string.Join("\n", aLines);

        BRepTextDiff aDiff = BRepTextComparison.Compare(aText, aPort);
        Assert.False(aDiff.IsIdentical);
        Assert.True(aDiff.IsIdenticalMaskingFree, aDiff.ToString());
        Assert.Equal(1, aDiff.FreeFlagDifferences);
    }

    [Fact]
    public void BRepText_CoordinateChanged_FirstDifferingLine()
    {
        string aText = BRepTextComparison.Write(Box()).Replace("\r\n", "\n", StringComparison.Ordinal);
        string[] aLines = aText.Split('\n');
        int aTShapes = Array.FindIndex(aLines, l => l.StartsWith("TShapes ", StringComparison.Ordinal));
        int aVertex = Array.FindIndex(aLines, aTShapes, l => l == "Ve");
        string aChanged = aLines[aVertex + 2];
        aLines[aVertex + 2] = "1" + aChanged[1..];
        Assert.NotEqual(aChanged, aLines[aVertex + 2]);

        BRepTextDiff aDiff = BRepTextComparison.Compare(aText, string.Join("\n", aLines));
        Assert.False(aDiff.IsIdenticalMaskingFree);
        Assert.Equal(aVertex + 3, aDiff.FirstDifferenceLine);
        Assert.Equal(aChanged, aDiff.NativeLine);
    }

    // ---- Canonical comparison

    [Fact]
    public void Canonical_SameShapeRenumberedAtRandom_IsIdentical()
    {
        int aShapeIndex = 0;
        foreach (TopoDS_Shape aShape in Primitives())
        {
            string aText = BRepTextComparison.Write(aShape);
            for (int aSeed = 1; aSeed <= 4; ++aSeed)
            {
                TopoDS_Shape aRenumbered = Renumber(aShape, new Random(aSeed * 31 + aShapeIndex));
                string aRenumberedText = BRepTextComparison.Write(aRenumbered);
                if (CountChildren(aShape) > 1)
                {
                    Assert.NotEqual(aText, aRenumberedText);
                }
                CanonicalComparison aComparison = CanonicalShapeComparer.Compare(aShape, aRenumbered);
                Assert.True(aComparison.IsIdentical, $"shape {aShapeIndex}, seed {aSeed}: {aComparison}");
            }
            ++aShapeIndex;
        }
    }

    // Whether some container of the shape has two children (so that a renumbering changes the BRep text).
    private static int CountChildren(TopoDS_Shape theShape)
    {
        int aMax = 0;
        foreach (TopAbs_ShapeEnum aType in new[] { TopAbs_COMPOUND, TopAbs_COMPSOLID, TopAbs_SOLID, TopAbs_SHELL })
        {
            foreach (TopoDS_Shape aContainer in CanonicalShapeComparer.SubShapes(theShape, aType))
            {
                int aNb = 0;
                for (var anIt = new TopoDS_Iterator(aContainer); anIt.More(); anIt.Next())
                {
                    ++aNb;
                }
                aMax = Math.Max(aMax, aNb);
            }
        }
        return aMax;
    }

    [Fact]
    public void Canonical_Copies_AreIdentical()
    {
        foreach (TopoDS_Shape aShape in Primitives())
        {
            TopoDS_Shape aCopy = new BRepBuilderAPI_Copy(aShape).Shape();
            CanonicalComparison aComparison = CanonicalShapeComparer.Compare(aShape, aCopy);
            Assert.True(aComparison.IsIdentical, aComparison.ToString());

            TopoDS_Shape anExact = BRepTools.ExactCopyForWrite(aShape, out _);
            CanonicalShapeComparer.AssertIdentical(aShape, anExact);
        }
    }

    [Fact]
    public void Canonical_VertexMoved_DetectedExactly_MatchedWithTolerance()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(aBox, out _);
        TopoDS_Vertex aVertex = TopoDS.Vertex(CanonicalShapeComparer.SubShapes(aCopy, TopAbs_VERTEX)[0]);
        gp_Pnt aP = BRep_Tool.Pnt(aVertex);
        new BRep_Builder().UpdateVertex(aVertex, new gp_Pnt(aP.X() + 1.0e-12, aP.Y(), aP.Z()), BRep_Tool.Tolerance(aVertex));

        CanonicalComparison anExact = CanonicalShapeComparer.Compare(aBox, aCopy);
        Assert.False(anExact.IsIdentical);
        Assert.Contains(anExact.Unmatched, l => l.StartsWith("TopAbs_VERTEX", StringComparison.Ordinal));

        CanonicalComparison aTolerant = CanonicalShapeComparer.Compare(aBox, aCopy, new CanonicalOptions { VertexTolerance = 1.0e-9 });
        Assert.True(aTolerant.IsIdentical, aTolerant.ToString());
    }

    [Fact]
    public void Canonical_ToleranceChanged_ReportedAsAttributeDifference()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(aBox, out _);
        TopoDS_Vertex aVertex = TopoDS.Vertex(CanonicalShapeComparer.SubShapes(aCopy, TopAbs_VERTEX)[3]);
        new BRep_Builder().UpdateVertex(aVertex, 1.0e-4);

        CanonicalComparison aComparison = CanonicalShapeComparer.Compare(aBox, aCopy);
        Assert.False(aComparison.IsIdentical);
        Assert.Empty(aComparison.Unmatched);
        Assert.Single(aComparison.AttributeDifferences);
        Assert.StartsWith("TopAbs_VERTEX", aComparison.AttributeDifferences[0], StringComparison.Ordinal);
    }

    [Fact]
    public void Canonical_FaceRemoved_UnmatchedAndCountsDiffer()
    {
        CanonicalComparison aComparison = CanonicalShapeComparer.Compare(BoxShell(-1), BoxShell(2),
                                                                         new CanonicalOptions { CheckValidity = false });
        Assert.False(aComparison.IsIdentical);
        Assert.False(aComparison.CountsEqual);
        Assert.Equal((6, 5), aComparison.Counts[TopAbs_FACE]);
        Assert.Contains(aComparison.Unmatched, l => l.StartsWith("TopAbs_FACE", StringComparison.Ordinal));
    }

    [Fact]
    public void Canonical_TranslatedShape_IsNotIdentical()
    {
        TopoDS_Shape aBox = Box();
        var aTrsf = new gp_Trsf();
        aTrsf.SetTranslation(new gp_Vec(0.0, 0.0, 1.0e-6));
        CanonicalComparison aComparison = CanonicalShapeComparer.Compare(aBox, aBox.Located(new TopLoc_Location(aTrsf)));
        Assert.False(aComparison.IsIdentical);
        Assert.True(aComparison.CountsEqual);
        Assert.NotEmpty(aComparison.Unmatched);
        Assert.NotEmpty(aComparison.PropertyDifferences);
    }

    [Fact]
    public void Canonical_Orientation_IgnoredByDefault_CheckedOnRequest()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Shape aReversed = aBox.Reversed();
        Assert.True(CanonicalShapeComparer.Compare(aBox, aReversed, new CanonicalOptions { CheckProperties = false }).IsIdentical);
        CanonicalComparison aChecked = CanonicalShapeComparer.Compare(aBox, aReversed,
                                                                      new CanonicalOptions { CheckOrientation = true, CheckProperties = false });
        Assert.False(aChecked.IsIdentical);
        Assert.NotNull(aChecked.OrientationDifference);
    }

    // ---- Determinism harness

    [Fact]
    public void Determinism_DeterministicParallelComputation_Passes()
    {
        string aReference = DeterminismHarness.AssertSerialEqualsParallel(() =>
        {
            double[] aValues = new double[2000];
            OSD_Parallel.For(0, aValues.Length, i => aValues[i] = Math.Sqrt(i) * Math.Sin(i));
            return string.Join("\n", aValues.Select(v => BitConverter.DoubleToInt64Bits(v)));
        });
        Assert.NotEmpty(aReference);
    }

    [Fact]
    public void Determinism_NondeterministicComputation_Fails()
    {
        if (Environment.ProcessorCount < 2)
        {
            return; // no parallelism to show
        }
        Assert.ThrowsAny<Exception>(() => DeterminismHarness.AssertSerialEqualsParallel(() =>
        {
            var anOrder = new ConcurrentQueue<int>();
            OSD_Parallel.For(0, 20000, i =>
            {
                Thread.SpinWait(50);
                anOrder.Enqueue(i);
            });
            return string.Join("\n", anOrder);
        }, 3, 2, 4));
    }

    [Fact]
    public void Determinism_ShapeAndHistory()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Shape aResult = DeterminismHarness.AssertSerialEqualsParallel(() => new BRepBuilderAPI_Copy(aBox).Shape(), 2);
        Assert.False(aResult.IsNull());

        DeterminismHarness.AssertSerialEqualsParallel([aBox], () =>
        {
            TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(aBox, out BRepTools_History aHistory);
            return (aCopy, aHistory);
        }, 2);

        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(aBox, out BRepTools_History aHistory);
        string aText = DeterminismHarness.HistoryText([aBox], aCopy, aHistory);
        Assert.Contains("TopAbs_FACE M[", aText, StringComparison.Ordinal);
        Assert.DoesNotContain("M[0", aText, StringComparison.Ordinal);
    }

    // ---- History comparison

    [Fact]
    public void History_SameUpToRenumbering_Agrees()
    {
        // Arguments and result renumbered at the top level only: the history knows the solids and below
        TopoDS_Shape anArgument = SharedCompound();
        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(anArgument, out BRepTools_History aHistory);
        TopoDS_Shape aPortArgument = ShuffleTopLevel(anArgument, new Random(5));
        TopoDS_Shape aPortCopy = ShuffleTopLevel(aCopy, new Random(6));
        Assert.NotEqual(BRepTextComparison.Write(aCopy), BRepTextComparison.Write(aPortCopy));

        HistoryComparison.AssertSame([anArgument], aCopy, aHistory, [aPortArgument], aPortCopy, aHistory);

        // An independent copy: different TShapes on the port side, the same geometry
        TopoDS_Shape aCopy2 = BRepTools.ExactCopyForWrite(anArgument, out BRepTools_History aHistory2);
        HistoryComparison.AssertSame([anArgument], aCopy, aHistory, [anArgument], aCopy2, aHistory2);
    }

    // The compound with its direct children in reverse order (the children kept as they are).
    private static TopoDS_Shape ShuffleTopLevel(TopoDS_Shape theCompound, Random theRandom)
    {
        var aChildren = new List<TopoDS_Shape>();
        for (var anIt = new TopoDS_Iterator(theCompound, false, false); anIt.More(); anIt.Next())
        {
            aChildren.Add(anIt.Value());
        }
        var aBuilder = new BRep_Builder();
        var aNew = new TopoDS_Compound();
        aBuilder.MakeCompound(ref aNew);
        foreach (TopoDS_Shape aChild in aChildren.OrderBy(_ => theRandom.Next()).Reverse())
        {
            aBuilder.Add(aNew, aChild);
        }
        return aNew;
    }

    [Fact]
    public void History_Different_IsReported()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(aBox, out BRepTools_History aHistory);
        var anEmpty = new BRepTools_History();
        TopoDS_Shape aFace = CanonicalShapeComparer.SubShapes(aBox, TopAbs_FACE)[0];
        anEmpty.Remove(aFace);

        List<string> aDifferences = HistoryComparison.Compare([aBox], aCopy, aHistory, [aBox], aCopy, anEmpty);
        Assert.NotEmpty(aDifferences);
        Assert.Contains(aDifferences, d => d.Contains("IsDeleted", StringComparison.Ordinal));
        Assert.Contains(aDifferences, d => d.Contains("Modified", StringComparison.Ordinal));
    }
}
