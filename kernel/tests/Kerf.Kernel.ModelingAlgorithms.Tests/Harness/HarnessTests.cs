using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Collections.Concurrent;
using Kerf.Kernel.TestHarness;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.Harness;

public class HarnessTests
{
    // ---- Shapes

    private static Shape Box() => new MakeBox(10.0, 20.0, 30.0).Shape();

    private static IEnumerable<Shape> Primitives()
    {
        yield return Box();
        yield return new ApiMakeCylinder(5.0, 12.0).Shape();
        yield return new MakeSphere(7.0).Shape();
        yield return new MakeTorus(10.0, 3.0).Shape();
        yield return new ApiMakeCone(6.0, 2.0, 9.0).Shape();
        yield return SharedCompound();
        yield return BoxShell(skipFace: -1);
    }

    // A compound of several solids, one of them twice at different locations (shared, located sub-shapes).
    private static Shape SharedCompound()
    {
        Shape aBox = Box();
        var aTrsf = new Transform3D();
        aTrsf = aTrsf.WithTranslation(new Vector3D(100.0, 0.0, 0.0));
        var aBuilder = new BrepBuilder();
        var aCompound = new Compound();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aBox);
        aBuilder.Add(aCompound, aBox.Located(new Placement(aTrsf)));
        aBuilder.Add(aCompound, new ApiMakeCylinder(5.0, 12.0).Shape());
        aBuilder.Add(aCompound, new MakeSphere(4.0).Shape());
        return aCompound;
    }

    // A shell of a box's faces, without the face of the given index (-1: all six).
    private static Shape BoxShell(int skipFace)
    {
        var aBuilder = new BrepBuilder();
        var aShell = new Shell();
        aBuilder.MakeShell(ref aShell);
        int i = 0;
        foreach (Shape aFace in CanonicalShapeComparer.SubShapes(Box(), ShapeType.Face))
        {
            if (i++ != skipFace)
            {
                aBuilder.Add(aShell, aFace);
            }
        }
        return aShell;
    }

    // The same shape with the children of every compound, compsolid, solid and shell in a random order.
    private static Shape Renumber(Shape theShape, Random theRandom)
    {
        ShapeType aType = theShape.ShapeType();
        if (aType is not (ShapeType.Compound or ShapeType.CompSolid or ShapeType.Solid or ShapeType.Shell))
        {
            return theShape;
        }
        var aChildren = new List<Shape>();
        for (var anIt = new Iterator(theShape, false, false); anIt.More(); anIt.Next())
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

        var aBuilder = new BrepBuilder();
        Shape aNew;
        switch (aType)
        {
            case ShapeType.Compound:
                var aCompound = new Compound();
                aBuilder.MakeCompound(ref aCompound);
                aNew = aCompound;
                break;
            case ShapeType.CompSolid:
                var aCompSolid = new CompSolid();
                aBuilder.MakeCompSolid(ref aCompSolid);
                aNew = aCompSolid;
                break;
            case ShapeType.Solid:
                var aSolid = new Solid();
                aBuilder.MakeSolid(ref aSolid);
                aNew = aSolid;
                break;
            default:
                var aShell = new Shell();
                aBuilder.MakeShell(ref aShell);
                aNew = aShell;
                break;
        }
        foreach (Shape aChild in aChildren)
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
        foreach (Shape aShape in Primitives())
        {
            string aText = BRepTextComparison.Write(aShape);
            for (int aSeed = 1; aSeed <= 4; ++aSeed)
            {
                Shape aRenumbered = Renumber(aShape, new Random(aSeed * 31 + aShapeIndex));
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
    private static int CountChildren(Shape theShape)
    {
        int aMax = 0;
        foreach (ShapeType aType in new[] { ShapeType.Compound, ShapeType.CompSolid, ShapeType.Solid, ShapeType.Shell })
        {
            foreach (Shape aContainer in CanonicalShapeComparer.SubShapes(theShape, aType))
            {
                int aNb = 0;
                for (var anIt = new Iterator(aContainer); anIt.More(); anIt.Next())
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
        foreach (Shape aShape in Primitives())
        {
            Shape aCopy = new Copy(aShape).Shape();
            CanonicalComparison aComparison = CanonicalShapeComparer.Compare(aShape, aCopy);
            Assert.True(aComparison.IsIdentical, aComparison.ToString());

            Shape anExact = BrepTools.ExactCopyForWrite(aShape, out _);
            CanonicalShapeComparer.AssertIdentical(aShape, anExact);
        }
    }

    [Fact]
    public void Canonical_VertexMoved_DetectedExactly_MatchedWithTolerance()
    {
        Shape aBox = Box();
        Shape aCopy = BrepTools.ExactCopyForWrite(aBox, out _);
        Vertex aVertex = ShapeCasts.Vertex(CanonicalShapeComparer.SubShapes(aCopy, ShapeType.Vertex)[0]);
        Point3D aP = Tool.Pnt(aVertex);
        new BrepBuilder().UpdateVertex(aVertex, new Point3D(aP.X() + 1.0e-12, aP.Y(), aP.Z()), Tool.Tolerance(aVertex));

        CanonicalComparison anExact = CanonicalShapeComparer.Compare(aBox, aCopy);
        Assert.False(anExact.IsIdentical);
        Assert.Contains(anExact.Unmatched, l => l.StartsWith("TopAbs_VERTEX", StringComparison.Ordinal));

        CanonicalComparison aTolerant = CanonicalShapeComparer.Compare(aBox, aCopy, new CanonicalOptions { VertexTolerance = 1.0e-9 });
        Assert.True(aTolerant.IsIdentical, aTolerant.ToString());
    }

    [Fact]
    public void Canonical_ToleranceChanged_ReportedAsAttributeDifference()
    {
        Shape aBox = Box();
        Shape aCopy = BrepTools.ExactCopyForWrite(aBox, out _);
        Vertex aVertex = ShapeCasts.Vertex(CanonicalShapeComparer.SubShapes(aCopy, ShapeType.Vertex)[3]);
        new BrepBuilder().UpdateVertex(aVertex, 1.0e-4);

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
        Assert.Equal((6, 5), aComparison.Counts[ShapeType.Face]);
        Assert.Contains(aComparison.Unmatched, l => l.StartsWith("TopAbs_FACE", StringComparison.Ordinal));
    }

    [Fact]
    public void Canonical_TranslatedShape_IsNotIdentical()
    {
        Shape aBox = Box();
        var aTrsf = new Transform3D();
        aTrsf = aTrsf.WithTranslation(new Vector3D(0.0, 0.0, 1.0e-6));
        CanonicalComparison aComparison = CanonicalShapeComparer.Compare(aBox, aBox.Located(new Placement(aTrsf)));
        Assert.False(aComparison.IsIdentical);
        Assert.True(aComparison.CountsEqual);
        Assert.NotEmpty(aComparison.Unmatched);
        Assert.NotEmpty(aComparison.PropertyDifferences);
    }

    [Fact]
    public void Canonical_Orientation_IgnoredByDefault_CheckedOnRequest()
    {
        Shape aBox = Box();
        Shape aReversed = aBox.Reversed();
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
            OsdParallel.For(0, aValues.Length, i => aValues[i] = Math.Sqrt(i) * Math.Sin(i));
            return string.Join("\n", aValues.Select(v => BitConverter.DoubleToInt64Bits(v)));
        });
        Assert.NotEmpty(aReference);
    }

    [Fact]
    public void Determinism_NondeterministicComputation_Fails()
    {
        // A result that differs on every run, so the harness must report it however the threads are scheduled (an
        // order-dependent parallel loop could come out in order on a busy machine whose pool runs it on one thread)
        int aRun = 0;
        Assert.ThrowsAny<Exception>(() => DeterminismHarness.AssertSerialEqualsParallel(() =>
        {
            int aThis = Interlocked.Increment(ref aRun);
            var aSum = new ConcurrentQueue<int>();
            OsdParallel.For(0, 100, i => aSum.Enqueue(i));
            return aThis + ":" + aSum.Count;
        }, 3, 2, 4));
    }

    [Fact]
    public void Determinism_ShapeAndHistory()
    {
        Shape aBox = Box();
        Shape aResult = DeterminismHarness.AssertSerialEqualsParallel(() => new Copy(aBox).Shape(), 2);
        Assert.False(aResult.IsNull);

        DeterminismHarness.AssertSerialEqualsParallel([aBox], () =>
        {
            Shape aCopy = BrepTools.ExactCopyForWrite(aBox, out History aHistory);
            return (aCopy, aHistory);
        }, 2);

        Shape aCopy = BrepTools.ExactCopyForWrite(aBox, out History aHistory);
        string aText = DeterminismHarness.HistoryText([aBox], aCopy, aHistory);
        Assert.Contains("TopAbs_FACE M[", aText, StringComparison.Ordinal);
        Assert.DoesNotContain("M[0", aText, StringComparison.Ordinal);
    }

    // ---- History comparison

    [Fact]
    public void History_SameUpToRenumbering_Agrees()
    {
        // Arguments and result renumbered at the top level only: the history knows the solids and below
        Shape anArgument = SharedCompound();
        Shape aCopy = BrepTools.ExactCopyForWrite(anArgument, out History aHistory);
        Shape aPortArgument = ShuffleTopLevel(anArgument, new Random(5));
        Shape aPortCopy = ShuffleTopLevel(aCopy, new Random(6));
        Assert.NotEqual(BRepTextComparison.Write(aCopy), BRepTextComparison.Write(aPortCopy));

        HistoryComparison.AssertSame([anArgument], aCopy, aHistory, [aPortArgument], aPortCopy, aHistory);

        // An independent copy: different TShapes on the port side, the same geometry
        Shape aCopy2 = BrepTools.ExactCopyForWrite(anArgument, out History aHistory2);
        HistoryComparison.AssertSame([anArgument], aCopy, aHistory, [anArgument], aCopy2, aHistory2);
    }

    // The compound with its direct children in reverse order (the children kept as they are).
    private static Shape ShuffleTopLevel(Shape theCompound, Random theRandom)
    {
        var aChildren = new List<Shape>();
        for (var anIt = new Iterator(theCompound, false, false); anIt.More(); anIt.Next())
        {
            aChildren.Add(anIt.Value());
        }
        var aBuilder = new BrepBuilder();
        var aNew = new Compound();
        aBuilder.MakeCompound(ref aNew);
        foreach (Shape aChild in aChildren.OrderBy(_ => theRandom.Next()).Reverse())
        {
            aBuilder.Add(aNew, aChild);
        }
        return aNew;
    }

    [Fact]
    public void History_Different_IsReported()
    {
        Shape aBox = Box();
        Shape aCopy = BrepTools.ExactCopyForWrite(aBox, out History aHistory);
        var anEmpty = new History();
        Shape aFace = CanonicalShapeComparer.SubShapes(aBox, ShapeType.Face)[0];
        anEmpty.Remove(aFace);

        List<string> aDifferences = HistoryComparison.Compare([aBox], aCopy, aHistory, [aBox], aCopy, anEmpty);
        Assert.NotEmpty(aDifferences);
        Assert.Contains(aDifferences, d => d.Contains("IsDeleted", StringComparison.Ordinal));
        Assert.Contains(aDifferences, d => d.Contains("Modified", StringComparison.Ordinal));
    }
}
