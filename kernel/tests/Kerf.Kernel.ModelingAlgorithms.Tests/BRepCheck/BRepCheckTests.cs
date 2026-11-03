// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Healing.Checks.StatusCheck;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>BRepCheck's statics, BRepCheck_Result's status bookkeeping and BRepCheck_Shell on BRep_Builder shells.</summary>
public class BRepCheckTests
{
    private static List<StatusCheck> ToList(NCollection_List<StatusCheck> theList) => [.. theList];

    private static Shell ShellOf(params Shape[] theFaces)
    {
        var aB = new BrepBuilder();
        Shell aShell = new();
        aB.MakeShell(ref aShell);
        foreach (Shape aF in theFaces)
        {
            aB.Add(aShell, aF);
        }

        return aShell;
    }

    private static Solid SolidOf(in Shell theShell)
    {
        var aB = new BrepBuilder();
        Solid aSolid = new();
        aB.MakeSolid(ref aSolid);
        aB.Add(aSolid, theShell);
        return aSolid;
    }

    private static List<Shape> Faces(in Shape theShape)
    {
        var aFaces = new List<Shape>();
        for (var anExp = new Explorer(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aFaces.Add(anExp.Current());
        }

        return aFaces;
    }

    private static Shell BoxShell(double dx = 1, double dy = 1, double dz = 1)
        => ShapeCasts.Shell(new Iterator(TestShapes.Box(dx, dy, dz)).Value());

    /// <summary>
    /// Two unit boxes, [0, 1]^3 and [1, 2] x [1, 2] x [0, 1], sharing the vertical edge x = y = 1: twelve faces in
    /// one shell, four of them on the shared edge.
    /// </summary>
    private static Shell TwoBoxesSharingAnEdge()
    {
        Point3D[] aPoints =
        [
            new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(1, 1, 0), new(0, 0, 1), new(1, 0, 1), new(0, 1, 1), new(1, 1, 1),
            new(2, 1, 0), new(1, 2, 0), new(2, 2, 0), new(2, 1, 1), new(1, 2, 1), new(2, 2, 1),
        ];
        Vertex[] aV = [.. aPoints.Select(p => TestShapes.Vertex(p))];
        // box B's corners in TestShapes.Box's numbering (bit 0: x, bit 1: y, bit 2: z)
        int[] aB = [3, 8, 9, 10, 7, 11, 12, 13];
        var anEdges = new Dictionary<(int, int), Edge>();
        (int[] Indices, Direction3D Normal)[] aFaces =
        [
            ([0, 2, 3, 1], new Direction3D(0, 0, -1)),
            ([4, 5, 7, 6], new Direction3D(0, 0, 1)),
            ([0, 1, 5, 4], new Direction3D(0, -1, 0)),
            ([2, 6, 7, 3], new Direction3D(0, 1, 0)),
            ([0, 4, 6, 2], new Direction3D(-1, 0, 0)),
            ([1, 3, 7, 5], new Direction3D(1, 0, 0)),
        ];
        var aShellFaces = new List<Shape>();
        foreach ((int[] anIndices, Direction3D aNormal) in aFaces)
        {
            aShellFaces.Add(TestShapes.PolygonFace(aV, aNormal, anEdges, anIndices));
        }

        foreach ((int[] anIndices, Direction3D aNormal) in aFaces)
        {
            aShellFaces.Add(TestShapes.PolygonFace(aV, aNormal, anEdges, [.. anIndices.Select(i => aB[i])]));
        }

        return ShellOf([.. aShellFaces]);
    }

    [Fact]
    public void Add_ReplacesNoErrorAndSkipsDuplicates()
    {
        var aList = new NCollection_List<StatusCheck>();
        BRepCheckCheck.Add(aList, StatusCheck.NoError);
        Assert.Equal([StatusCheck.NoError], ToList(aList));
        BRepCheckCheck.Add(aList, StatusCheck.NotClosed);
        Assert.Equal([StatusCheck.NotClosed], ToList(aList));
        BRepCheckCheck.Add(aList, StatusCheck.NotClosed);
        BRepCheckCheck.Add(aList, StatusCheck.FreeEdge);
        Assert.Equal([StatusCheck.NotClosed, StatusCheck.FreeEdge], ToList(aList));
        // NoError is appended after real errors, as in OCCT
        BRepCheckCheck.Add(aList, StatusCheck.NoError);
        Assert.Equal([StatusCheck.NotClosed, StatusCheck.FreeEdge, StatusCheck.NoError], ToList(aList));
    }

    [Fact]
    public void Print_WritesTheNameOfEachStatus()
    {
        foreach (StatusCheck aStatus in Enum.GetValues<StatusCheck>())
        {
            var aWriter = new StringWriter();
            BRepCheckCheck.Print(aStatus, aWriter);
            Assert.Equal(Kerf.Engine.Foundation.OcctTypeNames.Of(aStatus) + "\n", aWriter.ToString());
        }

        var anUnknown = new StringWriter();
        BRepCheckCheck.Print((StatusCheck)1000, anUnknown);
        Assert.Equal("", anUnknown.ToString());
    }

    [Fact]
    public void PrecCurve_EllipseUsesItsLargestNumber()
    {
        Assert.Equal(RealEpsilon(), BRepCheckCheck.PrecCurve(new CurveAdaptor(new LineCurve3D(new Point3D(500, 0, 0), new Direction3D(1, 0, 0)))));
        var anEllipse = new EllipseCurve3D(new Frame3D(new Point3D(-300, 20, 1), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 10, 5);
        Assert.Equal(Epsilon(300.0), BRepCheckCheck.PrecCurve(new CurveAdaptor(anEllipse)));
        var aBig = new EllipseCurve3D(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 4000, 5);
        Assert.Equal(Epsilon(4000.0), BRepCheckCheck.PrecCurve(new CurveAdaptor(aBig)));
    }

    [Fact]
    public void PrecSurface_ConeUsesItsLargestNumber()
    {
        Assert.Equal(RealEpsilon(), BRepCheckCheck.PrecSurface(new SurfaceAdaptor(new PlaneSurface(new Point3D(900, 0, 0), new Direction3D(0, 0, 1)))));
        var aCone = new ConicalSurface(new CoordinateSystem(new Point3D(0, 0, -200), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 0.3, 3);
        Assert.Equal(Epsilon(200.0), BRepCheckCheck.PrecSurface(new SurfaceAdaptor(aCone)));
    }

    [Fact]
    public void Shell_ClosedBox()
    {
        Solid aBox = TestShapes.Box(1, 2, 3);
        Shell aShell = ShapeCasts.Shell(new Iterator(aBox).Value());
        var aCheck = new ShellCheck(aShell);
        Assert.True(aCheck.IsMinimum);
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.Status()));
        Assert.Equal(StatusCheck.NoError, aCheck.Closed());
        Assert.Equal(StatusCheck.NoError, aCheck.Orientation());
        Assert.False(aCheck.IsUnorientable());

        aCheck.InContext(aBox);
        Assert.True(aCheck.IsStatusOnShape(aBox));
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.StatusOnShape(aBox)));
        aCheck.InitContextIterator();
        Assert.True(aCheck.MoreShapeInContext());
        Assert.True(aCheck.ContextualShape().IsSame(aBox));
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.StatusOnShape()));
        aCheck.NextShapeInContext();
        Assert.False(aCheck.MoreShapeInContext());

        var aSets = new NCollection_List<Shape>();
        Assert.Equal(0, aCheck.NbConnectedSet(aSets));
    }

    [Fact]
    public void Shell_FrozenInputIsReadOnly()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Shell aShell = ShapeCasts.Shell(new Iterator(aBox).Value());
        var aCheck = new ShellCheck(aShell);
        Assert.Equal(StatusCheck.NoError, aCheck.Closed(true));
        Assert.Equal(StatusCheck.NoError, aCheck.Orientation(true));
        aCheck.InContext(aBox);
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.StatusOnShape(aBox)));
    }

    [Fact]
    public void Shell_OpenBoxIsNotClosed()
    {
        List<Shape> aFaces = Faces(BoxShell());
        Shell anOpen = ShellOf([.. aFaces.Take(5)]);
        var aCheck = new ShellCheck(anOpen);
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.Status()));
        Assert.Equal(StatusCheck.NotClosed, aCheck.Closed());
        // Orientation accepts an open shell
        Assert.Equal(StatusCheck.NoError, aCheck.Orientation());
        // the status list is changed only on request
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.Status()));
        Assert.Equal(StatusCheck.NotClosed, aCheck.Closed(true));
        Assert.Equal([StatusCheck.NotClosed], ToList(aCheck.Status()));

        // in a solid, the shell must be closed
        var aCheck2 = new ShellCheck(anOpen);
        Solid aSolid = SolidOf(anOpen);
        aCheck2.InContext(aSolid);
        Assert.Equal([StatusCheck.NotClosed], ToList(aCheck2.StatusOnShape(aSolid)));
    }

    [Fact]
    public void Shell_ReversedFaceIsBadlyOriented()
    {
        List<Shape> aFaces = Faces(BoxShell());
        aFaces[2] = aFaces[2].Reversed();
        Shell aShell = ShellOf([.. aFaces]);
        var aCheck = new ShellCheck(aShell);
        Assert.Equal(StatusCheck.NoError, aCheck.Closed());
        Assert.Equal(StatusCheck.BadOrientationOfSubshape, aCheck.Orientation());
        Assert.True(aCheck.IsUnorientable());

        var aCheck2 = new ShellCheck(aShell);
        Solid aSolid = SolidOf(aShell);
        aCheck2.InContext(aSolid);
        Assert.Equal([StatusCheck.BadOrientationOfSubshape], ToList(aCheck2.StatusOnShape(aSolid)));

        // marked unorientable, the shell isn't checked for orientation in a solid
        var aCheck3 = new ShellCheck(aShell);
        aCheck3.SetUnorientable();
        Assert.True(aCheck3.IsUnorientable());
        Assert.Equal([StatusCheck.UnorientableShape], ToList(aCheck3.Status()));
    }

    [Fact]
    public void Shell_EmptyAndDisconnected()
    {
        var anEmpty = new ShellCheck(ShellOf());
        Assert.Equal([StatusCheck.EmptyShell], ToList(anEmpty.Status()));
        Assert.Equal(StatusCheck.EmptyShell, anEmpty.Closed());
        Assert.Equal(StatusCheck.EmptyShell, anEmpty.Orientation());

        var aFaces = new List<Shape>(Faces(BoxShell()));
        aFaces.AddRange(Faces(BoxShell(2, 2, 2)));
        var aTwo = new ShellCheck(ShellOf([.. aFaces]));
        Assert.Equal([StatusCheck.NotConnected], ToList(aTwo.Status()));
        Assert.Equal(StatusCheck.NotConnected, aTwo.Closed());
    }

    [Fact]
    public void Shell_RedundantFace()
    {
        List<Shape> aFaces = Faces(BoxShell());
        aFaces.Add(aFaces[0]);
        Shell aShell = ShellOf([.. aFaces]);
        var aCheck = new ShellCheck(aShell);
        // Minimum counts 7 faces but reaches 6 distinct ones, so the shell is "not connected", and Closed and
        // Orientation report that first status (Closed's own RedundantFace check can't fire anyway, occt-issues.md)
        Assert.Equal([StatusCheck.NotConnected], ToList(aCheck.Status()));
        Assert.Equal(StatusCheck.NotConnected, aCheck.Closed());
        Assert.Equal(StatusCheck.NotConnected, aCheck.Orientation());

        // with a blind status list (Init's Minimum reset to NoError), Closed still finds no redundant face
        var aBlind = new ShellCheck(aShell);
        aBlind.Status().Clear();
        aBlind.Status().Append(StatusCheck.NoError);
        // OCCT: IndexedMap::Add returns an index, never 0, so Closed's RedundantFace check never fires; the face's
        // edges are then found three times, and NbConnectedSet splits the shell at them
        Assert.Equal(StatusCheck.InvalidMultiConnexity, aBlind.Closed());
        Assert.Equal(StatusCheck.InvalidMultiConnexity, aBlind.Orientation());
    }

    [Fact]
    public void Shell_TwoBoxesSharingAnEdge()
    {
        Shell aShell = TwoBoxesSharingAnEdge();
        var aCheck = new ShellCheck(aShell);
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.Status()));
        Assert.Equal(StatusCheck.InvalidMultiConnexity, aCheck.Closed());

        var aSets = new NCollection_List<Shape>();
        Assert.Equal(2, aCheck.NbConnectedSet(aSets));
        foreach (Shape aSet in aSets)
        {
            Assert.Equal(ShapeType.Shell, aSet.ShapeType());
            Assert.Equal(6, Faces(aSet).Count);
            Assert.True(aSet.TShape()!.IsFrozen);
            Assert.True(aSet.Closed());
            Assert.Equal(StatusCheck.NoError, new ShellCheck(ShapeCasts.Shell(aSet)).Closed());
        }
    }

    [Fact]
    public void Shell_NotInContext()
    {
        var aCheck = new ShellCheck(BoxShell());
        Solid anOther = TestShapes.Box(1, 1, 1);
        aCheck.InContext(anOther);
        Assert.Equal([StatusCheck.SubshapeNotInShape], ToList(aCheck.StatusOnShape(anOther)));
        // a second call does nothing
        aCheck.InContext(anOther);
        Assert.Equal([StatusCheck.SubshapeNotInShape], ToList(aCheck.StatusOnShape(anOther)));
    }

    [Fact]
    public void Result_SetFailStatusAndContexts()
    {
        Shell aShell = BoxShell();
        var aCheck = new ShellCheck(aShell);
        aCheck.SetParallel(true);
        Assert.True(aCheck.IsParallel);
        aCheck.SetFailStatus(aShell);
        Assert.Equal([StatusCheck.CheckFail], ToList(aCheck.Status()));
        Face aFace = ShapeCasts.Face(Faces(aShell)[0]);
        Assert.False(aCheck.IsStatusOnShape(aFace));
        aCheck.SetFailStatus(aFace);
        Assert.Equal([StatusCheck.CheckFail], ToList(aCheck.StatusOnShape(aFace)));
        // the contexts skip the shell itself
        aCheck.InitContextIterator();
        Assert.True(aCheck.ContextualShape().IsSame(aFace));
        aCheck.NextShapeInContext();
        Assert.False(aCheck.MoreShapeInContext());
        // Closed reports the first status already saved
        Assert.Equal(StatusCheck.CheckFail, aCheck.Closed());
        Assert.False(aCheck.IsBlind);
        aCheck.Blind();
        Assert.True(aCheck.IsBlind);
        // Init starts again
        aCheck.Init(aShell);
        Assert.Equal([StatusCheck.NoError], ToList(aCheck.Status()));
        Assert.False(aCheck.IsStatusOnShape(aFace));
    }
}
