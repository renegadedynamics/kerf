// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using OcctNet.ModelingData.Tests;
using static Occt.BRepCheck_Status;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>BRepCheck's statics, BRepCheck_Result's status bookkeeping and BRepCheck_Shell on BRep_Builder shells.</summary>
public class BRepCheckTests
{
    private static List<BRepCheck_Status> ToList(NCollection_List<BRepCheck_Status> theList) => [.. theList];

    private static TopoDS_Shell ShellOf(params TopoDS_Shape[] theFaces)
    {
        var aB = new BRep_Builder();
        TopoDS_Shell aShell = new();
        aB.MakeShell(ref aShell);
        foreach (TopoDS_Shape aF in theFaces)
        {
            aB.Add(aShell, aF);
        }

        return aShell;
    }

    private static TopoDS_Solid SolidOf(in TopoDS_Shell theShell)
    {
        var aB = new BRep_Builder();
        TopoDS_Solid aSolid = new();
        aB.MakeSolid(ref aSolid);
        aB.Add(aSolid, theShell);
        return aSolid;
    }

    private static List<TopoDS_Shape> Faces(in TopoDS_Shape theShape)
    {
        var aFaces = new List<TopoDS_Shape>();
        for (var anExp = new TopExp_Explorer(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aFaces.Add(anExp.Current());
        }

        return aFaces;
    }

    private static TopoDS_Shell BoxShell(double dx = 1, double dy = 1, double dz = 1)
        => TopoDS.Shell(new TopoDS_Iterator(TestShapes.Box(dx, dy, dz)).Value());

    /// <summary>
    /// Two unit boxes, [0, 1]^3 and [1, 2] x [1, 2] x [0, 1], sharing the vertical edge x = y = 1: twelve faces in
    /// one shell, four of them on the shared edge.
    /// </summary>
    private static TopoDS_Shell TwoBoxesSharingAnEdge()
    {
        Point3D[] aPoints =
        [
            new(0, 0, 0), new(1, 0, 0), new(0, 1, 0), new(1, 1, 0), new(0, 0, 1), new(1, 0, 1), new(0, 1, 1), new(1, 1, 1),
            new(2, 1, 0), new(1, 2, 0), new(2, 2, 0), new(2, 1, 1), new(1, 2, 1), new(2, 2, 1),
        ];
        TopoDS_Vertex[] aV = [.. aPoints.Select(p => TestShapes.Vertex(p))];
        // box B's corners in TestShapes.Box's numbering (bit 0: x, bit 1: y, bit 2: z)
        int[] aB = [3, 8, 9, 10, 7, 11, 12, 13];
        var anEdges = new Dictionary<(int, int), TopoDS_Edge>();
        (int[] Indices, Direction3D Normal)[] aFaces =
        [
            ([0, 2, 3, 1], new Direction3D(0, 0, -1)),
            ([4, 5, 7, 6], new Direction3D(0, 0, 1)),
            ([0, 1, 5, 4], new Direction3D(0, -1, 0)),
            ([2, 6, 7, 3], new Direction3D(0, 1, 0)),
            ([0, 4, 6, 2], new Direction3D(-1, 0, 0)),
            ([1, 3, 7, 5], new Direction3D(1, 0, 0)),
        ];
        var aShellFaces = new List<TopoDS_Shape>();
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
        var aList = new NCollection_List<BRepCheck_Status>();
        BRepCheck.Add(aList, BRepCheck_NoError);
        Assert.Equal([BRepCheck_NoError], ToList(aList));
        BRepCheck.Add(aList, BRepCheck_NotClosed);
        Assert.Equal([BRepCheck_NotClosed], ToList(aList));
        BRepCheck.Add(aList, BRepCheck_NotClosed);
        BRepCheck.Add(aList, BRepCheck_FreeEdge);
        Assert.Equal([BRepCheck_NotClosed, BRepCheck_FreeEdge], ToList(aList));
        // NoError is appended after real errors, as in OCCT
        BRepCheck.Add(aList, BRepCheck_NoError);
        Assert.Equal([BRepCheck_NotClosed, BRepCheck_FreeEdge, BRepCheck_NoError], ToList(aList));
    }

    [Fact]
    public void Print_WritesTheNameOfEachStatus()
    {
        foreach (BRepCheck_Status aStatus in Enum.GetValues<BRepCheck_Status>())
        {
            var aWriter = new StringWriter();
            BRepCheck.Print(aStatus, aWriter);
            Assert.Equal(aStatus + "\n", aWriter.ToString());
        }

        var anUnknown = new StringWriter();
        BRepCheck.Print((BRepCheck_Status)1000, anUnknown);
        Assert.Equal("", anUnknown.ToString());
    }

    [Fact]
    public void PrecCurve_EllipseUsesItsLargestNumber()
    {
        Assert.Equal(RealEpsilon(), BRepCheck.PrecCurve(new GeomAdaptor_Curve(new Geom_Line(new Point3D(500, 0, 0), new Direction3D(1, 0, 0)))));
        var anEllipse = new Geom_Ellipse(new Frame3D(new Point3D(-300, 20, 1), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 10, 5);
        Assert.Equal(Epsilon(300.0), BRepCheck.PrecCurve(new GeomAdaptor_Curve(anEllipse)));
        var aBig = new Geom_Ellipse(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 4000, 5);
        Assert.Equal(Epsilon(4000.0), BRepCheck.PrecCurve(new GeomAdaptor_Curve(aBig)));
    }

    [Fact]
    public void PrecSurface_ConeUsesItsLargestNumber()
    {
        Assert.Equal(RealEpsilon(), BRepCheck.PrecSurface(new GeomAdaptor_Surface(new Geom_Plane(new Point3D(900, 0, 0), new Direction3D(0, 0, 1)))));
        var aCone = new Geom_ConicalSurface(new CoordinateSystem(new Point3D(0, 0, -200), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 0.3, 3);
        Assert.Equal(Epsilon(200.0), BRepCheck.PrecSurface(new GeomAdaptor_Surface(aCone)));
    }

    [Fact]
    public void Shell_ClosedBox()
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 2, 3);
        TopoDS_Shell aShell = TopoDS.Shell(new TopoDS_Iterator(aBox).Value());
        var aCheck = new BRepCheck_Shell(aShell);
        Assert.True(aCheck.IsMinimum());
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.Status()));
        Assert.Equal(BRepCheck_NoError, aCheck.Closed());
        Assert.Equal(BRepCheck_NoError, aCheck.Orientation());
        Assert.False(aCheck.IsUnorientable());

        aCheck.InContext(aBox);
        Assert.True(aCheck.IsStatusOnShape(aBox));
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.StatusOnShape(aBox)));
        aCheck.InitContextIterator();
        Assert.True(aCheck.MoreShapeInContext());
        Assert.True(aCheck.ContextualShape().IsSame(aBox));
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.StatusOnShape()));
        aCheck.NextShapeInContext();
        Assert.False(aCheck.MoreShapeInContext());

        var aSets = new NCollection_List<TopoDS_Shape>();
        Assert.Equal(0, aCheck.NbConnectedSet(aSets));
    }

    [Fact]
    public void Shell_FrozenInputIsReadOnly()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Shell aShell = TopoDS.Shell(new TopoDS_Iterator(aBox).Value());
        var aCheck = new BRepCheck_Shell(aShell);
        Assert.Equal(BRepCheck_NoError, aCheck.Closed(true));
        Assert.Equal(BRepCheck_NoError, aCheck.Orientation(true));
        aCheck.InContext(aBox);
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.StatusOnShape(aBox)));
    }

    [Fact]
    public void Shell_OpenBoxIsNotClosed()
    {
        List<TopoDS_Shape> aFaces = Faces(BoxShell());
        TopoDS_Shell anOpen = ShellOf([.. aFaces.Take(5)]);
        var aCheck = new BRepCheck_Shell(anOpen);
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.Status()));
        Assert.Equal(BRepCheck_NotClosed, aCheck.Closed());
        // Orientation accepts an open shell
        Assert.Equal(BRepCheck_NoError, aCheck.Orientation());
        // the status list is changed only on request
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.Status()));
        Assert.Equal(BRepCheck_NotClosed, aCheck.Closed(true));
        Assert.Equal([BRepCheck_NotClosed], ToList(aCheck.Status()));

        // in a solid, the shell must be closed
        var aCheck2 = new BRepCheck_Shell(anOpen);
        TopoDS_Solid aSolid = SolidOf(anOpen);
        aCheck2.InContext(aSolid);
        Assert.Equal([BRepCheck_NotClosed], ToList(aCheck2.StatusOnShape(aSolid)));
    }

    [Fact]
    public void Shell_ReversedFaceIsBadlyOriented()
    {
        List<TopoDS_Shape> aFaces = Faces(BoxShell());
        aFaces[2] = aFaces[2].Reversed();
        TopoDS_Shell aShell = ShellOf([.. aFaces]);
        var aCheck = new BRepCheck_Shell(aShell);
        Assert.Equal(BRepCheck_NoError, aCheck.Closed());
        Assert.Equal(BRepCheck_BadOrientationOfSubshape, aCheck.Orientation());
        Assert.True(aCheck.IsUnorientable());

        var aCheck2 = new BRepCheck_Shell(aShell);
        TopoDS_Solid aSolid = SolidOf(aShell);
        aCheck2.InContext(aSolid);
        Assert.Equal([BRepCheck_BadOrientationOfSubshape], ToList(aCheck2.StatusOnShape(aSolid)));

        // marked unorientable, the shell isn't checked for orientation in a solid
        var aCheck3 = new BRepCheck_Shell(aShell);
        aCheck3.SetUnorientable();
        Assert.True(aCheck3.IsUnorientable());
        Assert.Equal([BRepCheck_UnorientableShape], ToList(aCheck3.Status()));
    }

    [Fact]
    public void Shell_EmptyAndDisconnected()
    {
        var anEmpty = new BRepCheck_Shell(ShellOf());
        Assert.Equal([BRepCheck_EmptyShell], ToList(anEmpty.Status()));
        Assert.Equal(BRepCheck_EmptyShell, anEmpty.Closed());
        Assert.Equal(BRepCheck_EmptyShell, anEmpty.Orientation());

        var aFaces = new List<TopoDS_Shape>(Faces(BoxShell()));
        aFaces.AddRange(Faces(BoxShell(2, 2, 2)));
        var aTwo = new BRepCheck_Shell(ShellOf([.. aFaces]));
        Assert.Equal([BRepCheck_NotConnected], ToList(aTwo.Status()));
        Assert.Equal(BRepCheck_NotConnected, aTwo.Closed());
    }

    [Fact]
    public void Shell_RedundantFace()
    {
        List<TopoDS_Shape> aFaces = Faces(BoxShell());
        aFaces.Add(aFaces[0]);
        TopoDS_Shell aShell = ShellOf([.. aFaces]);
        var aCheck = new BRepCheck_Shell(aShell);
        // Minimum counts 7 faces but reaches 6 distinct ones, so the shell is "not connected", and Closed and
        // Orientation report that first status (Closed's own RedundantFace check can't fire anyway, occt-issues.md)
        Assert.Equal([BRepCheck_NotConnected], ToList(aCheck.Status()));
        Assert.Equal(BRepCheck_NotConnected, aCheck.Closed());
        Assert.Equal(BRepCheck_NotConnected, aCheck.Orientation());

        // with a blind status list (Init's Minimum reset to NoError), Closed still finds no redundant face
        var aBlind = new BRepCheck_Shell(aShell);
        aBlind.Status().Clear();
        aBlind.Status().Append(BRepCheck_NoError);
        // OCCT: IndexedMap::Add returns an index, never 0, so Closed's RedundantFace check never fires; the face's
        // edges are then found three times, and NbConnectedSet splits the shell at them
        Assert.Equal(BRepCheck_InvalidMultiConnexity, aBlind.Closed());
        Assert.Equal(BRepCheck_InvalidMultiConnexity, aBlind.Orientation());
    }

    [Fact]
    public void Shell_TwoBoxesSharingAnEdge()
    {
        TopoDS_Shell aShell = TwoBoxesSharingAnEdge();
        var aCheck = new BRepCheck_Shell(aShell);
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.Status()));
        Assert.Equal(BRepCheck_InvalidMultiConnexity, aCheck.Closed());

        var aSets = new NCollection_List<TopoDS_Shape>();
        Assert.Equal(2, aCheck.NbConnectedSet(aSets));
        foreach (TopoDS_Shape aSet in aSets)
        {
            Assert.Equal(TopAbs_SHELL, aSet.ShapeType());
            Assert.Equal(6, Faces(aSet).Count);
            Assert.True(aSet.TShape()!.IsFrozen);
            Assert.True(aSet.Closed());
            Assert.Equal(BRepCheck_NoError, new BRepCheck_Shell(TopoDS.Shell(aSet)).Closed());
        }
    }

    [Fact]
    public void Shell_NotInContext()
    {
        var aCheck = new BRepCheck_Shell(BoxShell());
        TopoDS_Solid anOther = TestShapes.Box(1, 1, 1);
        aCheck.InContext(anOther);
        Assert.Equal([BRepCheck_SubshapeNotInShape], ToList(aCheck.StatusOnShape(anOther)));
        // a second call does nothing
        aCheck.InContext(anOther);
        Assert.Equal([BRepCheck_SubshapeNotInShape], ToList(aCheck.StatusOnShape(anOther)));
    }

    [Fact]
    public void Result_SetFailStatusAndContexts()
    {
        TopoDS_Shell aShell = BoxShell();
        var aCheck = new BRepCheck_Shell(aShell);
        aCheck.SetParallel(true);
        Assert.True(aCheck.IsParallel());
        aCheck.SetFailStatus(aShell);
        Assert.Equal([BRepCheck_CheckFail], ToList(aCheck.Status()));
        TopoDS_Face aFace = TopoDS.Face(Faces(aShell)[0]);
        Assert.False(aCheck.IsStatusOnShape(aFace));
        aCheck.SetFailStatus(aFace);
        Assert.Equal([BRepCheck_CheckFail], ToList(aCheck.StatusOnShape(aFace)));
        // the contexts skip the shell itself
        aCheck.InitContextIterator();
        Assert.True(aCheck.ContextualShape().IsSame(aFace));
        aCheck.NextShapeInContext();
        Assert.False(aCheck.MoreShapeInContext());
        // Closed reports the first status already saved
        Assert.Equal(BRepCheck_CheckFail, aCheck.Closed());
        Assert.False(aCheck.IsBlind());
        aCheck.Blind();
        Assert.True(aCheck.IsBlind());
        // Init starts again
        aCheck.Init(aShell);
        Assert.Equal([BRepCheck_NoError], ToList(aCheck.Status()));
        Assert.False(aCheck.IsStatusOnShape(aFace));
    }
}
