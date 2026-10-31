// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Shapes for the tests of ShapeFix_Shell, Solid, Shape and Wireframe.</summary>
internal static class ShapeFixShapes
{
    public static List<TopoDS_Shape> SubShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
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

    public static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType) => SubShapes(theShape, theType).Count;

    public static bool HasTShapeOf(in TopoDS_Shape theShape, in TopoDS_Shape theSub, TopAbs_ShapeEnum theType)
    {
        foreach (TopoDS_Shape aS in SubShapes(theShape, theType))
        {
            if (ReferenceEquals(aS.TShape(), theSub.TShape()))
            {
                return true;
            }
        }
        return false;
    }

    public static double Volume(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    /// <summary>The faces of the box [0, 10]^3 (unfrozen, sharing their edges).</summary>
    public static List<TopoDS_Shape> BoxFaces(double theSize = 10.0)
    {
        var aList = new List<TopoDS_Shape>();
        for (var anExp = new TopExp_Explorer(TestShapes.Box(theSize, theSize, theSize), TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aList.Add(anExp.Current());
        }
        return aList;
    }

    /// <summary>A shell of the faces, the face <paramref name="theReversed"/> reversed (-1: none). Unfrozen.</summary>
    public static TopoDS_Shell Shell(IEnumerable<TopoDS_Shape> theFaces, int theReversed = -1, bool theClosed = false)
    {
        var aB = new BRep_Builder();
        var aShell = new TopoDS_Shell();
        aB.MakeShell(ref aShell);
        int i = 0;
        foreach (TopoDS_Shape aF in theFaces)
        {
            aB.Add(aShell, i++ == theReversed ? aF.Reversed() : aF);
        }
        aShell.Closed(theClosed);
        return aShell;
    }

    public static TopoDS_Solid Solid(params TopoDS_Shape[] theShells)
    {
        var aB = new BRep_Builder();
        var aSolid = new TopoDS_Solid();
        aB.MakeSolid(ref aSolid);
        foreach (TopoDS_Shape aSh in theShells)
        {
            aB.Add(aSolid, aSh);
        }
        return aSolid;
    }

    /// <summary>The five-vertex triangulated Moebius band: its faces can't be oriented consistently. Unfrozen.</summary>
    public static TopoDS_Shell Moebius()
    {
        var aV = new TopoDS_Vertex[5];
        for (int i = 0; i < 5; i++)
        {
            double a = 2.0 * Math.PI * i / 5.0;
            aV[i] = TestShapes.Vertex(new gp_Pnt(10.0 * Math.Cos(a), 10.0 * Math.Sin(a), (i % 2) * 3.0));
        }
        var anEdges = new Dictionary<(int, int), TopoDS_Edge>();
        int[][] aTriangles = [[0, 1, 2], [1, 2, 3], [2, 3, 4], [3, 4, 0], [4, 0, 1]];
        var aFaces = new List<TopoDS_Shape>();
        foreach (int[] t in aTriangles)
        {
            gp_Pnt p0 = BRep_Tool.Pnt(aV[t[0]]), p1 = BRep_Tool.Pnt(aV[t[1]]), p2 = BRep_Tool.Pnt(aV[t[2]]);
            gp_Vec aN = new gp_Vec(p0, p1).Crossed(new gp_Vec(p0, p2));
            aFaces.Add(TestShapes.PolygonFace(aV, new gp_Dir(aN), anEdges, t));
        }
        return Shell(aFaces);
    }

    /// <summary>
    /// Three faces sharing one edge (two box faces and a fin), with the rest of the box: a shell with a multishared
    /// edge. Unfrozen.
    /// </summary>
    public static TopoDS_Shell ShellWithFin()
    {
        var aV = new TopoDS_Vertex[10];
        for (int i = 0; i < 8; i++)
        {
            aV[i] = TestShapes.Vertex(new gp_Pnt((i & 1) != 0 ? 10 : 0, (i & 2) != 0 ? 10 : 0, (i & 4) != 0 ? 10 : 0));
        }
        aV[8] = TestShapes.Vertex(new gp_Pnt(20, 0, 0));
        aV[9] = TestShapes.Vertex(new gp_Pnt(20, 0, 10));
        var anEdges = new Dictionary<(int, int), TopoDS_Edge>();
        (int[] Indices, gp_Dir Normal)[] aFaces =
        [
            ([0, 2, 3, 1], new gp_Dir(0, 0, -1)),
            ([4, 5, 7, 6], new gp_Dir(0, 0, 1)),
            ([0, 1, 5, 4], new gp_Dir(0, -1, 0)),
            ([2, 6, 7, 3], new gp_Dir(0, 1, 0)),
            ([0, 4, 6, 2], new gp_Dir(-1, 0, 0)),
            ([1, 3, 7, 5], new gp_Dir(1, 0, 0)),
            ([1, 8, 9, 5], new gp_Dir(0, -1, 0)), // the fin, on the edge 1-5
        ];
        var aList = new List<TopoDS_Shape>();
        foreach ((int[] anIndices, gp_Dir aNormal) in aFaces)
        {
            aList.Add(TestShapes.PolygonFace(aV, aNormal, anEdges, anIndices));
        }
        return Shell(aList);
    }

    public static bool IsOriented(in TopoDS_Shape theShape)
    {
        var aSas = new ShapeAnalysis_Shell();
        return !aSas.CheckOrientedShells(theShape, false);
    }
}

/// <summary>
/// ShapeFix_Shell: the orientation of faces fixed, the multishared edges, the Moebius faces, the closed flag, on frozen
/// shells (the caller's unchanged, the results frozen, the caller's faces kept) and unfrozen ones (in place). The face
/// fixes (ShapeFix_Face) are switched off: they are tested with ShapeFix_Face.
/// </summary>
public class ShapeFix_ShellTests
{
    private static ShapeFix_Shell Fixer()
    {
        var aFix = new ShapeFix_Shell();
        aFix.FixFaceMode() = 0;
        return aFix;
    }

    [Fact]
    public void ReversedFace_IsReoriented_TheCallersFacesKept()
    {
        List<TopoDS_Shape> aFaces = ShapeFixShapes.BoxFaces();
        TopoDS_Shell aShell = TopoDS.Shell(TestShapes.Frozen((TopoDS_Shape)ShapeFixShapes.Shell(aFaces, 2, true)));
        Assert.False(ShapeFixShapes.IsOriented(aShell));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);

        ShapeFix_Shell aFix = Fixer();
        aFix.Init(aShell);
        Assert.True(aFix.Perform());
        Assert.True(aFix.Status(ShapeExtend_DONE2));
        Assert.Equal(1, aFix.NbShells());

        TopoDS_Shape aResult = aFix.Shape();
        aSnap.AssertUnchanged(aShell);
        Assert.True(aResult.TShape()!.IsFrozen());
        Assert.Equal(TopAbs_SHELL, aResult.ShapeType());
        Assert.True(ShapeFixShapes.IsOriented(aResult));
        Assert.True(BRep_Tool.IsClosed(aResult));
        Assert.True(ReferenceEquals(aFix.Shell().TShape(), aResult.TShape()));

        // every face is the caller's (only orientations changed)
        foreach (TopoDS_Shape aF in ShapeFixShapes.SubShapes(aShell, TopAbs_FACE))
        {
            Assert.True(ShapeFixShapes.HasTShapeOf(aResult, aF, TopAbs_FACE));
        }

        // the context answers for the caller's shell
        TopoDS_Shape aValue = aFix.Context()!.Value(aShell);
        Assert.True(aValue.IsSame(aResult));
        Assert.True(aFix.Context()!.Apply(aShell).IsSame(aResult));
    }

    [Fact]
    public void ValidShell_NothingDone_TheCallersFaces()
    {
        TopoDS_Shell aShell = TopoDS.Shell(TestShapes.Frozen((TopoDS_Shape)ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(), -1, true)));
        ShapeFix_Shell aFix = Fixer();
        aFix.Init(aShell);
        Assert.False(aFix.Perform());
        // as OCCT's, the orientation fix rebuilds the shell even when nothing is done (and records nothing): the
        // shell is new, its faces are the caller's
        TopoDS_Shape aResult = aFix.Shape();
        Assert.False(ReferenceEquals(aResult.TShape(), aShell.TShape()));
        Assert.Equal(6, ShapeFixShapes.Count(aResult, TopAbs_FACE));
        foreach (TopoDS_Shape aF in ShapeFixShapes.SubShapes(aShell, TopAbs_FACE))
        {
            Assert.True(ShapeFixShapes.HasTShapeOf(aResult, aF, TopAbs_FACE));
        }
        Assert.False(aFix.Context()!.IsRecorded(aShell));
        Assert.True(ReferenceEquals(aFix.Context()!.Apply(aShell).TShape(), aShell.TShape()));
    }

    [Fact]
    public void ClosedFlag_IsSetOnTheRebuiltShell()
    {
        // a closed box shell flagged open: the orientation fix sets the flag, on a copy of the shell
        TopoDS_Shell aShell = TopoDS.Shell(TestShapes.Frozen((TopoDS_Shape)ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(), -1, false)));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);
        ShapeFix_Shell aFix = Fixer();
        aFix.Init(aShell);
        aFix.Perform();
        TopoDS_Shape aResult = aFix.Shape();
        aSnap.AssertUnchanged(aShell);
        Assert.False(aShell.Closed());
        Assert.True(aResult.Closed());
        Assert.False(ReferenceEquals(aResult.TShape(), aShell.TShape()));
        foreach (TopoDS_Shape aF in ShapeFixShapes.SubShapes(aShell, TopAbs_FACE))
        {
            Assert.True(ShapeFixShapes.HasTShapeOf(aResult, aF, TopAbs_FACE));
        }
        // the result is the shell the orientation fix rebuilt, flagged as BRep_Tool::IsClosed finds it; OCCT doesn't
        // record it when it reoriented nothing, and the caller's shell wasn't changed (its copy isn't)
        Assert.False(aFix.Context()!.IsRecorded(aShell));
    }

    [Fact]
    public void DisconnectedFaces_MakeTwoShells()
    {
        List<TopoDS_Shape> aFaces = ShapeFixShapes.BoxFaces();
        List<TopoDS_Shape> anOther = ShapeFixShapes.BoxFaces(5.0);
        TopoDS_Shell aShell = TopoDS.Shell(TestShapes.Frozen((TopoDS_Shape)ShapeFixShapes.Shell(aFaces.Concat(anOther), 7)));
        ShapeFix_Shell aFix = Fixer();
        aFix.Init(aShell);
        Assert.True(aFix.Perform());
        Assert.Equal(2, aFix.NbShells());
        TopoDS_Shape aResult = aFix.Shape();
        Assert.Equal(TopAbs_COMPOUND, aResult.ShapeType());
        Assert.Equal(2, ShapeFixShapes.Count(aResult, TopAbs_SHELL));
        Assert.Equal(12, ShapeFixShapes.Count(aResult, TopAbs_FACE));
        Assert.True(ShapeFixShapes.IsOriented(aResult));
        Assert.True(aFix.Shell().TShape()!.IsFrozen());
    }

    [Fact]
    public void MultisharedEdge_KeepsTheFinApart()
    {
        TopoDS_Shell aShell = TopoDS.Shell(TestShapes.Frozen((TopoDS_Shape)ShapeFixShapes.ShellWithFin()));
        ShapeFix_Shell aFix = Fixer();
        aFix.Init(aShell);
        Assert.True(aFix.Perform());
        TopoDS_Shape aResult = aFix.Shape();
        Assert.Equal(TopAbs_COMPOUND, aResult.ShapeType());
        Assert.Equal(2, aFix.NbShells());
        Assert.Equal(7, ShapeFixShapes.Count(aResult, TopAbs_FACE));

        // without the multiconnexity, as one shell (non-manifold)
        var aFix2 = Fixer();
        Assert.True(aFix2.FixFaceOrientation(aShell, false, false) || aFix2.NbShells() == 1);
        Assert.Equal(7, ShapeFixShapes.Count(aFix2.Shape(), TopAbs_FACE));
    }

    [Fact]
    public void NonManifoldFlag_MakesOneShell()
    {
        TopoDS_Shell aShell = TopoDS.Shell(TestShapes.Frozen((TopoDS_Shape)ShapeFixShapes.ShellWithFin()));
        ShapeFix_Shell aFix = Fixer();
        aFix.SetNonManifoldFlag(true);
        aFix.Init(aShell);
        aFix.Perform();
        Assert.Equal(7, ShapeFixShapes.Count(aFix.Shape(), TopAbs_FACE));
        Assert.Equal(1, ShapeFixShapes.Count(aFix.Shape(), TopAbs_SHELL));
    }

    [Fact]
    public void Moebius_ErrorFaces_AndFailStatus()
    {
        TopoDS_Shell aShell = TopoDS.Shell(TestShapes.Frozen((TopoDS_Shape)ShapeFixShapes.Moebius()));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);
        ShapeFix_Shell aFix = Fixer();
        aFix.Init(aShell);
        Assert.False(aFix.Perform()); // OCCT: the fail status isn't DONE2, so Perform says nothing was done
        Assert.True(aFix.Status(ShapeExtend_FAIL));
        Assert.True(aFix.FixFaceOrientation(aShell));
        TopoDS_Compound anErr = aFix.ErrorFaces();
        Assert.True(anErr.TShape()!.IsFrozen());
        Assert.True(ShapeFixShapes.Count(anErr, TopAbs_FACE) > 0);
        aSnap.AssertUnchanged(aShell);
        // the error faces are the caller's faces
        foreach (TopoDS_Shape aF in ShapeFixShapes.SubShapes(anErr, TopAbs_FACE))
        {
            Assert.True(ShapeFixShapes.HasTShapeOf(aShell, aF, TopAbs_FACE));
        }
    }

    [Fact]
    public void UnfrozenShell_IsFixedInPlace()
    {
        List<TopoDS_Shape> aFaces = ShapeFixShapes.BoxFaces();
        TopoDS_Shell aShell = ShapeFixShapes.Shell(aFaces, 0, true);
        ShapeFix_Shell aFix = Fixer();
        aFix.Init(aShell);
        Assert.True(aFix.Perform());
        TopoDS_Shape aResult = aFix.InternalShape();
        Assert.False(aResult.TShape()!.IsFrozen());
        // the faces are the ones the shell was made of (no copy)
        foreach (TopoDS_Shape aF in aFaces)
        {
            Assert.True(ShapeFixShapes.HasTShapeOf(aResult, aF, TopAbs_FACE));
        }
        Assert.True(ShapeFixShapes.IsOriented(aResult));
    }
}

/// <summary>
/// ShapeFix_Solid: SolidFromShell (orientation), Perform on reversed, open and several shells, the open-solid mode, on
/// frozen inputs. The face fixes are switched off.
/// </summary>
public class ShapeFix_SolidTests
{
    private static ShapeFix_Solid Fixer()
    {
        var aFix = new ShapeFix_Solid();
        aFix.FixShellTool().FixFaceMode() = 0;
        return aFix;
    }

    [Fact]
    public void SolidFromShell_ReversedShell_IsReversed()
    {
        TopoDS_Shell aShell = TopoDS.Shell(TestShapes.Frozen((TopoDS_Shape)ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(), -1, true).Reversed()));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);
        var aFix = new ShapeFix_Solid();
        TopoDS_Solid aSolid = aFix.SolidFromShell(aShell);
        aSnap.AssertUnchanged(aShell);
        Assert.True(aFix.Status(ShapeExtend_DONE2));
        Assert.True(aSolid.TShape()!.IsFrozen());
        Assert.Equal(1000.0, ShapeFixShapes.Volume(aSolid), 9);
        // the shell is the caller's
        Assert.True(ShapeFixShapes.HasTShapeOf(aSolid, aShell, TopAbs_SHELL));
    }

    [Fact]
    public void SolidFromShell_GoodShell_NothingReversed()
    {
        TopoDS_Shell aShell = TopoDS.Shell(TestShapes.Frozen((TopoDS_Shape)ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(), -1, true)));
        var aFix = new ShapeFix_Solid();
        TopoDS_Solid aSolid = aFix.SolidFromShell(aShell);
        Assert.False(aFix.Status(ShapeExtend_DONE2));
        Assert.Equal(1000.0, ShapeFixShapes.Volume(aSolid), 9);
        Assert.Equal(TopAbs_FORWARD, new TopoDS_Iterator(aSolid).Value().Orientation());
    }

    [Fact]
    public void Perform_ReversedShell_GivesAPositiveSolid()
    {
        TopoDS_Shell aShell = ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(), -1, true);
        TopoDS_Solid aSolid = TestShapes.Frozen(ShapeFixShapes.Solid(aShell.Reversed()));
        Assert.True(ShapeFixShapes.Volume(aSolid) < 0);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aSolid);

        ShapeFix_Solid aFix = Fixer();
        aFix.Init(aSolid);
        aFix.Perform();
        TopoDS_Shape aResult = aFix.Shape();
        aSnap.AssertUnchanged(aSolid);
        Assert.True(aResult.TShape()!.IsFrozen());
        Assert.Equal(TopAbs_SOLID, aResult.ShapeType());
        Assert.Equal(1000.0, ShapeFixShapes.Volume(aResult), 9);
        Assert.True(ReferenceEquals(aFix.Solid().TShape(), aResult.TShape()));
        foreach (TopoDS_Shape aF in ShapeFixShapes.SubShapes(aSolid, TopAbs_FACE))
        {
            Assert.True(ShapeFixShapes.HasTShapeOf(aResult, aF, TopAbs_FACE));
        }
        Assert.True(aFix.Context()!.Apply(aSolid).IsSame(aResult));
    }

    [Fact]
    public void Perform_ValidSolid_GivesTheInput()
    {
        TopoDS_Solid aSolid = TestShapes.Frozen(TestShapes.Box(10, 10, 10));
        ShapeFix_Solid aFix = Fixer();
        aFix.Init(aSolid);
        aFix.Perform();
        Assert.True(ReferenceEquals(aFix.Shape().TShape(), aSolid.TShape()));
    }

    [Fact]
    public void Perform_OpenShell_GivesTheShell_OrAnOpenSolid()
    {
        List<TopoDS_Shape> aFaces = ShapeFixShapes.BoxFaces();
        aFaces.RemoveAt(1);
        TopoDS_Solid aSolid = TestShapes.Frozen(ShapeFixShapes.Solid(ShapeFixShapes.Shell(aFaces)));

        ShapeFix_Solid aFix = Fixer();
        aFix.Init(aSolid);
        Assert.True(aFix.Perform());
        Assert.True(aFix.Status(ShapeExtend_DONE3));
        Assert.Equal(TopAbs_SHELL, aFix.Shape().ShapeType());

        ShapeFix_Solid aFix2 = Fixer();
        aFix2.CreateOpenSolidMode() = true;
        aFix2.Init(aSolid);
        aFix2.Perform();
        Assert.Equal(TopAbs_SOLID, aFix2.Shape().ShapeType());
        Assert.Equal(5, ShapeFixShapes.Count(aFix2.Shape(), TopAbs_FACE));
    }

    [Fact]
    public void Perform_HoleShell_IsReversed()
    {
        // an outer box and an inner box, both outward: the inner one is a hole and is reversed
        TopoDS_Shell anOuter = ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(10.0), -1, true);
        TopoDS_Shape anInner = new BRepPrimAPI_MakeBox(new gp_Pnt(2, 2, 2), 4, 4, 4).Shell();
        TopoDS_Solid aSolid = TestShapes.Frozen(ShapeFixShapes.Solid(anOuter, anInner));
        ShapeFix_Solid aFix = Fixer();
        aFix.Init(aSolid);
        aFix.Perform();
        TopoDS_Shape aResult = aFix.Shape();
        Assert.Equal(1000.0 - 64.0, ShapeFixShapes.Volume(aResult), 9);
        Assert.Equal(2, ShapeFixShapes.Count(aResult, TopAbs_SHELL));
    }

    [Fact]
    public void Perform_TwoDisjointShells_GiveTwoSolids()
    {
        TopoDS_Shell aBox1 = ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(10.0), -1, true);
        TopoDS_Shape aBox2 = new BRepPrimAPI_MakeBox(new gp_Pnt(20, 0, 0), 4, 4, 4).Shell();
        TopoDS_Solid aSolid = TestShapes.Frozen(ShapeFixShapes.Solid(aBox1, aBox2));
        ShapeFix_Solid aFix = Fixer();
        aFix.Init(aSolid);
        aFix.Perform();
        TopoDS_Shape aResult = aFix.Shape();
        Assert.Equal(TopAbs_COMPOUND, aResult.ShapeType());
        Assert.Equal(2, ShapeFixShapes.Count(aResult, TopAbs_SOLID));
        Assert.Equal(1064.0, ShapeFixShapes.Volume(aResult), 9);
    }
}
