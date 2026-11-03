// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>Shapes for the tests of ShapeFix_Shell, Solid, Shape and Wireframe.</summary>
internal static class ShapeFixShapes
{
    public static List<Shape> SubShapes(in Shape theShape, ShapeType theType)
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

    public static int Count(in Shape theShape, ShapeType theType) => SubShapes(theShape, theType).Count;

    public static bool HasTShapeOf(in Shape theShape, in Shape theSub, ShapeType theType)
    {
        foreach (Shape aS in SubShapes(theShape, theType))
        {
            if (ReferenceEquals(aS.TShape(), theSub.TShape()))
            {
                return true;
            }
        }
        return false;
    }

    public static double Volume(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    /// <summary>The faces of the box [0, 10]^3 (unfrozen, sharing their edges).</summary>
    public static List<Shape> BoxFaces(double theSize = 10.0)
    {
        var aList = new List<Shape>();
        for (var anExp = new Explorer(TestShapes.Box(theSize, theSize, theSize), ShapeType.Face); anExp.More(); anExp.Next())
        {
            aList.Add(anExp.Current());
        }
        return aList;
    }

    /// <summary>A shell of the faces, the face <paramref name="theReversed"/> reversed (-1: none). Unfrozen.</summary>
    public static Shell Shell(IEnumerable<Shape> theFaces, int theReversed = -1, bool theClosed = false)
    {
        var aB = new BrepBuilder();
        var aShell = new Shell();
        aB.MakeShell(ref aShell);
        int i = 0;
        foreach (Shape aF in theFaces)
        {
            aB.Add(aShell, i++ == theReversed ? aF.Reversed() : aF);
        }
        aShell.Closed(theClosed);
        return aShell;
    }

    public static Solid Solid(params Shape[] theShells)
    {
        var aB = new BrepBuilder();
        var aSolid = new Solid();
        aB.MakeSolid(ref aSolid);
        foreach (Shape aSh in theShells)
        {
            aB.Add(aSolid, aSh);
        }
        return aSolid;
    }

    /// <summary>The five-vertex triangulated Moebius band: its faces can't be oriented consistently. Unfrozen.</summary>
    public static Shell Moebius()
    {
        var aV = new Vertex[5];
        for (int i = 0; i < 5; i++)
        {
            double a = 2.0 * Math.PI * i / 5.0;
            aV[i] = TestShapes.Vertex(new Point3D(10.0 * Math.Cos(a), 10.0 * Math.Sin(a), (i % 2) * 3.0));
        }
        var anEdges = new Dictionary<(int, int), Edge>();
        int[][] aTriangles = [[0, 1, 2], [1, 2, 3], [2, 3, 4], [3, 4, 0], [4, 0, 1]];
        var aFaces = new List<Shape>();
        foreach (int[] t in aTriangles)
        {
            Point3D p0 = Tool.Pnt(aV[t[0]]), p1 = Tool.Pnt(aV[t[1]]), p2 = Tool.Pnt(aV[t[2]]);
            Vector3D aN = new Vector3D(p0, p1).Crossed(new Vector3D(p0, p2));
            aFaces.Add(TestShapes.PolygonFace(aV, new Direction3D(aN), anEdges, t));
        }
        return Shell(aFaces);
    }

    /// <summary>
    /// Three faces sharing one edge (two box faces and a fin), with the rest of the box: a shell with a multishared
    /// edge. Unfrozen.
    /// </summary>
    public static Shell ShellWithFin()
    {
        var aV = new Vertex[10];
        for (int i = 0; i < 8; i++)
        {
            aV[i] = TestShapes.Vertex(new Point3D((i & 1) != 0 ? 10 : 0, (i & 2) != 0 ? 10 : 0, (i & 4) != 0 ? 10 : 0));
        }
        aV[8] = TestShapes.Vertex(new Point3D(20, 0, 0));
        aV[9] = TestShapes.Vertex(new Point3D(20, 0, 10));
        var anEdges = new Dictionary<(int, int), Edge>();
        (int[] Indices, Direction3D Normal)[] aFaces =
        [
            ([0, 2, 3, 1], new Direction3D(0, 0, -1)),
            ([4, 5, 7, 6], new Direction3D(0, 0, 1)),
            ([0, 1, 5, 4], new Direction3D(0, -1, 0)),
            ([2, 6, 7, 3], new Direction3D(0, 1, 0)),
            ([0, 4, 6, 2], new Direction3D(-1, 0, 0)),
            ([1, 3, 7, 5], new Direction3D(1, 0, 0)),
            ([1, 8, 9, 5], new Direction3D(0, -1, 0)), // the fin, on the edge 1-5
        ];
        var aList = new List<Shape>();
        foreach ((int[] anIndices, Direction3D aNormal) in aFaces)
        {
            aList.Add(TestShapes.PolygonFace(aV, aNormal, anEdges, anIndices));
        }
        return Shell(aList);
    }

    public static bool IsOriented(in Shape theShape)
    {
        var aSas = new ShellAnalysis();
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
    private static ShellFixer Fixer()
    {
        var aFix = new ShellFixer();
        aFix.FixFaceMode() = 0;
        return aFix;
    }

    [Fact]
    public void ReversedFace_IsReoriented_TheCallersFacesKept()
    {
        List<Shape> aFaces = ShapeFixShapes.BoxFaces();
        Shell aShell = ShapeCasts.Shell(TestShapes.Frozen((Shape)ShapeFixShapes.Shell(aFaces, 2, true)));
        Assert.False(ShapeFixShapes.IsOriented(aShell));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);

        ShellFixer aFix = Fixer();
        aFix.Init(aShell);
        Assert.True(aFix.Perform());
        Assert.True(aFix.Status(ExtendStatus.DONE2));
        Assert.Equal(1, aFix.ShellCount);

        Shape aResult = aFix.Shape();
        aSnap.AssertUnchanged(aShell);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(ShapeType.Shell, aResult.Type);
        Assert.True(ShapeFixShapes.IsOriented(aResult));
        Assert.True(Tool.IsClosed(aResult));
        Assert.True(ReferenceEquals(aFix.Shell().TShape(), aResult.TShape()));

        // every face is the caller's (only orientations changed)
        foreach (Shape aF in ShapeFixShapes.SubShapes(aShell, ShapeType.Face))
        {
            Assert.True(ShapeFixShapes.HasTShapeOf(aResult, aF, ShapeType.Face));
        }

        // the context answers for the caller's shell
        Shape aValue = aFix.Context()!.Value(aShell);
        Assert.True(aValue.IsSame(aResult));
        Assert.True(aFix.Context()!.Apply(aShell).IsSame(aResult));
    }

    [Fact]
    public void ValidShell_NothingDone_TheCallersFaces()
    {
        Shell aShell = ShapeCasts.Shell(TestShapes.Frozen((Shape)ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(), -1, true)));
        ShellFixer aFix = Fixer();
        aFix.Init(aShell);
        Assert.False(aFix.Perform());
        // as OCCT's, the orientation fix rebuilds the shell even when nothing is done (and records nothing): the
        // shell is new, its faces are the caller's
        Shape aResult = aFix.Shape();
        Assert.False(ReferenceEquals(aResult.TShape(), aShell.TShape()));
        Assert.Equal(6, ShapeFixShapes.Count(aResult, ShapeType.Face));
        foreach (Shape aF in ShapeFixShapes.SubShapes(aShell, ShapeType.Face))
        {
            Assert.True(ShapeFixShapes.HasTShapeOf(aResult, aF, ShapeType.Face));
        }
        Assert.False(aFix.Context()!.IsRecorded(aShell));
        Assert.True(ReferenceEquals(aFix.Context()!.Apply(aShell).TShape(), aShell.TShape()));
    }

    [Fact]
    public void ClosedFlag_IsSetOnTheRebuiltShell()
    {
        // a closed box shell flagged open: the orientation fix sets the flag, on a copy of the shell
        Shell aShell = ShapeCasts.Shell(TestShapes.Frozen((Shape)ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(), -1, false)));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);
        ShellFixer aFix = Fixer();
        aFix.Init(aShell);
        aFix.Perform();
        Shape aResult = aFix.Shape();
        aSnap.AssertUnchanged(aShell);
        Assert.False(aShell.IsClosed);
        Assert.True(aResult.IsClosed);
        Assert.False(ReferenceEquals(aResult.TShape(), aShell.TShape()));
        foreach (Shape aF in ShapeFixShapes.SubShapes(aShell, ShapeType.Face))
        {
            Assert.True(ShapeFixShapes.HasTShapeOf(aResult, aF, ShapeType.Face));
        }
        // the result is the shell the orientation fix rebuilt, flagged as BRep_Tool::IsClosed finds it; OCCT doesn't
        // record it when it reoriented nothing, and the caller's shell wasn't changed (its copy isn't)
        Assert.False(aFix.Context()!.IsRecorded(aShell));
    }

    [Fact]
    public void DisconnectedFaces_MakeTwoShells()
    {
        List<Shape> aFaces = ShapeFixShapes.BoxFaces();
        List<Shape> anOther = ShapeFixShapes.BoxFaces(5.0);
        Shell aShell = ShapeCasts.Shell(TestShapes.Frozen((Shape)ShapeFixShapes.Shell(aFaces.Concat(anOther), 7)));
        ShellFixer aFix = Fixer();
        aFix.Init(aShell);
        Assert.True(aFix.Perform());
        Assert.Equal(2, aFix.ShellCount);
        Shape aResult = aFix.Shape();
        Assert.Equal(ShapeType.Compound, aResult.Type);
        Assert.Equal(2, ShapeFixShapes.Count(aResult, ShapeType.Shell));
        Assert.Equal(12, ShapeFixShapes.Count(aResult, ShapeType.Face));
        Assert.True(ShapeFixShapes.IsOriented(aResult));
        Assert.True(aFix.Shell().TShape()!.IsFrozen);
    }

    [Fact]
    public void MultisharedEdge_KeepsTheFinApart()
    {
        Shell aShell = ShapeCasts.Shell(TestShapes.Frozen((Shape)ShapeFixShapes.ShellWithFin()));
        ShellFixer aFix = Fixer();
        aFix.Init(aShell);
        Assert.True(aFix.Perform());
        Shape aResult = aFix.Shape();
        Assert.Equal(ShapeType.Compound, aResult.Type);
        Assert.Equal(2, aFix.ShellCount);
        Assert.Equal(7, ShapeFixShapes.Count(aResult, ShapeType.Face));

        // without the multiconnexity, as one shell (non-manifold)
        var aFix2 = Fixer();
        Assert.True(aFix2.FixFaceOrientation(aShell, false, false) || aFix2.ShellCount == 1);
        Assert.Equal(7, ShapeFixShapes.Count(aFix2.Shape(), ShapeType.Face));
    }

    [Fact]
    public void NonManifoldFlag_MakesOneShell()
    {
        Shell aShell = ShapeCasts.Shell(TestShapes.Frozen((Shape)ShapeFixShapes.ShellWithFin()));
        ShellFixer aFix = Fixer();
        aFix.SetNonManifoldFlag(true);
        aFix.Init(aShell);
        aFix.Perform();
        Assert.Equal(7, ShapeFixShapes.Count(aFix.Shape(), ShapeType.Face));
        Assert.Equal(1, ShapeFixShapes.Count(aFix.Shape(), ShapeType.Shell));
    }

    [Fact]
    public void Moebius_ErrorFaces_AndFailStatus()
    {
        Shell aShell = ShapeCasts.Shell(TestShapes.Frozen((Shape)ShapeFixShapes.Moebius()));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);
        ShellFixer aFix = Fixer();
        aFix.Init(aShell);
        Assert.False(aFix.Perform()); // OCCT: the fail status isn't DONE2, so Perform says nothing was done
        Assert.True(aFix.Status(ExtendStatus.Fail));
        Assert.True(aFix.FixFaceOrientation(aShell));
        Compound anErr = aFix.ErrorFaces();
        Assert.True(anErr.TShape()!.IsFrozen);
        Assert.True(ShapeFixShapes.Count(anErr, ShapeType.Face) > 0);
        aSnap.AssertUnchanged(aShell);
        // the error faces are the caller's faces
        foreach (Shape aF in ShapeFixShapes.SubShapes(anErr, ShapeType.Face))
        {
            Assert.True(ShapeFixShapes.HasTShapeOf(aShell, aF, ShapeType.Face));
        }
    }

    [Fact]
    public void UnfrozenShell_IsFixedInPlace()
    {
        List<Shape> aFaces = ShapeFixShapes.BoxFaces();
        Shell aShell = ShapeFixShapes.Shell(aFaces, 0, true);
        ShellFixer aFix = Fixer();
        aFix.Init(aShell);
        Assert.True(aFix.Perform());
        Shape aResult = aFix.InternalShape();
        Assert.False(aResult.TShape()!.IsFrozen);
        // the faces are the ones the shell was made of (no copy)
        foreach (Shape aF in aFaces)
        {
            Assert.True(ShapeFixShapes.HasTShapeOf(aResult, aF, ShapeType.Face));
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
    private static SolidFixer Fixer()
    {
        var aFix = new SolidFixer();
        aFix.FixShellTool().FixFaceMode() = 0;
        return aFix;
    }

    [Fact]
    public void SolidFromShell_ReversedShell_IsReversed()
    {
        Shell aShell = ShapeCasts.Shell(TestShapes.Frozen((Shape)ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(), -1, true).Reversed()));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);
        var aFix = new SolidFixer();
        Solid aSolid = aFix.SolidFromShell(aShell);
        aSnap.AssertUnchanged(aShell);
        Assert.True(aFix.Status(ExtendStatus.DONE2));
        Assert.True(aSolid.TShape()!.IsFrozen);
        Assert.Equal(1000.0, ShapeFixShapes.Volume(aSolid), 9);
        // the shell is the caller's
        Assert.True(ShapeFixShapes.HasTShapeOf(aSolid, aShell, ShapeType.Shell));
    }

    [Fact]
    public void SolidFromShell_GoodShell_NothingReversed()
    {
        Shell aShell = ShapeCasts.Shell(TestShapes.Frozen((Shape)ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(), -1, true)));
        var aFix = new SolidFixer();
        Solid aSolid = aFix.SolidFromShell(aShell);
        Assert.False(aFix.Status(ExtendStatus.DONE2));
        Assert.Equal(1000.0, ShapeFixShapes.Volume(aSolid), 9);
        Assert.Equal(Forward, new Iterator(aSolid).Value().Orientation);
    }

    [Fact]
    public void Perform_ReversedShell_GivesAPositiveSolid()
    {
        Shell aShell = ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(), -1, true);
        Solid aSolid = TestShapes.Frozen(ShapeFixShapes.Solid(aShell.Reversed()));
        Assert.True(ShapeFixShapes.Volume(aSolid) < 0);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aSolid);

        SolidFixer aFix = Fixer();
        aFix.Init(aSolid);
        aFix.Perform();
        Shape aResult = aFix.Shape();
        aSnap.AssertUnchanged(aSolid);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(ShapeType.Solid, aResult.Type);
        Assert.Equal(1000.0, ShapeFixShapes.Volume(aResult), 9);
        Assert.True(ReferenceEquals(aFix.Solid().TShape(), aResult.TShape()));
        foreach (Shape aF in ShapeFixShapes.SubShapes(aSolid, ShapeType.Face))
        {
            Assert.True(ShapeFixShapes.HasTShapeOf(aResult, aF, ShapeType.Face));
        }
        Assert.True(aFix.Context()!.Apply(aSolid).IsSame(aResult));
    }

    [Fact]
    public void Perform_ValidSolid_GivesTheInput()
    {
        Solid aSolid = TestShapes.Frozen(TestShapes.Box(10, 10, 10));
        SolidFixer aFix = Fixer();
        aFix.Init(aSolid);
        aFix.Perform();
        Assert.True(ReferenceEquals(aFix.Shape().TShape(), aSolid.TShape()));
    }

    [Fact]
    public void Perform_OpenShell_GivesTheShell_OrAnOpenSolid()
    {
        List<Shape> aFaces = ShapeFixShapes.BoxFaces();
        aFaces.RemoveAt(1);
        Solid aSolid = TestShapes.Frozen(ShapeFixShapes.Solid(ShapeFixShapes.Shell(aFaces)));

        SolidFixer aFix = Fixer();
        aFix.Init(aSolid);
        Assert.True(aFix.Perform());
        Assert.True(aFix.Status(ExtendStatus.DONE3));
        Assert.Equal(ShapeType.Shell, aFix.Shape().Type);

        SolidFixer aFix2 = Fixer();
        aFix2.CreateOpenSolidMode() = true;
        aFix2.Init(aSolid);
        aFix2.Perform();
        Assert.Equal(ShapeType.Solid, aFix2.Shape().Type);
        Assert.Equal(5, ShapeFixShapes.Count(aFix2.Shape(), ShapeType.Face));
    }

    [Fact]
    public void Perform_HoleShell_IsReversed()
    {
        // an outer box and an inner box, both outward: the inner one is a hole and is reversed
        Shell anOuter = ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(10.0), -1, true);
        Shape anInner = new MakeBox(new Point3D(2, 2, 2), 4, 4, 4).Shell();
        Solid aSolid = TestShapes.Frozen(ShapeFixShapes.Solid(anOuter, anInner));
        SolidFixer aFix = Fixer();
        aFix.Init(aSolid);
        aFix.Perform();
        Shape aResult = aFix.Shape();
        Assert.Equal(1000.0 - 64.0, ShapeFixShapes.Volume(aResult), 9);
        Assert.Equal(2, ShapeFixShapes.Count(aResult, ShapeType.Shell));
    }

    [Fact]
    public void Perform_TwoDisjointShells_GiveTwoSolids()
    {
        Shell aBox1 = ShapeFixShapes.Shell(ShapeFixShapes.BoxFaces(10.0), -1, true);
        Shape aBox2 = new MakeBox(new Point3D(20, 0, 0), 4, 4, 4).Shell();
        Solid aSolid = TestShapes.Frozen(ShapeFixShapes.Solid(aBox1, aBox2));
        SolidFixer aFix = Fixer();
        aFix.Init(aSolid);
        aFix.Perform();
        Shape aResult = aFix.Shape();
        Assert.Equal(ShapeType.Compound, aResult.Type);
        Assert.Equal(2, ShapeFixShapes.Count(aResult, ShapeType.Solid));
        Assert.Equal(1064.0, ShapeFixShapes.Volume(aResult), 9);
    }
}
