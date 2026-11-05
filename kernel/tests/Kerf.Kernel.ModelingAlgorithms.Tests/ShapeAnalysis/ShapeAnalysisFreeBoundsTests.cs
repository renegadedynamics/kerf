// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of ShapeAnalysis_Shell, ShapeAnalysis_FreeBounds (the shell form and the statics) and
/// ShapeAnalysis_FreeBoundsProperties on boxes with faces removed or reversed; frozen inputs stay unchanged.
/// </summary>
public class ShapeAnalysisFreeBoundsTests
{
    private static Shape Box() => new MakeBox(10, 20, 30).Shape();

    private static List<Shape> Faces(in Shape theShape)
    {
        var aList = new List<Shape>();
        for (var anExp = new Explorer(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aList.Add(anExp.Current());
        }
        return aList;
    }

    // a shell of the box's faces but those listed; the face of rank theReversed (if any) reversed
    private static Shell ShellWithout(in Shape theBox, int[] theRemoved, int theReversed = -1)
    {
        var aB = new BrepBuilder();
        Shell aShell = new();
        aB.MakeShell(ref aShell);
        List<Shape> aFaces = Faces(theBox);
        for (int i = 0; i < aFaces.Count; i++)
        {
            if (Array.IndexOf(theRemoved, i) >= 0)
            {
                continue;
            }
            aB.Add(aShell, i == theReversed ? aFaces[i].Reversed() : aFaces[i]);
        }
        return aShell;
    }

    private static int Count(in Shape theShape, ShapeType theType)
    {
        int n = 0;
        for (var anExp = new Explorer(theShape, theType); anExp.More(); anExp.Next())
        {
            n++;
        }
        return n;
    }

    [Fact]
    public void Shell_ClosedBox_HasNoFreeOrBadEdges()
    {
        Shape aBox = Box();
        var sas = new ShellAnalysis();
        Assert.False(sas.CheckOrientedShells(aBox, true));
        Assert.False(sas.HasBadEdges);
        Assert.False(sas.HasFreeEdges);
        Assert.True(sas.HasConnectedEdges);
        Assert.Equal(0, sas.LoadedCount);

        sas.LoadShells(aBox);
        Assert.Equal(1, sas.LoadedCount);
        Assert.True(sas.IsLoaded(sas.Loaded(1)));
        sas.Clear();
        Assert.Equal(0, sas.LoadedCount);
    }

    [Fact]
    public void Shell_OpenAndMisoriented()
    {
        Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var sas = new ShellAnalysis();
        Assert.False(sas.CheckOrientedShells(ShellWithout(aBox, [0]), true));
        Assert.True(sas.HasFreeEdges);
        Assert.Equal(4, Count(sas.FreeEdges(), ShapeType.Edge));
        Assert.True(sas.FreeEdges().TShape()!.IsFrozen);
        Assert.False(sas.InternalFreeEdges().TShape()!.IsFrozen);

        var sas2 = new ShellAnalysis();
        Shell aBad = ShellWithout(aBox, [], 2);
        Assert.True(sas2.CheckOrientedShells(aBad, true));
        Assert.True(sas2.HasBadEdges);
        Assert.Equal(4, Count(sas2.BadEdges(), ShapeType.Edge)); // the reversed face's edges, met twice the same way
        Assert.Equal(1, sas2.LoadedCount);
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void FreeBounds_ShellForm_OneFaceRemoved()
    {
        Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aFB = new FreeBounds(ShellWithout(aBox, [0]));
        Compound aClosed = aFB.GetClosedWires();
        Compound anOpen = aFB.GetOpenWires();
        Assert.True(aClosed.TShape()!.IsFrozen);
        Assert.Equal(1, Count(aClosed, ShapeType.Wire));
        Assert.Equal(4, Count(aClosed, ShapeType.Edge));
        Assert.Equal(0, Count(anOpen, ShapeType.Wire));
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void FreeBounds_ShellForm_TwoAdjacentFacesRemoved()
    {
        Shape aBox = Box();
        List<Shape> aFaces = Faces(aBox);
        // faces 0 and 2 of BRepPrim's box are adjacent (x = 0 and y = 0)
        var aFB = new FreeBounds(ShellWithout(aBox, [0, 2]), false, false);
        Assert.Equal(1, Count(aFB.GetClosedWires(), ShapeType.Wire));
        Assert.Equal(6, Count(aFB.GetClosedWires(), ShapeType.Edge));
        Assert.Equal(6, aFaces.Count);

        // the closed box: nothing free (null compounds)
        var aNone = new FreeBounds(aBox);
        Assert.True(aNone.GetClosedWires().IsNull);
        Assert.True(aNone.GetOpenWires().IsNull);
    }

    [Fact]
    public void FreeBounds_Statics_OpenChainAndSplit()
    {
        // a polyline of 3 edges and a separate triangle, given unsorted and partly reversed
        var aB = new BrepBuilder();
        Point3D[] aPts = [new(0, 0, 0), new(1, 0, 0), new(2, 0, 0), new(3, 0, 0)];
        var aEdges = new NCollection_HSequence<Shape>();
        Edge e01 = new MakeEdge(aPts[0], aPts[1]).Edge();
        Edge e12 = new MakeEdge(aPts[1], aPts[2]).Edge();
        Edge e23 = new MakeEdge(aPts[2], aPts[3]).Edge();
        aEdges.Append(e23);
        aEdges.Append(e01.Reversed());
        aEdges.Append(e12);

        NCollection_HSequence<Shape> aWires = FreeBounds.ConnectEdgesToWires(aEdges, 1e-7, false);
        Assert.Equal(1, aWires.Length());
        Assert.Equal(3, Count(aWires.Value(1), ShapeType.Edge));
        Assert.False(aWires.Value(1).ClosedFlag);
        Assert.True(aWires.Value(1).TShape()!.IsFrozen);

        // dispatch into the caller's frozen compounds: new compounds
        Compound aClosed = new(), anOpen = new();
        FreeBounds.DispatchWires(aWires, ref aClosed, ref anOpen);
        Assert.Equal(1, Count(anOpen, ShapeType.Wire));
        Assert.Equal(0, Count(aClosed, ShapeType.Wire));
        Compound anOpenBefore = anOpen;
        FreeBounds.DispatchWires(aWires, ref aClosed, ref anOpen);
        Assert.Equal(2, Count(anOpen, ShapeType.Wire));
        Assert.Equal(1, Count(anOpenBefore, ShapeType.Wire));

        // a closed loop with a tail: the loop is split off
        Edge e30 = new MakeEdge(aPts[3], new Point3D(1, 1, 0)).Edge();
        var aMaker = new MakeWire();
        aMaker.Add(e01);
        aMaker.Add(e12);
        aMaker.Add(e23);
        Wire aW = aMaker.Wire();
        var aSeq = new NCollection_HSequence<Shape>();
        aSeq.Append(aW);
        FreeBounds.SplitWires(aSeq, 1e-7, true, out NCollection_HSequence<Shape> aC,
                                            out NCollection_HSequence<Shape> anO);
        Assert.Equal(0, aC.Length());
        Assert.Equal(1, anO.Length());
        Assert.False(e30.IsNull);
    }

    [Fact]
    public void FreeBounds_ConnectWiresToWires_FillsTheVertexMap()
    {
        // two wires whose ends are close but not shared: connected by distance, the ends merged into new vertices
        Edge e1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        Edge e2 = new MakeEdge(new Point3D(1, 0, 1e-5), new Point3D(2, 0, 0)).Edge();
        var aSeq = new NCollection_HSequence<Shape>();
        aSeq.Append(new MakeWire(e1).Wire());
        aSeq.Append(new MakeWire(e2).Wire());
        var aVertices = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        NCollection_HSequence<Shape> aWires = FreeBounds.ConnectWiresToWires(aSeq, 1e-3, false, aVertices);
        Assert.Equal(1, aWires.Length());
        Assert.Equal(2, Count(aWires.Value(1), ShapeType.Edge));
        Assert.Equal(2, aVertices.Count);
        foreach (KeyValuePair<Shape, Shape> aPair in aVertices)
        {
            Assert.True(aPair.Value.TShape()!.IsFrozen);
        }
    }

    [Fact]
    public void FreeBoundsProperties_OpenBox()
    {
        Shape aBox = Box();
        var aProps = new FreeBoundsPropertiesAnalysis(ShellWithout(aBox, [0]));
        Assert.True(aProps.IsLoaded);
        Assert.True(aProps.Perform());
        Assert.Equal(1, aProps.FreeBoundCount);
        Assert.Equal(1, aProps.ClosedFreeBoundCount);
        FreeBoundDataAnalysis aData = aProps.ClosedFreeBound(1);
        // the face x = 0 of the 10 x 20 x 30 box: 20 x 30
        Assert.Equal(600.0, aData.Area(), 1e-9);
        Assert.Equal(100.0, aData.Perimeter(), 1e-9);
        Assert.Equal(0, aData.NotchCount);
        Assert.True(aData.Ratio() > 0);
        Assert.True(aData.FreeBound().TShape()!.IsFrozen);
    }

    [Fact]
    public void FreeBoundsProperties_Notch()
    {
        // a wire turning back on itself: (0,0)-(10,0)-(0,0.1)
        Edge e1 = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        Edge e2 = new MakeEdge(new Point3D(10, 0, 0), new Point3D(0, 0.1, 0)).Edge();
        var aMaker = new MakeWire();
        aMaker.Add(e1);
        aMaker.Add(e2);
        Wire aW = aMaker.Wire();
        var aProps = new FreeBoundsPropertiesAnalysis();
        Assert.True(aProps.CheckNotches(aW, 1, out Wire aNotch, out double aDist));
        Assert.Equal(2, Count(aNotch, ShapeType.Edge));
        Assert.True(aNotch.TShape()!.IsFrozen);
        Assert.Equal(1.0 / Math.Sqrt(100.01), aDist, 1e-12); // from (0,0,0) to the second edge's line
        double aDist0 = aDist;
        Assert.False(aProps.CheckNotches(aW, 5, out _, out aDist));
        Assert.Equal(0.0, aDist);

        var aData = new FreeBoundDataAnalysis(aW);
        aProps.CheckNotches(aData);
        Assert.Equal(2, aData.NotchCount); // at the far end, and from the last edge back to the first (OCCT wraps round)
        Assert.Equal(aDist0, aData.NotchWidth(1));
        Assert.Equal(aData.NotchWidth(1), aData.NotchWidth(aData.Notch(1)));
        aData.Clear();
        Assert.Equal(-1.0, aData.Area());
        Assert.Equal(0, aData.NotchCount);
        Assert.Equal(Forward, aW.Orientation);
    }
}
