// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of ShapeAnalysis_Shell, ShapeAnalysis_FreeBounds (the shell form and the statics) and
/// ShapeAnalysis_FreeBoundsProperties on boxes with faces removed or reversed; frozen inputs stay unchanged.
/// </summary>
public class ShapeAnalysisFreeBoundsTests
{
    private static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(10, 20, 30).Shape();

    private static List<TopoDS_Shape> Faces(in TopoDS_Shape theShape)
    {
        var aList = new List<TopoDS_Shape>();
        for (var anExp = new TopExp_Explorer(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aList.Add(anExp.Current());
        }
        return aList;
    }

    // a shell of the box's faces but those listed; the face of rank theReversed (if any) reversed
    private static TopoDS_Shell ShellWithout(in TopoDS_Shape theBox, int[] theRemoved, int theReversed = -1)
    {
        var aB = new BRep_Builder();
        TopoDS_Shell aShell = new();
        aB.MakeShell(ref aShell);
        List<TopoDS_Shape> aFaces = Faces(theBox);
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

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        int n = 0;
        for (var anExp = new TopExp_Explorer(theShape, theType); anExp.More(); anExp.Next())
        {
            n++;
        }
        return n;
    }

    [Fact]
    public void Shell_ClosedBox_HasNoFreeOrBadEdges()
    {
        TopoDS_Shape aBox = Box();
        var sas = new ShapeAnalysis_Shell();
        Assert.False(sas.CheckOrientedShells(aBox, true));
        Assert.False(sas.HasBadEdges());
        Assert.False(sas.HasFreeEdges());
        Assert.True(sas.HasConnectedEdges());
        Assert.Equal(0, sas.NbLoaded());

        sas.LoadShells(aBox);
        Assert.Equal(1, sas.NbLoaded());
        Assert.True(sas.IsLoaded(sas.Loaded(1)));
        sas.Clear();
        Assert.Equal(0, sas.NbLoaded());
    }

    [Fact]
    public void Shell_OpenAndMisoriented()
    {
        TopoDS_Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var sas = new ShapeAnalysis_Shell();
        Assert.False(sas.CheckOrientedShells(ShellWithout(aBox, [0]), true));
        Assert.True(sas.HasFreeEdges());
        Assert.Equal(4, Count(sas.FreeEdges(), TopAbs_EDGE));
        Assert.True(sas.FreeEdges().TShape()!.IsFrozen());
        Assert.False(sas.InternalFreeEdges().TShape()!.IsFrozen());

        var sas2 = new ShapeAnalysis_Shell();
        TopoDS_Shell aBad = ShellWithout(aBox, [], 2);
        Assert.True(sas2.CheckOrientedShells(aBad, true));
        Assert.True(sas2.HasBadEdges());
        Assert.Equal(4, Count(sas2.BadEdges(), TopAbs_EDGE)); // the reversed face's edges, met twice the same way
        Assert.Equal(1, sas2.NbLoaded());
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void FreeBounds_ShellForm_OneFaceRemoved()
    {
        TopoDS_Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aFB = new ShapeAnalysis_FreeBounds(ShellWithout(aBox, [0]));
        TopoDS_Compound aClosed = aFB.GetClosedWires();
        TopoDS_Compound anOpen = aFB.GetOpenWires();
        Assert.True(aClosed.TShape()!.IsFrozen());
        Assert.Equal(1, Count(aClosed, TopAbs_WIRE));
        Assert.Equal(4, Count(aClosed, TopAbs_EDGE));
        Assert.Equal(0, Count(anOpen, TopAbs_WIRE));
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void FreeBounds_ShellForm_TwoAdjacentFacesRemoved()
    {
        TopoDS_Shape aBox = Box();
        List<TopoDS_Shape> aFaces = Faces(aBox);
        // faces 0 and 2 of BRepPrim's box are adjacent (x = 0 and y = 0)
        var aFB = new ShapeAnalysis_FreeBounds(ShellWithout(aBox, [0, 2]), false, false);
        Assert.Equal(1, Count(aFB.GetClosedWires(), TopAbs_WIRE));
        Assert.Equal(6, Count(aFB.GetClosedWires(), TopAbs_EDGE));
        Assert.Equal(6, aFaces.Count);

        // the closed box: nothing free (null compounds)
        var aNone = new ShapeAnalysis_FreeBounds(aBox);
        Assert.True(aNone.GetClosedWires().IsNull());
        Assert.True(aNone.GetOpenWires().IsNull());
    }

    [Fact]
    public void FreeBounds_Statics_OpenChainAndSplit()
    {
        // a polyline of 3 edges and a separate triangle, given unsorted and partly reversed
        var aB = new BRep_Builder();
        gp_Pnt[] aPts = [new(0, 0, 0), new(1, 0, 0), new(2, 0, 0), new(3, 0, 0)];
        var aEdges = new NCollection_HSequence<TopoDS_Shape>();
        TopoDS_Edge e01 = new BRepBuilderAPI_MakeEdge(aPts[0], aPts[1]).Edge();
        TopoDS_Edge e12 = new BRepBuilderAPI_MakeEdge(aPts[1], aPts[2]).Edge();
        TopoDS_Edge e23 = new BRepBuilderAPI_MakeEdge(aPts[2], aPts[3]).Edge();
        aEdges.Append(e23);
        aEdges.Append(e01.Reversed());
        aEdges.Append(e12);

        NCollection_HSequence<TopoDS_Shape> aWires = ShapeAnalysis_FreeBounds.ConnectEdgesToWires(aEdges, 1e-7, false);
        Assert.Equal(1, aWires.Length());
        Assert.Equal(3, Count(aWires.Value(1), TopAbs_EDGE));
        Assert.False(aWires.Value(1).Closed());
        Assert.True(aWires.Value(1).TShape()!.IsFrozen());

        // dispatch into the caller's frozen compounds: new compounds
        TopoDS_Compound aClosed = new(), anOpen = new();
        ShapeAnalysis_FreeBounds.DispatchWires(aWires, ref aClosed, ref anOpen);
        Assert.Equal(1, Count(anOpen, TopAbs_WIRE));
        Assert.Equal(0, Count(aClosed, TopAbs_WIRE));
        TopoDS_Compound anOpenBefore = anOpen;
        ShapeAnalysis_FreeBounds.DispatchWires(aWires, ref aClosed, ref anOpen);
        Assert.Equal(2, Count(anOpen, TopAbs_WIRE));
        Assert.Equal(1, Count(anOpenBefore, TopAbs_WIRE));

        // a closed loop with a tail: the loop is split off
        TopoDS_Edge e30 = new BRepBuilderAPI_MakeEdge(aPts[3], new gp_Pnt(1, 1, 0)).Edge();
        var aMaker = new BRepBuilderAPI_MakeWire();
        aMaker.Add(e01);
        aMaker.Add(e12);
        aMaker.Add(e23);
        TopoDS_Wire aW = aMaker.Wire();
        var aSeq = new NCollection_HSequence<TopoDS_Shape>();
        aSeq.Append(aW);
        ShapeAnalysis_FreeBounds.SplitWires(aSeq, 1e-7, true, out NCollection_HSequence<TopoDS_Shape> aC,
                                            out NCollection_HSequence<TopoDS_Shape> anO);
        Assert.Equal(0, aC.Length());
        Assert.Equal(1, anO.Length());
        Assert.False(e30.IsNull());
    }

    [Fact]
    public void FreeBounds_ConnectWiresToWires_FillsTheVertexMap()
    {
        // two wires whose ends are close but not shared: connected by distance, the ends merged into new vertices
        TopoDS_Edge e1 = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0)).Edge();
        TopoDS_Edge e2 = new BRepBuilderAPI_MakeEdge(new gp_Pnt(1, 0, 1e-5), new gp_Pnt(2, 0, 0)).Edge();
        var aSeq = new NCollection_HSequence<TopoDS_Shape>();
        aSeq.Append(new BRepBuilderAPI_MakeWire(e1).Wire());
        aSeq.Append(new BRepBuilderAPI_MakeWire(e2).Wire());
        var aVertices = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        NCollection_HSequence<TopoDS_Shape> aWires = ShapeAnalysis_FreeBounds.ConnectWiresToWires(aSeq, 1e-3, false, aVertices);
        Assert.Equal(1, aWires.Length());
        Assert.Equal(2, Count(aWires.Value(1), TopAbs_EDGE));
        Assert.Equal(2, aVertices.Extent());
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> aPair in aVertices)
        {
            Assert.True(aPair.Value.TShape()!.IsFrozen());
        }
    }

    [Fact]
    public void FreeBoundsProperties_OpenBox()
    {
        TopoDS_Shape aBox = Box();
        var aProps = new ShapeAnalysis_FreeBoundsProperties(ShellWithout(aBox, [0]));
        Assert.True(aProps.IsLoaded());
        Assert.True(aProps.Perform());
        Assert.Equal(1, aProps.NbFreeBounds());
        Assert.Equal(1, aProps.NbClosedFreeBounds());
        ShapeAnalysis_FreeBoundData aData = aProps.ClosedFreeBound(1);
        // the face x = 0 of the 10 x 20 x 30 box: 20 x 30
        Assert.Equal(600.0, aData.Area(), 1e-9);
        Assert.Equal(100.0, aData.Perimeter(), 1e-9);
        Assert.Equal(0, aData.NbNotches());
        Assert.True(aData.Ratio() > 0);
        Assert.True(aData.FreeBound().TShape()!.IsFrozen());
    }

    [Fact]
    public void FreeBoundsProperties_Notch()
    {
        // a wire turning back on itself: (0,0)-(10,0)-(0,0.1)
        TopoDS_Edge e1 = new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0)).Edge();
        TopoDS_Edge e2 = new BRepBuilderAPI_MakeEdge(new gp_Pnt(10, 0, 0), new gp_Pnt(0, 0.1, 0)).Edge();
        var aMaker = new BRepBuilderAPI_MakeWire();
        aMaker.Add(e1);
        aMaker.Add(e2);
        TopoDS_Wire aW = aMaker.Wire();
        var aProps = new ShapeAnalysis_FreeBoundsProperties();
        Assert.True(aProps.CheckNotches(aW, 1, out TopoDS_Wire aNotch, out double aDist));
        Assert.Equal(2, Count(aNotch, TopAbs_EDGE));
        Assert.True(aNotch.TShape()!.IsFrozen());
        Assert.Equal(1.0 / Math.Sqrt(100.01), aDist, 1e-12); // from (0,0,0) to the second edge's line
        double aDist0 = aDist;
        Assert.False(aProps.CheckNotches(aW, 5, out _, out aDist));
        Assert.Equal(0.0, aDist);

        var aData = new ShapeAnalysis_FreeBoundData(aW);
        aProps.CheckNotches(aData);
        Assert.Equal(1, aData.NbNotches()); // the notch at the far end; the closing one turns less than 0.95 pi
        Assert.Equal(aDist0, aData.NotchWidth(1));
        Assert.Equal(aData.NotchWidth(1), aData.NotchWidth(aData.Notch(1)));
        aData.Clear();
        Assert.Equal(-1.0, aData.Area());
        Assert.Equal(0, aData.NbNotches());
        Assert.Equal(TopAbs_FORWARD, aW.Orientation());
    }
}
