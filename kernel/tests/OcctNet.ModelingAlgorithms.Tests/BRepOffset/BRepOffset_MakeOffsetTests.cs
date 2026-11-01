// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.BRepOffset_Error;
using static Occt.BRepOffset_Mode;
using static Occt.GeomAbs_JoinType;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepOffset_MakeOffset (unit 7F) on frozen inputs: the caller's shape is copied at the entry (decision 1005), never
/// written; the results are frozen and hold the caller's sub-shapes where nothing changed.
/// </summary>
public class BRepOffset_MakeOffsetTests
{
    internal static List<TopoDS_Shape> SubShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
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

    internal static bool AllFrozen(in TopoDS_Shape theShape)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, aMap);
        return aMap.All(s => s.TShape()!.IsFrozen());
    }

    internal static double Volume(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void Arc_OfAFrozenBox_IsTheRoundedBox_TheInputIsUnchanged()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var anAlgo = new BRepOffset_MakeOffset();
        anAlgo.Initialize(aBox, 1.0, 1.0e-7, BRepOffset_Skin, false, false, GeomAbs_Arc);
        anAlgo.MakeOffsetShape();

        aSnap.AssertUnchanged(aBox);
        Assert.True(anAlgo.IsDone(), anAlgo.Error().ToString());
        TopoDS_Shape aResult = anAlgo.Shape();
        Assert.True(AllFrozen(aResult));
        Assert.Equal(TopAbs_SOLID, aResult.ShapeType());
        Assert.Equal(26, SubShapes(aResult, TopAbs_FACE).Count);
        double a = 10, b = 20, c = 30, r = 1;
        double anExpected = a * b * c + 2 * (a * b + b * c + c * a) * r + Math.PI * r * r * (a + b + c)
                            + 4.0 / 3.0 * Math.PI * r * r * r;
        Assert.Equal(anExpected, Volume(aResult), 3);
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
    }

    [Fact]
    public void Intersection_OfAFrozenBox_IsTheLargerBox()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var anAlgo = new BRepOffset_MakeOffset();
        anAlgo.Initialize(aBox, 1.0, 1.0e-7, BRepOffset_Skin, false, false, GeomAbs_Intersection);
        anAlgo.MakeOffsetShape();

        aSnap.AssertUnchanged(aBox);
        Assert.True(anAlgo.IsDone(), anAlgo.Error().ToString());
        TopoDS_Shape aResult = anAlgo.Shape();
        Assert.Equal(6, SubShapes(aResult, TopAbs_FACE).Count);
        Assert.Equal(12.0 * 22.0 * 32.0, Volume(aResult), 6);
    }

    [Fact]
    public void ThickSolid_OfAFrozenBox_KeepsTheCallersFaces()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        TopoDS_Shape aTop = SubShapes(aBox, TopAbs_FACE)[5];

        var anAlgo = new BRepOffsetAPI_MakeThickSolid();
        var aClosing = new NCollection_List<TopoDS_Shape>();
        aClosing.Append(aTop);
        anAlgo.MakeThickSolidByJoin(aBox, aClosing, -1.0, 1.0e-3, BRepOffset_Skin, false, false, GeomAbs_Arc);

        aSnap.AssertUnchanged(aBox);
        Assert.True(anAlgo.IsDone());
        TopoDS_Shape aResult = anAlgo.Shape();
        Assert.True(AllFrozen(aResult));
        Assert.Equal(TopAbs_SOLID, aResult.ShapeType());
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.Equal(10.0 * 20.0 * 30.0 - 8.0 * 18.0 * 29.0, Volume(aResult), 3);
    }
}
