// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepOffset_MakeSimpleOffset (unit 7E) on frozen inputs: the offset shell is the modifier's copy (decision 973); the
/// solid's walls write into the input's free edges, so the input is copied at the entry, and the caller's shape is
/// never written.
/// </summary>
public class BRepOffset_MakeSimpleOffsetTests
{
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

    private static bool AllFrozen(in TopoDS_Shape theShape)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, aMap);
        return aMap.All(s => s.TShape()!.IsFrozen());
    }

    [Fact]
    public void Shell_OfAFrozenBox_IsTheLargerBox_TheInputIsUnchanged()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 20, 30)).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var anAlgo = new BRepOffset_MakeSimpleOffset(aBox, 1.0);
        anAlgo.Perform();

        aSnap.AssertUnchanged(aBox);
        Assert.True(anAlgo.IsDone());
        Assert.Equal(BRepOffsetSimple_Status.BRepOffsetSimple_OK, anAlgo.GetError());
        TopoDS_Shape aResult = anAlgo.GetResultShape();
        Assert.True(AllFrozen(aResult));
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.Equal(6, SubShapes(aResult, TopAbs_FACE).Count);
        // each face is on its plane moved out by 1 (the sharp edges get large tolerances, as in OCCT)
        foreach (TopoDS_Shape anF in SubShapes(aBox, TopAbs_FACE))
        {
            gp_Pln anIn = ((Geom_Plane)BRep_Tool.Surface(TopoDS.Face(anF))!).Pln();
            gp_Pln anOut = ((Geom_Plane)BRep_Tool.Surface(TopoDS.Face(anAlgo.Generated(anF)))!).Pln();
            Assert.True(anIn.Axis().IsParallel(anOut.Axis(), 1e-12));
            Assert.Equal(1.0, anIn.Distance(anOut.Location()), 9);
        }
        TopoDS_Shape aFace = SubShapes(aBox, TopAbs_FACE)[0];
        TopoDS_Shape aGen = anAlgo.Generated(aFace);
        Assert.False(aGen.IsNull());
        Assert.True(aGen.TShape()!.IsFrozen());
        Assert.Contains(SubShapes(aResult, TopAbs_FACE), f => f.IsSame(aGen));
        Assert.True(anAlgo.Modified(aFace).IsNull());
    }

    [Fact]
    public void Solid_OfAFrozenFace_BuildsTheWalls_TheInputIsUnchanged()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 20, 30)).Shape();
        TopoDS_Shape aFace = SubShapes(aBox, TopAbs_FACE)[0];
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var anAlgo = new BRepOffset_MakeSimpleOffset(aFace, 2.0);
        anAlgo.SetBuildSolidFlag(true);
        anAlgo.Perform();

        aSnap.AssertUnchanged(aBox);
        Assert.True(anAlgo.IsDone(), anAlgo.GetErrorMessage());
        TopoDS_Shape aResult = anAlgo.GetResultShape();
        Assert.Equal(TopAbs_SOLID, aResult.ShapeType());
        Assert.True(AllFrozen(aResult));
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.Equal(6, SubShapes(aResult, TopAbs_FACE).Count);
        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(aResult, aProps);
        Assert.Equal(2.0 * SurfaceArea(aFace), Math.Abs(aProps.Mass()), 6);

        TopoDS_Shape aGen = anAlgo.Generated(aFace);
        Assert.False(aGen.IsNull());
        Assert.Contains(SubShapes(aResult, TopAbs_FACE), f => f.IsSame(aGen));
    }

    [Fact]
    public void Solid_OfAFrozenCylinderSide_IsATube_TheInputIsUnchanged()
    {
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(5.0, 10.0).Shape();
        TopoDS_Shape aSide = SubShapes(aCyl, TopAbs_FACE).First(f => BRep_Tool.Surface(TopoDS.Face(f)) is Geom_CylindricalSurface);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);

        var anAlgo = new BRepOffset_MakeSimpleOffset(aSide, 1.0);
        anAlgo.SetBuildSolidFlag(true);
        anAlgo.Perform();

        aSnap.AssertUnchanged(aCyl);
        Assert.True(anAlgo.IsDone(), anAlgo.GetErrorMessage());
        TopoDS_Shape aResult = anAlgo.GetResultShape();
        Assert.True(AllFrozen(aResult));
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(aResult, aProps);
        Assert.Equal(Math.PI * (6.0 * 6.0 - 5.0 * 5.0) * 10.0, Math.Abs(aProps.Mass()), 3);
    }

    private static double SurfaceArea(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }
}
