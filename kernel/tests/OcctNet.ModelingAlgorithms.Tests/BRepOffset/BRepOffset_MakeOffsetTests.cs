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

        // the result holds the caller's own faces where the algorithm left them unchanged: the bottom face; the four
        // side faces share edges with the closing face, whose edges get new vertices (changed copies)
        List<TopoDS_Shape> aResultFaces = SubShapes(aResult, TopAbs_FACE);
        int aKept = SubShapes(aBox, TopAbs_FACE).Count(f => aResultFaces.Any(r => r.IsSame(f)));
        Assert.Equal(1, aKept);
        // the history answers for the caller's sub-shapes, with frozen shapes of the result
        Assert.True(anAlgo.IsDeleted(aTop) || anAlgo.Modified(aTop).Count > 0);
        foreach (TopoDS_Shape aF in SubShapes(aBox, TopAbs_FACE))
        {
            foreach (TopoDS_Shape aG in anAlgo.Generated(aF).Concat(anAlgo.Modified(aF)))
            {
                Assert.True(aG.TShape()!.IsFrozen());
                Assert.Contains(aResultFaces, r => r.IsSame(aG));
            }
        }
    }

    [Fact]
    public void Thicken_AFrozenFace_KeepsTheCallersFace_AndItsHistory()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        TopoDS_Shape aFace = SubShapes(aBox, TopAbs_FACE)[0];
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var anAlgo = new BRepOffset_MakeOffset();
        anAlgo.Initialize(aFace, 2.0, 1.0e-5, BRepOffset_Skin, true, false, GeomAbs_Intersection, true, true);
        anAlgo.MakeOffsetShape();

        aSnap.AssertUnchanged(aBox);
        Assert.True(anAlgo.IsDone(), anAlgo.Error().ToString());
        TopoDS_Shape aResult = anAlgo.Shape();
        Assert.True(AllFrozen(aResult));
        Assert.Equal(TopAbs_SOLID, aResult.ShapeType());
        var anArea = new GProp_GProps();
        BRepGProp.SurfaceProperties(aFace, anArea);
        Assert.Equal(anArea.Mass() * 2.0, Math.Abs(Volume(aResult)), 6);
        Assert.False(anAlgo.IsDeleted(aFace));
        Assert.Single(anAlgo.Generated(aFace));
        // the thickened face's edges get pcurves on the walls: changed copies, so the caller's edges are not in the
        // result, but they are not deleted either (OCCT keeps its own, changed in place)
        foreach (TopoDS_Shape anE in SubShapes(aFace, TopAbs_EDGE))
        {
            Assert.False(anAlgo.IsDeleted(anE));
        }
    }

    [Fact]
    public void Initialize_ReusesTheAlgorithm_OnFrozenInputs()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(5, 10).Shape();
        var anAlgo = new BRepOffset_MakeOffset();
        anAlgo.Initialize(aBox, 1.0, 1.0e-7, BRepOffset_Skin, false, false, GeomAbs_Arc);
        anAlgo.MakeOffsetShape();
        TopoDS_Shape aFirst = anAlgo.Shape();
        anAlgo.Initialize(aCyl, -1.0, 1.0e-7, BRepOffset_Skin, false, false, GeomAbs_Arc);
        anAlgo.MakeOffsetShape();
        Assert.True(anAlgo.IsDone());
        Assert.Equal(Math.PI * 16 * 8, Volume(anAlgo.Shape()), 4);
        Assert.Equal(26, SubShapes(aFirst, TopAbs_FACE).Count);
        Assert.True(anAlgo.InitShape().IsSame(aCyl));
    }

    [Fact]
    public void ClosingFaces_AndImages_AnswerForTheCallersShapes()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        TopoDS_Face aTop = TopoDS.Face(SubShapes(aBox, TopAbs_FACE)[5]);
        var anAlgo = new BRepOffset_MakeOffset();
        anAlgo.Initialize(aBox, -1.0, 1.0e-7, BRepOffset_Skin, false, false, GeomAbs_Arc);
        anAlgo.AddFace(aTop);
        anAlgo.MakeThickSolid();
        Assert.True(anAlgo.IsDone());
        Assert.True(anAlgo.ClosingFaces().Contains(aTop));
        BRepAlgo_Image anImage = anAlgo.OffsetFacesFromShapes();
        foreach (TopoDS_Shape aF in SubShapes(aBox, TopAbs_FACE))
        {
            Assert.True(anImage.HasImage(aF));
            foreach (TopoDS_Shape anIm in anImage.Image(aF))
            {
                Assert.True(anIm.TShape()!.IsFrozen());
            }
        }
        Assert.Equal(GeomAbs_Arc, anAlgo.GetJoinType());
        Assert.Equal(BRepOffset_NoError, anAlgo.Error());
    }

    [Fact]
    public void CheckInputData_ReportsANullOffset()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        var anAlgo = new BRepOffset_MakeOffset();
        anAlgo.Initialize(aBox, 0.0, 1.0e-7);
        anAlgo.MakeOffsetShape();
        Assert.False(anAlgo.IsDone());
        Assert.Equal(BRepOffset_NullOffset, anAlgo.Error());
        Assert.True(anAlgo.Shape().IsNull());
    }

    [Theory]
    [InlineData(2.0)]
    [InlineData(-1.0)]
    public void CompleteIntersection_OfAnLShape_IsValid(double theOffset)
    {
        TopoDS_Shape aBox1 = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        TopoDS_Shape aBox2 = new BRepPrimAPI_MakeBox(new gp_Pnt(0, 0, 0), 20, 5, 5).Shape();
        TopoDS_Shape aL = new BRepAlgoAPI_Fuse(aBox1, aBox2).Shape();
        var aL2 = new ShapeUpgrade_UnifySameDomain(aL);
        aL2.Build();
        TopoDS_Shape anL = aL2.Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anL);

        var anAlgo = new BRepOffset_MakeOffset();
        anAlgo.Initialize(anL, theOffset, 1.0e-7, BRepOffset_Skin, true, false, GeomAbs_Intersection);
        anAlgo.MakeOffsetShape();

        aSnap.AssertUnchanged(anL);
        Assert.True(anAlgo.IsDone(), anAlgo.Error().ToString());
        TopoDS_Shape aResult = anAlgo.Shape();
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.True(Volume(aResult) > 0);
    }
}
