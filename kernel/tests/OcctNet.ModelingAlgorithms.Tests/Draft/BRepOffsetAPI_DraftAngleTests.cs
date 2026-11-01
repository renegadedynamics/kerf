// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.Draft_ErrorStatus;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Draft (unit 7G): BRepOffsetAPI_DraftAngle and Draft_Modification on frozen inputs. The modifier never writes into
/// the caller's shape, and CorrectWires works on a copy of the result where it holds the caller's sub-shapes
/// (decision 1044).
/// </summary>
public class BRepOffsetAPI_DraftAngleTests
{
    private static List<TopoDS_Shape> Faces(in TopoDS_Shape theShape) =>
        BRepOffset_MakeOffsetTests.SubShapes(theShape, TopAbs_FACE);

    private static TopoDS_Face FaceWithNormal(in TopoDS_Shape theShape, gp_Dir theNormal)
    {
        foreach (TopoDS_Shape aF in Faces(theShape))
        {
            if (BRep_Tool.Surface(TopoDS.Face(aF)) is Geom_Plane aPlane)
            {
                gp_Dir aN = aPlane.Pln().Axis().Direction();
                if (aF.Orientation() == TopAbs_Orientation.TopAbs_REVERSED)
                {
                    aN.Reverse();
                }
                if (aN.IsEqual(theNormal, 1.0e-9))
                {
                    return TopoDS.Face(aF);
                }
            }
        }
        throw new InvalidOperationException("no such face");
    }

    [Fact]
    public void Angle_OfPlanesAndCones()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        TopoDS_Face aSide = FaceWithNormal(aBox, new gp_Dir(1, 0, 0));
        Assert.Equal(0.0, Draft.Angle(aSide, new gp_Dir(0, 0, 1)), 12);
        TopoDS_Face aTop = FaceWithNormal(aBox, new gp_Dir(0, 0, 1));
        Assert.Equal(Math.PI / 2, Draft.Angle(aTop, new gp_Dir(0, 0, 1)), 12);

        TopoDS_Shape aCone = new BRepPrimAPI_MakeCone(5, 3, 10).Shape();
        TopoDS_Face aLateral = TopoDS.Face(Faces(aCone).First(f => BRep_Tool.Surface(TopoDS.Face(f)) is Geom_ConicalSurface));
        double anAngle = Draft.Angle(aLateral, new gp_Dir(0, 0, 1));
        Assert.Equal(Math.Atan2(2, 10), anAngle, 9);

        Assert.Throws<Standard_DomainError>(() => Draft.Angle(aLateral, new gp_Dir(1, 0, 0)));
    }

    [Fact]
    public void Draft_OfABoxSide_TurnsTheFace_TheInputIsUnchanged()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        TopoDS_Face aSide = FaceWithNormal(aBox, new gp_Dir(1, 0, 0));

        var aDraft = new BRepOffsetAPI_DraftAngle(aBox);
        aDraft.Add(aSide, new gp_Dir(0, 0, 1), 10.0 * Math.PI / 180.0, new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)));
        Assert.True(aDraft.AddDone());
        aDraft.Build();
        Assert.True(aDraft.IsDone());

        aSnap.AssertUnchanged(aBox);
        TopoDS_Shape aResult = aDraft.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(aResult));
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        // The side turned by 10 degrees about its bottom edge: the top is 10 tan(10) shorter (or longer).
        double aDelta = 0.5 * 10.0 * 10.0 * Math.Tan(10.0 * Math.PI / 180.0) * 10.0;
        double aVolume = BRepOffset_MakeOffsetTests.Volume(aResult);
        Assert.True(Math.Abs(Math.Abs(aVolume - 1000.0) - aDelta) < 1.0e-6, aVolume.ToString());

        // History: the side is generated into its drafted face, the bottom and the far side are the caller's faces.
        Assert.Single(aDraft.Generated(aSide));
        Assert.True(aDraft.Modified(aSide).Count == 0);
        TopoDS_Face aFar = FaceWithNormal(aBox, new gp_Dir(-1, 0, 0));
        Assert.Contains(Faces(aResult), f => f.IsSame(aFar));
        Assert.Single(aDraft.ModifiedFaces());
        Assert.Single(aDraft.ConnectedFaces(aSide));
    }

    [Fact]
    public void Draft_OfACylinder_MakesACone()
    {
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(5, 10).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);
        TopoDS_Face aLateral = TopoDS.Face(Faces(aCyl).First(f => BRep_Tool.Surface(TopoDS.Face(f)) is Geom_CylindricalSurface));

        var aDraft = new BRepOffsetAPI_DraftAngle(aCyl);
        aDraft.Add(aLateral, new gp_Dir(0, 0, 1), 5.0 * Math.PI / 180.0, new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)));
        Assert.True(aDraft.AddDone());
        aDraft.Build();
        Assert.True(aDraft.IsDone());
        aSnap.AssertUnchanged(aCyl);

        TopoDS_Shape aResult = aDraft.Shape();
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.Contains(Faces(aResult), f => BRep_Tool.Surface(TopoDS.Face(f)) is Geom_ConicalSurface
                                             || BRep_Tool.Surface(TopoDS.Face(f)) is Geom_RectangularTrimmedSurface
                                                 { } t && t.BasisSurface() is Geom_ConicalSurface);
        double r1 = 5.0, r2 = 5.0 - 10.0 * Math.Tan(5.0 * Math.PI / 180.0);
        double aCone = Math.PI * 10.0 / 3.0 * (r1 * r1 + r1 * r2 + r2 * r2);
        double r3 = 5.0 + 10.0 * Math.Tan(5.0 * Math.PI / 180.0);
        double aCone2 = Math.PI * 10.0 / 3.0 * (r1 * r1 + r1 * r3 + r3 * r3);
        double aVolume = BRepOffset_MakeOffsetTests.Volume(aResult);
        Assert.True(Math.Abs(aVolume - aCone) < 1.0e-4 || Math.Abs(aVolume - aCone2) < 1.0e-4, aVolume.ToString());
    }

    [Fact]
    public void Add_OfAnUnsupportedFace_ReportsIt()
    {
        TopoDS_Shape aSphere = new BRepPrimAPI_MakeSphere(5).Shape();
        TopoDS_Face aFace = TopoDS.Face(Faces(aSphere)[0]);

        var aDraft = new BRepOffsetAPI_DraftAngle(aSphere);
        aDraft.Add(aFace, new gp_Dir(0, 0, 1), 0.1, new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)));
        Assert.False(aDraft.AddDone());
        Assert.Equal(Draft_FaceRecomputation, aDraft.Status());
        Assert.True(aDraft.ProblematicShape().IsSame(aFace));
        Assert.Throws<Standard_ConstructionError>(() => aDraft.Build());
        // OCCT: the face that failed was never added, so Remove can't take it back (occt-issues.md, unit 7G).
        Assert.Throws<Standard_NoSuchObject>(() => aDraft.Remove(aFace));
        aDraft.Clear();
        Assert.True(aDraft.AddDone());
    }

    [Fact]
    public void TinyAngles_AreIgnored()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        var aDraft = new BRepOffsetAPI_DraftAngle(aBox);
        aDraft.Add(FaceWithNormal(aBox, new gp_Dir(1, 0, 0)), new gp_Dir(0, 0, 1), 1.0e-5,
                   new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)));
        Assert.True(aDraft.AddDone());
        aDraft.Build();
        Assert.True(aDraft.IsDone());
        Assert.True(aDraft.Shape().IsSame(aBox));
    }

    [Fact]
    public void Add_WithoutAShape_Throws()
    {
        var aDraft = new BRepOffsetAPI_DraftAngle();
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 10, 10).Shape();
        Assert.Throws<Standard_NullObject>(() => aDraft.Add(TopoDS.Face(Faces(aBox)[0]), new gp_Dir(0, 0, 1), 0.1,
                                                             new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1))));
    }

    [Fact]
    public void Draft_NextToACylinderCutAcrossItsSeam_KeepsTheCallersShapes()
    {
        // A cylinder cut by a small box across its seam (u = 0, the +X side), with its top face drafted.
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(5, 10).Shape();
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(3, -1, 4), new gp_Pnt(7, 1, 6)).Shape();
        TopoDS_Shape aCut = new BRepAlgoAPI_Cut(aCyl, aBox).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCut);
        // The top face is drafted: the lateral face keeps its surface, and its hole wire is the caller's.
        TopoDS_Face aTop = FaceWithNormal(aCut, new gp_Dir(0, 0, 1));

        var aDraft = new BRepOffsetAPI_DraftAngle(aCut);
        aDraft.Add(aTop, new gp_Dir(1, 0, 0), 5.0 * Math.PI / 180.0, new gp_Pln(new gp_Pnt(0, 0, 10), new gp_Dir(1, 0, 0)));
        Assert.True(aDraft.AddDone());
        aDraft.Build();
        aSnap.AssertUnchanged(aCut);
        Assert.True(aDraft.IsDone());
        TopoDS_Shape aResult = aDraft.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(aResult));
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        // the caller's bottom face (untouched by the draft) is in the result
        TopoDS_Face aBottom = FaceWithNormal(aCut, new gp_Dir(0, 0, -1));
        Assert.Contains(Faces(aResult), f => f.IsSame(aBottom));
    }
}
