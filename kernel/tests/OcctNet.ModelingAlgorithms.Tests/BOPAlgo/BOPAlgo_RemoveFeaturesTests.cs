// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.TestHarness;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_RemoveFeatures (unit 6G; OCCT has no GTest for it): holes, bosses, pockets and chamfer faces removed from
/// solids made by the ported Booleans, the history, the input errors and warnings, frozen inputs left unchanged, frozen
/// results, and serial = parallel.
/// </summary>
public class BOPAlgo_RemoveFeaturesTests
{
    internal static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    internal static double Volume(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    internal static List<TopoDS_Shape> SubShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return [.. Enumerable.Range(1, aMap.Extent()).Select(aMap.FindKey)];
    }

    // The faces of theShape whose surface is of the given kind.
    internal static List<TopoDS_Shape> FacesOf<T>(in TopoDS_Shape theShape) where T : Geom_Surface
        => [.. SubShapes(theShape, TopAbs_FACE).Where(f => BRep_Tool.Surface(TopoDS.Face(f)) is T)];

    // The faces of theShape not lying on the planes x, y or z = 0 or 10 (the box's own planes).
    internal static List<TopoDS_Shape> NonBoxFaces(in TopoDS_Shape theShape)
        => [.. SubShapes(theShape, TopAbs_FACE).Where(f => !OnBoxPlane(f))];

    private static bool OnBoxPlane(TopoDS_Shape theFace)
    {
        if (BRep_Tool.Surface(TopoDS.Face(theFace)) is not Geom_Plane aPlane)
        {
            return false;
        }
        gp_Pln aPln = aPlane.Pln();
        gp_Dir aN = aPln.Axis().Direction();
        gp_Pnt aP = aPln.Location();
        for (int i = 0; i < 3; ++i)
        {
            double aComp = i == 0 ? aN.X() : i == 1 ? aN.Y() : aN.Z();
            double aCoord = i == 0 ? aP.X() : i == 1 ? aP.Y() : aP.Z();
            if (Math.Abs(Math.Abs(aComp) - 1.0) < 1.0e-12 && (Math.Abs(aCoord) < 1.0e-9 || Math.Abs(aCoord - 10.0) < 1.0e-9))
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>A box with a cylindrical hole through it, along z at (x, y).</summary>
    internal static TopoDS_Shape BoxWithHole(double theX = 5.0, double theY = 5.0, double theR = 2.0)
    {
        TopoDS_Shape aTool = new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(theX, theY, -1), new gp_Dir(0, 0, 1)), theR, 12.0).Shape();
        return new BRepAlgoAPI_Cut(Box(), aTool).Shape();
    }

    /// <summary>A box with a cylindrical boss on its top face.</summary>
    internal static TopoDS_Shape BoxWithBoss()
    {
        TopoDS_Shape aTool = new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(5, 5, 10), new gp_Dir(0, 0, 1)), 2.0, 3.0).Shape();
        return new BRepAlgoAPI_Fuse(Box(), aTool).Shape();
    }

    /// <summary>A box with a rectangular pocket in its top face.</summary>
    internal static TopoDS_Shape BoxWithPocket()
    {
        TopoDS_Shape aTool = new BRepPrimAPI_MakeBox(new gp_Pnt(3, 3, 7), 4.0, 4.0, 5.0).Shape();
        return new BRepAlgoAPI_Cut(Box(), aTool).Shape();
    }

    /// <summary>A box with a 45° chamfer face on its edge x = 10, z = 10.</summary>
    internal static TopoDS_Shape BoxWithChamfer()
    {
        // a slab on the side y + z >= 18 of the plane through (y, z) = (8, 10) and (10, 8)
        TopoDS_Shape aTool = new BRepPrimAPI_MakeBox(new gp_Pnt(-1, -10, 0), 12.0, 20.0, 10.0).Shape();
        gp_Trsf aRot = new();
        aRot.SetRotation(new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), -Math.PI / 4.0);
        gp_Trsf aMove = new();
        aMove.SetTranslation(new gp_Vec(0, 9.0, 9.0));
        TopoDS_Shape aTurned = aTool.Moved(new TopLoc_Location(aMove * aRot));
        return new BRepAlgoAPI_Cut(Box(), aTurned).Shape();
    }

    private static BOPAlgo_RemoveFeatures Remove(in TopoDS_Shape theShape, IEnumerable<TopoDS_Shape> theFaces, bool theParallel = false)
    {
        BOPAlgo_RemoveFeatures aRF = new();
        aRF.SetShape(theShape);
        foreach (TopoDS_Shape aF in theFaces)
        {
            aRF.AddFaceToRemove(aF);
        }
        aRF.SetRunParallel(theParallel);
        aRF.Perform();
        return aRF;
    }

    [Fact]
    public void A_hole_is_removed_and_the_box_restored()
    {
        TopoDS_Shape aShape = BoxWithHole();
        string aBefore = BRepTextComparison.Write(aShape);
        List<TopoDS_Shape> aHole = FacesOf<Geom_CylindricalSurface>(aShape);
        Assert.NotEmpty(aHole);

        BOPAlgo_RemoveFeatures aRF = Remove(aShape, aHole);
        Assert.False(aRF.HasErrors());
        Assert.False(aRF.HasWarnings());
        TopoDS_Shape aResult = aRF.Shape();
        // the result has the input's type (a Boolean's compound), with one solid
        Assert.Equal(aShape.ShapeType(), aResult.ShapeType());
        Assert.Single(SubShapes(aResult, TopAbs_SOLID));
        Assert.Equal(1000.0, Volume(aResult), 1.0e-6);
        Assert.Equal(6, SubShapes(aResult, TopAbs_FACE).Count);
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.True(aResult.TShape()!.IsFrozen());

        // the history answers for the input's sub-shapes: the hole's faces are removed, the top and bottom faces modified
        foreach (TopoDS_Shape aF in aHole)
        {
            Assert.True(aRF.IsDeleted(aF));
        }
        TopoDS_Shape aTop = SubShapes(aShape, TopAbs_FACE).Single(f => BRep_Tool.Surface(TopoDS.Face(f)) is Geom_Plane p
                                                                         && Math.Abs(p.Pln().Location().Z() - 10.0) < 1.0e-9
                                                                         && Math.Abs(p.Pln().Axis().Direction().Z()) > 0.5);
        Assert.Single(aRF.Modified(aTop));
        Assert.False(aRF.IsDeleted(aTop));
        Assert.True(aRF.HasDeleted());
        Assert.True(aRF.HasModified());

        // the input is unchanged; a side face is in the result on its own surface (the result's own face, or its copy
        // where UnifySameDomain added p-curves to its edges: decisions 643 and 766)
        Assert.Equal(aBefore, BRepTextComparison.Write(aShape));
        TopoDS_Shape aSide = SubShapes(aShape, TopAbs_FACE).First(f => BRep_Tool.Surface(TopoDS.Face(f)) is Geom_Plane p
                                                                         && Math.Abs(p.Pln().Axis().Direction().X()) > 0.5);
        Assert.False(aRF.IsDeleted(aSide));
        Geom_Surface? aSideSurface = BRep_Tool.Surface(TopoDS.Face(aSide));
        Assert.Contains(SubShapes(aResult, TopAbs_FACE), f => f.IsSame(aSide) || ReferenceEquals(BRep_Tool.Surface(TopoDS.Face(f)), aSideSurface));
    }

    [Fact]
    public void A_boss_is_removed()
    {
        TopoDS_Shape aShape = BoxWithBoss();
        Assert.True(Volume(aShape) > 1000.0);
        List<TopoDS_Shape> aBoss = NonBoxFaces(aShape);
        BOPAlgo_RemoveFeatures aRF = Remove(aShape, aBoss);
        Assert.False(aRF.HasErrors());
        Assert.False(aRF.HasWarnings());
        Assert.Equal(1000.0, Volume(aRF.Shape()), 1.0e-6);
        Assert.Equal(6, SubShapes(aRF.Shape(), TopAbs_FACE).Count);
    }

    [Fact]
    public void A_pocket_is_removed()
    {
        TopoDS_Shape aShape = BoxWithPocket();
        Assert.Equal(1000.0 - 48.0, Volume(aShape), 1.0e-6);
        List<TopoDS_Shape> aPocket = NonBoxFaces(aShape);
        Assert.Equal(5, aPocket.Count);
        BOPAlgo_RemoveFeatures aRF = Remove(aShape, aPocket);
        Assert.False(aRF.HasErrors());
        Assert.False(aRF.HasWarnings());
        Assert.Equal(1000.0, Volume(aRF.Shape()), 1.0e-6);
        Assert.True(new BRepCheck_Analyzer(aRF.Shape()).IsValid());
    }

    [Fact]
    public void A_chamfer_face_is_removed()
    {
        TopoDS_Shape aShape = BoxWithChamfer();
        Assert.True(Volume(aShape) < 1000.0);
        List<TopoDS_Shape> aChamfer = NonBoxFaces(aShape);
        Assert.Single(aChamfer);
        BOPAlgo_RemoveFeatures aRF = Remove(aShape, aChamfer);
        Assert.False(aRF.HasErrors());
        Assert.False(aRF.HasWarnings());
        Assert.Equal(1000.0, Volume(aRF.Shape()), 1.0e-6);
    }

    [Fact]
    public void Two_holes_are_two_features_removed_one_by_one()
    {
        TopoDS_Shape aShape = new BRepAlgoAPI_Cut(BoxWithHole(3.0, 3.0, 1.0),
                                                  new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(7, 7, -1), new gp_Dir(0, 0, 1)), 1.5, 12.0).Shape()).Shape();
        List<TopoDS_Shape> aHoles = FacesOf<Geom_CylindricalSurface>(aShape);
        Assert.True(aHoles.Count >= 2);
        BOPAlgo_RemoveFeatures aRF = Remove(aShape, aHoles);
        Assert.False(aRF.HasErrors());
        Assert.False(aRF.HasWarnings());
        Assert.Equal(1000.0, Volume(aRF.Shape()), 1.0e-6);
        Assert.Equal(6, SubShapes(aRF.Shape(), TopAbs_FACE).Count);
    }

    [Fact]
    public void Input_errors_and_warnings()
    {
        // not a solid
        TopoDS_Shape aFace = SubShapes(Box(), TopAbs_FACE)[0];
        BOPAlgo_RemoveFeatures aRF = Remove(aFace, [aFace]);
        Assert.True(aRF.HasError(typeof(BOPAlgo_AlertTooFewArguments)));

        // faces not in the shape
        aRF = Remove(Box(), SubShapes(Box(), TopAbs_FACE));
        Assert.True(aRF.HasError(typeof(BOPAlgo_AlertNoFacesToRemove)));

        // a compound of a solid and an edge: the edge is left out with a warning, and removed in the history
        TopoDS_Shape aSolid = BoxWithHole();
        TopoDS_Shape anEdge = new BRepBuilderAPI_MakeEdge(new gp_Pnt(20, 0, 0), new gp_Pnt(30, 0, 0)).Shape();
        TopoDS_Compound aC = new();
        new BRep_Builder().MakeCompound(ref aC);
        new BRep_Builder().Add(aC, aSolid);
        new BRep_Builder().Add(aC, anEdge);
        aRF = Remove(aC, FacesOf<Geom_CylindricalSurface>(aSolid));
        Assert.False(aRF.HasErrors());
        Assert.True(aRF.HasWarning(typeof(BOPAlgo_AlertUnsupportedType)));
        Assert.True(aRF.IsDeleted(anEdge));
        Assert.Equal(TopAbs_COMPOUND, aRF.Shape().ShapeType());
        Assert.Equal(1000.0, Volume(aRF.Shape()), 1.0e-6);

        // an empty compound
        TopoDS_Compound anEmpty = new();
        new BRep_Builder().MakeCompound(ref anEmpty);
        aRF = Remove(anEmpty, [aFace]);
        Assert.True(aRF.HasError(typeof(BOPAlgo_AlertEmptyShape)));
    }

    [Fact]
    public void A_feature_that_cannot_be_removed_gives_a_warning_and_the_input()
    {
        // one face of a box: its extended neighbours don't close the gap
        TopoDS_Shape aBox = Box();
        BOPAlgo_RemoveFeatures aRF = Remove(aBox, [SubShapes(aBox, TopAbs_FACE)[0]]);
        Assert.False(aRF.HasErrors());
        Assert.True(aRF.HasWarning(typeof(BOPAlgo_AlertUnableToRemoveTheFeature)));
        Assert.True(aRF.Shape().IsSame(aBox));
    }

    [Fact]
    public void A_frozen_input_face_with_an_internal_edge_is_replaced_by_a_copy()
    {
        // the hole's box, with an internal edge on its face x = 0, away from the hole
        BOPAlgo_Splitter aSplitter = new();
        aSplitter.AddArgument(BoxWithHole());
        aSplitter.AddTool(new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 2, 2), new gp_Pnt(0, 8, 8)).Shape());
        aSplitter.Perform();
        Assert.False(aSplitter.HasErrors());
        TopoDS_Shape aShape = aSplitter.Shape();
        TopoDS_Shape aFaceX0 = SubShapes(aShape, TopAbs_FACE).Single(f => BRep_Tool.Surface(TopoDS.Face(f)) is Geom_Plane p
                                                                          && Math.Abs(p.Pln().Axis().Direction().X()) > 0.5
                                                                          && Math.Abs(p.Pln().Location().X()) < 1.0e-9);
        TopoDS_Shape anInternal = SubShapes(aFaceX0, TopAbs_EDGE).Single(e => e.Orientation() == TopAbs_Orientation.TopAbs_INTERNAL);
        string aBefore = BRepTextComparison.Write(aShape);

        BOPAlgo_RemoveFeatures aRF = Remove(aShape, FacesOf<Geom_CylindricalSurface>(aShape));
        Assert.False(aRF.HasErrors());
        Assert.False(aRF.HasWarnings());
        TopoDS_Shape aResult = aRF.Shape();
        Assert.Equal(1000.0, Volume(aResult), 1.0e-6);

        // OCCT removes the internal wire from the input's face itself; the port's input is unchanged and the face is
        // modified into a copy without it
        Assert.Equal(aBefore, BRepTextComparison.Write(aShape));
        Assert.DoesNotContain(SubShapes(aResult, TopAbs_EDGE), e => e.Orientation() == TopAbs_Orientation.TopAbs_INTERNAL);
        Assert.True(aRF.IsDeleted(anInternal));
        Assert.False(aRF.IsDeleted(aFaceX0));
        Assert.All(aRF.Modified(aFaceX0), f => Assert.Contains(SubShapes(aResult, TopAbs_FACE), r => r.IsSame(f)));
    }

    internal static string RunText(TopoDS_Shape theShape, IReadOnlyList<TopoDS_Shape> theFaces, bool theParallel)
    {
        BOPAlgo_RemoveFeatures aRF = Remove(theShape, theFaces, theParallel);
        TopoDS_Shape aResult = aRF.HasErrors() ? new TopoDS_Shape() : aRF.Shape();
        return (aResult.IsNull() ? "" : BRepTextComparison.Write(aResult))
               + DeterminismHarness.HistoryText([theShape], aResult, aRF.History());
    }

    [Fact]
    public void Serial_equals_parallel()
    {
        TopoDS_Shape aShape = new BRepAlgoAPI_Cut(BoxWithPocket(),
                                                  new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(1.5, 1.5, -1), new gp_Dir(0, 0, 1)), 1.0, 12.0).Shape()).Shape();
        List<TopoDS_Shape> aFaces = NonBoxFaces(aShape);
        string aSerial = RunText(aShape, aFaces, false);
        Assert.Equal(aSerial, DeterminismHarness.AssertSerialEqualsParallel(() => RunText(aShape, aFaces, true), 2));
    }
}
