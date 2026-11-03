// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.TestHarness;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BOPAlgo_RemoveFeatures (unit 6G; OCCT has no GTest for it): holes, bosses, pockets and chamfer faces removed from
/// solids made by the ported Booleans, the history, the input errors and warnings, frozen inputs left unchanged, frozen
/// results, and serial = parallel.
/// </summary>
public class BOPAlgo_RemoveFeaturesTests
{
    internal static Shape Box() => new MakeBox(10.0, 10.0, 10.0).Shape();

    internal static double Volume(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    internal static List<Shape> SubShapes(in Shape theShape, ShapeType theType)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return [.. Enumerable.Range(1, aMap.Extent()).Select(aMap.FindKey)];
    }

    // The faces of theShape whose surface is of the given kind.
    internal static List<Shape> FacesOf<T>(in Shape theShape) where T : Surface
        => [.. SubShapes(theShape, ShapeType.Face).Where(f => Tool.Surface(ShapeCasts.Face(f)) is T)];

    // The faces of theShape not lying on the planes x, y or z = 0 or 10 (the box's own planes).
    internal static List<Shape> NonBoxFaces(in Shape theShape)
        => [.. SubShapes(theShape, ShapeType.Face).Where(f => !OnBoxPlane(f))];

    private static bool OnBoxPlane(Shape theFace)
    {
        if (Tool.Surface(ShapeCasts.Face(theFace)) is not PlaneSurface aPlane)
        {
            return false;
        }
        Plane3D aPln = aPlane.Pln();
        Direction3D aN = aPln.Axis().Direction();
        Point3D aP = aPln.Location();
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
    internal static Shape BoxWithHole(double theX = 5.0, double theY = 5.0, double theR = 2.0)
    {
        Shape aTool = new ApiMakeCylinder(new Frame3D(new Point3D(theX, theY, -1), new Direction3D(0, 0, 1)), theR, 12.0).Shape();
        return new Cut(Box(), aTool).Shape();
    }

    /// <summary>A box with a cylindrical boss on its top face.</summary>
    internal static Shape BoxWithBoss()
    {
        Shape aTool = new ApiMakeCylinder(new Frame3D(new Point3D(5, 5, 10), new Direction3D(0, 0, 1)), 2.0, 3.0).Shape();
        return new Fuse(Box(), aTool).Shape();
    }

    /// <summary>A box with a rectangular pocket in its top face.</summary>
    internal static Shape BoxWithPocket()
    {
        Shape aTool = new MakeBox(new Point3D(3, 3, 7), 4.0, 4.0, 5.0).Shape();
        return new Cut(Box(), aTool).Shape();
    }

    /// <summary>A box with a 45° chamfer face on its edge x = 10, z = 10.</summary>
    internal static Shape BoxWithChamfer()
    {
        // a slab on the side y + z >= 18 of the plane through (y, z) = (8, 10) and (10, 8)
        Shape aTool = new MakeBox(new Point3D(-1, -10, 0), 12.0, 20.0, 10.0).Shape();
        Transform3D aRot = new();
        aRot.SetRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), -Math.PI / 4.0);
        Transform3D aMove = new();
        aMove.SetTranslation(new Vector3D(0, 9.0, 9.0));
        Shape aTurned = aTool.Moved(new Placement(aMove * aRot));
        return new Cut(Box(), aTurned).Shape();
    }

    private static RemoveFeaturesAlgorithm Remove(in Shape theShape, IEnumerable<Shape> theFaces, bool theParallel = false)
    {
        RemoveFeaturesAlgorithm aRF = new();
        aRF.SetShape(theShape);
        foreach (Shape aF in theFaces)
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
        Shape aShape = BoxWithHole();
        string aBefore = BRepTextComparison.Write(aShape);
        List<Shape> aHole = FacesOf<CylindricalSurface>(aShape);
        Assert.NotEmpty(aHole);

        RemoveFeaturesAlgorithm aRF = Remove(aShape, aHole);
        Assert.False(aRF.HasErrors);
        Assert.False(aRF.HasWarnings);
        Shape aResult = aRF.Shape();
        // the result has the input's type (a Boolean's compound), with one solid
        Assert.Equal(aShape.ShapeType(), aResult.ShapeType());
        Assert.Single(SubShapes(aResult, ShapeType.Solid));
        Assert.Equal(1000.0, Volume(aResult), 1.0e-6);
        Assert.Equal(6, SubShapes(aResult, ShapeType.Face).Count);
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.True(aResult.TShape()!.IsFrozen);

        // the history answers for the input's sub-shapes: the hole's faces are removed, the top and bottom faces modified
        foreach (Shape aF in aHole)
        {
            Assert.True(aRF.IsDeleted(aF));
        }
        Shape aTop = SubShapes(aShape, ShapeType.Face).Single(f => Tool.Surface(ShapeCasts.Face(f)) is PlaneSurface p
                                                                         && Math.Abs(p.Pln().Location().Z() - 10.0) < 1.0e-9
                                                                         && Math.Abs(p.Pln().Axis().Direction().Z()) > 0.5);
        Assert.Single(aRF.Modified(aTop));
        Assert.False(aRF.IsDeleted(aTop));
        Assert.True(aRF.HasDeleted);
        Assert.True(aRF.HasModified);

        // the input is unchanged; a side face is in the result on its own surface (the result's own face, or its copy
        // where UnifySameDomain added p-curves to its edges: decisions 643 and 766)
        Assert.Equal(aBefore, BRepTextComparison.Write(aShape));
        Shape aSide = SubShapes(aShape, ShapeType.Face).First(f => Tool.Surface(ShapeCasts.Face(f)) is PlaneSurface p
                                                                         && Math.Abs(p.Pln().Axis().Direction().X()) > 0.5);
        Assert.False(aRF.IsDeleted(aSide));
        Surface? aSideSurface = Tool.Surface(ShapeCasts.Face(aSide));
        Assert.Contains(SubShapes(aResult, ShapeType.Face), f => f.IsSame(aSide) || ReferenceEquals(Tool.Surface(ShapeCasts.Face(f)), aSideSurface));
    }

    [Fact]
    public void A_boss_is_removed()
    {
        Shape aShape = BoxWithBoss();
        Assert.True(Volume(aShape) > 1000.0);
        List<Shape> aBoss = NonBoxFaces(aShape);
        RemoveFeaturesAlgorithm aRF = Remove(aShape, aBoss);
        Assert.False(aRF.HasErrors);
        Assert.False(aRF.HasWarnings);
        Assert.Equal(1000.0, Volume(aRF.Shape()), 1.0e-6);
        Assert.Equal(6, SubShapes(aRF.Shape(), ShapeType.Face).Count);
    }

    [Fact]
    public void A_pocket_is_removed()
    {
        Shape aShape = BoxWithPocket();
        Assert.Equal(1000.0 - 48.0, Volume(aShape), 1.0e-6);
        List<Shape> aPocket = NonBoxFaces(aShape);
        Assert.Equal(5, aPocket.Count);
        RemoveFeaturesAlgorithm aRF = Remove(aShape, aPocket);
        Assert.False(aRF.HasErrors);
        Assert.False(aRF.HasWarnings);
        Assert.Equal(1000.0, Volume(aRF.Shape()), 1.0e-6);
        Assert.True(new Analyzer(aRF.Shape()).IsValid());
    }

    [Fact]
    public void A_chamfer_face_is_removed()
    {
        Shape aShape = BoxWithChamfer();
        Assert.True(Volume(aShape) < 1000.0);
        List<Shape> aChamfer = NonBoxFaces(aShape);
        Assert.Single(aChamfer);
        RemoveFeaturesAlgorithm aRF = Remove(aShape, aChamfer);
        Assert.False(aRF.HasErrors);
        Assert.False(aRF.HasWarnings);
        Assert.Equal(1000.0, Volume(aRF.Shape()), 1.0e-6);
    }

    [Fact]
    public void Two_holes_are_two_features_removed_one_by_one()
    {
        Shape aShape = new Cut(BoxWithHole(3.0, 3.0, 1.0),
                                                  new ApiMakeCylinder(new Frame3D(new Point3D(7, 7, -1), new Direction3D(0, 0, 1)), 1.5, 12.0).Shape()).Shape();
        List<Shape> aHoles = FacesOf<CylindricalSurface>(aShape);
        Assert.True(aHoles.Count >= 2);
        RemoveFeaturesAlgorithm aRF = Remove(aShape, aHoles);
        Assert.False(aRF.HasErrors);
        Assert.False(aRF.HasWarnings);
        Assert.Equal(1000.0, Volume(aRF.Shape()), 1.0e-6);
        Assert.Equal(6, SubShapes(aRF.Shape(), ShapeType.Face).Count);
    }

    [Fact]
    public void Input_errors_and_warnings()
    {
        // not a solid
        Shape aFace = SubShapes(Box(), ShapeType.Face)[0];
        RemoveFeaturesAlgorithm aRF = Remove(aFace, [aFace]);
        Assert.True(aRF.HasError(typeof(AlertTooFewArguments)));

        // faces not in the shape
        aRF = Remove(Box(), SubShapes(Box(), ShapeType.Face));
        Assert.True(aRF.HasError(typeof(AlertNoFacesToRemove)));

        // a compound of a solid and an edge: the edge is left out with a warning, and removed in the history
        Shape aSolid = BoxWithHole();
        Shape anEdge = new MakeEdge(new Point3D(20, 0, 0), new Point3D(30, 0, 0)).Shape();
        Compound aC = new();
        new BrepBuilder().MakeCompound(ref aC);
        new BrepBuilder().Add(aC, aSolid);
        new BrepBuilder().Add(aC, anEdge);
        aRF = Remove(aC, FacesOf<CylindricalSurface>(aSolid));
        Assert.False(aRF.HasErrors);
        Assert.True(aRF.HasWarning(typeof(AlertUnsupportedType)));
        Assert.True(aRF.IsDeleted(anEdge));
        Assert.Equal(ShapeType.Compound, aRF.Shape().ShapeType());
        Assert.Equal(1000.0, Volume(aRF.Shape()), 1.0e-6);

        // an empty compound
        Compound anEmpty = new();
        new BrepBuilder().MakeCompound(ref anEmpty);
        aRF = Remove(anEmpty, [aFace]);
        Assert.True(aRF.HasError(typeof(AlertEmptyShape)));
    }

    [Fact]
    public void A_feature_that_cannot_be_removed_gives_a_warning_and_the_input()
    {
        // one face of a box: its extended neighbours don't close the gap
        Shape aBox = Box();
        RemoveFeaturesAlgorithm aRF = Remove(aBox, [SubShapes(aBox, ShapeType.Face)[0]]);
        Assert.False(aRF.HasErrors);
        Assert.True(aRF.HasWarning(typeof(AlertUnableToRemoveTheFeature)));
        Assert.True(aRF.Shape().IsSame(aBox));
    }

    [Fact]
    public void A_frozen_input_face_with_an_internal_edge_is_replaced_by_a_copy()
    {
        // the hole's box, with an internal edge on its face x = 0, away from the hole
        Splitter aSplitter = new();
        aSplitter.AddArgument(BoxWithHole());
        aSplitter.AddTool(new MakeEdge(new Point3D(0, 2, 2), new Point3D(0, 8, 8)).Shape());
        aSplitter.Perform();
        Assert.False(aSplitter.HasErrors);
        Shape aShape = aSplitter.Shape();
        Shape aFaceX0 = SubShapes(aShape, ShapeType.Face).Single(f => Tool.Surface(ShapeCasts.Face(f)) is PlaneSurface p
                                                                          && Math.Abs(p.Pln().Axis().Direction().X()) > 0.5
                                                                          && Math.Abs(p.Pln().Location().X()) < 1.0e-9);
        Shape anInternal = SubShapes(aFaceX0, ShapeType.Edge).Single(e => e.Orientation() == Orientation.Internal);
        string aBefore = BRepTextComparison.Write(aShape);

        RemoveFeaturesAlgorithm aRF = Remove(aShape, FacesOf<CylindricalSurface>(aShape));
        Assert.False(aRF.HasErrors);
        Assert.False(aRF.HasWarnings);
        Shape aResult = aRF.Shape();
        Assert.Equal(1000.0, Volume(aResult), 1.0e-6);

        // OCCT removes the internal wire from the input's face itself; the port's input is unchanged and the face is
        // modified into a copy without it
        Assert.Equal(aBefore, BRepTextComparison.Write(aShape));
        Assert.DoesNotContain(SubShapes(aResult, ShapeType.Edge), e => e.Orientation() == Orientation.Internal);
        Assert.True(aRF.IsDeleted(anInternal));
        Assert.False(aRF.IsDeleted(aFaceX0));
        Assert.All(aRF.Modified(aFaceX0), f => Assert.Contains(SubShapes(aResult, ShapeType.Face), r => r.IsSame(f)));
    }

    internal static string RunText(Shape theShape, IReadOnlyList<Shape> theFaces, bool theParallel)
    {
        RemoveFeaturesAlgorithm aRF = Remove(theShape, theFaces, theParallel);
        Shape aResult = aRF.HasErrors ? new Shape() : aRF.Shape();
        return (aResult.IsNull ? "" : BRepTextComparison.Write(aResult))
               + DeterminismHarness.HistoryText([theShape], aResult, aRF.History());
    }

    [Fact]
    public void Serial_equals_parallel()
    {
        Shape aShape = new Cut(BoxWithPocket(),
                                                  new ApiMakeCylinder(new Frame3D(new Point3D(1.5, 1.5, -1), new Direction3D(0, 0, 1)), 1.0, 12.0).Shape()).Shape();
        List<Shape> aFaces = NonBoxFaces(aShape);
        string aSerial = RunText(aShape, aFaces, false);
        Assert.Equal(aSerial, DeterminismHarness.AssertSerialEqualsParallel(() => RunText(aShape, aFaces, true), 2));
    }
}
