// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Offsets;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BRepOffset_MakeSimpleOffset (unit 7E) on frozen inputs: the offset shell is the modifier's copy (decision 973); the
/// solid's walls write into the input's free edges, so the input is copied at the entry, and the caller's shape is
/// never written.
/// </summary>
public class BRepOffset_MakeSimpleOffsetTests
{
    private static List<Shape> SubShapes(in Shape theShape, ShapeType theType)
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

    private static bool AllFrozen(in Shape theShape)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, aMap);
        return aMap.All(s => s.TShape()!.IsFrozen);
    }

    [Fact]
    public void Shell_OfAFrozenBox_IsTheLargerBox_TheInputIsUnchanged()
    {
        Shape aBox = new MakeBox(new Point3D(0, 0, 0), new Point3D(10, 20, 30)).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var anAlgo = new MakeSimpleOffset(aBox, 1.0);
        anAlgo.Perform();

        aSnap.AssertUnchanged(aBox);
        Assert.True(anAlgo.IsDone);
        Assert.Equal(SimpleOffsetStatus.OK, anAlgo.GetError());
        Shape aResult = anAlgo.GetResultShape();
        Assert.True(AllFrozen(aResult));
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.Equal(6, SubShapes(aResult, ShapeType.Face).Count);
        // each face is on its plane moved out by 1 (the sharp edges get large tolerances, as in OCCT)
        foreach (Shape anF in SubShapes(aBox, ShapeType.Face))
        {
            Plane3D anIn = ((PlaneSurface)Tool.Surface(ShapeCasts.Face(anF))!).Plane;
            Plane3D anOut = ((PlaneSurface)Tool.Surface(ShapeCasts.Face(anAlgo.Generated(anF)))!).Plane;
            Assert.True(anIn.Axis.IsParallel(anOut.Axis, 1e-12));
            Assert.Equal(1.0, anIn.DistanceTo(anOut.Location), 9);
        }
        Shape aFace = SubShapes(aBox, ShapeType.Face)[0];
        Shape aGen = anAlgo.Generated(aFace);
        Assert.False(aGen.IsNull);
        Assert.True(aGen.TShape()!.IsFrozen);
        Assert.Contains(SubShapes(aResult, ShapeType.Face), f => f.IsSame(aGen));
        Assert.True(anAlgo.Modified(aFace).IsNull);
    }

    [Fact]
    public void Solid_OfAFrozenFace_BuildsTheWalls_TheInputIsUnchanged()
    {
        Shape aBox = new MakeBox(new Point3D(0, 0, 0), new Point3D(10, 20, 30)).Shape();
        Shape aFace = SubShapes(aBox, ShapeType.Face)[0];
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var anAlgo = new MakeSimpleOffset(aFace, 2.0);
        anAlgo.SetBuildSolidFlag(true);
        anAlgo.Perform();

        aSnap.AssertUnchanged(aBox);
        Assert.True(anAlgo.IsDone, anAlgo.GetErrorMessage());
        Shape aResult = anAlgo.GetResultShape();
        Assert.Equal(ShapeType.Solid, aResult.Type);
        Assert.True(AllFrozen(aResult));
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.Equal(6, SubShapes(aResult, ShapeType.Face).Count);
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aResult, aProps);
        Assert.Equal(2.0 * SurfaceArea(aFace), Math.Abs(aProps.Mass()), 6);

        Shape aGen = anAlgo.Generated(aFace);
        Assert.False(aGen.IsNull);
        Assert.Contains(SubShapes(aResult, ShapeType.Face), f => f.IsSame(aGen));
    }

    [Fact]
    public void Solid_OfAFrozenCylinderSide_IsATube_TheInputIsUnchanged()
    {
        Shape aCyl = new ApiMakeCylinder(5.0, 10.0).Shape();
        Shape aSide = SubShapes(aCyl, ShapeType.Face).First(f => Tool.Surface(ShapeCasts.Face(f)) is CylindricalSurface);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);

        var anAlgo = new MakeSimpleOffset(aSide, 1.0);
        anAlgo.SetBuildSolidFlag(true);
        anAlgo.Perform();

        aSnap.AssertUnchanged(aCyl);
        Assert.True(anAlgo.IsDone, anAlgo.GetErrorMessage());
        Shape aResult = anAlgo.GetResultShape();
        Assert.True(AllFrozen(aResult));
        Assert.True(new Analyzer(aResult).IsValid());
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aResult, aProps);
        Assert.Equal(Math.PI * (6.0 * 6.0 - 5.0 * 5.0) * 10.0, Math.Abs(aProps.Mass()), 3);
    }

    private static double SurfaceArea(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }
}
