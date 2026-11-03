// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Meshing;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Meshing for display (unit 4L2): Prs3d_Drawer's deflection settings and their links, Prs3d::GetDeflection, and
/// StdPrs_ToolTriangulatedShape's deflection, check and tessellation, as AIS uses them.
/// </summary>
public class StdPrsToolTriangulatedShapeTests
{
    private static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();

    [Fact]
    public void A_drawer_takes_its_links_settings_where_it_has_none_of_its_own()
    {
        var aDrawer = new Prs3d_Drawer();
        Assert.Equal(Aspect_TypeOfDeflection.Aspect_TOD_RELATIVE, aDrawer.TypeOfDeflection());
        Assert.Equal(0.001, aDrawer.DeviationCoefficient());
        Assert.Equal(20.0 * Math.PI / 180.0, aDrawer.DeviationAngle());
        Assert.Equal(0.0001, aDrawer.MaximalChordialDeviation());
        Assert.True(aDrawer.IsAutoTriangulation());
        Assert.False(aDrawer.HasOwnDeviationCoefficient());

        var aDefaults = new Prs3d_Drawer();
        aDefaults.SetupOwnDefaults();
        aDefaults.SetTypeOfDeflection(Aspect_TypeOfDeflection.Aspect_TOD_ABSOLUTE);
        aDefaults.SetMaximalChordialDeviation(0.5);
        aDefaults.SetDeviationAngle(0.1);
        aDefaults.SetAutoTriangulation(false);
        aDrawer.SetLink(aDefaults);
        Assert.True(aDrawer.HasLink());
        Assert.Equal(Aspect_TypeOfDeflection.Aspect_TOD_ABSOLUTE, aDrawer.TypeOfDeflection());
        Assert.Equal(0.5, aDrawer.MaximalChordialDeviation());
        Assert.Equal(0.1, aDrawer.DeviationAngle());
        Assert.False(aDrawer.IsAutoTriangulation());

        // own settings win; the previous values are kept, and the setters without a value undo them
        aDrawer.SetDeviationCoefficient(0.01);
        Assert.Equal(0.01, aDrawer.DeviationCoefficient());
        Assert.Equal(0.001, aDrawer.PreviousDeviationCoefficient());
        aDrawer.SetDeviationAngle(0.2);
        Assert.Equal(0.1, aDrawer.PreviousDeviationAngle());
        aDrawer.SetDeviationAngle();
        Assert.Equal(0.1, aDrawer.DeviationAngle());
        Assert.Equal(0.0, aDrawer.PreviousDeviationAngle());
        aDrawer.SetAutoTriangulation(true);
        Assert.True(aDrawer.IsAutoTriangulation());
        aDrawer.UnsetOwnIsAutoTriangulation();
        Assert.False(aDrawer.IsAutoTriangulation());
        aDrawer.SetTypeOfDeflection(Aspect_TypeOfDeflection.Aspect_TOD_RELATIVE);
        Assert.Equal(Aspect_TypeOfDeflection.Aspect_TOD_RELATIVE, aDrawer.TypeOfDeflection());
        aDrawer.UnsetOwnTypeOfDeflection();
        Assert.Equal(Aspect_TypeOfDeflection.Aspect_TOD_ABSOLUTE, aDrawer.TypeOfDeflection());
    }

    [Fact]
    public void Prs3d_deflection_is_four_times_the_coefficient_of_the_largest_extent()
    {
        var aBox = new Box();
        Assert.Equal(0.25, Prs3d.GetDeflection(aBox, 0.001, 0.25)); // void
        aBox.Update(0.0, 0.0, 0.0, 10.0, 20.0, 30.0);
        Assert.Equal(30.0 * 0.001 * 4.0, Prs3d.GetDeflection(aBox, 0.001, 0.25));
        Assert.Equal(global::Kerf.Engine.Foundation.Precision.Confusion(), Prs3d.GetDeflection(aBox, 1e-12, 0.25));
        Assert.Equal(30.0 * 0.001 * 4.0, Prs3d.GetDeflection(new Xyz(0.0, 0.0, 0.0), new Xyz(10.0, 20.0, 30.0), 0.001));
    }

    [Fact]
    public void The_deflection_is_relative_to_the_bounding_box_and_stored_in_the_drawer()
    {
        TopoDS_Shape aShape = Box();
        var aBndBox = new Box();
        BRepBndLib.Add(aShape, ref aBndBox, false);
        aBndBox.Get(out _, out _, out double aZMin, out _, out _, out double aZMax);

        var aDrawer = new Prs3d_Drawer();
        double aDeflection = StdPrs_ToolTriangulatedShape.GetDeflection(aShape, aDrawer);
        Assert.Equal((aZMax - aZMin) * 0.001 * 4.0, aDeflection);
        Assert.True(aDrawer.HasOwnMaximalChordialDeviation());
        Assert.Equal(aDeflection, aDrawer.MaximalChordialDeviation());

        // absolute: the drawer's maximal chordial deviation; an empty compound has no box
        aDrawer.SetTypeOfDeflection(Aspect_TypeOfDeflection.Aspect_TOD_ABSOLUTE);
        aDrawer.SetMaximalChordialDeviation(0.3);
        Assert.Equal(0.3, StdPrs_ToolTriangulatedShape.GetDeflection(aShape, aDrawer));
        Assert.Equal(0.0001, StdPrs_ToolTriangulatedShape.GetDeflection(TopoDS_Builder.MakeCompound(), new Prs3d_Drawer()));
    }

    [Fact]
    public void Tessellate_meshes_a_frozen_shape_on_a_copy_once()
    {
        TopoDS_Shape aShape = new BRepPrimAPI_MakeCylinder(5.0, 10.0).Shape();
        var aDrawer = new Prs3d_Drawer();
        Assert.False(StdPrs_ToolTriangulatedShape.IsTriangulated(aShape));
        Assert.False(StdPrs_ToolTriangulatedShape.IsTessellated(aShape, aDrawer));

        Assert.True(StdPrs_ToolTriangulatedShape.Tessellate(aShape, aDrawer, out TopoDS_Shape aMeshed, out BRepTools_History aHistory));
        Assert.False(aMeshed.IsSame(aShape));
        Assert.False(StdPrs_ToolTriangulatedShape.IsTriangulated(aShape)); // the input is never written
        Assert.True(StdPrs_ToolTriangulatedShape.IsTriangulated(aMeshed));
        Assert.True(StdPrs_ToolTriangulatedShape.IsTessellated(aMeshed, aDrawer));
        Assert.True(aHistory.HasModified());

        // the same mesh as the factory's algorithm with the drawer's deflection and angle
        double aDeflection = StdPrs_ToolTriangulatedShape.GetDeflection(aShape, new Prs3d_Drawer());
        TopoDS_Shape anExpected = new BRepMesh_IncrementalMesh(aShape, aDeflection, false, 20.0 * Math.PI / 180.0).Shape();
        var aFaces = new TopExp_Explorer(aMeshed, TopAbs_ShapeEnum.TopAbs_FACE);
        var anExpectedFaces = new TopExp_Explorer(anExpected, TopAbs_ShapeEnum.TopAbs_FACE);
        for (; aFaces.More(); aFaces.Next(), anExpectedFaces.Next())
        {
            Triangulation aTriangulation = BRep_Tool.Triangulation(TopoDS.Face(aFaces.Current()), out _)!;
            Triangulation anExpectedTriangulation = BRep_Tool.Triangulation(TopoDS.Face(anExpectedFaces.Current()), out _)!;
            Assert.Equal(anExpectedTriangulation.TriangleCount, aTriangulation.TriangleCount);
            Assert.Equal(anExpectedTriangulation.Deflection(), aTriangulation.Deflection());
        }

        // meshed finely enough already: nothing is done; a finer deflection meshes again
        Assert.False(StdPrs_ToolTriangulatedShape.Tessellate(aMeshed, aDrawer, out TopoDS_Shape aSame, out _));
        Assert.True(aSame.IsSame(aMeshed));
        var aFiner = new Prs3d_Drawer();
        aFiner.SetDeviationCoefficient(1e-6);
        Assert.True(StdPrs_ToolTriangulatedShape.Tessellate(aMeshed, aFiner));

        // without automatic triangulation, AIS doesn't call it; it still meshes when called
        Assert.True(StdPrs_ToolTriangulatedShape.Tessellate(aShape, new Prs3d_Drawer()));
    }
}
