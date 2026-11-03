// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Schematiq.Collections;
using OcctNet.ModelingData.Tests;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of BRepFill_Evolved and BRepOffsetAPI_MakeEvolved (unit 5O2): vertical segments (walls on the spine's
/// parallels) and horizontal ones (faces between two parallels), on square and L-shaped spines given as a wire or a
/// face, open and solid. The results are checked valid, with their exact areas and volumes; the caller's frozen spine
/// and profile are never written. Profiles with slanted parts are swept by BRepFill_Pipe (unit 5S2) and compared with
/// native values; the volume mode (unit 7G) builds with BRepFill_AdvancedEvolved. OCCT V8_0_1 has no GTests for these classes, and
/// native BRepOffsetAPI_MakeEvolved isn't bound for the oracle (tools/evorec records its values).
/// </summary>
public class BRepFill_EvolvedTests
{
    private static TopoDS_Wire Polygon(bool theClosed, params double[] theXYZ)
    {
        BRepBuilderAPI_MakePolygon aMP = new();
        for (int i = 0; i < theXYZ.Length; i += 3)
        {
            aMP.Add(new Point3D(theXYZ[i], theXYZ[i + 1], theXYZ[i + 2]));
        }
        if (theClosed)
        {
            aMP.Close();
        }
        return aMP.Wire();
    }

    private static TopoDS_Wire Square() => Polygon(true, 0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0);

    private static TopoDS_Wire LShape() => Polygon(true, 0, 0, 0, 10, 0, 0, 10, 4, 0, 4, 4, 0, 4, 10, 0, 0, 10, 0);

    private static string Text(in TopoDS_Shape theShape)
    {
        using StringWriter aWriter = new();
        BRepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }

    private static double Area(in TopoDS_Shape theShape)
    {
        GProp_GProps aProps = new();
        BRepGProp.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static double Volume(in TopoDS_Shape theShape)
    {
        GProp_GProps aProps = new();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        int aNb = 0;
        for (TopExp_Explorer anExp = new(theShape, theType); anExp.More(); anExp.Next())
        {
            aNb++;
        }
        return aNb;
    }

    [Fact]
    public void Vertical_profile_on_the_spine_gives_the_walls_of_the_square()
    {
        TopoDS_Wire aSpine = Square();
        TopoDS_Wire aProfile = Polygon(false, 0, 0, 0, 0, 0, 5);
        string aSpineText = Text(aSpine), aProfileText = Text(aProfile);

        BRepOffsetAPI_MakeEvolved aMaker = new(aSpine, aProfile);
        Assert.True(aMaker.IsDone());
        TopoDS_Shape aResult = aMaker.Shape();
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.Equal(4, Count(aResult, TopAbs_ShapeEnum.TopAbs_FACE));
        Assert.Equal(200.0, Area(aResult), 1.0e-9);

        // each spine edge generates one wall from the profile's edge
        TopExp_Explorer aSpineEdges = new(aSpine, TopAbs_ShapeEnum.TopAbs_EDGE);
        TopExp_Explorer aProfileEdges = new(aProfile, TopAbs_ShapeEnum.TopAbs_EDGE);
        ISchematiqList<TopoDS_Shape> aGenerated = aMaker.GeneratedShapes(aSpineEdges.Current(), aProfileEdges.Current());
        Assert.Single(aGenerated);
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_FACE, aGenerated[0].ShapeType());
        Assert.True(aGenerated[0].TShape()!.IsFrozen);
        Assert.Empty(aMaker.GeneratedShapes(aProfileEdges.Current(), aSpineEdges.Current()));

        Assert.Equal(aSpineText, Text(aSpine));
        Assert.Equal(aProfileText, Text(aProfile));
    }

    [Fact]
    public void Solid_of_a_vertical_profile_has_its_top_and_bottom()
    {
        TopoDS_Wire aSpine = Square();
        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(aSpine, true).Face();
        TopoDS_Wire aProfile = Polygon(false, 0, 0, 0, 0, 0, 5);
        string aFaceText = Text(aFace);

        BRepOffsetAPI_MakeEvolved aMaker = new(aFace, aProfile, global::Kerf.Geometry.JoinKind.Arc, true, true);
        TopoDS_Shape aResult = aMaker.Shape();
        Assert.Equal(TopAbs_ShapeEnum.TopAbs_SOLID, aResult.ShapeType());
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.Equal(500.0, Volume(aResult), 1.0e-9);
        Assert.False(aMaker.Top().IsNull());
        Assert.False(aMaker.Bottom().IsNull());
        Assert.Equal(100.0, Area(aMaker.Top()), 1.0e-9);
        Assert.Equal(100.0, Area(aMaker.Bottom()), 1.0e-9);
        Assert.Equal(aFaceText, Text(aFace));
    }

    // Native OCCT V8_0_1's values (tools/evorec): validity, volume, bottom and top areas, number of faces. The L shape's
    // parallel at 2 inside is invalid natively too.
    [Theory]
    [InlineData("square", 2.0, true, 108.0, 36.0, 36.0, 6)]
    [InlineData("lshape", 2.0, false, 2.5752220392306189, 0.85840734641020666, 0.85840734641020666, 7)]
    [InlineData("square", -1.5, true, 501.20575041173106, 167.06858347057698, 167.06858347057698, 10)]
    [InlineData("lshape", -1.5, true, 391.75718801466383, 130.58572933822123, 130.58572933822123, 13)]
    public void Vertical_profile_off_the_spine_follows_a_parallel(string theSpine, double theOffset, bool theIsValid, double theVolume,
                                                                  double theBottom, double theTop, int theNbFaces)
    {
        TopoDS_Wire aSpine = theSpine == "square" ? Square() : LShape();
        TopoDS_Wire aProfile = Polygon(false, 0, theOffset, 0, 0, theOffset, 3);
        BRepOffsetAPI_MakeEvolved aMaker = new(aSpine, aProfile, global::Kerf.Geometry.JoinKind.Arc, true, true);
        Assert.True(aMaker.IsDone());
        TopoDS_Shape aResult = aMaker.Shape();
        Assert.Equal(theIsValid, new BRepCheck_Analyzer(aResult).IsValid());
        Assert.Equal(theVolume, Volume(aResult), 1.0e-12);
        Assert.Equal(theBottom, Area(aMaker.Bottom()), 1.0e-12);
        Assert.Equal(theTop, Area(aMaker.Top()), 1.0e-12);
        Assert.Equal(theNbFaces, Count(aResult, TopAbs_ShapeEnum.TopAbs_FACE));
    }

    // Native OCCT V8_0_1's values (tools/evorec, unit 5S2): profiles with slanted segments, which BRepFill_Evolved sweeps
    // with BRepFill_Pipe (BRepFill_Sweep): validity, area, volume of the solids, number of faces.
    [Theory]
    [InlineData("square", "slant", false, 50.911688245431428, 0.0, 4)]
    [InlineData("square", "slant", true, 214.91168824543141, 81.333333333333357, 6)]
    [InlineData("lshape", "slant", false, 50.608195417597926, 0.0, 7)]
    [InlineData("lshape", "slant", true, 142.82279725420048, 45.40486727886752, 9)]
    [InlineData("square", "zigzag", false, 122.50966799187806, 0.0, 12)]
    [InlineData("square", "zigzag", true, 258.50966799187802, 194.66666666666666, 14)]
    [InlineData("lshape", "zigzag", false, 112.38123708894452, 0.0, 21)]
    [InlineData("lshape", "zigzag", true, 177.23964443535272, 85.833479293348191, 23)]
    public void Slanted_profiles_are_swept_by_the_pipe(string theSpine, string theProfile, bool theIsSolid, double theArea,
                                                       double theVolume, int theNbFaces)
    {
        TopoDS_Wire aSpine = theSpine == "square" ? Square() : LShape();
        TopoDS_Wire aProfile = theProfile == "slant" ? Polygon(false, 0, 0, 0, 0, 1, 1)
                                                     : Polygon(false, 0, 0, 0, 0, 1, 1, 0, 1, 2, 0, 2, 3);
        string aSpineText = Text(aSpine), aProfileText = Text(aProfile);
        BRepOffsetAPI_MakeEvolved aMaker = new(aSpine, aProfile, global::Kerf.Geometry.JoinKind.Arc, true, theIsSolid);
        Assert.True(aMaker.IsDone());
        TopoDS_Shape aResult = aMaker.Shape();
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.Equal(theArea, Area(aResult), 1.0e-9);
        if (theIsSolid)
        {
            Assert.Equal(theVolume, Volume(aResult), 1.0e-9);
        }
        Assert.Equal(theNbFaces, Count(aResult, TopAbs_ShapeEnum.TopAbs_FACE));
        Assert.Equal(aSpineText, Text(aSpine));
        Assert.Equal(aProfileText, Text(aProfile));
    }

    [Fact]
    public void Horizontal_profile_gives_the_face_between_two_parallels()
    {
        TopoDS_Wire aSpine = Square();
        TopoDS_Wire aProfile = Polygon(false, 0, 1, 0, 0, 3, 0);
        BRepOffsetAPI_MakeEvolved aMaker = new(aSpine, aProfile);
        TopoDS_Shape aResult = aMaker.Shape();
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.True(Count(aResult, TopAbs_ShapeEnum.TopAbs_FACE) >= 1);
        // a ring between the parallels at distances 1 and 3 of the square, outside or inside it
        double anArea = Area(aResult);
        double anOutside = (100 + 4 * 10 * 3 + Math.PI * 9) - (100 + 4 * 10 * 1 + Math.PI * 1);
        double anInside = 8 * 8 - 4 * 4;
        Assert.True(Math.Abs(anArea - anOutside) < 1.0e-6 || Math.Abs(anArea - anInside) < 1.0e-6, $"{anArea}");
    }

    [Fact]
    public void Automatic_referential_places_the_profile_on_the_spine()
    {
        TopoDS_Wire aSpine = Square();
        // a vertical profile standing on the spine's edge y = 0, given in global coordinates
        TopoDS_Wire aProfile = Polygon(false, 5, 0, 0, 5, 0, 4);
        BRepOffsetAPI_MakeEvolved aMaker = new(aSpine, aProfile, global::Kerf.Geometry.JoinKind.Arc, false, false, true);
        Assert.True(aMaker.IsDone());
        Assert.Equal(160.0, Area(aMaker.Shape()), 1.0e-9);
    }

    [Fact]
    public void Volume_mode_builds_the_evolved_volume_and_keeps_the_inputs()
    {
        // A square spine and a vertical segment on its edge: the volume is the square's 10x10x1 slab (unit 7G).
        TopoDS_Wire aSpine = Square();
        TopoDS_Wire aProfile = Polygon(false, 5, 0, 0, 5, 0, 1);
        ShapeSnapshot aSpineSnap = ShapeSnapshot.Of(aSpine);
        ShapeSnapshot aProfileSnap = ShapeSnapshot.Of(aProfile);
        BRepOffsetAPI_MakeEvolved aMaker = new(aSpine, aProfile, global::Kerf.Geometry.JoinKind.Arc, true, true, false, 1e-7, true);
        aSpineSnap.AssertUnchanged(aSpine);
        aProfileSnap.AssertUnchanged(aProfile);
        Assert.True(aMaker.IsDone());
        TopoDS_Shape aResult = aMaker.Shape();
        Assert.False(aResult.IsNull());
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.Equal(100.0, Volume(aResult), 1.0e-6);
        Assert.Throws<TypeMismatchException>(() => aMaker.Evolved());
    }

    [Fact]
    public void Unported_modes_throw()
    {
        TopoDS_Wire aSpine = Square();
        // (a slanted profile, swept by BRepFill_Pipe, works since unit 5S2: Slanted_profiles_are_swept_by_the_pipe)
        // only the arc join is implemented, as in OCCT
        Assert.Throws<MethodNotImplementedException>(() => new BRepOffsetAPI_MakeEvolved(aSpine, Polygon(false, 0, 0, 0, 0, 0, 1),
                                                                                    global::Kerf.Geometry.JoinKind.Intersection));
        Assert.Throws<TypeMismatchException>(() => new BRepOffsetAPI_MakeEvolved(new BRepBuilderAPI_MakeVertex(new Point3D()).Shape(),
                                                                                  Polygon(false, 0, 0, 0, 0, 0, 1)));
    }

    [Fact]
    public void Repeated_runs_give_the_same_result()
    {
        string? aFirst = null;
        for (int i = 0; i < 3; i++)
        {
            BRepOffsetAPI_MakeEvolved aMaker = new(LShape(), Polygon(false, 0, 1, 0, 0, 1, 2, 0, 3, 2), global::Kerf.Geometry.JoinKind.Arc, true, true);
            string aText = Text(aMaker.Shape());
            aFirst ??= aText;
            Assert.Equal(aFirst, aText);
        }
    }
}
