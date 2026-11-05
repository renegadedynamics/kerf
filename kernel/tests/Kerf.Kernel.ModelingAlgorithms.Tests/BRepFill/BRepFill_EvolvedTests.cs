// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

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
    private static Wire Polygon(bool theClosed, params double[] theXYZ)
    {
        MakePolygon aMP = new();
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

    private static Wire Square() => Polygon(true, 0, 0, 0, 10, 0, 0, 10, 10, 0, 0, 10, 0);

    private static Wire LShape() => Polygon(true, 0, 0, 0, 10, 0, 0, 10, 4, 0, 4, 4, 0, 4, 10, 0, 0, 10, 0);

    private static string Text(in Shape theShape)
    {
        using StringWriter aWriter = new();
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }

    private static double Area(in Shape theShape)
    {
        Properties aProps = new();
        MassPropertiesBuilder.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static double Volume(in Shape theShape)
    {
        Properties aProps = new();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static int Count(in Shape theShape, ShapeType theType)
    {
        int aNb = 0;
        for (Explorer anExp = new(theShape, theType); anExp.More(); anExp.Next())
        {
            aNb++;
        }
        return aNb;
    }

    [Fact]
    public void Vertical_profile_on_the_spine_gives_the_walls_of_the_square()
    {
        Wire aSpine = Square();
        Wire aProfile = Polygon(false, 0, 0, 0, 0, 0, 5);
        string aSpineText = Text(aSpine), aProfileText = Text(aProfile);

        MakeEvolved aMaker = new(aSpine, aProfile);
        Assert.True(aMaker.IsDone);
        Shape aResult = aMaker.Shape();
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.Equal(4, Count(aResult, ShapeType.Face));
        Assert.Equal(200.0, Area(aResult), 1.0e-9);

        // each spine edge generates one wall from the profile's edge
        Explorer aSpineEdges = new(aSpine, ShapeType.Edge);
        Explorer aProfileEdges = new(aProfile, ShapeType.Edge);
        IReadOnlyList<Shape> aGenerated = aMaker.GeneratedShapes(aSpineEdges.Current(), aProfileEdges.Current());
        Assert.Single(aGenerated);
        Assert.Equal(ShapeType.Face, aGenerated[0].Type);
        Assert.True(aGenerated[0].TShape()!.IsFrozen);
        Assert.Empty(aMaker.GeneratedShapes(aProfileEdges.Current(), aSpineEdges.Current()));

        Assert.Equal(aSpineText, Text(aSpine));
        Assert.Equal(aProfileText, Text(aProfile));
    }

    [Fact]
    public void Solid_of_a_vertical_profile_has_its_top_and_bottom()
    {
        Wire aSpine = Square();
        Face aFace = new MakeFace(aSpine, true).Face();
        Wire aProfile = Polygon(false, 0, 0, 0, 0, 0, 5);
        string aFaceText = Text(aFace);

        MakeEvolved aMaker = new(aFace, aProfile, global::Kerf.Geometry.JoinKind.Arc, true, true);
        Shape aResult = aMaker.Shape();
        Assert.Equal(ShapeType.Solid, aResult.Type);
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.Equal(500.0, Volume(aResult), 1.0e-9);
        Assert.False(aMaker.Top().IsNull);
        Assert.False(aMaker.Bottom().IsNull);
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
        Wire aSpine = theSpine == "square" ? Square() : LShape();
        Wire aProfile = Polygon(false, 0, theOffset, 0, 0, theOffset, 3);
        MakeEvolved aMaker = new(aSpine, aProfile, global::Kerf.Geometry.JoinKind.Arc, true, true);
        Assert.True(aMaker.IsDone);
        Shape aResult = aMaker.Shape();
        Assert.Equal(theIsValid, new Analyzer(aResult).IsValid());
        Assert.Equal(theVolume, Volume(aResult), 1.0e-12);
        Assert.Equal(theBottom, Area(aMaker.Bottom()), 1.0e-12);
        Assert.Equal(theTop, Area(aMaker.Top()), 1.0e-12);
        Assert.Equal(theNbFaces, Count(aResult, ShapeType.Face));
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
        Wire aSpine = theSpine == "square" ? Square() : LShape();
        Wire aProfile = theProfile == "slant" ? Polygon(false, 0, 0, 0, 0, 1, 1)
                                                     : Polygon(false, 0, 0, 0, 0, 1, 1, 0, 1, 2, 0, 2, 3);
        string aSpineText = Text(aSpine), aProfileText = Text(aProfile);
        MakeEvolved aMaker = new(aSpine, aProfile, global::Kerf.Geometry.JoinKind.Arc, true, theIsSolid);
        Assert.True(aMaker.IsDone);
        Shape aResult = aMaker.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.Equal(theArea, Area(aResult), 1.0e-9);
        if (theIsSolid)
        {
            Assert.Equal(theVolume, Volume(aResult), 1.0e-9);
        }
        Assert.Equal(theNbFaces, Count(aResult, ShapeType.Face));
        Assert.Equal(aSpineText, Text(aSpine));
        Assert.Equal(aProfileText, Text(aProfile));
    }

    [Fact]
    public void Horizontal_profile_gives_the_face_between_two_parallels()
    {
        Wire aSpine = Square();
        Wire aProfile = Polygon(false, 0, 1, 0, 0, 3, 0);
        MakeEvolved aMaker = new(aSpine, aProfile);
        Shape aResult = aMaker.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.True(Count(aResult, ShapeType.Face) >= 1);
        // a ring between the parallels at distances 1 and 3 of the square, outside or inside it
        double anArea = Area(aResult);
        double anOutside = (100 + 4 * 10 * 3 + Math.PI * 9) - (100 + 4 * 10 * 1 + Math.PI * 1);
        double anInside = 8 * 8 - 4 * 4;
        Assert.True(Math.Abs(anArea - anOutside) < 1.0e-6 || Math.Abs(anArea - anInside) < 1.0e-6, $"{anArea}");
    }

    [Fact]
    public void Automatic_referential_places_the_profile_on_the_spine()
    {
        Wire aSpine = Square();
        // a vertical profile standing on the spine's edge y = 0, given in global coordinates
        Wire aProfile = Polygon(false, 5, 0, 0, 5, 0, 4);
        MakeEvolved aMaker = new(aSpine, aProfile, global::Kerf.Geometry.JoinKind.Arc, false, false, true);
        Assert.True(aMaker.IsDone);
        Assert.Equal(160.0, Area(aMaker.Shape()), 1.0e-9);
    }

    [Fact]
    public void Volume_mode_builds_the_evolved_volume_and_keeps_the_inputs()
    {
        // A square spine and a vertical segment on its edge: the volume is the square's 10x10x1 slab (unit 7G).
        Wire aSpine = Square();
        Wire aProfile = Polygon(false, 5, 0, 0, 5, 0, 1);
        ShapeSnapshot aSpineSnap = ShapeSnapshot.Of(aSpine);
        ShapeSnapshot aProfileSnap = ShapeSnapshot.Of(aProfile);
        MakeEvolved aMaker = new(aSpine, aProfile, global::Kerf.Geometry.JoinKind.Arc, true, true, false, 1e-7, true);
        aSpineSnap.AssertUnchanged(aSpine);
        aProfileSnap.AssertUnchanged(aProfile);
        Assert.True(aMaker.IsDone);
        Shape aResult = aMaker.Shape();
        Assert.False(aResult.IsNull);
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.Equal(100.0, Volume(aResult), 1.0e-6);
        EngineAssert.Throws<TypeMismatchException>(() => aMaker.Evolved());
    }

    [Fact]
    public void Unported_modes_throw()
    {
        Wire aSpine = Square();
        // (a slanted profile, swept by BRepFill_Pipe, works since unit 5S2: Slanted_profiles_are_swept_by_the_pipe)
        // only the arc join is implemented, as in OCCT
        EngineAssert.Throws<MethodNotImplementedException>(() => new MakeEvolved(aSpine, Polygon(false, 0, 0, 0, 0, 0, 1),
                                                                                    global::Kerf.Geometry.JoinKind.Intersection));
        EngineAssert.Throws<TypeMismatchException>(() => new MakeEvolved(new MakeVertex(new Point3D()).Shape(),
                                                                                  Polygon(false, 0, 0, 0, 0, 0, 1)));
    }

    [Fact]
    public void Repeated_runs_give_the_same_result()
    {
        string? aFirst = null;
        for (int i = 0; i < 3; i++)
        {
            MakeEvolved aMaker = new(LShape(), Polygon(false, 0, 1, 0, 0, 1, 2, 0, 3, 2), global::Kerf.Geometry.JoinKind.Arc, true, true);
            string aText = Text(aMaker.Shape());
            aFirst ??= aText;
            Assert.Equal(aFirst, aText);
        }
    }
}
