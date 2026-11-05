// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Measurement;
using Kerf.Engine.Offsets;
using Kerf.Engine.Offsets.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Offsets.Error;
using static Kerf.Offsets.OffsetMode;
using static Kerf.Geometry.JoinKind;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BRepOffset_MakeOffset (unit 7F) on frozen inputs: the caller's shape is copied at the entry (decision 1005), never
/// written; the results are frozen and hold the caller's sub-shapes where nothing changed.
/// </summary>
public class BRepOffset_MakeOffsetTests
{
    internal static List<Shape> SubShapes(in Shape theShape, ShapeType theType)
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

    internal static bool AllFrozen(in Shape theShape)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, aMap);
        return aMap.All(s => s.TShape()!.IsFrozen);
    }

    internal static double Volume(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void Arc_OfAFrozenBox_IsTheRoundedBox_TheInputIsUnchanged()
    {
        Shape aBox = new MakeBox(10, 20, 30).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var anAlgo = new MakeOffset();
        anAlgo.Initialize(aBox, 1.0, 1.0e-7, global::Kerf.Offsets.OffsetMode.Skin, false, false, Arc);
        anAlgo.MakeOffsetShape();

        aSnap.AssertUnchanged(aBox);
        Assert.True(anAlgo.IsDone, anAlgo.Error().ToString());
        Shape aResult = anAlgo.Shape();
        Assert.True(AllFrozen(aResult));
        Assert.Equal(ShapeType.Solid, aResult.Type);
        Assert.Equal(26, SubShapes(aResult, ShapeType.Face).Count);
        double a = 10, b = 20, c = 30, r = 1;
        double anExpected = a * b * c + 2 * (a * b + b * c + c * a) * r + Math.PI * r * r * (a + b + c)
                            + 4.0 / 3.0 * Math.PI * r * r * r;
        Assert.Equal(anExpected, Volume(aResult), 3);
        Assert.True(new Analyzer(aResult).IsValid());
    }

    [Fact]
    public void Intersection_OfAFrozenBox_IsTheLargerBox()
    {
        Shape aBox = new MakeBox(10, 20, 30).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var anAlgo = new MakeOffset();
        anAlgo.Initialize(aBox, 1.0, 1.0e-7, global::Kerf.Offsets.OffsetMode.Skin, false, false, Intersection);
        anAlgo.MakeOffsetShape();

        aSnap.AssertUnchanged(aBox);
        Assert.True(anAlgo.IsDone, anAlgo.Error().ToString());
        Shape aResult = anAlgo.Shape();
        Assert.Equal(6, SubShapes(aResult, ShapeType.Face).Count);
        Assert.Equal(12.0 * 22.0 * 32.0, Volume(aResult), 6);
    }

    [Fact]
    public void ThickSolid_OfAFrozenBox_KeepsTheCallersFaces()
    {
        Shape aBox = new MakeBox(10, 20, 30).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        Shape aTop = SubShapes(aBox, ShapeType.Face)[5];

        var anAlgo = new MakeThickSolid();
        var aClosing = new NCollection_List<Shape>();
        aClosing.Append(aTop);
        anAlgo.MakeThickSolidByJoin(aBox, aClosing, -1.0, 1.0e-3, global::Kerf.Offsets.OffsetMode.Skin, false, false, Arc);

        aSnap.AssertUnchanged(aBox);
        Assert.True(anAlgo.IsDone);
        Shape aResult = anAlgo.Shape();
        Assert.True(AllFrozen(aResult));
        Assert.Equal(ShapeType.Solid, aResult.Type);
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.Equal(10.0 * 20.0 * 30.0 - 8.0 * 18.0 * 29.0, Volume(aResult), 3);

        // the result holds the caller's own faces where the algorithm left them unchanged: the bottom face; the four
        // side faces share edges with the closing face, whose edges get new vertices (changed copies)
        List<Shape> aResultFaces = SubShapes(aResult, ShapeType.Face);
        int aKept = SubShapes(aBox, ShapeType.Face).Count(f => aResultFaces.Any(r => r.IsSame(f)));
        Assert.Equal(1, aKept);
        // the history answers for the caller's sub-shapes, with frozen shapes of the result
        Assert.True(anAlgo.IsDeleted(aTop) || anAlgo.Modified(aTop).Count > 0);
        foreach (Shape aF in SubShapes(aBox, ShapeType.Face))
        {
            foreach (Shape aG in anAlgo.Generated(aF).Concat(anAlgo.Modified(aF)))
            {
                Assert.True(aG.TShape()!.IsFrozen);
                Assert.Contains(aResultFaces, r => r.IsSame(aG));
            }
        }
    }

    [Fact]
    public void Thicken_AFrozenFace_KeepsTheCallersFace_AndItsHistory()
    {
        Shape aBox = new MakeBox(10, 20, 30).Shape();
        Shape aFace = SubShapes(aBox, ShapeType.Face)[0];
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);

        var anAlgo = new MakeOffset();
        anAlgo.Initialize(aFace, 2.0, 1.0e-5, global::Kerf.Offsets.OffsetMode.Skin, true, false, Intersection, true, true);
        anAlgo.MakeOffsetShape();

        aSnap.AssertUnchanged(aBox);
        Assert.True(anAlgo.IsDone, anAlgo.Error().ToString());
        Shape aResult = anAlgo.Shape();
        Assert.True(AllFrozen(aResult));
        Assert.Equal(ShapeType.Solid, aResult.Type);
        var anArea = new Properties();
        MassPropertiesBuilder.SurfaceProperties(aFace, anArea);
        Assert.Equal(anArea.Mass() * 2.0, Math.Abs(Volume(aResult)), 6);
        Assert.False(anAlgo.IsDeleted(aFace));
        Assert.Single(anAlgo.Generated(aFace));
        // the thickened face's edges get pcurves on the walls: changed copies, so the caller's edges are not in the
        // result, but they are not deleted either (OCCT keeps its own, changed in place)
        foreach (Shape anE in SubShapes(aFace, ShapeType.Edge))
        {
            Assert.False(anAlgo.IsDeleted(anE));
        }
    }

    [Fact]
    public void Initialize_ReusesTheAlgorithm_OnFrozenInputs()
    {
        Shape aBox = new MakeBox(10, 20, 30).Shape();
        Shape aCyl = new ApiMakeCylinder(5, 10).Shape();
        var anAlgo = new MakeOffset();
        anAlgo.Initialize(aBox, 1.0, 1.0e-7, global::Kerf.Offsets.OffsetMode.Skin, false, false, Arc);
        anAlgo.MakeOffsetShape();
        Shape aFirst = anAlgo.Shape();
        anAlgo.Initialize(aCyl, -1.0, 1.0e-7, global::Kerf.Offsets.OffsetMode.Skin, false, false, Arc);
        anAlgo.MakeOffsetShape();
        Assert.True(anAlgo.IsDone);
        Assert.Equal(Math.PI * 16 * 8, Volume(anAlgo.Shape()), 4);
        Assert.Equal(26, SubShapes(aFirst, ShapeType.Face).Count);
        Assert.True(anAlgo.InitShape().IsSame(aCyl));
    }

    [Fact]
    public void ClosingFaces_AndImages_AnswerForTheCallersShapes()
    {
        Shape aBox = new MakeBox(10, 20, 30).Shape();
        Face aTop = ShapeCasts.Face(SubShapes(aBox, ShapeType.Face)[5]);
        var anAlgo = new MakeOffset();
        anAlgo.Initialize(aBox, -1.0, 1.0e-7, global::Kerf.Offsets.OffsetMode.Skin, false, false, Arc);
        anAlgo.AddFace(aTop);
        anAlgo.MakeThickSolid();
        Assert.True(anAlgo.IsDone);
        Assert.True(anAlgo.ClosingFaces().Contains(aTop));
        ShapeImage anImage = anAlgo.OffsetFacesFromShapes();
        foreach (Shape aF in SubShapes(aBox, ShapeType.Face))
        {
            Assert.True(anImage.HasImage(aF));
            foreach (Shape anIm in anImage.Image(aF))
            {
                Assert.True(anIm.TShape()!.IsFrozen);
            }
        }
        Assert.Equal(Arc, anAlgo.GetJoinType());
        Assert.Equal(Error.NoError, anAlgo.Error());
    }

    [Fact]
    public void CheckInputData_ReportsANullOffset()
    {
        Shape aBox = new MakeBox(10, 20, 30).Shape();
        var anAlgo = new MakeOffset();
        anAlgo.Initialize(aBox, 0.0, 1.0e-7);
        anAlgo.MakeOffsetShape();
        Assert.False(anAlgo.IsDone);
        Assert.Equal(Error.NullOffset, anAlgo.Error());
        Assert.True(anAlgo.Shape().IsNull);
    }

    [Theory]
    [InlineData(2.0)]
    [InlineData(-1.0)]
    public void CompleteIntersection_OfAnLShape_IsValid(double theOffset)
    {
        Shape aBox1 = new MakeBox(10, 10, 10).Shape();
        Shape aBox2 = new MakeBox(new Point3D(0, 0, 0), 20, 5, 5).Shape();
        Shape aL = new Fuse(aBox1, aBox2).Shape();
        var aL2 = new UnifySameDomain(aL);
        aL2.Build();
        Shape anL = aL2.Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(anL);

        var anAlgo = new MakeOffset();
        anAlgo.Initialize(anL, theOffset, 1.0e-7, global::Kerf.Offsets.OffsetMode.Skin, true, false, Intersection);
        anAlgo.MakeOffsetShape();

        aSnap.AssertUnchanged(anL);
        Assert.True(anAlgo.IsDone, anAlgo.Error().ToString());
        Shape aResult = anAlgo.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.True(Volume(aResult) > 0);
    }
    // bugs/modalg_2/bug5805_3 and _4: OCC5805.brep's thick solid with a closing face and offset -1. The offset faces are
    // extended to the closing face: OCCT enlarges their offset surfaces in place (decision 1101), and their lateral
    // edges then extend to the new boundary. Native's result is one shell of seven faces (OCCT's script checks it).
    [Fact]
    public void ThickSolid_OfOCC5805_IsOneShellOfSevenFaces_AsNative()
    {
        string aRoot = System.Reflection.CustomAttributeExtensions.GetCustomAttributes<System.Reflection.AssemblyMetadataAttribute>(typeof(BRepOffset_MakeOffsetTests).Assembly)
                                                         .Single(a => a.Key == "OcctTestDataDir").Value!;
        string? aFile = string.IsNullOrEmpty(aRoot) || !Directory.Exists(aRoot) ? null
            : Directory.EnumerateFiles(aRoot, "OCC5805.brep", SearchOption.AllDirectories).FirstOrDefault();
        if (aFile is null)
        {
            return; // the data isn't available
        }
        Assert.True(BrepTools.Read(out Shape aShape, aFile));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);
        var aClosing = new NCollection_List<Shape>();
        aClosing.Append(SubShapes(aShape, ShapeType.Face)[5]); // a_6
        var anAlgo = new MakeThickSolid();
        anAlgo.MakeThickSolidByJoin(aShape, aClosing, -1.0, 1.0e-7);
        aSnap.AssertUnchanged(aShape);
        Assert.True(anAlgo.IsDone);
        Shape aResult = anAlgo.Shape();
        Assert.True(AllFrozen(aResult));
        Assert.Equal(7, SubShapes(aResult, ShapeType.Face).Count);
        Assert.Single(SubShapes(aResult, ShapeType.Shell));
        Assert.Equal(15, SubShapes(aResult, ShapeType.Edge).Count);
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(aResult, aProps);
        Assert.Equal(1185.03, aProps.Mass(), 2);
    }
}
