// Port of: src/ModelingAlgorithms/TKShHealing/GTests/ShapeFix_Shape_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.ShapeExtend_Status;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>OCCT's GTests of ShapeFix_Shape.</summary>
public class ShapeFix_ShapeTest
{
    [Fact]
    public void FixValidBox()
    {
        var aMakeBox = new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0);
        Shape aBox = aMakeBox.Shape();
        Assert.True(aMakeBox.IsDone());

        var aFixer = new ShapeFix_Shape(aBox);
        aFixer.Perform();
        Shape aResult = aFixer.Shape();
        Assert.False(aResult.IsNull);

        var anAnalyzer = new BRepCheck_Analyzer(aResult);
        Assert.True(anAnalyzer.IsValid());
    }

    [Fact]
    public void StatusAfterFix()
    {
        var aMakeBox = new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0);
        Shape aBox = aMakeBox.Shape();
        Assert.True(aMakeBox.IsDone());

        var aFixer = new ShapeFix_Shape(aBox);
        aFixer.Perform();

        // A valid box should not require any failure-level fixes
        Assert.False(aFixer.Status(ShapeExtend_FAIL));
    }

    [Fact]
    public void SetPrecision()
    {
        var aFixer = new ShapeFix_Shape();

        const double aPrecision    = 0.01;
        const double aMinTolerance = 0.001;
        const double aMaxTolerance = 0.1;

        aFixer.SetPrecision(aPrecision);
        aFixer.SetMinTolerance(aMinTolerance);
        aFixer.SetMaxTolerance(aMaxTolerance);

        Assert.Equal(aPrecision, aFixer.Precision(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aMinTolerance, aFixer.MinTolerance(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aMaxTolerance, aFixer.MaxTolerance(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FixCompound()
    {
        var aMakeBox1 = new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0);
        Shape aBox1 = aMakeBox1.Shape();
        Assert.True(aMakeBox1.IsDone());

        var aMakeBox2 = new BRepPrimAPI_MakeBox(new Point3D(20.0, 0.0, 0.0), 5.0, 5.0, 5.0);
        Shape aBox2 = aMakeBox2.Shape();
        Assert.True(aMakeBox2.IsDone());

        var aBuilder = new BrepBuilder();
        var aCompound = new Compound();
        aBuilder.MakeCompound(ref aCompound);
        aBuilder.Add(aCompound, aBox1);
        aBuilder.Add(aCompound, aBox2);

        var aFixer = new ShapeFix_Shape(aCompound);
        aFixer.Perform();
        Shape aResult = aFixer.Shape();
        Assert.False(aResult.IsNull);

        var anAnalyzer = new BRepCheck_Analyzer(aResult);
        Assert.True(anAnalyzer.IsValid());
    }

    // Healing a prism built from a self-intersecting ("bowtie") face must not crash:
    // the cap is split into a compound and ShapeFix_Face must not cast it to a face.
    [Fact]
    public void HealPrismFromSelfIntersectingFace()
    {
        // Bowtie quad: vertices ordered so the two diagonals cross -> the wire self-intersects.
        var aPoly = new BRepBuilderAPI_MakePolygon();
        aPoly.Add(new Point3D(0.0, 0.0, 0.0));
        aPoly.Add(new Point3D(1.0, 1.0, 0.0));
        aPoly.Add(new Point3D(1.0, 0.0, 0.0));
        aPoly.Add(new Point3D(0.0, 1.0, 0.0));
        aPoly.Close();
        Assert.True(aPoly.IsDone());

        var aMakeFace = new BRepBuilderAPI_MakeFace(aPoly.Wire(), true);
        Assert.True(aMakeFace.IsDone());

        var aMakePrism = new BRepPrimAPI_MakePrism(aMakeFace.Face(), new Vector3D(0.0, 0.0, 1.0));
        Assert.True(aMakePrism.IsDone());

        var aFixer = new ShapeFix_Shape(aMakePrism.Shape());
        aFixer.Perform();
        Assert.False(aFixer.Shape().IsNull);

        var anAnalyzer = new BRepCheck_Analyzer(aFixer.Shape());
        Assert.True(anAnalyzer.IsValid());
    }
}
