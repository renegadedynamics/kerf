// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepBuilderAPI_Copy_Test.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepBuilderAPI_MakeEdge_Test.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepBuilderAPI_MakeFace_Test.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepBuilderAPI_MakeWire_Test.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepBuilderAPI_Transform_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// OCCT's BRepBuilderAPI_Copy GTests, on BRepPrimAPI_MakeBox's (frozen) box.
/// </summary>
public class BRepBuilderAPI_CopyTest
{
    private static Shape Box10() => new MakeBox(10.0, 10.0, 10.0).Shape();

    private static double Volume(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void CopyIsValid()
    {
        Shape aBox = Box10();

        var aCopier = new Copy(aBox);
        Shape aCopy = aCopier.Shape();
        Assert.False(aCopy.IsNull, "Copied shape is null");
        Assert.True(new Analyzer(aCopy).IsValid(), "aCopy is not valid");
    }

    [Fact]
    public void CopyVolume()
    {
        Shape aBox = Box10();
        double anOriginalVolume = Volume(aBox);

        var aCopier = new Copy(aBox);
        Shape aCopy = aCopier.Shape();
        Assert.False(aCopy.IsNull, "Copied shape is null");

        Assert.Equal(anOriginalVolume, Volume(aCopy), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void CopyIsDistinct()
    {
        Shape aBox = Box10();

        var aCopier = new Copy(aBox);
        Shape aCopy = aCopier.Shape();
        Assert.False(aCopy.IsNull, "Copied shape is null");

        Assert.False(aCopy.IsEqual(aBox), "Copy should not be equal to original (different TShape)");
    }

    [Fact]
    public void CopyGeomTrue()
    {
        Shape aBox = Box10();

        var aCopier = new Copy(aBox, true);
        Shape aCopy = aCopier.Shape();
        Assert.False(aCopy.IsNull, "Copied shape is null");

        Assert.Equal(Volume(aBox), Volume(aCopy), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void CopyGeomFalse()
    {
        Shape aBox = Box10();

        var aCopier = new Copy(aBox, false);
        Shape aCopy = aCopier.Shape();
        Assert.False(aCopy.IsNull, "Copied shape is null");
        Assert.True(new Analyzer(aCopy).IsValid(), "aCopy is not valid");
    }
}

/// <summary>OCCT's BRepBuilderAPI_MakeEdge GTests.</summary>
public class BRepBuilderAPI_MakeEdgeTest
{
    private static double Length(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.LinearProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void LinearEdge_TwoPoints()
    {
        var aP1 = new Point3D(0.0, 0.0, 0.0);
        var aP2 = new Point3D(10.0, 0.0, 0.0);

        var aMakeEdge = new MakeEdge(aP1, aP2);
        Assert.True(aMakeEdge.IsDone, "Linear edge creation failed");

        Edge anEdge = aMakeEdge.Edge();
        Assert.False(anEdge.IsNull, "Resulting edge is null");

        // Verify edge length
        Assert.Equal(10.0, Length(anEdge), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void CircularEdge_Full()
    {
        var anAxis = new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        var aCircle = new CircleCurve3D(anAxis, 5.0);

        var aMakeEdge = new MakeEdge(aCircle);
        Assert.True(aMakeEdge.IsDone, "Full circular edge creation failed");

        Edge anEdge = aMakeEdge.Edge();
        Assert.False(anEdge.IsNull);

        // Full circle length = 2*PI*R
        Assert.Equal(2.0 * Math.PI * 5.0, Length(anEdge), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void CircularEdge_Trimmed()
    {
        var anAxis = new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        var aCircle = new CircleCurve3D(anAxis, 5.0);

        // Half circle: 0 to PI
        var aMakeEdge = new MakeEdge(aCircle, 0.0, Math.PI);
        Assert.True(aMakeEdge.IsDone, "Trimmed circular edge creation failed");

        Edge anEdge = aMakeEdge.Edge();
        Assert.False(anEdge.IsNull);

        // Half circle length = PI*R
        Assert.Equal(Math.PI * 5.0, Length(anEdge), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void EdgeFromLine_WithBounds()
    {
        var aLine = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));

        var aMakeEdge = new MakeEdge(aLine, 0.0, 7.0);
        Assert.True(aMakeEdge.IsDone, "Edge from line with bounds failed");

        Edge anEdge = aMakeEdge.Edge();
        Assert.False(anEdge.IsNull);

        Assert.Equal(7.0, Length(anEdge), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void VertexExtraction()
    {
        var aP1 = new Point3D(0.0, 0.0, 0.0);
        var aP2 = new Point3D(10.0, 0.0, 0.0);

        var aMakeEdge = new MakeEdge(aP1, aP2);
        Assert.True(aMakeEdge.IsDone);

        Edge anEdge = aMakeEdge.Edge();
        Exploration.Vertices(anEdge, out Vertex aV1, out Vertex aV2);

        Assert.False(aV1.IsNull, "First vertex is null");
        Assert.False(aV2.IsNull, "Last vertex is null");

        Point3D aFirstPnt = Tool.Pnt(aV1);
        Point3D aLastPnt = Tool.Pnt(aV2);

        Assert.True(aFirstPnt.IsEqual(aP1, global::Kerf.Engine.Foundation.Precision.Confusion()));
        Assert.True(aLastPnt.IsEqual(aP2, global::Kerf.Engine.Foundation.Precision.Confusion()));
    }

    [Fact]
    public void ToleranceCheck()
    {
        var aP1 = new Point3D(0.0, 0.0, 0.0);
        var aP2 = new Point3D(10.0, 0.0, 0.0);

        var aMakeEdge = new MakeEdge(aP1, aP2);
        Assert.True(aMakeEdge.IsDone);

        Edge anEdge = aMakeEdge.Edge();
        double aTol = Tool.Tolerance(anEdge);
        Assert.True(aTol > 0.0, "Edge tolerance should be positive");
        Assert.True(aTol <= global::Kerf.Engine.Foundation.Precision.Confusion(), "Edge tolerance should not exceed confusion");
    }
}

/// <summary>
/// OCCT's BRepBuilderAPI_MakeFace GTests.
/// </summary>
public class BRepBuilderAPI_MakeFaceTest
{
    private static double Area(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void FaceFromPlane()
    {
        var aPln = new Plane3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        var aMakeFace = new MakeFace(aPln);
        Assert.True(aMakeFace.IsDone, "Face from plane creation failed");

        Face aFace = aMakeFace.Face();
        Assert.False(aFace.IsNull, "Resulting face is null");
    }

    [Fact]
    public void FaceFromWire()
    {
        // Create a rectangular wire in XY plane
        var aP1 = new Point3D(0.0, 0.0, 0.0);
        var aP2 = new Point3D(10.0, 0.0, 0.0);
        var aP3 = new Point3D(10.0, 5.0, 0.0);
        var aP4 = new Point3D(0.0, 5.0, 0.0);

        Edge anE1 = new MakeEdge(aP1, aP2).Edge();
        Edge anE2 = new MakeEdge(aP2, aP3).Edge();
        Edge anE3 = new MakeEdge(aP3, aP4).Edge();
        Edge anE4 = new MakeEdge(aP4, aP1).Edge();

        var aMakeWire = new MakeWire();
        aMakeWire.Add(anE1);
        aMakeWire.Add(anE2);
        aMakeWire.Add(anE3);
        aMakeWire.Add(anE4);
        Assert.True(aMakeWire.IsDone, "Wire creation failed");

        var aMakeFace = new MakeFace(aMakeWire.Wire());
        Assert.True(aMakeFace.IsDone, "Face from wire creation failed");

        Face aFace = aMakeFace.Face();
        Assert.False(aFace.IsNull);
        Assert.True(new Analyzer(aFace).IsValid(), "aFace is not valid");

        // Area should be 10 * 5 = 50
        Assert.Equal(50.0, Area(aFace), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FaceFromGeomPlane_WithBounds()
    {
        var aGeomPlane = new PlaneSurface(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        var aMakeFace = new MakeFace(aGeomPlane, 0.0, 10.0, 0.0, 5.0, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aMakeFace.IsDone, "Face from Geom_Plane with bounds failed");

        Face aFace = aMakeFace.Face();
        Assert.False(aFace.IsNull);
        Assert.True(new Analyzer(aFace).IsValid(), "aFace is not valid");

        Assert.Equal(50.0, Area(aFace), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FaceFromCylindricalSurface()
    {
        var anAxis = new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        var aCylSurf = new CylindricalSurface(anAxis, 5.0);

        // Create a bounded face: U from 0 to 2*PI, V from 0 to 10
        var aMakeFace = new MakeFace(aCylSurf, 0.0, 2.0 * Math.PI, 0.0, 10.0, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aMakeFace.IsDone, "Face from cylindrical surface failed");

        Face aFace = aMakeFace.Face();
        Assert.False(aFace.IsNull);
        Assert.True(new Analyzer(aFace).IsValid(), "aFace is not valid");

        // Area of cylinder: 2*PI*R*H = 2*PI*5*10 = 100*PI
        Assert.Equal(2.0 * Math.PI * 5.0 * 10.0, Area(aFace), 0.01);
    }
}

/// <summary>OCCT's BRepBuilderAPI_MakeWire GTest.</summary>
public class BRepBuilderAPI_MakeWire_Test
{
    // Bug OCC27552: Wire creation fails depending on the order of edges
    // This test verifies that BRepBuilderAPI_MakeWire can successfully add
    // edges individually and as a list
    [Fact]
    public void OCC27552_AddEdgesAndListOfEdges()
    {
        var aBB = new BrepBuilder();
        Vertex aV1 = new(), aV2 = new(), aV3 = new();

        // Create vertices
        aBB.MakeVertex(ref aV1, new Point3D(0, 0, 0), 0.1);
        aBB.MakeVertex(ref aV2, new Point3D(5, 0, 0), 0.1);
        aBB.MakeVertex(ref aV3, new Point3D(10, 0, 0), 0.1);

        // Create edges
        Edge anE1 = new MakeEdge(aV1, aV2).Edge();
        Edge anE2 = new MakeEdge(aV2, aV3).Edge();

        // Build wire with individually added edges
        var aMW = new MakeWire();
        aMW.Add(anE1);
        aMW.Add(anE2);

        // Create additional vertices and edges for list test
        Vertex aV4 = new(), aV5 = new(), aV6 = new(), aV7 = new();
        aBB.MakeVertex(ref aV4, new Point3D(10, 0.05, 0), 0.07);
        aBB.MakeVertex(ref aV5, new Point3D(10, -0.05, 0), 0.07);
        aBB.MakeVertex(ref aV6, new Point3D(10, 2, 0), 0.07);
        aBB.MakeVertex(ref aV7, new Point3D(10, -2, 0), 0.07);

        Edge anE3 = new MakeEdge(aV4, aV6).Edge();
        Edge anE4 = new MakeEdge(aV5, aV7).Edge();

        // Add edges as a list
        var aListOfEdges = new NCollection_List<Shape>();
        aListOfEdges.Append(anE3);
        aListOfEdges.Append(anE4);

        aMW.Add(aListOfEdges);

        // Verify wire was created successfully
        Assert.True(aMW.IsDone, "Wire builder should complete successfully");
        Wire aWire = aMW.Wire();
        Assert.False(aWire.IsNull, "Resulting wire should not be null");
    }
}

/// <summary>
/// OCCT's BRepBuilderAPI_Transform GTests, on BRepPrimAPI_MakeBox's (frozen) boxes.
/// </summary>
public class BRepBuilderAPI_TransformTest
{
    private static Shape Box10() => new MakeBox(10.0, 10.0, 10.0).Shape();

    private static Properties VolumeProps(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps;
    }

    [Fact]
    public void Translate()
    {
        Shape aBox = Box10();

        var aTrsf = new Transform3D();
        aTrsf.SetTranslation(new Vector3D(100.0, 0.0, 0.0));

        var aTransform = new Transform(aBox, aTrsf);
        Shape aResult = aTransform.Shape();
        Assert.False(aResult.IsNull, "Transformed shape is null");

        Point3D aCenterOfMass = VolumeProps(aResult).CentreOfMass();

        Assert.Equal(105.0, aCenterOfMass.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aCenterOfMass.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aCenterOfMass.Z(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        Shape aBox = Box10();

        var aTrsf = new Transform3D();
        var aZAxis = new Axis3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        aTrsf.SetRotation(aZAxis, Math.PI / 2.0);

        var aTransform = new Transform(aBox, aTrsf);
        Shape aResult = aTransform.Shape();
        Assert.False(aResult.IsNull, "Transformed shape is null");

        Point3D aCenterOfMass = VolumeProps(aResult).CentreOfMass();

        Assert.Equal(-5.0, aCenterOfMass.X(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(5.0, aCenterOfMass.Y(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Scale()
    {
        Shape aBox = Box10();

        var aTrsf = new Transform3D();
        aTrsf.SetScale(new Point3D(0.0, 0.0, 0.0), 2.0);

        var aTransform = new Transform(aBox, aTrsf);
        Shape aResult = aTransform.Shape();
        Assert.False(aResult.IsNull, "Transformed shape is null");

        Assert.Equal(8000.0, VolumeProps(aResult).Mass(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void Mirror()
    {
        Shape aBox = new MakeBox(new Point3D(10.0, 0.0, 0.0), 10.0, 10.0, 10.0).Shape();

        var aTrsf = new Transform3D();
        var aYZPlane = new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        aTrsf.SetMirror(aYZPlane);

        var aTransform = new Transform(aBox, aTrsf);
        Shape aResult = aTransform.Shape();
        Assert.False(aResult.IsNull, "Transformed shape is null");

        Point3D aCenterOfMass = VolumeProps(aResult).CentreOfMass();

        Assert.True(aCenterOfMass.X() < 0.0, "Center of mass X should be negative after mirroring through YZ plane");
    }

    [Fact]
    public void ShapeValidity()
    {
        Shape aBox = Box10();

        var aTrsf = new Transform3D();
        aTrsf.SetTranslation(new Vector3D(50.0, 50.0, 50.0));

        var aTransform = new Transform(aBox, aTrsf);
        Shape aResult = aTransform.Shape();
        Assert.False(aResult.IsNull, "Transformed shape is null");
        Assert.True(new Analyzer(aResult).IsValid(), "aResult is not valid");
    }
}
