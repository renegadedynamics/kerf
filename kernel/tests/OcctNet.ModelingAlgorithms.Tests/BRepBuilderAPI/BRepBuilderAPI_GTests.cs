// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepBuilderAPI_Copy_Test.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepBuilderAPI_MakeEdge_Test.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepBuilderAPI_MakeFace_Test.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepBuilderAPI_MakeWire_Test.cxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepBuilderAPI_Transform_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// OCCT's BRepBuilderAPI_Copy GTests, on BRepPrimAPI_MakeBox's (frozen) box.
/// </summary>
public class BRepBuilderAPI_CopyTest
{
    private static TopoDS_Shape Box10() => new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    private static double Volume(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void CopyIsValid()
    {
        TopoDS_Shape aBox = Box10();

        var aCopier = new BRepBuilderAPI_Copy(aBox);
        TopoDS_Shape aCopy = aCopier.Shape();
        Assert.False(aCopy.IsNull(), "Copied shape is null");
        Assert.True(new BRepCheck_Analyzer(aCopy).IsValid(), "aCopy is not valid");
    }

    [Fact]
    public void CopyVolume()
    {
        TopoDS_Shape aBox = Box10();
        double anOriginalVolume = Volume(aBox);

        var aCopier = new BRepBuilderAPI_Copy(aBox);
        TopoDS_Shape aCopy = aCopier.Shape();
        Assert.False(aCopy.IsNull(), "Copied shape is null");

        Assert.Equal(anOriginalVolume, Volume(aCopy), Precision.Confusion());
    }

    [Fact]
    public void CopyIsDistinct()
    {
        TopoDS_Shape aBox = Box10();

        var aCopier = new BRepBuilderAPI_Copy(aBox);
        TopoDS_Shape aCopy = aCopier.Shape();
        Assert.False(aCopy.IsNull(), "Copied shape is null");

        Assert.False(aCopy.IsEqual(aBox), "Copy should not be equal to original (different TShape)");
    }

    [Fact]
    public void CopyGeomTrue()
    {
        TopoDS_Shape aBox = Box10();

        var aCopier = new BRepBuilderAPI_Copy(aBox, true);
        TopoDS_Shape aCopy = aCopier.Shape();
        Assert.False(aCopy.IsNull(), "Copied shape is null");

        Assert.Equal(Volume(aBox), Volume(aCopy), Precision.Confusion());
    }

    [Fact]
    public void CopyGeomFalse()
    {
        TopoDS_Shape aBox = Box10();

        var aCopier = new BRepBuilderAPI_Copy(aBox, false);
        TopoDS_Shape aCopy = aCopier.Shape();
        Assert.False(aCopy.IsNull(), "Copied shape is null");
        Assert.True(new BRepCheck_Analyzer(aCopy).IsValid(), "aCopy is not valid");
    }
}

/// <summary>OCCT's BRepBuilderAPI_MakeEdge GTests.</summary>
public class BRepBuilderAPI_MakeEdgeTest
{
    private static double Length(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.LinearProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void LinearEdge_TwoPoints()
    {
        var aP1 = new gp_Pnt(0.0, 0.0, 0.0);
        var aP2 = new gp_Pnt(10.0, 0.0, 0.0);

        var aMakeEdge = new BRepBuilderAPI_MakeEdge(aP1, aP2);
        Assert.True(aMakeEdge.IsDone(), "Linear edge creation failed");

        TopoDS_Edge anEdge = aMakeEdge.Edge();
        Assert.False(anEdge.IsNull(), "Resulting edge is null");

        // Verify edge length
        Assert.Equal(10.0, Length(anEdge), Precision.Confusion());
    }

    [Fact]
    public void CircularEdge_Full()
    {
        var anAxis = new gp_Ax2(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        var aCircle = new Geom_Circle(anAxis, 5.0);

        var aMakeEdge = new BRepBuilderAPI_MakeEdge(aCircle);
        Assert.True(aMakeEdge.IsDone(), "Full circular edge creation failed");

        TopoDS_Edge anEdge = aMakeEdge.Edge();
        Assert.False(anEdge.IsNull());

        // Full circle length = 2*PI*R
        Assert.Equal(2.0 * Math.PI * 5.0, Length(anEdge), Precision.Confusion());
    }

    [Fact]
    public void CircularEdge_Trimmed()
    {
        var anAxis = new gp_Ax2(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        var aCircle = new Geom_Circle(anAxis, 5.0);

        // Half circle: 0 to PI
        var aMakeEdge = new BRepBuilderAPI_MakeEdge(aCircle, 0.0, Math.PI);
        Assert.True(aMakeEdge.IsDone(), "Trimmed circular edge creation failed");

        TopoDS_Edge anEdge = aMakeEdge.Edge();
        Assert.False(anEdge.IsNull());

        // Half circle length = PI*R
        Assert.Equal(Math.PI * 5.0, Length(anEdge), Precision.Confusion());
    }

    [Fact]
    public void EdgeFromLine_WithBounds()
    {
        var aLine = new Geom_Line(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));

        var aMakeEdge = new BRepBuilderAPI_MakeEdge(aLine, 0.0, 7.0);
        Assert.True(aMakeEdge.IsDone(), "Edge from line with bounds failed");

        TopoDS_Edge anEdge = aMakeEdge.Edge();
        Assert.False(anEdge.IsNull());

        Assert.Equal(7.0, Length(anEdge), Precision.Confusion());
    }

    [Fact]
    public void VertexExtraction()
    {
        var aP1 = new gp_Pnt(0.0, 0.0, 0.0);
        var aP2 = new gp_Pnt(10.0, 0.0, 0.0);

        var aMakeEdge = new BRepBuilderAPI_MakeEdge(aP1, aP2);
        Assert.True(aMakeEdge.IsDone());

        TopoDS_Edge anEdge = aMakeEdge.Edge();
        TopExp.Vertices(anEdge, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);

        Assert.False(aV1.IsNull(), "First vertex is null");
        Assert.False(aV2.IsNull(), "Last vertex is null");

        gp_Pnt aFirstPnt = BRep_Tool.Pnt(aV1);
        gp_Pnt aLastPnt = BRep_Tool.Pnt(aV2);

        Assert.True(aFirstPnt.IsEqual(aP1, Precision.Confusion()));
        Assert.True(aLastPnt.IsEqual(aP2, Precision.Confusion()));
    }

    [Fact]
    public void ToleranceCheck()
    {
        var aP1 = new gp_Pnt(0.0, 0.0, 0.0);
        var aP2 = new gp_Pnt(10.0, 0.0, 0.0);

        var aMakeEdge = new BRepBuilderAPI_MakeEdge(aP1, aP2);
        Assert.True(aMakeEdge.IsDone());

        TopoDS_Edge anEdge = aMakeEdge.Edge();
        double aTol = BRep_Tool.Tolerance(anEdge);
        Assert.True(aTol > 0.0, "Edge tolerance should be positive");
        Assert.True(aTol <= Precision.Confusion(), "Edge tolerance should not exceed confusion");
    }
}

/// <summary>
/// OCCT's BRepBuilderAPI_MakeFace GTests.
/// </summary>
public class BRepBuilderAPI_MakeFaceTest
{
    private static double Area(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void FaceFromPlane()
    {
        var aPln = new gp_Pln(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        var aMakeFace = new BRepBuilderAPI_MakeFace(aPln);
        Assert.True(aMakeFace.IsDone(), "Face from plane creation failed");

        TopoDS_Face aFace = aMakeFace.Face();
        Assert.False(aFace.IsNull(), "Resulting face is null");
    }

    [Fact]
    public void FaceFromWire()
    {
        // Create a rectangular wire in XY plane
        var aP1 = new gp_Pnt(0.0, 0.0, 0.0);
        var aP2 = new gp_Pnt(10.0, 0.0, 0.0);
        var aP3 = new gp_Pnt(10.0, 5.0, 0.0);
        var aP4 = new gp_Pnt(0.0, 5.0, 0.0);

        TopoDS_Edge anE1 = new BRepBuilderAPI_MakeEdge(aP1, aP2).Edge();
        TopoDS_Edge anE2 = new BRepBuilderAPI_MakeEdge(aP2, aP3).Edge();
        TopoDS_Edge anE3 = new BRepBuilderAPI_MakeEdge(aP3, aP4).Edge();
        TopoDS_Edge anE4 = new BRepBuilderAPI_MakeEdge(aP4, aP1).Edge();

        var aMakeWire = new BRepBuilderAPI_MakeWire();
        aMakeWire.Add(anE1);
        aMakeWire.Add(anE2);
        aMakeWire.Add(anE3);
        aMakeWire.Add(anE4);
        Assert.True(aMakeWire.IsDone(), "Wire creation failed");

        var aMakeFace = new BRepBuilderAPI_MakeFace(aMakeWire.Wire());
        Assert.True(aMakeFace.IsDone(), "Face from wire creation failed");

        TopoDS_Face aFace = aMakeFace.Face();
        Assert.False(aFace.IsNull());
        Assert.True(new BRepCheck_Analyzer(aFace).IsValid(), "aFace is not valid");

        // Area should be 10 * 5 = 50
        Assert.Equal(50.0, Area(aFace), Precision.Confusion());
    }

    [Fact]
    public void FaceFromGeomPlane_WithBounds()
    {
        var aGeomPlane = new Geom_Plane(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        var aMakeFace = new BRepBuilderAPI_MakeFace(aGeomPlane, 0.0, 10.0, 0.0, 5.0, Precision.Confusion());
        Assert.True(aMakeFace.IsDone(), "Face from Geom_Plane with bounds failed");

        TopoDS_Face aFace = aMakeFace.Face();
        Assert.False(aFace.IsNull());
        Assert.True(new BRepCheck_Analyzer(aFace).IsValid(), "aFace is not valid");

        Assert.Equal(50.0, Area(aFace), Precision.Confusion());
    }

    [Fact]
    public void FaceFromCylindricalSurface()
    {
        var anAxis = new gp_Ax3(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        var aCylSurf = new Geom_CylindricalSurface(anAxis, 5.0);

        // Create a bounded face: U from 0 to 2*PI, V from 0 to 10
        var aMakeFace = new BRepBuilderAPI_MakeFace(aCylSurf, 0.0, 2.0 * Math.PI, 0.0, 10.0, Precision.Confusion());
        Assert.True(aMakeFace.IsDone(), "Face from cylindrical surface failed");

        TopoDS_Face aFace = aMakeFace.Face();
        Assert.False(aFace.IsNull());
        Assert.True(new BRepCheck_Analyzer(aFace).IsValid(), "aFace is not valid");

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
        var aBB = new BRep_Builder();
        TopoDS_Vertex aV1 = new(), aV2 = new(), aV3 = new();

        // Create vertices
        aBB.MakeVertex(ref aV1, new gp_Pnt(0, 0, 0), 0.1);
        aBB.MakeVertex(ref aV2, new gp_Pnt(5, 0, 0), 0.1);
        aBB.MakeVertex(ref aV3, new gp_Pnt(10, 0, 0), 0.1);

        // Create edges
        TopoDS_Edge anE1 = new BRepBuilderAPI_MakeEdge(aV1, aV2).Edge();
        TopoDS_Edge anE2 = new BRepBuilderAPI_MakeEdge(aV2, aV3).Edge();

        // Build wire with individually added edges
        var aMW = new BRepBuilderAPI_MakeWire();
        aMW.Add(anE1);
        aMW.Add(anE2);

        // Create additional vertices and edges for list test
        TopoDS_Vertex aV4 = new(), aV5 = new(), aV6 = new(), aV7 = new();
        aBB.MakeVertex(ref aV4, new gp_Pnt(10, 0.05, 0), 0.07);
        aBB.MakeVertex(ref aV5, new gp_Pnt(10, -0.05, 0), 0.07);
        aBB.MakeVertex(ref aV6, new gp_Pnt(10, 2, 0), 0.07);
        aBB.MakeVertex(ref aV7, new gp_Pnt(10, -2, 0), 0.07);

        TopoDS_Edge anE3 = new BRepBuilderAPI_MakeEdge(aV4, aV6).Edge();
        TopoDS_Edge anE4 = new BRepBuilderAPI_MakeEdge(aV5, aV7).Edge();

        // Add edges as a list
        var aListOfEdges = new NCollection_List<TopoDS_Shape>();
        aListOfEdges.Append(anE3);
        aListOfEdges.Append(anE4);

        aMW.Add(aListOfEdges);

        // Verify wire was created successfully
        Assert.True(aMW.IsDone(), "Wire builder should complete successfully");
        TopoDS_Wire aWire = aMW.Wire();
        Assert.False(aWire.IsNull(), "Resulting wire should not be null");
    }
}

/// <summary>
/// OCCT's BRepBuilderAPI_Transform GTests, on BRepPrimAPI_MakeBox's (frozen) boxes.
/// </summary>
public class BRepBuilderAPI_TransformTest
{
    private static TopoDS_Shape Box10() => new BRepPrimAPI_MakeBox(10.0, 10.0, 10.0).Shape();

    private static GProp_GProps VolumeProps(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps;
    }

    [Fact]
    public void Translate()
    {
        TopoDS_Shape aBox = Box10();

        var aTrsf = new gp_Trsf();
        aTrsf.SetTranslation(new gp_Vec(100.0, 0.0, 0.0));

        var aTransform = new BRepBuilderAPI_Transform(aBox, aTrsf);
        TopoDS_Shape aResult = aTransform.Shape();
        Assert.False(aResult.IsNull(), "Transformed shape is null");

        gp_Pnt aCenterOfMass = VolumeProps(aResult).CentreOfMass();

        Assert.Equal(105.0, aCenterOfMass.X(), Precision.Confusion());
        Assert.Equal(5.0, aCenterOfMass.Y(), Precision.Confusion());
        Assert.Equal(5.0, aCenterOfMass.Z(), Precision.Confusion());
    }

    [Fact]
    public void Rotate()
    {
        TopoDS_Shape aBox = Box10();

        var aTrsf = new gp_Trsf();
        var aZAxis = new gp_Ax1(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0));
        aTrsf.SetRotation(aZAxis, Math.PI / 2.0);

        var aTransform = new BRepBuilderAPI_Transform(aBox, aTrsf);
        TopoDS_Shape aResult = aTransform.Shape();
        Assert.False(aResult.IsNull(), "Transformed shape is null");

        gp_Pnt aCenterOfMass = VolumeProps(aResult).CentreOfMass();

        Assert.Equal(-5.0, aCenterOfMass.X(), Precision.Confusion());
        Assert.Equal(5.0, aCenterOfMass.Y(), Precision.Confusion());
    }

    [Fact]
    public void Scale()
    {
        TopoDS_Shape aBox = Box10();

        var aTrsf = new gp_Trsf();
        aTrsf.SetScale(new gp_Pnt(0.0, 0.0, 0.0), 2.0);

        var aTransform = new BRepBuilderAPI_Transform(aBox, aTrsf);
        TopoDS_Shape aResult = aTransform.Shape();
        Assert.False(aResult.IsNull(), "Transformed shape is null");

        Assert.Equal(8000.0, VolumeProps(aResult).Mass(), Precision.Confusion());
    }

    [Fact]
    public void Mirror()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(10.0, 0.0, 0.0), 10.0, 10.0, 10.0).Shape();

        var aTrsf = new gp_Trsf();
        var aYZPlane = new gp_Ax2(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(1.0, 0.0, 0.0));
        aTrsf.SetMirror(aYZPlane);

        var aTransform = new BRepBuilderAPI_Transform(aBox, aTrsf);
        TopoDS_Shape aResult = aTransform.Shape();
        Assert.False(aResult.IsNull(), "Transformed shape is null");

        gp_Pnt aCenterOfMass = VolumeProps(aResult).CentreOfMass();

        Assert.True(aCenterOfMass.X() < 0.0, "Center of mass X should be negative after mirroring through YZ plane");
    }

    [Fact]
    public void ShapeValidity()
    {
        TopoDS_Shape aBox = Box10();

        var aTrsf = new gp_Trsf();
        aTrsf.SetTranslation(new gp_Vec(50.0, 50.0, 50.0));

        var aTransform = new BRepBuilderAPI_Transform(aBox, aTrsf);
        TopoDS_Shape aResult = aTransform.Shape();
        Assert.False(aResult.IsNull(), "Transformed shape is null");
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid(), "aResult is not valid");
    }
}
