// Port of: src/ModelingAlgorithms/TKPrim/GTests/BRepPrimAPI_MakeBox_Test.cxx
// Port of: src/ModelingAlgorithms/TKPrim/GTests/BRepPrimAPI_MakeCone_Test.cxx
// Port of: src/ModelingAlgorithms/TKPrim/GTests/BRepPrimAPI_MakeCylinder_Test.cxx
// Port of: src/ModelingAlgorithms/TKPrim/GTests/BRepPrimAPI_MakeSphere_Test.cxx
// Port of: src/ModelingAlgorithms/TKPrim/GTests/BRepPrimAPI_MakeTorus_Test.cxx
// Port of: src/ModelingAlgorithms/TKPrim/GTests/BRepPrimAPI_MakeWedge_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

internal static class PrimitiveGTestHelpers
{
    public static int CountFaces(in TopoDS_Shape theShape)
    {
        int aFaceCount = 0;
        for (TopExp_Explorer anExp = new(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aFaceCount++;
        }
        return aFaceCount;
    }

    public static double Volume(in TopoDS_Shape theShape)
    {
        GProp_GProps aProps = new();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }
}

public class BRepPrimAPI_MakeBoxTest
{
    [Fact]
    public void UnitBox()
    {
        BRepPrimAPI_MakeBox aBox = new(1.0, 1.0, 1.0);
        TopoDS_Shape aShape = aBox.Shape();

        Assert.True(aBox.IsDone(), "Unit box creation failed");
        Assert.False(aShape.IsNull(), "Resulting shape is null");

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Unit box shape is not valid");
    }

    [Fact]
    public void TopologyCounts()
    {
        BRepPrimAPI_MakeBox aBox = new(10.0, 20.0, 30.0);
        TopoDS_Shape aShape = aBox.Shape();
        Assert.True(aBox.IsDone());

        Assert.Equal(6, PrimitiveGTestHelpers.CountFaces(aShape));

        int anEdgeCount = 0;
        for (TopExp_Explorer anExp = new(aShape, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            anEdgeCount++;
        }
        // 12 edges x 2 faces each
        Assert.Equal(24, anEdgeCount);

        int aVertexCount = 0;
        for (TopExp_Explorer anExp = new(aShape, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            aVertexCount++;
        }
        // 8 vertices x 6 faces
        Assert.Equal(48, aVertexCount);
    }

    [Fact]
    public void Volume()
    {
        const double aWidth = 3.0;
        const double aHeight = 4.0;
        const double aDepth = 5.0;
        BRepPrimAPI_MakeBox aBox = new(aWidth, aHeight, aDepth);
        TopoDS_Shape aShape = aBox.Shape();
        Assert.True(aBox.IsDone());

        Assert.Equal(aWidth * aHeight * aDepth, PrimitiveGTestHelpers.Volume(aShape), Precision.Confusion());
    }

    [Fact]
    public void SurfaceArea()
    {
        const double aW = 3.0;
        const double aH = 4.0;
        const double aD = 5.0;
        BRepPrimAPI_MakeBox aBox = new(aW, aH, aD);
        TopoDS_Shape aShape = aBox.Shape();
        Assert.True(aBox.IsDone());

        GProp_GProps aProps = new();
        BRepGProp.SurfaceProperties(aShape, aProps);
        double anExpectedArea = 2.0 * (aW * aH + aW * aD + aH * aD);
        Assert.Equal(anExpectedArea, aProps.Mass(), Precision.Confusion());
    }

    [Fact]
    public void TwoCornerPoints()
    {
        gp_Pnt aP1 = new(1.0, 2.0, 3.0);
        gp_Pnt aP2 = new(4.0, 6.0, 8.0);
        BRepPrimAPI_MakeBox aBox = new(aP1, aP2);
        TopoDS_Shape aShape = aBox.Shape();
        Assert.True(aBox.IsDone(), "Box from two corner points failed");
        Assert.False(aShape.IsNull());

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Box from two points is not valid");

        // Volume = (4-1)*(6-2)*(8-3) = 3*4*5 = 60
        Assert.Equal(60.0, PrimitiveGTestHelpers.Volume(aShape), Precision.Confusion());
    }

    [Fact]
    public void ShapeValidity()
    {
        BRepPrimAPI_MakeBox aBox = new(100.0, 200.0, 300.0);
        TopoDS_Shape aShape = aBox.Shape();
        Assert.True(aBox.IsDone());

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Large box shape is not valid");
    }
}

public class BRepPrimAPI_MakeConeTest
{
    [Fact]
    public void FullCone()
    {
        BRepPrimAPI_MakeCone aCone = new(new gp_Ax2(), 5.0, 0.0, 10.0);
        TopoDS_Shape aShape = aCone.Shape();
        Assert.True(aCone.IsDone(), "Full cone creation failed");
        Assert.False(aShape.IsNull(), "Resulting shape is null");

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Full cone shape is not valid");
    }

    [Fact]
    public void TruncatedCone()
    {
        BRepPrimAPI_MakeCone aCone = new(new gp_Ax2(), 5.0, 2.0, 10.0);
        TopoDS_Shape aShape = aCone.Shape();
        Assert.True(aCone.IsDone(), "Truncated cone creation failed");
        Assert.False(aShape.IsNull(), "Resulting shape is null");

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Truncated cone shape is not valid");
    }

    [Fact]
    public void FullConeFaceCount()
    {
        BRepPrimAPI_MakeCone aCone = new(new gp_Ax2(), 5.0, 0.0, 10.0);
        TopoDS_Shape aShape = aCone.Shape();
        Assert.True(aCone.IsDone());

        // lateral + base
        Assert.Equal(2, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void TruncatedConeFaceCount()
    {
        BRepPrimAPI_MakeCone aCone = new(new gp_Ax2(), 5.0, 2.0, 10.0);
        TopoDS_Shape aShape = aCone.Shape();
        Assert.True(aCone.IsDone());

        // lateral + 2 bases
        Assert.Equal(3, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void TruncatedConeVolume()
    {
        const double aR1 = 5.0;
        const double aR2 = 2.0;
        const double aH = 10.0;
        BRepPrimAPI_MakeCone aCone = new(new gp_Ax2(), aR1, aR2, aH);
        TopoDS_Shape aShape = aCone.Shape();
        Assert.True(aCone.IsDone());

        // V = PI * H / 3 * (R1^2 + R1*R2 + R2^2)
        double anExpectedVolume = M_PI * aH / 3.0 * (aR1 * aR1 + aR1 * aR2 + aR2 * aR2);
        Assert.Equal(anExpectedVolume, PrimitiveGTestHelpers.Volume(aShape), Precision.Confusion());
    }

    [Fact]
    public void PartialCone()
    {
        BRepPrimAPI_MakeCone aCone = new(new gp_Ax2(), 5.0, 2.0, 10.0, M_PI);
        TopoDS_Shape aShape = aCone.Shape();
        Assert.True(aCone.IsDone(), "Partial cone creation failed");
        Assert.False(aShape.IsNull(), "Resulting shape is null");

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Partial cone shape is not valid");

        Assert.True(PrimitiveGTestHelpers.CountFaces(aShape) > 3, "Partial cone should have more than 3 faces");
    }
}

public class BRepPrimAPI_MakeCylinderTest
{
    [Fact]
    public void FullCylinder()
    {
        const double aRadius = 5.0;
        const double aHeight = 10.0;
        BRepPrimAPI_MakeCylinder aCylinder = new(aRadius, aHeight);
        TopoDS_Shape aShape = aCylinder.Shape();
        Assert.True(aCylinder.IsDone(), "Full cylinder creation failed");
        Assert.False(aShape.IsNull(), "Resulting shape is null");

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Full cylinder shape is not valid");
    }

    [Fact]
    public void FaceCount()
    {
        BRepPrimAPI_MakeCylinder aCylinder = new(5.0, 10.0);
        TopoDS_Shape aShape = aCylinder.Shape();
        Assert.True(aCylinder.IsDone());

        // Full cylinder: 1 lateral + 2 caps = 3
        Assert.Equal(3, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void Volume()
    {
        const double aRadius = 5.0;
        const double aHeight = 10.0;
        BRepPrimAPI_MakeCylinder aCylinder = new(aRadius, aHeight);
        TopoDS_Shape aShape = aCylinder.Shape();
        Assert.True(aCylinder.IsDone());

        double anExpectedVolume = M_PI * aRadius * aRadius * aHeight;
        Assert.Equal(anExpectedVolume, PrimitiveGTestHelpers.Volume(aShape), 0.01);
    }

    [Fact]
    public void PartialCylinder_AngleLimited()
    {
        const double aRadius = 5.0;
        const double aHeight = 10.0;
        // Half cylinder: angle = PI
        BRepPrimAPI_MakeCylinder aCylinder = new(aRadius, aHeight, M_PI);
        TopoDS_Shape aShape = aCylinder.Shape();
        Assert.True(aCylinder.IsDone(), "Partial cylinder creation failed");
        Assert.False(aShape.IsNull());

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Partial cylinder shape is not valid");

        // 1 lateral + 2 caps + 2 planar sides = 5
        Assert.Equal(5, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void ShapeValidity()
    {
        BRepPrimAPI_MakeCylinder aCylinder = new(100.0, 200.0);
        TopoDS_Shape aShape = aCylinder.Shape();
        Assert.True(aCylinder.IsDone());

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Large cylinder shape is not valid");
    }
}

public class BRepPrimAPI_MakeSphereTest
{
    [Fact]
    public void FullSphere()
    {
        const double aRadius = 5.0;
        BRepPrimAPI_MakeSphere aSphere = new(aRadius);
        TopoDS_Shape aShape = aSphere.Shape();
        Assert.True(aSphere.IsDone(), "Full sphere creation failed");
        Assert.False(aShape.IsNull(), "Resulting shape is null");

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Full sphere shape is not valid");
    }

    [Fact]
    public void FaceCount()
    {
        BRepPrimAPI_MakeSphere aSphere = new(5.0);
        TopoDS_Shape aShape = aSphere.Shape();
        Assert.True(aSphere.IsDone());

        Assert.Equal(1, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void Volume()
    {
        const double aRadius = 5.0;
        BRepPrimAPI_MakeSphere aSphere = new(aRadius);
        TopoDS_Shape aShape = aSphere.Shape();
        Assert.True(aSphere.IsDone());

        double anExpectedVolume = (4.0 / 3.0) * M_PI * aRadius * aRadius * aRadius;
        Assert.Equal(anExpectedVolume, PrimitiveGTestHelpers.Volume(aShape), 0.01);
    }

    [Fact]
    public void PartialSphere_AngleLimited()
    {
        const double aRadius = 5.0;
        // Half sphere: angle from 0 to PI
        BRepPrimAPI_MakeSphere aSphere = new(aRadius, M_PI);
        TopoDS_Shape aShape = aSphere.Shape();
        Assert.True(aSphere.IsDone(), "Partial sphere creation failed");
        Assert.False(aShape.IsNull());

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Partial sphere shape is not valid");

        Assert.True(PrimitiveGTestHelpers.CountFaces(aShape) > 1, "Partial sphere should have more than 1 face");
    }

    [Fact]
    public void CenterOfMass()
    {
        const double aRadius = 5.0;
        BRepPrimAPI_MakeSphere aSphere = new(aRadius);
        TopoDS_Shape aShape = aSphere.Shape();
        Assert.True(aSphere.IsDone());

        GProp_GProps aProps = new();
        BRepGProp.VolumeProperties(aShape, aProps);
        gp_Pnt aCOM = aProps.CentreOfMass();

        // Center of mass of full sphere should be at origin
        Assert.Equal(0.0, aCOM.X(), Precision.Confusion());
        Assert.Equal(0.0, aCOM.Y(), Precision.Confusion());
        Assert.Equal(0.0, aCOM.Z(), Precision.Confusion());
    }
}

public class BRepPrimAPI_MakeTorusTest
{
    // Creates a partial torus and checks its validity.
    private static bool createAndCheckPartialTorus(double theR1, double theR2, double theAngle1, double theAngle2,
                                                   double theAngle = 2.0 * M_PI)
    {
        gp_Ax2 anAxis = new(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        BRepPrimAPI_MakeTorus aMakeTorus = new(anAxis, theR1, theR2, theAngle1, theAngle2, theAngle);
        TopoDS_Shape aShape = aMakeTorus.Shape();
        if (!aMakeTorus.IsDone() || aShape.IsNull())
        {
            return false;
        }

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        return anAnalyzer.IsValid();
    }

    [Fact]
    public void FullTorus()
    {
        const double R1 = 10.0;
        const double R2 = 2.0;

        BRepPrimAPI_MakeTorus aMakeTorus = new(R1, R2);
        TopoDS_Shape aShape = aMakeTorus.Shape();

        Assert.True(aMakeTorus.IsDone(), "Full torus creation failed");
        Assert.False(aShape.IsNull(), "Resulting shape is null");

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Full torus shape is not valid");

        // A full torus should have exactly 1 face
        Assert.Equal(1, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void PartialTorusAngleOnly()
    {
        const double R1 = 10.0;
        const double R2 = 2.0;
        const double angle = M_PI; // Half torus around Z axis

        BRepPrimAPI_MakeTorus aMakeTorus = new(R1, R2, angle);
        TopoDS_Shape aShape = aMakeTorus.Shape();

        Assert.True(aMakeTorus.IsDone(), "Partial torus (angle only) creation failed");
        Assert.False(aShape.IsNull(), "Resulting shape is null");

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Partial torus (angle only) shape is not valid");

        // 1 lateral + 2 planar (start/end sides)
        Assert.Equal(3, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void LateralFaceParameterization()
    {
        const double R1 = 5.0;
        const double R2 = 1.0;

        BRepPrimAPI_MakeTorus aMakeTorus = new(R1, R2);
        TopoDS_Shape aShape = aMakeTorus.Shape();
        Assert.True(aMakeTorus.IsDone());

        TopExp_Explorer anExp = new(aShape, TopAbs_FACE);
        Assert.True(anExp.More(), "No faces found");

        TopoDS_Face aFace = TopoDS.Face(anExp.Current());
        Geom_Surface? aSurface = BRep_Tool.Surface(aFace);
        var aTorusSurf = aSurface as Geom_ToroidalSurface;
        Assert.NotNull(aTorusSurf);

        Assert.Equal(R1, aTorusSurf.MajorRadius(), 1e-10);
        Assert.Equal(R2, aTorusSurf.MinorRadius(), 1e-10);

        // At V = PI/2 (top of tube), Z should be +R2
        aTorusSurf.D0(0.0, M_PI_2, out gp_Pnt aPnt);
        Assert.Equal(R2, aPnt.Z(), 1e-10);

        // At V = -PI/2 (bottom of tube), Z should be -R2
        aTorusSurf.D0(0.0, -M_PI_2, out aPnt);
        Assert.Equal(-R2, aPnt.Z(), 1e-10);
    }

    // Bug 33133 original report: V range symmetric around 0 (-PI/2 to +PI/2) works.
    [Fact]
    public void PartialTorus_SymmetricAroundZero()
    {
        const double R1 = 1.0;
        const double R2 = 0.1;
        const double angle1 = -M_PI_2; // -90 degrees
        const double angle2 = M_PI_2;  // +90 degrees
        const double angle = 2.094;    // ~120 degrees

        gp_Ax2 anAxis = new(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        BRepPrimAPI_MakeTorus aMakeTorus = new(anAxis, R1, R2, angle1, angle2, angle);
        TopoDS_Shape aShape = aMakeTorus.Shape();

        Assert.True(aMakeTorus.IsDone(), "Torus creation failed");
        Assert.False(aShape.IsNull(), "Resulting shape is null");

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Symmetric partial torus should be valid");

        // 1 lateral + 2 top/bottom + 2 start/end = 5
        Assert.Equal(5, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void PartialTorus_SmallSymmetricRange()
    {
        const double angle1 = -M_PI / 4.0; // -45 degrees
        const double angle2 = M_PI / 4.0;  // +45 degrees

        bool isValid = createAndCheckPartialTorus(10.0, 2.0, angle1, angle2, M_PI / 4.0);
        Assert.True(isValid, "Small symmetric range (-45 to +45) should produce valid shape");
    }

    // Bug 23612: V range from 90 to 270 degrees (inverted height ordering in BRepPrim_OneAxis, fixed).
    [Fact]
    public void PartialTorus_Bug23612()
    {
        const double R1 = 10.0;
        const double R2 = 2.0;
        const double angle1 = M_PI_2;       // 90 degrees
        const double angle2 = 3.0 * M_PI_2; // 270 degrees
        const double angle = M_PI / 4.0;    // 45 degrees

        gp_Ax2 anAxis = new(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        BRepPrimAPI_MakeTorus aMakeTorus = new(anAxis, R1, R2, angle1, angle2, angle);
        TopoDS_Shape aShape = aMakeTorus.Shape();

        Assert.True(aMakeTorus.IsDone(), "Torus creation failed");
        Assert.False(aShape.IsNull(), "Resulting shape is null");

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Bug 23612 fixed: V range (90, 270) should produce valid shape");
    }

    // Equal heights (sin(0) = sin(180)): known limitation, degenerate AxisEdge topology.
    [Fact]
    public void PartialTorus_ZeroTo180()
    {
        bool isValid = createAndCheckPartialTorus(10.0, 2.0, 0.0, M_PI, M_PI / 4.0);
        Assert.False(isValid, "Known limitation: equal heights (sin(0)=sin(180)=0) not supported");
    }

    [Fact]
    public void PartialTorus_Minus180ToZero()
    {
        bool isValid = createAndCheckPartialTorus(10.0, 2.0, -M_PI, 0.0, M_PI / 4.0);
        Assert.False(isValid, "Known limitation: equal heights (sin(-180)=sin(0)=0) not supported");
    }

    [Fact]
    public void PartialTorus_180To360()
    {
        bool isValid = createAndCheckPartialTorus(10.0, 2.0, M_PI, 2 * M_PI, M_PI / 4.0);
        Assert.False(isValid, "Known limitation: equal heights (sin(180)=sin(360)=0) not supported");
    }

    [Fact]
    public void PartialTorus_PositiveRange()
    {
        bool isValid = createAndCheckPartialTorus(10.0, 2.0, M_PI / 4.0, 3.0 * M_PI / 4.0, M_PI / 4.0);
        Assert.False(isValid, "Known limitation: equal heights (sin(45)=sin(135)) not supported");
    }

    [Fact]
    public void PartialTorus_NegativeRange()
    {
        bool isValid = createAndCheckPartialTorus(10.0, 2.0, -3.0 * M_PI / 4.0, -M_PI / 4.0, M_PI / 4.0);
        Assert.False(isValid, "Known limitation: equal heights (sin(-135)=sin(-45)) not supported");
    }

    [Fact]
    public void PartialTorus_AsymmetricCrossingZero()
    {
        bool isValid = createAndCheckPartialTorus(10.0, 2.0, -M_PI / 6.0, M_PI / 3.0, M_PI / 4.0);
        Assert.True(isValid, "Asymmetric range crossing 0 but within (-PI/2, PI/2) should be valid");
    }

    [Fact]
    public void PartialTorus_SmallPositiveRange()
    {
        bool isValid = createAndCheckPartialTorus(10.0, 2.0, M_PI / 6.0, M_PI / 3.0, M_PI / 4.0);
        Assert.True(isValid, "Small positive range within (-PI/2, PI/2) should be valid");
    }

    // The root cause of bugs 23612/33133: BRepPrim_OneAxis assumed MeridianValue(VMax).Y() > MeridianValue(VMin).Y().
    [Fact]
    public void RootCause_HeightOrderingAssumption()
    {
        const double R1 = 10.0;
        const double R2 = 2.0;

        // For a torus meridian: Y(V) = R2 * sin(V)
        static double computeHeight(double angleRad) => R2 * Math.Sin(angleRad);

        // Case 1: Working range (-90 to +90 deg)
        {
            double vMin = -M_PI_2;
            double vMax = M_PI_2;
            Assert.True(computeHeight(vMax) > computeHeight(vMin), "Working case: Y(VMax) should be > Y(VMin)");
            Assert.True(createAndCheckPartialTorus(R1, R2, vMin, vMax, M_PI / 4.0),
                        "Range with correct height ordering should produce valid shape");
        }

        // Case 2: Previously failing range (90 to 270 deg) - Bug 23612 (FIXED)
        {
            double vMin = M_PI_2;
            double vMax = 3.0 * M_PI_2;
            Assert.True(computeHeight(vMax) < computeHeight(vMin), "Height ordering is inverted: Y(VMax) < Y(VMin)");
            Assert.True(createAndCheckPartialTorus(R1, R2, vMin, vMax, M_PI / 4.0),
                        "Bug 23612 fixed: inverted height ordering now handled correctly");
        }

        // Case 3: Equal heights (45 to 135 deg) - Known Limitation
        {
            double vMin = M_PI / 4.0;
            double vMax = 3.0 * M_PI / 4.0;
            Assert.Equal(computeHeight(vMin), computeHeight(vMax), 1e-10);
            Assert.False(createAndCheckPartialTorus(R1, R2, vMin, vMax, M_PI / 4.0),
                         "Known limitation: equal heights not yet supported");
        }
    }

    [Fact]
    public void FullTorus_EqualHeights()
    {
        const double R1 = 10.0;
        const double R2 = 2.0;
        const double angle1 = 0.0;  // 0 degrees
        const double angle2 = M_PI; // 180 degrees

        gp_Ax2 anAxis = new(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));

        // Full revolution (2*PI) - no sides
        BRepPrimAPI_MakeTorus aMakeTorus = new(anAxis, R1, R2, angle1, angle2);
        TopoDS_Shape aShape = aMakeTorus.Shape();

        Assert.True(aMakeTorus.IsDone(), "Torus creation failed");
        Assert.False(aShape.IsNull(), "Shape is null");

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Full torus (no sides) with equal heights should be valid");
    }
}

public class BRepPrimAPI_MakeWedgeTest
{
    [Fact]
    public void WedgeFromDimensions()
    {
        BRepPrimAPI_MakeWedge aWedge = new(10.0, 10.0, 10.0, 5.0);
        TopoDS_Shape aShape = aWedge.Shape();
        Assert.True(aWedge.IsDone(), "Wedge creation failed");
        Assert.False(aShape.IsNull(), "Resulting shape is null");

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Wedge shape is not valid");
    }

    [Fact]
    public void WedgeFaceCount_Ltx5()
    {
        BRepPrimAPI_MakeWedge aWedge = new(10.0, 10.0, 10.0, 5.0);
        TopoDS_Shape aShape = aWedge.Shape();
        Assert.True(aWedge.IsDone());

        Assert.Equal(6, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void WedgeFaceCount_Ltx0()
    {
        BRepPrimAPI_MakeWedge aWedge = new(10.0, 10.0, 10.0, 0.0);
        TopoDS_Shape aShape = aWedge.Shape();
        Assert.True(aWedge.IsDone());

        // the top degenerates to a line
        Assert.Equal(5, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void WedgeVolume()
    {
        const double aDx = 10.0;
        const double aDy = 10.0;
        const double aDz = 10.0;
        const double aLtx = 5.0;
        BRepPrimAPI_MakeWedge aWedge = new(aDx, aDy, aDz, aLtx);
        TopoDS_Shape aShape = aWedge.Shape();
        Assert.True(aWedge.IsDone());

        // Volume = dy * dz * (dx + ltx) / 2
        double anExpectedVolume = aDy * aDz * (aDx + aLtx) / 2.0;
        Assert.Equal(anExpectedVolume, PrimitiveGTestHelpers.Volume(aShape), Precision.Confusion());
    }

    [Fact]
    public void WedgeFullParams()
    {
        BRepPrimAPI_MakeWedge aWedge = new(20.0, 10.0, 20.0, 5.0, 5.0, 15.0, 15.0);
        TopoDS_Shape aShape = aWedge.Shape();
        Assert.True(aWedge.IsDone(), "Wedge with full parameters creation failed");
        Assert.False(aShape.IsNull(), "Resulting shape is null");

        BRepCheck_Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Wedge with full parameters shape is not valid");
    }
}
