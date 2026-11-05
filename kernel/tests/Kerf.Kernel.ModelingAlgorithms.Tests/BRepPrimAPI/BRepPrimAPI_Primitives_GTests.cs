// Port of: src/ModelingAlgorithms/TKPrim/GTests/BRepPrimAPI_MakeBox_Test.cxx
// Port of: src/ModelingAlgorithms/TKPrim/GTests/BRepPrimAPI_MakeCone_Test.cxx
// Port of: src/ModelingAlgorithms/TKPrim/GTests/BRepPrimAPI_MakeCylinder_Test.cxx
// Port of: src/ModelingAlgorithms/TKPrim/GTests/BRepPrimAPI_MakeSphere_Test.cxx
// Port of: src/ModelingAlgorithms/TKPrim/GTests/BRepPrimAPI_MakeTorus_Test.cxx
// Port of: src/ModelingAlgorithms/TKPrim/GTests/BRepPrimAPI_MakeWedge_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

internal static class PrimitiveGTestHelpers
{
    public static int CountFaces(in Shape theShape)
    {
        int aFaceCount = 0;
        for (Explorer anExp = new(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aFaceCount++;
        }
        return aFaceCount;
    }

    public static double Volume(in Shape theShape)
    {
        Properties aProps = new();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }
}

public class BRepPrimAPI_MakeBoxTest
{
    [Fact]
    public void UnitBox()
    {
        MakeBox aBox = new(1.0, 1.0, 1.0);
        Shape aShape = aBox.Shape();

        Assert.True(aBox.IsDone, "Unit box creation failed");
        Assert.False(aShape.IsNull, "Resulting shape is null");

        Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Unit box shape is not valid");
    }

    [Fact]
    public void TopologyCounts()
    {
        MakeBox aBox = new(10.0, 20.0, 30.0);
        Shape aShape = aBox.Shape();
        Assert.True(aBox.IsDone);

        Assert.Equal(6, PrimitiveGTestHelpers.CountFaces(aShape));

        int anEdgeCount = 0;
        for (Explorer anExp = new(aShape, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            anEdgeCount++;
        }
        // 12 edges x 2 faces each
        Assert.Equal(24, anEdgeCount);

        int aVertexCount = 0;
        for (Explorer anExp = new(aShape, ShapeType.Vertex); anExp.More(); anExp.Next())
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
        MakeBox aBox = new(aWidth, aHeight, aDepth);
        Shape aShape = aBox.Shape();
        Assert.True(aBox.IsDone);

        Assert.Equal(aWidth * aHeight * aDepth, PrimitiveGTestHelpers.Volume(aShape), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void SurfaceArea()
    {
        const double aW = 3.0;
        const double aH = 4.0;
        const double aD = 5.0;
        MakeBox aBox = new(aW, aH, aD);
        Shape aShape = aBox.Shape();
        Assert.True(aBox.IsDone);

        Properties aProps = new();
        MassPropertiesBuilder.SurfaceProperties(aShape, aProps);
        double anExpectedArea = 2.0 * (aW * aH + aW * aD + aH * aD);
        Assert.Equal(anExpectedArea, aProps.Mass(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void TwoCornerPoints()
    {
        Point3D aP1 = new(1.0, 2.0, 3.0);
        Point3D aP2 = new(4.0, 6.0, 8.0);
        MakeBox aBox = new(aP1, aP2);
        Shape aShape = aBox.Shape();
        Assert.True(aBox.IsDone, "Box from two corner points failed");
        Assert.False(aShape.IsNull);

        Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Box from two points is not valid");

        // Volume = (4-1)*(6-2)*(8-3) = 3*4*5 = 60
        Assert.Equal(60.0, PrimitiveGTestHelpers.Volume(aShape), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ShapeValidity()
    {
        MakeBox aBox = new(100.0, 200.0, 300.0);
        Shape aShape = aBox.Shape();
        Assert.True(aBox.IsDone);

        Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Large box shape is not valid");
    }
}

public class BRepPrimAPI_MakeConeTest
{
    [Fact]
    public void FullCone()
    {
        ApiMakeCone aCone = new(new Frame3D(), 5.0, 0.0, 10.0);
        Shape aShape = aCone.Shape();
        Assert.True(aCone.IsDone, "Full cone creation failed");
        Assert.False(aShape.IsNull, "Resulting shape is null");

        Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Full cone shape is not valid");
    }

    [Fact]
    public void TruncatedCone()
    {
        ApiMakeCone aCone = new(new Frame3D(), 5.0, 2.0, 10.0);
        Shape aShape = aCone.Shape();
        Assert.True(aCone.IsDone, "Truncated cone creation failed");
        Assert.False(aShape.IsNull, "Resulting shape is null");

        Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Truncated cone shape is not valid");
    }

    [Fact]
    public void FullConeFaceCount()
    {
        ApiMakeCone aCone = new(new Frame3D(), 5.0, 0.0, 10.0);
        Shape aShape = aCone.Shape();
        Assert.True(aCone.IsDone);

        // lateral + base
        Assert.Equal(2, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void TruncatedConeFaceCount()
    {
        ApiMakeCone aCone = new(new Frame3D(), 5.0, 2.0, 10.0);
        Shape aShape = aCone.Shape();
        Assert.True(aCone.IsDone);

        // lateral + 2 bases
        Assert.Equal(3, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void TruncatedConeVolume()
    {
        const double aR1 = 5.0;
        const double aR2 = 2.0;
        const double aH = 10.0;
        ApiMakeCone aCone = new(new Frame3D(), aR1, aR2, aH);
        Shape aShape = aCone.Shape();
        Assert.True(aCone.IsDone);

        // V = PI * H / 3 * (R1^2 + R1*R2 + R2^2)
        double anExpectedVolume = M_PI * aH / 3.0 * (aR1 * aR1 + aR1 * aR2 + aR2 * aR2);
        Assert.Equal(anExpectedVolume, PrimitiveGTestHelpers.Volume(aShape), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void PartialCone()
    {
        ApiMakeCone aCone = new(new Frame3D(), 5.0, 2.0, 10.0, M_PI);
        Shape aShape = aCone.Shape();
        Assert.True(aCone.IsDone, "Partial cone creation failed");
        Assert.False(aShape.IsNull, "Resulting shape is null");

        Analyzer anAnalyzer = new(aShape);
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
        ApiMakeCylinder aCylinder = new(aRadius, aHeight);
        Shape aShape = aCylinder.Shape();
        Assert.True(aCylinder.IsDone, "Full cylinder creation failed");
        Assert.False(aShape.IsNull, "Resulting shape is null");

        Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Full cylinder shape is not valid");
    }

    [Fact]
    public void FaceCount()
    {
        ApiMakeCylinder aCylinder = new(5.0, 10.0);
        Shape aShape = aCylinder.Shape();
        Assert.True(aCylinder.IsDone);

        // Full cylinder: 1 lateral + 2 caps = 3
        Assert.Equal(3, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void Volume()
    {
        const double aRadius = 5.0;
        const double aHeight = 10.0;
        ApiMakeCylinder aCylinder = new(aRadius, aHeight);
        Shape aShape = aCylinder.Shape();
        Assert.True(aCylinder.IsDone);

        double anExpectedVolume = M_PI * aRadius * aRadius * aHeight;
        Assert.Equal(anExpectedVolume, PrimitiveGTestHelpers.Volume(aShape), 0.01);
    }

    [Fact]
    public void PartialCylinder_AngleLimited()
    {
        const double aRadius = 5.0;
        const double aHeight = 10.0;
        // Half cylinder: angle = PI
        ApiMakeCylinder aCylinder = new(aRadius, aHeight, M_PI);
        Shape aShape = aCylinder.Shape();
        Assert.True(aCylinder.IsDone, "Partial cylinder creation failed");
        Assert.False(aShape.IsNull);

        Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Partial cylinder shape is not valid");

        // 1 lateral + 2 caps + 2 planar sides = 5
        Assert.Equal(5, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void ShapeValidity()
    {
        ApiMakeCylinder aCylinder = new(100.0, 200.0);
        Shape aShape = aCylinder.Shape();
        Assert.True(aCylinder.IsDone);

        Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Large cylinder shape is not valid");
    }
}

public class BRepPrimAPI_MakeSphereTest
{
    [Fact]
    public void FullSphere()
    {
        const double aRadius = 5.0;
        MakeSphere aSphere = new(aRadius);
        Shape aShape = aSphere.Shape();
        Assert.True(aSphere.IsDone, "Full sphere creation failed");
        Assert.False(aShape.IsNull, "Resulting shape is null");

        Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Full sphere shape is not valid");
    }

    [Fact]
    public void FaceCount()
    {
        MakeSphere aSphere = new(5.0);
        Shape aShape = aSphere.Shape();
        Assert.True(aSphere.IsDone);

        Assert.Equal(1, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void Volume()
    {
        const double aRadius = 5.0;
        MakeSphere aSphere = new(aRadius);
        Shape aShape = aSphere.Shape();
        Assert.True(aSphere.IsDone);

        double anExpectedVolume = (4.0 / 3.0) * M_PI * aRadius * aRadius * aRadius;
        Assert.Equal(anExpectedVolume, PrimitiveGTestHelpers.Volume(aShape), 0.01);
    }

    [Fact]
    public void PartialSphere_AngleLimited()
    {
        const double aRadius = 5.0;
        // Half sphere: angle from 0 to PI
        MakeSphere aSphere = new(aRadius, M_PI);
        Shape aShape = aSphere.Shape();
        Assert.True(aSphere.IsDone, "Partial sphere creation failed");
        Assert.False(aShape.IsNull);

        Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Partial sphere shape is not valid");

        Assert.True(PrimitiveGTestHelpers.CountFaces(aShape) > 1, "Partial sphere should have more than 1 face");
    }

    [Fact]
    public void CenterOfMass()
    {
        const double aRadius = 5.0;
        MakeSphere aSphere = new(aRadius);
        Shape aShape = aSphere.Shape();
        Assert.True(aSphere.IsDone);

        Properties aProps = new();
        MassPropertiesBuilder.VolumeProperties(aShape, aProps);
        Point3D aCOM = aProps.CentreOfMass();

        // Center of mass of full sphere should be at origin
        Assert.Equal(0.0, aCOM.X, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aCOM.Y, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(0.0, aCOM.Z, global::Kerf.Engine.Foundation.Precision.Confusion());
    }
}

public class BRepPrimAPI_MakeTorusTest
{
    // Creates a partial torus and checks its validity.
    private static bool createAndCheckPartialTorus(double theR1, double theR2, double theAngle1, double theAngle2,
                                                   double theAngle = 2.0 * M_PI)
    {
        Frame3D anAxis = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        MakeTorus aMakeTorus = new(anAxis, theR1, theR2, theAngle1, theAngle2, theAngle);
        Shape aShape = aMakeTorus.Shape();
        if (!aMakeTorus.IsDone || aShape.IsNull)
        {
            return false;
        }

        Analyzer anAnalyzer = new(aShape);
        return anAnalyzer.IsValid();
    }

    [Fact]
    public void FullTorus()
    {
        const double R1 = 10.0;
        const double R2 = 2.0;

        MakeTorus aMakeTorus = new(R1, R2);
        Shape aShape = aMakeTorus.Shape();

        Assert.True(aMakeTorus.IsDone, "Full torus creation failed");
        Assert.False(aShape.IsNull, "Resulting shape is null");

        Analyzer anAnalyzer = new(aShape);
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

        MakeTorus aMakeTorus = new(R1, R2, angle);
        Shape aShape = aMakeTorus.Shape();

        Assert.True(aMakeTorus.IsDone, "Partial torus (angle only) creation failed");
        Assert.False(aShape.IsNull, "Resulting shape is null");

        Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Partial torus (angle only) shape is not valid");

        // 1 lateral + 2 planar (start/end sides)
        Assert.Equal(3, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void LateralFaceParameterization()
    {
        const double R1 = 5.0;
        const double R2 = 1.0;

        MakeTorus aMakeTorus = new(R1, R2);
        Shape aShape = aMakeTorus.Shape();
        Assert.True(aMakeTorus.IsDone);

        Explorer anExp = new(aShape, ShapeType.Face);
        Assert.True(anExp.More(), "No faces found");

        Face aFace = ShapeCasts.Face(anExp.Current());
        Surface? aSurface = Tool.Surface(aFace);
        var aTorusSurf = aSurface as ToroidalSurface;
        Assert.NotNull(aTorusSurf);

        Assert.Equal(R1, aTorusSurf.MajorRadius, 1e-10);
        Assert.Equal(R2, aTorusSurf.MinorRadius, 1e-10);

        // At V = PI/2 (top of tube), Z should be +R2
        aTorusSurf.D0(0.0, M_PI_2, out Point3D aPnt);
        Assert.Equal(R2, aPnt.Z, 1e-10);

        // At V = -PI/2 (bottom of tube), Z should be -R2
        aTorusSurf.D0(0.0, -M_PI_2, out aPnt);
        Assert.Equal(-R2, aPnt.Z, 1e-10);
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

        Frame3D anAxis = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        MakeTorus aMakeTorus = new(anAxis, R1, R2, angle1, angle2, angle);
        Shape aShape = aMakeTorus.Shape();

        Assert.True(aMakeTorus.IsDone, "Torus creation failed");
        Assert.False(aShape.IsNull, "Resulting shape is null");

        Analyzer anAnalyzer = new(aShape);
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

        Frame3D anAxis = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        MakeTorus aMakeTorus = new(anAxis, R1, R2, angle1, angle2, angle);
        Shape aShape = aMakeTorus.Shape();

        Assert.True(aMakeTorus.IsDone, "Torus creation failed");
        Assert.False(aShape.IsNull, "Resulting shape is null");

        Analyzer anAnalyzer = new(aShape);
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

        Frame3D anAxis = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));

        // Full revolution (2*PI) - no sides
        MakeTorus aMakeTorus = new(anAxis, R1, R2, angle1, angle2);
        Shape aShape = aMakeTorus.Shape();

        Assert.True(aMakeTorus.IsDone, "Torus creation failed");
        Assert.False(aShape.IsNull, "Shape is null");

        Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Full torus (no sides) with equal heights should be valid");
    }
}

public class BRepPrimAPI_MakeWedgeTest
{
    [Fact]
    public void WedgeFromDimensions()
    {
        MakeWedge aWedge = new(10.0, 10.0, 10.0, 5.0);
        Shape aShape = aWedge.Shape();
        Assert.True(aWedge.IsDone, "Wedge creation failed");
        Assert.False(aShape.IsNull, "Resulting shape is null");

        Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Wedge shape is not valid");
    }

    [Fact]
    public void WedgeFaceCount_Ltx5()
    {
        MakeWedge aWedge = new(10.0, 10.0, 10.0, 5.0);
        Shape aShape = aWedge.Shape();
        Assert.True(aWedge.IsDone);

        Assert.Equal(6, PrimitiveGTestHelpers.CountFaces(aShape));
    }

    [Fact]
    public void WedgeFaceCount_Ltx0()
    {
        MakeWedge aWedge = new(10.0, 10.0, 10.0, 0.0);
        Shape aShape = aWedge.Shape();
        Assert.True(aWedge.IsDone);

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
        MakeWedge aWedge = new(aDx, aDy, aDz, aLtx);
        Shape aShape = aWedge.Shape();
        Assert.True(aWedge.IsDone);

        // Volume = dy * dz * (dx + ltx) / 2
        double anExpectedVolume = aDy * aDz * (aDx + aLtx) / 2.0;
        Assert.Equal(anExpectedVolume, PrimitiveGTestHelpers.Volume(aShape), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void WedgeFullParams()
    {
        MakeWedge aWedge = new(20.0, 10.0, 20.0, 5.0, 5.0, 15.0, 15.0);
        Shape aShape = aWedge.Shape();
        Assert.True(aWedge.IsDone, "Wedge with full parameters creation failed");
        Assert.False(aShape.IsNull, "Resulting shape is null");

        Analyzer anAnalyzer = new(aShape);
        Assert.True(anAnalyzer.IsValid(), "Wedge with full parameters shape is not valid");
    }
}
