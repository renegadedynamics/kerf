// Port of: src/ModelingAlgorithms/TKShHealing/GTests/ShapeAnalysis_CanonicalRecognition_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

// OCCT's tests intersect a cylinder with a plane with GeomAPI_IntSS (unit 5G). Until then, the intersection is the
// ellipse GeomAPI_IntSS computes.

/// <summary>Helpers shared by OCCT's canonical recognition tests.</summary>
internal static class CanonicalRecognitionTestUtils
{
    // OCCT's tests: BRepBuilderAPI_Sewing with its defaults, each face added, Perform, SewedShape.
    public static TopoDS_Shape Sew(params TopoDS_Face[] theFaces)
    {
        var aSewing = new BRepBuilderAPI_Sewing();
        foreach (TopoDS_Face aFace in theFaces)
        {
            aSewing.Add(aFace);
        }
        aSewing.Perform();
        return aSewing.SewedShape();
    }

    public static TopoDS_Shape NurbsConverted(in TopoDS_Shape theShape)
    {
        var aNurbsConvert = new BRepBuilderAPI_NurbsConvert(theShape);
        Assert.True(aNurbsConvert.IsDone(), "Failed to convert to NURBS");
        return aNurbsConvert.Shape();
    }
}

/// <summary>Surface recognition tests (cr/approx).</summary>
public class CanonicalRecognitionApproxTest
{
    private readonly ShapeAnalysis_CanonicalRecognition myRecognizer = new();
    private readonly double myTolerance = 1.0e-3;

    [Fact]
    public void PolylineToPlaneRecognition_A1()
    {
        var aPolygonMaker = new BRepBuilderAPI_MakePolygon();
        aPolygonMaker.Add(new Point3D(0, 0, 0));
        aPolygonMaker.Add(new Point3D(1, 0, 0));
        aPolygonMaker.Add(new Point3D(1, 1, 0));
        aPolygonMaker.Add(new Point3D(0, 1, 0));
        aPolygonMaker.Close();

        Assert.True(aPolygonMaker.IsDone(), "Failed to create polyline");
        TopoDS_Wire aWire = aPolygonMaker.Wire();

        myRecognizer.SetShape(aWire);
        var aResultPlane = new Plane3D();
        bool aResult = myRecognizer.IsPlane(myTolerance, ref aResultPlane);

        Assert.True(aResult, "Polyline should be recognized as planar");

        Point3D aOrigin = aResultPlane.Location();
        Direction3D aNormal = aResultPlane.Axis().Direction();
        Assert.Equal(1.0, Math.Abs(aNormal.Z()), myTolerance);
        Assert.Equal(0.0, aOrigin.Z(), myTolerance);
    }

    [Fact]
    public void CylinderRecognition_A2()
    {
        var aAxis = new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 1, 1));
        var aCylSurf = new Geom_CylindricalSurface(new CoordinateSystem(aAxis), 1.0);
        var aFaceMaker = new BRepBuilderAPI_MakeFace(aCylSurf, 0, 2 * M_PI, 0, 1, Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aFaceMaker.IsDone());
        TopoDS_Face aNurbsFace = TopoDS.Face(CanonicalRecognitionTestUtils.NurbsConverted(aFaceMaker.Face()));

        myRecognizer.SetShape(aNurbsFace);
        var aResultCylinder = new Cylinder();
        bool aResult = myRecognizer.IsCylinder(myTolerance, ref aResultCylinder);

        Assert.True(aResult, "NURBS face should be recognized as cylinder");
        Assert.Equal(1.0, aResultCylinder.Radius(), myTolerance);
    }

    [Fact]
    public void ConicalSurfaceRecognition_A3()
    {
        double aHalfAngle = M_PI / 6.0;
        var aAxis = new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z));
        var aConeSurf = new Geom_ConicalSurface(new CoordinateSystem(aAxis), aHalfAngle, 2.0);
        var aFaceMaker = new BRepBuilderAPI_MakeFace(aConeSurf, 0, 2 * M_PI, 0, 3, Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aFaceMaker.IsDone());
        TopoDS_Face aNurbsFace = TopoDS.Face(CanonicalRecognitionTestUtils.NurbsConverted(aFaceMaker.Face()));

        myRecognizer.SetShape(aNurbsFace);
        var aResultCone = new Cone();
        bool aResult = myRecognizer.IsCone(myTolerance, ref aResultCone);

        Assert.True(aResult, "NURBS face should be recognized as cone");
        Assert.Equal(aHalfAngle, aResultCone.SemiAngle(), myTolerance);
    }

    [Fact]
    public void SphericalSurfaceRecognition_A4()
    {
        var aAxis = new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z));
        var aSphereSurf = new Geom_SphericalSurface(new CoordinateSystem(aAxis), 1.0);
        var aFaceMaker = new BRepBuilderAPI_MakeFace(aSphereSurf, 0, 2 * M_PI, 0, M_PI / 2, Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aFaceMaker.IsDone());
        TopoDS_Face aNurbsFace = TopoDS.Face(CanonicalRecognitionTestUtils.NurbsConverted(aFaceMaker.Face()));

        myRecognizer.SetShape(aNurbsFace);
        var aResultSphere = new Sphere();
        bool aResult = myRecognizer.IsSphere(myTolerance, ref aResultSphere);

        Assert.True(aResult, "NURBS face should be recognized as sphere");
        Assert.Equal(1.0, aResultSphere.Radius(), myTolerance);
    }
}

/// <summary>Curve recognition tests (cr/base A-series).</summary>
public class CanonicalRecognitionBaseCurveTest
{
    private readonly ShapeAnalysis_CanonicalRecognition myRecognizer = new();
    private readonly double myTolerance = 1.0e-3;

    private static Geom_BezierCurve NearlyLinearBezier(double theDeviation)
    {
        var aControlPoints = new NCollection_Array1<Point3D>(1, 3);
        aControlPoints[1] = new Point3D(0, 0, 0);
        aControlPoints[2] = new Point3D(0.5, theDeviation, 0);
        aControlPoints[3] = new Point3D(1, 0, 0);
        return new Geom_BezierCurve(aControlPoints);
    }

    private static TopoDS_Wire ThreeEdgeWire(Geom_Curve theCurve, double theP1, double theP2, double theP3, double theP4)
    {
        var aEdgeMaker1 = new BRepBuilderAPI_MakeEdge(theCurve, theP1, theP2);
        var aEdgeMaker2 = new BRepBuilderAPI_MakeEdge(theCurve, theP2, theP3);
        var aEdgeMaker3 = new BRepBuilderAPI_MakeEdge(theCurve, theP3, theP4);
        Assert.True(aEdgeMaker1.IsDone() && aEdgeMaker2.IsDone() && aEdgeMaker3.IsDone(), "Failed to create edges");

        var aWireMaker = new BRepBuilderAPI_MakeWire();
        aWireMaker.Add(aEdgeMaker1.Edge());
        aWireMaker.Add(aEdgeMaker2.Edge());
        aWireMaker.Add(aEdgeMaker3.Edge());
        Assert.True(aWireMaker.IsDone(), "Failed to create wire");
        return aWireMaker.Wire();
    }

    [Fact]
    public void BezierToLineRecognition_A1()
    {
        var aEdgeMaker = new BRepBuilderAPI_MakeEdge(NearlyLinearBezier(0.0005));
        Assert.True(aEdgeMaker.IsDone());
        TopoDS_Edge anEdge = aEdgeMaker.Edge();

        myRecognizer.SetShape(anEdge);
        var aResultLine = new Line3D();
        bool aResult = myRecognizer.IsLine(myTolerance, ref aResultLine);

        Assert.True(aResult, "Nearly linear Bezier should be recognized as line");
        Direction3D aLineDir = aResultLine.Direction();
        Assert.Equal(1.0, Math.Abs(aLineDir.X()), myTolerance);
    }

    [Fact]
    public void BezierToCircleRecognition_A2()
    {
        var aEdgeMaker = new BRepBuilderAPI_MakeEdge(NearlyLinearBezier(0.0005));
        Assert.True(aEdgeMaker.IsDone());
        TopoDS_Edge anEdge = aEdgeMaker.Edge();

        myRecognizer.SetShape(anEdge);
        var aResultCircle = new Circle3D();
        bool aResult = myRecognizer.IsCircle(myTolerance, ref aResultCircle);

        Assert.True(aResult, "Bezier curve should be recognized as circle (matching original TCL test)");
        Assert.True(aResultCircle.Radius() > 0.0, "Circle radius should be positive");
    }

    [Fact]
    public void EllipseToEllipseRecognition_A3()
    {
        var aAxis = new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
        var aEllipse = new Geom_Ellipse(aAxis, 1.0, 0.5);
        Geom_BSplineCurve aBSplineCurve = GeomConvert.CurveToBSplineCurve(aEllipse);
        var aEdgeMaker = new BRepBuilderAPI_MakeEdge(aBSplineCurve);
        Assert.True(aEdgeMaker.IsDone());
        TopoDS_Edge anEdge = aEdgeMaker.Edge();

        myRecognizer.SetShape(anEdge);
        var aResultEllipse = new Ellipse3D();
        bool aResult = myRecognizer.IsEllipse(1.0e-7, ref aResultEllipse);

        Assert.True(aResult, "Ellipse should be recognized as ellipse");
        Assert.Equal(1.0, aResultEllipse.MajorRadius(), 1.0e-7);
        Assert.Equal(0.5, aResultEllipse.MinorRadius(), 1.0e-7);
    }

    [Fact]
    public void MultiSegmentWireToLineRecognition_A4()
    {
        TopoDS_Wire aWire = ThreeEdgeWire(NearlyLinearBezier(0.0000005), 0.0, 0.3, 0.7, 1.0);

        myRecognizer.SetShape(aWire);
        var aResultLine = new Line3D();
        bool aResult = myRecognizer.IsLine(myTolerance, ref aResultLine);

        Assert.True(aResult, "Multi-segment nearly linear wire should be recognized as line");
    }

    [Fact]
    public void MultiSegmentCircleWireRecognition_A5()
    {
        var aAxis = new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z));
        var aCircle = new Geom_Circle(aAxis, 1.0);
        Geom_BSplineCurve aBSplineCurve = GeomConvert.CurveToBSplineCurve(aCircle);
        TopoDS_Wire aWire = ThreeEdgeWire(aBSplineCurve, 0.0, 1.0, 2.5, 6.0);

        myRecognizer.SetShape(aWire);
        var aResultCircle = new Circle3D();
        bool aResult = myRecognizer.IsCircle(1.0e-7, ref aResultCircle);

        Assert.True(aResult, "Multi-segment circle wire should be recognized as circle");
        Assert.Equal(1.0, aResultCircle.Radius(), 1.0e-7);
    }

    [Fact]
    public void MultiSegmentEllipseWireRecognition_A6()
    {
        var aAxis = new Frame3D(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z), new Direction3D(Direction3D.D.X));
        var aEllipse = new Geom_Ellipse(aAxis, 1.0, 0.5);
        Geom_BSplineCurve aBSplineCurve = GeomConvert.CurveToBSplineCurve(aEllipse);
        TopoDS_Wire aWire = ThreeEdgeWire(aBSplineCurve, 0.0, 1.0, 2.5, 6.0);

        myRecognizer.SetShape(aWire);
        var aResultEllipse = new Ellipse3D();
        bool aResult = myRecognizer.IsEllipse(1.0e-7, ref aResultEllipse);

        Assert.True(aResult, "Multi-segment ellipse wire should be recognized as ellipse");
        Assert.Equal(1.0, aResultEllipse.MajorRadius(), 1.0e-7);
        Assert.Equal(0.5, aResultEllipse.MinorRadius(), 1.0e-7);
    }
}

/// <summary>Surface recognition tests (cr/base B-series).</summary>
public class CanonicalRecognitionBaseSurfaceTest
{
    private readonly ShapeAnalysis_CanonicalRecognition myRecognizer = new();
    private readonly double myTolerance = 1.0e-7;

    private static TopoDS_Face Face(Geom_Surface theSurf, double theU1, double theU2, double theV1, double theV2)
    {
        var aFaceMaker = new BRepBuilderAPI_MakeFace(theSurf, theU1, theU2, theV1, theV2, Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aFaceMaker.IsDone(), "Failed to create face");
        return aFaceMaker.Face();
    }

    [Fact]
    public void TrimmedPlaneRecognition_B1()
    {
        var aPlane = new Geom_Plane(new Plane3D());
        TopoDS_Face aNurbsFace = TopoDS.Face(CanonicalRecognitionTestUtils.NurbsConverted(Face(aPlane, -1, 1, -1, 1)));

        myRecognizer.SetShape(aNurbsFace);
        var aResultPlane = new Plane3D();
        bool aResult = myRecognizer.IsPlane(myTolerance, ref aResultPlane);

        Assert.True(aResult, "Planar face should be recognized as plane");
        Assert.Equal(1.0, Math.Abs(aResultPlane.Axis().Direction().Z()), myTolerance);
    }

    [Fact]
    public void TrimmedCylinderRecognition_B2()
    {
        var aCylSurf = new Geom_CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 1.0);
        TopoDS_Face aNurbsFace = TopoDS.Face(CanonicalRecognitionTestUtils.NurbsConverted(Face(aCylSurf, 0, 2 * M_PI, -1, 1)));

        myRecognizer.SetShape(aNurbsFace);
        var aResultCylinder = new Cylinder();
        bool aResult = myRecognizer.IsCylinder(myTolerance, ref aResultCylinder);

        Assert.True(aResult, "Cylindrical surface should be recognized as cylinder");
        Assert.Equal(1.0, aResultCylinder.Radius(), myTolerance);
    }

    [Fact]
    public void TrimmedConeRecognition_B3()
    {
        double aSemiAngle = M_PI / 6.0;
        var aConeSurf = new Geom_ConicalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), aSemiAngle, 0);
        TopoDS_Face aNurbsFace = TopoDS.Face(CanonicalRecognitionTestUtils.NurbsConverted(Face(aConeSurf, 0, 2 * M_PI, -1, 0)));

        myRecognizer.SetShape(aNurbsFace);
        var aResultCone = new Cone();
        bool aResult = myRecognizer.IsCone(myTolerance, ref aResultCone);

        Assert.True(aResult, "Conical surface should be recognized as cone");
        Assert.Equal(aSemiAngle, aResultCone.SemiAngle(), myTolerance);
    }

    [Fact]
    public void ConvertedSphereRecognition_B4()
    {
        var aSphereSurf = new Geom_SphericalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 1.0);
        Geom_BSplineSurface aBSplineSurf = GeomConvert.SurfaceToBSplineSurface(aSphereSurf);
        var aFaceMaker = new BRepBuilderAPI_MakeFace(aBSplineSurf, Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aFaceMaker.IsDone());
        TopoDS_Face aFace = aFaceMaker.Face();

        myRecognizer.SetShape(aFace);
        var aResultSphere = new Sphere();
        bool aResult = myRecognizer.IsSphere(myTolerance, ref aResultSphere);

        Assert.True(aResult, "Spherical surface should be recognized as sphere");
        Assert.Equal(1.0, aResultSphere.Radius(), myTolerance);
    }

    [Fact]
    public void SewnPlanarSurfaceRecognition_B5()
    {
        var aPlane = new Geom_Plane(new Plane3D());
        TopoDS_Shape aSewnShape = CanonicalRecognitionTestUtils.Sew(
            Face(aPlane, -1, 0, -1, 0), Face(aPlane, -1, 0, 0, 1), Face(aPlane, 0, 1, 0, 1), Face(aPlane, 0, 1, -1, 0));
        Assert.False(aSewnShape.IsNull());
        TopoDS_Shape aNurbsShape = CanonicalRecognitionTestUtils.NurbsConverted(aSewnShape);

        myRecognizer.SetShape(aNurbsShape);
        var aResultPlane = new Plane3D();
        bool aResult = myRecognizer.IsPlane(myTolerance, ref aResultPlane);

        Assert.True(aResult, "Sewn planar surface should be recognized as plane");
        Assert.Equal(1.0, Math.Abs(aResultPlane.Axis().Direction().Z()), myTolerance);
    }

    [Fact]
    public void SewnCylindricalSurfaceRecognition_B6()
    {
        var aCylSurf = new Geom_CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 1.0);
        TopoDS_Shape aSewnShape = CanonicalRecognitionTestUtils.Sew(
            Face(aCylSurf, 0, 3, -1, 0), Face(aCylSurf, 0, 3, 0, 1), Face(aCylSurf, 3, 6, 0, 1), Face(aCylSurf, 3, 6, -1, 0));
        TopoDS_Shape aNurbsShape = CanonicalRecognitionTestUtils.NurbsConverted(aSewnShape);

        myRecognizer.SetShape(aNurbsShape);
        var aResultCylinder = new Cylinder();
        bool aResult = myRecognizer.IsCylinder(myTolerance, ref aResultCylinder);

        Assert.True(aResult, "Sewn cylindrical surface should be recognized as cylinder");
        Assert.Equal(1.0, aResultCylinder.Radius(), myTolerance);
    }

    [Fact]
    public void SewnConicalSurfaceRecognition_B7()
    {
        double aSemiAngle = M_PI / 6.0;
        var aConeSurf = new Geom_ConicalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), aSemiAngle, 0);
        TopoDS_Shape aSewnShape = CanonicalRecognitionTestUtils.Sew(
            Face(aConeSurf, 0, 3, 0, 1), Face(aConeSurf, 0, 3, 1, 2), Face(aConeSurf, 3, 6, 1, 2), Face(aConeSurf, 3, 6, 0, 1));
        TopoDS_Shape aNurbsShape = CanonicalRecognitionTestUtils.NurbsConverted(aSewnShape);

        myRecognizer.SetShape(aNurbsShape);
        var aResultCone = new Cone();
        bool aResult = myRecognizer.IsCone(myTolerance, ref aResultCone);

        Assert.True(aResult, "Sewn conical surface should be recognized as cone");
        Assert.Equal(aSemiAngle, aResultCone.SemiAngle(), myTolerance);
    }

    [Fact]
    public void SewnSphericalSurfaceRecognition_B8()
    {
        var aSphereSurf = new Geom_SphericalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 1.0);
        Geom_BSplineSurface aBSplineSurf = GeomConvert.SurfaceToBSplineSurface(aSphereSurf);
        TopoDS_Shape aSewnShape = CanonicalRecognitionTestUtils.Sew(
            Face(aBSplineSurf, 0, 3, -1.5, 0), Face(aBSplineSurf, 0, 3, 0, 1.5), Face(aBSplineSurf, 3, 6, 0, 1.5),
            Face(aBSplineSurf, 3, 6, -1.5, 0));

        myRecognizer.SetShape(aSewnShape);
        var aResultSphere = new Sphere();
        bool aResult = myRecognizer.IsSphere(myTolerance, ref aResultSphere);

        Assert.True(aResult, "Sewn spherical surface should be recognized as sphere");
        Assert.Equal(1.0, aResultSphere.Radius(), myTolerance);
    }

    [Fact]
    public void ComplexCylindricalRecognitionWithSection_B9()
    {
        var aCylSurf = new Geom_CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 1.0);
        TopoDS_Shape aSewnShape = CanonicalRecognitionTestUtils.Sew(
            Face(aCylSurf, 0, 1, -1, 1), Face(aCylSurf, 1, 2, -1, 1), Face(aCylSurf, 2, 3, -1, 1), Face(aCylSurf, 3, 4, -1, 1));
        TopoDS_Shape aNurbsShape = CanonicalRecognitionTestUtils.NurbsConverted(aSewnShape);

        myRecognizer.SetShape(aNurbsShape);
        var aResultCylinder = new Cylinder();
        bool aResult = myRecognizer.IsCylinder(myTolerance, ref aResultCylinder);

        Assert.True(aResult, "Sewn cylindrical shape should be recognized as cylinder");
        Assert.Equal(1.0, aResultCylinder.Radius(), myTolerance);
    }

    [Fact]
    public void ExtrudedCylindricalSurfaceRecognition_B10()
    {
        var aCylSurf = new Geom_CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(Direction3D.D.Z)), 1.0);
        var aGeomPlane = new Geom_Plane(new Plane3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 1)));

        var anIntersector = new GeomAPI_IntSS(aCylSurf, aGeomPlane, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(anIntersector.IsDone(), "Surface-surface intersection failed");
        Assert.True(anIntersector.NbLines() > 0, "No intersection curves found");
        Geom_Curve anIntCurve = anIntersector.Line(1);
        Assert.NotNull(anIntCurve);

        var anExtSurf = new Geom_SurfaceOfLinearExtrusion(anIntCurve, new Direction3D(Direction3D.D.Z));
        anExtSurf.Bounds(out double uMin, out double uMax, out double _, out double _);
        TopoDS_Face aNurbsFace = TopoDS.Face(CanonicalRecognitionTestUtils.NurbsConverted(Face(anExtSurf, uMin, uMax, -1, 1)));

        myRecognizer.SetShape(aNurbsFace);
        var aResultCylinder = new Cylinder();
        bool aResult = myRecognizer.IsCylinder(myTolerance, ref aResultCylinder);

        Assert.True(aResult, "Extruded surface should be recognized as cylinder");
        Assert.Equal(1.0, aResultCylinder.Radius(), myTolerance);
    }

    [Fact]
    public void PlaneDetectionWithGapValidation_Bug33170()
    {
        var aBasePlane = new Geom_Plane(new Plane3D());
        TopoDS_Face aFace = Face(aBasePlane, -1, 1, -1, 1);

        myRecognizer.SetShape(aFace);
        var aResultPlane1 = new Plane3D();
        bool aResult1 = myRecognizer.IsPlane(0.006, ref aResultPlane1);
        Assert.True(aResult1, "Surface should be recognized as plane with tolerance 0.006");
        double aGap1 = myRecognizer.GetGap();
        Assert.True(aGap1 < 0.006);
        Assert.True(aGap1 >= 0.0);

        myRecognizer.ClearStatus();
        var aResultPlane2 = new Plane3D();
        bool aResult2 = myRecognizer.IsPlane(1.0, ref aResultPlane2);
        Assert.True(aResult2, "Surface should be recognized as plane with tolerance 1.0");
        double aGap2 = myRecognizer.GetGap();
        Assert.True(aGap2 < 1.0);
        Assert.True(aGap2 >= 0.0);

        Assert.Equal(aGap1, aGap2);
    }
}
