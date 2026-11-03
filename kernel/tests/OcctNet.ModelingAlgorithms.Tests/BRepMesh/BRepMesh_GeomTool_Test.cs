// Port of: src/ModelingAlgorithms/TKMesh/GTests/BRepMesh_GeomTool_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

public class BRepMesh_GeomTool_Test
{
    [Fact]
    public void OCC25547_StaticMethodsExportAndFunctionality()
    {
        // Bug OCC25547: static class methods not exported in BRepMesh_GeomTool
        // This test verifies that BRepMesh_GeomTool static methods are properly exported
        // and functional

        // Test 1: Discretize an arc
        double aFirstP = 0.0, aLastP = Math.PI;
        Geom_Circle aCircle = new(new Frame3D(Constants.Origin(), Constants.DZ()), 10);
        Geom_TrimmedCurve aHalf = new(aCircle, aFirstP, aLastP);
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(aHalf).Edge();
        BRepAdaptor_Curve anAdaptor = new(anEdge);
        BRepMesh_GeomTool aGeomTool = new(anAdaptor, aFirstP, aLastP, 0.1, 0.5);

        Assert.True(aGeomTool.NbPoints() > 0, "BRepMesh_GeomTool failed to discretize an arc");

        // Test 2: Test Normal() static method
        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(new Plane3D(Constants.Origin(), Constants.DZ())).Face();
        BRepAdaptor_Surface aSurf = new(aFace);
        BRepAdaptor_Surface aHSurf = (BRepAdaptor_Surface)aSurf.ShallowCopy();

        bool isNormalComputed = BRepMesh_GeomTool.Normal(aHSurf, 10.0, 10.0, out Point3D _, out Direction3D _);

        Assert.True(isNormalComputed, "BRepMesh_GeomTool failed to compute a normal of surface");

        // Test 3: Test IntLinLin() static method - line-line intersection
        Xy[] aRefPnts = [new(-10.0, -10.0), new(10.0, 10.0), new(-10.0, 10.0), new(10.0, -10.0)];

        Point2D anIntPnt = new();
        Span<double> aParams = stackalloc double[2];
        BRepMesh_GeomTool.IntFlag anIntFlag = BRepMesh_GeomTool.IntLinLin(aRefPnts[0],
                                                                          aRefPnts[1],
                                                                          aRefPnts[2],
                                                                          aRefPnts[3],
                                                                          ref anIntPnt.ChangeCoord(),
                                                                          aParams);

        double aDiff = anIntPnt.Distance(Constants.Origin2d());
        Assert.Equal(BRepMesh_GeomTool.IntFlag.Cross, anIntFlag);
        Assert.True(aDiff <= global::Kerf.Engine.Foundation.Precision.PConfusion(), "BRepMesh_GeomTool failed to intersect two lines at origin");

        // Test 4: Test IntSegSeg() static method - segment-segment intersection
        anIntFlag = BRepMesh_GeomTool.IntSegSeg(aRefPnts[0],
                                                aRefPnts[1],
                                                aRefPnts[2],
                                                aRefPnts[3],
                                                false,
                                                false,
                                                ref anIntPnt);

        aDiff = anIntPnt.Distance(Constants.Origin2d());
        Assert.Equal(BRepMesh_GeomTool.IntFlag.Cross, anIntFlag);
        Assert.True(aDiff <= global::Kerf.Engine.Foundation.Precision.PConfusion(), "BRepMesh_GeomTool failed to intersect two segments at origin");
    }
}
