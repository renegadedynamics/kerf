// Port of: src/ModelingData/TKG3d/GTests/GeomHash_MeshHasher_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class GeomHash_MeshHasherTest
{
    private static Polygon2D makePolygon2D(double theDeflection)
    {
        Polygon2D aPoly = new Polygon2D(2);
        aPoly.ChangeNodes().SetValue(1, new Point2D(0.0, 0.0));
        aPoly.ChangeNodes().SetValue(2, new Point2D(1.0, 0.0));
        aPoly.Deflection(theDeflection);
        return aPoly;
    }

    private static Polygon3D makePolygon3D(double theDeflection)
    {
        Polygon3D aPoly = new Polygon3D(2, false);
        aPoly.ChangeNodes().SetValue(1, new Point3D(0.0, 0.0, 0.0));
        aPoly.ChangeNodes().SetValue(2, new Point3D(1.0, 0.0, 0.0));
        aPoly.Deflection(theDeflection);
        return aPoly;
    }

    private static PolygonOnTriangulation makePolygonOnTriangulation(double theDeflection)
    {
        PolygonOnTriangulation aPoly = new PolygonOnTriangulation(2, false);
        aPoly.SetNode(1, 1);
        aPoly.SetNode(2, 2);
        aPoly.Deflection(theDeflection);
        return aPoly;
    }

    private static Triangulation makeTriangulation(double theDeflection)
    {
        Triangulation aTri = new Triangulation(3, 1, false);
        aTri.SetNode(1, new Point3D(0.0, 0.0, 0.0));
        aTri.SetNode(2, new Point3D(1.0, 0.0, 0.0));
        aTri.SetNode(3, new Point3D(0.0, 1.0, 0.0));
        aTri.SetTriangle(1, new Triangle1(1, 2, 3));
        aTri.Deflection(theDeflection);
        return aTri;
    }

    [Fact]
    public void Polygon2D_CloseDeflectionKeepsEqualHash()
    {
        var aHasher = new global::Kerf.Engine.Geometry.Hashing.Polygon2DHasher(0.1, 0.01);
        Polygon2D aPoly1 = makePolygon2D(0.004);
        Polygon2D aPoly2 = makePolygon2D(0.0044);

        Assert.True(aHasher.Equals(aPoly1, aPoly2));
        Assert.Equal(aHasher.GetHashCode(aPoly1), aHasher.GetHashCode(aPoly2));
    }

    [Fact]
    public void Polygon3D_CloseDeflectionKeepsEqualHash()
    {
        var aHasher = new global::Kerf.Engine.Geometry.Hashing.Polygon3DHasher(0.1, 0.01);
        Polygon3D aPoly1 = makePolygon3D(0.004);
        Polygon3D aPoly2 = makePolygon3D(0.0044);

        Assert.True(aHasher.Equals(aPoly1, aPoly2));
        Assert.Equal(aHasher.GetHashCode(aPoly1), aHasher.GetHashCode(aPoly2));
    }

    [Fact]
    public void PolygonOnTriangulation_CloseDeflectionKeepsEqualHash()
    {
        var aHasher = new global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHasher(0.1, 0.01);
        var aKey1 = new global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHashKey(makePolygonOnTriangulation(0.004), 7);
        var aKey2 = new global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHashKey(makePolygonOnTriangulation(0.0044), 7);

        Assert.True(aHasher.Equals(aKey1, aKey2));
        Assert.Equal(aHasher.GetHashCode(aKey1), aHasher.GetHashCode(aKey2));
    }

    [Fact]
    public void Triangulation_CloseDeflectionKeepsEqualHash()
    {
        var aHasher = new global::Kerf.Engine.Geometry.Hashing.TriangulationHasher(0.1, 0.01);
        Triangulation aTri1 = makeTriangulation(0.004);
        Triangulation aTri2 = makeTriangulation(0.0044);

        Assert.True(aHasher.Equals(aTri1, aTri2));
        Assert.Equal(aHasher.GetHashCode(aTri1), aHasher.GetHashCode(aTri2));
    }

    [Fact]
    public void Polygon2D_DifferentSameCountGeometryChangesHash()
    {
        var aHasher = new global::Kerf.Engine.Geometry.Hashing.Polygon2DHasher(0.1, 0.01);
        Polygon2D aPoly1 = makePolygon2D(0.004);
        Polygon2D aPoly2 = makePolygon2D(0.004);
        aPoly2.ChangeNodes().SetValue(2, new Point2D(2.0, 0.0));

        Assert.False(aHasher.Equals(aPoly1, aPoly2));
        Assert.NotEqual(aHasher.GetHashCode(aPoly1), aHasher.GetHashCode(aPoly2));
    }

    [Fact]
    public void Polygon3D_DifferentSameCountGeometryChangesHash()
    {
        var aHasher = new global::Kerf.Engine.Geometry.Hashing.Polygon3DHasher(0.1, 0.01);
        Polygon3D aPoly1 = makePolygon3D(0.004);
        Polygon3D aPoly2 = makePolygon3D(0.004);
        aPoly2.ChangeNodes().SetValue(2, new Point3D(2.0, 0.0, 0.0));

        Assert.False(aHasher.Equals(aPoly1, aPoly2));
        Assert.NotEqual(aHasher.GetHashCode(aPoly1), aHasher.GetHashCode(aPoly2));
    }

    [Fact]
    public void PolygonOnTriangulation_DifferentSameCountIndicesChangesHash()
    {
        var aHasher = new global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHasher(0.1, 0.01);
        var aKey1 = new global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHashKey(makePolygonOnTriangulation(0.004), 7);
        var aKey2 = new global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHashKey(makePolygonOnTriangulation(0.004), 7);
        aKey2.Poly!.SetNode(2, 3);

        Assert.False(aHasher.Equals(aKey1, aKey2));
        Assert.NotEqual(aHasher.GetHashCode(aKey1), aHasher.GetHashCode(aKey2));
    }

    [Fact]
    public void Triangulation_DifferentSameCountGeometryChangesHash()
    {
        var aHasher = new global::Kerf.Engine.Geometry.Hashing.TriangulationHasher(0.1, 0.01);
        Triangulation aTri1 = makeTriangulation(0.004);
        Triangulation aTri2 = makeTriangulation(0.004);
        aTri2.SetNode(3, new Point3D(0.0, 2.0, 0.0));

        Assert.False(aHasher.Equals(aTri1, aTri2));
        Assert.NotEqual(aHasher.GetHashCode(aTri1), aHasher.GetHashCode(aTri2));
    }

    [Fact]
    public void NullHandlesDoNotCrash()
    {
        var aPoly2dHasher = new global::Kerf.Engine.Geometry.Hashing.Polygon2DHasher();
        var aPoly3dHasher = new global::Kerf.Engine.Geometry.Hashing.Polygon3DHasher();
        var aPolyOnTriHasher = new global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHasher();
        var aTriHasher = new global::Kerf.Engine.Geometry.Hashing.TriangulationHasher();
        Polygon2D?              aNullPoly2d = null;
        Polygon3D?              aNullPoly3d = null;
        PolygonOnTriangulation? aNullPolyOnTri = null;
        Triangulation?          aNullTri = null;

        Assert.Equal(0, aPoly2dHasher.GetHashCode(aNullPoly2d));
        Assert.True(aPoly2dHasher.Equals(aNullPoly2d, aNullPoly2d));
        Assert.False(aPoly2dHasher.Equals(aNullPoly2d, makePolygon2D(0.004)));

        Assert.Equal(0, aPoly3dHasher.GetHashCode(aNullPoly3d));
        Assert.True(aPoly3dHasher.Equals(aNullPoly3d, aNullPoly3d));
        Assert.False(aPoly3dHasher.Equals(aNullPoly3d, makePolygon3D(0.004)));

        Assert.Equal(0, aTriHasher.GetHashCode(aNullTri));
        Assert.True(aTriHasher.Equals(aNullTri, aNullTri));
        Assert.False(aTriHasher.Equals(aNullTri, makeTriangulation(0.004)));

        var aNullKey1 = new global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHashKey(aNullPolyOnTri, 7);
        var aNullKey2 = new global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHashKey(aNullPolyOnTri, 7);
        var aNullKey3 = new global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHashKey(aNullPolyOnTri, 8);
        Assert.True(aPolyOnTriHasher.Equals(aNullKey1, aNullKey2));
        Assert.False(aPolyOnTriHasher.Equals(aNullKey1, aNullKey3));
        Assert.False(aPolyOnTriHasher.Equals(aNullKey1, new global::Kerf.Engine.Geometry.Hashing.PolygonOnTriHashKey(makePolygonOnTriangulation(0.004), 7)));
    }

    [Fact]
    public void HashToleranceAffectsNumericFields()
    {
        Polygon2D aPoly = makePolygon2D(0.06);

        var aFineHasher = new global::Kerf.Engine.Geometry.Hashing.Polygon2DHasher(0.1, 0.01);
        var aCoarseHasher = new global::Kerf.Engine.Geometry.Hashing.Polygon2DHasher(0.1, 0.1);

        Assert.NotEqual(aFineHasher.GetHashCode(aPoly), aCoarseHasher.GetHashCode(aPoly));
    }
}
