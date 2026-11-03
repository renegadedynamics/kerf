// Port of: src/FoundationClasses/TKMath/GTests/BVH_Triangulation_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bvh;
using Box3d = Kerf.Engine.Geometry.Bvh.Box<double, Occt.NCollection_Vec3<double>>;
using Tri3d = Kerf.Engine.Geometry.Bvh.Triangulation<double, Occt.NCollection_Vec3<double>>;

namespace OcctNet.Foundation.Tests;

// OCCT's BVH::Array<T, N>::Append/Size are the vertex and element arrays' own Append/Size.
public class BVH_TriangulationTest
{
    // Basic tests

    [Fact]
    public void DefaultConstructor()
    {
        Tri3d aTriangulation = new();

        Assert.Equal(0, aTriangulation.Size());
        Assert.Equal(0, aTriangulation.Vertices.Size());
        Assert.Equal(0, aTriangulation.Elements.Size());
    }

    [Fact]
    public void AddSingleTriangle()
    {
        Tri3d aTriangulation = new();

        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 1.0, 0.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        Assert.Equal(1, aTriangulation.Size());
        Assert.Equal(3, aTriangulation.Vertices.Size());
    }

    [Fact]
    public void AddMultipleTriangles()
    {
        Tri3d aTriangulation = new();

        for (int i = 0; i < 5; ++i)
        {
            double x = i;
            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 1.0, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 1.0, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        Assert.Equal(5, aTriangulation.Size());
        Assert.Equal(15, aTriangulation.Vertices.Size());
    }

    // Box tests

    [Fact]
    public void BoxForSingleTriangle()
    {
        Tri3d aTriangulation = new();

        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 2.0, 3.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(4.0, 1.0, 2.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(2.0, 3.0, 1.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        Box3d aBox = aTriangulation.Box(0);

        Assert.Equal(1.0, aBox.CornerMin().X(), 1e-10);
        Assert.Equal(1.0, aBox.CornerMin().Y(), 1e-10);
        Assert.Equal(1.0, aBox.CornerMin().Z(), 1e-10);

        Assert.Equal(4.0, aBox.CornerMax().X(), 1e-10);
        Assert.Equal(3.0, aBox.CornerMax().Y(), 1e-10);
        Assert.Equal(3.0, aBox.CornerMax().Z(), 1e-10);
    }

    [Fact]
    public void BoxForDegenerateTriangle()
    {
        Tri3d aTriangulation = new();

        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 1.0, 1.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 1.0, 1.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 1.0, 1.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        Box3d aBox = aTriangulation.Box(0);

        Assert.Equal(1.0, aBox.CornerMin().X(), 1e-10);
        Assert.Equal(1.0, aBox.CornerMin().Y(), 1e-10);
        Assert.Equal(1.0, aBox.CornerMin().Z(), 1e-10);
        Assert.Equal(1.0, aBox.CornerMax().X(), 1e-10);
        Assert.Equal(1.0, aBox.CornerMax().Y(), 1e-10);
        Assert.Equal(1.0, aBox.CornerMax().Z(), 1e-10);
    }

    [Fact]
    public void BoxForFlatTriangle()
    {
        Tri3d aTriangulation = new();

        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(2.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 3.0, 0.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        Box3d aBox = aTriangulation.Box(0);

        Assert.Equal(0.0, aBox.CornerMin().Z(), 1e-10);
        Assert.Equal(0.0, aBox.CornerMax().Z(), 1e-10);
        Assert.Equal(2.0, aBox.CornerMax().X(), 1e-10);
        Assert.Equal(3.0, aBox.CornerMax().Y(), 1e-10);
    }

    // Center tests

    [Fact]
    public void CenterComputation()
    {
        Tri3d aTriangulation = new();

        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(3.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 3.0, 0.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        Assert.Equal(1.0, aTriangulation.Center(0, 0), 1e-10);
        Assert.Equal(1.0, aTriangulation.Center(0, 1), 1e-10);
        Assert.Equal(0.0, aTriangulation.Center(0, 2), 1e-10);
    }

    [Fact]
    public void CenterAlongXAxis()
    {
        Tri3d aTriangulation = new();

        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(2.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(3.0, 0.0, 0.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        Assert.Equal(2.0, aTriangulation.Center(0, 0), 1e-10);
    }

    [Fact]
    public void CenterAlongYAxis()
    {
        Tri3d aTriangulation = new();

        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 2.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 4.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 6.0, 0.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        Assert.Equal(4.0, aTriangulation.Center(0, 1), 1e-10);
    }

    [Fact]
    public void CenterAlongZAxis()
    {
        Tri3d aTriangulation = new();

        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 1.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 4.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 7.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        Assert.Equal(4.0, aTriangulation.Center(0, 2), 1e-10);
    }

    // Swap tests

    [Fact]
    public void SwapTwoTriangles()
    {
        Tri3d aTriangulation = new();

        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 1.0, 0.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        aTriangulation.Vertices.Append(new BVH_Vec3d(5.0, 5.0, 5.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(6.0, 5.0, 5.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(5.0, 6.0, 5.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(3, 4, 5, 0));

        double aCentroid0Before = aTriangulation.Center(0, 0);
        double aCentroid1Before = aTriangulation.Center(1, 0);

        aTriangulation.Swap(0, 1);

        Assert.Equal(aCentroid1Before, aTriangulation.Center(0, 0), 1e-10);
        Assert.Equal(aCentroid0Before, aTriangulation.Center(1, 0), 1e-10);
    }

    [Fact]
    public void SwapPreservesVertices()
    {
        Tri3d aTriangulation = new();

        for (int i = 0; i < 3; ++i)
        {
            double x = i * 10.0;
            aTriangulation.Vertices.Append(new BVH_Vec3d(x, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 1.0, 0.0, 0.0));
            aTriangulation.Vertices.Append(new BVH_Vec3d(x + 0.5, 1.0, 0.0));
            aTriangulation.Elements.Append(new BVH_Vec4i(i * 3, i * 3 + 1, i * 3 + 2, 0));
        }

        int aVertexCount = aTriangulation.Vertices.Size();

        aTriangulation.Swap(0, 2);

        Assert.Equal(aVertexCount, aTriangulation.Vertices.Size());
    }

    // 2D tests

    [Fact]
    public void Triangulation2D()
    {
        Triangulation<double, BVH_Vec2d> aTriangulation2D = new();

        aTriangulation2D.Vertices.Append(new BVH_Vec2d(0.0, 0.0));
        aTriangulation2D.Vertices.Append(new BVH_Vec2d(1.0, 0.0));
        aTriangulation2D.Vertices.Append(new BVH_Vec2d(0.5, 1.0));
        aTriangulation2D.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        Assert.Equal(1, aTriangulation2D.Size());

        Box<double, BVH_Vec2d> aBox = aTriangulation2D.Box(0);
        Assert.Equal(0.0, aBox.CornerMin().X(), 1e-10);
        Assert.Equal(0.0, aBox.CornerMin().Y(), 1e-10);
        Assert.Equal(1.0, aBox.CornerMax().X(), 1e-10);
        Assert.Equal(1.0, aBox.CornerMax().Y(), 1e-10);
    }

    // Shared vertices tests

    [Fact]
    public void SharedVertices()
    {
        Tri3d aTriangulation = new();

        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 0.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, 1.0, 0.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 1.0, 0.0));

        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 2, 3, 0));

        Assert.Equal(2, aTriangulation.Size());
        Assert.Equal(4, aTriangulation.Vertices.Size());

        Box3d aBox0 = aTriangulation.Box(0);
        Box3d aBox1 = aTriangulation.Box(1);

        Assert.False(aBox0.IsOut(aBox1)); // Should overlap
    }

    // Float precision tests

    [Fact]
    public void FloatPrecision()
    {
        Triangulation<float, BVH_Vec3f> aTriangulation = new();

        aTriangulation.Vertices.Append(new BVH_Vec3f(0.0f, 0.0f, 0.0f));
        aTriangulation.Vertices.Append(new BVH_Vec3f(1.0f, 0.0f, 0.0f));
        aTriangulation.Vertices.Append(new BVH_Vec3f(0.0f, 1.0f, 0.0f));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        Assert.Equal(1, aTriangulation.Size());

        Box<float, BVH_Vec3f> aBox = aTriangulation.Box(0);
        Assert.Equal(1.0f, aBox.CornerMax().X(), 1e-6f);
        Assert.Equal(1.0f, aBox.CornerMax().Y(), 1e-6f);
    }

    // Negative coordinates tests

    [Fact]
    public void NegativeCoordinates()
    {
        Tri3d aTriangulation = new();

        aTriangulation.Vertices.Append(new BVH_Vec3d(-1.0, -2.0, -3.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(1.0, -1.0, -2.0));
        aTriangulation.Vertices.Append(new BVH_Vec3d(0.0, 0.0, -1.0));
        aTriangulation.Elements.Append(new BVH_Vec4i(0, 1, 2, 0));

        Box3d aBox = aTriangulation.Box(0);

        Assert.Equal(-1.0, aBox.CornerMin().X(), 1e-10);
        Assert.Equal(-2.0, aBox.CornerMin().Y(), 1e-10);
        Assert.Equal(-3.0, aBox.CornerMin().Z(), 1e-10);
        Assert.Equal(1.0, aBox.CornerMax().X(), 1e-10);
        Assert.Equal(0.0, aBox.CornerMax().Y(), 1e-10);
        Assert.Equal(-1.0, aBox.CornerMax().Z(), 1e-10);
    }
}
