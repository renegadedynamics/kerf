// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Poly_MeshPurpose;

namespace OcctNet.Foundation.Tests;

/// <summary>Poly_Triangle, Poly_ArrayOfNodes and Poly_Triangulation (OCCT has no GTests for the Poly package).</summary>
public class Poly_TriangulationTest
{
    /// <summary>A unit square in z = 0 as two triangles, (1, 2, 3) and (1, 3, 4), counterclockwise seen from +Z.</summary>
    internal static Poly_Triangulation Square()
    {
        var aTri = new Poly_Triangulation(4, 2, true);
        aTri.SetNode(1, new gp_Pnt(0, 0, 0));
        aTri.SetNode(2, new gp_Pnt(1, 0, 0));
        aTri.SetNode(3, new gp_Pnt(1, 1, 0));
        aTri.SetNode(4, new gp_Pnt(0, 1, 0));
        aTri.SetUVNode(1, new gp_Pnt2d(0, 0));
        aTri.SetUVNode(2, new gp_Pnt2d(1, 0));
        aTri.SetUVNode(3, new gp_Pnt2d(1, 1));
        aTri.SetUVNode(4, new gp_Pnt2d(0, 1));
        aTri.SetTriangle(1, new Poly_Triangle(1, 2, 3));
        aTri.SetTriangle(2, new Poly_Triangle(1, 3, 4));
        return aTri;
    }

    /// <summary>A regular tetrahedron's surface, outward oriented.</summary>
    internal static Poly_Triangulation Tetrahedron()
    {
        var aTri = new Poly_Triangulation(4, 4, false);
        aTri.SetNode(1, new gp_Pnt(1, 1, 1));
        aTri.SetNode(2, new gp_Pnt(1, -1, -1));
        aTri.SetNode(3, new gp_Pnt(-1, 1, -1));
        aTri.SetNode(4, new gp_Pnt(-1, -1, 1));
        aTri.SetTriangle(1, new Poly_Triangle(1, 2, 3));
        aTri.SetTriangle(2, new Poly_Triangle(1, 4, 2));
        aTri.SetTriangle(3, new Poly_Triangle(1, 3, 4));
        aTri.SetTriangle(4, new Poly_Triangle(2, 4, 3));
        return aTri;
    }

    [Fact]
    public void Triangle_ConstructionAndAccess()
    {
        var aDefault = new Poly_Triangle();
        aDefault.Get(out int n1, out int n2, out int n3);
        Assert.Equal((0, 0, 0), (n1, n2, n3));

        var aTri = new Poly_Triangle(4, 5, 6);
        Assert.Equal(4, aTri.Value(1));
        Assert.Equal(5, aTri[2]);
        Assert.Equal(6, aTri.Value(3));
        Assert.ThrowsAny<Standard_OutOfRange>(() => aTri.Value(0));
        Assert.ThrowsAny<Standard_OutOfRange>(() => aTri.Value(4));

        Poly_Triangle aChanged = aTri.WithNode(2, 9);
        Assert.Equal(new Poly_Triangle(4, 9, 6), aChanged);
        Assert.Equal(new Poly_Triangle(4, 5, 6), aTri);
        Assert.ThrowsAny<Standard_OutOfRange>(() => aTri.WithNode(4, 1));

        aTri.Set(1, 2, 3);
        aTri.ChangeValue(3) = 7;
        Assert.Equal(new Poly_Triangle(1, 2, 7), aTri);
        Assert.True(aTri != aChanged);
        Assert.Equal("Poly_Triangle(1, 2, 7)", aTri.ToString());
    }

    [Fact]
    public void Construction_Sizes()
    {
        var anEmpty = new Poly_Triangulation();
        Assert.Equal(0, anEmpty.NbNodes());
        Assert.Equal(0, anEmpty.NbTriangles());
        Assert.False(anEmpty.HasGeometry());
        Assert.False(anEmpty.HasUVNodes());
        Assert.False(anEmpty.HasNormals());
        Assert.True(anEmpty.IsDoublePrecision());
        Assert.Equal(Poly_MeshPurpose_NONE, anEmpty.MeshPurpose());

        var aTri = new Poly_Triangulation(5, 3, true, true);
        Assert.Equal(5, aTri.NbNodes());
        Assert.Equal(3, aTri.NbTriangles());
        Assert.True(aTri.HasUVNodes());
        Assert.True(aTri.HasNormals());
        Assert.True(aTri.HasGeometry());
        Assert.Equal(new gp_Pnt(0, 0, 0), aTri.Node(5));
        Assert.Equal(new Poly_Triangle(), aTri.Triangle(3));

        // as in OCCT, an array of nodes can't be allocated empty
        Assert.ThrowsAny<Standard_RangeError>(() => new Poly_Triangulation(0, 0, false));
    }

    [Fact]
    public void Construction_FromArrays()
    {
        var aNodes = new NCollection_Array1<gp_Pnt>(1, 3);
        aNodes[1] = new gp_Pnt(0, 0, 0);
        aNodes[2] = new gp_Pnt(1, 0, 0);
        aNodes[3] = new gp_Pnt(0, 1, 0);
        var aUV = new NCollection_Array1<gp_Pnt2d>(1, 3);
        aUV[2] = new gp_Pnt2d(1, 0);
        var aTris = new NCollection_Array1<Poly_Triangle>(1, 1);
        aTris[1] = new Poly_Triangle(1, 2, 3);

        var aTri = new Poly_Triangulation(aNodes, aTris);
        Assert.Equal(3, aTri.NbNodes());
        Assert.Equal(new gp_Pnt(1, 0, 0), aTri.Node(2));
        Assert.False(aTri.HasUVNodes());

        // the data are copied
        aNodes[2] = new gp_Pnt(5, 5, 5);
        Assert.Equal(new gp_Pnt(1, 0, 0), aTri.Node(2));

        var aTriUV = new Poly_Triangulation(aNodes, aUV, aTris);
        Assert.True(aTriUV.HasUVNodes());
        Assert.Equal(new gp_Pnt2d(1, 0), aTriUV.UVNode(2));
        Assert.Equal(new Poly_Triangle(1, 2, 3), aTriUV.Triangle(1));
    }

    [Fact]
    public void Construction_TrianglesKeepTheirBounds()
    {
        // OCCT: the triangle table takes the bounds of the array given (NCollection_Array1::operator=)
        var aNodes = new NCollection_Array1<gp_Pnt>(1, 3);
        var aTris = new NCollection_Array1<Poly_Triangle>(0, 0);
        aTris[0] = new Poly_Triangle(1, 2, 3);
        var aTri = new Poly_Triangulation(aNodes, aTris);
        Assert.Equal(new Poly_Triangle(1, 2, 3), aTri.Triangle(0));
        Assert.ThrowsAny<Standard_OutOfRange>(() => aTri.Triangle(1));
    }

    [Fact]
    public void NodeAndTriangleAccess()
    {
        Poly_Triangulation aTri = Square();
        Assert.Equal(new gp_Pnt(1, 1, 0), aTri.Node(3));
        Assert.Equal(new gp_Pnt2d(0, 1), aTri.UVNode(4));
        Assert.Equal(new Poly_Triangle(1, 3, 4), aTri.Triangle(2));
        Assert.ThrowsAny<Standard_OutOfRange>(() => aTri.Node(0));
        Assert.ThrowsAny<Standard_OutOfRange>(() => aTri.Node(5));
        Assert.ThrowsAny<Standard_OutOfRange>(() => aTri.Triangle(3));

        Assert.Equal(new double[] { 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0 }, aTri.NodeCoordinates().ToArray());
        Assert.Equal(new[] { 1, 2, 3, 1, 3, 4 }, aTri.TriangleIndices().ToArray());
        Assert.Equal(new double[] { 0, 0, 1, 0, 1, 1, 0, 1 }, aTri.UVNodeCoordinates().ToArray());
        Assert.Equal(2, aTri.TriangleSpan().Length);
        Assert.True(aTri.NormalComponents().IsEmpty);
        Assert.ThrowsAny<Standard_TypeMismatch>(() => aTri.NodeCoordinatesF().Length);

        var aFloats = new float[12];
        aTri.CopyNodeCoordinatesTo(aFloats);
        Assert.Equal(new float[] { 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0 }, aFloats);
    }

    [Fact]
    public void Normals_SetAndGet()
    {
        Poly_Triangulation aTri = Square();
        Assert.False(aTri.HasNormals());
        aTri.AddNormals();
        Assert.True(aTri.HasNormals());
        aTri.SetNormal(1, new gp_Dir(0, 0, 1));
        aTri.SetNormal(2, new NCollection_Vec3<float>(0.0f, 1.0f, 0.0f));
        Assert.Equal(new gp_Dir(0, 0, 1), aTri.Normal(1));
        aTri.Normal(2, out NCollection_Vec3<float> aVec);
        Assert.Equal(new NCollection_Vec3<float>(0.0f, 1.0f, 0.0f), aVec);
        Assert.Equal(12, aTri.NormalComponents().Length);
        aTri.RemoveNormals();
        Assert.False(aTri.HasNormals());
    }

    [Fact]
    public void ComputeNormals_FlatSquare()
    {
        Poly_Triangulation aTri = Square();
        aTri.ComputeNormals();
        Assert.True(aTri.HasNormals());
        for (int i = 1; i <= 4; i++)
        {
            Assert.Equal(new gp_Dir(0, 0, 1), aTri.Normal(i));
        }
    }

    [Fact]
    public void ComputeNormals_Tetrahedron()
    {
        Poly_Triangulation aTri = Tetrahedron();
        Poly.ComputeNormals(aTri);
        for (int i = 1; i <= 4; i++)
        {
            // each vertex normal points away from the centre, along the vertex
            gp_Dir anExpected = new(aTri.Node(i).XYZ());
            Assert.True(aTri.Normal(i).IsEqual(anExpected, 1e-6), $"node {i}");
        }
    }

    [Fact]
    public void ComputeNormals_IsolatedNodeGetsZ()
    {
        var aTri = new Poly_Triangulation(4, 1, false);
        aTri.SetNode(1, new gp_Pnt(0, 0, 0));
        aTri.SetNode(2, new gp_Pnt(0, 1, 0));
        aTri.SetNode(3, new gp_Pnt(0, 0, 1));
        aTri.SetNode(4, new gp_Pnt(5, 5, 5));
        aTri.SetTriangle(1, new Poly_Triangle(1, 2, 3));
        aTri.ComputeNormals();
        Assert.Equal(new gp_Dir(1, 0, 0), aTri.Normal(1));
        Assert.Equal(new gp_Dir(0, 0, 1), aTri.Normal(4));
    }

    [Fact]
    public void Copy_IsDeep()
    {
        Poly_Triangulation aTri = Square();
        aTri.Deflection(0.25);
        aTri.Parameters(new Poly_TriangulationParameters(0.1, 0.5));
        aTri.ComputeNormals();
        aTri.UpdateCachedMinMax();
        aTri.SetMeshPurpose(Poly_MeshPurpose_Presentation);

        Poly_Triangulation aCopy = aTri.Copy();
        Assert.Equal(0.25, aCopy.Deflection());
        Assert.Equal(4, aCopy.NbNodes());
        Assert.Equal(2, aCopy.NbTriangles());
        Assert.True(aCopy.HasUVNodes());
        Assert.True(aCopy.HasNormals());
        Assert.True(aCopy.HasCachedMinMax());
        Assert.Equal(Poly_MeshPurpose_Presentation, aCopy.MeshPurpose());
        Assert.NotSame(aTri.Parameters(), aCopy.Parameters());
        Assert.Equal(0.5, aCopy.Parameters()!.Angle());

        aTri.SetNode(1, new gp_Pnt(9, 9, 9));
        aTri.SetTriangle(1, new Poly_Triangle(4, 3, 2));
        Assert.Equal(new gp_Pnt(0, 0, 0), aCopy.Node(1));
        Assert.Equal(new Poly_Triangle(1, 2, 3), aCopy.Triangle(1));
    }

    [Fact]
    public void Clear_RemovesEverything()
    {
        Poly_Triangulation aTri = Square();
        aTri.ComputeNormals();
        aTri.Clear();
        Assert.Equal(0, aTri.NbNodes());
        Assert.Equal(0, aTri.NbTriangles());
        Assert.False(aTri.HasUVNodes());
        Assert.False(aTri.HasNormals());
        Assert.False(aTri.HasGeometry());
    }

    [Fact]
    public void Resize_KeepsOrDropsData()
    {
        Poly_Triangulation aTri = Square();
        aTri.ComputeNormals();
        aTri.ResizeNodes(6, true);
        Assert.Equal(6, aTri.NbNodes());
        Assert.Equal(new gp_Pnt(1, 1, 0), aTri.Node(3));
        Assert.Equal(new gp_Pnt2d(1, 1), aTri.UVNode(3));
        Assert.Equal(new gp_Dir(0, 0, 1), aTri.Normal(4));
        Assert.Equal(new gp_Pnt(0, 0, 0), aTri.Node(6));

        aTri.ResizeTriangles(1, true);
        Assert.Equal(1, aTri.NbTriangles());
        Assert.Equal(new Poly_Triangle(1, 2, 3), aTri.Triangle(1));

        aTri.ResizeNodes(2, false);
        Assert.Equal(new gp_Pnt(0, 0, 0), aTri.Node(2));

        aTri.RemoveUVNodes();
        Assert.False(aTri.HasUVNodes());
        aTri.AddUVNodes();
        Assert.True(aTri.HasUVNodes());
    }

    [Fact]
    public void SinglePrecision()
    {
        var aTri = new Poly_Triangulation();
        aTri.SetDoublePrecision(false);
        Assert.False(aTri.IsDoublePrecision());
        aTri.ResizeNodes(2, false);
        aTri.AddUVNodes();
        aTri.SetNode(1, new gp_Pnt(0.1, 0.2, 0.3));
        aTri.SetUVNode(2, new gp_Pnt2d(0.4, 0.5));
        Assert.Equal(new gp_Pnt((float)0.1, (float)0.2, (float)0.3), aTri.Node(1));
        Assert.Equal(new gp_Pnt2d((float)0.4, (float)0.5), aTri.UVNode(2));
        Assert.Equal(new[] { 0.1f, 0.2f, 0.3f, 0f, 0f, 0f }, aTri.NodeCoordinatesF().ToArray());
        Assert.Equal(new[] { 0f, 0f, 0.4f, 0.5f }, aTri.UVNodeCoordinatesF().ToArray());
        Assert.ThrowsAny<Standard_TypeMismatch>(() => aTri.NodeCoordinates().Length);

        var aDoubles = new double[6];
        aTri.CopyNodeCoordinatesTo(aDoubles);
        Assert.Equal((double)0.2f, aDoubles[1]);

        // the precision can't change once allocated
        Assert.ThrowsAny<Standard_ProgramError>(() => aTri.SetDoublePrecision(true));

        // a copy keeps the precision; MapNodeArray copies the nodes
        Poly_Triangulation aCopy = aTri.Copy();
        Assert.False(aCopy.IsDoublePrecision());
        NCollection_HArray1<gp_Pnt> aMapped = aCopy.MapNodeArray()!;
        Assert.Equal(aTri.Node(1), aMapped[1]);
    }

    [Fact]
    public void ArrayOfNodes_AssignConvertsAndWrapsShare()
    {
        var aDouble = new Poly_ArrayOfNodes(2);
        aDouble.SetValue(0, new gp_Pnt(1.1, 2.2, 3.3));
        var aSingle = new Poly_ArrayOfNodes();
        aSingle.SetDoublePrecision(false);
        aSingle.Resize(2, false);
        aSingle.Assign(aDouble);
        Assert.Equal(new gp_Pnt((float)1.1, (float)2.2, (float)3.3), aSingle[0]);
        Assert.Equal(12, aSingle.Stride());
        Assert.Equal(24, aDouble.Stride());
        Assert.Equal(48L, aDouble.SizeBytes());

        var aShort = new Poly_ArrayOfNodes(1);
        Assert.ThrowsAny<Standard_DimensionMismatch>(() => aShort.Assign(aDouble));

        gp_Pnt[] aPoints = [new(1, 1, 1), new(2, 2, 2), new(3, 3, 3)];
        var aWrap = new Poly_ArrayOfNodes(aPoints, 1, 2);
        Assert.False(aWrap.IsDeletable());
        Assert.Equal(new gp_Pnt(2, 2, 2), aWrap.Value(0));
        aWrap.SetValue(1, new gp_Pnt(7, 7, 7));
        Assert.Equal(new gp_Pnt(7, 7, 7), aPoints[2]);
        Assert.ThrowsAny<Standard_OutOfRange>(() => aWrap.Value(2));

        var aCopy = new Poly_ArrayOfNodes(aWrap);
        Assert.True(aCopy.IsDeletable());
        aCopy.SetValue(0, new gp_Pnt(0, 0, 0));
        Assert.Equal(new gp_Pnt(2, 2, 2), aPoints[1]);

        aCopy.Resize(3, true);
        Assert.Equal(new gp_Pnt(7, 7, 7), aCopy[1]);
        Assert.Equal(3, aCopy.AsSpan<gp_Pnt>().Length);
        Assert.ThrowsAny<Standard_TypeMismatch>(() => aCopy.AsSpan<NCollection_Vec3<float>>().Length);
    }

    [Fact]
    public void ArrayOfUVNodes_SinglePrecision()
    {
        var anArray = new Poly_ArrayOfUVNodes();
        anArray.SetDoublePrecision(false);
        anArray.Resize(1, false);
        anArray.SetValue(0, new gp_Pnt2d(0.3, 0.7));
        Assert.Equal(new gp_Pnt2d((float)0.3, (float)0.7), anArray.Value(0));
        Assert.Equal(8, anArray.Stride());
        Assert.False(anArray.IsDoublePrecision());
    }

    [Fact]
    public void MapArrays_ShareStorage()
    {
        Poly_Triangulation aTri = Square();
        aTri.ComputeNormals();

        NCollection_HArray1<gp_Pnt> aNodes = aTri.MapNodeArray()!;
        Assert.Equal(1, aNodes.Lower());
        Assert.Equal(4, aNodes.Upper());
        Assert.Equal(new gp_Pnt(1, 1, 0), aNodes[3]);
        aNodes[3] = new gp_Pnt(2, 2, 0);
        Assert.Equal(new gp_Pnt(2, 2, 0), aTri.Node(3));

        NCollection_HArray1<Poly_Triangle> aTris = aTri.MapTriangleArray()!;
        Assert.Equal(new Poly_Triangle(1, 3, 4), aTris[2]);
        aTris[2] = new Poly_Triangle(4, 3, 1);
        Assert.Equal(new Poly_Triangle(4, 3, 1), aTri.Triangle(2));

        NCollection_HArray1<gp_Pnt2d> aUV = aTri.MapUVNodeArray()!;
        Assert.Equal(new gp_Pnt2d(1, 1), aUV[3]);

        NCollection_HArray1<float> aNormals = aTri.MapNormalArray()!;
        Assert.Equal(12, aNormals.Length());
        Assert.Equal(1.0f, aNormals[3]);

        var anEmpty = new Poly_Triangulation();
        Assert.Null(anEmpty.MapNodeArray());
        Assert.Null(anEmpty.MapTriangleArray());
        Assert.Null(anEmpty.MapUVNodeArray());
        Assert.Null(anEmpty.MapNormalArray());
    }

    [Fact]
    public void SetNormals_Deprecated()
    {
        Poly_Triangulation aTri = Square();
        var aValues = new NCollection_HArray1<float>(1, 12, 0.0f);
        aValues[12] = 1.0f;
#pragma warning disable CS0618 // OCCT's deprecated method, tested as ported
        Assert.ThrowsAny<Standard_DomainError>(() => aTri.SetNormals(new NCollection_HArray1<float>(1, 3)));
        aTri.SetNormals(aValues);
#pragma warning restore CS0618
        Assert.Equal(new NCollection_Vec3<float>(0.0f, 0.0f, 1.0f), Normal(aTri, 4));
    }

    private static NCollection_Vec3<float> Normal(Poly_Triangulation theTri, int theIndex)
    {
        theTri.Normal(theIndex, out NCollection_Vec3<float> aVec);
        return aVec;
    }

    [Fact]
    public void MinMax_AndCache()
    {
        Poly_Triangulation aTri = Square();
        Assert.False(aTri.HasCachedMinMax());
        Assert.True(aTri.CachedMinMax().IsVoid());

        var aBox = new Bnd_Box();
        Assert.True(aTri.MinMax(ref aBox, new gp_Trsf()));
        Bnd_Box.Limits aLimits = aBox.Get();
        Assert.Equal(new Bnd_Box.Limits(0, 1, 0, 1, 0, 0), aLimits);

        // translated
        var aTrsf = new gp_Trsf();
        aTrsf.SetTranslation(new gp_Vec(10, 0, 0));
        var aMoved = new Bnd_Box();
        Assert.True(aTri.MinMax(ref aMoved, aTrsf, true));
        Assert.Equal(10.0, aMoved.Get().Xmin);

        // the cache is used, even when it doesn't match the nodes
        var aCache = new Bnd_Box(new gp_Pnt(-5, -5, -5), new gp_Pnt(5, 5, 5));
        aTri.SetCachedMinMax(aCache);
        Assert.True(aTri.HasCachedMinMax());
        var aCached = new Bnd_Box();
        aTri.MinMax(ref aCached, new gp_Trsf());
        Assert.Equal(-5.0, aCached.Get().Xmin);

        // an accurate box with a rotation ignores the cache
        var aRot = new gp_Trsf();
        aRot.SetRotation(new gp_Ax1(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), Math.PI / 2);
        var anAccurate = new Bnd_Box();
        aTri.MinMax(ref anAccurate, aRot, true);
        Assert.Equal(-1.0, anAccurate.Get().Xmin, 1e-12);
        Assert.Equal(0.0, anAccurate.Get().Ymin, 1e-12);

        aTri.SetCachedMinMax(new Bnd_Box());
        Assert.False(aTri.HasCachedMinMax());

        var anEmptyBox = new Bnd_Box();
        Assert.False(new Poly_Triangulation().MinMax(ref anEmptyBox, new gp_Trsf()));
    }

    [Fact]
    public void DeferredData_NoneByDefault()
    {
        Poly_Triangulation aTri = Square();
        Assert.False(aTri.HasDeferredData());
        Assert.False(aTri.LoadDeferredData());
        Assert.Null(aTri.DetachedLoadDeferredData());
        Assert.False(aTri.UnloadDeferredData());
        Assert.Equal(4, aTri.NbNodes());
    }

    private sealed class DeferredTriangulation : Poly_Triangulation
    {
        public override int NbDeferredTriangles() => 1;

        protected override Poly_Triangulation createNewEntity() => new DeferredTriangulation();

        protected override bool loadDeferredData(Poly_Triangulation theDestTriangulation)
        {
            theDestTriangulation.ResizeNodes(3, false);
            theDestTriangulation.ResizeTriangles(1, false);
            theDestTriangulation.SetTriangle(1, new Poly_Triangle(1, 2, 3));
            return true;
        }
    }

    [Fact]
    public void DeferredData_LoadAndUnload()
    {
        var aTri = new DeferredTriangulation();
        Assert.True(aTri.HasDeferredData());
        Assert.True(aTri.LoadDeferredData());
        Assert.Equal(3, aTri.NbNodes());
        Assert.Equal(Poly_MeshPurpose_Loaded, aTri.MeshPurpose() & Poly_MeshPurpose_Loaded);

        Poly_Triangulation aDetached = aTri.DetachedLoadDeferredData()!;
        Assert.IsType<DeferredTriangulation>(aDetached);
        Assert.Equal(1, aDetached.NbTriangles());

        Assert.True(aTri.UnloadDeferredData());
        Assert.Equal(0, aTri.NbNodes());
        Assert.Equal(Poly_MeshPurpose_NONE, aTri.MeshPurpose());
    }

    [Fact]
    public void TriangulationParameters()
    {
        var aDefault = new Poly_TriangulationParameters();
        Assert.False(aDefault.HasDeflection());
        Assert.False(aDefault.HasAngle());
        Assert.False(aDefault.HasMinSize());

        var aParams = new Poly_TriangulationParameters(0.1, 0.2, 0.0);
        Assert.True(aParams.HasDeflection());
        Assert.True(aParams.HasMinSize());
        Poly_TriangulationParameters aCopy = aParams.Copy();
        Assert.Equal((0.1, 0.2, 0.0), (aCopy.Deflection(), aCopy.Angle(), aCopy.MinSize()));
    }
}
