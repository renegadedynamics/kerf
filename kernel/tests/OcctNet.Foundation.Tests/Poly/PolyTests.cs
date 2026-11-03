// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace OcctNet.Foundation.Tests;

/// <summary>The Poly package class, the polygons and Poly_Connect (OCCT has no GTests for the Poly package).</summary>
public class PolyTest
{
    [Fact]
    public void Catenate_SumsWithoutSharing()
    {
        global::Kerf.Engine.Meshing.Triangulation aSquare = Poly_TriangulationTest.Square();
        global::Kerf.Engine.Meshing.Triangulation aTetra = Poly_TriangulationTest.Tetrahedron();
        var aList = new NCollection_List<global::Kerf.Engine.Meshing.Triangulation?>();
        aList.Append(aSquare);
        aList.Append((global::Kerf.Engine.Meshing.Triangulation?)null);
        aList.Append(aTetra);

        global::Kerf.Engine.Meshing.Triangulation aResult = global::Kerf.Engine.Meshing.Poly.Catenate(aList)!;
        Assert.Equal(8, aResult.NodeCount);
        Assert.Equal(6, aResult.TriangleCount);
        Assert.False(aResult.HasUVNodes);
        Assert.Equal(aTetra.Node(1), aResult.Node(5));
        Assert.Equal(new global::Kerf.Engine.Meshing.Triangle1(5, 6, 7), aResult.Triangle(3));
        Assert.Equal(new global::Kerf.Engine.Meshing.Triangle1(6, 8, 7), aResult.Triangle(6));

        Assert.Null(global::Kerf.Engine.Meshing.Poly.Catenate(new NCollection_List<global::Kerf.Engine.Meshing.Triangulation?>()));
    }

    [Fact]
    public void Write_Compact_ReadBack()
    {
        global::Kerf.Engine.Meshing.Triangulation aTri = Poly_TriangulationTest.Square();
        aTri.SetNode(3, new Point3D(1.5, 1e-7, -2.25));
        aTri.Deflection(0.125);
        var aWriter = new StringWriter();
        global::Kerf.Engine.Meshing.Poly.Write(aTri, aWriter);
        string aText = aWriter.ToString();
        Assert.Equal("Poly_Triangulation\n4 2 1\n0.125\n0 0 0\n1 0 0\n1.5 1e-07 -2.25\n0 1 0\n"
                     + "0 0\n1 0\n1 1\n0 1\n1 2 3\n1 3 4\n", aText);

        global::Kerf.Engine.Meshing.Triangulation aRead = global::Kerf.Engine.Meshing.Poly.ReadTriangulation(new StringReader(aText))!;
        Assert.Equal(4, aRead.NodeCount);
        Assert.Equal(2, aRead.TriangleCount);
        Assert.Equal(0.125, aRead.Deflection());
        Assert.Equal(new Point3D(1.5, 1e-7, -2.25), aRead.Node(3));
        Assert.Equal(new Point2D(1, 1), aRead.UVNode(3));
        Assert.Equal(new global::Kerf.Engine.Meshing.Triangle1(1, 3, 4), aRead.Triangle(2));

        Assert.Null(global::Kerf.Engine.Meshing.Poly.ReadTriangulation(new StringReader("Poly_Polygon3D 1 0 0 0 0 0")));
    }

    [Fact]
    public void Dump_Triangulation()
    {
        var aTri = new global::Kerf.Engine.Meshing.Triangulation(1, 1, false);
        aTri.SetNode(1, new Point3D(1, 2, 3));
        aTri.SetTriangle(1, new global::Kerf.Engine.Meshing.Triangle1(1, 1, 1));
        var aWriter = new StringWriter();
        global::Kerf.Engine.Meshing.Poly.Dump(aTri, aWriter);
        Assert.Equal("Poly_Triangulation\n       1 Nodes\n       1 Triangles\nwithout UV nodes\nDeflection : 0\n"
                     + "\n3D Nodes :\n         1 :                 1                 2                 3\n"
                     + "\nTriangles :\n         1 :          1          1          1\n", aWriter.ToString());
    }

    [Fact]
    public void Polygon3D_WriteReadCopy()
    {
        var aNodes = new NCollection_Array1<Point3D>(0, 2);
        aNodes[0] = new Point3D(0, 0, 0);
        aNodes[1] = new Point3D(1, 0, 0);
        aNodes[2] = new Point3D(1, 1, 0);
        var aParams = new NCollection_Array1<double>(0, 2);
        aParams[1] = 0.5;
        aParams[2] = 1.0;
        var aPoly = new global::Kerf.Engine.Meshing.Polygon3D(aNodes, aParams);
        aPoly.Deflection(0.01);
        Assert.Equal(3, aPoly.NodeCount);
        Assert.Equal(1, aPoly.Nodes().Lower());
        Assert.Equal(new Point3D(1, 1, 0), aPoly.Nodes()[3]);
        Assert.True(aPoly.HasParameters);
        Assert.Equal(0.5, aPoly.Parameters()[2]);

        global::Kerf.Engine.Meshing.Polygon3D aCopy = aPoly.Copy();
        aPoly.ChangeNodes()[1] = new Point3D(5, 5, 5);
        aPoly.ChangeParameters()[1] = 7.0;
        Assert.Equal(new Point3D(0, 0, 0), aCopy.Nodes()[1]);
        Assert.Equal(0.0, aCopy.Parameters()[1]);
        Assert.Equal(0.01, aCopy.Deflection());

        var aWriter = new StringWriter();
        global::Kerf.Engine.Meshing.Poly.Write(aCopy, aWriter);
        Assert.Equal("Poly_Polygon3D\n3 1\n0.01\n0 0 0\n1 0 0\n1 1 0\n0 0.5 1 \n", aWriter.ToString());
        global::Kerf.Engine.Meshing.Polygon3D aRead = global::Kerf.Engine.Meshing.Poly.ReadPolygon3D(new StringReader(aWriter.ToString()))!;
        Assert.Equal(1.0, aRead.Parameters()[3]);
        Assert.Equal(new Point3D(1, 0, 0), aRead.Nodes()[2]);

        var aNoParams = new global::Kerf.Engine.Meshing.Polygon3D(2, false);
        Assert.False(aNoParams.HasParameters);
        Assert.ThrowsAny<NullObjectException>(() => aNoParams.Parameters());
    }

    [Fact]
    public void Polygon2D_WriteReadCopy()
    {
        var aNodes = new NCollection_Array1<Point2D>(1, 2);
        aNodes[2] = new Point2D(0.5, -3);
        var aPoly = new global::Kerf.Engine.Meshing.Polygon2D(aNodes);
        global::Kerf.Engine.Meshing.Polygon2D aCopy = aPoly.Copy();
        aPoly.ChangeNodes()[2] = new Point2D(9, 9);
        Assert.Equal(new Point2D(0.5, -3), aCopy.Nodes()[2]);

        var aWriter = new StringWriter();
        global::Kerf.Engine.Meshing.Poly.Write(aCopy, aWriter);
        Assert.Equal("Poly_Polygon2D\n2 0\n0 0\n0.5 -3\n", aWriter.ToString());
        global::Kerf.Engine.Meshing.Polygon2D aRead = global::Kerf.Engine.Meshing.Poly.ReadPolygon2D(new StringReader(aWriter.ToString()))!;
        Assert.Equal(new Point2D(0.5, -3), aRead.Nodes()[2]);

        var aDump = new StringWriter();
        global::Kerf.Engine.Meshing.Poly.Dump(aCopy, aDump);
        Assert.StartsWith("Poly_Polygon2D\n       2 Nodes\nDeflection : 0\n\nNodes :\n", aDump.ToString());
    }

    [Fact]
    public void PolygonOnTriangulation()
    {
        var aNodes = new NCollection_Array1<int>(1, 3);
        aNodes[1] = 4;
        aNodes[2] = 7;
        aNodes[3] = 4;
        var aPoly = new global::Kerf.Engine.Meshing.PolygonOnTriangulation(aNodes);
        Assert.Equal(3, aPoly.NodeCount);
        Assert.Equal(7, aPoly.Node(2));
        Assert.False(aPoly.HasParameters);
        Assert.ThrowsAny<NullObjectException>(() => aPoly.Parameter(1));

        Assert.ThrowsAny<OutOfRangeException>(() => aPoly.SetParameters(new NCollection_HArray1<double>(1, 2)));
        var aParams = new NCollection_HArray1<double>(1, 3);
        aPoly.SetParameters(aParams);
        aPoly.SetParameter(2, 0.75);
        Assert.Equal(0.75, aParams[2]);
        Assert.Same(aParams, aPoly.Parameters());

        global::Kerf.Engine.Meshing.PolygonOnTriangulation aCopy = aPoly.Copy();
        aPoly.SetNode(1, 1);
        aPoly.SetParameter(2, 0.0);
        Assert.Equal(4, aCopy.Node(1));
        Assert.Equal(0.75, aCopy.Parameter(2));

        aPoly.SetParameters(null);
        Assert.False(aPoly.HasParameters);

        var aSized = new global::Kerf.Engine.Meshing.PolygonOnTriangulation(2, true);
        Assert.True(aSized.HasParameters);
        Assert.Equal(0.0, aSized.Parameter(2));
    }

    [Fact]
    public void PointOnTriangle()
    {
        var aP1 = new Xy(0, 0);
        var aP2 = new Xy(1, 0);
        var aP3 = new Xy(0, 1);

        Assert.Equal(0.0, global::Kerf.Engine.Meshing.Poly.PointOnTriangle(aP1, aP2, aP3, new Xy(0.25, 0.5), out Xy anUV));
        Assert.Equal(new Xy(0.25, 0.5), anUV);

        // outside, closest to the hypotenuse
        Assert.Equal(0.5, global::Kerf.Engine.Meshing.Poly.PointOnTriangle(aP1, aP2, aP3, new Xy(1, 1), out anUV), 1e-15);
        Assert.Equal(new Xy(0.5, 0.5), anUV);

        // outside, closest to side V = 0
        Assert.Equal(4.0, global::Kerf.Engine.Meshing.Poly.PointOnTriangle(aP1, aP2, aP3, new Xy(0.5, -2), out anUV), 1e-15);
        Assert.Equal(new Xy(0.5, 0), anUV);

        // degenerate to a point
        Assert.Equal(2.0, global::Kerf.Engine.Meshing.Poly.PointOnTriangle(aP1, aP1, aP1, new Xy(1, 1), out anUV), 1e-15);
        Assert.Equal(new Xy(0, 0), anUV);

        // collinear sides
        Assert.Equal(1.0, global::Kerf.Engine.Meshing.Poly.PointOnTriangle(aP1, aP2, new Xy(2, 0), new Xy(0.5, 1), out anUV), 1e-15);
    }

    [Fact]
    public void Intersect_ClosestAndFarthest()
    {
        // two parallel squares at z = 0 and z = 2
        global::Kerf.Engine.Meshing.Triangulation aSquare = Poly_TriangulationTest.Square();
        var aList = new NCollection_List<global::Kerf.Engine.Meshing.Triangulation?>();
        aList.Append(aSquare);
        aList.Append(aSquare);
        global::Kerf.Engine.Meshing.Triangulation aTwo = global::Kerf.Engine.Meshing.Poly.Catenate(aList)!;
        for (int i = 5; i <= 8; i++)
        {
            aTwo.SetNode(i, aTwo.Node(i).Translated(new Vector3D(0, 0, 2)));
        }

        var anAxis = new Axis3D(new Point3D(0.75, 0.25, 5), new Direction3D(0, 0, -1));
        var aTriangle = new global::Kerf.Engine.Meshing.Triangle1();
        double aDistance = -1;
        Assert.True(global::Kerf.Engine.Meshing.Poly.Intersect(aTwo, anAxis, true, ref aTriangle, ref aDistance));
        Assert.Equal(3.0, aDistance, 1e-12);
        Assert.Equal(new global::Kerf.Engine.Meshing.Triangle1(5, 6, 7), aTriangle);

        Assert.True(global::Kerf.Engine.Meshing.Poly.Intersect(aTwo, anAxis, false, ref aTriangle, ref aDistance));
        Assert.Equal(5.0, aDistance, 1e-12);
        Assert.Equal(new global::Kerf.Engine.Meshing.Triangle1(1, 2, 3), aTriangle);

        var aMiss = new Axis3D(new Point3D(3, 3, 5), new Direction3D(0, 0, -1));
        aDistance = -1;
        Assert.False(global::Kerf.Engine.Meshing.Poly.Intersect(aTwo, aMiss, true, ref aTriangle, ref aDistance));
        Assert.Equal(-1.0, aDistance);

        // behind the start
        var aBehind = new Axis3D(new Point3D(0.75, 0.25, 5), new Direction3D(0, 0, 1));
        Assert.False(global::Kerf.Engine.Meshing.Poly.Intersect(aTwo, aBehind, true, ref aTriangle, ref aDistance));
    }

    [Fact]
    public void IntersectTriLine()
    {
        double aParam = -1;
        var aV0 = new Xyz(0, 0, 0);
        var aV1 = new Xyz(1, 0, 0);
        var aV2 = new Xyz(0, 1, 0);
        Assert.Equal(1, global::Kerf.Engine.Meshing.Poly.IntersectTriLine(new Xyz(0.2, 0.2, 1), new Direction3D(0, 0, -1), aV0, aV1, aV2, ref aParam));
        Assert.Equal(1.0, aParam, 1e-15);
        Assert.Equal(1, global::Kerf.Engine.Meshing.Poly.IntersectTriLine(new Xyz(0.2, 0.2, 1), new Direction3D(0, 0, 1), aV0, aV1, aV2, ref aParam));
        Assert.Equal(-1.0, aParam, 1e-15);
        aParam = 7;
        Assert.Equal(0, global::Kerf.Engine.Meshing.Poly.IntersectTriLine(new Xyz(0.8, 0.8, 1), new Direction3D(0, 0, -1), aV0, aV1, aV2, ref aParam));
        Assert.Equal(7.0, aParam);
        Assert.Equal(0, global::Kerf.Engine.Meshing.Poly.IntersectTriLine(new Xyz(0.2, 0.2, 1), new Direction3D(1, 0, 0), aV0, aV1, aV2, ref aParam));
    }

    [Fact]
    public void PolygonProperties()
    {
        var aSquare = new NCollection_Array1<Point2D>(1, 4);
        aSquare[1] = new Point2D(0, 0);
        aSquare[2] = new Point2D(2, 0);
        aSquare[3] = new Point2D(2, 1);
        aSquare[4] = new Point2D(0, 1);
        Assert.True(global::Kerf.Engine.Meshing.Poly.PolygonProperties(aSquare, out double anArea, out double aPerimeter));
        Assert.Equal(2.0, anArea, 1e-15);
        Assert.Equal(6.0, aPerimeter, 1e-15);

        var aSeq = new NCollection_Sequence<Point2D>();
        for (int i = 4; i >= 1; i--)
        {
            aSeq.Append(aSquare[i]);
        }

        Assert.True(global::Kerf.Engine.Meshing.Poly.PolygonProperties(aSeq, out anArea, out aPerimeter));
        Assert.Equal(-2.0, anArea, 1e-15);
        Assert.Equal(6.0, aPerimeter, 1e-15);

        Assert.True(global::Kerf.Engine.Meshing.Poly.PolygonProperties(new NCollection_Sequence<Point2D>(), out anArea, out aPerimeter));
        Assert.Equal((0.0, 0.0), (anArea, aPerimeter));
    }

    [Fact]
    public void Connect_Adjacency()
    {
        // (1, 2, 3) and (1, 3, 4) share the edge 3-1
        var aConnect = new global::Kerf.Engine.Meshing.Connect(Poly_TriangulationTest.Square());
        aConnect.Triangles(1, out int t1, out int t2, out int t3);
        Assert.Equal((0, 0, 2), (t1, t2, t3));
        aConnect.Nodes(1, out int n1, out int n2, out int n3);
        Assert.Equal((0, 0, 4), (n1, n2, n3));
        aConnect.Triangles(2, out t1, out t2, out t3);
        Assert.Equal((1, 0, 0), (t1, t2, t3));
        aConnect.Nodes(2, out n1, out n2, out n3);
        Assert.Equal((2, 0, 0), (n1, n2, n3));

        Assert.Equal(1, aConnect.Triangle(2));
        Assert.Equal(2, aConnect.Triangle(4));
    }

    [Fact]
    public void Connect_TrianglesAroundNode()
    {
        // a closed fan of 6 triangles around node 1, and an open fan of 3 around node 8
        var aTri = new global::Kerf.Engine.Meshing.Triangulation(11, 9, false);
        aTri.SetNode(1, new Point3D(0, 0, 0));
        for (int i = 0; i < 6; i++)
        {
            aTri.SetNode(i + 2, new Point3D(Math.Cos(i * Math.PI / 3), Math.Sin(i * Math.PI / 3), 0));
            aTri.SetTriangle(i + 1, new global::Kerf.Engine.Meshing.Triangle1(1, i + 2, (i + 1) % 6 + 2));
        }

        aTri.SetNode(8, new Point3D(10, 0, 0));
        aTri.SetNode(9, new Point3D(11, 0, 0));
        aTri.SetNode(10, new Point3D(11, 1, 0));
        aTri.SetNode(11, new Point3D(10, 1, 0));
        aTri.SetTriangle(7, new global::Kerf.Engine.Meshing.Triangle1(8, 9, 10));
        aTri.SetTriangle(8, new global::Kerf.Engine.Meshing.Triangle1(8, 10, 11));
        aTri.SetTriangle(9, new global::Kerf.Engine.Meshing.Triangle1(8, 11, 9));

        var aConnect = new global::Kerf.Engine.Meshing.Connect(aTri);
        Assert.Same(aTri, aConnect.Triangulation());
        var aFound = new List<int>();
        for (aConnect.Initialize(1); aConnect.More(); aConnect.Next())
        {
            aFound.Add(aConnect.Value());
        }

        Assert.Equal(6, aFound.Count);
        Assert.Equal([1, 2, 3, 4, 5, 6], aFound.Order());

        aFound.Clear();
        for (aConnect.Initialize(9); aConnect.More(); aConnect.Next())
        {
            aFound.Add(aConnect.Value());
        }

        Assert.Equal([7, 9], aFound.Order());

        // a node of no triangle
        var aLoose = new global::Kerf.Engine.Meshing.Triangulation(4, 1, false);
        aLoose.SetTriangle(1, new global::Kerf.Engine.Meshing.Triangle1(1, 2, 3));
        var aLooseConnect = new global::Kerf.Engine.Meshing.Connect();
        aLooseConnect.Load(aLoose);
        aLooseConnect.Initialize(4);
        Assert.False(aLooseConnect.More());
    }
}
