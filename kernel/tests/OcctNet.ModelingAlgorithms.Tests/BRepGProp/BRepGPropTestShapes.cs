// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Geometry;
using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Shapes for the BRepGProp tests and other unit tests, built with BRep_Builder on purpose: closed cylinders, spheres
/// and sphere patches as natural faces (BRepPrimAPI's makers give faces with seams and degenerated edges, which the
/// ported GTests use), and meshes attached to the faces of a box.
/// </summary>
internal static class BRepGPropTestShapes
{
    private const double Tol = 1.0e-7;

    /// <summary>
    /// A closed cylinder of radius r and height h around the Z axis: the lateral face (seam at U = 0) and two discs
    /// whose circles are shared with it; a closed shell in a solid. Unfrozen.
    /// </summary>
    public static TopoDS_Solid CylinderSolid(double r, double h)
    {
        var aB = new BRep_Builder();
        double aTwoPi = 2.0 * Math.PI;
        var anAx3 = new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));
        var aSurf = new Geom_CylindricalSurface(anAx3, r);
        TopoDS_Vertex aV0 = TestShapes.Vertex(new Point3D(r, 0, 0));
        TopoDS_Vertex aV1 = TestShapes.Vertex(new Point3D(r, 0, h));
        TopoDS_Edge aBottom = TestShapes.Edge(new Geom_Circle(new Frame3D(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r),
                                              0.0, aTwoPi, aV0, aV0);
        TopoDS_Edge aTop = TestShapes.Edge(new Geom_Circle(new Frame3D(new Point3D(0, 0, h), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r),
                                           0.0, aTwoPi, aV1, aV1);
        TopoDS_Edge aSeam = TestShapes.Edge(new Geom_Line(new Point3D(r, 0, 0), new Direction3D(0, 0, 1)), 0.0, h, aV0, aV1);

        TopoDS_Face aLateral = new();
        aB.MakeFace(ref aLateral, aSurf, Tol);
        aB.UpdateEdge(aBottom, new Geom2d_Line(new Point2D(0, 0), new Direction2D(1, 0)), aLateral, Tol);
        aB.UpdateEdge(aTop, new Geom2d_Line(new Point2D(0, h), new Direction2D(1, 0)), aLateral, Tol);
        aB.UpdateEdge(aSeam,
                      new Geom2d_Line(new Point2D(aTwoPi, 0), new Direction2D(0, 1)),
                      new Geom2d_Line(new Point2D(0, 0), new Direction2D(0, 1)),
                      aLateral,
                      Tol);
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, aBottom);
        aB.Add(aW, aSeam);
        aB.Add(aW, aTop.Reversed());
        aB.Add(aW, aSeam.Reversed());
        aW.Closed(true);
        aB.Add(aLateral, aW);

        // the discs: planes of normal +Z, the bottom one reversed so that its normal points out
        TopoDS_Face aBottomFace = Disc(aBottom, 0.0, r);
        TopoDS_Face aTopFace = Disc(aTop, h, r);

        TopoDS_Shell aShell = new();
        aB.MakeShell(ref aShell);
        aB.Add(aShell, aLateral);
        aB.Add(aShell, aBottomFace.Reversed());
        aB.Add(aShell, aTopFace);
        aShell.Closed(true);
        TopoDS_Solid aSolid = new();
        aB.MakeSolid(ref aSolid);
        aB.Add(aSolid, aShell);
        return aSolid;
    }

    /// <summary>A disc on the plane z = <paramref name="z"/> (normal +Z) bounded by the circle edge.</summary>
    private static TopoDS_Face Disc(in TopoDS_Edge theCircle, double z, double r)
    {
        var aB = new BRep_Builder();
        var aPlane = new Geom_Plane(new CoordinateSystem(new Point3D(0, 0, z), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aPlane, Tol);
        aB.UpdateEdge(theCircle, new Geom2d_Circle(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), r), aF, Tol);
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, theCircle);
        aW.Closed(true);
        aB.Add(aF, aW);
        return aF;
    }

    /// <summary>A face on the surface with no wire (its natural restriction).</summary>
    public static TopoDS_Face NaturalFace(Geom_Surface theSurface)
    {
        var aB = new BRep_Builder();
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, theSurface, Tol);
        aB.NaturalRestriction(aF, true);
        return aF;
    }

    /// <summary>A sphere of radius r at the origin: one natural face in a closed shell, in a solid.</summary>
    public static TopoDS_Solid SphereSolid(double r)
    {
        var aB = new BRep_Builder();
        TopoDS_Face aF = NaturalFace(new Geom_SphericalSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r));
        TopoDS_Shell aShell = new();
        aB.MakeShell(ref aShell);
        aB.Add(aShell, aF);
        aShell.Closed(true);
        TopoDS_Solid aSolid = new();
        aB.MakeSolid(ref aSolid);
        aB.Add(aSolid, aShell);
        return aSolid;
    }

    /// <summary>A solid holding the given shell.</summary>
    public static TopoDS_Solid SolidOf(in TopoDS_Shell theShell)
    {
        var aB = new BRep_Builder();
        TopoDS_Solid aSolid = new();
        aB.MakeSolid(ref aSolid);
        aB.Add(aSolid, theShell);
        return aSolid;
    }

    /// <summary>A compound of the given shapes.</summary>
    public static TopoDS_Compound CompoundOf(params TopoDS_Shape[] theShapes)
    {
        var aB = new BRep_Builder();
        TopoDS_Compound aC = new();
        aB.MakeCompound(ref aC);
        foreach (TopoDS_Shape aS in theShapes)
        {
            aB.Add(aC, aS);
        }

        return aC;
    }

    /// <summary>The faces of the shape, in exploration order.</summary>
    public static List<TopoDS_Face> Faces(in TopoDS_Shape theShape)
    {
        var aFaces = new List<TopoDS_Face>();
        for (var anExp = new TopExp_Explorer(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aFaces.Add(TopoDS.Face(anExp.Current()));
        }

        return aFaces;
    }

    /// <summary>
    /// Attaches to each planar polygon face of a box (TestShapes.Box) a triangulation of its quadrangle (two
    /// triangles), wound as the face's surface: the mesh is exact.
    /// </summary>
    public static void AttachQuadMeshes(in TopoDS_Shape theBox)
    {
        var aB = new BRep_Builder();
        foreach (TopoDS_Face aF in Faces(theBox))
        {
            TopoDS_Face aFwd = TopoDS.Face(aF.Oriented(TopAbs_FORWARD));
            // the corners in wire order, which runs counter-clockwise around the plane's normal
            var aCorners = new List<Point3D>();
            for (var aW = new TopExp_Explorer(aFwd, TopAbs_EDGE); aW.More(); aW.Next())
            {
                TopoDS_Edge anE = TopoDS.Edge(aW.Current());
                TopExp.Vertices(anE, out TopoDS_Vertex aFirst, out _, true);
                aCorners.Add(BRep_Tool.Pnt(aFirst));
            }

            var aTri = new Triangulation(4, 2, false);
            for (int i = 0; i < 4; i++)
            {
                aTri.SetNode(i + 1, aCorners[i]);
            }

            aTri.SetTriangle(1, new Triangle1(1, 2, 3));
            aTri.SetTriangle(2, new Triangle1(1, 3, 4));
            aB.UpdateFace(aF, aTri);
        }
    }

    /// <summary>A face with no surface carrying a triangulation of the given points and triangles.</summary>
    public static TopoDS_Face MeshFace(Point3D[] theNodes, params (int, int, int)[] theTriangles)
    {
        var aTri = new Triangulation(theNodes.Length, theTriangles.Length, false);
        for (int i = 0; i < theNodes.Length; i++)
        {
            aTri.SetNode(i + 1, theNodes[i]);
        }

        for (int i = 0; i < theTriangles.Length; i++)
        {
            aTri.SetTriangle(i + 1, new Triangle1(theTriangles[i].Item1, theTriangles[i].Item2, theTriangles[i].Item3));
        }

        var aB = new BRep_Builder();
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aTri);
        return aF;
    }

    /// <summary>An edge with no curve carrying a 3D polygon through the points.</summary>
    public static TopoDS_Edge PolygonEdge(params Point3D[] thePoints)
    {
        var aNodes = new NCollection_Array1<Point3D>(1, thePoints.Length);
        for (int i = 0; i < thePoints.Length; i++)
        {
            aNodes[i + 1] = thePoints[i];
        }

        var aB = new BRep_Builder();
        TopoDS_Edge anE = new();
        aB.MakeEdge(ref anE, new Polygon3D(aNodes));
        return anE;
    }
}
