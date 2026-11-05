// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

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
    public static Solid CylinderSolid(double r, double h)
    {
        var aB = new BrepBuilder();
        double aTwoPi = 2.0 * Math.PI;
        var anAx3 = new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));
        var aSurf = new CylindricalSurface(anAx3, r);
        Vertex aV0 = TestShapes.Vertex(new Point3D(r, 0, 0));
        Vertex aV1 = TestShapes.Vertex(new Point3D(r, 0, h));
        Edge aBottom = TestShapes.Edge(new CircleCurve3D(new Frame3D(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r),
                                              0.0, aTwoPi, aV0, aV0);
        Edge aTop = TestShapes.Edge(new CircleCurve3D(new Frame3D(new Point3D(0, 0, h), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r),
                                           0.0, aTwoPi, aV1, aV1);
        Edge aSeam = TestShapes.Edge(new LineCurve3D(new Point3D(r, 0, 0), new Direction3D(0, 0, 1)), 0.0, h, aV0, aV1);

        Face aLateral = new();
        aB.MakeFace(ref aLateral, aSurf, Tol);
        aB.UpdateEdge(aBottom, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aLateral, Tol);
        aB.UpdateEdge(aTop, new LineCurve2D(new Point2D(0, h), new Direction2D(1, 0)), aLateral, Tol);
        aB.UpdateEdge(aSeam,
                      new LineCurve2D(new Point2D(aTwoPi, 0), new Direction2D(0, 1)),
                      new LineCurve2D(new Point2D(0, 0), new Direction2D(0, 1)),
                      aLateral,
                      Tol);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, aBottom);
        aB.Add(aW, aSeam);
        aB.Add(aW, aTop.Reversed());
        aB.Add(aW, aSeam.Reversed());
        aW.Closed(true);
        aB.Add(aLateral, aW);

        // the discs: planes of normal +Z, the bottom one reversed so that its normal points out
        Face aBottomFace = Disc(aBottom, 0.0, r);
        Face aTopFace = Disc(aTop, h, r);

        Shell aShell = new();
        aB.MakeShell(ref aShell);
        aB.Add(aShell, aLateral);
        aB.Add(aShell, aBottomFace.Reversed());
        aB.Add(aShell, aTopFace);
        aShell.Closed(true);
        Solid aSolid = new();
        aB.MakeSolid(ref aSolid);
        aB.Add(aSolid, aShell);
        return aSolid;
    }

    /// <summary>A disc on the plane z = <paramref name="z"/> (normal +Z) bounded by the circle edge.</summary>
    private static Face Disc(in Edge theCircle, double z, double r)
    {
        var aB = new BrepBuilder();
        var aPlane = new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, z), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        Face aF = new();
        aB.MakeFace(ref aF, aPlane, Tol);
        aB.UpdateEdge(theCircle, new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), r), aF, Tol);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, theCircle);
        aW.Closed(true);
        aB.Add(aF, aW);
        return aF;
    }

    /// <summary>A face on the surface with no wire (its natural restriction).</summary>
    public static Face NaturalFace(Surface theSurface)
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, theSurface, Tol);
        aB.NaturalRestriction(aF, true);
        return aF;
    }

    /// <summary>A sphere of radius r at the origin: one natural face in a closed shell, in a solid.</summary>
    public static Solid SphereSolid(double r)
    {
        var aB = new BrepBuilder();
        Face aF = NaturalFace(new SphericalSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r));
        Shell aShell = new();
        aB.MakeShell(ref aShell);
        aB.Add(aShell, aF);
        aShell.Closed(true);
        Solid aSolid = new();
        aB.MakeSolid(ref aSolid);
        aB.Add(aSolid, aShell);
        return aSolid;
    }

    /// <summary>A solid holding the given shell.</summary>
    public static Solid SolidOf(in Shell theShell)
    {
        var aB = new BrepBuilder();
        Solid aSolid = new();
        aB.MakeSolid(ref aSolid);
        aB.Add(aSolid, theShell);
        return aSolid;
    }

    /// <summary>A compound of the given shapes.</summary>
    public static Compound CompoundOf(params Shape[] theShapes)
    {
        var aB = new BrepBuilder();
        Compound aC = new();
        aB.MakeCompound(ref aC);
        foreach (Shape aS in theShapes)
        {
            aB.Add(aC, aS);
        }

        return aC;
    }

    /// <summary>The faces of the shape, in exploration order.</summary>
    public static List<Face> Faces(in Shape theShape)
    {
        var aFaces = new List<Face>();
        for (var anExp = new Explorer(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aFaces.Add(ShapeCasts.Face(anExp.Current()));
        }

        return aFaces;
    }

    /// <summary>
    /// Attaches to each planar polygon face of a box (TestShapes.Box) a triangulation of its quadrangle (two
    /// triangles), wound as the face's surface: the mesh is exact.
    /// </summary>
    public static void AttachQuadMeshes(in Shape theBox)
    {
        var aB = new BrepBuilder();
        foreach (Face aF in Faces(theBox))
        {
            Face aFwd = ShapeCasts.Face(aF.Oriented(Forward));
            // the corners in wire order, which runs counter-clockwise around the plane's normal
            var aCorners = new List<Point3D>();
            for (var aW = new Explorer(aFwd, ShapeType.Edge); aW.More(); aW.Next())
            {
                Edge anE = ShapeCasts.Edge(aW.Current());
                Exploration.Vertices(anE, out Vertex aFirst, out _, true);
                aCorners.Add(Tool.Pnt(aFirst));
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
    public static Face MeshFace(Point3D[] theNodes, params (int, int, int)[] theTriangles)
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

        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, aTri);
        return aF;
    }

    /// <summary>An edge with no curve carrying a 3D polygon through the points.</summary>
    public static Edge PolygonEdge(params Point3D[] thePoints)
    {
        var aNodes = new NCollection_Array1<Point3D>(1, thePoints.Length);
        for (int i = 0; i < thePoints.Length; i++)
        {
            aNodes[i + 1] = thePoints[i];
        }

        var aB = new BrepBuilder();
        Edge anE = new();
        aB.MakeEdge(ref anE, new Polygon3D(aNodes));
        return anE;
    }
}
