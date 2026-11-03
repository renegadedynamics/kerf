// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Shapes built with BRep_Builder on purpose for the unit tests (the ported GTests use BRepPrimAPI's makers): unfrozen
/// shapes the tests change with BRep_Builder, and shapes of a known structure: a vertex, an edge with its
/// two vertices (one, used twice, on a closed curve), and a box of 8 vertices, 12 edges, 6 wires, 6 faces with pcurves
/// on planes, a shell and a solid.
/// </summary>
/// <remarks>
/// The shapes are unfrozen unless asked; <c>Frozen</c> freezes one as a public maker will.
/// </remarks>
internal static class TestShapes
{
    /// <summary>The shape, frozen as a public algorithm's result is.</summary>
    public static Shape Frozen(Shape theShape)
    {
        theShape.TShape()!.FreezeGraph();
        return theShape;
    }

    public static Vertex Frozen(Vertex theShape) => ShapeCasts.Vertex(Frozen((Shape)theShape));

    public static Edge Frozen(Edge theShape) => ShapeCasts.Edge(Frozen((Shape)theShape));

    public static Wire Frozen(Wire theShape) => ShapeCasts.Wire(Frozen((Shape)theShape));

    public static Face Frozen(Face theShape) => ShapeCasts.Face(Frozen((Shape)theShape));

    public static Solid Frozen(Solid theShape) => ShapeCasts.Solid(Frozen((Shape)theShape));

    public static Compound Frozen(Compound theShape) => ShapeCasts.Compound(Frozen((Shape)theShape));

    /// <summary>A vertex at <paramref name="theP"/>, tolerance Precision::Confusion() (BRepLib_MakeVertex's).</summary>
    public static Vertex Vertex(in Point3D theP)
    {
        var aB = new BrepBuilder();
        Vertex aV = new Vertex();
        aB.MakeVertex(ref aV, theP, global::Kerf.Engine.Foundation.Precision.Confusion());
        return aV;
    }

    /// <summary>An edge on <paramref name="theC"/> over [p1, p2], with vertices at its ends (shared if they coincide).</summary>
    public static Edge Edge(Curve3D theC, double p1, double p2)
    {
        var aB = new BrepBuilder();
        Point3D aP1 = theC.Value(p1), aP2 = theC.Value(p2);
        Vertex aV1 = Vertex(aP1);
        Vertex aV2 = aP1.Distance(aP2) <= global::Kerf.Engine.Foundation.Precision.Confusion() ? aV1 : Vertex(aP2);
        return Edge(theC, p1, p2, aV1, aV2);
    }

    /// <summary>An edge on <paramref name="theC"/> over [p1, p2] between the given vertices.</summary>
    public static Edge Edge(Curve3D theC, double p1, double p2, in Vertex theV1, in Vertex theV2)
    {
        var aB = new BrepBuilder();
        Edge anE = new Edge();
        aB.MakeEdge(ref anE, theC, global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.Add(anE, theV1.Oriented(Forward));
        aB.Add(anE, theV2.Oriented(Reversed));
        aB.Range(anE, p1, p2);
        return anE;
    }

    /// <summary>A segment from <paramref name="theP1"/> to <paramref name="theP2"/> (a line parametrised by length).</summary>
    public static Edge Edge(in Point3D theP1, in Point3D theP2)
        => Edge(new LineCurve3D(theP1, new Direction3D(new Vector3D(theP1, theP2))), 0.0, theP1.Distance(theP2));

    /// <summary>A segment between two vertices.</summary>
    public static Edge Edge(in Vertex theV1, in Vertex theV2)
    {
        Point3D aP1 = Tool.Pnt(theV1), aP2 = Tool.Pnt(theV2);
        return Edge(new LineCurve3D(aP1, new Direction3D(new Vector3D(aP1, aP2))), 0.0, aP1.Distance(aP2), theV1, theV2);
    }

    /// <summary>
    /// A planar face bounded by the closed polygon of the given vertices, on a plane of normal <paramref name="theNormal"/>
    /// through the first vertex, with a pcurve for each edge. The edges are taken from <paramref name="theEdges"/> if they
    /// are already there (shared with a neighbouring face), else made and added to it. On <paramref name="thePlane"/>
    /// if given.
    /// </summary>
    public static Face PolygonFace(Vertex[] theVertices, in Direction3D theNormal,
                                          Dictionary<(int, int), Edge> theEdges, int[] theIndices,
                                          PlaneSurface? thePlane = null)
    {
        var aB = new BrepBuilder();
        Point3D aP0 = Tool.Pnt(theVertices[theIndices[0]]);
        PlaneSurface aPlane = thePlane ?? new PlaneSurface(aP0, theNormal);
        Plane3D aPln = aPlane.Pln();
        Face aF = new Face();
        aB.MakeFace(ref aF, aPlane, global::Kerf.Engine.Foundation.Precision.Confusion());
        Wire aW = new Wire();
        aB.MakeWire(ref aW);
        for (int i = 0; i < theIndices.Length; i++)
        {
            int a = theIndices[i], b = theIndices[(i + 1) % theIndices.Length];
            bool isReversed = a > b;
            (int, int) aKey = isReversed ? (b, a) : (a, b);
            if (!theEdges.TryGetValue(aKey, out Edge anEdge))
            {
                anEdge = Edge(theVertices[aKey.Item1], theVertices[aKey.Item2]);
                theEdges.Add(aKey, anEdge);
            }

            // the pcurve: the image of the edge's line in the plane, with the same parameter
            Point3D aS = Tool.Pnt(theVertices[aKey.Item1]), anE = Tool.Pnt(theVertices[aKey.Item2]);
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aPln, aS, out double u1, out double v1);
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aPln, anE, out double u2, out double v2);
            var aPCurve = new LineCurve2D(new Point2D(u1, v1), new Direction2D(u2 - u1, v2 - v1));
            aB.UpdateEdge(anEdge, aPCurve, aF, global::Kerf.Engine.Foundation.Precision.Confusion());
            aB.Add(aW, isReversed ? anEdge.Reversed() : anEdge);
        }

        aW.Closed(true);
        aB.Add(aF, aW);
        return aF;
    }

    /// <summary>
    /// A box [0, dx] x [0, dy] x [0, dz]: 8 vertices, 12 edges, 6 faces with outward normals, a closed shell and a
    /// solid. Unfrozen.
    /// </summary>
    public static Solid Box(double dx, double dy, double dz)
    {
        var aB = new BrepBuilder();
        var aV = new Vertex[8];
        for (int i = 0; i < 8; i++)
        {
            aV[i] = Vertex(new Point3D((i & 1) != 0 ? dx : 0, (i & 2) != 0 ? dy : 0, (i & 4) != 0 ? dz : 0));
        }

        var anEdges = new Dictionary<(int, int), Edge>();
        Shell aShell = new Shell();
        aB.MakeShell(ref aShell);
        // each face's vertices counter-clockwise seen from outside
        (int[] Indices, Direction3D Normal)[] aFaces =
        [
            ([0, 2, 3, 1], new Direction3D(0, 0, -1)),
            ([4, 5, 7, 6], new Direction3D(0, 0, 1)),
            ([0, 1, 5, 4], new Direction3D(0, -1, 0)),
            ([2, 6, 7, 3], new Direction3D(0, 1, 0)),
            ([0, 4, 6, 2], new Direction3D(-1, 0, 0)),
            ([1, 3, 7, 5], new Direction3D(1, 0, 0)),
        ];
        foreach ((int[] anIndices, Direction3D aNormal) in aFaces)
        {
            aB.Add(aShell, PolygonFace(aV, aNormal, anEdges, anIndices));
        }

        aShell.Closed(true);
        Solid aSolid = new Solid();
        aB.MakeSolid(ref aSolid);
        aB.Add(aSolid, aShell);
        return aSolid;
    }

    /// <summary>
    /// The lateral face of a cylinder of radius <paramref name="theR"/> and height <paramref name="theH"/> around the
    /// Z axis: a bottom and a top circle, each with one vertex, and a seam at U = 0 with its two pcurves (U = 2 Pi for
    /// the FORWARD seam, U = 0 for the REVERSED one). Unfrozen.
    /// </summary>
    public static Face CylinderFace(double theR, double theH)
    {
        var aB = new BrepBuilder();
        var aSurf = new CylindricalSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), theR);
        double aTwoPi = 2.0 * Math.PI;
        Vertex aV0 = Vertex(new Point3D(theR, 0, 0));
        Vertex aV1 = Vertex(new Point3D(theR, 0, theH));
        Edge aBottom = Edge(new CircleCurve3D(new Frame3D(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), theR),
                                   0.0, aTwoPi, aV0, aV0);
        Edge aTop = Edge(new CircleCurve3D(new Frame3D(new Point3D(0, 0, theH), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), theR),
                                0.0, aTwoPi, aV1, aV1);
        Edge aSeam = Edge(new LineCurve3D(new Point3D(theR, 0, 0), new Direction3D(0, 0, 1)), 0.0, theH, aV0, aV1);

        Face aF = new Face();
        aB.MakeFace(ref aF, aSurf, global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.UpdateEdge(aBottom, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.UpdateEdge(aTop, new LineCurve2D(new Point2D(0, theH), new Direction2D(1, 0)), aF, global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.UpdateEdge(aSeam,
                      new LineCurve2D(new Point2D(aTwoPi, 0), new Direction2D(0, 1)),
                      new LineCurve2D(new Point2D(0, 0), new Direction2D(0, 1)),
                      aF,
                      global::Kerf.Engine.Foundation.Precision.Confusion());

        Wire aW = new Wire();
        aB.MakeWire(ref aW);
        aB.Add(aW, aBottom);
        aB.Add(aW, aSeam);
        aB.Add(aW, aTop.Reversed());
        aB.Add(aW, aSeam.Reversed());
        aW.Closed(true);
        aB.Add(aF, aW);
        return aF;
    }
}
