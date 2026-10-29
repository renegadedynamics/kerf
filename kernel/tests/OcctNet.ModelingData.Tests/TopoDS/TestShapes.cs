// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

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
    public static TopoDS_Shape Frozen(TopoDS_Shape theShape)
    {
        theShape.TShape()!.FreezeGraph();
        return theShape;
    }

    public static TopoDS_Vertex Frozen(TopoDS_Vertex theShape) => TopoDS.Vertex(Frozen((TopoDS_Shape)theShape));

    public static TopoDS_Edge Frozen(TopoDS_Edge theShape) => TopoDS.Edge(Frozen((TopoDS_Shape)theShape));

    public static TopoDS_Wire Frozen(TopoDS_Wire theShape) => TopoDS.Wire(Frozen((TopoDS_Shape)theShape));

    public static TopoDS_Face Frozen(TopoDS_Face theShape) => TopoDS.Face(Frozen((TopoDS_Shape)theShape));

    public static TopoDS_Solid Frozen(TopoDS_Solid theShape) => TopoDS.Solid(Frozen((TopoDS_Shape)theShape));

    public static TopoDS_Compound Frozen(TopoDS_Compound theShape) => TopoDS.Compound(Frozen((TopoDS_Shape)theShape));

    /// <summary>A vertex at <paramref name="theP"/>, tolerance Precision::Confusion() (BRepLib_MakeVertex's).</summary>
    public static TopoDS_Vertex Vertex(in gp_Pnt theP)
    {
        var aB = new BRep_Builder();
        TopoDS_Vertex aV = new TopoDS_Vertex();
        aB.MakeVertex(ref aV, theP, Precision.Confusion());
        return aV;
    }

    /// <summary>An edge on <paramref name="theC"/> over [p1, p2], with vertices at its ends (shared if they coincide).</summary>
    public static TopoDS_Edge Edge(Geom_Curve theC, double p1, double p2)
    {
        var aB = new BRep_Builder();
        gp_Pnt aP1 = theC.Value(p1), aP2 = theC.Value(p2);
        TopoDS_Vertex aV1 = Vertex(aP1);
        TopoDS_Vertex aV2 = aP1.Distance(aP2) <= Precision.Confusion() ? aV1 : Vertex(aP2);
        return Edge(theC, p1, p2, aV1, aV2);
    }

    /// <summary>An edge on <paramref name="theC"/> over [p1, p2] between the given vertices.</summary>
    public static TopoDS_Edge Edge(Geom_Curve theC, double p1, double p2, in TopoDS_Vertex theV1, in TopoDS_Vertex theV2)
    {
        var aB = new BRep_Builder();
        TopoDS_Edge anE = new TopoDS_Edge();
        aB.MakeEdge(ref anE, theC, Precision.Confusion());
        aB.Add(anE, theV1.Oriented(TopAbs_FORWARD));
        aB.Add(anE, theV2.Oriented(TopAbs_REVERSED));
        aB.Range(anE, p1, p2);
        return anE;
    }

    /// <summary>A segment from <paramref name="theP1"/> to <paramref name="theP2"/> (a line parametrised by length).</summary>
    public static TopoDS_Edge Edge(in gp_Pnt theP1, in gp_Pnt theP2)
        => Edge(new Geom_Line(theP1, new gp_Dir(new gp_Vec(theP1, theP2))), 0.0, theP1.Distance(theP2));

    /// <summary>A segment between two vertices.</summary>
    public static TopoDS_Edge Edge(in TopoDS_Vertex theV1, in TopoDS_Vertex theV2)
    {
        gp_Pnt aP1 = BRep_Tool.Pnt(theV1), aP2 = BRep_Tool.Pnt(theV2);
        return Edge(new Geom_Line(aP1, new gp_Dir(new gp_Vec(aP1, aP2))), 0.0, aP1.Distance(aP2), theV1, theV2);
    }

    /// <summary>
    /// A planar face bounded by the closed polygon of the given vertices, on a plane of normal <paramref name="theNormal"/>
    /// through the first vertex, with a pcurve for each edge. The edges are taken from <paramref name="theEdges"/> if they
    /// are already there (shared with a neighbouring face), else made and added to it. On <paramref name="thePlane"/>
    /// if given.
    /// </summary>
    public static TopoDS_Face PolygonFace(TopoDS_Vertex[] theVertices, in gp_Dir theNormal,
                                          Dictionary<(int, int), TopoDS_Edge> theEdges, int[] theIndices,
                                          Geom_Plane? thePlane = null)
    {
        var aB = new BRep_Builder();
        gp_Pnt aP0 = BRep_Tool.Pnt(theVertices[theIndices[0]]);
        Geom_Plane aPlane = thePlane ?? new Geom_Plane(aP0, theNormal);
        gp_Pln aPln = aPlane.Pln();
        TopoDS_Face aF = new TopoDS_Face();
        aB.MakeFace(ref aF, aPlane, Precision.Confusion());
        TopoDS_Wire aW = new TopoDS_Wire();
        aB.MakeWire(ref aW);
        for (int i = 0; i < theIndices.Length; i++)
        {
            int a = theIndices[i], b = theIndices[(i + 1) % theIndices.Length];
            bool isReversed = a > b;
            (int, int) aKey = isReversed ? (b, a) : (a, b);
            if (!theEdges.TryGetValue(aKey, out TopoDS_Edge anEdge))
            {
                anEdge = Edge(theVertices[aKey.Item1], theVertices[aKey.Item2]);
                theEdges.Add(aKey, anEdge);
            }

            // the pcurve: the image of the edge's line in the plane, with the same parameter
            gp_Pnt aS = BRep_Tool.Pnt(theVertices[aKey.Item1]), anE = BRep_Tool.Pnt(theVertices[aKey.Item2]);
            ElSLib.Parameters(aPln, aS, out double u1, out double v1);
            ElSLib.Parameters(aPln, anE, out double u2, out double v2);
            var aPCurve = new Geom2d_Line(new gp_Pnt2d(u1, v1), new gp_Dir2d(u2 - u1, v2 - v1));
            aB.UpdateEdge(anEdge, aPCurve, aF, Precision.Confusion());
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
    public static TopoDS_Solid Box(double dx, double dy, double dz)
    {
        var aB = new BRep_Builder();
        var aV = new TopoDS_Vertex[8];
        for (int i = 0; i < 8; i++)
        {
            aV[i] = Vertex(new gp_Pnt((i & 1) != 0 ? dx : 0, (i & 2) != 0 ? dy : 0, (i & 4) != 0 ? dz : 0));
        }

        var anEdges = new Dictionary<(int, int), TopoDS_Edge>();
        TopoDS_Shell aShell = new TopoDS_Shell();
        aB.MakeShell(ref aShell);
        // each face's vertices counter-clockwise seen from outside
        (int[] Indices, gp_Dir Normal)[] aFaces =
        [
            ([0, 2, 3, 1], new gp_Dir(0, 0, -1)),
            ([4, 5, 7, 6], new gp_Dir(0, 0, 1)),
            ([0, 1, 5, 4], new gp_Dir(0, -1, 0)),
            ([2, 6, 7, 3], new gp_Dir(0, 1, 0)),
            ([0, 4, 6, 2], new gp_Dir(-1, 0, 0)),
            ([1, 3, 7, 5], new gp_Dir(1, 0, 0)),
        ];
        foreach ((int[] anIndices, gp_Dir aNormal) in aFaces)
        {
            aB.Add(aShell, PolygonFace(aV, aNormal, anEdges, anIndices));
        }

        aShell.Closed(true);
        TopoDS_Solid aSolid = new TopoDS_Solid();
        aB.MakeSolid(ref aSolid);
        aB.Add(aSolid, aShell);
        return aSolid;
    }

    /// <summary>
    /// The lateral face of a cylinder of radius <paramref name="theR"/> and height <paramref name="theH"/> around the
    /// Z axis: a bottom and a top circle, each with one vertex, and a seam at U = 0 with its two pcurves (U = 2 Pi for
    /// the FORWARD seam, U = 0 for the REVERSED one). Unfrozen.
    /// </summary>
    public static TopoDS_Face CylinderFace(double theR, double theH)
    {
        var aB = new BRep_Builder();
        var aSurf = new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), theR);
        double aTwoPi = 2.0 * Math.PI;
        TopoDS_Vertex aV0 = Vertex(new gp_Pnt(theR, 0, 0));
        TopoDS_Vertex aV1 = Vertex(new gp_Pnt(theR, 0, theH));
        TopoDS_Edge aBottom = Edge(new Geom_Circle(new gp_Ax2(new gp_Pnt(), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), theR),
                                   0.0, aTwoPi, aV0, aV0);
        TopoDS_Edge aTop = Edge(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, theH), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), theR),
                                0.0, aTwoPi, aV1, aV1);
        TopoDS_Edge aSeam = Edge(new Geom_Line(new gp_Pnt(theR, 0, 0), new gp_Dir(0, 0, 1)), 0.0, theH, aV0, aV1);

        TopoDS_Face aF = new TopoDS_Face();
        aB.MakeFace(ref aF, aSurf, Precision.Confusion());
        aB.UpdateEdge(aBottom, new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), aF, Precision.Confusion());
        aB.UpdateEdge(aTop, new Geom2d_Line(new gp_Pnt2d(0, theH), new gp_Dir2d(1, 0)), aF, Precision.Confusion());
        aB.UpdateEdge(aSeam,
                      new Geom2d_Line(new gp_Pnt2d(aTwoPi, 0), new gp_Dir2d(0, 1)),
                      new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(0, 1)),
                      aF,
                      Precision.Confusion());

        TopoDS_Wire aW = new TopoDS_Wire();
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
