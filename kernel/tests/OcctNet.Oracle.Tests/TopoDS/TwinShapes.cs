// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace OcctNet.Oracle.Tests;

/// <summary>A native shape and the port's, built by the same calls.</summary>
internal readonly record struct Twin<TN, TP>(TN N, TP P);

/// <summary>
/// Builds shapes with BRep_Builder on both sides at once, call for call, from the same raw numbers: vertices, segments,
/// planar faces with pcurves, boxes, cylinder faces with a seam, and compounds, at locations. The geometry is built from
/// raw coordinates on both sides (never from the port's normalised values), so both sides hold the same numbers.
/// </summary>
internal sealed class TwinShapes
{
    private readonly N.BRep_Builder myN = new();
    private readonly BRep_Builder myP = new();

    public const double Tol = 1e-7;

    public static N.TopAbs_Orientation ToNative(TopAbs_Orientation theO) => (N.TopAbs_Orientation)(int)theO;

    public static N.TopAbs_ShapeEnum ToNative(TopAbs_ShapeEnum theT) => (N.TopAbs_ShapeEnum)(int)theT;

    public Twin<N.TopoDS_Vertex, TopoDS_Vertex> Vertex(in gp_Pnt theP, double theTol = Tol)
    {
        var aNV = new N.TopoDS_Vertex();
        using var aNP = Oracle.ToNative(theP);
        myN.MakeVertex(aNV, aNP, theTol);
        TopoDS_Vertex aV = new();
        myP.MakeVertex(ref aV, theP, theTol);
        return new(aNV, aV);
    }

    /// <summary>A segment between the vertices at the given points (a line from the raw difference, parametrised by length).</summary>
    public Twin<N.TopoDS_Edge, TopoDS_Edge> Segment(Twin<N.TopoDS_Vertex, TopoDS_Vertex> theV1, in gp_Pnt theP1,
                                                    Twin<N.TopoDS_Vertex, TopoDS_Vertex> theV2, in gp_Pnt theP2)
    {
        double dx = theP2.X() - theP1.X(), dy = theP2.Y() - theP1.Y(), dz = theP2.Z() - theP1.Z();
        using var aNP = Oracle.ToNative(theP1);
        using var aND = new N.gp_Dir(dx, dy, dz);
        var aNLine = new N.Geom_Line(aNP, aND);
        var aLine = new Geom_Line(theP1, new gp_Dir(dx, dy, dz));
        double aLength = theP1.Distance(theP2);
        return EdgeOn(aNLine, aLine, new TopLoc_Location(), null, 0.0, aLength, theV1, theV2);
    }

    /// <summary>An edge on the curve (at a location for it) over [p1, p2] between the vertices.</summary>
    public Twin<N.TopoDS_Edge, TopoDS_Edge> EdgeOn(N.Geom_Curve theNC, Geom_Curve theC, in TopLoc_Location theL, N.TopLoc_Location? theNL,
                                                   double p1, double p2,
                                                   Twin<N.TopoDS_Vertex, TopoDS_Vertex> theV1, Twin<N.TopoDS_Vertex, TopoDS_Vertex> theV2)
    {
        var aNE = new N.TopoDS_Edge();
        if (theNL is null)
        {
            myN.MakeEdge(aNE, theNC, Tol);
        }
        else
        {
            myN.MakeEdge(aNE, theNC, theNL, Tol);
        }

        myN.Add(aNE, theV1.N.Oriented(N.TopAbs_Orientation.TopAbs_FORWARD));
        myN.Add(aNE, theV2.N.Oriented(N.TopAbs_Orientation.TopAbs_REVERSED));
        myN.Range(aNE, p1, p2);

        TopoDS_Edge anE = new();
        myP.MakeEdge(ref anE, theC, theL, Tol);
        myP.Add(anE, theV1.P.Oriented(TopAbs_FORWARD));
        myP.Add(anE, theV2.P.Oriented(TopAbs_REVERSED));
        myP.Range(anE, p1, p2);
        return new(aNE, anE);
    }

    /// <summary>
    /// A planar face bounded by closed polygons of the given points (first the outer one) on a plane through
    /// <paramref name="theOrigin"/> with the raw normal and X direction, with pcurves computed by the port and given to
    /// both sides. Edges are shared through <paramref name="theEdges"/> (keyed by vertex indices).
    /// </summary>
    public Twin<N.TopoDS_Face, TopoDS_Face> PolygonFace(gp_Pnt[] thePoints, Twin<N.TopoDS_Vertex, TopoDS_Vertex>[] theVertices,
                                                         Dictionary<(int, int), Twin<N.TopoDS_Edge, TopoDS_Edge>> theEdges,
                                                         in gp_Pnt theOrigin, in gp_XYZ theNormal, in gp_XYZ theXDir,
                                                         params int[][] theLoops)
    {
        using var aNO = Oracle.ToNative(theOrigin);
        using var aNN = new N.gp_Dir(theNormal.X(), theNormal.Y(), theNormal.Z());
        using var aNX = new N.gp_Dir(theXDir.X(), theXDir.Y(), theXDir.Z());
        using var aNAx = new N.gp_Ax3(aNO, aNN, aNX);
        var aNPlane = new N.Geom_Plane(aNAx);
        var aPlane = new Geom_Plane(new gp_Ax3(theOrigin, new gp_Dir(theNormal.X(), theNormal.Y(), theNormal.Z()),
                                               new gp_Dir(theXDir.X(), theXDir.Y(), theXDir.Z())));
        gp_Pln aPln = aPlane.Pln();

        var aNF = new N.TopoDS_Face();
        myN.MakeFace(aNF, aNPlane, Tol);
        TopoDS_Face aF = new();
        myP.MakeFace(ref aF, aPlane, Tol);

        foreach (int[] aLoop in theLoops)
        {
            var aNW = new N.TopoDS_Wire();
            myN.MakeWire(aNW);
            TopoDS_Wire aW = new();
            myP.MakeWire(ref aW);
            for (int i = 0; i < aLoop.Length; i++)
            {
                int a = aLoop[i], b = aLoop[(i + 1) % aLoop.Length];
                bool isReversed = a > b;
                (int, int) aKey = isReversed ? (b, a) : (a, b);
                if (!theEdges.TryGetValue(aKey, out Twin<N.TopoDS_Edge, TopoDS_Edge> anEdge))
                {
                    anEdge = Segment(theVertices[aKey.Item1], thePoints[aKey.Item1], theVertices[aKey.Item2], thePoints[aKey.Item2]);
                    theEdges.Add(aKey, anEdge);
                }

                // the pcurve: the image of the edge's line, with the same parameter (computed once, given to both)
                ElSLib.Parameters(aPln, thePoints[aKey.Item1], out double u1, out double v1);
                ElSLib.Parameters(aPln, thePoints[aKey.Item2], out double u2, out double v2);
                using (var aNP2 = new N.gp_Pnt2d(u1, v1))
                using (var aND2 = new N.gp_Dir2d(u2 - u1, v2 - v1))
                {
                    myN.UpdateEdge(anEdge.N, new N.Geom2d_Line(aNP2, aND2), aNF, Tol);
                }

                myP.UpdateEdge(anEdge.P, new Geom2d_Line(new gp_Pnt2d(u1, v1), new gp_Dir2d(u2 - u1, v2 - v1)), aF, Tol);
                myN.Add(aNW, isReversed ? anEdge.N.Reversed() : anEdge.N);
                myP.Add(aW, isReversed ? anEdge.P.Reversed() : anEdge.P);
            }

            aNW.Closed(true);
            aW.Closed(true);
            myN.Add(aNF, aNW);
            myP.Add(aF, aW);
        }

        return new(aNF, aF);
    }

    /// <summary>
    /// A planar face bounded by the closed polygon of the given (u, v) corners, on a plane through
    /// <paramref name="theOrigin"/> with the raw normal and X direction, whose edges have pcurves and no 3D curves (each
    /// pcurve a line from one corner to the next, parametrised by length over [0, length]).
    /// </summary>
    public Twin<N.TopoDS_Face, TopoDS_Face> PCurveOnlyFace(in gp_Pnt theOrigin, in gp_XYZ theNormal, in gp_XYZ theXDir,
                                                           params (double U, double V)[] theCorners)
    {
        using var aNO = Oracle.ToNative(theOrigin);
        using var aNN = new N.gp_Dir(theNormal.X(), theNormal.Y(), theNormal.Z());
        using var aNX = new N.gp_Dir(theXDir.X(), theXDir.Y(), theXDir.Z());
        using var aNAx = new N.gp_Ax3(aNO, aNN, aNX);
        var aNPlane = new N.Geom_Plane(aNAx);
        var aPlane = new Geom_Plane(new gp_Ax3(theOrigin, new gp_Dir(theNormal.X(), theNormal.Y(), theNormal.Z()),
                                               new gp_Dir(theXDir.X(), theXDir.Y(), theXDir.Z())));
        gp_Pln aPln = aPlane.Pln();

        var aNF = new N.TopoDS_Face();
        myN.MakeFace(aNF, aNPlane, Tol);
        TopoDS_Face aF = new();
        myP.MakeFace(ref aF, aPlane, Tol);

        int n = theCorners.Length;
        var aV = new Twin<N.TopoDS_Vertex, TopoDS_Vertex>[n];
        for (int i = 0; i < n; i++)
        {
            aV[i] = Vertex(ElSLib.Value(theCorners[i].U, theCorners[i].V, aPln), 1e-6);
        }

        var aNW = new N.TopoDS_Wire();
        myN.MakeWire(aNW);
        TopoDS_Wire aW = new();
        myP.MakeWire(ref aW);
        for (int i = 0; i < n; i++)
        {
            (double u1, double v1) = theCorners[i];
            (double u2, double v2) = theCorners[(i + 1) % n];
            double aLength = Math.Sqrt((u2 - u1) * (u2 - u1) + (v2 - v1) * (v2 - v1));
            var aNE = new N.TopoDS_Edge();
            myN.MakeEdge(aNE);
            using (var aNP2 = new N.gp_Pnt2d(u1, v1))
            using (var aND2 = new N.gp_Dir2d(u2 - u1, v2 - v1))
            {
                myN.UpdateEdge(aNE, new N.Geom2d_Line(aNP2, aND2), aNF, Tol);
            }

            myN.Range(aNE, 0.0, aLength);
            myN.Add(aNE, aV[i].N.Oriented(N.TopAbs_Orientation.TopAbs_FORWARD));
            myN.Add(aNE, aV[(i + 1) % n].N.Oriented(N.TopAbs_Orientation.TopAbs_REVERSED));
            myN.Add(aNW, aNE);

            TopoDS_Edge anE = new();
            myP.MakeEdge(ref anE);
            myP.UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(u1, v1), new gp_Dir2d(u2 - u1, v2 - v1)), aF, Tol);
            myP.Range(anE, 0.0, aLength);
            myP.Add(anE, aV[i].P.Oriented(TopAbs_FORWARD));
            myP.Add(anE, aV[(i + 1) % n].P.Oriented(TopAbs_REVERSED));
            myP.Add(aW, anE);
        }

        aNW.Closed(true);
        aW.Closed(true);
        myN.Add(aNF, aNW);
        myP.Add(aF, aW);
        return new(aNF, aF);
    }

    /// <summary>
    /// A face on the surface bounded by one closed wire of the given edges, in order, each reversed in the wire or not
    /// and given its pcurve on the face.
    /// </summary>
    public Twin<N.TopoDS_Face, TopoDS_Face> FaceOn(N.Geom_Surface theNS, Geom_Surface theS,
                                                   params (Twin<N.TopoDS_Edge, TopoDS_Edge> Edge, bool Reversed, N.Geom2d_Curve NC, Geom2d_Curve C)[] theEdges)
    {
        var aNF = new N.TopoDS_Face();
        myN.MakeFace(aNF, theNS, Tol);
        TopoDS_Face aF = new();
        myP.MakeFace(ref aF, theS, Tol);
        var aNW = new N.TopoDS_Wire();
        myN.MakeWire(aNW);
        TopoDS_Wire aW = new();
        myP.MakeWire(ref aW);
        foreach ((Twin<N.TopoDS_Edge, TopoDS_Edge> anEdge, bool isReversed, N.Geom2d_Curve aNC, Geom2d_Curve aC) in theEdges)
        {
            myN.UpdateEdge(anEdge.N, aNC, aNF, Tol);
            myP.UpdateEdge(anEdge.P, aC, aF, Tol);
            myN.Add(aNW, isReversed ? anEdge.N.Reversed() : anEdge.N);
            myP.Add(aW, isReversed ? anEdge.P.Reversed() : anEdge.P);
        }

        aNW.Closed(true);
        aW.Closed(true);
        myN.Add(aNF, aNW);
        myP.Add(aF, aW);
        return new(aNF, aF);
    }

    /// <summary>A wire of the edges, in this order.</summary>
    public Twin<N.TopoDS_Wire, TopoDS_Wire> Wire(bool theClosed, params Twin<N.TopoDS_Edge, TopoDS_Edge>[] theEdges)
    {
        var aNW = new N.TopoDS_Wire();
        myN.MakeWire(aNW);
        TopoDS_Wire aW = new();
        myP.MakeWire(ref aW);
        foreach (Twin<N.TopoDS_Edge, TopoDS_Edge> anE in theEdges)
        {
            myN.Add(aNW, anE.N);
            myP.Add(aW, anE.P);
        }

        aNW.Closed(theClosed);
        aW.Closed(theClosed);
        return new(aNW, aW);
    }

    /// <summary>A box [0, dx] x [0, dy] x [0, dz]: 8 vertices, 12 edges, 6 faces, a closed shell and a solid.</summary>
    public Twin<N.TopoDS_Solid, TopoDS_Solid> Box(double dx, double dy, double dz)
    {
        var aPoints = new gp_Pnt[8];
        var aV = new Twin<N.TopoDS_Vertex, TopoDS_Vertex>[8];
        for (int i = 0; i < 8; i++)
        {
            aPoints[i] = new gp_Pnt((i & 1) != 0 ? dx : 0, (i & 2) != 0 ? dy : 0, (i & 4) != 0 ? dz : 0);
            aV[i] = Vertex(aPoints[i]);
        }

        var anEdges = new Dictionary<(int, int), Twin<N.TopoDS_Edge, TopoDS_Edge>>();
        var aNShell = new N.TopoDS_Shell();
        myN.MakeShell(aNShell);
        TopoDS_Shell aShell = new();
        myP.MakeShell(ref aShell);
        (int[] Indices, gp_XYZ Normal, gp_XYZ XDir)[] aFaces =
        [
            ([0, 2, 3, 1], new gp_XYZ(0, 0, -1), new gp_XYZ(0, 1, 0)),
            ([4, 5, 7, 6], new gp_XYZ(0, 0, 1), new gp_XYZ(1, 0, 0)),
            ([0, 1, 5, 4], new gp_XYZ(0, -1, 0), new gp_XYZ(1, 0, 0)),
            ([2, 6, 7, 3], new gp_XYZ(0, 1, 0), new gp_XYZ(0, 0, 1)),
            ([0, 4, 6, 2], new gp_XYZ(-1, 0, 0), new gp_XYZ(0, 0, 1)),
            ([1, 3, 7, 5], new gp_XYZ(1, 0, 0), new gp_XYZ(0, 1, 0)),
        ];
        foreach ((int[] anIndices, gp_XYZ aNormal, gp_XYZ anX) in aFaces)
        {
            Twin<N.TopoDS_Face, TopoDS_Face> aF = PolygonFace(aPoints, aV, anEdges, aPoints[anIndices[0]], aNormal, anX, anIndices);
            myN.Add(aNShell, aF.N);
            myP.Add(aShell, aF.P);
        }

        aNShell.Closed(true);
        aShell.Closed(true);
        var aNSolid = new N.TopoDS_Solid();
        myN.MakeSolid(aNSolid);
        myN.Add(aNSolid, aNShell);
        TopoDS_Solid aSolid = new();
        myP.MakeSolid(ref aSolid);
        myP.Add(aSolid, aShell);
        return new(aNSolid, aSolid);
    }

    /// <summary>
    /// The lateral face of a cylinder of radius r and height h, with its surface and curves placed at the location
    /// (the face's own location is the identity): bottom and top circles, each with one vertex, and a seam with two
    /// pcurves (U = 2 Pi FORWARD, U = 0 REVERSED).
    /// </summary>
    public Twin<N.TopoDS_Face, TopoDS_Face> CylinderFace(double r, double h, in TopLoc_Location theL, N.TopLoc_Location theNL)
    {
        double aTwoPi = 2.0 * Math.PI;
        gp_Trsf aT = theL.Transformation();
        gp_Pnt aP0 = new gp_Pnt(r, 0, 0).Transformed(aT), aP1 = new gp_Pnt(r, 0, h).Transformed(aT);
        // the vertices are where the located curves put them; both sides use the port's coordinates
        Twin<N.TopoDS_Vertex, TopoDS_Vertex> aV0 = Vertex(aP0, 1e-6), aV1 = Vertex(aP1, 1e-6);

        using var aNO = new N.gp_Pnt(0, 0, 0);
        using var aNZ = new N.gp_Dir(0, 0, 1);
        using var aNXd = new N.gp_Dir(1, 0, 0);
        using var aNAx3 = new N.gp_Ax3(aNO, aNZ, aNXd);
        var aNSurf = new N.Geom_CylindricalSurface(aNAx3, r);
        var aSurf = new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), r);

        using var aNAx2 = new N.gp_Ax2(aNO, aNZ, aNXd);
        using var aNO2 = new N.gp_Pnt(0, 0, h);
        using var aNAx2Top = new N.gp_Ax2(aNO2, aNZ, aNXd);
        Twin<N.TopoDS_Edge, TopoDS_Edge> aBottom =
            EdgeOn(new N.Geom_Circle(aNAx2, r), new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), r),
                   theL, theNL, 0.0, aTwoPi, aV0, aV0);
        Twin<N.TopoDS_Edge, TopoDS_Edge> aTop =
            EdgeOn(new N.Geom_Circle(aNAx2Top, r), new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, h), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), r),
                   theL, theNL, 0.0, aTwoPi, aV1, aV1);
        using var aNSeamP = new N.gp_Pnt(r, 0, 0);
        Twin<N.TopoDS_Edge, TopoDS_Edge> aSeam =
            EdgeOn(new N.Geom_Line(aNSeamP, aNZ), new Geom_Line(new gp_Pnt(r, 0, 0), new gp_Dir(0, 0, 1)), theL, theNL, 0.0, h, aV0, aV1);

        var aNF = new N.TopoDS_Face();
        myN.MakeFace(aNF, aNSurf, theNL, Tol);
        TopoDS_Face aF = new();
        myP.MakeFace(ref aF, aSurf, theL, Tol);

        using (var aNPa = new N.gp_Pnt2d(0, 0))
        using (var aNPb = new N.gp_Pnt2d(0, h))
        using (var aNPc = new N.gp_Pnt2d(aTwoPi, 0))
        using (var aNU = new N.gp_Dir2d(1, 0))
        using (var aNV = new N.gp_Dir2d(0, 1))
        {
            myN.UpdateEdge(aBottom.N, new N.Geom2d_Line(aNPa, aNU), aNF, Tol);
            myN.UpdateEdge(aTop.N, new N.Geom2d_Line(aNPb, aNU), aNF, Tol);
            myN.UpdateEdge(aSeam.N, new N.Geom2d_Line(aNPc, aNV), new N.Geom2d_Line(aNPa, aNV), aNF, Tol);
        }

        myP.UpdateEdge(aBottom.P, new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), aF, Tol);
        myP.UpdateEdge(aTop.P, new Geom2d_Line(new gp_Pnt2d(0, h), new gp_Dir2d(1, 0)), aF, Tol);
        myP.UpdateEdge(aSeam.P, new Geom2d_Line(new gp_Pnt2d(aTwoPi, 0), new gp_Dir2d(0, 1)),
                       new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(0, 1)), aF, Tol);

        var aNW = new N.TopoDS_Wire();
        myN.MakeWire(aNW);
        myN.Add(aNW, aBottom.N);
        myN.Add(aNW, aSeam.N);
        myN.Add(aNW, aTop.N.Reversed());
        myN.Add(aNW, aSeam.N.Reversed());
        aNW.Closed(true);
        myN.Add(aNF, aNW);
        TopoDS_Wire aW = new();
        myP.MakeWire(ref aW);
        myP.Add(aW, aBottom.P);
        myP.Add(aW, aSeam.P);
        myP.Add(aW, aTop.P.Reversed());
        myP.Add(aW, aSeam.P.Reversed());
        aW.Closed(true);
        myP.Add(aF, aW);
        return new(aNF, aF);
    }

    /// <summary>A compound of the given shapes.</summary>
    public Twin<N.TopoDS_Compound, TopoDS_Compound> Compound(params Twin<N.TopoDS_Shape, TopoDS_Shape>[] theShapes)
    {
        var aNC = new N.TopoDS_Compound();
        myN.MakeCompound(aNC);
        TopoDS_Compound aC = new();
        myP.MakeCompound(ref aC);
        foreach (Twin<N.TopoDS_Shape, TopoDS_Shape> aS in theShapes)
        {
            myN.Add(aNC, aS.N);
            myP.Add(aC, aS.P);
        }

        return new(aNC, aC);
    }

    /// <summary>Attaches a one-triangle triangulation of the given deflection to the face, on both sides.</summary>
    public void AttachTriangulation(Twin<N.TopoDS_Face, TopoDS_Face> theFace, double theDeflection)
    {
        var aNT = new N.Poly_Triangulation(3, 1, false);
        var aT = new Poly_Triangulation(3, 1, false);
        for (int i = 1; i <= 3; i++)
        {
            var aP = new gp_Pnt(i == 2 ? 1 : 0, i == 3 ? 1 : 0, 0);
            using var aNP = Oracle.ToNative(aP);
            aNT.SetNode(i, aNP);
            aT.SetNode(i, aP);
        }

        using (var aNTri = new N.Poly_Triangle(1, 2, 3))
        {
            aNT.SetTriangle(1, aNTri);
        }

        aT.SetTriangle(1, new Poly_Triangle(1, 2, 3));
        aNT.Deflection(theDeflection);
        aT.Deflection(theDeflection);
        myN.UpdateFace(theFace.N, aNT);
        myP.UpdateFace(theFace.P, aT);
    }

    /// <summary>Adds the child to the shape on both sides.</summary>
    public void Add(Twin<N.TopoDS_Shape, TopoDS_Shape> theShape, Twin<N.TopoDS_Shape, TopoDS_Shape> theChild)
    {
        myN.Add(theShape.N, theChild.N);
        myP.Add(theShape.P, theChild.P);
    }

    public static Twin<N.TopoDS_Shape, TopoDS_Shape> Up<TN, TP>(Twin<TN, TP> theTwin)
        where TN : N.TopoDS_Shape
        where TP : struct
        => new(theTwin.N, ToShape(theTwin.P));

    private static TopoDS_Shape ToShape<TP>(TP theP) => theP switch
    {
        TopoDS_Shape s => s,
        TopoDS_Vertex s => s,
        TopoDS_Edge s => s,
        TopoDS_Wire s => s,
        TopoDS_Face s => s,
        TopoDS_Shell s => s,
        TopoDS_Solid s => s,
        TopoDS_CompSolid s => s,
        TopoDS_Compound s => s,
        _ => throw new ArgumentException("not a shape"),
    };
}
