// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Kernel.Oracle.Tests;

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
    private readonly BrepBuilder myP = new();

    public const double Tol = 1e-7;

    public static N.TopAbs_Orientation ToNative(Orientation theO) => (N.TopAbs_Orientation)(int)theO;

    public static N.TopAbs_ShapeEnum ToNative(ShapeType theT) => (N.TopAbs_ShapeEnum)(int)theT;

    public Twin<N.TopoDS_Vertex, Vertex> Vertex(in Point3D theP, double theTol = Tol)
    {
        var aNV = new N.TopoDS_Vertex();
        using var aNP = Oracle.ToNative(theP);
        myN.MakeVertex(aNV, aNP, theTol);
        Vertex aV = new();
        myP.MakeVertex(ref aV, theP, theTol);
        return new(aNV, aV);
    }

    /// <summary>A segment between the vertices at the given points (a line from the raw difference, parametrised by length).</summary>
    public Twin<N.TopoDS_Edge, Edge> Segment(Twin<N.TopoDS_Vertex, Vertex> theV1, in Point3D theP1,
                                                    Twin<N.TopoDS_Vertex, Vertex> theV2, in Point3D theP2)
    {
        double dx = theP2.X - theP1.X, dy = theP2.Y - theP1.Y, dz = theP2.Z - theP1.Z;
        using var aNP = Oracle.ToNative(theP1);
        using var aND = new N.gp_Dir(dx, dy, dz);
        var aNLine = new N.Geom_Line(aNP, aND);
        var aLine = new LineCurve3D(theP1, new Direction3D(dx, dy, dz));
        double aLength = theP1.DistanceTo(theP2);
        return EdgeOn(aNLine, aLine, new Placement(), null, 0.0, aLength, theV1, theV2);
    }

    /// <summary>An edge on the curve (at a location for it) over [p1, p2] between the vertices.</summary>
    public Twin<N.TopoDS_Edge, Edge> EdgeOn(N.Geom_Curve theNC, Curve3D theC, in Placement theL, N.TopLoc_Location? theNL,
                                                   double p1, double p2,
                                                   Twin<N.TopoDS_Vertex, Vertex> theV1, Twin<N.TopoDS_Vertex, Vertex> theV2)
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

        Edge anE = new();
        myP.MakeEdge(ref anE, theC, theL, Tol);
        myP.Add(anE, theV1.P.Oriented(Forward));
        myP.Add(anE, theV2.P.Oriented(Reversed));
        myP.Range(anE, p1, p2);
        return new(aNE, anE);
    }

    /// <summary>
    /// A planar face bounded by closed polygons of the given points (first the outer one) on a plane through
    /// <paramref name="theOrigin"/> with the raw normal and X direction, with pcurves computed by the port and given to
    /// both sides. Edges are shared through <paramref name="theEdges"/> (keyed by vertex indices).
    /// </summary>
    public Twin<N.TopoDS_Face, Face> PolygonFace(Point3D[] thePoints, Twin<N.TopoDS_Vertex, Vertex>[] theVertices,
                                                         Dictionary<(int, int), Twin<N.TopoDS_Edge, Edge>> theEdges,
                                                         in Point3D theOrigin, in global::Kerf.Engine.Geometry.Xyz theNormal, in global::Kerf.Engine.Geometry.Xyz theXDir,
                                                         params int[][] theLoops)
    {
        using var aNO = Oracle.ToNative(theOrigin);
        using var aNN = new N.gp_Dir(theNormal.X(), theNormal.Y(), theNormal.Z());
        using var aNX = new N.gp_Dir(theXDir.X(), theXDir.Y(), theXDir.Z());
        using var aNAx = new N.gp_Ax3(aNO, aNN, aNX);
        var aNPlane = new N.Geom_Plane(aNAx);
        var aPlane = new PlaneSurface(new CoordinateSystem(theOrigin, new Direction3D(theNormal.X(), theNormal.Y(), theNormal.Z()),
                                               new Direction3D(theXDir.X(), theXDir.Y(), theXDir.Z())));
        Plane3D aPln = aPlane.Plane;

        var aNF = new N.TopoDS_Face();
        myN.MakeFace(aNF, aNPlane, Tol);
        Face aF = new();
        myP.MakeFace(ref aF, aPlane, Tol);

        foreach (int[] aLoop in theLoops)
        {
            var aNW = new N.TopoDS_Wire();
            myN.MakeWire(aNW);
            Wire aW = new();
            myP.MakeWire(ref aW);
            for (int i = 0; i < aLoop.Length; i++)
            {
                int a = aLoop[i], b = aLoop[(i + 1) % aLoop.Length];
                bool isReversed = a > b;
                (int, int) aKey = isReversed ? (b, a) : (a, b);
                if (!theEdges.TryGetValue(aKey, out Twin<N.TopoDS_Edge, Edge> anEdge))
                {
                    anEdge = Segment(theVertices[aKey.Item1], thePoints[aKey.Item1], theVertices[aKey.Item2], thePoints[aKey.Item2]);
                    theEdges.Add(aKey, anEdge);
                }

                // the pcurve: the image of the edge's line, with the same parameter (computed once, given to both)
                global::Kerf.Engine.Geometry.ElSLib.Parameters(aPln, thePoints[aKey.Item1], out double u1, out double v1);
                global::Kerf.Engine.Geometry.ElSLib.Parameters(aPln, thePoints[aKey.Item2], out double u2, out double v2);
                using (var aNP2 = new N.gp_Pnt2d(u1, v1))
                using (var aND2 = new N.gp_Dir2d(u2 - u1, v2 - v1))
                {
                    myN.UpdateEdge(anEdge.N, new N.Geom2d_Line(aNP2, aND2), aNF, Tol);
                }

                myP.UpdateEdge(anEdge.P, new LineCurve2D(new Point2D(u1, v1), new Direction2D(u2 - u1, v2 - v1)), aF, Tol);
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
    public Twin<N.TopoDS_Face, Face> PCurveOnlyFace(in Point3D theOrigin, in global::Kerf.Engine.Geometry.Xyz theNormal, in global::Kerf.Engine.Geometry.Xyz theXDir,
                                                           params (double U, double V)[] theCorners)
    {
        using var aNO = Oracle.ToNative(theOrigin);
        using var aNN = new N.gp_Dir(theNormal.X(), theNormal.Y(), theNormal.Z());
        using var aNX = new N.gp_Dir(theXDir.X(), theXDir.Y(), theXDir.Z());
        using var aNAx = new N.gp_Ax3(aNO, aNN, aNX);
        var aNPlane = new N.Geom_Plane(aNAx);
        var aPlane = new PlaneSurface(new CoordinateSystem(theOrigin, new Direction3D(theNormal.X(), theNormal.Y(), theNormal.Z()),
                                               new Direction3D(theXDir.X(), theXDir.Y(), theXDir.Z())));
        Plane3D aPln = aPlane.Plane;

        var aNF = new N.TopoDS_Face();
        myN.MakeFace(aNF, aNPlane, Tol);
        Face aF = new();
        myP.MakeFace(ref aF, aPlane, Tol);

        int n = theCorners.Length;
        var aV = new Twin<N.TopoDS_Vertex, Vertex>[n];
        for (int i = 0; i < n; i++)
        {
            aV[i] = Vertex(global::Kerf.Engine.Geometry.ElSLib.Value(theCorners[i].U, theCorners[i].V, aPln), 1e-6);
        }

        var aNW = new N.TopoDS_Wire();
        myN.MakeWire(aNW);
        Wire aW = new();
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

            Edge anE = new();
            myP.MakeEdge(ref anE);
            myP.UpdateEdge(anE, new LineCurve2D(new Point2D(u1, v1), new Direction2D(u2 - u1, v2 - v1)), aF, Tol);
            myP.Range(anE, 0.0, aLength);
            myP.Add(anE, aV[i].P.Oriented(Forward));
            myP.Add(anE, aV[(i + 1) % n].P.Oriented(Reversed));
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
    public Twin<N.TopoDS_Face, Face> FaceOn(N.Geom_Surface theNS, Surface theS,
                                                   params (Twin<N.TopoDS_Edge, Edge> Edge, bool Reversed, N.Geom2d_Curve NC, Curve2D C)[] theEdges)
    {
        var aNF = new N.TopoDS_Face();
        myN.MakeFace(aNF, theNS, Tol);
        Face aF = new();
        myP.MakeFace(ref aF, theS, Tol);
        var aNW = new N.TopoDS_Wire();
        myN.MakeWire(aNW);
        Wire aW = new();
        myP.MakeWire(ref aW);
        foreach ((Twin<N.TopoDS_Edge, Edge> anEdge, bool isReversed, N.Geom2d_Curve aNC, Curve2D aC) in theEdges)
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
    public Twin<N.TopoDS_Wire, Wire> Wire(bool theClosed, params Twin<N.TopoDS_Edge, Edge>[] theEdges)
    {
        var aNW = new N.TopoDS_Wire();
        myN.MakeWire(aNW);
        Wire aW = new();
        myP.MakeWire(ref aW);
        foreach (Twin<N.TopoDS_Edge, Edge> anE in theEdges)
        {
            myN.Add(aNW, anE.N);
            myP.Add(aW, anE.P);
        }

        aNW.Closed(theClosed);
        aW.Closed(theClosed);
        return new(aNW, aW);
    }

    /// <summary>A box [0, dx] x [0, dy] x [0, dz]: 8 vertices, 12 edges, 6 faces, a closed shell and a solid.</summary>
    public Twin<N.TopoDS_Solid, Solid> Box(double dx, double dy, double dz)
    {
        var aPoints = new Point3D[8];
        var aV = new Twin<N.TopoDS_Vertex, Vertex>[8];
        for (int i = 0; i < 8; i++)
        {
            aPoints[i] = new Point3D((i & 1) != 0 ? dx : 0, (i & 2) != 0 ? dy : 0, (i & 4) != 0 ? dz : 0);
            aV[i] = Vertex(aPoints[i]);
        }

        var anEdges = new Dictionary<(int, int), Twin<N.TopoDS_Edge, Edge>>();
        var aNShell = new N.TopoDS_Shell();
        myN.MakeShell(aNShell);
        Shell aShell = new();
        myP.MakeShell(ref aShell);
        (int[] Indices, global::Kerf.Engine.Geometry.Xyz Normal, global::Kerf.Engine.Geometry.Xyz XDir)[] aFaces =
        [
            ([0, 2, 3, 1], new global::Kerf.Engine.Geometry.Xyz(0, 0, -1), new global::Kerf.Engine.Geometry.Xyz(0, 1, 0)),
            ([4, 5, 7, 6], new global::Kerf.Engine.Geometry.Xyz(0, 0, 1), new global::Kerf.Engine.Geometry.Xyz(1, 0, 0)),
            ([0, 1, 5, 4], new global::Kerf.Engine.Geometry.Xyz(0, -1, 0), new global::Kerf.Engine.Geometry.Xyz(1, 0, 0)),
            ([2, 6, 7, 3], new global::Kerf.Engine.Geometry.Xyz(0, 1, 0), new global::Kerf.Engine.Geometry.Xyz(0, 0, 1)),
            ([0, 4, 6, 2], new global::Kerf.Engine.Geometry.Xyz(-1, 0, 0), new global::Kerf.Engine.Geometry.Xyz(0, 0, 1)),
            ([1, 3, 7, 5], new global::Kerf.Engine.Geometry.Xyz(1, 0, 0), new global::Kerf.Engine.Geometry.Xyz(0, 1, 0)),
        ];
        foreach ((int[] anIndices, global::Kerf.Engine.Geometry.Xyz aNormal, global::Kerf.Engine.Geometry.Xyz anX) in aFaces)
        {
            Twin<N.TopoDS_Face, Face> aF = PolygonFace(aPoints, aV, anEdges, aPoints[anIndices[0]], aNormal, anX, anIndices);
            myN.Add(aNShell, aF.N);
            myP.Add(aShell, aF.P);
        }

        aNShell.Closed(true);
        aShell.Closed(true);
        var aNSolid = new N.TopoDS_Solid();
        myN.MakeSolid(aNSolid);
        myN.Add(aNSolid, aNShell);
        Solid aSolid = new();
        myP.MakeSolid(ref aSolid);
        myP.Add(aSolid, aShell);
        return new(aNSolid, aSolid);
    }

    /// <summary>
    /// The lateral face of a cylinder of radius r and height h, with its surface and curves placed at the location
    /// (the face's own location is the identity): bottom and top circles, each with one vertex, and a seam with two
    /// pcurves (U = 2 Pi FORWARD, U = 0 REVERSED).
    /// </summary>
    public Twin<N.TopoDS_Face, Face> CylinderFace(double r, double h, in Placement theL, N.TopLoc_Location theNL)
    {
        double aTwoPi = 2.0 * Math.PI;
        Transform3D aT = theL.Transformation;
        Point3D aP0 = new Point3D(r, 0, 0).Transformed(aT), aP1 = new Point3D(r, 0, h).Transformed(aT);
        // the vertices are where the located curves put them; both sides use the port's coordinates
        Twin<N.TopoDS_Vertex, Vertex> aV0 = Vertex(aP0, 1e-6), aV1 = Vertex(aP1, 1e-6);

        using var aNO = new N.gp_Pnt(0, 0, 0);
        using var aNZ = new N.gp_Dir(0, 0, 1);
        using var aNXd = new N.gp_Dir(1, 0, 0);
        using var aNAx3 = new N.gp_Ax3(aNO, aNZ, aNXd);
        var aNSurf = new N.Geom_CylindricalSurface(aNAx3, r);
        var aSurf = new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r);

        using var aNAx2 = new N.gp_Ax2(aNO, aNZ, aNXd);
        using var aNO2 = new N.gp_Pnt(0, 0, h);
        using var aNAx2Top = new N.gp_Ax2(aNO2, aNZ, aNXd);
        Twin<N.TopoDS_Edge, Edge> aBottom =
            EdgeOn(new N.Geom_Circle(aNAx2, r), new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r),
                   theL, theNL, 0.0, aTwoPi, aV0, aV0);
        Twin<N.TopoDS_Edge, Edge> aTop =
            EdgeOn(new N.Geom_Circle(aNAx2Top, r), new CircleCurve3D(new Frame3D(new Point3D(0, 0, h), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r),
                   theL, theNL, 0.0, aTwoPi, aV1, aV1);
        using var aNSeamP = new N.gp_Pnt(r, 0, 0);
        Twin<N.TopoDS_Edge, Edge> aSeam =
            EdgeOn(new N.Geom_Line(aNSeamP, aNZ), new LineCurve3D(new Point3D(r, 0, 0), new Direction3D(0, 0, 1)), theL, theNL, 0.0, h, aV0, aV1);

        var aNF = new N.TopoDS_Face();
        myN.MakeFace(aNF, aNSurf, theNL, Tol);
        Face aF = new();
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

        myP.UpdateEdge(aBottom.P, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, Tol);
        myP.UpdateEdge(aTop.P, new LineCurve2D(new Point2D(0, h), new Direction2D(1, 0)), aF, Tol);
        myP.UpdateEdge(aSeam.P, new LineCurve2D(new Point2D(aTwoPi, 0), new Direction2D(0, 1)),
                       new LineCurve2D(new Point2D(0, 0), new Direction2D(0, 1)), aF, Tol);

        var aNW = new N.TopoDS_Wire();
        myN.MakeWire(aNW);
        myN.Add(aNW, aBottom.N);
        myN.Add(aNW, aSeam.N);
        myN.Add(aNW, aTop.N.Reversed());
        myN.Add(aNW, aSeam.N.Reversed());
        aNW.Closed(true);
        myN.Add(aNF, aNW);
        Wire aW = new();
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
    public Twin<N.TopoDS_Compound, Compound> Compound(params Twin<N.TopoDS_Shape, Shape>[] theShapes)
    {
        var aNC = new N.TopoDS_Compound();
        myN.MakeCompound(aNC);
        Compound aC = new();
        myP.MakeCompound(ref aC);
        foreach (Twin<N.TopoDS_Shape, Shape> aS in theShapes)
        {
            myN.Add(aNC, aS.N);
            myP.Add(aC, aS.P);
        }

        return new(aNC, aC);
    }

    /// <summary>Attaches a one-triangle triangulation of the given deflection to the face, on both sides.</summary>
    public void AttachTriangulation(Twin<N.TopoDS_Face, Face> theFace, double theDeflection)
    {
        var aNT = new N.Poly_Triangulation(3, 1, false);
        var aT = new Triangulation(3, 1, false);
        for (int i = 1; i <= 3; i++)
        {
            var aP = new Point3D(i == 2 ? 1 : 0, i == 3 ? 1 : 0, 0);
            using var aNP = Oracle.ToNative(aP);
            aNT.SetNode(i, aNP);
            aT.SetNode(i, aP);
        }

        using (var aNTri = new N.Poly_Triangle(1, 2, 3))
        {
            aNT.SetTriangle(1, aNTri);
        }

        aT.SetTriangle(1, new Triangle1(1, 2, 3));
        aNT.Deflection(theDeflection);
        aT.Deflection(theDeflection);
        myN.UpdateFace(theFace.N, aNT);
        myP.UpdateFace(theFace.P, aT);
    }

    /// <summary>Adds the child to the shape on both sides.</summary>
    public void Add(Twin<N.TopoDS_Shape, Shape> theShape, Twin<N.TopoDS_Shape, Shape> theChild)
    {
        myN.Add(theShape.N, theChild.N);
        myP.Add(theShape.P, theChild.P);
    }

    public static Twin<N.TopoDS_Shape, Shape> Up<TN, TP>(Twin<TN, TP> theTwin)
        where TN : N.TopoDS_Shape
        where TP : struct
        => new(theTwin.N, ToShape(theTwin.P));

    private static Shape ToShape<TP>(TP theP) => theP switch
    {
        Shape s => s,
        Vertex s => s,
        Edge s => s,
        Wire s => s,
        Face s => s,
        Shell s => s,
        Solid s => s,
        CompSolid s => s,
        Compound s => s,
        _ => throw new ArgumentException("not a shape"),
    };
}
