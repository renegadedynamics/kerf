// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Shapes for the classification and box tests (unit 3G), built with BRep_Builder until the makers are ported: a
/// square with a square hole on the XY plane (UV = XY), a natural face of a trimmed surface of extrusion, a solid with
/// an inner void, and a located copy of a shape.
/// </summary>
internal static class ClassificationTestShapes
{
    private const double Tol = 1.0e-7;

    /// <summary>
    /// The square [0, 10] x [0, 10] of the plane z = 0 (normal +Z, UV = XY) with the square hole [4, 6] x [4, 6]:
    /// the outer wire counter-clockwise, the hole's clockwise. Unfrozen.
    /// </summary>
    public static Face HoledSquare()
    {
        var aB = new BrepBuilder();
        var aPlane = new PlaneSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        Face aF = new();
        aB.MakeFace(ref aF, aPlane, Tol);
        aB.Add(aF, PolygonWire(aF, (0, 0), (10, 0), (10, 10), (0, 10)));
        aB.Add(aF, PolygonWire(aF, (4, 4), (4, 6), (6, 6), (6, 4)));
        return aF;
    }

    /// <summary>A closed wire of segments through the (x, y) corners of the plane z = 0, with pcurves on the face.</summary>
    private static Wire PolygonWire(in Face theFace, params (double X, double Y)[] theCorners)
    {
        var aB = new BrepBuilder();
        int n = theCorners.Length;
        var aV = new Vertex[n];
        for (int i = 0; i < n; i++)
        {
            aV[i] = TestShapes.Vertex(new Point3D(theCorners[i].X, theCorners[i].Y, 0));
        }

        Wire aW = new();
        aB.MakeWire(ref aW);
        for (int i = 0; i < n; i++)
        {
            (double x1, double y1) = theCorners[i];
            (double x2, double y2) = theCorners[(i + 1) % n];
            Edge anE = TestShapes.Edge(aV[i], aV[(i + 1) % n]);
            aB.UpdateEdge(anE, new LineCurve2D(new Point2D(x1, y1), new Direction2D(x2 - x1, y2 - y1)), theFace, Tol);
            aB.Add(aW, anE);
        }

        aW.Closed(true);
        return aW;
    }

    /// <summary>
    /// The natural face of the surface of linear extrusion along +Z of the circle of radius <paramref name="r"/>
    /// around Z, trimmed to [0, 2 Pi] x [0, <paramref name="h"/>]: a cylinder that IntCurvesFace meets through a
    /// polyhedron.
    /// </summary>
    public static Face ExtrudedTube(double r, double h)
    {
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), r);
        var anExtr = new ExtrusionSurface(aCircle, new Direction3D(0, 0, 1));
        var aTrimmed = new TrimmedSurface(anExtr, 0.0, 2.0 * Math.PI, 0.0, h);
        return BRepGPropTestShapes.NaturalFace(aTrimmed);
    }

    /// <summary>
    /// A box [0, 10]^3 with a cubic void [3, 5]^3: the outer box's shell and the inner box's shell reversed, in one
    /// solid. Unfrozen.
    /// </summary>
    public static Solid BoxWithVoid(out Shell theOuter, out Shell theInner)
    {
        var aB = new BrepBuilder();
        Solid anOuterBox = TestShapes.Box(10, 10, 10);
        Solid anInnerBox = TestShapes.Box(2, 2, 2);
        theOuter = FirstShell(anOuterBox);
        var aT = new Transform3D();
        aT = aT.WithTranslation(new Vector3D(3, 3, 3));
        theInner = ShapeCasts.Shell(FirstShell(anInnerBox).Moved(new Placement(aT)).Reversed());
        Solid aSolid = new();
        aB.MakeSolid(ref aSolid);
        aB.Add(aSolid, theInner);
        aB.Add(aSolid, theOuter);
        return aSolid;
    }

    private static Shell FirstShell(in Shape theSolid)
    {
        var anIt = new Iterator(theSolid);
        return ShapeCasts.Shell(anIt.Value());
    }

    /// <summary>The shape moved by the translation <paramref name="theV"/>.</summary>
    public static Shape Translated(in Shape theShape, in Vector3D theV)
    {
        var aT = new Transform3D();
        aT = aT.WithTranslation(theV);
        return theShape.Moved(new Placement(aT));
    }

    /// <summary>The shape rotated by <paramref name="theAngle"/> around the Z axis.</summary>
    public static Shape RotatedZ(in Shape theShape, double theAngle)
    {
        var aT = new Transform3D();
        aT = aT.WithRotation(new Axis3D(new Point3D(), new Direction3D(0, 0, 1)), theAngle);
        return theShape.Moved(new Placement(aT));
    }
}
