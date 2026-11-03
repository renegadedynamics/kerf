// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_AnaFilletAlgo.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_AnaFilletAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>An analytical algorithm for 2D fillets, for segments and arcs of circle only.</summary>
/// <remarks>
/// A working object; its results are new, frozen edges. The edges it is given are only read. As in OCCT, initialising
/// it again by a wire keeps the edges of the first initialisation (occt-issues.md, unit 5R).
/// </remarks>
public sealed class ChFi2d_AnaFilletAlgo
{
    // Plane.
    private Plane3D plane = new();

    // Left neighbour.
    private Edge e1 = new();
    private bool segment1;
    private double x11;
    private double y11;
    private double x12;
    private double y12;
    private double xc1;
    private double yc1;
    private double radius1;
    private bool cw1;

    // Right neighbour.
    private Edge e2 = new();
    private bool segment2;
    private double x21;
    private double y21;
    private double x22;
    private double y22;
    private double xc2;
    private double yc2;
    private double radius2;
    private bool cw2;

    // Fillet (result).
    private Edge fillet = new();
    private Edge shrinke1 = new();
    private Edge shrinke2 = new();

    // Compute the flag: CW || CCW
    private static bool isCW(EdgeCurveAdaptor AC)
    {
        double f = AC.FirstParameter();
        double l = AC.LastParameter();
        // OCCT: the middle point is taken on the edge's curve without the edge's location, the ends with it
        CircleCurve3D circle = (AC.Curve().Curve() as CircleCurve3D)!; // OCCT dereferences down_cast's result unchecked
        Point3D start = AC.Value(f);
        Point3D end = AC.Value(l);
        Point3D center = AC.Circle().Location();
        CoordinateSystem plane = new(AC.Circle().Position());

        // Get point on circle at half angle
        circle.D0(0.5 * (f + l), out Point3D m);

        // Compare angles between vectors to middle point and to the end point.
        Vector3D startv = new(center, start), endv = new(center, end), middlev = new(center, m);
        double middlea = startv.AngleWithRef(middlev, new Vector3D(plane.Direction()));
        while (middlea < 0.0)
        {
            middlea += 2.0 * M_PI;
        }
        double enda = startv.AngleWithRef(endv, new Vector3D(plane.Direction()));
        while (enda < 0.0)
        {
            enda += 2.0 * M_PI;
        }

        bool is_cw = middlea > enda;
        return is_cw;
    }

    // Equality of points computed through square distance between the points.
    private static bool IsEqual(in Point3D p1, in Point3D p2)
    {
        return p1.SquareDistance(p2) < global::Kerf.Engine.Foundation.Precision.SquareConfusion();
    }

    private static bool IsEqual(in Point2D p1, in Point2D p2)
    {
        return p1.SquareDistance(p2) < global::Kerf.Engine.Foundation.Precision.SquareConfusion();
    }

    // An empty constructor.
    // Use the method Init() to initialize the class.
    public ChFi2d_AnaFilletAlgo()
    {
        segment1 = false;
        x11 = 0.0;
        y11 = 0.0;
        x12 = 0.0;
        y12 = 0.0;
        xc1 = 0.0;
        yc1 = 0.0;
        radius1 = 0.0;
        cw1 = false;
        segment2 = false;
        x21 = 0.0;
        y21 = 0.0;
        x22 = 0.0;
        y22 = 0.0;
        xc2 = 0.0;
        yc2 = 0.0;
        radius2 = 0.0;
        cw2 = false;
    }

    /// <summary>
    /// An algorithm on a wire of two edges, each a segment or an arc of circle.
    /// </summary>
    public ChFi2d_AnaFilletAlgo(in Wire theWire, in Plane3D thePlane)
        : this()
    {
        plane = thePlane;
        Init(theWire, thePlane);
    }

    /// <summary>
    /// An algorithm on two edges with a common point, each a segment or an arc of circle.
    /// </summary>
    public ChFi2d_AnaFilletAlgo(in Edge theEdge1, in Edge theEdge2, in Plane3D thePlane)
        : this()
    {
        plane = thePlane;
        // Make a wire consisting of two edges.
        Init(theEdge1, theEdge2, thePlane);
    }

    /// <summary>Initializes the class by a wire consisting of two edges.</summary>
    /// <exception cref="TypeMismatchException">The wire hasn't two edges, or one isn't a segment or an arc.</exception>
    /// <exception cref="FailureException">An edge is infinite.</exception>
    public void Init(in Wire theWire, in Plane3D thePlane)
    {
        plane = thePlane;
        // OCCT: e1 and e2 aren't reset, so a second Init keeps the edges of the first
        var itr = new Iterator(theWire);
        for (; itr.More(); itr.Next())
        {
            if (e1.IsNull)
            {
                e1 = ShapeCasts.Edge(itr.Value());
            }
            else if (e2.IsNull)
            {
                e2 = ShapeCasts.Edge(itr.Value());
            }
        }
        if (e1.IsNull || e2.IsNull)
        {
            throw new TypeMismatchException("The algorithm expects a wire consisting of two linear or circular edges.");
        }

        // Left neighbour.
        var AC1 = new EdgeCurveAdaptor(e1);
        if (AC1.GetType() != Line && AC1.GetType() != Circle)
        {
            throw new TypeMismatchException("A segment or an arc of circle is expected.");
        }

        Exploration.Vertices(e1, out Vertex v1, out Vertex v2, true);
        if (v1.IsNull || v2.IsNull)
        {
            throw new FailureException("An infinite edge.");
        }

        Point3D P1 = Tool.Pnt(v1);
        Point3D P2 = Tool.Pnt(v2);
        Point2D p1 = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(thePlane, P1);
        Point2D p2 = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(thePlane, P2);
        p1.Coord(out x11, out y11);
        p2.Coord(out x12, out y12);

        segment1 = true;
        if (AC1.GetType() == Circle)
        {
            segment1 = false;
            Circle3D c = AC1.Circle();

            Point2D loc = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(thePlane, c.Location());
            loc.Coord(out xc1, out yc1);

            radius1 = c.Radius();
            cw1 = isCW(AC1);
        }

        // Right neighbour.
        var AC2 = new EdgeCurveAdaptor(e2);
        if (AC2.GetType() != Line && AC2.GetType() != Circle)
        {
            throw new TypeMismatchException("A segment or an arc of circle is expected.");
        }

        Exploration.Vertices(e2, out v1, out v2, true);
        if (v1.IsNull || v2.IsNull)
        {
            throw new FailureException("An infinite edge.");
        }

        P1 = Tool.Pnt(v1);
        P2 = Tool.Pnt(v2);
        p1 = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(thePlane, P1);
        p2 = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(thePlane, P2);
        p1.Coord(out x21, out y21);
        p2.Coord(out x22, out y22);

        segment2 = true;
        if (AC2.GetType() == Circle)
        {
            segment2 = false;
            Circle3D c = AC2.Circle();

            Point2D loc = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(thePlane, c.Location());
            loc.Coord(out xc2, out yc2);

            radius2 = c.Radius();
            cw2 = isCW(AC2);
        }
    }

    /// <summary>Initializes the class by two edges with a common point.</summary>
    /// <exception cref="FailureException">An edge is infinite, the edges have no common point, or no wire is made.</exception>
    public void Init(in Edge theEdge1, in Edge theEdge2, in Plane3D thePlane)
    {
        // Make a wire consisting of two edges.

        // Get common point.
        Exploration.Vertices(theEdge1, out Vertex v11, out Vertex v12, true);
        Exploration.Vertices(theEdge2, out Vertex v21, out Vertex v22, true);
        if (v11.IsNull || v12.IsNull || v21.IsNull || v22.IsNull)
        {
            throw new FailureException("An infinite edge.");
        }

        Point3D p11 = Tool.Pnt(v11);
        Point3D p12 = Tool.Pnt(v12);
        Point3D p21 = Tool.Pnt(v21);
        Point3D p22 = Tool.Pnt(v22);

        Point3D pcommon;
        if (IsEqual(p11, p21) || IsEqual(p11, p22))
        {
            pcommon = p11;
        }
        else if (IsEqual(p12, p21) || IsEqual(p12, p22))
        {
            pcommon = p12;
        }
        else
        {
            throw new FailureException("The edges have no common point.");
        }

        // Reverse the edges in case of need (to construct a wire).
        bool is1stReversed = false, is2ndReversed = false;
        if (IsEqual(pcommon, p11))
        {
            is1stReversed = true;
        }
        else if (IsEqual(pcommon, p22))
        {
            is2ndReversed = true;
        }

        // Make a wire.
        var mkWire = new BRepBuilderAPI_MakeWire();
        if (is1stReversed)
        {
            mkWire.Add(ShapeCasts.Edge(theEdge1.Reversed()));
        }
        else
        {
            mkWire.Add(theEdge1);
        }
        if (is2ndReversed)
        {
            mkWire.Add(ShapeCasts.Edge(theEdge2.Reversed()));
        }
        else
        {
            mkWire.Add(theEdge2);
        }
        if (!mkWire.IsDone())
        {
            throw new FailureException("Can't make a wire.");
        }

        Wire W = mkWire.InternalWire();
        Init(W, thePlane);
    }

    /// <summary>Calculates a fillet of the radius. True if it was built.</summary>
    public bool Perform(double radius)
    {
        bool bRet = false;
        if (e1.IsNull || e2.IsNull || radius < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            return bRet;
        }

        // Fillet definition.
        double xc = 0.0, yc = 0.0;
        double start = 0.0, end = 0.0;                     // parameters on neighbours
        double xstart = double.MaxValue, ystart = double.MaxValue; // point on left neighbour
        double xend = double.MaxValue, yend = double.MaxValue;     // point on right neighbour
        bool cw = false;

        // Analytical algorithm works for non-intersecting arcs only.
        // Check arcs on self-intersection.
        bool isCut = false;
        if (!segment1 || !segment2)
        {
            var mkWire = new BRepBuilderAPI_MakeWire(e1, e2);
            if (mkWire.IsDone())
            {
                Wire W = mkWire.InternalWire();
                var mkFace = new BRepBuilderAPI_MakeFace(plane);
                if (mkFace.IsDone())
                {
                    Face F = mkFace.InternalFace();
                    var analyzer = new ShapeAnalysis_Wire(W, F, global::Kerf.Engine.Foundation.Precision.Confusion());
                    if (analyzer.CheckSelfIntersection())
                    {
                        // Cut the edges at the point of intersection.
                        isCut = true;
                        if (!Cut(plane, ref e1, ref e2))
                        {
                            return false;
                        }
                    }
                }
            }
        } // a case of segment - segment

        // Choose the case.
        var AC1 = new EdgeCurveAdaptor(e1);
        var AC2 = new EdgeCurveAdaptor(e2);
        if (segment1 && segment2)
        {
            bRet = SegmentFilletSegment(radius, ref xc, ref yc, ref cw, ref start, ref end);
        }
        else if (segment1 && !segment2)
        {
            bRet = SegmentFilletArc(radius, ref xc, ref yc, ref cw, ref start, ref end, ref xend, ref yend);
        }
        else if (!segment1 && segment2)
        {
            bRet = ArcFilletSegment(radius, ref xc, ref yc, ref cw, ref start, ref end, ref xstart, ref ystart);
        }
        else if (!segment1 && !segment2)
        {
            bRet = ArcFilletArc(radius, ref xc, ref yc, ref cw, ref start, ref end);
        }

        if (!bRet)
        {
            return false;
        }

        // Invert the fillet for left-handed plane.
        if (!plane.Position().Direct())
        {
            cw = !cw;
        }

        // Construct a fillet.
        // Make circle.
        Point3D center = global::Kerf.Engine.Geometry.ElSLib.Value(xc, yc, plane);
        Direction3D normal = plane.Position().Direction();
        var circ = new Circle3D(new Frame3D(center, cw ? -normal : normal), radius);

        // Fillet may only shrink a neighbour edge, it can't prolongate it.
        double delta1 = AC1.LastParameter() - AC1.FirstParameter();
        double delta2 = AC2.LastParameter() - AC2.FirstParameter();
        if (!isCut && (start > delta1 || end > delta2))
        {
            // Check a case when a neighbour edge almost disappears:
            // try to reduce the fillet radius for a little (1.e-5 mm).
            double little = 100.0 * global::Kerf.Engine.Foundation.Precision.Confusion();
            double d1 = Math.Abs(start - delta1);
            double d2 = Math.Abs(end - delta2);
            if (d1 < little || d2 < little)
            {
                if (segment1 && segment2)
                {
                    bRet = SegmentFilletSegment(radius - little, ref xc, ref yc, ref cw, ref start, ref end);
                }
                else if (segment1 && !segment2)
                {
                    bRet = SegmentFilletArc(radius - little, ref xc, ref yc, ref cw, ref start, ref end, ref xend, ref yend);
                }
                else if (!segment1 && segment2)
                {
                    bRet = ArcFilletSegment(radius - little, ref xc, ref yc, ref cw, ref start, ref end, ref xstart, ref ystart);
                }
                else if (!segment1 && !segment2)
                {
                    bRet = ArcFilletArc(radius - little, ref xc, ref yc, ref cw, ref start, ref end);
                }
                if (bRet)
                {
                    // Invert the fillet for left-handed planes.
                    if (!plane.Position().Direct())
                    {
                        cw = !cw;
                    }

                    // Make the circle again.
                    center = global::Kerf.Engine.Geometry.ElSLib.Value(xc, yc, plane);
                    circ.SetLocation(center);
                    circ.SetRadius(radius - little);
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }
        if (bRet)
        {
            // start: (xstart, ystart) - pstart.
            Point3D pstart;
            if (xstart != double.MaxValue)
            {
                pstart = global::Kerf.Engine.Geometry.ElSLib.Value(xstart, ystart, plane);
            }
            else
            {
                if (e1.Orientation() == Forward)
                {
                    pstart = AC1.Value(AC1.LastParameter() - start);
                }
                else
                {
                    pstart = AC1.Value(AC1.FirstParameter() + start);
                }
            }
            // end: (xend, yend) -> pend.
            Point3D pend;
            if (xend != double.MaxValue)
            {
                pend = global::Kerf.Engine.Geometry.ElSLib.Value(xend, yend, plane);
            }
            else
            {
                if (e2.Orientation() == Forward)
                {
                    pend = AC2.Value(AC2.FirstParameter() + end);
                }
                else
                {
                    pend = AC2.Value(AC2.LastParameter() - end);
                }
            }

            // Make arc.
            var mkEdge = new BRepBuilderAPI_MakeEdge(circ, pstart, pend);
            bRet = mkEdge.IsDone();
            if (bRet)
            {
                fillet = mkEdge.Edge();

                // Limit the neighbours.
                // Left neighbour.
                Point3D p1, p2;
                shrinke1.Nullify();
                if (e1.Orientation() == Forward)
                {
                    p1 = AC1.Value(AC1.FirstParameter());
                    p2 = pstart;
                }
                else
                {
                    p1 = pstart;
                    p2 = AC1.Value(AC1.LastParameter());
                }
                if (segment1)
                {
                    var mkSegment1 = new BRepBuilderAPI_MakeEdge();
                    mkSegment1.Init(AC1.Curve().Curve()!, p1, p2);
                    if (mkSegment1.IsDone())
                    {
                        shrinke1 = mkSegment1.Edge();
                    }
                }
                else
                {
                    var mkCirc1 = new BRepBuilderAPI_MakeEdge();
                    mkCirc1.Init(AC1.Curve().Curve()!, p1, p2);
                    if (mkCirc1.IsDone())
                    {
                        shrinke1 = mkCirc1.Edge();
                    }
                }

                // Right neighbour.
                shrinke2.Nullify();
                if (e1.Orientation() == Forward) // OCCT: e1's orientation decides e2's ends
                {
                    p1 = pend;
                    p2 = AC2.Value(AC2.LastParameter());
                }
                else
                {
                    p1 = AC2.Value(AC2.FirstParameter());
                    p2 = pend;
                }
                if (segment2)
                {
                    var mkSegment2 = new BRepBuilderAPI_MakeEdge();
                    mkSegment2.Init(AC2.Curve().Curve()!, p1, p2);
                    if (mkSegment2.IsDone())
                    {
                        shrinke2 = mkSegment2.Edge();
                    }
                }
                else
                {
                    var mkCirc2 = new BRepBuilderAPI_MakeEdge();
                    mkCirc2.Init(AC2.Curve().Curve()!, p1, p2);
                    if (mkCirc2.IsDone())
                    {
                        shrinke2 = mkCirc2.Edge();
                    }
                }

                bRet = !shrinke1.IsNull && !shrinke2.IsNull;
            } // fillet edge is done
        } // shrinking is good

        return bRet;
    }

    /// <summary>The fillet, and the two edges it shrinks in <paramref name="theE1"/> and <paramref name="theE2"/> (frozen).</summary>
    public Edge Result(ref Edge theE1, ref Edge theE2)
    {
        theE1 = shrinke1;
        theE2 = shrinke2;
        return fillet;
    }

    // WW5 method to compute fillet.
    // It returns a constructed fillet definition:
    //     center point (xc, yc)
    //     point on the 1st segment (xstart, ystart)
    //     point on the 2nd segment (xend, yend)
    //     is the arc of fillet clockwise (cw = true) or counterclockwise (cw = false).
    private bool SegmentFilletSegment(double radius, ref double xc, ref double yc, ref bool cw, ref double start, ref double end)
    {
        // Make normalized vectors at p12.
        var p11 = new Point2D(x11, y11);
        var p12 = new Point2D(x12, y12);
        var p22 = new Point2D(x22, y22);

        // Check length of segments.
        if (IsEqual(p12, p11) || IsEqual(p12, p22))
        {
            return false;
        }

        // Make vectors.
        var v1 = new Vector2D(p12, p11);
        var v2 = new Vector2D(p12, p22);
        v1.Normalize();
        v2.Normalize();

        // Make bisectrissa.
        Vector2D bisec = 0.5 * (v1 + v2);

        // Check bisectrissa.
        if (bisec.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return false;
        }

        // Normalize the bisectrissa.
        bisec.Normalize();

        // Angle at bisectrissa.
        double beta = v1.Angle(bisec);

        // Length along the bisectrissa till the center of fillet.
        double L = radius / Math.Sin(Math.Abs(beta));

        // Center point of fillet.
        Point2D pc = p12.Translated(L * bisec);
        pc.Coord(out xc, out yc);

        // Shrinking length along segments.
        start = Math.Sqrt(L * L - radius * radius);
        end = start;

        // Orientation of fillet.
        cw = beta > 0.0;
        return true;
    }

    // A function constructs a fillet between a segment and an arc.
    private bool SegmentFilletArc(double radius,
                                  ref double xc,
                                  ref double yc,
                                  ref bool cw,
                                  ref double start,
                                  ref double end,
                                  ref double xend,
                                  ref double yend)
    {
        // Make a line parallel to the segment at the side of center point of fillet.
        // This side may be defined through making a bisectrissa for vectors at p12 (or p21).

        // Make 2D points.
        var p12 = new Point2D(x12, y12);
        var p11 = new Point2D(x11, y11);
        var pc2 = new Point2D(xc2, yc2);

        // Check length of segment.
        if (p11.SquareDistance(p12) < Constants.Resolution())
        {
            return false;
        }

        // Make 2D vectors.
        var v1 = new Vector2D(p12, p11);
        var v2 = new Vector2D(p12, pc2);

        // Rotate the arc vector to become tangential at p21.
        if (cw2)
        {
            v2.Rotate(+M_PI_2);
        }
        else
        {
            v2.Rotate(-M_PI_2);
        }

        // If vectors coincide (segment and arc are tangent),
        // the algorithm doesn't work...
        double angle = v1.Angle(v2);
        if (Math.Abs(angle) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return false;
        }

        // Make a bissectrisa of vectors at p12.
        v2.Normalize();
        v1.Normalize();
        Vector2D bisec = 0.5 * (v1 + v2);

        // If segment and arc look in opposite direction,
        // no fillet is possible.
        if (bisec.SquareMagnitude() < Constants.Resolution())
        {
            return false;
        }

        // Define an appropriate point to choose center of fillet.
        bisec.Normalize();
        Point2D nearp = p12.Translated(radius * bisec);
        var nearl = new Line2D(p12, new Direction2D(bisec));

        // Make a line parallel to segment and
        // passing near the "near" point.
        Vector2D d1 = v1;
        var line = new Line2D(p11, new Direction2D(-d1));
        d1.Rotate(M_PI_2);
        line.Translate(radius * d1);
        if (line.Distance(nearp) > radius)
        {
            line.Translate(-2.0 * radius * d1);
        }

        // Make a circle of radius of the arc +/- fillet radius.
        var axes = new Axis2D(pc2, Constants.DX2d());
        var circ = new Circle2D(axes, radius2 + radius);
        if (radius2 > radius && circ.Distance(nearp) > radius)
        {
            circ.SetRadius(radius2 - radius);
        }

        // Calculate intersection of the line and the circle.
        var intersector = new AnalyticIntersection(line, circ);
        if (!intersector.IsDone || intersector.PointCount == 0)
        {
            return false;
        }

        // Find center point of fillet.
        int i;
        double minDist = double.MaxValue;
        for (i = 1; i <= intersector.PointCount; ++i)
        {
            IntPoint intp = intersector.Point(i);
            Point2D p = intp.Value();

            double d = nearl.Distance(p);
            if (d < minDist)
            {
                minDist = d;
                p.Coord(out xc, out yc);
            }
        }

        // Shrink of segment.
        var pc = new Point2D(xc, yc);
        double L2 = pc.SquareDistance(p12);
        double Rf2 = radius * radius;
        start = Math.Sqrt(L2 - Rf2);

        // Shrink of arc.
        var pcc = new Vector2D(pc2, pc);
        end = Math.Abs(new Vector2D(pc2, p12).Angle(pcc));

        // Duplicate the information on shrink the arc:
        // calculate a point on the arc coinciding with the end of fillet.
        line.SetLocation(pc2);
        line.SetDirection(new Direction2D(pcc));
        circ.SetLocation(pc2);
        circ.SetRadius(radius2);
        intersector.Perform(line, circ);
        if (!intersector.IsDone || intersector.PointCount == 0)
        {
            return false;
        }

        xend = double.MaxValue;
        yend = double.MaxValue;
        for (i = 1; i <= intersector.PointCount; ++i)
        {
            IntPoint intp = intersector.Point(i);
            Point2D p = intp.Value();

            double d2 = p.SquareDistance(pc);
            if (Math.Abs(d2 - Rf2) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                p.Coord(out xend, out yend);
                break;
            }
        }

        // Orientation of the fillet.
        angle = v1.Angle(v2);
        cw = angle > 0.0;
        return true;
    }

    // A function constructs a fillet between an arc and a segment.
    private bool ArcFilletSegment(double radius,
                                  ref double xc,
                                  ref double yc,
                                  ref bool cw,
                                  ref double start,
                                  ref double end,
                                  ref double xstart,
                                  ref double ystart)
    {
        // Make a line parallel to the segment at the side of center point of fillet.
        // This side may be defined through making a bisectrissa for vectors at p12 (or p21).

        // Make 2D points.
        var p12 = new Point2D(x12, y12);
        var p22 = new Point2D(x22, y22);
        var pc1 = new Point2D(xc1, yc1);

        // Check length of segment.
        if (p12.SquareDistance(p22) < Constants.Resolution())
        {
            return false;
        }

        // Make 2D vectors.
        var v1 = new Vector2D(p12, pc1);
        var v2 = new Vector2D(p12, p22);

        // Rotate the arc vector to become tangential at p21.
        if (cw1)
        {
            v1.Rotate(-M_PI_2);
        }
        else
        {
            v1.Rotate(+M_PI_2);
        }

        // If vectors coincide (segment and arc are tangent),
        // the algorithm doesn't work...
        double angle = v1.Angle(v2);
        if (Math.Abs(angle) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return false;
        }

        // Make a bisectrissa of vectors at p12.
        v1.Normalize();
        v2.Normalize();
        Vector2D bisec = 0.5 * (v1 + v2);

        // If segment and arc look in opposite direction,
        // no fillet is possible.
        if (bisec.SquareMagnitude() < Constants.Resolution())
        {
            return false;
        }

        // Define an appropriate point to choose center of fillet.
        bisec.Normalize();
        Point2D nearPoint = p12.Translated(radius * bisec);
        var nearLine = new Line2D(p12, new Direction2D(bisec));

        // Make a line parallel to segment and
        // passing near the "near" point.
        Vector2D aD2Vec = v2;
        var line = new Line2D(p22, new Direction2D(-aD2Vec));
        aD2Vec.Rotate(M_PI_2);
        line.Translate(radius * aD2Vec);
        if (line.Distance(nearPoint) > radius)
        {
            line.Translate(-2.0 * radius * aD2Vec);
        }

        // Make a circle of radius of the arc +/- fillet radius.
        var axes = new Axis2D(pc1, Constants.DX2d());
        var circ = new Circle2D(axes, radius1 + radius);
        if (radius1 > radius && circ.Distance(nearPoint) > radius)
        {
            circ.SetRadius(radius1 - radius);
        }

        // Calculate intersection of the line and the big circle.
        var intersector = new AnalyticIntersection(line, circ);
        if (!intersector.IsDone || intersector.PointCount == 0)
        {
            return false;
        }

        // Find center point of fillet.
        int i;
        double minDist = double.MaxValue;
        for (i = 1; i <= intersector.PointCount; ++i)
        {
            IntPoint intp = intersector.Point(i);
            Point2D p = intp.Value();

            double d = nearLine.Distance(p);
            if (d < minDist)
            {
                minDist = d;
                p.Coord(out xc, out yc);
            }
        }

        // Shrink of segment.
        var pc = new Point2D(xc, yc);
        double L2 = pc.SquareDistance(p12);
        double Rf2 = radius * radius;
        end = Math.Sqrt(L2 - Rf2);

        // Shrink of arc.
        var pcc = new Vector2D(pc1, pc);
        start = Math.Abs(new Vector2D(pc1, p12).Angle(pcc));

        // Duplicate the information on shrink the arc:
        // calculate a point on the arc coinciding with the start of fillet.
        line.SetLocation(pc1);
        line.SetDirection(new Direction2D(pcc));
        circ.SetLocation(pc1);
        circ.SetRadius(radius1);
        intersector.Perform(line, circ);
        if (!intersector.IsDone || intersector.PointCount == 0)
        {
            return false;
        }

        xstart = double.MaxValue;
        ystart = double.MaxValue;
        for (i = 1; i <= intersector.PointCount; ++i)
        {
            IntPoint intp = intersector.Point(i);
            Point2D p = intp.Value();

            double d2 = p.SquareDistance(pc);
            if (Math.Abs(d2 - Rf2) < global::Kerf.Engine.Foundation.Precision.SquareConfusion()) // OCCT: SegmentFilletArc compares with Confusion
            {
                p.Coord(out xstart, out ystart);
                break;
            }
        }

        // Orientation of the fillet.
        angle = v2.Angle(v1);
        cw = angle < 0.0;
        return true;
    }

    // WW5 method to compute fillet: arc - arc.
    // It returns a constructed fillet definition:
    //     center point (xc, yc)
    //     shrinking parameter of the 1st circle (start)
    //     shrinking parameter of the 2nd circle (end)
    //     if the arc of fillet clockwise (cw = true) or counterclockwise (cw = false).
    private bool ArcFilletArc(double radius, ref double xc, ref double yc, ref bool cw, ref double start, ref double end)
    {
        // Make points.
        var pc1 = new Point2D(xc1, yc1);
        var pc2 = new Point2D(xc2, yc2);
        var p12 = new Point2D(x12, y12);

        // Make vectors at p12.
        var v1 = new Vector2D(pc1, p12);
        var v2 = new Vector2D(pc2, p12);

        // Rotate the vectors so that they are tangent to circles at p12.
        if (cw1)
        {
            v1.Rotate(+M_PI_2);
        }
        else
        {
            v1.Rotate(-M_PI_2);
        }
        if (cw2)
        {
            v2.Rotate(-M_PI_2);
        }
        else
        {
            v2.Rotate(+M_PI_2);
        }

        // Make a "check" point for choosing an offset circle.
        v1.Normalize();
        v2.Normalize();
        Vector2D bisec = 0.5 * (v1 + v2);
        if (bisec.SquareMagnitude() < Constants.Resolution())
        {
            return false;
        }

        Point2D checkp = p12.Translated(radius * bisec);
        var checkl = new Line2D(p12, new Direction2D(bisec));
        _ = checkl; // unused, as in OCCT

        // Make two circles of radius r1 +/- r and r2 +/- r
        // with center point equal to pc1 and pc2.
        // Arc 1.
        var axes = new Axis2D(pc1, Constants.DX2d());
        var c1 = new Circle2D(axes, radius1 + radius);
        if (radius1 > radius && c1.Distance(checkp) > radius)
        {
            c1.SetRadius(radius1 - radius);
        }
        // Arc 2.
        axes.SetLocation(pc2);
        var c2 = new Circle2D(axes, radius2 + radius);
        if (radius2 > radius && c2.Distance(checkp) > radius)
        {
            c2.SetRadius(radius2 - radius);
        }

        // Calculate an intersection point of these two circles
        // and choose the one closer to the "check" point.
        var intersector = new AnalyticIntersection(c1, c2);
        if (!intersector.IsDone || intersector.PointCount == 0)
        {
            return false;
        }

        // Find center point of fillet.
        Point2D pc = new();
        double minDist = double.MaxValue;
        for (int i = 1; i <= intersector.PointCount; ++i)
        {
            IntPoint intp = intersector.Point(i);
            Point2D p = intp.Value();

            double d = checkp.SquareDistance(p);
            if (d < minDist)
            {
                minDist = d;
                pc = p;
            }
        }
        pc.Coord(out xc, out yc);

        // Orientation of fillet.
        double angle = v1.Angle(v2);
        if (Math.Abs(angle) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            angle = new Vector2D(pc, pc1).Angle(new Vector2D(pc, pc2));
            cw = angle < 0.0;
        }
        else
        {
            cw = angle > 0.0;
        }

        // Shrinking of circles.
        start = Math.Abs(new Vector2D(pc1, p12).Angle(new Vector2D(pc1, pc)));
        end = Math.Abs(new Vector2D(pc2, p12).Angle(new Vector2D(pc2, pc)));
        return true;
    }

    // Cuts intersecting edges of a contour.
    private bool Cut(in Plane3D thePlane, ref Edge theE1, ref Edge theE2)
    {
        Point3D p = new();
        bool found = false;
        double param1 = 0.0, param2 = 0.0;
        double f1, l1, f2, l2;
        Curve3D c1 = Tool.Curve(theE1, out f1, out l1)!;
        Curve3D c2 = Tool.Curve(theE2, out f2, out l2)!;
        var extrema = new GeomAPI_ExtremaCurveCurve(c1, c2, f1, l1, f2, l2);
        if (extrema.NbExtrema() != 0)
        {
            int i, nb = extrema.NbExtrema();
            for (i = 1; i <= nb; ++i)
            {
                double d = extrema.Distance(i);
                if (d < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    extrema.Parameters(i, out param1, out param2);
                    if (Math.Abs(l1 - param1) > global::Kerf.Engine.Foundation.Precision.Confusion()
                        && Math.Abs(f2 - param2) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        found = true;
                        extrema.Points(i, out _, out p); // OCCT passes p for both points: the second is kept
                        break;
                    }
                }
            }
        }

        if (found)
        {
            var mkEdge1 = new BRepBuilderAPI_MakeEdge(c1, f1, param1);
            if (mkEdge1.IsDone())
            {
                theE1 = mkEdge1.Edge();

                var mkEdge2 = new BRepBuilderAPI_MakeEdge(c2, param2, l2);
                if (mkEdge2.IsDone())
                {
                    theE2 = mkEdge2.Edge();

                    Point2D p2d = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(thePlane, p);
                    p2d.Coord(out x12, out y12);
                    x21 = x12;
                    y21 = y12;
                    return true;
                }
            }
        }
        return false;
    }
}
