// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_AnaFilletAlgo.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_AnaFilletAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Planar;

/// <summary>An analytical algorithm for 2D fillets, for segments and arcs of circle only.</summary>
/// <remarks>
/// A working object; its results are new, frozen edges. The edges it is given are only read. As in OCCT, initialising
/// it again by a wire keeps the edges of the first initialisation (occt-issues.md, unit 5R).
/// </remarks>
public sealed class AnaFilletAlgo
{
    // Plane.
    private Plane3D _plane = new();

    // Left neighbour.
    private Edge _e1 = new();
    private bool _segment1;
    private double _x11;
    private double _y11;
    private double _x12;
    private double _y12;
    private double _xc1;
    private double _yc1;
    private double _radius1;
    private bool _cw1;

    // Right neighbour.
    private Edge _e2 = new();
    private bool _segment2;
    private double _x21;
    private double _y21;
    private double _x22;
    private double _y22;
    private double _xc2;
    private double _yc2;
    private double _radius2;
    private bool _cw2;

    // Fillet (result).
    private Edge _fillet = new();
    private Edge _shrinke1 = new();
    private Edge _shrinke2 = new();

    // Compute the flag: CW || CCW
    private static bool IsCW(EdgeCurveAdaptor ac)
    {
        double f = ac.FirstParameter();
        double l = ac.LastParameter();
        // OCCT: the middle point is taken on the edge's curve without the edge's location, the ends with it
        CircleCurve3D circle = (ac.Curve().Curve() as CircleCurve3D)!; // OCCT dereferences down_cast's result unchecked
        Point3D start = ac.Value(f);
        Point3D end = ac.Value(l);
        Point3D center = ac.Circle().Location();
        CoordinateSystem plane = new(ac.Circle().Position());

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
    public AnaFilletAlgo()
    {
        _segment1 = false;
        _x11 = 0.0;
        _y11 = 0.0;
        _x12 = 0.0;
        _y12 = 0.0;
        _xc1 = 0.0;
        _yc1 = 0.0;
        _radius1 = 0.0;
        _cw1 = false;
        _segment2 = false;
        _x21 = 0.0;
        _y21 = 0.0;
        _x22 = 0.0;
        _y22 = 0.0;
        _xc2 = 0.0;
        _yc2 = 0.0;
        _radius2 = 0.0;
        _cw2 = false;
    }

    /// <summary>
    /// An algorithm on a wire of two edges, each a segment or an arc of circle.
    /// </summary>
    public AnaFilletAlgo(in Wire wire, in Plane3D plane)
        : this()
    {
        _plane = plane;
        Init(wire, plane);
    }

    /// <summary>
    /// An algorithm on two edges with a common point, each a segment or an arc of circle.
    /// </summary>
    public AnaFilletAlgo(in Edge edge1, in Edge edge2, in Plane3D plane)
        : this()
    {
        _plane = plane;
        // Make a wire consisting of two edges.
        Init(edge1, edge2, plane);
    }

    /// <summary>Initializes the class by a wire consisting of two edges.</summary>
    /// <exception cref="TypeMismatchException">The wire hasn't two edges, or one isn't a segment or an arc.</exception>
    /// <exception cref="FailureException">An edge is infinite.</exception>
    public void Init(in Wire wire, in Plane3D plane)
    {
        _plane = plane;
        // OCCT: e1 and e2 aren't reset, so a second Init keeps the edges of the first
        var itr = new Iterator(wire);
        for (; itr.More(); itr.Next())
        {
            if (_e1.IsNull)
            {
                _e1 = ShapeCasts.Edge(itr.Value());
            }
            else if (_e2.IsNull)
            {
                _e2 = ShapeCasts.Edge(itr.Value());
            }
        }
        if (_e1.IsNull || _e2.IsNull)
        {
            throw new TypeMismatchException("The algorithm expects a wire consisting of two linear or circular edges.");
        }

        // Left neighbour.
        var ac1 = new EdgeCurveAdaptor(_e1);
        if (ac1.GetType() != Line && ac1.GetType() != Circle)
        {
            throw new TypeMismatchException("A segment or an arc of circle is expected.");
        }

        Exploration.Vertices(_e1, out Vertex v1, out Vertex v2, true);
        if (v1.IsNull || v2.IsNull)
        {
            throw new FailureException("An infinite edge.");
        }

        Point3D P1 = Tool.Pnt(v1);
        Point3D P2 = Tool.Pnt(v2);
        Point2D p1 = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(plane, P1);
        Point2D p2 = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(plane, P2);
        p1.Coord(out _x11, out _y11);
        p2.Coord(out _x12, out _y12);

        _segment1 = true;
        if (ac1.GetType() == Circle)
        {
            _segment1 = false;
            Circle3D c = ac1.Circle();

            Point2D loc = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(plane, c.Location());
            loc.Coord(out _xc1, out _yc1);

            _radius1 = c.Radius();
            _cw1 = IsCW(ac1);
        }

        // Right neighbour.
        var ac2 = new EdgeCurveAdaptor(_e2);
        if (ac2.GetType() != Line && ac2.GetType() != Circle)
        {
            throw new TypeMismatchException("A segment or an arc of circle is expected.");
        }

        Exploration.Vertices(_e2, out v1, out v2, true);
        if (v1.IsNull || v2.IsNull)
        {
            throw new FailureException("An infinite edge.");
        }

        P1 = Tool.Pnt(v1);
        P2 = Tool.Pnt(v2);
        p1 = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(plane, P1);
        p2 = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(plane, P2);
        p1.Coord(out _x21, out _y21);
        p2.Coord(out _x22, out _y22);

        _segment2 = true;
        if (ac2.GetType() == Circle)
        {
            _segment2 = false;
            Circle3D c = ac2.Circle();

            Point2D loc = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(plane, c.Location());
            loc.Coord(out _xc2, out _yc2);

            _radius2 = c.Radius();
            _cw2 = IsCW(ac2);
        }
    }

    /// <summary>Initializes the class by two edges with a common point.</summary>
    /// <exception cref="FailureException">An edge is infinite, the edges have no common point, or no wire is made.</exception>
    public void Init(in Edge edge1, in Edge edge2, in Plane3D plane)
    {
        // Make a wire consisting of two edges.

        // Get common point.
        Exploration.Vertices(edge1, out Vertex v11, out Vertex v12, true);
        Exploration.Vertices(edge2, out Vertex v21, out Vertex v22, true);
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
        var mkWire = new MakeWire();
        if (is1stReversed)
        {
            mkWire.Add(ShapeCasts.Edge(edge1.Reversed()));
        }
        else
        {
            mkWire.Add(edge1);
        }
        if (is2ndReversed)
        {
            mkWire.Add(ShapeCasts.Edge(edge2.Reversed()));
        }
        else
        {
            mkWire.Add(edge2);
        }
        if (!mkWire.IsDone)
        {
            throw new FailureException("Can't make a wire.");
        }

        Wire w = mkWire.InternalWire();
        Init(w, plane);
    }

    /// <summary>Calculates a fillet of the radius. True if it was built.</summary>
    public bool Perform(double radius)
    {
        bool bRet = false;
        if (_e1.IsNull || _e2.IsNull || radius < global::Kerf.Engine.Foundation.Precision.Confusion())
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
        if (!_segment1 || !_segment2)
        {
            var mkWire = new MakeWire(_e1, _e2);
            if (mkWire.IsDone)
            {
                Wire w = mkWire.InternalWire();
                var mkFace = new MakeFace(_plane);
                if (mkFace.IsDone)
                {
                    Face f = mkFace.InternalFace();
                    var analyzer = new WireAnalysis(w, f, global::Kerf.Engine.Foundation.Precision.Confusion());
                    if (analyzer.CheckSelfIntersection())
                    {
                        // Cut the edges at the point of intersection.
                        isCut = true;
                        if (!Cut(_plane, ref _e1, ref _e2))
                        {
                            return false;
                        }
                    }
                }
            }
        } // a case of segment - segment

        // Choose the case.
        var ac1 = new EdgeCurveAdaptor(_e1);
        var ac2 = new EdgeCurveAdaptor(_e2);
        if (_segment1 && _segment2)
        {
            bRet = SegmentFilletSegment(radius, ref xc, ref yc, ref cw, ref start, ref end);
        }
        else if (_segment1 && !_segment2)
        {
            bRet = SegmentFilletArc(radius, ref xc, ref yc, ref cw, ref start, ref end, ref xend, ref yend);
        }
        else if (!_segment1 && _segment2)
        {
            bRet = ArcFilletSegment(radius, ref xc, ref yc, ref cw, ref start, ref end, ref xstart, ref ystart);
        }
        else if (!_segment1 && !_segment2)
        {
            bRet = ArcFilletArc(radius, ref xc, ref yc, ref cw, ref start, ref end);
        }

        if (!bRet)
        {
            return false;
        }

        // Invert the fillet for left-handed plane.
        if (!_plane.Position().Direct())
        {
            cw = !cw;
        }

        // Construct a fillet.
        // Make circle.
        Point3D center = global::Kerf.Engine.Geometry.ElSLib.Value(xc, yc, _plane);
        Direction3D normal = _plane.Position().Direction();
        var circ = new Circle3D(new Frame3D(center, cw ? -normal : normal), radius);

        // Fillet may only shrink a neighbour edge, it can't prolongate it.
        double delta1 = ac1.LastParameter() - ac1.FirstParameter();
        double delta2 = ac2.LastParameter() - ac2.FirstParameter();
        if (!isCut && (start > delta1 || end > delta2))
        {
            // Check a case when a neighbour edge almost disappears:
            // try to reduce the fillet radius for a little (1.e-5 mm).
            double little = 100.0 * global::Kerf.Engine.Foundation.Precision.Confusion();
            double d1 = Math.Abs(start - delta1);
            double d2 = Math.Abs(end - delta2);
            if (d1 < little || d2 < little)
            {
                if (_segment1 && _segment2)
                {
                    bRet = SegmentFilletSegment(radius - little, ref xc, ref yc, ref cw, ref start, ref end);
                }
                else if (_segment1 && !_segment2)
                {
                    bRet = SegmentFilletArc(radius - little, ref xc, ref yc, ref cw, ref start, ref end, ref xend, ref yend);
                }
                else if (!_segment1 && _segment2)
                {
                    bRet = ArcFilletSegment(radius - little, ref xc, ref yc, ref cw, ref start, ref end, ref xstart, ref ystart);
                }
                else if (!_segment1 && !_segment2)
                {
                    bRet = ArcFilletArc(radius - little, ref xc, ref yc, ref cw, ref start, ref end);
                }
                if (bRet)
                {
                    // Invert the fillet for left-handed planes.
                    if (!_plane.Position().Direct())
                    {
                        cw = !cw;
                    }

                    // Make the circle again.
                    center = global::Kerf.Engine.Geometry.ElSLib.Value(xc, yc, _plane);
                    circ = circ.WithLocation(center);
                    circ = circ.WithRadius(radius - little);
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
                pstart = global::Kerf.Engine.Geometry.ElSLib.Value(xstart, ystart, _plane);
            }
            else
            {
                if (_e1.Orientation() == Forward)
                {
                    pstart = ac1.Value(ac1.LastParameter() - start);
                }
                else
                {
                    pstart = ac1.Value(ac1.FirstParameter() + start);
                }
            }
            // end: (xend, yend) -> pend.
            Point3D pend;
            if (xend != double.MaxValue)
            {
                pend = global::Kerf.Engine.Geometry.ElSLib.Value(xend, yend, _plane);
            }
            else
            {
                if (_e2.Orientation() == Forward)
                {
                    pend = ac2.Value(ac2.FirstParameter() + end);
                }
                else
                {
                    pend = ac2.Value(ac2.LastParameter() - end);
                }
            }

            // Make arc.
            var mkEdge = new MakeEdge(circ, pstart, pend);
            bRet = mkEdge.IsDone;
            if (bRet)
            {
                _fillet = mkEdge.Edge();

                // Limit the neighbours.
                // Left neighbour.
                Point3D p1, p2;
                _shrinke1.Nullify();
                if (_e1.Orientation() == Forward)
                {
                    p1 = ac1.Value(ac1.FirstParameter());
                    p2 = pstart;
                }
                else
                {
                    p1 = pstart;
                    p2 = ac1.Value(ac1.LastParameter());
                }
                if (_segment1)
                {
                    var mkSegment1 = new MakeEdge();
                    mkSegment1.Init(ac1.Curve().Curve()!, p1, p2);
                    if (mkSegment1.IsDone)
                    {
                        _shrinke1 = mkSegment1.Edge();
                    }
                }
                else
                {
                    var mkCirc1 = new MakeEdge();
                    mkCirc1.Init(ac1.Curve().Curve()!, p1, p2);
                    if (mkCirc1.IsDone)
                    {
                        _shrinke1 = mkCirc1.Edge();
                    }
                }

                // Right neighbour.
                _shrinke2.Nullify();
                if (_e1.Orientation() == Forward) // OCCT: e1's orientation decides e2's ends
                {
                    p1 = pend;
                    p2 = ac2.Value(ac2.LastParameter());
                }
                else
                {
                    p1 = ac2.Value(ac2.FirstParameter());
                    p2 = pend;
                }
                if (_segment2)
                {
                    var mkSegment2 = new MakeEdge();
                    mkSegment2.Init(ac2.Curve().Curve()!, p1, p2);
                    if (mkSegment2.IsDone)
                    {
                        _shrinke2 = mkSegment2.Edge();
                    }
                }
                else
                {
                    var mkCirc2 = new MakeEdge();
                    mkCirc2.Init(ac2.Curve().Curve()!, p1, p2);
                    if (mkCirc2.IsDone)
                    {
                        _shrinke2 = mkCirc2.Edge();
                    }
                }

                bRet = !_shrinke1.IsNull && !_shrinke2.IsNull;
            } // fillet edge is done
        } // shrinking is good

        return bRet;
    }

    /// <summary>The fillet, and the two edges it shrinks in <paramref name="e1"/> and <paramref name="e2"/> (frozen).</summary>
    public Edge Result(ref Edge e1, ref Edge e2)
    {
        e1 = _shrinke1;
        e2 = _shrinke2;
        return _fillet;
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
        var p11 = new Point2D(_x11, _y11);
        var p12 = new Point2D(_x12, _y12);
        var p22 = new Point2D(_x22, _y22);

        // Check length of segments.
        if (IsEqual(p12, p11) || IsEqual(p12, p22))
        {
            return false;
        }

        // Make vectors.
        var v1 = new Vector2D(p12, p11);
        var v2 = new Vector2D(p12, p22);
        v1 = v1.Normalized();
        v2 = v2.Normalized();

        // Make bisectrissa.
        Vector2D bisec = 0.5 * (v1 + v2);

        // Check bisectrissa.
        if (bisec.SquareMagnitude() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return false;
        }

        // Normalize the bisectrissa.
        bisec = bisec.Normalized();

        // Angle at bisectrissa.
        double beta = v1.Angle(bisec);

        // Length along the bisectrissa till the center of fillet.
        double l = radius / Math.Sin(Math.Abs(beta));

        // Center point of fillet.
        Point2D pc = p12.Translated(l * bisec);
        pc.Coord(out xc, out yc);

        // Shrinking length along segments.
        start = Math.Sqrt(l * l - radius * radius);
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
        var p12 = new Point2D(_x12, _y12);
        var p11 = new Point2D(_x11, _y11);
        var pc2 = new Point2D(_xc2, _yc2);

        // Check length of segment.
        if (p11.SquareDistance(p12) < Constants.Resolution())
        {
            return false;
        }

        // Make 2D vectors.
        var v1 = new Vector2D(p12, p11);
        var v2 = new Vector2D(p12, pc2);

        // Rotate the arc vector to become tangential at p21.
        if (_cw2)
        {
            v2 = v2.Rotated(+M_PI_2);
        }
        else
        {
            v2 = v2.Rotated(-M_PI_2);
        }

        // If vectors coincide (segment and arc are tangent),
        // the algorithm doesn't work...
        double angle = v1.Angle(v2);
        if (Math.Abs(angle) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return false;
        }

        // Make a bissectrisa of vectors at p12.
        v2 = v2.Normalized();
        v1 = v1.Normalized();
        Vector2D bisec = 0.5 * (v1 + v2);

        // If segment and arc look in opposite direction,
        // no fillet is possible.
        if (bisec.SquareMagnitude() < Constants.Resolution())
        {
            return false;
        }

        // Define an appropriate point to choose center of fillet.
        bisec = bisec.Normalized();
        Point2D nearp = p12.Translated(radius * bisec);
        var nearl = new Line2D(p12, new Direction2D(bisec));

        // Make a line parallel to segment and
        // passing near the "near" point.
        Vector2D d1 = v1;
        var line = new Line2D(p11, new Direction2D(-d1));
        d1 = d1.Rotated(M_PI_2);
        line = line.Translated(radius * d1);
        if (line.Distance(nearp) > radius)
        {
            line = line.Translated(-2.0 * radius * d1);
        }

        // Make a circle of radius of the arc +/- fillet radius.
        var axes = new Axis2D(pc2, Constants.DX2d());
        var circ = new Circle2D(axes, _radius2 + radius);
        if (_radius2 > radius && circ.Distance(nearp) > radius)
        {
            circ = circ.WithRadius(_radius2 - radius);
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
        double l2 = pc.SquareDistance(p12);
        double rf2 = radius * radius;
        start = Math.Sqrt(l2 - rf2);

        // Shrink of arc.
        var pcc = new Vector2D(pc2, pc);
        end = Math.Abs(new Vector2D(pc2, p12).Angle(pcc));

        // Duplicate the information on shrink the arc:
        // calculate a point on the arc coinciding with the end of fillet.
        line = line.WithLocation(pc2);
        line = line.WithDirection(new Direction2D(pcc));
        circ = circ.WithLocation(pc2);
        circ = circ.WithRadius(_radius2);
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
            if (Math.Abs(d2 - rf2) < global::Kerf.Engine.Foundation.Precision.Confusion())
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
        var p12 = new Point2D(_x12, _y12);
        var p22 = new Point2D(_x22, _y22);
        var pc1 = new Point2D(_xc1, _yc1);

        // Check length of segment.
        if (p12.SquareDistance(p22) < Constants.Resolution())
        {
            return false;
        }

        // Make 2D vectors.
        var v1 = new Vector2D(p12, pc1);
        var v2 = new Vector2D(p12, p22);

        // Rotate the arc vector to become tangential at p21.
        if (_cw1)
        {
            v1 = v1.Rotated(-M_PI_2);
        }
        else
        {
            v1 = v1.Rotated(+M_PI_2);
        }

        // If vectors coincide (segment and arc are tangent),
        // the algorithm doesn't work...
        double angle = v1.Angle(v2);
        if (Math.Abs(angle) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return false;
        }

        // Make a bisectrissa of vectors at p12.
        v1 = v1.Normalized();
        v2 = v2.Normalized();
        Vector2D bisec = 0.5 * (v1 + v2);

        // If segment and arc look in opposite direction,
        // no fillet is possible.
        if (bisec.SquareMagnitude() < Constants.Resolution())
        {
            return false;
        }

        // Define an appropriate point to choose center of fillet.
        bisec = bisec.Normalized();
        Point2D nearPoint = p12.Translated(radius * bisec);
        var nearLine = new Line2D(p12, new Direction2D(bisec));

        // Make a line parallel to segment and
        // passing near the "near" point.
        Vector2D d2Vec = v2;
        var line = new Line2D(p22, new Direction2D(-d2Vec));
        d2Vec = d2Vec.Rotated(M_PI_2);
        line = line.Translated(radius * d2Vec);
        if (line.Distance(nearPoint) > radius)
        {
            line = line.Translated(-2.0 * radius * d2Vec);
        }

        // Make a circle of radius of the arc +/- fillet radius.
        var axes = new Axis2D(pc1, Constants.DX2d());
        var circ = new Circle2D(axes, _radius1 + radius);
        if (_radius1 > radius && circ.Distance(nearPoint) > radius)
        {
            circ = circ.WithRadius(_radius1 - radius);
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
        double l2 = pc.SquareDistance(p12);
        double rf2 = radius * radius;
        end = Math.Sqrt(l2 - rf2);

        // Shrink of arc.
        var pcc = new Vector2D(pc1, pc);
        start = Math.Abs(new Vector2D(pc1, p12).Angle(pcc));

        // Duplicate the information on shrink the arc:
        // calculate a point on the arc coinciding with the start of fillet.
        line = line.WithLocation(pc1);
        line = line.WithDirection(new Direction2D(pcc));
        circ = circ.WithLocation(pc1);
        circ = circ.WithRadius(_radius1);
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
            if (Math.Abs(d2 - rf2) < global::Kerf.Engine.Foundation.Precision.SquareConfusion()) // OCCT: SegmentFilletArc compares with Confusion
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
        var pc1 = new Point2D(_xc1, _yc1);
        var pc2 = new Point2D(_xc2, _yc2);
        var p12 = new Point2D(_x12, _y12);

        // Make vectors at p12.
        var v1 = new Vector2D(pc1, p12);
        var v2 = new Vector2D(pc2, p12);

        // Rotate the vectors so that they are tangent to circles at p12.
        if (_cw1)
        {
            v1 = v1.Rotated(+M_PI_2);
        }
        else
        {
            v1 = v1.Rotated(-M_PI_2);
        }
        if (_cw2)
        {
            v2 = v2.Rotated(-M_PI_2);
        }
        else
        {
            v2 = v2.Rotated(+M_PI_2);
        }

        // Make a "check" point for choosing an offset circle.
        v1 = v1.Normalized();
        v2 = v2.Normalized();
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
        var c1 = new Circle2D(axes, _radius1 + radius);
        if (_radius1 > radius && c1.Distance(checkp) > radius)
        {
            c1 = c1.WithRadius(_radius1 - radius);
        }
        // Arc 2.
        axes = axes.WithLocation(pc2);
        var c2 = new Circle2D(axes, _radius2 + radius);
        if (_radius2 > radius && c2.Distance(checkp) > radius)
        {
            c2 = c2.WithRadius(_radius2 - radius);
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
    private bool Cut(in Plane3D plane, ref Edge e1, ref Edge e2)
    {
        Point3D p = new();
        bool found = false;
        double param1 = 0.0, param2 = 0.0;
        double f1, l1, f2, l2;
        Curve3D c1 = Tool.Curve(e1, out f1, out l1)!;
        Curve3D c2 = Tool.Curve(e2, out f2, out l2)!;
        var extrema = new ExtremaCurveCurve(c1, c2, f1, l1, f2, l2);
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
            var mkEdge1 = new MakeEdge(c1, f1, param1);
            if (mkEdge1.IsDone)
            {
                e1 = mkEdge1.Edge();

                var mkEdge2 = new MakeEdge(c2, param2, l2);
                if (mkEdge2.IsDone)
                {
                    e2 = mkEdge2.Edge();

                    Point2D p2d = global::Kerf.Engine.Geometry.Projection.ProjLib.Project(plane, p);
                    p2d.Coord(out _x12, out _y12);
                    _x21 = _x12;
                    _y21 = _y12;
                    return true;
                }
            }
        }
        return false;
    }
}
