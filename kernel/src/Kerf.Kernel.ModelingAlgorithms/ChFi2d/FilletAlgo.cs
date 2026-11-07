// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_FilletAlgo.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi2d/ChFi2d_FilletAlgo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Fillets.Planar;

/// <summary>
/// Creates a fillet edge: an arc of the given radius tangent to two edges of a plane, which must be connected by
/// their ends (shared vertices are not needed), on any curves that support point projection and are C2.
/// </summary>
/// <remarks>
/// <para>The algorithm is iterative, close to the Newton method:</para>
/// <list type="number">
/// <item>The first edge is split into N segments where probably only one root can be found; N depends on the
/// complexity of its curve.</item>
/// <item>On each segment the value and derivative of a function are computed: its argument is a parameter on the first
/// curve, which gives a point of tangency; the centre of the fillet is on the perpendicular from it; the centre is
/// projected on the second curve; the length of the projection minus the radius is the value. The derivative is
/// computed from the value at a small shift.</item>
/// <item>Newton's method finds the point of the segment where the value is nearest to zero; if not near enough, steps 2
/// and 3 are repeated with the found point as an end of the segment.</item>
/// <item>The fillet is built from a root (the start point), the projection on the second curve (the end point) and
/// the centre; the edges are cut at the points of tangency.</item>
/// </list>
/// <para>A working object; its results are new, frozen edges. The edges it is given are only read.</para>
/// </remarks>
internal sealed class FilletAlgo
{
    //! Initial edges where the fillet must be computed.
    private Edge _edge1 = new(), _edge2 = new();
    //! Plane where fillet arc must be created.
    private PlaneSurface? _plane;
    //! Underlying curves of the initial edges
    private Curve2D? _curve1, _curve2;
    //! Start and end parameters of curves of initial edges.
    private double _start1, _end1, _start2, _end2, _radius;
    //! List of params where roots were found.
    private readonly NCollection_List<double> _resultParams = new();
    //! sequence of 0 or 1: position of the fillet relatively to the first curve
    private readonly NCollection_Sequence<int> _resultOrientation = new();
    //! position of the fillet relatively to the first curve
    private bool _startSide;
    //! are initial edges where exchanged in the beginning: to make first edge
    //! more simple and minimize number of iterations
    private bool _edgesExchnged;
    //! Number to avoid infinity recursion: indicates how deep the recursion is performed.
    private int _degreeOfRecursion;

    /// <summary>An empty algorithm: call <c>Init</c> before <see cref="Perform"/>.</summary>
    public FilletAlgo()
    {
        _start1 = 0.0;
        _end1 = 0.0;
        _start2 = 0.0;
        _end2 = 0.0;
        _radius = 0.0;
        _startSide = false;
        _edgesExchnged = false;
        _degreeOfRecursion = 0;
    }

    /// <summary>A fillet algorithm on a wire of two edges in a plane.</summary>
    public FilletAlgo(in Wire wire, in Plane3D plane)
        : this()
    {
        Init(wire, plane);
    }

    /// <summary>A fillet algorithm on two edges in a plane.</summary>
    public FilletAlgo(in Edge edge1, in Edge edge2, in Plane3D plane)
        : this()
    {
        _edge1 = edge1;
        _edge2 = edge2;
        Init(edge1, edge2, plane);
    }

    /// <summary>Initializes the algorithm by a wire of two edges in a plane.</summary>
    /// <exception cref="ConstructionErrorException">The wire has fewer than two edges.</exception>
    public void Init(in Wire wire, in Plane3D plane)
    {
        Edge theEdge1 = new(), theEdge2 = new();
        var itr = new Iterator(wire);
        for (; itr.More(); itr.Next())
        {
            if (theEdge1.IsNull)
            {
                theEdge1 = ShapeCasts.Edge(itr.Value());
            }
            else if (theEdge2.IsNull)
            {
                theEdge2 = ShapeCasts.Edge(itr.Value());
            }
            else
            {
                break;
            }
        }
        if (theEdge1.IsNull || theEdge2.IsNull)
        {
            throw new ConstructionErrorException("The fillet algorithms expects a wire consisting of two edges.");
        }
        Init(theEdge1, theEdge2, plane);
    }

    /// <summary>Initializes the algorithm by two edges in a plane.</summary>
    public void Init(in Edge edge1, in Edge edge2, in Plane3D plane)
    {
        _plane = new PlaneSurface(plane);

        _edgesExchnged = false;

        var bac1 = new EdgeCurveAdaptor(edge1);
        var bac2 = new EdgeCurveAdaptor(edge2);
        if (bac1.GetType() < bac2.GetType())
        { // first curve must be more complicated
            _edge1 = edge2;
            _edge2 = edge1;
            _edgesExchnged = true;
        }
        else
        {
            _edge1 = edge1;
            _edge2 = edge2;
        }

        Curve3D curve1 = Tool.Curve(_edge1, out _start1, out _end1)!;
        Curve3D curve2 = Tool.Curve(_edge2, out _start2, out _end2)!;

        _curve1 = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(curve1, _start1, _end1, _plane)!;
        _curve2 = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(curve2, _start2, _end2, _plane)!;

        for (int aStep = 0; _curve1.IsPeriodic && _start1 >= _end1; StepLimit.Next(ref aStep))
        {
            _end1 += _curve1.Period;
        }
        for (int aStep = 0; _curve2.IsPeriodic && _start2 >= _end2; StepLimit.Next(ref aStep))
        {
            _end2 += _curve2.Period;
        }

        if (bac1.GetType() == bac2.GetType())
        {
            if (_end2 - _start2 < _end1 - _start1)
            { // first curve must be parametrically shorter
                Edge edge = _edge1;
                _edge1 = _edge2;
                _edge2 = edge;
                Curve2D curve = _curve1;
                _curve1 = _curve2;
                _curve2 = curve;
                double a = _start1;
                _start1 = _start2;
                _start2 = a;
                a = _end1;
                _end1 = _end2;
                _end2 = a;
                _edgesExchnged = true;
            }
        }
    }

    //! This function returns true if linear segment from start point of the
    //! fillet arc to the end point is intersected by the first or second
    //! curve: in this case fillet is invalid.
    private static bool IsRadiusIntersected(Curve2D curve,
                                            double curveMin,
                                            double curveMax,
                                            Point2D start,
                                            Point2D end,
                                            bool startConnected)
    {
        // Check the given start and end if they are identical. If yes
        // return false
        if (start.SquareDistanceTo(end) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return false;
        }
        var line = new LineCurve2D(start, new Direction2D(new Vector2D(start, end)));
        var inter = new CurveCurveIntersection2D(curve, line, global::Kerf.Engine.Foundation.Precision.Confusion());
        int a;
        Point2D point;
        for (a = inter.NbPoints(); a > 0; a--)
        {
            point = inter.Point(a);
            var projInt = new ProjectPointOnCurve2D(point, curve, curveMin, curveMax);
            // OCCT compares the parameter of the nearest projection, not its distance, with the tolerance
            if (projInt.NbPoints() < 1 || projInt.LowerDistanceParameter() > global::Kerf.Engine.Foundation.Precision.Confusion()) // OCCT: a parameter
            {
                continue; // point is not on edge
            }

            if (point.DistanceTo(start) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                if (!startConnected)
                {
                    return true;
                }
            }
            if (point.DistanceTo(end) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return true;
            }
            if (new Vector2D(point, start).IsOpposite(new Vector2D(point, end), global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                return true;
            }
        }
        Curve2D aCurve = curve;
        for (a = inter.NbSegments(); a > 0; a--)
        {
            // anInter.Segment(a, aCurve); //not implemented (bug in OCC)
            point = aCurve.Value(aCurve.FirstParameter);

            var projInt = new ProjectPointOnCurve2D(point, curve, curveMin, curveMax);
            if (projInt.NbPoints() != 0 && projInt.LowerDistanceParameter() < global::Kerf.Engine.Foundation.Precision.Confusion()) // OCCT: a parameter
            { // point is on edge
                if (point.DistanceTo(start) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    if (!startConnected)
                    {
                        return true;
                    }
                }
                if (point.DistanceTo(end) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    return true;
                }
                if (new Vector2D(point, start).IsOpposite(new Vector2D(point, end), global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return true;
                }
            }
            point = aCurve.Value(aCurve.LastParameter);

            projInt.Init(point, curve, curveMin, curveMax);
            if (projInt.NbPoints() != 0 && projInt.LowerDistanceParameter() < global::Kerf.Engine.Foundation.Precision.Confusion()) // OCCT: a parameter
            { // point is on edge
                if (point.DistanceTo(start) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    if (!startConnected)
                    {
                        return true;
                    }
                }
                if (point.DistanceTo(end) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    return true;
                }
                if (new Vector2D(point, start).IsOpposite(new Vector2D(point, end), global::Kerf.Engine.Foundation.Precision.Angular()))
                {
                    return true;
                }
            }
        }
        return false;
    }

    //! Computes the value the function in the current point.
    //! <theLimit> is end parameter of the segment
    private void FillPoint(global::Kerf.Engine.Fillets.Planar.FilletPoint point, double limit)
    {
        // on the intersection point
        bool valid = false;
        double step = global::Kerf.Engine.Foundation.Precision.Confusion();
        Point2D center = new(), aPoint = new(); // center of fillet and point on curve1
        double param = point.GetParam();
        if (limit < param)
        {
            step = -step;
        }
        for (valid = false; !valid; param += step)
        {
            if ((param - step - limit) * (param - limit) <= 0)
            {
                break; // limit was exceeded
            }
            step *= 2;
            _curve1!.D1(param, out aPoint, out Vector2D vec);
            if (vec.SquareMagnitude < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                continue;
            }

            var perp = new Vector2D(((_startSide) ? -1 : 1) * vec.Y, ((_startSide) ? 1 : -1) * vec.X);
            perp = perp.Normalized();
            perp = perp.Multiplied(_radius);
            center = aPoint.Translated(perp);

            var projInt = new ProjectPointOnCurve2D(aPoint, _curve2!, _start2, _end2);
            if (projInt.NbPoints() == 0
                || aPoint.DistanceTo(projInt.NearestPoint()) > global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                valid = true;
                break;
            }
        }
        if (valid)
        {
            point.SetParam(param);
            point.SetCenter(center);
            valid = !IsRadiusIntersected(_curve2!, _start2, _end2, aPoint, center, true);
        }

        var proj = new ProjectPointOnCurve2D(center, _curve2!);
        int a, nb = proj.NbPoints();
        for (a = nb; a > 0; a--)
        {
            if (aPoint.SquareDistanceTo(proj.Point(a)) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                continue;
            }

            bool valid2 = valid;
            if (valid2)
            {
                valid2 = !IsRadiusIntersected(_curve1!, _start1, _end1, center, proj.Point(a), false);
            }

            // checking the right parameter
            double paramProj = proj.Parameter(a);
            for (int aStep = 0; _curve2!.IsPeriodic && paramProj < _start2; StepLimit.Next(ref aStep))
            {
                paramProj += _curve2.Period;
            }

            double d = proj.Distance(a);
            point.AppendValue(d * d - _radius * _radius,
                                 (paramProj >= _start2 && paramProj <= _end2 && valid2));
            if (Math.Abs(d - _radius) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                point.SetParam2(paramProj);
            }
        }
    }

    //! Computes the derivative value of the function in the current point.
    //! <theDiffStep> is small step for approximate derivative computation
    //! <theFront> is direction of the step: from or reversed
    private void FillDiff(global::Kerf.Engine.Fillets.Planar.FilletPoint point, double diffStep, bool front)
    {
        double delta = front ? (diffStep) : (-diffStep);
        var diff = new global::Kerf.Engine.Fillets.Planar.FilletPoint(point.GetParam() + delta);
        FillPoint(diff, delta * 999.0);
        if (!point.CalculateDiff(diff))
        {
            diff.SetParam(point.GetParam() - delta);
            FillPoint(diff, -delta * 999);
            point.CalculateDiff(diff);
        }
    }

    /// <summary>Constructs a fillet edge of the radius. True if at least one result was found.</summary>
    public bool Perform(double radius)
    {
        _degreeOfRecursion = 0;
        _resultParams.Clear();
        _resultOrientation.Clear();

        double nbSteps;
        var gac = new CurveAdaptor2D(_curve1!);
        switch (gac.GetType())
        {
            case Line:
                nbSteps = 2;
                break;
            case Circle:
                nbSteps = 4;
                break;
            case Ellipse:
                nbSteps = 5;
                break;
            case BSplineCurve:
                nbSteps = 2 + gac.Degree() * gac.PoleCount;
                break;
            default: // unknown: maximum
                nbSteps = 100;
                break;
        }
        // std::cout<<"aNBSteps = "<<aNBSteps<<std::endl;

        _radius = radius;
        double param, step, dStep;
        step = (_end1 - _start1) / nbSteps;
        dStep = 1.0e-4 * step;

        int cycle;
        for (cycle = 2, _startSide = false; cycle != 0; _startSide = !_startSide, cycle--)
        {
            global::Kerf.Engine.Fillets.Planar.FilletPoint? left = null, right;

            for (param = _start1 + step;
                 param < _end1 || Math.Abs(_end1 - param) < global::Kerf.Engine.Foundation.Precision.Confusion();
                 param += step)
            {
                if (left is null)
                {
                    left = new global::Kerf.Engine.Fillets.Planar.FilletPoint(param - step);
                    FillPoint(left, param);
                    FillDiff(left, dStep, true);
                }

                right = new global::Kerf.Engine.Fillets.Planar.FilletPoint(param);
                FillPoint(right, param - step);
                FillDiff(right, dStep, false);

                left.FilterPoints(right);
                PerformNewton(left, right);

                left = right;
            } // for
        } // for

        return !_resultParams.IsEmpty();
    }

    //! Splits segment by the parameter and calls Newton method for both segments.
    //! It supplies recursive iterations of the Newton methods calls
    //! (PerformNewton calls this function and this calls Netwton two times).
    private bool ProcessPoint(global::Kerf.Engine.Fillets.Planar.FilletPoint left, global::Kerf.Engine.Fillets.Planar.FilletPoint right, double parameter)
    {
        if (parameter >= left.GetParam() && parameter < right.GetParam())
        {
            double dx = (right.GetParam() - left.GetParam());
            if (parameter - left.GetParam() < dx / 100.0)
            {
                parameter = left.GetParam() + dx / 100.0;
            }
            if (right.GetParam() - parameter < dx / 100.0)
            {
                parameter = right.GetParam() - dx / 100.0;
            }

            // Protection on infinite loops.
            _degreeOfRecursion++;
            double diffx = 0.001 * dx;
            if (_degreeOfRecursion > 100)
            {
                diffx *= 10.0;
                if (_degreeOfRecursion > 1000)
                {
                    diffx *= 10.0;
                    if (_degreeOfRecursion > 3000)
                    {
                        return true;
                    }
                }
            }

            global::Kerf.Engine.Fillets.Planar.FilletPoint point1 = left.Copy();
            var point2 = new global::Kerf.Engine.Fillets.Planar.FilletPoint(parameter);
            FillPoint(point2, point1.GetParam());
            FillDiff(point2, diffx, true);

            point1.FilterPoints(point2);
            PerformNewton(point1, point2);
            point2.FilterPoints(right);
            PerformNewton(point2, right);

            return true;
        }

        return false;
    }

    //! Using Newton methods computes optimal point, that can be root of the
    //! function taking into account two input points, functions value and derivatives.
    //! Performs iteration until root is found or failed to find root.
    //! Stores roots in myResultParams.
    private void PerformNewton(global::Kerf.Engine.Fillets.Planar.FilletPoint left, global::Kerf.Engine.Fillets.Planar.FilletPoint right)
    {
        int a;
        // check the left: if this is solution store it and remove it from the list of researching points
        // of theLeft
        a = left.HasSolution(_radius);
        if (a != 0)
        {
            if (left.IsValid(a))
            {
                _resultParams.Append(left.GetParam());
                _resultOrientation.Append(_startSide ? 1 : 0);
            }
            return;
        }

        double dx = right.GetParam() - left.GetParam();
        if (dx < 1.0e-6 * global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            a = right.HasSolution(_radius);
            if (a != 0 && right.IsValid(a))
            {
                _resultParams.Append(right.GetParam());
                _resultOrientation.Append(_startSide ? 1 : 0);
            }
            return;
        }
        for (a = 1; a <= left.GetNBValues(); a++)
        {
            int near = left.GetNear(a);

            double aA = (right.GetDiff(near) - left.GetDiff(a)) / dx;
            double b = left.GetDiff(a) - aA * left.GetParam();
            double c = left.GetValue(a) - left.GetDiff(a) * left.GetParam()
                        + aA * left.GetParam() * left.GetParam() / 2.0;
            double det = b * b - 2.0 * aA * c;

            if (Math.Abs(aA) < global::Kerf.Engine.Foundation.Precision.Confusion())
            { // linear case
                // std::cout<<"###"<<std::endl;
                if (Math.Abs(b) > 10e-20)
                {
                    double x0 = -c / b; // use extremum
                    if (x0 > left.GetParam() && x0 < right.GetParam())
                    {
                        ProcessPoint(left, right, x0);
                    }
                }
                else
                {
                    ProcessPoint(left,
                                 right,
                                 left.GetParam() + dx / 2.0); // linear division otherwise
                }
            }
            else
            {
                if (Math.Abs(b) > Math.Abs(det * 1000000.0))
                { // possible floating point operations accuracy errors
                    // std::cout<<"*";
                    ProcessPoint(left,
                                 right,
                                 left.GetParam() + dx / 2.0); // linear division otherwise
                }
                else
                {
                    if (det > 0)
                    { // two solutions
                        det = Math.Sqrt(det);
                        bool res = ProcessPoint(left, right, (-b + det) / aA);
                        if (!res)
                        {
                            res = ProcessPoint(left, right, (-b - det) / aA);
                        }
                        if (!res)
                        {
                            ProcessPoint(left,
                                         right,
                                         left.GetParam() + dx / 2.0); // linear division otherwise
                        }
                    }
                    else
                    {
                        // std::cout<<"%%%"<<std::endl;
                        double x0 = -b / aA; // use extremum
                        if (x0 > left.GetParam() && x0 < right.GetParam())
                        {
                            ProcessPoint(left, right, x0);
                        }
                        else
                        {
                            ProcessPoint(left,
                                         right,
                                         left.GetParam() + dx / 2.0); // linear division otherwise
                        }
                    }
                }
            }
        } // for
    }

    /// <summary>
    /// The number of solutions; <paramref name="point"/> (the intersecting or common point of the edges) chooses
    /// among several fillets.
    /// </summary>
    public int NbResults(in Point3D point)
    {
        double x, y;
        Point2D targetPoint2d = new();
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(_plane!.Plane.Position, point, out x, out y);
        targetPoint2d = targetPoint2d.WithCoord(x, y);

        // iterate through all possible solutions.
        int i = 1, nb = 0;
        var iter = new NCollection_List<double>.Iterator(_resultParams);
        for (; iter.More(); iter.Next(), i++)
        {
            _startSide = (_resultOrientation.Value(i)) != 0;
            var aPoint = new global::Kerf.Engine.Fillets.Planar.FilletPoint(iter.Value());
            FillPoint(aPoint, iter.Value() + 1.0);
            if (aPoint.HasSolution(_radius) != 0)
            {
                nb++;
            }
        } // for

        return nb;
    }

    /// <summary>
    /// The fillet edge and the two edges it trims (all frozen): the solution nearest to <paramref name="point"/> if
    /// <paramref name="iSolution"/> is -1, else that solution. A null edge if there is none.
    /// </summary>
    public Edge Result(in Point3D point, ref Edge edge1, ref Edge edge2, int iSolution = -1)
    {
        Edge result = new();
        Point2D targetPoint2d = new();
        double x, y;
        global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(_plane!.Plane.Position, point, out x, out y);
        targetPoint2d = targetPoint2d.WithCoord(x, y);

        // choose the nearest circle
        double distance = 0.0, p;
        global::Kerf.Engine.Fillets.Planar.FilletPoint? nearest;
        int a, iSol = 1;
        var iter = new NCollection_List<double>.Iterator(_resultParams);
        for (nearest = null, a = 1; iter.More(); iter.Next(), a++)
        {
            _startSide = (_resultOrientation.Value(a)) != 0;
            var aPoint = new global::Kerf.Engine.Fillets.Planar.FilletPoint(iter.Value());
            FillPoint(aPoint, iter.Value() + 1.0);
            if (aPoint.HasSolution(_radius) == 0)
            {
                continue;
            }
            p = double.MaxValue;
            if (iSolution == -1)
            {
                p = Math.Abs(aPoint.GetCenter().DistanceTo(targetPoint2d) - _radius);
            }
            else if (iSolution == iSol)
            {
                p = 0.0;
            }
            if (nearest is null || p < distance)
            {
                nearest = aPoint;
                distance = p;
            }
            if (iSolution == iSol)
            {
                break;
            }
            iSol++;
        } // for

        if (nearest is null)
        {
            return result;
        }

        // create circle edge
        Point3D center = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(nearest.GetCenter().X,
                                           nearest.GetCenter().Y,
                                           _plane.Plane.Position);
        var circle = new CircleCurve3D(new Frame3D(center, _plane.Plane.Axis.Direction), _radius);
        _curve1!.D0(nearest.GetParam(), out Point2D point2d1);
        _curve2!.D0(nearest.GetParam2(), out Point2D point2d2);
        Point3D point1 = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(point2d1.X, point2d1.Y, _plane.Plane.Position);
        Point3D point2 = global::Kerf.Engine.Geometry.ElSLib.PlaneValue(point2d2.X, point2d2.Y, _plane.Plane.Position);

        var proj = new ProjectPointOnCurve(point, circle);
        double targetParam = proj.LowerDistanceParameter();
        Point3D pointOnCircle = proj.NearestPoint();

        // There is a bug in Open CASCADE in calculation of nearest point to a circle near the parameter
        // 0.0 Therefore I check this extrema point manually:
        Point3D p0 = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, circle.Circle);
        if (p0.DistanceTo(point) < pointOnCircle.DistanceTo(point))
        {
            targetParam = 0.0;
            pointOnCircle = p0;
        }

        proj.Perform(point1);
        double param1 = proj.LowerDistanceParameter();
        proj.Perform(point2);
        double param2 = proj.LowerDistanceParameter();
        bool isOut = ((param1 < targetParam && param2 < targetParam)
                       || (param1 > targetParam && param2 > targetParam));
        if (param1 > param2)
        {
            isOut = !isOut;
        }
        var builder = new MakeEdge(circle.Circle,
                                                   isOut ? param2 : param1,
                                                   isOut ? param1 : param2);
        result = builder.Edge();

        // divide edges
        double start, end;
        Curve3D curve = Tool.Curve(_edge1, out start, out end)!;
        curve.D1(nearest.GetParam(), out point1, out Vector3D dir);

        circle.D1(param1, out point1, out Vector3D circleDir);

        if ((circleDir.AngleTo(dir) > M_PI / 2.0) ? !isOut : isOut)
        {
            start = nearest.GetParam();
        }
        else
        {
            end = nearest.GetParam();
        }

        // Check the case when start and end are identical. This happens
        // when the edge decreases to size 0. Old ww5 allows such
        // cases. So we are again bug compatible
        if (Math.Abs(start - end) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            end = start + global::Kerf.Engine.Foundation.Precision.Confusion();
        }
        // Divide edge
        var divider1 = new MakeEdge(curve, start, end);
        if (_edgesExchnged)
        {
            edge2 = divider1.Edge();
        }
        else
        {
            edge1 = divider1.Edge();
        }

        curve = Tool.Curve(_edge2, out start, out end)!;
        curve.D1(nearest.GetParam2(), out point2, out dir);

        circle.D1(param2, out point2, out circleDir);

        if ((circleDir.AngleTo(dir) > M_PI / 2.0) ? isOut : !isOut)
        {
            start = nearest.GetParam2();
        }
        else
        {
            end = nearest.GetParam2();
        }

        // Check the case when start and end are identical. This happens
        // when the edge decreases to size 0. Old ww5 allows such
        // cases. So we are again bug compatible
        if (Math.Abs(start - end) < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            end = start + global::Kerf.Engine.Foundation.Precision.Confusion();
        }
        var divider2 = new MakeEdge(curve, start, end);
        if (_edgesExchnged)
        {
            edge1 = divider2.Edge();
        }
        else
        {
            edge2 = divider2.Edge();
        }

        return result;
    }
}

/// <summary>
/// A point on the first curve of <see cref="FilletAlgo"/>, with the fillet function's values and derivatives
/// there (OCCT's private class FilletPoint, declared in ChFi2d_FilletAlgo.hxx).
/// </summary>
internal sealed class FilletPoint
{
    //! Parameter on the first curve (start fillet point).
    private double _param;
    //! Parameter on the second curve (end fillet point).
    private double _param2;
    //! Values and derivative values of the fillet function.
    //! May be several if there are many projections on the second curve.
    private readonly NCollection_Sequence<double> _v = new(), _d = new();
    //! Center of the fillet arc.
    private Point2D _center = new();
    //! Flags for storage the validity of solutions. Indexes corresponds to indexes
    //! in sequences myV, myD.
    private readonly NCollection_Sequence<bool> _valid = new();
    private readonly NCollection_Sequence<int> _near = new();

    //! Creates a point on a first curve by parameter on this curve.
    public FilletPoint(double param)
    {
        _param = param;
        _param2 = 0.0;
    }

    //! Changes the point position by changing point parameter on the first curve.
    public void SetParam(double param) { _param = param; }

    //! Returns the point parameter on the first curve.
    public double GetParam() { return _param; }

    //! Returns number of found values of function in this point.
    public int GetNBValues() { return _v.Length(); }

    //! Returns value of function in this point.
    public double GetValue(int index) { return _v.Value(index); }

    //! Returns derivatives of function in this point.
    public double GetDiff(int index) { return _d.Value(index); }

    //! Returns true if function is valid (rediuses vectors of fillet do not intersect any curve).
    public bool IsValid(int index) { return _valid.Value(index); }

    //! Returns the index of the nearest value
    public int GetNear(int index) { return _near.Value(index); }

    //! Defines the parameter of the projected point on the second curve.
    public void SetParam2(double param2) { _param2 = param2; }

    //! Returns the parameter of the projected point on the second curve.
    public double GetParam2() { return _param2; }

    //! Center of the fillet.
    public void SetCenter(Point2D point) { _center = point; }

    //! Center of the fillet.
    public Point2D GetCenter() { return _center; }

    //! For debug only
    public double LowerValue()
    {
        int a, resultIndex = 0;
        double value = 0.0; // set before it is read, as in OCCT
        for (a = _v.Length(); a > 0; a--)
        {
            if (resultIndex == 0 || Math.Abs(value) > Math.Abs(_v.Value(a)))
            {
                resultIndex = a;
                value = _v.Value(a);
            }
        }
        return value;
    }

    //! Appends value of the function.
    public void AppendValue(double value, bool valid)
    {
        int a;
        for (a = 1; a <= _v.Length(); a++)
        {
            if (value < _v.Value(a))
            {
                _v.InsertBefore(a, value);
                _valid.InsertBefore(a, valid);
                return;
            }
        }
        _v.Append(value);
        _valid.Append(valid);
    }

    //! Computes difference between this point and the given. Stores difference in myD.
    public bool CalculateDiff(global::Kerf.Engine.Fillets.Planar.FilletPoint point)
    {
        int a;
        bool diffsSet = (_d.Length() != 0);
        double dx = point.GetParam() - _param, dy = 0.0;
        if (point._v.Length() == _v.Length())
        { // absolutely the same points
            for (a = 1; a <= _v.Length(); a++)
            {
                dy = point._v.Value(a) - _v.Value(a);
                if (diffsSet)
                {
                    _d.SetValue(a, dy / dx);
                }
                else
                {
                    _d.Append(dy / dx);
                }
            }
            return true;
        }
        // between the diffeerent points searching for nearest analogs
        int b;
        for (a = 1; a <= _v.Length(); a++)
        {
            for (b = 1; b <= point._v.Length(); b++)
            {
                if (b == 1 || Math.Abs(point._v.Value(b) - _v.Value(a)) < Math.Abs(dy))
                {
                    dy = point._v.Value(b) - _v.Value(a);
                }
            }
            if (diffsSet)
            {
                if (Math.Abs(dy / dx) < Math.Abs(_d.Value(a)))
                {
                    _d.SetValue(a, dy / dx);
                }
            }
            else
            {
                _d.Append(dy / dx);
            }
        } // for

        return false;
    }

    //! Filters out the values and leaves the most optimal one.
    public void FilterPoints(global::Kerf.Engine.Fillets.Planar.FilletPoint point)
    {
        int a, b;
        NCollection_Sequence<double> diffs = new();
        double y, y2, dx = point.GetParam() - _param;
        for (a = 1; a <= _v.Length(); a++)
        {
            // searching for near point from thePoint
            int near = 0;
            double diff = dx * 10000.0;
            y = _v.Value(a) + _d.Value(a) * dx;
            for (b = 1; b <= point._v.Length(); b++)
            {
                // calculate hypothesis value of the Y2 with the constant first and second derivative
                y2 = y + dx * (point._d.Value(b) - _d.Value(a)) / 2.0;
                if (near == 0 || Math.Abs(y2 - point._v.Value(b)) < Math.Abs(diff))
                {
                    near = b;
                    diff = y2 - point._v.Value(b);
                }
            } // for b...

            if (near != 0)
            {
                if (_v.Value(a) * point._v.Value(near) > 0)
                { // the same sign at the same sides of the interval
                    if (_v.Value(a) * _d.Value(a) > 0)
                    {
                        if (Math.Abs(_d.Value(a)) > global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            near = 0;
                        }
                    }
                    else
                    {
                        if (Math.Abs(_v.Value(a)) > Math.Abs(point._v.Value(near)))
                        {
                            if (point._v.Value(near) * point._d.Value(near) < 0
                                && Math.Abs(point._d.Value(near)) > global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                near = 0;
                            }
                        }
                    }
                }
            } // if aNear

            if (near != 0)
            {
                if (_v.Value(a) * point._v.Value(near) > 0)
                {
                    if ((_v.Value(a) + _d.Value(a) * dx) * _v.Value(a) > global::Kerf.Engine.Foundation.Precision.Confusion()
                        && (point._v.Value(near) + point._d.Value(near) * dx)
                               * point._v.Value(near)
                           > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        near = 0;
                    }
                }
            } // if aNear

            if (near != 0)
            {
                if (Math.Abs(diff / dx) > 1.0e+7)
                {
                    near = 0;
                }
            }

            if (near == 0)
            { // there is no near: remove it from the list
                _v.Remove(a);
                _d.Remove(a);
                _valid.Remove(a);
                a--;
            }
            else
            {
                bool found = false;
                for (b = 1; b <= _near.Length(); b++)
                {
                    if (_near.Value(b) == near)
                    {
                        if (Math.Abs(diffs.Value(b)) < Math.Abs(diff))
                        { // return this 'near'
                            found = true;
                            _v.Remove(a);
                            _d.Remove(a);
                            _valid.Remove(a);
                            a--;
                            break;
                        }
                        else
                        { // remove the old 'near'
                            // OCCT: b indexes myNear, used here to index myV, myD and myValid too
                            _v.Remove(b);
                            _d.Remove(b);
                            _valid.Remove(b);
                            _near.Remove(b);
                            diffs.Remove(b);
                            a--;
                            break;
                        }
                    }
                } // for b...
                if (!found)
                {
                    _near.Append(near);
                    diffs.Append(diff);
                }
            } // else
        } // for a...
    }

    //! Returns a pointer to created copy of the point
    //! warning: this is not the full copy! Copies only: myParam, myV, myD, myValid
    public global::Kerf.Engine.Fillets.Planar.FilletPoint Copy()
    {
        // OCCT: myNear isn't copied (FilterPoints rebuilds it before PerformNewton reads it)
        var copy = new global::Kerf.Engine.Fillets.Planar.FilletPoint(_param);
        int a;
        for (a = 1; a <= _v.Length(); a++)
        {
            copy._v.Append(_v.Value(a));
            copy._d.Append(_d.Value(a));
            copy._valid.Append(_valid.Value(a));
        }
        return copy;
    }

    //! Returns the index of the solution or zero if there is no solution
    public int HasSolution(double radius)
    {
        int a;
        for (a = 1; a <= _v.Length(); a++)
        {
            if (Math.Abs(Math.Sqrt(Math.Abs(Math.Abs(_v.Value(a)) + radius * radius)) - radius)
                < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return a;
            }
        }
        return 0;
    }

    //! Removes the found value by the given index.
    public void Remove(int index)
    {
        _v.Remove(index);
        _d.Remove(index);
        _valid.Remove(index);
        _near.Remove(index);
    }
}
