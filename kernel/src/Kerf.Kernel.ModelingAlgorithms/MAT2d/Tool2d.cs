// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Tool2d.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Tool2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Offsets.Bisectors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>The geometry of the bisecting locus for MAT2d_Mat2d: the items of the circuit, the bisectors (as
/// Bisector_Bisec), the points and vectors, each indexed from 1.</summary>
/// <remarks>
/// A working object of MAT2d_Mat2d and BRepMAT2d_BisectingLocus (decision 21). The bisectors' curves are trimmed in
/// place, as OCCT does: their basis curves are read with <c>ChangeBasisCurve</c>, which doesn't freeze them. The
/// maps are keyed by integers and only looked up. Dump throws Standard_NotImplemented, as OCCT's release build.
/// </remarks>
internal sealed class Tool2d
{
    // OCCT's static double, never changed.
    private const double MAT2d_TOLCONF = 1.0e-7;

    private double _direction;
    private JoinKind _joinType;
    private int _numberOfBisectors;
    private int _numberOfPnts;
    private int _numberOfVecs;
    private Circuit? _circuit;
    private readonly Dictionary<int, Bisec> _geomBisectors = new();
    private readonly Dictionary<int, Point2D> _geomPnts = new();
    private readonly Dictionary<int, Vector2D> _geomVecs = new();
    private readonly NCollection_Sequence<int> _linesLength = new();

    public Tool2d()
    {
        _direction = 1.0;
        _joinType = Arc; // default
        _numberOfBisectors = 0;
        _numberOfVecs = 0;
        _numberOfPnts = 0;
    }

    /// <summary>Starts from the items of the circuit EquiCircuit (the geometry of MAT's basic elements).</summary>
    public void InitItems(Circuit equiCircuit)
    {
        _geomBisectors.Clear();
        _geomPnts.Clear();
        _geomVecs.Clear();
        _linesLength.Clear();
        _numberOfBisectors = 0;
        _numberOfVecs = 0;
        _numberOfPnts = 0;

        _circuit = equiCircuit;
    }

    /// <summary>The side of the computation of the map.</summary>
    public void Sense(MedialSide aside)
    {
        if (aside == MedialSide.Left)
        {
            _direction = 1.0;
        }
        else
        {
            _direction = -1.0;
        }
    }

    public void SetJoinType(JoinKind aJoinType) => _joinType = aJoinType;

    /// <summary>The number of items.</summary>
    public int NumberOfItems() => _circuit!.NumberOfItems();

    /// <summary>The tolerance of confusion of two points.</summary>
    public double ToleranceOfConfusion() => 2 * MAT2d_TOLCONF;

    /// <summary>Creates the point at the origin of the bisector of anitem and the previous item; dist is its
    /// distance to anitem. Returns its index.</summary>
    public int FirstPoint(int anitem, out double dist)
    {
        Curve2D curve;
        Point2 point;
        _numberOfPnts++;

        if (_circuit!.ConnexionOn(anitem))
        {
            Point2D p1 = _circuit.Connexion(anitem).PointOnFirst();
            Point2D p2 = _circuit.Connexion(anitem).PointOnSecond();
            _geomPnts[_numberOfPnts] = new Point2D((p1.X + p2.X) * 0.5, (p1.Y + p2.Y) * 0.5);
            dist = p1.DistanceTo(p2) * 0.5;
            return _numberOfPnts;
        }

        Type type = _circuit.Value(anitem).GetType();
        dist = 0.0;

        if (type != typeof(CartesianPoint2))
        {
            curve = (Curve2D)_circuit.Value(anitem);
            _geomPnts[_numberOfPnts] = curve.Value(curve.FirstParameter);
        }
        else
        {
            point = (Point2)_circuit.Value(anitem);
            _geomPnts[_numberOfPnts] = point.Pnt2d();
        }
        return _numberOfPnts;
    }

    /// <summary>Creates the tangent at the end of the item anitem; returns its index.</summary>
    public int TangentBefore(int anitem, bool isOpenResult)
    {
        int item;
        Curve2D curve;
        _numberOfVecs++;

        if (!isOpenResult)
        {
            item = (anitem == _circuit!.NumberOfItems()) ? 1 : (anitem + 1);
        }
        else
        {
            item = (anitem == _circuit!.NumberOfItems()) ? (anitem - 1) : (anitem + 1);
        }
        if (_circuit.ConnexionOn(item))
        {
            _circuit.Connexion(item).PointOnFirst().Coord(out double x1, out double y1);
            _circuit.Connexion(item).PointOnSecond().Coord(out double x2, out double y2);
            _geomVecs[_numberOfVecs] = new Vector2D((x2 - x1), (y2 - y1));
            return _numberOfVecs;
        }

        Type type = _circuit.Value(anitem).GetType();
        if (type != typeof(CartesianPoint2))
        {
            curve = (Curve2D)_circuit.Value(anitem);
            _geomVecs[_numberOfVecs] = curve.DN(curve.LastParameter, 1);
        }
        else
        {
            curve = (Curve2D)_circuit.Value(item);
            double param = (isOpenResult && anitem == _circuit.NumberOfItems())
                ? curve.LastParameter
                : curve.FirstParameter;
            _geomVecs[_numberOfVecs] = curve.DN(param, 1);
        }

        return _numberOfVecs;
    }

    /// <summary>Creates the reversed tangent at the origin of the item anitem; returns its index.</summary>
    public int TangentAfter(int anitem, bool isOpenResult)
    {
        int item;
        Curve2D curve;
        Vector2D thevector;
        _numberOfVecs++;

        if (_circuit!.ConnexionOn(anitem))
        {
            _circuit.Connexion(anitem).PointOnFirst().Coord(out double x1, out double y1);
            _circuit.Connexion(anitem).PointOnSecond().Coord(out double x2, out double y2);
            _geomVecs[_numberOfVecs] = new Vector2D((x1 - x2), (y1 - y2));
            return _numberOfVecs;
        }

        Type type = _circuit.Value(anitem).GetType();
        if (type != typeof(CartesianPoint2))
        {
            curve = (Curve2D)_circuit.Value(anitem);
            thevector = curve.DN(curve.FirstParameter, 1);
        }
        else
        {
            if (!isOpenResult)
            {
                item = (anitem == 1) ? _circuit.NumberOfItems() : (anitem - 1);
            }
            else
            {
                item = (anitem == 1) ? 2 : (anitem - 1);
            }
            curve = (Curve2D)_circuit.Value(item);
            double param = (isOpenResult && anitem == 1) ? curve.FirstParameter : curve.LastParameter;
            thevector = curve.DN(param, 1);
        }
        _geomVecs[_numberOfVecs] = thevector.Reversed();
        return _numberOfVecs;
    }

    /// <summary>Creates the tangent at the end of the bisector; returns its index.</summary>
    public int Tangent(int bisector)
    {
        _numberOfVecs++;
        _geomVecs[_numberOfVecs] = GeomBis(bisector).Value().DN(GeomBis(bisector).Value().LastParameter, 1);
        return _numberOfVecs;
    }

    /// <summary>Creates the geometric bisector of abisector.</summary>
    public void CreateBisector(MedialBisector abisector)
    {
        Point2 point1, point2;
        Geometry2D elt1, elt2;
        Bisec bisector = new();
        double tolerance = MAT2d_TOLCONF;

        int edge1number = abisector.FirstEdge().EdgeNumber();
        int edge2number = abisector.SecondEdge().EdgeNumber();
        bool ontheline = AreNeighbours(edge1number, edge2number, NumberOfItems());
        bool initialNeighbour = ontheline;

        if (_circuit!.ConnexionOn(edge2number))
        {
            ontheline = false;
        }

        elt1 = _circuit.Value(edge1number);
        elt2 = _circuit.Value(edge2number);

        Type type1 = _circuit.Value(edge1number).GetType();
        Type type2 = _circuit.Value(edge2number).GetType();
        Curve2D? item1 = null;
        Curve2D? item2 = null;

        if (type1 != typeof(CartesianPoint2))
        {
            item1 = (Curve2D)elt1;
        }

        if (type2 != typeof(CartesianPoint2))
        {
            item2 = (Curve2D)elt2;
        }

        if (type1 != typeof(CartesianPoint2) && type2 != typeof(CartesianPoint2))
        {
            bisector.Perform(item1!, item2!, GeomPnt(abisector.IssuePoint()), GeomVec(abisector.FirstVector()),
                GeomVec(abisector.SecondVector()), _direction, _joinType, tolerance, ontheline);
        }
        else if (type1 == typeof(CartesianPoint2) && type2 == typeof(CartesianPoint2))
        {
            point1 = (Point2)elt1;
            point2 = (Point2)elt2;
            bisector.Perform(point1, point2, GeomPnt(abisector.IssuePoint()), GeomVec(abisector.FirstVector()),
                GeomVec(abisector.SecondVector()), _direction, tolerance, ontheline);
        }
        else if (type1 == typeof(CartesianPoint2))
        {
            point1 = (Point2)elt1;
            bisector.Perform(point1, item2!, GeomPnt(abisector.IssuePoint()), GeomVec(abisector.FirstVector()),
                GeomVec(abisector.SecondVector()), _direction, tolerance, ontheline);
        }
        else
        {
            point2 = (Point2)elt2;
            bisector.Perform(item1!, point2, GeomPnt(abisector.IssuePoint()), GeomVec(abisector.FirstVector()),
                GeomVec(abisector.SecondVector()), _direction, tolerance, ontheline);
        }

        // Restriction of the bisector.
        TrimBisec(bisector, edge1number, initialNeighbour, 1);
        TrimBisec(bisector, edge2number, initialNeighbour, 2);

        _numberOfBisectors++;
        _geomBisectors[_numberOfBisectors] = bisector;

        abisector.BisectorNumber(_numberOfBisectors);
        abisector.Sense(1);
    }

    // Restriction of the bisectors separating two elements linked by a connexion where at least one element is a
    // circle; needed by the logic of the algorithm.
    private void TrimBisec(Bisec b1, int indexEdge, bool initialNeighbour, int startOrEnd)
    {
        Curve2D curve;
        TrimmedCurve2D line;
        LineCurve2D? line1 = null, line2 = null;
        Point2D ori;
        int iNext;
        iNext = (indexEdge == _circuit!.NumberOfItems()) ? 1 : (indexEdge + 1);

        Type edgeType = _circuit.Value(indexEdge).GetType();

        if (edgeType != typeof(CartesianPoint2))
        {
            if (!initialNeighbour)
            {
                curve = ((TrimmedCurve2D)_circuit.Value(indexEdge)).BasisCurve;
                edgeType = curve.GetType();
                // The edge is linked to its previous neighbour by a connexion.
                if (_circuit.ConnexionOn(indexEdge) && startOrEnd == 1)
                {
                    if (edgeType == typeof(CircleCurve2D))
                    {
                        ori = ((CircleCurve2D)curve).Location;
                        Point2D p2 = _circuit.Connexion(indexEdge).PointOnFirst();
                        line1 = new LineCurve2D(ori, new Direction2D(p2.X - ori.X, p2.Y - ori.Y));
                    }
                }

                // The edge is linked to its next neighbour by a connexion.
                if (_circuit.ConnexionOn(iNext) && startOrEnd == 2)
                {
                    if (edgeType == typeof(CircleCurve2D))
                    {
                        ori = ((CircleCurve2D)curve).Location;
                        Point2D p2 = _circuit.Connexion(iNext).PointOnSecond();
                        line2 = new LineCurve2D(ori, new Direction2D(p2.X - ori.X, p2.Y - ori.Y));
                    }
                }
                if (line1 is null && line2 is null)
                {
                    return;
                }

                // Restriction of the bisector by the half-lines of the connexions.
                if (line1 is not null)
                {
                    line = new TrimmedCurve2D(line1, 0.0, global::Kerf.Engine.Foundation.Precision.Infinite());
                    SetTrim(b1, line);
                }
                if (line2 is not null)
                {
                    line = new TrimmedCurve2D(line2, 0.0, global::Kerf.Engine.Foundation.Precision.Infinite());
                    SetTrim(b1, line);
                }
            }
        }
    }

    /// <summary>Trims the geometric bisector of abisector at its first parameter; false if the parameter is off
    /// the bisector.</summary>
    public bool TrimBisector(MedialBisector abisector)
    {
        double param = abisector.FirstParameter();

        TrimmedCurve2D bisector = ChangeGeomBis(abisector.BisectorNumber()).ChangeValue();

        if (bisector.ChangeBasisCurve().IsPeriodic && param == global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            param = bisector.FirstParameter + 2 * M_PI;
        }
        if (param > bisector.ChangeBasisCurve().LastParameter)
        {
            param = bisector.ChangeBasisCurve().LastParameter;
        }
        if (bisector.FirstParameter == param)
        {
            return false;
        }

        bisector.SetTrim(bisector.FirstParameter, param);
        return true;
    }

    /// <summary>Trims the geometric bisector of abisector at the point apoint; false if the point is off the
    /// bisector.</summary>
    public bool TrimBisector(MedialBisector abisector, int apoint)
    {
        double param;
        TrimmedCurve2D bisector = ChangeGeomBis(abisector.BisectorNumber()).ChangeValue();

        BisectorCurve bis = (BisectorCurve)bisector.ChangeBasisCurve();

        param = bis.Parameter(GeomPnt(apoint));

        if (bisector.ChangeBasisCurve().IsPeriodic)
        {
            if (bisector.FirstParameter > param)
            {
                param = param + 2 * M_PI;
            }
        }
        if (bisector.FirstParameter >= param)
        {
            return false;
        }
        if (bisector.LastParameter < param)
        {
            return false;
        }
        bisector.SetTrim(bisector.FirstParameter, param);

        return true;
    }

    // True if PCom can be projected on the element IEdge; Distance is then the minimum distance to its
    // projections.
    private bool Projection(int iEdge, in Point2D pCom, out double distance)
    {
        Point2D pEdge;
        Geometry2D elt = _circuit!.Value(iEdge);
        Type type = elt.GetType();
        TrimmedCurve2D curve;
        int iNext;
        double eps = MAT2d_TOLCONF;

        if (type == typeof(CartesianPoint2))
        {
            pEdge = ((Point2)elt).Pnt2d();
            distance = pCom.DistanceTo(pEdge);
        }
        else
        {
            distance = global::Kerf.Engine.Foundation.Precision.Infinite();
            curve = (TrimmedCurve2D)elt;
            // The parameters on the edge: if it is linked to its neighbours by connexions, the curve is limited by
            // them.
            double paramMin = curve.FirstParameter;
            double paramMax = curve.LastParameter;
            if (_circuit.ConnexionOn(iEdge))
            {
                paramMin = _circuit.Connexion(iEdge).ParameterOnSecond();
            }
            iNext = (iEdge == _circuit.NumberOfItems()) ? 1 : (iEdge + 1);
            if (_circuit.ConnexionOn(iNext))
            {
                paramMax = _circuit.Connexion(iNext).ParameterOnFirst();
                if (curve.BasisCurve.IsPeriodic)
                {
                    global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2 * M_PI, eps, ref paramMin, ref paramMax);
                }
            }
            // The curve for the extrema, and its bounds.
            CurveAdaptor2D c1 = new(curve);
            CurveKind typeC1 = c1.GetType();
            if (typeC1 == Circle)
            {
                double r = c1.Circle().Radius;
                double epsCirc = 100.0 * eps;
                if (r < 1.0)
                {
                    epsCirc = eps / r;
                }
                if (((paramMax - paramMin + 2 * epsCirc) < 2 * M_PI))
                {
                    paramMax = paramMax + epsCirc;
                    paramMin = paramMin - epsCirc;
                }
            }
            else
            {
                paramMax = paramMax + eps;
                paramMin = paramMin - eps;
            }
            // The extrema, keeping the minimum of distance.
            ExtPC2d extremas = new(pCom, c1, paramMin, paramMax);
            if (extremas.IsDone)
            {
                distance = global::Kerf.Engine.Foundation.Precision.Infinite();
                if (extremas.ExtCount < 1)
                {
                    return false;
                }
                for (int i = 1; i <= extremas.ExtCount; i++)
                {
                    if (extremas.SquareDistance(i) < distance)
                    {
                        distance = extremas.SquareDistance(i);
                    }
                }
                distance = Math.Sqrt(distance);
            }
            else
            {
                if (typeC1 == Circle)
                {
                    distance = c1.Circle().Radius;
                }
            }
        }
        return true;
    }

    // True if PCom is equidistant from the elements separated by BisectorOne and BisectorTwo; Distance is then its
    // distance to them.
    private bool IsSameDistance(MedialBisector bisectorOne, MedialBisector bisectorTwo, in Point2D pCom,
        out double distance)
    {
        NCollection_Array1<double> dist = new(1, 4);
        const double eps = 1.0e-7;
        int iEdge1, iEdge2, iEdge3, iEdge4;

        iEdge1 = bisectorOne.FirstEdge().EdgeNumber();
        iEdge2 = bisectorOne.SecondEdge().EdgeNumber();
        iEdge3 = bisectorTwo.FirstEdge().EdgeNumber();
        iEdge4 = bisectorTwo.SecondEdge().EdgeNumber();

        bool isDone1 = Projection(iEdge1, pCom, out dist[1]);
        bool isDone2 = Projection(iEdge2, pCom, out dist[2]);

        if (isDone1)
        {
            if (!isDone2)
            {
                Geometry2D elt = _circuit!.Value(iEdge2);
                double tol = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), eps * dist[1]);
                if (CheckEnds(elt, pCom, dist[1], tol))
                {
                    dist[2] = dist[1];
                }
            }
        }
        else
        {
            if (isDone2)
            {
                Geometry2D elt = _circuit!.Value(iEdge1);
                double tol = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), eps * dist[2]);
                if (CheckEnds(elt, pCom, dist[2], tol))
                {
                    dist[1] = dist[2];
                }
            }
        }

        bool isDone3 = true, isDone4 = true;
        if (iEdge3 == iEdge1)
        {
            dist[3] = dist[1];
        }
        else if (iEdge3 == iEdge2)
        {
            dist[3] = dist[2];
        }
        else
        {
            isDone3 = Projection(iEdge3, pCom, out dist[3]);
        }

        if (iEdge4 == iEdge1)
        {
            dist[4] = dist[1];
        }
        else if (iEdge4 == iEdge2)
        {
            dist[4] = dist[2];
        }
        else
        {
            isDone4 = Projection(iEdge4, pCom, out dist[4]);
        }

        if (isDone3)
        {
            if (!isDone4)
            {
                Geometry2D elt = _circuit!.Value(iEdge4);
                double tol = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), eps * dist[3]);
                if (CheckEnds(elt, pCom, dist[3], tol))
                {
                    dist[4] = dist[3];
                }
            }
        }
        else
        {
            if (isDone4)
            {
                Geometry2D elt = _circuit!.Value(iEdge3);
                double tol = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), eps * dist[4]);
                if (CheckEnds(elt, pCom, dist[4], tol))
                {
                    dist[3] = dist[4];
                }
            }
        }

        double epsDist = MAT2d_TOLCONF * 300.0;
        distance = dist[1];
        if (_joinType == global::Kerf.Geometry.JoinKind.Intersection && global::Kerf.Engine.Foundation.Precision.IsInfinite(distance))
        {
            for (int i = 2; i <= 4; i++)
            {
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(dist[i]))
                {
                    distance = dist[i];
                    break;
                }
            }
        }
        for (int i = 1; i <= 4; i++)
        {
            if (_joinType == global::Kerf.Geometry.JoinKind.Intersection && global::Kerf.Engine.Foundation.Precision.IsInfinite(dist[i]))
            {
                continue;
            }

            if (Math.Abs(dist[i] - distance) > epsDist)
            {
                distance = global::Kerf.Engine.Foundation.Precision.Infinite();
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Intersects the bisectors of bisectorone and bisectortwo. If a point is found, intpnt is its index and the
    /// result its distance to the elements; else RealLast (Precision::Infinite()).
    /// </summary>
    public double IntersectBisector(MedialBisector bisectorOne, MedialBisector bisectorTwo, ref int intPnt)
    {
        double tolerance = MAT2d_TOLCONF;
        double param1, param2;
        double parama, paramb;
        double distance = 0.0, distanceMini;
        bool solutionValide;
        Point2D pointSolution = new();

        TrimmedCurve2D? bisector1 = ChangeGeomBis(bisectorOne.BisectorNumber()).ChangeValue();

        TrimmedCurve2D? bisector2 = ChangeGeomBis(bisectorTwo.BisectorNumber()).ChangeValue();

        if (bisector1 is null || bisector2 is null)
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        // If the two bisectors separate consecutive elements and start from connexions C1 and C2, and C1 is the
        // reverse of C2, the two bisectors start from the same point: the intersection is not valid.
        int is1 = bisectorOne.SecondEdge().EdgeNumber();
        int is2 = bisectorTwo.SecondEdge().EdgeNumber();
        int if1 = bisectorOne.FirstEdge().EdgeNumber();
        int if2 = bisectorTwo.FirstEdge().EdgeNumber();

        if (AreNeighbours(if1, is1, NumberOfItems()) && AreNeighbours(if2, is2, NumberOfItems())
            && _circuit!.ConnexionOn(is2) && _circuit.ConnexionOn(is1))
        {
            Connexion c1, c2;
            c1 = _circuit.Connexion(is1);
            c2 = _circuit.Connexion(is2);
            if (c2.IndexFirstLine() == c1.IndexSecondLine() && c1.IndexFirstLine() == c2.IndexSecondLine())
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        // The domains of intersection.
        TwoDDomain domain1 = Domain(bisector1, tolerance);
        TwoDDomain domain2 = Domain(bisector2, tolerance);

        if (domain1.LastParameter() - domain1.FirstParameter() < tolerance)
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }
        if (domain2.LastParameter() - domain2.FirstParameter() < tolerance)
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        // The intersection.
        Inter intersect = new();
        intersect.Perform(GeomBis(bisectorOne.BisectorNumber()), domain1, GeomBis(bisectorTwo.BisectorNumber()),
            domain2, tolerance, tolerance, true);

        // The solution: the point equidistant from the two edges nearest in parameter to the origin of the
        // bisectors.
        if (!intersect.IsDone)
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        if (intersect.IsEmpty)
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        distanceMini = global::Kerf.Engine.Foundation.Precision.Infinite();
        param1 = global::Kerf.Engine.Foundation.Precision.Infinite();
        param2 = global::Kerf.Engine.Foundation.Precision.Infinite();
        solutionValide = false;

        if (intersect.SegmentCount >= 1)
        {
            double maxSegmentLength = 10.0 * tolerance;
            for (int i = 1; i <= intersect.SegmentCount; i++)
            {
                IntersectionSegment2D segment = intersect.Segment(i);
                bool pointRetenu = false;
                Point2D pointOnSegment = new();
                // If the segments are small, the points on the segment equidistant from the edges.
                if ((segment.HasFirstPoint && segment.HasLastPoint))
                {
                    Point2D p1, p2;
                    double segmentLength;
                    p1 = segment.FirstPoint().Value();
                    p2 = segment.LastPoint().Value();
                    segmentLength = p1.DistanceTo(p2);
                    if (segmentLength <= tolerance)
                    {
                        pointOnSegment = p1;
                        if (IsSameDistance(bisectorOne, bisectorTwo, pointOnSegment, out distance))
                        {
                            pointRetenu = true;
                        }
                    }
                    else if (segmentLength <= maxSegmentLength)
                    {
                        Direction2D dir = new(p2.X - p1.X, p2.Y - p1.Y);
                        double dist = 0.0;
                        while (dist <= segmentLength + tolerance)
                        {
                            pointOnSegment = p1.Translated(dist * new Vector2D(dir));
                            if (IsSameDistance(bisectorOne, bisectorTwo, pointOnSegment, out distance))
                            {
                                pointRetenu = true;
                                break;
                            }
                            dist = dist + tolerance;
                        }
                    }
                }

                // The equidistant point of smallest parameter on the bisectors.
                if (pointRetenu)
                {
                    parama = ((BisectorCurve)bisector1.ChangeBasisCurve()).Parameter(pointOnSegment);
                    paramb = ((BisectorCurve)bisector2.ChangeBasisCurve()).Parameter(pointOnSegment);
                    if (parama < param1 && paramb < param2)
                    {
                        param1 = parama;
                        param2 = paramb;
                        distanceMini = distance;
                        pointSolution = pointOnSegment;
                        solutionValide = true;
                    }
                }
            }
        }

        if (intersect.PointCount != 1)
        {
            for (int i = 1; i <= intersect.PointCount; i++)
            {
                if (IsSameDistance(bisectorOne, bisectorTwo, intersect.Point(i).Value(), out distance)
                    && distance > tolerance)
                {
                    parama = intersect.Point(i).ParamOnFirst();
                    paramb = intersect.Point(i).ParamOnSecond();
                    if (parama < param1 && paramb < param2)
                    {
                        param1 = parama;
                        param2 = paramb;
                        distanceMini = distance;
                        pointSolution = intersect.Point(i).Value();
                        solutionValide = true;
                    }
                }
            }
        }
        else
        {
            pointSolution = intersect.Point(1).Value();
            param1 = intersect.Point(1).ParamOnFirst();
            param2 = intersect.Point(1).ParamOnSecond();
            solutionValide = IsSameDistance(bisectorOne, bisectorTwo, pointSolution, out distanceMini);
        }

        if (!solutionValide)
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }
        _numberOfPnts++;
        _geomPnts[_numberOfPnts] = pointSolution;
        intPnt = _numberOfPnts;

        // If the intersection point is almost one of the ends of a bisector, the intersection is not valid,
        // UNLESS one of the bisectors starts from a connexion and the edges it separates are neighbours on the
        // initial contour: the middle of the connexion, the origin of one bisector, can be on the other one (a
        // rectangle in a rectangle with a connexion between a corner and a side), and the bisector from the
        // connexion has length zero.
        int indexEdge1, indexEdge2, indexEdge3, indexEdge4;
        bool extremiteControle = true;

        indexEdge1 = bisectorOne.FirstEdge().EdgeNumber();
        indexEdge2 = bisectorOne.SecondEdge().EdgeNumber();
        indexEdge3 = bisectorTwo.FirstEdge().EdgeNumber();
        indexEdge4 = bisectorTwo.SecondEdge().EdgeNumber();

        if (_circuit!.ConnexionOn(indexEdge2))
        {
            // BisectorOne starts from a connexion.
            if (AreNeighbours(indexEdge1, indexEdge2, NumberOfItems())
                && AreNeighbours(indexEdge3, indexEdge4, NumberOfItems()) && indexEdge2 == indexEdge3)
            {
                extremiteControle = false;
                param1 = param1 + tolerance;
            }
        }

        if (_circuit.ConnexionOn(indexEdge4))
        {
            // BisectorTwo starts from a connexion.
            if (AreNeighbours(indexEdge1, indexEdge2, NumberOfItems())
                && AreNeighbours(indexEdge3, indexEdge4, NumberOfItems()) && indexEdge2 == indexEdge3)
            {
                extremiteControle = false;
                param2 = param2 + tolerance;
            }
        }

        if (extremiteControle)
        {
            if (bisector1.StartPoint.DistanceTo(pointSolution) < tolerance)
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            if (bisector2.StartPoint.DistanceTo(pointSolution) < tolerance)
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (bisectorOne.SecondParameter() < global::Kerf.Engine.Foundation.Precision.Infinite()
            && bisectorOne.SecondParameter() < param1 * (1.0 - tolerance))
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        if (bisectorTwo.FirstParameter() < global::Kerf.Engine.Foundation.Precision.Infinite()
            && bisectorTwo.FirstParameter() < param2 * (1.0 - tolerance))
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        bisectorOne.SecondParameter(param1);
        bisectorTwo.FirstParameter(param2);

        return distanceMini;
    }

    /// <summary>The distance between the points of parameters param1 and param2 on the bisector of abisector.
    /// </summary>
    public double Distance(MedialBisector bis, double param1, double param2)
    {
        double dist = global::Kerf.Engine.Foundation.Precision.Infinite();

        if (param1 != global::Kerf.Engine.Foundation.Precision.Infinite() && param2 != global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            Point2D p1 = GeomBis(bis.BisectorNumber()).Value().Value(param1);
            Point2D p2 = GeomBis(bis.BisectorNumber()).Value().Value(param2);
            dist = p1.DistanceTo(p2);
        }
        return dist;
    }

    /// <summary>Throws Standard_NotImplemented (OCCT's release build: the drawing exists only with OCCT_DEBUG).
    /// </summary>
    public void Dump(int bisector, int erease)
    {
        throw new MethodNotImplementedException();
    }

    /// <summary>The bisector of index Index.</summary>
    public Bisec GeomBis(int index) => _geomBisectors.GetValue(index);

    /// <summary>The bisector of index Index (OCCT's modifiable reference: the same object here).</summary>
    public Bisec ChangeGeomBis(int index) => _geomBisectors.GetValue(index);

    /// <summary>The item of index Index of the circuit.</summary>
    public Geometry2D GeomElt(int index) => _circuit!.Value(index);

    /// <summary>The point of index Index.</summary>
    public Point2D GeomPnt(int index) => _geomPnts.GetValue(index);

    /// <summary>The vector of index Index.</summary>
    public Vector2D GeomVec(int index) => _geomVecs.GetValue(index);

    public Circuit Circuit() => _circuit!;

    // Kerf: the point, the vector and the trimmed curve of an index that may have none, for MatCycle

    internal bool TryGeomPnt(int index, out Point2D point) => _geomPnts.TryGetValue(index, out point);

    internal bool TryGeomVec(int index, out Vector2D vector) => _geomVecs.TryGetValue(index, out vector);

    internal TrimmedCurve2D? TrimmedBisector(int index) => _geomBisectors.TryGetValue(index, out Bisec? bisector) ? bisector.Value() : null;

    /// <summary>Merges the bisector I2 into the bisector I1 (I1 is extended over I2).</summary>
    public void BisecFusion(int i1, int i2)
    {
        double du, ul1, uf1;
        TrimmedCurve2D bisector1;
        TrimmedCurve2D bisector2;

        bisector1 = GeomBis(i1).Value();
        bisector2 = GeomBis(i2).Value();
        uf1 = bisector1.FirstParameter;
        ul1 = bisector1.LastParameter;

        Type type1 = bisector1.ChangeBasisCurve().GetType();
        if (type1 == typeof(BisecCC))
        {
            // The curve/curve bisectors are built from a start point and can't be trimmed by a point on the other
            // side of it: the bisector of the two curves is rebuilt from the last point of Bisector2, then trimmed
            // by the last point of Bisector1.
            double tolerance = MAT2d_TOLCONF;

            Bisec bis = new();
            Vector2D vBid = new(1, 0);
            Point2D p2 = bisector2.Value(bisector2.LastParameter);
            Point2D p1 = bisector1.Value(bisector1.LastParameter);
            BisecCC bcc1 = (BisecCC)bisector1.ChangeBasisCurve();

            bis.Perform(bcc1.Curve(2), bcc1.Curve(1), p2, vBid, vBid, _direction, _joinType, tolerance, false);

            bisector1 = bis.Value();
            bcc1 = (BisecCC)bisector1.ChangeBasisCurve();
            uf1 = bcc1.FirstParameter;
            ul1 = bcc1.Parameter(p1);
            bisector1.SetTrim(uf1, ul1);
            _geomBisectors[i1] = bis;
        }
        else
        {
            du = bisector2.LastParameter - bisector2.FirstParameter;
            uf1 = uf1 - du;

            // OCCT assumes a bisector that isn't curve/curve is analytic: a Bisector_BisecPC gives a null handle,
            // dereferenced (an access violation natively, a NullReferenceException here).
            BisecAna? bAna = bisector1.ChangeBasisCurve() as BisecAna;
            bAna!.SetTrim(uf1, ul1);
            bisector1.SetTrim(uf1, ul1);
        }
    }

    // True if IEdge1 and IEdge2 are consecutive elements of a closed contour of NbEdge elements.
    private static bool AreNeighbours(int iEdge1, int iEdge2, int nbEdge)
    {
        if (Math.Abs(iEdge1 - iEdge2) == 1)
        {
            return true;
        }
        else if (Math.Abs(iEdge1 - iEdge2) == nbEdge - 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    private static void SetTrim(Bisec bis, Curve2D line1)
    {
        GInter intersect = new();
        double distance;
        double tolerance = MAT2d_TOLCONF;
        TrimmedCurve2D bisector = bis.ChangeValue();

        TwoDDomain domain1 = Domain(bisector, tolerance);
        double ub1 = bisector.FirstParameter;
        double ub2 = bisector.LastParameter;

        Point2D firstPointBisector = bisector.Value(ub1);
        double uTrim = global::Kerf.Engine.Foundation.Precision.Infinite();

        CurveAdaptor2D adapBisector = new(bisector);
        CurveAdaptor2D adapLine1 = new(line1);
        intersect.Perform(adapBisector, domain1, adapLine1, tolerance, tolerance);

        if (intersect.IsDone && !intersect.IsEmpty)
        {
            for (int i = 1; i <= intersect.PointCount; i++)
            {
                Point2D pInt = intersect.Point(i).Value();
                distance = firstPointBisector.DistanceTo(pInt);
                if (distance > 10.0 * tolerance && intersect.Point(i).ParamOnFirst() < uTrim)
                {
                    uTrim = intersect.Point(i).ParamOnFirst();
                }
            }
        }
        // Restriction of the bisector by the intersection point of smallest parameter.
        if (uTrim < ub2 && uTrim > ub1)
        {
            bisector.SetTrim(ub1, uTrim);
        }
    }

    // OCCT's static Type(): the type of a geometry, or of the basis of a trimmed curve.
    private static Type TypeOf(Geometry2D aGeom)
    {
        Type type = aGeom.GetType();
        Curve2D curve;

        if (type == typeof(TrimmedCurve2D))
        {
            curve = ((TrimmedCurve2D)aGeom).ChangeBasisCurve();
            type = curve.GetType();
        }
        return type;
    }

    private static TwoDDomain Domain(TrimmedCurve2D bisector1, double tolerance)
    {
        double param1 = bisector1.FirstParameter;
        double param2 = bisector1.LastParameter;
        if (param2 > 10000.0)
        {
            param2 = 10000.0;
            Type type1 = TypeOf(bisector1.ChangeBasisCurve());
            Curve2D? basisCurve = null;
            if (type1 == typeof(BisecAna))
            {
                basisCurve = ((BisecAna)bisector1.ChangeBasisCurve()).Geom2dCurve();
                type1 = basisCurve.GetType();
            }
            Parabola2D gpParabola;
            Hyperbola2D gpHyperbola;
            double focus;
            double limit = 50000.0;
            if (type1 == typeof(ParabolaCurve2D))
            {
                gpParabola = ((ParabolaCurve2D)basisCurve!).Parabola;
                focus = gpParabola.Focal;
                double val1 = Math.Sqrt(limit * focus);
                double val2 = Math.Sqrt(limit * limit);
                param2 = (val1 <= val2 ? val1 : val2);
            }
            else if (type1 == typeof(HyperbolaCurve2D))
            {
                gpHyperbola = ((HyperbolaCurve2D)basisCurve!).Hyperbola;
                double majr = gpHyperbola.MajorRadius;
                double minr = gpHyperbola.MinorRadius;
                double valu1 = limit / majr;
                double valu2 = limit / minr;
                double val1 = Math.Log(valu1 + Math.Sqrt(valu1 * valu1 - 1));
                double val2 = Math.Log(valu2 + Math.Sqrt(valu2 * valu2 + 1));
                param2 = (val1 <= val2 ? val1 : val2);
            }
        }

        TwoDDomain domain1 = new(bisector1.Value(param1), param1, tolerance, bisector1.Value(param2), param2,
            tolerance);
        if (bisector1.ChangeBasisCurve().IsPeriodic)
        {
            domain1.SetEquivalentParameters(0.0, 2.0 * M_PI);
        }
        return domain1;
    }

    private static bool CheckEnds(Geometry2D elt, in Point2D pCom, double distance, double tol)
    {
        Type type = elt.GetType();
        TrimmedCurve2D curve;

        if (type == typeof(CartesianPoint2))
        {
            return false;
        }
        else
        {
            curve = (TrimmedCurve2D)elt;
            Point2D pf = curve.StartPoint;
            Point2D pl = curve.EndPoint;
            double df = pCom.DistanceTo(pf);
            double dl = pCom.DistanceTo(pl);
            if (Math.Abs(df - distance) <= tol)
            {
                return true;
            }
            if (Math.Abs(dl - distance) <= tol)
            {
                return true;
            }
        }
        return false;
    }
}
