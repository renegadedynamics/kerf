// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Tool2d.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT2d/MAT2d_Tool2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;
using static Occt.MAT_Side;

namespace Occt;

/// <summary>The geometry of the bisecting locus for MAT2d_Mat2d: the items of the circuit, the bisectors (as
/// Bisector_Bisec), the points and vectors, each indexed from 1.</summary>
/// <remarks>
/// A working object of MAT2d_Mat2d and BRepMAT2d_BisectingLocus (decision 21). The bisectors' curves are trimmed in
/// place, as OCCT does: their basis curves are read with <c>ChangeBasisCurve</c>, which doesn't freeze them. The
/// maps are keyed by integers and only looked up. Dump throws Standard_NotImplemented, as OCCT's release build.
/// </remarks>
public sealed class MAT2d_Tool2d
{
    // OCCT's static double, never changed.
    private const double MAT2d_TOLCONF = 1.0e-7;

    private double theDirection;
    private JoinKind theJoinType;
    private int theNumberOfBisectors;
    private int theNumberOfPnts;
    private int theNumberOfVecs;
    private MAT2d_Circuit? theCircuit;
    private readonly NCollection_DataMap<int, Bisector_Bisec> theGeomBisectors = new();
    private readonly NCollection_DataMap<int, Point2D> theGeomPnts = new();
    private readonly NCollection_DataMap<int, Vector2D> theGeomVecs = new();
    private readonly NCollection_Sequence<int> theLinesLength = new();

    public MAT2d_Tool2d()
    {
        theDirection = 1.0;
        theJoinType = Arc; // default
        theNumberOfBisectors = 0;
        theNumberOfVecs = 0;
        theNumberOfPnts = 0;
    }

    /// <summary>Starts from the items of the circuit EquiCircuit (the geometry of MAT's basic elements).</summary>
    public void InitItems(MAT2d_Circuit EquiCircuit)
    {
        theGeomBisectors.Clear();
        theGeomPnts.Clear();
        theGeomVecs.Clear();
        theLinesLength.Clear();
        theNumberOfBisectors = 0;
        theNumberOfVecs = 0;
        theNumberOfPnts = 0;

        theCircuit = EquiCircuit;
    }

    /// <summary>The side of the computation of the map.</summary>
    public void Sense(MAT_Side aside)
    {
        if (aside == MAT_Left)
        {
            theDirection = 1.0;
        }
        else
        {
            theDirection = -1.0;
        }
    }

    public void SetJoinType(JoinKind aJoinType) => theJoinType = aJoinType;

    /// <summary>The number of items.</summary>
    public int NumberOfItems() => theCircuit!.NumberOfItems();

    /// <summary>The tolerance of confusion of two points.</summary>
    public double ToleranceOfConfusion() => 2 * MAT2d_TOLCONF;

    /// <summary>Creates the point at the origin of the bisector of anitem and the previous item; dist is its
    /// distance to anitem. Returns its index.</summary>
    public int FirstPoint(int anitem, out double dist)
    {
        Curve2D curve;
        Point2 point;
        theNumberOfPnts++;

        if (theCircuit!.ConnexionOn(anitem))
        {
            Point2D P1 = theCircuit.Connexion(anitem).PointOnFirst();
            Point2D P2 = theCircuit.Connexion(anitem).PointOnSecond();
            theGeomPnts.Bind(theNumberOfPnts, new Point2D((P1.X() + P2.X()) * 0.5, (P1.Y() + P2.Y()) * 0.5));
            dist = P1.Distance(P2) * 0.5;
            return theNumberOfPnts;
        }

        Type type = theCircuit.Value(anitem).GetType();
        dist = 0.0;

        if (type != typeof(CartesianPoint2))
        {
            curve = (Curve2D)theCircuit.Value(anitem);
            theGeomPnts.Bind(theNumberOfPnts, curve.Value(curve.FirstParameter()));
        }
        else
        {
            point = (Point2)theCircuit.Value(anitem);
            theGeomPnts.Bind(theNumberOfPnts, point.Pnt2d());
        }
        return theNumberOfPnts;
    }

    /// <summary>Creates the tangent at the end of the item anitem; returns its index.</summary>
    public int TangentBefore(int anitem, bool IsOpenResult)
    {
        int item;
        Curve2D curve;
        theNumberOfVecs++;

        if (!IsOpenResult)
        {
            item = (anitem == theCircuit!.NumberOfItems()) ? 1 : (anitem + 1);
        }
        else
        {
            item = (anitem == theCircuit!.NumberOfItems()) ? (anitem - 1) : (anitem + 1);
        }
        if (theCircuit.ConnexionOn(item))
        {
            theCircuit.Connexion(item).PointOnFirst().Coord(out double x1, out double y1);
            theCircuit.Connexion(item).PointOnSecond().Coord(out double x2, out double y2);
            theGeomVecs.Bind(theNumberOfVecs, new Vector2D((x2 - x1), (y2 - y1)));
            return theNumberOfVecs;
        }

        Type type = theCircuit.Value(anitem).GetType();
        if (type != typeof(CartesianPoint2))
        {
            curve = (Curve2D)theCircuit.Value(anitem);
            theGeomVecs.Bind(theNumberOfVecs, curve.DN(curve.LastParameter(), 1));
        }
        else
        {
            curve = (Curve2D)theCircuit.Value(item);
            double param = (IsOpenResult && anitem == theCircuit.NumberOfItems())
                ? curve.LastParameter()
                : curve.FirstParameter();
            theGeomVecs.Bind(theNumberOfVecs, curve.DN(param, 1));
        }

        return theNumberOfVecs;
    }

    /// <summary>Creates the reversed tangent at the origin of the item anitem; returns its index.</summary>
    public int TangentAfter(int anitem, bool IsOpenResult)
    {
        int item;
        Curve2D curve;
        Vector2D thevector;
        theNumberOfVecs++;

        if (theCircuit!.ConnexionOn(anitem))
        {
            theCircuit.Connexion(anitem).PointOnFirst().Coord(out double x1, out double y1);
            theCircuit.Connexion(anitem).PointOnSecond().Coord(out double x2, out double y2);
            theGeomVecs.Bind(theNumberOfVecs, new Vector2D((x1 - x2), (y1 - y2)));
            return theNumberOfVecs;
        }

        Type type = theCircuit.Value(anitem).GetType();
        if (type != typeof(CartesianPoint2))
        {
            curve = (Curve2D)theCircuit.Value(anitem);
            thevector = curve.DN(curve.FirstParameter(), 1);
        }
        else
        {
            if (!IsOpenResult)
            {
                item = (anitem == 1) ? theCircuit.NumberOfItems() : (anitem - 1);
            }
            else
            {
                item = (anitem == 1) ? 2 : (anitem - 1);
            }
            curve = (Curve2D)theCircuit.Value(item);
            double param = (IsOpenResult && anitem == 1) ? curve.FirstParameter() : curve.LastParameter();
            thevector = curve.DN(param, 1);
        }
        theGeomVecs.Bind(theNumberOfVecs, thevector.Reversed());
        return theNumberOfVecs;
    }

    /// <summary>Creates the tangent at the end of the bisector; returns its index.</summary>
    public int Tangent(int bisector)
    {
        theNumberOfVecs++;
        theGeomVecs.Bind(theNumberOfVecs, GeomBis(bisector).Value().DN(GeomBis(bisector).Value().LastParameter(), 1));
        return theNumberOfVecs;
    }

    /// <summary>Creates the geometric bisector of abisector.</summary>
    public void CreateBisector(MAT_Bisector abisector)
    {
        Point2 point1, point2;
        Geometry2D elt1, elt2;
        Bisector_Bisec bisector = new();
        double tolerance = MAT2d_TOLCONF;

        int edge1number = abisector.FirstEdge().EdgeNumber();
        int edge2number = abisector.SecondEdge().EdgeNumber();
        bool ontheline = AreNeighbours(edge1number, edge2number, NumberOfItems());
        bool InitialNeighbour = ontheline;

        if (theCircuit!.ConnexionOn(edge2number))
        {
            ontheline = false;
        }

        elt1 = theCircuit.Value(edge1number);
        elt2 = theCircuit.Value(edge2number);

        Type type1 = theCircuit.Value(edge1number).GetType();
        Type type2 = theCircuit.Value(edge2number).GetType();
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
                GeomVec(abisector.SecondVector()), theDirection, theJoinType, tolerance, ontheline);
        }
        else if (type1 == typeof(CartesianPoint2) && type2 == typeof(CartesianPoint2))
        {
            point1 = (Point2)elt1;
            point2 = (Point2)elt2;
            bisector.Perform(point1, point2, GeomPnt(abisector.IssuePoint()), GeomVec(abisector.FirstVector()),
                GeomVec(abisector.SecondVector()), theDirection, tolerance, ontheline);
        }
        else if (type1 == typeof(CartesianPoint2))
        {
            point1 = (Point2)elt1;
            bisector.Perform(point1, item2!, GeomPnt(abisector.IssuePoint()), GeomVec(abisector.FirstVector()),
                GeomVec(abisector.SecondVector()), theDirection, tolerance, ontheline);
        }
        else
        {
            point2 = (Point2)elt2;
            bisector.Perform(item1!, point2, GeomPnt(abisector.IssuePoint()), GeomVec(abisector.FirstVector()),
                GeomVec(abisector.SecondVector()), theDirection, tolerance, ontheline);
        }

        // Restriction of the bisector.
        TrimBisec(bisector, edge1number, InitialNeighbour, 1);
        TrimBisec(bisector, edge2number, InitialNeighbour, 2);

        theNumberOfBisectors++;
        theGeomBisectors.Bind(theNumberOfBisectors, bisector);

        abisector.BisectorNumber(theNumberOfBisectors);
        abisector.Sense(1);
    }

    // Restriction of the bisectors separating two elements linked by a connexion where at least one element is a
    // circle; needed by the logic of the algorithm.
    private void TrimBisec(Bisector_Bisec B1, int IndexEdge, bool InitialNeighbour, int StartOrEnd)
    {
        Curve2D Curve;
        TrimmedCurve2D Line;
        LineCurve2D? Line1 = null, Line2 = null;
        Point2D Ori;
        int INext;
        INext = (IndexEdge == theCircuit!.NumberOfItems()) ? 1 : (IndexEdge + 1);

        Type EdgeType = theCircuit.Value(IndexEdge).GetType();

        if (EdgeType != typeof(CartesianPoint2))
        {
            if (!InitialNeighbour)
            {
                Curve = ((TrimmedCurve2D)theCircuit.Value(IndexEdge)).BasisCurve();
                EdgeType = Curve.GetType();
                // The edge is linked to its previous neighbour by a connexion.
                if (theCircuit.ConnexionOn(IndexEdge) && StartOrEnd == 1)
                {
                    if (EdgeType == typeof(CircleCurve2D))
                    {
                        Ori = ((CircleCurve2D)Curve).Location();
                        Point2D P2 = theCircuit.Connexion(IndexEdge).PointOnFirst();
                        Line1 = new LineCurve2D(Ori, new Direction2D(P2.X() - Ori.X(), P2.Y() - Ori.Y()));
                    }
                }

                // The edge is linked to its next neighbour by a connexion.
                if (theCircuit.ConnexionOn(INext) && StartOrEnd == 2)
                {
                    if (EdgeType == typeof(CircleCurve2D))
                    {
                        Ori = ((CircleCurve2D)Curve).Location();
                        Point2D P2 = theCircuit.Connexion(INext).PointOnSecond();
                        Line2 = new LineCurve2D(Ori, new Direction2D(P2.X() - Ori.X(), P2.Y() - Ori.Y()));
                    }
                }
                if (Line1 is null && Line2 is null)
                {
                    return;
                }

                // Restriction of the bisector by the half-lines of the connexions.
                if (Line1 is not null)
                {
                    Line = new TrimmedCurve2D(Line1, 0.0, global::Kerf.Engine.Foundation.Precision.Infinite());
                    SetTrim(B1, Line);
                }
                if (Line2 is not null)
                {
                    Line = new TrimmedCurve2D(Line2, 0.0, global::Kerf.Engine.Foundation.Precision.Infinite());
                    SetTrim(B1, Line);
                }
            }
        }
    }

    /// <summary>Trims the geometric bisector of abisector at its first parameter; false if the parameter is off
    /// the bisector.</summary>
    public bool TrimBisector(MAT_Bisector abisector)
    {
        double param = abisector.FirstParameter();

        TrimmedCurve2D bisector = ChangeGeomBis(abisector.BisectorNumber()).ChangeValue();

        if (bisector.ChangeBasisCurve().IsPeriodic && param == global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            param = bisector.FirstParameter() + 2 * M_PI;
        }
        if (param > bisector.ChangeBasisCurve().LastParameter())
        {
            param = bisector.ChangeBasisCurve().LastParameter();
        }
        if (bisector.FirstParameter() == param)
        {
            return false;
        }

        bisector.SetTrim(bisector.FirstParameter(), param);
        return true;
    }

    /// <summary>Trims the geometric bisector of abisector at the point apoint; false if the point is off the
    /// bisector.</summary>
    public bool TrimBisector(MAT_Bisector abisector, int apoint)
    {
        double Param;
        TrimmedCurve2D Bisector = ChangeGeomBis(abisector.BisectorNumber()).ChangeValue();

        Bisector_Curve Bis = (Bisector_Curve)Bisector.ChangeBasisCurve();

        Param = Bis.Parameter(GeomPnt(apoint));

        if (Bisector.ChangeBasisCurve().IsPeriodic)
        {
            if (Bisector.FirstParameter() > Param)
            {
                Param = Param + 2 * M_PI;
            }
        }
        if (Bisector.FirstParameter() >= Param)
        {
            return false;
        }
        if (Bisector.LastParameter() < Param)
        {
            return false;
        }
        Bisector.SetTrim(Bisector.FirstParameter(), Param);

        return true;
    }

    // True if PCom can be projected on the element IEdge; Distance is then the minimum distance to its
    // projections.
    private bool Projection(int IEdge, in Point2D PCom, out double Distance)
    {
        Point2D PEdge;
        Geometry2D Elt = theCircuit!.Value(IEdge);
        Type Type = Elt.GetType();
        TrimmedCurve2D Curve;
        int INext;
        double Eps = MAT2d_TOLCONF;

        if (Type == typeof(CartesianPoint2))
        {
            PEdge = ((Point2)Elt).Pnt2d();
            Distance = PCom.Distance(PEdge);
        }
        else
        {
            Distance = global::Kerf.Engine.Foundation.Precision.Infinite();
            Curve = (TrimmedCurve2D)Elt;
            // The parameters on the edge: if it is linked to its neighbours by connexions, the curve is limited by
            // them.
            double ParamMin = Curve.FirstParameter();
            double ParamMax = Curve.LastParameter();
            if (theCircuit.ConnexionOn(IEdge))
            {
                ParamMin = theCircuit.Connexion(IEdge).ParameterOnSecond();
            }
            INext = (IEdge == theCircuit.NumberOfItems()) ? 1 : (IEdge + 1);
            if (theCircuit.ConnexionOn(INext))
            {
                ParamMax = theCircuit.Connexion(INext).ParameterOnFirst();
                if (Curve.BasisCurve().IsPeriodic)
                {
                    global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2 * M_PI, Eps, ref ParamMin, ref ParamMax);
                }
            }
            // The curve for the extrema, and its bounds.
            CurveAdaptor2D C1 = new(Curve);
            CurveKind TypeC1 = C1.GetType();
            if (TypeC1 == Circle)
            {
                double R = C1.Circle().Radius();
                double EpsCirc = 100.0 * Eps;
                if (R < 1.0)
                {
                    EpsCirc = Eps / R;
                }
                if (((ParamMax - ParamMin + 2 * EpsCirc) < 2 * M_PI))
                {
                    ParamMax = ParamMax + EpsCirc;
                    ParamMin = ParamMin - EpsCirc;
                }
            }
            else
            {
                ParamMax = ParamMax + Eps;
                ParamMin = ParamMin - Eps;
            }
            // The extrema, keeping the minimum of distance.
            ExtPC2d Extremas = new(PCom, C1, ParamMin, ParamMax);
            if (Extremas.IsDone)
            {
                Distance = global::Kerf.Engine.Foundation.Precision.Infinite();
                if (Extremas.ExtCount < 1)
                {
                    return false;
                }
                for (int i = 1; i <= Extremas.ExtCount; i++)
                {
                    if (Extremas.SquareDistance(i) < Distance)
                    {
                        Distance = Extremas.SquareDistance(i);
                    }
                }
                Distance = Math.Sqrt(Distance);
            }
            else
            {
                if (TypeC1 == Circle)
                {
                    Distance = C1.Circle().Radius();
                }
            }
        }
        return true;
    }

    // True if PCom is equidistant from the elements separated by BisectorOne and BisectorTwo; Distance is then its
    // distance to them.
    private bool IsSameDistance(MAT_Bisector BisectorOne, MAT_Bisector BisectorTwo, in Point2D PCom,
        out double Distance)
    {
        NCollection_Array1<double> Dist = new(1, 4);
        const double eps = 1.0e-7;
        int IEdge1, IEdge2, IEdge3, IEdge4;

        IEdge1 = BisectorOne.FirstEdge().EdgeNumber();
        IEdge2 = BisectorOne.SecondEdge().EdgeNumber();
        IEdge3 = BisectorTwo.FirstEdge().EdgeNumber();
        IEdge4 = BisectorTwo.SecondEdge().EdgeNumber();

        bool isDone1 = Projection(IEdge1, PCom, out Dist[1]);
        bool isDone2 = Projection(IEdge2, PCom, out Dist[2]);

        if (isDone1)
        {
            if (!isDone2)
            {
                Geometry2D Elt = theCircuit!.Value(IEdge2);
                double Tol = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), eps * Dist[1]);
                if (CheckEnds(Elt, PCom, Dist[1], Tol))
                {
                    Dist[2] = Dist[1];
                }
            }
        }
        else
        {
            if (isDone2)
            {
                Geometry2D Elt = theCircuit!.Value(IEdge1);
                double Tol = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), eps * Dist[2]);
                if (CheckEnds(Elt, PCom, Dist[2], Tol))
                {
                    Dist[1] = Dist[2];
                }
            }
        }

        bool isDone3 = true, isDone4 = true;
        if (IEdge3 == IEdge1)
        {
            Dist[3] = Dist[1];
        }
        else if (IEdge3 == IEdge2)
        {
            Dist[3] = Dist[2];
        }
        else
        {
            isDone3 = Projection(IEdge3, PCom, out Dist[3]);
        }

        if (IEdge4 == IEdge1)
        {
            Dist[4] = Dist[1];
        }
        else if (IEdge4 == IEdge2)
        {
            Dist[4] = Dist[2];
        }
        else
        {
            isDone4 = Projection(IEdge4, PCom, out Dist[4]);
        }

        if (isDone3)
        {
            if (!isDone4)
            {
                Geometry2D Elt = theCircuit!.Value(IEdge4);
                double Tol = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), eps * Dist[3]);
                if (CheckEnds(Elt, PCom, Dist[3], Tol))
                {
                    Dist[4] = Dist[3];
                }
            }
        }
        else
        {
            if (isDone4)
            {
                Geometry2D Elt = theCircuit!.Value(IEdge3);
                double Tol = Max(global::Kerf.Engine.Foundation.Precision.Confusion(), eps * Dist[4]);
                if (CheckEnds(Elt, PCom, Dist[4], Tol))
                {
                    Dist[3] = Dist[4];
                }
            }
        }

        double EpsDist = MAT2d_TOLCONF * 300.0;
        Distance = Dist[1];
        if (theJoinType == Intersection && global::Kerf.Engine.Foundation.Precision.IsInfinite(Distance))
        {
            for (int i = 2; i <= 4; i++)
            {
                if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(Dist[i]))
                {
                    Distance = Dist[i];
                    break;
                }
            }
        }
        for (int i = 1; i <= 4; i++)
        {
            if (theJoinType == Intersection && global::Kerf.Engine.Foundation.Precision.IsInfinite(Dist[i]))
            {
                continue;
            }

            if (Math.Abs(Dist[i] - Distance) > EpsDist)
            {
                Distance = global::Kerf.Engine.Foundation.Precision.Infinite();
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Intersects the bisectors of bisectorone and bisectortwo. If a point is found, intpnt is its index and the
    /// result its distance to the elements; else RealLast (Precision::Infinite()).
    /// </summary>
    public double IntersectBisector(MAT_Bisector BisectorOne, MAT_Bisector BisectorTwo, ref int IntPnt)
    {
        double Tolerance = MAT2d_TOLCONF;
        double Param1, Param2;
        double Parama, Paramb;
        double Distance = 0.0, DistanceMini;
        bool SolutionValide;
        Point2D PointSolution = new();

        TrimmedCurve2D? Bisector1 = ChangeGeomBis(BisectorOne.BisectorNumber()).ChangeValue();

        TrimmedCurve2D? Bisector2 = ChangeGeomBis(BisectorTwo.BisectorNumber()).ChangeValue();

        if (Bisector1 is null || Bisector2 is null)
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        // If the two bisectors separate consecutive elements and start from connexions C1 and C2, and C1 is the
        // reverse of C2, the two bisectors start from the same point: the intersection is not valid.
        int IS1 = BisectorOne.SecondEdge().EdgeNumber();
        int IS2 = BisectorTwo.SecondEdge().EdgeNumber();
        int IF1 = BisectorOne.FirstEdge().EdgeNumber();
        int IF2 = BisectorTwo.FirstEdge().EdgeNumber();

        if (AreNeighbours(IF1, IS1, NumberOfItems()) && AreNeighbours(IF2, IS2, NumberOfItems())
            && theCircuit!.ConnexionOn(IS2) && theCircuit.ConnexionOn(IS1))
        {
            MAT2d_Connexion C1, C2;
            C1 = theCircuit.Connexion(IS1);
            C2 = theCircuit.Connexion(IS2);
            if (C2.IndexFirstLine() == C1.IndexSecondLine() && C1.IndexFirstLine() == C2.IndexSecondLine())
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        // The domains of intersection.
        IntRes2d_Domain Domain1 = Domain(Bisector1, Tolerance);
        IntRes2d_Domain Domain2 = Domain(Bisector2, Tolerance);

        if (Domain1.LastParameter() - Domain1.FirstParameter() < Tolerance)
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }
        if (Domain2.LastParameter() - Domain2.FirstParameter() < Tolerance)
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        // The intersection.
        Bisector_Inter Intersect = new();
        Intersect.Perform(GeomBis(BisectorOne.BisectorNumber()), Domain1, GeomBis(BisectorTwo.BisectorNumber()),
            Domain2, Tolerance, Tolerance, true);

        // The solution: the point equidistant from the two edges nearest in parameter to the origin of the
        // bisectors.
        if (!Intersect.IsDone())
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        if (Intersect.IsEmpty())
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        DistanceMini = global::Kerf.Engine.Foundation.Precision.Infinite();
        Param1 = global::Kerf.Engine.Foundation.Precision.Infinite();
        Param2 = global::Kerf.Engine.Foundation.Precision.Infinite();
        SolutionValide = false;

        if (Intersect.NbSegments() >= 1)
        {
            double MaxSegmentLength = 10.0 * Tolerance;
            for (int i = 1; i <= Intersect.NbSegments(); i++)
            {
                IntRes2d_IntersectionSegment Segment = Intersect.Segment(i);
                bool PointRetenu = false;
                Point2D PointOnSegment = new();
                // If the segments are small, the points on the segment equidistant from the edges.
                if ((Segment.HasFirstPoint() && Segment.HasLastPoint()))
                {
                    Point2D P1, P2;
                    double SegmentLength;
                    P1 = Segment.FirstPoint().Value();
                    P2 = Segment.LastPoint().Value();
                    SegmentLength = P1.Distance(P2);
                    if (SegmentLength <= Tolerance)
                    {
                        PointOnSegment = P1;
                        if (IsSameDistance(BisectorOne, BisectorTwo, PointOnSegment, out Distance))
                        {
                            PointRetenu = true;
                        }
                    }
                    else if (SegmentLength <= MaxSegmentLength)
                    {
                        Direction2D Dir = new(P2.X() - P1.X(), P2.Y() - P1.Y());
                        double Dist = 0.0;
                        while (Dist <= SegmentLength + Tolerance)
                        {
                            PointOnSegment = P1.Translated(Dist * new Vector2D(Dir));
                            if (IsSameDistance(BisectorOne, BisectorTwo, PointOnSegment, out Distance))
                            {
                                PointRetenu = true;
                                break;
                            }
                            Dist = Dist + Tolerance;
                        }
                    }
                }

                // The equidistant point of smallest parameter on the bisectors.
                if (PointRetenu)
                {
                    Parama = ((Bisector_Curve)Bisector1.ChangeBasisCurve()).Parameter(PointOnSegment);
                    Paramb = ((Bisector_Curve)Bisector2.ChangeBasisCurve()).Parameter(PointOnSegment);
                    if (Parama < Param1 && Paramb < Param2)
                    {
                        Param1 = Parama;
                        Param2 = Paramb;
                        DistanceMini = Distance;
                        PointSolution = PointOnSegment;
                        SolutionValide = true;
                    }
                }
            }
        }

        if (Intersect.NbPoints() != 1)
        {
            for (int i = 1; i <= Intersect.NbPoints(); i++)
            {
                if (IsSameDistance(BisectorOne, BisectorTwo, Intersect.Point(i).Value(), out Distance)
                    && Distance > Tolerance)
                {
                    Parama = Intersect.Point(i).ParamOnFirst();
                    Paramb = Intersect.Point(i).ParamOnSecond();
                    if (Parama < Param1 && Paramb < Param2)
                    {
                        Param1 = Parama;
                        Param2 = Paramb;
                        DistanceMini = Distance;
                        PointSolution = Intersect.Point(i).Value();
                        SolutionValide = true;
                    }
                }
            }
        }
        else
        {
            PointSolution = Intersect.Point(1).Value();
            Param1 = Intersect.Point(1).ParamOnFirst();
            Param2 = Intersect.Point(1).ParamOnSecond();
            SolutionValide = IsSameDistance(BisectorOne, BisectorTwo, PointSolution, out DistanceMini);
        }

        if (!SolutionValide)
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }
        theNumberOfPnts++;
        theGeomPnts.Bind(theNumberOfPnts, PointSolution);
        IntPnt = theNumberOfPnts;

        // If the intersection point is almost one of the ends of a bisector, the intersection is not valid,
        // UNLESS one of the bisectors starts from a connexion and the edges it separates are neighbours on the
        // initial contour: the middle of the connexion, the origin of one bisector, can be on the other one (a
        // rectangle in a rectangle with a connexion between a corner and a side), and the bisector from the
        // connexion has length zero.
        int IndexEdge1, IndexEdge2, IndexEdge3, IndexEdge4;
        bool ExtremiteControle = true;

        IndexEdge1 = BisectorOne.FirstEdge().EdgeNumber();
        IndexEdge2 = BisectorOne.SecondEdge().EdgeNumber();
        IndexEdge3 = BisectorTwo.FirstEdge().EdgeNumber();
        IndexEdge4 = BisectorTwo.SecondEdge().EdgeNumber();

        if (theCircuit!.ConnexionOn(IndexEdge2))
        {
            // BisectorOne starts from a connexion.
            if (AreNeighbours(IndexEdge1, IndexEdge2, NumberOfItems())
                && AreNeighbours(IndexEdge3, IndexEdge4, NumberOfItems()) && IndexEdge2 == IndexEdge3)
            {
                ExtremiteControle = false;
                Param1 = Param1 + Tolerance;
            }
        }

        if (theCircuit.ConnexionOn(IndexEdge4))
        {
            // BisectorTwo starts from a connexion.
            if (AreNeighbours(IndexEdge1, IndexEdge2, NumberOfItems())
                && AreNeighbours(IndexEdge3, IndexEdge4, NumberOfItems()) && IndexEdge2 == IndexEdge3)
            {
                ExtremiteControle = false;
                Param2 = Param2 + Tolerance;
            }
        }

        if (ExtremiteControle)
        {
            if (Bisector1.StartPoint().Distance(PointSolution) < Tolerance)
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
            if (Bisector2.StartPoint().Distance(PointSolution) < Tolerance)
            {
                return global::Kerf.Engine.Foundation.Precision.Infinite();
            }
        }

        if (BisectorOne.SecondParameter() < global::Kerf.Engine.Foundation.Precision.Infinite()
            && BisectorOne.SecondParameter() < Param1 * (1.0 - Tolerance))
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        if (BisectorTwo.FirstParameter() < global::Kerf.Engine.Foundation.Precision.Infinite()
            && BisectorTwo.FirstParameter() < Param2 * (1.0 - Tolerance))
        {
            return global::Kerf.Engine.Foundation.Precision.Infinite();
        }

        BisectorOne.SecondParameter(Param1);
        BisectorTwo.FirstParameter(Param2);

        return DistanceMini;
    }

    /// <summary>The distance between the points of parameters param1 and param2 on the bisector of abisector.
    /// </summary>
    public double Distance(MAT_Bisector Bis, double Param1, double Param2)
    {
        double Dist = global::Kerf.Engine.Foundation.Precision.Infinite();

        if (Param1 != global::Kerf.Engine.Foundation.Precision.Infinite() && Param2 != global::Kerf.Engine.Foundation.Precision.Infinite())
        {
            Point2D P1 = GeomBis(Bis.BisectorNumber()).Value().Value(Param1);
            Point2D P2 = GeomBis(Bis.BisectorNumber()).Value().Value(Param2);
            Dist = P1.Distance(P2);
        }
        return Dist;
    }

    /// <summary>Throws Standard_NotImplemented (OCCT's release build: the drawing exists only with OCCT_DEBUG).
    /// </summary>
    public void Dump(int bisector, int erease)
    {
        throw new MethodNotImplementedException();
    }

    /// <summary>The bisector of index Index.</summary>
    public Bisector_Bisec GeomBis(int Index) => theGeomBisectors.Find(Index);

    /// <summary>The bisector of index Index (OCCT's modifiable reference: the same object here).</summary>
    public Bisector_Bisec ChangeGeomBis(int Index) => theGeomBisectors.ChangeFind(Index);

    /// <summary>The item of index Index of the circuit.</summary>
    public Geometry2D GeomElt(int Index) => theCircuit!.Value(Index);

    /// <summary>The point of index Index.</summary>
    public Point2D GeomPnt(int Index) => theGeomPnts.Find(Index);

    /// <summary>The vector of index Index.</summary>
    public Vector2D GeomVec(int Index) => theGeomVecs.Find(Index);

    public MAT2d_Circuit Circuit() => theCircuit!;

    /// <summary>Merges the bisector I2 into the bisector I1 (I1 is extended over I2).</summary>
    public void BisecFusion(int I1, int I2)
    {
        double DU, UL1, UF1;
        TrimmedCurve2D Bisector1;
        TrimmedCurve2D Bisector2;

        Bisector1 = GeomBis(I1).Value();
        Bisector2 = GeomBis(I2).Value();
        UF1 = Bisector1.FirstParameter();
        UL1 = Bisector1.LastParameter();

        Type Type1 = Bisector1.ChangeBasisCurve().GetType();
        if (Type1 == typeof(Bisector_BisecCC))
        {
            // The curve/curve bisectors are built from a start point and can't be trimmed by a point on the other
            // side of it: the bisector of the two curves is rebuilt from the last point of Bisector2, then trimmed
            // by the last point of Bisector1.
            double Tolerance = MAT2d_TOLCONF;

            Bisector_Bisec Bis = new();
            Vector2D VBid = new(1, 0);
            Point2D P2 = Bisector2.Value(Bisector2.LastParameter());
            Point2D P1 = Bisector1.Value(Bisector1.LastParameter());
            Bisector_BisecCC BCC1 = (Bisector_BisecCC)Bisector1.ChangeBasisCurve();

            Bis.Perform(BCC1.Curve(2), BCC1.Curve(1), P2, VBid, VBid, theDirection, theJoinType, Tolerance, false);

            Bisector1 = Bis.Value();
            BCC1 = (Bisector_BisecCC)Bisector1.ChangeBasisCurve();
            UF1 = BCC1.FirstParameter();
            UL1 = BCC1.Parameter(P1);
            Bisector1.SetTrim(UF1, UL1);
            theGeomBisectors.Bind(I1, Bis);
        }
        else
        {
            DU = Bisector2.LastParameter() - Bisector2.FirstParameter();
            UF1 = UF1 - DU;

            // OCCT assumes a bisector that isn't curve/curve is analytic: a Bisector_BisecPC gives a null handle,
            // dereferenced (an access violation natively, a NullReferenceException here).
            Bisector_BisecAna? BAna = Bisector1.ChangeBasisCurve() as Bisector_BisecAna;
            BAna!.SetTrim(UF1, UL1);
            Bisector1.SetTrim(UF1, UL1);
        }
    }

    // True if IEdge1 and IEdge2 are consecutive elements of a closed contour of NbEdge elements.
    private static bool AreNeighbours(int IEdge1, int IEdge2, int NbEdge)
    {
        if (Math.Abs(IEdge1 - IEdge2) == 1)
        {
            return true;
        }
        else if (Math.Abs(IEdge1 - IEdge2) == NbEdge - 1)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    private static void SetTrim(Bisector_Bisec Bis, Curve2D Line1)
    {
        Geom2dInt_GInter Intersect = new();
        double Distance;
        double Tolerance = MAT2d_TOLCONF;
        TrimmedCurve2D Bisector = Bis.ChangeValue();

        IntRes2d_Domain Domain1 = Domain(Bisector, Tolerance);
        double UB1 = Bisector.FirstParameter();
        double UB2 = Bisector.LastParameter();

        Point2D FirstPointBisector = Bisector.Value(UB1);
        double UTrim = global::Kerf.Engine.Foundation.Precision.Infinite();

        CurveAdaptor2D AdapBisector = new(Bisector);
        CurveAdaptor2D AdapLine1 = new(Line1);
        Intersect.Perform(AdapBisector, Domain1, AdapLine1, Tolerance, Tolerance);

        if (Intersect.IsDone() && !Intersect.IsEmpty())
        {
            for (int i = 1; i <= Intersect.NbPoints(); i++)
            {
                Point2D PInt = Intersect.Point(i).Value();
                Distance = FirstPointBisector.Distance(PInt);
                if (Distance > 10.0 * Tolerance && Intersect.Point(i).ParamOnFirst() < UTrim)
                {
                    UTrim = Intersect.Point(i).ParamOnFirst();
                }
            }
        }
        // Restriction of the bisector by the intersection point of smallest parameter.
        if (UTrim < UB2 && UTrim > UB1)
        {
            Bisector.SetTrim(UB1, UTrim);
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

    private static IntRes2d_Domain Domain(TrimmedCurve2D Bisector1, double Tolerance)
    {
        double Param1 = Bisector1.FirstParameter();
        double Param2 = Bisector1.LastParameter();
        if (Param2 > 10000.0)
        {
            Param2 = 10000.0;
            Type Type1 = TypeOf(Bisector1.ChangeBasisCurve());
            Curve2D? BasisCurve = null;
            if (Type1 == typeof(Bisector_BisecAna))
            {
                BasisCurve = ((Bisector_BisecAna)Bisector1.ChangeBasisCurve()).Geom2dCurve();
                Type1 = BasisCurve.GetType();
            }
            Parabola2D gpParabola;
            Hyperbola2D gpHyperbola;
            double Focus;
            double Limit = 50000.0;
            if (Type1 == typeof(ParabolaCurve2D))
            {
                gpParabola = ((ParabolaCurve2D)BasisCurve!).Parab2d();
                Focus = gpParabola.Focal();
                double Val1 = Math.Sqrt(Limit * Focus);
                double Val2 = Math.Sqrt(Limit * Limit);
                Param2 = (Val1 <= Val2 ? Val1 : Val2);
            }
            else if (Type1 == typeof(HyperbolaCurve2D))
            {
                gpHyperbola = ((HyperbolaCurve2D)BasisCurve!).Hypr2d();
                double Majr = gpHyperbola.MajorRadius();
                double Minr = gpHyperbola.MinorRadius();
                double Valu1 = Limit / Majr;
                double Valu2 = Limit / Minr;
                double Val1 = Math.Log(Valu1 + Math.Sqrt(Valu1 * Valu1 - 1));
                double Val2 = Math.Log(Valu2 + Math.Sqrt(Valu2 * Valu2 + 1));
                Param2 = (Val1 <= Val2 ? Val1 : Val2);
            }
        }

        IntRes2d_Domain Domain1 = new(Bisector1.Value(Param1), Param1, Tolerance, Bisector1.Value(Param2), Param2,
            Tolerance);
        if (Bisector1.ChangeBasisCurve().IsPeriodic)
        {
            Domain1.SetEquivalentParameters(0.0, 2.0 * M_PI);
        }
        return Domain1;
    }

    private static bool CheckEnds(Geometry2D Elt, in Point2D PCom, double Distance, double Tol)
    {
        Type Type = Elt.GetType();
        TrimmedCurve2D Curve;

        if (Type == typeof(CartesianPoint2))
        {
            return false;
        }
        else
        {
            Curve = (TrimmedCurve2D)Elt;
            Point2D aPf = Curve.StartPoint();
            Point2D aPl = Curve.EndPoint();
            double df = PCom.Distance(aPf);
            double dl = PCom.Distance(aPl);
            if (Math.Abs(df - Distance) <= Tol)
            {
                return true;
            }
            if (Math.Abs(dl - Distance) <= Tol)
            {
                return true;
            }
        }
        return false;
    }
}
