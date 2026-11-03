// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FaceExplorer.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass/BRepClass_FaceExplorer.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The exploration of a face for its classification: the half-lines from the point to classify, the wires and their
/// edges (with their pcurves).
/// </summary>
/// <remarks>A working object (decision 21). It only reads the face.</remarks>
public class BRepClass_FaceExplorer : ITopClass_FaceExplorer<BRepClass_Edge>
{
    private const double Probing_Start = 0.123;
    private const double Probing_End = 0.7;
    private const double Probing_Step = 0.2111;

    private Face myFace;
    private readonly Explorer myWExplorer = new();
    private readonly Explorer myEExplorer = new();
    private int myCurEdgeInd;
    private double myCurEdgePar;
    private double myMaxTolerance;
    private bool myUseBndBox;

    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myMapVE =
        new(ShapeHasher.Instance);

    private double myUMin;
    private double myUMax;
    private double myVMin;
    private double myVMax;

    /// <summary>An explorer of the face <paramref name="F"/> (taken FORWARD).</summary>
    public BRepClass_FaceExplorer(in Face F)
    {
        myFace = F;
        myCurEdgeInd = 1;
        myCurEdgePar = Probing_Start;
        myMaxTolerance = 0.1;
        myUseBndBox = false;
        myUMin = global::Kerf.Engine.Foundation.Precision.Infinite();
        myUMax = -global::Kerf.Engine.Foundation.Precision.Infinite();
        myVMin = global::Kerf.Engine.Foundation.Precision.Infinite();
        myVMax = -global::Kerf.Engine.Foundation.Precision.Infinite();
        myFace.Orientation(Forward);
    }

    /// <summary>Computes the UV bounds of the face.</summary>
    protected void ComputeFaceBounds()
    {
        Surface aSurface = Tool.Surface(myFace, out Placement _)!;
        aSurface.Bounds(out myUMin, out myUMax, out myVMin, out myVMax);
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(myUMin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(myUMax)
            || global::Kerf.Engine.Foundation.Precision.IsInfinite(myVMin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(myVMax))
        {
            BrepTools.UVBounds(myFace, out myUMin, out myUMax, out myVMin, out myVMax);
        }
    }

    /// <summary>
    /// Checks the point, and moves it if it is too far from the face's UV box: to the line from the box's centre to
    /// the point. Returns true if the point was not changed.
    /// </summary>
    public bool CheckPoint(ref Point2D thePoint)
    {
        if (myUMin > myUMax)
        {
            ComputeFaceBounds();
        }

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(myUMin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(myUMax)
            || global::Kerf.Engine.Foundation.Precision.IsInfinite(myVMin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(myVMax))
        {
            return true;
        }

        Point2D aCenterPnt = new((myUMin + myUMax) / 2, (myVMin + myVMax) / 2);
        double aDistance = aCenterPnt.Distance(thePoint);
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aDistance))
        {
            thePoint.SetCoord(myUMin - (myUMax - myUMin), myVMin - (myVMax - myVMin));
            return false;
        }
        else
        {
            double anEpsilon = Epsilon(aDistance);
            if (anEpsilon > Max(myUMax - myUMin, myVMax - myVMin))
            {
                Vector2D aLinVec = new(aCenterPnt, thePoint);
                Direction2D aLinDir = new(aLinVec);
                thePoint = new Point2D(aCenterPnt.XY() + aLinDir.XY() * (2.0 * anEpsilon));
                return false;
            }
        }

        return true;
    }

    /// <summary>True if the point is outside a bounding volume of the face: always false.</summary>
    public virtual bool Reject(in Point2D P) => false;

    /// <summary>
    /// A half-line from <paramref name="P"/> in <paramref name="L"/>, with in <paramref name="Par"/> the parameter up
    /// to which it meets the face's boundary at least once. Starts again from the first edge.
    /// </summary>
    public bool Segment(in Point2D P, out Line2D L, out double Par)
    {
        myCurEdgeInd = 1;
        myCurEdgePar = Probing_Start;

        return OtherSegment(P, out L, out Par);
    }

    /// <summary>Another half-line from <paramref name="P"/> (see <see cref="Segment"/>): each call gives another one.</summary>
    public bool OtherSegment(in Point2D P, out Line2D L, out double Par)
    {
        var anExpF = new Explorer(myFace, ShapeType.Edge);
        int i;
        double aFPar;
        double aLPar;
        Curve2D? aC2d;
        double aTolParConf2 = global::Kerf.Engine.Foundation.Precision.PConfusion() * global::Kerf.Engine.Foundation.Precision.PConfusion();
        Point2D aPOnC;
        double aParamIn;
        // C++ leaves L and Par unwritten for the edges it skips; they are written before every return
        L = new Line2D();
        Par = 0.0;

        for (i = 1; anExpF.More(); anExpF.Next(), i++)
        {
            if (i != myCurEdgeInd)
            {
                continue;
            }

            Shape aLocalShape = anExpF.Current();
            Orientation anOrientation = aLocalShape.Orientation();

            if (anOrientation == Forward || anOrientation == Reversed)
            {
                Edge anEdge = ShapeCasts.Edge(aLocalShape);

                aC2d = Tool.CurveOnSurface(anEdge, myFace, out aFPar, out aLPar);

                if (aC2d is not null)
                {
                    // Treatment of infinite cases.
                    if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(aFPar))
                    {
                        if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(aLPar))
                        {
                            aFPar = -1.0;
                            aLPar = 1.0;
                        }
                        else
                        {
                            aFPar = aLPar - 1.0;
                        }
                    }
                    else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(aLPar))
                    {
                        aLPar = aFPar + 1.0;
                    }

                    for (; myCurEdgePar < Probing_End; myCurEdgePar += Probing_Step)
                    {
                        aParamIn = myCurEdgePar * aFPar + (1.0 - myCurEdgePar) * aLPar;

                        aC2d.D1(aParamIn, out aPOnC, out Vector2D aTanVec);
                        Par = aPOnC.SquareDistance(P);

                        if (Par > aTolParConf2)
                        {
                            Vector2D aLinVec = new(P, aPOnC);
                            Direction2D aLinDir = new(aLinVec);

                            double aTanMod = aTanVec.SquareMagnitude();
                            if (aTanMod < aTolParConf2)
                            {
                                continue;
                            }

                            aTanVec /= Math.Sqrt(aTanMod);
                            double aSinA = aTanVec.Crossed(new Vector2D(aLinDir.XY()));
                            const double SmallAngle = 0.001;
                            bool isSmallAngle = false;
                            if (Math.Abs(aSinA) < SmallAngle)
                            {
                                isSmallAngle = true;
                                // The line from the input point P to the current point on edge
                                // is tangent to the edge curve. This condition is bad for classification.
                                // Therefore try to go to another point in the hope that there will be
                                // no tangent. If there tangent is preserved then leave the last point in
                                // order to get this edge chance to participate in classification.
                                if (myCurEdgePar + Probing_Step < Probing_End)
                                {
                                    continue;
                                }
                            }

                            L = new Line2D(P, aLinDir);

                            // Check if ends of a curve lie on a line.
                            aC2d.D0(aFPar, out aPOnC);
                            Point2D aFPOnC = aPOnC;
                            if (L.SquareDistance(aPOnC) > aTolParConf2)
                            {
                                aC2d.D0(aLPar, out aPOnC);
                                if (L.SquareDistance(aPOnC) > aTolParConf2)
                                {
                                    if (isSmallAngle)
                                    {
                                        // Try to find minimal distance between curve and line

                                        var aProj = new Geom2dAPI_ProjectPointOnCurve();
                                        aProj.Init(P, aC2d, aFPar, aLPar);
                                        if (aProj.NbPoints() > 0)
                                        {
                                            Point2D aLPOnC = aPOnC;
                                            double aFDist = P.SquareDistance(aFPOnC);
                                            double aLDist = P.SquareDistance(aLPOnC);
                                            double aMinDist = aProj.LowerDistance();
                                            aMinDist *= aMinDist;
                                            aPOnC = aProj.NearestPoint();
                                            if (aMinDist > aFDist)
                                            {
                                                aMinDist = aFDist;
                                                aPOnC = aFPOnC;
                                            }

                                            if (aMinDist > aLDist)
                                            {
                                                aMinDist = aLDist;
                                                aPOnC = aLPOnC;
                                            }

                                            if (aMinDist < Par)
                                            {
                                                Par = aMinDist;
                                                if (Par < aTolParConf2)
                                                {
                                                    continue;
                                                }

                                                aLinVec.SetXY((aPOnC.XY() - P.XY()));
                                                aLinDir.SetXY(aLinVec.XY());
                                                L = new Line2D(P, aLinDir);
                                            }
                                        }
                                    }

                                    myCurEdgePar += Probing_Step;
                                    if (myCurEdgePar >= Probing_End)
                                    {
                                        myCurEdgeInd++;
                                        myCurEdgePar = Probing_Start;
                                    }

                                    Par = Math.Sqrt(Par);
                                    return true;
                                }
                            }
                        }
                    }
                } // if (!aC2d.IsNull()) {
            } // if (anOrientation == TopAbs_FORWARD ...

            // This curve is not valid for line construction. Go to another edge.
            myCurEdgeInd++;
            myCurEdgePar = Probing_Start;
        }

        // nothing found, return an horizontal line
        Par = RealLast();
        L = new Line2D(P, new Direction2D(Direction2D.D.X));

        return false;
    }

    /// <summary>Starts an exploration of the wires.</summary>
    public void InitWires() => myWExplorer.Init(myFace, ShapeType.Wire);

    /// <summary>True if there is a current wire.</summary>
    public bool MoreWires() => myWExplorer.More();

    /// <summary>Moves to the next wire.</summary>
    public void NextWire() => myWExplorer.Next();

    /// <summary>True if the wire's bounding volume doesn't meet the segment: always false.</summary>
    public virtual bool RejectWire(in Line2D L, double Par) => false;

    /// <summary>Starts an exploration of the edges of the current wire.</summary>
    public void InitEdges()
    {
        myEExplorer.Init(myWExplorer.Current(), ShapeType.Edge);
        myMapVE.Clear();
        Exploration.MapShapesAndAncestors(myWExplorer.Current(), ShapeType.Vertex, ShapeType.Edge, myMapVE);
    }

    /// <summary>True if there is a current edge.</summary>
    public bool MoreEdges() => myEExplorer.More();

    /// <summary>Moves to the next edge.</summary>
    public void NextEdge() => myEExplorer.Next();

    /// <summary>True if the edge's bounding volume doesn't meet the segment: always false.</summary>
    public virtual bool RejectEdge(in Line2D L, double Par) => false;

    /// <summary>
    /// Writes the current edge of the current wire into <paramref name="E"/>, with its orientation. As in OCCT, the
    /// next edge of <paramref name="E"/> is left as it was when the current edge has none (see
    /// <see cref="BRepClass_Edge.SetNextEdge"/>).
    /// </summary>
    public void CurrentEdge(ref BRepClass_Edge E, out Orientation Or)
    {
        // OCCT: the next edge left from the previous edge reaches the intersector's CheckSkip for an edge without one
        // (a closed edge, or a vertex with more than two edges) (occt-issues.md)
        E.ChangeEdge() = ShapeCasts.Edge(myEExplorer.Current());
        E.ChangeFace() = myFace;
        Or = E.Edge().Orientation();
        E.SetNextEdge(myMapVE);
        E.SetMaxTolerance(myMaxTolerance);
        E.SetUseBndBox(myUseBndBox);
    }

    /// <summary>The maximum tolerance given to the edges.</summary>
    public double MaxTolerance() => myMaxTolerance;

    /// <summary>Sets the maximum tolerance from which the intersector checks for a vertex with a high tolerance.</summary>
    public void SetMaxTolerance(double theValue) => myMaxTolerance = theValue;

    /// <summary>True if the intersector uses bounding boxes.</summary>
    public bool UseBndBox() => myUseBndBox;

    /// <summary>Sets whether the intersector uses bounding boxes.</summary>
    public void SetUseBndBox(bool theValue) => myUseBndBox = theValue;
}
