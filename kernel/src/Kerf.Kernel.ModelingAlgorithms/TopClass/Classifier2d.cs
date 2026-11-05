// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopClass/TopClass_Classifier2d.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Engine.Intersection.TwoD.Situation;
using static Kerf.Engine.Intersection.TwoD.TypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Classification.Topological;

/// <summary>
/// The intersector of a 2D classifier, as <see cref="Classifier2d.Compare"/> uses it (OCCT's template
/// parameter TheIntersector: BRepClass_Intersector, Geom2dHatch_Intersector): an
/// <see cref="Intersection2D"/> that also implements this interface.
/// </summary>
internal interface IIntersector<TheEdge>
{
    /// <summary>Intersects the segment of <paramref name="l"/> from its origin to the parameter <paramref name="p"/> with the edge.</summary>
    void Perform(in Line2D l, double p, double tol, in TheEdge e);

    /// <summary>The tangent, normal and curvature of the edge at the parameter <paramref name="u"/>.</summary>
    void LocalGeometry(in TheEdge e, double u, out Direction2D t, out Direction2D n, out double c);
}

/// <summary>
/// The bodies of OCCT's 2D classifiers (BRepClass_FClass2dOfFClassifier, Geom2dHatch_FClass2dOfClassifier): the
/// state of a point found from the intersections of a half-line with the edges of a face. OCCT's template
/// functions over the classifier's fields, passed by reference.
/// </summary>
/// <remarks>
/// OCCT's private header TopClass_Classifier2d.pxx (a namespace of inline templates), ported as an internal static
/// class with generic methods (decision 285). The classifiers in BRepClass and Geom2dHatch keep their fields and
/// call these with <c>ref</c>, as OCCT's do.
/// </remarks>
internal static class Classifier2d
{
    /// <summary>Starts a classification along the half-line <paramref name="l"/>, up to the parameter <paramref name="p"/>.</summary>
    internal static void Reset(out Line2D lin,
                               out double param,
                               out double tolerance,
                               out Containment state,
                               out bool firstCompare,
                               out bool firstTrans,
                               out int closest,
                               out bool isSet,
                               out bool isHeadOrEnd,
                               in Line2D l,
                               double p,
                               double tol)
    {
        lin = l;
        param = p;
        tolerance = tol;
        state = global::Kerf.Topology.Containment.Unknown;
        firstCompare = true;
        firstTrans = true;
        closest = 0;
        isSet = true;
        isHeadOrEnd = false;
    }

    /// <summary>
    /// Updates the state with the edge <paramref name="edge"/> of orientation <paramref name="or"/>: the
    /// intersection of the edge with the half-line nearest to the line's origin decides the state, through the
    /// complex transition <paramref name="trans"/> when it is at an end of the edge.
    /// </summary>
    internal static void Compare<TheEdge, TheIntersector>(ref int closest,
                                                          TheIntersector intersector,
                                                          in Line2D lin,
                                                          ref double param,
                                                          double tolerance,
                                                          ref Containment state,
                                                          ref bool firstCompare,
                                                          ref bool firstTrans,
                                                          CurveTransition trans,
                                                          ref bool isHeadOrEnd,
                                                          in TheEdge edge,
                                                          Orientation or)
        where TheIntersector : Intersection2D, IIntersector<TheEdge>
    {
        // Intersect the edge and the segment.
        closest = 0;
        intersector.Perform(lin, param, tolerance, edge);

        if (!intersector.IsDone)
        {
            return;
        }

        if ((intersector.PointCount == 0) && (intersector.SegmentCount == 0))
        {
            return;
        }

        // Find the closest point.
        int point, segment, nbPoints, nbSegments;
        // C++ keeps a pointer to the closest point: a copy here (the points are values)
        IntersectionPoint2D pClosest = new();
        double dMin = RealLast();

        nbPoints = intersector.PointCount;
        for (point = 1; point <= nbPoints; point++)
        {
            IntersectionPoint2D pInter = intersector.Point(point);
            // Test for ON.
            if (pInter.TransitionOfFirst().PositionOnCurve() == Position2D.Head)
            {
                closest = point;
                state = On;
                return;
            }

            double paramFirst = pInter.ParamOnFirst();
            if (paramFirst < dMin)
            {
                closest = point;
                pClosest = pInter;
                dMin = paramFirst;
            }
        }

        // For the segments we only test the first point.
        nbSegments = intersector.SegmentCount;
        for (segment = 1; segment <= nbSegments; segment++)
        {
            IntersectionSegment2D segInter = intersector.Segment(segment);
            IntersectionPoint2D pInter = segInter.FirstPoint();
            if (pInter.TransitionOfFirst().PositionOnCurve() == Position2D.Head)
            {
                closest = nbPoints + segment + segment - 1;
                state = On;
                return;
            }

            double paramFirst = pInter.ParamOnFirst();
            if (paramFirst < dMin)
            {
                closest = nbPoints + segment + segment - 1;
                pClosest = pInter;
                dMin = paramFirst;
            }
        }

        // If no point was found return.
        if (closest == 0)
        {
            return;
        }

        // If the edge is INTERNAL or EXTERNAL, no problem.
        if (or == Internal)
        {
            state = global::Kerf.Topology.Containment.In;
            return;
        }
        else if (or == External)
        {
            state = global::Kerf.Topology.Containment.Out;
            return;
        }

        if (!firstCompare)
        {
            if (dMin > param)
            {
                return;
            }
        }

        // Process the closest point aPClosest, found at aDMin on line.
        firstCompare = false;

        if (param > dMin)
        {
            firstTrans = true;
        }

        param = dMin;

        Transition t2 = pClosest.TransitionOfSecond();
        isHeadOrEnd =
            (t2.PositionOnCurve() == Position2D.Head) || (t2.PositionOnCurve() == Position2D.End);

        // Transition on the segment.
        Orientation segTrans = Forward;

        Transition t1 = pClosest.TransitionOfFirst();
        switch (t1.TransitionType())
        {
            case TypeTrans.In:
                segTrans = (or == Reversed) ? Reversed : Forward;
                break;
            case TypeTrans.Out:
                segTrans = (or == Reversed) ? Forward : Reversed;
                break;
            case TypeTrans.Touch:
                switch (t1.Situation())
                {
                    case Situation.Inside:
                        segTrans = (or == Reversed) ? External : Internal;
                        break;
                    case Situation.Outside:
                        segTrans = (or == Reversed) ? Internal : External;
                        break;
                    case Situation.Unknown:
                        return;
                }

                break;
            case TypeTrans.Undecided:
                return;
        }

        // Are we inside the edge?
        if (!isHeadOrEnd)
        {
            // aPClosest is inside the edge.
            switch (segTrans)
            {
                case Forward:
                case External:
                    state = global::Kerf.Topology.Containment.Out;
                    break;
                case Reversed:
                case Internal:
                    state = global::Kerf.Topology.Containment.In;
                    break;
            }
        }
        else
        {
            // aPClosest is Head or End of the edge: update the complex transition.
            Direction2D tang2d, norm2d;
            double curv;
            intersector.LocalGeometry(edge, pClosest.ParamOnSecond(), out tang2d, out norm2d, out curv);
            Direction3D tang = new(tang2d.X, tang2d.Y, 0.0);
            Direction3D norm = new(norm2d.X, norm2d.Y, 0.0);

            if (firstTrans)
            {
                Direction3D dir = new(lin.Direction.X, lin.Direction.Y, 0.0);
                trans.Reset(dir);
                firstTrans = false;
            }

            Orientation ort =
                (t2.PositionOnCurve() == Position2D.Head) ? Forward : Reversed;
            trans.Compare(RealEpsilon(), tang, norm, curv, segTrans, ort);
            state = trans.StateBefore();
        }
    }
}
