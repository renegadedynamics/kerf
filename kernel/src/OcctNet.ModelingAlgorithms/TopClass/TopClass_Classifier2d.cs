// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopClass/TopClass_Classifier2d.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Occt.IntRes2d_Position;
using static Occt.IntRes2d_Situation;
using static Occt.IntRes2d_TypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// The intersector of a 2D classifier, as <see cref="TopClass_Classifier2d.Compare"/> uses it (OCCT's template
/// parameter TheIntersector: BRepClass_Intersector, Geom2dHatch_Intersector): an
/// <see cref="IntRes2d_Intersection"/> that also implements this interface.
/// </summary>
internal interface ITopClass_Intersector<TheEdge>
{
    /// <summary>Intersects the segment of <paramref name="L"/> from its origin to the parameter <paramref name="P"/> with the edge.</summary>
    void Perform(in Line2D L, double P, double Tol, in TheEdge E);

    /// <summary>The tangent, normal and curvature of the edge at the parameter <paramref name="U"/>.</summary>
    void LocalGeometry(in TheEdge E, double U, out Direction2D T, out Direction2D N, out double C);
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
internal static class TopClass_Classifier2d
{
    /// <summary>Starts a classification along the half-line <paramref name="theL"/>, up to the parameter <paramref name="theP"/>.</summary>
    internal static void Reset(out Line2D theLin,
                               out double theParam,
                               out double theTolerance,
                               out Containment theState,
                               out bool theFirstCompare,
                               out bool theFirstTrans,
                               out int theClosest,
                               out bool theIsSet,
                               out bool theIsHeadOrEnd,
                               in Line2D theL,
                               double theP,
                               double theTol)
    {
        theLin = theL;
        theParam = theP;
        theTolerance = theTol;
        theState = Unknown;
        theFirstCompare = true;
        theFirstTrans = true;
        theClosest = 0;
        theIsSet = true;
        theIsHeadOrEnd = false;
    }

    /// <summary>
    /// Updates the state with the edge <paramref name="theEdge"/> of orientation <paramref name="theOr"/>: the
    /// intersection of the edge with the half-line nearest to the line's origin decides the state, through the
    /// complex transition <paramref name="theTrans"/> when it is at an end of the edge.
    /// </summary>
    internal static void Compare<TheEdge, TheIntersector>(ref int theClosest,
                                                          TheIntersector theIntersector,
                                                          in Line2D theLin,
                                                          ref double theParam,
                                                          double theTolerance,
                                                          ref Containment theState,
                                                          ref bool theFirstCompare,
                                                          ref bool theFirstTrans,
                                                          TopTrans_CurveTransition theTrans,
                                                          ref bool theIsHeadOrEnd,
                                                          in TheEdge theEdge,
                                                          Orientation theOr)
        where TheIntersector : IntRes2d_Intersection, ITopClass_Intersector<TheEdge>
    {
        // Intersect the edge and the segment.
        theClosest = 0;
        theIntersector.Perform(theLin, theParam, theTolerance, theEdge);

        if (!theIntersector.IsDone())
        {
            return;
        }

        if ((theIntersector.NbPoints() == 0) && (theIntersector.NbSegments() == 0))
        {
            return;
        }

        // Find the closest point.
        int aPoint, aSegment, aNbPoints, aNbSegments;
        // C++ keeps a pointer to the closest point: a copy here (the points are values)
        IntRes2d_IntersectionPoint aPClosest = new();
        double aDMin = RealLast();

        aNbPoints = theIntersector.NbPoints();
        for (aPoint = 1; aPoint <= aNbPoints; aPoint++)
        {
            IntRes2d_IntersectionPoint aPInter = theIntersector.Point(aPoint);
            // Test for ON.
            if (aPInter.TransitionOfFirst().PositionOnCurve() == IntRes2d_Head)
            {
                theClosest = aPoint;
                theState = On;
                return;
            }

            double aParamFirst = aPInter.ParamOnFirst();
            if (aParamFirst < aDMin)
            {
                theClosest = aPoint;
                aPClosest = aPInter;
                aDMin = aParamFirst;
            }
        }

        // For the segments we only test the first point.
        aNbSegments = theIntersector.NbSegments();
        for (aSegment = 1; aSegment <= aNbSegments; aSegment++)
        {
            IntRes2d_IntersectionSegment aSegInter = theIntersector.Segment(aSegment);
            IntRes2d_IntersectionPoint aPInter = aSegInter.FirstPoint();
            if (aPInter.TransitionOfFirst().PositionOnCurve() == IntRes2d_Head)
            {
                theClosest = aNbPoints + aSegment + aSegment - 1;
                theState = On;
                return;
            }

            double aParamFirst = aPInter.ParamOnFirst();
            if (aParamFirst < aDMin)
            {
                theClosest = aNbPoints + aSegment + aSegment - 1;
                aPClosest = aPInter;
                aDMin = aParamFirst;
            }
        }

        // If no point was found return.
        if (theClosest == 0)
        {
            return;
        }

        // If the edge is INTERNAL or EXTERNAL, no problem.
        if (theOr == Internal)
        {
            theState = In;
            return;
        }
        else if (theOr == External)
        {
            theState = Out;
            return;
        }

        if (!theFirstCompare)
        {
            if (aDMin > theParam)
            {
                return;
            }
        }

        // Process the closest point aPClosest, found at aDMin on line.
        theFirstCompare = false;

        if (theParam > aDMin)
        {
            theFirstTrans = true;
        }

        theParam = aDMin;

        IntRes2d_Transition aT2 = aPClosest.TransitionOfSecond();
        theIsHeadOrEnd =
            (aT2.PositionOnCurve() == IntRes2d_Head) || (aT2.PositionOnCurve() == IntRes2d_End);

        // Transition on the segment.
        Orientation aSegTrans = Forward;

        IntRes2d_Transition aT1 = aPClosest.TransitionOfFirst();
        switch (aT1.TransitionType())
        {
            case IntRes2d_In:
                aSegTrans = (theOr == Reversed) ? Reversed : Forward;
                break;
            case IntRes2d_Out:
                aSegTrans = (theOr == Reversed) ? Forward : Reversed;
                break;
            case IntRes2d_Touch:
                switch (aT1.Situation())
                {
                    case IntRes2d_Inside:
                        aSegTrans = (theOr == Reversed) ? External : Internal;
                        break;
                    case IntRes2d_Outside:
                        aSegTrans = (theOr == Reversed) ? Internal : External;
                        break;
                    case IntRes2d_Unknown:
                        return;
                }

                break;
            case IntRes2d_Undecided:
                return;
        }

        // Are we inside the edge?
        if (!theIsHeadOrEnd)
        {
            // aPClosest is inside the edge.
            switch (aSegTrans)
            {
                case Forward:
                case External:
                    theState = Out;
                    break;
                case Reversed:
                case Internal:
                    theState = In;
                    break;
            }
        }
        else
        {
            // aPClosest is Head or End of the edge: update the complex transition.
            Direction2D aTang2d, aNorm2d;
            double aCurv;
            theIntersector.LocalGeometry(theEdge, aPClosest.ParamOnSecond(), out aTang2d, out aNorm2d, out aCurv);
            Direction3D aTang = new(aTang2d.X(), aTang2d.Y(), 0.0);
            Direction3D aNorm = new(aNorm2d.X(), aNorm2d.Y(), 0.0);

            if (theFirstTrans)
            {
                Direction3D aDir = new(theLin.Direction().X(), theLin.Direction().Y(), 0.0);
                theTrans.Reset(aDir);
                theFirstTrans = false;
            }

            Orientation aOrt =
                (aT2.PositionOnCurve() == IntRes2d_Head) ? Forward : Reversed;
            theTrans.Compare(RealEpsilon(), aTang, aNorm, aCurv, aSegTrans, aOrt);
            theState = theTrans.StateBefore();
        }
    }
}
