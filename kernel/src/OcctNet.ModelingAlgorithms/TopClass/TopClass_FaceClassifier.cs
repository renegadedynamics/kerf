// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopClass/TopClass_FaceClassifier.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// The face explorer of a face classifier, as <see cref="TopClass_FaceClassifier.Perform"/> uses it (OCCT's
/// template parameter TheFaceExplorer: BRepClass_FaceExplorer, Geom2dHatch_Elements).
/// </summary>
internal interface ITopClass_FaceExplorer<TheEdge>
{
    /// <summary>Checks the point and moves it if needed; returns false to be called again.</summary>
    bool CheckPoint(ref Point2D thePoint);

    /// <summary>True if the point is certainly outside the face.</summary>
    bool Reject(in Point2D P);

    /// <summary>A half-line from <paramref name="P"/> and the parameter to intersect it to; false if it is not valid.</summary>
    bool Segment(in Point2D P, out Line2D L, out double Par);

    /// <summary>Another half-line from <paramref name="P"/>, after a bad classification; false if none is valid.</summary>
    bool OtherSegment(in Point2D P, out Line2D L, out double Par);

    void InitWires();

    bool MoreWires();

    void NextWire();

    /// <summary>True if the current wire can't meet the segment of <paramref name="L"/> up to <paramref name="Par"/>.</summary>
    bool RejectWire(in Line2D L, double Par);

    void InitEdges();

    bool MoreEdges();

    void NextEdge();

    /// <summary>True if the current edge can't meet the segment of <paramref name="L"/> up to <paramref name="Par"/>.</summary>
    bool RejectEdge(in Line2D L, double Par);

    /// <summary>
    /// Writes the current edge into <paramref name="E"/>, with its orientation. The edge is <c>ref</c>, as OCCT's
    /// <c>TheEdge&amp;</c>: the classifier passes the same variable for every edge, so what an explorer leaves
    /// unwritten carries over from the previous edge (BRepClass_FaceExplorer's next edge does). If TheEdge is a
    /// class, each call must give a new object, since the classifier keeps the closest one.
    /// </summary>
    void CurrentEdge(ref TheEdge E, out Orientation Or);
}

/// <summary>
/// The 2D classifier of a face classifier, as <see cref="TopClass_FaceClassifier.Perform"/> uses it (OCCT's
/// template parameter FClass2d: BRepClass_FClass2dOfFClassifier, Geom2dHatch_FClass2dOfClassifier).
/// </summary>
internal interface ITopClass_FClass2d<TheEdge>
{
    void Reset(in Line2D L, double P, double Tol);

    void Compare(in TheEdge E, Orientation Or);

    double Parameter();

    int ClosestIntersection();

    /// <summary>The classifier's intersector (OCCT returns it by reference: the classifier's own object).</summary>
    IntRes2d_Intersection Intersector();

    Containment State();

    bool IsHeadOrEnd();
}

/// <summary>
/// The bodies of OCCT's face classifiers (BRepClass_FClassifier, Geom2dHatch_Classifier): the state of a point in a
/// face, from the edges met by half-lines from the point. OCCT's template functions over the classifier's fields,
/// passed by reference.
/// </summary>
/// <remarks>
/// OCCT's private header TopClass_FaceClassifier.pxx (a namespace of inline templates), ported as an internal static
/// class with generic methods (decision 285).
/// </remarks>
internal static class TopClass_FaceClassifier
{
    /// <summary>
    /// Classifies <paramref name="theP"/> in the face explored by <paramref name="theFexp"/>; the state is then in
    /// <paramref name="theClassifier"/> (see <see cref="State"/>), and the closest edge, the position and the
    /// parameter on it in the other outputs.
    /// </summary>
    internal static void Perform<TheFaceExplorer, TheEdge, FClass2d>(out bool theRejected,
                                                                    ref bool theNoWires,
                                                                    FClass2d theClassifier,
                                                                    ref TheEdge theEdge,
                                                                    ref IntRes2d_Position thePosition,
                                                                    ref double theEdgeParameter,
                                                                    TheFaceExplorer theFexp,
                                                                    in Point2D theP,
                                                                    double theTol)
        where TheFaceExplorer : ITopClass_FaceExplorer<TheEdge>
        where FClass2d : ITopClass_FClass2d<TheEdge>
        where TheEdge : new()
    {
        Point2D aPoint = theP;
        bool aResOfPointCheck = false;
        while (aResOfPointCheck == false)
        {
            aResOfPointCheck = theFexp.CheckPoint(ref aPoint);
        }

        // Test for rejection.
        theRejected = theFexp.Reject(aPoint);
        if (theRejected)
        {
            return;
        }

        Line2D aLine;
        double aParam;
        bool aIsValidSegment = theFexp.Segment(aPoint, out aLine, out aParam);
        // C++ default-constructs one edge for the whole classification (see ITopClass_FaceExplorer.CurrentEdge)
        TheEdge anEdge = new();
        Orientation anEdgeOri;
        int aClosestInd;
        IntRes2d_IntersectionPoint aPInter;
        Containment aState = Unknown;
        bool aIsWReject;
        bool aIsEReject;

        theNoWires = true;

        while (aIsValidSegment)
        {
            theClassifier.Reset(aLine, aParam, theTol);

            for (theFexp.InitWires(); theFexp.MoreWires(); theFexp.NextWire())
            {
                theNoWires = false;
                aIsWReject = theFexp.RejectWire(aLine, theClassifier.Parameter());

                if (!aIsWReject)
                {
                    // Test this wire.
                    for (theFexp.InitEdges(); theFexp.MoreEdges(); theFexp.NextEdge())
                    {
                        aIsEReject = theFexp.RejectEdge(aLine, theClassifier.Parameter());

                        if (!aIsEReject)
                        {
                            // Test this edge.
                            theFexp.CurrentEdge(ref anEdge, out anEdgeOri);

                            if (anEdgeOri == Forward || anEdgeOri == Reversed)
                            {
                                theClassifier.Compare(anEdge, anEdgeOri);
                                aClosestInd = theClassifier.ClosestIntersection();

                                if (aClosestInd != 0)
                                {
                                    // Save the closest edge.
                                    IntRes2d_Intersection anIntersector = theClassifier.Intersector();
                                    int aNbPnts = anIntersector.NbPoints();
                                    theEdge = anEdge;

                                    if (aClosestInd <= aNbPnts)
                                    {
                                        aPInter = anIntersector.Point(aClosestInd);
                                    }
                                    else
                                    {
                                        aClosestInd -= aNbPnts;
                                        if ((aClosestInd & 1) != 0)
                                        {
                                            aPInter = anIntersector.Segment((aClosestInd + 1) / 2).FirstPoint();
                                        }
                                        else
                                        {
                                            aPInter = anIntersector.Segment((aClosestInd + 1) / 2).LastPoint();
                                        }
                                    }

                                    thePosition = aPInter.TransitionOfSecond().PositionOnCurve();
                                    theEdgeParameter = aPInter.ParamOnSecond();
                                }

                                // If we are ON, we stop.
                                aState = theClassifier.State();
                                if (aState == On)
                                {
                                    return;
                                }
                            }
                        }
                    }

                    // If we are out of the wire we stop.
                    aState = theClassifier.State();
                    if (aState == Out)
                    {
                        return;
                    }
                }
            }

            if (!theClassifier.IsHeadOrEnd() && aState != Unknown)
            {
                break;
            }

            // Bad case for classification. Trying to get another segment.
            aIsValidSegment = theFexp.OtherSegment(aPoint, out aLine, out aParam);
        }
    }

    /// <summary>The state of the point: OUT if rejected, IN if the face has no wires, the classifier's otherwise.</summary>
    internal static Containment State<TheEdge>(bool theRejected, bool theNoWires,
                                                ITopClass_FClass2d<TheEdge> theClassifier)
    {
        if (theRejected)
        {
            return Out;
        }
        else if (theNoWires)
        {
            return In;
        }
        else
        {
            return theClassifier.State();
        }
    }

    /// <summary>The closest edge.</summary>
    /// <exception cref="DomainErrorException">The point was rejected.</exception>
    internal static TheEdge Edge<TheEdge>(bool theRejected, in TheEdge theEdge)
    {
        Standard_DomainError_Raise_if(theRejected, "TopClass_FaceClassifier::Edge : rejected");
        return theEdge;
    }

    /// <summary>The parameter on the closest edge.</summary>
    /// <exception cref="DomainErrorException">The point was rejected.</exception>
    internal static double EdgeParameter(bool theRejected, double theEdgeParameter)
    {
        Standard_DomainError_Raise_if(theRejected, "TopClass_FaceClassifier::EdgeParameter : rejected");
        return theEdgeParameter;
    }
}
