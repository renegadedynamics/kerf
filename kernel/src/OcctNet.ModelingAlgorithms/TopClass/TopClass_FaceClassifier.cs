// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopClass/TopClass_FaceClassifier.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// The face explorer of a face classifier, as <see cref="TopClass_FaceClassifier.Perform"/> uses it (OCCT's
/// template parameter TheFaceExplorer: BRepClass_FaceExplorer, Geom2dHatch_Elements).
/// </summary>
internal interface ITopClass_FaceExplorer<TheEdge>
{
    /// <summary>Checks the point and moves it if needed; returns false to be called again.</summary>
    bool CheckPoint(ref gp_Pnt2d thePoint);

    /// <summary>True if the point is certainly outside the face.</summary>
    bool Reject(in gp_Pnt2d P);

    /// <summary>A half-line from <paramref name="P"/> and the parameter to intersect it to; false if it is not valid.</summary>
    bool Segment(in gp_Pnt2d P, out gp_Lin2d L, out double Par);

    /// <summary>Another half-line from <paramref name="P"/>, after a bad classification; false if none is valid.</summary>
    bool OtherSegment(in gp_Pnt2d P, out gp_Lin2d L, out double Par);

    void InitWires();

    bool MoreWires();

    void NextWire();

    /// <summary>True if the current wire can't meet the segment of <paramref name="L"/> up to <paramref name="Par"/>.</summary>
    bool RejectWire(in gp_Lin2d L, double Par);

    void InitEdges();

    bool MoreEdges();

    void NextEdge();

    /// <summary>True if the current edge can't meet the segment of <paramref name="L"/> up to <paramref name="Par"/>.</summary>
    bool RejectEdge(in gp_Lin2d L, double Par);

    /// <summary>
    /// The current edge and its orientation. The edge is <c>out</c>: if TheEdge is a class, each call must return
    /// a new object, since the classifier keeps the closest one.
    /// </summary>
    void CurrentEdge(out TheEdge E, out TopAbs_Orientation Or);
}

/// <summary>
/// The 2D classifier of a face classifier, as <see cref="TopClass_FaceClassifier.Perform"/> uses it (OCCT's
/// template parameter FClass2d: BRepClass_FClass2dOfFClassifier, Geom2dHatch_FClass2dOfClassifier).
/// </summary>
internal interface ITopClass_FClass2d<TheEdge>
{
    void Reset(in gp_Lin2d L, double P, double Tol);

    void Compare(in TheEdge E, TopAbs_Orientation Or);

    double Parameter();

    int ClosestIntersection();

    /// <summary>The classifier's intersector (OCCT returns it by reference: the classifier's own object).</summary>
    IntRes2d_Intersection Intersector();

    TopAbs_State State();

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
                                                                    in gp_Pnt2d theP,
                                                                    double theTol)
        where TheFaceExplorer : ITopClass_FaceExplorer<TheEdge>
        where FClass2d : ITopClass_FClass2d<TheEdge>
    {
        gp_Pnt2d aPoint = theP;
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

        gp_Lin2d aLine;
        double aParam;
        bool aIsValidSegment = theFexp.Segment(aPoint, out aLine, out aParam);
        TheEdge anEdge;
        TopAbs_Orientation anEdgeOri;
        int aClosestInd;
        IntRes2d_IntersectionPoint aPInter;
        TopAbs_State aState = TopAbs_UNKNOWN;
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
                            theFexp.CurrentEdge(out anEdge, out anEdgeOri);

                            if (anEdgeOri == TopAbs_FORWARD || anEdgeOri == TopAbs_REVERSED)
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
                                if (aState == TopAbs_ON)
                                {
                                    return;
                                }
                            }
                        }
                    }

                    // If we are out of the wire we stop.
                    aState = theClassifier.State();
                    if (aState == TopAbs_OUT)
                    {
                        return;
                    }
                }
            }

            if (!theClassifier.IsHeadOrEnd() && aState != TopAbs_UNKNOWN)
            {
                break;
            }

            // Bad case for classification. Trying to get another segment.
            aIsValidSegment = theFexp.OtherSegment(aPoint, out aLine, out aParam);
        }
    }

    /// <summary>The state of the point: OUT if rejected, IN if the face has no wires, the classifier's otherwise.</summary>
    internal static TopAbs_State State<TheEdge>(bool theRejected, bool theNoWires,
                                                ITopClass_FClass2d<TheEdge> theClassifier)
    {
        if (theRejected)
        {
            return TopAbs_OUT;
        }
        else if (theNoWires)
        {
            return TopAbs_IN;
        }
        else
        {
            return theClassifier.State();
        }
    }

    /// <summary>The closest edge.</summary>
    /// <exception cref="Standard_DomainError">The point was rejected.</exception>
    internal static TheEdge Edge<TheEdge>(bool theRejected, in TheEdge theEdge)
    {
        Standard_DomainError_Raise_if(theRejected, "TopClass_FaceClassifier::Edge : rejected");
        return theEdge;
    }

    /// <summary>The parameter on the closest edge.</summary>
    /// <exception cref="Standard_DomainError">The point was rejected.</exception>
    internal static double EdgeParameter(bool theRejected, double theEdgeParameter)
    {
        Standard_DomainError_Raise_if(theRejected, "TopClass_FaceClassifier::EdgeParameter : rejected");
        return theEdgeParameter;
    }
}
