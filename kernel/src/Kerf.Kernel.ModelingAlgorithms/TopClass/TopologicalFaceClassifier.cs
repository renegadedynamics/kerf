// Port of: src/ModelingAlgorithms/TKGeomAlgo/TopClass/TopClass_FaceClassifier.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Classification.Topological;

/// <summary>
/// The face explorer of a face classifier, as <see cref="TopologicalFaceClassifier.Perform"/> uses it (OCCT's
/// template parameter TheFaceExplorer: BRepClass_FaceExplorer, Geom2dHatch_Elements).
/// </summary>
internal interface IFaceExplorer<TheEdge>
{
    /// <summary>Checks the point and moves it if needed; returns false to be called again.</summary>
    bool CheckPoint(ref Point2D point);

    /// <summary>True if the point is certainly outside the face.</summary>
    bool Reject(in Point2D p);

    /// <summary>A half-line from <paramref name="p"/> and the parameter to intersect it to; false if it is not valid.</summary>
    bool Segment(in Point2D p, out Line2D l, out double par);

    /// <summary>Another half-line from <paramref name="p"/>, after a bad classification; false if none is valid.</summary>
    bool OtherSegment(in Point2D p, out Line2D l, out double par);

    void InitWires();

    bool MoreWires();

    void NextWire();

    /// <summary>True if the current wire can't meet the segment of <paramref name="l"/> up to <paramref name="par"/>.</summary>
    bool RejectWire(in Line2D l, double par);

    void InitEdges();

    bool MoreEdges();

    void NextEdge();

    /// <summary>True if the current edge can't meet the segment of <paramref name="l"/> up to <paramref name="par"/>.</summary>
    bool RejectEdge(in Line2D l, double par);

    /// <summary>
    /// Writes the current edge into <paramref name="e"/>, with its orientation. The edge is <c>ref</c>, as OCCT's
    /// <c>TheEdge&amp;</c>: the classifier passes the same variable for every edge, so what an explorer leaves
    /// unwritten carries over from the previous edge (BRepClass_FaceExplorer's next edge does). If TheEdge is a
    /// class, each call must give a new object, since the classifier keeps the closest one.
    /// </summary>
    void CurrentEdge(ref TheEdge e, out Orientation or);
}

/// <summary>
/// The 2D classifier of a face classifier, as <see cref="TopologicalFaceClassifier.Perform"/> uses it (OCCT's
/// template parameter FClass2d: BRepClass_FClass2dOfFClassifier, Geom2dHatch_FClass2dOfClassifier).
/// </summary>
internal interface IFClass2d<TheEdge>
{
    void Reset(in Line2D l, double p, double tol);

    void Compare(in TheEdge e, Orientation or);

    double Parameter();

    int ClosestIntersection();

    /// <summary>The classifier's intersector (OCCT returns it by reference: the classifier's own object).</summary>
    Intersection2D Intersector();

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
internal static class TopologicalFaceClassifier
{
    /// <summary>
    /// Classifies <paramref name="p"/> in the face explored by <paramref name="fexp"/>; the state is then in
    /// <paramref name="classifier"/> (see <see cref="State"/>), and the closest edge, the position and the
    /// parameter on it in the other outputs.
    /// </summary>
    internal static void Perform<TheFaceExplorer, TheEdge, FClass2d>(out bool rejected,
                                                                    ref bool noWires,
                                                                    FClass2d classifier,
                                                                    ref TheEdge edge,
                                                                    ref Position2D position,
                                                                    ref double edgeParameter,
                                                                    TheFaceExplorer fexp,
                                                                    in Point2D p,
                                                                    double tol)
        where TheFaceExplorer : IFaceExplorer<TheEdge>
        where FClass2d : IFClass2d<TheEdge>
        where TheEdge : new()
    {
        Point2D point = p;
        bool resOfPointCheck = false;
        while (resOfPointCheck == false)
        {
            resOfPointCheck = fexp.CheckPoint(ref point);
        }

        // Test for rejection.
        rejected = fexp.Reject(point);
        if (rejected)
        {
            return;
        }

        Line2D line;
        double param;
        bool isValidSegment = fexp.Segment(point, out line, out param);
        // C++ default-constructs one edge for the whole classification (see ITopClass_FaceExplorer.CurrentEdge)
        TheEdge anEdge = new();
        Orientation edgeOri;
        int closestInd;
        IntersectionPoint2D pInter;
        Containment state = Unknown;
        bool isWReject;
        bool isEReject;

        noWires = true;

        while (isValidSegment)
        {
            classifier.Reset(line, param, tol);

            for (fexp.InitWires(); fexp.MoreWires(); fexp.NextWire())
            {
                noWires = false;
                isWReject = fexp.RejectWire(line, classifier.Parameter());

                if (!isWReject)
                {
                    // Test this wire.
                    for (fexp.InitEdges(); fexp.MoreEdges(); fexp.NextEdge())
                    {
                        isEReject = fexp.RejectEdge(line, classifier.Parameter());

                        if (!isEReject)
                        {
                            // Test this edge.
                            fexp.CurrentEdge(ref anEdge, out edgeOri);

                            if (edgeOri == Forward || edgeOri == Reversed)
                            {
                                classifier.Compare(anEdge, edgeOri);
                                closestInd = classifier.ClosestIntersection();

                                if (closestInd != 0)
                                {
                                    // Save the closest edge.
                                    Intersection2D intersector = classifier.Intersector();
                                    int nbPnts = intersector.PointCount;
                                    edge = anEdge;

                                    if (closestInd <= nbPnts)
                                    {
                                        pInter = intersector.Point(closestInd);
                                    }
                                    else
                                    {
                                        closestInd -= nbPnts;
                                        if ((closestInd & 1) != 0)
                                        {
                                            pInter = intersector.Segment((closestInd + 1) / 2).FirstPoint();
                                        }
                                        else
                                        {
                                            pInter = intersector.Segment((closestInd + 1) / 2).LastPoint();
                                        }
                                    }

                                    position = pInter.TransitionOfSecond().PositionOnCurve();
                                    edgeParameter = pInter.ParamOnSecond();
                                }

                                // If we are ON, we stop.
                                state = classifier.State();
                                if (state == On)
                                {
                                    return;
                                }
                            }
                        }
                    }

                    // If we are out of the wire we stop.
                    state = classifier.State();
                    if (state == Out)
                    {
                        return;
                    }
                }
            }

            if (!classifier.IsHeadOrEnd() && state != Unknown)
            {
                break;
            }

            // Bad case for classification. Trying to get another segment.
            isValidSegment = fexp.OtherSegment(point, out line, out param);
        }
    }

    /// <summary>The state of the point: OUT if rejected, IN if the face has no wires, the classifier's otherwise.</summary>
    internal static Containment State<TheEdge>(bool rejected, bool noWires,
                                                IFClass2d<TheEdge> classifier)
    {
        if (rejected)
        {
            return Out;
        }
        else if (noWires)
        {
            return In;
        }
        else
        {
            return classifier.State();
        }
    }

    /// <summary>The closest edge.</summary>
    /// <exception cref="DomainErrorException">The point was rejected.</exception>
    internal static TheEdge Edge<TheEdge>(bool rejected, in TheEdge edge)
    {
        Standard_DomainError_Raise_if(rejected, "TopClass_FaceClassifier::Edge : rejected");
        return edge;
    }

    /// <summary>The parameter on the closest edge.</summary>
    /// <exception cref="DomainErrorException">The point was rejected.</exception>
    internal static double EdgeParameter(bool rejected, double edgeParameter)
    {
        Standard_DomainError_Raise_if(rejected, "TopClass_FaceClassifier::EdgeParameter : rejected");
        return edgeParameter;
    }
}
