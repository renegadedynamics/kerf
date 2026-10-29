// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_DelaunayDeflectionControlMeshAlgo.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepMesh_DegreeOfFreedom;

namespace Occt;

/// <summary>
/// Extends the Delaunay node-insertion algorithm to control the deflection of the generated triangles from the
/// surface: triangles failing the check are split, for up to 11 passes.
/// </summary>
/// <remarks>
/// The Watson instantiation of OCCT's <c>BRepMesh_DelaunayDeflectionControlMeshAlgo&lt;RangeSplitter, BaseAlgo&gt;</c>;
/// <see cref="BRepMesh_DelaunayDeflectionControlMeshAlgoOfDelabella{RangeSplitter}"/> is the Delabella one
/// (phase4-plan section 6). OCCT's deviation functors are private structs, and its template <c>usePoint</c> a generic
/// method over them.
/// </remarks>
public class BRepMesh_DelaunayDeflectionControlMeshAlgo<RangeSplitter> : BRepMesh_DelaunayNodeInsertionMeshAlgo<RangeSplitter>
    where RangeSplitter : BRepMesh_DefaultRangeSplitter, new()
{
    private double myMaxSqDeflection;
    private double mySqMinSize;
    private bool myIsAllDegenerated;
    private NCollection_Map<BRepMesh_OrientedEdge>? myCouplesMap;
    private NCollection_List<gp_Pnt2d>? myControlNodes;
    private BRepMesh_CircleTool? myCircles;

    /// <summary>An algorithm.</summary>
    public BRepMesh_DelaunayDeflectionControlMeshAlgo()
    {
        myMaxSqDeflection = -1.0;
        mySqMinSize = -1.0;
        myIsAllDegenerated = false;
        myCircles = null;
    }

    /// <summary>Processes the generated mesh: inserts the surface nodes, then controls the deflection.</summary>
    protected override void postProcessMesh(BRepMesh_Delaun theMesher, Message_ProgressRange theRange)
    {
        using Message_ProgressScope aPS = new(theRange, "Post process mesh", 2);
        // Insert surface nodes.
        base.postProcessMesh(theMesher, aPS.Next());
        if (!aPS.More())
        {
            return;
        }

        if (getParameters().ControlSurfaceDeflection
            && getStructure().ElementsOfDomain().Extent() > 0)
        {
            optimizeMesh(theMesher, aPS.Next());
        }
        else
        {
            aPS.Next();
        }
    }

    /// <summary>Checks the deviation of the mesh from the surface and inserts nodes where it is too large.</summary>
    protected virtual void optimizeMesh(BRepMesh_Delaun theMesher, Message_ProgressRange theRange)
    {
        mySqMinSize = getParameters().MinSize * getParameters().MinSize;
        myCouplesMap = new NCollection_Map<BRepMesh_OrientedEdge>(); // OCCT preallocates 3 * ElementsOfDomain().Extent() buckets
        myControlNodes = new NCollection_List<gp_Pnt2d>();
        myCircles = theMesher.Circles();

        const int aIterationsNb = 11;
        bool isInserted = true;
        using Message_ProgressScope aPS = new(theRange, "Iteration", aIterationsNb);
        for (int aPass = 1; aPass <= aIterationsNb && isInserted && !myIsAllDegenerated; ++aPass)
        {
            if (!aPS.More())
            {
                return;
            }
            // Reset stop condition
            myMaxSqDeflection = -1.0;
            myIsAllDegenerated = true;
            myControlNodes.Clear();

            if (getStructure().ElementsOfDomain().Extent() < 1)
            {
                break;
            }
            // Iterate on current triangles
            NCollection_PackedMapOfInteger.Iterator aTriangleIt = new(getStructure().ElementsOfDomain());
            for (; aTriangleIt.More(); aTriangleIt.Next())
            {
                BRepMesh_Triangle aTriangle = getStructure().GetElement(aTriangleIt.Key());
                splitTriangleGeometry(aTriangle);
            }

            isInserted = insertNodes(myControlNodes, theMesher, aPS.Next());
        }

        myCouplesMap = null;
        myControlNodes = null;

        if (!(myMaxSqDeflection < 0.0))
        {
            getDFace().SetDeflection(Math.Sqrt(myMaxSqDeflection));
        }
    }

    // Contains geometrical data related to node of triangle.
    private struct TriangleNodeInfo
    {
        public gp_XY Point2d;
        public gp_XYZ Point;
        public bool isFrontierLink;
    }

    // A deviation functor.
    private interface IDeviation
    {
        double SquareDeviation(in gp_Pnt thePoint);
    }

    // Functor computing deflection of a point from surface.
    private readonly struct NormalDeviation(in gp_Pnt theRefPnt, in gp_Vec theNormal) : IDeviation
    {
        private readonly gp_Pnt myRefPnt = theRefPnt;
        private readonly gp_Vec myNormal = theNormal;

        public double SquareDeviation(in gp_Pnt thePoint)
        {
            double aDeflection = Math.Abs(myNormal.Dot(new gp_Vec(myRefPnt, thePoint)));
            return aDeflection * aDeflection;
        }
    }

    // Functor computing deflection of a point on triangle link from surface.
    private readonly struct LineDeviation(in gp_Pnt thePnt1, in gp_Pnt thePnt2) : IDeviation
    {
        private readonly gp_Pnt myPnt1 = thePnt1;
        private readonly gp_Pnt myPnt2 = thePnt2;

        public double SquareDeviation(in gp_Pnt thePoint)
        {
            return BRepMesh_GeomTool.SquareDeflectionOfSegment(myPnt1, myPnt2, thePoint);
        }
    }

    // Returns nodes info of the given triangle.
    private void getTriangleInfo(in BRepMesh_Triangle theTriangle,
                                 ReadOnlySpan<int> theNodesIndices,
                                 Span<TriangleNodeInfo> theInfo)
    {
        var e = theTriangle.myEdges;
        for (int i = 0; i < 3; ++i)
        {
            BRepMesh_Vertex aVertex = getStructure().GetNode(theNodesIndices[i]);
            theInfo[i].Point2d = getRangeSplitter().Scale(new gp_Pnt2d(aVertex.Coord()), false).XY();
            theInfo[i].Point = getNodesMap().Value(aVertex.Location3d()).XYZ();
            theInfo[i].isFrontierLink =
                (getStructure().GetLink(e[i]).Movability() == BRepMesh_Frontier);
        }
    }

    // Check geometry of the given triangle. If triangle does not suit specified deflection, inserts new point.
    private void splitTriangleGeometry(in BRepMesh_Triangle theTriangle)
    {
        if (theTriangle.Movability() != BRepMesh_Deleted)
        {
            Span<int> aNodexIndices = stackalloc int[3];
            getStructure().ElementNodes(theTriangle, aNodexIndices);

            Span<TriangleNodeInfo> aNodesInfo = stackalloc TriangleNodeInfo[3];
            getTriangleInfo(theTriangle, aNodexIndices, aNodesInfo);

            gp_Vec aNormal = new();
            Span<gp_Vec> aLinkVec = stackalloc gp_Vec[3];
            if (computeTriangleGeometry(aNodesInfo, aLinkVec, ref aNormal))
            {
                myIsAllDegenerated = false;

                gp_XY aCenter2d =
                    (aNodesInfo[0].Point2d + aNodesInfo[1].Point2d + aNodesInfo[2].Point2d) / 3.0;

                usePoint(aCenter2d, new NormalDeviation(new gp_Pnt(aNodesInfo[0].Point), aNormal));
                splitLinks(aNodesInfo, aNodexIndices);
            }
        }
    }

    // Updates array of links vectors. Returns false on degenerative triangle.
    private bool computeTriangleGeometry(ReadOnlySpan<TriangleNodeInfo> theNodesInfo,
                                         Span<gp_Vec> theLinks,
                                         ref gp_Vec theNormal)
    {
        if (checkTriangleForDegenerativityAndGetLinks(theNodesInfo, theLinks))
        {
            if (checkTriangleArea2d(theNodesInfo))
            {
                if (computeNormal(theLinks[0], theLinks[1], ref theNormal))
                {
                    return true;
                }
            }
        }

        return false;
    }

    // Updates array of links vectors. Returns false on degenerative triangle.
    private static bool checkTriangleForDegenerativityAndGetLinks(ReadOnlySpan<TriangleNodeInfo> theNodesInfo,
                                                                  Span<gp_Vec> theLinks)
    {
        const double MinimalSqLength3d = 1.0e-12;
        for (int i = 0; i < 3; ++i)
        {
            theLinks[i] = new gp_Vec(theNodesInfo[(i + 1) % 3].Point - theNodesInfo[i].Point);
            if (theLinks[i].SquareMagnitude() < MinimalSqLength3d)
            {
                return false;
            }
        }

        return true;
    }

    // Checks area of triangle in parametric space for degenerativity. Returns false on degenerative triangle.
    private static bool checkTriangleArea2d(ReadOnlySpan<TriangleNodeInfo> theNodesInfo)
    {
        gp_Vec2d aLink2d1 = new(new gp_Pnt2d(theNodesInfo[0].Point2d), new gp_Pnt2d(theNodesInfo[1].Point2d));
        gp_Vec2d aLink2d2 = new(new gp_Pnt2d(theNodesInfo[1].Point2d), new gp_Pnt2d(theNodesInfo[2].Point2d));

        const double MinimalArea2d = 1.0e-9;
        return (Math.Abs(aLink2d1 ^ aLink2d2) > MinimalArea2d);
    }

    // Computes normal using two link vectors. Returns true on success, false in case of normal of null magnitude.
    private static bool computeNormal(in gp_Vec theLink1, in gp_Vec theLink2, ref gp_Vec theNormal)
    {
        gp_Vec aNormal = theLink1 ^ theLink2;
        if (aNormal.SquareMagnitude() > gp.Resolution())
        {
            theNormal = aNormal.Normalized();
            return true;
        }

        return false;
    }

    // Computes deflection of midpoints of triangles links.
    private void splitLinks(ReadOnlySpan<TriangleNodeInfo> theNodesInfo, ReadOnlySpan<int> theNodesIndices)
    {
        // Check deflection at triangle links
        for (int i = 0; i < 3; ++i)
        {
            if (theNodesInfo[i].isFrontierLink)
            {
                continue;
            }

            int j = (i + 1) % 3;
            // Check if this link was already processed
            int aFirstVertex, aLastVertex;
            if (theNodesIndices[i] < theNodesIndices[j])
            {
                aFirstVertex = theNodesIndices[i];
                aLastVertex = theNodesIndices[j];
            }
            else
            {
                aFirstVertex = theNodesIndices[j];
                aLastVertex = theNodesIndices[i];
            }

            if (myCouplesMap!.Add(new BRepMesh_OrientedEdge(aFirstVertex, aLastVertex)))
            {
                gp_XY aMidPnt2d = (theNodesInfo[i].Point2d + theNodesInfo[j].Point2d) / 2.0;

                if (!usePoint(aMidPnt2d, new LineDeviation(new gp_Pnt(theNodesInfo[i].Point), new gp_Pnt(theNodesInfo[j].Point))))
                {
                    if (!rejectSplitLinksForMinSize(theNodesInfo[i], theNodesInfo[j], aMidPnt2d))
                    {
                        if (!checkLinkEndsForAngularDeviation(theNodesInfo[i], theNodesInfo[j], aMidPnt2d))
                        {
                            myControlNodes!.Append(new gp_Pnt2d(aMidPnt2d));
                        }
                    }
                }
            }
        }
    }

    // Checks that two links produced as the result of a split of the given link by the middle point fit MinSize
    // requirement.
    private bool rejectSplitLinksForMinSize(in TriangleNodeInfo theNodeInfo1,
                                            in TriangleNodeInfo theNodeInfo2,
                                            in gp_XY theMidPoint)
    {
        gp_Pnt aPnt = getPoint3d(theMidPoint);
        return ((theNodeInfo1.Point - aPnt.XYZ()).SquareModulus() < mySqMinSize
                || (theNodeInfo2.Point - aPnt.XYZ()).SquareModulus() < mySqMinSize);
    }

    // Checks the given point (located between the given nodes) for specified angular deviation.
    private bool checkLinkEndsForAngularDeviation(in TriangleNodeInfo theNodeInfo1,
                                                  in TriangleNodeInfo theNodeInfo2,
                                                  in gp_XY theMidPoint)
    {
        _ = theMidPoint;
        gp_Dir aNorm1 = new(), aNorm2 = new();
        Geom_Surface aSurf = getDFace().GetSurface().GeomSurfaceOriginal()!;

        if ((GeomLib.NormEstim(aSurf, new gp_Pnt2d(theNodeInfo1.Point2d), Precision.Confusion(), ref aNorm1) == 0)
            && (GeomLib.NormEstim(aSurf, new gp_Pnt2d(theNodeInfo2.Point2d), Precision.Confusion(), ref aNorm2) == 0))
        {
            double anAngle = aNorm1.Angle(aNorm2);
            if (anAngle > getParameters().AngleInterior)
            {
                return false;
            }
        }

        return true;
    }

    // Returns 3d point corresponding to the given one in 2d space.
    private gp_Pnt getPoint3d(in gp_XY thePnt2d)
    {
        getDFace().GetSurface().D0(thePnt2d.X(), thePnt2d.Y(), out gp_Pnt aPnt);
        return aPnt;
    }

    // Computes deflection of the given point and caches it for insertion in case if it overflows deflection.
    // Returns true if point has been cached for insertion.
    private bool usePoint<DeflectionFunctor>(in gp_XY thePnt2d, in DeflectionFunctor theDeflectionFunctor)
        where DeflectionFunctor : struct, IDeviation
    {
        gp_Pnt aPnt = getPoint3d(thePnt2d);
        if (!checkDeflectionOfPointAndUpdateCache(thePnt2d,
                                                  aPnt,
                                                  theDeflectionFunctor.SquareDeviation(aPnt)))
        {
            myControlNodes!.Append(new gp_Pnt2d(thePnt2d));
            return true;
        }

        return false;
    }

    // Checks the given point for specified linear deflection. Updates value of total mesh deflection.
    private bool checkDeflectionOfPointAndUpdateCache(in gp_XY thePnt2d, in gp_Pnt thePnt3d, double theSqDeflection)
    {
        if (theSqDeflection > myMaxSqDeflection)
        {
            myMaxSqDeflection = theSqDeflection;
        }

        double aSqDeflection = getDFace().GetDeflection() * getDFace().GetDeflection();
        if (theSqDeflection < aSqDeflection)
        {
            return true;
        }

        return rejectByMinSize(thePnt2d, thePnt3d);
    }

    // Checks distance between the given node and nodes of triangles shot by it for MinSize criteria.
    // This check is expected to roughly estimate and prevent generation of triangles with sides smaller than MinSize.
    private bool rejectByMinSize(in gp_XY thePnt2d, in gp_Pnt thePnt3d)
    {
        NCollection_PackedMapOfInteger aUsedNodes = new();
        NCollection_List<int> aCirclesList =
            myCircles!.Select(getRangeSplitter().Scale(new gp_Pnt2d(thePnt2d), true).XY());

        Span<int> aNodes = stackalloc int[3];
        NCollection_List<int>.Iterator aCircleIt = new(aCirclesList);
        for (; aCircleIt.More(); aCircleIt.Next())
        {
            BRepMesh_Triangle aTriangle = getStructure().GetElement(aCircleIt.Value());

            getStructure().ElementNodes(aTriangle, aNodes);

            for (int i = 0; i < 3; ++i)
            {
                if (!aUsedNodes.Contains(aNodes[i]))
                {
                    aUsedNodes.Add(aNodes[i]);
                    BRepMesh_Vertex aVertex = getStructure().GetNode(aNodes[i]);
                    gp_Pnt aPoint = getNodesMap().Value(aVertex.Location3d());

                    if (thePnt3d.SquareDistance(aPoint) < mySqMinSize)
                    {
                        return true;
                    }
                }
            }
        }

        return false;
    }
}
