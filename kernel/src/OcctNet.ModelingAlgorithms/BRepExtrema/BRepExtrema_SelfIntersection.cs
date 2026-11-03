// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_SelfIntersection.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_SelfIntersection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using BVH_Vec3d = Occt.NCollection_Vec3<double>;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Detects self-intersections of a shape from its tessellation: pairs of overlapping triangles of different faces.
/// The result depends on the mesh (a mesh whose adjacent faces don't share vertices on their common edge gives false
/// overlaps), so this is a fast approximate test giving the faces that may overlap.
/// </summary>
/// <remarks>A working object (decision 21). It only reads the shape's triangulations.</remarks>
public class BRepExtrema_SelfIntersection : BRepExtrema_ElementFilter
{
    private double myTolerance;
    private bool myIsInit;
    private readonly NCollection_DynamicArray<Shape> myFaceList = new();
    private BRepExtrema_TriangleSet? myElementSet;
    private readonly BRepExtrema_OverlapTool myOverlapTool = new();

    /// <summary>An uninitialized self-intersection tool.</summary>
    public BRepExtrema_SelfIntersection(double theTolerance = 0.0)
    {
        myTolerance = theTolerance;
        myIsInit = false;
    }

    /// <summary>A self-intersection tool for the shape.</summary>
    public BRepExtrema_SelfIntersection(in Shape theShape, double theTolerance = 0.0)
    {
        myTolerance = theTolerance;
        LoadShape(theShape);
    }

    /// <summary>The tolerance of the self-intersection test.</summary>
    public double Tolerance() => myTolerance;

    /// <summary>Sets the tolerance of the self-intersection test.</summary>
    public void SetTolerance(double theTolerance) => myTolerance = theTolerance;

    /// <summary>True if the detection is completed.</summary>
    public bool IsDone() => myOverlapTool.IsDone();

    /// <summary>The overlapped faces (indices from 0), each with the faces it overlaps.</summary>
    public NCollection_DataMap<int, NCollection_PackedMapOfInteger> OverlapElements() => myOverlapTool.OverlapSubShapes1();

    /// <summary>The face of the shape with the given index (from 0).</summary>
    public Face GetSubShape(int theID) => ShapeCasts.Face(myFaceList.Value(theID));

    /// <summary>The set of all the face triangles of the shape.</summary>
    public BRepExtrema_TriangleSet? ElementSet() => myElementSet;

    /// <summary>Loads the shape.</summary>
    public bool LoadShape(in Shape theShape)
    {
        myFaceList.Clear();

        for (var anIter = new Explorer(theShape, ShapeType.Face); anIter.More(); anIter.Next())
        {
            myFaceList.Append(ShapeCasts.Face(anIter.Current()));
        }

        myElementSet ??= new BRepExtrema_TriangleSet();

        myIsInit = myElementSet.Init(myFaceList);

        if (myIsInit)
        {
            myOverlapTool.LoadTriangleSets(myElementSet, myElementSet);
        }

        return myIsInit;
    }

    private static readonly BVH_Vec3d ZERO_VEC = new(0.0, 0.0, 0.0);

    // =======================================================================
    // function : ccw
    // purpose  : Check if triple is in counterclockwise order
    // =======================================================================
    private static bool ccw(in BVH_Vec3d theVertex0, in BVH_Vec3d theVertex1, in BVH_Vec3d theVertex2, int theX, int theY)
    {
        double aSum =
            (theVertex1[theX] - theVertex0[theX]) * (theVertex1[theY] + theVertex0[theY])
            + (theVertex2[theX] - theVertex1[theX]) * (theVertex2[theY] + theVertex1[theY])
            + (theVertex0[theX] - theVertex2[theX]) * (theVertex0[theY] + theVertex2[theY]);

        return aSum < 0.0;
    }

    // =======================================================================
    // function : rayInsideAngle
    // purpose  : Check the given ray is inside the angle
    // =======================================================================
    private static bool rayInsideAngle(in BVH_Vec3d theDirec, in BVH_Vec3d theEdge0, in BVH_Vec3d theEdge1, int theX, int theY)
    {
        bool aCCW = ccw(ZERO_VEC, theEdge0, theEdge1, theX, theY);

        return ccw(ZERO_VEC, theEdge0, theDirec, theX, theY) == aCCW
               && ccw(ZERO_VEC, theDirec, theEdge1, theX, theY) == aCCW;
    }

    private static void getProjectionAxes(in BVH_Vec3d theNorm, out int theAxisX, out int theAxisY)
    {
        if (Math.Abs(theNorm[0]) > Math.Abs(theNorm[1]))
        {
            theAxisX = Math.Abs(theNorm[0]) > Math.Abs(theNorm[2]) ? 1 : 0;
            theAxisY = Math.Abs(theNorm[0]) > Math.Abs(theNorm[2]) ? 2 : 1;
        }
        else
        {
            theAxisX = Math.Abs(theNorm[1]) > Math.Abs(theNorm[2]) ? 0 : 0;
            theAxisY = Math.Abs(theNorm[1]) > Math.Abs(theNorm[2]) ? 2 : 1;
        }
    }

    /// <summary>Whether two triangles sharing only one vertex overlap there.</summary>
    protected FilterResult isRegularSharedVertex(in BVH_Vec3d theSharedVert,
                                                 in BVH_Vec3d theTrng0Vtxs1,
                                                 in BVH_Vec3d theTrng0Vtxs2,
                                                 in BVH_Vec3d theTrng1Vtxs1,
                                                 in BVH_Vec3d theTrng1Vtxs2)
    {
        ReadOnlySpan<BVH_Vec3d> aTrng0Edges = [(theTrng0Vtxs1 - theSharedVert).Normalized(),
                                               (theTrng0Vtxs2 - theSharedVert).Normalized()];

        ReadOnlySpan<BVH_Vec3d> aTrng1Edges = [(theTrng1Vtxs1 - theSharedVert).Normalized(),
                                               (theTrng1Vtxs2 - theSharedVert).Normalized()];

        BVH_Vec3d aTrng0Normal = BVH_Vec3d.Cross(aTrng0Edges[0], aTrng0Edges[1]);
        BVH_Vec3d aTrng1Normal = BVH_Vec3d.Cross(aTrng1Edges[0], aTrng1Edges[1]);

        BVH_Vec3d aCrossLine = BVH_Vec3d.Cross(aTrng0Normal, aTrng1Normal);

        int anX;
        int anY;

        if (aCrossLine.SquareModulus() < global::Kerf.Engine.Foundation.Precision.SquareConfusion()) // coplanar case
        {
            getProjectionAxes(aTrng0Normal, out anX, out anY);

            if (rayInsideAngle(aTrng1Edges[0], aTrng0Edges[0], aTrng0Edges[1], anX, anY)
                || rayInsideAngle(aTrng1Edges[1], aTrng0Edges[0], aTrng0Edges[1], anX, anY)
                || rayInsideAngle(aTrng0Edges[0], aTrng1Edges[0], aTrng1Edges[1], anX, anY)
                || rayInsideAngle(aTrng0Edges[1], aTrng1Edges[0], aTrng1Edges[1], anX, anY))
            {
                return FilterResult.Overlap;
            }

            return FilterResult.NoCheck;
        }
        else // shared line should lie outside at least one triangle
        {
            getProjectionAxes(aTrng0Normal, out anX, out anY);

            bool aPosOutTrgn0 = !rayInsideAngle(aCrossLine, aTrng0Edges[0], aTrng0Edges[1], anX, anY);

            bool aNegOutTrgn0 = !rayInsideAngle(-aCrossLine, aTrng0Edges[0], aTrng0Edges[1], anX, anY);

            // Standard_ASSERT_RAISE
            if (!(aPosOutTrgn0 || aNegOutTrgn0))
            {
                throw new ProgramErrorException("Failed to detect if shared vertex is regular or not");
            }

            if (aPosOutTrgn0 && aNegOutTrgn0)
            {
                return FilterResult.NoCheck;
            }

            getProjectionAxes(aTrng1Normal, out anX, out anY);

            bool aPosOutTrgn1 = !rayInsideAngle(aCrossLine, aTrng1Edges[0], aTrng1Edges[1], anX, anY);

            bool aNegOutTrgn1 = !rayInsideAngle(-aCrossLine, aTrng1Edges[0], aTrng1Edges[1], anX, anY);

            // Standard_ASSERT_RAISE
            if (!(aPosOutTrgn1 || aNegOutTrgn1))
            {
                throw new ProgramErrorException("Failed to detect if shared vertex is regular or not");
            }

            if (aPosOutTrgn1 && aNegOutTrgn1)
            {
                return FilterResult.NoCheck;
            }

            return (aPosOutTrgn0 || aPosOutTrgn1) && (aNegOutTrgn0 || aNegOutTrgn1)
                       ? FilterResult.NoCheck
                       : FilterResult.Overlap;
        }
    }

    /// <summary>Whether two triangles sharing only one edge overlap there.</summary>
    protected FilterResult isRegularSharedEdge(in BVH_Vec3d theTrng0Vtxs0,
                                               in BVH_Vec3d theTrng0Vtxs1,
                                               in BVH_Vec3d theTrng0Vtxs2,
                                               in BVH_Vec3d theTrng1Vtxs2)
    {
        BVH_Vec3d aSharedEdge = (theTrng0Vtxs1 - theTrng0Vtxs0).Normalized();

        ReadOnlySpan<BVH_Vec3d> aUniqueEdges = [(theTrng0Vtxs2 - theTrng0Vtxs0).Normalized(),
                                                (theTrng1Vtxs2 - theTrng0Vtxs0).Normalized()];

        BVH_Vec3d aTrng0Normal = BVH_Vec3d.Cross(aSharedEdge, aUniqueEdges[0]);
        BVH_Vec3d aTrng1Normal = BVH_Vec3d.Cross(aSharedEdge, aUniqueEdges[1]);

        BVH_Vec3d aCrossLine = BVH_Vec3d.Cross(aTrng0Normal, aTrng1Normal);

        if (aCrossLine.SquareModulus() > global::Kerf.Engine.Foundation.Precision.SquareConfusion()) // non-coplanar case
        {
            return FilterResult.NoCheck;
        }

        getProjectionAxes(aTrng0Normal, out int anX, out int anY);

        return ccw(ZERO_VEC, aSharedEdge, aUniqueEdges[0], anX, anY)
               != ccw(ZERO_VEC, aSharedEdge, aUniqueEdges[1], anX, anY)
                   ? FilterResult.NoCheck
                   : FilterResult.Overlap;
    }

    /// <summary>Filters out correct adjacent mesh elements.</summary>
    public override FilterResult PreCheckElements(int theIndex1, int theIndex2)
    {
        if (myElementSet!.GetFaceID(theIndex1) == myElementSet.GetFaceID(theIndex2))
        {
            return FilterResult.NoCheck; // triangles are from the same face
        }

        Span<BVH_Vec3d> aTrng0Vtxs = stackalloc BVH_Vec3d[3];
        Span<BVH_Vec3d> aTrng1Vtxs = stackalloc BVH_Vec3d[3];

        myElementSet.GetVertices(theIndex1, out aTrng0Vtxs[0], out aTrng0Vtxs[1], out aTrng0Vtxs[2]);
        myElementSet.GetVertices(theIndex2, out aTrng1Vtxs[0], out aTrng1Vtxs[1], out aTrng1Vtxs[2]);

        Span<(int first, int second)> aSharedVtxs = stackalloc (int, int)[3];
        int aSharedVtxsSize = 0;

        for (int aVertIdx1 = 0; aVertIdx1 < 3; ++aVertIdx1)
        {
            for (int aVertIdx2 = 0; aVertIdx2 < 3; ++aVertIdx2)
            {
                if ((aTrng0Vtxs[aVertIdx1] - aTrng1Vtxs[aVertIdx2]).SquareModulus() < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    aSharedVtxs[aSharedVtxsSize++] = (aVertIdx1, aVertIdx2);

                    break; // go to next vertex of the 1st triangle
                }
            }
        }

        if (aSharedVtxsSize == 2) // check shared edge
        {
            return isRegularSharedEdge(aTrng0Vtxs[aSharedVtxs[0].first],
                                       aTrng0Vtxs[aSharedVtxs[1].first],
                                       aTrng0Vtxs[3 - aSharedVtxs[0].first - aSharedVtxs[1].first],
                                       aTrng1Vtxs[3 - aSharedVtxs[0].second - aSharedVtxs[1].second]);
        }
        else if (aSharedVtxsSize == 1) // check shared vertex
        {
            (aTrng0Vtxs[0], aTrng0Vtxs[aSharedVtxs[0].first]) = (aTrng0Vtxs[aSharedVtxs[0].first], aTrng0Vtxs[0]);
            (aTrng1Vtxs[0], aTrng1Vtxs[aSharedVtxs[0].second]) = (aTrng1Vtxs[aSharedVtxs[0].second], aTrng1Vtxs[0]);

            return isRegularSharedVertex(aTrng0Vtxs[0], aTrng0Vtxs[1], aTrng0Vtxs[2], aTrng1Vtxs[1], aTrng1Vtxs[2]);
        }

        return FilterResult.DoCheck;
    }

    /// <summary>Detects the self-intersections (once: nothing is done again until the shape is reloaded).</summary>
    public void Perform()
    {
        if (!myIsInit || myOverlapTool.IsDone())
        {
            return;
        }

        myOverlapTool.SetElementFilter(this);

        myOverlapTool.Perform(myTolerance);
    }
}
