// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_NodeInsertionMeshAlgo.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepMesh_DegreeOfFreedom;
using static Occt.IMeshData_Status;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Extends the Delabella base algorithm so that free vertices can be added to the mesh: the face's points are scaled
/// into the range splitter's basis, and the face's internal vertices are inserted when the parameters ask for it.
/// </summary>
/// <remarks>
/// The Delabella instantiation of the OCCT template (over
/// <c>BRepMesh_CustomDelaunayBaseMeshAlgo&lt;BRepMesh_DelabellaBaseMeshAlgo&gt;</c>), written by a script from the
/// Watson generic's body with only the names changed (phase4-plan section 6, decision 202's approach).
/// </remarks>
public class BRepMesh_NodeInsertionMeshAlgoOfDelabella<RangeSplitter> : BRepMesh_CustomDelaunayBaseMeshAlgoOfDelabella
    where RangeSplitter : BRepMesh_DefaultRangeSplitter, new()
{
    private readonly RangeSplitter myRangeSplitter = new();
    private BRepMesh_Classifier? myClassifier;

    /// <summary>An algorithm.</summary>
    public BRepMesh_NodeInsertionMeshAlgoOfDelabella()
    {
    }

    /// <summary>Meshes the face.</summary>
    public override void Perform(IMeshData_Face theDFace,
                                 in IMeshTools_Parameters theParameters,
                                 Message_ProgressRange theRange)
    {
        myRangeSplitter.Reset(theDFace, theParameters);
        myClassifier = new BRepMesh_Classifier();
        if (!theRange.More())
        {
            return;
        }
        base.Perform(theDFace, theParameters, theRange);
        myClassifier = null;
    }

    /// <summary>Initializes the data structure from the model's data.</summary>
    protected override bool initDataStructure()
    {
        IMeshData_Face aDFace = getDFace();
        NCollection_Array1<NCollection_Sequence<gp_Pnt2d>?> aWires = new(0, aDFace.WiresNb() - 1);
        for (int aWireIt = 0; aWireIt < aDFace.WiresNb(); ++aWireIt)
        {
            IMeshData_Wire aDWire = aDFace.GetWire(aWireIt);
            if (aDWire.IsSet(IMeshData_SelfIntersectingWire)
                || (aDWire.IsSet(IMeshData_OpenWire) && aWireIt != 0))
            {
                continue;
            }

            aWires[aWireIt] = collectWirePoints(aDWire);
        }

        myRangeSplitter.AdjustRange();
        if (!myRangeSplitter.IsValid())
        {
            aDFace.SetStatus(IMeshData_Failure);
            return false;
        }

        (double first, double second) aDelta = myRangeSplitter.GetDelta();
        (double first, double second) aTolUV = myRangeSplitter.GetToleranceUV();
        double uCellSize = 14.0 * aTolUV.first;
        double vCellSize = 14.0 * aTolUV.second;

        getStructure().Data().SetCellSize(uCellSize / aDelta.first, vCellSize / aDelta.second);
        getStructure().Data().SetTolerance(aTolUV.first / aDelta.first,
                                           aTolUV.second / aDelta.second);

        for (int aWireIt = 0; aWireIt < aDFace.WiresNb(); ++aWireIt)
        {
            NCollection_Sequence<gp_Pnt2d>? aWire = aWires[aWireIt];
            if (aWire is not null && !aWire.IsEmpty())
            {
                myClassifier!.RegisterWire(aWire,
                                           aTolUV,
                                           myRangeSplitter.GetRangeU(),
                                           myRangeSplitter.GetRangeV());
            }
        }

        if (getParameters().InternalVerticesMode)
        {
            insertInternalVertices();
        }

        return base.initDataStructure();
    }

    /// <summary>Adds the 2D point, scaled into the splitter's basis, to the mesh structure.</summary>
    protected override int addNodeToStructure(in gp_Pnt2d thePoint,
                                              int theLocation3d,
                                              BRepMesh_DegreeOfFreedom theMovability,
                                              bool isForceAdd)
    {
        return base.addNodeToStructure(myRangeSplitter.Scale(thePoint, true),
                                       theLocation3d,
                                       theMovability,
                                       isForceAdd);
    }

    /// <summary>The 2D point of the vertex, scaled back from the splitter's basis.</summary>
    protected override gp_Pnt2d getNodePoint2d(in BRepMesh_Vertex theVertex)
    {
        return myRangeSplitter.Scale(new gp_Pnt2d(theVertex.Coord()), false);
    }

    /// <summary>The range splitter.</summary>
    protected RangeSplitter getRangeSplitter() => myRangeSplitter;

    /// <summary>The classifier.</summary>
    protected BRepMesh_Classifier getClassifier() => myClassifier!;

    // Creates collection of points representing discrete wire (OCCT keeps pointers to the pcurves' points; the
    // points aren't changed while the face is meshed, so their values are kept).
    private NCollection_Sequence<gp_Pnt2d> collectWirePoints(IMeshData_Wire theDWire)
    {
        NCollection_Sequence<gp_Pnt2d> aWirePoints = new();
        for (int aEdgeIt = 0; aEdgeIt < theDWire.EdgesNb(); ++aEdgeIt)
        {
            IMeshData_Edge aDEdge = theDWire.GetEdge(aEdgeIt);
            IMeshData_PCurve aPCurve = aDEdge.GetPCurve(getDFace(), theDWire.GetEdgeOrientation(aEdgeIt));

            int aPointIt, aEndIndex, aInc;
            if (aPCurve.IsForward())
            {
                // For an infinite cylinder (for example)
                // aPCurve->ParametersNb() == 0

                aEndIndex = aPCurve.ParametersNb() - 1;
                aPointIt = Min(0, aEndIndex);
                aInc = 1;
            }
            else
            {
                // For an infinite cylinder (for example)
                // aPCurve->ParametersNb() == 0

                aPointIt = aPCurve.ParametersNb() - 1;
                aEndIndex = Min(0, aPointIt);
                aInc = -1;
            }

            // For an infinite cylinder (for example)
            // this cycle will not be executed.
            for (; aPointIt != aEndIndex; aPointIt += aInc)
            {
                gp_Pnt2d aPnt2d = aPCurve.GetPoint(aPointIt);
                aWirePoints.Append(aPnt2d);
                myRangeSplitter.AddPoint(aPnt2d);
            }
        }

        return aWirePoints;
    }

    // Iterates over internal vertices of a face and creates corresponding nodes in data structure.
    private void insertInternalVertices()
    {
        TopExp_Explorer aExplorer = new(getDFace().GetFace(), TopAbs_ShapeEnum.TopAbs_VERTEX, TopAbs_ShapeEnum.TopAbs_EDGE);
        for (; aExplorer.More(); aExplorer.Next())
        {
            TopoDS_Vertex aVertex = TopoDS.Vertex(aExplorer.Current());
            if (aVertex.Orientation() != TopAbs_Orientation.TopAbs_INTERNAL)
            {
                continue;
            }

            insertInternalVertex(aVertex);
        }
    }

    // Inserts the given vertex into mesh.
    private void insertInternalVertex(in TopoDS_Vertex theVertex)
    {
        try
        {
            gp_Pnt2d aPnt2d = BRep_Tool.Parameters(theVertex, getDFace().GetFace());
            // check UV values for internal vertices
            if (myClassifier!.Perform(aPnt2d) != TopAbs_IN)
            {
                return;
            }

            registerNode(BRep_Tool.Pnt(theVertex), aPnt2d, BRepMesh_Fixed, false);
        }
        catch (Exception anException) when (IMeshTools_ModelAlgo.IsCaughtFailure(anException))
        {
        }
    }
}
