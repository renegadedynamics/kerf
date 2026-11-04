// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_NodeInsertionMeshAlgo.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Meshing.DegreeOfFreedom;
using static Kerf.Engine.Meshing.Model.StatusModel;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Extends the Watson base algorithm so that free vertices can be added to the mesh: the face's points are scaled
/// into the range splitter's basis, and the face's internal vertices are inserted when the parameters ask for it.
/// </summary>
/// <remarks>
/// OCCT's <c>BRepMesh_NodeInsertionMeshAlgo&lt;RangeSplitter, BaseAlgo&gt;</c> derives from its <c>BaseAlgo</c>
/// parameter, which C# can't. This generic over the range splitter derives from the Watson base
/// (<see cref="DelaunayBaseMeshAlgo"/>); <see cref="NodeInsertionMeshAlgoOfDelabella{RangeSplitter}"/>
/// is the same body over the Delabella base (phase4-plan section 6). The splitter's methods the subclasses redefine
/// are virtual, as in OCCT, so they dispatch through the generic as through the template.
/// </remarks>
internal class NodeInsertionMeshAlgo<RangeSplitter> : DelaunayBaseMeshAlgo
    where RangeSplitter : DefaultRangeSplitter, new()
{
    private readonly RangeSplitter _rangeSplitter = new();
    private Classifier? _classifier;

    /// <summary>An algorithm.</summary>
    public NodeInsertionMeshAlgo()
    {
    }

    /// <summary>Meshes the face.</summary>
    public override void Perform(FaceModel dFace,
                                 in ToolParameters parameters,
                                 ProgressRange range)
    {
        _rangeSplitter.Reset(dFace, parameters);
        _classifier = new Classifier();
        if (!range.More())
        {
            return;
        }
        base.Perform(dFace, parameters, range);
        _classifier = null;
    }

    /// <summary>Initializes the data structure from the model's data.</summary>
    protected override bool InitDataStructure()
    {
        FaceModel dFace = GetDFace();
        NCollection_Array1<NCollection_Sequence<Point2D>?> wires = new(0, dFace.WiresNb() - 1);
        for (int wireIt = 0; wireIt < dFace.WiresNb(); ++wireIt)
        {
            WireModel dWire = dFace.GetWire(wireIt);
            if (dWire.IsSet(StatusModel.SelfIntersectingWire)
                || (dWire.IsSet(StatusModel.OpenWire) && wireIt != 0))
            {
                continue;
            }

            wires[wireIt] = CollectWirePoints(dWire);
        }

        _rangeSplitter.AdjustRange();
        if (!_rangeSplitter.IsValid)
        {
            dFace.SetStatus(StatusModel.Failure);
            return false;
        }

        (double first, double second) delta = _rangeSplitter.GetDelta();
        (double first, double second) tolUV = _rangeSplitter.GetToleranceUV();
        double uCellSize = 14.0 * tolUV.first;
        double vCellSize = 14.0 * tolUV.second;

        GetStructure().Data().SetCellSize(uCellSize / delta.first, vCellSize / delta.second);
        GetStructure().Data().SetTolerance(tolUV.first / delta.first,
                                           tolUV.second / delta.second);

        for (int wireIt = 0; wireIt < dFace.WiresNb(); ++wireIt)
        {
            NCollection_Sequence<Point2D>? wire = wires[wireIt];
            if (wire is not null && !wire.IsEmpty())
            {
                _classifier!.RegisterWire(wire,
                                           tolUV,
                                           _rangeSplitter.GetRangeU(),
                                           _rangeSplitter.GetRangeV());
            }
        }

        if (GetParameters().InternalVerticesMode)
        {
            InsertInternalVertices();
        }

        return base.InitDataStructure();
    }

    /// <summary>Adds the 2D point, scaled into the splitter's basis, to the mesh structure.</summary>
    protected override int AddNodeToStructure(in Point2D point,
                                              int location3d,
                                              DegreeOfFreedom movability,
                                              bool isForceAdd)
    {
        return base.AddNodeToStructure(_rangeSplitter.Scale(point, true),
                                       location3d,
                                       movability,
                                       isForceAdd);
    }

    /// <summary>The 2D point of the vertex, scaled back from the splitter's basis.</summary>
    protected override Point2D GetNodePoint2d(in MeshVertex vertex)
    {
        return _rangeSplitter.Scale(new Point2D(vertex.Coord()), false);
    }

    /// <summary>The range splitter.</summary>
    protected RangeSplitter GetRangeSplitter() => _rangeSplitter;

    /// <summary>The classifier.</summary>
    protected Classifier GetClassifier() => _classifier!;

    // Creates collection of points representing discrete wire (OCCT keeps pointers to the pcurves' points; the
    // points aren't changed while the face is meshed, so their values are kept).
    private NCollection_Sequence<Point2D> CollectWirePoints(WireModel dWire)
    {
        NCollection_Sequence<Point2D> wirePoints = new();
        for (int edgeIt = 0; edgeIt < dWire.EdgesNb(); ++edgeIt)
        {
            EdgeModel dEdge = dWire.GetEdge(edgeIt);
            PCurveModel pCurve = dEdge.GetPCurve(GetDFace(), dWire.GetEdgeOrientation(edgeIt));

            int pointIt, endIndex, inc;
            if (pCurve.IsForward)
            {
                // For an infinite cylinder (for example)
                // aPCurve->ParametersNb() == 0

                endIndex = pCurve.ParametersNb() - 1;
                pointIt = Min(0, endIndex);
                inc = 1;
            }
            else
            {
                // For an infinite cylinder (for example)
                // aPCurve->ParametersNb() == 0

                pointIt = pCurve.ParametersNb() - 1;
                endIndex = Min(0, pointIt);
                inc = -1;
            }

            // For an infinite cylinder (for example)
            // this cycle will not be executed.
            for (; pointIt != endIndex; pointIt += inc)
            {
                Point2D pnt2d = pCurve.GetPoint(pointIt);
                wirePoints.Append(pnt2d);
                _rangeSplitter.AddPoint(pnt2d);
            }
        }

        return wirePoints;
    }

    // Iterates over internal vertices of a face and creates corresponding nodes in data structure.
    private void InsertInternalVertices()
    {
        Explorer explorer = new(GetDFace().GetFace(), ShapeType.Vertex, ShapeType.Edge);
        for (; explorer.More(); explorer.Next())
        {
            Vertex vertex = ShapeCasts.Vertex(explorer.Current());
            if (vertex.Orientation != Orientation.Internal)
            {
                continue;
            }

            InsertInternalVertex(vertex);
        }
    }

    // Inserts the given vertex into mesh.
    private void InsertInternalVertex(in Vertex vertex)
    {
        try
        {
            Point2D pnt2d = Tool.Parameters(vertex, GetDFace().GetFace());
            // check UV values for internal vertices
            if (_classifier!.Perform(pnt2d) != In)
            {
                return;
            }

            RegisterNode(Tool.Pnt(vertex), pnt2d, DegreeOfFreedom.Fixed, false);
        }
        catch (Exception exception) when (ModelAlgo.IsCaughtFailure(exception))
        {
        }
    }
}
