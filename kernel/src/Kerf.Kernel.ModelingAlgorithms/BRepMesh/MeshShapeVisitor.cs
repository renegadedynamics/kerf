// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ShapeVisitor.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ShapeVisitor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Meshing.Model;
using Kerf.Engine.Meshing.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Meshing.Model.StatusModel;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Meshing;

/// <summary>
/// Builds the discrete model of a shape by adding its faces and free edges: each face with its wires, the outer one
/// first, each wire's edges in the order ShapeAnalysis_Wire finds in the face's parametric space.
/// </summary>
/// <remarks>
/// Only reads the shape (ShapeAnalysis' checks work on frozen shapes, decision 515), except
/// <see cref="BrepTools.Update(in Face)"/>, which marks an unfrozen face checked after updating its UV points,
/// as in OCCT, and does nothing on a frozen face.
/// </remarks>
public class MeshShapeVisitor : ShapeVisitor
{
    private readonly ModelModel _model;
    private readonly Dictionary<Shape, int> _dEdgeMap;

    /// <summary>A visitor filling the model.</summary>
    public MeshShapeVisitor(ModelModel model)
    {
        _model = model;
        _dEdgeMap = new Dictionary<Shape, int>(ShapeHasher.Instance);
    }

    /// <summary>Adds the edge to the model, once.</summary>
    public override void Visit(in Edge edge)
    {
        if (!_dEdgeMap.ContainsKey(edge))
        {
            _model.AddEdge(edge);
            _dEdgeMap[edge] = _model.EdgesNb() - 1;
        }
    }

    /// <summary>
    /// Adds the face to the model with its wires, the outer one first. A face whose outer wire can't be ordered gets
    /// <see cref="Failure"/>; an inner wire that can't is skipped, with
    /// <see cref="UnorientedWire"/>.
    /// </summary>
    public override void Visit(in Face face)
    {
        BrepTools.Update(face);
        FaceModel dFace = _model.AddFace(face);

        // Outer wire should always be the first in the model.
        Wire outerWire = ShapeAnalysisAnalysis.OuterWire(face);
        if (!AddWire(outerWire, dFace))
        {
            dFace.SetStatus(StatusModel.Failure);
            return;
        }

        Explorer wireIt = new(face, ShapeType.Wire);
        for (; wireIt.More(); wireIt.Next())
        {
            Wire wire = ShapeCasts.Wire(wireIt.Current());
            if (wire.IsSame(outerWire))
            {
                continue;
            }

            if (!AddWire(wire, dFace))
            {
                // If there is a failure on internal wire, just skip it.
                // The most significant is an outer wire.
                dFace.SetStatus(StatusModel.UnorientedWire);
            }
        }
    }

    // Adds wire to face discrete model.
    private bool AddWire(in Wire wire, FaceModel dFace)
    {
        if (wire.IsNull)
        {
            return false;
        }

        ExtendWireData wireData = new(wire, true, false);
        WireAnalysis wireTool = new(wireData, dFace.GetFace(), global::Kerf.Engine.Foundation.Precision.Confusion());

        WireOrderAnalysis orderTool = new();
        wireTool.CheckOrder(orderTool, true, false);
        if (wireTool.LastCheckStatus(ExtendStatus.Fail))
        {
            return false;
        }

        if (wireTool.LastCheckStatus(ExtendStatus.DONE3))
        {
            dFace.SetStatus(StatusModel.UnorientedWire);
        }

        int edgesNb = orderTool.EdgeCount;
        if (edgesNb != wireData.EdgeCount)
        {
            return false;
        }

        WireModel dWire = dFace.AddWire(wire, edgesNb);
        for (int i = 1; i <= edgesNb; ++i)
        {
            int edgeIndex = orderTool.Ordered(i);
            Edge edge = wireData.Edge(edgeIndex);
            if (edge.Orientation != External)
            {
                EdgeModel dEdge = _model.GetEdge(_dEdgeMap.GetValue(edge));

                dEdge.AddPCurve(dFace, edge.Orientation);
                dWire.AddEdge(dEdge, edge.Orientation);
            }
        }

        return true;
    }
}
