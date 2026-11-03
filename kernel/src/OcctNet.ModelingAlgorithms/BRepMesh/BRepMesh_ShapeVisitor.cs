// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ShapeVisitor.hxx
// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_ShapeVisitor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IMeshData_Status;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Builds the discrete model of a shape by adding its faces and free edges: each face with its wires, the outer one
/// first, each wire's edges in the order ShapeAnalysis_Wire finds in the face's parametric space.
/// </summary>
/// <remarks>
/// Only reads the shape (ShapeAnalysis' checks work on frozen shapes, decision 515), except
/// <see cref="BRepTools.Update(in TopoDS_Face)"/>, which marks an unfrozen face checked after updating its UV points,
/// as in OCCT, and does nothing on a frozen face.
/// </remarks>
public class BRepMesh_ShapeVisitor : IMeshTools_ShapeVisitor
{
    private readonly IMeshData_Model myModel;
    private readonly NCollection_DataMap<TopoDS_Shape, int> myDEdgeMap;

    /// <summary>A visitor filling the model.</summary>
    public BRepMesh_ShapeVisitor(IMeshData_Model theModel)
    {
        myModel = theModel;
        myDEdgeMap = new NCollection_DataMap<TopoDS_Shape, int>(TopTools_ShapeMapHasher.Instance);
    }

    /// <summary>Adds the edge to the model, once.</summary>
    public override void Visit(in TopoDS_Edge theEdge)
    {
        if (!myDEdgeMap.IsBound(theEdge))
        {
            myModel.AddEdge(theEdge);
            myDEdgeMap.Bind(theEdge, myModel.EdgesNb() - 1);
        }
    }

    /// <summary>
    /// Adds the face to the model with its wires, the outer one first. A face whose outer wire can't be ordered gets
    /// <see cref="IMeshData_Failure"/>; an inner wire that can't is skipped, with
    /// <see cref="IMeshData_UnorientedWire"/>.
    /// </summary>
    public override void Visit(in TopoDS_Face theFace)
    {
        BRepTools.Update(theFace);
        IMeshData_Face aDFace = myModel.AddFace(theFace);

        // Outer wire should always be the first in the model.
        TopoDS_Wire aOuterWire = ShapeAnalysis.OuterWire(theFace);
        if (!addWire(aOuterWire, aDFace))
        {
            aDFace.SetStatus(IMeshData_Failure);
            return;
        }

        TopExp_Explorer aWireIt = new(theFace, TopAbs_WIRE);
        for (; aWireIt.More(); aWireIt.Next())
        {
            TopoDS_Wire aWire = TopoDS.Wire(aWireIt.Current());
            if (aWire.IsSame(aOuterWire))
            {
                continue;
            }

            if (!addWire(aWire, aDFace))
            {
                // If there is a failure on internal wire, just skip it.
                // The most significant is an outer wire.
                aDFace.SetStatus(IMeshData_UnorientedWire);
            }
        }
    }

    // Adds wire to face discrete model.
    private bool addWire(in TopoDS_Wire theWire, IMeshData_Face theDFace)
    {
        if (theWire.IsNull())
        {
            return false;
        }

        ShapeExtend_WireData aWireData = new(theWire, true, false);
        ShapeAnalysis_Wire aWireTool = new(aWireData, theDFace.GetFace(), global::Kerf.Engine.Foundation.Precision.Confusion());

        ShapeAnalysis_WireOrder aOrderTool = new();
        aWireTool.CheckOrder(aOrderTool, true, false);
        if (aWireTool.LastCheckStatus(ShapeExtend_FAIL))
        {
            return false;
        }

        if (aWireTool.LastCheckStatus(ShapeExtend_DONE3))
        {
            theDFace.SetStatus(IMeshData_UnorientedWire);
        }

        int aEdgesNb = aOrderTool.NbEdges();
        if (aEdgesNb != aWireData.NbEdges())
        {
            return false;
        }

        IMeshData_Wire aDWire = theDFace.AddWire(theWire, aEdgesNb);
        for (int i = 1; i <= aEdgesNb; ++i)
        {
            int aEdgeIndex = aOrderTool.Ordered(i);
            TopoDS_Edge aEdge = aWireData.Edge(aEdgeIndex);
            if (aEdge.Orientation() != TopAbs_EXTERNAL)
            {
                IMeshData_Edge aDEdge = myModel.GetEdge(myDEdgeMap.Find(aEdge));

                aDEdge.AddPCurve(theDFace, aEdge.Orientation());
                aDWire.AddEdge(aDEdge, aEdge.Orientation());
            }
        }

        return true;
    }
}
