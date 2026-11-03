// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_Tool.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// The working data of a translation from STEP topology to a BRep: the shapes made from the topological items (bound
/// in the transient process too), the vertices and edges of faceted breps keyed by their points, and statistics on
/// the continuity of the geometry.
/// </summary>
/// <remarks>
/// A working object (OCCT's value class, decision 1185). It holds the translation's shapes as they are being built
/// (unfrozen); the maps are keyed by reference and never iterated.
/// </remarks>
public sealed class StepToTopoDS_Tool
{
    private NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape> myDataMap = new();
    private NCollection_DataMap<StepGeom_CartesianPoint, TopoDS_Vertex> myVertexMap = new();
    private NCollection_DataMap<StepToTopoDS_PointPair, TopoDS_Edge> myEdgeMap = new();
    private bool myComputePC;
    private Transfer_TransientProcess? myTransProc;
    private int myNbC0Surf;
    private int myNbC1Surf;
    private int myNbC2Surf;
    private int myNbC0Cur2;
    private int myNbC1Cur2;
    private int myNbC2Cur2;
    private int myNbC0Cur3;
    private int myNbC1Cur3;
    private int myNbC2Cur3;
    // OCCT's maps accept a null handle as a key (a file's missing reference); .NET's dictionaries don't
    private bool myHasNullItem;
    private TopoDS_Shape myNullItemShape;

    public StepToTopoDS_Tool()
    {
        myComputePC = false;
    }

    /// <summary>A tool with a copy of the map and the transient process.</summary>
    public StepToTopoDS_Tool(NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape> Map,
                             Transfer_TransientProcess? TP)
    {
        Init(Map, TP);
    }

    public void Init(NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape> Map, Transfer_TransientProcess? TP)
    {
        myComputePC = false;

        myDataMap = new NCollection_DataMap<StepShape_TopologicalRepresentationItem, TopoDS_Shape>(Map);
        myVertexMap = new NCollection_DataMap<StepGeom_CartesianPoint, TopoDS_Vertex>();
        myEdgeMap = new NCollection_DataMap<StepToTopoDS_PointPair, TopoDS_Edge>();
        myTransProc = TP;
        myHasNullItem = false;
        myNullItemShape = new TopoDS_Shape();

        myNbC0Surf = myNbC1Surf = myNbC2Surf = 0;
        myNbC0Cur2 = myNbC1Cur2 = myNbC2Cur2 = 0;
        myNbC0Cur3 = myNbC1Cur3 = myNbC2Cur3 = 0;
    }

    /// <summary>True if the item is bound in the map.</summary>
    public bool IsBound(StepShape_TopologicalRepresentationItem? TRI) => TRI is null ? myHasNullItem : myDataMap.IsBound(TRI);

    /// <summary>Binds the item with a shape in the map and, as its shape result, in the transient process.</summary>
    public void Bind(StepShape_TopologicalRepresentationItem? TRI, in TopoDS_Shape S)
    {
        if (TRI is null)
        {
            myHasNullItem = true;
            myNullItemShape = S;
        }
        else
        {
            myDataMap.Bind(TRI, S);
        }

        TransferBRep.SetShapeResult(myTransProc, TRI, S);
    }

    /// <summary>The shape bound to the item.</summary>
    /// <exception cref="NoSuchObjectException">The item is not bound.</exception>
    public TopoDS_Shape Find(StepShape_TopologicalRepresentationItem? TRI)
    {
        if (TRI is null)
        {
            return myHasNullItem ? myNullItemShape : throw new NoSuchObjectException("NCollection_DataMap::Find");
        }

        return myDataMap.Find(TRI);
    }

    public void ClearEdgeMap() => myEdgeMap.Clear();

    public bool IsEdgeBound(in StepToTopoDS_PointPair PP) => myEdgeMap.IsBound(PP);

    public void BindEdge(in StepToTopoDS_PointPair PP, in TopoDS_Edge E) => myEdgeMap.Bind(PP, E);

    public TopoDS_Edge FindEdge(in StepToTopoDS_PointPair PP) => myEdgeMap.Find(PP);

    public void ClearVertexMap() => myVertexMap.Clear();

    public bool IsVertexBound(StepGeom_CartesianPoint? PG) => PG is not null && myVertexMap.IsBound(PG);

    public void BindVertex(StepGeom_CartesianPoint? P, in TopoDS_Vertex V)
    {
        myVertexMap.Bind(P!, V);
        // OCCT_DEBUG: TransferBRep::SetShapeResult(myTransProc, P, V) left out
    }

    public TopoDS_Vertex FindVertex(StepGeom_CartesianPoint? P) => myVertexMap.Find(P!);

    public void ComputePCurve(bool B) => myComputePC = B;

    public bool ComputePCurve() => myComputePC;

    public Transfer_TransientProcess? TransientProcess() => myTransProc;

    // AddStatistics

    public void AddContinuity(Geom_Surface GeomSurf)
    {
        switch (GeomSurf.Continuity())
        {
            case C0:
                myNbC0Surf++;
                break;
            case C1:
                myNbC1Surf++;
                break;
            default:
                myNbC2Surf++;
                break;
        }
    }

    public void AddContinuity(Geom_Curve GeomCurve)
    {
        switch (GeomCurve.Continuity())
        {
            case C0:
                myNbC0Cur3++;
                break;
            case C1:
                myNbC1Cur3++;
                break;
            default:
                myNbC2Cur3++;
                break;
        }
    }

    public void AddContinuity(Geom2d_Curve GeomCur2d)
    {
        switch (GeomCur2d.Continuity())
        {
            case C0:
                myNbC0Cur2++;
                break;
            case C1:
                myNbC1Cur2++;
                break;
            default:
                myNbC2Cur2++;
                break;
        }
    }

    // Statistics

    public int C0Surf() => myNbC0Surf;

    public int C1Surf() => myNbC1Surf;

    public int C2Surf() => myNbC2Surf;

    public int C0Cur2() => myNbC0Cur2;

    public int C1Cur2() => myNbC1Cur2;

    public int C2Cur2() => myNbC2Cur2;

    public int C0Cur3() => myNbC0Cur3;

    public int C1Cur3() => myNbC1Cur3;

    public int C2Cur3() => myNbC2Cur3;
}
