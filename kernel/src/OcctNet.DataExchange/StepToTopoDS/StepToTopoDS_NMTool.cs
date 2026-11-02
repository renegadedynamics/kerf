// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_NMTool.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_NMTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Provides data to process non-manifold topology when reading from STEP.</summary>
/// <remarks>
/// A working object (OCCT's value class, decision 1185), holding the reader's shapes as they are being built. The
/// maps are keyed by reference (representation items) and by name, and never iterated.
/// </remarks>
public sealed class StepToTopoDS_NMTool
{
    private NCollection_DataMap<StepRepr_RepresentationItem, TopoDS_Shape> myRIMap = new();
    private NCollection_DataMap<string, TopoDS_Shape> myRINamesMap = new();
    private readonly NCollection_List<TopoDS_Shape> myNMEdges = new();
    private bool myIDEASCase;
    private bool myActiveFlag;

    public StepToTopoDS_NMTool()
    {
        myIDEASCase = false;
        myActiveFlag = false;
    }

    /// <summary>A tool with copies of the maps of representation items and of their names.</summary>
    public StepToTopoDS_NMTool(NCollection_DataMap<StepRepr_RepresentationItem, TopoDS_Shape> MapOfRI,
                               NCollection_DataMap<string, TopoDS_Shape> MapOfRINames)
    {
        myIDEASCase = false;
        myActiveFlag = false;
        Init(MapOfRI, MapOfRINames);
    }

    /// <summary>Initializes the internal maps of the tool with copies of the passed ones.</summary>
    public void Init(NCollection_DataMap<StepRepr_RepresentationItem, TopoDS_Shape> MapOfRI,
                     NCollection_DataMap<string, TopoDS_Shape> MapOfRINames)
    {
        myRIMap = new NCollection_DataMap<StepRepr_RepresentationItem, TopoDS_Shape>(MapOfRI);
        myRINamesMap = new NCollection_DataMap<string, TopoDS_Shape>(MapOfRINames);
    }

    /// <summary>Turns the tool on or off (off by default).</summary>
    public void SetActive(bool isActive) => myActiveFlag = isActive;

    public bool IsActive() => myActiveFlag;

    /// <summary>Clears the maps.</summary>
    public void CleanUp()
    {
        myRIMap.Clear();
        myRINamesMap.Clear();
    }

    public bool IsBound(StepRepr_RepresentationItem? RI) => RI is not null && myRIMap.IsBound(RI);

    public bool IsBound(string RIName) => myRINamesMap.IsBound(RIName);

    public void Bind(StepRepr_RepresentationItem? RI, in TopoDS_Shape S)
    {
        if (RI is not null) // OCCT binds a null handle too; no caller passes one (each binds an item it translated)
        {
            myRIMap.Bind(RI, S);
        }
    }

    public void Bind(string RIName, in TopoDS_Shape S) => myRINamesMap.Bind(RIName, S);

    public TopoDS_Shape Find(StepRepr_RepresentationItem? RI) => myRIMap.Find(RI!);

    public TopoDS_Shape Find(string RIName) => myRINamesMap.Find(RIName);

    /// <summary>Registers a non-manifold edge, unless it was registered before.</summary>
    public void RegisterNMEdge(in TopoDS_Shape Edge)
    {
        if (!isEdgeRegisteredAsNM(Edge))
        {
            myNMEdges.Append(Edge);
        }
    }

    /// <summary>True if <paramref name="SuspectedShell"/> is pure non-manifold and adjacent to <paramref name="BaseShell"/>.</summary>
    public bool IsSuspectedAsClosing(in TopoDS_Shape BaseShell, in TopoDS_Shape SuspectedShell)
        => IsPureNMShell(SuspectedShell) && isAdjacentShell(BaseShell, SuspectedShell);

    /// <summary>Sets the I-DEAS case flag (an I-DEAS-like STEP file is processed).</summary>
    public void SetIDEASCase(bool IDEASCase) => myIDEASCase = IDEASCase;

    public bool IsIDEASCase() => myIDEASCase;

    /// <summary>True if the shell contains only non-manifold edges.</summary>
    public bool IsPureNMShell(in TopoDS_Shape Shell)
    {
        bool result = true;
        var edgeExp = new TopExp_Explorer(Shell, TopAbs_EDGE);
        for (; edgeExp.More(); edgeExp.Next())
        {
            TopoDS_Shape currentEdge = edgeExp.Current();
            if (!isEdgeRegisteredAsNM(currentEdge))
            {
                result = false;
                break;
            }
        }

        return result;
    }

    private bool isEdgeRegisteredAsNM(in TopoDS_Shape Edge)
    {
        bool result = false;
        var it = new NCollection_List<TopoDS_Shape>.Iterator(myNMEdges);
        for (; it.More(); it.Next())
        {
            TopoDS_Shape currentShape = it.Value();
            if (currentShape.IsSame(Edge))
            {
                result = true;
                break;
            }
        }

        return result;
    }

    private static bool isAdjacentShell(in TopoDS_Shape ShellA, in TopoDS_Shape ShellB)
    {
        if (ShellA.IsSame(ShellB))
        {
            return false;
        }

        var edgeExpA = new TopExp_Explorer(ShellA, TopAbs_EDGE);
        for (; edgeExpA.More(); edgeExpA.Next())
        {
            TopoDS_Shape currentEdgeA = edgeExpA.Current();
            var edgeExpB = new TopExp_Explorer(ShellB, TopAbs_EDGE);
            for (; edgeExpB.More(); edgeExpB.Next())
            {
                TopoDS_Shape currentEdgeB = edgeExpB.Current();
                if (currentEdgeA.IsSame(currentEdgeB))
                {
                    return true;
                }
            }
        }

        return false;
    }
}
