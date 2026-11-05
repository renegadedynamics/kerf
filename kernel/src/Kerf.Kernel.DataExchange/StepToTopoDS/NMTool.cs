// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_NMTool.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_NMTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2010-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Topology;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>Provides data to process non-manifold topology when reading from STEP.</summary>
/// <remarks>
/// A working object (OCCT's value class, decision 1185), holding the reader's shapes as they are being built. The
/// maps are keyed by reference (representation items) and by name, and never iterated.
/// </remarks>
internal sealed class NMTool
{
    private Dictionary<RepresentationItem, Shape> _riMap = new();
    private Dictionary<string, Shape> _riNamesMap = new();
    private readonly NCollection_List<Shape> _nmEdges = new();
    private bool _ideasCase;
    private bool _activeFlag;

    public NMTool()
    {
        _ideasCase = false;
        _activeFlag = false;
    }

    /// <summary>A tool with copies of the maps of representation items and of their names.</summary>
    public NMTool(Dictionary<RepresentationItem, Shape> mapOfRI,
                               Dictionary<string, Shape> mapOfRINames)
    {
        _ideasCase = false;
        _activeFlag = false;
        Init(mapOfRI, mapOfRINames);
    }

    /// <summary>Initializes the internal maps of the tool with copies of the passed ones.</summary>
    public void Init(Dictionary<RepresentationItem, Shape> mapOfRI,
                     Dictionary<string, Shape> mapOfRINames)
    {
        _riMap = new Dictionary<RepresentationItem, Shape>(mapOfRI,mapOfRI.Comparer);
        _riNamesMap = new Dictionary<string, Shape>(mapOfRINames,mapOfRINames.Comparer);
    }

    /// <summary>Turns the tool on or off (off by default).</summary>
    public void SetActive(bool isActive) => _activeFlag = isActive;

    public bool IsActive => _activeFlag;

    /// <summary>Clears the maps.</summary>
    public void CleanUp()
    {
        _riMap.Clear();
        _riNamesMap.Clear();
    }

    public bool IsBound(RepresentationItem? ri) => ri is not null && _riMap.ContainsKey(ri);

    public bool IsBound(string riName) => _riNamesMap.ContainsKey(riName);

    public void Bind(RepresentationItem? ri, in Shape s)
    {
        if (ri is not null) // OCCT binds a null handle too; no caller passes one (each binds an item it translated)
        {
            _riMap[ri] = s;
        }
    }

    public void Bind(string riName, in Shape s) => _riNamesMap[riName] = s;

    public Shape Find(RepresentationItem? ri) => _riMap.GetValue(ri!);

    public Shape Find(string riName) => _riNamesMap.GetValue(riName);

    /// <summary>Registers a non-manifold edge, unless it was registered before.</summary>
    public void RegisterNMEdge(in Shape edge)
    {
        if (!IsEdgeRegisteredAsNM(edge))
        {
            _nmEdges.Append(edge);
        }
    }

    /// <summary>True if <paramref name="suspectedShell"/> is pure non-manifold and adjacent to <paramref name="baseShell"/>.</summary>
    public bool IsSuspectedAsClosing(in Shape baseShell, in Shape suspectedShell)
        => IsPureNMShell(suspectedShell) && IsAdjacentShell(baseShell, suspectedShell);

    /// <summary>Sets the I-DEAS case flag (an I-DEAS-like STEP file is processed).</summary>
    public void SetIDEASCase(bool ideasCase) => _ideasCase = ideasCase;

    public bool IsIDEASCase => _ideasCase;

    /// <summary>True if the shell contains only non-manifold edges.</summary>
    public bool IsPureNMShell(in Shape shell)
    {
        bool result = true;
        var edgeExp = new Explorer(shell, ShapeType.Edge);
        for (; edgeExp.More(); edgeExp.Next())
        {
            Shape currentEdge = edgeExp.Current();
            if (!IsEdgeRegisteredAsNM(currentEdge))
            {
                result = false;
                break;
            }
        }

        return result;
    }

    private bool IsEdgeRegisteredAsNM(in Shape edge)
    {
        bool result = false;
        var it = new NCollection_List<Shape>.Iterator(_nmEdges);
        for (; it.More(); it.Next())
        {
            Shape currentShape = it.Value();
            if (currentShape.IsSame(edge))
            {
                result = true;
                break;
            }
        }

        return result;
    }

    private static bool IsAdjacentShell(in Shape shellA, in Shape shellB)
    {
        if (shellA.IsSame(shellB))
        {
            return false;
        }

        var edgeExpA = new Explorer(shellA, ShapeType.Edge);
        for (; edgeExpA.More(); edgeExpA.Next())
        {
            Shape currentEdgeA = edgeExpA.Current();
            var edgeExpB = new Explorer(shellB, ShapeType.Edge);
            for (; edgeExpB.More(); edgeExpB.Next())
            {
                Shape currentEdgeB = edgeExpB.Current();
                if (currentEdgeA.IsSame(currentEdgeB))
                {
                    return true;
                }
            }
        }

        return false;
    }
}
