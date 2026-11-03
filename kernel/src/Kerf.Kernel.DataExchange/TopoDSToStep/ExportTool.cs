// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_Tool.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>
/// The working data of a translation from a BRep to STEP topology: the entities made from the shapes (keyed as
/// TopTools_ShapeMapHasher keys them: the same TShape and location), the current shell, face, wire, edge and vertex,
/// the faceted context, the lowest 3D tolerance met and the pcurve mode.
/// </summary>
/// <remarks>A working object (OCCT's value class, decision 1185). Its map iterates in the order of binding.</remarks>
public sealed class ExportTool
{
    private Dictionary<Shape, Transient> _dataMap = new(ShapeHasher.Instance);
    private bool _facetedContext;
    private double _lowestTol;
    private Shell _currentShell = new Shell();
    private Face _currentFace = new Face();
    private Wire _currentWire = new Wire();
    private Edge _currentEdge = new Edge();
    private Vertex _currentVertex = new Vertex();
    private bool _reversedSurface;
    private int _pCurveMode;

    /// <summary>A tool with the model's pcurve mode (write.surfacecurve.mode).</summary>
    public ExportTool(StepModel model)
    {
        _facetedContext = false;
        _lowestTol = 0.0;
        _reversedSurface = false;
        _pCurveMode = model.InternalParameters.WriteSurfaceCurMode ? 1 : 0;
    }

    public ExportTool(Dictionary<Shape, Transient> m, bool facetedContext, int surfCurveMode)
    {
        _lowestTol = 0.0;
        _reversedSurface = false;
        Init(m, facetedContext, surfCurveMode);
    }

    /// <summary>Initializes the tool with a copy of the map.</summary>
    public void Init(Dictionary<Shape, Transient> m, bool facetedContext, int surfCurveMode)
    {
        _dataMap = new Dictionary<Shape, Transient>(m,m.Comparer);
        _facetedContext = facetedContext;
        _pCurveMode = surfCurveMode;
    }

    public bool IsBound(in Shape s) => _dataMap.ContainsKey(s);

    public void Bind(in Shape s, StepTopologicalRepresentationItem? t) => _dataMap[s] = t!;

    public StepTopologicalRepresentationItem? Find(in Shape s) => _dataMap.GetValue(s) as StepTopologicalRepresentationItem;

    public bool Faceted() => _facetedContext;

    public void SetCurrentShell(in Shell s)
    {
        // OCCT_DEBUG output left out
        _currentShell = s;
    }

    public Shell CurrentShell() => _currentShell;

    public void SetCurrentFace(in Face f)
    {
        // OCCT_DEBUG output left out
        double faceTol = Tool.Tolerance(f);
        if (faceTol > _lowestTol)
        {
            _lowestTol = faceTol;
        }

        _currentFace = f;
    }

    public Face CurrentFace() => _currentFace;

    public void SetCurrentWire(in Wire w)
    {
        // OCCT_DEBUG output left out
        _currentWire = w;
    }

    public Wire CurrentWire() => _currentWire;

    public void SetCurrentEdge(in Edge e)
    {
        // OCCT_DEBUG output left out
        double edgeTol = Tool.Tolerance(e);
        if (edgeTol > _lowestTol)
        {
            _lowestTol = edgeTol;
        }

        _currentEdge = e;
    }

    public Edge CurrentEdge() => _currentEdge;

    public void SetCurrentVertex(in Vertex v)
    {
        double vertexTol = Tool.Tolerance(v);
        if (vertexTol > _lowestTol)
        {
            _lowestTol = vertexTol;
        }

        _currentVertex = v;
    }

    public Vertex CurrentVertex() => _currentVertex;

    public double Lowest3DTolerance() => _lowestTol;

    public void SetSurfaceReversed(bool b) => _reversedSurface = b;

    public bool SurfaceReversed() => _reversedSurface;

    /// <summary>The map of the shapes and their entities (the tool's own: read it, don't change it).</summary>
    public Dictionary<Shape, Transient> Map() => _dataMap;

    /// <summary>The mode for writing pcurves (write.surfacecurve.mode).</summary>
    public int PCurveMode() => _pCurveMode;
}
