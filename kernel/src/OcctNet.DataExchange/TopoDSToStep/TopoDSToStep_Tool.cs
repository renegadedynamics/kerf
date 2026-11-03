// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_Tool.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;

namespace Occt;

/// <summary>
/// The working data of a translation from a BRep to STEP topology: the entities made from the shapes (keyed as
/// TopTools_ShapeMapHasher keys them: the same TShape and location), the current shell, face, wire, edge and vertex,
/// the faceted context, the lowest 3D tolerance met and the pcurve mode.
/// </summary>
/// <remarks>A working object (OCCT's value class, decision 1185). Its map iterates in the order of binding.</remarks>
public sealed class TopoDSToStep_Tool
{
    private NCollection_DataMap<Shape, Transient> myDataMap = new(ShapeHasher.Instance);
    private bool myFacetedContext;
    private double myLowestTol;
    private Shell myCurrentShell = new Shell();
    private Face myCurrentFace = new Face();
    private Wire myCurrentWire = new Wire();
    private Edge myCurrentEdge = new Edge();
    private Vertex myCurrentVertex = new Vertex();
    private bool myReversedSurface;
    private int myPCurveMode;

    /// <summary>A tool with the model's pcurve mode (write.surfacecurve.mode).</summary>
    public TopoDSToStep_Tool(StepData_StepModel theModel)
    {
        myFacetedContext = false;
        myLowestTol = 0.0;
        myReversedSurface = false;
        myPCurveMode = theModel.InternalParameters.WriteSurfaceCurMode ? 1 : 0;
    }

    public TopoDSToStep_Tool(NCollection_DataMap<Shape, Transient> M, bool FacetedContext, int theSurfCurveMode)
    {
        myLowestTol = 0.0;
        myReversedSurface = false;
        Init(M, FacetedContext, theSurfCurveMode);
    }

    /// <summary>Initializes the tool with a copy of the map.</summary>
    public void Init(NCollection_DataMap<Shape, Transient> M, bool FacetedContext, int theSurfCurveMode)
    {
        myDataMap = new NCollection_DataMap<Shape, Transient>(M);
        myFacetedContext = FacetedContext;
        myPCurveMode = theSurfCurveMode;
    }

    public bool IsBound(in Shape S) => myDataMap.IsBound(S);

    public void Bind(in Shape S, StepShape_TopologicalRepresentationItem? T) => myDataMap.Bind(S, T!);

    public StepShape_TopologicalRepresentationItem? Find(in Shape S) => myDataMap.Find(S) as StepShape_TopologicalRepresentationItem;

    public bool Faceted() => myFacetedContext;

    public void SetCurrentShell(in Shell S)
    {
        // OCCT_DEBUG output left out
        myCurrentShell = S;
    }

    public Shell CurrentShell() => myCurrentShell;

    public void SetCurrentFace(in Face F)
    {
        // OCCT_DEBUG output left out
        double FaceTol = Tool.Tolerance(F);
        if (FaceTol > myLowestTol)
        {
            myLowestTol = FaceTol;
        }

        myCurrentFace = F;
    }

    public Face CurrentFace() => myCurrentFace;

    public void SetCurrentWire(in Wire W)
    {
        // OCCT_DEBUG output left out
        myCurrentWire = W;
    }

    public Wire CurrentWire() => myCurrentWire;

    public void SetCurrentEdge(in Edge E)
    {
        // OCCT_DEBUG output left out
        double EdgeTol = Tool.Tolerance(E);
        if (EdgeTol > myLowestTol)
        {
            myLowestTol = EdgeTol;
        }

        myCurrentEdge = E;
    }

    public Edge CurrentEdge() => myCurrentEdge;

    public void SetCurrentVertex(in Vertex V)
    {
        double VertexTol = Tool.Tolerance(V);
        if (VertexTol > myLowestTol)
        {
            myLowestTol = VertexTol;
        }

        myCurrentVertex = V;
    }

    public Vertex CurrentVertex() => myCurrentVertex;

    public double Lowest3DTolerance() => myLowestTol;

    public void SetSurfaceReversed(bool B) => myReversedSurface = B;

    public bool SurfaceReversed() => myReversedSurface;

    /// <summary>The map of the shapes and their entities (the tool's own: read it, don't change it).</summary>
    public NCollection_DataMap<Shape, Transient> Map() => myDataMap;

    /// <summary>The mode for writing pcurves (write.surfacecurve.mode).</summary>
    public int PCurveMode() => myPCurveMode;
}
