// Port of: src/DataExchange/TKDESTEP/STEPSelections/STEPSelections_SelectForTransfer.hxx
// Port of: src/DataExchange/TKDESTEP/STEPSelections/STEPSelections_SelectForTransfer.cxx
// Port of: src/DataExchange/TKDESTEP/STEPSelections/STEPSelections_SelectDerived.hxx
// Port of: src/DataExchange/TKDESTEP/STEPSelections/STEPSelections_SelectDerived.cxx
// Port of: src/DataExchange/TKDESTEP/STEPSelections/STEPSelections_SelectFaces.hxx
// Port of: src/DataExchange/TKDESTEP/STEPSelections/STEPSelections_SelectFaces.cxx
// Port of: src/DataExchange/TKDESTEP/STEPSelections/STEPSelections_SelectGSCurves.hxx
// Port of: src/DataExchange/TKDESTEP/STEPSelections/STEPSelections_SelectGSCurves.cxx
// Port of: src/DataExchange/TKDESTEP/STEPSelections/STEPSelections_SelectAssembly.hxx
// Port of: src/DataExchange/TKDESTEP/STEPSelections/STEPSelections_SelectAssembly.cxx
// Port of: src/DataExchange/TKDESTEP/STEPSelections/STEPSelections_SelectInstances.hxx
// Port of: src/DataExchange/TKDESTEP/STEPSelections/STEPSelections_SelectInstances.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>The roots the transfer reader's transient process recorded for transfer (a STEP reader's roots).</summary>
public class STEPSelections_SelectForTransfer : XSControl_SelectForTransfer
{
    public STEPSelections_SelectForTransfer()
    {
    }

    public STEPSelections_SelectForTransfer(XSControl_TransferReader TR)
    {
        SetReader(TR);
    }

    public override Interface_EntityIterator RootResult(Interface_Graph G)
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        NCollection_HSequence<Transient> roots = Reader().TransientProcess().RootsForTransfer();
        int nb = roots.Length();
        for (int i = 1; i <= nb; i++)
        {
            iter.GetOneItem(roots.Value(i));
        }
        return iter;
    }
}

/// <summary>A selection of entities of a STEP type or of a type derived from it.</summary>
public class STEPSelections_SelectDerived : StepSelect_StepType
{
    public STEPSelections_SelectDerived()
    {
    }

    private static Type GetStepType(StepData_ReadWriteModule module, string type)
    {
        Type atype = null;
        if (module is null)
        {
            return atype;
        }
        int num = module.CaseStep(type);
        if (num == 0)
        {
            return atype;
        }
        RWStepAP214_GeneralModule genModul = new RWStepAP214_GeneralModule();
        genModul.NewVoid(num, out Transient ent);
        atype = ent.DynamicType();
        return atype;
    }

    public override bool Matches(Transient ent, Interface_InterfaceModel model, string text, bool exact)
    {
        bool ok = thelib.Select(ent, out StepData_ReadWriteModule module, out int CN);
        if (!ok)
        {
            return false;
        }
        Type checker = GetStepType(module, text);
        if (checker is null)
        {
            return false;
        }
        bool plex = module.IsComplex(CN);
        if (!plex)
        {
            // OCCT first takes the entity as a Standard_Type itself: a System.Type is never an entity here
            Type atype = ent.DynamicType();
            return checker.IsAssignableFrom(atype);
        }
        else
        {
            NCollection_Sequence<string> list = new NCollection_Sequence<string>();
            module.ComplexType(CN, list);
            int nb = list.Length();
            for (int i = 1; i <= nb; i++)
            {
                Type atype = GetStepType(module, list.Value(i));
                // OCCT dereferences the null type of a component it doesn't know (a crash): false here
                if (atype is not null && checker.IsAssignableFrom(atype))
                {
                    return true;
                }
            }
        }
        return false;
    }
}

/// <summary>The faces of the input (face surfaces, and surfaces of geometric sets or alone).</summary>
public class STEPSelections_SelectFaces : IFSelect_SelectExplore
{
    public STEPSelections_SelectFaces() : base(-1)
    {
    }

    public override bool Explore(int level, Transient start, Interface_Graph G, Interface_EntityIterator explored)
    {
        if (start is null)
        {
            return false;
        }

        if (start.IsKind(typeof(StepShape_FaceSurface)))
        {
            return true;
        }

        bool isInFaceOfInSurface = false;
        if (start.IsKind(typeof(StepGeom_Surface)))
        {
            Interface_EntityIterator subs1 = G.Sharings(start);
            for (subs1.Start(); subs1.More(); subs1.Next())
            {
                if (subs1.Value().IsKind(typeof(StepShape_GeometricSet)))
                {
                    return true;
                }
                if (subs1.Value().IsKind(typeof(StepGeom_Surface)))
                {
                    isInFaceOfInSurface = true;
                }
                if (subs1.Value().IsKind(typeof(StepShape_FaceSurface)))
                {
                    isInFaceOfInSurface = true;
                }
            }
            return !isInFaceOfInSurface;
        }

        Interface_EntityIterator subs = G.Shareds(start);
        subs.Start();
        bool isSome = subs.More();
        for (; subs.More(); subs.Next())
        {
            explored.AddItem(subs.Value());
        }

        return isSome;
    }

    public override string ExploreLabel()
    {
        return "Faces";
    }
}

/// <summary>The curves of the geometric sets of the input.</summary>
/// <remarks>
/// OCCT keeps the flag that lets the first composite curve's geometric set through in a thread-local variable, reset
/// by every construction; the port keeps it in the selection (decision 1270).
/// </remarks>
public class STEPSelections_SelectGSCurves : IFSelect_SelectExplore
{
    private int flag;

    public STEPSelections_SelectGSCurves() : base(-1)
    {
        flag = 1;
    }

    public override bool Explore(int level, Transient start, Interface_Graph G, Interface_EntityIterator explored)
    {
        if (start is null)
        {
            return false;
        }

        if (start.IsKind(typeof(StepGeom_Curve)))
        {
            if (start.IsKind(typeof(StepGeom_CompositeCurve)))
            {
                Interface_EntityIterator subs1 = G.Sharings(start);
                bool isInGeomSet = false;
                for (subs1.Start(); subs1.More() && !isInGeomSet; subs1.Next())
                {
                    if (subs1.Value().IsKind(typeof(StepShape_GeometricSet)))
                    {
                        if (flag != 0)
                        {
                            explored.AddItem(subs1.Value());
                            flag = 0;
                        }
                        isInGeomSet = true;
                    }
                }
                if (isInGeomSet)
                {
                    Interface_EntityIterator aSubsShareds = G.Shareds(start);
                    aSubsShareds.Start();
                    bool isSome = aSubsShareds.More();
                    for (; aSubsShareds.More(); aSubsShareds.Next())
                    {
                        explored.AddItem(aSubsShareds.Value());
                    }
                    return isSome;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                Interface_EntityIterator subs1 = G.Sharings(start);
                for (subs1.Start(); subs1.More(); subs1.Next())
                {
                    if (subs1.Value().IsKind(typeof(StepShape_GeometricSet))
                        || subs1.Value().IsKind(typeof(StepGeom_CompositeCurveSegment)))
                    {
                        return true;
                    }
                }
            }
        }

        Interface_EntityIterator subs = G.Shareds(start);
        subs.Start();
        bool isSome1 = subs.More();
        for (; subs.More(); subs.Next())
        {
            explored.AddItem(subs.Value());
        }

        return isSome1;
    }

    public override string ExploreLabel()
    {
        return "Curves in GS";
    }
}

/// <summary>The assembly components of the input (placed items and context dependent representations).</summary>
public class STEPSelections_SelectAssembly : IFSelect_SelectExplore
{
    public STEPSelections_SelectAssembly() : base(-1)
    {
    }

    public override bool Explore(int level, Transient start, Interface_Graph G, Interface_EntityIterator explored)
    {
        if (start is null)
        {
            return false;
        }

        if (start.IsKind(typeof(StepShape_ContextDependentShapeRepresentation)))
        {
            StepShape_ContextDependentShapeRepresentation sdsr = start as StepShape_ContextDependentShapeRepresentation;
            StepRepr_ProductDefinitionShape pds = sdsr.RepresentedProductRelation();
            if (pds is null)
            {
                return false;
            }
            Transient ent = pds.Definition().ProductDefinitionRelationship();
            if (ent is null)
            {
                return false;
            }
            return (ent.IsKind(typeof(StepRepr_NextAssemblyUsageOccurrence)));
        }

        if (start.IsKind(typeof(StepRepr_MappedItem)))
        {
            StepRepr_MappedItem mapped = start as StepRepr_MappedItem;
            Interface_EntityIterator subs1 = G.Sharings(mapped);
            StepShape_ShapeRepresentation shrep = null;
            for (subs1.Start(); subs1.More() && shrep is null; subs1.Next())
            {
                if (subs1.Value().IsKind(typeof(StepShape_ShapeRepresentation)))
                {
                    shrep = subs1.Value() as StepShape_ShapeRepresentation;
                }
            }
            if (shrep is null)
            {
                return false;
            }

            subs1 = G.Sharings(shrep);
            StepShape_ShapeDefinitionRepresentation shdefrep = null;
            for (subs1.Start(); subs1.More() && shdefrep is null; subs1.Next())
            {
                if (subs1.Value().IsKind(typeof(StepShape_ShapeDefinitionRepresentation)))
                {
                    shdefrep = subs1.Value() as StepShape_ShapeDefinitionRepresentation;
                }
            }
            if (shdefrep is null)
            {
                return false;
            }

            StepRepr_ProductDefinitionShape pds = shdefrep.Definition().PropertyDefinition() as StepRepr_ProductDefinitionShape;
            if (pds is null)
            {
                return false;
            }
            Transient ent = pds.Definition().ProductDefinitionRelationship();
            if (ent is null)
            {
                return false;
            }
            return (ent.IsKind(typeof(StepRepr_NextAssemblyUsageOccurrence)));
        }

        Interface_EntityIterator subs = G.Shareds(start);
        subs.Start();
        bool isSome = subs.More();
        for (; subs.More(); subs.Next())
        {
            explored.AddItem(subs.Value());
        }

        return isSome;
    }

    public override string ExploreLabel()
    {
        return "Assembly components";
    }
}

/// <summary>The instances (the entities used by the assembly structure) of the model's roots.</summary>
/// <remarks>
/// OCCT caches the graph and the instances of its last model in thread-local variables shared by every instance of
/// the class; the port caches them in the selection (decision 1270).
/// </remarks>
public class STEPSelections_SelectInstances : IFSelect_SelectExplore
{
    private Interface_HGraph myGraph;
    private Interface_EntityIterator myEntities = new Interface_EntityIterator();

    public STEPSelections_SelectInstances() : base(-1)
    {
    }

    private static void AddAllSharings(Transient start, Interface_Graph graph, Interface_EntityIterator explored)
    {
        if (start is null)
        {
            return;
        }
        Interface_EntityIterator subs = graph.Shareds(start);
        for (subs.Start(); subs.More(); subs.Next())
        {
            explored.AddItem(subs.Value());
            AddAllSharings(subs.Value(), graph, explored);
        }
    }

    private static void AddInstances(Transient start, Interface_Graph graph, Interface_EntityIterator explored)
    {
        if (start is null)
        {
            return;
        }

        explored.AddItem(start);
        if (start.IsKind(typeof(StepShape_ShapeDefinitionRepresentation)))
        {
            StepShape_ShapeDefinitionRepresentation sdr = start as StepShape_ShapeDefinitionRepresentation;
            AddInstances(sdr.UsedRepresentation(), graph, explored);
            Interface_EntityIterator subs = graph.Shareds(start);
            for (subs.Start(); subs.More(); subs.Next())
            {
                StepShape_ContextDependentShapeRepresentation anitem = subs.Value() as StepShape_ContextDependentShapeRepresentation;
                if (anitem is null)
                {
                    continue;
                }
                AddInstances(anitem, graph, explored);
            }
            return;
        }

        if (start.IsKind(typeof(StepShape_ShapeRepresentation)))
        {
            StepShape_ShapeRepresentation sr = start as StepShape_ShapeRepresentation;
            int nb = sr.NbItems();
            for (int i = 1; i <= nb; i++)
            {
                StepRepr_RepresentationItem anitem = sr.ItemsValue(i);
                AddInstances(anitem, graph, explored);
            }
            return;
        }

        if (start.IsKind(typeof(StepShape_FacetedBrep))
            || start.IsKind(typeof(StepShape_BrepWithVoids))
            || start.IsKind(typeof(StepShape_ManifoldSolidBrep))
            || start.IsKind(typeof(StepShape_ShellBasedSurfaceModel))
            || start.IsKind(typeof(StepShape_FacetedBrepAndBrepWithVoids))
            || start.IsKind(typeof(StepShape_GeometricSet))
            || start.IsKind(typeof(StepShape_FaceSurface))
            || start.IsKind(typeof(StepRepr_MappedItem)))
        {
            AddAllSharings(start, graph, explored);
            return;
        }

        if (start.IsKind(typeof(StepShape_ContextDependentShapeRepresentation)))
        {
            StepShape_ContextDependentShapeRepresentation CDSR = start as StepShape_ContextDependentShapeRepresentation;
            StepRepr_RepresentationRelationship SRR = CDSR.RepresentationRelation();
            if (SRR is null)
            {
                return;
            }

            StepRepr_Representation rep;
            bool SRRReversed = STEPConstruct_Assembly.CheckSRRReversesNAUO(graph, CDSR);
            if (SRRReversed)
            {
                rep = SRR.Rep2();
            }
            else
            {
                rep = SRR.Rep1();
            }

            Interface_EntityIterator subs = graph.Sharings(rep);
            for (subs.Start(); subs.More(); subs.Next())
            {
                if (subs.Value().IsKind(typeof(StepShape_ShapeDefinitionRepresentation)))
                {
                    StepShape_ShapeDefinitionRepresentation SDR = subs.Value() as StepShape_ShapeDefinitionRepresentation;
                    AddInstances(SDR, graph, explored);
                }
            }
            //???
            return;
        }

        if (start.IsKind(typeof(StepRepr_ShapeRepresentationRelationship)))
        {
            StepRepr_ShapeRepresentationRelationship und = start as StepRepr_ShapeRepresentationRelationship;
            for (int i = 1; i <= 2; i++)
            {
                Transient anitem = null;
                if (i == 1)
                {
                    anitem = und.Rep1();
                }
                if (i == 2)
                {
                    anitem = und.Rep2();
                }
                AddInstances(anitem, graph, explored);
            }
            return;
        }
    }

    public override Interface_EntityIterator RootResult(Interface_Graph G)
    {
        if (myGraph is null || (G.Model() != myGraph.Graph().Model()))
        {
            Interface_EntityIterator roots = G.RootEntities();
            myGraph = new Interface_HGraph(G);
            myEntities.Destroy();
            for (roots.Start(); roots.More(); roots.Next())
            {
                AddInstances(roots.Value(), G, myEntities);
            }
        }

        if (HasInput() || HasAlternate())
        {
            Interface_EntityIterator select = InputResult(G);
            int nbSelected = select.NbEntities();
            HashSet<Transient> filter = new HashSet<Transient>(nbSelected, ReferenceEqualityComparer.Instance);
            for (select.Start(); select.More(); select.Next())
            {
                filter.Add(select.Value());
            }
            Interface_EntityIterator result = new Interface_EntityIterator();
            for (myEntities.Start(); myEntities.More(); myEntities.Next())
            {
                if (filter.Contains(myEntities.Value()))
                {
                    result.AddItem(myEntities.Value());
                }
            }
            return result;
        }
        else
        {
            return myEntities;
        }
    }

    public override bool Explore(int level, Transient start, Interface_Graph G, Interface_EntityIterator explored)
    {
        return false;
    }

    protected override bool HasUniqueResult()
    {
        return true;
    }

    public override string ExploreLabel()
    {
        return "Instances";
    }
}
