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
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Step.Ap214.ReadWrite;
using Kerf.Engine.Exchange.Step.Construct;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Selections;

/// <summary>The roots the transfer reader's transient process recorded for transfer (a STEP reader's roots).</summary>
internal class StepSelectForTransfer : SelectForTransfer
{
    public StepSelectForTransfer()
    {
    }

    public StepSelectForTransfer(TransferReader tr)
    {
        SetReader(tr);
    }

    public override EntityIterator RootResult(EntityGraph g)
    {
        EntityIterator iter = new EntityIterator();
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
internal class SelectDerived : StepType
{
    public SelectDerived()
    {
    }

    private static Type GetStepType(ReadWriteModule module, string type)
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
        Ap214GeneralModule genModul = new Ap214GeneralModule();
        genModul.NewVoid(num, out Transient ent);
        atype = ent.DynamicType();
        return atype;
    }

    public override bool Matches(Transient ent, InterfaceModel model, string text, bool exact)
    {
        bool ok = _thelib.Select(ent, out ReadWriteModule module, out int cn);
        if (!ok)
        {
            return false;
        }
        Type checker = GetStepType(module, text);
        if (checker is null)
        {
            return false;
        }
        bool plex = module.IsComplex(cn);
        if (!plex)
        {
            // OCCT first takes the entity as a Standard_Type itself: a System.Type is never an entity here
            Type atype = ent.DynamicType();
            return checker.IsAssignableFrom(atype);
        }
        else
        {
            NCollection_Sequence<string> list = new NCollection_Sequence<string>();
            module.ComplexType(cn, list);
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
internal class SelectFaces : SelectExplore
{
    public SelectFaces() : base(-1)
    {
    }

    public override bool Explore(int level, Transient start, EntityGraph g, EntityIterator explored)
    {
        if (start is null)
        {
            return false;
        }

        if (start.IsKind(typeof(StepFaceSurface)))
        {
            return true;
        }

        bool isInFaceOfInSurface = false;
        if (start.IsKind(typeof(StepSurface)))
        {
            EntityIterator subs1 = g.Sharings(start);
            for (subs1.Start(); subs1.More(); subs1.Next())
            {
                if (subs1.Value().IsKind(typeof(StepGeometricSet)))
                {
                    return true;
                }
                if (subs1.Value().IsKind(typeof(StepSurface)))
                {
                    isInFaceOfInSurface = true;
                }
                if (subs1.Value().IsKind(typeof(StepFaceSurface)))
                {
                    isInFaceOfInSurface = true;
                }
            }
            return !isInFaceOfInSurface;
        }

        EntityIterator subs = g.Shareds(start);
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
internal class SelectGSCurves : SelectExplore
{
    private int _flag;

    public SelectGSCurves() : base(-1)
    {
        _flag = 1;
    }

    public override bool Explore(int level, Transient start, EntityGraph g, EntityIterator explored)
    {
        if (start is null)
        {
            return false;
        }

        if (start.IsKind(typeof(StepCurve)))
        {
            if (start.IsKind(typeof(StepCompositeCurve)))
            {
                EntityIterator subs1 = g.Sharings(start);
                bool isInGeomSet = false;
                for (subs1.Start(); subs1.More() && !isInGeomSet; subs1.Next())
                {
                    if (subs1.Value().IsKind(typeof(StepGeometricSet)))
                    {
                        if (_flag != 0)
                        {
                            explored.AddItem(subs1.Value());
                            _flag = 0;
                        }
                        isInGeomSet = true;
                    }
                }
                if (isInGeomSet)
                {
                    EntityIterator subsShareds = g.Shareds(start);
                    subsShareds.Start();
                    bool isSome = subsShareds.More();
                    for (; subsShareds.More(); subsShareds.Next())
                    {
                        explored.AddItem(subsShareds.Value());
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
                EntityIterator subs1 = g.Sharings(start);
                for (subs1.Start(); subs1.More(); subs1.Next())
                {
                    if (subs1.Value().IsKind(typeof(StepGeometricSet))
                        || subs1.Value().IsKind(typeof(StepCompositeCurveSegment)))
                    {
                        return true;
                    }
                }
            }
        }

        EntityIterator subs = g.Shareds(start);
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
internal class SelectAssembly : SelectExplore
{
    public SelectAssembly() : base(-1)
    {
    }

    public override bool Explore(int level, Transient start, EntityGraph g, EntityIterator explored)
    {
        if (start is null)
        {
            return false;
        }

        if (start.IsKind(typeof(StepContextDependentShapeRepresentation)))
        {
            StepContextDependentShapeRepresentation sdsr = start as StepContextDependentShapeRepresentation;
            ProductDefinitionShape pds = sdsr.RepresentedProductRelation();
            if (pds is null)
            {
                return false;
            }
            Transient ent = pds.Definition().ProductDefinitionRelationship();
            if (ent is null)
            {
                return false;
            }
            return (ent.IsKind(typeof(NextAssemblyUsageOccurrence)));
        }

        if (start.IsKind(typeof(MappedItem)))
        {
            MappedItem mapped = start as MappedItem;
            EntityIterator subs1 = g.Sharings(mapped);
            StepShapeRepresentation shrep = null;
            for (subs1.Start(); subs1.More() && shrep is null; subs1.Next())
            {
                if (subs1.Value().IsKind(typeof(StepShapeRepresentation)))
                {
                    shrep = subs1.Value() as StepShapeRepresentation;
                }
            }
            if (shrep is null)
            {
                return false;
            }

            subs1 = g.Sharings(shrep);
            StepShapeDefinitionRepresentation shdefrep = null;
            for (subs1.Start(); subs1.More() && shdefrep is null; subs1.Next())
            {
                if (subs1.Value().IsKind(typeof(StepShapeDefinitionRepresentation)))
                {
                    shdefrep = subs1.Value() as StepShapeDefinitionRepresentation;
                }
            }
            if (shdefrep is null)
            {
                return false;
            }

            ProductDefinitionShape pds = shdefrep.Definition().PropertyDefinition() as ProductDefinitionShape;
            if (pds is null)
            {
                return false;
            }
            Transient ent = pds.Definition().ProductDefinitionRelationship();
            if (ent is null)
            {
                return false;
            }
            return (ent.IsKind(typeof(NextAssemblyUsageOccurrence)));
        }

        EntityIterator subs = g.Shareds(start);
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
internal class SelectInstances : SelectExplore
{
    private HGraph _graph;
    private EntityIterator _entities = new EntityIterator();

    public SelectInstances() : base(-1)
    {
    }

    private static void AddAllSharings(Transient start, EntityGraph graph, EntityIterator explored)
    {
        if (start is null)
        {
            return;
        }
        EntityIterator subs = graph.Shareds(start);
        for (subs.Start(); subs.More(); subs.Next())
        {
            explored.AddItem(subs.Value());
            AddAllSharings(subs.Value(), graph, explored);
        }
    }

    private static void AddInstances(Transient start, EntityGraph graph, EntityIterator explored)
    {
        if (start is null)
        {
            return;
        }

        explored.AddItem(start);
        if (start.IsKind(typeof(StepShapeDefinitionRepresentation)))
        {
            StepShapeDefinitionRepresentation sdr = start as StepShapeDefinitionRepresentation;
            AddInstances(sdr.UsedRepresentation(), graph, explored);
            EntityIterator subs = graph.Shareds(start);
            for (subs.Start(); subs.More(); subs.Next())
            {
                StepContextDependentShapeRepresentation anitem = subs.Value() as StepContextDependentShapeRepresentation;
                if (anitem is null)
                {
                    continue;
                }
                AddInstances(anitem, graph, explored);
            }
            return;
        }

        if (start.IsKind(typeof(StepShapeRepresentation)))
        {
            StepShapeRepresentation sr = start as StepShapeRepresentation;
            int nb = sr.NbItems();
            for (int i = 1; i <= nb; i++)
            {
                RepresentationItem anitem = sr.ItemsValue(i);
                AddInstances(anitem, graph, explored);
            }
            return;
        }

        if (start.IsKind(typeof(StepFacetedBrep))
            || start.IsKind(typeof(StepBrepWithVoids))
            || start.IsKind(typeof(StepManifoldSolidBrep))
            || start.IsKind(typeof(StepShellBasedSurfaceModel))
            || start.IsKind(typeof(StepFacetedBrepAndBrepWithVoids))
            || start.IsKind(typeof(StepGeometricSet))
            || start.IsKind(typeof(StepFaceSurface))
            || start.IsKind(typeof(MappedItem)))
        {
            AddAllSharings(start, graph, explored);
            return;
        }

        if (start.IsKind(typeof(StepContextDependentShapeRepresentation)))
        {
            StepContextDependentShapeRepresentation cdsr = start as StepContextDependentShapeRepresentation;
            RepresentationRelationship srr = cdsr.RepresentationRelation();
            if (srr is null)
            {
                return;
            }

            StepRepresentation rep;
            bool srrReversed = ConstructAssembly.CheckSRRReversesNAUO(graph, cdsr);
            if (srrReversed)
            {
                rep = srr.Rep2();
            }
            else
            {
                rep = srr.Rep1();
            }

            EntityIterator subs = graph.Sharings(rep);
            for (subs.Start(); subs.More(); subs.Next())
            {
                if (subs.Value().IsKind(typeof(StepShapeDefinitionRepresentation)))
                {
                    StepShapeDefinitionRepresentation SDR = subs.Value() as StepShapeDefinitionRepresentation;
                    AddInstances(SDR, graph, explored);
                }
            }
            //???
            return;
        }

        if (start.IsKind(typeof(ShapeRepresentationRelationship)))
        {
            ShapeRepresentationRelationship und = start as ShapeRepresentationRelationship;
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

    public override EntityIterator RootResult(EntityGraph g)
    {
        if (_graph is null || (g.Model() != _graph.Graph().Model()))
        {
            EntityIterator roots = g.RootEntities();
            _graph = new HGraph(g);
            _entities.Destroy();
            for (roots.Start(); roots.More(); roots.Next())
            {
                AddInstances(roots.Value(), g, _entities);
            }
        }

        if (HasInput || HasAlternate())
        {
            EntityIterator select = InputResult(g);
            int nbSelected = select.NbEntities();
            HashSet<Transient> filter = new HashSet<Transient>(nbSelected, ReferenceEqualityComparer.Instance);
            for (select.Start(); select.More(); select.Next())
            {
                filter.Add(select.Value());
            }
            EntityIterator result = new EntityIterator();
            for (_entities.Start(); _entities.More(); _entities.Next())
            {
                if (filter.Contains(_entities.Value()))
                {
                    result.AddItem(_entities.Value());
                }
            }
            return result;
        }
        else
        {
            return _entities;
        }
    }

    public override bool Explore(int level, Transient start, EntityGraph g, EntityIterator explored)
    {
        return false;
    }

    protected override bool HasUniqueResult
    {
        get
        {
            return true;
        }
    }

    public override string ExploreLabel()
    {
        return "Instances";
    }
}
