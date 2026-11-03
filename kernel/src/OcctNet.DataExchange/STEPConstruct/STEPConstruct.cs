// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Topology;

namespace Occt;

/// <summary>
/// Defines tools for creation and investigation STEP constructs used for representing various kinds of data, such as
/// product and assembly structure, unit contexts, associated information. The creation of these structures is made
/// according to currently active schema (AP203 or AP214 CD2 or DIS). This is taken from parameter write.step.schema
/// </summary>
public static class STEPConstruct
{
    /// <summary>
    /// Returns STEP entity of the (sub)type of RepresentationItem which is a result of the transfer of a Shape, or
    /// Null if not found
    /// </summary>
    public static StepRepr_RepresentationItem FindEntity(Transfer_FinderProcess FinderProcess, Shape Shape)
    {
        StepRepr_RepresentationItem item = null;
        TransferBRep_ShapeMapper mapper = TransferBRep.ShapeMapper(FinderProcess, Shape);
        FinderProcess.FindTypedTransient(mapper, typeof(StepRepr_RepresentationItem), ref item);
        // OCCT_DEBUG output left out
        return item;
    }

    /// <summary>
    /// The same as above, but in the case if item not found, repeats search on the same shape without location. The
    /// Loc corresponds to the location with which result is found (either location of the Shape, or Null)
    /// </summary>
    public static StepRepr_RepresentationItem FindEntity(Transfer_FinderProcess FinderProcess, Shape Shape,
                                                         ref Placement Loc)
    {
        StepRepr_RepresentationItem item = null;
        Loc = Shape.Location();
        TransferBRep_ShapeMapper mapper = TransferBRep.ShapeMapper(FinderProcess, Shape);
        if (!FinderProcess.FindTypedTransient(mapper, typeof(StepRepr_RepresentationItem), ref item) && !Loc.IsIdentity)
        {
            Loc = new Placement();
            Shape S = Shape.Located(Loc);
            mapper = TransferBRep.ShapeMapper(FinderProcess, S);
            FinderProcess.FindTypedTransient(mapper, typeof(StepRepr_RepresentationItem), ref item);
        }
        // OCCT_DEBUG output left out
        return item;
    }

    /// <summary>
    /// Returns Shape resulting from given STEP entity (Null if not mapped)
    /// </summary>
    public static Shape FindShape(Transfer_TransientProcess TransientProcess, StepRepr_RepresentationItem item)
    {
        Shape S = new Shape();
        Transfer_Binder binder = TransientProcess.Find(item);
        if (binder is not null && binder.HasResult())
        {
            S = TransferBRep.ShapeResult(TransientProcess, binder);
        }
        return S;
    }

    /// <summary>
    /// Find CDSR correcponding to the component in the specified assembly
    /// </summary>
    public static bool FindCDSR(Transfer_Binder ComponentBinder, StepShape_ShapeDefinitionRepresentation AssemblySDR,
                                out StepShape_ContextDependentShapeRepresentation ComponentCDSR)
    {
        bool result = false;
        ComponentCDSR = null;

        StepRepr_PropertyDefinition PropD = AssemblySDR.Definition().PropertyDefinition();
        if (PropD is not null)
        {
            StepBasic_ProductDefinition AssemblyPD = PropD.Definition().ProductDefinition();
            if (AssemblyPD is not null)
            {
                Transfer_Binder binder = ComponentBinder;
                while (binder is not null && !result)
                {
                    Transfer_SimpleBinderOfTransient trb = binder as Transfer_SimpleBinderOfTransient;
                    if (trb is not null)
                    {
                        ComponentCDSR = trb.Result() as StepShape_ContextDependentShapeRepresentation;
                        if (ComponentCDSR is not null)
                        {
                            StepRepr_ProductDefinitionShape PDS = ComponentCDSR.RepresentedProductRelation();
                            if (PDS is not null)
                            {
                                StepBasic_ProductDefinitionRelationship NAUO = PDS.Definition().ProductDefinitionRelationship();
                                if (NAUO is not null)
                                {
                                    StepBasic_ProductDefinition ComponentPD = NAUO.RelatingProductDefinition();
                                    result = ComponentPD == AssemblyPD;
                                }
                            }
                        }
                    }
                    binder = binder.NextResult();
                }
            }
        }
        return result;
    }
}
