// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Topology;

namespace Kerf.Engine.Exchange.Step.Construct;

/// <summary>
/// Defines tools for creation and investigation STEP constructs used for representing various kinds of data, such as
/// product and assembly structure, unit contexts, associated information. The creation of these structures is made
/// according to currently active schema (AP203 or AP214 CD2 or DIS). This is taken from parameter write.step.schema
/// </summary>
internal static class StepConstruct
{
    /// <summary>
    /// Returns STEP entity of the (sub)type of RepresentationItem which is a result of the transfer of a Shape, or
    /// Null if not found
    /// </summary>
    public static RepresentationItem FindEntity(FinderProcess finderProcess, Shape shape)
    {
        RepresentationItem item = null;
        ShapeMapper mapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(finderProcess, shape);
        finderProcess.FindTypedTransient(mapper, typeof(RepresentationItem), ref item);
        // OCCT_DEBUG output left out
        return item;
    }

    /// <summary>
    /// The same as above, but in the case if item not found, repeats search on the same shape without location. The
    /// Loc corresponds to the location with which result is found (either location of the Shape, or Null)
    /// </summary>
    public static RepresentationItem FindEntity(FinderProcess finderProcess, Shape shape,
                                                         ref Placement loc)
    {
        RepresentationItem item = null;
        loc = shape.Placement;
        ShapeMapper mapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(finderProcess, shape);
        if (!finderProcess.FindTypedTransient(mapper, typeof(RepresentationItem), ref item) && !loc.IsIdentity)
        {
            loc = new Placement();
            Shape s = shape.Located(loc);
            mapper = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeMapper(finderProcess, s);
            finderProcess.FindTypedTransient(mapper, typeof(RepresentationItem), ref item);
        }
        // OCCT_DEBUG output left out
        return item;
    }

    /// <summary>
    /// Returns Shape resulting from given STEP entity (Null if not mapped)
    /// </summary>
    public static Shape FindShape(TransientProcess transientProcess, RepresentationItem item)
    {
        Shape s = new Shape();
        Binder binder = transientProcess.Find(item);
        if (binder is not null && binder.HasResult)
        {
            s = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(transientProcess, binder);
        }
        return s;
    }

    /// <summary>
    /// Find CDSR correcponding to the component in the specified assembly
    /// </summary>
    public static bool FindCDSR(Binder componentBinder, StepShapeDefinitionRepresentation assemblySDR,
                                out StepContextDependentShapeRepresentation componentCDSR)
    {
        bool result = false;
        componentCDSR = null;

        PropertyDefinition propD = assemblySDR.Definition().PropertyDefinition();
        if (propD is not null)
        {
            ProductDefinition assemblyPD = propD.Definition().ProductDefinition();
            if (assemblyPD is not null)
            {
                Binder binder = componentBinder;
                while (binder is not null && !result)
                {
                    SimpleBinderOfTransient trb = binder as SimpleBinderOfTransient;
                    if (trb is not null)
                    {
                        componentCDSR = trb.Result() as StepContextDependentShapeRepresentation;
                        if (componentCDSR is not null)
                        {
                            ProductDefinitionShape pds = componentCDSR.RepresentedProductRelation();
                            if (pds is not null)
                            {
                                ProductDefinitionRelationship nauo = pds.Definition().ProductDefinitionRelationship();
                                if (nauo is not null)
                                {
                                    ProductDefinition componentPD = nauo.RelatingProductDefinition();
                                    result = componentPD == assemblyPD;
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
