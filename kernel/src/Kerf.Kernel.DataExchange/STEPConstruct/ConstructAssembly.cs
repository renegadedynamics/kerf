// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Assembly.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Assembly.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Step.Construct;

/// <summary>
/// This operator creates and checks an item of an assembly, from its basic data : a ShapeRepresentation, a Location
/// ...
/// </summary>
/// <remarks>
/// A C++ value class, a C# class here (decision 1185). OCCT numbers the NEXT_ASSEMBLY_USAGE_OCCURRENCEs it makes from a
/// process-wide counter (a function static); the port's writer passes its own counter
/// (<see cref="MakeRelationship(ref int)"/>, decision 1266), and <see cref="MakeRelationship()"/> keeps a
/// process-wide one for other callers.
/// </remarks>
public class ConstructAssembly
{
    //: k8 abv 06.01.99: TR10: writing unique names for NAUOs
    // :j4 16.03.99 gka S4134
    // abv 18.11.99 renamed from StepPDR_MakeItem

    // OCCT's "static int id" of MakeRelationship
    private static int s_processNauoId;

    public ConstructAssembly()
    {
    }

    /// <summary>
    /// Initialises with starting values Ax0 : origin axis (typically, standard XYZ) Loc : location to which place the
    /// item Makes a MappedItem Resulting Value is returned by ItemValue
    /// </summary>
    public void Init(StepShapeDefinitionRepresentation aSDR, StepShapeDefinitionRepresentation sdr0,
                     StepAxis2Placement3d ax0, StepAxis2Placement3d axLoc)
    {
        _thesdr = aSDR;
        _thesdr0 = sdr0;
        _thesr = aSDR.UsedRepresentation() as StepShapeRepresentation;
        _thesr0 = sdr0.UsedRepresentation() as StepShapeRepresentation;
        _theval = null;
        _theax0 = ax0;
        _theloc = axLoc;
        _isCartesianTrsf = false;
    }

    /// <summary>
    /// Initialises with starting values theTrsfOp : cartesian transformation operator Makes a MappedItem Resulting
    /// Value is returned by ItemValue
    /// </summary>
    public void Init(StepShapeDefinitionRepresentation aSDR, StepShapeDefinitionRepresentation sdr0,
                     StepCartesianTransformationOperator3d trsfOp)
    {
        _thesdr = aSDR;
        _thesdr0 = sdr0;
        _thesr = aSDR.UsedRepresentation() as StepShapeRepresentation;
        _thesr0 = sdr0.UsedRepresentation() as StepShapeRepresentation;
        _theval = null;
        _trsfOp = trsfOp;
        _isCartesianTrsf = true;
    }

    /// <summary>
    /// Make a (ShapeRepresentationRelationship,...WithTransformation) Resulting Value is returned by ItemValue
    /// </summary>
    /// <remarks>OCCT.NET: numbers the occurrence from a process-wide counter, as OCCT.</remarks>
    public void MakeRelationship()
    {
        MakeRelationship(Interlocked.Increment(ref s_processNauoId));
    }

    /// <summary>
    /// As <see cref="MakeRelationship()"/>, numbering the occurrence from the caller's counter (OCCT.NET: the
    /// writer's own, decision 1266).
    /// </summary>
    public void MakeRelationship(ref int nauoId)
    {
        MakeRelationship(++nauoId);
    }

    private void MakeRelationship(int id)
    {
        // get PDs for assembly (sdr0) and component (sdr)
        ProductDefinition pded = _thesdr.Definition().PropertyDefinition().Definition().ProductDefinition();
        ProductDefinition pding = _thesdr0.Definition().PropertyDefinition().Definition().ProductDefinition();

        // create NAUO
        //: k8 abv 06 Jan 99: TR10: writing unique names for NAUOs  !!!!!
        NextAssemblyUsageOccurrence nauo = new NextAssemblyUsageOccurrence();
        TCollection_HAsciiString ocid = new TCollection_HAsciiString(id);
        TCollection_HAsciiString ocname = new TCollection_HAsciiString("");
        TCollection_HAsciiString ocdesc = new TCollection_HAsciiString("");
        TCollection_HAsciiString refdes = null; // reste nulle
        nauo.Init(ocid, ocname, true, ocdesc, pding, pded, false, refdes);

        // create PDS for link CDSR->PDS->NAUO
        ProductDefinitionShape pds = new ProductDefinitionShape();
        TCollection_HAsciiString pdsname = new TCollection_HAsciiString("Placement");
        TCollection_HAsciiString pdsdesc = new TCollection_HAsciiString("Placement of an item");
        CharacterizedDefinition cd = new CharacterizedDefinition();
        cd.SetValue(nauo);
        pds.Init(pdsname, true, pdsdesc, cd);

        // create transformation
        Transient trsfItem;
        if (!_isCartesianTrsf)
        {
            ItemDefinedTransformation itemDef = new ItemDefinedTransformation();
            TCollection_HAsciiString idname = new TCollection_HAsciiString("");
            TCollection_HAsciiString idescr = new TCollection_HAsciiString("");
            itemDef.Init(idname, idescr, _theax0, _theloc);
            trsfItem = itemDef;
        }
        else
        {
            trsfItem = _trsfOp;
        }

        // create SRRWT
        ShapeRepresentationRelationshipWithTransformation srrwt =
            new ShapeRepresentationRelationshipWithTransformation();
        TCollection_HAsciiString stname = new TCollection_HAsciiString("");
        TCollection_HAsciiString stescr = new TCollection_HAsciiString("");
        Transformation stepTrans = new Transformation();
        stepTrans.SetValue(trsfItem);
        srrwt.Init(stname, stescr, _thesr, _thesr0, stepTrans);

        // create CDSR (final result, root)
        StepContextDependentShapeRepresentation cdsr = new StepContextDependentShapeRepresentation();
        cdsr.Init(srrwt, pds);

        _theval = cdsr;
    }

    /// <summary>Returns the Value If no Make... has been called, returns the starting SR</summary>
    public Transient ItemValue()
    {
        if (_theval is null)
        {
            return _thesr;
        }
        return _theval;
    }

    /// <summary>Returns the location of the item, computed from starting aLoc</summary>
    public StepAxis2Placement3d ItemLocation()
    {
        return _theloc;
    }

    /// <summary>Returns NAUO object describing the assembly link</summary>
    public NextAssemblyUsageOccurrence GetNAUO()
    {
        StepContextDependentShapeRepresentation cdsr = ItemValue() as StepContextDependentShapeRepresentation;
        if (cdsr is not null)
        {
            ProductDefinitionRelationship pdr =
                cdsr.RepresentedProductRelation().Definition().ProductDefinitionRelationship();
            return pdr as NextAssemblyUsageOccurrence;
        }
        return null;
    }

    /// <summary>
    /// Checks whether SRR's definition of assembly and component contradicts with NAUO definition or not, according
    /// to model schema (AP214 or AP203)
    /// </summary>
    public static bool CheckSRRReversesNAUO(EntityGraph graph, StepContextDependentShapeRepresentation cdsr)
    {
        NextAssemblyUsageOccurrence nauo =
            cdsr.RepresentedProductRelation().Definition().ProductDefinitionRelationship() as NextAssemblyUsageOccurrence;
        if (nauo is null)
        {
            return false;
        }

        ProductDefinition pd1 = null, pd2 = null;
        StepRepresentation rep1 = cdsr.RepresentationRelation().Rep1();
        StepRepresentation rep2 = cdsr.RepresentationRelation().Rep2();
        if (rep1 is null || rep2 is null)
        {
            return false;
        }

        // find SDRs corresponding to Rep1 and Rep2 and remember their PDs
        Type tSDR = typeof(StepShapeDefinitionRepresentation);
        EntityIterator iter = graph.Sharings(rep1);
        for (; iter.More() && pd1 is null; iter.Next())
        {
            Transient enti = iter.Value();
            if (enti.DynamicType() == tSDR)
            {
                StepShapeDefinitionRepresentation sdr = enti as StepShapeDefinitionRepresentation;
                if (sdr.UsedRepresentation() == rep1)
                {
                    PropertyDefinition pd = sdr.Definition().PropertyDefinition();
                    if (pd is not null)
                    {
                        pd1 = pd.Definition().ProductDefinition();
                    }
                }
            }
        }

        iter = graph.Sharings(rep2);
        for (; iter.More() && pd2 is null; iter.Next())
        {
            Transient enti = iter.Value();
            if (enti.DynamicType() == tSDR)
            {
                StepShapeDefinitionRepresentation sdr = enti as StepShapeDefinitionRepresentation;
                if (sdr.UsedRepresentation() == rep2)
                {
                    PropertyDefinition pd = sdr.Definition().PropertyDefinition();
                    if (pd is not null)
                    {
                        pd2 = pd.Definition().ProductDefinition();
                    }
                }
            }
        }

        // checks..

        if (pd1 == nauo.RelatedProductDefinition() && // OK
            pd2 == nauo.RelatingProductDefinition())
        {
            return false;
        }

        if (pd2 == nauo.RelatedProductDefinition() && // Reversed
            pd1 == nauo.RelatingProductDefinition())
        {
            return true;
        }

        if (pd2 == nauo.RelatedProductDefinition() || //: k3 abv 25 Nov 98: rp1sd.stp - bad assemblies
            pd1 == nauo.RelatingProductDefinition())
        {
            return true;
        }

        return false;
    }

    private StepShapeDefinitionRepresentation _thesdr;
    private StepShapeDefinitionRepresentation _thesdr0;
    private StepShapeRepresentation _thesr;
    private StepShapeRepresentation _thesr0;
    private Transient _theval;
    private StepAxis2Placement3d _theloc;
    private StepAxis2Placement3d _theax0;
    private StepCartesianTransformationOperator3d _trsfOp;
    private bool _isCartesianTrsf;
}
