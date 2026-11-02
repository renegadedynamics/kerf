// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Assembly.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Assembly.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This operator creates and checks an item of an assembly, from its basic data : a ShapeRepresentation, a Location
/// ...
/// </summary>
/// <remarks>
/// A C++ value class, a C# class here (decision 1185). OCCT numbers the NEXT_ASSEMBLY_USAGE_OCCURRENCEs it makes from a
/// process-wide counter (a function static); the port's writer passes its own counter
/// (<see cref="MakeRelationship(ref int)"/>, decision 13203), and <see cref="MakeRelationship()"/> keeps a
/// process-wide one for other callers.
/// </remarks>
public class STEPConstruct_Assembly
{
    //: k8 abv 06.01.99: TR10: writing unique names for NAUOs
    // :j4 16.03.99 gka S4134
    // abv 18.11.99 renamed from StepPDR_MakeItem

    // OCCT's "static int id" of MakeRelationship
    private static int theProcessNauoId;

    public STEPConstruct_Assembly()
    {
    }

    /// <summary>
    /// Initialises with starting values Ax0 : origin axis (typically, standard XYZ) Loc : location to which place the
    /// item Makes a MappedItem Resulting Value is returned by ItemValue
    /// </summary>
    public void Init(StepShape_ShapeDefinitionRepresentation aSDR, StepShape_ShapeDefinitionRepresentation SDR0,
                     StepGeom_Axis2Placement3d Ax0, StepGeom_Axis2Placement3d AxLoc)
    {
        thesdr = aSDR;
        thesdr0 = SDR0;
        thesr = aSDR.UsedRepresentation() as StepShape_ShapeRepresentation;
        thesr0 = SDR0.UsedRepresentation() as StepShape_ShapeRepresentation;
        theval = null;
        theax0 = Ax0;
        theloc = AxLoc;
        myIsCartesianTrsf = false;
    }

    /// <summary>
    /// Initialises with starting values theTrsfOp : cartesian transformation operator Makes a MappedItem Resulting
    /// Value is returned by ItemValue
    /// </summary>
    public void Init(StepShape_ShapeDefinitionRepresentation aSDR, StepShape_ShapeDefinitionRepresentation SDR0,
                     StepGeom_CartesianTransformationOperator3d theTrsfOp)
    {
        thesdr = aSDR;
        thesdr0 = SDR0;
        thesr = aSDR.UsedRepresentation() as StepShape_ShapeRepresentation;
        thesr0 = SDR0.UsedRepresentation() as StepShape_ShapeRepresentation;
        theval = null;
        myTrsfOp = theTrsfOp;
        myIsCartesianTrsf = true;
    }

    /// <summary>
    /// Make a (ShapeRepresentationRelationship,...WithTransformation) Resulting Value is returned by ItemValue
    /// </summary>
    /// <remarks>OCCT.NET: numbers the occurrence from a process-wide counter, as OCCT.</remarks>
    public void MakeRelationship()
    {
        MakeRelationship(Interlocked.Increment(ref theProcessNauoId));
    }

    /// <summary>
    /// As <see cref="MakeRelationship()"/>, numbering the occurrence from the caller's counter (OCCT.NET: the
    /// writer's own, decision 13203).
    /// </summary>
    public void MakeRelationship(ref int theNauoId)
    {
        MakeRelationship(++theNauoId);
    }

    private void MakeRelationship(int id)
    {
        // get PDs for assembly (sdr0) and component (sdr)
        StepBasic_ProductDefinition PDED = thesdr.Definition().PropertyDefinition().Definition().ProductDefinition();
        StepBasic_ProductDefinition PDING = thesdr0.Definition().PropertyDefinition().Definition().ProductDefinition();

        // create NAUO
        //: k8 abv 06 Jan 99: TR10: writing unique names for NAUOs  !!!!!
        StepRepr_NextAssemblyUsageOccurrence NAUO = new StepRepr_NextAssemblyUsageOccurrence();
        TCollection_HAsciiString ocid = new TCollection_HAsciiString(id);
        TCollection_HAsciiString ocname = new TCollection_HAsciiString("");
        TCollection_HAsciiString ocdesc = new TCollection_HAsciiString("");
        TCollection_HAsciiString refdes = null; // reste nulle
        NAUO.Init(ocid, ocname, true, ocdesc, PDING, PDED, false, refdes);

        // create PDS for link CDSR->PDS->NAUO
        StepRepr_ProductDefinitionShape PDS = new StepRepr_ProductDefinitionShape();
        TCollection_HAsciiString pdsname = new TCollection_HAsciiString("Placement");
        TCollection_HAsciiString pdsdesc = new TCollection_HAsciiString("Placement of an item");
        StepRepr_CharacterizedDefinition CD = new StepRepr_CharacterizedDefinition();
        CD.SetValue(NAUO);
        PDS.Init(pdsname, true, pdsdesc, CD);

        // create transformation
        Standard_Transient aTrsfItem;
        if (!myIsCartesianTrsf)
        {
            StepRepr_ItemDefinedTransformation ItemDef = new StepRepr_ItemDefinedTransformation();
            TCollection_HAsciiString idname = new TCollection_HAsciiString("");
            TCollection_HAsciiString idescr = new TCollection_HAsciiString("");
            ItemDef.Init(idname, idescr, theax0, theloc);
            aTrsfItem = ItemDef;
        }
        else
        {
            aTrsfItem = myTrsfOp;
        }

        // create SRRWT
        StepRepr_ShapeRepresentationRelationshipWithTransformation SRRWT =
            new StepRepr_ShapeRepresentationRelationshipWithTransformation();
        TCollection_HAsciiString stname = new TCollection_HAsciiString("");
        TCollection_HAsciiString stescr = new TCollection_HAsciiString("");
        StepRepr_Transformation StepTrans = new StepRepr_Transformation();
        StepTrans.SetValue(aTrsfItem);
        SRRWT.Init(stname, stescr, thesr, thesr0, StepTrans);

        // create CDSR (final result, root)
        StepShape_ContextDependentShapeRepresentation CDSR = new StepShape_ContextDependentShapeRepresentation();
        CDSR.Init(SRRWT, PDS);

        theval = CDSR;
    }

    /// <summary>Returns the Value If no Make... has been called, returns the starting SR</summary>
    public Standard_Transient ItemValue()
    {
        if (theval is null)
        {
            return thesr;
        }
        return theval;
    }

    /// <summary>Returns the location of the item, computed from starting aLoc</summary>
    public StepGeom_Axis2Placement3d ItemLocation()
    {
        return theloc;
    }

    /// <summary>Returns NAUO object describing the assembly link</summary>
    public StepRepr_NextAssemblyUsageOccurrence GetNAUO()
    {
        StepShape_ContextDependentShapeRepresentation CDSR = ItemValue() as StepShape_ContextDependentShapeRepresentation;
        if (CDSR is not null)
        {
            StepBasic_ProductDefinitionRelationship PDR =
                CDSR.RepresentedProductRelation().Definition().ProductDefinitionRelationship();
            return PDR as StepRepr_NextAssemblyUsageOccurrence;
        }
        return null;
    }

    /// <summary>
    /// Checks whether SRR's definition of assembly and component contradicts with NAUO definition or not, according
    /// to model schema (AP214 or AP203)
    /// </summary>
    public static bool CheckSRRReversesNAUO(Interface_Graph theGraph, StepShape_ContextDependentShapeRepresentation CDSR)
    {
        StepRepr_NextAssemblyUsageOccurrence nauo =
            CDSR.RepresentedProductRelation().Definition().ProductDefinitionRelationship() as StepRepr_NextAssemblyUsageOccurrence;
        if (nauo is null)
        {
            return false;
        }

        StepBasic_ProductDefinition pd1 = null, pd2 = null;
        StepRepr_Representation rep1 = CDSR.RepresentationRelation().Rep1();
        StepRepr_Representation rep2 = CDSR.RepresentationRelation().Rep2();
        if (rep1 is null || rep2 is null)
        {
            return false;
        }

        // find SDRs corresponding to Rep1 and Rep2 and remember their PDs
        Type tSDR = typeof(StepShape_ShapeDefinitionRepresentation);
        Interface_EntityIterator anIter = theGraph.Sharings(rep1);
        for (; anIter.More() && pd1 is null; anIter.Next())
        {
            Standard_Transient enti = anIter.Value();
            if (enti.DynamicType() == tSDR)
            {
                StepShape_ShapeDefinitionRepresentation SDR = enti as StepShape_ShapeDefinitionRepresentation;
                if (SDR.UsedRepresentation() == rep1)
                {
                    StepRepr_PropertyDefinition aPD = SDR.Definition().PropertyDefinition();
                    if (aPD is not null)
                    {
                        pd1 = aPD.Definition().ProductDefinition();
                    }
                }
            }
        }

        anIter = theGraph.Sharings(rep2);
        for (; anIter.More() && pd2 is null; anIter.Next())
        {
            Standard_Transient enti = anIter.Value();
            if (enti.DynamicType() == tSDR)
            {
                StepShape_ShapeDefinitionRepresentation SDR = enti as StepShape_ShapeDefinitionRepresentation;
                if (SDR.UsedRepresentation() == rep2)
                {
                    StepRepr_PropertyDefinition aPD = SDR.Definition().PropertyDefinition();
                    if (aPD is not null)
                    {
                        pd2 = aPD.Definition().ProductDefinition();
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

    private StepShape_ShapeDefinitionRepresentation thesdr;
    private StepShape_ShapeDefinitionRepresentation thesdr0;
    private StepShape_ShapeRepresentation thesr;
    private StepShape_ShapeRepresentation thesr0;
    private Standard_Transient theval;
    private StepGeom_Axis2Placement3d theloc;
    private StepGeom_Axis2Placement3d theax0;
    private StepGeom_CartesianTransformationOperator3d myTrsfOp;
    private bool myIsCartesianTrsf;
}
