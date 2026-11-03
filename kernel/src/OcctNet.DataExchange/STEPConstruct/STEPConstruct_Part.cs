// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Part.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Part.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.StepBasic_Source;

namespace Occt;

/// <summary>
/// Provides tools for creating STEP structures associated with part (SDR), such as PRODUCT, PDF etc., as required by
/// current schema. Also allows to investigate and modify this data.
/// </summary>
/// <remarks>A C++ value class, a C# class here (decision 1185).</remarks>
public class STEPConstruct_Part
{
    //: k8 abv 6 Jan 99: unique names for PRODUCT_DEFINITION_FORMATIONs
    //: k9 abv 6 Jan 99: TR10: eliminating duplicated APPLICATION_CONTEXT entities
    //: j4 gka 16.03.99 S4134
    //    abv 20.11.99 renamed from StepPDR_SDRtool

    public STEPConstruct_Part()
    {
        myDone = false;
    }

    public void MakeSDR(StepShape_ShapeRepresentation SR, TCollection_HAsciiString aName, StepBasic_ApplicationContext AC,
                        StepData_StepModel theStepModel)
    {
        // get current schema
        int schema = (int)theStepModel.InternalParameters.WriteSchema;

        // create PC
        StepBasic_ProductContext PC;
        switch (schema)
        {
            default:
            case 1:
                PC = new StepBasic_MechanicalContext();
                break;
            case 4:
            case 5:
            case 2:
                PC = new StepBasic_ProductContext();
                break;
            case 3:
                PC = new StepBasic_MechanicalContext();
                break;
        }
        TCollection_HAsciiString PCname = new TCollection_HAsciiString("");
        TCollection_HAsciiString PCdisciplineType = new TCollection_HAsciiString("mechanical");
        PC.Init(PCname, AC, PCdisciplineType);

        // create product
        StepBasic_Product P = new StepBasic_Product();
        NCollection_HArray1<StepBasic_ProductContext> PCs = new NCollection_HArray1<StepBasic_ProductContext>(1, 1);
        PCs.SetValue(1, PC);
        TCollection_HAsciiString Pdescription = new TCollection_HAsciiString("");
        P.Init(aName, aName, Pdescription, PCs);

        // create PDF
        StepBasic_ProductDefinitionFormation PDF;
        switch (schema)
        {
            default:
            case 1:
            case 2:
            case 5:
                PDF = new StepBasic_ProductDefinitionFormation();
                break;
            case 3:
                PDF = new StepBasic_ProductDefinitionFormationWithSpecifiedSource();
                (PDF as StepBasic_ProductDefinitionFormationWithSpecifiedSource).SetMakeOrBuy(StepBasic_Source.StepBasic_sNotKnown);
                break;
        }
        TCollection_HAsciiString PDFName = new TCollection_HAsciiString("");
        TCollection_HAsciiString PDFdescription = new TCollection_HAsciiString("");
        PDF.Init(PDFName, PDFdescription, P);

        // create PDC, depending on current schema
        StepBasic_ProductDefinitionContext PDC;
        TCollection_HAsciiString PDCname;
        switch (schema)
        {
            default:
            case 1:
            case 2:
            case 5:
                PDC = new StepBasic_ProductDefinitionContext();
                PDCname = new TCollection_HAsciiString("part definition");
                break;
            case 3:
                PDC = new StepBasic_DesignContext();
                PDCname = new TCollection_HAsciiString("");
                break;
        }
        TCollection_HAsciiString PDClifeCycleStage = new TCollection_HAsciiString("design");
        PDC.Init(PDCname, AC, PDClifeCycleStage);

        // create PD
        StepBasic_ProductDefinition PD = new StepBasic_ProductDefinition();
        TCollection_HAsciiString PDId = new TCollection_HAsciiString("design");
        TCollection_HAsciiString PDdescription = new TCollection_HAsciiString("");
        PD.Init(PDId, PDdescription, PDF, PDC);

        // create PDS
        StepRepr_ProductDefinitionShape PDS = new StepRepr_ProductDefinitionShape();
        TCollection_HAsciiString PDSname = new TCollection_HAsciiString("");
        TCollection_HAsciiString PDSdescription = new TCollection_HAsciiString("");
        StepRepr_CharacterizedDefinition CD = new StepRepr_CharacterizedDefinition();
        CD.SetValue(PD);
        PDS.Init(PDSname, true, PDSdescription, CD);

        // finally, create SDR
        mySDR = new StepShape_ShapeDefinitionRepresentation();
        StepRepr_RepresentedDefinition RD = new StepRepr_RepresentedDefinition();
        RD.SetValue(PDS);
        mySDR.Init(RD, SR);

        // and an associated PRPC
        TCollection_HAsciiString PRPCName;
        switch ((int)theStepModel.InternalParameters.WriteSchema)
        {
            default:
            case 1:
                myPRPC = new StepBasic_ProductType();
                PRPCName = new TCollection_HAsciiString("part");
                break;
            case 4:
            case 2:
            case 5:
                myPRPC = new StepBasic_ProductRelatedProductCategory();
                PRPCName = new TCollection_HAsciiString("part");
                break;
            case 3:
                myPRPC = new StepBasic_ProductRelatedProductCategory();
                PRPCName = new TCollection_HAsciiString("detail"); // !!!!! or "assembly"
                break;
        }
        NCollection_HArray1<StepBasic_Product> PRPCproducts = new NCollection_HArray1<StepBasic_Product>(1, 1);
        PRPCproducts.SetValue(1, P);
        myPRPC.Init(PRPCName, false, null, PRPCproducts);

        myDone = true;
    }

    public void ReadSDR(StepShape_ShapeDefinitionRepresentation aShape)
    {
        mySDR = aShape;
        myDone = (mySDR is not null);
    }

    public bool IsDone()
    {
        return myDone;
    }

    /// <summary>Returns SDR as the main result.</summary>
    public StepShape_ShapeDefinitionRepresentation SDRValue()
    {
        return mySDR;
    }

    /// <summary>Returns SDR->UsedRepresentation() cast to SR.</summary>
    public StepShape_ShapeRepresentation SRValue()
    {
        if (!myDone)
        {
            return null;
        }
        return mySDR.UsedRepresentation() as StepShape_ShapeRepresentation;
    }

    public StepRepr_ProductDefinitionShape PDS()
    {
        return mySDR.Definition().PropertyDefinition() as StepRepr_ProductDefinitionShape;
    }

    public TCollection_HAsciiString PDSname()
    {
        return mySDR.Definition().PropertyDefinition().Name();
    }

    public TCollection_HAsciiString PDSdescription()
    {
        return mySDR.Definition().PropertyDefinition().Description();
    }

    public void SetPDSname(TCollection_HAsciiString label)
    {
        mySDR.Definition().PropertyDefinition().SetName(label);
    }

    public void SetPDSdescription(TCollection_HAsciiString text)
    {
        mySDR.Definition().PropertyDefinition().SetDescription(text);
    }

    public StepBasic_ProductRelatedProductCategory PRPC()
    {
        return myPRPC;
    }

    public TCollection_HAsciiString PRPCname()
    {
        return myPRPC.Name();
    }

    public TCollection_HAsciiString PRPCdescription()
    {
        return myPRPC.Description();
    }

    public void SetPRPCname(TCollection_HAsciiString text)
    {
        myPRPC.SetName(text);
    }

    public void SetPRPCdescription(TCollection_HAsciiString text)
    {
        myPRPC.SetDescription(text);
    }

    public StepBasic_ProductContext PC()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1);
    }

    public TCollection_HAsciiString PCname()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1).Name();
    }

    public TCollection_HAsciiString PCdisciplineType()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1).DisciplineType();
    }

    public StepBasic_ApplicationContext AC()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1).FrameOfReference();
    }

    public TCollection_HAsciiString ACapplication()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1).FrameOfReference().Application();
    }

    public StepBasic_ProductDefinitionContext PDC()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().FrameOfReference();
    }

    public TCollection_HAsciiString PDCname()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().FrameOfReference().Name();
    }

    public TCollection_HAsciiString PDCstage()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().FrameOfReference().LifeCycleStage();
    }

    public StepBasic_Product Product()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct();
    }

    public TCollection_HAsciiString Pid()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().Id();
    }

    public TCollection_HAsciiString Pname()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().Name();
    }

    public TCollection_HAsciiString Pdescription()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().Description();
    }

    public StepBasic_ProductDefinitionFormation PDF()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation();
    }

    public TCollection_HAsciiString PDFid()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().Id();
    }

    public TCollection_HAsciiString PDFdescription()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().Description();
    }

    public StepBasic_ProductDefinition PD()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition();
    }

    public TCollection_HAsciiString PDdescription()
    {
        return mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Description();
    }

    public void SetPCname(TCollection_HAsciiString name)
    {
        mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1).SetName(name);
    }

    public void SetPCdisciplineType(TCollection_HAsciiString label)
    {
        mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1).SetDisciplineType(label);
    }

    public void SetACapplication(TCollection_HAsciiString text)
    {
        mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1).FrameOfReference().SetApplication(text);
    }

    public void SetPDCname(TCollection_HAsciiString label)
    {
        mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().FrameOfReference().SetName(label);
    }

    public void SetPDCstage(TCollection_HAsciiString label)
    {
        mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().FrameOfReference().SetLifeCycleStage(label);
    }

    public void SetPid(TCollection_HAsciiString id)
    {
        mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().SetId(id);
    }

    public void SetPname(TCollection_HAsciiString label)
    {
        mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().SetName(label);
    }

    public void SetPdescription(TCollection_HAsciiString text)
    {
        mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().SetDescription(text);
    }

    public void SetPDFid(TCollection_HAsciiString id)
    {
        mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().SetId(id);
    }

    public void SetPDFdescription(TCollection_HAsciiString text)
    {
        mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().SetDescription(text);
    }

    public void SetPDdescription(TCollection_HAsciiString text)
    {
        mySDR.Definition().PropertyDefinition().Definition().ProductDefinition().SetDescription(text);
    }

    private bool myDone;
    private StepShape_ShapeDefinitionRepresentation mySDR;
    private StepBasic_ProductRelatedProductCategory myPRPC;
}
