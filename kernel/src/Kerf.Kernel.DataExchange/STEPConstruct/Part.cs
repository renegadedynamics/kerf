// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Part.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_Part.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine;
using static Kerf.Engine.Exchange.Step.Basic.Source;

namespace Kerf.Engine.Exchange.Step.Construct;

/// <summary>
/// Provides tools for creating STEP structures associated with part (SDR), such as PRODUCT, PDF etc., as required by
/// current schema. Also allows to investigate and modify this data.
/// </summary>
/// <remarks>A C++ value class, a C# class here (decision 1185).</remarks>
internal class Part
{
    //: k8 abv 6 Jan 99: unique names for PRODUCT_DEFINITION_FORMATIONs
    //: k9 abv 6 Jan 99: TR10: eliminating duplicated APPLICATION_CONTEXT entities
    //: j4 gka 16.03.99 S4134
    //    abv 20.11.99 renamed from StepPDR_SDRtool

    public Part()
    {
        _done = false;
    }

    public void MakeSDR(StepShapeRepresentation sr, TCollection_HAsciiString aName, ApplicationContext ac,
                        StepModel stepModel)
    {
        // get current schema
        int schema = (int)stepModel.InternalParameters.WriteSchema;

        // create PC
        ProductContext pc;
        switch (schema)
        {
            default:
            case 1:
                pc = new MechanicalContext();
                break;
            case 4:
            case 5:
            case 2:
                pc = new ProductContext();
                break;
            case 3:
                pc = new MechanicalContext();
                break;
        }
        TCollection_HAsciiString pCname = new TCollection_HAsciiString("");
        TCollection_HAsciiString pCdisciplineType = new TCollection_HAsciiString("mechanical");
        pc.Init(pCname, ac, pCdisciplineType);

        // create product
        Product p = new Product();
        NCollection_HArray1<ProductContext> pCs = new NCollection_HArray1<ProductContext>(1, 1);
        pCs.SetValue(1, pc);
        TCollection_HAsciiString pdescription = new TCollection_HAsciiString("");
        p.Init(aName, aName, pdescription, pCs);

        // create PDF
        ProductDefinitionFormation pdf;
        switch (schema)
        {
            default:
            case 1:
            case 2:
            case 5:
                pdf = new ProductDefinitionFormation();
                break;
            case 3:
                pdf = new ProductDefinitionFormationWithSpecifiedSource();
                (pdf as ProductDefinitionFormationWithSpecifiedSource).SetMakeOrBuy(Source.SNotKnown);
                break;
        }
        TCollection_HAsciiString pdfName = new TCollection_HAsciiString("");
        TCollection_HAsciiString pdFdescription = new TCollection_HAsciiString("");
        pdf.Init(pdfName, pdFdescription, p);

        // create PDC, depending on current schema
        ProductDefinitionContext pdc;
        TCollection_HAsciiString pdCname;
        switch (schema)
        {
            default:
            case 1:
            case 2:
            case 5:
                pdc = new ProductDefinitionContext();
                pdCname = new TCollection_HAsciiString("part definition");
                break;
            case 3:
                pdc = new DesignContext();
                pdCname = new TCollection_HAsciiString("");
                break;
        }
        TCollection_HAsciiString pdClifeCycleStage = new TCollection_HAsciiString("design");
        pdc.Init(pdCname, ac, pdClifeCycleStage);

        // create PD
        ProductDefinition pd = new ProductDefinition();
        TCollection_HAsciiString pdId = new TCollection_HAsciiString("design");
        TCollection_HAsciiString pDdescription = new TCollection_HAsciiString("");
        pd.Init(pdId, pDdescription, pdf, pdc);

        // create PDS
        ProductDefinitionShape pds = new ProductDefinitionShape();
        TCollection_HAsciiString pdSname = new TCollection_HAsciiString("");
        TCollection_HAsciiString pdSdescription = new TCollection_HAsciiString("");
        CharacterizedDefinition cd = new CharacterizedDefinition();
        cd.SetValue(pd);
        pds.Init(pdSname, true, pdSdescription, cd);

        // finally, create SDR
        _sdr = new StepShapeDefinitionRepresentation();
        RepresentedDefinition rd = new RepresentedDefinition();
        rd.SetValue(pds);
        _sdr.Init(rd, sr);

        // and an associated PRPC
        TCollection_HAsciiString prpcName;
        switch ((int)stepModel.InternalParameters.WriteSchema)
        {
            default:
            case 1:
                _prpc = new ProductType();
                prpcName = new TCollection_HAsciiString("part");
                break;
            case 4:
            case 2:
            case 5:
                _prpc = new ProductRelatedProductCategory();
                prpcName = new TCollection_HAsciiString("part");
                break;
            case 3:
                _prpc = new ProductRelatedProductCategory();
                prpcName = new TCollection_HAsciiString("detail"); // !!!!! or "assembly"
                break;
        }
        NCollection_HArray1<Product> prpCproducts = new NCollection_HArray1<Product>(1, 1);
        prpCproducts.SetValue(1, p);
        _prpc.Init(prpcName, false, null, prpCproducts);

        _done = true;
    }

    public void ReadSDR(StepShapeDefinitionRepresentation aShape)
    {
        _sdr = aShape;
        _done = (_sdr is not null);
    }

    public bool IsDone
    {
        get
        {
            return _done;
        }
    }

    /// <summary>Returns SDR as the main result.</summary>
    public StepShapeDefinitionRepresentation SDRValue()
    {
        return _sdr;
    }

    /// <summary>Returns SDR->UsedRepresentation() cast to SR.</summary>
    public StepShapeRepresentation SRValue()
    {
        if (!_done)
        {
            return null;
        }
        return _sdr.UsedRepresentation() as StepShapeRepresentation;
    }

    public ProductDefinitionShape PDS()
    {
        return _sdr.Definition().PropertyDefinition() as ProductDefinitionShape;
    }

    public TCollection_HAsciiString PDSname()
    {
        return _sdr.Definition().PropertyDefinition().Name();
    }

    public TCollection_HAsciiString PDSdescription()
    {
        return _sdr.Definition().PropertyDefinition().Description();
    }

    public void SetPDSname(TCollection_HAsciiString label)
    {
        _sdr.Definition().PropertyDefinition().SetName(label);
    }

    public void SetPDSdescription(TCollection_HAsciiString text)
    {
        _sdr.Definition().PropertyDefinition().SetDescription(text);
    }

    public ProductRelatedProductCategory PRPC()
    {
        return _prpc;
    }

    public TCollection_HAsciiString PRPCname()
    {
        return _prpc.Name();
    }

    public TCollection_HAsciiString PRPCdescription()
    {
        return _prpc.Description();
    }

    public void SetPRPCname(TCollection_HAsciiString text)
    {
        _prpc.SetName(text);
    }

    public void SetPRPCdescription(TCollection_HAsciiString text)
    {
        _prpc.SetDescription(text);
    }

    public ProductContext PC()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1);
    }

    public TCollection_HAsciiString PCname()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1).Name();
    }

    public TCollection_HAsciiString PCdisciplineType()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1).DisciplineType();
    }

    public ApplicationContext AC()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1).FrameOfReference();
    }

    public TCollection_HAsciiString ACapplication()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1).FrameOfReference().Application();
    }

    public ProductDefinitionContext PDC()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().FrameOfReference();
    }

    public TCollection_HAsciiString PDCname()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().FrameOfReference().Name();
    }

    public TCollection_HAsciiString PDCstage()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().FrameOfReference().LifeCycleStage();
    }

    public Product Product()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct();
    }

    public TCollection_HAsciiString Pid()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().Id();
    }

    public TCollection_HAsciiString Pname()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().Name();
    }

    public TCollection_HAsciiString Pdescription()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().Description();
    }

    public ProductDefinitionFormation PDF()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation();
    }

    public TCollection_HAsciiString PDFid()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().Id();
    }

    public TCollection_HAsciiString PDFdescription()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().Description();
    }

    public ProductDefinition PD()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition();
    }

    public TCollection_HAsciiString PDdescription()
    {
        return _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Description();
    }

    public void SetPCname(TCollection_HAsciiString name)
    {
        _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1).SetName(name);
    }

    public void SetPCdisciplineType(TCollection_HAsciiString label)
    {
        _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1).SetDisciplineType(label);
    }

    public void SetACapplication(TCollection_HAsciiString text)
    {
        _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().FrameOfReferenceValue(1).FrameOfReference().SetApplication(text);
    }

    public void SetPDCname(TCollection_HAsciiString label)
    {
        _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().FrameOfReference().SetName(label);
    }

    public void SetPDCstage(TCollection_HAsciiString label)
    {
        _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().FrameOfReference().SetLifeCycleStage(label);
    }

    public void SetPid(TCollection_HAsciiString id)
    {
        _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().SetId(id);
    }

    public void SetPname(TCollection_HAsciiString label)
    {
        _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().SetName(label);
    }

    public void SetPdescription(TCollection_HAsciiString text)
    {
        _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().OfProduct().SetDescription(text);
    }

    public void SetPDFid(TCollection_HAsciiString id)
    {
        _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().SetId(id);
    }

    public void SetPDFdescription(TCollection_HAsciiString text)
    {
        _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().Formation().SetDescription(text);
    }

    public void SetPDdescription(TCollection_HAsciiString text)
    {
        _sdr.Definition().PropertyDefinition().Definition().ProductDefinition().SetDescription(text);
    }

    private bool _done;
    private StepShapeDefinitionRepresentation _sdr;
    private ProductRelatedProductCategory _prpc;
}
