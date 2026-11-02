// Port of: src/DataExchange/TKDESTEP/STEPEdit/STEPEdit.hxx
// Port of: src/DataExchange/TKDESTEP/STEPEdit/STEPEdit.cxx
// Port of: src/DataExchange/TKDESTEP/STEPEdit/STEPEdit_EditContext.hxx
// Port of: src/DataExchange/TKDESTEP/STEPEdit/STEPEdit_EditContext.cxx
// Port of: src/DataExchange/TKDESTEP/STEPEdit/STEPEdit_EditSDR.hxx
// Port of: src/DataExchange/TKDESTEP/STEPEdit/STEPEdit_EditSDR.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.IFSelect_EditValue;
using static Occt.Interface_ParamType;

namespace Occt;

/// <summary>
/// Provides tools to exploit and edit a set of STEP data : editors, selections ..
/// </summary>
/// <remarks>
/// OCCT.NET: <see cref="SignType"/>'s signature, created once in OCCT under a mutex and shared by every caller, is
/// created once too (a static readonly field); it doesn't change after creation (decision 1215).
/// </remarks>
public static class STEPEdit
{
    private static readonly StepSelect_StepType theSignType = CreateSignType();

    private static StepSelect_StepType CreateSignType()
    {
        // OCCT: the controller, which calls SignType, has registered the modules before
        RWHeaderSection.Init();
        RWStepAP214.Init();
        StepSelect_StepType sty = new StepSelect_StepType();
        sty.SetProtocol(Protocol());
        return sty;
    }

    /// <summary>Returns a Protocol fit for STEP (creates the first time)</summary>
    public static Interface_Protocol Protocol()
    {
        return StepAP214.Protocol();
    }

    /// <summary>Returns a new empty StepModel fit for STEP i.e. with its header determined from Protocol</summary>
    public static StepData_StepModel NewModel()
    {
        StepData_StepModel stepmodel = new StepData_StepModel();
        stepmodel.SetProtocol(Protocol());
        return stepmodel;
    }

    /// <summary>Returns a SignType fit for STEP (creates the first time)</summary>
    public static IFSelect_Signature SignType()
    {
        return theSignType;
    }

    /// <summary>
    /// Creates a Selection for ShapeDefinitionRepresentation By default searches among root entities
    /// </summary>
    public static IFSelect_SelectSignature NewSelectSDR()
    {
        IFSelect_SelectSignature sel = new IFSelect_SelectSignature(SignType(), "SHAPE_DEFINITION_REPRESENTATION");
        sel.SetInput(new IFSelect_SelectModelRoots());
        return sel;
    }

    /// <summary>
    /// Creates a Selection for Placed Items, i.e. MappedItem or ContextDependentShapeRepresentation, which itself
    /// refers to a RepresentationRelationship with possible subtypes (ShapeRepresentationRelationship and/or
    /// RepresentationRelationshipWithTransformation) By default searches among root entities
    /// </summary>
    public static IFSelect_SelectSignature NewSelectPlacedItem()
    {
        IFSelect_SelectSignature sel = new IFSelect_SelectSignature(SignType(),
                                                                    "MAPPED_ITEM|CONTEXT_DEPENDENT_SHAPE_REPRESENTATION",
                                                                    false);
        sel.SetInput(new IFSelect_SelectModelEntities());
        return sel;
    }

    /// <summary>
    /// Creates a Selection for ShapeRepresentation and its sub-types, plus ContextDependentShapeRepresentation (which is
    /// not a sub-type of ShapeRepresentation) By default searches among root entities
    /// </summary>
    public static IFSelect_SelectSignature NewSelectShapeRepr()
    {
        IFSelect_SelectSignature sel = new IFSelect_SelectSignature(SignType(), "SHAPE_REPRESENTATION", false);
        // REPRESENTATION_RELATIONSHIP passe par CONTEXT_DEPENDENT_SHAPE_REPRESENTATION
        sel.SetInput(new IFSelect_SelectModelEntities());
        return sel;
    }
}

/// <summary>
/// EditContext is an Editor fit for Product Definition Context (one per Model) , i.e. : - ProductDefinition -
/// ApplicationProtocolDefinition - ProductRelatedProductCategory
/// </summary>
public class STEPEdit_EditContext : IFSelect_Editor
{
    /// <summary>Creates an EditContext</summary>
    public STEPEdit_EditContext() : base(5)
    {
        Interface_TypedValue ac_val = new Interface_TypedValue("Application Context", Interface_ParamIdent);
        SetValue(1, ac_val, "AC", IFSelect_EditRead);

        Interface_TypedValue ac_status = new Interface_TypedValue("AppliContext_Status");
        SetValue(2, ac_status, "AC_Status");
        Interface_TypedValue ac_sche = new Interface_TypedValue("AppliContext_Schema");
        SetValue(3, ac_sche, "AC_Schema");
        Interface_TypedValue ac_year = new Interface_TypedValue("AppliContext_Year", Interface_ParamInteger);
        SetValue(4, ac_year, "AC_Year");
        Interface_TypedValue ac_name = new Interface_TypedValue("AppliContext_Name");
        SetValue(5, ac_name, "AC_Name");
    }

    public override string Label()
    {
        return "STEP : Product Definition Context";
    }

    public override bool Recognize(IFSelect_EditForm form)
    {
        // il faut 17 parametres
        return true;
    }

    public override TCollection_HAsciiString StringValue(IFSelect_EditForm form, int num)
    {
        TCollection_HAsciiString str = null;
        switch (num)
        {
            case 2:
                return new TCollection_HAsciiString("DIS");
            case 3:
                return new TCollection_HAsciiString("automotive_design");
            case 4:
                return new TCollection_HAsciiString("1998");
            case 5:
                return new TCollection_HAsciiString("EUCLID");
            case 7:
                return new TCollection_HAsciiString("Undefined Category");
            case 8:
                return new TCollection_HAsciiString("Undefined Description");
            default:
                break;
        }
        return str;
    }

    public override bool Load(IFSelect_EditForm form, Standard_Transient ent, Interface_InterfaceModel model)
    {
        StepData_StepModel modl = model as StepData_StepModel;
        if (modl is null)
        {
            return false;
        }

        STEPConstruct_ContextTool ctx = new STEPConstruct_ContextTool(modl);
        StepData_Factors aLocalFactors = new StepData_Factors();
        ctx.SetGlobalFactor(aLocalFactors);

        form.LoadValue(1, modl.StringLabel(ctx.GetAPD()));

        form.LoadValue(2, ctx.GetACstatus());
        form.LoadValue(3, ctx.GetACschemaName());
        form.LoadValue(4, new TCollection_HAsciiString(ctx.GetACyear()));
        form.LoadValue(5, ctx.GetACname());

        return true;
    }

    public override bool Apply(IFSelect_EditForm form, Standard_Transient ent, Interface_InterfaceModel model)
    {
        StepData_StepModel modl = model as StepData_StepModel;
        if (modl is null)
        {
            return false;
        }

        STEPConstruct_ContextTool ctx = new STEPConstruct_ContextTool(modl);
        StepData_Factors aLocalFactors = new StepData_Factors();
        ctx.SetGlobalFactor(aLocalFactors);

        ctx.AddAPD(); // on ne sait jamais

        if (form.IsModified(2))
        {
            ctx.SetACstatus(form.EditedValue(2));
        }
        if (form.IsModified(3))
        {
            ctx.SetACschemaName(form.EditedValue(3));
        }
        if (form.IsModified(4))
        {
            ctx.SetACyear(form.EditedValue(4).IntegerValue());
        }
        if (form.IsModified(5))
        {
            ctx.SetACname(form.EditedValue(5));
        }

        return true;
    }
}

/// <summary>
/// EditSDR is an Editor fit for a Shape Definition Representation which designates a Product Definition
/// </summary>
public class STEPEdit_EditSDR : IFSelect_Editor
{
    /// <summary>Creates an EditSDR</summary>
    public STEPEdit_EditSDR() : base(11)
    {
        Interface_TypedValue pdc_name = new Interface_TypedValue("PDC_Name");
        SetValue(1, pdc_name, "PDC_Name");
        Interface_TypedValue pdc_lfs = new Interface_TypedValue("PDC_LifeCycleStage");
        SetValue(2, pdc_lfs, "PDC_Stage");
        Interface_TypedValue pdc_descr = new Interface_TypedValue("PDC_Description");
        SetValue(3, pdc_descr, "PDC_Descr");

        Interface_TypedValue pv_vid = new Interface_TypedValue("P_Version_Id");
        SetValue(4, pv_vid, "PV_Id");
        Interface_TypedValue pv_descr = new Interface_TypedValue("P_Version_Description");
        SetValue(5, pv_descr, "PV_Descr");

        Interface_TypedValue pid = new Interface_TypedValue("Product_Id");
        SetValue(6, pid, "P_Id");
        Interface_TypedValue pname = new Interface_TypedValue("Product_Name");
        SetValue(7, pname, "P_Name");
        Interface_TypedValue pdescr = new Interface_TypedValue("Product_Description");
        SetValue(8, pdescr, "P_Descr");

        Interface_TypedValue pc_name = new Interface_TypedValue("P_Context_Name");
        SetValue(9, pc_name, "PC_Name");
        Interface_TypedValue pc_disc = new Interface_TypedValue("P_Context_Discipline");
        SetValue(10, pc_disc, "PC_Disc");

        Interface_TypedValue ac_ap = new Interface_TypedValue("Application");
        SetValue(11, ac_ap, "Appli");
    }

    public override string Label()
    {
        return "STEP : Product Data (SDR)";
    }

    public override bool Recognize(IFSelect_EditForm form)
    {
        // il faut 10 parametres
        return true;
    }

    public override TCollection_HAsciiString StringValue(IFSelect_EditForm form, int num)
    {
        TCollection_HAsciiString str = null;
        switch (num)
        {
            case 1:
                return new TCollection_HAsciiString("as proposed");
            case 2:
                return new TCollection_HAsciiString("First_Design");
            case 3:
                return new TCollection_HAsciiString("void");
            case 4:
                return new TCollection_HAsciiString("ID");
            case 5:
                return new TCollection_HAsciiString("void");
            case 8:
                return new TCollection_HAsciiString("void");
            case 9:
                return new TCollection_HAsciiString("Mechanical");
            case 10:
                return new TCollection_HAsciiString("Assembly");
            case 11:
                return new TCollection_HAsciiString("EUCLID");
            default:
                break;
        }
        return str;
    }

    public override bool Load(IFSelect_EditForm form, Standard_Transient ent, Interface_InterfaceModel model)
    {
        StepShape_ShapeDefinitionRepresentation sdr = ent as StepShape_ShapeDefinitionRepresentation;
        StepData_StepModel modl = model as StepData_StepModel;
        if (sdr is null || modl is null)
        {
            return false;
        }

        STEPConstruct_Part ctx = new STEPConstruct_Part();
        ctx.ReadSDR(sdr);

        form.LoadValue(1, ctx.PDCname());
        form.LoadValue(2, ctx.PDCstage());
        form.LoadValue(3, ctx.PDdescription());

        form.LoadValue(4, ctx.PDFid());
        form.LoadValue(5, ctx.PDFdescription());

        form.LoadValue(6, ctx.Pid());
        form.LoadValue(7, ctx.Pname());
        form.LoadValue(8, ctx.Pdescription());

        form.LoadValue(9, ctx.PCname());
        form.LoadValue(10, ctx.PCdisciplineType());

        form.LoadValue(11, ctx.ACapplication());

        return true;
    }

    public override bool Apply(IFSelect_EditForm form, Standard_Transient ent, Interface_InterfaceModel model)
    {
        StepShape_ShapeDefinitionRepresentation sdr = ent as StepShape_ShapeDefinitionRepresentation;
        StepData_StepModel modl = model as StepData_StepModel;
        if (sdr is null || modl is null)
        {
            return false;
        }

        STEPConstruct_Part ctx = new STEPConstruct_Part();
        ctx.ReadSDR(sdr);

        if (form.IsModified(1))
        {
            ctx.SetPDCname(form.EditedValue(1));
        }
        if (form.IsModified(2))
        {
            ctx.SetPDCstage(form.EditedValue(2));
        }
        if (form.IsModified(3))
        {
            ctx.SetPDdescription(form.EditedValue(3));
        }

        if (form.IsModified(4))
        {
            ctx.SetPDFid(form.EditedValue(4));
        }
        if (form.IsModified(5))
        {
            ctx.SetPDFdescription(form.EditedValue(5));
        }

        if (form.IsModified(6))
        {
            ctx.SetPid(form.EditedValue(6));
        }
        if (form.IsModified(7))
        {
            ctx.SetPname(form.EditedValue(7));
        }
        if (form.IsModified(8))
        {
            ctx.SetPdescription(form.EditedValue(8));
        }

        if (form.IsModified(9))
        {
            ctx.SetPCname(form.EditedValue(9));
        }
        if (form.IsModified(10))
        {
            ctx.SetPCdisciplineType(form.EditedValue(10));
        }

        if (form.IsModified(11))
        {
            ctx.SetACapplication(form.EditedValue(11));
        }

        return true;
    }
}
