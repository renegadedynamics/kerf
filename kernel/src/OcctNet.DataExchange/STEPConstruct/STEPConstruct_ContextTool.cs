// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_ContextTool.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_ContextTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// Maintains global context tool for writing. Gives access to Product Definition Context (one per Model) Maintains
/// ApplicationProtocolDefinition entity (common for all products) Also maintains context specific for AP203 and
/// provides set of methods to work with various STEP constructs as required by Actor
/// </summary>
/// <remarks>
/// A C++ value class, a C# class here (decision 1185). OCCT.NET: it also holds the writer's counter of
/// NEXT_ASSEMBLY_USAGE_OCCURRENCE identifiers (<see cref="NauoId"/>, decision 1266).
/// </remarks>
public class STEPConstruct_ContextTool
{
    // :j4 16.03.99 gka S4134
    // abv 18.11.99 renamed from StepPDR_ContextTool

    public STEPConstruct_ContextTool()
    {
    }

    public STEPConstruct_ContextTool(StepData_StepModel aStepModel)
    {
        SetModel(aStepModel);
    }

    /// <summary>Initialize ApplicationProtocolDefinition by the first entity of that type found in the model</summary>
    public void SetModel(StepData_StepModel aStepModel)
    {
        theAPD = null; // thePRPC.Nullify();
        mySchema = aStepModel.InternalParameters.WriteSchema;
        myProductName = aStepModel.InternalParameters.WriteProductName;
        int i, nb = aStepModel.NbEntities();
        for (i = 1; i <= nb && theAPD is null; i++)
        {
            Standard_Transient ent = aStepModel.Value(i);
            if (ent.IsKind(typeof(StepBasic_ApplicationProtocolDefinition)))
            {
                if (theAPD is null)
                {
                    theAPD = ent as StepBasic_ApplicationProtocolDefinition;
                }
            }
        }
    }

    /// <summary>Sets global unit factors (used by the default axis).</summary>
    public void SetGlobalFactor(in StepData_Factors theGlobalFactor)
    {
        myGlobalFactor = theGlobalFactor;
    }

    public StepBasic_ApplicationProtocolDefinition GetAPD()
    {
        return theAPD;
    }

    public void AddAPD(bool enforce = false)
    {
        bool noapd = theAPD is null;
        if (noapd || enforce)
        {
            theAPD = new StepBasic_ApplicationProtocolDefinition();
        }

        switch ((int)mySchema)
        { // j4
            default:
            case 1:
                theAPD.SetApplicationProtocolYear(1997);
                theAPD.SetStatus(new TCollection_HAsciiString("committee draft"));
                theAPD.SetApplicationInterpretedModelSchemaName(new TCollection_HAsciiString("automotive_design"));
                break;
            case 2:
                theAPD.SetApplicationProtocolYear(1998);
                theAPD.SetStatus(new TCollection_HAsciiString("draft international standard"));
                theAPD.SetApplicationInterpretedModelSchemaName(new TCollection_HAsciiString("automotive_design"));
                break;
            case 3:
                theAPD.SetApplicationProtocolYear(1994);
                theAPD.SetStatus(new TCollection_HAsciiString("international standard"));
                theAPD.SetApplicationInterpretedModelSchemaName(new TCollection_HAsciiString("config_control_design"));
                break;

            case 4:
                theAPD.SetApplicationProtocolYear(2000);
                theAPD.SetStatus(new TCollection_HAsciiString("international standard"));
                theAPD.SetApplicationInterpretedModelSchemaName(new TCollection_HAsciiString("automotive_design"));
                break;
            case 5:
                theAPD.SetApplicationProtocolYear(2013);
                theAPD.SetStatus(new TCollection_HAsciiString("international standard"));
                theAPD.SetApplicationInterpretedModelSchemaName(
                    new TCollection_HAsciiString("ap242_managed_model_based_3d_engineering"));
                break;
        }

        if (theAPD.Application() is null)
        {
            theAPD.SetApplication(new StepBasic_ApplicationContext());
        }
        TCollection_HAsciiString appl = null;
        switch ((int)mySchema)
        { // j4
            default:
            case 1:
            case 2:
                appl = new TCollection_HAsciiString("core data for automotive mechanical design processes");
                break;
            case 3:
                appl = new TCollection_HAsciiString("configuration controlled 3D designs of mechanical parts and assemblies");
                break;
            case 5:
                appl = new TCollection_HAsciiString("Managed model based 3d engineering");
                break;
        }
        theAPD.Application().SetApplication(appl);
        //  if (noapd || enforce) aStepModel->AddWithRefs (theAPD);
    }

    public bool IsAP203()
    {
        return SchemaIs("config_control_design");
    }

    public bool IsAP214()
    {
        return SchemaIs("automotive_design");
    }

    public bool IsAP242()
    {
        return SchemaIs("ap242_managed_model_based_3d_engineering");
    }

    private bool SchemaIs(string theName)
    {
        if (theAPD is null)
        {
            return false;
        }
        TCollection_HAsciiString schema = theAPD.ApplicationInterpretedModelSchemaName();
        if (schema is null)
        {
            return false;
        }
        string sch = schema.String().ToLowerInvariant();
        return sch == theName;
    }

    public TCollection_HAsciiString GetACstatus()
    {
        if (GetAPD() is null)
        {
            return new TCollection_HAsciiString("");
        }
        return GetAPD().Status();
    }

    public TCollection_HAsciiString GetACschemaName()
    {
        if (GetAPD() is null)
        {
            return new TCollection_HAsciiString("");
        }
        return GetAPD().ApplicationInterpretedModelSchemaName();
    }

    public int GetACyear()
    {
        return (GetAPD() is null ? 1998 : GetAPD().ApplicationProtocolYear());
    }

    public TCollection_HAsciiString GetACname()
    {
        if (GetAPD() is null)
        {
            return new TCollection_HAsciiString("");
        }
        if (GetAPD().Application() is null)
        {
            return new TCollection_HAsciiString("");
        }
        return GetAPD().Application().Application();
    }

    public void SetACstatus(TCollection_HAsciiString status)
    {
        if (GetAPD() is null)
        {
            return;
        }
        GetAPD().SetStatus(status);
    }

    public void SetACschemaName(TCollection_HAsciiString schemaName)
    {
        if (GetAPD() is null)
        {
            return;
        }
        GetAPD().SetApplicationInterpretedModelSchemaName(schemaName);
    }

    public void SetACyear(int year)
    {
        if (GetAPD() is null)
        {
            return;
        }
        GetAPD().SetApplicationProtocolYear(year);
    }

    public void SetACname(TCollection_HAsciiString name)
    {
        if (GetAPD() is null)
        {
            return;
        }
        if (GetAPD().Application() is null)
        {
            GetAPD().SetApplication(new StepBasic_ApplicationContext());
        }
        GetAPD().Application().SetApplication(name);
    }

    /// <summary>Returns a default axis placement</summary>
    public StepGeom_Axis2Placement3d GetDefaultAxis()
    {
        if (myAxis is null)
        {
            GeomToStep_MakeAxis2Placement3d mkax = new GeomToStep_MakeAxis2Placement3d(myGlobalFactor);
            myAxis = mkax.Value();
        }
        return myAxis;
    }

    /// <summary>Returns tool which maintains context specific for AP203</summary>
    public STEPConstruct_AP203Context AP203Context()
    {
        return theAP203;
    }

    /// <summary>Returns current assembly level</summary>
    public int Level()
    {
        return myLevel.Length();
    }

    public void NextLevel()
    {
        myLevel.Append(1);
    }

    public void PrevLevel()
    {
        if (myLevel.Length() > 0)
        {
            myLevel.Remove(myLevel.Length());
        }
    }

    /// <summary>Changes current assembly level</summary>
    public void SetLevel(int lev)
    {
        if (lev < myLevel.Length())
        {
            while (lev < myLevel.Length() && myLevel.Length() > 0)
            {
                myLevel.Remove(myLevel.Length());
            }
        }
        else
        {
            while (myLevel.Length() < lev)
            {
                myLevel.Append(1);
            }
        }
    }

    /// <summary>Returns current index of assembly component on current level</summary>
    public int Index()
    {
        return (myLevel.Length() > 0 ? myLevel.Last() : 0);
    }

    public void NextIndex()
    {
        if (myLevel.Length() > 0)
        {
            myLevel.SetValue(myLevel.Length(), myLevel.Last() + 1);
        }
    }

    public void PrevIndex()
    {
        if (myLevel.Length() > 0)
        {
            myLevel.SetValue(myLevel.Length(), myLevel.Last() - 1);
        }
    }

    /// <summary>Changes current index of assembly component on current level</summary>
    public void SetIndex(int ind)
    {
        if (myLevel.Length() > 0)
        {
            myLevel.SetValue(myLevel.Length(), ind);
        }
    }

    /// <summary>
    /// Generates a product name basing on write.step.product.name parameter and current position in the assembly
    /// structure
    /// </summary>
    public TCollection_HAsciiString GetProductName()
    {
        TCollection_HAsciiString PdtName;
        if (!string.IsNullOrEmpty(myProductName))
        {
            PdtName = new TCollection_HAsciiString(myProductName);
        }
        else
        {
            PdtName = new TCollection_HAsciiString("Product");
        }

        for (int i = 1; i <= myLevel.Length(); i++)
        {
            PdtName.AssignCat(i > 1 ? "." : " ");
            PdtName.AssignCat(myLevel.Value(i).ToString(System.Globalization.CultureInfo.InvariantCulture));
        }

        return PdtName;
    }

    /// <summary>
    /// Produces and returns a full list of root entities required for part identified by SDRTool (including SDR
    /// itself)
    /// </summary>
    public NCollection_HSequence<Standard_Transient> GetRootsForPart(STEPConstruct_Part SDRTool)
    {
        NCollection_HSequence<Standard_Transient> seq = new NCollection_HSequence<Standard_Transient>();

        seq.Append(SDRTool.SDRValue());
        //  seq->Append ( GetAPD() );

        if (SDRTool.PRPC() is not null)
        {
            seq.Append(SDRTool.PRPC());
        }

        // for AP203, add required product management data
        if ((int)mySchema == 3)
        {
            theAP203.Init(SDRTool);
            seq.Append(theAP203.GetProductCategoryRelationship());
            seq.Append(theAP203.GetCreator());
            seq.Append(theAP203.GetDesignOwner());
            seq.Append(theAP203.GetDesignSupplier());
            seq.Append(theAP203.GetClassificationOfficer());
            seq.Append(theAP203.GetSecurity());
            seq.Append(theAP203.GetCreationDate());
            seq.Append(theAP203.GetClassificationDate());
            seq.Append(theAP203.GetApproval());
            seq.Append(theAP203.GetApprover());
            seq.Append(theAP203.GetApprovalDateTime());
        }

        return seq;
    }

    /// <summary>
    /// Produces and returns a full list of root entities required for assembly link identified by assembly
    /// (including NAUO and CDSR)
    /// </summary>
    public NCollection_HSequence<Standard_Transient> GetRootsForAssemblyLink(STEPConstruct_Assembly assembly)
    {
        NCollection_HSequence<Standard_Transient> seq = new NCollection_HSequence<Standard_Transient>();

        seq.Append(assembly.ItemValue());

        // for AP203, write required product management data
        if ((int)mySchema == 3)
        {
            theAP203.Init(assembly.GetNAUO());
            seq.Append(theAP203.GetSecurity());
            seq.Append(theAP203.GetClassificationOfficer());
            seq.Append(theAP203.GetClassificationDate());
            seq.Append(theAP203.GetApproval());
            seq.Append(theAP203.GetApprover());
            seq.Append(theAP203.GetApprovalDateTime());
        }

        return seq;
    }

    /// <summary>
    /// The last identifier given to a NEXT_ASSEMBLY_USAGE_OCCURRENCE by this writer (OCCT.NET: OCCT numbers them
    /// from a process-wide counter, decision 1266).
    /// </summary>
    public ref int NauoId() => ref myNauoId;

    private NCollection_Sequence<int> myLevel = new NCollection_Sequence<int>();
    private StepBasic_ApplicationProtocolDefinition theAPD;
    private STEPConstruct_AP203Context theAP203 = new STEPConstruct_AP203Context();
    private StepGeom_Axis2Placement3d myAxis;
    private StepData_Factors myGlobalFactor = new StepData_Factors();
    private DESTEP_Parameters.WriteMode_StepSchema mySchema;
    private string myProductName = "";
    private int myNauoId;
}
