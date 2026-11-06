// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_ContextTool.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_ContextTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.FromGeometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Construct;

/// <summary>
/// Maintains global context tool for writing. Gives access to Product Definition Context (one per Model) Maintains
/// ApplicationProtocolDefinition entity (common for all products) Also maintains context specific for AP203 and
/// provides set of methods to work with various STEP constructs as required by Actor
/// </summary>
/// <remarks>
/// A C++ value class, a C# class here (decision 1185). Kerf: it also holds the writer's counter of
/// NEXT_ASSEMBLY_USAGE_OCCURRENCE identifiers (<see cref="NauoId"/>, decision 1266).
/// </remarks>
internal class ContextTool
{
    // :j4 16.03.99 gka S4134
    // abv 18.11.99 renamed from StepPDR_ContextTool

    public ContextTool()
    {
    }

    public ContextTool(StepModel aStepModel)
    {
        SetModel(aStepModel);
    }

    /// <summary>Initialize ApplicationProtocolDefinition by the first entity of that type found in the model</summary>
    public void SetModel(StepModel aStepModel)
    {
        _apd = null; // thePRPC.Nullify();
        _schema = aStepModel.InternalParameters.WriteSchema;
        _productName = aStepModel.InternalParameters.WriteProductName;
        int i, nb = aStepModel.EntityCount;
        for (i = 1; i <= nb && _apd is null; i++)
        {
            Transient ent = aStepModel.Value(i);
            if (ent.IsKind(typeof(ApplicationProtocolDefinition)))
            {
                if (_apd is null)
                {
                    _apd = ent as ApplicationProtocolDefinition;
                }
            }
        }
    }

    /// <summary>Sets global unit factors (used by the default axis).</summary>
    public void SetGlobalFactor(in Factors globalFactor)
    {
        _globalFactor = globalFactor;
    }

    public ApplicationProtocolDefinition GetAPD()
    {
        return _apd;
    }

    public void AddAPD(bool enforce = false)
    {
        bool noapd = _apd is null;
        if (noapd || enforce)
        {
            _apd = new ApplicationProtocolDefinition();
        }

        switch ((int)_schema)
        { // j4
            default:
            case 1:
                _apd.SetApplicationProtocolYear(1997);
                _apd.SetStatus(new TCollection_HAsciiString("committee draft"));
                _apd.SetApplicationInterpretedModelSchemaName(new TCollection_HAsciiString("automotive_design"));
                break;
            case 2:
                _apd.SetApplicationProtocolYear(1998);
                _apd.SetStatus(new TCollection_HAsciiString("draft international standard"));
                _apd.SetApplicationInterpretedModelSchemaName(new TCollection_HAsciiString("automotive_design"));
                break;
            case 3:
                _apd.SetApplicationProtocolYear(1994);
                _apd.SetStatus(new TCollection_HAsciiString("international standard"));
                _apd.SetApplicationInterpretedModelSchemaName(new TCollection_HAsciiString("config_control_design"));
                break;

            case 4:
                _apd.SetApplicationProtocolYear(2000);
                _apd.SetStatus(new TCollection_HAsciiString("international standard"));
                _apd.SetApplicationInterpretedModelSchemaName(new TCollection_HAsciiString("automotive_design"));
                break;
            case 5:
                _apd.SetApplicationProtocolYear(2013);
                _apd.SetStatus(new TCollection_HAsciiString("international standard"));
                _apd.SetApplicationInterpretedModelSchemaName(
                    new TCollection_HAsciiString("ap242_managed_model_based_3d_engineering"));
                break;
        }

        if (_apd.Application() is null)
        {
            _apd.SetApplication(new ApplicationContext());
        }
        TCollection_HAsciiString appl = null;
        switch ((int)_schema)
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
        _apd.Application().SetApplication(appl);
        //  if (noapd || enforce) aStepModel->AddWithRefs (theAPD);
    }

    public bool IsAP203
    {
        get
        {
            return SchemaIs("config_control_design");
        }
    }

    public bool IsAP214
    {
        get
        {
            return SchemaIs("automotive_design");
        }
    }

    public bool IsAP242
    {
        get
        {
            return SchemaIs("ap242_managed_model_based_3d_engineering");
        }
    }

    private bool SchemaIs(string name)
    {
        if (_apd is null)
        {
            return false;
        }
        TCollection_HAsciiString schema = _apd.ApplicationInterpretedModelSchemaName();
        if (schema is null)
        {
            return false;
        }
        string sch = schema.String().ToLowerInvariant();
        return sch == name;
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
            GetAPD().SetApplication(new ApplicationContext());
        }
        GetAPD().Application().SetApplication(name);
    }

    /// <summary>Returns a default axis placement</summary>
    public StepAxis2Placement3d GetDefaultAxis()
    {
        if (_axis is null)
        {
            Axis2Placement3dMaker mkax = new Axis2Placement3dMaker(_globalFactor);
            _axis = mkax.Value();
        }
        return _axis;
    }

    /// <summary>Returns tool which maintains context specific for AP203</summary>
    public AP203Context AP203Context()
    {
        return _ap203;
    }

    /// <summary>Returns current assembly level</summary>
    public int Level()
    {
        return _level.Length();
    }

    public void NextLevel()
    {
        _level.Append(1);
    }

    public void PrevLevel()
    {
        if (_level.Length() > 0)
        {
            _level.Remove(_level.Length());
        }
    }

    /// <summary>Changes current assembly level</summary>
    public void SetLevel(int lev)
    {
        if (lev < _level.Length())
        {
            while (lev < _level.Length() && _level.Length() > 0)
            {
                _level.Remove(_level.Length());
            }
        }
        else
        {
            while (_level.Length() < lev)
            {
                _level.Append(1);
            }
        }
    }

    /// <summary>Returns current index of assembly component on current level</summary>
    public int Index()
    {
        return (_level.Length() > 0 ? _level.Last() : 0);
    }

    public void NextIndex()
    {
        if (_level.Length() > 0)
        {
            _level.SetValue(_level.Length(), _level.Last() + 1);
        }
    }

    public void PrevIndex()
    {
        if (_level.Length() > 0)
        {
            _level.SetValue(_level.Length(), _level.Last() - 1);
        }
    }

    /// <summary>Changes current index of assembly component on current level</summary>
    public void SetIndex(int ind)
    {
        if (_level.Length() > 0)
        {
            _level.SetValue(_level.Length(), ind);
        }
    }

    /// <summary>
    /// Generates a product name basing on write.step.product.name parameter and current position in the assembly
    /// structure
    /// </summary>
    public TCollection_HAsciiString GetProductName()
    {
        TCollection_HAsciiString pdtName;
        if (!string.IsNullOrEmpty(_productName))
        {
            pdtName = new TCollection_HAsciiString(_productName);
        }
        else
        {
            pdtName = new TCollection_HAsciiString("Product");
        }

        for (int i = 1; i <= _level.Length(); i++)
        {
            pdtName.AssignCat(i > 1 ? "." : " ");
            pdtName.AssignCat(_level.Value(i).ToString(System.Globalization.CultureInfo.InvariantCulture));
        }

        return pdtName;
    }

    /// <summary>
    /// Produces and returns a full list of root entities required for part identified by SDRTool (including SDR
    /// itself)
    /// </summary>
    public NCollection_HSequence<Transient> GetRootsForPart(Part sdrTool)
    {
        NCollection_HSequence<Transient> seq = new NCollection_HSequence<Transient>();

        seq.Append(sdrTool.SDRValue());
        //  seq->Append ( GetAPD() );

        if (sdrTool.PRPC() is not null)
        {
            seq.Append(sdrTool.PRPC());
        }

        // for AP203, add required product management data
        if ((int)_schema == 3)
        {
            _ap203.Init(sdrTool);
            seq.Append(_ap203.GetProductCategoryRelationship());
            seq.Append(_ap203.GetCreator());
            seq.Append(_ap203.GetDesignOwner());
            seq.Append(_ap203.GetDesignSupplier());
            seq.Append(_ap203.GetClassificationOfficer());
            seq.Append(_ap203.GetSecurity());
            seq.Append(_ap203.GetCreationDate());
            seq.Append(_ap203.GetClassificationDate());
            seq.Append(_ap203.GetApproval());
            seq.Append(_ap203.GetApprover());
            seq.Append(_ap203.GetApprovalDateTime());
        }

        return seq;
    }

    /// <summary>
    /// Produces and returns a full list of root entities required for assembly link identified by assembly
    /// (including NAUO and CDSR)
    /// </summary>
    public NCollection_HSequence<Transient> GetRootsForAssemblyLink(ConstructAssembly assembly)
    {
        NCollection_HSequence<Transient> seq = new NCollection_HSequence<Transient>();

        seq.Append(assembly.ItemValue());

        // for AP203, write required product management data
        if ((int)_schema == 3)
        {
            _ap203.Init(assembly.GetNAUO());
            seq.Append(_ap203.GetSecurity());
            seq.Append(_ap203.GetClassificationOfficer());
            seq.Append(_ap203.GetClassificationDate());
            seq.Append(_ap203.GetApproval());
            seq.Append(_ap203.GetApprover());
            seq.Append(_ap203.GetApprovalDateTime());
        }

        return seq;
    }

    /// <summary>
    /// The last identifier given to a NEXT_ASSEMBLY_USAGE_OCCURRENCE by this writer (Kerf: OCCT numbers them
    /// from a process-wide counter, decision 1266).
    /// </summary>
    public ref int NauoId() => ref _nauoId;

    private NCollection_Sequence<int> _level = new NCollection_Sequence<int>();
    private ApplicationProtocolDefinition _apd;
    private AP203Context _ap203 = new AP203Context();
    private StepAxis2Placement3d _axis;
    private Factors _globalFactor = new Factors();
    private Parameters.WriteMode_StepSchema _schema;
    private string _productName = "";
    private int _nauoId;
}
