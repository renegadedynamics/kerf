// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepModel.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepModel.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Foundation;
using Kerf.Engine.Foundation.Resources;
using Occt;
using static Kerf.Engine.Exchange.Framework.FrameworkLengthUnit;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>Gives access to: entities in a STEP Model, the header, and the labels of the entities (their idents in the file).</summary>
internal class StepModel : InterfaceModel
{
    // File header: list of entities
    private readonly EntityList _theheader = new();
    private NCollection_HArray1<int> _theidnums;
    private double _writeUnit = 1.0;
    private double _localLengthUnit = 1.0;
    private bool _readUnitIsInitialized = false;
    private bool _writeUnitIsInitialized = false;

    /// <summary>The parameters of the read or write (a value, copied on assignment).</summary>
    public Parameters InternalParameters = new();

    /// <summary>Creates an empty STEP model with an empty header.</summary>
    public StepModel()
    {
    }

    /// <summary>Returns entity given its rank. Same as InterfaceEntity, but with a shorter name.</summary>
    public Transient? Entity(int num)
    {
        return Value(num);
    } // More user-friendly name for accessing entities

    /// <summary>Gets header from another Model (uses Header Protocol).</summary>
    public override void GetFromAnother(InterfaceModel? other)
    {
        _theheader.Clear();
        if (other is not StepModel another)
        {
            return;
        }
        EntityIterator iter = another.Header();
        // Copy the header. Important: header is distinct from content...
        CopyTool tc = new CopyTool(this, global::Kerf.Engine.Exchange.Step.Data.StepData.HeaderProtocol());
        for (; iter.More(); iter.Next())
        {
            Transient newhead = null;
            if (!tc.Copy(iter.Value(), ref newhead, false, false))
            {
                continue;
            }
            if (newhead is not null)
            {
                _theheader.Append(newhead);
            }
        }
    }

    /// <summary>Returns a New Empty Model, same type as this one, i.e. a StepModel.</summary>
    public override InterfaceModel? NewEmptyModel()
    {
        return new StepModel();
    }

    /// <summary>Returns Header entities under the form of an iterator.</summary>
    public EntityIterator Header()
    {
        EntityIterator iter = new EntityIterator();
        _theheader.FillIterator(iter);
        return iter;
    }

    /// <summary>Says if a Header entity has a specified type.</summary>
    public bool HasHeaderEntity(Type atype)
    {
        return (_theheader.NbTypedEntities(atype) == 1);
    }

    /// <summary>Returns Header entity with specified type, if there is.</summary>
    public Transient? HeaderEntity(Type atype)
    {
        return _theheader.TypedEntity(atype);
    }

    //   Header population methods

    /// <summary>Clears the Header.</summary>
    public override void ClearHeader()
    {
        _theheader.Clear();
    }

    /// <summary>Adds an Entity to the Header.</summary>
    public void AddHeaderEntity(Transient? ent)
    {
        _theheader.Append(ent);
    }

    /// <summary>Specific Check, checks Header Items with HeaderProtocol.</summary>
    public override void VerifyCheck(ref FrameworkCheck? ach)
    {
        GeneralLib lib = new GeneralLib(global::Kerf.Engine.Exchange.Step.Data.StepData.HeaderProtocol());
        StepModel me = this;
        Protocol hp = global::Kerf.Engine.Exchange.Step.Data.StepData.HeaderProtocol();
        ShareTool sh = new ShareTool(me, hp);
        for (EntityIterator iter = Header(); iter.More(); iter.Next())
        {
            Transient head = iter.Value();
            if (!lib.Select(head, out GeneralModule module, out int cn))
            {
                continue;
            }
            module.CheckCase(cn, head, sh, ref ach);
        }
    }

    /// <summary>Dumps the Header, with the Header Protocol of StepData. If the Header Protocol is not defined, for each Header Entity, prints its Type. Else sends the Header under the form of HEADER Section of an Ascii Step File. level is not used because Header is not so big.</summary>
    public override void DumpHeader(TextWriter s, int level = 0)
    {
        // Note: level parameter is not used in this implementation
        StepProtocol stepro = global::Kerf.Engine.Exchange.Step.Data.StepData.HeaderProtocol();
        bool iapro = stepro is not null;
        if (!iapro)
        {
            s.Write(" -- WARNING : StepModel DumpHeader, Protocol not defined\n");
        }

        EntityIterator iter = Header();
        int nb = iter.NbEntities();
        s.Write(" --  Step Model Header : " + iter.NbEntities() + " Entities :\n");
        for (iter.Start(); iter.More(); iter.Next())
        {
            s.Write("  " + iter.Value().DynamicTypeName() + "\n");
        }
        if (!iapro || nb == 0)
        {
            return;
        }
        s.Write(" --  --        STEP MODEL    HEADER  CONTENT      --  --" + "\n");
        s.Write(" --   Dumped with Protocol : " + stepro.DynamicTypeName() + "   --\n");

        StepModel me = this;
        StepWriter sw = new StepWriter(me);
        sw.SendModel(stepro, true); // Send HEADER only
        sw.Print(s);
    }

    /// <summary>Erases specific labels, i.e. clears the map (entity-ident).</summary>
    public override void ClearLabels()
    {
        _theidnums = null;
    }

    // Set identifier label for an entity (used in STEP file format)
    // The identifier is typically a number like #123 that appears in STEP files

    /// <summary>Attaches an ident to an entity to produce a label (does nothing if this entity is not in the model).</summary>
    public void SetIdentLabel(Transient? ent, int ident)
    {
        int num = Number(ent);
        if (num == 0)
        {
            return; // Entity not found in model
        }
        int nbEnt = EntityCount;

        // Initialize identifier array if not yet created
        if (_theidnums is null)
        {
            _theidnums = new NCollection_HArray1<int>(1, nbEnt);
            _theidnums.Init(0); // Initialize all values to 0
        }
        // Resize array if model has grown since last allocation
        else if (nbEnt > _theidnums.Length())
        {
            int prevLength = _theidnums.Length();
            NCollection_HArray1<int> idnums1 = new NCollection_HArray1<int>(1, nbEnt);
            idnums1.Init(0);
            // Copy existing identifier mappings
            int k = 1;
            for (; k <= prevLength; k++)
            {
                idnums1.SetValue(k, _theidnums.Value(k));
            }
            _theidnums = idnums1;
        }
        _theidnums.SetValue(num, ident);
    }

    /// <summary>Returns the label ident attached to an entity, 0 if not in me.</summary>
    public int IdentLabel(Transient? ent)
    {
        if (_theidnums is null)
        {
            return 0;
        }
        int num = Number(ent);
        return (num == 0 ? 0 : _theidnums.Value(num));
    }

    /// <summary>Prints label specific to STEP norm for a given entity, i.e. if a LabelIdent has been recorded, its value with "#", else the number in the model with "#" and between ().</summary>
    public override void PrintLabel(Transient? ent, TextWriter s)
    {
        int num = (_theidnums is null ? 0 : Number(ent));
        int nid = (num == 0 ? 0 : _theidnums.Value(num));
        if (nid > 0)
        {
            s.Write("#" + nid);
        }
        else if (num > 0)
        {
            s.Write("(#" + num + ")");
        }
        else
        {
            s.Write("(#0..)");
        }
    }

    /// <summary>Returns a string with the label attached to a given entity, warning: this string is the address of an internal object (OCCT: a static buffer); a new string is returned here.</summary>
    public override TCollection_HAsciiString? StringLabel(Transient? ent)
    {
        TCollection_HAsciiString label;
        string text;
        int num = (_theidnums is null ? 0 : Number(ent));
        int nid = (num == 0 ? 0 : _theidnums.Value(num));

        if (nid > 0)
        {
            text = global::Kerf.Engine.Foundation.Standard.Sprintf("#%d", nid);
        }
        else if (num > 0)
        {
            text = global::Kerf.Engine.Foundation.Standard.Sprintf("(#%d)", num);
        }
        else
        {
            text = "(#0..)";
        }

        label = new TCollection_HAsciiString(text);
        return label;
    }

    /// <summary>Return the encoding of STEP file for converting names into UNICODE. Initialized from "read.step.codepage" variable by constructor, which is Resource_UTF8 by default.</summary>
    public FormatType SourceCodePage() => InternalParameters.ReadCodePage;

    /// <summary>Return the encoding of STEP file for converting names into UNICODE.</summary>
    public void SetSourceCodePage(FormatType code) => InternalParameters.ReadCodePage = code;

    /// <summary>Sets local length unit using for transfer process.</summary>
    public void SetLocalLengthUnit(double unit)
    {
        _localLengthUnit = unit;
        _readUnitIsInitialized = true;
    }

    /// <summary>Returns local length unit using for transfer process (1 by default).</summary>
    public double LocalLengthUnit()
    {
        return _localLengthUnit;
    }

    /// <summary>Sets length unit using for writing process.</summary>
    public void SetWriteLengthUnit(double unit)
    {
        _writeUnit = unit;
        _writeUnitIsInitialized = true;
    }

    /// <summary>
    /// Returns length unit using for writing process (1 by default): the conversion factor from millimeters to the
    /// target unit, initialised on first call from <c>InternalParameters.WriteUnit</c>.
    /// </summary>
    public double WriteLengthUnit()
    {
        // Lazy initialization of write unit from global parameters
        if (!_writeUnitIsInitialized)
        {
            _writeUnitIsInitialized = true;
            switch (InternalParameters.WriteUnit)
            {
                case FrameworkLengthUnit.LengthUnit_Inch:
                    _writeUnit = 25.4;
                    break;
                case FrameworkLengthUnit.LengthUnit_Millimeter:
                    _writeUnit = 1.0;
                    break;
                case FrameworkLengthUnit.LengthUnit_Foot:
                    _writeUnit = 304.8;
                    break;
                case FrameworkLengthUnit.LengthUnit_Mile:
                    _writeUnit = 1609344.0;
                    break;
                case FrameworkLengthUnit.LengthUnit_Meter:
                    _writeUnit = 1000.0;
                    break;
                case FrameworkLengthUnit.LengthUnit_Kilometer:
                    _writeUnit = 1000000.0;
                    break;
                case FrameworkLengthUnit.LengthUnit_Mil:
                    _writeUnit = 0.0254;
                    break;
                case FrameworkLengthUnit.LengthUnit_Micron:
                    _writeUnit = 0.001;
                    break;
                case FrameworkLengthUnit.LengthUnit_Centimeter:
                    _writeUnit = 10.0;
                    break;
                case FrameworkLengthUnit.LengthUnit_Microinch:
                    _writeUnit = 0.0000254;
                    break;
                default:
                {
                    _writeUnitIsInitialized = false;
                    GlobalCheck().AddWarning("Incorrect write.step.unit parameter, use default value");
                    break;
                }
            }
        }
        return _writeUnit;
    }

    /// <summary>Returns the unit initialization flag: True - the unit was initialized, False - not initialized.</summary>
    public bool IsInitializedUnit => _readUnitIsInitialized;
}
