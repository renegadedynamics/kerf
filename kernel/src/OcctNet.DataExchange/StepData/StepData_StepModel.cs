// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepModel.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_StepModel.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.UnitsMethods_LengthUnit;

namespace Occt;

/// <summary>Gives access to: entities in a STEP Model, the header, and the labels of the entities (their idents in the file).</summary>
public class StepData_StepModel : Interface_InterfaceModel
{
    // File header: list of entities
    private readonly Interface_EntityList theheader = new();
    private NCollection_HArray1<int> theidnums;
    private double myWriteUnit = 1.0;
    private double myLocalLengthUnit = 1.0;
    private bool myReadUnitIsInitialized = false;
    private bool myWriteUnitIsInitialized = false;

    /// <summary>The parameters of the read or write (a value, copied on assignment).</summary>
    public DESTEP_Parameters InternalParameters = new();

    /// <summary>Creates an empty STEP model with an empty header.</summary>
    public StepData_StepModel()
    {
    }

    /// <summary>Returns entity given its rank. Same as InterfaceEntity, but with a shorter name.</summary>
    public Standard_Transient? Entity(int num)
    {
        return Value(num);
    } // More user-friendly name for accessing entities

    /// <summary>Gets header from another Model (uses Header Protocol).</summary>
    public override void GetFromAnother(Interface_InterfaceModel? other)
    {
        theheader.Clear();
        if (other is not StepData_StepModel another)
        {
            return;
        }
        Interface_EntityIterator iter = another.Header();
        // Copy the header. Important: header is distinct from content...
        Interface_CopyTool TC = new Interface_CopyTool(this, StepData.HeaderProtocol());
        for (; iter.More(); iter.Next())
        {
            Standard_Transient newhead = null;
            if (!TC.Copy(iter.Value(), ref newhead, false, false))
            {
                continue;
            }
            if (newhead is not null)
            {
                theheader.Append(newhead);
            }
        }
    }

    /// <summary>Returns a New Empty Model, same type as this one, i.e. a StepModel.</summary>
    public override Interface_InterfaceModel? NewEmptyModel()
    {
        return new StepData_StepModel();
    }

    /// <summary>Returns Header entities under the form of an iterator.</summary>
    public Interface_EntityIterator Header()
    {
        Interface_EntityIterator iter = new Interface_EntityIterator();
        theheader.FillIterator(iter);
        return iter;
    }

    /// <summary>Says if a Header entity has a specified type.</summary>
    public bool HasHeaderEntity(Type atype)
    {
        return (theheader.NbTypedEntities(atype) == 1);
    }

    /// <summary>Returns Header entity with specified type, if there is.</summary>
    public Standard_Transient? HeaderEntity(Type atype)
    {
        return theheader.TypedEntity(atype);
    }

    //   Header population methods

    /// <summary>Clears the Header.</summary>
    public override void ClearHeader()
    {
        theheader.Clear();
    }

    /// <summary>Adds an Entity to the Header.</summary>
    public void AddHeaderEntity(Standard_Transient? ent)
    {
        theheader.Append(ent);
    }

    /// <summary>Specific Check, checks Header Items with HeaderProtocol.</summary>
    public override void VerifyCheck(ref Interface_Check? ach)
    {
        Interface_GeneralLib lib = new Interface_GeneralLib(StepData.HeaderProtocol());
        StepData_StepModel me = this;
        Interface_Protocol aHP = StepData.HeaderProtocol();
        Interface_ShareTool sh = new Interface_ShareTool(me, aHP);
        for (Interface_EntityIterator iter = Header(); iter.More(); iter.Next())
        {
            Standard_Transient head = iter.Value();
            if (!lib.Select(head, out Interface_GeneralModule module, out int CN))
            {
                continue;
            }
            module.CheckCase(CN, head, sh, ref ach);
        }
    }

    /// <summary>Dumps the Header, with the Header Protocol of StepData. If the Header Protocol is not defined, for each Header Entity, prints its Type. Else sends the Header under the form of HEADER Section of an Ascii Step File. level is not used because Header is not so big.</summary>
    public override void DumpHeader(TextWriter S, int level = 0)
    {
        // Note: level parameter is not used in this implementation
        StepData_Protocol stepro = StepData.HeaderProtocol();
        bool iapro = stepro is not null;
        if (!iapro)
        {
            S.Write(" -- WARNING : StepModel DumpHeader, Protocol not defined\n");
        }

        Interface_EntityIterator iter = Header();
        int nb = iter.NbEntities();
        S.Write(" --  Step Model Header : " + iter.NbEntities() + " Entities :\n");
        for (iter.Start(); iter.More(); iter.Next())
        {
            S.Write("  " + iter.Value().DynamicType().Name + "\n");
        }
        if (!iapro || nb == 0)
        {
            return;
        }
        S.Write(" --  --        STEP MODEL    HEADER  CONTENT      --  --" + "\n");
        S.Write(" --   Dumped with Protocol : " + stepro.DynamicType().Name + "   --\n");

        StepData_StepModel me = this;
        StepData_StepWriter SW = new StepData_StepWriter(me);
        SW.SendModel(stepro, true); // Send HEADER only
        SW.Print(S);
    }

    /// <summary>Erases specific labels, i.e. clears the map (entity-ident).</summary>
    public override void ClearLabels()
    {
        theidnums = null;
    }

    // Set identifier label for an entity (used in STEP file format)
    // The identifier is typically a number like #123 that appears in STEP files

    /// <summary>Attaches an ident to an entity to produce a label (does nothing if this entity is not in the model).</summary>
    public void SetIdentLabel(Standard_Transient? ent, int ident)
    {
        int num = Number(ent);
        if (num == 0)
        {
            return; // Entity not found in model
        }
        int nbEnt = NbEntities();

        // Initialize identifier array if not yet created
        if (theidnums is null)
        {
            theidnums = new NCollection_HArray1<int>(1, nbEnt);
            theidnums.Init(0); // Initialize all values to 0
        }
        // Resize array if model has grown since last allocation
        else if (nbEnt > theidnums.Length())
        {
            int prevLength = theidnums.Length();
            NCollection_HArray1<int> idnums1 = new NCollection_HArray1<int>(1, nbEnt);
            idnums1.Init(0);
            // Copy existing identifier mappings
            int k = 1;
            for (; k <= prevLength; k++)
            {
                idnums1.SetValue(k, theidnums.Value(k));
            }
            theidnums = idnums1;
        }
        theidnums.SetValue(num, ident);
    }

    /// <summary>Returns the label ident attached to an entity, 0 if not in me.</summary>
    public int IdentLabel(Standard_Transient? ent)
    {
        if (theidnums is null)
        {
            return 0;
        }
        int num = Number(ent);
        return (num == 0 ? 0 : theidnums.Value(num));
    }

    /// <summary>Prints label specific to STEP norm for a given entity, i.e. if a LabelIdent has been recorded, its value with "#", else the number in the model with "#" and between ().</summary>
    public override void PrintLabel(Standard_Transient? ent, TextWriter S)
    {
        int num = (theidnums is null ? 0 : Number(ent));
        int nid = (num == 0 ? 0 : theidnums.Value(num));
        if (nid > 0)
        {
            S.Write("#" + nid);
        }
        else if (num > 0)
        {
            S.Write("(#" + num + ")");
        }
        else
        {
            S.Write("(#0..)");
        }
    }

    /// <summary>Returns a string with the label attached to a given entity, warning: this string is the address of an internal object (OCCT: a static buffer); a new string is returned here.</summary>
    public override TCollection_HAsciiString? StringLabel(Standard_Transient? ent)
    {
        TCollection_HAsciiString label;
        string text;
        int num = (theidnums is null ? 0 : Number(ent));
        int nid = (num == 0 ? 0 : theidnums.Value(num));

        if (nid > 0)
        {
            text = Standard.Sprintf("#%d", nid);
        }
        else if (num > 0)
        {
            text = Standard.Sprintf("(#%d)", num);
        }
        else
        {
            text = "(#0..)";
        }

        label = new TCollection_HAsciiString(text);
        return label;
    }

    /// <summary>Return the encoding of STEP file for converting names into UNICODE. Initialized from "read.step.codepage" variable by constructor, which is Resource_UTF8 by default.</summary>
    public Resource_FormatType SourceCodePage() => InternalParameters.ReadCodePage;

    /// <summary>Return the encoding of STEP file for converting names into UNICODE.</summary>
    public void SetSourceCodePage(Resource_FormatType theCode) => InternalParameters.ReadCodePage = theCode;

    /// <summary>Sets local length unit using for transfer process.</summary>
    public void SetLocalLengthUnit(double theUnit)
    {
        myLocalLengthUnit = theUnit;
        myReadUnitIsInitialized = true;
    }

    /// <summary>Returns local length unit using for transfer process (1 by default).</summary>
    public double LocalLengthUnit()
    {
        return myLocalLengthUnit;
    }

    /// <summary>Sets length unit using for writing process.</summary>
    public void SetWriteLengthUnit(double theUnit)
    {
        myWriteUnit = theUnit;
        myWriteUnitIsInitialized = true;
    }

    /// <summary>
    /// Returns length unit using for writing process (1 by default): the conversion factor from millimeters to the
    /// target unit, initialised on first call from <c>InternalParameters.WriteUnit</c>.
    /// </summary>
    public double WriteLengthUnit()
    {
        // Lazy initialization of write unit from global parameters
        if (!myWriteUnitIsInitialized)
        {
            myWriteUnitIsInitialized = true;
            switch (InternalParameters.WriteUnit)
            {
                case UnitsMethods_LengthUnit_Inch:
                    myWriteUnit = 25.4;
                    break;
                case UnitsMethods_LengthUnit_Millimeter:
                    myWriteUnit = 1.0;
                    break;
                case UnitsMethods_LengthUnit_Foot:
                    myWriteUnit = 304.8;
                    break;
                case UnitsMethods_LengthUnit_Mile:
                    myWriteUnit = 1609344.0;
                    break;
                case UnitsMethods_LengthUnit_Meter:
                    myWriteUnit = 1000.0;
                    break;
                case UnitsMethods_LengthUnit_Kilometer:
                    myWriteUnit = 1000000.0;
                    break;
                case UnitsMethods_LengthUnit_Mil:
                    myWriteUnit = 0.0254;
                    break;
                case UnitsMethods_LengthUnit_Micron:
                    myWriteUnit = 0.001;
                    break;
                case UnitsMethods_LengthUnit_Centimeter:
                    myWriteUnit = 10.0;
                    break;
                case UnitsMethods_LengthUnit_Microinch:
                    myWriteUnit = 0.0000254;
                    break;
                default:
                {
                    myWriteUnitIsInitialized = false;
                    GlobalCheck().AddWarning("Incorrect write.step.unit parameter, use default value");
                    break;
                }
            }
        }
        return myWriteUnit;
    }

    /// <summary>Returns the unit initialization flag: True - the unit was initialized, False - not initialized.</summary>
    public bool IsInitializedUnit() => myReadUnitIsInitialized;
}
