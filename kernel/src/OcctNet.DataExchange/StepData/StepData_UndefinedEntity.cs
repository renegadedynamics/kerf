// Port of: src/DataExchange/TKDESTEP/StepData/StepData_UndefinedEntity.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_UndefinedEntity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using static Occt.Interface_ParamType;

namespace Occt;

/// <summary>
/// Undefined entity specific to Step Interface, in which StepType is defined at each instance, or is a SubList of
/// another one. Uses an UndefinedContent, that from Interface is suitable. Also an Entity defined by STEP can be
/// "Complex Type" (see ANDOR clause in Express).
/// </summary>
/// <remarks>Its literal parameters keep the file's text as it is (one char per byte, decision 1174), quotes included.</remarks>
public class StepData_UndefinedEntity : Transient
{
    private TCollection_HAsciiString thetype;
    private Interface_UndefinedContent thecont;
    private bool thesub;
    private StepData_UndefinedEntity thenext;

    /// <summary>Creates an Unknown entity.</summary>
    public StepData_UndefinedEntity()
    {
        thecont = new Interface_UndefinedContent();
        thesub = false;
    }

    /// <summary>Creates an Unknown entity, which may be a Sub List or not.</summary>
    public StepData_UndefinedEntity(bool issub)
    {
        thesub = issub;
        thecont = new Interface_UndefinedContent();
    }

    /// <summary>Returns the UndefinedContent which brings the Parameters.</summary>
    public Interface_UndefinedContent UndefinedContent()
    {
        return thecont;
    }

    /// <summary>Returns True if an Unknown Entity is (in the file) a "Sub List" of another one.</summary>
    public bool IsSub()
    {
        return thesub;
    }

    // OCCT declares Super() without defining it: left out

    /// <summary>Returns True if this entity is Complex (i.e. has a Next).</summary>
    public bool IsComplex()
    {
        return (thenext is not null);
    }

    /// <summary>Returns the next component of a Complex Type Entity (Null if not Complex, or if it is the last one).</summary>
    public StepData_UndefinedEntity? Next()
    {
        return thenext;
    }

    /// <summary>Returns Record Type defined by the File (also good for a SubList).</summary>
    public string StepType()
    {
        if (thetype is null)
        {
            return "";
        }
        return thetype.ToCString();
    }

    /// <summary>Reads data from StepReaderData (i.e. from a File), by filling StepType and parameters stored in the UndefinedContent.</summary>
    public void ReadRecord(StepData_StepReaderData SR, int num, Interface_Check ach)
    {
        thetype = new TCollection_HAsciiString(SR.RecordType(num));
        int nb = SR.NbParams(num);

        thecont.Reservate(nb, 4);
        for (int i = 1; i <= nb; i++)
        {
            Transient anent = null;
            TCollection_HAsciiString hval = null;
            string val = SR.ParamCValue(num, i);
            Interface_ParamType partyp = SR.ParamType(num, i);
            int nume = 0;
            if (partyp == Interface_ParamIdent)
            {
                nume = SR.ParamNumber(num, i);
                if (nume > 0)
                {
                    anent = SR.BoundEntity(nume);
                    if (anent is null)
                    {
                        nume = 0;
                    }
                }
                if (nume <= 0)
                {
                    ach.AddFail("A reference to another entity is unresolved");
                    partyp = Interface_ParamVoid;
                }
            }
            else if (partyp == Interface_ParamSub)
            {
                nume = SR.ParamNumber(num, i);
                StepData_UndefinedEntity und = new StepData_UndefinedEntity(true);
                anent = und;
                und.ReadRecord(SR, nume, ach);
            }
            else if (partyp == Interface_ParamText)
            {
                //    Return integre a supprimer silya
                int lval = val.Length;
                int mval = -1;
                for (int j = 0; j < lval; j++)
                {
                    if (val[j] == '\n')
                    {
                        mval = i;
                        break;
                    }
                }
                if (mval > 0)
                {
                    nume = -1;
                    hval = new TCollection_HAsciiString(val);
                    hval.RemoveAll('\n');
                }
            }
            if (nume == 0)
            {
                hval = new TCollection_HAsciiString(val);
            }
            if (nume > 0)
            {
                thecont.AddEntity(partyp, anent);
            }
            else
            {
                thecont.AddLiteral(partyp, hval);
            }
        }
        int nextyp = SR.NextForComplex(num);
        if (nextyp == 0)
        {
            return;
        }
        thenext = new StepData_UndefinedEntity();
        thenext.ReadRecord(SR, nextyp, ach);
    }

    /// <summary>Write parameters to StepWriter (i.e. to a File), as part of the sending of the entity.</summary>
    public void WriteParams(StepData_StepWriter SW)
    {
        if (!IsSub())
        {
            SW.StartEntity(StepType());
        }
        int nb = thecont.NbParams();
        Transient anent;
        for (int i = 1; i <= nb; i++)
        {
            Interface_ParamType partyp = thecont.ParamType(i);
            if (partyp == Interface_ParamSub)
            {
                StepData_UndefinedEntity und = thecont.ParamEntity(i) as StepData_UndefinedEntity;
                und.StepType(); // svv #2
                if (und.IsSub())
                {
                    SW.OpenTypedSub(und.StepType());
                }
                und.WriteParams(SW);
                if (und.IsSub())
                {
                    SW.CloseSub();
                }
            }
            else if (partyp == Interface_ParamIdent)
            {
                anent = thecont.ParamEntity(i);
                SW.Send(anent);
            }
            else
            {
                SW.SendString(thecont.ParamValue(i).ToCString());
            }
        }
        //  if (IsSub()) return;
        //  SW.NewLine(true);
        if (thenext is null)
        {
            return;
        }
        thenext.WriteParams(SW);
    }

    /// <summary>Gets internal data from another UndefinedEntity.</summary>
    public void GetFromAnother(StepData_UndefinedEntity another, Interface_CopyTool TC)
    {
        thetype = new TCollection_HAsciiString(another.StepType());
        thecont = new Interface_UndefinedContent();
        thecont.GetFromAnother(another.UndefinedContent(), TC);
        thesub = another.IsSub();
        if (another.IsComplex())
        {
            thenext = TC.Transferred(another.Next()) as StepData_UndefinedEntity;
        }
        else
        {
            thenext = null;
        }
    }

    /// <summary>Fills the list of shared entities.</summary>
    public void FillShared(Interface_EntityIterator list)
    {
        int i, nb = thecont.NbParams();
        for (i = 1; i <= nb; i++)
        {
            Interface_ParamType ptype = thecont.ParamType(i);
            if (ptype == Interface_ParamSub)
            {
                StepData_UndefinedEntity subent = thecont.ParamEntity(i) as StepData_UndefinedEntity;
                subent.FillShared(list);
            }
            else if (ptype == Interface_ParamIdent)
            {
                list.AddItem(thecont.ParamEntity(i));
            }
        }
        if (thenext is not null)
        {
            thenext.FillShared(list);
        }
    }
}
