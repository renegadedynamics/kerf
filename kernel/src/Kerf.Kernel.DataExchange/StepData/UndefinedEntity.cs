// Port of: src/DataExchange/TKDESTEP/StepData/StepData_UndefinedEntity.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_UndefinedEntity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Exchange.Framework.ParamType;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// Undefined entity specific to Step Interface, in which StepType is defined at each instance, or is a SubList of
/// another one. Uses an UndefinedContent, that from Interface is suitable. Also an Entity defined by STEP can be
/// "Complex Type" (see ANDOR clause in Express).
/// </summary>
/// <remarks>Its literal parameters keep the file's text as it is (one char per byte, decision 1174), quotes included.</remarks>
internal class UndefinedEntity : Transient
{
    private TCollection_HAsciiString _thetype;
    private UndefinedContent _thecont;
    private bool _thesub;
    private UndefinedEntity _thenext;

    /// <summary>Creates an Unknown entity.</summary>
    public UndefinedEntity()
    {
        _thecont = new UndefinedContent();
        _thesub = false;
    }

    /// <summary>Creates an Unknown entity, which may be a Sub List or not.</summary>
    public UndefinedEntity(bool issub)
    {
        _thesub = issub;
        _thecont = new UndefinedContent();
    }

    /// <summary>Returns the UndefinedContent which brings the Parameters.</summary>
    public UndefinedContent UndefinedContent()
    {
        return _thecont;
    }

    /// <summary>Returns True if an Unknown Entity is (in the file) a "Sub List" of another one.</summary>
    public bool IsSub
    {
        get
        {
            return _thesub;
        }
    }

    // OCCT declares Super() without defining it: left out

    /// <summary>Returns True if this entity is Complex (i.e. has a Next).</summary>
    public bool IsComplex
    {
        get
        {
            return (_thenext is not null);
        }
    }

    /// <summary>Returns the next component of a Complex Type Entity (Null if not Complex, or if it is the last one).</summary>
    public UndefinedEntity? Next()
    {
        return _thenext;
    }

    /// <summary>Returns Record Type defined by the File (also good for a SubList).</summary>
    public string StepType()
    {
        if (_thetype is null)
        {
            return "";
        }
        return _thetype.ToCString();
    }

    /// <summary>Reads data from StepReaderData (i.e. from a File), by filling StepType and parameters stored in the UndefinedContent.</summary>
    public void ReadRecord(StepReaderData sr, int num, FrameworkCheck ach)
    {
        _thetype = new TCollection_HAsciiString(sr.RecordType(num));
        int nb = sr.NbParams(num);

        _thecont.Reservate(nb, 4);
        for (int i = 1; i <= nb; i++)
        {
            Transient anent = null;
            TCollection_HAsciiString hval = null;
            string val = sr.ParamCValue(num, i);
            ParamType partyp = sr.ParamType(num, i);
            int nume = 0;
            if (partyp == ParamType.ParamIdent)
            {
                nume = sr.ParamNumber(num, i);
                if (nume > 0)
                {
                    anent = sr.BoundEntity(nume);
                    if (anent is null)
                    {
                        nume = 0;
                    }
                }
                if (nume <= 0)
                {
                    ach.AddFail("A reference to another entity is unresolved");
                    partyp = ParamType.ParamVoid;
                }
            }
            else if (partyp == ParamType.ParamSub)
            {
                nume = sr.ParamNumber(num, i);
                UndefinedEntity und = new UndefinedEntity(true);
                anent = und;
                und.ReadRecord(sr, nume, ach);
            }
            else if (partyp == ParamType.ParamText)
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
                _thecont.AddEntity(partyp, anent);
            }
            else
            {
                _thecont.AddLiteral(partyp, hval);
            }
        }
        int nextyp = sr.NextForComplex(num);
        if (nextyp == 0)
        {
            return;
        }
        _thenext = new UndefinedEntity();
        _thenext.ReadRecord(sr, nextyp, ach);
    }

    /// <summary>Write parameters to StepWriter (i.e. to a File), as part of the sending of the entity.</summary>
    public void WriteParams(StepWriter sw)
    {
        if (!IsSub)
        {
            sw.StartEntity(StepType());
        }
        int nb = _thecont.ParamCount;
        Transient anent;
        for (int i = 1; i <= nb; i++)
        {
            ParamType partyp = _thecont.ParamType(i);
            if (partyp == ParamType.ParamSub)
            {
                UndefinedEntity und = _thecont.ParamEntity(i) as UndefinedEntity;
                und.StepType(); // svv #2
                if (und.IsSub)
                {
                    sw.OpenTypedSub(und.StepType());
                }
                und.WriteParams(sw);
                if (und.IsSub)
                {
                    sw.CloseSub();
                }
            }
            else if (partyp == ParamType.ParamIdent)
            {
                anent = _thecont.ParamEntity(i);
                sw.Send(anent);
            }
            else
            {
                sw.SendString(_thecont.ParamValue(i).ToCString());
            }
        }
        //  if (IsSub()) return;
        //  SW.NewLine(true);
        if (_thenext is null)
        {
            return;
        }
        _thenext.WriteParams(sw);
    }

    /// <summary>Gets internal data from another UndefinedEntity.</summary>
    public void GetFromAnother(UndefinedEntity another, CopyTool tc)
    {
        _thetype = new TCollection_HAsciiString(another.StepType());
        _thecont = new UndefinedContent();
        _thecont.GetFromAnother(another.UndefinedContent(), tc);
        _thesub = another.IsSub;
        if (another.IsComplex)
        {
            _thenext = tc.Transferred(another.Next()) as UndefinedEntity;
        }
        else
        {
            _thenext = null;
        }
    }

    /// <summary>Fills the list of shared entities.</summary>
    public void FillShared(EntityIterator list)
    {
        int i, nb = _thecont.ParamCount;
        for (i = 1; i <= nb; i++)
        {
            ParamType ptype = _thecont.ParamType(i);
            if (ptype == ParamType.ParamSub)
            {
                UndefinedEntity subent = _thecont.ParamEntity(i) as UndefinedEntity;
                subent.FillShared(list);
            }
            else if (ptype == ParamType.ParamIdent)
            {
                list.AddItem(_thecont.ParamEntity(i));
            }
        }
        if (_thenext is not null)
        {
            _thenext.FillShared(list);
        }
    }
}
