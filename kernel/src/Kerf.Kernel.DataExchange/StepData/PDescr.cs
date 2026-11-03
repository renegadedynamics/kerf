// Port of: src/DataExchange/TKDESTEP/StepData/StepData_PDescr.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_PDescr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework;
using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// This class is intended to describe the authorized form for a parameter, as a type or a value for a field. A
/// PDescr firstly describes a type, which can be SELECT, i.e. have several members.
/// </summary>
public class PDescr : Transient
{
    private const int KindInteger = 1;
    private const int KindBoolean = 2;
    private const int KindLogical = 3;
    private const int KindEnum = 4;
    private const int KindReal = 5;
    private const int KindString = 6;
    private const int KindEntity = 7;

    private string _thename = "";
    private int _thesel;
    private string _thesnam = "";
    private PDescr _thenext;
    private int _thekind;
    private readonly EnumTool _theenum = new();
    private Type _thetype;
    private string _thednam = "";
    private int _thearit;
    private PDescr _thefrom;
    private bool _theopt;
    private bool _theder;
    private string _thefnam = "";
    private int _thefnum;

    public PDescr()
    {
        _thesel = 0;
        _thekind = 0;
        _thearit = 0;
        _theopt = false;
        _theder = false;
        _thefnum = 0;
    }

    public void SetName(string name)
    {
        _thename = name ?? "";
    }

    public string Name()
    {
        return _thename;
    }

    private int Kind()
    {
        return _thekind;
    }

    /// <summary>Declares this PDescr to be a Select, hence to have members. This itself can be the first member.</summary>
    public void SetSelect()
    {
        _thesel = 4;
    }

    /// <summary>Adds a member to a SELECT description.</summary>
    public void AddMember(PDescr? member)
    {
        if (member is null)
        {
            return;
        }
        if (_thenext is null)
        {
            _thenext = member;
        }
        else
        {
            _thenext.AddMember(member);
        }
        if (_thesel == 3)
        {
            return;
        }
        if (_thekind < KindEntity && _thenext.Kind() >= KindEntity)
        {
            _thesel = 3;
        }
        else if (_thekind < KindEntity && (_thesel == 2 || _thesel == 4))
        {
            _thesel = 3;
        }
        else if (_thekind >= KindEntity && (_thesel == 1 || _thesel == 4))
        {
            _thesel = 2;
        }
    }

    /// <summary>Sets a name for SELECT member. To be used if a member is for an immediate type.</summary>
    public void SetMemberName(string memname)
    {
        _thesnam = memname ?? "";
    }

    public void SetInteger()
    {
        _thekind = KindInteger;
    }

    public void SetReal()
    {
        _thekind = KindReal;
    }

    public void SetString()
    {
        _thekind = KindString;
    }

    public void SetBoolean()
    {
        _thekind = KindBoolean;
    }

    public void SetLogical()
    {
        _thekind = KindLogical;
    }

    public void SetEnum()
    {
        _thekind = KindEnum;
    }

    /// <summary>Adds an enum value as a string.</summary>
    public void AddEnumDef(string enumdef)
    {
        _theenum.AddDefinition(enumdef);
    }

    /// <summary>Sets type for data to be a given type of entity.</summary>
    public void SetType(Type? atype)
    {
        _thekind = KindEntity;
        _thetype = atype;
        _thednam = "";
    }

    /// <summary>Sets type for data to be a described entity.</summary>
    public void SetDescr(string dscnam)
    {
        _thekind = KindEntity;
        _thetype = null;
        _thednam = dscnam ?? "";
    }

    /// <summary>Adds an arity count to this, by default 1. 1 : a simple field passes to a LIST/ARRAY etc or a LIST to a LIST OF LIST.</summary>
    public void AddArity(int arity = 1)
    {
        _thearit += arity;
    }

    /// <summary>Directly sets the arity count. 0 : simple field, 1 : LIST or ARRAY etc, 2 : LIST OF LIST.</summary>
    public void SetArity(int arity = 1)
    {
        _thearit = arity;
    }

    /// <summary>Sets this to be a copy of another one, except the select members (which are not copied).</summary>
    public void SetFrom(PDescr? other)
    {
        if (other is null)
        {
            return;
        }
        _thekind = other.Kind();
        int i, maxenum = other.EnumMax();
        for (i = 0; i <= maxenum; i++)
        {
            AddEnumDef(other.EnumText(i));
        }
        //  SELECT types are not copied
        _thetype = other.Type();
        _thearit = other.Arity();
        _thefrom = other;
        _theopt = other.IsOptional;
        _theder = other.IsDerived;
        _thefnam = other.FieldName();
        _thefnum = other.FieldRank();
    }

    /// <summary>Sets this description to be optional or not.</summary>
    public void SetOptional(bool opt = true)
    {
        _theopt = opt;
    }

    /// <summary>Sets this description to be derived or not.</summary>
    public void SetDerived(bool der = true)
    {
        _theder = der;
    }

    /// <summary>Sets this description to be a field of an entity: its name and rank.</summary>
    public void SetField(string name, int rank)
    {
        _thefnam = name ?? "";
        _thefnum = rank;
    }

    //    ######  INTERRO  ######

    /// <summary>Tells if this PDescr is a SELECT.</summary>
    public bool IsSelect()
    {
        if (_thefrom is not null)
        {
            return _thefrom.IsSelect();
        }
        return (_thesel > 0);
    }

    /// <summary>For a SELECT, returns the member whose name matches name. To this name, the following criteria are applied: for a member, its member name (if any); for a member being an entity, the entity name.</summary>
    public PDescr? Member(string name)
    {
        if (_thefrom is not null)
        {
            return _thefrom.Member(name);
        }
        PDescr descr = null;
        if (string.Equals(_thesnam, name, StringComparison.Ordinal))
        {
            return this;
        }
        if (_thenext is null)
        {
            return descr; // null
        }
        return _thenext.Member(name);
    }

    public bool IsInteger
    {
        get
        {
            return (_thekind == KindInteger);
        }
    }

    public bool IsReal
    {
        get
        {
            return (_thekind == KindReal);
        }
    }

    public bool IsString
    {
        get
        {
            return (_thekind == KindString);
        }
    }

    public bool IsBoolean
    {
        get
        {
            return (_thekind == KindBoolean || _thekind == KindLogical);
        }
    }

    public bool IsLogical
    {
        get
        {
            return (_thekind == KindLogical);
        }
    }

    public bool IsEnum
    {
        get
        {
            return (_thekind == KindEnum);
        }
    }

    public int EnumMax()
    {
        return _theenum.MaxValue();
    }

    public int EnumValue(string name)
    {
        return _theenum.Value(name);
    }

    public string EnumText(int val)
    {
        return _theenum.Text(val);
    }

    public bool IsEntity
    {
        get
        {
            return (_thekind == KindEntity);
        }
    }

    /// <summary>Tells if this PDescr is for a type of entity compatible with atype (its type or a sub-type of it).</summary>
    public bool IsType(Type? atype)
    {
        if (atype is null)
        {
            return false;
        }
        if (_thetype is not null)
        {
            if (_thetype.IsAssignableFrom(atype))
            {
                return true;
            }
        }
        if (_thenext is not null)
        {
            return _thenext.IsType(atype);
        }
        if (_thefrom is not null)
        {
            return _thefrom.IsType(atype);
        }
        return false;
    }

    /// <summary>For a non-select entity type, returns this type; else null.</summary>
    public Type? Type()
    {
        return _thetype;
    }

    /// <summary>Tells if this PDescr is for a described entity compatible with descr.</summary>
    public bool IsDescr(EDescr? descr)
    {
        if (descr is null)
        {
            return false;
        }
        if (_thednam.Length > 0)
        {
            if (descr.Matches(_thednam))
            {
                return true;
            }
        }
        if (_thenext is not null)
        {
            return _thenext.IsDescr(descr);
        }
        if (_thefrom is not null)
        {
            return _thefrom.IsDescr(descr);
        }
        return false;
    }

    /// <summary>For a described entity, returns its name.</summary>
    public string DescrName()
    {
        return _thednam;
    }

    /// <summary>Returns the arity of this PDescr: 0 for a simple field, 1 for a LIST, 2 for a LIST OF LIST.</summary>
    public int Arity()
    {
        return _thearit;
    }

    /// <summary>For a LIST or LIST OF LIST, returns the PDescr for the simpler PDescr. Else returns this. This allows to have different attributes for Optional for instance, on a field, and on the parameter of a LIST.</summary>
    public PDescr Simple()
    {
        if (_thearit == 0)
        {
            return this;
        }
        if (_thefrom is null)
        {
            return this;
        }
        return _thefrom;
    }

    public bool IsOptional
    {
        get
        {
            return _theopt;
        }
    }

    public bool IsDerived
    {
        get
        {
            return _theder;
        }
    }

    /// <summary>Tells if this field is defined for an entity.</summary>
    public bool IsField
    {
        get
        {
            return (_thefnum > 0);
        }
    }

    public string FieldName()
    {
        return _thefnam;
    }

    public int FieldRank()
    {
        return _thefnum;
    }

    /// <summary>Semantic Check of a Field: does it complies with the given description?</summary>
    public virtual void Check(Field afild, FrameworkCheck ach)
    {
        //  For now...
    }
}
