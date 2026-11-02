// Port of: src/DataExchange/TKDESTEP/StepData/StepData_PDescr.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_PDescr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This class is intended to describe the authorized form for a parameter, as a type or a value for a field. A
/// PDescr firstly describes a type, which can be SELECT, i.e. have several members.
/// </summary>
public class StepData_PDescr : Standard_Transient
{
    private const int KindInteger = 1;
    private const int KindBoolean = 2;
    private const int KindLogical = 3;
    private const int KindEnum = 4;
    private const int KindReal = 5;
    private const int KindString = 6;
    private const int KindEntity = 7;

    private string thename = "";
    private int thesel;
    private string thesnam = "";
    private StepData_PDescr thenext;
    private int thekind;
    private readonly StepData_EnumTool theenum = new();
    private Type thetype;
    private string thednam = "";
    private int thearit;
    private StepData_PDescr thefrom;
    private bool theopt;
    private bool theder;
    private string thefnam = "";
    private int thefnum;

    public StepData_PDescr()
    {
        thesel = 0;
        thekind = 0;
        thearit = 0;
        theopt = false;
        theder = false;
        thefnum = 0;
    }

    public void SetName(string name)
    {
        thename = name ?? "";
    }

    public string Name()
    {
        return thename;
    }

    private int Kind()
    {
        return thekind;
    }

    /// <summary>Declares this PDescr to be a Select, hence to have members. This itself can be the first member.</summary>
    public void SetSelect()
    {
        thesel = 4;
    }

    /// <summary>Adds a member to a SELECT description.</summary>
    public void AddMember(StepData_PDescr? member)
    {
        if (member is null)
        {
            return;
        }
        if (thenext is null)
        {
            thenext = member;
        }
        else
        {
            thenext.AddMember(member);
        }
        if (thesel == 3)
        {
            return;
        }
        if (thekind < KindEntity && thenext.Kind() >= KindEntity)
        {
            thesel = 3;
        }
        else if (thekind < KindEntity && (thesel == 2 || thesel == 4))
        {
            thesel = 3;
        }
        else if (thekind >= KindEntity && (thesel == 1 || thesel == 4))
        {
            thesel = 2;
        }
    }

    /// <summary>Sets a name for SELECT member. To be used if a member is for an immediate type.</summary>
    public void SetMemberName(string memname)
    {
        thesnam = memname ?? "";
    }

    public void SetInteger()
    {
        thekind = KindInteger;
    }

    public void SetReal()
    {
        thekind = KindReal;
    }

    public void SetString()
    {
        thekind = KindString;
    }

    public void SetBoolean()
    {
        thekind = KindBoolean;
    }

    public void SetLogical()
    {
        thekind = KindLogical;
    }

    public void SetEnum()
    {
        thekind = KindEnum;
    }

    /// <summary>Adds an enum value as a string.</summary>
    public void AddEnumDef(string enumdef)
    {
        theenum.AddDefinition(enumdef);
    }

    /// <summary>Sets type for data to be a given type of entity.</summary>
    public void SetType(Type? atype)
    {
        thekind = KindEntity;
        thetype = atype;
        thednam = "";
    }

    /// <summary>Sets type for data to be a described entity.</summary>
    public void SetDescr(string dscnam)
    {
        thekind = KindEntity;
        thetype = null;
        thednam = dscnam ?? "";
    }

    /// <summary>Adds an arity count to this, by default 1. 1 : a simple field passes to a LIST/ARRAY etc or a LIST to a LIST OF LIST.</summary>
    public void AddArity(int arity = 1)
    {
        thearit += arity;
    }

    /// <summary>Directly sets the arity count. 0 : simple field, 1 : LIST or ARRAY etc, 2 : LIST OF LIST.</summary>
    public void SetArity(int arity = 1)
    {
        thearit = arity;
    }

    /// <summary>Sets this to be a copy of another one, except the select members (which are not copied).</summary>
    public void SetFrom(StepData_PDescr? other)
    {
        if (other is null)
        {
            return;
        }
        thekind = other.Kind();
        int i, maxenum = other.EnumMax();
        for (i = 0; i <= maxenum; i++)
        {
            AddEnumDef(other.EnumText(i));
        }
        //  SELECT types are not copied
        thetype = other.Type();
        thearit = other.Arity();
        thefrom = other;
        theopt = other.IsOptional();
        theder = other.IsDerived();
        thefnam = other.FieldName();
        thefnum = other.FieldRank();
    }

    /// <summary>Sets this description to be optional or not.</summary>
    public void SetOptional(bool opt = true)
    {
        theopt = opt;
    }

    /// <summary>Sets this description to be derived or not.</summary>
    public void SetDerived(bool der = true)
    {
        theder = der;
    }

    /// <summary>Sets this description to be a field of an entity: its name and rank.</summary>
    public void SetField(string name, int rank)
    {
        thefnam = name ?? "";
        thefnum = rank;
    }

    //    ######  INTERRO  ######

    /// <summary>Tells if this PDescr is a SELECT.</summary>
    public bool IsSelect()
    {
        if (thefrom is not null)
        {
            return thefrom.IsSelect();
        }
        return (thesel > 0);
    }

    /// <summary>For a SELECT, returns the member whose name matches name. To this name, the following criteria are applied: for a member, its member name (if any); for a member being an entity, the entity name.</summary>
    public StepData_PDescr? Member(string name)
    {
        if (thefrom is not null)
        {
            return thefrom.Member(name);
        }
        StepData_PDescr descr = null;
        if (string.Equals(thesnam, name, StringComparison.Ordinal))
        {
            return this;
        }
        if (thenext is null)
        {
            return descr; // null
        }
        return thenext.Member(name);
    }

    public bool IsInteger()
    {
        return (thekind == KindInteger);
    }

    public bool IsReal()
    {
        return (thekind == KindReal);
    }

    public bool IsString()
    {
        return (thekind == KindString);
    }

    public bool IsBoolean()
    {
        return (thekind == KindBoolean || thekind == KindLogical);
    }

    public bool IsLogical()
    {
        return (thekind == KindLogical);
    }

    public bool IsEnum()
    {
        return (thekind == KindEnum);
    }

    public int EnumMax()
    {
        return theenum.MaxValue();
    }

    public int EnumValue(string name)
    {
        return theenum.Value(name);
    }

    public string EnumText(int val)
    {
        return theenum.Text(val);
    }

    public bool IsEntity()
    {
        return (thekind == KindEntity);
    }

    /// <summary>Tells if this PDescr is for a type of entity compatible with atype (its type or a sub-type of it).</summary>
    public bool IsType(Type? atype)
    {
        if (atype is null)
        {
            return false;
        }
        if (thetype is not null)
        {
            if (thetype.IsAssignableFrom(atype))
            {
                return true;
            }
        }
        if (thenext is not null)
        {
            return thenext.IsType(atype);
        }
        if (thefrom is not null)
        {
            return thefrom.IsType(atype);
        }
        return false;
    }

    /// <summary>For a non-select entity type, returns this type; else null.</summary>
    public Type? Type()
    {
        return thetype;
    }

    /// <summary>Tells if this PDescr is for a described entity compatible with descr.</summary>
    public bool IsDescr(StepData_EDescr? descr)
    {
        if (descr is null)
        {
            return false;
        }
        if (thednam.Length > 0)
        {
            if (descr.Matches(thednam))
            {
                return true;
            }
        }
        if (thenext is not null)
        {
            return thenext.IsDescr(descr);
        }
        if (thefrom is not null)
        {
            return thefrom.IsDescr(descr);
        }
        return false;
    }

    /// <summary>For a described entity, returns its name.</summary>
    public string DescrName()
    {
        return thednam;
    }

    /// <summary>Returns the arity of this PDescr: 0 for a simple field, 1 for a LIST, 2 for a LIST OF LIST.</summary>
    public int Arity()
    {
        return thearit;
    }

    /// <summary>For a LIST or LIST OF LIST, returns the PDescr for the simpler PDescr. Else returns this. This allows to have different attributes for Optional for instance, on a field, and on the parameter of a LIST.</summary>
    public StepData_PDescr Simple()
    {
        if (thearit == 0)
        {
            return this;
        }
        if (thefrom is null)
        {
            return this;
        }
        return thefrom;
    }

    public bool IsOptional()
    {
        return theopt;
    }

    public bool IsDerived()
    {
        return theder;
    }

    /// <summary>Tells if this field is defined for an entity.</summary>
    public bool IsField()
    {
        return (thefnum > 0);
    }

    public string FieldName()
    {
        return thefnam;
    }

    public int FieldRank()
    {
        return thefnum;
    }

    /// <summary>Semantic Check of a Field: does it complies with the given description?</summary>
    public virtual void Check(StepData_Field afild, Interface_Check ach)
    {
        //  For now...
    }
}
