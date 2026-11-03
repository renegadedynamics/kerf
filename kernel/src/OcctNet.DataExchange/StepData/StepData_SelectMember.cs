// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectMember.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectMember.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using static Occt.Interface_ParamType;
using static Occt.StepData_Logical;

namespace Occt;

/// <summary>
/// The general form for a Select Member. A Select Member can either define a value of a basic type (such as an
/// integer) with an additional information: a name or list of names which precise the meaning of this value, or be
/// an alternate value in a select, which also accepts an entity (in this case, the name is not mandatory).
/// </summary>
/// <remarks>
/// Several sub-types of SelectMember are defined for integer and real value, plus an "universal" one for any, and one
/// more to describe a select with several names. This class itself could be deferred: at least one of its virtual
/// methods must be redefined to be usable.
/// </remarks>
public class StepData_SelectMember : Transient
{
    //  Definitions taken from Field:
    private const int KindInteger = 1;
    private const int KindBoolean = 2;
    private const int KindLogical = 3;
    private const int KindEnum = 4;

    public StepData_SelectMember()
    {
    }

    /// <summary>Tells if a SelectMember has a name. Default is False.</summary>
    public virtual bool HasName()
    {
        return false;
    }

    /// <summary>Returns the name of a SelectMember. Default is empty.</summary>
    public virtual string Name()
    {
        return "";
    }

    /// <summary>
    /// Sets the name of a SelectMember, returns True if done, False if no name is allowed. Default does nothing and
    /// returns False.
    /// </summary>
    public virtual bool SetName(string name)
    {
        return false;
    }

    /// <summary>Tells if the name of a SelectMember matches a given one. By default, compares the strings.</summary>
    public virtual bool Matches(string name)
    {
        return string.Equals(name, Name(), StringComparison.Ordinal);
    }

    public virtual int Kind()
    {
        return 0;
    }

    public virtual void SetKind(int kind)
    {
    }

    /// <summary>Returns the Kind of the SelectMember, under the form of an enum ParamType.</summary>
    public Interface_ParamType ParamType()
    {
        int kind = Kind();
        if (kind == 0)
        {
            return Interface_ParamVoid;
        }
        if (kind == 1)
        {
            return Interface_ParamInteger;
        }
        if (kind == 2 || kind == 3)
        {
            return Interface_ParamLogical;
        }
        if (kind == 4)
        {
            return Interface_ParamEnum;
        }
        if (kind == 5)
        {
            return Interface_ParamReal;
        }
        if (kind == 6)
        {
            return Interface_ParamText;
        }
        return Interface_ParamMisc;
    }

    /// <summary>This internal method gives access to a value implemented by an Integer (to read it).</summary>
    public virtual int Int()
    {
        return 0;
    }

    /// <summary>This internal method gives access to a value implemented by an Integer (to set it).</summary>
    public virtual void SetInt(int val)
    {
    }

    /// <summary>Gets the value as an Integer.</summary>
    public int Integer()
    {
        return Int();
    }

    public void SetInteger(int val)
    {
        SetKind(KindInteger);
        SetInt(val);
    }

    public bool Boolean()
    {
        return (Int() > 0);
    }

    public void SetBoolean(bool val)
    {
        SetKind(KindBoolean);
        SetInt((val ? 1 : 0));
    }

    public StepData_Logical Logical()
    {
        int ival = Int();
        if (ival == 0)
        {
            return StepData_LFalse;
        }
        if (ival == 1)
        {
            return StepData_LTrue;
        }
        return StepData_LUnknown;
    }

    public void SetLogical(StepData_Logical val)
    {
        SetKind(KindLogical);
        // OCCT: every value is stored as 0, so a logical member always reads back as False (occt-issues.md, 8B)
        if (val == StepData_LFalse)
        {
            SetInt(0);
        }
        if (val == StepData_LTrue)
        {
            SetInt(0);
        }
        if (val == StepData_LUnknown)
        {
            SetInt(0);
        }
    }

    public virtual double Real()
    {
        return 0.0;
    }

    public virtual void SetReal(double val)
    {
    }

    public virtual string String()
    {
        return "";
    }

    public virtual void SetString(string val)
    {
    }

    public int Enum()
    {
        return Int();
    }

    public virtual string EnumText()
    {
        return String();
    }

    public void SetEnum(int val, string text = "")
    {
        SetKind(KindEnum);
        SetInt(val);
        if (!string.IsNullOrEmpty(text))
        {
            SetEnumText(val, text);
        }
    }

    public virtual void SetEnumText(int val, string text)
    {
        SetString(text);
    }
}
