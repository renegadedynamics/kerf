// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectType.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectType.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Exchange.Step.Data.Logical;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// SelectType is the basis used for SELECT_TYPE definitions from the EXPRESS form. A SELECT_TYPE in EXPRESS is an
/// enumeration of Types, it corresponds in a way to a Super-Type, but with no specific Methods, and no exclusivity (a
/// given Type can be member of several SELECT_TYPES, plus be itself a SUB_TYPE).
/// </summary>
/// <remarks>
/// <para>
/// A SelectType can be field of a Transient Entity or only used to control an input Argument. This class implies to
/// designate each member Type by a Case Number which is a positive Integer value (this allows a faster treatment).
/// With this class, a specific SelectType can recognize an Entity as complying or not with its definition, store it
/// (with the guarantee that the stored Entity complies with the definition of the SelectType), and give the stored
/// Entity under the good Type rather than simply "Transient".
/// </para>
/// <para>
/// A C++ value class, a C# class here (decision 1185): C++ copies a select when it assigns it (an entity's
/// <c>Init</c> stores a copy of its argument). A select holds a single reference, so sharing one differs from
/// copying only when the select is set again afterwards; <see cref="SetFrom"/> is C++'s assignment.
/// </para>
/// </remarks>
public abstract class SelectType
{
    private Transient _thevalue;

    /// <summary>
    /// Recognizes the Type of an Entity. Returns a positive Number which identifies the Type in the definition List
    /// of the SelectType. Returns Zero if its Type in not in this List.
    /// </summary>
    public abstract int CaseNum(Transient? ent);

    /// <summary>
    /// Returns True if the Type of an Entity complies with the definition list of the SelectType. Also checks for a
    /// SelectMember. Default Implementation looks for CaseNum or CaseMem positive.
    /// </summary>
    public bool Matches(Transient? ent)
    {
        if (CaseNum(ent) > 0)
        {
            return true;
        }
        if (ent is not SelectMember sm)
        {
            return false;
        }
        if (CaseMem(sm) > 0)
        {
            return true;
        }
        return false;
    }

    /// <summary>
    /// Stores an Entity. This allows to define a specific SelectType class with one read method per member Type,
    /// which returns the Value casted with the good Type.
    /// </summary>
    /// <exception cref="TypeMismatchException">The entity does not match the select (an undefined entity always does).</exception>
    public void SetValue(Transient? ent)
    {
        if (ent is null)
        {
            _thevalue = null;
        }
        else if (ent is UndefinedEntity)
        {
            _thevalue = ent;
        }
        else if (!Matches(ent))
        {
            throw new TypeMismatchException("StepData : SelectType, SetValue");
        }
        else
        {
            _thevalue = ent;
        }
    }

    /// <summary>C++'s assignment: takes the stored value of another select (of the same select type).</summary>
    public void SetFrom(SelectType other)
    {
        _thevalue = other._thevalue;
    }

    /// <summary>Nullifies the Stored Entity.</summary>
    public void Nullify()
    {
        _thevalue = null;
    }

    /// <summary>Returns the Stored Entity. Can be used to define specific read methods (see above).</summary>
    public Transient? Value()
    {
        return _thevalue;
    }

    /// <summary>Returns True if there is no Stored Entity (i.e. it is Null).</summary>
    public bool IsNull
    {
        get
        {
            return _thevalue is null;
        }
    }

    /// <summary>Returns the Effective (Dynamic) Type of the Stored Entity. If it is Null, returns Standard_Transient.</summary>
    public Type Type()
    {
        if (_thevalue is null)
        {
            return typeof(Transient);
        }
        return _thevalue.DynamicType();
    }

    /// <summary>Recognizes the Type of the stored Entity, or zero if it is Null or SelectMember. Calls the first method CaseNum on Value.</summary>
    public int CaseNumber()
    {
        if (_thevalue is null)
        {
            return 0;
        }
        return CaseNum(_thevalue);
    }

    //  **********   Types Immediats   ***********

    /// <summary>
    /// Returns the Description which corresponds to this select. Null if no specific description to give. This
    /// description is used to control reading an check validity. Default returns a Null Handle, i.e. undefined
    /// description. It can suffice if CaseNum and CaseMem give enough control.
    /// </summary>
    public virtual PDescr? Description()
    {
        PDescr nuldescr = null;
        return nuldescr;
    }

    /// <summary>
    /// Returns a preferred SelectMember. Default returns a Null. By default, a SelectMember can be set according to
    /// data type and Name: it is a SelectNamed if Name is defined. This method allows to define, for a specific
    /// SelectType, a specific SelectMember than SelectNamed.
    /// </summary>
    public virtual SelectMember? NewMember()
    {
        SelectMember nulmem = null;
        return nulmem;
    }

    /// <summary>
    /// Recognize a SelectMember (kind, name). Returns a positive value which identifies the case in the List of
    /// immediate cases (distinct from the List of Entity Types). Zero if not recognizes. Default returns 0, saying that
    /// no immediate value is allowed.
    /// </summary>
    public virtual int CaseMem(SelectMember? ent)
    {
        return 0;
    }

    /// <summary>Returns the Type of the stored SelectMember, or zero if it is Null or Entity. Calls the method CaseMem on Value.</summary>
    public int CaseMember()
    {
        if (_thevalue is not SelectMember sm)
        {
            return 0;
        }
        return CaseMem(sm);
    }

    /// <summary>Returns Value as a SelectMember. Null if not a SelectMember.</summary>
    public SelectMember? Member()
    {
        return _thevalue as SelectMember;
    }

    /// <summary>
    /// Returns the type name of SelectMember. If no SelectMember or with no type name, returns an empty string. To
    /// change it, pass through the SelectMember itself.
    /// </summary>
    public string SelectName()
    {
        if (_thevalue is not SelectMember sm)
        {
            return "";
        }
        return sm.Name();
    }

    /// <summary>This internal method gives access to a value implemented by an Integer (to read it).</summary>
    public int Int()
    {
        if (_thevalue is not SelectMember sm)
        {
            return 0;
        }
        return sm.Int();
    }

    /// <summary>This internal method gives access to a value implemented by an Integer (to set it): a SelectMember MUST ALREADY BE THERE.</summary>
    public void SetInt(int val)
    {
        if (_thevalue is not SelectMember sm)
        {
            throw new TypeMismatchException("StepData : SelectType, SetInt");
        }
        sm.SetInt(val);
    }

    //  **********   Types Immediats : Differents Cas  ***********

    private static SelectMember SelectVal(Transient thevalue, string name, int mode)
    {
        SelectMember sm = thevalue as SelectMember;
        if (sm is not null)
        {
            if (!string.IsNullOrEmpty(name))
            {
                if (!sm.SetName(name))
                {
                    throw new TypeMismatchException("StepData : SelectType, SetInteger");
                }
            }
        }
        else if (!string.IsNullOrEmpty(name))
        {
            SelectNamed sn = new SelectNamed();
            sn.SetName(name);
            sm = sn;
        }
        else
        {
            if (mode == 0)
            {
                sm = new SelectInt();
            }
            if (mode == 1)
            {
                sm = new SelectReal();
            }
        }
        return sm;
    }

    /// <summary>Gets the value as an Integer.</summary>
    public int Integer()
    {
        if (_thevalue is not SelectMember sm)
        {
            return 0;
        }
        return sm.Integer();
    }

    /// <summary>
    /// Sets a new Integer value, with an optional type name. If a SelectMember is already set, works on it: value and
    /// name must then be accepted by this SelectMember.
    /// </summary>
    public void SetInteger(int val, string name = "")
    {
        SelectMember sm = SelectVal(_thevalue, name, 0);
        sm.SetInteger(val);
        if (CaseMem(sm) == 0)
        {
            throw new TypeMismatchException("StepData : SelectType, SetInteger");
        }
        _thevalue = sm;
    }

    public bool Boolean()
    {
        if (_thevalue is not SelectMember sm)
        {
            return false;
        }
        return sm.Boolean();
    }

    public void SetBoolean(bool val, string name = "")
    {
        SelectMember sm = SelectVal(_thevalue, name, 0);
        sm.SetBoolean(val);
        if (CaseMem(sm) == 0)
        {
            throw new TypeMismatchException("StepData : SelectType, SetBoolean");
        }
        _thevalue = sm;
    }

    public Logical Logical()
    {
        if (_thevalue is not SelectMember sm)
        {
            return global::Kerf.Engine.Exchange.Step.Data.Logical.LUnknown;
        }
        return sm.Logical();
    }

    public void SetLogical(Logical val, string name = "")
    {
        SelectMember sm = SelectVal(_thevalue, name, 0);
        sm.SetLogical(val);
        if (CaseMem(sm) == 0)
        {
            throw new TypeMismatchException("StepData : SelectType, SetLogical");
        }
        _thevalue = sm;
    }

    public double Real()
    {
        if (_thevalue is not SelectMember sm)
        {
            return 0.0;
        }
        return sm.Real();
    }

    public void SetReal(double val, string name = "")
    {
        SelectMember sm = SelectVal(_thevalue, name, 1);
        sm.SetReal(val);
        if (CaseMem(sm) == 0)
        {
            throw new TypeMismatchException("StepData : SelectType, SetReal");
        }
        _thevalue = sm;
    }
}
