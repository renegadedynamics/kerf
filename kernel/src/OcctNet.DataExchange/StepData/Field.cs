// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Field.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Field.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Exchange.Step.Data.Logical;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// Defines a generally defined Field for STEP data: can be used either in any kind of entity to implement it, or in
/// free format entities in a "late-binding" mode. A field can have: no value (or derived), a single value of any
/// kind, a list of values (single or double list).
/// </summary>
/// <remarks>
/// A C++ value class, a C# class here (decision 1185): C++'s assignment (<c>operator=</c>, which copies) is
/// <see cref="CopyFrom"/>. The value is kept as <c>object</c>, since the port's <c>NCollection_HArray1</c> and
/// <c>HArray2</c> (which a field holds for its lists) are not <see cref="Transient"/> (decision 17).
/// </remarks>
public sealed class Field
{
    //  The kind encodes the data type, access mode (direct or via Select),
    //  and arity (simple, list, square array)
    //  Values for Kind: 0 = Clear/Undefined
    //  KindInteger KindBoolean KindLogical KindEnum KindReal KindString KindEntity
    //  + KindSelect which substitutes and can combine with them
    //  + KindList and KindList2 which can combine with them
    //  (on KindArity mask and ShiftArity offset)
    private const int KindInteger = 1;
    private const int KindBoolean = 2;
    private const int KindLogical = 3;
    private const int KindEnum = 4;
    private const int KindReal = 5;
    private const int KindString = 6;
    private const int KindEntity = 7;
    private const int KindAny = 8;
    private const int KindDerived = 9;
    private const int KindType = 15;
    private const int KindSelect = 16;
    private const int KindArity = 192;
    private const int KindList = 64;
    private const int KindList2 = 128;
    private const int ShiftArity = 6;

    private int _thekind;
    private int _theint;
    private double _thereal;
    private object _theany;

    private static int TrueKind(int kind)
    {
        return (kind & KindType);
    }

    /// <summary>Creates a Field, empty ("no value defined").</summary>
    public Field()
    {
        Clear();
    }

    /// <summary>
    /// Creates a Field from another one. If <paramref name="copy"/> is True, Handled data (Select, String, List, not
    /// entities) are copied.
    /// </summary>
    public Field(Field other, bool copy = false)
    {
        if (copy)
        {
            CopyFrom(other);
            return;
        }
        _thekind = other.Kind(false);
        _theint = other.Int();
        _thereal = other.Real();
        _theany = other.Transient();
    }

    /// <summary>Gets the copy of the values of another field (C++'s assignment operator).</summary>
    public void CopyFrom(Field other)
    {
        _thekind = other.Kind(false);
        _theint = other.Int();
        _thereal = other.Real();
        _theany = other.Transient();
        if (_thekind == KindString || _thekind == KindEnum)
        {
            if (_theany is TCollection_HAsciiString str)
            {
                _theany = new TCollection_HAsciiString(str.ToCString());
            }
            return;
        }
        if (_thekind == KindSelect)
        {
            //  Differents cas
            if (_theany is SelectReal sr)
            {
                double val = sr.Real();
                sr = new SelectReal();
                sr.SetReal(val);
                _theany = sr;
                return;
            }
            if (_theany is SelectInt si)
            {
                int ival = si.Int(), ik = si.Kind();
                si = new SelectInt();
                si.SetKind(ik);
                si.SetInt(ival);
                _theany = si;
                return;
            }
            if (_theany is SelectNamed sn)
            {
                SelectNamed sn2 = new SelectNamed();
                if (sn.HasName)
                {
                    sn2.SetName(sn2.Name()); // OCCT: names the copy with its own (empty) name
                }
                // OCCT.NET: OCCT copies *this (whose value is still sn) into sn2's field, which copies sn again into
                // a new member, without end: a stack overflow. The port copies sn's own field (occt-issues.md, 8B).
                sn2.CField().CopyFrom(sn.Field());
                _theany = sn2;
                return;
            }
        }
        //    Les listes ...
        if ((_thekind & KindArity) == KindList)
        {
            int i, low, up;
            if (_theany is NCollection_HArray1<int> hi)
            {
                low = hi.Lower();
                up = hi.Upper();
                NCollection_HArray1<int> hi2 = new NCollection_HArray1<int>(low, up);
                for (i = low; i <= up; i++)
                {
                    hi2.SetValue(i, hi.Value(i));
                }
                return; // OCCT: the copy is dropped, the field keeps sharing the list (occt-issues.md, 8B)
            }
            if (_theany is NCollection_HArray1<double> hr)
            {
                low = hr.Lower();
                up = hr.Upper();
                NCollection_HArray1<double> hr2 = new NCollection_HArray1<double>(low, up);
                for (i = low; i <= up; i++)
                {
                    hr2.SetValue(i, hr.Value(i));
                }
                return; // OCCT: the copy is dropped
            }
            if (_theany is NCollection_HArray1<TCollection_HAsciiString> hs)
            {
                low = hs.Lower();
                up = hs.Upper();
                NCollection_HArray1<TCollection_HAsciiString> hs2 = new NCollection_HArray1<TCollection_HAsciiString>(low, up);
                for (i = low; i <= up; i++)
                {
                    hs2.SetValue(i, new TCollection_HAsciiString(hs.Value(i)));
                }
                return; // OCCT: the copy is dropped
            }
            if (_theany is NCollection_HArray1<Transient> ht)
            {
                low = ht.Lower();
                up = ht.Upper();
                NCollection_HArray1<Transient> ht2 = new NCollection_HArray1<Transient>(low, up);
                //  Should handle SelectMember cases...
                for (i = low; i <= up; i++)
                {
                    ht2.SetValue(i, ht.Value(i));
                }
                return; // OCCT: the copy is dropped
            }
        }
        //    Remains the 2D list...
    }

    /// <summary>Clears the field, to set it as "no value defined". Just before SetList, predeclares it as "any".</summary>
    public void Clear(int kind = 0)
    {
        _thekind = kind;
        _theint = 0;
        _thereal = 0.0;
        _theany = null;
    }

    /// <summary>Codes a Field as derived (no proper value).</summary>
    public void SetDerived()
    {
        Clear(KindDerived);
    }

    /// <summary>Directly sets the Integer value, if its Kind matches Integer, Boolean, Logical, or Enum (does not change Kind).</summary>
    public void SetInt(int val)
    {
        if (_thekind == KindSelect)
        {
            if (_theany is SelectMember sm)
            {
                sm.SetInteger(val);
                return;
            }
        }
        if (_thekind == KindInteger || _thekind == KindBoolean || _thekind == KindLogical || _thekind == KindEnum)
        {
            _theint = val;
        }
        //  else ?
    }

    /// <summary>Sets an Integer value (before SetList* declares it as Integer).</summary>
    public void SetInteger(int val = 0)
    {
        if (_thekind == KindSelect)
        {
            if (_theany is SelectMember sm)
            {
                sm.SetInteger(val);
                return;
            }
        }
        Clear(KindInteger);
        _theint = val;
    }

    /// <summary>Sets a Boolean value (or predeclares a list as boolean).</summary>
    public void SetBoolean(bool val = false)
    {
        if (_thekind == KindSelect)
        {
            if (_theany is SelectMember sm)
            {
                sm.SetBoolean(val);
                return;
            }
        }
        Clear(KindBoolean);
        _theint = (val ? 1 : 0);
    }

    /// <summary>Sets a Logical Value (or predeclares a list as logical).</summary>
    public void SetLogical(Logical val = global::Kerf.Engine.Exchange.Step.Data.Logical.LFalse)
    {
        if (_thekind == KindSelect)
        {
            if (_theany is SelectMember sm)
            {
                sm.SetLogical(val);
                return;
            }
        }
        Clear(KindLogical);
        if (val == global::Kerf.Engine.Exchange.Step.Data.Logical.LFalse)
        {
            _theint = 0;
        }
        if (val == global::Kerf.Engine.Exchange.Step.Data.Logical.LTrue)
        {
            _theint = 1;
        }
        if (val == global::Kerf.Engine.Exchange.Step.Data.Logical.LUnknown)
        {
            _theint = 2;
        }
    }

    /// <summary>Sets a Real Value (or predeclares a list as Real).</summary>
    public void SetReal(double val = 0.0)
    {
        if (_thekind == KindSelect)
        {
            if (_theany is SelectMember sm)
            {
                sm.SetReal(val);
                return;
            }
        }
        Clear(KindReal);
        _thereal = val;
    }

    /// <summary>Sets a String Value (or predeclares a list as String). Does not redefine the kind if it is already String or Enum.</summary>
    public void SetString(string val = "")
    {
        if (_thekind == KindSelect)
        {
            if (_theany is SelectMember sm)
            {
                sm.SetString(val);
                return;
            }
        }
        if (_thekind != KindEnum)
        {
            Clear(KindString);
        }
        _theany = new TCollection_HAsciiString(val);
    }

    /// <summary>Sets an Enum Value (as its integer counterpart) (or predeclares a list as Enum). If <paramref name="text"/> is given, also sets its textual expression.</summary>
    public void SetEnum(int val = -1, string text = "")
    {
        Clear(KindEnum);
        SetInt(val);
        if (!string.IsNullOrEmpty(text))
        {
            SetString(text);
        }
    }

    /// <summary>Sets a SelectMember (for Integer, Boolean, Enum, Real, Logical). Hence, the value of the field is accessed through this member.</summary>
    public void SetSelectMember(SelectMember? val)
    {
        if (val is null)
        {
            return;
        }
        Clear(KindSelect);
        _theany = val;
    }

    /// <summary>Sets an Entity Value.</summary>
    public void SetEntity(Transient? val)
    {
        Clear(KindEntity);
        _theany = val;
    }

    /// <summary>Predeclares a list as of entity.</summary>
    public void SetEntity()
    {
        Transient nulent = null;
        SetEntity(nulent);
    }

    /// <summary>Declares a field as a list, with an initial size. Initial lower is defaulted as 1, can be defined. The list starts empty, typed by the last Set*. If no Set* before, sets it as "any" (transient/select).</summary>
    public void SetList(int size, int first = 1)
    {
        //  WARNING: we don't handle expansion...
        _theint = size;
        _thereal = 0.0;
        _theany = null; // ?? agrandissement ??
        switch (_thekind)
        {
            case KindInteger:
            case KindBoolean:
            case KindLogical:
                _theany = new NCollection_HArray1<int>(first, first + size - 1);
                break;
            case KindReal:
                _theany = new NCollection_HArray1<double>(first, first + size - 1);
                break;
            case KindEnum:
            case KindString:
                _theany = new NCollection_HArray1<TCollection_HAsciiString>(first, first + size - 1);
                break;
            //  default : en particulier si "non specifie" (any)
            default:
                _theany = new NCollection_HArray1<Transient>(first, first + size - 1);
                break;
        }
        if (_thekind == 0)
        {
            _thekind = KindAny;
        }
        _thekind |= KindList;
    }

    /// <summary>Declares a field as an homogeneous square list, with initial sizes, and initial lowers.</summary>
    public void SetList2(int siz1, int siz2, int f1 = 1, int f2 = 1)
    {
        //  WARNING: we don't handle expansion...
        _theint = siz1;
        _thereal = siz2;
        _theany = null;
        int kind = _thekind;
        if (_thekind == KindSelect)
        {
            if (_theany is SelectMember sm) // OCCT: theany was just cleared, never a member here
            {
                kind = sm.Kind();
            }
        }
        switch (kind)
        {
            case KindInteger:
            case KindBoolean:
            case KindLogical:
                _theany = new NCollection_HArray2<int>(f1, f1 + siz1 - 1, f2, f2 + siz2 - 1);
                break;
            case KindReal:
                _theany = new NCollection_HArray2<double>(f1, f1 + siz1 - 1, f2, f2 + siz2 - 1);
                break;
            case KindEnum:
            case KindString:
                _theany = new NCollection_HArray2<Transient>(f1, f1 + siz1 - 1, f2, f2 + siz2 - 1);
                break;
            //  default : en particulier si "non specifie" (any)
            default:
                _theany = new NCollection_HArray2<Transient>(f1, f1 + siz1 - 1, f2, f2 + siz2 - 1);
                break;
        }
        if (_thekind == 0)
        {
            _thekind = KindAny;
        }
        _thekind |= KindList2;
    }

    /// <summary>
    /// Sets an undetermined value: can be String, SelectMember, HArray(1-2) of Integer, Boolean, Real, String,
    /// Transient... not an Entity: use SetEntity. If the list was predeclared ("any"), the kind is kept.
    /// </summary>
    public void Set(object val)
    {
        int kind = _thekind;
        Clear();
        _theany = val;
        if (val is null)
        {
            return;
        }
        if (val is TCollection_HAsciiString)
        {
            _thekind = KindString;
            return;
        }
        if (val is SelectMember)
        {
            _thekind = KindSelect;
            return;
        }
        if (val is NCollection_HArray1<int> hi)
        {
            if (kind == 0)
            {
                kind = KindInteger;
            }
            _thekind = kind | KindList;
            _theint = hi.Length();
            return;
        }
        if (val is NCollection_HArray1<double> hr)
        {
            _thekind = KindReal | KindList;
            _theint = hr.Length();
            return;
        }
        if (val is NCollection_HArray1<TCollection_HAsciiString> hs)
        {
            _thekind = KindString | KindList;
            _theint = hs.Length();
            return;
        }
        if (val is NCollection_HArray1<Transient> ht)
        {
            if (kind == 0)
            {
                kind = KindAny;
            }
            _thekind = kind | KindList;
            _theint = ht.Length();
            return;
        }
        NCollection_HArray2<int> hi2 = val as NCollection_HArray2<int>;
        if (hi2 is not null)
        {
            if (kind == 0)
            {
                kind = KindInteger;
            }
            _thekind = kind | KindList2;
            _theint = hi2.ColLength();
            _thereal = hi2.RowLength();
            return;
        }
        if (val is NCollection_HArray2<double> hr2)
        {
            _thekind = KindInteger | KindList2; // OCCT: an array of reals is kind Integer
            _theint = hr2.ColLength();
            _thereal = hi2.RowLength(); // OCCT: hi2 is null here (a null dereference)
            return;
        }
        if (val is NCollection_HArray2<Transient> ht2)
        {
            if (kind == 0)
            {
                kind = KindAny;
            }
            _thekind = kind | KindList2;
            _theint = ht2.ColLength();
            _thereal = hi2.RowLength(); // OCCT: hi2 is null here (a null dereference)
            return;
        }
    }

    /// <summary>Declares an item of the list as undefined (ignored if list not defined as String, Entity or Any).</summary>
    public void ClearItem(int num)
    {
        if (_theany is NCollection_HArray1<Transient> ht)
        {
            ht.ChangeValue(num) = null;
        }
        if (_theany is NCollection_HArray1<TCollection_HAsciiString> hs)
        {
            hs.ChangeValue(num) = null;
        }
    }

    /// <summary>Internal access to an Integer Value for a list, plus its kind.</summary>
    public void SetInt(int num, int val, int kind)
    {
        if (_theany is NCollection_HArray1<int> hi)
        {
            hi.SetValue(num, val);
            return;
        }
        //   If already started with something else, change and put selects
        if (_theany is not NCollection_HArray1<Transient> ht)
        {
            return; // yena erreur, ou alors OfReal
        }
        _thekind = KindAny | KindList;
        SelectMember sm = ht.Value(num) as SelectMember;
        if (sm is null)
        {
            sm = new SelectInt();
            ht.SetValue(num, sm);
        }
        sm.SetKind(kind);
        sm.SetInt(val);
    }

    /// <summary>Sets an Integer Value for a list (rank num) (recognizes a SelectMember).</summary>
    public void SetInteger(int num, int val)
    {
        SetInt(num, val, KindInteger);
    }

    public void SetBoolean(int num, bool val)
    {
        SetInt(num, (val ? 1 : 0), KindBoolean);
    }

    public void SetLogical(int num, Logical val)
    {
        if (val == global::Kerf.Engine.Exchange.Step.Data.Logical.LFalse)
        {
            SetInt(num, 0, KindLogical);
        }
        if (val == global::Kerf.Engine.Exchange.Step.Data.Logical.LTrue)
        {
            SetInt(num, 1, KindLogical);
        }
        if (val == global::Kerf.Engine.Exchange.Step.Data.Logical.LUnknown)
        {
            SetInt(num, 2, KindLogical);
        }
    }

    /// <summary>Sets an Enum Value (Integer counterpart), also its text expression if known (if list has been set as "any").</summary>
    public void SetEnum(int num, int val, string text = "")
    {
        if (_theany is not NCollection_HArray1<Transient> ht)
        {
            SetInteger(num, val);
            return;
        }
        SelectMember sm = ht.Value(num) as SelectMember;
        _thekind = KindAny | KindList;
        if (sm is null)
        {
            sm = new SelectNamed();
            ht.SetValue(num, sm);
        }
        sm.SetEnum(val, text);
    }

    public void SetReal(int num, double val)
    {
        if (_theany is NCollection_HArray1<double> hr)
        {
            hr.SetValue(num, val);
            return;
        }
        //   If already started with something else, change and put selects
        if (_theany is not NCollection_HArray1<Transient> ht)
        {
            return; // yena erreur, ou alors OfInteger
        }
        _thekind = KindAny | KindList;
        SelectMember sm = ht.Value(num) as SelectMember;
        if (sm is null)
        {
            sm = new SelectReal();
            ht.SetValue(num, sm);
        }
        sm.SetReal(val);
    }

    public void SetString(int num, string val)
    {
        if (_theany is NCollection_HArray1<TCollection_HAsciiString> hs)
        {
            hs.SetValue(num, new TCollection_HAsciiString(val));
            return;
        }
        //    et si OfInteger ou OfReal ?
        if (_theany is not NCollection_HArray1<Transient> ht)
        {
            return;
        }
        _thekind = KindAny | KindList;
        ht.SetValue(num, new TCollection_HAsciiString(val));
    }

    public void SetEntity(int num, Transient? val)
    {
        if (_theany is NCollection_HArray1<Transient> aHt)
        {
            aHt.SetValue(num, val);
            return;
        }
        if (_theany is NCollection_HArray1<int> hi)
        {
            int low = hi.Lower(), up = hi.Upper();
            NCollection_HArray1<Transient> ht = new NCollection_HArray1<Transient>(low, up);
            SelectMember sm;
            int kind = Kind();
            for (int i = low; i <= up; i++)
            {
                if (i == num)
                {
                    ht.SetValue(i, val);
                }
                else
                {
                    sm = new SelectInt();
                    sm.SetKind(kind);
                    sm.SetInt(hi.Value(i));
                    ht.SetValue(i, sm);
                }
            }
            _thekind = KindAny | KindList;
            return; // OCCT: the new list is not kept, the field keeps its integers
        }
        if (_theany is NCollection_HArray1<double> hr)
        {
            int low = hr.Lower(), up = hr.Upper();
            NCollection_HArray1<Transient> ht = new NCollection_HArray1<Transient>(low, up);
            SelectMember sm;
            for (int i = low; i <= up; i++)
            {
                if (i == num)
                {
                    ht.SetValue(i, val);
                }
                else
                {
                    sm = new SelectReal();
                    sm.SetReal(hr.Value(i));
                    ht.SetValue(i, sm);
                }
            }
            _thekind = KindAny | KindList;
            return; // OCCT: the new list is not kept
        }
        if (_theany is NCollection_HArray1<TCollection_HAsciiString> hs)
        {
            int low = hs.Lower(), up = hs.Upper();
            NCollection_HArray1<Transient> ht = new NCollection_HArray1<Transient>(low, up);
            for (int i = low; i <= up; i++)
            {
                if (i == num)
                {
                    ht.SetValue(i, val);
                }
                else
                {
                    ht.SetValue(i, hs.Value(i));
                }
            }
            _thekind = KindAny | KindList;
            return; // OCCT: the new list is not kept
        }
    }

    //     QUERIES

    /// <summary>Tells if a Field is set (i.e. has a value).</summary>
    public bool IsSet(int n1 = 1, int n2 = 1)
    {
        if (_thekind == 0)
        {
            return false;
        }
        if (_thekind == KindSelect)
        {
            if (_theany is not SelectMember sm)
            {
                return false;
            }
            return (sm.Kind() != 0);
        }
        if ((_thekind & KindArity) == KindList)
        {
            if (_theany is NCollection_HArray1<Transient> ht)
            {
                return (ht.Value(n1) is not null);
            }
            if (_theany is NCollection_HArray1<TCollection_HAsciiString> hs)
            {
                return (hs.Value(n1) is not null);
            }
        }
        if ((_thekind & KindArity) == KindList2)
        {
            if (_theany is NCollection_HArray2<Transient> ht)
            {
                return (ht.Value(n1, n2) is not null);
            }
        }
        return true;
    }

    /// <summary>Returns the kind of an item in a list or double list. It is the kind of the list, except if it is "Any": then the effective item kind is returned.</summary>
    public int ItemKind(int n1 = 1, int n2 = 1)
    {
        if ((_thekind & KindArity) == 0)
        {
            return Kind(true);
        }
        int kind = TrueKind(_thekind); // si Any, evaluer ...
        if (kind != KindAny)
        {
            return kind;
        }
        //  Otherwise, look for a Transient
        Transient item = null;
        if ((_thekind & KindArity) == KindList)
        {
            NCollection_HArray1<Transient> ht = _theany as NCollection_HArray1<Transient>;
            if (ht is not null)
            {
                return kind; // OCCT: the test is inverted, a list of items answers Any ...
            }
            item = ht.Value(n1); // ... and any other one is dereferenced null
        }
        else if ((_thekind & KindArity) == KindList2)
        {
            NCollection_HArray2<Transient> ht = _theany as NCollection_HArray2<Transient>;
            if (ht is not null)
            {
                return kind; // OCCT: the test is inverted (see above)
            }
            item = ht.Value(n1, n2);
        }
        if (item is null)
        {
            return 0;
        }
        if (item is TCollection_HAsciiString)
        {
            return KindString;
        }
        if (item is not SelectMember sm)
        {
            return KindEntity;
        }
        return sm.Kind();
    }

    /// <summary>
    /// Returns the kind of the field. <paramref name="type"/> True (D) returns only the type itself, else returns the
    /// complete kind.
    /// </summary>
    public int Kind(bool type = true)
    {
        if (!type)
        {
            return _thekind;
        }
        if (_thekind == KindSelect)
        {
            if (_theany is SelectMember sm)
            {
                return TrueKind(sm.Kind());
            }
        }
        return TrueKind(_thekind);
    }

    /// <summary>Returns the list arity of the field: 0 for a single value, 1 for a simple list, 2 for a list of list.</summary>
    public int Arity()
    {
        return (_thekind & KindArity) >> ShiftArity;
    }

    /// <summary>Returns the length of a list (0 if not a list) for a list of list, specify index (1 or 2).</summary>
    public int Length(int index = 1)
    {
        if ((_thekind & KindArity) == KindList)
        {
            return _theint;
        }
        if ((_thekind & KindArity) == KindList2)
        {
            if (index == 2)
            {
                return (int)_thereal;
            }
            else
            {
                return _theint;
            }
        }
        return 0;
    }

    /// <summary>Returns the lower index of a list (0 if not a list).</summary>
    public int Lower(int index = 1)
    {
        if ((_thekind & KindArity) == KindList)
        {
            if (_theany is NCollection_HArray1<int> hi)
            {
                return hi.Lower();
            }
            if (_theany is NCollection_HArray1<double> hr)
            {
                return hr.Lower();
            }
            if (_theany is NCollection_HArray1<TCollection_HAsciiString> hs)
            {
                return hs.Lower();
            }
            if (_theany is NCollection_HArray1<Transient> ht)
            {
                return ht.Lower();
            }
        }
        if ((_thekind & KindArity) == KindList2)
        {
            if (_theany is not NCollection_HArray2<Transient> ht)
            {
                return 0;
            }
            if (index == 1)
            {
                return ht.LowerCol();
            }
            if (index == 2)
            {
                return ht.LowerRow();
            }
        }
        return 0;
    }

    /// <summary>Returns the integer value of a field (the direct value, whatever its kind).</summary>
    public int Int()
    {
        return _theint;
    }

    public int Integer(int n1 = 1, int n2 = 1)
    {
        if ((_thekind & KindArity) == 0)
        {
            if (_thekind == KindSelect)
            {
                if (_theany is SelectMember sm)
                {
                    return sm.Int();
                }
            }
            return _theint;
        }
        if ((_thekind & KindArity) == KindList)
        {
            if (_theany is NCollection_HArray1<int> hi)
            {
                return hi.Value(n1);
            }
            if (_theany is not NCollection_HArray1<Transient> ht)
            {
                return 0;
            }
            if (ht.Value(n1) is SelectMember sm)
            {
                return sm.Int();
            }
        }
        if ((_thekind & KindArity) == KindList2)
        {
            if (_theany is not NCollection_HArray2<Transient> ht)
            {
                return 0;
            }
            if (ht.Value(n1, n2) is SelectMember sm)
            {
                return sm.Int();
            }
        }
        return 0;
    }

    public bool Boolean(int n1 = 1, int n2 = 1)
    {
        return (Integer(n1, n2) > 0);
    }

    public Logical Logical(int n1 = 1, int n2 = 1)
    {
        int ival = Integer(n1, n2);
        if (ival == 0)
        {
            return global::Kerf.Engine.Exchange.Step.Data.Logical.LFalse;
        }
        if (ival == 1)
        {
            return global::Kerf.Engine.Exchange.Step.Data.Logical.LTrue;
        }
        return global::Kerf.Engine.Exchange.Step.Data.Logical.LUnknown;
    }

    public double Real(int n1 = 1, int n2 = 1)
    {
        if ((_thekind & KindArity) == 0)
        {
            if (_thekind == KindSelect)
            {
                if (_theany is SelectMember sm)
                {
                    return sm.Real();
                }
            }
            return _thereal;
        }
        if ((_thekind & KindArity) == KindList)
        {
            if (_theany is NCollection_HArray1<double> hr)
            {
                return hr.Value(n1);
            }
            if (_theany is not NCollection_HArray1<Transient> ht)
            {
                return 0;
            }
            if (ht.Value(n1) is SelectMember sm)
            {
                return sm.Real();
            }
        }
        if ((_thekind & KindArity) == KindList2)
        {
            if (_theany is NCollection_HArray2<double> hr)
            {
                return hr.Value(n1, n2);
            }
            if (_theany is not NCollection_HArray2<Transient> ht)
            {
                return 0;
            }
            if (ht.Value(n1, n2) is SelectMember sm)
            {
                return sm.Int(); // OCCT: the integer value of the member
            }
        }
        return 0.0;
    }

    public string String(int n1 = 1, int n2 = 1)
    {
        if (_thekind == KindString || _thekind == KindEnum)
        {
            if (_theany is TCollection_HAsciiString str)
            {
                return str.ToCString();
            }
            else
            {
                return "";
            }
        }
        if (_thekind == KindSelect)
        {
            if (_theany is SelectMember sm)
            {
                return sm.String();
            }
        }
        if ((_thekind & KindArity) == KindList)
        {
            if (_theany is NCollection_HArray1<TCollection_HAsciiString> hs)
            {
                if (hs.Value(n1) is null)
                {
                    return "";
                }
                else
                {
                    return hs.Value(n1).ToCString();
                }
            }
            if (_theany is not NCollection_HArray1<Transient> ht)
            {
                return "";
            }
            if (ht.Value(n1) is TCollection_HAsciiString str)
            {
                return str.ToCString();
            }
            if (ht.Value(n1) is SelectMember sm)
            {
                return sm.String();
            }
        }
        if ((_thekind & KindArity) == KindList2)
        {
            if (_theany is not NCollection_HArray2<Transient> ht)
            {
                return "";
            }
            if (ht.Value(n1, n2) is TCollection_HAsciiString str)
            {
                return str.ToCString();
            }
            if (ht.Value(n1, n2) is SelectMember sm)
            {
                return sm.String();
            }
        }
        return "";
    }

    public int Enum(int n1 = 1, int n2 = 1)
    {
        return Integer(n1, n2);
    }

    public string EnumText(int n1 = 1, int n2 = 1)
    {
        return String(n1, n2);
    }

    public Transient? Entity(int n1 = 1, int n2 = 1)
    {
        Transient nulval = null; // null handle
        if ((_thekind & KindArity) == 0)
        {
            if (_thekind == KindEntity)
            {
                return _theany as Transient;
            }
            return nulval;
        }
        if ((_thekind & KindArity) == KindList)
        {
            if (_theany is not NCollection_HArray1<Transient> ht)
            {
                return nulval;
            }
            nulval = ht.Value(n1);
            if (nulval is null)
            {
                return nulval;
            }
            if (nulval is SelectMember || nulval is TCollection_HAsciiString)
            {
                nulval = null;
            }
            return nulval;
        }
        if ((_thekind & KindArity) == KindList2)
        {
            if (_theany is not NCollection_HArray2<Transient> ht)
            {
                return nulval;
            }
            nulval = ht.Value(n1, n2);
            if (nulval is null)
            {
                return nulval;
            }
            if (nulval is SelectMember || nulval is TCollection_HAsciiString)
            {
                nulval = null;
            }
            return nulval;
        }
        return nulval;
    }

    /// <summary>The stored value as such: an entity, a string, a select member or a list (an <c>NCollection_HArray1/2</c>).</summary>
    public object Transient()
    {
        return _theany;
    }
}
