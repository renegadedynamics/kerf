// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransferIterator.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransferIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// Defines an Iterator on the result of a Transfer. Available for Normal Results or not (Erroneous Transfer). It gives
/// several kinds of Information, and allows to consider various criteria (criteria are cumulative).
/// </summary>
/// <remarks>A value class in C++ (decision 1185): a C++ copy is written <see cref="TransferIterator(TransferIterator)"/>.</remarks>
internal class TransferIterator
{
    /// <summary>Creates an empty Iterator.</summary>
    public TransferIterator()
    {
        _theitems = new NCollection_HSequence<Binder>();
        _theselect = new NCollection_HSequence<int>();
        _themaxi = 0;
        _thecurr = 1;
    }

    /// <summary>A copy (C++'s copy constructor, which shares the handles of the lists).</summary>
    public TransferIterator(TransferIterator other)
    {
        _theitems = other._theitems;
        _theselect = other._theselect;
        _themaxi = other._themaxi;
        _thecurr = other._thecurr;
    }

    /// <summary>Adds a Binder to the iteration list (construction).</summary>
    public void AddItem(Binder? atr)
    {
        _theitems.Append(atr);
        _theselect.Append(1);
        _themaxi = _theselect.Length();
    }

    /// <summary>
    /// Selects Items on the Type of Binder : keep only Binders which are of a given Type (if keep is True) or reject
    /// only them (if keep is False).
    /// </summary>
    public void SelectBinder(Type? atype, bool keep)
    {
        for (int i = _theitems.Length(); i > 0; i--)
        {
            if (IsKind(_theitems.Value(i), atype) != keep)
            {
                _theselect.SetValue(i, 0);
                if (_themaxi == i)
                {
                    _themaxi = i - 1;
                }
            }
        }
    }

    /// <summary>
    /// Selects Items on the Type of Result. Considers only Unique Results. Considers Dynamic Type for Transient
    /// Result, Static Type (the one given to define the Binder) else. Results which are of a given Type (if keep is
    /// True) or reject only them (if keep is False).
    /// </summary>
    public void SelectResult(Type? atype, bool keep)
    {
        int casetype = 0;
        if (SubType(atype, typeof(Transient)))
        {
            casetype = 2;
        }

        for (int i = _theitems.Length(); i > 0; i--)
        {
            Binder atr = _theitems.Value(i);
            Type? btype = ResultType(); // OCCT: the current item's result type, not atr's
            bool matchtype;
            if (!atr.HasResult)
            {
                matchtype = false;
            }
            else if (atr.IsMultiple())
            {
                matchtype = false;
            }
            else if (casetype == 0)
            {
                matchtype = atype == btype; // Type fixe
            }
            else
            {
                matchtype = SubType(btype, atype); // Dynamique
            }
            if (matchtype != keep)
            {
                _theselect.SetValue(i, 0);
                if (_themaxi == i)
                {
                    _themaxi = i - 1;
                }
            }
        }
    }

    /// <summary>
    /// Select Items according Unicity : keep only Unique Results (if keep is True) or keep only Multiple Results (if
    /// keep is False).
    /// </summary>
    public void SelectUnique(bool keep)
    {
        for (int i = _theitems.Length(); i > 0; i--)
        {
            Binder atr = _theitems.Value(i);
            if (atr.IsMultiple() == keep)
            {
                _theselect.SetValue(i, 0);
                if (_themaxi == i)
                {
                    _themaxi = i - 1;
                }
            }
        }
    }

    /// <summary>
    /// Selects/Unselect (according to <paramref name="keep"/>) an item designated by its rank <paramref name="num"/>
    /// in the list. Used by sub-classes which have specific criteria.
    /// </summary>
    public void SelectItem(int num, bool keep)
    {
        if (num < 1 || num > _theselect.Length())
        {
            return;
        }
        if (keep)
        {
            _theselect.SetValue(num, 1);
        }
        else
        {
            _theselect.SetValue(num, 0);
        }
    }

    //  ....                Iteration-Interrogations                ....

    /// <summary>Returns count of Binders to be iterated.</summary>
    public int Number()
    {
        int numb, i;
        numb = 0;
        for (i = 1; i <= _themaxi; i++)
        {
            if (_theselect.Value(i) != 0)
            {
                numb++;
            }
        }
        return numb;
    }

    /// <summary>Clears Iteration in progress, to allow it to be restarted.</summary>
    public void Start()
    {
        _thecurr = 0;
        Next();
    }

    /// <summary>Returns True if there are other Items to iterate.</summary>
    public bool More()
    {
        if (_thecurr > _themaxi)
        {
            return false;
        }
        if (_theselect.Value(_thecurr) == 0)
        {
            Next();
        }
        if (_thecurr > _themaxi)
        {
            return false;
        }
        return _theselect.Value(_thecurr) > 0;
    }

    /// <summary>Sets Iteration to the next Item.</summary>
    public void Next()
    {
        // OCCT recurses once per unselected item; a loop does the same without the depth
        do
        {
            _thecurr++;
            if (_thecurr > _themaxi)
            {
                return;
            }
        }
        while (_theselect.Value(_thecurr) == 0);
    }

    /// <summary>Returns the current Binder.</summary>
    /// <exception cref="NoSuchObjectException">No current item.</exception>
    public Binder Value()
    {
        if (_thecurr == 0 || _thecurr > _themaxi)
        {
            throw new NoSuchObjectException("TransferIterator : Value");
        }
        if (_theselect.Value(_thecurr) == 0)
        {
            throw new NoSuchObjectException("TransferIterator : Value");
        }
        return _theitems.Value(_thecurr);
    }

    //  ....                Access to Current Binder Data                ....

    /// <summary>
    /// Returns True if current Item brings a Result, Transient (Handle) or not or Multiple. That is to say, if it
    /// corresponds to a normally achieved Transfer, Transient Result is read by specific TransientResult below.
    /// </summary>
    public bool HasResult()
    {
        Binder atr = Value();
        return atr.HasResult;
    }

    /// <summary>Returns True if Current Item has a Unique Result.</summary>
    public bool HasUniqueResult()
    {
        Binder atr = Value();
        if (atr.IsMultiple())
        {
            return false;
        }
        return atr.HasResult;
    }

    /// <summary>
    /// Returns the Type of the Result of the current Item, if Unique. If No Unique Result (Error Transfer or Multiple
    /// Result), returns a Null Handle. The Type is : the Dynamic Type for a Transient Result, the Type defined by the
    /// Binder Class else.
    /// </summary>
    public Type? ResultType()
    {
        Type? btype = null;
        Binder atr = Value();
        if (!atr.IsMultiple())
        {
            btype = atr.ResultType();
        }
        //  Binder's ResultType takes into account the Dynamic Type for Handles
        return btype;
    }

    /// <summary>Returns True if the current Item has a Transient Unique Result (if yes, use TransientResult to get it).</summary>
    public bool HasTransientResult()
    {
        Type? btype = ResultType();
        if (btype is null)
        {
            return false;
        }
        return SubType(btype, typeof(Transient));
    }

    /// <summary>
    /// Returns the Transient Result of the current Item if there is (else, returns a null Handle). Supposes that
    /// Binding is done by a SimpleBinderOfTransient.
    /// </summary>
    public Transient? TransientResult()
    {
        SimpleBinderOfTransient? atr = Value() as SimpleBinderOfTransient;
        if (atr is not null)
        {
            return atr.Result();
        }
        return null;
    }

    /// <summary>Returns Execution Status of current Binder. Normal transfer corresponds to StatusDone.</summary>
    public StatusExec Status()
    {
        Binder atr = Value();
        return atr.StatusExec();
    }

    /// <summary>Returns True if Fail Messages are recorded with the current Binder.</summary>
    public bool HasFails()
    {
        Binder atr = Value();
        return atr.Check().HasFailed;
    }

    /// <summary>Returns True if Warning Messages are recorded with the current Binder.</summary>
    public bool HasWarnings()
    {
        Binder atr = Value();
        return atr.Check().HasWarnings;
    }

    /// <summary>Returns Check associated to current Binder (fails in case of error, warnings in case of warnings).</summary>
    public FrameworkCheck Check()
    {
        Binder atr = Value();
        return atr.Check();
    }

    // Standard_Type::SubType: false for a null type
    internal static bool SubType(Type? type, Type? other)
    {
        return type is not null && other is not null && other.IsAssignableFrom(type);
    }

    // Standard_Transient::IsKind: false for a null type
    internal static bool IsKind(Transient theObject, Type? type)
    {
        return type is not null && theObject.IsKind(type);
    }

    protected int _thecurr;
    private readonly NCollection_HSequence<Binder> _theitems;
    private readonly NCollection_HSequence<int> _theselect;
    private int _themaxi;
}
