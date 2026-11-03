// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransferIterator.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransferIterator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// Defines an Iterator on the result of a Transfer. Available for Normal Results or not (Erroneous Transfer). It gives
/// several kinds of Information, and allows to consider various criteria (criteria are cumulative).
/// </summary>
/// <remarks>A value class in C++ (decision 1185): a C++ copy is written <see cref="Transfer_TransferIterator(Transfer_TransferIterator)"/>.</remarks>
public class Transfer_TransferIterator
{
    /// <summary>Creates an empty Iterator.</summary>
    public Transfer_TransferIterator()
    {
        theitems = new NCollection_HSequence<Transfer_Binder>();
        theselect = new NCollection_HSequence<int>();
        themaxi = 0;
        thecurr = 1;
    }

    /// <summary>A copy (C++'s copy constructor, which shares the handles of the lists).</summary>
    public Transfer_TransferIterator(Transfer_TransferIterator theOther)
    {
        theitems = theOther.theitems;
        theselect = theOther.theselect;
        themaxi = theOther.themaxi;
        thecurr = theOther.thecurr;
    }

    /// <summary>Adds a Binder to the iteration list (construction).</summary>
    public void AddItem(Transfer_Binder? atr)
    {
        theitems.Append(atr);
        theselect.Append(1);
        themaxi = theselect.Length();
    }

    /// <summary>
    /// Selects Items on the Type of Binder : keep only Binders which are of a given Type (if keep is True) or reject
    /// only them (if keep is False).
    /// </summary>
    public void SelectBinder(Type? atype, bool keep)
    {
        for (int i = theitems.Length(); i > 0; i--)
        {
            if (IsKind(theitems.Value(i), atype) != keep)
            {
                theselect.SetValue(i, 0);
                if (themaxi == i)
                {
                    themaxi = i - 1;
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

        for (int i = theitems.Length(); i > 0; i--)
        {
            Transfer_Binder atr = theitems.Value(i);
            Type? btype = ResultType(); // OCCT: the current item's result type, not atr's
            bool matchtype;
            if (!atr.HasResult())
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
                theselect.SetValue(i, 0);
                if (themaxi == i)
                {
                    themaxi = i - 1;
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
        for (int i = theitems.Length(); i > 0; i--)
        {
            Transfer_Binder atr = theitems.Value(i);
            if (atr.IsMultiple() == keep)
            {
                theselect.SetValue(i, 0);
                if (themaxi == i)
                {
                    themaxi = i - 1;
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
        if (num < 1 || num > theselect.Length())
        {
            return;
        }
        if (keep)
        {
            theselect.SetValue(num, 1);
        }
        else
        {
            theselect.SetValue(num, 0);
        }
    }

    //  ....                Iteration-Interrogations                ....

    /// <summary>Returns count of Binders to be iterated.</summary>
    public int Number()
    {
        int numb, i;
        numb = 0;
        for (i = 1; i <= themaxi; i++)
        {
            if (theselect.Value(i) != 0)
            {
                numb++;
            }
        }
        return numb;
    }

    /// <summary>Clears Iteration in progress, to allow it to be restarted.</summary>
    public void Start()
    {
        thecurr = 0;
        Next();
    }

    /// <summary>Returns True if there are other Items to iterate.</summary>
    public bool More()
    {
        if (thecurr > themaxi)
        {
            return false;
        }
        if (theselect.Value(thecurr) == 0)
        {
            Next();
        }
        if (thecurr > themaxi)
        {
            return false;
        }
        return theselect.Value(thecurr) > 0;
    }

    /// <summary>Sets Iteration to the next Item.</summary>
    public void Next()
    {
        // OCCT recurses once per unselected item; a loop does the same without the depth
        do
        {
            thecurr++;
            if (thecurr > themaxi)
            {
                return;
            }
        }
        while (theselect.Value(thecurr) == 0);
    }

    /// <summary>Returns the current Binder.</summary>
    /// <exception cref="NoSuchObjectException">No current item.</exception>
    public Transfer_Binder Value()
    {
        if (thecurr == 0 || thecurr > themaxi)
        {
            throw new NoSuchObjectException("TransferIterator : Value");
        }
        if (theselect.Value(thecurr) == 0)
        {
            throw new NoSuchObjectException("TransferIterator : Value");
        }
        return theitems.Value(thecurr);
    }

    //  ....                Access to Current Binder Data                ....

    /// <summary>
    /// Returns True if current Item brings a Result, Transient (Handle) or not or Multiple. That is to say, if it
    /// corresponds to a normally achieved Transfer, Transient Result is read by specific TransientResult below.
    /// </summary>
    public bool HasResult()
    {
        Transfer_Binder atr = Value();
        return atr.HasResult();
    }

    /// <summary>Returns True if Current Item has a Unique Result.</summary>
    public bool HasUniqueResult()
    {
        Transfer_Binder atr = Value();
        if (atr.IsMultiple())
        {
            return false;
        }
        return atr.HasResult();
    }

    /// <summary>
    /// Returns the Type of the Result of the current Item, if Unique. If No Unique Result (Error Transfer or Multiple
    /// Result), returns a Null Handle. The Type is : the Dynamic Type for a Transient Result, the Type defined by the
    /// Binder Class else.
    /// </summary>
    public Type? ResultType()
    {
        Type? btype = null;
        Transfer_Binder atr = Value();
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
        Transfer_SimpleBinderOfTransient? atr = Value() as Transfer_SimpleBinderOfTransient;
        if (atr is not null)
        {
            return atr.Result();
        }
        return null;
    }

    /// <summary>Returns Execution Status of current Binder. Normal transfer corresponds to StatusDone.</summary>
    public Transfer_StatusExec Status()
    {
        Transfer_Binder atr = Value();
        return atr.StatusExec();
    }

    /// <summary>Returns True if Fail Messages are recorded with the current Binder.</summary>
    public bool HasFails()
    {
        Transfer_Binder atr = Value();
        return atr.Check().HasFailed();
    }

    /// <summary>Returns True if Warning Messages are recorded with the current Binder.</summary>
    public bool HasWarnings()
    {
        Transfer_Binder atr = Value();
        return atr.Check().HasWarnings();
    }

    /// <summary>Returns Check associated to current Binder (fails in case of error, warnings in case of warnings).</summary>
    public Interface_Check Check()
    {
        Transfer_Binder atr = Value();
        return atr.Check();
    }

    // Standard_Type::SubType: false for a null type
    internal static bool SubType(Type? theType, Type? theOther)
    {
        return theType is not null && theOther is not null && theOther.IsAssignableFrom(theType);
    }

    // Standard_Transient::IsKind: false for a null type
    internal static bool IsKind(Transient theObject, Type? theType)
    {
        return theType is not null && theObject.IsKind(theType);
    }

    protected int thecurr;
    private readonly NCollection_HSequence<Transfer_Binder> theitems;
    private readonly NCollection_HSequence<int> theselect;
    private int themaxi;
}
