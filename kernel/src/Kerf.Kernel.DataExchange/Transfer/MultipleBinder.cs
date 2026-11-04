// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_MultipleBinder.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_MultipleBinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// Allows direct binding between a starting Object and the Result of its transfer, when it can be made of several
/// Transient Objects. Result is a list of Transient Results.
/// </summary>
/// <remarks>
/// MultipleBinder itself is intended to be created and filled by TransferProcess itself (method BindMultiple), which
/// avoids conflicts between Unique (Standard) result and Multiple result.
/// </remarks>
internal class MultipleBinder : Binder
{
    /// <summary>Normal standard constructor, creates an empty MultipleBinder.</summary>
    public MultipleBinder()
    {
    }

    /// <summary>Returns True if a starting object is bound with SEVERAL results.</summary>
    public override bool IsMultiple()
    {
        if (_themulres is null)
        {
            return false;
        }
        return _themulres.Length() != 1;
    }

    /// <summary>Returns the Type permitted for Results, i.e. here Transient.</summary>
    public override Type? ResultType()
    {
        return typeof(Transient);
    }

    /// <summary>Returns "(list)".</summary>
    public override string ResultTypeName()
    {
        return "(list)";
    }

    //  ....        Multiple Result Management        ....

    /// <summary>Adds a new Item to the Multiple Result.</summary>
    public void AddResult(Transient? res)
    {
        if (_themulres is null)
        {
            _themulres = new NCollection_HSequence<Transient>();
        }
        _themulres.Append(res);
    }

    /// <summary>Returns the actual count of recorded (Transient) results.</summary>
    public int ResultCount
    {
        get
        {
            return _themulres is null ? 0 : _themulres.Length();
        }
    }

    /// <summary>Returns the value of the recorded result n0 <paramref name="num"/>.</summary>
    public Transient? ResultValue(int num)
    {
        return _themulres.Value(num);
    }

    /// <summary>Returns the Multiple Result, if it is defined (at least one Item). Else, returns an empty sequence.</summary>
    public NCollection_HSequence<Transient> MultipleResult()
    {
        if (_themulres is not null)
        {
            return _themulres;
        }
        return new NCollection_HSequence<Transient>();
    }

    /// <summary>Defines a Binding with a Multiple Result, given as a Sequence.</summary>
    public void SetMultipleResult(NCollection_HSequence<Transient>? mulres)
    {
        _themulres = mulres;
    }

    private NCollection_HSequence<Transient>? _themulres;
}
