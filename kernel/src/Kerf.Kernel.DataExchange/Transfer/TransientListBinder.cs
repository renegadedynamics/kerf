// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransientListBinder.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransientListBinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// This binder binds several (a list of) Transients with a starting entity, when this entity itself corresponds to a
/// simple list of Transients. Each part is not seen as a sub-result of an independent component, but as an item of a
/// built-in list.
/// </summary>
internal class TransientListBinder : Binder
{
    public TransientListBinder()
    {
        _theres = new NCollection_HSequence<Transient>();
    }

    public TransientListBinder(NCollection_HSequence<Transient>? list)
    {
        _theres = list;
    }

    public override bool IsMultiple()
    {
        return TransientCount > 1;
    }

    public override Type? ResultType()
    {
        return typeof(TransientListBinder);
    }

    public override string ResultTypeName()
    {
        return "list(Standard_Transient)";
    }

    /// <summary>Adds an item to the result list.</summary>
    public void AddResult(Transient? transient)
    {
        _theres.Append(transient);
    }

    public NCollection_HSequence<Transient>? Result()
    {
        return _theres;
    }

    /// <summary>Changes an already defined sub-result.</summary>
    public void SetResult(int num, Transient? transient)
    {
        _theres.SetValue(num, transient);
    }

    public int TransientCount
    {
        get
        {
            return _theres.Length();
        }
    }

    public Transient? Transient(int num)
    {
        return _theres.Value(num);
    }

    private readonly NCollection_HSequence<Transient>? _theres;
}
