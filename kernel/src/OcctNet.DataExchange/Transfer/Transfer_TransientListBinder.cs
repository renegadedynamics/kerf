// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransientListBinder.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_TransientListBinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This binder binds several (a list of) Transients with a starting entity, when this entity itself corresponds to a
/// simple list of Transients. Each part is not seen as a sub-result of an independent component, but as an item of a
/// built-in list.
/// </summary>
public class Transfer_TransientListBinder : Transfer_Binder
{
    public Transfer_TransientListBinder()
    {
        theres = new NCollection_HSequence<Standard_Transient>();
    }

    public Transfer_TransientListBinder(NCollection_HSequence<Standard_Transient>? list)
    {
        theres = list;
    }

    public override bool IsMultiple()
    {
        return NbTransients() > 1;
    }

    public override Type? ResultType()
    {
        return typeof(Transfer_TransientListBinder);
    }

    public override string ResultTypeName()
    {
        return "list(Standard_Transient)";
    }

    /// <summary>Adds an item to the result list.</summary>
    public void AddResult(Standard_Transient? Transient)
    {
        theres.Append(Transient);
    }

    public NCollection_HSequence<Standard_Transient>? Result()
    {
        return theres;
    }

    /// <summary>Changes an already defined sub-result.</summary>
    public void SetResult(int num, Standard_Transient? Transient)
    {
        theres.SetValue(num, Transient);
    }

    public int NbTransients()
    {
        return theres.Length();
    }

    public Standard_Transient? Transient(int num)
    {
        return theres.Value(num);
    }

    private readonly NCollection_HSequence<Standard_Transient>? theres;
}
