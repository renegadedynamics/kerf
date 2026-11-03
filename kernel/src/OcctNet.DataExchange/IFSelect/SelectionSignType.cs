// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SignType.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SignType.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// This Signature returns the cdl Type of an entity, under two forms : - complete dynamic type (package and
/// class) - class type, without package name
/// </summary>
public class SelectionSignType : Signature
{
    private const string Nulsign = "";

    /// <summary>
    /// Returns a SignType &lt;nopk&gt; false (D) : complete dynamic type (name = Dynamic Type) &lt;nopk&gt; true
    /// : class type without pk (name = Class Type)
    /// </summary>
    public SelectionSignType(bool nopk = false) : base(nopk ? "Class Type" : "Dynamic Type")
    {
        _thenopk = nopk;
    }

    /// <summary>
    /// Returns the Signature for a Transient object, as its Dynamic Type, with or without package name,
    /// according starting option
    /// </summary>
    public override string Value(Transient? ent, InterfaceModel? model)
    {
        if (ent is null)
        {
            return Nulsign;
        }
        // OCCT first takes the entity as a Standard_Type itself: a System.Type is never an entity here
        Type atype = ent.DynamicType();
        string tn = global::Kerf.Engine.Foundation.OcctTypeNames.Of(atype);
        if (!_thenopk)
        {
            return tn;
        }
        for (int i = 0; i < tn.Length; i++)
        {
            if (tn[i] == '_')
            {
                return tn.Substring(i + 1);
            }
        }
        return tn;
    }

    private bool _thenopk;
}
