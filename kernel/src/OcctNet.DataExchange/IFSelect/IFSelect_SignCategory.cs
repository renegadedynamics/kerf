// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SignCategory.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SignCategory.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// This Signature returns the Category of an entity, as recorded in the model
/// </summary>
public class IFSelect_SignCategory : IFSelect_Signature
{
    private const string nulsign = "";

    /// <summary>
    /// Returns a SignCategory
    /// </summary>
    public IFSelect_SignCategory() : base("Category")
    {
        Interface_Category.Init(); // if not already done
        int i, nb = Interface_Category.NbCategories();
        for (i = 1; i <= nb; i++)
        {
            AddCase(Interface_Category.Name(i));
        }
    }

    /// <summary>
    /// Returns the Signature for a Transient object, as its Category recorded in the model
    /// </summary>
    public override string Value(Transient? ent, Interface_InterfaceModel? model)
    {
        if (ent is null || model is null)
        {
            return nulsign;
        }
        return Interface_Category.Name(model.CategoryNumber(model.Number(ent)));
    }
}
