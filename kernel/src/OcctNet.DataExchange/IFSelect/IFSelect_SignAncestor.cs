// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SignAncestor.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SignAncestor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

public class IFSelect_SignAncestor : IFSelect_SignType
{

    public IFSelect_SignAncestor(bool nopk = false) : base(nopk)
    {
    }

    public override bool Matches(Standard_Transient? ent, Interface_InterfaceModel? model, string text, bool exact)
    {
        if (ent is null)
        {
            return false;
        }
        // OCCT first takes the entity as a Standard_Type itself: a System.Type is never an entity here
        Type atype = ent.DynamicType();
        // Standard_Type::SubType(name): the type or one of its ancestors has this name
        for (Type? t = atype; t is not null; t = t.BaseType)
        {
            if (t.Name == text)
            {
                return true;
            }
        }
        return false;
    }
}
