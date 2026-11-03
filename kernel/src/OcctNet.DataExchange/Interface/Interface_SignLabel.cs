// Port of: src/DataExchange/TKXSBase/Interface/Interface_SignLabel.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_SignLabel.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// Signature to give the Label from the Model
/// </summary>
public class Interface_SignLabel : MoniTool_SignText
{

    public Interface_SignLabel()
    {
    }

    /// <summary>
    /// Returns "Entity Label"
    /// </summary>
    public override string Name()
    {
        return "Entity Label";
    }

    /// <summary>
    /// Considers context as an InterfaceModel and returns the Label computed by it
    /// </summary>
    public override string Text(Transient? ent, Transient? context)
    {
        string atext = "";
        Interface_InterfaceModel model = (context as Interface_InterfaceModel);
        if (ent is null || model is null)
        {
            return atext;
        }
        TCollection_HAsciiString lab = model.StringLabel(ent);
        if (lab is not null)
        {
            atext = lab.String();
        }
        return atext;
    }
}
