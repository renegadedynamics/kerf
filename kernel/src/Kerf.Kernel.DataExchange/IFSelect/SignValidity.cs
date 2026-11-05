// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SignValidity.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SignValidity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// This Signature returns the Validity Status of an entity, as deducted from data in the model : it can be
/// "OK" "Unknown" "Unloaded" "Syntactic Fail"(but loaded) "Syntactic Warning" "Semantic Fail" "Semantic
/// Warning"
/// </summary>
internal class SignValidity : Signature
{
    private const string Nulsign = "";

    /// <summary>
    /// Returns a SignValidity
    /// </summary>
    public SignValidity() : base("Validity")
    {
        AddCase("UNKNOWN");
        AddCase("UNLOADED");
        AddCase("Load-Error");
        AddCase("Data-Error");
        AddCase("Load-Warning");
        AddCase("Data-Warning");
        AddCase("OK");
    }

    /// <summary>
    /// Returns the Signature for a Transient object, as a validity deducted from data (reports) stored in the
    /// model. Class method, can be called by any one
    /// </summary>
    public static string CVal(Transient? ent, InterfaceModel? model)
    {
        if (ent is null || model is null)
        {
            return Nulsign;
        }
        int num = model.Number(ent);
        int cas = 0;
        if (model.IsUnknownEntity(num))
        {
            return "UNKNOWN";
        }
        if (model.IsRedefinedContent(num))
        {
            return "UNLOADED";
        }

        FrameworkCheck ch1 = model.Check(num, true);
        FrameworkCheck ch2 = model.Check(num, false);
        if (ch1.FailCount > 0)
        {
            return "Load-Error";
        }
        else if (ch1.WarningCount > 0)
        {
            cas = 1;
        }

        if (ch2.FailCount > 0)
        {
            return "Data-Error";
        }
        else if (cas == 1)
        {
            return "Load-Warning";
        }
        else if (ch2.WarningCount > 0)
        {
            return "Data-Warning";
        }

        return "OK";
    }

    /// <summary>
    /// Returns the Signature for a Transient object, as a validity deducted from data (reports) stored in the
    /// model Calls the class method CVal
    /// </summary>
    public override string Value(Transient? ent, InterfaceModel? model)
    {
        return SignValidity.CVal(ent, model);
    }
}
