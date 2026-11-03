// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_CheckCounter.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_CheckCounter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// A CheckCounter allows to see a CheckList (i.e. CheckIterator) not per entity, its messages, but per
/// message, the entities attached (count and list). Because many messages can be repeated if they are due to
/// systematic errors
/// </summary>
public class IFSelect_CheckCounter : IFSelect_SignatureList
{

    /// <summary>
    /// Creates a CheckCounter, empty ready to work
    /// </summary>
    public IFSelect_CheckCounter(bool withlist = false) : base(withlist)
    {
        SetName("Check");
    }

    /// <summary>
    /// Sets a specific signature Else, the current SignType (in the model) is used
    /// </summary>
    public void SetSignature(MoniTool_SignText? sign)
    {
        thesign = sign;
    }

    /// <summary>
    /// Returns the Signature;
    /// </summary>
    public MoniTool_SignText? Signature()
    {
        return thesign;
    }

    /// <summary>
    /// Analyses a CheckIterator according a Model (which detains the entities for which the CheckIterator has
    /// messages), i.e. counts messages for entities If &lt;original&gt; is True, does not consider final
    /// messages but those before interpretation (such as inserting variables : integers, reals, strings) If
    /// &lt;failsonly&gt; is True, only Fails are considered Remark : global messages are recorded with a Null
    /// entity
    /// </summary>
    public void Analyse(Interface_CheckIterator list, Interface_InterfaceModel? model, bool original = false, bool failsonly = false)
    {
        int i, nb, num, nbe = (model is null ? 0 : model.NbEntities());
        string mess = Sprintf("Check %s", list.Name());
        SetName(mess);
        for (list.Start(); list.More(); list.Next())
        {
            num = list.Number();
            Transient ent = null;
            Interface_Check check = list.Value();
            ent = check.Entity();
            if (ent is null && num > 0 && num <= nbe)
            {
                ent = model.Value(num);
            }
            nb = check.NbFails();
            string tystr = null;
            if (ent is not null)
            {
                if (thesign is not null)
                {
                    // OCCT keeps the C string of the temporary text Text() returns, destroyed at once: natively the type
                    // printed is garbage or empty (occt-issues.md, 8A3); the port gives the text
                    tystr = thesign.Text(ent, model);
                }
                else if (model is not null)
                {
                    tystr = model.TypeName(ent);
                }
                else
                {
                    tystr = Interface_InterfaceModel.ClassName(ent.DynamicTypeName());
                }
            }
            for (i = 1; i <= nb; i++)
            {
                if (ent is null)
                {
                    mess = Sprintf("F: %s", check.CFail(i, original));
                }
                else
                {
                    mess = Sprintf("F:%s: %s", tystr, check.CFail(i, original));
                }
                Add(ent, mess);
            }
            nb = 0;
            if (!failsonly)
            {
                nb = check.NbWarnings();
            }
            for (i = 1; i <= nb; i++)
            {
                if (ent is null)
                {
                    mess = Sprintf("W: %s", check.CWarning(i, original));
                }
                else
                {
                    mess = Sprintf("W:%s: %s", tystr, check.CWarning(i, original));
                }
                Add(ent, mess);
            }
        }
    }

    private MoniTool_SignText? thesign;
}
