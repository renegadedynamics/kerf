// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectSignature.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectSignature.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// A SelectSignature sorts the Entities on a Signature Matching. The signature to match is given at creation
/// time. Also, the required match is given at creation time : exact (IsEqual) or contains (the Type's Name
/// must contain the criterium Text) Remark that no more interpretation is done, it is an alphanumeric
/// signature : for instance, DynamicType is matched as such, super-types are not considered Also, numeric
/// (integer) comparisons are supported : an item can be &lt;val ou &lt;=val or &gt;val or &gt;=val , val
/// being an Integer A SelectSignature may also be created from a SignCounter, which then just gives its
/// LastValue as SignatureValue
/// </summary>
public class IFSelect_SelectSignature : IFSelect_SelectExtract
{
    //  theexact : -1  YES   0  NO single value  > 0 NO number of values
    //  signmode: 1 take if contained, 2 refuse if contained
    //            3 take if equal,     4 refuse if equal
    //  or numeric test, add : 16 <  24 <=  32 >  40 >=
    private static int multsign(string signtext, NCollection_Sequence<string> signlist, NCollection_Sequence<int> signmode)
    {
        int i, nb = signtext.Length, mode = 0;
        for (i = 1; i <= nb; i++)
        {
            char unsign = signtext[i - 1];
            if (unsign == '|' || unsign == '!' || unsign == '=')
            {
                mode = 1;
                break;
            }
            if (unsign == '<' || unsign == '>')
            {
                mode = 1;
                break;
            }
        }
        if (mode == 0)
        {
            return mode;
        }
        //  On va tronconner
        string item = "";
        int imod = 1;
        for (i = 1; i <= nb; i++)
        {
            char unsign = signtext[i - 1];
            if (unsign == '|' || unsign == '!')
            {
                if (item.Length > 0)
                {
                    signlist.Append(item);
                    signmode.Append(imod);
                    item = "";
                    mode++;
                }
                imod = (unsign == '|' ? 1 : 2);
            }
            else if (unsign == '<')
            {
                imod += 16;
            }
            else if (unsign == '>')
            {
                imod += 32;
            }
            else if (unsign == '=')
            {
                if (imod < 8)
                {
                    imod += 2;
                }
                else
                {
                    imod += 8;
                }
            }
            else
            {
                item += unsign;
            }
        }
        if (item.Length > 0)
        {
            signlist.Append(item);
            signmode.Append(imod);
            //    mode ++;  was one at start
        }
        return mode;
    }

    /// <summary>
    /// Creates a SelectSignature with its Signature and its Text to Match. &lt;exact&gt; if True requires exact
    /// match, if False requires &lt;signtext&gt; to be contained in the Signature of the entity (default is
    /// "exact")
    /// </summary>
    public IFSelect_SelectSignature(IFSelect_Signature? matcher, string signtext, bool exact = true)
    {
        thematcher = matcher;
        thesigntext = signtext;
        theexact = exact ? -1 : 0;
        if (!exact)
        {
            theexact = multsign(thesigntext, thesignlist, thesignmode);
        }
    }

    /// <summary>
    /// Creates a SelectSignature with a Counter, more precisely a SelectSignature. Which is used here to just
    /// give a Signature Value (by SignOnly Mode) Matching is the default provided by the class Signature
    /// </summary>
    public IFSelect_SelectSignature(IFSelect_SignCounter? counter, string signtext, bool exact = true)
    {
        thecounter = counter;
        thesigntext = signtext;
        theexact = exact ? -1 : 0;
        if (!exact)
        {
            theexact = multsign(thesigntext, thesignlist, thesignmode);
        }
    }

    /// <summary>
    /// Returns the used Signature, then it is possible to access it, modify it as required. Can be null, hence
    /// see Counter
    /// </summary>
    public IFSelect_Signature? Signature()
    {
        return thematcher;
    }

    /// <summary>
    /// Returns the used SignCounter. Can be used as alternative for Signature
    /// </summary>
    public IFSelect_SignCounter? Counter()
    {
        return thecounter;
    }

    /// <summary>
    /// Returns True for an Entity (model-&gt;Value(num)) of which the signature matches the text given as
    /// creation time May also work with a Counter from the Graph
    /// </summary>
    public override bool SortInGraph(int rank, Transient? ent, Interface_Graph G)
    {
        bool res;
        string txt;
        Interface_InterfaceModel model = G.Model();
        if (theexact <= 0)
        {
            if (thematcher is not null)
            {
                return thematcher.Matches(ent, model, thesigntext, (theexact < 0));
            }
            txt = thecounter.ComputedSign(ent, G);
            return IFSelect_Signature.MatchValue(txt, thesigntext, (theexact < 0));
        }

        //  otherwise: list
        //  Sequence analysis: if take/don't-take alternation, the last one is right
        //   consequently, if only takes or only don't-takes, it's commutative
        //   THEREFORE recommendation: put the don't-takes at the end

        //  AT START: take = take only. don't-take = take-all-except ...
        //  So if the first is a don't-take, I start by taking everything
        int hmod = thesignmode.Value(1);
        int jmod = hmod / 8;
        int imod = hmod - (jmod * 8);
        res = (imod == 2 || imod == 4);
        for (int i = 1; i <= theexact; i++)
        {
            string signtext = thesignlist.Value(i);
            hmod = thesignmode.Value(i);
            jmod = hmod / 8;
            imod = hmod - (jmod * 8);
            bool quid;
            if (jmod == 0)
            {
                if (thematcher is not null)
                {
                    quid = thematcher.Matches(ent, model, signtext, (imod > 2));
                }
                else
                {
                    quid =
                        IFSelect_Signature.MatchValue(thecounter.ComputedSign(ent, G), signtext, (imod > 2));
                }
            }
            else
            {
                if (thematcher is not null)
                {
                    txt = thematcher.Value(ent, model);
                }
                else
                {
                    txt = thecounter.ComputedSign(ent, G);
                }

                int val = Atoi(txt);
                int lav = Atoi(signtext);
                switch (jmod)
                {
                    case 2:
                        quid = (val < lav);
                        break;
                    case 3:
                        quid = (val <= lav);
                        break;
                    case 4:
                        quid = (val > lav);
                        break;
                    case 5:
                        quid = (val >= lav);
                        break;
                    default:
                        quid = false;
                        break;
                }
            }
            if ((imod == 1 || imod == 3) && quid)
            {
                res = true;
            }
            if ((imod == 2 || imod == 4) && quid)
            {
                res = false;
            }
        }
        return res;
    }

    /// <summary>
    /// Not called, defined only to remove a deferred method here
    /// </summary>
    public override bool Sort(int rank, Transient? ent, Interface_InterfaceModel? model)
    {
        return true;
    }

    /// <summary>
    /// Returns Text used to Sort Entity on its Signature or SignCounter
    /// </summary>
    public string SignatureText()
    {
        return thesigntext;
    }

    /// <summary>
    /// Returns True if match must be exact
    /// </summary>
    public bool IsExact()
    {
        return (theexact < 0);
    }

    /// <summary>
    /// Returns a text defining the criterium. (it refers to the text and exact flag to be matched, and is
    /// qualified by the Name provided by the Signature)
    /// </summary>
    public override string ExtractLabel()
    {
        string lab = "";
        if (thematcher is not null)
        {
            lab += thematcher.Name();
        }
        else
        {
            lab += thecounter.Name();
        }
        if (theexact < 0)
        {
            lab += " matching ";
        }
        else if (theexact == 0)
        {
            lab += " containing ";
        }
        else
        {
            lab += " matching list ";
        }
        lab += thesigntext;
        return lab;
    }

    private IFSelect_Signature? thematcher;
    private IFSelect_SignCounter? thecounter;
    private string thesigntext = "";
    private int theexact;
    private NCollection_Sequence<string> thesignlist = new NCollection_Sequence<string>();
    private NCollection_Sequence<int> thesignmode = new NCollection_Sequence<int>();
}
