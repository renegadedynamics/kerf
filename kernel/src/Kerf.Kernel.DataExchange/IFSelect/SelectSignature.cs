// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectSignature.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SelectSignature.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// A SelectSignature sorts the Entities on a Signature Matching. The signature to match is given at creation
/// time. Also, the required match is given at creation time : exact (IsEqual) or contains (the Type's Name
/// must contain the criterium Text) Remark that no more interpretation is done, it is an alphanumeric
/// signature : for instance, DynamicType is matched as such, super-types are not considered Also, numeric
/// (integer) comparisons are supported : an item can be &lt;val ou &lt;=val or &gt;val or &gt;=val , val
/// being an Integer A SelectSignature may also be created from a SignCounter, which then just gives its
/// LastValue as SignatureValue
/// </summary>
internal class SelectSignature : SelectExtract
{
    //  theexact : -1  YES   0  NO single value  > 0 NO number of values
    //  signmode: 1 take if contained, 2 refuse if contained
    //            3 take if equal,     4 refuse if equal
    //  or numeric test, add : 16 <  24 <=  32 >  40 >=
    private static int Multsign(string signtext, NCollection_Sequence<string> signlist, NCollection_Sequence<int> signmode)
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
    public SelectSignature(Signature? matcher, string signtext, bool exact = true)
    {
        _thematcher = matcher;
        _thesigntext = signtext;
        _theexact = exact ? -1 : 0;
        if (!exact)
        {
            _theexact = Multsign(_thesigntext, _thesignlist, _thesignmode);
        }
    }

    /// <summary>
    /// Creates a SelectSignature with a Counter, more precisely a SelectSignature. Which is used here to just
    /// give a Signature Value (by SignOnly Mode) Matching is the default provided by the class Signature
    /// </summary>
    public SelectSignature(SignCounter? counter, string signtext, bool exact = true)
    {
        _thecounter = counter;
        _thesigntext = signtext;
        _theexact = exact ? -1 : 0;
        if (!exact)
        {
            _theexact = Multsign(_thesigntext, _thesignlist, _thesignmode);
        }
    }

    /// <summary>
    /// Returns the used Signature, then it is possible to access it, modify it as required. Can be null, hence
    /// see Counter
    /// </summary>
    public Signature? Signature()
    {
        return _thematcher;
    }

    /// <summary>
    /// Returns the used SignCounter. Can be used as alternative for Signature
    /// </summary>
    public SignCounter? Counter()
    {
        return _thecounter;
    }

    /// <summary>
    /// Returns True for an Entity (model-&gt;Value(num)) of which the signature matches the text given as
    /// creation time May also work with a Counter from the Graph
    /// </summary>
    public override bool SortInGraph(int rank, Transient? ent, EntityGraph g)
    {
        bool res;
        string txt;
        InterfaceModel model = g.Model();
        if (_theexact <= 0)
        {
            if (_thematcher is not null)
            {
                return _thematcher.Matches(ent, model, _thesigntext, (_theexact < 0));
            }
            txt = _thecounter.ComputedSign(ent, g);
            return global::Kerf.Engine.Exchange.Framework.Selection.Signature.MatchValue(txt, _thesigntext, (_theexact < 0));
        }

        //  otherwise: list
        //  Sequence analysis: if take/don't-take alternation, the last one is right
        //   consequently, if only takes or only don't-takes, it's commutative
        //   THEREFORE recommendation: put the don't-takes at the end

        //  AT START: take = take only. don't-take = take-all-except ...
        //  So if the first is a don't-take, I start by taking everything
        int hmod = _thesignmode.Value(1);
        int jmod = hmod / 8;
        int imod = hmod - (jmod * 8);
        res = (imod == 2 || imod == 4);
        for (int i = 1; i <= _theexact; i++)
        {
            string signtext = _thesignlist.Value(i);
            hmod = _thesignmode.Value(i);
            jmod = hmod / 8;
            imod = hmod - (jmod * 8);
            bool quid;
            if (jmod == 0)
            {
                if (_thematcher is not null)
                {
                    quid = _thematcher.Matches(ent, model, signtext, (imod > 2));
                }
                else
                {
                    quid =
                        global::Kerf.Engine.Exchange.Framework.Selection.Signature.MatchValue(_thecounter.ComputedSign(ent, g), signtext, (imod > 2));
                }
            }
            else
            {
                if (_thematcher is not null)
                {
                    txt = _thematcher.Value(ent, model);
                }
                else
                {
                    txt = _thecounter.ComputedSign(ent, g);
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
    public override bool Sort(int rank, Transient? ent, InterfaceModel? model)
    {
        return true;
    }

    /// <summary>
    /// Returns Text used to Sort Entity on its Signature or SignCounter
    /// </summary>
    public string SignatureText()
    {
        return _thesigntext;
    }

    /// <summary>
    /// Returns True if match must be exact
    /// </summary>
    public bool IsExact
    {
        get
        {
            return (_theexact < 0);
        }
    }

    /// <summary>
    /// Returns a text defining the criterium. (it refers to the text and exact flag to be matched, and is
    /// qualified by the Name provided by the Signature)
    /// </summary>
    public override string ExtractLabel()
    {
        string lab = "";
        if (_thematcher is not null)
        {
            lab += _thematcher.Name();
        }
        else
        {
            lab += _thecounter.Name();
        }
        if (_theexact < 0)
        {
            lab += " matching ";
        }
        else if (_theexact == 0)
        {
            lab += " containing ";
        }
        else
        {
            lab += " matching list ";
        }
        lab += _thesigntext;
        return lab;
    }

    private Signature? _thematcher;
    private SignCounter? _thecounter;
    private string _thesigntext = "";
    private int _theexact;
    private NCollection_Sequence<string> _thesignlist = new NCollection_Sequence<string>();
    private NCollection_Sequence<int> _thesignmode = new NCollection_Sequence<int>();
}
