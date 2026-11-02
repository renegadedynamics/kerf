// Port of: src/DataExchange/TKXSBase/Interface/Interface_ParamSet.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_ParamSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An ordered set of file parameters, efficient in memory and speed: the first <c>nres</c> parameters in a list, the
/// others in next sets.
/// </summary>
/// <remarks>
/// OCCT copies the parameters' characters into a page it manages and grows (re-pointing the parameters already
/// recorded when it reallocates); with .NET strings, a parameter keeps its string and the page is not needed, so
/// <c>lnval</c> only chooses which part of the text is kept.
/// </remarks>
public class Interface_ParamSet : Standard_Transient
{
    /// <summary>
    /// Creates an empty set, of initial reservation <paramref name="nres"/>: the <paramref name="nres"/> first
    /// parameters are put in a list, the remainders in next sets. <paramref name="nst"/> is not used.
    /// </summary>
    public Interface_ParamSet(int nres, int nst = 1)
    {
        _ = nst;
        thelist = new Interface_ParamList(); // (nst,nst+nres+2);
        themxpar = nres;
        thenbpar = 0;
    }

    /// <summary>
    /// Adds a parameter defined by its value (a text and its length) and type, with an optional entity number (for
    /// FileReaderData). If <paramref name="lnval"/> &lt; 0 the text is kept as it is, else its first
    /// <paramref name="lnval"/> characters are copied. Returns the new count of recorded parameters.
    /// </summary>
    public int Append(string val, int lnval, Interface_ParamType typ, int nument)
    {
        //  Here, local String management
        thenbpar++;
        if (thenbpar > themxpar)
        {
            // OCCT: each parameter past the reservation replaces the next set, so only the last one is kept there,
            // and the count returned adds the next set's to this one's
            thenext = new Interface_ParamSet(themxpar, 1);
            return (thenbpar + thenext.Append(val, lnval, typ, nument));
        }
        else if (lnval < 0)
        {
            //    ..  External character management  ..
            Interface_FileParameter FP = thelist.ChangeValue(thenbpar);
            FP.Init(val, typ);
            if (nument != 0)
            {
                FP.SetEntityNumber(nument);
            }
        }
        else
        {
            //    ..  Local character management  ..
            //      Register this parameter (OCCT copies <lnval> characters into its page, then a null)
            string aCopy = lnval < val.Length ? val.Substring(0, lnval) : val;
            Interface_FileParameter FP = thelist.ChangeValue(thenbpar);
            FP.Init(aCopy, typ);
            if (nument != 0)
            {
                FP.SetEntityNumber(nument);
            }
        }
        return thenbpar;
    }

    /// <summary>
    /// Adds a parameter at the end of the set (transparent about reservation and next sets). Returns the new count
    /// of recorded parameters.
    /// </summary>
    public int Append(Interface_FileParameter FP)
    {
        //  Here, FP ready: no memory management on String (too bad)
        thenbpar++;
        if (thenbpar > themxpar)
        {
            thenext = new Interface_ParamSet(themxpar, 1); // OCCT: as above, replaces the next set
            return thenbpar + thenext.Append(FP);
        }
        thelist.SetValue(thenbpar, FP);
        return thenbpar;
    }

    /// <summary>The total count of parameters (including next sets').</summary>
    public int NbParams() => thenbpar;

    /// <summary>A parameter, by its number.</summary>
    public Interface_FileParameter Param(int num)
    {
        if (num > themxpar)
        {
            return thenext!.Param(num - themxpar);
        }
        else
        {
            return thelist.Value(num);
        }
    }

    /// <summary>A parameter, by its number, to be changed in place.</summary>
    public Interface_FileParameter ChangeParam(int num)
    {
        if (num > themxpar)
        {
            return thenext!.ChangeParam(num - themxpar);
        }
        else
        {
            return thelist.ChangeValue(num);
        }
    }

    /// <summary>Changes a parameter, by its number.</summary>
    public void SetParam(int num, Interface_FileParameter FP)
    {
        if (num > themxpar)
        {
            thenext!.SetParam(num - themxpar, FP);
        }
        else
        {
            thelist.SetValue(num, FP);
        }
    }

    /// <summary>
    /// Builds and returns the sub-list of the parameters from <paramref name="num"/> included, with count
    /// <paramref name="nb"/>; the whole list (the set's own, not a copy, when it has no next set) if both are zero.
    /// </summary>
    public Interface_ParamList Params(int num, int nb)
    {
        int i, n0 = num - 1, nbp = nb;
        if (num > themxpar)
        {
            return thenext!.Params(num - themxpar, nb);
        }
        if (num == 0 && nb == 0)
        {
            n0 = 0;
            nbp = thenbpar;
            if (thenbpar <= themxpar)
            {
                return thelist; // and there you go
            }
        }
        Interface_ParamList list = new Interface_ParamList();
        if (nb == 0)
        {
            return list;
        }

        for (i = 1; i <= nbp; i++)
        {
            list.SetValue(i, Param(n0 + i));
        }
        return list;
    }

    /// <summary>Releases the parameters (OCCT's manual destruction).</summary>
    public void Destroy()
    {
        //  if (!thenext.IsNull()) thenext->Destroy();
        thenext = null;
        thelist.Clear();
    }

    private readonly int themxpar;
    private int thenbpar;
    private readonly Interface_ParamList thelist;
    private Interface_ParamSet? thenext;
}
