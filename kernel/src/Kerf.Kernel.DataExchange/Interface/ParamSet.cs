// Port of: src/DataExchange/TKXSBase/Interface/Interface_ParamSet.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_ParamSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// An ordered set of file parameters, efficient in memory and speed: the first <c>nres</c> parameters in a list, the
/// others in next sets.
/// </summary>
/// <remarks>
/// OCCT copies the parameters' characters into a page it manages and grows (re-pointing the parameters already
/// recorded when it reallocates); with .NET strings, a parameter keeps its string and the page is not needed, so
/// <c>lnval</c> only chooses which part of the text is kept.
/// </remarks>
internal class ParamSet : Transient
{
    /// <summary>
    /// Creates an empty set, of initial reservation <paramref name="nres"/>: the <paramref name="nres"/> first
    /// parameters are put in a list, the remainders in next sets. <paramref name="nst"/> is not used.
    /// </summary>
    public ParamSet(int nres, int nst = 1)
    {
        _ = nst;
        _thelist = new ParamList(); // (nst,nst+nres+2);
        _themxpar = nres;
        _thenbpar = 0;
    }

    /// <summary>
    /// Adds a parameter defined by its value (a text and its length) and type, with an optional entity number (for
    /// FileReaderData). If <paramref name="lnval"/> &lt; 0 the text is kept as it is, else its first
    /// <paramref name="lnval"/> characters are copied. Returns the new count of recorded parameters.
    /// </summary>
    public int Append(string val, int lnval, ParamType typ, int nument)
    {
        //  Here, local String management
        _thenbpar++;
        if (_thenbpar > _themxpar)
        {
            // OCCT: each parameter past the reservation replaces the next set, so only the last one is kept there,
            // and the count returned adds the next set's to this one's
            _thenext = new ParamSet(_themxpar, 1);
            return (_thenbpar + _thenext.Append(val, lnval, typ, nument));
        }
        else if (lnval < 0)
        {
            //    ..  External character management  ..
            FileParameter fp = _thelist.ChangeValue(_thenbpar);
            fp.Init(val, typ);
            if (nument != 0)
            {
                fp.SetEntityNumber(nument);
            }
        }
        else
        {
            //    ..  Local character management  ..
            //      Register this parameter (OCCT copies <lnval> characters into its page, then a null)
            string copy = lnval < val.Length ? val.Substring(0, lnval) : val;
            FileParameter fp = _thelist.ChangeValue(_thenbpar);
            fp.Init(copy, typ);
            if (nument != 0)
            {
                fp.SetEntityNumber(nument);
            }
        }
        return _thenbpar;
    }

    /// <summary>
    /// Adds a parameter at the end of the set (transparent about reservation and next sets). Returns the new count
    /// of recorded parameters.
    /// </summary>
    public int Append(FileParameter fp)
    {
        //  Here, FP ready: no memory management on String (too bad)
        _thenbpar++;
        if (_thenbpar > _themxpar)
        {
            _thenext = new ParamSet(_themxpar, 1); // OCCT: as above, replaces the next set
            return _thenbpar + _thenext.Append(fp);
        }
        _thelist.SetValue(_thenbpar, fp);
        return _thenbpar;
    }

    /// <summary>The total count of parameters (including next sets').</summary>
    public int ParamCount => _thenbpar;

    /// <summary>A parameter, by its number.</summary>
    public FileParameter Param(int num)
    {
        if (num > _themxpar)
        {
            return _thenext!.Param(num - _themxpar);
        }
        else
        {
            return _thelist.Value(num);
        }
    }

    /// <summary>A parameter, by its number, to be changed in place.</summary>
    public FileParameter ChangeParam(int num)
    {
        if (num > _themxpar)
        {
            return _thenext!.ChangeParam(num - _themxpar);
        }
        else
        {
            return _thelist.ChangeValue(num);
        }
    }

    /// <summary>Changes a parameter, by its number.</summary>
    public void SetParam(int num, FileParameter fp)
    {
        if (num > _themxpar)
        {
            _thenext!.SetParam(num - _themxpar, fp);
        }
        else
        {
            _thelist.SetValue(num, fp);
        }
    }

    /// <summary>
    /// Builds and returns the sub-list of the parameters from <paramref name="num"/> included, with count
    /// <paramref name="nb"/>; the whole list (the set's own, not a copy, when it has no next set) if both are zero.
    /// </summary>
    public ParamList Params(int num, int nb)
    {
        int i, n0 = num - 1, nbp = nb;
        if (num > _themxpar)
        {
            return _thenext!.Params(num - _themxpar, nb);
        }
        if (num == 0 && nb == 0)
        {
            n0 = 0;
            nbp = _thenbpar;
            if (_thenbpar <= _themxpar)
            {
                return _thelist; // and there you go
            }
        }
        ParamList list = new ParamList();
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
        _thenext = null;
        _thelist.Clear();
    }

    private readonly int _themxpar;
    private int _thenbpar;
    private readonly ParamList _thelist;
    private ParamSet? _thenext;
}
