// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Composite.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Composite.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Engine.Construction.Laws;

/// <summary>
/// A composite law: a list of laws on consecutive ranges, evaluated through the one whose range holds the
/// parameter (written for fillets' spines in OCCT).
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="ChangeLaws"/>, <see cref="ChangeElementaryLaw"/> and
/// <see cref="SetPeriodic"/> are public, and evaluation remembers the current law. The list keeps the caller's
/// laws, and <see cref="Trim"/> shares them, as OCCT shares the handles. <see cref="Bounds"/> gives the range of
/// the laws only after a first evaluation (before, OCCT's -1e100 and 1e100).
/// </remarks>
internal class Composite : LawsFunction
{
    private double _first;
    private double _last;
    private LawsFunction? _curfunc;
    private readonly NCollection_List<LawsFunction> _funclist = new NCollection_List<LawsFunction>();
    private bool _periodic;
    private readonly double _tFirst;
    private readonly double _tLast;
    private readonly double _pTol;

    /// <summary>An empty law.</summary>
    public Composite()
    {
        _first = -1.0e100;
        _last = 1.0e100;
        _periodic = false;
        _tFirst = -1.0e100;
        _tLast = 1.0e100;
        _pTol = 0.0;
    }

    /// <summary>An empty law trimmed to [First, Last]; Tol decides which law evaluates at a junction.</summary>
    public Composite(double first, double last, double tol)
    {
        _first = -1.0e100;
        _last = 1.0e100;
        _periodic = false;
        _tFirst = first;
        _tLast = last;
        _pTol = tol;
    }

    /// <summary>Not implemented in OCCT.</summary>
    /// <exception cref="MethodNotImplementedException">Always.</exception>
    public override Continuity Continuity()
    {
        throw new MethodNotImplementedException("Law_Composite::Continuity()");
    }

    /// <summary>The sum of the laws' intervals.</summary>
    public override int NbIntervals(Continuity s)
    {
        NCollection_List<LawsFunction>.Iterator it = new NCollection_List<LawsFunction>.Iterator(_funclist);
        LawsFunction func;
        int nbr_interval = 0;

        for (; it.More(); it.Next())
        {
            func = it.Value();
            nbr_interval += func.NbIntervals(s);
        }

        return nbr_interval;
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        NCollection_List<LawsFunction>.Iterator it = new NCollection_List<LawsFunction>.Iterator(_funclist);
        LawsFunction func;
        NCollection_HArray1<double> locT;
        int nb_index, iloc, iGlob = 2;

        func = _funclist.First();
        func.Bounds(out t[1], out t[2]);

        for (; it.More(); it.Next())
        {
            func = it.Value();
            nb_index = func.NbIntervals(s) + 1;
            locT = new NCollection_HArray1<double>(1, nb_index);
            func.Intervals(locT.ChangeArray1(), s);
            for (iloc = 2; iloc <= nb_index; iloc++, iGlob++)
            {
                t[iGlob] = locT.Value(iloc);
            }
        }
    }

    public override double Value(double x)
    {
        double w = x;
        Prepare(ref w);
        return _curfunc!.Value(w);
    }

    public override void D1(double x, out double f, out double d)
    {
        double w = x;
        Prepare(ref w);
        _curfunc!.D1(w, out f, out d);
    }

    public override void D2(double x, out double f, out double d, out double d2)
    {
        double w = x;
        Prepare(ref w);
        _curfunc!.D2(w, out f, out d, out d2);
    }

    /// <summary>A composite of the same laws trimmed to [PFirst, PLast].</summary>
    public override LawsFunction Trim(double pFirst, double pLast, double tol)
    {
        Composite l = new Composite(pFirst, pLast, tol);
        l.ChangeLaws().Assign(_funclist);
        return l;
    }

    public override void Bounds(out double pFirst, out double pLast)
    {
        pFirst = _first;
        pLast = _last;
    }

    // Lorsque le parametre est pres d'un "noeud" on determine la loi en
    // fonction du signe de tol:
    //   - negatif -> Loi precedente au noeud.
    //   - positif -> Loi consecutive au noeud.
    private void Prepare(ref double w)
    {
        double f, l, wtest, eps;
        if (w - _tFirst < _tLast - w)
        {
            eps = _pTol;
        }
        else
        {
            eps = -_pTol;
        }

        if (_curfunc is null)
        {
            _curfunc = _funclist.Last();
            _curfunc.Bounds(out f, out _last);
            _curfunc = _funclist.First();
            _curfunc.Bounds(out _first, out l);
        }

        wtest = w + eps; // Decalage pour discriminer les noeuds
        if (_periodic)
        {
            wtest = global::Kerf.Engine.Geometry.ElCLib.InPeriod(wtest, _first, _last);
            w = wtest - eps;
        }

        _curfunc.Bounds(out f, out l);
        if (f <= wtest && wtest <= l)
        {
            return;
        }
        if (w <= _first)
        {
            _curfunc = _funclist.First();
        }
        else if (w >= _last)
        {
            _curfunc = _funclist.Last();
        }
        else
        {
            NCollection_List<LawsFunction>.Iterator it = new NCollection_List<LawsFunction>.Iterator(_funclist);
            for (; it.More(); it.Next())
            {
                _curfunc = it.Value();
                _curfunc.Bounds(out f, out l);
                if (f <= wtest && wtest <= l)
                {
                    return;
                }
            }
        }
    }

    /// <summary>A reference to the elementary law that evaluates at W.</summary>
    public ref LawsFunction? ChangeElementaryLaw(double w)
    {
        double ww = w;
        Prepare(ref ww);
        return ref _curfunc;
    }

    /// <summary>The list of laws itself, to fill or change.</summary>
    public NCollection_List<LawsFunction> ChangeLaws() => _funclist;

    public bool IsPeriodic => _periodic;

    public void SetPeriodic()
    {
        _periodic = true;
    }
}
