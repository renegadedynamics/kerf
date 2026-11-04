// Port of: src/FoundationClasses/TKMath/math/math_FunctionAllRoots.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionAllRoots.lxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionAllRoots.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>
/// Uses a sample of a function to find all the intervals on which it is null, then
/// <see cref="FunctionRoots"/> to find the points where it is null outside them. The derivative is required.
/// </summary>
internal class FunctionAllRoots
{
    private bool _done;
    private readonly NCollection_Sequence<double> _pdeb = new();
    private readonly NCollection_Sequence<double> _pfin = new();
    private readonly NCollection_Sequence<double> _piso = new();
    private readonly NCollection_Sequence<int> _ideb = new();
    private readonly NCollection_Sequence<int> _ifin = new();
    private readonly NCollection_Sequence<int> _iiso = new();

    /// <summary>
    /// An interval is found where, for at least two consecutive sample points Ui and Ui+1, |F(Ui)| &lt;= EpsNul and
    /// |F(Ui+1)| &lt;= EpsNul; its real bounds, and the roots between intervals, are computed with
    /// <see cref="FunctionRoots"/>. Throws <see cref="NumericErrorException"/> if that fails.
    /// </summary>
    public FunctionAllRoots(FunctionWithDerivative f, FunctionSample s, double epsX, double epsF, double epsNul)
    {
        _done = false;

        bool nul, pNul, interNul, nuld, nulf;
        double debNul = 0.0, finNul = 0.0;
        int indd = 0, indf = 0;
        double cst, val, valsav = 0, valbid;
        bool fini;
        int nbp, i;

        nbp = s.PointCount;
        f.Value(s.GetParameter(1), out val);
        pNul = Math.Abs(val) <= epsNul;
        if (!pNul)
        {
            valsav = val;
        }

        interNul = false;
        nuld = false;
        nulf = false;

        i = 2;
        fini = (i > nbp);

        while (!fini)
        {
            f.Value(s.GetParameter(i), out val);
            nul = Math.Abs(val) <= epsNul;
            if (!nul)
            {
                valsav = val;
            }

            if (interNul && (!nul))
            {
                interNul = false;
                _pdeb.Append(debNul);
                _ideb.Append(indd);
                if (val > 0.0)
                {
                    cst = epsNul;
                }
                else
                {
                    cst = -epsNul;
                }

                FunctionRoots res1 = new(f, s.GetParameter(i - 1), s.GetParameter(i), 10, epsX, epsF, 0.0, cst);
                Standard_NumericError_Raise_if((!res1.IsDone) || (res1.IsAllNull) || (res1.SolutionCount == 0), " ");

                finNul = res1.Value(1);
                indf = res1.StateNumber(1);

                cst = -cst;
                FunctionRoots res2 = new(f, s.GetParameter(i - 1), s.GetParameter(i), 10, epsX, epsF, 0.0, cst);
                Standard_NumericError_Raise_if((!res2.IsDone) || (res2.IsAllNull), " ");

                //-- || (Res2.NbSolutions()!=0), " ");  lbr le 13 mai 87 (!=0 -> ==0)
                if (res2.SolutionCount != 0)
                {
                    if (res2.Value(1) < finNul)
                    {
                        finNul = res2.Value(1);
                        indf = res2.StateNumber(1);
                    }
                }

                _pfin.Append(finNul);
                _ifin.Append(indf);
            }
            else if ((!interNul) && pNul && nul)
            {
                interNul = true;
                if (i == 2)
                {
                    debNul = s.GetParameter(1);
                    f.Value(debNul, out valbid);
                    indd = f.GetStateNumber();
                    nuld = true;
                }
                else
                {
                    if (valsav > 0.0)
                    {
                        cst = epsNul;
                    }
                    else
                    {
                        cst = -epsNul;
                    }

                    FunctionRoots res1 = new(f, s.GetParameter(i - 2), s.GetParameter(i - 1), 10, epsX, epsF, 0.0, cst);
                    Standard_NumericError_Raise_if((!res1.IsDone) || (res1.IsAllNull) || (res1.SolutionCount == 0), " ");
                    debNul = res1.Value(res1.SolutionCount);
                    indd = res1.StateNumber(res1.SolutionCount);

                    cst = -cst;
                    FunctionRoots res3 = new(f, s.GetParameter(i - 2), s.GetParameter(i - 1), 10, epsX, epsF, 0.0, cst);
                    Standard_NumericError_Raise_if((!res3.IsDone) || (res3.IsAllNull), " ");

                    if (res3.SolutionCount != 0)
                    {
                        if (res3.Value(res3.SolutionCount) > debNul)
                        {
                            debNul = res3.Value(res3.SolutionCount);
                            indd = res3.StateNumber(res3.SolutionCount);
                        }
                    }
                }
            }

            i = i + 1;
            pNul = nul;
            fini = (i > nbp);
        }

        if (interNul)
        { // rajouter l intervalle finissant au dernier pt
            _pdeb.Append(debNul);
            _ideb.Append(indd);
            finNul = s.GetParameter(nbp);
            f.Value(finNul, out valbid);
            indf = f.GetStateNumber();
            _pfin.Append(finNul);
            _ifin.Append(indf);
            nulf = true;
        }

        if (_pdeb.Length() == 0)
        { // Pas d intervalle nul
            FunctionRoots res = new(f, s.GetParameter(1), s.GetParameter(nbp), nbp, epsX, epsF, 0.0);
            Standard_NumericError_Raise_if((!res.IsDone) || (res.IsAllNull), " ");

            for (int j = 1; j <= res.SolutionCount; j++)
            {
                _piso.Append(res.Value(j));
                _iiso.Append(res.StateNumber(j));
            }
        }
        else
        {
            int nbpMin = 3;
            int nbrpt;
            if (!nuld)
            { // Recherche des solutions entre S.GetParameter(1)
              // et le debut du 1er intervalle nul
                nbrpt = (int)Math.Truncate(Math.Abs((_pdeb.Value(1) - s.GetParameter(1)) / (s.GetParameter(nbp) - s.GetParameter(1))) * nbp);
                FunctionRoots res = new(f, s.GetParameter(1), _pdeb.Value(1), Max(nbrpt, nbpMin), epsX, epsF, 0.0);
                Standard_NumericError_Raise_if((!res.IsDone) || (res.IsAllNull), " ");

                for (int j = 1; j <= res.SolutionCount; j++)
                {
                    _piso.Append(res.Value(j));
                    _iiso.Append(res.StateNumber(j));
                }
            }

            for (int k = 2; k <= _pdeb.Length(); k++)
            {
                nbrpt = (int)Math.Truncate(Math.Abs((_pdeb.Value(k) - _pfin.Value(k - 1)) / (s.GetParameter(nbp) - s.GetParameter(1))) * nbp);
                FunctionRoots res = new(f, _pfin.Value(k - 1), _pdeb.Value(k), Max(nbrpt, nbpMin), epsX, epsF, 0.0);
                Standard_NumericError_Raise_if((!res.IsDone) || (res.IsAllNull), " ");

                for (int j = 1; j <= res.SolutionCount; j++)
                {
                    _piso.Append(res.Value(j));
                    _iiso.Append(res.StateNumber(j));
                }
            }

            if (!nulf)
            { // Recherche des solutions entre la fin du
              // dernier intervalle nul et Value(Nbp).
                nbrpt = (int)Math.Truncate(Math.Abs((s.GetParameter(nbp) - _pfin.Value(_pdeb.Length())) / (s.GetParameter(nbp) - s.GetParameter(1))) * nbp);
                FunctionRoots res = new(f, _pfin.Value(_pdeb.Length()), s.GetParameter(nbp), Max(nbrpt, nbpMin), epsX, epsF, 0.0);
                Standard_NumericError_Raise_if((!res.IsDone) || (res.IsAllNull), " ");

                for (int j = 1; j <= res.SolutionCount; j++)
                {
                    _piso.Append(res.Value(j));
                    _iiso.Append(res.StateNumber(j));
                }
            }
        }

        _done = true;
    }

    public bool IsDone => _done;

    /// <summary>The number of intervals on which the function is null. Throws <see cref="NotDoneException"/> if not done.</summary>
    public int IntervalCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, " ");
            return _pdeb.Length();
        }
    }

    /// <summary>The interval number <paramref name="index"/>, in 1..NbIntervals.</summary>
    public void GetInterval(int index, out double a, out double b)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        a = _pdeb.Value(index);
        b = _pfin.Value(index);
    }

    /// <summary>The state numbers of the interval number <paramref name="index"/>, in 1..NbIntervals.</summary>
    public void GetIntervalState(int index, out int iFirst, out int iLast)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        iFirst = _ideb.Value(index);
        iLast = _ifin.Value(index);
    }

    /// <summary>The number of points where the function is null. Throws <see cref="NotDoneException"/> if not done.</summary>
    public int PointCount
    {
        get
        {
            StdFail_NotDone_Raise_if(!_done, " ");
            return _piso.Length();
        }
    }

    /// <summary>The parameter of the point number <paramref name="index"/>, in 1..NbPoints.</summary>
    public double GetPoint(int index)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _piso.Value(index);
    }

    /// <summary>The state number of the point number <paramref name="index"/>, in 1..NbPoints.</summary>
    public int GetPointState(int index)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _iiso.Value(index);
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_FunctionAllRoots ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Number of null intervals = " + _pdeb.Length() + '\n');
            o.Write(" Number of points where the function is null: " + _piso.Length() + '\n');
        }
        else
        {
            o.Write(" Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
