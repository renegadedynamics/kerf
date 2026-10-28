// Port of: src/FoundationClasses/TKMath/math/math_FunctionAllRoots.hxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionAllRoots.lxx
// Port of: src/FoundationClasses/TKMath/math/math_FunctionAllRoots.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// Uses a sample of a function to find all the intervals on which it is null, then
/// <see cref="math_FunctionRoots"/> to find the points where it is null outside them. The derivative is required.
/// </summary>
public class math_FunctionAllRoots
{
    private bool done;
    private readonly NCollection_Sequence<double> pdeb = new();
    private readonly NCollection_Sequence<double> pfin = new();
    private readonly NCollection_Sequence<double> piso = new();
    private readonly NCollection_Sequence<int> ideb = new();
    private readonly NCollection_Sequence<int> ifin = new();
    private readonly NCollection_Sequence<int> iiso = new();

    /// <summary>
    /// An interval is found where, for at least two consecutive sample points Ui and Ui+1, |F(Ui)| &lt;= EpsNul and
    /// |F(Ui+1)| &lt;= EpsNul; its real bounds, and the roots between intervals, are computed with
    /// <see cref="math_FunctionRoots"/>. Throws <see cref="Standard_NumericError"/> if that fails.
    /// </summary>
    public math_FunctionAllRoots(math_FunctionWithDerivative F, math_FunctionSample S, double EpsX, double EpsF, double EpsNul)
    {
        done = false;

        bool Nul, PNul, InterNul, Nuld, Nulf;
        double DebNul = 0.0, FinNul = 0.0;
        int Indd = 0, Indf = 0;
        double cst, val, valsav = 0, valbid;
        bool fini;
        int Nbp, i;

        Nbp = S.NbPoints();
        F.Value(S.GetParameter(1), out val);
        PNul = Math.Abs(val) <= EpsNul;
        if (!PNul)
        {
            valsav = val;
        }

        InterNul = false;
        Nuld = false;
        Nulf = false;

        i = 2;
        fini = (i > Nbp);

        while (!fini)
        {
            F.Value(S.GetParameter(i), out val);
            Nul = Math.Abs(val) <= EpsNul;
            if (!Nul)
            {
                valsav = val;
            }

            if (InterNul && (!Nul))
            {
                InterNul = false;
                pdeb.Append(DebNul);
                ideb.Append(Indd);
                if (val > 0.0)
                {
                    cst = EpsNul;
                }
                else
                {
                    cst = -EpsNul;
                }

                math_FunctionRoots Res1 = new(F, S.GetParameter(i - 1), S.GetParameter(i), 10, EpsX, EpsF, 0.0, cst);
                Standard_NumericError_Raise_if((!Res1.IsDone()) || (Res1.IsAllNull()) || (Res1.NbSolutions() == 0), " ");

                FinNul = Res1.Value(1);
                Indf = Res1.StateNumber(1);

                cst = -cst;
                math_FunctionRoots Res2 = new(F, S.GetParameter(i - 1), S.GetParameter(i), 10, EpsX, EpsF, 0.0, cst);
                Standard_NumericError_Raise_if((!Res2.IsDone()) || (Res2.IsAllNull()), " ");

                //-- || (Res2.NbSolutions()!=0), " ");  lbr le 13 mai 87 (!=0 -> ==0)
                if (Res2.NbSolutions() != 0)
                {
                    if (Res2.Value(1) < FinNul)
                    {
                        FinNul = Res2.Value(1);
                        Indf = Res2.StateNumber(1);
                    }
                }

                pfin.Append(FinNul);
                ifin.Append(Indf);
            }
            else if ((!InterNul) && PNul && Nul)
            {
                InterNul = true;
                if (i == 2)
                {
                    DebNul = S.GetParameter(1);
                    F.Value(DebNul, out valbid);
                    Indd = F.GetStateNumber();
                    Nuld = true;
                }
                else
                {
                    if (valsav > 0.0)
                    {
                        cst = EpsNul;
                    }
                    else
                    {
                        cst = -EpsNul;
                    }

                    math_FunctionRoots Res1 = new(F, S.GetParameter(i - 2), S.GetParameter(i - 1), 10, EpsX, EpsF, 0.0, cst);
                    Standard_NumericError_Raise_if((!Res1.IsDone()) || (Res1.IsAllNull()) || (Res1.NbSolutions() == 0), " ");
                    DebNul = Res1.Value(Res1.NbSolutions());
                    Indd = Res1.StateNumber(Res1.NbSolutions());

                    cst = -cst;
                    math_FunctionRoots Res3 = new(F, S.GetParameter(i - 2), S.GetParameter(i - 1), 10, EpsX, EpsF, 0.0, cst);
                    Standard_NumericError_Raise_if((!Res3.IsDone()) || (Res3.IsAllNull()), " ");

                    if (Res3.NbSolutions() != 0)
                    {
                        if (Res3.Value(Res3.NbSolutions()) > DebNul)
                        {
                            DebNul = Res3.Value(Res3.NbSolutions());
                            Indd = Res3.StateNumber(Res3.NbSolutions());
                        }
                    }
                }
            }

            i = i + 1;
            PNul = Nul;
            fini = (i > Nbp);
        }

        if (InterNul)
        { // rajouter l intervalle finissant au dernier pt
            pdeb.Append(DebNul);
            ideb.Append(Indd);
            FinNul = S.GetParameter(Nbp);
            F.Value(FinNul, out valbid);
            Indf = F.GetStateNumber();
            pfin.Append(FinNul);
            ifin.Append(Indf);
            Nulf = true;
        }

        if (pdeb.Length() == 0)
        { // Pas d intervalle nul
            math_FunctionRoots Res = new(F, S.GetParameter(1), S.GetParameter(Nbp), Nbp, EpsX, EpsF, 0.0);
            Standard_NumericError_Raise_if((!Res.IsDone()) || (Res.IsAllNull()), " ");

            for (int j = 1; j <= Res.NbSolutions(); j++)
            {
                piso.Append(Res.Value(j));
                iiso.Append(Res.StateNumber(j));
            }
        }
        else
        {
            int NbpMin = 3;
            int Nbrpt;
            if (!Nuld)
            { // Recherche des solutions entre S.GetParameter(1)
              // et le debut du 1er intervalle nul
                Nbrpt = (int)Math.Truncate(Math.Abs((pdeb.Value(1) - S.GetParameter(1)) / (S.GetParameter(Nbp) - S.GetParameter(1))) * Nbp);
                math_FunctionRoots Res = new(F, S.GetParameter(1), pdeb.Value(1), Max(Nbrpt, NbpMin), EpsX, EpsF, 0.0);
                Standard_NumericError_Raise_if((!Res.IsDone()) || (Res.IsAllNull()), " ");

                for (int j = 1; j <= Res.NbSolutions(); j++)
                {
                    piso.Append(Res.Value(j));
                    iiso.Append(Res.StateNumber(j));
                }
            }

            for (int k = 2; k <= pdeb.Length(); k++)
            {
                Nbrpt = (int)Math.Truncate(Math.Abs((pdeb.Value(k) - pfin.Value(k - 1)) / (S.GetParameter(Nbp) - S.GetParameter(1))) * Nbp);
                math_FunctionRoots Res = new(F, pfin.Value(k - 1), pdeb.Value(k), Max(Nbrpt, NbpMin), EpsX, EpsF, 0.0);
                Standard_NumericError_Raise_if((!Res.IsDone()) || (Res.IsAllNull()), " ");

                for (int j = 1; j <= Res.NbSolutions(); j++)
                {
                    piso.Append(Res.Value(j));
                    iiso.Append(Res.StateNumber(j));
                }
            }

            if (!Nulf)
            { // Recherche des solutions entre la fin du
              // dernier intervalle nul et Value(Nbp).
                Nbrpt = (int)Math.Truncate(Math.Abs((S.GetParameter(Nbp) - pfin.Value(pdeb.Length())) / (S.GetParameter(Nbp) - S.GetParameter(1))) * Nbp);
                math_FunctionRoots Res = new(F, pfin.Value(pdeb.Length()), S.GetParameter(Nbp), Max(Nbrpt, NbpMin), EpsX, EpsF, 0.0);
                Standard_NumericError_Raise_if((!Res.IsDone()) || (Res.IsAllNull()), " ");

                for (int j = 1; j <= Res.NbSolutions(); j++)
                {
                    piso.Append(Res.Value(j));
                    iiso.Append(Res.StateNumber(j));
                }
            }
        }

        done = true;
    }

    public bool IsDone() => done;

    /// <summary>The number of intervals on which the function is null. Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public int NbIntervals()
    {
        StdFail_NotDone_Raise_if(!done, " ");
        return pdeb.Length();
    }

    /// <summary>The interval number <paramref name="Index"/>, in 1..NbIntervals.</summary>
    public void GetInterval(int Index, out double A, out double B)
    {
        StdFail_NotDone_Raise_if(!done, " ");
        A = pdeb.Value(Index);
        B = pfin.Value(Index);
    }

    /// <summary>The state numbers of the interval number <paramref name="Index"/>, in 1..NbIntervals.</summary>
    public void GetIntervalState(int Index, out int IFirst, out int ILast)
    {
        StdFail_NotDone_Raise_if(!done, " ");
        IFirst = ideb.Value(Index);
        ILast = ifin.Value(Index);
    }

    /// <summary>The number of points where the function is null. Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public int NbPoints()
    {
        StdFail_NotDone_Raise_if(!done, " ");
        return piso.Length();
    }

    /// <summary>The parameter of the point number <paramref name="Index"/>, in 1..NbPoints.</summary>
    public double GetPoint(int Index)
    {
        StdFail_NotDone_Raise_if(!done, " ");
        return piso.Value(Index);
    }

    /// <summary>The state number of the point number <paramref name="Index"/>, in 1..NbPoints.</summary>
    public int GetPointState(int Index)
    {
        StdFail_NotDone_Raise_if(!done, " ");
        return iiso.Value(Index);
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_FunctionAllRoots ");
        if (done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Number of null intervals = " + pdeb.Length() + '\n');
            o.Write(" Number of points where the function is null: " + piso.Length() + '\n');
        }
        else
        {
            o.Write(" Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
