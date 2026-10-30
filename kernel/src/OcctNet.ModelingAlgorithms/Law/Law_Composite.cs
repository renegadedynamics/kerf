// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Composite.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_Composite.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public class Law_Composite : Law_Function
{
    private double first;
    private double last;
    private Law_Function? curfunc;
    private readonly NCollection_List<Law_Function> funclist = new NCollection_List<Law_Function>();
    private bool periodic;
    private readonly double TFirst;
    private readonly double TLast;
    private readonly double PTol;

    /// <summary>An empty law.</summary>
    public Law_Composite()
    {
        first = -1.0e100;
        last = 1.0e100;
        periodic = false;
        TFirst = -1.0e100;
        TLast = 1.0e100;
        PTol = 0.0;
    }

    /// <summary>An empty law trimmed to [First, Last]; Tol decides which law evaluates at a junction.</summary>
    public Law_Composite(double First, double Last, double Tol)
    {
        first = -1.0e100;
        last = 1.0e100;
        periodic = false;
        TFirst = First;
        TLast = Last;
        PTol = Tol;
    }

    /// <summary>Not implemented in OCCT.</summary>
    /// <exception cref="Standard_NotImplemented">Always.</exception>
    public override GeomAbs_Shape Continuity()
    {
        throw new Standard_NotImplemented("Law_Composite::Continuity()");
    }

    /// <summary>The sum of the laws' intervals.</summary>
    public override int NbIntervals(GeomAbs_Shape S)
    {
        NCollection_List<Law_Function>.Iterator It = new NCollection_List<Law_Function>.Iterator(funclist);
        Law_Function func;
        int nbr_interval = 0;

        for (; It.More(); It.Next())
        {
            func = It.Value();
            nbr_interval += func.NbIntervals(S);
        }

        return nbr_interval;
    }

    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S)
    {
        NCollection_List<Law_Function>.Iterator It = new NCollection_List<Law_Function>.Iterator(funclist);
        Law_Function func;
        NCollection_HArray1<double> LocT;
        int nb_index, Iloc, IGlob = 2;

        func = funclist.First();
        func.Bounds(out T[1], out T[2]);

        for (; It.More(); It.Next())
        {
            func = It.Value();
            nb_index = func.NbIntervals(S) + 1;
            LocT = new NCollection_HArray1<double>(1, nb_index);
            func.Intervals(LocT.ChangeArray1(), S);
            for (Iloc = 2; Iloc <= nb_index; Iloc++, IGlob++)
            {
                T[IGlob] = LocT.Value(Iloc);
            }
        }
    }

    public override double Value(double X)
    {
        double W = X;
        Prepare(ref W);
        return curfunc!.Value(W);
    }

    public override void D1(double X, out double F, out double D)
    {
        double W = X;
        Prepare(ref W);
        curfunc!.D1(W, out F, out D);
    }

    public override void D2(double X, out double F, out double D, out double D2)
    {
        double W = X;
        Prepare(ref W);
        curfunc!.D2(W, out F, out D, out D2);
    }

    /// <summary>A composite of the same laws trimmed to [PFirst, PLast].</summary>
    public override Law_Function Trim(double PFirst, double PLast, double Tol)
    {
        Law_Composite l = new Law_Composite(PFirst, PLast, Tol);
        l.ChangeLaws().Assign(funclist);
        return l;
    }

    public override void Bounds(out double PFirst, out double PLast)
    {
        PFirst = first;
        PLast = last;
    }

    // Lorsque le parametre est pres d'un "noeud" on determine la loi en
    // fonction du signe de tol:
    //   - negatif -> Loi precedente au noeud.
    //   - positif -> Loi consecutive au noeud.
    private void Prepare(ref double W)
    {
        double f, l, Wtest, Eps;
        if (W - TFirst < TLast - W)
        {
            Eps = PTol;
        }
        else
        {
            Eps = -PTol;
        }

        if (curfunc is null)
        {
            curfunc = funclist.Last();
            curfunc.Bounds(out f, out last);
            curfunc = funclist.First();
            curfunc.Bounds(out first, out l);
        }

        Wtest = W + Eps; // Decalage pour discriminer les noeuds
        if (periodic)
        {
            Wtest = ElCLib.InPeriod(Wtest, first, last);
            W = Wtest - Eps;
        }

        curfunc.Bounds(out f, out l);
        if (f <= Wtest && Wtest <= l)
        {
            return;
        }
        if (W <= first)
        {
            curfunc = funclist.First();
        }
        else if (W >= last)
        {
            curfunc = funclist.Last();
        }
        else
        {
            NCollection_List<Law_Function>.Iterator It = new NCollection_List<Law_Function>.Iterator(funclist);
            for (; It.More(); It.Next())
            {
                curfunc = It.Value();
                curfunc.Bounds(out f, out l);
                if (f <= Wtest && Wtest <= l)
                {
                    return;
                }
            }
        }
    }

    /// <summary>A reference to the elementary law that evaluates at W.</summary>
    public ref Law_Function? ChangeElementaryLaw(double W)
    {
        double WW = W;
        Prepare(ref WW);
        return ref curfunc;
    }

    /// <summary>The list of laws itself, to fill or change.</summary>
    public NCollection_List<Law_Function> ChangeLaws() => funclist;

    public bool IsPeriodic() => periodic;

    public void SetPeriodic()
    {
        periodic = true;
    }
}
