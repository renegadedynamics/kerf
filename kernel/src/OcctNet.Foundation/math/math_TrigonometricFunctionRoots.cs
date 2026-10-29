// Port of: src/FoundationClasses/TKMath/math/math_TrigonometricFunctionRoots.hxx
// Port of: src/FoundationClasses/TKMath/math/math_TrigonometricFunctionRoots.lxx
// Port of: src/FoundationClasses/TKMath/math/math_TrigonometricFunctionRoots.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// The solutions of the equation a*cos(x)*cos(x) + 2*b*cos(x)*sin(x) + c*cos(x) + d*sin(x) + e = 0, of degree 4, 3
/// or 2, within given bounds.
/// </summary>
public class math_TrigonometricFunctionRoots
{
    // lpa, le 03/09/91
    // Implementation de la classe resolvant les equations en cosinus-sinus.

    private int NbSol;
    private readonly NCollection_Array1<double> Sol;
    private bool InfiniteStatus;
    private bool Done;

    /// <summary>
    /// Solves the equation with the coefficients a, b, c, d, e for solutions in [InfBound, SupBound] (for example
    /// 0 and 2*PI).
    /// </summary>
    public math_TrigonometricFunctionRoots(double theA, double theB, double theC, double theD, double theE,
                                           double theInfBound, double theSupBound)
    {
        NbSol = -1;
        Sol = new NCollection_Array1<double>(1, 4);
        InfiniteStatus = false;
        Done = false;
        Perform(theA, theB, theC, theD, theE, theInfBound, theSupBound);
    }

    /// <summary>Solves d*sin(x) + e = 0 for solutions in [InfBound, SupBound].</summary>
    public math_TrigonometricFunctionRoots(double theD, double theE, double theInfBound, double theSupBound)
    {
        NbSol = -1;
        Sol = new NCollection_Array1<double>(1, 4);
        InfiniteStatus = false;
        Done = false;
        double A = 0.0, B = 0.0, C = 0.0;
        Perform(A, B, C, theD, theE, theInfBound, theSupBound);
    }

    /// <summary>Solves c*cos(x) + d*sin(x) + e = 0 for solutions in [InfBound, SupBound].</summary>
    public math_TrigonometricFunctionRoots(double theC, double theD, double theE, double theInfBound, double theSupBound)
    {
        NbSol = -1;
        Sol = new NCollection_Array1<double>(1, 4);
        InfiniteStatus = false;
        Done = false;
        double A = 0.0, B = 0.0;
        Perform(A, B, theC, theD, theE, theInfBound, theSupBound);
    }

    public bool IsDone() => Done;

    /// <summary>True if there are infinitely many roots.</summary>
    public bool InfiniteRoots() => InfiniteStatus;

    /// <summary>
    /// The solution number <paramref name="Index"/>. Throws if not done, if there are infinitely many solutions, or if
    /// Index &gt; NbSolutions.
    /// </summary>
    public double Value(int Index)
    {
        StdFail_InfiniteSolutions_Raise_if(InfiniteStatus, " ");
        StdFail_NotDone_Raise_if(!Done, " ");
        Standard_OutOfRange_Raise_if(Index > NbSol, " ");
        return Sol[Index];
    }

    /// <summary>The number of solutions. Throws if not done or if there are infinitely many.</summary>
    public int NbSolutions()
    {
        StdFail_InfiniteSolutions_Raise_if(InfiniteStatus, " ");
        StdFail_NotDone_Raise_if(!Done, " ");
        return NbSol;
    }

    public void Dump(TextWriter o)
    {
        o.Write(" math_TrigonometricFunctionRoots: \n");
        if (!Done)
        {
            o.Write("Not Done \n");
        }
        else if (InfiniteStatus)
        {
            o.Write(" There is an infinity of roots\n");
        }
        else if (!InfiniteStatus)
        {
            o.Write(" Number of solutions = " + NbSol + "\n");
            for (int i = 1; i <= NbSol; i++)
            {
                o.Write(" Value number " + i + "= " + Sol[i].ToString(CultureInfo.InvariantCulture) + "\n");
            }
        }
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }

    /// <summary>Solves the equation; used by the constructors.</summary>
    protected void Perform(double A, double B, double C, double D, double E, double InfBound, double SupBound)
    {
        int i, j = 0, k, l, NZer = 0, Nit = 10;
        double Depi, Delta, Mod, AA, BB, CC, MyBorneInf;
        double Teta, X;
        double Eps, Tol1 = 1.0e-15;
        NCollection_Array1<double> ko = new(1, 5), Zer = new(1, 4);
        bool Flag4;
        InfiniteStatus = false;
        Done = true;

        Eps = 1.5e-12;

        Depi = M_PI + M_PI;
        if (InfBound <= RealFirst() && SupBound >= RealLast())
        {
            MyBorneInf = 0.0;
            Delta = Depi;
            Mod = 0.0;
        }
        else if (SupBound >= RealLast())
        {
            MyBorneInf = InfBound;
            Delta = Depi;
            Mod = MyBorneInf / Depi;
        }
        else if (InfBound <= RealFirst())
        {
            MyBorneInf = SupBound - Depi;
            Delta = Depi;
            Mod = MyBorneInf / Depi;
        }
        else
        {
            MyBorneInf = InfBound;
            Delta = SupBound - InfBound;
            Mod = InfBound / Depi;
            if ((SupBound - InfBound) > Depi)
            {
                Delta = Depi;
            }
        }

        if ((Math.Abs(A) <= Eps) && (Math.Abs(B) <= Eps))
        {
            if (Math.Abs(C) <= Eps)
            {
                if (Math.Abs(D) <= Eps)
                {
                    if (Math.Abs(E) <= Eps)
                    {
                        InfiniteStatus = true; // infinite de solutions.
                        return;
                    }
                    else
                    {
                        NbSol = 0;
                        return;
                    }
                }
                else
                {
                    // Equation du type d*sin(x) + e = 0
                    // =================================
                    NbSol = 0;
                    AA = -E / D;
                    if (Math.Abs(AA) > 1.0)
                    {
                        return;
                    }

                    Zer[1] = Math.Asin(AA);
                    Zer[2] = M_PI - Zer[1];
                    NZer = 2;
                    for (i = 1; i <= NZer; i++)
                    {
                        if (Zer[i] <= -Eps)
                        {
                            Zer[i] = Depi - Math.Abs(Zer[i]);
                        }

                        // On rend les solutions entre InfBound et SupBound:
                        // =================================================
                        Zer[i] += Math.Truncate(Mod) * Depi;
                        X = Zer[i] - MyBorneInf;
                        if ((X > (-Epsilon(Delta))) && (X < Delta + Epsilon(Delta)))
                        {
                            NbSol++;
                            Sol[NbSol] = Zer[i];
                        }
                    }
                }

                return;
            }
            else if (Math.Abs(D) <= Eps)
            {
                // Equation du premier degre de la forme c*cos(x) + e = 0
                // ======================================================
                NbSol = 0;
                AA = -E / C;
                if (Math.Abs(AA) > 1.0)
                {
                    return;
                }

                Zer[1] = Math.Acos(AA);
                Zer[2] = -Zer[1];
                NZer = 2;

                for (i = 1; i <= NZer; i++)
                {
                    if (Zer[i] <= -Eps)
                    {
                        Zer[i] = Depi - Math.Abs(Zer[i]);
                    }

                    // On rend les solutions entre InfBound et SupBound:
                    // =================================================
                    Zer[i] += Math.Truncate(Mod) * 2.0 * M_PI;
                    X = Zer[i] - MyBorneInf;
                    if ((X >= (-Epsilon(Delta))) && (X <= Delta + Epsilon(Delta)))
                    {
                        NbSol++;
                        Sol[NbSol] = Zer[i];
                    }
                }

                return;
            }
            else
            {
                // Equation du second degre:
                // =========================
                AA = E - C;
                BB = 2.0 * D;
                CC = E + C;

                math_DirectPolynomialRoots Resol = new(AA, BB, CC);
                if (!Resol.IsDone())
                {
                    Done = false;
                    return;
                }
                else if (!Resol.InfiniteRoots())
                {
                    NZer = Resol.NbSolutions();
                    for (i = 1; i <= NZer; i++)
                    {
                        Zer[i] = Resol.Value(i);
                    }
                }
                else if (Resol.InfiniteRoots())
                {
                    InfiniteStatus = true;
                    return;
                }
            }
        }
        else
        {
            // Two additional analytical cases.
            if ((Math.Abs(A) <= Eps) && (Math.Abs(E) <= Eps))
            {
                if (Math.Abs(C) <= Eps)
                {
                    // 2 * B * sin * cos + D * sin = 0
                    NZer = 2;
                    Zer[1] = 0.0;
                    Zer[2] = M_PI;

                    AA = -D / (B * 2);
                    if (Math.Abs(AA) <= 1.0 + Precision.PConfusion())
                    {
                        NZer = 4;
                        if (AA >= 1.0)
                        {
                            Zer[3] = 0.0;
                            Zer[4] = 0.0;
                        }
                        else if (AA <= -1.0)
                        {
                            Zer[3] = M_PI;
                            Zer[4] = M_PI;
                        }
                        else
                        {
                            Zer[3] = Math.Acos(AA);
                            Zer[4] = Depi - Zer[3];
                        }
                    }

                    NbSol = 0;
                    for (i = 1; i <= NZer; i++)
                    {
                        if (Zer[i] <= MyBorneInf - Eps)
                        {
                            Zer[i] += Depi;
                        }

                        // On rend les solutions entre InfBound et SupBound:
                        // =================================================
                        Zer[i] += Math.Truncate(Mod) * 2.0 * M_PI;
                        X = Zer[i] - MyBorneInf;
                        if ((X >= (-Precision.PConfusion())) && (X <= Delta + Precision.PConfusion()))
                        {
                            if (Zer[i] < InfBound)
                            {
                                Zer[i] = InfBound;
                            }

                            if (Zer[i] > SupBound)
                            {
                                Zer[i] = SupBound;
                            }

                            NbSol++;
                            Sol[NbSol] = Zer[i];
                        }
                    }

                    return;
                }

                if (Math.Abs(D) <= Eps)
                {
                    // 2 * B * sin * cos + C * cos = 0
                    NZer = 2;
                    Zer[1] = M_PI / 2.0;
                    Zer[2] = M_PI * 3.0 / 2.0;

                    AA = -C / (B * 2);
                    if (Math.Abs(AA) <= 1.0 + Precision.PConfusion())
                    {
                        NZer = 4;
                        if (AA >= 1.0)
                        {
                            Zer[3] = M_PI / 2.0;
                            Zer[4] = M_PI / 2.0;
                        }
                        else if (AA <= -1.0)
                        {
                            Zer[3] = M_PI * 3.0 / 2.0;
                            Zer[4] = M_PI * 3.0 / 2.0;
                        }
                        else
                        {
                            Zer[3] = Math.Asin(AA);
                            Zer[4] = M_PI - Zer[3];
                        }
                    }

                    NbSol = 0;
                    for (i = 1; i <= NZer; i++)
                    {
                        if (Zer[i] <= MyBorneInf - Eps)
                        {
                            Zer[i] += Depi;
                        }

                        // On rend les solutions entre InfBound et SupBound:
                        // =================================================
                        Zer[i] += Math.Truncate(Mod) * 2.0 * M_PI;
                        X = Zer[i] - MyBorneInf;
                        if ((X >= (-Precision.PConfusion())) && (X <= Delta + Precision.PConfusion()))
                        {
                            if (Zer[i] < InfBound)
                            {
                                Zer[i] = InfBound;
                            }

                            if (Zer[i] > SupBound)
                            {
                                Zer[i] = SupBound;
                            }

                            NbSol++;
                            Sol[NbSol] = Zer[i];
                        }
                    }

                    return;
                }
            }

            // Equation du 4 ieme degre
            // ========================
            ko[1] = A - C + E;
            ko[2] = 2.0 * D - 4.0 * B;
            ko[3] = 2.0 * E - 2.0 * A;
            ko[4] = 4.0 * B + 2.0 * D;
            ko[5] = A + C + E;
            bool bko;
            do
            {
                bko = false;
                math_DirectPolynomialRoots Resol4 = new(ko[1], ko[2], ko[3], ko[4], ko[5]);
                if (!Resol4.IsDone())
                {
                    Done = false;
                    return;
                }
                else if (!Resol4.InfiniteRoots())
                {
                    NZer = Resol4.NbSolutions();
                    for (i = 1; i <= NZer; i++)
                    {
                        Zer[i] = Resol4.Value(i);
                    }
                }
                else if (Resol4.InfiniteRoots())
                {
                    InfiniteStatus = true;
                    return;
                }

                bool triok;
                do
                {
                    triok = true;
                    for (i = 1; i < NZer; i++)
                    {
                        if (Zer[i] > Zer[i + 1])
                        {
                            double t = Zer[i];
                            Zer[i] = Zer[i + 1];
                            Zer[i + 1] = t;
                            triok = false;
                        }
                    }
                } while (!triok);

                for (i = 1; i < NZer; i++)
                {
                    if (Math.Abs(Zer[i + 1] - Zer[i]) < Eps)
                    {
                        //-- est ce une racine double ou une erreur numerique ?
                        double qw = Zer[i + 1];
                        double va = ko[4] + qw * (2.0 * ko[3] + qw * (3.0 * ko[2] + qw * (4.0 * ko[1])));
                        if (Math.Abs(va) > Eps)
                        {
                            bko = true;
                            break;
                        }
                    }
                }

                if (bko)
                {
                    //-- Si il y a un coeff petit, on divise
                    //--
                    ko[1] *= 0.0001;
                    ko[2] *= 0.0001;
                    ko[3] *= 0.0001;
                    ko[4] *= 0.0001;
                    ko[5] *= 0.0001;
                }
            } while (bko);
        }

        // Verification des solutions par rapport aux bornes:
        // ==================================================
        double SupmInfs100 = (SupBound - InfBound) * 0.01;
        NbSol = 0;
        for (i = 1; i <= NZer; i++)
        {
            Teta = Math.Atan(Zer[i]);
            Teta += Teta;
            if (Zer[i] <= (-Eps))
            {
                Teta = Depi - Math.Abs(Teta);
            }

            Teta += Math.Truncate(Mod) * Depi;
            if (Teta - MyBorneInf < 0)
            {
                Teta += Depi;
            }

            X = Teta - MyBorneInf;
            if ((X >= (-Epsilon(Delta))) && (X <= Delta + Epsilon(Delta)))
            {
                X = Teta;

                // Appel de Newton:
                // OCC541(apo):  double TetaNewton=0;
                double TetaNewton = Teta;
                math_TrigonometricEquationFunction MyF = new(A, B, C, D, E);
                math_NewtonFunctionRoot Resol = new(MyF, X, Tol1, Eps, Nit);
                if (Resol.IsDone())
                {
                    TetaNewton = Resol.Root();
                }

                //-- lbr le 7 mars 97 (newton converge tres tres loin de la solution initilale)
                double DeltaNewton = TetaNewton - Teta;
                if ((DeltaNewton > SupmInfs100) || (DeltaNewton < -SupmInfs100))
                {
                    //-- std::cout<<"\n Newton X0="<<Teta<<" -> "<<TetaNewton<<std::endl;
                }
                else
                {
                    Teta = TetaNewton;
                }

                Flag4 = false;

                for (k = 1; k <= NbSol; k++)
                {
                    // On met les valeurs par ordre croissant:
                    if (Teta < Sol[k])
                    {
                        for (l = k; l <= NbSol; l++)
                        {
                            j = NbSol - l + k;
                            Sol[j + 1] = Sol[j];
                        }

                        Sol[k] = Teta;
                        NbSol++;
                        Flag4 = true;
                        break;
                    }
                }

                if (!Flag4)
                {
                    NbSol++;
                    Sol[NbSol] = Teta;
                }
            }
        }

        // Cas particulier de  PI:
        if (NbSol < 4)
        {
            int startIndex = NbSol + 1;
            // OCCT checks for PI being already a solution on the first pass only, so when PI is a root that was not
            // found above, it is added once per free slot (4 - NbSol times).
            for (int solIt = startIndex; solIt <= 4; solIt++)
            {
                Teta = M_PI + Math.Truncate(Mod) * 2.0 * M_PI;
                X = Teta - MyBorneInf;
                if ((X >= (-Epsilon(Delta))) && (X <= Delta + Epsilon(Delta)))
                {
                    if (Math.Abs(A - C + E) <= Eps)
                    {
                        Flag4 = false;
                        for (k = 1; k <= NbSol; k++)
                        {
                            j = k;
                            if (Teta < Sol[k])
                            {
                                Flag4 = true;
                                break;
                            }

                            if ((solIt == startIndex) && (Math.Abs(Teta - Sol[k]) <= Eps))
                            {
                                return;
                            }
                        }

                        if (!Flag4)
                        {
                            NbSol++;
                            Sol[NbSol] = Teta;
                        }
                        else
                        {
                            for (k = j; k <= NbSol; k++)
                            {
                                i = NbSol - k + j;
                                Sol[i + 1] = Sol[i];
                            }

                            Sol[j] = Teta;
                            NbSol++;
                        }
                    }
                }
            }
        }
    }
}
