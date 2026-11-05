// Port of: src/FoundationClasses/TKMath/math/math_TrigonometricFunctionRoots.hxx
// Port of: src/FoundationClasses/TKMath/math/math_TrigonometricFunctionRoots.lxx
// Port of: src/FoundationClasses/TKMath/math/math_TrigonometricFunctionRoots.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The solutions of the equation a*cos(x)*cos(x) + 2*b*cos(x)*sin(x) + c*cos(x) + d*sin(x) + e = 0, of degree 4, 3
/// or 2, within given bounds.
/// </summary>
internal class TrigonometricFunctionRoots
{
    // lpa, le 03/09/91
    // Implementation de la classe resolvant les equations en cosinus-sinus.

    private int _nbSol;
    private readonly NCollection_Array1<double> _sol;
    private bool _infiniteStatus;
    private bool _done;

    /// <summary>
    /// Solves the equation with the coefficients a, b, c, d, e for solutions in [InfBound, SupBound] (for example
    /// 0 and 2*PI).
    /// </summary>
    public TrigonometricFunctionRoots(double a, double b, double c, double d, double e,
                                           double infBound, double supBound)
    {
        _nbSol = -1;
        _sol = new NCollection_Array1<double>(1, 4);
        _infiniteStatus = false;
        _done = false;
        Perform(a, b, c, d, e, infBound, supBound);
    }

    /// <summary>Solves d*sin(x) + e = 0 for solutions in [InfBound, SupBound].</summary>
    public TrigonometricFunctionRoots(double d, double e, double infBound, double supBound)
    {
        _nbSol = -1;
        _sol = new NCollection_Array1<double>(1, 4);
        _infiniteStatus = false;
        _done = false;
        double a = 0.0, b = 0.0, c = 0.0;
        Perform(a, b, c, d, e, infBound, supBound);
    }

    /// <summary>Solves c*cos(x) + d*sin(x) + e = 0 for solutions in [InfBound, SupBound].</summary>
    public TrigonometricFunctionRoots(double c, double d, double e, double infBound, double supBound)
    {
        _nbSol = -1;
        _sol = new NCollection_Array1<double>(1, 4);
        _infiniteStatus = false;
        _done = false;
        double a = 0.0, b = 0.0;
        Perform(a, b, c, d, e, infBound, supBound);
    }

    public bool IsDone => _done;

    /// <summary>True if there are infinitely many roots.</summary>
    public bool InfiniteRoots() => _infiniteStatus;

    /// <summary>
    /// The solution number <paramref name="index"/>. Throws if not done, if there are infinitely many solutions, or if
    /// Index &gt; NbSolutions.
    /// </summary>
    public double Value(int index)
    {
        StdFail_InfiniteSolutions_Raise_if(_infiniteStatus, " ");
        StdFail_NotDone_Raise_if(!_done, " ");
        Standard_OutOfRange_Raise_if(index > _nbSol, " ");
        return _sol[index];
    }

    /// <summary>The number of solutions. Throws if not done or if there are infinitely many.</summary>
    public int SolutionCount
    {
        get
        {
            StdFail_InfiniteSolutions_Raise_if(_infiniteStatus, " ");
            StdFail_NotDone_Raise_if(!_done, " ");
            return _nbSol;
        }
    }

    public void Dump(TextWriter o)
    {
        o.Write(" math_TrigonometricFunctionRoots: \n");
        if (!_done)
        {
            o.Write("Not Done \n");
        }
        else if (_infiniteStatus)
        {
            o.Write(" There is an infinity of roots\n");
        }
        else if (!_infiniteStatus)
        {
            o.Write(" Number of solutions = " + _nbSol + "\n");
            for (int i = 1; i <= _nbSol; i++)
            {
                o.Write(" Value number " + i + "= " + _sol[i].ToString(CultureInfo.InvariantCulture) + "\n");
            }
        }
    }

    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }

    /// <summary>Solves the equation; used by the constructors.</summary>
    protected void Perform(double a, double b, double c, double d, double e, double infBound, double supBound)
    {
        int i, j = 0, k, l, nZer = 0, nit = 10;
        double depi, delta, mod, aa, bb, cc, myBorneInf;
        double teta, x;
        double eps, tol1 = 1.0e-15;
        NCollection_Array1<double> ko = new(1, 5), zer = new(1, 4);
        bool flag4;
        _infiniteStatus = false;
        _done = true;

        eps = 1.5e-12;

        depi = M_PI + M_PI;
        if (infBound <= RealFirst() && supBound >= RealLast())
        {
            myBorneInf = 0.0;
            delta = depi;
            mod = 0.0;
        }
        else if (supBound >= RealLast())
        {
            myBorneInf = infBound;
            delta = depi;
            mod = myBorneInf / depi;
        }
        else if (infBound <= RealFirst())
        {
            myBorneInf = supBound - depi;
            delta = depi;
            mod = myBorneInf / depi;
        }
        else
        {
            myBorneInf = infBound;
            delta = supBound - infBound;
            mod = infBound / depi;
            if ((supBound - infBound) > depi)
            {
                delta = depi;
            }
        }

        if ((Math.Abs(a) <= eps) && (Math.Abs(b) <= eps))
        {
            if (Math.Abs(c) <= eps)
            {
                if (Math.Abs(d) <= eps)
                {
                    if (Math.Abs(e) <= eps)
                    {
                        _infiniteStatus = true; // infinite de solutions.
                        return;
                    }
                    else
                    {
                        _nbSol = 0;
                        return;
                    }
                }
                else
                {
                    // Equation du type d*sin(x) + e = 0
                    // =================================
                    _nbSol = 0;
                    aa = -e / d;
                    if (Math.Abs(aa) > 1.0)
                    {
                        return;
                    }

                    zer[1] = Math.Asin(aa);
                    zer[2] = M_PI - zer[1];
                    nZer = 2;
                    for (i = 1; i <= nZer; i++)
                    {
                        if (zer[i] <= -eps)
                        {
                            zer[i] = depi - Math.Abs(zer[i]);
                        }

                        // On rend les solutions entre InfBound et SupBound:
                        // =================================================
                        zer[i] += Math.Truncate(mod) * depi;
                        x = zer[i] - myBorneInf;
                        if ((x > (-Epsilon(delta))) && (x < delta + Epsilon(delta)))
                        {
                            _nbSol++;
                            _sol[_nbSol] = zer[i];
                        }
                    }
                }

                return;
            }
            else if (Math.Abs(d) <= eps)
            {
                // Equation du premier degre de la forme c*cos(x) + e = 0
                // ======================================================
                _nbSol = 0;
                aa = -e / c;
                if (Math.Abs(aa) > 1.0)
                {
                    return;
                }

                zer[1] = Math.Acos(aa);
                zer[2] = -zer[1];
                nZer = 2;

                for (i = 1; i <= nZer; i++)
                {
                    if (zer[i] <= -eps)
                    {
                        zer[i] = depi - Math.Abs(zer[i]);
                    }

                    // On rend les solutions entre InfBound et SupBound:
                    // =================================================
                    zer[i] += Math.Truncate(mod) * 2.0 * M_PI;
                    x = zer[i] - myBorneInf;
                    if ((x >= (-Epsilon(delta))) && (x <= delta + Epsilon(delta)))
                    {
                        _nbSol++;
                        _sol[_nbSol] = zer[i];
                    }
                }

                return;
            }
            else
            {
                // Equation du second degre:
                // =========================
                aa = e - c;
                bb = 2.0 * d;
                cc = e + c;

                DirectPolynomialRoots resol = new(aa, bb, cc);
                if (!resol.IsDone)
                {
                    _done = false;
                    return;
                }
                else if (!resol.InfiniteRoots())
                {
                    nZer = resol.SolutionCount;
                    for (i = 1; i <= nZer; i++)
                    {
                        zer[i] = resol.Value(i);
                    }
                }
                else if (resol.InfiniteRoots())
                {
                    _infiniteStatus = true;
                    return;
                }
            }
        }
        else
        {
            // Two additional analytical cases.
            if ((Math.Abs(a) <= eps) && (Math.Abs(e) <= eps))
            {
                if (Math.Abs(c) <= eps)
                {
                    // 2 * B * sin * cos + D * sin = 0
                    nZer = 2;
                    zer[1] = 0.0;
                    zer[2] = M_PI;

                    aa = -d / (b * 2);
                    if (Math.Abs(aa) <= 1.0 + global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        nZer = 4;
                        if (aa >= 1.0)
                        {
                            zer[3] = 0.0;
                            zer[4] = 0.0;
                        }
                        else if (aa <= -1.0)
                        {
                            zer[3] = M_PI;
                            zer[4] = M_PI;
                        }
                        else
                        {
                            zer[3] = Math.Acos(aa);
                            zer[4] = depi - zer[3];
                        }
                    }

                    _nbSol = 0;
                    for (i = 1; i <= nZer; i++)
                    {
                        if (zer[i] <= myBorneInf - eps)
                        {
                            zer[i] += depi;
                        }

                        // On rend les solutions entre InfBound et SupBound:
                        // =================================================
                        zer[i] += Math.Truncate(mod) * 2.0 * M_PI;
                        x = zer[i] - myBorneInf;
                        if ((x >= (-global::Kerf.Engine.Foundation.Precision.PConfusion())) && (x <= delta + global::Kerf.Engine.Foundation.Precision.PConfusion()))
                        {
                            if (zer[i] < infBound)
                            {
                                zer[i] = infBound;
                            }

                            if (zer[i] > supBound)
                            {
                                zer[i] = supBound;
                            }

                            _nbSol++;
                            _sol[_nbSol] = zer[i];
                        }
                    }

                    return;
                }

                if (Math.Abs(d) <= eps)
                {
                    // 2 * B * sin * cos + C * cos = 0
                    nZer = 2;
                    zer[1] = M_PI / 2.0;
                    zer[2] = M_PI * 3.0 / 2.0;

                    aa = -c / (b * 2);
                    if (Math.Abs(aa) <= 1.0 + global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        nZer = 4;
                        if (aa >= 1.0)
                        {
                            zer[3] = M_PI / 2.0;
                            zer[4] = M_PI / 2.0;
                        }
                        else if (aa <= -1.0)
                        {
                            zer[3] = M_PI * 3.0 / 2.0;
                            zer[4] = M_PI * 3.0 / 2.0;
                        }
                        else
                        {
                            zer[3] = Math.Asin(aa);
                            zer[4] = M_PI - zer[3];
                        }
                    }

                    _nbSol = 0;
                    for (i = 1; i <= nZer; i++)
                    {
                        if (zer[i] <= myBorneInf - eps)
                        {
                            zer[i] += depi;
                        }

                        // On rend les solutions entre InfBound et SupBound:
                        // =================================================
                        zer[i] += Math.Truncate(mod) * 2.0 * M_PI;
                        x = zer[i] - myBorneInf;
                        if ((x >= (-global::Kerf.Engine.Foundation.Precision.PConfusion())) && (x <= delta + global::Kerf.Engine.Foundation.Precision.PConfusion()))
                        {
                            if (zer[i] < infBound)
                            {
                                zer[i] = infBound;
                            }

                            if (zer[i] > supBound)
                            {
                                zer[i] = supBound;
                            }

                            _nbSol++;
                            _sol[_nbSol] = zer[i];
                        }
                    }

                    return;
                }
            }

            // Equation du 4 ieme degre
            // ========================
            ko[1] = a - c + e;
            ko[2] = 2.0 * d - 4.0 * b;
            ko[3] = 2.0 * e - 2.0 * a;
            ko[4] = 4.0 * b + 2.0 * d;
            ko[5] = a + c + e;
            bool bko;
            do
            {
                bko = false;
                DirectPolynomialRoots resol4 = new(ko[1], ko[2], ko[3], ko[4], ko[5]);
                if (!resol4.IsDone)
                {
                    _done = false;
                    return;
                }
                else if (!resol4.InfiniteRoots())
                {
                    nZer = resol4.SolutionCount;
                    for (i = 1; i <= nZer; i++)
                    {
                        zer[i] = resol4.Value(i);
                    }
                }
                else if (resol4.InfiniteRoots())
                {
                    _infiniteStatus = true;
                    return;
                }

                bool triok;
                do
                {
                    triok = true;
                    for (i = 1; i < nZer; i++)
                    {
                        if (zer[i] > zer[i + 1])
                        {
                            double t = zer[i];
                            zer[i] = zer[i + 1];
                            zer[i + 1] = t;
                            triok = false;
                        }
                    }
                } while (!triok);

                for (i = 1; i < nZer; i++)
                {
                    if (Math.Abs(zer[i + 1] - zer[i]) < eps)
                    {
                        //-- est ce une racine double ou une erreur numerique ?
                        double qw = zer[i + 1];
                        double va = ko[4] + qw * (2.0 * ko[3] + qw * (3.0 * ko[2] + qw * (4.0 * ko[1])));
                        if (Math.Abs(va) > eps)
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
        double supmInfs100 = (supBound - infBound) * 0.01;
        _nbSol = 0;
        for (i = 1; i <= nZer; i++)
        {
            teta = Math.Atan(zer[i]);
            teta += teta;
            if (zer[i] <= (-eps))
            {
                teta = depi - Math.Abs(teta);
            }

            teta += Math.Truncate(mod) * depi;
            if (teta - myBorneInf < 0)
            {
                teta += depi;
            }

            x = teta - myBorneInf;
            if ((x >= (-Epsilon(delta))) && (x <= delta + Epsilon(delta)))
            {
                x = teta;

                // Appel de Newton:
                // OCC541(apo):  double TetaNewton=0;
                double tetaNewton = teta;
                TrigonometricEquationFunction myF = new(a, b, c, d, e);
                NewtonFunctionRoot resol = new(myF, x, tol1, eps, nit);
                if (resol.IsDone)
                {
                    tetaNewton = resol.Root();
                }

                //-- lbr le 7 mars 97 (newton converge tres tres loin de la solution initilale)
                double deltaNewton = tetaNewton - teta;
                if ((deltaNewton > supmInfs100) || (deltaNewton < -supmInfs100))
                {
                    //-- std::cout<<"\n Newton X0="<<Teta<<" -> "<<TetaNewton<<std::endl;
                }
                else
                {
                    teta = tetaNewton;
                }

                flag4 = false;

                for (k = 1; k <= _nbSol; k++)
                {
                    // On met les valeurs par ordre croissant:
                    if (teta < _sol[k])
                    {
                        for (l = k; l <= _nbSol; l++)
                        {
                            j = _nbSol - l + k;
                            _sol[j + 1] = _sol[j];
                        }

                        _sol[k] = teta;
                        _nbSol++;
                        flag4 = true;
                        break;
                    }
                }

                if (!flag4)
                {
                    _nbSol++;
                    _sol[_nbSol] = teta;
                }
            }
        }

        // Cas particulier de  PI:
        if (_nbSol < 4)
        {
            int startIndex = _nbSol + 1;
            // OCCT checks for PI being already a solution on the first pass only, so when PI is a root that was not
            // found above, it is added once per free slot (4 - NbSol times).
            for (int solIt = startIndex; solIt <= 4; solIt++)
            {
                teta = M_PI + Math.Truncate(mod) * 2.0 * M_PI;
                x = teta - myBorneInf;
                if ((x >= (-Epsilon(delta))) && (x <= delta + Epsilon(delta)))
                {
                    if (Math.Abs(a - c + e) <= eps)
                    {
                        flag4 = false;
                        for (k = 1; k <= _nbSol; k++)
                        {
                            j = k;
                            if (teta < _sol[k])
                            {
                                flag4 = true;
                                break;
                            }

                            if ((solIt == startIndex) && (Math.Abs(teta - _sol[k]) <= eps))
                            {
                                return;
                            }
                        }

                        if (!flag4)
                        {
                            _nbSol++;
                            _sol[_nbSol] = teta;
                        }
                        else
                        {
                            for (k = j; k <= _nbSol; k++)
                            {
                                i = _nbSol - k + j;
                                _sol[i + 1] = _sol[i];
                            }

                            _sol[j] = teta;
                            _nbSol++;
                        }
                    }
                }
            }
        }
    }
}
