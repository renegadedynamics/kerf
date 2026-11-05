// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2d2Tan.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2d2Tan.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// 2D lines tangent to two qualified curves, or tangent to a qualified curve and through a point. Two circles (or a
/// circle and a point) go to GccAna_Lin2d2Tan; other arguments to Geom2dGcc_Lin2d2TanIter, from 21 sample parameters
/// on each curve or from given ones.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). Up to 4 solutions (2 with a point).</remarks>
internal sealed class Lin2d2Tan
{
    private readonly bool _wellDone;
    private readonly int _nbrSol;
    private readonly NCollection_Array1<Line2D> _linsol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<Qualifier> _qualifier2;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<Point2D> _pnttg2sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _par2sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<double> _pararg2;

    // The fields' initialiser list: arrays of nbSol elements.
    private Lin2d2Tan(int nbSol)
    {
        _linsol     = new(1, nbSol);
        _qualifier1 = new(1, nbSol);
        _qualifier2 = new(1, nbSol);
        _pnttg1sol  = new(1, nbSol);
        _pnttg2sol  = new(1, nbSol);
        _par1sol    = new(1, nbSol);
        _par2sol    = new(1, nbSol);
        _pararg1    = new(1, nbSol);
        _pararg2    = new(1, nbSol);
    }

    /// <summary>Lines tangent to two qualified curves. Tolang is the angular tolerance.</summary>
    public Lin2d2Tan(in QualifiedCurve qualified1, in QualifiedCurve qualified2, double tolang)
        : this(4)
    {
        CurveAdaptor2D C1    = qualified1.Qualified();
        CurveAdaptor2D C2    = qualified2.Qualified();
        Curve2D?       cc1   = C1.Curve();
        Curve2D?       cc2   = C2.Curve();
        CurveKind   type1 = C1.GetType();
        CurveKind   type2 = C2.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        _nbrSol = 0;
        if (type1 == Circle && type2 == Circle)
        {
            CircleCurve2D        ccc1 = (cc1 as CircleCurve2D)!;
            Circle2D            c1   = ccc1.Circle;
            CircleCurve2D        ccc2 = (cc2 as CircleCurve2D)!;
            Circle2D            c2   = ccc2.Circle;
            QualifiedCirc qc1  = new(c1, qualified1.Qualifier());
            QualifiedCirc qc2  = new(c2, qualified2.Qualifier());
            AnalyticLin2d2Tan     lin  = new(qc1, qc2, tolang);
            _wellDone                  = lin.IsDone;
            if (_wellDone)
            {
                _nbrSol = lin.SolutionCount;
                for (int i = 1; i <= _nbrSol; i++)
                {
                    _linsol[i] = lin.ThisSolution(i);
                    lin.Tangency1(i, out _par1sol[i], out _pararg1[i], out _pnttg1sol[i]);
                    lin.Tangency2(i, out _par2sol[i], out _pararg2[i], out _pnttg2sol[i]);
                    lin.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                }
            }
        }
        else
        {
            QCurve qc1         = new(C1, qualified1.Qualifier());
            double           a1FPar      = ConstraintCurveTool.FirstParameter(C1);
            double           a1LPar      = ConstraintCurveTool.LastParameter(C1);
            int              nbSamples1 = ConstraintCurveTool.NbSamples(C1);
            double           step1      = (a1LPar - a1FPar) / nbSamples1;
            double           param1      = a1FPar;
            QCurve qc2         = new(C2, qualified2.Qualifier());
            double           a2FPar      = ConstraintCurveTool.FirstParameter(C2);
            double           a2LPar      = ConstraintCurveTool.LastParameter(C2);
            int              nbSamples2 = ConstraintCurveTool.NbSamples(C2);
            double           step2      = (a2LPar - a2FPar) / nbSamples2;
            double           param2;
            int              i;
            int              j;

            for (i = 0; i <= nbSamples1 && _nbrSol < 4; i++)
            {
                param2 = a2FPar;

                for (j = 0; j <= nbSamples2 && _nbrSol < 4; j++)
                {
                    Lin2d2TanIter lin = new(qc1, qc2, param1, param2, tolang);

                    if (lin.IsDone)
                    {
                        if (Add(_nbrSol + 1, lin, tolang, C1, C2))
                        {
                            _nbrSol++;
                        }
                    }
                    param2 += step2;
                }
                param1 += step1;
            }

            _wellDone = (_nbrSol > 0);
        }
    }

    /// <summary>Lines tangent to a qualified curve through a point. Tolang is the angular tolerance.</summary>
    public Lin2d2Tan(in QualifiedCurve qualified1, in Point2D thePoint, double tolang)
        : this(2)
    {
        CurveAdaptor2D C1    = qualified1.Qualified();
        Curve2D?       cc1   = C1.Curve();
        CurveKind   type1 = C1.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        _nbrSol = 0;
        if (type1 == Circle)
        {
            CircleCurve2D        ccc1 = (cc1 as CircleCurve2D)!;
            Circle2D            c1   = ccc1.Circle;
            QualifiedCirc qc1  = new(c1, qualified1.Qualifier());
            AnalyticLin2d2Tan     lin  = new(qc1, thePoint, tolang);
            _wellDone                  = lin.IsDone;

            if (_wellDone)
            {
                _nbrSol = lin.SolutionCount;
                for (int i = 1; i <= _nbrSol; i++)
                {
                    _linsol[i] = lin.ThisSolution(i);
                    lin.Tangency1(i, out _par1sol[i], out _pararg1[i], out _pnttg1sol[i]);
                    lin.Tangency2(i, out _par2sol[i], out _pararg2[i], out _pnttg2sol[i]);
                    lin.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                }
            }
        }
        else
        {
            QCurve qc1        = new(C1, qualified1.Qualifier());
            double           firstPar  = ConstraintCurveTool.FirstParameter(C1);
            double           lastPar   = ConstraintCurveTool.LastParameter(C1);
            int              nbSamples = ConstraintCurveTool.NbSamples(C1);
            double           step      = (lastPar - firstPar) / nbSamples;
            double           param1     = firstPar;
            int              i;

            for (i = 0; i <= nbSamples && _nbrSol < 2; i++)
            {
                Lin2d2TanIter lin = new(qc1, thePoint, param1, tolang);

                if (lin.IsDone)
                {
                    if (Add(_nbrSol + 1, lin, tolang, C1, new CurveAdaptor2D()))
                    {
                        _nbrSol++;
                    }
                }

                param1 += step;
            }

            _wellDone = (_nbrSol > 0);
        }
    }

    /// <summary>
    /// The line tangent to two qualified curves found from the parameters Param1 and Param2 on them (two circles still
    /// go to GccAna).
    /// </summary>
    public Lin2d2Tan(in QualifiedCurve qualified1, in QualifiedCurve qualified2, double tolang,
                               double param1, double param2)
        : this(4)
    {
        CurveAdaptor2D C1    = qualified1.Qualified();
        CurveAdaptor2D C2    = qualified2.Qualified();
        Curve2D?       cc1   = C1.Curve();
        Curve2D?       cc2   = C2.Curve();
        CurveKind   type1 = C1.GetType();
        CurveKind   type2 = C2.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        _nbrSol = 0;
        if (type1 == Circle && type2 == Circle)
        {
            CircleCurve2D        ccc1 = (cc1 as CircleCurve2D)!;
            Circle2D            c1   = ccc1.Circle;
            CircleCurve2D        ccc2 = (cc2 as CircleCurve2D)!;
            Circle2D            c2   = ccc2.Circle;
            QualifiedCirc qc1  = new(c1, qualified1.Qualifier());
            QualifiedCirc qc2  = new(c2, qualified2.Qualifier());
            AnalyticLin2d2Tan     lin  = new(qc1, qc2, tolang);
            _wellDone                  = lin.IsDone;
            if (_wellDone)
            {
                _nbrSol = lin.SolutionCount;
                for (int i = 1; i <= _nbrSol; i++)
                {
                    _linsol[i] = lin.ThisSolution(i);
                    lin.Tangency1(i, out _par1sol[i], out _pararg1[i], out _pnttg1sol[i]);
                    lin.Tangency2(i, out _par2sol[i], out _pararg2[i], out _pnttg2sol[i]);
                    lin.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                }
            }
        }
        else
        {
            QCurve        qc1 = new(C1, qualified1.Qualifier());
            QCurve        qc2 = new(C2, qualified2.Qualifier());
            Lin2d2TanIter lin = new(qc1, qc2, param1, param2, tolang);
            _wellDone                    = lin.IsDone;
            if (_wellDone)
            {
                _nbrSol    = 1;
                _linsol[1] = lin.ThisSolution();
                lin.Tangency1(out _par1sol[1], out _pararg1[1], out _pnttg1sol[1]);
                lin.Tangency2(out _par2sol[1], out _pararg2[1], out _pnttg2sol[1]);
                lin.WhichQualifier(out _qualifier1[1], out _qualifier2[1]);
            }
        }
    }

    /// <summary>
    /// The line tangent to a qualified curve through a point found from the parameter Param1 on the curve (a circle
    /// still goes to GccAna).
    /// </summary>
    public Lin2d2Tan(in QualifiedCurve qualified1, in Point2D thePoint, double tolang, double param1)
        : this(2)
    {
        CurveAdaptor2D C1    = qualified1.Qualified();
        Curve2D?       cc1   = C1.Curve();
        CurveKind   type1 = C1.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        _nbrSol = 0;
        if (type1 == Circle)
        {
            CircleCurve2D        ccc1 = (cc1 as CircleCurve2D)!;
            Circle2D            c1   = ccc1.Circle;
            QualifiedCirc qc1  = new(c1, qualified1.Qualifier());
            AnalyticLin2d2Tan     lin  = new(qc1, thePoint, tolang);
            _wellDone                  = lin.IsDone;
            if (_wellDone)
            {
                _nbrSol = lin.SolutionCount;
                for (int i = 1; i <= _nbrSol; i++)
                {
                    _linsol[i] = lin.ThisSolution(i);
                    lin.Tangency1(i, out _par1sol[i], out _pararg1[i], out _pnttg1sol[i]);
                    lin.Tangency2(i, out _par2sol[i], out _pararg2[i], out _pnttg2sol[i]);
                    lin.WhichQualifier(i, out _qualifier1[i], out _qualifier2[i]);
                }
            }
        }
        else
        {
            QCurve        qc1 = new(C1, qualified1.Qualifier());
            Lin2d2TanIter lin = new(qc1, thePoint, param1, tolang);
            _wellDone                    = lin.IsDone;
            if (_wellDone)
            {
                _nbrSol    = 1;
                _linsol[1] = lin.ThisSolution();
                lin.Tangency1(out _par1sol[1], out _pararg1[1], out _pnttg1sol[1]);
                lin.Tangency2(out _par2sol[1], out _pararg2[1], out _pnttg2sol[1]);
                lin.WhichQualifier(out _qualifier1[1], out _qualifier2[1]);
            }
        }
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>The number of solutions.</summary>
    public int SolutionCount
    {
        get
        {
            return _nbrSol;
        }
    }

    /// <summary>The solution of index Index.</summary>
    /// <exception cref="OutOfRangeException">Index is not in 1..NbSolutions().</exception>
    public Line2D ThisSolution(int index)
    {
        if (index > _nbrSol || index <= 0)
        {
            throw new OutOfRangeException();
        }
        return _linsol[index];
    }

    /// <summary>The qualifiers of the arguments for the solution of index Index.</summary>
    public void WhichQualifier(int index, out Qualifier qualif1, out Qualifier qualif2)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            qualif1 = _qualifier1[index];
            qualif2 = _qualifier2[index];
        }
    }

    /// <summary>The tangency point between the solution of index Index and the first argument, and its parameters.</summary>
    public void Tangency1(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            parSol = _par1sol[index];
            parArg = _pararg1[index];
            pntSol = _pnttg1sol[index];
        }
    }

    /// <summary>The tangency point between the solution of index Index and the second argument, and its parameters.</summary>
    public void Tangency2(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            parSol = _par2sol[index];
            parArg = _pararg2[index];
            pntSol = _pnttg2sol[index];
        }
    }

    private bool Add(int index, Lin2d2TanIter lin, double tol, CurveAdaptor2D c1,
                     CurveAdaptor2D c2)
    {
        int      i;
        Line2D aLin = lin.ThisSolution();

        lin.Tangency1(out double par1sol, out double par1arg, out Point2D pnt1Sol);
        lin.Tangency2(out double par2sol, out double par2arg, out Point2D pnt2Sol);

        for (i = 1; i < index; i++)
        {
            if (Math.Abs(par1arg - _pararg1[i]) <= tol && Math.Abs(par2arg - _pararg2[i]) <= tol)
            {
                return false;
            }
        }

        Direction2D linDir = aLin.Direction;

        ConstraintCurveTool.D1(c1, par1arg, out _, out Vector2D vTan);

        if (Math.Abs(linDir.Crossed(new Direction2D(vTan))) > tol)
        {
            return false;
        }

        if (c2.Curve() is not null)
        {
            ConstraintCurveTool.D1(c2, par2arg, out _, out vTan);

            if (Math.Abs(linDir.Crossed(new Direction2D(vTan))) > tol)
            {
                return false;
            }
        }

        _linsol[index]    = aLin;
        _par1sol[index]   = par1sol;
        _pararg1[index]   = par1arg;
        _pnttg1sol[index] = pnt1Sol;
        _par2sol[index]   = par2sol;
        _pararg2[index]   = par2arg;
        _pnttg2sol[index] = pnt2Sol;

        lin.WhichQualifier(out _qualifier1[index], out _qualifier2[index]);

        return true;
    }
}
