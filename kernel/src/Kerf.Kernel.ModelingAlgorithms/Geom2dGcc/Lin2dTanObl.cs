// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2dTanObl.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Lin2dTanObl.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// 2D lines tangent to a qualified curve and making a given angle with a line. A circle goes to GccAna_Lin2dTanObl;
/// other curves (lines included) to Geom2dGcc_Lin2dTanOblIter, from 21 sample parameters or from a given one.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117). Up to 2 solutions. OCCT declares IsParallel2
/// without defining it (V8_0_1); it is left out.
/// </remarks>
internal sealed class Lin2dTanObl
{
    private readonly bool _wellDone;
    // The solutions parallel to the line, which have no intersection with it (OCCT's Paral2 was always false and read
    // only by the undefined IsParallel2)
    private readonly bool[] _parallel = new bool[3];
    private readonly int _nbrSol;
    private readonly NCollection_Array1<Line2D> _linsol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<Point2D> _pntint2sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _par2sol;
    private readonly NCollection_Array1<double> _pararg1;
    private readonly NCollection_Array1<double> _pararg2;

    /// <summary>
    /// Lines tangent to the qualified curve at an angle Angle (radians) to TheLin; TolAng is the angular tolerance.
    /// </summary>
    public Lin2dTanObl(in QualifiedCurve qualified1, in Line2D theLine, double tolAng, double angle)
    {
        _linsol     = new(1, 2);
        _qualifier1 = new(1, 2);
        _pnttg1sol  = new(1, 2);
        _pntint2sol = new(1, 2);
        _par1sol    = new(1, 2);
        _par2sol    = new(1, 2);
        _pararg1    = new(1, 2);
        _pararg2    = new(1, 2);

        CurveAdaptor2D C1    = qualified1.Qualified();
        Curve2D?       cc1   = C1.Curve();
        CurveKind   type1 = C1.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        _wellDone = false;
        _nbrSol   = 0;
        if (type1 == Circle)
        {
            CircleCurve2D        ccc1 = (cc1 as CircleCurve2D)!;
            Circle2D            c1   = ccc1.Circle;
            QualifiedCirc qc1  = new(c1, qualified1.Qualifier());
            AnalyticLin2dTanObl   lin  = new(qc1, theLine, angle);
            _wellDone                  = lin.IsDone;
            if (_wellDone)
            {
                _nbrSol = lin.SolutionCount;
                for (int i = 1; i <= _nbrSol; i++)
                {
                    _linsol[i] = lin.ThisSolution(i);
                    lin.Tangency1(i, out _par1sol[i], out _pararg1[i], out _pnttg1sol[i]);
                    CopyIntersection(lin, i);
                    lin.WhichQualifier(i, out _qualifier1[i]);
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
                Lin2dTanOblIter lin = new(qc1, theLine, param1, tolAng, angle);

                if (lin.IsDone)
                {
                    if (Add(_nbrSol + 1, lin, tolAng, C1))
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
    /// The line tangent to the qualified curve at an angle Angle to TheLin, found from the parameter Param1 on the
    /// curve (a circle still goes to GccAna).
    /// </summary>
    public Lin2dTanObl(in QualifiedCurve qualified1, in Line2D theLine, double tolAng, double param1,
                                 double angle)
    {
        _linsol     = new(1, 2);
        _qualifier1 = new(1, 2);
        _pnttg1sol  = new(1, 2);
        _pntint2sol = new(1, 2);
        _par1sol    = new(1, 2);
        _par2sol    = new(1, 2);
        _pararg1    = new(1, 2);
        _pararg2    = new(1, 2);

        CurveAdaptor2D C1    = qualified1.Qualified();
        Curve2D?       cc1   = C1.Curve();
        CurveKind   type1 = C1.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        _wellDone = false;
        _nbrSol   = 0;
        if (type1 == Circle)
        {
            CircleCurve2D        ccc1 = (cc1 as CircleCurve2D)!;
            Circle2D            c1   = ccc1.Circle;
            QualifiedCirc qc1  = new(c1, qualified1.Qualifier());
            AnalyticLin2dTanObl   lin  = new(qc1, theLine, angle);
            _wellDone                  = lin.IsDone;
            if (_wellDone)
            {
                _nbrSol = lin.SolutionCount;
                for (int i = 1; i <= _nbrSol; i++)
                {
                    _linsol[i] = lin.ThisSolution(i);
                    lin.Tangency1(i, out _par1sol[i], out _pararg1[i], out _pnttg1sol[i]);
                    CopyIntersection(lin, i);
                    lin.WhichQualifier(i, out _qualifier1[i]);
                }
            }
        }
        else
        {
            QCurve qc1 = new(C1, qualified1.Qualifier());
            // Param1 is the start parameter and TolAng the angular tolerance (OCCT swapped them: Kerf decision 1544)
            Lin2dTanOblIter lin = new(qc1, theLine, param1, tolAng, angle);
            _wellDone                      = lin.IsDone;
            if (_wellDone)
            {
                _linsol[1] = lin.ThisSolution();
                lin.Tangency1(out _par1sol[1], out _pararg1[1], out _pnttg1sol[1]);
                // kept if it is tangent to the curve, as the sampling constructor's Add checks
                ConstraintCurveTool.D1(C1, _pararg1[1], out _, out Vector2D vTan);
                if (vTan.Magnitude <= 0.0 || Math.Abs(_linsol[1].Direction.Crossed(new Direction2D(vTan))) > tolAng)
                {
                    _wellDone = false;
                    return;
                }
                _nbrSol = 1;
                try
                {
                    lin.Intersection2(out _par2sol[1], out _pararg2[1], out _pntint2sol[1]);
                }
                catch (IsParallelException)
                {
                    _parallel[1] = true;
                }
                lin.WhichQualifier(out _qualifier1[1]);
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

    /// <summary>The qualifier of the curve for the solution of index Index.</summary>
    public void WhichQualifier(int index, out Qualifier qualif1)
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
        }
    }

    /// <summary>The tangency point between the solution of index Index and the curve, and its parameters.</summary>
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

    /// <summary>The intersection point between the solution of index Index and the line, and its parameters.</summary>
    /// <exception cref="IsParallelException">The solution is parallel to the line.</exception>
    public void Intersection2(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else if (_parallel[index])
        {
            throw new IsParallelException();
        }
        else
        {
            parSol = _par2sol[index];
            parArg = _pararg2[index];
            pntSol = _pntint2sol[index];
        }
    }

    // IsParallel2 is declared in OCCT V8_0_1's header but never defined: left out.

    /// <summary>Copies GccAna's intersection of solution i, or marks the solution parallel to the line.</summary>
    private void CopyIntersection(AnalyticLin2dTanObl lin, int i)
    {
        try
        {
            lin.Intersection2(i, out _par2sol[i], out _pararg2[i], out _pntint2sol[i]);
        }
        catch (IsParallelException)
        {
            _parallel[i] = true;
        }
    }

    private bool Add(int index, Lin2dTanOblIter lin, double tol, CurveAdaptor2D c1)
    {
        int      i;
        Line2D aLin = lin.ThisSolution();

        lin.Tangency1(out double par1sol, out double par1arg, out Point2D pnt1Sol);
        // A solution parallel to the line (the angle 0 or pi) has no intersection (OCCT let the Iter's IsParallel
        // escape the constructor: Kerf decision 1544)
        double par2sol = 0, par2arg = 0;
        Point2D pnt2Sol = new();
        bool isParallel = false;
        try
        {
            lin.Intersection2(out par2sol, out par2arg, out pnt2Sol);
        }
        catch (IsParallelException)
        {
            isParallel = true;
        }

        for (i = 1; i < index; i++)
        {
            if (Math.Abs(par1arg - _pararg1[i]) <= tol && (isParallel || Math.Abs(par2arg - _pararg2[i]) <= tol))
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

        _linsol[index]     = aLin;
        _par1sol[index]    = par1sol;
        _pararg1[index]    = par1arg;
        _pnttg1sol[index]  = pnt1Sol;
        _par2sol[index]    = par2sol;
        _pararg2[index]    = par2arg;
        _pntint2sol[index] = pnt2Sol;
        _parallel[index]   = isParallel;

        lin.WhichQualifier(out _qualifier1[index]);

        return true;
    }
}
