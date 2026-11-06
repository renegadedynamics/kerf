// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanCen.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanCen.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Objects;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// 2D circles tangent to a qualified curve and centred on a point. Lines and circles go to GccAna_Circ2dTanCen, other
/// curves to Geom2dGcc_Circ2dTanCenGeo.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). Up to 2 solutions.</remarks>
internal sealed class Circ2dTanCen
{
    private readonly bool _wellDone;
    private readonly int _nbrSol;
    private readonly NCollection_Array1<Circle2D> _cirsol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<int> _theSame1;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _pararg1;

    /// <summary>Circles centred on PCenter tangent to the qualified curve.</summary>
    public Circ2dTanCen(in QualifiedCurve qualified1, Point2 pCenter, double tolerance)
    {
        _cirsol     = new(1, 2);
        _qualifier1 = new(1, 2);
        _theSame1   = new(1, 2);
        _pnttg1sol  = new(1, 2);
        _par1sol    = new(1, 2);
        _pararg1    = new(1, 2);

        CurveAdaptor2D C1    = qualified1.Qualified();
        Curve2D?       cc1   = C1.Curve();
        CurveKind   type1 = C1.GetType();

        //=============================================================================
        //                            Appel a GccAna.                                 +
        //=============================================================================

        Point2D pcenter = pCenter.Pnt2d();
        _nbrSol           = 0;
        if ((type1 == Line || type1 == Circle))
        {
            if (type1 == Circle)
            {
                CircleCurve2D        ccc1 = (cc1 as CircleCurve2D)!;
                Circle2D            c1   = ccc1.Circle;
                QualifiedCirc qc1  = new(c1, qualified1.Qualifier());
                AnalyticCirc2dTanCen  circ = new(qc1, pcenter, tolerance);
                _wellDone                  = circ.IsDone;
                _nbrSol                    = circ.SolutionCount;
                for (int j = 1; j <= _nbrSol; j++)
                {
                    _cirsol[j] = circ.ThisSolution(j);
                    circ.WhichQualifier(j, out _qualifier1[j]);
                    if (circ.IsTheSame1(j))
                    {
                        _theSame1[j] = 1;
                    }
                    else
                    {
                        _theSame1[j] = 0;
                    }
                    circ.Tangency1(j, out _par1sol[j], out _pararg1[j], out _pnttg1sol[j]);
                }
            }
            else
            {
                LineCurve2D         ll1  = (cc1 as LineCurve2D)!;
                Line2D            l1   = ll1.Line;
                AnalyticCirc2dTanCen circ = new(l1, pcenter);
                _wellDone                 = circ.IsDone;
                _nbrSol                   = circ.SolutionCount;
                for (int j = 1; j <= _nbrSol; j++)
                {
                    _cirsol[j] = circ.ThisSolution(j);
                    circ.WhichQualifier(j, out _qualifier1[j]);
                    if (circ.IsTheSame1(j))
                    {
                        _theSame1[j] = 1;
                    }
                    else
                    {
                        _theSame1[j] = 0;
                    }
                    circ.Tangency1(j, out _par1sol[j], out _pararg1[j], out _pnttg1sol[j]);
                }
            }
        }

        //=============================================================================
        //                            Appel a GccGeo.                                 +
        //=============================================================================

        else
        {
            QCurve          qc1  = new(C1, qualified1.Qualifier());
            Circ2dTanCenGeo circ = new(qc1, pcenter, tolerance);
            _wellDone                       = circ.IsDone;
            _nbrSol                         = circ.SolutionCount;
            for (int j = 1; j <= _nbrSol; j++)
            {
                _cirsol[j]   = circ.ThisSolution(j);
                _theSame1[j] = 0;
                circ.Tangency1(j, out _par1sol[j], out _pararg1[j], out _pnttg1sol[j]);
                circ.WhichQualifier(j, out _qualifier1[j]);
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
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in 1..NbSolutions().</exception>
    public Circle2D ThisSolution(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        return _cirsol[index];
    }

    /// <summary>The qualifier of the argument for the solution of index Index.</summary>
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

    /// <summary>The tangency point between the solution of index Index and the argument, and its parameters.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the argument itself.</exception>
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
            if (_theSame1[index] == 0)
            {
                parSol = _par1sol[index];
                parArg = _pararg1[index];
                pntSol = _pnttg1sol[index];
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>True if the solution of index Index is the argument itself.</summary>
    public bool IsTheSame1(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        if (_theSame1[index] == 0)
        {
            return false;
        }
        return true;
    }
}
