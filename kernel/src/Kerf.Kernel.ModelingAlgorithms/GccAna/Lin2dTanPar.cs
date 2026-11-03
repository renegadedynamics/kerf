// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dTanPar.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Lin2dTanPar.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// 2D lines parallel to a line and passing through a point (one solution) or tangent to a circle (two when
/// unqualified, one when enclosing or outside).
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117).</remarks>
public sealed class Lin2dTanPar
{
    private bool _wellDone;
    private int _nbrSol;
    private readonly NCollection_Array1<Line2D> _linsol;
    private readonly NCollection_Array1<Qualifier> _qualifier1;
    private readonly NCollection_Array1<Point2D> _pnttg1sol;
    private readonly NCollection_Array1<double> _par1sol;
    private readonly NCollection_Array1<double> _pararg1;

    //========================================================================
    //   Passing by a point :                                              +
    //   Create the straight line with origin ThePoint and                             +
    //                          direction Lin1.Direction().                    +
    //========================================================================

    /// <summary>The line through ThePoint parallel to Lin1.</summary>
    public Lin2dTanPar(in Point2D thePoint, in Line2D lin1)
    {
        _linsol = new NCollection_Array1<Line2D>(1, 1);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, 1);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, 1);
        _par1sol = new NCollection_Array1<double>(1, 1);
        _pararg1 = new NCollection_Array1<double>(1, 1);

        _linsol[1] = new Line2D(thePoint, lin1.Direction);
        // ===============================================
        _qualifier1[1] = Qualifier.Noqualifier;
        _pnttg1sol[1] = thePoint;
        _par1sol[1] = 0.0;
        _pararg1[1] = 0.0;
        _nbrSol = 1;
        _wellDone = true;
    }

    //========================================================================
    //   Tangent to a circle :                                               +
    //   Create following the qualifier the straight line                    +
    //          - with origin P1 (P1 is a point of intersection between C1 and +
    //                       a straight line passing by the center of C1 and  +
    //                         direction the normal to Lin1).                 +
    //                         the choice of the point of intersection depends +
    //                         on the qualifier.                            +
    //          - with direction the direction of Lin1.                         +
    //========================================================================

    /// <summary>The lines tangent to the qualified circle and parallel to Lin1.</summary>
    /// <exception cref="BadQualifierException">The circle is qualified enclosed.</exception>
    public Lin2dTanPar(in QualifiedCirc qualified1, in Line2D lin1)
    {
        _linsol = new NCollection_Array1<Line2D>(1, 2);
        _qualifier1 = new NCollection_Array1<Qualifier>(1, 2);
        _pnttg1sol = new NCollection_Array1<Point2D>(1, 2);
        _par1sol = new NCollection_Array1<double>(1, 2);
        _pararg1 = new NCollection_Array1<double>(1, 2);

        _wellDone = false;
        int signe = 0;
        int nbsol = 0;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D c1 = qualified1.Qualified();
        double xdir = (lin1.Direction).X;
        double ydir = (lin1.Direction).Y;

        if (qualified1.IsEnclosed)
        {
            throw new BadQualifierException();
        }
        // ============================
        else if (qualified1.IsEnclosing)
        {
            // ==================================
            nbsol = 1;
            signe = 1;
            _qualifier1[1] = Qualifier.Enclosing;
        }
        else if (qualified1.IsOutside)
        {
            // ===================================
            nbsol = 1;
            signe = -1;
            _qualifier1[1] = Qualifier.Outside;
        }
        else
        {
            nbsol = 2;
            signe = -1;
            _qualifier1[1] = Qualifier.Outside;
            _qualifier1[2] = Qualifier.Enclosing;
        }
        Xy xy = new(-c1.Radius * ydir, c1.Radius * xdir);
        for (int j = 1; j <= nbsol; j++)
        {
            signe = -signe;
            _nbrSol++;
            _linsol[_nbrSol] = new Line2D(new Point2D((c1.Location.XY()).Added(signe * xy)),
                                          //   =========================================================================
                                          lin1.Direction);
            //                             =================
            _pnttg1sol[_nbrSol] = new Point2D((c1.Location.XY()).Added(signe * xy));
            _par1sol[_nbrSol] = 0.0;
            // a circle of radius 0 puts the tangency point at its centre
            _pararg1[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
            _wellDone = true;
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
            if (!_wellDone)
            {
                throw new NotDoneException();
            }
            return _nbrSol;
        }
    }

    /// <summary>The solution number Index.</summary>
    public Line2D ThisSolution(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        return _linsol[index];
    }

    /// <summary>
    /// The qualifier of the argument for the solution number Index: the one given, the one computed for an unqualified
    /// circle (enclosing or outside), or GccEnt_noqualifier for a point.
    /// </summary>
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

    /// <summary>The tangency point of the solution number Index with the argument, and its parameters on both.</summary>
    public void Tangency1(int index, out double parSol, out double parArg, out Point2D pnt)
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
            pnt = _pnttg1sol[index];
        }
    }
}
