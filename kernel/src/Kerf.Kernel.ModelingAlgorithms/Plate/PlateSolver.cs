// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_Plate.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_Plate.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_Plate.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// A plate: the deformation of minimal bending energy (a thin-plate spline of order <c>ord</c>) of the UV plane that
/// meets pinpoint, linear and G1-G3 continuity constraints. Load the constraints, solve with <see cref="SolveTI"/>,
/// then evaluate the deformation and its derivatives at any 2D point.
/// </summary>
/// <remarks>
/// A working object (decision 21), as OCCT's: <c>Load</c>, <c>SolveTI</c>, <c>Init</c> and
/// <c>SetPolynomialPartOnly</c> change it. OCCT copies it by value; the port's copy is the copy constructor
/// <c>new Plate_Plate(other)</c> or <c>Copy(other)</c> (OCCT's <c>operator=</c>), which copy the solution and the
/// constraint lists (the linear constraints in the lists are shared, as their C++ copies share their arrays).
/// Evaluation doesn't change the plate (OCCT caches the last point's radius in it; the port doesn't, decision 870),
/// so a solved plate can be evaluated from several threads at once, as GeomPlate_Surface is.
/// </remarks>
internal class PlateSolver
{
    private int _order;
    private int _nEl;
    private int _nDim;
    private Xyz[]? _solution;
    private Xy[]? _points;
    private int[]? _deru;
    private int[]? _derv;
    private bool _ok;
    private readonly NCollection_Sequence<PinpointConstraint> _constraints = new();
    private readonly NCollection_Sequence<LinearXyzConstraint> _lxyzConstraints = new();
    private readonly NCollection_Sequence<LinearScalarConstraint> _lScalarConstraints = new();
    private readonly double[] _ddu = new double[10];
    private readonly double[] _ddv = new double[10];
    private int _maxConstraintOrder;
    private bool _polynomialPartOnly;
    // OCCT's SolEm caches the last point's U, V, U2, R and L in the plate (Uold, Vold, U2, R, L, through a
    // const_cast); the port computes them on each call (decision 870), which gives the same values.

    public PlateSolver()
    {
        _order = 0;
        _nEl = 0;
        _nDim = 0;
        _solution = null;
        _points = null;
        _deru = null;
        _derv = null;
        _ok = false;
        _maxConstraintOrder = 0;
        _polynomialPartOnly = false;
    }

    /// <summary>A copy of <paramref name="Ref"/> (OCCT's copy constructor).</summary>
    public PlateSolver(PlateSolver Ref)
    {
        _order = Ref._order;
        _nEl = Ref._nEl;
        _nDim = Ref._nDim;
        _solution = null;
        _points = null;
        _deru = null;
        _derv = null;
        _ok = Ref._ok;
        int i;
        if (Ref._ok)
        {
            if (_nDim > 0 && Ref._solution != null)
            {
                _solution = new Xyz[_nDim];
                for (i = 0; i < _nDim; i++)
                {
                    Solution(i) = Ref.Solution(i);
                }
            }

            if (_nEl > 0)
            {
                if (Ref._points != null)
                {
                    _points = new Xy[_nEl];
                    for (i = 0; i < _nEl; i++)
                    {
                        Points(i) = Ref.Points(i);
                    }
                }

                if (Ref._deru != null)
                {
                    _deru = new int[_nEl];
                    for (i = 0; i < _nEl; i++)
                    {
                        Deru(i) = Ref.Deru(i);
                    }
                }

                if (Ref._derv != null)
                {
                    _derv = new int[_nEl];
                    for (i = 0; i < _nEl; i++)
                    {
                        Derv(i) = Ref.Derv(i);
                    }
                }
            }
        }

        _constraints.Assign(Ref._constraints);
        _lxyzConstraints.Assign(Ref._lxyzConstraints);
        _lScalarConstraints.Assign(Ref._lScalarConstraints);
        _maxConstraintOrder = Ref._maxConstraintOrder;
        _polynomialPartOnly = Ref._polynomialPartOnly;
        for (i = 0; i < 10; i++)
        {
            _ddu[i] = Ref._ddu[i];
            _ddv[i] = Ref._ddv[i];
        }
    }

    /// <summary>Makes this plate a copy of <paramref name="Ref"/> (OCCT's <c>operator=</c>).</summary>
    public PlateSolver Copy(PlateSolver Ref)
    {
        Init();
        _order = Ref._order;
        _nEl = Ref._nEl;
        _nDim = Ref._nDim;
        _ok = Ref._ok;
        int i;
        if (Ref._ok)
        {
            if (_nDim > 0 && Ref._solution != null)
            {
                _solution = new Xyz[_nDim];
                for (i = 0; i < _nDim; i++)
                {
                    Solution(i) = Ref.Solution(i);
                }
            }

            if (_nEl > 0)
            {
                if (Ref._points != null)
                {
                    _points = new Xy[_nEl];
                    for (i = 0; i < _nEl; i++)
                    {
                        Points(i) = Ref.Points(i);
                    }
                }

                if (Ref._deru != null)
                {
                    _deru = new int[_nEl];
                    for (i = 0; i < _nEl; i++)
                    {
                        Deru(i) = Ref.Deru(i);
                    }
                }

                if (Ref._derv != null)
                {
                    _derv = new int[_nEl];
                    for (i = 0; i < _nEl; i++)
                    {
                        Derv(i) = Ref.Derv(i);
                    }
                }
            }
        }

        _constraints.Assign(Ref._constraints);
        _lxyzConstraints.Assign(Ref._lxyzConstraints);
        _lScalarConstraints.Assign(Ref._lScalarConstraints);
        _maxConstraintOrder = Ref._maxConstraintOrder;
        _polynomialPartOnly = Ref._polynomialPartOnly;

        for (i = 0; i < 10; i++)
        {
            _ddu[i] = Ref._ddu[i];
            _ddv[i] = Ref._ddv[i];
        }
        return this;
    }

    public void Load(in PinpointConstraint pConst)
    {
        _ok = false;
        _nEl++;
        _constraints.Append(pConst);
        int ordreConst = pConst.Idu() + pConst.Idv();
        if (_maxConstraintOrder < ordreConst)
        {
            _maxConstraintOrder = ordreConst;
        }
    }

    public void Load(LinearXyzConstraint lxyzConst)
    {
        _ok = false;
        _nEl += lxyzConst.Coeff().RowLength();

        _lxyzConstraints.Append(lxyzConst);
        for (int j = 1; j <= lxyzConst.GetPPC().Length(); j++)
        {
            int ordreConst = lxyzConst.GetPPC()[j].Idu() + lxyzConst.GetPPC()[j].Idv();
            if (_maxConstraintOrder < ordreConst)
            {
                _maxConstraintOrder = ordreConst;
            }
        }
    }

    public void Load(LinearScalarConstraint lScalarConst)
    {
        _ok = false;
        _nEl += lScalarConst.Coeff().RowLength();
        _lScalarConstraints.Append(lScalarConst);
        for (int j = 1; j <= lScalarConst.GetPPC().Length(); j++)
        {
            int ordreConst = lScalarConst.GetPPC()[j].Idu() + lScalarConst.GetPPC()[j].Idv();
            if (_maxConstraintOrder < ordreConst)
            {
                _maxConstraintOrder = ordreConst;
            }
        }
    }

    public void Load(LineConstraint lConst)
    {
        Load(lConst.LSC());
    }

    public void Load(PlaneConstraint pConst)
    {
        Load(pConst.LSC());
    }

    public void Load(SampledCurveConstraint scConst)
    {
        Load(scConst.LXYZC());
    }

    public void Load(GtoCConstraint gtoCConst)
    {
        for (int i = 0; i < gtoCConst.nb_PPC(); i++)
        {
            Load(gtoCConst.GetPPC(i));
        }
    }

    public void Load(FreeGtoCConstraint fGtoCConst)
    {
        int i;
        for (i = 0; i < fGtoCConst.nb_PPC(); i++)
        {
            Load(fGtoCConst.GetPPC(i));
        }
        for (i = 0; i < fGtoCConst.nb_LSC(); i++)
        {
            Load(fGtoCConst.LSC(i));
        }
    }

    public void Load(GlobalTranslationConstraint gtConst)
    {
        Load(gtConst.LXYZC());
    }

    public void SolveTI(int ord = 4, double anisotropie = 1.0, ProgressRange progress = default)
    {
        int iterationNumber = 0;
        _ok = false;
        _order = ord;
        if (ord <= 1)
        {
            return;
        }
        if (ord > 9)
        {
            return;
        }
        if (_nEl < 1)
        {
            return;
        }
        if (anisotropie < 1.0e-6)
        {
            return;
        }
        if (anisotropie > 1.0e+6)
        {
            return;
        }

        // computation of the bounding box of the 2d PPconstraints
        UVBox(out double xmin, out double xmax, out double ymin, out double ymax);

        double du = 0.5 * (xmax - xmin);
        if (anisotropie > 1.0)
        {
            du *= anisotropie;
        }
        if (du < 1.0e-10)
        {
            return;
        }
        _ddu[0] = 1;
        int i;
        for (i = 1; i <= 9; i++)
        {
            _ddu[i] = _ddu[i - 1] / du;
        }

        double dv = 0.5 * (ymax - ymin);
        if (anisotropie < 1.0)
        {
            dv /= anisotropie;
        }
        if (dv < 1.0e-10)
        {
            return;
        }
        _ddv[0] = 1;
        for (i = 1; i <= 9; i++)
        {
            _ddv[i] = _ddv[i - 1] / dv;
        }

        if (_lScalarConstraints.IsEmpty())
        {
            if (_lxyzConstraints.IsEmpty())
            {
                SolveTI1(iterationNumber, progress);
            }
            else
            {
                SolveTI2(iterationNumber, progress);
            }
        }
        else
        {
            SolveTI3(iterationNumber, progress);
        }
    }

    private void SolveTI1(int iterationNumber, ProgressRange progress)
    {
        // computation of square matrix members

        _nDim = _nEl + _order * (_order + 1) / 2;
        Matrix mat = new(0, _nDim - 1, 0, _nDim - 1, 0.0);
        _points = new Xy[_nEl];
        int i;
        for (i = 0; i < _nEl; i++)
        {
            Points(i) = _constraints[i + 1].Pnt2d();
        }
        _deru = new int[_nEl];
        for (i = 0; i < _nEl; i++)
        {
            Deru(i) = _constraints[i + 1].Idu();
        }
        _derv = new int[_nEl];
        for (i = 0; i < _nEl; i++)
        {
            Derv(i) = _constraints[i + 1].Idv();
        }

        for (i = 0; i < _nEl; i++)
        {
            for (int j = 0; j < i; j++)
            {
                double signe = 1;
                if (((Deru(j) + Derv(j)) % 2) == 1)
                {
                    signe = -1;
                }
                int iu = Deru(i) + Deru(j);
                int iv = Derv(i) + Derv(j);
                mat[i, j] = signe * SolEm(Points(i) - Points(j), iu, iv);
            }
        }

        i = _nEl;
        for (int iu = 0; iu < _order; iu++)
        {
            for (int iv = 0; iu + iv < _order; iv++)
            {
                for (int j = 0; j < _nEl; j++)
                {
                    int idu = Deru(j);
                    int idv = Derv(j);
                    mat[i, j] = Polm(Points(j), iu, iv, idu, idv);
                }
                i++;
            }
        }

        for (i = 0; i < _nDim; i++)
        {
            for (int j = i + 1; j < _nDim; j++)
            {
                mat[i, j] = mat[j, i];
            }
        }

        // initialisation of the Gauss algorithm
        double pivot_max = 1.0e-12;
        _ok = true;

        using var scope = new ProgressScope(progress, "Plate_Plate::SolveTI1()", 10);
        Gauss algo_gauss = new(mat, pivot_max, scope.Next(7));

        if (scope.UserBreak())
        {
            _ok = false;
            return;
        }

        if (!algo_gauss.IsDone)
        {
            int nbm = _order * (_order + 1) / 2;
            for (i = _nEl; i < _nEl + nbm; i++)
            {
                mat[i, i] = 1.0e-8;
            }
            pivot_max = 1.0e-18;

            Gauss thealgo = new(mat, pivot_max, scope.Next(3));

            if (scope.UserBreak())
            {
                _ok = false;
                return;
            }
            algo_gauss = thealgo;
            _ok = algo_gauss.IsDone;
        }

        if (_ok)
        {
            //   computation of the linear system solution for the X, Y and Z coordinates
            math_Vector sec_member = new(0, _nDim - 1, 0.0);
            math_Vector sol = new(0, _nDim - 1);
            _solution = new Xyz[_nDim];

            for (int icoor = 1; icoor <= 3; icoor++)
            {
                for (i = 0; i < _nEl; i++)
                {
                    sec_member[i] = _constraints[i + 1].Value().Coord(icoor);
                }
                algo_gauss.Solve(sec_member, sol);
                // alr iteration pour affiner la solution
                {
                    math_Vector sol1 = new(0, _nDim - 1);
                    math_Vector sec_member1 = new(0, _nDim - 1);
                    for (i = 1; i <= iterationNumber; i++)
                    {
                        sec_member1 = sec_member - mat * sol;
                        algo_gauss.Solve(sec_member1, sol1);
                        sol += sol1;
                    }
                }
                // finalr

                for (i = 0; i < _nDim; i++)
                {
                    Solution(i).SetCoord(icoor, sol[i]);
                }
            }
        }
    }

    private void SolveTI2(int iterationNumber, ProgressRange progress)
    {
        // computation of square matrix members

        int nCC1 = _constraints.Length();
        int nCC2 = 0;
        int i;
        for (i = 1; i <= _lxyzConstraints.Length(); i++)
        {
            nCC2 += _lxyzConstraints[i].Coeff().ColLength();
        }

        int n_dimat = nCC1 + nCC2 + _order * (_order + 1) / 2;
        _points = new Xy[_nEl];
        _deru = new int[_nEl];
        _derv = new int[_nEl];

        for (i = 0; i < nCC1; i++)
        {
            Points(i) = _constraints[i + 1].Pnt2d();
            Deru(i)   = _constraints[i + 1].Idu();
            Derv(i)   = _constraints[i + 1].Idv();
        }

        int k = nCC1;
        for (i = 1; i <= _lxyzConstraints.Length(); i++)
        {
            for (int j = 1; j <= _lxyzConstraints[i].GetPPC().Length(); j++)
            {
                Points(k) = _lxyzConstraints[i].GetPPC()[j].Pnt2d();
                Deru(k)   = _lxyzConstraints[i].GetPPC()[j].Idu();
                Derv(k)   = _lxyzConstraints[i].GetPPC()[j].Idv();
                k++;
            }
        }

        Matrix mat = new(0, n_dimat - 1, 0, n_dimat - 1, 0.0);

        FillXYZmatrix(mat, 0, 0, nCC1, nCC2);

        // initialisation of the Gauss algorithm
        double pivot_max = 1.0e-12;
        _ok = true; // ************ JHH

        using var scope = new ProgressScope(progress, "Plate_Plate::SolveTI2()", 10);
        Gauss algo_gauss = new(mat, pivot_max, scope.Next(7));

        if (scope.UserBreak())
        {
            _ok = false;
            return;
        }

        if (!algo_gauss.IsDone)
        {
            for (i = nCC1 + nCC2; i < n_dimat; i++)
            {
                mat[i, i] = 1.0e-8;
            }
            pivot_max = 1.0e-18;

            Gauss thealgo1 = new(mat, pivot_max, scope.Next(3));

            if (scope.UserBreak())
            {
                _ok = false;
                return;
            }
            algo_gauss = thealgo1;
            _ok = algo_gauss.IsDone;
        }

        if (_ok)
        {
            //   computation of the linear system solution for the X, Y and Z coordinates
            math_Vector sec_member = new(0, n_dimat - 1, 0.0);
            math_Vector sol = new(0, n_dimat - 1);
            _nDim = _nEl + _order * (_order + 1) / 2;
            _solution = new Xyz[_nDim];

            for (int icoor = 1; icoor <= 3; icoor++)
            {
                for (i = 0; i < nCC1; i++)
                {
                    sec_member[i] = _constraints[i + 1].Value().Coord(icoor);
                }

                k = nCC1;
                for (i = 1; i <= _lxyzConstraints.Length(); i++)
                {
                    for (int irow = 1; irow <= _lxyzConstraints[i].Coeff().ColLength(); irow++)
                    {
                        for (int icol = 1; icol <= _lxyzConstraints[i].Coeff().RowLength(); icol++)
                        {
                            sec_member[k] += _lxyzConstraints[i].Coeff()[irow, icol]
                                                              * _lxyzConstraints[i].GetPPC()[icol].Value().Coord(icoor);
                        }
                        k++;
                    }
                }

                algo_gauss.Solve(sec_member, sol);
                // alr iteration pour affiner la solution
                {
                    math_Vector sol1 = new(0, n_dimat - 1);
                    math_Vector sec_member1 = new(0, n_dimat - 1);
                    for (i = 1; i <= iterationNumber; i++)
                    {
                        sec_member1 = sec_member - mat * sol;
                        algo_gauss.Solve(sec_member1, sol1);
                        sol += sol1;
                    }
                }
                // finalr

                for (i = 0; i < nCC1; i++)
                {
                    Solution(i).SetCoord(icoor, sol[i]);
                }

                int kSolution = nCC1;
                int ksol = nCC1;

                for (i = 1; i <= _lxyzConstraints.Length(); i++)
                {
                    for (int icol = 1; icol <= _lxyzConstraints[i].Coeff().RowLength(); icol++)
                    {
                        double vsol = 0;
                        for (int irow = 1; irow <= _lxyzConstraints[i].Coeff().ColLength(); irow++)
                        {
                            vsol += _lxyzConstraints[i].Coeff()[irow, icol] * sol[ksol + irow - 1];
                        }
                        Solution(kSolution).SetCoord(icoor, vsol);
                        kSolution++;
                    }
                    ksol += _lxyzConstraints[i].Coeff().ColLength();
                }

                for (i = 0; i < _order * (_order + 1) / 2; i++)
                {
                    Solution(_nEl + i).SetCoord(icoor, sol[ksol + i]);
                }
            }
        }
    }

    private void SolveTI3(int iterationNumber, ProgressRange progress)
    {
        // computation of square matrix members

        int nCC1 = _constraints.Length();
        int j; // declared here: C# forbids OCCT's later declaration over the loops' own j

        int nCC2 = 0;
        int i;
        for (i = 1; i <= _lxyzConstraints.Length(); i++)
        {
            nCC2 += _lxyzConstraints[i].Coeff().ColLength();
        }

        int nCC3 = 0;
        for (i = 1; i <= _lScalarConstraints.Length(); i++)
        {
            nCC3 += _lScalarConstraints[i].Coeff().ColLength();
        }

        int nbm = _order * (_order + 1) / 2;
        int n_dimsousmat = nCC1 + nCC2 + nbm;
        int n_dimat = 3 * n_dimsousmat + nCC3;
        _points = new Xy[_nEl];
        _deru = new int[_nEl];
        _derv = new int[_nEl];

        for (i = 0; i < nCC1; i++)
        {
            Points(i) = _constraints[i + 1].Pnt2d();
            Deru(i)   = _constraints[i + 1].Idu();
            Derv(i)   = _constraints[i + 1].Idv();
        }

        int k = nCC1;
        for (i = 1; i <= _lxyzConstraints.Length(); i++)
        {
            for (j = 1; j <= _lxyzConstraints[i].GetPPC().Length(); j++)
            {
                Points(k) = _lxyzConstraints[i].GetPPC()[j].Pnt2d();
                Deru(k)   = _lxyzConstraints[i].GetPPC()[j].Idu();
                Derv(k)   = _lxyzConstraints[i].GetPPC()[j].Idv();
                k++;
            }
        }
        int nPPC2 = k;
        for (i = 1; i <= _lScalarConstraints.Length(); i++)
        {
            for (j = 1; j <= _lScalarConstraints[i].GetPPC().Length(); j++)
            {
                Points(k) = _lScalarConstraints[i].GetPPC()[j].Pnt2d();
                Deru(k)   = _lScalarConstraints[i].GetPPC()[j].Idu();
                Derv(k)   = _lScalarConstraints[i].GetPPC()[j].Idv();
                k++;
            }
        }

        Matrix mat = new(0, n_dimat - 1, 0, n_dimat - 1, 0.0);

        FillXYZmatrix(mat, 0, 0, nCC1, nCC2);
        FillXYZmatrix(mat, n_dimsousmat, n_dimsousmat, nCC1, nCC2);
        FillXYZmatrix(mat, 2 * n_dimsousmat, 2 * n_dimsousmat, nCC1, nCC2);

        k = 3 * n_dimsousmat;
        int kppc = nPPC2;
        for (i = 1; i <= _lScalarConstraints.Length(); i++)
        {
            for (j = 0; j < nCC1; j++)
            {

                math_Vector vmat = new(1, _lScalarConstraints[i].GetPPC().Length());

                for (int ippc = 1; ippc <= _lScalarConstraints[i].GetPPC().Length(); ippc++)
                {
                    double signe = 1;
                    if (((Deru(j) + Derv(j)) % 2) == 1)
                    {
                        signe = -1;
                    }
                    int iu = Deru(kppc + ippc - 1) + Deru(j);
                    int iv = Derv(kppc + ippc - 1) + Derv(j);
                    vmat[ippc] = signe * SolEm(Points(kppc + ippc - 1) - Points(j), iu, iv);
                }

                for (int irow = 1; irow <= _lScalarConstraints[i].Coeff().ColLength(); irow++)
                {
                    for (int icol = 1; icol <= _lScalarConstraints[i].Coeff().RowLength(); icol++)
                    {
                        mat[k + irow - 1, j] += _lScalarConstraints[i].Coeff()[irow, icol].X() * vmat[icol];
                        mat[k + irow - 1, n_dimsousmat + j] +=
                            _lScalarConstraints[i].Coeff()[irow, icol].Y() * vmat[icol];
                        mat[k + irow - 1, 2 * n_dimsousmat + j] +=
                            _lScalarConstraints[i].Coeff()[irow, icol].Z() * vmat[icol];
                    }
                }
            }

            int k2 = nCC1;
            int kppc2 = nCC1;
            int i2;
            for (i2 = 1; i2 <= _lxyzConstraints.Length(); i2++)
            {

                Matrix tmpmat = new(1,
                                                      _lScalarConstraints[i].GetPPC().Length(),
                                                      1,
                                                      _lxyzConstraints[i2].GetPPC().Length());

                for (int ippc = 1; ippc <= _lScalarConstraints[i].GetPPC().Length(); ippc++)
                {
                    for (int ippc2 = 1; ippc2 <= _lxyzConstraints[i2].GetPPC().Length(); ippc2++)
                    {
                        double signe = 1;
                        if (((Deru(kppc2 + ippc2 - 1) + Derv(kppc2 + ippc2 - 1)) % 2) == 1)
                        {
                            signe = -1;
                        }
                        int iu = Deru(kppc + ippc - 1) + Deru(kppc2 + ippc2 - 1);
                        int iv = Derv(kppc + ippc - 1) + Derv(kppc2 + ippc2 - 1);
                        tmpmat[ippc, ippc2] =
                            signe * SolEm(Points(kppc + ippc - 1) - Points(kppc2 + ippc2 - 1), iu, iv);
                    }
                }

                for (int irow = 1; irow <= _lScalarConstraints[i].Coeff().ColLength(); irow++)
                {
                    for (int irow2 = 1; irow2 <= _lxyzConstraints[i2].Coeff().ColLength(); irow2++)
                    {
                        for (int icol = 1; icol <= _lScalarConstraints[i].Coeff().RowLength(); icol++)
                        {
                            for (int icol2 = 1; icol2 <= _lxyzConstraints[i2].Coeff().RowLength(); icol2++)
                            {
                                mat[k + irow - 1, k2 + irow2 - 1] += _lScalarConstraints[i].Coeff()[irow, icol].X()
                                                                                                          * _lxyzConstraints[i2].Coeff()[irow2, icol2]
                                                                                                          * tmpmat[icol, icol2];
                                mat[k + irow - 1, n_dimsousmat + k2 + irow2 - 1] +=
                                    _lScalarConstraints[i].Coeff()[irow, icol].Y()
                                    * _lxyzConstraints[i2].Coeff()[irow2, icol2] * tmpmat[icol, icol2];
                                mat[k + irow - 1, 2 * n_dimsousmat + k2 + irow2 - 1] +=
                                    _lScalarConstraints[i].Coeff()[irow, icol].Z()
                                    * _lxyzConstraints[i2].Coeff()[irow2, icol2] * tmpmat[icol, icol2];
                            }
                        }
                    }
                }

                k2 += _lxyzConstraints[i2].Coeff().ColLength();
                kppc2 += _lxyzConstraints[i2].Coeff().RowLength();
            }

            j = nCC1 + nCC2;
            for (int iu = 0; iu < _order; iu++)
            {
                for (int iv = 0; iu + iv < _order; iv++)
                {

                    math_Vector vmat = new(1, _lScalarConstraints[i].GetPPC().Length());
                    for (int ippc = 1; ippc <= _lScalarConstraints[i].GetPPC().Length(); ippc++)
                    {
                        int idu = Deru(kppc + ippc - 1);
                        int idv = Derv(kppc + ippc - 1);
                        vmat[ippc] = Polm(Points(kppc + ippc - 1), iu, iv, idu, idv);
                    }

                    for (int irow = 1; irow <= _lScalarConstraints[i].Coeff().ColLength(); irow++)
                    {
                        for (int icol = 1; icol <= _lScalarConstraints[i].Coeff().RowLength(); icol++)
                        {
                            mat[k + irow - 1, j] += _lScalarConstraints[i].Coeff()[irow, icol].X() * vmat[icol];
                            mat[k + irow - 1, n_dimsousmat + j] +=
                                _lScalarConstraints[i].Coeff()[irow, icol].Y() * vmat[icol];
                            mat[k + irow - 1, 2 * n_dimsousmat + j] +=
                                _lScalarConstraints[i].Coeff()[irow, icol].Z() * vmat[icol];
                        }
                    }

                    j++;
                }
            }

            k2 = 3 * n_dimsousmat;
            kppc2 = nPPC2;
            for (i2 = 1; i2 <= i; i2++)
            {

                Matrix tmpmat = new(1,
                                                      _lScalarConstraints[i].GetPPC().Length(),
                                                      1,
                                                      _lScalarConstraints[i2].GetPPC().Length());

                for (int ippc = 1; ippc <= _lScalarConstraints[i].GetPPC().Length(); ippc++)
                {
                    for (int ippc2 = 1; ippc2 <= _lScalarConstraints[i2].GetPPC().Length(); ippc2++)
                    {
                        double signe = 1;
                        if (((Deru(kppc2 + ippc2 - 1) + Derv(kppc2 + ippc2 - 1)) % 2) == 1)
                        {
                            signe = -1;
                        }
                        int a_iu = Deru(kppc + ippc - 1) + Deru(kppc2 + ippc2 - 1);
                        int iv = Derv(kppc + ippc - 1) + Derv(kppc2 + ippc2 - 1);
                        tmpmat[ippc, ippc2] =
                            signe * SolEm(Points(kppc + ippc - 1) - Points(kppc2 + ippc2 - 1), a_iu, iv);
                    }
                }

                for (int irow = 1; irow <= _lScalarConstraints[i].Coeff().ColLength(); irow++)
                {
                    for (int irow2 = 1; irow2 <= _lScalarConstraints[i2].Coeff().ColLength(); irow2++)
                    {
                        for (int icol = 1; icol <= _lScalarConstraints[i].Coeff().RowLength(); icol++)
                        {
                            for (int icol2 = 1; icol2 <= _lScalarConstraints[i2].Coeff().RowLength(); icol2++)
                            {
                                mat[k + irow - 1, k2 + irow2 - 1] += _lScalarConstraints[i].Coeff()[irow, icol]
                                                                                                          * _lScalarConstraints[i2].Coeff()[irow2, icol2]
                                                                                                          * tmpmat[icol, icol2];
                            }
                        }
                    }
                }

                k2 += _lScalarConstraints[i2].Coeff().ColLength();
                kppc2 += _lScalarConstraints[i2].Coeff().RowLength();
            }

            k += _lScalarConstraints[i].Coeff().ColLength();
            kppc += _lScalarConstraints[i].Coeff().RowLength();
        }

        for (j = 3 * n_dimsousmat; j < n_dimat; j++)
        {
            for (i = 0; i < j; i++)
            {
                mat[i, j] = mat[j, i];
            }
        }

        // initialisation of the Gauss algorithm
        double pivot_max = 1.0e-12;
        _ok = true; // ************ JHH

        using var scope = new ProgressScope(progress, "Plate_Plate::SolveTI3()", 10);
        Gauss algo_gauss = new(mat, pivot_max, scope.Next(7));

        if (scope.UserBreak())
        {
            _ok = false;
            return;
        }

        if (!algo_gauss.IsDone)
        {
            for (i = nCC1 + nCC2; i < nCC1 + nCC2 + nbm; i++)
            {
                mat[i, i]                                       = 1.0e-8;
                mat[n_dimsousmat + i, n_dimsousmat + i]         = 1.0e-8;
                mat[2 * n_dimsousmat + i, 2 * n_dimsousmat + i] = 1.0e-8;
            }
            pivot_max = 1.0e-18;

            Gauss thealgo2 = new(mat, pivot_max, scope.Next(3));

            if (scope.UserBreak())
            {
                _ok = false;
                return;
            }
            algo_gauss = thealgo2;
            _ok = algo_gauss.IsDone;
        }

        if (_ok)
        {
            //   computation of the linear system solution for the X, Y and Z coordinates
            math_Vector sec_member = new(0, n_dimat - 1, 0.0);
            math_Vector sol = new(0, n_dimat - 1);
            _nDim = _nEl + _order * (_order + 1) / 2;
            _solution = new Xyz[_nDim];

            int icoor;
            for (icoor = 1; icoor <= 3; icoor++)
            {
                for (i = 0; i < nCC1; i++)
                {
                    sec_member[(icoor - 1) * n_dimsousmat + i] = _constraints[i + 1].Value().Coord(icoor);
                }

                k = nCC1;
                for (i = 1; i <= _lxyzConstraints.Length(); i++)
                {
                    for (int irow = 1; irow <= _lxyzConstraints[i].Coeff().ColLength(); irow++)
                    {
                        for (int icol = 1; icol <= _lxyzConstraints[i].Coeff().RowLength(); icol++)
                        {
                            sec_member[(icoor - 1) * n_dimsousmat + k] +=
                                _lxyzConstraints[i].Coeff()[irow, icol]
                                * _lxyzConstraints[i].GetPPC()[icol].Value().Coord(icoor);
                        }
                        k++;
                    }
                }
            }
            k = 3 * n_dimsousmat;
            for (i = 1; i <= _lScalarConstraints.Length(); i++)
            {
                for (int irow = 1; irow <= _lScalarConstraints[i].Coeff().ColLength(); irow++)
                {
                    for (int icol = 1; icol <= _lScalarConstraints[i].Coeff().RowLength(); icol++)
                    {
                        sec_member[k] += _lScalarConstraints[i].Coeff()[irow, icol]
                                                          * _lScalarConstraints[i].GetPPC()[icol].Value();
                    }
                    k++;
                }
            }

            algo_gauss.Solve(sec_member, sol);
            // iteration to refine the solution
            {
                math_Vector sol1 = new(0, n_dimat - 1);
                math_Vector sec_member1 = new(0, n_dimat - 1);
                for (i = 1; i <= iterationNumber; i++)
                {
                    sec_member1 = sec_member - mat * sol;
                    algo_gauss.Solve(sec_member1, sol1);
                    sol += sol1;
                }
            }

            for (icoor = 1; icoor <= 3; icoor++)
            {
                for (i = 0; i < nCC1; i++)
                {
                    Solution(i).SetCoord(icoor, sol[(icoor - 1) * n_dimsousmat + i]);
                }

                int kSolution = nCC1;
                int ksol = nCC1;

                for (i = 1; i <= _lxyzConstraints.Length(); i++)
                {
                    for (int icol = 1; icol <= _lxyzConstraints[i].Coeff().RowLength(); icol++)
                    {
                        double vsol = 0;
                        for (int irow = 1; irow <= _lxyzConstraints[i].Coeff().ColLength(); irow++)
                        {
                            vsol += _lxyzConstraints[i].Coeff()[irow, icol]
                                            * sol[(icoor - 1) * n_dimsousmat + ksol + irow - 1];
                        }
                        Solution(kSolution).SetCoord(icoor, vsol);
                        kSolution++;
                    }
                    ksol += _lxyzConstraints[i].Coeff().ColLength();
                }

                ksol = nCC1 + nCC2;
                for (i = 0; i < _order * (_order + 1) / 2; i++)
                {
                    Solution(_nEl + i).SetCoord(icoor, sol[(icoor - 1) * n_dimsousmat + ksol + i]);
                }
            }

            int ksolS = 3 * n_dimsousmat; // OCCT's ksolS and kSolutionS, renamed (C# scopes)
            int kSolutionS = nPPC2;
            for (i = 1; i <= _lScalarConstraints.Length(); i++)
            {
                for (int icol = 1; icol <= _lScalarConstraints[i].Coeff().RowLength(); icol++)
                {
                    Xyz Vsol = new(0.0, 0.0, 0.0);
                    for (int irow = 1; irow <= _lScalarConstraints[i].Coeff().ColLength(); irow++)
                    {
                        Vsol += _lScalarConstraints[i].Coeff()[irow, icol] * sol[ksolS + irow - 1];
                    }
                    Solution(kSolutionS) = Vsol;
                    kSolutionS++;
                }
                ksolS += _lScalarConstraints[i].Coeff().ColLength();
            }
        }
    }

    private void FillXYZmatrix(Matrix mat, int i0, int j0, int ncc1, int ncc2)
    {
        int i, j;
        for (i = 0; i < ncc1; i++)
        {
            for (j = 0; j < i; j++)
            {
                double signe = 1;
                if (((Deru(j) + Derv(j)) % 2) == 1)
                {
                    signe = -1;
                }
                int iu = Deru(i) + Deru(j);
                int iv = Derv(i) + Derv(j);
                mat[i0 + i, j0 + j] = signe * SolEm(Points(i) - Points(j), iu, iv);
            }
        }

        int k = ncc1;
        int kppc = ncc1;
        for (i = 1; i <= _lxyzConstraints.Length(); i++)
        {

            for (int a_j = 0; a_j < ncc1; a_j++)
            {

                math_Vector vmat = new(1, _lxyzConstraints[i].GetPPC().Length());

                for (int ippc = 1; ippc <= _lxyzConstraints[i].GetPPC().Length(); ippc++)
                {
                    double signe = 1;
                    if (((Deru(a_j) + Derv(a_j)) % 2) == 1)
                    {
                        signe = -1;
                    }
                    int iu = Deru(kppc + ippc - 1) + Deru(a_j);
                    int iv = Derv(kppc + ippc - 1) + Derv(a_j);
                    vmat[ippc] = signe * SolEm(Points(kppc + ippc - 1) - Points(a_j), iu, iv);
                }

                for (int irow = 1; irow <= _lxyzConstraints[i].Coeff().ColLength(); irow++)
                {
                    for (int icol = 1; icol <= _lxyzConstraints[i].Coeff().RowLength(); icol++)
                    {
                        mat[i0 + k + irow - 1, j0 + a_j] += _lxyzConstraints[i].Coeff()[irow, icol] * vmat[icol];
                    }
                }
            }

            int k2 = ncc1;
            int kppc2 = ncc1;
            for (int i2 = 1; i2 <= i; i2++)
            {

                Matrix tmpmat = new(1,
                                                      _lxyzConstraints[i].GetPPC().Length(),
                                                      1,
                                                      _lxyzConstraints[i2].GetPPC().Length());

                for (int ippc = 1; ippc <= _lxyzConstraints[i].GetPPC().Length(); ippc++)
                {
                    for (int ippc2 = 1; ippc2 <= _lxyzConstraints[i2].GetPPC().Length(); ippc2++)
                    {
                        double signe = 1;
                        if (((Deru(kppc2 + ippc2 - 1) + Derv(kppc2 + ippc2 - 1)) % 2) == 1)
                        {
                            signe = -1;
                        }
                        int iu = Deru(kppc + ippc - 1) + Deru(kppc2 + ippc2 - 1);
                        int iv = Derv(kppc + ippc - 1) + Derv(kppc2 + ippc2 - 1);
                        tmpmat[ippc, ippc2] =
                            signe * SolEm(Points(kppc + ippc - 1) - Points(kppc2 + ippc2 - 1), iu, iv);
                    }
                }

                for (int irow = 1; irow <= _lxyzConstraints[i].Coeff().ColLength(); irow++)
                {
                    for (int irow2 = 1; irow2 <= _lxyzConstraints[i2].Coeff().ColLength(); irow2++)
                    {
                        for (int icol = 1; icol <= _lxyzConstraints[i].Coeff().RowLength(); icol++)
                        {
                            for (int icol2 = 1; icol2 <= _lxyzConstraints[i2].Coeff().RowLength(); icol2++)
                            {
                                mat[i0 + k + irow - 1, j0 + k2 + irow2 - 1] +=
                                    _lxyzConstraints[i].Coeff()[irow, icol]
                                    * _lxyzConstraints[i2].Coeff()[irow2, icol2] * tmpmat[icol, icol2];
                            }
                        }
                    }
                }

                k2 += _lxyzConstraints[i2].Coeff().ColLength();
                kppc2 += _lxyzConstraints[i2].Coeff().RowLength();
            }

            k += _lxyzConstraints[i].Coeff().ColLength();
            kppc += _lxyzConstraints[i].Coeff().RowLength();
        }

        i = ncc1 + ncc2;
        for (int iu = 0; iu < _order; iu++)
        {
            for (int iv = 0; iu + iv < _order; iv++)
            {
                for (int a_j = 0; a_j < ncc1; a_j++)
                {
                    int idu = Deru(a_j);
                    int idv = Derv(a_j);
                    mat[i0 + i, j0 + a_j] = Polm(Points(a_j), iu, iv, idu, idv);
                }

                int k2 = ncc1;
                int kppc2 = ncc1;
                for (int i2 = 1; i2 <= _lxyzConstraints.Length(); i2++)
                {
                    math_Vector vmat = new(1, _lxyzConstraints[i2].GetPPC().Length());
                    for (int ippc2 = 1; ippc2 <= _lxyzConstraints[i2].GetPPC().Length(); ippc2++)
                    {
                        int idu = Deru(kppc2 + ippc2 - 1);
                        int idv = Derv(kppc2 + ippc2 - 1);
                        vmat[ippc2] = Polm(Points(kppc2 + ippc2 - 1), iu, iv, idu, idv);
                    }

                    for (int irow2 = 1; irow2 <= _lxyzConstraints[i2].Coeff().ColLength(); irow2++)
                    {
                        for (int icol2 = 1; icol2 <= _lxyzConstraints[i2].Coeff().RowLength(); icol2++)
                        {
                            mat[i0 + i, j0 + k2 + irow2 - 1] +=
                                _lxyzConstraints[i2].Coeff()[irow2, icol2] * vmat[icol2];
                        }
                    }

                    k2 += _lxyzConstraints[i2].Coeff().ColLength();
                    kppc2 += _lxyzConstraints[i2].Coeff().RowLength();
                }

                i++;
            }
        }

        int n_dimat = ncc1 + ncc2 + _order * (_order + 1) / 2;

        for (i = 0; i < n_dimat; i++)
        {
            for (int a_j = i + 1; a_j < n_dimat; a_j++)
            {
                mat[i0 + i, j0 + a_j] = mat[i0 + a_j, j0 + i];
            }
        }
    }

    public bool IsDone
    {
        get
        {
            return _ok;
        }
    }

    public void Destroy()
    {
        Init();
    }

    public void Init()
    {
        _constraints.Clear();
        _lxyzConstraints.Clear();
        _lScalarConstraints.Clear();
        _solution = null;
        _points = null;
        _deru = null;
        _derv = null;

        _order = 0;
        _nEl = 0;
        _nDim = 0;
        _ok = true; // OCCT: an empty plate reports IsDone (OCCT's GTest notes it; changing it broke Draw tests)
        _maxConstraintOrder = 0;
    }

    public Xyz Evaluate(in Xy point2d)
    {
        if (_solution == null)
        {
            return new Xyz(0, 0, 0);
        }
        if (!_ok)
        {
            return new Xyz(0, 0, 0);
        }

        Xyz valeur = new(0, 0, 0);

        int i;
        if (!_polynomialPartOnly)
        {
            for (i = 0; i < _nEl; i++)
            {
                double signe = 1;
                if (((Deru(i) + Derv(i)) % 2) == 1)
                {
                    signe = -1;
                }
                valeur += Solution(i) * (signe * SolEm(point2d - Points(i), Deru(i), Derv(i)));
            }
        }
        i = _nEl;
        for (int idu = 0; idu < _order; idu++)
        {
            for (int idv = 0; idu + idv < _order; idv++)
            {
                valeur += Solution(i) * Polm(point2d, idu, idv, 0, 0);
                i++;
            }
        }
        return valeur;
    }

    public Xyz EvaluateDerivative(in Xy point2d, int iu, int iv)
    {
        if (_solution == null)
        {
            return new Xyz(0, 0, 0);
        }
        if (!_ok)
        {
            return new Xyz(0, 0, 0);
        }

        Xyz valeur = new(0, 0, 0);
        int i;
        if (!_polynomialPartOnly)
        {
            for (i = 0; i < _nEl; i++)
            {
                double signe = 1;
                if (((Deru(i) + Derv(i)) % 2) == 1)
                {
                    signe = -1;
                }
                valeur += Solution(i) * (signe * SolEm(point2d - Points(i), Deru(i) + iu, Derv(i) + iv));
            }
        }
        i = _nEl;
        for (int idu = 0; idu < _order; idu++)
        {
            for (int idv = 0; idu + idv < _order; idv++)
            {
                valeur += Solution(i) * Polm(point2d, idu, idv, iu, iv);
                i++;
            }
        }
        return valeur;
    }

    /// <summary>The coefficients of the polynomial part of the plate function (a new array).</summary>
    public NCollection_HArray2<Xyz> CoefPol()
    {
        NCollection_HArray2<Xyz> coefs =
            new NCollection_HArray2<Xyz>(0, _order - 1, 0, _order - 1, new Xyz(0.0, 0.0, 0.0));
        int i = _nEl;
        for (int iu = 0; iu < _order; iu++)
        {
            for (int iv = 0; iu + iv < _order; iv++)
            {
                coefs.ChangeValue(iu, iv) = Solution(i) * _ddu[iu] * _ddv[iv];
                // Coefs.ChangeValue(idu,idv) = Solution(i);
                //  it is necessary to reset this line if one remove factors in method Polm.
                i++;
            }
        }
        return coefs;
    }

    [Obsolete("Use CoefPol() returning handle by value instead")]
    public void CoefPol(out NCollection_HArray2<Xyz> coefs)
    {
        coefs = CoefPol();
    }

    public int Continuity()
    {
        return 2 * _order - 3 - _maxConstraintOrder;
    }

    private double SolEm(in Xy point2d, int iu, int iv)
    {
        double u, v;
        int IU, IV;

        if (iv > iu)
        {
            // SolEm is symmetric in (u<->v) : we swap u and v if iv>iu
            // to avoid some code
            IU = iv;
            IV = iu;
            u = point2d.Y() * _ddv[1];
            v = point2d.X() * _ddu[1];
        }
        else
        {
            IU = iu;
            IV = iv;
            u = point2d.X() * _ddu[1];
            v = point2d.Y() * _ddv[1];
        }

        double u2 = u * u;
        double R = u2 + v * v;
        if (R < 1.0e-20)
        {
            return 0;
        }
        double l = Math.Log(R);
        double duv = 0;

        int m = _order;
        int mm1 = m - 1;
        double r = R;

        // double pr = pow(R, mm1 - IU - IV);
        //  this expression takes a lot of time
        //(does not take into account a small integer value of the exponent)
        //

        int expo = mm1 - IU - IV;
        double pr;
        if (expo < 0)
        {
            pr = R;
            for (int i = 1; i < -expo; i++)
            {
                pr *= R;
            }
            pr = 1.0 / pr;
        }
        else if (expo > 0)
        {
            pr = R;
            for (int i = 1; i < expo; i++)
            {
                pr *= R;
            }
        }
        else
        {
            pr = 1.0;
        }

        switch (IU)
        {
            case 0:
                switch (IV)
                {
                    case 0: {
                        duv = pr * l;
                    }
                    break;

                    default:
                        break;
                }
                break;

            case 1:
                switch (IV)
                {
                    case 0: {
                        duv = 2 * pr * u * (1 + l * mm1);
                    }
                    break;

                    case 1: {
                        double m2 = m * m;
                        // DUV = 4*pr*U*V*(-3+2*L+2*m-3*L*m+L*m2);
                        duv = 4 * pr * u * v * ((2 * m - 3) + (m2 - 3 * m + 2) * l);
                    }
                    break;

                    default:
                        break;
                }
                break;

            case 2:
                switch (IV)
                {
                    case 0: {
                        double m2 = m * m;
                        duv = 2 * pr
                                    * (R - l * R + l * m * R - 6 * u2 + 4 * l * u2 + 4 * m * u2 - 6 * l * m * u2
                                          + 2 * l * m2 * u2);
                    }
                    break;

                    case 1: {
                        double m2 = m * m;
                        double m3 = m2 * m;
                        duv = -3 * R + 2 * l * R + 2 * m * R - 3 * l * m * R + l * m2 * R + 22 * u2 - 12 * l * u2
                                    - 24 * m * u2 + 22 * l * m * u2 + 6 * m2 * u2 - 12 * l * m2 * u2 + 2 * l * m3 * u2;
                        duv = duv * 4 * pr * v;
                    }
                    break;

                    case 2: {
                        double m2 = m * m;
                        double m3 = m2 * m;
                        double m4 = m2 * m2;
                        double V2 = v * v;
                        double R2 = R * R;
                        duv = -3 * R2 + 2 * l * R2 + 2 * m * R2 - 3 * l * m * R2 + l * m2 * R2 + 22 * R * u2
                                    - 12 * l * R * u2 - 24 * m * R * u2 + 22 * l * m * R * u2 + 6 * m2 * R * u2
                                    - 12 * l * m2 * R * u2;
                        duv += 2 * l * m3 * R * u2 + 22 * R * V2 - 12 * l * R * V2 - 24 * m * R * V2
                                      + 22 * l * m * R * V2 + 6 * m2 * R * V2 - 12 * l * m2 * R * V2
                                      + 2 * l * m3 * R * V2 - 200 * u2 * V2 + 96 * l * u2 * V2;
                        duv += 280 * m * u2 * V2 - 200 * l * m * u2 * V2 - 120 * m2 * u2 * V2
                                      + 140 * l * m2 * u2 * V2 + 16 * m3 * u2 * V2 - 40 * l * m3 * u2 * V2
                                      + 4 * l * m4 * u2 * V2;
                        duv = 4 * pr * duv;
                    }
                    break;

                    default:
                        break;
                }
                break;

            case 3:
                switch (IV)
                {
                    case 0: {
                        double m2 = m * m;
                        double m3 = m2 * m;
                        duv = -9 * R + 6 * l * R + 6 * m * R - 9 * l * m * R + 3 * l * m2 * R + 22 * u2
                                    - 12 * l * u2 - 24 * m * u2 + 22 * l * m * u2 + 6 * m2 * u2 - 12 * l * m2 * u2
                                    + 2 * l * m3 * u2;
                        duv = duv * 4 * pr * u;
                    }
                    break;

                    case 1: {
                        double m2 = m * m;
                        double m3 = m2 * m;
                        double m4 = m2 * m2;
                        duv = 33 * R - 18 * l * R - 36 * m * R + 33 * l * m * R + 9 * m2 * R - 18 * l * m2 * R
                                    + 3 * l * m3 * R - 100 * u2 + 48 * l * u2 + 140 * m * u2 - 100 * l * m * u2
                                    - 60 * m2 * u2 + 70 * l * m2 * u2;
                        duv += 8 * m3 * u2 - 20 * l * m3 * u2 + 2 * l * m4 * u2;
                        duv = 8 * pr * u * v * duv;
                    }
                    break;

                    case 2: {
                        double m2 = m * m;
                        double m3 = m2 * m;
                        double m4 = m2 * m2;
                        double m5 = m4 * m;
                        double ru2 = R * u2;
                        double v2 = v * v;
                        double rv2 = R * v2;
                        double u2v2 = v2 * u2;
                        double r2 = r * r;

                        // copy-paste the mathematics
                        duv = -100 * ru2 + 48 * l * ru2 + 140 * m * ru2 - 100 * l * m * ru2 - 60 * m2 * ru2
                                    + 70 * l * m2 * ru2 + 8 * m3 * ru2 - 20 * l * m3 * ru2 + 2 * l * m4 * ru2
                                    - 300 * rv2 + 144 * l * rv2 + 420 * m * rv2 - 300 * l * m * rv2 - 180 * m2 * rv2
                                    + 210 * l * m2 * rv2 + 24 * m3 * rv2 - 60 * l * m3 * rv2 + 6 * l * m4 * rv2
                                    + 33 * r2 - 18 * l * r2 - 36 * m * r2 + 33 * l * m * r2 + 9 * m2 * r2
                                    - 18 * l * m2 * r2 + 3 * l * m3 * r2 + 1096 * u2v2 - 480 * l * u2v2
                                    - 1800 * m * u2v2 + 1096 * l * m * u2v2 + 1020 * m2 * u2v2 - 900 * l * m2 * u2v2
                                    - 240 * m3 * u2v2 + 340 * l * m3 * u2v2 + 20 * m4 * u2v2 - 60 * l * m4 * u2v2
                                    + 4 * l * m5 * u2v2;

                        duv = 8 * pr * u * duv;
                    }
                    break;

                    case 3: {
                        double m2 = m * m;
                        double m3 = m2 * m;
                        double m4 = m2 * m2;
                        double m5 = m3 * m2;
                        double m6 = m3 * m3;
                        double ru2 = r * u2;
                        double v2 = v * v;
                        double rv2 = R * v2;
                        double u2v2 = v2 * u2;
                        double r2 = r * r;

                        // copy-paste the mathematics
                        duv = 1644 * ru2 - 720 * l * ru2 - 2700 * m * ru2 + 1644 * l * m * ru2 + 1530 * m2 * ru2
                                    - 1350 * l * m2 * ru2 - 360 * m3 * ru2 + 510 * l * m3 * ru2 + 30 * m4 * ru2
                                    - 90 * l * m4 * ru2 + 6 * l * m5 * ru2 + 1644 * rv2 - 720 * l * rv2 - 2700 * m * rv2
                                    + 1644 * l * m * rv2 + 1530 * m2 * rv2 - 1350 * l * m2 * rv2 - 360 * m3 * rv2
                                    + 510 * l * m3 * rv2 + 30 * m4 * rv2 - 90 * l * m4 * rv2 + 6 * l * m5 * rv2
                                    - 450 * r2 + 216 * l * r2 + 630 * m * r2 - 450 * l * m * r2 - 270 * m2 * r2
                                    + 315 * l * m2 * r2 + 36 * m3 * r2 - 90 * l * m3 * r2 + 9 * l * m4 * r2
                                    - 7056 * u2v2 + 2880 * l * u2v2 + 12992 * m * u2v2 - 7056 * l * m * u2v2
                                    - 8820 * m2 * u2v2 + 6496 * l * m2 * u2v2 + 2800 * m3 * u2v2 - 2940 * l * m3 * u2v2
                                    - 420 * m4 * u2v2 + 700 * l * m4 * u2v2 + 24 * m5 * u2v2 - 84 * l * m5 * u2v2
                                    + 4 * l * m6 * u2v2;

                        duv = 16 * pr * u * v * duv;
                    }
                    break;

                    default:
                        break;
                }
                break;

            case 4:
                switch (IV)
                {
                    case 0: {
                        double m2 = m * m;
                        double m3 = m2 * m;
                        double m4 = m2 * m2;
                        double U4 = u2 * u2;
                        double R2 = R * R;
                        duv = -9 * R2 + 6 * l * R2 + 6 * m * R2 - 9 * l * m * R2 + 3 * l * m2 * R2 + 132 * R * u2
                                    - 72 * l * R * u2 - 144 * m * R * u2 + 132 * l * m * R * u2 + 36 * m2 * R * u2
                                    - 72 * l * m2 * R * u2;
                        duv += 12 * l * m3 * R * u2 - 200 * U4 + 96 * l * U4 + 280 * m * U4 - 200 * l * m * U4
                                      - 120 * m2 * U4 + 140 * l * m2 * U4 + 16 * m3 * U4 - 40 * l * m3 * U4
                                      + 4 * l * m4 * U4;
                        duv = 4 * pr * duv;
                    }
                    break;

                    case 1: {
                        double m2 = m * m;
                        double m3 = m2 * m;
                        double m4 = m2 * m2;
                        double m5 = m2 * m3;
                        double u4 = u2 * u2;
                        double ru2 = R * u2;
                        double r2 = R * R;

                        // copy-paste the mathematics
                        duv = -600 * ru2 + 288 * l * ru2 + 840 * m * ru2 - 600 * l * m * ru2 - 360 * m2 * ru2
                                    + 420 * l * m2 * ru2 + 48 * m3 * ru2 - 120 * l * m3 * ru2 + 12 * l * m4 * ru2
                                    + 33 * r2 - 18 * l * r2 - 36 * m * r2 + 33 * l * m * r2 + 9 * m2 * r2
                                    - 18 * l * m2 * r2 + 3 * l * m3 * r2 + 1096 * u4 - 480 * l * u4 - 1800 * m * u4
                                    + 1096 * l * m * u4 + 1020 * m2 * u4 - 900 * l * m2 * u4 - 240 * m3 * u4
                                    + 340 * l * m3 * u4 + 20 * m4 * u4 - 60 * l * m4 * u4 + 4 * l * m5 * u4;

                        duv = 8 * pr * v * duv;
                    }
                    break;

                    case 2: {
                        double m2 = m * m;
                        double m3 = m2 * m;
                        double m4 = m2 * m2;
                        double m5 = m2 * m3;
                        double m6 = m3 * m3;
                        double u4 = u2 * u2;
                        double r2 = r * r;
                        double r3 = r2 * r;
                        double v2 = v * v;
                        double u2v2 = v2 * u2;
                        double ru2v2 = R * u2v2;
                        double u4v2 = u4 * v2;
                        double r2u2 = r2 * u2;
                        double ru4 = r * u4;
                        double r2v2 = r2 * v2;

                        // copy-paste the mathematics
                        duv = 6576 * ru2v2 - 2880 * l * ru2v2 - 10800 * m * ru2v2 + 6576 * l * m * ru2v2
                                    + 6120 * m2 * ru2v2 - 5400 * l * m2 * ru2v2 - 1440 * m3 * ru2v2
                                    + 2040 * l * m3 * ru2v2 + 120 * m4 * ru2v2 - 360 * l * m4 * ru2v2
                                    + 24 * l * m5 * ru2v2 + 1096 * ru4 - 480 * l * ru4 - 1800 * m * ru4
                                    + 1096 * l * m * ru4 + 1020 * m2 * ru4 - 900 * l * m2 * ru4 - 240 * m3 * ru4
                                    + 340 * l * m3 * ru4 + 20 * m4 * ru4 - 60 * l * m4 * ru4 + 4 * l * m5 * ru4
                                    - 600 * r2u2 + 288 * l * r2u2 + 840 * m * r2u2 - 600 * l * m * r2u2
                                    - 360 * m2 * r2u2 + 420 * l * m2 * r2u2 + 48 * m3 * r2u2 - 120 * l * m3 * r2u2
                                    + 12 * l * m4 * r2u2 - 300 * r2v2 + 144 * l * r2v2 + 420 * m * r2v2
                                    - 300 * l * m * r2v2 - 180 * m2 * r2v2 + 210 * l * m2 * r2v2 + 24 * m3 * r2v2
                                    - 60 * l * m3 * r2v2 + 6 * l * m4 * r2v2 + 33 * r3 - 18 * l * r3 - 36 * m * r3
                                    + 33 * l * m * r3 + 9 * m2 * r3 - 18 * l * m2 * r3 + 3 * l * m3 * r3 - 14112 * u4v2
                                    + 5760 * l * u4v2 + 25984 * m * u4v2 - 14112 * l * m * u4v2 - 17640 * m2 * u4v2
                                    + 12992 * l * m2 * u4v2 + 5600 * m3 * u4v2 - 5880 * l * m3 * u4v2 - 840 * m4 * u4v2
                                    + 1400 * l * m4 * u4v2 + 48 * m5 * u4v2 - 168 * l * m5 * u4v2 + 8 * l * m6 * u4v2;

                        duv = 8 * pr * duv;
                    }
                    break;

                    case 3: {
                        double m2 = m * m;
                        double m3 = m2 * m;
                        double m4 = m2 * m2;
                        double m5 = m2 * m3;
                        double m6 = m3 * m3;
                        double m7 = m3 * m4;
                        double u4 = u2 * u2;
                        double r2 = r * r;
                        double r3 = r2 * r;
                        double v2 = v * v;
                        double u2v2 = v2 * u2;
                        double ru2v2 = R * u2v2;
                        double u4v2 = u4 * v2;
                        double r2u2 = r2 * u2;
                        double r2v2 = r2 * v2;
                        double ru4 = r * u4;

                        // copy-paste the mathematics
                        duv = -42336 * ru2v2 + 17280 * l * ru2v2 + 77952 * m * ru2v2 - 42336 * l * m * ru2v2
                                    - 52920 * m2 * ru2v2 + 38976 * l * m2 * ru2v2 + 16800 * m3 * ru2v2
                                    - 17640 * l * m3 * ru2v2 - 2520 * m4 * ru2v2 + 4200 * l * m4 * ru2v2
                                    + 144 * m5 * ru2v2 - 504 * l * m5 * ru2v2 + 24 * l * m6 * ru2v2 - 21168 * ru4
                                    + 8640 * l * ru4 + 38976 * m * ru4 - 21168 * l * m * ru4 - 26460 * m2 * ru4
                                    + 19488 * l * m2 * ru4 + 8400 * m3 * ru4 - 8820 * l * m3 * ru4 - 1260 * m4 * ru4
                                    + 2100 * l * m4 * ru4 + 72 * m5 * ru4 - 252 * l * m5 * ru4 + 12 * l * m6 * ru4
                                    + 9864 * r2u2 - 4320 * l * r2u2 - 16200 * m * r2u2 + 9864 * l * m * r2u2
                                    + 9180 * m2 * r2u2 - 8100 * l * m2 * r2u2 - 2160 * m3 * r2u2 + 3060 * l * m3 * r2u2
                                    + 180 * m4 * r2u2 - 540 * l * m4 * r2u2 + 36 * l * m5 * r2u2 + 1644 * r2v2
                                    - 720 * l * r2v2 - 2700 * m * r2v2 + 1644 * l * m * r2v2 + 1530 * m2 * r2v2
                                    - 1350 * l * m2 * r2v2 - 360 * m3 * r2v2 + 510 * l * m3 * r2v2 + 30 * m4 * r2v2
                                    - 90 * l * m4 * r2v2 + 6 * l * m5 * r2v2 - 450 * r3 + 216 * l * r3 + 630 * m * r3
                                    - 450 * l * m * r3 - 270 * m2 * r3 + 315 * l * m2 * r3 + 36 * m3 * r3
                                    - 90 * l * m3 * r3 + 9 * l * m4 * r3 + 104544 * u4v2 - 40320 * l * u4v2
                                    - 210112 * m * u4v2 + 104544 * l * m * u4v2 + 162456 * m2 * u4v2
                                    - 105056 * l * m2 * u4v2 - 62720 * m3 * u4v2 + 54152 * l * m3 * u4v2
                                    + 12880 * m4 * u4v2 - 15680 * l * m4 * u4v2 - 1344 * m5 * u4v2
                                    + 2576 * l * m5 * u4v2 + 56 * m6 * u4v2 - 224 * l * m6 * u4v2 + 8 * l * m7 * u4v2;

                        duv = 16 * pr * v * duv;
                    }
                    break;

                    default:
                        break;
                }
                break;

            case 5:
                switch (IV)
                {
                    case 0: {
                        double m2 = m * m;
                        double m3 = m2 * m;
                        double m4 = m2 * m2;
                        double m5 = m2 * m3;
                        double u4 = u2 * u2;
                        double r2 = R * R;
                        double ru2 = R * u2;

                        // copy-paste the mathematics
                        duv = -1000 * ru2 + 480 * l * ru2 + 1400 * m * ru2 - 1000 * l * m * ru2 - 600 * m2 * ru2
                                    + 700 * l * m2 * ru2 + 80 * m3 * ru2 - 200 * l * m3 * ru2 + 20 * l * m4 * ru2
                                    + 165 * r2 - 90 * l * r2 - 180 * m * r2 + 165 * l * m * r2 + 45 * m2 * r2
                                    - 90 * l * m2 * r2 + 15 * l * m3 * r2 + 1096 * u4 - 480 * l * u4 - 1800 * m * u4
                                    + 1096 * l * m * u4 + 1020 * m2 * u4 - 900 * l * m2 * u4 - 240 * m3 * u4
                                    + 340 * l * m3 * u4 + 20 * m4 * u4 - 60 * l * m4 * u4 + 4 * l * m5 * u4;

                        duv = 8 * pr * u * duv;
                    }
                    break;

                    case 1: {
                        double m2 = m * m;
                        double m3 = m2 * m;
                        double m4 = m2 * m2;
                        double m5 = m2 * m3;
                        double m6 = m3 * m3;
                        double u4 = u2 * u2;
                        double ru2 = r * u2;
                        double r2 = r * r;

                        // copy-paste the mathematics
                        duv = 5480 * ru2 - 2400 * l * ru2 - 9000 * m * ru2 + 5480 * l * m * ru2 + 5100 * m2 * ru2
                                    - 4500 * l * m2 * ru2 - 1200 * m3 * ru2 + 1700 * l * m3 * ru2 + 100 * m4 * ru2
                                    - 300 * l * m4 * ru2 + 20 * l * m5 * ru2 - 750 * r2 + 360 * l * r2 + 1050 * m * r2
                                    - 750 * l * m * r2 - 450 * m2 * r2 + 525 * l * m2 * r2 + 60 * m3 * r2
                                    - 150 * l * m3 * r2 + 15 * l * m4 * r2 - 7056 * u4 + 2880 * l * u4 + 12992 * m * u4
                                    - 7056 * l * m * u4 - 8820 * m2 * u4 + 6496 * l * m2 * u4 + 2800 * m3 * u4
                                    - 2940 * l * m3 * u4 - 420 * m4 * u4 + 700 * l * m4 * u4 + 24 * m5 * u4
                                    - 84 * l * m5 * u4 + 4 * l * m6 * u4;

                        duv = 16 * pr * u * v * duv;
                    }
                    break;

                    case 2: {
                        double m2 = m * m;
                        double m3 = m2 * m;
                        double m4 = m2 * m2;
                        double m5 = m2 * m3;
                        double m6 = m3 * m3;
                        double m7 = m3 * m4;
                        double u4 = u2 * u2;
                        double r2 = r * r;
                        double r3 = r2 * r;
                        double v2 = v * v;
                        double u2v2 = v2 * u2;
                        double ru2v2 = R * u2v2;
                        double u4v2 = u4 * v2;
                        double r2u2 = r2 * u2;
                        double r2v2 = r2 * v2;
                        double ru4 = r * u4;

                        // copy-paste the mathematics
                        duv =

                            -70560 * ru2v2 + 28800 * l * ru2v2 + 129920 * m * ru2v2 - 70560 * l * m * ru2v2
                            - 88200 * m2 * ru2v2 + 64960 * l * m2 * ru2v2 + 28000 * m3 * ru2v2
                            - 29400 * l * m3 * ru2v2 - 4200 * m4 * ru2v2 + 7000 * l * m4 * ru2v2 + 240 * m5 * ru2v2
                            - 840 * l * m5 * ru2v2 + 40 * l * m6 * ru2v2 - 7056 * ru4 + 2880 * l * ru4
                            + 12992 * m * ru4 - 7056 * l * m * ru4 - 8820 * m2 * ru4 + 6496 * l * m2 * ru4
                            + 2800 * m3 * ru4 - 2940 * l * m3 * ru4 - 420 * m4 * ru4 + 700 * l * m4 * ru4
                            + 24 * m5 * ru4 - 84 * l * m5 * ru4 + 4 * l * m6 * ru4 + 5480 * r2u2 - 2400 * l * r2u2
                            - 9000 * m * r2u2 + 5480 * l * m * r2u2 + 5100 * m2 * r2u2 - 4500 * l * m2 * r2u2
                            - 1200 * m3 * r2u2 + 1700 * l * m3 * r2u2 + 100 * m4 * r2u2 - 300 * l * m4 * r2u2
                            + 20 * l * m5 * r2u2 + 8220 * r2v2 - 3600 * l * r2v2 - 13500 * m * r2v2
                            + 8220 * l * m * r2v2 + 7650 * m2 * r2v2 - 6750 * l * m2 * r2v2 - 1800 * m3 * r2v2
                            + 2550 * l * m3 * r2v2 + 150 * m4 * r2v2 - 450 * l * m4 * r2v2 + 30 * l * m5 * r2v2
                            - 750 * r3 + 360 * l * r3 + 1050 * m * r3 - 750 * l * m * r3 - 450 * m2 * r3
                            + 525 * l * m2 * r3 + 60 * m3 * r3 - 150 * l * m3 * r3 + 15 * l * m4 * r3
                            + 104544 * u4v2 - 40320 * l * u4v2 - 210112 * m * u4v2 + 104544 * l * m * u4v2
                            + 162456 * m2 * u4v2 - 105056 * l * m2 * u4v2 - 62720 * m3 * u4v2
                            + 54152 * l * m3 * u4v2 + 12880 * m4 * u4v2 - 15680 * l * m4 * u4v2 - 1344 * m5 * u4v2
                            + 2576 * l * m5 * u4v2 + 56 * m6 * u4v2 - 224 * l * m6 * u4v2 + 8 * l * m7 * u4v2;

                        duv = 16 * pr * u * duv;
                    }
                    break;

                    default:
                        break;
                }
                break;

            case 6:
                switch (IV)
                {
                    case 0: {
                        double m2 = m * m;
                        double m3 = m2 * m;
                        double m4 = m2 * m2;
                        double m5 = m2 * m3;
                        double m6 = m3 * m3;
                        double u4 = u2 * u2;
                        double u6 = u2 * u4;
                        double r2 = r * r;
                        double r3 = r2 * r;
                        double r2u2 = r2 * u2;
                        double ru4 = r * u4;

                        // copy-paste the mathematics
                        duv = 16440 * ru4 - 7200 * l * ru4 - 27000 * m * ru4 + 16440 * l * m * ru4
                                    + 15300 * m2 * ru4 - 13500 * l * m2 * ru4 - 3600 * m3 * ru4 + 5100 * l * m3 * ru4
                                    + 300 * m4 * ru4 - 900 * l * m4 * ru4 + 60 * l * m5 * ru4 - 4500 * r2u2
                                    + 2160 * l * r2u2 + 6300 * m * r2u2 - 4500 * l * m * r2u2 - 2700 * m2 * r2u2
                                    + 3150 * l * m2 * r2u2 + 360 * m3 * r2u2 - 900 * l * m3 * r2u2 + 90 * l * m4 * r2u2
                                    + 165 * r3 - 90 * l * r3 - 180 * m * r3 + 165 * l * m * r3 + 45 * m2 * r3
                                    - 90 * l * m2 * r3 + 15 * l * m3 * r3 - 14112 * u6 + 5760 * l * u6 + 25984 * m * u6
                                    - 14112 * l * m * u6 - 17640 * m2 * u6 + 12992 * l * m2 * u6 + 5600 * m3 * u6
                                    - 5880 * l * m3 * u6 - 840 * m4 * u6 + 1400 * l * m4 * u6 + 48 * m5 * u6
                                    - 168 * l * m5 * u6 + 8 * l * m6 * u6;

                        duv = 8 * pr * duv;
                    }
                    break;

                    default:
                        break;
                }
                break;

            default:
                break;
        }

        return duv * _ddu[iu] * _ddv[iv];
    }

    public void UVBox(out double uMin, out double uMax, out double vMin, out double vMax)
    {
        int i;
        double bmin = 1.0e-3;
        uMin = _constraints[1].Pnt2d().X();
        uMax = uMin;
        vMin = _constraints[1].Pnt2d().Y();
        vMax = vMin;

        for (i = 2; i <= _constraints.Length(); i++)
        {
            double x = _constraints[i].Pnt2d().X();
            if (x < uMin)
            {
                uMin = x;
            }
            if (x > uMax)
            {
                uMax = x;
            }
            double y = _constraints[i].Pnt2d().Y();
            if (y < vMin)
            {
                vMin = y;
            }
            if (y > vMax)
            {
                vMax = y;
            }
        }

        for (i = 1; i <= _lxyzConstraints.Length(); i++)
        {
            for (int j = 1; j <= _lxyzConstraints[i].GetPPC().Length(); j++)
            {
                double x = _lxyzConstraints[i].GetPPC()[j].Pnt2d().X();
                if (x < uMin)
                {
                    uMin = x;
                }
                if (x > uMax)
                {
                    uMax = x;
                }
                double y = _lxyzConstraints[i].GetPPC()[j].Pnt2d().Y();
                if (y < vMin)
                {
                    vMin = y;
                }
                if (y > vMax)
                {
                    vMax = y;
                }
            }
        }

        for (i = 1; i <= _lScalarConstraints.Length(); i++)
        {
            for (int j = 1; j <= _lScalarConstraints[i].GetPPC().Length(); j++)
            {
                double x = _lScalarConstraints[i].GetPPC()[j].Pnt2d().X();
                if (x < uMin)
                {
                    uMin = x;
                }
                if (x > uMax)
                {
                    uMax = x;
                }
                double y = _lScalarConstraints[i].GetPPC()[j].Pnt2d().Y();
                if (y < vMin)
                {
                    vMin = y;
                }
                if (y > vMax)
                {
                    vMax = y;
                }
            }
        }

        if (uMax - uMin < bmin)
        {
            double um = 0.5 * (uMin + uMax);
            uMin = um - 0.5 * bmin;
            uMax = um + 0.5 * bmin;
        }
        if (vMax - vMin < bmin)
        {
            double vm = 0.5 * (vMin + vMax);
            vMin = vm - 0.5 * bmin;
            vMax = vm + 0.5 * bmin;
        }
    }

    public void UVConstraints(NCollection_Sequence<Xy> seq)
    {
        for (int i = 1; i <= _constraints.Length(); i++)
        {
            if ((_constraints.Value(i).Idu() == 0) && (_constraints.Value(i).Idv() == 0))
            {
                seq.Append((_constraints.Value(i)).Pnt2d());
            }
        }
    }

    public void SetPolynomialPartOnly(bool ppOnly = true)
    {
        _polynomialPartOnly = ppOnly;
    }

    private double Polm(in Xy point2d, int iu, int iv, int idu, int idv)
    {
        if (idu > iu)
        {
            return 0;
        }
        if (idv > iv)
        {
            return 0;
        }

        double u = point2d.X();
        double v = point2d.Y();

        double value = 1;

        int degu = iu - idu;
        int i;
        for (i = 0; i < degu; i++)
        {
            value *= u;
        }
        for (i = iu; i > degu; i--)
        {
            value *= i;
        }

        int degv = iv - idv;
        for (i = 0; i < degv; i++)
        {
            value *= v;
        }
        for (i = iv; i > degv; i--)
        {
            value *= i;
        }

        // the product by ddu[iu]*ddv[iv] isn't needed, but it improves the conditioning of the matrix (OCCT's note)
        return value * _ddu[iu] * _ddv[iv];
    }

    private ref int Deru(int index) => ref _deru![index];

    private ref int Derv(int index) => ref _derv![index];

    private ref Xyz Solution(int index) => ref _solution![index];

    private ref Xy Points(int index) => ref _points![index];
}
