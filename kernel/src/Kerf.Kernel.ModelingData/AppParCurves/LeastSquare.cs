// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_LeastSquare.gxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_TheLeastSquares.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// lpa, le 27/07/91
// pmn, 30/10/98 : Protection for overconstrained cases -> "isready"

// Approximation of a MultiLine of points described by the tool MLineTool.
// This program uses the least squares method for the following case:
// passing points and tangencies at extremities.

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.Constraint;
using static Kerf.Engine.Numerics.Recipes;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Least-squares approximation of a MultiLine by Bezier curves (Householder-QR or a banded Cholesky
/// resolution) or, given knots and multiplicities, by B-spline curves. The first and last points may be
/// passing, tangency or curvature points; the tangency and curvature values are computed in the resolution.
/// </summary>
/// <remarks>
/// <para>
/// The generic form of OCCT's <c>AppParCurves_LeastSquare.gxx</c> (decision 142), which OCCT instantiates as
/// AppDef_TheLeastSquares, AppDef_ParLeastSquareOf..., AppDef_BSpParLeastSquareOf..., and the GeomInt and
/// BRepApprox equivalents. The public members are those of the instantiations' headers.
/// </para>
/// <para>
/// A working object (decision 21). The system solved is A X = B, where A is the Bernstein (or B-spline) matrix
/// at the parameters, B the point coordinates and X the poles; A is the same for every coordinate of every curve.
/// The matrices returned (<see cref="FunctionMatrix"/>, <see cref="Poles"/>, <see cref="Distance"/>, ...) and the
/// MultiBSpCurve returned by <see cref="BSplineValue"/> are the object's own (decision 33; shared: don't modify).
/// </para>
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="ToolLine">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
internal class LeastSquare<MultiLine, ToolLine>
    where ToolLine : struct, ILineTool<MultiLine>
{
    private Constraint _firstConstraint;
    private Constraint _lastConstraint;
    private readonly MultiBSpCurve _scu;
    private NCollection_HArray1<double>? _myknots;
    private NCollection_HArray1<int>? _mymults;
    private readonly Matrix _mypoles;
    private readonly Matrix _a;
    private readonly Matrix _da;
    private readonly Matrix _b2;
    private readonly Matrix _mypoints;
    private readonly math_Vector _vflatknots;
    private readonly math_Vector _vec1t;
    private readonly math_Vector _vec1c;
    private readonly math_Vector _vec2t;
    private readonly math_Vector _vec2c;
    private readonly Matrix _error;
    private readonly math_IntegerVector _myindex;
    private double _lambda1;
    private double _lambda2;
    private int _firstP;
    private int _lastP;
    private int _nlignes;
    private int _ninc;
    private int _na;
    private int _myfirstp;
    private int _mylastp;
    private int _resinit;
    private int _resfin;
    private int _nbP2d;
    private int _nbP;
    private readonly int _nbpoles;
    private int _deg;
    private bool _done;
    private bool _iscalculated;
    private bool _isready;

    private static int FlatLength(NCollection_Array1<int> mults)
    {
        int sum = 0;
        for (int i = mults.Lower(); i <= mults.Upper(); i++)
        {
            sum += mults.Value(i);
        }
        return sum;
    }

    //=======================================================================
    // function : CheckTangents
    // purpose  : Checks if theArrTg3d and theArrTg2d have direction
    //            corresponded to the direction between theArrPt1 and theArrPt2.
    //            If it is not then reverses tangent vectors.
    //              theArrPt1 (as same as theArrPt2) is sub-set of all 3D-points in
    //            one multy-point (multy-point is union of sets of 2D- and 3D-points).
    //
    // ATTENTION!!!
    //              The property of correlation between Tg3d and Tg2d is used here.
    //            Therefore, only 3D-coinciding is checked.
    //=======================================================================
    private static void CheckTangents(NCollection_Array1<Point3D> arrPt1, NCollection_Array1<Point3D> arrPt2,
                                      NCollection_Array1<Vector3D> arrTg3d, NCollection_Array1<Vector2D> arrTg2d)
    {
        if (arrPt1.Lower() != arrPt2.Lower())
        {
            return;
        }

        if (arrPt1.Upper() != arrPt2.Upper())
        {
            return;
        }

        if (arrTg3d.Length() != arrPt1.Length())
        {
            return;
        }

        bool isToChangeDir = false;

        for (int i = arrPt1.Lower(); i <= arrPt1.Upper(); i++)
        {
            Vector3D v1 = new(arrPt1[i], arrPt2[i]);
            Vector3D v2 = arrTg3d[i];

            if (v1.Dot(v2) < 0.0)
            {
                isToChangeDir = true;
                break;
            }
        }

        if (!isToChangeDir)
        {
            return;
        }

        // Change directions for every 2D- and 3D-tangents

        for (int i = arrTg3d.Lower(); i <= arrTg3d.Upper(); i++)
        {
            arrTg3d[i] = arrTg3d[i].Reversed();
        }

        for (int i = arrTg2d.Lower(); i <= arrTg2d.Upper(); i++)
        {
            arrTg2d[i] = arrTg2d[i].Reversed();
        }
    }

    //=======================================================================
    // function : CheckTangents
    // purpose  : Checks if theArrTg2d have direction
    //            corresponded to the direction between theArrPt1 and theArrPt2.
    //            If it is not then reverses tangent vector.
    //              theArrPt1 (as same as theArrPt2) is sub-set of all 2D-points in
    //            one multy-point (multy-point is union of sets of 2D- and 3D-points).
    //=======================================================================
    private static void CheckTangents(NCollection_Array1<Point2D> arrPt1, NCollection_Array1<Point2D> arrPt2,
                                      NCollection_Array1<Vector2D> arrTg2d)
    {
        if (arrPt1.Lower() != arrPt2.Lower())
        {
            return;
        }

        if (arrPt1.Upper() != arrPt2.Upper())
        {
            return;
        }

        for (int i = arrPt1.Lower(); i <= arrPt1.Upper(); i++)
        {
            Vector2D v1 = new(arrPt1[i], arrPt2[i]);
            Vector2D v2 = arrTg2d[i];

            if (v1.Dot(v2) < 0.0)
            {
                arrTg2d[i] = arrTg2d[i].Reversed();
            }
        }
    }

    // The member initialisers shared by OCCT's four constructors
    private LeastSquare(MultiLine ssp, int firstPoint, int lastPoint, Constraint firstCons,
                                     Constraint lastCons, int nbPol, int flatLength)
    {
        _scu = new MultiBSpCurve(nbPol);
        _mypoles = new Matrix(1, nbPol, 1, NbBColumns(ssp));
        _a = new Matrix(firstPoint, lastPoint, 1, nbPol);
        _da = new Matrix(firstPoint, lastPoint, 1, nbPol);
        _b2 = new Matrix(TheFirstPoint(firstCons, firstPoint),
                             Math.Max(TheFirstPoint(firstCons, firstPoint), TheLastPoint(lastCons, lastPoint)),
                             1,
                             NbBColumns(ssp));
        _mypoints = new Matrix(firstPoint, lastPoint, 1, NbBColumns(ssp));
        _vflatknots = new math_Vector(1, flatLength);
        _vec1t = new math_Vector(1, NbBColumns(ssp));
        _vec1c = new math_Vector(1, NbBColumns(ssp));
        _vec2t = new math_Vector(1, NbBColumns(ssp));
        _vec2c = new math_Vector(1, NbBColumns(ssp));
        _error = new Matrix(firstPoint, lastPoint, 1, ToolLine.NbP3d(ssp) + ToolLine.NbP2d(ssp), 0.0);
        _myindex = new math_IntegerVector(firstPoint, lastPoint, 0);
        _nbpoles = nbPol;
    }

    /// <summary>
    /// Approximates the points <paramref name="firstPoint"/> to <paramref name="lastPoint"/> of
    /// <paramref name="ssp"/> by Bezier curves of <paramref name="nbPol"/> poles at the parameters
    /// <paramref name="parameters"/>, with the given end constraints.
    /// </summary>
    public LeastSquare(MultiLine ssp, int firstPoint, int lastPoint, Constraint firstCons,
                                    Constraint lastCons, math_Vector parameters, int nbPol)
        : this(ssp, firstPoint, lastPoint, firstCons, lastCons, nbPol, 1)
    {
        _firstConstraint = firstCons;
        _lastConstraint = lastCons;
        Init(ssp, firstPoint, lastPoint);
        Perform(parameters);
    }

    /// <summary>Initialises the fields for Bezier curves of <paramref name="nbPol"/> poles; call a Perform method next.</summary>
    public LeastSquare(MultiLine ssp, int firstPoint, int lastPoint, Constraint firstCons,
                                    Constraint lastCons, int nbPol)
        : this(ssp, firstPoint, lastPoint, firstCons, lastCons, nbPol, 1)
    {
        _firstConstraint = firstCons;
        _lastConstraint = lastCons;
        Init(ssp, firstPoint, lastPoint);
    }

    /// <summary>
    /// Approximates the points <paramref name="firstPoint"/> to <paramref name="lastPoint"/> of
    /// <paramref name="ssp"/> by B-spline curves with the knots <paramref name="knots"/> and multiplicities
    /// <paramref name="mults"/> at the parameters <paramref name="parameters"/>, with the given end constraints.
    /// </summary>
    public LeastSquare(MultiLine ssp, NCollection_Array1<double> knots, NCollection_Array1<int> mults,
                                    int firstPoint, int lastPoint, Constraint firstCons,
                                    Constraint lastCons, math_Vector parameters, int nbPol)
        : this(ssp, firstPoint, lastPoint, firstCons, lastCons, nbPol, FlatLength(mults))
    {
        _firstConstraint = firstCons;
        _lastConstraint = lastCons;
        _myknots = new NCollection_HArray1<double>(knots);
        _mymults = new NCollection_HArray1<int>(mults);
        _scu.SetKnots(knots);
        _scu.SetMultiplicities(mults);
        Init(ssp, firstPoint, lastPoint);
        Perform(parameters);
    }

    /// <summary>Initialises the fields for B-spline curves; call a Perform method next.</summary>
    public LeastSquare(MultiLine ssp, NCollection_Array1<double> knots, NCollection_Array1<int> mults,
                                    int firstPoint, int lastPoint, Constraint firstCons,
                                    Constraint lastCons, int nbPol)
        : this(ssp, firstPoint, lastPoint, firstCons, lastCons, nbPol, FlatLength(mults))
    {
        _myknots = new NCollection_HArray1<double>(knots);
        _mymults = new NCollection_HArray1<int>(mults);
        _scu.SetKnots(knots);
        _scu.SetMultiplicities(mults);
        _firstConstraint = firstCons;
        _lastConstraint = lastCons;
        Init(ssp, firstPoint, lastPoint);
    }

    /// <summary>Used by the constructors.</summary>
    protected void Init(MultiLine ssp, int firstPoint, int lastPoint)
    {
        // Variable de controle
        _iscalculated = false;
        _isready = true;

        _myfirstp = firstPoint;
        _mylastp = lastPoint;
        _firstP = TheFirstPoint(_firstConstraint, _myfirstp);
        _lastP = TheLastPoint(_lastConstraint, _mylastp);

        // Identification of constraints at extremities:
        // ========================================
        int i, j, k, i2;

        _nbP2d = ToolLine.NbP2d(ssp);
        _nbP = ToolLine.NbP3d(ssp);
        Point3D poi = new();
        Point2D poi2d = new();
        int mynbP2d = _nbP2d, mynbP = _nbP;
        if (_nbP2d == 0)
        {
            mynbP2d = 1;
        }
        if (_nbP == 0)
        {
            mynbP = 1;
        }
        NCollection_Array1<Point3D> tabP = new(1, mynbP);
        NCollection_Array1<Point2D> tabP2d = new(1, mynbP2d);

        _deg = _nbpoles - 1;

        if (_mymults is not null)
        {
            int sum = 0;
            for (i = _mymults.Lower(); i <= _mymults.Upper(); i++)
            {
                sum += _mymults.Value(i);
            }
            _deg = sum - _nbpoles - 1;
            k = 1;
            double val;
            for (i = _myknots!.Lower(); i <= _myknots.Upper(); i++)
            {
                for (j = 1; j <= _mymults.Value(i); j++)
                {
                    val = _myknots.Value(i);
                    _vflatknots[k] = val;
                    k++;
                }
            }
        }

        Affect(ssp, firstPoint, ref _firstConstraint, _vec1t, _vec1c);

        Affect(ssp, lastPoint, ref _lastConstraint, _vec2t, _vec2c);

        for (j = _myfirstp; j <= _mylastp; j++)
        {
            i2 = 1;
            if (_nbP != 0 && _nbP2d != 0)
            {
                ToolLine.Value(ssp, j, tabP, tabP2d);
            }
            else if (_nbP2d != 0)
            {
                ToolLine.Value(ssp, j, tabP2d);
            }
            else
            {
                ToolLine.Value(ssp, j, tabP);
            }
            for (i = 1; i <= _nbP; i++)
            {
                tabP[i].Coord(out _mypoints[j, i2], out _mypoints[j, i2 + 1], out _mypoints[j, i2 + 2]);
                i2 += 3;
            }
            for (i = 1; i <= _nbP2d; i++)
            {
                tabP2d[i].Coord(out _mypoints[j, i2], out _mypoints[j, i2 + 1]);
                i2 += 2;
            }
        }

        MultiPoint pole1 = new(_nbP, _nbP2d), poleN = new(_nbP, _nbP2d);

        if (_firstConstraint == PassPoint || _firstConstraint == TangencyPoint
            || _firstConstraint == CurvaturePoint)
        {
            i2 = 1;
            for (i = 1; i <= _nbP; i++)
            {
                poi = poi.WithCoord(_mypoints[_myfirstp, i2], _mypoints[_myfirstp, i2 + 1], _mypoints[_myfirstp, i2 + 2]);
                pole1.SetPoint(i, poi);
                i2 += 3;
            }
            for (i = 1; i <= _nbP2d; i++)
            {
                poi2d = poi2d.WithCoord(_mypoints[_myfirstp, i2], _mypoints[_myfirstp, i2 + 1]);
                pole1.SetPoint2d(i + _nbP, poi2d);
                i2 += 2;
            }
            for (i = 1; i <= _mypoles.ColNumber(); i++)
            {
                _mypoles[1, i] = _mypoints[_myfirstp, i];
            }
        }

        // OCCT: the last term tests FirstConstraint where LastConstraint is meant, so a curvature constraint at the
        // first point also copies the last point into the last pole, even when the last point is unconstrained
        if (_lastConstraint == PassPoint || _lastConstraint == TangencyPoint
            || _firstConstraint == CurvaturePoint)
        {
            i2 = 1;
            for (i = 1; i <= _nbP; i++)
            {
                poi = poi.WithCoord(_mypoints[_mylastp, i2], _mypoints[_mylastp, i2 + 1], _mypoints[_mylastp, i2 + 2]);
                poleN.SetPoint(i, poi);
                i2 += 3;
            }
            for (i = 1; i <= _nbP2d; i++)
            {
                poi2d = poi2d.WithCoord(_mypoints[_mylastp, i2], _mypoints[_mylastp, i2 + 1]);
                poleN.SetPoint2d(i + _nbP, poi2d);
                i2 += 2;
            }

            for (i = 1; i <= _mypoles.ColNumber(); i++)
            {
                _mypoles[_nbpoles, i] = _mypoints[_mylastp, i];
            }
        }

        if (_firstConstraint == NoConstraint)
        {
            _resinit = 1;
            _scu.SetValue(1, pole1);
            if (_lastConstraint == NoConstraint)
            {
                _resfin = _nbpoles;
            }
            else if (_lastConstraint == PassPoint)
            {
                _resfin = _nbpoles - 1;
                _scu.SetValue(_nbpoles, poleN);
            }
            else if (_lastConstraint == TangencyPoint)
            {
                _resfin = _nbpoles - 2;
                _scu.SetValue(_nbpoles, poleN);
            }
            else if (_lastConstraint == CurvaturePoint)
            {
                _resfin = _nbpoles - 3;
                _scu.SetValue(_nbpoles, poleN);
            }
        }
        else if (_firstConstraint == PassPoint)
        {
            _resinit = 2;
            _scu.SetValue(1, pole1);
            if (_lastConstraint == NoConstraint)
            {
                _resfin = _nbpoles;
            }
            else if (_lastConstraint == PassPoint)
            {
                _resfin = _nbpoles - 1;
                _scu.SetValue(_nbpoles, poleN);
            }
            else if (_lastConstraint == TangencyPoint)
            {
                _resfin = _nbpoles - 2;
                _scu.SetValue(_nbpoles, poleN);
            }
            else if (_lastConstraint == CurvaturePoint)
            {
                _resfin = _nbpoles - 3;
                _scu.SetValue(_nbpoles, poleN);
            }
        }
        else if (_firstConstraint == TangencyPoint)
        {
            _resinit = 3;
            _scu.SetValue(1, pole1);
            if (_lastConstraint == NoConstraint)
            {
                _resfin = _nbpoles;
            }
            if (_lastConstraint == PassPoint)
            {
                _resfin = _nbpoles - 1;
                _scu.SetValue(_nbpoles, poleN);
            }
            if (_lastConstraint == TangencyPoint)
            {
                _resfin = _nbpoles - 2;
                _scu.SetValue(_nbpoles, poleN);
            }
            else if (_lastConstraint == CurvaturePoint)
            {
                _resfin = _nbpoles - 3;
                _scu.SetValue(_nbpoles, poleN);
            }
        }
        else if (_firstConstraint == CurvaturePoint)
        {
            _resinit = 4;
            _scu.SetValue(1, pole1);
            if (_lastConstraint == NoConstraint)
            {
                _resfin = _nbpoles;
            }
            if (_lastConstraint == PassPoint)
            {
                _resfin = _nbpoles - 1;
                _scu.SetValue(_nbpoles, poleN);
            }
            if (_lastConstraint == TangencyPoint)
            {
                _resfin = _nbpoles - 2;
                _scu.SetValue(_nbpoles, poleN);
            }
            else if (_lastConstraint == CurvaturePoint)
            {
                _resfin = _nbpoles - 3;
                _scu.SetValue(_nbpoles, poleN);
            }
        }

        int nincx = _resfin - _resinit + 1;
        if (nincx < 1)
        { // Impossible d'aller plus loin
            _isready = false;
            return;
        }
        int neq = _lastP - _firstP + 1;

        _na = 3 * _nbP + 2 * _nbP2d;
        _nlignes = _na * neq;
        _ninc = _na * nincx;
        if (_firstConstraint >= TangencyPoint)
        {
            _ninc++;
        }
        if (_lastConstraint >= TangencyPoint)
        {
            _ninc++;
        }
    }

    /// <summary>
    /// Computes the approximation at <paramref name="parameters"/>, after the fields have been initialised.
    /// The curvature case is not treated here.
    /// </summary>
    public void Perform(math_Vector parameters)
    {
        _done = false;
        if (!_isready)
        {
            return;
        }
        int i, j, k, ci, i2, k1, k2;
        int nbpol1 = _nbpoles - 1, ninc1 = _ninc - 1;
        double ad1, a0;
        _iscalculated = false;

        // Calculation of matrix A and DA of approximation functions:
        ComputeFunction(parameters);

        if (_firstConstraint != TangencyPoint && _lastConstraint != TangencyPoint)
        {
            if (_firstConstraint == NoConstraint)
            {
                if (_lastConstraint == NoConstraint)
                {
                    Householder houResol = new(_a, _mypoints);
                    if (!(houResol.IsDone))
                    {
                        _done = false;
                        return;
                    }
                    _done = true;
                    _mypoles.Initialized(houResol.AllValues());
                    return;
                }
                else
                {
                    for (j = _firstP; j <= _lastP; j++)
                    {
                        ad1 = _a[j, _nbpoles];
                        for (i = 1; i <= _b2.ColNumber(); i++)
                        {
                            _b2[j, i] = _mypoints[j, i] - ad1 * _mypoles[_nbpoles, i];
                        }
                    }
                }
            }
            else if (_firstConstraint == PassPoint)
            {
                if (_lastConstraint == NoConstraint)
                {
                    for (j = _firstP; j <= _lastP; j++)
                    {
                        a0 = _a[j, 1];
                        for (i = 1; i <= _b2.ColNumber(); i++)
                        {
                            _b2[j, i] = _mypoints[j, i] - a0 * _mypoles[1, i];
                        }
                    }
                }
                else if (_lastConstraint == PassPoint)
                {
                    for (j = _firstP; j <= _lastP; j++)
                    {
                        a0 = _a[j, 1];
                        ad1 = _a[j, _nbpoles];
                        for (i = 1; i <= _b2.ColNumber(); i++)
                        {
                            _b2[j, i] = _mypoints[j, i] - a0 * _mypoles[1, i] - ad1 * _mypoles[_nbpoles, i];
                        }
                    }
                }
            }

            // resolution:

            // (OCCT declares this block's Nincx and Index again below, in the enclosing scope)
            int nincxB = _resfin - _resinit + 1;
            if (nincxB < 1)
            {
                _done = true;
                return;
            }
            math_IntegerVector indexB = new(1, nincxB);
            SearchIndex(indexB);
            Matrix mytab = new(_resinit, _resfin, 1, _b2.ColNumber(), 0.0);
            math_Vector theAA = new(1, indexB[nincxB], 0.0);
            math_Vector myTABB = new(1, nincxB, 0.0);

            MakeTAA(theAA, mytab);
            DACTCL_Decompose(theAA, indexB);

            int kk2;
            for (j = 1; j <= _b2.ColNumber(); j++)
            {
                kk2 = 1;
                for (i = _resinit; i <= _resfin; i++)
                {
                    myTABB[kk2] = mytab[i, j];
                    kk2++;
                }
                DACTCL_Solve(theAA, myTABB, indexB);

                i2 = 1;
                for (k = _resinit; k <= _resfin; k++)
                {
                    _mypoles[k, j] = myTABB.Value(i2);
                    i2++;
                }
            }
            _done = true;
            // No return here in OCCT: the passing-point cases go on to solve the system below as well, which has
            // the same solution and overwrites the poles just computed with it
        }

        // ===========================================================
        // cas de tangence:
        // ===========================================================

        int nincx = _resfin - _resinit + 1;
        int deport = 0, nincx2 = 2 * nincx;

        math_IntegerVector internalIndex = new(1, nincx);
        SearchIndex(internalIndex);
        math_IntegerVector index = new(1, _ninc);

        int l = 1;
        if (_resinit <= _resfin)
        {
            for (j = 0; j <= _na - 1; j++)
            {
                deport = j * internalIndex[nincx];
                for (i = 1; i <= nincx; i++)
                {
                    index[l] = internalIndex[i] + deport;
                    l++;
                }
            }
        }

        if (_resinit > _resfin)
        {
            index[1] = 1;
        }
        if (ninc1 > 1)
        {
            if (_firstConstraint >= TangencyPoint && _lastConstraint >= TangencyPoint)
            {
                index[ninc1] = index[ninc1 - 1] + ninc1;
            }
        }
        if (_firstConstraint >= TangencyPoint || _lastConstraint >= TangencyPoint)
        {
            index[_ninc] = index[_ninc - 1] + _ninc;
        }

        math_Vector theA = new(1, index[_ninc], 0.0);
        math_Vector myTAB = new(1, _ninc, 0.0);

        MakeTAA(theA, myTAB);

        int error = DACTCL_Decompose(theA, index);
        error = DACTCL_Solve(theA, myTAB, index);

        if (error == 0)
        {
            _done = true;
        }

        if (_firstConstraint >= TangencyPoint && _lastConstraint >= TangencyPoint)
        {
            _lambda1 = myTAB.Value(ninc1);
            _lambda2 = myTAB.Value(_ninc);
        }
        else if (_firstConstraint >= TangencyPoint)
        {
            _lambda1 = myTAB.Value(_ninc);
        }
        else if (_lastConstraint >= TangencyPoint)
        {
            _lambda2 = myTAB.Value(_ninc);
        }

        // The results are stored in mypoles.
        //=========================================
        k = 1;
        i2 = 1;
        for (ci = 1; ci <= _nbP; ci++)
        {
            k1 = k + 1;
            k2 = k + 2;
            for (j = _resinit; j <= _resfin; j++)
            {
                _mypoles[j, k] = myTAB.Value(i2);
                _mypoles[j, k1] = myTAB.Value(i2 + nincx);
                _mypoles[j, k2] = myTAB.Value(i2 + nincx2);
                i2++;
            }

            if (_firstConstraint >= TangencyPoint)
            {
                _mypoles[2, k] = _mypoints[_myfirstp, k] + _lambda1 * _vec1t[k];
                _mypoles[2, k1] = _mypoints[_myfirstp, k1] + _lambda1 * _vec1t[k1];
                _mypoles[2, k2] = _mypoints[_myfirstp, k2] + _lambda1 * _vec1t[k2];
            }
            if (_lastConstraint >= TangencyPoint)
            {
                _mypoles[nbpol1, k] = _mypoints[_mylastp, k] - _lambda2 * _vec2t[k];
                _mypoles[nbpol1, k1] = _mypoints[_mylastp, k1] - _lambda2 * _vec2t[k1];
                _mypoles[nbpol1, k2] = _mypoints[_mylastp, k2] - _lambda2 * _vec2t[k2];
            }
            k += 3;
            i2 += nincx2;
        }

        for (ci = 1; ci <= _nbP2d; ci++)
        {
            k1 = k + 1;
            k2 = k + 2;
            for (j = _resinit; j <= _resfin; j++)
            {
                _mypoles[j, k] = myTAB.Value(i2);
                _mypoles[j, k1] = myTAB.Value(i2 + nincx);
                i2++;
            }
            if (_firstConstraint >= TangencyPoint)
            {
                _mypoles[2, k] = _mypoints[_myfirstp, k] + _lambda1 * _vec1t[k];
                _mypoles[2, k1] = _mypoints[_myfirstp, k1] + _lambda1 * _vec1t[k1];
            }
            if (_lastConstraint >= TangencyPoint)
            {
                _mypoles[nbpol1, k] = _mypoints[_mylastp, k] - _lambda2 * _vec2t[k];
                _mypoles[nbpol1, k1] = _mypoints[_mylastp, k1] - _lambda2 * _vec2t[k1];
            }
            k += 2;
            i2 += nincx;
        }
    }

    /// <summary>
    /// Computes the approximation with the tangent vectors <paramref name="v1t"/> and <paramref name="v2t"/> at the
    /// first and last points, after the fields have been initialised.
    /// </summary>
    public void Perform(math_Vector parameters, math_Vector v1t, math_Vector v2t, double l1, double l2)
    {
        _done = false;
        if (!_isready)
        {
            return;
        }
        int i, lower1 = v1t.Lower(), lower2 = v2t.Lower();
        _resinit = 3;
        _resfin = _nbpoles - 2;
        int nincx = _resfin - _resinit + 1;
        _ninc = _na * nincx + 2;
        _firstConstraint = TangencyPoint;
        _lastConstraint = TangencyPoint;
        for (i = 1; i <= _vec1t.Upper(); i++)
        {
            _vec1t[i] = v1t[i + lower1 - 1];
            _vec2t[i] = v2t[i + lower2 - 1];
        }
        Perform(parameters, l1, l2);
    }

    /// <summary>
    /// Computes the approximation with the tangent vectors <paramref name="v1t"/>, <paramref name="v2t"/> and
    /// the curvature vectors <paramref name="v1c"/>, <paramref name="v2c"/> at the first and last points, after
    /// the fields have been initialised.
    /// </summary>
    public void Perform(math_Vector parameters, math_Vector v1t, math_Vector v2t, math_Vector v1c, math_Vector v2c,
                        double l1, double l2)
    {
        _done = false;
        if (!_isready)
        {
            return;
        }
        int i, lower1 = v1t.Lower(), lower2 = v2t.Lower();
        int lowc1 = v1c.Lower(), lowc2 = v2c.Lower();
        _resinit = 4;
        _resfin = _nbpoles - 3;
        int nincx = _resfin - _resinit + 1;
        _ninc = _na * nincx + 2;
        _firstConstraint = CurvaturePoint;
        _lastConstraint = CurvaturePoint;

        for (i = 1; i <= _vec1t.Upper(); i++)
        {
            _vec1t[i] = v1t[i + lower1 - 1];
            _vec2t[i] = v2t[i + lower2 - 1];
            _vec1c[i] = v1c[i + lowc1 - 1];
            _vec2c[i] = v2c[i + lowc2 - 1];
        }
        Perform(parameters, l1, l2);
    }

    /// <summary>
    /// Computes the approximation with the tangency values <paramref name="l1"/> and <paramref name="l2"/> given,
    /// after the fields have been initialised.
    /// </summary>
    public void Perform(math_Vector parameters, double l1, double l2)
    {
        _done = false;
        if (!_isready)
        {
            return;
        }
        if (_firstConstraint < TangencyPoint && _lastConstraint < TangencyPoint)
        {
            Perform(parameters);
            return;
        }
        _iscalculated = false;

        _lambda1 = l1;
        _lambda2 = l2;
        int i, j, k, i2;
        double ad0, ad1, ad2, a0, a1, a2;
        double l11 = _deg * l1, l22 = _deg * l2;

        ComputeFunction(parameters);

        if (_firstConstraint >= TangencyPoint)
        {
            for (i = 1; i <= _mypoles.ColNumber(); i++)
            {
                _mypoles[2, i] = _mypoints[_myfirstp, i] + l1 * _vec1t[i];
            }
        }

        if (_firstConstraint == CurvaturePoint)
        {
            for (i = 1; i <= _mypoles.ColNumber(); i++)
            {
                _mypoles[3, i] = 2 * _mypoles[2, i] - _mypoles[1, i] + l11 * l11 * _vec1c[i] / (_deg * (_deg - 1));
            }
        }

        if (_lastConstraint >= TangencyPoint)
        {
            for (i = 1; i <= _mypoles.ColNumber(); i++)
            {
                _mypoles[_nbpoles - 1, i] = _mypoints[_mylastp, i] - l2 * _vec2t[i];
            }
        }

        if (_lastConstraint == CurvaturePoint)
        {
            for (i = 1; i <= _mypoles.ColNumber(); i++)
            {
                _mypoles[_nbpoles - 2, i] = 2 * _mypoles[_nbpoles - 1, i] - _mypoles[_nbpoles, i]
                                          + l22 * l22 * _vec2c[i] / (_deg * (_deg - 1));
            }
        }

        if (_resinit > _resfin)
        {
            _done = true;
            return;
        }

        if (_firstConstraint == NoConstraint)
        {
            if (_lastConstraint == TangencyPoint)
            {
                for (j = _firstP; j <= _lastP; j++)
                {
                    ad0 = _a[j, _nbpoles];
                    ad1 = _a[j, _nbpoles - 1];
                    for (i = 1; i <= _b2.ColNumber(); i++)
                    {
                        _b2[j, i] = _mypoints[j, i] - ad0 * _mypoles[_nbpoles, i] - ad1 * _mypoles[_nbpoles - 1, i];
                    }
                }
            }
            if (_lastConstraint == CurvaturePoint)
            {
                for (j = _firstP; j <= _lastP; j++)
                {
                    ad0 = _a[j, _nbpoles];
                    ad1 = _a[j, _nbpoles - 1];
                    ad2 = _a[j, _nbpoles - 2];
                    for (i = 1; i <= _b2.ColNumber(); i++)
                    {
                        _b2[j, i] = _mypoints[j, i] - ad0 * _mypoles[_nbpoles, i] - ad1 * _mypoles[_nbpoles - 1, i]
                                   - ad2 * _mypoles[_nbpoles - 2, i];
                    }
                }
            }
        }
        else if (_firstConstraint == PassPoint)
        {
            if (_lastConstraint == TangencyPoint)
            {
                for (j = _firstP; j <= _lastP; j++)
                {
                    a0 = _a[j, 1];
                    ad0 = _a[j, _nbpoles];
                    ad1 = _a[j, _nbpoles - 1];
                    for (i = 1; i <= _b2.ColNumber(); i++)
                    {
                        _b2[j, i] = _mypoints[j, i] - a0 * _mypoles[1, i] - ad0 * _mypoles[_nbpoles, i]
                                   - ad1 * _mypoles[_nbpoles - 1, i];
                    }
                }
            }
            if (_lastConstraint == CurvaturePoint)
            {
                for (j = _firstP; j <= _lastP; j++)
                {
                    a0 = _a[j, 1];
                    ad0 = _a[j, _nbpoles];
                    ad1 = _a[j, _nbpoles - 1];
                    ad2 = _a[j, _nbpoles - 2];
                    for (i = 1; i <= _b2.ColNumber(); i++)
                    {
                        _b2[j, i] = _mypoints[j, i] - a0 * _mypoles[1, i] - ad0 * _mypoles[_nbpoles, i]
                                   - ad1 * _mypoles[_nbpoles - 1, i] - ad2 * _mypoles[_nbpoles - 2, i];
                    }
                }
            }
        }
        else if (_firstConstraint == TangencyPoint)
        {
            if (_lastConstraint == NoConstraint)
            {
                for (j = _firstP; j <= _lastP; j++)
                {
                    a0 = _a[j, 1];
                    a1 = _a[j, 2];
                    for (i = 1; i <= _b2.ColNumber(); i++)
                    {
                        _b2[j, i] = _mypoints[j, i] - a0 * _mypoles[1, i] - a1 * _mypoles[2, i];
                    }
                }
            }
            else if (_lastConstraint == PassPoint)
            {
                for (j = _firstP; j <= _lastP; j++)
                {
                    a0 = _a[j, 1];
                    ad0 = _a[j, _nbpoles];
                    a1 = _a[j, 2];
                    for (i = 1; i <= _b2.ColNumber(); i++)
                    {
                        _b2[j, i] = _mypoints[j, i] - a0 * _mypoles[1, i] - ad0 * _mypoles[_nbpoles, i] - a1 * _mypoles[2, i];
                    }
                }
            }
            else if (_lastConstraint == TangencyPoint)
            {
                for (j = _firstP; j <= _lastP; j++)
                {
                    a0 = _a[j, 1];
                    ad0 = _a[j, _nbpoles];
                    a1 = _a[j, 2];
                    ad1 = _a[j, _nbpoles - 1];
                    for (i = 1; i <= _b2.ColNumber(); i++)
                    {
                        _b2[j, i] = _mypoints[j, i] - a0 * _mypoles[1, i] - ad0 * _mypoles[_nbpoles, i]
                                   - a1 * _mypoles[2, i] - ad1 * _mypoles[_nbpoles - 1, i];
                    }
                }
            }
        }
        else if (_firstConstraint == CurvaturePoint)
        {
            if (_lastConstraint == NoConstraint)
            {
                for (j = _firstP; j <= _lastP; j++)
                {
                    a0 = _a[j, 1];
                    a1 = _a[j, 2];
                    a2 = _a[j, 3];
                    for (i = 1; i <= _b2.ColNumber(); i++)
                    {
                        _b2[j, i] = _mypoints[j, i] - a0 * _mypoles[1, i] - a1 * _mypoles[2, i] - a2 * _mypoles[3, i];
                    }
                }
            }
            else if (_lastConstraint == PassPoint)
            {
                for (j = _firstP; j <= _lastP; j++)
                {
                    a0 = _a[j, 1];
                    a1 = _a[j, 2];
                    a2 = _a[j, 3];
                    ad0 = _a[j, _nbpoles];
                    for (i = 1; i <= _b2.ColNumber(); i++)
                    {
                        _b2[j, i] = _mypoints[j, i] - a0 * _mypoles[1, i] - a1 * _mypoles[2, i] - a2 * _mypoles[3, i]
                                   - ad0 * _mypoles[_nbpoles, i];
                    }
                }
            }
            else if (_lastConstraint == TangencyPoint)
            {
                for (j = _firstP; j <= _lastP; j++)
                {
                    a0 = _a[j, 1];
                    a1 = _a[j, 2];
                    a2 = _a[j, 3];
                    ad0 = _a[j, _nbpoles];
                    ad1 = _a[j, _nbpoles - 1];
                    for (i = 1; i <= _b2.ColNumber(); i++)
                    {
                        _b2[j, i] = _mypoints[j, i] - a0 * _mypoles[1, i] - a1 * _mypoles[2, i] - a2 * _mypoles[3, i]
                                   - ad0 * _mypoles[_nbpoles, i] - ad1 * _mypoles[_nbpoles - 1, i];
                    }
                }
            }
            else if (_lastConstraint == CurvaturePoint)
            {
                for (j = _firstP; j <= _lastP; j++)
                {
                    a0 = _a[j, 1];
                    a1 = _a[j, 2];
                    a2 = _a[j, 3];
                    ad0 = _a[j, _nbpoles];
                    ad1 = _a[j, _nbpoles - 1];
                    ad2 = _a[j, _nbpoles - 2];
                    for (i = 1; i <= _b2.ColNumber(); i++)
                    {
                        _b2[j, i] = _mypoints[j, i] - a0 * _mypoles[1, i] - a1 * _mypoles[2, i] - a2 * _mypoles[3, i]
                                   - ad0 * _mypoles[_nbpoles, i] - ad1 * _mypoles[_nbpoles - 1, i]
                                   - ad2 * _mypoles[_nbpoles - 2, i];
                    }
                }
            }
        }

        int nincx = _resfin - _resinit + 1;

        Matrix mytab = new(_resinit, _resfin, 1, _b2.ColNumber(), 0.0);
        math_IntegerVector index = new(1, nincx);
        SearchIndex(index);
        math_Vector aa = new(1, index[nincx], 0.0);
        MakeTAA(aa, mytab);

        math_Vector myTABB = new(1, nincx, 0.0);

        DACTCL_Decompose(aa, index);

        int kk2;
        for (j = 1; j <= _b2.ColNumber(); j++)
        {
            kk2 = 1;
            for (i = _resinit; i <= _resfin; i++)
            {
                myTABB[kk2] = mytab[i, j];
                kk2++;
            }

            DACTCL_Solve(aa, myTABB, index);

            i2 = 1;
            for (k = _resinit; k <= _resfin; k++)
            {
                _mypoles[k, j] = myTABB.Value(i2);
                i2++;
            }
        }

        _done = true;
    }

    //=======================================================================
    // function : Affect
    // purpose  :    Index is an ID of the point in MultiLine. Every point is set of
    //            several 3D- and 2D-points. E.g. every points of Walking-line,
    //            obtained in intersection algorithm, is set of one 3D points
    //            (nbP == 1) and two 2D-points (nbP2d == 2).
    //=======================================================================
    /// <summary>Sets the tangent and curvature vectors of a constrained end point, relaxing the constraint if the line has none.</summary>
    protected void Affect(MultiLine ssp, int index, ref Constraint cons, math_Vector vt, math_Vector vc)
    {
        // Vt: vector of tangent, Vc: vector of curvature.

        if (cons >= TangencyPoint)
        {
            int i, i2 = 1;
            bool ok;
            int mynbP2d = _nbP2d, mynbP = _nbP;
            if (_nbP2d == 0)
            {
                mynbP2d = 1;
            }
            if (_nbP == 0)
            {
                mynbP = 1;
            }
            NCollection_Array1<Vector3D> tabV = new(1, mynbP);
            NCollection_Array1<Vector2D> tabV2d = new(1, mynbP2d);

            if (cons == CurvaturePoint)
            {
                if (_nbP != 0 && _nbP2d != 0)
                {
                    ok = ToolLine.Curvature(ssp, index, tabV, tabV2d);
                    if (!ok)
                    {
                        cons = TangencyPoint;
                    }
                }
                else if (_nbP2d != 0)
                {
                    ok = ToolLine.Curvature(ssp, index, tabV2d);
                    if (!ok)
                    {
                        cons = TangencyPoint;
                    }
                }
                else
                {
                    ok = ToolLine.Curvature(ssp, index, tabV);
                    if (!ok)
                    {
                        cons = TangencyPoint;
                    }
                }
                if (ok)
                {
                    for (i = 1; i <= _nbP; i++)
                    {
                        tabV[i].Coord(out vc[i2], out vc[i2 + 1], out vc[i2 + 2]);
                        i2 += 3;
                    }

                    for (i = 1; i <= _nbP2d; i++)
                    {
                        tabV2d[i].Coord(out vc[i2], out vc[i2 + 1]);
                        i2 += 2;
                    }
                }
            }

            i2 = 1;
            if (cons >= TangencyPoint)
            {
                if (_nbP != 0 && _nbP2d != 0)
                {
                    ok = ToolLine.Tangency(ssp, index, tabV, tabV2d);
                    if (!ok)
                    {
                        cons = PassPoint;
                    }
                }
                else if (_nbP2d != 0)
                {
                    ok = ToolLine.Tangency(ssp, index, tabV2d);
                    if (!ok)
                    {
                        cons = PassPoint;
                    }
                }
                else
                {
                    ok = ToolLine.Tangency(ssp, index, tabV);
                    if (!ok)
                    {
                        cons = PassPoint;
                    }
                }

                if (ok)
                {
                    NCollection_Array1<Point3D> arrPts3d1 = new(1, mynbP), arrPts3d2 = new(1, mynbP);

                    if (_nbP != 0)
                    {
                        if (index < ToolLine.LastPoint(ssp))
                        {
                            ToolLine.Value(ssp, index, arrPts3d1);
                            ToolLine.Value(ssp, index + 1, arrPts3d2);
                        }
                        else
                        { // (Index == ToolLine::LastPoint(theML))
                            ToolLine.Value(ssp, index - 1, arrPts3d1);
                            ToolLine.Value(ssp, index, arrPts3d2);
                        }

                        CheckTangents(arrPts3d1, arrPts3d2, tabV, tabV2d);
                    }
                    else if (_nbP2d != 0)
                    {
                        NCollection_Array1<Point2D> arrPts2d1 = new(1, mynbP2d), arrPts2d2 = new(1, mynbP2d);

                        if (index < ToolLine.LastPoint(ssp))
                        {
                            ToolLine.Value(ssp, index, arrPts3d1, arrPts2d1);
                            ToolLine.Value(ssp, index + 1, arrPts3d2, arrPts2d2);
                        }
                        else
                        { // (Index == ToolLine::LastPoint(theML))
                            ToolLine.Value(ssp, index - 1, arrPts3d1, arrPts2d1);
                            ToolLine.Value(ssp, index, arrPts3d2, arrPts2d2);
                        }

                        CheckTangents(arrPts2d1, arrPts2d2, tabV2d);
                    }

                    for (i = 1; i <= _nbP; i++)
                    {
                        tabV[i].Coord(out vt[i2], out vt[i2 + 1], out vt[i2 + 2]);
                        i2 += 3;
                    }

                    for (i = 1; i <= _nbP2d; i++)
                    {
                        tabV2d[i].Coord(out vt[i2], out vt[i2 + 1]);
                        i2 += 2;
                    }
                }
            }
        }
    }

    /// <summary>The number of columns of the second member: 3 per 3d point, 2 per 2d point.</summary>
    protected int NbBColumns(MultiLine ssp)
    {
        int bCol;
        bCol = (ToolLine.NbP3d(ssp)) * 3 + (ToolLine.NbP2d(ssp)) * 2;
        return bCol;
    }

    /// <summary>
    /// The errors between the MultiLine and the curves: <paramref name="f"/> is the sum of the squared distances,
    /// <paramref name="maxE3d"/> and <paramref name="maxE2d"/> the largest 3d and 2d distances.
    /// </summary>
    public void Error(out double f, out double maxE3d, out double maxE2d)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        int i, j, k, i2, indexdeb, indexfin;
        int i21, i22;
        double aa, bb, cc, fi, fx, fy, fz, aij;
        maxE3d = maxE2d = 0.0;
        f = 0.0;
        i2 = 1;
        math_Vector px = new(1, _nbpoles), py = new(1, _nbpoles), pz = new(1, _nbpoles);

        for (k = 1; k <= _nbP + _nbP2d; k++)
        {
            i21 = i2 + 1;
            i22 = i2 + 2;
            for (i = 1; i <= _nbpoles; i++)
            {
                px[i] = _mypoles[i, i2];
                py[i] = _mypoles[i, i21];
                if (k <= _nbP)
                {
                    pz[i] = _mypoles[i, i22];
                }
            }
            for (i = _firstP; i <= _lastP; i++)
            {
                aa = 0.0;
                bb = 0.0;
                cc = 0.0;
                indexdeb = _myindex[i] + 1;
                indexfin = indexdeb + _deg;
                for (j = indexdeb; j <= indexfin; j++)
                {
                    aij = _a[i, j];
                    aa += aij * px[j];
                    bb += aij * py[j];
                    if (k <= _nbP)
                    {
                        cc += aij * pz[j];
                    }
                }
                fx = aa - _mypoints[i, i2];
                fy = bb - _mypoints[i, i21];
                fi = fx * fx + fy * fy;
                if (k <= _nbP)
                {
                    fz = cc - _mypoints[i, i22];
                    fi += fz * fz;
                    if (fi > maxE3d)
                    {
                        maxE3d = fi;
                    }
                }
                else
                {
                    if (fi > maxE2d)
                    {
                        maxE2d = fi;
                    }
                }
                _error[i, k] = fi;
                f += fi;
            }
            if (k <= _nbP)
            {
                i2 += 3;
            }
            else
            {
                i2 += 2;
            }
        }
        maxE3d = Math.Sqrt(maxE3d);
        maxE2d = Math.Sqrt(maxE2d);
    }

    /// <summary>
    /// As <see cref="Error"/>, and fills <paramref name="grad"/> with the derivative of <paramref name="f"/> with
    /// respect to each parameter.
    /// </summary>
    public void ErrorGradient(math_Vector grad, out double f, out double maxE3d, out double maxE2d)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        int i, j, k, i2, indexdeb, indexfin;
        double aa, bb, cc, fi, fx, fy, fz, aij;
        double daij, daa, dbb, dcc, gr;
        maxE3d = maxE2d = 0.0;
        int i21, i22;
        f = 0.0;
        i2 = 1;
        math_Vector px = new(1, _nbpoles), py = new(1, _nbpoles), pz = new(1, _nbpoles);

        for (k = grad.Lower(); k <= grad.Upper(); k++)
        {
            grad[k] = 0.0;
        }

        for (k = 1; k <= _nbP + _nbP2d; k++)
        {
            i21 = i2 + 1;
            i22 = i2 + 2;
            for (i = 1; i <= _nbpoles; i++)
            {
                px[i] = _mypoles[i, i2];
                py[i] = _mypoles[i, i21];
                if (k <= _nbP)
                {
                    pz[i] = _mypoles[i, i22];
                }
            }
            for (i = _firstP; i <= _lastP; i++)
            {
                aa = 0.0;
                bb = 0.0;
                cc = 0.0;
                daa = 0.0;
                dbb = 0.0;
                dcc = 0.0;
                indexdeb = _myindex[i] + 1;
                indexfin = indexdeb + _deg;
                for (j = indexdeb; j <= indexfin; j++)
                {
                    aij = _a[i, j];
                    daij = _da[i, j];
                    aa += aij * px[j];
                    daa += daij * px[j];
                    bb += aij * py[j];
                    dbb += daij * py[j];
                    if (k <= _nbP)
                    {
                        cc += aij * pz[j];
                        dcc += daij * pz[j];
                    }
                }
                fx = aa - _mypoints[i, i2];
                fy = bb - _mypoints[i, i2 + 1];
                fi = fx * fx + fy * fy;
                gr = 2.0 * (daa * fx + dbb * fy);

                if (k <= _nbP)
                {
                    fz = cc - _mypoints[i, i2 + 2];
                    fi += fz * fz;
                    gr += 2.0 * dcc * fz;
                    if (fi > maxE3d)
                    {
                        maxE3d = fi;
                    }
                }
                else
                {
                    if (fi > maxE2d)
                    {
                        maxE2d = fi;
                    }
                }
                _error[i, k] = fi;
                grad[i] += gr;
                f += fi;
            }
            if (k <= _nbP)
            {
                i2 += 3;
            }
            else
            {
                i2 += 2;
            }
        }
        maxE3d = Math.Sqrt(maxE3d);
        maxE2d = Math.Sqrt(maxE2d);
    }

    /// <summary>
    /// The distances between the points of the MultiLine and the curves, after <see cref="Error"/> or
    /// <see cref="ErrorGradient"/> (shared: don't modify).
    /// </summary>
    public Matrix Distance()
    {
        if (!_iscalculated)
        {
            for (int i = _myfirstp; i <= _mylastp; i++)
            {
                for (int j = 1; j <= _nbP + _nbP2d; j++)
                {
                    _error[i, j] = Math.Sqrt(_error[i, j]);
                }
            }
            _iscalculated = true;
        }
        return _error;
    }

    /// <summary>(P2 - P1) / V1 if the first point is a tangency point.</summary>
    public double FirstLambda() => _lambda1;

    /// <summary>(PN - PN-1) / VN if the last point is a tangency point.</summary>
    public double LastLambda() => _lambda2;

    /// <summary>True if the approximation was computed.</summary>
    public bool IsDone => _done;

    /// <summary>The Bezier curves: a MultiCurve sharing the poles of <see cref="BSplineValue"/> (OCCT's slicing copy).</summary>
    public MultiCurve BezierValue()
    {
        if (_myknots is not null)
        {
            throw new NoSuchObjectException();
        }
        return new MultiCurve(BSplineValue());
    }

    /// <summary>The curves; the object's own MultiBSpCurve (shared: don't modify).</summary>
    public MultiBSpCurve BSplineValue()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }

        int i, j, j2, npoints = _nbP + _nbP2d;
        Point3D pt = new();
        Point2D pt2d = new();
        int ideb = _resinit, ifin = _resfin;
        if (ideb >= 2)
        {
            ideb = 2;
        }
        if (ifin <= _nbpoles - 1)
        {
            ifin = _nbpoles - 1;
        }

        // Put the result in the corresponding curves
        for (i = ideb; i <= ifin; i++)
        {
            j2 = 1;
            MultiPoint mPole = new(_nbP, _nbP2d);
            for (j = 1; j <= _nbP; j++)
            {
                pt = pt.WithCoord(_mypoles[i, j2], _mypoles[i, j2 + 1], _mypoles[i, j2 + 2]);
                mPole.SetPoint(j, pt);
                j2 += 3;
            }
            for (j = _nbP + 1; j <= npoints; j++)
            {
                pt2d = pt2d.WithCoord(_mypoles[i, j2], _mypoles[i, j2 + 1]);
                mPole.SetPoint2d(j, pt2d);
                j2 += 2;
            }
            _scu.SetValue(i, mPole);
        }
        return _scu;
    }

    /// <summary>The function matrix A (shared: don't modify).</summary>
    public Matrix FunctionMatrix()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _a;
    }

    /// <summary>The derivative function matrix DA (shared: don't modify).</summary>
    public Matrix DerivativeFunctionMatrix()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _da;
    }

    /// <summary>The first point fitted.</summary>
    protected int TheFirstPoint(Constraint firstCons, int firstPoint)
    {
        if (firstCons == NoConstraint)
        {
            return firstPoint;
        }
        else
        {
            return firstPoint + 1;
        }
    }

    /// <summary>The last point fitted.</summary>
    protected int TheLastPoint(Constraint lastCons, int lastPoint)
    {
        if (lastCons == NoConstraint)
        {
            return lastPoint;
        }
        else
        {
            return lastPoint - 1;
        }
    }

    /// <summary>Computes A and DA at the parameters.</summary>
    protected void ComputeFunction(math_Vector parameters)
    {
        if (_myknots is null)
        {
            global::Kerf.Engine.Approximation.AppParCurves.Bernstein(_nbpoles, parameters, _a, _da);
        }
        else
        {
            global::Kerf.Engine.Approximation.AppParCurves.SplineFunction(_nbpoles, _deg, parameters, _vflatknots, _a, _da, _myindex);
        }
    }

    /// <summary>The matrix of the point coordinates (shared: don't modify).</summary>
    public Matrix Points() => _mypoints;

    /// <summary>The matrix of the pole coordinates (shared: don't modify).</summary>
    public Matrix Poles() => _mypoles;

    /// <summary>
    /// The index before the first non-zero value of A and DA for each point: they are non-zero from
    /// KIndex(i) + 1 to KIndex(i) + degree + 1 (shared: don't modify).
    /// </summary>
    public math_IntegerVector KIndex() => _myindex;

    /// <summary>Builds the tangency system (packed for DACTCL) and its second member.</summary>
    protected void MakeTAA(math_Vector theA, math_Vector myTAB)
    {
        int i, j, k, ci, i2, i21, i22;
        double xx = 0.0, yy = 0.0;

        int nincx = _resfin - _resinit + 1;
        int nrow, neq = _lastP - _firstP + 1;

        int ninc1;

        if (_firstConstraint >= TangencyPoint && _lastConstraint >= TangencyPoint)
        {
            ninc1 = _ninc - 1;
        }
        else
        {
            ninc1 = _ninc;
        }

        int myfirst = _a.LowerRow();
        int ix, iy, iz;
        int mylast = myfirst + _nlignes - 1;
        int k1;
        double taf1 = 0.0, taf2 = 0.0, taf3 = 0.0, tab1 = 0.0, tab2 = 0.0;
        int nb, inc, jinit, jfin, u;
        int indexdeb, indexfin;
        int na1 = _na - 1;
        double v1 = 0, v2 = 0, b;
        double ai2, aid, akj;
        math_Vector myB = new(myfirst, mylast, 0.0), myV1 = new(myfirst, mylast, 0.0), myV2 = new(myfirst, mylast, 0.0);
        math_Vector theV1 = new(1, _ninc, 0.0), theV2 = new(1, _ninc, 0.0);

        for (i = _firstP; i <= _lastP; i++)
        {
            ai2 = _a[i, 2];
            aid = _a[i, _nbpoles - 1];
            if (_firstConstraint >= PassPoint)
            {
                xx = _a[i, 1];
            }
            if (_firstConstraint >= TangencyPoint)
            {
                xx += ai2;
            }
            if (_lastConstraint >= PassPoint)
            {
                yy = _a[i, _nbpoles];
            }
            if (_lastConstraint >= TangencyPoint)
            {
                yy += aid;
            }
            i2 = 1;
            nrow = myfirst - _firstP;
            for (ci = 1; ci <= _nbP; ci++)
            {
                i21 = i2 + 1;
                i22 = i2 + 2;
                ix = i + nrow;
                iy = ix + neq;
                iz = iy + neq;
                if (_firstConstraint >= TangencyPoint)
                {
                    myV1[ix] = ai2 * _vec1t[i2];
                    myV1[iy] = ai2 * _vec1t[i21];
                    myV1[iz] = ai2 * _vec1t[i22];
                }
                if (_lastConstraint >= TangencyPoint)
                {
                    myV2[ix] = -aid * _vec2t[i2];
                    myV2[iy] = -aid * _vec2t[i21];
                    myV2[iz] = -aid * _vec2t[i22];
                }
                myB[ix] = _mypoints[i, i2] - xx * _mypoints[_myfirstp, i2] - yy * _mypoints[_mylastp, i2];
                myB[iy] = _mypoints[i, i21] - xx * _mypoints[_myfirstp, i21] - yy * _mypoints[_mylastp, i21];
                myB[iz] = _mypoints[i, i22] - xx * _mypoints[_myfirstp, i22] - yy * _mypoints[_mylastp, i22];
                i2 += 3;
                nrow = nrow + 3 * neq;
            }

            for (ci = 1; ci <= _nbP2d; ci++)
            {
                i21 = i2 + 1;
                i22 = i2 + 2;
                ix = i + nrow;
                iy = ix + neq;
                if (_firstConstraint >= TangencyPoint)
                {
                    myV1[ix] = ai2 * _vec1t[i2];
                    myV1[iy] = ai2 * _vec1t[i21];
                }
                if (_lastConstraint >= TangencyPoint)
                {
                    myV2[ix] = -aid * _vec2t[i2];
                    myV2[iy] = -aid * _vec2t[i21];
                }
                myB[ix] = _mypoints[i, i2] - xx * _mypoints[_myfirstp, i2] - yy * _mypoints[_mylastp, i2];
                myB[iy] = _mypoints[i, i21] - xx * _mypoints[_myfirstp, i21] - yy * _mypoints[_mylastp, i21];
                nrow = nrow + 2 * neq;
                i2 += 2;
            }
        }

        // Construction de TA*A et TA*B:

        for (k = _firstP; k <= _lastP; k++)
        {
            indexdeb = _myindex[k] + 1;
            indexfin = indexdeb + _deg;
            jinit = Math.Max(_resinit, indexdeb);
            jfin = Math.Min(_resfin, indexfin);
            k1 = k + myfirst - _firstP;
            for (i = 0; i <= na1; i++)
            {
                nb = i * neq + k1;
                if (_firstConstraint >= TangencyPoint)
                {
                    v1 = myV1[nb];
                }
                if (_lastConstraint >= TangencyPoint)
                {
                    v2 = myV2[nb];
                }
                b = myB[nb];
                inc = i * nincx - _resinit + 1;
                for (j = jinit; j <= jfin; j++)
                {
                    akj = _a[k, j];
                    u = j + inc;
                    if (_firstConstraint >= TangencyPoint)
                    {
                        theV1[u] += akj * v1;
                    }
                    if (_lastConstraint >= TangencyPoint)
                    {
                        theV2[u] += akj * v2;
                    }
                    myTAB[u] += akj * b;
                }
                if (_firstConstraint >= TangencyPoint)
                {
                    taf1 += v1 * v1;
                    tab1 += v1 * b;
                }
                if (_lastConstraint >= TangencyPoint)
                {
                    taf2 += v2 * v2;
                    tab2 += v2 * b;
                }
                if (_firstConstraint >= TangencyPoint && _lastConstraint >= TangencyPoint)
                {
                    taf3 += v1 * v2;
                }
            }
        }

        if (_firstConstraint >= TangencyPoint)
        {
            theV1[ninc1] = taf1;
            myTAB[ninc1] = tab1;
        }
        if (_lastConstraint >= TangencyPoint)
        {
            theV2[_ninc] = taf2;
            myTAB[_ninc] = tab2;
        }
        if (_firstConstraint >= TangencyPoint && _lastConstraint >= TangencyPoint)
        {
            theV2[ninc1] = taf3;
        }

        if (_resinit <= _resfin)
        {
            math_IntegerVector index = new(1, nincx);
            SearchIndex(index);
            math_Vector aa = new(1, index[nincx]);
            MakeTAA(aa);

            int kk = 1;
            for (k = 1; k <= _na; k++)
            {
                for (i = 1; i <= aa.Length(); i++)
                {
                    theA[kk] = aa[i];
                    kk++;
                }
            }
        }

        int length = theA.Length();

        if (_firstConstraint >= TangencyPoint && _lastConstraint >= TangencyPoint)
        {
            for (j = 1; j <= ninc1; j++)
            {
                theA[length - 2 * _ninc + j + 1] = theV1[j];
            }
            for (j = 1; j <= _ninc; j++)
            {
                theA[length - _ninc + j] = theV2[j];
            }
        }

        else if (_firstConstraint >= TangencyPoint)
        {
            for (j = 1; j <= _ninc; j++)
            {
                theA[length - _ninc + j] = theV1[j];
            }
        }

        else if (_lastConstraint >= TangencyPoint)
        {
            for (j = 1; j <= _ninc; j++)
            {
                theA[length - _ninc + j] = theV2[j];
            }
        }
    }

    /// <summary>Builds the packed normal matrix tA A and the second members tA B2.</summary>
    protected void MakeTAA(math_Vector aa, Matrix myTAB)
    {
        int i, j, k;
        int indexdeb, indexfin, jinit, jfin;
        int iinit, ifin;
        double akj;
        Matrix theA = new(_resinit, _resfin, _resinit, _resfin);
        theA.Init(0.0);

        for (k = _firstP; k <= _lastP; k++)
        {
            indexdeb = _myindex[k] + 1;
            indexfin = indexdeb + _deg;
            jinit = Math.Max(_resinit, indexdeb);
            jfin = Math.Min(_resfin, indexfin);
            for (i = jinit; i <= jfin; i++)
            {
                akj = _a[k, i];
                for (j = jinit; j <= i; j++)
                {
                    theA[i, j] += _a[k, j] * akj;
                }
                for (j = 1; j <= _b2.ColNumber(); j++)
                {
                    myTAB[i, j] += akj * _b2[k, j];
                }
            }
        }

        int len;
        if (_myknots is not null)
        {
            len = _myknots.Length();
        }
        else
        {
            len = 2;
        }
        int d, i2 = 1;
        iinit = _resinit;
        jinit = _resinit;
        ifin = Math.Min(_resfin, _deg + 1);
        for (k = 2; k <= len; k++)
        {
            for (i = iinit; i <= ifin; i++)
            {
                for (j = jinit; j <= i; j++)
                {
                    aa[i2] = theA[i, j];
                    i2++;
                }
            }
            if (_mymults is not null)
            {
                iinit = ifin + 1;
                d = ifin + _mymults.Value(k);
                ifin = Math.Min(d, _resfin);
                jinit = Math.Max(_resinit, d - _deg);
            }
        }
    }

    /// <summary>Builds the packed normal matrix tA A.</summary>
    protected void MakeTAA(math_Vector aa)
    {
        int i, j, k;
        int indexdeb, indexfin, jinit, jfin;
        int iinit, ifin;
        double akj;
        Matrix theA = new(_resinit, _resfin, _resinit, _resfin, 0.0);

        for (k = _firstP; k <= _lastP; k++)
        {
            indexdeb = _myindex[k] + 1;
            indexfin = indexdeb + _deg;
            jinit = Math.Max(_resinit, indexdeb);
            jfin = Math.Min(_resfin, indexfin);
            for (i = jinit; i <= jfin; i++)
            {
                akj = _a[k, i];
                for (j = jinit; j <= i; j++)
                {
                    theA[i, j] += _a[k, j] * akj;
                }
            }
        }

        int i2 = 1;
        iinit = _resinit;
        jinit = _resinit;
        ifin = Math.Min(_resfin, _deg + 1);
        int len, d;
        if (_myknots is not null)
        {
            len = _myknots.Length();
        }
        else
        {
            len = 2;
        }
        for (k = 2; k <= len; k++)
        {
            for (i = iinit; i <= ifin; i++)
            {
                for (j = jinit; j <= i; j++)
                {
                    aa[i2] = theA[i, j];
                    i2++;
                }
            }
            if (_mymults is not null)
            {
                iinit = ifin + 1;
                d = ifin + _mymults.Value(k);
                ifin = Math.Min(d, _resfin);
                jinit = Math.Max(_resinit, d - _deg);
            }
        }
    }

    /// <summary>The DACTCL profile of the packed normal matrix: the index of each diagonal term.</summary>
    protected void SearchIndex(math_IntegerVector index)
    {
        int iinit, jinit, ifin;
        int i, j, k, d, i2 = 1;
        int len;
        int nincx = _resfin - _resinit + 1;
        index[1] = 1;

        if (_myknots is null)
        {
            if (_resinit <= _resfin)
            {
                int l = 1;
                for (i = 2; i <= nincx; i++)
                {
                    l++;
                    index[l] = index[l - 1] + i;
                }
            }
        }
        else
        {
            iinit = _resinit;
            jinit = _resinit;
            ifin = Math.Min(_resfin, _deg + 1);
            len = _myknots.Length();

            for (k = 2; k <= len; k++)
            {
                for (i = iinit; i <= ifin; i++)
                {
                    for (j = jinit; j <= i; j++)
                    {
                        if (i2 != 1)
                        {
                            index[i2] = index[i2 - 1] + i - jinit + 1;
                        }
                    }
                    i2++;
                }
                iinit = ifin + 1;
                d = ifin + _mymults!.Value(k);
                ifin = Math.Min(d, _resfin);
                jinit = Math.Max(_resinit, d - _deg);
            }
        }
    }
}
