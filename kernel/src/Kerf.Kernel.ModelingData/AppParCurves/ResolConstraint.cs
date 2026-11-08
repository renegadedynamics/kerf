// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_ResolConstraint.gxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_TheResol.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// lpa, le 11/09/91

// Approximation of a constrained set of points (MultiLine) with an
// approximate solution (MultiCurve). The algorithm used is the
// Uzawa algorithm from the mathematical package.

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.Constraint;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Corrects an approximating MultiCurve so that it satisfies the passing and tangency constraints of a
/// MultiLine, by the Uzawa algorithm.
/// </summary>
/// <remarks>
/// The generic form of OCCT's <c>AppParCurves_ResolConstraint.gxx</c> (decision 142), instantiated by OCCT as
/// AppDef_TheResol, AppDef_ResConstraintOf... and the GeomInt and BRepApprox equivalents. A working object; the
/// matrices returned are its own (decision 33; shared: don't modify). OCCT declares an <c>Error()</c> accessor
/// but never defines it; it is left out.
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="ToolLine">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
internal class ResolConstraint<MultiLine, ToolLine>
    where ToolLine : struct, ILineTool<MultiLine>
{
    private bool _done;
    private readonly Matrix _cont;
    private readonly Matrix _deCont;
    private readonly math_Vector _secont;
    private readonly Matrix _ctCinv;
    private readonly math_Vector _vardua;
    private int _incPass;
    private int _incTan;
    private int _incCurv;
    private readonly NCollection_Array1<int> _iPas;
    private readonly NCollection_Array1<int> _iTan;
    private readonly NCollection_Array1<int> _iCurv;

    /// <summary>
    /// Corrects the poles of <paramref name="sCurv"/> (changed in place) for the constraints
    /// <paramref name="theConstraints"/> on the points <paramref name="firstPoint"/> to
    /// <paramref name="lastPoint"/> of <paramref name="ssp"/>, given the Bernstein matrix and its derivative at the
    /// points' parameters.
    /// </summary>
    public ResolConstraint(MultiLine ssp, MultiCurve sCurv, int firstPoint, int lastPoint,
                                        NCollection_HArray1<ConstraintCouple> theConstraints, Matrix bern,
                                        Matrix derivativeBern, double tolerance = 1.0e-10)
    {
        _cont = new Matrix(1,
                               NbConstraints(ssp, firstPoint, lastPoint, theConstraints),
                               1,
                               NbColumns(ssp, sCurv.NbPoles() - 1),
                               0.0);
        _deCont = new Matrix(1,
                                 NbConstraints(ssp, firstPoint, lastPoint, theConstraints),
                                 1,
                                 NbColumns(ssp, sCurv.NbPoles() - 1),
                                 0.0);
        _secont = new math_Vector(1, NbConstraints(ssp, firstPoint, lastPoint, theConstraints), 0.0);
        _ctCinv = new Matrix(1,
                                 NbConstraints(ssp, firstPoint, lastPoint, theConstraints),
                                 1,
                                 NbConstraints(ssp, firstPoint, lastPoint, theConstraints));
        _vardua = new math_Vector(1, NbConstraints(ssp, firstPoint, lastPoint, theConstraints));
        _iPas = new NCollection_Array1<int>(1, lastPoint - firstPoint + 1);
        _iTan = new NCollection_Array1<int>(1, lastPoint - firstPoint + 1);
        _iCurv = new NCollection_Array1<int>(1, lastPoint - firstPoint + 1);

        int i, j, k, nbCu = sCurv.NbCurves();
        int npt;
        int inc3, incSec, incCol, ip = 0, cCol;
        int myindex, def = sCurv.NbPoles() - 1;
        int nb3d, nb2d, npol = def + 1, npol2 = 2 * npol;
        double t1 = 0.0, t2 = 0.0, t3, tmax, daij;
        Vector3D v;
        Vector2D v2d;
        Point3D poi;
        Point2D poi2d;
        ConstraintCouple mycouple;
        Constraint fc = NoConstraint, lc = NoConstraint, cons;

        // Boucle de calcul du nombre de points de passage afin de dimensionner
        // les matrices.
        _incPass = 0;
        _incTan = 0;
        _incCurv = 0;
        for (i = theConstraints.Lower(); i <= theConstraints.Upper(); i++)
        {
            mycouple = theConstraints.Value(i);
            myindex = mycouple.Index();
            cons = mycouple.Constraint();
            if (myindex == firstPoint)
            {
                fc = cons;
            }
            if (myindex == lastPoint)
            {
                lc = cons;
            }
            if ((int)cons >= 1)
            {
                _incPass++; // IncPass = nbre de points de passage.
                _iPas[_incPass] = myindex;
            }
            if ((int)cons >= 2)
            {
                _incTan++; // IncTan= nbre de points de tangence.
                _iTan[_incTan] = myindex;
            }
            if ((int)cons == 3)
            {
                _incCurv++; // IncCurv = nbre de pts de courbure.
                _iCurv[_incCurv] = myindex;
            }
        }

        if (_incPass == 0)
        {
            _done = true;
            return;
        }

        nb3d = ToolLine.NbP3d(ssp);
        nb2d = ToolLine.NbP2d(ssp);
        int mynb3d = nb3d, mynb2d = nb2d;
        if (nb3d == 0)
        {
            mynb3d = 1;
        }
        if (nb2d == 0)
        {
            mynb2d = 1;
        }
        cCol = nb3d * 3 + nb2d * 2;

        // Declaration et initialisation des matrices et vecteurs de contraintes:
        Matrix contInit = new(1, _incPass, 1, npol);
        math_Vector start = new(1, cCol * npol);
        NCollection_Array2<int> ibont = new(1, nbCu, 1, _incTan);

        // Filling Cont for the passing points:
        // =================================================
        for (i = 1; i <= _incPass; i++)
        { // Cette partie ne depend que de Bernstein
            npt = _iPas[i];
            for (j = 1; j <= npol; j++)
            {
                contInit[i, j] = bern[npt, j];
            }
        }
        for (i = 1; i <= cCol; i++)
        {
            _cont.Set(_incPass * (i - 1) + 1, _incPass * i, npol * (i - 1) + 1, npol * i, contInit);
        }

        // Retrieval of starting vectors for Uzawa. This vector represents the
        // poles of SCurv.
        // Filling of secont and resolution.

        NCollection_Array1<Vector3D> tabV = new(1, mynb3d);
        NCollection_Array1<Vector2D> tabV2d = new(1, mynb2d);
        NCollection_Array1<Point3D> tabP = new(1, mynb3d);
        NCollection_Array1<Point2D> tabP2d = new(1, mynb2d);

        inc3 = cCol * _incPass + 1;
        incCol = 0;
        incSec = 0;
        for (k = 1; k <= nbCu; k++)
        {
            if (k <= nb3d)
            {
                for (i = 1; i <= _incTan; i++)
                {
                    npt = _iTan[i];
                    // choix du maximum de tangence pour exprimer la colinearite:
                    ToolLine.Tangency(ssp, npt, tabV);
                    v = tabV[k];
                    v.Coord(out t1, out t2, out t3);
                    tmax = Math.Abs(t1);
                    ibont[k, i] = 1;
                    if (Math.Abs(t2) > tmax)
                    {
                        tmax = Math.Abs(t2);
                        ibont[k, i] = 2;
                    }
                    if (Math.Abs(t3) > tmax)
                    {
                        tmax = Math.Abs(t3);
                        ibont[k, i] = 3;
                    }
                    if (ibont[k, i] == 3)
                    {
                        for (j = 1; j <= npol; j++)
                        {
                            daij = derivativeBern[npt, j];
                            _cont[inc3, j + npol + incCol] = daij * t3 / tmax;
                            _cont[inc3, j + npol2 + incCol] = -daij * t2 / tmax;
                            _cont[inc3 + 1, j + incCol] = daij * t3 / tmax;
                            _cont[inc3 + 1, j + npol2 + incCol] = -daij * t1 / tmax;
                        }
                    }
                    else if (ibont[k, i] == 1)
                    {
                        for (j = 1; j <= npol; j++)
                        {
                            daij = derivativeBern[npt, j];
                            _cont[inc3, j + incCol] = daij * t3 / tmax;
                            _cont[inc3, j + npol2 + incCol] = -daij * t1 / tmax;
                            _cont[inc3 + 1, j + incCol] = daij * t2 / tmax;
                            _cont[inc3 + 1, j + npol + incCol] = -daij * t1 / tmax;
                        }
                    }
                    else if (ibont[k, i] == 2)
                    {
                        for (j = 1; j <= def + 1; j++)
                        {
                            daij = derivativeBern[npt, j];
                            _cont[inc3, j + npol + incCol] = daij * t3 / tmax;
                            _cont[inc3, j + npol2 + incCol] = -daij * t2 / tmax;
                            _cont[inc3 + 1, j + incCol] = daij * t2 / tmax;
                            _cont[inc3 + 1, j + npol + incCol] = -daij * t1 / tmax;
                        }
                    }
                    inc3 = inc3 + 2;
                }

                // Remplissage du second membre:
                for (i = 1; i <= _incPass; i++)
                {
                    ToolLine.Value(ssp, _iPas[i], tabP);
                    poi = tabP[k];
                    _secont[i + incSec] = poi.X;
                    _secont[i + _incPass + incSec] = poi.Y;
                    _secont[i + 2 * _incPass + incSec] = poi.Z;
                }
                incSec = incSec + 3 * _incPass;

                // Vecteur de depart:
                for (j = 1; j <= npol; j++)
                {
                    poi = sCurv.Value(j).Point(k);
                    start[j + incCol] = poi.X;
                    start[j + npol + incCol] = poi.Y;
                    start[j + npol2 + incCol] = poi.Z;
                }
                incCol = incCol + 3 * npol;
            }

            else
            {
                for (i = 1; i <= _incTan; i++)
                {
                    npt = _iTan[i];
                    ToolLine.Tangency(ssp, npt, tabV2d);
                    v2d = tabV2d[k - nb3d];
                    t1 = v2d.X;
                    t2 = v2d.Y;
                    tmax = Math.Abs(t1);
                    ibont[k, i] = 1;
                    if (Math.Abs(t2) > tmax)
                    {
                        tmax = Math.Abs(t2);
                        ibont[k, i] = 2;
                    }
                    for (j = 1; j <= npol; j++)
                    {
                        daij = derivativeBern[npt, j];
                        _cont[inc3, j + incCol] = daij * t2;
                        _cont[inc3, j + npol + incCol] = -daij * t1;
                    }
                    inc3 = inc3 + 1;
                }

                // Remplissage du second membre:
                for (i = 1; i <= _incPass; i++)
                {
                    ToolLine.Value(ssp, _iPas[i], tabP2d);
                    // OCCT: indexes the 2d points with the passing point i where the curve k - nb3d is meant; out of
                    // range unless i - nb3d happens to be a 2d curve (unchecked in OCCT; the port throws)
                    poi2d = tabP2d[i - nb3d];
                    _secont[i + incSec] = poi2d.X;
                    _secont[i + _incPass + incSec] = poi2d.Y;
                }
                incSec = incSec + 2 * _incPass;

                // Remplissage du vecteur de depart:
                for (j = 1; j <= npol; j++)
                {
                    poi2d = sCurv.Value(j).Point2d(k);
                    start[j + incCol] = poi2d.X;
                    start[j + npol + incCol] = poi2d.Y;
                }
                incCol = incCol + npol2;
            }
        }

        // Equations exprimant le meme rapport de tangence sur chaque courbe:
        // On prend les coordonnees les plus significatives.

        --inc3;
        for (i = 1; i <= _incTan; ++i)
        {
            incCol = 0;
            npt = _iTan[i];
            for (k = 1; k <= nbCu - 1; ++k)
            {
                ++inc3;
                // Initialize first relation variable (T1)
                int addIndex_1 = 0, val = ibont[k, i];
                switch (val)
                {
                    case 1: // T1 ~ T1x
                    {
                        if (k <= nb3d)
                        {
                            ToolLine.Tangency(ssp, npt, tabV);
                            v = tabV[k];
                            t1 = v.X;
                            ip = 3 * npol;
                        }
                        else
                        {
                            ToolLine.Tangency(ssp, npt, tabV2d);
                            v2d = tabV2d[k - nb3d];
                            t1 = v2d.X;
                            ip = 2 * npol;
                        }
                        addIndex_1 = 0;
                        break;
                    }
                    case 2: // T1 ~ T1y
                    {
                        if (k <= nb3d)
                        {
                            ToolLine.Tangency(ssp, npt, tabV);
                            v = tabV[k];
                            // OCCT: T1 = V.Y() is missing, so T1 keeps its previous value for a 3d curve
                            ip = 3 * npol;
                        }
                        else
                        {
                            ToolLine.Tangency(ssp, npt, tabV2d);
                            v2d = tabV2d[k - nb3d];
                            t1 = v2d.Y;
                            ip = 2 * npol;
                        }
                        addIndex_1 = npol;
                        break;
                    }
                    case 3: // T1 ~ T1z
                    {
                        ToolLine.Tangency(ssp, npt, tabV);
                        v = tabV[k];
                        t1 = v.Z;
                        ip = 3 * npol;
                        addIndex_1 = 2 * npol;
                        break;
                    }
                }
                // Initialize second relation variable (T2)
                int addIndex_2 = 0, nextVal = ibont[k + 1, i];
                switch (nextVal)
                {
                    case 1: // T2 ~ T2x
                    {
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(ssp, npt, tabV);
                            v = tabV[k + 1];
                            t2 = v.X;
                        }
                        else
                        {
                            ToolLine.Tangency(ssp, npt, tabV2d);
                            v2d = tabV2d[k + 1 - nb3d];
                            t2 = v2d.X;
                        }
                        addIndex_2 = 0;
                        break;
                    }
                    case 2: // T2 ~ T2y
                    {
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(ssp, npt, tabV);
                            v = tabV[k + 1];
                            t2 = v.Y;
                        }
                        else
                        {
                            ToolLine.Tangency(ssp, npt, tabV2d);
                            v2d = tabV2d[k + 1 - nb3d];
                            t2 = v2d.Y;
                        }
                        addIndex_2 = npol;
                        break;
                    }
                    case 3: // T2 ~ T2z
                    {
                        ToolLine.Tangency(ssp, npt, tabV);
                        v = tabV[k + 1];
                        t2 = v.Z;
                        addIndex_2 = 2 * npol;
                        break;
                    }
                }

                // Relations between T1 and T2:
                for (j = 1; j <= npol; j++)
                {
                    daij = derivativeBern[npt, j];
                    _cont[inc3, j + incCol + addIndex_1] = daij * t2;
                    _cont[inc3, j + ip + incCol + addIndex_2] = -daij * t1;
                }
                incCol += ip;
            }
        }

        // Equations concernant la courbure: (commented out in OCCT)

        // Resolution par Uzawa:

        Uzawa uzaResol = new(_cont, _secont, start, tolerance);
        if (!(uzaResol.IsDone))
        {
            _done = false;
            return;
        }
        _ctCinv.Initialized(uzaResol.InverseCont());
        uzaResol.Duale(_vardua);
        for (i = 1; i <= _ctCinv.RowNumber(); i++)
        {
            for (j = i; j <= _ctCinv.RowNumber(); j++)
            {
                _ctCinv[i, j] = _ctCinv[j, i];
            }
        }
        _done = true;
        math_Vector vecPoles = new(1, cCol * npol);
        vecPoles.Initialized(uzaResol.Value());

        int polinit = 1, polfin = npol;
        if ((int)fc >= 1)
        {
            polinit = 2;
        }
        if ((int)lc >= 1)
        {
            polfin = npol - 1;
        }

        for (i = polinit; i <= polfin; i++)
        {
            incCol = 0;
            MultiPoint mPol = new(nb3d, nb2d);
            for (k = 1; k <= nbCu; k++)
            {
                if (k <= nb3d)
                {
                    Point3D pol = new(vecPoles[incCol + i], vecPoles[incCol + npol + i], vecPoles[incCol + 2 * npol + i]);
                    mPol.SetPoint(k, pol);
                    incCol = incCol + 3 * npol;
                }
                else
                {
                    Point2D pol2d = new(vecPoles[incCol + i], vecPoles[incCol + npol + i]);
                    mPol.SetPoint2d(k, pol2d);
                    incCol = incCol + 2 * npol;
                }
            }
            sCurv.SetValue(i, mPol);
        }
    }

    /// <summary>True if the constraints were resolved.</summary>
    public bool IsDone => _done;

    /// <summary>The number of constraint equations.</summary>
    protected int NbConstraints(MultiLine ssp, int firstPoint, int lastPoint,
                                NCollection_HArray1<ConstraintCouple> theConstraints)
    {
        // Boucle de calcul du nombre de points de passage afin de dimensionner
        // les matrices.
        int incPass, incTan, incCurv, cCol;
        int i;
        Constraint cons;

        incPass = 0;
        incTan = 0;
        incCurv = 0;

        for (i = theConstraints.Lower(); i <= theConstraints.Upper(); i++)
        {
            cons = (theConstraints.Value(i)).Constraint();
            if ((int)cons >= 1)
            {
                incPass++; // IncPass = nbre de points de passage.
            }
            if ((int)cons >= 2)
            {
                incTan++; // IncTan= nbre de points de tangence.
            }
            if ((int)cons == 3)
            {
                incCurv++; // IncCurv = nbre de pts de courbure.
            }
        }
        int nb3d = ToolLine.NbP3d(ssp);
        int nb2d = ToolLine.NbP2d(ssp);
        cCol = nb3d * 3 + nb2d * 2;

        // Kerf: OCCT counts three rows for each curvature point, but its equations for them are commented out, so the
        // rows stay zero, the system is singular and any curvature point failed (NotDone). A curvature point is
        // constrained as the tangency it also is (decision 23107).
        return cCol * incPass + incTan * (cCol - 1);
    }

    /// <summary>The number of unknowns: the coordinates of every pole.</summary>
    protected int NbColumns(MultiLine ssp, int deg)
    {
        int nb3d = ToolLine.NbP3d(ssp);
        int nb2d = ToolLine.NbP2d(ssp);
        int cCol = nb3d * 3 + nb2d * 2;

        return cCol * (deg + 1);
    }

    /// <summary>The constraint matrix (shared: don't modify).</summary>
    public Matrix ConstraintMatrix() => _cont;

    /// <summary>The inverse of C tC, symmetrised (shared: don't modify).</summary>
    public Matrix InverseMatrix() => _ctCinv;

    /// <summary>The dual variables of the Uzawa resolution (shared: don't modify).</summary>
    public math_Vector Duale() => _vardua;

    /// <summary>
    /// The derivative of the constraint matrix with respect to the parameters (the object's own matrix,
    /// recomputed by each call; shared: don't modify).
    /// </summary>
    public Matrix ConstraintDerivative(MultiLine ssp, math_Vector parameters, int deg, Matrix da)
    {
        int i, j, k, nb2d, nb3d, cCol, inc3, incCol, npt;
        int npol, npol2, nbCu = ToolLine.NbP3d(ssp) + ToolLine.NbP2d(ssp);
        int ip;
        double daij;
        npol = deg + 1;
        npol2 = 2 * npol;
        NCollection_Array2<int> ibont = new(1, nbCu, 1, _incTan);
        Matrix deCInit = new(1, _incPass, 1, npol);
        math_Vector dda = new(1, npol);
        Vector3D v;
        Vector2D v2d;
        // T1 and T2 are uninitialised in OCCT; the relations below may read the values the loop above left
        double t1 = 0.0, t2 = 0.0, t3, tmax, dDaij;
        nb3d = ToolLine.NbP3d(ssp);
        nb2d = ToolLine.NbP2d(ssp);
        int mynb3d = nb3d, mynb2d = nb2d;
        if (nb3d == 0)
        {
            mynb3d = 1;
        }
        if (nb2d == 0)
        {
            mynb2d = 1;
        }
        cCol = nb3d * 3 + nb2d * 2;

        NCollection_Array1<Vector3D> tabV = new(1, mynb3d);
        NCollection_Array1<Vector2D> tabV2d = new(1, mynb2d);

        for (i = 1; i <= _deCont.RowNumber(); i++)
        {
            for (j = 1; j <= _deCont.ColNumber(); j++)
            {
                _deCont[i, j] = 0.0;
            }
        }

        //  Remplissage de DK pour les points de passages:

        for (i = 1; i <= _incPass; i++)
        {
            npt = _iPas[i];
            for (j = 1; j <= npol; j++)
            {
                deCInit[i, j] = da[npt, j];
            }
        }
        for (i = 1; i <= cCol; i++)
        {
            _deCont.Set(_incPass * (i - 1) + 1, _incPass * i, npol * (i - 1) + 1, npol * i, deCInit);
        }

        // Pour les points de tangence:

        inc3 = cCol * _incPass + 1;
        incCol = 0;

        for (k = 1; k <= nbCu; k++)
        {
            if (k <= nb3d)
            {
                for (i = 1; i <= _incTan; i++)
                {
                    npt = _iTan[i];
                    // choix du maximum de tangence pour exprimer la colinearite:
                    ToolLine.Tangency(ssp, npt, tabV);
                    v = tabV[k];
                    v.Coord(out t1, out t2, out t3);
                    tmax = Math.Abs(t1);
                    ibont[k, i] = 1;
                    if (Math.Abs(t2) > tmax)
                    {
                        tmax = Math.Abs(t2);
                        ibont[k, i] = 2;
                    }
                    if (Math.Abs(t3) > tmax)
                    {
                        tmax = Math.Abs(t3);
                        ibont[k, i] = 3;
                    }
                    global::Kerf.Engine.Approximation.AppParCurves.SecondDerivativeBernstein(parameters[npt], dda);
                    if (ibont[k, i] == 3)
                    {
                        for (j = 1; j <= npol; j++)
                        {
                            dDaij = dda[j];
                            _deCont[inc3, j + npol + incCol] = dDaij * t3 / tmax;
                            _deCont[inc3, j + npol2 + incCol] = -dDaij * t2 / tmax;
                            _deCont[inc3 + 1, j + incCol] = dDaij * t3 / tmax;
                            _deCont[inc3 + 1, j + npol2 + incCol] = -dDaij * t1 / tmax;
                        }
                    }
                    else if (ibont[k, i] == 1)
                    {
                        for (j = 1; j <= npol; j++)
                        {
                            dDaij = dda[j];
                            _deCont[inc3, j + incCol] = dDaij * t3 / tmax;
                            _deCont[inc3, j + npol2 + incCol] = -dDaij * t1 / tmax;
                            _deCont[inc3 + 1, j + incCol] = dDaij * t2 / tmax;
                            _deCont[inc3 + 1, j + npol + incCol] = -dDaij * t1 / tmax;
                        }
                    }
                    else if (ibont[k, i] == 2)
                    {
                        for (j = 1; j <= npol; j++)
                        {
                            dDaij = dda[j];
                            _deCont[inc3, j + npol + incCol] = dDaij * t3 / tmax;
                            _deCont[inc3, j + npol2 + incCol] = -dDaij * t2 / tmax;
                            _deCont[inc3 + 1, j + incCol] = dDaij * t2 / tmax;
                            _deCont[inc3 + 1, j + npol + incCol] = -dDaij * t1 / tmax;
                        }
                    }
                    inc3 = inc3 + 2;
                }
                incCol = incCol + 3 * npol;
            }
            else
            {
                for (i = 1; i <= _incTan; i++)
                {
                    npt = _iTan[i];
                    global::Kerf.Engine.Approximation.AppParCurves.SecondDerivativeBernstein(parameters[npt], dda);
                    ToolLine.Tangency(ssp, npt, tabV2d);
                    // OCCT: tabV2d(k) where k - nb3d is meant (out of range when there are 3d curves; the port throws)
                    v2d = tabV2d[k];
                    v2d.Coord(out t1, out t2);
                    tmax = Math.Abs(t1);
                    ibont[k, i] = 1;
                    if (Math.Abs(t2) > tmax)
                    {
                        tmax = Math.Abs(t2);
                        ibont[k, i] = 2;
                    }
                    for (j = 1; j <= npol; j++)
                    {
                        dDaij = dda[j];
                        _deCont[inc3, j + incCol] = dDaij * t2;
                        _deCont[inc3, j + npol + incCol] = -dDaij * t1;
                    }
                    inc3 = inc3 + 1;
                }
            }
        }

        // Equations exprimant le meme rapport de tangence sur chaque courbe:
        // On prend les coordonnees les plus significatives.
        // OCCT: the relations below are written into Cont, the constraint matrix, where DeCont is meant; and
        // tabV2d is indexed by k where k - nb3d is meant, as above

        inc3 = inc3 - 1;
        for (i = 1; i <= _incTan; i++)
        {
            incCol = 0;
            npt = _iTan[i];
            global::Kerf.Engine.Approximation.AppParCurves.SecondDerivativeBernstein(parameters[npt], dda);
            for (k = 1; k <= nbCu - 1; k++)
            {
                inc3 = inc3 + 1;
                if (ibont[k, i] == 1)
                {
                    if (k <= nb3d)
                    {
                        ToolLine.Tangency(ssp, npt, tabV);
                        v = tabV[k];
                        t1 = v.X;
                        ip = 3 * npol;
                    }
                    else
                    {
                        ToolLine.Tangency(ssp, npt, tabV2d);
                        v2d = tabV2d[k];
                        t1 = v2d.X;
                        ip = 2 * npol;
                    }
                    if (ibont[k + 1, i] == 1)
                    { // Relations entre T1x et T2x:
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(ssp, npt, tabV);
                            v = tabV[k + 1];
                            t2 = v.X;
                        }
                        else
                        {
                            ToolLine.Tangency(ssp, npt, tabV2d);
                            v2d = tabV2d[k + 1];
                            t2 = v2d.X;
                        }
                        for (j = 1; j <= npol; j++)
                        {
                            daij = dda[j];
                            _cont[inc3, j + incCol] = daij * t2;
                            _cont[inc3, j + ip + incCol] = -daij * t1;
                        }
                        incCol = incCol + ip;
                    }
                    else if (ibont[k + 1, i] == 2)
                    { // Relations entre T1x et T2y:
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(ssp, npt, tabV);
                            v = tabV[k + 1];
                            t2 = v.Y;
                        }
                        else
                        {
                            ToolLine.Tangency(ssp, npt, tabV2d);
                            v2d = tabV2d[k + 1];
                            t2 = v2d.Y;
                        }
                        for (j = 1; j <= npol; j++)
                        {
                            daij = dda[j];
                            _cont[inc3, j + incCol] = daij * t2;
                            _cont[inc3, j + ip + npol + incCol] = -daij * t1;
                        }
                        incCol = incCol + ip;
                    }
                    else if (ibont[k + 1, i] == 3)
                    { // Relations entre T1x et T2z:
                        ToolLine.Tangency(ssp, npt, tabV);
                        v = tabV[k + 1];
                        t2 = v.Z;
                        for (j = 1; j <= npol; j++)
                        {
                            daij = dda[j];
                            _cont[inc3, j + incCol] = daij * t2;
                            _cont[inc3, j + ip + 2 * npol + incCol] = -daij * t1;
                        }
                        incCol = incCol + ip;
                    }
                }
                else if (ibont[k, i] == 2)
                {
                    if (k <= nb3d)
                    {
                        ToolLine.Tangency(ssp, npt, tabV);
                        v = tabV[k];
                        t1 = v.Y;
                        ip = 3 * npol;
                    }
                    else
                    {
                        ToolLine.Tangency(ssp, npt, tabV2d);
                        v2d = tabV2d[k];
                        t1 = v2d.Y;
                        ip = 2 * npol;
                    }
                    if (ibont[k + 1, i] == 1)
                    { // Relations entre T1y et T2x:
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(ssp, npt, tabV);
                            v = tabV[k + 1];
                            t2 = v.X;
                        }
                        else
                        {
                            ToolLine.Tangency(ssp, npt, tabV2d);
                            v2d = tabV2d[k + 1];
                            t2 = v2d.X;
                        }
                        for (j = 1; j <= npol; j++)
                        {
                            daij = dda[j];
                            _cont[inc3, j + npol + incCol] = daij * t2;
                            _cont[inc3, j + ip + incCol] = -daij * t1;
                        }
                        incCol = incCol + ip;
                    }
                    else if (ibont[k + 1, i] == 2)
                    { // Relations entre T1y et T2y:
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(ssp, npt, tabV);
                            v = tabV[k + 1];
                            t2 = v.Y;
                        }
                        else
                        {
                            ToolLine.Tangency(ssp, npt, tabV2d);
                            v2d = tabV2d[k + 1];
                            t2 = v2d.Y;
                        }
                        for (j = 1; j <= npol; j++)
                        {
                            daij = dda[j];
                            _cont[inc3, j + npol + incCol] = daij * t2;
                            _cont[inc3, j + ip + npol + incCol] = -daij * t1;
                        }
                        incCol = incCol + ip;
                    }
                    else if (ibont[k + 1, i] == 3)
                    { // Relations entre T1y et T2z:
                        ToolLine.Tangency(ssp, npt, tabV);
                        v = tabV[k + 1];
                        t2 = v.Z;
                        for (j = 1; j <= npol; j++)
                        {
                            daij = dda[j];
                            _cont[inc3, j + npol + incCol] = daij * t2;
                            _cont[inc3, j + ip + 2 * npol + incCol] = -daij * t1;
                        }
                        incCol = incCol + ip;
                    }
                }

                else
                {
                    ToolLine.Tangency(ssp, npt, tabV);
                    v = tabV[k];
                    t1 = v.Z;
                    ip = 3 * npol;
                    if (ibont[k + 1, i] == 1)
                    { // Relations entre T1z et T2x:
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(ssp, npt, tabV);
                            v = tabV[k + 1];
                            t2 = v.X;
                        }
                        else
                        {
                            ToolLine.Tangency(ssp, npt, tabV2d);
                            v2d = tabV2d[k + 1];
                            t2 = v2d.X;
                        }
                        for (j = 1; j <= npol; j++)
                        {
                            daij = dda[j];
                            _cont[inc3, j + 2 * npol + incCol] = daij * t2;
                            _cont[inc3, j + ip + incCol] = -daij * t1;
                        }
                        incCol = incCol + ip;
                    }

                    else if (ibont[k + 1, i] == 2)
                    { // Relations entre T1z et T2y:
                        if ((k + 1) <= nb3d)
                        {
                            ToolLine.Tangency(ssp, npt, tabV);
                            v = tabV[k + 1];
                            t2 = v.Y;
                        }
                        else
                        {
                            ToolLine.Tangency(ssp, npt, tabV2d);
                            v2d = tabV2d[k + 1];
                            t2 = v2d.Y;
                        }
                        for (j = 1; j <= npol; j++)
                        {
                            daij = dda[j];
                            _cont[inc3, j + 2 * npol + incCol] = daij * t2;
                            _cont[inc3, j + ip + npol + incCol] = -daij * t1;
                        }
                        incCol = incCol + ip;
                    }

                    else if (ibont[k + 1, i] == 3)
                    { // Relations entre T1z et T2z:
                        ToolLine.Tangency(ssp, npt, tabV);
                        v = tabV[k + 1];
                        t2 = v.Z;
                        for (j = 1; j <= npol; j++)
                        {
                            daij = dda[j];
                            _cont[inc3, j + 2 * npol + incCol] = daij * t2;
                            _cont[inc3, j + ip + 2 * npol + incCol] = -daij * t1;
                        }
                        incCol = incCol + ip;
                    }
                }
            }
        }

        return _deCont;
    }
}
