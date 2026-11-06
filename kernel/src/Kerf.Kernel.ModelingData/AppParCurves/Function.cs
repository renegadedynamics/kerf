// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_Function.gxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_TheFunction.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Lpa, le 20/09/91

// Calcul de la valeur de F et grad_F, connaissant le parametrage.
// Cette fonction, appelee par le gradient conjugue, calcul F et
// DF(ui, Poles(ui)) ce qui implique un calcul des nouveaux poles
//  a chaque appel.

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.Constraint;

namespace Kerf.Engine.Approximation;

/// <summary>
/// The function F = sum ||C(ui) - Ptli||^2 of the parameters ui of a MultiLine's points, and its gradient,
/// minimised by <see cref="Gradient{MultiLine, ToolLine}"/>. Each evaluation computes new poles by
/// least squares (<see cref="LeastSquare{MultiLine, ToolLine}"/>) and, for interior constraints,
/// corrects them with <see cref="ResolConstraint{MultiLine, ToolLine}"/>.
/// </summary>
/// <remarks>
/// The generic form of OCCT's <c>AppParCurves_Function.gxx</c> (decision 142), whose <c>Squares</c> and
/// <c>ResolCons</c> arguments are always the LeastSquare and ResolConstraint instantiations over the same MultiLine
/// and tool. OCCT instantiates it as AppDef_TheFunction, AppDef_ParFunctionOf... and the GeomInt and BRepApprox
/// equivalents. The MultiLine is kept by reference (OCCT copies it; neither changes it).
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="ToolLine">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
internal class Function<MultiLine, ToolLine> : MultipleVarFunctionWithGradient
    where ToolLine : struct, ILineTool<MultiLine>
{
    // OCCT's Done field is written but never read; it is left out
    private readonly MultiLine _myMultiLine;
    private MultiCurve _myMultiCurve;
    private readonly int _degre;
    private readonly math_Vector _parameters;
    private double _fVal;
    private readonly math_Vector ValGrad_F;
    private readonly Matrix _myF;
    private readonly Matrix _ptlx;
    private readonly Matrix _ptly;
    private readonly Matrix _ptlz;
    private readonly Matrix _a;
    private readonly Matrix _da;
    private readonly LeastSquare<MultiLine, ToolLine> _myLeastSquare;
    private readonly bool _contraintes;
    private readonly int _nbP;
    private readonly int _nbCu;
    private readonly int _adeb;
    private readonly int _afin;
    private readonly NCollection_HArray1<int> _tabdim;
    private double _err3d;
    private double _err2d;
    private readonly int _firstP;
    private readonly int _lastP;
    private readonly NCollection_HArray1<ConstraintCouple> _constraints;

    /// <summary>
    /// The function for the points <paramref name="firstPoint"/> to <paramref name="lastPoint"/> of
    /// <paramref name="ssp"/>, approximated by Bezier curves of degree <paramref name="deg"/> under the constraints
    /// <paramref name="theConstraints"/>, starting from the parameters <paramref name="parameters"/> (copied).
    /// </summary>
    public Function(MultiLine ssp, int firstPoint, int lastPoint,
                                 NCollection_HArray1<ConstraintCouple> theConstraints, math_Vector parameters,
                                 int deg)
    {
        _myMultiLine = ssp;
        _myMultiCurve = new MultiCurve(deg + 1);
        _parameters = new math_Vector(parameters.Lower(), parameters.Upper());
        ValGrad_F = new math_Vector(firstPoint, lastPoint);
        _myF = new Matrix(firstPoint, lastPoint, 1, ToolLine.NbP3d(ssp) + ToolLine.NbP2d(ssp), 0.0);
        _ptlx = new Matrix(firstPoint, lastPoint, 1, ToolLine.NbP3d(ssp) + ToolLine.NbP2d(ssp), 0.0);
        _ptly = new Matrix(firstPoint, lastPoint, 1, ToolLine.NbP3d(ssp) + ToolLine.NbP2d(ssp), 0.0);
        _ptlz = new Matrix(firstPoint, lastPoint, 1, ToolLine.NbP3d(ssp) + ToolLine.NbP2d(ssp), 0.0);
        _a = new Matrix(firstPoint, lastPoint, 1, deg + 1);
        _da = new Matrix(firstPoint, lastPoint, 1, deg + 1);
        _myLeastSquare = new LeastSquare<MultiLine, ToolLine>(ssp,
                                                                          firstPoint,
                                                                          lastPoint,
                                                                          FirstConstraint(theConstraints, firstPoint),
                                                                          LastConstraint(theConstraints, lastPoint),
                                                                          deg + 1);

        int i;
        for (i = parameters.Lower(); i <= parameters.Upper(); i++)
        {
            _parameters[i] = parameters[i];
        }
        _firstP = firstPoint;
        _lastP = lastPoint;
        _constraints = theConstraints;
        _nbP = _lastP - _firstP + 1;
        _adeb = _firstP;
        _afin = _lastP;
        _degre = deg;
        _contraintes = false;
        int myindex;
        int low = theConstraints.Lower(), upp = theConstraints.Upper();
        ConstraintCouple mycouple;
        Constraint cons;

        for (i = low; i <= upp; i++)
        {
            mycouple = theConstraints.Value(i);
            cons = mycouple.Constraint();
            myindex = mycouple.Index();
            if (myindex == _firstP)
            {
                if ((int)cons >= 1)
                {
                    _adeb = _adeb + 1;
                }
            }
            else if (myindex == _lastP)
            {
                if ((int)cons >= 1)
                {
                    _afin = _afin - 1;
                }
            }
            else
            {
                if ((int)cons >= 1)
                {
                    _contraintes = true;
                }
            }
        }

        int nb3d = ToolLine.NbP3d(ssp);
        int nb2d = ToolLine.NbP2d(ssp);
        int mynb3d = nb3d, mynb2d = nb2d;
        if (nb3d == 0)
        {
            mynb3d = 1;
        }
        if (nb2d == 0)
        {
            mynb2d = 1;
        }

        _nbCu = nb3d + nb2d;
        _tabdim = new NCollection_HArray1<int>(0, _nbCu - 1);

        if (_contraintes)
        {
            for (i = 1; i <= _nbCu; i++)
            {
                if (i <= nb3d)
                {
                    _tabdim.SetValue(i - 1, 3);
                }
                else
                {
                    _tabdim.SetValue(i - 1, 2);
                }
            }

            NCollection_Array1<Point3D> tabP = new(1, mynb3d);
            NCollection_Array1<Point2D> tabP2d = new(1, mynb2d);

            for (i = _firstP; i <= _lastP; i++)
            {
                if (nb3d != 0 && nb2d != 0)
                {
                    ToolLine.Value(ssp, i, tabP, tabP2d);
                }
                else if (nb3d != 0)
                {
                    ToolLine.Value(ssp, i, tabP);
                }
                else
                {
                    ToolLine.Value(ssp, i, tabP2d);
                }
                for (int j = 1; j <= _nbCu; j++)
                {
                    if (_tabdim.Value(j - 1) == 3)
                    {
                        tabP[j].Coord(out _ptlx[i, j], out _ptly[i, j], out _ptlz[i, j]);
                    }
                    else
                    {
                        // OCCT: TabP2d(j) where j - nb3d is meant; out of range for a line with 3d and 2d points
                        // (unchecked in OCCT; the port throws)
                        tabP2d[j].Coord(out _ptlx[i, j], out _ptly[i, j]);
                    }
                }
            }
        }
    }

    /// <summary>The constraint of the point <paramref name="firstPoint"/> (the last couple's when it has none).</summary>
    public Constraint FirstConstraint(NCollection_HArray1<ConstraintCouple> theConstraints,
                                                   int firstPoint)
    {
        int i, myindex;
        int low = theConstraints.Lower(), upp = theConstraints.Upper();
        ConstraintCouple mycouple;
        Constraint cons = NoConstraint;

        for (i = low; i <= upp; i++)
        {
            mycouple = theConstraints.Value(i);
            cons = mycouple.Constraint();
            myindex = mycouple.Index();
            if (myindex == firstPoint)
            {
                break;
            }
        }
        return cons;
    }

    /// <summary>The constraint of the point <paramref name="lastPoint"/> (the last couple's when it has none).</summary>
    public Constraint LastConstraint(NCollection_HArray1<ConstraintCouple> theConstraints,
                                                  int lastPoint)
    {
        int i, myindex;
        int low = theConstraints.Lower(), upp = theConstraints.Upper();
        ConstraintCouple mycouple;
        Constraint cons = NoConstraint;

        for (i = low; i <= upp; i++)
        {
            mycouple = theConstraints.Value(i);
            cons = mycouple.Constraint();
            myindex = mycouple.Index();
            if (myindex == lastPoint)
            {
                break;
            }
        }
        return cons;
    }

    /// <summary>The value of F at the parameters <paramref name="x"/>; false (and F 0) if the resolution fails.</summary>
    public override bool Value(math_Vector x, out double f)
    {
        _parameters.Initialized(x);

        // Resolution moindres carres:
        // ===========================
        _myLeastSquare.Perform(_parameters);
        if (!(_myLeastSquare.IsDone))
        {
            f = 0.0;
            return false;
        }
        if (!_contraintes)
        {
            _myLeastSquare.Error(out _fVal, out _err3d, out _err2d);
            f = _fVal;
        }

        // Resolution avec contraintes:
        // ============================
        else
        {
            int npol = _degre + 1;
            int ci, i, j, dimen;
            double aa, bb, cc, aij, fx, fy, fz, fi;
            math_Vector ptcxci = new(1, npol), ptcyci = new(1, npol), ptczci = new(1, npol);
            _err3d = _err2d = 0.0;

            _myMultiCurve = _myLeastSquare.BezierValue();

            _a.Initialized(_myLeastSquare.FunctionMatrix());
            ResolConstraint<MultiLine, ToolLine> resol = new(_myMultiLine,
                                                                          _myMultiCurve,
                                                                          _firstP,
                                                                          _lastP,
                                                                          _constraints,
                                                                          _a,
                                                                          _myLeastSquare.DerivativeFunctionMatrix());
            if (!resol.IsDone)
            {
                f = 0.0;
                return false;
            }

            // Calcul de F = Sum||C(ui)-Ptli||2  sur toutes les courbes :
            // ========================================================================
            _fVal = 0.0;

            for (ci = 1; ci <= _nbCu; ci++)
            {
                dimen = _tabdim.Value(ci - 1);
                for (j = 1; j <= npol; j++)
                {
                    if (dimen == 3)
                    {
                        _myMultiCurve.Value(j).Point(ci).Coord(out ptcxci[j], out ptcyci[j], out ptczci[j]);
                    }
                    else
                    {
                        _myMultiCurve.Value(j).Point2d(ci).Coord(out ptcxci[j], out ptcyci[j]);
                    }
                }

                // Calcul de F:
                // ============
                for (i = _adeb; i <= _afin; i++)
                {
                    aa = 0.0;
                    bb = 0.0;
                    cc = 0.0;
                    for (j = 1; j <= npol; j++)
                    {
                        aij = _a[i, j];
                        aa += aij * ptcxci[j];
                        bb += aij * ptcyci[j];
                        if (dimen == 3)
                        {
                            cc += aij * ptczci[j];
                        }
                    }
                    fx = aa - _ptlx[i, ci];
                    fy = bb - _ptly[i, ci];
                    _myF[i, ci] = fx * fx + fy * fy;
                    if (dimen == 3)
                    {
                        fz = cc - _ptlz[i, ci];
                        _myF[i, ci] += fz * fz;
                        fi = _myF[i, ci];
                        if (Math.Sqrt(fi) > _err3d)
                        {
                            _err3d = Math.Sqrt(fi);
                        }
                    }
                    else
                    {
                        fi = _myF[i, ci];
                        if (Math.Sqrt(fi) > _err2d)
                        {
                            _err2d = Math.Sqrt(fi);
                        }
                    }
                    _fVal += fi;
                }
            }
            f = _fVal;
        }
        return true;
    }

    /// <summary>Computes F and its gradient at <paramref name="x"/>.</summary>
    protected void Perform(math_Vector x)
    {
        int j;

        _parameters.Initialized(x);
        // Resolution moindres carres:
        // ===========================
        _myLeastSquare.Perform(_parameters);

        if (!(_myLeastSquare.IsDone))
        {
            return;
        }

        for (j = _parameters.Lower(); j <= _parameters.Upper(); j++)
        {
            ValGrad_F[j] = 0.0;
        }

        if (!_contraintes)
        {
            _myLeastSquare.ErrorGradient(ValGrad_F, out _fVal, out _err3d, out _err2d);
        }
        else
        {
            int pi, ci, i, k, dimen;
            int npol = _degre + 1;
            double scal, aa, bb, cc, daa, dbb, dcc;
            double fx, fy, fz, aij, daij, px, py, pz, fi;
            Constraint cons = NoConstraint;
            Matrix Grad_F = new(_firstP, _lastP, 1, _nbCu, 0.0);
            math_Vector ptcxci = new(1, npol), ptcyci = new(1, npol), ptczci = new(1, npol);
            math_Vector ptcoxci = new(1, npol), ptcoyci = new(1, npol), ptcozci = new(1, npol);
            _err3d = _err2d = 0.0;

            Matrix ptcox = new(1, npol, 1, _nbCu), ptcoy = new(1, npol, 1, _nbCu), ptcoz = new(1, npol, 1, _nbCu);
            Matrix ptcx = new(1, npol, 1, _nbCu), ptcy = new(1, npol, 1, _nbCu), ptcz = new(1, npol, 1, _nbCu);
            int inc;

            _myMultiCurve = _myLeastSquare.BezierValue();

            for (ci = 1; ci <= _nbCu; ci++)
            {
                dimen = _tabdim.Value(ci - 1);
                for (j = 1; j <= npol; j++)
                {
                    if (dimen == 3)
                    {
                        _myMultiCurve.Value(j).Point(ci).Coord(out ptcox[j, ci], out ptcoy[j, ci], out ptcoz[j, ci]);
                    }
                    else
                    {
                        _myMultiCurve.Value(j).Point2d(ci).Coord(out ptcox[j, ci], out ptcoy[j, ci]);
                        ptcoz[j, ci] = 0.0;
                    }
                }
            }

            _a.Initialized(_myLeastSquare.FunctionMatrix());
            _da.Initialized(_myLeastSquare.DerivativeFunctionMatrix());

            // Resolution avec contraintes:
            // ============================

            ResolConstraint<MultiLine, ToolLine> resol = new(_myMultiLine, _myMultiCurve, _firstP, _lastP,
                                                                          _constraints, _a, _da);
            if (!resol.IsDone)
            {
                return;
            }

            // Calcul de F = Sum||C(ui)-Ptli||2 et du gradient non contraint de F pour
            // chaque point PointIndex.
            // ========================================================================
            _fVal = 0.0;
            for (j = _firstP; j <= _lastP; j++)
            {
                ValGrad_F[j] = 0.0;
            }

            Matrix trA = new(_a.LowerCol(), _a.UpperCol(), _a.LowerRow(), _a.UpperRow());
            Matrix trDA = new(_da.LowerCol(), _da.UpperCol(), _da.LowerRow(), _da.UpperRow());
            Matrix restm = new(_a.LowerCol(), _a.UpperCol(), _a.LowerCol(), _a.UpperCol());

            Matrix K = resol.ConstraintMatrix();
            Matrix dk = resol.ConstraintDerivative(_myMultiLine, x, _degre, _da);
            Matrix tk = new(K.LowerCol(), K.UpperCol(), K.LowerRow(), K.UpperRow());
            tk.Initialized(K.Transposed());
            math_Vector vardua = resol.Duale();
            Matrix kk = new(K.LowerCol(), K.UpperCol(), vardua.Lower(), vardua.Upper());
            kk.Initialized((K.Transposed()) * (resol.InverseMatrix()));
            Matrix dtk = new(dk.LowerCol(), dk.UpperCol(), dk.LowerRow(), dk.UpperRow());
            dtk.Initialized(dk.Transposed());
            trA.Initialized(_a.Transposed());
            trDA.Initialized(_da.Transposed());
            restm.Initialized((_a.Transposed() * _a).Inverse());

            math_Vector dptco = new(1, K.ColNumber());
            Matrix dptco1 = new(_firstP, _lastP, 1, K.ColNumber());
            math_Vector dkptc = new(1, K.RowNumber());

            _fVal = 0.0;
            for (ci = 1; ci <= _nbCu; ci++)
            {
                dimen = _tabdim.Value(ci - 1);
                for (j = 1; j <= npol; j++)
                {
                    if (dimen == 3)
                    {
                        _myMultiCurve.Value(j).Point(ci).Coord(out ptcx[j, ci], out ptcy[j, ci], out ptcz[j, ci]);
                    }
                    else
                    {
                        _myMultiCurve.Value(j).Point2d(ci).Coord(out ptcx[j, ci], out ptcy[j, ci]);
                        ptcz[j, ci] = 0.0;
                    }
                }
            }

            // Calcul du gradient sans contraintes:
            // ====================================

            for (ci = 1; ci <= _nbCu; ci++)
            {
                dimen = _tabdim.Value(ci - 1);
                for (i = _adeb; i <= _afin; i++)
                {
                    aa = 0.0;
                    bb = 0.0;
                    cc = 0.0;
                    daa = 0.0;
                    dbb = 0.0;
                    dcc = 0.0;
                    for (j = 1; j <= npol; j++)
                    {
                        aij = _a[i, j];
                        daij = _da[i, j];
                        px = ptcx[j, ci];
                        py = ptcy[j, ci];
                        aa += aij * px;
                        bb += aij * py;
                        daa += daij * px;
                        dbb += daij * py;
                        if (dimen == 3)
                        {
                            pz = ptcz[j, ci];
                            cc += aij * pz;
                            dcc += daij * pz;
                        }
                    }
                    fx = aa - _ptlx[i, ci];
                    fy = bb - _ptly[i, ci];
                    _myF[i, ci] = fx * fx + fy * fy;
                    Grad_F[i, ci] = 2.0 * (daa * fx + dbb * fy);
                    if (dimen == 3)
                    {
                        fz = cc - _ptlz[i, ci];
                        _myF[i, ci] += fz * fz;
                        Grad_F[i, ci] += 2.0 * dcc * fz;
                        fi = _myF[i, ci];
                        if (Math.Sqrt(fi) > _err3d)
                        {
                            _err3d = Math.Sqrt(fi);
                        }
                    }
                    else
                    {
                        fi = _myF[i, ci];
                        if (Math.Sqrt(fi) > _err2d)
                        {
                            _err2d = Math.Sqrt(fi);
                        }
                    }
                    _fVal += fi;
                    ValGrad_F[i] += Grad_F[i, ci];
                }
            }

            // Calcul de DK*PTC:
            // =================
            for (i = 1; i <= K.RowNumber(); i++)
            {
                inc = 0;
                for (ci = 1; ci <= _nbCu; ci++)
                {
                    dimen = _tabdim.Value(ci - 1);
                    dkptc[i] = 0.0;
                    for (j = 1; j <= npol; j++)
                    {
                        dkptc[i] += dk[i, j + inc] * ptcx[j, ci] + dk[i, j + inc + npol] * ptcy[j, ci];
                        if (dimen == 3)
                        {
                            dkptc[i] += dk[i, j + inc + 2 * npol] * ptcz[j, ci];
                        }
                    }
                    if (dimen == 3)
                    {
                        inc = inc + 3 * npol;
                    }
                    else
                    {
                        inc = inc + 2 * npol;
                    }
                }
            }

            math_Vector derr = new(dtk.LowerRow(), dtk.UpperRow());
            derr.Initialized((dtk) * vardua - kk * ((dkptc) + K * (dtk) * vardua));

            // rajout du gradient avec contraintes:
            // ====================================
            // dPTCO1/duk = [d(TA)/duk*[A*PTCO-PTL] + TA*dA/duk*PTCO]

            inc = 0;

            math_Vector errx = new(_a.LowerRow(), _a.UpperRow());
            math_Vector erry = new(_a.LowerRow(), _a.UpperRow());
            math_Vector errz = new(_a.LowerRow(), _a.UpperRow());
            math_Vector scalx = new(_da.LowerRow(), _da.UpperRow());
            math_Vector scaly = new(_da.LowerRow(), _da.UpperRow());
            math_Vector scalz = new(_da.LowerRow(), _da.UpperRow());
            math_Vector erruzax = new(ptcxci.Lower(), ptcxci.Upper());
            math_Vector erruzay = new(ptcyci.Lower(), ptcyci.Upper());
            math_Vector erruzaz = new(ptczci.Lower(), ptczci.Upper());
            math_Vector trDAPI = new(trDA.LowerRow(), trDA.UpperRow());
            math_Vector trAPI = new(trA.LowerRow(), trA.UpperRow());

            for (ci = 1; ci <= _nbCu; ci++)
            {
                dimen = _tabdim.Value(ci - 1);
                ptcoxci.Initialized(ptcox.Col(ci));
                ptcoyci.Initialized(ptcoy.Col(ci));
                ptcozci.Initialized(ptcoz.Col(ci));
                ptcxci.Initialized(ptcx.Col(ci));
                ptcyci.Initialized(ptcy.Col(ci));
                ptczci.Initialized(ptcz.Col(ci));

                errx.Initialized(_a * ptcoxci - _ptlx.Col(ci));
                erry.Initialized(_a * ptcoyci - _ptly.Col(ci));
                errz.Initialized(_a * ptcozci - _ptlz.Col(ci));
                scalx.Initialized(_da * ptcoxci); // Scal = DA * PTCO
                scaly.Initialized(_da * ptcoyci);
                scalz.Initialized(_da * ptcozci);
                erruzax.Initialized(ptcxci - ptcoxci);
                erruzay.Initialized(ptcyci - ptcoyci);
                erruzaz.Initialized(ptczci - ptcozci);

                for (pi = _firstP; pi <= _lastP; pi++)
                {
                    trDAPI.Initialized(trDA.Col(pi));
                    trAPI.Initialized(trA.Col(pi));
                    double taa = trAPI * _a.Row(pi);
                    scal = 0.0;
                    for (j = 1; j <= npol; j++)
                    {
                        dptco1[pi, j + inc] = (trDAPI * errx[pi] + trAPI * scalx[pi])[j];
                        dptco1[pi, j + inc + npol] = (trDAPI * erry[pi] + trAPI * scaly[pi])[j];
                        scal += dptco1[pi, j + inc] * taa * erruzax[j] + dptco1[pi, j + inc + npol] * taa * erruzay[j];
                        if (dimen == 3)
                        {
                            dptco1[pi, j + inc + 2 * npol] = (trDAPI * errz[pi] + trAPI * scalz[pi])[j];
                            scal += dptco1[pi, j + inc + 2 * npol] * taa * erruzaz[j];
                        }
                    }
                    ValGrad_F[pi] = ValGrad_F[pi] - 2 * scal;
                }
                if (dimen == 3)
                {
                    inc = inc + 3 * npol;
                }
                else
                {
                    inc = inc + 2 * npol;
                }
            }

            // on calcule DPTCO = - RESTM * DPTCO1:

            // Calcul de DPTCO/duk:
            // dPTCO/duk = -Inv(T(A)*A)*[d(TA)/duk*[A*PTCO-PTL] + TA*dA/duk*PTCO]

            int low = _constraints.Lower(), upp = _constraints.Upper();
            inc = 0;
            for (pi = _firstP; pi <= _lastP; pi++)
            {
                for (i = low; i <= upp; i++)
                {
                    if (_constraints.Value(i).Index() == pi)
                    {
                        cons = _constraints.Value(i).Constraint();
                        break;
                    }
                }
                if ((int)cons >= 1)
                {
                    inc = 0;
                    for (ci = 1; ci <= _nbCu; ci++)
                    {
                        dimen = _tabdim.Value(ci - 1);
                        for (j = 1; j <= npol; j++)
                        {
                            dptco[j + inc] = 0.0;
                            dptco[j + inc + npol] = 0.0;
                            if (dimen == 3)
                            {
                                dptco[j + inc + 2 * npol] = 0.0;
                            }
                            for (k = 1; k <= npol; k++)
                            {
                                // OCCT: the product reads DPTCO1(Pi, j + ...) where k + ... is meant
                                dptco[j + inc] = dptco[j + inc] - restm[j, k] * dptco1[pi, j + inc];
                                dptco[j + inc + npol] = dptco[j + inc + npol] - restm[j, k] * dptco1[pi, j + inc + npol];
                                if (dimen == 3)
                                {
                                    dptco[j + inc + 2 * npol] =
                                        dptco[j + inc + 2 * npol] - restm[j, k] * dptco1[pi, j + inc + 2 * npol];
                                }
                            }
                        }
                        if (dimen == 3)
                        {
                            inc += 3 * npol;
                        }
                        else
                        {
                            inc += 2 * npol;
                        }
                    }

                    derr.Initialized(derr - kk * K * dptco);

                    inc = 0;
                    for (ci = 1; ci <= _nbCu; ci++)
                    {
                        dimen = _tabdim.Value(ci - 1);
                        ptcoxci.Initialized(ptcox.Col(ci));
                        ptcoyci.Initialized(ptcoy.Col(ci));
                        ptcozci.Initialized(ptcoz.Col(ci));
                        ptcxci.Initialized(ptcx.Col(ci));
                        ptcyci.Initialized(ptcy.Col(ci));
                        ptczci.Initialized(ptcz.Col(ci));
                        erruzax.Initialized(ptcxci - ptcoxci);
                        erruzay.Initialized(ptcyci - ptcoyci);
                        erruzaz.Initialized(ptczci - ptcozci);
                        scal = 0.0;

                        for (j = 1; j <= npol; j++)
                        {
                            // OCCT: = where += is meant, so only the last pole counts; and the z term uses Erruzax
                            scal = (_a[pi, j] * erruzax[j]) * (_a[pi, j] * derr[j + inc])
                                   + (_a[pi, j] * erruzay[j]) * (_a[pi, j] * derr[j + inc + npol]);
                            if (dimen == 3)
                            {
                                scal += (_a[pi, j] * erruzax[j]) * (_a[pi, j] * derr[j + inc + 2 * npol]);
                            }
                        }

                        ValGrad_F[pi] = ValGrad_F[pi] + 2 * scal;
                        if (dimen == 3)
                        {
                            inc = inc + 3 * npol;
                        }
                        else
                        {
                            inc = inc + 2 * npol;
                        }
                    }
                }
            }
        }
    }

    /// <summary>The number of variables: the number of points.</summary>
    public override int VariableCount => _nbP;

    /// <summary>The gradient of F at <paramref name="x"/>.</summary>
    public override bool Gradient(math_Vector x, math_Vector g)
    {
        Perform(x);
        g.Initialized(ValGrad_F);

        return true;
    }

    /// <summary>The value and gradient of F at <paramref name="x"/>.</summary>
    public override bool Values(math_Vector x, out double f, math_Vector g)
    {
        Perform(x);
        f = _fVal;
        g.Initialized(ValGrad_F);
        return true;
    }

    /// <summary>The current curves (the object's own MultiCurve; shared: don't modify).</summary>
    public MultiCurve CurveValue()
    {
        if (!_contraintes)
        {
            _myMultiCurve = _myLeastSquare.BezierValue();
        }
        return _myMultiCurve;
    }

    /// <summary>The distance between the point <paramref name="iPoint"/> and the curve <paramref name="curveIndex"/>.</summary>
    public double Error(int iPoint, int curveIndex) => Math.Sqrt(_myF[iPoint, curveIndex]);

    /// <summary>The largest 3d distance.</summary>
    public double MaxError3d() => _err3d;

    /// <summary>The largest 2d distance.</summary>
    public double MaxError2d() => _err2d;

    /// <summary>The parameters of the last evaluation (shared: don't modify).</summary>
    public math_Vector NewParameters() => _parameters;
}
