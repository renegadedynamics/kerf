// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_BSpFunction.gxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_BSpParFunctionOfMyBSplGradientOfBSplineCompute.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Approximation.Constraint;

namespace Kerf.Engine.Approximation;

/// <summary>
/// The function F = sum ||C(ui) - Ptli||^2 of the parameters ui of a MultiLine's points, and its gradient, for
/// B-spline curves with given knots: each evaluation computes new poles by least squares
/// (<see cref="LeastSquare{MultiLine, ToolLine}"/> with knots). Minimised by
/// <see cref="BSpGradient{MultiLine, ToolLine}"/>.
/// </summary>
/// <remarks>
/// The generic form of OCCT's <c>AppParCurves_BSpFunction.gxx</c> (decision 142), whose <c>Squares</c> argument
/// is always the LeastSquare instantiation over the same MultiLine and tool. OCCT instantiates it as
/// AppDef_BSpParFunctionOf... and the GeomInt and BRepApprox equivalents. Interior constraints are not
/// implemented (OCCT's branches for them are empty). The tangency values <see cref="SetFirstLambda"/> and
/// <see cref="SetLastLambda"/> are uninitialised in OCCT until set; here they start at 0.
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="ToolLine">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
internal class BSpFunction<MultiLine, ToolLine> : MultipleVarFunctionWithGradient
    where ToolLine : struct, ILineTool<MultiLine>
{
    // OCCT's Done field is written but never read; it is left out
    private readonly MultiLine _myMultiLine;
    private MultiBSpCurve _myMultiBSpCurve;
    private readonly int _nbpoles;
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
    private double _mylambda1;
    private double _mylambda2;

    /// <summary>
    /// The function for the points <paramref name="firstPoint"/> to <paramref name="lastPoint"/> of
    /// <paramref name="ssp"/>, approximated by B-spline curves of <paramref name="nbPol"/> poles with the knots
    /// <paramref name="knots"/> and multiplicities <paramref name="mults"/> under the constraints
    /// <paramref name="theConstraints"/>, starting from the parameters <paramref name="parameters"/> (copied).
    /// </summary>
    public BSpFunction(MultiLine ssp, int firstPoint, int lastPoint,
                                    NCollection_HArray1<ConstraintCouple> theConstraints,
                                    math_Vector parameters, NCollection_Array1<double> knots, NCollection_Array1<int> mults,
                                    int nbPol)
    {
        _myMultiLine = ssp;
        _myMultiBSpCurve = new MultiBSpCurve(nbPol);
        _parameters = new math_Vector(parameters.Lower(), parameters.Upper());
        ValGrad_F = new math_Vector(firstPoint, lastPoint);
        _myF = new Matrix(firstPoint, lastPoint, 1, ToolLine.NbP3d(ssp) + ToolLine.NbP2d(ssp), 0.0);
        _ptlx = new Matrix(firstPoint, lastPoint, 1, ToolLine.NbP3d(ssp) + ToolLine.NbP2d(ssp), 0.0);
        _ptly = new Matrix(firstPoint, lastPoint, 1, ToolLine.NbP3d(ssp) + ToolLine.NbP2d(ssp), 0.0);
        _ptlz = new Matrix(firstPoint, lastPoint, 1, ToolLine.NbP3d(ssp) + ToolLine.NbP2d(ssp), 0.0);
        _a = new Matrix(firstPoint, lastPoint, 1, nbPol);
        _da = new Matrix(firstPoint, lastPoint, 1, nbPol);
        _myLeastSquare = new LeastSquare<MultiLine, ToolLine>(ssp,
                                                                          knots,
                                                                          mults,
                                                                          firstPoint,
                                                                          lastPoint,
                                                                          FirstConstraint(theConstraints, firstPoint),
                                                                          LastConstraint(theConstraints, lastPoint),
                                                                          nbPol);

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
        _nbpoles = nbPol;
        _myMultiBSpCurve.SetKnots(knots);
        _myMultiBSpCurve.SetMultiplicities(mults);
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

    /// <summary>
    /// The value of F at the parameters <paramref name="x"/>; false (and F 0) if the resolution fails. With interior
    /// constraints F is not computed (as in OCCT) and keeps its previous value.
    /// </summary>
    public override bool Value(math_Vector x, out double f)
    {
        _parameters.Initialized(x);

        // Resolution moindres carres:
        // ===========================
        _myLeastSquare.Perform(_parameters, _mylambda1, _mylambda2);
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
            // Not implemented in OCCT, where F is left unchanged
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
        _myLeastSquare.Perform(_parameters, _mylambda1, _mylambda2);

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
    }

    /// <summary>Sets the tangency value at the first point.</summary>
    public void SetFirstLambda(double l1) => _mylambda1 = l1;

    /// <summary>Sets the tangency value at the last point.</summary>
    public void SetLastLambda(double l2) => _mylambda2 = l2;

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

    /// <summary>The current curves: a copy sharing the poles, knots and multiplicities (OCCT's by-value copy).</summary>
    public MultiBSpCurve CurveValue()
    {
        if (!_contraintes)
        {
            _myMultiBSpCurve = new MultiBSpCurve(_myLeastSquare.BSplineValue());
        }
        return new MultiBSpCurve(_myMultiBSpCurve);
    }

    /// <summary>The distance between the point <paramref name="iPoint"/> and the curve <paramref name="curveIndex"/>.</summary>
    public double Error(int iPoint, int curveIndex)
    {
        Matrix dd = _myLeastSquare.Distance();
        double d = dd.Value(iPoint, curveIndex);
        if (!_contraintes)
        {
            return d;
        }
        else
        {
            return Math.Sqrt(_myF[iPoint, curveIndex]);
        }
    }

    /// <summary>The largest 3d distance.</summary>
    public double MaxError3d() => _err3d;

    /// <summary>The largest 2d distance.</summary>
    public double MaxError2d() => _err2d;

    /// <summary>The parameters of the last evaluation (shared: don't modify).</summary>
    public math_Vector NewParameters() => _parameters;

    /// <summary>The function matrix of the least squares (shared: don't modify).</summary>
    public Matrix FunctionMatrix() => _myLeastSquare.FunctionMatrix();

    /// <summary>The derivative function matrix of the least squares (shared: don't modify).</summary>
    public Matrix DerivativeFunctionMatrix() => _myLeastSquare.DerivativeFunctionMatrix();

    /// <summary>The index of the first non-zero basis function at each point (see KIndex; shared: don't modify).</summary>
    public math_IntegerVector Index() => _myLeastSquare.KIndex();
}
