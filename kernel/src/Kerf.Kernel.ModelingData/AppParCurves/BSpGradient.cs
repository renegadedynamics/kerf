// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_BSpGradient.gxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MyBSplGradientOfBSplineCompute.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// lpa, le 11/09/91

// Application of the corrected gradient method to minimize
// F  = sum(||C(ui, Poles(ui)) - ptli||2.
// The conjugate gradient method is implemented in the mathematical
// library: math_BFGS.

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Approximation.Constraint;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Approximates a MultiLine by B-spline curves with given knots, optimising the points' parameters: one
/// projection iteration (Rogers and Fog), then a BFGS minimisation of
/// <see cref="BSpFunction{MultiLine, ToolLine}"/>.
/// </summary>
/// <remarks>
/// The generic form of OCCT's <c>AppParCurves_BSpGradient.gxx</c> (decision 142), instantiated by OCCT as
/// AppDef_MyBSplGradientOfBSplineCompute and the GeomInt and BRepApprox equivalents. Its <c>BSpParLeastSquare</c>,
/// <c>BSpParFunction</c> and <c>BSpGradient_BFGS</c> arguments are always the LeastSquare, BSpFunction and
/// BSpGradient_BFGS instantiations over the same MultiLine and tool. The computation runs in the constructor, which
/// also writes the optimised parameters into <c>Parameters</c>.
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="ToolLine">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
internal class BSpGradient<MultiLine, ToolLine>
    where ToolLine : struct, ILineTool<MultiLine>
{
    private MultiBSpCurve _scu = new();
    private readonly math_Vector _parError;
    private double _avError;
    private double _mError3d;
    private double _mError2d;
    private double _mylambda1;
    private double _mylambda2;
    private readonly bool _isLambdaDefined;
    private bool _done;

    private static Constraint FirstConstraint(NCollection_HArray1<ConstraintCouple> theConstraints,
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

    private static Constraint LastConstraint(NCollection_HArray1<ConstraintCouple> theConstraints,
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
    /// Approximates the points <paramref name="firstPoint"/> to <paramref name="lastPoint"/> of
    /// <paramref name="ssp"/> by B-spline curves of degree <paramref name="deg"/> with the knots
    /// <paramref name="knots"/> and multiplicities <paramref name="mults"/> under the constraints
    /// <paramref name="theConstraints"/>, from the parameters <paramref name="parameters"/>, which receive the
    /// optimised parameters. End tangency values are computed by a first least-squares fit.
    /// </summary>
    public BSpGradient(MultiLine ssp, int firstPoint, int lastPoint,
                                    NCollection_HArray1<ConstraintCouple> theConstraints, math_Vector parameters,
                                    NCollection_Array1<double> knots, NCollection_Array1<int> mults, int deg, double tol3d,
                                    double tol2d, int nbIterations = 1)
    {
        _parError = new math_Vector(firstPoint, lastPoint, 0.0);
        _mylambda1 = 0.0;
        _mylambda2 = 0.0;
        _isLambdaDefined = false;
        Perform(ssp, firstPoint, lastPoint, theConstraints, parameters, knots, mults, deg, tol3d, tol2d, nbIterations);
    }

    /// <summary>As the other constructor, with the end tangency values <paramref name="lambda1"/> and <paramref name="lambda2"/> given.</summary>
    public BSpGradient(MultiLine ssp, int firstPoint, int lastPoint,
                                    NCollection_HArray1<ConstraintCouple> theConstraints, math_Vector parameters,
                                    NCollection_Array1<double> knots, NCollection_Array1<int> mults, int deg, double tol3d,
                                    double tol2d, int nbIterations, double lambda1, double lambda2)
    {
        _parError = new math_Vector(firstPoint, lastPoint, 0.0);
        _mylambda1 = lambda1;
        _mylambda2 = lambda2;
        _isLambdaDefined = true;
        Perform(ssp, firstPoint, lastPoint, theConstraints, parameters, knots, mults, deg, tol3d, tol2d, nbIterations);
    }

    /// <summary>Runs the approximation (called by the constructors).</summary>
    protected void Perform(MultiLine ssp, int firstPoint, int lastPoint,
                           NCollection_HArray1<ConstraintCouple> theConstraints, math_Vector parameters,
                           NCollection_Array1<double> knots, NCollection_Array1<int> mults, int deg, double tol3d,
                           double tol2d, int nbIterations = 200)
    {
        int i, j, k, i2, l;
        double uf, du, fval = 0.0, fu, dfu;
        int nbP3d = ToolLine.NbP3d(ssp);
        int nbP2d = ToolLine.NbP2d(ssp);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        int nbP = nbP3d + nbP2d;
        Point3D pt = new();
        Point2D pt2d = new();
        Vector3D v1 = new(), myV;
        Vector2D v12d = new(), myV2d;
        _done = false;

        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        NCollection_Array1<Point3D> tabP = new(1, mynbP3d);
        NCollection_Array1<Point2D> tabP2d = new(1, mynbP2d);

        // Calculation of the function F= sum(||C(ui)-Ptli||2):
        // Call to a function inheriting from MultipleVarFunctionWithGradient
        // to compute F and grad_F.
        // ================================================================

        int nbpoles = -deg - 1;
        for (i = mults.Lower(); i <= mults.Upper(); i++)
        {
            nbpoles += mults[i];
        }

        NCollection_Array1<Point3D> tabPole = new(1, nbpoles);
        NCollection_Array1<Point2D> tabPole2d = new(1, nbpoles);
        NCollection_Array1<Point3D> thePoles = new(1, (nbpoles) * mynbP3d);
        NCollection_Array1<Point2D> thePoles2d = new(1, (nbpoles) * mynbP2d);

        Constraint firstCons = FirstConstraint(theConstraints, firstPoint),
                                lastCons = LastConstraint(theConstraints, lastPoint);

        BSpFunction<MultiLine, ToolLine> myF =
            new(ssp, firstPoint, lastPoint, theConstraints, parameters, knots, mults, nbpoles);

        if (firstCons >= TangencyPoint || lastCons >= TangencyPoint)
        {
            if (!_isLambdaDefined)
            {
                LeastSquare<MultiLine, ToolLine> thefitt =
                    new(ssp, knots, mults, firstPoint, lastPoint, firstCons, lastCons, parameters, nbpoles);
                if (firstCons >= TangencyPoint)
                {
                    _mylambda1 = thefitt.FirstLambda();
                    myF.SetFirstLambda(_mylambda1);
                }
                if (lastCons >= TangencyPoint)
                {
                    _mylambda2 = thefitt.LastLambda();
                    myF.SetLastLambda(_mylambda2);
                }
            }
            else
            {
                myF.SetFirstLambda(_mylambda1);
                myF.SetLastLambda(_mylambda2);
            }
        }

        myF.Value(parameters, out fval);
        _mError3d = myF.MaxError3d();
        _mError2d = myF.MaxError2d();
        _scu = myF.CurveValue();

        if (_mError3d > tol3d || _mError2d > tol2d)
        {
            // Storage of curve poles for projection:
            // ============================================
            i2 = 0;
            for (k = 1; k <= nbP3d; k++)
            {
                _scu.Curve(k, tabPole);
                for (j = 1; j <= nbpoles; j++)
                {
                    thePoles[j + i2] = tabPole[j];
                }
                i2 += nbpoles;
            }
            i2 = 0;
            for (k = 1; k <= nbP2d; k++)
            {
                _scu.Curve(nbP3d + k, tabPole2d);
                for (j = 1; j <= nbpoles; j++)
                {
                    thePoles2d[j + i2] = tabPole2d[j];
                }
                i2 += nbpoles;
            }

            //  Une iteration rapide de projection est faite par la methode de
            //  Rogers & Fog 89, methode equivalente a Hoschek 88 qui ne necessite pas
            //  le calcul de D2.

            Matrix A = myF.FunctionMatrix();
            Matrix DA = myF.DerivativeFunctionMatrix();
            math_IntegerVector index = myF.Index();
            double aa, da, a, b, c, d, e, f, px, py, pz;
            int indexdeb, indexfin;

            for (j = firstPoint + 1; j <= lastPoint - 1; j++)
            {
                uf = parameters[j];
                if (nbP3d != 0 && nbP2d != 0)
                {
                    ToolLine.Value(ssp, j, tabP, tabP2d);
                }
                else if (nbP2d != 0)
                {
                    ToolLine.Value(ssp, j, tabP2d);
                }
                else
                {
                    ToolLine.Value(ssp, j, tabP);
                }

                fu = 0.0;
                dfu = 0.0;
                i2 = 0;

                indexdeb = index[j] + 1;
                indexfin = indexdeb + deg;

                for (k = 1; k <= nbP3d; k++)
                {
                    a = b = c = d = e = f = 0.0;
                    for (l = indexdeb; l <= indexfin; l++)
                    {
                        pt = thePoles[l + i2];
                        px = pt.X;
                        py = pt.Y;
                        pz = pt.Z;
                        aa = A[j, l];
                        da = DA[j, l];
                        a += aa * px;
                        d += da * px;
                        b += aa * py;
                        e += da * py;
                        c += aa * pz;
                        f += da * pz;
                    }
                    pt = pt.WithCoord(a, b, c);
                    v1 = v1.WithCoord(d, e, f);
                    i2 += nbpoles;

                    myV = new Vector3D(pt, tabP[k]);
                    fu += myV * v1;
                    dfu += v1.SquareMagnitude;
                }
                i2 = 0;
                for (k = 1; k <= nbP2d; k++)
                {
                    a = b = d = e = 0.0;
                    for (l = indexdeb; l <= indexfin; l++)
                    {
                        pt2d = thePoles2d[l + i2];
                        px = pt2d.X;
                        py = pt2d.Y;
                        aa = A[j, l];
                        da = DA[j, l];
                        a += aa * px;
                        d += da * px;
                        b += aa * py;
                        e += da * py;
                    }
                    pt2d = pt2d.WithCoord(a, b);
                    v12d = v12d.WithCoord(d, e);
                    i2 += nbpoles;

                    myV2d = new Vector2D(pt2d, tabP2d[k]);
                    fu += myV2d * v12d;
                    dfu += v12d.SquareMagnitude;
                }

                if (dfu >= RealEpsilon())
                {
                    du = fu / dfu;
                    du = Math.CopySign(Min(5.0e-02, Math.Abs(du)), du);
                    uf += du;
                    parameters[j] = uf;
                }
            }

            myF.Value(parameters, out fval);
            _mError3d = myF.MaxError3d();
            _mError2d = myF.MaxError2d();
        }

        if (_mError3d <= tol3d && _mError2d <= tol2d)
        {
            _done = true;
        }
        else if (nbIterations != 0)
        {
            // NbIterations de gradient conjugue:
            // =================================
            double eps = 1.0e-07;
            BSpGradientBfgs<MultiLine, ToolLine> fResol = new(myF, parameters, tol3d, tol2d, eps, nbIterations);
        }

        _scu = myF.CurveValue();

        _avError = 0.0;
        for (j = firstPoint; j <= lastPoint; j++)
        {
            parameters[j] = myF.NewParameters()[j];
            // Recherche des erreurs maxi et moyenne a un index donne:
            for (k = 1; k <= nbP; k++)
            {
                _parError[j] = Max(_parError[j], myF.Error(j, k));
            }
            _avError += _parError[j];
        }
        _avError = _avError / (lastPoint - firstPoint + 1);

        _mError3d = myF.MaxError3d();
        _mError2d = myF.MaxError2d();
        if (_mError3d <= tol3d && _mError2d <= tol2d)
        {
            _done = true;
        }
    }

    /// <summary>The approximating curves: a copy sharing the result's poles, knots and multiplicities (OCCT's by-value copy).</summary>
    public MultiBSpCurve Value() => new(_scu);

    /// <summary>True if the errors are within the tolerances.</summary>
    public bool IsDone => _done;

    /// <summary>The largest distance at the point <paramref name="index"/>.</summary>
    public double Error(int index) => _parError[index];

    /// <summary>The average of the points' largest distances.</summary>
    public double AverageError() => _avError;

    /// <summary>The largest 3d distance.</summary>
    public double MaxError3d() => _mError3d;

    /// <summary>The largest 2d distance.</summary>
    public double MaxError2d() => _mError2d;
}
