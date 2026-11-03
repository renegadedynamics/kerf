// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_Gradient.gxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_TheGradient.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// lpa, le 11/09/91

// Application of the corrected gradient method to minimize
// F  = sum(||C(ui, Poles(ui)) - ptli||2.
// The conjugate gradient method is implemented in the mathematical
// library: math_BFGS.
// This algorithm must be called only when dealing with a set
// of constrained points (other than at extremities). Indeed, the call to the
// function F to minimize implies a call to ParLeastSquare and ResConstraint.
// If this is not the case, the call to ResConstraint is equivalent to a
// second least squares resolution, thus wasting a lot of time.

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Approximation;

/// <summary>
/// Approximates a constrained MultiLine by Bezier curves, optimising the points' parameters: one projection
/// iteration (Rogers and Fog), then a BFGS minimisation of
/// <see cref="Function{MultiLine, ToolLine}"/>.
/// </summary>
/// <remarks>
/// The generic form of OCCT's <c>AppParCurves_Gradient.gxx</c> (decision 142), instantiated by OCCT as
/// AppDef_TheGradient, AppDef_MyGradientOf... and the GeomInt and BRepApprox equivalents. Meant for MultiLines
/// with interior constraints. The computation runs in the constructor, which also writes the optimised
/// parameters into <c>Parameters</c>.
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="ToolLine">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
public class Gradient<MultiLine, ToolLine>
    where ToolLine : struct, ILineTool<MultiLine>
{
    private MultiCurve _scu;
    private readonly math_Vector _parError;
    private double _avError;
    private double _mError3d;
    private double _mError2d;
    private bool _done;

    /// <summary>
    /// Approximates the points <paramref name="firstPoint"/> to <paramref name="lastPoint"/> of
    /// <paramref name="ssp"/> by Bezier curves of degree <paramref name="Deg"/> under the constraints
    /// <paramref name="theConstraints"/>, from the parameters <paramref name="parameters"/>, which receive the
    /// optimised parameters.
    /// </summary>
    public Gradient(MultiLine ssp, int firstPoint, int lastPoint,
                                 NCollection_HArray1<ConstraintCouple> theConstraints, math_Vector parameters,
                                 int Deg, double tol3d, double tol2d, int nbIterations = 200)
    {
        _parError = new math_Vector(firstPoint, lastPoint, 0.0);
        _avError = 0.0;
        _mError3d = 0.0;
        _mError2d = 0.0;

        int j, k, i2, l;
        double uf, du, fval = 0.0, fu, dfu;
        int nbP3d = ToolLine.NbP3d(ssp);
        int nbP2d = ToolLine.NbP2d(ssp);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        int nbP = nbP3d + nbP2d;
        Point3D pt;
        Point2D pt2d;
        Vector3D v1, myV;
        Vector2D v12d, myV2d;
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

        Function<MultiLine, ToolLine> myF = new(ssp, firstPoint, lastPoint, theConstraints, parameters, Deg);

        if (!myF.Value(parameters, out fval))
        {
            _scu = new MultiCurve();
            _done = false;
            return;
        }

        _scu = new MultiCurve(myF.CurveValue());
        int deg = _scu.NbPoles() - 1;
        NCollection_Array1<Point3D> tabPole = new(1, deg + 1), tabCoef = new(1, deg + 1);
        NCollection_Array1<Point2D> tabPole2d = new(1, deg + 1), tabCoef2d = new(1, deg + 1);
        NCollection_Array1<Point3D> theCoef = new(1, (deg + 1) * mynbP3d);
        NCollection_Array1<Point2D> theCoef2d = new(1, (deg + 1) * mynbP2d);

        // Storage of curve poles for projection:
        // ============================================
        i2 = 0;
        for (k = 1; k <= nbP3d; k++)
        {
            _scu.Curve(k, tabPole);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.PolesCoefficients(tabPole, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), tabCoef, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
            for (j = 1; j <= deg + 1; j++)
            {
                theCoef[j + i2] = tabCoef[j];
            }
            i2 += deg + 1;
        }
        i2 = 0;
        for (k = 1; k <= nbP2d; k++)
        {
            _scu.Curve(nbP3d + k, tabPole2d);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.PolesCoefficients(tabPole2d, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), tabCoef2d, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
            for (j = 1; j <= deg + 1; j++)
            {
                theCoef2d[j + i2] = tabCoef2d[j];
            }
            i2 += deg + 1;
        }

        //  Une iteration rapide de projection est faite par la methode de
        //  Rogers & Fog 89, methode equivalente a Hoschek 88 qui ne necessite pas
        //  le calcul de D2.

        // Iteration de Projection:
        // =======================
        for (j = firstPoint + 1; j <= lastPoint - 1; j++)
        {
            uf = parameters[j];
            if (nbP != 0 && nbP2d != 0)
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
            for (k = 1; k <= nbP3d; k++)
            {
                for (l = 1; l <= deg + 1; l++)
                {
                    tabCoef[l] = theCoef[l + i2];
                }
                i2 += deg + 1;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.CoefsD1(uf, tabCoef, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), out pt, out v1);
                myV = new Vector3D(pt, tabP[k]);
                fu += myV * v1;
                dfu += v1.SquareMagnitude;
            }
            i2 = 0;
            for (k = 1; k <= nbP2d; k++)
            {
                for (l = 1; l <= deg + 1; l++)
                {
                    tabCoef2d[l] = theCoef2d[l + i2];
                }
                i2 += deg + 1;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.CoefsD1(uf, tabCoef2d, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), out pt2d, out v12d);
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

        if (!myF.Value(parameters, out fval))
        {
            _scu = new MultiCurve();
            _done = false;
            return;
        }
        _mError3d = myF.MaxError3d();
        _mError2d = myF.MaxError2d();

        if (_mError3d <= tol3d && _mError2d <= tol2d)
        {
            _done = true;
            _scu = new MultiCurve(myF.CurveValue());
        }
        else if (nbIterations != 0)
        {
            // NbIterations de gradient conjugue:
            // =================================
            double eps = 1.0e-07;
            GradientBfgs<MultiLine, ToolLine> fResol = new(myF, parameters, tol3d, tol2d, eps, nbIterations);
            parameters.Initialized(myF.NewParameters());
            _scu = new MultiCurve(myF.CurveValue());
        }

        _avError = 0.0;
        for (j = firstPoint; j <= lastPoint; j++)
        {
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

    /// <summary>The approximating curves: a MultiCurve sharing the result's poles (OCCT's by-value copy).</summary>
    public MultiCurve Value() => new(_scu);

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
