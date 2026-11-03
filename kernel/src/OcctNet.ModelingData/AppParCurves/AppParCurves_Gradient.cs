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

namespace Occt;

/// <summary>
/// Approximates a constrained MultiLine by Bezier curves, optimising the points' parameters: one projection
/// iteration (Rogers and Fog), then a BFGS minimisation of
/// <see cref="AppParCurves_Function{MultiLine, ToolLine}"/>.
/// </summary>
/// <remarks>
/// The generic form of OCCT's <c>AppParCurves_Gradient.gxx</c> (decision 142), instantiated by OCCT as
/// AppDef_TheGradient, AppDef_MyGradientOf... and the GeomInt and BRepApprox equivalents. Meant for MultiLines
/// with interior constraints. The computation runs in the constructor, which also writes the optimised
/// parameters into <c>Parameters</c>.
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine type.</typeparam>
/// <typeparam name="ToolLine">The line tool reading <typeparamref name="MultiLine"/>.</typeparam>
public class AppParCurves_Gradient<MultiLine, ToolLine>
    where ToolLine : struct, IAppParCurves_LineTool<MultiLine>
{
    private AppParCurves_MultiCurve SCU;
    private readonly math_Vector ParError;
    private double AvError;
    private double MError3d;
    private double MError2d;
    private bool Done;

    /// <summary>
    /// Approximates the points <paramref name="FirstPoint"/> to <paramref name="LastPoint"/> of
    /// <paramref name="SSP"/> by Bezier curves of degree <paramref name="Deg"/> under the constraints
    /// <paramref name="TheConstraints"/>, from the parameters <paramref name="Parameters"/>, which receive the
    /// optimised parameters.
    /// </summary>
    public AppParCurves_Gradient(MultiLine SSP, int FirstPoint, int LastPoint,
                                 NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints, math_Vector Parameters,
                                 int Deg, double Tol3d, double Tol2d, int NbIterations = 200)
    {
        ParError = new math_Vector(FirstPoint, LastPoint, 0.0);
        AvError = 0.0;
        MError3d = 0.0;
        MError2d = 0.0;

        int j, k, i2, l;
        double UF, DU, Fval = 0.0, FU, DFU;
        int nbP3d = ToolLine.NbP3d(SSP);
        int nbP2d = ToolLine.NbP2d(SSP);
        int mynbP3d = nbP3d, mynbP2d = nbP2d;
        int nbP = nbP3d + nbP2d;
        Point3D Pt;
        Point2D Pt2d;
        Vector3D V1, MyV;
        Vector2D V12d, MyV2d;
        Done = false;

        if (nbP3d == 0)
        {
            mynbP3d = 1;
        }
        if (nbP2d == 0)
        {
            mynbP2d = 1;
        }
        NCollection_Array1<Point3D> TabP = new(1, mynbP3d);
        NCollection_Array1<Point2D> TabP2d = new(1, mynbP2d);

        // Calculation of the function F= sum(||C(ui)-Ptli||2):
        // Call to a function inheriting from MultipleVarFunctionWithGradient
        // to compute F and grad_F.
        // ================================================================

        AppParCurves_Function<MultiLine, ToolLine> MyF = new(SSP, FirstPoint, LastPoint, TheConstraints, Parameters, Deg);

        if (!MyF.Value(Parameters, out Fval))
        {
            SCU = new AppParCurves_MultiCurve();
            Done = false;
            return;
        }

        SCU = new AppParCurves_MultiCurve(MyF.CurveValue());
        int deg = SCU.NbPoles() - 1;
        NCollection_Array1<Point3D> TabPole = new(1, deg + 1), TabCoef = new(1, deg + 1);
        NCollection_Array1<Point2D> TabPole2d = new(1, deg + 1), TabCoef2d = new(1, deg + 1);
        NCollection_Array1<Point3D> TheCoef = new(1, (deg + 1) * mynbP3d);
        NCollection_Array1<Point2D> TheCoef2d = new(1, (deg + 1) * mynbP2d);

        // Storage of curve poles for projection:
        // ============================================
        i2 = 0;
        for (k = 1; k <= nbP3d; k++)
        {
            SCU.Curve(k, TabPole);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.PolesCoefficients(TabPole, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), TabCoef, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
            for (j = 1; j <= deg + 1; j++)
            {
                TheCoef[j + i2] = TabCoef[j];
            }
            i2 += deg + 1;
        }
        i2 = 0;
        for (k = 1; k <= nbP2d; k++)
        {
            SCU.Curve(nbP3d + k, TabPole2d);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.PolesCoefficients(TabPole2d, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), TabCoef2d, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
            for (j = 1; j <= deg + 1; j++)
            {
                TheCoef2d[j + i2] = TabCoef2d[j];
            }
            i2 += deg + 1;
        }

        //  Une iteration rapide de projection est faite par la methode de
        //  Rogers & Fog 89, methode equivalente a Hoschek 88 qui ne necessite pas
        //  le calcul de D2.

        // Iteration de Projection:
        // =======================
        for (j = FirstPoint + 1; j <= LastPoint - 1; j++)
        {
            UF = Parameters[j];
            if (nbP != 0 && nbP2d != 0)
            {
                ToolLine.Value(SSP, j, TabP, TabP2d);
            }
            else if (nbP2d != 0)
            {
                ToolLine.Value(SSP, j, TabP2d);
            }
            else
            {
                ToolLine.Value(SSP, j, TabP);
            }

            FU = 0.0;
            DFU = 0.0;
            i2 = 0;
            for (k = 1; k <= nbP3d; k++)
            {
                for (l = 1; l <= deg + 1; l++)
                {
                    TabCoef[l] = TheCoef[l + i2];
                }
                i2 += deg + 1;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.CoefsD1(UF, TabCoef, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), out Pt, out V1);
                MyV = new Vector3D(Pt, TabP[k]);
                FU += MyV * V1;
                DFU += V1.SquareMagnitude();
            }
            i2 = 0;
            for (k = 1; k <= nbP2d; k++)
            {
                for (l = 1; l <= deg + 1; l++)
                {
                    TabCoef2d[l] = TheCoef2d[l + i2];
                }
                i2 += deg + 1;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.CoefsD1(UF, TabCoef2d, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(), out Pt2d, out V12d);
                MyV2d = new Vector2D(Pt2d, TabP2d[k]);
                FU += MyV2d * V12d;
                DFU += V12d.SquareMagnitude();
            }

            if (DFU >= RealEpsilon())
            {
                DU = FU / DFU;
                DU = Math.CopySign(Min(5.0e-02, Math.Abs(DU)), DU);
                UF += DU;
                Parameters[j] = UF;
            }
        }

        if (!MyF.Value(Parameters, out Fval))
        {
            SCU = new AppParCurves_MultiCurve();
            Done = false;
            return;
        }
        MError3d = MyF.MaxError3d();
        MError2d = MyF.MaxError2d();

        if (MError3d <= Tol3d && MError2d <= Tol2d)
        {
            Done = true;
            SCU = new AppParCurves_MultiCurve(MyF.CurveValue());
        }
        else if (NbIterations != 0)
        {
            // NbIterations de gradient conjugue:
            // =================================
            double Eps = 1.0e-07;
            AppParCurves_Gradient_BFGS<MultiLine, ToolLine> FResol = new(MyF, Parameters, Tol3d, Tol2d, Eps, NbIterations);
            Parameters.Initialized(MyF.NewParameters());
            SCU = new AppParCurves_MultiCurve(MyF.CurveValue());
        }

        AvError = 0.0;
        for (j = FirstPoint; j <= LastPoint; j++)
        {
            // Recherche des erreurs maxi et moyenne a un index donne:
            for (k = 1; k <= nbP; k++)
            {
                ParError[j] = Max(ParError[j], MyF.Error(j, k));
            }
            AvError += ParError[j];
        }
        AvError = AvError / (LastPoint - FirstPoint + 1);

        MError3d = MyF.MaxError3d();
        MError2d = MyF.MaxError2d();
        if (MError3d <= Tol3d && MError2d <= Tol2d)
        {
            Done = true;
        }
    }

    /// <summary>The approximating curves: a MultiCurve sharing the result's poles (OCCT's by-value copy).</summary>
    public AppParCurves_MultiCurve Value() => new(SCU);

    /// <summary>True if the errors are within the tolerances.</summary>
    public bool IsDone() => Done;

    /// <summary>The largest distance at the point <paramref name="Index"/>.</summary>
    public double Error(int Index) => ParError[Index];

    /// <summary>The average of the points' largest distances.</summary>
    public double AverageError() => AvError;

    /// <summary>The largest 3d distance.</summary>
    public double MaxError3d() => MError3d;

    /// <summary>The largest 2d distance.</summary>
    public double MaxError2d() => MError2d;
}
