// Port of: src/ModelingData/TKGeomBase/AppParCurves/AppParCurves_BSpFunction.gxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_BSpParFunctionOfMyBSplGradientOfBSplineCompute.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.AppParCurves_Constraint;

namespace Occt;

/// <summary>
/// The function F = sum ||C(ui) - Ptli||^2 of the parameters ui of a MultiLine's points, and its gradient, for
/// B-spline curves with given knots: each evaluation computes new poles by least squares
/// (<see cref="AppParCurves_LeastSquare{MultiLine, ToolLine}"/> with knots). Minimised by
/// <see cref="AppParCurves_BSpGradient{MultiLine, ToolLine}"/>.
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
public class AppParCurves_BSpFunction<MultiLine, ToolLine> : math_MultipleVarFunctionWithGradient
    where ToolLine : struct, IAppParCurves_LineTool<MultiLine>
{
    // OCCT's Done field is written but never read; it is left out
    private readonly MultiLine MyMultiLine;
    private AppParCurves_MultiBSpCurve MyMultiBSpCurve;
    private readonly int nbpoles;
    private readonly math_Vector myParameters;
    private double FVal;
    private readonly math_Vector ValGrad_F;
    private readonly math_Matrix MyF;
    private readonly math_Matrix PTLX;
    private readonly math_Matrix PTLY;
    private readonly math_Matrix PTLZ;
    private readonly math_Matrix A;
    private readonly math_Matrix DA;
    private readonly AppParCurves_LeastSquare<MultiLine, ToolLine> MyLeastSquare;
    private readonly bool Contraintes;
    private readonly int NbP;
    private readonly int NbCu;
    private readonly int Adeb;
    private readonly int Afin;
    private readonly NCollection_HArray1<int> tabdim;
    private double ERR3d;
    private double ERR2d;
    private readonly int FirstP;
    private readonly int LastP;
    private readonly NCollection_HArray1<AppParCurves_ConstraintCouple> myConstraints;
    private double mylambda1;
    private double mylambda2;

    /// <summary>
    /// The function for the points <paramref name="FirstPoint"/> to <paramref name="LastPoint"/> of
    /// <paramref name="SSP"/>, approximated by B-spline curves of <paramref name="NbPol"/> poles with the knots
    /// <paramref name="Knots"/> and multiplicities <paramref name="Mults"/> under the constraints
    /// <paramref name="TheConstraints"/>, starting from the parameters <paramref name="Parameters"/> (copied).
    /// </summary>
    public AppParCurves_BSpFunction(MultiLine SSP, int FirstPoint, int LastPoint,
                                    NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints,
                                    math_Vector Parameters, NCollection_Array1<double> Knots, NCollection_Array1<int> Mults,
                                    int NbPol)
    {
        MyMultiLine = SSP;
        MyMultiBSpCurve = new AppParCurves_MultiBSpCurve(NbPol);
        myParameters = new math_Vector(Parameters.Lower(), Parameters.Upper());
        ValGrad_F = new math_Vector(FirstPoint, LastPoint);
        MyF = new math_Matrix(FirstPoint, LastPoint, 1, ToolLine.NbP3d(SSP) + ToolLine.NbP2d(SSP), 0.0);
        PTLX = new math_Matrix(FirstPoint, LastPoint, 1, ToolLine.NbP3d(SSP) + ToolLine.NbP2d(SSP), 0.0);
        PTLY = new math_Matrix(FirstPoint, LastPoint, 1, ToolLine.NbP3d(SSP) + ToolLine.NbP2d(SSP), 0.0);
        PTLZ = new math_Matrix(FirstPoint, LastPoint, 1, ToolLine.NbP3d(SSP) + ToolLine.NbP2d(SSP), 0.0);
        A = new math_Matrix(FirstPoint, LastPoint, 1, NbPol);
        DA = new math_Matrix(FirstPoint, LastPoint, 1, NbPol);
        MyLeastSquare = new AppParCurves_LeastSquare<MultiLine, ToolLine>(SSP,
                                                                          Knots,
                                                                          Mults,
                                                                          FirstPoint,
                                                                          LastPoint,
                                                                          FirstConstraint(TheConstraints, FirstPoint),
                                                                          LastConstraint(TheConstraints, LastPoint),
                                                                          NbPol);

        int i;
        for (i = Parameters.Lower(); i <= Parameters.Upper(); i++)
        {
            myParameters[i] = Parameters[i];
        }
        FirstP = FirstPoint;
        LastP = LastPoint;
        myConstraints = TheConstraints;
        NbP = LastP - FirstP + 1;
        Adeb = FirstP;
        Afin = LastP;
        nbpoles = NbPol;
        MyMultiBSpCurve.SetKnots(Knots);
        MyMultiBSpCurve.SetMultiplicities(Mults);
        Contraintes = false;
        int myindex;
        int low = TheConstraints.Lower(), upp = TheConstraints.Upper();
        AppParCurves_ConstraintCouple mycouple;
        AppParCurves_Constraint Cons;

        for (i = low; i <= upp; i++)
        {
            mycouple = TheConstraints.Value(i);
            Cons = mycouple.Constraint();
            myindex = mycouple.Index();
            if (myindex == FirstP)
            {
                if ((int)Cons >= 1)
                {
                    Adeb = Adeb + 1;
                }
            }
            else if (myindex == LastP)
            {
                if ((int)Cons >= 1)
                {
                    Afin = Afin - 1;
                }
            }
            else
            {
                if ((int)Cons >= 1)
                {
                    Contraintes = true;
                }
            }
        }

        int nb3d = ToolLine.NbP3d(SSP);
        int nb2d = ToolLine.NbP2d(SSP);
        int mynb3d = nb3d, mynb2d = nb2d;
        if (nb3d == 0)
        {
            mynb3d = 1;
        }
        if (nb2d == 0)
        {
            mynb2d = 1;
        }

        NbCu = nb3d + nb2d;
        tabdim = new NCollection_HArray1<int>(0, NbCu - 1);

        if (Contraintes)
        {
            for (i = 1; i <= NbCu; i++)
            {
                if (i <= nb3d)
                {
                    tabdim.SetValue(i - 1, 3);
                }
                else
                {
                    tabdim.SetValue(i - 1, 2);
                }
            }

            NCollection_Array1<gp_Pnt> TabP = new(1, mynb3d);
            NCollection_Array1<gp_Pnt2d> TabP2d = new(1, mynb2d);

            for (i = FirstP; i <= LastP; i++)
            {
                if (nb3d != 0 && nb2d != 0)
                {
                    ToolLine.Value(SSP, i, TabP, TabP2d);
                }
                else if (nb3d != 0)
                {
                    ToolLine.Value(SSP, i, TabP);
                }
                else
                {
                    ToolLine.Value(SSP, i, TabP2d);
                }
                for (int j = 1; j <= NbCu; j++)
                {
                    if (tabdim.Value(j - 1) == 3)
                    {
                        TabP[j].Coord(out PTLX[i, j], out PTLY[i, j], out PTLZ[i, j]);
                    }
                    else
                    {
                        // OCCT: TabP2d(j) where j - nb3d is meant; out of range for a line with 3d and 2d points
                        // (unchecked in OCCT; the port throws)
                        TabP2d[j].Coord(out PTLX[i, j], out PTLY[i, j]);
                    }
                }
            }
        }
    }

    /// <summary>The constraint of the point <paramref name="FirstPoint"/> (the last couple's when it has none).</summary>
    public AppParCurves_Constraint FirstConstraint(NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints,
                                                   int FirstPoint)
    {
        int i, myindex;
        int low = TheConstraints.Lower(), upp = TheConstraints.Upper();
        AppParCurves_ConstraintCouple mycouple;
        AppParCurves_Constraint Cons = AppParCurves_NoConstraint;

        for (i = low; i <= upp; i++)
        {
            mycouple = TheConstraints.Value(i);
            Cons = mycouple.Constraint();
            myindex = mycouple.Index();
            if (myindex == FirstPoint)
            {
                break;
            }
        }
        return Cons;
    }

    /// <summary>The constraint of the point <paramref name="LastPoint"/> (the last couple's when it has none).</summary>
    public AppParCurves_Constraint LastConstraint(NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints,
                                                  int LastPoint)
    {
        int i, myindex;
        int low = TheConstraints.Lower(), upp = TheConstraints.Upper();
        AppParCurves_ConstraintCouple mycouple;
        AppParCurves_Constraint Cons = AppParCurves_NoConstraint;

        for (i = low; i <= upp; i++)
        {
            mycouple = TheConstraints.Value(i);
            Cons = mycouple.Constraint();
            myindex = mycouple.Index();
            if (myindex == LastPoint)
            {
                break;
            }
        }
        return Cons;
    }

    /// <summary>
    /// The value of F at the parameters <paramref name="X"/>; false (and F 0) if the resolution fails. With interior
    /// constraints F is not computed (as in OCCT) and keeps its previous value.
    /// </summary>
    public override bool Value(math_Vector X, out double F)
    {
        myParameters.Initialized(X);

        // Resolution moindres carres:
        // ===========================
        MyLeastSquare.Perform(myParameters, mylambda1, mylambda2);
        if (!(MyLeastSquare.IsDone()))
        {
            F = 0.0;
            return false;
        }
        if (!Contraintes)
        {
            MyLeastSquare.Error(out FVal, out ERR3d, out ERR2d);
            F = FVal;
        }

        // Resolution avec contraintes:
        // ============================
        else
        {
            // Not implemented in OCCT, where F is left unchanged
            F = FVal;
        }
        return true;
    }

    /// <summary>Computes F and its gradient at <paramref name="X"/>.</summary>
    protected void Perform(math_Vector X)
    {
        int j;

        myParameters.Initialized(X);
        // Resolution moindres carres:
        // ===========================
        MyLeastSquare.Perform(myParameters, mylambda1, mylambda2);

        if (!(MyLeastSquare.IsDone()))
        {
            return;
        }

        for (j = myParameters.Lower(); j <= myParameters.Upper(); j++)
        {
            ValGrad_F[j] = 0.0;
        }

        if (!Contraintes)
        {
            MyLeastSquare.ErrorGradient(ValGrad_F, out FVal, out ERR3d, out ERR2d);
        }
    }

    /// <summary>Sets the tangency value at the first point.</summary>
    public void SetFirstLambda(double l1) => mylambda1 = l1;

    /// <summary>Sets the tangency value at the last point.</summary>
    public void SetLastLambda(double l2) => mylambda2 = l2;

    /// <summary>The number of variables: the number of points.</summary>
    public override int NbVariables() => NbP;

    /// <summary>The gradient of F at <paramref name="X"/>.</summary>
    public override bool Gradient(math_Vector X, math_Vector G)
    {
        Perform(X);
        G.Initialized(ValGrad_F);

        return true;
    }

    /// <summary>The value and gradient of F at <paramref name="X"/>.</summary>
    public override bool Values(math_Vector X, out double F, math_Vector G)
    {
        Perform(X);
        F = FVal;
        G.Initialized(ValGrad_F);
        return true;
    }

    /// <summary>The current curves: a copy sharing the poles, knots and multiplicities (OCCT's by-value copy).</summary>
    public AppParCurves_MultiBSpCurve CurveValue()
    {
        if (!Contraintes)
        {
            MyMultiBSpCurve = new AppParCurves_MultiBSpCurve(MyLeastSquare.BSplineValue());
        }
        return new AppParCurves_MultiBSpCurve(MyMultiBSpCurve);
    }

    /// <summary>The distance between the point <paramref name="IPoint"/> and the curve <paramref name="CurveIndex"/>.</summary>
    public double Error(int IPoint, int CurveIndex)
    {
        math_Matrix DD = MyLeastSquare.Distance();
        double d = DD.Value(IPoint, CurveIndex);
        if (!Contraintes)
        {
            return d;
        }
        else
        {
            return Math.Sqrt(MyF[IPoint, CurveIndex]);
        }
    }

    /// <summary>The largest 3d distance.</summary>
    public double MaxError3d() => ERR3d;

    /// <summary>The largest 2d distance.</summary>
    public double MaxError2d() => ERR2d;

    /// <summary>The parameters of the last evaluation (shared: don't modify).</summary>
    public math_Vector NewParameters() => myParameters;

    /// <summary>The function matrix of the least squares (shared: don't modify).</summary>
    public math_Matrix FunctionMatrix() => MyLeastSquare.FunctionMatrix();

    /// <summary>The derivative function matrix of the least squares (shared: don't modify).</summary>
    public math_Matrix DerivativeFunctionMatrix() => MyLeastSquare.DerivativeFunctionMatrix();

    /// <summary>The index of the first non-zero basis function at each point (see KIndex; shared: don't modify).</summary>
    public math_IntegerVector Index() => MyLeastSquare.KIndex();
}
