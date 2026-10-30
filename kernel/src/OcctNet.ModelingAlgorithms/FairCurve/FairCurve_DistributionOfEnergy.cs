// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfEnergy.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfEnergy.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The base of the energy distributions of a fair curve: functions of the curve's parameter whose values are the
/// local energy, then (by derivative order) its gradient and hessian with respect to the poles.
/// </summary>
/// <remarks>
/// A working object (decision 21). It keeps the caller's knot and pole arrays as OCCT keeps the handles, and reads the
/// poles as the energy that owns it moves them.
/// </remarks>
public abstract class FairCurve_DistributionOfEnergy : math_FunctionSet
{
    protected int MyBSplOrder;
    protected NCollection_HArray1<double> MyFlatKnots;
    protected NCollection_HArray1<gp_Pnt2d> MyPoles;
    protected int MyDerivativeOrder;
    protected int MyNbVar;
    protected int MyNbEqua;
    protected int MyNbValAux;

    /// <summary>The number of variables of the function.</summary>
    public override int NbVariables()
    {
        return MyNbVar;
    }

    /// <summary>The number of equations of the function.</summary>
    public override int NbEquations()
    {
        return MyNbEqua;
    }

    protected FairCurve_DistributionOfEnergy(int BSplOrder,
                                             NCollection_HArray1<double> FlatKnots,
                                             NCollection_HArray1<gp_Pnt2d> Poles,
                                             int DerivativeOrder,
                                             int NbValAux = 0)
    {
        MyBSplOrder = BSplOrder;
        MyFlatKnots = FlatKnots;
        MyPoles = Poles;
        MyDerivativeOrder = DerivativeOrder;
        MyNbValAux = NbValAux;
        MyNbVar = 1;
        SetDerivativeOrder(DerivativeOrder);
    }

    /// <summary>Sets the order of the derivatives computed (0: the energy; 1: and its gradient; 2: and its hessian).</summary>
    public void SetDerivativeOrder(int DerivativeOrder)
    {
        MyNbEqua = 1;
        if (DerivativeOrder >= 1)
        {
            MyNbEqua += 2 * MyPoles.Length() + MyNbValAux;
        }
        if (DerivativeOrder >= 2)
        {
            MyNbEqua = MyNbEqua + (MyNbEqua - 1) * MyNbEqua / 2;
        }
        MyDerivativeOrder = DerivativeOrder;
    }
}
