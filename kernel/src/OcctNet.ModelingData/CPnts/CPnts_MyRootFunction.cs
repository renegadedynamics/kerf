// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_MyRootFunction.hxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_MyRootFunction.lxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_MyRootFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>
/// The function solved by Newton's method to find X with Integral(X0, X, F) = L (the length of a curve and its
/// derivative).
/// </summary>
public sealed class CPnts_MyRootFunction : FunctionWithDerivative
{
    private readonly CPnts_MyGaussFunction myFunction = new();
    private double myX0;
    private double myL;
    private int myOrder;
    private double myTol;

    public CPnts_MyRootFunction()
    {
        myX0 = 0.0;
        myL = 0.0;
        myOrder = 0;
        myTol = 0.0;
    }

    /// <summary>F is the function, D the client data, Order the order of the integration.</summary>
    public void Init(CPnts_RealFunction F, object? D, int Order)
    {
        myFunction.Init(F, D);
        myOrder = Order;
    }

    /// <summary>Solves Integral(X0, X, F(X, D)) = L.</summary>
    public void Init(double X0, double L)
    {
        myX0 = X0;
        myL = L;
        myTol = -1; // to suppress the tolerance
    }

    /// <summary>Solves Integral(X0, X, F(X, D)) = L with the given tolerance.</summary>
    public void Init(double X0, double L, double Tol)
    {
        myX0 = X0;
        myL = L;
        myTol = Tol;
    }

    /// <summary>Integral(X0, X, F(X, D)) - L.</summary>
    /// <remarks>On failure F is 0, where OCCT leaves it unchanged (decision 32).</remarks>
    public override bool Value(double X, out double F)
    {
        GaussSingleIntegration Length;
        if (myTol <= 0)
        {
            Length = new GaussSingleIntegration(myFunction, myX0, X, myOrder);
        }
        else
        {
            Length = new GaussSingleIntegration(myFunction, myX0, X, myOrder, myTol);
        }
        if (Length.IsDone)
        {
            F = Length.Value() - myL;
            return true;
        }
        else
        {
            F = 0.0;
            return false;
        }
    }

    /// <summary>F(X, D).</summary>
    public override bool Derivative(double X, out double Df)
    {
        return myFunction.Value(X, out Df);
    }

    /// <remarks>On failure F and Df are 0, where OCCT leaves them unchanged (decision 32).</remarks>
    public override bool Values(double X, out double F, out double Df)
    {
        GaussSingleIntegration Length;
        if (myTol <= 0)
        {
            Length = new GaussSingleIntegration(myFunction, myX0, X, myOrder);
        }
        else
        {
            Length = new GaussSingleIntegration(myFunction, myX0, X, myOrder, myTol);
        }
        if (Length.IsDone)
        {
            F = Length.Value() - myL;
            return myFunction.Value(X, out Df);
        }
        else
        {
            F = 0.0;
            Df = 0.0;
            return false;
        }
    }
}
