// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_MyRootFunction.hxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_MyRootFunction.lxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_MyRootFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Measurement.Points;

/// <summary>
/// The function solved by Newton's method to find X with Integral(X0, X, F) = L (the length of a curve and its
/// derivative).
/// </summary>
public sealed class MyRootFunction : FunctionWithDerivative
{
    private readonly MyGaussFunction _function = new();
    private double _x0;
    private double _l;
    private int _order;
    private double _tol;

    public MyRootFunction()
    {
        _x0 = 0.0;
        _l = 0.0;
        _order = 0;
        _tol = 0.0;
    }

    /// <summary>F is the function, D the client data, Order the order of the integration.</summary>
    public void Init(RealFunction f, object? d, int order)
    {
        _function.Init(f, d);
        _order = order;
    }

    /// <summary>Solves Integral(X0, X, F(X, D)) = L.</summary>
    public void Init(double x0, double l)
    {
        _x0 = x0;
        _l = l;
        _tol = -1; // to suppress the tolerance
    }

    /// <summary>Solves Integral(X0, X, F(X, D)) = L with the given tolerance.</summary>
    public void Init(double x0, double l, double tol)
    {
        _x0 = x0;
        _l = l;
        _tol = tol;
    }

    /// <summary>Integral(X0, X, F(X, D)) - L.</summary>
    /// <remarks>On failure F is 0, where OCCT leaves it unchanged (decision 32).</remarks>
    public override bool Value(double x, out double f)
    {
        GaussSingleIntegration length;
        if (_tol <= 0)
        {
            length = new GaussSingleIntegration(_function, _x0, x, _order);
        }
        else
        {
            length = new GaussSingleIntegration(_function, _x0, x, _order, _tol);
        }
        if (length.IsDone)
        {
            f = length.Value() - _l;
            return true;
        }
        else
        {
            f = 0.0;
            return false;
        }
    }

    /// <summary>F(X, D).</summary>
    public override bool Derivative(double x, out double df)
    {
        return _function.Value(x, out df);
    }

    /// <remarks>On failure F and Df are 0, where OCCT leaves them unchanged (decision 32).</remarks>
    public override bool Values(double x, out double f, out double df)
    {
        GaussSingleIntegration length;
        if (_tol <= 0)
        {
            length = new GaussSingleIntegration(_function, _x0, x, _order);
        }
        else
        {
            length = new GaussSingleIntegration(_function, _x0, x, _order, _tol);
        }
        if (length.IsDone)
        {
            f = length.Value() - _l;
            return _function.Value(x, out df);
        }
        else
        {
            f = 0.0;
            df = 0.0;
            return false;
        }
    }
}
