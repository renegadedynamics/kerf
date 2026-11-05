// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_InterFunc.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_InterFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// The function U(t) - FixVal (Fix = 1) or V(t) - FixVal (Fix = 2) of a 2D curve, whose roots locate the
/// parameters where an Adaptor3d_CurveOnSurface crosses a discontinuity of its surface.
/// </summary>
internal class InterFunc : FunctionWithDerivative
{
    private readonly Curve2DAdaptorBase _curve2d;
    private readonly double _fixVal;
    private readonly int _fix;

    /// <exception cref="ConstructionErrorException">Fix is neither 1 nor 2.</exception>
    public InterFunc(Curve2DAdaptorBase c, double fixVal, int fix)
    {
        _curve2d = c;
        _fixVal = fixVal;
        _fix = fix;
        if (fix != 1 && fix != 2)
        {
            throw new ConstructionErrorException();
        }
    }

    public override bool Value(double x, out double f)
    {
        Point2D c;
        _curve2d.D0(x, out c);
        if (_fix == 1)
        {
            f = c.X - _fixVal;
        }
        else
        {
            f = c.Y - _fixVal;
        }
        return true;
    }

    public override bool Derivative(double x, out double d)
    {
        double f;
        return Values(x, out f, out d);
    }

    public override bool Values(double x, out double f, out double d)
    {
        Point2D c;
        Vector2D dc;
        _curve2d.D1(x, out c, out dc);
        if (_fix == 1)
        {
            f = c.X - _fixVal;
            d = dc.X;
        }
        else
        {
            f = c.Y - _fixVal;
            d = dc.Y;
        }
        return true;
    }
}
