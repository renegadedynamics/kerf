// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_FuncSphereLSDist.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_FuncSphereLSDist.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// The least-squares criterion for fitting a sphere to points (x(i), y(i), z(i)):
/// F(x0, y0, z0, R) = Sum[(x(i) - x0)^2 + (y(i) - y0)^2 + (z(i) - z0)^2 - R^2]^2, with its gradient, for
/// math_BFGS. The variables are the centre (X(1) to X(3)) and the radius X(4).
/// </summary>
internal sealed class FuncSphereLSDist : MultipleVarFunctionWithGradient
{
    private NCollection_HArray1<Xyz>? _points;

    /// <summary>A function without points (set them with <see cref="SetPoints"/>).</summary>
    public FuncSphereLSDist()
    {
    }

    /// <summary>The function for thePoints (kept, not copied).</summary>
    public FuncSphereLSDist(NCollection_HArray1<Xyz> points)
    {
        _points = points;
    }

    /// <summary>Sets the points (kept, not copied).</summary>
    public void SetPoints(NCollection_HArray1<Xyz> points) => _points = points;

    /// <summary>4: the centre and the radius.</summary>
    public override int VariableCount => 4;

    /// <summary>The criterion at X.</summary>
    public override bool Value(math_Vector x, out double f)
    {
        var loc = new Xyz(x[1], x[2], x[3]);
        double r2 = x[4] * x[4];
        f = 0.0;
        int i;
        for (i = _points!.Lower(); i <= _points.Upper(); ++i)
        {
            double d = (_points.Value(i) - loc).SquareModulus() - r2;
            f += d * d;
        }

        return true;
    }

    /// <summary>The gradient at X.</summary>
    public override bool Gradient(math_Vector x, math_Vector g)
    {
        var loc = new Xyz(x[1], x[2], x[3]);
        double r = x[4], r2 = r * r;
        g.Init(0.0);
        int i;
        for (i = _points!.Lower(); i <= _points.Upper(); ++i)
        {
            Xyz dLoc = _points.Value(i) - loc;
            double d = dLoc.SquareModulus() - r2;
            g[1] += d * dLoc.X();
            g[2] += d * dLoc.Y();
            g[3] += d * dLoc.Z();
            g[4] += d;
        }

        g *= -4;
        g[4] *= r;
        return true;
    }

    /// <summary>The criterion and its gradient at X.</summary>
    public override bool Values(math_Vector x, out double f, math_Vector g)
    {
        var loc = new Xyz(x[1], x[2], x[3]);
        double r = x[4], r2 = r * r;
        g.Init(0.0);
        f = 0.0;
        int i;
        for (i = _points!.Lower(); i <= _points.Upper(); ++i)
        {
            Xyz dLoc = _points.Value(i) - loc;
            double d = dLoc.SquareModulus() - r2;
            g[1] += d * dLoc.X();
            g[2] += d * dLoc.Y();
            g[3] += d * dLoc.Z();
            g[4] += d;
            f += d * d;
        }

        g *= -4;
        g[4] *= r;
        return true;
    }
}
