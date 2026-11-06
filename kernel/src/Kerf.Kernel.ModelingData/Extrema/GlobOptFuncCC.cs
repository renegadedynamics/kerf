// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncCC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncCC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Extrema.GlobOptFuncCC;

namespace Kerf.Engine.Extrema;

/// <summary>The file-static functions of Extrema_GlobOptFuncCC.cxx, for 3D and 2D curves.</summary>
internal static class GlobOptFuncCC
{
    internal static int NbVariables() => 2;

    // 3d _Value
    internal static bool _Value(CurveAdaptorBase c1, CurveAdaptorBase c2, math_Vector x, out double f)
    {
        double u = x[1];
        double v = x[2];

        if (u < c1.FirstParameter() || u > c1.LastParameter() || v < c2.FirstParameter() || v > c2.LastParameter())
        {
            f = 0.0; // OCCT leaves F unchanged (decision 32)
            return false;
        }

        f = c2.Value(v).DistanceTo(c1.Value(u));
        return true;
    }

    // 2d _Value
    internal static bool _Value(Curve2DAdaptorBase c1, Curve2DAdaptorBase c2, math_Vector x, out double f)
    {
        double u = x[1];
        double v = x[2];

        if (u < c1.FirstParameter() || u > c1.LastParameter() || v < c2.FirstParameter() || v > c2.LastParameter())
        {
            f = 0.0; // OCCT leaves F unchanged (decision 32)
            return false;
        }

        f = c2.Value(v).DistanceTo(c1.Value(u));
        return true;
    }

    // F = (x2(v) - x1(u))^2 + (y2(v) - y1(u))^2 + (z2(v) - z1(u))^2

    // 3d _Gradient
    internal static bool _Gradient(CurveAdaptorBase c1, CurveAdaptorBase c2, math_Vector x, math_Vector g)
    {
        if (x[1] < c1.FirstParameter() || x[1] > c1.LastParameter() || x[2] < c2.FirstParameter()
            || x[2] > c2.LastParameter())
        {
            return false;
        }

        c1.D1(x[1], out Point3D c1D0, out Vector3D c1D1);
        c2.D1(x[2], out Point3D c2D0, out Vector3D c2D1);

        g[1] = -(c2D0.X - c1D0.X) * c1D1.X - (c2D0.Y - c1D0.Y) * c1D1.Y
               - (c2D0.Z - c1D0.Z) * c1D1.Z;
        g[2] = (c2D0.X - c1D0.X) * c2D1.X + (c2D0.Y - c1D0.Y) * c2D1.Y
               + (c2D0.Z - c1D0.Z) * c2D1.Z;
        g *= 2.0;
        return true;
    }

    // 2d _Gradient
    internal static bool _Gradient(Curve2DAdaptorBase c1, Curve2DAdaptorBase c2, math_Vector x, math_Vector g)
    {
        if (x[1] < c1.FirstParameter() || x[1] > c1.LastParameter() || x[2] < c2.FirstParameter()
            || x[2] > c2.LastParameter())
        {
            return false;
        }

        c1.D1(x[1], out Point2D c1D0, out Vector2D c1D1);
        c2.D1(x[2], out Point2D c2D0, out Vector2D c2D1);

        g[1] = -(c2D0.X - c1D0.X) * c1D1.X - (c2D0.Y - c1D0.Y) * c1D1.Y;

        g[2] = (c2D0.X - c1D0.X) * c2D1.X + (c2D0.Y - c1D0.Y) * c2D1.Y;
        g *= 2.0;

        return true;
    }

    // 3d Hessian
    internal static bool Hessian(CurveAdaptorBase c1, CurveAdaptorBase c2, math_Vector x, Matrix h)
    {
        if (x[1] < c1.FirstParameter() || x[1] > c1.LastParameter() || x[2] < c2.FirstParameter()
            || x[2] > c2.LastParameter())
        {
            return false;
        }

        c1.D2(x[1], out Point3D c1D0, out Vector3D c1D1, out Vector3D c1D2);
        c2.D2(x[2], out Point3D c2D0, out Vector3D c2D1, out Vector3D c2D2);

        h[1, 1] = c1D1.X * c1D1.X + c1D1.Y * c1D1.Y + c1D1.Z * c1D1.Z
                  - (c2D0.X - c1D0.X) * c1D2.X - (c2D0.Y - c1D0.Y) * c1D2.Y
                  - (c2D0.Z - c1D0.Z) * c1D2.Z;

        h[1, 2] = -c2D1.X * c1D1.X - c2D1.Y * c1D1.Y - c2D1.Z * c1D1.Z;

        h[2, 1] = h[1, 2];

        h[2, 2] = c2D1.X * c2D1.X + c2D1.Y * c2D1.Y + c2D1.Z * c2D1.Z
                  + (c2D0.X - c1D0.X) * c2D2.X + (c2D0.Y - c1D0.Y) * c2D2.Y
                  + (c2D0.Z - c1D0.Z) * c2D2.Z;
        h *= 2.0;
        return true;
    }

    // 2d Hessian
    internal static bool Hessian(Curve2DAdaptorBase c1, Curve2DAdaptorBase c2, math_Vector x, Matrix h)
    {
        if (x[1] < c1.FirstParameter() || x[1] > c1.LastParameter() || x[2] < c2.FirstParameter()
            || x[2] > c2.LastParameter())
        {
            return false;
        }

        c1.D2(x[1], out Point2D c1D0, out Vector2D c1D1, out Vector2D c1D2);
        c2.D2(x[2], out Point2D c2D0, out Vector2D c2D1, out Vector2D c2D2);

        h[1, 1] = c1D1.X * c1D1.X + c1D1.Y * c1D1.Y - (c2D0.X - c1D0.X) * c1D2.X
                  - (c2D0.Y - c1D0.Y) * c1D2.Y;

        h[1, 2] = -c2D1.X * c1D1.X - c2D1.Y * c1D1.Y;

        h[2, 1] = h[1, 2];

        h[2, 2] = c2D1.X * c2D1.X + c2D1.Y * c2D1.Y + (c2D0.X - c1D0.X) * c2D2.X
                  + (c2D0.Y - c1D0.Y) * c2D2.Y;
        h *= 2.0;
        return true;
    }
}

/// <summary>
/// The distance between two curves (3D or 2D) as a function of their parameters (u, v), for math_GlobOptMin; false
/// outside the curves' ranges.
/// </summary>
/// <remarks>Keeps references to the curves, as OCCT does.</remarks>
internal class GlobOptFuncCcc0 : MultipleVarFunction
{
    private readonly CurveAdaptorBase? _c13d, _c23d;
    private readonly Curve2DAdaptorBase? _c12d, _c22d;
    private readonly int _type;

    public GlobOptFuncCcc0(CurveAdaptorBase c1, CurveAdaptorBase c2)
    {
        _c13d = c1;
        _c23d = c2;
        _c12d = null;
        _c22d = null;
        _type = 1;
    }

    public GlobOptFuncCcc0(Curve2DAdaptorBase c1, Curve2DAdaptorBase c2)
    {
        _c13d = null;
        _c23d = null;
        _c12d = c1;
        _c22d = c2;
        _type = 2;
    }

    public override int VariableCount => NbVariables();

    public override bool Value(math_Vector x, out double f)
    {
        if (_type == 1)
        {
            return _Value(_c13d!, _c23d!, x, out f);
        }
        else
        {
            return _Value(_c12d!, _c22d!, x, out f);
        }
    }
}

/// <summary>The distance between two curves with its gradient (see <see cref="GlobOptFuncCcc0"/>); the gradient
/// is the one of the square distance.</summary>
internal class GlobOptFuncCcc1 : MultipleVarFunctionWithGradient
{
    private readonly CurveAdaptorBase? _c13d, _c23d;
    private readonly Curve2DAdaptorBase? _c12d, _c22d;
    private readonly int _type;

    public GlobOptFuncCcc1(CurveAdaptorBase c1, CurveAdaptorBase c2)
    {
        _c13d = c1;
        _c23d = c2;
        _c12d = null;
        _c22d = null;
        _type = 1;
    }

    public GlobOptFuncCcc1(Curve2DAdaptorBase c1, Curve2DAdaptorBase c2)
    {
        _c13d = null;
        _c23d = null;
        _c12d = c1;
        _c22d = c2;
        _type = 2;
    }

    public override int VariableCount => NbVariables();

    public override bool Value(math_Vector x, out double f)
    {
        if (_type == 1)
        {
            return _Value(_c13d!, _c23d!, x, out f);
        }
        else
        {
            return _Value(_c12d!, _c22d!, x, out f);
        }
    }

    public override bool Gradient(math_Vector x, math_Vector g)
    {
        if (_type == 1)
        {
            return _Gradient(_c13d!, _c23d!, x, g);
        }
        else
        {
            return _Gradient(_c12d!, _c22d!, x, g);
        }
    }

    public override bool Values(math_Vector x, out double f, math_Vector g) => (Value(x, out f) && Gradient(x, g));
}

/// <summary>The distance between two curves with its gradient and hessian (see <see cref="GlobOptFuncCcc0"/>);
/// the gradient and the hessian are those of the square distance.</summary>
internal class GlobOptFuncCcc2 : MultipleVarFunctionWithHessian
{
    private readonly CurveAdaptorBase? _c13d, _c23d;
    private readonly Curve2DAdaptorBase? _c12d, _c22d;
    private readonly int _type;

    public GlobOptFuncCcc2(CurveAdaptorBase c1, CurveAdaptorBase c2)
    {
        _c13d = c1;
        _c23d = c2;
        _c12d = null;
        _c22d = null;
        _type = 1;
    }

    public GlobOptFuncCcc2(Curve2DAdaptorBase c1, Curve2DAdaptorBase c2)
    {
        _c13d = null;
        _c23d = null;
        _c12d = c1;
        _c22d = c2;
        _type = 2;
    }

    public override int VariableCount => NbVariables();

    public override bool Value(math_Vector x, out double f)
    {
        if (_type == 1)
        {
            return _Value(_c13d!, _c23d!, x, out f);
        }
        else
        {
            return _Value(_c12d!, _c22d!, x, out f);
        }
    }

    public override bool Gradient(math_Vector x, math_Vector g)
    {
        if (_type == 1)
        {
            return _Gradient(_c13d!, _c23d!, x, g);
        }
        else
        {
            return _Gradient(_c12d!, _c22d!, x, g);
        }
    }

    public override bool Values(math_Vector x, out double f, math_Vector g) => (Value(x, out f) && Gradient(x, g));

    public override bool Values(math_Vector x, out double f, math_Vector g, Matrix h)
    {
        bool isHessianComputed = false;
        if (_type == 1)
        {
            isHessianComputed = Hessian(_c13d!, _c23d!, x, h);
        }
        else
        {
            isHessianComputed = Hessian(_c12d!, _c22d!, x, h);
        }

        return (Value(x, out f) && Gradient(x, g) && isHessianComputed);
    }
}
