// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_PrjFunc.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_PrjFunc.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Created on: 1997-11-06
// Created by: Roman BORISOV

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Projection;

/// <summary>
/// The function whose zero is the orthogonal projection of a curve point on a surface: the gradient of
/// <c>|S(u, v) - C(t)|</c> in two of the variables (t, u, v), the third one fixed (1: t, 2: u, 3: v).
/// </summary>
/// <remarks>
/// A working object (decision 21). It keeps the caller's adaptors, as OCCT keeps its pointers: they must outlive
/// the function. <see cref="Values"/> remembers the last point it was evaluated at, which
/// <see cref="Solution"/> returns.
/// </remarks>
internal class PrjFunc : FunctionSetWithDerivatives
{
    private readonly CurveAdaptorBase _curve;
    private readonly SurfaceAdaptorBase _surface;
    private double _myt;
    private double _u;
    private double _v;
    private readonly int _fix;
    private readonly double _norm;

    /// <exception cref="ConstructionErrorException"><paramref name="fix"/> is not 1, 2 or 3.</exception>
    public PrjFunc(CurveAdaptorBase c, double fixVal, SurfaceAdaptorBase s, int fix)
    {
        _curve = c;
        _surface = s;
        _myt = 0;
        _u = 0;
        _v = 0;
        _fix = fix;
        _norm = Min(1.0, Min(_surface.UResolution(1.0), _surface.VResolution(1.0)));
        // myNorm=1.;
        switch (_fix)
        {
            case 1:
                _myt = fixVal;
                break;
            case 2:
                _u = fixVal;
                break;
            case 3:
                _v = fixVal;
                break;
            default:
                throw new ConstructionErrorException();
        }
    }

    /// <summary>The number of variables of the function: 2.</summary>
    public override int NbVariables() => 2;

    /// <summary>The number of equations of the function: 2.</summary>
    public override int NbEquations() => 2;

    public override bool Value(math_Vector x, math_Vector f)
    {
        Matrix d = new(1, 2, 1, 2);
        return Values(x, f, d);
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        math_Vector f = new(1, 2);
        return Values(x, f, d);
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        double u = 0.0, v = 0.0, t = 0.0;
        switch (_fix)
        {
            case 1:
                t = _myt;
                u = x[1];
                v = x[2];
                break;
            case 2:
                t = x[1];
                u = _u;
                v = x[2];
                break;
            case 3:
                t = x[1];
                u = x[2];
                v = _v;
                break;
        }

        /*  if(X(1) > mySup.X() || X(1) < myInf.X()) return false;
          if(X(2) > mySup.Y() || X(2) < myInf.Y()) return false;
        */
        Point3D s, c;
        Vector3D DS1_u, DS1_v, DS2_u, DS2_uv, DS2_v, DC1_t;
        _curve.D1(t, out c, out DC1_t);
        _surface.D2(u, v, out s, out DS1_u, out DS1_v, out DS2_u, out DS2_v, out DS2_uv);

        Vector3D V = new(c, s);

        f[1] = V * DS1_u * _norm;
        f[2] = V * DS1_v * _norm;

        switch (_fix)
        {
            case 1:
                d[1, 1] = (DS1_u.SquareMagnitude + V * DS2_u) * _norm; // dE1/du
                d[1, 2] = (DS1_v * DS1_u + V * DS2_uv) * _norm;          // dE1/dv
                d[2, 1] = d[1, 2];                                        // dE2/du
                d[2, 2] = (DS1_v.SquareMagnitude + V * DS2_v) * _norm; // dE2/dv
                break;
            case 2:
                d[1, 1] = (-DC1_t * DS1_u) * _norm;                      // dE1/dt
                d[1, 2] = (DS1_v * DS1_u + V * DS2_uv) * _norm;          // dE1/dv
                d[2, 1] = (-DC1_t * DS1_v) * _norm;                      // dE2/dt
                d[2, 2] = (DS1_v.SquareMagnitude + V * DS2_v) * _norm; // dE2/dv
                break;
            case 3:
                d[1, 1] = -DC1_t * DS1_u * _norm;                        // dE1/dt
                d[1, 2] = (DS1_u.SquareMagnitude + V * DS2_u) * _norm; // dE1/du
                d[2, 1] = -DC1_t * DS1_v * _norm;                        // dE2/dt
                d[2, 2] = (DS1_v * DS1_u + V * DS2_uv) * _norm;          // dE2/du
                break;
        }

        _u = u;
        _v = v;
        _myt = t;

        return true;
    }

    /// <summary>
    /// The point of the last evaluation in the two free variables: (u, v) for Fix 1, (t, v) for Fix 2, (t, u) for
    /// Fix 3.
    /// </summary>
    public Point2D Solution()
    {
        switch (_fix)
        {
            case 1:
                return new Point2D(_u, _v);
            case 2:
                return new Point2D(_myt, _v);
            case 3:
                return new Point2D(_myt, _u);
        }
        // For NT, even if we never reach this point.
        return new Point2D(0.0, 0.0);
    }
}
