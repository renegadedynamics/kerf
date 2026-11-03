// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Corde.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Corde.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>
/// The point pts of a surface on the plane normal to a guide curve at a chosen parameter, at a given distance from
/// the guide: X(1), X(2) are the parameters U, V of pts on the surface.
/// </summary>
/// <remarks>
/// OCCT holds it by value inside the chamfer functions; here it is a working class (decision 21).
/// </remarks>
public sealed class Corde
{
    private readonly SurfaceAdaptorBase _surf;
    private readonly CurveAdaptorBase _guide;
    private Point3D _pts;
    // OCCT's pt2d is never used: left out.
    private double _dis;
    private double _normtg;
    private double _d;
    private Point3D _ptgui;
    private Vector3D _nplan;
    private Vector3D _d1gui;
    private Vector3D _d2gui;
    private Vector3D _tgs;
    private Vector2D _tg2d;
    private bool _istangent;

    public Corde(SurfaceAdaptorBase s, CurveAdaptorBase cg)
    {
        _surf = s;
        _guide = cg;
        _dis = 0.0;
        _normtg = 0.0;
        _d = 0.0;
        _istangent = false;
    }

    public void SetDist(double dist)
    {
        _dis = dist;
    }

    public void SetParam(double param)
    {
        _guide.D2(param, out _ptgui, out _d1gui, out _d2gui);
        _normtg = _d1gui.Magnitude();
        _nplan = _d1gui.Normalized();
        _d = -(_nplan.XYZ().Dot(_ptgui.XYZ()));
    }

    //=======================================================================
    // function : Value
    // purpose  : returns F(U,V)
    //=======================================================================
    public bool Value(math_Vector x, math_Vector f)
    {
        _surf.D1(x[1], x[2], out _pts, out _, out _);

        f[1] = _nplan.XYZ().Dot(_pts.XYZ()) + _d;
        Vector3D vref = new Vector3D(_ptgui, _pts);

        f[2] = vref.SquareMagnitude() - _dis * _dis;

        return true;
    }

    //=======================================================================
    // function : Derivatives
    // purpose  : D = grad F(U,V)
    //=======================================================================
    public bool Derivatives(math_Vector x, Matrix d)
    {
        _surf.D1(x[1], x[2], out _pts, out Vector3D d1u, out Vector3D d1v);

        d[1, 1] = _nplan.Dot(d1u);
        d[1, 2] = _nplan.Dot(d1v);
        d[2, 1] = 2.0 * new Vector3D(_ptgui, _pts).Dot(d1u);
        d[2, 2] = 2.0 * new Vector3D(_ptgui, _pts).Dot(d1v);

        return true;
    }

    public Point3D PointOnS()
    {
        return _pts;
    }

    public Point3D PointOnGuide()
    {
        return _ptgui;
    }

    public Vector3D NPlan()
    {
        return _nplan;
    }

    public bool IsTangencyPoint
    {
        get
        {
            return _istangent;
        }
    }

    public Vector3D TangentOnS()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_Corde::TangentOnS");
        }

        return _tgs;
    }

    public Vector2D Tangent2dOnS()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_Corde::Tangent2dOnS");
        }

        return _tg2d;
    }

    /// <summary>The derivative of F in the guide's parameter at the solution <paramref name="sol"/>.</summary>
    public void DerFguide(math_Vector sol, ref Vector2D derF)
    {
        _surf.D1(sol[1], sol[2], out _pts, out _, out _);
        Vector3D dnplan = new Vector3D();
        dnplan.SetLinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

        Vector3D temp = new Vector3D(_pts.XYZ() - _ptgui.XYZ());

        derF.SetX(dnplan.Dot(temp) - _nplan.Dot(_d1gui));
        derF.SetY(-2.0 * _d1gui.Dot(temp));
    }

    public bool IsSolution(math_Vector sol, double tol)
    {
        math_Vector secmember = new(1, 2), valsol = new(1, 2);
        Matrix gradsol = new(1, 2, 1, 2);
        Vector3D dnplan = new Vector3D(), temp = new Vector3D();

        Value(sol, valsol);
        Derivatives(sol, gradsol);
        if (Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol * tol)
        {
            _surf.D1(sol[1], sol[2], out _pts, out Vector3D d1u, out Vector3D d1v);
            dnplan.SetLinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

            temp.SetXYZ(_pts.XYZ() - _ptgui.XYZ());

            secmember[1] = _nplan.Dot(_d1gui) - dnplan.Dot(temp);
            secmember[2] = 2.0 * _d1gui.Dot(temp);

            //  gradsol*der = secmember
            //  with  der(1) = dU/dW, der(2) = dU/dW, W is the guide parameter

            Gauss resol = new(gradsol);
            if (resol.IsDone)
            {
                resol.Solve(secmember);
                _tgs.SetLinearForm(secmember[1], d1u, secmember[2], d1v);
                _tg2d.SetCoord(secmember[1], secmember[2]);
                _istangent = false;
            }
            else
            {
                _istangent = true;
            }

            return true;
        }

        return false;
    }
}
