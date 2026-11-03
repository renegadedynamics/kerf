// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstThroat.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstThroat.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>
/// The function of a symmetric chamfer of constant throat: the height of the isosceles triangle in the section.
/// </summary>
public class ConstThroat : GenChamfer
{
    protected Point3D _pts1;
    protected Point3D _pts2;
    protected Vector3D _d1u1;
    protected Vector3D _d1v1;
    protected Vector3D _d1u2;
    protected Vector3D _d1v2;
    protected bool _istangent;
    protected Vector3D _tg1;
    protected Vector2D _tg12d;
    protected Vector3D _tg2;
    protected Vector2D _tg22d;
    protected double _param;
    protected double _throat;

    protected Point3D _ptgui;
    protected Vector3D _nplan;
    protected double _normtg;
    protected double _d;
    protected Vector3D _d1gui;
    protected Vector3D _d2gui;

    public ConstThroat(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, CurveAdaptorBase c)
        : base(s1, s2, c)
    {
        _istangent = false;
        _param = 0.0;
        _throat = 0.0;
        _normtg = 0.0;
        _d = 0.0;
    }

    /// <summary>Sets the throat and the "quadrant"; the second distance is not used.</summary>
    public override void Set(double aThroat, double dist2, int choix)
    {
        _throat = aThroat;
        _choix = choix;
    }

    public override void Set(double param)
    {
        _param = param;
        _curv.D2(_param, out _ptgui, out _d1gui, out _d2gui);
        _normtg = _d1gui.Magnitude;
        _nplan = _d1gui.Normalized();
        _d = -(_nplan.XYZ().Dot(_ptgui.XYZ()));
    }

    public override bool IsSolution(math_Vector sol, double tol)
    {
        math_Vector secmember = new(1, 4), valsol = new(1, 4);
        Matrix gradsol = new(1, 4, 1, 4);

        Value(sol, valsol);
        Derivatives(sol, gradsol);

        _tol = tol;

        Vector3D dnplan = new Vector3D(), temp1 = new Vector3D(), temp2 = new Vector3D(), tempmid = new Vector3D();

        if (Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol && Math.Abs(valsol[3]) <= tol * tol
            && Math.Abs(valsol[4]) <= tol * tol)
        {
            dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

            temp1 = temp1.WithXYZ(_pts1.XYZ() - _ptgui.XYZ());
            temp2 = temp2.WithXYZ(_pts2.XYZ() - _ptgui.XYZ());
            tempmid = tempmid.WithXYZ((_pts1.XYZ() + _pts2.XYZ()) / 2 - _ptgui.XYZ());

            _surf1.D1(sol[1], sol[2], out _pts1, out _d1u1, out _d1v1);
            _surf2.D1(sol[3], sol[4], out _pts2, out _d1u2, out _d1v2);

            secmember[1] = _nplan.Dot(_d1gui) - dnplan.Dot(temp1);
            secmember[2] = _nplan.Dot(_d1gui) - dnplan.Dot(temp2);
            secmember[3] = 2.0 * _d1gui.Dot(tempmid);
            secmember[4] = 2.0 * _d1gui.Dot(temp2) - 2.0 * _d1gui.Dot(temp1);

            Gauss resol = new(gradsol);
            if (resol.IsDone)
            {
                resol.Solve(secmember);
                _tg1 = Vector3D.LinearForm(secmember[1], _d1u1, secmember[2], _d1v1);
                _tg2 = Vector3D.LinearForm(secmember[3], _d1u2, secmember[4], _d1v2);
                _tg12d = _tg12d.WithCoord(secmember[1], secmember[2]);
                _tg22d = _tg22d.WithCoord(secmember[3], secmember[4]);
                _istangent = false;
            }
            else
            {
                _istangent = true;
            }

            _distmin = Min(_distmin, _pts1.DistanceTo(_pts2));

            return true;
        }

        return false;
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        _surf1.D0(x[1], x[2], out _pts1);
        _surf2.D0(x[3], x[4], out _pts2);

        f[1] = _nplan.XYZ().Dot(_pts1.XYZ()) + _d;
        f[2] = _nplan.XYZ().Dot(_pts2.XYZ()) + _d;

        Point3D ptmid = new Point3D((_pts1.XYZ() + _pts2.XYZ()) / 2);
        Vector3D vmid = new Vector3D(_ptgui, ptmid);

        f[3] = vmid.SquareMagnitude - _throat * _throat;

        Vector3D vref1 = new Vector3D(_ptgui, _pts1);
        Vector3D vref2 = new Vector3D(_ptgui, _pts2);

        f[4] = vref1.SquareMagnitude - vref2.SquareMagnitude;

        return true;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        _surf1.D1(x[1], x[2], out _pts1, out _d1u1, out _d1v1);
        _surf2.D1(x[3], x[4], out _pts2, out _d1u2, out _d1v2);

        d[1, 1] = _nplan.Dot(_d1u1);
        d[1, 2] = _nplan.Dot(_d1v1);
        d[1, 3] = 0.0;
        d[1, 4] = 0.0;
        d[2, 1] = 0.0;
        d[2, 2] = 0.0;
        d[2, 3] = _nplan.Dot(_d1u2);
        d[2, 4] = _nplan.Dot(_d1v2);
        d[3, 1] = new Vector3D((_pts1.XYZ() + _pts2.XYZ()) / 2 - _ptgui.XYZ()).Dot(_d1u1);
        d[3, 2] = new Vector3D((_pts1.XYZ() + _pts2.XYZ()) / 2 - _ptgui.XYZ()).Dot(_d1v1);
        d[3, 3] = new Vector3D((_pts1.XYZ() + _pts2.XYZ()) / 2 - _ptgui.XYZ()).Dot(_d1u2);
        d[3, 4] = new Vector3D((_pts1.XYZ() + _pts2.XYZ()) / 2 - _ptgui.XYZ()).Dot(_d1v2);
        d[4, 1] = 2.0 * new Vector3D(_ptgui, _pts1).Dot(_d1u1);
        d[4, 2] = 2.0 * new Vector3D(_ptgui, _pts1).Dot(_d1v1);
        d[4, 3] = -2.0 * new Vector3D(_ptgui, _pts2).Dot(_d1u2);
        d[4, 4] = -2.0 * new Vector3D(_ptgui, _pts2).Dot(_d1v2);

        return true;
    }

    public override Point3D PointOnS1()
    {
        return _pts1;
    }

    public override Point3D PointOnS2()
    {
        return _pts2;
    }

    public override bool IsTangencyPoint
    {
        get
        {
            return _istangent;
        }
    }

    public override Vector3D TangentOnS1()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_ConstThroat::TangentOnS1");
        }

        return _tg1;
    }

    public override Vector3D TangentOnS2()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_ConstThroat::TangentOnS2");
        }

        return _tg2;
    }

    public override Vector2D Tangent2dOnS1()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_ConstThroat::Tangent2dOnS1");
        }

        return _tg12d;
    }

    public override Vector2D Tangent2dOnS2()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_ConstThroat::Tangent2dOnS2");
        }

        return _tg22d;
    }

    public override void Tangent(double u1, double v1, double u2, double v2, out Vector3D tgF, out Vector3D tgL,
                                 out Vector3D nmF, out Vector3D nmL)
    {
        bool revF = false;
        bool revL = false;

        _surf1.D1(u1, v1, out _, out Vector3D d1u, out Vector3D d1v);
        nmF = d1u.Crossed(d1v);

        _surf2.D1(u2, v2, out _, out d1u, out d1v);
        nmL = d1u.Crossed(d1v);

        tgF = (_nplan.Crossed(nmF)).Normalized();
        tgL = (_nplan.Crossed(nmL)).Normalized();

        if ((_choix == 2) || (_choix == 5))
        {
            revF = true;
            revL = true;
        }

        if ((_choix == 4) || (_choix == 7))
        {
            revL = true;
        }

        if ((_choix == 3) || (_choix == 8))
        {
            revF = true;
        }

        if (revF)
        {
            tgF = tgF.Reversed();
        }

        if (revL)
        {
            tgL = tgL.Reversed();
        }
    }

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double GetSectionSize()
    {
        throw new MethodNotImplementedException("BlendFunc_ConstThroat::GetSectionSize()");
    }
}
