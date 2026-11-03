// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstThroatWithPenetration.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstThroatWithPenetration.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>
/// The function of a chamfer of constant throat whose section is a right-angled triangle: the first surface is
/// virtually offset into the solid, the apex lies on its intersection with the second surface, the right angle is at
/// the top of the chamfer, and the leg from the apex to the top (the throat) is constant.
/// </summary>
public class ConstThroatWithPenetration : ConstThroat
{
    public ConstThroatWithPenetration(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, CurveAdaptorBase c)
        : base(s1, s2, c)
    {
    }

    public override bool IsSolution(math_Vector sol, double tol)
    {
        math_Vector secmember = new(1, 4), valsol = new(1, 4);
        Matrix gradsol = new(1, 4, 1, 4);

        Value(sol, valsol);
        Derivatives(sol, gradsol);

        _tol = tol;

        Vector3D dnplan = new Vector3D(), temp1 = new Vector3D(), temp2 = new Vector3D(), temp3 = new Vector3D();

        if (Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol && Math.Abs(valsol[3]) <= tol * tol
            && Math.Abs(valsol[4]) <= tol)
        {
            dnplan.SetLinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

            temp1.SetXYZ(_pts1.XYZ() - _ptgui.XYZ());
            temp2.SetXYZ(_pts2.XYZ() - _ptgui.XYZ());
            temp3.SetXYZ(_pts2.XYZ() - _pts1.XYZ());

            _surf1.D1(sol[1], sol[2], out _pts1, out _d1u1, out _d1v1);
            _surf2.D1(sol[3], sol[4], out _pts2, out _d1u2, out _d1v2);

            secmember[1] = _nplan.Dot(_d1gui) - dnplan.Dot(temp1);
            secmember[2] = _nplan.Dot(_d1gui) - dnplan.Dot(temp2);
            secmember[3] = 2.0 * _d1gui.Dot(temp1);
            secmember[4] = _d1gui.Dot(temp3);

            Gauss resol = new(gradsol);
            if (resol.IsDone)
            {
                resol.Solve(secmember);
                _tg1.SetLinearForm(secmember[1], _d1u1, secmember[2], _d1v1);
                _tg2.SetLinearForm(secmember[3], _d1u2, secmember[4], _d1v2);
                _tg12d.SetCoord(secmember[1], secmember[2]);
                _tg22d.SetCoord(secmember[3], secmember[4]);
                _istangent = false;
            }
            else
            {
                _istangent = true;
            }

            _distmin = Min(_distmin, _pts1.Distance(_pts2));

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

        Vector3D vref = new Vector3D(_ptgui, _pts1);

        f[3] = vref.SquareMagnitude() - _throat * _throat;

        Vector3D vec12 = new Vector3D(_pts1, _pts2);

        f[4] = vref.Dot(vec12);

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
        d[3, 1] = 2.0 * new Vector3D(_ptgui, _pts1).Dot(_d1u1);
        d[3, 2] = 2.0 * new Vector3D(_ptgui, _pts1).Dot(_d1v1);
        d[3, 3] = 0.0;
        d[3, 4] = 0.0;
        d[4, 1] = _d1u1.Dot(new Vector3D(_pts1, _pts2)) - new Vector3D(_ptgui, _pts1).Dot(_d1u1);
        d[4, 2] = _d1v1.Dot(new Vector3D(_pts1, _pts2)) - new Vector3D(_ptgui, _pts1).Dot(_d1v1);
        d[4, 3] = new Vector3D(_ptgui, _pts1).Dot(_d1u2);
        d[4, 4] = new Vector3D(_ptgui, _pts1).Dot(_d1v2);

        return true;
    }

    public override Vector3D TangentOnS1()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_ConstThroatWithPenetration::TangentOnS1");
        }

        return _tg1;
    }

    public override Vector3D TangentOnS2()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_ConstThroatWithPenetration::TangentOnS2");
        }

        return _tg2;
    }

    public override Vector2D Tangent2dOnS1()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_ConstThroatWithPenetration::Tangent2dOnS1");
        }

        return _tg12d;
    }

    public override Vector2D Tangent2dOnS2()
    {
        if (_istangent)
        {
            throw new DomainErrorException("BlendFunc_ConstThroatWithPenetration::Tangent2dOnS2");
        }

        return _tg22d;
    }

    /// <summary>Not implemented: throws <see cref="MethodNotImplementedException"/>.</summary>
    public override double GetSectionSize()
    {
        throw new MethodNotImplementedException("BlendFunc_ConstThroatWithPenetration::GetSectionSize()");
    }
}
