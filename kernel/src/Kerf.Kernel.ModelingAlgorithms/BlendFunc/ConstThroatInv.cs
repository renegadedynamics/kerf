// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstThroatInv.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_ConstThroatInv.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>The function of a chamfer of constant throat with one end on a surface's boundary.</summary>
internal class ConstThroatInv : GenChamfInv
{
    protected double _throat;
    protected double _param;
    protected double _sign1;
    protected double _sign2;

    protected Point3D _ptgui;
    protected Vector3D _nplan;
    protected double _normtg;
    protected double _d;
    protected Vector3D _d1gui;
    protected Vector3D _d2gui;

    protected Point3D _pts1;
    protected Point3D _pts2;
    protected Vector3D _d1u1;
    protected Vector3D _d1v1;
    protected Vector3D _d1u2;
    protected Vector3D _d1v2;

    public ConstThroatInv(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, CurveAdaptorBase c)
        : base(s1, s2, c)
    {
        _throat = 0.0;
        _param = 0.0;
        _sign1 = 0.0;
        _sign2 = 0.0;
        _normtg = 0.0;
        _d = 0.0;
    }

    /// <summary>Sets the throat and the "quadrant"; the second distance is not used.</summary>
    public override void Set(double throat, double dist2, int choix)
    {
        _throat = throat;
        _choix = choix;
        switch (_choix)
        {
            case 1:
            case 2:
                {
                    _sign1 = -1;
                    _sign2 = -1;
                }
                break;
            case 3:
            case 4:
                {
                    _sign1 = 1;
                    _sign2 = -1;
                }
                break;
            case 5:
            case 6:
                {
                    _sign1 = 1;
                    _sign2 = 1;
                }
                break;
            case 7:
            case 8:
                {
                    _sign1 = -1;
                    _sign2 = 1;
                }
                break;
            default:
                _sign1 = -1;
                _sign2 = -1;
                break;
        }
    }

    public override bool IsSolution(math_Vector sol, double tol)
    {
        math_Vector valsol = new(1, 4);
        Value(sol, valsol);
        return Math.Abs(valsol[1]) <= tol && Math.Abs(valsol[2]) <= tol
               && Math.Abs(valsol[3]) <= tol * tol && Math.Abs(valsol[4]) <= tol * tol;
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        _csurf!.D1(x[1], out Point2D p2d, out _);

        _param = x[2];
        _curv.D2(_param, out _ptgui, out _d1gui, out _d2gui);
        _normtg = _d1gui.Magnitude;
        _nplan = _d1gui.Normalized();
        _d = -(_nplan.XYZ().Dot(_ptgui.XYZ()));

        math_Vector xx = new(1, 4);
        if (_first)
        {
            xx[1] = p2d.X;
            xx[2] = p2d.Y;
            xx[3] = x[3];
            xx[4] = x[4];
        }
        else
        {
            xx[1] = x[3];
            xx[2] = x[4];
            xx[3] = p2d.X;
            xx[4] = p2d.Y;
        }

        _surf1.D0(xx[1], xx[2], out _pts1);
        _surf2.D0(xx[3], xx[4], out _pts2);

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
        Vector3D dnplan = new Vector3D(), temp = new Vector3D(), temp1 = new Vector3D(), temp2 = new Vector3D(),
               tempmid = new Vector3D();
        math_Vector xx = new(1, 4);

        _csurf!.D1(x[1], out Point2D p2d, out Vector2D v2d);

        _param = x[2];
        _curv.D2(_param, out _ptgui, out _d1gui, out _d2gui);
        _normtg = _d1gui.Magnitude;
        _nplan = _d1gui.Normalized();
        _d = -(_nplan.XYZ().Dot(_ptgui.XYZ()));

        dnplan = Vector3D.LinearForm(1.0 / _normtg, _d2gui, -1.0 / _normtg * (_nplan.Dot(_d2gui)), _nplan);

        // OCCT: temp1, temp2 and tempmid use the points of the previous call, before the D1 below (decision 1040).
        temp1 = temp1.WithXYZ(_pts1.XYZ() - _ptgui.XYZ());
        temp2 = temp2.WithXYZ(_pts2.XYZ() - _ptgui.XYZ());
        tempmid = tempmid.WithXYZ((_pts1.XYZ() + _pts2.XYZ()) / 2 - _ptgui.XYZ());

        if (_first)
        {
            xx[1] = p2d.X;
            xx[2] = p2d.Y;
            xx[3] = x[3];
            xx[4] = x[4];
        }
        else
        {
            xx[1] = x[3];
            xx[2] = x[4];
            xx[3] = p2d.X;
            xx[4] = p2d.Y;
        }

        _surf1.D1(xx[1], xx[2], out _pts1, out _d1u1, out _d1v1);
        _surf2.D1(xx[3], xx[4], out _pts2, out _d1u2, out _d1v2);

        if (_first)
        {
            // p2d = pts est sur surf1
            temp = Vector3D.LinearForm(v2d.X, _d1u1, v2d.Y, _d1v1);
            d[1, 1] = _nplan.Dot(temp);
            d[2, 1] = 0.0;
            d[3, 1] = new Vector3D(_ptgui, _pts1).Dot(temp);
            d[4, 1] = 2 * (new Vector3D(_ptgui, _pts1).Dot(temp));

            d[1, 3] = 0.0;
            d[1, 4] = 0.0;
            d[2, 3] = _nplan.Dot(_d1u2);
            d[2, 4] = _nplan.Dot(_d1v2);
            d[3, 3] = new Vector3D((_pts1.XYZ() + _pts2.XYZ()) / 2 - _ptgui.XYZ()).Dot(_d1u2);
            d[3, 4] = new Vector3D((_pts1.XYZ() + _pts2.XYZ()) / 2 - _ptgui.XYZ()).Dot(_d1v2);
            d[4, 3] = -2.0 * new Vector3D(_ptgui, _pts2).Dot(_d1u2);
            d[4, 4] = -2.0 * new Vector3D(_ptgui, _pts2).Dot(_d1v2);
        }
        else
        {
            //  p2d = pts est sur surf2
            temp = Vector3D.LinearForm(v2d.X, _d1u2, v2d.Y, _d1v2);
            d[1, 1] = 0.0;
            d[2, 1] = _nplan.Dot(temp);
            d[3, 1] = new Vector3D(_ptgui, _pts2).Dot(temp);
            d[4, 1] = -2 * (new Vector3D(_ptgui, _pts2).Dot(temp));

            d[1, 3] = _nplan.Dot(_d1u1);
            d[1, 4] = _nplan.Dot(_d1v1);
            d[2, 3] = 0.0;
            d[2, 4] = 0.0;
            d[3, 3] = new Vector3D((_pts1.XYZ() + _pts2.XYZ()) / 2 - _ptgui.XYZ()).Dot(_d1u1);
            d[3, 4] = new Vector3D((_pts1.XYZ() + _pts2.XYZ()) / 2 - _ptgui.XYZ()).Dot(_d1v1);
            d[4, 3] = 2.0 * new Vector3D(_ptgui, _pts1).Dot(_d1u1);
            d[4, 4] = 2.0 * new Vector3D(_ptgui, _pts1).Dot(_d1v1);
        }

        d[1, 2] = dnplan.Dot(temp1) - _nplan.Dot(_d1gui);
        d[2, 2] = dnplan.Dot(temp2) - _nplan.Dot(_d1gui);
        d[3, 2] = -2.0 * _d1gui.Dot(tempmid);
        d[4, 2] = 2.0 * _d1gui.Dot(temp1) - 2.0 * _d1gui.Dot(temp2);

        return true;
    }
}
