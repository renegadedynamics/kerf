// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_EvolRad.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_EvolRad.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).


using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics;
using Kerf.Geometry;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>
/// The function of a fillet of evolving radius between two surfaces along a guide curve: as BlendFunc_ConstRad,
/// with the radius given by a law of the guide's parameter.
/// </summary>
/// <remarks>
/// A working object (decision 21): <see cref="ComputeValues"/> caches its last positioning, and the walking reads
/// the cached points and tangents. OCCT's <c>thread_local</c> scratch values of ComputeValues are read across calls
/// (when the cache skips a positioning), so they are the instance's (decision 1037 had them per thread).
/// </remarks>
internal class EvolRad : BlendFunction
{
    private const double Eps = 1.0e-15;

    // Scratch reused across calls via the t_OK / myX_OK guards of ComputeValues (OCCT's thread_local).
    private Vector3D _d3u1, _d3v1, _d3uuv1, _d3uvv1, _d3u2, _d3v2, _d3uuv2, _d3uvv2;
    private Vector3D _d1gui, _d2gui, _d3gui;
    private Point3D _ptgui;
    private double _invnormtg, _dinvnormtg;

    private readonly SurfaceAdaptorBase _surf1;
    private readonly SurfaceAdaptorBase _surf2;
    private readonly CurveAdaptorBase _curv;
    private CurveAdaptorBase _tcurv;
    private readonly LawsFunction _fevol;
    private LawsFunction _tevol;
    private Point3D _pts1;
    private Point3D _pts2;
    private bool _istangent;
    private Vector3D _tg1;
    private Vector2D _tg12d;
    private Vector3D _tg2;
    private Vector2D _tg22d;
    private double _param;
    private double _sg1;
    private double _sg2;
    private double _ray;
    private double _dray;
    private double _d2ray;
    private int _choix;
    private int _xOrder;
    private int _tOrder;
    private readonly math_Vector _xval;
    private double _tval;
    private Vector3D _d1u1;
    private Vector3D _d1u2;
    private Vector3D _d1v1;
    private Vector3D _d1v2;
    private Vector3D _d2u1;
    private Vector3D _d2v1;
    private Vector3D _d2uv1;
    private Vector3D _d2u2;
    private Vector3D _d2v2;
    private Vector3D _d2uv2;
    private Vector3D _dn1w;
    private Vector3D _dn2w;
    private Vector3D _d2n1w;
    private Vector3D _d2n2w;
    private Vector3D _nplan;
    private Vector3D _nsurf1;
    private Vector3D _nsurf2;
    private Vector3D _dns1u1;
    private Vector3D _dns1u2;
    private Vector3D _dns1v1;
    private Vector3D _dns1v2;
    private Vector3D _dnplan;
    private Vector3D _d2nplan;
    // OCCT's dnsurf1 and dnsurf2 are never used: left out.
    private Vector3D _dndu1;
    private Vector3D _dndu2;
    private Vector3D _dndv1;
    private Vector3D _dndv2;
    private Vector3D _d2ndu1;
    private Vector3D _d2ndu2;
    private Vector3D _d2ndv1;
    private Vector3D _d2ndv2;
    private Vector3D _d2nduv1;
    private Vector3D _d2nduv2;
    private Vector3D _d2ndtu1;
    private Vector3D _d2ndtu2;
    private Vector3D _d2ndtv1;
    private Vector3D _d2ndtv2;
    private readonly math_Vector _e;
    private readonly Matrix _dedx;
    private readonly math_Vector _dedt;
    private readonly Tensor _d2EDX2;
    private readonly Matrix _d2EDXDT;
    private readonly math_Vector _d2EDT2;
    private double _minang;
    private double _maxang;
    private double _lengthmin;
    private double _lengthmax;
    private double _distmin;
    private SectionShape _sShape;
    private ParameterisationType _tConv;

    /// <remarks>OCCT leaves param, the radius and the "quadrant" uninitialised; here they are zero.</remarks>
    public EvolRad(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, CurveAdaptorBase c, LawsFunction law)
    {
        _surf1 = s1;
        _surf2 = s2;
        _curv = c;
        _tcurv = c;
        _istangent = true;
        _xval = new math_Vector(1, 4);
        _e = new math_Vector(1, 4);
        _dedx = new Matrix(1, 4, 1, 4);
        _dedt = new math_Vector(1, 4);
        _d2EDX2 = new Tensor(4, 4, 4);
        _d2EDXDT = new Matrix(1, 4, 1, 4);
        _d2EDT2 = new math_Vector(1, 4);
        _minang = RealLast();
        _maxang = RealFirst();
        _lengthmin = RealLast();
        _lengthmax = RealFirst();
        _distmin = RealLast();
        _sShape = SectionShape.Rational;

        _fevol = law;
        _tevol = law;

        // Initialisaton of cash control variables.
        _tval = -9.876e100;
        _xval.Init(-9.876e100);
        _xOrder = -1;
        _tOrder = -1;
    }

    /// <summary>Returns 4.</summary>
    public override int NbEquations()
    {
        return 4;
    }

    /// <summary>Sets the "quadrant" <paramref name="choix"/> (1 to 8) in which the fillet lies.</summary>
    public void Set(int choix)
    {
        _choix = choix;
        switch (_choix)
        {
            case 1:
            case 2:
                {
                    _sg1 = -1.0;
                    _sg2 = -1.0;
                }
                break;
            case 3:
            case 4:
                {
                    _sg1 = 1.0;
                    _sg2 = -1.0;
                }
                break;
            case 5:
            case 6:
                {
                    _sg1 = 1.0;
                    _sg2 = 1.0;
                }
                break;
            case 7:
            case 8:
                {
                    _sg1 = -1.0;
                    _sg2 = 1.0;
                }
                break;
            default:
                _sg1 = _sg2 = -1.0;
                break;
        }
    }

    /// <summary>Sets the type of the sections the approximation makes.</summary>
    public void Set(SectionShape typeSection)
    {
        _sShape = typeSection;
    }

    //=======================================================================
    // function : ComputeValues
    // purpose  : OBLIGATORY passage for all computations
    //           This method manages the positioning on Surfaces and Curves
    //           Partial calculation of equations and their derivatives
    //           Storage of some intermediary results in fields to be
    //           used in other methods.
    //=======================================================================
    private bool ComputeValues(math_Vector x, int order, bool byParam = false, double param = 0)
    {
        double t = param, aux;

        // Case of implicite parameter
        if (!byParam)
        {
            t = _param;
        }

        // Is the work already done ?
        bool myX_OK = (order <= _xOrder);
        for (int ii = 1; ((ii <= x.Length()) && myX_OK); ii++)
        {
            myX_OK = (x[ii] == _xval[ii]);
        }

        bool t_OK = ((t == _tval) && ((order <= _tOrder) || (!byParam)));

        if (myX_OK && (t_OK))
        {
            return true;
        }

        // Processing of t
        if (!t_OK)
        {
            _tval = t;
            if (byParam)
            {
                _tOrder = order;
            }
            else
            {
                _tOrder = 0;
            }

            //----- Positioning on the curve ----------------
            switch (_tOrder)
            {
                case 0:
                    {
                        _tcurv.D1(t, out _ptgui, out _d1gui);
                        _nplan = _d1gui.Normalized();
                        _ray = _tevol.Value(t);
                    }
                    break;

                case 1:
                    {
                        _tcurv.D2(t, out _ptgui, out _d1gui, out _d2gui);
                        _nplan = _d1gui.Normalized();
                        _invnormtg = ((double)1) / _d1gui.Magnitude;
                        _dnplan = Vector3D.LinearForm(_invnormtg, _d2gui, -_invnormtg * (_nplan.Dot(_d2gui)), _nplan);
                        _tevol.D1(t, out _ray, out _dray);
                    }
                    break;
                case 2:
                    {
                        _tcurv.D3(t, out _ptgui, out _d1gui, out _d2gui, out _d3gui);
                        _nplan = _d1gui.Normalized();
                        _invnormtg = ((double)1) / _d1gui.Magnitude;
                        _dnplan = Vector3D.LinearForm(_invnormtg, _d2gui, -_invnormtg * (_nplan.Dot(_d2gui)), _nplan);
                        _dinvnormtg = -_nplan.Dot(_d2gui) * _invnormtg * _invnormtg;
                        _d2nplan = Vector3D.LinearForm(_invnormtg, _d3gui, _dinvnormtg, _d2gui);
                        aux = _dinvnormtg * (_nplan.Dot(_d2gui)) + _invnormtg * (_dnplan.Dot(_d2gui) + _nplan.Dot(_d3gui));
                        _d2nplan = Vector3D.LinearForm(-_invnormtg * (_nplan.Dot(_d2gui)), _dnplan, -aux, _nplan, _d2nplan);
                        _tevol.D2(t, out _ray, out _dray, out _d2ray);
                        break;
                    }
                default:
                    return false;
            }
        }

        // Processing of X
        if (!myX_OK)
        {
            _xval.Initialized(x);
            _xOrder = order;
            //-------------- Positioning on surfaces -----------------
            switch (_xOrder)
            {
                case 0:
                    {
                        _surf1.D1(x[1], x[2], out _pts1, out _d1u1, out _d1v1);
                        _nsurf1 = _d1u1.Crossed(_d1v1);
                        _surf2.D1(x[3], x[4], out _pts2, out _d1u2, out _d1v2);
                        _nsurf2 = _d1u2.Crossed(_d1v2);
                        break;
                    }
                case 1:
                    {
                        _surf1.D2(x[1], x[2], out _pts1, out _d1u1, out _d1v1, out _d2u1, out _d2v1, out _d2uv1);
                        _nsurf1 = _d1u1.Crossed(_d1v1);
                        _dns1u1 = _d2u1.Crossed(_d1v1).Added(_d1u1.Crossed(_d2uv1));
                        _dns1v1 = _d2uv1.Crossed(_d1v1).Added(_d1u1.Crossed(_d2v1));
                        _surf2.D2(x[3], x[4], out _pts2, out _d1u2, out _d1v2, out _d2u2, out _d2v2, out _d2uv2);
                        _nsurf2 = _d1u2.Crossed(_d1v2);
                        _dns1u2 = _d2u2.Crossed(_d1v2).Added(_d1u2.Crossed(_d2uv2));
                        _dns1v2 = _d2uv2.Crossed(_d1v2).Added(_d1u2.Crossed(_d2v2));
                        break;
                    }
                case 2:
                    {
                        // Kerf (decision 2215): the derivatives of the normals at this point, as BlendFunc_ConstRad's
                        // order 2; OCCT kept those of the last order-1 positioning here (decision 1039), which the unit
                        // normals' second derivatives can't take.
                        _surf1.D3(x[1], x[2], out _pts1, out _d1u1, out _d1v1, out _d2u1, out _d2v1, out _d2uv1, out _d3u1,
                                 out _d3v1, out _d3uuv1, out _d3uvv1);
                        _nsurf1 = _d1u1.Crossed(_d1v1);
                        _dns1u1 = _d2u1.Crossed(_d1v1).Added(_d1u1.Crossed(_d2uv1));
                        _dns1v1 = _d2uv1.Crossed(_d1v1).Added(_d1u1.Crossed(_d2v1));
                        _surf2.D3(x[3], x[4], out _pts2, out _d1u2, out _d1v2, out _d2u2, out _d2v2, out _d2uv2, out _d3u2,
                                 out _d3v2, out _d3uuv2, out _d3uvv2);
                        _nsurf2 = _d1u2.Crossed(_d1v2);
                        _dns1u2 = _d2u2.Crossed(_d1v2).Added(_d1u2.Crossed(_d2uv2));
                        _dns1v2 = _d2uv2.Crossed(_d1v2).Added(_d1u2.Crossed(_d2v2));
                        break;
                    }
                default:
                    return false;
            }

            // Case of degenerated surfaces
            if (_nsurf1.Magnitude < Eps)
            {
                Point2D p = new Point2D(x[1], x[2]);
                if (order == 0)
                {
                    global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(_surf1, p, ref _nsurf1);
                }
                else
                {
                    global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeDNormal(_surf1, p, ref _nsurf1, ref _dns1u1, ref _dns1v1);
                }
            }

            if (_nsurf2.Magnitude < Eps)
            {
                Point2D p = new Point2D(x[3], x[4]);
                if (order == 0)
                {
                    global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeNormal(_surf2, p, ref _nsurf2);
                }
                else
                {
                    global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.ComputeDNormal(_surf2, p, ref _nsurf2, ref _dns1u2, ref _dns1v2);
                }
            }
        }

        // -------------------- Positioning of order 0 ---------------------
        // Kerf (decision 2215): as BlendFunc_ConstRad's (decision 2156), the centre is at the radius from each contact
        // along that surface's own unit normal, so each section is the rolling ball's, of the radius the law gives where
        // the section crosses the guide. OCCT projects the normals into the plane square to the guide. The guide's plane
        // only places the section, through the contacts' midpoint; the normals don't depend on the guide, the radius does.
        double invnorm1, invnorm2, theD;
        double ray1 = _sg1 * _ray;
        double ray2 = _sg2 * _ray;
        Vector3D nn1, nn2, resul, n1, n2;

        theD = -(_nplan.XYZ().Dot(_ptgui.XYZ()));

        _e[1] = (_nplan.X * (_pts1.X + _pts2.X) + _nplan.Y * (_pts1.Y + _pts2.Y)
                + _nplan.Z * (_pts1.Z + _pts2.Z))
                 / 2
               + theD;

        invnorm1 = ConstRad.InverseNorm(_nsurf1);
        invnorm2 = ConstRad.InverseNorm(_nsurf2);
        nn1 = invnorm1 * _nsurf1;
        nn2 = invnorm2 * _nsurf2;
        // OCCT's n1 and n2 are the projected normals reversed: here the unit normals reversed.
        n1 = -nn1;
        n2 = -nn2;

        // The centre is pts1 + ray1 * n1 = pts2 + ray2 * n2.
        resul = Vector3D.LinearForm(ray1, n1, -ray2, n2, new Vector3D(_pts2, _pts1));

        _e[2] = resul.X;
        _e[3] = resul.Y;
        _e[4] = resul.Z;

        // -------------------- Positioning of order 1 ---------------------
        if (order >= 1)
        {
            _dedx[1, 1] = _nplan.Dot(_d1u1) / 2;
            _dedx[1, 2] = _nplan.Dot(_d1v1) / 2;
            _dedx[1, 3] = _nplan.Dot(_d1u2) / 2;
            _dedx[1, 4] = _nplan.Dot(_d1v2) / 2;

            // The derivatives of n1 and n2, the reversed unit normals, as OCCT's projected ones were.
            _dndu1 = -ConstRad.UnitNormalD1(nn1, invnorm1, _dns1u1);
            resul = Vector3D.LinearForm(ray1, _dndu1, _d1u1);
            _dedx[2, 1] = resul.X;
            _dedx[3, 1] = resul.Y;
            _dedx[4, 1] = resul.Z;

            _dndv1 = -ConstRad.UnitNormalD1(nn1, invnorm1, _dns1v1);
            resul = Vector3D.LinearForm(ray1, _dndv1, _d1v1);
            _dedx[2, 2] = resul.X;
            _dedx[3, 2] = resul.Y;
            _dedx[4, 2] = resul.Z;

            _dndu2 = -ConstRad.UnitNormalD1(nn2, invnorm2, _dns1u2);
            resul = Vector3D.LinearForm(-ray2, _dndu2, -1, _d1u2);
            _dedx[2, 3] = resul.X;
            _dedx[3, 3] = resul.Y;
            _dedx[4, 3] = resul.Z;

            _dndv2 = -ConstRad.UnitNormalD1(nn2, invnorm2, _dns1v2);
            resul = Vector3D.LinearForm(-ray2, _dndv2, -1, _d1v2);
            _dedx[2, 4] = resul.X;
            _dedx[3, 4] = resul.Y;
            _dedx[4, 4] = resul.Z;

            if (byParam)
            {
                // The normals don't move with the guide: the plane's equation and the radius do.
                Vector3D temp = new Vector3D((_pts1.XYZ() + _pts2.XYZ()) / 2 - _ptgui.XYZ());
                _dn1w = new Vector3D();
                _dn2w = new Vector3D();

                _dedt[1] = _dnplan.Dot(temp) - 1.0 / _invnormtg;
                resul = Vector3D.LinearForm(_sg1 * _dray, n1, -_sg2 * _dray, n2);
                _dedt[2] = resul.X;
                _dedt[3] = resul.Y;
                _dedt[4] = resul.Z;
            }

            // ------   Positioning of order 2  -----------------------------
            if (order == 2)
            {
                Vector3D d2ns1u1, d2ns1u2, d2ns1v1, d2ns1v2, d2ns1uv1, d2ns1uv2;
                _d2EDX2.Init(0);

                _d2EDX2[1, 1, 1] = _nplan.Dot(_d2u1) / 2;
                _d2EDX2[1, 2, 1] = _d2EDX2[1, 1, 2] = _nplan.Dot(_d2uv1) / 2;
                _d2EDX2[1, 2, 2] = _nplan.Dot(_d2v1) / 2;

                _d2EDX2[1, 3, 3] = _nplan.Dot(_d2u2) / 2;
                _d2EDX2[1, 4, 3] = _d2EDX2[1, 3, 4] = _nplan.Dot(_d2uv2) / 2;
                _d2EDX2[1, 4, 4] = _nplan.Dot(_d2v2) / 2;
                // ================
                // ==  Surface 1 ==
                // ================
                // The second derivatives of the surface's normal d1u ^ d1v
                d2ns1u1 = Vector3D.LinearForm(1, _d3u1.Crossed(_d1v1), 2, _d2u1.Crossed(_d2uv1), 1, _d1u1.Crossed(_d3uuv1));
                d2ns1uv1 = (_d3uuv1.Crossed(_d1v1)) + (_d2u1.Crossed(_d2v1)) + (_d1u1.Crossed(_d3uvv1));
                d2ns1v1 = Vector3D.LinearForm(1, _d1u1.Crossed(_d3v1), 2, _d2uv1.Crossed(_d2v1), 1, _d3uvv1.Crossed(_d1v1));

                _d2ndu1 = -ConstRad.UnitNormalD2(nn1, invnorm1, _dns1u1, _dns1u1, d2ns1u1);
                resul = Vector3D.LinearForm(ray1, _d2ndu1, _d2u1);
                _d2EDX2[2, 1, 1] = resul.X;
                _d2EDX2[3, 1, 1] = resul.Y;
                _d2EDX2[4, 1, 1] = resul.Z;

                _d2nduv1 = -ConstRad.UnitNormalD2(nn1, invnorm1, _dns1u1, _dns1v1, d2ns1uv1);
                resul = Vector3D.LinearForm(ray1, _d2nduv1, _d2uv1);
                _d2EDX2[2, 2, 1] = _d2EDX2[2, 1, 2] = resul.X;
                _d2EDX2[3, 2, 1] = _d2EDX2[3, 1, 2] = resul.Y;
                _d2EDX2[4, 2, 1] = _d2EDX2[4, 1, 2] = resul.Z;

                _d2ndv1 = -ConstRad.UnitNormalD2(nn1, invnorm1, _dns1v1, _dns1v1, d2ns1v1);
                resul = Vector3D.LinearForm(ray1, _d2ndv1, _d2v1);
                _d2EDX2[2, 2, 2] = resul.X;
                _d2EDX2[3, 2, 2] = resul.Y;
                _d2EDX2[4, 2, 2] = resul.Z;
                // ================
                // ==  Surface 2 ==
                // ================
                d2ns1u2 = Vector3D.LinearForm(1, _d3u2.Crossed(_d1v2), 2, _d2u2.Crossed(_d2uv2), 1, _d1u2.Crossed(_d3uuv2));
                d2ns1uv2 = (_d3uuv2.Crossed(_d1v2)) + (_d2u2.Crossed(_d2v2)) + (_d1u2.Crossed(_d3uvv2));
                d2ns1v2 = Vector3D.LinearForm(1, _d1u2.Crossed(_d3v2), 2, _d2uv2.Crossed(_d2v2), 1, _d3uvv2.Crossed(_d1v2));

                _d2ndu2 = -ConstRad.UnitNormalD2(nn2, invnorm2, _dns1u2, _dns1u2, d2ns1u2);
                resul = Vector3D.LinearForm(-ray2, _d2ndu2, -1, _d2u2);
                _d2EDX2[2, 3, 3] = resul.X;
                _d2EDX2[3, 3, 3] = resul.Y;
                _d2EDX2[4, 3, 3] = resul.Z;

                _d2nduv2 = -ConstRad.UnitNormalD2(nn2, invnorm2, _dns1u2, _dns1v2, d2ns1uv2);
                resul = Vector3D.LinearForm(-ray2, _d2nduv2, -1, _d2uv2);
                _d2EDX2[2, 4, 3] = _d2EDX2[2, 3, 4] = resul.X;
                _d2EDX2[3, 4, 3] = _d2EDX2[3, 3, 4] = resul.Y;
                _d2EDX2[4, 4, 3] = _d2EDX2[4, 3, 4] = resul.Z;

                _d2ndv2 = -ConstRad.UnitNormalD2(nn2, invnorm2, _dns1v2, _dns1v2, d2ns1v2);
                resul = Vector3D.LinearForm(-ray2, _d2ndv2, -1, _d2v2);
                _d2EDX2[2, 4, 4] = resul.X;
                _d2EDX2[3, 4, 4] = resul.Y;
                _d2EDX2[4, 4, 4] = resul.Z;

                if (byParam)
                {
                    //  ---------- Derivation double in t, X --------------------------
                    // The normals don't move with the guide; the radius does.
                    _d2EDXDT.Init(0);
                    _d2EDXDT[1, 1] = _dnplan.Dot(_d1u1) / 2;
                    _d2EDXDT[1, 2] = _dnplan.Dot(_d1v1) / 2;
                    _d2EDXDT[1, 3] = _dnplan.Dot(_d1u2) / 2;
                    _d2EDXDT[1, 4] = _dnplan.Dot(_d1v2) / 2;
                    _d2ndtu1 = new Vector3D();
                    _d2ndtv1 = new Vector3D();
                    _d2ndtu2 = new Vector3D();
                    _d2ndtv2 = new Vector3D();

                    resul = _sg1 * _dray * _dndu1;
                    _d2EDXDT[2, 1] = resul.X;
                    _d2EDXDT[3, 1] = resul.Y;
                    _d2EDXDT[4, 1] = resul.Z;

                    resul = _sg1 * _dray * _dndv1;
                    _d2EDXDT[2, 2] = resul.X;
                    _d2EDXDT[3, 2] = resul.Y;
                    _d2EDXDT[4, 2] = resul.Z;

                    resul = -_sg2 * _dray * _dndu2;
                    _d2EDXDT[2, 3] = resul.X;
                    _d2EDXDT[3, 3] = resul.Y;
                    _d2EDXDT[4, 3] = resul.Z;

                    resul = -_sg2 * _dray * _dndv2;
                    _d2EDXDT[2, 4] = resul.X;
                    _d2EDXDT[3, 4] = resul.Y;
                    _d2EDXDT[4, 4] = resul.Z;

                    //  ---------- Derivation double in t -----------------------------
                    _d2n1w = new Vector3D();
                    _d2n2w = new Vector3D();

                    Vector3D temp = new Vector3D((_pts1.XYZ() + _pts2.XYZ()) / 2 - _ptgui.XYZ());
                    _d2EDT2[1] = _d2nplan.Dot(temp) - 2 * _dnplan.Dot(_d1gui) - _nplan.Dot(_d2gui);
                    resul = Vector3D.LinearForm(_sg1 * _d2ray, n1, -_sg2 * _d2ray, n2);
                    _d2EDT2[2] = resul.X;
                    _d2EDT2[3] = resul.Y;
                    _d2EDT2[4] = resul.Z;
                }
            }
        }

        return true;
    }

    public override void Set(double param)
    {
        _param = param;
    }

    //=======================================================================
    // function : Set
    // purpose  : Segments curve in its useful part.
    //           Small precision is taken at random
    //=======================================================================
    public override void Set(double first, double last)
    {
        _tcurv = _curv.Trim(first, last, 1.0e-12);
        _tevol = _fevol.Trim(first, last, 1.0e-12);
    }

    public override void GetTolerance(math_Vector tolerance, double tol)
    {
        tolerance[1] = _surf1.UResolution(tol);
        tolerance[2] = _surf1.VResolution(tol);
        tolerance[3] = _surf2.UResolution(tol);
        tolerance[4] = _surf2.VResolution(tol);
    }

    public override void GetBounds(math_Vector infBound, math_Vector supBound)
    {
        infBound[1] = _surf1.FirstUParameter();
        infBound[2] = _surf1.FirstVParameter();
        infBound[3] = _surf2.FirstUParameter();
        infBound[4] = _surf2.FirstVParameter();
        supBound[1] = _surf1.LastUParameter();
        supBound[2] = _surf1.LastVParameter();
        supBound[3] = _surf2.LastUParameter();
        supBound[4] = _surf2.LastVParameter();

        for (int i = 1; i <= 4; i++)
        {
            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(infBound[i]) && !global::Kerf.Engine.Foundation.Precision.IsInfinite(supBound[i]))
            {
                double range = (supBound[i] - infBound[i]);
                infBound[i] -= range;
                supBound[i] += range;
            }
        }
    }

    public override bool IsSolution(math_Vector sol, double tol)
    {
        double cosa, sina, angle;
        bool ok = true;

        ok = ComputeValues(sol, 1, true, _param);

        if (Math.Abs(_e[1]) <= tol && _e[2] * _e[2] + _e[3] * _e[3] + _e[4] * _e[4] <= tol * tol)
        {
            // ns1, ns2, np are copied locally to avoid crushing the fields !
            Vector3D ns1, ns2, np;
            ns1 = _nsurf1;
            ns2 = _nsurf2;
            np = _nplan;

            // The surfaces' own normals, reversed (decision 2215).
            ns1 = -ConstRad.InverseNorm(ns1) * ns1;
            ns2 = -ConstRad.InverseNorm(ns2) * ns2;

            double maxpiv = 1.0e-14;
            Gauss resol = new(_dedx, maxpiv);
            _istangent = false;
            if (resol.IsDone)
            {
                math_Vector controle = new(1, 4), solution = new(1, 4), tolerances = new(1, 4);
                GetTolerance(tolerances, tol);
                resol.Solve(-_dedt, solution);
                controle.Initialized(_dedt.Added(_dedx.Multiplied(solution)));
                if (Math.Abs(controle[1]) > tolerances[1] || Math.Abs(controle[2]) > tolerances[2]
                    || Math.Abs(controle[3]) > tolerances[3] || Math.Abs(controle[4]) > tolerances[4])
                {
                    // OCCT_DEBUG trace left out ("Cheminement : echec calcul des derivees").
                    _istangent = true;
                }

                if (!_istangent)
                {
                    _tg1 = Vector3D.LinearForm(solution[1], _d1u1, solution[2], _d1v1);
                    _tg2 = Vector3D.LinearForm(solution[3], _d1u2, solution[4], _d1v2);
                    _tg12d = _tg12d.WithCoord(solution[1], solution[2]);
                    _tg22d = _tg22d.WithCoord(solution[3], solution[4]);
                }
            }
            else
            {
                _istangent = true;
            }

            // update of maxang
            if (_sg1 > 0.0)
            { // sg1*ray
                ns1 = ns1.Reversed();
            }

            if (_sg2 > 0.0)
            { // sg2*ray
                ns2 = ns2.Reversed();
            }

            cosa = ns1.Dot(ns2);
            sina = _nplan.Dot(ns1.Crossed(ns2));
            if (_choix % 2 != 0)
            {
                sina = -sina; // nplan is changed into -nplan
            }

            if (cosa > 1.0)
            {
                cosa = 1.0;
                sina = 0.0;
            }

            angle = Math.Acos(cosa);

            // Reframing on  ]-pi/2, 3pi/2]
            if (sina < 0.0)
            {
                if (cosa > 0.0)
                {
                    angle = -angle;
                }
                else
                {
                    angle = 2.0 * M_PI - angle;
                }
            }

            if (Math.Abs(angle) > _maxang)
            {
                _maxang = Math.Abs(angle);
            }

            if (Math.Abs(angle) < _minang)
            {
                _minang = Math.Abs(angle);
            }

            if (Math.Abs(angle * _ray) < _lengthmin)
            {
                _lengthmin = Math.Abs(angle * _ray);
            }

            if (Math.Abs(angle * _ray) > _lengthmax)
            {
                _lengthmax = Math.Abs(angle * _ray);
            }

            _distmin = Min(_distmin, _pts1.DistanceTo(_pts2));

            return ok;
        }

        _istangent = true;
        return false;
    }

    /// <summary>The minimal distance between the two extremities of the computed sections.</summary>
    public override double GetMinimalDistance()
    {
        return _distmin;
    }

    public override bool Value(math_Vector x, math_Vector f)
    {
        bool ok = ComputeValues(x, 0);
        f.Initialized(_e);
        return ok;
    }

    public override bool Derivatives(math_Vector x, Matrix d)
    {
        bool ok = ComputeValues(x, 1);
        d.Initialized(_dedx);
        return ok;
    }

    public override bool Values(math_Vector x, math_Vector f, Matrix d)
    {
        bool ok = ComputeValues(x, 1);
        f.Initialized(_e);
        d.Initialized(_dedx);
        return ok;
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
            throw new DomainErrorException();
        }

        return _tg1;
    }

    public override Vector3D TangentOnS2()
    {
        if (_istangent)
        {
            throw new DomainErrorException();
        }

        return _tg2;
    }

    public override Vector2D Tangent2dOnS1()
    {
        if (_istangent)
        {
            throw new DomainErrorException();
        }

        return _tg12d;
    }

    public override Vector2D Tangent2dOnS2()
    {
        if (_istangent)
        {
            throw new DomainErrorException();
        }

        return _tg22d;
    }

    public override void Tangent(double u1, double v1, double u2, double v2, out Vector3D tgF, out Vector3D tgL,
                                 out Vector3D nmF, out Vector3D nmL)
    {
        Point3D center = new Point3D();
        Vector3D ns1;
        double invnorm1;

        if ((u1 != _xval[1]) || (v1 != _xval[2]) || (u2 != _xval[3]) || (v2 != _xval[4]))
        {
            _surf1.D1(u1, v1, out _, out Vector3D d1u, out Vector3D d1v);
            nmF = ns1 = d1u.Crossed(d1v);
            _surf2.D1(u2, v2, out _, out d1u, out d1v);
            nmL = d1u.Crossed(d1v);
        }
        else
        {
            nmF = ns1 = _nsurf1;
            nmL = _nsurf2;
        }

        // The ball's circle: the surface's own normal, and the plane square to both normals (decision 2215).
        invnorm1 = ConstRad.InverseNorm(ns1);
        ns1 = -invnorm1 * ns1;
        center = center.WithXYZ(_pts1.XYZ() + _sg1 * _ray * ns1.XYZ());
        Vector3D np = ConstRad.SectionNormal(nmF, nmL, _nplan);

        tgF = np.Crossed(new Vector3D(center, _pts1));
        tgL = np.Crossed(new Vector3D(center, _pts2));
        if (_choix % 2 == 1)
        {
            tgF = tgF.Reversed();
            tgL = tgL.Reversed();
        }
    }

    public override bool TwistOnS1()
    {
        if (_istangent)
        {
            throw new DomainErrorException();
        }

        return _tg1.Dot(_nplan) < 0.0;
    }

    public override bool TwistOnS2()
    {
        if (_istangent)
        {
            throw new DomainErrorException();
        }

        return _tg2.Dot(_nplan) < 0.0;
    }

    /// <summary>
    /// The circular section at <paramref name="param"/> through the points (U1, V1) and (U2, V2), with its parameter
    /// range: for a quick, approximate view of the surface.
    /// </summary>
    public void Section(double param, double u1, double v1, double u2, double v2, out double pdeb, out double pfin,
                        ref Circle3D c)
    {
        Point3D center = new Point3D();
        Vector3D ns1, np;

        math_Vector x = new(1, 4);
        x[1] = u1;
        x[2] = v1;
        x[3] = u2;
        x[4] = v2;
        double prm = param;

        ComputeValues(x, 0, true, prm);

        // The ball's circle: the surface's own normal, and the plane square to both normals (decision 2215).
        ns1 = -ConstRad.InverseNorm(_nsurf1) * _nsurf1;
        np = ConstRad.SectionNormal(_nsurf1, _nsurf2, _nplan);
        center = center.WithXYZ(_pts1.XYZ() + _sg1 * _ray * ns1.XYZ());

        // ns1 is oriented from the center to pts1

        if (_sg1 > 0.0)
        {
            ns1 = ns1.Reversed();
        }

        if (_choix % 2 != 0)
        {
            np = np.Reversed();
        }

        c = c.WithRadius(Math.Abs(_ray));
        c = c.WithPosition(new Frame3D(center, new Direction3D(np), new Direction3D(ns1)));
        pdeb = 0.0;
        pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, _pts2);
        // Test negative and almost null angles : Singular Case
        if (pfin > 1.5 * M_PI)
        {
            np = np.Reversed();
            c = c.WithPosition(new Frame3D(center, new Direction3D(np), new Direction3D(ns1)));
            pfin = global::Kerf.Engine.Geometry.ElCLib.Parameter(c, _pts2);
        }

        if (pfin < global::Kerf.Engine.Foundation.Precision.PConfusion())
        {
            pfin += global::Kerf.Engine.Foundation.Precision.PConfusion();
        }
    }

    public override bool IsRational
    {
        get
        {
            return (_sShape == SectionShape.Rational || _sShape == SectionShape.QuasiAngular);
        }
    }

    public override double GetSectionSize()
    {
        return _lengthmax;
    }

    public override void GetMinimalWeight(NCollection_Array1<double> weigths)
    {
        global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.GetMinimalWeights(_sShape, _tConv, _minang, _maxang, weigths);
    }

    public override int NbIntervals(Continuity s)
    {
        int Nb_Int_Courbe, Nb_Int_Loi;
        Nb_Int_Courbe = _curv.NbIntervals(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
        Nb_Int_Loi = _fevol.NbIntervals(s);

        if (Nb_Int_Loi == 1)
        {
            return Nb_Int_Courbe;
        }

        NCollection_Array1<double> intC = new(1, Nb_Int_Courbe + 1);
        NCollection_Array1<double> intL = new(1, Nb_Int_Loi + 1);
        NCollection_Sequence<double> inter = new();
        _curv.Intervals(intC, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
        _fevol.Intervals(intL, s);

        FusionneIntervalles(intC, intL, inter);
        return inter.Length() - 1;
    }

    public override void Intervals(NCollection_Array1<double> t, Continuity s)
    {
        int Nb_Int_Courbe, Nb_Int_Loi;
        Nb_Int_Courbe = _curv.NbIntervals(global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
        Nb_Int_Loi = _fevol.NbIntervals(s);

        if (Nb_Int_Loi == 1)
        {
            _curv.Intervals(t, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
        }
        else
        {
            NCollection_Array1<double> intC = new(1, Nb_Int_Courbe + 1);
            NCollection_Array1<double> intL = new(1, Nb_Int_Loi + 1);
            NCollection_Sequence<double> inter = new();
            _curv.Intervals(intC, global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.NextShape(s));
            _fevol.Intervals(intL, s);

            FusionneIntervalles(intC, intL, inter);
            for (int ii = 1; ii <= inter.Length(); ii++)
            {
                t[ii] = inter[ii];
            }
        }
    }

    private static void FusionneIntervalles(NCollection_Array1<double> i1, NCollection_Array1<double> i2,
                                            NCollection_Sequence<double> seq)
    {
        int ind1 = 1, ind2 = 1;
        double epspar = global::Kerf.Engine.Foundation.Precision.PConfusion() * 0.99;
        // supposed that positioning works with PConfusion()/2
        double v1, v2;
        // Initialisation : IND1 and IND2 point at the 1st element
        // of each of 2 tables to be processed. INDS points at the last
        // element of TABSOR

        //--- TABSOR is filled by parsing TABLE1 and TABLE2 simultaneously ---
        //------------------ by removing multiple occurrences ------------

        while ((ind1 <= i1.Upper()) && (ind2 <= i2.Upper()))
        {
            v1 = i1[ind1];
            v2 = i2[ind2];
            if (Math.Abs(v1 - v2) <= epspar)
            {
                // Here elements of I1 and I2 are suitable.
                seq.Append((v1 + v2) / 2);
                ind1++;
                ind2++;
            }
            else if (v1 < v2)
            {
                // Here the element of I1 is suitable.
                seq.Append(v1);
                ind1++;
            }
            else
            {
                // Here the element of TABLE2 is suitable.
                seq.Append(v2);
                ind2++;
            }
        }

        if (ind1 > i1.Upper())
        {
            //----- Here I1 is empty, to be completed with the end of TABLE2 -------
            for (; ind2 <= i2.Upper(); ind2++)
            {
                seq.Append(i2[ind2]);
            }
        }

        if (ind2 > i2.Upper())
        {
            //----- Here I2 is empty, to be completed with the end of I1 -------
            for (; ind1 <= i1.Upper(); ind1++)
            {
                seq.Append(i1[ind1]);
            }
        }
    }

    public override void GetShape(out int nbPoles, out int nbKnots, out int degree, out int nbPoles2d)
    {
        nbPoles2d = 2;
        global::Kerf.Engine.Fillets.BlendFunctions.BlendFunc.GetShape(_sShape, _maxang, out nbPoles, out nbKnots, out degree, ref _tConv);
    }

    //=======================================================================
    // function : GetTolerance
    // purpose  : Determine the Tolerance to be used in approximations.
    //=======================================================================
    public override void GetTolerance(double boundTol, double surfTol, double angleTol, math_Vector tol3d,
                                      math_Vector tol1d)
    {
        int low = tol3d.Lower(), up = tol3d.Upper();
        double rayon = _lengthmin / _maxang; // a radius is subtracted
        double tol;
        tol = global::Kerf.Engine.Construction.Surfaces.GeomFill.GetTolerance(_tConv, _maxang, rayon, angleTol, surfTol);
        tol1d.Init(surfTol);
        tol3d.Init(surfTol);
        tol3d[low + 1] = tol3d[up - 1] = Min(tol, surfTol);
        tol3d[low] = tol3d[up] = Min(tol, boundTol);
    }

    public override void Knots(NCollection_Array1<double> tKnots)
    {
        global::Kerf.Engine.Construction.Surfaces.GeomFill.Knots(_tConv, tKnots);
    }

    public override void Mults(NCollection_Array1<int> tMults)
    {
        global::Kerf.Engine.Construction.Surfaces.GeomFill.Mults(_tConv, tMults);
    }

    public override void Section(in BlendPoint p, NCollection_Array1<Point3D> poles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<double> weights)
    {
        Point3D center = new Point3D();
        Vector3D ns1, ns2, np;

        math_Vector x = new(1, 4);
        double prm = p.Parameter();

        int low = poles.Lower();
        int upp = poles.Upper();

        p.ParametersOnS1(out x[1], out x[2]);
        p.ParametersOnS2(out x[3], out x[4]);

        // Calculation and storage of distmin
        ComputeValues(x, 0, true, prm);
        _distmin = Min(_distmin, _pts1.DistanceTo(_pts2));

        // ns1, ns2, np are copied locally to avoid crashing the fields !
        ns1 = _nsurf1;
        ns2 = _nsurf2;
        np = _nplan;

        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(x[1], x[2]);
        poles2d[poles2d.Upper()] = poles2d[poles2d.Upper()].WithCoord(x[3], x[4]);

        if (_sShape == SectionShape.Linear)
        {
            poles[low] = _pts1;
            poles[upp] = _pts2;
            weights[low] = 1.0;
            weights[upp] = 1.0;
            return;
        }

        // The ball's circle: the surfaces' own normals, and the plane square to both (decision 2215).
        np = ConstRad.SectionNormal(ns1, ns2, _nplan);
        ns1 = -ConstRad.InverseNorm(ns1) * ns1;
        ns2 = -ConstRad.InverseNorm(ns2) * ns2;

        center = center.WithXYZ(_pts1.XYZ() + _sg1 * _ray * ns1.XYZ());

        // ns1 (resp. ns2) is oriented from center to pts1 (resp. pts2),
        // and the trihedron ns1,ns2,nplan is made direct.

        if (_sg1 > 0.0)
        {
            ns1 = ns1.Reversed();
        }

        if (_sg2 > 0.0)
        {
            ns2 = ns2.Reversed();
        }

        if (_choix % 2 != 0)
        {
            np = np.Reversed();
        }

        global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns1, ns2, np, _pts1, _pts2, Math.Abs(_ray), center, poles, weights);
    }

    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                                 NCollection_Array1<double> weights, NCollection_Array1<double> dWeights)
    {
        Vector3D ns1, ns2, np, dnp, dnorm1w = new Vector3D(), dnorm2w = new Vector3D(), tgc = new Vector3D();
        double rayprim;

        Point3D center = new Point3D();
        math_Vector sol = new(1, 4), secmember = new(1, 4);

        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();
        bool istgt = true;

        p.ParametersOnS1(out sol[1], out sol[2]);
        p.ParametersOnS2(out sol[3], out sol[4]);

        // Calculation of equations
        ComputeValues(sol, 1, true, prm);
        _distmin = Min(_distmin, _pts1.DistanceTo(_pts2));

        // ns1, ns2, np are copied locally to avoid crushing the fields !
        ns1 = _nsurf1;
        ns2 = _nsurf2;
        np = _nplan;
        dnp = _dnplan;
        rayprim = _dray;

        if (!_pts1.IsEqual(_pts2, 1.0e-4))
        {
            // Calculation of derived  Normal processing
            Gauss resol = new(_dedx, 1.0e-9);

            if (resol.IsDone)
            {
                resol.Solve(-_dedt, secmember);
                istgt = false;
            }
        }

        if (istgt)
        {
            Svd singRS = new(_dedx);
            if (singRS.IsDone)
            {
                singRS.Solve(-_dedt, secmember, 1.0e-6);
                istgt = false;
            }
        }

        if (!istgt)
        {
            _tg1 = Vector3D.LinearForm(secmember[1], _d1u1, secmember[2], _d1v1);
            _tg2 = Vector3D.LinearForm(secmember[3], _d1u2, secmember[4], _d1v2);

            dnorm1w = Vector3D.LinearForm(secmember[1], _dndu1, secmember[2], _dndv1, _dn1w);
            dnorm2w = Vector3D.LinearForm(secmember[3], _dndu2, secmember[4], _dndv2, _dn2w);
            istgt = false;
        }

        // Tops 2d
        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(sol[1], sol[2]);
        poles2d[poles2d.Upper()] = poles2d[poles2d.Upper()].WithCoord(sol[3], sol[4]);
        if (!istgt)
        {
            dPoles2d[poles2d.Lower()] = dPoles2d[poles2d.Lower()].WithCoord(secmember[1], secmember[2]);
            dPoles2d[poles2d.Upper()] = dPoles2d[poles2d.Upper()].WithCoord(secmember[3], secmember[4]);
        }

        // the linear case is processed...
        if (_sShape == SectionShape.Linear)
        {
            poles[low] = _pts1;
            poles[upp] = _pts2;
            weights[low] = 1.0;
            weights[upp] = 1.0;
            if (!istgt)
            {
                dPoles[low] = _tg1;
                dPoles[upp] = _tg2;
                dWeights[low] = 0.0;
                dWeights[upp] = 0.0;
            }

            return (!istgt);
        }

        // Case of the circle
        // The surfaces' own normals (decision 2215); the plane, square to both, is set below.
        ns1 = -ConstRad.InverseNorm(ns1) * ns1;
        ns2 = -ConstRad.InverseNorm(ns2) * ns2;

        center = center.WithXYZ(_pts1.XYZ() + _sg1 * _ray * ns1.XYZ());
        if (!istgt)
        {
            tgc = Vector3D.LinearForm(_sg1 * _ray, dnorm1w, _sg1 * _dray, ns1, _tg1);
        }

        // ns1 is oriented from center to pts1, and  ns2 from center to pts2
        // and the trihedron ns1,ns2,nplan is made direct

        if (_sg1 > 0.0)
        {
            ns1 = ns1.Reversed();
            if (!istgt)
            {
                dnorm1w = dnorm1w.Reversed();
            }
        }

        if (_sg2 > 0.0)
        {
            ns2 = ns2.Reversed();
            if (!istgt)
            {
                dnorm2w = dnorm2w.Reversed();
            }
        }

        if (_choix % 2 != 0)
        {
            np = np.Reversed();
            dnp = dnp.Reversed();
        }

        // The plane of the ball's circle, square to both normals, on the guide plane's side.
        Vector3D zero = new Vector3D();
        ConstRad.SectionNormal(ns1, ns2, dnorm1w, dnorm2w, zero, zero, np, dnp, zero, out Vector3D npBall, out Vector3D dnpBall, out _);
        np = npBall;
        dnp = dnpBall;

        if (_ray < 0.0)
        { // to avoid std::abs(dray) some lines below
            rayprim = -rayprim;
        }

        if (!istgt)
        {
            return global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns1, ns2, dnorm1w, dnorm2w, np, dnp, _pts1, _pts2, _tg1, _tg2,
                                      Math.Abs(_ray), rayprim, center, tgc, poles, dPoles, weights, dWeights);
        }
        else
        {
            global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns1, ns2, np, _pts1, _pts2, Math.Abs(_ray), center, poles, weights);
            return false;
        }
    }

    public override bool Section(in BlendPoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                                 NCollection_Array1<Vector3D> d2Poles, NCollection_Array1<Point2D> poles2d,
                                 NCollection_Array1<Vector2D> dPoles2d, NCollection_Array1<Vector2D> d2Poles2d,
                                 NCollection_Array1<double> weights, NCollection_Array1<double> dWeights,
                                 NCollection_Array1<double> d2Weights)
    {
        Vector3D ns1, ns2, np, dnp, d2np, dnorm1w = new Vector3D(), dnorm2w = new Vector3D(), d2norm1w = new Vector3D(),
               d2norm2w = new Vector3D();
        Vector3D tgc = new Vector3D(), dtgc = new Vector3D(), dtg1 = new Vector3D(), dtg2 = new Vector3D(),
               temp = new Vector3D(), tempbis = new Vector3D();
        double rayprim, raysecn;

        Point3D center = new Point3D();
        math_Vector x = new(1, 4), sol = new(1, 4), secmember = new(1, 4);
        Matrix d2DXdSdt = new(1, 4, 1, 4);

        double prm = p.Parameter();
        int low = poles.Lower();
        int upp = poles.Upper();
        bool istgt = true;

        p.ParametersOnS1(out x[1], out x[2]);
        p.ParametersOnS2(out x[3], out x[4]);

        // A commented-out OCCT_DEBUG check of the derivatives by finite differences is left out.

        // Calculation of equations
        ComputeValues(x, 2, true, prm);
        _distmin = Min(_distmin, _pts1.DistanceTo(_pts2));

        // ns1, ns2, np are copied locally to avois crushing the fields !
        ns1 = _nsurf1;
        ns2 = _nsurf2;
        np = _nplan;
        dnp = _dnplan;
        d2np = _d2nplan;
        rayprim = _dray;
        raysecn = _d2ray;

        // Calculation of derivatives

        if (!_pts1.IsEqual(_pts2, 1.0e-4))
        {
            Gauss resol = new(_dedx, 1.0e-9); // Precise tolerance !!!!!
            // Calculation of derivatives Processing Normal
            if (resol.IsDone)
            {
                resol.Solve(-_dedt, sol);
                _d2EDX2.Multiply(sol, d2DXdSdt);
                secmember.Initialized(-(_d2EDT2 + (2 * _d2EDXDT + d2DXdSdt) * sol));
                resol.Solve(secmember);
                istgt = false;
            }
        }

        if (istgt)
        {
            Svd singRS = new(_dedx);
            math_Vector vbis = new(1, 4);
            if (singRS.IsDone)
            {
                singRS.Solve(-_dedt, sol, 1.0e-6);
                _d2EDX2.Multiply(sol, d2DXdSdt);
                vbis.Initialized(-(_d2EDT2 + (2 * _d2EDXDT + d2DXdSdt) * sol));
                singRS.Solve(vbis, secmember, 1.0e-6);
                istgt = false;
            }
        }

        if (!istgt)
        {
            _tg1 = Vector3D.LinearForm(sol[1], _d1u1, sol[2], _d1v1);
            _tg2 = Vector3D.LinearForm(sol[3], _d1u2, sol[4], _d1v2);

            dnorm1w = Vector3D.LinearForm(sol[1], _dndu1, sol[2], _dndv1, _dn1w);
            dnorm2w = Vector3D.LinearForm(sol[3], _dndu2, sol[4], _dndv2, _dn2w);
            temp = Vector3D.LinearForm(sol[1] * sol[1], _d2u1, 2 * sol[1] * sol[2], _d2uv1, sol[2] * sol[2], _d2v1);

            dtg1 = Vector3D.LinearForm(secmember[1], _d1u1, secmember[2], _d1v1, temp);

            temp = Vector3D.LinearForm(sol[3] * sol[3], _d2u2, 2 * sol[3] * sol[4], _d2uv2, sol[4] * sol[4], _d2v2);
            dtg2 = Vector3D.LinearForm(secmember[3], _d1u2, secmember[4], _d1v2, temp);

            temp = Vector3D.LinearForm(sol[1] * sol[1], _d2ndu1, 2 * sol[1] * sol[2], _d2nduv1, sol[2] * sol[2], _d2ndv1);

            tempbis = Vector3D.LinearForm(2 * sol[1], _d2ndtu1, 2 * sol[2], _d2ndtv1, _d2n1w);
            temp += tempbis;
            d2norm1w = Vector3D.LinearForm(secmember[1], _dndu1, secmember[2], _dndv1, temp);

            temp = Vector3D.LinearForm(sol[3] * sol[3], _d2ndu2, 2 * sol[3] * sol[4], _d2nduv2, sol[4] * sol[4], _d2ndv2);
            tempbis = Vector3D.LinearForm(2 * sol[3], _d2ndtu2, 2 * sol[4], _d2ndtv2, _d2n2w);
            temp += tempbis;
            d2norm2w = Vector3D.LinearForm(secmember[3], _dndu2, secmember[4], _dndv2, temp);
        }

        // Tops 2d
        poles2d[poles2d.Lower()] = poles2d[poles2d.Lower()].WithCoord(x[1], x[2]);
        poles2d[poles2d.Upper()] = poles2d[poles2d.Upper()].WithCoord(x[3], x[4]);
        if (!istgt)
        {
            dPoles2d[poles2d.Lower()] = dPoles2d[poles2d.Lower()].WithCoord(sol[1], sol[2]);
            dPoles2d[poles2d.Upper()] = dPoles2d[poles2d.Upper()].WithCoord(sol[3], sol[4]);
            d2Poles2d[poles2d.Lower()] = d2Poles2d[poles2d.Lower()].WithCoord(secmember[1], secmember[2]);
            d2Poles2d[poles2d.Upper()] = d2Poles2d[poles2d.Upper()].WithCoord(secmember[3], secmember[4]);
        }

        // linear case is processed...
        if (_sShape == SectionShape.Linear)
        {
            poles[low] = _pts1;
            poles[upp] = _pts2;
            weights[low] = 1.0;
            weights[upp] = 1.0;
            if (!istgt)
            {
                dPoles[low] = _tg1;
                dPoles[upp] = _tg2;
                dPoles[low] = dtg1; // OCCT writes DPoles twice and never D2Poles
                dPoles[upp] = dtg2;
                dWeights[low] = 0.0;
                dWeights[upp] = 0.0;
                d2Weights[low] = 0.0;
                d2Weights[upp] = 0.0;
            }

            return (!istgt);
        }

        // Case of circle
        // The surfaces' own normals (decision 2215); the plane, square to both, is set below.
        ns1 = -ConstRad.InverseNorm(ns1) * ns1;
        ns2 = -ConstRad.InverseNorm(ns2) * ns2;

        center = center.WithXYZ(_pts1.XYZ() + _sg1 * _ray * ns1.XYZ());
        if (!istgt)
        {
            tgc = Vector3D.LinearForm(_sg1 * _ray, dnorm1w, _sg1 * _dray, ns1, _tg1);
            dtgc = Vector3D.LinearForm(_sg1 * _ray, d2norm1w, 2 * _sg1 * _dray, dnorm1w, _sg1 * _d2ray, ns1);
            dtgc += dtg1;
        }

        // ns1 is oriented from the center to pts1, and ns2 from the center to pts2
        // and the trihedron ns1,ns2,nplan is made direct

        if (_sg1 > 0.0)
        {
            ns1 = ns1.Reversed();
            if (!istgt)
            {
                dnorm1w = dnorm1w.Reversed();
                d2norm1w = d2norm1w.Reversed();
            }
        }

        if (_sg2 > 0.0)
        {
            ns2 = ns2.Reversed();
            if (!istgt)
            {
                dnorm2w = dnorm2w.Reversed();
                d2norm2w = d2norm2w.Reversed();
            }
        }

        if (_choix % 2 != 0)
        {
            np = np.Reversed();
            dnp = dnp.Reversed();
            d2np = d2np.Reversed();
        }

        // The plane of the ball's circle, square to both normals, on the guide plane's side.
        ConstRad.SectionNormal(ns1, ns2, dnorm1w, dnorm2w, d2norm1w, d2norm2w, np, dnp, d2np, out Vector3D npBall,
                               out Vector3D dnpBall, out Vector3D d2npBall);
        np = npBall;
        dnp = dnpBall;
        d2np = d2npBall;

        if (_ray < 0.0)
        { // to avoid std::abs(dray) several lines below
            rayprim = -rayprim;
            raysecn = -raysecn;
        }

        if (!istgt)
        {
            return global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns1, ns2, dnorm1w, dnorm2w, d2norm1w, d2norm2w, np, dnp, d2np, _pts1,
                                      _pts2, _tg1, _tg2, dtg1, dtg2, Math.Abs(_ray), rayprim, raysecn, center, tgc,
                                      dtgc, poles, dPoles, d2Poles, weights, dWeights, d2Weights);
        }
        else
        {
            // OCCT passes the guide's plane unreversed here.
            global::Kerf.Engine.Construction.Surfaces.GeomFill.GetCircle(_tConv, ns1, ns2, ConstRad.SectionNormal(ns1, ns2, _nplan), _pts1, _pts2, Math.Abs(_ray), center, poles, weights);
            return false;
        }
    }


    public override void Resolution(int ic2d, double tol, out double tolU, out double tolV)
    {
        if (ic2d == 1)
        {
            tolU = _surf1.UResolution(tol);
            tolV = _surf1.VResolution(tol);
        }
        else
        {
            tolU = _surf2.UResolution(tol);
            tolV = _surf2.VResolution(tol);
        }
    }
}
