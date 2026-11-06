// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_CSWalking.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_CSWalking.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Fillets.Blend.BlendStatus;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// Walks a blend between a curve and a surface (a <see cref="CSFunction"/>, such as BlendFunc_CSConstRad)
/// along a guide, building a <see cref="Line"/>.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT never wrote its recadrage on the surface's restrictions (the code is
/// commented out), so the walking stops with "same points" when it leaves the face, and OCCT prints a line to the
/// standard output there, left out. Its <c>OCCT_DEBUG</c> drawing of the sections is left out too.
/// </remarks>
internal class CSWalking
{
    private bool _done;
    private Line? _line;
    private readonly SurfaceAdaptorBase _surf;
    private readonly CurveAdaptorBase _curv;
    private TopolTool _domain;
    private double _tolpoint3d;
    private double _tolgui;
    private double _pasmax;
    private double _fleche;
    private double _param;
    private double _firstparam;
    private NCollection_HArray1<double>? _firstsol;
    private BlendPoint _previousP = new();
#pragma warning disable CS0414 // set and never read, as in OCCT
    private bool _rebrou;
#pragma warning restore CS0414
    private bool _iscomplete;
    private bool _comptra;
    private double _sens;

    public CSWalking(CurveAdaptorBase curv, SurfaceAdaptorBase surf, TopolTool domain)
    {
        _done = false;
        _surf = surf;
        _curv = curv;
        _domain = domain;
    }

    /// <summary>
    /// Walks from the guide's parameter <paramref name="pdep"/> towards <paramref name="pmax"/>, starting from the
    /// solution <paramref name="parDep"/> (solved again first when <paramref name="appro"/>).
    /// </summary>
    public void Perform(CSFunction func, double pdep, double pmax, double maxStep, double tol3d,
                        double tolGuide, math_Vector parDep, double fleche, bool appro = false)
    {
        _done = false;
        _iscomplete = false;
        _comptra = false;
        _line = new Line();
        int nbvar = func.NbVariables();
        _tolpoint3d = tol3d;
        _tolgui = Math.Abs(tolGuide);
        _fleche = Math.Abs(fleche);
        _rebrou = false;
        _pasmax = Math.Abs(maxStep);
        math_Vector sol = new(1, nbvar);

        _firstsol = new NCollection_HArray1<double>(1, nbvar);

        if (pmax - pdep >= 0.0)
        {
            _sens = 1.0;
        }
        else
        {
            _sens = -1.0;
        }

        BlendStatus state;

        _param = pdep;
        _firstparam = pdep;
        func.Set(_param);

        if (appro)
        {
            Containment situ;
            math_Vector tolerance = new(1, nbvar), infbound = new(1, nbvar), supbound = new(1, nbvar);
            func.GetTolerance(tolerance, _tolpoint3d);
            func.GetBounds(infbound, supbound);
            FunctionSetRoot rsnld = new(func, tolerance, 30);

            rsnld.Perform(func, parDep, infbound, supbound);

            if (!rsnld.IsDone)
            {
                return;
            }
            rsnld.Root(sol);

            situ = _domain.Classify(func.Pnt2d(), Min(tolerance[1], tolerance[2]));

            if (situ != global::Kerf.Topology.Containment.In)
            {
                return;
            }
        }
        else
        {
            sol.Initialized(parDep);
        }

        for (int i = 1; i <= nbvar; i++)
        {
            _firstsol.ChangeValue(i) = sol[i];
        }
        state = TestArret(func, sol, false, BlendStatus.OK);
        if (state != BlendStatus.OK)
        {
            return;
        }
        // OCCT_DEBUG Drawsect left out.
        // Update the line.
        _line.Append(_previousP);
        double u, v, w;
        _previousP.ParametersOnS(out u, out v);
        w = _previousP.ParameterOnC();
        Extremity p1 = new(_previousP.PointOnS(), u, v, _previousP.Parameter(), _tolpoint3d);
        Extremity p2 = new(_previousP.PointOnC(), w, _previousP.Parameter(), _tolpoint3d);
        if (_sens > 0.0)
        {
            _line.SetStartPoints(p1, p2);
        }
        else
        {
            _line.SetEndPoints(p1, p2);
        }

        InternalPerform(func, sol, pmax);

        _done = true;
    }

    /// <summary>Walks the other way from the first section, down to <paramref name="pmin"/>.</summary>
    public bool Complete(CSFunction func, double pmin)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_iscomplete)
        {
            return true;
        }

        _sens = -_sens;
        int nbvar = func.NbVariables();
        math_Vector sol = new(1, nbvar);
        for (int i = 1; i <= nbvar; i++)
        {
            sol[i] = _firstsol!.Value(i);
        }

        _param = _firstparam;

        InternalPerform(func, sol, pmin);

        _sens = -_sens;
        _iscomplete = true;
        return true;
    }

    public bool IsDone => _done;

    public Line Line()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _line!;
    }

    private BlendStatus TestArret(CSFunction function, math_Vector sol, bool testDefl, BlendStatus state)
    {
        Point3D pt1;
        Vector3D v1, v2;
        Vector3D tgp1, nor1;
        Vector2D v12d;
        Point2D pt2d;
        double pOnC;
        BlendStatus state1, state2;
        WalkingTypeTrans tras = WalkingTypeTrans.Undecided;

        if (function.IsSolution(sol, _tolpoint3d))
        {
            pt1 = function.PointOnS();
            _ = function.PointOnC();
            pt2d = function.Pnt2d();
            pOnC = function.ParameterOnC();
            v1 = function.TangentOnS();
            v2 = function.TangentOnC();
            v12d = function.Tangent2d();

            if (testDefl)
            {
                // Check the deflection on each surface and on the guide.
                state1 = CheckDeflectionOnSurf(pt1, pt2d, v1, v12d);
                // For problems in point/face walkings, the test on the curve is temporarily shelved.
                state2 = BlendStatus.StepTooSmall;
            }
            else
            {
                state1 = BlendStatus.OK;
                state2 = BlendStatus.OK;
            }

            if (state1 == BlendStatus.Backward)
            {
                state1 = BlendStatus.StepTooLarge;
                _rebrou = true;
            }

            if (state2 == BlendStatus.Backward)
            {
                state2 = BlendStatus.StepTooLarge;
                _rebrou = true;
            }

            if (state1 == BlendStatus.StepTooLarge || state2 == BlendStatus.StepTooLarge)
            {
                return BlendStatus.StepTooLarge;
            }

            if (!_comptra)
            {
                function.Tangent(pt2d.X, pt2d.Y, out tgp1, out nor1);
                double testra = tgp1.Dot(nor1.Crossed(v1));
                if (Math.Abs(testra) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    if (testra < 0.0)
                    {
                        tras = WalkingTypeTrans.In;
                    }
                    else if (testra > 0.0)
                    {
                        tras = WalkingTypeTrans.Out;
                    }
                    _comptra = true;
                    _line!.Set(tras);
                }
            }

            if (state1 == BlendStatus.OK || state2 == BlendStatus.OK)
            {
                _previousP.SetValue(function.PointOnS(), function.PointOnC(), _param, pt2d.X, pt2d.Y, pOnC, v1, v2,
                                   v12d);

                return state;
            }
            if (state1 == BlendStatus.StepTooSmall && state2 == BlendStatus.StepTooSmall)
            {
                _previousP.SetValue(function.PointOnS(), function.PointOnC(), _param, pt2d.X, pt2d.Y, pOnC, v1, v2,
                                   v12d);
                if (state == BlendStatus.OK)
                {
                    return BlendStatus.StepTooSmall;
                }
                else
                {
                    return state;
                }
            }

            if (state == BlendStatus.OK)
            {
                return BlendStatus.SamePoints;
            }
            else
            {
                return state;
            }
        }
        else
        {
            return BlendStatus.StepTooLarge;
        }
    }

    private BlendStatus CheckDeflectionOnSurf(in Point3D psurf, in Point2D ponsurf, in Vector3D tgsurf,
                                               in Vector2D tgonsurf)
    {
        // set by tests in U4, corresponds to 11.478 degrees
        const double cosRef3D = 0.98;

        const double cosRef2D = 0.88; // corresponds to 25 degrees

        double norme, prevNorme, cosi, cosi2;
        double flecheCourante;
        double du, dv, duv;
        double paramu, paramv, tolu, tolv;
        Point3D prevP;
        Vector3D prevTg;
        Vector2D previousd2d;

        prevP = _previousP.PointOnS();
        prevTg = _previousP.TangentOnS();
        tolu = HSurfaceTool.UResolution(_surf, _tolpoint3d);
        tolv = HSurfaceTool.VResolution(_surf, _tolpoint3d);

        Vector3D corde = new(prevP, psurf);
        norme = corde.SquareMagnitude;
        prevNorme = prevTg.SquareMagnitude;

        double toler3d = _tolpoint3d;
        if (norme <= toler3d * toler3d || prevNorme <= toler3d * toler3d)
        {
            // the same point may have to be forced
            return BlendStatus.SamePoints;
        }
        cosi = _sens * corde * prevTg;
        if (cosi < 0.0)
        { // 3d angle > pi/2: going back
            return BlendStatus.Backward;
        }

        cosi2 = cosi * cosi / prevNorme / norme;
        if (cosi2 < cosRef3D)
        {
            return BlendStatus.StepTooLarge;
        }

        _previousP.ParametersOnS(out paramu, out paramv);
        previousd2d = _previousP.Tangent2d();

        du = ponsurf.X - paramu;
        dv = ponsurf.Y - paramv;
        duv = du * du + dv * dv;
        if ((Math.Abs(du) < tolu && Math.Abs(dv) < tolv)
            || (Math.Abs(previousd2d.X) < tolu && Math.Abs(previousd2d.Y) < tolv))
        {
            // the same point may have to be forced
            return BlendStatus.SamePoints; // same 2d point
        }
        cosi = _sens * (du * previousd2d.X + dv * previousd2d.Y);
        if (cosi < 0)
        {
            return BlendStatus.Backward;
        }

        cosi = _sens * corde * tgsurf;
        cosi2 = cosi * cosi / tgsurf.SquareMagnitude / norme;
        if (cosi2 < cosRef3D || cosi < 0.0)
        {
            return BlendStatus.StepTooLarge;
        }

        cosi = _sens * (du * tgonsurf.X + dv * tgonsurf.Y) / tgonsurf.Magnitude;
        cosi2 = cosi * cosi / duv;
        if (cosi2 < cosRef2D || cosi < 0.0)
        {
            return BlendStatus.StepTooLarge;
        }

        flecheCourante = (prevTg.Normalized().XYZ() - tgsurf.Normalized().XYZ()).SquareModulus() * norme / 64.0;

        if (flecheCourante <= 0.25 * _fleche * _fleche)
        {
            return BlendStatus.StepTooSmall;
        }
        if (flecheCourante > _fleche * _fleche)
        {
            // step too large
            return BlendStatus.StepTooLarge;
        }

        return BlendStatus.OK;
    }

    // OCCT's Recadre is commented out in its source; not ported.

    private void Transition(Curve2DAdaptorBase a, double param, ref WalkingTransition tLine,
                            ref WalkingTransition tArc)
    {
        Point2D p2d;
        Vector2D dp2d;

        Point3D pbid;
        Vector3D d1u, d1v, normale, tgrst = new();

        HCurve2dTool.D1(a, param, out p2d, out dp2d);
        HSurfaceTool.D1(_surf, p2d.X, p2d.Y, out pbid, out d1u, out d1v);

        tgrst = Vector3D.LinearForm(dp2d.X, d1u, dp2d.Y, d1v);
        normale = d1u.Crossed(d1v);

        global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(_previousP.TangentOnS(), tgrst, new Direction3D(normale), ref tLine, ref tArc);
    }

    private void MakeExtremity(ref Extremity extrem, int index, double param, bool isVtx,
                               HVertex? vtx)
    {
        WalkingTransition tline = new(), tarc = new();
        double prm, u, v;
        int nbarc;
        TopolTool iter;

        _previousP.ParametersOnS(out u, out v);
        extrem.SetValue(_previousP.PointOnS(), u, v, _previousP.Parameter(), _tolpoint3d);
        iter = _domain;

        iter.Init();
        nbarc = 1;

        if (!isVtx)
        {
            while (nbarc < index)
            {
                nbarc++;
                iter.Next();
            }
            Transition(iter.Value(), param, ref tline, ref tarc);
            extrem.AddArc(iter.Value(), param, tline, tarc);
        }

        else
        {
            extrem.SetVertex(vtx!);
            while (iter.More())
            {
                Curve2DAdaptorBase arc = iter.Value();
                if (nbarc != index)
                {
                    iter.Initialize(arc);
                    iter.InitVertexIterator();
                    while (iter.MoreVertex())
                    {
                        if (iter.Identical(vtx!, iter.Vertex()))
                        {
                            prm = BlendTool.Parameter(vtx!, arc);
                            Transition(arc, prm, ref tline, ref tarc);
                            extrem.AddArc(arc, prm, tline, tarc);
                        }
                        iter.NextVertex();
                    }
                }
                else
                {
                    Transition(arc, param, ref tline, ref tarc);
                    extrem.AddArc(arc, param, tline, tarc);
                }
                nbarc++;
                iter.Next();
            }
        }
    }

    private void InternalPerform(CSFunction func, math_Vector sol, double bound)
    {
        double stepw = _pasmax;
        double parprec = _param;

        BlendStatus state;
        Containment situ;
        double w, u, v;
        int nbarc;
        int index = 0;
        bool isvtx = false;
        int nbvar = func.NbVariables();
        bool arrive, recad, echecrecad;
        Point2D p2d;
        math_Vector tolerance = new(1, nbvar), infbound = new(1, nbvar), supbound = new(1, nbvar),
                    parinit = new(1, nbvar);
        math_Vector solrst = new(1, nbvar);
        HVertex? vtx = null;
        Extremity exts = new(), extc = new();

        func.GetTolerance(tolerance, _tolpoint3d);
        func.GetBounds(infbound, supbound);

        FunctionSetRoot rsnld = new(func, tolerance, 30);
        parinit.Initialized(sol);

        _param = parprec + _sens * stepw;
        arrive = (_sens * (_param - bound) > 0.0);

        while (!arrive)
        {
            func.Set(_param);
            rsnld.Perform(func, parinit, infbound, supbound);

            if (!rsnld.IsDone)
            {
                state = BlendStatus.StepTooLarge;
            }
            else
            {
                rsnld.Root(sol);

                situ = _domain.Classify(func.Pnt2d(), Min(tolerance[1], tolerance[2]));

                w = bound;
                recad = false;
                echecrecad = false;

                if (situ == global::Kerf.Topology.Containment.Out || situ == On)
                {
                    // inverse problem on surf
                    //	recad = Recadre(FuncInv,sol,solrst,Index,Isvtx,Vtx);
                    isvtx = false; // waiting for Recadre
                    if (recad)
                    {
                        w = solrst[2];
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }
                if (!echecrecad)
                {
                    if (recad)
                    {
                        // solution on surf
                        state = BlendStatus.OnRst1;
                        _param = w;
                        _domain.Init();
                        nbarc = 1;
                        while (nbarc < index)
                        {
                            nbarc++;
                            _domain.Next();
                        }
                        p2d = HCurve2dTool.Value(_domain.Value(), solrst[1]);
                        sol[1] = p2d.X;
                        sol[2] = p2d.Y;
                        sol[3] = solrst[3];
                        func.Set(_param);
                    }
                    else
                    {
                        state = BlendStatus.OK;
                    }
                    state = TestArret(func, sol, true, state);
                }
                else
                {
                    // Recadrage failed. Leave with SamePoints
                    state = BlendStatus.SamePoints;
                }
            }

            switch (state)
            {
                case BlendStatus.OK:
                {
                    // OCCT_DEBUG Drawsect left out.
                    // Update the line.
                    if (_sens > 0.0)
                    {
                        _line!.Append(_previousP);
                    }
                    else
                    {
                        _line!.Prepend(_previousP);
                    }
                    parinit.Initialized(sol);
                    parprec = _param;

                    if (_param == bound)
                    {
                        arrive = true;
                        _previousP.ParametersOnS(out u, out v);
                        exts.SetValue(_previousP.PointOnS(), u, v, _previousP.Parameter(), _tolpoint3d);
                        extc.SetValue(_previousP.PointOnC(), _previousP.ParameterOnC(), _previousP.Parameter(),
                                      _tolpoint3d);
                        // Indicate the end on Bound.
                    }
                    else
                    {
                        _param = _param + _sens * stepw;
                        if (_sens * (_param - bound) > -_tolgui)
                        {
                            _param = bound;
                        }
                    }
                }
                break;

                case BlendStatus.StepTooLarge:
                {
                    stepw = stepw / 2.0;
                    if (Math.Abs(stepw) < _tolgui)
                    {
                        _previousP.ParametersOnS(out u, out v);
                        exts.SetValue(_previousP.PointOnS(), u, v, _previousP.Parameter(), _tolpoint3d);
                        extc.SetValue(_previousP.PointOnC(), _previousP.ParameterOnC(), _previousP.Parameter(),
                                      _tolpoint3d);
                        arrive = true;
                    }
                    else
                    {
                        _param = parprec + _sens * stepw; // no risk of going past Bound.
                    }
                }
                break;

                case BlendStatus.StepTooSmall:
                {
                    // OCCT_DEBUG Drawsect left out.
                    // Update the line.
                    if (_sens > 0.0)
                    {
                        _line!.Append(_previousP);
                    }
                    else
                    {
                        _line!.Prepend(_previousP);
                    }
                    parinit.Initialized(sol);
                    parprec = _param;

                    stepw = Min(1.5 * stepw, _pasmax);
                    if (_param == bound)
                    {
                        arrive = true;
                        _previousP.ParametersOnS(out u, out v);
                        exts.SetValue(_previousP.PointOnS(), u, v, _previousP.Parameter(), _tolpoint3d);
                        extc.SetValue(_previousP.PointOnC(), _previousP.ParameterOnC(), _previousP.Parameter(),
                                      _tolpoint3d);
                        // Indicate the end on Bound.
                    }
                    else
                    {
                        _param = _param + _sens * stepw;
                        if (_sens * (_param - bound) > -_tolgui)
                        {
                            _param = bound;
                        }
                    }
                }
                break;

                case BlendStatus.OnRst1:
                {
                    // OCCT_DEBUG Drawsect left out.
                    if (_sens > 0.0)
                    {
                        _line!.Append(_previousP);
                    }
                    else
                    {
                        _line!.Prepend(_previousP);
                    }
                    MakeExtremity(ref exts, index, solrst[1], isvtx, vtx);
                    extc.SetValue(_previousP.PointOnC(), _previousP.ParameterOnC(), _previousP.Parameter(), _tolpoint3d);
                    arrive = true;
                }
                break;

                case BlendStatus.SamePoints:
                {
                    // Stop. (OCCT prints " Points confondus dans le cheminement" to the standard output: left out.)
                    _previousP.ParametersOnS(out u, out v);
                    exts.SetValue(_previousP.PointOnS(), u, v, _previousP.Parameter(), _tolpoint3d);
                    extc.SetValue(_previousP.PointOnC(), _previousP.ParameterOnC(), _previousP.Parameter(), _tolpoint3d);
                    arrive = true;
                }
                break;

                default:
                    break;
            }
            if (arrive)
            {
                if (_sens > 0.0)
                {
                    _line!.SetEndPoints(exts, extc);
                }
                else
                {
                    _line!.SetStartPoints(exts, extc);
                }
            }
        }
    }
}
