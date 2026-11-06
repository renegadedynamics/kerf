// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_RstRstLineBuilder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_RstRstLineBuilder.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_RstRstLineBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Fillets.Blend.DecrochStatus;
using static Kerf.Engine.Fillets.Blend.BlendStatus;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// Walks a blend between two restrictions (arcs of two surfaces) along a guide, building a
/// <see cref="Line"/>. The walking stops where the section leaves a restriction's ends (solving the
/// curve/point inverse function) or loses contact with a surface (the surface/curve inverse function, "décroché").
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT's <c>OCCT_DEBUG</c> drawing of the sections and its derivative traces are
/// left out.
/// </remarks>
internal class RstRstLineBuilder
{
    private bool _done;
    private Line? _line;
    private readonly math_Vector _sol;
    private readonly SurfaceAdaptorBase _surf1;
    private readonly TopolTool _domain1;
    private readonly SurfaceAdaptorBase _surf2;
    private readonly TopolTool _domain2;
    private readonly Curve2DAdaptorBase _rst1;
    private readonly Curve2DAdaptorBase _rst2;
    private double _tolpoint3d;
    private double _tolgui;
    private double _pasmax;
    private double _fleche;
    private double _param;
    private BlendPoint _previousP = new();
    private bool _iscomplete;
    private bool _comptra;
    private double _sens;
    private DecrochStatus _decrochdeb;
    private DecrochStatus _decrochfin;

    public RstRstLineBuilder(SurfaceAdaptorBase surf1, Curve2DAdaptorBase rst1, TopolTool domain1,
                                       SurfaceAdaptorBase surf2, Curve2DAdaptorBase rst2, TopolTool domain2)
    {
        _done = false;
        _sol = new math_Vector(1, 2);
        _surf1 = surf1;
        _domain1 = domain1;
        _surf2 = surf2;
        _domain2 = domain2;
        _rst1 = rst1;
        _rst2 = rst2;
        _tolpoint3d = 0.0;
        _tolgui = 0.0;
        _pasmax = 0.0;
        _fleche = 0.0;
        _param = 0.0;
        _iscomplete = false;
        _comptra = false;
        _sens = 0.0;
        _decrochdeb = DecrochStatus.NoDecroch;
        _decrochfin = DecrochStatus.NoDecroch;
    }

    /// <summary>
    /// Walks from the guide's parameter <paramref name="pdep"/> towards <paramref name="pmax"/>, starting from the
    /// solution <paramref name="parDep"/> (solved again first when <paramref name="appro"/>).
    /// </summary>
    public void Perform(RstRstFunction func, SurfCurvFuncInv finv1, CurvPointFuncInv finvP1,
                        SurfCurvFuncInv finv2, CurvPointFuncInv finvP2, double pdep, double pmax,
                        double maxStep, double tol3d, double tolGuide, math_Vector parDep, double fleche,
                        bool appro = false)
    {
        _done = false;
        _iscomplete = false;
        _comptra = false;
        _line = new Line();
        _tolpoint3d = tol3d;
        _tolgui = Math.Abs(tolGuide);
        _fleche = Math.Abs(fleche);
        _pasmax = Math.Abs(maxStep);

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
        func.Set(_param);

        if (appro)
        {
            Containment siturst1, siturst2;
            DecrochStatus decroch;
            math_Vector tolerance = new(1, 2), infbound = new(1, 2), supbound = new(1, 2);
            func.GetTolerance(tolerance, _tolpoint3d);
            func.GetBounds(infbound, supbound);
            FunctionSetRoot rsnld = new(func, tolerance, 30);

            rsnld.Perform(func, parDep, infbound, supbound);

            if (!rsnld.IsDone)
            {
                return;
            }
            rsnld.Root(_sol);
            if (!CheckInside(func, out siturst1, out siturst2, out decroch))
            {
                return;
            }
        }
        else
        {
            _sol.Initialized(parDep);
        }

        state = TestArret(func, false, BlendStatus.OK);
        if (state != BlendStatus.OK)
        {
            return;
        }
        // OCCT_DEBUG Drawsect left out.
        // Update the line.
        _line.Append(_previousP);
        double u, v;
        u = _previousP.ParameterOnC1();
        v = _previousP.ParameterOnC2();
        Extremity ptf1 = new(_previousP.PointOnC1(), u, _previousP.Parameter(), _tolpoint3d);
        Extremity ptf2 = new(_previousP.PointOnC2(), v, _previousP.Parameter(), _tolpoint3d);
        if (!_previousP.IsTangencyPoint)
        {
            ptf1.SetTangent(_previousP.TangentOnC1());
            ptf2.SetTangent(_previousP.TangentOnC2());
        }

        if (_sens > 0.0)
        {
            _line.SetStartPoints(ptf1, ptf2);
        }
        else
        {
            _line.SetEndPoints(ptf1, ptf2);
        }

        InternalPerform(func, finv1, finvP1, finv2, finvP2, pmax);
        _done = true;
    }

    /// <summary>
    /// Solves a first section at <paramref name="pdep"/> and moves it to where the walking stops (the restrictions'
    /// ends or a lost contact): <paramref name="psol"/> and <paramref name="parSol"/> receive it.
    /// </summary>
    public bool PerformFirstSection(RstRstFunction func, SurfCurvFuncInv finv1,
                                    CurvPointFuncInv finvP1, SurfCurvFuncInv finv2,
                                    CurvPointFuncInv finvP2, double pdep, double pmax, math_Vector parDep,
                                    double tol3d, double tolGuide, bool recRst1, bool recP1, bool recRst2, bool recP2,
                                    ref double psol, math_Vector parSol)
    {
        _done = false;
        _iscomplete = false;
        _comptra = false;
        _line = new Line();
        _tolpoint3d = tol3d;
        _tolgui = Math.Abs(tolGuide);

        if (pmax - pdep >= 0.0)
        {
            _sens = 1.0;
        }
        else
        {
            _sens = -1.0;
        }

        bool recadp1, recadp2, recadrst1, recadrst2;
        double wp1, wp2, wrst1, wrst2;
        BlendStatus state = BlendStatus.OnRst12;
        double trst11 = 0.0, trst12 = 0.0, trst21 = 0.0, trst22 = 0.0;
        math_Vector infbound = new(1, 2), supbound = new(1, 2), tolerance = new(1, 2);
        math_Vector solinvp1 = new(1, 2), solinvp2 = new(1, 2), solinvrst1 = new(1, 3), solinvrst2 = new(1, 3);
        HVertex? vtxp1 = null, vtxp2 = null, vtxrst1 = null, vtxrst2 = null;
        bool isVtxp1 = false, isVtxp2 = false, isVtxrst1 = false, isVtxrst2 = false;
        wp1 = wp2 = wrst1 = wrst2 = pmax;
        _param = pdep;
        func.Set(_param);
        func.GetTolerance(tolerance, _tolpoint3d);
        func.GetBounds(infbound, supbound);

        FunctionSetRoot rsnld = new(func, tolerance, 30);
        rsnld.Perform(func, parDep, infbound, supbound);
        if (!rsnld.IsDone)
        {
            return false;
        }
        rsnld.Root(_sol);

        recadrst1 = recRst1 && Recadre1(func, finv1, solinvrst1, ref isVtxrst1, ref vtxrst1);
        if (recadrst1)
        {
            wrst1 = solinvrst1[1];
        }

        recadp1 = recP1 && Recadre1(finvP1, solinvp1, ref isVtxp1, ref vtxp1);
        if (recadp1)
        {
            wp1 = solinvp1[1];
        }

        recadrst2 = recRst2 && Recadre2(func, finv2, solinvrst2, ref isVtxrst2, ref vtxrst2);
        if (recadrst2)
        {
            wrst2 = solinvrst2[1];
        }

        recadp2 = recP2 && Recadre2(finvP2, solinvp2, ref isVtxp2, ref vtxp2);
        if (recadp2)
        {
            wp2 = solinvp2[1];
        }

        if (!recadrst1 && !recadp1 && !recadrst2 && !recadp2)
        {
            return false;
        }

        // it is checked if the contact was lost or domain 1 was left
        if (recadp1 && recadrst1)
        {
            if (_sens * (wrst1 - wp1) > _tolgui)
            { // at first one leaves the domain
                wrst1 = wp1;
                trst12 = solinvp1[2];
                trst11 = BlendTool.Parameter(vtxp1!, _rst1);
                isVtxrst2 = isVtxp1; // OCCT: the second restriction's vertex is set where the first's is meant
                vtxrst2 = vtxp1;
                recadrst1 = false;
            }
            else
            { // the contact is lost
                trst11 = solinvrst1[3];
                trst12 = solinvrst1[2];
                recadp1 = false;
            }
        }
        else if (recadp1)
        {
            wrst1 = wp1;
            trst12 = solinvp1[2];
            trst11 = BlendTool.Parameter(vtxp1!, _rst1);
            isVtxrst1 = isVtxp1;
            vtxrst1 = vtxp1;
        }
        else if (recadrst1)
        {
            trst11 = solinvrst1[3];
            trst12 = solinvrst1[2];
        }

        // it is checked if the contact was lost or domain 2 was left
        if (recadp2 && recadrst2)
        {
            if (_sens * (wrst2 - wp2) > _tolgui)
            { // at first one leaves the domain
                wrst2 = wp2;
                trst21 = solinvp2[2];
                trst22 = BlendTool.Parameter(vtxp2!, _rst2);
                isVtxrst2 = isVtxp2;
                vtxrst2 = vtxp2;
                recadrst2 = false;
            }
            else
            {
                trst22 = solinvrst2[3];
                trst21 = solinvrst2[2];
                recadp2 = false;
            }
        }
        else if (recadp2)
        {
            wrst2 = wp2;
            trst21 = solinvp2[2];
            trst22 = BlendTool.Parameter(vtxp2!, _rst2);
            isVtxrst2 = isVtxp2;
            vtxrst2 = vtxp2;
        }
        else if (recadrst2)
        {
            trst22 = solinvrst2[3];
            trst21 = solinvrst2[2];
        }

        // it is checked on which curve the contact is lost earlier
        if (recadrst1 && recadrst2)
        {
            if (Math.Abs(wrst1 - wrst2) < _tolgui)
            {
                state = BlendStatus.OnRst12;
                _param = 0.5 * (wrst1 + wrst2);
                _sol[1] = trst11;
                _sol[2] = trst22;
            }
            else if (_sens * (wrst1 - wrst2) < 0)
            {
                // contact lost on Rst1
                state = BlendStatus.OnRst1;
                _param = wrst1;
                _sol[1] = trst11;
                _sol[2] = trst12;
            }
            else
            {
                // contact lost on rst2
                state = BlendStatus.OnRst2;
                _param = wrst2;
                _sol[1] = trst21;
                _sol[2] = trst22;
            }
            func.Set(_param);
        }
        else if (recadrst1)
        {
            // ground on rst1
            state = BlendStatus.OnRst1;
            _param = wrst1;
            _sol[1] = trst11;
            _sol[2] = trst12;
            func.Set(_param);
        }
        else if (recadrst2)
        {
            // ground on rst2
            state = BlendStatus.OnRst2;
            _param = wrst2;
            _sol[1] = trst21;
            _sol[2] = trst22;
            func.Set(_param);
        }
        // it is checked on which curves one leaves first
        else if (recadp1 && recadp2)
        {
            if (Math.Abs(wrst1 - wrst2) < _tolgui)
            {
                state = BlendStatus.OnRst12;
                _param = 0.5 * (wrst1 + wrst2);
                _sol[1] = trst11;
                _sol[2] = trst22;
            }
            else if (_sens * (wrst1 - wrst2) < 0)
            {
                // sol on Rst1
                state = BlendStatus.OnRst1;
                _param = wrst1;
                _sol[1] = trst11;
                _sol[2] = trst12;
            }
            else
            {
                // ground on rst2
                state = BlendStatus.OnRst2;
                _param = wrst2;
                _sol[1] = trst21;
                _sol[2] = trst22;
            }
            func.Set(_param);
        }
        else if (recadp1)
        {
            // ground on rst1
            state = BlendStatus.OnRst1;
            _param = wrst1;
            _sol[1] = trst11;
            _sol[2] = trst12;
            func.Set(_param);
        }
        else if (recadp2)
        {
            // ground on rst2
            state = BlendStatus.OnRst2;
            _param = wrst2;
            _sol[1] = trst21;
            _sol[2] = trst22;
            func.Set(_param);
        }

        state = TestArret(func, false, state);
        psol = _param;
        parSol.Initialized(_sol);
        return true;
    }

    /// <summary>Walks the other way from the line's start, down to <paramref name="pmin"/>.</summary>
    public bool Complete(RstRstFunction func, SurfCurvFuncInv finv1, CurvPointFuncInv finvP1,
                         SurfCurvFuncInv finv2, CurvPointFuncInv finvP2, double pmin)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_iscomplete)
        {
            return true;
        }
        if (_sens > 0.0)
        {
            _previousP = _line!.Point(1);
        }
        else
        {
            _previousP = _line!.Point(_line.NbPoints());
        }
        _sens = -_sens;
        _param = _previousP.Parameter();
        _sol[1] = _previousP.ParameterOnC1();
        _sol[2] = _previousP.ParameterOnC2();

        InternalPerform(func, finv1, finvP1, finv2, finvP2, pmin);
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

    public bool Decroch1Start() => _decrochdeb == DecrochStatus.DecrochRst1 || _decrochdeb == DecrochStatus.DecrochBoth;

    public bool Decroch1End() => _decrochfin == DecrochStatus.DecrochRst1 || _decrochfin == DecrochStatus.DecrochBoth;

    public bool Decroch2Start() => _decrochdeb == DecrochStatus.DecrochRst2 || _decrochdeb == DecrochStatus.DecrochBoth;

    public bool Decroch2End() => _decrochfin == DecrochStatus.DecrochRst2 || _decrochfin == DecrochStatus.DecrochBoth;

    // The walking without the extremities.
    private void InternalPerform(RstRstFunction func, SurfCurvFuncInv finv1,
                                 CurvPointFuncInv finvP1, SurfCurvFuncInv finv2,
                                 CurvPointFuncInv finvP2, double bound)
    {
        double stepw = _pasmax;
        int nbp = _line!.NbPoints();
        if (nbp >= 2)
        { // The last step is redone if it is not too small.
            if (_sens < 0.0)
            {
                stepw = (_line.Point(2).Parameter() - _line.Point(1).Parameter());
            }
            else
            {
                stepw = (_line.Point(nbp).Parameter() - _line.Point(nbp - 1).Parameter());
            }
            stepw = Max(stepw, 100.0 * _tolgui);
        }
        double parprec = _param;
        if (_sens * (parprec - bound) >= -_tolgui)
        {
            return;
        }
        BlendStatus state = BlendStatus.OnRst12;
        double trst11 = 0.0, trst12 = 0.0, trst21 = 0.0, trst22 = 0.0;
        Containment situonc1 = Unknown, situonc2 = Unknown;
        DecrochStatus decroch = DecrochStatus.NoDecroch;
        bool arrive, recadp1, recadp2, recadrst1, recadrst2, echecrecad;
        double wp1, wp2, wrst1, wrst2;
        math_Vector infbound = new(1, 2), supbound = new(1, 2);
        math_Vector parinit = new(1, 2), tolerance = new(1, 2);
        math_Vector solinvp1 = new(1, 2), solinvp2 = new(1, 2), solinvrst1 = new(1, 3), solinvrst2 = new(1, 3);
        HVertex? vtxp1 = null, vtxp2 = null, vtxrst1 = null, vtxrst2 = null;
        bool isVtxp1 = false, isVtxp2 = false, isVtxrst1 = false, isVtxrst2 = false;
        Extremity extrst1 = new(), extrst2 = new();

        func.GetTolerance(tolerance, _tolpoint3d);
        func.GetBounds(infbound, supbound);

        FunctionSetRoot rsnld = new(func, tolerance, 30);
        parinit.Initialized(_sol);

        arrive = false;
        _param = parprec + _sens * stepw;
        if (_sens * (_param - bound) > 0.0)
        {
            stepw = _sens * (bound - parprec) * 0.5;
            _param = parprec + _sens * stepw;
        }

        while (!arrive)
        {
            bool bonpoint = true;
            // OCCT_DEBUG_BBPP_N_TRDERIV traces left out.
            func.Set(_param);
            rsnld.Perform(func, parinit, infbound, supbound);

            if (rsnld.IsDone)
            {
                rsnld.Root(_sol);
                if (!CheckInside(func, out situonc1, out situonc2, out decroch) && _line.NbPoints() == 1)
                {
                    state = BlendStatus.StepTooLarge;
                    bonpoint = false;
                }
            }
            else
            {
                state = BlendStatus.StepTooLarge;
                bonpoint = false;
            }
            if (bonpoint)
            {
                wp1 = wp2 = wrst1 = wrst2 = bound;
                recadp1 = recadp2 = recadrst1 = recadrst2 = false;
                echecrecad = false;
                if (situonc1 != global::Kerf.Topology.Containment.In)
                {
                    // pb inversion rst/rst
                    recadp1 = Recadre1(finvP1, solinvp1, ref isVtxp1, ref vtxp1);
                    if (recadp1)
                    {
                        wp1 = solinvp1[1];
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }

                if (situonc2 != global::Kerf.Topology.Containment.In)
                {
                    // pb inversion point/surf
                    recadp2 = Recadre2(finvP2, solinvp2, ref isVtxp2, ref vtxp2);
                    if (recadp2)
                    {
                        wp2 = solinvp2[1];
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }

                if (decroch == DecrochStatus.DecrochRst1 || decroch == DecrochStatus.DecrochBoth)
                {
                    // pb inversion rst1/surf1
                    recadrst1 = Recadre1(func, finv1, solinvrst1, ref isVtxrst1, ref vtxrst1);
                    if (recadrst1)
                    {
                        wrst1 = solinvrst1[1];
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }

                if (decroch == DecrochStatus.DecrochRst2 || decroch == DecrochStatus.DecrochBoth)
                {
                    // pb inverse rst2/surf2
                    recadrst2 = Recadre2(func, finv2, solinvrst2, ref isVtxrst2, ref vtxrst2);
                    if (recadrst2)
                    {
                        wrst2 = solinvrst2[1];
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }

                decroch = DecrochStatus.NoDecroch;
                if (recadp1 || recadp2 || recadrst1 || recadrst2)
                {
                    echecrecad = false;
                }

                if (!echecrecad)
                {
                    // it is checked if the contact was lost or domain 1 was left
                    if (recadp1 && recadrst1)
                    {
                        if (_sens * (wrst1 - wp1) > _tolgui)
                        { // first one leaves the domain
                            wrst1 = wp1;
                            trst12 = solinvp1[2];
                            trst11 = BlendTool.Parameter(vtxp1!, _rst1);
                            isVtxrst2 = isVtxp1; // OCCT: the second restriction's vertex is set where the first's is meant
                            vtxrst2 = vtxp1;
                            recadrst1 = false;
                        }
                        else
                        { // contact is lost
                            trst11 = solinvrst1[3];
                            trst12 = solinvrst1[2];
                            recadp1 = false;
                        }
                    }
                    else if (recadp1)
                    {
                        wrst1 = wp1;
                        trst12 = solinvp1[2];
                        trst11 = BlendTool.Parameter(vtxp1!, _rst1);
                        isVtxrst1 = isVtxp1;
                        vtxrst1 = vtxp1;
                    }
                    else if (recadrst1)
                    {
                        trst11 = solinvrst1[3];
                        trst12 = solinvrst1[2];
                    }

                    // it is checked if the contact was lost or domain 2 was left
                    if (recadp2 && recadrst2)
                    {
                        if (_sens * (wrst2 - wp2) > _tolgui)
                        { // first one leaves the domain
                            wrst2 = wp2;
                            trst21 = solinvp2[2];
                            trst22 = BlendTool.Parameter(vtxp2!, _rst2);
                            isVtxrst2 = isVtxp2;
                            vtxrst2 = vtxp2;
                            recadrst2 = false;
                        }
                        else
                        {
                            trst22 = solinvrst2[3];
                            trst21 = solinvrst2[2];
                            recadp2 = false;
                        }
                    }
                    else if (recadp2)
                    {
                        wrst2 = wp2;
                        trst21 = solinvp2[2];
                        trst22 = BlendTool.Parameter(vtxp2!, _rst2);
                        isVtxrst2 = isVtxp2;
                        vtxrst2 = vtxp2;
                    }
                    else if (recadrst2)
                    {
                        trst22 = solinvrst2[3];
                        trst21 = solinvrst2[2];
                    }

                    // it is checked on which curve the contact is lost earlier
                    if (recadrst1 && recadrst2)
                    {
                        if (Math.Abs(wrst1 - wrst2) < _tolgui)
                        {
                            state = BlendStatus.OnRst12;
                            decroch = DecrochStatus.DecrochBoth;
                            _param = 0.5 * (wrst1 + wrst2);
                            _sol[1] = trst11;
                            _sol[2] = trst22;
                        }
                        else if (_sens * (wrst1 - wrst2) < 0)
                        {
                            // contact is lost on Rst1
                            state = BlendStatus.OnRst1;
                            decroch = DecrochStatus.DecrochRst1;
                            _param = wrst1;
                            _sol[1] = trst11;
                            _sol[2] = trst12;
                        }
                        else
                        {
                            // contact is lost on rst2
                            state = BlendStatus.OnRst2;
                            decroch = DecrochStatus.DecrochRst2;
                            _param = wrst2;
                            _sol[1] = trst21;
                            _sol[2] = trst22;
                        }
                        func.Set(_param);
                    }
                    else if (recadrst1)
                    {
                        // ground on rst1
                        state = BlendStatus.OnRst1;
                        decroch = DecrochStatus.DecrochRst1;
                        _param = wrst1;
                        _sol[1] = trst11;
                        _sol[2] = trst12;
                        func.Set(_param);
                    }
                    else if (recadrst2)
                    {
                        // ground on rst2
                        state = BlendStatus.OnRst2;
                        decroch = DecrochStatus.DecrochRst2;
                        _param = wrst2;
                        _sol[1] = trst21;
                        _sol[2] = trst22;
                        func.Set(_param);
                    }
                    //  it is checked on which curve the contact is lost earlier
                    else if (recadp1 && recadp2)
                    {
                        if (Math.Abs(wrst1 - wrst2) < _tolgui)
                        {
                            state = BlendStatus.OnRst12;
                            _param = 0.5 * (wrst1 + wrst2);
                            _sol[1] = trst11;
                            _sol[2] = trst22;
                        }
                        else if (_sens * (wrst1 - wrst2) < 0)
                        {
                            // ground on Rst1
                            state = BlendStatus.OnRst1;
                            _param = wrst1;
                            _sol[1] = trst11;
                            _sol[2] = trst12;
                        }
                        else
                        {
                            // ground on rst2
                            state = BlendStatus.OnRst2;
                            _param = wrst2;
                            _sol[1] = trst21;
                            _sol[2] = trst22;
                        }
                        func.Set(_param);
                    }
                    else if (recadp1)
                    {
                        // ground on rst1
                        state = BlendStatus.OnRst1;
                        _param = wrst1;
                        _sol[1] = trst11;
                        _sol[2] = trst12;
                        func.Set(_param);
                    }
                    else if (recadp2)
                    {
                        // ground on rst2
                        state = BlendStatus.OnRst2;
                        _param = wrst2;
                        _sol[1] = trst21;
                        _sol[2] = trst22;
                        func.Set(_param);
                    }
                    else
                    {
                        state = BlendStatus.OK;
                    }

                    state = TestArret(func, true, state);
                }
                else
                {
                    // reframing failed. Leave with PointsConfondus
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
                        _line.Append(_previousP);
                    }
                    else
                    {
                        _line.Prepend(_previousP);
                    }
                    parinit.Initialized(_sol);
                    parprec = _param;

                    if (_param == bound)
                    {
                        arrive = true;
                        extrst1.SetValue(_previousP.PointOnC1(), _previousP.ParameterOnC1(), _previousP.Parameter(),
                                         _tolpoint3d);
                        MakeExtremity(ref extrst2, false, _rst2, _sol[2], isVtxrst2, vtxrst2);
                        // Show that end is on Bound.
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
                        extrst1.SetValue(_previousP.PointOnC1(), _previousP.ParameterOnC1(), _previousP.Parameter(),
                                         _tolpoint3d);
                        extrst2.SetValue(_previousP.PointOnC2(), _previousP.ParameterOnC2(), _previousP.Parameter(),
                                         _tolpoint3d);
                        arrive = true;
                    }
                    else
                    {
                        _param = parprec + _sens * stepw; // there is no risk to exceed Bound.
                    }
                }
                break;

                case BlendStatus.StepTooSmall:
                {
                    // OCCT_DEBUG Drawsect left out.
                    // Update the line.
                    if (_sens > 0.0)
                    {
                        _line.Append(_previousP);
                    }
                    else
                    {
                        _line.Prepend(_previousP);
                    }
                    parinit.Initialized(_sol);
                    parprec = _param;

                    stepw = Min(1.5 * stepw, _pasmax);
                    if (_param == bound)
                    {
                        arrive = true;
                        extrst1.SetValue(_previousP.PointOnC1(), _previousP.ParameterOnC1(), _previousP.Parameter(),
                                         _tolpoint3d);
                        MakeExtremity(ref extrst2, false, _rst2, _sol[2], isVtxrst2, vtxrst2);
                        // Indicate that end is on Bound.
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
                        _line.Append(_previousP);
                    }
                    else
                    {
                        _line.Prepend(_previousP);
                    }
                    MakeExtremity(ref extrst1, true, _rst1, _sol[1], isVtxrst1, vtxrst1);
                    MakeExtremity(ref extrst2, false, _rst2, _sol[2], isVtxrst2, vtxrst2);
                    arrive = true;
                }
                break;

                case BlendStatus.OnRst2:
                {
                    // OCCT_DEBUG Drawsect left out.
                    if (_sens > 0.0)
                    {
                        _line.Append(_previousP);
                    }
                    else
                    {
                        _line.Prepend(_previousP);
                    }

                    MakeExtremity(ref extrst1, true, _rst1, _sol[1], isVtxrst1, vtxrst1);
                    MakeExtremity(ref extrst2, false, _rst2, _sol[2], isVtxrst2, vtxrst2);
                    arrive = true;
                }
                break;

                case BlendStatus.OnRst12:
                {
                    // OCCT_DEBUG Drawsect left out.
                    if (_sens > 0.0)
                    {
                        _line.Append(_previousP);
                    }
                    else
                    {
                        _line.Prepend(_previousP);
                    }

                    MakeExtremity(ref extrst1, true, _rst1, _sol[1], isVtxrst1, vtxrst1);
                    MakeExtremity(ref extrst2, false, _rst2, _sol[2], isVtxrst2, vtxrst2);
                    arrive = true;
                }
                break;

                case BlendStatus.SamePoints:
                {
                    // Stop
                    extrst1.SetValue(_previousP.PointOnC1(), _previousP.ParameterOnC1(), _previousP.Parameter(),
                                     _tolpoint3d);
                    extrst2.SetValue(_previousP.PointOnC2(), _previousP.ParameterOnC2(), _previousP.Parameter(),
                                     _tolpoint3d);
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
                    _line.SetEndPoints(extrst1, extrst2);
                    _decrochfin = decroch;
                }
                else
                {
                    _line.SetStartPoints(extrst1, extrst2);
                    _decrochdeb = decroch;
                }
            }
        }
    }

    // IsVtx and Vtx are written only on success (OCCT's references), so they are ref parameters.
    private bool Recadre1(RstRstFunction func, SurfCurvFuncInv finv, math_Vector solinv, ref bool isVtx,
                          ref HVertex? vtx)
    {
        math_Vector toler = new(1, 3), infb = new(1, 3), supb = new(1, 3);
        finv.GetTolerance(toler, _tolpoint3d);
        finv.GetBounds(infb, supb);
        solinv[1] = _param;
        solinv[2] = _sol[2];
        solinv[3] = _sol[1];

        // The point where contact is not lost is found
        FunctionSetRoot rsnld = new(finv, toler, 30);
        rsnld.Perform(finv, solinv, infb, supb);
        if (!rsnld.IsDone)
        {
            return false;
        }

        rsnld.Root(solinv);

        // It is necessary to check if the function value meets the second restriction
        if (finv.IsSolution(solinv, _tolpoint3d))
        {
            double w = solinv[2];
            if (w < _rst2.FirstParameter() - toler[2] || w > _rst2.LastParameter() + toler[2])
            {
                return false;
            }

            // it is checked if it is on a Vertex
            _domain1.Initialize(_rst1);
            _domain1.InitVertexIterator();
            isVtx = !_domain1.MoreVertex();
            while (!isVtx)
            {
                vtx = _domain1.Vertex();
                if (Math.Abs(BlendTool.Parameter(vtx, _rst1) - solinv[3])
                    <= BlendTool.Tolerance(vtx, _rst1))
                {
                    isVtx = true;
                }
                else
                {
                    _domain1.NextVertex();
                    isVtx = !_domain1.MoreVertex();
                }
            }
            if (!_domain1.MoreVertex())
            {
                isVtx = false;
            }
            // The section is recalculated by direct solution, otherwise return incoherences between the parameter
            // and the ground caused by yawn.

            math_Vector infbound = new(1, 2), supbound = new(1, 2);
            math_Vector parinit = new(1, 2), tolerance = new(1, 2);
            func.GetTolerance(tolerance, _tolpoint3d);
            func.GetBounds(infbound, supbound);

            FunctionSetRoot rsnld2 = new(func, tolerance, 30);
            parinit[1] = solinv[3];
            parinit[2] = solinv[2];
            func.Set(solinv[1]);
            rsnld2.Perform(func, parinit, infbound, supbound);
            if (!rsnld2.IsDone)
            {
                return false;
            }
            rsnld2.Root(parinit);
            solinv[2] = parinit[2];
            solinv[3] = parinit[1];
            return true;
        }
        return false;
    }

    private bool Recadre2(RstRstFunction func, SurfCurvFuncInv finv, math_Vector solinv, ref bool isVtx,
                          ref HVertex? vtx)
    {
        math_Vector toler = new(1, 3), infb = new(1, 3), supb = new(1, 3);
        finv.GetTolerance(toler, _tolpoint3d);
        finv.GetBounds(infb, supb);
        solinv[1] = _param;
        solinv[2] = _sol[1];
        solinv[3] = _sol[2];

        FunctionSetRoot rsnld = new(finv, toler, 30);
        rsnld.Perform(finv, solinv, infb, supb);
        if (!rsnld.IsDone)
        {
            return false;
        }

        rsnld.Root(solinv);

        // It is necessary to check the value of the function
        if (finv.IsSolution(solinv, _tolpoint3d))
        {
            double w = solinv[2];
            if (w < _rst1.FirstParameter() - toler[2] || w > _rst1.LastParameter() + toler[2])
            {
                return false;
            }

            _domain2.Initialize(_rst2);
            _domain2.InitVertexIterator();
            isVtx = !_domain2.MoreVertex();
            while (!isVtx)
            {
                vtx = _domain2.Vertex();
                if (Math.Abs(BlendTool.Parameter(vtx, _rst2) - solinv[3])
                    <= BlendTool.Tolerance(vtx, _rst2))
                {
                    isVtx = true;
                }
                else
                {
                    _domain2.NextVertex();
                    isVtx = !_domain2.MoreVertex();
                }
            }
            if (!_domain2.MoreVertex())
            {
                isVtx = false;
            }
            // The section is recalculated by direct solution, otherwise return incoherences between the parameter
            // and the ground caused by yawn.

            math_Vector infbound = new(1, 2), supbound = new(1, 2);
            math_Vector parinit = new(1, 2), tolerance = new(1, 2);
            func.GetTolerance(tolerance, _tolpoint3d);
            func.GetBounds(infbound, supbound);

            FunctionSetRoot rsnld2 = new(func, tolerance, 30);
            parinit[1] = solinv[2];
            parinit[2] = solinv[3];
            func.Set(solinv[1]);
            rsnld2.Perform(func, parinit, infbound, supbound);
            if (!rsnld2.IsDone)
            {
                return false;
            }
            rsnld2.Root(parinit);
            solinv[2] = parinit[1];
            solinv[3] = parinit[2];
            return true;
        }
        return false;
    }

    private bool Recadre1(CurvPointFuncInv finvP, math_Vector solinv, ref bool isVtx, ref HVertex? vtx)
    {
        // One is located on the last or the first point, following the direction of processing.
        Point2D p2drst1;
        double firstrst1 = _rst1.FirstParameter();
        double lastrst1 = _rst1.LastParameter();
        double upoint = firstrst1;

        if ((_sol[1] - firstrst1) > (lastrst1 - _sol[1]))
        {
            upoint = lastrst1;
        }
        p2drst1 = _rst1.Value(upoint);
        Point3D thepoint = _surf1.Value(p2drst1.X, p2drst1.Y);

        finvP.Set(thepoint);
        math_Vector toler = new(1, 2), infb = new(1, 2), supb = new(1, 2);
        finvP.GetTolerance(toler, _tolpoint3d);
        finvP.GetBounds(infb, supb);
        solinv[1] = _param;
        solinv[2] = _sol[2];

        FunctionSetRoot rsnld = new(finvP, toler, 30);
        rsnld.Perform(finvP, solinv, infb, supb);
        if (!rsnld.IsDone)
        {
            return false;
        }
        rsnld.Root(solinv);

        if (finvP.IsSolution(solinv, _tolpoint3d))
        {
            Point2D p2drst2 = _rst2.Value(solinv[2]);
            Containment situ = _domain2.Classify(p2drst2, toler[2], false);
            if ((situ != global::Kerf.Topology.Containment.In) && (situ != On))
            {
                return false;
            }
            _domain1.Initialize(_rst1);
            _domain1.InitVertexIterator();
            isVtx = !_domain1.MoreVertex();
            while (!isVtx)
            {
                vtx = _domain1.Vertex();
                if (Math.Abs(BlendTool.Parameter(vtx, _rst1) - upoint)
                    <= BlendTool.Tolerance(vtx, _rst1))
                {
                    isVtx = true;
                }
                else
                {
                    _domain1.NextVertex();
                    isVtx = !_domain1.MoreVertex();
                }
            }
            if (!_domain1.MoreVertex())
            {
                isVtx = false;
            }
            return true;
        }
        return false;
    }

    private bool Recadre2(CurvPointFuncInv finvP, math_Vector solinv, ref bool isVtx, ref HVertex? vtx)
    {
        // One is located on the last or the first point, following the direction of processing.
        Point2D p2drst2;
        double firstrst2 = _rst2.FirstParameter();
        double lastrst2 = _rst2.LastParameter();
        double vpoint = firstrst2;

        if ((_sol[2] - firstrst2) > (lastrst2 - _sol[2]))
        {
            vpoint = lastrst2;
        }
        p2drst2 = _rst2.Value(vpoint);
        Point3D thepoint = _surf2.Value(p2drst2.X, p2drst2.Y);

        finvP.Set(thepoint);
        math_Vector toler = new(1, 2), infb = new(1, 2), supb = new(1, 2);
        finvP.GetTolerance(toler, _tolpoint3d);
        finvP.GetBounds(infb, supb);
        solinv[1] = _param;
        solinv[2] = _sol[1];

        FunctionSetRoot rsnld = new(finvP, toler, 30);
        rsnld.Perform(finvP, solinv, infb, supb);
        if (!rsnld.IsDone)
        {
            return false;
        }
        rsnld.Root(solinv);

        if (finvP.IsSolution(solinv, _tolpoint3d))
        {
            Point2D p2drst1 = _rst1.Value(solinv[2]);
            Containment situ = _domain1.Classify(p2drst1, toler[2], false);
            if ((situ != global::Kerf.Topology.Containment.In) && (situ != On))
            {
                return false;
            }
            _domain2.Initialize(_rst2);
            _domain2.InitVertexIterator();
            isVtx = !_domain2.MoreVertex();
            while (!isVtx)
            {
                vtx = _domain2.Vertex();
                if (Math.Abs(BlendTool.Parameter(vtx, _rst2) - vpoint)
                    <= BlendTool.Tolerance(vtx, _rst2))
                {
                    isVtx = true;
                }
                else
                {
                    _domain2.NextVertex();
                    isVtx = !_domain2.MoreVertex();
                }
            }
            if (!_domain2.MoreVertex())
            {
                isVtx = false;
            }
            return true;
        }
        return false;
    }

    private void Transition(bool onFirst, Curve2DAdaptorBase arc, double param, ref WalkingTransition tLine,
                            ref WalkingTransition tArc)
    {
        bool computetranstionaveclacorde = false;
        Vector3D tgline;
        BlendPoint prevprev = new();

        if (_previousP.IsTangencyPoint)
        {
            if (_line!.NbPoints() < 2)
            {
                return;
            }
            computetranstionaveclacorde = true;
            if (_sens < 0)
            {
                prevprev = _line.Point(2);
            }
            else
            {
                prevprev = _line.Point(_line.NbPoints() - 1);
            }
        }
        Point2D p2d;
        Vector2D dp2d;

        Point3D pbid;
        Vector3D d1u, d1v, normale, tgrst = new();

        arc.D1(param, out p2d, out dp2d);
        if (onFirst)
        {
            _surf1.D1(p2d.X, p2d.Y, out pbid, out d1u, out d1v);
            if (!computetranstionaveclacorde)
            {
                tgline = _previousP.TangentOnC1();
            }
            else
            {
                tgline = new Vector3D(prevprev.PointOnC1(), _previousP.PointOnC1());
            }
        }
        else
        {
            _surf2.D1(p2d.X, p2d.Y, out pbid, out d1u, out d1v);
            if (!computetranstionaveclacorde)
            {
                tgline = _previousP.TangentOnC2();
            }
            else
            {
                tgline = new Vector3D(prevprev.PointOnC2(), _previousP.PointOnC2());
            }
        }

        tgrst = Vector3D.LinearForm(dp2d.X, d1u, dp2d.Y, d1v);
        normale = d1u.Crossed(d1v);

        global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(tgline, tgrst, new Direction3D(normale), ref tLine, ref tArc);
    }

    // Produces the extremity of a curve.
    private void MakeExtremity(ref Extremity extrem, bool onFirst, Curve2DAdaptorBase Arc, double param,
                               bool isVtx, HVertex? vtx)
    {
        WalkingTransition tline = new(), tarc = new();
        double prm;
        TopolTool iter;
        if (onFirst)
        {
            extrem.SetValue(_previousP.PointOnC1(), _sol[1], _previousP.Parameter(), _tolpoint3d);
            if (!_previousP.IsTangencyPoint)
            {
                extrem.SetTangent(_previousP.TangentOnC1());
            }
            iter = _domain1;
        }
        else
        {
            extrem.SetValue(_previousP.PointOnC2(), _sol[2], _previousP.Parameter(), _tolpoint3d);
            if (!_previousP.IsTangencyPoint)
            {
                extrem.SetTangent(_previousP.TangentOnC1()); // OCCT: the first curve's tangent on the second's extremity
            }
            iter = _domain2;
        }

        iter.Init();
        if (!isVtx)
        {
            Transition(onFirst, Arc, param, ref tline, ref tarc);
            extrem.AddArc(Arc, param, tline, tarc);
        }
        else
        {
            extrem.SetVertex(vtx!);
            while (iter.More())
            {
                Curve2DAdaptorBase arc = iter.Value();
                if (!ReferenceEquals(arc, Arc))
                {
                    iter.Initialize(arc);
                    iter.InitVertexIterator();
                    while (iter.MoreVertex())
                    {
                        if (iter.Identical(vtx!, iter.Vertex()))
                        {
                            prm = BlendTool.Parameter(vtx!, arc);
                            Transition(onFirst, arc, prm, ref tline, ref tarc);
                            extrem.AddArc(arc, prm, tline, tarc);
                        }
                        iter.NextVertex();
                    }
                }
                else
                {
                    Transition(onFirst, arc, param, ref tline, ref tarc);
                    extrem.AddArc(arc, param, tline, tarc);
                }
                iter.Next();
            }
        }
    }

    private BlendStatus CheckDeflectionOnRst1(in BlendPoint curPoint)
    {
        // Controls 3d of Blend_CSWalking.

        // rule by tests in U4 corresponds to 11.478
        const double cosRef3D = 0.98;
        double cosi, cosi2;
        bool curpointistangent = curPoint.IsTangencyPoint;
        bool prevpointistangent = _previousP.IsTangencyPoint;

        Point3D psurf = curPoint.PointOnC1();
        Vector3D tgsurf = new();
        if (!curpointistangent)
        {
            tgsurf = curPoint.TangentOnC1();
        }
        Point3D prevP = _previousP.PointOnC1();
        Vector3D prevTg = new();
        if (!prevpointistangent)
        {
            prevTg = _previousP.TangentOnC1();
        }
        double norme;
        double prevNorme = 0.0;
        Vector3D corde = new(prevP, psurf);
        norme = corde.SquareMagnitude;
        if (!prevpointistangent)
        {
            prevNorme = prevTg.SquareMagnitude;
        }

        double toler3d = 0.01 * _tolpoint3d;
        if (norme <= toler3d * toler3d)
        {
            // it can be necessary to force the same point
            return BlendStatus.SamePoints;
        }
        if (!prevpointistangent)
        {
            if (prevNorme <= toler3d * toler3d)
            {
                return BlendStatus.SamePoints;
            }
            cosi = _sens * corde * prevTg;
            if (cosi < 0.0)
            { // angle 3d>pi/2. --> return back
                return BlendStatus.Backward;
            }

            cosi2 = cosi * cosi / prevNorme / norme;
            if (cosi2 < cosRef3D)
            {
                return BlendStatus.StepTooLarge;
            }
        }

        if (!curpointistangent)
        {
            // Check if it is necessary to control the sign of prevtg*Tgsurf
            cosi = _sens * corde * tgsurf;
            cosi2 = cosi * cosi / tgsurf.SquareMagnitude / norme;
            if (cosi2 < cosRef3D || cosi < 0.0)
            {
                return BlendStatus.StepTooLarge;
            }
        }

        if (!curpointistangent && !prevpointistangent)
        {
            // Estimation of the current arrow
            double flecheCourante =
                (prevTg.Normalized().XYZ() - tgsurf.Normalized().XYZ()).SquareModulus() * norme / 64.0;

            if (flecheCourante <= 0.25 * _fleche * _fleche)
            {
                return BlendStatus.StepTooSmall;
            }
            if (flecheCourante > _fleche * _fleche)
            {
                // not too great
                return BlendStatus.StepTooLarge;
            }
        }
        return BlendStatus.OK;
    }

    private BlendStatus CheckDeflectionOnRst2(in BlendPoint curPoint)
    {
        // 3D Controls of Blend_CSWalking.

        // rule by tests in U4 corresponding to 11.478 d
        const double cosRef3D = 0.98;
        double cosi, cosi2;
        bool curpointistangent = curPoint.IsTangencyPoint;
        bool prevpointistangent = _previousP.IsTangencyPoint;

        Point3D psurf = curPoint.PointOnC2();
        Vector3D tgsurf = new();

        if (!curpointistangent)
        {
            tgsurf = curPoint.TangentOnC2();
        }
        Point3D prevP = _previousP.PointOnC2();
        Vector3D prevTg = new();
        if (!prevpointistangent)
        {
            prevTg = _previousP.TangentOnC2();
        }
        double norme;
        double prevNorme = 0.0;
        Vector3D corde = new(prevP, psurf);
        norme = corde.SquareMagnitude;
        if (!prevpointistangent)
        {
            prevNorme = prevTg.SquareMagnitude;
        }

        double toler3d = 0.01 * _tolpoint3d;
        if (norme <= toler3d * toler3d)
        {
            // it can be necessary to force the same point
            return BlendStatus.SamePoints;
        }
        if (!prevpointistangent)
        {
            if (prevNorme <= toler3d * toler3d)
            {
                return BlendStatus.SamePoints;
            }
            cosi = _sens * corde * prevTg;
            if (cosi < 0.0)
            { // angle 3d>pi/2. --> return back
                return BlendStatus.Backward;
            }

            cosi2 = cosi * cosi / prevNorme / norme;
            if (cosi2 < cosRef3D)
            {
                return BlendStatus.StepTooLarge;
            }
        }

        if (!curpointistangent)
        {
            // Check if it is necessary to control the sign of prevtg*Tgsurf
            cosi = _sens * corde * tgsurf;
            cosi2 = cosi * cosi / tgsurf.SquareMagnitude / norme;
            if (cosi2 < cosRef3D || cosi < 0.0)
            {
                return BlendStatus.StepTooLarge;
            }
        }

        if (!curpointistangent && !prevpointistangent)
        {
            // Estimation of the current arrow
            double flecheCourante =
                (prevTg.Normalized().XYZ() - tgsurf.Normalized().XYZ()).SquareModulus() * norme / 64.0;

            if (flecheCourante <= 0.25 * _fleche * _fleche)
            {
                return BlendStatus.StepTooSmall;
            }
            if (flecheCourante > _fleche * _fleche)
            {
                // not too great
                return BlendStatus.StepTooLarge;
            }
        }
        return BlendStatus.OK;
    }

    private static WalkingTypeTrans ConvOrToTra(Orientation o)
    {
        if (o == Forward)
        {
            return WalkingTypeTrans.In;
        }
        return WalkingTypeTrans.Out;
    }

    private BlendStatus TestArret(RstRstFunction func, bool testDeflection, BlendStatus state)
    {
        Point3D ptrst1, ptrst2;
        Point2D pt2drst1, pt2drst2;
        Vector3D tgrst1, tgrst2;
        Vector2D tg2drst1 = new(), tg2drst2 = new();
        BlendStatus stateRst1, stateRst2;
        WalkingTypeTrans trarst1 = WalkingTypeTrans.Undecided, trarst2 = WalkingTypeTrans.Undecided;
        BlendPoint curpoint = new();

        if (func.IsSolution(_sol, _tolpoint3d))
        {
            bool curpointistangent = func.IsTangencyPoint;
            ptrst1 = func.PointOnRst1();
            ptrst2 = func.PointOnRst2();
            pt2drst1 = func.Pnt2dOnRst1();
            pt2drst2 = func.Pnt2dOnRst2();

            if (curpointistangent)
            {
                curpoint.SetValue(ptrst1, ptrst2, _param, pt2drst1.X, pt2drst1.Y, pt2drst2.X, pt2drst2.Y, _sol[1],
                                  _sol[2]);
            }
            else
            {
                tgrst1 = func.TangentOnRst1();
                tgrst2 = func.TangentOnRst2();
                tg2drst1 = func.Tangent2dOnRst1();
                tg2drst2 = func.Tangent2dOnRst2();
                curpoint.SetValue(ptrst1, ptrst2, _param, pt2drst1.X, pt2drst1.Y, pt2drst2.X, pt2drst2.Y, _sol[1],
                                  _sol[2], tgrst1, tgrst2, tg2drst1, tg2drst2);
            }
            if (testDeflection)
            {
                stateRst1 = CheckDeflectionOnRst1(curpoint);
                stateRst2 = CheckDeflectionOnRst2(curpoint);
            }
            else
            {
                stateRst1 = stateRst2 = BlendStatus.OK;
            }
            if (stateRst1 == BlendStatus.Backward)
            {
                stateRst1 = BlendStatus.StepTooLarge;
            }
            if (stateRst2 == BlendStatus.Backward)
            {
                stateRst2 = BlendStatus.StepTooLarge;
            }
            if (stateRst1 == BlendStatus.StepTooLarge || stateRst2 == BlendStatus.StepTooLarge)
            {
                return BlendStatus.StepTooLarge;
            }

            if (!_comptra && !curpointistangent)
            {
                Point2D p2drstref;
                Vector2D tg2drstref;
                _rst1.D1(_sol[1], out p2drstref, out tg2drstref);
                double testra = tg2drst1.Dot(tg2drstref);
                Orientation or = _domain1.Orientation(_rst1);

                if (Math.Abs(testra) > _tolpoint3d)
                {
                    if (testra < 0.0)
                    {
                        trarst1 = ConvOrToTra(global::Kerf.Engine.Topology.TopAbs.Reverse(or));
                    }
                    else if (testra > 0.0)
                    {
                        trarst1 = ConvOrToTra(or);
                    }

                    _rst2.D1(_sol[2], out p2drstref, out tg2drstref);
                    testra = tg2drst2.Dot(tg2drstref);

                    or = _domain2.Orientation(_rst2);
                    if (Math.Abs(testra) > _tolpoint3d)
                    {
                        if (testra < 0.0)
                        {
                            trarst2 = ConvOrToTra(global::Kerf.Engine.Topology.TopAbs.Reverse(or));
                        }
                        else if (testra > 0.0)
                        {
                            trarst2 = ConvOrToTra(or);
                        }
                        _comptra = true;
                        _line!.Set(trarst1, trarst2);
                    }
                }
            }
            if (stateRst1 == BlendStatus.OK || stateRst2 == BlendStatus.OK)
            {
                _previousP = curpoint;
                return state;
            }
            if (stateRst1 == BlendStatus.StepTooSmall && stateRst2 == BlendStatus.StepTooSmall)
            {
                _previousP = curpoint;
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
        return BlendStatus.StepTooLarge;
    }

    private bool CheckInside(RstRstFunction func, out Containment situOnC1, out Containment situOnC2,
                             out DecrochStatus decroch)
    {
        math_Vector tolerance = new(1, 2);
        func.GetTolerance(tolerance, _tolpoint3d);

        // face pcurve 1.
        double v = _sol[1];
        if (v < _rst1.FirstParameter() - tolerance[2] || v > _rst1.LastParameter() + tolerance[2])
        {
            situOnC1 = global::Kerf.Topology.Containment.Out;
        }
        else if (v > _rst1.FirstParameter() && v < _rst1.LastParameter())
        {
            situOnC1 = global::Kerf.Topology.Containment.In;
        }
        else
        {
            situOnC1 = On;
        }

        // face pcurve 2.
        v = _sol[2];
        if (v < _rst2.FirstParameter() - tolerance[2] || v > _rst2.LastParameter() + tolerance[2])
        {
            situOnC2 = global::Kerf.Topology.Containment.Out;
        }
        else if (v > _rst2.FirstParameter() && v < _rst2.LastParameter())
        {
            situOnC2 = global::Kerf.Topology.Containment.In;
        }
        else
        {
            situOnC2 = On;
        }

        // lost contact
        Vector3D tgrst1, norst1, tgrst2, norst2;
        decroch = func.Decroch(_sol, out tgrst1, out norst1, out tgrst2, out norst2);

        return (situOnC1 == global::Kerf.Topology.Containment.In && situOnC2 == global::Kerf.Topology.Containment.In && decroch == DecrochStatus.NoDecroch);
    }
}
