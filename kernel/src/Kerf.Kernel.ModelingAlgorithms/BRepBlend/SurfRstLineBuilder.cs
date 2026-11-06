// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstLineBuilder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstLineBuilder.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstLineBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
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
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// Walks a blend between a surface and a restriction (an arc of another surface) along a guide, building a
/// <see cref="Line"/>. The walking stops where the section leaves the surface's face (the surface/curve
/// inverse function on its arcs), the restriction's ends (the surface/point inverse function) or loses contact with
/// the restriction's surface (the inverse function on the restriction, "décroché").
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT's <c>OCCT_DEBUG</c> drawing of the sections and its derivative traces are
/// left out.
/// </remarks>
internal class SurfRstLineBuilder
{
    private bool _done;
    private Line? _line;
    private readonly math_Vector _sol;
    private readonly SurfaceAdaptorBase _surf1;
    private readonly TopolTool _domain1;
    private readonly SurfaceAdaptorBase _surf2;
    private readonly Curve2DAdaptorBase _rst;
    private readonly TopolTool _domain2;
    private double _tolpoint3d;
    private double _tolpoint2d;
    private double _tolgui;
    private double _pasmax;
    private double _fleche;
    private double _param;
    private BlendPoint _previousP = new();
    private bool _iscomplete;
    private bool _comptra;
    private double _sens;
    private bool _decrochdeb;
    private bool _decrochfin;

    /// <summary>
    /// Finds the arc of the first surface's domain that the section <paramref name="sol"/> leaves through: by
    /// intersection with the segment from the previous point, else by projection. <paramref name="prevIndex"/> is an
    /// arc already tried, rejected. Returns the arc's index (0 if none) and its parameter in <paramref name="ponarc"/>.
    /// </summary>
    // OCCT: the header names the points (pt2d, lastpt2d), the definition (lastpt2d, pt2d); the definition's order here.
    public int ArcToRecadre(math_Vector sol, int prevIndex, out Point2D lastpt2d, out Point2D pt2d,
                            ref double ponarc)
    {
        int indexSol = 0, nbarc = 0;
        bool ok = false;
        bool byinter = (_line!.NbPoints() != 0), okinter = false;
        double distmin = RealLast();
        double uprev = 0.0, vprev = 0.0, prm = 0.0, dist = 0.0;

        if (byinter)
        {
            _previousP.ParametersOnS(out uprev, out vprev);
        }
        pt2d = new Point2D(sol[1], sol[2]);
        lastpt2d = new Point2D(uprev, vprev);
        _domain1.Init();

        while (_domain1.More())
        {
            nbarc++;
            ok = false;
            if (byinter)
            {
                ok = okinter = BlendTool.Inters(pt2d, lastpt2d, _surf1, _domain1.Value(), ref prm, ref dist);
            }
            if (!ok)
            {
                ok = BlendTool.Project(pt2d, _surf1, _domain1.Value(), out prm, out dist);
            }

            if (ok && (nbarc != prevIndex))
            {
                if (dist < distmin || okinter)
                {
                    distmin = dist;
                    ponarc = prm;
                    indexSol = nbarc;
                    if (okinter && (prevIndex == 0))
                    {
                        break;
                    }
                }
            }
            _domain1.Next();
        }
        return indexSol;
    }

    public SurfRstLineBuilder(SurfaceAdaptorBase surf1, TopolTool domain1, SurfaceAdaptorBase surf2,
                                        Curve2DAdaptorBase rst, TopolTool domain2)
    {
        _done = false;
        _sol = new math_Vector(1, 3);
        _surf1 = surf1;
        _domain1 = domain1;
        _surf2 = surf2;
        _rst = rst;
        _domain2 = domain2;
        _tolpoint3d = 0.0;
        _tolpoint2d = 0.0;
        _tolgui = 0.0;
        _pasmax = 0.0;
        _fleche = 0.0;
        _param = 0.0;
        _iscomplete = false;
        _comptra = false;
        _sens = 0.0;
        _decrochdeb = false;
        _decrochfin = false;
    }

    /// <summary>
    /// Walks from the guide's parameter <paramref name="pdep"/> towards <paramref name="pmax"/>, starting from the
    /// solution <paramref name="parDep"/> (solved again first when <paramref name="appro"/>).
    /// </summary>
    public void Perform(SurfRstFunction func, FuncInv finv, SurfPointFuncInv finvP,
                        SurfCurvFuncInv finvC, double pdep, double pmax, double maxStep, double tol3d,
                        double tol2d, double tolGuide, math_Vector parDep, double fleche, bool appro = false)
    {
        _done = false;
        _iscomplete = false;
        _comptra = false;
        _line = new Line();
        _tolpoint3d = tol3d;
        _tolpoint2d = tol2d;
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
            Containment siturst, situs;
            bool decroch;
            math_Vector tolerance = new(1, 3), infbound = new(1, 3), supbound = new(1, 3);
            func.GetTolerance(tolerance, _tolpoint3d);
            func.GetBounds(infbound, supbound);
            FunctionSetRoot rsnld = new(func, tolerance, 30);

            rsnld.Perform(func, parDep, infbound, supbound);

            if (!rsnld.IsDone)
            {
                return;
            }
            rsnld.Root(_sol);
            if (!CheckInside(func, out siturst, out situs, out decroch))
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
        _previousP.ParametersOnS(out u, out v);

        Extremity ptf1 = new(_previousP.PointOnS(), u, v, _previousP.Parameter(), _tolpoint3d);
        Extremity ptf2 = new(_previousP.PointOnC(), u, v, _previousP.Parameter(), _tolpoint3d); // OCCT: the surface's (U, V)
        if (!_previousP.IsTangencyPoint)
        {
            ptf1.SetTangent(_previousP.TangentOnS());
            ptf2.SetTangent(_previousP.TangentOnC());
        }
        if (_sens > 0.0)
        {
            _line.SetStartPoints(ptf1, ptf2);
        }
        else
        {
            _line.SetEndPoints(ptf1, ptf2);
        }
        InternalPerform(func, finv, finvP, finvC, pmax);
        _done = true;
    }

    /// <summary>
    /// Solves a first section at <paramref name="pdep"/> and moves it to where the walking stops (the face's
    /// boundary, the restriction's ends or a lost contact): <paramref name="psol"/> and <paramref name="parSol"/>
    /// receive it.
    /// </summary>
    public bool PerformFirstSection(SurfRstFunction func, FuncInv finv, SurfPointFuncInv finvP,
                                    SurfCurvFuncInv finvC, double pdep, double pmax, math_Vector parDep,
                                    double tol3d, double tol2d, double tolGuide, bool recRst, bool recP, bool recS,
                                    ref double psol, math_Vector parSol)
    {
        _done = false;
        _iscomplete = false;
        _comptra = false;
        _line = new Line();
        _tolpoint3d = tol3d;
        _tolpoint2d = tol2d;
        _tolgui = Math.Abs(tolGuide);

        if (pmax - pdep >= 0.0)
        {
            _sens = 1.0;
        }
        else
        {
            _sens = -1.0;
        }
        BlendStatus state = BlendStatus.OnRst12;
        double trst = 0.0;
        bool recadp, recadrst, recads;
        double wp, wrst, ws;
        double u = 0.0, v = 0.0;
        math_Vector infbound = new(1, 3), supbound = new(1, 3), tolerance = new(1, 3);
        math_Vector solinvp = new(1, 3), solinvrst = new(1, 4), solinvs = new(1, 3);
        HVertex? vtxp = null, vtxrst = null, vtxs = null;
        bool isVtxp = false, isVtxrst = false, isVtxs = false;
        Curve2DAdaptorBase? arc = null;
        wp = wrst = ws = pmax;
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

        recads = recS && Recadre(finvC, solinvs, ref arc, ref isVtxs, ref vtxs);
        if (recads)
        {
            ws = solinvs[1];
        }
        recadp = recP && Recadre(finvP, solinvp, ref isVtxp, ref vtxp);
        if (recadp)
        {
            wp = solinvp[1];
        }
        recadrst = recRst && Recadre(func, finv, solinvrst, ref isVtxrst, ref vtxrst);
        if (recadrst)
        {
            wrst = solinvrst[2];
        }
        if (!recads && !recadp && !recadrst)
        {
            return false;
        }
        if (recadp && recadrst)
        {
            if (_sens * (wrst - wp) > _tolgui)
            { // first one leaves the domain
                wrst = wp;
                u = solinvp[2];
                v = solinvp[3];
                trst = BlendTool.Parameter(vtxp!, _rst);
                isVtxrst = isVtxp;
                vtxrst = vtxp;
            }
            else
            {
                u = solinvrst[3];
                v = solinvrst[4];
                trst = solinvrst[1];
            }
        }
        else if (recadp)
        {
            wrst = wp;
            u = solinvp[2];
            v = solinvp[3];
            trst = BlendTool.Parameter(vtxp!, _rst);
            isVtxrst = isVtxp;
            vtxrst = vtxp;
            recadrst = true;
        }
        else if (recadrst)
        {
            u = solinvrst[3];
            v = solinvrst[4];
            trst = solinvrst[1];
        }
        if (recads && recadrst)
        {
            if (Math.Abs(ws - wrst) < _tolgui)
            {
                state = BlendStatus.OnRst12;
                _param = 0.5 * (ws + wrst);
                _sol[1] = u;
                _sol[2] = v;
                _sol[3] = solinvs[2];
            }
            else if (_sens * (ws - wrst) < 0)
            {
                // ground on surf
                state = BlendStatus.OnRst1;
                _param = ws;
                arc!.Value(solinvs[3]).Coord(out u, out v);
                _sol[1] = u;
                _sol[2] = v;
                _sol[3] = solinvs[2];
            }
            else
            {
                // ground on rst
                state = BlendStatus.OnRst2;
                _param = wrst;
                _sol[1] = u;
                _sol[2] = v;
                _sol[3] = trst;
            }
            func.Set(_param);
        }
        else if (recads)
        {
            // ground on surf
            state = BlendStatus.OnRst1;
            _param = ws;
            arc!.Value(solinvs[3]).Coord(out u, out v);
            _sol[1] = u;
            _sol[2] = v;
            _sol[3] = solinvs[2];
            func.Set(_param);
        }
        else if (recadrst)
        {
            // ground on rst
            state = BlendStatus.OnRst2;
            _param = wrst;
            _sol[1] = u;
            _sol[2] = v;
            _sol[3] = trst;
            func.Set(_param);
        }
        state = TestArret(func, false, state);
        psol = _param;
        parSol.Initialized(_sol);
        return true;
    }

    /// <summary>Walks the other way from the line's start, down to <paramref name="pmin"/>.</summary>
    public bool Complete(SurfRstFunction func, FuncInv finv, SurfPointFuncInv finvP,
                         SurfCurvFuncInv finvC, double pmin)
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
        _previousP.ParametersOnS(out _sol[1], out _sol[2]);
        _sol[3] = _previousP.ParameterOnC();

        InternalPerform(func, finv, finvP, finvC, pmin);
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

    public bool DecrochStart() => _decrochdeb;

    public bool DecrochEnd() => _decrochfin;

    private void InternalPerform(SurfRstFunction func, FuncInv finv, SurfPointFuncInv finvP,
                                 SurfCurvFuncInv finvC, double bound)
    {
        double stepw = _pasmax;
        int nbp = _line!.NbPoints();
        if (nbp >= 2)
        { // The last step is reproduced if it is not too small.
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
        Containment situonc = Unknown, situons = Unknown;
        bool decroch = false;
        bool arrive, recadp, recadrst, recads, echecrecad;
        double wp, wrst, ws;
        double u = 0.0, v = 0.0;
        double trst = 0.0;
        math_Vector infbound = new(1, 3), supbound = new(1, 3);
        math_Vector parinit = new(1, 3), tolerance = new(1, 3);
        math_Vector solinvp = new(1, 3), solinvrst = new(1, 4), solinvs = new(1, 3);
        HVertex? vtxp = null, vtxrst = null, vtxs = null;
        bool isVtxp = false, isVtxrst = false, isVtxs = false;
        Extremity extrst = new(), exts = new();
        Curve2DAdaptorBase? arc = null;

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
                if (!CheckInside(func, out situonc, out situons, out decroch) && _line.NbPoints() == 1)
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
                wp = wrst = ws = bound;
                recadp = recadrst = recads = false;
                echecrecad = false;
                if (situons == global::Kerf.Topology.Containment.Out || situons == On)
                {
                    // pb inverse rst/rst
                    recads = Recadre(finvC, solinvs, ref arc, ref isVtxs, ref vtxs);
                    if (recads)
                    {
                        ws = solinvs[1];
                        // It is necessary to reevaluate the deviation (BUC60360)
                        Vector3D t, n;
                        func.Set(ws);
                        arc!.Value(solinvs[3]).Coord(out u, out v);
                        _sol[1] = u;
                        _sol[2] = v;
                        _sol[3] = solinvs[2];
                        decroch = func.Decroch(_sol, out t, out n);
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }
                if (situonc == global::Kerf.Topology.Containment.Out || situonc == On)
                {
                    // pb inverse point/surf
                    recadp = Recadre(finvP, solinvp, ref isVtxp, ref vtxp);
                    if (recadp)
                    {
                        wp = solinvp[1];
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }
                if (decroch)
                {
                    // pb inverse rst/surf
                    recadrst = Recadre(func, finv, solinvrst, ref isVtxrst, ref vtxrst);
                    if (recadrst)
                    {
                        wrst = solinvrst[2];
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }
                decroch = false;
                if (recadp || recads || recadrst)
                {
                    echecrecad = false;
                }
                if (!echecrecad)
                {
                    if (recadp && recadrst)
                    {
                        if (_sens * (wrst - wp) > _tolgui)
                        { // first one leaves the domain
                            wrst = wp;
                            u = solinvp[2];
                            v = solinvp[3];
                            trst = BlendTool.Parameter(vtxp!, _rst);
                            isVtxrst = isVtxp;
                            vtxrst = vtxp;
                        }
                        else
                        {
                            decroch = true;
                            u = solinvrst[3];
                            v = solinvrst[4];
                            trst = solinvrst[1];
                        }
                    }
                    else if (recadp)
                    {
                        wrst = wp;
                        u = solinvp[2];
                        v = solinvp[3];
                        trst = BlendTool.Parameter(vtxp!, _rst);
                        isVtxrst = isVtxp;
                        vtxrst = vtxp;
                        recadrst = true;
                    }
                    else if (recadrst)
                    {
                        decroch = true;
                        u = solinvrst[3];
                        v = solinvrst[4];
                        trst = solinvrst[1];
                    }
                    if (recads && recadrst)
                    {
                        if (Math.Abs(ws - wrst) < _tolgui)
                        {
                            state = BlendStatus.OnRst12;
                            _param = 0.5 * (ws + wrst);
                            _sol[1] = u;
                            _sol[2] = v;
                            _sol[3] = solinvs[3]; // OCCT: the arc's parameter where PerformFirstSection takes the restriction's (solinvs(2))
                        }
                        else if (_sens * (ws - wrst) < 0)
                        {
                            // ground on surf
                            decroch = false;
                            state = BlendStatus.OnRst1;
                            _param = ws;
                            arc!.Value(solinvs[3]).Coord(out u, out v);
                            _sol[1] = u;
                            _sol[2] = v;
                            _sol[3] = solinvs[2];
                        }
                        else
                        {
                            // ground on rst
                            state = BlendStatus.OnRst2;
                            _param = wrst;
                            _sol[1] = u;
                            _sol[2] = v;
                            _sol[3] = trst;
                        }
                        func.Set(_param);
                    }
                    else if (recads)
                    {
                        // ground on surf
                        state = BlendStatus.OnRst1;
                        _param = ws;
                        arc!.Value(solinvs[3]).Coord(out u, out v);
                        _sol[1] = u;
                        _sol[2] = v;
                        _sol[3] = solinvs[2];
                        func.Set(_param);
                    }
                    else if (recadrst)
                    {
                        // ground on rst
                        state = BlendStatus.OnRst2;
                        _param = wrst;
                        _sol[1] = u;
                        _sol[2] = v;
                        _sol[3] = trst;
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
                    // Failed reframing. Leave with PointsConfondus
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
                        exts.SetValue(_previousP.PointOnS(), _sol[1], _sol[2], _previousP.Parameter(), _tolpoint3d);
                        MakeExtremity(ref extrst, false, _rst, _sol[3], isVtxrst, vtxrst);
                        // Indicate end on Bound.
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
                        extrst.SetValue(_previousP.PointOnC(), _previousP.ParameterOnC(), _previousP.Parameter(),
                                        _tolpoint3d);
                        arrive = true;
                    }
                    else
                    {
                        _param = parprec + _sens * stepw; // no risk to exceed Bound.
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
                        exts.SetValue(_previousP.PointOnS(), _sol[1], _sol[2], _previousP.Parameter(), _tolpoint3d);
                        MakeExtremity(ref extrst, false, _rst, _sol[3], isVtxrst, vtxrst);
                        // Indicate end on Bound.
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
                    MakeExtremity(ref exts, true, arc!, solinvs[3], isVtxs, vtxs);
                    MakeExtremity(ref extrst, false, _rst, _sol[3], isVtxrst, vtxrst);
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
                    exts.SetValue(_previousP.PointOnS(), _sol[1], _sol[2], _previousP.Parameter(), _tolpoint3d);
                    MakeExtremity(ref extrst, false, _rst, _sol[3], isVtxrst, vtxrst);
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
                    MakeExtremity(ref exts, true, arc!, solinvs[1], isVtxs, vtxs);
                    MakeExtremity(ref extrst, false, _rst, _sol[3], isVtxrst, vtxrst);
                    arrive = true;
                }
                break;

                case BlendStatus.SamePoints:
                {
                    // Stop
                    _previousP.ParametersOnS(out u, out v);
                    exts.SetValue(_previousP.PointOnS(), u, v, _previousP.Parameter(), _tolpoint3d);
                    extrst.SetValue(_previousP.PointOnC(), _previousP.ParameterOnC(), _previousP.Parameter(),
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
                    _line.SetEndPoints(exts, extrst);
                    _decrochfin = decroch;
                }
                else
                {
                    _line.SetStartPoints(exts, extrst);
                    _decrochdeb = decroch;
                }
            }
        }
    }

    // Reframes the section surface / restriction on an arc of the surface's domain. Arc, IsVtx and Vtx are OCCT's
    // references (Arc and Vtx written only on the paths that find them).
    private bool Recadre(SurfCurvFuncInv finvC, math_Vector solinv, ref Curve2DAdaptorBase? arc, ref bool isVtx,
                         ref HVertex? vtx)
    {
        bool recadre = false;

        Point2D pt2d, lastpt2d;
        int indexSol, nbarc;
        double pmin = 0.0; // OCCT leaves it uninitialised; ArcToRecadre sets it whenever it finds an arc

        indexSol = ArcToRecadre(_sol, 0, out lastpt2d, out pt2d, ref pmin);

        isVtx = false;
        if (indexSol == 0)
        {
            return false;
        }

        _domain1.Init();
        nbarc = 1;
        while (nbarc < indexSol)
        {
            nbarc++;
            _domain1.Next();
        }
        arc = _domain1.Value();

        finvC.Set(arc);

        math_Vector toler = new(1, 3), infb = new(1, 3), supb = new(1, 3);
        // use reduced Tol argument value to pass testcase blend complex A6 with scale factor of model 0.1 (base
        // scale = 1000). So, here we using 1.0e-5 rather than 1.0e-4 of tolerance of point in 3d
        finvC.GetTolerance(toler, 0.1 * _tolpoint3d);
        finvC.GetBounds(infb, supb);
        solinv[1] = _param;
        solinv[2] = _sol[3];
        solinv[3] = pmin;

        FunctionSetRoot rsnld = new(finvC, toler, 30);
        rsnld.Perform(finvC, solinv, infb, supb);

        if (!rsnld.IsDone)
        {
            // OCCT_DEBUG trace left out.
        }
        else
        {
            // It is necessary to check the value of the function
            rsnld.Root(solinv);
            recadre = finvC.IsSolution(solinv, _tolpoint3d);
        }

        // In case of fail, it is checked if another arc can be useful (case of output at the proximity of a vertex)
        if (!recadre)
        {
            indexSol = ArcToRecadre(_sol, indexSol, out lastpt2d, out pt2d, ref pmin);
            if (indexSol == 0)
            {
                return false; // No other solution
            }

            _domain1.Init();
            nbarc = 1;
            while (nbarc < indexSol)
            {
                nbarc++;
                _domain1.Next();
            }

            arc = _domain1.Value();
            finvC.Set(arc);

            finvC.GetTolerance(toler, _tolpoint3d);
            finvC.GetBounds(infb, supb);

            solinv[3] = pmin;

            FunctionSetRoot aRsnld = new(finvC, toler, 30);
            aRsnld.Perform(finvC, solinv, infb, supb);

            if (!aRsnld.IsDone)
            {
                // OCCT_DEBUG trace left out.
            }
            else
            {
                // It is necessary to check the value of the function
                aRsnld.Root(solinv);
                recadre = finvC.IsSolution(solinv, _tolpoint3d);
            }
        }

        if (recadre)
        {
            double w = solinv[2];
            if (w < _rst.FirstParameter() - toler[2] || w > _rst.LastParameter() + toler[2])
            {
                return false;
            }
            _domain1.Initialize(arc);
            _domain1.InitVertexIterator();
            isVtx = !_domain1.MoreVertex();
            while (!isVtx)
            {
                vtx = _domain1.Vertex();
                if (Math.Abs(BlendTool.Parameter(vtx, arc) - solinv[3])
                    <= BlendTool.Tolerance(vtx, arc))
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

    private bool Recadre(SurfRstFunction func, FuncInv finv, math_Vector solinv, ref bool isVtx,
                         ref HVertex? vtx)
    {
        math_Vector toler = new(1, 4), infb = new(1, 4), supb = new(1, 4);
        finv.GetTolerance(toler, _tolpoint3d);
        finv.GetBounds(infb, supb);
        solinv[1] = _sol[3];
        solinv[2] = _param;
        solinv[3] = _sol[1];
        solinv[4] = _sol[2];

        FunctionSetRoot rsnld = new(finv, toler, 30);
        rsnld.Perform(finv, solinv, infb, supb);
        if (!rsnld.IsDone)
        {
            return false;
        }
        rsnld.Root(solinv);

        if (finv.IsSolution(solinv, _tolpoint3d))
        {
            Point2D p2d = new(solinv[3], solinv[4]);
            Containment situ = _domain1.Classify(p2d, _tolpoint2d, false);
            if ((situ != global::Kerf.Topology.Containment.In) && (situ != On))
            {
                return false;
            }
            _domain2.Initialize(_rst);
            _domain2.InitVertexIterator();
            isVtx = !_domain2.MoreVertex();
            while (!isVtx)
            {
                vtx = _domain2.Vertex();
                if (Math.Abs(BlendTool.Parameter(vtx, _rst) - solinv[1])
                    <= BlendTool.Tolerance(vtx, _rst))
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
            // The section is recalculated by direct resolution, otherwise incoherences between the parameter and
            // the ground caused by yawn are returned.

            math_Vector infbound = new(1, 3), supbound = new(1, 3);
            math_Vector parinit = new(1, 3), tolerance = new(1, 3);
            func.GetTolerance(tolerance, _tolpoint3d);
            func.GetBounds(infbound, supbound);

            FunctionSetRoot rsnld2 = new(func, tolerance, 30);
            parinit[1] = solinv[3];
            parinit[2] = solinv[4];
            parinit[3] = solinv[1];
            func.Set(solinv[2]);
            rsnld2.Perform(func, parinit, infbound, supbound);
            if (!rsnld2.IsDone)
            {
                return false;
            }
            rsnld2.Root(parinit);
            solinv[3] = parinit[1];
            solinv[4] = parinit[2];
            solinv[1] = parinit[3];
            return true;
        }
        return false;
    }

    private bool Recadre(SurfPointFuncInv finvP, math_Vector solinv, ref bool isVtx, ref HVertex? vtx)
    {
        // The point.
        Point2D p2drst;
        double firstrst = _rst.FirstParameter();
        double lastrst = _rst.LastParameter();
        double wpoint = firstrst;
        if ((_sol[3] - firstrst) > (lastrst - _sol[3]))
        {
            wpoint = lastrst;
        }
        p2drst = _rst.Value(wpoint);
        Point3D thepoint = _surf2.Value(p2drst.X, p2drst.Y);

        finvP.Set(thepoint);
        math_Vector toler = new(1, 3), infb = new(1, 3), supb = new(1, 3);
        finvP.GetTolerance(toler, _tolpoint3d);
        finvP.GetBounds(infb, supb);
        solinv[1] = _param;
        solinv[2] = _sol[1];
        solinv[3] = _sol[2];

        FunctionSetRoot rsnld = new(finvP, toler, 30);
        rsnld.Perform(finvP, solinv, infb, supb);
        if (!rsnld.IsDone)
        {
            return false;
        }
        rsnld.Root(solinv);

        if (finvP.IsSolution(solinv, _tolpoint3d))
        {
            Point2D p2d = new(solinv[2], solinv[3]);
            Containment situ = _domain1.Classify(p2d, _tolpoint2d, false);
            if ((situ != global::Kerf.Topology.Containment.In) && (situ != On))
            {
                return false;
            }
            _domain2.Initialize(_rst);
            _domain2.InitVertexIterator();
            isVtx = !_domain2.MoreVertex();
            while (!isVtx)
            {
                vtx = _domain2.Vertex();
                if (Math.Abs(BlendTool.Parameter(vtx, _rst) - wpoint)
                    <= BlendTool.Tolerance(vtx, _rst))
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
                tgline = _previousP.TangentOnS1();
            }
            else
            {
                tgline = new Vector3D(prevprev.PointOnS(), _previousP.PointOnS());
            }
        }
        else
        {
            _surf2.D1(p2d.X, p2d.Y, out pbid, out d1u, out d1v);
            if (!computetranstionaveclacorde)
            {
                tgline = _previousP.TangentOnS2();
            }
            else
            {
                tgline = new Vector3D(prevprev.PointOnC(), _previousP.PointOnC());
            }
        }

        tgrst = Vector3D.LinearForm(dp2d.X, d1u, dp2d.Y, d1v);
        normale = d1u.Crossed(d1v);

        global::Kerf.Engine.Intersection.Walking.IntSurf.MakeTransition(tgline, tgrst, new Direction3D(normale), ref tLine, ref tArc);
    }

    private void MakeExtremity(ref Extremity extrem, bool onFirst, Curve2DAdaptorBase Arc, double param,
                               bool isVtx, HVertex? vtx)
    {
        WalkingTransition tline = new(), tarc = new();
        double prm;
        TopolTool iter;
        if (onFirst)
        {
            extrem.SetValue(_previousP.PointOnS(), _sol[1], _sol[2], _previousP.Parameter(), _tolpoint3d);
            if (!_previousP.IsTangencyPoint)
            {
                extrem.SetTangent(_previousP.TangentOnS());
            }
            iter = _domain1;
        }
        else
        {
            extrem.SetValue(_previousP.PointOnC(), _sol[3], _previousP.Parameter(), _tolpoint3d);
            if (!_previousP.IsTangencyPoint)
            {
                extrem.SetTangent(_previousP.TangentOnC());
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

    private BlendStatus CheckDeflectionOnSurf(in BlendPoint curPoint)
    {
        // Controls 3d of Blend_CSWalking.

        // rule by tests in U4 corresponds to 11.478 d
        const double cosRef3D = 0.98;
        double cosi = 0, cosi2 = 0;
        bool curpointistangent = curPoint.IsTangencyPoint;
        bool prevpointistangent = _previousP.IsTangencyPoint;

        Point3D psurf = curPoint.PointOnS();
        Vector3D tgsurf = new();
        if (!curpointistangent)
        {
            tgsurf = curPoint.TangentOnS();
        }
        Point3D prevP = _previousP.PointOnS();
        Vector3D prevTg = new();
        if (!prevpointistangent)
        {
            prevTg = _previousP.TangentOnS();
        }
        double norme, prevNorme = 0.0;
        Vector3D corde = new(prevP, psurf);
        norme = corde.SquareMagnitude;
        if (!prevpointistangent)
        {
            prevNorme = prevTg.SquareMagnitude;
        }

        double toler3d = 0.01 * _tolpoint3d;
        if (norme <= toler3d * toler3d)
        {
            // it can be necessary to force same point
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
                // not too great :
                return BlendStatus.StepTooLarge;
            }
        }
        return BlendStatus.OK;
    }

    private BlendStatus CheckDeflectionOnRst(in BlendPoint curPoint)
    {
        // Controls 3D of Blend_CSWalking.

        // rule by tests in U4 corresponds to 11.478 d
        const double cosRef3D = 0.98;
        double cosi, cosi2;
        bool curpointistangent = curPoint.IsTangencyPoint;
        bool prevpointistangent = _previousP.IsTangencyPoint;

        Point3D psurf = curPoint.PointOnC();
        Vector3D tgsurf = new();
        if (!curpointistangent)
        {
            tgsurf = curPoint.TangentOnC();
        }
        Point3D prevP = _previousP.PointOnC();
        Vector3D prevTg = new();
        if (!prevpointistangent)
        {
            prevTg = _previousP.TangentOnC();
        }
        double norme, prevNorme = 0.0;
        Vector3D corde = new(prevP, psurf);
        norme = corde.SquareMagnitude;
        if (!prevpointistangent)
        {
            prevNorme = prevTg.SquareMagnitude;
        }

        double toler3d = 0.01 * _tolpoint3d;
        if (norme <= toler3d * toler3d)
        {
            // it can be necessary to force same point
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

    private BlendStatus TestArret(SurfRstFunction func, bool testDeflection, BlendStatus state)
    {
        Point3D pts, ptrst;
        Point2D pt2drst;
        Vector3D tgs = new(), tgrst;
        Vector2D tg2ds, tg2drst = new();
        BlendStatus stateS, stateRst;
        WalkingTypeTrans tras = WalkingTypeTrans.Undecided, trarst = WalkingTypeTrans.Undecided;
        BlendPoint curpoint = new();

        if (func.IsSolution(_sol, _tolpoint3d))
        {
            bool curpointistangent = func.IsTangencyPoint;
            pts = func.PointOnS();
            ptrst = func.PointOnRst();
            pt2drst = func.Pnt2dOnRst();
            if (curpointistangent)
            {
                curpoint.SetValue(pts, ptrst, _param, _sol[1], _sol[2], pt2drst.X, pt2drst.Y, _sol[3]);
            }
            else
            {
                tgs = func.TangentOnS();
                tgrst = func.TangentOnRst();
                tg2ds = func.Tangent2dOnS();
                tg2drst = func.Tangent2dOnRst();

                curpoint.SetValue(pts, ptrst, _param, _sol[1], _sol[2], pt2drst.X, pt2drst.Y, _sol[3], tgs, tgrst,
                                  tg2ds, tg2drst);
            }
            if (testDeflection)
            {
                stateS = CheckDeflectionOnSurf(curpoint);
                stateRst = CheckDeflectionOnRst(curpoint);
            }
            else
            {
                stateS = stateRst = BlendStatus.OK;
            }
            if (stateS == BlendStatus.Backward)
            {
                stateS = BlendStatus.StepTooLarge;
            }
            if (stateRst == BlendStatus.Backward)
            {
                stateRst = BlendStatus.StepTooLarge;
            }
            if (stateS == BlendStatus.StepTooLarge || stateRst == BlendStatus.StepTooLarge)
            {
                return BlendStatus.StepTooLarge;
            }

            if (!_comptra && !curpointistangent)
            {
                Vector3D tgsecs, nors;
                func.Decroch(_sol, out nors, out tgsecs);
                nors = nors.Normalized();
                double testra = tgsecs.Dot(nors.Crossed(tgs));
                if (Math.Abs(testra) > _tolpoint3d)
                {
                    if (testra < 0.0)
                    {
                        tras = WalkingTypeTrans.In;
                    }
                    else if (testra > 0.0)
                    {
                        tras = WalkingTypeTrans.Out;
                    }
                    Point2D p2drstref;
                    Vector2D tg2drstref;
                    _rst.D1(_sol[3], out p2drstref, out tg2drstref);
                    testra = tg2drst.Dot(tg2drstref);
                    Orientation or = _domain2.Orientation(_rst);
                    if (Math.Abs(testra) > 1.0e-8)
                    {
                        if (testra < 0.0)
                        {
                            trarst = ConvOrToTra(global::Kerf.Engine.Topology.TopAbs.Reverse(or));
                        }
                        else if (testra > 0.0)
                        {
                            trarst = ConvOrToTra(or);
                        }
                        _comptra = true;
                        _line!.Set(tras, trarst);
                    }
                }
            }
            if (stateS == BlendStatus.OK || stateRst == BlendStatus.OK)
            {
                _previousP = curpoint;
                return state;
            }
            if (stateS == BlendStatus.StepTooSmall && stateRst == BlendStatus.StepTooSmall)
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

    private bool CheckInside(SurfRstFunction func, out Containment situOnC, out Containment situOnS,
                             out bool decroch)
    {
        math_Vector tolerance = new(1, 3);
        func.GetTolerance(tolerance, _tolpoint3d);
        // face pcurve.
        double w = _sol[3];
        if (w < _rst.FirstParameter() - tolerance[3] || w > _rst.LastParameter() + tolerance[3])
        {
            situOnC = global::Kerf.Topology.Containment.Out;
        }
        else if (w > _rst.FirstParameter() && w < _rst.LastParameter())
        {
            situOnC = global::Kerf.Topology.Containment.In;
        }
        else
        {
            situOnC = On;
        }

        // face surface
        Point2D p2d = new(_sol[1], _sol[2]);
        situOnS = _domain1.Classify(p2d, _tolpoint2d, false);

        // lost contact
        Vector3D tgs, nors;
        decroch = func.Decroch(_sol, out tgs, out nors);

        return (situOnC == global::Kerf.Topology.Containment.In && situOnS == global::Kerf.Topology.Containment.In && !decroch);
    }
}
