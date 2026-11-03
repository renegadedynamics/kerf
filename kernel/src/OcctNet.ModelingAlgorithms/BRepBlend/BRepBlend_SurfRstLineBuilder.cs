// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstLineBuilder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstLineBuilder.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_SurfRstLineBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.Blend_Status;
using static Occt.IntSurf_TypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// Walks a blend between a surface and a restriction (an arc of another surface) along a guide, building a
/// <see cref="BRepBlend_Line"/>. The walking stops where the section leaves the surface's face (the surface/curve
/// inverse function on its arcs), the restriction's ends (the surface/point inverse function) or loses contact with
/// the restriction's surface (the inverse function on the restriction, "décroché").
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT's <c>OCCT_DEBUG</c> drawing of the sections and its derivative traces are
/// left out.
/// </remarks>
public class BRepBlend_SurfRstLineBuilder
{
    private bool done;
    private BRepBlend_Line? line;
    private readonly math_Vector sol;
    private readonly SurfaceAdaptorBase surf1;
    private readonly TopolTool domain1;
    private readonly SurfaceAdaptorBase surf2;
    private readonly Curve2DAdaptorBase rst;
    private readonly TopolTool domain2;
    private double tolpoint3d;
    private double tolpoint2d;
    private double tolgui;
    private double pasmax;
    private double fleche;
    private double param;
    private Blend_Point previousP = new();
#pragma warning disable CS0414 // set and never read, as in OCCT
    private bool rebrou;
#pragma warning restore CS0414
    private bool iscomplete;
    private bool comptra;
    private double sens;
    private bool decrochdeb;
    private bool decrochfin;

    /// <summary>
    /// Finds the arc of the first surface's domain that the section <paramref name="theSol"/> leaves through: by
    /// intersection with the segment from the previous point, else by projection. <paramref name="PrevIndex"/> is an
    /// arc already tried, rejected. Returns the arc's index (0 if none) and its parameter in <paramref name="ponarc"/>.
    /// </summary>
    // OCCT: the header names the points (pt2d, lastpt2d), the definition (lastpt2d, pt2d); the definition's order here.
    public int ArcToRecadre(math_Vector theSol, int PrevIndex, out Point2D lastpt2d, out Point2D pt2d,
                            ref double ponarc)
    {
        int IndexSol = 0, nbarc = 0;
        bool ok = false;
        bool byinter = (line!.NbPoints() != 0), okinter = false;
        double distmin = RealLast();
        double uprev = 0.0, vprev = 0.0, prm = 0.0, dist = 0.0;

        if (byinter)
        {
            previousP.ParametersOnS(out uprev, out vprev);
        }
        pt2d = new Point2D(theSol[1], theSol[2]);
        lastpt2d = new Point2D(uprev, vprev);
        domain1.Init();

        while (domain1.More())
        {
            nbarc++;
            ok = false;
            if (byinter)
            {
                ok = okinter = BRepBlend_BlendTool.Inters(pt2d, lastpt2d, surf1, domain1.Value(), ref prm, ref dist);
            }
            if (!ok)
            {
                ok = BRepBlend_BlendTool.Project(pt2d, surf1, domain1.Value(), out prm, out dist);
            }

            if (ok && (nbarc != PrevIndex))
            {
                if (dist < distmin || okinter)
                {
                    distmin = dist;
                    ponarc = prm;
                    IndexSol = nbarc;
                    if (okinter && (PrevIndex == 0))
                    {
                        break;
                    }
                }
            }
            domain1.Next();
        }
        return IndexSol;
    }

    public BRepBlend_SurfRstLineBuilder(SurfaceAdaptorBase Surf1, TopolTool Domain1, SurfaceAdaptorBase Surf2,
                                        Curve2DAdaptorBase Rst, TopolTool Domain2)
    {
        done = false;
        sol = new math_Vector(1, 3);
        surf1 = Surf1;
        domain1 = Domain1;
        surf2 = Surf2;
        rst = Rst;
        domain2 = Domain2;
        tolpoint3d = 0.0;
        tolpoint2d = 0.0;
        tolgui = 0.0;
        pasmax = 0.0;
        fleche = 0.0;
        param = 0.0;
        rebrou = false;
        iscomplete = false;
        comptra = false;
        sens = 0.0;
        decrochdeb = false;
        decrochfin = false;
    }

    /// <summary>
    /// Walks from the guide's parameter <paramref name="Pdep"/> towards <paramref name="Pmax"/>, starting from the
    /// solution <paramref name="ParDep"/> (solved again first when <paramref name="Appro"/>).
    /// </summary>
    public void Perform(Blend_SurfRstFunction Func, Blend_FuncInv Finv, Blend_SurfPointFuncInv FinvP,
                        Blend_SurfCurvFuncInv FinvC, double Pdep, double Pmax, double MaxStep, double Tol3d,
                        double Tol2d, double TolGuide, math_Vector ParDep, double Fleche, bool Appro = false)
    {
        done = false;
        iscomplete = false;
        comptra = false;
        line = new BRepBlend_Line();
        tolpoint3d = Tol3d;
        tolpoint2d = Tol2d;
        tolgui = Math.Abs(TolGuide);
        fleche = Math.Abs(Fleche);
        rebrou = false;
        pasmax = Math.Abs(MaxStep);

        if (Pmax - Pdep >= 0.0)
        {
            sens = 1.0;
        }
        else
        {
            sens = -1.0;
        }

        Blend_Status State;

        param = Pdep;
        Func.Set(param);

        if (Appro)
        {
            Containment siturst, situs;
            bool decroch;
            math_Vector tolerance = new(1, 3), infbound = new(1, 3), supbound = new(1, 3);
            Func.GetTolerance(tolerance, tolpoint3d);
            Func.GetBounds(infbound, supbound);
            FunctionSetRoot rsnld = new(Func, tolerance, 30);

            rsnld.Perform(Func, ParDep, infbound, supbound);

            if (!rsnld.IsDone)
            {
                return;
            }
            rsnld.Root(sol);
            if (!CheckInside(Func, out siturst, out situs, out decroch))
            {
                return;
            }
        }
        else
        {
            sol.Initialized(ParDep);
        }

        State = TestArret(Func, false, Blend_OK);
        if (State != Blend_OK)
        {
            return;
        }
        // OCCT_DEBUG Drawsect left out.
        // Update the line.
        line.Append(previousP);
        double U, V;
        previousP.ParametersOnS(out U, out V);

        BRepBlend_Extremity ptf1 = new(previousP.PointOnS(), U, V, previousP.Parameter(), tolpoint3d);
        BRepBlend_Extremity ptf2 = new(previousP.PointOnC(), U, V, previousP.Parameter(), tolpoint3d); // OCCT: the surface's (U, V)
        if (!previousP.IsTangencyPoint())
        {
            ptf1.SetTangent(previousP.TangentOnS());
            ptf2.SetTangent(previousP.TangentOnC());
        }
        if (sens > 0.0)
        {
            line.SetStartPoints(ptf1, ptf2);
        }
        else
        {
            line.SetEndPoints(ptf1, ptf2);
        }
        InternalPerform(Func, Finv, FinvP, FinvC, Pmax);
        done = true;
    }

    /// <summary>
    /// Solves a first section at <paramref name="Pdep"/> and moves it to where the walking stops (the face's
    /// boundary, the restriction's ends or a lost contact): <paramref name="Psol"/> and <paramref name="ParSol"/>
    /// receive it.
    /// </summary>
    public bool PerformFirstSection(Blend_SurfRstFunction Func, Blend_FuncInv Finv, Blend_SurfPointFuncInv FinvP,
                                    Blend_SurfCurvFuncInv FinvC, double Pdep, double Pmax, math_Vector ParDep,
                                    double Tol3d, double Tol2d, double TolGuide, bool RecRst, bool RecP, bool RecS,
                                    ref double Psol, math_Vector ParSol)
    {
        done = false;
        iscomplete = false;
        comptra = false;
        line = new BRepBlend_Line();
        tolpoint3d = Tol3d;
        tolpoint2d = Tol2d;
        tolgui = Math.Abs(TolGuide);
        rebrou = false;

        if (Pmax - Pdep >= 0.0)
        {
            sens = 1.0;
        }
        else
        {
            sens = -1.0;
        }
        Blend_Status State = Blend_OnRst12;
        double trst = 0.0;
        bool recadp, recadrst, recads;
        double wp, wrst, ws;
        double U = 0.0, V = 0.0;
        math_Vector infbound = new(1, 3), supbound = new(1, 3), tolerance = new(1, 3);
        math_Vector solinvp = new(1, 3), solinvrst = new(1, 4), solinvs = new(1, 3);
        HVertex? Vtxp = null, Vtxrst = null, Vtxs = null;
        bool IsVtxp = false, IsVtxrst = false, IsVtxs = false;
        Curve2DAdaptorBase? Arc = null;
        wp = wrst = ws = Pmax;
        param = Pdep;
        Func.Set(param);
        Func.GetTolerance(tolerance, tolpoint3d);
        Func.GetBounds(infbound, supbound);

        FunctionSetRoot rsnld = new(Func, tolerance, 30);
        rsnld.Perform(Func, ParDep, infbound, supbound);
        if (!rsnld.IsDone)
        {
            return false;
        }
        rsnld.Root(sol);

        recads = RecS && Recadre(FinvC, solinvs, ref Arc, ref IsVtxs, ref Vtxs);
        if (recads)
        {
            ws = solinvs[1];
        }
        recadp = RecP && Recadre(FinvP, solinvp, ref IsVtxp, ref Vtxp);
        if (recadp)
        {
            wp = solinvp[1];
        }
        recadrst = RecRst && Recadre(Func, Finv, solinvrst, ref IsVtxrst, ref Vtxrst);
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
            if (sens * (wrst - wp) > tolgui)
            { // first one leaves the domain
                wrst = wp;
                U = solinvp[2];
                V = solinvp[3];
                trst = BRepBlend_BlendTool.Parameter(Vtxp!, rst);
                IsVtxrst = IsVtxp;
                Vtxrst = Vtxp;
            }
            else
            {
                U = solinvrst[3];
                V = solinvrst[4];
                trst = solinvrst[1];
            }
        }
        else if (recadp)
        {
            wrst = wp;
            U = solinvp[2];
            V = solinvp[3];
            trst = BRepBlend_BlendTool.Parameter(Vtxp!, rst);
            IsVtxrst = IsVtxp;
            Vtxrst = Vtxp;
            recadrst = true;
        }
        else if (recadrst)
        {
            U = solinvrst[3];
            V = solinvrst[4];
            trst = solinvrst[1];
        }
        if (recads && recadrst)
        {
            if (Math.Abs(ws - wrst) < tolgui)
            {
                State = Blend_OnRst12;
                param = 0.5 * (ws + wrst);
                sol[1] = U;
                sol[2] = V;
                sol[3] = solinvs[2];
            }
            else if (sens * (ws - wrst) < 0)
            {
                // ground on surf
                State = Blend_OnRst1;
                param = ws;
                Arc!.Value(solinvs[3]).Coord(out U, out V);
                sol[1] = U;
                sol[2] = V;
                sol[3] = solinvs[2];
            }
            else
            {
                // ground on rst
                State = Blend_OnRst2;
                param = wrst;
                sol[1] = U;
                sol[2] = V;
                sol[3] = trst;
            }
            Func.Set(param);
        }
        else if (recads)
        {
            // ground on surf
            State = Blend_OnRst1;
            param = ws;
            Arc!.Value(solinvs[3]).Coord(out U, out V);
            sol[1] = U;
            sol[2] = V;
            sol[3] = solinvs[2];
            Func.Set(param);
        }
        else if (recadrst)
        {
            // ground on rst
            State = Blend_OnRst2;
            param = wrst;
            sol[1] = U;
            sol[2] = V;
            sol[3] = trst;
            Func.Set(param);
        }
        State = TestArret(Func, false, State);
        Psol = param;
        ParSol.Initialized(sol);
        return true;
    }

    /// <summary>Walks the other way from the line's start, down to <paramref name="Pmin"/>.</summary>
    public bool Complete(Blend_SurfRstFunction Func, Blend_FuncInv Finv, Blend_SurfPointFuncInv FinvP,
                         Blend_SurfCurvFuncInv FinvC, double Pmin)
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        if (iscomplete)
        {
            return true;
        }
        if (sens > 0.0)
        {
            previousP = line!.Point(1);
        }
        else
        {
            previousP = line!.Point(line.NbPoints());
        }
        sens = -sens;
        param = previousP.Parameter();
        previousP.ParametersOnS(out sol[1], out sol[2]);
        sol[3] = previousP.ParameterOnC();

        InternalPerform(Func, Finv, FinvP, FinvC, Pmin);
        iscomplete = true;
        return true;
    }

    public bool IsDone() => done;

    public BRepBlend_Line Line()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return line!;
    }

    public bool DecrochStart() => decrochdeb;

    public bool DecrochEnd() => decrochfin;

    private void InternalPerform(Blend_SurfRstFunction Func, Blend_FuncInv Finv, Blend_SurfPointFuncInv FinvP,
                                 Blend_SurfCurvFuncInv FinvC, double Bound)
    {
        double stepw = pasmax;
        int nbp = line!.NbPoints();
        if (nbp >= 2)
        { // The last step is reproduced if it is not too small.
            if (sens < 0.0)
            {
                stepw = (line.Point(2).Parameter() - line.Point(1).Parameter());
            }
            else
            {
                stepw = (line.Point(nbp).Parameter() - line.Point(nbp - 1).Parameter());
            }
            stepw = Max(stepw, 100.0 * tolgui);
        }
        double parprec = param;
        if (sens * (parprec - Bound) >= -tolgui)
        {
            return;
        }
        Blend_Status State = Blend_OnRst12;
        Containment situonc = Unknown, situons = Unknown;
        bool decroch = false;
        bool Arrive, recadp, recadrst, recads, echecrecad;
        double wp, wrst, ws;
        double U = 0.0, V = 0.0;
        double trst = 0.0;
        math_Vector infbound = new(1, 3), supbound = new(1, 3);
        math_Vector parinit = new(1, 3), tolerance = new(1, 3);
        math_Vector solinvp = new(1, 3), solinvrst = new(1, 4), solinvs = new(1, 3);
        HVertex? Vtxp = null, Vtxrst = null, Vtxs = null;
        bool IsVtxp = false, IsVtxrst = false, IsVtxs = false;
        BRepBlend_Extremity Extrst = new(), Exts = new();
        Curve2DAdaptorBase? Arc = null;

        Func.GetTolerance(tolerance, tolpoint3d);
        Func.GetBounds(infbound, supbound);

        FunctionSetRoot rsnld = new(Func, tolerance, 30);
        parinit.Initialized(sol);

        Arrive = false;
        param = parprec + sens * stepw;
        if (sens * (param - Bound) > 0.0)
        {
            stepw = sens * (Bound - parprec) * 0.5;
            param = parprec + sens * stepw;
        }

        while (!Arrive)
        {
            bool bonpoint = true;
            // OCCT_DEBUG_BBPP_N_TRDERIV traces left out.
            Func.Set(param);
            rsnld.Perform(Func, parinit, infbound, supbound);

            if (rsnld.IsDone)
            {
                rsnld.Root(sol);
                if (!CheckInside(Func, out situonc, out situons, out decroch) && line.NbPoints() == 1)
                {
                    State = Blend_StepTooLarge;
                    bonpoint = false;
                }
            }
            else
            {
                State = Blend_StepTooLarge;
                bonpoint = false;
            }
            if (bonpoint)
            {
                wp = wrst = ws = Bound;
                recadp = recadrst = recads = false;
                echecrecad = false;
                if (situons == Out || situons == On)
                {
                    // pb inverse rst/rst
                    recads = Recadre(FinvC, solinvs, ref Arc, ref IsVtxs, ref Vtxs);
                    if (recads)
                    {
                        ws = solinvs[1];
                        // It is necessary to reevaluate the deviation (BUC60360)
                        Vector3D t, n;
                        Func.Set(ws);
                        Arc!.Value(solinvs[3]).Coord(out U, out V);
                        sol[1] = U;
                        sol[2] = V;
                        sol[3] = solinvs[2];
                        decroch = Func.Decroch(sol, out t, out n);
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }
                if (situonc == Out || situonc == On)
                {
                    // pb inverse point/surf
                    recadp = Recadre(FinvP, solinvp, ref IsVtxp, ref Vtxp);
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
                    recadrst = Recadre(Func, Finv, solinvrst, ref IsVtxrst, ref Vtxrst);
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
                        if (sens * (wrst - wp) > tolgui)
                        { // first one leaves the domain
                            wrst = wp;
                            U = solinvp[2];
                            V = solinvp[3];
                            trst = BRepBlend_BlendTool.Parameter(Vtxp!, rst);
                            IsVtxrst = IsVtxp;
                            Vtxrst = Vtxp;
                        }
                        else
                        {
                            decroch = true;
                            U = solinvrst[3];
                            V = solinvrst[4];
                            trst = solinvrst[1];
                        }
                    }
                    else if (recadp)
                    {
                        wrst = wp;
                        U = solinvp[2];
                        V = solinvp[3];
                        trst = BRepBlend_BlendTool.Parameter(Vtxp!, rst);
                        IsVtxrst = IsVtxp;
                        Vtxrst = Vtxp;
                        recadrst = true;
                    }
                    else if (recadrst)
                    {
                        decroch = true;
                        U = solinvrst[3];
                        V = solinvrst[4];
                        trst = solinvrst[1];
                    }
                    if (recads && recadrst)
                    {
                        if (Math.Abs(ws - wrst) < tolgui)
                        {
                            State = Blend_OnRst12;
                            param = 0.5 * (ws + wrst);
                            sol[1] = U;
                            sol[2] = V;
                            sol[3] = solinvs[3]; // OCCT: the arc's parameter where PerformFirstSection takes the restriction's (solinvs(2))
                        }
                        else if (sens * (ws - wrst) < 0)
                        {
                            // ground on surf
                            decroch = false;
                            State = Blend_OnRst1;
                            param = ws;
                            Arc!.Value(solinvs[3]).Coord(out U, out V);
                            sol[1] = U;
                            sol[2] = V;
                            sol[3] = solinvs[2];
                        }
                        else
                        {
                            // ground on rst
                            State = Blend_OnRst2;
                            param = wrst;
                            sol[1] = U;
                            sol[2] = V;
                            sol[3] = trst;
                        }
                        Func.Set(param);
                    }
                    else if (recads)
                    {
                        // ground on surf
                        State = Blend_OnRst1;
                        param = ws;
                        Arc!.Value(solinvs[3]).Coord(out U, out V);
                        sol[1] = U;
                        sol[2] = V;
                        sol[3] = solinvs[2];
                        Func.Set(param);
                    }
                    else if (recadrst)
                    {
                        // ground on rst
                        State = Blend_OnRst2;
                        param = wrst;
                        sol[1] = U;
                        sol[2] = V;
                        sol[3] = trst;
                        Func.Set(param);
                    }
                    else
                    {
                        State = Blend_OK;
                    }
                    State = TestArret(Func, true, State);
                }
                else
                {
                    // Failed reframing. Leave with PointsConfondus
                    State = Blend_SamePoints;
                }
            }

            switch (State)
            {
                case Blend_OK:
                {
                    // OCCT_DEBUG Drawsect left out.
                    // Update the line.
                    if (sens > 0.0)
                    {
                        line.Append(previousP);
                    }
                    else
                    {
                        line.Prepend(previousP);
                    }
                    parinit.Initialized(sol);
                    parprec = param;

                    if (param == Bound)
                    {
                        Arrive = true;
                        Exts.SetValue(previousP.PointOnS(), sol[1], sol[2], previousP.Parameter(), tolpoint3d);
                        MakeExtremity(ref Extrst, false, rst, sol[3], IsVtxrst, Vtxrst);
                        // Indicate end on Bound.
                    }
                    else
                    {
                        param = param + sens * stepw;
                        if (sens * (param - Bound) > -tolgui)
                        {
                            param = Bound;
                        }
                    }
                }
                break;

                case Blend_StepTooLarge:
                {
                    stepw = stepw / 2.0;
                    if (Math.Abs(stepw) < tolgui)
                    {
                        previousP.ParametersOnS(out U, out V);
                        Exts.SetValue(previousP.PointOnS(), U, V, previousP.Parameter(), tolpoint3d);
                        Extrst.SetValue(previousP.PointOnC(), previousP.ParameterOnC(), previousP.Parameter(),
                                        tolpoint3d);
                        Arrive = true;
                    }
                    else
                    {
                        param = parprec + sens * stepw; // no risk to exceed Bound.
                    }
                }
                break;

                case Blend_StepTooSmall:
                {
                    // OCCT_DEBUG Drawsect left out.
                    // Update the line.
                    if (sens > 0.0)
                    {
                        line.Append(previousP);
                    }
                    else
                    {
                        line.Prepend(previousP);
                    }
                    parinit.Initialized(sol);
                    parprec = param;

                    stepw = Min(1.5 * stepw, pasmax);
                    if (param == Bound)
                    {
                        Arrive = true;
                        Exts.SetValue(previousP.PointOnS(), sol[1], sol[2], previousP.Parameter(), tolpoint3d);
                        MakeExtremity(ref Extrst, false, rst, sol[3], IsVtxrst, Vtxrst);
                        // Indicate end on Bound.
                    }
                    else
                    {
                        param = param + sens * stepw;
                        if (sens * (param - Bound) > -tolgui)
                        {
                            param = Bound;
                        }
                    }
                }
                break;

                case Blend_OnRst1:
                {
                    // OCCT_DEBUG Drawsect left out.
                    if (sens > 0.0)
                    {
                        line.Append(previousP);
                    }
                    else
                    {
                        line.Prepend(previousP);
                    }
                    MakeExtremity(ref Exts, true, Arc!, solinvs[3], IsVtxs, Vtxs);
                    MakeExtremity(ref Extrst, false, rst, sol[3], IsVtxrst, Vtxrst);
                    Arrive = true;
                }
                break;

                case Blend_OnRst2:
                {
                    // OCCT_DEBUG Drawsect left out.
                    if (sens > 0.0)
                    {
                        line.Append(previousP);
                    }
                    else
                    {
                        line.Prepend(previousP);
                    }
                    Exts.SetValue(previousP.PointOnS(), sol[1], sol[2], previousP.Parameter(), tolpoint3d);
                    MakeExtremity(ref Extrst, false, rst, sol[3], IsVtxrst, Vtxrst);
                    Arrive = true;
                }
                break;

                case Blend_OnRst12:
                {
                    // OCCT_DEBUG Drawsect left out.
                    if (sens > 0.0)
                    {
                        line.Append(previousP);
                    }
                    else
                    {
                        line.Prepend(previousP);
                    }
                    MakeExtremity(ref Exts, true, Arc!, solinvs[1], IsVtxs, Vtxs);
                    MakeExtremity(ref Extrst, false, rst, sol[3], IsVtxrst, Vtxrst);
                    Arrive = true;
                }
                break;

                case Blend_SamePoints:
                {
                    // Stop
                    previousP.ParametersOnS(out U, out V);
                    Exts.SetValue(previousP.PointOnS(), U, V, previousP.Parameter(), tolpoint3d);
                    Extrst.SetValue(previousP.PointOnC(), previousP.ParameterOnC(), previousP.Parameter(),
                                    tolpoint3d);
                    Arrive = true;
                }
                break;
                default:
                    break;
            }
            if (Arrive)
            {
                if (sens > 0.0)
                {
                    line.SetEndPoints(Exts, Extrst);
                    decrochfin = decroch;
                }
                else
                {
                    line.SetStartPoints(Exts, Extrst);
                    decrochdeb = decroch;
                }
            }
        }
    }

    // Reframes the section surface / restriction on an arc of the surface's domain. Arc, IsVtx and Vtx are OCCT's
    // references (Arc and Vtx written only on the paths that find them).
    private bool Recadre(Blend_SurfCurvFuncInv FinvC, math_Vector Solinv, ref Curve2DAdaptorBase? Arc, ref bool IsVtx,
                         ref HVertex? Vtx)
    {
        bool recadre = false;

        Point2D pt2d, lastpt2d;
        int IndexSol, nbarc;
        double pmin = 0.0; // OCCT leaves it uninitialised; ArcToRecadre sets it whenever it finds an arc

        IndexSol = ArcToRecadre(sol, 0, out lastpt2d, out pt2d, ref pmin);

        IsVtx = false;
        if (IndexSol == 0)
        {
            return false;
        }

        domain1.Init();
        nbarc = 1;
        while (nbarc < IndexSol)
        {
            nbarc++;
            domain1.Next();
        }
        Arc = domain1.Value();

        FinvC.Set(Arc);

        math_Vector toler = new(1, 3), infb = new(1, 3), supb = new(1, 3);
        // use reduced Tol argument value to pass testcase blend complex A6 with scale factor of model 0.1 (base
        // scale = 1000). So, here we using 1.0e-5 rather than 1.0e-4 of tolerance of point in 3d
        FinvC.GetTolerance(toler, 0.1 * tolpoint3d);
        FinvC.GetBounds(infb, supb);
        Solinv[1] = param;
        Solinv[2] = sol[3];
        Solinv[3] = pmin;

        FunctionSetRoot rsnld = new(FinvC, toler, 30);
        rsnld.Perform(FinvC, Solinv, infb, supb);

        if (!rsnld.IsDone)
        {
            // OCCT_DEBUG trace left out.
        }
        else
        {
            // It is necessary to check the value of the function
            rsnld.Root(Solinv);
            recadre = FinvC.IsSolution(Solinv, tolpoint3d);
        }

        // In case of fail, it is checked if another arc can be useful (case of output at the proximity of a vertex)
        if (!recadre)
        {
            IndexSol = ArcToRecadre(sol, IndexSol, out lastpt2d, out pt2d, ref pmin);
            if (IndexSol == 0)
            {
                return false; // No other solution
            }

            domain1.Init();
            nbarc = 1;
            while (nbarc < IndexSol)
            {
                nbarc++;
                domain1.Next();
            }

            Arc = domain1.Value();
            FinvC.Set(Arc);

            FinvC.GetTolerance(toler, tolpoint3d);
            FinvC.GetBounds(infb, supb);

            Solinv[3] = pmin;

            FunctionSetRoot aRsnld = new(FinvC, toler, 30);
            aRsnld.Perform(FinvC, Solinv, infb, supb);

            if (!aRsnld.IsDone)
            {
                // OCCT_DEBUG trace left out.
            }
            else
            {
                // It is necessary to check the value of the function
                aRsnld.Root(Solinv);
                recadre = FinvC.IsSolution(Solinv, tolpoint3d);
            }
        }

        if (recadre)
        {
            double w = Solinv[2];
            if (w < rst.FirstParameter() - toler[2] || w > rst.LastParameter() + toler[2])
            {
                return false;
            }
            domain1.Initialize(Arc);
            domain1.InitVertexIterator();
            IsVtx = !domain1.MoreVertex();
            while (!IsVtx)
            {
                Vtx = domain1.Vertex();
                if (Math.Abs(BRepBlend_BlendTool.Parameter(Vtx, Arc) - Solinv[3])
                    <= BRepBlend_BlendTool.Tolerance(Vtx, Arc))
                {
                    IsVtx = true;
                }
                else
                {
                    domain1.NextVertex();
                    IsVtx = !domain1.MoreVertex();
                }
            }
            if (!domain1.MoreVertex())
            {
                IsVtx = false;
            }
            return true;
        }
        return false;
    }

    private bool Recadre(Blend_SurfRstFunction Func, Blend_FuncInv Finv, math_Vector Solinv, ref bool IsVtx,
                         ref HVertex? Vtx)
    {
        math_Vector toler = new(1, 4), infb = new(1, 4), supb = new(1, 4);
        Finv.GetTolerance(toler, tolpoint3d);
        Finv.GetBounds(infb, supb);
        Solinv[1] = sol[3];
        Solinv[2] = param;
        Solinv[3] = sol[1];
        Solinv[4] = sol[2];

        FunctionSetRoot rsnld = new(Finv, toler, 30);
        rsnld.Perform(Finv, Solinv, infb, supb);
        if (!rsnld.IsDone)
        {
            return false;
        }
        rsnld.Root(Solinv);

        if (Finv.IsSolution(Solinv, tolpoint3d))
        {
            Point2D p2d = new(Solinv[3], Solinv[4]);
            Containment situ = domain1.Classify(p2d, tolpoint2d, false);
            if ((situ != In) && (situ != On))
            {
                return false;
            }
            domain2.Initialize(rst);
            domain2.InitVertexIterator();
            IsVtx = !domain2.MoreVertex();
            while (!IsVtx)
            {
                Vtx = domain2.Vertex();
                if (Math.Abs(BRepBlend_BlendTool.Parameter(Vtx, rst) - Solinv[1])
                    <= BRepBlend_BlendTool.Tolerance(Vtx, rst))
                {
                    IsVtx = true;
                }
                else
                {
                    domain2.NextVertex();
                    IsVtx = !domain2.MoreVertex();
                }
            }
            if (!domain2.MoreVertex())
            {
                IsVtx = false;
            }
            // The section is recalculated by direct resolution, otherwise incoherences between the parameter and
            // the ground caused by yawn are returned.

            math_Vector infbound = new(1, 3), supbound = new(1, 3);
            math_Vector parinit = new(1, 3), tolerance = new(1, 3);
            Func.GetTolerance(tolerance, tolpoint3d);
            Func.GetBounds(infbound, supbound);

            FunctionSetRoot rsnld2 = new(Func, tolerance, 30);
            parinit[1] = Solinv[3];
            parinit[2] = Solinv[4];
            parinit[3] = Solinv[1];
            Func.Set(Solinv[2]);
            rsnld2.Perform(Func, parinit, infbound, supbound);
            if (!rsnld2.IsDone)
            {
                return false;
            }
            rsnld2.Root(parinit);
            Solinv[3] = parinit[1];
            Solinv[4] = parinit[2];
            Solinv[1] = parinit[3];
            return true;
        }
        return false;
    }

    private bool Recadre(Blend_SurfPointFuncInv FinvP, math_Vector Solinv, ref bool IsVtx, ref HVertex? Vtx)
    {
        // The point.
        Point2D p2drst;
        double firstrst = rst.FirstParameter();
        double lastrst = rst.LastParameter();
        double wpoint = firstrst;
        if ((sol[3] - firstrst) > (lastrst - sol[3]))
        {
            wpoint = lastrst;
        }
        p2drst = rst.Value(wpoint);
        Point3D thepoint = surf2.Value(p2drst.X(), p2drst.Y());

        FinvP.Set(thepoint);
        math_Vector toler = new(1, 3), infb = new(1, 3), supb = new(1, 3);
        FinvP.GetTolerance(toler, tolpoint3d);
        FinvP.GetBounds(infb, supb);
        Solinv[1] = param;
        Solinv[2] = sol[1];
        Solinv[3] = sol[2];

        FunctionSetRoot rsnld = new(FinvP, toler, 30);
        rsnld.Perform(FinvP, Solinv, infb, supb);
        if (!rsnld.IsDone)
        {
            return false;
        }
        rsnld.Root(Solinv);

        if (FinvP.IsSolution(Solinv, tolpoint3d))
        {
            Point2D p2d = new(Solinv[2], Solinv[3]);
            Containment situ = domain1.Classify(p2d, tolpoint2d, false);
            if ((situ != In) && (situ != On))
            {
                return false;
            }
            domain2.Initialize(rst);
            domain2.InitVertexIterator();
            IsVtx = !domain2.MoreVertex();
            while (!IsVtx)
            {
                Vtx = domain2.Vertex();
                if (Math.Abs(BRepBlend_BlendTool.Parameter(Vtx, rst) - wpoint)
                    <= BRepBlend_BlendTool.Tolerance(Vtx, rst))
                {
                    IsVtx = true;
                }
                else
                {
                    domain2.NextVertex();
                    IsVtx = !domain2.MoreVertex();
                }
            }
            if (!domain2.MoreVertex())
            {
                IsVtx = false;
            }
            return true;
        }
        return false;
    }

    private void Transition(bool OnFirst, Curve2DAdaptorBase Arc, double Param, ref IntSurf_Transition TLine,
                            ref IntSurf_Transition TArc)
    {
        bool computetranstionaveclacorde = false;
        Vector3D tgline;
        Blend_Point prevprev = new();

        if (previousP.IsTangencyPoint())
        {
            if (line!.NbPoints() < 2)
            {
                return;
            }
            computetranstionaveclacorde = true;
            if (sens < 0)
            {
                prevprev = line.Point(2);
            }
            else
            {
                prevprev = line.Point(line.NbPoints() - 1);
            }
        }
        Point2D p2d;
        Vector2D dp2d;

        Point3D pbid;
        Vector3D d1u, d1v, normale, tgrst = new();

        Arc.D1(Param, out p2d, out dp2d);
        if (OnFirst)
        {
            surf1.D1(p2d.X(), p2d.Y(), out pbid, out d1u, out d1v);
            if (!computetranstionaveclacorde)
            {
                tgline = previousP.TangentOnS1();
            }
            else
            {
                tgline = new Vector3D(prevprev.PointOnS(), previousP.PointOnS());
            }
        }
        else
        {
            surf2.D1(p2d.X(), p2d.Y(), out pbid, out d1u, out d1v);
            if (!computetranstionaveclacorde)
            {
                tgline = previousP.TangentOnS2();
            }
            else
            {
                tgline = new Vector3D(prevprev.PointOnC(), previousP.PointOnC());
            }
        }

        tgrst.SetLinearForm(dp2d.X(), d1u, dp2d.Y(), d1v);
        normale = d1u.Crossed(d1v);

        IntSurf.MakeTransition(tgline, tgrst, new Direction3D(normale), ref TLine, ref TArc);
    }

    private void MakeExtremity(ref BRepBlend_Extremity Extrem, bool OnFirst, Curve2DAdaptorBase Arc, double Param,
                               bool IsVtx, HVertex? Vtx)
    {
        IntSurf_Transition Tline = new(), Tarc = new();
        double prm;
        TopolTool Iter;
        if (OnFirst)
        {
            Extrem.SetValue(previousP.PointOnS(), sol[1], sol[2], previousP.Parameter(), tolpoint3d);
            if (!previousP.IsTangencyPoint())
            {
                Extrem.SetTangent(previousP.TangentOnS());
            }
            Iter = domain1;
        }
        else
        {
            Extrem.SetValue(previousP.PointOnC(), sol[3], previousP.Parameter(), tolpoint3d);
            if (!previousP.IsTangencyPoint())
            {
                Extrem.SetTangent(previousP.TangentOnC());
            }
            Iter = domain2;
        }

        Iter.Init();
        if (!IsVtx)
        {
            Transition(OnFirst, Arc, Param, ref Tline, ref Tarc);
            Extrem.AddArc(Arc, Param, Tline, Tarc);
        }
        else
        {
            Extrem.SetVertex(Vtx!);
            while (Iter.More())
            {
                Curve2DAdaptorBase arc = Iter.Value();
                if (!ReferenceEquals(arc, Arc))
                {
                    Iter.Initialize(arc);
                    Iter.InitVertexIterator();
                    while (Iter.MoreVertex())
                    {
                        if (Iter.Identical(Vtx!, Iter.Vertex()))
                        {
                            prm = BRepBlend_BlendTool.Parameter(Vtx!, arc);
                            Transition(OnFirst, arc, prm, ref Tline, ref Tarc);
                            Extrem.AddArc(arc, prm, Tline, Tarc);
                        }
                        Iter.NextVertex();
                    }
                }
                else
                {
                    Transition(OnFirst, arc, Param, ref Tline, ref Tarc);
                    Extrem.AddArc(arc, Param, Tline, Tarc);
                }
                Iter.Next();
            }
        }
    }

    private Blend_Status CheckDeflectionOnSurf(in Blend_Point CurPoint)
    {
        // Controls 3d of Blend_CSWalking.

        // rule by tests in U4 corresponds to 11.478 d
        const double CosRef3D = 0.98;
        double Cosi = 0, Cosi2 = 0;
        bool curpointistangent = CurPoint.IsTangencyPoint();
        bool prevpointistangent = previousP.IsTangencyPoint();

        Point3D Psurf = CurPoint.PointOnS();
        Vector3D Tgsurf = new();
        if (!curpointistangent)
        {
            Tgsurf = CurPoint.TangentOnS();
        }
        Point3D prevP = previousP.PointOnS();
        Vector3D prevTg = new();
        if (!prevpointistangent)
        {
            prevTg = previousP.TangentOnS();
        }
        double Norme, prevNorme = 0.0;
        Vector3D Corde = new(prevP, Psurf);
        Norme = Corde.SquareMagnitude();
        if (!prevpointistangent)
        {
            prevNorme = prevTg.SquareMagnitude();
        }

        double toler3d = 0.01 * tolpoint3d;
        if (Norme <= toler3d * toler3d)
        {
            // it can be necessary to force same point
            return Blend_SamePoints;
        }
        if (!prevpointistangent)
        {
            if (prevNorme <= toler3d * toler3d)
            {
                return Blend_SamePoints;
            }
            Cosi = sens * Corde * prevTg;
            if (Cosi < 0.0)
            { // angle 3d>pi/2. --> return back
                return Blend_Backward;
            }

            Cosi2 = Cosi * Cosi / prevNorme / Norme;
            if (Cosi2 < CosRef3D)
            {
                return Blend_StepTooLarge;
            }
        }

        if (!curpointistangent)
        {
            // Check if it is necessary to control the sign of prevtg*Tgsurf
            Cosi = sens * Corde * Tgsurf;
            Cosi2 = Cosi * Cosi / Tgsurf.SquareMagnitude() / Norme;
            if (Cosi2 < CosRef3D || Cosi < 0.0)
            {
                return Blend_StepTooLarge;
            }
        }

        if (!curpointistangent && !prevpointistangent)
        {
            // Estimation of the current arrow
            double FlecheCourante =
                (prevTg.Normalized().XYZ() - Tgsurf.Normalized().XYZ()).SquareModulus() * Norme / 64.0;

            if (FlecheCourante <= 0.25 * fleche * fleche)
            {
                return Blend_StepTooSmall;
            }
            if (FlecheCourante > fleche * fleche)
            {
                // not too great :
                return Blend_StepTooLarge;
            }
        }
        return Blend_OK;
    }

    private Blend_Status CheckDeflectionOnRst(in Blend_Point CurPoint)
    {
        // Controls 3D of Blend_CSWalking.

        // rule by tests in U4 corresponds to 11.478 d
        const double CosRef3D = 0.98;
        double Cosi, Cosi2;
        bool curpointistangent = CurPoint.IsTangencyPoint();
        bool prevpointistangent = previousP.IsTangencyPoint();

        Point3D Psurf = CurPoint.PointOnC();
        Vector3D Tgsurf = new();
        if (!curpointistangent)
        {
            Tgsurf = CurPoint.TangentOnC();
        }
        Point3D prevP = previousP.PointOnC();
        Vector3D prevTg = new();
        if (!prevpointistangent)
        {
            prevTg = previousP.TangentOnC();
        }
        double Norme, prevNorme = 0.0;
        Vector3D Corde = new(prevP, Psurf);
        Norme = Corde.SquareMagnitude();
        if (!prevpointistangent)
        {
            prevNorme = prevTg.SquareMagnitude();
        }

        double toler3d = 0.01 * tolpoint3d;
        if (Norme <= toler3d * toler3d)
        {
            // it can be necessary to force same point
            return Blend_SamePoints;
        }
        if (!prevpointistangent)
        {
            if (prevNorme <= toler3d * toler3d)
            {
                return Blend_SamePoints;
            }
            Cosi = sens * Corde * prevTg;
            if (Cosi < 0.0)
            { // angle 3d>pi/2. --> return back
                return Blend_Backward;
            }

            Cosi2 = Cosi * Cosi / prevNorme / Norme;
            if (Cosi2 < CosRef3D)
            {
                return Blend_StepTooLarge;
            }
        }

        if (!curpointistangent)
        {
            // Check if it is necessary to control the sign of prevtg*Tgsurf
            Cosi = sens * Corde * Tgsurf;
            Cosi2 = Cosi * Cosi / Tgsurf.SquareMagnitude() / Norme;
            if (Cosi2 < CosRef3D || Cosi < 0.0)
            {
                return Blend_StepTooLarge;
            }
        }

        if (!curpointistangent && !prevpointistangent)
        {
            // Estimation of the current arrow
            double FlecheCourante =
                (prevTg.Normalized().XYZ() - Tgsurf.Normalized().XYZ()).SquareModulus() * Norme / 64.0;

            if (FlecheCourante <= 0.25 * fleche * fleche)
            {
                return Blend_StepTooSmall;
            }
            if (FlecheCourante > fleche * fleche)
            {
                // not too great
                return Blend_StepTooLarge;
            }
        }
        return Blend_OK;
    }

    private static IntSurf_TypeTrans ConvOrToTra(Orientation O)
    {
        if (O == Forward)
        {
            return IntSurf_In;
        }
        return IntSurf_Out;
    }

    private Blend_Status TestArret(Blend_SurfRstFunction Func, bool TestDeflection, Blend_Status State)
    {
        Point3D pts, ptrst;
        Point2D pt2drst;
        Vector3D tgs = new(), tgrst;
        Vector2D tg2ds, tg2drst = new();
        Blend_Status StateS, StateRst;
        IntSurf_TypeTrans tras = IntSurf_Undecided, trarst = IntSurf_Undecided;
        Blend_Point curpoint = new();

        if (Func.IsSolution(sol, tolpoint3d))
        {
            bool curpointistangent = Func.IsTangencyPoint();
            pts = Func.PointOnS();
            ptrst = Func.PointOnRst();
            pt2drst = Func.Pnt2dOnRst();
            if (curpointistangent)
            {
                curpoint.SetValue(pts, ptrst, param, sol[1], sol[2], pt2drst.X(), pt2drst.Y(), sol[3]);
            }
            else
            {
                tgs = Func.TangentOnS();
                tgrst = Func.TangentOnRst();
                tg2ds = Func.Tangent2dOnS();
                tg2drst = Func.Tangent2dOnRst();

                curpoint.SetValue(pts, ptrst, param, sol[1], sol[2], pt2drst.X(), pt2drst.Y(), sol[3], tgs, tgrst,
                                  tg2ds, tg2drst);
            }
            if (TestDeflection)
            {
                StateS = CheckDeflectionOnSurf(curpoint);
                StateRst = CheckDeflectionOnRst(curpoint);
            }
            else
            {
                StateS = StateRst = Blend_OK;
            }
            if (StateS == Blend_Backward)
            {
                StateS = Blend_StepTooLarge;
                rebrou = true;
            }
            if (StateRst == Blend_Backward)
            {
                StateRst = Blend_StepTooLarge;
                rebrou = true;
            }
            if (StateS == Blend_StepTooLarge || StateRst == Blend_StepTooLarge)
            {
                return Blend_StepTooLarge;
            }

            if (!comptra && !curpointistangent)
            {
                Vector3D tgsecs, nors;
                Func.Decroch(sol, out nors, out tgsecs);
                nors.Normalize();
                double testra = tgsecs.Dot(nors.Crossed(tgs));
                if (Math.Abs(testra) > tolpoint3d)
                {
                    if (testra < 0.0)
                    {
                        tras = IntSurf_In;
                    }
                    else if (testra > 0.0)
                    {
                        tras = IntSurf_Out;
                    }
                    Point2D p2drstref;
                    Vector2D tg2drstref;
                    rst.D1(sol[3], out p2drstref, out tg2drstref);
                    testra = tg2drst.Dot(tg2drstref);
                    Orientation Or = domain2.Orientation(rst);
                    if (Math.Abs(testra) > 1.0e-8)
                    {
                        if (testra < 0.0)
                        {
                            trarst = ConvOrToTra(global::Kerf.Engine.Topology.TopAbs.Reverse(Or));
                        }
                        else if (testra > 0.0)
                        {
                            trarst = ConvOrToTra(Or);
                        }
                        comptra = true;
                        line!.Set(tras, trarst);
                    }
                }
            }
            if (StateS == Blend_OK || StateRst == Blend_OK)
            {
                previousP = curpoint;
                return State;
            }
            if (StateS == Blend_StepTooSmall && StateRst == Blend_StepTooSmall)
            {
                previousP = curpoint;
                if (State == Blend_OK)
                {
                    return Blend_StepTooSmall;
                }
                else
                {
                    return State;
                }
            }
            if (State == Blend_OK)
            {
                return Blend_SamePoints;
            }
            else
            {
                return State;
            }
        }
        return Blend_StepTooLarge;
    }

    private bool CheckInside(Blend_SurfRstFunction Func, out Containment SituOnC, out Containment SituOnS,
                             out bool Decroch)
    {
        math_Vector tolerance = new(1, 3);
        Func.GetTolerance(tolerance, tolpoint3d);
        // face pcurve.
        double w = sol[3];
        if (w < rst.FirstParameter() - tolerance[3] || w > rst.LastParameter() + tolerance[3])
        {
            SituOnC = Out;
        }
        else if (w > rst.FirstParameter() && w < rst.LastParameter())
        {
            SituOnC = In;
        }
        else
        {
            SituOnC = On;
        }

        // face surface
        Point2D p2d = new(sol[1], sol[2]);
        SituOnS = domain1.Classify(p2d, tolpoint2d, false);

        // lost contact
        Vector3D tgs, nors;
        Decroch = Func.Decroch(sol, out tgs, out nors);

        return (SituOnC == In && SituOnS == In && !Decroch);
    }
}
