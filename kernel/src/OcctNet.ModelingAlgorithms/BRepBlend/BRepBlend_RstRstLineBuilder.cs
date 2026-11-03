// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_RstRstLineBuilder.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_RstRstLineBuilder.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_RstRstLineBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.Blend_DecrochStatus;
using static Occt.Blend_Status;
using static Occt.IntSurf_TypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Occt;

/// <summary>
/// Walks a blend between two restrictions (arcs of two surfaces) along a guide, building a
/// <see cref="BRepBlend_Line"/>. The walking stops where the section leaves a restriction's ends (solving the
/// curve/point inverse function) or loses contact with a surface (the surface/curve inverse function, "décroché").
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT's <c>OCCT_DEBUG</c> drawing of the sections and its derivative traces are
/// left out.
/// </remarks>
public class BRepBlend_RstRstLineBuilder
{
    private bool done;
    private BRepBlend_Line? line;
    private readonly math_Vector sol;
    private readonly SurfaceAdaptorBase surf1;
    private readonly TopolTool domain1;
    private readonly SurfaceAdaptorBase surf2;
    private readonly TopolTool domain2;
    private readonly Curve2DAdaptorBase rst1;
    private readonly Curve2DAdaptorBase rst2;
    private double tolpoint3d;
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
    private Blend_DecrochStatus decrochdeb;
    private Blend_DecrochStatus decrochfin;

    public BRepBlend_RstRstLineBuilder(SurfaceAdaptorBase Surf1, Curve2DAdaptorBase Rst1, TopolTool Domain1,
                                       SurfaceAdaptorBase Surf2, Curve2DAdaptorBase Rst2, TopolTool Domain2)
    {
        done = false;
        sol = new math_Vector(1, 2);
        surf1 = Surf1;
        domain1 = Domain1;
        surf2 = Surf2;
        domain2 = Domain2;
        rst1 = Rst1;
        rst2 = Rst2;
        tolpoint3d = 0.0;
        tolgui = 0.0;
        pasmax = 0.0;
        fleche = 0.0;
        param = 0.0;
        rebrou = false;
        iscomplete = false;
        comptra = false;
        sens = 0.0;
        decrochdeb = Blend_NoDecroch;
        decrochfin = Blend_NoDecroch;
    }

    /// <summary>
    /// Walks from the guide's parameter <paramref name="Pdep"/> towards <paramref name="Pmax"/>, starting from the
    /// solution <paramref name="ParDep"/> (solved again first when <paramref name="Appro"/>).
    /// </summary>
    public void Perform(Blend_RstRstFunction Func, Blend_SurfCurvFuncInv Finv1, Blend_CurvPointFuncInv FinvP1,
                        Blend_SurfCurvFuncInv Finv2, Blend_CurvPointFuncInv FinvP2, double Pdep, double Pmax,
                        double MaxStep, double Tol3d, double TolGuide, math_Vector ParDep, double Fleche,
                        bool Appro = false)
    {
        done = false;
        iscomplete = false;
        comptra = false;
        line = new BRepBlend_Line();
        tolpoint3d = Tol3d;
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
            Containment siturst1, siturst2;
            Blend_DecrochStatus decroch;
            math_Vector tolerance = new(1, 2), infbound = new(1, 2), supbound = new(1, 2);
            Func.GetTolerance(tolerance, tolpoint3d);
            Func.GetBounds(infbound, supbound);
            FunctionSetRoot rsnld = new(Func, tolerance, 30);

            rsnld.Perform(Func, ParDep, infbound, supbound);

            if (!rsnld.IsDone)
            {
                return;
            }
            rsnld.Root(sol);
            if (!CheckInside(Func, out siturst1, out siturst2, out decroch))
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
        U = previousP.ParameterOnC1();
        V = previousP.ParameterOnC2();
        BRepBlend_Extremity ptf1 = new(previousP.PointOnC1(), U, previousP.Parameter(), tolpoint3d);
        BRepBlend_Extremity ptf2 = new(previousP.PointOnC2(), V, previousP.Parameter(), tolpoint3d);
        if (!previousP.IsTangencyPoint())
        {
            ptf1.SetTangent(previousP.TangentOnC1());
            ptf2.SetTangent(previousP.TangentOnC2());
        }

        if (sens > 0.0)
        {
            line.SetStartPoints(ptf1, ptf2);
        }
        else
        {
            line.SetEndPoints(ptf1, ptf2);
        }

        InternalPerform(Func, Finv1, FinvP1, Finv2, FinvP2, Pmax);
        done = true;
    }

    /// <summary>
    /// Solves a first section at <paramref name="Pdep"/> and moves it to where the walking stops (the restrictions'
    /// ends or a lost contact): <paramref name="Psol"/> and <paramref name="ParSol"/> receive it.
    /// </summary>
    public bool PerformFirstSection(Blend_RstRstFunction Func, Blend_SurfCurvFuncInv Finv1,
                                    Blend_CurvPointFuncInv FinvP1, Blend_SurfCurvFuncInv Finv2,
                                    Blend_CurvPointFuncInv FinvP2, double Pdep, double Pmax, math_Vector ParDep,
                                    double Tol3d, double TolGuide, bool RecRst1, bool RecP1, bool RecRst2, bool RecP2,
                                    ref double Psol, math_Vector ParSol)
    {
        done = false;
        iscomplete = false;
        comptra = false;
        line = new BRepBlend_Line();
        tolpoint3d = Tol3d;
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

        bool recadp1, recadp2, recadrst1, recadrst2;
        double wp1, wp2, wrst1, wrst2;
        Blend_Status State = Blend_OnRst12;
        double trst11 = 0.0, trst12 = 0.0, trst21 = 0.0, trst22 = 0.0;
        math_Vector infbound = new(1, 2), supbound = new(1, 2), tolerance = new(1, 2);
        math_Vector solinvp1 = new(1, 2), solinvp2 = new(1, 2), solinvrst1 = new(1, 3), solinvrst2 = new(1, 3);
        HVertex? Vtxp1 = null, Vtxp2 = null, Vtxrst1 = null, Vtxrst2 = null;
        bool IsVtxp1 = false, IsVtxp2 = false, IsVtxrst1 = false, IsVtxrst2 = false;
        wp1 = wp2 = wrst1 = wrst2 = Pmax;
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

        recadrst1 = RecRst1 && Recadre1(Func, Finv1, solinvrst1, ref IsVtxrst1, ref Vtxrst1);
        if (recadrst1)
        {
            wrst1 = solinvrst1[1];
        }

        recadp1 = RecP1 && Recadre1(FinvP1, solinvp1, ref IsVtxp1, ref Vtxp1);
        if (recadp1)
        {
            wp1 = solinvp1[1];
        }

        recadrst2 = RecRst2 && Recadre2(Func, Finv2, solinvrst2, ref IsVtxrst2, ref Vtxrst2);
        if (recadrst2)
        {
            wrst2 = solinvrst2[1];
        }

        recadp2 = RecP2 && Recadre2(FinvP2, solinvp2, ref IsVtxp2, ref Vtxp2);
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
            if (sens * (wrst1 - wp1) > tolgui)
            { // at first one leaves the domain
                wrst1 = wp1;
                trst12 = solinvp1[2];
                trst11 = BRepBlend_BlendTool.Parameter(Vtxp1!, rst1);
                IsVtxrst2 = IsVtxp1; // OCCT: the second restriction's vertex is set where the first's is meant
                Vtxrst2 = Vtxp1;
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
            trst11 = BRepBlend_BlendTool.Parameter(Vtxp1!, rst1);
            IsVtxrst1 = IsVtxp1;
            Vtxrst1 = Vtxp1;
        }
        else if (recadrst1)
        {
            trst11 = solinvrst1[3];
            trst12 = solinvrst1[2];
        }

        // it is checked if the contact was lost or domain 2 was left
        if (recadp2 && recadrst2)
        {
            if (sens * (wrst2 - wp2) > tolgui)
            { // at first one leaves the domain
                wrst2 = wp2;
                trst21 = solinvp2[2];
                trst22 = BRepBlend_BlendTool.Parameter(Vtxp2!, rst2);
                IsVtxrst2 = IsVtxp2;
                Vtxrst2 = Vtxp2;
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
            trst22 = BRepBlend_BlendTool.Parameter(Vtxp2!, rst2);
            IsVtxrst2 = IsVtxp2;
            Vtxrst2 = Vtxp2;
        }
        else if (recadrst2)
        {
            trst22 = solinvrst2[3];
            trst21 = solinvrst2[2];
        }

        // it is checked on which curve the contact is lost earlier
        if (recadrst1 && recadrst2)
        {
            if (Math.Abs(wrst1 - wrst2) < tolgui)
            {
                State = Blend_OnRst12;
                param = 0.5 * (wrst1 + wrst2);
                sol[1] = trst11;
                sol[2] = trst22;
            }
            else if (sens * (wrst1 - wrst2) < 0)
            {
                // contact lost on Rst1
                State = Blend_OnRst1;
                param = wrst1;
                sol[1] = trst11;
                sol[2] = trst12;
            }
            else
            {
                // contact lost on rst2
                State = Blend_OnRst2;
                param = wrst2;
                sol[1] = trst21;
                sol[2] = trst22;
            }
            Func.Set(param);
        }
        else if (recadrst1)
        {
            // ground on rst1
            State = Blend_OnRst1;
            param = wrst1;
            sol[1] = trst11;
            sol[2] = trst12;
            Func.Set(param);
        }
        else if (recadrst2)
        {
            // ground on rst2
            State = Blend_OnRst2;
            param = wrst2;
            sol[1] = trst21;
            sol[2] = trst22;
            Func.Set(param);
        }
        // it is checked on which curves one leaves first
        else if (recadp1 && recadp2)
        {
            if (Math.Abs(wrst1 - wrst2) < tolgui)
            {
                State = Blend_OnRst12;
                param = 0.5 * (wrst1 + wrst2);
                sol[1] = trst11;
                sol[2] = trst22;
            }
            else if (sens * (wrst1 - wrst2) < 0)
            {
                // sol on Rst1
                State = Blend_OnRst1;
                param = wrst1;
                sol[1] = trst11;
                sol[2] = trst12;
            }
            else
            {
                // ground on rst2
                State = Blend_OnRst2;
                param = wrst2;
                sol[1] = trst21;
                sol[2] = trst22;
            }
            Func.Set(param);
        }
        else if (recadp1)
        {
            // ground on rst1
            State = Blend_OnRst1;
            param = wrst1;
            sol[1] = trst11;
            sol[2] = trst12;
            Func.Set(param);
        }
        else if (recadp2)
        {
            // ground on rst2
            State = Blend_OnRst2;
            param = wrst2;
            sol[1] = trst21;
            sol[2] = trst22;
            Func.Set(param);
        }

        State = TestArret(Func, false, State);
        Psol = param;
        ParSol.Initialized(sol);
        return true;
    }

    /// <summary>Walks the other way from the line's start, down to <paramref name="Pmin"/>.</summary>
    public bool Complete(Blend_RstRstFunction Func, Blend_SurfCurvFuncInv Finv1, Blend_CurvPointFuncInv FinvP1,
                         Blend_SurfCurvFuncInv Finv2, Blend_CurvPointFuncInv FinvP2, double Pmin)
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
        sol[1] = previousP.ParameterOnC1();
        sol[2] = previousP.ParameterOnC2();

        InternalPerform(Func, Finv1, FinvP1, Finv2, FinvP2, Pmin);
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

    public bool Decroch1Start() => decrochdeb == Blend_DecrochRst1 || decrochdeb == Blend_DecrochBoth;

    public bool Decroch1End() => decrochfin == Blend_DecrochRst1 || decrochfin == Blend_DecrochBoth;

    public bool Decroch2Start() => decrochdeb == Blend_DecrochRst2 || decrochdeb == Blend_DecrochBoth;

    public bool Decroch2End() => decrochfin == Blend_DecrochRst2 || decrochfin == Blend_DecrochBoth;

    // The walking without the extremities.
    private void InternalPerform(Blend_RstRstFunction Func, Blend_SurfCurvFuncInv Finv1,
                                 Blend_CurvPointFuncInv FinvP1, Blend_SurfCurvFuncInv Finv2,
                                 Blend_CurvPointFuncInv FinvP2, double Bound)
    {
        double stepw = pasmax;
        int nbp = line!.NbPoints();
        if (nbp >= 2)
        { // The last step is redone if it is not too small.
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
        double trst11 = 0.0, trst12 = 0.0, trst21 = 0.0, trst22 = 0.0;
        Containment situonc1 = Unknown, situonc2 = Unknown;
        Blend_DecrochStatus decroch = Blend_NoDecroch;
        bool Arrive, recadp1, recadp2, recadrst1, recadrst2, echecrecad;
        double wp1, wp2, wrst1, wrst2;
        math_Vector infbound = new(1, 2), supbound = new(1, 2);
        math_Vector parinit = new(1, 2), tolerance = new(1, 2);
        math_Vector solinvp1 = new(1, 2), solinvp2 = new(1, 2), solinvrst1 = new(1, 3), solinvrst2 = new(1, 3);
        HVertex? Vtxp1 = null, Vtxp2 = null, Vtxrst1 = null, Vtxrst2 = null;
        bool IsVtxp1 = false, IsVtxp2 = false, IsVtxrst1 = false, IsVtxrst2 = false;
        BRepBlend_Extremity Extrst1 = new(), Extrst2 = new();

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
                if (!CheckInside(Func, out situonc1, out situonc2, out decroch) && line.NbPoints() == 1)
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
                wp1 = wp2 = wrst1 = wrst2 = Bound;
                recadp1 = recadp2 = recadrst1 = recadrst2 = false;
                echecrecad = false;
                if (situonc1 != In)
                {
                    // pb inversion rst/rst
                    recadp1 = Recadre1(FinvP1, solinvp1, ref IsVtxp1, ref Vtxp1);
                    if (recadp1)
                    {
                        wp1 = solinvp1[1];
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }

                if (situonc2 != In)
                {
                    // pb inversion point/surf
                    recadp2 = Recadre2(FinvP2, solinvp2, ref IsVtxp2, ref Vtxp2);
                    if (recadp2)
                    {
                        wp2 = solinvp2[1];
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }

                if (decroch == Blend_DecrochRst1 || decroch == Blend_DecrochBoth)
                {
                    // pb inversion rst1/surf1
                    recadrst1 = Recadre1(Func, Finv1, solinvrst1, ref IsVtxrst1, ref Vtxrst1);
                    if (recadrst1)
                    {
                        wrst1 = solinvrst1[1];
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }

                if (decroch == Blend_DecrochRst2 || decroch == Blend_DecrochBoth)
                {
                    // pb inverse rst2/surf2
                    recadrst2 = Recadre2(Func, Finv2, solinvrst2, ref IsVtxrst2, ref Vtxrst2);
                    if (recadrst2)
                    {
                        wrst2 = solinvrst2[1];
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }

                decroch = Blend_NoDecroch;
                if (recadp1 || recadp2 || recadrst1 || recadrst2)
                {
                    echecrecad = false;
                }

                if (!echecrecad)
                {
                    // it is checked if the contact was lost or domain 1 was left
                    if (recadp1 && recadrst1)
                    {
                        if (sens * (wrst1 - wp1) > tolgui)
                        { // first one leaves the domain
                            wrst1 = wp1;
                            trst12 = solinvp1[2];
                            trst11 = BRepBlend_BlendTool.Parameter(Vtxp1!, rst1);
                            IsVtxrst2 = IsVtxp1; // OCCT: the second restriction's vertex is set where the first's is meant
                            Vtxrst2 = Vtxp1;
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
                        trst11 = BRepBlend_BlendTool.Parameter(Vtxp1!, rst1);
                        IsVtxrst1 = IsVtxp1;
                        Vtxrst1 = Vtxp1;
                    }
                    else if (recadrst1)
                    {
                        trst11 = solinvrst1[3];
                        trst12 = solinvrst1[2];
                    }

                    // it is checked if the contact was lost or domain 2 was left
                    if (recadp2 && recadrst2)
                    {
                        if (sens * (wrst2 - wp2) > tolgui)
                        { // first one leaves the domain
                            wrst2 = wp2;
                            trst21 = solinvp2[2];
                            trst22 = BRepBlend_BlendTool.Parameter(Vtxp2!, rst2);
                            IsVtxrst2 = IsVtxp2;
                            Vtxrst2 = Vtxp2;
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
                        trst22 = BRepBlend_BlendTool.Parameter(Vtxp2!, rst2);
                        IsVtxrst2 = IsVtxp2;
                        Vtxrst2 = Vtxp2;
                    }
                    else if (recadrst2)
                    {
                        trst22 = solinvrst2[3];
                        trst21 = solinvrst2[2];
                    }

                    // it is checked on which curve the contact is lost earlier
                    if (recadrst1 && recadrst2)
                    {
                        if (Math.Abs(wrst1 - wrst2) < tolgui)
                        {
                            State = Blend_OnRst12;
                            decroch = Blend_DecrochBoth;
                            param = 0.5 * (wrst1 + wrst2);
                            sol[1] = trst11;
                            sol[2] = trst22;
                        }
                        else if (sens * (wrst1 - wrst2) < 0)
                        {
                            // contact is lost on Rst1
                            State = Blend_OnRst1;
                            decroch = Blend_DecrochRst1;
                            param = wrst1;
                            sol[1] = trst11;
                            sol[2] = trst12;
                        }
                        else
                        {
                            // contact is lost on rst2
                            State = Blend_OnRst2;
                            decroch = Blend_DecrochRst2;
                            param = wrst2;
                            sol[1] = trst21;
                            sol[2] = trst22;
                        }
                        Func.Set(param);
                    }
                    else if (recadrst1)
                    {
                        // ground on rst1
                        State = Blend_OnRst1;
                        decroch = Blend_DecrochRst1;
                        param = wrst1;
                        sol[1] = trst11;
                        sol[2] = trst12;
                        Func.Set(param);
                    }
                    else if (recadrst2)
                    {
                        // ground on rst2
                        State = Blend_OnRst2;
                        decroch = Blend_DecrochRst2;
                        param = wrst2;
                        sol[1] = trst21;
                        sol[2] = trst22;
                        Func.Set(param);
                    }
                    //  it is checked on which curve the contact is lost earlier
                    else if (recadp1 && recadp2)
                    {
                        if (Math.Abs(wrst1 - wrst2) < tolgui)
                        {
                            State = Blend_OnRst12;
                            param = 0.5 * (wrst1 + wrst2);
                            sol[1] = trst11;
                            sol[2] = trst22;
                        }
                        else if (sens * (wrst1 - wrst2) < 0)
                        {
                            // ground on Rst1
                            State = Blend_OnRst1;
                            param = wrst1;
                            sol[1] = trst11;
                            sol[2] = trst12;
                        }
                        else
                        {
                            // ground on rst2
                            State = Blend_OnRst2;
                            param = wrst2;
                            sol[1] = trst21;
                            sol[2] = trst22;
                        }
                        Func.Set(param);
                    }
                    else if (recadp1)
                    {
                        // ground on rst1
                        State = Blend_OnRst1;
                        param = wrst1;
                        sol[1] = trst11;
                        sol[2] = trst12;
                        Func.Set(param);
                    }
                    else if (recadp2)
                    {
                        // ground on rst2
                        State = Blend_OnRst2;
                        param = wrst2;
                        sol[1] = trst21;
                        sol[2] = trst22;
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
                    // reframing failed. Leave with PointsConfondus
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
                        Extrst1.SetValue(previousP.PointOnC1(), previousP.ParameterOnC1(), previousP.Parameter(),
                                         tolpoint3d);
                        MakeExtremity(ref Extrst2, false, rst2, sol[2], IsVtxrst2, Vtxrst2);
                        // Show that end is on Bound.
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
                        Extrst1.SetValue(previousP.PointOnC1(), previousP.ParameterOnC1(), previousP.Parameter(),
                                         tolpoint3d);
                        Extrst2.SetValue(previousP.PointOnC2(), previousP.ParameterOnC2(), previousP.Parameter(),
                                         tolpoint3d);
                        Arrive = true;
                    }
                    else
                    {
                        param = parprec + sens * stepw; // there is no risk to exceed Bound.
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
                        Extrst1.SetValue(previousP.PointOnC1(), previousP.ParameterOnC1(), previousP.Parameter(),
                                         tolpoint3d);
                        MakeExtremity(ref Extrst2, false, rst2, sol[2], IsVtxrst2, Vtxrst2);
                        // Indicate that end is on Bound.
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
                    MakeExtremity(ref Extrst1, true, rst1, sol[1], IsVtxrst1, Vtxrst1);
                    MakeExtremity(ref Extrst2, false, rst2, sol[2], IsVtxrst2, Vtxrst2);
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

                    MakeExtremity(ref Extrst1, true, rst1, sol[1], IsVtxrst1, Vtxrst1);
                    MakeExtremity(ref Extrst2, false, rst2, sol[2], IsVtxrst2, Vtxrst2);
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

                    MakeExtremity(ref Extrst1, true, rst1, sol[1], IsVtxrst1, Vtxrst1);
                    MakeExtremity(ref Extrst2, false, rst2, sol[2], IsVtxrst2, Vtxrst2);
                    Arrive = true;
                }
                break;

                case Blend_SamePoints:
                {
                    // Stop
                    Extrst1.SetValue(previousP.PointOnC1(), previousP.ParameterOnC1(), previousP.Parameter(),
                                     tolpoint3d);
                    Extrst2.SetValue(previousP.PointOnC2(), previousP.ParameterOnC2(), previousP.Parameter(),
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
                    line.SetEndPoints(Extrst1, Extrst2);
                    decrochfin = decroch;
                }
                else
                {
                    line.SetStartPoints(Extrst1, Extrst2);
                    decrochdeb = decroch;
                }
            }
        }
    }

    // IsVtx and Vtx are written only on success (OCCT's references), so they are ref parameters.
    private bool Recadre1(Blend_RstRstFunction Func, Blend_SurfCurvFuncInv Finv, math_Vector Solinv, ref bool IsVtx,
                          ref HVertex? Vtx)
    {
        math_Vector toler = new(1, 3), infb = new(1, 3), supb = new(1, 3);
        Finv.GetTolerance(toler, tolpoint3d);
        Finv.GetBounds(infb, supb);
        Solinv[1] = param;
        Solinv[2] = sol[2];
        Solinv[3] = sol[1];

        // The point where contact is not lost is found
        FunctionSetRoot rsnld = new(Finv, toler, 30);
        rsnld.Perform(Finv, Solinv, infb, supb);
        if (!rsnld.IsDone)
        {
            return false;
        }

        rsnld.Root(Solinv);

        // It is necessary to check if the function value meets the second restriction
        if (Finv.IsSolution(Solinv, tolpoint3d))
        {
            double w = Solinv[2];
            if (w < rst2.FirstParameter() - toler[2] || w > rst2.LastParameter() + toler[2])
            {
                return false;
            }

            // it is checked if it is on a Vertex
            domain1.Initialize(rst1);
            domain1.InitVertexIterator();
            IsVtx = !domain1.MoreVertex();
            while (!IsVtx)
            {
                Vtx = domain1.Vertex();
                if (Math.Abs(BRepBlend_BlendTool.Parameter(Vtx, rst1) - Solinv[3])
                    <= BRepBlend_BlendTool.Tolerance(Vtx, rst1))
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
            // The section is recalculated by direct solution, otherwise return incoherences between the parameter
            // and the ground caused by yawn.

            math_Vector infbound = new(1, 2), supbound = new(1, 2);
            math_Vector parinit = new(1, 2), tolerance = new(1, 2);
            Func.GetTolerance(tolerance, tolpoint3d);
            Func.GetBounds(infbound, supbound);

            FunctionSetRoot rsnld2 = new(Func, tolerance, 30);
            parinit[1] = Solinv[3];
            parinit[2] = Solinv[2];
            Func.Set(Solinv[1]);
            rsnld2.Perform(Func, parinit, infbound, supbound);
            if (!rsnld2.IsDone)
            {
                return false;
            }
            rsnld2.Root(parinit);
            Solinv[2] = parinit[2];
            Solinv[3] = parinit[1];
            return true;
        }
        return false;
    }

    private bool Recadre2(Blend_RstRstFunction Func, Blend_SurfCurvFuncInv Finv, math_Vector Solinv, ref bool IsVtx,
                          ref HVertex? Vtx)
    {
        math_Vector toler = new(1, 3), infb = new(1, 3), supb = new(1, 3);
        Finv.GetTolerance(toler, tolpoint3d);
        Finv.GetBounds(infb, supb);
        Solinv[1] = param;
        Solinv[2] = sol[1];
        Solinv[3] = sol[2];

        FunctionSetRoot rsnld = new(Finv, toler, 30);
        rsnld.Perform(Finv, Solinv, infb, supb);
        if (!rsnld.IsDone)
        {
            return false;
        }

        rsnld.Root(Solinv);

        // It is necessary to check the value of the function
        if (Finv.IsSolution(Solinv, tolpoint3d))
        {
            double w = Solinv[2];
            if (w < rst1.FirstParameter() - toler[2] || w > rst1.LastParameter() + toler[2])
            {
                return false;
            }

            domain2.Initialize(rst2);
            domain2.InitVertexIterator();
            IsVtx = !domain2.MoreVertex();
            while (!IsVtx)
            {
                Vtx = domain2.Vertex();
                if (Math.Abs(BRepBlend_BlendTool.Parameter(Vtx, rst2) - Solinv[3])
                    <= BRepBlend_BlendTool.Tolerance(Vtx, rst2))
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
            // The section is recalculated by direct solution, otherwise return incoherences between the parameter
            // and the ground caused by yawn.

            math_Vector infbound = new(1, 2), supbound = new(1, 2);
            math_Vector parinit = new(1, 2), tolerance = new(1, 2);
            Func.GetTolerance(tolerance, tolpoint3d);
            Func.GetBounds(infbound, supbound);

            FunctionSetRoot rsnld2 = new(Func, tolerance, 30);
            parinit[1] = Solinv[2];
            parinit[2] = Solinv[3];
            Func.Set(Solinv[1]);
            rsnld2.Perform(Func, parinit, infbound, supbound);
            if (!rsnld2.IsDone)
            {
                return false;
            }
            rsnld2.Root(parinit);
            Solinv[2] = parinit[1];
            Solinv[3] = parinit[2];
            return true;
        }
        return false;
    }

    private bool Recadre1(Blend_CurvPointFuncInv FinvP, math_Vector Solinv, ref bool IsVtx, ref HVertex? Vtx)
    {
        // One is located on the last or the first point, following the direction of processing.
        Point2D p2drst1;
        double firstrst1 = rst1.FirstParameter();
        double lastrst1 = rst1.LastParameter();
        double upoint = firstrst1;

        if ((sol[1] - firstrst1) > (lastrst1 - sol[1]))
        {
            upoint = lastrst1;
        }
        p2drst1 = rst1.Value(upoint);
        Point3D thepoint = surf1.Value(p2drst1.X(), p2drst1.Y());

        FinvP.Set(thepoint);
        math_Vector toler = new(1, 2), infb = new(1, 2), supb = new(1, 2);
        FinvP.GetTolerance(toler, tolpoint3d);
        FinvP.GetBounds(infb, supb);
        Solinv[1] = param;
        Solinv[2] = sol[2];

        FunctionSetRoot rsnld = new(FinvP, toler, 30);
        rsnld.Perform(FinvP, Solinv, infb, supb);
        if (!rsnld.IsDone)
        {
            return false;
        }
        rsnld.Root(Solinv);

        if (FinvP.IsSolution(Solinv, tolpoint3d))
        {
            Point2D p2drst2 = rst2.Value(Solinv[2]);
            Containment situ = domain2.Classify(p2drst2, toler[2], false);
            if ((situ != In) && (situ != On))
            {
                return false;
            }
            domain1.Initialize(rst1);
            domain1.InitVertexIterator();
            IsVtx = !domain1.MoreVertex();
            while (!IsVtx)
            {
                Vtx = domain1.Vertex();
                if (Math.Abs(BRepBlend_BlendTool.Parameter(Vtx, rst1) - upoint)
                    <= BRepBlend_BlendTool.Tolerance(Vtx, rst1))
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

    private bool Recadre2(Blend_CurvPointFuncInv FinvP, math_Vector Solinv, ref bool IsVtx, ref HVertex? Vtx)
    {
        // One is located on the last or the first point, following the direction of processing.
        Point2D p2drst2;
        double firstrst2 = rst2.FirstParameter();
        double lastrst2 = rst2.LastParameter();
        double vpoint = firstrst2;

        if ((sol[2] - firstrst2) > (lastrst2 - sol[2]))
        {
            vpoint = lastrst2;
        }
        p2drst2 = rst2.Value(vpoint);
        Point3D thepoint = surf2.Value(p2drst2.X(), p2drst2.Y());

        FinvP.Set(thepoint);
        math_Vector toler = new(1, 2), infb = new(1, 2), supb = new(1, 2);
        FinvP.GetTolerance(toler, tolpoint3d);
        FinvP.GetBounds(infb, supb);
        Solinv[1] = param;
        Solinv[2] = sol[1];

        FunctionSetRoot rsnld = new(FinvP, toler, 30);
        rsnld.Perform(FinvP, Solinv, infb, supb);
        if (!rsnld.IsDone)
        {
            return false;
        }
        rsnld.Root(Solinv);

        if (FinvP.IsSolution(Solinv, tolpoint3d))
        {
            Point2D p2drst1 = rst1.Value(Solinv[2]);
            Containment situ = domain1.Classify(p2drst1, toler[2], false);
            if ((situ != In) && (situ != On))
            {
                return false;
            }
            domain2.Initialize(rst2);
            domain2.InitVertexIterator();
            IsVtx = !domain2.MoreVertex();
            while (!IsVtx)
            {
                Vtx = domain2.Vertex();
                if (Math.Abs(BRepBlend_BlendTool.Parameter(Vtx, rst2) - vpoint)
                    <= BRepBlend_BlendTool.Tolerance(Vtx, rst2))
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
                tgline = previousP.TangentOnC1();
            }
            else
            {
                tgline = new Vector3D(prevprev.PointOnC1(), previousP.PointOnC1());
            }
        }
        else
        {
            surf2.D1(p2d.X(), p2d.Y(), out pbid, out d1u, out d1v);
            if (!computetranstionaveclacorde)
            {
                tgline = previousP.TangentOnC2();
            }
            else
            {
                tgline = new Vector3D(prevprev.PointOnC2(), previousP.PointOnC2());
            }
        }

        tgrst.SetLinearForm(dp2d.X(), d1u, dp2d.Y(), d1v);
        normale = d1u.Crossed(d1v);

        IntSurf.MakeTransition(tgline, tgrst, new Direction3D(normale), ref TLine, ref TArc);
    }

    // Produces the extremity of a curve.
    private void MakeExtremity(ref BRepBlend_Extremity Extrem, bool OnFirst, Curve2DAdaptorBase Arc, double Param,
                               bool IsVtx, HVertex? Vtx)
    {
        IntSurf_Transition Tline = new(), Tarc = new();
        double prm;
        TopolTool Iter;
        if (OnFirst)
        {
            Extrem.SetValue(previousP.PointOnC1(), sol[1], previousP.Parameter(), tolpoint3d);
            if (!previousP.IsTangencyPoint())
            {
                Extrem.SetTangent(previousP.TangentOnC1());
            }
            Iter = domain1;
        }
        else
        {
            Extrem.SetValue(previousP.PointOnC2(), sol[2], previousP.Parameter(), tolpoint3d);
            if (!previousP.IsTangencyPoint())
            {
                Extrem.SetTangent(previousP.TangentOnC1()); // OCCT: the first curve's tangent on the second's extremity
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

    private Blend_Status CheckDeflectionOnRst1(in Blend_Point CurPoint)
    {
        // Controls 3d of Blend_CSWalking.

        // rule by tests in U4 corresponds to 11.478
        const double CosRef3D = 0.98;
        double Cosi, Cosi2;
        bool curpointistangent = CurPoint.IsTangencyPoint();
        bool prevpointistangent = previousP.IsTangencyPoint();

        Point3D Psurf = CurPoint.PointOnC1();
        Vector3D Tgsurf = new();
        if (!curpointistangent)
        {
            Tgsurf = CurPoint.TangentOnC1();
        }
        Point3D prevP = previousP.PointOnC1();
        Vector3D prevTg = new();
        if (!prevpointistangent)
        {
            prevTg = previousP.TangentOnC1();
        }
        double Norme;
        double prevNorme = 0.0;
        Vector3D Corde = new(prevP, Psurf);
        Norme = Corde.SquareMagnitude();
        if (!prevpointistangent)
        {
            prevNorme = prevTg.SquareMagnitude();
        }

        double toler3d = 0.01 * tolpoint3d;
        if (Norme <= toler3d * toler3d)
        {
            // it can be necessary to force the same point
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

    private Blend_Status CheckDeflectionOnRst2(in Blend_Point CurPoint)
    {
        // 3D Controls of Blend_CSWalking.

        // rule by tests in U4 corresponding to 11.478 d
        const double CosRef3D = 0.98;
        double Cosi, Cosi2;
        bool curpointistangent = CurPoint.IsTangencyPoint();
        bool prevpointistangent = previousP.IsTangencyPoint();

        Point3D Psurf = CurPoint.PointOnC2();
        Vector3D Tgsurf = new();

        if (!curpointistangent)
        {
            Tgsurf = CurPoint.TangentOnC2();
        }
        Point3D prevP = previousP.PointOnC2();
        Vector3D prevTg = new();
        if (!prevpointistangent)
        {
            prevTg = previousP.TangentOnC2();
        }
        double Norme;
        double prevNorme = 0.0;
        Vector3D Corde = new(prevP, Psurf);
        Norme = Corde.SquareMagnitude();
        if (!prevpointistangent)
        {
            prevNorme = prevTg.SquareMagnitude();
        }

        double toler3d = 0.01 * tolpoint3d;
        if (Norme <= toler3d * toler3d)
        {
            // it can be necessary to force the same point
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

    private Blend_Status TestArret(Blend_RstRstFunction Func, bool TestDeflection, Blend_Status State)
    {
        Point3D ptrst1, ptrst2;
        Point2D pt2drst1, pt2drst2;
        Vector3D tgrst1, tgrst2;
        Vector2D tg2drst1 = new(), tg2drst2 = new();
        Blend_Status StateRst1, StateRst2;
        IntSurf_TypeTrans trarst1 = IntSurf_Undecided, trarst2 = IntSurf_Undecided;
        Blend_Point curpoint = new();

        if (Func.IsSolution(sol, tolpoint3d))
        {
            bool curpointistangent = Func.IsTangencyPoint();
            ptrst1 = Func.PointOnRst1();
            ptrst2 = Func.PointOnRst2();
            pt2drst1 = Func.Pnt2dOnRst1();
            pt2drst2 = Func.Pnt2dOnRst2();

            if (curpointistangent)
            {
                curpoint.SetValue(ptrst1, ptrst2, param, pt2drst1.X(), pt2drst1.Y(), pt2drst2.X(), pt2drst2.Y(), sol[1],
                                  sol[2]);
            }
            else
            {
                tgrst1 = Func.TangentOnRst1();
                tgrst2 = Func.TangentOnRst2();
                tg2drst1 = Func.Tangent2dOnRst1();
                tg2drst2 = Func.Tangent2dOnRst2();
                curpoint.SetValue(ptrst1, ptrst2, param, pt2drst1.X(), pt2drst1.Y(), pt2drst2.X(), pt2drst2.Y(), sol[1],
                                  sol[2], tgrst1, tgrst2, tg2drst1, tg2drst2);
            }
            if (TestDeflection)
            {
                StateRst1 = CheckDeflectionOnRst1(curpoint);
                StateRst2 = CheckDeflectionOnRst2(curpoint);
            }
            else
            {
                StateRst1 = StateRst2 = Blend_OK;
            }
            if (StateRst1 == Blend_Backward)
            {
                StateRst1 = Blend_StepTooLarge;
                rebrou = true;
            }
            if (StateRst2 == Blend_Backward)
            {
                StateRst2 = Blend_StepTooLarge;
                rebrou = true;
            }
            if (StateRst1 == Blend_StepTooLarge || StateRst2 == Blend_StepTooLarge)
            {
                return Blend_StepTooLarge;
            }

            if (!comptra && !curpointistangent)
            {
                Point2D p2drstref;
                Vector2D tg2drstref;
                rst1.D1(sol[1], out p2drstref, out tg2drstref);
                double testra = tg2drst1.Dot(tg2drstref);
                Orientation Or = domain1.Orientation(rst1);

                if (Math.Abs(testra) > tolpoint3d)
                {
                    if (testra < 0.0)
                    {
                        trarst1 = ConvOrToTra(global::Kerf.Engine.Topology.TopAbs.Reverse(Or));
                    }
                    else if (testra > 0.0)
                    {
                        trarst1 = ConvOrToTra(Or);
                    }

                    rst2.D1(sol[2], out p2drstref, out tg2drstref);
                    testra = tg2drst2.Dot(tg2drstref);

                    Or = domain2.Orientation(rst2);
                    if (Math.Abs(testra) > tolpoint3d)
                    {
                        if (testra < 0.0)
                        {
                            trarst2 = ConvOrToTra(global::Kerf.Engine.Topology.TopAbs.Reverse(Or));
                        }
                        else if (testra > 0.0)
                        {
                            trarst2 = ConvOrToTra(Or);
                        }
                        comptra = true;
                        line!.Set(trarst1, trarst2);
                    }
                }
            }
            if (StateRst1 == Blend_OK || StateRst2 == Blend_OK)
            {
                previousP = curpoint;
                return State;
            }
            if (StateRst1 == Blend_StepTooSmall && StateRst2 == Blend_StepTooSmall)
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

    private bool CheckInside(Blend_RstRstFunction Func, out Containment SituOnC1, out Containment SituOnC2,
                             out Blend_DecrochStatus Decroch)
    {
        math_Vector tolerance = new(1, 2);
        Func.GetTolerance(tolerance, tolpoint3d);

        // face pcurve 1.
        double v = sol[1];
        if (v < rst1.FirstParameter() - tolerance[2] || v > rst1.LastParameter() + tolerance[2])
        {
            SituOnC1 = Out;
        }
        else if (v > rst1.FirstParameter() && v < rst1.LastParameter())
        {
            SituOnC1 = In;
        }
        else
        {
            SituOnC1 = On;
        }

        // face pcurve 2.
        v = sol[2];
        if (v < rst2.FirstParameter() - tolerance[2] || v > rst2.LastParameter() + tolerance[2])
        {
            SituOnC2 = Out;
        }
        else if (v > rst2.FirstParameter() && v < rst2.LastParameter())
        {
            SituOnC2 = In;
        }
        else
        {
            SituOnC2 = On;
        }

        // lost contact
        Vector3D tgrst1, norst1, tgrst2, norst2;
        Decroch = Func.Decroch(sol, out tgrst1, out norst1, out tgrst2, out norst2);

        return (SituOnC1 == In && SituOnC2 == In && Decroch == Blend_NoDecroch);
    }
}
