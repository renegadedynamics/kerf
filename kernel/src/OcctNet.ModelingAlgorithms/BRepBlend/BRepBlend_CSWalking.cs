// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_CSWalking.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_CSWalking.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Blend_Status;
using static Occt.IntSurf_TypeTrans;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Walks a blend between a curve and a surface (a <see cref="Blend_CSFunction"/>, such as BlendFunc_CSConstRad)
/// along a guide, building a <see cref="BRepBlend_Line"/>.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT never wrote its recadrage on the surface's restrictions (the code is
/// commented out), so the walking stops with "same points" when it leaves the face, and OCCT prints a line to the
/// standard output there, left out. Its <c>OCCT_DEBUG</c> drawing of the sections is left out too.
/// </remarks>
public class BRepBlend_CSWalking
{
    private bool done;
    private BRepBlend_Line? line;
    private readonly Adaptor3d_Surface surf;
    private readonly Adaptor3d_Curve curv;
    private Adaptor3d_TopolTool domain;
    private double tolpoint3d;
    private double tolgui;
    private double pasmax;
    private double fleche;
    private double param;
    private double firstparam;
    private NCollection_HArray1<double>? firstsol;
    private Blend_Point previousP = new();
#pragma warning disable CS0414 // set and never read, as in OCCT
    private bool rebrou;
#pragma warning restore CS0414
    private bool iscomplete;
    private bool comptra;
    private double sens;

    public BRepBlend_CSWalking(Adaptor3d_Curve Curv, Adaptor3d_Surface Surf, Adaptor3d_TopolTool Domain)
    {
        done = false;
        surf = Surf;
        curv = Curv;
        domain = Domain;
    }

    /// <summary>
    /// Walks from the guide's parameter <paramref name="Pdep"/> towards <paramref name="Pmax"/>, starting from the
    /// solution <paramref name="ParDep"/> (solved again first when <paramref name="Appro"/>).
    /// </summary>
    public void Perform(Blend_CSFunction Func, double Pdep, double Pmax, double MaxStep, double Tol3d,
                        double TolGuide, math_Vector ParDep, double Fleche, bool Appro = false)
    {
        done = false;
        iscomplete = false;
        comptra = false;
        line = new BRepBlend_Line();
        int Nbvar = Func.NbVariables();
        tolpoint3d = Tol3d;
        tolgui = Math.Abs(TolGuide);
        fleche = Math.Abs(Fleche);
        rebrou = false;
        pasmax = Math.Abs(MaxStep);
        math_Vector sol = new(1, Nbvar);

        firstsol = new NCollection_HArray1<double>(1, Nbvar);

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
        firstparam = Pdep;
        Func.Set(param);

        if (Appro)
        {
            TopAbs_State situ;
            math_Vector tolerance = new(1, Nbvar), infbound = new(1, Nbvar), supbound = new(1, Nbvar);
            Func.GetTolerance(tolerance, tolpoint3d);
            Func.GetBounds(infbound, supbound);
            math_FunctionSetRoot rsnld = new(Func, tolerance, 30);

            rsnld.Perform(Func, ParDep, infbound, supbound);

            if (!rsnld.IsDone())
            {
                return;
            }
            rsnld.Root(sol);

            situ = domain.Classify(Func.Pnt2d(), Min(tolerance[1], tolerance[2]));

            if (situ != TopAbs_IN)
            {
                return;
            }
        }
        else
        {
            sol.Initialized(ParDep);
        }

        for (int i = 1; i <= Nbvar; i++)
        {
            firstsol.ChangeValue(i) = sol[i];
        }
        State = TestArret(Func, sol, false, Blend_OK);
        if (State != Blend_OK)
        {
            return;
        }
        // OCCT_DEBUG Drawsect left out.
        // Update the line.
        line.Append(previousP);
        double U, V, W;
        previousP.ParametersOnS(out U, out V);
        W = previousP.ParameterOnC();
        BRepBlend_Extremity P1 = new(previousP.PointOnS(), U, V, previousP.Parameter(), tolpoint3d);
        BRepBlend_Extremity P2 = new(previousP.PointOnC(), W, previousP.Parameter(), tolpoint3d);
        if (sens > 0.0)
        {
            line.SetStartPoints(P1, P2);
        }
        else
        {
            line.SetEndPoints(P1, P2);
        }

        InternalPerform(Func, sol, Pmax);

        done = true;
    }

    /// <summary>Walks the other way from the first section, down to <paramref name="Pmin"/>.</summary>
    public bool Complete(Blend_CSFunction Func, double Pmin)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        if (iscomplete)
        {
            return true;
        }

        sens = -sens;
        int Nbvar = Func.NbVariables();
        math_Vector sol = new(1, Nbvar);
        for (int i = 1; i <= Nbvar; i++)
        {
            sol[i] = firstsol!.Value(i);
        }

        param = firstparam;

        InternalPerform(Func, sol, Pmin);

        sens = -sens;
        iscomplete = true;
        return true;
    }

    public bool IsDone() => done;

    public BRepBlend_Line Line()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return line!;
    }

    private Blend_Status TestArret(Blend_CSFunction Function, math_Vector Sol, bool TestDefl, Blend_Status State)
    {
        gp_Pnt pt1;
        gp_Vec V1, V2;
        gp_Vec Tgp1, Nor1;
        gp_Vec2d V12d;
        gp_Pnt2d pt2d;
        double pOnC;
        Blend_Status State1, State2;
        IntSurf_TypeTrans tras = IntSurf_Undecided;

        if (Function.IsSolution(Sol, tolpoint3d))
        {
            pt1 = Function.PointOnS();
            _ = Function.PointOnC();
            pt2d = Function.Pnt2d();
            pOnC = Function.ParameterOnC();
            V1 = Function.TangentOnS();
            V2 = Function.TangentOnC();
            V12d = Function.Tangent2d();

            if (TestDefl)
            {
                // Check the deflection on each surface and on the guide.
                State1 = CheckDeflectionOnSurf(pt1, pt2d, V1, V12d);
                // For problems in point/face walkings, the test on the curve is temporarily shelved.
                State2 = Blend_StepTooSmall;
            }
            else
            {
                State1 = Blend_OK;
                State2 = Blend_OK;
            }

            if (State1 == Blend_Backward)
            {
                State1 = Blend_StepTooLarge;
                rebrou = true;
            }

            if (State2 == Blend_Backward)
            {
                State2 = Blend_StepTooLarge;
                rebrou = true;
            }

            if (State1 == Blend_StepTooLarge || State2 == Blend_StepTooLarge)
            {
                return Blend_StepTooLarge;
            }

            if (!comptra)
            {
                Function.Tangent(pt2d.X(), pt2d.Y(), out Tgp1, out Nor1);
                double testra = Tgp1.Dot(Nor1.Crossed(V1));
                if (Math.Abs(testra) > Precision.Confusion())
                {
                    if (testra < 0.0)
                    {
                        tras = IntSurf_In;
                    }
                    else if (testra > 0.0)
                    {
                        tras = IntSurf_Out;
                    }
                    comptra = true;
                    line!.Set(tras);
                }
            }

            if (State1 == Blend_OK || State2 == Blend_OK)
            {
                previousP.SetValue(Function.PointOnS(), Function.PointOnC(), param, pt2d.X(), pt2d.Y(), pOnC, V1, V2,
                                   V12d);

                return State;
            }
            if (State1 == Blend_StepTooSmall && State2 == Blend_StepTooSmall)
            {
                previousP.SetValue(Function.PointOnS(), Function.PointOnC(), param, pt2d.X(), pt2d.Y(), pOnC, V1, V2,
                                   V12d);
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
        else
        {
            return Blend_StepTooLarge;
        }
    }

    private Blend_Status CheckDeflectionOnSurf(in gp_Pnt Psurf, in gp_Pnt2d Ponsurf, in gp_Vec Tgsurf,
                                               in gp_Vec2d Tgonsurf)
    {
        // set by tests in U4, corresponds to 11.478 degrees
        const double CosRef3D = 0.98;

        const double CosRef2D = 0.88; // corresponds to 25 degrees

        double Norme, prevNorme, Cosi, Cosi2;
        double FlecheCourante;
        double Du, Dv, Duv;
        double paramu, paramv, tolu, tolv;
        gp_Pnt prevP;
        gp_Vec prevTg;
        gp_Vec2d previousd2d;

        prevP = previousP.PointOnS();
        prevTg = previousP.TangentOnS();
        tolu = Adaptor3d_HSurfaceTool.UResolution(surf, tolpoint3d);
        tolv = Adaptor3d_HSurfaceTool.VResolution(surf, tolpoint3d);

        gp_Vec Corde = new(prevP, Psurf);
        Norme = Corde.SquareMagnitude();
        prevNorme = prevTg.SquareMagnitude();

        double toler3d = tolpoint3d;
        if (Norme <= toler3d * toler3d || prevNorme <= toler3d * toler3d)
        {
            // the same point may have to be forced
            return Blend_SamePoints;
        }
        Cosi = sens * Corde * prevTg;
        if (Cosi < 0.0)
        { // 3d angle > pi/2: going back
            return Blend_Backward;
        }

        Cosi2 = Cosi * Cosi / prevNorme / Norme;
        if (Cosi2 < CosRef3D)
        {
            return Blend_StepTooLarge;
        }

        previousP.ParametersOnS(out paramu, out paramv);
        previousd2d = previousP.Tangent2d();

        Du = Ponsurf.X() - paramu;
        Dv = Ponsurf.Y() - paramv;
        Duv = Du * Du + Dv * Dv;
        if ((Math.Abs(Du) < tolu && Math.Abs(Dv) < tolv)
            || (Math.Abs(previousd2d.X()) < tolu && Math.Abs(previousd2d.Y()) < tolv))
        {
            // the same point may have to be forced
            return Blend_SamePoints; // same 2d point
        }
        Cosi = sens * (Du * previousd2d.X() + Dv * previousd2d.Y());
        if (Cosi < 0)
        {
            return Blend_Backward;
        }

        Cosi = sens * Corde * Tgsurf;
        Cosi2 = Cosi * Cosi / Tgsurf.SquareMagnitude() / Norme;
        if (Cosi2 < CosRef3D || Cosi < 0.0)
        {
            return Blend_StepTooLarge;
        }

        Cosi = sens * (Du * Tgonsurf.X() + Dv * Tgonsurf.Y()) / Tgonsurf.Magnitude();
        Cosi2 = Cosi * Cosi / Duv;
        if (Cosi2 < CosRef2D || Cosi < 0.0)
        {
            return Blend_StepTooLarge;
        }

        FlecheCourante = (prevTg.Normalized().XYZ() - Tgsurf.Normalized().XYZ()).SquareModulus() * Norme / 64.0;

        if (FlecheCourante <= 0.25 * fleche * fleche)
        {
            return Blend_StepTooSmall;
        }
        if (FlecheCourante > fleche * fleche)
        {
            // step too large
            return Blend_StepTooLarge;
        }

        return Blend_OK;
    }

    // Not called in OCCT (its call is commented out in TestArret), kept with the class.
    private Blend_Status CheckDeflectionOnCurv(in gp_Pnt Pcurv, double Param, in gp_Vec Tgcurv)
    {
        // set by tests in U4, corresponds to 11.478 degrees
        const double CosRef3D = 0.98;

        double Norme, prevNorme, Cosi, Cosi2;
        double FlecheCourante;
        double Du, paramu, tolu;
        gp_Pnt prevP;
        gp_Vec prevTg;

        prevP = previousP.PointOnC();
        prevTg = previousP.TangentOnC();
        tolu = BRepBlend_HCurveTool.Resolution(curv, tolpoint3d);

        gp_Vec Corde = new(prevP, Pcurv);
        Norme = Corde.SquareMagnitude();
        prevNorme = prevTg.SquareMagnitude();

        double toler3d = tolpoint3d;
        if (Norme <= toler3d * toler3d)
        {
            // the same point may have to be forced
            return Blend_SamePoints;
        }
        else if (prevNorme > toler3d * toler3d)
        {
            Cosi = sens * Corde * prevTg;
            if (Cosi < 0.0)
            { // 3d angle > pi/2: going back
                return Blend_Backward;
            }

            Cosi2 = Cosi * Cosi / prevNorme / Norme;
            if (Cosi2 < CosRef3D)
            {
                return Blend_StepTooLarge;
            }
        }

        paramu = previousP.ParameterOnC();
        Du = Param - paramu;
        if (Math.Abs(Du) < tolu)
        {
            return Blend_SamePoints; // same 2d point
        }

        if (Tgcurv.Magnitude() <= tolpoint3d)
        {
            return Blend_SamePoints; // GROS BOBARD EN ATTENDANT
        }

        Cosi = sens * Corde * Tgcurv;
        Cosi2 = Cosi * Cosi / Tgcurv.SquareMagnitude() / Norme;
        if (Cosi2 < CosRef3D || Cosi < 0.0)
        {
            return Blend_StepTooLarge;
        }

        if (prevNorme > toler3d * toler3d)
        {
            FlecheCourante = (prevTg.Normalized().XYZ() - Tgcurv.Normalized().XYZ()).SquareModulus() * Norme / 64.0;

            if (FlecheCourante <= 0.25 * fleche * fleche)
            {
                return Blend_StepTooSmall;
            }
            if (FlecheCourante > fleche * fleche)
            {
                // step too large
                return Blend_StepTooLarge;
            }
        }
        return Blend_OK;
    }

    // OCCT's Recadre is commented out in its source; not ported.

    private void Transition(Adaptor2d_Curve2d A, double Param, ref IntSurf_Transition TLine,
                            ref IntSurf_Transition TArc)
    {
        gp_Pnt2d p2d;
        gp_Vec2d dp2d;

        gp_Pnt pbid;
        gp_Vec d1u, d1v, normale, tgrst = new();

        BRepBlend_HCurve2dTool.D1(A, Param, out p2d, out dp2d);
        Adaptor3d_HSurfaceTool.D1(surf, p2d.X(), p2d.Y(), out pbid, out d1u, out d1v);

        tgrst.SetLinearForm(dp2d.X(), d1u, dp2d.Y(), d1v);
        normale = d1u.Crossed(d1v);

        IntSurf.MakeTransition(previousP.TangentOnS(), tgrst, new gp_Dir(normale), ref TLine, ref TArc);
    }

    private void MakeExtremity(ref BRepBlend_Extremity Extrem, int Index, double Param, bool IsVtx,
                               Adaptor3d_HVertex? Vtx)
    {
        IntSurf_Transition Tline = new(), Tarc = new();
        double prm, U, V;
        int nbarc;
        Adaptor3d_TopolTool Iter;

        previousP.ParametersOnS(out U, out V);
        Extrem.SetValue(previousP.PointOnS(), U, V, previousP.Parameter(), tolpoint3d);
        Iter = domain;

        Iter.Init();
        nbarc = 1;

        if (!IsVtx)
        {
            while (nbarc < Index)
            {
                nbarc++;
                Iter.Next();
            }
            Transition(Iter.Value(), Param, ref Tline, ref Tarc);
            Extrem.AddArc(Iter.Value(), Param, Tline, Tarc);
        }

        else
        {
            Extrem.SetVertex(Vtx!);
            while (Iter.More())
            {
                Adaptor2d_Curve2d arc = Iter.Value();
                if (nbarc != Index)
                {
                    Iter.Initialize(arc);
                    Iter.InitVertexIterator();
                    while (Iter.MoreVertex())
                    {
                        if (Iter.Identical(Vtx!, Iter.Vertex()))
                        {
                            prm = BRepBlend_BlendTool.Parameter(Vtx!, arc);
                            Transition(arc, prm, ref Tline, ref Tarc);
                            Extrem.AddArc(arc, prm, Tline, Tarc);
                        }
                        Iter.NextVertex();
                    }
                }
                else
                {
                    Transition(arc, Param, ref Tline, ref Tarc);
                    Extrem.AddArc(arc, Param, Tline, Tarc);
                }
                nbarc++;
                Iter.Next();
            }
        }
    }

    private void InternalPerform(Blend_CSFunction Func, math_Vector sol, double Bound)
    {
        double stepw = pasmax;
        double parprec = param;

        Blend_Status State;
        TopAbs_State situ;
        double w, U, V;
        int nbarc;
        int Index = 0;
        bool Isvtx = false;
        int Nbvar = Func.NbVariables();
        bool Arrive, recad, echecrecad;
        gp_Pnt2d p2d;
        math_Vector tolerance = new(1, Nbvar), infbound = new(1, Nbvar), supbound = new(1, Nbvar),
                    parinit = new(1, Nbvar);
        math_Vector solrst = new(1, Nbvar);
        Adaptor3d_HVertex? Vtx = null;
        BRepBlend_Extremity Exts = new(), Extc = new();

        Func.GetTolerance(tolerance, tolpoint3d);
        Func.GetBounds(infbound, supbound);

        math_FunctionSetRoot rsnld = new(Func, tolerance, 30);
        parinit.Initialized(sol);

        param = parprec + sens * stepw;
        Arrive = (sens * (param - Bound) > 0.0);

        while (!Arrive)
        {
            Func.Set(param);
            rsnld.Perform(Func, parinit, infbound, supbound);

            if (!rsnld.IsDone())
            {
                State = Blend_StepTooLarge;
            }
            else
            {
                rsnld.Root(sol);

                situ = domain.Classify(Func.Pnt2d(), Min(tolerance[1], tolerance[2]));

                w = Bound;
                recad = false;
                echecrecad = false;

                if (situ == TopAbs_OUT || situ == TopAbs_ON)
                {
                    // inverse problem on surf
                    //	recad = Recadre(FuncInv,sol,solrst,Index,Isvtx,Vtx);
                    Isvtx = false; // waiting for Recadre
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
                        State = Blend_OnRst1;
                        param = w;
                        domain.Init();
                        nbarc = 1;
                        while (nbarc < Index)
                        {
                            nbarc++;
                            domain.Next();
                        }
                        p2d = BRepBlend_HCurve2dTool.Value(domain.Value(), solrst[1]);
                        sol[1] = p2d.X();
                        sol[2] = p2d.Y();
                        sol[3] = solrst[3];
                        Func.Set(param);
                    }
                    else
                    {
                        State = Blend_OK;
                    }
                    State = TestArret(Func, sol, true, State);
                }
                else
                {
                    // Recadrage failed. Leave with SamePoints
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
                        line!.Append(previousP);
                    }
                    else
                    {
                        line!.Prepend(previousP);
                    }
                    parinit.Initialized(sol);
                    parprec = param;

                    if (param == Bound)
                    {
                        Arrive = true;
                        previousP.ParametersOnS(out U, out V);
                        Exts.SetValue(previousP.PointOnS(), U, V, previousP.Parameter(), tolpoint3d);
                        Extc.SetValue(previousP.PointOnC(), previousP.ParameterOnC(), previousP.Parameter(),
                                      tolpoint3d);
                        // Indicate the end on Bound.
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
                        Extc.SetValue(previousP.PointOnC(), previousP.ParameterOnC(), previousP.Parameter(),
                                      tolpoint3d);
                        Arrive = true;
                    }
                    else
                    {
                        param = parprec + sens * stepw; // no risk of going past Bound.
                    }
                }
                break;

                case Blend_StepTooSmall:
                {
                    // OCCT_DEBUG Drawsect left out.
                    // Update the line.
                    if (sens > 0.0)
                    {
                        line!.Append(previousP);
                    }
                    else
                    {
                        line!.Prepend(previousP);
                    }
                    parinit.Initialized(sol);
                    parprec = param;

                    stepw = Min(1.5 * stepw, pasmax);
                    if (param == Bound)
                    {
                        Arrive = true;
                        previousP.ParametersOnS(out U, out V);
                        Exts.SetValue(previousP.PointOnS(), U, V, previousP.Parameter(), tolpoint3d);
                        Extc.SetValue(previousP.PointOnC(), previousP.ParameterOnC(), previousP.Parameter(),
                                      tolpoint3d);
                        // Indicate the end on Bound.
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
                        line!.Append(previousP);
                    }
                    else
                    {
                        line!.Prepend(previousP);
                    }
                    MakeExtremity(ref Exts, Index, solrst[1], Isvtx, Vtx);
                    Extc.SetValue(previousP.PointOnC(), previousP.ParameterOnC(), previousP.Parameter(), tolpoint3d);
                    Arrive = true;
                }
                break;

                case Blend_SamePoints:
                {
                    // Stop. (OCCT prints " Points confondus dans le cheminement" to the standard output: left out.)
                    previousP.ParametersOnS(out U, out V);
                    Exts.SetValue(previousP.PointOnS(), U, V, previousP.Parameter(), tolpoint3d);
                    Extc.SetValue(previousP.PointOnC(), previousP.ParameterOnC(), previousP.Parameter(), tolpoint3d);
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
                    line!.SetEndPoints(Exts, Extc);
                }
                else
                {
                    line!.SetStartPoints(Exts, Extc);
                }
            }
        }
    }
}
