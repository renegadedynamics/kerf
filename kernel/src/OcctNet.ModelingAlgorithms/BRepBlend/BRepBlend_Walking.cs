// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Walking.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BRepBlend/BRepBlend_Walking.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.Blend_Status;
using static Kerf.Engine.Geometry.NormalStatus;
using static Occt.Extrema_ExtFlag;
using static Occt.IntSurf_TypeTrans;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Walks a blend between two surfaces along a guide: from a first section, solves the blend function at steps of
/// the guide's parameter, checks the deflection, and stops on the surfaces' restrictions (the "recadrage", solving
/// the inverse function on the arc), building a <see cref="BRepBlend_Line"/>.
/// </summary>
/// <remarks>
/// A working object (decision 21), the walker of ChFi3d_Builder's <c>ComputeData</c>. The order of its calls to the
/// function (<c>Set</c>, the root finder, <c>IsSolution</c>, the tangents) is OCCT's line for line, which the
/// functions' caches depend on (decisions 1037 and 1043). OCCT's <c>OCCT_DEBUG</c> drawing of the sections
/// (<c>Drawsect</c>) and its traces are left out.
/// </remarks>
public class BRepBlend_Walking
{
    private const double CosRef3D_File = 0.88; // OCCT's file-static CosRef3D (InternalPerform)

    private Blend_Point previousP = new();
    private BRepBlend_Line? line;
    private readonly math_Vector sol;
    private readonly NCollection_Sequence<Blend_Point> jalons = new();
    private readonly Adaptor3d_Surface surf1;
    private readonly Adaptor3d_Surface surf2;
    private Adaptor3d_TopolTool domain1;
    private Adaptor3d_TopolTool domain2;
    private Adaptor3d_TopolTool recdomain1;
    private Adaptor3d_TopolTool recdomain2;
    private ChFiDS_ElSpine hguide;
    private bool ToCorrectOnRst1;
    private bool ToCorrectOnRst2;
    private double CorrectedParam;
    private double tolpoint3d;
    private double tolgui;
    private double pasmax;
    private double fleche;
    private double param;
    private double sens;
    private bool done;
#pragma warning disable CS0414 // set and never read, as in OCCT
    private bool rebrou;
#pragma warning restore CS0414
    private bool iscomplete;
    private bool comptra;
    private bool clasonS1;
    private bool clasonS2;
    private bool check2d;
    private bool check;
    private bool twistflag1;
    private bool twistflag2;

    public BRepBlend_Walking(Adaptor3d_Surface Surf1, Adaptor3d_Surface Surf2, Adaptor3d_TopolTool Domain1,
                             Adaptor3d_TopolTool Domain2, ChFiDS_ElSpine HGuide)
    {
        sol = new math_Vector(1, 4);
        surf1 = Surf1;
        surf2 = Surf2;
        ToCorrectOnRst1 = false;
        ToCorrectOnRst2 = false;
        done = false;
        clasonS1 = true;
        clasonS2 = true;
        check2d = true;
        check = true;
        twistflag1 = false;
        twistflag2 = false;

        domain1 = Domain1;
        domain2 = Domain2;
        recdomain1 = Domain1;
        recdomain2 = Domain2;
        hguide = HGuide;
    }

    /// <summary>The domains on which the walking stops (the restrictions), when they differ from the classification's.</summary>
    public void SetDomainsToRecadre(Adaptor3d_TopolTool Domain1, Adaptor3d_TopolTool Domain2)
    {
        recdomain1 = Domain1;
        recdomain2 = Domain2;
    }

    /// <summary>Adds a singular point (a "jalon") the walking must pass, in the order of the guide's parameter.</summary>
    public void AddSingularPoint(in Blend_Point P)
    {
        if (jalons.Length() == 0)
        {
            jalons.Append(P);
        }
        else
        {
            int ii, jj;
            double tp = P.Parameter(), ti = jalons.First().Parameter();
            for (jj = 1, ii = 1; ii <= jalons.Length() && tp > ti; ii++)
            {
                jj = ii;
                ti = jalons.Value(jj).Parameter();
            }
            if (tp > ti)
            {
                jalons.InsertAfter(jj, P);
            }
            else
            {
                jalons.InsertBefore(jj, P);
            }
        }
    }

    /// <summary>
    /// Walks from the guide's parameter <paramref name="Pdep"/> towards <paramref name="Pmax"/>, starting from the
    /// solution <paramref name="ParDep"/> (solved again first when <paramref name="Appro"/>).
    /// </summary>
    public void Perform(Blend_Function Func, Blend_FuncInv FuncInv, double Pdep, double Pmax, double MaxStep,
                        double Tol3d, double TolGuide, math_Vector ParDep, double Fleche, bool Appro = false)
    {
        done = false;
        iscomplete = false;
        comptra = false;
        bool doextremities = true;
        if (line is null)
        {
            line = new BRepBlend_Line();
        }
        else
        {
            line.Clear();
            doextremities = false;
        }
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
            TopAbs_State situ1, situ2;
            math_Vector tolerance = new(1, 4), infbound = new(1, 4), supbound = new(1, 4);
            Func.GetTolerance(tolerance, tolpoint3d);
            Func.GetBounds(infbound, supbound);
            FunctionSetRoot rsnld = new(Func, tolerance, 30);

            rsnld.Perform(Func, ParDep, infbound, supbound);

            if (!rsnld.IsDone)
            {
                return;
            }
            rsnld.Root(sol);

            if (clasonS1)
            {
                situ1 = domain1.Classify(new Point2D(sol[1], sol[2]), Min(tolerance[1], tolerance[2]), false);
            }
            else
            {
                situ1 = TopAbs_IN;
            }
            if (clasonS2)
            {
                situ2 = domain2.Classify(new Point2D(sol[3], sol[4]), Min(tolerance[3], tolerance[4]), false);
            }
            else
            {
                situ2 = TopAbs_IN;
            }

            if (situ1 != TopAbs_IN || situ2 != TopAbs_IN)
            {
                return;
            }
        }
        else
        {
            sol.Initialized(ParDep);
        }

        State = TestArret(Func, Blend_OK, false);
        if (State != Blend_OK)
        {
            return;
        }
        // OCCT_DEBUG Drawsect left out.
        // Update the line.
        // Correct first parameter if needed
        if (ToCorrectOnRst1 || ToCorrectOnRst2)
        {
            previousP.SetParameter(CorrectedParam);
        }
        line.Append(previousP);

        if (doextremities)
        {
            // OCCT: four arguments make the curve overload (P, W, Param, Tol): U is u, the guide's parameter v
            BRepBlend_Extremity ptf1 = new(previousP.PointOnS1(), sol[1], sol[2], tolpoint3d);
            BRepBlend_Extremity ptf2 = new(previousP.PointOnS2(), sol[3], sol[4], tolpoint3d);
            if (!previousP.IsTangencyPoint())
            {
                ptf1.SetTangent(previousP.TangentOnS1());
                ptf2.SetTangent(previousP.TangentOnS2());
            }

            if (sens > 0.0)
            {
                line.SetStartPoints(ptf1, ptf2);
            }
            else
            {
                line.SetEndPoints(ptf1, ptf2);
            }
        }

        InternalPerform(Func, FuncInv, Pmax);

        done = true;
    }

    /// <summary>
    /// Solves the first section at <paramref name="Pdep"/> from <paramref name="ParDep"/> (which receives the
    /// solution) and classifies its points: true if both are inside their faces.
    /// </summary>
    public bool PerformFirstSection(Blend_Function Func, double Pdep, math_Vector ParDep, double Tol3d,
                                    double TolGuide, out TopAbs_State Pos1, out TopAbs_State Pos2)
    {
        iscomplete = false;
        comptra = false;
        line = new BRepBlend_Line();
        tolpoint3d = Tol3d;
        tolgui = Math.Abs(TolGuide);

        Pos1 = Pos2 = TopAbs_UNKNOWN;

        param = Pdep;
        Func.Set(param);

        math_Vector tolerance = new(1, 4), infbound = new(1, 4), supbound = new(1, 4);
        Func.GetTolerance(tolerance, tolpoint3d);
        Func.GetBounds(infbound, supbound);
        FunctionSetRoot rsnld = new(Func, tolerance, 30);

        rsnld.Perform(Func, ParDep, infbound, supbound);

        if (!rsnld.IsDone)
        {
            return false;
        }
        rsnld.Root(sol);
        ParDep.Initialized(sol);
        Pos1 = domain1.Classify(new Point2D(sol[1], sol[2]), Min(tolerance[1], tolerance[2]), false);
        Pos2 = domain2.Classify(new Point2D(sol[3], sol[4]), Min(tolerance[3], tolerance[4]), false);
        if (Pos1 != TopAbs_IN || Pos2 != TopAbs_IN)
        {
            return false;
        }

        TestArret(Func, Blend_OK, false);
        // OCCT_DEBUG Drawsect left out.
        return true;
    }

    /// <summary>
    /// Solves a first section at <paramref name="Pdep"/> and moves it onto the restrictions (of the first surface if
    /// <paramref name="RecOnS1"/>, of the second if <paramref name="RecOnS2"/>): <paramref name="Psol"/> and
    /// <paramref name="ParSol"/> receive the section on the restriction.
    /// </summary>
    public bool PerformFirstSection(Blend_Function Func, Blend_FuncInv FuncInv, double Pdep, double Pmax,
                                    math_Vector ParDep, double Tol3d, double TolGuide, bool RecOnS1, bool RecOnS2,
                                    ref double Psol, math_Vector ParSol)
    {
        iscomplete = false;
        comptra = false;
        line = new BRepBlend_Line();

        double w1, w2, extrapol;
        bool recad1, recad2;

        tolpoint3d = Tol3d;
        tolgui = Math.Abs(TolGuide);
        if (Pmax - Pdep >= 0.0)
        {
            sens = 1.0;
        }
        else
        {
            sens = -1.0;
        }
        extrapol = Math.Abs(Pmax - Pdep) / 50.0; // 2%

        Blend_Status State;

        param = Pdep;
        Func.Set(param);

        math_Vector tolerance = new(1, 4), infbound = new(1, 4), supbound = new(1, 4);
        math_Vector solrst1 = new(1, 4), solrst2 = new(1, 4);
        BRepBlend_Extremity Ext1 = new(), Ext2 = new();
        int Index1 = 0, Index2 = 0, nbarc;
        bool Isvtx1 = false, Isvtx2 = false;
        Adaptor3d_HVertex? Vtx1 = null, Vtx2 = null;
        Point2D p2d;
        double CorrectedU = 0.0, CorrectedV = 0.0;
        Point3D CorrectedPnt = new();

        Func.GetTolerance(tolerance, tolpoint3d);
        Func.GetBounds(infbound, supbound);
        FunctionSetRoot rsnld = new(Func, tolerance, 30);

        rsnld.Perform(Func, ParDep, infbound, supbound);

        if (!rsnld.IsDone)
        {
            return false;
        }
        rsnld.Root(sol);

        w1 = w2 = Pmax;

        recad1 = RecOnS1 && Recadre(FuncInv, true, sol, solrst1, out Index1, out Isvtx1, ref Vtx1, extrapol);
        if (recad1)
        {
            w1 = solrst1[2];
        }

        recad2 = RecOnS2 && Recadre(FuncInv, false, sol, solrst2, out Index2, out Isvtx2, ref Vtx2, extrapol);
        if (recad2)
        {
            w2 = solrst2[2];
        }

        if (!recad1 && !recad2)
        {
            return false;
        }

        if (recad1 && recad2)
        {
            if (Math.Abs(w1 - w2) <= tolgui)
            {
                // the solution is on 1 and 2 at once
                State = Blend_OnRst12;
                param = w1;
                ParSol[1] = solrst2[3];
                ParSol[2] = solrst2[4];
                ParSol[3] = solrst1[3];
                ParSol[4] = solrst1[4];
            }
            else if (sens * (w2 - w1) < 0.0)
            { // keep the greater
                // solution on 1
                State = Blend_OnRst1;
                param = w1;

                recdomain1.Init();
                nbarc = 1;
                while (nbarc < Index1)
                {
                    nbarc++;
                    recdomain1.Next();
                }
                p2d = BRepBlend_HCurve2dTool.Value(recdomain1.Value(), solrst1[1]);
                ParSol[1] = p2d.X();
                ParSol[2] = p2d.Y();
                ParSol[3] = solrst1[3];
                ParSol[4] = solrst1[4];
            }
            else
            {
                // solution on 2
                State = Blend_OnRst2;
                param = w2;

                recdomain2.Init();
                nbarc = 1;
                while (nbarc < Index2)
                {
                    nbarc++;
                    recdomain2.Next();
                }
                p2d = BRepBlend_HCurve2dTool.Value(recdomain2.Value(), solrst2[1]);
                ParSol[1] = solrst2[3];
                ParSol[2] = solrst2[4];
                ParSol[3] = p2d.X();
                ParSol[4] = p2d.Y();
            }
        }
        else if (recad1)
        {
            // solution on 1
            State = Blend_OnRst1;
            param = w1;
            recdomain1.Init();
            nbarc = 1;
            while (nbarc < Index1)
            {
                nbarc++;
                recdomain1.Next();
            }
            p2d = BRepBlend_HCurve2dTool.Value(recdomain1.Value(), solrst1[1]);
            ParSol[1] = p2d.X();
            ParSol[2] = p2d.Y();
            ParSol[3] = solrst1[3];
            ParSol[4] = solrst1[4];
            Point3D thePntOnRst = Adaptor3d_HSurfaceTool.Value(surf1, ParSol[1], ParSol[2]);
            if (CorrectExtremityOnOneRst(1, ParSol[3], ParSol[4], param, thePntOnRst, ref CorrectedU, ref CorrectedV,
                                         ref CorrectedPnt, ref CorrectedParam))
            {
                ToCorrectOnRst1 = true;
            }
        }
        else
        { // if (recad2) {
            // solution on 2
            State = Blend_OnRst2;
            param = w2;

            recdomain2.Init();
            nbarc = 1;
            while (nbarc < Index2)
            {
                nbarc++;
                recdomain2.Next();
            }
            p2d = BRepBlend_HCurve2dTool.Value(recdomain2.Value(), solrst2[1]);
            ParSol[1] = solrst2[3];
            ParSol[2] = solrst2[4];
            ParSol[3] = p2d.X();
            ParSol[4] = p2d.Y();
            Point3D thePntOnRst = Adaptor3d_HSurfaceTool.Value(surf2, ParSol[3], ParSol[4]);
            if (CorrectExtremityOnOneRst(2, ParSol[1], ParSol[2], param, thePntOnRst, ref CorrectedU, ref CorrectedV,
                                         ref CorrectedPnt, ref CorrectedParam))
            {
                ToCorrectOnRst2 = true;
            }
        }

        Psol = param;
        sol.Initialized(ParSol);
        Func.Set(param);
        State = TestArret(Func, State, false);
        switch (State)
        {
            case Blend_OnRst1:
            {
                // OCCT_DEBUG Drawsect left out.
                MakeExtremity(ref Ext1, true, Index1, solrst1[1], Isvtx1, Vtx1);
                if (ToCorrectOnRst1)
                {
                    Ext2.SetValue(CorrectedPnt, CorrectedU, CorrectedV, tolpoint3d); // OCCT: the curve overload
                }
                else
                {
                    Ext2.SetValue(previousP.PointOnS2(), sol[3], sol[4], tolpoint3d);
                }
            }
            break;

            case Blend_OnRst2:
            {
                // OCCT_DEBUG Drawsect left out.
                if (ToCorrectOnRst2)
                {
                    Ext1.SetValue(CorrectedPnt, CorrectedU, CorrectedV, tolpoint3d); // OCCT: the curve overload
                }
                else
                {
                    Ext1.SetValue(previousP.PointOnS1(), sol[1], sol[2], tolpoint3d);
                }
                MakeExtremity(ref Ext2, false, Index2, solrst2[1], Isvtx2, Vtx2);
            }
            break;

            case Blend_OnRst12:
            {
                // OCCT_DEBUG Drawsect left out.
                MakeExtremity(ref Ext1, true, Index1, solrst1[1], Isvtx1, Vtx1);
                MakeExtremity(ref Ext2, false, Index2, solrst2[1], Isvtx2, Vtx2);
            }
            break;
            default:
            {
                throw new FailureException("BRepBlend_Walking::PerformFirstSection : echec");
            }
        }
        if (sens < 0.0)
        {
            line.SetEndPoints(Ext1, Ext2);
        }
        else
        {
            line.SetStartPoints(Ext1, Ext2);
        }
        return true;
    }

    /// <summary>Continues the walking up to the guide's parameter <paramref name="P"/>, beyond the line's ends.</summary>
    public bool Continu(Blend_Function Func, Blend_FuncInv FuncInv, double P)
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        Blend_Point firstBP = line!.Point(1);
        Blend_Point lastBP = line.Point(line.NbPoints());

        if (P < firstBP.Parameter())
        {
            sens = -1.0;
            previousP = firstBP;
        }
        else if (P > lastBP.Parameter())
        {
            sens = 1.0;
            previousP = lastBP;
        }

        param = previousP.Parameter();
        previousP.ParametersOnS1(out sol[1], out sol[2]);
        previousP.ParametersOnS2(out sol[3], out sol[4]);

        InternalPerform(Func, FuncInv, P);
        return true;
    }

    /// <summary>
    /// Continues the walking up to <paramref name="P"/> past the restriction of one surface (the first if
    /// <paramref name="OnS1"/>), without classifying on it; false, and the line restored, if it does not reach the
    /// other surface's restriction.
    /// </summary>
    public bool Continu(Blend_Function Func, Blend_FuncInv FuncInv, double P, bool OnS1)
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        BRepBlend_Extremity Ext1, Ext2;
        if (sens < 0.0)
        {
            Ext1 = line!.StartPointOnFirst();
            Ext2 = line.StartPointOnSecond();
            if ((OnS1 && Ext1.NbPointOnRst() == 0) || (!OnS1 && Ext2.NbPointOnRst() == 0))
            {
                return false;
            }
            previousP = line.Point(1);
        }
        else
        {
            Ext1 = line!.EndPointOnFirst();
            Ext2 = line.EndPointOnSecond();
            if ((OnS1 && Ext1.NbPointOnRst() == 0) || (!OnS1 && Ext2.NbPointOnRst() == 0))
            {
                return false;
            }
            previousP = line.Point(line.NbPoints());
        }

        int length = line.NbPoints();
        param = previousP.Parameter();
        previousP.ParametersOnS1(out sol[1], out sol[2]);
        previousP.ParametersOnS2(out sol[3], out sol[4]);

        if (OnS1)
        {
            clasonS1 = false;
        }
        else
        {
            clasonS2 = false;
        }

        InternalPerform(Func, FuncInv, P);

        clasonS1 = true;
        clasonS2 = true;

        int newlength = line.NbPoints();
        if (sens < 0.0)
        {
            if ((OnS1 && line.StartPointOnSecond().NbPointOnRst() == 0)
                || (!OnS1 && line.StartPointOnFirst().NbPointOnRst() == 0))
            {
                line.Remove(1, newlength - length);
                line.SetStartPoints(Ext1, Ext2);
                return false;
            }
        }
        else
        {
            if ((OnS1 && line.EndPointOnSecond().NbPointOnRst() == 0)
                || (!OnS1 && line.EndPointOnFirst().NbPointOnRst() == 0))
            {
                line.Remove(length, newlength);
                line.SetEndPoints(Ext1, Ext2);
                return false;
            }
        }
        return true;
    }

    /// <summary>Walks the other way from the line's start, down to <paramref name="Pmin"/>.</summary>
    public bool Complete(Blend_Function Func, Blend_FuncInv FuncInv, double Pmin)
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
        previousP.ParametersOnS1(out sol[1], out sol[2]);
        previousP.ParametersOnS2(out sol[3], out sol[4]);

        InternalPerform(Func, FuncInv, Pmin);

        iscomplete = true;
        return true;
    }

    public void ClassificationOnS1(bool C)
    {
        clasonS1 = C;
    }

    public void ClassificationOnS2(bool C)
    {
        clasonS2 = C;
    }

    public void Check2d(bool C)
    {
        check2d = C;
    }

    public void Check(bool C)
    {
        check = C;
    }

    public bool TwistOnS1() => twistflag1;

    public bool TwistOnS2() => twistflag2;

    public bool IsDone() => done;

    public BRepBlend_Line Line()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return line!;
    }

    // Is the given point a solution? If so, check the deflection on surf1 and surf2; if that is OK, classify the
    // points on surf1 and surf2: both inside, Blend_OK; one inside, solve the inverse problem on that restriction;
    // otherwise on the surface where the point is the furthest. If the deflection is not OK, or the point is not a
    // solution, Blend_StepTooLarge.
    private Blend_Status TestArret(Blend_Function Function, Blend_Status State, bool TestDefl, bool TestSolu = true,
                                   bool TestLengthStep = false)
    {
        Point3D pt1, pt2;
        Vector3D V1 = new(), V2 = new();
        Vector3D Tgp1, Tgp2, Nor1, Nor2;
        Vector2D V12d, V22d;
        Blend_Status State1, State2;
        IntSurf_TypeTrans tras1, tras2;
        Blend_Point curpoint = new();
        bool loctwist1 = false, loctwist2 = false;
        double tolsolu = tolpoint3d;

        if (!TestSolu)
        {
            tolsolu *= 1000; // it must always be good
        }
        if (Function.IsSolution(sol, tolsolu))
        {
            bool curpointistangent = Function.IsTangencyPoint();
            pt1 = Function.PointOnS1();
            pt2 = Function.PointOnS2();
            if (curpointistangent)
            {
                curpoint.SetValue(pt1, pt2, param, sol[1], sol[2], sol[3], sol[4]);
            }
            else
            {
                V1 = Function.TangentOnS1();
                V2 = Function.TangentOnS2();
                V12d = Function.Tangent2dOnS1();
                V22d = Function.Tangent2dOnS2();
                curpoint.SetValue(pt1, pt2, param, sol[1], sol[2], sol[3], sol[4], V1, V2, V12d, V22d);
                if (Function.TwistOnS1())
                {
                    loctwist1 = true;
                }
                if (Function.TwistOnS2())
                {
                    loctwist2 = true;
                }
            }

            if (TestDefl && check)
            {
                // Check the deflection on each surface and on the guide.
                State1 = CheckDeflection(true, curpoint);
                State2 = CheckDeflection(false, curpoint);
            }
            else
            {
                State1 = Blend_OK;
                State2 = Blend_OK;
                if (TestLengthStep)
                {
                    // Only check that the step is not too large (bad extensions).
                    double curparamu, curparamv, prevparamu, prevparamv;
                    math_Vector inf = new(1, 4), sup = new(1, 4);
                    Function.GetBounds(inf, sup);
                    sup -= inf;
                    sup *= 0.05; // max step: 5% of the domain

                    curpoint.ParametersOnS1(out curparamu, out curparamv);
                    previousP.ParametersOnS1(out prevparamu, out prevparamv);
                    if (Math.Abs(curparamu - prevparamu) > sup[1])
                    {
                        State1 = Blend_StepTooLarge;
                    }
                    if (Math.Abs(curparamv - prevparamv) > sup[2])
                    {
                        State1 = Blend_StepTooLarge;
                    }
                    curpoint.ParametersOnS2(out curparamu, out curparamv);
                    previousP.ParametersOnS2(out prevparamu, out prevparamv);
                    if (Math.Abs(curparamu - prevparamu) > sup[3])
                    {
                        State2 = Blend_StepTooLarge;
                    }
                    if (Math.Abs(curparamv - prevparamv) > sup[4])
                    {
                        State2 = Blend_StepTooLarge;
                    }
                }
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

            // Only here can the twist be decided, the rejections being done (BUC60322).
            if (loctwist1)
            {
                twistflag1 = true;
            }
            if (loctwist2)
            {
                twistflag2 = true;
            }

            if (!comptra && !curpointistangent)
            {
                Function.Tangent(sol[1], sol[2], sol[3], sol[4], out Tgp1, out Tgp2, out Nor1, out Nor2);
                Nor1.Normalize();
                Nor2.Normalize();
                double testra = Tgp1.Dot(Nor1.Crossed(V1));
                if (Math.Abs(testra) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    tras1 = IntSurf_In;
                    if ((testra > 0.0 && !loctwist1) || (testra < 0.0 && loctwist1))
                    {
                        tras1 = IntSurf_Out;
                    }

                    testra = Tgp2.Dot(Nor2.Crossed(V2));
                    if (Math.Abs(testra) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        tras2 = IntSurf_Out;
                        if ((testra > 0.0 && !loctwist2) || (testra < 0.0 && loctwist2))
                        {
                            tras2 = IntSurf_In;
                        }
                        comptra = true;
                        line!.Set(tras1, tras2);
                    }
                }
            }

            if (State1 == Blend_OK || State2 == Blend_OK)
            {
                previousP = curpoint;
                return State;
            }

            if (State1 == Blend_StepTooSmall && State2 == Blend_StepTooSmall)
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
        else
        {
            return Blend_StepTooLarge;
        }
    }

    private Blend_Status CheckDeflection(bool OnFirst, in Blend_Point CurPoint)
    {
        // set by tests in U4, corresponds to 11.478 degrees
        const double CosRef3D = 0.98;

        const double CosRef2D = 0.88; // corresponds to 25 degrees

        double Norme, Cosi, Cosi2;
        double prevNorme = 0.0;
        double FlecheCourante;
        double Du, Dv, Duv;
        double tolu, tolv;

        Point3D Psurf;
        Vector3D Tgsurf = new();
        Vector2D Tgonsurf = new();
        double curparamu, curparamv;
        bool curpointistangent = CurPoint.IsTangencyPoint();

        Point3D prevP;
        Vector3D prevTg = new();
        Vector2D previousd2d = new();
        double prevparamu, prevparamv;
        bool prevpointistangent = previousP.IsTangencyPoint();

        if (OnFirst)
        {
            Psurf = CurPoint.PointOnS1();
            if (!curpointistangent)
            {
                Tgsurf = CurPoint.TangentOnS1();
            }
            prevP = previousP.PointOnS1();
            if (!prevpointistangent)
            {
                prevTg = previousP.TangentOnS1();
            }
            tolu = Adaptor3d_HSurfaceTool.UResolution(surf1, tolpoint3d);
            tolv = Adaptor3d_HSurfaceTool.VResolution(surf1, tolpoint3d);
        }
        else
        {
            Psurf = CurPoint.PointOnS2();
            if (!curpointistangent)
            {
                Tgsurf = CurPoint.TangentOnS2();
            }
            prevP = previousP.PointOnS2();
            if (!prevpointistangent)
            {
                prevTg = previousP.TangentOnS2();
            }
            tolu = Adaptor3d_HSurfaceTool.UResolution(surf2, tolpoint3d);
            tolv = Adaptor3d_HSurfaceTool.VResolution(surf2, tolpoint3d);
        }

        Vector3D Corde = new(prevP, Psurf);
        Norme = Corde.SquareMagnitude();
        if (!prevpointistangent)
        {
            prevNorme = prevTg.SquareMagnitude();
        }

        double toler3d = 0.01 * tolpoint3d;
        if (Norme <= toler3d * toler3d)
        {
            // the same point may have to be forced
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
            { // 3d angle > pi/2: going back
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
            Cosi = sens * Corde * Tgsurf;
            Cosi2 = Cosi * Cosi / Tgsurf.SquareMagnitude() / Norme;
            if (Cosi2 < CosRef3D || Cosi < 0.0)
            {
                return Blend_StepTooLarge;
            }
        }

        if (check2d)
        {
            if (OnFirst)
            {
                CurPoint.ParametersOnS1(out curparamu, out curparamv);
                if (!curpointistangent)
                {
                    Tgonsurf = CurPoint.Tangent2dOnS1();
                }
                previousP.ParametersOnS1(out prevparamu, out prevparamv);
                if (!prevpointistangent)
                {
                    previousd2d = previousP.Tangent2dOnS1();
                }
            }
            else
            {
                CurPoint.ParametersOnS2(out curparamu, out curparamv);
                if (!curpointistangent)
                {
                    Tgonsurf = CurPoint.Tangent2dOnS2();
                }
                previousP.ParametersOnS2(out prevparamu, out prevparamv);
                if (!prevpointistangent)
                {
                    previousd2d = previousP.Tangent2dOnS2();
                }
            }

            Du = curparamu - prevparamu;
            Dv = curparamv - prevparamv;
            Duv = Du * Du + Dv * Dv;
            if (Math.Abs(Du) < tolu && Math.Abs(Dv) < tolv)
            {
                // the same point may have to be forced
                return Blend_SamePoints; // same 2d point
            }
            if (!prevpointistangent)
            {
                if (Math.Abs(previousd2d.X()) < tolu && Math.Abs(previousd2d.Y()) < tolv)
                {
                    return Blend_SamePoints; // same 2d point
                }
                Cosi = sens * (Du * previousd2d.X() + Dv * previousd2d.Y());
                if (Cosi < 0)
                {
                    return Blend_Backward;
                }
            }
            if (!curpointistangent)
            {
                Cosi = sens * (Du * Tgonsurf.X() + Dv * Tgonsurf.Y()) / Tgonsurf.Magnitude();
                Cosi2 = Cosi * Cosi / Duv;
                if (Cosi2 < CosRef2D || Cosi < 0.0)
                {
                    return Blend_StepTooLarge;
                }
            }
        }
        if (!curpointistangent && !prevpointistangent)
        {
            // Estimate of the current deflection
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
        }
        return Blend_OK;
    }

    private int ArcToRecadre(bool OnFirst, math_Vector theSol, int PrevIndex, out Point2D lastpt2d,
                             out Point2D pt2d, ref double ponarc)
    {
        int IndexSol = 0, nbarc = 0;
        bool ok = false;
        bool byinter = (line!.NbPoints() != 0), okinter = false;
        double distmin = RealLast();
        double uprev = 0.0, vprev = 0.0, prm = 0.0, dist = 0.0;
        Adaptor3d_TopolTool Iter;

        if (OnFirst)
        {
            if (byinter)
            {
                previousP.ParametersOnS1(out uprev, out vprev);
            }
            pt2d = new Point2D(theSol[1], theSol[2]);
            Iter = recdomain1;
        }
        else
        {
            if (byinter)
            {
                previousP.ParametersOnS2(out uprev, out vprev);
            }
            pt2d = new Point2D(theSol[3], theSol[4]);
            Iter = recdomain2;
        }
        lastpt2d = new Point2D(uprev, vprev);
        Iter.Init();
        while (Iter.More())
        {
            nbarc++;
            ok = false;
            if (OnFirst)
            {
                if (byinter)
                {
                    ok = okinter = BRepBlend_BlendTool.Inters(pt2d, lastpt2d, surf1, Iter.Value(), ref prm, ref dist);
                }
                if (!ok)
                {
                    ok = BRepBlend_BlendTool.Project(pt2d, surf1, Iter.Value(), out prm, out dist);
                }
            }
            else
            {
                if (byinter)
                {
                    ok = okinter = BRepBlend_BlendTool.Inters(pt2d, lastpt2d, surf2, Iter.Value(), ref prm, ref dist);
                }
                if (!ok)
                {
                    ok = BRepBlend_BlendTool.Project(pt2d, surf2, Iter.Value(), out prm, out dist);
                }
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
            Iter.Next();
        }
        return IndexSol;
    }

    private bool Recadre(Blend_FuncInv FuncInv, bool OnFirst, math_Vector theSol, math_Vector solrst,
                         out int Indexsol, out bool IsVtx, ref Adaptor3d_HVertex? Vtx, double Extrap = 0.0)
    {
        bool jalons_Trouve = false;
        bool recadre = true, ok;
        bool byinter = (line!.NbPoints() != 0);
        int LeJalon = 0;

        int nbarc;
        double dist, prm, pmin = 0.0, vtol;
        Point2D pt2d, lastpt2d;

        math_Vector toler = new(1, 4), infb = new(1, 4), supb = new(1, 4);

        Adaptor2d_Curve2d thecur;
        Adaptor3d_TopolTool Iter;

        if (OnFirst)
        {
            Iter = recdomain1;
        }
        else
        {
            Iter = recdomain2;
        }

        Indexsol = ArcToRecadre(OnFirst, theSol, 0, out lastpt2d, out pt2d, ref pmin);
        IsVtx = false;
        if (Indexsol == 0)
        {
            return false;
        }

        Iter.Init();
        nbarc = 1;
        while (nbarc < Indexsol)
        {
            nbarc++;
            Iter.Next();
        }

        Adaptor2d_Curve2d thearc = Iter.Value();

        if (OnFirst)
        {
            thecur = BRepBlend_BlendTool.CurveOnSurf(thearc, surf1);
        }
        else
        {
            thecur = BRepBlend_BlendTool.CurveOnSurf(thearc, surf2);
        }

        // The problem to solve
        FuncInv.Set(OnFirst, thecur);
        FuncInv.GetBounds(infb, supb);
        infb[2] -= Extrap;
        supb[2] += Extrap;

        FuncInv.GetTolerance(toler, 0.1 * tolpoint3d); // better keep some margin
        FunctionSetRoot rsnld = new(FuncInv, toler, 35);
        toler *= 10; // but make the tests correctly

        // Computation of a starting point
        double ufirst, ulast;
        BRepBlend_BlendTool.Bounds(thecur, out ufirst, out ulast);
        // To help finding singular corners, the parameter may be moved
        if (Math.Abs(pmin - ufirst) < Math.Abs(ulast - ufirst) / 1000)
        {
            pmin = ufirst;
        }
        if (Math.Abs(pmin - ulast) < Math.Abs(ulast - ufirst) / 1000)
        {
            pmin = ulast;
        }

        if (byinter)
        {
            double lastParam = previousP.Parameter();
            // Check that the recadrage is not a jalon
            if (jalons.Length() != 0)
            {
                double t1, t2, t;
                bool Cherche = true;
                int ii;
                if (lastParam < param)
                {
                    t1 = lastParam;
                    t2 = param;
                }
                else
                {
                    t1 = param;
                    t2 = lastParam;
                }
                for (ii = 1; ii <= jalons.Length() && Cherche; ii++)
                {
                    t = jalons.Value(ii).Parameter();
                    if (((t1 < t) && (t2 > t)) || (t == param))
                    {
                        jalons_Trouve = true;
                        LeJalon = ii;
                        Cherche = false; // only works if both are left at once
                    }
                    else
                    {
                        Cherche = t < t2; // stop if t >= t2
                    }
                }
            }
            if (!jalons_Trouve)
            {
                // Initialisation by interpolation
                double lambda, u, v;
                Point2D Pnt, Pnt1, Pnt2;
                thecur.D0(pmin, out Pnt);
                if (OnFirst)
                {
                    previousP.ParametersOnS2(out u, out v);
                    Pnt1 = new Point2D(u, v);
                    Pnt2 = new Point2D(theSol[3], theSol[4]);
                }
                else
                {
                    previousP.ParametersOnS1(out u, out v);
                    Pnt1 = new Point2D(u, v);
                    Pnt2 = new Point2D(theSol[1], theSol[2]);
                }

                lambda = Pnt.Distance(lastpt2d);
                if (lambda > 1.0e-12)
                {
                    lambda /= Pnt.Distance(lastpt2d) + Pnt.Distance(pt2d);
                }
                else
                {
                    lambda = 0;
                }
                solrst[1] = pmin;
                solrst[2] = (1 - lambda) * lastParam + lambda * param;
                solrst[3] = (1 - lambda) * Pnt1.X() + lambda * Pnt2.X();
                solrst[4] = (1 - lambda) * Pnt1.Y() + lambda * Pnt2.Y();
            }
        }
        else
        { // otherwise start from the last computed point
            solrst[1] = pmin;
            solrst[2] = param;
            if (OnFirst)
            {
                solrst[3] = theSol[3];
                solrst[4] = theSol[4];
            }
            else
            {
                solrst[3] = theSol[1];
                solrst[4] = theSol[2];
            }
        }

        if (jalons_Trouve)
        { // take the jalon
            Blend_Point MonJalon;
            bool periodic;
            double uperiod = 0, vperiod = 0;
            Point2D Pnt;
            double distaux;
            MonJalon = jalons.Value(LeJalon);
            solrst[2] = MonJalon.Parameter();
            if (OnFirst)
            {
                MonJalon.ParametersOnS2(out solrst[3], out solrst[4]);
                periodic = (surf2.IsUPeriodic() || surf2.IsVPeriodic());
            }
            else
            {
                MonJalon.ParametersOnS1(out solrst[3], out solrst[4]);
                periodic = (surf1.IsUPeriodic() || surf1.IsVPeriodic());
            }

            // Possible recadrage for the periodic case
            if (periodic)
            {
                Adaptor3d_Surface surf;
                if (OnFirst)
                {
                    surf = surf2;
                }
                else
                {
                    surf = surf1;
                }

                lastpt2d = thecur.Value(pmin);

                if (surf.IsUPeriodic())
                {
                    uperiod = surf.UPeriod();
                    if (solrst[3] - lastpt2d.X() > uperiod * 0.6)
                    {
                        solrst[3] -= uperiod;
                    }
                    if (solrst[3] - lastpt2d.X() < -uperiod * 0.6)
                    {
                        solrst[3] += uperiod;
                    }
                }
                if (surf.IsVPeriodic())
                {
                    vperiod = surf.VPeriod();
                    if (solrst[4] - lastpt2d.Y() > vperiod * 0.6)
                    {
                        solrst[4] -= vperiod;
                    }
                    if (solrst[4] - lastpt2d.Y() < -vperiod * 0.6)
                    {
                        solrst[4] += vperiod;
                    }
                }
            }

            // For the parameter on the arc, project...
            pt2d = new Point2D(solrst[3], solrst[4]);
            Pnt = thecur.Value(ufirst);
            dist = pt2d.Distance(Pnt);
            solrst[1] = ufirst;
            Pnt = thecur.Value(ulast);
            distaux = pt2d.Distance(Pnt);
            if (distaux < dist)
            {
                solrst[1] = ulast;
                dist = distaux;
            }

            if (dist > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                prm = pmin;
                if (OnFirst)
                {
                    ok = BRepBlend_BlendTool.Project(pt2d, surf1, thearc, out prm, out distaux);
                }
                else
                {
                    ok = BRepBlend_BlendTool.Project(pt2d, surf2, thearc, out prm, out distaux);
                }
                if (ok && (pt2d.Distance(thecur.Value(prm)) < dist))
                {
                    solrst[1] = prm;
                }
                else
                {
                    solrst[1] = pmin;
                }
            }
            // Check the jalon
            jalons_Trouve = (FuncInv.IsSolution(solrst, tolpoint3d));
        }

        if (!jalons_Trouve)
        {
            // Solve...
            rsnld.Perform(FuncInv, solrst, infb, supb);
            if (!rsnld.IsDone)
            {
                recadre = false;
            }
            else
            {
                rsnld.Root(solrst);
                recadre = FuncInv.IsSolution(solrst, tolpoint3d);
            }
        }

        // On failure, see whether another arc fits (exits near a vertex)
        dist = (ulast - ufirst) / 100;
        if ((!recadre) && ((Math.Abs(pmin - ulast) < dist) || (Math.Abs(pmin - ufirst) < dist)))
        {
            Indexsol = ArcToRecadre(OnFirst, theSol, Indexsol, out lastpt2d, out pt2d, ref pmin);
            if (Indexsol == 0)
            {
                return false;
            }

            Iter.Init();
            nbarc = 1;
            while (nbarc < Indexsol)
            {
                nbarc++;
                Iter.Next();
            }
            thearc = Iter.Value();

            if (OnFirst)
            {
                thecur = BRepBlend_BlendTool.CurveOnSurf(thearc, surf1);
            }
            else
            {
                thecur = BRepBlend_BlendTool.CurveOnSurf(thearc, surf2);
            }
            solrst[1] = pmin;
            // The problem to solve
            FuncInv.Set(OnFirst, thecur);
            FuncInv.GetBounds(infb, supb);
            FuncInv.GetTolerance(toler, 0.1 * tolpoint3d); // better keep some margin
            FunctionSetRoot aRsnld = new(FuncInv, toler, 35);
            toler *= 10; // but make the tests correctly
            // Solve...
            aRsnld.Perform(FuncInv, solrst, infb, supb);

            if (!aRsnld.IsDone)
            {
                recadre = false;
            }
            else
            {
                aRsnld.Root(solrst);
                recadre = FuncInv.IsSolution(solrst, tolpoint3d);
            }
        }

        if (recadre)
        {
            // Topological classification
            if (OnFirst)
            {
                thecur = BRepBlend_BlendTool.CurveOnSurf(thearc, surf1);
            }
            else
            {
                thecur = BRepBlend_BlendTool.CurveOnSurf(thearc, surf2);
            }
            BRepBlend_BlendTool.Bounds(thecur, out ufirst, out ulast);

            Iter.Initialize(thearc);
            Iter.InitVertexIterator();
            IsVtx = !Iter.MoreVertex();
            while (!IsVtx)
            {
                Vtx = Iter.Vertex();
                vtol = 0.4 * Math.Abs(ulast - ufirst); // an upper bound of the tolerance
                if (vtol > Max(BRepBlend_BlendTool.Tolerance(Vtx, thearc), toler[1]))
                {
                    vtol = Max(BRepBlend_BlendTool.Tolerance(Vtx, thearc), toler[1]);
                }
                if (Math.Abs(BRepBlend_BlendTool.Parameter(Vtx, thearc) - solrst[1]) <= vtol)
                {
                    IsVtx = true; // in the vertex's ball, or the vertex in the recadrage's "ball"
                }
                else
                {
                    Iter.NextVertex();
                    IsVtx = !Iter.MoreVertex();
                }
            }
            if (!Iter.MoreVertex())
            {
                IsVtx = false;
            }
            return true;
        }
        return false;
    }

    private void Transition(bool OnFirst, Adaptor2d_Curve2d A, double Param, ref IntSurf_Transition TLine,
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
        Vector3D d1u, d1v, normale = new(), tgrst = new();
        Direction3D thenormal = new();
        NormalStatus stat;

        BRepBlend_HCurve2dTool.D1(A, Param, out p2d, out dp2d);
        if (OnFirst)
        {
            Adaptor3d_HSurfaceTool.D1(surf1, p2d.X(), p2d.Y(), out pbid, out d1u, out d1v);
            if (!computetranstionaveclacorde)
            {
                tgline = previousP.TangentOnS1();
            }
            else
            {
                tgline = new Vector3D(prevprev.PointOnS1(), previousP.PointOnS1());
            }
        }
        else
        {
            Adaptor3d_HSurfaceTool.D1(surf2, p2d.X(), p2d.Y(), out pbid, out d1u, out d1v);
            if (!computetranstionaveclacorde)
            {
                tgline = previousP.TangentOnS2();
            }
            else
            {
                tgline = new Vector3D(prevprev.PointOnS2(), previousP.PointOnS2());
            }
        }

        tgrst.SetLinearForm(dp2d.X(), d1u, dp2d.Y(), d1v);

        global::Kerf.Engine.Geometry.CSLib.Normal(d1u, d1v, 1.0e-9, out stat, ref thenormal);
        if (stat == Defined)
        {
            normale.SetXYZ(thenormal.XYZ());
        }
        else
        {
            Adaptor3d_Surface surf;
            if (OnFirst)
            {
                surf = surf1;
            }
            else
            {
                surf = surf2;
            }
            int iu, iv;
            NCollection_Array2<Vector3D> Der = new(0, 2, 0, 2);
            Adaptor3d_HSurfaceTool.D2(surf, p2d.X(), p2d.Y(), out pbid, out Der[1, 0], out Der[0, 1], out Der[2, 0],
                                      out Der[0, 2], out Der[1, 1]);
            Der[2, 1] = Adaptor3d_HSurfaceTool.DN(surf, p2d.X(), p2d.Y(), 2, 1);
            Der[1, 2] = Adaptor3d_HSurfaceTool.DN(surf, p2d.X(), p2d.Y(), 1, 2);
            Der[2, 2] = Adaptor3d_HSurfaceTool.DN(surf, p2d.X(), p2d.Y(), 2, 2);
            global::Kerf.Engine.Geometry.CSLib.Normal(2, Der, 1.0e-9, p2d.X(), p2d.Y(), Adaptor3d_HSurfaceTool.FirstUParameter(surf),
                         Adaptor3d_HSurfaceTool.LastUParameter(surf), Adaptor3d_HSurfaceTool.FirstVParameter(surf),
                         Adaptor3d_HSurfaceTool.LastVParameter(surf), out stat, ref thenormal, out iu, out iv);
            normale.SetXYZ(thenormal.XYZ());
            // OCCT_DEBUG trace left out.
        }

        IntSurf.MakeTransition(tgline, tgrst, new Direction3D(normale), ref TLine, ref TArc);
    }

    private void MakeExtremity(ref BRepBlend_Extremity Extrem, bool OnFirst, int Index, double Param, bool IsVtx,
                               Adaptor3d_HVertex? Vtx)
    {
        IntSurf_Transition Tline = new(), Tarc = new();
        int nbarc;
        Adaptor3d_TopolTool Iter;

        if (OnFirst)
        {
            Extrem.SetValue(previousP.PointOnS1(), sol[1], sol[2], previousP.Parameter(), tolpoint3d);
            if (!previousP.IsTangencyPoint())
            {
                Extrem.SetTangent(previousP.TangentOnS1());
            }
            Iter = recdomain1;
        }
        else
        {
            Extrem.SetValue(previousP.PointOnS2(), sol[3], sol[4], previousP.Parameter(), tolpoint3d);
            if (!previousP.IsTangencyPoint())
            {
                Extrem.SetTangent(previousP.TangentOnS2());
            }
            Iter = recdomain2;
        }

        Iter.Init();
        nbarc = 1;

        while (nbarc < Index)
        {
            nbarc++;
            Iter.Next();
        }

        Transition(OnFirst, Iter.Value(), Param, ref Tline, ref Tarc);
        Extrem.AddArc(Iter.Value(), Param, Tline, Tarc);
        if (IsVtx)
        {
            Extrem.SetVertex(Vtx!);
        }
    }

    private void MakeSingularExtremity(ref BRepBlend_Extremity Extrem, bool OnFirst, Adaptor3d_HVertex Vtx)
    {
        IntSurf_Transition Tline = new(), Tarc = new();
        Adaptor3d_TopolTool Iter;
        double prm;

        if (OnFirst)
        {
            Iter = recdomain1;
            if (!previousP.IsTangencyPoint())
            {
                Extrem.SetTangent(previousP.TangentOnS1());
            }
        }
        else
        {
            if (!previousP.IsTangencyPoint())
            {
                Extrem.SetTangent(previousP.TangentOnS2());
            }
            Iter = recdomain2;
        }

        Iter.Init();
        Extrem.SetVertex(Vtx);
        while (Iter.More())
        {
            Adaptor2d_Curve2d arc = Iter.Value();
            Iter.Initialize(arc);
            Iter.InitVertexIterator();
            while (Iter.MoreVertex())
            {
                if (Iter.Identical(Vtx, Iter.Vertex()))
                {
                    prm = BRepBlend_BlendTool.Parameter(Vtx, arc);
                    Transition(OnFirst, arc, prm, ref Tline, ref Tarc);
                    Extrem.AddArc(arc, prm, Tline, Tarc);
                }
                Iter.NextVertex();
            }
            Iter.Next();
        }
    }

    private static void RecadreIfPeriodic(ref double NewU, ref double NewV, double OldU, double OldV, double UPeriod,
                                          double VPeriod)
    {
        if (UPeriod > 0.0)
        {
            double sign = (NewU < OldU) ? 1 : -1;
            while (Math.Abs(NewU - OldU) > UPeriod / 2)
            {
                NewU += sign * UPeriod;
            }
        }
        if (VPeriod > 0.0)
        {
            double sign = (NewV < OldV) ? 1 : -1;
            while (Math.Abs(NewV - OldV) > VPeriod / 2)
            {
                NewV += sign * VPeriod;
            }
        }
    }

    private static void evalpinit(math_Vector parinit, in Blend_Point previousP, double parprec, double param,
                                  math_Vector infbound, math_Vector supbound, bool classonS1, bool classonS2)
    {
        if (previousP.IsTangencyPoint())
        {
            previousP.ParametersOnS1(out parinit[1], out parinit[2]);
            previousP.ParametersOnS2(out parinit[3], out parinit[4]);
        }
        else
        {
            double u1, v1, u2, v2;
            double du1, dv1, du2, dv2;
            bool Inside = true;
            previousP.ParametersOnS1(out u1, out v1);
            previousP.ParametersOnS2(out u2, out v2);
            previousP.Tangent2dOnS1().Coord(out du1, out dv1);
            previousP.Tangent2dOnS2().Coord(out du2, out dv2);
            double step = param - parprec;
            u1 += step * du1;
            v1 += step * dv1;
            if (classonS1)
            {
                if ((u1 < infbound[1]) || (u1 > supbound[1]))
                {
                    Inside = false;
                }
                if ((v1 < infbound[2]) || (v1 > supbound[2]))
                {
                    Inside = false;
                }
            }
            u2 += step * du2;
            v2 += step * dv2;
            if (classonS2)
            {
                if ((u2 < infbound[3]) || (u2 > supbound[3]))
                {
                    Inside = false;
                }
                if ((v2 < infbound[4]) || (v2 > supbound[4]))
                {
                    Inside = false;
                }
            }

            if (Inside)
            {
                parinit[1] = u1;
                parinit[2] = v1;
                parinit[3] = u2;
                parinit[4] = v2;
            }
            else
            { // don't play the smart one
                previousP.ParametersOnS1(out parinit[1], out parinit[2]);
                previousP.ParametersOnS2(out parinit[3], out parinit[4]);
            }
        }
    }

    private void InternalPerform(Blend_Function Func, Blend_FuncInv FuncInv, double Bound)
    {
        double Cosi = 0.0, Cosi2 = 0.0;

        double stepw = pasmax;
        int nbp = line!.NbPoints();
        if (nbp >= 2)
        { // Take the last step back if it is not too small.
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
        Vector3D TgOnGuide, PrevTgOnGuide;
        Point3D PtOnGuide;
        hguide.D1(parprec, out PtOnGuide, out TgOnGuide);
        PrevTgOnGuide = TgOnGuide;

        if (sens * (parprec - Bound) >= -tolgui)
        {
            return;
        }
        Blend_Status State = Blend_OnRst12;
        TopAbs_State situ1 = TopAbs_IN, situ2 = TopAbs_IN;
        double w1, w2;
        int Index1 = 0, Index2 = 0, nbarc;
        bool Arrive, recad1, recad2, control;
        bool Isvtx1 = false, Isvtx2 = false, echecrecad;
        Point2D p2d;
        math_Vector tolerance = new(1, 4), infbound = new(1, 4), supbound = new(1, 4), parinit = new(1, 4);
        math_Vector solrst1 = new(1, 4), solrst2 = new(1, 4);
        Adaptor3d_HVertex? Vtx1 = null, Vtx2 = null;
        BRepBlend_Extremity Ext1 = new(), Ext2 = new();

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

        evalpinit(parinit, previousP, parprec, param, infbound, supbound, clasonS1, clasonS2);

        while (!Arrive)
        {
            hguide.D1(param, out PtOnGuide, out TgOnGuide);
            // Check deflection on guide
            Cosi = PrevTgOnGuide * TgOnGuide;
            if (Cosi < Constants.Resolution())
            { // angle>=pi/2 or null magnitude
                Cosi2 = 0.0;
            }
            else
            {
                Cosi2 = Cosi * Cosi / PrevTgOnGuide.SquareMagnitude() / TgOnGuide.SquareMagnitude();
            }
            if (Cosi2 < CosRef3D_File) // angle 3d too great
            {
                State = Blend_StepTooLarge;
                stepw = stepw / 2.0;
                param = parprec + sens * stepw; // no risk of going past Bound.
                if (Math.Abs(stepw) < tolgui)
                {
                    Ext1.SetValue(previousP.PointOnS1(), sol[1], sol[2], previousP.Parameter(), tolpoint3d);
                    Ext2.SetValue(previousP.PointOnS2(), sol[3], sol[4], previousP.Parameter(), tolpoint3d);
                    if (!previousP.IsTangencyPoint())
                    {
                        Ext1.SetTangent(previousP.TangentOnS1());
                        Ext2.SetTangent(previousP.TangentOnS2());
                    }
                    Arrive = true;
                }
                continue;
            }
            PrevTgOnGuide = TgOnGuide;
            //////////////////////////

            bool bonpoint = true;
            Func.Set(param);
            rsnld.Perform(Func, parinit, infbound, supbound);

            if (!rsnld.IsDone)
            {
                State = Blend_StepTooLarge;
                bonpoint = false;
            }
            else
            {
                rsnld.Root(sol);

                if (clasonS1)
                {
                    situ1 = domain1.Classify(new Point2D(sol[1], sol[2]), Min(tolerance[1], tolerance[2]), false);
                }
                else
                {
                    situ1 = TopAbs_IN;
                }
                if (clasonS2)
                {
                    situ2 = domain2.Classify(new Point2D(sol[3], sol[4]), Min(tolerance[3], tolerance[4]), false);
                }
                else
                {
                    situ2 = TopAbs_IN;
                }
            }
            if (bonpoint && line.NbPoints() == 1 && (situ1 != TopAbs_IN || situ2 != TopAbs_IN))
            {
                State = Blend_StepTooLarge;
                bonpoint = false;
            }
            if (bonpoint)
            {
                w1 = w2 = Bound;
                recad1 = false;
                recad2 = false;
                echecrecad = false;
                control = false;

                if (situ1 == TopAbs_OUT || situ1 == TopAbs_ON)
                {
                    // inverse problem on surf1
                    // If the recadrage goes in the direction of the progression within a tolerance, the wrong
                    // solution was taken.
                    recad1 = Recadre(FuncInv, true, sol, solrst1, out Index1, out Isvtx1, ref Vtx1);

                    if (recad1)
                    {
                        double wtemp;
                        wtemp = solrst1[2];
                        if ((param - wtemp) / sens >= -10 * tolgui)
                        {
                            w1 = solrst1[2];
                            control = true;
                        }
                        else
                        {
                            echecrecad = true;
                            recad1 = false;
                            State = Blend_StepTooLarge;
                            bonpoint = false;
                            stepw = stepw / 2.0;
                        }
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }
                if (situ2 == TopAbs_OUT || situ2 == TopAbs_ON)
                {
                    // inverse problem on surf2
                    recad2 = Recadre(FuncInv, false, sol, solrst2, out Index2, out Isvtx2, ref Vtx2);

                    if (recad2)
                    {
                        double wtemp;
                        wtemp = solrst2[2];
                        if ((param - wtemp) / sens >= -10 * tolgui)
                        {
                            w2 = solrst2[2];
                            control = true;
                        }
                        else
                        {
                            echecrecad = true;
                            recad2 = false;
                            State = Blend_StepTooLarge;
                            bonpoint = false;
                            stepw = stepw / 2.0;
                        }
                    }
                    else
                    {
                        echecrecad = true;
                    }
                }

                // What must be checked
                if (recad1 && recad2)
                {
                    if (Math.Abs(w1 - w2) <= 10 * tolgui)
                    {
                        // no need to check the recadrages, the check could fail (cf model blend10). The tolerance
                        // is large so that the next walking can put a few sections...
                        control = false;
                    }
                    else if (sens * (w1 - w2) < 0.0)
                    {
                        // solution on 1?
                        recad2 = false;
                    }
                    else
                    {
                        // solution on 2?
                        recad1 = false;
                    }
                }

                // Effective check of the recadrages
                if (control)
                {
                    TopAbs_State situ;
                    if (recad1 && clasonS2)
                    {
                        situ = recdomain2.Classify(new Point2D(solrst1[3], solrst1[4]),
                                                   Min(tolerance[3], tolerance[4]));
                        if (situ == TopAbs_OUT)
                        {
                            recad1 = false;
                            echecrecad = true;
                        }
                    }
                    else if (recad2 && clasonS1)
                    {
                        situ = recdomain1.Classify(new Point2D(solrst2[3], solrst2[4]),
                                                   Min(tolerance[1], tolerance[1]));
                        if (situ == TopAbs_OUT)
                        {
                            recad2 = false;
                            echecrecad = true;
                        }
                    }
                }

                if (recad1 || recad2)
                {
                    echecrecad = false;
                }

                if (!echecrecad)
                {
                    if (recad1 && recad2)
                    {
                        // solution on 1 and 2 at once
                        // Go through the arcs, to avoid problems with periodic surfaces.
                        State = Blend_OnRst12;
                        param = (w1 + w2) / 2;
                        Point3D Pnt1, Pnt2;
                        p2d = BRepBlend_HCurve2dTool.Value(recdomain1.Value(), solrst1[1]);
                        sol[1] = p2d.X();
                        sol[2] = p2d.Y();
                        Pnt1 = Adaptor3d_HSurfaceTool.Value(surf1, sol[1], sol[2]);
                        p2d = BRepBlend_HCurve2dTool.Value(recdomain2.Value(), solrst2[1]);
                        sol[3] = p2d.X();
                        sol[4] = p2d.Y();
                        Pnt2 = Adaptor3d_HSurfaceTool.Value(surf2, sol[3], sol[4]);
                        const double TolProd = 1.0e-5;
                        double[] SavedParams = new double[2];
                        bool[] SameDirs = { false, false };
                        ChFiDS_ElSpine theElSpine = hguide;
                        SavedParams[0] = theElSpine.GetSavedFirstParameter();
                        SavedParams[1] = theElSpine.GetSavedLastParameter();
                        for (int ind = 0; ind < 2; ind++)
                        {
                            if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(SavedParams[ind]))
                            {
                                // Check the original first and last parameters of guide curve for equality to
                                // found parameter <param>: check equality of tangent to guide curve and normal to
                                // plane built on 3 points: point on guide curve and points on restrictions of
                                // adjacent surfaces.
                                Point3D Pnt0;
                                Vector3D Dir0;
                                hguide.D1(SavedParams[ind], out Pnt0, out Dir0);
                                double Length = Dir0.Magnitude();
                                if (Length <= Constants.Resolution())
                                {
                                    continue;
                                }
                                Dir0 /= Length;
                                gce_MakePln PlaneBuilder = new(Pnt0, Pnt1, Pnt2);
                                if (!PlaneBuilder.IsDone())
                                {
                                    continue;
                                }
                                Plane3D thePlane = PlaneBuilder.Value();
                                Direction3D DirPlane = thePlane.Axis().Direction();
                                Vector3D theProd = Dir0 ^ new Vector3D(DirPlane);
                                double ProdMod = theProd.Magnitude();
                                if (ProdMod <= TolProd)
                                {
                                    SameDirs[ind] = true;
                                }
                            }
                        }
                        double theParam = global::Kerf.Engine.Foundation.Precision.Infinite();
                        // Choose the closest parameter
                        if (SameDirs[0] && SameDirs[1])
                        {
                            theParam = (Math.Abs(param - SavedParams[0]) < Math.Abs(param - SavedParams[1]))
                                         ? SavedParams[0]
                                         : SavedParams[1];
                        }
                        else if (SameDirs[0])
                        {
                            theParam = SavedParams[0];
                        }
                        else if (SameDirs[1])
                        {
                            theParam = SavedParams[1];
                        }

                        double NewU = 0.0, NewV = 0.0, NewParam = 0.0;
                        Point3D NewPnt = new();
                        bool Corrected = CorrectExtremityOnOneRst(1, sol[3], sol[4], param, Pnt1, ref NewU, ref NewV,
                                                                  ref NewPnt, ref NewParam);
                        if (Corrected)
                        {
                            if (Math.Abs(param - NewParam) < Math.Abs(param - theParam))
                            {
                                theParam = NewParam;
                            }
                        }

                        if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(theParam))
                        {
                            param = theParam;
                        }
                    }
                    else if (recad1)
                    {
                        // solution on 1
                        State = Blend_OnRst1;
                        param = w1;
                        recdomain1.Init();
                        nbarc = 1;
                        while (nbarc < Index1)
                        {
                            nbarc++;
                            recdomain1.Next();
                        }
                        p2d = BRepBlend_HCurve2dTool.Value(recdomain1.Value(), solrst1[1]);
                        sol[1] = p2d.X();
                        sol[2] = p2d.Y();
                        sol[3] = solrst1[3];
                        sol[4] = solrst1[4];
                        Point3D thePntOnRst = Adaptor3d_HSurfaceTool.Value(surf1, sol[1], sol[2]);
                        double NewU = 0.0, NewV = 0.0, NewParam = 0.0;
                        Point3D NewPnt = new();
                        bool Corrected = CorrectExtremityOnOneRst(1, sol[3], sol[4], param, thePntOnRst, ref NewU,
                                                                  ref NewV, ref NewPnt, ref NewParam);
                        if (Corrected)
                        {
                            param = NewParam;
                            sol[3] = NewU;
                            sol[4] = NewV;
                        }
                    }
                    else if (recad2)
                    {
                        // solution on 2
                        State = Blend_OnRst2;
                        param = w2;

                        recdomain2.Init();
                        nbarc = 1;
                        while (nbarc < Index2)
                        {
                            nbarc++;
                            recdomain2.Next();
                        }
                        p2d = BRepBlend_HCurve2dTool.Value(recdomain2.Value(), solrst2[1]);
                        sol[1] = solrst2[3];
                        sol[2] = solrst2[4];
                        sol[3] = p2d.X();
                        sol[4] = p2d.Y();
                        Point3D thePntOnRst = Adaptor3d_HSurfaceTool.Value(surf2, sol[3], sol[4]);
                        double NewU = 0.0, NewV = 0.0, NewParam = 0.0;
                        Point3D NewPnt = new();
                        bool Corrected = CorrectExtremityOnOneRst(2, sol[1], sol[2], param, thePntOnRst, ref NewU,
                                                                  ref NewV, ref NewPnt, ref NewParam);
                        if (Corrected)
                        {
                            param = NewParam;
                            sol[1] = NewU;
                            sol[2] = NewV;
                        }
                    }
                    else
                    {
                        State = Blend_OK;
                    }

                    bool testdefl = true;
                    // OCCT_DEBUG: testdefl = !Blend_GetcontextNOTESTDEFL(), left out.
                    if (recad1 || recad2)
                    {
                        Func.Set(param);
                        // Better an unorthodox step than no recadrage!! PMN
                        State = TestArret(Func, State, (testdefl && (Math.Abs(stepw) > 3 * tolgui)), false, true);
                    }
                    else
                    {
                        State = TestArret(Func, State, testdefl);
                    }
                }
                else
                {
                    // Or the max step is badly set. Divide.
                    if (stepw > 2 * tolgui)
                    {
                        State = Blend_StepTooLarge;
                        // Otherwise the recadrage failed. Leave with SamePoints
                    }
                    else
                    {
                        State = Blend_SamePoints;
                    }
                }
            }

            // OCCT_DEBUG Drawsect left out.
            switch (State)
            {
                case Blend_OK:
                {
                    // Update the line.
                    if (sens > 0.0)
                    {
                        line.Append(previousP);
                    }
                    else
                    {
                        line.Prepend(previousP);
                    }

                    parprec = param;

                    if (param == Bound)
                    {
                        Arrive = true;
                        Ext1.SetValue(previousP.PointOnS1(), sol[1], sol[2], previousP.Parameter(), tolpoint3d);
                        Ext2.SetValue(previousP.PointOnS2(), sol[3], sol[4], previousP.Parameter(), tolpoint3d);
                        if (!previousP.IsTangencyPoint())
                        {
                            Ext1.SetTangent(previousP.TangentOnS1());
                            Ext2.SetTangent(previousP.TangentOnS2());
                        }

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
                    evalpinit(parinit, previousP, parprec, param, infbound, supbound, clasonS1, clasonS2);
                }
                break;

                case Blend_StepTooLarge:
                {
                    stepw = stepw / 2.0;
                    if (Math.Abs(stepw) < tolgui)
                    {
                        Ext1.SetValue(previousP.PointOnS1(), sol[1], sol[2], previousP.Parameter(), tolpoint3d);
                        Ext2.SetValue(previousP.PointOnS2(), sol[3], sol[4], previousP.Parameter(), tolpoint3d);
                        if (!previousP.IsTangencyPoint())
                        {
                            Ext1.SetTangent(previousP.TangentOnS1());
                            Ext2.SetTangent(previousP.TangentOnS2());
                        }
                        Arrive = true;
                    }
                    else
                    {
                        param = parprec + sens * stepw; // no risk of going past Bound.
                        evalpinit(parinit, previousP, parprec, param, infbound, supbound, clasonS1, clasonS2);
                    }
                }
                break;

                case Blend_StepTooSmall:
                {
                    // Update the line.
                    if (sens > 0.0)
                    {
                        line.Append(previousP);
                    }
                    else
                    {
                        line.Prepend(previousP);
                    }

                    parprec = param;

                    stepw = Min(1.5 * stepw, pasmax);
                    if (param == Bound)
                    {
                        Arrive = true;
                        Ext1.SetValue(previousP.PointOnS1(), sol[1], sol[2], previousP.Parameter(), tolpoint3d);
                        Ext2.SetValue(previousP.PointOnS2(), sol[3], sol[4], previousP.Parameter(), tolpoint3d);
                        if (!previousP.IsTangencyPoint())
                        {
                            Ext1.SetTangent(previousP.TangentOnS1());
                            Ext2.SetTangent(previousP.TangentOnS2());
                        }
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
                    evalpinit(parinit, previousP, parprec, param, infbound, supbound, clasonS1, clasonS2);
                }
                break;

                case Blend_OnRst1:
                {
                    if (sens > 0.0)
                    {
                        line.Append(previousP);
                    }
                    else
                    {
                        line.Prepend(previousP);
                    }

                    MakeExtremity(ref Ext1, true, Index1, solrst1[1], Isvtx1, Vtx1);
                    // Guard the singular case where one of the recadrages failed
                    if (previousP.PointOnS1().IsEqual(previousP.PointOnS2(), 2.0 * tolpoint3d))
                    {
                        Ext2.SetValue(previousP.PointOnS1(), sol[3], sol[4], tolpoint3d); // OCCT: the curve overload
                        if (Isvtx1)
                        {
                            MakeSingularExtremity(ref Ext2, false, Vtx1!);
                        }
                    }
                    else
                    {
                        Ext2.SetValue(previousP.PointOnS2(), sol[3], sol[4], previousP.Parameter(), tolpoint3d);
                    }
                    Arrive = true;
                }
                break;

                case Blend_OnRst2:
                {
                    if (sens > 0.0)
                    {
                        line.Append(previousP);
                    }
                    else
                    {
                        line.Prepend(previousP);
                    }

                    // Guard the singular case where one of the recadrages failed
                    if (previousP.PointOnS1().IsEqual(previousP.PointOnS2(), 2.0 * tolpoint3d))
                    {
                        Ext1.SetValue(previousP.PointOnS2(), sol[1], sol[2], tolpoint3d); // OCCT: the curve overload
                        if (Isvtx2)
                        {
                            MakeSingularExtremity(ref Ext1, true, Vtx2!);
                        }
                    }
                    else
                    {
                        Ext1.SetValue(previousP.PointOnS1(), sol[1], sol[2], previousP.Parameter(), tolpoint3d);
                    }
                    MakeExtremity(ref Ext2, false, Index2, solrst2[1], Isvtx2, Vtx2);
                    Arrive = true;
                }
                break;

                case Blend_OnRst12:
                {
                    if (sens > 0.0)
                    {
                        line.Append(previousP);
                    }
                    else
                    {
                        line.Prepend(previousP);
                    }

                    if ((Isvtx1 != Isvtx2) && (previousP.PointOnS1().IsEqual(previousP.PointOnS2(), 2.0 * tolpoint3d)))
                    {
                        // Guard the singular case where only one recadrage is recognised as a vertex.
                        if (Isvtx1)
                        {
                            Isvtx2 = true;
                            Vtx2 = Vtx1;
                        }
                        else
                        {
                            Isvtx1 = true;
                            Vtx1 = Vtx2;
                        }
                    }

                    MakeExtremity(ref Ext1, true, Index1, solrst1[1], Isvtx1, Vtx1);
                    MakeExtremity(ref Ext2, false, Index2, solrst2[1], Isvtx2, Vtx2);
                    Arrive = true;
                }
                break;

                case Blend_SamePoints:
                {
                    // Stop
                    Ext1.SetValue(previousP.PointOnS1(), sol[1], sol[2], previousP.Parameter(), tolpoint3d);
                    Ext2.SetValue(previousP.PointOnS2(), sol[3], sol[4], previousP.Parameter(), tolpoint3d);
                    if (!previousP.IsTangencyPoint())
                    {
                        Ext1.SetTangent(previousP.TangentOnS1());
                        Ext2.SetTangent(previousP.TangentOnS2());
                    }
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
                    line.SetEndPoints(Ext1, Ext2);
                }
                else
                {
                    line.SetStartPoints(Ext1, Ext2);
                }
            }
        }
    }

    // NewU, NewV and NewPoint are written only on success; NewParam as soon as the guide is projected (OCCT's
    // references), so they are ref parameters.
    private bool CorrectExtremityOnOneRst(int IndexOfRst, double theU, double theV, double theParam,
                                          in Point3D thePntOnRst, ref double NewU, ref double NewV,
                                          ref Point3D NewPoint, ref double NewParam)
    {
        const double TolAng = 0.001; // bug OCC25701

        ChFiDS_ElSpine theElSpine = hguide;
        if (theElSpine.NbVertices() == 0)
        {
            return false;
        }

        Adaptor3d_TopolTool DomainOfRst = (IndexOfRst == 1) ? recdomain1 : recdomain2;
        Adaptor3d_Surface SurfOfRst = (IndexOfRst == 1) ? surf1 : surf2;
        Adaptor3d_Surface AnotherSurf = (IndexOfRst == 1) ? surf2 : surf1;

        // Correct point on surface 2
        // First we find right <param>
        Span<double> Ends = stackalloc double[2];
        Ends[0] = BRepBlend_HCurve2dTool.FirstParameter(DomainOfRst.Value());
        Ends[1] = BRepBlend_HCurve2dTool.LastParameter(DomainOfRst.Value());
        double GlobalMinSqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
        double ParamOnGuide = 0;
        Point3D PointOnGuide = new();
        for (int k = 0; k < 2; k++)
        {
            Point2D P2dOnEnd = BRepBlend_HCurve2dTool.Value(DomainOfRst.Value(), Ends[k]);
            Point3D PntOnEnd = Adaptor3d_HSurfaceTool.Value(SurfOfRst, P2dOnEnd.X(), P2dOnEnd.Y());
            Extrema_ExtPC projoncurv = new(PntOnEnd, theElSpine);
            if (!projoncurv.IsDone())
            {
                continue;
            }
            double MinSqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
            int imin = 0;
            for (int ind = 1; ind <= projoncurv.NbExt(); ind++)
            {
                double aSqDist = projoncurv.SquareDistance(ind);
                if (aSqDist < MinSqDist)
                {
                    MinSqDist = aSqDist;
                    imin = ind;
                }
            }
            if (MinSqDist < GlobalMinSqDist)
            {
                GlobalMinSqDist = MinSqDist;
                ParamOnGuide = projoncurv.Point(imin).Parameter();
                PointOnGuide = projoncurv.Point(imin).Value();
            }
        }
        NewParam = ParamOnGuide;
        if (hguide.IsPeriodic())
        {
            double Period = hguide.Period();
            double sign = (NewParam < theParam) ? 1 : -1;
            while (Math.Abs(NewParam - theParam) > Period / 2)
            {
                NewParam += sign * Period;
            }
        }

        // Second we find right point and tangent on guide
        GlobalMinSqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
        Axis3D theAx1 = new();
        for (int ind = 1; ind <= theElSpine.NbVertices(); ind++)
        {
            Axis3D anAx1 = theElSpine.VertexWithTangent(ind);
            Point3D aPnt = anAx1.Location();
            double aSqDist = PointOnGuide.SquareDistance(aPnt);
            if (aSqDist < GlobalMinSqDist)
            {
                GlobalMinSqDist = aSqDist;
                theAx1 = anAx1;
            }
        }
        Point3D Pnt0 = theAx1.Location();
        Direction3D Dir0 = theAx1.Direction();
        // Check new point: is it real solution?
        Point3D OldPonGuide = hguide.Value(theParam);
        Point3D PntOnSurf2 = Adaptor3d_HSurfaceTool.Value(AnotherSurf, theU, theV); // old point
        gce_MakePln PlaneBuilder = new(thePntOnRst, OldPonGuide, PntOnSurf2);
        if (!PlaneBuilder.IsDone())
        {
            return false;
        }
        Plane3D OldPlane = PlaneBuilder.Value();
        Direction3D OldDir = OldPlane.Axis().Direction();
        double Angle = OldDir.Angle(Dir0);
        if (Angle > M_PI / 2)
        {
            Angle = M_PI - Angle;
        }
        if (Angle > TolAng)
        {
            return false;
        }
        ///////////////////////////////////////
        // Project the point(theU,theV) on the plane(Pnt0,Dir0)
        Vector3D aVec = new(Pnt0, PntOnSurf2);
        Vector3D aTranslation = new((aVec.XYZ() * Dir0.XYZ()) * Dir0.XYZ());
        Point3D PntOnPlane = PntOnSurf2.Translated(-aTranslation);

        // Check new point again: does point on restriction belong to the plane?
        PlaneBuilder = new gce_MakePln(thePntOnRst, Pnt0, PntOnPlane);
        if (!PlaneBuilder.IsDone())
        {
            return false;
        }
        Plane3D NewPlane = PlaneBuilder.Value();
        Direction3D DirOfNewPlane = NewPlane.Axis().Direction();
        Angle = Dir0.Angle(DirOfNewPlane);
        if (Angle > M_PI / 2)
        {
            Angle = M_PI - Angle;
        }
        if (Angle > TolAng)
        {
            return false;
        }
        ////////////////////////////////////////////////////////////////////////

        // Project the point <PntOnPlane> on the surface 2
        Extrema_ExtPS projonsurf = new(PntOnPlane, AnotherSurf, global::Kerf.Engine.Foundation.Precision.PConfusion(), global::Kerf.Engine.Foundation.Precision.PConfusion(),
                                       Extrema_ExtFlag_MIN);
        if (projonsurf.IsDone())
        {
            double MinSqDist = global::Kerf.Engine.Foundation.Precision.Infinite();
            int imin = 0;
            for (int ind = 1; ind <= projonsurf.NbExt(); ind++)
            {
                double aSqDist = projonsurf.SquareDistance(ind);
                if (aSqDist < MinSqDist)
                {
                    MinSqDist = aSqDist;
                    imin = ind;
                }
            }
            if (imin != 0)
            {
                Extrema_POnSurf NewPOnSurf2 = projonsurf.Point(imin);
                NewPoint = NewPOnSurf2.Value();
                NewPOnSurf2.Parameter(out NewU, out NewV);
                double uperiod = (AnotherSurf.IsUPeriodic()) ? AnotherSurf.UPeriod() : 0.0;
                double vperiod = (AnotherSurf.IsVPeriodic()) ? AnotherSurf.VPeriod() : 0.0;
                RecadreIfPeriodic(ref NewU, ref NewV, theU, theV, uperiod, vperiod);
                return true;
            }
        }

        return false;
    }
}
