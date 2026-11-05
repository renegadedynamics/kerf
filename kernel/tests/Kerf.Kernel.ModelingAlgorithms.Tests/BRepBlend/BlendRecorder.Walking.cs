// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Fillets.Blend;
using Kerf.Engine.Fillets.BlendFunctions;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The walking part of tools/blendrec (unit 7K) in C#, line for line: each BRepBlend walker on simple configurations,
/// the line it builds point by point with its extremities, then the approximations of the line.
/// </summary>
internal sealed partial class BlendRecorder
{
    private void RecTrans(WalkingTransition T)
    {
        Int((int)T.TransitionType());
        if (T.TransitionType() != WalkingTypeTrans.Undecided)
        {
            Int(T.IsTangent ? 1 : 0);
        }
        if (T.TransitionType() == WalkingTypeTrans.Touch)
        {
            Int((int)T.Situation());
            Int(T.IsOpposite ? 1 : 0);
        }
    }

    private void RecPoint(in BlendPoint P)
    {
        Tag("pt");
        Hex(P.Parameter());
        Int(P.IsTangencyPoint ? 1 : 0);
        Pnt(P.PointOnS1());
        Pnt(P.PointOnS2());
        double u, v;
        try
        {
            P.ParametersOnS1(out u, out v);
            Hex(u);
            Hex(v);
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            Tag(" X");
        }
        try
        {
            P.ParametersOnS2(out u, out v);
            Hex(u);
            Hex(v);
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            Tag(" X");
        }
        try
        {
            Hex(P.ParameterOnC1());
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            Tag(" X");
        }
        try
        {
            Hex(P.ParameterOnC2());
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            Tag(" X");
        }
        if (!P.IsTangencyPoint)
        {
            Vec(P.TangentOnS1());
            Vec(P.TangentOnS2());
            try
            {
                Vec2d(P.Tangent2dOnS1());
            }
            catch (Exception caught) when (Refusal.IsFailure(caught))
            {
                Tag(" X");
            }
            try
            {
                Vec2d(P.Tangent2dOnS2());
            }
            catch (Exception caught) when (Refusal.IsFailure(caught))
            {
                Tag(" X");
            }
        }
        End();
    }

    private void RecExt(string theTag, in Extremity E)
    {
        Tag(theTag);
        Pnt(E.Value());
        E.Parameters(out double u, out double v);
        Hex(u);
        Hex(v);
        Hex(E.ParameterOnGuide());
        Hex(E.Tolerance());
        Int(E.HasTangent ? 1 : 0);
        if (E.HasTangent)
        {
            Vec(E.Tangent());
        }
        Int(E.IsVertex ? 1 : 0);
        Int(E.PointOnRstCount);
        for (int i = 1; i <= E.PointOnRstCount; i++)
        {
            PointOnRst R = E.PointOnRst(i);
            Hex(R.ParameterOnArc());
            RecTrans(R.TransitionOnLine());
            RecTrans(R.TransitionOnArc());
        }
        End();
    }

    private void RecLine(Line L)
    {
        Tag("line");
        Int(L.NbPoints());
        try
        {
            Int((int)L.TransitionOnS1());
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            Tag(" X");
        }
        try
        {
            Int((int)L.TransitionOnS2());
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            Tag(" X");
        }
        End();
        for (int i = 1; i <= L.NbPoints(); i++)
        {
            RecPoint(L.Point(i));
        }
        RecExt("start1", L.StartPointOnFirst());
        RecExt("start2", L.StartPointOnSecond());
        RecExt("end1", L.EndPointOnFirst());
        RecExt("end2", L.EndPointOnSecond());
    }

    private void RecFuncState(AppFunction F)
    {
        Tag("funcstate");
        try
        {
            Hex(F.GetMinimalDistance());
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            Tag(" X");
        }
        Hex(F.GetSectionSize());
        End();
    }

    private void RecApprox(Approx A)
    {
        A.SurfShape(out int ud, out int vd, out int nup, out int nvp, out int nuk, out int nvk);
        Tag("surfshape");
        Int(ud);
        Int(vd);
        Int(nup);
        Int(nvp);
        Int(nuk);
        Int(nvk);
        End();
        NCollection_Array2<Point3D> aPoles = A.SurfPoles();
        NCollection_Array2<double> aW = A.SurfWeights();
        Tag("poles");
        for (int i = aPoles.LowerRow(); i <= aPoles.UpperRow(); i++)
        {
            for (int j = aPoles.LowerCol(); j <= aPoles.UpperCol(); j++)
            {
                Pnt(aPoles[i, j]);
                Hex(aW[i, j]);
            }
        }
        End();
        Tag("uknots");
        for (int i = A.SurfUKnots().Lower(); i <= A.SurfUKnots().Upper(); i++)
        {
            Hex(A.SurfUKnots()[i]);
            Int(A.SurfUMults()[i]);
        }
        End();
        Tag("vknots");
        for (int i = A.SurfVKnots().Lower(); i <= A.SurfVKnots().Upper(); i++)
        {
            Hex(A.SurfVKnots()[i]);
            Int(A.SurfVMults()[i]);
        }
        End();
        int aNb2d = A.Curves2dCount;
        A.Curves2dShape(out int d2, out int np2, out int nk2);
        Tag("curves2d");
        Int(aNb2d);
        Int(d2);
        Int(np2);
        Int(nk2);
        End();
        for (int k = 1; k <= aNb2d; k++)
        {
            NCollection_Array1<Point2D> aP2 = A.Curve2dPoles(k);
            Tag("c2d");
            for (int i = aP2.Lower(); i <= aP2.Upper(); i++)
            {
                Pnt2d(aP2[i]);
            }
            End();
        }
        if (aNb2d > 0)
        {
            Tag("c2dknots");
            for (int i = A.Curves2dKnots().Lower(); i <= A.Curves2dKnots().Upper(); i++)
            {
                Hex(A.Curves2dKnots()[i]);
                Int(A.Curves2dMults()[i]);
            }
            End();
        }
        A.TolReached(out double t3, out double t2);
        Tag("tolreached");
        Hex(t3);
        Hex(t2);
        for (int k = 1; k <= aNb2d; k++)
        {
            Hex(A.TolCurveOnSurf(k));
        }
        End();
    }

    private void RecAppSurface(AppFuncRoot F, Line L)
    {
        AppSurface approx = new(F, L.Point(1).Parameter(), L.Point(L.NbPoints()).Parameter(), 1.0e-4, 1.0e-5,
                                          1.0e-2, C1, 20, 5000);
        Tag("appsurface");
        Int(approx.IsDone ? 1 : 0);
        End();
        if (!approx.IsDone)
        {
            return;
        }
        RecApprox(approx);
        Tag("errors");
        Hex(approx.MaxErrorOnSurf());
        for (int k = 1; k <= approx.Curves2dCount; k++)
        {
            Hex(approx.Max2dError(k));
        }
        End();
        Tag("linepoints");
        Int(L.NbPoints());
        End();
    }

    private void RecAppSurf(AppFunction F, Line L)
    {
        AppSurf app = new(3, 8, 1.0e-4, 1.0e-5, 5);
        app.Perform(L, F);
        Tag("appsurf");
        Int(app.IsDone ? 1 : 0);
        End();
        if (app.IsDone)
        {
            RecApprox(app);
        }
    }

    private static ElSpine ElGuide(Curve3D C, double theF, double theL)
    {
        ElSpine E = new();
        E.SetCurve(C);
        E.FirstParameter(theF);
        E.LastParameter(theL);
        return E;
    }

    // A bounded copy of the walking surfaces of THE_PAIRS (Surf(theId)'s geometry).
    private static SurfaceAdaptorBase BSurf(int theId, double u1, double u2, double v1, double v2)
    {
        SurfaceAdaptor S = (SurfaceAdaptor)Surf(theId);
        return new SurfaceAdaptor(S.Surface()!, u1, u2, v1, v2);
    }

    private static LawsFunction EvolLaw(double theF, double theL)
    {
        Linear aLaw = new();
        aLaw.Set(theF, 0.8, theL, 1.4);
        Composite aComp = new(theF, theL, 1.0e-9);
        aComp.ChangeLaws().Append(aLaw);
        return aComp;
    }

    internal sealed record WalkCase(string Name, int S1, int S2, double[] B1, double[] B2, int G, double GF, double GL,
                                    double R, double[] X0, double Pdep, double Pmax, double Pmin, double Pcont, bool Evol);

    internal static readonly WalkCase[] THE_WALKS =
    [
        new("planeplane", 0, 1, [0.0, 8.0, 0.0, 5.0], [1.0, 10.0, -5.0, 0.0], 0, 0.0, 10.0, 1.0, [4.0, 1.0, 4.0, -1.0],
            4.0, 10.0, 0.0, 0.0, false),
        new("planeplanecont", 0, 1, [0.0, 8.0, 0.0, 5.0], [1.0, 10.0, -5.0, 0.0], 0, 0.0, 10.0, 1.0,
            [4.0, 1.0, 4.0, -1.0], 4.0, 6.0, 0.0, 7.5, false),
        new("planeplaneevol", 0, 1, [0.0, 8.0, 0.0, 5.0], [1.0, 10.0, -5.0, 0.0], 0, 0.0, 10.0, 1.0,
            [4.0, 1.0, 4.0, -1.0], 4.0, 10.0, 0.0, 0.0, true),
        new("planecyl", 0, 2, [-6.0, 6.0, -6.0, 6.0], [0.5, 2.5, 0.0, 5.0], 1, 0.0, 2.0 * Math.PI, 1.0,
            [4.0 * 0.5403023058681398, 4.0 * 0.8414709848078965, 1.0, 1.0], 1.0, 2.0, 0.0, 0.0, false),
        new("bspline", 6, 7, [0.0, 1.0, 0.0, 1.0], [0.0, 1.0, 0.0, 1.0], 2, 0.0, 3.0, 0.5, [0.83, 0.5, 0.5, 0.75], 1.5,
            3.0, 0.0, 0.0, false),
    ];

    private static Curve3D WalkGuideCurve(int theG)
    {
        return theG switch
        {
            0 => new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)),
            1 => new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5.0),
            _ => new LineCurve3D(new Point3D(1.5, 0, 0), new Direction3D(0, 1, 0)),
        };
    }

    private void RecWalk(WalkCase C, int choix)
    {
        SurfaceAdaptorBase S1 = BSurf(C.S1, C.B1[0], C.B1[1], C.B1[2], C.B1[3]);
        SurfaceAdaptorBase S2 = BSurf(C.S2, C.B2[0], C.B2[1], C.B2[2], C.B2[3]);
        TopolTool D1 = new(S1);
        TopolTool D2 = new(S2);
        ElSpine G = ElGuide(WalkGuideCurve(C.G), C.GF, C.GL);
        math_Vector X0 = new(1, 4);
        for (int i = 1; i <= 4; i++)
        {
            X0[i] = C.X0[i - 1];
        }
        BlendFunction F;
        FuncInv FInv;
        if (C.Evol)
        {
            LawsFunction aLaw = EvolLaw(C.GF, C.GL);
            EvolRad anEvol = new(S1, S2, G, aLaw);
            anEvol.Set(choix);
            EvolRadInv anEvolInv = new(S1, S2, G, aLaw);
            anEvolInv.Set(choix);
            F = anEvol;
            FInv = anEvolInv;
        }
        else
        {
            ConstRad aConst = new(S1, S2, G);
            aConst.Set(C.R, choix);
            ConstRadInv aConstInv = new(S1, S2, G);
            aConstInv.Set(C.R, choix);
            F = aConst;
            FInv = aConstInv;
        }
        Walking W = new(S1, S2, D1, D2, G);
        W.Check2d(false);
        W.Perform(F, FInv, C.Pdep, C.Pmax, 1.0, 1.0e-4, 1.0e-5, X0, 0.01, true);
        Tag("perform");
        Int(W.IsDone ? 1 : 0);
        End();
        if (!W.IsDone)
        {
            return;
        }
        if (C.Pcont != 0.0)
        {
            bool isOk = W.Continu(F, FInv, C.Pcont);
            Tag("continu");
            Int(isOk ? 1 : 0);
            End();
        }
        bool isComplete = W.Complete(F, FInv, C.Pmin);
        Tag("complete");
        Int(isComplete ? 1 : 0);
        Int(W.TwistOnS1() ? 1 : 0);
        Int(W.TwistOnS2() ? 1 : 0);
        End();
        Line aLin = W.Line();
        RecLine(aLin);
        RecFuncState(F);
        AppFuncRoot AF = new AppFunc(aLin, F, 1.0e-4, 1.0e-5);
        RecAppSurface(AF, aLin);
        RecAppSurf(F, aLin);
    }

    // The first sections: from inside, and from outside the first face, moved onto its restriction (then walked).
    private void RecFirstSection(int choix)
    {
        WalkCase C = THE_WALKS[0];
        SurfaceAdaptorBase S1 = BSurf(C.S1, C.B1[0], C.B1[1], C.B1[2], C.B1[3]);
        SurfaceAdaptorBase S2 = BSurf(C.S2, C.B2[0], C.B2[1], C.B2[2], C.B2[3]);
        TopolTool D1 = new(S1);
        TopolTool D2 = new(S2);
        ElSpine G = ElGuide(WalkGuideCurve(C.G), C.GF, C.GL);
        ConstRad F = new(S1, S2, G);
        F.Set(C.R, choix);
        ConstRadInv FInv = new(S1, S2, G);
        FInv.Set(C.R, choix);

        Walking W1 = new(S1, S2, D1, D2, G);
        math_Vector X0 = new(1, 4);
        X0[1] = 3.0;
        X0[2] = 1.2;
        X0[3] = 3.0;
        X0[4] = -0.8;
        bool isOk1 = W1.PerformFirstSection(F, 3.0, X0, 1.0e-4, 1.0e-5, out Containment aPos1, out Containment aPos2);
        Tag("firstsection");
        Int(isOk1 ? 1 : 0);
        Int((int)aPos1);
        Int((int)aPos2);
        End();
        Vector("pardep", X0);

        Walking W2 = new(S1, S2, D1, D2, G);
        W2.Check2d(false);
        X0[1] = 8.5;
        X0[2] = 1.0;
        X0[3] = 8.5;
        X0[4] = -1.0;
        double aPsol = 0.0;
        math_Vector aParSol = new(1, 4, 0.0);
        bool isOk2 = W2.PerformFirstSection(F, FInv, 8.5, 0.0, X0, 1.0e-4, 1.0e-5, true, true, ref aPsol, aParSol);
        Tag("firstsectionrst");
        Int(isOk2 ? 1 : 0);
        Hex(aPsol);
        End();
        Vector("parsol", aParSol);
        if (!isOk2)
        {
            return;
        }
        W2.Perform(F, FInv, aPsol, 0.0, 1.0, 1.0e-4, 1.0e-5, aParSol, 0.01, false);
        Tag("perform");
        Int(W2.IsDone ? 1 : 0);
        End();
        if (W2.IsDone)
        {
            RecLine(W2.Line());
        }
    }

    private void RecCSWalk(int choix)
    {
        SurfaceAdaptorBase S = BSurf(0, 0.0, 8.0, -3.0, 3.0);
        TopolTool D = new(S);
        CurveAdaptorBase C = Guide(4);
        ElSpine G = ElGuide(WalkGuideCurve(0), 0.0, 10.0);
        CSConstRad F = new(S, C, G);
        F.Set(1.0, choix);
        math_Vector X3 = new(1, 3);
        X3[1] = 3.0;
        X3[2] = -0.5;
        X3[3] = 3.0;
        CSWalking W = new(C, S, D);
        W.Perform(F, 3.0, 9.0, 1.0, 1.0e-4, 1.0e-5, X3, 0.01, true);
        Tag("perform");
        Int(W.IsDone ? 1 : 0);
        End();
        if (!W.IsDone)
        {
            return;
        }
        bool isComplete = W.Complete(F, 0.0);
        Tag("complete");
        Int(isComplete ? 1 : 0);
        End();
        Line aLin = W.Line();
        RecLine(aLin);
        RecFuncState(F);
        RecAppSurf(F, aLin);
    }

    // A surface (the floor) and a restriction (a line on the wall at z = 1), or two restrictions (lines on the floor
    // at y = 1 and on the wall at z = 1).
    internal sealed record RstCase(string Name, double[] B1, double[] R1, double[] R2, double R, bool Evol);

    internal static readonly RstCase[] THE_SURFRSTS =
    [
        new("surfrst", [0.0, 10.0, -1.0, 5.0], [0.0, 0.0], [0.5, 9.0], 1.2, false),
        new("surfrstface", [1.0, 8.0, -1.0, 5.0], [0.0, 0.0], [0.0, 10.0], 1.2, false),
        new("surfrstevol", [0.0, 10.0, -1.0, 5.0], [0.0, 0.0], [0.5, 9.0], 0.0, true),
    ];

    internal static readonly RstCase[] THE_RSTRSTS =
    [
        new("rstrst", [0.0, 10.0, -1.0, 5.0], [0.5, 8.0], [1.0, 9.0], 1.0, false),
        new("rstrstevol", [0.0, 10.0, -1.0, 5.0], [0.5, 8.0], [1.0, 9.0], 0.0, true),
    ];

    private void RecSurfRstWalk(RstCase C, int choix, int bigchoix)
    {
        SurfaceAdaptorBase S1 = BSurf(0, C.B1[0], C.B1[1], C.B1[2], C.B1[3]);
        SurfaceAdaptorBase S2 = BSurf(1, 0.0, 10.0, -5.0, 0.0);
        TopolTool D1 = new(S1);
        TopolTool D2 = new(S2);
        ElSpine G = ElGuide(WalkGuideCurve(0), 0.0, 10.0);
        Curve2DAdaptorBase Rst = new CurveAdaptor2D(new LineCurve2D(new Point2D(0.0, -1.0), new Direction2D(1.0, 0.0)),
                                                        C.R2[0], C.R2[1]);
        CurveOnSurface HC = new();
        HC.Load(Rst, S2);
        math_Vector X0 = new(1, 3);
        X0[1] = 4.0;
        X0[2] = 1.0;
        X0[3] = 4.0;

        SurfRstFunction F;
        SurfCurvFuncInv FInvC;
        SurfPointFuncInv FInvP;
        FuncInv FInv;
        if (C.Evol)
        {
            LawsFunction aLaw = EvolLaw(0.0, 10.0);
            SurfRstEvolRad anEvol = new(S1, S2, Rst, G, aLaw);
            anEvol.Set(S2, Rst);
            anEvol.Set(choix);
            SurfCurvEvolRadInv anEvolC = new(S1, HC, G, aLaw);
            anEvolC.Set(choix);
            SurfPointEvolRadInv anEvolP = new(S1, G, aLaw);
            anEvolP.Set(choix);
            EvolRadInv anEvolI = new(S1, S2, G, aLaw);
            anEvolI.Set(false, Rst);
            anEvolI.Set(bigchoix);
            F = anEvol;
            FInvC = anEvolC;
            FInvP = anEvolP;
            FInv = anEvolI;
        }
        else
        {
            SurfRstConstRad aConst = new(S1, S2, Rst, G);
            aConst.Set(S2, Rst);
            aConst.Set(C.R, choix);
            SurfCurvConstRadInv aConstC = new(S1, HC, G);
            aConstC.Set(C.R, choix);
            SurfPointConstRadInv aConstP = new(S1, G);
            aConstP.Set(C.R, choix);
            ConstRadInv aConstI = new(S1, S2, G);
            aConstI.Set(false, Rst);
            aConstI.Set(C.R, bigchoix);
            F = aConst;
            FInvC = aConstC;
            FInvP = aConstP;
            FInv = aConstI;
        }
        SurfRstLineBuilder W = new(S1, D1, S2, Rst, D2);
        W.Perform(F, FInv, FInvP, FInvC, 4.0, 9.5, 1.0, 1.0e-4, 1.0e-5, 1.0e-5, X0, 0.01, true);
        Tag("perform");
        Int(W.IsDone ? 1 : 0);
        End();
        if (!W.IsDone)
        {
            return;
        }
        bool isComplete = W.Complete(F, FInv, FInvP, FInvC, 0.0);
        Tag("complete");
        Int(isComplete ? 1 : 0);
        Int(W.DecrochStart() ? 1 : 0);
        Int(W.DecrochEnd() ? 1 : 0);
        End();
        Line aLin = W.Line();
        RecLine(aLin);
        RecFuncState(F);
        AppFuncRoot AF = new AppFuncRst(aLin, F, 1.0e-4, 1.0e-5);
        RecAppSurface(AF, aLin);
    }

    private void RecRstRstWalk(RstCase C, int choix)
    {
        SurfaceAdaptorBase S1 = BSurf(0, C.B1[0], C.B1[1], C.B1[2], C.B1[3]);
        SurfaceAdaptorBase S2 = BSurf(1, 0.0, 10.0, -5.0, 0.0);
        TopolTool D1 = new(S1);
        TopolTool D2 = new(S2);
        ElSpine G = ElGuide(WalkGuideCurve(0), 0.0, 10.0);
        Curve2DAdaptorBase Rst1 = new CurveAdaptor2D(
            new LineCurve2D(new Point2D(0.0, 1.0), new Direction2D(1.0, 0.0)), C.R1[0], C.R1[1]);
        Curve2DAdaptorBase Rst2 = new CurveAdaptor2D(
            new LineCurve2D(new Point2D(0.0, -1.0), new Direction2D(1.0, 0.0)), C.R2[0], C.R2[1]);
        CurveOnSurface HC1 = new();
        HC1.Load(Rst1, S1);
        CurveOnSurface HC2 = new();
        HC2.Load(Rst2, S2);
        math_Vector X0 = new(1, 2);
        X0[1] = 4.0;
        X0[2] = 4.0;

        RstRstFunction F;
        SurfCurvFuncInv FInv1;
        SurfCurvFuncInv FInv2;
        CurvPointRadInv FInvP1 = new(G, HC2), FInvP2 = new(G, HC1);
        FInvP1.Set(choix);
        FInvP2.Set(choix);
        if (C.Evol)
        {
            LawsFunction aLaw = EvolLaw(0.0, 10.0);
            RstRstEvolRad anEvol = new(S1, Rst1, S2, Rst2, G, aLaw);
            anEvol.Set(S1, Rst1, S2, Rst2);
            anEvol.Set(choix);
            SurfCurvEvolRadInv anEvol1 = new(S1, HC2, G, aLaw);
            anEvol1.Set(Rst1);
            anEvol1.Set(1);
            SurfCurvEvolRadInv anEvol2 = new(S2, HC1, G, aLaw);
            anEvol2.Set(Rst2);
            anEvol2.Set(2);
            F = anEvol;
            FInv1 = anEvol1;
            FInv2 = anEvol2;
        }
        else
        {
            RstRstConstRad aConst = new(S1, Rst1, S2, Rst2, G);
            aConst.Set(S1, Rst1, S2, Rst2);
            aConst.Set(C.R, choix);
            SurfCurvConstRadInv aConst1 = new(S1, HC2, G);
            aConst1.Set(Rst1);
            aConst1.Set(C.R, 1);
            SurfCurvConstRadInv aConst2 = new(S2, HC1, G);
            aConst2.Set(Rst2);
            aConst2.Set(C.R, 2);
            F = aConst;
            FInv1 = aConst1;
            FInv2 = aConst2;
        }
        RstRstLineBuilder W = new(S1, Rst1, D1, S2, Rst2, D2);
        W.Perform(F, FInv1, FInvP1, FInv2, FInvP2, 4.0, 9.5, 1.0, 1.0e-4, 1.0e-5, X0, 0.01, true);
        Tag("perform");
        Int(W.IsDone ? 1 : 0);
        End();
        if (!W.IsDone)
        {
            return;
        }
        bool isComplete = W.Complete(F, FInv1, FInvP1, FInv2, FInvP2, 0.0);
        Tag("complete");
        Int(isComplete ? 1 : 0);
        Int(W.Decroch1Start() ? 1 : 0);
        Int(W.Decroch1End() ? 1 : 0);
        Int(W.Decroch2Start() ? 1 : 0);
        Int(W.Decroch2End() ? 1 : 0);
        End();
        Line aLin = W.Line();
        RecLine(aLin);
        RecFuncState(F);
        AppFuncRoot AF = new AppFuncRstRst(aLin, F, 1.0e-4, 1.0e-5);
        RecAppSurface(AF, aLin);
    }

    private void Guarded(Action theCase)
    {
        try
        {
            theCase();
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            Fail("exception");
        }
    }

    internal void RecordWalking()
    {
        foreach (WalkCase aWalk in THE_WALKS)
        {
            for (int choix = 1; choix <= 8; choix++)
            {
                Line($"case walk {aWalk.Name} {choix}");
                Guarded(() => RecWalk(aWalk, choix));
            }
        }
        for (int choix = 1; choix <= 8; choix++)
        {
            Line($"case walk firstsection {choix}");
            Guarded(() => RecFirstSection(choix));
        }
        for (int choix = 1; choix <= 8; choix++)
        {
            Line($"case walk cs {choix}");
            Guarded(() => RecCSWalk(choix));
        }
        foreach (RstCase aCase in THE_SURFRSTS)
        {
            // As ChFi3d_FilBuilder: the restriction's function and inverses take a "small" choice (1 or 3, plus 1 for
            // an even choice), the inverse on the restriction the surfaces' choice.
            for (int big = 1; big <= 8; big++)
            {
                for (int rev = 0; rev <= 1; rev++)
                {
                    int choix = (rev == 1 ? 3 : 1) + (big % 2 == 0 ? 1 : 0);
                    int aBig = big;
                    Line($"case walk {aCase.Name} {choix} {big}");
                    Guarded(() => RecSurfRstWalk(aCase, choix, aBig));
                }
            }
        }
        foreach (RstCase aCase in THE_RSTRSTS)
        {
            for (int choix = 1; choix <= 4; choix++)
            {
                Line($"case walk {aCase.Name} {choix}");
                Guarded(() => RecRstRstWalk(aCase, choix));
            }
        }
    }
}
