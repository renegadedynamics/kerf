// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The file-static functions of IntPatch_ImpPrmIntersection.cxx (see IntPatch_ImpPrmIntersection.cs).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Occt.IntPatch_IType;
using static Occt.IntPatch_SpecPntType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_State;

namespace Occt;

public partial class IntPatch_ImpPrmIntersection
{
    private static IntPatch_SpecPntType IsSeamOrPole(Adaptor3d_Surface theQSurf, IntSurf_LineOn2S theLine,
                                                     bool IsReversed, int theRefIndex, double theTol3D,
                                                     double theDeltaMax)
    {
        if ((theRefIndex < 1) || (theRefIndex >= theLine.NbPoints()))
        {
            return IntPatch_SPntNone;
        }

        // Parameters on Quadric and on parametric for reference point
        double aUQRef, aVQRef, aUPRef, aVPRef;
        double aUQNext, aVQNext, aUPNext, aVPNext;

        Point3D aP3d = theLine.Value(theRefIndex + 1).Value();

        if (IsReversed)
        {
            theLine.Value(theRefIndex).Parameters(out aUPRef, out aVPRef, out aUQRef, out aVQRef);
            theLine.Value(theRefIndex + 1).Parameters(out aUPNext, out aVPNext, out aUQNext, out aVQNext);
        }
        else
        {
            theLine.Value(theRefIndex).Parameters(out aUQRef, out aVQRef, out aUPRef, out aVPRef);
            theLine.Value(theRefIndex + 1).Parameters(out aUQNext, out aVQNext, out aUPNext, out aVPNext);
        }

        SurfaceKind aType = theQSurf.GetType();

        if ((aType == SurfaceKind.Cone) && (theQSurf.Cone().Apex().SquareDistance(aP3d) < theTol3D * theTol3D))
        {
            return IntPatch_SPntPoleSeamU;
        }
        else if (aType == SurfaceKind.Sphere)
        {
            double aSqTol = theTol3D * theTol3D;
            Point3D aP = global::Kerf.Engine.Geometry.ElSLib.Value(0.0, M_PI_2, theQSurf.Sphere());
            if (aP.SquareDistance(aP3d) < aSqTol)
            {
                return IntPatch_SPntPoleSeamU;
            }

            aP = global::Kerf.Engine.Geometry.ElSLib.Value(0.0, -M_PI_2, theQSurf.Sphere());
            if (aP.SquareDistance(aP3d) < aSqTol)
            {
                return IntPatch_SPntPoleSeamU;
            }
        }

        double aDeltaU = Math.Abs(aUQRef - aUQNext);

        if ((aType != SurfaceKind.Torus) && (aDeltaU < theDeltaMax))
        {
            return IntPatch_SPntNone;
        }

        switch (aType)
        {
            case SurfaceKind.Cylinder:
                return IntPatch_SPntSeamU;

            case SurfaceKind.Torus:
            {
                double aDeltaV = Math.Abs(aVQRef - aVQNext);

                if ((aDeltaU >= theDeltaMax) && (aDeltaV >= theDeltaMax))
                {
                    return IntPatch_SPntSeamUV;
                }

                if (aDeltaU >= theDeltaMax)
                {
                    return IntPatch_SPntSeamU;
                }

                if (aDeltaV >= theDeltaMax)
                {
                    return IntPatch_SPntSeamV;
                }
            }

            break;
            case SurfaceKind.Sphere:
            case SurfaceKind.Cone:
                return IntPatch_SPntPoleSeamU;
            default:
                break;
        }

        return IntPatch_SPntNone;
    }

    private static void ComputeTangency(IntPatch_TheSOnBounds solrst, NCollection_Sequence<IntSurf_PathPoint> seqpdep,
                                        Adaptor3d_TopolTool Domain, IntPatch_TheSurfFunction Func,
                                        Adaptor3d_Surface PSurf, NCollection_Array1<int> Destination)
    {
        int i, k, NbPoints, seqlength;
        double theparam, test;
        bool fairpt, ispassing;
        TopAbs_Orientation arcorien, vtxorien;
        Adaptor2d_Curve2d thearc;
        // OCCT keeps the vertex handle across the points: in the complex-transition branch of an INTERNAL or EXTERNAL
        // arc it is the one an earlier point left (null if none), as here.
        Adaptor3d_HVertex? vtx = null, vtxbis;
        IntPatch_ThePathPointOfTheSOnBounds PStart;
        IntSurf_PathPoint PPoint = new();
        Vector3D vectg;
        Direction2D dirtg;
        Vector3D d1u, d1v, v1, v2 = new();
        Point2D p2d;
        Vector2D d2d;
        //
        math_Vector X = new(1, 2);
        math_Vector F = new(1, 1);
        Matrix D = new(1, 1, 1, 2);
        //
        seqlength = 0;
        NbPoints = solrst.NbPoints();
        for (i = 1; i <= NbPoints; i++)
        {
            if (Destination.Value(i) == 0)
            {
                PStart = solrst.Point(i);
                thearc = PStart.Arc();
                theparam = PStart.Parameter();
                arcorien = Domain.Orientation(thearc);
                ispassing = (arcorien == TopAbs_INTERNAL || arcorien == TopAbs_EXTERNAL);

                thearc.D0(theparam, out p2d);
                X[1] = p2d.X();
                X[2] = p2d.Y();
                PPoint.SetValue(PStart.Value(), X[1], X[2]);

                Func.Values(X, F, D);
                if (Func.IsTangent())
                {
                    PPoint.SetTangency(true);
                    Destination.SetValue(i, seqlength + 1);
                    if (!PStart.IsNew())
                    {
                        vtx = PStart.Vertex();
                        for (k = i + 1; k <= NbPoints; k++)
                        {
                            if (Destination.Value(k) == 0)
                            {
                                PStart = solrst.Point(k);
                                if (!PStart.IsNew())
                                {
                                    vtxbis = PStart.Vertex();
                                    if (Domain.Identical(vtx, vtxbis))
                                    {
                                        thearc = PStart.Arc();
                                        theparam = PStart.Parameter();
                                        arcorien = Domain.Orientation(thearc);
                                        ispassing = ispassing
                                                    && (arcorien == TopAbs_INTERNAL || arcorien == TopAbs_EXTERNAL);

                                        thearc.D0(theparam, out p2d);
                                        PPoint.AddUV(p2d.X(), p2d.Y());
                                        Destination.SetValue(k, seqlength + 1);
                                    }
                                }
                            }
                        }
                    }
                    PPoint.SetPassing(ispassing);
                    seqpdep.Append(PPoint);
                    seqlength++;
                }
                else
                { // on a un point de depart potentiel

                    vectg = Func.Direction3d();
                    dirtg = Func.Direction2d();

                    PSurf.D1(X[1], X[2], out _, out d1u, out d1v);
                    thearc.D1(theparam, out p2d, out d2d);
                    v2.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);
                    v1 = d1u.Crossed(d1v);

                    test = vectg.Dot(v1.Crossed(v2));
                    if (PStart.IsNew())
                    {
                        if ((test < 0.0 && arcorien == TopAbs_FORWARD) || (test > 0.0 && arcorien == TopAbs_REVERSED))
                        {
                            vectg.Reverse();
                            dirtg.Reverse();
                        }
                        PPoint.SetDirections(vectg, dirtg);
                        PPoint.SetPassing(ispassing);
                        Destination.SetValue(i, seqlength + 1);
                        seqpdep.Append(PPoint);
                        seqlength++;
                    }
                    else
                    { // traiter la transition complexe
                        Direction3D bidnorm = new(1.0, 1.0, 1.0);
                        double tole = 1.0e-8;
                        TopAbs_Orientation LocTrans;
                        TopTrans_CurveTransition comptrans = new();
                        comptrans.Reset(new Direction3D(vectg), bidnorm, 0.0);
                        if (arcorien == TopAbs_FORWARD || arcorien == TopAbs_REVERSED)
                        {
                            // pour essai

                            vtx = PStart.Vertex();
                            vtxorien = Domain.Orientation(vtx);
                            if (Math.Abs(test) <= tole)
                            {
                                LocTrans = TopAbs_EXTERNAL; // et pourquoi pas INTERNAL
                            }
                            else
                            {
                                if (((test > 0.0) && arcorien == TopAbs_FORWARD)
                                    || ((test < 0.0) && arcorien == TopAbs_REVERSED))
                                {
                                    LocTrans = TopAbs_FORWARD;
                                }
                                else
                                {
                                    LocTrans = TopAbs_REVERSED;
                                }
                                if (arcorien == TopAbs_REVERSED)
                                {
                                    v2.Reverse();
                                }
                            }

                            comptrans.Compare(tole, new Direction3D(v2), bidnorm, 0.0, LocTrans, vtxorien);
                        }
                        Destination.SetValue(i, seqlength + 1);
                        for (k = i + 1; k <= NbPoints; k++)
                        {
                            if (Destination.Value(k) == 0)
                            {
                                PStart = solrst.Point(k);
                                if (!PStart.IsNew())
                                {
                                    vtxbis = PStart.Vertex();
                                    if (Domain.Identical(vtx!, vtxbis))
                                    {
                                        thearc = PStart.Arc();
                                        theparam = PStart.Parameter();
                                        arcorien = Domain.Orientation(thearc);

                                        PPoint.AddUV(X[1], X[2]);

                                        thearc.D1(theparam, out p2d, out d2d);
                                        PPoint.AddUV(p2d.X(), p2d.Y());

                                        if (arcorien == TopAbs_FORWARD || arcorien == TopAbs_REVERSED)
                                        {
                                            ispassing = false;
                                            v2.SetLinearForm(d2d.X(), d1u, d2d.Y(), d1v);

                                            test = vectg.Dot(v1.Crossed(v2));
                                            vtxorien = Domain.Orientation(PStart.Vertex());
                                            if (Math.Abs(test) <= tole)
                                            {
                                                LocTrans = TopAbs_EXTERNAL; // et pourquoi pas INTERNAL
                                            }
                                            else
                                            {
                                                if (((test > 0.0) && arcorien == TopAbs_FORWARD)
                                                    || ((test < 0.0) && arcorien == TopAbs_REVERSED))
                                                {
                                                    LocTrans = TopAbs_FORWARD;
                                                }
                                                else
                                                {
                                                    LocTrans = TopAbs_REVERSED;
                                                }
                                                if (arcorien == TopAbs_REVERSED)
                                                {
                                                    v2.Reverse();
                                                }
                                            }

                                            comptrans.Compare(tole, new Direction3D(v2), bidnorm, 0.0, LocTrans, vtxorien);
                                        }
                                        Destination.SetValue(k, seqlength + 1);
                                    }
                                }
                            }
                        }
                        fairpt = true;
                        if (!ispassing)
                        {
                            TopAbs_State Before = comptrans.StateBefore();
                            TopAbs_State After = comptrans.StateAfter();
                            if ((Before == TopAbs_UNKNOWN) || (After == TopAbs_UNKNOWN))
                            {
                                fairpt = false;
                            }
                            else if (Before == TopAbs_IN)
                            {
                                if (After == TopAbs_IN)
                                {
                                    ispassing = true;
                                }
                                else
                                {
                                    vectg.Reverse();
                                    dirtg.Reverse();
                                }
                            }
                            else
                            {
                                if (After != TopAbs_IN)
                                {
                                    fairpt = false;
                                }
                            }
                        }
                        if (fairpt)
                        {
                            PPoint.SetDirections(vectg, dirtg);
                            PPoint.SetPassing(ispassing);
                            seqpdep.Append(PPoint);
                            seqlength++;
                        }
                        else
                        { // il faut remettre en "ordre" si on ne garde pas le point.
                            for (k = i; k <= NbPoints; k++)
                            {
                                if (Destination.Value(k) == seqlength + 1)
                                {
                                    Destination.SetValue(k, -Destination.Value(k));
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    private static void Recadre(bool _, SurfaceKind typeS1, SurfaceKind typeS2, ref IntPatch_Point pt,
                                IntPatch_TheIWLineOfTheIWalking iwline, int Param, double U1, double V1, double U2,
                                double V2)
    {
        iwline.Line().Value(Param).Parameters(out double U1p, out double V1p, out double U2p, out double V2p);
        switch (typeS1)
        {
            case SurfaceKind.Torus:
                while (V1 < (V1p - 1.5 * M_PI))
                {
                    V1 += M_PI + M_PI;
                }
                while (V1 > (V1p + 1.5 * M_PI))
                {
                    V1 -= M_PI + M_PI;
                }
                goto case SurfaceKind.Cylinder;
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
                while (U1 < (U1p - 1.5 * M_PI))
                {
                    U1 += M_PI + M_PI;
                }
                while (U1 > (U1p + 1.5 * M_PI))
                {
                    U1 -= M_PI + M_PI;
                }
                break;
            default:
                break;
        }
        switch (typeS2)
        {
            case SurfaceKind.Torus:
                while (V2 < (V2p - 1.5 * M_PI))
                {
                    V2 += M_PI + M_PI;
                }
                while (V2 > (V2p + 1.5 * M_PI))
                {
                    V2 -= M_PI + M_PI;
                }
                goto case SurfaceKind.Cylinder;
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
                while (U2 < (U2p - 1.5 * M_PI))
                {
                    U2 += M_PI + M_PI;
                }
                while (U2 > (U2p + 1.5 * M_PI))
                {
                    U2 -= M_PI + M_PI;
                }
                break;
            default:
                break;
        }
        pt.SetParameters(U1, V1, U2, V2);
    }

    private static double GetLocalStep(Adaptor3d_Surface theSurf, double theStep)
    {
        double aLocalStep = theStep;
        if (theSurf.UContinuity() > C0 && theSurf.VContinuity() > C0)
        {
            SurfaceKind aSType = theSurf.GetType();

            if (aSType == BezierSurface || aSType == BSplineSurface)
            {
                double aMinRes;
                int aMaxDeg;
                double aLimRes = 1.0e-10;

                aMinRes = Math.Min(theSurf.UResolution(global::Kerf.Engine.Foundation.Precision.Confusion()),
                                   theSurf.VResolution(global::Kerf.Engine.Foundation.Precision.Confusion()));
                aMaxDeg = Math.Max(theSurf.UDegree(), theSurf.VDegree());
                if (aMinRes < aLimRes && aMaxDeg > 3)
                {
                    aLocalStep = 0.0001;
                }
            }
        }
        if (theSurf.UContinuity() == C0)
        {
            int aNbInt = theSurf.NbUIntervals(C1);
            if (aNbInt > 1)
            {
                NCollection_Array1<double> anInts = new(1, aNbInt + 1);
                theSurf.UIntervals(anInts, C1);
                double aMinInt = global::Kerf.Engine.Foundation.Precision.Infinite();
                for (int i = 1; i <= aNbInt; ++i)
                {
                    aMinInt = Math.Min(aMinInt, anInts.Value(i + 1) - anInts.Value(i));
                }

                aMinInt /= theSurf.LastUParameter() - theSurf.FirstUParameter();
                if (aMinInt < 0.002)
                {
                    aLocalStep = 0.0001;
                }
            }
        }

        if (theSurf.VContinuity() == C0)
        {
            int aNbInt = theSurf.NbVIntervals(C1);
            if (aNbInt > 1)
            {
                NCollection_Array1<double> anInts = new(1, aNbInt + 1);
                theSurf.VIntervals(anInts, C1);
                double aMinInt = global::Kerf.Engine.Foundation.Precision.Infinite();
                for (int i = 1; i <= aNbInt; ++i)
                {
                    aMinInt = Math.Min(aMinInt, anInts.Value(i + 1) - anInts.Value(i));
                }

                aMinInt /= theSurf.LastVParameter() - theSurf.FirstVParameter();
                if (aMinInt < 0.002)
                {
                    aLocalStep = 0.0001;
                }
            }
        }

        aLocalStep = Math.Min(theStep, aLocalStep);
        return aLocalStep;
    }

    // correct U parameter of the start point of line on Quadric
    // (change 0->2PI or vs, if necessary)
    private static double AdjustUFirst(double U1, double U2)
    {
        double u = U1;

        // case: no adjustment
        if (U1 > 0.0 && U1 < (2.0 * M_PI))
        {
            return u;
        }

        // case: near '0'
        if (U1 == 0.0 || Math.Abs(U1) <= 1.0e-9)
        {
            if (U2 > 0.0 && U2 < (2.0 * M_PI))
            {
                u = (U2 < ((2.0 * M_PI) - U2)) ? 0.0 : (2.0 * M_PI);
            }
            else
            {
                double uu = U2;
                if (U2 > (2.0 * M_PI))
                {
                    while (uu > (2.0 * M_PI))
                    {
                        uu -= (2.0 * M_PI);
                    }
                }
                else
                {
                    while (uu < 0.0)
                    {
                        uu += (2.0 * M_PI);
                    }
                }

                u = (uu < ((2.0 * M_PI) - uu)) ? 0.0 : (2.0 * M_PI);
            }
        }
        // case: near '2PI'
        else if (U1 == (2.0 * M_PI) || Math.Abs((2.0 * M_PI) - Math.Abs(U1)) <= 1.0e-9)
        {
            if (U2 > 0.0 && U2 < (2.0 * M_PI))
            {
                u = (U2 < ((2.0 * M_PI) - U2)) ? 0.0 : (2.0 * M_PI);
            }
            else
            {
                double uu = U2;
                if (U2 > (2.0 * M_PI))
                {
                    while (uu > (2.0 * M_PI))
                    {
                        uu -= (2.0 * M_PI);
                    }
                }
                else
                {
                    while (uu < 0.0)
                    {
                        uu += (2.0 * M_PI);
                    }
                }

                u = (uu < ((2.0 * M_PI) - uu)) ? 0.0 : (2.0 * M_PI);
            }
        }
        // case: '<0. || >2PI'
        else
        {
            if (U1 < 0.0)
            {
                while (u < 0.0)
                {
                    u += 2.0 * M_PI;
                }
            }
            if (U1 > (2.0 * M_PI))
            {
                while (u > (2.0 * M_PI))
                {
                    u -= (2.0 * M_PI);
                }
            }
        }

        return u;
    }

    // collect vertices, reject equals
    private static IntSurf_LineOn2S GetVertices(IntPatch_PointLine thePLine, double TOL3D, double TOL2D)
    {
        IntSurf_LineOn2S vertices = new();

        double U1, U2, V1, V2;
        int i, k;
        int NbVrt = thePLine.NbVertex();

        NCollection_Array1<int> anVrts = new(1, NbVrt);
        anVrts.Init(0);

        // check equal vertices
        for (i = 1; i <= NbVrt; i++)
        {
            if (anVrts.Value(i) == -1)
            {
                continue;
            }

            IntPatch_Point Pi = thePLine.Vertex(i);

            for (k = (i + 1); k <= NbVrt; k++)
            {
                if (anVrts.Value(k) == -1)
                {
                    continue;
                }

                IntPatch_Point Pk = thePLine.Vertex(k);

                if (Pi.Value().Distance(Pk.Value()) <= TOL3D)
                {
                    // suggest the points are equal;
                    // test 2d parameters on surface
                    bool sameU1 = false;
                    bool sameV1 = false;
                    bool sameU2 = false;
                    bool sameV2 = false;

                    Pi.ParametersOnS1(out U1, out V1);
                    Pk.ParametersOnS1(out U2, out V2);
                    if (Math.Abs(U1 - U2) <= TOL2D)
                    {
                        sameU1 = true;
                    }
                    if (Math.Abs(V1 - V2) <= TOL2D)
                    {
                        sameV1 = true;
                    }

                    Pi.ParametersOnS2(out U1, out V1);
                    Pk.ParametersOnS2(out U2, out V2);
                    if (Math.Abs(U1 - U2) <= TOL2D)
                    {
                        sameU2 = true;
                    }
                    if (Math.Abs(V1 - V2) <= TOL2D)
                    {
                        sameV2 = true;
                    }

                    if ((sameU1 && sameV1) && (sameU2 && sameV2))
                    {
                        anVrts.SetValue(k, -1);
                    }
                }
            }
        }

        // copy further processed vertices
        for (i = 1; i <= NbVrt; i++)
        {
            if (anVrts.Value(i) == -1)
            {
                continue;
            }
            vertices.Add(thePLine.Vertex(i).PntOn2S());
        }
        return vertices;
    }

    private static void SearchVertices(IntSurf_LineOn2S Line, IntSurf_LineOn2S Vertices, NCollection_Array1<int> PTypes)
    {
        int nbp = Line.NbPoints(), nbv = Vertices.NbPoints();
        for (int ip = 1; ip <= nbp; ip++)
        {
            IntSurf_PntOn2S aP = Line.Value(ip);
            int type = 0;
            for (int iv = 1; iv <= nbv; iv++)
            {
                IntSurf_PntOn2S aV = Vertices.Value(iv);
                if (aP.IsSame(aV, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.PConfusion()))
                {
                    type = iv;
                    break;
                }
            }
            PTypes.SetValue(ip, type);
        }
    }

    private static bool IsSeamParameter(double U, double TOL2D) =>
        (Math.Abs(U) <= TOL2D || Math.Abs(2.0 * M_PI - U) <= TOL2D);

    private static double AdjustU(double U)
    {
        double u = U, DBLPI = 2.0 * M_PI;
        if (u < 0.0 || u > DBLPI)
        {
            if (u < 0.0)
            {
                while (u < 0.0)
                {
                    u += DBLPI;
                }
            }
            else
            {
                while (u > DBLPI)
                {
                    u -= DBLPI;
                }
            }
        }
        return u;
    }

    private static void Correct2DBounds(double UF, double UL, double VF, double VL, double TOL2D, ref double U,
                                        ref double V)
    {
        double Eps = 1.0e-16;
        double dUF = Math.Abs(U - UF);
        double dUL = Math.Abs(U - UL);
        double dVF = Math.Abs(V - VF);
        double dVL = Math.Abs(V - VL);
        if (dUF <= TOL2D && dUF > Eps)
        {
            U = UF;
        }
        if (dUL <= TOL2D && dUL > Eps)
        {
            U = UL;
        }
        if (dVF <= TOL2D && dVF > Eps)
        {
            V = VF;
        }
        if (dVL <= TOL2D && dVL > Eps)
        {
            V = VL;
        }
    }

    private static void AdjustLine(IntSurf_LineOn2S Line, bool IsReversed, Adaptor3d_Surface QSurf, double TOL2D)
    {
        double VF = QSurf.FirstVParameter();
        double VL = QSurf.LastVParameter();
        double UF = QSurf.FirstUParameter();
        double UL = QSurf.LastUParameter();

        int nbp = Line.NbPoints();
        double U, V;
        for (int ip = 1; ip <= nbp; ip++)
        {
            if (IsReversed)
            {
                Line.Value(ip).ParametersOnS2(out U, out V);
                U = AdjustU(U);
                Correct2DBounds(UF, UL, VF, VL, TOL2D, ref U, ref V);
                Line.SetUV(ip, false, U, V);
            }
            else
            {
                Line.Value(ip).ParametersOnS1(out U, out V);
                U = AdjustU(U);
                Correct2DBounds(UF, UL, VF, VL, TOL2D, ref U, ref V);
                Line.SetUV(ip, true, U, V);
            }
        }
    }

    private static bool InsertSeamVertices(IntSurf_LineOn2S Line, bool IsReversed, IntSurf_LineOn2S Vertices,
                                           NCollection_Array1<int> PTypes, double TOL2D)
    {
        bool result = false;
        int nbp = Line.NbPoints();
        double U, V;
        for (int ip = 1; ip <= nbp; ip++)
        {
            int ipt = PTypes.Value(ip);
            if (ipt != 0)
            {
                IntSurf_PntOn2S aP = Line.Value(ip);
                if (IsReversed)
                {
                    aP.ParametersOnS2(out U, out V); // S2 - quadric
                }
                else
                {
                    aP.ParametersOnS1(out U, out V); // S1 - quadric
                }
                U = AdjustU(U);
                if (IsSeamParameter(U, TOL2D))
                {
                    if (ip == 1 || ip == nbp)
                    {
                        double U1, V1;
                        int ipp = (ip == 1) ? (ip + 1) : (ip - 1);
                        if (IsReversed)
                        {
                            Line.Value(ipp).ParametersOnS2(out U1, out V1); // S2 - quadric
                        }
                        else
                        {
                            Line.Value(ipp).ParametersOnS1(out U1, out V1); // S1 - quadric
                        }
                        double u = AdjustUFirst(U, U1);
                        if (Math.Abs(u - U) >= 1.5 * M_PI)
                        {
                            double U2, V2;
                            if (IsReversed)
                            {
                                Line.Value(ip).ParametersOnS1(out U2, out V2); // prm
                                Line.SetUV(ip, false, u, V);
                                Line.SetUV(ip, true, U2, V2);
                            }
                            else
                            {
                                Line.Value(ip).ParametersOnS2(out U2, out V2); // prm
                                Line.SetUV(ip, true, u, V);
                                Line.SetUV(ip, false, U2, V2);
                            }
                        }
                    }
                    else
                    {
                        int ipp = ip - 1;
                        int ipn = ip + 1;
                        double U1, V1, U2, V2;
                        if (IsReversed)
                        {
                            Line.Value(ipp).ParametersOnS2(out U1, out V1); // quad
                            Line.Value(ipn).ParametersOnS2(out U2, out V2); // quad
                        }
                        else
                        {
                            Line.Value(ipp).ParametersOnS1(out U1, out V1); // quad
                            Line.Value(ipn).ParametersOnS1(out U2, out V2); // quad
                        }
                        U1 = AdjustU(U1);
                        U2 = AdjustU(U2);
                        bool pnearZero = Math.Abs(U1) < Math.Abs(2.0 * M_PI - U1);
                        bool cnearZero = Math.Abs(U) < Math.Abs(2.0 * M_PI - U);
                        if (pnearZero == cnearZero)
                        {
                            if (!IsSeamParameter(U2, TOL2D) && !IsSeamParameter(U1, TOL2D))
                            {
                                double nU = (cnearZero) ? (2.0 * M_PI) : 0.0;
                                IntSurf_PntOn2S nP = new();
                                nP.SetValue(aP.Value());
                                double U3, V3;
                                if (IsReversed)
                                {
                                    Line.Value(ip).ParametersOnS1(out U3, out V3); // prm
                                    nP.SetValue(false, nU, V);
                                    nP.SetValue(true, U3, V3);
                                }
                                else
                                {
                                    Line.Value(ip).ParametersOnS2(out U3, out V3); // prm
                                    nP.SetValue(true, nU, V);
                                    nP.SetValue(false, U3, V3);
                                }
                                Line.InsertBefore(ipn, nP);
                                Vertices.Add(nP);
                                result = true;
                                break;
                            }
                        }
                        else
                        {
                            if (!IsSeamParameter(U2, TOL2D) && !IsSeamParameter(U1, TOL2D))
                            {
                                double nU = (cnearZero) ? (2.0 * M_PI) : 0.0;
                                IntSurf_PntOn2S nP = new();
                                nP.SetValue(aP.Value());
                                double U3, V3;
                                if (IsReversed)
                                {
                                    Line.Value(ip).ParametersOnS1(out U3, out V3); // prm
                                    nP.SetValue(false, nU, V);
                                    nP.SetValue(true, U3, V3);
                                }
                                else
                                {
                                    Line.Value(ip).ParametersOnS2(out U3, out V3); // prm
                                    nP.SetValue(true, nU, V);
                                    nP.SetValue(false, U3, V3);
                                }
                                Line.InsertBefore(ip, nP);
                                Vertices.Add(nP);
                                result = true;
                                break;
                            }
                        }
                    }
                }
            }
        }
        return result;
    }

    private static void ToSmooth(IntSurf_LineOn2S Line, bool IsReversed, in IntSurf_Quadric Quad, bool IsFirst,
                                 ref double D3D)
    {
        if (Line.NbPoints() <= 10)
        {
            return;
        }

        D3D = 0.0;
        int NbTestPnts = Line.NbPoints() / 5;
        if (NbTestPnts < 5)
        {
            NbTestPnts = 5;
        }

        int startp = (IsFirst) ? 2 : (Line.NbPoints() - NbTestPnts - 2);
        double Uc, Vc, Un, Vn, DDU = 0.0;

        // OCCT compares ip with NbTestPnts, not startp + NbTestPnts: for the last points (IsFirst false) the loop
        // runs only on short lines, and D3D and DDU stay 0 otherwise (reproduced).
        for (int ip = startp; ip <= NbTestPnts; ip++)
        {
            if (IsReversed)
            {
                Line.Value(ip).ParametersOnS2(out Uc, out Vc); // S2 - quadric
                Line.Value(ip + 1).ParametersOnS2(out Un, out Vn);
            }
            else
            {
                Line.Value(ip).ParametersOnS1(out Uc, out Vc); // S1 - quadric
                Line.Value(ip + 1).ParametersOnS1(out Un, out Vn);
            }
            DDU += Math.Abs(Math.Abs(Uc) - Math.Abs(Un));

            if (ip > startp)
            {
                double DP = Line.Value(ip).Value().Distance(Line.Value(ip - 1).Value());
                D3D += DP;
            }
        }

        DDU /= (double)NbTestPnts + 1;

        D3D /= (double)NbTestPnts + 1;

        int Index1 = (IsFirst) ? 1 : (Line.NbPoints());
        int Index2 = (IsFirst) ? 2 : (Line.NbPoints() - 1);
        int Index3 = (IsFirst) ? 3 : (Line.NbPoints() - 2);

        bool doU = false;

        double U1, U2, V1, V2, U3, V3;

        if (IsReversed)
        {
            Line.Value(Index1).ParametersOnS2(out U1, out V1); // S2 - quadric
            Line.Value(Index2).ParametersOnS2(out U2, out V2);
            Line.Value(Index3).ParametersOnS2(out U3, out V3);
        }
        else
        {
            Line.Value(Index1).ParametersOnS1(out U1, out V1); // S1 - quadric
            Line.Value(Index2).ParametersOnS1(out U2, out V2);
            Line.Value(Index3).ParametersOnS1(out U3, out V3);
        }

        if (!doU && Quad.TypeQuadric() == SurfaceKind.Sphere)
        {
            if (Math.Abs(Math.Abs(U1) - Math.Abs(U2)) > (M_PI / 16.0))
            {
                doU = true;
            }

            if (doU && (Math.Abs(U1) <= 1.0e-9 || Math.Abs(U1 - 2.0 * M_PI) <= 1.0e-9))
            {
                if (Math.Abs(V1 - M_PI / 2.0) <= 1.0e-9 || Math.Abs(V1 + M_PI / 2.0) <= 1.0e-9)
                {
                }
                else
                {
                    doU = false;
                }
            }
        }

        if (Quad.TypeQuadric() == SurfaceKind.Cone)
        {
            double Uapx = 0.0, Vapx = 0.0;
            Quad.Parameters(Quad.Cone().Apex(), ref Uapx, ref Vapx);

            if (Math.Abs(Math.Abs(U1) - Math.Abs(U2)) > M_PI / 32.0)
            {
                doU = true;
            }

            if (doU && (Math.Abs(U1) <= 1.0e-9 || Math.Abs(U1 - 2.0 * M_PI) <= 1.0e-9))
            {
                if (Math.Abs(V1 - Vapx) <= 1.0e-9)
                {
                }
                else
                {
                    doU = false;
                }
            }
        }

        if (doU)
        {
            double dU = Math.Min((DDU / 10.0), 5.0e-8);
            double U = (U2 > U3) ? (U2 + dU) : (U2 - dU);
            if (IsReversed)
            {
                Line.SetUV(Index1, false, U, V1);
            }
            else
            {
                Line.SetUV(Index1, true, U, V1);
            }
        }
    }

    private static bool TestMiddleOnPrm(in IntSurf_PntOn2S aP, in IntSurf_PntOn2S aV, bool IsReversed, double ArcTol,
                                        Adaptor3d_TopolTool PDomain)
    {
        bool result = false;
        double Up, Vp, Uv, Vv;
        if (IsReversed)
        {
            aP.ParametersOnS1(out Up, out Vp); // S1 - parametric
            aV.ParametersOnS1(out Uv, out Vv);
        }
        else
        {
            aP.ParametersOnS2(out Up, out Vp); // S2 - parametric
            aV.ParametersOnS2(out Uv, out Vv);
        }
        double Um = (Up + Uv) * 0.5, Vm = (Vp + Vv) * 0.5;
        Point2D a2DPntM = new(Um, Vm);
        TopAbs_State PosM = PDomain.Classify(a2DPntM, ArcTol);
        if (PosM == TopAbs_ON || PosM == TopAbs_IN)
        {
            result = true;
        }
        return result;
    }

    private static void VerifyVertices(IntSurf_LineOn2S Line, bool IsReversed, IntSurf_LineOn2S Vertices, double TOL2D,
                                       double ArcTol, Adaptor3d_TopolTool PDomain, ref IntSurf_PntOn2S VrtF,
                                       out bool AddFirst, ref IntSurf_PntOn2S VrtL, out bool AddLast)
    {
        int nbp = Line.NbPoints(), nbv = Vertices.NbPoints();
        int FIndexSame = 0, FIndexNear = 0, LIndexSame = 0, LIndexNear = 0;
        IntSurf_PntOn2S aPF = Line.Value(1);
        IntSurf_PntOn2S aPL = Line.Value(nbp);
        double UF, VF, UL, VL;
        if (IsReversed)
        {
            aPF.ParametersOnS2(out UF, out VF);
            aPL.ParametersOnS2(out UL, out VL);
        }
        else
        {
            aPF.ParametersOnS1(out UF, out VF);
            aPL.ParametersOnS1(out UL, out VL);
        }
        Point2D a2DPF = new(UF, VF);
        Point2D a2DPL = new(UL, VL);
        double DistMinF = 1.0e+100, DistMinL = 1.0e+100;
        int FConjugated = 0, LConjugated = 0;

        int iv;

        for (iv = 1; iv <= nbv; iv++)
        {
            double Uv, Vv;
            if (IsReversed)
            {
                Vertices.Value(iv).ParametersOnS2(out Uv, out Vv);
                Uv = AdjustU(Uv);
                Vertices.SetUV(iv, false, Uv, Vv);
            }
            else
            {
                Vertices.Value(iv).ParametersOnS1(out Uv, out Vv);
                Uv = AdjustU(Uv);
                Vertices.SetUV(iv, true, Uv, Vv);
            }
        }

        for (iv = 1; iv <= nbv; iv++)
        {
            IntSurf_PntOn2S aV = Vertices.Value(iv);
            if (aPF.IsSame(aV, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                FIndexSame = iv;
                break;
            }
            else
            {
                double Uv, Vv;
                if (IsReversed)
                {
                    aV.ParametersOnS2(out Uv, out Vv);
                }
                else
                {
                    aV.ParametersOnS1(out Uv, out Vv);
                }
                Point2D a2DV = new(Uv, Vv);
                double Dist = a2DV.Distance(a2DPF);
                if (Dist < DistMinF)
                {
                    DistMinF = Dist;
                    FIndexNear = iv;
                    if (FConjugated != 0)
                    {
                        FConjugated = 0;
                    }
                }
                if (IsSeamParameter(Uv, TOL2D))
                {
                    double Ucv = (Math.Abs(Uv) < Math.Abs(2.0 * M_PI - Uv)) ? (2.0 * M_PI) : 0.0;
                    Point2D a2DCV = new(Ucv, Vv);
                    double CDist = a2DCV.Distance(a2DPF);
                    if (CDist < DistMinF)
                    {
                        DistMinF = CDist;
                        FConjugated = iv;
                        FIndexNear = iv;
                    }
                }
            }
        }

        for (iv = 1; iv <= nbv; iv++)
        {
            IntSurf_PntOn2S aV = Vertices.Value(iv);
            if (aPL.IsSame(aV, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                LIndexSame = iv;
                break;
            }
            else
            {
                double Uv, Vv;
                if (IsReversed)
                {
                    aV.ParametersOnS2(out Uv, out Vv);
                }
                else
                {
                    aV.ParametersOnS1(out Uv, out Vv);
                }
                Point2D a2DV = new(Uv, Vv);
                double Dist = a2DV.Distance(a2DPL);
                if (Dist < DistMinL)
                {
                    DistMinL = Dist;
                    LIndexNear = iv;
                    if (LConjugated != 0)
                    {
                        LConjugated = 0;
                    }
                }
                if (IsSeamParameter(Uv, TOL2D))
                {
                    double Ucv = (Math.Abs(Uv) < Math.Abs(2.0 * M_PI - Uv)) ? (2.0 * M_PI) : 0.0;
                    Point2D a2DCV = new(Ucv, Vv);
                    double CDist = a2DCV.Distance(a2DPL);
                    if (CDist < DistMinL)
                    {
                        DistMinL = CDist;
                        LConjugated = iv;
                        LIndexNear = iv;
                    }
                }
            }
        }

        AddFirst = false;
        AddLast = false;

        if (FIndexSame == 0)
        {
            if (FIndexNear != 0)
            {
                IntSurf_PntOn2S aV = Vertices.Value(FIndexNear);
                double Uv, Vv;
                if (IsReversed)
                {
                    aV.ParametersOnS2(out Uv, out Vv);
                }
                else
                {
                    aV.ParametersOnS1(out Uv, out Vv);
                }
                if (IsSeamParameter(Uv, TOL2D))
                {
                    double Ucv = (Math.Abs(Uv) < Math.Abs(2.0 * M_PI - Uv)) ? (2.0 * M_PI) : 0.0;
                    bool test = TestMiddleOnPrm(aPF, aV, IsReversed, ArcTol, PDomain);
                    if (test)
                    {
                        VrtF.SetValue(aV.Value());
                        if (IsReversed)
                        {
                            aV.ParametersOnS1(out double U2, out double V2); // S1 - prm
                            VrtF.SetValue(true, U2, V2);
                            if (FConjugated == 0)
                            {
                                VrtF.SetValue(false, Uv, Vv);
                            }
                            else
                            {
                                VrtF.SetValue(false, Ucv, Vv);
                            }
                        }
                        else
                        {
                            aV.ParametersOnS2(out double U2, out double V2); // S2 - prm
                            VrtF.SetValue(false, U2, V2);
                            if (FConjugated == 0)
                            {
                                VrtF.SetValue(true, Uv, Vv);
                            }
                            else
                            {
                                VrtF.SetValue(true, Ucv, Vv);
                            }
                        }
                        double Dist3D = VrtF.Value().Distance(aPF.Value());
                        if (Dist3D > 1.5e-7 && DistMinF > TOL2D)
                        {
                            AddFirst = true;
                        }
                    }
                }
                else
                {
                    // to do: analyze internal vertex
                }
            }
        }

        if (LIndexSame == 0)
        {
            if (LIndexNear != 0)
            {
                IntSurf_PntOn2S aV = Vertices.Value(LIndexNear);
                double Uv, Vv;
                if (IsReversed)
                {
                    aV.ParametersOnS2(out Uv, out Vv);
                }
                else
                {
                    aV.ParametersOnS1(out Uv, out Vv);
                }
                if (IsSeamParameter(Uv, TOL2D))
                {
                    double Ucv = (Math.Abs(Uv) < Math.Abs(2.0 * M_PI - Uv)) ? (2.0 * M_PI) : 0.0;
                    bool test = TestMiddleOnPrm(aPL, aV, IsReversed, ArcTol, PDomain);
                    if (test)
                    {
                        VrtL.SetValue(aV.Value());
                        if (IsReversed)
                        {
                            aV.ParametersOnS1(out double U2, out double V2); // S1 - prm
                            VrtL.SetValue(true, U2, V2);
                            if (LConjugated == 0)
                            {
                                VrtL.SetValue(false, Uv, Vv);
                            }
                            else
                            {
                                VrtL.SetValue(false, Ucv, Vv);
                            }
                        }
                        else
                        {
                            aV.ParametersOnS2(out double U2, out double V2); // S2 - prm
                            VrtL.SetValue(false, U2, V2);
                            if (LConjugated == 0)
                            {
                                VrtL.SetValue(true, Uv, Vv);
                            }
                            else
                            {
                                VrtL.SetValue(true, Ucv, Vv);
                            }
                        }
                        double Dist3D = VrtL.Value().Distance(aPL.Value());
                        if (Dist3D > 1.5e-7 && DistMinL > TOL2D)
                        {
                            AddLast = true;
                        }
                    }
                }
                else
                {
                    // to do: analyze internal vertex
                }
            }
        }
    }

    private static bool AddVertices(IntSurf_LineOn2S Line, in IntSurf_PntOn2S VrtF, bool AddFirst,
                                    in IntSurf_PntOn2S VrtL, bool AddLast, double D3DF, double D3DL)
    {
        bool result = false;
        if (AddFirst)
        {
            double DF = Line.Value(1).Value().Distance(VrtF.Value());
            if ((D3DF * 2.0) > DF && DF > 1.5e-7)
            {
                Line.InsertBefore(1, VrtF);
                result = true;
            }
        }
        if (AddLast)
        {
            double DL = Line.Value(Line.NbPoints()).Value().Distance(VrtL.Value());
            if ((D3DL * 2.0) > DL && DL > 1.5e-7)
            {
                Line.Add(VrtL);
                result = true;
            }
        }
        return result;
    }

    private static void PutIntVertices(IntPatch_PointLine Line, IntSurf_LineOn2S Result, bool theIsReversed,
                                       IntSurf_LineOn2S Vertices, double ArcTol)
    {
        int nbp = Result.NbPoints(), nbv = Vertices.NbPoints();

        if (nbp < 3)
        {
            return;
        }

        IntPatch_RLine? aRLine = Line as IntPatch_RLine;

        Point3D aPnt;
        IntPatch_Point thePnt = new();

        for (int ip = 2; ip <= (nbp - 1); ip++)
        {
            IntSurf_PntOn2S aP = Result.Value(ip);
            for (int iv = 1; iv <= nbv; iv++)
            {
                IntSurf_PntOn2S aV = Vertices.Value(iv);
                if (aP.IsSame(aV, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.PConfusion()))
                {
                    aPnt = Result.Value(ip).Value();
                    Result.Value(ip).ParametersOnS1(out double U1, out double V1);
                    Result.Value(ip).ParametersOnS2(out double U2, out double V2);
                    thePnt.SetValue(aPnt, ArcTol, false);
                    thePnt.SetParameters(U1, V1, U2, V2);

                    double aParam = (double)ip;

                    if (aRLine is not null)
                    {
                        // In fact, aRLine is always on the parametric surface.
                        // If (theIsReversed == TRUE) then (U1, V1) - point on
                        // parametric surface, otherwise - point on quadric.
                        Adaptor2d_Curve2d anArc = (aRLine.IsArcOnS1() ? aRLine.ArcOnS1() : aRLine.ArcOnS2())!;

                        Line2D aLin = anArc.Line();
                        Point2D aPSurf;

                        if (theIsReversed)
                        {
                            aPSurf = new Point2D(U1, V1);
                        }
                        else
                        {
                            aPSurf = new Point2D(U2, V2);
                        }

                        aParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(aLin, aPSurf);
                    }

                    thePnt.SetParameter(aParam);
                    Line.AddVertex(thePnt);
                }
            }
        }
    }

    private static bool HasInternals(IntSurf_LineOn2S Line, IntSurf_LineOn2S Vertices)
    {
        int nbp = Line.NbPoints(), nbv = Vertices.NbPoints();
        bool result = false;

        if (nbp < 3)
        {
            return result;
        }

        for (int ip = 2; ip <= (nbp - 1); ip++)
        {
            IntSurf_PntOn2S aP = Line.Value(ip);
            for (int iv = 1; iv <= nbv; iv++)
            {
                IntSurf_PntOn2S aV = Vertices.Value(iv);
                if (aP.IsSame(aV, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.PConfusion()))
                {
                    result = true;
                    break;
                }
            }
            if (result)
            {
                break;
            }
        }

        return result;
    }

    private static IntPatch_WLine MakeSplitWLine(IntPatch_WLine WLine, bool Tang, IntSurf_TypeTrans Trans1,
                                                 IntSurf_TypeTrans Trans2, double ArcTol, int ParFirst, int ParLast)
    {
        IntSurf_LineOn2S SLine = WLine.Curve();
        IntSurf_LineOn2S sline = new();

        for (int ip = ParFirst; ip <= ParLast; ip++)
        {
            sline.Add(SLine.Value(ip));
        }

        IntPatch_WLine wline = new(sline, Tang, Trans1, Trans2);
        wline.SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLImpPrm);

        Point3D aSPnt;
        IntPatch_Point TPntF = new(), TPntL = new();
        double uu1, vv1, uu2, vv2;

        aSPnt = sline.Value(1).Value();
        sline.Value(1).ParametersOnS1(out uu1, out vv1);
        sline.Value(1).ParametersOnS2(out uu2, out vv2);
        TPntF.SetValue(aSPnt, ArcTol, false);
        TPntF.SetParameters(uu1, vv1, uu2, vv2);
        TPntF.SetParameter(1.0);
        wline.AddVertex(TPntF);
        wline.SetFirstPoint(1);

        aSPnt = sline.Value(sline.NbPoints()).Value();
        sline.Value(sline.NbPoints()).ParametersOnS1(out uu1, out vv1);
        sline.Value(sline.NbPoints()).ParametersOnS2(out uu2, out vv2);
        TPntL.SetValue(aSPnt, ArcTol, false);
        TPntL.SetParameters(uu1, vv1, uu2, vv2);
        TPntL.SetParameter((double)sline.NbPoints());
        wline.AddVertex(TPntL);
        wline.SetLastPoint(wline.NbVertex());

        return wline;
    }

    private static bool SplitOnSegments(IntPatch_WLine WLine, bool Tang, IntSurf_TypeTrans Trans1,
                                        IntSurf_TypeTrans Trans2, double ArcTol,
                                        NCollection_Sequence<IntPatch_Line> Segments)
    {
        bool result = false;
        Segments.Clear();

        int nbv = WLine.NbVertex();
        if (nbv > 3)
        {
            for (int iv = 1; iv < nbv; iv++)
            {
                int firstPar = (int)WLine.Vertex(iv).ParameterOnLine();
                int lastPar = (int)WLine.Vertex(iv + 1).ParameterOnLine();
                if ((lastPar - firstPar) <= 1)
                {
                    continue;
                }
                else
                {
                    IntPatch_WLine splitwline = MakeSplitWLine(WLine, Tang, Trans1, Trans2, ArcTol, firstPar, lastPar);
                    Segments.Append(splitwline);
                    if (!result)
                    {
                        result = true;
                    }
                }
            }
        }
        return result;
    }

    // Returns TRUE if point <theParam> matches <theBoundary +/- thePeriod> with given tolerance criterion.
    // For not-periodic case, thePeriod must be equal to 0.0.
    private static bool IsPointOnBoundary(double theToler2D, double theBoundary, double thePeriod, double theParam)
    {
        double aDelta = Math.Abs(theParam - theBoundary);
        if (thePeriod != 0.0)
        {
            aDelta = aDelta % thePeriod; // fmod

            // 0 <= aDelta < thePeriod
            return ((aDelta < theToler2D) || ((thePeriod - aDelta) < theToler2D));
        }

        // Here, thePeriod == 0.0, aDelta > 0.0

        return (aDelta < theToler2D);
    }

    // Can change values of theNewLine (by adding the computed point on boundary, which parameter will be adjusted)
    // and theIsOnBoundary variables.
    private static void DetectOfBoundaryAchievement(Adaptor3d_Surface theQSurf, bool theIsReversed,
                                                    IntSurf_LineOn2S theSourceLine, int thePointIndex,
                                                    double theToler2D, IntSurf_LineOn2S theNewLine,
                                                    ref bool theIsOnBoundary)
    {
        double aUPeriod = theQSurf.IsUPeriodic() ? theQSurf.UPeriod() : 0.0,
               aVPeriod = theQSurf.IsVPeriodic() ? theQSurf.VPeriod() : 0.0;
        double aUf = theQSurf.FirstUParameter(), aUl = theQSurf.LastUParameter(), aVf = theQSurf.FirstVParameter(),
               aVl = theQSurf.LastVParameter();

        IntSurf_PntOn2S aPPrev = theSourceLine.Value(thePointIndex - 1), aPCurr = theSourceLine.Value(thePointIndex);
        double aUPrev, aVPrev, aUCurr, aVCurr;
        if (theIsReversed)
        {
            aPPrev.ParametersOnS2(out aUPrev, out aVPrev); // S2 - quadric, set U,V by Pnt3D
            aPCurr.ParametersOnS2(out aUCurr, out aVCurr); // S2 - quadric, set U,V by Pnt3D
        }
        else
        {
            aPPrev.ParametersOnS1(out aUPrev, out aVPrev); // S1 - quadric, set U,V by Pnt3D
            aPCurr.ParametersOnS1(out aUCurr, out aVCurr); // S1 - quadric, set U,V by Pnt3D
        }

        // Ignore cases when the WLine goes along the surface boundary completely.

        if (IsPointOnBoundary(theToler2D, aUf, aUPeriod, aUCurr) && !IsPointOnBoundary(theToler2D, aUf, aUPeriod, aUPrev))
        {
            theIsOnBoundary = true;
        }
        else if (IsPointOnBoundary(theToler2D, aUl, aUPeriod, aUCurr)
                 && !IsPointOnBoundary(theToler2D, aUl, aUPeriod, aUPrev))
        {
            theIsOnBoundary = true;
        }
        else if (IsPointOnBoundary(theToler2D, aVf, aVPeriod, aVCurr)
                 && !IsPointOnBoundary(theToler2D, aVf, aVPeriod, aVPrev))
        {
            theIsOnBoundary = true;
        }
        else if (IsPointOnBoundary(theToler2D, aVl, aVPeriod, aVCurr)
                 && !IsPointOnBoundary(theToler2D, aVl, aVPeriod, aVPrev))
        {
            theIsOnBoundary = true;
        }

        if (theIsOnBoundary)
        {
            // Adjust, to avoid bad jumping of the WLine.

            double aDu = (aUPrev - aUCurr);
            double aDv = (aVPrev - aVCurr);
            if (aUPeriod > 0.0 && (2.0 * Math.Abs(aDu) > aUPeriod))
            {
                aUCurr += Math.CopySign(aUPeriod, aDu);
            }

            if (aVPeriod > 0.0 && (2.0 * Math.Abs(aDv) > aVPeriod))
            {
                aVCurr += Math.CopySign(aVPeriod, aDv);
            }

            IntSurf_PntOn2S aPoint = aPCurr;
            aPoint.SetValue(!theIsReversed, aUCurr, aVCurr);
            theNewLine.Add(aPoint);
        }
    }

    // Splits theLine in the places where it passes through seam edge or singularity (apex of cone or pole of
    // sphere). This passage is detected by jump of U-parameter from point to point.
    private static bool DecomposeResult(IntPatch_PointLine theLine, bool IsReversed, in IntSurf_Quadric theQuad,
                                        Adaptor3d_TopolTool thePDomain, Adaptor3d_Surface theQSurf,
                                        Adaptor3d_Surface thePSurf, double theArcTol, double theTolTang,
                                        NCollection_Sequence<IntPatch_Line> theLines)
    {
        if (theLine.ArcType() == IntPatch_Restriction)
        {
            if (theLine is IntPatch_RLine aRL)
            {
                Adaptor2d_Curve2d anArc = (aRL.IsArcOnS1() ? aRL.ArcOnS1() : aRL.ArcOnS2())!;
                if (anArc.GetType() != CurveKind.Line)
                {
                    // Restriction line must be isoline.
                    // Other cases are not supported by
                    // existing algorithms.

                    return false;
                }
            }
        }

        double aDeltaUmax = M_PI_2;
        double aTOL3D = 1.0e-10, aTOL2D = global::Kerf.Engine.Foundation.Precision.PConfusion(), aTOL2DS = global::Kerf.Engine.Foundation.Precision.PConfusion();

        IntSurf_LineOn2S aSLine = theLine.Curve()!;

        if (aSLine.NbPoints() <= 2)
        {
            return false;
        }

        // Deletes repeated vertices
        IntSurf_LineOn2S aVLine = GetVertices(theLine, aTOL3D, aTOL2D);

        // OCCT's aSSLine is a second handle to the line's own points: they are edited in place.
        IntSurf_LineOn2S aSSLine = aSLine;

        if (aSSLine.NbPoints() <= 1)
        {
            return false;
        }

        AdjustLine(aSSLine, IsReversed, theQSurf, aTOL2D);

        if (theLine.ArcType() == IntPatch_Walking)
        {
            bool isInserted = true;
            while (isInserted)
            {
                int aNbPnts = aSSLine.NbPoints();
                NCollection_Array1<int> aPTypes = new(1, aNbPnts);
                SearchVertices(aSSLine, aVLine, aPTypes);
                isInserted = InsertSeamVertices(aSSLine, IsReversed, aVLine, aPTypes, aTOL2D);
            }
        }

        int aLindex = aSSLine.NbPoints();
        int aFindex = 1, aBindex = 0;

        // build WLine parts (if any)
        bool flNextLine = true;
        bool hasBeenDecomposed = false;
        IntPatch_SpecPntType aPrePointExist = IntPatch_SPntNone;

        IntSurf_PntOn2S PrePoint = new();
        while (flNextLine)
        {
            // reset variables
            flNextLine = false;
            bool isDecomposited = false;

            IntSurf_LineOn2S sline = new();

            // if((Lindex-Findex+1) <= 2 )
            if ((aLindex <= aFindex) && aPrePointExist == IntPatch_SPntNone)
            {
                // break of "while(flNextLine)" cycle
                break;
            }

            if (aPrePointExist != IntPatch_SPntNone)
            {
                IntSurf_PntOn2S aRefPt = aSSLine.Value(aFindex);

                double aURes = theQSurf.UResolution(theArcTol), aVRes = theQSurf.VResolution(theArcTol);

                double aTol2d = (aPrePointExist == IntPatch_SPntPole)     ? -1.0
                                : (aPrePointExist == IntPatch_SPntSeamV)  ? aVRes
                                : (aPrePointExist == IntPatch_SPntSeamUV) ? Math.Max(aURes, aVRes)
                                                                          : aURes;

                if (IntPatch_SpecialPoints.ContinueAfterSpecialPoint(theQSurf, thePSurf, aRefPt, aPrePointExist, aTol2d,
                                                                     ref PrePoint, IsReversed))
                {
                    sline.Add(PrePoint);

                    // Avoid adding duplicate points.
                    for (; aFindex <= aLindex; aFindex++)
                    {
                        if (!PrePoint.IsSame(aSSLine.Value(aFindex), theTolTang))
                        {
                            break;
                        }
                    }
                }
                else
                {
                    // break of "while(flNextLine)" cycle
                    break;
                }
            }

            aPrePointExist = IntPatch_SPntNone;

            // analyze other points
            for (int k = aFindex; k <= aLindex; k++)
            {
                if (k == aFindex)
                {
                    PrePoint = aSSLine.Value(k);
                    sline.Add(PrePoint);
                    continue;
                }

                // Check whether the current point is on the boundary of theQSurf.
                // If that is TRUE then the Walking-line will be decomposed in this point.
                // However, this boundary is not singular-point (like seam or pole of sphere).
                // Therefore, its processing will be simplified.
                bool isOnBoundary = false;

                // Values of sline and isOnBoundary can be changed by this function
                DetectOfBoundaryAchievement(theQSurf, IsReversed, aSSLine, k, aTOL2D, sline, ref isOnBoundary);

                aPrePointExist = IsSeamOrPole(theQSurf, aSSLine, IsReversed, k - 1, theTolTang, aDeltaUmax);

                if (isOnBoundary && (aPrePointExist != IntPatch_SPntPoleSeamU))
                {
                    // If the considered point is on seam then its UV-parameters
                    // are defined to within the surface period. Therefore, we can
                    // trust already computed parameters of this point.
                    // However, if this point (which is on the surface boundary) is
                    // a sphere pole or cone apex then its (point's) parameters
                    // have to be recomputed in the code below
                    // (see IntPatch_SpecialPoints::AddSingularPole() method).
                    // E.g. see "bugs modalg_6 bug26684_2" test case.

                    aPrePointExist = IntPatch_SPntNone;
                }

                if (aPrePointExist != IntPatch_SPntNone)
                {
                    aBindex = k;
                    isDecomposited = true;
                    ////
                    IntSurf_PntOn2S aRefPt = aSSLine.Value(aBindex - 1);

                    double aCompareTol3D = global::Kerf.Engine.Foundation.Precision.Confusion();
                    double aCompareTol2D = global::Kerf.Engine.Foundation.Precision.PConfusion();

                    IntSurf_PntOn2S aNewPoint = aRefPt;
                    IntPatch_SpecPntType aLastType = IntPatch_SPntNone;

                    if (aPrePointExist == IntPatch_SPntSeamUV)
                    {
                        aPrePointExist = IntPatch_SPntNone;
                        aLastType = IntPatch_SPntSeamUV;
                        IntPatch_SpecialPoints.AddCrossUVIsoPoint(theQSurf, thePSurf, aRefPt, theTolTang, ref aNewPoint,
                                                                  IsReversed);
                    }
                    else if (aPrePointExist == IntPatch_SPntSeamV)
                    { // WLine goes through seam
                        aPrePointExist = IntPatch_SPntNone;
                        aLastType = IntPatch_SPntSeamV;

                        // Not quadric point
                        double aU0, aV0;
                        // Quadric point
                        double aUQuadRef, aVQuadRef;

                        if (IsReversed)
                        {
                            aRefPt.Parameters(out aU0, out aV0, out aUQuadRef, out aVQuadRef);
                        }
                        else
                        {
                            aRefPt.Parameters(out aUQuadRef, out aVQuadRef, out aU0, out aV0);
                        }

                        math_Vector aTol = new(1, 3), aStartPoint = new(1, 3), anInfBound = new(1, 3),
                                    aSupBound = new(1, 3);

                        // Parameters on parametric surface
                        double aUp, aVp, aUq, aVq;
                        if (IsReversed)
                        {
                            aSSLine.Value(k).Parameters(out aUp, out aVp, out aUq, out aVq);
                        }
                        else
                        {
                            aSSLine.Value(k).Parameters(out aUq, out aVq, out aUp, out aVp);
                        }

                        aTol[1] = thePSurf.UResolution(theArcTol);
                        aTol[2] = thePSurf.VResolution(theArcTol);
                        aTol[3] = theQSurf.UResolution(theArcTol);
                        aStartPoint[1] = 0.5 * (aU0 + aUp);
                        aStartPoint[2] = 0.5 * (aV0 + aVp);
                        aStartPoint[3] = 0.5 * (aUQuadRef + aUq);
                        anInfBound[1] = thePSurf.FirstUParameter();
                        anInfBound[2] = thePSurf.FirstVParameter();
                        anInfBound[3] = theQSurf.FirstUParameter();
                        aSupBound[1] = thePSurf.LastUParameter();
                        aSupBound[2] = thePSurf.LastVParameter();
                        aSupBound[3] = theQSurf.LastUParameter();

                        IntPatch_SpecialPoints.AddPointOnUorVIso(theQSurf, thePSurf, aRefPt, false, 0.0, aTol, aStartPoint,
                                                                 anInfBound, aSupBound, ref aNewPoint, IsReversed);
                    }
                    else if (aPrePointExist == IntPatch_SPntPoleSeamU)
                    {
                        aPrePointExist = IntPatch_SPntNone;

                        IntPatch_Point aVert = new();
                        aVert.SetValue(aRefPt);
                        aVert.SetTolerance(theTolTang);

                        if (IntPatch_SpecialPoints.AddSingularPole(theQSurf, thePSurf, aRefPt, ref aVert, ref aNewPoint,
                                                                   IsReversed))
                        {
                            aPrePointExist = IntPatch_SPntPole;
                            aLastType = IntPatch_SPntPole;
                            if (isOnBoundary)
                            {
                                // It is necessary to replace earlier added point on
                                // the surface boundary with the pole. For that,
                                // here we delete excess point. New point will be added later.
                                isOnBoundary = false;
                                sline.RemovePoint(sline.NbPoints());
                            }

                            aCompareTol2D = -1.0;
                        } // if(IntPatch_AddSpecialPoints::AddSingularPole(...))
                        else
                        { // Pole is not an intersection point
                            aPrePointExist = IntPatch_SPntSeamU;
                        }
                    }

                    if (aPrePointExist == IntPatch_SPntSeamU)
                    { // WLine goes through seam
                        aPrePointExist = IntPatch_SPntNone;
                        aLastType = IntPatch_SPntSeamU;

                        // Not quadric point
                        double aU0, aV0;
                        // Quadric point
                        double aUQuadRef, aVQuadRef;

                        if (IsReversed)
                        {
                            aRefPt.Parameters(out aU0, out aV0, out aUQuadRef, out aVQuadRef);
                        }
                        else
                        {
                            aRefPt.Parameters(out aUQuadRef, out aVQuadRef, out aU0, out aV0);
                        }

                        math_Vector aTol = new(1, 3), aStartPoint = new(1, 3), anInfBound = new(1, 3),
                                    aSupBound = new(1, 3);

                        // Parameters on parametric surface
                        double aUp, aVp, aUq, aVq;
                        if (IsReversed)
                        {
                            aSSLine.Value(k).Parameters(out aUp, out aVp, out aUq, out aVq);
                        }
                        else
                        {
                            aSSLine.Value(k).Parameters(out aUq, out aVq, out aUp, out aVp);
                        }

                        aTol[1] = thePSurf.UResolution(theArcTol);
                        aTol[2] = thePSurf.VResolution(theArcTol);
                        aTol[3] = theQSurf.VResolution(theArcTol);
                        aStartPoint[1] = 0.5 * (aU0 + aUp);
                        aStartPoint[2] = 0.5 * (aV0 + aVp);
                        aStartPoint[3] = 0.5 * (aVQuadRef + aVq);
                        anInfBound[1] = thePSurf.FirstUParameter();
                        anInfBound[2] = thePSurf.FirstVParameter();
                        anInfBound[3] = theQSurf.FirstVParameter();
                        aSupBound[1] = thePSurf.LastUParameter();
                        aSupBound[2] = thePSurf.LastVParameter();
                        aSupBound[3] = theQSurf.LastVParameter();

                        IntPatch_SpecialPoints.AddPointOnUorVIso(theQSurf, thePSurf, aRefPt, true, 0.0, aTol, aStartPoint,
                                                                 anInfBound, aSupBound, ref aNewPoint, IsReversed);
                    }

                    if (!aNewPoint.IsSame(aRefPt, aCompareTol3D, aCompareTol2D))
                    {
                        if (isOnBoundary)
                        {
                            break;
                        }

                        sline.Add(aNewPoint);
                        aPrePointExist = aLastType;
                        PrePoint = aNewPoint;
                    }
                    else
                    {
                        if (isOnBoundary || (sline.NbPoints() == 1))
                        {
                            // FIRST point of the sline is the pole of the quadric.
                            // Therefore, there is no point in decomposition.

                            // If the considered point is on surface boundary then
                            // it is already marked as vertex. So, decomposition is
                            // not required, too.

                            PrePoint = aRefPt;
                            aPrePointExist = isOnBoundary ? IntPatch_SPntNone : aLastType;
                        }
                    }

                    ////
                    break;
                } // if (aPrePointExist != IntPatch_SPntNone) cond.

                PrePoint = aSSLine.Value(k);

                if (isOnBoundary)
                {
                    aBindex = k;
                    isDecomposited = true;
                    aPrePointExist = IntPatch_SPntNone;
                    break;
                }
                else
                {
                    sline.Add(aSSLine.Value(k));
                }
            } // for(int k = aFindex; k <= aLindex; k++)

            // Creation of new line as part of existing theLine.
            // This part is defined by sline.

            if (sline.NbPoints() == 1)
            {
                flNextLine = true;

                if (aFindex < aBindex)
                {
                    aFindex = aBindex;
                }

                // Go to the next part of aSSLine
                // because we cannot create the line
                // with single point.

                continue;
            }

            IntSurf_PntOn2S aVF = new(), aVL = new();
            VerifyVertices(sline, IsReversed, aVLine, aTOL2DS, theArcTol, thePDomain, ref aVF, out bool addVF, ref aVL,
                           out bool addVL);

            bool hasInternals = HasInternals(sline, aVLine);

            double D3F = 0.0, D3L = 0.0;
            ToSmooth(sline, IsReversed, theQuad, true, ref D3F);
            ToSmooth(sline, IsReversed, theQuad, false, ref D3L);

            if (addVF || addVL)
            {
                bool isAdded = AddVertices(sline, aVF, addVF, aVL, addVL, D3F, D3L);
                if (isAdded)
                {
                    ToSmooth(sline, IsReversed, theQuad, true, ref D3F);
                    ToSmooth(sline, IsReversed, theQuad, false, ref D3L);
                }
            }

            if (theLine.ArcType() == IntPatch_Walking)
            {
                IntPatch_Point aTPntF = new(), aTPntL = new();

                IntPatch_WLine wline = new(sline, false, theLine.TransitionOnS1(), theLine.TransitionOnS2());
                wline.SetCreatingWayInfo(IntPatch_WLine.IntPatch_WLType.IntPatch_WLImpPrm);

                Point3D aSPnt = sline.Value(1).Value();
                sline.Value(1).Parameters(out double aU1, out double aV1, out double aU2, out double aV2);
                aTPntF.SetValue(aSPnt, theArcTol, false);
                aTPntF.SetParameters(aU1, aV1, aU2, aV2);
                aTPntF.SetParameter(1.0);
                wline.AddVertex(aTPntF);
                wline.SetFirstPoint(1);

                if (hasInternals)
                {
                    PutIntVertices(wline, sline, IsReversed, aVLine, theArcTol);
                }

                aSPnt = sline.Value(sline.NbPoints()).Value();
                sline.Value(sline.NbPoints()).Parameters(out aU1, out aV1, out aU2, out aV2);
                aTPntL.SetValue(aSPnt, theArcTol, false);
                aTPntL.SetParameters(aU1, aV1, aU2, aV2);
                aTPntL.SetParameter(sline.NbPoints());
                wline.AddVertex(aTPntL);
                wline.SetLastPoint(wline.NbVertex());

                NCollection_Sequence<IntPatch_Line> segm = new();
                bool isSplited = SplitOnSegments(wline, false, theLine.TransitionOnS1(), theLine.TransitionOnS2(),
                                                 theArcTol, segm);

                if (!isSplited)
                {
                    theLines.Append(wline);
                }
                else
                {
                    int nbsegms = segm.Length();
                    for (int iseg = 1; iseg <= nbsegms; iseg++)
                    {
                        theLines.Append(segm.Value(iseg));
                    }
                }
            }
            else
            { // theLine->ArcType() == IntPatch_Restriction
                if (!isDecomposited && !hasBeenDecomposed)
                {
                    // The line has not been changed
                    theLines.Append((IntPatch_RLine)theLine);
                    return hasBeenDecomposed;
                }

                IntPatch_Point aTPnt = new();
                Point2D aPSurf;
                Point3D aSPnt;

                IntPatch_RLine aRLine = new((IntPatch_RLine)theLine);

                aRLine.ClearVertexes();
                aRLine.SetCurve(sline);

                if (hasInternals)
                {
                    PutIntVertices(aRLine, sline, IsReversed, aVLine, theArcTol);
                }

                Adaptor2d_Curve2d anArc = (aRLine.IsArcOnS1() ? aRLine.ArcOnS1() : aRLine.ArcOnS2())!;

                double aFPar = anArc.FirstParameter(), aLPar = anArc.LastParameter();

                IntSurf_PntOn2S aRFirst = sline.Value(1), aRLast = sline.Value(sline.NbPoints());

                Line2D aLin = anArc.Line();

                for (int aFLIndex = 0; aFLIndex < 2; aFLIndex++)
                {
                    double aU1, aV1, aU2, aV2;
                    if (aFLIndex == 0)
                    {
                        aRFirst.Parameters(out aU1, out aV1, out aU2, out aV2);
                        aSPnt = aRFirst.Value();
                    }
                    else
                    {
                        aRLast.Parameters(out aU1, out aV1, out aU2, out aV2);
                        aSPnt = aRLast.Value();
                    }

                    if (IsReversed)
                    {
                        aPSurf = new Point2D(aU1, aV1);
                    }
                    else
                    {
                        aPSurf = new Point2D(aU2, aV2);
                    }

                    double aPar = global::Kerf.Engine.Geometry.ElCLib.Parameter(aLin, aPSurf);

                    if (aFLIndex == 0)
                    {
                        aFPar = Math.Max(aFPar, aPar);
                        aPar = aFPar;
                    }
                    else
                    {
                        aLPar = Math.Min(aLPar, aPar);
                        aPar = aLPar;
                    }

                    aTPnt.SetParameter(aPar);
                    aTPnt.SetValue(aSPnt, theArcTol, false);
                    aTPnt.SetParameters(aU1, aV1, aU2, aV2);

                    aRLine.AddVertex(aTPnt);
                }

                if (aLPar - aFPar > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    aRLine.SetFirstPoint(1);
                    aRLine.SetLastPoint(aRLine.NbVertex());

                    // OCCT: Trim returns a new adaptor, which is discarded; the arc itself is not trimmed (reproduced).
                    _ = anArc.Trim(aFPar, aLPar, theArcTol);

                    theLines.Append(aRLine);
                }
            }

            if (isDecomposited)
            {
                aFindex = aBindex;
                flNextLine = hasBeenDecomposed = true;
            }
        }

        return hasBeenDecomposed;
    }

    // Returns TRUE if the segment [theParF, theParL] is included in the segment [theRefParF, theRefParL] segment.
    private static bool CheckSegmSegm(double theRefParF, double theRefParL, double theParF, double theParL)
    {
        if ((theParF < theRefParF) || (theParF > theRefParL))
        {
            return false;
        }

        if ((theParL < theRefParF) || (theParL > theRefParL))
        {
            return false;
        }

        return true;
    }

    // Check, if theLine is coincided with theArc (in 2d-space). Cases when theArc is not 2d-line adaptor are
    // supported by TopOpeBRep classes only (i.e. are archaic).
    private static bool IsCoincide(IntPatch_TheSurfFunction theFunc, IntPatch_PointLine theLine,
                                   Adaptor2d_Curve2d theArc, bool isTheSurface1Using, double theToler3D,
                                   double theToler2D, double thePeriod)
    {
        ReadOnlySpan<double> aCoeffs =
        [
            0.02447174185, 0.09549150281, 0.20610737385, 0.34549150281, /*Sin(x)*Sin(x)*/
            0.5, 0.65450849719, 0.79389262615,
        ];
        if (theLine.ArcType() == IntPatch_Restriction)
        { // Restriction-restriction processing
            IntPatch_RLine aRL2 = (IntPatch_RLine)theLine;
            Adaptor2d_Curve2d anArc = (aRL2.IsArcOnS1() ? aRL2.ArcOnS1() : aRL2.ArcOnS2())!;

            if (anArc.GetType() != CurveKind.Line)
            {
                // Restriction line must be isoline.
                // Other cases are not supported by
                // existing algorithms.

                return false;
            }

            Line2D aLin1 = theArc.Line(), aLin2 = anArc.Line();

            if (!aLin1.Direction().IsParallel(aLin2.Direction(), global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                return false;
            }

            double aDist = theArc.Line().Distance(anArc.Line());
            if ((aDist < theToler2D) || (Math.Abs(aDist - thePeriod) < theToler2D))
            {
                double aRf = theArc.FirstParameter(), aRl = theArc.LastParameter();
                double aParf = anArc.FirstParameter(), aParl = anArc.LastParameter();
                Point2D aP1 = global::Kerf.Engine.Geometry.ElCLib.Value(aParf, aLin2), aP2 = global::Kerf.Engine.Geometry.ElCLib.Value(aParl, aLin2);

                double aParam1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aLin1, aP1), aParam2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(aLin1, aP2);

                if (CheckSegmSegm(aRf, aRl, aParam1, aParam2))
                {
                    return true;
                }

                // Lines are parallel. Therefore, there is no point in
                // projecting points to another line in order to check
                // if segment second line is included in segment of first one.

                return CheckSegmSegm(aParam1, aParam2, aRf, aRl);
            }

            return false;
        }

        int aNbPnts = theLine.NbPnts();
        double aUAf = theArc.FirstParameter(), aUAl = theArc.LastParameter();
        Line2D anArcLin = theArc.Line();

        math_Vector aX = new(1, 2), aVal = new(1, 1);

        for (int aPtID = 1; aPtID <= aNbPnts; aPtID++)
        {
            double aUf, aVf;
            if (isTheSurface1Using)
            {
                theLine.Point(aPtID).ParametersOnS1(out aUf, out aVf);
            }
            else
            {
                theLine.Point(aPtID).ParametersOnS2(out aUf, out aVf);
            }

            // Take 2d-point in parametric surface (because theArc is
            // 2d-line in parametric surface).
            Point2D aPloc = new(aUf, aVf);

            double aRParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(anArcLin, aPloc);

            if ((aRParam < aUAf) || (aRParam > aUAl))
            {
                return false;
            }

            Point2D aPmin = global::Kerf.Engine.Geometry.ElCLib.Value(aRParam, anArcLin);

            double aDist = aPloc.Distance(aPmin);
            if ((aDist < theToler2D) || (Math.Abs(aDist - thePeriod) < theToler2D))
            { // Considered point is in Restriction line.
                // Go to the next point.
                continue;
            }

            // Check if intermediate points between aPloc and theArc are
            // intersection point (i.e. if aPloc is in tangent zone between
            // two intersected surfaces).

            double aUl = aPmin.X(), aVl = aPmin.Y();

            double aU, aV;
            double dU = aUl - aUf, dV = aVl - aVf;
            for (int i = 0; i < 7; i++)
            {
                aU = aUf + aCoeffs[i] * dU;
                aV = aVf + aCoeffs[i] * dV;

                aX[1] = aU;
                aX[2] = aV;

                if (!theFunc.Value(aX, aVal))
                {
                    return false;
                }

                if (Math.Abs(theFunc.Root()) > theToler3D)
                {
                    return false;
                }
            }
        }

        return true;
    }
}
