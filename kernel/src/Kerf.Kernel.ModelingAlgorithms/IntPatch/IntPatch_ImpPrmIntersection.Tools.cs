// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// The file-static functions of IntPatch_ImpPrmIntersection.cxx (see IntPatch_ImpPrmIntersection.cs).

using Kerf.Engine.Booleans.Legacy;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Patches;

internal partial class ImpPrmIntersection
{
    private static SpecPntType IsSeamOrPole(SurfaceAdaptorBase qSurf, LineOn2S line,
                                                     bool isReversed, int refIndex, double tol3D,
                                                     double deltaMax)
    {
        if ((refIndex < 1) || (refIndex >= line.PointCount))
        {
            return SpecPntType.SPntNone;
        }

        // Parameters on Quadric and on parametric for reference point
        double uqRef, vqRef, upRef, vpRef;
        double uqNext, vqNext, upNext, vpNext;

        Point3D p3d = line.Value(refIndex + 1).Value();

        if (isReversed)
        {
            line.Value(refIndex).Parameters(out upRef, out vpRef, out uqRef, out vqRef);
            line.Value(refIndex + 1).Parameters(out upNext, out vpNext, out uqNext, out vqNext);
        }
        else
        {
            line.Value(refIndex).Parameters(out uqRef, out vqRef, out upRef, out vpRef);
            line.Value(refIndex + 1).Parameters(out uqNext, out vqNext, out upNext, out vpNext);
        }

        SurfaceKind type = qSurf.GetType();

        if ((type == SurfaceKind.Cone) && (qSurf.Cone().Apex.SquareDistanceTo(p3d) < tol3D * tol3D))
        {
            return SpecPntType.SPntPoleSeamU;
        }
        else if (type == SurfaceKind.Sphere)
        {
            double sqTol = tol3D * tol3D;
            Point3D p = global::Kerf.Engine.Geometry.ElSLib.Value(0.0, M_PI_2, qSurf.Sphere());
            if (p.SquareDistanceTo(p3d) < sqTol)
            {
                return SpecPntType.SPntPoleSeamU;
            }

            p = global::Kerf.Engine.Geometry.ElSLib.Value(0.0, -M_PI_2, qSurf.Sphere());
            if (p.SquareDistanceTo(p3d) < sqTol)
            {
                return SpecPntType.SPntPoleSeamU;
            }
        }

        double deltaU = Math.Abs(uqRef - uqNext);

        if ((type != SurfaceKind.Torus) && (deltaU < deltaMax))
        {
            return SpecPntType.SPntNone;
        }

        switch (type)
        {
            case SurfaceKind.Cylinder:
                return SpecPntType.SPntSeamU;

            case SurfaceKind.Torus:
            {
                double deltaV = Math.Abs(vqRef - vqNext);

                if ((deltaU >= deltaMax) && (deltaV >= deltaMax))
                {
                    return SpecPntType.SPntSeamUV;
                }

                if (deltaU >= deltaMax)
                {
                    return SpecPntType.SPntSeamU;
                }

                if (deltaV >= deltaMax)
                {
                    return SpecPntType.SPntSeamV;
                }
            }

            break;
            case SurfaceKind.Sphere:
            case SurfaceKind.Cone:
                return SpecPntType.SPntPoleSeamU;
            default:
                break;
        }

        return SpecPntType.SPntNone;
    }

    private static void ComputeTangency(TheSOnBounds solrst, NCollection_Sequence<PathPoint> seqpdep,
                                        TopolTool domain, TheSurfFunction func,
                                        SurfaceAdaptorBase pSurf, NCollection_Array1<int> destination)
    {
        int i, k, nbPoints, seqlength;
        double theparam, test;
        bool fairpt, ispassing;
        Orientation arcorien, vtxorien;
        Curve2DAdaptorBase thearc;
        // OCCT keeps the vertex handle across the points: in the complex-transition branch of an INTERNAL or EXTERNAL
        // arc it is the one an earlier point left (null if none), as here.
        HVertex? vtx = null, vtxbis;
        ThePathPointOfTheSOnBounds pStart;
        PathPoint pPoint = new();
        Vector3D vectg;
        Direction2D dirtg;
        Vector3D d1u, d1v, v1, v2 = new();
        Point2D p2d;
        Vector2D d2d;
        //
        math_Vector x = new(1, 2);
        math_Vector f = new(1, 1);
        Matrix d = new(1, 1, 1, 2);
        //
        seqlength = 0;
        nbPoints = solrst.PointCount;
        for (i = 1; i <= nbPoints; i++)
        {
            if (destination.Value(i) == 0)
            {
                pStart = solrst.Point(i);
                thearc = pStart.Arc();
                theparam = pStart.Parameter();
                arcorien = domain.Orientation(thearc);
                ispassing = (arcorien == Internal || arcorien == External);

                thearc.D0(theparam, out p2d);
                x[1] = p2d.X;
                x[2] = p2d.Y;
                pPoint.SetValue(pStart.Value(), x[1], x[2]);

                func.Values(x, f, d);
                if (func.IsTangent())
                {
                    pPoint.SetTangency(true);
                    destination.SetValue(i, seqlength + 1);
                    if (!pStart.IsNew())
                    {
                        vtx = pStart.Vertex();
                        for (k = i + 1; k <= nbPoints; k++)
                        {
                            if (destination.Value(k) == 0)
                            {
                                pStart = solrst.Point(k);
                                if (!pStart.IsNew())
                                {
                                    vtxbis = pStart.Vertex();
                                    if (domain.Identical(vtx, vtxbis))
                                    {
                                        thearc = pStart.Arc();
                                        theparam = pStart.Parameter();
                                        arcorien = domain.Orientation(thearc);
                                        ispassing = ispassing
                                                    && (arcorien == Internal || arcorien == External);

                                        thearc.D0(theparam, out p2d);
                                        pPoint.AddUV(p2d.X, p2d.Y);
                                        destination.SetValue(k, seqlength + 1);
                                    }
                                }
                            }
                        }
                    }
                    pPoint.SetPassing(ispassing);
                    seqpdep.Append(pPoint);
                    seqlength++;
                }
                else
                { // on a un point de depart potentiel

                    vectg = func.Direction3d();
                    dirtg = func.Direction2d();

                    pSurf.D1(x[1], x[2], out _, out d1u, out d1v);
                    thearc.D1(theparam, out p2d, out d2d);
                    v2 = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);
                    v1 = d1u.Crossed(d1v);

                    test = vectg.Dot(v1.Crossed(v2));
                    if (pStart.IsNew())
                    {
                        if ((test < 0.0 && arcorien == Forward) || (test > 0.0 && arcorien == Reversed))
                        {
                            vectg = vectg.Reversed();
                            dirtg = dirtg.Reversed();
                        }
                        pPoint.SetDirections(vectg, dirtg);
                        pPoint.SetPassing(ispassing);
                        destination.SetValue(i, seqlength + 1);
                        seqpdep.Append(pPoint);
                        seqlength++;
                    }
                    else
                    { // traiter la transition complexe
                        Direction3D bidnorm = new(1.0, 1.0, 1.0);
                        double tole = 1.0e-8;
                        Orientation locTrans;
                        CurveTransition comptrans = new();
                        comptrans.Reset(new Direction3D(vectg), bidnorm, 0.0);
                        if (arcorien == Forward || arcorien == Reversed)
                        {
                            // pour essai

                            vtx = pStart.Vertex();
                            vtxorien = domain.Orientation(vtx);
                            if (Math.Abs(test) <= tole)
                            {
                                locTrans = External; // et pourquoi pas INTERNAL
                            }
                            else
                            {
                                if (((test > 0.0) && arcorien == Forward)
                                    || ((test < 0.0) && arcorien == Reversed))
                                {
                                    locTrans = Forward;
                                }
                                else
                                {
                                    locTrans = Reversed;
                                }
                                if (arcorien == Reversed)
                                {
                                    v2 = v2.Reversed();
                                }
                            }

                            comptrans.Compare(tole, new Direction3D(v2), bidnorm, 0.0, locTrans, vtxorien);
                        }
                        destination.SetValue(i, seqlength + 1);
                        for (k = i + 1; k <= nbPoints; k++)
                        {
                            if (destination.Value(k) == 0)
                            {
                                pStart = solrst.Point(k);
                                if (!pStart.IsNew())
                                {
                                    vtxbis = pStart.Vertex();
                                    if (domain.Identical(vtx!, vtxbis))
                                    {
                                        thearc = pStart.Arc();
                                        theparam = pStart.Parameter();
                                        arcorien = domain.Orientation(thearc);

                                        pPoint.AddUV(x[1], x[2]);

                                        thearc.D1(theparam, out p2d, out d2d);
                                        pPoint.AddUV(p2d.X, p2d.Y);

                                        if (arcorien == Forward || arcorien == Reversed)
                                        {
                                            ispassing = false;
                                            v2 = Vector3D.LinearForm(d2d.X, d1u, d2d.Y, d1v);

                                            test = vectg.Dot(v1.Crossed(v2));
                                            vtxorien = domain.Orientation(pStart.Vertex());
                                            if (Math.Abs(test) <= tole)
                                            {
                                                locTrans = External; // et pourquoi pas INTERNAL
                                            }
                                            else
                                            {
                                                if (((test > 0.0) && arcorien == Forward)
                                                    || ((test < 0.0) && arcorien == Reversed))
                                                {
                                                    locTrans = Forward;
                                                }
                                                else
                                                {
                                                    locTrans = Reversed;
                                                }
                                                if (arcorien == Reversed)
                                                {
                                                    v2 = v2.Reversed();
                                                }
                                            }

                                            comptrans.Compare(tole, new Direction3D(v2), bidnorm, 0.0, locTrans, vtxorien);
                                        }
                                        destination.SetValue(k, seqlength + 1);
                                    }
                                }
                            }
                        }
                        fairpt = true;
                        if (!ispassing)
                        {
                            Containment before = comptrans.StateBefore();
                            Containment after = comptrans.StateAfter();
                            if ((before == Unknown) || (after == Unknown))
                            {
                                fairpt = false;
                            }
                            else if (before == In)
                            {
                                if (after == In)
                                {
                                    ispassing = true;
                                }
                                else
                                {
                                    vectg = vectg.Reversed();
                                    dirtg = dirtg.Reversed();
                                }
                            }
                            else
                            {
                                if (after != In)
                                {
                                    fairpt = false;
                                }
                            }
                        }
                        if (fairpt)
                        {
                            pPoint.SetDirections(vectg, dirtg);
                            pPoint.SetPassing(ispassing);
                            seqpdep.Append(pPoint);
                            seqlength++;
                        }
                        else
                        { // il faut remettre en "ordre" si on ne garde pas le point.
                            for (k = i; k <= nbPoints; k++)
                            {
                                if (destination.Value(k) == seqlength + 1)
                                {
                                    destination.SetValue(k, -destination.Value(k));
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    private static void Recadre(bool _, SurfaceKind typeS1, SurfaceKind typeS2, ref PatchPoint pt,
                                PatchTheIWLineOfTheIWalking iwline, int param, double u1, double v1, double u2,
                                double v2)
    {
        iwline.Line().Value(param).Parameters(out double u1p, out double v1p, out double u2p, out double v2p);
        switch (typeS1)
        {
            case SurfaceKind.Torus:
                while (v1 < (v1p - 1.5 * M_PI))
                {
                    v1 += M_PI + M_PI;
                }
                while (v1 > (v1p + 1.5 * M_PI))
                {
                    v1 -= M_PI + M_PI;
                }
                goto case SurfaceKind.Cylinder;
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
                while (u1 < (u1p - 1.5 * M_PI))
                {
                    u1 += M_PI + M_PI;
                }
                while (u1 > (u1p + 1.5 * M_PI))
                {
                    u1 -= M_PI + M_PI;
                }
                break;
            default:
                break;
        }
        switch (typeS2)
        {
            case SurfaceKind.Torus:
                while (v2 < (v2p - 1.5 * M_PI))
                {
                    v2 += M_PI + M_PI;
                }
                while (v2 > (v2p + 1.5 * M_PI))
                {
                    v2 -= M_PI + M_PI;
                }
                goto case SurfaceKind.Cylinder;
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
                while (u2 < (u2p - 1.5 * M_PI))
                {
                    u2 += M_PI + M_PI;
                }
                while (u2 > (u2p + 1.5 * M_PI))
                {
                    u2 -= M_PI + M_PI;
                }
                break;
            default:
                break;
        }
        pt.SetParameters(u1, v1, u2, v2);
    }

    private static double GetLocalStep(SurfaceAdaptorBase surf, double step)
    {
        double localStep = step;
        if (surf.UContinuity() > C0 && surf.VContinuity() > C0)
        {
            SurfaceKind sType = surf.GetType();

            if (sType == SurfaceKind.BezierSurface || sType == SurfaceKind.BSplineSurface)
            {
                double minRes;
                int maxDeg;
                double limRes = 1.0e-10;

                minRes = Math.Min(surf.UResolution(global::Kerf.Engine.Foundation.Precision.Confusion()),
                                   surf.VResolution(global::Kerf.Engine.Foundation.Precision.Confusion()));
                maxDeg = Math.Max(surf.UDegree(), surf.VDegree());
                if (minRes < limRes && maxDeg > 3)
                {
                    localStep = 0.0001;
                }
            }
        }
        if (surf.UContinuity() == C0)
        {
            int nbInt = surf.NbUIntervals(C1);
            if (nbInt > 1)
            {
                NCollection_Array1<double> ints = new(1, nbInt + 1);
                surf.UIntervals(ints, C1);
                double minInt = global::Kerf.Engine.Foundation.Precision.Infinite();
                for (int i = 1; i <= nbInt; ++i)
                {
                    minInt = Math.Min(minInt, ints.Value(i + 1) - ints.Value(i));
                }

                minInt /= surf.LastUParameter() - surf.FirstUParameter();
                if (minInt < 0.002)
                {
                    localStep = 0.0001;
                }
            }
        }

        if (surf.VContinuity() == C0)
        {
            int nbInt = surf.NbVIntervals(C1);
            if (nbInt > 1)
            {
                NCollection_Array1<double> ints = new(1, nbInt + 1);
                surf.VIntervals(ints, C1);
                double minInt = global::Kerf.Engine.Foundation.Precision.Infinite();
                for (int i = 1; i <= nbInt; ++i)
                {
                    minInt = Math.Min(minInt, ints.Value(i + 1) - ints.Value(i));
                }

                minInt /= surf.LastVParameter() - surf.FirstVParameter();
                if (minInt < 0.002)
                {
                    localStep = 0.0001;
                }
            }
        }

        localStep = Math.Min(step, localStep);
        return localStep;
    }

    // correct U parameter of the start point of line on Quadric
    // (change 0->2PI or vs, if necessary)
    private static double AdjustUFirst(double u1, double u2)
    {
        double u = u1;

        // case: no adjustment
        if (u1 > 0.0 && u1 < (2.0 * M_PI))
        {
            return u;
        }

        // case: near '0'
        if (u1 == 0.0 || Math.Abs(u1) <= 1.0e-9)
        {
            if (u2 > 0.0 && u2 < (2.0 * M_PI))
            {
                u = (u2 < ((2.0 * M_PI) - u2)) ? 0.0 : (2.0 * M_PI);
            }
            else
            {
                double uu = u2;
                if (u2 > (2.0 * M_PI))
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
        else if (u1 == (2.0 * M_PI) || Math.Abs((2.0 * M_PI) - Math.Abs(u1)) <= 1.0e-9)
        {
            if (u2 > 0.0 && u2 < (2.0 * M_PI))
            {
                u = (u2 < ((2.0 * M_PI) - u2)) ? 0.0 : (2.0 * M_PI);
            }
            else
            {
                double uu = u2;
                if (u2 > (2.0 * M_PI))
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
            if (u1 < 0.0)
            {
                while (u < 0.0)
                {
                    u += 2.0 * M_PI;
                }
            }
            if (u1 > (2.0 * M_PI))
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
    private static LineOn2S GetVertices(PointLine pLine, double tol3D, double tol2D)
    {
        LineOn2S vertices = new();

        double u1, u2, v1, v2;
        int i, k;
        int nbVrt = pLine.VertexCount;

        NCollection_Array1<int> vrts = new(1, nbVrt);
        vrts.Init(0);

        // check equal vertices
        for (i = 1; i <= nbVrt; i++)
        {
            if (vrts.Value(i) == -1)
            {
                continue;
            }

            PatchPoint pi = pLine.Vertex(i);

            for (k = (i + 1); k <= nbVrt; k++)
            {
                if (vrts.Value(k) == -1)
                {
                    continue;
                }

                PatchPoint pk = pLine.Vertex(k);

                if (pi.Value().DistanceTo(pk.Value()) <= tol3D)
                {
                    // suggest the points are equal;
                    // test 2d parameters on surface
                    bool sameU1 = false;
                    bool sameV1 = false;
                    bool sameU2 = false;
                    bool sameV2 = false;

                    pi.ParametersOnS1(out u1, out v1);
                    pk.ParametersOnS1(out u2, out v2);
                    if (Math.Abs(u1 - u2) <= tol2D)
                    {
                        sameU1 = true;
                    }
                    if (Math.Abs(v1 - v2) <= tol2D)
                    {
                        sameV1 = true;
                    }

                    pi.ParametersOnS2(out u1, out v1);
                    pk.ParametersOnS2(out u2, out v2);
                    if (Math.Abs(u1 - u2) <= tol2D)
                    {
                        sameU2 = true;
                    }
                    if (Math.Abs(v1 - v2) <= tol2D)
                    {
                        sameV2 = true;
                    }

                    if ((sameU1 && sameV1) && (sameU2 && sameV2))
                    {
                        vrts.SetValue(k, -1);
                    }
                }
            }
        }

        // copy further processed vertices
        for (i = 1; i <= nbVrt; i++)
        {
            if (vrts.Value(i) == -1)
            {
                continue;
            }
            vertices.Add(pLine.Vertex(i).PntOn2S());
        }
        return vertices;
    }

    private static void SearchVertices(LineOn2S line, LineOn2S vertices, NCollection_Array1<int> pTypes)
    {
        int nbp = line.PointCount, nbv = vertices.PointCount;
        for (int ip = 1; ip <= nbp; ip++)
        {
            PntOn2S p = line.Value(ip);
            int type = 0;
            for (int iv = 1; iv <= nbv; iv++)
            {
                PntOn2S v = vertices.Value(iv);
                if (p.IsSame(v, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.PConfusion()))
                {
                    type = iv;
                    break;
                }
            }
            pTypes.SetValue(ip, type);
        }
    }

    private static bool IsSeamParameter(double u, double tol2D) =>
        (Math.Abs(u) <= tol2D || Math.Abs(2.0 * M_PI - u) <= tol2D);

    private static double AdjustU(double U)
    {
        double u = U, dblpi = 2.0 * M_PI;
        if (u < 0.0 || u > dblpi)
        {
            if (u < 0.0)
            {
                while (u < 0.0)
                {
                    u += dblpi;
                }
            }
            else
            {
                while (u > dblpi)
                {
                    u -= dblpi;
                }
            }
        }
        return u;
    }

    private static void Correct2DBounds(double uf, double ul, double vf, double vl, double tol2D, ref double u,
                                        ref double v)
    {
        double eps = 1.0e-16;
        double dUF = Math.Abs(u - uf);
        double dUL = Math.Abs(u - ul);
        double dVF = Math.Abs(v - vf);
        double dVL = Math.Abs(v - vl);
        if (dUF <= tol2D && dUF > eps)
        {
            u = uf;
        }
        if (dUL <= tol2D && dUL > eps)
        {
            u = ul;
        }
        if (dVF <= tol2D && dVF > eps)
        {
            v = vf;
        }
        if (dVL <= tol2D && dVL > eps)
        {
            v = vl;
        }
    }

    private static void AdjustLine(LineOn2S line, bool isReversed, SurfaceAdaptorBase qSurf, double tol2D)
    {
        double vf = qSurf.FirstVParameter();
        double vl = qSurf.LastVParameter();
        double uf = qSurf.FirstUParameter();
        double ul = qSurf.LastUParameter();

        int nbp = line.PointCount;
        double u, v;
        for (int ip = 1; ip <= nbp; ip++)
        {
            if (isReversed)
            {
                line.Value(ip).ParametersOnS2(out u, out v);
                u = AdjustU(u);
                Correct2DBounds(uf, ul, vf, vl, tol2D, ref u, ref v);
                line.SetUV(ip, false, u, v);
            }
            else
            {
                line.Value(ip).ParametersOnS1(out u, out v);
                u = AdjustU(u);
                Correct2DBounds(uf, ul, vf, vl, tol2D, ref u, ref v);
                line.SetUV(ip, true, u, v);
            }
        }
    }

    private static bool InsertSeamVertices(LineOn2S line, bool isReversed, LineOn2S vertices,
                                           NCollection_Array1<int> pTypes, double tol2D)
    {
        bool result = false;
        int nbp = line.PointCount;
        double U, v;
        for (int ip = 1; ip <= nbp; ip++)
        {
            int ipt = pTypes.Value(ip);
            if (ipt != 0)
            {
                PntOn2S p = line.Value(ip);
                if (isReversed)
                {
                    p.ParametersOnS2(out U, out v); // S2 - quadric
                }
                else
                {
                    p.ParametersOnS1(out U, out v); // S1 - quadric
                }
                U = AdjustU(U);
                if (IsSeamParameter(U, tol2D))
                {
                    if (ip == 1 || ip == nbp)
                    {
                        double u1, v1;
                        int ipp = (ip == 1) ? (ip + 1) : (ip - 1);
                        if (isReversed)
                        {
                            line.Value(ipp).ParametersOnS2(out u1, out v1); // S2 - quadric
                        }
                        else
                        {
                            line.Value(ipp).ParametersOnS1(out u1, out v1); // S1 - quadric
                        }
                        double u = AdjustUFirst(U, u1);
                        if (Math.Abs(u - U) >= 1.5 * M_PI)
                        {
                            double u2, v2;
                            if (isReversed)
                            {
                                line.Value(ip).ParametersOnS1(out u2, out v2); // prm
                                line.SetUV(ip, false, u, v);
                                line.SetUV(ip, true, u2, v2);
                            }
                            else
                            {
                                line.Value(ip).ParametersOnS2(out u2, out v2); // prm
                                line.SetUV(ip, true, u, v);
                                line.SetUV(ip, false, u2, v2);
                            }
                        }
                    }
                    else
                    {
                        int ipp = ip - 1;
                        int ipn = ip + 1;
                        double u1, v1, u2, v2;
                        if (isReversed)
                        {
                            line.Value(ipp).ParametersOnS2(out u1, out v1); // quad
                            line.Value(ipn).ParametersOnS2(out u2, out v2); // quad
                        }
                        else
                        {
                            line.Value(ipp).ParametersOnS1(out u1, out v1); // quad
                            line.Value(ipn).ParametersOnS1(out u2, out v2); // quad
                        }
                        u1 = AdjustU(u1);
                        u2 = AdjustU(u2);
                        bool pnearZero = Math.Abs(u1) < Math.Abs(2.0 * M_PI - u1);
                        bool cnearZero = Math.Abs(U) < Math.Abs(2.0 * M_PI - U);
                        if (pnearZero == cnearZero)
                        {
                            if (!IsSeamParameter(u2, tol2D) && !IsSeamParameter(u1, tol2D))
                            {
                                double nU = (cnearZero) ? (2.0 * M_PI) : 0.0;
                                PntOn2S nP = new();
                                nP.SetValue(p.Value());
                                double u3, v3;
                                if (isReversed)
                                {
                                    line.Value(ip).ParametersOnS1(out u3, out v3); // prm
                                    nP.SetValue(false, nU, v);
                                    nP.SetValue(true, u3, v3);
                                }
                                else
                                {
                                    line.Value(ip).ParametersOnS2(out u3, out v3); // prm
                                    nP.SetValue(true, nU, v);
                                    nP.SetValue(false, u3, v3);
                                }
                                line.InsertBefore(ipn, nP);
                                vertices.Add(nP);
                                result = true;
                                break;
                            }
                        }
                        else
                        {
                            if (!IsSeamParameter(u2, tol2D) && !IsSeamParameter(u1, tol2D))
                            {
                                double nU = (cnearZero) ? (2.0 * M_PI) : 0.0;
                                PntOn2S nP = new();
                                nP.SetValue(p.Value());
                                double u3, v3;
                                if (isReversed)
                                {
                                    line.Value(ip).ParametersOnS1(out u3, out v3); // prm
                                    nP.SetValue(false, nU, v);
                                    nP.SetValue(true, u3, v3);
                                }
                                else
                                {
                                    line.Value(ip).ParametersOnS2(out u3, out v3); // prm
                                    nP.SetValue(true, nU, v);
                                    nP.SetValue(false, u3, v3);
                                }
                                line.InsertBefore(ip, nP);
                                vertices.Add(nP);
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

    private static void ToSmooth(LineOn2S line, bool isReversed, in SurfaceQuadric quad, bool isFirst,
                                 ref double d3D)
    {
        if (line.PointCount <= 10)
        {
            return;
        }

        d3D = 0.0;
        int nbTestPnts = line.PointCount / 5;
        if (nbTestPnts < 5)
        {
            nbTestPnts = 5;
        }

        int startp = (isFirst) ? 2 : (line.PointCount - nbTestPnts - 2);
        double uc, vc, un, vn, ddu = 0.0;

        // OCCT compares ip with NbTestPnts, not startp + NbTestPnts: for the last points (IsFirst false) the loop
        // runs only on short lines, and D3D and DDU stay 0 otherwise (reproduced).
        for (int ip = startp; ip <= nbTestPnts; ip++)
        {
            if (isReversed)
            {
                line.Value(ip).ParametersOnS2(out uc, out vc); // S2 - quadric
                line.Value(ip + 1).ParametersOnS2(out un, out vn);
            }
            else
            {
                line.Value(ip).ParametersOnS1(out uc, out vc); // S1 - quadric
                line.Value(ip + 1).ParametersOnS1(out un, out vn);
            }
            ddu += Math.Abs(Math.Abs(uc) - Math.Abs(un));

            if (ip > startp)
            {
                double dp = line.Value(ip).Value().DistanceTo(line.Value(ip - 1).Value());
                d3D += dp;
            }
        }

        ddu /= (double)nbTestPnts + 1;

        d3D /= (double)nbTestPnts + 1;

        int index1 = (isFirst) ? 1 : (line.PointCount);
        int index2 = (isFirst) ? 2 : (line.PointCount - 1);
        int index3 = (isFirst) ? 3 : (line.PointCount - 2);

        bool doU = false;

        double u1, u2, v1, v2, u3, v3;

        if (isReversed)
        {
            line.Value(index1).ParametersOnS2(out u1, out v1); // S2 - quadric
            line.Value(index2).ParametersOnS2(out u2, out v2);
            line.Value(index3).ParametersOnS2(out u3, out v3);
        }
        else
        {
            line.Value(index1).ParametersOnS1(out u1, out v1); // S1 - quadric
            line.Value(index2).ParametersOnS1(out u2, out v2);
            line.Value(index3).ParametersOnS1(out u3, out v3);
        }

        if (!doU && quad.TypeQuadric() == SurfaceKind.Sphere)
        {
            if (Math.Abs(Math.Abs(u1) - Math.Abs(u2)) > (M_PI / 16.0))
            {
                doU = true;
            }

            if (doU && (Math.Abs(u1) <= 1.0e-9 || Math.Abs(u1 - 2.0 * M_PI) <= 1.0e-9))
            {
                if (Math.Abs(v1 - M_PI / 2.0) <= 1.0e-9 || Math.Abs(v1 + M_PI / 2.0) <= 1.0e-9)
                {
                }
                else
                {
                    doU = false;
                }
            }
        }

        if (quad.TypeQuadric() == SurfaceKind.Cone)
        {
            double uapx = 0.0, vapx = 0.0;
            quad.Parameters(quad.Cone().Apex, ref uapx, ref vapx);

            if (Math.Abs(Math.Abs(u1) - Math.Abs(u2)) > M_PI / 32.0)
            {
                doU = true;
            }

            if (doU && (Math.Abs(u1) <= 1.0e-9 || Math.Abs(u1 - 2.0 * M_PI) <= 1.0e-9))
            {
                if (Math.Abs(v1 - vapx) <= 1.0e-9)
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
            double dU = Math.Min((ddu / 10.0), 5.0e-8);
            double u = (u2 > u3) ? (u2 + dU) : (u2 - dU);
            if (isReversed)
            {
                line.SetUV(index1, false, u, v1);
            }
            else
            {
                line.SetUV(index1, true, u, v1);
            }
        }
    }

    private static bool TestMiddleOnPrm(in PntOn2S aP, in PntOn2S aV, bool isReversed, double arcTol,
                                        TopolTool pDomain)
    {
        bool result = false;
        double up, vp, uv, vv;
        if (isReversed)
        {
            aP.ParametersOnS1(out up, out vp); // S1 - parametric
            aV.ParametersOnS1(out uv, out vv);
        }
        else
        {
            aP.ParametersOnS2(out up, out vp); // S2 - parametric
            aV.ParametersOnS2(out uv, out vv);
        }
        double um = (up + uv) * 0.5, vm = (vp + vv) * 0.5;
        Point2D a2DPntM = new(um, vm);
        Containment posM = pDomain.Classify(a2DPntM, arcTol);
        if (posM == On || posM == In)
        {
            result = true;
        }
        return result;
    }

    private static void VerifyVertices(LineOn2S line, bool isReversed, LineOn2S vertices, double tol2D,
                                       double arcTol, TopolTool pDomain, ref PntOn2S vrtF,
                                       out bool addFirst, ref PntOn2S vrtL, out bool addLast)
    {
        int nbp = line.PointCount, nbv = vertices.PointCount;
        int fIndexSame = 0, fIndexNear = 0, lIndexSame = 0, lIndexNear = 0;
        PntOn2S pf = line.Value(1);
        PntOn2S pl = line.Value(nbp);
        double uf, vf, ul, vl;
        if (isReversed)
        {
            pf.ParametersOnS2(out uf, out vf);
            pl.ParametersOnS2(out ul, out vl);
        }
        else
        {
            pf.ParametersOnS1(out uf, out vf);
            pl.ParametersOnS1(out ul, out vl);
        }
        Point2D a2DPF = new(uf, vf);
        Point2D a2DPL = new(ul, vl);
        double distMinF = 1.0e+100, distMinL = 1.0e+100;
        int fConjugated = 0, lConjugated = 0;

        int iv;

        for (iv = 1; iv <= nbv; iv++)
        {
            double uv, vv;
            if (isReversed)
            {
                vertices.Value(iv).ParametersOnS2(out uv, out vv);
                uv = AdjustU(uv);
                vertices.SetUV(iv, false, uv, vv);
            }
            else
            {
                vertices.Value(iv).ParametersOnS1(out uv, out vv);
                uv = AdjustU(uv);
                vertices.SetUV(iv, true, uv, vv);
            }
        }

        for (iv = 1; iv <= nbv; iv++)
        {
            PntOn2S v = vertices.Value(iv);
            if (pf.IsSame(v, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                fIndexSame = iv;
                break;
            }
            else
            {
                double uv, vv;
                if (isReversed)
                {
                    v.ParametersOnS2(out uv, out vv);
                }
                else
                {
                    v.ParametersOnS1(out uv, out vv);
                }
                Point2D a2DV = new(uv, vv);
                double dist = a2DV.DistanceTo(a2DPF);
                if (dist < distMinF)
                {
                    distMinF = dist;
                    fIndexNear = iv;
                    if (fConjugated != 0)
                    {
                        fConjugated = 0;
                    }
                }
                if (IsSeamParameter(uv, tol2D))
                {
                    double ucv = (Math.Abs(uv) < Math.Abs(2.0 * M_PI - uv)) ? (2.0 * M_PI) : 0.0;
                    Point2D a2DCV = new(ucv, vv);
                    double cDist = a2DCV.DistanceTo(a2DPF);
                    if (cDist < distMinF)
                    {
                        distMinF = cDist;
                        fConjugated = iv;
                        fIndexNear = iv;
                    }
                }
            }
        }

        for (iv = 1; iv <= nbv; iv++)
        {
            PntOn2S v = vertices.Value(iv);
            if (pl.IsSame(v, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.PConfusion()))
            {
                lIndexSame = iv;
                break;
            }
            else
            {
                double uv, vv;
                if (isReversed)
                {
                    v.ParametersOnS2(out uv, out vv);
                }
                else
                {
                    v.ParametersOnS1(out uv, out vv);
                }
                Point2D a2DV = new(uv, vv);
                double dist = a2DV.DistanceTo(a2DPL);
                if (dist < distMinL)
                {
                    distMinL = dist;
                    lIndexNear = iv;
                    if (lConjugated != 0)
                    {
                        lConjugated = 0;
                    }
                }
                if (IsSeamParameter(uv, tol2D))
                {
                    double ucv = (Math.Abs(uv) < Math.Abs(2.0 * M_PI - uv)) ? (2.0 * M_PI) : 0.0;
                    Point2D a2DCV = new(ucv, vv);
                    double cDist = a2DCV.DistanceTo(a2DPL);
                    if (cDist < distMinL)
                    {
                        distMinL = cDist;
                        lConjugated = iv;
                        lIndexNear = iv;
                    }
                }
            }
        }

        addFirst = false;
        addLast = false;

        if (fIndexSame == 0)
        {
            if (fIndexNear != 0)
            {
                PntOn2S v = vertices.Value(fIndexNear);
                double uv, vv;
                if (isReversed)
                {
                    v.ParametersOnS2(out uv, out vv);
                }
                else
                {
                    v.ParametersOnS1(out uv, out vv);
                }
                if (IsSeamParameter(uv, tol2D))
                {
                    double ucv = (Math.Abs(uv) < Math.Abs(2.0 * M_PI - uv)) ? (2.0 * M_PI) : 0.0;
                    bool test = TestMiddleOnPrm(pf, v, isReversed, arcTol, pDomain);
                    if (test)
                    {
                        vrtF.SetValue(v.Value());
                        if (isReversed)
                        {
                            v.ParametersOnS1(out double u2, out double v2); // S1 - prm
                            vrtF.SetValue(true, u2, v2);
                            if (fConjugated == 0)
                            {
                                vrtF.SetValue(false, uv, vv);
                            }
                            else
                            {
                                vrtF.SetValue(false, ucv, vv);
                            }
                        }
                        else
                        {
                            v.ParametersOnS2(out double u2, out double v2); // S2 - prm
                            vrtF.SetValue(false, u2, v2);
                            if (fConjugated == 0)
                            {
                                vrtF.SetValue(true, uv, vv);
                            }
                            else
                            {
                                vrtF.SetValue(true, ucv, vv);
                            }
                        }
                        double dist3D = vrtF.Value().DistanceTo(pf.Value());
                        if (dist3D > 1.5e-7 && distMinF > tol2D)
                        {
                            addFirst = true;
                        }
                    }
                }
                else
                {
                    // to do: analyze internal vertex
                }
            }
        }

        if (lIndexSame == 0)
        {
            if (lIndexNear != 0)
            {
                PntOn2S v = vertices.Value(lIndexNear);
                double uv, vv;
                if (isReversed)
                {
                    v.ParametersOnS2(out uv, out vv);
                }
                else
                {
                    v.ParametersOnS1(out uv, out vv);
                }
                if (IsSeamParameter(uv, tol2D))
                {
                    double ucv = (Math.Abs(uv) < Math.Abs(2.0 * M_PI - uv)) ? (2.0 * M_PI) : 0.0;
                    bool test = TestMiddleOnPrm(pl, v, isReversed, arcTol, pDomain);
                    if (test)
                    {
                        vrtL.SetValue(v.Value());
                        if (isReversed)
                        {
                            v.ParametersOnS1(out double u2, out double v2); // S1 - prm
                            vrtL.SetValue(true, u2, v2);
                            if (lConjugated == 0)
                            {
                                vrtL.SetValue(false, uv, vv);
                            }
                            else
                            {
                                vrtL.SetValue(false, ucv, vv);
                            }
                        }
                        else
                        {
                            v.ParametersOnS2(out double u2, out double v2); // S2 - prm
                            vrtL.SetValue(false, u2, v2);
                            if (lConjugated == 0)
                            {
                                vrtL.SetValue(true, uv, vv);
                            }
                            else
                            {
                                vrtL.SetValue(true, ucv, vv);
                            }
                        }
                        double dist3D = vrtL.Value().DistanceTo(pl.Value());
                        if (dist3D > 1.5e-7 && distMinL > tol2D)
                        {
                            addLast = true;
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

    private static bool AddVertices(LineOn2S line, in PntOn2S vrtF, bool addFirst,
                                    in PntOn2S vrtL, bool addLast, double d3DF, double d3DL)
    {
        bool result = false;
        if (addFirst)
        {
            double df = line.Value(1).Value().DistanceTo(vrtF.Value());
            if ((d3DF * 2.0) > df && df > 1.5e-7)
            {
                line.InsertBefore(1, vrtF);
                result = true;
            }
        }
        if (addLast)
        {
            double dl = line.Value(line.PointCount).Value().DistanceTo(vrtL.Value());
            if ((d3DL * 2.0) > dl && dl > 1.5e-7)
            {
                line.Add(vrtL);
                result = true;
            }
        }
        return result;
    }

    private static void PutIntVertices(PointLine line, LineOn2S result, bool isReversed,
                                       LineOn2S vertices, double arcTol)
    {
        int nbp = result.PointCount, nbv = vertices.PointCount;

        if (nbp < 3)
        {
            return;
        }

        RLine? rLine = line as RLine;

        Point3D pnt;
        PatchPoint thePnt = new();

        for (int ip = 2; ip <= (nbp - 1); ip++)
        {
            PntOn2S p = result.Value(ip);
            for (int iv = 1; iv <= nbv; iv++)
            {
                PntOn2S v = vertices.Value(iv);
                if (p.IsSame(v, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.PConfusion()))
                {
                    pnt = result.Value(ip).Value();
                    result.Value(ip).ParametersOnS1(out double u1, out double v1);
                    result.Value(ip).ParametersOnS2(out double u2, out double v2);
                    thePnt.SetValue(pnt, arcTol, false);
                    thePnt.SetParameters(u1, v1, u2, v2);

                    double param = (double)ip;

                    if (rLine is not null)
                    {
                        // In fact, aRLine is always on the parametric surface.
                        // If (theIsReversed == TRUE) then (U1, V1) - point on
                        // parametric surface, otherwise - point on quadric.
                        Curve2DAdaptorBase arc = (rLine.IsArcOnS1 ? rLine.ArcOnS1() : rLine.ArcOnS2())!;

                        Line2D lin = arc.Line();
                        Point2D pSurf;

                        if (isReversed)
                        {
                            pSurf = new Point2D(u1, v1);
                        }
                        else
                        {
                            pSurf = new Point2D(u2, v2);
                        }

                        param = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, pSurf);
                    }

                    thePnt.SetParameter(param);
                    line.AddVertex(thePnt);
                }
            }
        }
    }

    private static bool HasInternals(LineOn2S line, LineOn2S vertices)
    {
        int nbp = line.PointCount, nbv = vertices.PointCount;
        bool result = false;

        if (nbp < 3)
        {
            return result;
        }

        for (int ip = 2; ip <= (nbp - 1); ip++)
        {
            PntOn2S p = line.Value(ip);
            for (int iv = 1; iv <= nbv; iv++)
            {
                PntOn2S v = vertices.Value(iv);
                if (p.IsSame(v, global::Kerf.Engine.Foundation.Precision.Confusion(), global::Kerf.Engine.Foundation.Precision.PConfusion()))
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

    private static WLine MakeSplitWLine(WLine wLine, bool tang, WalkingTypeTrans trans1,
                                                 WalkingTypeTrans trans2, double arcTol, int parFirst, int parLast)
    {
        LineOn2S sLine = wLine.Curve();
        LineOn2S sline = new();

        for (int ip = parFirst; ip <= parLast; ip++)
        {
            sline.Add(sLine.Value(ip));
        }

        WLine wline = new(sline, tang, trans1, trans2);
        wline.SetCreatingWayInfo(global::Kerf.Engine.Intersection.Patches.WLine.IntPatch_WLType.WLImpPrm);

        Point3D sPnt;
        PatchPoint tPntF = new(), tPntL = new();
        double uu1, vv1, uu2, vv2;

        sPnt = sline.Value(1).Value();
        sline.Value(1).ParametersOnS1(out uu1, out vv1);
        sline.Value(1).ParametersOnS2(out uu2, out vv2);
        tPntF.SetValue(sPnt, arcTol, false);
        tPntF.SetParameters(uu1, vv1, uu2, vv2);
        tPntF.SetParameter(1.0);
        wline.AddVertex(tPntF);
        wline.SetFirstPoint(1);

        sPnt = sline.Value(sline.PointCount).Value();
        sline.Value(sline.PointCount).ParametersOnS1(out uu1, out vv1);
        sline.Value(sline.PointCount).ParametersOnS2(out uu2, out vv2);
        tPntL.SetValue(sPnt, arcTol, false);
        tPntL.SetParameters(uu1, vv1, uu2, vv2);
        tPntL.SetParameter((double)sline.PointCount);
        wline.AddVertex(tPntL);
        wline.SetLastPoint(wline.VertexCount);

        return wline;
    }

    private static bool SplitOnSegments(WLine wLine, bool tang, WalkingTypeTrans trans1,
                                        WalkingTypeTrans trans2, double arcTol,
                                        NCollection_Sequence<PatchLine> segments)
    {
        bool result = false;
        segments.Clear();

        int nbv = wLine.VertexCount;
        if (nbv > 3)
        {
            for (int iv = 1; iv < nbv; iv++)
            {
                int firstPar = (int)wLine.Vertex(iv).ParameterOnLine();
                int lastPar = (int)wLine.Vertex(iv + 1).ParameterOnLine();
                if ((lastPar - firstPar) <= 1)
                {
                    continue;
                }
                else
                {
                    WLine splitwline = MakeSplitWLine(wLine, tang, trans1, trans2, arcTol, firstPar, lastPar);
                    segments.Append(splitwline);
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
    private static bool IsPointOnBoundary(double toler2D, double boundary, double period, double param)
    {
        double delta = Math.Abs(param - boundary);
        if (period != 0.0)
        {
            delta = delta % period; // fmod

            // 0 <= aDelta < thePeriod
            return ((delta < toler2D) || ((period - delta) < toler2D));
        }

        // Here, thePeriod == 0.0, aDelta > 0.0

        return (delta < toler2D);
    }

    // Can change values of theNewLine (by adding the computed point on boundary, which parameter will be adjusted)
    // and theIsOnBoundary variables.
    private static void DetectOfBoundaryAchievement(SurfaceAdaptorBase qSurf, bool isReversed,
                                                    LineOn2S sourceLine, int pointIndex,
                                                    double toler2D, LineOn2S newLine,
                                                    ref bool isOnBoundary)
    {
        double uPeriod = qSurf.IsUPeriodic ? qSurf.UPeriod() : 0.0,
               vPeriod = qSurf.IsVPeriodic ? qSurf.VPeriod() : 0.0;
        double uf = qSurf.FirstUParameter(), ul = qSurf.LastUParameter(), vf = qSurf.FirstVParameter(),
               vl = qSurf.LastVParameter();

        PntOn2S pPrev = sourceLine.Value(pointIndex - 1), pCurr = sourceLine.Value(pointIndex);
        double uPrev, vPrev, uCurr, vCurr;
        if (isReversed)
        {
            pPrev.ParametersOnS2(out uPrev, out vPrev); // S2 - quadric, set U,V by Pnt3D
            pCurr.ParametersOnS2(out uCurr, out vCurr); // S2 - quadric, set U,V by Pnt3D
        }
        else
        {
            pPrev.ParametersOnS1(out uPrev, out vPrev); // S1 - quadric, set U,V by Pnt3D
            pCurr.ParametersOnS1(out uCurr, out vCurr); // S1 - quadric, set U,V by Pnt3D
        }

        // Ignore cases when the WLine goes along the surface boundary completely.

        if (IsPointOnBoundary(toler2D, uf, uPeriod, uCurr) && !IsPointOnBoundary(toler2D, uf, uPeriod, uPrev))
        {
            isOnBoundary = true;
        }
        else if (IsPointOnBoundary(toler2D, ul, uPeriod, uCurr)
                 && !IsPointOnBoundary(toler2D, ul, uPeriod, uPrev))
        {
            isOnBoundary = true;
        }
        else if (IsPointOnBoundary(toler2D, vf, vPeriod, vCurr)
                 && !IsPointOnBoundary(toler2D, vf, vPeriod, vPrev))
        {
            isOnBoundary = true;
        }
        else if (IsPointOnBoundary(toler2D, vl, vPeriod, vCurr)
                 && !IsPointOnBoundary(toler2D, vl, vPeriod, vPrev))
        {
            isOnBoundary = true;
        }

        if (isOnBoundary)
        {
            // Adjust, to avoid bad jumping of the WLine.

            double du = (uPrev - uCurr);
            double dv = (vPrev - vCurr);
            if (uPeriod > 0.0 && (2.0 * Math.Abs(du) > uPeriod))
            {
                uCurr += Math.CopySign(uPeriod, du);
            }

            if (vPeriod > 0.0 && (2.0 * Math.Abs(dv) > vPeriod))
            {
                vCurr += Math.CopySign(vPeriod, dv);
            }

            PntOn2S point = pCurr;
            point.SetValue(!isReversed, uCurr, vCurr);
            newLine.Add(point);
        }
    }

    // Splits theLine in the places where it passes through seam edge or singularity (apex of cone or pole of
    // sphere). This passage is detected by jump of U-parameter from point to point.
    private static bool DecomposeResult(PointLine line, bool isReversed, in SurfaceQuadric quad,
                                        TopolTool pDomain, SurfaceAdaptorBase qSurf,
                                        SurfaceAdaptorBase pSurf, double arcTol, double tolTang,
                                        NCollection_Sequence<PatchLine> lines)
    {
        if (line.ArcType() == PatchLineType.Restriction)
        {
            if (line is RLine rl)
            {
                Curve2DAdaptorBase arc = (rl.IsArcOnS1 ? rl.ArcOnS1() : rl.ArcOnS2())!;
                if (arc.GetType() != CurveKind.Line)
                {
                    // Restriction line must be isoline.
                    // Other cases are not supported by
                    // existing algorithms.

                    return false;
                }
            }
        }

        double deltaUmax = M_PI_2;
        double tol3D = 1.0e-10, tol2D = global::Kerf.Engine.Foundation.Precision.PConfusion(), tol2DS = global::Kerf.Engine.Foundation.Precision.PConfusion();

        LineOn2S sLine = line.Curve()!;

        if (sLine.PointCount <= 2)
        {
            return false;
        }

        // Deletes repeated vertices
        LineOn2S vLine = GetVertices(line, tol3D, tol2D);

        // OCCT's aSSLine is a second handle to the line's own points: they are edited in place.
        LineOn2S ssLine = sLine;

        if (ssLine.PointCount <= 1)
        {
            return false;
        }

        AdjustLine(ssLine, isReversed, qSurf, tol2D);

        if (line.ArcType() == PatchLineType.Walking)
        {
            bool isInserted = true;
            while (isInserted)
            {
                int nbPnts = ssLine.PointCount;
                NCollection_Array1<int> pTypes = new(1, nbPnts);
                SearchVertices(ssLine, vLine, pTypes);
                isInserted = InsertSeamVertices(ssLine, isReversed, vLine, pTypes, tol2D);
            }
        }

        int lindex = ssLine.PointCount;
        int findex = 1, bindex = 0;

        // build WLine parts (if any)
        bool flNextLine = true;
        bool hasBeenDecomposed = false;
        SpecPntType prePointExist = SpecPntType.SPntNone;

        PntOn2S prePoint = new();
        while (flNextLine)
        {
            // reset variables
            flNextLine = false;
            bool isDecomposited = false;

            LineOn2S sline = new();

            // if((Lindex-Findex+1) <= 2 )
            if ((lindex <= findex) && prePointExist == SpecPntType.SPntNone)
            {
                // break of "while(flNextLine)" cycle
                break;
            }

            if (prePointExist != SpecPntType.SPntNone)
            {
                PntOn2S refPt = ssLine.Value(findex);

                double uRes = qSurf.UResolution(arcTol), vRes = qSurf.VResolution(arcTol);

                double tol2d = (prePointExist == SpecPntType.SPntPole)     ? -1.0
                                : (prePointExist == SpecPntType.SPntSeamV)  ? vRes
                                : (prePointExist == SpecPntType.SPntSeamUV) ? Math.Max(uRes, vRes)
                                                                          : uRes;

                if (SpecialPoints.ContinueAfterSpecialPoint(qSurf, pSurf, refPt, prePointExist, tol2d,
                                                                     ref prePoint, isReversed))
                {
                    sline.Add(prePoint);

                    // Avoid adding duplicate points.
                    for (; findex <= lindex; findex++)
                    {
                        if (!prePoint.IsSame(ssLine.Value(findex), tolTang))
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

            prePointExist = SpecPntType.SPntNone;

            // analyze other points
            for (int k = findex; k <= lindex; k++)
            {
                if (k == findex)
                {
                    prePoint = ssLine.Value(k);
                    sline.Add(prePoint);
                    continue;
                }

                // Check whether the current point is on the boundary of theQSurf.
                // If that is TRUE then the Walking-line will be decomposed in this point.
                // However, this boundary is not singular-point (like seam or pole of sphere).
                // Therefore, its processing will be simplified.
                bool isOnBoundary = false;

                // Values of sline and isOnBoundary can be changed by this function
                DetectOfBoundaryAchievement(qSurf, isReversed, ssLine, k, tol2D, sline, ref isOnBoundary);

                prePointExist = IsSeamOrPole(qSurf, ssLine, isReversed, k - 1, tolTang, deltaUmax);

                if (isOnBoundary && (prePointExist != SpecPntType.SPntPoleSeamU))
                {
                    // If the considered point is on seam then its UV-parameters
                    // are defined to within the surface period. Therefore, we can
                    // trust already computed parameters of this point.
                    // However, if this point (which is on the surface boundary) is
                    // a sphere pole or cone apex then its (point's) parameters
                    // have to be recomputed in the code below
                    // (see IntPatch_SpecialPoints::AddSingularPole() method).
                    // E.g. see "bugs modalg_6 bug26684_2" test case.

                    prePointExist = SpecPntType.SPntNone;
                }

                if (prePointExist != SpecPntType.SPntNone)
                {
                    bindex = k;
                    isDecomposited = true;
                    ////
                    PntOn2S refPt = ssLine.Value(bindex - 1);

                    double compareTol3D = global::Kerf.Engine.Foundation.Precision.Confusion();
                    double compareTol2D = global::Kerf.Engine.Foundation.Precision.PConfusion();

                    PntOn2S newPoint = refPt;
                    SpecPntType lastType = SpecPntType.SPntNone;

                    if (prePointExist == SpecPntType.SPntSeamUV)
                    {
                        prePointExist = SpecPntType.SPntNone;
                        lastType = SpecPntType.SPntSeamUV;
                        SpecialPoints.AddCrossUVIsoPoint(qSurf, pSurf, refPt, tolTang, ref newPoint,
                                                                  isReversed);
                    }
                    else if (prePointExist == SpecPntType.SPntSeamV)
                    { // WLine goes through seam
                        prePointExist = SpecPntType.SPntNone;
                        lastType = SpecPntType.SPntSeamV;

                        // Not quadric point
                        double u0, v0;
                        // Quadric point
                        double uQuadRef, vQuadRef;

                        if (isReversed)
                        {
                            refPt.Parameters(out u0, out v0, out uQuadRef, out vQuadRef);
                        }
                        else
                        {
                            refPt.Parameters(out uQuadRef, out vQuadRef, out u0, out v0);
                        }

                        math_Vector tol = new(1, 3), startPoint = new(1, 3), infBound = new(1, 3),
                                    supBound = new(1, 3);

                        // Parameters on parametric surface
                        double up, vp, uq, vq;
                        if (isReversed)
                        {
                            ssLine.Value(k).Parameters(out up, out vp, out uq, out vq);
                        }
                        else
                        {
                            ssLine.Value(k).Parameters(out uq, out vq, out up, out vp);
                        }

                        tol[1] = pSurf.UResolution(arcTol);
                        tol[2] = pSurf.VResolution(arcTol);
                        tol[3] = qSurf.UResolution(arcTol);
                        startPoint[1] = 0.5 * (u0 + up);
                        startPoint[2] = 0.5 * (v0 + vp);
                        startPoint[3] = 0.5 * (uQuadRef + uq);
                        infBound[1] = pSurf.FirstUParameter();
                        infBound[2] = pSurf.FirstVParameter();
                        infBound[3] = qSurf.FirstUParameter();
                        supBound[1] = pSurf.LastUParameter();
                        supBound[2] = pSurf.LastVParameter();
                        supBound[3] = qSurf.LastUParameter();

                        SpecialPoints.AddPointOnUorVIso(qSurf, pSurf, refPt, false, 0.0, tol, startPoint,
                                                                 infBound, supBound, ref newPoint, isReversed);
                    }
                    else if (prePointExist == SpecPntType.SPntPoleSeamU)
                    {
                        prePointExist = SpecPntType.SPntNone;

                        PatchPoint vert = new();
                        vert.SetValue(refPt);
                        vert.SetTolerance(tolTang);

                        if (SpecialPoints.AddSingularPole(qSurf, pSurf, refPt, ref vert, ref newPoint,
                                                                   isReversed))
                        {
                            prePointExist = SpecPntType.SPntPole;
                            lastType = SpecPntType.SPntPole;
                            if (isOnBoundary)
                            {
                                // It is necessary to replace earlier added point on
                                // the surface boundary with the pole. For that,
                                // here we delete excess point. New point will be added later.
                                isOnBoundary = false;
                                sline.RemovePoint(sline.PointCount);
                            }

                            compareTol2D = -1.0;
                        } // if(IntPatch_AddSpecialPoints::AddSingularPole(...))
                        else
                        { // Pole is not an intersection point
                            prePointExist = SpecPntType.SPntSeamU;
                        }
                    }

                    if (prePointExist == SpecPntType.SPntSeamU)
                    { // WLine goes through seam
                        prePointExist = SpecPntType.SPntNone;
                        lastType = SpecPntType.SPntSeamU;

                        // Not quadric point
                        double u0, v0;
                        // Quadric point
                        double uQuadRef, vQuadRef;

                        if (isReversed)
                        {
                            refPt.Parameters(out u0, out v0, out uQuadRef, out vQuadRef);
                        }
                        else
                        {
                            refPt.Parameters(out uQuadRef, out vQuadRef, out u0, out v0);
                        }

                        math_Vector tol = new(1, 3), startPoint = new(1, 3), infBound = new(1, 3),
                                    supBound = new(1, 3);

                        // Parameters on parametric surface
                        double up, vp, uq, vq;
                        if (isReversed)
                        {
                            ssLine.Value(k).Parameters(out up, out vp, out uq, out vq);
                        }
                        else
                        {
                            ssLine.Value(k).Parameters(out uq, out vq, out up, out vp);
                        }

                        tol[1] = pSurf.UResolution(arcTol);
                        tol[2] = pSurf.VResolution(arcTol);
                        tol[3] = qSurf.VResolution(arcTol);
                        startPoint[1] = 0.5 * (u0 + up);
                        startPoint[2] = 0.5 * (v0 + vp);
                        startPoint[3] = 0.5 * (vQuadRef + vq);
                        infBound[1] = pSurf.FirstUParameter();
                        infBound[2] = pSurf.FirstVParameter();
                        infBound[3] = qSurf.FirstVParameter();
                        supBound[1] = pSurf.LastUParameter();
                        supBound[2] = pSurf.LastVParameter();
                        supBound[3] = qSurf.LastVParameter();

                        SpecialPoints.AddPointOnUorVIso(qSurf, pSurf, refPt, true, 0.0, tol, startPoint,
                                                                 infBound, supBound, ref newPoint, isReversed);
                    }

                    if (!newPoint.IsSame(refPt, compareTol3D, compareTol2D))
                    {
                        if (isOnBoundary)
                        {
                            break;
                        }

                        sline.Add(newPoint);
                        prePointExist = lastType;
                        prePoint = newPoint;
                    }
                    else
                    {
                        if (isOnBoundary || (sline.PointCount == 1))
                        {
                            // FIRST point of the sline is the pole of the quadric.
                            // Therefore, there is no point in decomposition.

                            // If the considered point is on surface boundary then
                            // it is already marked as vertex. So, decomposition is
                            // not required, too.

                            prePoint = refPt;
                            prePointExist = isOnBoundary ? SpecPntType.SPntNone : lastType;
                        }
                    }

                    ////
                    break;
                } // if (aPrePointExist != IntPatch_SPntNone) cond.

                prePoint = ssLine.Value(k);

                if (isOnBoundary)
                {
                    bindex = k;
                    isDecomposited = true;
                    prePointExist = SpecPntType.SPntNone;
                    break;
                }
                else
                {
                    sline.Add(ssLine.Value(k));
                }
            } // for(int k = aFindex; k <= aLindex; k++)

            // Creation of new line as part of existing theLine.
            // This part is defined by sline.

            if (sline.PointCount == 1)
            {
                flNextLine = true;

                if (findex < bindex)
                {
                    findex = bindex;
                }

                // Go to the next part of aSSLine
                // because we cannot create the line
                // with single point.

                continue;
            }

            PntOn2S vf = new(), vl = new();
            VerifyVertices(sline, isReversed, vLine, tol2DS, arcTol, pDomain, ref vf, out bool addVF, ref vl,
                           out bool addVL);

            bool hasInternals = HasInternals(sline, vLine);

            double d3F = 0.0, d3L = 0.0;
            ToSmooth(sline, isReversed, quad, true, ref d3F);
            ToSmooth(sline, isReversed, quad, false, ref d3L);

            if (addVF || addVL)
            {
                bool isAdded = AddVertices(sline, vf, addVF, vl, addVL, d3F, d3L);
                if (isAdded)
                {
                    ToSmooth(sline, isReversed, quad, true, ref d3F);
                    ToSmooth(sline, isReversed, quad, false, ref d3L);
                }
            }

            if (line.ArcType() == PatchLineType.Walking)
            {
                PatchPoint tPntF = new(), tPntL = new();

                WLine wline = new(sline, false, line.TransitionOnS1(), line.TransitionOnS2());
                wline.SetCreatingWayInfo(WLine.IntPatch_WLType.WLImpPrm);

                Point3D sPnt = sline.Value(1).Value();
                sline.Value(1).Parameters(out double u1, out double v1, out double u2, out double v2);
                tPntF.SetValue(sPnt, arcTol, false);
                tPntF.SetParameters(u1, v1, u2, v2);
                tPntF.SetParameter(1.0);
                wline.AddVertex(tPntF);
                wline.SetFirstPoint(1);

                if (hasInternals)
                {
                    PutIntVertices(wline, sline, isReversed, vLine, arcTol);
                }

                sPnt = sline.Value(sline.PointCount).Value();
                sline.Value(sline.PointCount).Parameters(out u1, out v1, out u2, out v2);
                tPntL.SetValue(sPnt, arcTol, false);
                tPntL.SetParameters(u1, v1, u2, v2);
                tPntL.SetParameter(sline.PointCount);
                wline.AddVertex(tPntL);
                wline.SetLastPoint(wline.VertexCount);

                NCollection_Sequence<PatchLine> segm = new();
                bool isSplited = SplitOnSegments(wline, false, line.TransitionOnS1(), line.TransitionOnS2(),
                                                 arcTol, segm);

                if (!isSplited)
                {
                    lines.Append(wline);
                }
                else
                {
                    int nbsegms = segm.Length();
                    for (int iseg = 1; iseg <= nbsegms; iseg++)
                    {
                        lines.Append(segm.Value(iseg));
                    }
                }
            }
            else
            { // theLine->ArcType() == IntPatch_Restriction
                if (!isDecomposited && !hasBeenDecomposed)
                {
                    // The line has not been changed
                    lines.Append((RLine)line);
                    return hasBeenDecomposed;
                }

                PatchPoint tPnt = new();
                Point2D aPSurf;
                Point3D sPnt;

                RLine rLine = new((RLine)line);

                rLine.ClearVertexes();
                rLine.SetCurve(sline);

                if (hasInternals)
                {
                    PutIntVertices(rLine, sline, isReversed, vLine, arcTol);
                }

                Curve2DAdaptorBase arc = (rLine.IsArcOnS1 ? rLine.ArcOnS1() : rLine.ArcOnS2())!;

                double fPar = arc.FirstParameter(), lPar = arc.LastParameter();

                PntOn2S rFirst = sline.Value(1), rLast = sline.Value(sline.PointCount);

                Line2D lin = arc.Line();

                for (int flIndex = 0; flIndex < 2; flIndex++)
                {
                    double u1, v1, u2, v2;
                    if (flIndex == 0)
                    {
                        rFirst.Parameters(out u1, out v1, out u2, out v2);
                        sPnt = rFirst.Value();
                    }
                    else
                    {
                        rLast.Parameters(out u1, out v1, out u2, out v2);
                        sPnt = rLast.Value();
                    }

                    if (isReversed)
                    {
                        aPSurf = new Point2D(u1, v1);
                    }
                    else
                    {
                        aPSurf = new Point2D(u2, v2);
                    }

                    double par = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin, aPSurf);

                    if (flIndex == 0)
                    {
                        fPar = Math.Max(fPar, par);
                        par = fPar;
                    }
                    else
                    {
                        lPar = Math.Min(lPar, par);
                        par = lPar;
                    }

                    tPnt.SetParameter(par);
                    tPnt.SetValue(sPnt, arcTol, false);
                    tPnt.SetParameters(u1, v1, u2, v2);

                    rLine.AddVertex(tPnt);
                }

                if (lPar - fPar > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    rLine.SetFirstPoint(1);
                    rLine.SetLastPoint(rLine.VertexCount);

                    // OCCT: Trim returns a new adaptor, which is discarded; the arc itself is not trimmed (reproduced).
                    _ = arc.Trim(fPar, lPar, arcTol);

                    lines.Append(rLine);
                }
            }

            if (isDecomposited)
            {
                findex = bindex;
                flNextLine = hasBeenDecomposed = true;
            }
        }

        return hasBeenDecomposed;
    }

    // Returns TRUE if the segment [theParF, theParL] is included in the segment [theRefParF, theRefParL] segment.
    private static bool CheckSegmSegm(double refParF, double refParL, double parF, double parL)
    {
        if ((parF < refParF) || (parF > refParL))
        {
            return false;
        }

        if ((parL < refParF) || (parL > refParL))
        {
            return false;
        }

        return true;
    }

    // Check, if theLine is coincided with theArc (in 2d-space). Cases when theArc is not 2d-line adaptor are
    // supported by TopOpeBRep classes only (i.e. are archaic).
    private static bool IsCoincide(TheSurfFunction func, PointLine line,
                                   Curve2DAdaptorBase arc, bool isTheSurface1Using, double toler3D,
                                   double toler2D, double period)
    {
        ReadOnlySpan<double> coeffs =
        [
            0.02447174185, 0.09549150281, 0.20610737385, 0.34549150281, /*Sin(x)*Sin(x)*/
            0.5, 0.65450849719, 0.79389262615,
        ];
        if (line.ArcType() == PatchLineType.Restriction)
        { // Restriction-restriction processing
            RLine rl2 = (RLine)line;
            Curve2DAdaptorBase anArc = (rl2.IsArcOnS1 ? rl2.ArcOnS1() : rl2.ArcOnS2())!;

            if (anArc.GetType() != CurveKind.Line)
            {
                // Restriction line must be isoline.
                // Other cases are not supported by
                // existing algorithms.

                return false;
            }

            Line2D lin1 = arc.Line(), lin2 = anArc.Line();

            if (!lin1.Direction.IsParallel(lin2.Direction, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                return false;
            }

            double dist = arc.Line().DistanceTo(anArc.Line());
            if ((dist < toler2D) || (Math.Abs(dist - period) < toler2D))
            {
                double rf = arc.FirstParameter(), rl = arc.LastParameter();
                double parf = anArc.FirstParameter(), parl = anArc.LastParameter();
                Point2D p1 = global::Kerf.Engine.Geometry.ElCLib.Value(parf, lin2), p2 = global::Kerf.Engine.Geometry.ElCLib.Value(parl, lin2);

                double param1 = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin1, p1), param2 = global::Kerf.Engine.Geometry.ElCLib.Parameter(lin1, p2);

                if (CheckSegmSegm(rf, rl, param1, param2))
                {
                    return true;
                }

                // Lines are parallel. Therefore, there is no point in
                // projecting points to another line in order to check
                // if segment second line is included in segment of first one.

                return CheckSegmSegm(param1, param2, rf, rl);
            }

            return false;
        }

        int nbPnts = line.NbPnts();
        double uAf = arc.FirstParameter(), uAl = arc.LastParameter();
        Line2D arcLin = arc.Line();

        math_Vector x = new(1, 2), val = new(1, 1);

        for (int ptID = 1; ptID <= nbPnts; ptID++)
        {
            double uf, vf;
            if (isTheSurface1Using)
            {
                line.Point(ptID).ParametersOnS1(out uf, out vf);
            }
            else
            {
                line.Point(ptID).ParametersOnS2(out uf, out vf);
            }

            // Take 2d-point in parametric surface (because theArc is
            // 2d-line in parametric surface).
            Point2D ploc = new(uf, vf);

            double rParam = global::Kerf.Engine.Geometry.ElCLib.Parameter(arcLin, ploc);

            if ((rParam < uAf) || (rParam > uAl))
            {
                return false;
            }

            Point2D pmin = global::Kerf.Engine.Geometry.ElCLib.Value(rParam, arcLin);

            double dist = ploc.DistanceTo(pmin);
            if ((dist < toler2D) || (Math.Abs(dist - period) < toler2D))
            { // Considered point is in Restriction line.
                // Go to the next point.
                continue;
            }

            // Check if intermediate points between aPloc and theArc are
            // intersection point (i.e. if aPloc is in tangent zone between
            // two intersected surfaces).

            double ul = pmin.X, vl = pmin.Y;

            double u, v;
            double dU = ul - uf, dV = vl - vf;
            for (int i = 0; i < 7; i++)
            {
                u = uf + coeffs[i] * dU;
                v = vf + coeffs[i] * dV;

                x[1] = u;
                x[2] = v;

                if (!func.Value(x, val))
                {
                    return false;
                }

                if (Math.Abs(func.Root()) > toler3D)
                {
                    return false;
                }
            }
        }

        return true;
    }
}
