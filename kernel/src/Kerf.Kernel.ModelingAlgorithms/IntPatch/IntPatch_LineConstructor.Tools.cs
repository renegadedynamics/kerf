// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_LineConstructor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Engine.Intersection.Walking.WalkingSituation;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Intersection.Patches;

// The file-static functions of IntPatch_LineConstructor.cxx.
internal partial class PatchLineConstructor
{
    private static void Recadre(SurfaceAdaptorBase myHS1, SurfaceAdaptorBase myHS2, ref double u1, ref double v1,
                                ref double u2, ref double v2)
    {
        double f, l, lmf;
        SurfaceKind typs1 = myHS1.GetType();
        SurfaceKind typs2 = myHS2.GetType();

        bool myHS1IsUPeriodic, myHS1IsVPeriodic;
        switch (typs1)
        {
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            {
                myHS1IsUPeriodic = true;
                myHS1IsVPeriodic = false;
                break;
            }
            case SurfaceKind.Torus:
            {
                myHS1IsUPeriodic = myHS1IsVPeriodic = true;
                break;
            }
            default:
            {
                //-- Le cas de biparametrees periodiques est gere en amont
                myHS1IsUPeriodic = myHS1IsVPeriodic = false;
                break;
            }
        }

        bool myHS2IsUPeriodic, myHS2IsVPeriodic;
        switch (typs2)
        {
            case SurfaceKind.Cylinder:
            case SurfaceKind.Cone:
            case SurfaceKind.Sphere:
            {
                myHS2IsUPeriodic = true;
                myHS2IsVPeriodic = false;
                break;
            }
            case SurfaceKind.Torus:
            {
                myHS2IsUPeriodic = myHS2IsVPeriodic = true;
                break;
            }
            default:
            {
                //-- Le cas de biparametrees periodiques est gere en amont
                myHS2IsUPeriodic = myHS2IsVPeriodic = false;
                break;
            }
        }
        if (myHS1IsUPeriodic)
        {
            lmf = M_PI + M_PI; //-- myHS1->UPeriod();
            f = myHS1.FirstUParameter();
            l = myHS1.LastUParameter();
            while (u1 < f)
            {
                u1 += lmf;
            }
            while (u1 > l)
            {
                u1 -= lmf;
            }
        }
        if (myHS1IsVPeriodic)
        {
            lmf = M_PI + M_PI; //-- myHS1->VPeriod();
            f = myHS1.FirstVParameter();
            l = myHS1.LastVParameter();
            while (v1 < f)
            {
                v1 += lmf;
            }
            while (v1 > l)
            {
                v1 -= lmf;
            }
        }
        if (myHS2IsUPeriodic)
        {
            lmf = M_PI + M_PI; //-- myHS2->UPeriod();
            f = myHS2.FirstUParameter();
            l = myHS2.LastUParameter();
            while (u2 < f)
            {
                u2 += lmf;
            }
            while (u2 > l)
            {
                u2 -= lmf;
            }
        }
        if (myHS2IsVPeriodic)
        {
            lmf = M_PI + M_PI; //-- myHS2->VPeriod();
            f = myHS2.FirstVParameter();
            l = myHS2.LastVParameter();
            while (v2 < f)
            {
                v2 += lmf;
            }
            while (v2 > l)
            {
                v2 -= lmf;
            }
        }
    }

    private static void Parameters(SurfaceAdaptorBase myHS1, SurfaceAdaptorBase myHS2, in Point3D ptref, ref double u1,
                                   ref double v1, ref double u2, ref double v2)
    {
        SurfaceQuadric quad1 = new SurfaceQuadric(), quad2 = new SurfaceQuadric();
        SurfaceKind typs = myHS1.GetType();
        switch (typs)
        {
            case Plane:
                quad1.SetValue(myHS1.Plane());
                break;
            case SurfaceKind.Cylinder:
                quad1.SetValue(myHS1.Cylinder());
                break;
            case SurfaceKind.Cone:
                quad1.SetValue(myHS1.Cone());
                break;
            case SurfaceKind.Sphere:
                quad1.SetValue(myHS1.Sphere());
                break;
            case SurfaceKind.Torus:
                quad1.SetValue(myHS1.Torus());
                break;
            default:
                throw new ConstructionErrorException("IntPatch_IntSS::MakeCurve");
        }

        typs = myHS2.GetType();
        switch (typs)
        {
            case Plane:
                quad2.SetValue(myHS2.Plane());
                break;
            case SurfaceKind.Cylinder:
                quad2.SetValue(myHS2.Cylinder());
                break;
            case SurfaceKind.Cone:
                quad2.SetValue(myHS2.Cone());
                break;
            case SurfaceKind.Sphere:
                quad2.SetValue(myHS2.Sphere());
                break;
            case SurfaceKind.Torus:
                quad2.SetValue(myHS2.Torus());
                break;
            default:
                throw new ConstructionErrorException("IntPatch_IntSS::MakeCurve");
        }
        quad1.Parameters(ptref, ref u1, ref v1);
        quad2.Parameters(ptref, ref u2, ref v2);
    }

    private static double LocalFirstParameter(PatchLine l)
    {
        double firstp = 0.0;
        PatchLineType typl = l.ArcType();
        switch (typl)
        {
            case PatchLineType.Analytic:
            {
                ALine alin = (ALine)l;
                if (alin.HasFirstPoint)
                {
                    firstp = alin.FirstPoint().ParameterOnLine();
                }
                else
                {
                    firstp = alin.FirstParameter(out bool included);
                    if (!included)
                    {
                        firstp += Epsilon(firstp);
                    }
                }
                return firstp;
            }

            case PatchLineType.Restriction:
            {
                RLine rlin = (RLine)l;
                if (rlin.HasFirstPoint)
                {
                    firstp = rlin.FirstPoint().ParameterOnLine();
                }
                else
                {
                    firstp = -global::Kerf.Engine.Foundation.Precision.Infinite(); // a voir selon le type de la ligne 2d
                }
                return firstp;
            }
            case PatchLineType.Walking:
            {
                WLine wlin = (WLine)l;
                if (wlin.HasFirstPoint)
                {
                    firstp = wlin.FirstPoint().ParameterOnLine();
                }
                else
                {
                    firstp = 1.0;
                }
                return firstp;
            }

            default:
            {
                GLine glin = (GLine)l;
                if (glin.HasFirstPoint)
                {
                    firstp = glin.FirstPoint().ParameterOnLine();
                }
                else
                {
                    switch (typl)
                    {
                        case PatchLineType.Lin:
                        case PatchLineType.Parabola:
                        case PatchLineType.Hyperbola:
                            firstp = -global::Kerf.Engine.Foundation.Precision.Infinite();
                            break;

                        case PatchLineType.Circle:
                        case PatchLineType.Ellipse:
                            firstp = 0.0;
                            break;
                        default:
                        {
                            break;
                        }
                    }
                }
                return firstp;
            }
        }
    }

    private static double LocalLastParameter(PatchLine l)
    {
        double lastp = 0.0;
        PatchLineType typl = l.ArcType();
        switch (typl)
        {
            case PatchLineType.Analytic:
            {
                ALine alin = (ALine)l;

                if (alin.HasLastPoint)
                {
                    lastp = alin.LastPoint().ParameterOnLine();
                }
                else
                {
                    lastp = alin.LastParameter(out bool included);
                    if (!included)
                    {
                        lastp -= Epsilon(lastp);
                    }
                }
                return lastp;
            }

            case PatchLineType.Restriction:
            {
                RLine rlin = (RLine)l;

                if (rlin.HasLastPoint)
                {
                    lastp = rlin.LastPoint().ParameterOnLine();
                }
                else
                {
                    lastp = global::Kerf.Engine.Foundation.Precision.Infinite(); // a voir selon le type de la ligne 2d
                }
                return lastp;
            }
            case PatchLineType.Walking:
            {
                WLine wlin = (WLine)l;

                if (wlin.HasLastPoint)
                {
                    lastp = wlin.LastPoint().ParameterOnLine();
                }
                else
                {
                    lastp = wlin.NbPnts();
                }
                return lastp;
            }

            default:
            {
                GLine glin = (GLine)l;

                if (glin.HasLastPoint)
                {
                    lastp = glin.LastPoint().ParameterOnLine();
                }
                else
                {
                    switch (typl)
                    {
                        case PatchLineType.Lin:
                        case PatchLineType.Parabola:
                        case PatchLineType.Hyperbola:
                            lastp = global::Kerf.Engine.Foundation.Precision.Infinite();
                            break;

                        case PatchLineType.Circle:
                        case PatchLineType.Ellipse:
                            lastp = M_PI + M_PI;
                            break;
                        default:
                        {
                            break;
                        }
                    }
                }
                return lastp;
            }
        }
    }

    // modified by NIZHNY-MKK  Tue Apr  3 15:03:06 2001.BEGIN
    private static double ComputeParametricTolerance(double tol3d, in Vector3D d1u, in Vector3D d1v)
    {
        double nad1u = d1u.Magnitude;
        double nad1v = d1v.Magnitude;
        double tolu = 0.0, tolv = 0.0;
        if (nad1u > 1e-12)
        {
            tolu = tol3d / nad1u;
        }
        else
        {
            tolu = 0.1;
        }
        if (nad1v > 1e-12)
        {
            tolv = tol3d / nad1v;
        }
        else
        {
            tolv = 0.1;
        }
        double tolerance = (tolu > tolv) ? tolu : tolv;
        return tolerance;
    }
    // modified by NIZHNY-MKK  Tue Apr  3 15:03:11 2001.END

    private static int AppendSameVertexA(ALine alig, ALine l, int index, int[] tabIndex)
    {
        int i, a, n;
        a = 0;
        n = l.VertexCount;
        PatchPoint vtxindex = l.Vertex(index);
        double thetol1 = vtxindex.Tolerance();
        for (i = 1; i <= n; i++)
        {
            if (i != index)
            {
                PatchPoint vtxi = l.Vertex(i);
                double thetol2 = vtxi.Tolerance();
                if (thetol2 < thetol1)
                {
                    thetol2 = thetol1;
                }
                double d_4 = vtxindex.Value().DistanceTo(vtxi.Value());
                if (d_4 <= thetol2)
                {
                    alig.AddVertex(vtxi);
                    a++;
                    tabIndex[i] = tabIndex[index];
                }
            }
        }
        return (a);
    }

    private static int AppendSameVertexG(GLine glig, GLine l, int index, double decal,
                                         int[] tabIndex)
    {
        int i, a, n;
        double p1, p2, d;
        bool aajouter;
        a = 0;
        n = l.VertexCount;
        PatchPoint vtxindex = l.Vertex(index);
        double thetol1 = vtxindex.Tolerance();
        for (i = 1; i <= n; i++)
        {
            if (i != index)
            {
                PatchPoint vtxi = l.Vertex(i);
                aajouter = false;
                double thetol2 = vtxi.Tolerance();
                if (thetol2 < thetol1)
                {
                    thetol2 = thetol1;
                }
                d = vtxindex.Value().DistanceTo(vtxi.Value());
                if (d <= thetol2)
                {
                    aajouter = true;
                }

                //-- Le test suivant a ete ajoute le 20 aout 98 (??? mefiance ???)
                else
                {
                    p1 = vtxindex.ParameterOnLine();
                    p2 = vtxi.ParameterOnLine();
                    if (Math.Abs(p1 - p2) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        aajouter = true;
                    }
                }
                if (aajouter)
                {
                    p1 = vtxindex.ParameterOnLine();
                    PatchPoint vtx = vtxi;
                    vtx.SetParameter(p1 + decal);
                    glig.AddVertex(vtx);
                    a++;
                    tabIndex[i] = tabIndex[index];
                }
            }
        }
        return (a);
    }

    private static int AppendSameVertexW(WLine wlig, WLine l, int index, double par,
                                         int[] tabIndex)
    {
        int i, a, n;
        a = 0;
        n = l.VertexCount;
        PatchPoint vtxindex = l.Vertex(index);
        Point3D pntindex = vtxindex.Value();
        double thetol1 = vtxindex.Tolerance();
        for (i = 1; i <= n; i++)
        {
            if (i != index)
            {
                PatchPoint vtxi = l.Vertex(i);
                double d_2 = pntindex.DistanceTo(vtxi.Value());
                double thetol2 = vtxi.Tolerance();
                if (thetol2 < thetol1)
                {
                    thetol2 = thetol1;
                }
                //-- le debugger voit 2 fois la variable d ici. ???? -> d_2
                if (d_2 <= thetol2)
                {
                    vtxi.SetParameter(par);
                    vtxindex.ParametersOnS1(out double u1, out double v1);
                    vtxindex.ParametersOnS2(out double u2, out double v2);
                    vtxi.SetParameters(u1, v1, u2, v2);
                    vtxi.SetValue(pntindex);
                    wlig.AddVertex(vtxi);
                    a++;
                    tabIndex[i] = tabIndex[index];
                }
            }
        }
        return (a);
    }

    private static int AppendSameVertexR(RLine rlig, RLine l, int index, int[] tabIndex)
    {
        int i, a, n;
        a = 0;
        n = l.VertexCount;
        PatchPoint vtxindex = l.Vertex(index);
        double thetol1 = vtxindex.Tolerance();
        for (i = 1; i <= n; i++)
        {
            if (i != index)
            {
                PatchPoint vtxi = l.Vertex(i);
                double d_3 = vtxindex.Value().DistanceTo(vtxi.Value());
                double thetol2 = vtxi.Tolerance();
                if (thetol2 < thetol1)
                {
                    thetol2 = thetol1;
                }
                if (d_3 < thetol2)
                {
                    if (vtxi.ParameterOnLine() != vtxindex.ParameterOnLine())
                    {
                        PatchPoint vtxicop = l.Vertex(i);
                        vtxicop.SetParameter(vtxindex.ParameterOnLine());
                        rlig.AddVertex(vtxicop);
                    }
                    else
                    {
                        rlig.AddVertex(vtxi);
                    }
                    a++;
                    tabIndex[i] = tabIndex[index];
                }
            }
        }
        return (a);
    }

    // OCCT's two GeomAbs_SurfaceType arguments are unused.
    private static void AddLine(PatchLine l, int i, int j, SurfaceKind typeS1, SurfaceKind typeS2,
                                int[] tabIndex, NCollection_Sequence<PatchLine> slin)
    {
        int indexFirstVertex = 1;
        int indexLastVertex = 2;
        if (i == j)
        {
            indexLastVertex = 1;
        }
        PatchLineType typl = l.ArcType();
        switch (typl)
        {
            case PatchLineType.Analytic:
            {
                ALine aLine = (ALine)l;
                ALine alig;
                if (l.TransitionOnS1() == WalkingTypeTrans.Undecided)
                {
                    alig = new ALine(aLine.Curve(), l.IsTangent);
                }
                else if (l.TransitionOnS1() == WalkingTypeTrans.Touch)
                {
                    alig = new ALine(aLine.Curve(), l.IsTangent, l.SituationS1(), l.SituationS2());
                }
                else
                {
                    alig = new ALine(aLine.Curve(), l.IsTangent, l.TransitionOnS1(), l.TransitionOnS2());
                }
                alig.AddVertex(aLine.Vertex(i));
                indexLastVertex += AppendSameVertexA(alig, aLine, i, tabIndex);
                if (i != j)
                {
                    alig.AddVertex(aLine.Vertex(j));
                    indexLastVertex += AppendSameVertexA(alig, aLine, j, tabIndex);
                }
                alig.SetFirstPoint(indexFirstVertex);
                alig.SetLastPoint(indexLastVertex);
                slin.Append(alig);
                break;
            }
            case PatchLineType.Walking:
            { //-- ****************************************
                WLine wLine = (WLine)l;
                LineOn2S lori = wLine.Curve();
                LineOn2S lineOn2S = new LineOn2S();
                int paramMinOnLine = (int)wLine.Vertex(i).ParameterOnLine();
                int paramMaxOnLine = (int)wLine.Vertex(j).ParameterOnLine();
                for (int k = paramMinOnLine; k <= paramMaxOnLine; k++)
                {
                    lineOn2S.Add(lori.Value(k));
                }
                WLine wlig;
                if (l.TransitionOnS1() == WalkingTypeTrans.Undecided)
                {
                    wlig = new WLine(lineOn2S, l.IsTangent);
                }
                else if (l.TransitionOnS1() == WalkingTypeTrans.Touch)
                {
                    wlig = new WLine(lineOn2S, l.IsTangent, l.SituationS1(), l.SituationS2());
                }
                else
                {
                    wlig = new WLine(lineOn2S, l.IsTangent, l.TransitionOnS1(), l.TransitionOnS2());
                }
                if (wLine.HasArcOnS1)
                {
                    wlig.SetArcOnS1(wLine.GetArcOnS1()!);
                }
                if (wLine.HasArcOnS2)
                {
                    wlig.SetArcOnS2(wLine.GetArcOnS2()!);
                }
                PatchPoint vtx = wLine.Vertex(i);
                vtx.SetParameter(1);
                wlig.AddVertex(vtx);
                indexLastVertex += AppendSameVertexW(wlig, wLine, i, 1, tabIndex);
                if (i != j)
                {
                    vtx = wLine.Vertex(j);
                    vtx.SetParameter(lineOn2S.PointCount);
                    wlig.AddVertex(vtx);
                    indexLastVertex += AppendSameVertexW(wlig, wLine, j, lineOn2S.PointCount, tabIndex);
                }
                wlig.SetFirstPoint(indexFirstVertex);
                wlig.SetLastPoint(indexLastVertex);
                wlig.SetPeriod(wLine.U1Period(), wLine.V1Period(), wLine.U2Period(), wLine.V2Period());
                wlig.ComputeVertexParameters(global::Kerf.Engine.Foundation.Precision.Confusion());
                slin.Append(wlig);
                //-- **********************************************************************

                break;
            }
            case PatchLineType.Restriction:
            {
                RLine rLine = (RLine)l;
                indexLastVertex = 2;
                indexFirstVertex = 1;
                RLine rlig;
                if (l.TransitionOnS1() == WalkingTypeTrans.Undecided)
                {
                    rlig = new RLine(l.IsTangent);
                }
                else if (l.TransitionOnS1() == WalkingTypeTrans.Touch)
                {
                    rlig = new RLine(l.IsTangent, l.SituationS1(), l.SituationS2());
                }
                else
                {
                    rlig = new RLine(l.IsTangent, l.TransitionOnS1(), l.TransitionOnS2());
                }
                if (rLine.IsArcOnS1)
                {
                    rlig.SetArcOnS1(rLine.ArcOnS1()!);
                }
                if (rLine.IsArcOnS2)
                {
                    rlig.SetArcOnS2(rLine.ArcOnS2()!);
                }

                rlig.AddVertex(rLine.Vertex(i));
                // XPU1009
                indexLastVertex += AppendSameVertexR(rlig, rLine, i, tabIndex);
                for (int k = i + 1; k < j; k++)
                {
                    rlig.AddVertex(rLine.Vertex(k));
                    indexLastVertex++;
                }
                if (i != j)
                {
                    rlig.AddVertex(rLine.Vertex(j));
                    // XPU1009
                    indexLastVertex += AppendSameVertexR(rlig, rLine, j, tabIndex);
                }
                rlig.SetFirstPoint(indexFirstVertex);
                rlig.SetLastPoint(indexLastVertex);
                rlig.ComputeVertexParameters(global::Kerf.Engine.Foundation.Precision.Confusion());
                slin.Append(rlig);
                break;
            }
            case PatchLineType.Lin:
            case PatchLineType.Parabola:
            case PatchLineType.Hyperbola:
            case PatchLineType.Circle:
            case PatchLineType.Ellipse:
            {
                GLine gLine = (GLine)l;
                GLine glig;
                switch (typl)
                {
                    case PatchLineType.Lin:
                        if (l.TransitionOnS1() == WalkingTypeTrans.Undecided)
                        {
                            glig = new GLine(gLine.Line(), l.IsTangent);
                        }
                        else if (l.TransitionOnS1() == WalkingTypeTrans.Touch)
                        {
                            glig = new GLine(gLine.Line(), l.IsTangent, l.SituationS1(), l.SituationS2());
                        }
                        else
                        {
                            glig = new GLine(gLine.Line(), l.IsTangent, l.TransitionOnS1(),
                                                      l.TransitionOnS2());
                        }
                        break;
                    case PatchLineType.Parabola:
                        if (l.TransitionOnS1() == WalkingTypeTrans.Undecided)
                        {
                            glig = new GLine(gLine.Parabola(), l.IsTangent);
                        }
                        else if (l.TransitionOnS1() == WalkingTypeTrans.Touch)
                        {
                            glig = new GLine(gLine.Parabola(), l.IsTangent, l.SituationS1(),
                                                      l.SituationS2());
                        }
                        else
                        {
                            glig = new GLine(gLine.Parabola(), l.IsTangent, l.TransitionOnS1(),
                                                      l.TransitionOnS2());
                        }
                        break;
                    case PatchLineType.Hyperbola:
                        if (l.TransitionOnS1() == WalkingTypeTrans.Undecided)
                        {
                            glig = new GLine(gLine.Hyperbola(), l.IsTangent);
                        }
                        else if (l.TransitionOnS1() == WalkingTypeTrans.Touch)
                        {
                            glig = new GLine(gLine.Hyperbola(), l.IsTangent, l.SituationS1(),
                                                      l.SituationS2());
                        }
                        else
                        {
                            glig = new GLine(gLine.Hyperbola(), l.IsTangent, l.TransitionOnS1(),
                                                      l.TransitionOnS2());
                        }
                        break;
                    case PatchLineType.Circle:
                        if (l.TransitionOnS1() == WalkingTypeTrans.Undecided)
                        {
                            glig = new GLine(gLine.Circle(), l.IsTangent);
                        }
                        else if (l.TransitionOnS1() == WalkingTypeTrans.Touch)
                        {
                            glig = new GLine(gLine.Circle(), l.IsTangent, l.SituationS1(),
                                                      l.SituationS2());
                        }
                        else
                        {
                            glig = new GLine(gLine.Circle(), l.IsTangent, l.TransitionOnS1(),
                                                      l.TransitionOnS2());
                        }
                        break;
                    case PatchLineType.Ellipse:
                    default:
                        if (l.TransitionOnS1() == WalkingTypeTrans.Undecided)
                        {
                            glig = new GLine(gLine.Ellipse(), l.IsTangent);
                        }
                        else if (l.TransitionOnS1() == WalkingTypeTrans.Touch)
                        {
                            glig = new GLine(gLine.Ellipse(), l.IsTangent, l.SituationS1(),
                                                      l.SituationS2());
                        }
                        else
                        {
                            glig = new GLine(gLine.Ellipse(), l.IsTangent, l.TransitionOnS1(),
                                                      l.TransitionOnS2());
                        }
                        break;
                }
                glig.AddVertex(gLine.Vertex(i));
                indexLastVertex += AppendSameVertexG(glig, gLine, i, 0, tabIndex);
                if (i != j)
                {
                    if ((typl == PatchLineType.Circle || typl == PatchLineType.Ellipse) && i > j)
                    {
                        PatchPoint vtx = gLine.Vertex(j);
                        vtx.SetParameter(gLine.Vertex(j).ParameterOnLine() + M_PI + M_PI);
                        glig.AddVertex(vtx);
                        indexLastVertex += AppendSameVertexG(glig, gLine, j, M_PI + M_PI, tabIndex);
                    }
                    else
                    {
                        glig.AddVertex(gLine.Vertex(j));
                        indexLastVertex += AppendSameVertexG(glig, gLine, j, 0, tabIndex);
                    }
                }
                glig.SetFirstPoint(indexFirstVertex);
                glig.SetLastPoint(indexLastVertex);
                slin.Append(glig);
                break;
            }
            default:
            {
                throw new ConstructionErrorException("IntPatch_LineConstructor::AddLine");
            }
        }
    }

    private static double GetVertexTolerance(in PatchPoint vtx)
    {
        double tol = vtx.Tolerance();
        return tol;
    }

    // OCCT's TolArc argument is commented out.
    private static bool IsSegmentSmall(WLine wLine, int ivFirst, int ivLast)
    {
        PatchPoint vtxF = wLine.Vertex(ivFirst);
        PatchPoint vtxL = wLine.Vertex(ivLast);
        int ipF = (int)vtxF.ParameterOnLine();
        int ipL = (int)vtxL.ParameterOnLine();
        if (ipF >= ipL)
        {
            return true;
        }

        double tolF = GetVertexTolerance(vtxF);
        double tolL = GetVertexTolerance(vtxL);
        double tol = Max(tolF, tolL);

        double len = 0.0;
        Point3D p1 = wLine.Point(ipF).Value();
        for (int i = ipF + 1; i <= ipL; i++)
        {
            Point3D p2 = wLine.Point(i).Value();
            len += p1.DistanceTo(p2);
            if (len > tol)
            {
                break;
            }
            p1 = p2;
        }
        return len <= tol;
    }

    private static bool TestWLineIsARLine(NCollection_Sequence<PatchLine> slinref, WLine wlin,
                                          double tol2d)
    {
        int nbpnt = wlin.NbPnts();
        int indicepnt = nbpnt / 2;
        if (indicepnt < 1)
        {
            return (false);
        }
        PntOn2S pOn2S = wlin.Point(indicepnt);
        PntOn2S pOn2S1 = wlin.Point(indicepnt + 1);
        int lastl = slinref.Length();
        for (int i = 1; i <= lastl; i++)
        {
            if (slinref.Value(i).ArcType() == PatchLineType.Restriction)
            {
                RLine rlin = (RLine)slinref.Value(i);
                for (int @is = 0; @is < 2; @is++)
                {
                    bool onFirst = @is == 0;
                    if ((onFirst && rlin.IsArcOnS1) || (!onFirst && rlin.IsArcOnS2))
                    {
                        Curve2DAdaptorBase arc;
                        double u, v, u1, v1;
                        if (onFirst)
                        {
                            arc = rlin.ArcOnS1()!;
                            pOn2S.ParametersOnS1(out u, out v);
                            pOn2S1.ParametersOnS1(out u1, out v1);
                        }
                        else
                        {
                            arc = rlin.ArcOnS2()!;
                            pOn2S.ParametersOnS2(out u, out v);
                            pOn2S1.ParametersOnS2(out u1, out v1);
                        }
                        if (indicepnt == 1)
                        {
                            u = (u + u1) * 0.5;
                            v = (v + v1) * 0.5;
                        }
                        Curve2DAdaptorBase c2d = arc;
                        Point2D pObt, p2d = new Point2D(u, v);
                        double par = TheProjPCurOfGInter.FindParameter(c2d, p2d, 1e-7);
                        pObt = c2d.Value(par);
                        if (pObt.DistanceTo(p2d) < tol2d)
                        {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    private static bool TestIfWLineIsRestriction(NCollection_Sequence<PatchLine> slinref, WLine wlin,
                                                 SurfaceAdaptorBase s1, TopolTool d1, SurfaceAdaptorBase s2,
                                                 TopolTool d2, double tolArc)
    {
        int nbPnts = wlin.NbPnts();
        int allon1 = 0, allon2 = 0, i;
        double tol2d1 = 0.0, tol2d2 = 0.0;
        Point3D ap = new Point3D();
        Vector3D ad1u = new Vector3D(), ad1v = new Vector3D();
        for (i = 1; i <= nbPnts; i++)
        {
            PntOn2S pmid = wlin.Point(i);
            pmid.Parameters(out double u1, out double v1, out double u2, out double v2);
            //-- Estimation d un majorant de Toluv a partir de Tol
            double tol;
            //------------------------------------------
            s1.D1(u1, v1, out ap, out ad1u, out ad1v);
            tol = ComputeParametricTolerance(tolArc, ad1u, ad1v);
            if (tol > tol2d1)
            {
                tol2d1 = tol;
            }
            //--
            if (allon1 + 1 == i && d1.IsThePointOn(new Point2D(u1, v1), tol))
            {
                allon1++;
            }
            //------------------------------------------
            s2.D1(u2, v2, out ap, out ad1u, out ad1v);
            tol = ComputeParametricTolerance(tolArc, ad1u, ad1v);
            if (tol > tol2d2)
            {
                tol2d2 = tol;
            }
            //--
            if (allon2 + 1 == i && d2.IsThePointOn(new Point2D(u2, v2), tol))
            {
                allon2++;
            }
            if (allon1 != i && allon2 != i)
            {
                break;
            }
        }
        if (allon1 == nbPnts || allon2 == nbPnts)
        {
            double tol2d = Max(tol2d1, tol2d2);
            return TestWLineIsARLine(slinref, wlin, tol2d);
        }
        return false;
    }

    private static bool ProjectOnArc(double u, double v, Curve2DAdaptorBase arc, SurfaceAdaptorBase surf, double tolArc,
                                     out double par, out double dist)
    {
        Point3D pbid = new Point3D();
        Vector3D ad1u = new Vector3D(), ad1v = new Vector3D();
        surf.D1(u, v, out pbid, out ad1u, out ad1v);
        double tol2d = ComputeParametricTolerance(tolArc, ad1u, ad1v);
        Curve2DAdaptorBase c2d = arc;
        Point2D p = new Point2D(u, v), pprj;
        par = TheProjPCurOfGInter.FindParameter(c2d, p, 1e-7);
        pprj = c2d.Value(par);
        dist = pprj.DistanceTo(p);
        return dist <= tol2d;
    }

    // OCCT's pointers to member functions (piParOnS, pIsOnDomS, pArcOnS, pParOnArc) are these, chosen by onFirst.
    private static void ParOnS(bool onFirst, in PntOn2S p, out double u, out double v)
    {
        if (onFirst)
        {
            p.ParametersOnS1(out u, out v);
        }
        else
        {
            p.ParametersOnS2(out u, out v);
        }
    }

    private static bool IsOnDomS(bool onFirst, in PatchPoint p) => onFirst ? p.IsOnDomS1 : p.IsOnDomS2;

    private static Curve2DAdaptorBase ArcOnS(bool onFirst, in PatchPoint p) => onFirst ? p.ArcOnS1() : p.ArcOnS2();

    private static double ParOnArc(bool onFirst, in PatchPoint p) =>
        onFirst ? p.ParameterOnArc1() : p.ParameterOnArc2();

    private static void TestWLineToRLine(NCollection_Sequence<PatchLine> slinref,
                                         NCollection_Sequence<PatchLine> slin, SurfaceAdaptorBase mySurf1,
                                         TopolTool myDom1, SurfaceAdaptorBase mySurf2,
                                         TopolTool myDom2, double tolArc)
    {
        int lastwline = slin.Length();
        WLine wLine = (WLine)slin.Value(lastwline);

        int nbvtx = wLine.VertexCount;
        if (nbvtx < 2)
        {
            return;
        }
        int paramMinOnLine = (int)wLine.Vertex(1).ParameterOnLine();
        int paramMaxOnLine = (int)wLine.Vertex(nbvtx).ParameterOnLine();
        if (paramMinOnLine >= paramMaxOnLine)
        {
            return;
        }
        int midInd = (paramMaxOnLine + paramMinOnLine) / 2;

        NCollection_Sequence<int> indicesV1 = new(), indicesV2 = new();
        int iv;
        for (iv = 1; iv <= nbvtx; iv++)
        {
            int plin = (int)wLine.Vertex(iv).ParameterOnLine();
            if (plin == paramMinOnLine)
            {
                indicesV1.Append(iv);
            }
            else if (plin == paramMaxOnLine)
            {
                indicesV2.Append(iv);
            }
        }

        bool isRLine = false;

        // cycle for both surfaces
        int @is;
        for (@is = 0; @is < 2; @is++)
        {
            bool onFirst = @is == 0;
            if ((onFirst && wLine.HasArcOnS1) || (!onFirst && wLine.HasArcOnS2))
            {
                SurfaceAdaptorBase surf;
                TopolTool domain;
                if (onFirst)
                {
                    surf = mySurf1;
                    domain = myDom1;
                }
                else
                {
                    surf = mySurf2;
                    domain = myDom2;
                }

                // resolve arcs for vertices not having a link to an arc
                double utst, vtst;
                NCollection_Array1<double> paramsResolved = new(1, nbvtx);
                NCollection_Array1<Curve2DAdaptorBase?> arcsResolved = new(1, nbvtx);
                arcsResolved.Init(null);
                for (iv = 1; iv <= nbvtx; iv++)
                {
                    if (!IsOnDomS(onFirst, wLine.Vertex(iv)))
                    {
                        int ip = (int)wLine.Vertex(iv).ParameterOnLine();
                        ParOnS(onFirst, wLine.Point(ip), out utst, out vtst);
                        double distmin = RealLast();
                        for (domain.Init(); domain.More(); domain.Next())
                        {
                            Curve2DAdaptorBase arc = domain.Value();
                            if (ProjectOnArc(utst, vtst, arc, surf, tolArc, out double par, out double dist)
                                && dist < distmin)
                            {
                                arcsResolved[iv] = arc;
                                paramsResolved[iv] = par;
                                distmin = dist;
                            }
                        }
                    }
                }

                // prepare list of common arcs for both ends of wline
                IndexedSet<Curve2DAdaptorBase> mapArcsV1 = new(ReferenceEqualityComparer.Instance),
                                                          mapArcs = new(ReferenceEqualityComparer.Instance);
                int i;
                for (i = 1; i <= indicesV1.Length(); i++)
                {
                    iv = indicesV1[i];
                    Curve2DAdaptorBase? arc;
                    if (IsOnDomS(onFirst, wLine.Vertex(iv)))
                    {
                        arc = ArcOnS(onFirst, wLine.Vertex(iv));
                    }
                    else
                    {
                        arc = arcsResolved[iv];
                    }
                    if (arc is not null)
                    {
                        mapArcsV1.Add(arc);
                    }
                }
                for (i = 1; i <= indicesV2.Length(); i++)
                {
                    iv = indicesV2[i];
                    Curve2DAdaptorBase? arc;
                    if (IsOnDomS(onFirst, wLine.Vertex(iv)))
                    {
                        arc = ArcOnS(onFirst, wLine.Vertex(iv));
                    }
                    else
                    {
                        arc = arcsResolved[iv];
                    }
                    if (arc is not null && mapArcsV1.Contains(arc))
                    {
                        mapArcs.Add(arc);
                    }
                }

                // for each common arc
                for (int ia = 0; ia < mapArcs.Count; ia++)
                {
                    Curve2DAdaptorBase arc = mapArcs[ia];
                    // get end vertices of wline linked with this arc
                    int iv1 = 0, iv2 = 0;
                    for (i = 1; i <= indicesV1.Length() && iv1 == 0; i++)
                    {
                        iv = indicesV1[i];
                        Curve2DAdaptorBase? arc1;
                        if (IsOnDomS(onFirst, wLine.Vertex(iv)))
                        {
                            arc1 = ArcOnS(onFirst, wLine.Vertex(iv));
                        }
                        else
                        {
                            arc1 = arcsResolved[iv];
                        }
                        if (arc1 is not null && ReferenceEquals(arc1, arc))
                        {
                            iv1 = iv;
                        }
                    }
                    for (i = 1; i <= indicesV2.Length() && iv2 == 0; i++)
                    {
                        iv = indicesV2[i];
                        Curve2DAdaptorBase? arc1;
                        if (IsOnDomS(onFirst, wLine.Vertex(iv)))
                        {
                            arc1 = ArcOnS(onFirst, wLine.Vertex(iv));
                        }
                        else
                        {
                            arc1 = arcsResolved[iv];
                        }
                        if (arc1 is not null && ReferenceEquals(arc1, arc))
                        {
                            iv2 = iv;
                        }
                    }
                    if (iv1 == 0 || iv2 == 0)
                    {
                        continue;
                    }
                    double par1 = (arcsResolved[iv1] is null ? ParOnArc(onFirst, wLine.Vertex(iv1)) : paramsResolved[iv1]);
                    double par2 = (arcsResolved[iv2] is null ? ParOnArc(onFirst, wLine.Vertex(iv2)) : paramsResolved[iv2]);

                    // check that the middle point is on arc
                    ParOnS(onFirst, wLine.Point(midInd), out utst, out vtst);
                    if (midInd == paramMinOnLine)
                    {
                        ParOnS(onFirst, wLine.Point(midInd + 1), out double utst1, out double vtst1);
                        utst = (utst + utst1) * 0.5;
                        vtst = (vtst + vtst1) * 0.5;
                    }
                    if (!ProjectOnArc(utst, vtst, arc, surf, tolArc, out _, out _))
                    {
                        continue;
                    }

                    //-- codage de la WLine en RLine
                    RLine rlig = new RLine(true, WalkingSituation.Unknown, WalkingSituation.Unknown);
                    if (onFirst)
                    {
                        rlig.SetArcOnS1(arc);
                    }
                    else
                    {
                        rlig.SetArcOnS2(arc);
                    }

                    LineOn2S lineOn2S = new LineOn2S();
                    LineOn2S lori = wLine.Curve();
                    int ivmin, ivmax;
                    double parmin, parmax;
                    bool reverse = false;
                    NCollection_Sequence<int> pIndVmin, pIndVmax;
                    if (par1 < par2)
                    {
                        for (i = paramMinOnLine; i <= paramMaxOnLine; i++)
                        {
                            lineOn2S.Add(lori.Value(i));
                        }
                        ivmin = iv1;
                        ivmax = iv2;
                        parmin = par1;
                        parmax = par2;
                        pIndVmin = indicesV1;
                        pIndVmax = indicesV2;
                    }
                    else
                    {
                        for (i = paramMaxOnLine; i >= paramMinOnLine; i--)
                        {
                            lineOn2S.Add(lori.Value(i));
                        }
                        ivmin = iv2;
                        ivmax = iv1;
                        parmin = par2;
                        parmax = par1;
                        pIndVmin = indicesV2;
                        pIndVmax = indicesV1;
                        reverse = true;
                    }
                    rlig.Add(lineOn2S);
                    WalkingTransition transitionUndecided = new WalkingTransition();
                    PatchPoint vtxFirst = wLine.Vertex(ivmin);
                    vtxFirst.SetParameter(parmin);
                    if (arcsResolved[ivmin] is not null)
                    {
                        vtxFirst.SetArc(onFirst, arc, parmin, transitionUndecided, transitionUndecided);
                    }
                    if (reverse)
                    {
                        vtxFirst.ReverseTransition(); //-- inversion des transitions
                    }
                    rlig.AddVertex(vtxFirst);
                    for (i = 1; i <= pIndVmin.Length(); i++)
                    {
                        iv = pIndVmin.Value(i);
                        if (iv != ivmin)
                        {
                            PatchPoint vtx = wLine.Vertex(iv);
                            vtx.SetParameter(parmin);
                            if (arcsResolved[iv] is not null)
                            {
                                vtx.SetArc(onFirst, arc, parmin, transitionUndecided, transitionUndecided);
                            }
                            if (reverse)
                            {
                                vtx.ReverseTransition();
                            }
                            rlig.AddVertex(vtx);
                        }
                    }
                    for (i = 1; i <= pIndVmax.Length(); i++)
                    {
                        iv = pIndVmax.Value(i);
                        if (iv != ivmax)
                        {
                            PatchPoint vtx = wLine.Vertex(iv);
                            vtx.SetParameter(parmax);
                            if (arcsResolved[iv] is not null)
                            {
                                vtx.SetArc(onFirst, arc, parmax, transitionUndecided, transitionUndecided);
                            }
                            if (reverse)
                            {
                                vtx.ReverseTransition();
                            }
                            rlig.AddVertex(vtx);
                        }
                    }
                    PatchPoint vtxLast = wLine.Vertex(ivmax);
                    vtxLast.SetParameter(parmax);
                    if (arcsResolved[ivmax] is not null)
                    {
                        vtxLast.SetArc(onFirst, arc, parmax, transitionUndecided, transitionUndecided);
                    }
                    if (reverse)
                    {
                        vtxLast.ReverseTransition();
                    }
                    rlig.AddVertex(vtxLast);
                    rlig.SetFirstPoint(1);
                    rlig.SetLastPoint(indicesV1.Length() + indicesV2.Length());
                    slin.Append(rlig);
                    isRLine = true;
                }
            }
        }

        if (isRLine || TestIfWLineIsRestriction(slinref, wLine, mySurf1, myDom1, mySurf2, myDom2, tolArc))
        {
            slin.Remove(lastwline);
        }
    }
}
