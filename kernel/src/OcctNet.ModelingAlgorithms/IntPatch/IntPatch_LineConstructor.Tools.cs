// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_LineConstructor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Occt.IntPatch_IType;
using static Occt.IntSurf_Situation;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

// The file-static functions of IntPatch_LineConstructor.cxx.
public partial class IntPatch_LineConstructor
{
    private static void Recadre(Adaptor3d_Surface myHS1, Adaptor3d_Surface myHS2, ref double u1, ref double v1,
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

    private static void Parameters(Adaptor3d_Surface myHS1, Adaptor3d_Surface myHS2, in Point3D Ptref, ref double U1,
                                   ref double V1, ref double U2, ref double V2)
    {
        IntSurf_Quadric quad1 = new IntSurf_Quadric(), quad2 = new IntSurf_Quadric();
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
        quad1.Parameters(Ptref, ref U1, ref V1);
        quad2.Parameters(Ptref, ref U2, ref V2);
    }

    private static double LocalFirstParameter(IntPatch_Line L)
    {
        double firstp = 0.0;
        IntPatch_IType typl = L.ArcType();
        switch (typl)
        {
            case IntPatch_Analytic:
            {
                IntPatch_ALine alin = (IntPatch_ALine)L;
                if (alin.HasFirstPoint())
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

            case IntPatch_Restriction:
            {
                IntPatch_RLine rlin = (IntPatch_RLine)L;
                if (rlin.HasFirstPoint())
                {
                    firstp = rlin.FirstPoint().ParameterOnLine();
                }
                else
                {
                    firstp = -global::Kerf.Engine.Foundation.Precision.Infinite(); // a voir selon le type de la ligne 2d
                }
                return firstp;
            }
            case IntPatch_Walking:
            {
                IntPatch_WLine wlin = (IntPatch_WLine)L;
                if (wlin.HasFirstPoint())
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
                IntPatch_GLine glin = (IntPatch_GLine)L;
                if (glin.HasFirstPoint())
                {
                    firstp = glin.FirstPoint().ParameterOnLine();
                }
                else
                {
                    switch (typl)
                    {
                        case IntPatch_Lin:
                        case IntPatch_Parabola:
                        case IntPatch_Hyperbola:
                            firstp = -global::Kerf.Engine.Foundation.Precision.Infinite();
                            break;

                        case IntPatch_Circle:
                        case IntPatch_Ellipse:
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

    private static double LocalLastParameter(IntPatch_Line L)
    {
        double lastp = 0.0;
        IntPatch_IType typl = L.ArcType();
        switch (typl)
        {
            case IntPatch_Analytic:
            {
                IntPatch_ALine alin = (IntPatch_ALine)L;

                if (alin.HasLastPoint())
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

            case IntPatch_Restriction:
            {
                IntPatch_RLine rlin = (IntPatch_RLine)L;

                if (rlin.HasLastPoint())
                {
                    lastp = rlin.LastPoint().ParameterOnLine();
                }
                else
                {
                    lastp = global::Kerf.Engine.Foundation.Precision.Infinite(); // a voir selon le type de la ligne 2d
                }
                return lastp;
            }
            case IntPatch_Walking:
            {
                IntPatch_WLine wlin = (IntPatch_WLine)L;

                if (wlin.HasLastPoint())
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
                IntPatch_GLine glin = (IntPatch_GLine)L;

                if (glin.HasLastPoint())
                {
                    lastp = glin.LastPoint().ParameterOnLine();
                }
                else
                {
                    switch (typl)
                    {
                        case IntPatch_Lin:
                        case IntPatch_Parabola:
                        case IntPatch_Hyperbola:
                            lastp = global::Kerf.Engine.Foundation.Precision.Infinite();
                            break;

                        case IntPatch_Circle:
                        case IntPatch_Ellipse:
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
    private static double ComputeParametricTolerance(double theTol3d, in Vector3D theD1u, in Vector3D theD1v)
    {
        double nad1u = theD1u.Magnitude();
        double nad1v = theD1v.Magnitude();
        double tolu = 0.0, tolv = 0.0;
        if (nad1u > 1e-12)
        {
            tolu = theTol3d / nad1u;
        }
        else
        {
            tolu = 0.1;
        }
        if (nad1v > 1e-12)
        {
            tolv = theTol3d / nad1v;
        }
        else
        {
            tolv = 0.1;
        }
        double aTolerance = (tolu > tolv) ? tolu : tolv;
        return aTolerance;
    }
    // modified by NIZHNY-MKK  Tue Apr  3 15:03:11 2001.END

    private static int AppendSameVertexA(IntPatch_ALine alig, IntPatch_ALine L, int index, int[] TabIndex)
    {
        int i, a, n;
        a = 0;
        n = L.NbVertex();
        IntPatch_Point Vtxindex = L.Vertex(index);
        double thetol1 = Vtxindex.Tolerance();
        for (i = 1; i <= n; i++)
        {
            if (i != index)
            {
                IntPatch_Point Vtxi = L.Vertex(i);
                double thetol2 = Vtxi.Tolerance();
                if (thetol2 < thetol1)
                {
                    thetol2 = thetol1;
                }
                double d_4 = Vtxindex.Value().Distance(Vtxi.Value());
                if (d_4 <= thetol2)
                {
                    alig.AddVertex(Vtxi);
                    a++;
                    TabIndex[i] = TabIndex[index];
                }
            }
        }
        return (a);
    }

    private static int AppendSameVertexG(IntPatch_GLine glig, IntPatch_GLine L, int index, double decal,
                                         int[] TabIndex)
    {
        int i, a, n;
        double p1, p2, d;
        bool aajouter;
        a = 0;
        n = L.NbVertex();
        IntPatch_Point Vtxindex = L.Vertex(index);
        double thetol1 = Vtxindex.Tolerance();
        for (i = 1; i <= n; i++)
        {
            if (i != index)
            {
                IntPatch_Point Vtxi = L.Vertex(i);
                aajouter = false;
                double thetol2 = Vtxi.Tolerance();
                if (thetol2 < thetol1)
                {
                    thetol2 = thetol1;
                }
                d = Vtxindex.Value().Distance(Vtxi.Value());
                if (d <= thetol2)
                {
                    aajouter = true;
                }

                //-- Le test suivant a ete ajoute le 20 aout 98 (??? mefiance ???)
                else
                {
                    p1 = Vtxindex.ParameterOnLine();
                    p2 = Vtxi.ParameterOnLine();
                    if (Math.Abs(p1 - p2) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        aajouter = true;
                    }
                }
                if (aajouter)
                {
                    p1 = Vtxindex.ParameterOnLine();
                    IntPatch_Point aVtx = Vtxi;
                    aVtx.SetParameter(p1 + decal);
                    glig.AddVertex(aVtx);
                    a++;
                    TabIndex[i] = TabIndex[index];
                }
            }
        }
        return (a);
    }

    private static int AppendSameVertexW(IntPatch_WLine wlig, IntPatch_WLine L, int index, double par,
                                         int[] TabIndex)
    {
        int i, a, n;
        a = 0;
        n = L.NbVertex();
        IntPatch_Point Vtxindex = L.Vertex(index);
        Point3D Pntindex = Vtxindex.Value();
        double thetol1 = Vtxindex.Tolerance();
        for (i = 1; i <= n; i++)
        {
            if (i != index)
            {
                IntPatch_Point Vtxi = L.Vertex(i);
                double d_2 = Pntindex.Distance(Vtxi.Value());
                double thetol2 = Vtxi.Tolerance();
                if (thetol2 < thetol1)
                {
                    thetol2 = thetol1;
                }
                //-- le debugger voit 2 fois la variable d ici. ???? -> d_2
                if (d_2 <= thetol2)
                {
                    Vtxi.SetParameter(par);
                    Vtxindex.ParametersOnS1(out double u1, out double v1);
                    Vtxindex.ParametersOnS2(out double u2, out double v2);
                    Vtxi.SetParameters(u1, v1, u2, v2);
                    Vtxi.SetValue(Pntindex);
                    wlig.AddVertex(Vtxi);
                    a++;
                    TabIndex[i] = TabIndex[index];
                }
            }
        }
        return (a);
    }

    private static int AppendSameVertexR(IntPatch_RLine rlig, IntPatch_RLine L, int index, int[] TabIndex)
    {
        int i, a, n;
        a = 0;
        n = L.NbVertex();
        IntPatch_Point Vtxindex = L.Vertex(index);
        double thetol1 = Vtxindex.Tolerance();
        for (i = 1; i <= n; i++)
        {
            if (i != index)
            {
                IntPatch_Point Vtxi = L.Vertex(i);
                double d_3 = Vtxindex.Value().Distance(Vtxi.Value());
                double thetol2 = Vtxi.Tolerance();
                if (thetol2 < thetol1)
                {
                    thetol2 = thetol1;
                }
                if (d_3 < thetol2)
                {
                    if (Vtxi.ParameterOnLine() != Vtxindex.ParameterOnLine())
                    {
                        IntPatch_Point Vtxicop = L.Vertex(i);
                        Vtxicop.SetParameter(Vtxindex.ParameterOnLine());
                        rlig.AddVertex(Vtxicop);
                    }
                    else
                    {
                        rlig.AddVertex(Vtxi);
                    }
                    a++;
                    TabIndex[i] = TabIndex[index];
                }
            }
        }
        return (a);
    }

    // OCCT's two GeomAbs_SurfaceType arguments are unused.
    private static void AddLine(IntPatch_Line L, int i, int j, SurfaceKind TypeS1, SurfaceKind TypeS2,
                                int[] TabIndex, NCollection_Sequence<IntPatch_Line> slin)
    {
        int IndexFirstVertex = 1;
        int IndexLastVertex = 2;
        if (i == j)
        {
            IndexLastVertex = 1;
        }
        IntPatch_IType typl = L.ArcType();
        switch (typl)
        {
            case IntPatch_Analytic:
            {
                IntPatch_ALine ALine = (IntPatch_ALine)L;
                IntPatch_ALine alig;
                if (L.TransitionOnS1() == IntSurf_Undecided)
                {
                    alig = new IntPatch_ALine(ALine.Curve(), L.IsTangent());
                }
                else if (L.TransitionOnS1() == IntSurf_Touch)
                {
                    alig = new IntPatch_ALine(ALine.Curve(), L.IsTangent(), L.SituationS1(), L.SituationS2());
                }
                else
                {
                    alig = new IntPatch_ALine(ALine.Curve(), L.IsTangent(), L.TransitionOnS1(), L.TransitionOnS2());
                }
                alig.AddVertex(ALine.Vertex(i));
                IndexLastVertex += AppendSameVertexA(alig, ALine, i, TabIndex);
                if (i != j)
                {
                    alig.AddVertex(ALine.Vertex(j));
                    IndexLastVertex += AppendSameVertexA(alig, ALine, j, TabIndex);
                }
                alig.SetFirstPoint(IndexFirstVertex);
                alig.SetLastPoint(IndexLastVertex);
                slin.Append(alig);
                break;
            }
            case IntPatch_Walking:
            { //-- ****************************************
                IntPatch_WLine WLine = (IntPatch_WLine)L;
                IntSurf_LineOn2S Lori = WLine.Curve();
                IntSurf_LineOn2S LineOn2S = new IntSurf_LineOn2S();
                int ParamMinOnLine = (int)WLine.Vertex(i).ParameterOnLine();
                int ParamMaxOnLine = (int)WLine.Vertex(j).ParameterOnLine();
                for (int k = ParamMinOnLine; k <= ParamMaxOnLine; k++)
                {
                    LineOn2S.Add(Lori.Value(k));
                }
                IntPatch_WLine wlig;
                if (L.TransitionOnS1() == IntSurf_Undecided)
                {
                    wlig = new IntPatch_WLine(LineOn2S, L.IsTangent());
                }
                else if (L.TransitionOnS1() == IntSurf_Touch)
                {
                    wlig = new IntPatch_WLine(LineOn2S, L.IsTangent(), L.SituationS1(), L.SituationS2());
                }
                else
                {
                    wlig = new IntPatch_WLine(LineOn2S, L.IsTangent(), L.TransitionOnS1(), L.TransitionOnS2());
                }
                if (WLine.HasArcOnS1())
                {
                    wlig.SetArcOnS1(WLine.GetArcOnS1()!);
                }
                if (WLine.HasArcOnS2())
                {
                    wlig.SetArcOnS2(WLine.GetArcOnS2()!);
                }
                IntPatch_Point Vtx = WLine.Vertex(i);
                Vtx.SetParameter(1);
                wlig.AddVertex(Vtx);
                IndexLastVertex += AppendSameVertexW(wlig, WLine, i, 1, TabIndex);
                if (i != j)
                {
                    Vtx = WLine.Vertex(j);
                    Vtx.SetParameter(LineOn2S.NbPoints());
                    wlig.AddVertex(Vtx);
                    IndexLastVertex += AppendSameVertexW(wlig, WLine, j, LineOn2S.NbPoints(), TabIndex);
                }
                wlig.SetFirstPoint(IndexFirstVertex);
                wlig.SetLastPoint(IndexLastVertex);
                wlig.SetPeriod(WLine.U1Period(), WLine.V1Period(), WLine.U2Period(), WLine.V2Period());
                wlig.ComputeVertexParameters(global::Kerf.Engine.Foundation.Precision.Confusion());
                slin.Append(wlig);
                //-- **********************************************************************

                break;
            }
            case IntPatch_Restriction:
            {
                IntPatch_RLine RLine = (IntPatch_RLine)L;
                IndexLastVertex = 2;
                IndexFirstVertex = 1;
                IntPatch_RLine rlig;
                if (L.TransitionOnS1() == IntSurf_Undecided)
                {
                    rlig = new IntPatch_RLine(L.IsTangent());
                }
                else if (L.TransitionOnS1() == IntSurf_Touch)
                {
                    rlig = new IntPatch_RLine(L.IsTangent(), L.SituationS1(), L.SituationS2());
                }
                else
                {
                    rlig = new IntPatch_RLine(L.IsTangent(), L.TransitionOnS1(), L.TransitionOnS2());
                }
                if (RLine.IsArcOnS1())
                {
                    rlig.SetArcOnS1(RLine.ArcOnS1()!);
                }
                if (RLine.IsArcOnS2())
                {
                    rlig.SetArcOnS2(RLine.ArcOnS2()!);
                }

                rlig.AddVertex(RLine.Vertex(i));
                // XPU1009
                IndexLastVertex += AppendSameVertexR(rlig, RLine, i, TabIndex);
                for (int k = i + 1; k < j; k++)
                {
                    rlig.AddVertex(RLine.Vertex(k));
                    IndexLastVertex++;
                }
                if (i != j)
                {
                    rlig.AddVertex(RLine.Vertex(j));
                    // XPU1009
                    IndexLastVertex += AppendSameVertexR(rlig, RLine, j, TabIndex);
                }
                rlig.SetFirstPoint(IndexFirstVertex);
                rlig.SetLastPoint(IndexLastVertex);
                rlig.ComputeVertexParameters(global::Kerf.Engine.Foundation.Precision.Confusion());
                slin.Append(rlig);
                break;
            }
            case IntPatch_Lin:
            case IntPatch_Parabola:
            case IntPatch_Hyperbola:
            case IntPatch_Circle:
            case IntPatch_Ellipse:
            {
                IntPatch_GLine GLine = (IntPatch_GLine)L;
                IntPatch_GLine glig;
                switch (typl)
                {
                    case IntPatch_Lin:
                        if (L.TransitionOnS1() == IntSurf_Undecided)
                        {
                            glig = new IntPatch_GLine(GLine.Line(), L.IsTangent());
                        }
                        else if (L.TransitionOnS1() == IntSurf_Touch)
                        {
                            glig = new IntPatch_GLine(GLine.Line(), L.IsTangent(), L.SituationS1(), L.SituationS2());
                        }
                        else
                        {
                            glig = new IntPatch_GLine(GLine.Line(), L.IsTangent(), L.TransitionOnS1(),
                                                      L.TransitionOnS2());
                        }
                        break;
                    case IntPatch_Parabola:
                        if (L.TransitionOnS1() == IntSurf_Undecided)
                        {
                            glig = new IntPatch_GLine(GLine.Parabola(), L.IsTangent());
                        }
                        else if (L.TransitionOnS1() == IntSurf_Touch)
                        {
                            glig = new IntPatch_GLine(GLine.Parabola(), L.IsTangent(), L.SituationS1(),
                                                      L.SituationS2());
                        }
                        else
                        {
                            glig = new IntPatch_GLine(GLine.Parabola(), L.IsTangent(), L.TransitionOnS1(),
                                                      L.TransitionOnS2());
                        }
                        break;
                    case IntPatch_Hyperbola:
                        if (L.TransitionOnS1() == IntSurf_Undecided)
                        {
                            glig = new IntPatch_GLine(GLine.Hyperbola(), L.IsTangent());
                        }
                        else if (L.TransitionOnS1() == IntSurf_Touch)
                        {
                            glig = new IntPatch_GLine(GLine.Hyperbola(), L.IsTangent(), L.SituationS1(),
                                                      L.SituationS2());
                        }
                        else
                        {
                            glig = new IntPatch_GLine(GLine.Hyperbola(), L.IsTangent(), L.TransitionOnS1(),
                                                      L.TransitionOnS2());
                        }
                        break;
                    case IntPatch_Circle:
                        if (L.TransitionOnS1() == IntSurf_Undecided)
                        {
                            glig = new IntPatch_GLine(GLine.Circle(), L.IsTangent());
                        }
                        else if (L.TransitionOnS1() == IntSurf_Touch)
                        {
                            glig = new IntPatch_GLine(GLine.Circle(), L.IsTangent(), L.SituationS1(),
                                                      L.SituationS2());
                        }
                        else
                        {
                            glig = new IntPatch_GLine(GLine.Circle(), L.IsTangent(), L.TransitionOnS1(),
                                                      L.TransitionOnS2());
                        }
                        break;
                    case IntPatch_Ellipse:
                    default:
                        if (L.TransitionOnS1() == IntSurf_Undecided)
                        {
                            glig = new IntPatch_GLine(GLine.Ellipse(), L.IsTangent());
                        }
                        else if (L.TransitionOnS1() == IntSurf_Touch)
                        {
                            glig = new IntPatch_GLine(GLine.Ellipse(), L.IsTangent(), L.SituationS1(),
                                                      L.SituationS2());
                        }
                        else
                        {
                            glig = new IntPatch_GLine(GLine.Ellipse(), L.IsTangent(), L.TransitionOnS1(),
                                                      L.TransitionOnS2());
                        }
                        break;
                }
                glig.AddVertex(GLine.Vertex(i));
                IndexLastVertex += AppendSameVertexG(glig, GLine, i, 0, TabIndex);
                if (i != j)
                {
                    if ((typl == IntPatch_Circle || typl == IntPatch_Ellipse) && i > j)
                    {
                        IntPatch_Point Vtx = GLine.Vertex(j);
                        Vtx.SetParameter(GLine.Vertex(j).ParameterOnLine() + M_PI + M_PI);
                        glig.AddVertex(Vtx);
                        IndexLastVertex += AppendSameVertexG(glig, GLine, j, M_PI + M_PI, TabIndex);
                    }
                    else
                    {
                        glig.AddVertex(GLine.Vertex(j));
                        IndexLastVertex += AppendSameVertexG(glig, GLine, j, 0, TabIndex);
                    }
                }
                glig.SetFirstPoint(IndexFirstVertex);
                glig.SetLastPoint(IndexLastVertex);
                slin.Append(glig);
                break;
            }
            default:
            {
                throw new ConstructionErrorException("IntPatch_LineConstructor::AddLine");
            }
        }
    }

    private static double GetVertexTolerance(in IntPatch_Point vtx)
    {
        double tol = vtx.Tolerance();
        return tol;
    }

    // OCCT's TolArc argument is commented out.
    private static bool IsSegmentSmall(IntPatch_WLine WLine, int ivFirst, int ivLast)
    {
        IntPatch_Point vtxF = WLine.Vertex(ivFirst);
        IntPatch_Point vtxL = WLine.Vertex(ivLast);
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
        Point3D p1 = WLine.Point(ipF).Value();
        for (int i = ipF + 1; i <= ipL; i++)
        {
            Point3D p2 = WLine.Point(i).Value();
            len += p1.Distance(p2);
            if (len > tol)
            {
                break;
            }
            p1 = p2;
        }
        return len <= tol;
    }

    private static bool TestWLineIsARLine(NCollection_Sequence<IntPatch_Line> slinref, IntPatch_WLine wlin,
                                          double tol2d)
    {
        int nbpnt = wlin.NbPnts();
        int indicepnt = nbpnt / 2;
        if (indicepnt < 1)
        {
            return (false);
        }
        IntSurf_PntOn2S POn2S = wlin.Point(indicepnt);
        IntSurf_PntOn2S POn2S1 = wlin.Point(indicepnt + 1);
        int lastl = slinref.Length();
        for (int i = 1; i <= lastl; i++)
        {
            if (slinref.Value(i).ArcType() == IntPatch_Restriction)
            {
                IntPatch_RLine rlin = (IntPatch_RLine)slinref.Value(i);
                for (int @is = 0; @is < 2; @is++)
                {
                    bool onFirst = @is == 0;
                    if ((onFirst && rlin.IsArcOnS1()) || (!onFirst && rlin.IsArcOnS2()))
                    {
                        Adaptor2d_Curve2d arc;
                        double u, v, u1, v1;
                        if (onFirst)
                        {
                            arc = rlin.ArcOnS1()!;
                            POn2S.ParametersOnS1(out u, out v);
                            POn2S1.ParametersOnS1(out u1, out v1);
                        }
                        else
                        {
                            arc = rlin.ArcOnS2()!;
                            POn2S.ParametersOnS2(out u, out v);
                            POn2S1.ParametersOnS2(out u1, out v1);
                        }
                        if (indicepnt == 1)
                        {
                            u = (u + u1) * 0.5;
                            v = (v + v1) * 0.5;
                        }
                        Adaptor2d_Curve2d C2d = arc;
                        Point2D PObt, P2d = new Point2D(u, v);
                        double par = Geom2dInt_TheProjPCurOfGInter.FindParameter(C2d, P2d, 1e-7);
                        PObt = C2d.Value(par);
                        if (PObt.Distance(P2d) < tol2d)
                        {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    private static bool TestIfWLineIsRestriction(NCollection_Sequence<IntPatch_Line> slinref, IntPatch_WLine wlin,
                                                 Adaptor3d_Surface S1, Adaptor3d_TopolTool D1, Adaptor3d_Surface S2,
                                                 Adaptor3d_TopolTool D2, double TolArc)
    {
        int NbPnts = wlin.NbPnts();
        int allon1 = 0, allon2 = 0, i;
        double tol2d1 = 0.0, tol2d2 = 0.0;
        Point3D ap = new Point3D();
        Vector3D ad1u = new Vector3D(), ad1v = new Vector3D();
        for (i = 1; i <= NbPnts; i++)
        {
            IntSurf_PntOn2S Pmid = wlin.Point(i);
            Pmid.Parameters(out double u1, out double v1, out double u2, out double v2);
            //-- Estimation d un majorant de Toluv a partir de Tol
            double tol;
            //------------------------------------------
            S1.D1(u1, v1, out ap, out ad1u, out ad1v);
            tol = ComputeParametricTolerance(TolArc, ad1u, ad1v);
            if (tol > tol2d1)
            {
                tol2d1 = tol;
            }
            //--
            if (allon1 + 1 == i && D1.IsThePointOn(new Point2D(u1, v1), tol))
            {
                allon1++;
            }
            //------------------------------------------
            S2.D1(u2, v2, out ap, out ad1u, out ad1v);
            tol = ComputeParametricTolerance(TolArc, ad1u, ad1v);
            if (tol > tol2d2)
            {
                tol2d2 = tol;
            }
            //--
            if (allon2 + 1 == i && D2.IsThePointOn(new Point2D(u2, v2), tol))
            {
                allon2++;
            }
            if (allon1 != i && allon2 != i)
            {
                break;
            }
        }
        if (allon1 == NbPnts || allon2 == NbPnts)
        {
            double tol2d = Max(tol2d1, tol2d2);
            return TestWLineIsARLine(slinref, wlin, tol2d);
        }
        return false;
    }

    private static bool ProjectOnArc(double u, double v, Adaptor2d_Curve2d arc, Adaptor3d_Surface surf, double TolArc,
                                     out double par, out double dist)
    {
        Point3D aPbid = new Point3D();
        Vector3D ad1u = new Vector3D(), ad1v = new Vector3D();
        surf.D1(u, v, out aPbid, out ad1u, out ad1v);
        double tol2d = ComputeParametricTolerance(TolArc, ad1u, ad1v);
        Adaptor2d_Curve2d C2d = arc;
        Point2D aP = new Point2D(u, v), aPprj;
        par = Geom2dInt_TheProjPCurOfGInter.FindParameter(C2d, aP, 1e-7);
        aPprj = C2d.Value(par);
        dist = aPprj.Distance(aP);
        return dist <= tol2d;
    }

    // OCCT's pointers to member functions (piParOnS, pIsOnDomS, pArcOnS, pParOnArc) are these, chosen by onFirst.
    private static void ParOnS(bool onFirst, in IntSurf_PntOn2S P, out double U, out double V)
    {
        if (onFirst)
        {
            P.ParametersOnS1(out U, out V);
        }
        else
        {
            P.ParametersOnS2(out U, out V);
        }
    }

    private static bool IsOnDomS(bool onFirst, in IntPatch_Point P) => onFirst ? P.IsOnDomS1() : P.IsOnDomS2();

    private static Adaptor2d_Curve2d ArcOnS(bool onFirst, in IntPatch_Point P) => onFirst ? P.ArcOnS1() : P.ArcOnS2();

    private static double ParOnArc(bool onFirst, in IntPatch_Point P) =>
        onFirst ? P.ParameterOnArc1() : P.ParameterOnArc2();

    private static void TestWLineToRLine(NCollection_Sequence<IntPatch_Line> slinref,
                                         NCollection_Sequence<IntPatch_Line> slin, Adaptor3d_Surface mySurf1,
                                         Adaptor3d_TopolTool myDom1, Adaptor3d_Surface mySurf2,
                                         Adaptor3d_TopolTool myDom2, double TolArc)
    {
        int lastwline = slin.Length();
        IntPatch_WLine WLine = (IntPatch_WLine)slin.Value(lastwline);

        int nbvtx = WLine.NbVertex();
        if (nbvtx < 2)
        {
            return;
        }
        int ParamMinOnLine = (int)WLine.Vertex(1).ParameterOnLine();
        int ParamMaxOnLine = (int)WLine.Vertex(nbvtx).ParameterOnLine();
        if (ParamMinOnLine >= ParamMaxOnLine)
        {
            return;
        }
        int midInd = (ParamMaxOnLine + ParamMinOnLine) / 2;

        NCollection_Sequence<int> indicesV1 = new(), indicesV2 = new();
        int iv;
        for (iv = 1; iv <= nbvtx; iv++)
        {
            int plin = (int)WLine.Vertex(iv).ParameterOnLine();
            if (plin == ParamMinOnLine)
            {
                indicesV1.Append(iv);
            }
            else if (plin == ParamMaxOnLine)
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
            if ((onFirst && WLine.HasArcOnS1()) || (!onFirst && WLine.HasArcOnS2()))
            {
                Adaptor3d_Surface surf;
                Adaptor3d_TopolTool aDomain;
                if (onFirst)
                {
                    surf = mySurf1;
                    aDomain = myDom1;
                }
                else
                {
                    surf = mySurf2;
                    aDomain = myDom2;
                }

                // resolve arcs for vertices not having a link to an arc
                double utst, vtst;
                NCollection_Array1<double> paramsResolved = new(1, nbvtx);
                NCollection_Array1<Adaptor2d_Curve2d?> arcsResolved = new(1, nbvtx);
                arcsResolved.Init(null);
                for (iv = 1; iv <= nbvtx; iv++)
                {
                    if (!IsOnDomS(onFirst, WLine.Vertex(iv)))
                    {
                        int ip = (int)WLine.Vertex(iv).ParameterOnLine();
                        ParOnS(onFirst, WLine.Point(ip), out utst, out vtst);
                        double distmin = RealLast();
                        for (aDomain.Init(); aDomain.More(); aDomain.Next())
                        {
                            Adaptor2d_Curve2d arc = aDomain.Value();
                            if (ProjectOnArc(utst, vtst, arc, surf, TolArc, out double par, out double dist)
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
                NCollection_IndexedMap<Adaptor2d_Curve2d> mapArcsV1 = new(ReferenceEqualityComparer.Instance),
                                                          mapArcs = new(ReferenceEqualityComparer.Instance);
                int i;
                for (i = 1; i <= indicesV1.Length(); i++)
                {
                    iv = indicesV1[i];
                    Adaptor2d_Curve2d? arc;
                    if (IsOnDomS(onFirst, WLine.Vertex(iv)))
                    {
                        arc = ArcOnS(onFirst, WLine.Vertex(iv));
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
                    Adaptor2d_Curve2d? arc;
                    if (IsOnDomS(onFirst, WLine.Vertex(iv)))
                    {
                        arc = ArcOnS(onFirst, WLine.Vertex(iv));
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
                for (int ia = 1; ia <= mapArcs.Extent(); ia++)
                {
                    Adaptor2d_Curve2d arc = mapArcs.FindKey(ia);
                    // get end vertices of wline linked with this arc
                    int iv1 = 0, iv2 = 0;
                    for (i = 1; i <= indicesV1.Length() && iv1 == 0; i++)
                    {
                        iv = indicesV1[i];
                        Adaptor2d_Curve2d? arc1;
                        if (IsOnDomS(onFirst, WLine.Vertex(iv)))
                        {
                            arc1 = ArcOnS(onFirst, WLine.Vertex(iv));
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
                        Adaptor2d_Curve2d? arc1;
                        if (IsOnDomS(onFirst, WLine.Vertex(iv)))
                        {
                            arc1 = ArcOnS(onFirst, WLine.Vertex(iv));
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
                    double par1 = (arcsResolved[iv1] is null ? ParOnArc(onFirst, WLine.Vertex(iv1)) : paramsResolved[iv1]);
                    double par2 = (arcsResolved[iv2] is null ? ParOnArc(onFirst, WLine.Vertex(iv2)) : paramsResolved[iv2]);

                    // check that the middle point is on arc
                    ParOnS(onFirst, WLine.Point(midInd), out utst, out vtst);
                    if (midInd == ParamMinOnLine)
                    {
                        ParOnS(onFirst, WLine.Point(midInd + 1), out double utst1, out double vtst1);
                        utst = (utst + utst1) * 0.5;
                        vtst = (vtst + vtst1) * 0.5;
                    }
                    if (!ProjectOnArc(utst, vtst, arc, surf, TolArc, out _, out _))
                    {
                        continue;
                    }

                    //-- codage de la WLine en RLine
                    IntPatch_RLine rlig = new IntPatch_RLine(true, IntSurf_Unknown, IntSurf_Unknown);
                    if (onFirst)
                    {
                        rlig.SetArcOnS1(arc);
                    }
                    else
                    {
                        rlig.SetArcOnS2(arc);
                    }

                    IntSurf_LineOn2S LineOn2S = new IntSurf_LineOn2S();
                    IntSurf_LineOn2S Lori = WLine.Curve();
                    int ivmin, ivmax;
                    double parmin, parmax;
                    bool reverse = false;
                    NCollection_Sequence<int> pIndVmin, pIndVmax;
                    if (par1 < par2)
                    {
                        for (i = ParamMinOnLine; i <= ParamMaxOnLine; i++)
                        {
                            LineOn2S.Add(Lori.Value(i));
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
                        for (i = ParamMaxOnLine; i >= ParamMinOnLine; i--)
                        {
                            LineOn2S.Add(Lori.Value(i));
                        }
                        ivmin = iv2;
                        ivmax = iv1;
                        parmin = par2;
                        parmax = par1;
                        pIndVmin = indicesV2;
                        pIndVmax = indicesV1;
                        reverse = true;
                    }
                    rlig.Add(LineOn2S);
                    IntSurf_Transition TransitionUndecided = new IntSurf_Transition();
                    IntPatch_Point VtxFirst = WLine.Vertex(ivmin);
                    VtxFirst.SetParameter(parmin);
                    if (arcsResolved[ivmin] is not null)
                    {
                        VtxFirst.SetArc(onFirst, arc, parmin, TransitionUndecided, TransitionUndecided);
                    }
                    if (reverse)
                    {
                        VtxFirst.ReverseTransition(); //-- inversion des transitions
                    }
                    rlig.AddVertex(VtxFirst);
                    for (i = 1; i <= pIndVmin.Length(); i++)
                    {
                        iv = pIndVmin.Value(i);
                        if (iv != ivmin)
                        {
                            IntPatch_Point Vtx = WLine.Vertex(iv);
                            Vtx.SetParameter(parmin);
                            if (arcsResolved[iv] is not null)
                            {
                                Vtx.SetArc(onFirst, arc, parmin, TransitionUndecided, TransitionUndecided);
                            }
                            if (reverse)
                            {
                                Vtx.ReverseTransition();
                            }
                            rlig.AddVertex(Vtx);
                        }
                    }
                    for (i = 1; i <= pIndVmax.Length(); i++)
                    {
                        iv = pIndVmax.Value(i);
                        if (iv != ivmax)
                        {
                            IntPatch_Point Vtx = WLine.Vertex(iv);
                            Vtx.SetParameter(parmax);
                            if (arcsResolved[iv] is not null)
                            {
                                Vtx.SetArc(onFirst, arc, parmax, TransitionUndecided, TransitionUndecided);
                            }
                            if (reverse)
                            {
                                Vtx.ReverseTransition();
                            }
                            rlig.AddVertex(Vtx);
                        }
                    }
                    IntPatch_Point VtxLast = WLine.Vertex(ivmax);
                    VtxLast.SetParameter(parmax);
                    if (arcsResolved[ivmax] is not null)
                    {
                        VtxLast.SetArc(onFirst, arc, parmax, TransitionUndecided, TransitionUndecided);
                    }
                    if (reverse)
                    {
                        VtxLast.ReverseTransition();
                    }
                    rlig.AddVertex(VtxLast);
                    rlig.SetFirstPoint(1);
                    rlig.SetLastPoint(indicesV1.Length() + indicesV2.Length());
                    slin.Append(rlig);
                    isRLine = true;
                }
            }
        }

        if (isRLine || TestIfWLineIsRestriction(slinref, WLine, mySurf1, myDom1, mySurf2, myDom2, TolArc))
        {
            slin.Remove(lastwline);
        }
    }
}
