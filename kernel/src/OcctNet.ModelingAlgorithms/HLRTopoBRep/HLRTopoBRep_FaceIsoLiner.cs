// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_FaceIsoLiner.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_FaceIsoLiner.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Builds the isoparametric lines of a face for the outlining: nbIsos lines in each direction, hatched by the face's
/// edges and internal outlines, with their end vertices inserted on those edges.
/// </summary>
/// <remarks>
/// OCCT's static class. It writes into the face's edges' vertices (<see cref="MakeIsoLine"/> adds the point
/// representations of an isoline's ends, which may be the face's own vertices), so the face must be unfrozen:
/// HLRTopoBRep_OutLiner gives it its working copy (decision 1095). <c>OCCT_DEBUG</c> traces are left out.
/// </remarks>
public static class HLRTopoBRep_FaceIsoLiner
{
    private const double IntersectorConfusion = 1.0e-10;
    private const double IntersectorTangency = 1.0e-10;
    private const double HatcherConfusion2d = 1.0e-8;
    private const double HatcherConfusion3d = 1.0e-8;
    private const double Infinite = 100.0;

    /// <summary>Builds nbIsos isolines in each direction on the face F (the face number FI is only for traces).</summary>
    public static void Perform(int FI, in TopoDS_Face F, HLRTopoBRep_Data DS, int nbIsos)
    {
        _ = FI; // avoid compiler warning

        double UMin, UMax, VMin, VMax, U1, U2;
        int ne = 0;

        TopExp_Explorer ExpEdges = new();
        TopoDS_Face TF = F;
        TF.Orientation(TopAbs_FORWARD);
        gp_Pnt2d P = new();
        gp_Pnt P1, P2;
        TopoDS_Vertex V1U = new(), V2U = new(), V1V = new(), V2V = new();

        Geom2dHatch_Intersector Intersector = new(IntersectorConfusion, IntersectorTangency);
        Geom2dHatch_Hatcher Hatcher = new(Intersector, HatcherConfusion2d, HatcherConfusion3d, true);

        BRepTools.UVBounds(TF, out UMin, out UMax, out VMin, out VMax);
        bool InfiniteUMin = Precision.IsNegativeInfinite(UMin);
        bool InfiniteUMax = Precision.IsPositiveInfinite(UMax);
        bool InfiniteVMin = Precision.IsNegativeInfinite(VMin);
        bool InfiniteVMax = Precision.IsPositiveInfinite(VMax);

        if (InfiniteUMin && InfiniteUMax)
        {
            UMin = -Infinite;
            UMax = Infinite;
        }
        else if (InfiniteUMin)
        {
            UMin = UMax - Infinite;
        }
        else if (InfiniteUMax)
        {
            UMax = UMin + Infinite;
        }

        if (InfiniteVMin && InfiniteVMax)
        {
            VMin = -Infinite;
            VMax = Infinite;
        }
        else if (InfiniteVMin)
        {
            VMin = VMax - Infinite;
        }
        else if (InfiniteVMax)
        {
            VMax = VMin + Infinite;
        }

        for (ExpEdges.Init(TF, TopAbs_EDGE); // Edges of the face TF
             ExpEdges.More();
             ExpEdges.Next())
        {
            ne++;
        }

        if (DS.FaceHasIntL(TF))
        { // OutLines built on face TF.
            NCollection_List<TopoDS_Shape>.Iterator itE = new();
            for (itE.Initialize(DS.FaceIntL(TF)); itE.More(); itE.Next())
            {
                ne++;
            }
        }

        NCollection_Array1<TopoDS_Shape> SH = new(1, ne);
        NCollection_Array1<bool> IL = new(1, ne); // internal OutLine

        for (ExpEdges.Init(TF, TopAbs_EDGE); ExpEdges.More(); ExpEdges.Next())
        {
            int IndE;
            TopoDS_Edge newE = TopoDS.Edge(ExpEdges.Current());
            Geom2d_Curve PC = BRep_Tool.CurveOnSurface(newE, TF, out U1, out U2)!;
            if (Abs(PC.FirstParameter() - U1) <= Precision.PConfusion()
                && Abs(PC.LastParameter() - U2) <= Precision.PConfusion())
            {
                IndE = Hatcher.AddElement(PC, newE.Orientation());
            }
            else
            {
                Geom2d_TrimmedCurve TPC = new(PC, U1, U2);
                Geom2dAdaptor_Curve aGAC = new(TPC);
                IndE = Hatcher.AddElement(aGAC, newE.Orientation());
            }
            SH[IndE] = newE;
            if (DS.IsOutLFaceEdge(TF, newE))
            {
                IL[IndE] = true;
            }
            else
            {
                IL[IndE] = false;
            }
        }

        if (DS.FaceHasIntL(TF))
        { // get the internal OutLines built on face F.
            NCollection_List<TopoDS_Shape>.Iterator itE = new();
            for (itE.Initialize(DS.FaceIntL(TF)); itE.More(); itE.Next())
            {
                int IndE;
                TopoDS_Edge newE = TopoDS.Edge(itE.Value());
                Geom2d_Curve PC = BRep_Tool.CurveOnSurface(newE, TF, out U1, out U2)!;
                if (Abs(PC.FirstParameter() - U1) <= Precision.PConfusion()
                    && Abs(PC.LastParameter() - U2) <= Precision.PConfusion())
                {
                    IndE = Hatcher.AddElement(PC, TopAbs_INTERNAL);
                }
                else
                {
                    Geom2d_TrimmedCurve TPC = new(PC, U1, U2);
                    Geom2dAdaptor_Curve aGAC = new(TPC);
                    IndE = Hatcher.AddElement(aGAC, TopAbs_INTERNAL);
                }
                SH[IndE] = newE;
                IL[IndE] = true;
            }
        }

        //-----------------------------------------------------------------------
        // Creation des hachures.
        //-----------------------------------------------------------------------

        BRepAdaptor_Surface Surface = new(TF);
        double Tolerance = BRep_Tool.Tolerance(TF);

        int IIso;
        double DeltaU = Abs(UMax - UMin);
        double DeltaV = Abs(VMax - VMin);
        double Confusion = Min(DeltaU, DeltaV) * HatcherConfusion3d;
        Hatcher.Confusion3d(Confusion);

        //-----------------------------------------------------------------------
        // Courbes Iso U.
        //-----------------------------------------------------------------------

        double StepU = DeltaU / (double)nbIsos;
        if (StepU > Confusion)
        {
            double UPrm = UMin + StepU / 2.0;
            gp_Dir2d Dir = new(gp_Dir2d.D.Y);

            for (IIso = 1; IIso <= nbIsos; IIso++)
            {
                gp_Pnt2d Ori = new(UPrm, 0.0);
                Geom2d_Line IsoLine = new(Ori, Dir);

                Geom2dAdaptor_Curve aGAC = new(IsoLine);
                int IndH = Hatcher.AddHatching(aGAC);
                Hatcher.Trim(IndH);
                if (Hatcher.TrimDone(IndH) && !Hatcher.TrimFailed(IndH))
                {
                    Hatcher.ComputeDomains(IndH);
                }
                if (!Hatcher.IsDone(IndH))
                {
                    // OCCT_DEBUG trace left out
                    Hatcher.RemHatching(IndH);
                    continue;
                }

                int NbDom = Hatcher.NbDomains(IndH);
                if (NbDom > 0)
                {

                    for (int IDom = 1; IDom <= NbDom; IDom++)
                    {
                        HatchGen_Domain Dom = Hatcher.Domain(IndH, IDom);
                        double U11 = Dom.HasFirstPoint() ? Dom.FirstPoint().Parameter() : VMin - Infinite;
                        double U21 = Dom.HasSecondPoint() ? Dom.SecondPoint().Parameter() : VMax + Infinite;
                        IsoLine.D0(U11, out P);
                        Surface.D0(P.X(), P.Y(), out P1);
                        IsoLine.D0(U21, out P);
                        Surface.D0(P.X(), P.Y(), out P2);
                        if (Dom.HasFirstPoint())
                        { // Iso U - Premier point
                            HatchGen_PointOnHatching PntH = Dom.FirstPoint();

                            for (int IPntE = 1; IPntE <= PntH.NbPoints(); IPntE++)
                            {
                                HatchGen_PointOnElement PntE = PntH.Point(IPntE);
                                V1U = MakeVertex(TopoDS.Edge(SH[PntE.Index()]), P1, PntE.Parameter(), Tolerance, DS);
                                if (IL[PntE.Index()])
                                {
                                    DS.AddOutV(V1U);
                                }
                            }
                        }
                        if (Dom.HasSecondPoint())
                        { // Iso U - Deuxieme point
                            HatchGen_PointOnHatching PntH = Dom.SecondPoint();

                            for (int IPntE = 1; IPntE <= PntH.NbPoints(); IPntE++)
                            {
                                HatchGen_PointOnElement PntE = PntH.Point(IPntE);
                                V2U = MakeVertex(TopoDS.Edge(SH[PntE.Index()]), P2, PntE.Parameter(), Tolerance, DS);
                                if (IL[PntE.Index()])
                                {
                                    DS.AddOutV(V2U);
                                }
                            }
                        }
                        if (!V1U.IsNull() && !V2U.IsNull())
                        {
                            MakeIsoLine(F, IsoLine, ref V1U, ref V2U, U11, U21, Tolerance, DS);
                        }
                    }
                }

                Hatcher.RemHatching(IndH);
                UPrm += StepU;
            }
        }

        //-----------------------------------------------------------------------
        // Courbes Iso V.
        //-----------------------------------------------------------------------

        double StepV = DeltaV / (double)nbIsos;
        if (StepV > Confusion)
        {
            double VPrm = VMin + StepV / 2.0;
            gp_Dir2d Dir = new(gp_Dir2d.D.X);

            for (IIso = 1; IIso <= nbIsos; IIso++)
            {
                gp_Pnt2d Ori = new(0.0, VPrm);
                Geom2d_Line IsoLine = new(Ori, Dir);

                Geom2dAdaptor_Curve aGAC = new(IsoLine);
                int IndH = Hatcher.AddHatching(aGAC);
                Hatcher.Trim(IndH);
                if (Hatcher.TrimDone(IndH) && !Hatcher.TrimFailed(IndH))
                {
                    Hatcher.ComputeDomains(IndH);
                }
                if (!Hatcher.IsDone(IndH))
                {
                    // OCCT_DEBUG trace left out
                    Hatcher.RemHatching(IndH);
                    continue;
                }

                int NbDom = Hatcher.NbDomains(IndH);
                if (NbDom > 0)
                {

                    for (int IDom = 1; IDom <= NbDom; IDom++)
                    {
                        HatchGen_Domain Dom = Hatcher.Domain(IndH, IDom);
                        double U12 = Dom.HasFirstPoint() ? Dom.FirstPoint().Parameter() : VMin - Infinite;
                        double U22 = Dom.HasSecondPoint() ? Dom.SecondPoint().Parameter() : VMax + Infinite;
                        IsoLine.D0(U12, out P);
                        Surface.D0(P.X(), P.Y(), out P1);
                        IsoLine.D0(U22, out P);
                        Surface.D0(P.X(), P.Y(), out P2);
                        if (Dom.HasFirstPoint())
                        { // Iso V - Premier point
                            HatchGen_PointOnHatching PntH = Dom.FirstPoint();

                            for (int IPntE = 1; IPntE <= PntH.NbPoints(); IPntE++)
                            {
                                HatchGen_PointOnElement PntE = PntH.Point(IPntE);
                                V1V = MakeVertex(TopoDS.Edge(SH[PntE.Index()]), P1, PntE.Parameter(), Tolerance, DS);
                                if (IL[PntE.Index()])
                                {
                                    DS.AddOutV(V1V);
                                }
                            }
                        }
                        if (Dom.HasSecondPoint())
                        { // Iso V - Deuxieme point
                            HatchGen_PointOnHatching PntH = Dom.SecondPoint();

                            for (int IPntE = 1; IPntE <= PntH.NbPoints(); IPntE++)
                            {
                                HatchGen_PointOnElement PntE = PntH.Point(IPntE);
                                V2V = MakeVertex(TopoDS.Edge(SH[PntE.Index()]), P2, PntE.Parameter(), Tolerance, DS);
                                if (IL[PntE.Index()])
                                {
                                    DS.AddOutV(V2V);
                                }
                            }
                        }
                        if (!V1V.IsNull() && !V2V.IsNull())
                        {
                            MakeIsoLine(F, IsoLine, ref V1V, ref V2V, U12, U22, Tolerance, DS);
                        }
                    }
                }

                Hatcher.RemHatching(IndH);
                VPrm += StepV;
            }
        }
    }

    /// <summary>
    /// The vertex at the point P of parameter Par on the edge E: one of E's end vertices or of the vertices already
    /// inserted on it if P is on it, else a new internal vertex inserted in E's list.
    /// </summary>
    public static TopoDS_Vertex MakeVertex(in TopoDS_Edge E, in gp_Pnt P, double Par, double Tol,
                                           HLRTopoBRep_Data DS)
    {
        TopoDS_Vertex V = new(), VF, VL;
        BRep_Builder B = new();
        TopExp.Vertices(E, out VF, out VL);
        if (P.IsEqual(BRep_Tool.Pnt(VF), BRep_Tool.Tolerance(VF)))
        {
            return VF;
        }
        if (P.IsEqual(BRep_Tool.Pnt(VL), BRep_Tool.Tolerance(VL)))
        {
            return VL;
        }

        for (DS.InitVertex(E); DS.MoreVertex(); DS.NextVertex())
        {
            TopoDS_Vertex curV = DS.Vertex();
            double curP = DS.Parameter();
            if (P.IsEqual(BRep_Tool.Pnt(curV), BRep_Tool.Tolerance(curV)))
            {
                V = curV;
                break;
            }
            else if (Par < curP)
            {
                B.MakeVertex(ref V, P, Tol);
                V.Orientation(TopAbs_INTERNAL);
                DS.InsertBefore(V, Par);
                break;
            }
        }

        if (!DS.MoreVertex())
        {
            B.MakeVertex(ref V, P, Tol);
            V.Orientation(TopAbs_INTERNAL);
            DS.Append(V, Par);
        }

        return V;
    }

    /// <summary>Builds the isoline edge between V1 and V2 (parameters U1 and U2 on Iso) and records it as F's isoline.</summary>
    public static void MakeIsoLine(in TopoDS_Face F, Geom2d_Line Iso, ref TopoDS_Vertex V1, ref TopoDS_Vertex V2,
                                   double U1, double U2, double Tol, HLRTopoBRep_Data DS)
    {
        BRep_Builder B = new();
        TopoDS_Edge E = new();
        E.Orientation(TopAbs_INTERNAL);
        V1.Orientation(TopAbs_FORWARD);
        V2.Orientation(TopAbs_REVERSED);
        B.MakeEdge(ref E);
        B.UpdateEdge(E, Iso, F, Tol);
        B.Add(E, V1);
        B.UpdateVertex(V1, U1, E, Tol);
        B.Add(E, V2);
        B.UpdateVertex(V2, U2, E, Tol);
        DS.AddIsoL(F).Append(E);
    }
}
