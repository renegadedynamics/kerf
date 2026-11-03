// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_FaceIsoLiner.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_FaceIsoLiner.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    public static void Perform(int FI, in Face F, HLRTopoBRep_Data DS, int nbIsos)
    {
        _ = FI; // avoid compiler warning

        double UMin, UMax, VMin, VMax, U1, U2;
        int ne = 0;

        Explorer ExpEdges = new();
        Face TF = F;
        TF.Orientation(Forward);
        Point2D P = new();
        Point3D P1, P2;
        Vertex V1U = new(), V2U = new(), V1V = new(), V2V = new();

        Geom2dHatch_Intersector Intersector = new(IntersectorConfusion, IntersectorTangency);
        Geom2dHatch_Hatcher Hatcher = new(Intersector, HatcherConfusion2d, HatcherConfusion3d, true);

        BrepTools.UVBounds(TF, out UMin, out UMax, out VMin, out VMax);
        bool InfiniteUMin = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(UMin);
        bool InfiniteUMax = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(UMax);
        bool InfiniteVMin = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(VMin);
        bool InfiniteVMax = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(VMax);

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

        for (ExpEdges.Init(TF, ShapeType.Edge); // Edges of the face TF
             ExpEdges.More();
             ExpEdges.Next())
        {
            ne++;
        }

        if (DS.FaceHasIntL(TF))
        { // OutLines built on face TF.
            NCollection_List<Shape>.Iterator itE = new();
            for (itE.Initialize(DS.FaceIntL(TF)); itE.More(); itE.Next())
            {
                ne++;
            }
        }

        NCollection_Array1<Shape> SH = new(1, ne);
        NCollection_Array1<bool> IL = new(1, ne); // internal OutLine

        for (ExpEdges.Init(TF, ShapeType.Edge); ExpEdges.More(); ExpEdges.Next())
        {
            int IndE;
            Edge newE = ShapeCasts.Edge(ExpEdges.Current());
            Curve2D PC = Tool.CurveOnSurface(newE, TF, out U1, out U2)!;
            if (Abs(PC.FirstParameter() - U1) <= global::Kerf.Engine.Foundation.Precision.PConfusion()
                && Abs(PC.LastParameter() - U2) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                IndE = Hatcher.AddElement(PC, newE.Orientation());
            }
            else
            {
                TrimmedCurve2D TPC = new(PC, U1, U2);
                CurveAdaptor2D aGAC = new(TPC);
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
            NCollection_List<Shape>.Iterator itE = new();
            for (itE.Initialize(DS.FaceIntL(TF)); itE.More(); itE.Next())
            {
                int IndE;
                Edge newE = ShapeCasts.Edge(itE.Value());
                Curve2D PC = Tool.CurveOnSurface(newE, TF, out U1, out U2)!;
                if (Abs(PC.FirstParameter() - U1) <= global::Kerf.Engine.Foundation.Precision.PConfusion()
                    && Abs(PC.LastParameter() - U2) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    IndE = Hatcher.AddElement(PC, Internal);
                }
                else
                {
                    TrimmedCurve2D TPC = new(PC, U1, U2);
                    CurveAdaptor2D aGAC = new(TPC);
                    IndE = Hatcher.AddElement(aGAC, Internal);
                }
                SH[IndE] = newE;
                IL[IndE] = true;
            }
        }

        //-----------------------------------------------------------------------
        // Creation des hachures.
        //-----------------------------------------------------------------------

        FaceSurfaceAdaptor Surface = new(TF);
        double Tolerance = Tool.Tolerance(TF);

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
            Direction2D Dir = new(Direction2D.D.Y);

            for (IIso = 1; IIso <= nbIsos; IIso++)
            {
                Point2D Ori = new(UPrm, 0.0);
                LineCurve2D IsoLine = new(Ori, Dir);

                CurveAdaptor2D aGAC = new(IsoLine);
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
                                V1U = MakeVertex(ShapeCasts.Edge(SH[PntE.Index()]), P1, PntE.Parameter(), Tolerance, DS);
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
                                V2U = MakeVertex(ShapeCasts.Edge(SH[PntE.Index()]), P2, PntE.Parameter(), Tolerance, DS);
                                if (IL[PntE.Index()])
                                {
                                    DS.AddOutV(V2U);
                                }
                            }
                        }
                        if (!V1U.IsNull && !V2U.IsNull)
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
            Direction2D Dir = new(Direction2D.D.X);

            for (IIso = 1; IIso <= nbIsos; IIso++)
            {
                Point2D Ori = new(0.0, VPrm);
                LineCurve2D IsoLine = new(Ori, Dir);

                CurveAdaptor2D aGAC = new(IsoLine);
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
                                V1V = MakeVertex(ShapeCasts.Edge(SH[PntE.Index()]), P1, PntE.Parameter(), Tolerance, DS);
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
                                V2V = MakeVertex(ShapeCasts.Edge(SH[PntE.Index()]), P2, PntE.Parameter(), Tolerance, DS);
                                if (IL[PntE.Index()])
                                {
                                    DS.AddOutV(V2V);
                                }
                            }
                        }
                        if (!V1V.IsNull && !V2V.IsNull)
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
    public static Vertex MakeVertex(in Edge E, in Point3D P, double Par, double Tol,
                                           HLRTopoBRep_Data DS)
    {
        Vertex V = new(), VF, VL;
        BrepBuilder B = new();
        Exploration.Vertices(E, out VF, out VL);
        if (P.IsEqual(Tool.Pnt(VF), Tool.Tolerance(VF)))
        {
            return VF;
        }
        if (P.IsEqual(Tool.Pnt(VL), Tool.Tolerance(VL)))
        {
            return VL;
        }

        for (DS.InitVertex(E); DS.MoreVertex(); DS.NextVertex())
        {
            Vertex curV = DS.Vertex();
            double curP = DS.Parameter();
            if (P.IsEqual(Tool.Pnt(curV), Tool.Tolerance(curV)))
            {
                V = curV;
                break;
            }
            else if (Par < curP)
            {
                B.MakeVertex(ref V, P, Tol);
                V.Orientation(Internal);
                DS.InsertBefore(V, Par);
                break;
            }
        }

        if (!DS.MoreVertex())
        {
            B.MakeVertex(ref V, P, Tol);
            V.Orientation(Internal);
            DS.Append(V, Par);
        }

        return V;
    }

    /// <summary>Builds the isoline edge between V1 and V2 (parameters U1 and U2 on Iso) and records it as F's isoline.</summary>
    public static void MakeIsoLine(in Face F, LineCurve2D Iso, ref Vertex V1, ref Vertex V2,
                                   double U1, double U2, double Tol, HLRTopoBRep_Data DS)
    {
        BrepBuilder B = new();
        Edge E = new();
        E.Orientation(Internal);
        V1.Orientation(Forward);
        V2.Orientation(Reversed);
        B.MakeEdge(ref E);
        B.UpdateEdge(E, Iso, F, Tol);
        B.Add(E, V1);
        B.UpdateVertex(V1, U1, E, Tol);
        B.Add(E, V2);
        B.UpdateVertex(V2, U2, E, Tol);
        DS.AddIsoL(F).Append(E);
    }
}
