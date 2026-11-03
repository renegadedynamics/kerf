// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_FilDS)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopOpeBRepDS_Kind;

namespace Occt;

internal static partial class ChFi3d_Builder_0
{
    //=================================================================================================

    /// <summary>
    /// Writes the stripe <paramref name="CorDat"/> into the data structure: the solid's interferences with the fillet
    /// surfaces, the curves between the fillets and on the faces with their pcurves and points, the points on the
    /// edges they cut; the regularities found go into <paramref name="reglist"/>.
    /// </summary>
    /// <remarks>
    /// Writes into the builder's data (the data structure, the surface data's common points of degenerated
    /// interferences) and, through <see cref="ChFi3d_IndexPointInDS"/>, raises the tolerance of the working shape's
    /// vertices (decision 1087).
    /// </remarks>
    internal static void ChFi3d_FilDS(int SolidIndex, ChFiDS_Stripe CorDat, TopOpeBRepDS_DataStructure DStr,
                                      NCollection_List<ChFiDS_Regul> reglist, double tol3d, double tol2d)
    {
        //  BRep_Tool Outil;
        TopExp_Explorer ex = new();
        ChFiDS_Spine? spine = CorDat.Spine();
        bool Closed = false;
        bool Degene = false, isVertex1 = false, isVertex2 = false, Singulier_en_Bout = false;
        if (spine is not null)
        {
            Closed = spine.IsPeriodic();
        }
        NCollection_Sequence<ChFiDS_SurfData> SeqFil = CorDat.SetOfSurfData()!;
        int Ipoin1 = CorDat.IndexFirstPointOnS1();
        int Ipoin2 = CorDat.IndexFirstPointOnS2();
        int NumEdge = 1;
        TopoDS_Vertex BoutdeVtx = new();
        int Icurv = 0;
        int Iarc1 = 0, Iarc2 = 0;
        TopAbs_Orientation trafil1 = TopAbs_FORWARD, trafil2 = TopAbs_FORWARD;
        int IcFil1, IcFil2, Isurf, Ishape1, Ishape2;
        double Pardeb = 0.0, Parfin = 0.0;
        TopAbs_Orientation ET1;
        TopOpeBRepDS_CurvePointInterference Interfp1, Interfp2;
        TopOpeBRepDS_SurfaceCurveInterference Interfc1, Interfc2;
        TopOpeBRepDS_SurfaceCurveInterference Interfc3, Interfc4;
        TopOpeBRepDS_CurvePointInterference Interfp3, Interfp4;
        TopOpeBRepDS_CurvePointInterference Interfp5, Interfp6;
        Geom2d_Curve? PCurv = null;
        TopOpeBRepDS_Curve Crv;

        NCollection_List<TopOpeBRepDS_Interference> SolidInterfs = DStr.ChangeShapeInterferences(SolidIndex);

        ChFiDS_Regul regcout = new();   // for closed and tangent CD
        ChFiDS_Regul regfilfil = new(); // for connections Surf/Surf

        ChFiDS_CommonPoint V3 = new();
        ChFiDS_CommonPoint V4 = new();

        // Nullify degenerated ChFi/Faces interferences, eap occ293
        int j;
        if (SeqFil.Length() > 1)
        {
            for (j = 1; j <= SeqFil.Length(); j++)
            {
                ChFiDS_SurfData Fd = SeqFil.Value(j);
                int onS;
                for (onS = 1; onS <= 2; onS++)
                {
                    ChFiDS_FaceInterference Fi = Fd.Interference(onS);
                    IcFil1 = Fi.LineIndex();
                    if (IcFil1 == 0)
                    {
                        continue;
                    }
                    double FiLen = Math.Abs(Fi.FirstParameter() - Fi.LastParameter());
                    if (FiLen > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        continue;
                    }
                    ref TopOpeBRepDS_Curve cc = ref DStr.ChangeCurve(IcFil1);
                    cc.ChangeCurve() = null;

                    // care of CommonPoint, eap occ354
                    if (j != 1 && j != SeqFil.Length())
                    {
                        continue;
                    }
                    bool isfirst = (j == 1);
                    int i = isfirst ? j + 1 : j - 1;
                    ref ChFiDS_CommonPoint CP1 = ref SeqFil.Value(i).ChangeVertex(isfirst, onS);
                    if (Fd.Vertex(isfirst, onS).IsOnArc() && CP1.IsOnArc())
                    {
                        ref ChFiDS_CommonPoint CP2 = ref Fd.ChangeVertex(!isfirst, onS);
                        CP1.Reset();
                        CP1.SetPoint(CP2.Point());
                        CP2.Reset();
                        CP2.SetPoint(CP1.Point());
                    }
                }
            }
        }

        for (j = 1; j <= SeqFil.Length(); j++)
        {
            ChFiDS_SurfData Fd = SeqFil.Value(j);
            Isurf = Fd.Surf();
            Ishape1 = Fd.IndexOfS1();
            Ishape2 = Fd.IndexOfS2();

            // eap, Apr 29 2002, occ 293
            // now IsInDS() returns nb of surfaces at end being in DS;
            // vars showing which end is in DS
            bool isInDS1 = false, isInDS2 = false;
            if (j <= CorDat.IsInDS(true))
            {
                isInDS1 = true;
                isInDS2 = (j + 1 <= CorDat.IsInDS(true));
            }
            if (SeqFil.Length() - j < CorDat.IsInDS(false))
            {
                isInDS2 = true;
                isInDS1 = isInDS1 || SeqFil.Length() - j + 1 < CorDat.IsInDS(false);
            }

            // creation of SolidSurfaceInterference

            TopOpeBRepDS_SolidSurfaceInterference SSI =
                new(new TopOpeBRepDS_Transition(Fd.Orientation()), TopOpeBRepDS_SOLID, SolidIndex,
                    TopOpeBRepDS_SURFACE, Isurf);

            SolidInterfs.Append(SSI);

            ChFiDS_FaceInterference Fi1 = Fd.InterferenceOnS1();
            ChFiDS_FaceInterference Fi2 = Fd.InterferenceOnS2();
            ChFiDS_CommonPoint V1 = Fd.VertexFirstOnS1();
            ChFiDS_CommonPoint V2 = Fd.VertexFirstOnS2();

            // Processing to manage double interferences
            if (j > 1)
            {
                if (V1.IsOnArc() && V3.IsOnArc() && V1.Arc().IsSame(V3.Arc()))
                {
                    // Iarc1 is initialized
                    // Iarc1 = DStr.AddShape(V1.Arc());
                    if (ChFi3d_Contains(DStr.ShapeInterferences(Iarc1), Iarc1, Ipoin1)
                        && (V1.TransitionOnArc() != V3.TransitionOnArc()))
                    {
                        Interfp1 = ChFi3d_FilPointInDS(V1.TransitionOnArc(), Iarc1, Ipoin1, V1.ParameterOnArc());
                        DStr.ChangeShapeInterferences(V1.Arc()).Append(Interfp1);
                    }
                }

                if (V2.IsOnArc() && V4.IsOnArc() && V2.Arc().IsSame(V4.Arc()))
                {
                    // Iarc2 is initialized
                    // Iarc2 = DStr.AddShape(V2.Arc());
                    if (ChFi3d_Contains(DStr.ShapeInterferences(Iarc2), Iarc2, Ipoin2)
                        && (V2.TransitionOnArc() != V4.TransitionOnArc()))
                    {
                        Interfp2 = ChFi3d_FilPointInDS(V2.TransitionOnArc(), Iarc2, Ipoin2, V2.ParameterOnArc());
                        DStr.ChangeShapeInterferences(V2.Arc()).Append(Interfp2);
                    }
                }
            }

            V3 = Fd.VertexLastOnS1();
            V4 = Fd.VertexLastOnS2();

            if (Ishape1 != 0)
            {
                if (Ishape1 > 0)
                {
                    trafil1 = DStr.Shape(Ishape1).Orientation();
                }
                else
                {
                    ChFi3d_Orientation(SolidInterfs, SolidIndex, -Ishape1, ref trafil1);
                }
                trafil1 = TopAbs.Compose(trafil1, Fd.Orientation());
                trafil1 = TopAbs.Compose(TopAbs.Reverse(Fi1.Transition()), trafil1);
                trafil2 = TopAbs.Reverse(trafil1);
            }
            else
            {
                if (Ishape2 > 0)
                {
                    trafil2 = DStr.Shape(Ishape2).Orientation();
                }
                else
                {
                    ChFi3d_Orientation(SolidInterfs, SolidIndex, -Ishape2, ref trafil2);
                }
                trafil2 = TopAbs.Compose(trafil2, Fd.Orientation());
                trafil2 = TopAbs.Compose(TopAbs.Reverse(Fi2.Transition()), trafil2);
                trafil1 = TopAbs.Reverse(trafil2);
            }

            ET1 = TopAbs.Reverse(trafil1);

            // A small paragraph to process contacts of edges, which touch
            // a vertex of the obstacle.
            if (V1.IsVertex() && Fd.IsOnCurve1())
            {
                TopoDS_Vertex vv1 = V1.Vertex();
                CutEdge(vv1, Fd, DStr, true, 1);
            }
            if (V2.IsVertex() && Fd.IsOnCurve2())
            {
                TopoDS_Vertex vv2 = V2.Vertex();
                CutEdge(vv2, Fd, DStr, true, 2);
            }
            if (V3.IsVertex() && Fd.IsOnCurve1())
            {
                TopoDS_Vertex vv3 = V3.Vertex();
                CutEdge(vv3, Fd, DStr, false, 1);
            }
            if (V4.IsVertex() && Fd.IsOnCurve2())
            {
                TopoDS_Vertex vv4 = V4.Vertex();
                CutEdge(vv4, Fd, DStr, false, 2);
            }

            if (j == 1)
            {
                isVertex1 = V1.IsVertex();
                isVertex2 = V2.IsVertex();
                Singulier_en_Bout = (V1.Point().IsEqual(V2.Point(), 0));

                if (Singulier_en_Bout)
                {
                    // Queue de Billard
                    if ((!V1.IsVertex()) || (!V2.IsVertex()))
                    {
                    }
                    else
                    {
                        isVertex1 = isVertex2 = true; // caution...
                        // The edge is removed from spine starting on this vertex.
                        TopoDS_Edge Arcspine = spine!.Edges(1);
                        BoutdeVtx = V1.Vertex();
                        int IArcspine = DStr.AddShape(Arcspine);
                        int IVtx = CorDat.IndexFirstPointOnS1();

                        TopAbs_Orientation OVtx = TopAbs_FORWARD;

                        for (ex.Init(Arcspine.Oriented(TopAbs_FORWARD), TopAbs_VERTEX); ex.More(); ex.Next())
                        {
                            if (BoutdeVtx.IsSame(ex.Current()))
                            {
                                OVtx = ex.Current().Orientation();
                                break;
                            }
                        }
                        OVtx = TopAbs.Reverse(OVtx);
                        double parVtx = BRep_Tool.Parameter(BoutdeVtx, Arcspine);
                        TopOpeBRepDS_CurvePointInterference interfv =
                            ChFi3d_FilVertexInDS(OVtx, IArcspine, IVtx, parVtx);
                        DStr.ChangeShapeInterferences(IArcspine).Append(interfv);
                    }
                }
                else
                {
                    if (V1.IsOnArc())
                    {
                        Iarc1 = DStr.AddShape(V1.Arc());
                        if (!ChFi3d_Contains(DStr.ShapeInterferences(Iarc1), Iarc1, Ipoin1))
                        {
                            Interfp1 = ChFi3d_FilPointInDS(V1.TransitionOnArc(), Iarc1, Ipoin1, V1.ParameterOnArc(),
                                                           isVertex1);
                            DStr.ChangeShapeInterferences(V1.Arc()).Append(Interfp1);
                        }
                    }

                    if (V2.IsOnArc())
                    {
                        Iarc2 = DStr.AddShape(V2.Arc());
                        if (!ChFi3d_Contains(DStr.ShapeInterferences(Iarc2), Iarc2, Ipoin2))
                        {
                            Interfp2 = ChFi3d_FilPointInDS(V2.TransitionOnArc(), Iarc2, Ipoin2, V2.ParameterOnArc(),
                                                           isVertex2);
                            DStr.ChangeShapeInterferences(V2.Arc()).Append(Interfp2);
                        }
                    }
                }

                if (!isInDS1)
                {
                    ET1 = TopAbs.Compose(ET1, CorDat.FirstPCurveOrientation());
                    Icurv = CorDat.FirstCurve();
                    if (Closed && !Singulier_en_Bout)
                    {
                        regcout.SetCurve(Icurv);
                        regcout.SetS1(Isurf, false);
                    }
                    PCurv = CorDat.FirstPCurve();
                    CorDat.FirstParameters(out Pardeb, out Parfin);

                    NCollection_List<TopOpeBRepDS_Interference> Li = DStr.ChangeCurveInterferences(Icurv);
                    if (Li.IsEmpty())
                    {
                        if (CorDat.FirstPCurveOrientation() == TopAbs_REVERSED)
                        {
                            Interfp1 = ChFi3d_FilPointInDS(TopAbs_REVERSED, Icurv, Ipoin1, Parfin, isVertex1);
                            Interfp2 = ChFi3d_FilPointInDS(TopAbs_FORWARD, Icurv, Ipoin2, Pardeb, isVertex2);
                        }
                        else
                        {
                            Interfp1 = ChFi3d_FilPointInDS(TopAbs_FORWARD, Icurv, Ipoin1, Pardeb, isVertex1);
                            Interfp2 = ChFi3d_FilPointInDS(TopAbs_REVERSED, Icurv, Ipoin2, Parfin, isVertex2);
                        }
                        Li.Append(Interfp1);
                        Li.Append(Interfp2);
                    }
                    Interfc1 = ChFi3d_FilCurveInDS(Icurv, Isurf, PCurv, ET1);
                    DStr.ChangeSurfaceInterferences(Isurf).Append(Interfc1);
                    if (Ipoin1 == Ipoin2)
                    {
                        ref TopOpeBRepDS_Curve TCurv = ref DStr.ChangeCurve(Icurv);
                        TCurv.ChangeCurve() = null;
                        TopOpeBRepDS_Interference? bidinterf = null;
                        TCurv.SetSCI(Interfc1, bidinterf);
                    }
                }
            } // End of the Initial Processing (j==1)
            else
            {
                // ---- Interference between Fillets ------

                if (!isInDS1)
                { // eap, Apr 29 2002, occ 293

                    if (Degene && isVertex1)
                    {
                        // The edge is removed from the spine starting on this vertex.
                        NumEdge++; // The previous edge of the vertex has already been found.
                        TopoDS_Edge Arcspine = spine!.Edges(NumEdge);
                        int IArcspine = DStr.AddShape(Arcspine);
                        int IVtx = DStr.AddShape(BoutdeVtx);
                        TopAbs_Orientation OVtx = TopAbs_FORWARD;
                        for (ex.Init(Arcspine.Oriented(TopAbs_FORWARD), TopAbs_VERTEX); ex.More(); ex.Next())
                        {
                            if (BoutdeVtx.IsSame(ex.Current()))
                            {
                                OVtx = ex.Current().Orientation();
                                break;
                            }
                        }
                        OVtx = TopAbs.Reverse(OVtx);
                        double parVtx = BRep_Tool.Parameter(BoutdeVtx, Arcspine);
                        TopOpeBRepDS_CurvePointInterference interfv =
                            ChFi3d_FilVertexInDS(OVtx, IArcspine, IVtx, parVtx);
                        DStr.ChangeShapeInterferences(IArcspine).Append(interfv);
                    } // End of the removal

                    Point2D UV1 =
                        Fd.InterferenceOnS1().PCurveOnSurf()!.Value(Fd.InterferenceOnS1().FirstParameter());
                    Point2D UV2 =
                        Fd.InterferenceOnS2().PCurveOnSurf()!.Value(Fd.InterferenceOnS2().FirstParameter());
                    ref TopOpeBRepDS_Curve TCurv = ref DStr.ChangeCurve(Icurv);
                    if (Degene)
                    {
                        // pcurve is associated via SCI to TopOpeBRepDSCurve.
                        ChFi3d_ComputePCurv(UV1, UV2, out Geom2d_Curve aPCurv, Pardeb, Parfin);
                        PCurv = aPCurv;
                        Interfc1 = ChFi3d_FilCurveInDS(Icurv, Isurf, PCurv, ET1);
                        DStr.ChangeSurfaceInterferences(Isurf).Append(Interfc1);
                        TCurv.ChangeCurve() = null;
                        TopOpeBRepDS_Interference? bidinterf = null;
                        TCurv.SetSCI(Interfc1, bidinterf);
                    }
                    else
                    {
                        regfilfil.SetS2(Isurf, false);
                        reglist.Append(regfilfil);
                        double tolreached;
                        ChFi3d_ComputePCurv(TCurv.ChangeCurve()!, UV1, UV2, out Geom2d_Curve aPCurv,
                                            DStr.Surface(Fd.Surf()).Surface()!, Pardeb, Parfin, tol3d,
                                            out tolreached);
                        PCurv = aPCurv;
                        TCurv.Tolerance(Math.Max(TCurv.Tolerance(), tolreached));
                        Interfc1 = ChFi3d_FilCurveInDS(Icurv, Isurf, PCurv, ET1);
                        DStr.ChangeSurfaceInterferences(Isurf).Append(Interfc1);
                    }
                }
            } // End of Interference between fillets

            // ---- Interference Fillets / Faces
            IcFil1 = Fi1.LineIndex();

            if (IcFil1 != 0)
            {
                Interfc3 = ChFi3d_FilCurveInDS(IcFil1, Isurf, Fi1.PCurveOnSurf(), trafil1);
                DStr.ChangeSurfaceInterferences(Isurf).Append(Interfc3);
                Ishape1 = Fd.IndexOfS1();
                // Case of degenerated edge : pcurve is associated via SCI
                // to TopOpeBRepDSCurve.
                ref TopOpeBRepDS_Curve cc = ref DStr.ChangeCurve(IcFil1);
                if (cc.Curve() is null)
                {
                    TopOpeBRepDS_Interference? bidinterf = null;
                    cc.SetSCI(Interfc3, bidinterf);
                }
                else
                {
                    ChFiDS_Regul regon1 = new();
                    regon1.SetCurve(IcFil1);
                    regon1.SetS1(Isurf, false);
                    if (Ishape1 < 0)
                    {
                        Ishape1 = -Ishape1;
                        regon1.SetS2(Ishape1, false);
                        Interfc1 = ChFi3d_FilCurveInDS(IcFil1, Ishape1, Fi1.PCurveOnFace(), Fi1.Transition());
                        DStr.ChangeSurfaceInterferences(Ishape1).Append(Interfc1);
                    }
                    else if (Ishape1 > 0)
                    {
                        regon1.SetS2(Ishape1, true);
                        Interfc1 = ChFi3d_FilCurveInDS(IcFil1, Ishape1, Fi1.PCurveOnFace(), Fi1.Transition());
                        DStr.ChangeShapeInterferences(Ishape1).Append(Interfc1);
                    }
                    reglist.Append(regon1);
                }
                // Indice and type of the point at End
                int ipoin;
                bool isVertex = Fd.VertexLastOnS1().IsVertex();
                if (j == SeqFil.Length())
                {
                    ipoin = CorDat.IndexLastPointOnS1();
                }
                else if (j == (SeqFil.Length() - 1) && /*Closed &&*/
                         (DStr.Curve(SeqFil.Last().InterferenceOnS1().LineIndex()).Curve() is null))
                {
                    if (Closed)
                    {
                        ipoin = CorDat.IndexFirstPointOnS1();
                        isVertex = SeqFil.Value(1).VertexFirstOnS1().IsVertex();
                    }
                    else
                    {
                        ipoin = CorDat.IndexLastPointOnS1();
                        isVertex = SeqFil.Last().VertexLastOnS1().IsVertex();
                    }
                }
                else if (DStr.Curve(IcFil1).Curve() is null)
                { // Rotation !!
                    ipoin = Ipoin1;
                    isVertex = isVertex1;
                }
                else if (((j == 1) || (j == SeqFil.Length() - 1))
                         && ((Fd.VertexLastOnS1().Point().IsEqual(SeqFil.Value(1).VertexFirstOnS1().Point(), 1.0e-7))
                             || (Fd.VertexLastOnS1().Point().IsEqual(
                                 SeqFil.Value(SeqFil.Length()).VertexLastOnS1().Point(), 1.0e-7))))
                {
                    // Case of SurfData cut in "Triangular" way.
                    ipoin = CorDat.IndexLastPointOnS1();
                    // eap, Apr 29 2002, occ 293
                }
                else if (isInDS2 && findIndexPoint(DStr, Fd, 1, out ipoin))
                {
                }
                else
                {
                    ipoin = ChFi3d_IndexPointInDS(Fd.VertexLastOnS1(), DStr);
                }

                NCollection_List<TopOpeBRepDS_Interference> Li = DStr.ChangeCurveInterferences(IcFil1);

                if (!ChFi3d_Contains(Li, IcFil1, Ipoin1))
                {
                    Interfp1 = ChFi3d_FilPointInDS(TopAbs_FORWARD, IcFil1, Ipoin1, Fi1.FirstParameter(), isVertex1);
                    DStr.ChangeCurveInterferences(IcFil1).Append(Interfp1);
                }

                if (ipoin == Ipoin1 || !ChFi3d_Contains(Li, IcFil1, ipoin))
                {
                    Interfp3 = ChFi3d_FilPointInDS(TopAbs_REVERSED, IcFil1, ipoin, Fi1.LastParameter(), isVertex);
                    DStr.ChangeCurveInterferences(IcFil1).Append(Interfp3);
                }
                Ipoin1 = ipoin;
                isVertex1 = isVertex;
            }

            IcFil2 = Fi2.LineIndex();
            if (IcFil2 != 0)
            {
                Interfc4 = ChFi3d_FilCurveInDS(IcFil2, Isurf, Fi2.PCurveOnSurf(), trafil2);
                DStr.ChangeSurfaceInterferences(Isurf).Append(Interfc4);
                Ishape2 = Fd.IndexOfS2();
                // Case of degenerated edge : pcurve is associated via SCI
                // to TopOpeBRepDSCurve.
                ref TopOpeBRepDS_Curve cc = ref DStr.ChangeCurve(IcFil2);
                if (cc.Curve() is null)
                {
                    TopOpeBRepDS_Interference? bidinterf = null;
                    cc.SetSCI(Interfc4, bidinterf);
                }
                else
                {
                    ChFiDS_Regul regon2 = new();
                    regon2.SetCurve(IcFil2);
                    regon2.SetS1(Isurf, false);
                    if (Ishape2 < 0)
                    {
                        Ishape2 = -Ishape2;
                        regon2.SetS2(Ishape2, false);
                        Interfc2 = ChFi3d_FilCurveInDS(IcFil2, Ishape2, Fi2.PCurveOnFace(), Fi2.Transition());
                        DStr.ChangeSurfaceInterferences(Ishape2).Append(Interfc2);
                    }
                    else if (Ishape2 > 0)
                    {
                        regon2.SetS2(Ishape2, true);
                        Interfc2 = ChFi3d_FilCurveInDS(IcFil2, Ishape2, Fi2.PCurveOnFace(), Fi2.Transition());
                        DStr.ChangeShapeInterferences(Ishape2).Append(Interfc2);
                    }
                    reglist.Append(regon2);
                }
                // Indice and type of the point in End
                int ipoin;
                bool isVertex = Fd.VertexLastOnS2().IsVertex();
                if (j == SeqFil.Length())
                {
                    ipoin = CorDat.IndexLastPointOnS2();
                }
                else if (j == (SeqFil.Length() - 1) && /*Closed &&*/
                         (DStr.Curve(SeqFil.Last().InterferenceOnS2().LineIndex()).Curve() is null))
                {
                    if (Closed)
                    {
                        ipoin = CorDat.IndexFirstPointOnS2();
                        isVertex = SeqFil.Value(1).VertexFirstOnS2().IsVertex();
                    }
                    else
                    {
                        ipoin = CorDat.IndexLastPointOnS2();
                        isVertex = SeqFil.Last().VertexLastOnS2().IsVertex();
                    }
                }
                else if (DStr.Curve(IcFil2).Curve() is null)
                { // Rotation !!
                    ipoin = Ipoin2;
                    isVertex = isVertex2;
                }
                else if (Fd.VertexLastOnS2().Point().IsEqual(Fd.VertexLastOnS1().Point(), 0))
                { // Pinch !!
                    ipoin = Ipoin1;
                    isVertex = isVertex1;
                }
                else if (((j == 1) || (j == SeqFil.Length() - 1))
                         && ((Fd.VertexLastOnS2().Point().IsEqual(SeqFil.Value(1).VertexFirstOnS2().Point(), 1.0e-7))
                             || (Fd.VertexLastOnS2().Point().IsEqual(
                                 SeqFil.Value(SeqFil.Length()).VertexLastOnS2().Point(), 1.0e-7))))
                {
                    // Case of SurfData cut in "Triangular" way.
                    ipoin = CorDat.IndexLastPointOnS2();
                    // eap, Apr 29 2002, occ 293
                }
                else if (isInDS2 && findIndexPoint(DStr, Fd, 2, out ipoin))
                {
                }
                else
                {
                    ipoin = ChFi3d_IndexPointInDS(Fd.VertexLastOnS2(), DStr);
                }

                NCollection_List<TopOpeBRepDS_Interference> Li = DStr.ChangeCurveInterferences(IcFil2);

                if (!ChFi3d_Contains(Li, IcFil2, Ipoin2))
                {
                    Interfp2 = ChFi3d_FilPointInDS(TopAbs_FORWARD, IcFil2, Ipoin2, Fi2.FirstParameter(), isVertex2);
                    DStr.ChangeCurveInterferences(IcFil2).Append(Interfp2);
                }

                if (ipoin == Ipoin2 || !ChFi3d_Contains(Li, IcFil2, ipoin))
                {
                    Interfp4 = ChFi3d_FilPointInDS(TopAbs_REVERSED, IcFil2, ipoin, Fi2.LastParameter(), isVertex);
                    DStr.ChangeCurveInterferences(IcFil2).Append(Interfp4);
                }
                Ipoin2 = ipoin;
                isVertex2 = isVertex;
            }

            ET1 = trafil1;
            if (j == SeqFil.Length())
            {
                if (!isInDS2)
                {
                    Icurv = CorDat.LastCurve();
                    if (Closed && !Singulier_en_Bout && (Ipoin1 != Ipoin2))
                    {
                        regcout.SetS2(Isurf, false);
                        reglist.Append(regcout);
                    }
                    PCurv = CorDat.LastPCurve();
                    ET1 = TopAbs.Compose(ET1, CorDat.LastPCurveOrientation());
                    CorDat.LastParameters(out Pardeb, out Parfin);
                    NCollection_List<TopOpeBRepDS_Interference> Li = DStr.ChangeCurveInterferences(Icurv);
                    if (Li.IsEmpty())
                    {
                        if (CorDat.LastPCurveOrientation() == TopAbs_REVERSED)
                        {
                            Interfp5 = ChFi3d_FilPointInDS(TopAbs_REVERSED, Icurv, Ipoin1, Parfin, isVertex1);
                            Interfp6 = ChFi3d_FilPointInDS(TopAbs_FORWARD, Icurv, Ipoin2, Pardeb, isVertex2);
                        }
                        else
                        {
                            Interfp5 = ChFi3d_FilPointInDS(TopAbs_FORWARD, Icurv, Ipoin1, Pardeb, isVertex1);
                            Interfp6 = ChFi3d_FilPointInDS(TopAbs_REVERSED, Icurv, Ipoin2, Parfin, isVertex2);
                        }
                        Li.Append(Interfp5);
                        Li.Append(Interfp6);
                    }
                    Interfc1 = ChFi3d_FilCurveInDS(Icurv, Isurf, PCurv, ET1);
                    DStr.ChangeSurfaceInterferences(Isurf).Append(Interfc1);
                    if (Ipoin1 == Ipoin2)
                    {
                        ref TopOpeBRepDS_Curve TCurv = ref DStr.ChangeCurve(Icurv);
                        TCurv.ChangeCurve() = null;
                        TopOpeBRepDS_Interference? bidinterf = null;
                        TCurv.SetSCI(Interfc1, bidinterf);
                        //         bidinterf = TCurv.GetSCI1();
                        //	  TCurv.SetSCI(bidinterf, Interfc1);
                    }
                }
            }
            else
            {
                //      Degene = (Fd->VertexLastOnS1().Point().IsEqual(
                //                Fd->VertexLastOnS2().Point(), 0) );

                // eap, Apr 29 2002, occ 293
                if (!isInDS2)
                {
                    Geom_Curve? C3d = null;
                    double tolreached;
                    ChFi3d_ComputeArete(Fd.VertexLastOnS1(),
                                        Fd.InterferenceOnS1().PCurveOnSurf()!.Value(
                                            Fd.InterferenceOnS1().LastParameter()),
                                        Fd.VertexLastOnS2(),
                                        Fd.InterferenceOnS2().PCurveOnSurf()!.Value(
                                            Fd.InterferenceOnS2().LastParameter()),
                                        DStr.Surface(Fd.Surf()).Surface()!, ref C3d, ref PCurv, ref Pardeb,
                                        ref Parfin, tol3d, tol2d, out tolreached, 0);
                    Crv = new TopOpeBRepDS_Curve(C3d, tolreached);
                    Icurv = DStr.AddCurve(ref Crv);
                    regfilfil.SetCurve(Icurv);
                    regfilfil.SetS1(Isurf, false);
                    Interfp5 = ChFi3d_FilPointInDS(TopAbs_FORWARD, Icurv, Ipoin1, Pardeb, isVertex1);
                    DStr.ChangeCurveInterferences(Icurv).Append(Interfp5);
                    Interfp6 = ChFi3d_FilPointInDS(TopAbs_REVERSED, Icurv, Ipoin2, Parfin, isVertex2);
                    DStr.ChangeCurveInterferences(Icurv).Append(Interfp6);
                    Interfc1 = ChFi3d_FilCurveInDS(Icurv, Isurf, PCurv, ET1);
                    DStr.ChangeSurfaceInterferences(Isurf).Append(Interfc1);
                }
            }

            Degene = V3.Point().IsEqual(V4.Point(), 0);

            // Processing of degenerated case
            if (Degene)
            {
                // Queue de Billard
                bool Vertex = (V3.IsVertex()) && (V4.IsVertex());
                if (!Vertex)
                {
                }
                else
                {
                    // The edge of the spine starting on this vertex is removed.
                    bool Trouve = false;
                    TopoDS_Edge Arcspine = new();
                    TopAbs_Orientation OVtx = TopAbs_FORWARD;
                    BoutdeVtx = V3.Vertex();

                    while (NumEdge <= spine!.NbEdges() && !Trouve)
                    {
                        Arcspine = spine.Edges(NumEdge);
                        for (ex.Init(Arcspine.Oriented(TopAbs_FORWARD), TopAbs_VERTEX); ex.More() && (!Trouve);
                             ex.Next())
                        {
                            if (BoutdeVtx.IsSame(ex.Current()))
                            {
                                OVtx = ex.Current().Orientation();
                                if (Closed && (NumEdge == 1))
                                {
                                    Trouve = (spine.NbEdges() == 1);
                                }
                                else
                                {
                                    Trouve = true;
                                }
                            }
                        }
                        if (!Trouve)
                        {
                            NumEdge++; // Go to the next edge
                        }
                    }
                    int IArcspine = DStr.AddShape(Arcspine);
                    int IVtx;
                    if (j == SeqFil.Length())
                    {
                        IVtx = CorDat.IndexLastPointOnS1();
                    }
                    else
                    {
                        IVtx = DStr.AddShape(BoutdeVtx);
                    }
                    OVtx = TopAbs.Reverse(OVtx);
                    double parVtx = BRep_Tool.Parameter(BoutdeVtx, Arcspine);
                    TopOpeBRepDS_CurvePointInterference interfv = ChFi3d_FilVertexInDS(OVtx, IArcspine, IVtx, parVtx);
                    DStr.ChangeShapeInterferences(IArcspine).Append(interfv);
                }
            } // end of degenerated case
            else if (!Closed || j != SeqFil.Length())
            {
                // Processing of interference Point / Edges
                if (V3.IsOnArc())
                {
                    if (!(V3.IsVertex() && Fd.IsOnCurve1()))
                    {
                        Iarc1 = DStr.AddShape(V3.Arc());
                        if (!ChFi3d_Contains(DStr.ShapeInterferences(Iarc1), Iarc1, Ipoin1, V3.IsVertex(), true))
                        {
                            TopOpeBRepDS_CurvePointInterference Interfpp =
                                ChFi3d_FilPointInDS(V3.TransitionOnArc(), Iarc1, Ipoin1, V3.ParameterOnArc(),
                                                    V3.IsVertex());
                            DStr.ChangeShapeInterferences(V3.Arc()).Append(Interfpp);
                        }
                    }
                }

                if (V4.IsOnArc())
                {
                    if (!(V4.IsVertex() && Fd.IsOnCurve2()))
                    {
                        Iarc2 = DStr.AddShape(V4.Arc());
                        if (!ChFi3d_Contains(DStr.ShapeInterferences(Iarc2), Iarc2, Ipoin2, V4.IsVertex(), true))
                        {
                            TopOpeBRepDS_CurvePointInterference Intfpp =
                                ChFi3d_FilPointInDS(V4.TransitionOnArc(), Iarc2, Ipoin2, V4.ParameterOnArc(),
                                                    V4.IsVertex());
                            DStr.ChangeShapeInterferences(V4.Arc()).Append(Intfpp);
                        }
                    }
                }
            }
        }
    }
}
