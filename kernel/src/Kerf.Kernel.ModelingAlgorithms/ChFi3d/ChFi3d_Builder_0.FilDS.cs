// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_Builder_0.cxx (ChFi3d_FilDS)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Booleans.Legacy.Data.DsKind;

namespace Kerf.Engine.Fillets;

internal static partial class Builder_0
{
    //=================================================================================================

    /// <summary>
    /// Writes the stripe <paramref name="corDat"/> into the data structure: the solid's interferences with the fillet
    /// surfaces, the curves between the fillets and on the faces with their pcurves and points, the points on the
    /// edges they cut; the regularities found go into <paramref name="reglist"/>.
    /// </summary>
    /// <remarks>
    /// Writes into the builder's data (the data structure, the surface data's common points of degenerated
    /// interferences) and, through <see cref="ChFi3d_IndexPointInDS"/>, raises the tolerance of the working shape's
    /// vertices (decision 1087).
    /// </remarks>
    internal static void ChFi3d_FilDS(int solidIndex, Stripe corDat, DsDataStructure dStr,
                                      NCollection_List<Regul> reglist, double tol3d, double tol2d)
    {
        //  BRep_Tool Outil;
        Explorer ex = new();
        Spine? spine = corDat.Spine();
        bool closed = false;
        bool degene = false, isVertex1 = false, isVertex2 = false, Singulier_en_Bout = false;
        if (spine is not null)
        {
            closed = spine.IsPeriodic;
        }
        NCollection_Sequence<SurfData> seqFil = corDat.SetOfSurfData()!;
        int ipoin1 = corDat.IndexFirstPointOnS1();
        int ipoin2 = corDat.IndexFirstPointOnS2();
        int numEdge = 1;
        Vertex boutdeVtx = new();
        int icurv = 0;
        int iarc1 = 0, iarc2 = 0;
        Orientation trafil1 = Forward, trafil2 = Forward;
        int icFil1, icFil2, isurf, ishape1, ishape2;
        double pardeb = 0.0, parfin = 0.0;
        Orientation et1;
        DsCurvePointInterference interfp1, interfp2;
        DsSurfaceCurveInterference interfc1, interfc2;
        DsSurfaceCurveInterference interfc3, interfc4;
        DsCurvePointInterference interfp3, interfp4;
        DsCurvePointInterference interfp5, interfp6;
        Curve2D? PCurv = null;
        DsCurve crv;

        NCollection_List<DsInterference> solidInterfs = dStr.ChangeShapeInterferences(solidIndex);

        Regul regcout = new();   // for closed and tangent CD
        Regul regfilfil = new(); // for connections Surf/Surf

        CommonPoint v3 = new();
        CommonPoint v4 = new();

        // Nullify degenerated ChFi/Faces interferences, eap occ293
        int j;
        if (seqFil.Length() > 1)
        {
            for (j = 1; j <= seqFil.Length(); j++)
            {
                SurfData fd = seqFil.Value(j);
                int onS;
                for (onS = 1; onS <= 2; onS++)
                {
                    FaceInterference fi = fd.Interference(onS);
                    icFil1 = fi.LineIndex();
                    if (icFil1 == 0)
                    {
                        continue;
                    }
                    double fiLen = Math.Abs(fi.FirstParameter() - fi.LastParameter());
                    if (fiLen > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        continue;
                    }
                    ref DsCurve cc = ref dStr.ChangeCurve(icFil1);
                    cc.ChangeCurve() = null;

                    // care of CommonPoint, eap occ354
                    if (j != 1 && j != seqFil.Length())
                    {
                        continue;
                    }
                    bool isfirst = (j == 1);
                    int i = isfirst ? j + 1 : j - 1;
                    ref CommonPoint cp1 = ref seqFil.Value(i).ChangeVertex(isfirst, onS);
                    if (fd.Vertex(isfirst, onS).IsOnArc && cp1.IsOnArc)
                    {
                        ref CommonPoint cp2 = ref fd.ChangeVertex(!isfirst, onS);
                        cp1.Reset();
                        cp1.SetPoint(cp2.Point());
                        cp2.Reset();
                        cp2.SetPoint(cp1.Point());
                    }
                }
            }
        }

        for (j = 1; j <= seqFil.Length(); j++)
        {
            SurfData fd = seqFil.Value(j);
            isurf = fd.Surf();
            ishape1 = fd.IndexOfS1();
            ishape2 = fd.IndexOfS2();

            // eap, Apr 29 2002, occ 293
            // now IsInDS() returns nb of surfaces at end being in DS;
            // vars showing which end is in DS
            bool isInDS1 = false, isInDS2 = false;
            if (j <= corDat.IsInDS(true))
            {
                isInDS1 = true;
                isInDS2 = (j + 1 <= corDat.IsInDS(true));
            }
            if (seqFil.Length() - j < corDat.IsInDS(false))
            {
                isInDS2 = true;
                isInDS1 = isInDS1 || seqFil.Length() - j + 1 < corDat.IsInDS(false);
            }

            // creation of SolidSurfaceInterference

            DsSolidSurfaceInterference ssi =
                new(new DsTransition(fd.Orientation()), DsKind.Solid, solidIndex,
                    DsKind.Surface, isurf);

            solidInterfs.Append(ssi);

            FaceInterference fi1 = fd.InterferenceOnS1();
            FaceInterference fi2 = fd.InterferenceOnS2();
            CommonPoint v1 = fd.VertexFirstOnS1();
            CommonPoint v2 = fd.VertexFirstOnS2();

            // Processing to manage double interferences
            if (j > 1)
            {
                if (v1.IsOnArc && v3.IsOnArc && v1.Arc().IsSame(v3.Arc()))
                {
                    // Iarc1 is initialized
                    // Iarc1 = DStr.AddShape(V1.Arc());
                    if (ChFi3d_Contains(dStr.ShapeInterferences(iarc1), iarc1, ipoin1)
                        && (v1.TransitionOnArc() != v3.TransitionOnArc()))
                    {
                        interfp1 = ChFi3d_FilPointInDS(v1.TransitionOnArc(), iarc1, ipoin1, v1.ParameterOnArc());
                        dStr.ChangeShapeInterferences(v1.Arc()).Append(interfp1);
                    }
                }

                if (v2.IsOnArc && v4.IsOnArc && v2.Arc().IsSame(v4.Arc()))
                {
                    // Iarc2 is initialized
                    // Iarc2 = DStr.AddShape(V2.Arc());
                    if (ChFi3d_Contains(dStr.ShapeInterferences(iarc2), iarc2, ipoin2)
                        && (v2.TransitionOnArc() != v4.TransitionOnArc()))
                    {
                        interfp2 = ChFi3d_FilPointInDS(v2.TransitionOnArc(), iarc2, ipoin2, v2.ParameterOnArc());
                        dStr.ChangeShapeInterferences(v2.Arc()).Append(interfp2);
                    }
                }
            }

            v3 = fd.VertexLastOnS1();
            v4 = fd.VertexLastOnS2();

            if (ishape1 != 0)
            {
                if (ishape1 > 0)
                {
                    trafil1 = dStr.Shape(ishape1).Orientation;
                }
                else
                {
                    ChFi3d_Orientation(solidInterfs, solidIndex, -ishape1, ref trafil1);
                }
                trafil1 = global::Kerf.Engine.Topology.TopAbs.Compose(trafil1, fd.Orientation());
                trafil1 = global::Kerf.Engine.Topology.TopAbs.Compose(global::Kerf.Engine.Topology.TopAbs.Reverse(fi1.Transition()), trafil1);
                trafil2 = global::Kerf.Engine.Topology.TopAbs.Reverse(trafil1);
            }
            else
            {
                if (ishape2 > 0)
                {
                    trafil2 = dStr.Shape(ishape2).Orientation;
                }
                else
                {
                    ChFi3d_Orientation(solidInterfs, solidIndex, -ishape2, ref trafil2);
                }
                trafil2 = global::Kerf.Engine.Topology.TopAbs.Compose(trafil2, fd.Orientation());
                trafil2 = global::Kerf.Engine.Topology.TopAbs.Compose(global::Kerf.Engine.Topology.TopAbs.Reverse(fi2.Transition()), trafil2);
                trafil1 = global::Kerf.Engine.Topology.TopAbs.Reverse(trafil2);
            }

            et1 = global::Kerf.Engine.Topology.TopAbs.Reverse(trafil1);

            // A small paragraph to process contacts of edges, which touch
            // a vertex of the obstacle.
            if (v1.IsVertex && fd.IsOnCurve1)
            {
                Vertex vv1 = v1.Vertex();
                CutEdge(vv1, fd, dStr, true, 1);
            }
            if (v2.IsVertex && fd.IsOnCurve2)
            {
                Vertex vv2 = v2.Vertex();
                CutEdge(vv2, fd, dStr, true, 2);
            }
            if (v3.IsVertex && fd.IsOnCurve1)
            {
                Vertex vv3 = v3.Vertex();
                CutEdge(vv3, fd, dStr, false, 1);
            }
            if (v4.IsVertex && fd.IsOnCurve2)
            {
                Vertex vv4 = v4.Vertex();
                CutEdge(vv4, fd, dStr, false, 2);
            }

            if (j == 1)
            {
                isVertex1 = v1.IsVertex;
                isVertex2 = v2.IsVertex;
                Singulier_en_Bout = (v1.Point().IsEqual(v2.Point(), 0));

                if (Singulier_en_Bout)
                {
                    // Queue de Billard
                    if ((!v1.IsVertex) || (!v2.IsVertex))
                    {
                    }
                    else
                    {
                        isVertex1 = isVertex2 = true; // caution...
                        // The edge is removed from spine starting on this vertex.
                        Edge arcspine = spine!.Edges(1);
                        boutdeVtx = v1.Vertex();
                        int iArcspine = dStr.AddShape(arcspine);
                        int iVtx = corDat.IndexFirstPointOnS1();

                        Orientation oVtx = Forward;

                        for (ex.Init(arcspine.Oriented(Forward), ShapeType.Vertex); ex.More(); ex.Next())
                        {
                            if (boutdeVtx.IsSame(ex.Current()))
                            {
                                oVtx = ex.Current().Orientation;
                                break;
                            }
                        }
                        oVtx = global::Kerf.Engine.Topology.TopAbs.Reverse(oVtx);
                        double parVtx = Tool.Parameter(boutdeVtx, arcspine);
                        DsCurvePointInterference interfv =
                            ChFi3d_FilVertexInDS(oVtx, iArcspine, iVtx, parVtx);
                        dStr.ChangeShapeInterferences(iArcspine).Append(interfv);
                    }
                }
                else
                {
                    if (v1.IsOnArc)
                    {
                        iarc1 = dStr.AddShape(v1.Arc());
                        if (!ChFi3d_Contains(dStr.ShapeInterferences(iarc1), iarc1, ipoin1))
                        {
                            interfp1 = ChFi3d_FilPointInDS(v1.TransitionOnArc(), iarc1, ipoin1, v1.ParameterOnArc(),
                                                           isVertex1);
                            dStr.ChangeShapeInterferences(v1.Arc()).Append(interfp1);
                        }
                    }

                    if (v2.IsOnArc)
                    {
                        iarc2 = dStr.AddShape(v2.Arc());
                        if (!ChFi3d_Contains(dStr.ShapeInterferences(iarc2), iarc2, ipoin2))
                        {
                            interfp2 = ChFi3d_FilPointInDS(v2.TransitionOnArc(), iarc2, ipoin2, v2.ParameterOnArc(),
                                                           isVertex2);
                            dStr.ChangeShapeInterferences(v2.Arc()).Append(interfp2);
                        }
                    }
                }

                if (!isInDS1)
                {
                    et1 = global::Kerf.Engine.Topology.TopAbs.Compose(et1, corDat.FirstPCurveOrientation());
                    icurv = corDat.FirstCurve();
                    if (closed && !Singulier_en_Bout)
                    {
                        regcout.SetCurve(icurv);
                        regcout.SetS1(isurf, false);
                    }
                    PCurv = corDat.FirstPCurve();
                    corDat.FirstParameters(out pardeb, out parfin);

                    NCollection_List<DsInterference> li = dStr.ChangeCurveInterferences(icurv);
                    if (li.IsEmpty())
                    {
                        if (corDat.FirstPCurveOrientation() == Reversed)
                        {
                            interfp1 = ChFi3d_FilPointInDS(Reversed, icurv, ipoin1, parfin, isVertex1);
                            interfp2 = ChFi3d_FilPointInDS(Forward, icurv, ipoin2, pardeb, isVertex2);
                        }
                        else
                        {
                            interfp1 = ChFi3d_FilPointInDS(Forward, icurv, ipoin1, pardeb, isVertex1);
                            interfp2 = ChFi3d_FilPointInDS(Reversed, icurv, ipoin2, parfin, isVertex2);
                        }
                        li.Append(interfp1);
                        li.Append(interfp2);
                    }
                    interfc1 = ChFi3d_FilCurveInDS(icurv, isurf, PCurv, et1);
                    dStr.ChangeSurfaceInterferences(isurf).Append(interfc1);
                    if (ipoin1 == ipoin2)
                    {
                        ref DsCurve tCurv = ref dStr.ChangeCurve(icurv);
                        tCurv.ChangeCurve() = null;
                        DsInterference? bidinterf = null;
                        tCurv.SetSCI(interfc1, bidinterf);
                    }
                }
            } // End of the Initial Processing (j==1)
            else
            {
                // ---- Interference between Fillets ------

                if (!isInDS1)
                { // eap, Apr 29 2002, occ 293

                    if (degene && isVertex1)
                    {
                        // The edge is removed from the spine starting on this vertex.
                        numEdge++; // The previous edge of the vertex has already been found.
                        Edge arcspine = spine!.Edges(numEdge);
                        int iArcspine = dStr.AddShape(arcspine);
                        int iVtx = dStr.AddShape(boutdeVtx);
                        Orientation oVtx = Forward;
                        for (ex.Init(arcspine.Oriented(Forward), ShapeType.Vertex); ex.More(); ex.Next())
                        {
                            if (boutdeVtx.IsSame(ex.Current()))
                            {
                                oVtx = ex.Current().Orientation;
                                break;
                            }
                        }
                        oVtx = global::Kerf.Engine.Topology.TopAbs.Reverse(oVtx);
                        double parVtx = Tool.Parameter(boutdeVtx, arcspine);
                        DsCurvePointInterference interfv =
                            ChFi3d_FilVertexInDS(oVtx, iArcspine, iVtx, parVtx);
                        dStr.ChangeShapeInterferences(iArcspine).Append(interfv);
                    } // End of the removal

                    Point2D uv1 =
                        fd.InterferenceOnS1().PCurveOnSurf()!.Value(fd.InterferenceOnS1().FirstParameter());
                    Point2D uv2 =
                        fd.InterferenceOnS2().PCurveOnSurf()!.Value(fd.InterferenceOnS2().FirstParameter());
                    ref DsCurve tCurv = ref dStr.ChangeCurve(icurv);
                    if (degene)
                    {
                        // pcurve is associated via SCI to TopOpeBRepDSCurve.
                        ChFi3d_ComputePCurv(uv1, uv2, out Curve2D pCurv, pardeb, parfin);
                        PCurv = pCurv;
                        interfc1 = ChFi3d_FilCurveInDS(icurv, isurf, PCurv, et1);
                        dStr.ChangeSurfaceInterferences(isurf).Append(interfc1);
                        tCurv.ChangeCurve() = null;
                        DsInterference? bidinterf = null;
                        tCurv.SetSCI(interfc1, bidinterf);
                    }
                    else
                    {
                        regfilfil.SetS2(isurf, false);
                        reglist.Append(regfilfil);
                        double tolreached;
                        ChFi3d_ComputePCurv(tCurv.ChangeCurve()!, uv1, uv2, out Curve2D pCurv,
                                            dStr.Surface(fd.Surf()).Surface()!, pardeb, parfin, tol3d,
                                            out tolreached);
                        PCurv = pCurv;
                        tCurv.Tolerance(Math.Max(tCurv.Tolerance(), tolreached));
                        interfc1 = ChFi3d_FilCurveInDS(icurv, isurf, PCurv, et1);
                        dStr.ChangeSurfaceInterferences(isurf).Append(interfc1);
                    }
                }
            } // End of Interference between fillets

            // ---- Interference Fillets / Faces
            icFil1 = fi1.LineIndex();

            if (icFil1 != 0)
            {
                interfc3 = ChFi3d_FilCurveInDS(icFil1, isurf, fi1.PCurveOnSurf(), trafil1);
                dStr.ChangeSurfaceInterferences(isurf).Append(interfc3);
                ishape1 = fd.IndexOfS1();
                // Case of degenerated edge : pcurve is associated via SCI
                // to TopOpeBRepDSCurve.
                ref DsCurve cc = ref dStr.ChangeCurve(icFil1);
                if (cc.Curve() is null)
                {
                    DsInterference? bidinterf = null;
                    cc.SetSCI(interfc3, bidinterf);
                }
                else
                {
                    Regul regon1 = new();
                    regon1.SetCurve(icFil1);
                    regon1.SetS1(isurf, false);
                    if (ishape1 < 0)
                    {
                        ishape1 = -ishape1;
                        regon1.SetS2(ishape1, false);
                        interfc1 = ChFi3d_FilCurveInDS(icFil1, ishape1, fi1.PCurveOnFace(), fi1.Transition());
                        dStr.ChangeSurfaceInterferences(ishape1).Append(interfc1);
                    }
                    else if (ishape1 > 0)
                    {
                        regon1.SetS2(ishape1, true);
                        interfc1 = ChFi3d_FilCurveInDS(icFil1, ishape1, fi1.PCurveOnFace(), fi1.Transition());
                        dStr.ChangeShapeInterferences(ishape1).Append(interfc1);
                    }
                    reglist.Append(regon1);
                }
                // Indice and type of the point at End
                int ipoin;
                bool isVertex = fd.VertexLastOnS1().IsVertex;
                if (j == seqFil.Length())
                {
                    ipoin = corDat.IndexLastPointOnS1();
                }
                else if (j == (seqFil.Length() - 1) && /*Closed &&*/
                         (dStr.Curve(seqFil.Last().InterferenceOnS1().LineIndex()).Curve() is null))
                {
                    if (closed)
                    {
                        ipoin = corDat.IndexFirstPointOnS1();
                        isVertex = seqFil.Value(1).VertexFirstOnS1().IsVertex;
                    }
                    else
                    {
                        ipoin = corDat.IndexLastPointOnS1();
                        isVertex = seqFil.Last().VertexLastOnS1().IsVertex;
                    }
                }
                else if (dStr.Curve(icFil1).Curve() is null)
                { // Rotation !!
                    ipoin = ipoin1;
                    isVertex = isVertex1;
                }
                else if (((j == 1) || (j == seqFil.Length() - 1))
                         && ((fd.VertexLastOnS1().Point().IsEqual(seqFil.Value(1).VertexFirstOnS1().Point(), 1.0e-7))
                             || (fd.VertexLastOnS1().Point().IsEqual(
                                 seqFil.Value(seqFil.Length()).VertexLastOnS1().Point(), 1.0e-7))))
                {
                    // Case of SurfData cut in "Triangular" way.
                    ipoin = corDat.IndexLastPointOnS1();
                    // eap, Apr 29 2002, occ 293
                }
                else if (isInDS2 && FindIndexPoint(dStr, fd, 1, out ipoin))
                {
                }
                else
                {
                    ipoin = ChFi3d_IndexPointInDS(fd.VertexLastOnS1(), dStr);
                }

                NCollection_List<DsInterference> li = dStr.ChangeCurveInterferences(icFil1);

                if (!ChFi3d_Contains(li, icFil1, ipoin1))
                {
                    interfp1 = ChFi3d_FilPointInDS(Forward, icFil1, ipoin1, fi1.FirstParameter(), isVertex1);
                    dStr.ChangeCurveInterferences(icFil1).Append(interfp1);
                }

                if (ipoin == ipoin1 || !ChFi3d_Contains(li, icFil1, ipoin))
                {
                    interfp3 = ChFi3d_FilPointInDS(Reversed, icFil1, ipoin, fi1.LastParameter(), isVertex);
                    dStr.ChangeCurveInterferences(icFil1).Append(interfp3);
                }
                ipoin1 = ipoin;
                isVertex1 = isVertex;
            }

            icFil2 = fi2.LineIndex();
            if (icFil2 != 0)
            {
                interfc4 = ChFi3d_FilCurveInDS(icFil2, isurf, fi2.PCurveOnSurf(), trafil2);
                dStr.ChangeSurfaceInterferences(isurf).Append(interfc4);
                ishape2 = fd.IndexOfS2();
                // Case of degenerated edge : pcurve is associated via SCI
                // to TopOpeBRepDSCurve.
                ref DsCurve cc = ref dStr.ChangeCurve(icFil2);
                if (cc.Curve() is null)
                {
                    DsInterference? bidinterf = null;
                    cc.SetSCI(interfc4, bidinterf);
                }
                else
                {
                    Regul regon2 = new();
                    regon2.SetCurve(icFil2);
                    regon2.SetS1(isurf, false);
                    if (ishape2 < 0)
                    {
                        ishape2 = -ishape2;
                        regon2.SetS2(ishape2, false);
                        interfc2 = ChFi3d_FilCurveInDS(icFil2, ishape2, fi2.PCurveOnFace(), fi2.Transition());
                        dStr.ChangeSurfaceInterferences(ishape2).Append(interfc2);
                    }
                    else if (ishape2 > 0)
                    {
                        regon2.SetS2(ishape2, true);
                        interfc2 = ChFi3d_FilCurveInDS(icFil2, ishape2, fi2.PCurveOnFace(), fi2.Transition());
                        dStr.ChangeShapeInterferences(ishape2).Append(interfc2);
                    }
                    reglist.Append(regon2);
                }
                // Indice and type of the point in End
                int ipoin;
                bool isVertex = fd.VertexLastOnS2().IsVertex;
                if (j == seqFil.Length())
                {
                    ipoin = corDat.IndexLastPointOnS2();
                }
                else if (j == (seqFil.Length() - 1) && /*Closed &&*/
                         (dStr.Curve(seqFil.Last().InterferenceOnS2().LineIndex()).Curve() is null))
                {
                    if (closed)
                    {
                        ipoin = corDat.IndexFirstPointOnS2();
                        isVertex = seqFil.Value(1).VertexFirstOnS2().IsVertex;
                    }
                    else
                    {
                        ipoin = corDat.IndexLastPointOnS2();
                        isVertex = seqFil.Last().VertexLastOnS2().IsVertex;
                    }
                }
                else if (dStr.Curve(icFil2).Curve() is null)
                { // Rotation !!
                    ipoin = ipoin2;
                    isVertex = isVertex2;
                }
                else if (fd.VertexLastOnS2().Point().IsEqual(fd.VertexLastOnS1().Point(), 0))
                { // Pinch !!
                    ipoin = ipoin1;
                    isVertex = isVertex1;
                }
                else if (((j == 1) || (j == seqFil.Length() - 1))
                         && ((fd.VertexLastOnS2().Point().IsEqual(seqFil.Value(1).VertexFirstOnS2().Point(), 1.0e-7))
                             || (fd.VertexLastOnS2().Point().IsEqual(
                                 seqFil.Value(seqFil.Length()).VertexLastOnS2().Point(), 1.0e-7))))
                {
                    // Case of SurfData cut in "Triangular" way.
                    ipoin = corDat.IndexLastPointOnS2();
                    // eap, Apr 29 2002, occ 293
                }
                else if (isInDS2 && FindIndexPoint(dStr, fd, 2, out ipoin))
                {
                }
                else
                {
                    ipoin = ChFi3d_IndexPointInDS(fd.VertexLastOnS2(), dStr);
                }

                NCollection_List<DsInterference> li = dStr.ChangeCurveInterferences(icFil2);

                if (!ChFi3d_Contains(li, icFil2, ipoin2))
                {
                    interfp2 = ChFi3d_FilPointInDS(Forward, icFil2, ipoin2, fi2.FirstParameter(), isVertex2);
                    dStr.ChangeCurveInterferences(icFil2).Append(interfp2);
                }

                if (ipoin == ipoin2 || !ChFi3d_Contains(li, icFil2, ipoin))
                {
                    interfp4 = ChFi3d_FilPointInDS(Reversed, icFil2, ipoin, fi2.LastParameter(), isVertex);
                    dStr.ChangeCurveInterferences(icFil2).Append(interfp4);
                }
                ipoin2 = ipoin;
                isVertex2 = isVertex;
            }

            et1 = trafil1;
            if (j == seqFil.Length())
            {
                if (!isInDS2)
                {
                    icurv = corDat.LastCurve();
                    if (closed && !Singulier_en_Bout && (ipoin1 != ipoin2))
                    {
                        regcout.SetS2(isurf, false);
                        reglist.Append(regcout);
                    }
                    PCurv = corDat.LastPCurve();
                    et1 = global::Kerf.Engine.Topology.TopAbs.Compose(et1, corDat.LastPCurveOrientation());
                    corDat.LastParameters(out pardeb, out parfin);
                    NCollection_List<DsInterference> li = dStr.ChangeCurveInterferences(icurv);
                    if (li.IsEmpty())
                    {
                        if (corDat.LastPCurveOrientation() == Reversed)
                        {
                            interfp5 = ChFi3d_FilPointInDS(Reversed, icurv, ipoin1, parfin, isVertex1);
                            interfp6 = ChFi3d_FilPointInDS(Forward, icurv, ipoin2, pardeb, isVertex2);
                        }
                        else
                        {
                            interfp5 = ChFi3d_FilPointInDS(Forward, icurv, ipoin1, pardeb, isVertex1);
                            interfp6 = ChFi3d_FilPointInDS(Reversed, icurv, ipoin2, parfin, isVertex2);
                        }
                        li.Append(interfp5);
                        li.Append(interfp6);
                    }
                    interfc1 = ChFi3d_FilCurveInDS(icurv, isurf, PCurv, et1);
                    dStr.ChangeSurfaceInterferences(isurf).Append(interfc1);
                    if (ipoin1 == ipoin2)
                    {
                        ref DsCurve tCurv = ref dStr.ChangeCurve(icurv);
                        tCurv.ChangeCurve() = null;
                        DsInterference? bidinterf = null;
                        tCurv.SetSCI(interfc1, bidinterf);
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
                    Curve3D? c3d = null;
                    double tolreached;
                    ChFi3d_ComputeArete(fd.VertexLastOnS1(),
                                        fd.InterferenceOnS1().PCurveOnSurf()!.Value(
                                            fd.InterferenceOnS1().LastParameter()),
                                        fd.VertexLastOnS2(),
                                        fd.InterferenceOnS2().PCurveOnSurf()!.Value(
                                            fd.InterferenceOnS2().LastParameter()),
                                        dStr.Surface(fd.Surf()).Surface()!, ref c3d, ref PCurv, ref pardeb,
                                        ref parfin, tol3d, tol2d, out tolreached, 0);
                    crv = new DsCurve(c3d, tolreached);
                    icurv = dStr.AddCurve(ref crv);
                    regfilfil.SetCurve(icurv);
                    regfilfil.SetS1(isurf, false);
                    interfp5 = ChFi3d_FilPointInDS(Forward, icurv, ipoin1, pardeb, isVertex1);
                    dStr.ChangeCurveInterferences(icurv).Append(interfp5);
                    interfp6 = ChFi3d_FilPointInDS(Reversed, icurv, ipoin2, parfin, isVertex2);
                    dStr.ChangeCurveInterferences(icurv).Append(interfp6);
                    interfc1 = ChFi3d_FilCurveInDS(icurv, isurf, PCurv, et1);
                    dStr.ChangeSurfaceInterferences(isurf).Append(interfc1);
                }
            }

            degene = v3.Point().IsEqual(v4.Point(), 0);

            // Processing of degenerated case
            if (degene)
            {
                // Queue de Billard
                bool vertex = (v3.IsVertex) && (v4.IsVertex);
                if (!vertex)
                {
                }
                else
                {
                    // The edge of the spine starting on this vertex is removed.
                    bool trouve = false;
                    Edge arcspine = new();
                    Orientation oVtx = Forward;
                    boutdeVtx = v3.Vertex();

                    while (numEdge <= spine!.EdgeCount && !trouve)
                    {
                        arcspine = spine.Edges(numEdge);
                        for (ex.Init(arcspine.Oriented(Forward), global::Kerf.Topology.ShapeType.Vertex); ex.More() && (!trouve);
                             ex.Next())
                        {
                            if (boutdeVtx.IsSame(ex.Current()))
                            {
                                oVtx = ex.Current().Orientation;
                                if (closed && (numEdge == 1))
                                {
                                    trouve = (spine.EdgeCount == 1);
                                }
                                else
                                {
                                    trouve = true;
                                }
                            }
                        }
                        if (!trouve)
                        {
                            numEdge++; // Go to the next edge
                        }
                    }
                    int iArcspine = dStr.AddShape(arcspine);
                    int iVtx;
                    if (j == seqFil.Length())
                    {
                        iVtx = corDat.IndexLastPointOnS1();
                    }
                    else
                    {
                        iVtx = dStr.AddShape(boutdeVtx);
                    }
                    oVtx = global::Kerf.Engine.Topology.TopAbs.Reverse(oVtx);
                    double parVtx = Tool.Parameter(boutdeVtx, arcspine);
                    DsCurvePointInterference interfv = ChFi3d_FilVertexInDS(oVtx, iArcspine, iVtx, parVtx);
                    dStr.ChangeShapeInterferences(iArcspine).Append(interfv);
                }
            } // end of degenerated case
            else if (!closed || j != seqFil.Length())
            {
                // Processing of interference Point / Edges
                if (v3.IsOnArc)
                {
                    if (!(v3.IsVertex && fd.IsOnCurve1))
                    {
                        iarc1 = dStr.AddShape(v3.Arc());
                        if (!ChFi3d_Contains(dStr.ShapeInterferences(iarc1), iarc1, ipoin1, v3.IsVertex, true))
                        {
                            DsCurvePointInterference interfpp =
                                ChFi3d_FilPointInDS(v3.TransitionOnArc(), iarc1, ipoin1, v3.ParameterOnArc(),
                                                    v3.IsVertex);
                            dStr.ChangeShapeInterferences(v3.Arc()).Append(interfpp);
                        }
                    }
                }

                if (v4.IsOnArc)
                {
                    if (!(v4.IsVertex && fd.IsOnCurve2))
                    {
                        iarc2 = dStr.AddShape(v4.Arc());
                        if (!ChFi3d_Contains(dStr.ShapeInterferences(iarc2), iarc2, ipoin2, v4.IsVertex, true))
                        {
                            DsCurvePointInterference intfpp =
                                ChFi3d_FilPointInDS(v4.TransitionOnArc(), iarc2, ipoin2, v4.ParameterOnArc(),
                                                    v4.IsVertex);
                            dStr.ChangeShapeInterferences(v4.Arc()).Append(intfpp);
                        }
                    }
                }
            }
        }
    }
}
