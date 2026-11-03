// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_LineConstructor.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_LineConstructor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using static Kerf.Geometry.SurfaceKind;
using static Occt.IntPatch_IType;
using static Occt.IntSurf_TypeTrans;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Splits an intersection line at its vertices into the parts that lie inside both surfaces' domains: each part is a
/// new line of the same kind between two vertices (with the vertices at the same place), and walking parts lying on a
/// restriction become restriction lines.
/// </summary>
/// <remarks>
/// A working object, as OCCT's: <see cref="Perform"/> fills it. The file-static functions of OCCT's .cxx are private
/// statics (IntPatch_LineConstructor.Tools.cs).
/// </remarks>
public partial class IntPatch_LineConstructor
{
    private readonly NCollection_Sequence<IntPatch_Line> slin = new();

    /// <summary>The mode is unused, as in OCCT.</summary>
    public IntPatch_LineConstructor(int mode) { }

    /// <summary>The number of lines Perform made.</summary>
    public int NbLines() => (slin.Length());

    /// <summary>The line of index index (1 to NbLines), the line itself.</summary>
    public IntPatch_Line Line(int index) => (slin.Value(index));

    /// <summary>
    /// Splits <paramref name="L"/> into its parts inside the domains <paramref name="myDom1"/> and
    /// <paramref name="myDom2"/>; <paramref name="slinref"/> holds the lines of the intersection, whose restriction lines
    /// tell walking parts that are restrictions.
    /// </summary>
    public void Perform(NCollection_Sequence<IntPatch_Line> slinref, IntPatch_Line L, Adaptor3d_Surface mySurf1,
                        Adaptor3d_TopolTool myDom1, Adaptor3d_Surface mySurf2, Adaptor3d_TopolTool myDom2,
                        double TolArc)
    {
        int i = 1, nbvtx;
        double firstp, lastp;
        double Tol = global::Kerf.Engine.Foundation.Precision.PConfusion() * 100.0; // JMB le 13 Jan 2000. Report de la correction du PRO19653
        SurfaceKind typs1 = mySurf1.GetType();
        SurfaceKind typs2 = mySurf2.GetType();

        IntPatch_IType typl = L.ArcType();
        if (typl == IntPatch_Analytic)
        {
            double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
            IntPatch_ALine ALine = (IntPatch_ALine)L;
            slin.Clear();
            nbvtx = ALine.NbVertex();
            //-- -------------------------------------------------------------------
            int[] TabIndex = new int[nbvtx + 2];
            int numline = 0;
            for (i = 1; i <= nbvtx; i++)
            {
                TabIndex[i] = 0;
            }
            //-- -------------------------------------------------------------------
            for (i = 1; i < nbvtx; i++)
            {
                IntPatch_Point ALine_Vertex_i = ALine.Vertex(i);
                IntPatch_Point ALine_Vertex_ip1 = ALine.Vertex(i + 1);
                firstp = ALine_Vertex_i.ParameterOnLine();
                lastp = ALine_Vertex_ip1.ParameterOnLine();
                if (firstp != lastp)
                {
                    double pmid = (firstp + lastp) * 0.5;
                    Point3D Pmid = ALine.Value(pmid);
                    Parameters(mySurf1, mySurf2, Pmid, ref u1, ref v1, ref u2, ref v2);
                    Recadre(mySurf1, mySurf2, ref u1, ref v1, ref u2, ref v2);
                    TopAbs_State in1, in2;
                    in1 = myDom1.Classify(new Point2D(u1, v1), Tol, false);
                    in2 = (in1 != TopAbs_OUT) ? myDom2.Classify(new Point2D(u2, v2), Tol, false) : TopAbs_OUT;
                    if (in1 == TopAbs_OUT || in2 == TopAbs_OUT)
                    {
                    }
                    else
                    {
                        TabIndex[i] = TabIndex[i + 1] = ++numline;
                        AddLine(L, i, i + 1, typs1, typs2, TabIndex, slin);
                    }
                }
            }
            //-- -------------------------------------------------------------------
            //-- On recherche les vertex interference Edge Edge Externe
            //-- Ces vertex ne figurent sur aucune ligne et sont Restriction
            //-- sur les 2 edges
            for (i = 1; i <= nbvtx; i++)
            {
                if (TabIndex[i] == 0)
                {
                    IntPatch_Point ALine_Vertex_i = ALine.Vertex(i);
                    if (ALine_Vertex_i.IsOnDomS1() && ALine_Vertex_i.IsOnDomS2())
                    {
                        TabIndex[i] = ++numline;
                        AddLine(L, i, i, typs1, typs2, TabIndex, slin);
                    }
                }
            }
            //-- -------------------------------------------------------------------
            return;
        }
        else if (typl == IntPatch_Walking)
        {
            double u1, v1, u2, v2;
            IntPatch_WLine WLine = (IntPatch_WLine)L;
            slin.Clear();
            nbvtx = WLine.NbVertex();
            //-- -------------------------------------------------------------------
            int[] TabIndex = new int[nbvtx + 2];
            int numline = 0;
            for (i = 1; i <= nbvtx; i++)
            {
                TabIndex[i] = 0;
            }
            //-- -------------------------------------------------------------------
            for (i = 1; i < nbvtx; i++)
            {
                IntPatch_Point WLineVertex_i = WLine.Vertex(i);
                IntPatch_Point WLineVertex_ip1 = WLine.Vertex(i + 1);
                firstp = WLineVertex_i.ParameterOnLine();
                lastp = WLineVertex_ip1.ParameterOnLine();
                if (firstp != lastp && !IsSegmentSmall(WLine, i, i + 1))
                {
                    int pmid;
                    pmid = (int)((firstp + lastp) / 2);
                    int int_lastp = (int)lastp;
                    int int_firstp = (int)firstp;
                    if (pmid == int_lastp)
                    {
                        pmid = int_firstp;
                    }
                    IntSurf_PntOn2S Pmid = WLine.Point(pmid);
                    Pmid.Parameters(out u1, out v1, out u2, out v2);
                    Recadre(mySurf1, mySurf2, ref u1, ref v1, ref u2, ref v2);

                    // modified by NIZHNY-MKK  Tue Apr  3 15:03:40 2001.BEGIN
                    //------------------------------------------
                    Point3D ap = new Point3D();
                    Vector3D ad1u = new Vector3D(), ad1v = new Vector3D();
                    mySurf1.D1(u1, v1, out ap, out ad1u, out ad1v);
                    double aTolerance = ComputeParametricTolerance(TolArc, ad1u, ad1v);
                    //------------------------------------------

                    TopAbs_State in1 = myDom1.Classify(new Point2D(u1, v1), aTolerance, false);
                    TopAbs_State in2 = TopAbs_OUT;
                    if (in1 != TopAbs_OUT)
                    {
                        //------------------------------------------
                        mySurf2.D1(u2, v2, out ap, out ad1u, out ad1v);
                        aTolerance = ComputeParametricTolerance(TolArc, ad1u, ad1v);
                        //------------------------------------------
                        in2 = myDom2.Classify(new Point2D(u2, v2), aTolerance, false);
                    }
                    // modified by NIZHNY-MKK  Tue Apr  3 15:06:31 2001.END

                    // modified by NIZHNY-OFV  Wed Jun 13 17:31:23 2001
                    // --purpose: If on a face (lastp-firstp) == 1,
                    //            sometimes it could mean a bad parametrisation of WLine.
                    //            In this case we try to classify the "virtual" WLine point:
                    //            the geometrical point between two vertices. This emulates
                    //            situation when (lastp-firstp) != 1.
                    if (Math.Abs(int_lastp - int_firstp) == 1)
                    {
                        double vFu1, vFv1, vFu2, vFv2, vLu1, vLv1, vLu2, vLv2;
                        IntSurf_PntOn2S vF = WLineVertex_i.PntOn2S();
                        IntSurf_PntOn2S vL = WLineVertex_ip1.PntOn2S();
                        vF.Parameters(out vFu1, out vFv1, out vFu2, out vFv2);
                        Recadre(mySurf1, mySurf2, ref vFu1, ref vFv1, ref vFu2, ref vFv2);
                        vL.Parameters(out vLu1, out vLv1, out vLu2, out vLv2);
                        Recadre(mySurf1, mySurf2, ref vLu1, ref vLv1, ref vLu2, ref vLv2);
                        if (in1 != TopAbs_IN)
                        {
                            double du, dv;
                            Point2D pvF = new Point2D(vFu1, vFv1);
                            Point2D pvL = new Point2D(vLu1, vLv1);
                            Point2D pPm = new Point2D(u1, v1);
                            double dpvFpPm = pvF.Distance(pPm);
                            double dpvLpPm = pvL.Distance(pPm);
                            if (dpvFpPm > dpvLpPm)
                            {
                                du = (vFu1 + u1) * 0.5;
                                dv = (vFv1 + v1) * 0.5;
                            }
                            else
                            {
                                du = (vLu1 + u1) * 0.5;
                                dv = (vLv1 + v1) * 0.5;
                            }
                            mySurf1.D1(du, dv, out ap, out ad1u, out ad1v);
                            aTolerance = ComputeParametricTolerance(TolArc, ad1u, ad1v);
                            in1 = myDom1.Classify(new Point2D(du, dv), aTolerance, false);
                        }
                        if (in2 != TopAbs_IN)
                        {
                            double du, dv;
                            Point2D pvF = new Point2D(vFu2, vFv2);
                            Point2D pvL = new Point2D(vLu2, vLv2);
                            Point2D pPm = new Point2D(u2, v2);
                            double dpvFpPm = pvF.Distance(pPm);
                            double dpvLpPm = pvL.Distance(pPm);
                            if (dpvFpPm > dpvLpPm)
                            {
                                du = (vFu2 + u2) * 0.5;
                                dv = (vFv2 + v2) * 0.5;
                            }
                            else
                            {
                                du = (vLu2 + u2) * 0.5;
                                dv = (vLv2 + v2) * 0.5;
                            }
                            mySurf2.D1(du, dv, out ap, out ad1u, out ad1v);
                            aTolerance = ComputeParametricTolerance(TolArc, ad1u, ad1v);
                            in2 = myDom2.Classify(new Point2D(du, dv), aTolerance, false);
                        }
                    } // end of if(std::abs(int_lastp-int_firstp) == 1)

                    if (in1 != TopAbs_OUT && in2 != TopAbs_OUT)
                    {
                        bool LignetropPetite = false;
                        IntSurf_PntOn2S Pmid1 = WLine.Point((int)firstp);
                        Pmid1.Parameters(out double u1a, out double v1a, out double u2a, out double v2a);
                        Recadre(mySurf1, mySurf2, ref u1a, ref v1a, ref u2a, ref v2a);

                        IntSurf_PntOn2S Pmid2 = WLine.Point((int)lastp);
                        Pmid2.Parameters(out double u1b, out double v1b, out double u2b, out double v2b);
                        Recadre(mySurf1, mySurf2, ref u1b, ref v1b, ref u2b, ref v2b);

                        double dd12_u = Math.Abs(u1a - u1b);
                        double dd12_v = Math.Abs(v1a - v1b);
                        if (dd12_u + dd12_v < 1e-12)
                        {
                            dd12_u = Math.Abs(u1 - u1b);
                            dd12_v = Math.Abs(v1 - v1b);
                            if (dd12_u + dd12_v < 1e-12)
                            {
                                LignetropPetite = true;
                            }
                        }
                        if (!LignetropPetite)
                        {
                            TabIndex[i] = TabIndex[i + 1] = ++numline;
                            AddLine(L, i, i + 1, typs1, typs2, TabIndex, slin);
                            TestWLineToRLine(slinref, slin, mySurf1, myDom1, mySurf2, myDom2,
                                             TolArc); //-- on teste la derniere entree de slin
                        }
                    } // end of if (in1 != TopAbs_OUT && in2 != TopAbs_OUT)
                } // end of if(firstp!=lastp && !IsSegmentSmall(WLine,i,i+1/*,TolArc*/))
            } // end of for(i=1;i<nbvtx;i++)

            //-- -------------------------------------------------------------------
            //-- On recherche les vertex interference Edge Edge Externe
            //-- Ces vertex ne figurent sur aucune ligne et sont Restriction
            //-- sur les 2 edges
            for (i = 1; i <= nbvtx; i++)
            {
                if (TabIndex[i] == 0)
                {
                    IntPatch_Point WLine_Vertex_i = WLine.Vertex(i);
                    if (WLine_Vertex_i.IsOnDomS1() && WLine_Vertex_i.IsOnDomS2())
                    {
                        TabIndex[i] = ++numline;
                        AddLine(L, i, i, typs1, typs2, TabIndex, slin);
                    }
                }
            }
            //-- -------------------------------------------------------------------
            return;
        }
        else if (typl != IntPatch_Restriction)
        { // JAG 01.07.96
            double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
            IntPatch_GLine GLine = (IntPatch_GLine)L;
            slin.Clear();
            nbvtx = GLine.NbVertex();
            //-- -------------------------------------------------------------------
            int[] TabIndex = new int[nbvtx + 2];
            int numline = 0;
            for (i = 1; i <= nbvtx; i++)
            {
                TabIndex[i] = 0;
            }
            //-- -------------------------------------------------------------------
            bool intrvtested = false;
            for (i = 1; i < nbvtx; i++)
            {
                firstp = GLine.Vertex(i).ParameterOnLine();
                lastp = GLine.Vertex(i + 1).ParameterOnLine();
                if (Math.Abs(firstp - lastp) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    intrvtested = true;
                    double pmid = (firstp + lastp) * 0.5;
                    Point3D Pmid = new Point3D();
                    if (typl == IntPatch_Lin)
                    {
                        Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, GLine.Line());
                    }
                    else if (typl == IntPatch_Circle)
                    {
                        Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, GLine.Circle());
                    }
                    else if (typl == IntPatch_Ellipse)
                    {
                        Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, GLine.Ellipse());
                    }
                    else if (typl == IntPatch_Hyperbola)
                    {
                        Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, GLine.Hyperbola());
                    }
                    else if (typl == IntPatch_Parabola)
                    {
                        Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, GLine.Parabola());
                    }
                    Parameters(mySurf1, mySurf2, Pmid, ref u1, ref v1, ref u2, ref v2);
                    Recadre(mySurf1, mySurf2, ref u1, ref v1, ref u2, ref v2);

                    Vector3D Du = new Vector3D(), Dv = new Vector3D();
                    Point3D P = new Point3D();
                    myDom1.Init();
                    if (myDom2.More()) // OCCT tests myDom2 after initialising myDom1
                    {
                        mySurf1.D1(u1, v1, out P, out Du, out Dv);
                        Tol = ComputeParametricTolerance(myDom1.Tol3d(myDom1.Value()), Du, Dv);
                    }
                    TopAbs_State in1 = myDom1.Classify(new Point2D(u1, v1), Tol, false);

                    myDom2.Init();
                    if (in1 != TopAbs_OUT && myDom2.More())
                    {
                        mySurf2.D1(u2, v2, out P, out Du, out Dv);
                        Tol = ComputeParametricTolerance(myDom2.Tol3d(myDom2.Value()), Du, Dv);
                    }
                    TopAbs_State in2 = (in1 != TopAbs_OUT) ? myDom2.Classify(new Point2D(u2, v2), Tol, false)
                                                           : TopAbs_OUT;
                    // modified by NIZHNY-OFV  Wed May 30 17:04:08 2001.BEGIN
                    // --purpose: section algo with infinite prism works now!!!
                    if (in1 == TopAbs_UNKNOWN)
                    {
                        in1 = TopAbs_OUT;
                    }
                    if (in2 == TopAbs_UNKNOWN)
                    {
                        in2 = TopAbs_OUT;
                    }
                    // modified by NIZHNY-OFV  Wed May 30 17:05:47 2001.END
                    if (in1 == TopAbs_OUT || in2 == TopAbs_OUT)
                    {
                    }
                    else
                    {
                        TabIndex[i] = TabIndex[i + 1] = ++numline;
                        AddLine(L, i, i + 1, typs1, typs2, TabIndex, slin);
                    }
                }
            }
            if (typl == IntPatch_Circle || typl == IntPatch_Ellipse)
            {
                firstp = GLine.Vertex(nbvtx).ParameterOnLine();
                lastp = M_PI + M_PI + GLine.Vertex(1).ParameterOnLine();
                double cadrinf = LocalFirstParameter(L);
                double cadrsup = LocalLastParameter(L);
                double acadr = (firstp + lastp) * 0.5;
                while (acadr < cadrinf)
                {
                    acadr += M_PI + M_PI;
                }
                while (acadr > cadrsup)
                {
                    acadr -= M_PI + M_PI;
                }
                if (acadr >= cadrinf && acadr <= cadrsup)
                {
                    if (Math.Abs(firstp - lastp) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        intrvtested = true;
                        double pmid = (firstp + lastp) * 0.5;
                        Point3D Pmid;
                        if (typl == IntPatch_Circle)
                        {
                            Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, GLine.Circle());
                        }
                        else
                        {
                            Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, GLine.Ellipse());
                        }
                        Parameters(mySurf1, mySurf2, Pmid, ref u1, ref v1, ref u2, ref v2);
                        Recadre(mySurf1, mySurf2, ref u1, ref v1, ref u2, ref v2);
                        // Tol is the last one the loop above computed, as in OCCT.
                        TopAbs_State in1 = myDom1.Classify(new Point2D(u1, v1), Tol, false);
                        TopAbs_State in2 = (in1 != TopAbs_OUT) ? myDom2.Classify(new Point2D(u2, v2), Tol, false)
                                                               : TopAbs_OUT;
                        // modified by NIZHNY-OFV  Wed May 30 17:04:08 2001.BEGIN
                        // --purpose: section algo with infinite prism works now!!!
                        if (in1 == TopAbs_UNKNOWN)
                        {
                            in1 = TopAbs_OUT;
                        }
                        if (in2 == TopAbs_UNKNOWN)
                        {
                            in2 = TopAbs_OUT;
                        }
                        // modified by NIZHNY-OFV  Wed May 30 17:05:47 2001.END
                        if (in1 == TopAbs_OUT || in2 == TopAbs_OUT)
                        {
                        }
                        else
                        {
                            TabIndex[nbvtx] = TabIndex[1] = ++numline;
                            AddLine(L, nbvtx, 1, typs1, typs2, TabIndex, slin);
                        }
                    }
                }
            }
            if (!intrvtested)
            {
                // on garde a priori. Il faudrait un point 2d sur chaque
                // surface pour prendre la decision. Sera fait dans
                // l`appelant
            }
            //-- -------------------------------------------------------------------
            //-- On recherche les vertex interference Edge Edge Externe
            //-- Ces vertex ne figurent sur aucune ligne et sont Restriction
            //-- sur les 2 edges
            for (i = 1; i <= nbvtx; i++)
            {
                if (TabIndex[i] == 0)
                {
                    IntPatch_Point GLine_Vertex_i = GLine.Vertex(i);
                    if (GLine_Vertex_i.IsOnDomS1() && GLine_Vertex_i.IsOnDomS2())
                    {
                        TabIndex[i] = ++numline;
                        AddLine(L, i, i, typs1, typs2, TabIndex, slin);
                    }
                }
            }
            //-- -------------------------------------------------------------------
            return;
        }
        else
        { //-- Restriction
            IntPatch_RLine RLine = (IntPatch_RLine)L;
            slin.Clear();
            int NbVtx = RLine.NbVertex();
            bool RestOnS1 = RLine.IsArcOnS1();
            bool RestOnS2 = RLine.IsArcOnS2();
            //-- -------------------------------------------------------------------
            int[] TabIndex = new int[NbVtx + 2];
            for (i = 1; i <= NbVtx; i++)
            {
                TabIndex[i] = 0;
            }
            //-- -------------------------------------------------------------------
            for (i = 1; i < NbVtx; i++)
            {
                IntPatch_Point Vtx1 = RLine.Vertex(i);
                IntPatch_Point Vtx2 = RLine.Vertex(i + 1);
                if (RestOnS1 && RestOnS2)
                {
                    AddLine(L, i, i + 1, typs1, typs2, TabIndex, slin);
                }
                else if (RestOnS1)
                { //-- On na classifie pas sur 1
                    double u0 = Vtx1.ParameterOnLine();
                    double u1 = Vtx2.ParameterOnLine();
                    if (Math.Abs(u1 - u0) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        double u = (999.0 * u0 + u1) * 0.001;

                        Point3D P0 = Vtx1.Value();
                        Point2D Px2d = RLine.ArcOnS1()!.Value(u);
                        Point3D Px = mySurf1.Value(Px2d.X(), Px2d.Y());
                        Vector3D P0Px = new Vector3D(P0, Px);

                        Vtx1.PntOn2S().Parameters(out double U1, out double V1, out double U2, out double V2);

                        Vector3D D1u = new Vector3D(), D1v = new Vector3D();
                        Point3D P = new Point3D();
                        mySurf2.D1(U2, V2, out P, out D1u, out D1v);
                        myDom2.Init();
                        if (myDom2.More())
                        {
                            Tol = ComputeParametricTolerance(myDom2.Tol3d(myDom2.Value()), D1u, D1v);
                        }

                        //-- le 23 mars 1999
                        TopAbs_State bornin = myDom2.Classify(new Point2D(U2, V2), Tol, false);
                        if (bornin != TopAbs_OUT)
                        {
                            Vtx2.PntOn2S().Parameters(out _, out _, out double U2t, out double V2t);
                            bornin = myDom2.Classify(new Point2D(U2t, V2t), Tol, false);
                        }
                        if (bornin == TopAbs_OUT)
                        {
                            continue;
                        }

                        //-- Attention , on faisait  une estimatoin de deltau et deltav
                        //-- Maintenant :
                        //-- POPx . D1u = deltau * D1u.D1u  + deltav * D1u.D1v
                        //-- POPx . D1v = deltau * D1u.D1v  + deltav * D1v.D1v
                        //--
                        //-- deltau=
                        double D1uD1v, TgD1u, TgD1v, D1uD1u, D1vD1v, DIS;
                        D1uD1u = D1u.Dot(D1u);
                        D1vD1v = D1v.Dot(D1v);
                        D1uD1v = D1u.Dot(D1v);
                        TgD1u = P0Px.Dot(D1u);
                        TgD1v = P0Px.Dot(D1v);
                        DIS = D1uD1u * D1vD1v - D1uD1v * D1uD1v;

                        double deltau = 1e-10;
                        double deltav = 1e-10;
                        if (DIS < -1e-10 || DIS > 1e-10)
                        {
                            deltau = (TgD1u * D1vD1v - TgD1v * D1uD1v) / DIS;
                            deltav = (TgD1v * D1uD1u - TgD1u * D1uD1v) / DIS;
                        }

                        U2 += deltau;
                        V2 += deltav;
                        if (bornin != TopAbs_OUT)
                        {
                            TopAbs_State in2 = myDom2.Classify(new Point2D(U2, V2), Tol, false);
                            deltau *= 0.05;
                            deltav *= 0.05;
                            if (in2 == TopAbs_OUT)
                            {
                                in2 = myDom2.Classify(new Point2D(U2 + deltau, V2), Tol, false);
                            }
                            if (in2 == TopAbs_OUT)
                            {
                                in2 = myDom2.Classify(new Point2D(U2 - deltau, V2), Tol, false);
                            }
                            if (in2 == TopAbs_OUT)
                            {
                                in2 = myDom2.Classify(new Point2D(U2, V2 + deltav), Tol, false);
                            }
                            if (in2 == TopAbs_OUT)
                            {
                                in2 = myDom2.Classify(new Point2D(U2, V2 - deltav), Tol, false);
                            }

                            if (in2 != TopAbs_OUT)
                            {
                                AddLine(L, i, i + 1, typs1, typs2, TabIndex, slin);
                            }
                        }
                    }
                }
                else
                {
                    double u0 = Vtx1.ParameterOnLine();
                    double u1 = Vtx2.ParameterOnLine();
                    if (Math.Abs(u1 - u0) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        double u = (999.0 * u0 + u1) * 0.001;

                        Point3D P0 = Vtx1.Value();
                        Point2D Px2d = RLine.ArcOnS2()!.Value(u);
                        Point3D Px = mySurf2.Value(Px2d.X(), Px2d.Y());
                        Vector3D P0Px = new Vector3D(P0, Px);

                        Vtx1.PntOn2S().Parameters(out double U1, out double V1, out _, out _);

                        Vector3D D1u = new Vector3D(), D1v = new Vector3D();
                        Point3D P = new Point3D();
                        mySurf1.D1(U1, V1, out P, out D1u, out D1v);
                        myDom1.Init();
                        if (myDom2.More()) // OCCT tests myDom2 after initialising myDom1
                        {
                            Tol = ComputeParametricTolerance(myDom1.Tol3d(myDom1.Value()), D1u, D1v);
                        }

                        //-- le 23 mars 1999
                        TopAbs_State bornin = myDom1.Classify(new Point2D(U1, V1), Tol, false);
                        if (bornin != TopAbs_OUT)
                        {
                            Vtx2.PntOn2S().Parameters(out double U1t, out double V1t, out _, out _);
                            bornin = myDom1.Classify(new Point2D(U1t, V1t), Tol, false);
                        }
                        if (bornin == TopAbs_OUT)
                        {
                            continue;
                        }

                        //-- Attention , on faisait  une estimatoin de deltau et deltav
                        //-- Maintenant :
                        //-- POPx . D1u = deltau * D1u.D1u  + deltav * D1u.D1v
                        //-- POPx . D1v = deltau * D1u.D1v  + deltav * D1v.D1v
                        //--
                        //-- deltau=
                        double D1uD1v, TgD1u, TgD1v, D1uD1u, D1vD1v, DIS;
                        D1uD1u = D1u.Dot(D1u);
                        D1vD1v = D1v.Dot(D1v);
                        D1uD1v = D1u.Dot(D1v);
                        TgD1u = P0Px.Dot(D1u);
                        TgD1v = P0Px.Dot(D1v);
                        DIS = D1uD1u * D1vD1v - D1uD1v * D1uD1v;

                        double deltau = 1e-10;
                        double deltav = 1e-10;
                        if (DIS < -1e-10 || DIS > 1e-10)
                        {
                            deltau = (TgD1u * D1vD1v - TgD1v * D1uD1v) / DIS;
                            deltav = (TgD1v * D1uD1u - TgD1u * D1uD1v) / DIS;
                        }

                        U1 += deltau;
                        V1 += deltav;

                        if (bornin != TopAbs_OUT)
                        {
                            TopAbs_State in2 = myDom1.Classify(new Point2D(U1, V1), Tol, false);
                            deltau *= 0.05;
                            deltav *= 0.05;
                            if (in2 == TopAbs_OUT)
                            {
                                in2 = myDom1.Classify(new Point2D(U1 + deltau, V1), Tol, false);
                            }
                            if (in2 == TopAbs_OUT)
                            {
                                in2 = myDom1.Classify(new Point2D(U1 - deltau, V1), Tol, false);
                            }
                            if (in2 == TopAbs_OUT)
                            {
                                in2 = myDom1.Classify(new Point2D(U1, V1 + deltav), Tol, false);
                            }
                            if (in2 == TopAbs_OUT)
                            {
                                in2 = myDom1.Classify(new Point2D(U1, V1 - deltav), Tol, false);
                            }

                            if (in2 != TopAbs_OUT)
                            {
                                AddLine(L, i, i + 1, typs1, typs2, TabIndex, slin);
                            }
                        }
                    }
                }
            }
        }
    }
}
