// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_LineConstructor.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_LineConstructor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// Splits an intersection line at its vertices into the parts that lie inside both surfaces' domains: each part is a
/// new line of the same kind between two vertices (with the vertices at the same place), and walking parts lying on a
/// restriction become restriction lines.
/// </summary>
/// <remarks>
/// A working object, as OCCT's: <see cref="Perform"/> fills it. The file-static functions of OCCT's .cxx are private
/// statics (IntPatch_LineConstructor.Tools.cs).
/// </remarks>
internal partial class PatchLineConstructor
{
    private readonly NCollection_Sequence<PatchLine> _slin = new();

    /// <summary>The mode is unused, as in OCCT.</summary>
    public PatchLineConstructor(int mode) { }

    /// <summary>The number of lines Perform made.</summary>
    public int LineCount => (_slin.Length());

    /// <summary>The line of index index (1 to NbLines), the line itself.</summary>
    public PatchLine Line(int index) => (_slin.Value(index));

    /// <summary>
    /// Splits <paramref name="l"/> into its parts inside the domains <paramref name="myDom1"/> and
    /// <paramref name="myDom2"/>; <paramref name="slinref"/> holds the lines of the intersection, whose restriction lines
    /// tell walking parts that are restrictions.
    /// </summary>
    public void Perform(NCollection_Sequence<PatchLine> slinref, PatchLine l, SurfaceAdaptorBase mySurf1,
                        TopolTool myDom1, SurfaceAdaptorBase mySurf2, TopolTool myDom2,
                        double tolArc)
    {
        int i = 1, nbvtx;
        double firstp, lastp;
        double tol = global::Kerf.Engine.Foundation.Precision.PConfusion() * 100.0; // JMB le 13 Jan 2000. Report de la correction du PRO19653

        PatchLineType typl = l.ArcType();
        if (typl == PatchLineType.Analytic)
        {
            double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
            ALine aLine = (ALine)l;
            _slin.Clear();
            nbvtx = aLine.VertexCount;
            //-- -------------------------------------------------------------------
            int[] tabIndex = new int[nbvtx + 2];
            int numline = 0;
            for (i = 1; i <= nbvtx; i++)
            {
                tabIndex[i] = 0;
            }
            //-- -------------------------------------------------------------------
            for (i = 1; i < nbvtx; i++)
            {
                PatchPoint ALine_Vertex_i = aLine.Vertex(i);
                PatchPoint ALine_Vertex_ip1 = aLine.Vertex(i + 1);
                firstp = ALine_Vertex_i.ParameterOnLine();
                lastp = ALine_Vertex_ip1.ParameterOnLine();
                if (firstp != lastp)
                {
                    double pmid = (firstp + lastp) * 0.5;
                    Point3D Pmid = aLine.Value(pmid);
                    Parameters(mySurf1, mySurf2, Pmid, ref u1, ref v1, ref u2, ref v2);
                    Recadre(mySurf1, mySurf2, ref u1, ref v1, ref u2, ref v2);
                    Containment in1, in2;
                    in1 = myDom1.Classify(new Point2D(u1, v1), tol, false);
                    in2 = (in1 != global::Kerf.Topology.Containment.Out) ? myDom2.Classify(new Point2D(u2, v2), tol, false) : global::Kerf.Topology.Containment.Out;
                    if (in1 == global::Kerf.Topology.Containment.Out || in2 == global::Kerf.Topology.Containment.Out)
                    {
                    }
                    else
                    {
                        tabIndex[i] = tabIndex[i + 1] = ++numline;
                        AddLine(l, i, i + 1, tabIndex, _slin);
                    }
                }
            }
            //-- -------------------------------------------------------------------
            //-- On recherche les vertex interference Edge Edge Externe
            //-- Ces vertex ne figurent sur aucune ligne et sont Restriction
            //-- sur les 2 edges
            for (i = 1; i <= nbvtx; i++)
            {
                if (tabIndex[i] == 0)
                {
                    PatchPoint ALine_Vertex_i = aLine.Vertex(i);
                    if (ALine_Vertex_i.IsOnDomS1 && ALine_Vertex_i.IsOnDomS2)
                    {
                        tabIndex[i] = ++numline;
                        AddLine(l, i, i, tabIndex, _slin);
                    }
                }
            }
            //-- -------------------------------------------------------------------
            return;
        }
        else if (typl == PatchLineType.Walking)
        {
            double u1, v1, u2, v2;
            WLine wLine = (WLine)l;
            _slin.Clear();
            nbvtx = wLine.VertexCount;
            //-- -------------------------------------------------------------------
            int[] tabIndex = new int[nbvtx + 2];
            int numline = 0;
            for (i = 1; i <= nbvtx; i++)
            {
                tabIndex[i] = 0;
            }
            //-- -------------------------------------------------------------------
            for (i = 1; i < nbvtx; i++)
            {
                PatchPoint WLineVertex_i = wLine.Vertex(i);
                PatchPoint WLineVertex_ip1 = wLine.Vertex(i + 1);
                firstp = WLineVertex_i.ParameterOnLine();
                lastp = WLineVertex_ip1.ParameterOnLine();
                if (firstp != lastp && !IsSegmentSmall(wLine, i, i + 1))
                {
                    int pmid;
                    pmid = (int)((firstp + lastp) / 2);
                    int int_lastp = (int)lastp;
                    int int_firstp = (int)firstp;
                    if (pmid == int_lastp)
                    {
                        pmid = int_firstp;
                    }
                    PntOn2S Pmid = wLine.Point(pmid);
                    Pmid.Parameters(out u1, out v1, out u2, out v2);
                    Recadre(mySurf1, mySurf2, ref u1, ref v1, ref u2, ref v2);

                    // modified by NIZHNY-MKK  Tue Apr  3 15:03:40 2001.BEGIN
                    //------------------------------------------
                    Point3D ap = new Point3D();
                    Vector3D ad1u = new Vector3D(), ad1v = new Vector3D();
                    mySurf1.D1(u1, v1, out ap, out ad1u, out ad1v);
                    double tolerance = ComputeParametricTolerance(tolArc, ad1u, ad1v);
                    //------------------------------------------

                    Containment in1 = myDom1.Classify(new Point2D(u1, v1), tolerance, false);
                    Containment in2 = global::Kerf.Topology.Containment.Out;
                    if (in1 != global::Kerf.Topology.Containment.Out)
                    {
                        //------------------------------------------
                        mySurf2.D1(u2, v2, out ap, out ad1u, out ad1v);
                        tolerance = ComputeParametricTolerance(tolArc, ad1u, ad1v);
                        //------------------------------------------
                        in2 = myDom2.Classify(new Point2D(u2, v2), tolerance, false);
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
                        PntOn2S vF = WLineVertex_i.PntOn2S();
                        PntOn2S vL = WLineVertex_ip1.PntOn2S();
                        vF.Parameters(out vFu1, out vFv1, out vFu2, out vFv2);
                        Recadre(mySurf1, mySurf2, ref vFu1, ref vFv1, ref vFu2, ref vFv2);
                        vL.Parameters(out vLu1, out vLv1, out vLu2, out vLv2);
                        Recadre(mySurf1, mySurf2, ref vLu1, ref vLv1, ref vLu2, ref vLv2);
                        if (in1 != global::Kerf.Topology.Containment.In)
                        {
                            double du, dv;
                            Point2D pvF = new Point2D(vFu1, vFv1);
                            Point2D pvL = new Point2D(vLu1, vLv1);
                            Point2D pPm = new Point2D(u1, v1);
                            double dpvFpPm = pvF.DistanceTo(pPm);
                            double dpvLpPm = pvL.DistanceTo(pPm);
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
                            tolerance = ComputeParametricTolerance(tolArc, ad1u, ad1v);
                            in1 = myDom1.Classify(new Point2D(du, dv), tolerance, false);
                        }
                        if (in2 != global::Kerf.Topology.Containment.In)
                        {
                            double du, dv;
                            Point2D pvF = new Point2D(vFu2, vFv2);
                            Point2D pvL = new Point2D(vLu2, vLv2);
                            Point2D pPm = new Point2D(u2, v2);
                            double dpvFpPm = pvF.DistanceTo(pPm);
                            double dpvLpPm = pvL.DistanceTo(pPm);
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
                            tolerance = ComputeParametricTolerance(tolArc, ad1u, ad1v);
                            in2 = myDom2.Classify(new Point2D(du, dv), tolerance, false);
                        }
                    } // end of if(std::abs(int_lastp-int_firstp) == 1)

                    if (in1 != global::Kerf.Topology.Containment.Out && in2 != global::Kerf.Topology.Containment.Out)
                    {
                        bool lignetropPetite = false;
                        PntOn2S pmid1 = wLine.Point((int)firstp);
                        pmid1.Parameters(out double u1a, out double v1a, out double u2a, out double v2a);
                        Recadre(mySurf1, mySurf2, ref u1a, ref v1a, ref u2a, ref v2a);

                        PntOn2S pmid2 = wLine.Point((int)lastp);
                        pmid2.Parameters(out double u1b, out double v1b, out double u2b, out double v2b);
                        Recadre(mySurf1, mySurf2, ref u1b, ref v1b, ref u2b, ref v2b);

                        double dd12_u = Math.Abs(u1a - u1b);
                        double dd12_v = Math.Abs(v1a - v1b);
                        if (dd12_u + dd12_v < 1e-12)
                        {
                            dd12_u = Math.Abs(u1 - u1b);
                            dd12_v = Math.Abs(v1 - v1b);
                            if (dd12_u + dd12_v < 1e-12)
                            {
                                lignetropPetite = true;
                            }
                        }
                        if (!lignetropPetite)
                        {
                            tabIndex[i] = tabIndex[i + 1] = ++numline;
                            AddLine(l, i, i + 1, tabIndex, _slin);
                            TestWLineToRLine(slinref, _slin, mySurf1, myDom1, mySurf2, myDom2,
                                             tolArc); //-- on teste la derniere entree de slin
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
                if (tabIndex[i] == 0)
                {
                    PatchPoint WLine_Vertex_i = wLine.Vertex(i);
                    if (WLine_Vertex_i.IsOnDomS1 && WLine_Vertex_i.IsOnDomS2)
                    {
                        tabIndex[i] = ++numline;
                        AddLine(l, i, i, tabIndex, _slin);
                    }
                }
            }
            //-- -------------------------------------------------------------------
            return;
        }
        else if (typl != PatchLineType.Restriction)
        { // JAG 01.07.96
            double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
            GLine gLine = (GLine)l;
            _slin.Clear();
            nbvtx = gLine.VertexCount;
            //-- -------------------------------------------------------------------
            int[] tabIndex = new int[nbvtx + 2];
            int numline = 0;
            for (i = 1; i <= nbvtx; i++)
            {
                tabIndex[i] = 0;
            }
            //-- -------------------------------------------------------------------
            bool intrvtested = false;
            for (i = 1; i < nbvtx; i++)
            {
                firstp = gLine.Vertex(i).ParameterOnLine();
                lastp = gLine.Vertex(i + 1).ParameterOnLine();
                if (Math.Abs(firstp - lastp) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    intrvtested = true;
                    double pmid = (firstp + lastp) * 0.5;
                    Point3D Pmid = new Point3D();
                    if (typl == PatchLineType.Lin)
                    {
                        Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, gLine.Line());
                    }
                    else if (typl == PatchLineType.Circle)
                    {
                        Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, gLine.Circle());
                    }
                    else if (typl == PatchLineType.Ellipse)
                    {
                        Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, gLine.Ellipse());
                    }
                    else if (typl == PatchLineType.Hyperbola)
                    {
                        Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, gLine.Hyperbola());
                    }
                    else if (typl == PatchLineType.Parabola)
                    {
                        Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, gLine.Parabola());
                    }
                    Parameters(mySurf1, mySurf2, Pmid, ref u1, ref v1, ref u2, ref v2);
                    Recadre(mySurf1, mySurf2, ref u1, ref v1, ref u2, ref v2);

                    Vector3D Du = new Vector3D(), Dv = new Vector3D();
                    Point3D p = new Point3D();
                    myDom1.Init();
                    if (myDom2.More()) // OCCT tests myDom2 after initialising myDom1
                    {
                        mySurf1.D1(u1, v1, out p, out Du, out Dv);
                        tol = ComputeParametricTolerance(myDom1.Tol3d(myDom1.Value()), Du, Dv);
                    }
                    Containment in1 = myDom1.Classify(new Point2D(u1, v1), tol, false);

                    myDom2.Init();
                    if (in1 != global::Kerf.Topology.Containment.Out && myDom2.More())
                    {
                        mySurf2.D1(u2, v2, out p, out Du, out Dv);
                        tol = ComputeParametricTolerance(myDom2.Tol3d(myDom2.Value()), Du, Dv);
                    }
                    Containment in2 = (in1 != global::Kerf.Topology.Containment.Out) ? myDom2.Classify(new Point2D(u2, v2), tol, false)
                                                           : global::Kerf.Topology.Containment.Out;
                    // modified by NIZHNY-OFV  Wed May 30 17:04:08 2001.BEGIN
                    // --purpose: section algo with infinite prism works now!!!
                    if (in1 == Unknown)
                    {
                        in1 = global::Kerf.Topology.Containment.Out;
                    }
                    if (in2 == Unknown)
                    {
                        in2 = global::Kerf.Topology.Containment.Out;
                    }
                    // modified by NIZHNY-OFV  Wed May 30 17:05:47 2001.END
                    if (in1 == global::Kerf.Topology.Containment.Out || in2 == global::Kerf.Topology.Containment.Out)
                    {
                    }
                    else
                    {
                        tabIndex[i] = tabIndex[i + 1] = ++numline;
                        AddLine(l, i, i + 1, tabIndex, _slin);
                    }
                }
            }
            if (typl == PatchLineType.Circle || typl == PatchLineType.Ellipse)
            {
                firstp = gLine.Vertex(nbvtx).ParameterOnLine();
                lastp = M_PI + M_PI + gLine.Vertex(1).ParameterOnLine();
                double cadrinf = LocalFirstParameter(l);
                double cadrsup = LocalLastParameter(l);
                double acadr = (firstp + lastp) * 0.5;
                for (int aStep = 0; acadr < cadrinf; StepLimit.Next(ref aStep))
                {
                    acadr += M_PI + M_PI;
                }
                for (int aStep = 0; acadr > cadrsup; StepLimit.Next(ref aStep))
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
                        if (typl == PatchLineType.Circle)
                        {
                            Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, gLine.Circle());
                        }
                        else
                        {
                            Pmid = global::Kerf.Engine.Geometry.ElCLib.Value(pmid, gLine.Ellipse());
                        }
                        Parameters(mySurf1, mySurf2, Pmid, ref u1, ref v1, ref u2, ref v2);
                        Recadre(mySurf1, mySurf2, ref u1, ref v1, ref u2, ref v2);
                        // Tol is the last one the loop above computed, as in OCCT.
                        Containment in1 = myDom1.Classify(new Point2D(u1, v1), tol, false);
                        Containment in2 = (in1 != global::Kerf.Topology.Containment.Out) ? myDom2.Classify(new Point2D(u2, v2), tol, false)
                                                               : global::Kerf.Topology.Containment.Out;
                        // modified by NIZHNY-OFV  Wed May 30 17:04:08 2001.BEGIN
                        // --purpose: section algo with infinite prism works now!!!
                        if (in1 == Unknown)
                        {
                            in1 = global::Kerf.Topology.Containment.Out;
                        }
                        if (in2 == Unknown)
                        {
                            in2 = global::Kerf.Topology.Containment.Out;
                        }
                        // modified by NIZHNY-OFV  Wed May 30 17:05:47 2001.END
                        if (in1 == global::Kerf.Topology.Containment.Out || in2 == global::Kerf.Topology.Containment.Out)
                        {
                        }
                        else
                        {
                            tabIndex[nbvtx] = tabIndex[1] = ++numline;
                            AddLine(l, nbvtx, 1, tabIndex, _slin);
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
                if (tabIndex[i] == 0)
                {
                    PatchPoint GLine_Vertex_i = gLine.Vertex(i);
                    if (GLine_Vertex_i.IsOnDomS1 && GLine_Vertex_i.IsOnDomS2)
                    {
                        tabIndex[i] = ++numline;
                        AddLine(l, i, i, tabIndex, _slin);
                    }
                }
            }
            //-- -------------------------------------------------------------------
            return;
        }
        else
        { //-- Restriction
            RLine rLine = (RLine)l;
            _slin.Clear();
            int nbVtx = rLine.VertexCount;
            bool restOnS1 = rLine.IsArcOnS1;
            bool restOnS2 = rLine.IsArcOnS2;
            //-- -------------------------------------------------------------------
            int[] tabIndex = new int[nbVtx + 2];
            for (i = 1; i <= nbVtx; i++)
            {
                tabIndex[i] = 0;
            }
            //-- -------------------------------------------------------------------
            for (i = 1; i < nbVtx; i++)
            {
                PatchPoint vtx1 = rLine.Vertex(i);
                PatchPoint vtx2 = rLine.Vertex(i + 1);
                if (restOnS1 && restOnS2)
                {
                    AddLine(l, i, i + 1, tabIndex, _slin);
                }
                else if (restOnS1)
                { //-- On na classifie pas sur 1
                    double u0 = vtx1.ParameterOnLine();
                    double u1 = vtx2.ParameterOnLine();
                    if (Math.Abs(u1 - u0) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        double u = (999.0 * u0 + u1) * 0.001;

                        Point3D p0 = vtx1.Value();
                        Point2D px2d = rLine.ArcOnS1()!.Value(u);
                        Point3D px = mySurf1.Value(px2d.X, px2d.Y);
                        Vector3D p0Px = new Vector3D(p0, px);

                        vtx1.PntOn2S().Parameters(out double U1, out double V1, out double U2, out double V2);

                        Vector3D d1u = new Vector3D(), d1v = new Vector3D();
                        Point3D p = new Point3D();
                        mySurf2.D1(U2, V2, out p, out d1u, out d1v);
                        myDom2.Init();
                        if (myDom2.More())
                        {
                            tol = ComputeParametricTolerance(myDom2.Tol3d(myDom2.Value()), d1u, d1v);
                        }

                        //-- le 23 mars 1999
                        Containment bornin = myDom2.Classify(new Point2D(U2, V2), tol, false);
                        if (bornin != global::Kerf.Topology.Containment.Out)
                        {
                            vtx2.PntOn2S().Parameters(out _, out _, out double u2t, out double v2t);
                            bornin = myDom2.Classify(new Point2D(u2t, v2t), tol, false);
                        }
                        if (bornin == global::Kerf.Topology.Containment.Out)
                        {
                            continue;
                        }

                        //-- Attention , on faisait  une estimatoin de deltau et deltav
                        //-- Maintenant :
                        //-- POPx . D1u = deltau * D1u.D1u  + deltav * D1u.D1v
                        //-- POPx . D1v = deltau * D1u.D1v  + deltav * D1v.D1v
                        //--
                        //-- deltau=
                        double d1uD1v, tgD1u, tgD1v, d1uD1u, d1vD1v, dis;
                        d1uD1u = d1u.Dot(d1u);
                        d1vD1v = d1v.Dot(d1v);
                        d1uD1v = d1u.Dot(d1v);
                        tgD1u = p0Px.Dot(d1u);
                        tgD1v = p0Px.Dot(d1v);
                        dis = d1uD1u * d1vD1v - d1uD1v * d1uD1v;

                        double deltau = 1e-10;
                        double deltav = 1e-10;
                        if (dis < -1e-10 || dis > 1e-10)
                        {
                            deltau = (tgD1u * d1vD1v - tgD1v * d1uD1v) / dis;
                            deltav = (tgD1v * d1uD1u - tgD1u * d1uD1v) / dis;
                        }

                        U2 += deltau;
                        V2 += deltav;
                        if (bornin != global::Kerf.Topology.Containment.Out)
                        {
                            Containment in2 = myDom2.Classify(new Point2D(U2, V2), tol, false);
                            deltau *= 0.05;
                            deltav *= 0.05;
                            if (in2 == global::Kerf.Topology.Containment.Out)
                            {
                                in2 = myDom2.Classify(new Point2D(U2 + deltau, V2), tol, false);
                            }
                            if (in2 == global::Kerf.Topology.Containment.Out)
                            {
                                in2 = myDom2.Classify(new Point2D(U2 - deltau, V2), tol, false);
                            }
                            if (in2 == global::Kerf.Topology.Containment.Out)
                            {
                                in2 = myDom2.Classify(new Point2D(U2, V2 + deltav), tol, false);
                            }
                            if (in2 == global::Kerf.Topology.Containment.Out)
                            {
                                in2 = myDom2.Classify(new Point2D(U2, V2 - deltav), tol, false);
                            }

                            if (in2 != global::Kerf.Topology.Containment.Out)
                            {
                                AddLine(l, i, i + 1, tabIndex, _slin);
                            }
                        }
                    }
                }
                else
                {
                    double u0 = vtx1.ParameterOnLine();
                    double u1 = vtx2.ParameterOnLine();
                    if (Math.Abs(u1 - u0) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        double u = (999.0 * u0 + u1) * 0.001;

                        Point3D p0 = vtx1.Value();
                        Point2D px2d = rLine.ArcOnS2()!.Value(u);
                        Point3D px = mySurf2.Value(px2d.X, px2d.Y);
                        Vector3D p0Px = new Vector3D(p0, px);

                        vtx1.PntOn2S().Parameters(out double U1, out double V1, out _, out _);

                        Vector3D d1u = new Vector3D(), d1v = new Vector3D();
                        Point3D p = new Point3D();
                        mySurf1.D1(U1, V1, out p, out d1u, out d1v);
                        myDom1.Init();
                        if (myDom2.More()) // OCCT tests myDom2 after initialising myDom1
                        {
                            tol = ComputeParametricTolerance(myDom1.Tol3d(myDom1.Value()), d1u, d1v);
                        }

                        //-- le 23 mars 1999
                        Containment bornin = myDom1.Classify(new Point2D(U1, V1), tol, false);
                        if (bornin != global::Kerf.Topology.Containment.Out)
                        {
                            vtx2.PntOn2S().Parameters(out double u1t, out double v1t, out _, out _);
                            bornin = myDom1.Classify(new Point2D(u1t, v1t), tol, false);
                        }
                        if (bornin == global::Kerf.Topology.Containment.Out)
                        {
                            continue;
                        }

                        //-- Attention , on faisait  une estimatoin de deltau et deltav
                        //-- Maintenant :
                        //-- POPx . D1u = deltau * D1u.D1u  + deltav * D1u.D1v
                        //-- POPx . D1v = deltau * D1u.D1v  + deltav * D1v.D1v
                        //--
                        //-- deltau=
                        double d1uD1v, tgD1u, tgD1v, d1uD1u, d1vD1v, dis;
                        d1uD1u = d1u.Dot(d1u);
                        d1vD1v = d1v.Dot(d1v);
                        d1uD1v = d1u.Dot(d1v);
                        tgD1u = p0Px.Dot(d1u);
                        tgD1v = p0Px.Dot(d1v);
                        dis = d1uD1u * d1vD1v - d1uD1v * d1uD1v;

                        double deltau = 1e-10;
                        double deltav = 1e-10;
                        if (dis < -1e-10 || dis > 1e-10)
                        {
                            deltau = (tgD1u * d1vD1v - tgD1v * d1uD1v) / dis;
                            deltav = (tgD1v * d1uD1u - tgD1u * d1uD1v) / dis;
                        }

                        U1 += deltau;
                        V1 += deltav;

                        if (bornin != global::Kerf.Topology.Containment.Out)
                        {
                            Containment in2 = myDom1.Classify(new Point2D(U1, V1), tol, false);
                            deltau *= 0.05;
                            deltav *= 0.05;
                            if (in2 == global::Kerf.Topology.Containment.Out)
                            {
                                in2 = myDom1.Classify(new Point2D(U1 + deltau, V1), tol, false);
                            }
                            if (in2 == global::Kerf.Topology.Containment.Out)
                            {
                                in2 = myDom1.Classify(new Point2D(U1 - deltau, V1), tol, false);
                            }
                            if (in2 == global::Kerf.Topology.Containment.Out)
                            {
                                in2 = myDom1.Classify(new Point2D(U1, V1 + deltav), tol, false);
                            }
                            if (in2 == global::Kerf.Topology.Containment.Out)
                            {
                                in2 = myDom1.Classify(new Point2D(U1, V1 - deltav), tol, false);
                            }

                            if (in2 != global::Kerf.Topology.Containment.Out)
                            {
                                AddLine(l, i, i + 1, tabIndex, _slin);
                            }
                        }
                    }
                }
            }
        }
    }
}
