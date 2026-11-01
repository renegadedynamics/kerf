// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_DSFiller.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_DSFiller.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Contap_IType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Fills HLRTopoBRep_Data for a shape: the contours (outlines) of each face as new edges with their vertices, the
/// isolines, and the splits of the edges at the vertices inserted on them.
/// </summary>
/// <remarks>
/// OCCT's static class. It writes into the shape it is given (the point representations of the end vertices of the
/// edges it splits, in <c>ProcessEdges</c>), so the shape must be unfrozen: HLRTopoBRep_OutLiner gives it its
/// working copy (decision 1095); a frozen shape throws TopoDS_LockedShape. The contours are built as OCCT does
/// (decision 1084). The commented-out vertex map and interpolation of short walking lines are left out.
/// </remarks>
public static class HLRTopoBRep_DSFiller
{
    /// <summary>Outlines each face of S with FO into DS (nbIso isolines per direction), then splits the edges.</summary>
    public static void Insert(in TopoDS_Shape S, Contap_Contour FO, HLRTopoBRep_Data DS,
                              NCollection_DataMap<TopoDS_Shape, BRepTopAdaptor_Tool> MST, int nbIso)
    {
        NCollection_Map<TopoDS_Shape> ShapeMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp_Explorer ex = new(S, TopAbs_FACE);
        DS.Clear();
        bool withPCurve = true; // instead of nbIso != 0;
        int f = 0;

        while (ex.More())
        {
            if (ShapeMap.Add(ex.Current()))
            {
                f++;
                TopoDS_Face S1 = TopoDS.Face(ex.Current());
                S1.Orientation(TopAbs_FORWARD);
                BRepTopAdaptor_TopolTool Domain;
                Adaptor3d_Surface? Surface;
                if (MST.IsBound(S1))
                {
                    BRepTopAdaptor_Tool BRT = MST.ChangeFind(S1);
                    Domain = BRT.GetTopolTool();
                    Surface = BRT.GetSurface();
                }
                else
                {
                    BRepTopAdaptor_Tool BRT = new(S1, Precision.PConfusion());
                    MST.Bind(S1, BRT);
                    Domain = BRT.GetTopolTool();
                    Surface = BRT.GetSurface();
                }
                FO.Perform(Surface!, Domain);
                if (FO.IsDone())
                {
                    if (!FO.IsEmpty())
                    {
                        InsertFace(f, S1, FO, DS, withPCurve);
                    }
                }

                if (nbIso != 0)
                {
                    HLRTopoBRep_FaceIsoLiner.Perform(f, S1, DS, nbIso);
                }
            }
            ex.Next();
        }
        ProcessEdges(DS);
    }

    /// <summary>Builds the edges of the contours of F and records them as F's outlines and internal outlines.</summary>
    private static void InsertFace(int FI, in TopoDS_Face F, Contap_Contour FO, HLRTopoBRep_Data DS, bool withPCurve)
    {
        double tol = BRep_Tool.Tolerance(F);
        NCollection_List<TopoDS_Shape> IntL = DS.AddIntL(F);
        NCollection_List<TopoDS_Shape> OutL = DS.AddOutL(F);

        TopoDS_Vertex VF = new(), VL = new();
        // OCCT's commented-out map of the vertices of the closed edges is left out.

        int NbLines = FO.NbLines();
        int CurLine;
        for (CurLine = 1; CurLine <= NbLines; CurLine++)
        {
            Contap_Line Line = FO.Line(CurLine);
            int NbPoints = Line.NbVertex();
            int CurPoint;
            if (Line.TypeContour() == Contap_Restriction)
            {
                // OutLine on restriction
                TopoDS_Edge E = ((BRepAdaptor_Curve2d)Line.Arc()).Edge();
                OutL.Append(E);
                TopExp.Vertices(E, out VF, out VL);
                // insert the Internal points.

                for (CurPoint = 1; CurPoint <= NbPoints; CurPoint++)
                {
                    Contap_Point P = Line.Vertex(CurPoint);
                    if (P.IsInternal())
                    {
                        if (P.Value().IsEqual(BRep_Tool.Pnt(VF), BRep_Tool.Tolerance(VF)))
                        {
                            if (P.Value().IsEqual(BRep_Tool.Pnt(VL), BRep_Tool.Tolerance(VL)))
                            {
                                InsertVertex(P, tol, E, DS);
                            }
                        }
                    }
                }
            }
            else
            {
                for (CurPoint = 1; CurPoint <= NbPoints; CurPoint++)
                {
                    Contap_Point PF = Line.Vertex(CurPoint);
                    if (PF.IsInternal() && CurPoint != 1)
                    {
                        VF = VL;
                    }
                    else
                    {
                        VF = MakeVertex(PF, tol, DS);
                    }
                    double parF = PF.ParameterOnLine();

                    if (CurPoint < NbPoints)
                    {
                        Contap_Point PL = Line.Vertex(CurPoint + 1);
                        VL = MakeVertex(PL, tol, DS);
                        double parL = PL.ParameterOnLine();

                        if ((parL - parF) > Precision.PConfusion())
                        {

                            Geom_Curve? C = null;
                            Geom2d_Curve? C2d = null;
                            double first = parF;
                            double last = parL;
                            bool InsuffisantNumberOfPoints = false;

                            switch (Line.TypeContour())
                            {

                                case Contap_Lin:
                                {
                                    C = new Geom_Line(Line.Line());
                                    if (withPCurve)
                                    {
                                        Geom_Surface S = BRep_Tool.Surface(F)!;
                                        double Tol = 1e-7;
                                        C2d = GeomProjLib.Curve2d(C, first, last, S, ref Tol);
                                    }
                                }
                                break;

                                case Contap_Circle:
                                {
                                    C = new Geom_Circle(Line.Circle());
                                    if (withPCurve)
                                    {
                                        Geom_Surface S = BRep_Tool.Surface(F, out TopLoc_Location Loc)!;
                                        if (!Loc.IsIdentity())
                                        {
                                            S = (Geom_Surface)S.Transformed(Loc.Transformation());
                                        }
                                        double Tol = 1e-7;
                                        C2d = GeomProjLib.Curve2d(C, first, last, S, ref Tol);
                                    }
                                }
                                break;

                                case Contap_Walking:
                                {
                                    // copy the points
                                    int ipF = (int)(parF);
                                    int ipL = (int)(parL);

                                    if (ipL - ipF < 1)
                                    {
                                        InsuffisantNumberOfPoints = true;
                                        // OCCT_DEBUG trace left out
                                    }

                                    // OCCT's commented-out interpolation of the lines of fewer than 6 points is left out.

                                    else if (ipL - ipF < 5)
                                    {
                                        int nbp = ipL - ipF + 1;
                                        NCollection_Array1<double> knots = new(1, nbp);
                                        NCollection_Array1<int> mults = new(1, nbp);
                                        NCollection_Array1<gp_Pnt> Points = new(1, nbp);

                                        for (int i = 1; i <= nbp; i++)
                                        {
                                            knots.SetValue(i, (double)i);
                                            mults.SetValue(i, 1);
                                            Points.SetValue(i, Line.Point(i + ipF - 1).Value());
                                        }
                                        mults[1] = mults[nbp] = 2;
                                        C = new Geom_BSplineCurve(Points, knots, mults, 1);

                                        if (withPCurve)
                                        {
                                            NCollection_Array1<gp_Pnt2d> Points2d = new(1, nbp);
                                            for (int i = 1; i <= nbp; i++)
                                            {
                                                double u, v;
                                                Line.Point(i + ipF - 1).ParametersOnS2(out u, out v);
                                                Points2d.SetValue(i, new gp_Pnt2d(u, v));
                                            }
                                            C2d = new Geom2d_BSplineCurve(Points2d, knots, mults, 1);
                                        }
                                        first = 1;
                                        last = nbp;
                                    }
                                    else
                                    {
                                        int nbp = ipL - ipF + 1;
                                        NCollection_Array1<double> knots = new(1, nbp);
                                        NCollection_Array1<int> mults = new(1, nbp);
                                        NCollection_Array1<gp_Pnt> Points = new(1, nbp);

                                        double Maxx, Maxy, Maxz, Maxu, Maxv;
                                        double Minx, Miny, Minz, Minu, Minv;
                                        Maxx = Maxy = Maxz = Maxu = Maxv = -RealLast();
                                        Minx = Miny = Minz = Minu = Minv = RealLast();

                                        for (int i = 1; i <= nbp; i++)
                                        {
                                            knots.SetValue(i, (double)i);
                                            mults.SetValue(i, 1);
                                            gp_Pnt P = Line.Point(i + ipF - 1).Value();
                                            if (P.X() < Minx)
                                            {
                                                Minx = P.X();
                                            }
                                            if (P.Y() < Miny)
                                            {
                                                Miny = P.Y();
                                            }
                                            if (P.Z() < Minz)
                                            {
                                                Minz = P.Z();
                                            }
                                            if (P.X() > Maxx)
                                            {
                                                Maxx = P.X();
                                            }
                                            if (P.Y() > Maxy)
                                            {
                                                Maxy = P.Y();
                                            }
                                            if (P.Z() > Maxz)
                                            {
                                                Maxz = P.Z();
                                            }
                                            Points.SetValue(i, P);
                                        }
                                        mults[1] = mults[nbp] = 2;
                                        Geom_BSplineCurve AppC;
                                        Geom2d_BSplineCurve? AppC2d = null;
                                        AppC = new Geom_BSplineCurve(Points, knots, mults, 1);

                                        if (withPCurve)
                                        {
                                            NCollection_Array1<gp_Pnt2d> Points2d = new(1, nbp);
                                            for (int i = 1; i <= nbp; i++)
                                            {
                                                double u, v;
                                                Line.Point(i + ipF - 1).ParametersOnS2(out u, out v);
                                                if (u < Minu)
                                                {
                                                    Minu = u;
                                                }
                                                if (v < Minv)
                                                {
                                                    Minv = v;
                                                }
                                                if (u > Maxu)
                                                {
                                                    Maxu = u;
                                                }
                                                if (v > Maxv)
                                                {
                                                    Maxv = v;
                                                }
                                                Points2d.SetValue(i, new gp_Pnt2d(u, v));
                                            }
                                            AppC2d = new Geom2d_BSplineCurve(Points2d, knots, mults, 1);
                                        }
                                        first = 1;
                                        last = nbp;

                                        BRepApprox_ApproxLine AppLine;
                                        Geom2d_BSplineCurve? CNull = null;
                                        AppLine = new BRepApprox_ApproxLine(AppC, AppC2d, CNull);

                                        int dmin = 4, dmax = 8, niter = 0;
                                        bool tg = false;
                                        BRepApprox_Approx Approx = new();
                                        double TOL3d, TOL2d, TOL = 0.0001;

                                        Maxx -= Minx;
                                        Maxy -= Miny;
                                        Maxz -= Minz;
                                        Maxu -= Minu;
                                        Maxv -= Minv;
                                        if (Maxy > Maxx)
                                        {
                                            Maxx = Maxy;
                                        }
                                        if (Maxz > Maxx)
                                        {
                                            Maxx = Maxy; // OCCT: Maxy where Maxz was meant
                                        }
                                        if (Maxv > Maxu)
                                        {
                                            Maxu = Maxv;
                                        }

                                        TOL3d = TOL * Maxx;
                                        if (TOL3d < 1e-12)
                                        {
                                            TOL3d = 1e-12;
                                        }
                                        else if (TOL3d > 0.1)
                                        {
                                            TOL3d = 0.1;
                                        }
                                        TOL2d = TOL * Maxu;
                                        if (TOL2d < 1e-12)
                                        {
                                            TOL2d = 1e-12;
                                        }
                                        else if (TOL2d > 0.1)
                                        {
                                            TOL2d = 0.1;
                                        }

                                        // std::cout<<"\nHLRTopoBRep_DSFiller : nbp="<<nbp<<"  Tol3d="<<TOL3d<<"
                                        // Tol2d="<<TOL2d<<std::endl;

                                        Approx.SetParameters(TOL3d, TOL2d, dmin, dmax, niter, 30, tg);
                                        Approx.Perform(AppLine, true, true, false, 1, nbp);
                                        if (!Approx.IsDone())
                                        {
                                            C = AppC;
                                            C2d = AppC2d;
                                            first = 1;
                                            last = nbp;
                                        }
                                        else
                                        {
                                            AppParCurves_MultiBSpCurve AppVal = Approx.Value(1);
                                            NCollection_Array1<gp_Pnt> poles3d = new(1, AppVal.NbPoles());
                                            AppVal.Curve(1, poles3d);
                                            C = new Geom_BSplineCurve(poles3d, AppVal.Knots(), AppVal.Multiplicities(),
                                                                      AppVal.Degree());

                                            AppParCurves_MultiBSpCurve AppVal2 = Approx.Value(2);
                                            NCollection_Array1<gp_Pnt2d> poles2d = new(1, AppVal2.NbPoles());
                                            AppVal2.Curve(2, poles2d);
                                            C2d = new Geom2d_BSplineCurve(poles2d, AppVal2.Knots(),
                                                                          AppVal2.Multiplicities(), AppVal2.Degree());
                                            first = C2d.FirstParameter();
                                            last = C2d.LastParameter();
                                        }
                                    }
                                }
                                break;

                                case Contap_Restriction:
                                {
                                    throw new Standard_ProgramError("HLRTopoBRep_DSFiller::InsertFace : Restriction");
                                }
                            }

                            // compute the PCurve
                            // make the edge
                            if (!InsuffisantNumberOfPoints)
                            {
                                TopoDS_Edge E = new();
                                BRep_Builder B = new();
                                B.MakeEdge(ref E, C, tol);
                                VF.Orientation(TopAbs_FORWARD);
                                VL.Orientation(TopAbs_REVERSED);
                                B.Add(E, VF);
                                B.Add(E, VL);
                                B.Range(E, first, last);

                                if (C2d is not null)
                                {
                                    B.UpdateEdge(E, C2d, F, BRep_Tool.Tolerance(F));
                                }

                                // add the edge in the DS
                                if (!E.IsNull())
                                {
                                    IntL.Append(E);
                                }
                            }
                        }
                    }
                }
            }
        }

        // Correction of internal outlines: unite coinciding vertices
        double SqTol = tol * tol;

        NCollection_List<TopoDS_Shape>.Iterator itl1 = new(IntL);
        for (; itl1.More(); itl1.Next())
        {
            TopoDS_Edge anIntLine = TopoDS.Edge(itl1.Value());
            anIntLine.Orientation(TopAbs_FORWARD);
            TopoDS_Vertex[] aVer = new TopoDS_Vertex[2];
            TopExp.Vertices(anIntLine, out aVer[0], out aVer[1]);
            NCollection_List<TopoDS_Shape>.Iterator itl2 = new(itl1);
            for (; itl2.More(); itl2.Next())
            {
                TopoDS_Edge anIntLine2 = TopoDS.Edge(itl2.Value());
                anIntLine2.Orientation(TopAbs_FORWARD);
                if (anIntLine2.IsSame(anIntLine))
                {
                    continue;
                }
                TopoDS_Vertex[] aVer2 = new TopoDS_Vertex[2];
                TopExp.Vertices(anIntLine2, out aVer2[0], out aVer2[1]);
                for (int i = 0; i < 2; i++)
                {
                    if (i == 1 && aVer[0].IsSame(aVer[1]))
                    {
                        continue;
                    }
                    gp_Pnt Pnt1 = BRep_Tool.Pnt(aVer[i]);
                    for (int j = 0; j < 2; j++)
                    {
                        if (aVer[i].IsSame(aVer2[j]))
                        {
                            continue;
                        }
                        gp_Pnt Pnt2 = BRep_Tool.Pnt(aVer2[j]);
                        if (Pnt1.SquareDistance(Pnt2) <= SqTol)
                        {
                            BRep_Builder aBB = new();
                            aBB.Remove(anIntLine2, aVer2[j]);
                            aVer[i].Orientation((j == 0) ? TopAbs_FORWARD : TopAbs_REVERSED);
                            aBB.Add(anIntLine2, aVer[i]);
                        }
                    }
                }
            }
        }
    }

    /// <summary>The vertex of a contour point: the face's vertex, a vertex inserted on the arc it lies on, or a new vertex.</summary>
    private static TopoDS_Vertex MakeVertex(in Contap_Point P, double tol, HLRTopoBRep_Data DS)
    {
        BRep_Builder B = new();
        TopoDS_Vertex V = new();
        if (P.IsVertex())
        {
            V = ((BRepTopAdaptor_HVertex)P.Vertex()).Vertex();
            DS.AddOutV(V);
        }
        else
        {
            // if on arc, insert in the DS
            if (P.IsOnArc())
            {
                TopoDS_Edge E = ((BRepAdaptor_Curve2d)P.Arc()).Edge();
                double Par = P.ParameterOnArc();
                gp_Pnt P3d = P.Value();

                for (DS.InitVertex(E); DS.MoreVertex(); DS.NextVertex())
                {
                    TopoDS_Vertex curV = DS.Vertex();
                    double curP = DS.Parameter();
                    gp_Pnt PPP = BRep_Tool.Pnt(curV);
                    double TTT = BRep_Tool.Tolerance(curV);
                    if (P3d.IsEqual(PPP, TTT))
                    {
                        V = curV;
                        break;
                    }
                    else if (Par < curP)
                    {
                        B.MakeVertex(ref V, P.Value(), tol);
                        DS.InsertBefore(V, Par);
                        break;
                    }
                }
                if (!DS.MoreVertex())
                {
                    B.MakeVertex(ref V, P.Value(), tol);
                    DS.Append(V, Par);
                }
                DS.AddOutV(V);
            }
            // if internal create a vertex and insert in the DS
            else
            {
                B.MakeVertex(ref V, P.Value(), tol);
                if (P.IsInternal())
                {
                    DS.AddIntV(V);
                }
                else
                {
                    DS.AddOutV(V);
                }
            }
        }
        return V;
    }

    /// <summary>Inserts the vertex of a contour point on the edge E.</summary>
    private static void InsertVertex(in Contap_Point P, double tol, in TopoDS_Edge E, HLRTopoBRep_Data DS)
    {
        BRep_Builder B = new();
        TopoDS_Vertex V = new();

        if (P.IsVertex())
        {
            V = ((BRepTopAdaptor_HVertex)P.Vertex()).Vertex();
        }
        else
        {
            double Par = P.ParameterOnLine();

            for (DS.InitVertex(E); DS.MoreVertex(); DS.NextVertex())
            {
                TopoDS_Vertex curV = DS.Vertex();
                double curP = DS.Parameter();
                if (P.Value().IsEqual(BRep_Tool.Pnt(curV), BRep_Tool.Tolerance(curV)))
                {
                    V = curV;
                    break;
                }
                else if (Par < curP)
                {
                    B.MakeVertex(ref V, P.Value(), tol);
                    DS.InsertBefore(V, Par);
                    break;
                }
            }
            if (!DS.MoreVertex())
            {
                B.MakeVertex(ref V, P.Value(), tol);
                DS.Append(V, Par);
            }
        }
        DS.AddIntV(V);
    }

    /// <summary>Splits each edge with inserted vertices into new edges, recorded as its splits.</summary>
    private static void ProcessEdges(HLRTopoBRep_Data DS)
    {
        BRep_Builder B = new();
        TopoDS_Edge newE;
        TopoDS_Vertex VF, VL, VI;
        double PF, PL, PI;

        for (DS.InitEdge(); DS.MoreEdge(); DS.NextEdge())
        {
            TopoDS_Edge E = DS.Edge();
            NCollection_List<TopoDS_Shape> SplE = DS.AddSplE(E);
            VF = TopExp.FirstVertex(E);
            VL = TopExp.LastVertex(E);
            BRep_Tool.Range(E, out PF, out PL);
            VF.Orientation(TopAbs_FORWARD);
            VL.Orientation(TopAbs_REVERSED);

            for (DS.InitVertex(E); DS.MoreVertex(); DS.NextVertex())
            {
                VI = DS.Vertex();
                PI = DS.Parameter();
                VI.Orientation(TopAbs_REVERSED);
                newE = E;
                newE.EmptyCopy();
                newE.Orientation(TopAbs_FORWARD);
                B.Add(newE, VF);
                B.UpdateVertex(VF, PF, newE, BRep_Tool.Tolerance(VF));
                B.Add(newE, VI);
                B.UpdateVertex(VI, PI, newE, BRep_Tool.Tolerance(VI));
                newE.Orientation(E.Orientation());
                SplE.Append(newE);
                // prepare next edge
                VF = VI;
                PF = PI;
                VF.Orientation(TopAbs_FORWARD);
            }
            newE = E;
            newE.EmptyCopy();
            newE.Orientation(TopAbs_FORWARD);
            B.Add(newE, VF);
            B.UpdateVertex(VF, PF, newE, BRep_Tool.Tolerance(VF));
            B.Add(newE, VL);
            B.UpdateVertex(VL, PL, newE, BRep_Tool.Tolerance(VL));
            newE.Orientation(E.Orientation());
            SplE.Append(newE);
        }
    }
}
