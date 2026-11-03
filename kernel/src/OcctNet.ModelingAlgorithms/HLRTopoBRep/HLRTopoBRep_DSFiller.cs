// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_DSFiller.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRTopoBRep/HLRTopoBRep_DSFiller.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.Contap_IType;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    public static void Insert(in Shape S, Contap_Contour FO, HLRTopoBRep_Data DS,
                              NCollection_DataMap<Shape, BRepTopAdaptor_Tool> MST, int nbIso)
    {
        NCollection_Map<Shape> ShapeMap = new(ShapeHasher.Instance);
        Explorer ex = new(S, ShapeType.Face);
        DS.Clear();
        bool withPCurve = true; // instead of nbIso != 0;
        int f = 0;

        while (ex.More())
        {
            if (ShapeMap.Add(ex.Current()))
            {
                f++;
                Face S1 = ShapeCasts.Face(ex.Current());
                S1.Orientation(Forward);
                BRepTopAdaptor_TopolTool Domain;
                SurfaceAdaptorBase? Surface;
                if (MST.IsBound(S1))
                {
                    BRepTopAdaptor_Tool BRT = MST.ChangeFind(S1);
                    Domain = BRT.GetTopolTool();
                    Surface = BRT.GetSurface();
                }
                else
                {
                    BRepTopAdaptor_Tool BRT = new(S1, global::Kerf.Engine.Foundation.Precision.PConfusion());
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
    private static void InsertFace(int FI, in Face F, Contap_Contour FO, HLRTopoBRep_Data DS, bool withPCurve)
    {
        double tol = Tool.Tolerance(F);
        NCollection_List<Shape> IntL = DS.AddIntL(F);
        NCollection_List<Shape> OutL = DS.AddOutL(F);

        Vertex VF = new(), VL = new();
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
                Edge E = ((EdgeCurve2DAdaptor)Line.Arc()).Edge();
                OutL.Append(E);
                Exploration.Vertices(E, out VF, out VL);
                // insert the Internal points.

                for (CurPoint = 1; CurPoint <= NbPoints; CurPoint++)
                {
                    Contap_Point P = Line.Vertex(CurPoint);
                    if (P.IsInternal())
                    {
                        if (P.Value().IsEqual(Tool.Pnt(VF), Tool.Tolerance(VF)))
                        {
                            if (P.Value().IsEqual(Tool.Pnt(VL), Tool.Tolerance(VL)))
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

                        if ((parL - parF) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {

                            Curve3D? C = null;
                            Curve2D? C2d = null;
                            double first = parF;
                            double last = parL;
                            bool InsuffisantNumberOfPoints = false;

                            switch (Line.TypeContour())
                            {

                                case Contap_Lin:
                                {
                                    C = new LineCurve3D(Line.Line());
                                    if (withPCurve)
                                    {
                                        Surface S = Tool.Surface(F)!;
                                        double Tol = 1e-7;
                                        C2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(C, first, last, S, ref Tol);
                                    }
                                }
                                break;

                                case Contap_Circle:
                                {
                                    C = new CircleCurve3D(Line.Circle());
                                    if (withPCurve)
                                    {
                                        Surface S = Tool.Surface(F, out Placement Loc)!;
                                        if (!Loc.IsIdentity)
                                        {
                                            S = (Surface)S.Transformed(Loc.Transformation());
                                        }
                                        double Tol = 1e-7;
                                        C2d = global::Kerf.Engine.Geometry.Projection.GeomProjLib.Curve2d(C, first, last, S, ref Tol);
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
                                        NCollection_Array1<Point3D> Points = new(1, nbp);

                                        for (int i = 1; i <= nbp; i++)
                                        {
                                            knots.SetValue(i, (double)i);
                                            mults.SetValue(i, 1);
                                            Points.SetValue(i, Line.Point(i + ipF - 1).Value());
                                        }
                                        mults[1] = mults[nbp] = 2;
                                        C = new BSplineCurve3D(Points, knots, mults, 1);

                                        if (withPCurve)
                                        {
                                            NCollection_Array1<Point2D> Points2d = new(1, nbp);
                                            for (int i = 1; i <= nbp; i++)
                                            {
                                                double u, v;
                                                Line.Point(i + ipF - 1).ParametersOnS2(out u, out v);
                                                Points2d.SetValue(i, new Point2D(u, v));
                                            }
                                            C2d = new BSplineCurve2D(Points2d, knots, mults, 1);
                                        }
                                        first = 1;
                                        last = nbp;
                                    }
                                    else
                                    {
                                        int nbp = ipL - ipF + 1;
                                        NCollection_Array1<double> knots = new(1, nbp);
                                        NCollection_Array1<int> mults = new(1, nbp);
                                        NCollection_Array1<Point3D> Points = new(1, nbp);

                                        double Maxx, Maxy, Maxz, Maxu, Maxv;
                                        double Minx, Miny, Minz, Minu, Minv;
                                        Maxx = Maxy = Maxz = Maxu = Maxv = -RealLast();
                                        Minx = Miny = Minz = Minu = Minv = RealLast();

                                        for (int i = 1; i <= nbp; i++)
                                        {
                                            knots.SetValue(i, (double)i);
                                            mults.SetValue(i, 1);
                                            Point3D P = Line.Point(i + ipF - 1).Value();
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
                                        BSplineCurve3D AppC;
                                        BSplineCurve2D? AppC2d = null;
                                        AppC = new BSplineCurve3D(Points, knots, mults, 1);

                                        if (withPCurve)
                                        {
                                            NCollection_Array1<Point2D> Points2d = new(1, nbp);
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
                                                Points2d.SetValue(i, new Point2D(u, v));
                                            }
                                            AppC2d = new BSplineCurve2D(Points2d, knots, mults, 1);
                                        }
                                        first = 1;
                                        last = nbp;

                                        BRepApprox_ApproxLine AppLine;
                                        BSplineCurve2D? CNull = null;
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
                                            MultiBSpCurve AppVal = Approx.Value(1);
                                            NCollection_Array1<Point3D> poles3d = new(1, AppVal.NbPoles());
                                            AppVal.Curve(1, poles3d);
                                            C = new BSplineCurve3D(poles3d, AppVal.Knots(), AppVal.Multiplicities(),
                                                                      AppVal.Degree());

                                            MultiBSpCurve AppVal2 = Approx.Value(2);
                                            NCollection_Array1<Point2D> poles2d = new(1, AppVal2.NbPoles());
                                            AppVal2.Curve(2, poles2d);
                                            C2d = new BSplineCurve2D(poles2d, AppVal2.Knots(),
                                                                          AppVal2.Multiplicities(), AppVal2.Degree());
                                            first = C2d.FirstParameter();
                                            last = C2d.LastParameter();
                                        }
                                    }
                                }
                                break;

                                case Contap_Restriction:
                                {
                                    throw new ProgramErrorException("HLRTopoBRep_DSFiller::InsertFace : Restriction");
                                }
                            }

                            // compute the PCurve
                            // make the edge
                            if (!InsuffisantNumberOfPoints)
                            {
                                Edge E = new();
                                BrepBuilder B = new();
                                B.MakeEdge(ref E, C, tol);
                                VF.Orientation(Forward);
                                VL.Orientation(Reversed);
                                B.Add(E, VF);
                                B.Add(E, VL);
                                B.Range(E, first, last);

                                if (C2d is not null)
                                {
                                    B.UpdateEdge(E, C2d, F, Tool.Tolerance(F));
                                }

                                // add the edge in the DS
                                if (!E.IsNull)
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

        NCollection_List<Shape>.Iterator itl1 = new(IntL);
        for (; itl1.More(); itl1.Next())
        {
            Edge anIntLine = ShapeCasts.Edge(itl1.Value());
            anIntLine.Orientation(Forward);
            Vertex[] aVer = new Vertex[2];
            Exploration.Vertices(anIntLine, out aVer[0], out aVer[1]);
            NCollection_List<Shape>.Iterator itl2 = new(itl1);
            for (; itl2.More(); itl2.Next())
            {
                Edge anIntLine2 = ShapeCasts.Edge(itl2.Value());
                anIntLine2.Orientation(Forward);
                if (anIntLine2.IsSame(anIntLine))
                {
                    continue;
                }
                Vertex[] aVer2 = new Vertex[2];
                Exploration.Vertices(anIntLine2, out aVer2[0], out aVer2[1]);
                for (int i = 0; i < 2; i++)
                {
                    if (i == 1 && aVer[0].IsSame(aVer[1]))
                    {
                        continue;
                    }
                    Point3D Pnt1 = Tool.Pnt(aVer[i]);
                    for (int j = 0; j < 2; j++)
                    {
                        if (aVer[i].IsSame(aVer2[j]))
                        {
                            continue;
                        }
                        Point3D Pnt2 = Tool.Pnt(aVer2[j]);
                        if (Pnt1.SquareDistance(Pnt2) <= SqTol)
                        {
                            BrepBuilder aBB = new();
                            aBB.Remove(anIntLine2, aVer2[j]);
                            aVer[i].Orientation((j == 0) ? Forward : Reversed);
                            aBB.Add(anIntLine2, aVer[i]);
                        }
                    }
                }
            }
        }
    }

    /// <summary>The vertex of a contour point: the face's vertex, a vertex inserted on the arc it lies on, or a new vertex.</summary>
    private static Vertex MakeVertex(in Contap_Point P, double tol, HLRTopoBRep_Data DS)
    {
        BrepBuilder B = new();
        Vertex V = new();
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
                Edge E = ((EdgeCurve2DAdaptor)P.Arc()).Edge();
                double Par = P.ParameterOnArc();
                Point3D P3d = P.Value();

                for (DS.InitVertex(E); DS.MoreVertex(); DS.NextVertex())
                {
                    Vertex curV = DS.Vertex();
                    double curP = DS.Parameter();
                    Point3D PPP = Tool.Pnt(curV);
                    double TTT = Tool.Tolerance(curV);
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
    private static void InsertVertex(in Contap_Point P, double tol, in Edge E, HLRTopoBRep_Data DS)
    {
        BrepBuilder B = new();
        Vertex V = new();

        if (P.IsVertex())
        {
            V = ((BRepTopAdaptor_HVertex)P.Vertex()).Vertex();
        }
        else
        {
            double Par = P.ParameterOnLine();

            for (DS.InitVertex(E); DS.MoreVertex(); DS.NextVertex())
            {
                Vertex curV = DS.Vertex();
                double curP = DS.Parameter();
                if (P.Value().IsEqual(Tool.Pnt(curV), Tool.Tolerance(curV)))
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
        BrepBuilder B = new();
        Edge newE;
        Vertex VF, VL, VI;
        double PF, PL, PI;

        for (DS.InitEdge(); DS.MoreEdge(); DS.NextEdge())
        {
            Edge E = DS.Edge();
            NCollection_List<Shape> SplE = DS.AddSplE(E);
            VF = Exploration.FirstVertex(E);
            VL = Exploration.LastVertex(E);
            Tool.Range(E, out PF, out PL);
            VF.Orientation(Forward);
            VL.Orientation(Reversed);

            for (DS.InitVertex(E); DS.MoreVertex(); DS.NextVertex())
            {
                VI = DS.Vertex();
                PI = DS.Parameter();
                VI.Orientation(Reversed);
                newE = E;
                newE.EmptyCopy();
                newE.Orientation(Forward);
                B.Add(newE, VF);
                B.UpdateVertex(VF, PF, newE, Tool.Tolerance(VF));
                B.Add(newE, VI);
                B.UpdateVertex(VI, PI, newE, Tool.Tolerance(VI));
                newE.Orientation(E.Orientation());
                SplE.Append(newE);
                // prepare next edge
                VF = VI;
                PF = PI;
                VF.Orientation(Forward);
            }
            newE = E;
            newE.EmptyCopy();
            newE.Orientation(Forward);
            B.Add(newE, VF);
            B.UpdateVertex(VF, PF, newE, Tool.Tolerance(VF));
            B.Add(newE, VL);
            B.UpdateVertex(VL, PL, newE, Tool.Tolerance(VL));
            newE.Orientation(E.Orientation());
            SplE.Append(newE);
        }
    }
}
