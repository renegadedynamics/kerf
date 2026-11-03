// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TrimEdgeTool.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_TrimEdgeTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.JoinKind;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Geometric tool of the 2D offset (BRepFill_OffsetWire): the intersections of a bisector of the medial axis with the
/// parallels of its two generating elements.
/// </summary>
/// <remarks>
/// A working object (decision 21). It reads the bisector it is given and never changes it: OCCT keeps a copy of the
/// Bisector_Bisec sharing the same curve, the port the Bisector_Bisec itself (decision 822).
/// </remarks>
public sealed class BRepFill_TrimEdgeTool
{
    private readonly bool isPoint1;
    private readonly bool isPoint2;
    private readonly Point2D myP1;
    private readonly Point2D myP2;
    private readonly Geom2d_Curve? myC1;
    private readonly Geom2d_Curve? myC2;
    private readonly double myOffset;
    private readonly Bisector_Bisec? myBisec;
    private readonly Geom2dAdaptor_Curve myBis = new();

    // the simple expression of the bisector: a BisecAna is replaced by its conic, trimmed as the bisector
    private static void SimpleExpression(Bisector_Bisec B, out Geom2d_Curve Bis)
    {
        Bis = B.Value();

        Type BT = Bis.GetType();
        if (BT == typeof(Geom2d_TrimmedCurve))
        {
            Geom2d_TrimmedCurve TrBis = (Geom2d_TrimmedCurve)Bis;
            // OCCT.NET: ChangeBasisCurve, so the live bisector of the locus isn't frozen (decision 816)
            Geom2d_Curve BasBis = TrBis.ChangeBasisCurve();
            BT = BasBis.GetType();
            if (BT == typeof(Bisector_BisecAna))
            {
                Bis = ((Bisector_BisecAna)BasBis).Geom2dCurve();
                Bis = new Geom2d_TrimmedCurve(Bis, TrBis.FirstParameter(), TrBis.LastParameter());
            }
        }
    }

    /// <summary>An empty tool.</summary>
    public BRepFill_TrimEdgeTool()
    {
    }

    /// <summary>
    /// The tool of the bisector <paramref name="Bisec"/> of the elements <paramref name="S1"/> and <paramref name="S2"/>
    /// (2D points or curves) for the offset <paramref name="Offset"/>.
    /// </summary>
    public BRepFill_TrimEdgeTool(Bisector_Bisec Bisec, Geom2d_Geometry S1, Geom2d_Geometry S2, double Offset)
    {
        myOffset = Offset;
        myBisec = Bisec;
        isPoint1 = S1.GetType() == typeof(Geom2d_CartesianPoint);
        isPoint2 = S2.GetType() == typeof(Geom2d_CartesianPoint);

        // return geometries of shapes.
        if (isPoint1)
        {
            myP1 = ((Geom2d_Point)S1).Pnt2d();
        }
        else
        {
            myC1 = S1 as Geom2d_Curve;
        }
        if (isPoint2)
        {
            myP2 = ((Geom2d_Point)S2).Pnt2d();
        }
        else
        {
            myC2 = S2 as Geom2d_Curve;
        }
        // return the simple expression of the bissectrice
        SimpleExpression(myBisec, out Geom2d_Curve Bis);
        myBis = new Geom2dAdaptor_Curve(Bis);
    }

    // Order the sequence of points by increasing x.
    private static void Bubble(NCollection_Sequence<Point3D> Seq)
    {
        bool Invert = true;
        int NbPoints = Seq.Length();
        while (Invert)
        {
            Invert = false;
            for (int i = 1; i < NbPoints; i++)
            {
                Point3D P1 = Seq.Value(i);
                Point3D P2 = Seq.Value(i + 1);
                if (P2.X() < P1.X())
                {
                    Seq.Exchange(i, i + 1);
                    Invert = true;
                }
            }
        }
    }

    private static void EvalParameters(Geom2dAdaptor_Curve Bis, Geom2dAdaptor_Curve AC, NCollection_Sequence<Point3D> Params)
    {
        Geom2dInt_GInter Intersector;
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        Geom2dAdaptor_Curve CBis = Bis;
        Geom2dAdaptor_Curve CAC = AC;

        Intersector = new Geom2dInt_GInter(CAC, CBis, Tol, Tol);

        int NbPoints, NbSegments;
        double U1, U2;
        Point3D P;

        if (!Intersector.IsDone())
        {
            throw new NotDoneException("BRepFill_TrimSurfaceTool::IntersectWith");
        }

        NbPoints = Intersector.NbPoints();

        if (NbPoints > 0)
        {
            for (int i = 1; i <= NbPoints; i++)
            {
                U1 = Intersector.Point(i).ParamOnSecond();
                U2 = Intersector.Point(i).ParamOnFirst();
                P = new Point3D(U1, U2, 0.0);
                Params.Append(P);
            }
        }

        NbSegments = Intersector.NbSegments();

        if (NbSegments > 0)
        {
            IntRes2d_IntersectionSegment Seg;
            for (int i = 1; i <= NbSegments; i++)
            {
                Seg = Intersector.Segment(i);
                U1 = Seg.FirstPoint().ParamOnSecond();
                double Ulast = Seg.LastPoint().ParamOnSecond();
                if (Math.Abs(U1 - CBis.FirstParameter()) <= Tol
                    && Math.Abs(Ulast - CBis.LastParameter()) <= Tol)
                {
                    P = new Point3D(U1, Seg.FirstPoint().ParamOnFirst(), 0.0);
                    Params.Append(P);
                    P = new Point3D(Ulast, Seg.LastPoint().ParamOnFirst(), 0.0);
                    Params.Append(P);
                }
                else
                {
                    U1 += Seg.LastPoint().ParamOnSecond();
                    U1 /= 2.0;
                    U2 = Seg.FirstPoint().ParamOnFirst();
                    U2 += Seg.LastPoint().ParamOnFirst();
                    U2 /= 2.0;
                    P = new Point3D(U1, U2, 0.0);
                    Params.Append(P);
                }
            }
        }

        // Order the sequence by growing parameter on the bissectrice.
        Bubble(Params);
    }

    private static void EvalParametersBis(Geom2dAdaptor_Curve Bis, Geom2dAdaptor_Curve AC, NCollection_Sequence<Point3D> Params,
                                          double Tol)
    {
        Geom2dInt_GInter Intersector;
        double TolC = Tol;

        Geom2dAdaptor_Curve CBis = Bis;
        Geom2dAdaptor_Curve CAC = AC;

        Intersector = new Geom2dInt_GInter(CAC, CBis, TolC, Tol);

        int NbPoints, NbSegments;
        double U1, U2;
        Point3D P;

        if (!Intersector.IsDone())
        {
            throw new NotDoneException("BRepFill_TrimSurfaceTool::IntersectWith");
        }

        NbPoints = Intersector.NbPoints();

        if (NbPoints > 0)
        {
            for (int i = 1; i <= NbPoints; i++)
            {
                U1 = Intersector.Point(i).ParamOnSecond();
                U2 = Intersector.Point(i).ParamOnFirst();
                P = new Point3D(U1, U2, 0.0);
                Params.Append(P);
            }
        }

        NbSegments = Intersector.NbSegments();

        if (NbSegments > 0)
        {
            IntRes2d_IntersectionSegment Seg;
            for (int i = 1; i <= NbSegments; i++)
            {
                Seg = Intersector.Segment(i);
                U1 = Seg.FirstPoint().ParamOnSecond();
                double Ulast = Seg.LastPoint().ParamOnSecond();
                if (Math.Abs(U1 - CBis.FirstParameter()) <= Tol
                    && Math.Abs(Ulast - CBis.LastParameter()) <= Tol)
                {
                    P = new Point3D(U1, Seg.FirstPoint().ParamOnFirst(), 0.0);
                    Params.Append(P);
                    P = new Point3D(Ulast, Seg.LastPoint().ParamOnFirst(), 0.0);
                    Params.Append(P);
                }
                else
                {
                    U1 += Seg.LastPoint().ParamOnSecond();
                    U1 /= 2.0;
                    U2 = Seg.FirstPoint().ParamOnFirst();
                    U2 += Seg.LastPoint().ParamOnFirst();
                    U2 /= 2.0;
                    P = new Point3D(U1, U2, 0.0);
                    Params.Append(P);
                }
            }
        }

        // Order the sequence by parameter growing on the bissectrice.
        Bubble(Params);
    }

    /// <summary>
    /// The intersections of the bisector with the parallels <paramref name="Edge1"/> and <paramref name="Edge2"/>
    /// (generated by <paramref name="InitShape1"/> and <paramref name="InitShape2"/>): for each point, X is the
    /// parameter on the bisector, Y on the first parallel and Z on the second.
    /// </summary>
    public void IntersectWith(in TopoDS_Edge Edge1,
                              in TopoDS_Edge Edge2,
                              in TopoDS_Shape InitShape1,
                              in TopoDS_Shape InitShape2,
                              in TopoDS_Vertex End1,
                              in TopoDS_Vertex End2,
                              JoinKind theJoinType,
                              bool IsOpenResult,
                              NCollection_Sequence<Point3D> Params)
    {
        Params.Clear();

        // return curves associated to edges.
        double f, l;

        BRep_Tool.CurveOnSurface(Edge1, out Geom2d_Curve? C1, out Geom_Surface? _, out Placement _, out f, out l);
        Geom2dAdaptor_Curve AC1 = new(C1!, f, l);

        BRep_Tool.CurveOnSurface(Edge2, out Geom2d_Curve? C2, out Geom_Surface? _, out Placement _, out f, out l);
        Geom2dAdaptor_Curve AC2 = new(C2!, f, l);

        // Calculate intersection
        NCollection_Sequence<Point3D> Points2 = new();
        Point3D PSeq;

        EvalParameters(myBis, AC1, Params);
        EvalParameters(myBis, AC2, Points2);

        int SeanceDeRattrapage = 0;
        double TolInit = 1.0e-9;
        int nn = 7;

        if ((AC1.GetType() != Circle && AC1.GetType() != Line)
            || (AC2.GetType() != Circle && AC2.GetType() != Line))
        {
            TolInit = 1.0e-8;
            nn = 6;
        }

        if (Params.IsEmpty() && Points2.IsEmpty())
        {
            // Check, may be there are no intersections at all
            //  for case myBis == Line
            if (myBis.GetType() == Line)
            {
                double dmax = TolInit;
                int n = 0;
                while (n < nn)
                {
                    dmax *= 10.0;
                    ++n;
                }
                dmax *= dmax;
                //
                Line2D anL = myBis.Line();
                bool isFar1;
                bool isFar2;
                Point2D aP;
                //
                double d = RealLast();
                AC1.D0(AC1.FirstParameter(), out aP);
                double par = global::Kerf.Engine.Geometry.ElCLib.Parameter(anL, aP);
                if (par >= myBis.FirstParameter() && par <= myBis.LastParameter())
                {
                    d = anL.SquareDistance(aP);
                }
                AC1.D0(AC1.LastParameter(), out aP);
                par = global::Kerf.Engine.Geometry.ElCLib.Parameter(anL, aP);
                if (par >= myBis.FirstParameter() && par <= myBis.LastParameter())
                {
                    d = Min(anL.SquareDistance(aP), d);
                }
                isFar1 = d > dmax;
                //
                d = RealLast();
                AC2.D0(AC2.FirstParameter(), out aP);
                par = global::Kerf.Engine.Geometry.ElCLib.Parameter(anL, aP);
                if (par >= myBis.FirstParameter() && par <= myBis.LastParameter())
                {
                    d = anL.SquareDistance(aP);
                }
                AC2.D0(AC2.LastParameter(), out aP);
                par = global::Kerf.Engine.Geometry.ElCLib.Parameter(anL, aP);
                if (par >= myBis.FirstParameter() && par <= myBis.LastParameter())
                {
                    d = Min(anL.SquareDistance(aP), d);
                }
                isFar2 = d > dmax;
                //
                if (isFar1 && isFar2)
                {
                    return;
                }
            }
        }

        while (SeanceDeRattrapage < nn // TolInit <= 0.01
               && (Points2.Length() != Params.Length() || (Points2.Length() == 0 && Params.Length() == 0)))
        {
            Params.Clear();
            Points2.Clear();

            TolInit *= 10.0;

            EvalParametersBis(myBis, AC1, Params, TolInit);
            EvalParametersBis(myBis, AC2, Points2, TolInit);
            SeanceDeRattrapage++;
        }

        if (Params.Length() == 0 && Points2.Length() == 1)
        {
            double dmin, dmax = 0.25 * myOffset * myOffset;
            double tBis = Points2[1].X();
            Point2D PBis = myBis.Value(tBis);

            double t = AC1.FirstParameter();
            Point2D PC = AC1.Value(t);
            dmin = PC.SquareDistance(PBis);
            Point3D P = new(tBis, t, 0.0);
            if (dmin < dmax)
            {
                Params.Append(P);
            }

            t = AC1.LastParameter();
            PC = AC1.Value(t);
            double dmin1 = PC.SquareDistance(PBis);
            if (dmin > dmin1 && dmin1 < dmax)
            {
                P.SetY(t);
                if (Params.IsEmpty())
                {
                    Params.Append(P);
                }
                else
                {
                    Params.SetValue(1, P);
                }
            }
        }
        else if (Params.Length() == 1 && Points2.Length() == 0)
        {
            double dmin, dmax = 0.25 * myOffset * myOffset;
            double tBis = Params[1].X();
            Point2D PBis = myBis.Value(tBis);

            double t = AC2.FirstParameter();
            Point2D PC = AC2.Value(t);
            dmin = PC.SquareDistance(PBis);
            Point3D P = new(tBis, t, 0.0);
            if (dmin < dmax)
            {
                Points2.Append(P);
            }

            t = AC2.LastParameter();
            PC = AC2.Value(t);
            double dmin1 = PC.SquareDistance(PBis);
            if (dmin > dmin1 && dmin1 < dmax)
            {
                P.SetY(t);
                if (Points2.IsEmpty())
                {
                    Points2.Append(P);
                }
                else
                {
                    Points2.SetValue(1, P);
                }
            }
        }

        // small manipulation to remove incorrect intersections:
        // return only common intersections (same parameter
        // on the bissectrice.).
        // The tolerance can be eventually changed.

        Point3D P1, P2;
        double Tol = 4 * 100 * global::Kerf.Engine.Foundation.Precision.PConfusion();
        int i;
        int NbPoints = Params.Length();

        if (NbPoints == 1 && Points2.Length() == 1)
        {
            for (i = 1; i <= NbPoints; i++)
            {
                PSeq = Params[i];
                PSeq.SetZ(Points2.Value(i).Y());
                Params.SetValue(i, PSeq);
            }
            return;
        }

        i = 1;
        while (i <= Min(Params.Length(), Points2.Length()))
        {
            P1 = Params[i];
            P2 = Points2[i];
            double P1xP2x = Math.Abs(P1.X() - P2.X());

            if (P1xP2x > Tol)
            {
                if (P1xP2x > TolInit)
                {
                    i++;
                }
                else
                {
                    if (P1.X() < P2.X())
                    {
                        Params.Remove(i);
                    }
                    else
                    {
                        Points2.Remove(i);
                    }
                }
            }
            else
            {
                i++;
            }
        }

        if (Params.Length() > Points2.Length())
        {
            Params.Remove(Points2.Length() + 1, Params.Length());
        }
        else if (Params.Length() < Points2.Length())
        {
            Points2.Remove(Params.Length() + 1, Points2.Length());
        }

        NbPoints = Params.Length();

        // Now we define: if there are more than one point of intersection
        // is it Ok ?
        double init_fpar = RealFirst(), init_lpar = RealLast();
        if (NbPoints > 1 && theJoinType == Intersection && InitShape1.ShapeType() != TopAbs_VERTEX
            && InitShape2.ShapeType() != TopAbs_VERTEX)
        {
            // definition of initial first and last parameters:
            // this is inverse procedure to extension of parameters
            //(see BRepFill_OffsetWire, function MakeOffset, case of Circle)
            TopoDS_Edge InitEdge1 = TopoDS.Edge(InitShape1);
            bool ToExtendFirstPar = true;
            bool ToExtendLastPar = true;
            if (IsOpenResult)
            {
                TopExp.Vertices(InitEdge1, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                if (V1.IsSame(End1) || V1.IsSame(End2))
                {
                    ToExtendFirstPar = false;
                }
                if (V2.IsSame(End1) || V2.IsSame(End2))
                {
                    ToExtendLastPar = false;
                }
            }
            BRepAdaptor_Curve IC1 = new(InitEdge1);
            if (IC1.GetType() == Circle)
            {
                double Delta = 2 * Math.PI - IC1.LastParameter() + IC1.FirstParameter();
                if (ToExtendFirstPar && ToExtendLastPar)
                {
                    init_fpar = AC1.FirstParameter() + Delta / 2;
                }
                else if (ToExtendFirstPar)
                {
                    init_fpar = AC1.FirstParameter() + Delta;
                }
                else if (ToExtendLastPar)
                {
                    init_fpar = AC1.FirstParameter();
                }
                init_lpar = init_fpar + IC1.LastParameter() - IC1.FirstParameter();
            }
        }

        if (NbPoints > 1 && theJoinType == Intersection)
        {
            // Remove all vertices with non-minimal parameter
            // if they are out of initial range
            int imin = 1;
            for (i = 2; i <= NbPoints; i++)
            {
                if (Params[i].X() < Params[imin].X())
                {
                    imin = i;
                }
            }

            NCollection_Sequence<Point3D> ParamsCopy = new(Params);
            NCollection_Sequence<Point3D> Points2Copy = new(Points2);
            Params.Clear();
            Points2.Clear();
            for (i = 1; i <= ParamsCopy.Length(); i++)
            {
                if (imin == i || (ParamsCopy[i].Y() >= init_fpar && ParamsCopy[i].Y() <= init_lpar))
                {
                    Params.Append(ParamsCopy[i]);
                    Points2.Append(Points2Copy[i]);
                }
            }
        }

        NbPoints = Params.Length();
        for (i = 1; i <= NbPoints; i++)
        {
            PSeq = Params[i];
            PSeq.SetZ(Points2.Value(i).Y());
            Params.SetValue(i, PSeq);
        }
    }

    /// <summary>
    /// The first (<paramref name="Start"/>) or last point of the bisector is on the parallels: if it wasn't found among
    /// the intersections, it is projected on both parallels and added to <paramref name="Params"/>.
    /// </summary>
    public void AddOrConfuse(bool Start, in TopoDS_Edge Edge1, in TopoDS_Edge Edge2, NCollection_Sequence<Point3D> Params)
    {
        bool ToProj = true;
        Point2D PBis;
        double Tol = 10 * global::Kerf.Engine.Foundation.Precision.Confusion();

        // return curves associated to edges.
        BRep_Tool.CurveOnSurface(Edge1, out Geom2d_Curve? C1, out Geom_Surface? _, out Placement _, out double f, out double l);
        Geom2dAdaptor_Curve AC1 = new(C1!, f, l);

        if (Start)
        {
            PBis = myBis.Value(myBis.FirstParameter());
        }
        else
        {
            PBis = myBis.Value(myBis.LastParameter());
        }

        // Test if the end of the bissectrice is in the set of intersection points.
        if (!Params.IsEmpty())
        {
            Point2D P;
            if (Start)
            {
                P = AC1.Value(Params.First().Y());
            }
            else
            {
                P = AC1.Value(Params.Last().Y());
            }
            ToProj = !PBis.IsEqual(P, Tol);
        }

        if (ToProj)
        {
            // Project point on parallels and add in Params
            BRep_Tool.CurveOnSurface(Edge2, out Geom2d_Curve? C2, out Geom_Surface? _, out Placement _, out double f2, out double l2);

            Geom2dAPI_ProjectPointOnCurve Projector1 = new(PBis, C1!, f, l);
            Geom2dAPI_ProjectPointOnCurve Projector2 = new(PBis, C2!, f2, l2);

            if (Projector1.NbPoints() == 0)
            {
                return;
            }
            if (!Projector1.NearestPoint().IsEqual(PBis, Tol))
            {
                return;
            }
            if (Projector2.NbPoints() == 0)
            {
                return;
            }
            if (!Projector2.NearestPoint().IsEqual(PBis, Tol))
            {
                return;
            }
            Point3D PInt = new(0, Projector1.LowerDistanceParameter(), Projector2.LowerDistanceParameter());
            if (Start)
            {
                PInt.SetX(myBis.FirstParameter());
                Params.Prepend(PInt);
            }
            else
            {
                PInt.SetX(myBis.LastParameter());
                Params.Append(PInt);
            }
        }
    }

    /// <summary>True if <paramref name="P"/> is closer than the offset to the first generating element.</summary>
    public bool IsInside(in Point2D P)
    {
        double Dist = RealLast();
        if (isPoint1)
        {
            Dist = P.Distance(myP1);
        }
        else if (isPoint2)
        {
            Dist = P.Distance(myP2);
        }
        else
        {
            Geom2dAPI_ProjectPointOnCurve Projector = new(P, myC1!);
            if (Projector.NbPoints() > 0)
            {
                Dist = Projector.LowerDistance();
            }

            // Check of distances between P and first and last point of the first curve
            // should be performed in any case, despite of the results of projection.
            Point2D PF = myC1!.Value(myC1.FirstParameter());
            Point2D PL = myC1.Value(myC1.LastParameter());
            double aDistMin = Min(P.Distance(PF), P.Distance(PL));

            if (Dist > aDistMin)
            {
                Dist = aDistMin;
            }
        }

        return Dist < Math.Abs(myOffset) - global::Kerf.Engine.Foundation.Precision.Confusion();
    }
}
