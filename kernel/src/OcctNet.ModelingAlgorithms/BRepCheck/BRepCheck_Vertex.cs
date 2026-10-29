// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Vertex.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepCheck/BRepCheck_Vertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepCheck_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The checks of a vertex: in the context of an edge or a face, that the points its representations give are within
/// its tolerance of its point.
/// </summary>
public class BRepCheck_Vertex : BRepCheck_Result
{
    /// <summary>Checks the vertex on its own (<see cref="Minimum"/>).</summary>
    public BRepCheck_Vertex(in TopoDS_Vertex V)
    {
        Init(V);
    }

    /// <summary>Records <see cref="BRepCheck_NoError"/>: a vertex on its own has nothing to check.</summary>
    public override void Minimum()
    {
        if (!myMin)
        {
            // checks the existence of a point 3D
            var aNewList = new NCollection_List<BRepCheck_Status>();
            NCollection_List<BRepCheck_Status> lst = myMap.Bound(myShape, aNewList);
            lst.Append(BRepCheck_NoError);
            myMin = true;
        }
    }

    /// <summary>
    /// Checks the vertex in an edge (its point on each curve representation of the edge) or a face (its point on the
    /// surface), within the larger of the two tolerances.
    /// </summary>
    public override void InContext(in TopoDS_Shape S)
    {
        NCollection_List<BRepCheck_Status> aHList;
        {
            bool aLocked = myIsParallel;
            if (aLocked)
            {
                myMutex.Enter();
            }

            try
            {
                if (myMap.IsBound(S))
                {
                    return;
                }

                var aNewList = new NCollection_List<BRepCheck_Status>();
                aHList = myMap.Bound(S, aNewList);
            }
            finally
            {
                if (aLocked)
                {
                    myMutex.Exit();
                }
            }
        }

        NCollection_List<BRepCheck_Status> lst = aHList;

        var exp = new TopExp_Explorer(S, TopAbs_VERTEX);
        for (; exp.More(); exp.Next())
        {
            if (exp.Current().IsSame(myShape))
            {
                break;
            }
        }

        if (!exp.More())
        {
            BRepCheck.Add(lst, BRepCheck_SubshapeNotInShape);
            return; // leaves
        }

        var TV = (BRep_TVertex)myShape.TShape()!;
        gp_Pnt prep = TV.Pnt();
        gp_Pnt Controlp;

        TopAbs_ShapeEnum styp = S.ShapeType();
        switch (styp)
        {
            case TopAbs_EDGE:
            {
                // Try to find the vertex on the edge
                TopoDS_Edge E = TopoDS.Edge(S);
                var itv = new TopoDS_Iterator(E.Oriented(TopAbs_FORWARD));
                TopoDS_Vertex VFind = new();
                bool multiple = false;
                while (itv.More())
                {
                    TopoDS_Vertex VF = TopoDS.Vertex(itv.Value());
                    if (itv.Value().IsSame(myShape))
                    {
                        if (VFind.IsNull())
                        {
                            VFind = VF;
                        }
                        else
                        {
                            if ((VFind.Orientation() == TopAbs_FORWARD && VF.Orientation() == TopAbs_REVERSED)
                                || (VFind.Orientation() == TopAbs_REVERSED && VF.Orientation() == TopAbs_FORWARD))
                            {
                                // the vertex on the edge is at once F and R
                                multiple = true;
                            }

                            if (VFind.Orientation() != TopAbs_FORWARD && VFind.Orientation() != TopAbs_REVERSED)
                            {
                                if (VF.Orientation() == TopAbs_FORWARD || VF.Orientation() == TopAbs_REVERSED)
                                {
                                    VFind = VF;
                                }
                            }
                        }
                    }

                    itv.Next();
                }

                // VFind is not null for sure
                TopAbs_Orientation orv = VFind.Orientation();

                double Tol = BRep_Tool.Tolerance(TopoDS.Vertex(myShape));
                Tol = Max(Tol, BRep_Tool.Tolerance(E)); // to check
                Tol *= Tol;

                var TE = (BRep_TEdge)E.TShape()!;
                var itcr = new NCollection_List<BRep_CurveRepresentation>.Iterator(TE.Curves());
                TopLoc_Location Eloc = E.Location();

                var itpr = new NCollection_List<BRep_PointRepresentation>.Iterator();
                while (itcr.More())
                {
                    // For each CurveRepresentation, the provided parameter is checked
                    BRep_CurveRepresentation cr = itcr.Value();
                    TopLoc_Location loc = cr.Location();
                    TopLoc_Location L = (Eloc * loc).Predivided(myShape.Location());

                    if (cr.IsCurve3D())
                    {
                        Geom_Curve? C = cr.Curve3D();
                        if (C is not null) // edge non degenerated
                        {
                            itpr.Initialize(TV.Points());
                            while (itpr.More())
                            {
                                BRep_PointRepresentation pr = itpr.Value();
                                if (pr.IsPointOnCurve(C, L))
                                {
                                    Controlp = C.Value(pr.Parameter());
                                    Controlp.Transform(L.Transformation());
                                    if (prep.SquareDistance(Controlp) > Tol)
                                    {
                                        BRepCheck.Add(lst, BRepCheck_InvalidPointOnCurve);
                                    }
                                }

                                itpr.Next();
                            }

                            if (orv == TopAbs_FORWARD || orv == TopAbs_REVERSED)
                            {
                                var GC = (BRep_GCurve)cr;
                                if (orv == TopAbs_FORWARD || multiple)
                                {
                                    Controlp = C.Value(GC.First());
                                    Controlp.Transform(L.Transformation());
                                    if (prep.SquareDistance(Controlp) > Tol)
                                    {
                                        BRepCheck.Add(lst, BRepCheck_InvalidPointOnCurve);
                                    }
                                }

                                if (orv == TopAbs_REVERSED || multiple)
                                {
                                    Controlp = C.Value(GC.Last());
                                    Controlp.Transform(L.Transformation());
                                    if (prep.SquareDistance(Controlp) > Tol)
                                    {
                                        BRepCheck.Add(lst, BRepCheck_InvalidPointOnCurve);
                                    }
                                }
                            }
                        }
                    }
                    else if (cr.IsCurveOnSurface())
                    {
                        Geom_Surface Su = cr.Surface()!;
                        Geom2d_Curve PC = cr.PCurve()!;
                        Geom2d_Curve? PC2 = null;
                        if (cr.IsCurveOnClosedSurface())
                        {
                            PC2 = cr.PCurve2();
                        }

                        itpr.Initialize(TV.Points());
                        while (itpr.More())
                        {
                            BRep_PointRepresentation pr = itpr.Value();
                            if (pr.IsPointOnCurveOnSurface(PC, Su, L))
                            {
                                gp_Pnt2d p2d = PC.Value(pr.Parameter());
                                Controlp = Su.Value(p2d.X(), p2d.Y());
                                Controlp.Transform(L.Transformation());
                                if (prep.SquareDistance(Controlp) > Tol)
                                {
                                    BRepCheck.Add(lst, BRepCheck_InvalidPointOnCurveOnSurface);
                                }
                            }

                            if (PC2 is not null && pr.IsPointOnCurveOnSurface(PC2, Su, L))
                            {
                                gp_Pnt2d p2d = PC2.Value(pr.Parameter());
                                Controlp = Su.Value(p2d.X(), p2d.Y());
                                Controlp.Transform(L.Transformation());
                                if (prep.SquareDistance(Controlp) > Tol)
                                {
                                    BRepCheck.Add(lst, BRepCheck_InvalidPointOnCurveOnSurface);
                                }
                            }

                            itpr.Next();
                        }
                    }

                    itcr.Next();
                }

                if (lst.IsEmpty())
                {
                    lst.Append(BRepCheck_NoError);
                }

                break;
            }
            case TopAbs_FACE:
            {
                var TF = (BRep_TFace)S.TShape()!;
                TopLoc_Location Floc = S.Location();
                TopLoc_Location TFloc = TF.Location();
                Geom_Surface? Su = TF.Surface();
                TopLoc_Location L = (Floc * TFloc).Predivided(myShape.Location());

                double Tol = BRep_Tool.Tolerance(TopoDS.Vertex(myShape));
                Tol = Max(Tol, BRep_Tool.Tolerance(TopoDS.Face(S))); // to check
                Tol *= Tol;

                var itpr = new NCollection_List<BRep_PointRepresentation>.Iterator(TV.Points());
                while (itpr.More())
                {
                    BRep_PointRepresentation pr = itpr.Value();
                    if (pr.IsPointOnSurface(Su, L))
                    {
                        Controlp = Su!.Value(pr.Parameter(), pr.Parameter2());
                        Controlp.Transform(L.Transformation());
                        if (prep.SquareDistance(Controlp) > Tol)
                        {
                            BRepCheck.Add(lst, BRepCheck_InvalidPointOnSurface);
                        }
                    }

                    itpr.Next();
                }

                if (lst.IsEmpty())
                {
                    lst.Append(BRepCheck_NoError);
                }

                break;
            }
            default:
            {
                break;
            }
        }
    }

    /// <summary>Nothing more than the minimum (OCCT removed the body).</summary>
    public override void Blind()
    {
        if (myBlind)
        {
            return;
        }

        myBlind = true;
    }

    /// <summary>
    /// The tolerance the vertex needs: the largest distance from its point to the points its representations give
    /// (at least its tolerance), with a 5% margin.
    /// </summary>
    public double Tolerance()
    {
        // Check all the representations  of the vertex. (i-e checks the TVertex
        var TV = (BRep_TVertex)myShape.TShape()!;
        gp_Pnt prep = TV.Pnt();
        double Tol = BRep_Tool.Tolerance(TopoDS.Vertex(myShape));
        Tol *= Tol;

        gp_Pnt Controlp;
        Controlp = prep;
        var itpr = new NCollection_List<BRep_PointRepresentation>.Iterator(TV.Points());
        while (itpr.More())
        {
            BRep_PointRepresentation pr = itpr.Value();
            TopLoc_Location loc = pr.Location();
            if (pr.IsPointOnCurve())
            {
                // OCCT: with a null curve, the previous point is kept and transformed again below
                if (pr.Curve() is not null)
                {
                    Controlp = pr.Curve()!.Value(pr.Parameter());
                }
            }
            else if (pr.IsPointOnCurveOnSurface())
            {
                gp_Pnt2d Puv = pr.PCurve()!.Value(pr.Parameter());
                Controlp = pr.Surface()!.Value(Puv.X(), Puv.Y());
            }
            else if (pr.IsPointOnSurface())
            {
                Controlp = pr.Surface()!.Value(pr.Parameter(), pr.Parameter2());
            }

            Controlp.Transform(loc.Transformation());
            if (prep.SquareDistance(Controlp) > Tol)
            {
                Tol = prep.SquareDistance(Controlp);
            }

            itpr.Next();
        }

        return Math.Sqrt(Tol * 1.05);
    }
}
