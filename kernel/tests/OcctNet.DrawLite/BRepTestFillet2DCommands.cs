// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// BRepTest's 2D fillet commands (BRepTest_Fillet2DCommands.cxx): <c>chfi2d</c> (BRepFilletAPI_MakeFillet2d on a
/// face), <c>fillet2d</c> (ChFi2d_FilletAPI on a wire or two edges) and <c>chamfer2d</c> (ChFi2d_ChamferAPI), written
/// as OCCT writes them.
/// </summary>
/// <remarks>
/// <c>fillet2d</c> and <c>chamfer2d</c> read their numbers with C's <c>atof</c> (a prefix, 0 when there is none), as
/// OCCT does, where <c>chfi2d</c> reads expressions with Draw::Atof.
/// </remarks>
internal static class BRepTestFillet2DCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "chfi2d", Chfi2d);
        DrawCxx.Add(theInterpreter, "fillet2d", Fillet2d);
        DrawCxx.Add(theInterpreter, "chamfer2d", Chamfer2d);
    }

    // a[i][k]: the word's character, '\0' past its end (C's terminating null)
    private static char At(string theWord, int theIndex) => theIndex < theWord.Length ? theWord[theIndex] : '\0';

    // C's atof (OCCT's Atof): the number at the start of the word, 0 when there is none
    private static double CAtof(string theWord) => Standard_Stream.Strtod(theWord, out _);

    //=================================================================================================

    private static int Chfi2d(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            di.Append("chfi2d : not enough args");
            return 1;
        }

        // set up the algorithm
        TopoDS_Shape F = theDI.GetShape(a[2], TopAbs_ShapeEnum.TopAbs_FACE);
        if (F.IsNull())
        {
            di.Append("chfi2d : ").Append(a[2]).Append(" not a face");
            return 1;
        }

        var MF = new BRepFilletAPI_MakeFillet2d(TopoDS.Face(F));
        if (MF.Status() == ChFi2d_ConstructionError.ChFi2d_NotPlanar)
        {
            di.Append("chfi2d : not a planar face");
            return 1;
        }

        TopoDS_Shape res = new();
        bool partial_result = false;
        int i = 3;
        while (i + 1 < n)
        {
            TopoDS_Shape aLocalEdge = theDI.GetShape(a[i], TopAbs_ShapeEnum.TopAbs_EDGE);
            TopoDS_Edge E1 = TopoDS.Edge(aLocalEdge);
            aLocalEdge = theDI.GetShape(a[i + 1], TopAbs_ShapeEnum.TopAbs_EDGE);
            TopoDS_Edge E2 = TopoDS.Edge(aLocalEdge);

            if (E1.IsNull() || E2.IsNull())
            {
                di.Append("chfi2d : ").Append(a[i]).Append(" or ").Append(a[i + 1]).Append(" not an edge");
                if (partial_result)
                {
                    di.Append(" WARNING : this is a partial result ");
                    theDI.SetShape(a[1], res);
                }
                return 1;
            }

            TopoDS_Vertex V = new();
            if (!TopExp.CommonVertex(E1, E2, ref V))
            {
                di.Append("chfi2d ").Append(a[i]).Append(" and ").Append(a[i + 1]).Append(" does not share a vertex");
                if (partial_result)
                {
                    di.Append(" WARNING : this is a partial result ");
                    theDI.SetShape(a[1], res);
                }
                return 1;
            }

            i += 2;
            if (i + 1 >= n)
            {
                di.Append("chfi2d : not enough args");
                if (partial_result)
                {
                    di.Append(" WARNING : this is a partial result ");
                    theDI.SetShape(a[1], res);
                }
                return 1;
            }

            double p1 = theDI.Atof(a[i + 1]);
            if (At(a[i], 0) == 'F')
            {
                MF.AddFillet(V, p1);
            }
            else
            {
                if (i + 2 >= n)
                {
                    di.Append("chfi2d : not enough args");
                    if (partial_result)
                    {
                        di.Append(" WARNING : this is a partial result ");
                        theDI.SetShape(a[1], res);
                    }
                    return 1;
                }
                double p2 = theDI.Atof(a[i + 2]);
                if (At(a[i], 2) == 'D')
                {
                    MF.AddChamfer(E1, E2, p1, p2);
                }
                else
                {
                    MF.AddChamfer(E1, V, p1, p2 * (Math.PI / 180.0));
                }
            }

            if (MF.Status() == ChFi2d_ConstructionError.ChFi2d_TangencyError)
            {
                di.Append("chfi2d : ").Append(a[i - 2]).Append(" and ").Append(a[i - 1]).Append(" are tangent ");
                if (partial_result)
                {
                    di.Append(" WARNING : this is a partial result ");
                    theDI.SetShape(a[1], res);
                }
                return 1;
            }

            if (MF.Status() == ChFi2d_ConstructionError.ChFi2d_NotAuthorized)
            {
                di.Append("chfi2d : ").Append(a[i - 2]).Append(" or ").Append(a[i - 1]).Append(" is not a line or a circle ");
                if (partial_result)
                {
                    di.Append(" WARNING : this is a partial result ");
                    theDI.SetShape(a[1], res);
                }
                return 1;
            }

            if (MF.Status() != ChFi2d_ConstructionError.ChFi2d_IsDone)
            {
                di.Append("chfi2d : operation failed on ").Append(a[i - 2]);
                if (partial_result)
                {
                    di.Append(" WARNING : this is a partial result ");
                    theDI.SetShape(a[1], res);
                }
                return 1;
            }
            else
            {
                partial_result = true;
                MF.Build();
                res = MF.Shape();
            }

            if (At(a[i], 0) == 'F')
            {
                i += 2;
            }
            else
            {
                i += 3;
            }
        }

        MF.Build();
        theDI.SetShape(a[1], MF.Shape());

        return 0;
    }

    //=======================================================================
    // function : fillet2d
    // purpose  : A method to find a plane for 2 edges.
    //         : It may return a NULL object of the plane is not found
    //         : (the edge are located not in a plane).
    //=======================================================================

    private static Geom_Plane? findPlane(in TopoDS_Shape S)
    {
        Geom_Plane? plane = null;
        var planeFinder = new BRepBuilderAPI_FindPlane(S);
        if (planeFinder.Found())
        {
            plane = planeFinder.Plane();
        }
        return plane;
    }

    private static Geom_Plane? findPlane(in TopoDS_Shape E1, in TopoDS_Shape E2)
    {
        var B = new BRep_Builder();
        TopoDS_Compound C = new();
        B.MakeCompound(ref C);
        B.Add(C, E1);
        B.Add(C, E2);
        return findPlane(C);
    }

    //=======================================================================
    // function : findCommonPoint
    // purpose  : Find a common (or the most close) point of two edges.
    //=======================================================================

    private static gp_Pnt findCommonPoint(in TopoDS_Shape E1, in TopoDS_Shape E2)
    {
        TopExp.Vertices(TopoDS.Edge(E1), out TopoDS_Vertex v11, out TopoDS_Vertex v12);
        TopExp.Vertices(TopoDS.Edge(E2), out TopoDS_Vertex v21, out TopoDS_Vertex v22);

        gp_Pnt p11 = BRep_Tool.Pnt(v11);
        gp_Pnt p12 = BRep_Tool.Pnt(v12);
        gp_Pnt p21 = BRep_Tool.Pnt(v21);
        gp_Pnt p22 = BRep_Tool.Pnt(v22);

        gp_Pnt common = new();
        double d1121 = p11.SquareDistance(p21);
        double d1122 = p11.SquareDistance(p22);
        double d1221 = p12.SquareDistance(p21);
        double d1222 = p12.SquareDistance(p22);
        if (d1121 < d1122 && d1121 < d1221 && d1121 < d1222)
        {
            common = p11;
        }
        else if (d1122 < d1121 && d1122 < d1221 && d1122 < d1222)
        {
            common = p11;
        }
        else if (d1221 < d1121 && d1221 < d1122 && d1221 < d1222)
        {
            common = p12;
        }
        else if (d1222 < d1121 && d1222 < d1122 && d1222 < d1221)
        {
            common = p12;
        }

        return common;
    }

    private static gp_Pnt findCommonPoint(in TopoDS_Shape W)
    {
        // The common point for two edges inside a wire
        // is a sharing vertex of two edges.
        NCollection_Map<TopoDS_Shape> vertices = new(TopTools_ShapeMapHasher.Instance);
        var aExp = new TopExp_Explorer(W, TopAbs_ShapeEnum.TopAbs_VERTEX);
        for (; aExp.More(); aExp.Next())
        {
            if (!vertices.Add(aExp.Current()))
            {
                return BRep_Tool.Pnt(TopoDS.Vertex(aExp.Current()));
            }
        }
        return gp.Origin(); // not found
    }

    //=======================================================================
    // function : fillet2d
    // purpose  : Fillet 2d based on Newton method (recursive, iteration)
    // usage    : fillet2d result wire (or edge1 edge2) radius
    //=======================================================================

    private static int Fillet2d(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n != 4 && n != 5)
        {
            di.Append("Usage : fillet2d result wire (or edge1 edge2) radius");
            return 1;
        }

        TopoDS_Shape E1 = new(), E2 = new(), W = new();
        if (n == 5)
        {
            // Get the edges.
            E1 = theDI.GetShape(a[2], TopAbs_ShapeEnum.TopAbs_EDGE, true);
            E2 = theDI.GetShape(a[3], TopAbs_ShapeEnum.TopAbs_EDGE, true);
        }
        else
        {
            // Get the wire.
            W = theDI.GetShape(a[2], TopAbs_ShapeEnum.TopAbs_WIRE, true);
        }

        // Get the radius value.
        double radius = CAtof(n == 5 ? a[4] : a[3]);

        // Find plane of the edges.
        Geom_Plane? hPlane = n == 5 ? findPlane(E1, E2) : findPlane(W);
        if (hPlane is null)
        {
            di.Append("Error: the edges are located not in a plane.");
            return 1;
        }

        // Algo.
        var algo = new ChFi2d_FilletAPI();
        gp_Pln plane = hPlane.Pln();
        if (n == 5)
        {
            TopoDS_Edge e1 = TopoDS.Edge(E1);
            TopoDS_Edge e2 = TopoDS.Edge(E2);
            algo.Init(e1, e2, plane);
        }
        else
        {
            TopoDS_Wire w = TopoDS.Wire(W);
            algo.Init(w, plane);
        }
        bool status = algo.Perform(radius);
        if (!status)
        {
            di.Append("Error: the algorithm failed.");
            return 1;
        }

        // Find a common point of the edges.
        gp_Pnt common = n == 5 ? findCommonPoint(E1, E2) : findCommonPoint(W);

        // Get the number of solutions (usually it is equal to 1).
        int nbSolutions = algo.NbResults(common);
        if (nbSolutions == 0)
        {
            di.Append("Error: no solutions.");
            return 1;
        }

        // Get the result for the "nearest" solution (near the common point).
        TopoDS_Edge M1 = new(), M2 = new(); // modified E1 and E2
        TopoDS_Edge fillet = algo.Result(common, ref M1, ref M2);
        if (fillet.IsNull())
        {
            di.Append("Error: the algorithm produced no result.");
            return 1;
        }

        // Set result for DRAW.
        theDI.SetShape(a[1], fillet);

        // Update neighbour edges in DRAW.
        if (n == 5)
        {
            theDI.SetShape(a[2], M1);
            theDI.SetShape(a[3], M2);
        }
        else // recreate the wire using the fillet
        {
            var mkWire = new BRepBuilderAPI_MakeWire(M1, fillet, M2);
            if (mkWire.IsDone())
            {
                theDI.SetShape(a[1], mkWire.Wire());
            }
            else
            {
                theDI.SetShape(a[1], fillet);
            }
        }
        return 0;
    }

    //=======================================================================
    // function : chamfer2d
    // purpose  : Chamfer 2d.
    // usage    : chamfer2d result wire (or edge1 edge2) length1 length2
    //=======================================================================

    private static int Chamfer2d(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n != 5 && n != 6)
        {
            di.Append("Usage : chamfer2d result wire (or edge1 edge2) length1 length2");
            return 1;
        }

        TopoDS_Shape W = new();
        TopoDS_Shape E1 = new(), E2 = new();
        if (n == 6)
        {
            // Get the edges.
            E1 = theDI.GetShape(a[2], TopAbs_ShapeEnum.TopAbs_EDGE, true);
            E2 = theDI.GetShape(a[3], TopAbs_ShapeEnum.TopAbs_EDGE, true);
        }
        else
        {
            W = theDI.GetShape(a[2], TopAbs_ShapeEnum.TopAbs_WIRE, true);
        }

        // Get the lengths.
        double length1 = (n == 6) ? CAtof(a[4]) : CAtof(a[3]);
        double length2 = (n == 6) ? CAtof(a[5]) : CAtof(a[4]);

        // Algo.
        var algo = new ChFi2d_ChamferAPI();
        if (n == 6)
        {
            TopoDS_Edge e1 = TopoDS.Edge(E1);
            TopoDS_Edge e2 = TopoDS.Edge(E2);
            algo.Init(e1, e2);
        }
        else
        {
            TopoDS_Wire w = TopoDS.Wire(W);
            algo.Init(w);
        }

        // Prepare the chamfer.
        algo.Perform();

        // Get the result.
        TopoDS_Edge M1 = new(), M2 = new(); // modified E1 and E2
        TopoDS_Edge chamfer = algo.Result(ref M1, ref M2, length1, length2);
        if (chamfer.IsNull())
        {
            di.Append("Error: the algorithm produced no result.");
            return 1;
        }

        if (n == 6)
        {
            // Set result for DRAW.
            theDI.SetShape(a[1], chamfer);

            // Update neighbour edges in DRAW.
            theDI.SetShape(a[2], M1);
            theDI.SetShape(a[3], M2);
        }
        else // recreate the wire using the chamfer
        {
            var mkWire = new BRepBuilderAPI_MakeWire(M1, chamfer, M2);
            if (mkWire.IsDone())
            {
                theDI.SetShape(a[1], mkWire.Wire());
            }
            else
            {
                theDI.SetShape(a[1], chamfer);
            }
        }

        return 0;
    }
}
