// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Fillets.Planar;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Kernel.DrawLite;

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
    private static double CAtof(string theWord) => StreamUtilities.Strtod(theWord, out _);

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
        Shape F = theDI.GetShape(a[2], ShapeType.Face);
        if (F.IsNull)
        {
            di.Append("chfi2d : ").Append(a[2]).Append(" not a face");
            return 1;
        }

        var MF = new MakeFillet2D(ShapeCasts.Face(F));
        if (MF.Status() == ConstructionError.NotPlanar)
        {
            di.Append("chfi2d : not a planar face");
            return 1;
        }

        Shape res = new();
        bool partial_result = false;
        int i = 3;
        while (i + 1 < n)
        {
            Shape aLocalEdge = theDI.GetShape(a[i], ShapeType.Edge);
            Edge E1 = ShapeCasts.Edge(aLocalEdge);
            aLocalEdge = theDI.GetShape(a[i + 1], ShapeType.Edge);
            Edge E2 = ShapeCasts.Edge(aLocalEdge);

            if (E1.IsNull || E2.IsNull)
            {
                di.Append("chfi2d : ").Append(a[i]).Append(" or ").Append(a[i + 1]).Append(" not an edge");
                if (partial_result)
                {
                    di.Append(" WARNING : this is a partial result ");
                    theDI.SetShape(a[1], res);
                }
                return 1;
            }

            Vertex V = new();
            if (!Exploration.CommonVertex(E1, E2, ref V))
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

            if (MF.Status() == ConstructionError.TangencyError)
            {
                di.Append("chfi2d : ").Append(a[i - 2]).Append(" and ").Append(a[i - 1]).Append(" are tangent ");
                if (partial_result)
                {
                    di.Append(" WARNING : this is a partial result ");
                    theDI.SetShape(a[1], res);
                }
                return 1;
            }

            if (MF.Status() == ConstructionError.NotAuthorized)
            {
                di.Append("chfi2d : ").Append(a[i - 2]).Append(" or ").Append(a[i - 1]).Append(" is not a line or a circle ");
                if (partial_result)
                {
                    di.Append(" WARNING : this is a partial result ");
                    theDI.SetShape(a[1], res);
                }
                return 1;
            }

            if (MF.Status() != ConstructionError.IsDone)
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

    private static PlaneSurface? findPlane(in Shape S)
    {
        PlaneSurface? plane = null;
        var planeFinder = new FindPlane(S);
        if (planeFinder.Found())
        {
            plane = planeFinder.Plane();
        }
        return plane;
    }

    private static PlaneSurface? findPlane(in Shape E1, in Shape E2)
    {
        var B = new BrepBuilder();
        Compound C = new();
        B.MakeCompound(ref C);
        B.Add(C, E1);
        B.Add(C, E2);
        return findPlane(C);
    }

    //=======================================================================
    // function : findCommonPoint
    // purpose  : Find a common (or the most close) point of two edges.
    //=======================================================================

    private static Point3D findCommonPoint(in Shape E1, in Shape E2)
    {
        Exploration.Vertices(ShapeCasts.Edge(E1), out Vertex v11, out Vertex v12);
        Exploration.Vertices(ShapeCasts.Edge(E2), out Vertex v21, out Vertex v22);

        Point3D p11 = Tool.Pnt(v11);
        Point3D p12 = Tool.Pnt(v12);
        Point3D p21 = Tool.Pnt(v21);
        Point3D p22 = Tool.Pnt(v22);

        Point3D common = new();
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

    private static Point3D findCommonPoint(in Shape W)
    {
        // The common point for two edges inside a wire
        // is a sharing vertex of two edges.
        HashSet<Shape> vertices = new(ShapeHasher.Instance);
        var aExp = new Explorer(W, ShapeType.Vertex);
        for (; aExp.More(); aExp.Next())
        {
            if (!vertices.Add(aExp.Current()))
            {
                return Tool.Pnt(ShapeCasts.Vertex(aExp.Current()));
            }
        }
        return Constants.Origin(); // not found
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

        Shape E1 = new(), E2 = new(), W = new();
        if (n == 5)
        {
            // Get the edges.
            E1 = theDI.GetShape(a[2], ShapeType.Edge, true);
            E2 = theDI.GetShape(a[3], ShapeType.Edge, true);
        }
        else
        {
            // Get the wire.
            W = theDI.GetShape(a[2], ShapeType.Wire, true);
        }

        // Get the radius value.
        double radius = CAtof(n == 5 ? a[4] : a[3]);

        // Find plane of the edges.
        PlaneSurface? hPlane = n == 5 ? findPlane(E1, E2) : findPlane(W);
        if (hPlane is null)
        {
            di.Append("Error: the edges are located not in a plane.");
            return 1;
        }

        // Algo.
        var algo = new FilletApi();
        Plane3D plane = hPlane.Pln();
        if (n == 5)
        {
            Edge e1 = ShapeCasts.Edge(E1);
            Edge e2 = ShapeCasts.Edge(E2);
            algo.Init(e1, e2, plane);
        }
        else
        {
            Wire w = ShapeCasts.Wire(W);
            algo.Init(w, plane);
        }
        bool status = algo.Perform(radius);
        if (!status)
        {
            di.Append("Error: the algorithm failed.");
            return 1;
        }

        // Find a common point of the edges.
        Point3D common = n == 5 ? findCommonPoint(E1, E2) : findCommonPoint(W);

        // Get the number of solutions (usually it is equal to 1).
        int nbSolutions = algo.NbResults(common);
        if (nbSolutions == 0)
        {
            di.Append("Error: no solutions.");
            return 1;
        }

        // Get the result for the "nearest" solution (near the common point).
        Edge M1 = new(), M2 = new(); // modified E1 and E2
        Edge fillet = algo.Result(common, ref M1, ref M2);
        if (fillet.IsNull)
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
            var mkWire = new MakeWire(M1, fillet, M2);
            if (mkWire.IsDone)
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

        Shape W = new();
        Shape E1 = new(), E2 = new();
        if (n == 6)
        {
            // Get the edges.
            E1 = theDI.GetShape(a[2], ShapeType.Edge, true);
            E2 = theDI.GetShape(a[3], ShapeType.Edge, true);
        }
        else
        {
            W = theDI.GetShape(a[2], ShapeType.Wire, true);
        }

        // Get the lengths.
        double length1 = (n == 6) ? CAtof(a[4]) : CAtof(a[3]);
        double length2 = (n == 6) ? CAtof(a[5]) : CAtof(a[4]);

        // Algo.
        var algo = new ChamferApi();
        if (n == 6)
        {
            Edge e1 = ShapeCasts.Edge(E1);
            Edge e2 = ShapeCasts.Edge(E2);
            algo.Init(e1, e2);
        }
        else
        {
            Wire w = ShapeCasts.Wire(W);
            algo.Init(w);
        }

        // Prepare the chamfer.
        algo.Perform();

        // Get the result.
        Edge M1 = new(), M2 = new(); // modified E1 and E2
        Edge chamfer = algo.Result(ref M1, ref M2, length1, length2);
        if (chamfer.IsNull)
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
            var mkWire = new MakeWire(M1, chamfer, M2);
            if (mkWire.IsDone)
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
