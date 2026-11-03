// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// The commands of BRepTest_CurveCommands.cxx that need only BRepBuilderAPI's makers and BRepLib: vertex, edge,
/// mkedge, wire, polyline, polyvertex, mkcurve, mkoricurve, mkpoint, mk2dcurve, uisoedge, visoedge and profile; and
/// mkoffset and openoffset (BRepOffsetAPI_MakeOffset, unit 5O2); concatC0wire (BRepAlgo::ConcatenateWireC0, unit 7W).
/// </summary>
/// <remarks>
/// The makers' results are read through their internal accessors where OCCT writes into them before binding
/// (isoedge's pcurve, profile's 3D curves on a non-planar face); <see cref="DrawInterpreter.SetShape"/> freezes what
/// is bound. Not here: range, etrim and transfert (they change a bound edge in place), bsplineprof and 2dprofile
/// (Geom2dAPI_Interpolate and friends), edgeintersector (TopOpeBRep),
/// arclinconvert, concatwire, build3d and reducepcurves.
/// </remarks>
internal static class BRepTestCurveCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "vertex", Vertex);
        DrawCxx.Add(theInterpreter, "polyline", Polyline);
        DrawCxx.Add(theInterpreter, "polyvertex", Polyvertex);
        DrawCxx.Add(theInterpreter, "wire", Wire);
        DrawCxx.Add(theInterpreter, "mkedge", MkEdge);
        DrawCxx.Add(theInterpreter, "mkcurve", MkCurve);
        DrawCxx.Add(theInterpreter, "mkoricurve", MkCurve);
        DrawCxx.Add(theInterpreter, "mkpoint", MkPoint);
        DrawCxx.Add(theInterpreter, "mk2dcurve", Mk2dCurve);
        DrawCxx.Add(theInterpreter, "edge", Edge);
        DrawCxx.Add(theInterpreter, "uisoedge", IsoEdge);
        DrawCxx.Add(theInterpreter, "visoedge", IsoEdge);
        DrawCxx.Add(theInterpreter, "profile", Profile);
        DrawCxx.Add(theInterpreter, "mkoffset", MkOffset);
        DrawCxx.Add(theInterpreter, "openoffset", OpenOffset);
        DrawCxx.Add(theInterpreter, "concatC0wire", ConcatC0Wire);
    }

    // concatC0wire result wire
    private static int ConcatC0Wire(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> c)
    {
        int n = c.Count;
        if (n < 3)
        {
            return 1;
        }

        Shape S = theDI.GetShape(c[2], ShapeType.Wire);

        if (S.IsNull)
        {
            return 1; // test if the shape is empty
        }

        Wire W = ShapeCasts.Wire(S);

        Shape res = BRepAlgo.ConcatenateWireC0(W); // treatment
        theDI.SetShape(c[1], res);
        return 0;
    }

    private static int Vertex(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        if (n >= 5)
        {
            theDI.SetShape(a[1], new BRepBuilderAPI_MakeVertex(new Point3D(theDI.Atof(a[2]), theDI.Atof(a[3]), theDI.Atof(a[4]))).Shape());
        }
        else if (n == 4)
        {
            Shape S = theDI.GetShape(a[3]);
            if (S.IsNull)
            {
                return 0;
            }

            if (S.ShapeType() != ShapeType.Edge)
            {
                return 0;
            }

            var C = new EdgeCurveAdaptor(ShapeCasts.Edge(S));
            C.D0(theDI.Atof(a[2]), out Point3D P);
            theDI.SetShape(a[1], new BRepBuilderAPI_MakeVertex(P).Shape());
        }
        else
        {
            // OCCT dereferences the down-cast handle without a test: a name that isn't a point is an access violation
            var aP = new Point3D();
            if (!DrawTrSurf.GetPoint(theDI, a[2], ref aP))
            {
                throw new global::Kerf.Engine.Foundation.NullObjectException();
            }

            theDI.SetShape(a[1], new BRepBuilderAPI_MakeVertex(aP).Shape());
        }

        return 0;
    }

    private static int Polyline(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 8)
        {
            return 1;
        }

        if ((n - 2) % 3 != 0)
        {
            return 1;
        }

        int np = (n - 2) / 3;
        var W = new BRepBuilderAPI_MakePolygon();
        int j = 2;
        for (int i = 1; i <= np; i++)
        {
            W.Add(new Point3D(theDI.Atof(a[j]), theDI.Atof(a[j + 1]), theDI.Atof(a[j + 2])));
            j += 3;
        }

        theDI.SetShape(a[1], W.Wire());
        return 0;
    }

    private static int Polyvertex(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        var W = new BRepBuilderAPI_MakePolygon();
        for (int i = 2; i < n; i++)
        {
            Shape S = theDI.GetShape(a[i]);
            if (S.IsNull)
            {
                return 1;
            }

            if (S.ShapeType() != ShapeType.Vertex)
            {
                return 1;
            }

            W.Add(ShapeCasts.Vertex(S));
        }

        theDI.SetShape(a[1], W.Wire());
        return 0;
    }

    private static int Wire(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        var MW = new BRepBuilderAPI_MakeWire();
        bool IsUnsorted = a[2] == "-unsorted";
        if (!IsUnsorted)
        {
            for (int i = 2; i < n; i++)
            {
                Shape S = theDI.GetShape(a[i]);
                if (S.IsNull)
                {
                    continue;
                }

                if (S.ShapeType() == ShapeType.Edge)
                {
                    MW.Add(ShapeCasts.Edge(S));
                }
                else if (S.ShapeType() == ShapeType.Wire)
                {
                    MW.Add(ShapeCasts.Wire(S));
                }
            }
        }
        else
        {
            var aLE = new NCollection_List<Shape>();
            for (int i = 3; i < n; i++)
            {
                Shape S = theDI.GetShape(a[i]);
                for (var Exp = new Explorer(S, ShapeType.Edge); Exp.More(); Exp.Next())
                {
                    Edge anE = ShapeCasts.Edge(Exp.Current());
                    if (!anE.IsNull)
                    {
                        aLE.Append(anE);
                    }
                }
            }

            MW.Add(aLE);
        }

        if (!MW.IsDone())
        {
            di.Append("Wire not done with an error:\n");
            switch (MW.Error())
            {
                case BRepBuilderAPI_WireError.BRepBuilderAPI_EmptyWire:
                    di.Append("BRepBuilderAPI_EmptyWire\n");
                    break;
                case BRepBuilderAPI_WireError.BRepBuilderAPI_DisconnectedWire:
                    di.Append("BRepBuilderAPI_DisconnectedWire\n");
                    break;
                case BRepBuilderAPI_WireError.BRepBuilderAPI_NonManifoldWire:
                    di.Append("BRepBuilderAPI_NonManifoldWire\n");
                    break;
            }
        }
        else
        {
            theDI.SetShape(a[1], MW.Shape());
        }

        return 0;
    }

    private static int MkEdge(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        Curve3D? C = DrawTrSurf.GetCurve(theDI, a[2]);
        Curve2D? C2d = DrawTrSurf.GetCurve2d(theDI, a[2]);
        var P3d = (theDI.GetDrawable(a[2]) as DrawGeometry)?.Geometry as Polygon3D;
        if (C is null && C2d is null && P3d is null)
        {
            di.Append(a[2]).Append(" is not a curve or polygon 3d\n");
            return 1;
        }

        Edge edge = new();
        if (n == 3)
        {
            if (C is not null)
            {
                edge = new BRepBuilderAPI_MakeEdge(C).Edge();
            }
            else if (C2d is not null)
            {
                edge = new BRepBuilderAPI_MakeEdge2d(C2d).Edge();
            }
            else
            {
                new BrepBuilder().MakeEdge(ref edge, P3d);
            }
        }
        else
        {
            Surface? S = null;
            int i = 0;
            if (C2d is not null)
            {
                S = DrawTrSurf.GetSurface(theDI, a[3]);
                if (S is not null)
                {
                    i = 1;
                }
            }

            Vertex V1 = ShapeCasts.Vertex(theDI.GetShape(At(a, 3 + i), ShapeType.Vertex));
            if (n == 5 + i)
            {
                if (V1.IsNull)
                {
                    if (C is not null)
                    {
                        edge = new BRepBuilderAPI_MakeEdge(C, theDI.Atof(a[3]), theDI.Atof(a[4])).Edge();
                    }
                    else if (S is null)
                    {
                        edge = new BRepBuilderAPI_MakeEdge2d(C2d!, theDI.Atof(a[3]), theDI.Atof(a[4])).Edge();
                    }
                    else
                    {
                        edge = new BRepBuilderAPI_MakeEdge(C2d!, S, theDI.Atof(a[4]), theDI.Atof(a[5])).Edge();
                    }
                }
                else
                {
                    Vertex V2 = ShapeCasts.Vertex(theDI.GetShape(a[4 + i], ShapeType.Vertex));
                    if (C is not null)
                    {
                        edge = new BRepBuilderAPI_MakeEdge(C, V1, V2).Edge();
                    }
                    else if (S is null)
                    {
                        edge = new BRepBuilderAPI_MakeEdge2d(C2d!, V1, V2).Edge();
                    }
                    else
                    {
                        edge = new BRepBuilderAPI_MakeEdge(C2d!, S, V1, V2).Edge();
                    }
                }
            }
            else if (n == 7 + i)
            {
                Vertex V2 = ShapeCasts.Vertex(theDI.GetShape(a[5 + i], ShapeType.Vertex));
                if (C is not null)
                {
                    edge = new BRepBuilderAPI_MakeEdge(C, V1, V2, theDI.Atof(a[4]), theDI.Atof(a[6])).Edge();
                }
                else if (S is null)
                {
                    edge = new BRepBuilderAPI_MakeEdge2d(C2d!, V1, V2, theDI.Atof(a[4]), theDI.Atof(a[6])).Edge();
                }
                else
                {
                    edge = new BRepBuilderAPI_MakeEdge(C2d!, S, V1, V2, theDI.Atof(a[5]), theDI.Atof(a[7])).Edge();
                }
            }
            else
            {
                return 1;
            }
        }

        theDI.SetShape(a[1], edge);
        return 0;
    }

    // a[i], or Tcl's null argv[argc] read as an empty name
    private static string At(IReadOnlyList<string> a, int i) => i < a.Count ? a[i] : "";

    private static int MkCurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            return 1;
        }

        // mkoricurve only changes the colour of the drawn curve
        Shape S = theDI.GetShape(a[2], ShapeType.Edge);
        if (S.IsNull)
        {
            return 1;
        }

        Curve3D? C = Tool.Curve(ShapeCasts.Edge(S), out Placement L, out double f, out double l);
        if (C is null)
        {
            di.Append(a[2]).Append(" has no 3d curve\n");
            return 1;
        }

        C = new TrimmedCurve3D(C, f, l);
        DrawTrSurf.Set(theDI, a[1], C.Transformed(L.Transformation()));
        return 0;
    }

    private static int MkPoint(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            return 1;
        }

        Shape S = theDI.GetShape(a[2], ShapeType.Vertex);
        if (S.IsNull)
        {
            return 1;
        }

        DrawTrSurf.Set(theDI, a[1], Tool.Pnt(ShapeCasts.Vertex(S)));
        return 0;
    }

    private static int Mk2dCurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int na = a.Count;
        if (na < 3)
        {
            return 1;
        }

        Shape S = theDI.GetShape(a[2], ShapeType.Edge);
        if (S.IsNull)
        {
            return 1;
        }

        Edge E = ShapeCasts.Edge(S);
        Placement L = new();
        double f = 0.0, l = 0.0;
        Curve2D? C = null;
        Surface? Surf = null;
        bool hasFace = false;
        if (na == 3)
        {
            // get the first PCurve connected to edge E
            Tool.CurveOnSurface(E, out C, out Surf, out L, out f, out l);
        }
        else if (na == 4)
        {
            S = theDI.GetShape(a[3], ShapeType.Face);
            if (S.IsNull)
            {
                int ind = theDI.Atoi(a[3]);
                Tool.CurveOnSurface(E, ref C, ref Surf, ref L, ref f, ref l, ind);
            }
            else
            {
                hasFace = true;
                C = Tool.CurveOnSurface(E, ShapeCasts.Face(S), out f, out l);
            }
        }

        if (C is null)
        {
            di.Append(a[2]).Append(" has no 2d curve");
            if (hasFace)
            {
                di.Append(" on ").Append(a[3]);
            }

            di.Append("\n");
            return 1;
        }

        DrawTrSurf.Set(theDI, a[1], new TrimmedCurve2D(C, f, l));
        return 0;
    }

    private static int Edge(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 4)
        {
            return 1;
        }

        Shape V1 = theDI.GetShape(a[2], ShapeType.Vertex);
        Shape V2 = theDI.GetShape(a[3], ShapeType.Vertex);
        if (V1.IsNull || V2.IsNull)
        {
            return 1;
        }

        theDI.SetShape(a[1], new BRepBuilderAPI_MakeEdge(ShapeCasts.Vertex(V1), ShapeCasts.Vertex(V2)).Edge());
        return 0;
    }

    private static int IsoEdge(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 6)
        {
            return 1;
        }

        bool uiso = a[0][0] == 'u';
        double p = theDI.Atof(a[3]);
        double p1 = theDI.Atof(a[4]);
        double p2 = theDI.Atof(a[5]);
        Shape Sh = theDI.GetShape(a[2], ShapeType.Face);
        if (Sh.IsNull)
        {
            return 1;
        }

        Surface S = Tool.Surface(ShapeCasts.Face(Sh), out Placement Loc)!;
        BrepTools.UVBounds(ShapeCasts.Face(Sh), out double UMin, out double UMax, out double VMin, out double VMax);
        if (uiso)
        {
            VMin = Math.Min(VMin, Math.Min(p1, p2));
            VMax = Math.Max(VMax, Math.Max(p1, p2));
        }
        else
        {
            UMin = Math.Min(UMin, Math.Min(p1, p2));
            // OCCT: the U maximum is taken from VMax
            UMax = Math.Max(VMax, Math.Max(p1, p2));
        }

        var TS = new TrimmedSurface(S, UMin, UMax, VMin, VMax);
        Curve3D C;
        LineCurve2D L;
        if (uiso)
        {
            C = TS.UIso(p);
            L = new LineCurve2D(new Point2D(p, 0), new Direction2D(0, 1));
        }
        else
        {
            C = TS.VIso(p);
            L = new LineCurve2D(new Point2D(0, p), new Direction2D(1, 0));
        }

        // OCCT writes the location and the pcurve into the new edge before binding it
        Edge E = new BRepBuilderAPI_MakeEdge(C, p1, p2).InternalEdge();
        E.Location(Loc);
        new BrepBuilder().UpdateEdge(E, L, ShapeCasts.Face(Sh), 0.0);
        theDI.SetShape(a[1], E);
        return 0;
    }

    private const string ProfileHelp =
        "profile name [code values] [code values] ...\n"
      + "  Build a profile in the XY plane from a moving point and direction\n"
      + "  The original point and direction are 0 0 and 1 0\n"
      + "  Codes and values describe the point or direction change\n"
      + "  When the point change the direction becomes the tangent\n"
      + "  All angles are in degree (may be negative)\n"
      + "  By default the profile is closed and a face is built\n"
      + "\n"
      + "  Instruction Parameters         Action\n"
      + "  O           X Y Z              Set the origin of the plane\n"
      + "  P           DX DY DZ UX UY UZ  Set the normal and X of the plane\n"
      + "  S           Face               Set the Plane (Face must be a Face)\n"
      + "  F           X Y                Set the first point\n"
      + "  X           DX                 Translate point along X\n"
      + "  Y           DY                 Translate point along Y\n"
      + "  L           DL                 Translate point along direction\n"
      + "  XX          X                  Set point X coordinate\n"
      + "  YY          Y                  Set point Y coordinate\n"
      + "  T           DX DY              Translate point\n"
      + "  TT          X Y                Set point\n"
      + "  R           Angle              Rotate direction\n"
      + "  RR          Angle              Set direction\n"
      + "  D           DX DY              Set direction\n"
      + "  IX          X                  Intersect with vertical\n"
      + "  IY          Y                  Intersect with horizontal\n"
      + "  C           Radius Angle       Arc of circle tangent to direction\n"
      + "  W                              Make a closed wire\n"
      + "  WW                             Make an open wire\n"
      + "\n";

    private enum Move
    {
        Line,
        Circle,
        None,
    }

    // the second character of a code, or C's terminating zero
    private static char Second(string theCode) => theCode.Length > 1 ? theCode[1] : '\0';

    /// <summary>
    /// profile: OCCT's moving point. The edges, the wire and the face are taken unfrozen from the makers, since
    /// OCCT builds the 3D curves of a profile on a non-planar face in place (BRepLib::BuildCurves3d) before binding.
    /// </summary>
    private static int Profile(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n == 1)
        {
            di.Append(ProfileHelp);
            return 0;
        }

        int i = 2;
        double x0 = 0, y0 = 0, x = 0, y = 0, dx = 1, dy = 0;
        var MW = new BRepBuilderAPI_MakeWire();
        var P = new Plane3D(new CoordinateSystem(Constants.XOY()));
        bool face = true;
        bool close = true;
        bool first = true;
        bool stayfirst = false;
        bool isplanar = true;
        Shape S;
        Placement TheLocation = new();
        Surface? Surface = null;
        while (i < n)
        {
            double length = 0, radius = 0, angle = 0;
            Move move = Move.None;
            switch (a[i].Length > 0 ? a[i][0] : '\0')
            {
                case 'F':
                case 'f':
                    i += 2;
                    if (i >= n)
                    {
                        goto badargs;
                    }

                    if (!first)
                    {
                        di.Append("profile: The F instruction must precede all moves");
                        return 1;
                    }

                    x0 = x = theDI.Atof(a[i - 1]);
                    y0 = y = theDI.Atof(a[i]);
                    stayfirst = true;
                    break;

                case 'O':
                case 'o':
                    i += 3;
                    if (i >= n)
                    {
                        goto badargs;
                    }

                    P.SetLocation(new Point3D(theDI.Atof(a[i - 2]), theDI.Atof(a[i - 1]), theDI.Atof(a[i])));
                    stayfirst = true;
                    break;

                case 'P':
                case 'p':
                    i += 6;
                    if (i >= n)
                    {
                        goto badargs;
                    }

                    {
                        var vn = new Vector3D(theDI.Atof(a[i - 5]), theDI.Atof(a[i - 4]), theDI.Atof(a[i - 3]));
                        var vx = new Vector3D(theDI.Atof(a[i - 2]), theDI.Atof(a[i - 1]), theDI.Atof(a[i]));
                        if (vn.Magnitude() <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            di.Append("profile : null direction");
                            return 1;
                        }

                        if (vx.Magnitude() <= global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            di.Append("profile : null direction");
                            return 1;
                        }

                        var ax = new Frame3D(P.Location(), new Direction3D(vn), new Direction3D(vx));
                        P.SetPosition(new CoordinateSystem(ax));
                    }

                    stayfirst = true;
                    break;

                case 'S':
                case 's':
                    i += 1;
                    if (i >= n)
                    {
                        goto badargs;
                    }

                    {
                        Face Face = ShapeCasts.Face(theDI.GetShape(a[i], ShapeType.Face));
                        if (Face.IsNull)
                        {
                            di.Append("profile : no face found");
                            return 1;
                        }

                        Surface = Tool.Surface(Face, out TheLocation);
                        if (Surface is PlaneSurface Plane)
                        {
                            P = Plane.Pln();
                        }
                        else
                        {
                            isplanar = false;
                        }
                    }

                    stayfirst = true;
                    break;

                case 'X':
                case 'x':
                    i++;
                    if (i >= n)
                    {
                        goto badargs;
                    }

                    length = theDI.Atof(a[i]);
                    if (Second(a[i - 1]) is 'X' or 'x')
                    {
                        length -= x;
                    }

                    dx = 1;
                    dy = 0;
                    move = Move.Line;
                    break;

                case 'Y':
                case 'y':
                    i++;
                    if (i >= n)
                    {
                        goto badargs;
                    }

                    length = theDI.Atof(a[i]);
                    if (Second(a[i - 1]) is 'Y' or 'y')
                    {
                        length -= y;
                    }

                    dx = 0;
                    dy = 1;
                    move = Move.Line;
                    break;

                case 'L':
                case 'l':
                    i++;
                    if (i >= n)
                    {
                        goto badargs;
                    }

                    length = theDI.Atof(a[i]);
                    move = Move.Line;
                    break;

                case 'T':
                case 't':
                    i += 2;
                    if (i >= n)
                    {
                        goto badargs;
                    }

                    {
                        double vx = theDI.Atof(a[i - 1]);
                        double vy = theDI.Atof(a[i]);
                        if (Second(a[i - 2]) is 'T' or 't')
                        {
                            vx -= x;
                            vy -= y;
                        }

                        length = Math.Sqrt(vx * vx + vy * vy);
                        if (length > global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            move = Move.Line;
                            dx = vx / length;
                            dy = vy / length;
                        }
                    }

                    break;

                case 'R':
                case 'r':
                    i++;
                    if (i >= n)
                    {
                        goto badargs;
                    }

                    angle = theDI.Atof(a[i]) * (Math.PI / 180.0);
                    if (Second(a[i - 1]) is 'R' or 'r')
                    {
                        dx = Math.Cos(angle);
                        dy = Math.Sin(angle);
                    }
                    else
                    {
                        double c = Math.Cos(angle);
                        double s = Math.Sin(angle);
                        double t = c * dx - s * dy;
                        dy = s * dx + c * dy;
                        dx = t;
                    }

                    break;

                case 'D':
                case 'd':
                    i += 2;
                    if (i >= n)
                    {
                        goto badargs;
                    }

                    {
                        double vx = theDI.Atof(a[i - 1]);
                        double vy = theDI.Atof(a[i]);
                        length = Math.Sqrt(vx * vx + vy * vy);
                        if (length > global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            dx = vx / length;
                            dy = vy / length;
                        }
                    }

                    break;

                case 'C':
                case 'c':
                    i += 2;
                    if (i >= n)
                    {
                        goto badargs;
                    }

                    radius = theDI.Atof(a[i - 1]);
                    if (Math.Abs(radius) > global::Kerf.Engine.Foundation.Precision.Confusion())
                    {
                        angle = theDI.Atof(a[i]) * (Math.PI / 180.0);
                        move = Move.Circle;
                    }

                    break;

                case 'I':
                case 'i':
                    i++;
                    if (i >= n)
                    {
                        goto badargs;
                    }

                    length = theDI.Atof(a[i]);
                    if (Second(a[i - 1]) is 'X' or 'x')
                    {
                        if (Math.Abs(dx) < global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            di.Append("Profile : cannot intersect, arg ").Append(i - 1);
                            return 1;
                        }

                        length = (length - x) / dx;
                        move = Move.Line;
                    }
                    else if (Second(a[i - 1]) is 'Y' or 'y')
                    {
                        if (Math.Abs(dy) < global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            di.Append("Profile : cannot intersect, arg ").Append(i - 1);
                            return 1;
                        }

                        length = (length - y) / dy;
                        move = Move.Line;
                    }

                    break;

                case 'W':
                case 'w':
                    face = false;
                    if (Second(a[i]) is 'W' or 'w')
                    {
                        close = false;
                    }

                    i = n - 1;
                    break;

                default:
                    di.Append("profile : unknown code ").Append(a[i]);
                    return 1;
            }

        again:
            switch (move)
            {
                case Move.Line:
                {
                    if (length < 0)
                    {
                        length = -length;
                        dx = -dx;
                        dy = -dy;
                    }

                    var l = new LineCurve2D(new Point2D(x, y), new Direction2D(dx, dy));
                    MW.Add(isplanar
                               ? new BRepBuilderAPI_MakeEdge(GeomAPI.To3d(l, P), 0, length).InternalEdge()
                               : new BRepBuilderAPI_MakeEdge(l, Surface!, 0, length).InternalEdge());
                    x += length * dx;
                    y += length * dy;
                    break;
                }

                case Move.Circle:
                {
                    bool sense = true;
                    if (radius < 0)
                    {
                        radius = -radius;
                        sense = !sense;
                        dx = -dx;
                        dy = -dy;
                    }

                    var ax = new Axis2D(new Point2D(x - radius * dy, y + radius * dx), new Direction2D(dy, -dx));
                    if (angle < 0)
                    {
                        angle = -angle;
                        sense = !sense;
                    }

                    var c = new CircleCurve2D(ax, radius, sense);
                    MW.Add(isplanar
                               ? new BRepBuilderAPI_MakeEdge(GeomAPI.To3d(c, P), 0, angle).InternalEdge()
                               : new BRepBuilderAPI_MakeEdge(c, Surface!, 0, angle).InternalEdge());
                    c.D1(angle, out Point2D p, out Vector2D v);
                    x = p.X();
                    y = p.Y();
                    dx = v.X() / radius;
                    dy = v.Y() / radius;
                    break;
                }

                case Move.None:
                    break;
            }

            // update first
            first = stayfirst;
            stayfirst = false;

            // next segment....
            i++;
            if (i == n && close)
            {
                // the closing segment
                dx = x0 - x;
                dy = y0 - y;
                length = Math.Sqrt(dx * dx + dy * dy);
                if (length > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    move = Move.Line;
                    dx /= length;
                    dy /= length;
                    goto again;
                }
            }
        }

        // get the result, face or wire
        if (face)
        {
            if (isplanar)
            {
                S = new BRepBuilderAPI_MakeFace(P, MW.InternalWire()).InternalFace();
            }
            else
            {
                var MFace = new BRepBuilderAPI_MakeFace();
                MFace.Init(Surface!, false, global::Kerf.Engine.Foundation.Precision.Confusion());
                MFace.Add(MW.InternalWire());
                S = MFace.InternalFace();
            }
        }
        else
        {
            S = MW.InternalWire();
        }

        if (!TheLocation.IsIdentity)
        {
            S.Move(TheLocation);
        }

        if (!isplanar)
        {
            double Tol = 1.0e-5;
            BRepLib.BuildCurves3d(S, Tol);
        }

        theDI.SetShape(a[1], S);
        return 0;

    badargs:
        di.Append("profile : bad number of arguments");
        return 1;
    }

    // mkoffset result face/compound of wires nboffset stepoffset [-approx] [jointype(a/i) [alt]]
    private static int MkOffset(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 5)
        {
            return 1;
        }

        BRepOffsetAPI_MakeOffset Paral = new();
        bool ToApprox = false;
        JoinKind theJoinType = JoinKind.Arc;

        int anIndArg = 6;
        if (n >= 6)
        {
            if (a[5] == "-approx")
            {
                ToApprox = true;
                anIndArg++;
            }

            if (n >= anIndArg && a[anIndArg - 1] == "i")
            {
                theJoinType = JoinKind.Intersection;
            }
        }

        Shape Base = theDI.GetShape(a[2], ShapeType.Face);

        if (Base.IsNull)
        {
            Base = theDI.GetShape(a[2]);
            if (Base.IsNull)
            {
                return 1;
            }
            Paral.Init(theJoinType);
            Explorer exp = new();
            for (exp.Init(Base, ShapeType.Wire); exp.More(); exp.Next())
            {
                Wire aLocalShape = ShapeCasts.Wire(exp.Current());
                Paral.AddWire(aLocalShape);
            }
        }
        else
        {
            Base = Base.Oriented(Orientation.Forward);
            Paral.Init(ShapeCasts.Face(Base), theJoinType);
        }

        Paral.SetApprox(ToApprox);

        double U, dU;
        int Nb;
        dU = theDI.Atof(a[4]);
        Nb = theDI.Atoi(a[3]);

        double Alt = 0.0;
        if (n > anIndArg)
        {
            Alt = theDI.Atof(a[anIndArg]);
        }

        int Compt = 1;

        for (int i = 1; i <= Nb; i++)
        {
            U = i * dU;
            Paral.Perform(U, Alt);

            if (!Paral.IsDone())
            {
                di.Append(" Error: Offset is not done.\n");
                return 1;
            }
            else
            {
                theDI.SetShape($"{a[1]}_{Compt++}", Paral.Shape());
            }
        }

        return 0;
    }

    // openoffset result face/wire nboffset stepoffset [-approx] [jointype(a/i)]
    private static int OpenOffset(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 5)
        {
            return 1;
        }

        BRepOffsetAPI_MakeOffset Paral = new();
        bool ToApprox = false;
        JoinKind theJoinType = JoinKind.Arc;

        int anIndArg = 6;
        if (n >= 6)
        {
            if (a[5] == "-approx")
            {
                ToApprox = true;
                anIndArg++;
            }

            if (n >= anIndArg && a[anIndArg - 1] == "i")
            {
                theJoinType = JoinKind.Intersection;
            }
        }

        Shape Base = theDI.GetShape(a[2], ShapeType.Face);

        if (Base.IsNull)
        {
            Base = theDI.GetShape(a[2], ShapeType.Wire);
            if (Base.IsNull)
            {
                return 1;
            }
            Paral.Init(theJoinType, true);
            Paral.AddWire(ShapeCasts.Wire(Base));
        }
        else
        {
            Base = Base.Oriented(Orientation.Forward);
            Paral.Init(ShapeCasts.Face(Base), theJoinType, true);
        }

        Paral.SetApprox(ToApprox);

        double U, dU;
        int Nb;
        dU = theDI.Atof(a[4]);
        Nb = theDI.Atoi(a[3]);

        double Alt = 0.0;
        int Compt = 1;

        for (int i = 1; i <= Nb; i++)
        {
            U = i * dU;
            Paral.Perform(U, Alt);

            if (!Paral.IsDone())
            {
                di.Append(" Error: Offset is not done.\n");
                return 1;
            }
            else
            {
                theDI.SetShape($"{a[1]}_{Compt++}", Paral.Shape());
            }
        }

        return 0;
    }
}
