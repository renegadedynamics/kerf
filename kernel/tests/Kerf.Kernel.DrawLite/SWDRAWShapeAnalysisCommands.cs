// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// The commands of SWDRAW_ShapeAnalysis.cxx (unit 6L): <c>projface</c>, <c>projcurve</c>, <c>projpcurve</c>,
/// <c>statshape</c>, <c>comptol</c>, <c>freebounds</c>, <c>fbprops</c>, <c>fbclose</c>, <c>getareacontour</c>,
/// <c>checkselfintersection</c>, <c>checkedge</c>, <c>getanasurf</c> and <c>getanacurve</c>. <c>tolerance</c> is
/// BRepTestCommands' (unit 3L1).
/// </summary>
/// <remarks>
/// Left out: <c>anaface</c>, a French-language debugging dump that no corpus case calls. <c>fbclose</c>'s
/// ShapeFix_FreeBounds replaces vertices in the edges of the shape it is given, in place in DRAWEXE; the port's copies
/// a frozen shape at the entry (decision 583), so the variable keeps its shape.
/// </remarks>
internal static class SWDRAWShapeAnalysisCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "projface", ProjFace);
        DrawCxx.Add(theInterpreter, "projcurve", ProjCurve);
        DrawCxx.Add(theInterpreter, "projpcurve", ProjPCurve);
        DrawCxx.Add(theInterpreter, "statshape", StatShape);
        DrawCxx.Add(theInterpreter, "comptol", CompTolEdge);
        DrawCxx.Add(theInterpreter, "freebounds", FreeBounds);
        DrawCxx.Add(theInterpreter, "fbprops", FreeBoundsProps);
        DrawCxx.Add(theInterpreter, "fbclose", CloseFreeBounds);
        DrawCxx.Add(theInterpreter, "getareacontour", GetAreaContour);
        DrawCxx.Add(theInterpreter, "checkselfintersection", CheckSelfIntersection);
        DrawCxx.Add(theInterpreter, "checkedge", CheckEdge);
        DrawCxx.Add(theInterpreter, "getanasurf", GetAnaSurf);
        DrawCxx.Add(theInterpreter, "getanacurve", GetAnaCurve);
    }

    private static int ProjFace(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 4)
        {
            di.Append("Give FACE name and X Y [Z]\n");
            return 1;
        }
        string arg1 = argv[1];
        Shape Shape = theDI.GetShape(arg1);
        if (Shape.IsNull)
        {
            di.Append("Shape unknown : ").Append(arg1).Append("\n");
            return 1 /* Error */;
        }
        if (Shape.Type != ShapeType.Face)
        {
            di.Append("Not a face\n");
            return 1 /* Error */;
        }
        Face F = ShapeCasts.Face(Shape);
        Surface thesurf = Tool.Surface(F)!; // pas locface
        var aClassifier = new FClass2d(F, global::Kerf.Engine.Foundation.Precision.Confusion());
        //  On y va
        double X, Y, Z, U, V;
        X = U = theDI.Atof(argv[2]);
        Y = V = theDI.Atof(argv[3]);
        if (argc > 4)
        {
            Z = theDI.Atof(argv[4]);
            var P3D = new Point3D(X, Y, Z);
            di.Append(" Point 3D X = ").Append(X).Append("  Y = ").Append(Y).Append("  Z = ").Append(Z).Append("\n");
            thesurf.ParameterBounds(out double uf, out double ul, out double vf, out double vl);

            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(uf))
            {
                uf = -1000;
            }
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(ul))
            {
                ul = 1000;
            }
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(vf))
            {
                vf = -1000;
            }
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(vl))
            {
                vl = 1000;
            }
            double du = Math.Abs(ul - uf) / 10;
            double dv = Math.Abs(vl - vf) / 10;

            var proj = new ProjectPointOnSurface(P3D, thesurf, uf - du, ul + du, vf - dv, vl + dv);
            int sol, nPSurf = proj.NbPoints();
            int anIndSol = 0, anIndMin = 0;
            double aMinDist = global::Kerf.Engine.Foundation.Standard.RealLast();
            for (sol = 1; sol <= nPSurf; sol++)
            {
                proj.Parameters(sol, out U, out V);
                Containment aStatus = aClassifier.Perform(new Point2D(U, V));
                if (aStatus == Containment.Out)
                {
                    continue;
                }

                anIndSol++;
                double aDist = proj.Distance(sol);
                di.Append("n0 ").Append(anIndSol).Append(" Distance ").Append(aDist);
                di.Append("  U = ").Append(U).Append("  V = ").Append(V).Append("\n");

                if (aDist < aMinDist)
                {
                    aMinDist = aDist;
                    anIndMin = sol;
                }

                //  reprojection
                P3D = thesurf.Value(U, V);
                di.Append("  => reproj  X = ").Append(P3D.X).Append("  Y = ").Append(P3D.Y).Append("  Z = ").Append(P3D.Z).Append("\n");
            }
            di.Append(" Found ").Append(anIndSol).Append(" Points\n");

            if (anIndMin != 0) // there is at least one suitable solution
            {
                di.Append("** Minimal distance to face = ").Append(aMinDist).Append("\n");
                proj.Parameters(anIndMin, out U, out V);
                di.Append("**  Solution of minimal distance:  U = ").Append(U).Append("  V = ").Append(V).Append("\n");
                P3D = thesurf.Value(U, V);
                di.Append("  => reproj  X = ").Append(P3D.X).Append("  Y = ").Append(P3D.Y).Append("  Z = ").Append(P3D.Z).Append("\n");
            }
        }
        else // Check 2D point
        {
            di.Append(" Point UV  U = ").Append(U).Append("  V = ").Append(V).Append("\n");
            Containment aStatus = aClassifier.Perform(new Point2D(U, V));
            if (aStatus == Containment.Out)
            {
                di.Append("does not belong to the face").Append("\n");
            }
            else
            {
                Point3D P3D = thesurf.Value(U, V);
                di.Append(" => proj  X = ").Append(P3D.X).Append("  Y = ").Append(P3D.Y).Append("  Z = ").Append(P3D.Z).Append("\n");
            }
        }
        return 0;
    }

    private static int ProjCurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        //  admet une EDGE ou une CURVE
        if (argc < 5)
        {
            di.Append("Give : EDGE X Y Z  or  CURVE3D X Y Z  or  CURVE3D first last X Y Z\n");
            return 1 /* Error */;
        }
        string arg1 = argv[1];
        int i0 = 0;
        Shape Shape = theDI.GetShape(arg1);

        Curve3D? C;
        double cf, cl;
        if (!Shape.IsNull)
        {
            if (Shape.Type == ShapeType.Edge)
            {
                Edge E = ShapeCasts.Edge(Shape);
                C = Tool.Curve(E, out cf, out cl);
                di.Append("Edge ").Append(arg1).Append(" Params from ").Append(cf).Append(" to ").Append(cl).Append("\n");
            }
            else
            {
                di.Append("SHAPE ").Append(arg1).Append(" Not an EDGE\n");
                return 1 /* Error */;
            }
        }
        else
        {
            C = DrawTrSurf.GetCurve(theDI, arg1);
            if (C is null)
            {
                di.Append(arg1).Append(" neither EDGE nor CURVE 3D\n");
                return 1 /* Error */;
            }
            cf = C.FirstParameter;
            cl = C.LastParameter;
            if (argc >= 7)
            {
                cf = theDI.Atof(argv[2]);
                cl = theDI.Atof(argv[3]);
                i0 = 2;
            }
            di.Append("Curve 3D ").Append(arg1).Append(" Params from ").Append(cf).Append(" to ").Append(cl).Append("\n");
        }

        double X, Y, Z;
        X = theDI.Atof(argv[2 + i0]);
        Y = theDI.Atof(argv[3 + i0]);
        Z = theDI.Atof(argv[4 + i0]);
        di.Append("Precision (BRepBuilderAPI) : ").Append(BRepBuilderApi.Precision()).Append("  Projection : ").Append(X)
          .Append("  ").Append(Y).Append("  ").Append(Z).Append("\n");

        var P3D = new Point3D(X, Y, Z);
        double dist = new CurveAnalysis().Project(C!, P3D, BRepBuilderApi.Precision(), out Point3D res, out double param, cf, cl);
        res.Coord(out X, out Y, out Z);
        di.Append("Result : ").Append(X).Append("  ").Append(Y).Append("  ").Append(Z).Append("\nParam = ").Append(param)
          .Append("  Gap = ").Append(dist).Append("\n");
        return 0;
    }

    private static int ProjPCurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        //  admet une EDGE ou une CURVE
        if (argc < 7)
        {
            di.Append("Give : projpcurve edge face tol X Y Z [start_param]\n");
            return 1;
        }

        Edge aEdge = ShapeCasts.Edge(theDI.GetShape(argv[1]));
        if (aEdge.IsNull)
        {
            di.Append("SHAPE ").Append(argv[1]).Append(" is not an EDGE\n");
            return 1 /* Error */;
        }
        Face aFace = ShapeCasts.Face(theDI.GetShape(argv[2]));
        if (aFace.IsNull)
        {
            di.Append("SHAPE ").Append(argv[2]).Append(" is not a FACE\n");
            return 1 /* Error */;
        }

        double aTol = theDI.Atof(argv[3]);

        double X = theDI.Atof(argv[4]);
        double Y = theDI.Atof(argv[5]);
        double Z = theDI.Atof(argv[6]);

        var aP3D = new Point3D(X, Y, Z);

        bool IsStartPoint = false;
        double startpar = 0.0;
        if (argc > 7)
        {
            startpar = theDI.Atof(argv[7]);
            IsStartPoint = true;
        }

        var aCOnS = new CurveOnSurface(new EdgeCurve2DAdaptor(aEdge, aFace), new FaceSurfaceAdaptor(aFace, false));

        Point3D aPnt;
        double aParam;
        var aTool = new CurveAnalysis();
        double aDist;
        if (IsStartPoint)
        {
            aDist = aTool.NextProject(startpar, aCOnS, aP3D, aTol, out aPnt, out aParam);
        }
        else
        {
            aDist = aTool.Project(aCOnS, aP3D, aTol, out aPnt, out aParam, false);
        }

        di.Append("Point:").Append("\n").Append(aPnt.X).Append(" ").Append(aPnt.Y).Append(" ").Append(aPnt.Z).Append("\n");
        di.Append("Param: ").Append(aParam).Append("\n");
        di.Append("Dist: ").Append(aDist).Append("\n");
        return 0;
    }

    private static int StatShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 2)
        {
            di.Append("ShapeName; + options : prefix for particular cases : what to get\n");
            return 1 /* Error */;
        }
        string arg1 = argv[1];
        string? arg2 = null;
        string? arg3 = null;
        if (argc > 2)
        {
            arg2 = argv[2];
        }
        if (argc > 3)
        {
            arg3 = argv[3];
        }
        Shape Shape = theDI.GetShape(arg1);
        if (Shape.IsNull)
        {
            di.Append("Shape unknown : ").Append(arg1).Append("\n");
            return 1 /* Error */;
        }

        var analyzer = new ShapeContentsAnalysis();
        if (arg3 is not null)
        {
            // strstr("bigspl", arg3): the option is a part of the keyword
            analyzer.ModifyBigSplineMode() = "bigspl".Contains(arg3, StringComparison.Ordinal);
            analyzer.ModifyIndirectMode() = "indsur".Contains(arg3, StringComparison.Ordinal);
            analyzer.ModifyOffsetSurfaceMode() = "ofsur".Contains(arg3, StringComparison.Ordinal);
            analyzer.ModifyTrimmed3dMode() = "trc3d".Contains(arg3, StringComparison.Ordinal);
            analyzer.ModifyOffsetCurveMode() = "ofcur".Contains(arg3, StringComparison.Ordinal);
            analyzer.ModifyTrimmed2dMode() = "trc2d".Contains(arg3, StringComparison.Ordinal);
        }
        analyzer.Perform(Shape);

        void Line(int theNb, string theText)
        {
            if (theNb > 0)
            {
                di.Append(theNb).Append(theText);
            }
        }

        di.Append("Count    Item\n-----    ----\n");
        Line(analyzer.EdgeCount, "\t EDGE (Oriented)\n");
        Line(analyzer.SharedEdgeCount, "\t EDGE (Shared)\n");
        Line(analyzer.FreeEdgeCount, "\t EDGE (Free)\n");
        Line(analyzer.FaceCount, "\t FACE\n");
        Line(analyzer.FreeFaceCount, "\t FACE (Free)\n");
        Line(analyzer.FreeWireCount, "\t WIRE (Free)\n");
        Line(analyzer.ShellCount, "\t SHELL\n");
        Line(analyzer.SolidCount, "\t SOLID\n");
        Line(analyzer.VertexCount, "\t VERTEX (Oriented)\n");
        Line(analyzer.SharedVertexCount, "\t VERTEX (Shared)\n");
        Line(analyzer.WireCount, "\t WIRE\n");
        Line(analyzer.FaceWithSevWireCount, "\tFace with more than one wire\n");
        Line(analyzer.NoPCurveCount, "\tNo pcurve\n");
        Line(analyzer.SolidsWithVoidCount, "\tSOLID with voids\n");
        Line(analyzer.WireWitnSeamCount, "\tWire(s) with one seam edge\n");
        Line(analyzer.WireWithSevSeamCount, "\tWire(s) with several seam edges\n");
        Line(analyzer.BigSplineCount, "\tbigspl : BSpline > 8192 poles\n");
        Line(analyzer.BezierSurfCount, "\tbezsur : BezierSurface\n");
        Line(analyzer.BSplibeSurfCount, "\tbspsur : BSplineSurface\n");
        Line(analyzer.C0CurveCount, "\tc0curv : Curve Only C0\n");
        Line(analyzer.C0SurfaceCount, "\tc0surf : Surface Only C0\n");
        Line(analyzer.IndirectSurfCount, "\tindsur : Indirect Surface\n");
        Line(analyzer.OffsetCurveCount, "\tofcur  : Offset Curve(s)\n");
        Line(analyzer.OffsetSurfCount, "\tofsur  : Offset Surface\n");
        Line(analyzer.TrimmedCurve2dCount, "\ttrc2d  : Trimmed Curve2d\n");
        Line(analyzer.TrimmedCurve3dCount, "\ttrc3d  : Trimmed Curve3d\n");
        Line(analyzer.TrimSurfCount, "\ttrimsu : RectangularTrimmedSurface\n");

        if (arg3 is null)
        {
            return 0;
        }

        void Save(bool theMode, NCollection_HSequence<Shape> theSec, string theKey)
        {
            if (!theMode)
            {
                return;
            }
            for (int i = 1; i <= theSec.Length(); i++)
            {
                theDI.SetShape($"{arg2}_{theKey}_{i}", theSec.Value(i));
            }
        }

        Save(analyzer.ModifyBigSplineMode(), analyzer.BigSplineSec(), "bigspl");
        Save(analyzer.ModifyIndirectMode(), analyzer.IndirectSec(), "indsur");
        Save(analyzer.ModifyOffsetSurfaceMode(), analyzer.OffsetSurfaceSec(), "ofsur");
        Save(analyzer.ModifyTrimmed3dMode(), analyzer.Trimmed3dSec(), "trc3d");
        Save(analyzer.ModifyOffsetCurveMode(), analyzer.OffsetCurveSec(), "ofcur");
        Save(analyzer.ModifyTrimmed2dMode(), analyzer.Trimmed2dSec(), "trc2d");
        return 0;
    }

    private static int CompTolEdge(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 2)
        {
            di.Append("Use:\n"
                      + "> comptol shape [nbpoints=371] [prefix]\n\n"
                      + "Computes real tolerance of edges in the shape \n"
                      + "as maximal deviation of 3d curve and pcurves.\n"
                      + "Deviation is computed by nbpoints sample points (default is 371).\n"
                      + "Gives the max, min and average value on all edges in the shape\n"
                      + "If prefix is defined, edges with maximal real tolerance and\n"
                      + "relation (and corresponding faces) will be saved with such names\n\n");
            return 0;
        }

        Shape shape = theDI.GetShape(argv[1]);
        if (shape.IsNull)
        {
            di.Append("Shape name \"").Append(argv[1]).Append("\" is invalid\n");
            return 1;
        }
        int nbpnts = 371;
        string? prefix = null;
        if (argc > 2)
        {
            if (argv[2].Length > 0 && char.IsAsciiDigit(argv[2][0]))
            {
                nbpnts = theDI.Atoi(argv[2]);
                if (nbpnts < 2)
                {
                    nbpnts = 2;
                }
                if (argc > 3)
                {
                    prefix = argv[3];
                }
            }
            else
            {
                prefix = argv[2];
            }
        }

        int num = 0;
        double max = 0, min = 0, ave = 0.0;
        double relmax = 0, relmin = 0, relave = 0.0;
        var sae = new EdgeAnalysis();
        Edge edmax = new(), edmaxrel = new();
        for (var exp = new Explorer(shape, ShapeType.Edge); exp.More(); exp.Next())
        {
            double tol = 0;
            Edge edge = ShapeCasts.Edge(exp.Current());
            sae.CheckSameParameter(edge, ref tol, nbpnts);
            double t = Tool.Tolerance(edge);
            double rel = tol / (t > global::Kerf.Engine.Foundation.Precision.Confusion() ? t : global::Kerf.Engine.Foundation.Precision.Confusion());
            ave += tol;
            relave += rel;
            if (num == 0)
            {
                max = min = tol;
                relmax = relmin = rel;
                edmax = edmaxrel = edge;
            }
            else
            {
                if (max < tol)
                {
                    max = tol;
                    edmax = edge;
                }
                if (min > tol)
                {
                    min = tol;
                }
                if (relmax < rel)
                {
                    relmax = rel;
                    edmaxrel = edge;
                }
                if (relmin > rel)
                {
                    relmin = rel;
                }
            }
            num++;
        }
        if (num == 0)
        {
            di.Append("No edges found in the shape\n");
            return 1;
        }

        di.Append("Edges tolerance computed by ").Append(nbpnts).Append(" points: \nMAX=").Append(max).Append(" AVG=")
          .Append(ave / num).Append(" MIN=").Append(min).Append("\n");
        di.Append("Relation real tolerance / tolerance set in edge\nMAX=").Append(relmax).Append(" AVG=").Append(relave / num)
          .Append(" MIN=").Append(relmin).Append("\n");
        if (!string.IsNullOrEmpty(prefix))
        {
            // Sprintf(name, "%.10s_edge_tol", prefix)
            string aPrefix = prefix.Length > 10 ? prefix[..10] : prefix;
            string name = aPrefix + "_edge_tol";
            theDI.SetShape(name, edmax);
            di.Append("Edge with max tolerance saved to ").Append(name);
            if (edmax.IsSame(edmaxrel))
            {
                di.Append("\n");
            }
            else
            {
                name = aPrefix + "_edge_rel";
                theDI.SetShape(name, edmaxrel);
                di.Append("; edge with max relation saved to ").Append(name).Append("\n");
            }
            int num1 = 0;
            for (var fac = new Explorer(shape, ShapeType.Face); fac.More(); fac.Next())
            {
                Face face = ShapeCasts.Face(fac.Current());
                for (var ed = new Explorer(face, ShapeType.Edge); ed.More(); ed.Next())
                {
                    Edge edge = ShapeCasts.Edge(ed.Current());
                    if (edge.IsSame(edmax) || edge.IsSame(edmaxrel))
                    {
                        if (num1 == 0)
                        {
                            di.Append("Concerned faces saved to shapes ");
                        }
                        name = $"{aPrefix}_{num1 + 1}";
                        theDI.SetShape(name, face);
                        if (num1 == 0)
                        {
                            di.Append("").Append(name);
                        }
                        else
                        {
                            di.Append(", ").Append(name);
                        }
                        num1++;
                        break;
                    }
                }
            }
            if (num1 > 0)
            {
                di.Append("\n");
            }
        }
        return 0;
    }

    private static int FreeBounds(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if ((n < 3) || (n > 5))
        {
            return 1;
        }
        Shape shape = theDI.GetShape(a[1]);
        if (shape.IsNull)
        {
            return 1;
        }
        double toler = theDI.Atof(a[2]);
        bool splitclosed = false, splitopen = false;
        if (n > 3)
        {
            splitclosed = theDI.Atoi(a[3]) != 0;
        }
        if (n > 4)
        {
            splitopen = theDI.Atoi(a[4]) != 0;
        }

        FreeBounds F;
        if (toler <= 0)
        {
            F = new FreeBounds(shape, splitclosed, splitopen);
        }
        else
        {
            F = new FreeBounds(shape, toler, splitclosed, splitopen);
        }

        Shape wires = F.GetClosedWires();
        string name = a[1] + "_c";
        theDI.SetShape(name, wires);
        di.Append(name).Append(" - closed wires\n");
        wires = F.GetOpenWires();
        name = a[1] + "_o";
        theDI.SetShape(name, wires);
        di.Append(name).Append(" - open wires\n");

        return 0;
    }

    private static void PrintProps(int i, FreeBoundDataAnalysis fbd, DrawResult di)
    {
        double area = fbd.Area();
        double perimeter = fbd.Perimeter();
        double ratio = fbd.Ratio();
        double width = fbd.Width();
        int notch = fbd.NotchCount;
        // Sprintf(" %d\t%12.5f\t%12.5f\t%12.5f\t%12.5f\t%d")
        di.Append(" ").Append(i).Append("\t").Append(Fixed(area)).Append("\t").Append(Fixed(perimeter)).Append("\t")
          .Append(Fixed(ratio)).Append("\t").Append(Fixed(width)).Append("\t").Append(notch).Append("\n");

        static string Fixed(double theValue)
            => theValue.ToString("F5", System.Globalization.CultureInfo.InvariantCulture).PadLeft(12);
    }

    private static int FreeBoundsProps(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if ((n < 2) || (n > 5))
        {
            di.Append("Usage : freeprops shapename [tolerance [splitclosed [splitopen]]]\n");
            return 1;
        }

        Shape source = theDI.GetShape(a[1]);
        if (source.IsNull)
        {
            di.Append("Error : unknown shape ").Append(a[1]).Append("\n");
            return 1;
        }

        double toler = 0.0;
        bool splitclosed = false, splitopen = false;
        if (n > 2)
        {
            toler = theDI.Atof(a[2]);
        }
        if (n > 3)
        {
            splitclosed = theDI.Atoi(a[3]) != 0;
        }
        if (n > 4)
        {
            splitopen = theDI.Atoi(a[4]) != 0;
        }
        var analyzer = new FreeBoundsPropertiesAnalysis();
        if (toler > 0)
        {
            analyzer.Init(source, toler, splitclosed, splitopen);
        }
        else
        {
            analyzer.Init(source, splitclosed, splitopen);
        }
        analyzer.Perform();
        var B = new BrepBuilder();

        int nb = analyzer.ClosedFreeBoundCount;
        di.Append("\n");
        di.Append(" \tArea mm2\tLength mm\tRatio L/W\tWidth mm\tNb notch\n");
        var closed = new Compound();
        B.MakeCompound(ref closed);
        if (nb != 0)
        {
            di.Append("Closed bounds properties\n");
            for (int i = 1; i <= nb; i++)
            {
                FreeBoundDataAnalysis fbd = analyzer.ClosedFreeBound(i);
                PrintProps(i, fbd, di);
                B.Add(closed, fbd.FreeBound());
            }
        }

        nb = analyzer.OpenFreeBoundCount;
        var open = new Compound();
        B.MakeCompound(ref open);
        if (nb != 0)
        {
            di.Append("Open bounds properties\n");
            for (int i = 1; i <= nb; i++)
            {
                FreeBoundDataAnalysis fbd = analyzer.OpenFreeBound(i);
                PrintProps(i, fbd, di);
                B.Add(open, fbd.FreeBound());
            }
        }

        string name = a[1] + "_c";
        di.Append(name).Append(" - closed wires,  ");
        theDI.SetShape(name, closed);
        name = a[1] + "_o";
        di.Append(name).Append(" - closed wires \n");
        theDI.SetShape(name, open);
        return 0;
    }

    private static int CloseFreeBounds(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if ((n < 4) || (n > 6))
        {
            return 1;
        }
        Shape shape = theDI.GetShape(a[1]);
        if (shape.IsNull)
        {
            return 1;
        }
        double sewtoler = theDI.Atof(a[2]), closetoler = theDI.Atof(a[3]);
        bool splitclosed = false, splitopen = false;
        if (n > 4)
        {
            splitclosed = theDI.Atoi(a[3]) != 0; // OCCT reads the closing tolerance's word, not a[4]
        }
        if (n > 5)
        {
            splitopen = theDI.Atoi(a[4]) != 0; // OCCT reads a[4], not a[5]
        }

        FreeBoundsFixer F;
        if (sewtoler <= 0)
        {
            F = new FreeBoundsFixer(shape, closetoler, splitclosed, splitopen);
        }
        else
        {
            F = new FreeBoundsFixer(shape, sewtoler, closetoler, splitclosed, splitopen);
        }

        Shape wires = F.GetClosedWires();
        string name = a[1] + "_c";
        theDI.SetShape(name, wires);
        di.Append(name).Append(" - closed wires\n");
        wires = F.GetOpenWires();
        name = a[1] + "_o";
        theDI.SetShape(name, wires);
        di.Append(name).Append(" - open wires\n");

        return 0;
    }

    private static int GetAreaContour(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }
        Shape shape = theDI.GetShape(a[1]);
        if (shape.IsNull)
        {
            di.Append("Shape is not defined\n");
            return 1;
        }
        if (shape.Type != ShapeType.Wire)
        {
            di.Append("invalid type of argument\n");
            return 1;
        }
        double anArea = ShapeAnalysisAnalysis.ContourArea(ShapeCasts.Wire(shape));
        di.Append("Area = ").Append(anArea).Append("\n");
        return 0;
    }

    private static int CheckSelfIntersection(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 2)
        {
            di.Append("Call please \"checkselfintersection wire [face]\"\n");
            return 1;
        }

        // Get wire.
        Shape wire = theDI.GetShape(argv[1]);
        if (wire.IsNull || wire.Type != ShapeType.Wire)
        {
            di.Append("A null shape or not a wire is used.\n");
            return 2;
        }

        // Get face if the user provided us with a face.
        Shape face = new();
        if (argc > 2)
        {
            face = theDI.GetShape(argv[2]);
            if (face.IsNull || face.Type != ShapeType.Face)
            {
                di.Append("A null shape or not a face is used.\n");
                return 3;
            }
        }

        // If the face is null, make a plane inside the wire.
        if (face.IsNull)
        {
            var mkFace = new MakeFace(ShapeCasts.Wire(wire), true);
            if (mkFace.IsDone)
            {
                face = mkFace.Face();
            }
            else
            {
                di.Append("Can't make a face for the wire. Provide please a face for analysis.\n");
                return 4;
            }
        }

        var analyser = new WireAnalysis(ShapeCasts.Wire(wire), ShapeCasts.Face(face), global::Kerf.Engine.Foundation.Precision.Confusion());
        bool result = analyser.CheckSelfIntersection();

        if (result)
        {
            di.Append("A self-intersecting wire.\n");
        }
        else
        {
            di.Append("Not self-intersecting wire.\n");
        }
        return 0;
    }

    private static int CheckEdge(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 2)
        {
            di.Append("Call please \"checkedge edge [face]\"\n");
            return 1;
        }

        // Get edge.
        Shape edge = theDI.GetShape(argv[1]);
        if (edge.IsNull || edge.Type != ShapeType.Edge)
        {
            di.Append("A null shape or not an edge is used.\n");
            return 2;
        }

        // Get face.
        Shape face = new();
        if (argc == 3)
        {
            face = theDI.GetShape(argv[2]);
            if (face.IsNull || face.Type != ShapeType.Face)
            {
                di.Append("A null shape or not a face is used.\n");
                return 3;
            }
        }

        // Analysis of the edge.
        var analyser = new EdgeAnalysis();
        bool isOk = true;

        // Curve 3D.
        if (analyser.HasCurve3d(ShapeCasts.Edge(edge)))
        {
            // Check vertices.
            if (analyser.CheckVerticesWithCurve3d(ShapeCasts.Edge(edge)))
            {
                isOk = false;
                di.Append("Vertices of the edge don't coincide with start/end points of 3d-curve (using tolerance of the vertices).\n");
            }
        }
        else
        {
            isOk = false;
            di.Append("Edge doesn't have a 3d-curve\n");
        }

        if (!face.IsNull)
        {
            // Curve 2D.
            if (analyser.HasPCurve(ShapeCasts.Edge(edge), ShapeCasts.Face(face)))
            {
                // Check vertices.
                if (analyser.CheckVerticesWithPCurve(ShapeCasts.Edge(edge), ShapeCasts.Face(face)))
                {
                    isOk = false;
                    di.Append("Vertices of the edge don't coincide with start/end points of 2d-curve (using tolerance of the vertices).\n");
                }
            }
            else
            {
                isOk = false;
                di.Append("Edge doesn't have a 2d-curve on this face\n");
            }
        }

        if (isOk)
        {
            di.Append("Edge seems OK.\n");
        }

        return 0;
    }

    private static int GetAnaSurf(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            di.Append("Usage: \n");
            di.Append("getanasurf res shape [target [tol [sample]]] \n");
            di.Append("target is required type of surface and can be: pln, cyl, con sph \n");
            di.Append("sample is surface of required type, which parameters are used as starting \n");
            di.Append("point for searching parameters of surface by Least Square method when input shape \n");
            di.Append("is edge or wire \n");
            return 1;
        }
        Shape sh = theDI.GetShape(a[2]);
        if (sh.IsNull)
        {
            return 1;
        }
        ShapeType aShType = sh.Type;
        if (aShType != ShapeType.Shell && aShType != ShapeType.Face
            && aShType != ShapeType.Edge && aShType != ShapeType.Wire)
        {
            di.Append("Wrong shape type, shape can be shell or face or edge or wire\n");
            return 1;
        }

        SurfaceKind[] aTargets =
            [SurfaceKind.Plane, SurfaceKind.Cylinder, SurfaceKind.Cone, SurfaceKind.Sphere];
        int isurf = 0;
        if (n > 3)
        {
            if (a[3] == "pln")
            {
                isurf = 0;
            }
            else if (a[3] == "cyl")
            {
                isurf = 1;
            }
            else if (a[3] == "con")
            {
                isurf = 2;
            }
            else if (a[3] == "sph")
            {
                isurf = 3;
            }
        }

        double tol = 1.0e-7;
        if (n > 4)
        {
            tol = theDI.Atof(a[4]);
        }

        // get sample target for edge and wire
        var aSampleSurf = new SurfaceAdaptor();
        if (n > 5 && (sh.Type == ShapeType.Edge || sh.Type == ShapeType.Wire))
        {
            Surface? aGSurf = DrawTrSurf.GetSurface(theDI, a[5]);
            if (aGSurf is null)
            {
                di.Append("Sample surface is null").Append("\n");
                return 1;
            }
            aSampleSurf.Load(aGSurf);
            SurfaceKind aSType = aSampleSurf.GetType();
            if (aSType != aTargets[isurf])
            {
                di.Append("Sample surface has wrong type").Append("\n");
                return 1;
            }
        }

        var aCanonRec = new CanonicalRecognitionAnalysis(sh);
        Surface? aRes = null;
        switch (aTargets[isurf])
        {
            case SurfaceKind.Plane:
            {
                var aPln = new Plane3D();
                if (aSampleSurf.GetType() == SurfaceKind.Plane)
                {
                    aPln = aSampleSurf.Plane();
                }
                if (aCanonRec.IsPlane(tol, ref aPln))
                {
                    aRes = new PlaneSurface(aPln);
                }
                break;
            }
            case SurfaceKind.Cylinder:
            {
                var aCyl = new Cylinder3D();
                if (aSampleSurf.GetType() == SurfaceKind.Cylinder)
                {
                    aCyl = aSampleSurf.Cylinder();
                }
                if (aCanonRec.IsCylinder(tol, ref aCyl))
                {
                    aRes = new CylindricalSurface(aCyl);
                }
                break;
            }
            case SurfaceKind.Cone:
            {
                var aCon = new Cone3D();
                if (aSampleSurf.GetType() == SurfaceKind.Cone)
                {
                    aCon = aSampleSurf.Cone();
                }
                if (aCanonRec.IsCone(tol, ref aCon))
                {
                    aRes = new ConicalSurface(aCon);
                }
                break;
            }
            case SurfaceKind.Sphere:
            {
                var aSph = new Sphere3D();
                if (aSampleSurf.GetType() == SurfaceKind.Sphere)
                {
                    aSph = aSampleSurf.Sphere();
                }
                if (aCanonRec.IsSphere(tol, ref aSph))
                {
                    aRes = new SphericalSurface(aSph);
                }
                break;
            }
        }

        if (aRes is not null)
        {
            DrawTrSurf.Set(theDI, a[1], aRes);
            di.Append("Gap = ").Append(aCanonRec.GetGap()).Append("\n");
        }
        else
        {
            di.Append("Cannot get required surface").Append("\n");
        }
        return 0;
    }

    private static int GetAnaCurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            di.Append("Usage: \n");
            di.Append("getanacurve res shape [target [tol]] \n");
            di.Append("target is required type of curve and can be: lin, cir, ell  \n");
            return 1;
        }
        Shape sh = theDI.GetShape(a[2]);
        if (sh.IsNull)
        {
            return 1;
        }
        ShapeType aShType = sh.Type;
        if (aShType != ShapeType.Wire && aShType != ShapeType.Edge)
        {
            di.Append("Wrong shape type, shape can be wire or an edge \n");
            return 1;
        }

        CurveKind[] aTargets = [CurveKind.Line, CurveKind.Circle, CurveKind.Ellipse];
        int icurv = 0;
        if (n > 3)
        {
            if (a[3] == "lin")
            {
                icurv = 0;
            }
            else if (a[3] == "cir")
            {
                icurv = 1;
            }
            else if (a[3] == "ell")
            {
                icurv = 2;
            }
        }

        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (n > 4)
        {
            tol = theDI.Atof(a[4]);
        }

        var aCanonRec = new CanonicalRecognitionAnalysis(sh);
        Curve3D? aRes = null;
        switch (aTargets[icurv])
        {
            case CurveKind.Line:
            {
                var aLin = new Line3D();
                if (aCanonRec.IsLine(tol, ref aLin))
                {
                    aRes = new LineCurve3D(aLin);
                }
                break;
            }
            case CurveKind.Circle:
            {
                var aCirc = new Circle3D();
                if (aCanonRec.IsCircle(tol, ref aCirc))
                {
                    aRes = new CircleCurve3D(aCirc);
                }
                break;
            }
            case CurveKind.Ellipse:
            {
                var anElips = new Ellipse3D();
                if (aCanonRec.IsEllipse(tol, ref anElips))
                {
                    aRes = new EllipseCurve3D(anElips);
                }
                break;
            }
        }

        if (aRes is not null)
        {
            DrawTrSurf.Set(theDI, a[1], aRes);
            di.Append("Gap = ").Append(aCanonRec.GetGap()).Append("\n");
        }
        else
        {
            di.Append("Cannot get required curve").Append("\n");
        }
        return 0;
    }
}
