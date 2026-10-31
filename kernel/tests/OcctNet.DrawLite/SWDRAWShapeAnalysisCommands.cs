// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

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
        TopoDS_Shape Shape = theDI.GetShape(arg1);
        if (Shape.IsNull())
        {
            di.Append("Shape unknown : ").Append(arg1).Append("\n");
            return 1 /* Error */;
        }
        if (Shape.ShapeType() != TopAbs_ShapeEnum.TopAbs_FACE)
        {
            di.Append("Not a face\n");
            return 1 /* Error */;
        }
        TopoDS_Face F = TopoDS.Face(Shape);
        Geom_Surface thesurf = BRep_Tool.Surface(F)!; // pas locface
        var aClassifier = new BRepTopAdaptor_FClass2d(F, Precision.Confusion());
        //  On y va
        double X, Y, Z, U, V;
        X = U = theDI.Atof(argv[2]);
        Y = V = theDI.Atof(argv[3]);
        if (argc > 4)
        {
            Z = theDI.Atof(argv[4]);
            var P3D = new gp_Pnt(X, Y, Z);
            di.Append(" Point 3D X = ").Append(X).Append("  Y = ").Append(Y).Append("  Z = ").Append(Z).Append("\n");
            thesurf.Bounds(out double uf, out double ul, out double vf, out double vl);

            if (Precision.IsInfinite(uf))
            {
                uf = -1000;
            }
            if (Precision.IsInfinite(ul))
            {
                ul = 1000;
            }
            if (Precision.IsInfinite(vf))
            {
                vf = -1000;
            }
            if (Precision.IsInfinite(vl))
            {
                vl = 1000;
            }
            double du = Math.Abs(ul - uf) / 10;
            double dv = Math.Abs(vl - vf) / 10;

            var proj = new GeomAPI_ProjectPointOnSurf(P3D, thesurf, uf - du, ul + du, vf - dv, vl + dv);
            int sol, nPSurf = proj.NbPoints();
            int anIndSol = 0, anIndMin = 0;
            double aMinDist = Standard.RealLast();
            for (sol = 1; sol <= nPSurf; sol++)
            {
                proj.Parameters(sol, out U, out V);
                TopAbs_State aStatus = aClassifier.Perform(new gp_Pnt2d(U, V));
                if (aStatus == TopAbs_State.TopAbs_OUT)
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
                di.Append("  => reproj  X = ").Append(P3D.X()).Append("  Y = ").Append(P3D.Y()).Append("  Z = ").Append(P3D.Z()).Append("\n");
            }
            di.Append(" Found ").Append(anIndSol).Append(" Points\n");

            if (anIndMin != 0) // there is at least one suitable solution
            {
                di.Append("** Minimal distance to face = ").Append(aMinDist).Append("\n");
                proj.Parameters(anIndMin, out U, out V);
                di.Append("**  Solution of minimal distance:  U = ").Append(U).Append("  V = ").Append(V).Append("\n");
                P3D = thesurf.Value(U, V);
                di.Append("  => reproj  X = ").Append(P3D.X()).Append("  Y = ").Append(P3D.Y()).Append("  Z = ").Append(P3D.Z()).Append("\n");
            }
        }
        else // Check 2D point
        {
            di.Append(" Point UV  U = ").Append(U).Append("  V = ").Append(V).Append("\n");
            TopAbs_State aStatus = aClassifier.Perform(new gp_Pnt2d(U, V));
            if (aStatus == TopAbs_State.TopAbs_OUT)
            {
                di.Append("does not belong to the face").Append("\n");
            }
            else
            {
                gp_Pnt P3D = thesurf.Value(U, V);
                di.Append(" => proj  X = ").Append(P3D.X()).Append("  Y = ").Append(P3D.Y()).Append("  Z = ").Append(P3D.Z()).Append("\n");
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
        TopoDS_Shape Shape = theDI.GetShape(arg1);

        Geom_Curve? C;
        double cf, cl;
        if (!Shape.IsNull())
        {
            if (Shape.ShapeType() == TopAbs_ShapeEnum.TopAbs_EDGE)
            {
                TopoDS_Edge E = TopoDS.Edge(Shape);
                C = BRep_Tool.Curve(E, out cf, out cl);
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
            cf = C.FirstParameter();
            cl = C.LastParameter();
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
        di.Append("Precision (BRepBuilderAPI) : ").Append(BRepBuilderAPI.Precision()).Append("  Projection : ").Append(X)
          .Append("  ").Append(Y).Append("  ").Append(Z).Append("\n");

        var P3D = new gp_Pnt(X, Y, Z);
        double dist = new ShapeAnalysis_Curve().Project(C!, P3D, BRepBuilderAPI.Precision(), out gp_Pnt res, out double param, cf, cl);
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

        TopoDS_Edge aEdge = TopoDS.Edge(theDI.GetShape(argv[1]));
        if (aEdge.IsNull())
        {
            di.Append("SHAPE ").Append(argv[1]).Append(" is not an EDGE\n");
            return 1 /* Error */;
        }
        TopoDS_Face aFace = TopoDS.Face(theDI.GetShape(argv[2]));
        if (aFace.IsNull())
        {
            di.Append("SHAPE ").Append(argv[2]).Append(" is not a FACE\n");
            return 1 /* Error */;
        }

        double aTol = theDI.Atof(argv[3]);

        double X = theDI.Atof(argv[4]);
        double Y = theDI.Atof(argv[5]);
        double Z = theDI.Atof(argv[6]);

        var aP3D = new gp_Pnt(X, Y, Z);

        bool IsStartPoint = false;
        double startpar = 0.0;
        if (argc > 7)
        {
            startpar = theDI.Atof(argv[7]);
            IsStartPoint = true;
        }

        var aCOnS = new Adaptor3d_CurveOnSurface(new BRepAdaptor_Curve2d(aEdge, aFace), new BRepAdaptor_Surface(aFace, false));

        gp_Pnt aPnt;
        double aParam;
        var aTool = new ShapeAnalysis_Curve();
        double aDist;
        if (IsStartPoint)
        {
            aDist = aTool.NextProject(startpar, aCOnS, aP3D, aTol, out aPnt, out aParam);
        }
        else
        {
            aDist = aTool.Project(aCOnS, aP3D, aTol, out aPnt, out aParam, false);
        }

        di.Append("Point:").Append("\n").Append(aPnt.X()).Append(" ").Append(aPnt.Y()).Append(" ").Append(aPnt.Z()).Append("\n");
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
        TopoDS_Shape Shape = theDI.GetShape(arg1);
        if (Shape.IsNull())
        {
            di.Append("Shape unknown : ").Append(arg1).Append("\n");
            return 1 /* Error */;
        }

        var analyzer = new ShapeAnalysis_ShapeContents();
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
        Line(analyzer.NbEdges(), "\t EDGE (Oriented)\n");
        Line(analyzer.NbSharedEdges(), "\t EDGE (Shared)\n");
        Line(analyzer.NbFreeEdges(), "\t EDGE (Free)\n");
        Line(analyzer.NbFaces(), "\t FACE\n");
        Line(analyzer.NbFreeFaces(), "\t FACE (Free)\n");
        Line(analyzer.NbFreeWires(), "\t WIRE (Free)\n");
        Line(analyzer.NbShells(), "\t SHELL\n");
        Line(analyzer.NbSolids(), "\t SOLID\n");
        Line(analyzer.NbVertices(), "\t VERTEX (Oriented)\n");
        Line(analyzer.NbSharedVertices(), "\t VERTEX (Shared)\n");
        Line(analyzer.NbWires(), "\t WIRE\n");
        Line(analyzer.NbFaceWithSevWires(), "\tFace with more than one wire\n");
        Line(analyzer.NbNoPCurve(), "\tNo pcurve\n");
        Line(analyzer.NbSolidsWithVoids(), "\tSOLID with voids\n");
        Line(analyzer.NbWireWitnSeam(), "\tWire(s) with one seam edge\n");
        Line(analyzer.NbWireWithSevSeams(), "\tWire(s) with several seam edges\n");
        Line(analyzer.NbBigSplines(), "\tbigspl : BSpline > 8192 poles\n");
        Line(analyzer.NbBezierSurf(), "\tbezsur : BezierSurface\n");
        Line(analyzer.NbBSplibeSurf(), "\tbspsur : BSplineSurface\n");
        Line(analyzer.NbC0Curves(), "\tc0curv : Curve Only C0\n");
        Line(analyzer.NbC0Surfaces(), "\tc0surf : Surface Only C0\n");
        Line(analyzer.NbIndirectSurf(), "\tindsur : Indirect Surface\n");
        Line(analyzer.NbOffsetCurves(), "\tofcur  : Offset Curve(s)\n");
        Line(analyzer.NbOffsetSurf(), "\tofsur  : Offset Surface\n");
        Line(analyzer.NbTrimmedCurve2d(), "\ttrc2d  : Trimmed Curve2d\n");
        Line(analyzer.NbTrimmedCurve3d(), "\ttrc3d  : Trimmed Curve3d\n");
        Line(analyzer.NbTrimSurf(), "\ttrimsu : RectangularTrimmedSurface\n");

        if (arg3 is null)
        {
            return 0;
        }

        void Save(bool theMode, NCollection_HSequence<TopoDS_Shape> theSec, string theKey)
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

        TopoDS_Shape shape = theDI.GetShape(argv[1]);
        if (shape.IsNull())
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
        var sae = new ShapeAnalysis_Edge();
        TopoDS_Edge edmax = new(), edmaxrel = new();
        for (var exp = new TopExp_Explorer(shape, TopAbs_ShapeEnum.TopAbs_EDGE); exp.More(); exp.Next())
        {
            double tol = 0;
            TopoDS_Edge edge = TopoDS.Edge(exp.Current());
            sae.CheckSameParameter(edge, ref tol, nbpnts);
            double t = BRep_Tool.Tolerance(edge);
            double rel = tol / (t > Precision.Confusion() ? t : Precision.Confusion());
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
            for (var fac = new TopExp_Explorer(shape, TopAbs_ShapeEnum.TopAbs_FACE); fac.More(); fac.Next())
            {
                TopoDS_Face face = TopoDS.Face(fac.Current());
                for (var ed = new TopExp_Explorer(face, TopAbs_ShapeEnum.TopAbs_EDGE); ed.More(); ed.Next())
                {
                    TopoDS_Edge edge = TopoDS.Edge(ed.Current());
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
        TopoDS_Shape shape = theDI.GetShape(a[1]);
        if (shape.IsNull())
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

        ShapeAnalysis_FreeBounds F;
        if (toler <= 0)
        {
            F = new ShapeAnalysis_FreeBounds(shape, splitclosed, splitopen);
        }
        else
        {
            F = new ShapeAnalysis_FreeBounds(shape, toler, splitclosed, splitopen);
        }

        TopoDS_Shape wires = F.GetClosedWires();
        string name = a[1] + "_c";
        theDI.SetShape(name, wires);
        di.Append(name).Append(" - closed wires\n");
        wires = F.GetOpenWires();
        name = a[1] + "_o";
        theDI.SetShape(name, wires);
        di.Append(name).Append(" - open wires\n");

        return 0;
    }

    private static void PrintProps(int i, ShapeAnalysis_FreeBoundData fbd, DrawResult di)
    {
        double area = fbd.Area();
        double perimeter = fbd.Perimeter();
        double ratio = fbd.Ratio();
        double width = fbd.Width();
        int notch = fbd.NbNotches();
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

        TopoDS_Shape source = theDI.GetShape(a[1]);
        if (source.IsNull())
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
        var analyzer = new ShapeAnalysis_FreeBoundsProperties();
        if (toler > 0)
        {
            analyzer.Init(source, toler, splitclosed, splitopen);
        }
        else
        {
            analyzer.Init(source, splitclosed, splitopen);
        }
        analyzer.Perform();
        var B = new BRep_Builder();

        int nb = analyzer.NbClosedFreeBounds();
        di.Append("\n");
        di.Append(" \tArea mm2\tLength mm\tRatio L/W\tWidth mm\tNb notch\n");
        var closed = new TopoDS_Compound();
        B.MakeCompound(ref closed);
        if (nb != 0)
        {
            di.Append("Closed bounds properties\n");
            for (int i = 1; i <= nb; i++)
            {
                ShapeAnalysis_FreeBoundData fbd = analyzer.ClosedFreeBound(i);
                PrintProps(i, fbd, di);
                B.Add(closed, fbd.FreeBound());
            }
        }

        nb = analyzer.NbOpenFreeBounds();
        var open = new TopoDS_Compound();
        B.MakeCompound(ref open);
        if (nb != 0)
        {
            di.Append("Open bounds properties\n");
            for (int i = 1; i <= nb; i++)
            {
                ShapeAnalysis_FreeBoundData fbd = analyzer.OpenFreeBound(i);
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
        TopoDS_Shape shape = theDI.GetShape(a[1]);
        if (shape.IsNull())
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

        ShapeFix_FreeBounds F;
        if (sewtoler <= 0)
        {
            F = new ShapeFix_FreeBounds(shape, closetoler, splitclosed, splitopen);
        }
        else
        {
            F = new ShapeFix_FreeBounds(shape, sewtoler, closetoler, splitclosed, splitopen);
        }

        TopoDS_Shape wires = F.GetClosedWires();
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
        TopoDS_Shape shape = theDI.GetShape(a[1]);
        if (shape.IsNull())
        {
            di.Append("Shape is not defined\n");
            return 1;
        }
        if (shape.ShapeType() != TopAbs_ShapeEnum.TopAbs_WIRE)
        {
            di.Append("invalid type of argument\n");
            return 1;
        }
        double anArea = ShapeAnalysis.ContourArea(TopoDS.Wire(shape));
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
        TopoDS_Shape wire = theDI.GetShape(argv[1]);
        if (wire.IsNull() || wire.ShapeType() != TopAbs_ShapeEnum.TopAbs_WIRE)
        {
            di.Append("A null shape or not a wire is used.\n");
            return 2;
        }

        // Get face if the user provided us with a face.
        TopoDS_Shape face = new();
        if (argc > 2)
        {
            face = theDI.GetShape(argv[2]);
            if (face.IsNull() || face.ShapeType() != TopAbs_ShapeEnum.TopAbs_FACE)
            {
                di.Append("A null shape or not a face is used.\n");
                return 3;
            }
        }

        // If the face is null, make a plane inside the wire.
        if (face.IsNull())
        {
            var mkFace = new BRepBuilderAPI_MakeFace(TopoDS.Wire(wire), true);
            if (mkFace.IsDone())
            {
                face = mkFace.Face();
            }
            else
            {
                di.Append("Can't make a face for the wire. Provide please a face for analysis.\n");
                return 4;
            }
        }

        var analyser = new ShapeAnalysis_Wire(TopoDS.Wire(wire), TopoDS.Face(face), Precision.Confusion());
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
        TopoDS_Shape edge = theDI.GetShape(argv[1]);
        if (edge.IsNull() || edge.ShapeType() != TopAbs_ShapeEnum.TopAbs_EDGE)
        {
            di.Append("A null shape or not an edge is used.\n");
            return 2;
        }

        // Get face.
        TopoDS_Shape face = new();
        if (argc == 3)
        {
            face = theDI.GetShape(argv[2]);
            if (face.IsNull() || face.ShapeType() != TopAbs_ShapeEnum.TopAbs_FACE)
            {
                di.Append("A null shape or not a face is used.\n");
                return 3;
            }
        }

        // Analysis of the edge.
        var analyser = new ShapeAnalysis_Edge();
        bool isOk = true;

        // Curve 3D.
        if (analyser.HasCurve3d(TopoDS.Edge(edge)))
        {
            // Check vertices.
            if (analyser.CheckVerticesWithCurve3d(TopoDS.Edge(edge)))
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

        if (!face.IsNull())
        {
            // Curve 2D.
            if (analyser.HasPCurve(TopoDS.Edge(edge), TopoDS.Face(face)))
            {
                // Check vertices.
                if (analyser.CheckVerticesWithPCurve(TopoDS.Edge(edge), TopoDS.Face(face)))
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
        TopoDS_Shape sh = theDI.GetShape(a[2]);
        if (sh.IsNull())
        {
            return 1;
        }
        TopAbs_ShapeEnum aShType = sh.ShapeType();
        if (aShType != TopAbs_ShapeEnum.TopAbs_SHELL && aShType != TopAbs_ShapeEnum.TopAbs_FACE
            && aShType != TopAbs_ShapeEnum.TopAbs_EDGE && aShType != TopAbs_ShapeEnum.TopAbs_WIRE)
        {
            di.Append("Wrong shape type, shape can be shell or face or edge or wire\n");
            return 1;
        }

        GeomAbs_SurfaceType[] aTargets =
            [GeomAbs_SurfaceType.GeomAbs_Plane, GeomAbs_SurfaceType.GeomAbs_Cylinder, GeomAbs_SurfaceType.GeomAbs_Cone, GeomAbs_SurfaceType.GeomAbs_Sphere];
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
        var aSampleSurf = new GeomAdaptor_Surface();
        if (n > 5 && (sh.ShapeType() == TopAbs_ShapeEnum.TopAbs_EDGE || sh.ShapeType() == TopAbs_ShapeEnum.TopAbs_WIRE))
        {
            Geom_Surface? aGSurf = DrawTrSurf.GetSurface(theDI, a[5]);
            if (aGSurf is null)
            {
                di.Append("Sample surface is null").Append("\n");
                return 1;
            }
            aSampleSurf.Load(aGSurf);
            GeomAbs_SurfaceType aSType = aSampleSurf.GetType();
            if (aSType != aTargets[isurf])
            {
                di.Append("Sample surface has wrong type").Append("\n");
                return 1;
            }
        }

        var aCanonRec = new ShapeAnalysis_CanonicalRecognition(sh);
        Geom_Surface? aRes = null;
        switch (aTargets[isurf])
        {
            case GeomAbs_SurfaceType.GeomAbs_Plane:
            {
                var aPln = new gp_Pln();
                if (aSampleSurf.GetType() == GeomAbs_SurfaceType.GeomAbs_Plane)
                {
                    aPln = aSampleSurf.Plane();
                }
                if (aCanonRec.IsPlane(tol, ref aPln))
                {
                    aRes = new Geom_Plane(aPln);
                }
                break;
            }
            case GeomAbs_SurfaceType.GeomAbs_Cylinder:
            {
                var aCyl = new gp_Cylinder();
                if (aSampleSurf.GetType() == GeomAbs_SurfaceType.GeomAbs_Cylinder)
                {
                    aCyl = aSampleSurf.Cylinder();
                }
                if (aCanonRec.IsCylinder(tol, ref aCyl))
                {
                    aRes = new Geom_CylindricalSurface(aCyl);
                }
                break;
            }
            case GeomAbs_SurfaceType.GeomAbs_Cone:
            {
                var aCon = new gp_Cone();
                if (aSampleSurf.GetType() == GeomAbs_SurfaceType.GeomAbs_Cone)
                {
                    aCon = aSampleSurf.Cone();
                }
                if (aCanonRec.IsCone(tol, ref aCon))
                {
                    aRes = new Geom_ConicalSurface(aCon);
                }
                break;
            }
            case GeomAbs_SurfaceType.GeomAbs_Sphere:
            {
                var aSph = new gp_Sphere();
                if (aSampleSurf.GetType() == GeomAbs_SurfaceType.GeomAbs_Sphere)
                {
                    aSph = aSampleSurf.Sphere();
                }
                if (aCanonRec.IsSphere(tol, ref aSph))
                {
                    aRes = new Geom_SphericalSurface(aSph);
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
        TopoDS_Shape sh = theDI.GetShape(a[2]);
        if (sh.IsNull())
        {
            return 1;
        }
        TopAbs_ShapeEnum aShType = sh.ShapeType();
        if (aShType != TopAbs_ShapeEnum.TopAbs_WIRE && aShType != TopAbs_ShapeEnum.TopAbs_EDGE)
        {
            di.Append("Wrong shape type, shape can be wire or an edge \n");
            return 1;
        }

        GeomAbs_CurveType[] aTargets = [GeomAbs_CurveType.GeomAbs_Line, GeomAbs_CurveType.GeomAbs_Circle, GeomAbs_CurveType.GeomAbs_Ellipse];
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

        double tol = Precision.Confusion();
        if (n > 4)
        {
            tol = theDI.Atof(a[4]);
        }

        var aCanonRec = new ShapeAnalysis_CanonicalRecognition(sh);
        Geom_Curve? aRes = null;
        switch (aTargets[icurv])
        {
            case GeomAbs_CurveType.GeomAbs_Line:
            {
                var aLin = new gp_Lin();
                if (aCanonRec.IsLine(tol, ref aLin))
                {
                    aRes = new Geom_Line(aLin);
                }
                break;
            }
            case GeomAbs_CurveType.GeomAbs_Circle:
            {
                var aCirc = new gp_Circ();
                if (aCanonRec.IsCircle(tol, ref aCirc))
                {
                    aRes = new Geom_Circle(aCirc);
                }
                break;
            }
            case GeomAbs_CurveType.GeomAbs_Ellipse:
            {
                var anElips = new gp_Elips();
                if (aCanonRec.IsEllipse(tol, ref anElips))
                {
                    aRes = new Geom_Ellipse(anElips);
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
