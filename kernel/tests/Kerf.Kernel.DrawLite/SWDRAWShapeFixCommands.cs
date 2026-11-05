// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// SWDRAW_ShapeFix.cxx's commands: <c>checkoverlapedges</c> (ShapeAnalysis_Edge::CheckOverlapping), which the
/// lowalgos/intss grid's CheckOverlapIntCurves runs on every pair of intersection lines; and, unit 6L,
/// <c>edgesameparam</c>, <c>settolerance</c>, <c>reface</c>, <c>fixshape</c>, <c>fixwgaps</c>, <c>fixsmall</c>,
/// <c>fixsmalledges</c>, <c>fixsmallfaces</c>, <c>checkfclass2d</c> and <c>connectedges</c>.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's healing tools write into the shape they are given (tolerances raised, pcurves added, vertices replaced), so
/// in DRAWEXE every variable holding the input's sub-shapes sees the change. Draw-lite runs them as
/// <c>unifysamedom</c> does (decision 495's in-place commands): on an unfrozen copy of the input
/// (<see cref="InPlace"/>), which the tool then changes in place as OCCT's does; the copy's sub-shapes left unchanged
/// are the input's again, and every variable sharing a changed one is rebound (DrawShapeRebind).
/// </para>
/// <para>
/// Left out: <c>stwire</c>, a French-language debugging command that no corpus case calls and whose vertex fixing
/// writes into the edges it explores.
/// </para>
/// </remarks>
internal static class SWDRAWShapeFixCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "checkoverlapedges", CheckOverlapEdges);
        DrawCxx.Add(theInterpreter, "edgesameparam", EdgeSameParam);
        DrawCxx.Add(theInterpreter, "settolerance", SetTolerance);
        DrawCxx.Add(theInterpreter, "reface", ReFace);
        DrawCxx.Add(theInterpreter, "fixshape", FixShape);
        DrawCxx.Add(theInterpreter, "fixwgaps", FixGaps);
        DrawCxx.Add(theInterpreter, "fixsmall", FixSmall);
        DrawCxx.Add(theInterpreter, "fixsmalledges", FixSmallEdges);
        DrawCxx.Add(theInterpreter, "fixsmallfaces", FixSmallFaces);
        DrawCxx.Add(theInterpreter, "checkfclass2d", CheckFClass2d);
        DrawCxx.Add(theInterpreter, "connectedges", ConnectEdges);
    }

    /// <summary>
    /// Runs an in-place healing tool as DRAWEXE does, on the Draw variable's shape: the tool runs on the shape itself in
    /// a working overlay (unit 10H6; until then on an unfrozen copy) and returns its result; every variable sharing a
    /// changed sub-shape is rebound, and the result (unfrozen, made of the input's shapes where unchanged) is returned.
    /// </summary>
    internal static Shape InPlace(DrawInterpreter theDI, in Shape theShape, Func<Shape, Shape> theTool)
    {
        using var anOverlay = WorkingOverlay.Begin(theShape);
        Shape aResult = theTool(theShape);
        aResult = anOverlay.Finish(aResult, [theShape]);
        DrawShapeRebind.Rebind(theDI, theShape, anOverlay.Final(theShape));
        return aResult;
    }

    private static int CheckOverlapEdges(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            di.Append("Invalid number of arguments\n");
            return 1;
        }
        Shape Sh1 = theDI.GetShape(a[1]);
        Shape Sh2 = theDI.GetShape(a[2]);
        if (Sh1.IsNull || Sh2.IsNull)
        {
            di.Append("Invalid arguments\n");
            return 1;
        }
        Edge e1 = ShapeCasts.Edge(Sh1);
        Edge e2 = ShapeCasts.Edge(Sh2);
        if (e1.IsNull || e2.IsNull)
        {
            di.Append("Invalid type of arguments\n");
            return 1;
        }

        if (Tool.Degenerated(e1))
        {
            di.Append(a[1]).Append(" is degenerated\n");
            return 1;
        }

        if (Tool.Degenerated(e2))
        {
            di.Append(a[2]).Append(" is degenerated\n");
            return 1;
        }

        double aTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        double aDistDomain = 0.0;
        int k = 3;
        if (k < n)
        {
            aTol = theDI.Atof(a[k++]);
        }
        if (k < n)
        {
            aDistDomain = theDI.Atof(a[k++]);
        }

        var sae = new EdgeAnalysis();
        if (sae.CheckOverlapping(e1, e2, ref aTol, aDistDomain))
        {
            if (aDistDomain == 0.0)
            {
                di.Append("Edges are overlapping completely\n");
            }
            else
            {
                di.Append("Edges are overlapped\n");
                di.Append("with tolerance = ").Append(aTol).Append("\n");
                di.Append("on segment length = ").Append(aDistDomain).Append("\n");
            }
        }
        else
        {
            di.Append("Edges are not overlapped\n");
        }
        return 0;
    }

    private static int EdgeSameParam(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        string? arg2 = argc > 2 ? argv[2] : null;
        //        ****    Edge:SameParameter         ****
        if (argc < 2)
        {
            di.Append("shapename ,  option f to force, else only Edges not-SameParameter are computed\n");
            return 1 /* Error */;
        }
        Shape Shape = theDI.GetShape(argv[1]);

        bool isDone = true;
        // OCCT works on the shape itself: Draw-lite on an unfrozen copy, rebound afterwards (see the class)
        InPlace(theDI, Shape, aWork =>
        {
            isDone = ShapeFixFixer.SameParameter(aWork, argc > 2 && arg2!.Length > 0 && arg2[0] == 'f', BRepBuilderApi.Precision());
            return aWork;
        });
        if (!isDone)
        {
            di.Append("Some edges were not processed\n");
        }
        di.Append("\n");
        return 0; // Done
    }

    private static int SetTolerance(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 3)
        {
            di.Append("myshape val   : forces tolerances at <val>\n")
              .Append("myshape < max : sets maximum tolerance to <max>\n")
              .Append("myshape > min : sets minimum tolerance to <min>\n")
              .Append("myshape min max : bounds tolerances between <min> and <max>\n")
              .Append("myshape mode=v-e-f other args : idem but only on vertex-edge-face\n");
            return argc < 2 ? 0 : 1 /* Error */;
        }
        string arg1 = argv[1];
        string arg2 = argv[2];
        Shape Shape = theDI.GetShape(arg1);
        if (Shape.IsNull)
        {
            di.Append("Shape unknown : ").Append(arg1).Append("\n");
            return 1 /* Error */;
        }
        char mod2 = arg2.Length > 0 ? arg2[0] : '\0';
        int premarg = 2;
        ShapeType styp = ShapeType.Shape;
        if (mod2 == 'v')
        {
            styp = ShapeType.Vertex;
            premarg = 3;
        }
        if (mod2 == 'e')
        {
            styp = ShapeType.Edge;
            premarg = 3;
        }
        if (mod2 == 'w')
        {
            styp = ShapeType.Wire;
            premarg = 3;
        }
        if (mod2 == 'f')
        {
            styp = ShapeType.Face;
            premarg = 3;
        }
        if (mod2 == 'a')
        {
            styp = ShapeType.Shape;
            premarg = 3;
        }

        // OCCT reads argv[premarg] even past the words (a null pointer); Draw-lite reads an empty word
        string aPremWord = premarg < argc ? argv[premarg] : "";
        double tmin, tmax;
        mod2 = aPremWord.Length > 0 ? aPremWord[0] : '\0';
        if (mod2 == '=')
        {
            tmin = tmax = theDI.Atof(argv[argc - 1]);
        }
        else if (mod2 == '<')
        {
            tmin = 0;
            tmax = theDI.Atof(argv[argc - 1]);
        }
        else if (mod2 == '>')
        {
            tmin = theDI.Atof(argv[argc - 1]);
            tmax = 0;
        }
        else
        {
            tmin = theDI.Atof(aPremWord);
            tmax = theDI.Atof(argv[argc - 1]);
        }

        if (argc == premarg + 1 || tmin == tmax)
        {
            di.Append("Setting Tolerance to ").Append(tmin).Append("\n");
        }
        else if (tmax < tmin)
        {
            di.Append("Minimum Tolerance to ").Append(tmin).Append("\n");
        }
        else if (tmin <= 0)
        {
            di.Append("Maximum Tolerance to ").Append(tmax).Append("\n");
        }
        else
        {
            di.Append("Tolerance Limited between ").Append(tmin).Append(" and ").Append(tmax).Append("\n");
        }
        var sat = new ShapeToleranceFixer();
        // OCCT works on the shape itself: Draw-lite on an unfrozen copy, rebound afterwards (see the class)
        InPlace(theDI, Shape, aWork =>
        {
            sat.LimitTolerance(aWork, tmin, tmax, styp);
            return aWork;
        });
        return 0; // Done
    }

    private static int ReFace(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 3)
        {
            di.Append("Donner un nom de SHAPE (SHELL ou FACE) + un nom de RESULTAT\n");
            return 1 /* Error */;
        }
        string arg1 = argv[1];
        string arg2 = argv[2];
        Shape Shape = theDI.GetShape(arg1);
        if (Shape.IsNull)
        {
            di.Append("Shape unknown : ").Append(arg1).Append("\n");
            return 1 /* Error */;
        }

        bool rebuild = false;

        var STF = new FaceFixer();
        //  Options ?
        for (int i = 3; i < argc; i++)
        {
            bool valopt = true;
            char opt = argv[i].Length > 0 ? argv[i][0] : '\0';
            if (opt == '+')
            {
                opt = argv[i].Length > 1 ? argv[i][1] : '\0';
            }
            if (opt == '-')
            {
                opt = argv[i].Length > 1 ? argv[i][1] : '\0';
                valopt = false;
            }
            if (!valopt)
            {
                di.Append(".. NO");
            }
            else
            {
                di.Append("..");
            }

            if (opt == 'R')
            {
                di.Append(" REBUILD-ANYWAY ..");
                rebuild = valopt;
            }
            if (opt == 'd')
            {
                di.Append(" fix-dgnr ..");
                STF.FixWireTool().FixDegeneratedMode() = valopt ? 1 : 0;
            }
            if (opt == 'r')
            {
                di.Append(" fix-reorder-wire ..");
                STF.FixWireTool().FixReorderMode() = valopt ? 1 : 0;
            }
        }

        var resh = new ReShapeBuilder();

        int nbfc = 0;
        for (var EF = new Explorer(Shape, ShapeType.Face); EF.More(); EF.Next())
        {
            Face F = ShapeCasts.Face(EF.Current());
            // ShapeFix_Face copies a frozen face at the entry (decision 583): the variable's face is unchanged
            STF.Init(F); // qui fait tout
            STF.Perform();
            Face face = STF.Face();
            bool newface = STF.Status(ExtendStatus.Done) || rebuild;
            if (newface)
            {
                nbfc++;
                resh.Replace(F, face);
            }
        }
        if (nbfc > 0)
        {
            di.Append("Faces reprises par ShapeFix_Face : ").Append(nbfc).Append("\n");
            theDI.SetShape(arg2, resh.Apply(Shape, ShapeType.Face, 2));
            return 0; // Done
        }
        else
        {
            di.Append("ShapeFix_Face n a rien trouve a redire\n");
        }
        return 0;
    }

    private static int FixShape(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        var msg = new MsgRegistrator();
        var sfs = new ShapeFixer();
        sfs.SetMsgRegistrator(msg);

        string? res = null;
        Shape initShape = new();
        double? aPrecision = null, aMaxTolerance = null;
        int par = 0, mess = 0;
        for (int i = 1; i < argc; i++)
        {
            if (argv[i].Length == 2 && (argv[i][0] == '-' || argv[i][0] == '+' || argv[i][0] == '*'))
            {
                int val = argv[i][0] == '-' ? 0 : argv[i][0] == '+' ? 1 : -1;
                switch (argv[i][1])
                {
                    case 'l':
                        sfs.FixWireTool().FixLackingMode() = val;
                        break;
                    case 'o':
                        sfs.FixFaceTool().FixOrientationMode() = val;
                        break;
                    case 'h':
                        sfs.FixWireTool().FixShiftedMode() = val;
                        break;
                    case 'm':
                        sfs.FixFaceTool().FixMissingSeamMode() = val;
                        break;
                    case 'd':
                        sfs.FixWireTool().FixDegeneratedMode() = val;
                        break;
                    case 's':
                        sfs.FixWireTool().FixSmallMode() = val;
                        break;
                    case 'i':
                        sfs.FixWireTool().FixSelfIntersectionMode() = val;
                        break;
                    case 'n':
                        sfs.FixWireTool().FixNotchedEdgesMode() = val;
                        break;
                    case '?':
                        mess = val;
                        break;
                }
                continue;
            }
            else if (argv[i] == "-maxtaila")
            {
                if (++i >= argc)
                {
                    break;
                }

                sfs.FixWireTool().SetMaxTailAngle(theDI.Atof(argv[i]) * (Math.PI / 180));
            }
            else if (argv[i] == "-maxtailw")
            {
                if (++i >= argc)
                {
                    break;
                }

                sfs.FixWireTool().SetMaxTailWidth(theDI.Atof(argv[i]));
                sfs.FixWireTool().FixTailMode() = 1;
            }
            else
            {
                switch (par)
                {
                    case 0:
                        res = argv[i];
                        break;
                    case 1:
                    {
                        Shape aShape = theDI.GetShape(argv[i]);
                        if (aShape.IsNull)
                        {
                            continue;
                        }
                        initShape = aShape;
                    }
                    break;
                    case 2:
                        // OCCT sets these on the tool as they come; Draw-lite after its Init on the copy, which
                        // changes neither
                        aPrecision = theDI.Atof(argv[i]);
                        break;
                    case 3:
                        aMaxTolerance = theDI.Atof(argv[i]);
                        break;
                }
            }
            par++;
        }

        if (par < 2)
        {
            di.Append("Use: ").Append(argv[0]).Append(" result shape [tolerance [max_tolerance]] [switches]\n"
                                                     + "[-maxtaila <degrees>] [-maxtailw <width>]\n");
            di.Append("Switches allow to tune parameters of ShapeFix\n");
            di.Append("The following syntax is used: <symbol><parameter>\n");
            di.Append("- symbol may be - to set parameter off, + to set on or * to set default\n");
            di.Append("- parameters are identified by letters:\n");
            di.Append("  l - FixLackingMode\n");
            di.Append("  o - FixOrientationMode\n");
            di.Append("  h - FixShiftedMode\n");
            di.Append("  m - FixMissingSeamMode\n");
            di.Append("  d - FixDegeneratedMode\n");
            di.Append("  s - FixSmallMode\n");
            di.Append("  i - FixSelfIntersectionMode\n");
            di.Append("  n - FixNotchedEdgesMode\n");
            di.Append("For enhanced message output, use switch '+?'\n");
            return 1;
        }

        if (aPrecision is double aPreci)
        {
            sfs.SetPrecision(aPreci);
        }
        if (aMaxTolerance is double aMaxTol)
        {
            sfs.SetMaxTolerance(aMaxTol);
        }

        Shape aResult;
        if (initShape.IsNull)
        {
            // OCCT performs on the empty tool and sets a null shape
            sfs.Perform();
            aResult = sfs.InternalShape();
        }
        else
        {
            // OCCT works on the shape itself: Draw-lite on an unfrozen copy, rebound afterwards (see the class)
            aResult = InPlace(theDI, initShape, aWork =>
            {
                sfs.Init(aWork);
                sfs.Perform();
                return sfs.InternalShape();
            });
        }
        theDI.SetShape(res!, aResult);

        if (mess != 0)
        {
            // OCCT counts in an NCollection_DataMap keyed by the message text, which it iterates in its hash order;
            // Draw-lite lists the messages in the order they first come
            var aMapOfNumberOfFixes = new Dictionary<string, int>(StringComparer.Ordinal);
            var aOrder = new List<string>();
            var aBuilder = new BrepBuilder();
            var aCompound = new Compound();
            aBuilder.MakeCompound(ref aCompound);
            // Counting the number of each type of fixes. If the switch '*?' store all modified shapes in compound.
            foreach (var it in msg.MapShape())
            {
                foreach (global::Kerf.Engine.Foundation.Msg iter in it.Value)
                {
                    string aKey = iter.Value();
                    if (aMapOfNumberOfFixes.TryGetValue(aKey, out int aCount))
                    {
                        aMapOfNumberOfFixes[aKey] = aCount + 1;
                    }
                    else
                    {
                        aMapOfNumberOfFixes[aKey] = 1;
                        aOrder.Add(aKey);
                    }
                }
                if (mess < 0)
                {
                    aBuilder.Add(aCompound, it.Key);
                }
            }

            var aSStream = new System.Text.StringBuilder();
            aSStream.Append(" Fix").Append("Count\n".PadLeft(58)).Append(" ------------------------------------------------------------\n");
            foreach (string aKey in aOrder)
            {
                string aCount = aMapOfNumberOfFixes[aKey].ToString(System.Globalization.CultureInfo.InvariantCulture);
                aSStream.Append(' ').Append(aKey).Append(aCount.PadLeft(Math.Max(0, 60 - aKey.Length))).Append('\n');
            }
            aSStream.Append(" ------------------------------------------------------------\n");
            di.Append(aSStream.ToString());

            if (mess < 0)
            {
                string buff = res + "_m";
                di.Append(" Modified shapes saved in compound: ").Append(buff);
                theDI.SetShape(buff, aCompound);
            }
        }

        return 0; // Done
    }

    private static int FixGaps(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        Shape S = theDI.GetShape(a[2]);
        if (S.IsNull)
        {
            di.Append(" Shape is null\n");
            return 1;
        }

        double prec = n > 3 ? theDI.Atof(a[3]) : 0.0;
        bool isFixed = false;
        // OCCT works on the shape itself: Draw-lite on an unfrozen copy, rebound afterwards (see the class)
        Shape aResult = InPlace(theDI, S, aWork =>
        {
            var SFWF = new WireframeFixer(aWork);
            SFWF.SetPrecision(prec);
            isFixed = SFWF.FixWireGaps();
            return SFWF.InternalShape();
        });
        if (isFixed)
        {
            theDI.SetShape(a[1], aResult);
            di.Append(" Wireframe gaps fixed on shape\n");
        }

        return 0;
    }

    private static int FixSmall(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            return 1;
        }

        Shape S = theDI.GetShape(a[2]);
        if (S.IsNull)
        {
            di.Append(" Shape is null\n");
            return 1;
        }

        double prec = (n == 4) ? theDI.Atof(a[3]) : 1.0;
        bool isFixed = false;
        // OCCT works on the shape itself: Draw-lite on an unfrozen copy, rebound afterwards (see the class)
        Shape aResult = InPlace(theDI, S, aWork =>
        {
            var SFWF = new WireframeFixer(aWork);
            SFWF.SetPrecision(prec);
            isFixed = SFWF.FixSmallEdges();
            return SFWF.InternalShape();
        });

        if (isFixed)
        {
            theDI.SetShape(a[1], aResult);
            di.Append(" Small edges fixed on shape\n");
        }

        return 0;
    }

    private static int FixSmallEdges(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            di.Append("Invalid number of arguments\n");
            return 1;
        }
        Shape Sh = theDI.GetShape(a[2]);

        int k = 3;
        double tol = 100000;
        int mode = 2;
        double tolang = Math.PI / 2;
        if (n > k)
        {
            tol = theDI.Atof(a[k++]);
        }

        if (n > k)
        {
            mode = theDI.Atoi(a[k++]);
        }

        if (n > k)
        {
            tolang = theDI.Atof(a[k++]);
        }

        bool aModeDrop = mode != 2;

        // OCCT works on the shape itself: Draw-lite on an unfrozen copy, rebound afterwards (see the class)
        Shape resShape = InPlace(theDI, Sh, aWork =>
        {
            var aSfwr = new WireframeFixer();
            var aReShape = new ReShapeBuilder();
            aSfwr.SetContext(aReShape);
            aSfwr.Load(aWork);
            aSfwr.SetPrecision(tol);

            var theSmallEdges = new HashSet<Shape>(ShapeHasher.Instance);
            var theMultyEdges = new HashSet<Shape>(ShapeHasher.Instance);
            var theEdgeToFaces = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            var theFaceWithSmall = new Dictionary<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
            aSfwr.CheckSmallEdges(theSmallEdges, theEdgeToFaces, theFaceWithSmall, theMultyEdges);
            aSfwr.MergeSmallEdges(theSmallEdges, theEdgeToFaces, theFaceWithSmall, theMultyEdges, aModeDrop, tolang);
            return aSfwr.InternalShape();
        });
        theDI.SetShape(a[1], resShape);
        return 0;
    }

    private static int FixSmallFaces(DrawInterpreter theDI, DrawResult theDi, IReadOnlyList<string> theArgv)
    {
        int theArgc = theArgv.Count;
        if (theArgc < 3)
        {
            theDi.Append("Use: ").Append(theArgv[0]).Append(" result shape [tolerance]");
            return 1;
        }

        Shape aShape = theDI.GetShape(theArgv[2]);
        if (aShape.IsNull)
        {
            theDi.Append("Error: Shape ").Append(theArgv[2]).Append(" is null\n");
            return 1;
        }

        double aPrec = theArgc < 4 ? 1.0 : theDI.Atof(theArgv[3]);

        bool isRemoved = false;
        // OCCT works on the shape itself: Draw-lite on an unfrozen copy, rebound afterwards (see the class)
        Shape aResult = InPlace(theDI, aShape, aWork =>
        {
            var aFixSmallFaces = new FixSmallFaceFixer();
            aFixSmallFaces.Init(aWork);
            aFixSmallFaces.SetPrecision(aPrec);
            aFixSmallFaces.Perform();
            isRemoved = !aFixSmallFaces.InternalShape().IsSame(aWork);
            return isRemoved ? aFixSmallFaces.InternalShape() : aWork;
        });
        if (isRemoved)
        {
            theDi.Append("Small faces are removed");
        }

        theDI.SetShape(theArgv[1], aResult);

        return 0;
    }

    private static int CheckFClass2d(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            di.Append("Invalid number of arguments\n");
            return 1;
        }
        Shape Sh1 = theDI.GetShape(a[1]);
        double ucoord = theDI.Atof(a[2]);
        double vcoord = theDI.Atof(a[3]);
        if (Sh1.IsNull || Sh1.Type != ShapeType.Face)
        {
            di.Append("Invalid arguments\n");
            return 1;
        }
        double tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (n > 4)
        {
            // OCCT calls C's atof here, which reads a plain number only
            tol = double.TryParse(a[4], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture,
                                  out double aTol) ? aTol : 0.0;
        }

        Face aFace = ShapeCasts.Face(Sh1);
        var p2d = new Point2D(ucoord, vcoord);
        var f2d = new FClass2d(aFace, tol);
        Containment stat = f2d.Perform(p2d);
        if (stat == Containment.Out)
        {
            di.Append("Point is OUT\n");
        }
        else if (stat == Containment.In)
        {
            di.Append("Point is IN\n");
        }
        else if (stat == Containment.On)
        {
            di.Append("Point is ON\n");
        }
        else
        {
            di.Append("Point is UNKNOWN\n");
        }
        return 0;
    }

    private static int ConnectEdges(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            di.Append("Invalid number of arguments. Should be : result shape [toler shared]\n");
            return 1;
        }
        Shape aSh1 = theDI.GetShape(a[2]);
        if (aSh1.IsNull)
        {
            di.Append("Shape is null\n");
            return 1;
        }
        double aTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        if (n > 3)
        {
            aTol = theDI.Atof(a[3]);
        }

        bool shared = true;
        if (n > 4)
        {
            shared = theDI.Atoi(a[4]) == 1;
        }
        var aExpE = new Explorer(aSh1, ShapeType.Edge);
        var aSeqEdges = new NCollection_HSequence<Shape>();
        var aMapEdges = new IndexedSet<Shape>(ShapeHasher.Instance);
        for (; aExpE.More(); aExpE.Next())
        {
            aSeqEdges.Append(aExpE.Current());
            aMapEdges.Add(aExpE.Current());
        }

        NCollection_HSequence<Shape> aSeqWires = FreeBounds.ConnectEdgesToWires(aSeqEdges, aTol, shared);
        var aB = new BrepBuilder();
        var aComp = new Compound();
        aB.MakeCompound(ref aComp);
        int i = 1;
        for (; i <= aSeqWires.Length(); i++)
        {
            Shape aW = aSeqWires.Value(i);
            di.Append("Wire - ").Append(i).Append(" : \n");

            var aExp1 = new Explorer(aW, ShapeType.Edge);
            for (; aExp1.More(); aExp1.Next())
            {
                if (shared)
                {
                    int ind = aMapEdges.IndexOf(aExp1.Current()) + 1;
                    di.Append(ind).Append(" ");
                }
                else
                {
                    Exploration.Vertices(ShapeCasts.Edge(aExp1.Current()), out Vertex aV1, out Vertex _);
                    Point3D aP = Tool.Pnt(aV1);
                    di.Append(aP.X).Append(" ").Append(aP.Y).Append(" ").Append(aP.Z).Append("\n");
                }
            }

            di.Append("\n");
            aB.Add(aComp, aSeqWires.Value(i));
        }
        theDI.SetShape(a[1], aComp);
        return 0;
    }
}
