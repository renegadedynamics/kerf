// Port of: src/Draw/TKTopTest/SWDRAW/SWDRAW_ShapeUpgrade.cxx (the divide and convert commands, unit 7C)
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Custom;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace OcctNet.DrawLite;

internal static partial class SWDrawShapeUpgradeCommands
{
    private static readonly (ExtendStatus Status, string Name)[] theStatuses =
    [
        (ExtendStatus.OK, "OK"),
        (ExtendStatus.DONE1, "DONE1"), (ExtendStatus.DONE2, "DONE2"), (ExtendStatus.DONE3, "DONE3"), (ExtendStatus.DONE4, "DONE4"),
        (ExtendStatus.DONE5, "DONE5"), (ExtendStatus.DONE6, "DONE6"), (ExtendStatus.DONE7, "DONE7"), (ExtendStatus.DONE8, "DONE8"),
        (ExtendStatus.FAIL1, "FAIL1"), (ExtendStatus.FAIL2, "FAIL2"), (ExtendStatus.FAIL3, "FAIL3"), (ExtendStatus.FAIL4, "FAIL4"),
        (ExtendStatus.FAIL5, "FAIL5"), (ExtendStatus.FAIL6, "FAIL6"), (ExtendStatus.FAIL7, "FAIL7"), (ExtendStatus.FAIL8, "FAIL8"),
    ];

    // the "if (tool.Status(ShapeExtend_XXX)) di << "Status: XXX\n";" blocks of OCCT's commands
    private static void PrintStatuses(DrawResult di, Func<ExtendStatus, bool> theStatus)
    {
        foreach ((ExtendStatus aStatus, string aName) in theStatuses)
        {
            if (theStatus(aStatus))
            {
                di.Append("Status: ").Append(aName).Append("\n");
            }
        }
    }

    /// <summary>
    /// <c>ShapeFix::SameParameter(res, false)</c> as DRAWEXE runs it after the divide and convert tools: in place on
    /// the result, which shares the input's sub-shapes (see the class). Returns the result to bind.
    /// </summary>
    internal static Shape SameParameterInPlace(DrawInterpreter theDI, in Shape theResult)
    {
        if (theResult.IsNull)
        {
            return theResult;
        }
        return SWDRAWShapeFixCommands.InPlace(theDI, theResult, aWork =>
        {
            ShapeFixFixer.SameParameter(aWork, false);
            return aWork;
        });
    }

    private static int DT_ShapeDivide(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        // DT_ShapeDivide result Shape Tol
        int n = a.Count;
        if (n < 3)
        {
            di.Append("bad number of arguments\n");
            return 1;
        }

        // try to read a shape:
        Shape inputShape = theDI.GetShape(a[2]);
        if (inputShape.IsNull)
        {
            di.Append("Unknown shape\n");
            return 1;
        }

        // OCCT works on the input itself: Draw-lite on an unfrozen copy, rebound afterwards (see the class)
        Shape res = SWDRAWShapeFixCommands.InPlace(theDI, inputShape, aWork =>
        {
            var tool = new ShapeDivideContinuity(aWork);

            // tolerance is optional
            if (n == 4)
            {
                double Tol = theDI.Atof(a[3]);
                tool.SetTolerance(Tol);
            }

            tool.Perform();
            PrintStatuses(di, tool.Status);
            return tool.InternalResult();
        });

        // fixes
        res = SameParameterInPlace(theDI, res);

        theDI.SetShape(a[1], res);
        return 0;
    }

    // DT_ShapeConvert and DT_ShapeConvertRev: ShapeConvertToBezier on the shape, with the conversions to set
    private static Shape ConvertToBezier(DrawInterpreter theDI, DrawResult di, in Shape theShape,
                                                Action<ShapeConvertToBezier> theSetUp)
    {
        return SWDRAWShapeFixCommands.InPlace(theDI, theShape, aWork =>
        {
            var tool = new ShapeConvertToBezier(aWork);
            theSetUp(tool);
            tool.Perform();
            PrintStatuses(di, tool.Status);
            return tool.InternalResult();
        });
    }

    private static int DT_ShapeConvertRev(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 5)
        {
            di.Append("bad number of arguments\n");
            return 1;
        }

        // try to read a shape:
        Shape inputShape = theDI.GetShape(a[2]);
        if (inputShape.IsNull)
        {
            di.Append("Unknown shape\n");
            return 1;
        }

        int c2d = theDI.Atoi(a[3]);
        int c3d = theDI.Atoi(a[4]);
        Shape revsh = Customize.ConvertToRevolution(inputShape);
        if (revsh.IsNull)
        {
            di.Append("NO RESULT\n");
            return 1;
        }
        else if (revsh == inputShape)
        {
            di.Append("No modif\n");
        }
        else
        {
            di.Append("ConvertToRevolution -> Result : \n");
        }

        Shape res = ConvertToBezier(theDI, di, revsh, tool =>
        {
            tool.SetSurfaceConversion(true);
            if (c2d != 0)
            {
                tool.Set2dConversion(true);
            }
            if (c3d != 0)
            {
                tool.Set3dConversion(true);
                if (n > 5)
                {
                    tool.Set3dLineConversion(false);
                }
                if (n > 6)
                {
                    tool.Set3dCircleConversion(false);
                }
                if (n > 7)
                {
                    tool.Set3dConicConversion(false);
                }
            }
        });

        // fixes
        res = SameParameterInPlace(theDI, res);

        theDI.SetShape(a[1], res);
        return 0;
    }

    private static int DT_ShapeConvert(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 5)
        {
            di.Append("bad number of arguments\n");
            return 1;
        }

        // try to read a shape:
        Shape inputShape = theDI.GetShape(a[2]);
        if (inputShape.IsNull)
        {
            di.Append("Unknown shape\n");
            return 1;
        }

        int c2d = theDI.Atoi(a[3]);
        int c3d = theDI.Atoi(a[4]);

        Shape res = ConvertToBezier(theDI, di, inputShape, tool =>
        {
            tool.SetSurfaceConversion(true);
            if (c2d != 0)
            {
                tool.Set2dConversion(true);
            }
            if (c3d != 0)
            {
                tool.Set3dConversion(true);
            }
        });

        // fixes
        res = SameParameterInPlace(theDI, res);

        theDI.SetShape(a[1], res);
        return 0;
    }

    private static int DT_SplitAngle(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            di.Append("bad number of arguments\n");
            return 1;
        }

        Shape inputShape = theDI.GetShape(a[2]);
        if (inputShape.IsNull)
        {
            di.Append("Unknown shape\n");
            return 1;
        }

        double maxangle = 95;
        if (n > 3)
        {
            maxangle = theDI.Atof(a[3]);
            if (maxangle < 1)
            {
                maxangle = 1;
            }
        }

        Shape res = SWDRAWShapeFixCommands.InPlace(theDI, inputShape, aWork =>
        {
            var tool = new ShapeDivideAngle(maxangle * Math.PI / 180, aWork);
            tool.Perform();
            PrintStatuses(di, tool.Status);
            return tool.InternalResult();
        });

        // fixes
        res = SameParameterInPlace(theDI, res);

        theDI.SetShape(a[1], res);
        return 0;
    }

    /// <summary><c>DT_SplitCurve Curve Tol Split</c>: splits the curve with C1 criterion; binds Curve_1, Curve_2...</summary>
    private static int DT_SplitCurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        // a[1]= input curve. This name is used with a suffix to name the output curves
        // a[2]= Tolerance
        int n = a.Count;
        if (n < 3)
        {
            di.Append("bad number of arguments\n");
            return 1;
        }

        double Tol = theDI.Atof(a[2]);
        Curve3D? GC = DrawTrSurf.GetCurve(theDI, a[1]);
        if (GC is null)
        {
            return 1;
        }
        // OCCT reads a[3] even when it isn't given (argv[argc] is null); Draw-lite takes no split values then
        int Split = n > 3 ? theDI.Atoi(a[3]) : 0;
        var theTool = new SplitCurve3dContinuity();
        theTool.Init(GC);
        theTool.SetTolerance(Tol);
        theTool.SetCriterion(Continuity.C1);
        if (Split == 1)
        {
            var spval = new NCollection_HSequence<double>();
            for (int i = 1; i <= 5; i++)
            {
                spval.Append(i);
            }
            theTool.SetSplitValues(spval);
        }
        theTool.Perform(true);
        NCollection_HArray1<Curve3D> theCurves = theTool.GetCurves()!;
        int NbC = theCurves.Length();
        for (int icurv = 1; icurv <= NbC; icurv++)
        {
            string newname = a[1] + "_" + icurv;
            DrawTrSurf.Set(theDI, newname, theCurves.Value(theCurves.Lower() + icurv - 1));
            di.AppendElement(newname);
        }
        return 0;
    }

    /// <summary><c>DT_SplitCurve2d Curve Tol Split</c>: splits the 2D curve with C1 criterion.</summary>
    private static int DT_SplitCurve2d(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 3)
        {
            di.Append("bad number of arguments\n");
            return 1;
        }

        double Tol = theDI.Atof(a[2]);
        Curve2D? GC = DrawTrSurf.GetCurve2d(theDI, a[1]);
        if (GC is null)
        {
            return 1;
        }
        // OCCT reads a[3] even when it isn't given (argv[argc] is null); Draw-lite takes no split values then
        int Split = n > 3 ? theDI.Atoi(a[3]) : 0;
        var theTool = new SplitCurve2dContinuity();
        theTool.Init(GC);
        theTool.SetTolerance(Tol);
        theTool.SetCriterion(Continuity.C1);
        if (Split == 1)
        {
            var spval = new NCollection_HSequence<double>();
            for (int i = 1; i <= 5; i++)
            {
                spval.Append(i);
            }
            theTool.SetSplitValues(spval);
        }
        theTool.Perform(true);
        NCollection_HArray1<Curve2D> theCurves = theTool.GetCurves()!;
        int NbC = theCurves.Length();
        for (int icurv = 1; icurv <= NbC; icurv++)
        {
            string newname = a[1] + "_" + icurv;
            DrawTrSurf.Set(theDI, newname, theCurves.Value(theCurves.Lower() + icurv - 1));
            di.AppendElement(newname);
        }
        return 0;
    }

    /// <summary><c>DT_SplitSurface Result Surface Tol Split</c>: splits the surface with C1 criterion.</summary>
    private static int DT_SplitSurface(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            di.Append("bad number of arguments\n");
            return 1;
        }

        var theTool = new SplitSurfaceContinuity(); // S4137

        double Tol = theDI.Atof(a[3]);
        // OCCT reads a[4] even when it isn't given (argv[argc] is null); Draw-lite takes no split values then
        int Split = n > 4 ? theDI.Atoi(a[4]) : 0;
        theTool.SetTolerance(Tol);
        theTool.SetCriterion(Continuity.C1);
        Surface? GS = DrawTrSurf.GetSurface(theDI, a[2]);
        // Case of single surface
        di.Append("single surf\n");

        di.Append("appel a SplitSurface::Init\n");
        theTool.Init(GS!);
        if (Split == 1)
        {
            var spval = new NCollection_HSequence<double>();
            for (int i = 1; i <= 5; i++)
            {
                spval.Append(i);
            }
            theTool.SetUSplitValues(spval);
            theTool.SetVSplitValues(spval);
        }

        di.Append("appel a SplitSurface::Build\n");
        theTool.Build(true);

        di.Append("appel a SplitSurface::GlobalU/VKnots\n");
        CompositeSurface Grid = theTool.ResSurfaces()!;
        NCollection_HArray1<double> GlobalU = Grid.UJointValues()!;
        NCollection_HArray1<double> GlobalV = Grid.VJointValues()!;
        int nbGlU = GlobalU.Length();
        int nbGlV = GlobalV.Length();
        di.Append("nb GlobalU ; nb GlobalV=").Append(nbGlU).Append(" ").Append(nbGlV);
        for (int iu = 1; iu <= nbGlU; iu++)
        {
            di.Append(" ").Append(GlobalU.Value(iu));
        }
        for (int iv = 1; iv <= nbGlV; iv++)
        {
            di.Append(" ").Append(GlobalV.Value(iv));
        }
        di.Append("\n");

        di.Append("appel a Surfaces\n");
        NCollection_HArray2<Surface> theSurfaces = Grid.Patches()!;

        di.Append("transfert resultat\n");
        int NbRow = theSurfaces.ColLength();
        int NbCol = theSurfaces.RowLength();
        for (int irow = 1; irow <= NbRow; irow++)
        {
            for (int icol = 1; icol <= NbCol; icol++)
            {
                string newname = a[1] + "_" + irow + "_" + icol;
                DrawTrSurf.Set(theDI, newname, theSurfaces.Value(irow, icol));
                di.AppendElement(newname);
            }
        }
        return 0;
    }

    private static int offset2dcurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 4)
        {
            di.Append("result + curve + offset\n");

            return 1 /* Error */;
        }
        double Offset = theDI.Atof(argv[3]);
        Curve2D? GC = DrawTrSurf.GetCurve2d(theDI, argv[2]);
        if (GC is null)
        {
            return 1;
        }
        var offcrv = new OffsetCurve2D(GC, Offset);
        DrawTrSurf.Set(theDI, argv[1], offcrv);
        return 0;
    }

    private static int offsetcurve(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 5)
        {
            di.Append("result + curve + offset + Dir\n");

            return 1 /* Error */;
        }
        double Offset = theDI.Atof(argv[3]);
        Curve3D? GC = DrawTrSurf.GetCurve(theDI, argv[2]);
        if (GC is null)
        {
            return 1;
        }
        var point = new Point3D();
        DrawTrSurf.GetPoint(theDI, argv[4], ref point);
        var dir = new Direction3D(point.XYZ());
        var offcrv = new OffsetCurve3D(GC, Offset, dir);
        DrawTrSurf.Set(theDI, argv[1], offcrv);
        return 0;
    }

    private static int splitface(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 5)
        {
            di.Append("Split face: splitface result face [u usplit1 usplit2...] [v vsplit1 vsplit2 ...]\n");
            return 1;
        }

        Shape aLocalShape = theDI.GetShape(argv[2]);
        Face face = ShapeCasts.Face(aLocalShape);
        if (face.IsNull)
        {
            di.Append(argv[2]).Append(" is not Face\n");
            return 1;
        }

        Surface S = Tool.Surface(face)!;
        BrepTools.UVBounds(face, out double Uf, out double Ul, out double Vf, out double Vl);
        S.Bounds(out double Umin, out double Umax, out double Vmin, out double Vmax);
        if (Uf < Umin && !S.IsUPeriodic)
        {
            Uf = Umin;
        }
        else if (Uf > Umin)
        {
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(Umin))
            {
                Uf -= 100;
            }
            else
            {
                Uf = Umin;
            }
        }
        if (Vf < Vmin && !S.IsVPeriodic)
        {
            Vf = Vmin;
        }
        else if (Vf > Vmin)
        {
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(Vmin))
            {
                Vf -= 100;
            }
            else
            {
                Vf = Vmin;
            }
        }
        if (Ul > Umax && !S.IsUPeriodic)
        {
            Ul = Umax;
        }
        else if (Ul < Umax)
        {
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(Umax))
            {
                Ul += 100;
            }
            else
            {
                Ul = Umax;
            }
        }
        if (Vl > Vmax && !S.IsVPeriodic)
        {
            Vl = Vmax;
        }
        else if (Vl < Vmax)
        {
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(Vmax))
            {
                Vl += 100;
            }
            else
            {
                Vl = Vmax;
            }
        }

        var uval = new NCollection_Sequence<double>();
        var vval = new NCollection_Sequence<double>();

        bool byV = false;
        int i;
        for (i = 3; i < argc; i++)
        {
            if (argv[i].Length > 0 && argv[i][0] == 'u')
            {
                byV = false;
            }
            else if (argv[i].Length > 0 && argv[i][0] == 'v')
            {
                byV = true;
            }
            else
            {
                double val = theDI.Atof(argv[i]);
                NCollection_Sequence<double> vals = (byV ? vval : uval);
                if (vals.Length() > 0 && val - vals.Last() < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    di.Append("Values should be sorted in increasing order; skipped\n");
                    continue;
                }
                if ((byV && (val < Vf + global::Kerf.Engine.Foundation.Precision.PConfusion() || val > Vl - global::Kerf.Engine.Foundation.Precision.PConfusion()))
                    || (!byV && (val < Uf + global::Kerf.Engine.Foundation.Precision.PConfusion() || val > Ul - global::Kerf.Engine.Foundation.Precision.PConfusion())))
                {
                    di.Append("Values should be inside range of surface; skipped\n");
                    continue;
                }
                vals.Append(val);
            }
        }
        if (uval.Length() < 1 && vval.Length() < 1)
        {
            di.Append("No splitting defined\n");
            return 1;
        }
        if (uval.Length() > 0)
        {
            di.Append("Splitting by U: ");
            for (int j = 1; j <= uval.Length(); j++)
            {
                // OCCT: compares i (the argument count by now) with j, so every value gets a separator before it
                if (i > j)
                {
                    di.Append(", ");
                }
                else
                {
                    di.Append("");
                }
                di.Append(uval[j]);
            }
            di.Append("\n");
        }
        if (vval.Length() > 0)
        {
            di.Append("Splitting by V: ");
            for (int j = 1; j <= vval.Length(); j++)
            {
                if (j > 1)
                {
                    di.Append(", ");
                }
                else
                {
                    di.Append("");
                }
                di.Append(vval[j]);
            }
            di.Append("\n");
        }

        var AS = new NCollection_HArray2<Surface>(1, uval.Length() + 1, 1, vval.Length() + 1);
        for (i = 0; i <= uval.Length(); i++)
        {
            double umin = (i != 0 ? uval[i] : Uf);
            double umax = (i < uval.Length() ? uval[i + 1] : Ul);
            for (int j = 0; j <= vval.Length(); j++)
            {
                double vmin = (j != 0 ? vval[j] : Vf);
                double vmax = (j < vval.Length() ? vval[j + 1] : Vl);
                var rect = new TrimmedSurface(S, umin, umax, vmin, vmax);
                AS.SetValue(i + 1, j + 1, rect);
            }
        }

        var Grid = new CompositeSurface();
        if (!Grid.Init(AS))
        {
            di.Append("Grid badly connected!\n");
        }

        // OCCT splits the face's edges in place: Draw-lite on an unfrozen copy, rebound afterwards (see the class)
        Shape sh = SWDRAWShapeFixCommands.InPlace(theDI, face, aWork =>
        {
            var SUCS = new ComposeShellFixer();
            var l = new Placement();
            SUCS.Init(Grid, l, ShapeCasts.Face(aWork), global::Kerf.Engine.Foundation.Precision.Confusion());
            var RS = new ReShapeBuilder();
            SUCS.SetContext(RS);
            SUCS.Perform();

            PrintStatuses(di, SUCS.Status);
            return SUCS.InternalResult();
        });

        sh = SameParameterInPlace(theDI, sh);
        theDI.SetShape(argv[1], sh);
        return 0;
    }

    private static int converttobspline(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 3)
        {
            di.Append("Use: ").Append(argv[0]).Append(" result shape [options=ero]\n");
            di.Append("where options is combination of letters indicating kinds of\n");
            di.Append("surfaces to be converted:\n");
            di.Append("e - extrusion\n");
            di.Append("r - revolution\n");
            di.Append("o - offset\n");
            di.Append("p - plane");
            return 1;
        }
        string options = (argc > 3 ? argv[3] : "ero");

        Shape inputShape = theDI.GetShape(argv[2]);
        if (inputShape.IsNull)
        {
            di.Append("Unknown shape\n");
            return 1;
        }
        Shape revsh = Customize.ConvertToRevolution(inputShape);
        Shape res = Customize.ConvertToBSpline(revsh, options.Contains('e'), options.Contains('r'),
                                                        options.Contains('o'), options.Contains('p'));
        res = SameParameterInPlace(theDI, res);
        theDI.SetShape(argv[1], res);
        return 0;
    }

    private static int splitclosed(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        if (argv.Count < 3)
        {
            di.Append("bad number of arguments\n");
            return 1;
        }

        Shape inputShape = theDI.GetShape(argv[2]);
        if (inputShape.IsNull)
        {
            di.Append("Unknown shape\n");
            return 1;
        }

        Shape res = SWDRAWShapeFixCommands.InPlace(theDI, inputShape, aWork =>
        {
            var tool = new ShapeDivideClosed(aWork);
            tool.Perform();
            return tool.InternalResult();
        });

        res = SameParameterInPlace(theDI, res);
        theDI.SetShape(argv[1], res);
        return 0;
    }

    private static int splitarea(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 4)
        {
            di.Append("bad number of arguments\n");
            return 1;
        }

        Shape inputShape = theDI.GetShape(argv[2]);
        if (inputShape.IsNull)
        {
            di.Append("Unknown shape\n");
            return 1;
        }
        double aMaxArea = theDI.Atof(argv[3]);

        Shape res = SWDRAWShapeFixCommands.InPlace(theDI, inputShape, aWork =>
        {
            var tool = new ShapeDivideArea(aWork);
            if (argc > 4)
            {
                double prec = theDI.Atof(argv[4]);
                tool.SetPrecision(prec);
            }
            tool.MaxArea() = aMaxArea;
            tool.Perform();
            return tool.InternalResult();
        });

        res = SameParameterInPlace(theDI, res);
        theDI.SetShape(argv[1], res);
        return 0;
    }

    private static int splitbynumber(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 4)
        {
            di.Append("bad number of arguments\n");
            return 1;
        }

        Shape inputShape = theDI.GetShape(argv[2], ShapeType.Face);
        if (inputShape.IsNull)
        {
            di.Append("Unknown face\n");
            return 1;
        }

        int aNbParts, aNumber1 = 0, aNumber2 = 0;
        aNbParts = aNumber1 = theDI.Atoi(argv[3]);
        if (argc > 4)
        {
            aNumber2 = theDI.Atoi(argv[4]);
        }

        if (argc == 4 && aNbParts <= 0)
        {
            di.Append("Incorrect number of parts\n");
            return 1;
        }
        if (argc == 5 && (aNumber1 <= 0 || aNumber2 <= 0))
        {
            di.Append("Incorrect numbers in U or V\n");
            return 1;
        }

        Shape res = SWDRAWShapeFixCommands.InPlace(theDI, inputShape, aWork =>
        {
            var tool = new ShapeDivideArea(aWork);
            tool.SetSplittingByNumber(true);
            if (argc == 4)
            {
                tool.NbParts() = aNbParts;
            }
            else
            {
                tool.SetNumbersUVSplits(aNumber1, aNumber2);
            }
            tool.Perform();
            return tool.InternalResult();
        });

        res = SameParameterInPlace(theDI, res);
        theDI.SetShape(argv[1], res);
        return 0;
    }

    private static int removeinternalwires(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        int argc = argv.Count;
        if (argc < 4)
        {
            di.Append("bad number of arguments\n");
            return 1;
        }
        double aMinArea = theDI.Atof(argv[2]);
        Shape inputShape = theDI.GetShape(argv[3]);
        if (inputShape.IsNull)
        {
            di.Append("Unknown shape\n");
            return 1;
        }
        if (inputShape.ShapeType() >= ShapeType.Wire)
        {
            di.Append("Invalid type of first shape: should be FACE,SHELL,SOLID or COMPOUND\n");
            return 1;
        }

        // RemoveInternalWires only reads the shape (decision 999): it runs on the input
        var aTool = new RemoveInternalWires(inputShape);
        var aSeqShapes = new NCollection_Sequence<Shape>();

        int k = 4;
        bool isShape = true;
        bool aModeRemoveFaces = true;

        for (; k < argc; k++)
        {
            if (isShape)
            {
                Shape aShape = theDI.GetShape(argv[k]);
                isShape = !aShape.IsNull;
                if (isShape)
                {
                    if (aShape.ShapeType() == ShapeType.Face || aShape.ShapeType() == ShapeType.Wire)
                    {
                        aSeqShapes.Append(aShape);
                    }
                }
            }
            if (!isShape)
            {
                aModeRemoveFaces = (theDI.Atoi(argv[k]) == 1);
            }
        }

        aTool.MinArea() = aMinArea;
        aTool.RemoveFaceMode() = aModeRemoveFaces;
        if (aSeqShapes.Length() != 0)
        {
            aTool.Perform(aSeqShapes);
        }
        else
        {
            aTool.Perform();
        }
        if (aTool.Status(ExtendStatus.FAIL1))
        {
            di.Append("Initial shape has invalid type\n");
        }
        else if (aTool.Status(ExtendStatus.FAIL2))
        {
            di.Append("Specified sub-shape is not belonged to whole shape\n");
        }
        if (aTool.Status(ExtendStatus.DONE1))
        {
            NCollection_Sequence<Shape> aRemovedWires = aTool.RemovedWires();
            di.Append(aRemovedWires.Length()).Append(" internal wires were removed\n");
        }
        if (aTool.Status(ExtendStatus.DONE2))
        {
            NCollection_Sequence<Shape> aRemovedFaces = aTool.RemovedFaces();
            di.Append(aRemovedFaces.Length()).Append(" small faces were removed\n");
        }
        Shape res = aTool.GetResult();

        theDI.SetShape(argv[1], res);
        return 0;
    }
}
