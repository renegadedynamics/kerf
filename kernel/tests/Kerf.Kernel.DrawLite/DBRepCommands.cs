// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.Runtime.CompilerServices;
using System.Text;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// DBRep's basic shape commands (DBRep.cxx): the display settings (isos, hlr, triangles, polygons, discretisation,
/// vori, vconn) without a viewer; compound, add, explode, nexplode, exwire, emptycopy, orientation, treverse,
/// complement, invert, nbshapes, numshapes, countshapes, setflags, writebrep, readbrep, binsave and binrestore;
/// XProgress and removeinternals (unit 7W).
/// </summary>
/// <remarks>
/// Shapes are frozen values (decision 272), so the commands that change a shape in place in OCCT (add, setflags) bind
/// a new shape to the variable instead (docs/decisions.md, unit 3L1).
/// <c>XProgress</c> sets and prints Draw_ProgressIndicator's default modes, per interpreter; Draw-lite's commands make
/// no progress indicator (decision 954), so the modes change no command's output. <c>removeinternals</c> removes the
/// internal sub-shapes in place, as OCCT, on a copy whose changes every variable sharing them sees
/// (<see cref="SWDRAWShapeFixCommands.InPlace"/>).
/// </remarks>
internal static class DBRepCommands
{
    /// <summary>DBRep_Params: the display settings, kept so that the commands that print them print Draw's values.</summary>
    private sealed class Params
    {
        public int NbIsos = 2;
        public int Discretization = 30;
        public bool DispTriangles;
        public bool DisplayPolygons;
        public bool WithHLR;
        public bool WithRg1 = true;
        public bool WithRgN;
        public bool WithHid;
        public double HLRAngle = 35.0 * Math.PI / 180.0;
        // Draw_ProgressIndicator::DefaultTclMode, DefaultConsoleMode, DefaultGraphMode
        public bool ProgressTclMode;
        public bool ProgressConsoleMode;
        public bool ProgressGraphMode;
        public const double HAngMin = 1.0 * Math.PI / 180.0;
        public const double HAngMax = 35.0 * Math.PI / 180.0;
    }

    private static readonly ConditionalWeakTable<DrawInterpreter, Params> s_params = new();

    private static Params Parameters(DrawInterpreter theDI) => s_params.GetOrCreateValue(theDI);

    public static void Register(DrawInterpreter theInterpreter)
    {
        DrawCxx.Add(theInterpreter, "isos", Isos);
        DrawCxx.Add(theInterpreter, "hlr", Hlr);
        DrawCxx.Add(theInterpreter, "vori", DispOr);
        DrawCxx.Add(theInterpreter, "vconn", DispOr);
        DrawCxx.Add(theInterpreter, "triangles", Triangles);
        DrawCxx.Add(theInterpreter, "polygons", Polygons);
        DrawCxx.Add(theInterpreter, "discretisation", Discretisation);
        DrawCxx.Add(theInterpreter, "compound", Compound);
        DrawCxx.Add(theInterpreter, "add", Add);
        DrawCxx.Add(theInterpreter, "explode", Explode);
        DrawCxx.Add(theInterpreter, "nexplode", NExplode);
        DrawCxx.Add(theInterpreter, "exwire", ExWire);
        DrawCxx.Add(theInterpreter, "emptycopy", EmptyCopy);
        DrawCxx.Add(theInterpreter, "orientation", Orientation);
        DrawCxx.Add(theInterpreter, "treverse", Orientation);
        DrawCxx.Add(theInterpreter, "complement", Orientation);
        DrawCxx.Add(theInterpreter, "invert", Invert);
        DrawCxx.Add(theInterpreter, "nbshapes", NbShapes);
        DrawCxx.Add(theInterpreter, "numshapes", NumShapes);
        DrawCxx.Add(theInterpreter, "countshapes", CountShapes);
        DrawCxx.Add(theInterpreter, "setflags", SetFlags);
        DrawCxx.Add(theInterpreter, "purgemmgt", static (_, _, _) => 0);
        DrawCxx.Add(theInterpreter, "writebrep", WriteBRep);
        DrawCxx.Add(theInterpreter, "binsave", WriteBRep);
        DrawCxx.Add(theInterpreter, "readbrep", ReadBRep);
        DrawCxx.Add(theInterpreter, "binrestore", ReadBRep);
        DrawCxx.Add(theInterpreter, "XProgress", XProgress);
        DrawCxx.Add(theInterpreter, "removeinternals", RemoveInternals);
    }

    // DBRep.cxx's Characters and Float macros
    private static bool Characters(string theArg) => theArg.Any(c => !"0123456789.+-eE".Contains(c));

    private static bool Float(string theArg) => theArg.Any(c => !"0123456789+-".Contains(c));

    private static int Isos(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        Params aParams = Parameters(theDI);
        int aNbArg = a.Count - 1;
        if (aNbArg <= 0)
        {
            di.Append("Current number of isos : ").Append(aParams.NbIsos).Append("\n");
            return 0;
        }

        int aNbIsos = 0;
        bool aChange = false;
        if (!Characters(a[aNbArg]) && Float(a[aNbArg]))
        {
            return 1;
        }

        if (!Characters(a[aNbArg]))
        {
            aNbIsos = theDI.Atoi(a[aNbArg]);
            aNbArg--;
            aChange = true;
        }

        if (aNbArg <= 0)
        {
            aParams.NbIsos = aNbIsos;
            di.Append("New current number of isos : ").Append(aParams.NbIsos).Append("\n");
        }
        else if (!aChange)
        {
            for (int i = 1; i <= aNbArg; i++)
            {
                if (theDI.GetDrawable(a[i]) is DrawShape)
                {
                    // the port keeps no per-shape display settings: a shape shows the default
                    di.Append("Number of isos for ").Append(a[i]).Append(" : ").Append(aParams.NbIsos).Append("\n");
                }
            }
        }

        return 0;
    }

    private static int Hlr(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        Params p = Parameters(theDI);
        double aDegrees() => p.HLRAngle * 180 / Math.PI;
        if (a.Count <= 1)
        {
            if (p.WithHLR)
            {
                di.Append(" HLR");
                di.Append(p.WithRgN ? " RgNLines" : p.WithRg1 ? " Rg1Lines" : " no RegLines");
                di.Append(p.WithHid ? " HiddenLines" : " no HiddenLines");
                di.Append("\n");
                di.Append("Angle of discretization : ").Append(aDegrees()).Append(" degrees\n");
            }
            else
            {
                di.Append(" wireframe");
            }

            di.Append("\n");
            return 0;
        }

        string aMode = a[1].ToLowerInvariant();
        if (a.Count == 2)
        {
            switch (aMode)
            {
                case "nohlr":
                    p.WithHLR = false;
                    break;
                case "hlr":
                    p.WithHLR = true;
                    break;
                case "nohid":
                    p.WithHid = false;
                    break;
                case "hid":
                    p.WithHLR = true;
                    p.WithHid = true;
                    break;
                case "norg1":
                    p.WithRg1 = false;
                    p.WithRgN = false;
                    break;
                case "rg1":
                    p.WithHLR = true;
                    p.WithRg1 = true;
                    p.WithRgN = false;
                    break;
                case "norgn":
                    p.WithRgN = false;
                    break;
                case "rgn":
                    p.WithHLR = true;
                    p.WithRg1 = true;
                    p.WithRgN = true;
                    break;
                case "ang":
                    di.Append("Angle of discretization : ").Append(aDegrees()).Append(" degrees\n");
                    break;
                default:
                    return 1;
            }
        }

        if (a.Count >= 3 && aMode == "ang")
        {
            if (a.Count == 3)
            {
                p.HLRAngle = Math.Clamp(theDI.Atof(a[2]) * Math.PI / 180, Params.HAngMin, Params.HAngMax);
            }

            di.Append("Angle of discretization : ").Append(aDegrees()).Append(" degrees\n");
        }

        // per-shape HLR settings only change the display
        return 0;
    }

    private static int DispOr(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a[0].Equals("vori", StringComparison.OrdinalIgnoreCase))
        {
            // DBRep_WriteColorOrientation
            theDI.Output.Write("\nrouge  FORWARD\nbleu   REVERSED\nrose   EXTERNAL\norange INTERNAL\n");
        }

        return 0;
    }

    private static int Triangles(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count == 1)
        {
            Parameters(theDI).DispTriangles = !Parameters(theDI).DispTriangles;
        }

        return 0;
    }

    private static int Polygons(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count == 1)
        {
            Parameters(theDI).DisplayPolygons = !Parameters(theDI).DisplayPolygons;
        }

        return 0;
    }

    private static int Discretisation(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count <= 1)
        {
            di.Append("Current number of points : ").Append(Parameters(theDI).Discretization).Append("\n");
        }
        else
        {
            Parameters(theDI).Discretization = theDI.Atoi(a[1]);
        }

        return 0;
    }

    private static int Compound(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n <= 1)
        {
            return 1;
        }

        var B = new BrepBuilder();
        Compound C = new();
        B.MakeCompound(ref C);
        for (int i = 1; i < n - 1; i++)
        {
            Shape S2 = theDI.GetShape(a[i]);
            if (!S2.IsNull)
            {
                B.Add(C, S2);
            }
        }

        theDI.SetShape(a[n - 1], C);
        return 0;
    }

    private static int EmptyCopy(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n <= 1)
        {
            return 1;
        }

        Shape S = theDI.GetShape(a[n == 2 ? 1 : 2]);
        if (S.IsNull)
        {
            return 1;
        }

        S.EmptyCopy();
        theDI.SetShape(a[1], S);
        return 0;
    }

    /// <summary>
    /// add what where: OCCT adds to the TShape of "where" in place. "where" is frozen, so the port binds to it a new
    /// TShape with the same type, geometry, flags and children, plus "what" (decision 404).
    /// </summary>
    private static int Add(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            return 1;
        }

        Shape S1 = theDI.GetShape(a[1]);
        if (S1.IsNull)
        {
            return 1;
        }

        Shape S2 = theDI.GetShape(a[2]);
        if (S2.IsNull)
        {
            return 1;
        }

        // OCCT's TopoDS_Builder::Add refuses a shape that isn't free before it changes anything
        if (!S2.Free())
        {
            throw new FrozenShapeException("TopoDS_Builder::Add");
        }

        Shape aCopy = WritableCopy(S2);
        new BrepBuilder().Add(aCopy, S1);
        theDI.SetShape(a[2], aCopy);
        return 0;
    }

    /// <summary>
    /// A new, unfrozen TShape in place of the shape's: its EmptyCopy (the geometry), the flags, and the same children,
    /// at the shape's location and orientation. What OCCT's in-place changes of a TShape change, the copy can take.
    /// </summary>
    internal static Shape WritableCopy(in Shape theShape)
    {
        Shape aCopy = theShape.EmptyCopied();
        TShape aFrom = theShape.TShape()!;
        TShape aTo = aCopy.TShape()!;
        aTo.Free(true);
        var B = new BrepBuilder();
        for (var it = new Iterator(theShape); it.More(); it.Next())
        {
            B.Add(aCopy, it.Value());
        }

        aTo.Free(aFrom.Free());
        aTo.Modified(aFrom.Modified());
        aTo.Checked(aFrom.Checked());
        aTo.Orientable(aFrom.Orientable());
        aTo.Closed(aFrom.Closed());
        aTo.Infinite(aFrom.Infinite());
        aTo.Convex(aFrom.Convex());
        aTo.Locked(aFrom.LockedFlag());
        return aCopy;
    }

    // explode name [Cd/C/So/Sh/F/W/E/V]
    private static int Explode(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n <= 1)
        {
            return 1;
        }

        Shape S = theDI.GetShape(a[1]);
        if (S.IsNull)
        {
            return 0;
        }

        string aPrefix = a[1] + "_";
        int i = 0;
        if (n == 2)
        {
            for (var itr = new Iterator(S); itr.More(); itr.Next())
            {
                i++;
                string aName = aPrefix + i.ToString(CultureInfo.InvariantCulture);
                theDI.SetShape(aName, itr.Value());
                di.AppendElement(aName);
            }

            return 0;
        }

        // explode a type
        string aType = a[2];
        char c0 = aType.Length > 0 ? aType[0] : '\0';
        char c1 = aType.Length > 1 ? aType[1] : '\0';
        ShapeType typ;
        switch (c0)
        {
            case 'C':
            case 'c':
                typ = c1 is 'd' or 'D' ? global::Kerf.Topology.ShapeType.Compound : ShapeType.CompSolid;
                break;
            case 'S':
            case 's':
                if (c1 is 'O' or 'o')
                {
                    typ = ShapeType.Solid;
                }
                else if (c1 is 'H' or 'h')
                {
                    typ = ShapeType.Shell;
                }
                else
                {
                    return 1;
                }

                break;
            case 'F':
            case 'f':
                typ = ShapeType.Face;
                break;
            case 'W':
            case 'w':
                typ = ShapeType.Wire;
                break;
            case 'E':
            case 'e':
                typ = ShapeType.Edge;
                break;
            case 'V':
            case 'v':
                typ = ShapeType.Vertex;
                break;
            default:
                return 1;
        }

        var M = new HashSet<Shape>(ShapeHasher.Instance);
        M.Add(S);
        for (var ex = new Explorer(S, typ); ex.More(); ex.Next())
        {
            Shape Sx = ex.Current();
            if (M.Add(Sx))
            {
                i++;
                string aName = aPrefix + i.ToString(CultureInfo.InvariantCulture);
                theDI.SetShape(aName, Sx);
                di.AppendElement(aName);
            }
        }

        return 0;
    }

    // nexplode: stable numbered explode, by the position of the centre of mass
    private static int NExplode(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count <= 2)
        {
            return 1;
        }

        Shape S = theDI.GetShape(a[1]);
        if (S.IsNull)
        {
            return 0;
        }

        string aPrefix = a[1] + "_";
        ShapeType typ;
        switch (a[2].Length > 0 ? a[2][0] : '\0')
        {
            case 'F':
            case 'f':
                typ = ShapeType.Face;
                break;
            case 'E':
            case 'e':
                typ = ShapeType.Edge;
                break;
            case 'V':
            case 'v':
                typ = ShapeType.Vertex;
                break;
            default:
                return 1;
        }

        var IMOStmp = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        var MShape = new HashSet<Shape>(ShapeHasher.Instance);
        IMOStmp.Add(S);
        Exploration.MapShapes(S, typ, IMOStmp);
        // OCCT bug: one less than the sub-shapes when the shape is of the type itself (occt-issues.md)
        int MaxShapes = IMOStmp.Extent() - 1;
        var aShapes = new NCollection_Array1<Shape>(1, MaxShapes);

        // explode
        int Index = 0;
        for (var Exp = new Explorer(S, typ); Exp.More(); Exp.Next())
        {
            if (MShape.Add(Exp.Current()))
            {
                Index++;
                aShapes.SetValue(Index, Exp.Current());
            }
        }

        var OrderInd = new NCollection_Array1<int>(1, MaxShapes);
        var MidXYZ = new NCollection_Array1<double>(1, MaxShapes);
        var GPr = new Properties();

        // the centre of mass of an edge or a face (of its edges), the point of a vertex
        for (Index = 1; Index <= MaxShapes; Index++)
        {
            OrderInd.SetValue(Index, Index);
            Shape aS = aShapes[Index];
            Point3D GPoint;
            if (aS.ShapeType() != ShapeType.Vertex)
            {
                MassPropertiesBuilder.LinearProperties(aS, GPr);
                GPoint = GPr.CentreOfMass();
            }
            else
            {
                GPoint = Tool.Pnt(ShapeCasts.Vertex(aS));
            }

            MidXYZ.SetValue(Index, GPoint.X() * 999 + GPoint.Y() * 99 + GPoint.Z() * 0.9);
        }

        // sorting
        bool NoSort = true;
        while (NoSort)
        {
            NoSort = false;
            for (Index = 1; Index < MaxShapes; Index++)
            {
                if (MidXYZ[OrderInd[Index]] > MidXYZ[OrderInd[Index + 1]])
                {
                    (OrderInd[Index], OrderInd[Index + 1]) = (OrderInd[Index + 1], OrderInd[Index]);
                    NoSort = true;
                }
            }
        }

        // check equality of MidXYZ
        for (Index = 1; Index < MaxShapes; Index++)
        {
            if (MidXYZ[OrderInd[Index + 1]] == MidXYZ[OrderInd[Index]])
            {
                di.Append("Warning! For this shape the results may be incorrect.\n");
            }
        }

        for (Index = 1; Index <= MaxShapes; Index++)
        {
            string aName = aPrefix + Index.ToString(CultureInfo.InvariantCulture);
            theDI.SetShape(aName, aShapes[OrderInd[Index]]);
            di.AppendElement(aName);
        }

        return 0;
    }

    private static int ExWire(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count <= 1)
        {
            return 1;
        }

        Shape S = theDI.GetShape(a[1]);
        if (S.IsNull || S.ShapeType() != ShapeType.Wire)
        {
            return 0;
        }

        int i = 0;
        for (var ex = new WireExplorer(ShapeCasts.Wire(S)); ex.More(); ex.Next())
        {
            i++;
            theDI.SetShape(a[1] + "_" + i.ToString(CultureInfo.InvariantCulture), ex.Current());
        }

        return 0;
    }

    // invert name: the sub-shapes reversed, in a new TShape (as OCCT: an EmptyCopied one)
    private static int Invert(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count <= 1)
        {
            return 1;
        }

        Shape S = theDI.GetShape(a[1]);
        if (S.IsNull)
        {
            return 0;
        }

        var B = new BrepBuilder();
        Shape NS = S.EmptyCopied();
        NS.Closed(S.Closed());
        for (var itr = new Iterator(S); itr.More(); itr.Next())
        {
            B.Add(NS, itr.Value().Reversed());
        }

        theDI.SetShape(a[1], NS);
        return 0;
    }

    // orientation name1 name2.. F/R/E/I, treverse name1 name2 ..., complement name1 name2 ...
    private static int Orientation(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n <= 1)
        {
            return 1;
        }

        int cas = 0;
        Orientation ori = Forward;
        int last = n;
        string aCommand = a[0].ToLowerInvariant();
        if (aCommand == "orientation")
        {
            if (n <= 2)
            {
                return 1;
            }

            last--;
            switch (a[n - 1].Length > 0 ? a[n - 1][0] : '\0')
            {
                case 'F':
                    ori = Forward;
                    break;
                case 'R':
                    ori = Reversed;
                    break;
                case 'I':
                    ori = Internal;
                    break;
                case 'E':
                    ori = External;
                    break;
            }
        }
        else if (aCommand == "treverse")
        {
            cas = -1;
        }
        else if (aCommand == "complement")
        {
            cas = -2;
        }

        for (int i = 1; i < last; i++)
        {
            Shape S = theDI.GetShape(a[i]);
            if (!S.IsNull)
            {
                S = cas == -2 ? S.Complemented() : cas == -1 ? S.Reversed() : S.Oriented(ori);
                theDI.SetShape(a[i], S);
            }
        }

        return 0;
    }

    private static int NumShapes(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }

        for (int i = 1; i < a.Count; i++)
        {
            Shape S = theDI.GetShape(a[i]);
            if (!S.IsNull)
            {
                var BS = new BrepShapeSet();
                BS.Add(S);
                di.Append("Number of shapes in ").Append(a[i]).Append("\n");
                string Astr = "";
                BS.DumpExtent(ref Astr);
                di.Append(Astr);
                di.Append("\n");
            }
        }

        return 0;
    }

    /// <summary>nbshapes -t: the sub-shapes of each type with different locations counted apart.</summary>
    private static string DumpExtent(in Shape aS)
    {
        string[] pNames =
        [
            " SHAPE     : ", " COMPOUND  : ", " COMPSOLID : ", " SOLID     : ", " SHELL     : ", " FACE      : ",
            " WIRE      : ", " EDGE      : ", " VERTEX    : ",
        ];
        var aStr = new StringBuilder();
        int aNbSh = 0;
        var aM = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        for (int i = 7; i >= 0; --i)
        {
            aM.Clear();
            Exploration.MapShapes(aS, (ShapeType)i, aM);
            int aNb = aM.Extent();
            aStr.Append(pNames[i + 1]).Append(aNb.ToString(CultureInfo.InvariantCulture)).Append('\n');
            aNbSh += aNb;
        }

        aStr.Append(pNames[0]).Append(aNbSh.ToString(CultureInfo.InvariantCulture)).Append('\n');
        return aStr.ToString();
    }

    // XProgress [+|-t] [+|-c] [+|-g]
    private static int XProgress(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> argv)
    {
        Params aParams = Parameters(theDI);
        int argc = argv.Count;
        for (int i = 1; i < argc; i++)
        {
            bool turn = true;
            char aFirst = argv[i].Length > 0 ? argv[i][0] : '\0';
            if (aFirst == '-')
            {
                turn = false;
            }
            else if (aFirst != '+')
            {
                continue;
            }

            char aSecond = argv[i].Length > 1 ? argv[i][1] : '\0';
            if (aSecond == 't')
            {
                aParams.ProgressTclMode = turn;
            }
            else if (aSecond == 'c')
            {
                aParams.ProgressConsoleMode = turn;
            }
            else if (aSecond == 'g')
            {
                aParams.ProgressGraphMode = turn;
            }
            else if (argv[i] == "-stop" && i + 1 < argc)
            {
                // Draw_ProgressIndicator::StopIndicator: the address of an indicator to stop; there is none
                ++i;
                return 0;
            }
        }
        di.Append("Progress Indicator defaults: tcl mode is ").Append(aParams.ProgressTclMode ? "ON" : "OFF");
        di.Append(", console mode is ").Append(aParams.ProgressConsoleMode ? "ON" : "OFF");
        di.Append(", graphical mode is ").Append(aParams.ProgressGraphMode ? "ON" : "OFF");
        di.Append("\n");
        return 0;
    }

    // removeinternals shape [force flag {0/1}]
    private static int RemoveInternals(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            // di.PrintHelp: Draw-lite has no help texts
            return 1;
        }

        Shape aShape = theDI.GetShape(a[1]);
        if (aShape.IsNull)
        {
            di.Append(a[1]).Append("is a null shape\n");
            return 1;
        }

        bool isForce = false;
        if (n > 2)
        {
            isForce = theDI.Atoi(a[2]) != 0;
        }

        aShape = SWDRAWShapeFixCommands.InPlace(theDI, aShape, s =>
        {
            BrepTools.RemoveInternals(s, isForce);
            return s;
        });

        theDI.SetShape(a[1], aShape);

        return 0;
    }

    private static int NbShapes(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 2)
        {
            return 1;
        }

        bool aTotal = a[n - 1] == "-t";
        for (int i = 1; i < n; i++)
        {
            Shape S = theDI.GetShape(a[i]);
            if (!S.IsNull)
            {
                di.Append("Number of shapes in ").Append(a[i]).Append("\n");
                if (aTotal)
                {
                    di.Append(DumpExtent(S));
                }
                else
                {
                    var BS = new BrepShapeSet();
                    BS.Add(S);
                    string Astr = "";
                    BS.DumpExtent(ref Astr);
                    di.Append(Astr);
                }
            }
        }

        return 0;
    }

    private static int CountShapes(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 2)
        {
            return 1;
        }

        (ShapeType Type, string Label)[] aTypes =
        [
            (ShapeType.Vertex, " VERTEX     : "), (ShapeType.Edge, " EDGE       : "), (ShapeType.Wire, " WIRE       : "),
            (ShapeType.Face, " FACE       : "), (ShapeType.Shell, " SHELL      : "), (ShapeType.Solid, " SOLID      : "),
            (ShapeType.CompSolid, " COMPSOLID  : "), (global::Kerf.Topology.ShapeType.Compound, " COMPOUND   : "),
        ];
        for (int i = 1; i < a.Count; i++)
        {
            Shape Sh = theDI.GetShape(a[i]);
            if (Sh.IsNull)
            {
                continue;
            }

            di.Append("Number of shapes in ").Append(a[i]).Append("\n");
            var M = new HashSet<Shape>(ShapeHasher.Instance);
            foreach ((ShapeType aType, string aLabel) in aTypes)
            {
                int nbElem = 0;
                for (var ex = new Explorer(Sh, aType); ex.More(); ex.Next())
                {
                    if (M.Add(ex.Current()))
                    {
                        nbElem++;
                    }
                }

                di.Append(aLabel).Append(nbElem).Append("\n");
            }

            di.Append(" SHAPE      : ").Append(M.Count).Append("\n");
            di.Append("\n");
        }

        return 0;
    }

    // setflags shape flag1 [flag2...]: free, modified, checked, orientable, closed, infinite, convex, locked
    private static void SetProp(TShape Sh, IReadOnlyList<string> a)
    {
        for (int i = 2; i < a.Count; i++)
        {
            bool aValue = !a[i].StartsWith('-');
            if (a[i].Contains("free", StringComparison.Ordinal))
            {
                Sh.Free(aValue);
            }

            if (a[i].Contains("modified", StringComparison.Ordinal))
            {
                Sh.Modified(aValue);
            }

            if (a[i].Contains("checked", StringComparison.Ordinal))
            {
                Sh.Checked(aValue);
            }

            if (a[i].Contains("orientable", StringComparison.Ordinal))
            {
                Sh.Orientable(aValue);
            }

            if (a[i].Contains("closed", StringComparison.Ordinal))
            {
                Sh.Closed(aValue);
            }

            if (a[i].Contains("infinite", StringComparison.Ordinal))
            {
                Sh.Infinite(aValue);
            }

            if (a[i].Contains("convex", StringComparison.Ordinal))
            {
                Sh.Convex(aValue);
            }

            if (a[i].Contains("locked", StringComparison.Ordinal))
            {
                Sh.Locked(aValue);
            }
        }
    }

    /// <summary>
    /// setflags: OCCT sets the flags in place on the shape and its vertices, edges and faces. The port sets them on a
    /// structural copy (BRepTools.CopyForWrite: new TShapes sharing the geometry) and binds the copy (decision 404).
    /// </summary>
    private static int SetFlags(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count < 3)
        {
            return 1;
        }

        Shape Sh = theDI.GetShape(a[1]);
        if (Sh.IsNull)
        {
            return 1;
        }

        Shape aCopy = BrepTools.CopyForWrite(Sh, out _);
        SetProp(aCopy.TShape()!, a);
        foreach (ShapeType aType in new[] { ShapeType.Vertex, ShapeType.Edge, ShapeType.Face })
        {
            for (var ex = new Explorer(aCopy, aType); ex.More(); ex.Next())
            {
                SetProp(ex.Current().TShape()!, a);
            }
        }

        theDI.SetShape(a[1], aCopy);
        return 0;
    }

    // Draw::ParseOnOff, ParseOnOffIterator, ParseOnOffNoIterator
    internal static bool ParseOnOff(string theArg, ref bool theIsOn)
    {
        switch (theArg.ToLowerInvariant())
        {
            case "on" or "1":
                theIsOn = true;
                return true;
            case "off" or "0":
                theIsOn = false;
                return true;
            default:
                return false;
        }
    }

    internal static bool ParseOnOffIterator(IReadOnlyList<string> a, ref int theArgIter)
    {
        bool isOn = true;
        if (theArgIter + 1 < a.Count && ParseOnOff(a[theArgIter + 1], ref isOn))
        {
            ++theArgIter;
        }

        return isOn;
    }

    internal static bool ParseOnOffNoIterator(IReadOnlyList<string> a, ref int theArgIter)
    {
        bool toReverse = a[theArgIter].StartsWith("no", StringComparison.OrdinalIgnoreCase)
                         || a[theArgIter].StartsWith("-no", StringComparison.OrdinalIgnoreCase);
        bool isOn = ParseOnOffIterator(a, ref theArgIter);
        return toReverse ? !isOn : isOn;
    }

    // writebrep shape filename [-binary {0|1}] [-version V] [-triangles {0|1}] [-normals {0|1}]; binsave
    private static int WriteBRep(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int aVersion = -1;
        string aShapeName = "", aFileName = "";
        Shape aShape = new();
        bool isBinaryFormat = a[0].Equals("binsave", StringComparison.OrdinalIgnoreCase);
        bool isWithTriangles = true;
        bool isWithNormals = false;
        for (int anArgIter = 1; anArgIter < a.Count; ++anArgIter)
        {
            string aParam = a[anArgIter].ToLowerInvariant();
            if (aParam == "-binary")
            {
                isBinaryFormat = ParseOnOffNoIterator(a, ref anArgIter);
            }
            else if (aParam == "-version" && anArgIter + 1 < a.Count)
            {
                aVersion = theDI.Atoi(a[++anArgIter]);
                if (aVersion <= 0)
                {
                    di.Append("Syntax error: unknown version");
                    return 1;
                }
            }
            else if (aParam is "-notriangles" or "-triangles")
            {
                isWithTriangles = ParseOnOffNoIterator(a, ref anArgIter);
            }
            else if (aParam is "-nonormals" or "-normals")
            {
                isWithNormals = ParseOnOffIterator(a, ref anArgIter);
                if (aParam == "-nonormals")
                {
                    isWithNormals = !isWithNormals;
                }
            }
            else if (aShapeName.Length == 0)
            {
                aShapeName = a[anArgIter];
                aShape = theDI.GetShape(aShapeName);
                if (aShape.IsNull)
                {
                    di.Append("Syntax error: ").Append(aShapeName).Append(" is not a shape");
                    return 1;
                }
            }
            else if (aFileName.Length == 0)
            {
                aFileName = a[anArgIter];
            }
            else
            {
                di.Append("Syntax error: unknown argument '").Append(aParam).Append("'");
                return 1;
            }
        }

        if (aFileName.Length == 0)
        {
            di.Append("Syntax error: wrong number of arguments");
            return 1;
        }

        byte[] aBytes;
        if (isBinaryFormat)
        {
            if (aVersion > (int)global::Kerf.Engine.Exchange.Brep.Binary.BinaryFormatVersionBounds.BinTools_FormatVersion_UPPER)
            {
                di.Append("Syntax error: unknown format version");
                return 1;
            }

            if (isWithNormals && aVersion > 0 && aVersion < (int)global::Kerf.Engine.Exchange.Brep.Binary.BinaryFormatVersion.FormatVersion_VERSION_4)
            {
                di.Append("Error: vertex normals require binary format version 4 or later");
                return 1;
            }

            var aVersionNb = aVersion > 0 ? (global::Kerf.Engine.Exchange.Brep.Binary.BinaryFormatVersion)aVersion : global::Kerf.Engine.Exchange.Brep.Binary.BinaryFormatVersion.FormatVersion_CURRENT;
            var aStream = new MemoryStream();
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Write(aShape, aStream, isWithTriangles, isWithNormals, aVersionNb);
            aBytes = aStream.ToArray();
        }
        else
        {
            if (aVersion > (int)FormatVersionBounds.TopTools_FormatVersion_UPPER)
            {
                di.Append("Syntax error: unknown format version");
                return 1;
            }

            if (isWithNormals && aVersion > 0 && aVersion < (int)FormatVersion.FormatVersion_VERSION_3)
            {
                di.Append("Error: vertex normals require ascii format version 3 or later");
                return 1;
            }

            var aVersionNb = aVersion > 0 ? (FormatVersion)aVersion : FormatVersion.FormatVersion_CURRENT;
            var aStream = new StringWriter { NewLine = "\n" };
            aStream.Write("DBRep_DrawableShape\n");
            BrepTools.Write(aShape, aStream, isWithTriangles, isWithNormals, aVersionNb);
            aBytes = Encoding.Latin1.GetBytes(aStream.ToString());
        }

        theDI.Files.WriteBytes(aFileName, aBytes);
        di.Append(aShapeName);
        return 0;
    }

    // readbrep filename shape (binrestore): binary unless the file starts with DBRep_DrawableShape
    private static int ReadBRep(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        if (a.Count != 3)
        {
            di.Append("Syntax error: wrong number of arguments");
            return 1;
        }

        string aFileName = a[1];
        string aShapeName = a[2];
        if (!theDI.Files.TryReadBytes(aFileName, out byte[] aBytes) || aBytes.Length < 255)
        {
            // OCCT reads 255 bytes to probe the format, and fails on a shorter file
            di.Append("Error: cannot read the file '").Append(aFileName).Append("'");
            return 1;
        }

        bool isBinaryFormat = !aBytes.AsSpan().StartsWith("DBRep_DrawableShape"u8);
        Shape aShape;
        if (isBinaryFormat)
        {
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Read(out aShape, new MemoryStream(aBytes, false));
        }
        else
        {
            string aText = Encoding.Latin1.GetString(aBytes);
            var aReader = new StringReader(aText);
            StreamUtilities.SetInputLength(aReader, aText.Length);
            var aShapeSet = new BrepShapeSet();
            aShapeSet.Read(aReader);
            if (aShapeSet.ShapeCount == 0)
            {
                di.Append("Error: cannot read from the file '").Append(aFileName).Append("'");
                return 1;
            }

            aShapeSet.Read(out aShape, aReader);
        }

        theDI.SetShape(aShapeName, aShape);
        di.Append(aShapeName);
        return 0;
    }
}
