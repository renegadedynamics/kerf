// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Fillets.Analytic;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using static Kerf.Engine.Fillets.Data.ChamfMode;
using static Kerf.Engine.Fillets.Builder_0;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The port's side of <c>dsrec --kpart</c> (tools/dsrec/kpart.cpp): the same program in C#, case for case, writing the
/// same text. Each case reads its input shape from the recording.
/// </summary>
internal static class KPartRecorder
{
    private static readonly Lazy<List<(string Name, string BRep, string Text)>> theCases = new(Load);

    /// <summary>The recorded cases: name, input shape as BRep text, and the rest of native's text.</summary>
    internal static List<(string Name, string BRep, string Text)> Cases => theCases.Value;

    private static List<(string, string, string)> Load()
    {
        using Stream aRes = typeof(KPartRecorder).Assembly.GetManifestResourceStream("ChFiKPart.kpartrec.txt.gz")
                            ?? throw new InvalidOperationException("ChFiKPart.kpartrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        string[] aLines = aReader.ReadToEnd().Replace("\r\n", "\n").Split('\n');
        List<(string, string, string)> aCases = [];
        int k = 0;
        while (k < aLines.Length && aLines[k].StartsWith("case ", StringComparison.Ordinal))
        {
            string aName = aLines[k++][5..];
            int nB = int.Parse(aLines[k++][5..], CultureInfo.InvariantCulture);
            string aBRep = string.Join("\n", aLines, k, nB) + "\n";
            k += nB;
            StringBuilder aText = new();
            while (aLines[k] != "end-case")
            {
                aText.Append(aLines[k++]).Append('\n');
            }
            k++;
            aCases.Add((aName, aBRep, aText.ToString()));
        }
        return aCases;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // the writer (kpart.cpp's)
    // -----------------------------------------------------------------------------------------------------------------

    private static readonly string[] theOri = ["FORWARD", "REVERSED", "INTERNAL", "EXTERNAL"];

    private sealed class Out
    {
        internal readonly StringWriter OS = new() { NewLine = "\n" };

        internal Out() => StreamUtilities.Precision(OS, 17);

        internal static string R(double v) => StreamUtilities.FormatReal(v, 17);

        internal void L(string s) => OS.Write(s + "\n");

        internal void P3(string theTag, in Point3D P) => L(theTag + " " + R(P.X) + " " + R(P.Y) + " " + R(P.Z));

        internal void V3(string theTag, in Vector3D V) => L(theTag + " " + R(V.X) + " " + R(V.Y) + " " + R(V.Z));

        internal void P2(string theTag, in Point2D P) => L(theTag + " " + R(P.X) + " " + R(P.Y));

        internal void Real(string theTag, double v) => L(theTag + " " + R(v));

        internal void C2(string theTag, Curve2D? C)
        {
            if (C is null)
            {
                L(theTag + " null");
                return;
            }
            L(theTag);
            global::Kerf.Engine.Exchange.Brep.GeomTools.Write(C, OS);
        }

        internal void C3(string theTag, Curve3D? C)
        {
            if (C is null)
            {
                L(theTag + " null");
                return;
            }
            L(theTag);
            global::Kerf.Engine.Exchange.Brep.GeomTools.Write(C, OS);
        }

        internal void Cp(string theTag, in CommonPoint P)
        {
            L(theTag + " vertex " + (P.IsVertex ? 1 : 0) + " arc " + (P.IsOnArc ? 1 : 0) + " tol "
              + R(P.Tolerance()) + " param " + R(P.Parameter()));
            P3("  point", P.Point());
            if (P.HasVector)
            {
                V3("  vector", P.Vector());
            }
            if (P.IsOnArc)
            {
                L("  onarc " + R(P.ParameterOnArc()) + " " + theOri[(int)P.TransitionOnArc()]);
            }
        }

        internal void SurfData(SurfData D)
        {
            L("surf " + D.Surf() + " " + theOri[(int)D.Orientation()]);
            for (int i = 1; i <= 2; i++)
            {
                FaceInterference F = D.Interference(i);
                L("fi" + i + " line " + F.LineIndex() + " " + theOri[(int)F.Transition()] + " " + R(F.FirstParameter())
                  + " " + R(F.LastParameter()));
                C2("pcface", F.PCurveOnFace());
                C2("pcsurf", F.PCurveOnSurf());
            }
            Cp("vf1", D.VertexFirstOnS1());
            Cp("vl1", D.VertexLastOnS1());
            Cp("vf2", D.VertexFirstOnS2());
            Cp("vl2", D.VertexLastOnS2());
            D.Get2dPoints(out Point2D a, out Point2D b, out Point2D c, out Point2D d);
            P2("p2d", a);
            P2("p2d", b);
            P2("p2d", c);
            P2("p2d", d);
        }

        internal void DS(DsDataStructure DStr)
        {
            string s = TextDump.Write(DStr, new Shape());
            L("ds " + s.Count(c => c == '\n'));
            OS.Write(s);
        }

        internal void Exception(Exception e) => L("exception class " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(e.GetType()));
    }

    private static Edge EdgeOf(in Shape S, int k)
    {
        IndexedSet<Shape> M = new(ShapeHasher.Instance);
        Exploration.MapShapes(S, ShapeType.Edge, M);
        return ShapeCasts.Edge(M[k - 1]);
    }

    private static Face FaceOf(in Shape S, int k)
    {
        IndexedSet<Shape> M = new(ShapeHasher.Instance);
        Exploration.MapShapes(S, ShapeType.Face, M);
        return ShapeCasts.Face(M[k - 1]);
    }

    private static void FacesOf(in Shape S, in Edge E, out Face F1, out Face F2)
    {
        Map M = new();
        M.Fill(S, ShapeType.Edge, ShapeType.Face);
        ChFi3d_conexfaces(E, out F1, out F2, M);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // the cases
    // -----------------------------------------------------------------------------------------------------------------

    /// <summary>The port's text for the case, its input read from the recording.</summary>
    internal static string Run(string theName, string theBRep)
    {
        BrepTools.Read(out Shape S, new StringReader(theBRep));
        Out o = new();
        string aBase = theName.EndsWith("_swapped", StringComparison.Ordinal) ? theName[..^8] : theName;
        bool aSwap = aBase != theName;
        switch (aBase)
        {
            case "fil_pln_pln": KPart(o, S, 5, 0, 10, 0, ChamfMode.ClassicChamfer, aSwap); break;
            case "fil_pln_cyl_line": KPart(o, S, CylQLines(S)[0], 0, 10, 0, ChamfMode.ClassicChamfer, aSwap); break;
            case "fil_pln_cyl_circle": KPart(o, S, CylQCircles(S)[0], 0, 10, 0, ChamfMode.ClassicChamfer, aSwap); break;
            case "fil_pln_cyl_full": KPart(o, S, Circles(S)[0], 0, 10, 0, ChamfMode.ClassicChamfer, aSwap); break;
            case "fil_pln_cyl_sphere": KPart(o, S, Circles(S)[0], 0, 50, 0, ChamfMode.ClassicChamfer, aSwap); break;
            case "fil_pln_con_bottom": KPart(o, S, Circles(S)[0], 0, 10, 0, ChamfMode.ClassicChamfer, aSwap); break;
            case "fil_pln_con_top": KPart(o, S, Circles(S)[1], 0, 10, 0, ChamfMode.ClassicChamfer, aSwap); break;
            case "ch_pln_pln": KPart(o, S, 5, 1, 10, 0, ChamfMode.ClassicChamfer, aSwap); break;
            case "ch_pln_pln_two": KPart(o, S, 5, 2, 10, 20, ChamfMode.ClassicChamfer, aSwap); break;
            case "ch_pln_pln_angle": KPart(o, S, 5, 3, 10, 0.6, ChamfMode.ClassicChamfer, aSwap); break;
            case "ch_pln_pln_throat": KPart(o, S, 5, 1, 10, 0, ChamfMode.ConstThroatChamfer, aSwap); break;
            case "ch_pln_pln_penetration":
                KPart(o, S, 5, 2, 10, 15, ChamfMode.ConstThroatWithPenetrationChamfer, aSwap);
                break;
            case "ch_pln_cyl_line": KPart(o, S, CylQLines(S)[0], 1, 10, 0, ChamfMode.ClassicChamfer, aSwap); break;
            case "ch_pln_cyl_line_two": KPart(o, S, CylQLines(S)[0], 2, 10, 20, ChamfMode.ClassicChamfer, aSwap); break;
            case "ch_pln_cyl_line_angle":
                KPart(o, S, CylQLines(S)[0], 3, 10, 0.6, ChamfMode.ClassicChamfer, aSwap);
                break;
            case "ch_pln_cyl_circle": KPart(o, S, Circles(S)[0], 1, 10, 0, ChamfMode.ClassicChamfer, aSwap); break;
            case "ch_pln_cyl_circle_two": KPart(o, S, Circles(S)[0], 2, 10, 20, ChamfMode.ClassicChamfer, aSwap); break;
            case "ch_pln_cyl_circle_angle":
                KPart(o, S, Circles(S)[0], 3, 10, 0.6, ChamfMode.ClassicChamfer, aSwap);
                break;
            case "ch_pln_cyl_circle_throat":
                KPart(o, S, Circles(S)[0], 1, 10, 0, ChamfMode.ConstThroatChamfer, aSwap);
                break;
            case "ch_pln_cyl_circle_penetration":
                KPart(o, S, Circles(S)[0], 2, 10, 15, ChamfMode.ConstThroatWithPenetrationChamfer, aSwap);
                break;
            case "ch_pln_con": KPart(o, S, Circles(S)[0], 1, 10, 0, ChamfMode.ClassicChamfer, aSwap); break;
            case "ch_pln_con_two": KPart(o, S, Circles(S)[1], 2, 10, 20, ChamfMode.ClassicChamfer, aSwap); break;
            case "ch_pln_con_angle": KPart(o, S, Circles(S)[0], 3, 10, 0.6, ChamfMode.ClassicChamfer, aSwap); break;
            case "ch_pln_con_angle_top": KPart(o, S, Circles(S)[1], 3, 10, 0.6, ChamfMode.ClassicChamfer, aSwap); break;
            case "ch_pln_con_throat": KPart(o, S, Circles(S)[0], 1, 10, 0, ChamfMode.ConstThroatChamfer, aSwap); break;
            case "fil_cyl_cyl_not_implemented": KPart(o, S, Lines(S)[0], 0, 10, 0, ChamfMode.ClassicChamfer, false); break;
            case "corner_torus_cylinder":
                CornerTorus(o, new CylindricalSurface(new CoordinateSystem(), 20.0));
                break;
            case "corner_torus_sphere":
                CornerTorus(o, new SphericalSurface(new CoordinateSystem(), 20.0));
                break;
            case "corner_sphere": CornerSphere(o, 10.0, Forward); break;
            case "corner_sphere_reversed": CornerSphere(o, 10.0, Reversed); break;
            case "rotule": Rotule(o, S, 1, 3, 5, Forward, Reversed, Reversed); break;
            case "rotule_reversed": Rotule(o, S, 1, 3, 5, Reversed, Forward, Reversed); break;
            case "ComputePCurv": ComputePCurv(o); break;
            case "BuildPCurve": BuildPCurve(o); break;
            case "ComputeArete": ComputeArete(o); break;
            case "ComputeCurves": ComputeCurves(o); break;
            case "Misc": Misc(o); break;
            case "Topology": Topology(o, S); break;
            case "elspine_circle": ElSpine(o, S, Circles(S)[0], -5.0, 2 * Math.PI * 50 + 5.0); break;
            case "elspine_arc": ElSpine(o, S, CylQCircles(S)[0], 10.0, 60.0); break;
            case "elspine_line_extended": ElSpine(o, S, 5, -10.0, 110.0); break;
            default: throw new ArgumentException(theName);
        }
        return o.OS.ToString();
    }

    // the edges of the quarter cylinder between its lateral face and a plane: lines and circles
    private static List<int> CylQLines(in Shape S) => CylQEdges(S, Line);

    private static List<int> CylQCircles(in Shape S) => CylQEdges(S, Circle);

    private static List<int> CylQEdges(in Shape S, CurveKind theType)
    {
        List<int> aRes = [];
        IndexedSet<Shape> M = new(ShapeHasher.Instance);
        Exploration.MapShapes(S, ShapeType.Edge, M);
        for (int k = 1; k <= M.Count; k++)
        {
            Edge E = ShapeCasts.Edge(M[k - 1]);
            EdgeCurveAdaptor C = new(E);
            FacesOf(S, E, out Face F1, out Face F2);
            bool aCylFace = new FaceSurfaceAdaptor(F1).GetType() == SurfaceKind.Cylinder
                            || (!F2.IsNull && new FaceSurfaceAdaptor(F2).GetType() == SurfaceKind.Cylinder);
            if (!aCylFace || F2.IsNull)
            {
                continue;
            }
            if (C.GetType() == theType)
            {
                aRes.Add(k);
            }
        }
        return aRes;
    }

    private static List<int> Circles(in Shape S) => EdgesOfType(S, Circle);

    private static List<int> Lines(in Shape S) => EdgesOfType(S, Line);

    private static List<int> EdgesOfType(in Shape S, CurveKind theType)
    {
        List<int> aRes = [];
        IndexedSet<Shape> M = new(ShapeHasher.Instance);
        Exploration.MapShapes(S, ShapeType.Edge, M);
        for (int k = 1; k <= M.Count; k++)
        {
            if (new EdgeCurveAdaptor(ShapeCasts.Edge(M[k - 1])).GetType() == theType)
            {
                aRes.Add(k);
            }
        }
        return aRes;
    }

    private static void KPart(Out o, in Shape S, int k, int theKind, double a, double b,
                              ChamfMode theMode, bool theSwap)
    {
        try
        {
            Edge E = EdgeOf(S, k);
            FacesOf(S, E, out Face F1, out Face F2);
            if (theSwap)
            {
                (F1, F2) = (F2, F1);
            }
            FaceSurfaceAdaptor HS1 = new(F1);
            FaceSurfaceAdaptor HS2 = new(F2);
            int aChoix = global::Kerf.Engine.Fillets.ChFi3d.ConcaveSide(HS1, HS2, E, out Orientation Or1, out Orientation Or2);
            o.L("concave " + aChoix + " " + theOri[(int)Or1] + " " + theOri[(int)Or2]);
            Spine Sp;
            if (theKind == 0)
            {
                FilSpine fs = new();
                fs.SetEdges(E);
                fs.Load();
                fs.SetRadius(a);
                Sp = fs;
            }
            else
            {
                ChamfSpine cs = new();
                cs.SetEdges(E);
                if (theKind == 1)
                {
                    cs.SetDist(a);
                }
                else if (theKind == 2)
                {
                    cs.SetDists(a, b);
                }
                else
                {
                    cs.SetDistAngle(a, b);
                }
                cs.SetMode(theMode);
                Sp = cs;
            }
            Sp.Load();
            DsDataStructure DStr = new();
            SurfData Data = new();
            bool ok = ComputeData.Compute(DStr, Data, HS1, HS2, Or1, Or2, Sp, 1);
            o.L("ok " + (ok ? 1 : 0));
            o.SurfData(Data);
            o.DS(DStr);
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            o.Exception(e);
        }
    }

    private static void CornerTorus(Out o, Surface S2)
    {
        try
        {
            SurfaceAdaptor HS1 = new(new PlaneSurface(new Plane3D()));
            SurfaceAdaptor HS2 = new(S2);
            DsDataStructure DStr = new();
            SurfData Data = new();
            bool ok = ComputeData.ComputeCorner(DStr, Data, HS1, HS2, Forward, Forward,
                                                          Forward, Reversed, 5.0, 20.0,
                                                          new Point2D(15, 0), new Point2D(0, 15), new Point2D(0, 0),
                                                          new Point2D(Math.PI / 2, 0));
            o.L("ok " + (ok ? 1 : 0));
            o.SurfData(Data);
            o.DS(DStr);
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            o.Exception(e);
        }
    }

    private static void CornerSphere(Out o, double theRad, Orientation Or1)
    {
        try
        {
            SurfaceAdaptor HS1 = new(new PlaneSurface(new Plane3D()));
            SurfaceAdaptor HS2 = new(new CylindricalSurface(new CoordinateSystem(), 10.0));
            DsDataStructure DStr = new();
            SurfData Data = new();
            bool ok = ComputeData.ComputeCorner(DStr, Data, HS1, HS2, Forward, Forward, Or1,
                                                          Forward, theRad, new Point2D(0.0, 0.0),
                                                          new Point2D(0.0, 10.0), new Point2D(Math.PI / 2, 10.0));
            o.L("ok " + (ok ? 1 : 0));
            o.SurfData(Data);
            o.DS(DStr);
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            o.Exception(e);
        }
    }

    private static void Rotule(Out o, in Shape S, int fs, int f1, int f2, Orientation OS,
                               Orientation OS1, Orientation OS2)
    {
        try
        {
            Face F = FaceOf(S, fs);
            FaceSurfaceAdaptor HS = new(F);
            FaceSurfaceAdaptor H1 = new(FaceOf(S, f1));
            FaceSurfaceAdaptor H2 = new(FaceOf(S, f2));
            DsDataStructure DStr = new();
            SurfData Data = new();
            bool ok = ComputeData.ComputeCorner(DStr, Data, HS, H1, H2, F.Orientation, OS, OS1, OS2, 10.0);
            o.L("ok " + (ok ? 1 : 0));
            o.SurfData(Data);
            o.DS(DStr);
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            o.Exception(e);
        }
    }

    private static void ComputePCurv(Out o)
    {
        Point2D[,] aUV =
        {
            { new(1, 2), new(1, 5) }, { new(1, 5), new(1, 2) }, { new(1, 2), new(4, 2) }, { new(4, 2), new(1, 2) },
            { new(1, 2), new(3, 7) }
        };
        for (int i = 0; i < 5; i++)
        {
            for (int r = 0; r < 2; r++)
            {
                ChFi3d_ComputePCurv(aUV[i, 0], aUV[i, 1], out Curve2D C, 2.0, 5.0, r == 1);
                o.C2("pcurv", C);
            }
        }
        Surface aCyl = new CylindricalSurface(new CoordinateSystem(), 3.0);
        Curve3D aC3d = aCyl.VIso(1.0);
        ChFi3d_ComputePCurv(aC3d, new Point2D(0.5, 1.0), new Point2D(2.0, 1.0), out Curve2D C2, aCyl, 0.5, 2.0,
                            1.0e-6, out double tolr, false);
        o.C2("pcurv", C2);
        o.Real("tolreached", tolr);
    }

    private static void BuildPCurve(Out o)
    {
        Direction2D d1 = new(1, 1), d2 = new(1, -1);
        o.C2("bezier", ChFi3d_BuildPCurve(new Point2D(0, 0), ref d1, new Point2D(3, 1), ref d2, true));
        Direction2D e1 = new(1, 1), e2 = new(1, -1);
        o.C2("bezier", ChFi3d_BuildPCurve(new Point2D(0, 0), ref e1, new Point2D(3, 1), ref e2, false));
        SurfaceAdaptor aCyl = new(new CylindricalSurface(new CoordinateSystem(), 3.0));
        o.C2("bezier", ChFi3d_BuildPCurve(aCyl, new Point2D(0.2, 0), new Vector2D(1, 0.5), new Point2D(1.5, 2),
                                          new Vector2D(0.2, 1), true));
        o.C2("bezier", ChFi3d_BuildPCurve(aCyl, new Point2D(0.2, 0), new Vector3D(0, 1, 0.5), new Point2D(1.5, 2),
                                          new Vector3D(-1, 0, 1), true));
        Boundary aB = ChFi3d_mkbound(aCyl, out Curve2D aCurv, new Point2D(0.2, 0), new Point2D(1.5, 2),
                                              1.0e-4, 1.0e-2);
        o.C2("mkbound", aCurv);
        for (int i = 0; i <= 4; i++)
        {
            o.P3("value", aB.Value(0.25 * i));
        }
    }

    private static void ComputeArete(Out o)
    {
        Surface aCyl = new CylindricalSurface(new CoordinateSystem(), 3.0);
        CommonPoint P1 = new(), P2 = new();
        Point2D[,] aUV = { { new(1, 0.5), new(1, 4) }, { new(2, 1), new(0.5, 1) }, { new(0.5, 0.5), new(2, 3) } };
        for (int i = 0; i < 3; i++)
        {
            Curve3D? C3d = null;
            Curve2D? Pc = null;
            double Pardeb = 0.0, Parfin = 0.0;
            ChFi3d_ComputeArete(P1, aUV[i, 0], P2, aUV[i, 1], aCyl, ref C3d, ref Pc, ref Pardeb, ref Parfin, 1.0e-6,
                                1.0e-7, out double tolr, 0);
            o.C3("c3d", C3d);
            o.C2("pcurv", Pc);
            o.L("range " + Out.R(Pardeb) + " " + Out.R(Parfin) + " tol " + Out.R(tolr));
        }
    }

    private static void ComputeCurves(Out o)
    {
        double aPi = Math.PI;
        SurfaceAdaptor aPlnZ = new(new PlaneSurface(new Plane3D()));
        SurfaceAdaptor aPlnX = new(new PlaneSurface(new Plane3D(new Point3D(1, 0, 0), new Direction3D(1, 0, 0))));
        SurfaceAdaptor aCylZ = new(new CylindricalSurface(new CoordinateSystem(), 3.0), 0.0, 2 * aPi, -10.0, 10.0);
        SurfaceAdaptor aPlnZ2 = new(new PlaneSurface(new Plane3D(new Point3D(0, 0, 2), new Direction3D(0, 0, 1))));
        SurfaceAdaptor aPlnY3 = new(new PlaneSurface(new Plane3D(new Point3D(0, 3, 0), new Direction3D(0, 1, 0))));
        SurfaceAdaptor aSph =
            new(new SphericalSurface(new CoordinateSystem(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)), 4.0));
        List<(SurfaceAdaptorBase S1, SurfaceAdaptorBase S2, double[] d, double[] f)> aPairs = [];
        double u, v;
        {
            double[] d = new double[4], f = new double[4];
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aPlnX.Plane(), new Point3D(1, -2, 0), out u, out v);
            d[0] = 1;
            d[1] = -2;
            d[2] = u;
            d[3] = v;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aPlnX.Plane(), new Point3D(1, 3, 0), out u, out v);
            f[0] = 1;
            f[1] = 3;
            f[2] = u;
            f[3] = v;
            aPairs.Add((aPlnZ, aPlnX, d, f));
        }
        {
            double[] d = new double[4], f = new double[4];
            Point3D a = global::Kerf.Engine.Geometry.ElSLib.Value(0.5, 2.0, aCylZ.Cylinder()), b = global::Kerf.Engine.Geometry.ElSLib.Value(2.0, 2.0, aCylZ.Cylinder());
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aPlnZ2.Plane(), a, out u, out v);
            d[0] = u;
            d[1] = v;
            d[2] = 0.5;
            d[3] = 2.0;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aPlnZ2.Plane(), b, out u, out v);
            f[0] = u;
            f[1] = v;
            f[2] = 2.0;
            f[3] = 2.0;
            aPairs.Add((aPlnZ2, aCylZ, d, f));
        }
        {
            double[] d = new double[4], f = new double[4];
            d[0] = aPi / 2;
            d[1] = -1.0;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aPlnY3.Plane(), new Point3D(0, 3, -1), out u, out v);
            d[2] = u;
            d[3] = v;
            f[0] = aPi / 2;
            f[1] = 4.0;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aPlnY3.Plane(), new Point3D(0, 3, 4), out u, out v);
            f[2] = u;
            f[3] = v;
            aPairs.Add((aCylZ, aPlnY3, d, f));
        }
        {
            double[] d = new double[4], f = new double[4];
            double z = 1.0 + Math.Sqrt(7.0);
            d[0] = 0.3;
            d[1] = z;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aSph.Sphere(), global::Kerf.Engine.Geometry.ElSLib.Value(0.3, z, aCylZ.Cylinder()), out u, out v);
            d[2] = u;
            d[3] = v;
            f[0] = 1.8;
            f[1] = z;
            global::Kerf.Engine.Geometry.ElSLib.Parameters(aSph.Sphere(), global::Kerf.Engine.Geometry.ElSLib.Value(1.8, z, aCylZ.Cylinder()), out u, out v);
            f[2] = u;
            f[3] = v;
            aPairs.Add((aCylZ, aSph, d, f));
        }
        foreach (var p in aPairs)
        {
            for (int whole = 0; whole < 2; whole++)
            {
                NCollection_Array1<double> d = new(1, 4), f = new(1, 4);
                for (int j = 0; j < 4; j++)
                {
                    d[j + 1] = p.d[j];
                    f[j + 1] = p.f[j];
                }
                Curve3D? C3d = null;
                Curve2D? Pc1 = null, Pc2 = null;
                try
                {
                    bool ok = ChFi3d_ComputeCurves(p.S1, p.S2, d, f, ref C3d, ref Pc1, ref Pc2, 1.0e-6, 1.0e-7,
                                                   out double tolr, whole == 1);
                    o.L("ok " + (ok ? 1 : 0) + " tol " + Out.R(tolr));
                    if (ok)
                    {
                        o.C3("c3d", C3d);
                        o.C2("pc1", Pc1);
                        o.C2("pc2", Pc2);
                    }
                }
                catch (Exception e) when (Refusal.IsFailure(e))
                {
                    o.Exception(e);
                }
            }
        }
    }

    private static void Misc(Out o)
    {
        double aPi = Math.PI;
        SurfaceAdaptor aCyl = new(new CylindricalSurface(new CoordinateSystem(), 3.0), 0.0, 2 * aPi, -10.0, 10.0);
        CurveAdaptor aLin = new(new LineCurve3D(new Point3D(-10, 1, 2), new Direction3D(1, 0, 0.1)), 0.0, 30.0);
        foreach (double wc in new[] { -2.0e100, 2.0e100, 15.0 })
        {
            Point2D p2d = new();
            double w = wc;
            bool ok = ChFi3d_IntCS(aCyl, aLin, ref p2d, ref w);
            o.L("intcs " + (ok ? 1 : 0) + " " + Out.R(p2d.X) + " " + Out.R(p2d.Y) + " " + Out.R(w));
        }
        SurfaceAdaptor aPln = new(new PlaneSurface(new Plane3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0.2))));
        CurveAdaptor aCir = new(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5.0), 0.0,
                                     2 * aPi);
        for (int s = 0; s < 2; s++)
        {
            double W = -1.0;
            bool ok = ChFi3d_InterPlaneEdge(aPln, aCir, ref W, s == 1, 1.0e-7);
            o.L("interplane " + (ok ? 1 : 0) + " " + Out.R(W));
        }
        ChFi3d_Parameters(new CylindricalSurface(new CoordinateSystem(), 3.0), new Point3D(0, 3, 7), out double u,
                          out double v);
        o.L("parameters " + Out.R(u) + " " + Out.R(v));
        ChFi3d_Parameters(new SphericalSurface(new CoordinateSystem(), 3.0), new Point3D(0, 0, 3), out u, out v);
        o.L("parameters " + Out.R(u) + " " + Out.R(v));
        ChFi3d_TrimCurve(new CircleCurve3D(new Frame3D(), 5.0), new Point3D(0, 5, 0), new Point3D(-5, 0, 0),
                         out TrimmedCurve3D aTr);
        o.C3("trimmed", aTr);
        double wf = -1.0, wl = -1.0;
        CircleCurve3D? aCs = ChFi3d_CircularSpine(ref wf, ref wl, new Point3D(5, 0, 0), new Vector3D(0, 1, 0),
                                                new Point3D(0, 5, 0), new Vector3D(-1, 0, 0), 2.0);
        o.C3("circularspine", aCs);
        o.L("w " + Out.R(wf) + " " + Out.R(wl));
        Vector3D vd = new(0, 1, 0), vf = new(-1, 0, 0);
        o.C3("spine", ChFi3d_Spine(new Point3D(5, 0, 0), ref vd, new Point3D(0, 5, 0), ref vf, 2.0));
        o.V3("vd", vd);
        o.V3("vf", vf);
        o.Real("convtol", ChFi3d_ConvTol2dToTol3d(aCyl, 1.0e-5));
        Curve3D aC3 = new TrimmedCurve3D(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)),
                                                               3.0), 0.0, 2.0);
        Curve2D aPc1 = new LineCurve2D(new Point2D(0.0, 1.0), new Direction2D(1.0, 0.0));
        SurfaceAdaptor aSph = new(new SphericalSurface(new CoordinateSystem(), 3.2));
        Curve2D aPc2 = new LineCurve2D(new Point2D(0.0, 0.3), new Direction2D(1.0, 0.0));
        o.Real("evaltol", ChFi3d_EvalTolReached(aCyl, aPc1, aSph, aPc2, aC3));
        SurfaceAdaptor aB = new(new CylindricalSurface(new CoordinateSystem(), 3.0));
        ChFi3d_BoundSrf(aB, 0.5, 1.0, 0.0, 2.0, true);
        o.L("boundsrf " + Out.R(aB.FirstUParameter()) + " " + Out.R(aB.LastUParameter()) + " "
            + Out.R(aB.FirstVParameter()) + " " + Out.R(aB.LastVParameter()));
        SurfaceAdaptor aP = new(new PlaneSurface(new Plane3D()));
        ChFi3d_BoundSrf(aP, 0.0, 1.0, 0.0, 4.0, true);
        o.L("boundsrf " + Out.R(aP.FirstUParameter()) + " " + Out.R(aP.LastUParameter()) + " "
            + Out.R(aP.FirstVParameter()) + " " + Out.R(aP.LastVParameter()));
        Curve3D aArc = new TrimmedCurve3D(new CircleCurve3D(new Frame3D(), 5.0), 0.0, 2.0);
        o.C3("approxbyc2", ChFi3d_ApproxByC2(aArc));
        o.L("issmooth " + (ChFi3d_IsSmooth(aArc) ? 1 : 0) + " " + (ChFi3d_IsSmooth(ChFi3d_ApproxByC2(aArc)) ? 1 : 0));
    }

    private static void Topology(Out o, in Shape S)
    {
        Map EF = new(), VE = new(), VF = new();
        EF.Fill(S, ShapeType.Edge, ShapeType.Face);
        VE.Fill(S, ShapeType.Vertex, ShapeType.Edge);
        VF.Fill(S, ShapeType.Vertex, ShapeType.Face);
        IndexedSet<Shape> MV = new(ShapeHasher.Instance);
        Exploration.MapShapes(S, ShapeType.Vertex, MV);
        for (int i = 1; i <= MV.Count; i++)
        {
            Vertex V = ShapeCasts.Vertex(MV[i - 1]);
            o.L("vertex " + i + " nbface " + ChFi3d_nbface(VF[V]) + " nbedges " + ChFi3d_NumberOfEdges(V, VE)
                + " nbsharp " + ChFi3d_NumberOfSharpEdges(V, VE, EF));
            Edge[] E = new Edge[3];
            int n = 0;
            foreach (Shape it in VE[V])
            {
                if (n >= 3)
                {
                    break;
                }
                bool seen = false;
                for (int j = 0; j < n; j++)
                {
                    seen = seen || E[j].IsSame(it);
                }
                if (!seen)
                {
                    E[n++] = ShapeCasts.Edge(it);
                }
            }
            if (n == 3)
            {
                o.L("edgestate " + (int)ChFi3d_EdgeState(E, EF) + " angle " + Out.R(ChFi3d_AngleEdge(V, E[0], E[1])));
            }
        }
        IndexedSet<Shape> ME = new(ShapeHasher.Instance);
        Exploration.MapShapes(S, ShapeType.Edge, ME);
        for (int i = 1; i <= ME.Count; i++)
        {
            Edge E = ShapeCasts.Edge(ME[i - 1]);
            ChFi3d_conexfaces(E, out Face F1, out Face F2, EF);
            o.L("edge " + i + " conti " + (int)ChFi3d_evalconti(E, F1, F2) + " pseudoseam "
                + (ChFi3d_IsPseudoSeam(E, F1) ? 1 : 0));
        }
    }

    private static void ElSpine(Out o, in Shape S, int k, double WF, double WL)
    {
        try
        {
            FilSpine fs = new();
            fs.SetEdges(EdgeOf(S, k));
            fs.Load();
            fs.SetRadius(1.0);
            Spine Sp = fs;
            ElSpine HES = new();
            HES.FirstParameter(WF);
            HES.LastParameter(WL);
            Sp.D1(WF, out Point3D P, out Vector3D V);
            HES.SetFirstPointAndTgt(P, V);
            Sp.D1(WL, out P, out V);
            HES.SetLastPointAndTgt(P, V);
            ChFi3d_PerformElSpine(HES, Sp, C1, 1.0e-4, false);
            o.L("range " + Out.R(HES.FirstParameter()) + " " + Out.R(HES.LastParameter()) + " type "
                + (int)HES.GetType());
            for (int i = 0; i <= 4; i++)
            {
                double aW = HES.FirstParameter() + 0.25 * i * (HES.LastParameter() - HES.FirstParameter());
                o.P3("value", HES.Value(aW));
            }
            if (HES.GetType() == BSplineCurve)
            {
                o.C3("curve", HES.BSpline());
            }
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            o.Exception(e);
        }
    }
}
