// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.IO.Compression;
using System.Text;
using static Occt.ChFiDS_ChamfMode;
using static Occt.ChFi3d_Builder_0;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

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

        internal Out() => Standard_Stream.Precision(OS, 17);

        internal static string R(double v) => Standard_Stream.FormatReal(v, 17);

        internal void L(string s) => OS.Write(s + "\n");

        internal void P3(string theTag, in gp_Pnt P) => L(theTag + " " + R(P.X()) + " " + R(P.Y()) + " " + R(P.Z()));

        internal void V3(string theTag, in gp_Vec V) => L(theTag + " " + R(V.X()) + " " + R(V.Y()) + " " + R(V.Z()));

        internal void P2(string theTag, in gp_Pnt2d P) => L(theTag + " " + R(P.X()) + " " + R(P.Y()));

        internal void Real(string theTag, double v) => L(theTag + " " + R(v));

        internal void C2(string theTag, Geom2d_Curve? C)
        {
            if (C is null)
            {
                L(theTag + " null");
                return;
            }
            L(theTag);
            GeomTools.Write(C, OS);
        }

        internal void C3(string theTag, Geom_Curve? C)
        {
            if (C is null)
            {
                L(theTag + " null");
                return;
            }
            L(theTag);
            GeomTools.Write(C, OS);
        }

        internal void Cp(string theTag, in ChFiDS_CommonPoint P)
        {
            L(theTag + " vertex " + (P.IsVertex() ? 1 : 0) + " arc " + (P.IsOnArc() ? 1 : 0) + " tol " + R(P.Tolerance())
              + " param " + R(P.Parameter()));
            P3("  point", P.Point());
            if (P.HasVector())
            {
                V3("  vector", P.Vector());
            }
            if (P.IsOnArc())
            {
                L("  onarc " + R(P.ParameterOnArc()) + " " + theOri[(int)P.TransitionOnArc()]);
            }
        }

        internal void SurfData(ChFiDS_SurfData D)
        {
            L("surf " + D.Surf() + " " + theOri[(int)D.Orientation()]);
            for (int i = 1; i <= 2; i++)
            {
                ChFiDS_FaceInterference F = D.Interference(i);
                L("fi" + i + " line " + F.LineIndex() + " " + theOri[(int)F.Transition()] + " " + R(F.FirstParameter())
                  + " " + R(F.LastParameter()));
                C2("pcface", F.PCurveOnFace());
                C2("pcsurf", F.PCurveOnSurf());
            }
            Cp("vf1", D.VertexFirstOnS1());
            Cp("vl1", D.VertexLastOnS1());
            Cp("vf2", D.VertexFirstOnS2());
            Cp("vl2", D.VertexLastOnS2());
            D.Get2dPoints(out gp_Pnt2d a, out gp_Pnt2d b, out gp_Pnt2d c, out gp_Pnt2d d);
            P2("p2d", a);
            P2("p2d", b);
            P2("p2d", c);
            P2("p2d", d);
        }

        internal void DS(TopOpeBRepDS_DataStructure DStr)
        {
            string s = TopOpeBRepDS_TextDump.Write(DStr, new TopoDS_Shape());
            L("ds " + s.Count(c => c == '\n'));
            OS.Write(s);
        }

        internal void Exception(Exception e) => L("exception class " + e.GetType().Name);
    }

    private static TopoDS_Edge EdgeOf(in TopoDS_Shape S, int k)
    {
        NCollection_IndexedMap<TopoDS_Shape> M = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, TopAbs_EDGE, M);
        return TopoDS.Edge(M.FindKey(k));
    }

    private static TopoDS_Face FaceOf(in TopoDS_Shape S, int k)
    {
        NCollection_IndexedMap<TopoDS_Shape> M = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, TopAbs_FACE, M);
        return TopoDS.Face(M.FindKey(k));
    }

    private static void FacesOf(in TopoDS_Shape S, in TopoDS_Edge E, out TopoDS_Face F1, out TopoDS_Face F2)
    {
        ChFiDS_Map M = new();
        M.Fill(S, TopAbs_EDGE, TopAbs_FACE);
        ChFi3d_conexfaces(E, out F1, out F2, M);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // the cases
    // -----------------------------------------------------------------------------------------------------------------

    /// <summary>The port's text for the case, its input read from the recording.</summary>
    internal static string Run(string theName, string theBRep)
    {
        BRepTools.Read(out TopoDS_Shape S, new StringReader(theBRep));
        Out o = new();
        string aBase = theName.EndsWith("_swapped", StringComparison.Ordinal) ? theName[..^8] : theName;
        bool aSwap = aBase != theName;
        switch (aBase)
        {
            case "fil_pln_pln": KPart(o, S, 5, 0, 10, 0, ChFiDS_ClassicChamfer, aSwap); break;
            case "fil_pln_cyl_line": KPart(o, S, CylQLines(S)[0], 0, 10, 0, ChFiDS_ClassicChamfer, aSwap); break;
            case "fil_pln_cyl_circle": KPart(o, S, CylQCircles(S)[0], 0, 10, 0, ChFiDS_ClassicChamfer, aSwap); break;
            case "fil_pln_cyl_full": KPart(o, S, Circles(S)[0], 0, 10, 0, ChFiDS_ClassicChamfer, aSwap); break;
            case "fil_pln_cyl_sphere": KPart(o, S, Circles(S)[0], 0, 50, 0, ChFiDS_ClassicChamfer, aSwap); break;
            case "fil_pln_con_bottom": KPart(o, S, Circles(S)[0], 0, 10, 0, ChFiDS_ClassicChamfer, aSwap); break;
            case "fil_pln_con_top": KPart(o, S, Circles(S)[1], 0, 10, 0, ChFiDS_ClassicChamfer, aSwap); break;
            case "ch_pln_pln": KPart(o, S, 5, 1, 10, 0, ChFiDS_ClassicChamfer, aSwap); break;
            case "ch_pln_pln_two": KPart(o, S, 5, 2, 10, 20, ChFiDS_ClassicChamfer, aSwap); break;
            case "ch_pln_pln_angle": KPart(o, S, 5, 3, 10, 0.6, ChFiDS_ClassicChamfer, aSwap); break;
            case "ch_pln_pln_throat": KPart(o, S, 5, 1, 10, 0, ChFiDS_ConstThroatChamfer, aSwap); break;
            case "ch_pln_pln_penetration":
                KPart(o, S, 5, 2, 10, 15, ChFiDS_ConstThroatWithPenetrationChamfer, aSwap);
                break;
            case "ch_pln_cyl_line": KPart(o, S, CylQLines(S)[0], 1, 10, 0, ChFiDS_ClassicChamfer, aSwap); break;
            case "ch_pln_cyl_line_two": KPart(o, S, CylQLines(S)[0], 2, 10, 20, ChFiDS_ClassicChamfer, aSwap); break;
            case "ch_pln_cyl_line_angle":
                KPart(o, S, CylQLines(S)[0], 3, 10, 0.6, ChFiDS_ClassicChamfer, aSwap);
                break;
            case "ch_pln_cyl_circle": KPart(o, S, Circles(S)[0], 1, 10, 0, ChFiDS_ClassicChamfer, aSwap); break;
            case "ch_pln_cyl_circle_two": KPart(o, S, Circles(S)[0], 2, 10, 20, ChFiDS_ClassicChamfer, aSwap); break;
            case "ch_pln_cyl_circle_angle":
                KPart(o, S, Circles(S)[0], 3, 10, 0.6, ChFiDS_ClassicChamfer, aSwap);
                break;
            case "ch_pln_cyl_circle_throat":
                KPart(o, S, Circles(S)[0], 1, 10, 0, ChFiDS_ConstThroatChamfer, aSwap);
                break;
            case "ch_pln_cyl_circle_penetration":
                KPart(o, S, Circles(S)[0], 2, 10, 15, ChFiDS_ConstThroatWithPenetrationChamfer, aSwap);
                break;
            case "ch_pln_con": KPart(o, S, Circles(S)[0], 1, 10, 0, ChFiDS_ClassicChamfer, aSwap); break;
            case "ch_pln_con_two": KPart(o, S, Circles(S)[1], 2, 10, 20, ChFiDS_ClassicChamfer, aSwap); break;
            case "ch_pln_con_angle": KPart(o, S, Circles(S)[0], 3, 10, 0.6, ChFiDS_ClassicChamfer, aSwap); break;
            case "ch_pln_con_angle_top": KPart(o, S, Circles(S)[1], 3, 10, 0.6, ChFiDS_ClassicChamfer, aSwap); break;
            case "ch_pln_con_throat": KPart(o, S, Circles(S)[0], 1, 10, 0, ChFiDS_ConstThroatChamfer, aSwap); break;
            case "fil_cyl_cyl_not_implemented": KPart(o, S, Lines(S)[0], 0, 10, 0, ChFiDS_ClassicChamfer, false); break;
            case "corner_torus_cylinder":
                CornerTorus(o, new Geom_CylindricalSurface(new gp_Ax3(), 20.0));
                break;
            case "corner_torus_sphere":
                CornerTorus(o, new Geom_SphericalSurface(new gp_Ax3(), 20.0));
                break;
            case "corner_sphere": CornerSphere(o, 10.0, TopAbs_FORWARD); break;
            case "corner_sphere_reversed": CornerSphere(o, 10.0, TopAbs_REVERSED); break;
            case "rotule": Rotule(o, S, 1, 3, 5, TopAbs_FORWARD, TopAbs_REVERSED, TopAbs_REVERSED); break;
            case "rotule_reversed": Rotule(o, S, 1, 3, 5, TopAbs_REVERSED, TopAbs_FORWARD, TopAbs_REVERSED); break;
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
    private static List<int> CylQLines(in TopoDS_Shape S) => CylQEdges(S, GeomAbs_Line);

    private static List<int> CylQCircles(in TopoDS_Shape S) => CylQEdges(S, GeomAbs_Circle);

    private static List<int> CylQEdges(in TopoDS_Shape S, GeomAbs_CurveType theType)
    {
        List<int> aRes = [];
        NCollection_IndexedMap<TopoDS_Shape> M = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, TopAbs_EDGE, M);
        for (int k = 1; k <= M.Extent(); k++)
        {
            TopoDS_Edge E = TopoDS.Edge(M.FindKey(k));
            BRepAdaptor_Curve C = new(E);
            FacesOf(S, E, out TopoDS_Face F1, out TopoDS_Face F2);
            bool aCylFace = new BRepAdaptor_Surface(F1).GetType() == GeomAbs_Cylinder
                            || (!F2.IsNull() && new BRepAdaptor_Surface(F2).GetType() == GeomAbs_Cylinder);
            if (!aCylFace || F2.IsNull())
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

    private static List<int> Circles(in TopoDS_Shape S) => EdgesOfType(S, GeomAbs_Circle);

    private static List<int> Lines(in TopoDS_Shape S) => EdgesOfType(S, GeomAbs_Line);

    private static List<int> EdgesOfType(in TopoDS_Shape S, GeomAbs_CurveType theType)
    {
        List<int> aRes = [];
        NCollection_IndexedMap<TopoDS_Shape> M = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, TopAbs_EDGE, M);
        for (int k = 1; k <= M.Extent(); k++)
        {
            if (new BRepAdaptor_Curve(TopoDS.Edge(M.FindKey(k))).GetType() == theType)
            {
                aRes.Add(k);
            }
        }
        return aRes;
    }

    private static void KPart(Out o, in TopoDS_Shape S, int k, int theKind, double a, double b,
                              ChFiDS_ChamfMode theMode, bool theSwap)
    {
        try
        {
            TopoDS_Edge E = EdgeOf(S, k);
            FacesOf(S, E, out TopoDS_Face F1, out TopoDS_Face F2);
            if (theSwap)
            {
                (F1, F2) = (F2, F1);
            }
            BRepAdaptor_Surface HS1 = new(F1);
            BRepAdaptor_Surface HS2 = new(F2);
            int aChoix = ChFi3d.ConcaveSide(HS1, HS2, E, out TopAbs_Orientation Or1, out TopAbs_Orientation Or2);
            o.L("concave " + aChoix + " " + theOri[(int)Or1] + " " + theOri[(int)Or2]);
            ChFiDS_Spine Sp;
            if (theKind == 0)
            {
                ChFiDS_FilSpine fs = new();
                fs.SetEdges(E);
                fs.Load();
                fs.SetRadius(a);
                Sp = fs;
            }
            else
            {
                ChFiDS_ChamfSpine cs = new();
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
            TopOpeBRepDS_DataStructure DStr = new();
            ChFiDS_SurfData Data = new();
            bool ok = ChFiKPart_ComputeData.Compute(DStr, Data, HS1, HS2, Or1, Or2, Sp, 1);
            o.L("ok " + (ok ? 1 : 0));
            o.SurfData(Data);
            o.DS(DStr);
        }
        catch (Standard_Failure e)
        {
            o.Exception(e);
        }
    }

    private static void CornerTorus(Out o, Geom_Surface S2)
    {
        try
        {
            GeomAdaptor_Surface HS1 = new(new Geom_Plane(new gp_Pln()));
            GeomAdaptor_Surface HS2 = new(S2);
            TopOpeBRepDS_DataStructure DStr = new();
            ChFiDS_SurfData Data = new();
            bool ok = ChFiKPart_ComputeData.ComputeCorner(DStr, Data, HS1, HS2, TopAbs_FORWARD, TopAbs_FORWARD,
                                                          TopAbs_FORWARD, TopAbs_REVERSED, 5.0, 20.0,
                                                          new gp_Pnt2d(15, 0), new gp_Pnt2d(0, 15), new gp_Pnt2d(0, 0),
                                                          new gp_Pnt2d(Math.PI / 2, 0));
            o.L("ok " + (ok ? 1 : 0));
            o.SurfData(Data);
            o.DS(DStr);
        }
        catch (Standard_Failure e)
        {
            o.Exception(e);
        }
    }

    private static void CornerSphere(Out o, double theRad, TopAbs_Orientation Or1)
    {
        try
        {
            GeomAdaptor_Surface HS1 = new(new Geom_Plane(new gp_Pln()));
            GeomAdaptor_Surface HS2 = new(new Geom_CylindricalSurface(new gp_Ax3(), 10.0));
            TopOpeBRepDS_DataStructure DStr = new();
            ChFiDS_SurfData Data = new();
            bool ok = ChFiKPart_ComputeData.ComputeCorner(DStr, Data, HS1, HS2, TopAbs_FORWARD, TopAbs_FORWARD, Or1,
                                                          TopAbs_FORWARD, theRad, new gp_Pnt2d(0.0, 0.0),
                                                          new gp_Pnt2d(0.0, 10.0), new gp_Pnt2d(Math.PI / 2, 10.0));
            o.L("ok " + (ok ? 1 : 0));
            o.SurfData(Data);
            o.DS(DStr);
        }
        catch (Standard_Failure e)
        {
            o.Exception(e);
        }
    }

    private static void Rotule(Out o, in TopoDS_Shape S, int fs, int f1, int f2, TopAbs_Orientation OS,
                               TopAbs_Orientation OS1, TopAbs_Orientation OS2)
    {
        try
        {
            TopoDS_Face F = FaceOf(S, fs);
            BRepAdaptor_Surface HS = new(F);
            BRepAdaptor_Surface H1 = new(FaceOf(S, f1));
            BRepAdaptor_Surface H2 = new(FaceOf(S, f2));
            TopOpeBRepDS_DataStructure DStr = new();
            ChFiDS_SurfData Data = new();
            bool ok = ChFiKPart_ComputeData.ComputeCorner(DStr, Data, HS, H1, H2, F.Orientation(), OS, OS1, OS2, 10.0);
            o.L("ok " + (ok ? 1 : 0));
            o.SurfData(Data);
            o.DS(DStr);
        }
        catch (Standard_Failure e)
        {
            o.Exception(e);
        }
    }

    private static void ComputePCurv(Out o)
    {
        gp_Pnt2d[,] aUV =
        {
            { new(1, 2), new(1, 5) }, { new(1, 5), new(1, 2) }, { new(1, 2), new(4, 2) }, { new(4, 2), new(1, 2) },
            { new(1, 2), new(3, 7) }
        };
        for (int i = 0; i < 5; i++)
        {
            for (int r = 0; r < 2; r++)
            {
                ChFi3d_ComputePCurv(aUV[i, 0], aUV[i, 1], out Geom2d_Curve C, 2.0, 5.0, r == 1);
                o.C2("pcurv", C);
            }
        }
        Geom_Surface aCyl = new Geom_CylindricalSurface(new gp_Ax3(), 3.0);
        Geom_Curve aC3d = aCyl.VIso(1.0);
        ChFi3d_ComputePCurv(aC3d, new gp_Pnt2d(0.5, 1.0), new gp_Pnt2d(2.0, 1.0), out Geom2d_Curve C2, aCyl, 0.5, 2.0,
                            1.0e-6, out double tolr, false);
        o.C2("pcurv", C2);
        o.Real("tolreached", tolr);
    }

    private static void BuildPCurve(Out o)
    {
        gp_Dir2d d1 = new(1, 1), d2 = new(1, -1);
        o.C2("bezier", ChFi3d_BuildPCurve(new gp_Pnt2d(0, 0), ref d1, new gp_Pnt2d(3, 1), ref d2, true));
        gp_Dir2d e1 = new(1, 1), e2 = new(1, -1);
        o.C2("bezier", ChFi3d_BuildPCurve(new gp_Pnt2d(0, 0), ref e1, new gp_Pnt2d(3, 1), ref e2, false));
        GeomAdaptor_Surface aCyl = new(new Geom_CylindricalSurface(new gp_Ax3(), 3.0));
        o.C2("bezier", ChFi3d_BuildPCurve(aCyl, new gp_Pnt2d(0.2, 0), new gp_Vec2d(1, 0.5), new gp_Pnt2d(1.5, 2),
                                          new gp_Vec2d(0.2, 1), true));
        o.C2("bezier", ChFi3d_BuildPCurve(aCyl, new gp_Pnt2d(0.2, 0), new gp_Vec(0, 1, 0.5), new gp_Pnt2d(1.5, 2),
                                          new gp_Vec(-1, 0, 1), true));
        GeomFill_Boundary aB = ChFi3d_mkbound(aCyl, out Geom2d_Curve aCurv, new gp_Pnt2d(0.2, 0), new gp_Pnt2d(1.5, 2),
                                              1.0e-4, 1.0e-2);
        o.C2("mkbound", aCurv);
        for (int i = 0; i <= 4; i++)
        {
            o.P3("value", aB.Value(0.25 * i));
        }
    }

    private static void ComputeArete(Out o)
    {
        Geom_Surface aCyl = new Geom_CylindricalSurface(new gp_Ax3(), 3.0);
        ChFiDS_CommonPoint P1 = new(), P2 = new();
        gp_Pnt2d[,] aUV = { { new(1, 0.5), new(1, 4) }, { new(2, 1), new(0.5, 1) }, { new(0.5, 0.5), new(2, 3) } };
        for (int i = 0; i < 3; i++)
        {
            Geom_Curve? C3d = null;
            Geom2d_Curve? Pc = null;
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
        GeomAdaptor_Surface aPlnZ = new(new Geom_Plane(new gp_Pln()));
        GeomAdaptor_Surface aPlnX = new(new Geom_Plane(new gp_Pln(new gp_Pnt(1, 0, 0), new gp_Dir(1, 0, 0))));
        GeomAdaptor_Surface aCylZ = new(new Geom_CylindricalSurface(new gp_Ax3(), 3.0), 0.0, 2 * aPi, -10.0, 10.0);
        GeomAdaptor_Surface aPlnZ2 = new(new Geom_Plane(new gp_Pln(new gp_Pnt(0, 0, 2), new gp_Dir(0, 0, 1))));
        GeomAdaptor_Surface aPlnY3 = new(new Geom_Plane(new gp_Pln(new gp_Pnt(0, 3, 0), new gp_Dir(0, 1, 0))));
        GeomAdaptor_Surface aSph =
            new(new Geom_SphericalSurface(new gp_Ax3(new gp_Pnt(0, 0, 1), new gp_Dir(0, 0, 1)), 4.0));
        List<(Adaptor3d_Surface S1, Adaptor3d_Surface S2, double[] d, double[] f)> aPairs = [];
        double u, v;
        {
            double[] d = new double[4], f = new double[4];
            ElSLib.Parameters(aPlnX.Plane(), new gp_Pnt(1, -2, 0), out u, out v);
            d[0] = 1;
            d[1] = -2;
            d[2] = u;
            d[3] = v;
            ElSLib.Parameters(aPlnX.Plane(), new gp_Pnt(1, 3, 0), out u, out v);
            f[0] = 1;
            f[1] = 3;
            f[2] = u;
            f[3] = v;
            aPairs.Add((aPlnZ, aPlnX, d, f));
        }
        {
            double[] d = new double[4], f = new double[4];
            gp_Pnt a = ElSLib.Value(0.5, 2.0, aCylZ.Cylinder()), b = ElSLib.Value(2.0, 2.0, aCylZ.Cylinder());
            ElSLib.Parameters(aPlnZ2.Plane(), a, out u, out v);
            d[0] = u;
            d[1] = v;
            d[2] = 0.5;
            d[3] = 2.0;
            ElSLib.Parameters(aPlnZ2.Plane(), b, out u, out v);
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
            ElSLib.Parameters(aPlnY3.Plane(), new gp_Pnt(0, 3, -1), out u, out v);
            d[2] = u;
            d[3] = v;
            f[0] = aPi / 2;
            f[1] = 4.0;
            ElSLib.Parameters(aPlnY3.Plane(), new gp_Pnt(0, 3, 4), out u, out v);
            f[2] = u;
            f[3] = v;
            aPairs.Add((aCylZ, aPlnY3, d, f));
        }
        {
            double[] d = new double[4], f = new double[4];
            double z = 1.0 + Math.Sqrt(7.0);
            d[0] = 0.3;
            d[1] = z;
            ElSLib.Parameters(aSph.Sphere(), ElSLib.Value(0.3, z, aCylZ.Cylinder()), out u, out v);
            d[2] = u;
            d[3] = v;
            f[0] = 1.8;
            f[1] = z;
            ElSLib.Parameters(aSph.Sphere(), ElSLib.Value(1.8, z, aCylZ.Cylinder()), out u, out v);
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
                Geom_Curve? C3d = null;
                Geom2d_Curve? Pc1 = null, Pc2 = null;
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
                catch (Standard_Failure e)
                {
                    o.Exception(e);
                }
            }
        }
    }

    private static void Misc(Out o)
    {
        double aPi = Math.PI;
        GeomAdaptor_Surface aCyl = new(new Geom_CylindricalSurface(new gp_Ax3(), 3.0), 0.0, 2 * aPi, -10.0, 10.0);
        GeomAdaptor_Curve aLin = new(new Geom_Line(new gp_Pnt(-10, 1, 2), new gp_Dir(1, 0, 0.1)), 0.0, 30.0);
        foreach (double wc in new[] { -2.0e100, 2.0e100, 15.0 })
        {
            gp_Pnt2d p2d = new();
            double w = wc;
            bool ok = ChFi3d_IntCS(aCyl, aLin, ref p2d, ref w);
            o.L("intcs " + (ok ? 1 : 0) + " " + Out.R(p2d.X()) + " " + Out.R(p2d.Y()) + " " + Out.R(w));
        }
        GeomAdaptor_Surface aPln = new(new Geom_Plane(new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0.2))));
        GeomAdaptor_Curve aCir = new(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 5.0), 0.0,
                                     2 * aPi);
        for (int s = 0; s < 2; s++)
        {
            double W = -1.0;
            bool ok = ChFi3d_InterPlaneEdge(aPln, aCir, ref W, s == 1, 1.0e-7);
            o.L("interplane " + (ok ? 1 : 0) + " " + Out.R(W));
        }
        ChFi3d_Parameters(new Geom_CylindricalSurface(new gp_Ax3(), 3.0), new gp_Pnt(0, 3, 7), out double u,
                          out double v);
        o.L("parameters " + Out.R(u) + " " + Out.R(v));
        ChFi3d_Parameters(new Geom_SphericalSurface(new gp_Ax3(), 3.0), new gp_Pnt(0, 0, 3), out u, out v);
        o.L("parameters " + Out.R(u) + " " + Out.R(v));
        ChFi3d_TrimCurve(new Geom_Circle(new gp_Ax2(), 5.0), new gp_Pnt(0, 5, 0), new gp_Pnt(-5, 0, 0),
                         out Geom_TrimmedCurve aTr);
        o.C3("trimmed", aTr);
        double wf = -1.0, wl = -1.0;
        Geom_Circle? aCs = ChFi3d_CircularSpine(ref wf, ref wl, new gp_Pnt(5, 0, 0), new gp_Vec(0, 1, 0),
                                                new gp_Pnt(0, 5, 0), new gp_Vec(-1, 0, 0), 2.0);
        o.C3("circularspine", aCs);
        o.L("w " + Out.R(wf) + " " + Out.R(wl));
        gp_Vec vd = new(0, 1, 0), vf = new(-1, 0, 0);
        o.C3("spine", ChFi3d_Spine(new gp_Pnt(5, 0, 0), ref vd, new gp_Pnt(0, 5, 0), ref vf, 2.0));
        o.V3("vd", vd);
        o.V3("vf", vf);
        o.Real("convtol", ChFi3d_ConvTol2dToTol3d(aCyl, 1.0e-5));
        Geom_Curve aC3 = new Geom_TrimmedCurve(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 1), new gp_Dir(0, 0, 1)),
                                                               3.0), 0.0, 2.0);
        Geom2d_Curve aPc1 = new Geom2d_Line(new gp_Pnt2d(0.0, 1.0), new gp_Dir2d(1.0, 0.0));
        GeomAdaptor_Surface aSph = new(new Geom_SphericalSurface(new gp_Ax3(), 3.2));
        Geom2d_Curve aPc2 = new Geom2d_Line(new gp_Pnt2d(0.0, 0.3), new gp_Dir2d(1.0, 0.0));
        o.Real("evaltol", ChFi3d_EvalTolReached(aCyl, aPc1, aSph, aPc2, aC3));
        GeomAdaptor_Surface aB = new(new Geom_CylindricalSurface(new gp_Ax3(), 3.0));
        ChFi3d_BoundSrf(aB, 0.5, 1.0, 0.0, 2.0, true);
        o.L("boundsrf " + Out.R(aB.FirstUParameter()) + " " + Out.R(aB.LastUParameter()) + " "
            + Out.R(aB.FirstVParameter()) + " " + Out.R(aB.LastVParameter()));
        GeomAdaptor_Surface aP = new(new Geom_Plane(new gp_Pln()));
        ChFi3d_BoundSrf(aP, 0.0, 1.0, 0.0, 4.0, true);
        o.L("boundsrf " + Out.R(aP.FirstUParameter()) + " " + Out.R(aP.LastUParameter()) + " "
            + Out.R(aP.FirstVParameter()) + " " + Out.R(aP.LastVParameter()));
        Geom_Curve aArc = new Geom_TrimmedCurve(new Geom_Circle(new gp_Ax2(), 5.0), 0.0, 2.0);
        o.C3("approxbyc2", ChFi3d_ApproxByC2(aArc));
        o.L("issmooth " + (ChFi3d_IsSmooth(aArc) ? 1 : 0) + " " + (ChFi3d_IsSmooth(ChFi3d_ApproxByC2(aArc)) ? 1 : 0));
    }

    private static void Topology(Out o, in TopoDS_Shape S)
    {
        ChFiDS_Map EF = new(), VE = new(), VF = new();
        EF.Fill(S, TopAbs_EDGE, TopAbs_FACE);
        VE.Fill(S, TopAbs_VERTEX, TopAbs_EDGE);
        VF.Fill(S, TopAbs_VERTEX, TopAbs_FACE);
        NCollection_IndexedMap<TopoDS_Shape> MV = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, TopAbs_VERTEX, MV);
        for (int i = 1; i <= MV.Extent(); i++)
        {
            TopoDS_Vertex V = TopoDS.Vertex(MV.FindKey(i));
            o.L("vertex " + i + " nbface " + ChFi3d_nbface(VF[V]) + " nbedges " + ChFi3d_NumberOfEdges(V, VE)
                + " nbsharp " + ChFi3d_NumberOfSharpEdges(V, VE, EF));
            TopoDS_Edge[] E = new TopoDS_Edge[3];
            int n = 0;
            foreach (TopoDS_Shape it in VE[V])
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
                    E[n++] = TopoDS.Edge(it);
                }
            }
            if (n == 3)
            {
                o.L("edgestate " + (int)ChFi3d_EdgeState(E, EF) + " angle " + Out.R(ChFi3d_AngleEdge(V, E[0], E[1])));
            }
        }
        NCollection_IndexedMap<TopoDS_Shape> ME = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, TopAbs_EDGE, ME);
        for (int i = 1; i <= ME.Extent(); i++)
        {
            TopoDS_Edge E = TopoDS.Edge(ME.FindKey(i));
            ChFi3d_conexfaces(E, out TopoDS_Face F1, out TopoDS_Face F2, EF);
            o.L("edge " + i + " conti " + (int)ChFi3d_evalconti(E, F1, F2) + " pseudoseam "
                + (ChFi3d_IsPseudoSeam(E, F1) ? 1 : 0));
        }
    }

    private static void ElSpine(Out o, in TopoDS_Shape S, int k, double WF, double WL)
    {
        try
        {
            ChFiDS_FilSpine fs = new();
            fs.SetEdges(EdgeOf(S, k));
            fs.Load();
            fs.SetRadius(1.0);
            ChFiDS_Spine Sp = fs;
            ChFiDS_ElSpine HES = new();
            HES.FirstParameter(WF);
            HES.LastParameter(WL);
            Sp.D1(WF, out gp_Pnt P, out gp_Vec V);
            HES.SetFirstPointAndTgt(P, V);
            Sp.D1(WL, out P, out V);
            HES.SetLastPointAndTgt(P, V);
            ChFi3d_PerformElSpine(HES, Sp, GeomAbs_C1, 1.0e-4, false);
            o.L("range " + Out.R(HES.FirstParameter()) + " " + Out.R(HES.LastParameter()) + " type "
                + (int)HES.GetType());
            for (int i = 0; i <= 4; i++)
            {
                o.P3("value", HES.Value(HES.FirstParameter() + 0.25 * i * (HES.LastParameter() - HES.FirstParameter())));
            }
            if (HES.GetType() == GeomAbs_BSplineCurve)
            {
                o.C3("curve", HES.BSpline());
            }
        }
        catch (Standard_Failure e)
        {
            o.Exception(e);
        }
    }
}
