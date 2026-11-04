// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.IO.Compression;
using System.Text;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The port's side of tools/brepfillrec (unit 5S1): the same cases, built by the same generator in the same order, and
/// written in the same text, so that each case's result compares with native OCCT's recording as a string. Kept line
/// for line with tools/brepfillrec/main.cpp.
/// </summary>
internal static class BRepFillRecordingCases
{
    /// <summary>The kinds of case and their counts, as in main.cpp.</summary>
    public static readonly (string Name, int Count)[] Kinds =
    [
        ("edge3d", 80), ("acr", 20), ("guideplan", 20), ("edgeonsurf", 40), ("shapelaw", 80), ("nsections", 40),
        ("removeloc", 60), ("section", 40), ("placement", 80),
    ];

    private static readonly Lazy<Dictionary<string, string>> theNative = new(LoadNative);

    /// <summary>Native OCCT's text of each case, keyed "kind seed".</summary>
    public static Dictionary<string, string> Native => theNative.Value;

    private static Dictionary<string, string> LoadNative()
    {
        using Stream aRes = typeof(BRepFillRecordingCases).Assembly.GetManifestResourceStream("BRepFill.brepfillrec.txt.gz")
                            ?? throw new InvalidOperationException("BRepFill.brepfillrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        Dictionary<string, string> aCases = [];
        string? aKey = null;
        StringBuilder aText = new();
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            if (aLine.StartsWith("CASE ", StringComparison.Ordinal))
            {
                if (aKey is not null)
                {
                    aCases[aKey] = aText.ToString();
                }
                aKey = aLine[5..];
                aText.Clear();
            }
            aText.Append(aLine).Append('\n');
        }
        if (aKey is not null)
        {
            aCases[aKey] = aText.ToString();
        }
        return aCases;
    }

    /// <summary>The port's text of the case <paramref name="theSeed"/> of the kind <paramref name="theKind"/>.</summary>
    public static string Run(string theKind, int theSeed)
    {
        int aKindIndex = Array.FindIndex(Kinds, k => k.Name == theKind);
        Rng r = new((ulong)(aKindIndex * 1000 + theSeed));
        theOut.Clear();
        Line($"CASE {theKind} {theSeed}");
        Guard(() =>
        {
            switch (theKind)
            {
                case "edge3d": Edge3D(r); break;
                case "acr": Guided(r, true); break;
                case "guideplan": Guided(r, false); break;
                case "edgeonsurf": EdgeOnSurf(r); break;
                case "shapelaw": ShapeLaw(r); break;
                case "nsections": NSections(r); break;
                case "removeloc": RemoveLoc(r); break;
                case "section": Section(r); break;
                case "placement": Placement(r); break;
            }
        });
        return theOut.ToString();
    }

    // ----------------------------------------------------------------------------------------------- text

    private static string Hex(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x16");

    private static string Hex(Point3D theP) => Hex(theP.X) + "," + Hex(theP.Y) + "," + Hex(theP.Z);

    private static string Hex(Vector3D theV) => Hex(theV.X) + "," + Hex(theV.Y) + "," + Hex(theV.Z);

    private static string Hex(Matrix3x3 theM)
    {
        StringBuilder aS = new();
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aS.Append(aS.Length == 0 ? "" : ",").Append(Hex(theM[i, j]));
            }
        }
        return aS.ToString();
    }

    private static string Hex(Transform3D theT)
    {
        StringBuilder aS = new();
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aS.Append(aS.Length == 0 ? "" : ",").Append(Hex(theT.Value(i, j)));
            }
        }
        return aS.ToString();
    }

    private static string Int(int theValue) => theValue.ToString(System.Globalization.CultureInfo.InvariantCulture);

    private static string Int(bool theValue) => theValue ? "1" : "0";

    /// <summary>The generator of main.cpp: a 64-bit LCG; U() has 32 significant bits.</summary>
    private sealed class Rng(ulong theSeed)
    {
        private ulong s = unchecked(theSeed * 2862933555777941757UL + 3037000493UL);

        public ulong Next()
        {
            s = unchecked(s * 6364136223846793005UL + 1442695040888963407UL);
            return s;
        }

        public double U() => (double)(Next() >> 32) / 4294967296.0;

        public double R(double a, double b) => a + (b - a) * U();

        public int I(int a, int b) => a + (int)((Next() >> 33) % (ulong)(b - a + 1));
    }

    [ThreadStatic] private static StringBuilder? theOutStorage;

    private static StringBuilder theOut => theOutStorage ??= new StringBuilder();

    private static void Line(string theLine) => theOut.Append(theLine).Append('\n');

    /// <summary>The BRep text of a shape with the Free flag masked, as main.cpp's Text.</summary>
    private static void Text(string theTag, in Shape theShape)
    {
        StringWriter aWriter = new();
        BrepTools.Write(theShape, aWriter);
        Line(theTag);
        string[] aLines = aWriter.ToString().Split('\n');
        int aCount = aLines.Length;
        if (aCount > 0 && aLines[aCount - 1].Length == 0)
        {
            aCount--;
        }
        for (int i = 0; i < aCount; i++)
        {
            string aLine = aLines[i].TrimEnd('\r');
            if (aLine.Length == 7 && aLine.All(c => c is '0' or '1'))
            {
                aLine = "x" + aLine[1..];
            }
            Line(" " + aLine);
        }
    }

    private static void Guard(Action theBody)
    {
        try
        {
            theBody();
        }
        catch (Exception anExc)
        {
            Line("EXC " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(anExc).GetType()));
        }
    }

    // ----------------------------------------------------------------------------------------------- inputs

    private static Point3D Pnt(Rng r, double theScale)
    {
        double x = r.R(-theScale, theScale);
        double y = r.R(-theScale, theScale);
        double z = r.R(-theScale, theScale);
        return new Point3D(x, y, z);
    }

    private static Direction3D Dir(Rng r)
    {
        for (;;)
        {
            double x = r.R(-1, 1);
            double y = r.R(-1, 1);
            double z = r.R(-1, 1);
            if (x * x + y * y + z * z > 0.01)
            {
                return new Direction3D(x, y, z);
            }
        }
    }

    private static Transform3D Trsf(Rng r, double theScale)
    {
        Transform3D aRot = new();
        double anAngle = r.R(-3, 3);
        aRot = aRot.WithRotation(new Axis3D(new Point3D(0, 0, 0), Dir(r)), anAngle);
        Transform3D aTr = new();
        Point3D aP = Pnt(r, theScale);
        aTr = aTr.WithTranslation(new Vector3D(aP.XYZ()));
        return aTr * aRot;
    }

    private static BSplineCurve3D Cubic(Rng r, Point3D P1, Point3D P2, double theWiggle)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = P1;
        aPoles[4] = P2;
        for (int i = 2; i <= 3; i++)
        {
            double t = (i - 1) / 3.0;
            Point3D aB = new(P1.XYZ() * (1 - t) + P2.XYZ() * t);
            double dx = r.R(-theWiggle, theWiggle);
            double dy = r.R(-theWiggle, theWiggle);
            double dz = r.R(-theWiggle, theWiggle);
            aPoles[i] = new Point3D(aB.X + dx, aB.Y + dy, aB.Z + dz);
        }
        NCollection_Array1<double> aK = new(1, 2);
        aK[1] = 0;
        aK[2] = 1;
        NCollection_Array1<int> aM = new(1, 2);
        aM[1] = 4;
        aM[2] = 4;
        return new BSplineCurve3D(aPoles, aK, aM, 3);
    }

    private static Vertex Vertex(Point3D theP) => new MakeVertex(theP).Vertex();

    private static Wire MakeWire(Rng r, int theKind, double theScale)
    {
        MakeWire aMW = new();
        if (theKind == 2)
        {
            int aNb = r.I(5, 8);
            NCollection_Array1<Point3D> aPoles = new(1, aNb);
            for (int i = 1; i <= aNb; i++)
            {
                double x = theScale * (i - 1) + r.R(-1, 1);
                double y = r.R(-theScale, theScale) * 0.5;
                double z = r.R(-theScale, theScale) * 0.5;
                aPoles[i] = new Point3D(x, y, z);
            }
            int aNbK = aNb - 2;
            NCollection_Array1<double> aK = new(1, aNbK);
            NCollection_Array1<int> aM = new(1, aNbK);
            for (int i = 1; i <= aNbK; i++)
            {
                aK[i] = i - 1;
                aM[i] = 1;
            }
            aM[1] = 4;
            aM[aNbK] = 4;
            BSplineCurve3D aC = new(aPoles, aK, aM, 3);
            int aNE2 = r.I(1, 4);
            double aL = aK[aNbK];
            List<double> aT = [0];
            for (int i = 1; i < aNE2; i++)
            {
                aT.Add(aL * i / aNE2 + r.R(-0.1, 0.1));
            }
            aT.Add(aL);
            List<Vertex> aV2 = [];
            foreach (double t in aT)
            {
                aV2.Add(Vertex(aC.Value(t)));
            }
            for (int i = 0; i < aNE2; i++)
            {
                Edge anE = new MakeEdge(aC, aV2[i], aV2[i + 1], aT[i], aT[i + 1]).Edge();
                aMW.Add(anE);
            }
            return aMW.Wire();
        }
        bool aClosed = theKind == 1;
        int aNE = aClosed ? r.I(3, 5) : r.I(1, 4);
        List<Point3D> aP = [];
        List<Vertex> aV = [];
        for (int i = 0; i < aNE; i++)
        {
            Point3D aQ = Pnt(r, theScale);
            aP.Add(new Point3D(aQ.X + 2 * theScale * i, aQ.Y, aQ.Z));
        }
        if (!aClosed)
        {
            Point3D aQ = Pnt(r, theScale);
            aP.Add(new Point3D(aQ.X + 2 * theScale * aNE, aQ.Y, aQ.Z));
        }
        foreach (Point3D aQ in aP)
        {
            aV.Add(Vertex(aQ));
        }
        for (int i = 0; i < aNE; i++)
        {
            int j = (i + 1) % aP.Count;
            int aTyp = r.I(0, 2);
            bool aRev = r.I(0, 3) == 0;
            int a = aRev ? j : i, b = aRev ? i : j;
            Curve3D? aC = null;
            if (aTyp == 1)
            {
                Point3D aM = new(aP[a].XYZ() * 0.5 + aP[b].XYZ() * 0.5);
                Point3D aQ = Pnt(r, theScale * 0.5);
                aC = new MakeArcOfCircle(aP[a], new Point3D(aM.X + aQ.X, aM.Y + aQ.Y, aM.Z + aQ.Z), aP[b]).Value();
            }
            else if (aTyp == 2)
            {
                aC = Cubic(r, aP[a], aP[b], theScale * 0.5);
            }
            Edge anE;
            if (aC is null)
            {
                anE = new MakeEdge(aV[a], aV[b]).Edge();
            }
            else
            {
                anE = new MakeEdge(aC, aV[a], aV[b]).Edge();
            }
            if (aRev)
            {
                anE.Reverse();
            }
            aMW.Add(anE);
        }
        return aMW.Wire();
    }

    private static Wire RandomWire(Rng r, int theMaxKind, double theS0, double theS1)
    {
        int aKind = r.I(0, theMaxKind);
        double aScale = r.R(theS0, theS1);
        return MakeWire(r, aKind, aScale);
    }

    private static Shape Box(Rng r)
    {
        double dx = r.R(1, 3);
        double dy = r.R(1, 3);
        double dz = r.R(1, 3);
        return new MakeBox(dx, dy, dz).Shape();
    }

    private static Shape Cylinder(Rng r)
    {
        double aR = r.R(1, 3);
        double aH = r.R(1, 3);
        return new ApiMakeCylinder(aR, aH).Shape();
    }

    private static SurfaceLocationLaw MakeTrihedron(Rng r, int theKind)
    {
        TrihedronLaw aT;
        switch (theKind)
        {
            case 0:
                aT = new Frenet();
                break;
            case 1:
                aT = new CorrectedFrenet();
                break;
            case 2:
                {
                    Direction3D aD = Dir(r);
                    Direction3D aN = new CoordinateSystem(new Point3D(0, 0, 0), aD).XDirection;
                    aT = new Fixed(new Vector3D(aD), new Vector3D(aN));
                    break;
                }
            default:
                aT = new ConstantBiNormal(Dir(r));
                break;
        }
        return new CurveAndTrihedron(aT);
    }

    // ----------------------------------------------------------------------------------------------- outputs

    private static void DumpLocationLaw(Rng r, LocationLaw theLaw)
    {
        int N = theLaw.LawCount;
        Line("LAW " + Int(N) + " " + Int(theLaw.IsClosed()) + " " + Int((int)theLaw.GetStatus()));
        // the abscissa before the lengths are known (OCCT's bug)
        Guard(() =>
        {
            theLaw.Law(N).GetDomain(out double f, out double l);
            Line("ABS0 " + Hex(theLaw.Abscissa(N, 0.5 * (f + l))));
        });
        for (int i = 0; i <= N; i++)
        {
            Guard(() => Line("G1 " + Int(i) + " " + Int(theLaw.IsG1(i)) + " " + Int(theLaw.IsG1(i, 1.0e-4, 1.0e-2))));
        }
        int NH = theLaw.NbHoles();
        Line("HOLES " + Int(NH));
        if (NH > 0)
        {
            NCollection_Array1<int> aH = new(1, NH);
            theLaw.Holes(aH);
            string aS = "H";
            for (int i = 1; i <= NH; i++)
            {
                aS += " " + Int(aH[i]);
            }
            Line(aS);
        }
        double aTotal = 0;
        for (int i = 1; i <= N; i++)
        {
            theLaw.CurvilinearBounds(i, out double f, out double l);
            aTotal = l;
            Line("CB " + Hex(f) + " " + Hex(l));
            Line("V " + Hex(Tool.Pnt(theLaw.Vertex(i))));
        }
        Line("V " + Hex(Tool.Pnt(theLaw.Vertex(N + 1))));
        for (int k = 0; k <= 6; k++)
        {
            double s = k == 6 ? aTotal * 1.1 : (k == 5 ? r.R(0, aTotal) : aTotal * k / 4.0);
            double U = 0;
            theLaw.Parameter(s, out int anInd, ref U);
            Line("PAR " + Int(anInd) + " " + Hex(U));
            if (anInd > 0)
            {
                Line("ABS " + Hex(theLaw.Abscissa(anInd, U)));
            }
            Shape aV = Vertex(Pnt(r, 1));
            Guard(() =>
            {
                theLaw.D0(s, ref aV);
                Line(aV.IsNull ? "D0 null" : "D0 " + Hex(Tool.Pnt(ShapeCasts.Vertex(aV))));
            });
        }
        for (int i = 1; i <= N; i++)
        {
            theLaw.Law(i).GetDomain(out double f, out double l);
            Matrix3x3 M = new();
            Vector3D V = new();
            theLaw.Law(i).D0(0.5 * (f + l), ref M, ref V);
            Line("LD0 " + Hex(M) + " " + Hex(V));
        }
        Vertex anIn = Vertex(Pnt(r, 1));
        void Perform(string theTag)
        {
            for (int i = 0; i <= N; i++)
            {
                for (int iloc = -1; iloc <= 1; iloc++)
                {
                    Guard(() =>
                    {
                        theLaw.PerformVertex(i, anIn, 1.0e-7, out Vertex anOut, iloc);
                        Line(theTag + " " + Hex(Tool.Pnt(anOut)) + " " + Hex(Tool.Tolerance(anOut)));
                    });
                }
            }
        }
        Perform("PV");
        Guard(() =>
        {
            theLaw.TransformInG0Law();
            Perform("PVG0");
        });
        Guard(() =>
        {
            theLaw.DeleteTransform();
            theLaw.TransformInCompatibleLaw(0.01);
            Perform("PVC");
        });
        theLaw.DeleteTransform();
        Line("HOLES2 " + Int(theLaw.NbHoles(1.0e-4)));
    }

    private static void DumpSectionLaw(SectionLaw theLaw, Wire theWire)
    {
        Line("SEC " + Int(theLaw.LawCount) + " " + Int(theLaw.IsUClosed) + " " + Int(theLaw.IsVClosed) + " "
             + Int(theLaw.IsConstant) + " " + Int(theLaw.IsVertex) + " " + Int(theLaw.IsDone));
        if (!theLaw.IsDone)
        {
            return;
        }
        int N = theLaw.LawCount;
        double VF = 0, VL = 1;
        for (int i = 1; i <= N; i++)
        {
            SurfaceSectionLaw? aL = theLaw.Law(i);
            if (aL is null)
            {
                Line("NULLLAW");
                continue;
            }
            aL.GetDomain(out VF, out VL);
            aL.SectionShape(out int NbP, out int NbK, out int Deg);
            Line("SS " + Int(NbP) + " " + Int(NbK) + " " + Int(Deg) + " " + Int(aL.IsRational) + " "
                 + Int(aL.IsUPeriodic) + " " + Hex(VF) + " " + Hex(VL));
            NCollection_Array1<double> aK = new(1, NbK);
            aL.Knots(aK);
            NCollection_Array1<int> aM = new(1, NbK);
            aL.Mults(aM);
            string aS = "K";
            for (int k = 1; k <= NbK; k++)
            {
                aS += " " + Hex(aK[k]) + "/" + Int(aM[k]);
            }
            Line(aS);
            for (int k = 0; k <= 2; k++)
            {
                NCollection_Array1<Point3D> aP = new(1, NbP);
                NCollection_Array1<double> aW = new(1, NbP);
                bool aOk = aL.D0(VF + (VL - VF) * k / 2.0, aP, aW);
                StringBuilder aT = new("D0 " + Int(aOk));
                for (int j = 1; j <= NbP; j++)
                {
                    aT.Append(' ').Append(Hex(aP[j])).Append(',').Append(Hex(aW[j]));
                }
                Line(aT.ToString());
            }
        }
        double aMid = 0.5 * (VF + VL);
        if (!theLaw.IsVertex)
        {
            for (int i = 0; i <= N; i++)
            {
                Guard(() => Line("VT " + Hex(theLaw.VertexTol(i, aMid))));
                Guard(() => Line("CONT " + Int((int)theLaw.Continuity(i, 1.0e-4))));
            }
            for (int i = 1; i <= N + 1; i++)
            {
                Guard(() => Line("VX " + Hex(Tool.Pnt(theLaw.Vertex(i, aMid)))));
            }
            for (Explorer anExp = new(theWire, ShapeType.Edge); anExp.More(); anExp.Next())
            {
                Shape anE = anExp.Current();
                Guard(() => Line("IDX " + Int(theLaw.IndexOfEdge(anE))));
            }
        }
        Guard(() =>
        {
            SurfaceSectionLaw? aC = theLaw.ConcatenedLaw();
            if (aC is null)
            {
                Line("CONCAT null");
                return;
            }
            aC.SectionShape(out int NbP, out int NbK, out int Deg);
            aC.GetDomain(out double f, out double l);
            NCollection_Array1<Point3D> aP = new(1, NbP);
            NCollection_Array1<double> aW = new(1, NbP);
            aC.D0(0.5 * (f + l), aP, aW);
            StringBuilder aT = new("CONCAT " + Int(NbP) + " " + Int(NbK) + " " + Int(Deg));
            for (int j = 1; j <= NbP; j++)
            {
                aT.Append(' ').Append(Hex(aP[j])).Append(',').Append(Hex(aW[j]));
            }
            Line(aT.ToString());
        });
        Guard(() =>
        {
            Shape aS = new();
            theLaw.D0(aMid, ref aS);
            Text("SD0", aS);
        });
    }

    // ----------------------------------------------------------------------------------------------- cases

    private static void Edge3D(Rng r)
    {
        Wire aW = RandomWire(r, 2, 1, 5);
        int aKind = r.I(0, 3);
        Line("TRI " + Int(aKind));
        SurfaceLocationLaw aL = MakeTrihedron(r, aKind);
        Guard(() =>
        {
            LocationLaw aLaw = new Edge3DLaw(aW, aL);
            DumpLocationLaw(r, aLaw);
        });
    }

    private static void Guided(Rng r, bool theACR)
    {
        Wire aW = RandomWire(r, 2, 1, 5);
        Transform3D aT = new();
        double tx = r.R(-1, 1);
        double ty = r.R(2, 4);
        double tz = r.R(-1, 1);
        aT = aT.WithTranslation(new Vector3D(tx, ty, tz));
        Wire aG = ShapeCasts.Wire(new Transform(aW, aT, true).Shape());
        Guard(() =>
        {
            WireCurveAdaptor aGuide = new(aG);
            LocationLaw aLaw;
            if (theACR)
            {
                GuideTrihedronAC aTLaw = new(aGuide);
                LocationGuide aLoc = new(aTLaw);
                aLaw = new AcrLaw(aW, aLoc);
            }
            else
            {
                GuideTrihedronPlan aTLaw = new(aGuide);
                LocationGuide aLoc = new(aTLaw);
                aLaw = new Edge3DLaw(aW, aLoc);
            }
            DumpLocationLaw(r, aLaw);
        });
    }

    private static void EdgeOnSurf(Rng r)
    {
        int aKind = r.I(0, 2);
        Surface aS;
        double u0 = 0, u1 = 1, v0 = 0, v1 = 1;
        if (aKind == 0)
        {
            Point3D aO = Pnt(r, 2);
            Direction3D aD = Dir(r);
            aS = new PlaneSurface(new CoordinateSystem(aO, aD));
            u1 = v1 = 10;
        }
        else if (aKind == 1)
        {
            Point3D aO = Pnt(r, 2);
            Direction3D aD = Dir(r);
            double aR = r.R(2, 5);
            aS = new CylindricalSurface(new CoordinateSystem(aO, aD), aR);
            u1 = 6;
            v1 = 10;
        }
        else
        {
            NCollection_Array2<Point3D> aP = new(1, 4, 1, 4);
            for (int i = 1; i <= 4; i++)
            {
                for (int j = 1; j <= 4; j++)
                {
                    double z = r.R(-2, 2);
                    aP[i, j] = new Point3D(3.0 * i, 3.0 * j, z);
                }
            }
            NCollection_Array1<double> aK = new(1, 2);
            aK[1] = 0;
            aK[2] = 1;
            NCollection_Array1<int> aM = new(1, 2);
            aM[1] = 4;
            aM[2] = 4;
            aS = new BSplineSurface(aP, aK, aK, aM, aM, 3, 3);
        }
        Face aF = new MakeFace(aS, u0, u1, v0, v1, 1.0e-7).Face();
        int aNE = r.I(1, 4);
        List<Point2D> aUV = [];
        for (int i = 0; i <= aNE; i++)
        {
            double u = u0 + (u1 - u0) * (0.1 + 0.8 * i / aNE);
            double v = v0 + (v1 - v0) * r.R(0.1, 0.9);
            aUV.Add(new Point2D(u, v));
        }
        // the vertices and edges stay unfrozen: BuildCurves3d writes into them, as natively
        BrepBuilder B = new();
        List<Vertex> aV = [];
        foreach (Point2D aQ in aUV)
        {
            Vertex aVx = new();
            B.MakeVertex(ref aVx, aS.Value(aQ.X, aQ.Y), global::Kerf.Engine.Foundation.Precision.Confusion());
            aV.Add(aVx);
        }
        MakeWire aMW = new();
        bool aOff = r.I(0, 4) == 0;
        for (int i = 0; i < aNE; i++)
        {
            Vector2D aD = new(aUV[i], aUV[i + 1]);
            LineCurve2D aL = new(aUV[i], new Direction2D(aD));
            Edge anE;
            if (aOff && i == aNE - 1)
            {
                anE = new MakeEdge(aV[i], aV[i + 1]).InternalEdge();
            }
            else
            {
                anE = new MakeEdge(aL, aS, aV[i], aV[i + 1], 0, aD.Magnitude).InternalEdge();
                BrepLibrary.BuildCurves3d(anE);
            }
            aMW.Add(anE);
        }
        Wire aW = aMW.Wire();
        Guard(() =>
        {
            EdgeOnSurfLaw aLaw = new(aW, aF);
            Line("HAS " + Int(aLaw.HasResult));
            if (aLaw.HasResult)
            {
                DumpLocationLaw(r, aLaw);
            }
        });
    }

    private static void ShapeLaw(Rng r)
    {
        int aKind = r.I(0, 3);
        Line("KIND " + Int(aKind));
        if (aKind == 0)
        {
            Vertex aV = Vertex(Pnt(r, 5));
            Guard(() => DumpSectionLaw(new ShapeLaw(aV), new Wire()));
            return;
        }
        Wire aW = RandomWire(r, 2, 0.5, 2);
        if (aKind == 1)
        {
            Guard(() => DumpSectionLaw(new ShapeLaw(aW), aW));
        }
        else
        {
            Linear aL = new();
            double a = r.R(0.5, 2);
            double b = r.R(0.5, 2);
            aL.Set(0, a, 1, b);
            Guard(() => DumpSectionLaw(new ShapeLaw(aW, aL), aW));
        }
    }

    private static Wire PunctualWire(Point3D theP)
    {
        Vertex aV = Vertex(theP);
        BrepBuilder BB = new();
        Edge DegEdge = new();
        BB.MakeEdge(ref DegEdge);
        BB.Add(DegEdge, aV.Oriented(Forward));
        BB.Add(DegEdge, aV.Oriented(Reversed));
        BB.Degenerated(DegEdge, true);
        Wire aW = new();
        BB.MakeWire(ref aW);
        BB.Add(aW, DegEdge);
        aW.Closed(true);
        return aW;
    }

    private static void NSections(Rng r)
    {
        Wire aW0 = RandomWire(r, 1, 0.5, 2);
        int aNb = r.I(2, 4);
        NCollection_Sequence<Shape> aS = new();
        NCollection_Sequence<Transform3D> aT = new();
        NCollection_Sequence<double> aP = new();
        bool aFirstPoint = r.I(0, 3) == 0;
        bool aLastPoint = r.I(0, 3) == 0;
        bool aLoop = !aFirstPoint && !aLastPoint && r.I(0, 4) == 0;
        double aPar = 0;
        for (int i = 1; i <= aNb; i++)
        {
            Transform3D aTr = new();
            double tx = r.R(-1, 1);
            double ty = r.R(-1, 1);
            aTr = aTr.WithTranslation(new Vector3D(tx, ty, 5.0 * i));
            Transform3D aRot = new Transform3D().WithRotation(new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), r.R(-0.5, 0.5));
            aTr = aTr * aRot;
            if ((i == 1 && aFirstPoint) || (i == aNb && aLastPoint))
            {
                double px = r.R(-1, 1);
                double py = r.R(-1, 1);
                aS.Append(PunctualWire(new Point3D(px, py, 5.0 * i)));
            }
            else if (i == aNb && aLoop)
            {
                aS.Append(aS[1]);
            }
            else
            {
                aS.Append(new Transform(aW0, aTr, true).Shape());
            }
            aT.Append(aTr);
            aP.Append(aPar);
            aPar += r.R(0.5, 2);
        }
        Wire aRef = ShapeCasts.Wire(aS[aFirstPoint ? 2 : 1]);
        if (r.I(0, 1) == 0)
        {
            Line("NS1");
            Guard(() => DumpSectionLaw(new NSections(aS), aRef));
        }
        else
        {
            Line("NS2");
            Guard(() => DumpSectionLaw(new NSections(aS, aT, aP, aP[1], aP[aNb]), aRef));
        }
    }

    private static Shape Located(Rng r, in Shape theShape) => theShape.Moved(new Placement(Trsf(r, 3)));

    private static void RemoveLoc(Rng r)
    {
        int aKind = r.I(0, 3);
        Shape aShape;
        if (aKind == 0)
        {
            aShape = Located(r, Box(r));
        }
        else if (aKind == 1)
        {
            aShape = Located(r, Cylinder(r));
        }
        else if (aKind == 2)
        {
            aShape = Located(r, RandomWire(r, 2, 2, 2));
        }
        else
        {
            BrepBuilder B = new();
            Compound C = new();
            B.MakeCompound(ref C);
            B.Add(C, Located(r, Box(r)));
            B.Add(C, Located(r, Cylinder(r)));
            B.Add(C, MakeWire(r, 0, 2));
            aShape = Located(r, C);
        }
        int aLevel = r.I(0, 4);
        ShapeType aL = aLevel == 0 ? ShapeType.Shape : (ShapeType)(aLevel - 1);
        Line("LEVEL " + Int(aLevel));
        Guard(() =>
        {
            RemoveLocations aRL = new();
            aRL.SetRemoveLevel(aL);
            Line("REMOVE " + Int(aRL.Remove(aShape)) + " " + Int((int)aRL.RemoveLevel()) + " "
                 + Int(aRL.GetModifiedShapesMap().Count));
            Text("RESULT", aRL.GetResult());
        });
    }

    private static void Section(Rng r)
    {
        int aKind = r.I(0, 1);
        Shape aProfile;
        if (aKind == 0)
        {
            aProfile = Located(r, Vertex(Pnt(r, 2)));
        }
        else
        {
            aProfile = Located(r, RandomWire(r, 2, 1, 1));
        }
        Vertex aV = Vertex(Pnt(r, 2));
        Guard(() =>
        {
            bool aContact = r.I(0, 1) == 1;
            bool aCorrection = r.I(0, 1) == 1;
            FillingSection aSec = new(aProfile, aV, aContact, aCorrection);
            Line("SECT " + Int(aSec.IsPunctual) + " " + Int(aSec.WithContact()) + " " + Int(aSec.WithCorrection()));
            Text("WIRE", aSec.Wire());
            IndexedSet<Shape> aMap = new(ShapeHasher.Instance);
            Exploration.MapShapes(aSec.Wire(), aMap);
            for (Explorer anExp = new(aProfile, ShapeType.Edge); anExp.More(); anExp.Next())
            {
                Line("MOD " + Int(aMap.IndexOf(aSec.ModifiedShape(anExp.Current())) + 1));
            }
            for (Explorer anExp = new(aProfile, global::Kerf.Topology.ShapeType.Vertex); anExp.More(); anExp.Next())
            {
                Line("MOD " + Int(aMap.IndexOf(aSec.ModifiedShape(anExp.Current())) + 1));
            }
            Line("MOD " + Int(aMap.IndexOf(aSec.ModifiedShape(aProfile)) + 1));
        });
    }

    private static void Placement(Rng r)
    {
        Wire aW = RandomWire(r, 2, 1, 5);
        int aKind = r.I(0, 1);
        SurfaceLocationLaw aL = MakeTrihedron(r, aKind);
        int aSecKind = r.I(0, 1);
        // a section near a point of the spine
        WireCurveAdaptor aC = new(aW);
        double t = r.R(aC.FirstParameter(), aC.LastParameter());
        Point3D aQ = aC.Value(t);
        double ox = r.R(-0.5, 0.5);
        double oy = r.R(-0.5, 0.5);
        double oz = r.R(-0.5, 0.5);
        Point3D aO = new(aQ.X + ox, aQ.Y + oy, aQ.Z + oz);
        Shape aSec;
        if (aSecKind == 0)
        {
            aSec = Vertex(aO);
        }
        else
        {
            Transform3D aT = new Transform3D().WithTranslation(new Vector3D(aO.XYZ()));
            aSec = new Transform(RandomWire(r, 1, 0.3, 0.3), aT, true).Shape();
        }
        bool aContact = r.I(0, 1) == 1;
        bool aCorrection = r.I(0, 1) == 1;
        int aOnVertex = r.I(0, 2);
        Line("PL " + Int(aKind) + " " + Int(aSecKind) + " " + Int(aContact) + " " + Int(aCorrection) + " " + Int(aOnVertex));
        Guard(() =>
        {
            LocationLaw aLaw = new Edge3DLaw(aW, aL);
            if (aOnVertex == 0)
            {
                SectionPlacement aP = new(aLaw, aSec, aContact, aCorrection);
                Line("TRSF " + Hex(aP.Transformation()));
                Line("ABSC " + Hex(aP.AbscissaOnPath()));
            }
            else
            {
                int anIdx = r.I(1, aLaw.LawCount + 1);
                Vertex aV = aLaw.Vertex(anIdx);
                SectionPlacement aP = new(aLaw, aSec, aV, aContact, aCorrection);
                Line("TRSF " + Hex(aP.Transformation()));
                Line("ABSC " + Hex(aP.AbscissaOnPath()));
            }
        });
    }
}
