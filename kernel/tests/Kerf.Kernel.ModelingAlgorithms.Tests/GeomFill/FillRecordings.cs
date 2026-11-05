// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using System.Globalization;
using System.IO.Compression;
using System.Text;
using static Kerf.Engine.Construction.Surfaces.FillingStyle;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Native OCCT's GeomFill fillings recorded by tools/fillrec (unit 5L3): GeomFill_BSplineCurves and BezierCurves on
/// two, three and four curves in each style, GeomFill_Coons, Curved and Stretch, and GeomFill_ConstrainedFilling with
/// its boundaries, algorithmic patch and tangency fields. Each case's inputs are round-trip decimals and native's
/// results named tokens, every double as its 64 bits in hex (pole grids as an FNV-1a digest of their bits).
/// <see cref="Run"/> replays a case on the port and writes its results in the same format.
/// </summary>
internal static class FillRecordings
{
    public sealed record Case(string Kind, double[] Inputs, string[] Native);

    private static readonly Lazy<List<Case>> theCases = new(Load);

    public static List<Case> Cases => theCases.Value;

    private static List<Case> Load()
    {
        using Stream aRes = typeof(FillRecordings).Assembly.GetManifestResourceStream("GeomFill.fillrec.txt.gz")
                            ?? throw new InvalidOperationException("GeomFill.fillrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        List<Case> aCases = [];
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            int aSemi = aLine.IndexOf(';');
            string[] aIn = aLine[..aSemi].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            double[] aValues = aIn.Skip(1).Select(s => double.Parse(s, CultureInfo.InvariantCulture)).ToArray();
            string[] aOut = aLine[(aSemi + 1)..].Split(' ', StringSplitOptions.RemoveEmptyEntries);
            aCases.Add(new Case(aIn[0], aValues, aOut));
        }
        return aCases;
    }

    //=============================================================================================
    // Output, as fillrec writes it

    private sealed class Writer
    {
        public readonly List<string> Tokens = [];

        public void Put(string theTok) => Tokens.Add(theTok);

        public void Tok(string theName, Func<string> theF)
        {
            string aVal;
            try
            {
                aVal = theF();
            }
            catch (Exception anExc) when (Refusal.IsFailure(anExc))
            {
                aVal = ExcName(Refusal.Of(anExc)!);
            }
            Put(theName + ":" + aVal);
        }
    }

    public static string ExcName(FailureException theF) => "!" + global::Kerf.Engine.Foundation.OcctTypeNames.Of(theF.GetType());

    public static string Hex(double theValue) =>
        ((ulong)BitConverter.DoubleToInt64Bits(theValue)).ToString("x", CultureInfo.InvariantCulture);

    public static string Hex(Xyz theP) => Hex(theP.X()) + "," + Hex(theP.Y()) + "," + Hex(theP.Z());

    public static string Hex(Point3D theP) => Hex(theP.XYZ());

    public static string Hex(Vector3D theV) => Hex(theV.XYZ());

    private static string Int(int theValue) => theValue.ToString(CultureInfo.InvariantCulture);

    private static string Join(string theA, string theB) => theA.Length == 0 ? theB : theA + "," + theB;

    // FNV-1a over the 64 bits of each value
    private sealed class Digest
    {
        private ulong myH = 14695981039346656037UL;
        private int myN;

        public void Add(double theValue)
        {
            ulong aBits = (ulong)BitConverter.DoubleToInt64Bits(theValue);
            for (int i = 0; i < 8; i++)
            {
                myH ^= (aBits >> (8 * i)) & 0xff;
                myH *= 1099511628211UL;
            }
            myN++;
        }

        public void Add(Point3D theP)
        {
            Add(theP.X);
            Add(theP.Y);
            Add(theP.Z);
        }

        public string Str() => Int(myN) + "," + myH.ToString("x", CultureInfo.InvariantCulture);
    }

    private static void PutBSplineSurface(Writer theW, BSplineSurface? theS)
    {
        if (theS is null)
        {
            theW.Put("srf:null");
            return;
        }
        theW.Put("sdeg:" + Int(theS.UDegree) + "," + Int(theS.VDegree) + "," + Int(theS.UPoleCount) + ","
                 + Int(theS.VPoleCount) + "," + (theS.IsURational ? "1" : "0") + (theS.IsVRational ? "1" : "0"));
        Digest aP = new(), aW = new();
        for (int i = 1; i <= theS.UPoleCount; i++)
        {
            for (int j = 1; j <= theS.VPoleCount; j++)
            {
                aP.Add(theS.Pole(i, j));
                aW.Add(theS.Weight(i, j));
            }
        }
        theW.Put("spol:" + aP.Str());
        theW.Put("swt:" + aW.Str());
        string aU = "", aV = "";
        for (int i = 1; i <= theS.UKnotCount; i++)
        {
            aU = Join(aU, Hex(theS.UKnot(i)) + "," + Int(theS.UMultiplicity(i)));
        }
        for (int i = 1; i <= theS.VKnotCount; i++)
        {
            aV = Join(aV, Hex(theS.VKnot(i)) + "," + Int(theS.VMultiplicity(i)));
        }
        theW.Put("suk:" + aU);
        theW.Put("svk:" + aV);
        theS.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        string aD = "";
        for (int i = 0; i <= 2; i++)
        {
            for (int j = 0; j <= 2; j++)
            {
                theS.D1(u1 + (u2 - u1) * i / 2.0, v1 + (v2 - v1) * j / 2.0, out Point3D P, out Vector3D DU, out Vector3D DV);
                aD = Join(aD, Hex(P) + "," + Hex(DU) + "," + Hex(DV));
            }
        }
        theW.Put("sd1:" + aD);
    }

    private static void PutBezierSurface(Writer theW, BezierSurface? theS)
    {
        if (theS is null)
        {
            theW.Put("srf:null");
            return;
        }
        theW.Put("sdeg:" + Int(theS.UDegree) + "," + Int(theS.VDegree) + "," + (theS.IsURational ? "1" : "0")
                 + (theS.IsVRational ? "1" : "0"));
        Digest aP = new(), aW = new();
        for (int i = 1; i <= theS.UPoleCount; i++)
        {
            for (int j = 1; j <= theS.VPoleCount; j++)
            {
                aP.Add(theS.Pole(i, j));
                aW.Add(theS.Weight(i, j));
            }
        }
        theW.Put("spol:" + aP.Str());
        theW.Put("swt:" + aW.Str());
        string aD = "";
        for (int i = 0; i <= 2; i++)
        {
            for (int j = 0; j <= 2; j++)
            {
                theS.D1(i / 2.0, j / 2.0, out Point3D P, out Vector3D DU, out Vector3D DV);
                aD = Join(aD, Hex(P) + "," + Hex(DU) + "," + Hex(DV));
            }
        }
        theW.Put("sd1:" + aD);
    }

    private static void PutFilling(Writer theW, Filling theF)
    {
        theW.Put("nb:" + Int(theF.UPoleCount) + "," + Int(theF.VPoleCount) + "," + (theF.IsRational() ? "1" : "0"));
        NCollection_Array2<Point3D> aP = new(1, theF.UPoleCount, 1, theF.VPoleCount);
        theF.Poles(aP);
        string aS = "";
        for (int i = aP.LowerRow(); i <= aP.UpperRow(); i++)
        {
            for (int j = aP.LowerCol(); j <= aP.UpperCol(); j++)
            {
                aS = Join(aS, Hex(aP[i, j]));
            }
        }
        theW.Put("pol:" + aS);
        if (theF.IsRational())
        {
            NCollection_Array2<double> aW = new(1, theF.UPoleCount, 1, theF.VPoleCount);
            theF.Weights(aW);
            string aT = "";
            for (int i = aW.LowerRow(); i <= aW.UpperRow(); i++)
            {
                for (int j = aW.LowerCol(); j <= aW.UpperCol(); j++)
                {
                    aT = Join(aT, Hex(aW[i, j]));
                }
            }
            theW.Put("wt:" + aT);
        }
    }

    //=============================================================================================
    // Inputs, read in the order fillrec generated them

    private sealed class Reader(double[] theIn)
    {
        private int myPos;

        public double D() => theIn[myPos++];

        public int I() => (int)theIn[myPos++];

        public bool B() => theIn[myPos++] != 0.0;

        public Point3D Pnt()
        {
            double x = D();
            double y = D();
            double z = D();
            return new Point3D(x, y, z);
        }

        public BSplineCurve3D BSpline()
        {
            int aDeg = I();
            bool aRat = B();
            int aNbP = I();
            NCollection_Array1<Point3D> aPoles = new(1, aNbP);
            NCollection_Array1<double> aW = new(1, aNbP);
            for (int i = 1; i <= aNbP; i++)
            {
                aPoles[i] = Pnt();
                if (aRat)
                {
                    aW[i] = D();
                }
            }
            int aNbK = I();
            NCollection_Array1<double> aKnots = new(1, aNbK);
            NCollection_Array1<int> aMults = new(1, aNbK);
            for (int i = 1; i <= aNbK; i++)
            {
                aKnots[i] = D();
                aMults[i] = I();
            }
            return aRat ? new BSplineCurve3D(aPoles, aW, aKnots, aMults, aDeg)
                        : new BSplineCurve3D(aPoles, aKnots, aMults, aDeg);
        }

        public BezierCurve3D Bezier()
        {
            bool aRat = B();
            int aNbP = I();
            NCollection_Array1<Point3D> aPoles = new(1, aNbP);
            NCollection_Array1<double> aW = new(1, aNbP);
            for (int i = 1; i <= aNbP; i++)
            {
                aPoles[i] = Pnt();
                if (aRat)
                {
                    aW[i] = D();
                }
            }
            return aRat ? new BezierCurve3D(aPoles, aW) : new BezierCurve3D(aPoles);
        }
    }

    private static readonly FillingStyle[] THE_STYLES =
        [FillingStyle.StretchStyle, FillingStyle.CoonsStyle, FillingStyle.CurvedStyle];

    /// <summary>Replays a case on the port; returns its tokens.</summary>
    public static List<string> Run(Case theCase)
    {
        Writer aW = new();
        Reader aR = new(theCase.Inputs);
        string aKind = theCase.Kind;
        if (aKind.StartsWith("BSC", StringComparison.Ordinal) || aKind.StartsWith("BZC", StringComparison.Ordinal))
        {
            CurvesCase(aW, aR, aKind.StartsWith("BSC", StringComparison.Ordinal), aKind[3] - '0');
        }
        else if (aKind == "CF")
        {
            ConstrainedCase(aW, aR);
        }
        else
        {
            FillingCase(aW, aR, aKind);
        }
        return aW.Tokens;
    }

    private static void CurvesCase(Writer theW, Reader theR, bool theBSpline, int theNbCurves)
    {
        int aStyle = theR.I();
        List<BSplineCurve3D> aBS = [];
        List<BezierCurve3D> aBZ = [];
        void Make()
        {
            if (theBSpline)
            {
                aBS.Add(theR.BSpline());
            }
            else
            {
                aBZ.Add(theR.Bezier());
            }
        }
        if (theNbCurves >= 3)
        {
            for (int k = 0; k < theNbCurves; k++)
            {
                theR.D(); // the direction drawn by fillrec
                Make();
            }
        }
        else
        {
            theR.D(); // the way the two curves share an end
            Make();
            Make();
        }
        FillingStyle aType = THE_STYLES[aStyle];
        try
        {
            if (theBSpline)
            {
                BSplineCurves aFill = new();
                if (theNbCurves == 4)
                {
                    aFill.Init(aBS[0], aBS[1], aBS[2], aBS[3], aType);
                }
                else if (theNbCurves == 3)
                {
                    aFill.Init(aBS[0], aBS[1], aBS[2], aType);
                }
                else
                {
                    aFill.Init(aBS[0], aBS[1], aType);
                }
                PutBSplineSurface(theW, aFill.Surface());
            }
            else
            {
                BezierCurves aFill = new();
                if (theNbCurves == 4)
                {
                    aFill.Init(aBZ[0], aBZ[1], aBZ[2], aBZ[3], aType);
                }
                else if (theNbCurves == 3)
                {
                    aFill.Init(aBZ[0], aBZ[1], aBZ[2], aType);
                }
                else
                {
                    aFill.Init(aBZ[0], aBZ[1], aType);
                }
                PutBezierSurface(theW, aFill.Surface());
            }
        }
        catch (Exception anExc) when (Refusal.IsFailure(anExc))
        {
            theW.Put("init:" + ExcName(Refusal.Of(anExc)!));
        }
    }

    private static void FillingCase(Writer theW, Reader theR, string theKind)
    {
        bool aRat = theR.B();
        int NU = theR.I();
        int NV = theR.I();
        bool aTwo = theKind == "CURVED2";
        int aNb = aTwo ? 2 : 4;
        NCollection_Array1<Point3D>[] P = [new(1, NU), new(1, NV), new(1, NU), new(1, NV)];
        NCollection_Array1<double>[] Wt = [new(1, NU), new(1, NV), new(1, NU), new(1, NV)];
        for (int k = 0; k < aNb; k++)
        {
            for (int i = P[k].Lower(); i <= P[k].Upper(); i++)
            {
                P[k][i] = theR.Pnt();
            }
        }
        if (aRat)
        {
            for (int k = 0; k < aNb; k++)
            {
                for (int i = Wt[k].Lower(); i <= Wt[k].Upper(); i++)
                {
                    Wt[k][i] = theR.D();
                }
            }
        }
        try
        {
            Filling F;
            switch (theKind)
            {
                case "COONS":
                    F = aRat ? new Coons(P[0], P[1], P[2], P[3], Wt[0], Wt[1], Wt[2], Wt[3])
                             : new Coons(P[0], P[1], P[2], P[3]);
                    break;
                case "STRETCH":
                    F = aRat ? new Stretch(P[0], P[1], P[2], P[3], Wt[0], Wt[1], Wt[2], Wt[3])
                             : new Stretch(P[0], P[1], P[2], P[3]);
                    break;
                case "CURVED":
                    F = aRat ? new Curved(P[0], P[1], P[2], P[3], Wt[0], Wt[1], Wt[2], Wt[3])
                             : new Curved(P[0], P[1], P[2], P[3]);
                    break;
                default:
                    F = aRat ? new Curved(P[0], P[1], Wt[0], Wt[1]) : new Curved(P[0], P[1]);
                    break;
            }
            PutFilling(theW, F);
        }
        catch (Exception anExc) when (Refusal.IsFailure(anExc))
        {
            theW.Put("init:" + ExcName(Refusal.Of(anExc)!));
        }
    }

    private static BSplineSurface MakeSurface(Reader theR)
    {
        int NU = theR.I();
        int NV = theR.I();
        NCollection_Array2<Point3D> aP = new(1, NU, 1, NV);
        for (int i = 1; i <= NU; i++)
        {
            for (int j = 1; j <= NV; j++)
            {
                aP[i, j] = theR.Pnt();
            }
        }
        static void Knots(int theNb, out NCollection_Array1<double> theK, out NCollection_Array1<int> theM)
        {
            int aNbK = theNb - 2;
            theK = new NCollection_Array1<double>(1, aNbK);
            theM = new NCollection_Array1<int>(1, aNbK);
            for (int i = 1; i <= aNbK; i++)
            {
                theK[i] = (double)(i - 1) / (aNbK - 1);
                theM[i] = (i == 1 || i == aNbK) ? 4 : 1;
            }
        }
        Knots(NU, out NCollection_Array1<double> UK, out NCollection_Array1<int> UM);
        Knots(NV, out NCollection_Array1<double> VK, out NCollection_Array1<int> VM);
        return new BSplineSurface(aP, UK, VK, UM, VM, 3, 3);
    }

    // Side k of S: 0 v=0 (u 0->1), 1 u=1 (v 0->1), 2 v=1 (u 0->1), 3 u=0 (v 0->1)
    internal static Boundary MakeBound(BSplineSurface theS, int theSide, int theType, double theTol3d,
                                                double theTolAng)
    {
        if (theType == 0)
        {
            Curve3D anIso = theSide switch
            {
                0 => theS.VIso(0.0),
                1 => theS.UIso(1.0),
                2 => theS.VIso(1.0),
                _ => theS.UIso(0.0),
            };
            return new SimpleBound(new CurveAdaptor(anIso), theTol3d, theTolAng);
        }
        Point2D aP = theSide switch
        {
            0 => new Point2D(0, 0),
            1 => new Point2D(1, 0),
            2 => new Point2D(0, 1),
            _ => new Point2D(0, 0),
        };
        Direction2D aD = (theSide == 0 || theSide == 2) ? new Direction2D(1, 0) : new Direction2D(0, 1);
        LineCurve2D aL = new(aP, aD);
        CurveAdaptor2D aC2 = new(aL, 0.0, 1.0);
        SurfaceAdaptor aSA = new(theS);
        CurveOnSurface aCoS = new(aC2, aSA);
        return new BoundWithSurf(aCoS, theTol3d, theTolAng);
    }

    private static void ConstrainedCase(Writer theW, Reader theR)
    {
        int aNbSides = theR.I();
        int aMaxDeg = theR.I();
        int aMaxSeg = theR.I();
        bool aNoCheck = theR.B();
        double aTol3d = theR.D();
        double aTolAng = theR.D();
        BSplineSurface S = MakeSurface(theR);
        int[] aType = new int[4];
        for (int k = 0; k < 4; k++)
        {
            aType[k] = theR.I();
        }
        int[] aOrder = new int[4];
        for (int k = 0; k < 4; k++)
        {
            aOrder[k] = theR.I();
        }
        bool aHasDF = theR.B();
        bool aHasDL = theR.B();
        double aDF = theR.D();
        double aDL = theR.D();
        bool aRev = theR.B();
        bool aDomain = theR.B();
        int aDomSide = theR.I();
        double aDomL = theR.D();

        // the boundaries alone
        for (int k = 0; k < 4; k++)
        {
            Boundary B = MakeBound(S, k, aType[k], aTol3d, aTolAng);
            string aK = Int(k);
            theW.Tok("b" + aK, () =>
            {
                string s = "";
                foreach (double w in new[] { 0.0, 0.37, 1.0 })
                {
                    B.D1(w, out Point3D P, out Vector3D V);
                    s = Join(s, Hex(B.Value(w)) + "," + Hex(P) + "," + Hex(V));
                    if (B.HasNormals)
                    {
                        B.D1Norm(w, out Vector3D N, out Vector3D DN);
                        s += "," + Hex(B.Norm(w)) + "," + Hex(N) + "," + Hex(DN);
                    }
                }
                return s;
            });
            theW.Tok("r" + aK, () =>
            {
                B.Reparametrize(0.0, 1.0, aHasDF, aHasDL, aDF, aDL, aRev);
                B.Bounds(out double f, out double l);
                B.D1(0.21, out Point3D P, out Vector3D V);
                B.Points(out Point3D P1, out Point3D P2);
                return Hex(f) + "," + Hex(l) + "," + Hex(B.Value(0.21)) + "," + Hex(P) + "," + Hex(V) + "," + Hex(P1) + ","
                       + Hex(P2);
            });
        }

        // the algorithmic patch and its tangent fields
        {
            Boundary[] B = new Boundary[4];
            for (int k = 0; k < 4; k++)
            {
                B[k] = MakeBound(S, k, aType[k], aTol3d, aTolAng);
            }
            CoonsAlgPatch K = new(B[0], B[1], B[2], B[3]);
            theW.Tok("pc", () => Hex(K.Corner(0)) + "," + Hex(K.Corner(1)) + "," + Hex(K.Corner(2)) + "," + Hex(K.Corner(3)));
            theW.Tok("pv", () =>
            {
                string s = "";
                foreach (double u in new[] { 0.25, 0.5, 0.9 })
                {
                    foreach (double v in new[] { 0.1, 0.5, 0.75 })
                    {
                        s = Join(s, Hex(K.Value(u, v)) + "," + Hex(K.D1U(u, v)) + "," + Hex(K.D1V(u, v)) + "," + Hex(K.DUV(u, v)));
                    }
                }
                return s;
            });
            for (int k = 0; k < 4; k++)
            {
                if (!B[k].HasNormals)
                {
                    continue;
                }
                TgtOnCoons T = new(K, k);
                theW.Tok("t" + Int(k), () =>
                {
                    string s = "";
                    foreach (double w in new[] { 0.3, 0.7 })
                    {
                        T.D1(w, out Vector3D V, out Vector3D DV);
                        s = Join(s, Hex(T.Value(w)) + "," + Hex(T.D1(w)) + "," + Hex(V) + "," + Hex(DV));
                    }
                    return s;
                });
            }
        }

        // the filling
        Boundary[] Bs = new Boundary[4];
        for (int k = 0; k < 4; k++)
        {
            Bs[k] = MakeBound(S, k, aType[k], aTol3d, aTolAng);
        }
        List<Boundary> aUsed = [];
        for (int k = 0; k < 4; k++)
        {
            if (aNbSides == 4 || aOrder[k] != 3)
            {
                aUsed.Add(Bs[aOrder[k]]);
            }
        }
        ConstrainedFilling CF = new(aMaxDeg, aMaxSeg);
        try
        {
            if (aNbSides == 4)
            {
                CF.Init(aUsed[0], aUsed[1], aUsed[2], aUsed[3], aNoCheck);
            }
            else
            {
                CF.Init(aUsed[0], aUsed[1], aUsed[2], aNoCheck);
            }
        }
        catch (Exception anExc) when (Refusal.IsFailure(anExc))
        {
            theW.Put("init:" + ExcName(Refusal.Of(anExc)!));
            return;
        }
        PutBSplineSurface(theW, CF.Surface());
        theW.Tok("bnd", () =>
        {
            string s = "";
            for (int i = 0; i < 4; i++)
            {
                Boundary b = CF.Boundary(i);
                b.Bounds(out double f, out double l);
                s = Join(s, Hex(f) + "," + Hex(l) + "," + Hex(b.Value(0.5)) + "," + (b.IsDegenerated ? "1" : "0"));
            }
            return s;
        });
        theW.Tok("eval", () =>
        {
            string s = "";
            foreach (double w in new[] { 0.0, 0.3, 1.0 })
            {
                for (int ord = 0; ord <= 1; ord++)
                {
                    double[] r = new double[12];
                    CF.Eval(w, ord, r);
                    foreach (double x in r)
                    {
                        s = Join(s, Hex(x));
                    }
                }
            }
            return s;
        });
        if (aDomain)
        {
            if (CF.Boundary(aDomSide) is BoundWithSurf aBWS)
            {
                theW.Tok("rebuild", () =>
                {
                    CF.SetDomain(aDomL, aBWS);
                    CF.ReBuild();
                    return "ok";
                });
                PutBSplineSurface(theW, CF.Surface());
            }
        }
    }
}
