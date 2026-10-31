#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Xunit.Abstractions;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The seven Geom2dGcc solvers build123d.NET binds (Circ2d2TanRad, Circ2d2TanOn, Circ2d3Tan, Circ2dTanCen,
/// Circ2dTanOnRad, Lin2d2Tan, Lin2dTanObl), every bound constructor, against native OCCT on random arguments: lines,
/// circles, ellipses, B-splines, offset curves (of ellipses, circles and smooth B-splines) and trimmed curves (of
/// circles, ellipses and B-splines), each with a random qualifier, and points. Analytic arguments go through GccAna,
/// the others through the Geo and Iter solvers.
/// </summary>
/// <remarks>
/// <para>
/// Each case's results are written on both sides as named tokens (every double as its 64 bits): the exception a
/// constructor throws, IsDone, NbSolutions, then for each solution the circle or line, each tangency (or the exception
/// the accessor throws), IsTheSame and CenterOn3. WhichQualifier isn't bound. Native's 2D offset curves aren't bound
/// either: both sides read them as the pcurve of an edge from the same BRep file the port writes.
/// </para>
/// <para>
/// Results OCCT leaves uninitialised are masked (decision 5980), and whole cases whose native results depend on
/// uninitialised memory are attributed by kind; every other token must be identical.
/// </para>
/// </remarks>
public sealed class Geom2dGccOracleTests(ITestOutputHelper output) : IDisposable
{
    private const int CasesPerConstructor = 500;

    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-gcc-").FullName;
    private int myFileNumber;

    public void Dispose()
    {
        try
        {
            Directory.Delete(myDirectory, true);
        }
        catch (IOException)
        {
        }
    }

    private enum Kind { Line, Circle, Ellipse, BSpline, Offset, Trimmed }

    /// <summary>A 2D curve argument, built identically on both sides, and its qualifier.</summary>
    private sealed class Arg
    {
        public Kind Kind;
        public double Px, Py, Dx, Dy, R1, R2, Offset, U1, U2;
        public bool Sense = true;
        public BSplineSpec? BSpline;
        public Arg? Basis;
        public GccEnt_Position Q;
        public string Name = "";

        /// <summary>True when the adaptor's type is a line or a circle, so that Geom2dGcc dispatches to GccAna.</summary>
        public bool IsAnalytic => Kind is Kind.Line or Kind.Circle || (Kind == Kind.Trimmed && Basis!.IsAnalytic);

        public bool IsCircle => Kind == Kind.Circle || (Kind == Kind.Trimmed && Basis!.IsCircle);

        public bool IsLine => Kind == Kind.Line || (Kind == Kind.Trimmed && Basis!.IsLine);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Inputs

    private static (double X, double Y) RawDir(Oracle theO)
    {
        gp_XY aV = GeomOracleInputs.RawDir2d(theO);
        return (aV.X(), aV.Y());
    }

    private static Arg RandomArg(Oracle theO, bool theAllowFreeForm = true)
    {
        int aK = theAllowFreeForm ? theO.Int(0, 9) : theO.Int(0, 3);
        Kind aKind = aK switch
        {
            0 or 1 => Kind.Line,
            2 or 3 => Kind.Circle,
            4 => Kind.Ellipse,
            5 or 6 => Kind.BSpline,
            7 => Kind.Offset,
            _ => Kind.Trimmed,
        };
        Arg anA = BaseArg(theO, aKind);
        int aQ = theO.Int(0, 6);
        anA.Q = aQ switch
        {
            0 or 1 or 2 => GccEnt_Position.GccEnt_unqualified,
            3 => GccEnt_Position.GccEnt_enclosing,
            4 => GccEnt_Position.GccEnt_enclosed,
            _ => GccEnt_Position.GccEnt_outside,
        };
        return anA;
    }

    private static Arg BaseArg(Oracle theO, Kind theKind)
    {
        var anA = new Arg { Kind = theKind };
        anA.Px = theO.Real(-8, 8);
        anA.Py = theO.Real(-8, 8);
        (anA.Dx, anA.Dy) = RawDir(theO);
        anA.Sense = theO.Int(0, 3) != 0;
        switch (theKind)
        {
            case Kind.Line:
                anA.Name = "line";
                break;
            case Kind.Circle:
                anA.R1 = theO.Bool() ? theO.Real(0.5, 6) : theO.Int(1, 6);
                anA.Name = "circle";
                break;
            case Kind.Ellipse:
                anA.R1 = theO.Real(1, 7);
                anA.R2 = theO.Real(0.3, anA.R1);
                anA.Name = "ellipse";
                break;
            case Kind.BSpline:
                anA.BSpline = Geom2dOracle.RandomBSpline(theO);
                anA.Name = $"bspline{anA.BSpline.Degree}";
                break;
            case Kind.Offset:
            {
                int aB = theO.Int(0, 2);
                anA.Basis = aB switch
                {
                    0 => BaseArg(theO, Kind.Circle),
                    1 => BaseArg(theO, Kind.Ellipse),
                    _ => SmoothBSpline(theO),
                };
                anA.Offset = theO.Bool() ? theO.Real(0.2, 2) : -theO.Real(0.2, 0.9 * MinRadius(anA.Basis));
                anA.Name = "offset of " + anA.Basis.Name;
                break;
            }
            default:
            {
                int aB = theO.Int(0, 3);
                anA.Basis = BaseArg(theO, aB switch { 0 => Kind.Circle, 1 => Kind.Ellipse, 2 => Kind.Line, _ => Kind.BSpline });
                Geom2d_Curve aC = BuildPort(anA.Basis);
                double aFirst = Precision.IsInfinite(aC.FirstParameter()) ? -15 : aC.FirstParameter();
                double aLast = Precision.IsInfinite(aC.LastParameter()) ? 15 : aC.LastParameter();
                double a = theO.Real(aFirst, aLast), b = theO.Real(aFirst, aLast);
                if (Math.Abs(a - b) < 0.1 * (aLast - aFirst))
                {
                    b = a + 0.5 * (aLast - aFirst);
                }
                (anA.U1, anA.U2) = a < b || aC.IsPeriodic() ? (a, b) : (b, a);
                if (!aC.IsPeriodic())
                {
                    anA.U2 = Math.Min(anA.U2, aLast);
                }
                anA.Name = "trimmed " + anA.Basis.Name;
                break;
            }
        }
        return anA;
    }

    private static double MinRadius(Arg theA) => theA.Kind switch
    {
        Kind.Circle => theA.R1,
        Kind.Ellipse => theA.R2 * theA.R2 / theA.R1,
        _ => 0.3,
    };

    /// <summary>A B-spline of degree 2 to 5 with simple knots (at least C1), a valid offset basis.</summary>
    private static Arg SmoothBSpline(Oracle theO)
    {
        while (true)
        {
            BSplineSpec aS = Geom2dOracle.RandomBSpline(theO, null, true);
            bool aSmooth = aS.Degree >= 2;
            for (int i = 1; i < aS.Mults.Length - 1; i++)
            {
                aSmooth &= aS.Mults[i] < aS.Degree;
            }
            if (aSmooth)
            {
                return new Arg { Kind = Kind.BSpline, BSpline = aS, Name = $"bspline{aS.Degree}" };
            }
        }
    }

    private static Geom2d_Curve BuildPort(Arg theA)
    {
        switch (theA.Kind)
        {
            case Kind.Line:
                return new Geom2d_Line(new gp_Pnt2d(theA.Px, theA.Py), new gp_Dir2d(theA.Dx, theA.Dy));
            case Kind.Circle:
                return new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(theA.Px, theA.Py), new gp_Dir2d(theA.Dx, theA.Dy)), theA.R1, theA.Sense);
            case Kind.Ellipse:
                return new Geom2d_Ellipse(new gp_Ax2d(new gp_Pnt2d(theA.Px, theA.Py), new gp_Dir2d(theA.Dx, theA.Dy)), theA.R1, theA.R2,
                                          theA.Sense);
            case Kind.BSpline:
                return Geom2dOracle.ToPort(theA.BSpline!);
            case Kind.Offset:
                return new Geom2d_OffsetCurve(BuildPort(theA.Basis!), theA.Offset);
            default:
                return new Geom2d_TrimmedCurve(BuildPort(theA.Basis!), theA.U1, theA.U2);
        }
    }

    private static N.Geom2d_Curve BuildNative(Arg theA)
    {
        switch (theA.Kind)
        {
            case Kind.Line:
            {
                using var aP = new N.gp_Pnt2d(theA.Px, theA.Py);
                using var aD = new N.gp_Dir2d(theA.Dx, theA.Dy);
                return new N.Geom2d_Line(aP, aD);
            }
            case Kind.Circle:
            case Kind.Ellipse:
            {
                using var aP = new N.gp_Pnt2d(theA.Px, theA.Py);
                using var aD = new N.gp_Dir2d(theA.Dx, theA.Dy);
                using var anAx = new N.gp_Ax2d(aP, aD);
                return theA.Kind == Kind.Circle
                           ? new N.Geom2d_Circle(anAx, theA.R1, theA.Sense)
                           : new N.Geom2d_Ellipse(anAx, theA.R1, theA.R2, theA.Sense);
            }
            case Kind.BSpline:
                return Geom2dOracle.ToNative(theA.BSpline!);
            case Kind.Trimmed:
                return new N.Geom2d_TrimmedCurve(BuildNative(theA.Basis!), theA.U1, theA.U2);
            default:
                throw new InvalidOperationException("offset curves go through a file");
        }
    }

    /// <summary>
    /// The curve of an argument on both sides. Offset curves (not bound natively) are the pcurve of an edge on a plane
    /// face, written by the port and read by both sides; directions are checked to be the same bits on both sides.
    /// </summary>
    private (N.Geom2d_Curve Native, Geom2d_Curve Port)? Build(Arg theA)
    {
        if (theA.Kind == Kind.Offset)
        {
            Geom2d_Curve anOffset = BuildPort(theA);
            var aPlane = new Geom_Plane(new gp_Ax3());
            TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(aPlane, -50, 50, -50, 50, 1e-7).Face();
            TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(anOffset, BRep_Tool.Surface(aFace)!).Edge();
            string aFile = Path.Combine(myDirectory, $"c{myFileNumber++}.brep");
            Assert.True(BRepTools.Write(TopoDS_Builder.MakeCompound(aFace, anEdge), aFile));
            Assert.True(BRepTools.Read(out TopoDS_Shape aPShape, aFile));
            TopoDS_Face aPF = TopoDS.Face(new TopExp_Explorer(aPShape, TopAbs_ShapeEnum.TopAbs_FACE).Current());
            // The free edge is the last one met (after the face's four).
            TopoDS_Shape aPLast = new();
            for (var anEx = new TopExp_Explorer(aPShape, TopAbs_ShapeEnum.TopAbs_EDGE); anEx.More(); anEx.Next())
            {
                aPLast = anEx.Current();
            }
            TopoDS_Edge aPE = TopoDS.Edge(aPLast);
            Geom2d_Curve aP = BRep_Tool.CurveOnSurface(aPE, aPF, out _, out _)!;
            var aNShape = new N.TopoDS_Shape();
            using (var aBuilder = new N.BRep_Builder())
            {
                Assert.True(N.BRepTools.Read(aNShape, aFile, aBuilder));
            }
            using var aFX = new N.TopExp_Explorer(aNShape, N.TopAbs_ShapeEnum.TopAbs_FACE);
            using var aEX = new N.TopExp_Explorer(aNShape, N.TopAbs_ShapeEnum.TopAbs_EDGE);
            N.TopoDS_Shape aNLast = aEX.Current();
            for (; aEX.More(); aEX.Next())
            {
                aNLast = aEX.Current();
            }
            double f = 0, l = 0;
            N.Geom2d_Curve aN = N.BRep_Tool.CurveOnSurface(N.TopoDS.Edge(aNLast), N.TopoDS.Face(aFX.Current()), ref f, ref l)!;
            Assert.True(aP is Geom2d_OffsetCurve, theA.Name + " read as " + aP.GetType().Name);
            return (aN, aP);
        }
        N.Geom2d_Curve aNative = BuildNative(theA);
        Geom2d_Curve aPort = BuildPort(theA);
        // The same directions on both sides (native's normalisation can differ in the last bit): else redraw.
        if (theA.Kind is Kind.Line or Kind.Circle or Kind.Ellipse
            || (theA.Kind == Kind.Trimmed && theA.Basis!.Kind != Kind.BSpline))
        {
            using var aNA = new N.Geom2dAdaptor_Curve(aNative);
            var aPA = new Geom2dAdaptor_Curve(aPort);
            double u = theA.Kind == Kind.Trimmed ? theA.U1 : 0.7;
            using N.gp_Pnt2d aNP = aNA.Value(u);
            gp_Pnt2d aPP = aPA.Value(u);
            if (aNP.X() != aPP.X() || aNP.Y() != aPP.Y())
            {
                return null;
            }
        }
        return (aNative, aPort);
    }

    private static double StartParam(Oracle theO, Geom2d_Curve theC)
    {
        double aFirst = theC.FirstParameter(), aLast = theC.LastParameter();
        if (Precision.IsInfinite(aFirst) || Precision.IsInfinite(aLast))
        {
            return theO.Real(-10, 10);
        }
        return theO.Real(aFirst, aLast);
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Results as tokens

    internal sealed class Rec
    {
        public readonly List<(string Name, string Value)> T = [];

        public void D(string theName, double theV)
            => T.Add((theName, double.IsNaN(theV) ? "NaN" : BitConverter.DoubleToInt64Bits(theV).ToString("X16", CultureInfo.InvariantCulture)));

        public void I(string theName, int theV) => T.Add((theName, theV.ToString(CultureInfo.InvariantCulture)));

        public void B(string theName, bool theV) => T.Add((theName, theV ? "1" : "0"));

        public void Throw(string theName, string theType) => T.Add((theName, "throws " + theType));

        public void Try(string theName, Action theA)
        {
            try
            {
                theA();
            }
            catch (N.OcctException anE)
            {
                Throw(theName, anE.IsFault ? "fault" : anE.OcctType);
            }
            catch (Exception anE) when (anE is Standard_Failure or NullReferenceException or IndexOutOfRangeException)
            {
                Throw(theName, anE switch
                {
                    NullReferenceException => "fault",
                    IndexOutOfRangeException => "Standard_OutOfRange",
                    _ => anE.GetType().Name,
                });
            }
        }

        public void P(string theName, N.gp_Pnt2d theP)
        {
            D(theName + ".x", theP.X());
            D(theName + ".y", theP.Y());
        }

        public void P(string theName, in gp_Pnt2d theP)
        {
            D(theName + ".x", theP.X());
            D(theName + ".y", theP.Y());
        }

        public void Circ(string theName, N.gp_Circ2d theC)
        {
            using N.gp_Pnt2d aL = theC.Location();
            using N.gp_Ax2d aX = theC.XAxis();
            using N.gp_Dir2d aXD = aX.Direction();
            using N.gp_Ax2d aY = theC.YAxis();
            using N.gp_Dir2d aYD = aY.Direction();
            P(theName + ".loc", aL);
            D(theName + ".xd.x", aXD.X());
            D(theName + ".xd.y", aXD.Y());
            D(theName + ".yd.x", aYD.X());
            D(theName + ".yd.y", aYD.Y());
            D(theName + ".r", theC.Radius());
        }

        public void Circ(string theName, in gp_Circ2d theC)
        {
            P(theName + ".loc", theC.Location());
            D(theName + ".xd.x", theC.XAxis().Direction().X());
            D(theName + ".xd.y", theC.XAxis().Direction().Y());
            D(theName + ".yd.x", theC.YAxis().Direction().X());
            D(theName + ".yd.y", theC.YAxis().Direction().Y());
            D(theName + ".r", theC.Radius());
        }

        public void Lin(string theName, N.gp_Lin2d theL)
        {
            using N.gp_Pnt2d aL = theL.Location();
            using N.gp_Dir2d aD = theL.Direction();
            P(theName + ".loc", aL);
            D(theName + ".d.x", aD.X());
            D(theName + ".d.y", aD.Y());
        }

        public void Lin(string theName, in gp_Lin2d theL)
        {
            P(theName + ".loc", theL.Location());
            D(theName + ".d.x", theL.Direction().X());
            D(theName + ".d.y", theL.Direction().Y());
        }
    }

    internal delegate void NTan(int theI, ref double theParSol, ref double theParArg, N.gp_Pnt2d thePnt);

    internal delegate void PTan(int theI, out double theParSol, out double theParArg, out gp_Pnt2d thePnt);

    internal delegate void NCen(int theI, ref double theParArg, N.gp_Pnt2d thePnt);

    internal delegate void PCen(int theI, out double theParArg, out gp_Pnt2d thePnt);

    /// <summary>The accessors of one native solver, as the bindings give them.</summary>
    internal sealed record NSolver(Func<bool> IsDone, Func<int> NbSolutions, Action<Rec, int> Solution, NTan[] Tangency,
                                  Func<int, bool>[] IsTheSame, NCen? CenterOn3);

    internal sealed record PSolver(Func<bool> IsDone, Func<int> NbSolutions, Action<Rec, int> Solution, PTan[] Tangency,
                                  Func<int, bool>[] IsTheSame, PCen? CenterOn3);

    private static void Record(Rec theR, NSolver theS)
    {
        theR.B("done", theS.IsDone());
        int aNb = theS.NbSolutions();
        theR.I("nb", aNb);
        for (int i = 1; i <= aNb; i++)
        {
            int anI = i;
            theR.Try($"s.{i}", () => theS.Solution(theR, anI));
            for (int k = 0; k < theS.Tangency.Length; k++)
            {
                int aK = k;
                theR.Try($"t{k + 1}.{i}", () =>
                {
                    double a = 0, b = 0;
                    using var aP = new N.gp_Pnt2d();
                    theS.Tangency[aK](anI, ref a, ref b, aP);
                    theR.D($"t{aK + 1}.{anI}.parsol", a);
                    theR.D($"t{aK + 1}.{anI}.pararg", b);
                    theR.P($"t{aK + 1}.{anI}.pnt", aP);
                });
            }
            for (int k = 0; k < theS.IsTheSame.Length; k++)
            {
                int aK = k;
                theR.Try($"same{k + 1}.{i}", () => theR.B($"same{aK + 1}.{anI}", theS.IsTheSame[aK](anI)));
            }
            if (theS.CenterOn3 is not null)
            {
                theR.Try($"c3.{i}", () =>
                {
                    double a = 0;
                    using var aP = new N.gp_Pnt2d();
                    theS.CenterOn3(anI, ref a, aP);
                    theR.D($"c3.{anI}.pararg", a);
                    theR.P($"c3.{anI}.pnt", aP);
                });
            }
        }
    }

    private static void Record(Rec theR, PSolver theS)
    {
        theR.B("done", theS.IsDone());
        int aNb = theS.NbSolutions();
        theR.I("nb", aNb);
        for (int i = 1; i <= aNb; i++)
        {
            int anI = i;
            theR.Try($"s.{i}", () => theS.Solution(theR, anI));
            for (int k = 0; k < theS.Tangency.Length; k++)
            {
                int aK = k;
                theR.Try($"t{k + 1}.{i}", () =>
                {
                    theS.Tangency[aK](anI, out double a, out double b, out gp_Pnt2d aP);
                    theR.D($"t{aK + 1}.{anI}.parsol", a);
                    theR.D($"t{aK + 1}.{anI}.pararg", b);
                    theR.P($"t{aK + 1}.{anI}.pnt", aP);
                });
            }
            for (int k = 0; k < theS.IsTheSame.Length; k++)
            {
                int aK = k;
                theR.Try($"same{k + 1}.{i}", () => theR.B($"same{aK + 1}.{anI}", theS.IsTheSame[aK](anI)));
            }
            if (theS.CenterOn3 is not null)
            {
                theR.Try($"c3.{i}", () =>
                {
                    theS.CenterOn3(anI, out double a, out gp_Pnt2d aP);
                    theR.D($"c3.{anI}.pararg", a);
                    theR.P($"c3.{anI}.pnt", aP);
                });
            }
        }
    }

    // Native solvers' circles and lines
    private static Action<Rec, int> NCircles(Func<int, N.gp_Circ2d> theSol) => (r, i) =>
    {
        using N.gp_Circ2d aC = theSol(i);
        r.Circ($"s.{i}", aC);
    };

    private static Action<Rec, int> PCircles(Func<int, gp_Circ2d> theSol) => (r, i) => r.Circ($"s.{i}", theSol(i));

    // ---------------------------------------------------------------------------------------------------------------
    // Cases

    /// <summary>
    /// One random case of a constructor: draws its arguments, builds both solvers in <paramref name="theRunNative"/> and
    /// <paramref name="theRunPort"/>, and returns both records (a constructor's exception is the record's one token).
    /// </summary>
    private sealed class Case
    {
        public required string Ctor;
        public required string Description;
        public required Func<Rec> Native;
        public required Func<Rec> Port;

        /// <summary>Token names (prefixes before the solution index) whose values OCCT leaves uninitialised.</summary>
        public Func<string, bool> Masked = _ => false;

        /// <summary>Why native's results may differ in this case (null: they mustn't).</summary>
        public string? Attribution;

        /// <summary>Whether the attribution explains these differences (by default, any).</summary>
        public Func<List<string>, bool> Explained = _ => true;
    }

    internal static Rec RunNative(Func<NSolver> theCtor)
    {
        var aR = new Rec();
        NSolver? aS = null;
        aR.Try("ctor", () => aS = theCtor());
        if (aS is not null)
        {
            Record(aR, aS);
        }
        return aR;
    }

    internal static Rec RunPort(Func<PSolver> theCtor)
    {
        var aR = new Rec();
        PSolver? aS = null;
        aR.Try("ctor", () => aS = theCtor());
        if (aS is not null)
        {
            Record(aR, aS);
        }
        return aR;
    }

    private static N.Geom2dGcc_QualifiedCurve NQ(N.Geom2d_Curve theC, GccEnt_Position theQ)
    {
        using var anAd = new N.Geom2dAdaptor_Curve(theC);
        return new N.Geom2dGcc_QualifiedCurve(anAd, (N.GccEnt_Position)(int)theQ);
    }

    private static Geom2dGcc_QualifiedCurve PQ(Geom2d_Curve theC, GccEnt_Position theQ) => new(new Geom2dAdaptor_Curve(theC), theQ);

    private static string Describe(params Arg[] theArgs)
        => string.Join(", ", theArgs.Select(a => a.Name + " " + GccEnt.PositionToString(a.Q).Replace("GccEnt_", "", StringComparison.Ordinal)));

    private (Arg A, N.Geom2d_Curve N, Geom2d_Curve P) Draw(Oracle theO, bool theAllowFreeForm = true)
    {
        while (true)
        {
            Arg anA = RandomArg(theO, theAllowFreeForm);
            var aB = Build(anA);
            if (aB is not null)
            {
                return (anA, aB.Value.Native, aB.Value.Port);
            }
        }
    }

    private static (double X, double Y) RandomPoint(Oracle theO) => (theO.Real(-8, 8), theO.Real(-8, 8));

    internal static NSolver NCirc2d2TanRad(N.Geom2dGcc_Circ2d2TanRad s)
        => new(s.IsDone, s.NbSolutions, NCircles(s.ThisSolution), [s.Tangency1, s.Tangency2], [s.IsTheSame1, s.IsTheSame2], null);

    internal static PSolver PCirc2d2TanRad(Geom2dGcc_Circ2d2TanRad s)
        => new(s.IsDone, s.NbSolutions, PCircles(s.ThisSolution), [s.Tangency1, s.Tangency2], [s.IsTheSame1, s.IsTheSame2], null);

    internal static NSolver NCirc2d2TanOn(N.Geom2dGcc_Circ2d2TanOn s)
        => new(s.IsDone, s.NbSolutions, NCircles(s.ThisSolution), [s.Tangency1, s.Tangency2], [s.IsTheSame1, s.IsTheSame2],
               s.CenterOn3);

    internal static PSolver PCirc2d2TanOn(Geom2dGcc_Circ2d2TanOn s)
        => new(s.IsDone, s.NbSolutions, PCircles(s.ThisSolution), [s.Tangency1, s.Tangency2], [s.IsTheSame1, s.IsTheSame2],
               s.CenterOn3);

    internal static NSolver NCirc2d3Tan(N.Geom2dGcc_Circ2d3Tan s)
        => new(s.IsDone, s.NbSolutions, NCircles(s.ThisSolution), [s.Tangency1, s.Tangency2, s.Tangency3],
               [s.IsTheSame1, s.IsTheSame2, s.IsTheSame3], null);

    internal static PSolver PCirc2d3Tan(Geom2dGcc_Circ2d3Tan s)
        => new(s.IsDone, s.NbSolutions, PCircles(s.ThisSolution), [s.Tangency1, s.Tangency2, s.Tangency3],
               [s.IsTheSame1, s.IsTheSame2, s.IsTheSame3], null);

    internal static NSolver NCirc2dTanCen(N.Geom2dGcc_Circ2dTanCen s)
        => new(s.IsDone, s.NbSolutions, NCircles(s.ThisSolution), [s.Tangency1], [s.IsTheSame1], null);

    internal static PSolver PCirc2dTanCen(Geom2dGcc_Circ2dTanCen s)
        => new(s.IsDone, s.NbSolutions, PCircles(s.ThisSolution), [s.Tangency1], [s.IsTheSame1], null);

    internal static NSolver NCirc2dTanOnRad(N.Geom2dGcc_Circ2dTanOnRad s)
        => new(s.IsDone, s.NbSolutions, NCircles(s.ThisSolution), [s.Tangency1], [s.IsTheSame1], s.CenterOn3);

    internal static PSolver PCirc2dTanOnRad(Geom2dGcc_Circ2dTanOnRad s)
        => new(s.IsDone, s.NbSolutions, PCircles(s.ThisSolution), [s.Tangency1], [s.IsTheSame1], s.CenterOn3);

    internal static NSolver NLin2d2Tan(N.Geom2dGcc_Lin2d2Tan s)
        => new(s.IsDone, s.NbSolutions, (r, i) =>
        {
            using N.gp_Lin2d aL = s.ThisSolution(i);
            r.Lin($"s.{i}", aL);
        }, [s.Tangency1, s.Tangency2], [], null);

    internal static PSolver PLin2d2Tan(Geom2dGcc_Lin2d2Tan s)
        => new(s.IsDone, s.NbSolutions, (r, i) => r.Lin($"s.{i}", s.ThisSolution(i)), [s.Tangency1, s.Tangency2], [], null);

    internal static NSolver NLin2dTanObl(N.Geom2dGcc_Lin2dTanObl s)
        => new(s.IsDone, s.NbSolutions, (r, i) =>
        {
            using N.gp_Lin2d aL = s.ThisSolution(i);
            r.Lin($"s.{i}", aL);
        }, [s.Tangency1, s.Intersection2], [], null);

    internal static PSolver PLin2dTanObl(Geom2dGcc_Lin2dTanObl s)
        => new(s.IsDone, s.NbSolutions, (r, i) => r.Lin($"s.{i}", s.ThisSolution(i)), [s.Tangency1, s.Intersection2], [], null);

    private static bool Starts(string theName, params string[] thePrefixes) => thePrefixes.Any(p => theName.StartsWith(p, StringComparison.Ordinal));

    /// <summary>Every bound constructor, as a generator of random cases.</summary>
    private IEnumerable<(string Ctor, Func<Oracle, Case> Make)> Constructors()
    {
        // Circ2d2TanRad
        yield return ("Circ2d2TanRad(curve, curve)", o =>
        {
            var (a1, n1, p1) = Draw(o);
            var (a2, n2, p2) = Draw(o);
            double r = o.Real(0.2, 6), tol = o.Bool() ? 1e-7 : 1e-4;
            var c = new Case
            {
                Ctor = "Circ2d2TanRad(curve, curve)",
                Description = Describe(a1, a2) + $", r {r}",
                Native = () => RunNative(() => NCirc2d2TanRad(new N.Geom2dGcc_Circ2d2TanRad(NQ(n1, a1.Q), NQ(n2, a2.Q), r, tol))),
                Port = () => RunPort(() => PCirc2d2TanRad(new Geom2dGcc_Circ2d2TanRad(PQ(p1, a1.Q), PQ(p2, a2.Q), r, tol))),
            };
            if (a1.IsCircle && a2.IsCircle)
            {
                // GccAna_Circ2d2TanRad(circle, circle) leaves TheSame2(1) unset (decision 781): Results reads it
                // through IsTheSame2 and then calls Tangency2, which throws StdFail_NotDone when it isn't 0.
                c.Masked = n => Starts(n, "same2.", "t2.");
                c.Attribution = "GccAna_Circ2d2TanRad(circle, circle)'s unset TheSame2 (native throws from Results when it isn't 0)";
            }
            return c;
        });
        yield return ("Circ2d2TanRad(curve, point)", o =>
        {
            var (a1, n1, p1) = Draw(o);
            var (x, y) = RandomPoint(o);
            double r = o.Real(0.2, 6), tol = o.Bool() ? 1e-7 : 1e-4;
            return new Case
            {
                Ctor = "Circ2d2TanRad(curve, point)",
                Description = Describe(a1) + $", point ({x}, {y}), r {r}",
                Native = () => RunNative(() => NCirc2d2TanRad(new N.Geom2dGcc_Circ2d2TanRad(NQ(n1, a1.Q), new N.Geom2d_CartesianPoint(x, y), r, tol))),
                Port = () => RunPort(() => PCirc2d2TanRad(new Geom2dGcc_Circ2d2TanRad(PQ(p1, a1.Q), new Geom2d_CartesianPoint(x, y), r, tol))),
            };
        });
        yield return ("Circ2d2TanRad(point, point)", o =>
        {
            var (x1, y1) = RandomPoint(o);
            var (x2, y2) = RandomPoint(o);
            double r = o.Real(0.2, 9), tol = o.Bool() ? 1e-7 : 1e-4;
            return new Case
            {
                Ctor = "Circ2d2TanRad(point, point)",
                Description = $"({x1}, {y1}), ({x2}, {y2}), r {r}",
                Native = () => RunNative(() => NCirc2d2TanRad(new N.Geom2dGcc_Circ2d2TanRad(new N.Geom2d_CartesianPoint(x1, y1), new N.Geom2d_CartesianPoint(x2, y2), r, tol))),
                Port = () => RunPort(() => PCirc2d2TanRad(new Geom2dGcc_Circ2d2TanRad(new Geom2d_CartesianPoint(x1, y1), new Geom2d_CartesianPoint(x2, y2), r, tol))),
            };
        });

        // Circ2d2TanOn
        yield return ("Circ2d2TanOn(curve, curve, on)", o =>
        {
            var (a1, n1, p1) = Draw(o);
            var (a2, n2, p2) = Draw(o);
            var (a3, n3, p3) = Draw(o);
            double tol = o.Bool() ? 1e-7 : 1e-4;
            double u1 = StartParam(o, p1), u2 = StartParam(o, p2), u3 = StartParam(o, p3);
            var c = new Case
            {
                Ctor = "Circ2d2TanOn(curve, curve, on)",
                Description = Describe(a1, a2) + ", on " + a3.Name,
                Native = () => RunNative(() =>
                {
                    using var anOn = new N.Geom2dAdaptor_Curve(n3);
                    return NCirc2d2TanOn(new N.Geom2dGcc_Circ2d2TanOn(NQ(n1, a1.Q), NQ(n2, a2.Q), anOn, tol, u1, u2, u3));
                }),
                Port = () => RunPort(() => PCirc2d2TanOn(new Geom2dGcc_Circ2d2TanOn(PQ(p1, a1.Q), PQ(p2, a2.Q), new Geom2dAdaptor_Curve(p3), tol, u1, u2, u3))),
            };
            if (!(a1.IsAnalytic && a2.IsAnalytic && a3.IsAnalytic))
            {
                // The Iter branches never write parcen3 (CenterOn3's parameter).
                c.Masked = n => n.StartsWith("c3.", StringComparison.Ordinal) && n.EndsWith(".pararg", StringComparison.Ordinal);
            }
            if (a1.IsCircle && a2.IsCircle && !a3.IsAnalytic && (a1.Q == GccEnt_Position.GccEnt_unqualified || a2.Q == GccEnt_Position.GccEnt_unqualified))
            {
                c.Attribution = "Circ2d2TanOnGeo(circle, circle, curve)'s unset Rbid(2)/RBid(2)";
            }
            return c;
        });
        yield return ("Circ2d2TanOn(curve, point, on)", o =>
        {
            var (a1, n1, p1) = Draw(o);
            var (x, y) = RandomPoint(o);
            var (a3, n3, p3) = Draw(o);
            double tol = o.Bool() ? 1e-7 : 1e-4;
            double u1 = StartParam(o, p1), u3 = StartParam(o, p3);
            var c = new Case
            {
                Ctor = "Circ2d2TanOn(curve, point, on)",
                Description = Describe(a1) + $", point ({x}, {y}), on " + a3.Name,
                Native = () => RunNative(() =>
                {
                    using var anOn = new N.Geom2dAdaptor_Curve(n3);
                    return NCirc2d2TanOn(new N.Geom2dGcc_Circ2d2TanOn(NQ(n1, a1.Q), new N.Geom2d_CartesianPoint(x, y), anOn, tol, u1, u3));
                }),
                Port = () => RunPort(() => PCirc2d2TanOn(new Geom2dGcc_Circ2d2TanOn(PQ(p1, a1.Q), new Geom2d_CartesianPoint(x, y), new Geom2dAdaptor_Curve(p3), tol, u1, u3))),
            };
            if (a1.IsCircle && a3.IsLine)
            {
                // GccAna_Circ2d2TanOn(circle, point, line) leaves pararg1/pararg2 unset on some paths (decision 781).
                c.Masked = n => Starts(n, "t1.", "t2.") && n.EndsWith(".pararg", StringComparison.Ordinal);
            }
            else if (!(a1.IsAnalytic && a3.IsAnalytic))
            {
                c.Masked = n => n.StartsWith("c3.", StringComparison.Ordinal) && n.EndsWith(".pararg", StringComparison.Ordinal);
                // Circ2d2TanOnIter's point constructors give math_FunctionSetRoot 3-vectors for a 4-variable function.
                // Native's math_FunctionSetRoot.cxx compiles its range checks out (No_Standard_OutOfRange): SetTolerance
                // reads past the tolerances and Perform throws Standard_DimensionError. The port's math_Vector checks
                // the read and throws Standard_OutOfRange.
                c.Attribution = "Circ2d2TanOnIter's 3-vectors for 4 variables: native Standard_DimensionError, port Standard_OutOfRange";
                c.Explained = d => d.Count == 1 && d[0] == "ctor: native throws Standard_DimensionError, port throws Standard_OutOfRange";
            }
            return c;
        });
        yield return ("Circ2d2TanOn(point, point, on)", o =>
        {
            var (x1, y1) = RandomPoint(o);
            var (x2, y2) = RandomPoint(o);
            var (a3, n3, p3) = Draw(o);
            double tol = o.Bool() ? 1e-7 : 1e-4;
            return new Case
            {
                Ctor = "Circ2d2TanOn(point, point, on)",
                Description = $"({x1}, {y1}), ({x2}, {y2}), on " + a3.Name,
                Native = () => RunNative(() =>
                {
                    using var anOn = new N.Geom2dAdaptor_Curve(n3);
                    return NCirc2d2TanOn(new N.Geom2dGcc_Circ2d2TanOn(new N.Geom2d_CartesianPoint(x1, y1), new N.Geom2d_CartesianPoint(x2, y2), anOn, tol));
                }),
                Port = () => RunPort(() => PCirc2d2TanOn(new Geom2dGcc_Circ2d2TanOn(new Geom2d_CartesianPoint(x1, y1), new Geom2d_CartesianPoint(x2, y2), new Geom2dAdaptor_Curve(p3), tol))),
            };
        });

        // Circ2d3Tan
        yield return ("Circ2d3Tan(curve, curve, curve)", o =>
        {
            var (a1, n1, p1) = Draw(o);
            var (a2, n2, p2) = Draw(o);
            var (a3, n3, p3) = Draw(o);
            double tol = o.Bool() ? 1e-7 : 1e-4;
            double u1 = StartParam(o, p1), u2 = StartParam(o, p2), u3 = StartParam(o, p3);
            var c = new Case
            {
                Ctor = "Circ2d3Tan(curve, curve, curve)",
                Description = Describe(a1, a2, a3),
                Native = () => RunNative(() => NCirc2d3Tan(new N.Geom2dGcc_Circ2d3Tan(NQ(n1, a1.Q), NQ(n2, a2.Q), NQ(n3, a3.Q), tol, u1, u2, u3))),
                Port = () => RunPort(() => PCirc2d3Tan(new Geom2dGcc_Circ2d3Tan(PQ(p1, a1.Q), PQ(p2, a2.Q), PQ(p3, a3.Q), tol, u1, u2, u3))),
            };
            if (!(a1.IsAnalytic && a2.IsAnalytic && a3.IsAnalytic))
            {
                c.Attribution = "FunctionTanCuCuCu's empty Values: math_FunctionSetRoot starts from its uninitialised F and DF";
            }
            else if (a1.IsLine && a2.IsLine && a3.IsLine)
            {
                c.Attribution = "GccAna_Circ2d3Tan(three lines)'s unset cote3(2) (decision 781)";
            }
            return c;
        });
        yield return ("Circ2d3Tan(curve, curve, point)", o =>
        {
            var (a1, n1, p1) = Draw(o);
            var (a2, n2, p2) = Draw(o);
            var (x, y) = RandomPoint(o);
            double tol = o.Bool() ? 1e-7 : 1e-4;
            double u1 = StartParam(o, p1), u2 = StartParam(o, p2);
            var c = new Case
            {
                Ctor = "Circ2d3Tan(curve, curve, point)",
                Description = Describe(a1, a2) + $", point ({x}, {y})",
                Native = () => RunNative(() => NCirc2d3Tan(new N.Geom2dGcc_Circ2d3Tan(NQ(n1, a1.Q), NQ(n2, a2.Q), new N.Geom2d_CartesianPoint(x, y), tol, u1, u2))),
                Port = () => RunPort(() => PCirc2d3Tan(new Geom2dGcc_Circ2d3Tan(PQ(p1, a1.Q), PQ(p2, a2.Q), new Geom2d_CartesianPoint(x, y), tol, u1, u2))),
            };
            if (!(a1.IsAnalytic && a2.IsAnalytic))
            {
                c.Attribution = "FunctionTanCuCuCu's empty Values: math_FunctionSetRoot starts from its uninitialised F and DF";
            }
            return c;
        });
        yield return ("Circ2d3Tan(curve, point, point)", o =>
        {
            var (a1, n1, p1) = Draw(o);
            var (x1, y1) = RandomPoint(o);
            var (x2, y2) = RandomPoint(o);
            double tol = o.Bool() ? 1e-7 : 1e-4;
            double u1 = StartParam(o, p1);
            var c = new Case
            {
                Ctor = "Circ2d3Tan(curve, point, point)",
                Description = Describe(a1) + $", ({x1}, {y1}), ({x2}, {y2})",
                Native = () => RunNative(() => NCirc2d3Tan(new N.Geom2dGcc_Circ2d3Tan(NQ(n1, a1.Q), new N.Geom2d_CartesianPoint(x1, y1), new N.Geom2d_CartesianPoint(x2, y2), tol, u1))),
                Port = () => RunPort(() => PCirc2d3Tan(new Geom2dGcc_Circ2d3Tan(PQ(p1, a1.Q), new Geom2d_CartesianPoint(x1, y1), new Geom2d_CartesianPoint(x2, y2), tol, u1))),
            };
            if (!a1.IsAnalytic)
            {
                c.Attribution = "FunctionTanCuCuCu's empty Values: math_FunctionSetRoot starts from its uninitialised F and DF";
            }
            return c;
        });
        yield return ("Circ2d3Tan(point, point, point)", o =>
        {
            var (x1, y1) = RandomPoint(o);
            var (x2, y2) = RandomPoint(o);
            var (x3, y3) = RandomPoint(o);
            double tol = o.Bool() ? 1e-7 : 1e-4;
            return new Case
            {
                Ctor = "Circ2d3Tan(point, point, point)",
                Description = $"({x1}, {y1}), ({x2}, {y2}), ({x3}, {y3})",
                Native = () => RunNative(() => NCirc2d3Tan(new N.Geom2dGcc_Circ2d3Tan(new N.Geom2d_CartesianPoint(x1, y1), new N.Geom2d_CartesianPoint(x2, y2), new N.Geom2d_CartesianPoint(x3, y3), tol))),
                Port = () => RunPort(() => PCirc2d3Tan(new Geom2dGcc_Circ2d3Tan(new Geom2d_CartesianPoint(x1, y1), new Geom2d_CartesianPoint(x2, y2), new Geom2d_CartesianPoint(x3, y3), tol))),
            };
        });

        // Circ2dTanCen
        yield return ("Circ2dTanCen(curve, centre)", o =>
        {
            var (a1, n1, p1) = Draw(o);
            var (x, y) = RandomPoint(o);
            double tol = o.Bool() ? 1e-7 : 1e-4;
            var c = new Case
            {
                Ctor = "Circ2dTanCen(curve, centre)",
                Description = Describe(a1) + $", centre ({x}, {y})",
                Native = () => RunNative(() => NCirc2dTanCen(new N.Geom2dGcc_Circ2dTanCen(NQ(n1, a1.Q), new N.Geom2d_CartesianPoint(x, y), tol))),
                Port = () => RunPort(() => PCirc2dTanCen(new Geom2dGcc_Circ2dTanCen(PQ(p1, a1.Q), new Geom2d_CartesianPoint(x, y), tol))),
            };
            if (!a1.IsAnalytic)
            {
                c.Attribution = "Circ2dTanCenGeo's uninitialised Index array (one or two solutions)";
                // The port (Index 0 and 0) keeps the nearest extremum only; native, when its Index differ, also the
                // farthest: native then has solutions the port hasn't, never other values for the same ones.
                c.Explained = d => d.All(x => x.StartsWith("nb:", StringComparison.Ordinal) || x.StartsWith("done:", StringComparison.Ordinal)
                                               || x.EndsWith("port -", StringComparison.Ordinal));
            }
            return c;
        });

        // Circ2dTanOnRad
        yield return ("Circ2dTanOnRad(curve, on)", o =>
        {
            var (a1, n1, p1) = Draw(o);
            var (a2, n2, p2) = Draw(o);
            double r = o.Real(0.2, 6), tol = o.Bool() ? 1e-7 : 1e-4;
            return new Case
            {
                Ctor = "Circ2dTanOnRad(curve, on)",
                Description = Describe(a1) + ", on " + a2.Name + $", r {r}",
                Native = () => RunNative(() =>
                {
                    using var anOn = new N.Geom2dAdaptor_Curve(n2);
                    return NCirc2dTanOnRad(new N.Geom2dGcc_Circ2dTanOnRad(NQ(n1, a1.Q), anOn, r, tol));
                }),
                Port = () => RunPort(() => PCirc2dTanOnRad(new Geom2dGcc_Circ2dTanOnRad(PQ(p1, a1.Q), new Geom2dAdaptor_Curve(p2), r, tol))),
            };
        });
        yield return ("Circ2dTanOnRad(point, on)", o =>
        {
            var (x, y) = RandomPoint(o);
            var (a2, n2, p2) = Draw(o);
            double r = o.Real(0.2, 6), tol = o.Bool() ? 1e-7 : 1e-4;
            var c = new Case
            {
                Ctor = "Circ2dTanOnRad(point, on)",
                Description = $"point ({x}, {y}), on " + a2.Name + $", r {r}",
                Native = () => RunNative(() =>
                {
                    using var anOn = new N.Geom2dAdaptor_Curve(n2);
                    return NCirc2dTanOnRad(new N.Geom2dGcc_Circ2dTanOnRad(new N.Geom2d_CartesianPoint(x, y), anOn, r, tol));
                }),
                Port = () => RunPort(() => PCirc2dTanOnRad(new Geom2dGcc_Circ2dTanOnRad(new Geom2d_CartesianPoint(x, y), new Geom2dAdaptor_Curve(p2), r, tol))),
            };
            if (a2.IsCircle)
            {
                // GccAna_Circ2dTanOnRad(point, circle) writes its second solution into the first (decision 781).
                c.Masked = n => n.Contains(".2", StringComparison.Ordinal);
            }
            return c;
        });

        // Lin2d2Tan
        yield return ("Lin2d2Tan(curve, curve)", o =>
        {
            var (a1, n1, p1) = Draw(o);
            var (a2, n2, p2) = Draw(o);
            double tol = o.Bool() ? 1e-7 : 1e-4;
            return new Case
            {
                Ctor = "Lin2d2Tan(curve, curve)",
                Description = Describe(a1, a2),
                Native = () => RunNative(() => NLin2d2Tan(new N.Geom2dGcc_Lin2d2Tan(NQ(n1, a1.Q), NQ(n2, a2.Q), tol))),
                Port = () => RunPort(() => PLin2d2Tan(new Geom2dGcc_Lin2d2Tan(PQ(p1, a1.Q), PQ(p2, a2.Q), tol))),
            };
        });
        yield return ("Lin2d2Tan(curve, point)", o =>
        {
            var (a1, n1, p1) = Draw(o);
            var (x, y) = RandomPoint(o);
            double tol = o.Bool() ? 1e-7 : 1e-4;
            return new Case
            {
                Ctor = "Lin2d2Tan(curve, point)",
                Description = Describe(a1) + $", point ({x}, {y})",
                Native = () => RunNative(() =>
                {
                    using var aP = new N.gp_Pnt2d(x, y);
                    return NLin2d2Tan(new N.Geom2dGcc_Lin2d2Tan(NQ(n1, a1.Q), aP, tol));
                }),
                Port = () => RunPort(() => PLin2d2Tan(new Geom2dGcc_Lin2d2Tan(PQ(p1, a1.Q), new gp_Pnt2d(x, y), tol))),
            };
        });
        yield return ("Lin2d2Tan(curve, curve, params)", o =>
        {
            var (a1, n1, p1) = Draw(o);
            var (a2, n2, p2) = Draw(o);
            double tol = o.Bool() ? 1e-7 : 1e-4;
            double u1 = StartParam(o, p1), u2 = StartParam(o, p2);
            return new Case
            {
                Ctor = "Lin2d2Tan(curve, curve, params)",
                Description = Describe(a1, a2) + $", from {u1}, {u2}",
                Native = () => RunNative(() => NLin2d2Tan(new N.Geom2dGcc_Lin2d2Tan(NQ(n1, a1.Q), NQ(n2, a2.Q), tol, u1, u2))),
                Port = () => RunPort(() => PLin2d2Tan(new Geom2dGcc_Lin2d2Tan(PQ(p1, a1.Q), PQ(p2, a2.Q), tol, u1, u2))),
            };
        });
        yield return ("Lin2d2Tan(curve, point, param)", o =>
        {
            var (a1, n1, p1) = Draw(o);
            var (x, y) = RandomPoint(o);
            double tol = o.Bool() ? 1e-7 : 1e-4;
            double u1 = StartParam(o, p1);
            return new Case
            {
                Ctor = "Lin2d2Tan(curve, point, param)",
                Description = Describe(a1) + $", point ({x}, {y}), from {u1}",
                Native = () => RunNative(() =>
                {
                    using var aP = new N.gp_Pnt2d(x, y);
                    return NLin2d2Tan(new N.Geom2dGcc_Lin2d2Tan(NQ(n1, a1.Q), aP, tol, u1));
                }),
                Port = () => RunPort(() => PLin2d2Tan(new Geom2dGcc_Lin2d2Tan(PQ(p1, a1.Q), new gp_Pnt2d(x, y), tol, u1))),
            };
        });

        // Lin2dTanObl
        foreach (bool aWithParam in new[] { false, true })
        {
            string aName = aWithParam ? "Lin2dTanObl(curve, line, param, angle)" : "Lin2dTanObl(curve, line, angle)";
            yield return (aName, o =>
            {
                var (a1, n1, p1) = Draw(o);
                var (x, y) = RandomPoint(o);
                var (dx, dy) = RawDir(o);
                double angle = o.Int(0, 4) == 0 ? o.Int(0, 3) * Math.PI / 2 : o.Angle();
                double tolang = o.Bool() ? 1e-7 : 1e-4;
                double u1 = StartParam(o, p1);
                return new Case
                {
                    Ctor = aName,
                    Description = Describe(a1) + $", line ({x}, {y}) ({dx}, {dy}), angle {angle}",
                    Native = () => RunNative(() =>
                    {
                        using var aP = new N.gp_Pnt2d(x, y);
                        using var aD = new N.gp_Dir2d(dx, dy);
                        using var aL = new N.gp_Lin2d(aP, aD);
                        return NLin2dTanObl(aWithParam ? new N.Geom2dGcc_Lin2dTanObl(NQ(n1, a1.Q), aL, tolang, u1, angle)
                                                       : new N.Geom2dGcc_Lin2dTanObl(NQ(n1, a1.Q), aL, tolang, angle));
                    }),
                    Port = () => RunPort(() =>
                    {
                        var aL = new gp_Lin2d(new gp_Pnt2d(x, y), new gp_Dir2d(dx, dy));
                        return PLin2dTanObl(aWithParam ? new Geom2dGcc_Lin2dTanObl(PQ(p1, a1.Q), aL, tolang, u1, angle)
                                                       : new Geom2dGcc_Lin2dTanObl(PQ(p1, a1.Q), aL, tolang, angle));
                    }),
                };
            });
        }
    }

    // ---------------------------------------------------------------------------------------------------------------
    // The comparison

    /// <summary>The names whose values differ (masked names excluded), or that one side has and the other hasn't.</summary>
    internal static List<string> Differences(Rec theNative, Rec thePort, Func<string, bool> theMasked, out int theCompared, out int theMaskedCount)
    {
        var aN = theNative.T.ToDictionary(t => t.Name, t => t.Value);
        var aP = thePort.T.ToDictionary(t => t.Name, t => t.Value);
        var aDiff = new List<string>();
        theCompared = 0;
        theMaskedCount = 0;
        foreach (string aName in aN.Keys.Union(aP.Keys))
        {
            if (theMasked(aName))
            {
                theMaskedCount++;
                continue;
            }
            theCompared++;
            aN.TryGetValue(aName, out string? aNV);
            aP.TryGetValue(aName, out string? aPV);
            if (aNV != aPV)
            {
                aDiff.Add($"{aName}: native {aNV ?? "-"}, port {aPV ?? "-"}");
            }
        }
        return aDiff;
    }

    [Fact]
    public void Solvers_match_native_token_for_token()
    {
        var anO = new Oracle(output, 20261030);
        int aTotalCases = 0, aTotalIdentical = 0, aTotalTokens = 0, aTotalMasked = 0, aTotalWithSolutions = 0;
        var anAttributed = new Dictionary<string, (int Cases, int NativeRepeats)>();
        var aFailures = new List<string>();
        foreach (var (aCtor, aMake) in Constructors())
        {
            int aCases = 0, anIdentical = 0, aWithSolutions = 0, aThrows = 0;
            for (int i = 0; i < CasesPerConstructor; i++)
            {
                Case aCase = aMake(anO);
                Rec aN = aCase.Native();
                Rec aP = aCase.Port();
                aCases++;
                if (aN.T.Any(t => t.Name == "nb" && t.Value != "0"))
                {
                    aWithSolutions++;
                }
                if (aN.T.Count == 1 && aN.T[0].Name == "ctor")
                {
                    aThrows++;
                }
                List<string> aDiff = Differences(aN, aP, aCase.Masked, out int aCompared, out int aMaskedCount);
                aTotalTokens += aCompared;
                aTotalMasked += aMaskedCount;
                if (aDiff.Count == 0)
                {
                    anIdentical++;
                    continue;
                }
                if (aCase.Attribution is not null && aCase.Explained(aDiff))
                {
                    // Native again on the same inputs: whether its result repeats is recorded, not required.
                    Rec aN2 = aCase.Native();
                    bool aRepeats = Differences(aN, aN2, aCase.Masked, out _, out _).Count == 0;
                    anAttributed.TryGetValue(aCase.Attribution, out var aCount);
                    anAttributed[aCase.Attribution] = (aCount.Cases + 1, aCount.NativeRepeats + (aRepeats ? 1 : 0));
                    continue;
                }
                aFailures.Add($"{aCtor} #{i} [{aCase.Description}]: {string.Join("; ", aDiff.Take(6))}");
            }
            output.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"{aCtor}: {anIdentical}/{aCases} cases identical, {aWithSolutions} with solutions, {aThrows} throwing on both sides or natively"));
            aTotalCases += aCases;
            aTotalIdentical += anIdentical;
            aTotalWithSolutions += aWithSolutions;
        }
        output.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"Total: {aTotalIdentical}/{aTotalCases} cases identical ({aTotalWithSolutions} with solutions), {aTotalTokens} tokens compared, {aTotalMasked} masked"));
        foreach (var (aWhy, aCount) in anAttributed)
        {
            output.WriteLine($"Attributed: {aCount.Cases} cases ({aCount.NativeRepeats} where native repeats itself): {aWhy}");
        }
        foreach (string aF in aFailures.Take(40))
        {
            output.WriteLine(aF);
        }
        Assert.True(aFailures.Count == 0, $"{aFailures.Count} unattributed differences; first: {aFailures.FirstOrDefault()}");
    }
}
#endif
