// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using System.Globalization;
using System.Text;
using static Occt.ChFiDS_State;
using static Kerf.Engine.Geometry.Conversion.ParameterisationType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// tools/blendrec's main.cpp in C#, line for line: the same blend functions and spines, the same calls in the same
/// order, written in the same text (every double as its 64 bits in hex), so the tests compare the port's text with
/// native OCCT's recording.
/// </summary>
internal sealed partial class BlendRecorder
{
    private readonly StringBuilder gOut = new();

    public string Text => gOut.ToString();

    private void Hex(double theValue) =>
        gOut.Append(' ').Append(BitConverter.DoubleToUInt64Bits(theValue).ToString("x16", CultureInfo.InvariantCulture));

    private void Tag(string theTag) => gOut.Append(theTag);

    private void Int(int theValue) => gOut.Append(' ').Append(theValue.ToString(CultureInfo.InvariantCulture));

    private void End() => gOut.Append('\n');

    private void Line(string theText) => gOut.Append(theText).Append('\n');

    private void Pnt(Point3D P)
    {
        Hex(P.X());
        Hex(P.Y());
        Hex(P.Z());
    }

    private void Vec(Vector3D V)
    {
        Hex(V.X());
        Hex(V.Y());
        Hex(V.Z());
    }

    private void Pnt2d(Point2D P)
    {
        Hex(P.X());
        Hex(P.Y());
    }

    private void Vec2d(Vector2D V)
    {
        Hex(V.X());
        Hex(V.Y());
    }

    private void Vector(string theTag, math_Vector V)
    {
        Tag(theTag);
        for (int i = V.Lower(); i <= V.Upper(); i++)
        {
            Hex(V[i]);
        }

        End();
    }

    private void Matrix(string theTag, Matrix M)
    {
        Tag(theTag);
        for (int i = M.LowerRow(); i <= M.UpperRow(); i++)
        {
            for (int j = M.LowerCol(); j <= M.UpperCol(); j++)
            {
                Hex(M[i, j]);
            }
        }

        End();
    }

    private void Fail(string theTag)
    {
        Tag(theTag);
        Tag(" X");
        End();
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Geometry

    internal static Geom_BSplineSurface Patch()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 4, 1, 4);
        double[,] aZ =
        {
            { 0.10, -0.20, 0.30, 0.00 },
            { -0.30, 0.40, -0.10, 0.20 },
            { 0.20, -0.10, 0.50, -0.30 },
            { 0.00, 0.30, -0.20, 0.10 },
        };
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aPoles[i, j] = new Point3D(3.0 * (i - 1) / 3.0, 3.0 * (j - 1) / 3.0, aZ[i - 1, j - 1]);
            }
        }

        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        return new Geom_BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3);
    }

    internal static Geom_BSplineSurface Wall()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        for (int i = 1; i <= 2; i++)
        {
            for (int j = 1; j <= 2; j++)
            {
                aPoles[i, j] = new Point3D(1.5, -1.0 + 5.0 * (i - 1), -2.0 + 4.0 * (j - 1));
            }
        }

        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 2;
        aMults[2] = 2;
        return new Geom_BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 1, 1);
    }

    internal static Adaptor3d_Surface Surf(int theId)
    {
        Geom_Surface aS = theId switch
        {
            0 => new Geom_Plane(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0))),
            1 => new Geom_Plane(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), new Direction3D(1, 0, 0))),
            2 => new Geom_CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5.0),
            3 => new Geom_CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 3.0),
            4 => new Geom_CylindricalSurface(new CoordinateSystem(new Point3D(5, 0, 0), new Direction3D(0, 0, 1)), 3.0),
            5 => new Geom_ConicalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), Math.PI / 6.0, 5.0),
            6 => Patch(),
            _ => Wall(),
        };
        return new GeomAdaptor_Surface(aS);
    }

    internal static Adaptor3d_Curve Guide(int theId)
    {
        return theId switch
        {
            0 => new GeomAdaptor_Curve(new Geom_Line(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0.0, 10.0),
            1 => new GeomAdaptor_Curve(new Geom_Circle(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5.0), 0.0,
                                       2.0 * Math.PI),
            2 => new GeomAdaptor_Curve(new Geom_Line(new Point3D(2.5, Math.Sqrt(2.75), 0), new Direction3D(0, 0, 1)), -5.0, 5.0),
            3 => new GeomAdaptor_Curve(new Geom_Line(new Point3D(1.5, 0, 0), new Direction3D(0, 1, 0)), 0.0, 3.0),
            _ => new GeomAdaptor_Curve(new Geom_Line(new Point3D(0, -1, 1.5), new Direction3D(1, 0, 0)), -2.0, 12.0),
        };
    }

    internal sealed record PairCase(int S1, int S2, int G, double W, double[] X0);

    internal static readonly PairCase[] THE_PAIRS =
    [
        new(0, 1, 0, 4.0, [4.0, 1.0, 4.0, -1.0]), // plane / plane
        new(0, 2, 1, 1.0, [4.0 * 0.5403023058681398, 4.0 * 0.8414709848078965, 1.0, 1.0]), // plane / cylinder
        new(3, 4, 2, 0.5, [0.2857, 0.5, 2.8559, 0.5]), // cylinder / cylinder
        new(5, 0, 1, 1.0, [1.0, 1.0, 4.0 * 0.5403023058681398, 4.0 * 0.8414709848078965]), // cone / plane
        new(6, 7, 3, 1.5, [0.83, 0.5, 0.5, 0.75]), // B-spline / B-spline
    ];

    // -----------------------------------------------------------------------------------------------------------------
    // Functions

    private bool Solve(FunctionSetWithDerivatives F, math_Vector X0, math_Vector Tol, math_Vector Inf,
                       math_Vector Sup, math_Vector Sol)
    {
        FunctionSetRoot aRoot = new(F, Tol, 30);
        aRoot.Perform(F, X0, Inf, Sup);
        if (!aRoot.IsDone)
        {
            Tag("root 0");
            End();
            return false;
        }

        aRoot.Root(Sol);
        Vector("root 1", Sol);
        return true;
    }

    private void RecValues(FunctionSetWithDerivatives F, math_Vector X)
    {
        int n = F.NbEquations(), m = F.NbVariables();
        math_Vector aF = new(1, n);
        Matrix aD = new(1, n, 1, m);
        Tag("dims");
        Int(n);
        Int(m);
        End();
        try
        {
            bool isOk = F.Value(X, aF);
            Tag("ok");
            Int(isOk ? 1 : 0);
            End();
            Vector("value", aF);
        }
        catch (FailureException)
        {
            Fail("value");
        }

        try
        {
            F.Derivatives(X, aD);
            Matrix("derivatives", aD);
        }
        catch (FailureException)
        {
            Fail("derivatives");
        }

        try
        {
            F.Values(X, aF, aD);
            Vector("values.f", aF);
            Matrix("values.d", aD);
        }
        catch (FailureException)
        {
            Fail("values");
        }
    }

    private void RecApp(Blend_AppFunction F)
    {
        F.GetShape(out int aNbPoles, out int aNbKnots, out int aDegree, out int aNbPoles2d);
        Tag("shape");
        Int(aNbPoles);
        Int(aNbKnots);
        Int(aDegree);
        Int(aNbPoles2d);
        Int(F.IsRational() ? 1 : 0);
        End();
        var aKnots = new NCollection_Array1<double>(1, aNbKnots);
        var aMults = new NCollection_Array1<int>(1, aNbKnots);
        F.Knots(aKnots);
        F.Mults(aMults);
        Tag("knots");
        for (int i = 1; i <= aNbKnots; i++)
        {
            Hex(aKnots[i]);
            Int(aMults[i]);
        }

        End();
        var aW = new NCollection_Array1<double>(1, aNbPoles);
        F.GetMinimalWeight(aW);
        Tag("minweight");
        for (int i = 1; i <= aNbPoles; i++)
        {
            Hex(aW[i]);
        }

        End();
        try
        {
            double aSize = F.GetSectionSize();
            Tag("sectionsize");
            Hex(aSize);
            End();
        }
        catch (FailureException)
        {
            Fail("sectionsize");
        }

        try
        {
            double aDist = F.GetMinimalDistance();
            Tag("mindist");
            Hex(aDist);
            End();
        }
        catch (FailureException)
        {
            Fail("mindist");
        }

        math_Vector aTol3d = new(1, aNbPoles, 0.0), aTol1d = new(1, aNbPoles, 0.0); // GetTolerance may leave Tol1d unset
        try
        {
            F.GetTolerance(1.0e-4, 1.0e-5, 1.0e-3, aTol3d, aTol1d);
            Vector("tol3d", aTol3d);
            Vector("tol1d", aTol1d);
        }
        catch (FailureException)
        {
            Fail("tol3d");
        }

        int aNbInt = F.NbIntervals(C1);
        var aT = new NCollection_Array1<double>(1, aNbInt + 1);
        F.Intervals(aT, C1);
        Tag("intervals");
        for (int i = 1; i <= aNbInt + 1; i++)
        {
            Hex(aT[i]);
        }

        End();
        F.Resolution(1, 1.0e-3, out double aTolU, out double aTolV);
        Tag("resolution");
        Hex(aTolU);
        Hex(aTolV);
        End();
    }

    private void RecSections(Blend_AppFunction F, in Blend_Point P, int theNbPoles2d)
    {
        F.GetShape(out int aNbPoles, out _, out _, out _);
        var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
        var aDPoles = new NCollection_Array1<Vector3D>(1, aNbPoles);
        var aD2Poles = new NCollection_Array1<Vector3D>(1, aNbPoles);
        var aPoles2d = new NCollection_Array1<Point2D>(1, theNbPoles2d);
        var aDPoles2d = new NCollection_Array1<Vector2D>(1, theNbPoles2d);
        var aD2Poles2d = new NCollection_Array1<Vector2D>(1, theNbPoles2d);
        var aW = new NCollection_Array1<double>(1, aNbPoles);
        var aDW = new NCollection_Array1<double>(1, aNbPoles);
        var aD2W = new NCollection_Array1<double>(1, aNbPoles);
        try
        {
            F.Section(P, aPoles, aPoles2d, aW);
            Tag("section0");
            for (int i = 1; i <= aNbPoles; i++)
            {
                Pnt(aPoles[i]);
                Hex(aW[i]);
            }

            for (int i = 1; i <= theNbPoles2d; i++)
            {
                Pnt2d(aPoles2d[i]);
            }

            End();
        }
        catch (FailureException)
        {
            Fail("section0");
        }

        try
        {
            bool isD = F.Section(P, aPoles, aDPoles, aPoles2d, aDPoles2d, aW, aDW);
            Tag("section1");
            Int(isD ? 1 : 0);
            for (int i = 1; i <= aNbPoles; i++)
            {
                Pnt(aPoles[i]);
                Hex(aW[i]);
                if (isD)
                {
                    Vec(aDPoles[i]);
                    Hex(aDW[i]);
                }
            }

            for (int i = 1; i <= theNbPoles2d; i++)
            {
                Pnt2d(aPoles2d[i]);
                if (isD)
                {
                    Vec2d(aDPoles2d[i]);
                }
            }

            End();
        }
        catch (FailureException)
        {
            Fail("section1");
        }

        try
        {
            bool isD = F.Section(P, aPoles, aDPoles, aD2Poles, aPoles2d, aDPoles2d, aD2Poles2d, aW, aDW, aD2W);
            Tag("section2");
            Int(isD ? 1 : 0);
            if (isD)
            {
                for (int i = 1; i <= aNbPoles; i++)
                {
                    Pnt(aPoles[i]);
                    Vec(aDPoles[i]);
                    Vec(aD2Poles[i]);
                    Hex(aW[i]);
                    Hex(aDW[i]);
                    Hex(aD2W[i]);
                }

                for (int i = 1; i <= theNbPoles2d; i++)
                {
                    Pnt2d(aPoles2d[i]);
                    Vec2d(aDPoles2d[i]);
                    Vec2d(aD2Poles2d[i]);
                }
            }

            End();
        }
        catch (FailureException)
        {
            Fail("section2");
        }
    }

    private void RecFunction(Blend_Function F, PairCase C)
    {
        F.Set(C.W);
        math_Vector X0 = new(1, 4), Tol = new(1, 4), Inf = new(1, 4), Sup = new(1, 4), Sol = new(1, 4);
        for (int i = 1; i <= 4; i++)
        {
            X0[i] = C.X0[i - 1];
        }

        RecValues(F, X0);
        F.GetTolerance(Tol, 1.0e-7);
        Vector("tolerance", Tol);
        F.GetBounds(Inf, Sup);
        Vector("inf", Inf);
        Vector("sup", Sup);
        if (!Solve(F, X0, Tol, Inf, Sup, Sol))
        {
            return;
        }

        bool isSol;
        try
        {
            isSol = F.IsSolution(Sol, 1.0e-6);
        }
        catch (FailureException)
        {
            Fail("issolution");
            return;
        }

        Tag("issolution");
        Int(isSol ? 1 : 0);
        Int(F.IsTangencyPoint() ? 1 : 0);
        Pnt(F.PointOnS1());
        Pnt(F.PointOnS2());
        End();
        if (!F.IsTangencyPoint())
        {
            Tag("tangents");
            Vec(F.TangentOnS1());
            Vec(F.TangentOnS2());
            Vec2d(F.Tangent2dOnS1());
            Vec2d(F.Tangent2dOnS2());
            try
            {
                int t1 = F.TwistOnS1() ? 1 : 0;
                Int(t1);
                Int(F.TwistOnS2() ? 1 : 0);
            }
            catch (FailureException)
            {
                Tag(" X");
            }

            End();
        }

        F.Tangent(Sol[1], Sol[2], Sol[3], Sol[4], out Vector3D aTgF, out Vector3D aTgL, out Vector3D aNmF, out Vector3D aNmL);
        Tag("tangent");
        Vec(aTgF);
        Vec(aTgL);
        Vec(aNmF);
        Vec(aNmL);
        End();
        RecApp(F);
        Blend_Point aP;
        if (F.IsTangencyPoint())
        {
            aP = new Blend_Point(F.PointOnS1(), F.PointOnS2(), C.W, Sol[1], Sol[2], Sol[3], Sol[4]);
        }
        else
        {
            aP = new Blend_Point(F.PointOnS1(), F.PointOnS2(), C.W, Sol[1], Sol[2], Sol[3], Sol[4], F.TangentOnS1(),
                                 F.TangentOnS2(), F.Tangent2dOnS1(), F.Tangent2dOnS2());
        }

        RecSections(F, aP, 2);
    }

    private void RecInv(Blend_FuncInv F, math_Vector X0)
    {
        math_Vector Tol = new(1, 4), Inf = new(1, 4), Sup = new(1, 4), Sol = new(1, 4);
        RecValues(F, X0);
        F.GetTolerance(Tol, 1.0e-7);
        Vector("tolerance", Tol);
        F.GetBounds(Inf, Sup);
        Vector("inf", Inf);
        Vector("sup", Sup);
        if (!Solve(F, X0, Tol, Inf, Sup, Sol))
        {
            return;
        }

        bool isSol;
        try
        {
            isSol = F.IsSolution(Sol, 1.0e-6);
        }
        catch (FailureException)
        {
            Fail("issolution");
            return;
        }

        Tag("issolution");
        Int(isSol ? 1 : 0);
        End();
    }

    private void RecCS(Blend_CSFunction F, double theW, math_Vector X0)
    {
        int m = F.NbVariables();
        math_Vector Tol = new(1, m), Inf = new(1, m), Sup = new(1, m), Sol = new(1, m);
        F.Set(theW);
        RecValues(F, X0);
        F.GetTolerance(Tol, 1.0e-7);
        Vector("tolerance", Tol);
        F.GetBounds(Inf, Sup);
        Vector("inf", Inf);
        Vector("sup", Sup);
        if (!Solve(F, X0, Tol, Inf, Sup, Sol))
        {
            return;
        }

        bool isSol = F.IsSolution(Sol, 1.0e-6);
        Tag("issolution");
        Int(isSol ? 1 : 0);
        Int(F.IsTangencyPoint() ? 1 : 0);
        Pnt(F.PointOnS());
        Pnt(F.PointOnC());
        Pnt2d(F.Pnt2d());
        Hex(F.ParameterOnC());
        End();
        if (!F.IsTangencyPoint())
        {
            Tag("tangents");
            Vec(F.TangentOnS());
            Vec(F.TangentOnC());
            Vec2d(F.Tangent2d());
            End();
        }

        F.Tangent(Sol[1], Sol[2], out Vector3D aTgS, out Vector3D aNmS);
        Tag("tangent");
        Vec(aTgS);
        Vec(aNmS);
        End();
        RecApp(F);
        double aW = (m == 3) ? Sol[3] : F.ParameterOnC();
        Blend_Point aP;
        if (F.IsTangencyPoint())
        {
            aP = new Blend_Point(F.PointOnS(), F.PointOnC(), theW, Sol[1], Sol[2], aW);
        }
        else
        {
            aP = new Blend_Point(F.PointOnS(), F.PointOnC(), theW, Sol[1], Sol[2], aW, F.TangentOnS(), F.TangentOnC(),
                                 F.Tangent2d());
        }

        RecSections(F, aP, 1);
    }

    internal void RecordFunctions()
    {
        for (int ip = 0; ip < THE_PAIRS.Length; ip++)
        {
            PairCase C = THE_PAIRS[ip];
            Adaptor3d_Surface S1 = Surf(C.S1), S2 = Surf(C.S2);
            Adaptor3d_Curve G = Guide(C.G);
            for (int choix = 1; choix <= 8; choix++)
            {
                Line($"case constrad {ip} {choix}");
                BlendFunc_ConstRad aConstRad = new(S1, S2, G);
                aConstRad.Set(1.0, choix);
                aConstRad.Set(G.FirstParameter(), G.LastParameter());
                RecFunction(aConstRad, C);
                Circle3D aCirc = new();
                aConstRad.Section(C.W, C.X0[0], C.X0[1], C.X0[2], C.X0[3], out double aPdeb, out double aPfin, ref aCirc);
                Tag("circle");
                Pnt(aCirc.Location());
                Vec(new Vector3D(aCirc.Axis().Direction()));
                Vec(new Vector3D(aCirc.XAxis().Direction()));
                Hex(aCirc.Radius());
                Hex(aPdeb);
                Hex(aPfin);
                End();
                Axis3D anAx = aConstRad.AxeRot(C.W);
                Tag("axerot");
                Pnt(anAx.Location());
                Vec(new Vector3D(anAx.Direction()));
                End();

                Line($"case evolrad {ip} {choix}");
                Law_Linear aLaw = new();
                aLaw.Set(G.FirstParameter(), 0.8, G.LastParameter(), 1.4);
                Law_Composite aComp = new(G.FirstParameter(), G.LastParameter(), 1.0e-9);
                aComp.ChangeLaws().Append(aLaw);
                BlendFunc_EvolRad anEvolRad = new(S1, S2, G, aComp);
                anEvolRad.Set(choix);
                anEvolRad.Set(G.FirstParameter(), G.LastParameter());
                RecFunction(anEvolRad, C);
            }

            for (int choix = 1; choix <= 2; choix++)
            {
                Line($"case chamfer {ip} {choix}");
                BlendFunc_Chamfer aChamfer = new(S1, S2, G);
                aChamfer.Set(1.0, 1.5, choix);
                RecFunction(aChamfer, C);

                Line($"case constthroat {ip} {choix}");
                BlendFunc_ConstThroat aThroat = new(S1, S2, G);
                aThroat.Set(0.7, 0.0, choix);
                RecFunction(aThroat, C);

                Line($"case constthroatpen {ip} {choix}");
                BlendFunc_ConstThroatWithPenetration aPen = new(S1, S2, G);
                aPen.Set(1.0, 0.0, choix);
                RecFunction(aPen, C);

                Line($"case chasym {ip} {choix}");
                BlendFunc_ChAsym aChAsym = new(S1, S2, G);
                aChAsym.Set(1.0, Math.PI / 3.0, choix);
                aChAsym.Set(G.FirstParameter(), G.LastParameter());
                RecFunction(aChAsym, C);

                Line($"case ruled {ip} {choix}");
                BlendFunc_Ruled aRuled = new(S1, S2, G);
                RecFunction(aRuled, C);
                var aTabP = new NCollection_Array1<Point3D>(1, 5);
                var aTabV = new NCollection_Array1<Vector3D>(1, 5);
                try
                {
                    bool isOk = aRuled.GetSection(C.W, C.X0[0], C.X0[1], C.X0[2], C.X0[3], aTabP, aTabV);
                    Tag("getsection");
                    Int(isOk ? 1 : 0);
                    if (isOk)
                    {
                        for (int i = 1; i <= 5; i++)
                        {
                            Pnt(aTabP[i]);
                            Vec(aTabV[i]);
                        }
                    }

                    End();
                }
                catch (FailureException)
                {
                    Fail("getsection");
                }
            }
        }

        (int Pair, bool OnFirst, double[] P0, double[] D0, double[] X0)[] THE_INVS =
        [
            (0, true, [5.0, 0.0], [0.0, 1.0], [1.0, 5.0, 5.0, -1.0]),
            (0, false, [5.0, 0.0], [0.0, 1.0], [-1.0, 5.0, 5.0, 1.0]),
            (1, true, [0.0, 0.0], [0.3623577544766736, 0.9320390859672263], [4.0, 1.2, 1.2, 1.0]),
            (4, true, [0.0, 0.5], [1.0, 0.0], [0.83, 1.5, 0.5, 0.75]),
        ];
        for (int ii = 0; ii < THE_INVS.Length; ii++)
        {
            var I = THE_INVS[ii];
            PairCase C = THE_PAIRS[I.Pair];
            Adaptor3d_Surface S1 = Surf(C.S1), S2 = Surf(C.S2);
            Adaptor3d_Curve G = Guide(C.G);
            Adaptor2d_Curve2d R = new Geom2dAdaptor_Curve(
                new Geom2d_Line(new Point2D(I.P0[0], I.P0[1]), new Direction2D(I.D0[0], I.D0[1])), -10.0, 10.0);
            math_Vector X0 = new(1, 4);
            for (int i = 1; i <= 4; i++)
            {
                X0[i] = I.X0[i - 1];
            }

            for (int choix = 1; choix <= 8; choix += 3)
            {
                Line($"case constradinv {ii} {choix}");
                BlendFunc_ConstRadInv aConstRad = new(S1, S2, G);
                aConstRad.Set(1.0, choix);
                aConstRad.Set(I.OnFirst, R);
                RecInv(aConstRad, X0);

                Line($"case evolradinv {ii} {choix}");
                Law_Linear aLaw = new();
                aLaw.Set(G.FirstParameter(), 0.8, G.LastParameter(), 1.4);
                BlendFunc_EvolRadInv anEvolRad = new(S1, S2, G, aLaw);
                anEvolRad.Set(choix);
                anEvolRad.Set(I.OnFirst, R);
                RecInv(anEvolRad, X0);

                Line($"case chamfinv {ii} {choix}");
                BlendFunc_ChamfInv aChamf = new(S1, S2, G);
                aChamf.Set(1.0, 1.5, choix);
                aChamf.Set(I.OnFirst, R);
                RecInv(aChamf, X0);

                Line($"case constthroatinv {ii} {choix}");
                BlendFunc_ConstThroatInv aThroat = new(S1, S2, G);
                aThroat.Set(0.7, 0.0, choix);
                aThroat.Set(I.OnFirst, R);
                RecInv(aThroat, X0);
                RecInv(aThroat, X0); // again: Derivatives reads the previous call's points

                Line($"case constthroatpeninv {ii} {choix}");
                BlendFunc_ConstThroatWithPenetrationInv aPen = new(S1, S2, G);
                aPen.Set(1.0, 0.0, choix);
                aPen.Set(I.OnFirst, R);
                RecInv(aPen, X0);

                Line($"case chasyminv {ii} {choix}");
                BlendFunc_ChAsymInv aChAsym = new(S1, S2, G);
                aChAsym.Set(1.0, Math.PI / 3.0, choix);
                aChAsym.Set(I.OnFirst, R);
                RecInv(aChAsym, X0);

                Line($"case ruledinv {ii} {choix}");
                BlendFunc_RuledInv aRuled = new(S1, S2, G);
                aRuled.Set(I.OnFirst, R);
                RecInv(aRuled, X0);
            }
        }

        // A surface and a curve.
        {
            Adaptor3d_Surface S = Surf(0);
            Adaptor3d_Curve C = Guide(4);
            Adaptor3d_Curve G = Guide(0);
            math_Vector X3 = new(1, 3), X2 = new(1, 2);
            X3[1] = 3.0;
            X3[2] = -0.5;
            X3[3] = 3.0;
            X2[1] = 3.0;
            X2[2] = -0.5;
            for (int choix = 1; choix <= 8; choix++)
            {
                Line($"case csconstrad {choix}");
                BlendFunc_CSConstRad aCS = new(S, C, G);
                aCS.Set(1.0, choix);
                RecCS(aCS, 3.0, X3);
                var aTabP = new NCollection_Array1<Point3D>(1, 5);
                var aTabV = new NCollection_Array1<Vector3D>(1, 5);
                Tag("getsection");
                bool isOk = aCS.GetSection(3.0, X3[1], X3[2], X3[3], aTabP, aTabV);
                Int(isOk ? 1 : 0);
                if (isOk)
                {
                    for (int i = 1; i <= 5; i++)
                    {
                        Pnt(aTabP[i]);
                        Vec(aTabV[i]);
                    }
                }

                End();
                Circle3D aCirc = new();
                aCS.Section(3.0, X3[1], X3[2], X3[3], out double aPdeb, out double aPfin, ref aCirc);
                Tag("circle");
                Pnt(aCirc.Location());
                Vec(new Vector3D(aCirc.Axis().Direction()));
                Hex(aCirc.Radius());
                Hex(aPdeb);
                Hex(aPfin);
                End();

                Line($"case cscircular {choix}");
                Law_Linear aLaw = new();
                aLaw.Set(0.0, 0.0, 10.0, 10.0);
                BlendFunc_CSCircular aCirc2 = new(S, C, G, aLaw);
                aCirc2.Set(1.0, choix);
                RecCS(aCirc2, 3.0, X2);
            }
        }
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Spines

    private void RecSpineEval(ChFiDS_Spine Sp, double theFrom, double theTo)
    {
        for (int k = 0; k <= 12; k++)
        {
            double W = theFrom + (theTo - theFrom) * k / 12.0;
            try
            {
                Sp.D2(W, out Point3D P, out Vector3D V1, out Vector3D V2);
                Tag("d2");
                Hex(W);
                Pnt(P);
                Vec(V1);
                Vec(V2);
                Int(Sp.Index(W, true));
                Int(Sp.Index(W, false));
                Int(Sp.CurrentIndexOfElementarySpine());
                End();
                Sp.D1(W, out Point3D Q, out Vector3D T);
                Tag("d1");
                Pnt(Q);
                Vec(T);
                Pnt(Sp.Value(W));
                End();
            }
            catch (FailureException)
            {
                Fail("d2");
            }
        }
    }

    private void RecSpine(ChFiDS_Spine Sp)
    {
        Sp.Load();
        int n = Sp.NbEdges();
        Tag("spine");
        Int(n);
        Hex(Sp.FirstParameter());
        Hex(Sp.LastParameter());
        Int(Sp.IsClosed() ? 1 : 0);
        Int(Sp.IsPeriodic() ? 1 : 0);
        Hex(Sp.GetTolesp());
        Pnt(BRep_Tool.Pnt(Sp.FirstVertex()));
        Pnt(BRep_Tool.Pnt(Sp.LastVertex()));
        End();
        for (int i = 1; i <= n; i++)
        {
            TopoDS_Edge E = Sp.Edges(i);
            Tag("edge");
            Int(i);
            Int((int)E.Orientation());
            Hex(Sp.FirstParameter(i));
            Hex(Sp.LastParameter(i));
            Hex(Sp.Length(i));
            Int(Sp.Index(E));
            TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
            Hex(Sp.Absc(V1));
            Hex(Sp.Absc(V2));
            End();
            BRepAdaptor_Curve aC = Sp.CurrentElementarySpine(i);
            double U = 0.3 * aC.FirstParameter() + 0.7 * aC.LastParameter();
            double A = Sp.Absc(U);
            Sp.Parameter(A, out double U1, true);
            Sp.Parameter(i, A, out double U2, false);
            Tag("absc");
            Hex(U);
            Hex(A);
            Hex(U1);
            Hex(U2);
            Int((int)Sp.GetType());
            End();
            if (Sp.GetType() == global::Kerf.Geometry.CurveKind.Line)
            {
                Line3D L = Sp.Line();
                Tag("line");
                Pnt(L.Location());
                Vec(new Vector3D(L.Direction()));
                End();
            }
            else if (Sp.GetType() == Circle)
            {
                Circle3D C = Sp.Circle();
                Tag("circ");
                Pnt(C.Location());
                Vec(new Vector3D(C.Axis().Direction()));
                Vec(new Vector3D(C.XAxis().Direction()));
                Hex(C.Radius());
                End();
            }
        }

        RecSpineEval(Sp, Sp.FirstParameter(), Sp.LastParameter());
    }

    internal static TopoDS_Face BoxFace()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10.0, 20.0, 30.0).Shape();
        TopExp_Explorer anExpF = new(aBox, TopAbs_FACE);
        return TopoDS.Face(anExpF.Current());
    }

    internal static TopoDS_Face CylinderTopFace()
    {
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(5.0, 10.0).Shape();
        TopExp_Explorer anExpF2 = new(aCyl, TopAbs_FACE);
        anExpF2.Next();
        return TopoDS.Face(anExpF2.Current());
    }

    internal static TopoDS_Face BSplineSolidFace(out Geom_BSplineCurve theCurve)
    {
        Geom_BSplineCurve aBS = GeomConvert.CurveToBSplineCurve(
            new Geom_Circle(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 4.0), QuasiAngular);
        aBS.SetPole(2, new Point3D(aBS.Pole(2).X() * 1.3, aBS.Pole(2).Y() * 0.8, 0.0));
        double f = aBS.FirstParameter(), l = aBS.LastParameter(), m = 0.4 * f + 0.6 * l;
        BRepBuilderAPI_MakeWire aW = new();
        aW.Add(new BRepBuilderAPI_MakeEdge(aBS, f, m).Edge());
        aW.Add(new BRepBuilderAPI_MakeEdge(aBS, m, l).Edge());
        TopoDS_Face aF = new BRepBuilderAPI_MakeFace(aW.Wire(), true).Face();
        TopoDS_Shape aPrism = new BRepPrimAPI_MakePrism(aF, new Vector3D(0, 0, 5)).Shape();
        TopExp_Explorer anExpF3 = new(aPrism, TopAbs_FACE);
        for (; anExpF3.More(); anExpF3.Next())
        {
            if (BRep_Tool.Surface(TopoDS.Face(anExpF3.Current())) is Geom_Plane)
            {
                break;
            }
        }

        theCurve = aBS;
        return TopoDS.Face(anExpF3.Current());
    }

    internal void RecSpines()
    {
        TopoDS_Face aFace = BoxFace();
        {
            Line("case spine-box");
            ChFiDS_Spine Sp = new(1.0e-7);
            for (TopExp_Explorer anExp = new(aFace, TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                Sp.SetEdges(TopoDS.Edge(anExp.Current()));
            }

            RecSpine(Sp);
            Sp.SetFirstStatus(ChFiDS_Closed);
            Sp.SetLastStatus(ChFiDS_Closed);
            Tag("periodic");
            Int(Sp.IsPeriodic() ? 1 : 0);
            Hex(Sp.Period());
            End();
            RecSpineEval(Sp, -15.0, 75.0);
        }

        {
            Line("case spine-box-ext");
            ChFiDS_Spine Sp = new();
            TopExp_Explorer anExp = new(aFace, TopAbs_EDGE);
            TopoDS_Edge E1 = TopoDS.Edge(anExp.Current());
            anExp.Next();
            TopoDS_Edge E2 = TopoDS.Edge(anExp.Current());
            Sp.SetEdges(E2);
            Sp.PutInFirst(E1);
            RecSpine(Sp);
            double L = Sp.LastParameter();
            Sp.SetFirstParameter(-3.0);
            Sp.SetLastParameter(L + 4.0);
            Sp.SetFirstTgt(0.0);
            Sp.SetLastTgt(L);
            Tag("ext");
            Hex(Sp.FirstParameter());
            Hex(Sp.LastParameter());
            Int(Sp.HasFirstTgt() ? 1 : 0);
            Int(Sp.HasLastTgt() ? 1 : 0);
            End();
            RecSpineEval(Sp, -3.0, L + 4.0);
            Sp.SetReference(1);
            RecSpineEval(Sp, 9.0, 11.0);
            Sp.SetReference(15.0);
            RecSpineEval(Sp, 9.0, 11.0);
            Sp.UnsetReference();
            Sp.Reset(true);
            Tag("reset");
            Hex(Sp.FirstParameter());
            Hex(Sp.LastParameter());
            End();
        }

        {
            Line("case spine-cylinder");
            ChFiDS_Spine Sp = new(1.0e-7);
            for (TopExp_Explorer anExp = new(CylinderTopFace(), TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                Sp.SetEdges(TopoDS.Edge(anExp.Current()));
            }

            RecSpine(Sp);
            Sp.SetFirstStatus(ChFiDS_Closed);
            Tag("periodic");
            Int(Sp.IsPeriodic() ? 1 : 0);
            Hex(Sp.Period());
            End();
            RecSpineEval(Sp, -10.0, 40.0);
        }

        {
            Line("case spine-bspline");
            TopoDS_Face aBSFace = BSplineSolidFace(out Geom_BSplineCurve aBS);
            ChFiDS_FilSpine Sp = new(1.0e-7);
            for (TopExp_Explorer anExp = new(aBSFace, TopAbs_EDGE); anExp.More(); anExp.Next())
            {
                Sp.SetEdges(TopoDS.Edge(anExp.Current()));
            }

            RecSpine(Sp);
            Sp.SetFirstStatus(ChFiDS_Closed);
            Tag("periodic");
            Int(Sp.IsPeriodic() ? 1 : 0);
            Hex(Sp.Period());
            End();
            RecSpineEval(Sp, -5.0, 35.0);

            // The radii and laws.
            Sp.SetFirstStatus(ChFiDS_OnSame);
            Sp.SetRadius(2.0);
            Tag("radius");
            Int(Sp.IsConstant() ? 1 : 0);
            Hex(Sp.Radius());
            End();
            Sp.SetRadius(3.0, Sp.Edges(2));
            Tag("radius2");
            Int(Sp.IsConstant() ? 1 : 0);
            Int(Sp.IsConstant(1) ? 1 : 0);
            Int(Sp.IsConstant(2) ? 1 : 0);
            Hex(Sp.Radius(2));
            Hex(Sp.MaxRadFromSeqAndLaws());
            End();
            ChFiDS_ElSpine anEls = new();
            anEls.FirstParameter(Sp.FirstParameter());
            anEls.LastParameter(Sp.LastParameter());
            anEls.SetCurve(aBS);
            Sp.AppendElSpine(anEls);
            Law_Composite aLaw = Sp.Law(anEls)!;
            Tag("law");
            for (int k = 0; k <= 10; k++)
            {
                double W = Sp.FirstParameter() + (Sp.LastParameter() - Sp.FirstParameter()) * k / 10.0;
                Hex(aLaw.Value(W));
            }

            Hex(Sp.MaxRadFromSeqAndLaws());
            End();
        }

        {
            Line("case elspine");
            Geom_BSplineCurve aBS = GeomConvert.CurveToBSplineCurve(
                new Geom_Circle(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 4.0), QuasiAngular);
            aBS.SetPeriodic();
            ChFiDS_ElSpine anEls = new();
            anEls.SetCurve(aBS);
            anEls.FirstParameter(aBS.FirstParameter());
            anEls.LastParameter(aBS.LastParameter());
            anEls.SetPeriodic(true);
            anEls.SetFirstPointAndTgt(new Point3D(1, 2, 3), new Vector3D(0, 1, 0));
            anEls.AddVertexWithTangent(new Axis3D(new Point3D(4, 0, 0), new Direction3D(0, 1, 0)));
            Tag("els");
            Hex(anEls.FirstParameter());
            Hex(anEls.LastParameter());
            Hex(anEls.Period());
            Int((int)anEls.Continuity());
            Int(anEls.NbIntervals(C2));
            Int((int)anEls.GetType());
            Int(anEls.NbVertices());
            Hex(anEls.Resolution(1.0e-3));
            End();
            Adaptor3d_Curve aCopy = anEls.ShallowCopy();
            for (int k = 0; k <= 8; k++)
            {
                double U = 6.0 * k / 8.0;
                aCopy.D1(U, out Point3D Q, out Vector3D W1);
                Tag("elscopy");
                Pnt(Q);
                Vec(W1);
                End();
            }

            anEls.SetOrigin(1.0);
            for (int k = 0; k <= 8; k++)
            {
                double U = 6.0 * k / 8.0;
                anEls.D3(U, out Point3D P, out Vector3D V1, out Vector3D V2, out Vector3D V3);
                Tag("elsd3");
                Pnt(P);
                Vec(V1);
                Vec(V2);
                Vec(V3);
                End();
            }
        }
    }
}
