// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static OcctNet.ModelingAlgorithms.Tests.LocationLawTestSupport;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>GeomFill_LocFunction, GeomFill_FunctionDraft and GeomFill_FunctionGuide.</summary>
public class GeomFill_FunctionsTests
{
    [Fact]
    public void LocFunction_DN_IsTheVectorThenTheColumns()
    {
        var aLaw = new GeomFill_CurveAndTrihedron(new GeomFill_DraftTrihedron(new Vector3D(0, 0, 1), 0.2));
        aLaw.SetCurve(new GeomAdaptor_Curve(WavyCurve()));
        var aFunc = new GeomFill_LocFunction(aLaw);

        var P = new NCollection_Array1<Point2D>(1, 1);
        var DP = new NCollection_Array1<Vector2D>(1, 1);
        var D2P = new NCollection_Array1<Vector2D>(1, 1);
        Matrix3x3 M = new(), DM = new(), D2M = new();
        Vector3D V = new(), DV = new(), D2V = new();
        aLaw.D2(0.45, ref M, ref V, ref DM, ref DV, ref D2M, ref D2V, P, DP, D2P);

        Span<double> aRes = stackalloc double[12];
        int anIer = -1;
        (Matrix3x3, Vector3D)[] anExpected = [(M, V), (DM, DV), (D2M, D2V)];
        for (int anOrder = 0; anOrder <= 2; anOrder++)
        {
            aFunc.DN(0.45, 0, 1, anOrder, aRes, ref anIer);
            Assert.Equal(0, anIer);
            (Matrix3x3 aM, Vector3D aV) = anExpected[anOrder];
            Assert.Equal(aV.XYZ(), new Xyz(aRes[0], aRes[1], aRes[2]));
            for (int c = 1; c <= 3; c++)
            {
                Assert.Equal(aM.Column(c), new Xyz(aRes[3 * c], aRes[3 * c + 1], aRes[3 * c + 2]));
            }
        }
        // OCCT reads through a null pointer for another order; the port throws
        Assert.Throws<OutOfRangeException>(() =>
        {
            Span<double> aR = stackalloc double[12];
            int anI = 0;
            aFunc.DN(0.45, 0, 1, 3, aR, ref anI);
        });
    }

    [Fact]
    public void LocFunction_DN_ReportsAFailedLaw()
    {
        var aLaw = new GeomFill_CurveAndTrihedron(new GeomFill_DraftTrihedron(new Vector3D(0, 0, 1), 0.2));
        aLaw.SetCurve(new GeomAdaptor_Curve(new Geom_Line(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0, 1));
        var aFunc = new GeomFill_LocFunction(aLaw);
        var aRes = new double[12];
        int anIer = 0;
        aFunc.DN(0.5, 0, 1, 0, aRes, ref anIer);
        Assert.Equal(1, anIer);
    }

    private static void CheckDerivatives(FunctionSetWithDerivatives theF, math_Vector theX, double theTol)
    {
        int n = theF.NbVariables();
        var D = new Matrix(1, n, 1, n);
        var F = new math_Vector(1, n);
        var D2 = new Matrix(1, n, 1, n);
        Assert.True(theF.Derivatives(theX, D));
        Assert.True(theF.Values(theX, F, D2));
        const double h = 1e-6;
        for (int j = 1; j <= n; j++)
        {
            var Xp = new math_Vector(theX);
            var Xm = new math_Vector(theX);
            Xp[j] += h;
            Xm[j] -= h;
            var Fp = new math_Vector(1, n);
            var Fm = new math_Vector(1, n);
            theF.Value(Xp, Fp);
            theF.Value(Xm, Fm);
            for (int i = 1; i <= n; i++)
            {
                Assert.Equal((Fp[i] - Fm[i]) / (2 * h), D[i, j], theTol);
                Assert.Equal(D[i, j], D2[i, j]);
            }
        }
        var F0 = new math_Vector(1, n);
        theF.Value(theX, F0);
        for (int i = 1; i <= n; i++)
        {
            Assert.Equal(F0[i], F[i]);
        }
    }

    [Fact]
    public void FunctionDraft_DerivativesMatchFiniteDifferences()
    {
        var aSurf = new GeomAdaptor_Surface(new Geom_SphericalSurface(new CoordinateSystem(Constants.XOY()), 5));
        var aGen = new GeomAdaptor_Curve(new Geom_Line(new Point3D(1, 2, 0), new Direction3D(1, 1, -1)));
        var E = new GeomFill_FunctionDraft(aSurf, aGen);
        Assert.Equal(3, E.NbVariables());
        Assert.Equal(3, E.NbEquations());
        var X = new math_Vector(1, 3);
        X[1] = 0.7;
        X[2] = 0.4;
        X[3] = -0.3;
        CheckDerivatives(E, X, 1e-6);

        // the second derivatives in (u, v) are the surface's, negated
        var T = new GeomFill_Tensor(3, 3, 3);
        Assert.True(E.Deriv2X(X, T));
        aSurf.D2(0.4, -0.3, out _, out _, out _, out Vector3D D2U, out Vector3D D2V, out Vector3D D2UV);
        for (int i = 1; i <= 3; i++)
        {
            Assert.Equal(0.0, T[i, 1, 1]);
            Assert.Equal(-D2U.Coord(i), T[i, 2, 2]);
            Assert.Equal(-D2UV.Coord(i), T[i, 2, 3]);
            Assert.Equal(-D2UV.Coord(i), T.Value(i, 3, 2));
            Assert.Equal(-D2V.Coord(i), T[i, 3, 3]);
        }

        var DTX = new Matrix(1, 3, 1, 3);
        Assert.True(E.DerivTX(new Vector3D(1, 2, 3), 0.5, DTX));
        Assert.Equal(2 * Math.Sin(0.5), DTX[2, 1]);
        Assert.Equal(0.0, DTX[2, 2]);

        var F = new math_Vector(1, 3);
        Assert.True(E.DerivT(aGen, 0.3, 2.0, new Vector3D(1, 0, 0), 0.5, F));
        Assert.Equal(1 / Math.Sqrt(3) + 2 * Math.Sin(0.5), F[1], 1e-15);
    }

    [Fact]
    public void Tensor_Multiply()
    {
        var T = new GeomFill_Tensor(2, 3, 2);
        T.Init(1.0);
        T[2, 3, 2] = 5.0;
        T.ChangeValue(1, 1, 1) = 2.0;
        var V = new math_Vector(1, 2);
        V[1] = 3;
        V[2] = 4;
        var M = new Matrix(1, 2, 1, 3);
        T.Multiply(V, M);
        Assert.Equal(2 * 3 + 4, M[1, 1]);
        Assert.Equal(3 + 4, M[1, 2]);
        Assert.Equal(3 + 5 * 4, M[2, 3]);
    }

    [Fact]
    public void FunctionGuide_DerivativesMatchFiniteDifferences()
    {
        // a constant section: the radial segment [0, 3] along X, revolved about the path's tangent
        var aSection = new GeomFill_UniformSection(new Geom_TrimmedCurve(new Geom_Line(Constants.OX()), 0, 3));
        var aGuide = new GeomAdaptor_Curve(new Geom_Line(new Point3D(1, 1, 0), new Direction3D(0, 0.3, 1)));
        var E = new GeomFill_FunctionGuide(aSection, aGuide);
        E.SetParam(0.0, new Point3D(0, 0, 1), new Xyz(0, 0, 1), new Xyz(1, 0, 0));
        var X = new math_Vector(1, 3);
        X[1] = 0.8;
        X[2] = 0.6;
        X[3] = 1.2;
        CheckDerivatives(E, X, 1e-6);

        // a root: the guide meets the plane z = 1 at w = 1 / |(0, 0.3, 1)|, at (1, 1.3, 1)
        double w = Math.Sqrt(1.09);
        X[1] = w;
        X[2] = Math.Atan2(1.3, 1);
        X[3] = Math.Sqrt(1 + 1.3 * 1.3);
        var F = new math_Vector(1, 3);
        E.Value(X, F);
        Assert.Equal(0.0, F.Norm(), 1e-12);

        var DFDT = new math_Vector(1, 3);
        Assert.True(E.DerivT(X, new Xyz(0, 0, 1), new Xyz(0, 0, 0), DFDT));
    }
}
