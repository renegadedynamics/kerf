// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>GeomConvert_FuncSphereLSDist, FuncCylinderLSDist and FuncConeLSDist (OCCT has no GTests for them).</summary>
public class GeomConvert_FuncLSDistTests
{
    private static NCollection_HArray1<global::Kerf.Engine.Geometry.Xyz> Points(int theNb, Func<int, global::Kerf.Engine.Geometry.Xyz> thePoint)
    {
        var aPoints = new NCollection_HArray1<global::Kerf.Engine.Geometry.Xyz>(1, theNb);
        for (int i = 1; i <= theNb; i++)
        {
            aPoints[i] = thePoint(i);
        }

        return aPoints;
    }

    private static math_Vector Vector(params double[] theValues)
    {
        var aV = new math_Vector(1, theValues.Length);
        for (int i = 0; i < theValues.Length; i++)
        {
            aV[i + 1] = theValues[i];
        }

        return aV;
    }

    /// <summary>Central finite differences of F at X.</summary>
    private static math_Vector Numeric(MultipleVarFunction theF, math_Vector theX)
    {
        var aG = new math_Vector(1, theX.Length());
        const double h = 1e-6;
        for (int i = 1; i <= theX.Length(); i++)
        {
            var aXp = new math_Vector(theX);
            var aXm = new math_Vector(theX);
            aXp[i] += h;
            aXm[i] -= h;
            theF.Value(aXp, out double aFp);
            theF.Value(aXm, out double aFm);
            aG[i] = (aFp - aFm) / (2 * h);
        }

        return aG;
    }

    private static void AssertGradient(math_Vector theExpected, math_Vector theActual)
    {
        for (int i = 1; i <= theExpected.Length(); i++)
        {
            Assert.True(Math.Abs(theExpected[i] - theActual[i]) <= 1e-5 * Math.Max(1.0, Math.Abs(theExpected[i])),
                        $"G({i}): {theExpected[i]} vs {theActual[i]}");
        }
    }

    // Points scattered around a sphere of centre (1, 2, 3), radius 2
    private static readonly NCollection_HArray1<global::Kerf.Engine.Geometry.Xyz> s_SpherePoints = Points(30, i =>
    {
        double u = 0.7 * i, v = -1.2 + 0.08 * i, r = 2.0 + 0.01 * Math.Sin(3.0 * i);
        return new global::Kerf.Engine.Geometry.Xyz(1.0 + r * Math.Cos(v) * Math.Cos(u), 2.0 + r * Math.Cos(v) * Math.Sin(u), 3.0 + r * Math.Sin(v));
    });

    [Fact]
    public void Sphere_ValueGradientValues()
    {
        var aF = new FuncSphereLSDist(s_SpherePoints);
        Assert.Equal(4, aF.VariableCount);

        // Near zero at the sphere
        var anExact = new FuncSphereLSDist(Points(10, i => new global::Kerf.Engine.Geometry.Xyz(2.0 * Math.Cos(i), 2.0 * Math.Sin(i), 0.0)));
        Assert.True(anExact.Value(Vector(0, 0, 0, 2), out double aZero));
        Assert.Equal(0.0, aZero, 15);

        math_Vector X = Vector(1.1, 1.9, 3.2, 1.8);
        var aG = new math_Vector(1, 4);
        Assert.True(aF.Gradient(X, aG));
        AssertGradient(Numeric(aF, X), aG);

        var aG2 = new math_Vector(1, 4);
        Assert.True(aF.Values(X, out double aF2, aG2));
        aF.Value(X, out double aF1);
        Assert.Equal(aF1, aF2);
        AssertGradient(aG, aG2);
    }

    [Fact]
    public void Cylinder_ValueValues_AndGradientReproducesOcctError()
    {
        var aDir = new Direction3D(0.2, -0.1, 1.0);
        CoordinateSystem anAx = new(new Point3D(1, 2, 3), aDir);
        NCollection_HArray1<global::Kerf.Engine.Geometry.Xyz> aPoints = Points(25, i =>
        {
            double u = 0.5 * i, v = -3.0 + 0.25 * i, r = 1.5 + 0.01 * Math.Cos(2.0 * i);
            return global::Kerf.Engine.Geometry.ElSLib.CylinderValue(u, v, anAx, r).XYZ();
        });
        var aF = new FuncCylinderLSDist(aPoints, aDir);
        Assert.Equal(4, aF.VariableCount);

        math_Vector X = Vector(1.05, 2.1, 2.9, 1.4);
        var aG = new math_Vector(1, 4);
        Assert.True(aF.Values(X, out double aFv, aG));
        aF.Value(X, out double aF1);
        Assert.Equal(aF1, aFv);
        AssertGradient(Numeric(aF, X), aG);

        // OCCT scales G(6) where it means G(4): out of range (occt-issues.md)
        Assert.ThrowsAny<Exception>(() => aF.Gradient(X, new math_Vector(1, 4)));

        // Exact cylinder: zero
        var anExact = new FuncCylinderLSDist();
        anExact.SetPoints(Points(8, i => global::Kerf.Engine.Geometry.ElSLib.CylinderValue(i, i - 4.0, anAx, 1.5).XYZ()));
        anExact.SetDir(aDir);
        anExact.Value(Vector(1, 2, 3, 1.5), out double aZero);
        Assert.Equal(0.0, aZero, 15);
    }

    [Fact]
    public void Cone_Value()
    {
        var aDir = new Direction3D(0.0, 0.3, 1.0);
        CoordinateSystem anAx = new(new Point3D(-1, 0, 2), aDir);
        double anAngle = 0.4, aR = 1.2;
        NCollection_HArray1<global::Kerf.Engine.Geometry.Xyz> aPoints = Points(12, i => global::Kerf.Engine.Geometry.ElSLib.ConeValue(0.5 * i, -1.0 + 0.3 * i, anAx, aR, anAngle).XYZ());
        var aF = new FuncConeLSDist(aPoints, aDir);
        Assert.Equal(5, aF.VariableCount);
        Assert.True(aF.Value(Vector(-1, 0, 2, anAngle, aR), out double aZero));
        Assert.Equal(0.0, aZero, 15);

        // Moving the cone away increases the criterion
        aF.Value(Vector(-1, 0, 2, anAngle, aR + 0.1), out double aFar);
        Assert.True(aFar > 1e-3);

        var aLater = new FuncConeLSDist();
        aLater.SetPoints(aPoints);
        aLater.SetDir(aDir);
        aLater.Value(Vector(-1, 0, 2, anAngle, aR + 0.1), out double aSame);
        Assert.Equal(aFar, aSame);
    }
}
