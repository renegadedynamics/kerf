// Port of OCCT's src/ModelingData/TKG3d/GTests/GeomGridEval_Ellipse_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class GeomGridEval_EllipseTest
{
    private const double THE_TOLERANCE = 1e-10;

    private static NCollection_Array1<double> CreateUniformParams(double theFirst, double theLast, int theNbPoints)
    {
        NCollection_Array1<double> aParams = new(1, theNbPoints);
        double aStep = (theLast - theFirst) / (theNbPoints - 1);
        for (int i = 1; i <= theNbPoints; ++i)
        {
            aParams.SetValue(i, theFirst + (i - 1) * aStep);
        }
        return aParams;
    }

    // Ellipse in XY plane, Major=3, Minor=2, center at origin
    private static EllipseCurve3D MakeCurve() => new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 3.0, 2.0);

    [Fact]
    public void BasicEvaluation()
    {
        // Ellipse in XY plane, Major=3, Minor=2, center at origin
        EllipseCurve3D anEllipse = MakeCurve();

        EllipseGrid anEval = new(anEllipse);
        Assert.NotNull(anEval.Geometry());

        // Test at 0, PI/2, PI, 3PI/2, 2PI
        NCollection_Array1<double> aParams = new(1, 5);
        aParams.SetValue(1, 0.0);
        aParams.SetValue(2, M_PI / 2);
        aParams.SetValue(3, M_PI);
        aParams.SetValue(4, 3 * M_PI / 2);
        aParams.SetValue(5, 2 * M_PI);

        NCollection_Array1<Point3D> aGrid = anEval.EvaluateGrid(aParams);

        // u=0: (3, 0, 0)
        Assert.Equal(3.0, aGrid.Value(1).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(1).Y(), THE_TOLERANCE);

        // u=PI/2: (0, 2, 0)
        Assert.Equal(0.0, aGrid.Value(2).X(), THE_TOLERANCE);
        Assert.Equal(2.0, aGrid.Value(2).Y(), THE_TOLERANCE);

        // u=PI: (-3, 0, 0)
        Assert.Equal(-3.0, aGrid.Value(3).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(3).Y(), THE_TOLERANCE);

        // u=3PI/2: (0, -2, 0)
        Assert.Equal(0.0, aGrid.Value(4).X(), THE_TOLERANCE);
        Assert.Equal(-2.0, aGrid.Value(4).Y(), THE_TOLERANCE);

        // u=2PI: back to (3, 0, 0)
        Assert.Equal(3.0, aGrid.Value(5).X(), THE_TOLERANCE);
        Assert.Equal(0.0, aGrid.Value(5).Y(), THE_TOLERANCE);
    }

    [Fact]
    public void DerivativeD1()
    {
        EllipseCurve3D aCurve = MakeCurve();
        EllipseGrid anEval = new(aCurve);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<Curve3D.ResD1> aGrid = anEval.EvaluateGridD1(aParams);

        for (int i = 1; i <= 9; ++i)
        {
            aCurve.D1(aParams.Value(i), out Point3D aPnt, out Vector3D aD1);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeD2()
    {
        EllipseCurve3D aCurve = MakeCurve();
        EllipseGrid anEval = new(aCurve);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<Curve3D.ResD2> aGrid = anEval.EvaluateGridD2(aParams);

        for (int i = 1; i <= 9; ++i)
        {
            aCurve.D2(aParams.Value(i), out Point3D aPnt, out Vector3D aD1, out Vector3D aD2);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
        }
    }

    [Fact]
    public void DerivativeD3()
    {
        EllipseCurve3D aCurve = MakeCurve();
        EllipseGrid anEval = new(aCurve);

        NCollection_Array1<double> aParams = CreateUniformParams(0.0, 2 * M_PI, 9);
        NCollection_Array1<Curve3D.ResD3> aGrid = anEval.EvaluateGridD3(aParams);

        for (int i = 1; i <= 9; ++i)
        {
            aCurve.D3(aParams.Value(i), out Point3D aPnt, out Vector3D aD1, out Vector3D aD2, out Vector3D aD3);
            Assert.Equal(0.0, aGrid.Value(i).Point.Distance(aPnt), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D1 - aD1).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D2 - aD2).Magnitude(), THE_TOLERANCE);
            Assert.Equal(0.0, (aGrid.Value(i).D3 - aD3).Magnitude(), THE_TOLERANCE);
        }
    }
}
