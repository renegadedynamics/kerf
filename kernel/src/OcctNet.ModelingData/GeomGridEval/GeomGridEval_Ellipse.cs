// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Ellipse.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Ellipse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Efficient batch evaluator for ellipse grid points: P(u) = C + MajR * cos(u) * X + MinR * sin(u) * Y.
/// </summary>
public sealed class GeomGridEval_Ellipse : GeomGridEval.ICurveEvaluator
{
    private readonly Geom_Ellipse? myGeom;

    /// <summary>Constructor with the ellipse to evaluate (null gives empty results).</summary>
    public GeomGridEval_Ellipse(Geom_Ellipse? theEllipse)
    {
        myGeom = theEllipse;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_Ellipse? Geometry() => myGeom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point3D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point3D>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Point3D> aResult = new(1, aNb);

        Ellipse3D anElips = myGeom.Elips();
        Point3D aCenter = anElips.Location();
        Direction3D aXDir = anElips.XAxis().Direction();
        Direction3D aYDir = anElips.YAxis().Direction();
        double aMajR = anElips.MajorRadius();
        double aMinR = anElips.MinorRadius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aCZ = aCenter.Z();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            aResult.SetValue(i - theParams.Lower() + 1,
                             new Point3D(aCX + aMajR * cosU * aXX + aMinR * sinU * aYX,
                                        aCY + aMajR * cosU * aXY + aMinR * sinU * aYY,
                                        aCZ + aMajR * cosU * aXZ + aMinR * sinU * aYZ));
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Geom_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD1>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom_Curve.ResD1> aResult = new(1, aNb);

        Ellipse3D anElips = myGeom.Elips();
        Point3D aCenter = anElips.Location();
        Direction3D aXDir = anElips.XAxis().Direction();
        Direction3D aYDir = anElips.YAxis().Direction();
        double aMajR = anElips.MajorRadius();
        double aMinR = anElips.MinorRadius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aCZ = aCenter.Z();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            // D1 = -MajorR * sin(u) * XDir + MinorR * cos(u) * YDir
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD1
            {
                Point = new Point3D(aCX + aMajR * cosU * aXX + aMinR * sinU * aYX,
                                   aCY + aMajR * cosU * aXY + aMinR * sinU * aYY,
                                   aCZ + aMajR * cosU * aXZ + aMinR * sinU * aYZ),
                D1 = new Vector3D(-aMajR * sinU * aXX + aMinR * cosU * aYX,
                                -aMajR * sinU * aXY + aMinR * cosU * aYY,
                                -aMajR * sinU * aXZ + aMinR * cosU * aYZ),
            };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Geom_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD2>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom_Curve.ResD2> aResult = new(1, aNb);

        Ellipse3D anElips = myGeom.Elips();
        Point3D aCenter = anElips.Location();
        Direction3D aXDir = anElips.XAxis().Direction();
        Direction3D aYDir = anElips.YAxis().Direction();
        double aMajR = anElips.MajorRadius();
        double aMinR = anElips.MinorRadius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aCZ = aCenter.Z();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            // D1 = -MajorR * sin(u) * XDir + MinorR * cos(u) * YDir
            // D2 = -MajorR * cos(u) * XDir - MinorR * sin(u) * YDir
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD2
            {
                Point = new Point3D(aCX + aMajR * cosU * aXX + aMinR * sinU * aYX,
                                   aCY + aMajR * cosU * aXY + aMinR * sinU * aYY,
                                   aCZ + aMajR * cosU * aXZ + aMinR * sinU * aYZ),
                D1 = new Vector3D(-aMajR * sinU * aXX + aMinR * cosU * aYX,
                                -aMajR * sinU * aXY + aMinR * cosU * aYY,
                                -aMajR * sinU * aXZ + aMinR * cosU * aYZ),
                D2 = new Vector3D(-aMajR * cosU * aXX - aMinR * sinU * aYX,
                                -aMajR * cosU * aXY - aMinR * sinU * aYY,
                                -aMajR * cosU * aXZ - aMinR * sinU * aYZ),
            };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third.</summary>
    public NCollection_Array1<Geom_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom_Curve.ResD3>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom_Curve.ResD3> aResult = new(1, aNb);

        Ellipse3D anElips = myGeom.Elips();
        Point3D aCenter = anElips.Location();
        Direction3D aXDir = anElips.XAxis().Direction();
        Direction3D aYDir = anElips.YAxis().Direction();
        double aMajR = anElips.MajorRadius();
        double aMinR = anElips.MinorRadius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aCZ = aCenter.Z();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            // D1 = -MajorR * sin(u) * XDir + MinorR * cos(u) * YDir
            // D2 = -MajorR * cos(u) * XDir - MinorR * sin(u) * YDir
            // D3 =  MajorR * sin(u) * XDir - MinorR * cos(u) * YDir = -D1
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD3
            {
                Point = new Point3D(aCX + aMajR * cosU * aXX + aMinR * sinU * aYX,
                                   aCY + aMajR * cosU * aXY + aMinR * sinU * aYY,
                                   aCZ + aMajR * cosU * aXZ + aMinR * sinU * aYZ),
                D1 = new Vector3D(-aMajR * sinU * aXX + aMinR * cosU * aYX,
                                -aMajR * sinU * aXY + aMinR * cosU * aYY,
                                -aMajR * sinU * aXZ + aMinR * cosU * aYZ),
                D2 = new Vector3D(-aMajR * cosU * aXX - aMinR * sinU * aYX,
                                -aMajR * cosU * aXY - aMinR * sinU * aYY,
                                -aMajR * cosU * aXZ - aMinR * sinU * aYZ),
                D3 = new Vector3D(aMajR * sinU * aXX - aMinR * cosU * aYX,
                                aMajR * sinU * aXY - aMinR * cosU * aYY,
                                aMajR * sinU * aXZ - aMinR * cosU * aYZ),
            };
        }
        return aResult;
    }

    /// <summary>Evaluates the Nth derivative (N &gt;= 1) at all grid points; the derivatives cycle with period 4.</summary>
    public NCollection_Array1<Vector3D> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (myGeom is null || theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<Vector3D>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Vector3D> aResult = new(1, aNb);

        Ellipse3D anElips = myGeom.Elips();
        Direction3D aXDir = anElips.XAxis().Direction();
        Direction3D aYDir = anElips.YAxis().Direction();
        double aMajR = anElips.MajorRadius();
        double aMinR = anElips.MinorRadius();

        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        // Ellipse derivatives are cyclic with period 4:
        // D1 = -MajR * sin(u) * X + MinR * cos(u) * Y  -> coefficients: (-sin, cos)
        // D2 = -MajR * cos(u) * X - MinR * sin(u) * Y  -> coefficients: (-cos, -sin)
        // D3 =  MajR * sin(u) * X - MinR * cos(u) * Y  -> coefficients: (sin, -cos)
        // D4 =  MajR * cos(u) * X + MinR * sin(u) * Y  -> coefficients: (cos, sin) = D0
        int aPhase = (theN - 1) % 4; // 0->D1, 1->D2, 2->D3, 3->D4

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            double aCoeffMajR, aCoeffMinR;
            switch (aPhase)
            {
                case 0: // D1, D5, D9, ...
                    aCoeffMajR = -sinU;
                    aCoeffMinR = cosU;
                    break;
                case 1: // D2, D6, D10, ...
                    aCoeffMajR = -cosU;
                    aCoeffMinR = -sinU;
                    break;
                case 2: // D3, D7, D11, ...
                    aCoeffMajR = sinU;
                    aCoeffMinR = -cosU;
                    break;
                default: // D4, D8, D12, ... (case 3)
                    aCoeffMajR = cosU;
                    aCoeffMinR = sinU;
                    break;
            }

            aResult.SetValue(i - theParams.Lower() + 1,
                             new Vector3D(aMajR * aCoeffMajR * aXX + aMinR * aCoeffMinR * aYX,
                                        aMajR * aCoeffMajR * aXY + aMinR * aCoeffMinR * aYY,
                                        aMajR * aCoeffMajR * aXZ + aMinR * aCoeffMinR * aYZ));
        }
        return aResult;
    }
}
