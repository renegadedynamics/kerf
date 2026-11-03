// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Hyperbola.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Hyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Efficient batch evaluator for hyperbola grid points: P(u) = C + MajR * cosh(u) * X + MinR * sinh(u) * Y.
/// </summary>
public sealed class GeomGridEval_Hyperbola : GeomGridEval.ICurveEvaluator
{
    private readonly Geom_Hyperbola? myGeom;

    /// <summary>Constructor with the hyperbola to evaluate (null gives empty results).</summary>
    public GeomGridEval_Hyperbola(Geom_Hyperbola? theHyperbola)
    {
        myGeom = theHyperbola;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_Hyperbola? Geometry() => myGeom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<Point3D> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Point3D>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Point3D> aResult = new(1, aNb);

        Hyperbola3D aHypr = myGeom.Hypr();
        Point3D aCenter = aHypr.Location();
        Direction3D aXDir = aHypr.XAxis().Direction();
        Direction3D aYDir = aHypr.YAxis().Direction();
        double aMajR = aHypr.MajorRadius();
        double aMinR = aHypr.MinorRadius();

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
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            // P = Center + MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir
            aResult.SetValue(i - theParams.Lower() + 1,
                             new Point3D(aCX + aMajR * coshU * aXX + aMinR * sinhU * aYX,
                                        aCY + aMajR * coshU * aXY + aMinR * sinhU * aYY,
                                        aCZ + aMajR * coshU * aXZ + aMinR * sinhU * aYZ));
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

        Hyperbola3D aHypr = myGeom.Hypr();
        Point3D aCenter = aHypr.Location();
        Direction3D aXDir = aHypr.XAxis().Direction();
        Direction3D aYDir = aHypr.YAxis().Direction();
        double aMajR = aHypr.MajorRadius();
        double aMinR = aHypr.MinorRadius();

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
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            // P = Center + MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir
            // D1 = MajorR * sinh(u) * XDir + MinorR * cosh(u) * YDir
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD1
            {
                Point = new Point3D(aCX + aMajR * coshU * aXX + aMinR * sinhU * aYX,
                                   aCY + aMajR * coshU * aXY + aMinR * sinhU * aYY,
                                   aCZ + aMajR * coshU * aXZ + aMinR * sinhU * aYZ),
                D1 = new Vector3D(aMajR * sinhU * aXX + aMinR * coshU * aYX,
                                aMajR * sinhU * aXY + aMinR * coshU * aYY,
                                aMajR * sinhU * aXZ + aMinR * coshU * aYZ),
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

        Hyperbola3D aHypr = myGeom.Hypr();
        Point3D aCenter = aHypr.Location();
        Direction3D aXDir = aHypr.XAxis().Direction();
        Direction3D aYDir = aHypr.YAxis().Direction();
        double aMajR = aHypr.MajorRadius();
        double aMinR = aHypr.MinorRadius();

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
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            // P = Center + MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir
            // D1 = MajorR * sinh(u) * XDir + MinorR * cosh(u) * YDir
            // D2 = MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD2
            {
                Point = new Point3D(aCX + aMajR * coshU * aXX + aMinR * sinhU * aYX,
                                   aCY + aMajR * coshU * aXY + aMinR * sinhU * aYY,
                                   aCZ + aMajR * coshU * aXZ + aMinR * sinhU * aYZ),
                D1 = new Vector3D(aMajR * sinhU * aXX + aMinR * coshU * aYX,
                                aMajR * sinhU * aXY + aMinR * coshU * aYY,
                                aMajR * sinhU * aXZ + aMinR * coshU * aYZ),
                D2 = new Vector3D(aMajR * coshU * aXX + aMinR * sinhU * aYX,
                                aMajR * coshU * aXY + aMinR * sinhU * aYY,
                                aMajR * coshU * aXZ + aMinR * sinhU * aYZ),
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

        Hyperbola3D aHypr = myGeom.Hypr();
        Point3D aCenter = aHypr.Location();
        Direction3D aXDir = aHypr.XAxis().Direction();
        Direction3D aYDir = aHypr.YAxis().Direction();
        double aMajR = aHypr.MajorRadius();
        double aMinR = aHypr.MinorRadius();

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
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            // P = Center + MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir
            // D1 = MajorR * sinh(u) * XDir + MinorR * cosh(u) * YDir
            // D2 = MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir
            // D3 = MajorR * sinh(u) * XDir + MinorR * cosh(u) * YDir = D1
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD3
            {
                Point = new Point3D(aCX + aMajR * coshU * aXX + aMinR * sinhU * aYX,
                                   aCY + aMajR * coshU * aXY + aMinR * sinhU * aYY,
                                   aCZ + aMajR * coshU * aXZ + aMinR * sinhU * aYZ),
                D1 = new Vector3D(aMajR * sinhU * aXX + aMinR * coshU * aYX,
                                aMajR * sinhU * aXY + aMinR * coshU * aYY,
                                aMajR * sinhU * aXZ + aMinR * coshU * aYZ),
                D2 = new Vector3D(aMajR * coshU * aXX + aMinR * sinhU * aYX,
                                aMajR * coshU * aXY + aMinR * sinhU * aYY,
                                aMajR * coshU * aXZ + aMinR * sinhU * aYZ),
                D3 = new Vector3D(aMajR * sinhU * aXX + aMinR * coshU * aYX,
                                aMajR * sinhU * aXY + aMinR * coshU * aYY,
                                aMajR * sinhU * aXZ + aMinR * coshU * aYZ),
            };
        }
        return aResult;
    }

    /// <summary>Evaluates the Nth derivative (N &gt;= 1) at all grid points; the derivatives cycle with period 2.</summary>
    public NCollection_Array1<Vector3D> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (myGeom is null || theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<Vector3D>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Vector3D> aResult = new(1, aNb);

        Hyperbola3D aHypr = myGeom.Hypr();
        Direction3D aXDir = aHypr.XAxis().Direction();
        Direction3D aYDir = aHypr.YAxis().Direction();
        double aMajR = aHypr.MajorRadius();
        double aMinR = aHypr.MinorRadius();

        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        // Hyperbola derivatives are cyclic with period 2:
        // D0 = MajR * cosh(u) * X + MinR * sinh(u) * Y  -> coefficients: (cosh, sinh)
        // D1 = MajR * sinh(u) * X + MinR * cosh(u) * Y  -> coefficients: (sinh, cosh)
        // D2 = D0, D3 = D1, etc.
        bool isOdd = (theN % 2) == 1;

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            if (isOdd)
            {
                // Odd derivatives (D1, D3, D5, ...): coefficients (sinh, cosh)
                aResult.SetValue(i - theParams.Lower() + 1,
                                 new Vector3D(aMajR * sinhU * aXX + aMinR * coshU * aYX,
                                            aMajR * sinhU * aXY + aMinR * coshU * aYY,
                                            aMajR * sinhU * aXZ + aMinR * coshU * aYZ));
            }
            else
            {
                // Even derivatives (D2, D4, D6, ...): coefficients (cosh, sinh) = D0
                aResult.SetValue(i - theParams.Lower() + 1,
                                 new Vector3D(aMajR * coshU * aXX + aMinR * sinhU * aYX,
                                            aMajR * coshU * aXY + aMinR * sinhU * aYY,
                                            aMajR * coshU * aXZ + aMinR * sinhU * aYZ));
            }
        }
        return aResult;
    }
}
