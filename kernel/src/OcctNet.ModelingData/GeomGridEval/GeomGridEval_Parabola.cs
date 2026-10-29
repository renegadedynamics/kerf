// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Parabola.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Parabola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Efficient batch evaluator for parabola grid points: P(u) = C + (u^2 / 4F) * X + u * Y.
/// </summary>
public sealed class GeomGridEval_Parabola : GeomGridEval.ICurveEvaluator
{
    private readonly Geom_Parabola? myGeom;

    /// <summary>Constructor with the parabola to evaluate (null gives empty results).</summary>
    public GeomGridEval_Parabola(Geom_Parabola? theParabola)
    {
        myGeom = theParabola;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_Parabola? Geometry() => myGeom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<gp_Pnt> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<gp_Pnt>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<gp_Pnt> aResult = new(1, aNb);

        gp_Parab aParab = myGeom.Parab();
        gp_Pnt aCenter = aParab.Location();
        gp_Dir aXDir = aParab.XAxis().Direction();
        gp_Dir aYDir = aParab.YAxis().Direction();
        double aFocal = aParab.Focal();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aCZ = aCenter.Z();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        double aCoeff = 1.0 / (4.0 * aFocal);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);

            // P = Center + (u^2 / 4F) * XDir + u * YDir
            double u2Term = u * u * aCoeff;
            aResult.SetValue(i - theParams.Lower() + 1,
                             new gp_Pnt(aCX + u2Term * aXX + u * aYX,
                                        aCY + u2Term * aXY + u * aYY,
                                        aCZ + u2Term * aXZ + u * aYZ));
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

        gp_Parab aParab = myGeom.Parab();
        gp_Pnt aCenter = aParab.Location();
        gp_Dir aXDir = aParab.XAxis().Direction();
        gp_Dir aYDir = aParab.YAxis().Direction();
        double aFocal = aParab.Focal();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aCZ = aCenter.Z();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        double aCoeff = 1.0 / (4.0 * aFocal);
        double aCoeff2 = 1.0 / (2.0 * aFocal); // Derivative of u^2/4F is 2u/4F = u/2F

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);

            // P = Center + (u^2 / 4F) * XDir + u * YDir
            // D1 = (u / 2F) * XDir + YDir
            double u2Term = u * u * aCoeff;
            double d1Term = u * aCoeff2;
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD1
            {
                Point = new gp_Pnt(aCX + u2Term * aXX + u * aYX,
                                   aCY + u2Term * aXY + u * aYY,
                                   aCZ + u2Term * aXZ + u * aYZ),
                D1 = new gp_Vec(d1Term * aXX + aYX,
                                d1Term * aXY + aYY,
                                d1Term * aXZ + aYZ),
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

        gp_Parab aParab = myGeom.Parab();
        gp_Pnt aCenter = aParab.Location();
        gp_Dir aXDir = aParab.XAxis().Direction();
        gp_Dir aYDir = aParab.YAxis().Direction();
        double aFocal = aParab.Focal();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aCZ = aCenter.Z();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        double aCoeff = 1.0 / (4.0 * aFocal);
        double aCoeff2 = 1.0 / (2.0 * aFocal);
        double aCoeff3 = aCoeff2; // Derivative of u/2F is 1/2F

        // D2 is constant for parabola: (1/2F) * XDir
        gp_Vec aD2 = new(aCoeff3 * aXX, aCoeff3 * aXY, aCoeff3 * aXZ);

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);

            // P = Center + (u^2 / 4F) * XDir + u * YDir
            // D1 = (u / 2F) * XDir + YDir
            // D2 = (1 / 2F) * XDir
            double u2Term = u * u * aCoeff;
            double d1Term = u * aCoeff2;
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD2
            {
                Point = new gp_Pnt(aCX + u2Term * aXX + u * aYX,
                                   aCY + u2Term * aXY + u * aYY,
                                   aCZ + u2Term * aXZ + u * aYZ),
                D1 = new gp_Vec(d1Term * aXX + aYX,
                                d1Term * aXY + aYY,
                                d1Term * aXZ + aYZ),
                D2 = aD2,
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

        gp_Parab aParab = myGeom.Parab();
        gp_Pnt aCenter = aParab.Location();
        gp_Dir aXDir = aParab.XAxis().Direction();
        gp_Dir aYDir = aParab.YAxis().Direction();
        double aFocal = aParab.Focal();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aCZ = aCenter.Z();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        double aCoeff = 1.0 / (4.0 * aFocal);
        double aCoeff2 = 1.0 / (2.0 * aFocal);
        double aCoeff3 = aCoeff2;

        gp_Vec aD2 = new(aCoeff3 * aXX, aCoeff3 * aXY, aCoeff3 * aXZ);
        gp_Vec aD3 = new(0.0, 0.0, 0.0); // D3 is 0

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);

            double u2Term = u * u * aCoeff;
            double d1Term = u * aCoeff2;
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD3
            {
                Point = new gp_Pnt(aCX + u2Term * aXX + u * aYX,
                                   aCY + u2Term * aXY + u * aYY,
                                   aCZ + u2Term * aXZ + u * aYZ),
                D1 = new gp_Vec(d1Term * aXX + aYX,
                                d1Term * aXY + aYY,
                                d1Term * aXZ + aYZ),
                D2 = aD2,
                D3 = aD3,
            };
        }
        return aResult;
    }

    /// <summary>Evaluates the Nth derivative (N &gt;= 1) at all grid points: D1 depends on u, D2 is constant, DN is 0 above.</summary>
    public NCollection_Array1<gp_Vec> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (myGeom is null || theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<gp_Vec>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<gp_Vec> aResult = new(1, aNb);

        gp_Parab aParab = myGeom.Parab();
        gp_Dir aXDir = aParab.XAxis().Direction();
        gp_Dir aYDir = aParab.YAxis().Direction();
        double aFocal = aParab.Focal();

        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        double aCoeff2 = 1.0 / (2.0 * aFocal);

        if (theN == 1)
        {
            // D1 = (u/2F) * X + Y (depends on u)
            for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
            {
                double u = theParams.Value(i);
                double d1Term = u * aCoeff2;
                aResult.SetValue(i - theParams.Lower() + 1,
                                 new gp_Vec(d1Term * aXX + aYX, d1Term * aXY + aYY, d1Term * aXZ + aYZ));
            }
        }
        else if (theN == 2)
        {
            // D2 = (1/2F) * X (constant)
            gp_Vec aD2 = new(aCoeff2 * aXX, aCoeff2 * aXY, aCoeff2 * aXZ);
            for (int i = 1; i <= aNb; ++i)
            {
                aResult.SetValue(i, aD2);
            }
        }
        else
        {
            // DN = 0 for N >= 3
            gp_Vec aZero = new(0.0, 0.0, 0.0);
            for (int i = 1; i <= aNb; ++i)
            {
                aResult.SetValue(i, aZero);
            }
        }
        return aResult;
    }
}
