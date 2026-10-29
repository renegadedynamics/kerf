// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Hyperbola.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Hyperbola.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Efficient batch evaluator for 2D hyperbola grid points:
/// P(u) = Center + MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir.
/// </summary>
public sealed class Geom2dGridEval_Hyperbola : Geom2dGridEval.ICurveEvaluator
{
    private readonly Geom2d_Hyperbola? myGeom;

    /// <summary>Constructor with the hyperbola to evaluate (null gives empty results).</summary>
    public Geom2dGridEval_Hyperbola(Geom2d_Hyperbola? theHyperbola)
    {
        myGeom = theHyperbola;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom2d_Hyperbola? Geometry() => myGeom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<gp_Pnt2d> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<gp_Pnt2d>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<gp_Pnt2d> aResult = new(1, aNb);

        gp_Hypr2d aHypr = myGeom.Hypr2d();
        gp_Pnt2d aCenter = aHypr.Location();
        gp_Dir2d aXDir = aHypr.XAxis().Direction();
        gp_Dir2d aYDir = aHypr.YAxis().Direction();
        double aMajR = aHypr.MajorRadius();
        double aMinR = aHypr.MinorRadius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            aResult.SetValue(i - theParams.Lower() + 1,
                             new gp_Pnt2d(aCX + aMajR * coshU * aXX + aMinR * sinhU * aYX,
                                          aCY + aMajR * coshU * aXY + aMinR * sinhU * aYY));
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first derivative.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD1> EvaluateGridD1(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD1>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD1> aResult = new(1, aNb);

        gp_Hypr2d aHypr = myGeom.Hypr2d();
        gp_Pnt2d aCenter = aHypr.Location();
        gp_Dir2d aXDir = aHypr.XAxis().Direction();
        gp_Dir2d aYDir = aHypr.YAxis().Direction();
        double aMajR = aHypr.MajorRadius();
        double aMinR = aHypr.MinorRadius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            // P = Center + MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir
            // D1 = MajorR * sinh(u) * XDir + MinorR * cosh(u) * YDir
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD1
            {
                Point = new gp_Pnt2d(aCX + aMajR * coshU * aXX + aMinR * sinhU * aYX,
                                     aCY + aMajR * coshU * aXY + aMinR * sinhU * aYY),
                D1 = new gp_Vec2d(aMajR * sinhU * aXX + aMinR * coshU * aYX,
                                  aMajR * sinhU * aXY + aMinR * coshU * aYY),
            };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with the first and second derivatives.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD2> EvaluateGridD2(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD2>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD2> aResult = new(1, aNb);

        gp_Hypr2d aHypr = myGeom.Hypr2d();
        gp_Pnt2d aCenter = aHypr.Location();
        gp_Dir2d aXDir = aHypr.XAxis().Direction();
        gp_Dir2d aYDir = aHypr.YAxis().Direction();
        double aMajR = aHypr.MajorRadius();
        double aMinR = aHypr.MinorRadius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            // D2 = MajorR * cosh(u) * XDir + MinorR * sinh(u) * YDir = (P - Center)
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD2
            {
                Point = new gp_Pnt2d(aCX + aMajR * coshU * aXX + aMinR * sinhU * aYX,
                                     aCY + aMajR * coshU * aXY + aMinR * sinhU * aYY),
                D1 = new gp_Vec2d(aMajR * sinhU * aXX + aMinR * coshU * aYX,
                                  aMajR * sinhU * aXY + aMinR * coshU * aYY),
                D2 = new gp_Vec2d(aMajR * coshU * aXX + aMinR * sinhU * aYX,
                                  aMajR * coshU * aXY + aMinR * sinhU * aYY),
            };
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with derivatives up to the third.</summary>
    public NCollection_Array1<Geom2d_Curve.ResD3> EvaluateGridD3(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<Geom2d_Curve.ResD3>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<Geom2d_Curve.ResD3> aResult = new(1, aNb);

        gp_Hypr2d aHypr = myGeom.Hypr2d();
        gp_Pnt2d aCenter = aHypr.Location();
        gp_Dir2d aXDir = aHypr.XAxis().Direction();
        gp_Dir2d aYDir = aHypr.YAxis().Direction();
        double aMajR = aHypr.MajorRadius();
        double aMinR = aHypr.MinorRadius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double coshU = Math.Cosh(u);
            double sinhU = Math.Sinh(u);

            // D3 = MajorR * sinh(u) * XDir + MinorR * cosh(u) * YDir = D1
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD3
            {
                Point = new gp_Pnt2d(aCX + aMajR * coshU * aXX + aMinR * sinhU * aYX,
                                     aCY + aMajR * coshU * aXY + aMinR * sinhU * aYY),
                D1 = new gp_Vec2d(aMajR * sinhU * aXX + aMinR * coshU * aYX,
                                  aMajR * sinhU * aXY + aMinR * coshU * aYY),
                D2 = new gp_Vec2d(aMajR * coshU * aXX + aMinR * sinhU * aYX,
                                  aMajR * coshU * aXY + aMinR * sinhU * aYY),
                D3 = new gp_Vec2d(aMajR * sinhU * aXX + aMinR * coshU * aYX,
                                  aMajR * sinhU * aXY + aMinR * coshU * aYY),
            };
        }
        return aResult;
    }

    /// <summary>Evaluates the Nth derivative at all grid points; derivatives cycle with period 2.</summary>
    public NCollection_Array1<gp_Vec2d> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (myGeom is null || theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<gp_Vec2d>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<gp_Vec2d> aResult = new(1, aNb);

        gp_Hypr2d aHypr = myGeom.Hypr2d();
        gp_Dir2d aXDir = aHypr.XAxis().Direction();
        gp_Dir2d aYDir = aHypr.YAxis().Direction();
        double aMajR = aHypr.MajorRadius();
        double aMinR = aHypr.MinorRadius();

        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

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
                aResult.SetValue(i - theParams.Lower() + 1,
                                 new gp_Vec2d(aMajR * sinhU * aXX + aMinR * coshU * aYX,
                                              aMajR * sinhU * aXY + aMinR * coshU * aYY));
            }
            else
            {
                aResult.SetValue(i - theParams.Lower() + 1,
                                 new gp_Vec2d(aMajR * coshU * aXX + aMinR * sinhU * aYX,
                                              aMajR * coshU * aXY + aMinR * sinhU * aYY));
            }
        }
        return aResult;
    }
}
