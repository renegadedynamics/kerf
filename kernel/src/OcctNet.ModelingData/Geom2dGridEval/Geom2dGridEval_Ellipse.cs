// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Ellipse.hxx
// Port of: src/ModelingData/TKG2d/Geom2dGridEval/Geom2dGridEval_Ellipse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Efficient batch evaluator for 2D ellipse grid points:
/// P(u) = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir.
/// </summary>
public sealed class Geom2dGridEval_Ellipse : Geom2dGridEval.ICurveEvaluator
{
    private readonly Geom2d_Ellipse? myGeom;

    /// <summary>Constructor with the ellipse to evaluate (null gives empty results).</summary>
    public Geom2dGridEval_Ellipse(Geom2d_Ellipse? theEllipse)
    {
        myGeom = theEllipse;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom2d_Ellipse? Geometry() => myGeom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<gp_Pnt2d> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<gp_Pnt2d>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<gp_Pnt2d> aResult = new(1, aNb);

        gp_Elips2d anElips = myGeom.Elips2d();
        gp_Pnt2d aCenter = anElips.Location();
        gp_Dir2d aXDir = anElips.XAxis().Direction();
        gp_Dir2d aYDir = anElips.YAxis().Direction();
        double aMajR = anElips.MajorRadius();
        double aMinR = anElips.MinorRadius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            aResult.SetValue(i - theParams.Lower() + 1,
                             new gp_Pnt2d(aCX + aMajR * cosU * aXX + aMinR * sinU * aYX,
                                          aCY + aMajR * cosU * aXY + aMinR * sinU * aYY));
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

        gp_Elips2d anElips = myGeom.Elips2d();
        gp_Pnt2d aCenter = anElips.Location();
        gp_Dir2d aXDir = anElips.XAxis().Direction();
        gp_Dir2d aYDir = anElips.YAxis().Direction();
        double aMajR = anElips.MajorRadius();
        double aMinR = anElips.MinorRadius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            // D1 = -MajorR * sin(u) * XDir + MinorR * cos(u) * YDir
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD1
            {
                Point = new gp_Pnt2d(aCX + aMajR * cosU * aXX + aMinR * sinU * aYX,
                                     aCY + aMajR * cosU * aXY + aMinR * sinU * aYY),
                D1 = new gp_Vec2d(-aMajR * sinU * aXX + aMinR * cosU * aYX, -aMajR * sinU * aXY + aMinR * cosU * aYY),
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

        gp_Elips2d anElips = myGeom.Elips2d();
        gp_Pnt2d aCenter = anElips.Location();
        gp_Dir2d aXDir = anElips.XAxis().Direction();
        gp_Dir2d aYDir = anElips.YAxis().Direction();
        double aMajR = anElips.MajorRadius();
        double aMinR = anElips.MinorRadius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            // D1 = -MajorR * sin(u) * XDir + MinorR * cos(u) * YDir
            // D2 = -MajorR * cos(u) * XDir - MinorR * sin(u) * YDir = -(P - Center)
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD2
            {
                Point = new gp_Pnt2d(aCX + aMajR * cosU * aXX + aMinR * sinU * aYX,
                                     aCY + aMajR * cosU * aXY + aMinR * sinU * aYY),
                D1 = new gp_Vec2d(-aMajR * sinU * aXX + aMinR * cosU * aYX, -aMajR * sinU * aXY + aMinR * cosU * aYY),
                D2 = new gp_Vec2d(-aMajR * cosU * aXX - aMinR * sinU * aYX, -aMajR * cosU * aXY - aMinR * sinU * aYY),
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

        gp_Elips2d anElips = myGeom.Elips2d();
        gp_Pnt2d aCenter = anElips.Location();
        gp_Dir2d aXDir = anElips.XAxis().Direction();
        gp_Dir2d aYDir = anElips.YAxis().Direction();
        double aMajR = anElips.MajorRadius();
        double aMinR = anElips.MinorRadius();

        double aCX = aCenter.X();
        double aCY = aCenter.Y();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            // P = Center + MajorR * cos(u) * XDir + MinorR * sin(u) * YDir
            // D1 = -MajorR * sin(u) * XDir + MinorR * cos(u) * YDir
            // D2 = -MajorR * cos(u) * XDir - MinorR * sin(u) * YDir
            // D3 =  MajorR * sin(u) * XDir - MinorR * cos(u) * YDir = -D1
            aResult[i - theParams.Lower() + 1] = new Geom2d_Curve.ResD3
            {
                Point = new gp_Pnt2d(aCX + aMajR * cosU * aXX + aMinR * sinU * aYX,
                                     aCY + aMajR * cosU * aXY + aMinR * sinU * aYY),
                D1 = new gp_Vec2d(-aMajR * sinU * aXX + aMinR * cosU * aYX, -aMajR * sinU * aXY + aMinR * cosU * aYY),
                D2 = new gp_Vec2d(-aMajR * cosU * aXX - aMinR * sinU * aYX, -aMajR * cosU * aXY - aMinR * sinU * aYY),
                D3 = new gp_Vec2d(aMajR * sinU * aXX - aMinR * cosU * aYX, aMajR * sinU * aXY - aMinR * cosU * aYY),
            };
        }
        return aResult;
    }

    /// <summary>Evaluates the Nth derivative at all grid points; derivatives cycle with period 4.</summary>
    public NCollection_Array1<gp_Vec2d> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (myGeom is null || theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<gp_Vec2d>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<gp_Vec2d> aResult = new(1, aNb);

        gp_Elips2d anElips = myGeom.Elips2d();
        gp_Dir2d aXDir = anElips.XAxis().Direction();
        gp_Dir2d aYDir = anElips.YAxis().Direction();
        double aMajR = anElips.MajorRadius();
        double aMinR = anElips.MinorRadius();

        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();

        // Ellipse derivatives are cyclic with period 4:
        // D1 = -MajR * sin(u) * X + MinR * cos(u) * Y  -> coefficients: (-sin, cos)
        // D2 = -MajR * cos(u) * X - MinR * sin(u) * Y  -> coefficients: (-cos, -sin)
        // D3 =  MajR * sin(u) * X - MinR * cos(u) * Y  -> coefficients: (sin, -cos)
        // D4 =  MajR * cos(u) * X + MinR * sin(u) * Y  -> coefficients: (cos, sin) = D0
        int aPhase = (theN - 1) % 4;

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
                             new gp_Vec2d(aMajR * aCoeffMajR * aXX + aMinR * aCoeffMinR * aYX,
                                          aMajR * aCoeffMajR * aXY + aMinR * aCoeffMinR * aYY));
        }
        return aResult;
    }
}
