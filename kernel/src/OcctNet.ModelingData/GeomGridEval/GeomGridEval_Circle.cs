// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Circle.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Circle.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Efficient batch evaluator for circle grid points: P(u) = C + R * (cos(u) * X + sin(u) * Y).
/// </summary>
public sealed class GeomGridEval_Circle : GeomGridEval.ICurveEvaluator
{
    private readonly Geom_Circle? myGeom;

    /// <summary>Constructor with the circle to evaluate (null gives empty results).</summary>
    public GeomGridEval_Circle(Geom_Circle? theCircle)
    {
        myGeom = theCircle;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_Circle? Geometry() => myGeom;

    /// <summary>Evaluates all grid points; 1-based, or empty if the geometry is null or there are no parameters.</summary>
    public NCollection_Array1<gp_Pnt> EvaluateGrid(NCollection_Array1<double> theParams)
    {
        if (myGeom is null || theParams.IsEmpty())
        {
            return new NCollection_Array1<gp_Pnt>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<gp_Pnt> aResult = new(1, aNb);

        // Extract circle data from geometry
        gp_Circ aCirc = myGeom.Circ();
        gp_Pnt aCenter = aCirc.Location();
        gp_Dir aXDir = aCirc.Position().XDirection();
        gp_Dir aYDir = aCirc.Position().YDirection();
        double aRadius = aCirc.Radius();

        // Pre-extract coordinates for performance
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

            // P = C + R * (cos(u) * X + sin(u) * Y)
            aResult.SetValue(i - theParams.Lower() + 1,
                             new gp_Pnt(aCX + aRadius * (cosU * aXX + sinU * aYX),
                                        aCY + aRadius * (cosU * aXY + sinU * aYY),
                                        aCZ + aRadius * (cosU * aXZ + sinU * aYZ)));
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

        // Extract circle data from geometry
        gp_Circ aCirc = myGeom.Circ();
        gp_Pnt aCenter = aCirc.Location();
        gp_Dir aXDir = aCirc.Position().XDirection();
        gp_Dir aYDir = aCirc.Position().YDirection();
        double aRadius = aCirc.Radius();

        // Pre-extract coordinates for performance
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

            // P = C + R * (cos(u) * X + sin(u) * Y)
            // D1 = R * (-sin(u) * X + cos(u) * Y)
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD1
            {
                Point = new gp_Pnt(aCX + aRadius * (cosU * aXX + sinU * aYX),
                                   aCY + aRadius * (cosU * aXY + sinU * aYY),
                                   aCZ + aRadius * (cosU * aXZ + sinU * aYZ)),
                D1 = new gp_Vec(aRadius * (-sinU * aXX + cosU * aYX),
                                aRadius * (-sinU * aXY + cosU * aYY),
                                aRadius * (-sinU * aXZ + cosU * aYZ)),
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

        // Extract circle data from geometry
        gp_Circ aCirc = myGeom.Circ();
        gp_Pnt aCenter = aCirc.Location();
        gp_Dir aXDir = aCirc.Position().XDirection();
        gp_Dir aYDir = aCirc.Position().YDirection();
        double aRadius = aCirc.Radius();

        // Pre-extract coordinates for performance
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

            // P = C + R * (cos(u) * X + sin(u) * Y)
            // D1 = R * (-sin(u) * X + cos(u) * Y)
            // D2 = R * (-cos(u) * X - sin(u) * Y)
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD2
            {
                Point = new gp_Pnt(aCX + aRadius * (cosU * aXX + sinU * aYX),
                                   aCY + aRadius * (cosU * aXY + sinU * aYY),
                                   aCZ + aRadius * (cosU * aXZ + sinU * aYZ)),
                D1 = new gp_Vec(aRadius * (-sinU * aXX + cosU * aYX),
                                aRadius * (-sinU * aXY + cosU * aYY),
                                aRadius * (-sinU * aXZ + cosU * aYZ)),
                D2 = new gp_Vec(aRadius * (-cosU * aXX - sinU * aYX),
                                aRadius * (-cosU * aXY - sinU * aYY),
                                aRadius * (-cosU * aXZ - sinU * aYZ)),
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

        // Extract circle data from geometry
        gp_Circ aCirc = myGeom.Circ();
        gp_Pnt aCenter = aCirc.Location();
        gp_Dir aXDir = aCirc.Position().XDirection();
        gp_Dir aYDir = aCirc.Position().YDirection();
        double aRadius = aCirc.Radius();

        // Pre-extract coordinates for performance
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

            // P = C + R * (cos(u) * X + sin(u) * Y)
            // D1 = R * (-sin(u) * X + cos(u) * Y)
            // D2 = R * (-cos(u) * X - sin(u) * Y)
            // D3 = R * (sin(u) * X - cos(u) * Y)
            aResult[i - theParams.Lower() + 1] = new Geom_Curve.ResD3
            {
                Point = new gp_Pnt(aCX + aRadius * (cosU * aXX + sinU * aYX),
                                   aCY + aRadius * (cosU * aXY + sinU * aYY),
                                   aCZ + aRadius * (cosU * aXZ + sinU * aYZ)),
                D1 = new gp_Vec(aRadius * (-sinU * aXX + cosU * aYX),
                                aRadius * (-sinU * aXY + cosU * aYY),
                                aRadius * (-sinU * aXZ + cosU * aYZ)),
                D2 = new gp_Vec(aRadius * (-cosU * aXX - sinU * aYX),
                                aRadius * (-cosU * aXY - sinU * aYY),
                                aRadius * (-cosU * aXZ - sinU * aYZ)),
                D3 = new gp_Vec(aRadius * (sinU * aXX - cosU * aYX),
                                aRadius * (sinU * aXY - cosU * aYY),
                                aRadius * (sinU * aXZ - cosU * aYZ)),
            };
        }
        return aResult;
    }

    /// <summary>Evaluates the Nth derivative (N &gt;= 1) at all grid points; the derivatives cycle with period 4.</summary>
    public NCollection_Array1<gp_Vec> EvaluateGridDN(NCollection_Array1<double> theParams, int theN)
    {
        if (myGeom is null || theParams.IsEmpty() || theN < 1)
        {
            return new NCollection_Array1<gp_Vec>();
        }

        int aNb = theParams.Length();
        NCollection_Array1<gp_Vec> aResult = new(1, aNb);

        gp_Circ aCirc = myGeom.Circ();
        gp_Dir aXDir = aCirc.Position().XDirection();
        gp_Dir aYDir = aCirc.Position().YDirection();
        double aRadius = aCirc.Radius();

        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        // Circle derivatives are cyclic with period 4:
        // D1 = R * (-sin(u) * X + cos(u) * Y)  -> coefficients: (-sin, cos)
        // D2 = R * (-cos(u) * X - sin(u) * Y)  -> coefficients: (-cos, -sin)
        // D3 = R * (sin(u) * X - cos(u) * Y)   -> coefficients: (sin, -cos)
        // D4 = R * (cos(u) * X + sin(u) * Y)   -> coefficients: (cos, sin) = D0
        int aPhase = (theN - 1) % 4; // 0->D1, 1->D2, 2->D3, 3->D4

        for (int i = theParams.Lower(); i <= theParams.Upper(); ++i)
        {
            double u = theParams.Value(i);
            double cosU = Math.Cos(u);
            double sinU = Math.Sin(u);

            double aCoeffX, aCoeffY;
            switch (aPhase)
            {
                case 0: // D1, D5, D9, ...
                    aCoeffX = -sinU;
                    aCoeffY = cosU;
                    break;
                case 1: // D2, D6, D10, ...
                    aCoeffX = -cosU;
                    aCoeffY = -sinU;
                    break;
                case 2: // D3, D7, D11, ...
                    aCoeffX = sinU;
                    aCoeffY = -cosU;
                    break;
                default: // D4, D8, D12, ... (case 3)
                    aCoeffX = cosU;
                    aCoeffY = sinU;
                    break;
            }

            aResult.SetValue(i - theParams.Lower() + 1,
                             new gp_Vec(aRadius * (aCoeffX * aXX + aCoeffY * aYX),
                                        aRadius * (aCoeffX * aXY + aCoeffY * aYY),
                                        aRadius * (aCoeffX * aXZ + aCoeffY * aYZ)));
        }
        return aResult;
    }
}
