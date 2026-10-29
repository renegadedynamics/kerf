// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Plane.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Efficient batch evaluator for plane grid points, by the direct formula P(u, v) = Location + u * XDir + v * YDir.
/// </summary>
public sealed class GeomGridEval_Plane : GeomGridEval.ISurfaceEvaluator
{
    private readonly Geom_Plane? myGeom;

    /// <summary>Constructor with the plane to evaluate (null gives empty results).</summary>
    public GeomGridEval_Plane(Geom_Plane? thePlane)
    {
        myGeom = thePlane;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_Plane? Geometry() => myGeom;

    /// <summary>Evaluates grid points at the Cartesian product of U and V parameters; [1..NbU, 1..NbV].</summary>
    public NCollection_Array2<gp_Pnt> EvaluateGrid(NCollection_Array1<double> theUParams,
                                                   NCollection_Array1<double> theVParams)
    {
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        if (myGeom is null || aNbU == 0 || aNbV == 0)
        {
            return new NCollection_Array2<gp_Pnt>();
        }

        NCollection_Array2<gp_Pnt> aResult = new(1, aNbU, 1, aNbV);

        // Extract plane data from geometry
        gp_Pln aPln = myGeom.Pln();
        gp_Pnt aLoc = aPln.Location();
        gp_Dir aXDir = aPln.Position().XDirection();
        gp_Dir aYDir = aPln.Position().YDirection();

        // Pre-extract coordinates for performance
        double aLocX = aLoc.X();
        double aLocY = aLoc.Y();
        double aLocZ = aLoc.Z();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double u = theUParams.Value(theUParams.Lower() + iU - 1);
            double uX = aLocX + u * aXX;
            double uY = aLocY + u * aXY;
            double uZ = aLocZ + u * aXZ;

            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double v = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult.SetValue(iU, iV, new gp_Pnt(uX + v * aYX, uY + v * aYY, uZ + v * aYZ));
            }
        }
        return aResult;
    }

    /// <summary>Evaluates grid points with first partial derivatives (constant for a plane).</summary>
    public NCollection_Array2<Geom_Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        if (myGeom is null || aNbU == 0 || aNbV == 0)
        {
            return new NCollection_Array2<Geom_Surface.ResD1>();
        }

        NCollection_Array2<Geom_Surface.ResD1> aResult = new(1, aNbU, 1, aNbV);

        gp_Pln aPln = myGeom.Pln();
        gp_Pnt aLoc = aPln.Location();
        gp_Dir aXDir = aPln.Position().XDirection();
        gp_Dir aYDir = aPln.Position().YDirection();

        double aLocX = aLoc.X();
        double aLocY = aLoc.Y();
        double aLocZ = aLoc.Z();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        // D1U and D1V are constant for a plane
        gp_Vec aD1U = new(aXX, aXY, aXZ);
        gp_Vec aD1V = new(aYX, aYY, aYZ);

        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double u = theUParams.Value(theUParams.Lower() + iU - 1);
            double uX = aLocX + u * aXX;
            double uY = aLocY + u * aXY;
            double uZ = aLocZ + u * aXZ;

            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double v = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult[iU, iV] = new Geom_Surface.ResD1
                {
                    Point = new gp_Pnt(uX + v * aYX, uY + v * aYY, uZ + v * aYZ),
                    D1U = aD1U,
                    D1V = aD1V,
                };
            }
        }
        return aResult;
    }

    /// <summary>Evaluates grid points with first and second partial derivatives.</summary>
    public NCollection_Array2<Geom_Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        if (myGeom is null || aNbU == 0 || aNbV == 0)
        {
            return new NCollection_Array2<Geom_Surface.ResD2>();
        }

        NCollection_Array2<Geom_Surface.ResD2> aResult = new(1, aNbU, 1, aNbV);

        gp_Pln aPln = myGeom.Pln();
        gp_Pnt aLoc = aPln.Location();
        gp_Dir aXDir = aPln.Position().XDirection();
        gp_Dir aYDir = aPln.Position().YDirection();

        double aLocX = aLoc.X();
        double aLocY = aLoc.Y();
        double aLocZ = aLoc.Z();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        gp_Vec aD1U = new(aXX, aXY, aXZ);
        gp_Vec aD1V = new(aYX, aYY, aYZ);
        gp_Vec aZero = new(0, 0, 0); // All second derivatives are zero for a plane

        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double u = theUParams.Value(theUParams.Lower() + iU - 1);
            double uX = aLocX + u * aXX;
            double uY = aLocY + u * aXY;
            double uZ = aLocZ + u * aXZ;

            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double v = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult[iU, iV] = new Geom_Surface.ResD2
                {
                    Point = new gp_Pnt(uX + v * aYX, uY + v * aYY, uZ + v * aYZ),
                    D1U = aD1U,
                    D1V = aD1V,
                    D2U = aZero,
                    D2V = aZero,
                    D2UV = aZero,
                };
            }
        }
        return aResult;
    }

    /// <summary>Evaluates grid points with derivatives up to the third order.</summary>
    public NCollection_Array2<Geom_Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        if (myGeom is null || aNbU == 0 || aNbV == 0)
        {
            return new NCollection_Array2<Geom_Surface.ResD3>();
        }

        NCollection_Array2<Geom_Surface.ResD3> aResult = new(1, aNbU, 1, aNbV);

        gp_Pln aPln = myGeom.Pln();
        gp_Pnt aLoc = aPln.Location();
        gp_Dir aXDir = aPln.Position().XDirection();
        gp_Dir aYDir = aPln.Position().YDirection();

        double aLocX = aLoc.X();
        double aLocY = aLoc.Y();
        double aLocZ = aLoc.Z();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        gp_Vec aD1U = new(aXX, aXY, aXZ);
        gp_Vec aD1V = new(aYX, aYY, aYZ);
        gp_Vec aZero = new(0, 0, 0); // All derivatives of order >= 2 are zero for a plane

        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double u = theUParams.Value(theUParams.Lower() + iU - 1);
            double uX = aLocX + u * aXX;
            double uY = aLocY + u * aXY;
            double uZ = aLocZ + u * aXZ;

            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double v = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult[iU, iV] = new Geom_Surface.ResD3
                {
                    Point = new gp_Pnt(uX + v * aYX, uY + v * aYY, uZ + v * aYZ),
                    D1U = aD1U,
                    D1V = aD1V,
                    D2U = aZero,
                    D2V = aZero,
                    D2UV = aZero,
                    D3U = aZero,
                    D3V = aZero,
                    D3UUV = aZero,
                    D3UVV = aZero,
                };
            }
        }
        return aResult;
    }

    /// <summary>Evaluates the partial derivative d^(NU+NV)S/(dU^NU dV^NV) at all grid points.</summary>
    public NCollection_Array2<gp_Vec> EvaluateGridDN(NCollection_Array1<double> theUParams,
                                                     NCollection_Array1<double> theVParams,
                                                     int theNU,
                                                     int theNV)
    {
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        if (myGeom is null || aNbU == 0 || aNbV == 0 || theNU < 0 || theNV < 0 || (theNU + theNV) < 1)
        {
            return new NCollection_Array2<gp_Vec>();
        }

        NCollection_Array2<gp_Vec> aResult = new(1, aNbU, 1, aNbV);

        // For a plane, only D1U (1,0) and D1V (0,1) are non-zero
        gp_Vec aDerivative = new(0, 0, 0);
        if (theNU == 1 && theNV == 0)
        {
            // D1U = XDir
            gp_Dir aXDir = myGeom.Pln().Position().XDirection();
            aDerivative = new gp_Vec(aXDir.X(), aXDir.Y(), aXDir.Z());
        }
        else if (theNU == 0 && theNV == 1)
        {
            // D1V = YDir
            gp_Dir aYDir = myGeom.Pln().Position().YDirection();
            aDerivative = new gp_Vec(aYDir.X(), aYDir.Y(), aYDir.Z());
        }
        // All other derivatives are zero

        for (int iU = 1; iU <= aNbU; ++iU)
        {
            for (int iV = 1; iV <= aNbV; ++iV)
            {
                aResult.SetValue(iU, iV, aDerivative);
            }
        }
        return aResult;
    }
}
