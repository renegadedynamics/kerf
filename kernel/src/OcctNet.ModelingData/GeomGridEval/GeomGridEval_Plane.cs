// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Plane.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

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
    public NCollection_Array2<Point3D> EvaluateGrid(NCollection_Array1<double> theUParams,
                                                   NCollection_Array1<double> theVParams)
    {
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        if (myGeom is null || aNbU == 0 || aNbV == 0)
        {
            return new NCollection_Array2<Point3D>();
        }

        NCollection_Array2<Point3D> aResult = new(1, aNbU, 1, aNbV);

        // Extract plane data from geometry
        Plane3D aPln = myGeom.Pln();
        Point3D aLoc = aPln.Location();
        Direction3D aXDir = aPln.Position().XDirection();
        Direction3D aYDir = aPln.Position().YDirection();

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
                aResult.SetValue(iU, iV, new Point3D(uX + v * aYX, uY + v * aYY, uZ + v * aYZ));
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

        Plane3D aPln = myGeom.Pln();
        Point3D aLoc = aPln.Location();
        Direction3D aXDir = aPln.Position().XDirection();
        Direction3D aYDir = aPln.Position().YDirection();

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
        Vector3D aD1U = new(aXX, aXY, aXZ);
        Vector3D aD1V = new(aYX, aYY, aYZ);

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
                    Point = new Point3D(uX + v * aYX, uY + v * aYY, uZ + v * aYZ),
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

        Plane3D aPln = myGeom.Pln();
        Point3D aLoc = aPln.Location();
        Direction3D aXDir = aPln.Position().XDirection();
        Direction3D aYDir = aPln.Position().YDirection();

        double aLocX = aLoc.X();
        double aLocY = aLoc.Y();
        double aLocZ = aLoc.Z();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        Vector3D aD1U = new(aXX, aXY, aXZ);
        Vector3D aD1V = new(aYX, aYY, aYZ);
        Vector3D aZero = new(0, 0, 0); // All second derivatives are zero for a plane

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
                    Point = new Point3D(uX + v * aYX, uY + v * aYY, uZ + v * aYZ),
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

        Plane3D aPln = myGeom.Pln();
        Point3D aLoc = aPln.Location();
        Direction3D aXDir = aPln.Position().XDirection();
        Direction3D aYDir = aPln.Position().YDirection();

        double aLocX = aLoc.X();
        double aLocY = aLoc.Y();
        double aLocZ = aLoc.Z();
        double aXX = aXDir.X();
        double aXY = aXDir.Y();
        double aXZ = aXDir.Z();
        double aYX = aYDir.X();
        double aYY = aYDir.Y();
        double aYZ = aYDir.Z();

        Vector3D aD1U = new(aXX, aXY, aXZ);
        Vector3D aD1V = new(aYX, aYY, aYZ);
        Vector3D aZero = new(0, 0, 0); // All derivatives of order >= 2 are zero for a plane

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
                    Point = new Point3D(uX + v * aYX, uY + v * aYY, uZ + v * aYZ),
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
    public NCollection_Array2<Vector3D> EvaluateGridDN(NCollection_Array1<double> theUParams,
                                                     NCollection_Array1<double> theVParams,
                                                     int theNU,
                                                     int theNV)
    {
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();
        if (myGeom is null || aNbU == 0 || aNbV == 0 || theNU < 0 || theNV < 0 || (theNU + theNV) < 1)
        {
            return new NCollection_Array2<Vector3D>();
        }

        NCollection_Array2<Vector3D> aResult = new(1, aNbU, 1, aNbV);

        // For a plane, only D1U (1,0) and D1V (0,1) are non-zero
        Vector3D aDerivative = new(0, 0, 0);
        if (theNU == 1 && theNV == 0)
        {
            // D1U = XDir
            Direction3D aXDir = myGeom.Pln().Position().XDirection();
            aDerivative = new Vector3D(aXDir.X(), aXDir.Y(), aXDir.Z());
        }
        else if (theNU == 0 && theNV == 1)
        {
            // D1V = YDir
            Direction3D aYDir = myGeom.Pln().Position().YDirection();
            aDerivative = new Vector3D(aYDir.X(), aYDir.Y(), aYDir.Z());
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
