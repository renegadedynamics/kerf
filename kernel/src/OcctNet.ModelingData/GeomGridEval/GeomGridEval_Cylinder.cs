// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Cylinder.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Cylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Efficient batch evaluator for cylindrical surface grid points:
/// P(u, v) = Center + R * (cos(u) * XDir + sin(u) * YDir) + v * ZDir. The trigonometric values are computed
/// once per U parameter.
/// </summary>
public sealed class GeomGridEval_Cylinder : GeomGridEval.ISurfaceEvaluator
{
    private struct Data
    {
        public double CX, CY, CZ; // Center coordinates
        public double XX, XY, XZ; // XDir coordinates
        public double YX, YY, YZ; // YDir coordinates
        public double ZX, ZY, ZZ; // ZDir coordinates
        public double Radius;
    }

    private struct UContext
    {
        public double cosU, sinU;
        public double dirUX, dirUY, dirUZ;    // DirU = cosU*XDir + sinU*YDir (circle point direction)
        public double dDirUX, dDirUY, dDirUZ; // DerivDirU = -sinU*XDir + cosU*YDir
    }

    private readonly Geom_CylindricalSurface? myGeom;

    /// <summary>Constructor with the cylinder to evaluate (null gives empty results).</summary>
    public GeomGridEval_Cylinder(Geom_CylindricalSurface? theCylinder)
    {
        myGeom = theCylinder;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_CylindricalSurface? Geometry() => myGeom;

    private Data extractData()
    {
        Cylinder aCyl = myGeom!.Cylinder();
        Point3D aCenter = aCyl.Location();
        Direction3D aXDir = aCyl.Position().XDirection();
        Direction3D aYDir = aCyl.Position().YDirection();
        Direction3D aZDir = aCyl.Position().Direction();
        return new Data
        {
            CX = aCenter.X(),
            CY = aCenter.Y(),
            CZ = aCenter.Z(),
            XX = aXDir.X(),
            XY = aXDir.Y(),
            XZ = aXDir.Z(),
            YX = aYDir.X(),
            YY = aYDir.Y(),
            YZ = aYDir.Z(),
            ZX = aZDir.X(),
            ZY = aZDir.Y(),
            ZZ = aZDir.Z(),
            Radius = aCyl.Radius(),
        };
    }

    private static UContext computeUContext(in Data theData, double theU)
    {
        double cosU = Math.Cos(theU);
        double sinU = Math.Sin(theU);
        // DirU = cosU*XDir + sinU*YDir (circle point direction at angle U)
        // dDirU = -sinU*XDir + cosU*YDir (derivative of DirU w.r.t. U)
        return new UContext
        {
            cosU = cosU,
            sinU = sinU,
            dirUX = cosU * theData.XX + sinU * theData.YX,
            dirUY = cosU * theData.XY + sinU * theData.YY,
            dirUZ = cosU * theData.XZ + sinU * theData.YZ,
            dDirUX = -sinU * theData.XX + cosU * theData.YX,
            dDirUY = -sinU * theData.XY + cosU * theData.YY,
            dDirUZ = -sinU * theData.XZ + cosU * theData.YZ,
        };
    }

    private static Point3D computeD0(in Data theData, in UContext theUCtx, double theV)
    {
        // P = Center + R * DirU + v * ZDir
        return new Point3D(theData.CX + theData.Radius * theUCtx.dirUX + theV * theData.ZX,
                          theData.CY + theData.Radius * theUCtx.dirUY + theV * theData.ZY,
                          theData.CZ + theData.Radius * theUCtx.dirUZ + theV * theData.ZZ);
    }

    private static Geom_Surface.ResD1 computeD1(in Data theData, in UContext theUCtx, double theV)
    {
        // D1U = R * dDirU
        // D1V = ZDir
        return new Geom_Surface.ResD1
        {
            Point = new Point3D(theData.CX + theData.Radius * theUCtx.dirUX + theV * theData.ZX,
                               theData.CY + theData.Radius * theUCtx.dirUY + theV * theData.ZY,
                               theData.CZ + theData.Radius * theUCtx.dirUZ + theV * theData.ZZ),
            D1U = new Vector3D(theData.Radius * theUCtx.dDirUX,
                             theData.Radius * theUCtx.dDirUY,
                             theData.Radius * theUCtx.dDirUZ),
            D1V = new Vector3D(theData.ZX, theData.ZY, theData.ZZ),
        };
    }

    private static Geom_Surface.ResD2 computeD2(in Data theData, in UContext theUCtx, double theV)
    {
        // D2U = R * (-DirU) = -R * DirU
        // D2V = 0
        // D2UV = 0
        Vector3D aZero = new(0.0, 0.0, 0.0);
        return new Geom_Surface.ResD2
        {
            Point = new Point3D(theData.CX + theData.Radius * theUCtx.dirUX + theV * theData.ZX,
                               theData.CY + theData.Radius * theUCtx.dirUY + theV * theData.ZY,
                               theData.CZ + theData.Radius * theUCtx.dirUZ + theV * theData.ZZ),
            D1U = new Vector3D(theData.Radius * theUCtx.dDirUX,
                             theData.Radius * theUCtx.dDirUY,
                             theData.Radius * theUCtx.dDirUZ),
            D1V = new Vector3D(theData.ZX, theData.ZY, theData.ZZ),
            D2U = new Vector3D(-theData.Radius * theUCtx.dirUX,
                             -theData.Radius * theUCtx.dirUY,
                             -theData.Radius * theUCtx.dirUZ),
            D2V = aZero,
            D2UV = aZero,
        };
    }

    private static Geom_Surface.ResD3 computeD3(in Data theData, in UContext theUCtx, double theV)
    {
        // D3U = R * (-dDirU) = -D1U
        // D3V = 0
        // D3UUV = 0
        // D3UVV = 0
        Vector3D aZero = new(0.0, 0.0, 0.0);
        return new Geom_Surface.ResD3
        {
            Point = new Point3D(theData.CX + theData.Radius * theUCtx.dirUX + theV * theData.ZX,
                               theData.CY + theData.Radius * theUCtx.dirUY + theV * theData.ZY,
                               theData.CZ + theData.Radius * theUCtx.dirUZ + theV * theData.ZZ),
            D1U = new Vector3D(theData.Radius * theUCtx.dDirUX,
                             theData.Radius * theUCtx.dDirUY,
                             theData.Radius * theUCtx.dDirUZ),
            D1V = new Vector3D(theData.ZX, theData.ZY, theData.ZZ),
            D2U = new Vector3D(-theData.Radius * theUCtx.dirUX,
                             -theData.Radius * theUCtx.dirUY,
                             -theData.Radius * theUCtx.dirUZ),
            D2V = aZero,
            D2UV = aZero,
            D3U = new Vector3D(-theData.Radius * theUCtx.dDirUX,
                             -theData.Radius * theUCtx.dDirUY,
                             -theData.Radius * theUCtx.dDirUZ),
            D3V = aZero,
            D3UUV = aZero,
            D3UVV = aZero,
        };
    }

    private static Vector3D computeDN(in Data theData, in UContext theUCtx, double theV, int theNU, int theNV)
    {
        _ = theV;
        // For cylinder P(u,v) = C + R*(cos(u)*X + sin(u)*Y) + v*Z:
        // - V derivatives: only D_{0,1} = Z is non-zero, all D_{nu,nv} = 0 for nv >= 2
        // - Mixed derivatives D_{nu,1} = 0 for nu >= 1 (Z term has no U dependence)
        // - Pure U derivatives cycle with period 4
        if (theNV > 1)
        {
            return new Vector3D(0.0, 0.0, 0.0);
        }
        if (theNV == 1)
        {
            if (theNU == 0)
            {
                return new Vector3D(theData.ZX, theData.ZY, theData.ZZ);
            }
            return new Vector3D(0.0, 0.0, 0.0);
        }

        // Pure U derivatives (theNV == 0, theNU >= 1)
        // d^n/du^n of DirU cycles: DirU, dDirU, -DirU, -dDirU
        int aPhase = theNU % 4;
        double dirX, dirY, dirZ;
        switch (aPhase)
        {
            case 0:
                dirX = theUCtx.dirUX;
                dirY = theUCtx.dirUY;
                dirZ = theUCtx.dirUZ;
                break;
            case 1:
                dirX = theUCtx.dDirUX;
                dirY = theUCtx.dDirUY;
                dirZ = theUCtx.dDirUZ;
                break;
            case 2:
                dirX = -theUCtx.dirUX;
                dirY = -theUCtx.dirUY;
                dirZ = -theUCtx.dirUZ;
                break;
            case 3:
                dirX = -theUCtx.dDirUX;
                dirY = -theUCtx.dDirUY;
                dirZ = -theUCtx.dDirUZ;
                break;
            default:
                dirX = 0.0;
                dirY = 0.0;
                dirZ = 0.0;
                break;
        }
        return new Vector3D(theData.Radius * dirX, theData.Radius * dirY, theData.Radius * dirZ);
    }

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty if the geometry is null or a list is empty.</summary>
    public NCollection_Array2<Point3D> EvaluateGrid(NCollection_Array1<double> theUParams,
                                                   NCollection_Array1<double> theVParams)
    {
        if (myGeom is null || theUParams.IsEmpty() || theVParams.IsEmpty())
        {
            return new NCollection_Array2<Point3D>();
        }

        Data aData = extractData();
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        NCollection_Array2<Point3D> aResult = new(1, aNbU, 1, aNbV);

        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double aU = theUParams.Value(theUParams.Lower() + iU - 1);
            UContext aUCtx = computeUContext(aData, aU);

            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double aV = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult.SetValue(iU, iV, computeD0(aData, aUCtx, aV));
            }
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with first partial derivatives.</summary>
    public NCollection_Array2<Geom_Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        if (myGeom is null || theUParams.IsEmpty() || theVParams.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD1>();
        }

        Data aData = extractData();
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        NCollection_Array2<Geom_Surface.ResD1> aResult = new(1, aNbU, 1, aNbV);

        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double aU = theUParams.Value(theUParams.Lower() + iU - 1);
            UContext aUCtx = computeUContext(aData, aU);

            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double aV = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult.SetValue(iU, iV, computeD1(aData, aUCtx, aV));
            }
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the second order.</summary>
    public NCollection_Array2<Geom_Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        if (myGeom is null || theUParams.IsEmpty() || theVParams.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD2>();
        }

        Data aData = extractData();
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        NCollection_Array2<Geom_Surface.ResD2> aResult = new(1, aNbU, 1, aNbV);

        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double aU = theUParams.Value(theUParams.Lower() + iU - 1);
            UContext aUCtx = computeUContext(aData, aU);

            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double aV = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult.SetValue(iU, iV, computeD2(aData, aUCtx, aV));
            }
        }
        return aResult;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the third order.</summary>
    public NCollection_Array2<Geom_Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> theUParams,
                                                                 NCollection_Array1<double> theVParams)
    {
        if (myGeom is null || theUParams.IsEmpty() || theVParams.IsEmpty())
        {
            return new NCollection_Array2<Geom_Surface.ResD3>();
        }

        Data aData = extractData();
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        NCollection_Array2<Geom_Surface.ResD3> aResult = new(1, aNbU, 1, aNbV);

        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double aU = theUParams.Value(theUParams.Lower() + iU - 1);
            UContext aUCtx = computeUContext(aData, aU);

            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double aV = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult.SetValue(iU, iV, computeD3(aData, aUCtx, aV));
            }
        }
        return aResult;
    }

    /// <summary>Evaluates the partial derivative of orders (NU, NV) on the grid; empty unless NU, NV &gt;= 0 and NU + NV &gt;= 1.</summary>
    public NCollection_Array2<Vector3D> EvaluateGridDN(NCollection_Array1<double> theUParams,
                                                     NCollection_Array1<double> theVParams,
                                                     int theNU,
                                                     int theNV)
    {
        if (myGeom is null || theUParams.IsEmpty() || theVParams.IsEmpty() || theNU < 0 || theNV < 0
            || (theNU + theNV) < 1)
        {
            return new NCollection_Array2<Vector3D>();
        }

        Data aData = extractData();
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        NCollection_Array2<Vector3D> aResult = new(1, aNbU, 1, aNbV);

        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double aU = theUParams.Value(theUParams.Lower() + iU - 1);
            UContext aUCtx = computeUContext(aData, aU);

            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double aV = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult.SetValue(iU, iV, computeDN(aData, aUCtx, aV, theNU, theNV));
            }
        }
        return aResult;
    }
}
