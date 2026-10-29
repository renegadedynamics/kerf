// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Sphere.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Efficient batch evaluator for spherical surface grid points:
/// P(u, v) = Center + R * (cos(v) * (cos(u) * XDir + sin(u) * YDir) + sin(v) * ZDir). The trigonometric values of
/// U are computed once per U parameter.
/// </summary>
public sealed class GeomGridEval_Sphere : GeomGridEval.ISurfaceEvaluator
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
        public double dirUX, dirUY, dirUZ;    // DirU = cosU*XDir + sinU*YDir
        public double dDirUX, dDirUY, dDirUZ; // DerivDirU = -sinU*XDir + cosU*YDir
    }

    private readonly Geom_SphericalSurface? myGeom;

    /// <summary>Constructor with the sphere to evaluate (null gives empty results).</summary>
    public GeomGridEval_Sphere(Geom_SphericalSurface? theSphere)
    {
        myGeom = theSphere;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_SphericalSurface? Geometry() => myGeom;

    private Data extractData()
    {
        gp_Sphere aSph = myGeom!.Sphere();
        gp_Pnt aCenter = aSph.Location();
        gp_Dir aXDir = aSph.Position().XDirection();
        gp_Dir aYDir = aSph.Position().YDirection();
        gp_Dir aZDir = aSph.Position().Direction();
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
            Radius = aSph.Radius(),
        };
    }

    private static UContext computeUContext(in Data theData, double theU)
    {
        double cosU = Math.Cos(theU);
        double sinU = Math.Sin(theU);
        // DirU = cosU*XDir + sinU*YDir (horizontal direction at angle U)
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

    private static gp_Pnt computeD0(in Data theData, in UContext theUCtx, double theV)
    {
        double cosV = Math.Cos(theV);
        double sinV = Math.Sin(theV);

        // P = Center + R * (cosV * DirU + sinV * ZDir)
        double RcosV = theData.Radius * cosV;
        double RsinV = theData.Radius * sinV;
        return new gp_Pnt(theData.CX + RcosV * theUCtx.dirUX + RsinV * theData.ZX,
                          theData.CY + RcosV * theUCtx.dirUY + RsinV * theData.ZY,
                          theData.CZ + RcosV * theUCtx.dirUZ + RsinV * theData.ZZ);
    }

    private static Geom_Surface.ResD1 computeD1(in Data theData, in UContext theUCtx, double theV)
    {
        double cosV = Math.Cos(theV);
        double sinV = Math.Sin(theV);

        double RcosV = theData.Radius * cosV;
        double RsinV = theData.Radius * sinV;

        // D1U = R * cosV * dDirU
        // D1V = R * (-sinV * DirU + cosV * ZDir)
        return new Geom_Surface.ResD1
        {
            Point = new gp_Pnt(theData.CX + RcosV * theUCtx.dirUX + RsinV * theData.ZX,
                               theData.CY + RcosV * theUCtx.dirUY + RsinV * theData.ZY,
                               theData.CZ + RcosV * theUCtx.dirUZ + RsinV * theData.ZZ),
            D1U = new gp_Vec(RcosV * theUCtx.dDirUX, RcosV * theUCtx.dDirUY, RcosV * theUCtx.dDirUZ),
            D1V = new gp_Vec(-RsinV * theUCtx.dirUX + RcosV * theData.ZX,
                             -RsinV * theUCtx.dirUY + RcosV * theData.ZY,
                             -RsinV * theUCtx.dirUZ + RcosV * theData.ZZ),
        };
    }

    private static Geom_Surface.ResD2 computeD2(in Data theData, in UContext theUCtx, double theV)
    {
        double cosV = Math.Cos(theV);
        double sinV = Math.Sin(theV);

        double RcosV = theData.Radius * cosV;
        double RsinV = theData.Radius * sinV;

        // D2U = R * cosV * (-DirU) = -R * cosV * DirU
        // D2V = R * (-cosV * DirU - sinV * ZDir) = -(P - Center) = -relP
        // D2UV = d/dV(D1U) = d/dV(R * cosV * dDirU) = -R * sinV * dDirU
        return new Geom_Surface.ResD2
        {
            Point = new gp_Pnt(theData.CX + RcosV * theUCtx.dirUX + RsinV * theData.ZX,
                               theData.CY + RcosV * theUCtx.dirUY + RsinV * theData.ZY,
                               theData.CZ + RcosV * theUCtx.dirUZ + RsinV * theData.ZZ),
            D1U = new gp_Vec(RcosV * theUCtx.dDirUX, RcosV * theUCtx.dDirUY, RcosV * theUCtx.dDirUZ),
            D1V = new gp_Vec(-RsinV * theUCtx.dirUX + RcosV * theData.ZX,
                             -RsinV * theUCtx.dirUY + RcosV * theData.ZY,
                             -RsinV * theUCtx.dirUZ + RcosV * theData.ZZ),
            D2U = new gp_Vec(-RcosV * theUCtx.dirUX, -RcosV * theUCtx.dirUY, -RcosV * theUCtx.dirUZ),
            D2V = new gp_Vec(-RcosV * theUCtx.dirUX - RsinV * theData.ZX,
                             -RcosV * theUCtx.dirUY - RsinV * theData.ZY,
                             -RcosV * theUCtx.dirUZ - RsinV * theData.ZZ),
            D2UV = new gp_Vec(-RsinV * theUCtx.dDirUX, -RsinV * theUCtx.dDirUY, -RsinV * theUCtx.dDirUZ),
        };
    }

    private static Geom_Surface.ResD3 computeD3(in Data theData, in UContext theUCtx, double theV)
    {
        double cosV = Math.Cos(theV);
        double sinV = Math.Sin(theV);

        double RcosV = theData.Radius * cosV;
        double RsinV = theData.Radius * sinV;

        // D3U = R * cosV * dDirU = D1U
        // D3V = R * (sinV * DirU - cosV * ZDir) = -D1V
        // D3UUV = d/dV(D2U) = d/dV(-R * cosV * DirU) = R * sinV * DirU
        // D3UVV = d/dV(D2UV) = d/dV(-R * sinV * dDirU) = -R * cosV * dDirU
        return new Geom_Surface.ResD3
        {
            Point = new gp_Pnt(theData.CX + RcosV * theUCtx.dirUX + RsinV * theData.ZX,
                               theData.CY + RcosV * theUCtx.dirUY + RsinV * theData.ZY,
                               theData.CZ + RcosV * theUCtx.dirUZ + RsinV * theData.ZZ),
            D1U = new gp_Vec(RcosV * theUCtx.dDirUX, RcosV * theUCtx.dDirUY, RcosV * theUCtx.dDirUZ),
            D1V = new gp_Vec(-RsinV * theUCtx.dirUX + RcosV * theData.ZX,
                             -RsinV * theUCtx.dirUY + RcosV * theData.ZY,
                             -RsinV * theUCtx.dirUZ + RcosV * theData.ZZ),
            D2U = new gp_Vec(-RcosV * theUCtx.dirUX, -RcosV * theUCtx.dirUY, -RcosV * theUCtx.dirUZ),
            D2V = new gp_Vec(-RcosV * theUCtx.dirUX - RsinV * theData.ZX,
                             -RcosV * theUCtx.dirUY - RsinV * theData.ZY,
                             -RcosV * theUCtx.dirUZ - RsinV * theData.ZZ),
            D2UV = new gp_Vec(-RsinV * theUCtx.dDirUX, -RsinV * theUCtx.dDirUY, -RsinV * theUCtx.dDirUZ),
            D3U = new gp_Vec(-RcosV * theUCtx.dDirUX, -RcosV * theUCtx.dDirUY, -RcosV * theUCtx.dDirUZ),
            D3V = new gp_Vec(RsinV * theUCtx.dirUX - RcosV * theData.ZX,
                             RsinV * theUCtx.dirUY - RcosV * theData.ZY,
                             RsinV * theUCtx.dirUZ - RcosV * theData.ZZ),
            D3UUV = new gp_Vec(RsinV * theUCtx.dirUX, RsinV * theUCtx.dirUY, RsinV * theUCtx.dirUZ),
            D3UVV = new gp_Vec(-RcosV * theUCtx.dDirUX, -RcosV * theUCtx.dDirUY, -RcosV * theUCtx.dDirUZ),
        };
    }

    private static gp_Vec computeDN(in Data theData, in UContext theUCtx, double theV, int theNU, int theNV)
    {
        double cosV = Math.Cos(theV);
        double sinV = Math.Sin(theV);

        // Cyclic phases for U and V (period 4)
        int aPhaseU = theNU % 4;
        int aPhaseV = theNV % 4;

        // U-phase coefficients: d^n/du^n of cosU and sinU
        // n=0: cosU, sinU -> DirU
        // n=1: -sinU, cosU -> dDirU
        // n=2: -cosU, -sinU -> -DirU
        // n=3: sinU, -cosU -> -dDirU
        double dirX, dirY, dirZ;
        switch (aPhaseU)
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

        // V-phase coefficients for XY term (cosV) and Z term (sinV)
        double aCoeffVXY, aCoeffVZ;
        switch (aPhaseV)
        {
            case 0:
                aCoeffVXY = cosV;
                aCoeffVZ = sinV;
                break;
            case 1:
                aCoeffVXY = -sinV;
                aCoeffVZ = cosV;
                break;
            case 2:
                aCoeffVXY = -cosV;
                aCoeffVZ = -sinV;
                break;
            case 3:
                aCoeffVXY = sinV;
                aCoeffVZ = -cosV;
                break;
            default:
                aCoeffVXY = 0.0;
                aCoeffVZ = 0.0;
                break;
        }

        // Z contribution only if NU == 0 (no U derivative on Z term)
        double aZTermX = 0.0, aZTermY = 0.0, aZTermZ = 0.0;
        if (theNU == 0)
        {
            aZTermX = aCoeffVZ * theData.ZX;
            aZTermY = aCoeffVZ * theData.ZY;
            aZTermZ = aCoeffVZ * theData.ZZ;
        }

        return new gp_Vec(theData.Radius * (aCoeffVXY * dirX + aZTermX),
                          theData.Radius * (aCoeffVXY * dirY + aZTermY),
                          theData.Radius * (aCoeffVXY * dirZ + aZTermZ));
    }

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty if the geometry is null or a list is empty.</summary>
    public NCollection_Array2<gp_Pnt> EvaluateGrid(NCollection_Array1<double> theUParams,
                                                   NCollection_Array1<double> theVParams)
    {
        if (myGeom is null || theUParams.IsEmpty() || theVParams.IsEmpty())
        {
            return new NCollection_Array2<gp_Pnt>();
        }

        Data aData = extractData();
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        NCollection_Array2<gp_Pnt> aResult = new(1, aNbU, 1, aNbV);

        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double u = theUParams.Value(theUParams.Lower() + iU - 1);
            UContext aUCtx = computeUContext(aData, u);

            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double v = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult.SetValue(iU, iV, computeD0(aData, aUCtx, v));
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
            double u = theUParams.Value(theUParams.Lower() + iU - 1);
            UContext aUCtx = computeUContext(aData, u);

            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double v = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult.SetValue(iU, iV, computeD1(aData, aUCtx, v));
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
            double u = theUParams.Value(theUParams.Lower() + iU - 1);
            UContext aUCtx = computeUContext(aData, u);

            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double v = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult.SetValue(iU, iV, computeD2(aData, aUCtx, v));
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
            double u = theUParams.Value(theUParams.Lower() + iU - 1);
            UContext aUCtx = computeUContext(aData, u);

            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double v = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult.SetValue(iU, iV, computeD3(aData, aUCtx, v));
            }
        }
        return aResult;
    }

    /// <summary>Evaluates the partial derivative of orders (NU, NV) on the grid; empty unless NU, NV &gt;= 0 and NU + NV &gt;= 1.</summary>
    public NCollection_Array2<gp_Vec> EvaluateGridDN(NCollection_Array1<double> theUParams,
                                                     NCollection_Array1<double> theVParams,
                                                     int theNU,
                                                     int theNV)
    {
        if (myGeom is null || theUParams.IsEmpty() || theVParams.IsEmpty() || theNU < 0 || theNV < 0
            || (theNU + theNV) < 1)
        {
            return new NCollection_Array2<gp_Vec>();
        }

        Data aData = extractData();
        int aNbU = theUParams.Length();
        int aNbV = theVParams.Length();

        NCollection_Array2<gp_Vec> aResult = new(1, aNbU, 1, aNbV);

        for (int iU = 1; iU <= aNbU; ++iU)
        {
            double u = theUParams.Value(theUParams.Lower() + iU - 1);
            UContext aUCtx = computeUContext(aData, u);

            for (int iV = 1; iV <= aNbV; ++iV)
            {
                double v = theVParams.Value(theVParams.Lower() + iV - 1);
                aResult.SetValue(iU, iV, computeDN(aData, aUCtx, v, theNU, theNV));
            }
        }
        return aResult;
    }
}
