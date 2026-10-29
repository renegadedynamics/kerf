// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Cone.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Cone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Efficient batch evaluator for conical surface grid points:
/// P(u, v) = Center + (RefRadius + v * sin(Ang)) * (cos(u) * XDir + sin(u) * YDir) + v * cos(Ang) * ZDir. The
/// trigonometric values of U are computed once per U parameter.
/// </summary>
public sealed class GeomGridEval_Cone : GeomGridEval.ISurfaceEvaluator
{
    private struct Data
    {
        public double CX, CY, CZ; // Center coordinates
        public double XX, XY, XZ; // XDir coordinates
        public double YX, YY, YZ; // YDir coordinates
        public double ZX, ZY, ZZ; // ZDir coordinates
        public double RefRadius;  // Reference radius
        public double SinAng;     // sin(SemiAngle)
        public double CosAng;     // cos(SemiAngle)
    }

    private struct UContext
    {
        public double cosU, sinU;
        public double dirUX, dirUY, dirUZ;    // DirU = cosU*XDir + sinU*YDir
        public double dDirUX, dDirUY, dDirUZ; // DerivDirU = -sinU*XDir + cosU*YDir
    }

    private readonly Geom_ConicalSurface? myGeom;

    /// <summary>Constructor with the cone to evaluate (null gives empty results).</summary>
    public GeomGridEval_Cone(Geom_ConicalSurface? theCone)
    {
        myGeom = theCone;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_ConicalSurface? Geometry() => myGeom;

    private Data extractData()
    {
        gp_Cone aCone = myGeom!.Cone();
        gp_Pnt aCenter = aCone.Location();
        gp_Dir aXDir = aCone.Position().XDirection();
        gp_Dir aYDir = aCone.Position().YDirection();
        gp_Dir aZDir = aCone.Position().Direction();
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
            RefRadius = aCone.RefRadius(),
            SinAng = Math.Sin(aCone.SemiAngle()),
            CosAng = Math.Cos(aCone.SemiAngle()),
        };
    }

    private static UContext computeUContext(in Data theData, double theU)
    {
        double cosU = Math.Cos(theU);
        double sinU = Math.Sin(theU);
        // DirU = cosU*XDir + sinU*YDir (radial direction at angle U)
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
        // K(v) = RefRadius + v * sin(Ang)
        double K = theData.RefRadius + theV * theData.SinAng;
        // ZOffset(v) = v * cos(Ang)
        double ZOff = theV * theData.CosAng;

        // P = Center + K(v) * DirU + ZOffset(v) * ZDir
        return new gp_Pnt(theData.CX + K * theUCtx.dirUX + ZOff * theData.ZX,
                          theData.CY + K * theUCtx.dirUY + ZOff * theData.ZY,
                          theData.CZ + K * theUCtx.dirUZ + ZOff * theData.ZZ);
    }

    private static Geom_Surface.ResD1 computeD1(in Data theData, in UContext theUCtx, double theV)
    {
        double K = theData.RefRadius + theV * theData.SinAng;
        double ZOff = theV * theData.CosAng;

        // D1U = K(v) * dDirU
        // D1V = sin(Ang)*DirU + cos(Ang)*ZDir
        return new Geom_Surface.ResD1
        {
            Point = new gp_Pnt(theData.CX + K * theUCtx.dirUX + ZOff * theData.ZX,
                               theData.CY + K * theUCtx.dirUY + ZOff * theData.ZY,
                               theData.CZ + K * theUCtx.dirUZ + ZOff * theData.ZZ),
            D1U = new gp_Vec(K * theUCtx.dDirUX, K * theUCtx.dDirUY, K * theUCtx.dDirUZ),
            D1V = new gp_Vec(theData.SinAng * theUCtx.dirUX + theData.CosAng * theData.ZX,
                             theData.SinAng * theUCtx.dirUY + theData.CosAng * theData.ZY,
                             theData.SinAng * theUCtx.dirUZ + theData.CosAng * theData.ZZ),
        };
    }

    private static Geom_Surface.ResD2 computeD2(in Data theData, in UContext theUCtx, double theV)
    {
        double K = theData.RefRadius + theV * theData.SinAng;
        double ZOff = theV * theData.CosAng;

        // D2U = K * (-DirU) = -K * DirU
        // D2V = 0
        // D2UV = sin(Ang) * dDirU
        gp_Vec aZero = new(0.0, 0.0, 0.0);
        return new Geom_Surface.ResD2
        {
            Point = new gp_Pnt(theData.CX + K * theUCtx.dirUX + ZOff * theData.ZX,
                               theData.CY + K * theUCtx.dirUY + ZOff * theData.ZY,
                               theData.CZ + K * theUCtx.dirUZ + ZOff * theData.ZZ),
            D1U = new gp_Vec(K * theUCtx.dDirUX, K * theUCtx.dDirUY, K * theUCtx.dDirUZ),
            D1V = new gp_Vec(theData.SinAng * theUCtx.dirUX + theData.CosAng * theData.ZX,
                             theData.SinAng * theUCtx.dirUY + theData.CosAng * theData.ZY,
                             theData.SinAng * theUCtx.dirUZ + theData.CosAng * theData.ZZ),
            D2U = new gp_Vec(-K * theUCtx.dirUX, -K * theUCtx.dirUY, -K * theUCtx.dirUZ),
            D2V = aZero,
            D2UV = new gp_Vec(theData.SinAng * theUCtx.dDirUX,
                              theData.SinAng * theUCtx.dDirUY,
                              theData.SinAng * theUCtx.dDirUZ),
        };
    }

    private static Geom_Surface.ResD3 computeD3(in Data theData, in UContext theUCtx, double theV)
    {
        double K = theData.RefRadius + theV * theData.SinAng;
        double ZOff = theV * theData.CosAng;

        // D3U = -K * dDirU = -D1U
        // D3V = 0
        // D3UUV = -sin(Ang) * DirU
        // D3UVV = 0
        gp_Vec aZero = new(0.0, 0.0, 0.0);
        return new Geom_Surface.ResD3
        {
            Point = new gp_Pnt(theData.CX + K * theUCtx.dirUX + ZOff * theData.ZX,
                               theData.CY + K * theUCtx.dirUY + ZOff * theData.ZY,
                               theData.CZ + K * theUCtx.dirUZ + ZOff * theData.ZZ),
            D1U = new gp_Vec(K * theUCtx.dDirUX, K * theUCtx.dDirUY, K * theUCtx.dDirUZ),
            D1V = new gp_Vec(theData.SinAng * theUCtx.dirUX + theData.CosAng * theData.ZX,
                             theData.SinAng * theUCtx.dirUY + theData.CosAng * theData.ZY,
                             theData.SinAng * theUCtx.dirUZ + theData.CosAng * theData.ZZ),
            D2U = new gp_Vec(-K * theUCtx.dirUX, -K * theUCtx.dirUY, -K * theUCtx.dirUZ),
            D2V = aZero,
            D2UV = new gp_Vec(theData.SinAng * theUCtx.dDirUX,
                              theData.SinAng * theUCtx.dDirUY,
                              theData.SinAng * theUCtx.dDirUZ),
            D3U = new gp_Vec(-K * theUCtx.dDirUX, -K * theUCtx.dDirUY, -K * theUCtx.dDirUZ),
            D3V = aZero,
            D3UUV = new gp_Vec(-theData.SinAng * theUCtx.dirUX,
                               -theData.SinAng * theUCtx.dirUY,
                               -theData.SinAng * theUCtx.dirUZ),
            D3UVV = aZero,
        };
    }

    private static gp_Vec computeDN(in Data theData, in UContext theUCtx, double theV, int theNU, int theNV)
    {
        // For cone P(u,v) = C + K(v) * DirU(u) + v*cos(Ang)*Z
        // where K(v) = RefRadius + v*sin(Ang), DirU(u) = cos(u)*X + sin(u)*Y
        //
        // V is linear, so:
        //   - All derivatives with NV >= 2 are zero
        //   - D_{0,1} = sin(Ang)*DirU + cos(Ang)*Z
        //   - D_{nu,1} = sin(Ang) * d^nu(DirU)/du^nu for nu >= 1
        //   - D_{nu,0} = K(v) * d^nu(DirU)/du^nu

        if (theNV > 1)
        {
            return new gp_Vec(0.0, 0.0, 0.0);
        }

        // DirU derivatives cycle with period 4:
        //   phase 0: DirU
        //   phase 1: dDirU
        //   phase 2: -DirU
        //   phase 3: -dDirU
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

        if (theNV == 1)
        {
            if (theNU == 0)
            {
                // D_{0,1} = sin(Ang)*DirU + cos(Ang)*Z
                return new gp_Vec(theData.SinAng * theUCtx.dirUX + theData.CosAng * theData.ZX,
                                  theData.SinAng * theUCtx.dirUY + theData.CosAng * theData.ZY,
                                  theData.SinAng * theUCtx.dirUZ + theData.CosAng * theData.ZZ);
            }
            // D_{nu,1} = sin(Ang) * d^nu(DirU)/du^nu for nu >= 1
            return new gp_Vec(theData.SinAng * dirX, theData.SinAng * dirY, theData.SinAng * dirZ);
        }

        // Pure U derivative (theNV == 0, theNU >= 1)
        // D_{nu,0} = K(v) * d^nu(DirU)/du^nu
        double K = theData.RefRadius + theV * theData.SinAng;
        return new gp_Vec(K * dirX, K * dirY, K * dirZ);
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
