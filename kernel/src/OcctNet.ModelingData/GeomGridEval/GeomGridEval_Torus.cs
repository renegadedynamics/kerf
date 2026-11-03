// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Torus.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Torus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Efficient batch evaluator for toroidal surface grid points:
/// P(u, v) = Center + (R + r * cos(v)) * (cos(u) * XDir + sin(u) * YDir) + r * sin(v) * ZDir. The trigonometric
/// values of U are computed once per U parameter.
/// </summary>
public sealed class GeomGridEval_Torus : GeomGridEval.ISurfaceEvaluator
{
    private struct Data
    {
        public double CX, CY, CZ;  // Center coordinates
        public double XX, XY, XZ;  // XDir coordinates
        public double YX, YY, YZ;  // YDir coordinates
        public double ZX, ZY, ZZ;  // ZDir coordinates
        public double MajorRadius; // Major radius
        public double MinorRadius; // Minor radius
    }

    private struct UContext
    {
        public double cosU, sinU;
        public double dirUX, dirUY, dirUZ;    // DirU = cosU*XDir + sinU*YDir
        public double dDirUX, dDirUY, dDirUZ; // DerivDirU = -sinU*XDir + cosU*YDir
    }

    private readonly Geom_ToroidalSurface? myGeom;

    /// <summary>Constructor with the torus to evaluate (null gives empty results).</summary>
    public GeomGridEval_Torus(Geom_ToroidalSurface? theTorus)
    {
        myGeom = theTorus;
    }

    /// <summary>Returns the geometry.</summary>
    public Geom_ToroidalSurface? Geometry() => myGeom;

    private Data extractData()
    {
        Torus aTorus = myGeom!.Torus();
        Point3D aCenter = aTorus.Location();
        Direction3D aXDir = aTorus.Position().XDirection();
        Direction3D aYDir = aTorus.Position().YDirection();
        Direction3D aZDir = aTorus.Position().Direction();
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
            MajorRadius = aTorus.MajorRadius(),
            MinorRadius = aTorus.MinorRadius(),
        };
    }

    private static UContext computeUContext(in Data theData, double theU)
    {
        double cosU = Math.Cos(theU);
        double sinU = Math.Sin(theU);
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
        double cosV = Math.Cos(theV);
        double sinV = Math.Sin(theV);

        // K = MajorRadius + MinorRadius * cos(v)
        double K = theData.MajorRadius + theData.MinorRadius * cosV;

        // P = Center + K * DirU + MinorRadius * sinV * ZDir
        return new Point3D(theData.CX + K * theUCtx.dirUX + theData.MinorRadius * sinV * theData.ZX,
                          theData.CY + K * theUCtx.dirUY + theData.MinorRadius * sinV * theData.ZY,
                          theData.CZ + K * theUCtx.dirUZ + theData.MinorRadius * sinV * theData.ZZ);
    }

    private static Geom_Surface.ResD1 computeD1(in Data theData, in UContext theUCtx, double theV)
    {
        double cosV = Math.Cos(theV);
        double sinV = Math.Sin(theV);

        double K = theData.MajorRadius + theData.MinorRadius * cosV;

        // D1U = K * DerivDirU
        double dU1 = K * theUCtx.dDirUX;
        double dU2 = K * theUCtx.dDirUY;
        double dU3 = K * theUCtx.dDirUZ;

        // D1V = -r * sinV * DirU + r * cosV * ZDir
        double dV1 = -theData.MinorRadius * sinV * theUCtx.dirUX + theData.MinorRadius * cosV * theData.ZX;
        double dV2 = -theData.MinorRadius * sinV * theUCtx.dirUY + theData.MinorRadius * cosV * theData.ZY;
        double dV3 = -theData.MinorRadius * sinV * theUCtx.dirUZ + theData.MinorRadius * cosV * theData.ZZ;

        return new Geom_Surface.ResD1
        {
            Point = new Point3D(theData.CX + K * theUCtx.dirUX + theData.MinorRadius * sinV * theData.ZX,
                               theData.CY + K * theUCtx.dirUY + theData.MinorRadius * sinV * theData.ZY,
                               theData.CZ + K * theUCtx.dirUZ + theData.MinorRadius * sinV * theData.ZZ),
            D1U = new Vector3D(dU1, dU2, dU3),
            D1V = new Vector3D(dV1, dV2, dV3),
        };
    }

    private static Geom_Surface.ResD2 computeD2(in Data theData, in UContext theUCtx, double theV)
    {
        double cosV = Math.Cos(theV);
        double sinV = Math.Sin(theV);

        double K = theData.MajorRadius + theData.MinorRadius * cosV;

        // D1U = K * DerivDirU
        double dU1 = K * theUCtx.dDirUX;
        double dU2 = K * theUCtx.dDirUY;
        double dU3 = K * theUCtx.dDirUZ;

        // D1V = -r * sinV * DirU + r * cosV * ZDir
        double dV1 = -theData.MinorRadius * sinV * theUCtx.dirUX + theData.MinorRadius * cosV * theData.ZX;
        double dV2 = -theData.MinorRadius * sinV * theUCtx.dirUY + theData.MinorRadius * cosV * theData.ZY;
        double dV3 = -theData.MinorRadius * sinV * theUCtx.dirUZ + theData.MinorRadius * cosV * theData.ZZ;

        // D2U = K * Deriv2DirU = -K * DirU
        double d2U1 = -K * theUCtx.dirUX;
        double d2U2 = -K * theUCtx.dirUY;
        double d2U3 = -K * theUCtx.dirUZ;

        // D2V = -r * cosV * DirU - r * sinV * ZDir
        double d2V1 = -theData.MinorRadius * cosV * theUCtx.dirUX - theData.MinorRadius * sinV * theData.ZX;
        double d2V2 = -theData.MinorRadius * cosV * theUCtx.dirUY - theData.MinorRadius * sinV * theData.ZY;
        double d2V3 = -theData.MinorRadius * cosV * theUCtx.dirUZ - theData.MinorRadius * sinV * theData.ZZ;

        // D2UV = -r * sinV * DerivDirU
        double d2UV1 = -theData.MinorRadius * sinV * theUCtx.dDirUX;
        double d2UV2 = -theData.MinorRadius * sinV * theUCtx.dDirUY;
        double d2UV3 = -theData.MinorRadius * sinV * theUCtx.dDirUZ;

        return new Geom_Surface.ResD2
        {
            Point = new Point3D(theData.CX + K * theUCtx.dirUX + theData.MinorRadius * sinV * theData.ZX,
                               theData.CY + K * theUCtx.dirUY + theData.MinorRadius * sinV * theData.ZY,
                               theData.CZ + K * theUCtx.dirUZ + theData.MinorRadius * sinV * theData.ZZ),
            D1U = new Vector3D(dU1, dU2, dU3),
            D1V = new Vector3D(dV1, dV2, dV3),
            D2U = new Vector3D(d2U1, d2U2, d2U3),
            D2V = new Vector3D(d2V1, d2V2, d2V3),
            D2UV = new Vector3D(d2UV1, d2UV2, d2UV3),
        };
    }

    private static Geom_Surface.ResD3 computeD3(in Data theData, in UContext theUCtx, double theV)
    {
        double cosV = Math.Cos(theV);
        double sinV = Math.Sin(theV);

        // R1 = r * cosV, R2 = r * sinV
        double R1 = theData.MinorRadius * cosV;
        double R2 = theData.MinorRadius * sinV;

        // K = R + r * cosV
        double K = theData.MajorRadius + R1;

        // A3 = R2 * cosU, A4 = R2 * sinU (for Som3)
        double A3 = R2 * theUCtx.cosU;
        double A4 = R2 * theUCtx.sinU;

        // A5 = R1 * cosU, A6 = R1 * sinU (for Vvv, Vuvv)
        double A5 = R1 * theUCtx.cosU;
        double A6 = R1 * theUCtx.sinU;

        // Som1 = K * DirU (for P and D2U)
        double Som1X = K * theUCtx.dirUX;
        double Som1Y = K * theUCtx.dirUY;
        double Som1Z = K * theUCtx.dirUZ;

        // Som3 = R2 * (cosU*X + sinU*Y) = A3*X + A4*Y
        double Som3X = A3 * theData.XX + A4 * theData.YX;
        double Som3Y = A3 * theData.XY + A4 * theData.YY;
        double Som3Z = A3 * theData.XZ + A4 * theData.YZ;

        // D1U = K * DerivDirU
        double dU1 = K * theUCtx.dDirUX;
        double dU2 = K * theUCtx.dDirUY;
        double dU3 = K * theUCtx.dDirUZ;

        // D1V = -Som3 + R1*Z = -r*sinV*DirU + r*cosV*Z
        double dV1 = -Som3X + R1 * theData.ZX;
        double dV2 = -Som3Y + R1 * theData.ZY;
        double dV3 = -Som3Z + R1 * theData.ZZ;

        // D2U = -Som1 = -K * DirU
        double d2U1 = -Som1X;
        double d2U2 = -Som1Y;
        double d2U3 = -Som1Z;

        // D2V = -R1*DirU - R2*Z = -(A5*X + A6*Y) - R2*Z
        double d2V1 = -A5 * theData.XX - A6 * theData.YX - R2 * theData.ZX;
        double d2V2 = -A5 * theData.XY - A6 * theData.YY - R2 * theData.ZY;
        double d2V3 = -A5 * theData.XZ - A6 * theData.YZ - R2 * theData.ZZ;

        // D2UV = r*sinV*(sinU*X - cosU*Y) = A4*X - A3*Y
        double d2UV1 = A4 * theData.XX - A3 * theData.YX;
        double d2UV2 = A4 * theData.XY - A3 * theData.YY;
        double d2UV3 = A4 * theData.XZ - A3 * theData.YZ;

        // D3U = -D1U (Vuuu = Dif1 = -DerivDirU*K)
        double d3U1 = -dU1;
        double d3U2 = -dU2;
        double d3U3 = -dU3;

        // D3V = -D1V (Vvvv = Som3 - R1*Z)
        double d3V1 = Som3X - R1 * theData.ZX;
        double d3V2 = Som3Y - R1 * theData.ZY;
        double d3V3 = Som3Z - R1 * theData.ZZ;

        // D3UUV = Som3 = r*sinV*(cosU*X + sinU*Y)
        double d3UUV1 = Som3X;
        double d3UUV2 = Som3Y;
        double d3UUV3 = Som3Z;

        // D3UVV = r*cosV*(sinU*X - cosU*Y) = A6*X - A5*Y
        double d3UVV1 = A6 * theData.XX - A5 * theData.YX;
        double d3UVV2 = A6 * theData.XY - A5 * theData.YY;
        double d3UVV3 = A6 * theData.XZ - A5 * theData.YZ;

        return new Geom_Surface.ResD3
        {
            Point = new Point3D(theData.CX + Som1X + R2 * theData.ZX,
                               theData.CY + Som1Y + R2 * theData.ZY,
                               theData.CZ + Som1Z + R2 * theData.ZZ),
            D1U = new Vector3D(dU1, dU2, dU3),
            D1V = new Vector3D(dV1, dV2, dV3),
            D2U = new Vector3D(d2U1, d2U2, d2U3),
            D2V = new Vector3D(d2V1, d2V2, d2V3),
            D2UV = new Vector3D(d2UV1, d2UV2, d2UV3),
            D3U = new Vector3D(d3U1, d3U2, d3U3),
            D3V = new Vector3D(d3V1, d3V2, d3V3),
            D3UUV = new Vector3D(d3UUV1, d3UUV2, d3UUV3),
            D3UVV = new Vector3D(d3UVV1, d3UVV2, d3UVV3),
        };
    }

    private static Vector3D computeDN(in Data theData, in UContext theUCtx, double theV, int theNU, int theNV)
    {
        // For torus P(u,v) = C + K(v)*DirU(u) + r*sin(v)*Z
        // where K(v) = R + r*cos(v), DirU(u) = cos(u)*X + sin(u)*Y
        //
        // Both U and V derivatives are cyclic with period 4:
        //
        // d^n(DirU)/du^n cycles: cos->-sin->-cos->sin (phase = n % 4)
        // d^n(cos(v))/dv^n cycles: cos->-sin->-cos->sin (phase = n % 4)
        // d^n(sin(v))/dv^n cycles: sin->cos->-sin->-cos (phase = n % 4)
        //
        // The partial derivative D_{nu,nv} involves:
        // - d^nv(K)/dv^nv = r * d^nv(cos(v))/dv^nv
        // - d^nu(DirU)/du^nu
        // - d^nv(r*sin(v))/dv^nv = r * d^nv(sin(v))/dv^nv (only contributes when nu == 0)

        double cosV = Math.Cos(theV);
        double sinV = Math.Sin(theV);

        // Phase for U derivatives (period 4)
        int aPhaseU = theNU % 4;
        // Phase for V derivatives (period 4)
        int aPhaseV = theNV % 4;

        // Compute d^nu(DirU)/du^nu based on phase (using pre-computed cosU, sinU from UContext)
        double dirDnX, dirDnY, dirDnZ;
        switch (aPhaseU)
        {
            case 0: // cos(u)*X + sin(u)*Y = DirU
                dirDnX = theUCtx.dirUX;
                dirDnY = theUCtx.dirUY;
                dirDnZ = theUCtx.dirUZ;
                break;
            case 1: // -sin(u)*X + cos(u)*Y = DerivDirU
                dirDnX = theUCtx.dDirUX;
                dirDnY = theUCtx.dDirUY;
                dirDnZ = theUCtx.dDirUZ;
                break;
            case 2: // -cos(u)*X - sin(u)*Y = -DirU
                dirDnX = -theUCtx.dirUX;
                dirDnY = -theUCtx.dirUY;
                dirDnZ = -theUCtx.dirUZ;
                break;
            default: // case 3: sin(u)*X - cos(u)*Y = -DerivDirU
                dirDnX = -theUCtx.dDirUX;
                dirDnY = -theUCtx.dDirUY;
                dirDnZ = -theUCtx.dDirUZ;
                break;
        }

        // Compute d^nv(cos(v))/dv^nv based on phase
        double dCosV;
        switch (aPhaseV)
        {
            case 0:
                dCosV = cosV;
                break;
            case 1:
                dCosV = -sinV;
                break;
            case 2:
                dCosV = -cosV;
                break;
            default: // case 3
                dCosV = sinV;
                break;
        }

        // Compute d^nv(sin(v))/dv^nv based on phase
        double dSinV;
        switch (aPhaseV)
        {
            case 0:
                dSinV = sinV;
                break;
            case 1:
                dSinV = cosV;
                break;
            case 2:
                dSinV = -sinV;
                break;
            default: // case 3
                dSinV = -cosV;
                break;
        }

        // For the general derivative D_{nu,nv}:
        // - If nv == 0: D = K(v) * d^nu(DirU) where K = R + r*cos(v)
        // - If nv >= 1 and nu == 0: D = d^nv(K)/dv^nv * DirU + r * d^nv(sin(v))/dv^nv * Z
        // - If nv >= 1 and nu >= 1: D = r * d^nv(cos(v))/dv^nv * d^nu(DirU)/du^nu

        double resX, resY, resZ;

        if (theNV == 0)
        {
            // Pure U derivative: K(v) * d^nu(DirU)/du^nu
            double K = theData.MajorRadius + theData.MinorRadius * cosV;
            resX = K * dirDnX;
            resY = K * dirDnY;
            resZ = K * dirDnZ;
        }
        else if (theNU == 0)
        {
            // Pure V derivative with nu=0: d^nv(K)/dv^nv * DirU + r * d^nv(sin(v))/dv^nv * Z
            // where d^nv(K)/dv^nv = r * d^nv(cos(v))/dv^nv
            double dK = theData.MinorRadius * dCosV;
            resX = dK * dirDnX + theData.MinorRadius * dSinV * theData.ZX;
            resY = dK * dirDnY + theData.MinorRadius * dSinV * theData.ZY;
            resZ = dK * dirDnZ + theData.MinorRadius * dSinV * theData.ZZ;
        }
        else
        {
            // Mixed derivative with both nu >= 1 and nv >= 1:
            // D = r * d^nv(cos(v))/dv^nv * d^nu(DirU)/du^nu
            double coeff = theData.MinorRadius * dCosV;
            resX = coeff * dirDnX;
            resY = coeff * dirDnY;
            resZ = coeff * dirDnZ;
        }

        return new Vector3D(resX, resY, resZ);
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
