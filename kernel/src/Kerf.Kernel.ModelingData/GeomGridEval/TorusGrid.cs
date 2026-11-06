// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Torus.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Torus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for toroidal surface grid points:
/// P(u, v) = Center + (R + r * cos(v)) * (cos(u) * XDir + sin(u) * YDir) + r * sin(v) * ZDir. The trigonometric
/// values of U are computed once per U parameter.
/// </summary>
internal sealed class TorusGrid : GeomGridEvalGrid.ISurfaceEvaluator
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

    private readonly ToroidalSurface? _geom;

    /// <summary>Constructor with the torus to evaluate (null gives empty results).</summary>
    public TorusGrid(ToroidalSurface? torus)
    {
        _geom = torus;
    }

    /// <summary>Returns the geometry.</summary>
    public ToroidalSurface? Geometry() => _geom;

    private Data ExtractData()
    {
        Torus3D torus = _geom!.Torus;
        Point3D center = torus.Location;
        Direction3D xDir = torus.Position.XDirection;
        Direction3D yDir = torus.Position.YDirection;
        Direction3D zDir = torus.Position.Direction;
        return new Data
        {
            CX = center.X,
            CY = center.Y,
            CZ = center.Z,
            XX = xDir.X,
            XY = xDir.Y,
            XZ = xDir.Z,
            YX = yDir.X,
            YY = yDir.Y,
            YZ = yDir.Z,
            ZX = zDir.X,
            ZY = zDir.Y,
            ZZ = zDir.Z,
            MajorRadius = torus.MajorRadius,
            MinorRadius = torus.MinorRadius,
        };
    }

    private static UContext ComputeUContext(in Data data, double u)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        return new UContext
        {
            cosU = cosU,
            sinU = sinU,
            dirUX = cosU * data.XX + sinU * data.YX,
            dirUY = cosU * data.XY + sinU * data.YY,
            dirUZ = cosU * data.XZ + sinU * data.YZ,
            dDirUX = -sinU * data.XX + cosU * data.YX,
            dDirUY = -sinU * data.XY + cosU * data.YY,
            dDirUZ = -sinU * data.XZ + cosU * data.YZ,
        };
    }

    private static Point3D ComputeD0(in Data data, in UContext uCtx, double v)
    {
        double cosV = Math.Cos(v);
        double sinV = Math.Sin(v);

        // K = MajorRadius + MinorRadius * cos(v)
        double k = data.MajorRadius + data.MinorRadius * cosV;

        // P = Center + K * DirU + MinorRadius * sinV * ZDir
        return new Point3D(data.CX + k * uCtx.dirUX + data.MinorRadius * sinV * data.ZX,
                          data.CY + k * uCtx.dirUY + data.MinorRadius * sinV * data.ZY,
                          data.CZ + k * uCtx.dirUZ + data.MinorRadius * sinV * data.ZZ);
    }

    private static Surface.ResD1 ComputeD1(in Data data, in UContext uCtx, double v)
    {
        double cosV = Math.Cos(v);
        double sinV = Math.Sin(v);

        double k = data.MajorRadius + data.MinorRadius * cosV;

        // D1U = K * DerivDirU
        double dU1 = k * uCtx.dDirUX;
        double dU2 = k * uCtx.dDirUY;
        double dU3 = k * uCtx.dDirUZ;

        // D1V = -r * sinV * DirU + r * cosV * ZDir
        double dV1 = -data.MinorRadius * sinV * uCtx.dirUX + data.MinorRadius * cosV * data.ZX;
        double dV2 = -data.MinorRadius * sinV * uCtx.dirUY + data.MinorRadius * cosV * data.ZY;
        double dV3 = -data.MinorRadius * sinV * uCtx.dirUZ + data.MinorRadius * cosV * data.ZZ;

        return new Surface.ResD1
        {
            Point = new Point3D(data.CX + k * uCtx.dirUX + data.MinorRadius * sinV * data.ZX,
                               data.CY + k * uCtx.dirUY + data.MinorRadius * sinV * data.ZY,
                               data.CZ + k * uCtx.dirUZ + data.MinorRadius * sinV * data.ZZ),
            D1U = new Vector3D(dU1, dU2, dU3),
            D1V = new Vector3D(dV1, dV2, dV3),
        };
    }

    private static Surface.ResD2 ComputeD2(in Data data, in UContext uCtx, double v)
    {
        double cosV = Math.Cos(v);
        double sinV = Math.Sin(v);

        double k = data.MajorRadius + data.MinorRadius * cosV;

        // D1U = K * DerivDirU
        double dU1 = k * uCtx.dDirUX;
        double dU2 = k * uCtx.dDirUY;
        double dU3 = k * uCtx.dDirUZ;

        // D1V = -r * sinV * DirU + r * cosV * ZDir
        double dV1 = -data.MinorRadius * sinV * uCtx.dirUX + data.MinorRadius * cosV * data.ZX;
        double dV2 = -data.MinorRadius * sinV * uCtx.dirUY + data.MinorRadius * cosV * data.ZY;
        double dV3 = -data.MinorRadius * sinV * uCtx.dirUZ + data.MinorRadius * cosV * data.ZZ;

        // D2U = K * Deriv2DirU = -K * DirU
        double d2U1 = -k * uCtx.dirUX;
        double d2U2 = -k * uCtx.dirUY;
        double d2U3 = -k * uCtx.dirUZ;

        // D2V = -r * cosV * DirU - r * sinV * ZDir
        double d2V1 = -data.MinorRadius * cosV * uCtx.dirUX - data.MinorRadius * sinV * data.ZX;
        double d2V2 = -data.MinorRadius * cosV * uCtx.dirUY - data.MinorRadius * sinV * data.ZY;
        double d2V3 = -data.MinorRadius * cosV * uCtx.dirUZ - data.MinorRadius * sinV * data.ZZ;

        // D2UV = -r * sinV * DerivDirU
        double d2UV1 = -data.MinorRadius * sinV * uCtx.dDirUX;
        double d2UV2 = -data.MinorRadius * sinV * uCtx.dDirUY;
        double d2UV3 = -data.MinorRadius * sinV * uCtx.dDirUZ;

        return new Surface.ResD2
        {
            Point = new Point3D(data.CX + k * uCtx.dirUX + data.MinorRadius * sinV * data.ZX,
                               data.CY + k * uCtx.dirUY + data.MinorRadius * sinV * data.ZY,
                               data.CZ + k * uCtx.dirUZ + data.MinorRadius * sinV * data.ZZ),
            D1U = new Vector3D(dU1, dU2, dU3),
            D1V = new Vector3D(dV1, dV2, dV3),
            D2U = new Vector3D(d2U1, d2U2, d2U3),
            D2V = new Vector3D(d2V1, d2V2, d2V3),
            D2UV = new Vector3D(d2UV1, d2UV2, d2UV3),
        };
    }

    private static Surface.ResD3 ComputeD3(in Data data, in UContext uCtx, double v)
    {
        double cosV = Math.Cos(v);
        double sinV = Math.Sin(v);

        // R1 = r * cosV, R2 = r * sinV
        double r1 = data.MinorRadius * cosV;
        double r2 = data.MinorRadius * sinV;

        // K = R + r * cosV
        double k = data.MajorRadius + r1;

        // A3 = R2 * cosU, A4 = R2 * sinU (for Som3)
        double a3 = r2 * uCtx.cosU;
        double a4 = r2 * uCtx.sinU;

        // A5 = R1 * cosU, A6 = R1 * sinU (for Vvv, Vuvv)
        double a5 = r1 * uCtx.cosU;
        double a6 = r1 * uCtx.sinU;

        // Som1 = K * DirU (for P and D2U)
        double som1X = k * uCtx.dirUX;
        double som1Y = k * uCtx.dirUY;
        double som1Z = k * uCtx.dirUZ;

        // Som3 = R2 * (cosU*X + sinU*Y) = A3*X + A4*Y
        double som3X = a3 * data.XX + a4 * data.YX;
        double som3Y = a3 * data.XY + a4 * data.YY;
        double som3Z = a3 * data.XZ + a4 * data.YZ;

        // D1U = K * DerivDirU
        double dU1 = k * uCtx.dDirUX;
        double dU2 = k * uCtx.dDirUY;
        double dU3 = k * uCtx.dDirUZ;

        // D1V = -Som3 + R1*Z = -r*sinV*DirU + r*cosV*Z
        double dV1 = -som3X + r1 * data.ZX;
        double dV2 = -som3Y + r1 * data.ZY;
        double dV3 = -som3Z + r1 * data.ZZ;

        // D2U = -Som1 = -K * DirU
        double d2U1 = -som1X;
        double d2U2 = -som1Y;
        double d2U3 = -som1Z;

        // D2V = -R1*DirU - R2*Z = -(A5*X + A6*Y) - R2*Z
        double d2V1 = -a5 * data.XX - a6 * data.YX - r2 * data.ZX;
        double d2V2 = -a5 * data.XY - a6 * data.YY - r2 * data.ZY;
        double d2V3 = -a5 * data.XZ - a6 * data.YZ - r2 * data.ZZ;

        // D2UV = r*sinV*(sinU*X - cosU*Y) = A4*X - A3*Y
        double d2UV1 = a4 * data.XX - a3 * data.YX;
        double d2UV2 = a4 * data.XY - a3 * data.YY;
        double d2UV3 = a4 * data.XZ - a3 * data.YZ;

        // D3U = -D1U (Vuuu = Dif1 = -DerivDirU*K)
        double d3U1 = -dU1;
        double d3U2 = -dU2;
        double d3U3 = -dU3;

        // D3V = -D1V (Vvvv = Som3 - R1*Z)
        double d3V1 = som3X - r1 * data.ZX;
        double d3V2 = som3Y - r1 * data.ZY;
        double d3V3 = som3Z - r1 * data.ZZ;

        // D3UUV = Som3 = r*sinV*(cosU*X + sinU*Y)
        double d3UUV1 = som3X;
        double d3UUV2 = som3Y;
        double d3UUV3 = som3Z;

        // D3UVV = r*cosV*(sinU*X - cosU*Y) = A6*X - A5*Y
        double d3UVV1 = a6 * data.XX - a5 * data.YX;
        double d3UVV2 = a6 * data.XY - a5 * data.YY;
        double d3UVV3 = a6 * data.XZ - a5 * data.YZ;

        return new Surface.ResD3
        {
            Point = new Point3D(data.CX + som1X + r2 * data.ZX,
                               data.CY + som1Y + r2 * data.ZY,
                               data.CZ + som1Z + r2 * data.ZZ),
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

    private static Vector3D ComputeDN(in Data data, in UContext uCtx, double v, int nu, int nv)
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

        double cosV = Math.Cos(v);
        double sinV = Math.Sin(v);

        // Phase for U derivatives (period 4)
        int phaseU = nu % 4;
        // Phase for V derivatives (period 4)
        int phaseV = nv % 4;

        // Compute d^nu(DirU)/du^nu based on phase (using pre-computed cosU, sinU from UContext)
        double dirDnX, dirDnY, dirDnZ;
        switch (phaseU)
        {
            case 0: // cos(u)*X + sin(u)*Y = DirU
                dirDnX = uCtx.dirUX;
                dirDnY = uCtx.dirUY;
                dirDnZ = uCtx.dirUZ;
                break;
            case 1: // -sin(u)*X + cos(u)*Y = DerivDirU
                dirDnX = uCtx.dDirUX;
                dirDnY = uCtx.dDirUY;
                dirDnZ = uCtx.dDirUZ;
                break;
            case 2: // -cos(u)*X - sin(u)*Y = -DirU
                dirDnX = -uCtx.dirUX;
                dirDnY = -uCtx.dirUY;
                dirDnZ = -uCtx.dirUZ;
                break;
            default: // case 3: sin(u)*X - cos(u)*Y = -DerivDirU
                dirDnX = -uCtx.dDirUX;
                dirDnY = -uCtx.dDirUY;
                dirDnZ = -uCtx.dDirUZ;
                break;
        }

        // Compute d^nv(cos(v))/dv^nv based on phase
        double dCosV;
        switch (phaseV)
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
        switch (phaseV)
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

        if (nv == 0)
        {
            // Pure U derivative: K(v) * d^nu(DirU)/du^nu
            double k = data.MajorRadius + data.MinorRadius * cosV;
            resX = k * dirDnX;
            resY = k * dirDnY;
            resZ = k * dirDnZ;
        }
        else if (nu == 0)
        {
            // Pure V derivative with nu=0: d^nv(K)/dv^nv * DirU + r * d^nv(sin(v))/dv^nv * Z
            // where d^nv(K)/dv^nv = r * d^nv(cos(v))/dv^nv
            double dK = data.MinorRadius * dCosV;
            resX = dK * dirDnX + data.MinorRadius * dSinV * data.ZX;
            resY = dK * dirDnY + data.MinorRadius * dSinV * data.ZY;
            resZ = dK * dirDnZ + data.MinorRadius * dSinV * data.ZZ;
        }
        else
        {
            // Mixed derivative with both nu >= 1 and nv >= 1:
            // D = r * d^nv(cos(v))/dv^nv * d^nu(DirU)/du^nu
            double coeff = data.MinorRadius * dCosV;
            resX = coeff * dirDnX;
            resY = coeff * dirDnY;
            resZ = coeff * dirDnZ;
        }

        return new Vector3D(resX, resY, resZ);
    }

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty if the geometry is null or a list is empty.</summary>
    public NCollection_Array2<Point3D> EvaluateGrid(NCollection_Array1<double> uParams,
                                                   NCollection_Array1<double> vParams)
    {
        if (_geom is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Point3D>();
        }

        Data data = ExtractData();
        int nbU = uParams.Length();
        int nbV = vParams.Length();

        NCollection_Array2<Point3D> result = new(1, nbU, 1, nbV);

        for (int iU = 1; iU <= nbU; ++iU)
        {
            double u = uParams.Value(uParams.Lower() + iU - 1);
            UContext uCtx = ComputeUContext(data, u);

            for (int iV = 1; iV <= nbV; ++iV)
            {
                double v = vParams.Value(vParams.Lower() + iV - 1);
                result.SetValue(iU, iV, ComputeD0(data, uCtx, v));
            }
        }
        return result;
    }

    /// <summary>Evaluates all grid points with first partial derivatives.</summary>
    public NCollection_Array2<Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (_geom is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD1>();
        }

        Data data = ExtractData();
        int nbU = uParams.Length();
        int nbV = vParams.Length();

        NCollection_Array2<Surface.ResD1> result = new(1, nbU, 1, nbV);

        for (int iU = 1; iU <= nbU; ++iU)
        {
            double u = uParams.Value(uParams.Lower() + iU - 1);
            UContext uCtx = ComputeUContext(data, u);

            for (int iV = 1; iV <= nbV; ++iV)
            {
                double v = vParams.Value(vParams.Lower() + iV - 1);
                result.SetValue(iU, iV, ComputeD1(data, uCtx, v));
            }
        }
        return result;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the second order.</summary>
    public NCollection_Array2<Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (_geom is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD2>();
        }

        Data data = ExtractData();
        int nbU = uParams.Length();
        int nbV = vParams.Length();

        NCollection_Array2<Surface.ResD2> result = new(1, nbU, 1, nbV);

        for (int iU = 1; iU <= nbU; ++iU)
        {
            double u = uParams.Value(uParams.Lower() + iU - 1);
            UContext uCtx = ComputeUContext(data, u);

            for (int iV = 1; iV <= nbV; ++iV)
            {
                double v = vParams.Value(vParams.Lower() + iV - 1);
                result.SetValue(iU, iV, ComputeD2(data, uCtx, v));
            }
        }
        return result;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the third order.</summary>
    public NCollection_Array2<Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        if (_geom is null || uParams.IsEmpty() || vParams.IsEmpty())
        {
            return new NCollection_Array2<Surface.ResD3>();
        }

        Data data = ExtractData();
        int nbU = uParams.Length();
        int nbV = vParams.Length();

        NCollection_Array2<Surface.ResD3> result = new(1, nbU, 1, nbV);

        for (int iU = 1; iU <= nbU; ++iU)
        {
            double u = uParams.Value(uParams.Lower() + iU - 1);
            UContext uCtx = ComputeUContext(data, u);

            for (int iV = 1; iV <= nbV; ++iV)
            {
                double v = vParams.Value(vParams.Lower() + iV - 1);
                result.SetValue(iU, iV, ComputeD3(data, uCtx, v));
            }
        }
        return result;
    }

    /// <summary>Evaluates the partial derivative of orders (NU, NV) on the grid; empty unless NU, NV &gt;= 0 and NU + NV &gt;= 1.</summary>
    public NCollection_Array2<Vector3D> EvaluateGridDN(NCollection_Array1<double> uParams,
                                                     NCollection_Array1<double> vParams,
                                                     int nu,
                                                     int nv)
    {
        if (_geom is null || uParams.IsEmpty() || vParams.IsEmpty() || nu < 0 || nv < 0
            || (nu + nv) < 1)
        {
            return new NCollection_Array2<Vector3D>();
        }

        Data data = ExtractData();
        int nbU = uParams.Length();
        int nbV = vParams.Length();

        NCollection_Array2<Vector3D> result = new(1, nbU, 1, nbV);

        for (int iU = 1; iU <= nbU; ++iU)
        {
            double u = uParams.Value(uParams.Lower() + iU - 1);
            UContext uCtx = ComputeUContext(data, u);

            for (int iV = 1; iV <= nbV; ++iV)
            {
                double v = vParams.Value(vParams.Lower() + iV - 1);
                result.SetValue(iU, iV, ComputeDN(data, uCtx, v, nu, nv));
            }
        }
        return result;
    }
}
