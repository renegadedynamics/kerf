// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Cone.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Cone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for conical surface grid points:
/// P(u, v) = Center + (RefRadius + v * sin(Ang)) * (cos(u) * XDir + sin(u) * YDir) + v * cos(Ang) * ZDir. The
/// trigonometric values of U are computed once per U parameter.
/// </summary>
public sealed class ConeGrid : GeomGridEvalGrid.ISurfaceEvaluator
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

    private readonly ConicalSurface? _geom;

    /// <summary>Constructor with the cone to evaluate (null gives empty results).</summary>
    public ConeGrid(ConicalSurface? cone)
    {
        _geom = cone;
    }

    /// <summary>Returns the geometry.</summary>
    public ConicalSurface? Geometry() => _geom;

    private Data ExtractData()
    {
        Cone cone = _geom!.Cone();
        Point3D center = cone.Location();
        Direction3D xDir = cone.Position().XDirection();
        Direction3D yDir = cone.Position().YDirection();
        Direction3D zDir = cone.Position().Direction();
        return new Data
        {
            CX = center.X(),
            CY = center.Y(),
            CZ = center.Z(),
            XX = xDir.X(),
            XY = xDir.Y(),
            XZ = xDir.Z(),
            YX = yDir.X(),
            YY = yDir.Y(),
            YZ = yDir.Z(),
            ZX = zDir.X(),
            ZY = zDir.Y(),
            ZZ = zDir.Z(),
            RefRadius = cone.RefRadius(),
            SinAng = Math.Sin(cone.SemiAngle()),
            CosAng = Math.Cos(cone.SemiAngle()),
        };
    }

    private static UContext ComputeUContext(in Data data, double u)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        // DirU = cosU*XDir + sinU*YDir (radial direction at angle U)
        // dDirU = -sinU*XDir + cosU*YDir (derivative of DirU w.r.t. U)
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
        // K(v) = RefRadius + v * sin(Ang)
        double k = data.RefRadius + v * data.SinAng;
        // ZOffset(v) = v * cos(Ang)
        double zOff = v * data.CosAng;

        // P = Center + K(v) * DirU + ZOffset(v) * ZDir
        return new Point3D(data.CX + k * uCtx.dirUX + zOff * data.ZX,
                          data.CY + k * uCtx.dirUY + zOff * data.ZY,
                          data.CZ + k * uCtx.dirUZ + zOff * data.ZZ);
    }

    private static Surface.ResD1 ComputeD1(in Data data, in UContext uCtx, double v)
    {
        double k = data.RefRadius + v * data.SinAng;
        double zOff = v * data.CosAng;

        // D1U = K(v) * dDirU
        // D1V = sin(Ang)*DirU + cos(Ang)*ZDir
        return new Surface.ResD1
        {
            Point = new Point3D(data.CX + k * uCtx.dirUX + zOff * data.ZX,
                               data.CY + k * uCtx.dirUY + zOff * data.ZY,
                               data.CZ + k * uCtx.dirUZ + zOff * data.ZZ),
            D1U = new Vector3D(k * uCtx.dDirUX, k * uCtx.dDirUY, k * uCtx.dDirUZ),
            D1V = new Vector3D(data.SinAng * uCtx.dirUX + data.CosAng * data.ZX,
                             data.SinAng * uCtx.dirUY + data.CosAng * data.ZY,
                             data.SinAng * uCtx.dirUZ + data.CosAng * data.ZZ),
        };
    }

    private static Surface.ResD2 ComputeD2(in Data data, in UContext uCtx, double v)
    {
        double k = data.RefRadius + v * data.SinAng;
        double zOff = v * data.CosAng;

        // D2U = K * (-DirU) = -K * DirU
        // D2V = 0
        // D2UV = sin(Ang) * dDirU
        Vector3D zero = new(0.0, 0.0, 0.0);
        return new Surface.ResD2
        {
            Point = new Point3D(data.CX + k * uCtx.dirUX + zOff * data.ZX,
                               data.CY + k * uCtx.dirUY + zOff * data.ZY,
                               data.CZ + k * uCtx.dirUZ + zOff * data.ZZ),
            D1U = new Vector3D(k * uCtx.dDirUX, k * uCtx.dDirUY, k * uCtx.dDirUZ),
            D1V = new Vector3D(data.SinAng * uCtx.dirUX + data.CosAng * data.ZX,
                             data.SinAng * uCtx.dirUY + data.CosAng * data.ZY,
                             data.SinAng * uCtx.dirUZ + data.CosAng * data.ZZ),
            D2U = new Vector3D(-k * uCtx.dirUX, -k * uCtx.dirUY, -k * uCtx.dirUZ),
            D2V = zero,
            D2UV = new Vector3D(data.SinAng * uCtx.dDirUX,
                              data.SinAng * uCtx.dDirUY,
                              data.SinAng * uCtx.dDirUZ),
        };
    }

    private static Surface.ResD3 ComputeD3(in Data data, in UContext uCtx, double v)
    {
        double k = data.RefRadius + v * data.SinAng;
        double zOff = v * data.CosAng;

        // D3U = -K * dDirU = -D1U
        // D3V = 0
        // D3UUV = -sin(Ang) * DirU
        // D3UVV = 0
        Vector3D zero = new(0.0, 0.0, 0.0);
        return new Surface.ResD3
        {
            Point = new Point3D(data.CX + k * uCtx.dirUX + zOff * data.ZX,
                               data.CY + k * uCtx.dirUY + zOff * data.ZY,
                               data.CZ + k * uCtx.dirUZ + zOff * data.ZZ),
            D1U = new Vector3D(k * uCtx.dDirUX, k * uCtx.dDirUY, k * uCtx.dDirUZ),
            D1V = new Vector3D(data.SinAng * uCtx.dirUX + data.CosAng * data.ZX,
                             data.SinAng * uCtx.dirUY + data.CosAng * data.ZY,
                             data.SinAng * uCtx.dirUZ + data.CosAng * data.ZZ),
            D2U = new Vector3D(-k * uCtx.dirUX, -k * uCtx.dirUY, -k * uCtx.dirUZ),
            D2V = zero,
            D2UV = new Vector3D(data.SinAng * uCtx.dDirUX,
                              data.SinAng * uCtx.dDirUY,
                              data.SinAng * uCtx.dDirUZ),
            D3U = new Vector3D(-k * uCtx.dDirUX, -k * uCtx.dDirUY, -k * uCtx.dDirUZ),
            D3V = zero,
            D3UUV = new Vector3D(-data.SinAng * uCtx.dirUX,
                               -data.SinAng * uCtx.dirUY,
                               -data.SinAng * uCtx.dirUZ),
            D3UVV = zero,
        };
    }

    private static Vector3D ComputeDN(in Data data, in UContext uCtx, double v, int nu, int nv)
    {
        // For cone P(u,v) = C + K(v) * DirU(u) + v*cos(Ang)*Z
        // where K(v) = RefRadius + v*sin(Ang), DirU(u) = cos(u)*X + sin(u)*Y
        //
        // V is linear, so:
        //   - All derivatives with NV >= 2 are zero
        //   - D_{0,1} = sin(Ang)*DirU + cos(Ang)*Z
        //   - D_{nu,1} = sin(Ang) * d^nu(DirU)/du^nu for nu >= 1
        //   - D_{nu,0} = K(v) * d^nu(DirU)/du^nu

        if (nv > 1)
        {
            return new Vector3D(0.0, 0.0, 0.0);
        }

        // DirU derivatives cycle with period 4:
        //   phase 0: DirU
        //   phase 1: dDirU
        //   phase 2: -DirU
        //   phase 3: -dDirU
        int phase = nu % 4;
        double dirX, dirY, dirZ;
        switch (phase)
        {
            case 0:
                dirX = uCtx.dirUX;
                dirY = uCtx.dirUY;
                dirZ = uCtx.dirUZ;
                break;
            case 1:
                dirX = uCtx.dDirUX;
                dirY = uCtx.dDirUY;
                dirZ = uCtx.dDirUZ;
                break;
            case 2:
                dirX = -uCtx.dirUX;
                dirY = -uCtx.dirUY;
                dirZ = -uCtx.dirUZ;
                break;
            case 3:
                dirX = -uCtx.dDirUX;
                dirY = -uCtx.dDirUY;
                dirZ = -uCtx.dDirUZ;
                break;
            default:
                dirX = 0.0;
                dirY = 0.0;
                dirZ = 0.0;
                break;
        }

        if (nv == 1)
        {
            if (nu == 0)
            {
                // D_{0,1} = sin(Ang)*DirU + cos(Ang)*Z
                return new Vector3D(data.SinAng * uCtx.dirUX + data.CosAng * data.ZX,
                                  data.SinAng * uCtx.dirUY + data.CosAng * data.ZY,
                                  data.SinAng * uCtx.dirUZ + data.CosAng * data.ZZ);
            }
            // D_{nu,1} = sin(Ang) * d^nu(DirU)/du^nu for nu >= 1
            return new Vector3D(data.SinAng * dirX, data.SinAng * dirY, data.SinAng * dirZ);
        }

        // Pure U derivative (theNV == 0, theNU >= 1)
        // D_{nu,0} = K(v) * d^nu(DirU)/du^nu
        double k = data.RefRadius + v * data.SinAng;
        return new Vector3D(k * dirX, k * dirY, k * dirZ);
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
