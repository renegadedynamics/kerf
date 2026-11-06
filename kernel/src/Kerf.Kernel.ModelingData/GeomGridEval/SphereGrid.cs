// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Sphere.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for spherical surface grid points:
/// P(u, v) = Center + R * (cos(v) * (cos(u) * XDir + sin(u) * YDir) + sin(v) * ZDir). The trigonometric values of
/// U are computed once per U parameter.
/// </summary>
internal sealed class SphereGrid : GeomGridEvalGrid.ISurfaceEvaluator
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

    private readonly SphericalSurface? _geom;

    /// <summary>Constructor with the sphere to evaluate (null gives empty results).</summary>
    public SphereGrid(SphericalSurface? sphere)
    {
        _geom = sphere;
    }

    /// <summary>Returns the geometry.</summary>
    public SphericalSurface? Geometry() => _geom;

    private Data ExtractData()
    {
        Sphere3D sph = _geom!.Sphere;
        Point3D center = sph.Location;
        Direction3D xDir = sph.Position.XDirection;
        Direction3D yDir = sph.Position.YDirection;
        Direction3D zDir = sph.Position.Direction;
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
            Radius = sph.Radius,
        };
    }

    private static UContext ComputeUContext(in Data data, double u)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        // DirU = cosU*XDir + sinU*YDir (horizontal direction at angle U)
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
        double cosV = Math.Cos(v);
        double sinV = Math.Sin(v);

        // P = Center + R * (cosV * DirU + sinV * ZDir)
        double rcosV = data.Radius * cosV;
        double rsinV = data.Radius * sinV;
        return new Point3D(data.CX + rcosV * uCtx.dirUX + rsinV * data.ZX,
                          data.CY + rcosV * uCtx.dirUY + rsinV * data.ZY,
                          data.CZ + rcosV * uCtx.dirUZ + rsinV * data.ZZ);
    }

    private static Surface.ResD1 ComputeD1(in Data data, in UContext uCtx, double v)
    {
        double cosV = Math.Cos(v);
        double sinV = Math.Sin(v);

        double rcosV = data.Radius * cosV;
        double rsinV = data.Radius * sinV;

        // D1U = R * cosV * dDirU
        // D1V = R * (-sinV * DirU + cosV * ZDir)
        return new Surface.ResD1
        {
            Point = new Point3D(data.CX + rcosV * uCtx.dirUX + rsinV * data.ZX,
                               data.CY + rcosV * uCtx.dirUY + rsinV * data.ZY,
                               data.CZ + rcosV * uCtx.dirUZ + rsinV * data.ZZ),
            D1U = new Vector3D(rcosV * uCtx.dDirUX, rcosV * uCtx.dDirUY, rcosV * uCtx.dDirUZ),
            D1V = new Vector3D(-rsinV * uCtx.dirUX + rcosV * data.ZX,
                             -rsinV * uCtx.dirUY + rcosV * data.ZY,
                             -rsinV * uCtx.dirUZ + rcosV * data.ZZ),
        };
    }

    private static Surface.ResD2 ComputeD2(in Data data, in UContext uCtx, double v)
    {
        double cosV = Math.Cos(v);
        double sinV = Math.Sin(v);

        double rcosV = data.Radius * cosV;
        double rsinV = data.Radius * sinV;

        // D2U = R * cosV * (-DirU) = -R * cosV * DirU
        // D2V = R * (-cosV * DirU - sinV * ZDir) = -(P - Center) = -relP
        // D2UV = d/dV(D1U) = d/dV(R * cosV * dDirU) = -R * sinV * dDirU
        return new Surface.ResD2
        {
            Point = new Point3D(data.CX + rcosV * uCtx.dirUX + rsinV * data.ZX,
                               data.CY + rcosV * uCtx.dirUY + rsinV * data.ZY,
                               data.CZ + rcosV * uCtx.dirUZ + rsinV * data.ZZ),
            D1U = new Vector3D(rcosV * uCtx.dDirUX, rcosV * uCtx.dDirUY, rcosV * uCtx.dDirUZ),
            D1V = new Vector3D(-rsinV * uCtx.dirUX + rcosV * data.ZX,
                             -rsinV * uCtx.dirUY + rcosV * data.ZY,
                             -rsinV * uCtx.dirUZ + rcosV * data.ZZ),
            D2U = new Vector3D(-rcosV * uCtx.dirUX, -rcosV * uCtx.dirUY, -rcosV * uCtx.dirUZ),
            D2V = new Vector3D(-rcosV * uCtx.dirUX - rsinV * data.ZX,
                             -rcosV * uCtx.dirUY - rsinV * data.ZY,
                             -rcosV * uCtx.dirUZ - rsinV * data.ZZ),
            D2UV = new Vector3D(-rsinV * uCtx.dDirUX, -rsinV * uCtx.dDirUY, -rsinV * uCtx.dDirUZ),
        };
    }

    private static Surface.ResD3 ComputeD3(in Data data, in UContext uCtx, double v)
    {
        double cosV = Math.Cos(v);
        double sinV = Math.Sin(v);

        double rcosV = data.Radius * cosV;
        double rsinV = data.Radius * sinV;

        // D3U = R * cosV * dDirU = D1U
        // D3V = R * (sinV * DirU - cosV * ZDir) = -D1V
        // D3UUV = d/dV(D2U) = d/dV(-R * cosV * DirU) = R * sinV * DirU
        // D3UVV = d/dV(D2UV) = d/dV(-R * sinV * dDirU) = -R * cosV * dDirU
        return new Surface.ResD3
        {
            Point = new Point3D(data.CX + rcosV * uCtx.dirUX + rsinV * data.ZX,
                               data.CY + rcosV * uCtx.dirUY + rsinV * data.ZY,
                               data.CZ + rcosV * uCtx.dirUZ + rsinV * data.ZZ),
            D1U = new Vector3D(rcosV * uCtx.dDirUX, rcosV * uCtx.dDirUY, rcosV * uCtx.dDirUZ),
            D1V = new Vector3D(-rsinV * uCtx.dirUX + rcosV * data.ZX,
                             -rsinV * uCtx.dirUY + rcosV * data.ZY,
                             -rsinV * uCtx.dirUZ + rcosV * data.ZZ),
            D2U = new Vector3D(-rcosV * uCtx.dirUX, -rcosV * uCtx.dirUY, -rcosV * uCtx.dirUZ),
            D2V = new Vector3D(-rcosV * uCtx.dirUX - rsinV * data.ZX,
                             -rcosV * uCtx.dirUY - rsinV * data.ZY,
                             -rcosV * uCtx.dirUZ - rsinV * data.ZZ),
            D2UV = new Vector3D(-rsinV * uCtx.dDirUX, -rsinV * uCtx.dDirUY, -rsinV * uCtx.dDirUZ),
            D3U = new Vector3D(-rcosV * uCtx.dDirUX, -rcosV * uCtx.dDirUY, -rcosV * uCtx.dDirUZ),
            D3V = new Vector3D(rsinV * uCtx.dirUX - rcosV * data.ZX,
                             rsinV * uCtx.dirUY - rcosV * data.ZY,
                             rsinV * uCtx.dirUZ - rcosV * data.ZZ),
            D3UUV = new Vector3D(rsinV * uCtx.dirUX, rsinV * uCtx.dirUY, rsinV * uCtx.dirUZ),
            D3UVV = new Vector3D(-rcosV * uCtx.dDirUX, -rcosV * uCtx.dDirUY, -rcosV * uCtx.dDirUZ),
        };
    }

    private static Vector3D ComputeDN(in Data data, in UContext uCtx, double v, int nu, int nv)
    {
        double cosV = Math.Cos(v);
        double sinV = Math.Sin(v);

        // Cyclic phases for U and V (period 4)
        int phaseU = nu % 4;
        int phaseV = nv % 4;

        // U-phase coefficients: d^n/du^n of cosU and sinU
        // n=0: cosU, sinU -> DirU
        // n=1: -sinU, cosU -> dDirU
        // n=2: -cosU, -sinU -> -DirU
        // n=3: sinU, -cosU -> -dDirU
        double dirX, dirY, dirZ;
        switch (phaseU)
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

        // V-phase coefficients for XY term (cosV) and Z term (sinV)
        double coeffVXY, coeffVZ;
        switch (phaseV)
        {
            case 0:
                coeffVXY = cosV;
                coeffVZ = sinV;
                break;
            case 1:
                coeffVXY = -sinV;
                coeffVZ = cosV;
                break;
            case 2:
                coeffVXY = -cosV;
                coeffVZ = -sinV;
                break;
            case 3:
                coeffVXY = sinV;
                coeffVZ = -cosV;
                break;
            default:
                coeffVXY = 0.0;
                coeffVZ = 0.0;
                break;
        }

        // Z contribution only if NU == 0 (no U derivative on Z term)
        double zTermX = 0.0, zTermY = 0.0, zTermZ = 0.0;
        if (nu == 0)
        {
            zTermX = coeffVZ * data.ZX;
            zTermY = coeffVZ * data.ZY;
            zTermZ = coeffVZ * data.ZZ;
        }

        return new Vector3D(data.Radius * (coeffVXY * dirX + zTermX),
                          data.Radius * (coeffVXY * dirY + zTermY),
                          data.Radius * (coeffVXY * dirZ + zTermZ));
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
