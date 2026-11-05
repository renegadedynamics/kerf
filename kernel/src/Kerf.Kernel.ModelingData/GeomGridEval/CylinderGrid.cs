// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Cylinder.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Cylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for cylindrical surface grid points:
/// P(u, v) = Center + R * (cos(u) * XDir + sin(u) * YDir) + v * ZDir. The trigonometric values are computed
/// once per U parameter.
/// </summary>
internal sealed class CylinderGrid : GeomGridEvalGrid.ISurfaceEvaluator
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

    private readonly CylindricalSurface? _geom;

    /// <summary>Constructor with the cylinder to evaluate (null gives empty results).</summary>
    public CylinderGrid(CylindricalSurface? cylinder)
    {
        _geom = cylinder;
    }

    /// <summary>Returns the geometry.</summary>
    public CylindricalSurface? Geometry() => _geom;

    private Data ExtractData()
    {
        Cylinder3D cyl = _geom!.Cylinder;
        Point3D center = cyl.Location;
        Direction3D xDir = cyl.Position.XDirection;
        Direction3D yDir = cyl.Position.YDirection;
        Direction3D zDir = cyl.Position.Direction;
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
            Radius = cyl.Radius,
        };
    }

    private static UContext ComputeUContext(in Data data, double u)
    {
        double cosU = Math.Cos(u);
        double sinU = Math.Sin(u);
        // DirU = cosU*XDir + sinU*YDir (circle point direction at angle U)
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
        // P = Center + R * DirU + v * ZDir
        return new Point3D(data.CX + data.Radius * uCtx.dirUX + v * data.ZX,
                          data.CY + data.Radius * uCtx.dirUY + v * data.ZY,
                          data.CZ + data.Radius * uCtx.dirUZ + v * data.ZZ);
    }

    private static Surface.ResD1 ComputeD1(in Data data, in UContext uCtx, double v)
    {
        // D1U = R * dDirU
        // D1V = ZDir
        return new Surface.ResD1
        {
            Point = new Point3D(data.CX + data.Radius * uCtx.dirUX + v * data.ZX,
                               data.CY + data.Radius * uCtx.dirUY + v * data.ZY,
                               data.CZ + data.Radius * uCtx.dirUZ + v * data.ZZ),
            D1U = new Vector3D(data.Radius * uCtx.dDirUX,
                             data.Radius * uCtx.dDirUY,
                             data.Radius * uCtx.dDirUZ),
            D1V = new Vector3D(data.ZX, data.ZY, data.ZZ),
        };
    }

    private static Surface.ResD2 ComputeD2(in Data data, in UContext uCtx, double v)
    {
        // D2U = R * (-DirU) = -R * DirU
        // D2V = 0
        // D2UV = 0
        Vector3D zero = new(0.0, 0.0, 0.0);
        return new Surface.ResD2
        {
            Point = new Point3D(data.CX + data.Radius * uCtx.dirUX + v * data.ZX,
                               data.CY + data.Radius * uCtx.dirUY + v * data.ZY,
                               data.CZ + data.Radius * uCtx.dirUZ + v * data.ZZ),
            D1U = new Vector3D(data.Radius * uCtx.dDirUX,
                             data.Radius * uCtx.dDirUY,
                             data.Radius * uCtx.dDirUZ),
            D1V = new Vector3D(data.ZX, data.ZY, data.ZZ),
            D2U = new Vector3D(-data.Radius * uCtx.dirUX,
                             -data.Radius * uCtx.dirUY,
                             -data.Radius * uCtx.dirUZ),
            D2V = zero,
            D2UV = zero,
        };
    }

    private static Surface.ResD3 ComputeD3(in Data data, in UContext uCtx, double v)
    {
        // D3U = R * (-dDirU) = -D1U
        // D3V = 0
        // D3UUV = 0
        // D3UVV = 0
        Vector3D zero = new(0.0, 0.0, 0.0);
        return new Surface.ResD3
        {
            Point = new Point3D(data.CX + data.Radius * uCtx.dirUX + v * data.ZX,
                               data.CY + data.Radius * uCtx.dirUY + v * data.ZY,
                               data.CZ + data.Radius * uCtx.dirUZ + v * data.ZZ),
            D1U = new Vector3D(data.Radius * uCtx.dDirUX,
                             data.Radius * uCtx.dDirUY,
                             data.Radius * uCtx.dDirUZ),
            D1V = new Vector3D(data.ZX, data.ZY, data.ZZ),
            D2U = new Vector3D(-data.Radius * uCtx.dirUX,
                             -data.Radius * uCtx.dirUY,
                             -data.Radius * uCtx.dirUZ),
            D2V = zero,
            D2UV = zero,
            D3U = new Vector3D(-data.Radius * uCtx.dDirUX,
                             -data.Radius * uCtx.dDirUY,
                             -data.Radius * uCtx.dDirUZ),
            D3V = zero,
            D3UUV = zero,
            D3UVV = zero,
        };
    }

    private static Vector3D ComputeDN(in Data data, in UContext uCtx, double v, int nu, int nv)
    {
        _ = v;
        // For cylinder P(u,v) = C + R*(cos(u)*X + sin(u)*Y) + v*Z:
        // - V derivatives: only D_{0,1} = Z is non-zero, all D_{nu,nv} = 0 for nv >= 2
        // - Mixed derivatives D_{nu,1} = 0 for nu >= 1 (Z term has no U dependence)
        // - Pure U derivatives cycle with period 4
        if (nv > 1)
        {
            return new Vector3D(0.0, 0.0, 0.0);
        }
        if (nv == 1)
        {
            if (nu == 0)
            {
                return new Vector3D(data.ZX, data.ZY, data.ZZ);
            }
            return new Vector3D(0.0, 0.0, 0.0);
        }

        // Pure U derivatives (theNV == 0, theNU >= 1)
        // d^n/du^n of DirU cycles: DirU, dDirU, -DirU, -dDirU
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
        return new Vector3D(data.Radius * dirX, data.Radius * dirY, data.Radius * dirZ);
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
