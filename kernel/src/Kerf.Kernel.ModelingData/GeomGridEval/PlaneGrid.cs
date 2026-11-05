// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_Plane.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Efficient batch evaluator for plane grid points, by the direct formula P(u, v) = Location + u * XDir + v * YDir.
/// </summary>
internal sealed class PlaneGrid : GeomGridEvalGrid.ISurfaceEvaluator
{
    private readonly PlaneSurface? _geom;

    /// <summary>Constructor with the plane to evaluate (null gives empty results).</summary>
    public PlaneGrid(PlaneSurface? plane)
    {
        _geom = plane;
    }

    /// <summary>Returns the geometry.</summary>
    public PlaneSurface? Geometry() => _geom;

    /// <summary>Evaluates grid points at the Cartesian product of U and V parameters; [1..NbU, 1..NbV].</summary>
    public NCollection_Array2<Point3D> EvaluateGrid(NCollection_Array1<double> uParams,
                                                   NCollection_Array1<double> vParams)
    {
        int nbU = uParams.Length();
        int nbV = vParams.Length();
        if (_geom is null || nbU == 0 || nbV == 0)
        {
            return new NCollection_Array2<Point3D>();
        }

        NCollection_Array2<Point3D> result = new(1, nbU, 1, nbV);

        // Extract plane data from geometry
        Plane3D pln = _geom.Plane;
        Point3D loc = pln.Location;
        Direction3D xDir = pln.Position.XDirection;
        Direction3D yDir = pln.Position.YDirection;

        // Pre-extract coordinates for performance
        double locX = loc.X;
        double locY = loc.Y;
        double locZ = loc.Z;
        double xx = xDir.X;
        double xy = xDir.Y;
        double xz = xDir.Z;
        double yx = yDir.X;
        double yy = yDir.Y;
        double yz = yDir.Z;

        for (int iU = 1; iU <= nbU; ++iU)
        {
            double u = uParams.Value(uParams.Lower() + iU - 1);
            double uX = locX + u * xx;
            double uY = locY + u * xy;
            double uZ = locZ + u * xz;

            for (int iV = 1; iV <= nbV; ++iV)
            {
                double v = vParams.Value(vParams.Lower() + iV - 1);
                result.SetValue(iU, iV, new Point3D(uX + v * yx, uY + v * yy, uZ + v * yz));
            }
        }
        return result;
    }

    /// <summary>Evaluates grid points with first partial derivatives (constant for a plane).</summary>
    public NCollection_Array2<Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        int nbU = uParams.Length();
        int nbV = vParams.Length();
        if (_geom is null || nbU == 0 || nbV == 0)
        {
            return new NCollection_Array2<Surface.ResD1>();
        }

        NCollection_Array2<Surface.ResD1> result = new(1, nbU, 1, nbV);

        Plane3D pln = _geom.Plane;
        Point3D loc = pln.Location;
        Direction3D xDir = pln.Position.XDirection;
        Direction3D yDir = pln.Position.YDirection;

        double locX = loc.X;
        double locY = loc.Y;
        double locZ = loc.Z;
        double xx = xDir.X;
        double xy = xDir.Y;
        double xz = xDir.Z;
        double yx = yDir.X;
        double yy = yDir.Y;
        double yz = yDir.Z;

        // D1U and D1V are constant for a plane
        Vector3D d1U = new(xx, xy, xz);
        Vector3D d1V = new(yx, yy, yz);

        for (int iU = 1; iU <= nbU; ++iU)
        {
            double u = uParams.Value(uParams.Lower() + iU - 1);
            double uX = locX + u * xx;
            double uY = locY + u * xy;
            double uZ = locZ + u * xz;

            for (int iV = 1; iV <= nbV; ++iV)
            {
                double v = vParams.Value(vParams.Lower() + iV - 1);
                result[iU, iV] = new Surface.ResD1
                {
                    Point = new Point3D(uX + v * yx, uY + v * yy, uZ + v * yz),
                    D1U = d1U,
                    D1V = d1V,
                };
            }
        }
        return result;
    }

    /// <summary>Evaluates grid points with first and second partial derivatives.</summary>
    public NCollection_Array2<Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        int nbU = uParams.Length();
        int nbV = vParams.Length();
        if (_geom is null || nbU == 0 || nbV == 0)
        {
            return new NCollection_Array2<Surface.ResD2>();
        }

        NCollection_Array2<Surface.ResD2> result = new(1, nbU, 1, nbV);

        Plane3D pln = _geom.Plane;
        Point3D loc = pln.Location;
        Direction3D xDir = pln.Position.XDirection;
        Direction3D yDir = pln.Position.YDirection;

        double locX = loc.X;
        double locY = loc.Y;
        double locZ = loc.Z;
        double xx = xDir.X;
        double xy = xDir.Y;
        double xz = xDir.Z;
        double yx = yDir.X;
        double yy = yDir.Y;
        double yz = yDir.Z;

        Vector3D d1U = new(xx, xy, xz);
        Vector3D d1V = new(yx, yy, yz);
        Vector3D zero = new(0, 0, 0); // All second derivatives are zero for a plane

        for (int iU = 1; iU <= nbU; ++iU)
        {
            double u = uParams.Value(uParams.Lower() + iU - 1);
            double uX = locX + u * xx;
            double uY = locY + u * xy;
            double uZ = locZ + u * xz;

            for (int iV = 1; iV <= nbV; ++iV)
            {
                double v = vParams.Value(vParams.Lower() + iV - 1);
                result[iU, iV] = new Surface.ResD2
                {
                    Point = new Point3D(uX + v * yx, uY + v * yy, uZ + v * yz),
                    D1U = d1U,
                    D1V = d1V,
                    D2U = zero,
                    D2V = zero,
                    D2UV = zero,
                };
            }
        }
        return result;
    }

    /// <summary>Evaluates grid points with derivatives up to the third order.</summary>
    public NCollection_Array2<Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        int nbU = uParams.Length();
        int nbV = vParams.Length();
        if (_geom is null || nbU == 0 || nbV == 0)
        {
            return new NCollection_Array2<Surface.ResD3>();
        }

        NCollection_Array2<Surface.ResD3> result = new(1, nbU, 1, nbV);

        Plane3D pln = _geom.Plane;
        Point3D loc = pln.Location;
        Direction3D xDir = pln.Position.XDirection;
        Direction3D yDir = pln.Position.YDirection;

        double locX = loc.X;
        double locY = loc.Y;
        double locZ = loc.Z;
        double xx = xDir.X;
        double xy = xDir.Y;
        double xz = xDir.Z;
        double yx = yDir.X;
        double yy = yDir.Y;
        double yz = yDir.Z;

        Vector3D d1U = new(xx, xy, xz);
        Vector3D d1V = new(yx, yy, yz);
        Vector3D zero = new(0, 0, 0); // All derivatives of order >= 2 are zero for a plane

        for (int iU = 1; iU <= nbU; ++iU)
        {
            double u = uParams.Value(uParams.Lower() + iU - 1);
            double uX = locX + u * xx;
            double uY = locY + u * xy;
            double uZ = locZ + u * xz;

            for (int iV = 1; iV <= nbV; ++iV)
            {
                double v = vParams.Value(vParams.Lower() + iV - 1);
                result[iU, iV] = new Surface.ResD3
                {
                    Point = new Point3D(uX + v * yx, uY + v * yy, uZ + v * yz),
                    D1U = d1U,
                    D1V = d1V,
                    D2U = zero,
                    D2V = zero,
                    D2UV = zero,
                    D3U = zero,
                    D3V = zero,
                    D3UUV = zero,
                    D3UVV = zero,
                };
            }
        }
        return result;
    }

    /// <summary>Evaluates the partial derivative d^(NU+NV)S/(dU^NU dV^NV) at all grid points.</summary>
    public NCollection_Array2<Vector3D> EvaluateGridDN(NCollection_Array1<double> uParams,
                                                     NCollection_Array1<double> vParams,
                                                     int nu,
                                                     int nv)
    {
        int nbU = uParams.Length();
        int nbV = vParams.Length();
        if (_geom is null || nbU == 0 || nbV == 0 || nu < 0 || nv < 0 || (nu + nv) < 1)
        {
            return new NCollection_Array2<Vector3D>();
        }

        NCollection_Array2<Vector3D> result = new(1, nbU, 1, nbV);

        // For a plane, only D1U (1,0) and D1V (0,1) are non-zero
        Vector3D derivative = new(0, 0, 0);
        if (nu == 1 && nv == 0)
        {
            // D1U = XDir
            Direction3D xDir = _geom.Plane.Position.XDirection;
            derivative = new Vector3D(xDir.X, xDir.Y, xDir.Z);
        }
        else if (nu == 0 && nv == 1)
        {
            // D1V = YDir
            Direction3D yDir = _geom.Plane.Position.YDirection;
            derivative = new Vector3D(yDir.X, yDir.Y, yDir.Z);
        }
        // All other derivatives are zero

        for (int iU = 1; iU <= nbU; ++iU)
        {
            for (int iV = 1; iV <= nbV; ++iV)
            {
                result.SetValue(iU, iV, derivative);
            }
        }
        return result;
    }
}
