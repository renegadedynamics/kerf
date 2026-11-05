// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OtherSurface.hxx
// Port of: src/ModelingData/TKG3d/GeomGridEval/GeomGridEval_OtherSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Evaluation.Grid;

/// <summary>
/// Fallback evaluator for any surface type: point-by-point evaluation through the D0/D1/D2/D3/DN methods of an
/// Adaptor3d_Surface or a Geom_Surface. The slowest evaluator, but it handles any surface.
/// </summary>
/// <remarks>The evaluator keeps the adaptor or surface it is given (OCCT keeps a pointer or a handle).</remarks>
internal sealed class OtherSurfaceGrid : GeomGridEvalGrid.ISurfaceEvaluator
{
    // OCCT's std::variant<const Adaptor3d_Surface*, occ::handle<Geom_Surface>>: one of the two.
    private readonly object _surface;

    /// <summary>Constructor with an adaptor.</summary>
    public OtherSurfaceGrid(SurfaceAdaptorBase surface)
    {
        _surface = surface;
    }

    /// <summary>Constructor with a geometry.</summary>
    public OtherSurfaceGrid(Surface surface)
    {
        _surface = surface;
    }

    /// <summary>Evaluate D0 at given UV parameters.</summary>
    private void EvaluateD0(double u, double v, out Point3D point)
    {
        if (_surface is SurfaceAdaptorBase surf)
        {
            surf.D0(u, v, out point);
        }
        else
        {
            ((Surface)_surface).D0(u, v, out point);
        }
    }

    /// <summary>Evaluate D1 at given UV parameters.</summary>
    private void EvaluateD1(double u, double v, out Point3D point, out Vector3D d1U, out Vector3D d1V)
    {
        if (_surface is SurfaceAdaptorBase surf)
        {
            surf.D1(u, v, out point, out d1U, out d1V);
        }
        else
        {
            ((Surface)_surface).D1(u, v, out point, out d1U, out d1V);
        }
    }

    /// <summary>Evaluate D2 at given UV parameters.</summary>
    private void EvaluateD2(double u,
                            double v,
                            out Point3D point,
                            out Vector3D d1U,
                            out Vector3D d1V,
                            out Vector3D d2U,
                            out Vector3D d2V,
                            out Vector3D d2UV)
    {
        if (_surface is SurfaceAdaptorBase surf)
        {
            surf.D2(u, v, out point, out d1U, out d1V, out d2U, out d2V, out d2UV);
        }
        else
        {
            ((Surface)_surface).D2(u, v, out point, out d1U, out d1V, out d2U, out d2V,
                                         out d2UV);
        }
    }

    /// <summary>Evaluate D3 at given UV parameters.</summary>
    private void EvaluateD3(double u,
                            double v,
                            out Point3D point,
                            out Vector3D d1U,
                            out Vector3D d1V,
                            out Vector3D d2U,
                            out Vector3D d2V,
                            out Vector3D d2UV,
                            out Vector3D d3U,
                            out Vector3D d3V,
                            out Vector3D d3UUV,
                            out Vector3D d3UVV)
    {
        if (_surface is SurfaceAdaptorBase surf)
        {
            surf.D3(u, v, out point, out d1U, out d1V, out d2U, out d2V, out d2UV,
                     out d3U, out d3V, out d3UUV, out d3UVV);
        }
        else
        {
            ((Surface)_surface).D3(u, v, out point, out d1U, out d1V, out d2U, out d2V,
                                         out d2UV, out d3U, out d3V, out d3UUV, out d3UVV);
        }
    }

    /// <summary>Evaluate DN at given UV parameters.</summary>
    private Vector3D EvaluateDN(double u, double v, int nu, int nv)
    {
        if (_surface is SurfaceAdaptorBase surf)
        {
            return surf.DN(u, v, nu, nv);
        }
        return ((Surface)_surface).DN(u, v, nu, nv);
    }

    /// <summary>Evaluates all grid points; [1..NbU, 1..NbV], or empty if a list is empty.</summary>
    public NCollection_Array2<Point3D> EvaluateGrid(NCollection_Array1<double> uParams,
                                                   NCollection_Array1<double> vParams)
    {
        int nbU = uParams.Length();
        int nbV = vParams.Length();
        if (nbU == 0 || nbV == 0)
        {
            return new NCollection_Array2<Point3D>();
        }

        NCollection_Array2<Point3D> result = new(1, nbU, 1, nbV);
        for (int iU = 1; iU <= nbU; ++iU)
        {
            double u = uParams.Value(uParams.Lower() + iU - 1);
            for (int iV = 1; iV <= nbV; ++iV)
            {
                double v = vParams.Value(vParams.Lower() + iV - 1);
                EvaluateD0(u, v, out Point3D point);
                result.SetValue(iU, iV, point);
            }
        }
        return result;
    }

    /// <summary>Evaluates all grid points with first partial derivatives.</summary>
    public NCollection_Array2<Surface.ResD1> EvaluateGridD1(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        int nbU = uParams.Length();
        int nbV = vParams.Length();
        if (nbU == 0 || nbV == 0)
        {
            return new NCollection_Array2<Surface.ResD1>();
        }

        NCollection_Array2<Surface.ResD1> result = new(1, nbU, 1, nbV);
        for (int iU = 1; iU <= nbU; ++iU)
        {
            double u = uParams.Value(uParams.Lower() + iU - 1);
            for (int iV = 1; iV <= nbV; ++iV)
            {
                double v = vParams.Value(vParams.Lower() + iV - 1);
                ref Surface.ResD1 res = ref result.ChangeValue(iU, iV);
                EvaluateD1(u, v, out res.Point, out res.D1U, out res.D1V);
            }
        }
        return result;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the second order.</summary>
    public NCollection_Array2<Surface.ResD2> EvaluateGridD2(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        int nbU = uParams.Length();
        int nbV = vParams.Length();
        if (nbU == 0 || nbV == 0)
        {
            return new NCollection_Array2<Surface.ResD2>();
        }

        NCollection_Array2<Surface.ResD2> result = new(1, nbU, 1, nbV);
        for (int iU = 1; iU <= nbU; ++iU)
        {
            double u = uParams.Value(uParams.Lower() + iU - 1);
            for (int iV = 1; iV <= nbV; ++iV)
            {
                double v = vParams.Value(vParams.Lower() + iV - 1);
                ref Surface.ResD2 res = ref result.ChangeValue(iU, iV);
                EvaluateD2(u, v, out res.Point, out res.D1U, out res.D1V, out res.D2U, out res.D2V,
                           out res.D2UV);
            }
        }
        return result;
    }

    /// <summary>Evaluates all grid points with partial derivatives up to the third order.</summary>
    public NCollection_Array2<Surface.ResD3> EvaluateGridD3(NCollection_Array1<double> uParams,
                                                                 NCollection_Array1<double> vParams)
    {
        int nbU = uParams.Length();
        int nbV = vParams.Length();
        if (nbU == 0 || nbV == 0)
        {
            return new NCollection_Array2<Surface.ResD3>();
        }

        NCollection_Array2<Surface.ResD3> result = new(1, nbU, 1, nbV);
        for (int iU = 1; iU <= nbU; ++iU)
        {
            double u = uParams.Value(uParams.Lower() + iU - 1);
            for (int iV = 1; iV <= nbV; ++iV)
            {
                double v = vParams.Value(vParams.Lower() + iV - 1);
                ref Surface.ResD3 res = ref result.ChangeValue(iU, iV);
                EvaluateD3(u, v, out res.Point, out res.D1U, out res.D1V, out res.D2U, out res.D2V,
                           out res.D2UV, out res.D3U, out res.D3V, out res.D3UUV, out res.D3UVV);
            }
        }
        return result;
    }

    /// <summary>Evaluates the partial derivative of orders (NU, NV) on the grid.</summary>
    public NCollection_Array2<Vector3D> EvaluateGridDN(NCollection_Array1<double> uParams,
                                                     NCollection_Array1<double> vParams,
                                                     int nu,
                                                     int nv)
    {
        int nbU = uParams.Length();
        int nbV = vParams.Length();
        if (nbU == 0 || nbV == 0 || nu < 0 || nv < 0 || (nu + nv) < 1)
        {
            return new NCollection_Array2<Vector3D>();
        }

        NCollection_Array2<Vector3D> result = new(1, nbU, 1, nbV);
        for (int iU = 1; iU <= nbU; ++iU)
        {
            double u = uParams.Value(uParams.Lower() + iU - 1);
            for (int iV = 1; iV <= nbV; ++iV)
            {
                double v = vParams.Value(vParams.Lower() + iV - 1);
                Vector3D dn = EvaluateDN(u, v, nu, nv);
                result.SetValue(iU, iV, dn);
            }
        }
        return result;
    }
}
