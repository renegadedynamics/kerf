// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_SvSurfaces.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/ApproxInt/ApproxInt_SvSurfaces.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Approximation;

/// <summary>
/// The root of the classes that compute the 3D and 2D points and tangents of the intersection line of two surfaces
/// at given parameters (u1, v1) and (u2, v2) of an intersection point.
/// </summary>
/// <remarks>
/// With <see cref="GetUseSolver"/> true, the parameters are a first approximation of an exact intersection point,
/// refined by the intersection's solver before the values are computed; otherwise they are taken as an exact
/// intersection point. A working object (decision 21): its implementations cache their last two results.
/// </remarks>
public abstract class SvSurfaces
{
    private bool _useSolver;

    protected SvSurfaces()
    {
        _useSolver = false;
    }

    /// <summary>
    /// Computes the point, the 3D tangent and the 2D tangents on both surfaces of the intersection line at
    /// (u1, v1, u2, v2), which it may refine; true if the tangents can be computed.
    /// </summary>
    /// <remarks>
    /// OCCT's references: the outputs are left unchanged where OCCT leaves them (a result found in the cache, a
    /// failure), so they are <c>ref</c>.
    /// </remarks>
    public abstract bool Compute(ref double u1, ref double v1, ref double u2, ref double v2, ref Point3D pt,
                                 ref Vector3D tg, ref Vector2D tguv1, ref Vector2D tguv2);

    /// <summary>The point of the intersection line at (u1, v1, u2, v2).</summary>
    public abstract void Pnt(double u1, double v1, double u2, double v2, ref Point3D p);

    /// <summary>Computes the point on the curve and its parameters on the surfaces.</summary>
    public abstract bool SeekPoint(double u1, double v1, double u2, double v2, ref PntOn2S point);

    /// <summary>The 3D tangent of the intersection line at (u1, v1, u2, v2).</summary>
    public abstract bool Tangency(double u1, double v1, double u2, double v2, ref Vector3D tg);

    /// <summary>The 2D tangent of the intersection line on the first surface.</summary>
    public abstract bool TangencyOnSurf1(double u1, double v1, double u2, double v2, ref Vector2D tg);

    /// <summary>The 2D tangent of the intersection line on the second surface.</summary>
    public abstract bool TangencyOnSurf2(double u1, double v1, double u2, double v2, ref Vector2D tg);

    /// <summary>Whether the parameters given are refined by the solver (see the remarks of the class).</summary>
    public void SetUseSolver(bool useSol)
    {
        _useSolver = useSol;
    }

    public virtual bool GetUseSolver() => _useSolver;
}
