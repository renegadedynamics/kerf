// Port of: src/ModelingData/TKGeomBase/AppCont/AppCont_Function.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Approximation;

/// <summary>
/// A continuous function f(u) whose values are a set of 3d and/or 2d points: what the continuous approximations
/// (AppCont_LeastSquare, Approx_FitAndDivide) approximate.
/// </summary>
/// <remarks>
/// An implementation sets <see cref="_nbPnt"/> and <see cref="_nbPnt2d"/> in its constructor (both are -1
/// until then, as in OCCT), and fills the arrays passed to <see cref="Value"/> and <see cref="D1"/> from index 1:
/// the 3d points (or vectors) in one array, the 2d ones in the other. The arrays are allocated by the caller with at
/// least one element each, even when there are no points of that dimension.
/// </remarks>
internal abstract class Function
{
    /// <summary>The number of 3d points returned by Value and D1.</summary>
    protected int _nbPnt;

    /// <summary>The number of 2d points returned by Value and D1.</summary>
    protected int _nbPnt2d;

    protected Function()
    {
        _nbPnt = -1;
        _nbPnt2d = -1;
    }

    /// <summary>The number of 3d and 2d points returned by <see cref="Value"/> and <see cref="D1"/>.</summary>
    public void GetNumberOfPoints(out int nbPnt, out int nbPnt2d)
    {
        nbPnt = _nbPnt;
        nbPnt2d = _nbPnt2d;
    }

    /// <summary>The number of 3d points returned by <see cref="Value"/> and <see cref="D1"/>.</summary>
    public int GetNbOf3dPoints() => _nbPnt;

    /// <summary>The number of 2d points returned by <see cref="Value"/> and <see cref="D1"/>.</summary>
    public int GetNbOf2dPoints() => _nbPnt2d;

    /// <summary>The first parameter of the function.</summary>
    public abstract double FirstParameter();

    /// <summary>The last parameter of the function.</summary>
    public abstract double LastParameter();

    /// <summary>Fills the arrays with the points at <paramref name="u"/>; false if they can't be computed.</summary>
    public abstract bool Value(double u, NCollection_Array1<Point2D> pnt2d, NCollection_Array1<Point3D> pnt);

    /// <summary>Fills the arrays with the derivatives at <paramref name="u"/>; false if they can't be computed.</summary>
    public abstract bool D1(double u, NCollection_Array1<Vector2D> vec2d, NCollection_Array1<Vector3D> vec);

    /// <summary>
    /// The periodicity of the output coordinate <paramref name="dimIdx"/> (1 to 3 * NbPnt + 2 * NbPnt2d: the
    /// coordinates of the 3d points, then of the 2d points). Not periodic by default.
    /// </summary>
    public virtual void PeriodInformation(int dimIdx, out bool isPeriodic, out double period)
    {
        isPeriodic = false;
        period = 0.0;
    }
}
