// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_ThePolyhedronOfHInter.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurveSurface/IntCurveSurface_ThePolyhedronOfHInter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// A polyhedron discretizing a surface for IntCurveSurface_HInter: a grid of points in (U, V) cut into two
/// triangles per cell, with the boxes of the triangles enlarged by the deflection.
/// </summary>
/// <remarks>
/// A working object (decision 21): the singularity flags and <c>DeflectionOverEstimation(double)</c> are public, as
/// in OCCT. OCCT's destructor calls <c>Destroy</c>, which drops the arrays; the port keeps <see cref="Destroy"/>
/// public, as OCCT does, and lets the garbage collector do the rest. The setter <c>Point(thePnt, lig, col, U, V)</c>
/// is empty in OCCT and kept so; <c>Dump</c> (empty) is left out.
/// </remarks>
public sealed class ThePolyhedronOfHInter
    : IThePolyhedron<ThePolyhedronOfHInter, SurfaceAdaptorBase>
{
    private readonly int _nbdeltaU;
    private readonly int _nbdeltaV;
    private Box _theBnd = new();
    private NCollection_HArray1<Box>? _theComponentsBnd;
    private double _theDeflection;
    private Point3D[]? C_MyPnts;
    private double[]? C_MyU;
    private double[]? C_MyV;
    private bool _uMinSingular;
    private bool _uMaxSingular;
    private bool _vMinSingular;
    private bool _vMaxSingular;
    private double _theBorderDeflection;
    private bool[]? C_MyIsOnBounds;

    private static int NbDeltaFromParameters(NCollection_Array1<double> pars, string argumentName)
    {
        Standard_OutOfRange_Raise_if(pars.Length() < 2, argumentName);
        return pars.Length() - 1;
    }

    /// <summary>
    /// A polyhedron of <paramref name="nbdU"/> by <paramref name="nbdV"/> cells (at least 3 by 3) over the
    /// parameter range [<paramref name="u1"/>, <paramref name="u2"/>] × [<paramref name="v1"/>, <paramref name="v2"/>].
    /// </summary>
    public ThePolyhedronOfHInter(SurfaceAdaptorBase surface, int nbdU, int nbdV, double u1, double v1,
                                                 double u2, double v2)
    {
        _nbdeltaU = (nbdU < 3) ? 3 : nbdU;
        _nbdeltaV = (nbdV < 3) ? 3 : nbdV;
        _theDeflection = Epsilon(100.0);
        _uMinSingular = false;
        _uMaxSingular = false;
        _vMinSingular = false;
        _vMaxSingular = false;
        PolyhedronUtils.AllocateArrays(_nbdeltaU, _nbdeltaV, out C_MyPnts, out C_MyU, out C_MyV,
                                                       out C_MyIsOnBounds);
        Init(surface, u1, v1, u2, v2);
    }

    /// <summary>A polyhedron through the points of the surface at the parameters <paramref name="upars"/> × <paramref name="vpars"/>.</summary>
    /// <exception cref="OutOfRangeException">An array has fewer than two values.</exception>
    public ThePolyhedronOfHInter(SurfaceAdaptorBase surface, NCollection_Array1<double> upars,
                                                 NCollection_Array1<double> vpars)
    {
        _nbdeltaU = NbDeltaFromParameters(upars, "IntCurveSurface_ThePolyhedronOfHInter::IntCurveSurface_"
                                                + "ThePolyhedronOfHInter() - Upars must contain at least two values");
        _nbdeltaV = NbDeltaFromParameters(vpars, "IntCurveSurface_ThePolyhedronOfHInter::IntCurveSurface_"
                                                + "ThePolyhedronOfHInter() - Vpars must contain at least two values");
        _theDeflection = Epsilon(100.0);
        _uMinSingular = false;
        _uMaxSingular = false;
        _vMinSingular = false;
        _vMaxSingular = false;
        PolyhedronUtils.AllocateArrays(_nbdeltaU, _nbdeltaV, out C_MyPnts, out C_MyU, out C_MyV,
                                                       out C_MyIsOnBounds);
        Init(surface, upars, vpars);
    }

    static ThePolyhedronOfHInter IThePolyhedron<ThePolyhedronOfHInter, SurfaceAdaptorBase>.Create(
        in SurfaceAdaptorBase surface, int nbdU, int nbdV, double u1, double v1, double u2, double v2) =>
        new(surface, nbdU, nbdV, u1, v1, u2, v2);

    static ThePolyhedronOfHInter IThePolyhedron<ThePolyhedronOfHInter, SurfaceAdaptorBase>.Create(
        in SurfaceAdaptorBase surface, NCollection_Array1<double> upars, NCollection_Array1<double> vpars) =>
        new(surface, upars, vpars);

    /// <summary>Drops the arrays of points and parameters (OCCT's destructor calls it).</summary>
    public void Destroy() =>
        PolyhedronUtils.Destroy(ref C_MyPnts, ref C_MyU, ref C_MyV, ref C_MyIsOnBounds);

    private void Init(SurfaceAdaptorBase surface, double u0, double v0, double u1, double v1)
    {
        SurfaceGrid eval = new(surface);
        PolyhedronUtils.InitUniform(eval, u0, v0, u1, v1, _nbdeltaU, _nbdeltaV, C_MyPnts!, C_MyU!,
                                                    C_MyV!, C_MyIsOnBounds!, ref _theBnd);
        double tol = PolyhedronUtils.ComputeMaxDeflection(surface, this, TriangleCount);
        DeflectionOverEstimation(tol * 1.2);
        FillBounding();
        _theBorderDeflection =
            PolyhedronUtils.ComputeMaxBorderDeflection(eval, u0, v0, u1, v1, _nbdeltaU, _nbdeltaV);
    }

    private void Init(SurfaceAdaptorBase surface, NCollection_Array1<double> upars, NCollection_Array1<double> vpars)
    {
        SurfaceGrid eval = new(surface);
        PolyhedronUtils.InitWithParams(eval, upars, vpars, _nbdeltaU, _nbdeltaV, C_MyPnts!, C_MyU!,
                                                       C_MyV!, C_MyIsOnBounds!, ref _theBnd);
        double tol = PolyhedronUtils.ComputeMaxDeflection(surface, this, TriangleCount);
        DeflectionOverEstimation(tol * 1.2);
        FillBounding();
        _theBorderDeflection = PolyhedronUtils.ComputeMaxBorderDeflection(
            eval, upars[upars.Lower()], vpars[vpars.Lower()], upars[upars.Upper()], vpars[vpars.Upper()], _nbdeltaU,
            _nbdeltaV);
    }

    /// <summary>The parameters of the point of index <paramref name="index"/>.</summary>
    public void Parameters(int index, out double u, out double v)
    {
        u = C_MyU![index];
        v = C_MyV![index];
    }

    /// <summary>Sets the deflection (at least 1e-4) and enlarges the box by it.</summary>
    public void DeflectionOverEstimation(double flec) =>
        PolyhedronUtils.SetDeflectionOverEstimation(flec, out _theDeflection, ref _theBnd);

    public double DeflectionOverEstimation() => _theDeflection;

    /// <summary>The box of the polyhedron.</summary>
    public Box Bounding() => _theBnd;

    /// <summary>Computes the boxes of the triangles.</summary>
    public void FillBounding()
    {
        _theComponentsBnd = new NCollection_HArray1<Box>(1, TriangleCount);
        PolyhedronUtils.FillBounding(C_MyPnts!, _nbdeltaU, _nbdeltaV, _theDeflection, _theComponentsBnd);
    }

    /// <summary>The boxes of the triangles (the polyhedron's own array: don't modify).</summary>
    public NCollection_HArray1<Box> ComponentsBounding() => _theComponentsBnd!;

    public int TriangleCount => PolyhedronUtils.NbTriangles(_nbdeltaU, _nbdeltaV);

    public int PointCount => PolyhedronUtils.NbPoints(_nbdeltaU, _nbdeltaV);

    /// <summary>
    /// The triangle connected to <paramref name="triang"/> through the edge from <paramref name="pivot"/> to
    /// <paramref name="pedge"/> (0 on the boundary), and its other point.
    /// </summary>
    public int TriConnex(int triang, int pivot, int pedge, out int triCon, out int otherP) =>
        PolyhedronUtils.TriConnex(triang, pivot, pedge, out triCon, out otherP, _nbdeltaU, _nbdeltaV);

    /// <summary>The plane equation of the triangle <paramref name="triang"/>.</summary>
    public void PlaneEquation(int triang, out Xyz normalVector, out double polarDistance)
    {
        Triangle(triang, out int i1, out int i2, out int i3);
        PolyhedronUtils.PlaneEquation(Point(i1), Point(i2), Point(i3), out normalVector,
                                                      out polarDistance);
    }

    /// <summary>True if the triangle <paramref name="triang"/> contains <paramref name="thePnt"/>.</summary>
    public bool Contain(int triang, in Point3D thePnt)
    {
        Triangle(triang, out int i1, out int i2, out int i3);
        return PolyhedronUtils.Contain(Point(i1), Point(i2), Point(i3), thePnt);
    }

    /// <summary>The numbers of cells in U and in V.</summary>
    public void Size(out int nbdu, out int nbdv)
    {
        nbdu = _nbdeltaU;
        nbdv = _nbdeltaV;
    }

    /// <summary>The three vertex indices of the triangle <paramref name="index"/>.</summary>
    public void Triangle(int index, out int p1, out int p2, out int p3) =>
        PolyhedronUtils.Triangle(index, out p1, out p2, out p3, _nbdeltaV);

    /// <summary>The point of index <paramref name="index"/> and its parameters.</summary>
    public Point3D Point(int index, out double u, out double v)
    {
        u = C_MyU![index];
        v = C_MyV![index];
        return C_MyPnts![index];
    }

    /// <summary>The point of index <paramref name="index"/>.</summary>
    public Point3D Point(int index) => C_MyPnts![index];

    /// <summary>Does nothing (as in OCCT).</summary>
    public void Point(in Point3D pnt, int lig, int col, double u, double v)
    {
        _ = pnt;
        _ = lig;
        _ = col;
        _ = u;
        _ = v;
    }

    /// <summary>The point of index <paramref name="index"/>.</summary>
    public void Point(int index, out Point3D p) => p = C_MyPnts![index];

    /// <summary>True if the edge between the two points lies on the boundary of the parameter range.</summary>
    public bool IsOnBound(int index1, int index2) =>
        PolyhedronUtils.IsOnBound(index1, index2, C_MyIsOnBounds!, _nbdeltaU, _nbdeltaV);

    /// <summary>The maximum deflection between the boundary isolines and the boundary edges of the triangles.</summary>
    public double GetBorderDeflection() => _theBorderDeflection;

    public void UMinSingularity(bool sing) => _uMinSingular = sing;

    public void UMaxSingularity(bool sing) => _uMaxSingular = sing;

    public void VMinSingularity(bool sing) => _vMinSingular = sing;

    public void VMaxSingularity(bool sing) => _vMaxSingular = sing;

    public bool HasUMinSingularity => _uMinSingular;

    public bool HasUMaxSingularity => _uMaxSingular;

    public bool HasVMinSingularity => _vMinSingular;

    public bool HasVMaxSingularity => _vMaxSingular;
}
