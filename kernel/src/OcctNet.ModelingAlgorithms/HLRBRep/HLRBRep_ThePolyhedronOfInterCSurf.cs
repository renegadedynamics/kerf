// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ThePolyhedronOfInterCSurf.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRBRep/HLRBRep_ThePolyhedronOfInterCSurf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Evaluation.Grid;
using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A polyhedron discretizing a surface for HLRBRep_InterCSurf: a grid of points in (U, V) cut into two
/// triangles per cell, with the boxes of the triangles enlarged by the deflection.
/// </summary>
/// <remarks>
/// A working object (decision 21): the singularity flags and <c>DeflectionOverEstimation(double)</c> are public, as
/// in OCCT. OCCT's destructor calls <c>Destroy</c>, which drops the arrays; the port keeps <see cref="Destroy"/>
/// public, as OCCT does, and lets the garbage collector do the rest. The setter <c>Point(thePnt, lig, col, U, V)</c>
/// is empty in OCCT and kept so; <c>Dump</c> (empty) is left out.
/// </remarks>
public sealed class HLRBRep_ThePolyhedronOfInterCSurf
    : IIntCurveSurface_ThePolyhedron<HLRBRep_ThePolyhedronOfInterCSurf, HLRBRep_Surface>
{
    private readonly int nbdeltaU;
    private readonly int nbdeltaV;
    private Box TheBnd = new();
    private NCollection_HArray1<Box>? TheComponentsBnd;
    private double TheDeflection;
    private Point3D[]? C_MyPnts;
    private double[]? C_MyU;
    private double[]? C_MyV;
    private bool UMinSingular;
    private bool UMaxSingular;
    private bool VMinSingular;
    private bool VMaxSingular;
    private double TheBorderDeflection;
    private bool[]? C_MyIsOnBounds;

    /// <summary>
    /// A polyhedron of <paramref name="nbdU"/> by <paramref name="nbdV"/> cells (at least 3 by 3) over the
    /// parameter range [<paramref name="u1"/>, <paramref name="u2"/>] × [<paramref name="v1"/>, <paramref name="v2"/>].
    /// </summary>
    public HLRBRep_ThePolyhedronOfInterCSurf(HLRBRep_Surface Surface, int nbdU, int nbdV, double u1, double v1,
                                                 double u2, double v2)
    {
        nbdeltaU = (nbdU < 3) ? 3 : nbdU;
        nbdeltaV = (nbdV < 3) ? 3 : nbdV;
        TheDeflection = Epsilon(100.0);
        UMinSingular = false;
        UMaxSingular = false;
        VMinSingular = false;
        VMaxSingular = false;
        IntCurveSurface_PolyhedronUtils.AllocateArrays(nbdeltaU, nbdeltaV, out C_MyPnts, out C_MyU, out C_MyV,
                                                       out C_MyIsOnBounds);
        Init(Surface, u1, v1, u2, v2);
    }

    /// <summary>A polyhedron through the points of the surface at the parameters <paramref name="Upars"/> × <paramref name="Vpars"/>.</summary>
    public HLRBRep_ThePolyhedronOfInterCSurf(HLRBRep_Surface Surface, NCollection_Array1<double> Upars,
                                                 NCollection_Array1<double> Vpars)
    {
        nbdeltaU = Upars.Length() - 1; // OCCT: HLRBRep's instantiation doesn't check the length
        nbdeltaV = Vpars.Length() - 1;
        TheDeflection = Epsilon(100.0);
        UMinSingular = false;
        UMaxSingular = false;
        VMinSingular = false;
        VMaxSingular = false;
        IntCurveSurface_PolyhedronUtils.AllocateArrays(nbdeltaU, nbdeltaV, out C_MyPnts, out C_MyU, out C_MyV,
                                                       out C_MyIsOnBounds);
        Init(Surface, Upars, Vpars);
    }

    static HLRBRep_ThePolyhedronOfInterCSurf IIntCurveSurface_ThePolyhedron<HLRBRep_ThePolyhedronOfInterCSurf, HLRBRep_Surface>.Create(
        in HLRBRep_Surface Surface, int nbdU, int nbdV, double U1, double V1, double U2, double V2) =>
        new(Surface, nbdU, nbdV, U1, V1, U2, V2);

    static HLRBRep_ThePolyhedronOfInterCSurf IIntCurveSurface_ThePolyhedron<HLRBRep_ThePolyhedronOfInterCSurf, HLRBRep_Surface>.Create(
        in HLRBRep_Surface Surface, NCollection_Array1<double> Upars, NCollection_Array1<double> Vpars) =>
        new(Surface, Upars, Vpars);

    /// <summary>Drops the arrays of points and parameters (OCCT's destructor calls it).</summary>
    public void Destroy() =>
        IntCurveSurface_PolyhedronUtils.Destroy(ref C_MyPnts, ref C_MyU, ref C_MyV, ref C_MyIsOnBounds);

    private void Init(HLRBRep_Surface Surface, double U0, double V0, double U1, double V1)
    {
        SurfaceGrid anEval = new(Surface.Surface());
        IntCurveSurface_PolyhedronUtils.InitUniform(anEval, U0, V0, U1, V1, nbdeltaU, nbdeltaV, C_MyPnts!, C_MyU!,
                                                    C_MyV!, C_MyIsOnBounds!, ref TheBnd);
        double tol = IntCurveSurface_PolyhedronUtils.ComputeMaxDeflection(Surface.Surface(), this, NbTriangles());
        DeflectionOverEstimation(tol * 1.2);
        FillBounding();
        TheBorderDeflection =
            IntCurveSurface_PolyhedronUtils.ComputeMaxBorderDeflection(anEval, U0, V0, U1, V1, nbdeltaU, nbdeltaV);
    }

    private void Init(HLRBRep_Surface Surface, NCollection_Array1<double> Upars, NCollection_Array1<double> Vpars)
    {
        SurfaceGrid anEval = new(Surface.Surface());
        IntCurveSurface_PolyhedronUtils.InitWithParams(anEval, Upars, Vpars, nbdeltaU, nbdeltaV, C_MyPnts!, C_MyU!,
                                                       C_MyV!, C_MyIsOnBounds!, ref TheBnd);
        double tol = IntCurveSurface_PolyhedronUtils.ComputeMaxDeflection(Surface.Surface(), this, NbTriangles());
        DeflectionOverEstimation(tol * 1.2);
        FillBounding();
        TheBorderDeflection = IntCurveSurface_PolyhedronUtils.ComputeMaxBorderDeflection(
            anEval, Upars[Upars.Lower()], Vpars[Vpars.Lower()], Upars[Upars.Upper()], Vpars[Vpars.Upper()], nbdeltaU,
            nbdeltaV);
    }

    /// <summary>The parameters of the point of index <paramref name="Index"/>.</summary>
    public void Parameters(int Index, out double U, out double V)
    {
        U = C_MyU![Index];
        V = C_MyV![Index];
    }

    /// <summary>Sets the deflection (at least 1e-4) and enlarges the box by it.</summary>
    public void DeflectionOverEstimation(double flec) =>
        IntCurveSurface_PolyhedronUtils.SetDeflectionOverEstimation(flec, out TheDeflection, ref TheBnd);

    public double DeflectionOverEstimation() => TheDeflection;

    /// <summary>The box of the polyhedron.</summary>
    public Box Bounding() => TheBnd;

    /// <summary>Computes the boxes of the triangles.</summary>
    public void FillBounding()
    {
        TheComponentsBnd = new NCollection_HArray1<Box>(1, NbTriangles());
        IntCurveSurface_PolyhedronUtils.FillBounding(C_MyPnts!, nbdeltaU, nbdeltaV, TheDeflection, TheComponentsBnd);
    }

    /// <summary>The boxes of the triangles (the polyhedron's own array: don't modify).</summary>
    public NCollection_HArray1<Box> ComponentsBounding() => TheComponentsBnd!;

    public int NbTriangles() => IntCurveSurface_PolyhedronUtils.NbTriangles(nbdeltaU, nbdeltaV);

    public int NbPoints() => IntCurveSurface_PolyhedronUtils.NbPoints(nbdeltaU, nbdeltaV);

    /// <summary>
    /// The triangle connected to <paramref name="Triang"/> through the edge from <paramref name="Pivot"/> to
    /// <paramref name="Pedge"/> (0 on the boundary), and its other point.
    /// </summary>
    public int TriConnex(int Triang, int Pivot, int Pedge, out int TriCon, out int OtherP) =>
        IntCurveSurface_PolyhedronUtils.TriConnex(Triang, Pivot, Pedge, out TriCon, out OtherP, nbdeltaU, nbdeltaV);

    /// <summary>The plane equation of the triangle <paramref name="Triang"/>.</summary>
    public void PlaneEquation(int Triang, out Xyz NormalVector, out double PolarDistance)
    {
        Triangle(Triang, out int i1, out int i2, out int i3);
        IntCurveSurface_PolyhedronUtils.PlaneEquation(Point(i1), Point(i2), Point(i3), out NormalVector,
                                                      out PolarDistance);
    }

    /// <summary>True if the triangle <paramref name="Triang"/> contains <paramref name="ThePnt"/>.</summary>
    public bool Contain(int Triang, in Point3D ThePnt)
    {
        Triangle(Triang, out int i1, out int i2, out int i3);
        return IntCurveSurface_PolyhedronUtils.Contain(Point(i1), Point(i2), Point(i3), ThePnt);
    }

    /// <summary>The numbers of cells in U and in V.</summary>
    public void Size(out int nbdu, out int nbdv)
    {
        nbdu = nbdeltaU;
        nbdv = nbdeltaV;
    }

    /// <summary>The three vertex indices of the triangle <paramref name="Index"/>.</summary>
    public void Triangle(int Index, out int P1, out int P2, out int P3) =>
        IntCurveSurface_PolyhedronUtils.Triangle(Index, out P1, out P2, out P3, nbdeltaV);

    /// <summary>The point of index <paramref name="Index"/> and its parameters.</summary>
    public Point3D Point(int Index, out double U, out double V)
    {
        U = C_MyU![Index];
        V = C_MyV![Index];
        return C_MyPnts![Index];
    }

    /// <summary>The point of index <paramref name="Index"/>.</summary>
    public Point3D Point(int Index) => C_MyPnts![Index];

    /// <summary>Does nothing (as in OCCT).</summary>
    public void Point(in Point3D thePnt, int lig, int col, double U, double V)
    {
        _ = thePnt;
        _ = lig;
        _ = col;
        _ = U;
        _ = V;
    }

    /// <summary>The point of index <paramref name="Index"/>.</summary>
    public void Point(int Index, out Point3D P) => P = C_MyPnts![Index];

    /// <summary>True if the edge between the two points lies on the boundary of the parameter range.</summary>
    public bool IsOnBound(int Index1, int Index2) =>
        IntCurveSurface_PolyhedronUtils.IsOnBound(Index1, Index2, C_MyIsOnBounds!, nbdeltaU, nbdeltaV);

    /// <summary>The maximum deflection between the boundary isolines and the boundary edges of the triangles.</summary>
    public double GetBorderDeflection() => TheBorderDeflection;

    public void UMinSingularity(bool Sing) => UMinSingular = Sing;

    public void UMaxSingularity(bool Sing) => UMaxSingular = Sing;

    public void VMinSingularity(bool Sing) => VMinSingular = Sing;

    public void VMaxSingularity(bool Sing) => VMaxSingular = Sing;

    public bool HasUMinSingularity() => UMinSingular;

    public bool HasUMaxSingularity() => UMaxSingular;

    public bool HasVMinSingularity() => VMinSingular;

    public bool HasVMaxSingularity() => VMaxSingular;
}
