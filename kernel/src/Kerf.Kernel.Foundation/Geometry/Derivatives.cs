// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Geometry;

/// <summary>
/// A curve's point and derivatives at a parameter, up to the third: what <c>Curve3D.Derivatives</c> returns in place of
/// OCCT's <c>D1</c>, <c>D2</c> and <c>D3</c>. The derivatives above <see cref="Order"/> are zero.
/// </summary>
/// <param name="Order">How many derivatives were computed, 0 to 3.</param>
/// <param name="Point">The point.</param>
/// <param name="D1">The first derivative.</param>
/// <param name="D2">The second derivative.</param>
/// <param name="D3">The third derivative.</param>
public readonly record struct CurveDerivatives(int Order, Point3D Point, Vector3D D1 = default, Vector3D D2 = default, Vector3D D3 = default);

/// <summary>A 2D curve's point and derivatives at a parameter, up to the third (<see cref="CurveDerivatives"/>'s twin).</summary>
/// <param name="Order">How many derivatives were computed, 0 to 3.</param>
/// <param name="Point">The point.</param>
/// <param name="D1">The first derivative.</param>
/// <param name="D2">The second derivative.</param>
/// <param name="D3">The third derivative.</param>
public readonly record struct CurveDerivatives2D(int Order, Point2D Point, Vector2D D1 = default, Vector2D D2 = default, Vector2D D3 = default);

/// <summary>
/// A surface's point and partial derivatives at (u, v), up to the third order: what <c>Surface.Derivatives</c> returns in
/// place of OCCT's <c>D1</c>, <c>D2</c> and <c>D3</c>. The derivatives above <see cref="Order"/> are zero.
/// </summary>
/// <param name="Order">The highest order computed, 0 to 3.</param>
/// <param name="Point">The point.</param>
/// <param name="DU">The first derivative in u.</param>
/// <param name="DV">The first derivative in v.</param>
/// <param name="DUU">The second derivative in u.</param>
/// <param name="DVV">The second derivative in v.</param>
/// <param name="DUV">The mixed second derivative.</param>
/// <param name="DUUU">The third derivative in u.</param>
/// <param name="DVVV">The third derivative in v.</param>
/// <param name="DUUV">The third derivative, twice in u and once in v.</param>
/// <param name="DUVV">The third derivative, once in u and twice in v.</param>
public readonly record struct SurfaceDerivatives(int Order, Point3D Point, Vector3D DU = default, Vector3D DV = default,
                                                 Vector3D DUU = default, Vector3D DVV = default, Vector3D DUV = default,
                                                 Vector3D DUUU = default, Vector3D DVVV = default, Vector3D DUUV = default,
                                                 Vector3D DUVV = default);

/// <summary>A point on a curve: its parameter, the point, and its distance from what was projected.</summary>
/// <param name="Parameter">The curve's parameter.</param>
/// <param name="Point">The point on the curve.</param>
/// <param name="Distance">The distance from the point projected; zero where there was none.</param>
public readonly record struct CurvePoint(double Parameter, Point3D Point, double Distance = 0);

/// <summary>A point on a surface: its parameters, the point, and its distance from what was projected.</summary>
/// <param name="U">The surface's U parameter.</param>
/// <param name="V">The surface's V parameter.</param>
/// <param name="Point">The point on the surface.</param>
/// <param name="Distance">The distance from the point projected; zero where there was none.</param>
public readonly record struct SurfacePoint(double U, double V, Point3D Point, double Distance = 0);
