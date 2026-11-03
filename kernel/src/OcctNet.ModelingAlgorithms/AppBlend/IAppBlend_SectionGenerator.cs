// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// What <see cref="AppBlend_AppSurf{TheSectionGenerator, TheLine, ThePoint}"/> calls on its section generator
/// (the <c>TheSectionGenerator</c> macro of AppBlend_AppSurf.gxx): GeomFill_SectionGenerator, GeomFill_SweepSectionGenerator
/// and, later, Blend_AppFunction implement it (decision 845).
/// </summary>
/// <typeparam name="ThePoint">What the line gives for each of its points: an index for GeomFill_Line, a
/// Blend_Point for BRepBlend_Line.</typeparam>
public interface IAppBlend_SectionGenerator<ThePoint>
{
    /// <summary>The shape of every section: its poles, knots, degree and 2d poles.</summary>
    void GetShape(out int NbPoles, out int NbKnots, out int Degree, out int NbPoles2d);

    /// <summary>Copies the sections' knots into <paramref name="TKnots"/>.</summary>
    void Knots(NCollection_Array1<double> TKnots);

    /// <summary>Copies the sections' multiplicities into <paramref name="TMults"/>.</summary>
    void Mults(NCollection_Array1<int> TMults);

    /// <summary>The section at <paramref name="P"/> with its derivatives; false when they are not computed.</summary>
    bool Section(ThePoint P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Vector3D> DPoles,
                 NCollection_Array1<Point2D> Poles2d, NCollection_Array1<Vector2D> DPoles2d,
                 NCollection_Array1<double> Weigths, NCollection_Array1<double> DWeigths);

    /// <summary>The section at <paramref name="P"/>.</summary>
    void Section(ThePoint P, NCollection_Array1<Point3D> Poles, NCollection_Array1<Point2D> Poles2d,
                 NCollection_Array1<double> Weigths);

    /// <summary>The parameter of the section at <paramref name="P"/>, to impose it on the approximation.</summary>
    double Parameter(ThePoint P);
}

/// <summary>
/// What <see cref="AppBlend_AppSurf{TheSectionGenerator, TheLine, ThePoint}"/> calls on its line (the <c>TheLine</c>
/// macro of AppBlend_AppSurf.gxx): GeomFill_Line and, later, BRepBlend_Line implement it (decision 845).
/// </summary>
public interface IAppBlend_Line<ThePoint>
{
    /// <summary>The number of points (sections) of the line.</summary>
    int NbPoints();

    /// <summary>The point <paramref name="Index"/> (from 1), as the section generator takes it.</summary>
    ThePoint Point(int Index);
}
