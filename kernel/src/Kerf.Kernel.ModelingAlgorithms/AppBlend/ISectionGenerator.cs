// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Fillets.Blend;

/// <summary>
/// What <see cref="AppSurf{TheSectionGenerator, TheLine, ThePoint}"/> calls on its section generator
/// (the <c>TheSectionGenerator</c> macro of AppBlend_AppSurf.gxx): GeomFill_SectionGenerator, GeomFill_SweepSectionGenerator
/// and, later, Blend_AppFunction implement it (decision 845).
/// </summary>
/// <typeparam name="ThePoint">What the line gives for each of its points: an index for GeomFill_Line, a
/// Blend_Point for BRepBlend_Line.</typeparam>
internal interface ISectionGenerator<ThePoint>
{
    /// <summary>The shape of every section: its poles, knots, degree and 2d poles.</summary>
    void GetShape(out int nbPoles, out int nbKnots, out int degree, out int nbPoles2d);

    /// <summary>Copies the sections' knots into <paramref name="tKnots"/>.</summary>
    void Knots(NCollection_Array1<double> tKnots);

    /// <summary>Copies the sections' multiplicities into <paramref name="tMults"/>.</summary>
    void Mults(NCollection_Array1<int> tMults);

    /// <summary>The section at <paramref name="p"/> with its derivatives; false when they are not computed.</summary>
    bool Section(ThePoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Vector3D> dPoles,
                 NCollection_Array1<Point2D> poles2d, NCollection_Array1<Vector2D> dPoles2d,
                 NCollection_Array1<double> weigths, NCollection_Array1<double> dWeigths);

    /// <summary>The section at <paramref name="p"/>.</summary>
    void Section(ThePoint p, NCollection_Array1<Point3D> poles, NCollection_Array1<Point2D> poles2d,
                 NCollection_Array1<double> weigths);

    /// <summary>The parameter of the section at <paramref name="p"/>, to impose it on the approximation.</summary>
    double Parameter(ThePoint p);
}

/// <summary>
/// What <see cref="AppSurf{TheSectionGenerator, TheLine, ThePoint}"/> calls on its line (the <c>TheLine</c>
/// macro of AppBlend_AppSurf.gxx): GeomFill_Line and, later, BRepBlend_Line implement it (decision 845).
/// </summary>
internal interface ILine<ThePoint>
{
    /// <summary>The number of points (sections) of the line.</summary>
    int NbPoints();

    /// <summary>The point <paramref name="index"/> (from 1), as the section generator takes it.</summary>
    ThePoint Point(int index);
}
