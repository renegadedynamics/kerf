// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The "ToolLine" of the AppParCurves generics: what OCCT's <c>.gxx</c> files call on the <c>ToolLine</c> macro, the
/// static tool that reads a MultiLine (AppDef_MyLineTool over AppDef_MultiLine, GeomInt_TheMultiLineToolOfWLApprox
/// over GeomInt_TheMultiLineOfWLApprox, BRepApprox_TheMultiLineToolOfApprox, ...).
/// </summary>
/// <remarks>
/// <para>
/// Not an OCCT type (decision 142). An instantiation is a struct implementing these static members, used as the
/// <c>ToolLine</c> type argument: <c>AppParCurves_LeastSquare&lt;AppDef_MultiLine, AppDef_MyLineTool&gt;</c>. The
/// JIT specialises the generic code for each struct, as the C++ compiler does for each <c>_0.cxx</c> instantiation
/// (decision 75). Lines and points are indexed from <see cref="FirstPoint"/> to <see cref="LastPoint"/>; the
/// arrays passed in are indexed from 1, the 3d points (or vectors) first, then the 2d ones.
/// </para>
/// <para>
/// OCCT's line tools have more members (<c>WhatStatus</c>, <c>MakeMLBetween</c>, <c>MakeMLOneMorePoint</c>, used by
/// Approx's ComputeLine and BSplComputeLine): the units porting those derive an interface from this one.
/// </para>
/// </remarks>
/// <typeparam name="MultiLine">The MultiLine: a set of points, each made of NbP3d 3d points and NbP2d 2d points.
/// A class is expected (it is passed by value to every member).</typeparam>
public interface IAppParCurves_LineTool<MultiLine>
{
    /// <summary>The index of the first point of the line.</summary>
    static abstract int FirstPoint(MultiLine ML);

    /// <summary>The index of the last point of the line.</summary>
    static abstract int LastPoint(MultiLine ML);

    /// <summary>The number of 2d points of each MultiPoint.</summary>
    static abstract int NbP2d(MultiLine ML);

    /// <summary>The number of 3d points of each MultiPoint.</summary>
    static abstract int NbP3d(MultiLine ML);

    /// <summary>Fills <paramref name="tabPt"/> with the 3d points of the MultiPoint <paramref name="MPointIndex"/>.</summary>
    static abstract void Value(MultiLine ML, int MPointIndex, NCollection_Array1<Point3D> tabPt);

    /// <summary>Fills <paramref name="tabPt2d"/> with the 2d points of the MultiPoint <paramref name="MPointIndex"/>.</summary>
    static abstract void Value(MultiLine ML, int MPointIndex, NCollection_Array1<Point2D> tabPt2d);

    /// <summary>Fills both arrays with the points of the MultiPoint <paramref name="MPointIndex"/>.</summary>
    static abstract void Value(MultiLine ML, int MPointIndex, NCollection_Array1<Point3D> tabPt,
                               NCollection_Array1<Point2D> tabPt2d);

    /// <summary>
    /// Fills <paramref name="tabV"/> with the tangents at the MultiPoint <paramref name="MPointIndex"/>; false if
    /// there are none.
    /// </summary>
    static abstract bool Tangency(MultiLine ML, int MPointIndex, NCollection_Array1<Vector3D> tabV);

    /// <summary>
    /// Fills <paramref name="tabV2d"/> with the 2d tangents at the MultiPoint <paramref name="MPointIndex"/>; false if
    /// there are none.
    /// </summary>
    static abstract bool Tangency(MultiLine ML, int MPointIndex, NCollection_Array1<Vector2D> tabV2d);

    /// <summary>Fills both arrays with the tangents at the MultiPoint <paramref name="MPointIndex"/>; false if there are none.</summary>
    static abstract bool Tangency(MultiLine ML, int MPointIndex, NCollection_Array1<Vector3D> tabV,
                                  NCollection_Array1<Vector2D> tabV2d);

    /// <summary>
    /// Fills <paramref name="tabV"/> with the curvature vectors at the MultiPoint <paramref name="MPointIndex"/>;
    /// false if there are none.
    /// </summary>
    static abstract bool Curvature(MultiLine ML, int MPointIndex, NCollection_Array1<Vector3D> tabV);

    /// <summary>
    /// Fills <paramref name="tabV2d"/> with the 2d curvature vectors at the MultiPoint
    /// <paramref name="MPointIndex"/>; false if there are none.
    /// </summary>
    static abstract bool Curvature(MultiLine ML, int MPointIndex, NCollection_Array1<Vector2D> tabV2d);

    /// <summary>
    /// Fills both arrays with the curvature vectors at the MultiPoint <paramref name="MPointIndex"/>; false if
    /// there are none.
    /// </summary>
    static abstract bool Curvature(MultiLine ML, int MPointIndex, NCollection_Array1<Vector3D> tabV,
                                   NCollection_Array1<Vector2D> tabV2d);
}
