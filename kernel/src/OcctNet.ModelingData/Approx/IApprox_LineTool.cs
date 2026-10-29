// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The "LineTool" of Approx's ComputeLine and BSplComputeLine: the members of
/// <see cref="IAppParCurves_LineTool{MultiLine}"/>, and the three that let a line tool add points to a MultiLine
/// when an approximation fails (AppDef_MyLineTool never adds any; GeomInt's and BRepApprox's tools do).
/// </summary>
/// <remarks>Not an OCCT type (decisions 142 and 202).</remarks>
/// <typeparam name="MultiLine">The MultiLine.</typeparam>
public interface IApprox_LineTool<MultiLine> : IAppParCurves_LineTool<MultiLine>
{
    /// <summary>Whether points were (or can be) added between the points <paramref name="I1"/> and <paramref name="I2"/>.</summary>
    static abstract Approx_Status WhatStatus(MultiLine ML, int I1, int I2);

    /// <summary>
    /// A MultiLine with at least <paramref name="NbPMin"/> points between the points <paramref name="I1"/> and
    /// <paramref name="I2"/> of <paramref name="ML"/>.
    /// </summary>
    static abstract MultiLine MakeMLBetween(MultiLine ML, int I1, int I2, int NbPMin);

    /// <summary>
    /// Makes in <paramref name="OtherLine"/> a MultiLine with one more point, near the point
    /// <paramref name="indbad"/>, between the points <paramref name="I1"/> and <paramref name="I2"/>; false if it
    /// can't.
    /// </summary>
    /// <remarks>OCCT passes a default-constructed line by reference; implementations must always assign it.</remarks>
    static abstract bool MakeMLOneMorePoint(MultiLine ML, int I1, int I2, int indbad, out MultiLine OtherLine);
}
