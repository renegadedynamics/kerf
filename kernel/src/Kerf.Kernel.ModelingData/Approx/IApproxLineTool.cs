// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Approximation.Interfaces;

/// <summary>
/// The "LineTool" of Approx's ComputeLine and BSplComputeLine: the members of
/// <see cref="ILineTool{MultiLine}"/>, and the three that let a line tool add points to a MultiLine
/// when an approximation fails (AppDef_MyLineTool never adds any; GeomInt's and BRepApprox's tools do).
/// </summary>
/// <remarks>Not an OCCT type (decisions 142 and 202).</remarks>
/// <typeparam name="MultiLine">The MultiLine.</typeparam>
public interface IApproxLineTool<MultiLine> : ILineTool<MultiLine>
{
    /// <summary>Whether points were (or can be) added between the points <paramref name="i1"/> and <paramref name="i2"/>.</summary>
    static abstract Status WhatStatus(MultiLine ml, int i1, int i2);

    /// <summary>
    /// A MultiLine with at least <paramref name="nbPMin"/> points between the points <paramref name="i1"/> and
    /// <paramref name="i2"/> of <paramref name="ml"/>.
    /// </summary>
    static abstract MultiLine MakeMLBetween(MultiLine ml, int i1, int i2, int nbPMin);

    /// <summary>
    /// Makes in <paramref name="otherLine"/> a MultiLine with one more point, near the point
    /// <paramref name="indbad"/>, between the points <paramref name="i1"/> and <paramref name="i2"/>; false if it
    /// can't.
    /// </summary>
    /// <remarks>OCCT passes a default-constructed line by reference; implementations must always assign it.</remarks>
    static abstract bool MakeMLOneMorePoint(MultiLine ml, int i1, int i2, int indbad, out MultiLine otherLine);
}
