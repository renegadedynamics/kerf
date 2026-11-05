// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Geometry;

/// <summary>
/// A geometric value that an intersection of values can give: a <see cref="Point3D"/> here, and the modelling layer's
/// axis, plane and location. Such a result is one of several kinds, to pattern-match:
/// <c>if (a.Intersect(b) is Point3D point) ...</c>.
/// </summary>
public interface IGeometry
{
}
