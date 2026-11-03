namespace Kerf.Modeling;

/// <summary>
/// A geometric value: <see cref="Vector"/>, <see cref="Axis"/>, <see cref="Plane"/> or <see cref="Location"/>.
/// Intersections between them can produce any of these, so they return an <see cref="IGeometry"/>
/// to pattern-match: <c>if (a.Intersect(b) is Vector point) ...</c>.
/// </summary>
public interface IGeometry
{
}
