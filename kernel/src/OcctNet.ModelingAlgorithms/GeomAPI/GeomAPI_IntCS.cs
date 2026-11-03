// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_IntCS.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_IntCS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The intersections of a curve and a surface: points (with their parameters on the surface and the curve) and
/// segments (parts of the curve lying on the surface), through IntCurveSurface_HInter on the whole curve and
/// surface.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Perform</c> can be called again. Results are returned by value;
/// <see cref="Segment"/> builds a new trimmed curve on the stored curve. (IntCurveSurface_HInter makes no segments,
/// so <see cref="NbSegments"/> is always 0.)
/// </remarks>
public class GeomAPI_IntCS
{
    private Geom_Curve? myCurve;
    private readonly IntCurveSurface_HInter myIntCS = new();

    /// <summary>An empty intersector; <see cref="Perform"/> computes the intersections.</summary>
    public GeomAPI_IntCS()
    {
    }

    /// <summary>Computes the intersections of the curve <paramref name="C"/> and the surface <paramref name="S"/>.</summary>
    public GeomAPI_IntCS(Geom_Curve C, Geom_Surface S)
    {
        Perform(C, S);
    }

    /// <summary>Computes the intersections of the curve <paramref name="C"/> and the surface <paramref name="S"/>.</summary>
    public void Perform(Geom_Curve C, Geom_Surface S)
    {
        myCurve = C;
        GeomAdaptor_Curve HC = new(C);
        GeomAdaptor_Surface HS = new(S);
        myIntCS.Perform(HC, HS);
    }

    /// <summary>True if the intersections were computed.</summary>
    public bool IsDone() => myIntCS.IsDone();

    /// <summary>The number of intersection points.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int NbPoints() => myIntCS.NbPoints();

    /// <summary>The intersection point of range <paramref name="Index"/>.</summary>
    public Point3D Point(int Index) => myIntCS.Point(Index).Pnt();

    /// <summary>
    /// The parameters (<paramref name="U"/>, <paramref name="V"/>) on the surface and <paramref name="W"/> on the
    /// curve of the intersection point of range <paramref name="Index"/>.
    /// </summary>
    public void Parameters(int Index, out double U, out double V, out double W)
    {
        IntCurveSurface_IntersectionPoint ThePoint = myIntCS.Point(Index);
        U = ThePoint.U();
        V = ThePoint.V();
        W = ThePoint.W();
    }

    /// <summary>The number of intersection segments.</summary>
    public int NbSegments() => myIntCS.NbSegments();

    /// <summary>The part of the curve that is the intersection segment of range <paramref name="Index"/>.</summary>
    public Geom_Curve Segment(int Index)
    {
        IntCurveSurface_IntersectionPoint FirstPoint = myIntCS.Segment(Index).FirstPoint();
        IntCurveSurface_IntersectionPoint LastPoint = myIntCS.Segment(Index).SecondPoint();
        Geom_TrimmedCurve TheCurve = new(myCurve!, FirstPoint.W(), LastPoint.W());
        return TheCurve;
    }

    /// <summary>The parameters on the surface of the ends of the intersection segment of range <paramref name="Index"/>.</summary>
    public void Parameters(int Index, out double U1, out double V1, out double U2, out double V2)
    {
        IntCurveSurface_IntersectionPoint FirstPoint = myIntCS.Segment(Index).FirstPoint();
        IntCurveSurface_IntersectionPoint LastPoint = myIntCS.Segment(Index).SecondPoint();
        U1 = FirstPoint.U();
        V1 = FirstPoint.V();
        U2 = LastPoint.U();
        V2 = LastPoint.V();
    }
}
