// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ThePathPointOfTheSearch.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_ThePathPointOfTheSearch_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A solution point of the boundary search (<see cref="Contap_TheSearch"/>): the point, its tolerance, the arc
/// and the parameter on it, and the vertex of the domain when the point is one (not new).
/// </summary>
/// <remarks>
/// A C++ value class, so a struct (decision 609). <c>new()</c> is OCCT's empty constructor (a new point);
/// <c>default</c> is not new, so ported code writes <c>new()</c>. The <c>SetValue</c> overloads are internal
/// mutators (reached by the boundary search through <see cref="IIntStart_PathPoint{TSelf}"/>); the constructors are
/// their twins. The arc and the vertex are compared by reference (OCCT's handles).
/// </remarks>
public partial struct Contap_ThePathPointOfTheSearch
    : IIntStart_PathPoint<Contap_ThePathPointOfTheSearch>, IEquatable<Contap_ThePathPointOfTheSearch>
{
    private Point3D point;
    private double tol;
    private bool isnew;
    private HVertex? vtx;
    private Curve2DAdaptorBase? arc;
    private double param;

    public Contap_ThePathPointOfTheSearch()
    {
        point = new Point3D();
        tol = 0.0;
        isnew = true;
        vtx = null;
        arc = null;
        param = 0.0;
    }

    /// <summary>A point on the vertex <paramref name="V"/> of the arc <paramref name="A"/>.</summary>
    public Contap_ThePathPointOfTheSearch(in Point3D P, double Tol, HVertex V, Curve2DAdaptorBase A,
                                               double Parameter)
    {
        point = P;
        tol = Tol;
        isnew = false;
        vtx = V;
        arc = A;
        param = Parameter;
    }

    /// <summary>A new point on the arc <paramref name="A"/>.</summary>
    public Contap_ThePathPointOfTheSearch(in Point3D P, double Tol, Curve2DAdaptorBase A, double Parameter)
    {
        point = P;
        tol = Tol;
        isnew = true;
        vtx = null;
        arc = A;
        param = Parameter;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValue(in Point3D P, double Tol, HVertex V, Curve2DAdaptorBase A, double Parameter)
    {
        isnew = false;
        point = P;
        tol = Tol;
        vtx = V;
        arc = A;
        param = Parameter;
    }

    /// <summary>Sets a new point (the vertex field is kept, as in OCCT; it is not readable on a new point).</summary>
    internal void SetValue(in Point3D P, double Tol, Curve2DAdaptorBase A, double Parameter)
    {
        isnew = true;
        point = P;
        tol = Tol;
        arc = A;
        param = Parameter;
    }

    // Queries

    public readonly Point3D Value() => point;

    public readonly double Tolerance() => tol;

    /// <summary>True if the point is not a vertex of the domain.</summary>
    public readonly bool IsNew() => isnew;

    /// <summary>The vertex; throws Standard_DomainError on a new point.</summary>
    public readonly HVertex Vertex()
    {
        if (isnew)
        {
            throw new DomainErrorException();
        }
        return vtx!;
    }

    public readonly Curve2DAdaptorBase Arc() => arc!;

    public readonly double Parameter() => param;

    static Contap_ThePathPointOfTheSearch IIntStart_PathPoint<Contap_ThePathPointOfTheSearch>.Create(
        in Point3D P, double Tol, HVertex V, Curve2DAdaptorBase A, double Parameter) => new(P, Tol, V, A, Parameter);

    static Contap_ThePathPointOfTheSearch IIntStart_PathPoint<Contap_ThePathPointOfTheSearch>.Create(
        in Point3D P, double Tol, Curve2DAdaptorBase A, double Parameter) => new(P, Tol, A, Parameter);

    void IIntStart_PathPoint<Contap_ThePathPointOfTheSearch>.SetValue(in Point3D P, double Tol, HVertex V,
                                                                             Curve2DAdaptorBase A, double Parameter) =>
        SetValue(P, Tol, V, A, Parameter);

    void IIntStart_PathPoint<Contap_ThePathPointOfTheSearch>.SetValue(in Point3D P, double Tol, Curve2DAdaptorBase A,
                                                                             double Parameter) =>
        SetValue(P, Tol, A, Parameter);

    public readonly bool Equals(Contap_ThePathPointOfTheSearch other)
        => point.Equals(other.point) && tol.Equals(other.tol) && isnew == other.isnew && ReferenceEquals(vtx, other.vtx)
           && ReferenceEquals(arc, other.arc) && param.Equals(other.param);

    public override readonly bool Equals(object? obj) => obj is Contap_ThePathPointOfTheSearch other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(point, tol, isnew, vtx, arc, param);

    public static bool operator ==(in Contap_ThePathPointOfTheSearch theLeft,
                                   in Contap_ThePathPointOfTheSearch theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Contap_ThePathPointOfTheSearch theLeft,
                                   in Contap_ThePathPointOfTheSearch theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"{Constants.Format("Contap_ThePathPointOfTheSearch", point.X(), point.Y(), point.Z(), tol, param)} new {isnew}";
}
