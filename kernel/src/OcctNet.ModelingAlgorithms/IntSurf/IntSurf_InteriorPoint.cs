// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_InteriorPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_InteriorPoint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_InteriorPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A solution point of the intersection between an implicit and a parametric surface inside the domain: a passing
/// point of an intersection line, or a starting point of a closed line on the parametric surface. It holds the 3D
/// point, its parameters on the parametric surface and the tangents of the line in 3D and in the parametric space.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct; <c>new()</c> and <c>default</c> are the same. <see cref="SetValue"/> is an internal
/// mutator; the constructor is its twin.
/// </remarks>
public partial struct IntSurf_InteriorPoint : IEquatable<IntSurf_InteriorPoint>
{
    private gp_Pnt point;
    private double paramu;
    private double paramv;
    private gp_Vec direc;
    private gp_Vec2d direc2d;

    public IntSurf_InteriorPoint()
    {
        point = new gp_Pnt();
        paramu = 0.0;
        paramv = 0.0;
        direc = new gp_Vec();
        direc2d = new gp_Vec2d();
    }

    public IntSurf_InteriorPoint(in gp_Pnt P, double U, double V, in gp_Vec Direc, in gp_Vec2d Direc2d)
    {
        point = P;
        paramu = U;
        paramv = V;
        direc = Direc;
        direc2d = Direc2d;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetValue(in gp_Pnt P, double U, double V, in gp_Vec Direc, in gp_Vec2d Direc2d)
    {
        point = P;
        paramu = U;
        paramv = V;
        direc = Direc;
        direc2d = Direc2d;
    }

    // Queries

    /// <summary>The 3D point.</summary>
    public readonly gp_Pnt Value() => point;

    /// <summary>The parameters on the parametric surface.</summary>
    public readonly void Parameters(out double U, out double V)
    {
        U = paramu;
        V = paramv;
    }

    public readonly double UParameter() => paramu;

    public readonly double VParameter() => paramv;

    /// <summary>The tangent of the intersection in 3D.</summary>
    public readonly gp_Vec Direction() => direc;

    /// <summary>The tangent of the intersection in the parametric space of the parametric surface.</summary>
    public readonly gp_Vec2d Direction2d() => direc2d;

    public readonly bool Equals(IntSurf_InteriorPoint other)
        => point.Equals(other.point) && paramu.Equals(other.paramu) && paramv.Equals(other.paramv)
           && direc.Equals(other.direc) && direc2d.Equals(other.direc2d);

    public override readonly bool Equals(object? obj) => obj is IntSurf_InteriorPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(point, paramu, paramv, direc, direc2d);

    public static bool operator ==(in IntSurf_InteriorPoint theLeft, in IntSurf_InteriorPoint theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntSurf_InteriorPoint theLeft, in IntSurf_InteriorPoint theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => gp.Format("IntSurf_InteriorPoint", point.X(), point.Y(), point.Z(), paramu, paramv, direc.X(), direc.Y(),
                     direc.Z(), direc2d.X(), direc2d.Y());
}
