// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PntOn2S.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PntOn2S.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_PntOn2S.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// An intersection point between two surfaces: its 3D coordinates and its parameters (U1, V1) on the first surface
/// and (U2, V2) on the second.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct: OCCT copies it into lines and vertices. <c>new()</c> and <c>default</c> are the
/// same (the origin, zero parameters). The <c>SetValue</c> overloads are internal mutators (translation rules,
/// section 3); <see cref="WithValue(in Point3D, double, double, double, double)"/> is the functional twin of the one
/// that sets everything.
/// </remarks>
public partial struct IntSurf_PntOn2S : IEquatable<IntSurf_PntOn2S>
{
    private Point3D pt;
    private double u1;
    private double v1;
    private double u2;
    private double v2;

    /// <summary>Empty constructor: the origin, zero parameters.</summary>
    public IntSurf_PntOn2S()
    {
        pt = new Point3D(0, 0, 0);
        u1 = 0;
        v1 = 0;
        u2 = 0;
        v2 = 0;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the point in 3D space.</summary>
    internal void SetValue(in Point3D Pt) => pt = Pt;

    /// <summary>Sets the point in 3D space and in the parametric space of one of the surfaces.</summary>
    internal void SetValue(in Point3D Pt, bool OnFirst, double U, double V)
    {
        pt = Pt;
        if (OnFirst)
        {
            u1 = U;
            v1 = V;
        }
        else
        {
            u2 = U;
            v2 = V;
        }
    }

    /// <summary>Sets the point in 3D space and in the parametric space of each surface.</summary>
    internal void SetValue(in Point3D Pt, double U1, double V1, double U2, double V2)
    {
        pt = Pt;
        u1 = U1;
        v1 = V1;
        u2 = U2;
        v2 = V2;
    }

    /// <summary>Sets the point in the parametric space of one of the surfaces.</summary>
    internal void SetValue(bool OnFirst, double U, double V)
    {
        if (OnFirst)
        {
            u1 = U;
            v1 = V;
        }
        else
        {
            u2 = U;
            v2 = V;
        }
    }

    /// <summary>Sets the point in the parametric space of each surface.</summary>
    internal void SetValue(double U1, double V1, double U2, double V2)
    {
        u1 = U1;
        v1 = V1;
        u2 = U2;
        v2 = V2;
    }

    /// <summary>A copy with the point and both pairs of parameters set.</summary>
    public readonly IntSurf_PntOn2S WithValue(in Point3D Pt, double U1, double V1, double U2, double V2)
    {
        IntSurf_PntOn2S aCopy = this;
        aCopy.SetValue(Pt, U1, V1, U2, V2);
        return aCopy;
    }

    // Queries

    /// <summary>The point in 3D space.</summary>
    public readonly Point3D Value() => pt;

    /// <summary>The point in the parametric space of one of the surfaces.</summary>
    public readonly Point2D ValueOnSurface(bool OnFirst)
    {
        Point2D PointOnSurf = new();
        if (OnFirst)
        {
            PointOnSurf.SetCoord(u1, v1);
        }
        else
        {
            PointOnSurf.SetCoord(u2, v2);
        }
        return PointOnSurf;
    }

    /// <summary>The parameters on the first surface.</summary>
    public readonly void ParametersOnS1(out double U1, out double V1)
    {
        U1 = u1;
        V1 = v1;
    }

    /// <summary>The parameters on the second surface.</summary>
    public readonly void ParametersOnS2(out double U2, out double V2)
    {
        U2 = u2;
        V2 = v2;
    }

    /// <summary>The parameters on one of the surfaces.</summary>
    public readonly void ParametersOnSurface(bool OnFirst, out double U, out double V)
    {
        if (OnFirst)
        {
            U = u1;
            V = v1;
        }
        else
        {
            U = u2;
            V = v2;
        }
    }

    /// <summary>The parameters on both surfaces.</summary>
    public readonly void Parameters(out double U1, out double V1, out double U2, out double V2)
    {
        U1 = u1;
        V1 = v1;
        U2 = u2;
        V2 = v2;
    }

    /// <summary>
    /// True if the 3D points are within <paramref name="theTol3D"/> and, unless <paramref name="theTol2D"/> is
    /// negative, the parameters on each surface within <paramref name="theTol2D"/>.
    /// </summary>
    public readonly bool IsSame(in IntSurf_PntOn2S theOtherPoint, double theTol3D = 0.0, double theTol2D = -1.0)
    {
        if (pt.SquareDistance(theOtherPoint.Value()) > theTol3D * theTol3D)
        {
            return false;
        }

        if (theTol2D < 0.0)
        { // We need not compare 2D-coordinates of the points
            return true;
        }

        theOtherPoint.Parameters(out double aU1, out double aV1, out double aU2, out double aV2);

        Point2D aP1 = new(u1, v1), aP2 = new(aU1, aV1);

        if (!aP1.IsEqual(aP2, theTol2D))
        {
            return false;
        }

        aP1.SetCoord(u2, v2);
        aP2.SetCoord(aU2, aV2);

        return aP1.IsEqual(aP2, theTol2D);
    }

    public readonly bool Equals(IntSurf_PntOn2S other)
        => pt.Equals(other.pt) && u1.Equals(other.u1) && v1.Equals(other.v1) && u2.Equals(other.u2)
           && v2.Equals(other.v2);

    public override readonly bool Equals(object? obj) => obj is IntSurf_PntOn2S other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pt, u1, v1, u2, v2);

    public static bool operator ==(in IntSurf_PntOn2S theLeft, in IntSurf_PntOn2S theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntSurf_PntOn2S theLeft, in IntSurf_PntOn2S theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => Constants.Format("IntSurf_PntOn2S", pt.X(), pt.Y(), pt.Z(), u1, v1, u2, v2);
}
