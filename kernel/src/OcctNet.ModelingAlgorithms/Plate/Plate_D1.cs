// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_D1.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_D1.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_D1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

namespace Occt;

/// <summary>The first derivatives (DU, DV) of a surface at a point, as the G1 constraints of a plate take them.</summary>
/// <remarks>A value type (rules, section 3). The fields are internal, as OCCT makes them visible to its friends
/// Plate_GtoCConstraint and Plate_FreeGtoCConstraint (which transform a copy in place).</remarks>
public struct Plate_D1 : IEquatable<Plate_D1>
{
    internal Xyz Du;
    internal Xyz Dv;

    public Plate_D1(in Xyz du, in Xyz dv)
    {
        Du = du;
        Dv = dv;
    }

    public Plate_D1(in Plate_D1 @ref)
    {
        Du = @ref.Du;
        Dv = @ref.Dv;
    }

    public readonly Xyz DU() => Du;

    public readonly Xyz DV() => Dv;

    public readonly bool Equals(Plate_D1 other) => Du.Equals(other.Du) && Dv.Equals(other.Dv);

    public override readonly bool Equals(object? obj) => obj is Plate_D1 other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(Du, Dv);

    public static bool operator ==(in Plate_D1 theLeft, in Plate_D1 theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Plate_D1 theLeft, in Plate_D1 theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"Plate_D1({Du}, {Dv})";
}
