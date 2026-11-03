// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_D2.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_D2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

namespace Occt;

/// <summary>The second derivatives (DUU, DUV, DVV) of a surface at a point, as the G2 constraints of a plate take them.</summary>
/// <remarks>A value type (rules, section 3). OCCT has no accessors: only its friends Plate_GtoCConstraint and
/// Plate_FreeGtoCConstraint read the fields, which are internal here.</remarks>
public struct Plate_D2 : IEquatable<Plate_D2>
{
    internal Xyz Duu;
    internal Xyz Duv;
    internal Xyz Dvv;

    public Plate_D2(in Xyz duu, in Xyz duv, in Xyz dvv)
    {
        Duu = duu;
        Duv = duv;
        Dvv = dvv;
    }

    public Plate_D2(in Plate_D2 @ref)
    {
        Duu = @ref.Duu;
        Duv = @ref.Duv;
        Dvv = @ref.Dvv;
    }

    public readonly bool Equals(Plate_D2 other) => Duu.Equals(other.Duu) && Duv.Equals(other.Duv) && Dvv.Equals(other.Dvv);

    public override readonly bool Equals(object? obj) => obj is Plate_D2 other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(Duu, Duv, Dvv);

    public static bool operator ==(in Plate_D2 theLeft, in Plate_D2 theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Plate_D2 theLeft, in Plate_D2 theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"Plate_D2({Duu}, {Duv}, {Dvv})";
}
