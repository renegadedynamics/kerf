// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_D3.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_D3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

namespace Occt;

/// <summary>The third derivatives (DUUU, DUUV, DUVV, DVVV) of a surface at a point, as the G3 constraints of a plate
/// take them.</summary>
/// <remarks>A value type (rules, section 3). OCCT has no accessors: only its friends Plate_GtoCConstraint and
/// Plate_FreeGtoCConstraint read the fields, which are internal here.</remarks>
public struct Plate_D3 : IEquatable<Plate_D3>
{
    internal Xyz Duuu;
    internal Xyz Duuv;
    internal Xyz Duvv;
    internal Xyz Dvvv;

    public Plate_D3(in Xyz duuu, in Xyz duuv, in Xyz duvv, in Xyz dvvv)
    {
        Duuu = duuu;
        Duuv = duuv;
        Duvv = duvv;
        Dvvv = dvvv;
    }

    public Plate_D3(in Plate_D3 @ref)
    {
        Duuu = @ref.Duuu;
        Duuv = @ref.Duuv;
        Duvv = @ref.Duvv;
        Dvvv = @ref.Dvvv;
    }

    public readonly bool Equals(Plate_D3 other)
        => Duuu.Equals(other.Duuu) && Duuv.Equals(other.Duuv) && Duvv.Equals(other.Duvv) && Dvvv.Equals(other.Dvvv);

    public override readonly bool Equals(object? obj) => obj is Plate_D3 other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(Duuu, Duuv, Duvv, Dvvv);

    public static bool operator ==(in Plate_D3 theLeft, in Plate_D3 theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Plate_D3 theLeft, in Plate_D3 theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"Plate_D3({Duuu}, {Duuv}, {Duvv}, {Dvvv})";
}
