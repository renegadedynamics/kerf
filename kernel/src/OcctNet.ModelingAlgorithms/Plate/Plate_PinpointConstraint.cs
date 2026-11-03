// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_PinpointConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_PinpointConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_PinpointConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

namespace Occt;

/// <summary>
/// A pinpoint constraint of a plate: the derivative (<c>Idu</c>, <c>Idv</c>) of the plate's deformation at a 2D point
/// takes an imposed value (the value itself for order 0).
/// </summary>
/// <remarks>A value type (rules, section 3); <c>default</c> is OCCT's default constraint (all zero).</remarks>
public readonly struct Plate_PinpointConstraint : IEquatable<Plate_PinpointConstraint>
{
    private readonly Xyz value;
    private readonly Xy pnt2d;
    private readonly int idu;
    private readonly int idv;

    public Plate_PinpointConstraint()
    {
        pnt2d = new Xy(0, 0);
        value = new Xyz(0, 0, 0);
        idu = 0;
        idv = 0;
    }

    public Plate_PinpointConstraint(in Xy point2d, in Xyz ImposedValue, int iu = 0, int iv = 0)
    {
        pnt2d = point2d;
        value = ImposedValue;
        idu = iu;
        idv = iv;
    }

    public Xy Pnt2d() => pnt2d;

    public int Idu() => idu;

    public int Idv() => idv;

    public Xyz Value() => value;

    public bool Equals(Plate_PinpointConstraint other)
        => value.Equals(other.value) && pnt2d.Equals(other.pnt2d) && idu == other.idu && idv == other.idv;

    public override bool Equals(object? obj) => obj is Plate_PinpointConstraint other && Equals(other);

    public override int GetHashCode() => HashCode.Combine(value, pnt2d, idu, idv);

    public static bool operator ==(in Plate_PinpointConstraint theLeft, in Plate_PinpointConstraint theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Plate_PinpointConstraint theLeft, in Plate_PinpointConstraint theRight) => !theLeft.Equals(theRight);

    public override string ToString() => $"Plate_PinpointConstraint({pnt2d}, {value}, {idu}, {idv})";
}
