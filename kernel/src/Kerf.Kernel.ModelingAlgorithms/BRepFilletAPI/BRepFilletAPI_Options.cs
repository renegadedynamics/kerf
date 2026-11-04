// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Fillets.Data;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Fillets.Api;

/// <summary>
/// The options of a fillet for the functional twin <see cref="BRepFilletApi.Fillet(in Shape, IReadOnlyList{Edge}, double, in MakeFilletOptions, out History)"/>
/// (decision 1139): BRepFilletAPI_MakeFillet's options under OCCT's names, with OCCT's defaults.
/// </summary>
/// <remarks>
/// An OCCT.NET addition, as <see cref="AlgoApiOptions"/> (decision 579): a struct with public fields, equatable
/// field by field (doubles as <c>double.Equals</c>). <c>new BRepFilletAPI_FilletOptions()</c> has OCCT's defaults
/// (those of the MakeFillet constructor); <c>default</c> doesn't (all zero).
/// </remarks>
internal struct MakeFilletOptions : IEquatable<MakeFilletOptions>
{
    /// <summary>OCCT's defaults.</summary>
    public MakeFilletOptions()
    {
        FShape = global::Kerf.Fillets.FilletShape.Rational;
        Tang = 1.0e-2;
        Tesp = 1.0e-4;
        T2d = 1.0e-5;
        TApp3d = 1.0e-4;
        TolApp2d = 1.0e-5;
        Fleche = 1.0e-3;
        InternalContinuity = Continuity.C1;
        AngularTolerance = 1.0e-2;
    }

    /// <summary>The type of fillet surface (the constructor's <c>FShape</c>; ChFi3d_Rational by default).</summary>
    public global::Kerf.Fillets.FilletShape FShape;

    /// <summary><c>SetParams</c>' angular tolerance (1e-2 by default).</summary>
    public double Tang;

    /// <summary><c>SetParams</c>' 3D tolerance of the spine (1e-4 by default).</summary>
    public double Tesp;

    /// <summary><c>SetParams</c>' 2D tolerance (1e-5 by default).</summary>
    public double T2d;

    /// <summary><c>SetParams</c>' 3D approximation tolerance (1e-4 by default).</summary>
    public double TApp3d;

    /// <summary><c>SetParams</c>' 2D approximation tolerance (1e-5 by default).</summary>
    public double TolApp2d;

    /// <summary><c>SetParams</c>' deflection (1e-3 by default).</summary>
    public double Fleche;

    /// <summary><c>SetContinuity</c>'s continuity of the fillet surfaces (GeomAbs_C1 by default).</summary>
    public Continuity InternalContinuity;

    /// <summary><c>SetContinuity</c>'s G1 tolerance between the fillets and their faces (1e-2 by default).</summary>
    public double AngularTolerance;

    public readonly bool Equals(MakeFilletOptions other) =>
        FShape == other.FShape && Tang.Equals(other.Tang) && Tesp.Equals(other.Tesp) && T2d.Equals(other.T2d)
        && TApp3d.Equals(other.TApp3d) && TolApp2d.Equals(other.TolApp2d) && Fleche.Equals(other.Fleche)
        && InternalContinuity == other.InternalContinuity && AngularTolerance.Equals(other.AngularTolerance);

    public override readonly bool Equals(object? obj) => obj is MakeFilletOptions other && Equals(other);

    public override readonly int GetHashCode()
    {
        HashCode hash = new();
        hash.Add(FShape);
        hash.Add(Tang);
        hash.Add(Tesp);
        hash.Add(T2d);
        hash.Add(TApp3d);
        hash.Add(TolApp2d);
        hash.Add(Fleche);
        hash.Add(InternalContinuity);
        hash.Add(AngularTolerance);
        return hash.ToHashCode();
    }

    public static bool operator ==(MakeFilletOptions left, MakeFilletOptions right) => left.Equals(right);

    public static bool operator !=(MakeFilletOptions left, MakeFilletOptions right) => !left.Equals(right);

    public override readonly string ToString() =>
        string.Create(CultureInfo.InvariantCulture,
                      $"BRepFilletAPI_FilletOptions(FShape={FShape}, Tang={Tang:R}, Tesp={Tesp:R}, T2d={T2d:R}, TApp3d={TApp3d:R}, TolApp2d={TolApp2d:R}, Fleche={Fleche:R}, InternalContinuity={InternalContinuity}, AngularTolerance={AngularTolerance:R})");
}

/// <summary>
/// The options of a chamfer for the functional twin <see cref="BRepFilletApi.Chamfer(in Shape, IReadOnlyList{Edge}, double, in MakeChamferOptions, out History)"/>
/// (decision 1139): BRepFilletAPI_MakeChamfer's one option, <c>SetMode</c>, with OCCT's default.
/// </summary>
internal struct MakeChamferOptions : IEquatable<MakeChamferOptions>
{
    /// <summary>OCCT's default.</summary>
    public MakeChamferOptions()
    {
        Mode = ChamfMode.ClassicChamfer;
    }

    /// <summary><c>SetMode</c>: the classic chamfer (the default) or a throat chamfer.</summary>
    public ChamfMode Mode;

    public readonly bool Equals(MakeChamferOptions other) => Mode == other.Mode;

    public override readonly bool Equals(object? obj) => obj is MakeChamferOptions other && Equals(other);

    public override readonly int GetHashCode() => Mode.GetHashCode();

    public static bool operator ==(MakeChamferOptions left, MakeChamferOptions right) => left.Equals(right);

    public static bool operator !=(MakeChamferOptions left, MakeChamferOptions right) => !left.Equals(right);

    public override readonly string ToString() => $"BRepFilletAPI_ChamferOptions(Mode={Mode})";
}
