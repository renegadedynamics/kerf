// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;

namespace Occt;

/// <summary>
/// The options of a fillet for the functional twin <see cref="BRepFilletAPI.Fillet(in Shape, IReadOnlyList{Edge}, double, in BRepFilletAPI_FilletOptions, out History)"/>
/// (decision 1139): BRepFilletAPI_MakeFillet's options under OCCT's names, with OCCT's defaults.
/// </summary>
/// <remarks>
/// An OCCT.NET addition, as <see cref="BRepAlgoAPI_Options"/> (decision 579): a struct with public fields, equatable
/// field by field (doubles as <c>double.Equals</c>). <c>new BRepFilletAPI_FilletOptions()</c> has OCCT's defaults
/// (those of the MakeFillet constructor); <c>default</c> doesn't (all zero).
/// </remarks>
public struct BRepFilletAPI_FilletOptions : IEquatable<BRepFilletAPI_FilletOptions>
{
    /// <summary>OCCT's defaults.</summary>
    public BRepFilletAPI_FilletOptions()
    {
        FShape = ChFi3d_FilletShape.ChFi3d_Rational;
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
    public ChFi3d_FilletShape FShape;

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

    public readonly bool Equals(BRepFilletAPI_FilletOptions theOther) =>
        FShape == theOther.FShape && Tang.Equals(theOther.Tang) && Tesp.Equals(theOther.Tesp) && T2d.Equals(theOther.T2d)
        && TApp3d.Equals(theOther.TApp3d) && TolApp2d.Equals(theOther.TolApp2d) && Fleche.Equals(theOther.Fleche)
        && InternalContinuity == theOther.InternalContinuity && AngularTolerance.Equals(theOther.AngularTolerance);

    public override readonly bool Equals(object? theObj) => theObj is BRepFilletAPI_FilletOptions anOther && Equals(anOther);

    public override readonly int GetHashCode()
    {
        HashCode aHash = new();
        aHash.Add(FShape);
        aHash.Add(Tang);
        aHash.Add(Tesp);
        aHash.Add(T2d);
        aHash.Add(TApp3d);
        aHash.Add(TolApp2d);
        aHash.Add(Fleche);
        aHash.Add(InternalContinuity);
        aHash.Add(AngularTolerance);
        return aHash.ToHashCode();
    }

    public static bool operator ==(BRepFilletAPI_FilletOptions theLeft, BRepFilletAPI_FilletOptions theRight) => theLeft.Equals(theRight);

    public static bool operator !=(BRepFilletAPI_FilletOptions theLeft, BRepFilletAPI_FilletOptions theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() =>
        string.Create(CultureInfo.InvariantCulture,
                      $"BRepFilletAPI_FilletOptions(FShape={FShape}, Tang={Tang:R}, Tesp={Tesp:R}, T2d={T2d:R}, TApp3d={TApp3d:R}, TolApp2d={TolApp2d:R}, Fleche={Fleche:R}, InternalContinuity={InternalContinuity}, AngularTolerance={AngularTolerance:R})");
}

/// <summary>
/// The options of a chamfer for the functional twin <see cref="BRepFilletAPI.Chamfer(in Shape, IReadOnlyList{Edge}, double, in BRepFilletAPI_ChamferOptions, out History)"/>
/// (decision 1139): BRepFilletAPI_MakeChamfer's one option, <c>SetMode</c>, with OCCT's default.
/// </summary>
public struct BRepFilletAPI_ChamferOptions : IEquatable<BRepFilletAPI_ChamferOptions>
{
    /// <summary>OCCT's default.</summary>
    public BRepFilletAPI_ChamferOptions()
    {
        Mode = ChFiDS_ChamfMode.ChFiDS_ClassicChamfer;
    }

    /// <summary><c>SetMode</c>: the classic chamfer (the default) or a throat chamfer.</summary>
    public ChFiDS_ChamfMode Mode;

    public readonly bool Equals(BRepFilletAPI_ChamferOptions theOther) => Mode == theOther.Mode;

    public override readonly bool Equals(object? theObj) => theObj is BRepFilletAPI_ChamferOptions anOther && Equals(anOther);

    public override readonly int GetHashCode() => Mode.GetHashCode();

    public static bool operator ==(BRepFilletAPI_ChamferOptions theLeft, BRepFilletAPI_ChamferOptions theRight) => theLeft.Equals(theRight);

    public static bool operator !=(BRepFilletAPI_ChamferOptions theLeft, BRepFilletAPI_ChamferOptions theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"BRepFilletAPI_ChamferOptions(Mode={Mode})";
}
