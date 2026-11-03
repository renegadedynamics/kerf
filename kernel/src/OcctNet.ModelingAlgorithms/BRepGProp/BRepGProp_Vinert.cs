// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Vinert.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Vinert.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Geometry;
using static Occt.BRepGProp_Gauss.BRepGProp_GaussType;

namespace Occt;

/// <summary>
/// The global properties of the region of 3D space delimited by a face and the reference point, a point
/// <c>O</c>, or a plane, by Gauss integration (<see cref="BRepGProp_Gauss"/>). The face may be closed; without a domain
/// it is bounded by its natural (U, V) bounds, with one by its edges.
/// </summary>
/// <remarks>
/// The forms with <c>Eps</c> integrate adaptively to that relative error of the mass (non-adaptively above 0.001) and
/// return the error reached; the others use numbers of Gauss points chosen by the types of the surface and curves.
/// </remarks>
public class BRepGProp_Vinert : Properties
{
    private double myEpsilon;

    /// <summary>An empty system.</summary>
    public BRepGProp_Vinert() => myEpsilon = 0.0;

    public BRepGProp_Vinert(BRepGProp_Face S, in Point3D VLocation, double Eps)
    {
        SetLocation(VLocation);
        Perform(S, Eps);
    }

    public BRepGProp_Vinert(BRepGProp_Face S, BRepGProp_Domain D, in Point3D VLocation, double Eps)
    {
        SetLocation(VLocation);
        Perform(S, D, Eps);
    }

    public BRepGProp_Vinert(BRepGProp_Face S, BRepGProp_Domain D, in Point3D VLocation)
    {
        SetLocation(VLocation);
        Perform(S, D);
    }

    public BRepGProp_Vinert(BRepGProp_Face S, in Point3D VLocation)
    {
        SetLocation(VLocation);
        Perform(S);
    }

    public BRepGProp_Vinert(BRepGProp_Face S, in Point3D O, in Point3D VLocation, double Eps)
    {
        SetLocation(VLocation);
        Perform(S, O, Eps);
    }

    public BRepGProp_Vinert(BRepGProp_Face S, BRepGProp_Domain D, in Point3D O, in Point3D VLocation, double Eps)
    {
        SetLocation(VLocation);
        Perform(S, D, O, Eps);
    }

    public BRepGProp_Vinert(BRepGProp_Face S, in Point3D O, in Point3D VLocation)
    {
        SetLocation(VLocation);
        Perform(S, O);
    }

    public BRepGProp_Vinert(BRepGProp_Face S, BRepGProp_Domain D, in Point3D O, in Point3D VLocation)
    {
        SetLocation(VLocation);
        Perform(S, D, O);
    }

    public BRepGProp_Vinert(BRepGProp_Face S, in Plane3D Pl, in Point3D VLocation, double Eps)
    {
        SetLocation(VLocation);
        Perform(S, Pl, Eps);
    }

    public BRepGProp_Vinert(BRepGProp_Face S, BRepGProp_Domain D, in Plane3D Pl, in Point3D VLocation, double Eps)
    {
        SetLocation(VLocation);
        Perform(S, D, Pl, Eps);
    }

    public BRepGProp_Vinert(BRepGProp_Face S, in Plane3D Pl, in Point3D VLocation)
    {
        SetLocation(VLocation);
        Perform(S, Pl);
    }

    public BRepGProp_Vinert(BRepGProp_Face S, BRepGProp_Domain D, in Plane3D Pl, in Point3D VLocation)
    {
        SetLocation(VLocation);
        Perform(S, D, Pl);
    }

    /// <summary>Sets the reference point.</summary>
    public void SetLocation(in Point3D VLocation) => _loc = VLocation;

    public double Perform(BRepGProp_Face S, double Eps)
    {
        var anEmptyDomain = new BRepGProp_Domain();
        return Perform(S, anEmptyDomain, Eps);
    }

    public double Perform(BRepGProp_Face S, BRepGProp_Domain D, double Eps)
    {
        ReadOnlySpan<double> aCoeff = [0.0, 0.0, 0.0];
        var aGauss = new BRepGProp_Gauss(Vinert);
        return myEpsilon = aGauss.Compute(S, D, _loc, Eps, aCoeff, true, ref _dim, ref _g, ref _inertia);
    }

    public void Perform(BRepGProp_Face S)
    {
        ReadOnlySpan<double> aCoeff = [0.0, 0.0, 0.0];
        var aGauss = new BRepGProp_Gauss(Vinert);

        myEpsilon = 1.0;
        aGauss.Compute(S, _loc, aCoeff, true, ref _dim, ref _g, ref _inertia);
    }

    public void Perform(BRepGProp_Face S, BRepGProp_Domain D)
    {
        ReadOnlySpan<double> aCoeff = [0.0, 0.0, 0.0];
        var aGauss = new BRepGProp_Gauss(Vinert);

        myEpsilon = 1.0;
        aGauss.Compute(S, D, _loc, aCoeff, true, ref _dim, ref _g, ref _inertia);
    }

    public double Perform(BRepGProp_Face S, in Point3D O, double Eps)
    {
        var anEmptyDomain = new BRepGProp_Domain();
        return Perform(S, anEmptyDomain, O, Eps);
    }

    public double Perform(BRepGProp_Face S, BRepGProp_Domain D, in Point3D O, double Eps)
    {
        ReadOnlySpan<double> aCoeff = [O.X() - _loc.X(), O.Y() - _loc.Y(), O.Z() - _loc.Z()];

        var aGauss = new BRepGProp_Gauss(Vinert);
        return myEpsilon = aGauss.Compute(S, D, _loc, Eps, aCoeff, true, ref _dim, ref _g, ref _inertia);
    }

    public void Perform(BRepGProp_Face S, in Point3D O)
    {
        var aGauss = new BRepGProp_Gauss(Vinert);
        ReadOnlySpan<double> aCoeff = [O.X() - _loc.X(), O.Y() - _loc.Y(), O.Z() - _loc.Z()];

        myEpsilon = 1.0;
        aGauss.Compute(S, _loc, aCoeff, true, ref _dim, ref _g, ref _inertia);
    }

    public void Perform(BRepGProp_Face S, BRepGProp_Domain D, in Point3D O)
    {
        var aGauss = new BRepGProp_Gauss(Vinert);
        ReadOnlySpan<double> aCoeff = [O.X() - _loc.X(), O.Y() - _loc.Y(), O.Z() - _loc.Z()];

        myEpsilon = 1.0;
        aGauss.Compute(S, D, _loc, aCoeff, true, ref _dim, ref _g, ref _inertia);
    }

    public double Perform(BRepGProp_Face S, in Plane3D Pl, double Eps)
    {
        var anEmptyDomain = new BRepGProp_Domain();
        return Perform(S, anEmptyDomain, Pl, Eps);
    }

    public double Perform(BRepGProp_Face S, BRepGProp_Domain D, in Plane3D Pl, double Eps)
    {
        Span<double> aCoeff = stackalloc double[4];
        Pl.Coefficients(out aCoeff[0], out aCoeff[1], out aCoeff[2], out aCoeff[3]);
        aCoeff[3] = aCoeff[3] - aCoeff[0] * _loc.X() - aCoeff[1] * _loc.Y() - aCoeff[2] * _loc.Z();

        var aGauss = new BRepGProp_Gauss(Vinert);
        return myEpsilon = aGauss.Compute(S, D, _loc, Eps, aCoeff, false, ref _dim, ref _g, ref _inertia);
    }

    public void Perform(BRepGProp_Face S, in Plane3D Pl)
    {
        var aGauss = new BRepGProp_Gauss(Vinert);
        Span<double> aCoeff = stackalloc double[4];
        Pl.Coefficients(out aCoeff[0], out aCoeff[1], out aCoeff[2], out aCoeff[3]);
        aCoeff[3] = aCoeff[3] - aCoeff[0] * _loc.X() - aCoeff[1] * _loc.Y() - aCoeff[2] * _loc.Z();

        myEpsilon = 1.0;
        aGauss.Compute(S, _loc, aCoeff, false, ref _dim, ref _g, ref _inertia);
    }

    public void Perform(BRepGProp_Face S, BRepGProp_Domain D, in Plane3D Pl)
    {
        var aGauss = new BRepGProp_Gauss(Vinert);
        Span<double> aCoeff = stackalloc double[4];
        Pl.Coefficients(out aCoeff[0], out aCoeff[1], out aCoeff[2], out aCoeff[3]);
        aCoeff[3] = aCoeff[3] - aCoeff[0] * _loc.X() - aCoeff[1] * _loc.Y() - aCoeff[2] * _loc.Z();

        myEpsilon = 1.0;
        aGauss.Compute(S, D, _loc, aCoeff, false, ref _dim, ref _g, ref _inertia);
    }

    /// <summary>The relative error of the last adaptive computation, else 1.</summary>
    public double GetEpsilon() => myEpsilon;
}
