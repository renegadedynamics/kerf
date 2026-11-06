// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Vinert.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Vinert.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The global properties of the region of 3D space delimited by a face and the reference point, a point
/// <c>O</c>, or a plane, by Gauss integration (<see cref="MeasurementGauss"/>). The face may be closed; without a domain
/// it is bounded by its natural (U, V) bounds, with one by its edges.
/// </summary>
/// <remarks>
/// The forms with <c>Eps</c> integrate adaptively to that relative error of the mass (non-adaptively above 0.001) and
/// return the error reached; the others use numbers of Gauss points chosen by the types of the surface and curves.
/// </remarks>
internal class Vinert : Properties
{
    private double _epsilon;

    /// <summary>An empty system.</summary>
    public Vinert() => _epsilon = 0.0;

    public Vinert(FaceIntegrand s, in Point3D vLocation, double eps)
    {
        SetLocation(vLocation);
        Perform(s, eps);
    }

    public Vinert(FaceIntegrand s, Domain d, in Point3D vLocation, double eps)
    {
        SetLocation(vLocation);
        Perform(s, d, eps);
    }

    public Vinert(FaceIntegrand s, Domain d, in Point3D vLocation)
    {
        SetLocation(vLocation);
        Perform(s, d);
    }

    public Vinert(FaceIntegrand s, in Point3D vLocation)
    {
        SetLocation(vLocation);
        Perform(s);
    }

    public Vinert(FaceIntegrand s, in Point3D o, in Point3D vLocation, double eps)
    {
        SetLocation(vLocation);
        Perform(s, o, eps);
    }

    public Vinert(FaceIntegrand s, Domain d, in Point3D o, in Point3D vLocation, double eps)
    {
        SetLocation(vLocation);
        Perform(s, d, o, eps);
    }

    public Vinert(FaceIntegrand s, in Point3D o, in Point3D vLocation)
    {
        SetLocation(vLocation);
        Perform(s, o);
    }

    public Vinert(FaceIntegrand s, Domain d, in Point3D o, in Point3D vLocation)
    {
        SetLocation(vLocation);
        Perform(s, d, o);
    }

    public Vinert(FaceIntegrand s, in Plane3D pl, in Point3D vLocation, double eps)
    {
        SetLocation(vLocation);
        Perform(s, pl, eps);
    }

    public Vinert(FaceIntegrand s, Domain d, in Plane3D pl, in Point3D vLocation, double eps)
    {
        SetLocation(vLocation);
        Perform(s, d, pl, eps);
    }

    public Vinert(FaceIntegrand s, in Plane3D pl, in Point3D vLocation)
    {
        SetLocation(vLocation);
        Perform(s, pl);
    }

    public Vinert(FaceIntegrand s, Domain d, in Plane3D pl, in Point3D vLocation)
    {
        SetLocation(vLocation);
        Perform(s, d, pl);
    }

    /// <summary>Sets the reference point.</summary>
    public void SetLocation(in Point3D vLocation) => _loc = vLocation;

    public double Perform(FaceIntegrand s, double eps)
    {
        var emptyDomain = new Domain();
        return Perform(s, emptyDomain, eps);
    }

    public double Perform(FaceIntegrand s, Domain d, double eps)
    {
        ReadOnlySpan<double> coeff = [0.0, 0.0, 0.0];
        var gauss = new MeasurementGauss(global::Kerf.Engine.Measurement.MeasurementGauss.BRepGProp_GaussType.Vinert);
        return _epsilon = gauss.Compute(s, d, _loc, eps, coeff, true, ref _dim, ref _g, ref _inertia);
    }

    public void Perform(FaceIntegrand s)
    {
        ReadOnlySpan<double> coeff = [0.0, 0.0, 0.0];
        var gauss = new MeasurementGauss(global::Kerf.Engine.Measurement.MeasurementGauss.BRepGProp_GaussType.Vinert);

        _epsilon = 1.0;
        gauss.Compute(s, _loc, coeff, true, ref _dim, ref _g, ref _inertia);
    }

    public void Perform(FaceIntegrand s, Domain d)
    {
        ReadOnlySpan<double> coeff = [0.0, 0.0, 0.0];
        var gauss = new MeasurementGauss(global::Kerf.Engine.Measurement.MeasurementGauss.BRepGProp_GaussType.Vinert);

        _epsilon = 1.0;
        gauss.Compute(s, d, _loc, coeff, true, ref _dim, ref _g, ref _inertia);
    }

    public double Perform(FaceIntegrand s, in Point3D o, double eps)
    {
        var emptyDomain = new Domain();
        return Perform(s, emptyDomain, o, eps);
    }

    public double Perform(FaceIntegrand s, Domain d, in Point3D o, double eps)
    {
        ReadOnlySpan<double> coeff = [o.X - _loc.X, o.Y - _loc.Y, o.Z - _loc.Z];

        var gauss = new MeasurementGauss(global::Kerf.Engine.Measurement.MeasurementGauss.BRepGProp_GaussType.Vinert);
        return _epsilon = gauss.Compute(s, d, _loc, eps, coeff, true, ref _dim, ref _g, ref _inertia);
    }

    public void Perform(FaceIntegrand s, in Point3D o)
    {
        var gauss = new MeasurementGauss(global::Kerf.Engine.Measurement.MeasurementGauss.BRepGProp_GaussType.Vinert);
        ReadOnlySpan<double> coeff = [o.X - _loc.X, o.Y - _loc.Y, o.Z - _loc.Z];

        _epsilon = 1.0;
        gauss.Compute(s, _loc, coeff, true, ref _dim, ref _g, ref _inertia);
    }

    public void Perform(FaceIntegrand s, Domain d, in Point3D o)
    {
        var gauss = new MeasurementGauss(global::Kerf.Engine.Measurement.MeasurementGauss.BRepGProp_GaussType.Vinert);
        ReadOnlySpan<double> coeff = [o.X - _loc.X, o.Y - _loc.Y, o.Z - _loc.Z];

        _epsilon = 1.0;
        gauss.Compute(s, d, _loc, coeff, true, ref _dim, ref _g, ref _inertia);
    }

    public double Perform(FaceIntegrand s, in Plane3D pl, double eps)
    {
        var emptyDomain = new Domain();
        return Perform(s, emptyDomain, pl, eps);
    }

    public double Perform(FaceIntegrand s, Domain d, in Plane3D pl, double eps)
    {
        Span<double> coeff = stackalloc double[4];
        pl.Coefficients(out coeff[0], out coeff[1], out coeff[2], out coeff[3]);
        coeff[3] = coeff[3] - coeff[0] * _loc.X - coeff[1] * _loc.Y - coeff[2] * _loc.Z;

        var gauss = new MeasurementGauss(global::Kerf.Engine.Measurement.MeasurementGauss.BRepGProp_GaussType.Vinert);
        return _epsilon = gauss.Compute(s, d, _loc, eps, coeff, false, ref _dim, ref _g, ref _inertia);
    }

    public void Perform(FaceIntegrand s, in Plane3D pl)
    {
        var gauss = new MeasurementGauss(global::Kerf.Engine.Measurement.MeasurementGauss.BRepGProp_GaussType.Vinert);
        Span<double> coeff = stackalloc double[4];
        pl.Coefficients(out coeff[0], out coeff[1], out coeff[2], out coeff[3]);
        coeff[3] = coeff[3] - coeff[0] * _loc.X - coeff[1] * _loc.Y - coeff[2] * _loc.Z;

        _epsilon = 1.0;
        gauss.Compute(s, _loc, coeff, false, ref _dim, ref _g, ref _inertia);
    }

    public void Perform(FaceIntegrand s, Domain d, in Plane3D pl)
    {
        var gauss = new MeasurementGauss(global::Kerf.Engine.Measurement.MeasurementGauss.BRepGProp_GaussType.Vinert);
        Span<double> coeff = stackalloc double[4];
        pl.Coefficients(out coeff[0], out coeff[1], out coeff[2], out coeff[3]);
        coeff[3] = coeff[3] - coeff[0] * _loc.X - coeff[1] * _loc.Y - coeff[2] * _loc.Z;

        _epsilon = 1.0;
        gauss.Compute(s, d, _loc, coeff, false, ref _dim, ref _g, ref _inertia);
    }

    /// <summary>The relative error of the last adaptive computation, else 1.</summary>
    public double GetEpsilon() => _epsilon;
}
