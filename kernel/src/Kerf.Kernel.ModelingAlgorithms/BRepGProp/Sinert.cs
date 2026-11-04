// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Sinert.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Sinert.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Measurement;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Measurement.MeasurementGauss.BRepGProp_GaussType;

namespace Kerf.Engine.Measurement;

/// <summary>The global properties of a face in 3D space, by Gauss integration (<see cref="MeasurementGauss"/>).</summary>
internal class Sinert : Properties
{
    private double _epsilon;

    /// <summary>An empty system.</summary>
    public Sinert() => _epsilon = 0.0;

    /// <summary>The global properties of the face over its natural bounds, inertia at <paramref name="sLocation"/>.</summary>
    public Sinert(FaceIntegrand s, in Point3D sLocation)
    {
        SetLocation(sLocation);
        Perform(s);
    }

    /// <summary>The global properties of the face bounded by its edges (<paramref name="d"/>), inertia at <paramref name="sLocation"/>.</summary>
    public Sinert(FaceIntegrand s, Domain d, in Point3D sLocation)
    {
        SetLocation(sLocation);
        Perform(s, d);
    }

    /// <summary>The global properties of the face over its natural bounds, adaptively to the relative error <paramref name="eps"/>.</summary>
    public Sinert(FaceIntegrand s, in Point3D sLocation, double eps)
    {
        SetLocation(sLocation);
        Perform(s, eps);
    }

    /// <summary>The global properties of the face bounded by its edges, adaptively to the relative error <paramref name="eps"/>.</summary>
    public Sinert(FaceIntegrand s, Domain d, in Point3D sLocation, double eps)
    {
        SetLocation(sLocation);
        Perform(s, d, eps);
    }

    /// <summary>Sets the reference point.</summary>
    public void SetLocation(in Point3D sLocation) => _loc = sLocation;

    /// <summary>Computes the global properties of the face over its natural bounds.</summary>
    public void Perform(FaceIntegrand s)
    {
        _epsilon = 1.0;

        var gauss = new MeasurementGauss(global::Kerf.Engine.Measurement.MeasurementGauss.BRepGProp_GaussType.Sinert);
        gauss.Compute(s, _loc, ref _dim, ref _g, ref _inertia);
    }

    /// <summary>Computes the global properties of the face bounded by its edges.</summary>
    public void Perform(FaceIntegrand s, Domain d)
    {
        _epsilon = 1.0;

        var gauss = new MeasurementGauss(global::Kerf.Engine.Measurement.MeasurementGauss.BRepGProp_GaussType.Sinert);
        gauss.Compute(s, d, _loc, ref _dim, ref _g, ref _inertia);
    }

    /// <summary>Computes the global properties of the face over its natural bounds, adaptively; returns the relative error.</summary>
    public double Perform(FaceIntegrand s, double eps)
    {
        var emptyDomian = new Domain();
        return Perform(s, emptyDomian, eps);
    }

    /// <summary>Computes the global properties of the face bounded by its edges, adaptively; returns the relative error.</summary>
    public double Perform(FaceIntegrand s, Domain d, double eps)
    {
        var gauss = new MeasurementGauss(global::Kerf.Engine.Measurement.MeasurementGauss.BRepGProp_GaussType.Sinert);
        return _epsilon = gauss.Compute(s, d, _loc, eps, ref _dim, ref _g, ref _inertia);
    }

    /// <summary>The relative error of the last adaptive computation, else 1.</summary>
    public double GetEpsilon() => _epsilon;
}
