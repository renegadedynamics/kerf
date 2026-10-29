// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Sinert.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Sinert.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BRepGProp_Gauss.BRepGProp_GaussType;

namespace Occt;

/// <summary>The global properties of a face in 3D space, by Gauss integration (<see cref="BRepGProp_Gauss"/>).</summary>
public class BRepGProp_Sinert : GProp_GProps
{
    private double myEpsilon;

    /// <summary>An empty system.</summary>
    public BRepGProp_Sinert() => myEpsilon = 0.0;

    /// <summary>The global properties of the face over its natural bounds, inertia at <paramref name="SLocation"/>.</summary>
    public BRepGProp_Sinert(BRepGProp_Face S, in gp_Pnt SLocation)
    {
        SetLocation(SLocation);
        Perform(S);
    }

    /// <summary>The global properties of the face bounded by its edges (<paramref name="D"/>), inertia at <paramref name="SLocation"/>.</summary>
    public BRepGProp_Sinert(BRepGProp_Face S, BRepGProp_Domain D, in gp_Pnt SLocation)
    {
        SetLocation(SLocation);
        Perform(S, D);
    }

    /// <summary>The global properties of the face over its natural bounds, adaptively to the relative error <paramref name="Eps"/>.</summary>
    public BRepGProp_Sinert(BRepGProp_Face S, in gp_Pnt SLocation, double Eps)
    {
        SetLocation(SLocation);
        Perform(S, Eps);
    }

    /// <summary>The global properties of the face bounded by its edges, adaptively to the relative error <paramref name="Eps"/>.</summary>
    public BRepGProp_Sinert(BRepGProp_Face S, BRepGProp_Domain D, in gp_Pnt SLocation, double Eps)
    {
        SetLocation(SLocation);
        Perform(S, D, Eps);
    }

    /// <summary>Sets the reference point.</summary>
    public void SetLocation(in gp_Pnt SLocation) => loc = SLocation;

    /// <summary>Computes the global properties of the face over its natural bounds.</summary>
    public void Perform(BRepGProp_Face S)
    {
        myEpsilon = 1.0;

        var aGauss = new BRepGProp_Gauss(Sinert);
        aGauss.Compute(S, loc, ref dim, ref g, ref inertia);
    }

    /// <summary>Computes the global properties of the face bounded by its edges.</summary>
    public void Perform(BRepGProp_Face S, BRepGProp_Domain D)
    {
        myEpsilon = 1.0;

        var aGauss = new BRepGProp_Gauss(Sinert);
        aGauss.Compute(S, D, loc, ref dim, ref g, ref inertia);
    }

    /// <summary>Computes the global properties of the face over its natural bounds, adaptively; returns the relative error.</summary>
    public double Perform(BRepGProp_Face S, double Eps)
    {
        var anEmptyDomian = new BRepGProp_Domain();
        return Perform(S, anEmptyDomian, Eps);
    }

    /// <summary>Computes the global properties of the face bounded by its edges, adaptively; returns the relative error.</summary>
    public double Perform(BRepGProp_Face S, BRepGProp_Domain D, double Eps)
    {
        var aGauss = new BRepGProp_Gauss(Sinert);
        return myEpsilon = aGauss.Compute(S, D, loc, Eps, ref dim, ref g, ref inertia);
    }

    /// <summary>The relative error of the last adaptive computation, else 1.</summary>
    public double GetEpsilon() => myEpsilon;
}
