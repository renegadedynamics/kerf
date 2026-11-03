// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_BattenLaw.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_BattenLaw.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_BattenLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>The height of a batten along its neutral fibre: linear, from its height at the middle and its slope.</summary>
/// <remarks>
/// OCCT copies this class by value (the energies and distributions each keep their own copy of the law they are
/// given); the port copies explicitly with the copy constructor at each of those sites (decision 858).
/// </remarks>
public class FairCurve_BattenLaw : Function
{
    private double MiddleHeigth;
    private double GeometricSlope;
    private double LengthSliding;

    /// <summary>
    /// A linear batten of height <paramref name="Heigth"/> at the middle, geometric slope <paramref name="Slope"/>
    /// and active length <paramref name="Sliding"/> without extension.
    /// </summary>
    public FairCurve_BattenLaw(double Heigth, double Slope, double Sliding)
    {
        MiddleHeigth = Heigth;
        GeometricSlope = Slope;
        LengthSliding = Sliding;
    }

    /// <summary>A copy of <paramref name="theOther"/> (OCCT's implicit copy constructor).</summary>
    public FairCurve_BattenLaw(FairCurve_BattenLaw theOther)
    {
        MiddleHeigth = theOther.MiddleHeigth;
        GeometricSlope = theOther.GeometricSlope;
        LengthSliding = theOther.LengthSliding;
    }

    /// <summary>Changes the sliding.</summary>
    public void SetSliding(double Sliding)
    {
        LengthSliding = Sliding;
    }

    /// <summary>Changes the height at the middle point.</summary>
    public void SetHeigth(double Heigth)
    {
        MiddleHeigth = Heigth;
    }

    /// <summary>Changes the geometric slope.</summary>
    public void SetSlope(double Slope)
    {
        GeometricSlope = Slope;
    }

    /// <summary>The height at parameter <paramref name="T"/> of the neutral fibre; false if it is not positive.</summary>
    public override bool Value(double T, out double THeigth)
    {
        THeigth = MiddleHeigth + (T - 0.5) * GeometricSlope * LengthSliding;
        return (THeigth > 0);
    }
}
