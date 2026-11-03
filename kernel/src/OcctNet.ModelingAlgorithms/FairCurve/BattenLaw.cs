// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_BattenLaw.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_BattenLaw.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_BattenLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Construction.Fairing;

/// <summary>The height of a batten along its neutral fibre: linear, from its height at the middle and its slope.</summary>
/// <remarks>
/// OCCT copies this class by value (the energies and distributions each keep their own copy of the law they are
/// given); the port copies explicitly with the copy constructor at each of those sites (decision 858).
/// </remarks>
public class BattenLaw : Function
{
    private double _middleHeigth;
    private double _geometricSlope;
    private double _lengthSliding;

    /// <summary>
    /// A linear batten of height <paramref name="heigth"/> at the middle, geometric slope <paramref name="slope"/>
    /// and active length <paramref name="sliding"/> without extension.
    /// </summary>
    public BattenLaw(double heigth, double slope, double sliding)
    {
        _middleHeigth = heigth;
        _geometricSlope = slope;
        _lengthSliding = sliding;
    }

    /// <summary>A copy of <paramref name="other"/> (OCCT's implicit copy constructor).</summary>
    public BattenLaw(BattenLaw other)
    {
        _middleHeigth = other._middleHeigth;
        _geometricSlope = other._geometricSlope;
        _lengthSliding = other._lengthSliding;
    }

    /// <summary>Changes the sliding.</summary>
    public void SetSliding(double sliding)
    {
        _lengthSliding = sliding;
    }

    /// <summary>Changes the height at the middle point.</summary>
    public void SetHeigth(double heigth)
    {
        _middleHeigth = heigth;
    }

    /// <summary>Changes the geometric slope.</summary>
    public void SetSlope(double slope)
    {
        _geometricSlope = slope;
    }

    /// <summary>The height at parameter <paramref name="t"/> of the neutral fibre; false if it is not positive.</summary>
    public override bool Value(double t, out double tHeigth)
    {
        tHeigth = _middleHeigth + (t - 0.5) * _geometricSlope * _lengthSliding;
        return (tHeigth > 0);
    }
}
