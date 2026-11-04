// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Factors.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Factors.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>Class for using units variables: the length, plane angle and solid angle factors of a STEP file.</summary>
/// <remarks>
/// A C++ value class copied by value, a struct here (translation rules, section 3): mutators internal, with public
/// twins. Its default is OCCT's (all factors 1): write <c>new StepData_Factors()</c>, never <c>default</c>.
/// </remarks>
internal struct Factors
{
    private double _lengthFactor;
    private double _planeAngleFactor;
    private double _solidAngleFactor;
    private double _factRD;
    private double _factDR;
    private double _cascadeUnit;

    /// <summary>Constructor.</summary>
    public Factors()
    {
        _lengthFactor = 1.0;
        _planeAngleFactor = 1.0;
        _solidAngleFactor = 1.0;
        _factRD = 1.0;
        _factDR = 1.0;
        _cascadeUnit = 1.0;
    }

    /// <summary>Initializes the 3 factors for the conversion of units.</summary>
    internal void InitializeFactors(double lengthFactor, double planeAngleFactor, double solidAngleFactor)
    {
        _lengthFactor = lengthFactor;
        _planeAngleFactor = planeAngleFactor;
        _solidAngleFactor = solidAngleFactor;
        _factRD = 1.0 / planeAngleFactor;
        _factDR = planeAngleFactor;
    }

    /// <summary>A copy with the 3 factors for the conversion of units initialized.</summary>
    public readonly Factors InitializedFactors(double lengthFactor, double planeAngleFactor, double solidAngleFactor)
    {
        Factors copy = this;
        copy.InitializeFactors(lengthFactor, planeAngleFactor, solidAngleFactor);
        return copy;
    }

    /// <summary>Sets length unit for current transfer process.</summary>
    internal void SetCascadeUnit(double unit)
    {
        _cascadeUnit = unit;
    }

    /// <summary>A copy with the length unit for current transfer process set.</summary>
    public readonly Factors WithCascadeUnit(double unit)
    {
        Factors copy = this;
        copy.SetCascadeUnit(unit);
        return copy;
    }

    /// <summary>Returns length unit for current transfer process (mm by default).</summary>
    public readonly double CascadeUnit()
    {
        return _cascadeUnit;
    }

    /// <summary>Returns transient length factor for scaling of shapes at one stage of transfer process.</summary>
    public readonly double LengthFactor()
    {
        return _lengthFactor;
    }

    /// <summary>Returns transient plane angle factor for conversion of angles at one stage of transfer process.</summary>
    public readonly double PlaneAngleFactor()
    {
        return _planeAngleFactor;
    }

    /// <summary>Returns transient solid angle factor for conversion of angles at one stage of transfer process.</summary>
    public readonly double SolidAngleFactor()
    {
        return _solidAngleFactor;
    }

    /// <summary>Returns transient factor radian degree for conversion of angles at one stage of transfer process.</summary>
    public readonly double FactorRadianDegree()
    {
        return _factRD;
    }

    /// <summary>Returns transient factor degree radian for conversion of angles at one stage of transfer process.</summary>
    public readonly double FactorDegreeRadian()
    {
        return _factDR;
    }
}
