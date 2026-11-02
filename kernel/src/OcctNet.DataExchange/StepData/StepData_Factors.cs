// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Factors.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_Factors.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Class for using units variables: the length, plane angle and solid angle factors of a STEP file.</summary>
/// <remarks>
/// A C++ value class copied by value, a struct here (translation rules, section 3): mutators internal, with public
/// twins. Its default is OCCT's (all factors 1): write <c>new StepData_Factors()</c>, never <c>default</c>.
/// </remarks>
public struct StepData_Factors
{
    private double myLengthFactor;
    private double myPlaneAngleFactor;
    private double mySolidAngleFactor;
    private double myFactRD;
    private double myFactDR;
    private double myCascadeUnit;

    /// <summary>Constructor.</summary>
    public StepData_Factors()
    {
        myLengthFactor = 1.0;
        myPlaneAngleFactor = 1.0;
        mySolidAngleFactor = 1.0;
        myFactRD = 1.0;
        myFactDR = 1.0;
        myCascadeUnit = 1.0;
    }

    /// <summary>Initializes the 3 factors for the conversion of units.</summary>
    internal void InitializeFactors(double theLengthFactor, double thePlaneAngleFactor, double theSolidAngleFactor)
    {
        myLengthFactor = theLengthFactor;
        myPlaneAngleFactor = thePlaneAngleFactor;
        mySolidAngleFactor = theSolidAngleFactor;
        myFactRD = 1.0 / thePlaneAngleFactor;
        myFactDR = thePlaneAngleFactor;
    }

    /// <summary>A copy with the 3 factors for the conversion of units initialized.</summary>
    public readonly StepData_Factors InitializedFactors(double theLengthFactor, double thePlaneAngleFactor, double theSolidAngleFactor)
    {
        StepData_Factors aCopy = this;
        aCopy.InitializeFactors(theLengthFactor, thePlaneAngleFactor, theSolidAngleFactor);
        return aCopy;
    }

    /// <summary>Sets length unit for current transfer process.</summary>
    internal void SetCascadeUnit(double theUnit)
    {
        myCascadeUnit = theUnit;
    }

    /// <summary>A copy with the length unit for current transfer process set.</summary>
    public readonly StepData_Factors WithCascadeUnit(double theUnit)
    {
        StepData_Factors aCopy = this;
        aCopy.SetCascadeUnit(theUnit);
        return aCopy;
    }

    /// <summary>Returns length unit for current transfer process (mm by default).</summary>
    public readonly double CascadeUnit()
    {
        return myCascadeUnit;
    }

    /// <summary>Returns transient length factor for scaling of shapes at one stage of transfer process.</summary>
    public readonly double LengthFactor()
    {
        return myLengthFactor;
    }

    /// <summary>Returns transient plane angle factor for conversion of angles at one stage of transfer process.</summary>
    public readonly double PlaneAngleFactor()
    {
        return myPlaneAngleFactor;
    }

    /// <summary>Returns transient solid angle factor for conversion of angles at one stage of transfer process.</summary>
    public readonly double SolidAngleFactor()
    {
        return mySolidAngleFactor;
    }

    /// <summary>Returns transient factor radian degree for conversion of angles at one stage of transfer process.</summary>
    public readonly double FactorRadianDegree()
    {
        return myFactRD;
    }

    /// <summary>Returns transient factor degree radian for conversion of angles at one stage of transfer process.</summary>
    public readonly double FactorDegreeRadian()
    {
        return myFactDR;
    }
}
