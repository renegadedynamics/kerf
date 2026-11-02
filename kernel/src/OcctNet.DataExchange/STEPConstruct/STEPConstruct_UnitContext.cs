// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_UnitContext.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_UnitContext.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.StepBasic_SiPrefix;
using static Occt.StepBasic_SiUnitName;

namespace Occt;

/// <summary>
/// Tool for creation (encoding) and decoding (for writing and reading accordingly) context defining units and
/// tolerances (uncertainties).
/// </summary>
/// <remarks>A C++ value class, a C# class here (decision 1185).</remarks>
public class STEPConstruct_UnitContext
{
    // abv 17.11.99: renamed from StepPDR_MakeUnitAndToleranceContext and merged with STEPControl_Unit
    // abv 30.02.00: ability to write file in units other than MM

    /// <summary>Creates empty tool.</summary>
    public STEPConstruct_UnitContext()
    {
        done = false;
        lengthFactor = 0.0;
        planeAngleFactor = 0.0;
        solidAngleFactor = 0.0;
        areaFactor = 0.0;
        volumeFactor = 0.0;
        lengthDone = planeAngleDone = solidAngleDone = hasUncertainty = areaDone = volumeDone = false;
        // pdn file r_47-sd.stp initialize field.
        theUncertainty = double.MaxValue; // OCCT: RealLast()
    }

    /// <summary>
    /// Creates new context (units are MM and radians, uncertainty equal to Tol3d)
    /// </summary>
    public void Init(double Tol3d, StepData_StepModel theModel, in StepData_Factors theLocalFactors)
    {
        done = true;

        GRC = new StepGeom_GeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx();
        TCollection_HAsciiString contextID = new TCollection_HAsciiString("Context #1"); // ?????

        TCollection_HAsciiString contextType = new TCollection_HAsciiString("3D Context with UNIT and UNCERTAINTY");

        // Units : LengthUnit and PlaneAngleUnit (no SolidAngleUnit applicable)

        StepBasic_NamedUnit lengthUnit;
        string uName = null;
        bool hasPref = true;
        StepBasic_SiPrefix siPref = StepBasic_spMilli;
        double aScale = 1.0;
        switch ((int)theModel.InternalParameters.WriteUnit)
        {
            case 1:
                uName = "INCH";
                aScale = 25.4;
                break;
            default:
            case 2:
                break;
            case 4:
                uName = "FOOT";
                aScale = 304.8;
                break;
            case 5:
                uName = "MILE";
                aScale = 1609344.0;
                break;
            case 6:
                hasPref = false;
                aScale = 1000.0;
                break;
            case 7:
                siPref = StepBasic_spKilo;
                aScale = 1000000.0;
                break;
            case 8:
                uName = "MIL";
                aScale = 0.0254;
                break;
            case 9:
                siPref = StepBasic_spMicro;
                aScale = 0.001;
                break;
            case 10:
                siPref = StepBasic_spCenti;
                aScale = 10.0;
                break;
            case 11:
                uName = "MICROINCH";
                aScale = 0.0000254;
                break;
        }

        StepBasic_SiUnitAndLengthUnit siUnit = new StepBasic_SiUnitAndLengthUnit();
        siUnit.Init(hasPref, siPref, StepBasic_sunMetre);

        if (uName is not null)
        { // for non-metric units, create conversion_based_unit
            StepBasic_MeasureValueMember val = new StepBasic_MeasureValueMember();
            val.SetName("LENGTH_UNIT");
            val.SetReal(aScale);

            StepBasic_LengthMeasureWithUnit measure = new StepBasic_LengthMeasureWithUnit();
            StepBasic_Unit Unit1 = new StepBasic_Unit();
            Unit1.SetValue(siUnit);
            measure.Init(val, Unit1);

            StepBasic_DimensionalExponents theDimExp = new StepBasic_DimensionalExponents();
            theDimExp.Init(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

            TCollection_HAsciiString convName = new TCollection_HAsciiString(uName);
            StepBasic_ConversionBasedUnitAndLengthUnit convUnit = new StepBasic_ConversionBasedUnitAndLengthUnit();
            convUnit.Init(theDimExp, convName, measure);

            lengthUnit = convUnit;
        }
        else
        {
            lengthUnit = siUnit;
        }

        StepBasic_SiUnitAndPlaneAngleUnit radianUnit = new StepBasic_SiUnitAndPlaneAngleUnit();
        radianUnit.Init(false,
                        StepBasic_spMilli, // the unit is radian, no prefix
                        StepBasic_sunRadian);

        NCollection_HArray1<StepBasic_NamedUnit> units = new NCollection_HArray1<StepBasic_NamedUnit>(1, 3);

        StepBasic_SiUnitAndSolidAngleUnit sradUnit = new StepBasic_SiUnitAndSolidAngleUnit();
        sradUnit.Init(false,
                      StepBasic_spMilli, // the unit is steradian, no prefix
                      StepBasic_sunSteradian);

        units.SetValue(1, lengthUnit);
        units.SetValue(2, radianUnit);
        units.SetValue(3, sradUnit);

        // Uncertainty : 3D confusion Tolerance

        NCollection_HArray1<StepBasic_UncertaintyMeasureWithUnit> Tols = new NCollection_HArray1<StepBasic_UncertaintyMeasureWithUnit>(1, 1);
        StepBasic_UncertaintyMeasureWithUnit theTol3d = new StepBasic_UncertaintyMeasureWithUnit();

        TCollection_HAsciiString TolName = new TCollection_HAsciiString("distance_accuracy_value");
        TCollection_HAsciiString TolDesc = new TCollection_HAsciiString("confusion accuracy");

        StepBasic_MeasureValueMember mvs = new StepBasic_MeasureValueMember();
        mvs.SetName("LENGTH_MEASURE");
        mvs.SetReal(Tol3d / theLocalFactors.LengthFactor());
        StepBasic_Unit Unit = new StepBasic_Unit();
        Unit.SetValue(lengthUnit);
        theTol3d.Init(mvs, Unit, TolName, TolDesc);
        Tols.SetValue(1, theTol3d);

        GRC.Init(contextID, contextType, 3, units, Tols);
    }

    /// <summary>Creates new context with the default factors.</summary>
    public void Init(double Tol3d, StepData_StepModel theModel) => Init(Tol3d, theModel, new StepData_Factors());

    /// <summary>Returns True if Init was called successfully.</summary>
    public bool IsDone()
    {
        return done;
    }

    /// <summary>Returns context (or Null if not done).</summary>
    public StepGeom_GeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx Value()
    {
        return GRC;
    }

    /// <summary>Convert SI prefix defined by enumeration to corresponding real factor (e.g. 1e6 for mega).</summary>
    public static double ConvertSiPrefix(StepBasic_SiPrefix aPrefix)
    {
        switch (aPrefix)
        {
            case StepBasic_spExa:
                return 1.0E+18;
            case StepBasic_spPeta:
                return 1.0E+15;
            case StepBasic_spTera:
                return 1.0E+12;
            case StepBasic_spGiga:
                return 1.0E+9;
            case StepBasic_spMega:
                return 1.0E+6;
            case StepBasic_spKilo:
                return 1.0E+3;
            case StepBasic_spHecto:
                return 1.0E+2;
            case StepBasic_spDeca:
                return 1.0E+1;
            case StepBasic_spDeci:
                return 1.0E-1;
            case StepBasic_spCenti:
                return 1.0E-2;
            case StepBasic_spMilli:
                return 1.0E-3;
            case StepBasic_spMicro:
                return 1.0E-6;
            case StepBasic_spNano:
                return 1.0E-9;
            case StepBasic_spPico:
                return 1.0E-12;
            case StepBasic_spFemto:
                return 1.0E-15;
            case StepBasic_spAtto:
                return 1.0E-18;
            default:
                break;
        }
        return 1.0;
    }

    private bool SiUnitNameFactor(StepBasic_SiUnit aSiUnit, out double theSIUNFactor)
    {
        theSIUNFactor = 1.0;
        switch (aSiUnit.Name())
        {
            case StepBasic_sunMetre:
            case StepBasic_sunRadian:
            case StepBasic_sunSteradian:
                return true;
            default:
                //	std::cout << "Unknown SiUnitName : " << aSiUnit->Name() << std::endl;
                return false;
        }
    }

    /// <summary>
    /// Computes the length, plane angle and solid angle conversion factor. Returns a status, 0 if OK.
    /// </summary>
    public int ComputeFactors(StepRepr_GlobalUnitAssignedContext aContext, in StepData_Factors theLocalFactors)
    {
        int status = 0;

        // Initialise the default value
        // status : 0 if OK, else 1 2 3
        lengthFactor = solidAngleFactor = 1.0;
        planeAngleFactor = Math.PI / 180.0;
        lengthDone = planeAngleDone = solidAngleDone = false;

        if (aContext is null)
        {
            return 1;
        }

        // Start Computation
        NCollection_HArray1<StepBasic_NamedUnit> theUnits = aContext.Units();
        int nbU = aContext.NbUnits();

        for (int i = 1; i <= nbU; i++)
        {
            StepBasic_NamedUnit theNamedUnit = aContext.UnitsValue(i);
            status = ComputeFactors(theNamedUnit, theLocalFactors);
        }
        return status;
    }

    public int ComputeFactors(StepRepr_GlobalUnitAssignedContext aContext) => ComputeFactors(aContext, new StepData_Factors());

    public int ComputeFactors(StepBasic_NamedUnit aUnit, in StepData_Factors theLocalFactors)
    {
        //: f3 abv 8 Apr 98: ProSTEP TR8 tr8_as_sd_sw: the case of unrecognized entity
        if (aUnit is null)
        {
            return -1;
        }

        int status = 0;
        double theFactor = 0.0;
        double theSIUNF = 0.0;

        double parameter = 0.0;
        bool parameterDone = false;
        if (aUnit.IsKind(typeof(StepBasic_ConversionBasedUnit)))
        {
            StepBasic_ConversionBasedUnit theCBU = aUnit as StepBasic_ConversionBasedUnit;

            StepBasic_MeasureWithUnit aMWU = null;
            if (theCBU is not null)
            {
                Standard_Transient aConvFactor = theCBU.ConversionFactor();
                if (aConvFactor.IsKind(typeof(StepBasic_MeasureWithUnit)))
                {
                    aMWU = aConvFactor as StepBasic_MeasureWithUnit;
                }
                else if (aConvFactor.IsKind(typeof(StepRepr_ReprItemAndMeasureWithUnit)))
                {
                    StepRepr_ReprItemAndMeasureWithUnit aReprMeasureItem = aConvFactor as StepRepr_ReprItemAndMeasureWithUnit;
                    aMWU = aReprMeasureItem.GetMeasureWithUnit();
                }

                // sln 8.10.2001: the case of unrecognized entity
                if (aMWU is null)
                {
                    return -1;
                }

                StepBasic_NamedUnit theTargetUnit = aMWU.UnitComponent().NamedUnit();
                double theSIPFactor = 1.0;

                //: f5 abv 24 Apr 98: ProSTEP TR8 tr8_bv1_tc: INCHES
                StepBasic_SiUnit theSIU = theTargetUnit as StepBasic_SiUnit;

                if (theSIU is not null)
                {
                    if (theSIU.HasPrefix())
                    {
                        // Treat the prefix
                        StepBasic_SiPrefix aPrefix = theSIU.Prefix();
                        theSIPFactor = ConvertSiPrefix(aPrefix);
                    }
                    // Treat the SiUnitName
                    if (!SiUnitNameFactor(theSIU, out theSIUNF))
                    {
                        status = 11; // et continue
                    }
                }
                else
                {
                    return 3;
                }
                double theMVAL = aMWU.ValueComponent();
                theFactor = theSIPFactor * theMVAL; // * theSIUNF * pow(10.,theLExp)
            }
            parameter = theFactor;
            if (!parameterDone)
            {
                parameterDone = true;
            }
            else
            {
                status = 14;
            }
        }
        else if (aUnit.IsKind(typeof(StepBasic_SiUnit)))
        {
            StepBasic_SiUnit theSIU = aUnit as StepBasic_SiUnit;
            double theSIPFactor = 1.0;
            if (theSIU.HasPrefix())
            {
                // Treat the prefix
                StepBasic_SiPrefix aPrefix = theSIU.Prefix();
                theSIPFactor = ConvertSiPrefix(aPrefix);
            }

            // Treat the SiUnitName
            if (!SiUnitNameFactor(theSIU, out theSIUNF))
            {
                status = 11;
            }

            // final computation for lengthFactor
            theFactor = theSIPFactor * theSIUNF;
            parameter = theFactor;
            if (!parameterDone)
            {
                parameterDone = true;
            }
            else
            {
                status = 14;
            }
        }

        // Defining a type of unit
        if (!parameterDone)
        {
            return 0;
        }

        double aCascadeUnit = theLocalFactors.CascadeUnit();
        if (aUnit.IsKind(typeof(StepBasic_ConversionBasedUnitAndLengthUnit))
            || aUnit.IsKind(typeof(StepBasic_SiUnitAndLengthUnit)))
        {
            lengthFactor = parameter * 1000.0 / aCascadeUnit;
            if (!lengthDone)
            {
                lengthDone = true;
            }
            else
            {
                status = 14;
            }
        } // end of LengthUnit
        else if (aUnit.IsKind(typeof(StepBasic_ConversionBasedUnitAndPlaneAngleUnit))
                 || aUnit.IsKind(typeof(StepBasic_SiUnitAndPlaneAngleUnit)))
        {
            planeAngleFactor = parameter;
            planeAngleDone = true;
        } // end of PlaneAngleUnit
        else if (aUnit.IsKind(typeof(StepBasic_ConversionBasedUnitAndSolidAngleUnit))
                 || aUnit.IsKind(typeof(StepBasic_SiUnitAndSolidAngleUnit)))
        {
            solidAngleFactor = parameter;
            solidAngleDone = true;
        } // end of SolidAngleUnit
        else if (aUnit.IsKind(typeof(StepBasic_ConversionBasedUnitAndAreaUnit))
                 || aUnit.IsKind(typeof(StepBasic_SiUnitAndAreaUnit)))
        {
            double af = parameter * 1000.0 / aCascadeUnit;
            areaDone = true;
            areaFactor = Math.Pow(af, 2);
        }
        else if (aUnit.IsKind(typeof(StepBasic_ConversionBasedUnitAndVolumeUnit))
                 || aUnit.IsKind(typeof(StepBasic_SiUnitAndVolumeUnit)))
        {
            double af = parameter * 1000.0 / aCascadeUnit;
            volumeDone = true;
            volumeFactor = Math.Pow(af, 3);
        }
        return status;
    }

    public int ComputeFactors(StepBasic_NamedUnit aUnit) => ComputeFactors(aUnit, new StepData_Factors());

    /// <summary>Computes the uncertainty value (for length).</summary>
    public int ComputeTolerance(StepRepr_GlobalUncertaintyAssignedContext aContext)
    {
        int status = 0;
        // Decode the Uncertainty information (geometric accuracy)

        hasUncertainty = false;
        int nbUncertainty = 0;

        if (aContext is not null)
        {
            nbUncertainty = aContext.NbUncertainty();
        }
        else
        {
            return 40;
        }

        for (int un = 1; un <= nbUncertainty; un++)
        {
            StepBasic_UncertaintyMeasureWithUnit aUMWU = aContext.UncertaintyValue(un);
            if (aUMWU is null)
            {
                continue;
            }
            // Decode the associated Unit
            StepBasic_SiUnitAndLengthUnit aUnit = aUMWU.UnitComponent().NamedUnit() as StepBasic_SiUnitAndLengthUnit;
            if (aUnit is not null)
            {
                // Extract Uncertainty value
                double LengthUncertainty = aUMWU.ValueComponent();
                // Update it according to the Length Unit Factor
                // pdn r_47-sd.stp to choose minimal uncertainty
                if (theUncertainty > LengthUncertainty)
                {
                    theUncertainty = LengthUncertainty;
                }
                hasUncertainty = true;
            }
            else
            {
                StepBasic_ConversionBasedUnitAndLengthUnit aCBULU =
                    aUMWU.UnitComponent().NamedUnit() as StepBasic_ConversionBasedUnitAndLengthUnit;
                if (aCBULU is not null)
                {
                    // Extract Uncertainty value
                    double LengthUncertainty = aUMWU.ValueComponent();
                    // Update it according to the Length Unit Factor
                    // pdn r_47-sd.stp to choose minimal uncertainty
                    if (theUncertainty > LengthUncertainty)
                    {
                        theUncertainty = LengthUncertainty; // *lengthFactor; fait par appelant
                    }
                    hasUncertainty = true;
                }
            }
        }

        return status;
    }

    /// <summary>Returns the lengthFactor.</summary>
    public double LengthFactor() => lengthFactor;

    /// <summary>Returns the planeAngleFactor.</summary>
    public double PlaneAngleFactor() => planeAngleFactor;

    /// <summary>Returns the solidAngleFactor.</summary>
    public double SolidAngleFactor() => solidAngleFactor;

    /// <summary>Returns the Uncertainty value (for Length).</summary>
    public double Uncertainty() => theUncertainty;

    /// <summary>Returns the areaFactor.</summary>
    public double AreaFactor() => areaFactor;

    /// <summary>Returns the volumeFactor.</summary>
    public double VolumeFactor() => volumeFactor;

    /// <summary>Tells if a Uncertainty (for length) is recorded.</summary>
    public bool HasUncertainty() => hasUncertainty;

    /// <summary>Returns true if ComputeFactors has calculated a lengthFactor (pre-defined or calculated).</summary>
    public bool LengthDone() => lengthDone;

    /// <summary>Returns true if ComputeFactors has calculated a planeAngleFactor.</summary>
    public bool PlaneAngleDone() => planeAngleDone;

    /// <summary>Returns true if ComputeFactors has calculated a solidAngleFactor.</summary>
    public bool SolidAngleDone() => solidAngleDone;

    /// <summary>Returns true if areaFactor is computed.</summary>
    public bool AreaDone() => areaDone;

    /// <summary>Returns true if volumeFactor is computed.</summary>
    public bool VolumeDone() => volumeDone;

    /// <summary>Returns a message for a given status (0 - empty) This message can then be added as warning for transfer.</summary>
    public string StatusMessage(int status)
    {
        switch (status)
        {
            case 0:
                return "";
            case 1:
                return "No GlobalUnitAssignedContext, default taken";
            case 2:
                return "No LengthMeasureWithUnit, default taken";
            case 3:
                return "No SiUnit for LengthMeasure undefined, default taken";
            case 4:
                return "No PlaneAngleMeasureWithUnit, default taken";
            case 5:
                return "No SiUnit for PlaneAngleMeasure undefined, default taken";
            case 6:
                return "No SolidAngleMeasureWithUnit, default taken";
            case 7:
                return "No SiUnit for SolidAngleMeasure undefined, default taken";

            case 11:
                return "Length Unit not recognized, default taken";
            case 12:
                return "Plane Angle Unit not recognized, default taken";
            case 13:
                return "Solid Angle Unit not recognized, default taken";
            case 14:
                return "At least one unit is twice defined";

            case 40:
                return "Bad GlobalUncertaintyAssignedContext, default unit taken";

            default:
                break;
        }

        return "Badly defined units, default taken";
    }

    private bool done;
    private StepGeom_GeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx GRC;
    private double lengthFactor;
    private double planeAngleFactor;
    private double solidAngleFactor;
    private double theUncertainty;
    private double areaFactor;
    private double volumeFactor;
    private bool areaDone;
    private bool volumeDone;
    private bool lengthDone;
    private bool planeAngleDone;
    private bool solidAngleDone;
    private bool hasUncertainty;
}
