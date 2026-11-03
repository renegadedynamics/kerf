// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_UnitContext.hxx
// Port of: src/DataExchange/TKDESTEP/STEPConstruct/STEPConstruct_UnitContext.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Basic;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Representation;
using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Exchange.Step.Basic.SiPrefix;
using static Kerf.Engine.Exchange.Step.Basic.SiUnitName;

namespace Kerf.Engine.Exchange.Step.Construct;

/// <summary>
/// Tool for creation (encoding) and decoding (for writing and reading accordingly) context defining units and
/// tolerances (uncertainties).
/// </summary>
/// <remarks>A C++ value class, a C# class here (decision 1185).</remarks>
public class UnitContext
{
    // abv 17.11.99: renamed from StepPDR_MakeUnitAndToleranceContext and merged with STEPControl_Unit
    // abv 30.02.00: ability to write file in units other than MM

    /// <summary>Creates empty tool.</summary>
    public UnitContext()
    {
        _done = false;
        _lengthFactor = 0.0;
        _planeAngleFactor = 0.0;
        _solidAngleFactor = 0.0;
        _areaFactor = 0.0;
        _volumeFactor = 0.0;
        _lengthDone = _planeAngleDone = _solidAngleDone = _hasUncertainty = _areaDone = _volumeDone = false;
        // pdn file r_47-sd.stp initialize field.
        _uncertainty = double.MaxValue; // OCCT: RealLast()
    }

    /// <summary>
    /// Creates new context (units are MM and radians, uncertainty equal to Tol3d)
    /// </summary>
    public void Init(double tol3d, StepModel model, in Factors localFactors)
    {
        _done = true;

        _grc = new StepGeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx();
        TCollection_HAsciiString contextID = new TCollection_HAsciiString("Context #1"); // ?????

        TCollection_HAsciiString contextType = new TCollection_HAsciiString("3D Context with UNIT and UNCERTAINTY");

        // Units : LengthUnit and PlaneAngleUnit (no SolidAngleUnit applicable)

        NamedUnit lengthUnit;
        string uName = null;
        bool hasPref = true;
        SiPrefix siPref = SiPrefix.SpMilli;
        double scale = 1.0;
        switch ((int)model.InternalParameters.WriteUnit)
        {
            case 1:
                uName = "INCH";
                scale = 25.4;
                break;
            default:
            case 2:
                break;
            case 4:
                uName = "FOOT";
                scale = 304.8;
                break;
            case 5:
                uName = "MILE";
                scale = 1609344.0;
                break;
            case 6:
                hasPref = false;
                scale = 1000.0;
                break;
            case 7:
                siPref = SiPrefix.SpKilo;
                scale = 1000000.0;
                break;
            case 8:
                uName = "MIL";
                scale = 0.0254;
                break;
            case 9:
                siPref = SiPrefix.SpMicro;
                scale = 0.001;
                break;
            case 10:
                siPref = SiPrefix.SpCenti;
                scale = 10.0;
                break;
            case 11:
                uName = "MICROINCH";
                scale = 0.0000254;
                break;
        }

        SiUnitAndLengthUnit siUnit = new SiUnitAndLengthUnit();
        siUnit.Init(hasPref, siPref, SiUnitName.SunMetre);

        if (uName is not null)
        { // for non-metric units, create conversion_based_unit
            MeasureValueMember val = new MeasureValueMember();
            val.SetName("LENGTH_UNIT");
            val.SetReal(scale);

            LengthMeasureWithUnit measure = new LengthMeasureWithUnit();
            Unit unit1 = new Unit();
            unit1.SetValue(siUnit);
            measure.Init(val, unit1);

            DimensionalExponents theDimExp = new DimensionalExponents();
            theDimExp.Init(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

            TCollection_HAsciiString convName = new TCollection_HAsciiString(uName);
            ConversionBasedUnitAndLengthUnit convUnit = new ConversionBasedUnitAndLengthUnit();
            convUnit.Init(theDimExp, convName, measure);

            lengthUnit = convUnit;
        }
        else
        {
            lengthUnit = siUnit;
        }

        SiUnitAndPlaneAngleUnit radianUnit = new SiUnitAndPlaneAngleUnit();
        radianUnit.Init(false,
                        SiPrefix.SpMilli, // the unit is radian, no prefix
                        SiUnitName.SunRadian);

        NCollection_HArray1<NamedUnit> units = new NCollection_HArray1<NamedUnit>(1, 3);

        SiUnitAndSolidAngleUnit sradUnit = new SiUnitAndSolidAngleUnit();
        sradUnit.Init(false,
                      SiPrefix.SpMilli, // the unit is steradian, no prefix
                      SiUnitName.SunSteradian);

        units.SetValue(1, lengthUnit);
        units.SetValue(2, radianUnit);
        units.SetValue(3, sradUnit);

        // Uncertainty : 3D confusion Tolerance

        NCollection_HArray1<UncertaintyMeasureWithUnit> tols = new NCollection_HArray1<UncertaintyMeasureWithUnit>(1, 1);
        UncertaintyMeasureWithUnit theTol3d = new UncertaintyMeasureWithUnit();

        TCollection_HAsciiString tolName = new TCollection_HAsciiString("distance_accuracy_value");
        TCollection_HAsciiString tolDesc = new TCollection_HAsciiString("confusion accuracy");

        MeasureValueMember mvs = new MeasureValueMember();
        mvs.SetName("LENGTH_MEASURE");
        mvs.SetReal(tol3d / localFactors.LengthFactor());
        Unit unit = new Unit();
        unit.SetValue(lengthUnit);
        theTol3d.Init(mvs, unit, tolName, tolDesc);
        tols.SetValue(1, theTol3d);

        _grc.Init(contextID, contextType, 3, units, tols);
    }

    /// <summary>Creates new context with the default factors.</summary>
    public void Init(double tol3d, StepModel model) => Init(tol3d, model, new Factors());

    /// <summary>Returns True if Init was called successfully.</summary>
    public bool IsDone
    {
        get
        {
            return _done;
        }
    }

    /// <summary>Returns context (or Null if not done).</summary>
    public StepGeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx Value()
    {
        return _grc;
    }

    /// <summary>Convert SI prefix defined by enumeration to corresponding real factor (e.g. 1e6 for mega).</summary>
    public static double ConvertSiPrefix(SiPrefix aPrefix)
    {
        switch (aPrefix)
        {
            case SiPrefix.SpExa:
                return 1.0E+18;
            case SiPrefix.SpPeta:
                return 1.0E+15;
            case SiPrefix.SpTera:
                return 1.0E+12;
            case SiPrefix.SpGiga:
                return 1.0E+9;
            case SiPrefix.SpMega:
                return 1.0E+6;
            case SiPrefix.SpKilo:
                return 1.0E+3;
            case SiPrefix.SpHecto:
                return 1.0E+2;
            case SiPrefix.SpDeca:
                return 1.0E+1;
            case SiPrefix.SpDeci:
                return 1.0E-1;
            case SiPrefix.SpCenti:
                return 1.0E-2;
            case SiPrefix.SpMilli:
                return 1.0E-3;
            case SiPrefix.SpMicro:
                return 1.0E-6;
            case SiPrefix.SpNano:
                return 1.0E-9;
            case SiPrefix.SpPico:
                return 1.0E-12;
            case SiPrefix.SpFemto:
                return 1.0E-15;
            case SiPrefix.SpAtto:
                return 1.0E-18;
            default:
                break;
        }
        return 1.0;
    }

    private bool SiUnitNameFactor(SiUnit aSiUnit, out double siunFactor)
    {
        siunFactor = 1.0;
        switch (aSiUnit.Name())
        {
            case SiUnitName.SunMetre:
            case SiUnitName.SunRadian:
            case SiUnitName.SunSteradian:
                return true;
            default:
                //	std::cout << "Unknown SiUnitName : " << aSiUnit->Name() << std::endl;
                return false;
        }
    }

    /// <summary>
    /// Computes the length, plane angle and solid angle conversion factor. Returns a status, 0 if OK.
    /// </summary>
    public int ComputeFactors(GlobalUnitAssignedContext aContext, in Factors localFactors)
    {
        int status = 0;

        // Initialise the default value
        // status : 0 if OK, else 1 2 3
        _lengthFactor = _solidAngleFactor = 1.0;
        _planeAngleFactor = Math.PI / 180.0;
        _lengthDone = _planeAngleDone = _solidAngleDone = false;

        if (aContext is null)
        {
            return 1;
        }

        // Start Computation
        NCollection_HArray1<NamedUnit> theUnits = aContext.Units();
        int nbU = aContext.NbUnits();

        for (int i = 1; i <= nbU; i++)
        {
            NamedUnit theNamedUnit = aContext.UnitsValue(i);
            status = ComputeFactors(theNamedUnit, localFactors);
        }
        return status;
    }

    public int ComputeFactors(GlobalUnitAssignedContext aContext) => ComputeFactors(aContext, new Factors());

    public int ComputeFactors(NamedUnit aUnit, in Factors localFactors)
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
        if (aUnit.IsKind(typeof(ConversionBasedUnit)))
        {
            ConversionBasedUnit theCBU = aUnit as ConversionBasedUnit;

            MeasureWithUnit mwu = null;
            if (theCBU is not null)
            {
                Transient convFactor = theCBU.ConversionFactor();
                if (convFactor.IsKind(typeof(MeasureWithUnit)))
                {
                    mwu = convFactor as MeasureWithUnit;
                }
                else if (convFactor.IsKind(typeof(ReprItemAndMeasureWithUnit)))
                {
                    ReprItemAndMeasureWithUnit reprMeasureItem = convFactor as ReprItemAndMeasureWithUnit;
                    mwu = reprMeasureItem.GetMeasureWithUnit();
                }

                // sln 8.10.2001: the case of unrecognized entity
                if (mwu is null)
                {
                    return -1;
                }

                NamedUnit theTargetUnit = mwu.UnitComponent().NamedUnit();
                double theSIPFactor = 1.0;

                //: f5 abv 24 Apr 98: ProSTEP TR8 tr8_bv1_tc: INCHES
                SiUnit theSIU = theTargetUnit as SiUnit;

                if (theSIU is not null)
                {
                    if (theSIU.HasPrefix)
                    {
                        // Treat the prefix
                        SiPrefix prefix = theSIU.Prefix();
                        theSIPFactor = ConvertSiPrefix(prefix);
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
                double theMVAL = mwu.ValueComponent();
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
        else if (aUnit.IsKind(typeof(SiUnit)))
        {
            SiUnit theSIU = aUnit as SiUnit;
            double theSIPFactor = 1.0;
            if (theSIU.HasPrefix)
            {
                // Treat the prefix
                SiPrefix prefix = theSIU.Prefix();
                theSIPFactor = ConvertSiPrefix(prefix);
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

        double cascadeUnit = localFactors.CascadeUnit();
        if (aUnit.IsKind(typeof(ConversionBasedUnitAndLengthUnit))
            || aUnit.IsKind(typeof(SiUnitAndLengthUnit)))
        {
            _lengthFactor = parameter * 1000.0 / cascadeUnit;
            if (!_lengthDone)
            {
                _lengthDone = true;
            }
            else
            {
                status = 14;
            }
        } // end of LengthUnit
        else if (aUnit.IsKind(typeof(ConversionBasedUnitAndPlaneAngleUnit))
                 || aUnit.IsKind(typeof(SiUnitAndPlaneAngleUnit)))
        {
            _planeAngleFactor = parameter;
            _planeAngleDone = true;
        } // end of PlaneAngleUnit
        else if (aUnit.IsKind(typeof(ConversionBasedUnitAndSolidAngleUnit))
                 || aUnit.IsKind(typeof(SiUnitAndSolidAngleUnit)))
        {
            _solidAngleFactor = parameter;
            _solidAngleDone = true;
        } // end of SolidAngleUnit
        else if (aUnit.IsKind(typeof(ConversionBasedUnitAndAreaUnit))
                 || aUnit.IsKind(typeof(SiUnitAndAreaUnit)))
        {
            double af = parameter * 1000.0 / cascadeUnit;
            _areaDone = true;
            _areaFactor = Math.Pow(af, 2);
        }
        else if (aUnit.IsKind(typeof(ConversionBasedUnitAndVolumeUnit))
                 || aUnit.IsKind(typeof(SiUnitAndVolumeUnit)))
        {
            double af = parameter * 1000.0 / cascadeUnit;
            _volumeDone = true;
            _volumeFactor = Math.Pow(af, 3);
        }
        return status;
    }

    public int ComputeFactors(NamedUnit aUnit) => ComputeFactors(aUnit, new Factors());

    /// <summary>Computes the uncertainty value (for length).</summary>
    public int ComputeTolerance(GlobalUncertaintyAssignedContext aContext)
    {
        int status = 0;
        // Decode the Uncertainty information (geometric accuracy)

        _hasUncertainty = false;
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
            UncertaintyMeasureWithUnit umwu = aContext.UncertaintyValue(un);
            if (umwu is null)
            {
                continue;
            }
            // Decode the associated Unit
            SiUnitAndLengthUnit unit = umwu.UnitComponent().NamedUnit() as SiUnitAndLengthUnit;
            if (unit is not null)
            {
                // Extract Uncertainty value
                double lengthUncertainty = umwu.ValueComponent();
                // Update it according to the Length Unit Factor
                // pdn r_47-sd.stp to choose minimal uncertainty
                if (_uncertainty > lengthUncertainty)
                {
                    _uncertainty = lengthUncertainty;
                }
                _hasUncertainty = true;
            }
            else
            {
                ConversionBasedUnitAndLengthUnit cbulu =
                    umwu.UnitComponent().NamedUnit() as ConversionBasedUnitAndLengthUnit;
                if (cbulu is not null)
                {
                    // Extract Uncertainty value
                    double lengthUncertainty = umwu.ValueComponent();
                    // Update it according to the Length Unit Factor
                    // pdn r_47-sd.stp to choose minimal uncertainty
                    if (_uncertainty > lengthUncertainty)
                    {
                        _uncertainty = lengthUncertainty; // *lengthFactor; fait par appelant
                    }
                    _hasUncertainty = true;
                }
            }
        }

        return status;
    }

    /// <summary>Returns the lengthFactor.</summary>
    public double LengthFactor() => _lengthFactor;

    /// <summary>Returns the planeAngleFactor.</summary>
    public double PlaneAngleFactor() => _planeAngleFactor;

    /// <summary>Returns the solidAngleFactor.</summary>
    public double SolidAngleFactor() => _solidAngleFactor;

    /// <summary>Returns the Uncertainty value (for Length).</summary>
    public double Uncertainty() => _uncertainty;

    /// <summary>Returns the areaFactor.</summary>
    public double AreaFactor() => _areaFactor;

    /// <summary>Returns the volumeFactor.</summary>
    public double VolumeFactor() => _volumeFactor;

    /// <summary>Tells if a Uncertainty (for length) is recorded.</summary>
    public bool HasUncertainty => _hasUncertainty;

    /// <summary>Returns true if ComputeFactors has calculated a lengthFactor (pre-defined or calculated).</summary>
    public bool LengthDone() => _lengthDone;

    /// <summary>Returns true if ComputeFactors has calculated a planeAngleFactor.</summary>
    public bool PlaneAngleDone() => _planeAngleDone;

    /// <summary>Returns true if ComputeFactors has calculated a solidAngleFactor.</summary>
    public bool SolidAngleDone() => _solidAngleDone;

    /// <summary>Returns true if areaFactor is computed.</summary>
    public bool AreaDone() => _areaDone;

    /// <summary>Returns true if volumeFactor is computed.</summary>
    public bool VolumeDone() => _volumeDone;

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

    private bool _done;
    private StepGeomRepContextAndGlobUnitAssCtxAndGlobUncertaintyAssCtx _grc;
    private double _lengthFactor;
    private double _planeAngleFactor;
    private double _solidAngleFactor;
    private double _uncertainty;
    private double _areaFactor;
    private double _volumeFactor;
    private bool _areaDone;
    private bool _volumeDone;
    private bool _lengthDone;
    private bool _planeAngleDone;
    private bool _solidAngleDone;
    private bool _hasUncertainty;
}
