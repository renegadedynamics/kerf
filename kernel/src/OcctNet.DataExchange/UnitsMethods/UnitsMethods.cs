// Port of: src/FoundationClasses/TKernel/UnitsMethods/UnitsMethods.hxx
// Port of: src/FoundationClasses/TKernel/UnitsMethods/UnitsMethods.cxx
// Port of: src/FoundationClasses/TKernel/UnitsMethods/UnitsMethods_LengthUnit.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.UnitsMethods_LengthUnit;

namespace Occt;

/// <summary>The Enumeration describes possible values for length units.</summary>
public enum UnitsMethods_LengthUnit
{
    UnitsMethods_LengthUnit_Undefined = 0,  // 1.
    UnitsMethods_LengthUnit_Inch = 1,       // 25.4
    UnitsMethods_LengthUnit_Millimeter = 2, // 1.
    UnitsMethods_LengthUnit_Foot = 4,       // 304.8
    UnitsMethods_LengthUnit_Mile = 5,       // 1609344.
    UnitsMethods_LengthUnit_Meter = 6,      // 1000.
    UnitsMethods_LengthUnit_Kilometer = 7,  // 1000000.
    UnitsMethods_LengthUnit_Mil = 8,        // (0.001 inch) // 0.0254
    UnitsMethods_LengthUnit_Micron = 9,     // 0.001
    UnitsMethods_LengthUnit_Centimeter = 10, // 10.
    UnitsMethods_LengthUnit_Microinch = 11, // 0.0000254
}

/// <summary>Class for using global units variables.</summary>
/// <remarks>
/// OCCT's "CasCade length unit" is a process-wide value (<c>UnitsMethods_CascadeLengthUnit</c>), which
/// XSAlgo_ShapeProcessor and Draw set from the parameter <c>xstep.cascade.unit</c> and the STEP translators read. The
/// port has no such value (plan 2.7, decision 12505): <see cref="GetCasCadeLengthUnit(Interface_StaticSet, UnitsMethods_LengthUnit)"/>
/// reads <c>xstep.cascade.unit</c> from the parameter set of the read or write, and OCCT's setters are left out.
/// </remarks>
public static class UnitsMethods
{
    /// <summary>
    /// Returns value of current internal unit for CASCADE in millimeters by default: the unit the parameter
    /// <c>xstep.cascade.unit</c> of <paramref name="theStatics"/> designates (an IGES unit code), millimeters if the
    /// set is null or has no such parameter.
    /// </summary>
    public static double GetCasCadeLengthUnit(Interface_StaticSet? theStatics, UnitsMethods_LengthUnit theBaseUnit = UnitsMethods_LengthUnit_Millimeter)
    {
        double aCascadeLengthUnit = theStatics is not null && theStatics.IsPresent("xstep.cascade.unit")
            ? GetLengthFactorValue(theStatics.IVal("xstep.cascade.unit"))
            : 1.0;
        return aCascadeLengthUnit * GetLengthUnitScale(UnitsMethods_LengthUnit_Millimeter, theBaseUnit);
    }

    /// <summary>
    /// Returns value of unit encoded by parameter theUnit (integer value denoting unit, as described in IGES
    /// standard) in millimeters by default.
    /// </summary>
    public static double GetLengthFactorValue(int theUnit)
    {
        switch (theUnit)
        {
            case 1:
                return 25.4; // inch
            case 2:
                return 1.0; // millimeter
            case 4:
                return 304.8; // foot
            case 5:
                return 1609344.0; // mile
            case 6:
                return 1000.0; // meter
            case 7:
                return 1000000.0; // kilometer
            case 8:
                return 0.0254; // mil (0.001 inch)
            case 9:
                return 0.001; // micron
            case 10:
                return 10.0; // centimeter
            case 11:
                return 0.0000254; // microinch
            default:
                return 1.0;
        }
    }

    /// <summary>Returns the scale factor for switch from first given unit to second given unit.</summary>
    public static double GetLengthUnitScale(UnitsMethods_LengthUnit theFromUnit, UnitsMethods_LengthUnit theToUnit)
    {
        double aVal1 = GetLengthFactorValue((int)theFromUnit);
        double aVal2 = GetLengthFactorValue((int)theToUnit);
        return aVal1 / aVal2;
    }

    /// <summary>Returns the enumeration corresponding to the given scale factor.</summary>
    public static UnitsMethods_LengthUnit GetLengthUnitByFactorValue(double theFactorValue, UnitsMethods_LengthUnit theBaseUnit = UnitsMethods_LengthUnit_Millimeter)
    {
        const double aPreci = 1.0e-6;
        double aValue = theFactorValue * GetLengthUnitScale(theBaseUnit, UnitsMethods_LengthUnit_Millimeter);
        if (Math.Abs(1.0 - aValue) < aPreci)
        {
            return UnitsMethods_LengthUnit_Millimeter;
        }
        else if (Math.Abs(25.4 - aValue) < aPreci)
        {
            return UnitsMethods_LengthUnit_Inch;
        }
        else if (Math.Abs(304.8 - aValue) < aPreci)
        {
            return UnitsMethods_LengthUnit_Foot;
        }
        else if (Math.Abs(1609344.0 - aValue) < aPreci)
        {
            return UnitsMethods_LengthUnit_Mile;
        }
        else if (Math.Abs(1000.0 - aValue) < aPreci)
        {
            return UnitsMethods_LengthUnit_Meter;
        }
        else if (Math.Abs(1000000.0 - aValue) < aPreci)
        {
            return UnitsMethods_LengthUnit_Kilometer;
        }
        else if (Math.Abs(0.0254 - aValue) < aPreci)
        {
            return UnitsMethods_LengthUnit_Mil;
        }
        else if (Math.Abs(0.001 - aValue) < aPreci)
        {
            return UnitsMethods_LengthUnit_Micron;
        }
        else if (Math.Abs(10.0 - aValue) < aPreci)
        {
            return UnitsMethods_LengthUnit_Centimeter;
        }
        else if (Math.Abs(0.0000254 - aValue) < aPreci)
        {
            return UnitsMethods_LengthUnit_Microinch;
        }
        return UnitsMethods_LengthUnit_Undefined;
    }

    /// <summary>Returns string for the given value of LengthUnit.</summary>
    public static string DumpLengthUnit(UnitsMethods_LengthUnit theUnit)
    {
        switch (theUnit)
        {
            case UnitsMethods_LengthUnit_Millimeter:
                return "mm";
            case UnitsMethods_LengthUnit_Meter:
                return "m";
            case UnitsMethods_LengthUnit_Centimeter:
                return "cm";
            case UnitsMethods_LengthUnit_Kilometer:
                return "km";
            case UnitsMethods_LengthUnit_Micron:
                return "micron";
            case UnitsMethods_LengthUnit_Inch:
                return "in";
            case UnitsMethods_LengthUnit_Mil:
                return "min";
            case UnitsMethods_LengthUnit_Microinch:
                return "nin";
            case UnitsMethods_LengthUnit_Foot:
                return "ft";
            case UnitsMethods_LengthUnit_Mile:
                return "stat.mile";
            default:
                return "UNDEFINED";
        }
    }

    /// <summary>Returns string name for the given scale factor.</summary>
    public static string DumpLengthUnit(double theScaleFactor, UnitsMethods_LengthUnit theBaseUnit = UnitsMethods_LengthUnit_Millimeter)
    {
        UnitsMethods_LengthUnit aUnit = GetLengthUnitByFactorValue(theScaleFactor, theBaseUnit);
        return DumpLengthUnit(aUnit);
    }

    /// <summary>Make conversion of given string to value of LengthUnit.</summary>
    public static UnitsMethods_LengthUnit LengthUnitFromString(string theStr, bool theCaseSensitive)
    {
        string aStr = theStr;
        if (!theCaseSensitive)
        {
            aStr = AsciiLowerCase(aStr);
        }
        if (aStr == "mm")
        {
            return UnitsMethods_LengthUnit_Millimeter;
        }
        else if (aStr == "m")
        {
            return UnitsMethods_LengthUnit_Meter;
        }
        else if (aStr == "cm")
        {
            return UnitsMethods_LengthUnit_Centimeter;
        }
        else if (aStr == "km")
        {
            return UnitsMethods_LengthUnit_Kilometer;
        }
        else if (aStr == "micron")
        {
            return UnitsMethods_LengthUnit_Micron;
        }
        else if (aStr == "in")
        {
            return UnitsMethods_LengthUnit_Inch;
        }
        else if (aStr == "min")
        {
            return UnitsMethods_LengthUnit_Mil;
        }
        else if (aStr == "nin")
        {
            return UnitsMethods_LengthUnit_Microinch;
        }
        else if (aStr == "ft")
        {
            return UnitsMethods_LengthUnit_Foot;
        }
        else if (aStr == "stat.mile")
        {
            return UnitsMethods_LengthUnit_Mile;
        }
        else
        {
            return UnitsMethods_LengthUnit_Undefined;
        }
    }

    // TCollection_AsciiString::LowerCase: C's tolower in the "C" locale
    private static string AsciiLowerCase(string theStr)
    {
        char[] aChars = theStr.ToCharArray();
        for (int i = 0; i < aChars.Length; i++)
        {
            if (aChars[i] >= 'A' && aChars[i] <= 'Z')
            {
                aChars[i] = (char)(aChars[i] + ('a' - 'A'));
            }
        }
        return new string(aChars);
    }
}
