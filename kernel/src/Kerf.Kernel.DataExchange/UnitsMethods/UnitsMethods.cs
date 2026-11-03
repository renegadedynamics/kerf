// Port of: src/FoundationClasses/TKernel/UnitsMethods/UnitsMethods.hxx
// Port of: src/FoundationClasses/TKernel/UnitsMethods/UnitsMethods.cxx
// Port of: src/FoundationClasses/TKernel/UnitsMethods/UnitsMethods_LengthUnit.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Exchange.Framework.FrameworkLengthUnit;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>The Enumeration describes possible values for length units.</summary>
public enum FrameworkLengthUnit
{
    LengthUnit_Undefined = 0,  // 1.
    LengthUnit_Inch = 1,       // 25.4
    LengthUnit_Millimeter = 2, // 1.
    LengthUnit_Foot = 4,       // 304.8
    LengthUnit_Mile = 5,       // 1609344.
    LengthUnit_Meter = 6,      // 1000.
    LengthUnit_Kilometer = 7,  // 1000000.
    LengthUnit_Mil = 8,        // (0.001 inch) // 0.0254
    LengthUnit_Micron = 9,     // 0.001
    LengthUnit_Centimeter = 10, // 10.
    LengthUnit_Microinch = 11, // 0.0000254
}

/// <summary>Class for using global units variables.</summary>
/// <remarks>
/// OCCT's "CasCade length unit" is a process-wide value (<c>UnitsMethods_CascadeLengthUnit</c>), which
/// XSAlgo_ShapeProcessor and Draw set from the parameter <c>xstep.cascade.unit</c> and the STEP translators read. The
/// port has no such value (plan 2.7, decision 1195): <see cref="GetCasCadeLengthUnit(StaticSet, FrameworkLengthUnit)"/>
/// reads <c>xstep.cascade.unit</c> from the parameter set of the read or write, and OCCT's setters are left out.
/// </remarks>
public static class UnitsMethods
{
    /// <summary>
    /// Returns value of current internal unit for CASCADE in millimeters by default: the unit the parameter
    /// <c>xstep.cascade.unit</c> of <paramref name="statics"/> designates (an IGES unit code), millimeters if the
    /// set is null or has no such parameter.
    /// </summary>
    public static double GetCasCadeLengthUnit(StaticSet? statics, FrameworkLengthUnit baseUnit = FrameworkLengthUnit.LengthUnit_Millimeter)
    {
        double cascadeLengthUnit = statics is not null && statics.IsPresent("xstep.cascade.unit")
            ? GetLengthFactorValue(statics.IVal("xstep.cascade.unit"))
            : 1.0;
        return cascadeLengthUnit * GetLengthUnitScale(FrameworkLengthUnit.LengthUnit_Millimeter, baseUnit);
    }

    /// <summary>
    /// Returns value of unit encoded by parameter theUnit (integer value denoting unit, as described in IGES
    /// standard) in millimeters by default.
    /// </summary>
    public static double GetLengthFactorValue(int unit)
    {
        switch (unit)
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
    public static double GetLengthUnitScale(FrameworkLengthUnit fromUnit, FrameworkLengthUnit toUnit)
    {
        double val1 = GetLengthFactorValue((int)fromUnit);
        double val2 = GetLengthFactorValue((int)toUnit);
        return val1 / val2;
    }

    /// <summary>Returns the enumeration corresponding to the given scale factor.</summary>
    public static FrameworkLengthUnit GetLengthUnitByFactorValue(double factorValue, FrameworkLengthUnit baseUnit = FrameworkLengthUnit.LengthUnit_Millimeter)
    {
        const double preci = 1.0e-6;
        double value = factorValue * GetLengthUnitScale(baseUnit, FrameworkLengthUnit.LengthUnit_Millimeter);
        if (Math.Abs(1.0 - value) < preci)
        {
            return FrameworkLengthUnit.LengthUnit_Millimeter;
        }
        else if (Math.Abs(25.4 - value) < preci)
        {
            return FrameworkLengthUnit.LengthUnit_Inch;
        }
        else if (Math.Abs(304.8 - value) < preci)
        {
            return FrameworkLengthUnit.LengthUnit_Foot;
        }
        else if (Math.Abs(1609344.0 - value) < preci)
        {
            return FrameworkLengthUnit.LengthUnit_Mile;
        }
        else if (Math.Abs(1000.0 - value) < preci)
        {
            return FrameworkLengthUnit.LengthUnit_Meter;
        }
        else if (Math.Abs(1000000.0 - value) < preci)
        {
            return FrameworkLengthUnit.LengthUnit_Kilometer;
        }
        else if (Math.Abs(0.0254 - value) < preci)
        {
            return FrameworkLengthUnit.LengthUnit_Mil;
        }
        else if (Math.Abs(0.001 - value) < preci)
        {
            return FrameworkLengthUnit.LengthUnit_Micron;
        }
        else if (Math.Abs(10.0 - value) < preci)
        {
            return FrameworkLengthUnit.LengthUnit_Centimeter;
        }
        else if (Math.Abs(0.0000254 - value) < preci)
        {
            return FrameworkLengthUnit.LengthUnit_Microinch;
        }
        return FrameworkLengthUnit.LengthUnit_Undefined;
    }

    /// <summary>Returns string for the given value of LengthUnit.</summary>
    public static string DumpLengthUnit(FrameworkLengthUnit unit)
    {
        switch (unit)
        {
            case FrameworkLengthUnit.LengthUnit_Millimeter:
                return "mm";
            case FrameworkLengthUnit.LengthUnit_Meter:
                return "m";
            case FrameworkLengthUnit.LengthUnit_Centimeter:
                return "cm";
            case FrameworkLengthUnit.LengthUnit_Kilometer:
                return "km";
            case FrameworkLengthUnit.LengthUnit_Micron:
                return "micron";
            case FrameworkLengthUnit.LengthUnit_Inch:
                return "in";
            case FrameworkLengthUnit.LengthUnit_Mil:
                return "min";
            case FrameworkLengthUnit.LengthUnit_Microinch:
                return "nin";
            case FrameworkLengthUnit.LengthUnit_Foot:
                return "ft";
            case FrameworkLengthUnit.LengthUnit_Mile:
                return "stat.mile";
            default:
                return "UNDEFINED";
        }
    }

    /// <summary>Returns string name for the given scale factor.</summary>
    public static string DumpLengthUnit(double scaleFactor, FrameworkLengthUnit baseUnit = FrameworkLengthUnit.LengthUnit_Millimeter)
    {
        FrameworkLengthUnit unit = GetLengthUnitByFactorValue(scaleFactor, baseUnit);
        return DumpLengthUnit(unit);
    }

    /// <summary>Make conversion of given string to value of LengthUnit.</summary>
    public static FrameworkLengthUnit LengthUnitFromString(string str, bool caseSensitive)
    {
        string aStr = str;
        if (!caseSensitive)
        {
            aStr = AsciiLowerCase(aStr);
        }
        if (aStr == "mm")
        {
            return FrameworkLengthUnit.LengthUnit_Millimeter;
        }
        else if (aStr == "m")
        {
            return FrameworkLengthUnit.LengthUnit_Meter;
        }
        else if (aStr == "cm")
        {
            return FrameworkLengthUnit.LengthUnit_Centimeter;
        }
        else if (aStr == "km")
        {
            return FrameworkLengthUnit.LengthUnit_Kilometer;
        }
        else if (aStr == "micron")
        {
            return FrameworkLengthUnit.LengthUnit_Micron;
        }
        else if (aStr == "in")
        {
            return FrameworkLengthUnit.LengthUnit_Inch;
        }
        else if (aStr == "min")
        {
            return FrameworkLengthUnit.LengthUnit_Mil;
        }
        else if (aStr == "nin")
        {
            return FrameworkLengthUnit.LengthUnit_Microinch;
        }
        else if (aStr == "ft")
        {
            return FrameworkLengthUnit.LengthUnit_Foot;
        }
        else if (aStr == "stat.mile")
        {
            return FrameworkLengthUnit.LengthUnit_Mile;
        }
        else
        {
            return FrameworkLengthUnit.LengthUnit_Undefined;
        }
    }

    // TCollection_AsciiString::LowerCase: C's tolower in the "C" locale
    private static string AsciiLowerCase(string str)
    {
        char[] chars = str.ToCharArray();
        for (int i = 0; i < chars.Length; i++)
        {
            if (chars[i] >= 'A' && chars[i] <= 'Z')
            {
                chars[i] = (char)(chars[i] + ('a' - 'A'));
            }
        }
        return new string(chars);
    }
}
