// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>An IntPolyh item whose OCCT <c>Dump()</c> takes no index (what <c>IntPolyh_Array::Dump</c> calls).</summary>
public interface IntPolyh_IDump
{
    /// <summary>Writes the item as OCCT's <c>Dump()</c> prints it to standard output.</summary>
    void Dump(TextWriter theStream);
}

/// <summary>The <c>printf</c> conversions of IntPolyh's dumps, culture-invariant.</summary>
internal static class IntPolyh_Format
{
    /// <summary><c>%+8.3e</c>: sign, three decimals, an exponent of at least two digits.</summary>
    public static string E(double theValue)
        => double.IsFinite(theValue)
               ? theValue.ToString("+0.000e+00;-0.000e+00;+0.000e+00", CultureInfo.InvariantCulture)
               : (double.IsNaN(theValue) ? "+nan" : (theValue > 0 ? "+inf" : "-inf")).PadLeft(8);

    /// <summary><c>%Nd</c>.</summary>
    public static string D(int theValue, int theWidth = 0)
        => theValue.ToString(CultureInfo.InvariantCulture).PadLeft(theWidth);

    /// <summary><c>%Nf</c> (six decimals).</summary>
    public static string F(double theValue, int theWidth = 0)
        => theValue.ToString("F6", CultureInfo.InvariantCulture).PadLeft(theWidth);
}
