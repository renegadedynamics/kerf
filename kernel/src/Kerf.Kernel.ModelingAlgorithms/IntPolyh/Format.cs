// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Globalization;

namespace Kerf.Engine.Intersection.Polyhedra;

/// <summary>An IntPolyh item whose OCCT <c>Dump()</c> takes no index (what <c>IntPolyh_Array::Dump</c> calls).</summary>
internal interface IPolyhedronDump
{
    /// <summary>Writes the item as OCCT's <c>Dump()</c> prints it to standard output.</summary>
    void Dump(TextWriter stream);
}

/// <summary>The <c>printf</c> conversions of IntPolyh's dumps, culture-invariant.</summary>
internal static class Format
{
    /// <summary><c>%+8.3e</c>: sign, three decimals, an exponent of at least two digits.</summary>
    public static string E(double value)
        => double.IsFinite(value)
               ? value.ToString("+0.000e+00;-0.000e+00;+0.000e+00", CultureInfo.InvariantCulture)
               : (double.IsNaN(value) ? "+nan" : (value > 0 ? "+inf" : "-inf")).PadLeft(8);

    /// <summary><c>%Nd</c>.</summary>
    public static string D(int value, int width = 0)
        => value.ToString(CultureInfo.InvariantCulture).PadLeft(width);

    /// <summary><c>%Nf</c> (six decimals).</summary>
    public static string F(double value, int width = 0)
        => value.ToString("F6", CultureInfo.InvariantCulture).PadLeft(width);
}
