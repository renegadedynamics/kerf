// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_Datum3D.hxx
// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_Datum3D.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// An elementary coordinate system: a transformation relative to the default datum (origin (0, 0, 0), axes
/// (1, 0, 0), (0, 1, 0), (0, 0, 1)).
/// </summary>
/// <remarks>
/// A datum is compared by identity, not by value: two <see cref="TopLoc_Location"/>s are equal only if they are
/// built from the same datum objects. OCCT relies on this to recognise shared sub-shapes. A datum never changes
/// once built.
/// </remarks>
public class TopLoc_Datum3D : Standard_Transient
{
    private readonly gp_Trsf myTrsf;

    /// <summary>The default datum: the identity.</summary>
    public TopLoc_Datum3D() => myTrsf = new gp_Trsf();

    /// <summary>A datum from a transformation, which is expected to be rigid (OCCT doesn't check it).</summary>
    public TopLoc_Datum3D(in gp_Trsf T) => myTrsf = T;

    /// <summary>The transformation which, applied to this datum, gives the default datum.</summary>
    public gp_Trsf Transformation() => myTrsf;

    /// <summary>The transformation which, applied to this datum, gives the default datum.</summary>
    public gp_Trsf Trsf() => myTrsf;

    /// <summary>The form of the transformation.</summary>
    public gp_TrsfForm Form() => myTrsf.Form();

    /// <summary>Writes this datum (its identity and the rows of its transformation) to S.</summary>
    /// <remarks>
    /// OCCT writes the object's address; the port writes its runtime hash code. Numbers are written with six
    /// significant digits, as a C++ stream does by default, though .NET formats exponents differently.
    /// </remarks>
    public void ShallowDump(TextWriter S)
    {
        S.Write(" TopLoc_Datum3D " + RuntimeHelpers.GetHashCode(this).ToString("x8", CultureInfo.InvariantCulture) + '\n');
        int i;
        gp_Trsf T = myTrsf;
        for (i = 1; i <= 3; i++)
        {
            S.Write("  ( " + Setw10(T.Value(i, 1)));
            S.Write("," + Setw10(T.Value(i, 2)));
            S.Write("," + Setw10(T.Value(i, 3)));
            S.Write("," + Setw10(T.Value(i, 4)));
            S.Write(")\n");
        }

        S.Write('\n');
    }

    private static string Setw10(double theValue) => theValue.ToString("G6", CultureInfo.InvariantCulture).PadLeft(10);

    public override string ToString() => $"TopLoc_Datum3D({myTrsf})";
}
