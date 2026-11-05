// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_Datum3D.hxx
// Port of: src/FoundationClasses/TKMath/TopLoc/TopLoc_Datum3D.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Globalization;
using System.Runtime.CompilerServices;

namespace Kerf.Engine.Topology;

/// <summary>
/// An elementary coordinate system: a transformation relative to the default datum (origin (0, 0, 0), axes
/// (1, 0, 0), (0, 1, 0), (0, 0, 1)).
/// </summary>
/// <remarks>
/// A datum is compared by identity, not by value: two <see cref="Placement"/>s are equal only if they are
/// built from the same datum objects. OCCT relies on this to recognise shared sub-shapes. A datum never changes
/// once built.
/// </remarks>
internal class Datum3D : Transient
{
    private readonly Transform3D _trsf;

    /// <summary>The default datum: the identity.</summary>
    public Datum3D() => _trsf = new Transform3D();

    /// <summary>A datum from a transformation, which is expected to be rigid (OCCT doesn't check it).</summary>
    public Datum3D(in Transform3D t) => _trsf = t;

    /// <summary>The transformation which, applied to this datum, gives the default datum.</summary>
    public Transform3D Transformation() => _trsf;

    /// <summary>The transformation which, applied to this datum, gives the default datum.</summary>
    public Transform3D Trsf() => _trsf;

    /// <summary>The form of the transformation.</summary>
    public TransformForm Form() => _trsf.Form;

    /// <summary>Writes this datum (its identity and the rows of its transformation) to S.</summary>
    /// <remarks>
    /// OCCT writes the object's address; the port writes its runtime hash code. Numbers are written with six
    /// significant digits, as a C++ stream does by default, though .NET formats exponents differently.
    /// </remarks>
    public void ShallowDump(TextWriter s)
    {
        s.Write(" TopLoc_Datum3D " + RuntimeHelpers.GetHashCode(this).ToString("x8", CultureInfo.InvariantCulture) + '\n');
        int i;
        Transform3D t = _trsf;
        for (i = 1; i <= 3; i++)
        {
            s.Write("  ( " + Setw10(t.Value(i, 1)));
            s.Write("," + Setw10(t.Value(i, 2)));
            s.Write("," + Setw10(t.Value(i, 3)));
            s.Write("," + Setw10(t.Value(i, 4)));
            s.Write(")\n");
        }

        s.Write('\n');
    }

    private static string Setw10(double value) => value.ToString("G6", CultureInfo.InvariantCulture).PadLeft(10);

    public override string ToString() => $"TopLoc_Datum3D({_trsf})";
}
