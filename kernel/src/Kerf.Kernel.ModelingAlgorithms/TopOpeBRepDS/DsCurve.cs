// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Curve.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>
/// A curve of the legacy data structure: a 3D curve with a tolerance and an optional range, the two shapes it lies on
/// with its two surface/curve interferences there (which hold its pcurves), and bookkeeping (kept, mother, index).
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. The data structure keeps its own copy (<c>TopOpeBRepDS_DataStructure.AddCurve</c>,
/// which also sets the caller's <see cref="DSIndex"/>, as OCCT does) and gives it for writing by <c>ref</c>
/// (<see cref="DsDataStructure.ChangeCurve"/>).
/// </remarks>
public partial struct DsCurve : IEquatable<DsCurve>
{
    private Curve3D? _curve;
    private double _first;
    private double _last;
    private bool _rangeDefined;
    private double _tolerance;
    private bool _isWalk;
    private Shape _s1;
    private Shape _s2;
    private DsInterference? _sci1;
    private DsInterference? _sci2;
    private bool _keep;
    private int _mother;
    private int _dsIndex;

    /// <summary>An empty curve with the confusion tolerance, kept.</summary>
    public DsCurve()
    {
        _curve = null;
        _first = 0.0;
        _last = 0.0;
        _rangeDefined = false;
        _tolerance = global::Kerf.Engine.Foundation.Precision.Confusion();
        _isWalk = false;
        _s1 = new Shape();
        _s2 = new Shape();
        _sci1 = null;
        _sci2 = null;
        _keep = true;
        _mother = 0;
        _dsIndex = 0;
    }

    /// <summary>
    /// The curve <paramref name="c"/> with the tolerance <paramref name="t"/>; <paramref name="isWalk"/> if it comes
    /// from a walking line.
    /// </summary>
    public DsCurve(Curve3D? c, double t, bool isWalk = false)
    {
        _first = 0.0;
        _last = 0.0;
        _rangeDefined = false;
        _s1 = new Shape();
        _s2 = new Shape();
        _sci1 = null;
        _sci2 = null;
        _keep = true;
        _mother = 0;
        _dsIndex = 0;
        DefineCurve(c, t, isWalk);
    }

    internal void DefineCurve(Curve3D? c, double t, bool isWalk)
    {
        _curve = c;
        _tolerance = t;
        _isWalk = isWalk;
    }

    internal void Tolerance(double t)
    {
        _tolerance = t;
    }

    /// <summary>Sets the surface/curve interferences on the first and second shapes.</summary>
    internal void SetSCI(DsInterference? sci1, DsInterference? sci2)
    {
        _sci1 = sci1;
        _sci2 = sci2;
    }

    /// <summary>The surface/curve interferences on the first and second shapes.</summary>
    public readonly void GetSCI(out DsInterference? i1, out DsInterference? i2)
    {
        i1 = GetSCI1();
        i2 = GetSCI2();
    }

    /// <summary>The surface/curve interference on the first shape.</summary>
    public readonly DsInterference? GetSCI1() => _sci1;

    /// <summary>The surface/curve interference on the second shape.</summary>
    public readonly DsInterference? GetSCI2() => _sci2;

    internal void SetShapes(in Shape s1, in Shape s2)
    {
        _s1 = s1;
        _s2 = s2;
    }

    /// <summary>The two shapes the curve lies on.</summary>
    public readonly void GetShapes(out Shape s1, out Shape s2)
    {
        s1 = _s1;
        s2 = _s2;
    }

    /// <summary>The first shape.</summary>
    public readonly Shape Shape1() => _s1;

    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref Shape ChangeShape1() => ref _s1;

    /// <summary>The second shape.</summary>
    public readonly Shape Shape2() => _s2;

    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref Shape ChangeShape2() => ref _s2;

    /// <summary>The 3D curve (null for a degenerated curve).</summary>
    public readonly Curve3D? Curve() => _curve;

    internal void SetRange(double first, double last)
    {
        _first = first;
        _last = last;
        _rangeDefined = true;
    }

    /// <summary>Whether the range is defined; if so, its bounds.</summary>
    /// <remarks>OCCT leaves the bounds unchanged when the range is not defined; the port writes 0.</remarks>
    public readonly bool Range(out double first, out double last)
    {
        first = 0.0;
        last = 0.0;
        if (_rangeDefined)
        {
            first = _first;
            last = _last;
        }
        return _rangeDefined;
    }

    /// <summary>The tolerance.</summary>
    public readonly double Tolerance() => _tolerance;

    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref Curve3D? ChangeCurve() => ref _curve;

    internal void Curve(Curve3D? c3D, double tol)
    {
        _curve = c3D;
        _tolerance = tol;
    }

    /// <summary>The pcurve on the first shape: that of its surface/curve interference, or null.</summary>
    /// <exception cref="InvalidCastException">The first interference is not a surface/curve interference.</exception>
    public readonly Curve2D? Curve1()
    {
        if (_sci1 is not null)
        {
            // OCCT down-casts without checking (and dereferences a null handle on a mismatch); the port's cast throws
            return ((DsSurfaceCurveInterference)_sci1).PCurve();
        }
        else
        {
            return null;
        }
    }

    internal readonly void Curve1(Curve2D? pc1)
    {
        if (_sci1 is not null)
        {
            ((DsSurfaceCurveInterference)_sci1).PCurve(pc1);
        }
    }

    /// <summary>The pcurve on the second shape: that of its surface/curve interference, or null.</summary>
    /// <exception cref="InvalidCastException">The second interference is not a surface/curve interference.</exception>
    public readonly Curve2D? Curve2()
    {
        if (_sci2 is not null)
        {
            return ((DsSurfaceCurveInterference)_sci2).PCurve();
        }
        else
        {
            return null;
        }
    }

    internal readonly void Curve2(Curve2D? pc2)
    {
        if (_sci2 is not null)
        {
            ((DsSurfaceCurveInterference)_sci2).PCurve(pc2);
        }
    }

    /// <summary>Whether the curve comes from a walking line.</summary>
    public readonly bool IsWalk => _isWalk;

    internal void ChangeIsWalk(bool b)
    {
        _isWalk = b;
    }

    /// <summary>Whether the curve is kept (not removed from the data structure).</summary>
    public readonly bool Keep() => _keep;

    internal void ChangeKeep(bool b)
    {
        _keep = b;
    }

    /// <summary>The index of the curve this one was made from (<see cref="DsHDataStructure.MakeCurve"/>), or 0.</summary>
    public readonly int Mother() => _mother;

    internal void ChangeMother(int b)
    {
        _mother = b;
    }

    /// <summary>The curve's index in the data structure (set when it is added), or 0.</summary>
    public readonly int DSIndex() => _dsIndex;

    internal void ChangeDSIndex(int b)
    {
        _dsIndex = b;
    }

    /// <summary>A copy with the range [<paramref name="first"/>, <paramref name="last"/>].</summary>
    public readonly DsCurve WithRange(double first, double last)
    {
        DsCurve copy = this;
        copy.SetRange(first, last);
        return copy;
    }

    /// <summary>A copy with the tolerance <paramref name="t"/>.</summary>
    public readonly DsCurve WithTolerance(double t)
    {
        DsCurve copy = this;
        copy.Tolerance(t);
        return copy;
    }

    /// <inheritdoc/>
    public readonly bool Equals(DsCurve other)
        => ReferenceEquals(_curve, other._curve) && _first.Equals(other._first) && _last.Equals(other._last)
           && _rangeDefined == other._rangeDefined && _tolerance.Equals(other._tolerance)
           && _isWalk == other._isWalk && _s1.Equals(other._s1) && _s2.Equals(other._s2)
           && ReferenceEquals(_sci1, other._sci1) && ReferenceEquals(_sci2, other._sci2)
           && _keep == other._keep && _mother == other._mother && _dsIndex == other._dsIndex;

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is DsCurve other && Equals(other);

    /// <inheritdoc/>
    public override readonly int GetHashCode()
        => HashCode.Combine(_curve is null ? 0 : System.Runtime.CompilerServices.RuntimeHelpers.GetHashCode(_curve),
                            _first, _last, _tolerance, _s1, _s2, _mother, _dsIndex);

    public static bool operator ==(in DsCurve left, in DsCurve right) => left.Equals(right);

    public static bool operator !=(in DsCurve left, in DsCurve right) => !left.Equals(right);

    /// <inheritdoc/>
    public override readonly string ToString()
        => "TopOpeBRepDS_Curve(" + (_curve is null ? "null" : global::Kerf.Engine.Foundation.OcctTypeNames.Of(_curve.GetType())) + ", " + _tolerance.ToString("R", System.Globalization.CultureInfo.InvariantCulture)
           + (_rangeDefined ? ", [" + _first.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + ", " + _last.ToString("R", System.Globalization.CultureInfo.InvariantCulture) + "]" : "")
           + ", index " + _dsIndex.ToString(System.Globalization.CultureInfo.InvariantCulture) + ")";
}
