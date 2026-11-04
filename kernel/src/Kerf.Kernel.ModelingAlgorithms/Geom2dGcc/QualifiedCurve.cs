// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_QualifiedCurve.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_QualifiedCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// A 2D curve with a qualifier: whether the solutions computed from it enclose it (GccEnt_enclosing), are enclosed
/// by it (GccEnt_enclosed), are outside it (GccEnt_outside), or may be any of these (GccEnt_unqualified). The interior
/// of a curve is the left-hand side of the curve in relation to its orientation.
/// </summary>
/// <remarks>
/// A value, as OCCT uses it (no setters). OCCT stores a copy of the adaptor and returns copies: the port keeps a
/// shallow copy (<see cref="Curve2DAdaptorBase.ShallowCopy"/>: the curve is shared, the evaluation caches are not)
/// and <see cref="Qualified"/> returns a new one each time, so no caller or solver shares the evaluation caches of
/// another (decision 797). Two qualified curves are equal when they have the same qualifier, the same Geom2d curve
/// object and the same parameter range. <c>default</c> has no curve.
/// </remarks>
internal readonly struct QualifiedCurve : IEquatable<QualifiedCurve>
{
    private readonly Qualifier _theQualifier;
    private readonly CurveAdaptor2D _theQualified;

    /// <summary>Assigns the qualifier to the curve (copied).</summary>
    public QualifiedCurve(CurveAdaptor2D curve, Qualifier qualifier)
    {
        _theQualified = (CurveAdaptor2D)curve.ShallowCopy();
        _theQualifier = qualifier;
    }

    /// <summary>The curve to which the qualifier is assigned: a new adaptor on the same curve and range.</summary>
    public CurveAdaptor2D Qualified()
    {
        return (CurveAdaptor2D)_theQualified.ShallowCopy();
    }

    /// <summary>The qualifier.</summary>
    public Qualifier Qualifier()
    {
        return _theQualifier;
    }

    /// <summary>True if the solutions are unqualified.</summary>
    public bool IsUnqualified
    {
        get
        {
            return _theQualifier == global::Kerf.Engine.Construction.Constraints.Qualifier.Unqualified;
        }
    }

    /// <summary>True if the solutions enclose the curve.</summary>
    public bool IsEnclosing
    {
        get
        {
            return _theQualifier == global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosing;
        }
    }

    /// <summary>True if the curve encloses the solutions.</summary>
    public bool IsEnclosed
    {
        get
        {
            return _theQualifier == global::Kerf.Engine.Construction.Constraints.Qualifier.Enclosed;
        }
    }

    /// <summary>True if the solutions and the curve are outside each other.</summary>
    public bool IsOutside
    {
        get
        {
            return _theQualifier == global::Kerf.Engine.Construction.Constraints.Qualifier.Outside;
        }
    }

    public bool Equals(QualifiedCurve other)
    {
        if (_theQualifier != other._theQualifier)
        {
            return false;
        }
        if (_theQualified is null || other._theQualified is null)
        {
            return _theQualified is null && other._theQualified is null;
        }
        return ReferenceEquals(_theQualified.Curve(), other._theQualified.Curve())
               && _theQualified.FirstParameter().Equals(other._theQualified.FirstParameter())
               && _theQualified.LastParameter().Equals(other._theQualified.LastParameter());
    }

    public override bool Equals(object? obj) => obj is QualifiedCurve other && Equals(other);

    public override int GetHashCode()
        => _theQualified is null
               ? HashCode.Combine(_theQualifier)
               : HashCode.Combine(_theQualifier, _theQualified.Curve(), _theQualified.FirstParameter(), _theQualified.LastParameter());

    public static bool operator ==(in QualifiedCurve left, in QualifiedCurve right) => left.Equals(right);

    public static bool operator !=(in QualifiedCurve left, in QualifiedCurve right) => !left.Equals(right);

    public override string ToString()
        => "Geom2dGcc_QualifiedCurve(" + (_theQualified is null
                                       ? "null"
                                       : _theQualified.GetType().ToString() + " "
                                         + Constants.Format("range", _theQualified.FirstParameter(), _theQualified.LastParameter()))
           + ", " + global::Kerf.Engine.Construction.Constraints.GccEnt.PositionToString(_theQualifier) + ")";
}
