// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_FaceInterference.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_FaceInterference.lxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_FaceInterference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using System.Diagnostics.CodeAnalysis;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Data;

/// <summary>
/// The interference of a fillet with one of its faces: the index of the curve of intersection in the data
/// structure, its transition, its pcurves on the face and on the fillet, and its parameter range.
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. <see cref="SurfData"/> holds two of them and the fillet builders change
/// them in place through its <c>Change*</c> accessors. The pcurves are the fillet's own working geometry.
/// </remarks>
internal partial struct FaceInterference : IEquatable<FaceInterference>
{
    private double _firstParam;
    private double _lastParam;
    private Curve2D? _pCurveOnFace;
    private Curve2D? _pCurveOnSurf;
    private int _lineindex;
    private Orientation _lineTransition;

    public FaceInterference()
    {
        _firstParam = 0.0;
        _lastParam = 0.0;
        _lineindex = 0;
        _lineTransition = Forward;
    }

    internal void SetInterference(int lineIndex, Orientation trans, Curve2D? pCurv1, Curve2D? pCurv2)
    {
        _lineindex = lineIndex;
        _lineTransition = trans;
        _pCurveOnFace = pCurv1;
        _pCurveOnSurf = pCurv2;
    }

    internal void SetTransition(Orientation trans)
    {
        _lineTransition = trans;
    }

    internal void SetFirstParameter(double u1)
    {
        _firstParam = u1;
    }

    internal void SetLastParameter(double u1)
    {
        _lastParam = u1;
    }

    internal void SetParameter(double u1, bool isFirst)
    {
        if (isFirst)
        {
            SetFirstParameter(u1);
        }
        else
        {
            SetLastParameter(u1);
        }
    }

    internal void SetLineIndex(int i)
    {
        _lineindex = i;
    }

    /// <summary>OCCT's <c>ChangePCurveOnFace</c>: the pcurve on the face, for reading and writing.</summary>
    [UnscopedRef]
    internal ref Curve2D? ChangePCurveOnFace() => ref _pCurveOnFace;

    /// <summary>OCCT's <c>ChangePCurveOnSurf</c>: the pcurve on the fillet, for reading and writing.</summary>
    [UnscopedRef]
    internal ref Curve2D? ChangePCurveOnSurf() => ref _pCurveOnSurf;

    public readonly FaceInterference WithInterference(int lineIndex, Orientation trans,
                                                             Curve2D? pCurv1, Curve2D? pCurv2)
    {
        FaceInterference copy = this;
        copy.SetInterference(lineIndex, trans, pCurv1, pCurv2);
        return copy;
    }

    public readonly FaceInterference WithTransition(Orientation trans)
    {
        FaceInterference copy = this;
        copy.SetTransition(trans);
        return copy;
    }

    public readonly FaceInterference WithFirstParameter(double u1)
    {
        FaceInterference copy = this;
        copy.SetFirstParameter(u1);
        return copy;
    }

    public readonly FaceInterference WithLastParameter(double u1)
    {
        FaceInterference copy = this;
        copy.SetLastParameter(u1);
        return copy;
    }

    public readonly FaceInterference WithParameter(double u1, bool isFirst)
    {
        FaceInterference copy = this;
        copy.SetParameter(u1, isFirst);
        return copy;
    }

    public readonly FaceInterference WithLineIndex(int i)
    {
        FaceInterference copy = this;
        copy.SetLineIndex(i);
        return copy;
    }

    /// <summary>The index of the curve of intersection in the data structure.</summary>
    public readonly int LineIndex()
    {
        return _lineindex;
    }

    public readonly Orientation Transition()
    {
        return _lineTransition;
    }

    public readonly Curve2D? PCurveOnFace()
    {
        return _pCurveOnFace;
    }

    public readonly Curve2D? PCurveOnSurf()
    {
        return _pCurveOnSurf;
    }

    public readonly double FirstParameter()
    {
        return _firstParam;
    }

    public readonly double LastParameter()
    {
        return _lastParam;
    }

    public readonly double Parameter(bool isFirst)
    {
        if (isFirst)
        {
            return FirstParameter();
        }
        else
        {
            return LastParameter();
        }
    }

    public readonly bool Equals(FaceInterference other)
        => _firstParam.Equals(other._firstParam) && _lastParam.Equals(other._lastParam)
           && ReferenceEquals(_pCurveOnFace, other._pCurveOnFace) && ReferenceEquals(_pCurveOnSurf, other._pCurveOnSurf)
           && _lineindex == other._lineindex && _lineTransition == other._lineTransition;

    public override readonly bool Equals(object? obj) => obj is FaceInterference other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_firstParam, _lastParam, _lineindex, _lineTransition);

    public static bool operator ==(in FaceInterference left, in FaceInterference right)
        => left.Equals(right);

    public static bool operator !=(in FaceInterference left, in FaceInterference right)
        => !left.Equals(right);

    public override readonly string ToString()
        => Constants.Format("ChFiDS_FaceInterference", _lineindex, (int)_lineTransition, _firstParam, _lastParam);
}
