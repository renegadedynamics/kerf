// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_PntFace.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Features.Local;

/// <summary>
/// An intersection point of a curve with a face of a shape: the point, the face, the transition, the parameter on the
/// curve and the parameters on the face.
/// </summary>
/// <remarks><c>default(LocOpe_PntFace)</c> is OCCT's empty point (its orientation is FORWARD).</remarks>
internal partial struct PntFace : IEquatable<PntFace>
{
    private Point3D _pnt;
    private Face _face;
    private Orientation _ori;
    private double _par;
    private double _uPar;
    private double _vPar;

    /// <summary>Empty constructor. Useful only for the list.</summary>
    public PntFace()
    {
        _pnt = new Point3D();
        _face = new Face();
        _ori = global::Kerf.Topology.Orientation.Forward; // OCCT: uninitialised
        _par = 0.0;
        _uPar = 0.0;
        _vPar = 0.0;
    }

    public PntFace(in Point3D p, in Face f, Orientation or, double param, double uPar, double vPar)
    {
        _pnt = p;
        _face = f;
        _ori = or;
        _par = param;
        _uPar = uPar;
        _vPar = vPar;
    }

    public readonly Point3D Pnt() => _pnt;

    public readonly Face Face() => _face;

    public readonly Orientation Orientation() => _ori;

    [System.Diagnostics.CodeAnalysis.UnscopedRef]
    internal ref Orientation ChangeOrientation() => ref _ori;

    public readonly double Parameter() => _par;

    public readonly double UParameter() => _uPar;

    public readonly double VParameter() => _vPar;

    /// <summary>A copy with the orientation <paramref name="ori"/>.</summary>
    public readonly PntFace WithOrientation(Orientation ori)
    {
        PntFace copy = this;
        copy._ori = ori;
        return copy;
    }

    public readonly bool Equals(PntFace other) =>
        _pnt == other._pnt && _face == other._face && _face.Orientation == other._face.Orientation
        && _ori == other._ori && _par.Equals(other._par) && _uPar.Equals(other._uPar) && _vPar.Equals(other._vPar);

    public override readonly bool Equals(object? obj) => obj is PntFace other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pnt, _face, _ori, _par, _uPar, _vPar);

    public static bool operator ==(in PntFace left, in PntFace right) => left.Equals(right);

    public static bool operator !=(in PntFace left, in PntFace right) => !left.Equals(right);

    public override readonly string ToString() =>
        Constants.Format("LocOpe_PntFace", _pnt.X, _pnt.Y, _pnt.Z, (double)_ori, _par, _uPar, _vPar);
}
