// Port of: src/ModelingData/TKG3d/Geom/Geom_SweptSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_SweptSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// A surface built by sweeping a curve: the surface of revolution and the surface of linear extrusion.
/// </summary>
public abstract class SweptSurface : Surface
{
    protected Curve3D? _basisCurve;
    protected Direction3D _direction = new Direction3D();
    protected Continuity _smooth;

    /// <summary>The continuity of the surface.</summary>
    public override Continuity Continuity => _smooth;

    /// <summary>The reference direction: the axis of revolution, or the direction of extrusion.</summary>
    public Direction3D Direction => _direction;

    /// <summary>The swept curve: the revolved curve, or the extruded curve.</summary>
    /// <remarks>
    /// The surface's own copy, frozen and shared, as OCCT returns its handle; the same object each time. The
    /// surface's mutators copy it again before changing it (decision 159). Set by the constructors of the derived
    /// classes; null only while one is running.
    /// </remarks>
    public Curve3D BasisCurve
    {
        get
        {
            _basisCurve!.Freeze();
            return _basisCurve;
        }
    }

    /// <summary>
    /// OCCT.NET: the basis curve may have been handed out (and frozen) by <see cref="BasisCurve"/>; the derived
    /// classes' mutators call this to copy it before changing it.
    /// </summary>
    private protected void OwnBasisCurve()
    {
        if (_basisCurve!.IsFrozen)
        {
            _basisCurve = (Curve3D)_basisCurve.Copy();
        }
    }
}
