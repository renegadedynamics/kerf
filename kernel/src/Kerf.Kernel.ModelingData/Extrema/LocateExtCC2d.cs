// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_LocateExtCC2d.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_LocateExtCC2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Occt;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The extremum distance between two 2D curves nearest the starting parameters (U0, V0), by
/// <see cref="LocEcc2d"/> with the curves' resolutions of Precision::Confusion() as tolerances.
/// </summary>
/// <remarks>Computed by its constructor.</remarks>
internal class LocateExtCC2d
{
    private readonly bool _done;
    private readonly double _sqDist;
    private readonly POnCurv2d _point1 = new();
    private readonly POnCurv2d _point2 = new();

    /// <summary>The extremum between C1 and C2 nearest (U0, V0).</summary>
    public LocateExtCC2d(Curve2DAdaptorBase c1, Curve2DAdaptorBase c2, double u0, double v0)
    {
        _sqDist = RealLast();
        double tolU = Curve2dTool.Resolution(c1, global::Kerf.Engine.Foundation.Precision.Confusion());
        double tolV = Curve2dTool.Resolution(c2, global::Kerf.Engine.Foundation.Precision.Confusion());
        POnCurv2d p1, p2;

        // Not implemented for now: the call to Geom2dExtrema_ExtCC.

        LocEcc2d xtrem = new(c1, c2, u0, v0, tolU, tolV);
        // Exploitation

        _done = xtrem.IsDone;
        if (xtrem.IsDone)
        {
            _sqDist = xtrem.SquareDistance();
            xtrem.Point(out p1, out p2);
            _point1 = p1;
            _point2 = p2;
        }
    }

    /// <summary>True if the distance is found.</summary>
    public bool IsDone => _done;

    /// <summary>The square distance of the extremum.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double SquareDistance()
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        return _sqDist;
    }

    /// <summary>The points of the extremum, on the first and on the second curve.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public void Point(out POnCurv2d p1, out POnCurv2d p2)
    {
        if (!IsDone)
        {
            throw new NotDoneException();
        }
        p1 = _point1;
        p2 = _point2;
    }
}
