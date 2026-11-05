// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_FuncConeLSDist.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_FuncConeLSDist.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// The least-squares criterion for fitting a cone of given axis direction to a set of points: the sum of the
/// squared distances from the points to the cone. The variables are the location (X(1) to X(3)), the semi-angle
/// X(4) and the reference radius X(5).
/// </summary>
internal sealed class FuncConeLSDist : MultipleVarFunction
{
    private NCollection_HArray1<Xyz>? _points;
    private Direction3D _dir = new Direction3D();

    /// <summary>A function without points (set them with <see cref="SetPoints"/>).</summary>
    public FuncConeLSDist()
    {
    }

    /// <summary>The function for thePoints (kept, not copied) and the axis direction theDir.</summary>
    public FuncConeLSDist(NCollection_HArray1<Xyz> points, in Direction3D dir)
    {
        _points = points;
        _dir = dir;
    }

    /// <summary>Sets the points (kept, not copied).</summary>
    public void SetPoints(NCollection_HArray1<Xyz> points) => _points = points;

    /// <summary>Sets the axis direction.</summary>
    public void SetDir(in Direction3D dir) => _dir = dir;

    /// <summary>5: the location, the semi-angle and the radius.</summary>
    public override int VariableCount => 5;

    /// <summary>The sum of the squared distances from the points to the cone X.</summary>
    public override bool Value(math_Vector x, out double f)
    {
        var loc = new Point3D(x[1], x[2], x[3]);
        double semiAngle = x[4], r = x[5];
        var pos = new CoordinateSystem(loc, _dir);
        f = 0.0;
        int i;
        for (i = _points!.Lower(); i <= _points.Upper(); ++i)
        {
            double u, v;
            var pi = new Point3D(_points.Value(i));
            global::Kerf.Engine.Geometry.ElSLib.ConeParameters(pos, r, semiAngle, pi, out u, out v);
            Point3D pp;
            global::Kerf.Engine.Geometry.ElSLib.ConeD0(u, v, pos, r, semiAngle, out pp);
            f += pi.SquareDistanceTo(pp);
        }

        return true;
    }
}
