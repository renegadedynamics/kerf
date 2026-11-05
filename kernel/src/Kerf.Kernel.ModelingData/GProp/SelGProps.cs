// Port of: src/ModelingData/TKGeomBase/GProp/GProp_SelGProps.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_SelGProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Measurement;

/// <summary>Global properties of bounded elementary surfaces in 3D space (gp cylinders, cones, spheres and tori).</summary>
/// <remarks>A working object: <c>SetLocation</c> and <c>Perform</c> are public, as in OCCT.</remarks>
internal class SelGProps : Properties
{
    public SelGProps()
    {
    }

    public void SetLocation(in Point3D sLocation)
    {
        _loc = sLocation;
    }

    public void Perform(in Cylinder3D s, double alpha1, double alpha2, double z1, double z2)
    {
        // the patch (R cos a, R sin a, z), area R da dz (decision 1471)
        double r = s.Radius;
        CoordinateSystem aPos = s.Position;
        _dim = global::Kerf.Engine.Measurement.GProp.FromRevolution(aPos.Location, aPos.XDirection.XYZ(), aPos.YDirection.XYZ(), aPos.Direction.XYZ(),
                                                                    alpha1, alpha2, r * (z2 - z1), r * r * (z2 - z1), r * r * r * (z2 - z1),
                                                                    r * (z2 * z2 - z1 * z1) / 2.0, r * r * (z2 * z2 - z1 * z1) / 2.0,
                                                                    r * (z2 * z2 * z2 - z1 * z1 * z1) / 3.0, _loc, out _g, out _inertia);
    }

    public void Perform(in Cone3D s, double alpha1, double alpha2, double z1, double z2)

    {
        // the patch (rho cos a, rho sin a, v cos t), rho = R + v sin t, v along the generatrix, area rho da dv
        // (decision 1471; OCCT's area had an extra cos t)
        double cnt = Math.Cos(s.SemiAngle), snt = Math.Sin(s.SemiAngle), r = s.RefRadius;
        double P(int a, int b) => global::Kerf.Engine.Measurement.GProp.PolyMoment(r, snt, a, b, z1, z2);
        CoordinateSystem aPos = s.Position;
        _dim = global::Kerf.Engine.Measurement.GProp.FromRevolution(aPos.Location, aPos.XDirection.XYZ(), aPos.YDirection.XYZ(), aPos.Direction.XYZ(),
                                                                    alpha1, alpha2, P(1, 0), P(2, 0), P(3, 0), cnt * P(1, 1), cnt * P(2, 1),
                                                                    cnt * cnt * P(1, 2), _loc, out _g, out _inertia);
    }

    public void Perform(in Sphere3D s, double teta1, double teta2, double alpha1, double alpha2)
    {
        // the patch r (cos v cos u, cos v sin u, sin v), u in [teta1, teta2], v in [alpha1, alpha2], area r^2 cos v
        // du dv (decision 1471)
        double r = s.Radius;
        double T(int k, int b) => global::Kerf.Engine.Measurement.GProp.TrigMoment(k, b, alpha1, alpha2);
        CoordinateSystem aPos = s.Position;
        double r2 = r * r, r3 = r2 * r, r4 = r3 * r;
        _dim = global::Kerf.Engine.Measurement.GProp.FromRevolution(aPos.Location, aPos.XDirection.XYZ(), aPos.YDirection.XYZ(), aPos.Direction.XYZ(),
                                                                    teta1, teta2, r2 * T(1, 0), r3 * T(2, 0), r4 * T(3, 0), r3 * T(1, 1),
                                                                    r4 * T(2, 1), r4 * T(1, 2), _loc, out _g, out _inertia);
    }

    public void Perform(in Torus3D s, double teta1, double teta2, double alpha1, double alpha2)
    {
        // the patch (w cos u, w sin u, r sin v), w = R + r cos v, area r w du dv (decision 1471)
        double rMax = s.MajorRadius, rmin = s.MinorRadius;
        double M(int e, int n, int b) => global::Kerf.Engine.Measurement.GProp.TorusMoment(rMax, rmin, e, n, b, alpha1, alpha2, false);
        _dim = global::Kerf.Engine.Measurement.GProp.FromRevolution(s.Location, s.XAxis.Direction.XYZ(), s.YAxis.Direction.XYZ(),
                                                                    s.Axis.Direction.XYZ(), teta1, teta2, M(1, 1, 0), M(1, 2, 0), M(1, 3, 0),
                                                                    M(2, 1, 1), M(2, 2, 1), M(3, 1, 2), _loc, out _g, out _inertia);
    }

    public SelGProps(in Cone3D s, double alpha1, double alpha2, double z1, double z2, in Point3D sLocation)
    {
        SetLocation(sLocation);
        Perform(s, alpha1, alpha2, z1, z2);
    }

    public SelGProps(in Cylinder3D s, double alpha1, double alpha2, double z1, double z2, in Point3D sLocation)
    {
        SetLocation(sLocation);
        Perform(s, alpha1, alpha2, z1, z2);
    }

    public SelGProps(in Sphere3D s, double teta1, double teta2, double alpha1, double alpha2, in Point3D sLocation)
    {
        SetLocation(sLocation);
        Perform(s, teta1, teta2, alpha1, alpha2);
    }

    public SelGProps(in Torus3D s, double teta1, double teta2, double alpha1, double alpha2, in Point3D sLocation)
    {
        SetLocation(sLocation);
        Perform(s, teta1, teta2, alpha1, alpha2);
    }
}
