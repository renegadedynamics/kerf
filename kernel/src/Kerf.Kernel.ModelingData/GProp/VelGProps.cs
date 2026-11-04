// Port of: src/ModelingData/TKGeomBase/GProp/GProp_VelGProps.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_VelGProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Measurement;

/// <summary>Global properties of solids bounded by elementary surfaces (gp cylinders, cones, spheres and tori).</summary>
/// <remarks>A working object: <c>SetLocation</c> and <c>Perform</c> are public, as in OCCT.</remarks>
internal class VelGProps : Properties
{
    public VelGProps()
    {
    }

    public void SetLocation(in Point3D vLocation)
    {
        _loc = vLocation;
    }

    public VelGProps(in Cylinder s, double alpha1, double alpha2, double z1, double z2, in Point3D vLocation)
    {
        SetLocation(vLocation);
        Perform(s, alpha1, alpha2, z1, z2);
    }

    public VelGProps(in Cone s, double alpha1, double alpha2, double z1, double z2, in Point3D vLocation)
    {
        SetLocation(vLocation);
        Perform(s, alpha1, alpha2, z1, z2);
    }

    public VelGProps(in Sphere s, double teta1, double teta2, double alpha1, double alpha2, in Point3D vLocation)
    {
        SetLocation(vLocation);
        Perform(s, teta1, teta2, alpha1, alpha2);
    }

    public VelGProps(in Torus s, double teta1, double teta2, double alpha1, double alpha2, in Point3D vLocation)
    {
        SetLocation(vLocation);
        Perform(s, teta1, teta2, alpha1, alpha2);
    }

    public void Perform(in Cylinder s, double alpha1, double alpha2, double z1, double z2)
    {
        // the sector between the patch and the axis: (rho cos a, rho sin a, z), rho in [0, R], volume rho drho da dz
        // (decision 1471; OCCT's axial moment was twice too large)
        double r = s.Radius;
        double dz = z2 - z1, z12 = (z2 * z2 - z1 * z1) / 2.0, z13 = (z2 * z2 * z2 - z1 * z1 * z1) / 3.0;
        CoordinateSystem aPos = s.Position;
        _dim = global::Kerf.Engine.Measurement.GProp.FromRevolution(aPos.Location, aPos.XDirection.XYZ(), aPos.YDirection.XYZ(), aPos.Direction.XYZ(),
                                                                    alpha1, alpha2, r * r / 2.0 * dz, r * r * r / 3.0 * dz, r * r * r * r / 4.0 * dz,
                                                                    r * r / 2.0 * z12, r * r * r / 3.0 * z12, r * r / 2.0 * z13, _loc, out _g,
                                                                    out _inertia);
    }

    public void Perform(in Cone s, double alpha1, double alpha2, double z1, double z2)
    {
        // the sector between the patch and the axis: (sigma cos a, sigma sin a, v cos t), sigma in [0, R + v sin t],
        // volume sigma dsigma da cos t dv (decision 1471; OCCT's volume was not the frustum's)
        double cnt = Math.Cos(s.SemiAngle), snt = Math.Sin(s.SemiAngle), r = s.RefRadius;
        double P(int a, int b) => global::Kerf.Engine.Measurement.GProp.PolyMoment(r, snt, a, b, z1, z2);
        CoordinateSystem aPos = s.Position;
        _dim = global::Kerf.Engine.Measurement.GProp.FromRevolution(aPos.Location, aPos.XDirection.XYZ(), aPos.YDirection.XYZ(), aPos.Direction.XYZ(),
                                                                    alpha1, alpha2, cnt * P(2, 0) / 2.0, cnt * P(3, 0) / 3.0, cnt * P(4, 0) / 4.0,
                                                                    cnt * cnt * P(2, 1) / 2.0, cnt * cnt * P(3, 1) / 3.0,
                                                                    cnt * cnt * cnt * P(2, 2) / 2.0, _loc, out _g, out _inertia);
    }

    public void Perform(in Sphere s, double teta1, double teta2, double alpha1, double alpha2)
    {
        // the sector between the patch and the centre: rho (cos v cos u, cos v sin u, sin v), rho in [0, r], volume
        // rho^2 cos v drho du dv (decision 1471)
        double r = s.Radius;
        double T(int k, int b) => global::Kerf.Engine.Measurement.GProp.TrigMoment(k, b, alpha1, alpha2);
        CoordinateSystem aPos = s.Position;
        double r3 = r * r * r / 3.0, r4 = r * r * r * r / 4.0, r5 = r * r * r * r * r / 5.0;
        _dim = global::Kerf.Engine.Measurement.GProp.FromRevolution(aPos.Location, aPos.XDirection.XYZ(), aPos.YDirection.XYZ(), aPos.Direction.XYZ(),
                                                                    teta1, teta2, r3 * T(1, 0), r4 * T(2, 0), r5 * T(3, 0), r4 * T(1, 1),
                                                                    r5 * T(2, 1), r5 * T(1, 2), _loc, out _g, out _inertia);
    }

    public void Perform(in Torus s, double teta1, double teta2, double alpha1, double alpha2)
    {
        // the sector of the tube between the patch and the tube's axis: (w cos u, w sin u, rho sin v), w = R + rho cos
        // v, rho in [0, r], volume rho w drho du dv (decision 1471; OCCT read cos and sin of alpha2 for teta2)
        double rMax = s.MajorRadius, rmin = s.MinorRadius;
        double M(int e, int n, int b) => global::Kerf.Engine.Measurement.GProp.TorusMoment(rMax, rmin, e, n, b, alpha1, alpha2, true);
        _dim = global::Kerf.Engine.Measurement.GProp.FromRevolution(s.Location, s.XAxis.Direction.XYZ(), s.YAxis.Direction.XYZ(),
                                                                    s.Axis.Direction.XYZ(), teta1, teta2, M(0, 1, 0), M(0, 2, 0), M(0, 3, 0),
                                                                    M(1, 1, 1), M(1, 2, 1), M(2, 1, 2), _loc, out _g, out _inertia);
    }
}
