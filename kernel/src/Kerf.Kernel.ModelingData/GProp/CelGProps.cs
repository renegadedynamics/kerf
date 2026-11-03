// Port of: src/ModelingData/TKGeomBase/GProp/GProp_CelGProps.hxx
// Port of: src/ModelingData/TKGeomBase/GProp/GProp_CelGProps.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Measurement;

/// <summary>Global properties of bounded elementary curves in 3D space (gp lines and circles).</summary>
/// <remarks>A working object: <c>SetLocation</c> and <c>Perform</c> are public, as in OCCT.</remarks>
public class CelGProps : Properties
{
    public CelGProps()
    {
    }

    public void SetLocation(in Point3D cLocation)
    {
        _loc = cLocation;
    }

    public void Perform(in Circle3D c, double u1, double u2)
    {
        // a ring of radius R about the circle's axis (decision 1471)
        double ray = c.Radius;
        double aU1 = Math.Min(u1, u2), aU2 = Math.Max(u1, u2);
        _dim = global::Kerf.Engine.Measurement.GProp.FromRevolution(c.Location, c.XAxis.Direction.XYZ(), c.YAxis.Direction.XYZ(),
                                                                    c.Axis.Direction.XYZ(), aU1, aU2, ray, ray * ray, ray * ray * ray,
                                                                    0.0, 0.0, 0.0, _loc, out _g, out _inertia);
    }

    public void Perform(in Line3D c, double u1, double u2)
    {
        // the segment's moments along its direction, about its location (decision 1471)
        Axis3D pos = c.Position;
        double aU1 = Math.Min(u1, u2), aU2 = Math.Max(u1, u2);
        Xyz v = pos.Direction.XYZ();
        _dim = aU2 - aU1;
        global::Kerf.Engine.Measurement.GProp.FromLocalMoments(pos.Location, v, v, v, _dim, new Xyz((aU2 * aU2 - aU1 * aU1) / 2.0, 0.0, 0.0),
                                                               [(aU2 * aU2 * aU2 - aU1 * aU1 * aU1) / 3.0, 0.0, 0.0, 0.0, 0.0, 0.0], _loc, out _g,
                                                               out _inertia);
    }

    public CelGProps(in Circle3D c, in Point3D cLocation)
    {
        SetLocation(cLocation);
        Perform(c, 0.0, 2.0 * M_PI);
    }

    public CelGProps(in Circle3D c, double u1, double u2, in Point3D cLocation)
    {
        SetLocation(cLocation);
        Perform(c, u1, u2);
    }

    public CelGProps(in Line3D c, double u1, double u2, in Point3D cLocation)
    {
        SetLocation(cLocation);
        Perform(c, u1, u2);
    }
}
