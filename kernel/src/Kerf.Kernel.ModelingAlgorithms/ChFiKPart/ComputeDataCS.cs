// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_CS.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiKPart/ChFiKPart_ComputeData_CS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets.Analytic;

/// <summary>The guide circle and cylinder of a toric corner fillet.</summary>
internal static class ComputeDataCS
{
    /// <summary>The circle of radius <paramref name="r"/> from the point <paramref name="p1S2"/> to
    /// <paramref name="p2S2"/> of <paramref name="s2"/>, in the plane normal to the plane <paramref name="s1"/>, and
    /// the cylinder around it; [<paramref name="first"/>, <paramref name="last"/>] its parameters.</summary>
    internal static void ChFiKPart_CornerSpine(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, in Point2D p1S1,
                                               in Point2D p2S1, in Point2D p1S2, in Point2D p2S2, double r,
                                               ref Cylinder3D cyl, ref Circle3D circ, out double first,
                                               out double last)
    {
        CoordinateSystem ax = s1.Plane().Position;
        Vector3D v1 = new(ax.XDirection);
        Vector3D v2 = new(ax.YDirection);
        s2.D1(p1S2.X, p1S2.Y, out Point3D p, out Vector3D du, out Vector3D dv);
        Vector3D v = new(p, s1.Value(p1S1.X, p1S1.Y));
        v = v.Dot(v1) * v1 + v.Dot(v2) * v2;
        v = v.Normalized();
        Point3D p2 = s2.Value(p2S2.X, p2S2.Y);
        Vector3D vorien = new(p, p2);
        Point3D cent = new();
        Direction3D dx = new(v);
        if (v.Dot(vorien) >= 0.0)
        {
            cent = cent.WithCoord(p.X + r * v.X, p.Y + r * v.Y, p.Z + r * v.Z);
            dx = dx.Reversed();
        }
        else
        {
            cent = cent.WithCoord(p.X - r * v.X, p.Y - r * v.Y, p.Z - r * v.Z);
        }
        Direction3D dy = new(new Vector3D(cent, p2));
        dy = (dx ^ dy) ^ dx;
        Frame3D circax2 = new(cent, dx ^ dy, dx);
        CoordinateSystem cylax3 = new(circax2);
        if ((du ^ dv).Dot(new Vector3D(dx)) < 0.0)
        {
            cylax3 = cylax3.ZReversed();
        }
        first = 0.0;
        last = global::Kerf.Engine.Geometry.ElCLib.CircleParameter(circax2, p2);
        circ = circ.WithPosition(circax2);
        circ = circ.WithRadius(r);
        cyl = cyl.WithPosition(cylax3);
        cyl = cyl.WithRadius(r);
    }
}
