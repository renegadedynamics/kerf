// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools_SurfaceSet.hxx
// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools_SurfaceSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Exchange.Brep;

/// <summary>
/// A set of Geom surfaces, indexed from 1, written and read in OCCT's text format (the "Surfaces" section of a
/// <c>.brep</c> file).
/// </summary>
/// <remarks>
/// As <see cref="CurveSet"/>: OCCT's exact output, C++'s extraction when reading, surfaces read are frozen,
/// and surfaces that can't be read are one null entry. As in OCCT, <see cref="Add"/> and <see cref="Index"/> accept
/// null (the null entry).
/// </remarks>
public class SurfaceSet
{
    private const int Plane = 1;
    private const int Cylinder = 2;
    private const int Cone = 3;
    private const int Sphere = 4;
    private const int Torus = 5;
    private const int Linearextrusion = 6;
    private const int Revolution = 7;
    private const int Bezier = 8;
    private const int Bspline = 9;
    private const int Rectangular = 10;
    private const int Offset = 11;

    // OCCT's NCollection_IndexedMap of handles, which hashes by address and can hold one null handle
    private readonly NCollection_IndexedMap<object> _map = new(ReferenceEqualityComparer.Instance);

    /// <summary>An empty set of surfaces.</summary>
    public SurfaceSet()
    {
    }

    /// <summary>Clears the content of the set.</summary>
    public void Clear()
    {
        _map.Clear();
    }

    /// <summary>Adds a surface to the set (once) and returns its index. Null is added too, as in OCCT.</summary>
    public int Add(Surface? s)
    {
        // OCCT: adds the null handle too, as one entry (occt-issues.md)
        return _map.Add((object?)s ?? CurveSet.NullKey);
    }

    /// <summary>The surface of index I, or null if there is none.</summary>
    public Surface? Surface(int i)
    {
        if (i <= 0 || i > _map.Extent())
        {
            return null;
        }

        return _map[i] as Surface;
    }

    /// <summary>The index of the surface, 0 if it isn't in the set (null finds the null entry, as in OCCT).</summary>
    public int Index(Surface? s)
    {
        return _map.FindIndex((object?)s ?? CurveSet.NullKey);
    }

    private static void Out(TextWriter os, double value) => StreamUtilities.Write(os, value);

    private static void Out(TextWriter os, int value) => StreamUtilities.Write(os, value);

    private static string Setw(int width, int value) => StreamUtilities.Setw(width, value.ToString(CultureInfo.InvariantCulture));

    private static void Print(in Point3D p, TextWriter os, bool compact)
    {
        Out(os, p.X());
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, p.Y());
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, p.Z());
        os.Write(" ");
    }

    private static void Print(in Direction3D d, TextWriter os, bool compact)
    {
        Out(os, d.X());
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, d.Y());
        if (!compact)
        {
            os.Write(",");
        }

        os.Write(" ");
        Out(os, d.Z());
        os.Write(" ");
    }

    private static void Print(PlaneSurface s, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Plane);
            os.Write(" ");
        }
        else
        {
            os.Write("Plane");
        }

        Plane3D p = s.Pln();
        if (!compact)
        {
            os.Write("\n  Origin :");
        }

        Print(p.Location(), os, compact);
        if (!compact)
        {
            os.Write("\n  Axis   :");
        }

        Print(p.Axis().Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  XAxis  :");
        }

        Print(p.XAxis().Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  YAxis  :");
        }

        Print(p.YAxis().Direction(), os, compact);
        os.Write("\n");
        if (!compact)
        {
            os.Write("\n");
        }
    }

    private static void Print(CylindricalSurface s, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, global::Kerf.Engine.Exchange.Brep.SurfaceSet.Cylinder);
            os.Write(" ");
        }
        else
        {
            os.Write("CylindricalSurface");
        }

        Cylinder p = s.Cylinder();
        if (!compact)
        {
            os.Write("\n  Origin :");
        }

        Print(p.Location(), os, compact);
        if (!compact)
        {
            os.Write("\n  Axis   :");
        }

        Print(p.Axis().Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  XAxis  :");
        }

        Print(p.XAxis().Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  YAxis  :");
        }

        Print(p.YAxis().Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  Radius :");
        }

        Out(os, p.Radius());
        os.Write("\n");
        if (!compact)
        {
            os.Write("\n");
        }
    }

    private static void Print(ConicalSurface s, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, global::Kerf.Engine.Exchange.Brep.SurfaceSet.Cone);
            os.Write(" ");
        }
        else
        {
            os.Write("ConicalSurface");
        }

        Cone p = s.Cone();
        if (!compact)
        {
            os.Write("\n  Origin :");
        }

        Print(p.Location(), os, compact);
        if (!compact)
        {
            os.Write("\n  Axis   :");
        }

        Print(p.Axis().Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  XAxis  :");
        }

        Print(p.XAxis().Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  YAxis  :");
        }

        Print(p.YAxis().Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  Radius :");
        }

        Out(os, p.RefRadius());
        os.Write("\n");
        if (!compact)
        {
            os.Write("\n  Angle :");
        }

        Out(os, p.SemiAngle());
        os.Write("\n");
        if (!compact)
        {
            os.Write("\n");
        }
    }

    private static void Print(SphericalSurface s, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, global::Kerf.Engine.Exchange.Brep.SurfaceSet.Sphere);
            os.Write(" ");
        }
        else
        {
            os.Write("SphericalSurface");
        }

        Sphere p = s.Sphere();
        if (!compact)
        {
            os.Write("\n  Center :");
        }

        Print(p.Location(), os, compact);
        if (!compact)
        {
            os.Write("\n  Axis   :");
        }

        Print(p.Position().Axis().Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  XAxis  :");
        }

        Print(p.XAxis().Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  YAxis  :");
        }

        Print(p.YAxis().Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  Radius :");
        }

        Out(os, p.Radius());
        os.Write("\n");
        if (!compact)
        {
            os.Write("\n");
        }
    }

    private static void Print(ToroidalSurface s, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, global::Kerf.Engine.Exchange.Brep.SurfaceSet.Torus);
            os.Write(" ");
        }
        else
        {
            os.Write("ToroidalSurface");
        }

        Torus p = s.Torus();
        if (!compact)
        {
            os.Write("\n  Origin :");
        }

        Print(p.Location(), os, compact);
        if (!compact)
        {
            os.Write("\n  Axis   :");
        }

        Print(p.Axis().Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  XAxis  :");
        }

        Print(p.XAxis().Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  YAxis  :");
        }

        Print(p.YAxis().Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  Radii  :");
        }

        Out(os, p.MajorRadius());
        os.Write(" ");
        Out(os, p.MinorRadius());
        os.Write("\n");
        if (!compact)
        {
            os.Write("\n");
        }
    }

    private static void Print(ExtrusionSurface s, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Linearextrusion);
            os.Write(" ");
        }
        else
        {
            os.Write("SurfaceOfLinearExtrusion");
        }

        if (!compact)
        {
            os.Write("\n  Direction :");
        }

        Print(s.Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  Basis curve : ");
        }

        os.Write("\n");
        CurveSet.PrintCurve(s.BasisCurve(), os, compact);
    }

    private static void Print(RevolutionSurface s, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Revolution);
            os.Write(" ");
        }
        else
        {
            os.Write("SurfaceOfRevolution");
        }

        if (!compact)
        {
            os.Write("\n  Origin    :");
        }

        Print(s.Location(), os, compact);
        if (!compact)
        {
            os.Write("\n  Direction :");
        }

        Print(s.Direction(), os, compact);
        if (!compact)
        {
            os.Write("\n  Basis curve : ");
        }

        os.Write("\n");
        CurveSet.PrintCurve(s.BasisCurve(), os, compact);
    }

    private static void Print(BezierSurface s, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Bezier);
            os.Write(" ");
        }
        else
        {
            os.Write("BezierSurface");
        }

        bool urational = s.IsURational;
        bool vrational = s.IsVRational;
        if (compact)
        {
            Out(os, urational ? 1 : 0);
            os.Write(" ");
        }
        else
        {
            if (urational)
            {
                os.Write(" urational");
            }
        }

        if (compact)
        {
            Out(os, vrational ? 1 : 0);
            os.Write(" ");
        }
        else
        {
            if (vrational)
            {
                os.Write(" vrational");
            }
        }

        if (!compact)
        {
            bool uclosed = s.IsUClosed;
            bool vclosed = s.IsVClosed;
            if (uclosed)
            {
                os.Write(" uclosed");
            }

            if (vclosed)
            {
                os.Write(" vclosed");
            }
        }

        // poles and weights
        int i, j, udegree, vdegree;
        udegree = s.UDegree();
        vdegree = s.VDegree();
        if (!compact)
        {
            os.Write("\n  Degrees :");
        }

        Out(os, udegree);
        os.Write(" ");
        Out(os, vdegree);
        os.Write(" ");

        for (i = 1; i <= udegree + 1; i++)
        {
            for (j = 1; j <= vdegree + 1; j++)
            {
                if (!compact)
                {
                    os.Write("\n  ");
                    os.Write(Setw(2, i));
                    os.Write(", ");
                    os.Write(Setw(2, j));
                    os.Write(" : ");
                }

                Print(s.Pole(i, j), os, compact);
                if (urational || vrational)
                {
                    os.Write(" ");
                    Out(os, s.Weight(i, j));
                }

                if (compact)
                {
                    os.Write(" ");
                }
            }

            os.Write("\n");
        }

        os.Write("\n");
        if (!compact)
        {
            os.Write("\n");
        }
    }

    private static void Print(BSplineSurface s, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Bspline);
            os.Write(" ");
        }
        else
        {
            os.Write("BSplineSurface");
        }

        bool urational = s.IsURational;
        bool vrational = s.IsVRational;
        if (compact)
        {
            Out(os, urational ? 1 : 0);
            os.Write(" ");
        }
        else
        {
            if (urational)
            {
                os.Write(" urational");
            }
        }

        if (compact)
        {
            Out(os, vrational ? 1 : 0);
            os.Write(" ");
        }
        else
        {
            if (vrational)
            {
                os.Write(" vrational");
            }
        }

        bool uperiodic = s.IsUPeriodic;
        bool vperiodic = s.IsVPeriodic;
        if (compact)
        {
            Out(os, uperiodic ? 1 : 0);
            os.Write(" ");
        }
        else
        {
            if (uperiodic)
            {
                os.Write(" uperiodic");
            }
        }

        if (compact)
        {
            Out(os, vperiodic ? 1 : 0);
            os.Write(" ");
        }
        else
        {
            if (vperiodic)
            {
                os.Write(" vperiodic");
            }
        }

        if (!compact)
        {
            bool uclosed = s.IsUClosed;
            bool vclosed = s.IsVClosed;
            if (uclosed)
            {
                os.Write(" uclosed");
            }

            if (vclosed)
            {
                os.Write(" vclosed");
            }
        }

        // poles and weights
        int i, j, udegree, vdegree, nbupoles, nbvpoles, nbuknots, nbvknots;
        udegree = s.UDegree();
        vdegree = s.VDegree();
        nbupoles = s.UPoleCount;
        nbvpoles = s.VPoleCount;
        nbuknots = s.UKnotCount;
        nbvknots = s.VKnotCount;
        if (!compact)
        {
            os.Write("\n  Degrees :");
        }

        Out(os, udegree);
        os.Write(" ");
        Out(os, vdegree);
        os.Write(" ");
        if (!compact)
        {
            os.Write("\n  NbPoles :");
        }

        Out(os, nbupoles);
        os.Write(" ");
        Out(os, nbvpoles);
        os.Write(" ");
        if (!compact)
        {
            os.Write("\n  NbKnots :");
        }

        Out(os, nbuknots);
        os.Write(" ");
        Out(os, nbvknots);
        os.Write(" ");

        if (!compact)
        {
            os.Write("\n Poles :\n");
        }

        for (i = 1; i <= nbupoles; i++)
        {
            for (j = 1; j <= nbvpoles; j++)
            {
                if (!compact)
                {
                    os.Write("\n  ");
                    os.Write(Setw(2, i));
                    os.Write(", ");
                    os.Write(Setw(2, j));
                    os.Write(" : ");
                }

                Print(s.Pole(i, j), os, compact);
                if (urational || vrational)
                {
                    os.Write(" ");
                    Out(os, s.Weight(i, j));
                }

                if (compact)
                {
                    os.Write(" ");
                }
            }

            os.Write("\n");
        }

        os.Write("\n");
        if (!compact)
        {
            os.Write("\n UKnots :\n");
        }

        for (i = 1; i <= nbuknots; i++)
        {
            if (!compact)
            {
                os.Write("\n  ");
                os.Write(Setw(2, i));
                os.Write(" : ");
            }

            Out(os, s.UKnot(i));
            os.Write(" ");
            Out(os, s.UMultiplicity(i));
            os.Write("\n");
        }

        os.Write("\n");
        if (!compact)
        {
            os.Write("\n VKnots :\n");
        }

        for (i = 1; i <= nbvknots; i++)
        {
            if (!compact)
            {
                os.Write("\n  ");
                os.Write(Setw(2, i));
                os.Write(" : ");
            }

            Out(os, s.VKnot(i));
            os.Write(" ");
            Out(os, s.VMultiplicity(i));
            os.Write("\n");
        }

        os.Write("\n");
        if (!compact)
        {
            os.Write("\n");
        }
    }

    private static void Print(TrimmedSurface s, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Rectangular);
            os.Write(" ");
        }
        else
        {
            os.Write("RectangularTrimmedSurface");
        }

        double u1, u2, v1, v2;
        s.Bounds(out u1, out u2, out v1, out v2);
        if (!compact)
        {
            os.Write("\nParameters : ");
        }

        Out(os, u1);
        os.Write(" ");
        Out(os, u2);
        os.Write(" ");
        Out(os, v1);
        os.Write(" ");
        Out(os, v2);
        os.Write("\n");
        if (!compact)
        {
            os.Write("BasisSurface :\n");
        }

        PrintSurface(s.BasisSurface(), os, compact);
    }

    private static void Print(OffsetSurface s, TextWriter os, bool compact)
    {
        if (compact)
        {
            Out(os, Offset);
            os.Write(" ");
        }
        else
        {
            os.Write("OffsetSurface");
        }

        if (!compact)
        {
            os.Write("\nOffset : ");
        }

        Out(os, s.Offset());
        os.Write("\n");
        if (!compact)
        {
            os.Write("BasisSurface :\n");
        }

        PrintSurface(s.BasisSurface(), os, compact);
    }

    /// <summary>
    /// Writes the surface; if <paramref name="compact"/>, in the format <see cref="ReadSurface"/> reads, otherwise as
    /// a readable dump, at the writer's precision. An unknown surface type goes to
    /// <see cref="GeomTools.GetUndefinedTypeHandler"/>.
    /// </summary>
    public static void PrintSurface(Surface s, TextWriter os, bool compact = false)
    {
        Type theType = s.DynamicType();

        if (theType == typeof(PlaneSurface))
        {
            Print((PlaneSurface)s, os, compact);
        }
        else if (theType == typeof(CylindricalSurface))
        {
            Print((CylindricalSurface)s, os, compact);
        }
        else if (theType == typeof(ConicalSurface))
        {
            Print((ConicalSurface)s, os, compact);
        }
        else if (theType == typeof(SphericalSurface))
        {
            Print((SphericalSurface)s, os, compact);
        }
        else if (theType == typeof(ToroidalSurface))
        {
            Print((ToroidalSurface)s, os, compact);
        }
        else if (theType == typeof(ExtrusionSurface))
        {
            Print((ExtrusionSurface)s, os, compact);
        }
        else if (theType == typeof(RevolutionSurface))
        {
            Print((RevolutionSurface)s, os, compact);
        }
        else if (theType == typeof(BezierSurface))
        {
            Print((BezierSurface)s, os, compact);
        }
        else if (theType == typeof(BSplineSurface))
        {
            Print((BSplineSurface)s, os, compact);
        }
        else if (theType == typeof(TrimmedSurface))
        {
            Print((TrimmedSurface)s, os, compact);
        }
        else if (theType == typeof(OffsetSurface))
        {
            Print((OffsetSurface)s, os, compact);
        }
        else
        {
            global::Kerf.Engine.Exchange.Brep.GeomTools.GetUndefinedTypeHandler().PrintSurface(s, os, compact);
        }
    }

    /// <summary>Writes a readable dump of the set.</summary>
    public void Dump(TextWriter os)
    {
        int i, nbsurf = _map.Extent();
        os.Write("\n -------\n");
        os.Write("Dump of ");
        Out(os, nbsurf);
        os.Write(" surfaces ");
        os.Write("\n -------\n\n");

        for (i = 1; i <= nbsurf; i++)
        {
            os.Write(Setw(4, i));
            os.Write(" : ");
            // OCCT dereferences a null surface here
            PrintSurface(Surface(i)!, os, false);
        }
    }

    /// <summary>
    /// Writes the set in the format <see cref="Read"/> reads: "Surfaces", the count, then each surface, at 17
    /// significant digits (the writer's precision is restored afterwards).
    /// </summary>
    public void Write(TextWriter os, ProgressRange progress = default)
    {
        int prec = StreamUtilities.Precision(os, 17);

        int i, nbsurf = _map.Extent();
        os.Write("Surfaces ");
        Out(os, nbsurf);
        os.Write("\n");
        using var ps = new ProgressScope(progress, "Surfaces", nbsurf);
        for (i = 1; i <= nbsurf && ps.More(); i++, ps.Next())
        {
            // OCCT dereferences a null surface here
            PrintSurface(Surface(i)!, os, true);
        }

        StreamUtilities.Precision(os, prec);
    }

    // operator>> of the points, directions, axes and surfaces

    private static void ReadPnt(TextReader IS, out Point3D p)
    {
        double x, y, z;
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out x);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out y);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out z);
        p = new Point3D(x, y, z);
    }

    private static void ReadDir(TextReader IS, ref Direction3D d)
    {
        double x, y, z;
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out x);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out y);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out z);
        d.SetCoord(x, y, z);
    }

    private static void ReadAx3(TextReader IS, out CoordinateSystem a3)
    {
        Point3D p;
        Direction3D a = new(Direction3D.D.X), ax = new(Direction3D.D.X), ay = new(Direction3D.D.X);
        ReadPnt(IS, out p);
        ReadDir(IS, ref a);
        ReadDir(IS, ref ax);
        ReadDir(IS, ref ay);
        CoordinateSystem ax3 = new(p, a, ax);
        if (ay.DotCross(a, ax) < 0)
        {
            ax3.YReverse();
        }

        a3 = ax3;
    }

    private static PlaneSurface ReadPlane(TextReader IS)
    {
        CoordinateSystem a;
        ReadAx3(IS, out a);
        return new PlaneSurface(a);
    }

    private static CylindricalSurface ReadCylinder(TextReader IS)
    {
        CoordinateSystem a;
        double r;
        ReadAx3(IS, out a);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r);
        return new CylindricalSurface(a, r);
    }

    private static ConicalSurface ReadCone(TextReader IS)
    {
        CoordinateSystem a;
        double r, ang;
        ReadAx3(IS, out a);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out ang);
        return new ConicalSurface(a, ang, r);
    }

    private static SphericalSurface ReadSphere(TextReader IS)
    {
        CoordinateSystem a;
        double r;
        ReadAx3(IS, out a);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r);
        return new SphericalSurface(a, r);
    }

    private static ToroidalSurface ReadTorus(TextReader IS)
    {
        CoordinateSystem a;
        double r1, r2;
        ReadAx3(IS, out a);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r1);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out r2);
        return new ToroidalSurface(a, r1, r2);
    }

    private static ExtrusionSurface ReadLinearExtrusion(TextReader IS)
    {
        Direction3D d = new(Direction3D.D.X);
        ReadDir(IS, ref d);
        Curve3D? c = CurveSet.ReadCurve(IS);
        return new ExtrusionSurface(c!, d);
    }

    private static RevolutionSurface ReadRevolution(TextReader IS)
    {
        Point3D p;
        Direction3D d = new(Direction3D.D.X);
        ReadPnt(IS, out p);
        ReadDir(IS, ref d);
        Curve3D? c = CurveSet.ReadCurve(IS);
        return new RevolutionSurface(c!, new Axis3D(p, d));
    }

    private static BezierSurface ReadBezier(TextReader IS)
    {
        bool urational = false, vrational = false;
        StreamUtilities.Read(IS, ref urational);
        StreamUtilities.Read(IS, ref vrational);
        int udegree = 0, vdegree = 0;
        StreamUtilities.Read(IS, ref udegree);
        StreamUtilities.Read(IS, ref vdegree);
        // OCCT.NET: untrusted input: counts checked against the input before allocating (a pole takes at least 6
        // characters, a knot 4)
        StreamUtilities.CheckCount(IS, Math.Max(udegree + 1L, 0), 6, "Bezier poles");
        StreamUtilities.CheckCount(IS, Math.Max(vdegree + 1L, 0), 6, "Bezier poles");
        StreamUtilities.CheckCount(IS, Math.Max(udegree + 1L, 0) * Math.Max(vdegree + 1L, 0), 6, "Bezier poles");
        var poles = new NCollection_Array2<Point3D>(1, udegree + 1, 1, vdegree + 1);
        var weights = new NCollection_Array2<double>(1, udegree + 1, 1, vdegree + 1);

        int i, j;
        for (i = 1; i <= udegree + 1; i++)
        {
            for (j = 1; j <= vdegree + 1; j++)
            {
                ReadPnt(IS, out poles[i, j]);
                if (urational || vrational)
                {
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out weights[i, j]);
                }
            }
        }

        if (urational || vrational)
        {
            return new BezierSurface(poles, weights);
        }

        return new BezierSurface(poles);
    }

    private static BSplineSurface ReadBSpline(TextReader IS)
    {
        bool urational = false, vrational = false, uperiodic = false, vperiodic = false;
        StreamUtilities.Read(IS, ref urational);
        StreamUtilities.Read(IS, ref vrational);
        StreamUtilities.Read(IS, ref uperiodic);
        StreamUtilities.Read(IS, ref vperiodic);
        int udegree = 0, vdegree = 0, nbupoles = 0, nbvpoles = 0, nbuknots = 0, nbvknots = 0;
        StreamUtilities.Read(IS, ref udegree);
        StreamUtilities.Read(IS, ref vdegree);
        StreamUtilities.Read(IS, ref nbupoles);
        StreamUtilities.Read(IS, ref nbvpoles);
        StreamUtilities.Read(IS, ref nbuknots);
        StreamUtilities.Read(IS, ref nbvknots);

        // OCCT.NET: untrusted input: counts checked against the input before allocating (a pole takes at least 6
        // characters, a knot 4)
        StreamUtilities.CheckCount(IS, Math.Max(nbupoles, 0), 6, "B-spline poles");
        StreamUtilities.CheckCount(IS, Math.Max(nbvpoles, 0), 6, "B-spline poles");
        StreamUtilities.CheckCount(IS, Math.Max(nbupoles, 0) * (long)Math.Max(nbvpoles, 0), 6, "B-spline poles");
        StreamUtilities.CheckCount(IS, Math.Max(nbuknots, 0), 4, "B-spline knots");
        StreamUtilities.CheckCount(IS, Math.Max(nbvknots, 0), 4, "B-spline knots");
        var poles = new NCollection_Array2<Point3D>(1, nbupoles, 1, nbvpoles);
        var weights = new NCollection_Array2<double>(1, nbupoles, 1, nbvpoles);

        int i, j;
        for (i = 1; i <= nbupoles; i++)
        {
            for (j = 1; j <= nbvpoles; j++)
            {
                ReadPnt(IS, out poles[i, j]);
                if (urational || vrational)
                {
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out weights[i, j]);
                }
            }
        }

        var uknots = new NCollection_Array1<double>(1, nbuknots);
        var umults = new NCollection_Array1<int>(1, nbuknots);
        for (i = 1; i <= nbuknots; i++)
        {
            global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out uknots[i]);
            StreamUtilities.Read(IS, ref umults[i]);
        }

        var vknots = new NCollection_Array1<double>(1, nbvknots);
        var vmults = new NCollection_Array1<int>(1, nbvknots);
        for (i = 1; i <= nbvknots; i++)
        {
            global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out vknots[i]);
            StreamUtilities.Read(IS, ref vmults[i]);
        }

        if (urational || vrational)
        {
            return new BSplineSurface(poles,
                                           weights,
                                           uknots,
                                           vknots,
                                           umults,
                                           vmults,
                                           udegree,
                                           vdegree,
                                           uperiodic,
                                           vperiodic);
        }

        return new BSplineSurface(poles,
                                       uknots,
                                       vknots,
                                       umults,
                                       vmults,
                                       udegree,
                                       vdegree,
                                       uperiodic,
                                       vperiodic);
    }

    private static TrimmedSurface ReadRectangular(TextReader IS)
    {
        double u1, u2, v1, v2;
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out u1);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out u2);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out v1);
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out v2);
        Surface? bs = ReadSurface(IS);
        return new TrimmedSurface(bs!, u1, u2, v1, v2);
    }

    private static OffsetSurface ReadOffset(TextReader IS)
    {
        double o;
        global::Kerf.Engine.Exchange.Brep.GeomTools.GetReal(IS, out o);
        Surface? bs = ReadSurface(IS);
        return new OffsetSurface(bs!, o, true);
    }

    /// <summary>
    /// Reads a surface written by <see cref="PrintSurface"/> (compact). Returns null if the surface can't be built
    /// from what was read; an unknown type goes to <see cref="GeomTools.GetUndefinedTypeHandler"/>.
    /// </summary>
    public static Surface? ReadSurface(TextReader IS)
    {
        // OCCT's stype is uninitialised, and left so when the stream has already failed
        int stype = 0;

        Surface? s = null;
        // OCCT.NET: untrusted input: trimmed, offset and swept geometry recurse; the nesting is capped
        using var nesting = global::Kerf.Engine.Exchange.Brep.GeomTools.Nesting();
        try
        {
            StreamUtilities.Read(IS, ref stype);
            switch (stype)
            {
                case Plane:
                    s = ReadPlane(IS);
                    break;

                case global::Kerf.Engine.Exchange.Brep.SurfaceSet.Cylinder:
                    s = ReadCylinder(IS);
                    break;

                case global::Kerf.Engine.Exchange.Brep.SurfaceSet.Cone:
                    s = ReadCone(IS);
                    break;

                case global::Kerf.Engine.Exchange.Brep.SurfaceSet.Sphere:
                    s = ReadSphere(IS);
                    break;

                case global::Kerf.Engine.Exchange.Brep.SurfaceSet.Torus:
                    s = ReadTorus(IS);
                    break;

                case Linearextrusion:
                    s = ReadLinearExtrusion(IS);
                    break;

                case Revolution:
                    s = ReadRevolution(IS);
                    break;

                case Bezier:
                    s = ReadBezier(IS);
                    break;

                case Bspline:
                    s = ReadBSpline(IS);
                    break;

                case Rectangular:
                    s = ReadRectangular(IS);
                    break;

                case Offset:
                    s = ReadOffset(IS);
                    break;

                default:
                {
                    Surface? ss = null;
                    global::Kerf.Engine.Exchange.Brep.GeomTools.GetUndefinedTypeHandler().ReadSurface(stype, IS, ref ss);
                    s = ss;
                }
                    break;
            }
        }
        catch (FailureException exception) when (exception is not StreamFormatErrorException)
        {
            // OCCT_DEBUG only: "EXCEPTION in GeomTools_SurfaceSet::ReadSurface(..)!!!"
        }
        catch (NullReferenceException)
        {
            // OCCT: a swept, trimmed or offset surface whose basis couldn't be read: OCCT dereferences the null handle,
            // which OCC_CATCH_SIGNALS turns into a caught Standard_AccessViolation where signals are converted
        }

        return s;
    }

    /// <summary>
    /// Reads a set written by <see cref="Write"/>, adding its surfaces (frozen) to this one. Writes "Not a surface
    /// table" to the console, as OCCT does, if the text doesn't start with "Surfaces".
    /// </summary>
    public void Read(TextReader IS, ProgressRange progress = default)
    {
        // OCCT reads into char[255]: a longer word is an error (OCCT.NET: untrusted input)
        char[] buffer = new char[255];
        bool isRead = StreamUtilities.ReadWord(IS, buffer);
        if (!isRead || StreamUtilities.CString(buffer) != "Surfaces")
        {
            Console.Out.Write("Not a surface table" + "\n");
            return;
        }

        // OCCT's nbsurf is uninitialised if the count can't be read
        int i, nbsurf = 0;
        StreamUtilities.Read(IS, ref nbsurf);
        // OCCT.NET: untrusted input: an object takes at least 2 characters
        StreamUtilities.CheckCount(IS, Math.Max(nbsurf, 0), 2, "surfaces");
        using var ps = new ProgressScope(progress, "Surfaces", nbsurf);
        for (i = 1; i <= nbsurf && ps.More(); i++, ps.Next())
        {
            Surface? s = ReadSurface(IS);
            s?.Freeze();
            // OCCT: every object that can't be read is the one null entry, so the later indices shift
            _map.Add((object?)s ?? CurveSet.NullKey);
        }
    }
}
