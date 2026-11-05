// Port of: src/ModelingData/TKBRep/BinTools/BinTools_SurfaceSet.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_SurfaceSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Exchange.Brep.Binary;

/// <summary>
/// A set of Geom surfaces, indexed from 1, written and read in OCCT's binary format (the "Surfaces" section of a binary
/// BRep).
/// </summary>
/// <remarks>
/// As in OCCT, the set can hold the null surface once (<see cref="Add"/> adds it). The surfaces that <see cref="Read(Stream, ProgressRange)"/>
/// creates are frozen.
/// </remarks>
internal class BinarySurfaceSet
{
    private const byte Plane = 1;
    private const byte Cylinder = 2;
    private const byte Cone = 3;
    private const byte Sphere = 4;
    private const byte Torus = 5;
    private const byte Linearextrusion = 6;
    private const byte Revolution = 7;
    private const byte Bezier = 8;
    private const byte Bspline = 9;
    private const byte Rectangular = 10;
    private const byte Offset = 11;

    // OCCT's NCollection_IndexedMap of handles, which hashes by address and can hold one null handle
    private readonly IndexedSet<object> _map = new(ReferenceEqualityComparer.Instance);

    // the null handle as a key
    private static readonly object NullKey = new();

    /// <summary>An empty set of surfaces.</summary>
    public BinarySurfaceSet()
    {
    }

    /// <summary>Clears the content of the set.</summary>
    public void Clear()
    {
        _map.Clear();
    }

    /// <summary>Adds a surface to the set (once; null too) and returns its index.</summary>
    public int Add(Surface? s)
    {
        return _map.Add((object?)s ?? NullKey) + 1;
    }

    /// <summary>The surface of index I (null for the null entry).</summary>
    /// <exception cref="OutOfRangeException">There is no surface of index I.</exception>
    public Surface? Surface(int i)
    {
        return _map[i - 1] as Surface;
    }

    /// <summary>The index of the surface, 0 if it isn't in the set.</summary>
    public int Index(Surface? s)
    {
        return _map.IndexOf((object?)s ?? NullKey) + 1;
    }

    // the number of surfaces, for the readers' checks
    internal int SurfaceCount => _map.Count;

    private static void Write(OStream os, PlaneSurface s)
    {
        os = os << Plane;
        Plane3D p = s.Plane;
        os = os << p.Location; // Pnt
        os = os << p.Axis.Direction;
        os = os << p.XAxis.Direction;
        os = os << p.YAxis.Direction;
    }

    private static void Write(OStream os, CylindricalSurface s)
    {
        os = os << global::Kerf.Engine.Exchange.Brep.Binary.BinarySurfaceSet.Cylinder;
        Cylinder3D p = s.Cylinder;
        os = os << p.Location; // Pnt
        os = os << p.Axis.Direction;
        os = os << p.XAxis.Direction;
        os = os << p.YAxis.Direction;
        os = os << p.Radius;
    }

    private static void Write(OStream os, ConicalSurface s)
    {
        os = os << global::Kerf.Engine.Exchange.Brep.Binary.BinarySurfaceSet.Cone;
        Cone3D p = s.Cone;
        os = os << p.Location; // Pnt
        os = os << p.Axis.Direction;
        os = os << p.XAxis.Direction;
        os = os << p.YAxis.Direction;
        os = os << p.RefRadius;
        os = os << p.SemiAngle;
    }

    private static void Write(OStream os, SphericalSurface s)
    {
        os = os << global::Kerf.Engine.Exchange.Brep.Binary.BinarySurfaceSet.Sphere;
        Sphere3D p = s.Sphere;
        os = os << p.Location; // Pnt
        os = os << p.Position.Axis.Direction;
        os = os << p.XAxis.Direction;
        os = os << p.YAxis.Direction;
        os = os << p.Radius;
    }

    private static void Write(OStream os, ToroidalSurface s)
    {
        os = os << global::Kerf.Engine.Exchange.Brep.Binary.BinarySurfaceSet.Torus;
        Torus3D p = s.Torus;
        os = os << p.Location; // Pnt
        os = os << p.Axis.Direction;
        os = os << p.XAxis.Direction;
        os = os << p.YAxis.Direction;
        os = os << p.MajorRadius;
        os = os << p.MinorRadius;
    }

    private static void Write(OStream os, ExtrusionSurface s)
    {
        os = os << Linearextrusion;
        os = os << s.Direction;
        BinaryCurveSet.WriteCurve(s.BasisCurve, os);
    }

    private static void Write(OStream os, RevolutionSurface s)
    {
        os = os << Revolution;
        os = os << s.Location;
        os = os << s.Direction;
        BinaryCurveSet.WriteCurve(s.BasisCurve, os);
    }

    private static void Write(OStream os, BezierSurface s)
    {
        os = os << Bezier;
        bool urational = s.IsURational;
        bool vrational = s.IsVRational;
        os = os << urational; // rational
        os = os << vrational;

        // poles and weights
        int i, j, udegree, vdegree;
        udegree = s.UDegree;
        vdegree = s.VDegree;
        os = os << (char)udegree;
        os = os << (char)vdegree;
        for (i = 1; i <= udegree + 1; i++)
        {
            for (j = 1; j <= vdegree + 1; j++)
            {
                os = os << s.Pole(i, j); // Pnt
                if (urational || vrational)
                {
                    os = os << s.Weight(i, j); // Real
                }
            }
        }
    }

    private static void Write(OStream os, BSplineSurface s)
    {
        os = os << Bspline;
        bool urational = s.IsURational;
        bool vrational = s.IsVRational;
        bool uperiodic = s.IsUPeriodic;
        bool vperiodic = s.IsVPeriodic;
        os = os << urational;
        os = os << vrational;
        os = os << uperiodic;
        os = os << vperiodic;

        // poles and weights
        int i, j, udegree, vdegree, nbupoles, nbvpoles, nbuknots, nbvknots;
        udegree = s.UDegree;
        vdegree = s.VDegree;
        nbupoles = s.UPoleCount;
        nbvpoles = s.VPoleCount;
        nbuknots = s.UKnotCount;
        nbvknots = s.VKnotCount;
        os = os << (char)udegree;
        os = os << (char)vdegree;
        os = os << nbupoles;
        os = os << nbvpoles;
        os = os << nbuknots;
        os = os << nbvknots;
        for (i = 1; i <= nbupoles; i++)
        {
            for (j = 1; j <= nbvpoles; j++)
            {
                os = os << s.Pole(i, j); // Pnt
                if (urational || vrational)
                {
                    os = os << s.Weight(i, j); // Real
                }
            }
        }

        for (i = 1; i <= nbuknots; i++)
        {
            os = os << s.UKnot(i);
            os = os << s.UMultiplicity(i);
        }

        for (i = 1; i <= nbvknots; i++)
        {
            os = os << s.VKnot(i);
            os = os << s.VMultiplicity(i);
        }
    }

    private static void Write(OStream os, TrimmedSurface s)
    {
        os = os << Rectangular;
        s.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        os = os << u1 << u2 << v1 << v2;
        WriteSurface(s.BasisSurface, os);
    }

    private static void Write(OStream os, OffsetSurface s)
    {
        os = os << Offset;
        os = os << s.Offset;
        WriteSurface(s.BasisSurface, os);
    }

    /// <summary>Writes a surface (its type byte, then its data).</summary>
    /// <exception cref="FailureException">The surface is of a type the format doesn't know.</exception>
    public static void WriteSurface(Surface s, OStream os)
    {
        Type theType = s.GetType();
        try
        {
            if (theType == typeof(PlaneSurface))
            {
                Write(os, (PlaneSurface)s);
            }
            else if (theType == typeof(CylindricalSurface))
            {
                Write(os, (CylindricalSurface)s);
            }
            else if (theType == typeof(ConicalSurface))
            {
                Write(os, (ConicalSurface)s);
            }
            else if (theType == typeof(SphericalSurface))
            {
                Write(os, (SphericalSurface)s);
            }
            else if (theType == typeof(ToroidalSurface))
            {
                Write(os, (ToroidalSurface)s);
            }
            else if (theType == typeof(ExtrusionSurface))
            {
                Write(os, (ExtrusionSurface)s);
            }
            else if (theType == typeof(RevolutionSurface))
            {
                Write(os, (RevolutionSurface)s);
            }
            else if (theType == typeof(BezierSurface))
            {
                Write(os, (BezierSurface)s);
            }
            else if (theType == typeof(BSplineSurface))
            {
                Write(os, (BSplineSurface)s);
            }
            else if (theType == typeof(TrimmedSurface))
            {
                Write(os, (TrimmedSurface)s);
            }
            else if (theType == typeof(OffsetSurface))
            {
                Write(os, (OffsetSurface)s);
            }
            else
            {
                throw new FailureException("UNKNOWN SURFACE TYPE");
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_SurfaceSet::WriteSurface(..)", exception);
        }
    }

    /// <summary>Writes the set ("Surfaces", the count, then each surface).</summary>
    public void Write(Stream os, ProgressRange range = default)
    {
        int i, nbsurf = _map.Count;
        using var ps = new ProgressScope(range, "Writing surfaces", nbsurf);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "Surfaces ");
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, nbsurf);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.PutText(os, "\n");
        var stream = new OStream(os);
        for (i = 1; i <= nbsurf && ps.More(); i++, ps.Next())
        {
            // OCCT dereferences the null entry outside its try block, and crashes; the port throws
            Surface surface = _map[i - 1] as Surface
                                    ?? throw new FailureException("BinTools_SurfaceSet::Write: a null surface");
            WriteSurface(surface, stream);
        }
    }

    private static void Read(Stream IS, out CoordinateSystem a3)
    {
        Direction3D a = new(Direction3D.D.X), ax = new(Direction3D.D.X), ay = new(Direction3D.D.X);
        BinaryCurveSet.Read(IS, out Point3D p);
        BinaryCurveSet.Read(IS, ref a);
        BinaryCurveSet.Read(IS, ref ax);
        BinaryCurveSet.Read(IS, ref ay);
        CoordinateSystem ax3 = new(p, a, ax);
        if (ay.DotCross(a, ax) < 0)
        {
            ax3 = ax3.YReversed();
        }

        a3 = ax3;
    }

    private static PlaneSurface ReadPlane(Stream IS)
    {
        Read(IS, out CoordinateSystem a);
        return new PlaneSurface(a);
    }

    private static CylindricalSurface ReadCylinder(Stream IS)
    {
        Read(IS, out CoordinateSystem a);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r);
        return new CylindricalSurface(a, r);
    }

    private static ConicalSurface ReadCone(Stream IS)
    {
        Read(IS, out CoordinateSystem a);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double ang);
        return new ConicalSurface(a, ang, r);
    }

    private static SphericalSurface ReadSphere(Stream IS)
    {
        Read(IS, out CoordinateSystem a);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r);
        return new SphericalSurface(a, r);
    }

    private static ToroidalSurface ReadTorus(Stream IS)
    {
        Read(IS, out CoordinateSystem a);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r1);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double r2);
        return new ToroidalSurface(a, r1, r2);
    }

    private static ExtrusionSurface ReadLinearExtrusion(Stream IS, int depth)
    {
        Direction3D d = new(Direction3D.D.X);
        BinaryCurveSet.Read(IS, ref d);
        BinaryCurveSet.ReadCurve(IS, out Curve3D? c, depth + 1);
        return new ExtrusionSurface(c!, d);
    }

    private static RevolutionSurface ReadRevolution(Stream IS, int depth)
    {
        Direction3D d = new(Direction3D.D.X);
        BinaryCurveSet.Read(IS, out Point3D p);
        BinaryCurveSet.Read(IS, ref d);
        BinaryCurveSet.ReadCurve(IS, out Curve3D? c, depth + 1);
        return new RevolutionSurface(c!, new Axis3D(p, d));
    }

    private static BezierSurface ReadBezier(Stream IS)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool urational);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool vrational);

        // poles and weights
        int udegree, vdegree;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetExtChar(IS, out char val);
        udegree = val;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetExtChar(IS, out val);
        vdegree = val;
        // Kerf: untrusted input
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, (udegree + 1L) * (vdegree + 1L), urational || vrational ? 32 : 24, "poles");

        var poles = new NCollection_Array2<Point3D>(1, udegree + 1, 1, vdegree + 1);
        var weights = new NCollection_Array2<double>(1, udegree + 1, 1, vdegree + 1);

        int i, j;
        for (i = 1; i <= udegree + 1; i++)
        {
            for (j = 1; j <= vdegree + 1; j++)
            {
                BinaryCurveSet.Read(IS, out poles[i, j]); // Pnt
                if (urational || vrational)
                {
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out weights[i, j]);
                }
            }
        }

        if (urational || vrational)
        {
            return new BezierSurface(poles, weights);
        }

        return new BezierSurface(poles);
    }

    private static BSplineSurface ReadBSpline(Stream IS)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool urational);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool vrational);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool uperiodic);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetBool(IS, out bool vperiodic);

        // poles and weights
        int udegree, vdegree, nbupoles, nbvpoles, nbuknots, nbvknots;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetExtChar(IS, out char val);
        udegree = val;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetExtChar(IS, out val);
        vdegree = val;
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out nbupoles);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out nbvpoles);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out nbuknots);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out nbvknots);
        // Kerf: untrusted input
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbupoles, 1, "poles");
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbvpoles, 1, "poles");
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, (long)nbupoles * nbvpoles, urational || vrational ? 32 : 24, "poles");
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, (long)nbuknots + nbvknots, 12, "knots");

        var poles = new NCollection_Array2<Point3D>(1, nbupoles, 1, nbvpoles);
        var weights = new NCollection_Array2<double>(1, nbupoles, 1, nbvpoles);

        int i, j;
        for (i = 1; i <= nbupoles; i++)
        {
            for (j = 1; j <= nbvpoles; j++)
            {
                BinaryCurveSet.Read(IS, out poles[i, j]); // Pnt
                if (urational || vrational)
                {
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out weights[i, j]);
                }
            }
        }

        var uknots = new NCollection_Array1<double>(1, nbuknots);
        var umults = new NCollection_Array1<int>(1, nbuknots);
        for (i = 1; i <= nbuknots; i++)
        {
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out uknots[i]);
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out umults[i]);
        }

        var vknots = new NCollection_Array1<double>(1, nbvknots);
        var vmults = new NCollection_Array1<int>(1, nbvknots);
        for (i = 1; i <= nbvknots; i++)
        {
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out vknots[i]);
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetInteger(IS, out vmults[i]);
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

    private static TrimmedSurface ReadRectangular(Stream IS, int depth)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double u1);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double u2);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v1);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double v2);
        ReadSurface(IS, out Surface? bs, depth + 1);
        return new TrimmedSurface(bs!, u1, u2, v1, v2);
    }

    private static OffsetSurface ReadOffset(Stream IS, int depth)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.GetReal(IS, out double o);
        ReadSurface(IS, out Surface? bs, depth + 1);
        return new OffsetSurface(bs!, o);
    }

    /// <summary>Reads a surface written by <see cref="WriteSurface"/>. The surface is new and not frozen.</summary>
    /// <exception cref="FailureException">The surface can't be read.</exception>
    public static Stream ReadSurface(Stream IS, out Surface? s) => ReadSurface(IS, out s, 0);

    internal static Stream ReadSurface(Stream IS, out Surface? s, int depth)
    {
        s = null;
        try
        {
            if (depth > global::Kerf.Engine.Exchange.Brep.Binary.BinTools.MaxGeometryNesting)
            {
                // Kerf: untrusted input: OCCT recurses without limit
                throw new StreamFormatErrorException("BinTools_SurfaceSet: surfaces nested too deeply");
            }

            byte stype = (byte)global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS);
            switch (stype)
            {
                case Plane:
                    s = ReadPlane(IS);
                    break;

                case global::Kerf.Engine.Exchange.Brep.Binary.BinarySurfaceSet.Cylinder:
                    s = ReadCylinder(IS);
                    break;

                case global::Kerf.Engine.Exchange.Brep.Binary.BinarySurfaceSet.Cone:
                    s = ReadCone(IS);
                    break;

                case global::Kerf.Engine.Exchange.Brep.Binary.BinarySurfaceSet.Sphere:
                    s = ReadSphere(IS);
                    break;

                case global::Kerf.Engine.Exchange.Brep.Binary.BinarySurfaceSet.Torus:
                    s = ReadTorus(IS);
                    break;

                case Linearextrusion:
                    s = ReadLinearExtrusion(IS, depth);
                    break;

                case Revolution:
                    s = ReadRevolution(IS, depth);
                    break;

                case Bezier:
                    s = ReadBezier(IS);
                    break;

                case Bspline:
                    s = ReadBSpline(IS);
                    break;

                case Rectangular:
                    s = ReadRectangular(IS, depth);
                    break;

                case Offset:
                    s = ReadOffset(IS, depth);
                    break;

                default:
                {
                    s = null;
                    throw new FailureException("UNKNOWN SURFACE TYPE");
                }
            }
        }
        catch (Exception exception) when (global::Kerf.Engine.Exchange.Brep.Binary.BinTools.IsCaught(exception))
        {
            s = null;
            throw global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Rethrow("BinTools_SurfaceSet::ReadSurface(..)", exception);
        }

        return IS;
    }

    /// <summary>Reads a set written by <see cref="Write(Stream, ProgressRange)"/>, adding its surfaces (frozen) to this one.</summary>
    /// <exception cref="FailureException">The stream doesn't hold a surface table, or a surface can't be read.</exception>
    public void Read(Stream IS, ProgressRange range = default)
    {
        string? buffer = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadWord(IS);
        if (buffer is null || buffer != "Surfaces")
        {
            throw new FailureException("BinTools_SurfaceSet::Read:  Not a surface table" + "\n");
        }

        int i, nbsurf;
        nbsurf = global::Kerf.Engine.Exchange.Brep.Binary.BinTools.ReadFormattedInteger(IS);
        using var ps = new ProgressScope(range, "Reading surfaces", nbsurf);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Get(IS); // remove <lf>
        // Kerf: untrusted input: a surface takes at least 31 bytes (a Bezier surface of degree 0)
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.CheckCount(IS, nbsurf, 31, "surfaces");
        for (i = 1; i <= nbsurf && ps.More(); i++, ps.Next())
        {
            ReadSurface(IS, out Surface? s);
            s!.Freeze();
            _map.Add(s);
        }
    }
}
