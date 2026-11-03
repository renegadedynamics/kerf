// Port of: src/ModelingData/TKBRep/BinTools/BinTools_SurfaceSet.hxx
// Port of: src/ModelingData/TKBRep/BinTools/BinTools_SurfaceSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2004-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// A set of Geom surfaces, indexed from 1, written and read in OCCT's binary format (the "Surfaces" section of a binary
/// BRep).
/// </summary>
/// <remarks>
/// As in OCCT, the set can hold the null surface once (<see cref="Add"/> adds it). The surfaces that <see cref="Read(Stream, ProgressRange)"/>
/// creates are frozen.
/// </remarks>
public class BinTools_SurfaceSet
{
    private const byte PLANE = 1;
    private const byte CYLINDER = 2;
    private const byte CONE = 3;
    private const byte SPHERE = 4;
    private const byte TORUS = 5;
    private const byte LINEAREXTRUSION = 6;
    private const byte REVOLUTION = 7;
    private const byte BEZIER = 8;
    private const byte BSPLINE = 9;
    private const byte RECTANGULAR = 10;
    private const byte OFFSET = 11;

    // OCCT's NCollection_IndexedMap of handles, which hashes by address and can hold one null handle
    private readonly NCollection_IndexedMap<object> myMap = new(ReferenceEqualityComparer.Instance);

    // the null handle as a key
    private static readonly object theNullKey = new();

    /// <summary>An empty set of surfaces.</summary>
    public BinTools_SurfaceSet()
    {
    }

    /// <summary>Clears the content of the set.</summary>
    public void Clear()
    {
        myMap.Clear();
    }

    /// <summary>Adds a surface to the set (once; null too) and returns its index.</summary>
    public int Add(Geom_Surface? S)
    {
        return myMap.Add((object?)S ?? theNullKey);
    }

    /// <summary>The surface of index I (null for the null entry).</summary>
    /// <exception cref="OutOfRangeException">There is no surface of index I.</exception>
    public Geom_Surface? Surface(int I)
    {
        return myMap[I] as Geom_Surface;
    }

    /// <summary>The index of the surface, 0 if it isn't in the set.</summary>
    public int Index(Geom_Surface? S)
    {
        return myMap.FindIndex((object?)S ?? theNullKey);
    }

    // the number of surfaces, for the readers' checks
    internal int NbSurfaces() => myMap.Extent();

    private static void Write(BinTools_OStream OS, Geom_Plane S)
    {
        OS = OS << PLANE;
        Plane3D P = S.Pln();
        OS = OS << P.Location(); // Pnt
        OS = OS << P.Axis().Direction();
        OS = OS << P.XAxis().Direction();
        OS = OS << P.YAxis().Direction();
    }

    private static void Write(BinTools_OStream OS, Geom_CylindricalSurface S)
    {
        OS = OS << CYLINDER;
        Cylinder P = S.Cylinder();
        OS = OS << P.Location(); // Pnt
        OS = OS << P.Axis().Direction();
        OS = OS << P.XAxis().Direction();
        OS = OS << P.YAxis().Direction();
        OS = OS << P.Radius();
    }

    private static void Write(BinTools_OStream OS, Geom_ConicalSurface S)
    {
        OS = OS << CONE;
        Cone P = S.Cone();
        OS = OS << P.Location(); // Pnt
        OS = OS << P.Axis().Direction();
        OS = OS << P.XAxis().Direction();
        OS = OS << P.YAxis().Direction();
        OS = OS << P.RefRadius();
        OS = OS << P.SemiAngle();
    }

    private static void Write(BinTools_OStream OS, Geom_SphericalSurface S)
    {
        OS = OS << SPHERE;
        Sphere P = S.Sphere();
        OS = OS << P.Location(); // Pnt
        OS = OS << P.Position().Axis().Direction();
        OS = OS << P.XAxis().Direction();
        OS = OS << P.YAxis().Direction();
        OS = OS << P.Radius();
    }

    private static void Write(BinTools_OStream OS, Geom_ToroidalSurface S)
    {
        OS = OS << TORUS;
        Torus P = S.Torus();
        OS = OS << P.Location(); // Pnt
        OS = OS << P.Axis().Direction();
        OS = OS << P.XAxis().Direction();
        OS = OS << P.YAxis().Direction();
        OS = OS << P.MajorRadius();
        OS = OS << P.MinorRadius();
    }

    private static void Write(BinTools_OStream OS, Geom_SurfaceOfLinearExtrusion S)
    {
        OS = OS << LINEAREXTRUSION;
        OS = OS << S.Direction();
        BinTools_CurveSet.WriteCurve(S.BasisCurve(), OS);
    }

    private static void Write(BinTools_OStream OS, Geom_SurfaceOfRevolution S)
    {
        OS = OS << REVOLUTION;
        OS = OS << S.Location();
        OS = OS << S.Direction();
        BinTools_CurveSet.WriteCurve(S.BasisCurve(), OS);
    }

    private static void Write(BinTools_OStream OS, Geom_BezierSurface S)
    {
        OS = OS << BEZIER;
        bool urational = S.IsURational();
        bool vrational = S.IsVRational();
        OS = OS << urational; // rational
        OS = OS << vrational;

        // poles and weights
        int i, j, udegree, vdegree;
        udegree = S.UDegree();
        vdegree = S.VDegree();
        OS = OS << (char)udegree;
        OS = OS << (char)vdegree;
        for (i = 1; i <= udegree + 1; i++)
        {
            for (j = 1; j <= vdegree + 1; j++)
            {
                OS = OS << S.Pole(i, j); // Pnt
                if (urational || vrational)
                {
                    OS = OS << S.Weight(i, j); // Real
                }
            }
        }
    }

    private static void Write(BinTools_OStream OS, Geom_BSplineSurface S)
    {
        OS = OS << BSPLINE;
        bool urational = S.IsURational();
        bool vrational = S.IsVRational();
        bool uperiodic = S.IsUPeriodic();
        bool vperiodic = S.IsVPeriodic();
        OS = OS << urational;
        OS = OS << vrational;
        OS = OS << uperiodic;
        OS = OS << vperiodic;

        // poles and weights
        int i, j, udegree, vdegree, nbupoles, nbvpoles, nbuknots, nbvknots;
        udegree = S.UDegree();
        vdegree = S.VDegree();
        nbupoles = S.NbUPoles();
        nbvpoles = S.NbVPoles();
        nbuknots = S.NbUKnots();
        nbvknots = S.NbVKnots();
        OS = OS << (char)udegree;
        OS = OS << (char)vdegree;
        OS = OS << nbupoles;
        OS = OS << nbvpoles;
        OS = OS << nbuknots;
        OS = OS << nbvknots;
        for (i = 1; i <= nbupoles; i++)
        {
            for (j = 1; j <= nbvpoles; j++)
            {
                OS = OS << S.Pole(i, j); // Pnt
                if (urational || vrational)
                {
                    OS = OS << S.Weight(i, j); // Real
                }
            }
        }

        for (i = 1; i <= nbuknots; i++)
        {
            OS = OS << S.UKnot(i);
            OS = OS << S.UMultiplicity(i);
        }

        for (i = 1; i <= nbvknots; i++)
        {
            OS = OS << S.VKnot(i);
            OS = OS << S.VMultiplicity(i);
        }
    }

    private static void Write(BinTools_OStream OS, Geom_RectangularTrimmedSurface S)
    {
        OS = OS << RECTANGULAR;
        S.Bounds(out double U1, out double U2, out double V1, out double V2);
        OS = OS << U1 << U2 << V1 << V2;
        WriteSurface(S.BasisSurface(), OS);
    }

    private static void Write(BinTools_OStream OS, Geom_OffsetSurface S)
    {
        OS = OS << OFFSET;
        OS = OS << S.Offset();
        WriteSurface(S.BasisSurface(), OS);
    }

    /// <summary>Writes a surface (its type byte, then its data).</summary>
    /// <exception cref="FailureException">The surface is of a type the format doesn't know.</exception>
    public static void WriteSurface(Geom_Surface S, BinTools_OStream OS)
    {
        Type TheType = S.GetType();
        try
        {
            if (TheType == typeof(Geom_Plane))
            {
                Write(OS, (Geom_Plane)S);
            }
            else if (TheType == typeof(Geom_CylindricalSurface))
            {
                Write(OS, (Geom_CylindricalSurface)S);
            }
            else if (TheType == typeof(Geom_ConicalSurface))
            {
                Write(OS, (Geom_ConicalSurface)S);
            }
            else if (TheType == typeof(Geom_SphericalSurface))
            {
                Write(OS, (Geom_SphericalSurface)S);
            }
            else if (TheType == typeof(Geom_ToroidalSurface))
            {
                Write(OS, (Geom_ToroidalSurface)S);
            }
            else if (TheType == typeof(Geom_SurfaceOfLinearExtrusion))
            {
                Write(OS, (Geom_SurfaceOfLinearExtrusion)S);
            }
            else if (TheType == typeof(Geom_SurfaceOfRevolution))
            {
                Write(OS, (Geom_SurfaceOfRevolution)S);
            }
            else if (TheType == typeof(Geom_BezierSurface))
            {
                Write(OS, (Geom_BezierSurface)S);
            }
            else if (TheType == typeof(Geom_BSplineSurface))
            {
                Write(OS, (Geom_BSplineSurface)S);
            }
            else if (TheType == typeof(Geom_RectangularTrimmedSurface))
            {
                Write(OS, (Geom_RectangularTrimmedSurface)S);
            }
            else if (TheType == typeof(Geom_OffsetSurface))
            {
                Write(OS, (Geom_OffsetSurface)S);
            }
            else
            {
                throw new FailureException("UNKNOWN SURFACE TYPE");
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            throw BinTools.Rethrow("BinTools_SurfaceSet::WriteSurface(..)", anException);
        }
    }

    /// <summary>Writes the set ("Surfaces", the count, then each surface).</summary>
    public void Write(Stream OS, ProgressRange theRange = default)
    {
        int i, nbsurf = myMap.Extent();
        using var aPS = new ProgressScope(theRange, "Writing surfaces", nbsurf);
        BinTools.PutText(OS, "Surfaces ");
        BinTools.PutText(OS, nbsurf);
        BinTools.PutText(OS, "\n");
        var aStream = new BinTools_OStream(OS);
        for (i = 1; i <= nbsurf && aPS.More(); i++, aPS.Next())
        {
            // OCCT dereferences the null entry outside its try block, and crashes; the port throws
            Geom_Surface aSurface = myMap[i] as Geom_Surface
                                    ?? throw new FailureException("BinTools_SurfaceSet::Write: a null surface");
            WriteSurface(aSurface, aStream);
        }
    }

    private static void Read(Stream IS, out CoordinateSystem A3)
    {
        Direction3D A = new(Direction3D.D.X), AX = new(Direction3D.D.X), AY = new(Direction3D.D.X);
        BinTools_CurveSet.Read(IS, out Point3D P);
        BinTools_CurveSet.Read(IS, ref A);
        BinTools_CurveSet.Read(IS, ref AX);
        BinTools_CurveSet.Read(IS, ref AY);
        CoordinateSystem ax3 = new(P, A, AX);
        if (AY.DotCross(A, AX) < 0)
        {
            ax3.YReverse();
        }

        A3 = ax3;
    }

    private static Geom_Plane ReadPlane(Stream IS)
    {
        Read(IS, out CoordinateSystem A);
        return new Geom_Plane(A);
    }

    private static Geom_CylindricalSurface ReadCylinder(Stream IS)
    {
        Read(IS, out CoordinateSystem A);
        BinTools.GetReal(IS, out double R);
        return new Geom_CylindricalSurface(A, R);
    }

    private static Geom_ConicalSurface ReadCone(Stream IS)
    {
        Read(IS, out CoordinateSystem A);
        BinTools.GetReal(IS, out double R);
        BinTools.GetReal(IS, out double Ang);
        return new Geom_ConicalSurface(A, Ang, R);
    }

    private static Geom_SphericalSurface ReadSphere(Stream IS)
    {
        Read(IS, out CoordinateSystem A);
        BinTools.GetReal(IS, out double R);
        return new Geom_SphericalSurface(A, R);
    }

    private static Geom_ToroidalSurface ReadTorus(Stream IS)
    {
        Read(IS, out CoordinateSystem A);
        BinTools.GetReal(IS, out double R1);
        BinTools.GetReal(IS, out double R2);
        return new Geom_ToroidalSurface(A, R1, R2);
    }

    private static Geom_SurfaceOfLinearExtrusion ReadLinearExtrusion(Stream IS, int theDepth)
    {
        Direction3D D = new(Direction3D.D.X);
        BinTools_CurveSet.Read(IS, ref D);
        BinTools_CurveSet.ReadCurve(IS, out Geom_Curve? C, theDepth + 1);
        return new Geom_SurfaceOfLinearExtrusion(C!, D);
    }

    private static Geom_SurfaceOfRevolution ReadRevolution(Stream IS, int theDepth)
    {
        Direction3D D = new(Direction3D.D.X);
        BinTools_CurveSet.Read(IS, out Point3D P);
        BinTools_CurveSet.Read(IS, ref D);
        BinTools_CurveSet.ReadCurve(IS, out Geom_Curve? C, theDepth + 1);
        return new Geom_SurfaceOfRevolution(C!, new Axis3D(P, D));
    }

    private static Geom_BezierSurface ReadBezier(Stream IS)
    {
        BinTools.GetBool(IS, out bool urational);
        BinTools.GetBool(IS, out bool vrational);

        // poles and weights
        int udegree, vdegree;
        BinTools.GetExtChar(IS, out char aVal);
        udegree = aVal;
        BinTools.GetExtChar(IS, out aVal);
        vdegree = aVal;
        // OCCT.NET: untrusted input
        BinTools.CheckCount(IS, (udegree + 1L) * (vdegree + 1L), urational || vrational ? 32 : 24, "poles");

        var poles = new NCollection_Array2<Point3D>(1, udegree + 1, 1, vdegree + 1);
        var weights = new NCollection_Array2<double>(1, udegree + 1, 1, vdegree + 1);

        int i, j;
        for (i = 1; i <= udegree + 1; i++)
        {
            for (j = 1; j <= vdegree + 1; j++)
            {
                BinTools_CurveSet.Read(IS, out poles[i, j]); // Pnt
                if (urational || vrational)
                {
                    BinTools.GetReal(IS, out weights[i, j]);
                }
            }
        }

        if (urational || vrational)
        {
            return new Geom_BezierSurface(poles, weights);
        }

        return new Geom_BezierSurface(poles);
    }

    private static Geom_BSplineSurface ReadBSpline(Stream IS)
    {
        BinTools.GetBool(IS, out bool urational);
        BinTools.GetBool(IS, out bool vrational);
        BinTools.GetBool(IS, out bool uperiodic);
        BinTools.GetBool(IS, out bool vperiodic);

        // poles and weights
        int udegree, vdegree, nbupoles, nbvpoles, nbuknots, nbvknots;
        BinTools.GetExtChar(IS, out char aVal);
        udegree = aVal;
        BinTools.GetExtChar(IS, out aVal);
        vdegree = aVal;
        BinTools.GetInteger(IS, out nbupoles);
        BinTools.GetInteger(IS, out nbvpoles);
        BinTools.GetInteger(IS, out nbuknots);
        BinTools.GetInteger(IS, out nbvknots);
        // OCCT.NET: untrusted input
        BinTools.CheckCount(IS, nbupoles, 1, "poles");
        BinTools.CheckCount(IS, nbvpoles, 1, "poles");
        BinTools.CheckCount(IS, (long)nbupoles * nbvpoles, urational || vrational ? 32 : 24, "poles");
        BinTools.CheckCount(IS, (long)nbuknots + nbvknots, 12, "knots");

        var poles = new NCollection_Array2<Point3D>(1, nbupoles, 1, nbvpoles);
        var weights = new NCollection_Array2<double>(1, nbupoles, 1, nbvpoles);

        int i, j;
        for (i = 1; i <= nbupoles; i++)
        {
            for (j = 1; j <= nbvpoles; j++)
            {
                BinTools_CurveSet.Read(IS, out poles[i, j]); // Pnt
                if (urational || vrational)
                {
                    BinTools.GetReal(IS, out weights[i, j]);
                }
            }
        }

        var uknots = new NCollection_Array1<double>(1, nbuknots);
        var umults = new NCollection_Array1<int>(1, nbuknots);
        for (i = 1; i <= nbuknots; i++)
        {
            BinTools.GetReal(IS, out uknots[i]);
            BinTools.GetInteger(IS, out umults[i]);
        }

        var vknots = new NCollection_Array1<double>(1, nbvknots);
        var vmults = new NCollection_Array1<int>(1, nbvknots);
        for (i = 1; i <= nbvknots; i++)
        {
            BinTools.GetReal(IS, out vknots[i]);
            BinTools.GetInteger(IS, out vmults[i]);
        }

        if (urational || vrational)
        {
            return new Geom_BSplineSurface(poles,
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

        return new Geom_BSplineSurface(poles,
                                       uknots,
                                       vknots,
                                       umults,
                                       vmults,
                                       udegree,
                                       vdegree,
                                       uperiodic,
                                       vperiodic);
    }

    private static Geom_RectangularTrimmedSurface ReadRectangular(Stream IS, int theDepth)
    {
        BinTools.GetReal(IS, out double U1);
        BinTools.GetReal(IS, out double U2);
        BinTools.GetReal(IS, out double V1);
        BinTools.GetReal(IS, out double V2);
        ReadSurface(IS, out Geom_Surface? BS, theDepth + 1);
        return new Geom_RectangularTrimmedSurface(BS!, U1, U2, V1, V2);
    }

    private static Geom_OffsetSurface ReadOffset(Stream IS, int theDepth)
    {
        BinTools.GetReal(IS, out double O);
        ReadSurface(IS, out Geom_Surface? BS, theDepth + 1);
        return new Geom_OffsetSurface(BS!, O);
    }

    /// <summary>Reads a surface written by <see cref="WriteSurface"/>. The surface is new and not frozen.</summary>
    /// <exception cref="FailureException">The surface can't be read.</exception>
    public static Stream ReadSurface(Stream IS, out Geom_Surface? S) => ReadSurface(IS, out S, 0);

    internal static Stream ReadSurface(Stream IS, out Geom_Surface? S, int theDepth)
    {
        S = null;
        try
        {
            if (theDepth > BinTools.MaxGeometryNesting)
            {
                // OCCT.NET: untrusted input: OCCT recurses without limit
                throw new StreamFormatErrorException("BinTools_SurfaceSet: surfaces nested too deeply");
            }

            byte stype = (byte)BinTools.Get(IS);
            switch (stype)
            {
                case PLANE:
                    S = ReadPlane(IS);
                    break;

                case CYLINDER:
                    S = ReadCylinder(IS);
                    break;

                case CONE:
                    S = ReadCone(IS);
                    break;

                case SPHERE:
                    S = ReadSphere(IS);
                    break;

                case TORUS:
                    S = ReadTorus(IS);
                    break;

                case LINEAREXTRUSION:
                    S = ReadLinearExtrusion(IS, theDepth);
                    break;

                case REVOLUTION:
                    S = ReadRevolution(IS, theDepth);
                    break;

                case BEZIER:
                    S = ReadBezier(IS);
                    break;

                case BSPLINE:
                    S = ReadBSpline(IS);
                    break;

                case RECTANGULAR:
                    S = ReadRectangular(IS, theDepth);
                    break;

                case OFFSET:
                    S = ReadOffset(IS, theDepth);
                    break;

                default:
                {
                    S = null;
                    throw new FailureException("UNKNOWN SURFACE TYPE");
                }
            }
        }
        catch (Exception anException) when (BinTools.IsCaught(anException))
        {
            S = null;
            throw BinTools.Rethrow("BinTools_SurfaceSet::ReadSurface(..)", anException);
        }

        return IS;
    }

    /// <summary>Reads a set written by <see cref="Write(Stream, ProgressRange)"/>, adding its surfaces (frozen) to this one.</summary>
    /// <exception cref="FailureException">The stream doesn't hold a surface table, or a surface can't be read.</exception>
    public void Read(Stream IS, ProgressRange theRange = default)
    {
        string? buffer = BinTools.ReadWord(IS);
        if (buffer is null || buffer != "Surfaces")
        {
            throw new FailureException("BinTools_SurfaceSet::Read:  Not a surface table" + "\n");
        }

        int i, nbsurf;
        nbsurf = BinTools.ReadFormattedInteger(IS);
        using var aPS = new ProgressScope(theRange, "Reading surfaces", nbsurf);
        BinTools.Get(IS); // remove <lf>
        // OCCT.NET: untrusted input: a surface takes at least 31 bytes (a Bezier surface of degree 0)
        BinTools.CheckCount(IS, nbsurf, 31, "surfaces");
        for (i = 1; i <= nbsurf && aPS.More(); i++, aPS.Next())
        {
            ReadSurface(IS, out Geom_Surface? S);
            S!.Freeze();
            myMap.Add(S);
        }
    }
}
