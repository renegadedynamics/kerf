// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools_SurfaceSet.hxx
// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools_SurfaceSet.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// A set of Geom surfaces, indexed from 1, written and read in OCCT's text format (the "Surfaces" section of a
/// <c>.brep</c> file).
/// </summary>
/// <remarks>
/// As <see cref="GeomTools_CurveSet"/>: OCCT's exact output, C++'s extraction when reading, surfaces read are frozen,
/// and surfaces that can't be read are one null entry. As in OCCT, <see cref="Add"/> and <see cref="Index"/> accept
/// null (the null entry).
/// </remarks>
public class GeomTools_SurfaceSet
{
    private const int PLANE = 1;
    private const int CYLINDER = 2;
    private const int CONE = 3;
    private const int SPHERE = 4;
    private const int TORUS = 5;
    private const int LINEAREXTRUSION = 6;
    private const int REVOLUTION = 7;
    private const int BEZIER = 8;
    private const int BSPLINE = 9;
    private const int RECTANGULAR = 10;
    private const int OFFSET = 11;

    // OCCT's NCollection_IndexedMap of handles, which hashes by address and can hold one null handle
    private readonly NCollection_IndexedMap<object> myMap = new(ReferenceEqualityComparer.Instance);

    /// <summary>An empty set of surfaces.</summary>
    public GeomTools_SurfaceSet()
    {
    }

    /// <summary>Clears the content of the set.</summary>
    public void Clear()
    {
        myMap.Clear();
    }

    /// <summary>Adds a surface to the set (once) and returns its index. Null is added too, as in OCCT.</summary>
    public int Add(Geom_Surface? S)
    {
        // OCCT: adds the null handle too, as one entry (occt-issues.md)
        return myMap.Add((object?)S ?? GeomTools_CurveSet.theNullKey);
    }

    /// <summary>The surface of index I, or null if there is none.</summary>
    public Geom_Surface? Surface(int I)
    {
        if (I <= 0 || I > myMap.Extent())
        {
            return null;
        }

        return myMap[I] as Geom_Surface;
    }

    /// <summary>The index of the surface, 0 if it isn't in the set (null finds the null entry, as in OCCT).</summary>
    public int Index(Geom_Surface? S)
    {
        return myMap.FindIndex((object?)S ?? GeomTools_CurveSet.theNullKey);
    }

    private static void Out(TextWriter OS, double theValue) => Standard_Stream.Write(OS, theValue);

    private static void Out(TextWriter OS, int theValue) => Standard_Stream.Write(OS, theValue);

    private static string Setw(int theWidth, int theValue) => Standard_Stream.Setw(theWidth, theValue.ToString(CultureInfo.InvariantCulture));

    private static void Print(in gp_Pnt P, TextWriter OS, bool compact)
    {
        Out(OS, P.X());
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, P.Y());
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, P.Z());
        OS.Write(" ");
    }

    private static void Print(in gp_Dir D, TextWriter OS, bool compact)
    {
        Out(OS, D.X());
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, D.Y());
        if (!compact)
        {
            OS.Write(",");
        }

        OS.Write(" ");
        Out(OS, D.Z());
        OS.Write(" ");
    }

    private static void Print(Geom_Plane S, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, PLANE);
            OS.Write(" ");
        }
        else
        {
            OS.Write("Plane");
        }

        gp_Pln P = S.Pln();
        if (!compact)
        {
            OS.Write("\n  Origin :");
        }

        Print(P.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Axis   :");
        }

        Print(P.Axis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  XAxis  :");
        }

        Print(P.XAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  YAxis  :");
        }

        Print(P.YAxis().Direction(), OS, compact);
        OS.Write("\n");
        if (!compact)
        {
            OS.Write("\n");
        }
    }

    private static void Print(Geom_CylindricalSurface S, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, CYLINDER);
            OS.Write(" ");
        }
        else
        {
            OS.Write("CylindricalSurface");
        }

        gp_Cylinder P = S.Cylinder();
        if (!compact)
        {
            OS.Write("\n  Origin :");
        }

        Print(P.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Axis   :");
        }

        Print(P.Axis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  XAxis  :");
        }

        Print(P.XAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  YAxis  :");
        }

        Print(P.YAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Radius :");
        }

        Out(OS, P.Radius());
        OS.Write("\n");
        if (!compact)
        {
            OS.Write("\n");
        }
    }

    private static void Print(Geom_ConicalSurface S, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, CONE);
            OS.Write(" ");
        }
        else
        {
            OS.Write("ConicalSurface");
        }

        gp_Cone P = S.Cone();
        if (!compact)
        {
            OS.Write("\n  Origin :");
        }

        Print(P.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Axis   :");
        }

        Print(P.Axis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  XAxis  :");
        }

        Print(P.XAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  YAxis  :");
        }

        Print(P.YAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Radius :");
        }

        Out(OS, P.RefRadius());
        OS.Write("\n");
        if (!compact)
        {
            OS.Write("\n  Angle :");
        }

        Out(OS, P.SemiAngle());
        OS.Write("\n");
        if (!compact)
        {
            OS.Write("\n");
        }
    }

    private static void Print(Geom_SphericalSurface S, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, SPHERE);
            OS.Write(" ");
        }
        else
        {
            OS.Write("SphericalSurface");
        }

        gp_Sphere P = S.Sphere();
        if (!compact)
        {
            OS.Write("\n  Center :");
        }

        Print(P.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Axis   :");
        }

        Print(P.Position().Axis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  XAxis  :");
        }

        Print(P.XAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  YAxis  :");
        }

        Print(P.YAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Radius :");
        }

        Out(OS, P.Radius());
        OS.Write("\n");
        if (!compact)
        {
            OS.Write("\n");
        }
    }

    private static void Print(Geom_ToroidalSurface S, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, TORUS);
            OS.Write(" ");
        }
        else
        {
            OS.Write("ToroidalSurface");
        }

        gp_Torus P = S.Torus();
        if (!compact)
        {
            OS.Write("\n  Origin :");
        }

        Print(P.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Axis   :");
        }

        Print(P.Axis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  XAxis  :");
        }

        Print(P.XAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  YAxis  :");
        }

        Print(P.YAxis().Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Radii  :");
        }

        Out(OS, P.MajorRadius());
        OS.Write(" ");
        Out(OS, P.MinorRadius());
        OS.Write("\n");
        if (!compact)
        {
            OS.Write("\n");
        }
    }

    private static void Print(Geom_SurfaceOfLinearExtrusion S, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, LINEAREXTRUSION);
            OS.Write(" ");
        }
        else
        {
            OS.Write("SurfaceOfLinearExtrusion");
        }

        if (!compact)
        {
            OS.Write("\n  Direction :");
        }

        Print(S.Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Basis curve : ");
        }

        OS.Write("\n");
        GeomTools_CurveSet.PrintCurve(S.BasisCurve(), OS, compact);
    }

    private static void Print(Geom_SurfaceOfRevolution S, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, REVOLUTION);
            OS.Write(" ");
        }
        else
        {
            OS.Write("SurfaceOfRevolution");
        }

        if (!compact)
        {
            OS.Write("\n  Origin    :");
        }

        Print(S.Location(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Direction :");
        }

        Print(S.Direction(), OS, compact);
        if (!compact)
        {
            OS.Write("\n  Basis curve : ");
        }

        OS.Write("\n");
        GeomTools_CurveSet.PrintCurve(S.BasisCurve(), OS, compact);
    }

    private static void Print(Geom_BezierSurface S, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, BEZIER);
            OS.Write(" ");
        }
        else
        {
            OS.Write("BezierSurface");
        }

        bool urational = S.IsURational();
        bool vrational = S.IsVRational();
        if (compact)
        {
            Out(OS, urational ? 1 : 0);
            OS.Write(" ");
        }
        else
        {
            if (urational)
            {
                OS.Write(" urational");
            }
        }

        if (compact)
        {
            Out(OS, vrational ? 1 : 0);
            OS.Write(" ");
        }
        else
        {
            if (vrational)
            {
                OS.Write(" vrational");
            }
        }

        if (!compact)
        {
            bool uclosed = S.IsUClosed();
            bool vclosed = S.IsVClosed();
            if (uclosed)
            {
                OS.Write(" uclosed");
            }

            if (vclosed)
            {
                OS.Write(" vclosed");
            }
        }

        // poles and weights
        int i, j, udegree, vdegree;
        udegree = S.UDegree();
        vdegree = S.VDegree();
        if (!compact)
        {
            OS.Write("\n  Degrees :");
        }

        Out(OS, udegree);
        OS.Write(" ");
        Out(OS, vdegree);
        OS.Write(" ");

        for (i = 1; i <= udegree + 1; i++)
        {
            for (j = 1; j <= vdegree + 1; j++)
            {
                if (!compact)
                {
                    OS.Write("\n  ");
                    OS.Write(Setw(2, i));
                    OS.Write(", ");
                    OS.Write(Setw(2, j));
                    OS.Write(" : ");
                }

                Print(S.Pole(i, j), OS, compact);
                if (urational || vrational)
                {
                    OS.Write(" ");
                    Out(OS, S.Weight(i, j));
                }

                if (compact)
                {
                    OS.Write(" ");
                }
            }

            OS.Write("\n");
        }

        OS.Write("\n");
        if (!compact)
        {
            OS.Write("\n");
        }
    }

    private static void Print(Geom_BSplineSurface S, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, BSPLINE);
            OS.Write(" ");
        }
        else
        {
            OS.Write("BSplineSurface");
        }

        bool urational = S.IsURational();
        bool vrational = S.IsVRational();
        if (compact)
        {
            Out(OS, urational ? 1 : 0);
            OS.Write(" ");
        }
        else
        {
            if (urational)
            {
                OS.Write(" urational");
            }
        }

        if (compact)
        {
            Out(OS, vrational ? 1 : 0);
            OS.Write(" ");
        }
        else
        {
            if (vrational)
            {
                OS.Write(" vrational");
            }
        }

        bool uperiodic = S.IsUPeriodic();
        bool vperiodic = S.IsVPeriodic();
        if (compact)
        {
            Out(OS, uperiodic ? 1 : 0);
            OS.Write(" ");
        }
        else
        {
            if (uperiodic)
            {
                OS.Write(" uperiodic");
            }
        }

        if (compact)
        {
            Out(OS, vperiodic ? 1 : 0);
            OS.Write(" ");
        }
        else
        {
            if (vperiodic)
            {
                OS.Write(" vperiodic");
            }
        }

        if (!compact)
        {
            bool uclosed = S.IsUClosed();
            bool vclosed = S.IsVClosed();
            if (uclosed)
            {
                OS.Write(" uclosed");
            }

            if (vclosed)
            {
                OS.Write(" vclosed");
            }
        }

        // poles and weights
        int i, j, udegree, vdegree, nbupoles, nbvpoles, nbuknots, nbvknots;
        udegree = S.UDegree();
        vdegree = S.VDegree();
        nbupoles = S.NbUPoles();
        nbvpoles = S.NbVPoles();
        nbuknots = S.NbUKnots();
        nbvknots = S.NbVKnots();
        if (!compact)
        {
            OS.Write("\n  Degrees :");
        }

        Out(OS, udegree);
        OS.Write(" ");
        Out(OS, vdegree);
        OS.Write(" ");
        if (!compact)
        {
            OS.Write("\n  NbPoles :");
        }

        Out(OS, nbupoles);
        OS.Write(" ");
        Out(OS, nbvpoles);
        OS.Write(" ");
        if (!compact)
        {
            OS.Write("\n  NbKnots :");
        }

        Out(OS, nbuknots);
        OS.Write(" ");
        Out(OS, nbvknots);
        OS.Write(" ");

        if (!compact)
        {
            OS.Write("\n Poles :\n");
        }

        for (i = 1; i <= nbupoles; i++)
        {
            for (j = 1; j <= nbvpoles; j++)
            {
                if (!compact)
                {
                    OS.Write("\n  ");
                    OS.Write(Setw(2, i));
                    OS.Write(", ");
                    OS.Write(Setw(2, j));
                    OS.Write(" : ");
                }

                Print(S.Pole(i, j), OS, compact);
                if (urational || vrational)
                {
                    OS.Write(" ");
                    Out(OS, S.Weight(i, j));
                }

                if (compact)
                {
                    OS.Write(" ");
                }
            }

            OS.Write("\n");
        }

        OS.Write("\n");
        if (!compact)
        {
            OS.Write("\n UKnots :\n");
        }

        for (i = 1; i <= nbuknots; i++)
        {
            if (!compact)
            {
                OS.Write("\n  ");
                OS.Write(Setw(2, i));
                OS.Write(" : ");
            }

            Out(OS, S.UKnot(i));
            OS.Write(" ");
            Out(OS, S.UMultiplicity(i));
            OS.Write("\n");
        }

        OS.Write("\n");
        if (!compact)
        {
            OS.Write("\n VKnots :\n");
        }

        for (i = 1; i <= nbvknots; i++)
        {
            if (!compact)
            {
                OS.Write("\n  ");
                OS.Write(Setw(2, i));
                OS.Write(" : ");
            }

            Out(OS, S.VKnot(i));
            OS.Write(" ");
            Out(OS, S.VMultiplicity(i));
            OS.Write("\n");
        }

        OS.Write("\n");
        if (!compact)
        {
            OS.Write("\n");
        }
    }

    private static void Print(Geom_RectangularTrimmedSurface S, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, RECTANGULAR);
            OS.Write(" ");
        }
        else
        {
            OS.Write("RectangularTrimmedSurface");
        }

        double U1, U2, V1, V2;
        S.Bounds(out U1, out U2, out V1, out V2);
        if (!compact)
        {
            OS.Write("\nParameters : ");
        }

        Out(OS, U1);
        OS.Write(" ");
        Out(OS, U2);
        OS.Write(" ");
        Out(OS, V1);
        OS.Write(" ");
        Out(OS, V2);
        OS.Write("\n");
        if (!compact)
        {
            OS.Write("BasisSurface :\n");
        }

        PrintSurface(S.BasisSurface(), OS, compact);
    }

    private static void Print(Geom_OffsetSurface S, TextWriter OS, bool compact)
    {
        if (compact)
        {
            Out(OS, OFFSET);
            OS.Write(" ");
        }
        else
        {
            OS.Write("OffsetSurface");
        }

        if (!compact)
        {
            OS.Write("\nOffset : ");
        }

        Out(OS, S.Offset());
        OS.Write("\n");
        if (!compact)
        {
            OS.Write("BasisSurface :\n");
        }

        PrintSurface(S.BasisSurface(), OS, compact);
    }

    /// <summary>
    /// Writes the surface; if <paramref name="compact"/>, in the format <see cref="ReadSurface"/> reads, otherwise as
    /// a readable dump, at the writer's precision. An unknown surface type goes to
    /// <see cref="GeomTools.GetUndefinedTypeHandler"/>.
    /// </summary>
    public static void PrintSurface(Geom_Surface S, TextWriter OS, bool compact = false)
    {
        Type TheType = S.DynamicType();

        if (TheType == typeof(Geom_Plane))
        {
            Print((Geom_Plane)S, OS, compact);
        }
        else if (TheType == typeof(Geom_CylindricalSurface))
        {
            Print((Geom_CylindricalSurface)S, OS, compact);
        }
        else if (TheType == typeof(Geom_ConicalSurface))
        {
            Print((Geom_ConicalSurface)S, OS, compact);
        }
        else if (TheType == typeof(Geom_SphericalSurface))
        {
            Print((Geom_SphericalSurface)S, OS, compact);
        }
        else if (TheType == typeof(Geom_ToroidalSurface))
        {
            Print((Geom_ToroidalSurface)S, OS, compact);
        }
        else if (TheType == typeof(Geom_SurfaceOfLinearExtrusion))
        {
            Print((Geom_SurfaceOfLinearExtrusion)S, OS, compact);
        }
        else if (TheType == typeof(Geom_SurfaceOfRevolution))
        {
            Print((Geom_SurfaceOfRevolution)S, OS, compact);
        }
        else if (TheType == typeof(Geom_BezierSurface))
        {
            Print((Geom_BezierSurface)S, OS, compact);
        }
        else if (TheType == typeof(Geom_BSplineSurface))
        {
            Print((Geom_BSplineSurface)S, OS, compact);
        }
        else if (TheType == typeof(Geom_RectangularTrimmedSurface))
        {
            Print((Geom_RectangularTrimmedSurface)S, OS, compact);
        }
        else if (TheType == typeof(Geom_OffsetSurface))
        {
            Print((Geom_OffsetSurface)S, OS, compact);
        }
        else
        {
            GeomTools.GetUndefinedTypeHandler().PrintSurface(S, OS, compact);
        }
    }

    /// <summary>Writes a readable dump of the set.</summary>
    public void Dump(TextWriter OS)
    {
        int i, nbsurf = myMap.Extent();
        OS.Write("\n -------\n");
        OS.Write("Dump of ");
        Out(OS, nbsurf);
        OS.Write(" surfaces ");
        OS.Write("\n -------\n\n");

        for (i = 1; i <= nbsurf; i++)
        {
            OS.Write(Setw(4, i));
            OS.Write(" : ");
            // OCCT dereferences a null surface here
            PrintSurface(Surface(i)!, OS, false);
        }
    }

    /// <summary>
    /// Writes the set in the format <see cref="Read"/> reads: "Surfaces", the count, then each surface, at 17
    /// significant digits (the writer's precision is restored afterwards).
    /// </summary>
    public void Write(TextWriter OS, Message_ProgressRange theProgress = default)
    {
        int prec = Standard_Stream.Precision(OS, 17);

        int i, nbsurf = myMap.Extent();
        OS.Write("Surfaces ");
        Out(OS, nbsurf);
        OS.Write("\n");
        using var aPS = new Message_ProgressScope(theProgress, "Surfaces", nbsurf);
        for (i = 1; i <= nbsurf && aPS.More(); i++, aPS.Next())
        {
            // OCCT dereferences a null surface here
            PrintSurface(Surface(i)!, OS, true);
        }

        Standard_Stream.Precision(OS, prec);
    }

    // operator>> of the points, directions, axes and surfaces

    private static void ReadPnt(TextReader IS, out gp_Pnt P)
    {
        double X, Y, Z;
        GeomTools.GetReal(IS, out X);
        GeomTools.GetReal(IS, out Y);
        GeomTools.GetReal(IS, out Z);
        P = new gp_Pnt(X, Y, Z);
    }

    private static void ReadDir(TextReader IS, ref gp_Dir D)
    {
        double X, Y, Z;
        GeomTools.GetReal(IS, out X);
        GeomTools.GetReal(IS, out Y);
        GeomTools.GetReal(IS, out Z);
        D.SetCoord(X, Y, Z);
    }

    private static void ReadAx3(TextReader IS, out gp_Ax3 A3)
    {
        gp_Pnt P;
        gp_Dir A = new(gp_Dir.D.X), AX = new(gp_Dir.D.X), AY = new(gp_Dir.D.X);
        ReadPnt(IS, out P);
        ReadDir(IS, ref A);
        ReadDir(IS, ref AX);
        ReadDir(IS, ref AY);
        gp_Ax3 ax3 = new(P, A, AX);
        if (AY.DotCross(A, AX) < 0)
        {
            ax3.YReverse();
        }

        A3 = ax3;
    }

    private static Geom_Plane ReadPlane(TextReader IS)
    {
        gp_Ax3 A;
        ReadAx3(IS, out A);
        return new Geom_Plane(A);
    }

    private static Geom_CylindricalSurface ReadCylinder(TextReader IS)
    {
        gp_Ax3 A;
        double R;
        ReadAx3(IS, out A);
        GeomTools.GetReal(IS, out R);
        return new Geom_CylindricalSurface(A, R);
    }

    private static Geom_ConicalSurface ReadCone(TextReader IS)
    {
        gp_Ax3 A;
        double R, Ang;
        ReadAx3(IS, out A);
        GeomTools.GetReal(IS, out R);
        GeomTools.GetReal(IS, out Ang);
        return new Geom_ConicalSurface(A, Ang, R);
    }

    private static Geom_SphericalSurface ReadSphere(TextReader IS)
    {
        gp_Ax3 A;
        double R;
        ReadAx3(IS, out A);
        GeomTools.GetReal(IS, out R);
        return new Geom_SphericalSurface(A, R);
    }

    private static Geom_ToroidalSurface ReadTorus(TextReader IS)
    {
        gp_Ax3 A;
        double R1, R2;
        ReadAx3(IS, out A);
        GeomTools.GetReal(IS, out R1);
        GeomTools.GetReal(IS, out R2);
        return new Geom_ToroidalSurface(A, R1, R2);
    }

    private static Geom_SurfaceOfLinearExtrusion ReadLinearExtrusion(TextReader IS)
    {
        gp_Dir D = new(gp_Dir.D.X);
        ReadDir(IS, ref D);
        Geom_Curve? C = GeomTools_CurveSet.ReadCurve(IS);
        return new Geom_SurfaceOfLinearExtrusion(C!, D);
    }

    private static Geom_SurfaceOfRevolution ReadRevolution(TextReader IS)
    {
        gp_Pnt P;
        gp_Dir D = new(gp_Dir.D.X);
        ReadPnt(IS, out P);
        ReadDir(IS, ref D);
        Geom_Curve? C = GeomTools_CurveSet.ReadCurve(IS);
        return new Geom_SurfaceOfRevolution(C!, new gp_Ax1(P, D));
    }

    private static Geom_BezierSurface ReadBezier(TextReader IS)
    {
        bool urational = false, vrational = false;
        Standard_Stream.Read(IS, ref urational);
        Standard_Stream.Read(IS, ref vrational);
        int udegree = 0, vdegree = 0;
        Standard_Stream.Read(IS, ref udegree);
        Standard_Stream.Read(IS, ref vdegree);
        var poles = new NCollection_Array2<gp_Pnt>(1, udegree + 1, 1, vdegree + 1);
        var weights = new NCollection_Array2<double>(1, udegree + 1, 1, vdegree + 1);

        int i, j;
        for (i = 1; i <= udegree + 1; i++)
        {
            for (j = 1; j <= vdegree + 1; j++)
            {
                ReadPnt(IS, out poles[i, j]);
                if (urational || vrational)
                {
                    GeomTools.GetReal(IS, out weights[i, j]);
                }
            }
        }

        if (urational || vrational)
        {
            return new Geom_BezierSurface(poles, weights);
        }

        return new Geom_BezierSurface(poles);
    }

    private static Geom_BSplineSurface ReadBSpline(TextReader IS)
    {
        bool urational = false, vrational = false, uperiodic = false, vperiodic = false;
        Standard_Stream.Read(IS, ref urational);
        Standard_Stream.Read(IS, ref vrational);
        Standard_Stream.Read(IS, ref uperiodic);
        Standard_Stream.Read(IS, ref vperiodic);
        int udegree = 0, vdegree = 0, nbupoles = 0, nbvpoles = 0, nbuknots = 0, nbvknots = 0;
        Standard_Stream.Read(IS, ref udegree);
        Standard_Stream.Read(IS, ref vdegree);
        Standard_Stream.Read(IS, ref nbupoles);
        Standard_Stream.Read(IS, ref nbvpoles);
        Standard_Stream.Read(IS, ref nbuknots);
        Standard_Stream.Read(IS, ref nbvknots);

        var poles = new NCollection_Array2<gp_Pnt>(1, nbupoles, 1, nbvpoles);
        var weights = new NCollection_Array2<double>(1, nbupoles, 1, nbvpoles);

        int i, j;
        for (i = 1; i <= nbupoles; i++)
        {
            for (j = 1; j <= nbvpoles; j++)
            {
                ReadPnt(IS, out poles[i, j]);
                if (urational || vrational)
                {
                    GeomTools.GetReal(IS, out weights[i, j]);
                }
            }
        }

        var uknots = new NCollection_Array1<double>(1, nbuknots);
        var umults = new NCollection_Array1<int>(1, nbuknots);
        for (i = 1; i <= nbuknots; i++)
        {
            GeomTools.GetReal(IS, out uknots[i]);
            Standard_Stream.Read(IS, ref umults[i]);
        }

        var vknots = new NCollection_Array1<double>(1, nbvknots);
        var vmults = new NCollection_Array1<int>(1, nbvknots);
        for (i = 1; i <= nbvknots; i++)
        {
            GeomTools.GetReal(IS, out vknots[i]);
            Standard_Stream.Read(IS, ref vmults[i]);
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

    private static Geom_RectangularTrimmedSurface ReadRectangular(TextReader IS)
    {
        double U1, U2, V1, V2;
        GeomTools.GetReal(IS, out U1);
        GeomTools.GetReal(IS, out U2);
        GeomTools.GetReal(IS, out V1);
        GeomTools.GetReal(IS, out V2);
        Geom_Surface? BS = ReadSurface(IS);
        return new Geom_RectangularTrimmedSurface(BS!, U1, U2, V1, V2);
    }

    private static Geom_OffsetSurface ReadOffset(TextReader IS)
    {
        double O;
        GeomTools.GetReal(IS, out O);
        Geom_Surface? BS = ReadSurface(IS);
        return new Geom_OffsetSurface(BS!, O, true);
    }

    /// <summary>
    /// Reads a surface written by <see cref="PrintSurface"/> (compact). Returns null if the surface can't be built
    /// from what was read; an unknown type goes to <see cref="GeomTools.GetUndefinedTypeHandler"/>.
    /// </summary>
    public static Geom_Surface? ReadSurface(TextReader IS)
    {
        // OCCT's stype is uninitialised, and left so when the stream has already failed
        int stype = 0;

        Geom_Surface? S = null;
        try
        {
            Standard_Stream.Read(IS, ref stype);
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
                    S = ReadLinearExtrusion(IS);
                    break;

                case REVOLUTION:
                    S = ReadRevolution(IS);
                    break;

                case BEZIER:
                    S = ReadBezier(IS);
                    break;

                case BSPLINE:
                    S = ReadBSpline(IS);
                    break;

                case RECTANGULAR:
                    S = ReadRectangular(IS);
                    break;

                case OFFSET:
                    S = ReadOffset(IS);
                    break;

                default:
                {
                    Geom_Surface? SS = null;
                    GeomTools.GetUndefinedTypeHandler().ReadSurface(stype, IS, ref SS);
                    S = SS;
                }
                    break;
            }
        }
        catch (Standard_Failure)
        {
            // OCCT_DEBUG only: "EXCEPTION in GeomTools_SurfaceSet::ReadSurface(..)!!!"
        }
        catch (NullReferenceException)
        {
            // OCCT: a swept, trimmed or offset surface whose basis couldn't be read: OCCT dereferences the null handle,
            // which OCC_CATCH_SIGNALS turns into a caught Standard_AccessViolation where signals are converted
        }

        return S;
    }

    /// <summary>
    /// Reads a set written by <see cref="Write"/>, adding its surfaces (frozen) to this one. Writes "Not a surface
    /// table" to the console, as OCCT does, if the text doesn't start with "Surfaces".
    /// </summary>
    public void Read(TextReader IS, Message_ProgressRange theProgress = default)
    {
        string? buffer = Standard_Stream.ReadWord(IS);
        if (buffer != "Surfaces")
        {
            Console.Out.Write("Not a surface table" + "\n");
            return;
        }

        // OCCT's nbsurf is uninitialised if the count can't be read
        int i, nbsurf = 0;
        Standard_Stream.Read(IS, ref nbsurf);
        using var aPS = new Message_ProgressScope(theProgress, "Surfaces", nbsurf);
        for (i = 1; i <= nbsurf && aPS.More(); i++, aPS.Next())
        {
            Geom_Surface? S = ReadSurface(IS);
            S?.Freeze();
            // OCCT: every object that can't be read is the one null entry, so the later indices shift
            myMap.Add((object?)S ?? GeomTools_CurveSet.theNullKey);
        }
    }
}
