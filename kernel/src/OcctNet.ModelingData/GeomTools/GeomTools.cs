// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools.hxx
// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// Writing and reading curves and surfaces in OCCT's text format (the geometry of a <c>.brep</c> file), and dumping
/// them readably. The sets (<see cref="GeomTools_SurfaceSet"/>, <see cref="GeomTools_CurveSet"/>,
/// <see cref="GeomTools_Curve2dSet"/>) write and read whole tables.
/// </summary>
/// <remarks>
/// OCCT's streams are a <see cref="TextWriter"/> and a <see cref="TextReader"/>, with C++'s stream state kept by
/// <see cref="StreamUtilities"/>: <see cref="Write(Geom_Curve, TextWriter)"/> writes reals at the writer's precision
/// (6 significant digits unless <see cref="StreamUtilities.Precision(TextWriter, int)"/> sets another, as C++'s
/// <c>OS.precision(n)</c>); the sets' <c>Write</c> methods use 17. OCCT's debugger helper <c>GeomTools_Dump(void*)</c>
/// (GeomTools_Debug.cxx) is not ported.
/// </remarks>
public static class GeomTools
{
    private static volatile GeomTools_UndefinedTypeHandler theActiveHandler = new();

    /// <summary>Writes a readable dump of the surface.</summary>
    public static void Dump(Geom_Surface S, TextWriter OS)
    {
        GeomTools_SurfaceSet.PrintSurface(S, OS);
    }

    /// <summary>Writes the surface in the compact (file) format.</summary>
    public static void Write(Geom_Surface S, TextWriter OS)
    {
        GeomTools_SurfaceSet.PrintSurface(S, OS, true);
    }

    /// <summary>Reads a surface written by <see cref="Write(Geom_Surface, TextWriter)"/>; null if it can't be built.</summary>
    public static void Read(out Geom_Surface? S, TextReader IS)
    {
        S = GeomTools_SurfaceSet.ReadSurface(IS);
    }

    /// <summary>Writes a readable dump of the curve.</summary>
    public static void Dump(Geom_Curve C, TextWriter OS)
    {
        GeomTools_CurveSet.PrintCurve(C, OS);
    }

    /// <summary>Writes the curve in the compact (file) format.</summary>
    public static void Write(Geom_Curve C, TextWriter OS)
    {
        GeomTools_CurveSet.PrintCurve(C, OS, true);
    }

    /// <summary>Reads a curve written by <see cref="Write(Geom_Curve, TextWriter)"/>; null if it can't be built.</summary>
    public static void Read(out Geom_Curve? C, TextReader IS)
    {
        C = GeomTools_CurveSet.ReadCurve(IS);
    }

    /// <summary>Writes a readable dump of the 2D curve.</summary>
    public static void Dump(Geom2d_Curve C, TextWriter OS)
    {
        GeomTools_Curve2dSet.PrintCurve2d(C, OS);
    }

    /// <summary>Writes the 2D curve in the compact (file) format.</summary>
    public static void Write(Geom2d_Curve C, TextWriter OS)
    {
        GeomTools_Curve2dSet.PrintCurve2d(C, OS, true);
    }

    /// <summary>Reads a 2D curve written by <see cref="Write(Geom2d_Curve, TextWriter)"/>; null if it can't be built.</summary>
    public static void Read(out Geom2d_Curve? C, TextReader IS)
    {
        C = GeomTools_Curve2dSet.ReadCurve2d(IS);
    }

    /// <summary>
    /// Installs the handler of the curve and surface types GeomTools doesn't know, for every thread (OCCT's handler
    /// is a global too). A null handler is ignored.
    /// </summary>
    public static void SetUndefinedTypeHandler(GeomTools_UndefinedTypeHandler? aHandler)
    {
        if (aHandler is not null)
        {
            theActiveHandler = aHandler;
        }
    }

    /// <summary>The handler of the curve and surface types GeomTools doesn't know.</summary>
    public static GeomTools_UndefinedTypeHandler GetUndefinedTypeHandler()
    {
        return theActiveHandler;
    }

    /// <summary>
    /// The deepest nesting of trimmed, offset and swept geometry the readers accept (a trimmed curve of a line is 2):
    /// reading recurses once per level.
    /// </summary>
    public const int MaxGeometryNesting = 64;

    [ThreadStatic]
    private static int theNesting;

    /// <summary>
    /// Enters one level of nested geometry, for the readers; disposing the result leaves it.
    /// </summary>
    /// <exception cref="StreamFormatErrorException">The geometry is nested more than <see cref="MaxGeometryNesting"/>
    /// deep.</exception>
    internal static NestingLevel Nesting()
    {
        // OCCT.NET: untrusted input
        if (theNesting >= MaxGeometryNesting)
        {
            throw new StreamFormatErrorException("GeomTools: geometry nested more than " + MaxGeometryNesting + " deep");
        }

        theNesting++;
        return new NestingLevel();
    }

    internal readonly ref struct NestingLevel
    {
        public void Dispose() => theNesting--;
    }

    /// <summary>
    /// Reads a real: the next word, at most 31 characters, converted with <see cref="StreamUtilities.Strtod(string)"/>
    /// (its longest numeric prefix). 0 at the end of the text or when there is no number.
    /// </summary>
    public static void GetReal(TextReader IS, out double theValue)
    {
        theValue = 0.0;
        if (StreamUtilities.Eof(IS))
        {
            return;
        }

        // According IEEE-754 Specification and standard stream parameters
        // the most optimal buffer length not less then 25
        const int THE_BUFFER_SIZE = 32;
        string aBuffer = StreamUtilities.ReadWord(IS, THE_BUFFER_SIZE - 1) ?? "";
        theValue = StreamUtilities.Strtod(aBuffer);
    }
}
