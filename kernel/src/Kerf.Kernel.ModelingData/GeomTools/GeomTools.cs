// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools.hxx
// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;

namespace Kerf.Engine.Exchange.Brep;

/// <summary>
/// Writing and reading curves and surfaces in OCCT's text format (the geometry of a <c>.brep</c> file), and dumping
/// them readably. The sets (<see cref="SurfaceSet"/>, <see cref="CurveSet"/>,
/// <see cref="Curve2dSet"/>) write and read whole tables.
/// </summary>
/// <remarks>
/// OCCT's streams are a <see cref="TextWriter"/> and a <see cref="TextReader"/>, with C++'s stream state kept by
/// <see cref="StreamUtilities"/>: <see cref="Write(Curve3D, TextWriter)"/> writes reals at the writer's precision
/// (6 significant digits unless <see cref="StreamUtilities.Precision(TextWriter, int)"/> sets another, as C++'s
/// <c>OS.precision(n)</c>); the sets' <c>Write</c> methods use 17. OCCT's debugger helper <c>GeomTools_Dump(void*)</c>
/// (GeomTools_Debug.cxx) is not ported.
/// </remarks>
internal static class GeomTools
{
    // OCCT's global handler: a setting of the operation context (unit 10I), by default one that raises on any type
    internal static readonly ContextKey<UndefinedTypeHandler> HandlerSetting = new("GeomTools.UndefinedTypeHandler", new());

    /// <summary>Writes a readable dump of the surface.</summary>
    public static void Dump(Surface s, TextWriter os)
    {
        SurfaceSet.PrintSurface(s, os);
    }

    /// <summary>Writes the surface in the compact (file) format.</summary>
    public static void Write(Surface s, TextWriter os)
    {
        SurfaceSet.PrintSurface(s, os, true);
    }

    /// <summary>Reads a surface written by <see cref="Write(Surface, TextWriter)"/>; null if it can't be built.</summary>
    public static void Read(out Surface? s, TextReader IS)
    {
        s = SurfaceSet.ReadSurface(IS);
    }

    /// <summary>Writes a readable dump of the curve.</summary>
    public static void Dump(Curve3D c, TextWriter os)
    {
        CurveSet.PrintCurve(c, os);
    }

    /// <summary>Writes the curve in the compact (file) format.</summary>
    public static void Write(Curve3D c, TextWriter os)
    {
        CurveSet.PrintCurve(c, os, true);
    }

    /// <summary>Reads a curve written by <see cref="Write(Curve3D, TextWriter)"/>; null if it can't be built.</summary>
    public static void Read(out Curve3D? c, TextReader IS)
    {
        c = CurveSet.ReadCurve(IS);
    }

    /// <summary>Writes a readable dump of the 2D curve.</summary>
    public static void Dump(Curve2D c, TextWriter os)
    {
        Curve2dSet.PrintCurve2d(c, os);
    }

    /// <summary>Writes the 2D curve in the compact (file) format.</summary>
    public static void Write(Curve2D c, TextWriter os)
    {
        Curve2dSet.PrintCurve2d(c, os, true);
    }

    /// <summary>Reads a 2D curve written by <see cref="Write(Curve2D, TextWriter)"/>; null if it can't be built.</summary>
    public static void Read(out Curve2D? c, TextReader IS)
    {
        c = Curve2dSet.ReadCurve2d(IS);
    }

    /// <summary>
    /// Installs the handler of the curve and surface types GeomTools doesn't know, for the rest of the current flow of
    /// execution (<see cref="OperationContext"/>; OCCT's handler is a global). A null handler is ignored.
    /// </summary>
    public static void SetUndefinedTypeHandler(UndefinedTypeHandler? aHandler)
    {
        if (aHandler is not null)
        {
            OperationContext.Set(HandlerSetting, aHandler);
        }
    }

    /// <summary>The handler of the curve and surface types GeomTools doesn't know.</summary>
    public static UndefinedTypeHandler GetUndefinedTypeHandler() => HandlerSetting.Value;

    /// <summary>
    /// The deepest nesting of trimmed, offset and swept geometry the readers accept (a trimmed curve of a line is 2):
    /// reading recurses once per level.
    /// </summary>
    public const int MaxGeometryNesting = 64;

    [ThreadStatic]
    private static int s_nesting;

    /// <summary>
    /// Enters one level of nested geometry, for the readers; disposing the result leaves it.
    /// </summary>
    /// <exception cref="StreamFormatErrorException">The geometry is nested more than <see cref="MaxGeometryNesting"/>
    /// deep.</exception>
    internal static NestingLevel Nesting()
    {
        // Kerf: untrusted input
        if (s_nesting >= MaxGeometryNesting)
        {
            throw new StreamFormatErrorException("GeomTools: geometry nested more than " + MaxGeometryNesting + " deep");
        }

        s_nesting++;
        return new NestingLevel();
    }

    internal readonly ref struct NestingLevel
    {
        public void Dispose() => s_nesting--;
    }

    /// <summary>
    /// Reads a real: the next word, at most 31 characters, converted with <see cref="StreamUtilities.Strtod(string)"/>
    /// (its longest numeric prefix). 0 at the end of the text or when there is no number.
    /// </summary>
    public static void GetReal(TextReader IS, out double value)
    {
        value = 0.0;
        if (StreamUtilities.Eof(IS))
        {
            return;
        }

        // According IEEE-754 Specification and standard stream parameters
        // the most optimal buffer length not less then 25
        const int THE_BUFFER_SIZE = 32;
        string buffer = StreamUtilities.ReadWord(IS, THE_BUFFER_SIZE - 1) ?? "";
        value = StreamUtilities.Strtod(buffer);
    }
}
