// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools_UndefinedTypeHandler.hxx
// Port of: src/ModelingData/TKGeomBase/GeomTools/GeomTools_UndefinedTypeHandler.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Writes and reads the curves and surfaces that GeomTools doesn't know (types derived outside Geom and Geom2d).
/// The default writes a "****** UNKNOWN ... TYPE ******" line and reads nothing; derive from it and install it with
/// <see cref="GeomTools.SetUndefinedTypeHandler"/> to support other types.
/// </summary>
/// <remarks>As in OCCT, the compact (file) form of the message goes to the console, not to the stream.</remarks>
public class GeomTools_UndefinedTypeHandler : Standard_Transient
{
    public GeomTools_UndefinedTypeHandler()
    {
    }

    /// <summary>Writes an unknown 3D curve.</summary>
    public virtual void PrintCurve(Geom_Curve C, TextWriter OS, bool compact = false)
    {
        if (!compact)
        {
            OS.Write("****** UNKNOWN CURVE TYPE ******\n");
        }
        else
        {
            // OCCT: the compact (file) form goes to the console, not to the stream
            Console.Out.Write("****** UNKNOWN CURVE TYPE ******" + "\n");
        }
    }

    /// <summary>Reads a 3D curve of the type number <paramref name="ctype"/> into <paramref name="C"/>.</summary>
    public virtual TextReader ReadCurve(int ctype, TextReader IS, ref Geom_Curve? C)
    {
        return IS;
    }

    /// <summary>Writes an unknown 2D curve.</summary>
    public virtual void PrintCurve2d(Geom2d_Curve C, TextWriter OS, bool compact = false)
    {
        if (!compact)
        {
            OS.Write("****** UNKNOWN CURVE2d TYPE ******\n");
        }
        else
        {
            // OCCT: the compact (file) form goes to the console, not to the stream
            Console.Out.Write("****** UNKNOWN CURVE2d TYPE ******" + "\n");
        }
    }

    /// <summary>Reads a 2D curve of the type number <paramref name="ctype"/> into <paramref name="C"/>.</summary>
    public virtual TextReader ReadCurve2d(int ctype, TextReader IS, ref Geom2d_Curve? C)
    {
        return IS;
    }

    /// <summary>Writes an unknown surface.</summary>
    public virtual void PrintSurface(Geom_Surface S, TextWriter OS, bool compact = false)
    {
        if (!compact)
        {
            OS.Write("****** UNKNOWN SURFACE TYPE ******\n");
        }
        else
        {
            // OCCT: the compact (file) form goes to the console, not to the stream
            Console.Out.Write("****** UNKNOWN SURFACE TYPE ******" + "\n");
        }
    }

    /// <summary>Reads a surface of the type number <paramref name="ctype"/> into <paramref name="S"/>.</summary>
    public virtual TextReader ReadSurface(int ctype, TextReader IS, ref Geom_Surface? S)
    {
        return IS;
    }
}
