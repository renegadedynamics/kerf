// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccEnt/GccEnt.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccEnt/GccEnt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GccEnt_Position;

namespace Occt;

/// <summary>
/// Qualified lines and circles: the arguments of the constraint solvers, each with the relative position of the
/// solutions and the argument.
/// </summary>
public static class GccEnt
{
    private static readonly string[] GccEnt_Table_PrintPosition =
        ["UNQUALIFIED", "ENCLOSING", "ENCLOSED", "OUTSIDE", "NOQUALIFIER"];

    /// <summary>Writes the name of the position to the stream.</summary>
    public static TextWriter Print(GccEnt_Position thePosition, TextWriter theStream)
    {
        theStream.Write(PositionToString(thePosition));
        return theStream;
    }

    /// <summary>The name of the position: UNQUALIFIED, ENCLOSING, ENCLOSED, OUTSIDE or NOQUALIFIER.</summary>
    public static string PositionToString(GccEnt_Position thePosition)
    {
        return GccEnt_Table_PrintPosition[(int)thePosition];
    }

    /// <summary>The position named by the string (ignoring case), or GccEnt_unqualified if the name is unknown.</summary>
    public static GccEnt_Position PositionFromString(string thePositionString)
    {
        GccEnt_Position aPosition = GccEnt_unqualified;
        PositionFromString(thePositionString, ref aPosition);
        return aPosition;
    }

    /// <summary>
    /// The position named by the string (ignoring case); false, leaving the position unchanged, if the name is unknown.
    /// </summary>
    public static bool PositionFromString(string thePositionString, ref GccEnt_Position thePosition)
    {
        string aName = thePositionString.ToUpperInvariant();
        for (int aTypeIter = 0; aTypeIter <= (int)GccEnt_noqualifier; ++aTypeIter)
        {
            string aTypeName = GccEnt_Table_PrintPosition[aTypeIter];
            if (aName == aTypeName)
            {
                thePosition = (GccEnt_Position)aTypeIter;
                return true;
            }
        }
        return false;
    }

    /// <summary>A line whose solutions may lie on either side.</summary>
    public static GccEnt_QualifiedLin Unqualified(in gp_Lin2d Line)
    {
        return new GccEnt_QualifiedLin(Line, GccEnt_unqualified);
    }

    /// <summary>A circle whose solutions may take any position.</summary>
    public static GccEnt_QualifiedCirc Unqualified(in gp_Circ2d Circle)
    {
        return new GccEnt_QualifiedCirc(Circle, GccEnt_unqualified);
    }

    /// <summary>A circle that the solutions enclose.</summary>
    public static GccEnt_QualifiedCirc Enclosing(in gp_Circ2d Circle)
    {
        return new GccEnt_QualifiedCirc(Circle, GccEnt_enclosing);
    }

    /// <summary>A line whose solutions lie on its left (its inside).</summary>
    public static GccEnt_QualifiedLin Enclosed(in gp_Lin2d Line)
    {
        return new GccEnt_QualifiedLin(Line, GccEnt_enclosed);
    }

    /// <summary>A circle that encloses the solutions.</summary>
    public static GccEnt_QualifiedCirc Enclosed(in gp_Circ2d Circle)
    {
        return new GccEnt_QualifiedCirc(Circle, GccEnt_enclosed);
    }

    /// <summary>A circle outside the solutions, as they are outside it.</summary>
    public static GccEnt_QualifiedCirc Outside(in gp_Circ2d Circle)
    {
        return new GccEnt_QualifiedCirc(Circle, GccEnt_outside);
    }

    /// <summary>A line whose solutions lie on its right (its outside).</summary>
    public static GccEnt_QualifiedLin Outside(in gp_Lin2d Line)
    {
        return new GccEnt_QualifiedLin(Line, GccEnt_outside);
    }
}
