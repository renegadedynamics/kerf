// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccEnt/GccEnt.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccEnt/GccEnt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// Qualified lines and circles: the arguments of the constraint solvers, each with the relative position of the
/// solutions and the argument.
/// </summary>
internal static class GccEnt
{
    private static readonly string[] GccEnt_Table_PrintPosition =
        ["UNQUALIFIED", "ENCLOSING", "ENCLOSED", "OUTSIDE", "NOQUALIFIER"];

    /// <summary>Writes the name of the position to the stream.</summary>
    public static TextWriter Print(Qualifier position, TextWriter stream)
    {
        stream.Write(PositionToString(position));
        return stream;
    }

    /// <summary>The name of the position: UNQUALIFIED, ENCLOSING, ENCLOSED, OUTSIDE or NOQUALIFIER.</summary>
    public static string PositionToString(Qualifier position)
    {
        return GccEnt_Table_PrintPosition[(int)position];
    }

    /// <summary>The position named by the string (ignoring case), or GccEnt_unqualified if the name is unknown.</summary>
    public static Qualifier PositionFromString(string positionString)
    {
        Qualifier position = Qualifier.Unqualified;
        PositionFromString(positionString, ref position);
        return position;
    }

    /// <summary>
    /// The position named by the string (ignoring case); false, leaving the position unchanged, if the name is unknown.
    /// </summary>
    public static bool PositionFromString(string positionString, ref Qualifier position)
    {
        string name = positionString.ToUpperInvariant();
        for (int typeIter = 0; typeIter <= (int)Qualifier.Noqualifier; ++typeIter)
        {
            string typeName = GccEnt_Table_PrintPosition[typeIter];
            if (name == typeName)
            {
                position = (Qualifier)typeIter;
                return true;
            }
        }
        return false;
    }

    /// <summary>A line whose solutions may lie on either side.</summary>
    public static QualifiedLin Unqualified(in Line2D line)
    {
        return new QualifiedLin(line, Qualifier.Unqualified);
    }

    /// <summary>A circle whose solutions may take any position.</summary>
    public static QualifiedCirc Unqualified(in Circle2D circle)
    {
        return new QualifiedCirc(circle, Qualifier.Unqualified);
    }

    /// <summary>A circle that the solutions enclose.</summary>
    public static QualifiedCirc Enclosing(in Circle2D circle)
    {
        return new QualifiedCirc(circle, Qualifier.Enclosing);
    }

    /// <summary>A line whose solutions lie on its left (its inside).</summary>
    public static QualifiedLin Enclosed(in Line2D line)
    {
        return new QualifiedLin(line, Qualifier.Enclosed);
    }

    /// <summary>A circle that encloses the solutions.</summary>
    public static QualifiedCirc Enclosed(in Circle2D circle)
    {
        return new QualifiedCirc(circle, Qualifier.Enclosed);
    }

    /// <summary>A circle outside the solutions, as they are outside it.</summary>
    public static QualifiedCirc Outside(in Circle2D circle)
    {
        return new QualifiedCirc(circle, Qualifier.Outside);
    }

    /// <summary>A line whose solutions lie on its right (its outside).</summary>
    public static QualifiedLin Outside(in Line2D line)
    {
        return new QualifiedLin(line, Qualifier.Outside);
    }
}
