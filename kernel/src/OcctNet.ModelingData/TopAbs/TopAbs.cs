// Port of: src/ModelingData/TKG3d/TopAbs/TopAbs.hxx
// Port of: src/ModelingData/TKG3d/TopAbs/TopAbs.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Topology;

/// <summary>Operations on the topological enumerations: composing and reversing orientations, names of types.</summary>
public static class TopAbs
{
    private static readonly Orientation[,] ComposeTable =
    {
        { Forward, Reversed, Internal, External },
        { Reversed, Forward, Internal, External },
        { Internal, Internal, Internal, Internal },
        { External, External, External, External },
    };

    private static readonly Orientation[] ReverseTable =
        [Reversed, Forward, Internal, External];

    private static readonly Orientation[] ComplementTable =
        [Reversed, Forward, External, Internal];

    private static readonly string[] TopAbs_Table_PrintShapeEnum =
        ["COMPOUND", "COMPSOLID", "SOLID", "SHELL", "FACE", "WIRE", "EDGE", "VERTEX", "SHAPE"];

    private static readonly string[] TopAbs_Table_PrintOrientation = ["FORWARD", "REVERSED", "INTERNAL", "EXTERNAL"];

    // OCCT: the table is in the order ON, IN, OUT, but TopAbs_State is IN, OUT, ON, so IN prints "ON", OUT "IN"
    // and ON "OUT".
    private static readonly string[] TopAbs_Table_PrintState = ["ON", "IN", "OUT", "UNKNOWN"];

    /// <summary>
    /// The orientation of a shape S of orientation <paramref name="or2"/> inside a shape of orientation
    /// <paramref name="or1"/>: FORWARD is the identity, REVERSED reverses, INTERNAL and EXTERNAL absorb.
    /// </summary>
    public static Orientation Compose(Orientation or1, Orientation or2) =>
        ComposeTable[(int)or2, (int)or1];

    /// <summary>The reverse orientation: FORWARD and REVERSED swap, INTERNAL and EXTERNAL stay.</summary>
    public static Orientation Reverse(Orientation or) => ReverseTable[(int)or];

    /// <summary>The complement orientation: FORWARD and REVERSED swap, as do INTERNAL and EXTERNAL.</summary>
    public static Orientation Complement(Orientation or) => ComplementTable[(int)or];

    /// <summary>Writes the shape type's name.</summary>
    public static TextWriter Print(ShapeType shapeType, TextWriter stream)
    {
        stream.Write(ShapeTypeToString(shapeType));
        return stream;
    }

    /// <summary>Writes the orientation's name.</summary>
    public static TextWriter Print(Orientation orientation, TextWriter stream)
    {
        stream.Write(ShapeOrientationToString(orientation));
        return stream;
    }

    /// <summary>Writes the state's name (with OCCT's mismatched table, see the remarks of occt-issues.md).</summary>
    public static TextWriter Print(Containment st, TextWriter s)
    {
        s.Write(TopAbs_Table_PrintState[(int)st]);
        return s;
    }

    /// <summary>The upper-case name of the shape type ("COMPOUND", ..., "SHAPE").</summary>
    public static string ShapeTypeToString(ShapeType type) => TopAbs_Table_PrintShapeEnum[(int)type];

    /// <summary>The shape type of a name (case-insensitive), or SHAPE if the name is unknown.</summary>
    public static ShapeType ShapeTypeFromString(string typeString)
    {
        ShapeType type = ShapeType.Shape;
        ShapeTypeFromString(typeString, ref type);
        return type;
    }

    /// <summary>Finds the shape type of a name (case-insensitive); false, leaving <paramref name="type"/> unchanged, if unknown.</summary>
    public static bool ShapeTypeFromString(string typeString, ref ShapeType type)
    {
        string name = typeString.ToUpperInvariant();
        for (int typeIter = 0; typeIter <= (int)ShapeType.Shape; ++typeIter)
        {
            string typeName = TopAbs_Table_PrintShapeEnum[typeIter];
            if (name == typeName)
            {
                type = (ShapeType)typeIter;
                return true;
            }
        }
        return false;
    }

    /// <summary>The upper-case name of the orientation ("FORWARD", ...).</summary>
    public static string ShapeOrientationToString(Orientation orientation) =>
        TopAbs_Table_PrintOrientation[(int)orientation];

    /// <summary>The orientation of a name (case-insensitive), or FORWARD if the name is unknown.</summary>
    public static Orientation ShapeOrientationFromString(string orientationString)
    {
        Orientation type = Forward;
        ShapeOrientationFromString(orientationString, ref type);
        return type;
    }

    /// <summary>Finds the orientation of a name (case-insensitive); false, leaving <paramref name="orientation"/> unchanged, if unknown.</summary>
    public static bool ShapeOrientationFromString(string orientationString, ref Orientation orientation)
    {
        string name = orientationString.ToUpperInvariant();
        for (int orientationIter = 0; orientationIter <= (int)External; ++orientationIter)
        {
            string orientationName = TopAbs_Table_PrintOrientation[orientationIter];
            if (name == orientationName)
            {
                orientation = (Orientation)orientationIter;
                return true;
            }
        }
        return false;
    }
}
