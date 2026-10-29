// Port of: src/ModelingData/TKG3d/TopAbs/TopAbs.hxx
// Port of: src/ModelingData/TKG3d/TopAbs/TopAbs.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>Operations on the topological enumerations: composing and reversing orientations, names of types.</summary>
public static class TopAbs
{
    private static readonly TopAbs_Orientation[,] ComposeTable =
    {
        { TopAbs_FORWARD, TopAbs_REVERSED, TopAbs_INTERNAL, TopAbs_EXTERNAL },
        { TopAbs_REVERSED, TopAbs_FORWARD, TopAbs_INTERNAL, TopAbs_EXTERNAL },
        { TopAbs_INTERNAL, TopAbs_INTERNAL, TopAbs_INTERNAL, TopAbs_INTERNAL },
        { TopAbs_EXTERNAL, TopAbs_EXTERNAL, TopAbs_EXTERNAL, TopAbs_EXTERNAL },
    };

    private static readonly TopAbs_Orientation[] ReverseTable =
        [TopAbs_REVERSED, TopAbs_FORWARD, TopAbs_INTERNAL, TopAbs_EXTERNAL];

    private static readonly TopAbs_Orientation[] ComplementTable =
        [TopAbs_REVERSED, TopAbs_FORWARD, TopAbs_EXTERNAL, TopAbs_INTERNAL];

    private static readonly string[] TopAbs_Table_PrintShapeEnum =
        ["COMPOUND", "COMPSOLID", "SOLID", "SHELL", "FACE", "WIRE", "EDGE", "VERTEX", "SHAPE"];

    private static readonly string[] TopAbs_Table_PrintOrientation = ["FORWARD", "REVERSED", "INTERNAL", "EXTERNAL"];

    // OCCT: the table is in the order ON, IN, OUT, but TopAbs_State is IN, OUT, ON, so IN prints "ON", OUT "IN"
    // and ON "OUT".
    private static readonly string[] TopAbs_Table_PrintState = ["ON", "IN", "OUT", "UNKNOWN"];

    /// <summary>
    /// The orientation of a shape S of orientation <paramref name="Or2"/> inside a shape of orientation
    /// <paramref name="Or1"/>: FORWARD is the identity, REVERSED reverses, INTERNAL and EXTERNAL absorb.
    /// </summary>
    public static TopAbs_Orientation Compose(TopAbs_Orientation Or1, TopAbs_Orientation Or2) =>
        ComposeTable[(int)Or2, (int)Or1];

    /// <summary>The reverse orientation: FORWARD and REVERSED swap, INTERNAL and EXTERNAL stay.</summary>
    public static TopAbs_Orientation Reverse(TopAbs_Orientation Or) => ReverseTable[(int)Or];

    /// <summary>The complement orientation: FORWARD and REVERSED swap, as do INTERNAL and EXTERNAL.</summary>
    public static TopAbs_Orientation Complement(TopAbs_Orientation Or) => ComplementTable[(int)Or];

    /// <summary>Writes the shape type's name.</summary>
    public static TextWriter Print(TopAbs_ShapeEnum theShapeType, TextWriter theStream)
    {
        theStream.Write(ShapeTypeToString(theShapeType));
        return theStream;
    }

    /// <summary>Writes the orientation's name.</summary>
    public static TextWriter Print(TopAbs_Orientation theOrientation, TextWriter theStream)
    {
        theStream.Write(ShapeOrientationToString(theOrientation));
        return theStream;
    }

    /// <summary>Writes the state's name (with OCCT's mismatched table, see the remarks of occt-issues.md).</summary>
    public static TextWriter Print(TopAbs_State St, TextWriter S)
    {
        S.Write(TopAbs_Table_PrintState[(int)St]);
        return S;
    }

    /// <summary>The upper-case name of the shape type ("COMPOUND", ..., "SHAPE").</summary>
    public static string ShapeTypeToString(TopAbs_ShapeEnum theType) => TopAbs_Table_PrintShapeEnum[(int)theType];

    /// <summary>The shape type of a name (case-insensitive), or SHAPE if the name is unknown.</summary>
    public static TopAbs_ShapeEnum ShapeTypeFromString(string theTypeString)
    {
        TopAbs_ShapeEnum aType = TopAbs_SHAPE;
        ShapeTypeFromString(theTypeString, ref aType);
        return aType;
    }

    /// <summary>Finds the shape type of a name (case-insensitive); false, leaving <paramref name="theType"/> unchanged, if unknown.</summary>
    public static bool ShapeTypeFromString(string theTypeString, ref TopAbs_ShapeEnum theType)
    {
        string aName = theTypeString.ToUpperInvariant();
        for (int aTypeIter = 0; aTypeIter <= (int)TopAbs_SHAPE; ++aTypeIter)
        {
            string aTypeName = TopAbs_Table_PrintShapeEnum[aTypeIter];
            if (aName == aTypeName)
            {
                theType = (TopAbs_ShapeEnum)aTypeIter;
                return true;
            }
        }
        return false;
    }

    /// <summary>The upper-case name of the orientation ("FORWARD", ...).</summary>
    public static string ShapeOrientationToString(TopAbs_Orientation theOrientation) =>
        TopAbs_Table_PrintOrientation[(int)theOrientation];

    /// <summary>The orientation of a name (case-insensitive), or FORWARD if the name is unknown.</summary>
    public static TopAbs_Orientation ShapeOrientationFromString(string theOrientationString)
    {
        TopAbs_Orientation aType = TopAbs_FORWARD;
        ShapeOrientationFromString(theOrientationString, ref aType);
        return aType;
    }

    /// <summary>Finds the orientation of a name (case-insensitive); false, leaving <paramref name="theOrientation"/> unchanged, if unknown.</summary>
    public static bool ShapeOrientationFromString(string theOrientationString, ref TopAbs_Orientation theOrientation)
    {
        string aName = theOrientationString.ToUpperInvariant();
        for (int anOrientationIter = 0; anOrientationIter <= (int)TopAbs_EXTERNAL; ++anOrientationIter)
        {
            string anOrientationName = TopAbs_Table_PrintOrientation[anOrientationIter];
            if (aName == anOrientationName)
            {
                theOrientation = (TopAbs_Orientation)anOrientationIter;
                return true;
            }
        }
        return false;
    }
}
