// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Tools.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Tools.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Data;

/// <summary>Auxiliary statics of the BOPDS package.</summary>
internal static class DataTools
{
    /// <summary>True if the type is that of a shape with a boundary representation: vertex, edge or face.</summary>
    public static bool HasBRep(ShapeType aTi) => aTi == ShapeType.Vertex || aTi == ShapeType.Edge || aTi == ShapeType.Face;

    /// <summary>True if a shape of the type can take part in an interference: one with BRep, or a solid.</summary>
    public static bool IsInterfering(ShapeType aTi) => HasBRep(aTi) || aTi == ShapeType.Solid;

    /// <summary>
    /// The interference type of two shape types: 0 VV, 1 VE, 2 EE, 3 VF, 4 EF, 5 FF, 6 VZ, 7 EZ, 8 FZ, 9 ZZ, or -1.
    /// </summary>
    public static int TypeToInteger(ShapeType aType1, ShapeType aType2)
    {
        int iRet, iT1, iT2, iX;
        //
        iRet = -1;
        iT1 = TypeToInteger(aType1);
        iT2 = TypeToInteger(aType2);
        //
        iX = iT2 * 10 + iT1;
        switch (iX)
        {
            case 77:
                iRet = 0; // VV
                break;
            case 76:
            case 67:
                iRet = 1; // VE
                break;
            case 66:
                iRet = 2; // EE
                break;
            case 74:
            case 47:
                iRet = 3; // VF
                break;
            case 64:
            case 46:
                iRet = 4; // EF
                break;
            case 44:
                iRet = 5; // FF
                break;
            case 72:
            case 27:
                iRet = 6; // VZ
                break;
            case 62:
            case 26:
                iRet = 7; // EZ
                break;
            case 42:
            case 24:
                iRet = 8; // FZ
                break;
            case 22:
                iRet = 9; // ZZ
                break;
            default:
                break;
        }
        return iRet;
    }

    /// <summary>The shape type as an integer: compound 0, compsolid 1, solid 2, ..., vertex 7, shape 8.</summary>
    public static int TypeToInteger(ShapeType aType1)
    {
        int iRet = 9;
        //
        switch (aType1)
        {
            case ShapeType.Compound:
                iRet = 0;
                break;
            case ShapeType.CompSolid:
                iRet = 1;
                break;
            case ShapeType.Solid:
                iRet = 2;
                break;
            case ShapeType.Shell:
                iRet = 3;
                break;
            case ShapeType.Face:
                iRet = 4;
                break;
            case ShapeType.Wire:
                iRet = 5;
                break;
            case ShapeType.Edge:
                iRet = 6;
                break;
            case ShapeType.Vertex:
                iRet = 7;
                break;
            case ShapeType.Shape:
                iRet = 8;
                break;
            default:
                break;
        }
        return iRet;
    }
}
