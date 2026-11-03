// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;
using static Occt.TopOpeBRepDS_Config;
using static Occt.TopOpeBRepDS_Kind;

namespace Occt;

/// <summary>The legacy data structure's package functions: the names of states, kinds and configurations, and kind/shape conversions.</summary>
public static class TopOpeBRepDS
{
    /// <summary>The two-letter name of a state (IN, OU, ON, UN).</summary>
    public static string SPrint(TopAbs_State S)
    {
        string s = "";
        switch (S)
        {
            case TopAbs_IN:
                s = s + "IN";
                break;
            case TopAbs_OUT:
                s = s + "OU";
                break;
            case TopAbs_ON:
                s = s + "ON";
                break;
            case TopAbs_UNKNOWN:
                s = s + "UN";
                break;
        }
        return s;
    }

    /// <summary>Writes the name of a state.</summary>
    public static TextWriter Print(TopAbs_State S, TextWriter OS)
    {
        OS.Write(SPrint(S));
        return OS;
    }

    /// <summary>The two-letter name of a kind (PO, CU, SU, VE, ED, WI, FA, SH, SO, CS, CO; empty for UNKNOWN).</summary>
    public static string SPrint(TopOpeBRepDS_Kind k)
    {
        string s = "";
        switch (k)
        {
            case TopOpeBRepDS_POINT:
                s = s + "PO";
                break;
            case TopOpeBRepDS_CURVE:
                s = s + "CU";
                break;
            case TopOpeBRepDS_SURFACE:
                s = s + "SU";
                break;
            case TopOpeBRepDS_VERTEX:
                s = s + "VE";
                break;
            case TopOpeBRepDS_EDGE:
                s = s + "ED";
                break;
            case TopOpeBRepDS_WIRE:
                s = s + "WI";
                break;
            case TopOpeBRepDS_FACE:
                s = s + "FA";
                break;
            case TopOpeBRepDS_SHELL:
                s = s + "SH";
                break;
            case TopOpeBRepDS_SOLID:
                s = s + "SO";
                break;
            case TopOpeBRepDS_COMPSOLID:
                s = s + "CS";
                break;
            case TopOpeBRepDS_COMPOUND:
                s = s + "CO";
                break;
            default:
                break;
        }
        return s;
    }

    /// <summary>A kind and index, as <c>S1(FA, 3)S2</c> (a space before a one-digit index).</summary>
    public static string SPrint(TopOpeBRepDS_Kind k, int i, string S1 = "", string S2 = "")
    {
        string si = "";
        if (i >= 0 && i <= 9)
        {
            si = " ";
        }
        string s = S1 + "(" + SPrint(k) + "," + si + i.ToString(CultureInfo.InvariantCulture) + ")" + S2;
        return s;
    }

    /// <summary>Writes the name of a kind.</summary>
    public static TextWriter Print(TopOpeBRepDS_Kind k, TextWriter OS)
    {
        OS.Write(SPrint(k));
        return OS;
    }

    /// <summary>Writes a kind and index.</summary>
    public static TextWriter Print(TopOpeBRepDS_Kind k, int i, TextWriter OS, string S1 = "", string S2 = "")
    {
        OS.Write(SPrint(k, i, S1, S2));
        OS.Flush();
        return OS;
    }

    /// <summary>The two-letter name of a shape type.</summary>
    /// <exception cref="ProgramErrorException">The type is TopAbs_SHAPE.</exception>
    public static string SPrint(TopAbs_ShapeEnum t) => SPrint(ShapeToKind(t));

    /// <summary>A shape type and index.</summary>
    public static string SPrint(TopAbs_ShapeEnum t, int i) => SPrint(ShapeToKind(t), i);

    /// <summary>Writes a shape type and index.</summary>
    public static TextWriter Print(TopAbs_ShapeEnum t, int i, TextWriter s)
    {
        s.Write(SPrint(ShapeToKind(t), i));
        return s;
    }

    /// <summary>The name of an orientation.</summary>
    public static string SPrint(TopAbs_Orientation o) => TopAbs.ShapeOrientationToString(o);

    /// <summary>The name of a configuration (UNSH, SAME, DIFF).</summary>
    public static string SPrint(TopOpeBRepDS_Config C)
    {
        string SS = "";
        switch (C)
        {
            case TopOpeBRepDS_UNSHGEOMETRY:
                SS = "UNSH";
                break;
            case TopOpeBRepDS_SAMEORIENTED:
                SS = "SAME";
                break;
            case TopOpeBRepDS_DIFFORIENTED:
                SS = "DIFF";
                break;
        }
        return SS;
    }

    /// <summary>Writes the name of a configuration.</summary>
    public static TextWriter Print(TopOpeBRepDS_Config C, TextWriter OS)
    {
        OS.Write(SPrint(C));
        return OS;
    }

    /// <summary>Whether the kind is a shape type.</summary>
    public static bool IsTopology(TopOpeBRepDS_Kind k)
    {
        switch (k)
        {
            case TopOpeBRepDS_COMPOUND:
            case TopOpeBRepDS_COMPSOLID:
            case TopOpeBRepDS_SOLID:
            case TopOpeBRepDS_SHELL:
            case TopOpeBRepDS_FACE:
            case TopOpeBRepDS_WIRE:
            case TopOpeBRepDS_EDGE:
            case TopOpeBRepDS_VERTEX:
                return true;
            default:
                return false;
        }
    }

    /// <summary>Whether the kind is a geometry (surface, curve, point).</summary>
    public static bool IsGeometry(TopOpeBRepDS_Kind k)
    {
        switch (k)
        {
            case TopOpeBRepDS_SURFACE:
            case TopOpeBRepDS_CURVE:
            case TopOpeBRepDS_POINT:
                return true;
            default:
                break;
        }
        return false;
    }

    /// <summary>The kind of a shape type.</summary>
    /// <exception cref="ProgramErrorException">The type is TopAbs_SHAPE.</exception>
    public static TopOpeBRepDS_Kind ShapeToKind(TopAbs_ShapeEnum S)
    {
        TopOpeBRepDS_Kind res = TopOpeBRepDS_SOLID; // bidon
        switch (S)
        {
            case TopAbs_VERTEX:
                res = TopOpeBRepDS_VERTEX;
                break;
            case TopAbs_EDGE:
                res = TopOpeBRepDS_EDGE;
                break;
            case TopAbs_WIRE:
                res = TopOpeBRepDS_WIRE;
                break;
            case TopAbs_FACE:
                res = TopOpeBRepDS_FACE;
                break;
            case TopAbs_SHELL:
                res = TopOpeBRepDS_SHELL;
                break;
            case TopAbs_SOLID:
                res = TopOpeBRepDS_SOLID;
                break;
            case TopAbs_COMPSOLID:
                res = TopOpeBRepDS_COMPSOLID;
                break;
            case TopAbs_COMPOUND:
                res = TopOpeBRepDS_COMPOUND;
                break;
            default:
                throw new ProgramErrorException("TopOpeBRepDS::ShapeToKind");
        }
        return res;
    }

    /// <summary>The shape type of a kind (TopAbs_SHAPE for a geometry).</summary>
    public static TopAbs_ShapeEnum KindToShape(TopOpeBRepDS_Kind K)
    {
        TopAbs_ShapeEnum res = TopAbs_SHAPE; // bidon
        if (!IsTopology(K))
        {
            return res;
        }

        switch (K)
        {
            case TopOpeBRepDS_VERTEX:
                res = TopAbs_VERTEX;
                break;
            case TopOpeBRepDS_EDGE:
                res = TopAbs_EDGE;
                break;
            case TopOpeBRepDS_WIRE:
                res = TopAbs_WIRE;
                break;
            case TopOpeBRepDS_FACE:
                res = TopAbs_FACE;
                break;
            case TopOpeBRepDS_SHELL:
                res = TopAbs_SHELL;
                break;
            case TopOpeBRepDS_SOLID:
                res = TopAbs_SOLID;
                break;
            case TopOpeBRepDS_COMPSOLID:
                res = TopAbs_COMPSOLID;
                break;
            case TopOpeBRepDS_COMPOUND:
                res = TopAbs_COMPOUND;
                break;
            default:
                throw new ProgramErrorException("TopOpeBRepDS::KindToShape");
        }
        return res;
    }
}
