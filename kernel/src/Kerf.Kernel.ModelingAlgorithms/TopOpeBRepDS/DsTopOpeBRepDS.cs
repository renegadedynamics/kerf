// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using System.Globalization;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;
using static Kerf.Engine.Booleans.Legacy.Data.DsConfig;
using static Kerf.Engine.Booleans.Legacy.Data.DsKind;

namespace Kerf.Engine.Booleans.Legacy.Data;

/// <summary>The legacy data structure's package functions: the names of states, kinds and configurations, and kind/shape conversions.</summary>
internal static class DsTopOpeBRepDS
{
    /// <summary>The two-letter name of a state (IN, OU, ON, UN).</summary>
    public static string SPrint(Containment S)
    {
        string s = "";
        switch (S)
        {
            case In:
                s = s + "IN";
                break;
            case Out:
                s = s + "OU";
                break;
            case On:
                s = s + "ON";
                break;
            case global::Kerf.Topology.Containment.Unknown:
                s = s + "UN";
                break;
        }
        return s;
    }

    /// <summary>Writes the name of a state.</summary>
    public static TextWriter Print(Containment s, TextWriter os)
    {
        os.Write(SPrint(s));
        return os;
    }

    /// <summary>The two-letter name of a kind (PO, CU, SU, VE, ED, WI, FA, SH, SO, CS, CO; empty for UNKNOWN).</summary>
    public static string SPrint(DsKind k)
    {
        string s = "";
        switch (k)
        {
            case DsKind.Point:
                s = s + "PO";
                break;
            case DsKind.Curve:
                s = s + "CU";
                break;
            case DsKind.Surface:
                s = s + "SU";
                break;
            case DsKind.Vertex:
                s = s + "VE";
                break;
            case DsKind.Edge:
                s = s + "ED";
                break;
            case DsKind.Wire:
                s = s + "WI";
                break;
            case DsKind.Face:
                s = s + "FA";
                break;
            case DsKind.Shell:
                s = s + "SH";
                break;
            case DsKind.Solid:
                s = s + "SO";
                break;
            case DsKind.CompSolid:
                s = s + "CS";
                break;
            case DsKind.Compound:
                s = s + "CO";
                break;
            default:
                break;
        }
        return s;
    }

    /// <summary>A kind and index, as <c>S1(FA, 3)S2</c> (a space before a one-digit index).</summary>
    public static string SPrint(DsKind k, int i, string s1 = "", string s2 = "")
    {
        string si = "";
        if (i >= 0 && i <= 9)
        {
            si = " ";
        }
        string s = s1 + "(" + SPrint(k) + "," + si + i.ToString(CultureInfo.InvariantCulture) + ")" + s2;
        return s;
    }

    /// <summary>Writes the name of a kind.</summary>
    public static TextWriter Print(DsKind k, TextWriter os)
    {
        os.Write(SPrint(k));
        return os;
    }

    /// <summary>Writes a kind and index.</summary>
    public static TextWriter Print(DsKind k, int i, TextWriter os, string s1 = "", string s2 = "")
    {
        os.Write(SPrint(k, i, s1, s2));
        os.Flush();
        return os;
    }

    /// <summary>The two-letter name of a shape type.</summary>
    /// <exception cref="ProgramErrorException">The type is TopAbs_SHAPE.</exception>
    public static string SPrint(ShapeType t) => SPrint(ShapeToKind(t));

    /// <summary>A shape type and index.</summary>
    public static string SPrint(ShapeType t, int i) => SPrint(ShapeToKind(t), i);

    /// <summary>Writes a shape type and index.</summary>
    public static TextWriter Print(ShapeType t, int i, TextWriter s)
    {
        s.Write(SPrint(ShapeToKind(t), i));
        return s;
    }

    /// <summary>The name of an orientation.</summary>
    public static string SPrint(Orientation o) => global::Kerf.Engine.Topology.TopAbs.ShapeOrientationToString(o);

    /// <summary>The name of a configuration (UNSH, SAME, DIFF).</summary>
    public static string SPrint(DsConfig c)
    {
        string ss = "";
        switch (c)
        {
            case DsConfig.Unshgeometry:
                ss = "UNSH";
                break;
            case DsConfig.Sameoriented:
                ss = "SAME";
                break;
            case DsConfig.Difforiented:
                ss = "DIFF";
                break;
        }
        return ss;
    }

    /// <summary>Writes the name of a configuration.</summary>
    public static TextWriter Print(DsConfig c, TextWriter os)
    {
        os.Write(SPrint(c));
        return os;
    }

    /// <summary>Whether the kind is a shape type.</summary>
    public static bool IsTopology(DsKind k)
    {
        switch (k)
        {
            case DsKind.Compound:
            case DsKind.CompSolid:
            case DsKind.Solid:
            case DsKind.Shell:
            case DsKind.Face:
            case DsKind.Wire:
            case DsKind.Edge:
            case DsKind.Vertex:
                return true;
            default:
                return false;
        }
    }

    /// <summary>Whether the kind is a geometry (surface, curve, point).</summary>
    public static bool IsGeometry(DsKind k)
    {
        switch (k)
        {
            case DsKind.Surface:
            case DsKind.Curve:
            case DsKind.Point:
                return true;
            default:
                break;
        }
        return false;
    }

    /// <summary>The kind of a shape type.</summary>
    /// <exception cref="ProgramErrorException">The type is TopAbs_SHAPE.</exception>
    public static DsKind ShapeToKind(ShapeType s)
    {
        DsKind res = DsKind.Solid; // bidon
        switch (s)
        {
            case ShapeType.Vertex:
                res = DsKind.Vertex;
                break;
            case ShapeType.Edge:
                res = DsKind.Edge;
                break;
            case ShapeType.Wire:
                res = DsKind.Wire;
                break;
            case ShapeType.Face:
                res = DsKind.Face;
                break;
            case ShapeType.Shell:
                res = DsKind.Shell;
                break;
            case ShapeType.Solid:
                res = DsKind.Solid;
                break;
            case ShapeType.CompSolid:
                res = DsKind.CompSolid;
                break;
            case ShapeType.Compound:
                res = DsKind.Compound;
                break;
            default:
                throw new ProgramErrorException("TopOpeBRepDS::ShapeToKind");
        }
        return res;
    }

    /// <summary>The shape type of a kind (TopAbs_SHAPE for a geometry).</summary>
    public static ShapeType KindToShape(DsKind k)
    {
        ShapeType res = ShapeType.Shape; // bidon
        if (!IsTopology(k))
        {
            return res;
        }

        switch (k)
        {
            case DsKind.Vertex:
                res = ShapeType.Vertex;
                break;
            case DsKind.Edge:
                res = ShapeType.Edge;
                break;
            case DsKind.Wire:
                res = ShapeType.Wire;
                break;
            case DsKind.Face:
                res = ShapeType.Face;
                break;
            case DsKind.Shell:
                res = ShapeType.Shell;
                break;
            case DsKind.Solid:
                res = ShapeType.Solid;
                break;
            case DsKind.CompSolid:
                res = ShapeType.CompSolid;
                break;
            case DsKind.Compound:
                res = ShapeType.Compound;
                break;
            default:
                throw new ProgramErrorException("TopOpeBRepDS::KindToShape");
        }
        return res;
    }
}
