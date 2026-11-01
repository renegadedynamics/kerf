// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Interference.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepDS/TopOpeBRepDS_Interference.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The attachment of a new geometry on a geometry or shape of the legacy data structure: for example an
/// intersection point on an edge or on a curve.
/// </summary>
/// <remarks>
/// <para>
/// It holds the <see cref="Transition()"/> (how the interference separates the geometry it lies on into IN and OUT),
/// the kind and index of its support (FACE, EDGE, VERTEX, SURFACE, CURVE) and the kind and index of its geometry
/// (SURFACE, CURVE, POINT, or a shape).
/// </para>
/// <para>
/// A working object of the fillet builders and the data structure: the setters are internal. The same interference
/// is often held by several lists of the data structure (a shape's and a curve's), so a change is seen by all.
/// </para>
/// </remarks>
public class TopOpeBRepDS_Interference : Standard_Transient
{
    private TopOpeBRepDS_Transition myTransition;
    private int mySupport;
    private int myGeometry;
    private TopOpeBRepDS_Kind mySupportType;
    private TopOpeBRepDS_Kind myGeometryType;

    /// <summary>An empty interference.</summary>
    /// <remarks>OCCT leaves the indices and kinds unset; the port's are 0 and POINT (C#'s zero).</remarks>
    public TopOpeBRepDS_Interference()
    {
        myTransition = new TopOpeBRepDS_Transition();
    }

    /// <summary>
    /// The interference of the geometry <paramref name="G"/> of kind <paramref name="GT"/> on the support
    /// <paramref name="S"/> of kind <paramref name="ST"/>, with the transition <paramref name="T"/>.
    /// </summary>
    public TopOpeBRepDS_Interference(in TopOpeBRepDS_Transition T,
                                     TopOpeBRepDS_Kind ST,
                                     int S,
                                     TopOpeBRepDS_Kind GT,
                                     int G)
    {
        myTransition = T;
        mySupport = S;
        myGeometry = G;
        mySupportType = ST;
        myGeometryType = GT;
    }

    /// <summary>A copy of the transition, support and geometry of <paramref name="I"/>.</summary>
    public TopOpeBRepDS_Interference(TopOpeBRepDS_Interference I)
    {
        myTransition = I.Transition();
        mySupport = I.Support();
        myGeometry = I.Geometry();
        mySupportType = I.SupportType();
        myGeometryType = I.GeometryType();
    }

    /// <summary>The transition.</summary>
    public TopOpeBRepDS_Transition Transition() => myTransition;

    internal ref TopOpeBRepDS_Transition ChangeTransition() => ref myTransition;

    internal void Transition(in TopOpeBRepDS_Transition T)
    {
        myTransition = T;
    }

    /// <summary>The geometry's kind and index, then the support's.</summary>
    public void GKGSKS(out TopOpeBRepDS_Kind GK, out int G, out TopOpeBRepDS_Kind SK, out int S)
    {
        GK = myGeometryType;
        G = myGeometry;
        SK = mySupportType;
        S = mySupport;
    }

    /// <summary>The kind of the support.</summary>
    public TopOpeBRepDS_Kind SupportType() => mySupportType;

    /// <summary>The index of the support in the data structure.</summary>
    public int Support() => mySupport;

    /// <summary>The kind of the geometry.</summary>
    public TopOpeBRepDS_Kind GeometryType() => myGeometryType;

    /// <summary>The index of the geometry in the data structure.</summary>
    public int Geometry() => myGeometry;

    internal void SetGeometry(int GI)
    {
        myGeometry = GI;
    }

    internal void SupportType(TopOpeBRepDS_Kind ST)
    {
        mySupportType = ST;
    }

    internal void Support(int S)
    {
        mySupport = S;
    }

    internal void GeometryType(TopOpeBRepDS_Kind GT)
    {
        myGeometryType = GT;
    }

    internal void Geometry(int G)
    {
        myGeometry = G;
    }

    /// <summary>Whether <paramref name="I"/> has the same support kind and index.</summary>
    public bool HasSameSupport(TopOpeBRepDS_Interference I)
        => (mySupportType == I.mySupportType && mySupport == I.mySupport);

    /// <summary>Whether <paramref name="I"/> has the same geometry kind and index.</summary>
    public bool HasSameGeometry(TopOpeBRepDS_Interference I)
        => (myGeometryType == I.myGeometryType && myGeometry == I.myGeometry);
}
