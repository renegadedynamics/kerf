// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Line.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Line.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntSurf_Situation;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

/// <summary>
/// An intersection line between two surfaces: geometric (a line or a conic, <see cref="IntPatch_GLine"/>), analytic
/// (<see cref="IntPatch_ALine"/>), a set of points from a walking algorithm (<see cref="IntPatch_WLine"/>) or a
/// restriction (<see cref="IntPatch_RLine"/>); with the transitions of the line on each surface and its isoparametric
/// flags.
/// </summary>
/// <remarks>
/// Intersection lines are working objects (decision 609): the intersection algorithms build them, split, purge and
/// edit them in place, so their mutators are public, and they are never frozen. Maps and sequences hold them by
/// reference, as OCCT's handles.
/// </remarks>
public class IntPatch_Line : Standard_Transient
{
    /// <summary>The kind of the line, set by the derived classes' constructors.</summary>
    protected IntPatch_IType typ;

    private readonly bool tg;
    private readonly IntSurf_TypeTrans tS1;
    private readonly IntSurf_TypeTrans tS2;
    private readonly IntSurf_Situation sit1;
    private readonly IntSurf_Situation sit2;
    private bool uS1;
    private bool vS1;
    private bool uS2;
    private bool vS2;

    /// <summary>A line whose transitions are in or out.</summary>
    protected IntPatch_Line(bool Tang, IntSurf_TypeTrans Trans1, IntSurf_TypeTrans Trans2)
    {
        tg = Tang;
        tS1 = Trans1;
        tS2 = Trans2;
        sit1 = IntSurf_Unknown;
        sit2 = IntSurf_Unknown;
        uS1 = false;
        vS1 = false;
        uS2 = false;
        vS2 = false;
    }

    /// <summary>A line whose transitions are touch.</summary>
    protected IntPatch_Line(bool Tang, IntSurf_Situation Situ1, IntSurf_Situation Situ2)
    {
        tg = Tang;
        tS1 = IntSurf_Touch;
        tS2 = IntSurf_Touch;
        sit1 = Situ1;
        sit2 = Situ2;
        uS1 = false;
        vS1 = false;
        uS2 = false;
        vS2 = false;
    }

    /// <summary>A member-wise copy of <paramref name="theOther"/> (OCCT's implicit copy constructor).</summary>
    protected IntPatch_Line(IntPatch_Line theOther)
    {
        typ = theOther.typ;
        tg = theOther.tg;
        tS1 = theOther.tS1;
        tS2 = theOther.tS2;
        sit1 = theOther.sit1;
        sit2 = theOther.sit2;
        uS1 = theOther.uS1;
        vS1 = theOther.vS1;
        uS2 = theOther.uS2;
        vS2 = theOther.vS2;
    }

    /// <summary>A line whose transitions are undecided.</summary>
    protected IntPatch_Line(bool Tang)
    {
        tg = Tang;
        tS1 = IntSurf_Undecided;
        tS2 = IntSurf_Undecided;
        sit1 = IntSurf_Unknown;
        sit2 = IntSurf_Unknown;
        uS1 = false;
        vS1 = false;
        uS2 = false;
        vS2 = false;
    }

    /// <summary>Sets the values returned by IsUIsoOnS1, ... (false by default).</summary>
    public void SetValue(bool Uiso1, bool Viso1, bool Uiso2, bool Viso2)
    {
        uS1 = Uiso1;
        vS1 = Viso1;
        uS2 = Uiso2;
        vS2 = Viso2;
    }

    /// <summary>The kind of the 3D geometry (line, circle, ..., analytic, walking, restriction).</summary>
    public IntPatch_IType ArcType() => typ;

    /// <summary>True if the line is a line of tangency between the two surfaces.</summary>
    public bool IsTangent() => tg;

    /// <summary>
    /// The transition of the line on the first surface, constant along the line: in if (N1, N2, T) is right-handed
    /// (the normals and the tangent at a point), out if left-handed; touch or undecided where the normals are
    /// collinear all along the line (the other transition is then the same).
    /// </summary>
    public IntSurf_TypeTrans TransitionOnS1() => tS1;

    /// <summary>The transition of the line on the second surface, constant along the line.</summary>
    public IntSurf_TypeTrans TransitionOnS2() => tS2;

    /// <summary>
    /// The situation of the first surface compared to the second when the transitions are touch; throws
    /// Standard_DomainError otherwise.
    /// </summary>
    public IntSurf_Situation SituationS1()
    {
        if (tS1 != IntSurf_Touch)
        {
            throw new Standard_DomainError();
        }
        return sit1;
    }

    /// <summary>
    /// The situation of the second surface compared to the first when the transitions are touch; throws
    /// Standard_DomainError otherwise.
    /// </summary>
    public IntSurf_Situation SituationS2()
    {
        if (tS2 != IntSurf_Touch)
        {
            throw new Standard_DomainError();
        }
        return sit2;
    }

    /// <summary>True if the line is a U isoparametric of the first surface.</summary>
    public bool IsUIsoOnS1() => uS1;

    /// <summary>True if the line is a V isoparametric of the first surface.</summary>
    public bool IsVIsoOnS1() => vS1;

    /// <summary>True if the line is a U isoparametric of the second surface.</summary>
    public bool IsUIsoOnS2() => uS2;

    /// <summary>True if the line is a V isoparametric of the second surface.</summary>
    public bool IsVIsoOnS2() => vS2;
}
