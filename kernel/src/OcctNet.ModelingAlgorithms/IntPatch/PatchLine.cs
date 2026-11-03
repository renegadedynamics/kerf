// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Line.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Line.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Intersection.Walking;
using Occt;
using static Kerf.Engine.Intersection.Walking.WalkingSituation;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// An intersection line between two surfaces: geometric (a line or a conic, <see cref="GLine"/>), analytic
/// (<see cref="ALine"/>), a set of points from a walking algorithm (<see cref="WLine"/>) or a
/// restriction (<see cref="RLine"/>); with the transitions of the line on each surface and its isoparametric
/// flags.
/// </summary>
/// <remarks>
/// Intersection lines are working objects (decision 609): the intersection algorithms build them, split, purge and
/// edit them in place, so their mutators are public, and they are never frozen. Maps and sequences hold them by
/// reference, as OCCT's handles.
/// </remarks>
public class PatchLine : Transient
{
    /// <summary>The kind of the line, set by the derived classes' constructors.</summary>
    protected PatchLineType _typ;

    private readonly bool _tg;
    private readonly WalkingTypeTrans _tS1;
    private readonly WalkingTypeTrans _tS2;
    private readonly WalkingSituation _sit1;
    private readonly WalkingSituation _sit2;
    private bool _uS1;
    private bool _vS1;
    private bool _uS2;
    private bool _vS2;

    /// <summary>A line whose transitions are in or out.</summary>
    protected PatchLine(bool tang, WalkingTypeTrans trans1, WalkingTypeTrans trans2)
    {
        _tg = tang;
        _tS1 = trans1;
        _tS2 = trans2;
        _sit1 = WalkingSituation.Unknown;
        _sit2 = WalkingSituation.Unknown;
        _uS1 = false;
        _vS1 = false;
        _uS2 = false;
        _vS2 = false;
    }

    /// <summary>A line whose transitions are touch.</summary>
    protected PatchLine(bool tang, WalkingSituation situ1, WalkingSituation situ2)
    {
        _tg = tang;
        _tS1 = WalkingTypeTrans.Touch;
        _tS2 = WalkingTypeTrans.Touch;
        _sit1 = situ1;
        _sit2 = situ2;
        _uS1 = false;
        _vS1 = false;
        _uS2 = false;
        _vS2 = false;
    }

    /// <summary>A member-wise copy of <paramref name="other"/> (OCCT's implicit copy constructor).</summary>
    protected PatchLine(PatchLine other)
    {
        _typ = other._typ;
        _tg = other._tg;
        _tS1 = other._tS1;
        _tS2 = other._tS2;
        _sit1 = other._sit1;
        _sit2 = other._sit2;
        _uS1 = other._uS1;
        _vS1 = other._vS1;
        _uS2 = other._uS2;
        _vS2 = other._vS2;
    }

    /// <summary>A line whose transitions are undecided.</summary>
    protected PatchLine(bool tang)
    {
        _tg = tang;
        _tS1 = WalkingTypeTrans.Undecided;
        _tS2 = WalkingTypeTrans.Undecided;
        _sit1 = WalkingSituation.Unknown;
        _sit2 = WalkingSituation.Unknown;
        _uS1 = false;
        _vS1 = false;
        _uS2 = false;
        _vS2 = false;
    }

    /// <summary>Sets the values returned by IsUIsoOnS1, ... (false by default).</summary>
    public void SetValue(bool uiso1, bool viso1, bool uiso2, bool viso2)
    {
        _uS1 = uiso1;
        _vS1 = viso1;
        _uS2 = uiso2;
        _vS2 = viso2;
    }

    /// <summary>The kind of the 3D geometry (line, circle, ..., analytic, walking, restriction).</summary>
    public PatchLineType ArcType() => _typ;

    /// <summary>True if the line is a line of tangency between the two surfaces.</summary>
    public bool IsTangent => _tg;

    /// <summary>
    /// The transition of the line on the first surface, constant along the line: in if (N1, N2, T) is right-handed
    /// (the normals and the tangent at a point), out if left-handed; touch or undecided where the normals are
    /// collinear all along the line (the other transition is then the same).
    /// </summary>
    public WalkingTypeTrans TransitionOnS1() => _tS1;

    /// <summary>The transition of the line on the second surface, constant along the line.</summary>
    public WalkingTypeTrans TransitionOnS2() => _tS2;

    /// <summary>
    /// The situation of the first surface compared to the second when the transitions are touch; throws
    /// Standard_DomainError otherwise.
    /// </summary>
    public WalkingSituation SituationS1()
    {
        if (_tS1 != WalkingTypeTrans.Touch)
        {
            throw new DomainErrorException();
        }
        return _sit1;
    }

    /// <summary>
    /// The situation of the second surface compared to the first when the transitions are touch; throws
    /// Standard_DomainError otherwise.
    /// </summary>
    public WalkingSituation SituationS2()
    {
        if (_tS2 != WalkingTypeTrans.Touch)
        {
            throw new DomainErrorException();
        }
        return _sit2;
    }

    /// <summary>True if the line is a U isoparametric of the first surface.</summary>
    public bool IsUIsoOnS1 => _uS1;

    /// <summary>True if the line is a V isoparametric of the first surface.</summary>
    public bool IsVIsoOnS1 => _vS1;

    /// <summary>True if the line is a U isoparametric of the second surface.</summary>
    public bool IsUIsoOnS2 => _uS2;

    /// <summary>True if the line is a V isoparametric of the second surface.</summary>
    public bool IsVIsoOnS2 => _vS2;
}
