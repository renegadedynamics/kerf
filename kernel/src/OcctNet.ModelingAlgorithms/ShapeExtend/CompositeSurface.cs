// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_CompositeSurface.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_CompositeSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Engine.Healing.Extend.Parametrisation;

namespace Kerf.Engine.Healing.Extend;

/// <summary>
/// A composite surface: a grid of surfaces (patches) connected geometrically, with a global parametrisation in which
/// patch (i, j) covers [UJointValue(i), UJointValue(i+1)] x [VJointValue(j), VJointValue(j+1)], mapped linearly onto
/// the patch's own parameters. The patches should be connected (closeness of the boundary of (i, j) to that of
/// (i+1, j) and (i, j+1)) and parametrised in the same directions; it is the caller's responsibility to give such
/// patches. The surface is C0, never periodic, and closed in a direction when its first and last rows (columns) meet.
/// </summary>
/// <remarks>
/// <para>
/// A geometry (translation-rules.md, section 8): its mutators (<c>Init</c>, the joint setters,
/// <c>ComputeJointValues</c>, <c>CheckConnectivity</c>, <c>Transform</c>) are internal and refuse a frozen surface;
/// the public <see cref="WithUFirstValue"/>, <see cref="WithVFirstValue"/>, <see cref="WithUJointValues"/>,
/// <see cref="WithVJointValues"/> and <see cref="WithJointValues"/> change a copy.
/// </para>
/// <para>
/// OCCT keeps the caller's array of patches; the port keeps its own array holding the same surfaces, so a later change
/// to the caller's array doesn't reach the surface. <see cref="Patch(int, int)"/> and the other patch accessors freeze
/// the patch they hand out (as Geom_RectangularTrimmedSurface's BasisSurface), <see cref="Patches"/> and the joint
/// arrays are copies, and <c>Transform</c> copies a frozen patch before transforming it.
/// </para>
/// </remarks>
public class CompositeSurface : Surface
{
    private NCollection_HArray2<Surface>? _patches;
    private NCollection_HArray1<double>? _uJointValues;
    private NCollection_HArray1<double>? _vJointValues;
    private bool _uClosed;
    private bool _vClosed;

    /// <summary>An empty surface.</summary>
    public CompositeSurface()
    {
    }

    /// <summary>A surface on the grid of patches, with joint values computed for the parametrisation.</summary>
    public CompositeSurface(NCollection_HArray2<Surface>? gridSurf, Parametrisation param = Parametrisation.Natural)
    {
        Init(gridSurf, param);
    }

    /// <summary>A surface on the grid of patches with the given joint values.</summary>
    public CompositeSurface(NCollection_HArray2<Surface>? gridSurf, NCollection_Array1<double> uJoints,
                                        NCollection_Array1<double> vJoints)
    {
        Init(gridSurf, uJoints, vJoints);
    }

    /// <summary>
    /// Initializes the surface with the grid of patches and joint values computed for the parametrisation. False if
    /// the grid is null, or the patches aren't connected (then the surface is initialized anyway).
    /// </summary>
    internal bool Init(NCollection_HArray2<Surface>? gridSurf, Parametrisation param = Parametrisation.Natural)
    {
        CheckMutable();
        if (gridSurf is null)
        {
            return false;
        }
        _patches = new NCollection_HArray2<Surface>(gridSurf); // OCCT.NET: OCCT shares the caller's array
        ComputeJointValues(param);
        return CheckConnectivity(Kerf.Engine.Foundation.Precision.Confusion());
    }

    /// <summary>
    /// Initializes the surface with the grid of patches and the joint values. If the joint values are bad (lengths
    /// not the numbers of patches + 1, or not increasing), the natural ones are computed and false is returned; false
    /// also if the patches aren't connected.
    /// </summary>
    internal bool Init(NCollection_HArray2<Surface>? gridSurf, NCollection_Array1<double> uJoints, NCollection_Array1<double> vJoints)
    {
        CheckMutable();
        if (gridSurf is null)
        {
            return false;
        }
        _patches = new NCollection_HArray2<Surface>(gridSurf); // OCCT.NET: OCCT shares the caller's array

        bool ok = true;
        if (!SetUJointValues(uJoints) || !SetVJointValues(vJoints))
        {
            ok = false;
            ComputeJointValues(Parametrisation.Natural);
        }

        return (CheckConnectivity(Kerf.Engine.Foundation.Precision.Confusion()) ? ok : false);
    }

    /// <summary>The number of patches in U.</summary>
    public int UPatchCount => _patches!.ColLength();

    /// <summary>The number of patches in V.</summary>
    public int VPatchCount => _patches!.RowLength();

    /// <summary>The patch (i, j). Frozen (shared, not copied).</summary>
    public Surface Patch(int i, int j) => Shared(_patches!.Value(i, j));

    /// <summary>A copy of the grid of patches (the patches themselves shared, and frozen).</summary>
    public NCollection_HArray2<Surface>? Patches()
    {
        if (_patches is null)
        {
            return null;
        }
        var copy = new NCollection_HArray2<Surface>(_patches);
        for (int i = copy.LowerRow(); i <= copy.UpperRow(); i++)
        {
            for (int j = copy.LowerCol(); j <= copy.UpperCol(); j++)
            {
                Shared(copy.Value(i, j));
            }
        }
        return copy;
    }

    /// <summary>A copy of the U joint values (the global U of the patches' boundaries, NbUPatches + 1 values).</summary>
    public NCollection_HArray1<double>? UJointValues() =>
        _uJointValues is null ? null : new NCollection_HArray1<double>(_uJointValues);

    /// <summary>A copy of the V joint values (the global V of the patches' boundaries, NbVPatches + 1 values).</summary>
    public NCollection_HArray1<double>? VJointValues() =>
        _vJointValues is null ? null : new NCollection_HArray1<double>(_vJointValues);

    /// <summary>The stored U joint values, for ported code that only reads them (ShapeFix_ComposeShell).</summary>
    internal NCollection_HArray1<double>? InternalUJointValues() => _uJointValues;

    /// <summary>The stored V joint values, for ported code that only reads them (ShapeFix_ComposeShell).</summary>
    internal NCollection_HArray1<double>? InternalVJointValues() => _vJointValues;

    /// <summary>The i-th U joint value.</summary>
    public double UJointValue(int i) => _uJointValues!.Value(i);

    /// <summary>The j-th V joint value.</summary>
    public double VJointValue(int i) => _vJointValues!.Value(i);

    /// <summary>
    /// Sets the U joint values: false (nothing changed) if their number isn't NbUPatches + 1 or they don't increase.
    /// </summary>
    internal bool SetUJointValues(NCollection_Array1<double> uJoints)
    {
        CheckMutable();
        int nbU = UPatchCount;
        if (uJoints.Length() != nbU + 1)
        {
            return false;
        }

        var uJointValues = new NCollection_HArray1<double>(1, nbU + 1);
        for (int i = 1, j = uJoints.Lower(); i <= nbU + 1; i++, j++)
        {
            uJointValues.SetValue(i, uJoints[j]);
            if (i > 1 && uJoints[j] - uJoints[j - 1] < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return false;
            }
        }
        _uJointValues = uJointValues;
        return true;
    }

    /// <summary>
    /// Sets the V joint values: false (nothing changed) if their number isn't NbVPatches + 1 or they don't increase.
    /// </summary>
    internal bool SetVJointValues(NCollection_Array1<double> vJoints)
    {
        CheckMutable();
        int nbV = VPatchCount;
        if (vJoints.Length() != nbV + 1)
        {
            return false;
        }

        var vJointValues = new NCollection_HArray1<double>(1, nbV + 1);
        for (int i = 1, j = vJoints.Lower(); i <= nbV + 1; i++, j++)
        {
            vJointValues.SetValue(i, vJoints[j]);
            if (i > 1 && vJoints[j] - vJoints[j - 1] < Kerf.Engine.Foundation.Precision.PConfusion())
            {
                return false;
            }
        }
        _vJointValues = vJointValues;
        return true;
    }

    /// <summary>Shifts the U joint values so that the first is <paramref name="uFirst"/>.</summary>
    internal void SetUFirstValue(double uFirst)
    {
        CheckMutable();
        if (_uJointValues is null)
        {
            return;
        }

        double shift = uFirst - _uJointValues.Value(1);
        int nbU = _uJointValues.Length();
        for (int i = 1; i <= nbU; i++)
        {
            _uJointValues.SetValue(i, _uJointValues.Value(i) + shift);
        }
    }

    /// <summary>Shifts the V joint values so that the first is <paramref name="vFirst"/>.</summary>
    internal void SetVFirstValue(double vFirst)
    {
        CheckMutable();
        if (_vJointValues is null)
        {
            return;
        }

        double shift = vFirst - _vJointValues.Value(1);
        int nbV = _vJointValues.Length();
        for (int i = 1; i <= nbV; i++)
        {
            _vJointValues.SetValue(i, _vJointValues.Value(i) + shift);
        }
    }

    /// <summary>A copy whose U joint values start at <paramref name="uFirst"/>.</summary>
    public CompositeSurface WithUFirstValue(double uFirst)
    {
        CompositeSurface copy = CopyKeepingJoints();
        copy.SetUFirstValue(uFirst);
        return copy;
    }

    /// <summary>A copy whose V joint values start at <paramref name="vFirst"/>.</summary>
    public CompositeSurface WithVFirstValue(double vFirst)
    {
        CompositeSurface copy = CopyKeepingJoints();
        copy.SetVFirstValue(vFirst);
        return copy;
    }

    /// <summary>
    /// A copy with the U joint values; <paramref name="isDone"/> is false (and the copy keeps this surface's values)
    /// when <see cref="SetUJointValues"/> would refuse them.
    /// </summary>
    public CompositeSurface WithUJointValues(NCollection_Array1<double> uJoints, out bool isDone)
    {
        CompositeSurface copy = CopyKeepingJoints();
        isDone = copy.SetUJointValues(uJoints);
        return copy;
    }

    /// <summary>
    /// A copy with the V joint values; <paramref name="isDone"/> is false (and the copy keeps this surface's values)
    /// when <see cref="SetVJointValues"/> would refuse them.
    /// </summary>
    public CompositeSurface WithVJointValues(NCollection_Array1<double> vJoints, out bool isDone)
    {
        CompositeSurface copy = CopyKeepingJoints();
        isDone = copy.SetVJointValues(vJoints);
        return copy;
    }

    /// <summary>A copy with joint values computed for the parametrisation (<see cref="ComputeJointValues"/>).</summary>
    public CompositeSurface WithJointValues(Parametrisation param)
    {
        CompositeSurface copy = CopyKeepingJoints();
        copy.ComputeJointValues(param);
        return copy;
    }

    /// <summary>The index of the U patch holding the global parameter U (the first or last outside the range).</summary>
    public int LocateUParameter(double u)
    {
        int nbPatch = UPatchCount;
        for (int i = 2; i <= nbPatch; i++)
        {
            if (u < _uJointValues!.Value(i))
            {
                return i - 1;
            }
        }
        return nbPatch;
    }

    /// <summary>The index of the V patch holding the global parameter V (the first or last outside the range).</summary>
    public int LocateVParameter(double v)
    {
        int nbPatch = VPatchCount;
        for (int i = 2; i <= nbPatch; i++)
        {
            if (v < _vJointValues!.Value(i))
            {
                return i - 1;
            }
        }
        return nbPatch;
    }

    /// <summary>The indices of the patch holding the global point.</summary>
    public void LocateUVPoint(in Point2D pnt, out int i, out int j)
    {
        i = LocateUParameter(pnt.X());
        j = LocateVParameter(pnt.Y());
    }

    /// <summary>The patch holding the global point (U, V). Frozen.</summary>
    public Surface Patch(double u, double v) => Shared(_patches!.Value(LocateUParameter(u), LocateVParameter(v)));

    /// <summary>The patch holding the global point. Frozen.</summary>
    public Surface Patch(in Point2D pnt) => Shared(_patches!.Value(LocateUParameter(pnt.X()), LocateVParameter(pnt.Y())));

    /// <summary>The global U of the local parameter u on patch (i, j).</summary>
    public double ULocalToGlobal(int i, int j, double u)
    {
        _patches!.Value(i, j).Bounds(out double u1, out double u2, out _, out _);
        double scale = (_uJointValues!.Value(i + 1) - _uJointValues.Value(i)) / (u2 - u1);
        return u * scale + (_uJointValues.Value(i) - u1 * scale); // ! this formula is stable if u1 is infinite
    }

    /// <summary>The global V of the local parameter v on patch (i, j).</summary>
    public double VLocalToGlobal(int i, int j, double v)
    {
        _patches!.Value(i, j).Bounds(out _, out _, out double v1, out double v2);
        double scale = (_vJointValues!.Value(j + 1) - _vJointValues.Value(j)) / (v2 - v1);
        return v * scale + (_vJointValues.Value(j) - v1 * scale); // ! this formula is stable if v1 is infinite
    }

    /// <summary>The global point of the local point uv on patch (i, j).</summary>
    public Point2D LocalToGlobal(int i, int j, in Point2D uv)
    {
        _patches!.Value(i, j).Bounds(out double u1, out double u2, out double v1, out double v2);
        double scaleu = (_uJointValues!.Value(i + 1) - _uJointValues.Value(i)) / (u2 - u1);
        double scalev = (_vJointValues!.Value(j + 1) - _vJointValues.Value(j)) / (v2 - v1);
        return new Point2D(uv.X() * scaleu
                            + (_uJointValues.Value(i)
                               - u1 * scaleu), // ! this formula is stable if u1 or v1 is infinite
                            uv.Y() * scalev + (_vJointValues.Value(j) - v1 * scalev));
    }

    /// <summary>The local u on patch (i, j) of the global parameter U.</summary>
    public double UGlobalToLocal(int i, int j, double u)
    {
        _patches!.Value(i, j).Bounds(out double u1, out double u2, out _, out _);
        double scale = (u2 - u1) / (_uJointValues!.Value(i + 1) - _uJointValues.Value(i));
        return u * scale + (u1 - _uJointValues.Value(i) * scale); // ! this formula is stable if u1 is infinite
    }

    /// <summary>The local v on patch (i, j) of the global parameter V.</summary>
    public double VGlobalToLocal(int i, int j, double v)
    {
        _patches!.Value(i, j).Bounds(out _, out _, out double v1, out double v2);
        double scale = (v2 - v1) / (_vJointValues!.Value(j + 1) - _vJointValues.Value(j));
        return v * scale + (v1 - _vJointValues.Value(j) * scale); // ! this formula is stable if v1 is infinite
    }

    /// <summary>The local point on patch (i, j) of the global point UV.</summary>
    public Point2D GlobalToLocal(int i, int j, in Point2D uv)
    {
        _patches!.Value(i, j).Bounds(out double u1, out double u2, out double v1, out double v2);
        double scaleu = (u2 - u1) / (_uJointValues!.Value(i + 1) - _uJointValues.Value(i));
        double scalev = (v2 - v1) / (_vJointValues!.Value(j + 1) - _vJointValues.Value(j));
        return new Point2D(
            uv.X() * scaleu
            + (u1
               - _uJointValues.Value(i) * scaleu), // ! this formula is stable if u1 or v1 is infinite
            uv.Y() * scalev + (v1 - _vJointValues.Value(j) * scalev));
    }

    /// <summary>
    /// The transformation from the global parameters to the local ones of patch (i, j), as a 2D transformation
    /// <paramref name="trsf"/> (applied after scaling U by <paramref name="uFact"/>). True unless it is the identity.
    /// </summary>
    public bool GlobalToLocalTransformation(int i, int j, out double uFact, out Transform2D trsf)
    {
        _patches!.Value(i, j).Bounds(out double u1, out double u2, out double v1, out double v2);

        double scaleu = (u2 - u1) / (_uJointValues!.Value(i + 1) - _uJointValues.Value(i));
        double scalev = (v2 - v1) / (_vJointValues!.Value(j + 1) - _vJointValues.Value(j));
        var shift = new Vector2D(u1 / scaleu - _uJointValues.Value(i), v1 / scalev - _vJointValues.Value(j));

        uFact = scaleu / scalev;
        Transform2D Shift = new(), scale = new();
        if (shift.X() != 0.0 || shift.Y() != 0.0)
        {
            Shift.SetTranslation(shift);
        }
        if (scalev != 1.0)
        {
            scale.SetScale(new Point2D(0, 0), scalev);
        }
        trsf = scale * Shift;
        return uFact != 1.0 || trsf.Form() != TransformForm.Identity;
    }

    //=======================================================================
    // Inherited methods (from Geom_Geometry and Geom_Surface)
    //=======================================================================

    /// <summary>Transforms each patch (a frozen patch is copied first).</summary>
    protected internal override void Transform(in Transform3D t)
    {
        CheckMutable();
        if (_patches is null)
        {
            return;
        }
        for (int i = 1; i <= UPatchCount; i++)
        {
            for (int j = 1; j <= VPatchCount; j++)
            {
                // OCCT.NET: frozen: OCCT transforms the patch in place, which may be shared; the port transforms its
                // own copy of a frozen patch.
                Surface patch = _patches.Value(i, j);
                if (patch.IsFrozen)
                {
                    patch = (Surface)patch.Copy();
                    _patches.SetValue(i, j, patch);
                }
                patch.Transform(t);
            }
        }
    }

    /// <summary>
    /// A copy with copies of the patches, initialized with the natural parametrisation as OCCT's (the joint values
    /// aren't kept).
    /// </summary>
    public override Geometry3D Copy()
    {
        var surf = new CompositeSurface();
        if (_patches is null)
        {
            return surf;
        }

        var patches = new NCollection_HArray2<Surface>(1, UPatchCount, 1, VPatchCount);
        for (int i = 1; i <= UPatchCount; i++)
        {
            for (int j = 1; j <= VPatchCount; j++)
            {
                patches.SetValue(i, j, (Surface)_patches.Value(i, j).Copy());
            }
        }
        surf.Init(patches); // OCCT: the copy gets natural joint values, whatever this surface's are
        return surf;
    }

    // OCCT.NET: a copy keeping the joint values, for the With* twins (Copy() resets them, as OCCT's).
    private CompositeSurface CopyKeepingJoints()
    {
        var surf = (CompositeSurface)Copy();
        if (_uJointValues is not null)
        {
            surf._uJointValues = new NCollection_HArray1<double>(_uJointValues);
        }
        if (_vJointValues is not null)
        {
            surf._vJointValues = new NCollection_HArray1<double>(_vJointValues);
        }
        return surf;
    }

    /// <summary>Does nothing (as OCCT).</summary>
    protected internal override void UReverse()
    {
    }

    /// <summary>Returns U (as OCCT: the surface isn't reversed).</summary>
    public override double UReversedParameter(double u) => u;

    /// <summary>Does nothing (as OCCT).</summary>
    protected internal override void VReverse()
    {
    }

    /// <summary>Returns V (as OCCT: the surface isn't reversed).</summary>
    public override double VReversedParameter(double v) => v;

    /// <summary>The first and last joint values.</summary>
    public override void Bounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = UJointValue(1);
        v1 = VJointValue(1);
        u2 = UJointValue(UPatchCount + 1);
        v2 = VJointValue(VPatchCount + 1);
    }

    /// <summary>False.</summary>
    public override bool IsUPeriodic => false;

    /// <summary>False.</summary>
    public override bool IsVPeriodic => false;

    /// <summary>A null curve: not implemented (as OCCT).</summary>
    public override Curve3D UIso(double u) => null!; // OCCT returns a null handle

    /// <summary>A null curve: not implemented (as OCCT).</summary>
    public override Curve3D VIso(double v) => null!; // OCCT returns a null handle

    /// <summary>C0.</summary>
    public override Continuity Continuity() => C0;

    /// <summary>True for N &lt;= 0 only.</summary>
    public override bool IsCNu(int n) => n <= 0;

    /// <summary>True for N &lt;= 0 only.</summary>
    public override bool IsCNv(int n) => n <= 0;

    /// <summary>True if the first and last U rows meet (computed by <see cref="CheckConnectivity"/>).</summary>
    public override bool IsUClosed => _uClosed;

    /// <summary>True if the first and last V columns meet (computed by <see cref="CheckConnectivity"/>).</summary>
    public override bool IsVClosed => _vClosed;

    public override Point3D EvalD0(double u, double v)
    {
        int i = LocateUParameter(u);
        int j = LocateVParameter(v);
        Point2D uv = GlobalToLocal(i, j, new Point2D(u, v));
        return _patches!.Value(i, j).EvalD0(uv.X(), uv.Y());
    }

    // OCCT returns the patch's derivatives unscaled by the joint values.
    public override ResD1 EvalD1(double u, double v)
    {
        int i = LocateUParameter(u);
        int j = LocateVParameter(v);
        Point2D uv = GlobalToLocal(i, j, new Point2D(u, v));
        return _patches!.Value(i, j).EvalD1(uv.X(), uv.Y());
    }

    public override ResD2 EvalD2(double u, double v)
    {
        int i = LocateUParameter(u);
        int j = LocateVParameter(v);
        Point2D uv = GlobalToLocal(i, j, new Point2D(u, v));
        return _patches!.Value(i, j).EvalD2(uv.X(), uv.Y());
    }

    public override ResD3 EvalD3(double u, double v)
    {
        int i = LocateUParameter(u);
        int j = LocateVParameter(v);
        Point2D uv = GlobalToLocal(i, j, new Point2D(u, v));
        return _patches!.Value(i, j).EvalD3(uv.X(), uv.Y());
    }

    public override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        int i = LocateUParameter(u);
        int j = LocateVParameter(v);
        Point2D uv = GlobalToLocal(i, j, new Point2D(u, v));
        return _patches!.Value(i, j).EvalDN(uv.X(), uv.Y(), nu, nv);
    }

    /// <summary>The point at the global parameters.</summary>
    public Point3D Value(in Point2D pnt)
    {
        int i = LocateUParameter(pnt.X());
        int j = LocateVParameter(pnt.Y());
        Point2D uv = GlobalToLocal(i, j, pnt);
        _patches!.Value(i, j).D0(uv.X(), uv.Y(), out Point3D point);
        return point;
    }

    /// <summary>
    /// Computes the joint values: natural (each patch of the first row and column adds its range), uniform (range
    /// 1 each) or unitary (the global range [0, 1]).
    /// </summary>
    internal void ComputeJointValues(Parametrisation param = Parametrisation.Natural)
    {
        CheckMutable();
        int nbU = UPatchCount;
        int nbV = VPatchCount;
        _uJointValues = new NCollection_HArray1<double>(1, nbU + 1);
        _vJointValues = new NCollection_HArray1<double>(1, nbV + 1);

        if (param == Parametrisation.Natural)
        {
            double u1, u2, v1, v2, u = 0, v = 0;
            int i; // svv Jan 10 2000 : porting on DEC
            for (i = 1; i <= nbU; i++)
            {
                _patches!.Value(i, 1).Bounds(out u1, out u2, out v1, out v2);
                if (i == 1)
                {
                    _uJointValues.SetValue(1, u = u1);
                }
                u += (u2 - u1);
                _uJointValues.SetValue(i + 1, u);
            }
            for (i = 1; i <= nbV; i++)
            {
                _patches!.Value(1, i).Bounds(out u1, out u2, out v1, out v2);
                if (i == 1)
                {
                    _vJointValues.SetValue(1, v = v1);
                }
                v += (v2 - v1);
                _vJointValues.SetValue(i + 1, v);
            }
        }
        else
        {
            double stepu = 1.0, stepv = 1.0; // suppose param == ShapeExtend_Uniform
            if (param == Parametrisation.Unitary)
            {
                stepu /= nbU;
                stepv /= nbV;
            }
            int i; // svv Jan 10 2000 : porting on DEC
            for (i = 0; i <= nbU; i++)
            {
                _uJointValues.SetValue(i + 1, i * stepu);
            }
            for (i = 0; i <= nbV; i++)
            {
                _vJointValues.SetValue(i + 1, i * stepv);
            }
        }
    }

    private static double LimitValue(double par) =>
        Kerf.Engine.Foundation.Precision.IsInfinite(par) ? (par < 0 ? -10000.0 : 10000.0) : par;

    private static void GetLimitedBounds(Surface surf, out double u1, out double u2, out double v1, out double v2)
    {
        surf.Bounds(out u1, out u2, out v1, out v2);
        u1 = LimitValue(u1);
        u2 = LimitValue(u2);
        v1 = LimitValue(v1);
        v2 = LimitValue(v2);
    }

    /// <summary>
    /// Checks the geometric connectivity of the patches, with the tolerance <paramref name="prec"/>, sampling 23
    /// points along each boundary, and sets the closure flags. True if every inner boundary is connected.
    /// </summary>
    internal bool CheckConnectivity(double prec)
    {
        CheckMutable();
        const int npoints = 23;
        bool ok = true;
        int nbU = UPatchCount;
        int nbV = VPatchCount;

        // check in u direction
        int i, j; // svv Jan 10 2000 : porting on DEC
        for (i = 1, j = nbU; i <= nbU; j = i++)
        {
            double maxdist2 = 0.0;
            for (int k = 1; k <= nbV; k++)
            {
                Surface sj = _patches!.Value(j, k);
                Surface si = _patches.Value(i, k);
                GetLimitedBounds(sj, out _, out double uj2, out double vj1, out double vj2);
                GetLimitedBounds(si, out double ui1, out _, out double vi1, out double vi2);
                double stepj = (vj2 - vj1) / (npoints - 1);
                double stepi = (vi2 - vi1) / (npoints - 1);
                for (int isample = 0; isample < npoints; isample++)
                {
                    double parj = vj1 + stepj * isample;
                    double pari = vi1 + stepi * isample;
                    double dist2 = sj.Value(uj2, parj).SquareDistance(si.Value(ui1, pari));
                    if (maxdist2 < dist2)
                    {
                        maxdist2 = dist2;
                    }
                }
            }
            if (i == 1)
            {
                _uClosed = (maxdist2 <= prec * prec);
            }
            else if (maxdist2 > prec * prec)
            {
                ok = false;
            }
        }

        // check in v direction
        for (i = 1, j = nbV; i <= nbV; j = i++)
        {
            double maxdist2 = 0.0;
            for (int k = 1; k <= nbU; k++)
            {
                Surface sj = _patches!.Value(k, j);
                Surface si = _patches.Value(k, i);
                GetLimitedBounds(sj, out double uj1, out double uj2, out _, out double vj2);
                GetLimitedBounds(si, out double ui1, out double ui2, out double vi1, out _);
                double stepj = (uj2 - uj1) / (npoints - 1);
                double stepi = (ui2 - ui1) / (npoints - 1);
                for (int isample = 0; isample < npoints; isample++)
                {
                    double parj = uj1 + stepj * isample;
                    double pari = ui1 + stepi * isample;
                    double dist2 = sj.Value(parj, vj2).SquareDistance(si.Value(pari, vi1));
                    if (maxdist2 < dist2)
                    {
                        maxdist2 = dist2;
                    }
                }
            }
            if (i == 1)
            {
                _vClosed = (maxdist2 <= prec * prec);
            }
            else if (maxdist2 > prec * prec)
            {
                ok = false;
            }
        }

        return ok;
    }

    // Freezes a patch handed out (shared, not copied), as Geom_RectangularTrimmedSurface's BasisSurface.
    private static Surface Shared(Surface patch)
    {
        patch.Freeze();
        return patch;
    }
}
