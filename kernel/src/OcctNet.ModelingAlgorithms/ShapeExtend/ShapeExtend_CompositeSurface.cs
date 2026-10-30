// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_CompositeSurface.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeExtend/ShapeExtend_CompositeSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;
using static Occt.ShapeExtend_Parametrisation;

namespace Occt;

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
public class ShapeExtend_CompositeSurface : Geom_Surface
{
    private NCollection_HArray2<Geom_Surface>? myPatches;
    private NCollection_HArray1<double>? myUJointValues;
    private NCollection_HArray1<double>? myVJointValues;
    private bool myUClosed;
    private bool myVClosed;

    /// <summary>An empty surface.</summary>
    public ShapeExtend_CompositeSurface()
    {
    }

    /// <summary>A surface on the grid of patches, with joint values computed for the parametrisation.</summary>
    public ShapeExtend_CompositeSurface(NCollection_HArray2<Geom_Surface>? GridSurf, ShapeExtend_Parametrisation param = ShapeExtend_Natural)
    {
        Init(GridSurf, param);
    }

    /// <summary>A surface on the grid of patches with the given joint values.</summary>
    public ShapeExtend_CompositeSurface(NCollection_HArray2<Geom_Surface>? GridSurf, NCollection_Array1<double> UJoints,
                                        NCollection_Array1<double> VJoints)
    {
        Init(GridSurf, UJoints, VJoints);
    }

    /// <summary>
    /// Initializes the surface with the grid of patches and joint values computed for the parametrisation. False if
    /// the grid is null, or the patches aren't connected (then the surface is initialized anyway).
    /// </summary>
    internal bool Init(NCollection_HArray2<Geom_Surface>? GridSurf, ShapeExtend_Parametrisation param = ShapeExtend_Natural)
    {
        CheckMutable();
        if (GridSurf is null)
        {
            return false;
        }
        myPatches = new NCollection_HArray2<Geom_Surface>(GridSurf); // OCCT.NET: OCCT shares the caller's array
        ComputeJointValues(param);
        return CheckConnectivity(Occt.Precision.Confusion());
    }

    /// <summary>
    /// Initializes the surface with the grid of patches and the joint values. If the joint values are bad (lengths
    /// not the numbers of patches + 1, or not increasing), the natural ones are computed and false is returned; false
    /// also if the patches aren't connected.
    /// </summary>
    internal bool Init(NCollection_HArray2<Geom_Surface>? GridSurf, NCollection_Array1<double> UJoints, NCollection_Array1<double> VJoints)
    {
        CheckMutable();
        if (GridSurf is null)
        {
            return false;
        }
        myPatches = new NCollection_HArray2<Geom_Surface>(GridSurf); // OCCT.NET: OCCT shares the caller's array

        bool ok = true;
        if (!SetUJointValues(UJoints) || !SetVJointValues(VJoints))
        {
            ok = false;
            ComputeJointValues(ShapeExtend_Natural);
        }

        return (CheckConnectivity(Occt.Precision.Confusion()) ? ok : false);
    }

    /// <summary>The number of patches in U.</summary>
    public int NbUPatches() => myPatches!.ColLength();

    /// <summary>The number of patches in V.</summary>
    public int NbVPatches() => myPatches!.RowLength();

    /// <summary>The patch (i, j). Frozen (shared, not copied).</summary>
    public Geom_Surface Patch(int i, int j) => Shared(myPatches!.Value(i, j));

    /// <summary>A copy of the grid of patches (the patches themselves shared, and frozen).</summary>
    public NCollection_HArray2<Geom_Surface>? Patches()
    {
        if (myPatches is null)
        {
            return null;
        }
        var aCopy = new NCollection_HArray2<Geom_Surface>(myPatches);
        for (int i = aCopy.LowerRow(); i <= aCopy.UpperRow(); i++)
        {
            for (int j = aCopy.LowerCol(); j <= aCopy.UpperCol(); j++)
            {
                Shared(aCopy.Value(i, j));
            }
        }
        return aCopy;
    }

    /// <summary>A copy of the U joint values (the global U of the patches' boundaries, NbUPatches + 1 values).</summary>
    public NCollection_HArray1<double>? UJointValues() =>
        myUJointValues is null ? null : new NCollection_HArray1<double>(myUJointValues);

    /// <summary>A copy of the V joint values (the global V of the patches' boundaries, NbVPatches + 1 values).</summary>
    public NCollection_HArray1<double>? VJointValues() =>
        myVJointValues is null ? null : new NCollection_HArray1<double>(myVJointValues);

    /// <summary>The i-th U joint value.</summary>
    public double UJointValue(int i) => myUJointValues!.Value(i);

    /// <summary>The j-th V joint value.</summary>
    public double VJointValue(int i) => myVJointValues!.Value(i);

    /// <summary>
    /// Sets the U joint values: false (nothing changed) if their number isn't NbUPatches + 1 or they don't increase.
    /// </summary>
    internal bool SetUJointValues(NCollection_Array1<double> UJoints)
    {
        CheckMutable();
        int NbU = NbUPatches();
        if (UJoints.Length() != NbU + 1)
        {
            return false;
        }

        var UJointValues = new NCollection_HArray1<double>(1, NbU + 1);
        for (int i = 1, j = UJoints.Lower(); i <= NbU + 1; i++, j++)
        {
            UJointValues.SetValue(i, UJoints[j]);
            if (i > 1 && UJoints[j] - UJoints[j - 1] < Occt.Precision.PConfusion())
            {
                return false;
            }
        }
        myUJointValues = UJointValues;
        return true;
    }

    /// <summary>
    /// Sets the V joint values: false (nothing changed) if their number isn't NbVPatches + 1 or they don't increase.
    /// </summary>
    internal bool SetVJointValues(NCollection_Array1<double> VJoints)
    {
        CheckMutable();
        int NbV = NbVPatches();
        if (VJoints.Length() != NbV + 1)
        {
            return false;
        }

        var VJointValues = new NCollection_HArray1<double>(1, NbV + 1);
        for (int i = 1, j = VJoints.Lower(); i <= NbV + 1; i++, j++)
        {
            VJointValues.SetValue(i, VJoints[j]);
            if (i > 1 && VJoints[j] - VJoints[j - 1] < Occt.Precision.PConfusion())
            {
                return false;
            }
        }
        myVJointValues = VJointValues;
        return true;
    }

    /// <summary>Shifts the U joint values so that the first is <paramref name="UFirst"/>.</summary>
    internal void SetUFirstValue(double UFirst)
    {
        CheckMutable();
        if (myUJointValues is null)
        {
            return;
        }

        double shift = UFirst - myUJointValues.Value(1);
        int NbU = myUJointValues.Length();
        for (int i = 1; i <= NbU; i++)
        {
            myUJointValues.SetValue(i, myUJointValues.Value(i) + shift);
        }
    }

    /// <summary>Shifts the V joint values so that the first is <paramref name="VFirst"/>.</summary>
    internal void SetVFirstValue(double VFirst)
    {
        CheckMutable();
        if (myVJointValues is null)
        {
            return;
        }

        double shift = VFirst - myVJointValues.Value(1);
        int NbV = myVJointValues.Length();
        for (int i = 1; i <= NbV; i++)
        {
            myVJointValues.SetValue(i, myVJointValues.Value(i) + shift);
        }
    }

    /// <summary>A copy whose U joint values start at <paramref name="UFirst"/>.</summary>
    public ShapeExtend_CompositeSurface WithUFirstValue(double UFirst)
    {
        ShapeExtend_CompositeSurface aCopy = CopyKeepingJoints();
        aCopy.SetUFirstValue(UFirst);
        return aCopy;
    }

    /// <summary>A copy whose V joint values start at <paramref name="VFirst"/>.</summary>
    public ShapeExtend_CompositeSurface WithVFirstValue(double VFirst)
    {
        ShapeExtend_CompositeSurface aCopy = CopyKeepingJoints();
        aCopy.SetVFirstValue(VFirst);
        return aCopy;
    }

    /// <summary>
    /// A copy with the U joint values; <paramref name="isDone"/> is false (and the copy keeps this surface's values)
    /// when <see cref="SetUJointValues"/> would refuse them.
    /// </summary>
    public ShapeExtend_CompositeSurface WithUJointValues(NCollection_Array1<double> UJoints, out bool isDone)
    {
        ShapeExtend_CompositeSurface aCopy = CopyKeepingJoints();
        isDone = aCopy.SetUJointValues(UJoints);
        return aCopy;
    }

    /// <summary>
    /// A copy with the V joint values; <paramref name="isDone"/> is false (and the copy keeps this surface's values)
    /// when <see cref="SetVJointValues"/> would refuse them.
    /// </summary>
    public ShapeExtend_CompositeSurface WithVJointValues(NCollection_Array1<double> VJoints, out bool isDone)
    {
        ShapeExtend_CompositeSurface aCopy = CopyKeepingJoints();
        isDone = aCopy.SetVJointValues(VJoints);
        return aCopy;
    }

    /// <summary>A copy with joint values computed for the parametrisation (<see cref="ComputeJointValues"/>).</summary>
    public ShapeExtend_CompositeSurface WithJointValues(ShapeExtend_Parametrisation param)
    {
        ShapeExtend_CompositeSurface aCopy = CopyKeepingJoints();
        aCopy.ComputeJointValues(param);
        return aCopy;
    }

    /// <summary>The index of the U patch holding the global parameter U (the first or last outside the range).</summary>
    public int LocateUParameter(double U)
    {
        int nbPatch = NbUPatches();
        for (int i = 2; i <= nbPatch; i++)
        {
            if (U < myUJointValues!.Value(i))
            {
                return i - 1;
            }
        }
        return nbPatch;
    }

    /// <summary>The index of the V patch holding the global parameter V (the first or last outside the range).</summary>
    public int LocateVParameter(double V)
    {
        int nbPatch = NbVPatches();
        for (int i = 2; i <= nbPatch; i++)
        {
            if (V < myVJointValues!.Value(i))
            {
                return i - 1;
            }
        }
        return nbPatch;
    }

    /// <summary>The indices of the patch holding the global point.</summary>
    public void LocateUVPoint(in gp_Pnt2d pnt, out int i, out int j)
    {
        i = LocateUParameter(pnt.X());
        j = LocateVParameter(pnt.Y());
    }

    /// <summary>The patch holding the global point (U, V). Frozen.</summary>
    public Geom_Surface Patch(double U, double V) => Shared(myPatches!.Value(LocateUParameter(U), LocateVParameter(V)));

    /// <summary>The patch holding the global point. Frozen.</summary>
    public Geom_Surface Patch(in gp_Pnt2d pnt) => Shared(myPatches!.Value(LocateUParameter(pnt.X()), LocateVParameter(pnt.Y())));

    /// <summary>The global U of the local parameter u on patch (i, j).</summary>
    public double ULocalToGlobal(int i, int j, double u)
    {
        myPatches!.Value(i, j).Bounds(out double u1, out double u2, out _, out _);
        double scale = (myUJointValues!.Value(i + 1) - myUJointValues.Value(i)) / (u2 - u1);
        return u * scale + (myUJointValues.Value(i) - u1 * scale); // ! this formula is stable if u1 is infinite
    }

    /// <summary>The global V of the local parameter v on patch (i, j).</summary>
    public double VLocalToGlobal(int i, int j, double v)
    {
        myPatches!.Value(i, j).Bounds(out _, out _, out double v1, out double v2);
        double scale = (myVJointValues!.Value(j + 1) - myVJointValues.Value(j)) / (v2 - v1);
        return v * scale + (myVJointValues.Value(j) - v1 * scale); // ! this formula is stable if v1 is infinite
    }

    /// <summary>The global point of the local point uv on patch (i, j).</summary>
    public gp_Pnt2d LocalToGlobal(int i, int j, in gp_Pnt2d uv)
    {
        myPatches!.Value(i, j).Bounds(out double u1, out double u2, out double v1, out double v2);
        double scaleu = (myUJointValues!.Value(i + 1) - myUJointValues.Value(i)) / (u2 - u1);
        double scalev = (myVJointValues!.Value(j + 1) - myVJointValues.Value(j)) / (v2 - v1);
        return new gp_Pnt2d(uv.X() * scaleu
                            + (myUJointValues.Value(i)
                               - u1 * scaleu), // ! this formula is stable if u1 or v1 is infinite
                            uv.Y() * scalev + (myVJointValues.Value(j) - v1 * scalev));
    }

    /// <summary>The local u on patch (i, j) of the global parameter U.</summary>
    public double UGlobalToLocal(int i, int j, double U)
    {
        myPatches!.Value(i, j).Bounds(out double u1, out double u2, out _, out _);
        double scale = (u2 - u1) / (myUJointValues!.Value(i + 1) - myUJointValues.Value(i));
        return U * scale + (u1 - myUJointValues.Value(i) * scale); // ! this formula is stable if u1 is infinite
    }

    /// <summary>The local v on patch (i, j) of the global parameter V.</summary>
    public double VGlobalToLocal(int i, int j, double V)
    {
        myPatches!.Value(i, j).Bounds(out _, out _, out double v1, out double v2);
        double scale = (v2 - v1) / (myVJointValues!.Value(j + 1) - myVJointValues.Value(j));
        return V * scale + (v1 - myVJointValues.Value(j) * scale); // ! this formula is stable if v1 is infinite
    }

    /// <summary>The local point on patch (i, j) of the global point UV.</summary>
    public gp_Pnt2d GlobalToLocal(int i, int j, in gp_Pnt2d UV)
    {
        myPatches!.Value(i, j).Bounds(out double u1, out double u2, out double v1, out double v2);
        double scaleu = (u2 - u1) / (myUJointValues!.Value(i + 1) - myUJointValues.Value(i));
        double scalev = (v2 - v1) / (myVJointValues!.Value(j + 1) - myVJointValues.Value(j));
        return new gp_Pnt2d(
            UV.X() * scaleu
            + (u1
               - myUJointValues.Value(i) * scaleu), // ! this formula is stable if u1 or v1 is infinite
            UV.Y() * scalev + (v1 - myVJointValues.Value(j) * scalev));
    }

    /// <summary>
    /// The transformation from the global parameters to the local ones of patch (i, j), as a 2D transformation
    /// <paramref name="Trsf"/> (applied after scaling U by <paramref name="uFact"/>). True unless it is the identity.
    /// </summary>
    public bool GlobalToLocalTransformation(int i, int j, out double uFact, out gp_Trsf2d Trsf)
    {
        myPatches!.Value(i, j).Bounds(out double u1, out double u2, out double v1, out double v2);

        double scaleu = (u2 - u1) / (myUJointValues!.Value(i + 1) - myUJointValues.Value(i));
        double scalev = (v2 - v1) / (myVJointValues!.Value(j + 1) - myVJointValues.Value(j));
        var shift = new gp_Vec2d(u1 / scaleu - myUJointValues.Value(i), v1 / scalev - myVJointValues.Value(j));

        uFact = scaleu / scalev;
        gp_Trsf2d Shift = new(), Scale = new();
        if (shift.X() != 0.0 || shift.Y() != 0.0)
        {
            Shift.SetTranslation(shift);
        }
        if (scalev != 1.0)
        {
            Scale.SetScale(new gp_Pnt2d(0, 0), scalev);
        }
        Trsf = Scale * Shift;
        return uFact != 1.0 || Trsf.Form() != gp_TrsfForm.gp_Identity;
    }

    //=======================================================================
    // Inherited methods (from Geom_Geometry and Geom_Surface)
    //=======================================================================

    /// <summary>Transforms each patch (a frozen patch is copied first).</summary>
    protected internal override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        if (myPatches is null)
        {
            return;
        }
        for (int i = 1; i <= NbUPatches(); i++)
        {
            for (int j = 1; j <= NbVPatches(); j++)
            {
                // OCCT.NET: frozen: OCCT transforms the patch in place, which may be shared; the port transforms its
                // own copy of a frozen patch.
                Geom_Surface aPatch = myPatches.Value(i, j);
                if (aPatch.IsFrozen())
                {
                    aPatch = (Geom_Surface)aPatch.Copy();
                    myPatches.SetValue(i, j, aPatch);
                }
                aPatch.Transform(T);
            }
        }
    }

    /// <summary>
    /// A copy with copies of the patches, initialized with the natural parametrisation as OCCT's (the joint values
    /// aren't kept).
    /// </summary>
    public override Geom_Geometry Copy()
    {
        var surf = new ShapeExtend_CompositeSurface();
        if (myPatches is null)
        {
            return surf;
        }

        var patches = new NCollection_HArray2<Geom_Surface>(1, NbUPatches(), 1, NbVPatches());
        for (int i = 1; i <= NbUPatches(); i++)
        {
            for (int j = 1; j <= NbVPatches(); j++)
            {
                patches.SetValue(i, j, (Geom_Surface)myPatches.Value(i, j).Copy());
            }
        }
        surf.Init(patches); // OCCT: the copy gets natural joint values, whatever this surface's are
        return surf;
    }

    // OCCT.NET: a copy keeping the joint values, for the With* twins (Copy() resets them, as OCCT's).
    private ShapeExtend_CompositeSurface CopyKeepingJoints()
    {
        var surf = (ShapeExtend_CompositeSurface)Copy();
        if (myUJointValues is not null)
        {
            surf.myUJointValues = new NCollection_HArray1<double>(myUJointValues);
        }
        if (myVJointValues is not null)
        {
            surf.myVJointValues = new NCollection_HArray1<double>(myVJointValues);
        }
        return surf;
    }

    /// <summary>Does nothing (as OCCT).</summary>
    protected internal override void UReverse()
    {
    }

    /// <summary>Returns U (as OCCT: the surface isn't reversed).</summary>
    public override double UReversedParameter(double U) => U;

    /// <summary>Does nothing (as OCCT).</summary>
    protected internal override void VReverse()
    {
    }

    /// <summary>Returns V (as OCCT: the surface isn't reversed).</summary>
    public override double VReversedParameter(double V) => V;

    /// <summary>The first and last joint values.</summary>
    public override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = UJointValue(1);
        V1 = VJointValue(1);
        U2 = UJointValue(NbUPatches() + 1);
        V2 = VJointValue(NbVPatches() + 1);
    }

    /// <summary>False.</summary>
    public override bool IsUPeriodic() => false;

    /// <summary>False.</summary>
    public override bool IsVPeriodic() => false;

    /// <summary>A null curve: not implemented (as OCCT).</summary>
    public override Geom_Curve UIso(double U) => null!; // OCCT returns a null handle

    /// <summary>A null curve: not implemented (as OCCT).</summary>
    public override Geom_Curve VIso(double V) => null!; // OCCT returns a null handle

    /// <summary>C0.</summary>
    public override GeomAbs_Shape Continuity() => GeomAbs_C0;

    /// <summary>True for N &lt;= 0 only.</summary>
    public override bool IsCNu(int N) => N <= 0;

    /// <summary>True for N &lt;= 0 only.</summary>
    public override bool IsCNv(int N) => N <= 0;

    /// <summary>True if the first and last U rows meet (computed by <see cref="CheckConnectivity"/>).</summary>
    public override bool IsUClosed() => myUClosed;

    /// <summary>True if the first and last V columns meet (computed by <see cref="CheckConnectivity"/>).</summary>
    public override bool IsVClosed() => myVClosed;

    public override gp_Pnt EvalD0(double U, double V)
    {
        int i = LocateUParameter(U);
        int j = LocateVParameter(V);
        gp_Pnt2d uv = GlobalToLocal(i, j, new gp_Pnt2d(U, V));
        return myPatches!.Value(i, j).EvalD0(uv.X(), uv.Y());
    }

    // OCCT returns the patch's derivatives unscaled by the joint values.
    public override ResD1 EvalD1(double U, double V)
    {
        int i = LocateUParameter(U);
        int j = LocateVParameter(V);
        gp_Pnt2d uv = GlobalToLocal(i, j, new gp_Pnt2d(U, V));
        return myPatches!.Value(i, j).EvalD1(uv.X(), uv.Y());
    }

    public override ResD2 EvalD2(double U, double V)
    {
        int i = LocateUParameter(U);
        int j = LocateVParameter(V);
        gp_Pnt2d uv = GlobalToLocal(i, j, new gp_Pnt2d(U, V));
        return myPatches!.Value(i, j).EvalD2(uv.X(), uv.Y());
    }

    public override ResD3 EvalD3(double U, double V)
    {
        int i = LocateUParameter(U);
        int j = LocateVParameter(V);
        gp_Pnt2d uv = GlobalToLocal(i, j, new gp_Pnt2d(U, V));
        return myPatches!.Value(i, j).EvalD3(uv.X(), uv.Y());
    }

    public override gp_Vec EvalDN(double U, double V, int Nu, int Nv)
    {
        int i = LocateUParameter(U);
        int j = LocateVParameter(V);
        gp_Pnt2d uv = GlobalToLocal(i, j, new gp_Pnt2d(U, V));
        return myPatches!.Value(i, j).EvalDN(uv.X(), uv.Y(), Nu, Nv);
    }

    /// <summary>The point at the global parameters.</summary>
    public gp_Pnt Value(in gp_Pnt2d pnt)
    {
        int i = LocateUParameter(pnt.X());
        int j = LocateVParameter(pnt.Y());
        gp_Pnt2d uv = GlobalToLocal(i, j, pnt);
        myPatches!.Value(i, j).D0(uv.X(), uv.Y(), out gp_Pnt point);
        return point;
    }

    /// <summary>
    /// Computes the joint values: natural (each patch of the first row and column adds its range), uniform (range
    /// 1 each) or unitary (the global range [0, 1]).
    /// </summary>
    internal void ComputeJointValues(ShapeExtend_Parametrisation param = ShapeExtend_Natural)
    {
        CheckMutable();
        int NbU = NbUPatches();
        int NbV = NbVPatches();
        myUJointValues = new NCollection_HArray1<double>(1, NbU + 1);
        myVJointValues = new NCollection_HArray1<double>(1, NbV + 1);

        if (param == ShapeExtend_Natural)
        {
            double U1, U2, V1, V2, U = 0, V = 0;
            int i; // svv Jan 10 2000 : porting on DEC
            for (i = 1; i <= NbU; i++)
            {
                myPatches!.Value(i, 1).Bounds(out U1, out U2, out V1, out V2);
                if (i == 1)
                {
                    myUJointValues.SetValue(1, U = U1);
                }
                U += (U2 - U1);
                myUJointValues.SetValue(i + 1, U);
            }
            for (i = 1; i <= NbV; i++)
            {
                myPatches!.Value(1, i).Bounds(out U1, out U2, out V1, out V2);
                if (i == 1)
                {
                    myVJointValues.SetValue(1, V = V1);
                }
                V += (V2 - V1);
                myVJointValues.SetValue(i + 1, V);
            }
        }
        else
        {
            double stepu = 1.0, stepv = 1.0; // suppose param == ShapeExtend_Uniform
            if (param == ShapeExtend_Unitary)
            {
                stepu /= NbU;
                stepv /= NbV;
            }
            int i; // svv Jan 10 2000 : porting on DEC
            for (i = 0; i <= NbU; i++)
            {
                myUJointValues.SetValue(i + 1, i * stepu);
            }
            for (i = 0; i <= NbV; i++)
            {
                myVJointValues.SetValue(i + 1, i * stepv);
            }
        }
    }

    private static double LimitValue(double par) =>
        Occt.Precision.IsInfinite(par) ? (par < 0 ? -10000.0 : 10000.0) : par;

    private static void GetLimitedBounds(Geom_Surface surf, out double U1, out double U2, out double V1, out double V2)
    {
        surf.Bounds(out U1, out U2, out V1, out V2);
        U1 = LimitValue(U1);
        U2 = LimitValue(U2);
        V1 = LimitValue(V1);
        V2 = LimitValue(V2);
    }

    /// <summary>
    /// Checks the geometric connectivity of the patches, with the tolerance <paramref name="Prec"/>, sampling 23
    /// points along each boundary, and sets the closure flags. True if every inner boundary is connected.
    /// </summary>
    internal bool CheckConnectivity(double Prec)
    {
        CheckMutable();
        const int NPOINTS = 23;
        bool ok = true;
        int NbU = NbUPatches();
        int NbV = NbVPatches();

        // check in u direction
        int i, j; // svv Jan 10 2000 : porting on DEC
        for (i = 1, j = NbU; i <= NbU; j = i++)
        {
            double maxdist2 = 0.0;
            for (int k = 1; k <= NbV; k++)
            {
                Geom_Surface sj = myPatches!.Value(j, k);
                Geom_Surface si = myPatches.Value(i, k);
                GetLimitedBounds(sj, out _, out double Uj2, out double Vj1, out double Vj2);
                GetLimitedBounds(si, out double Ui1, out _, out double Vi1, out double Vi2);
                double stepj = (Vj2 - Vj1) / (NPOINTS - 1);
                double stepi = (Vi2 - Vi1) / (NPOINTS - 1);
                for (int isample = 0; isample < NPOINTS; isample++)
                {
                    double parj = Vj1 + stepj * isample;
                    double pari = Vi1 + stepi * isample;
                    double dist2 = sj.Value(Uj2, parj).SquareDistance(si.Value(Ui1, pari));
                    if (maxdist2 < dist2)
                    {
                        maxdist2 = dist2;
                    }
                }
            }
            if (i == 1)
            {
                myUClosed = (maxdist2 <= Prec * Prec);
            }
            else if (maxdist2 > Prec * Prec)
            {
                ok = false;
            }
        }

        // check in v direction
        for (i = 1, j = NbV; i <= NbV; j = i++)
        {
            double maxdist2 = 0.0;
            for (int k = 1; k <= NbU; k++)
            {
                Geom_Surface sj = myPatches!.Value(k, j);
                Geom_Surface si = myPatches.Value(k, i);
                GetLimitedBounds(sj, out double Uj1, out double Uj2, out _, out double Vj2);
                GetLimitedBounds(si, out double Ui1, out double Ui2, out double Vi1, out _);
                double stepj = (Uj2 - Uj1) / (NPOINTS - 1);
                double stepi = (Ui2 - Ui1) / (NPOINTS - 1);
                for (int isample = 0; isample < NPOINTS; isample++)
                {
                    double parj = Uj1 + stepj * isample;
                    double pari = Ui1 + stepi * isample;
                    double dist2 = sj.Value(parj, Vj2).SquareDistance(si.Value(pari, Vi1));
                    if (maxdist2 < dist2)
                    {
                        maxdist2 = dist2;
                    }
                }
            }
            if (i == 1)
            {
                myVClosed = (maxdist2 <= Prec * Prec);
            }
            else if (maxdist2 > Prec * Prec)
            {
                ok = false;
            }
        }

        return ok;
    }

    // Freezes a patch handed out (shared, not copied), as Geom_RectangularTrimmedSurface's BasisSurface.
    private static Geom_Surface Shared(Geom_Surface thePatch)
    {
        thePatch.Freeze();
        return thePatch;
    }
}
