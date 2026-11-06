// Port of: src/ModelingData/TKBRep/BRep/BRep_TFace.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_TFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Meshing.MeshPurpose;

namespace Kerf.Engine.Topology.Brep;

/// <summary>
/// The geometry of a face: a surface, a location for it, a tolerance, the NaturalRestriction flag (the boundary is the
/// surface's parametric bounds) and an optional list of triangulations, one of them active. With triangulations the
/// surface can be absent.
/// </summary>
/// <remarks>
/// <para>
/// The location applies to the surface, not to the triangulations, which are in the face's own coordinate system.
/// </para>
/// <para>
/// The setters are internal to Kerf and throw <see cref="ImmutableObjectException"/> once the face is frozen, unless
/// a working overlay is open: then they write into its working copy of the face (<see cref="WorkingOverlay"/>). The
/// surface and the triangulations are frozen as soon as the face takes them (docs/phase3-design.md, section 4.1).
/// OCCT marks the active triangulation with the Poly_MeshPurpose_Active bit of its purpose; the port leaves the
/// purpose of a frozen triangulation as it is (docs/decisions.md, 3106).
/// </para>
/// </remarks>
internal class BrepTFace : TFace
{
    private readonly NCollection_List<Triangulation> _triangulations = new();
    private Triangulation? _activeTriangulation;
    private Surface? _surface;
    private Placement _location;
    private double _tolerance;
    private bool _naturalRestriction;

    /// <summary>An empty face: no surface, tolerance RealEpsilon().</summary>
    public BrepTFace()
    {
        _location = new Placement();
        _tolerance = RealEpsilon();
        _naturalRestriction = false;
    }

    /// <summary>The face's surface.</summary>
    public Surface? Surface() => View()._surface;

    internal void Surface(Surface? surface)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            ((BrepTFace)working).Surface(surface);
            return;
        }

        CheckMutable();
        surface?.Freeze(); // Kerf: attached geometry is frozen
        _surface = surface;
    }

    /// <summary>The location of the surface.</summary>
    public Placement Location() => View()._location;

    internal void Location(in Placement location)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            ((BrepTFace)working).Location(location);
            return;
        }

        CheckMutable();
        _location = location;
    }

    /// <summary>The face's tolerance.</summary>
    public double Tolerance() => View()._tolerance;

    internal void Tolerance(double tolerance)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            ((BrepTFace)working).Tolerance(tolerance);
            return;
        }

        CheckMutable();
        _tolerance = tolerance;
    }

    /// <summary>True if the boundary of the face is known to be the parametric space (UMin, UMax, VMin, VMax).</summary>
    public bool NaturalRestriction() => View()._naturalRestriction;

    internal void NaturalRestriction(bool restriction)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            ((BrepTFace)working).NaturalRestriction(restriction);
            return;
        }

        CheckMutable();
        _naturalRestriction = restriction;
    }

    /// <summary>
    /// The triangulation for the purpose: the active one for NONE, else the first one with a matching purpose bit, else
    /// the first one if the purpose includes AnyFallback, else null.
    /// </summary>
    public Triangulation? Triangulation(MeshPurpose purpose = MeshPurpose_NONE)
    {
        if (WorkingOverlay.TryView(this, out TShape? view))
        {
            return ((BrepTFace)view).Triangulation(purpose);
        }

        if (purpose == MeshPurpose_NONE)
        {
            return ActiveTriangulation();
        }

        for (var iter = new NCollection_List<Triangulation>.Iterator(_triangulations); iter.More(); iter.Next())
        {
            Triangulation triangulation = iter.Value();
            if ((triangulation.MeshPurpose() & purpose) != 0)
            {
                return triangulation;
            }
        }

        if ((purpose & MeshPurpose_AnyFallback) != 0 && !_triangulations.IsEmpty())
        {
            // if none matching other criteria was found return the first defined triangulation
            return _triangulations.First();
        }

        return null;
    }

    /// <summary>
    /// Sets the triangulation. With <paramref name="toReset"/> (or a null triangulation) the list becomes just this
    /// one, active (empty if null). Otherwise the triangulation becomes active if it is in the list, or replaces the
    /// active one.
    /// </summary>
    /// <param name="triangulation">The triangulation, or null to remove them all.</param>
    /// <param name="toReset">Whether the list becomes just this triangulation.</param>
    /// <param name="toFreeze">
    /// False for the mesher's non-freezing attach (decision 488): the triangulation freezes with the face's graph.
    /// </param>
    internal void Triangulation(Triangulation? triangulation, bool toReset = true, bool toFreeze = true)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            ((BrepTFace)working).Triangulation(triangulation, toReset, toFreeze);
            return;
        }

        CheckMutable();
        if (toReset || triangulation is null)
        {
            if (_activeTriangulation is not null)
            {
                // Reset Active bit
                SetActiveBit(_activeTriangulation, false);
                _activeTriangulation = null;
            }

            _triangulations.Clear();
            if (triangulation is not null)
            {
                // Reset list of triangulations to new list with only one input triangulation that will be active
                _triangulations.Append(triangulation);
                _activeTriangulation = triangulation;
                // Set Active bit
                SetActiveBit(triangulation, true);
                if (toFreeze)
                {
                    triangulation.Freeze(); // Kerf: attached meshes are frozen
                }
            }

            return;
        }

        for (var iter = new NCollection_List<Triangulation>.Iterator(_triangulations); iter.More(); iter.Next())
        {
            // Make input triangulation active if it is already contained in list of triangulations
            if (ReferenceEquals(iter.Value(), triangulation))
            {
                if (_activeTriangulation is not null)
                {
                    // Reset Active bit
                    SetActiveBit(_activeTriangulation, false);
                }

                _activeTriangulation = triangulation;
                // Set Active bit
                SetActiveBit(triangulation, true);
                return;
            }
        }

        for (var iter = new NCollection_List<Triangulation>.Iterator(_triangulations); iter.More(); iter.Next())
        {
            // Replace active triangulation to input one
            if (ReferenceEquals(iter.Value(), _activeTriangulation))
            {
                // Reset Active bit
                SetActiveBit(_activeTriangulation!, false);
                iter.ChangeValue() = triangulation;
                _activeTriangulation = triangulation;
                // Set Active bit
                SetActiveBit(triangulation, true);
                if (toFreeze)
                {
                    triangulation.Freeze(); // Kerf: attached meshes are frozen
                }
                return;
            }
        }
    }

    /// <summary>A new face with the same surface, location and tolerance, without triangulations and sub-shapes.</summary>
    public override TShape EmptyCopy()
    {
        BrepTFace view = View();
        BrepTFace tf = new BrepTFace();
        tf.Surface(view._surface);
        tf.Location(view._location);
        tf.Tolerance(view._tolerance);
        return tf;
    }

    /// <summary>The face's triangulations (internal: ported code reads the list itself).</summary>
    internal NCollection_List<Triangulation> Triangulations() => View()._triangulations;

    /// <summary>The face's triangulations, as a new read-only list (Kerf addition).</summary>
    public IReadOnlyList<Triangulation> TriangulationList() => ImmutableArrays.Own([.. View()._triangulations]);

    /// <summary>
    /// Sets the list of triangulations and the active one (the first if null). An empty list clears them.
    /// </summary>
    /// <exception cref="ProgramErrorException">The list holds a null triangulation, or doesn't hold the active one.</exception>
    internal void Triangulations(NCollection_List<Triangulation> triangulations,
                                 Triangulation? activeTriangulation)
    {
        if (WorkingOverlay.TryWritable(this, out TShape? working))
        {
            ((BrepTFace)working).Triangulations(triangulations, activeTriangulation);
            return;
        }

        CheckMutable();
        if (triangulations.IsEmpty())
        {
            _activeTriangulation = null;
            _triangulations.Clear();
            return;
        }

        bool activeInList = false;
        for (var iter = new NCollection_List<Triangulation>.Iterator(triangulations); iter.More(); iter.Next())
        {
            Triangulation triangulation = iter.Value();
            Standard_ProgramError_Raise_if(triangulation is null, "Try to set list with NULL triangulation to the face");
            if (ReferenceEquals(triangulation, activeTriangulation))
            {
                activeInList = true;
            }

            // Reset Active bit
            SetActiveBit(triangulation!, false);
        }

        Standard_ProgramError_Raise_if(activeTriangulation is not null && !activeInList,
                                       "Active triangulation isn't part of triangulations list");
        _triangulations.Assign(triangulations);
        if (activeTriangulation is null)
        {
            // Save the first one as active
            _activeTriangulation = _triangulations.First();
        }
        else
        {
            _activeTriangulation = activeTriangulation;
        }

        SetActiveBit(_activeTriangulation, true);
        foreach (Triangulation triangulation in _triangulations)
        {
            triangulation.Freeze(); // Kerf: attached meshes are frozen
        }
    }

    /// <summary>The number of triangulations.</summary>
    public int TriangulationCount => View()._triangulations.Extent();

    /// <summary>The active triangulation, or null.</summary>
    public Triangulation? ActiveTriangulation() => View()._activeTriangulation;

    // The face whose data the current working overlay reads (WorkingOverlay): its working copy, or itself.
    private BrepTFace View() => WorkingOverlay.TryView(this, out TShape? view) ? (BrepTFace)view : this;

    internal override void FreezeRepresentations()
    {
        _surface?.Freeze();
        foreach (Triangulation triangulation in _triangulations)
        {
            triangulation.Freeze();
        }
    }

    // Kerf: frozen: OCCT sets and clears the Active bit of the purpose of attached triangulations; a frozen
    // triangulation (attached before, possibly shared by another face) keeps its purpose (docs/decisions.md, 3106).
    private static void SetActiveBit(Triangulation triangulation, bool isActive)
    {
        if (triangulation.IsFrozen)
        {
            return;
        }

        triangulation.SetMeshPurpose(isActive
                                            ? triangulation.MeshPurpose() | MeshPurpose_Active
                                            : triangulation.MeshPurpose() & ~MeshPurpose_Active);
    }
}
