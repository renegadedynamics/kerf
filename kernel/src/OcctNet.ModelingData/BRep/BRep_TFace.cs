// Port of: src/ModelingData/TKBRep/BRep/BRep_TFace.hxx
// Port of: src/ModelingData/TKBRep/BRep/BRep_TFace.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using static Occt.Poly_MeshPurpose;

namespace Occt;

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
/// The setters are internal to OCCT.NET and throw <see cref="Standard_ImmutableObject"/> once the face is frozen. The
/// surface and the triangulations are frozen as soon as the face takes them (docs/phase3-design.md, section 4.1).
/// OCCT marks the active triangulation with the Poly_MeshPurpose_Active bit of its purpose; the port leaves the
/// purpose of a frozen triangulation as it is (docs/decisions.md, 3106).
/// </para>
/// </remarks>
public class BRep_TFace : TopoDS_TFace
{
    private readonly NCollection_List<Poly_Triangulation> myTriangulations = new();
    private Poly_Triangulation? myActiveTriangulation;
    private Geom_Surface? mySurface;
    private TopLoc_Location myLocation;
    private double myTolerance;
    private bool myNaturalRestriction;

    /// <summary>An empty face: no surface, tolerance RealEpsilon().</summary>
    public BRep_TFace()
    {
        myLocation = new TopLoc_Location();
        myTolerance = RealEpsilon();
        myNaturalRestriction = false;
    }

    /// <summary>The face's surface.</summary>
    public Geom_Surface? Surface() => mySurface;

    internal void Surface(Geom_Surface? theSurface)
    {
        CheckMutable();
        theSurface?.Freeze(); // OCCT.NET: attached geometry is frozen
        mySurface = theSurface;
    }

    /// <summary>The location of the surface.</summary>
    public TopLoc_Location Location() => myLocation;

    internal void Location(in TopLoc_Location theLocation)
    {
        CheckMutable();
        myLocation = theLocation;
    }

    /// <summary>The face's tolerance.</summary>
    public double Tolerance() => myTolerance;

    internal void Tolerance(double theTolerance)
    {
        CheckMutable();
        myTolerance = theTolerance;
    }

    /// <summary>True if the boundary of the face is known to be the parametric space (UMin, UMax, VMin, VMax).</summary>
    public bool NaturalRestriction() => myNaturalRestriction;

    internal void NaturalRestriction(bool theRestriction)
    {
        CheckMutable();
        myNaturalRestriction = theRestriction;
    }

    /// <summary>
    /// The triangulation for the purpose: the active one for NONE, else the first one with a matching purpose bit, else
    /// the first one if the purpose includes AnyFallback, else null.
    /// </summary>
    public Poly_Triangulation? Triangulation(Poly_MeshPurpose thePurpose = Poly_MeshPurpose_NONE)
    {
        if (thePurpose == Poly_MeshPurpose_NONE)
        {
            return ActiveTriangulation();
        }

        for (var anIter = new NCollection_List<Poly_Triangulation>.Iterator(myTriangulations); anIter.More(); anIter.Next())
        {
            Poly_Triangulation aTriangulation = anIter.Value();
            if ((aTriangulation.MeshPurpose() & thePurpose) != 0)
            {
                return aTriangulation;
            }
        }

        if ((thePurpose & Poly_MeshPurpose_AnyFallback) != 0 && !myTriangulations.IsEmpty())
        {
            // if none matching other criteria was found return the first defined triangulation
            return myTriangulations.First();
        }

        return null;
    }

    /// <summary>
    /// Sets the triangulation. With <paramref name="theToReset"/> (or a null triangulation) the list becomes just this
    /// one, active (empty if null). Otherwise the triangulation becomes active if it is in the list, or replaces the
    /// active one.
    /// </summary>
    /// <param name="theTriangulation">The triangulation, or null to remove them all.</param>
    /// <param name="theToReset">Whether the list becomes just this triangulation.</param>
    /// <param name="theToFreeze">
    /// False for the mesher's non-freezing attach (decision 4003): the triangulation freezes with the face's graph.
    /// </param>
    internal void Triangulation(Poly_Triangulation? theTriangulation, bool theToReset = true, bool theToFreeze = true)
    {
        CheckMutable();
        if (theToReset || theTriangulation is null)
        {
            if (myActiveTriangulation is not null)
            {
                // Reset Active bit
                setActiveBit(myActiveTriangulation, false);
                myActiveTriangulation = null;
            }

            myTriangulations.Clear();
            if (theTriangulation is not null)
            {
                // Reset list of triangulations to new list with only one input triangulation that will be active
                myTriangulations.Append(theTriangulation);
                myActiveTriangulation = theTriangulation;
                // Set Active bit
                setActiveBit(theTriangulation, true);
                if (theToFreeze)
                {
                    theTriangulation.Freeze(); // OCCT.NET: attached meshes are frozen
                }
            }

            return;
        }

        for (var anIter = new NCollection_List<Poly_Triangulation>.Iterator(myTriangulations); anIter.More(); anIter.Next())
        {
            // Make input triangulation active if it is already contained in list of triangulations
            if (ReferenceEquals(anIter.Value(), theTriangulation))
            {
                if (myActiveTriangulation is not null)
                {
                    // Reset Active bit
                    setActiveBit(myActiveTriangulation, false);
                }

                myActiveTriangulation = theTriangulation;
                // Set Active bit
                setActiveBit(theTriangulation, true);
                return;
            }
        }

        for (var anIter = new NCollection_List<Poly_Triangulation>.Iterator(myTriangulations); anIter.More(); anIter.Next())
        {
            // Replace active triangulation to input one
            if (ReferenceEquals(anIter.Value(), myActiveTriangulation))
            {
                // Reset Active bit
                setActiveBit(myActiveTriangulation!, false);
                anIter.ChangeValue() = theTriangulation;
                myActiveTriangulation = theTriangulation;
                // Set Active bit
                setActiveBit(theTriangulation, true);
                if (theToFreeze)
                {
                    theTriangulation.Freeze(); // OCCT.NET: attached meshes are frozen
                }
                return;
            }
        }
    }

    /// <summary>A new face with the same surface, location and tolerance, without triangulations and sub-shapes.</summary>
    public override TopoDS_TShape EmptyCopy()
    {
        BRep_TFace TF = new BRep_TFace();
        TF.Surface(mySurface);
        TF.Location(myLocation);
        TF.Tolerance(myTolerance);
        return TF;
    }

    /// <summary>The face's triangulations (internal: ported code reads the list itself).</summary>
    internal NCollection_List<Poly_Triangulation> Triangulations() => myTriangulations;

    /// <summary>The face's triangulations, as a new read-only list (OCCT.NET addition).</summary>
    public IBentoList<Poly_Triangulation> TriangulationList() => PackedList.Own([.. myTriangulations]);

    /// <summary>
    /// Sets the list of triangulations and the active one (the first if null). An empty list clears them.
    /// </summary>
    /// <exception cref="Standard_ProgramError">The list holds a null triangulation, or doesn't hold the active one.</exception>
    internal void Triangulations(NCollection_List<Poly_Triangulation> theTriangulations,
                                 Poly_Triangulation? theActiveTriangulation)
    {
        CheckMutable();
        if (theTriangulations.IsEmpty())
        {
            myActiveTriangulation = null;
            myTriangulations.Clear();
            return;
        }

        bool anActiveInList = false;
        for (var anIter = new NCollection_List<Poly_Triangulation>.Iterator(theTriangulations); anIter.More(); anIter.Next())
        {
            Poly_Triangulation aTriangulation = anIter.Value();
            Standard_ProgramError_Raise_if(aTriangulation is null, "Try to set list with NULL triangulation to the face");
            if (ReferenceEquals(aTriangulation, theActiveTriangulation))
            {
                anActiveInList = true;
            }

            // Reset Active bit
            setActiveBit(aTriangulation!, false);
        }

        Standard_ProgramError_Raise_if(theActiveTriangulation is not null && !anActiveInList,
                                       "Active triangulation isn't part of triangulations list");
        myTriangulations.Assign(theTriangulations);
        if (theActiveTriangulation is null)
        {
            // Save the first one as active
            myActiveTriangulation = myTriangulations.First();
        }
        else
        {
            myActiveTriangulation = theActiveTriangulation;
        }

        setActiveBit(myActiveTriangulation, true);
        foreach (Poly_Triangulation aTriangulation in myTriangulations)
        {
            aTriangulation.Freeze(); // OCCT.NET: attached meshes are frozen
        }
    }

    /// <summary>The number of triangulations.</summary>
    public int NbTriangulations() => myTriangulations.Extent();

    /// <summary>The active triangulation, or null.</summary>
    public Poly_Triangulation? ActiveTriangulation() => myActiveTriangulation;

    internal override void FreezeRepresentations()
    {
        mySurface?.Freeze();
        foreach (Poly_Triangulation aTriangulation in myTriangulations)
        {
            aTriangulation.Freeze();
        }
    }

    // OCCT.NET: frozen: OCCT sets and clears the Active bit of the purpose of attached triangulations; a frozen
    // triangulation (attached before, possibly shared by another face) keeps its purpose (docs/decisions.md, 3106).
    private static void setActiveBit(Poly_Triangulation theTriangulation, bool theIsActive)
    {
        if (theTriangulation.IsFrozen())
        {
            return;
        }

        theTriangulation.SetMeshPurpose(theIsActive
                                            ? theTriangulation.MeshPurpose() | Poly_MeshPurpose_Active
                                            : theTriangulation.MeshPurpose() & ~Poly_MeshPurpose_Active);
    }
}
