// Port of: src/FoundationClasses/TKMath/Convert/Convert_ElementarySurfaceToBSplineSurface.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_ElementarySurfaceToBSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Root class for the algorithms converting an elementary surface (gp_Cone, gp_Cylinder, gp_Sphere, gp_Torus) into
/// a B-spline surface: the degrees, periodicity, poles with weights, and knots with multiplicities in each
/// parametric direction, computed by the constructor of the derived class.
/// </summary>
/// <remarks>
/// The batch accessors (<see cref="Poles"/>, <see cref="Weights"/>, <see cref="UKnots"/>, ...) return copies, where
/// OCCT returns references to its fields: the converter's result can't be changed through them. The per-index
/// accessors (<see cref="Pole"/>, ...) are deprecated in OCCT in favour of the batch accessors.
/// </remarks>
public class Convert_ElementarySurfaceToBSplineSurface
{
    protected NCollection_Array2<gp_Pnt> myPoles;
    protected NCollection_Array2<double> myWeights;
    protected NCollection_Array1<double> myUKnots;
    protected NCollection_Array1<double> myVKnots;
    protected NCollection_Array1<int> myUMults;
    protected NCollection_Array1<int> myVMults;
    protected int myUDegree = 0;
    protected int myVDegree = 0;
    protected bool myIsUPeriodic = false;
    protected bool myIsVPeriodic = false;
    protected int myNbUPoles;
    protected int myNbVPoles;
    protected int myNbUKnots;
    protected int myNbVKnots;

    /// <summary>
    /// Trims the arrays, sized for the largest case, down to the actual numbers of poles and knots. OCCT's
    /// <c>Finalize</c>, renamed because <c>Finalize()</c> is .NET's finaliser.
    /// </summary>
    protected void FinalizeArrays()
    {
        // Trim oversized arrays down to actual sizes, preserving 2D element positions.
        if (myPoles.NbRows() != myNbUPoles || myPoles.NbColumns() != myNbVPoles)
        {
            myPoles.ResizeWithTrim(1, myNbUPoles, 1, myNbVPoles, true);
        }
        if (myWeights.NbRows() != myNbUPoles || myWeights.NbColumns() != myNbVPoles)
        {
            myWeights.ResizeWithTrim(1, myNbUPoles, 1, myNbVPoles, true);
        }
        if (myUKnots.Length() != myNbUKnots)
        {
            myUKnots.Resize(1, myNbUKnots, true);
        }
        if (myUMults.Length() != myNbUKnots)
        {
            myUMults.Resize(1, myNbUKnots, true);
        }
        if (myVKnots.Length() != myNbVKnots)
        {
            myVKnots.Resize(1, myNbVKnots, true);
        }
        if (myVMults.Length() != myNbVKnots)
        {
            myVMults.Resize(1, myNbVKnots, true);
        }
    }

    protected Convert_ElementarySurfaceToBSplineSurface(int theNbUPoles,
                                                        int theNbVPoles,
                                                        int theNbUKnots,
                                                        int theNbVKnots,
                                                        int theUDegree,
                                                        int theVDegree)
    {
        myPoles = new NCollection_Array2<gp_Pnt>(1, theNbUPoles, 1, theNbVPoles);
        myWeights = new NCollection_Array2<double>(1, theNbUPoles, 1, theNbVPoles);
        myUKnots = new NCollection_Array1<double>(1, theNbUKnots);
        myVKnots = new NCollection_Array1<double>(1, theNbVKnots);
        myUMults = new NCollection_Array1<int>(1, theNbUKnots);
        myVMults = new NCollection_Array1<int>(1, theNbVKnots);
        myUDegree = theUDegree;
        myVDegree = theVDegree;
        myNbUPoles = theNbUPoles;
        myNbVPoles = theNbVPoles;
        myNbUKnots = theNbUKnots;
        myNbVKnots = theNbVKnots;
    }

    /// <summary>The degree in the U direction.</summary>
    public int UDegree()
    {
        return myUDegree;
    }

    /// <summary>The degree in the V direction.</summary>
    public int VDegree()
    {
        return myVDegree;
    }

    /// <summary>The number of poles in the U direction.</summary>
    public int NbUPoles()
    {
        return myNbUPoles;
    }

    /// <summary>The number of poles in the V direction.</summary>
    public int NbVPoles()
    {
        return myNbVPoles;
    }

    /// <summary>The number of knots in the U direction.</summary>
    public int NbUKnots()
    {
        return myNbUKnots;
    }

    /// <summary>The number of knots in the V direction.</summary>
    public int NbVKnots()
    {
        return myNbVKnots;
    }

    /// <summary>True if the surface is periodic in U.</summary>
    public bool IsUPeriodic()
    {
        return myIsUPeriodic;
    }

    /// <summary>True if the surface is periodic in V.</summary>
    public bool IsVPeriodic()
    {
        return myIsVPeriodic;
    }

    /// <summary>The pole (<paramref name="UIndex"/>, <paramref name="VIndex"/>), from 1. Deprecated in OCCT: use <see cref="Poles"/>.</summary>
    public gp_Pnt Pole(int UIndex, int VIndex)
    {
        Standard_OutOfRange_Raise_if(
          UIndex < 1 || UIndex > myNbUPoles || VIndex < 1 || VIndex > myNbVPoles,
          "Convert_ElementarySurfaceToBSplineSurface::Pole: Index out of range");
        return myPoles[UIndex, VIndex];
    }

    /// <summary>The weight of the pole (<paramref name="UIndex"/>, <paramref name="VIndex"/>). Deprecated in OCCT: use <see cref="Weights"/>.</summary>
    public double Weight(int UIndex, int VIndex)
    {
        Standard_OutOfRange_Raise_if(
          UIndex < 1 || UIndex > myNbUPoles || VIndex < 1 || VIndex > myNbVPoles,
          "Convert_ElementarySurfaceToBSplineSurface::Weight: Index out of range");
        return myWeights[UIndex, VIndex];
    }

    /// <summary>The U knot of index <paramref name="UIndex"/>. Deprecated in OCCT: use <see cref="UKnots"/>.</summary>
    public double UKnot(int UIndex)
    {
        Standard_OutOfRange_Raise_if(
          UIndex < 1 || UIndex > myNbUKnots,
          "Convert_ElementarySurfaceToBSplineSurface::UKnot: Index out of range");
        return myUKnots[UIndex];
    }

    /// <summary>The V knot of index <paramref name="VIndex"/>. Deprecated in OCCT: use <see cref="VKnots"/>.</summary>
    public double VKnot(int VIndex)
    {
        Standard_OutOfRange_Raise_if(
          VIndex < 1 || VIndex > myNbVKnots,
          "Convert_ElementarySurfaceToBSplineSurface::VKnot: Index out of range");
        return myVKnots[VIndex];
    }

    /// <summary>The multiplicity of the U knot of index <paramref name="UIndex"/>. Deprecated in OCCT: use <see cref="UMultiplicities"/>.</summary>
    public int UMultiplicity(int UIndex)
    {
        Standard_OutOfRange_Raise_if(
          UIndex < 1 || UIndex > myNbUKnots,
          "Convert_ElementarySurfaceToBSplineSurface::UMultiplicity: Index out of range");
        return myUMults[UIndex];
    }

    /// <summary>The multiplicity of the V knot of index <paramref name="VIndex"/>. Deprecated in OCCT: use <see cref="VMultiplicities"/>.</summary>
    public int VMultiplicity(int VIndex)
    {
        Standard_OutOfRange_Raise_if(
          VIndex < 1 || VIndex > myNbVKnots,
          "Convert_ElementarySurfaceToBSplineSurface::VMultiplicity: Index out of range");
        return myVMults[VIndex];
    }

    /// <summary>The poles: a copy.</summary>
    public NCollection_Array2<gp_Pnt> Poles()
    {
        return new NCollection_Array2<gp_Pnt>(myPoles);
    }

    /// <summary>The weights: a copy.</summary>
    public NCollection_Array2<double> Weights()
    {
        return new NCollection_Array2<double>(myWeights);
    }

    /// <summary>The U knots: a copy.</summary>
    public NCollection_Array1<double> UKnots()
    {
        return new NCollection_Array1<double>(myUKnots);
    }

    /// <summary>The V knots: a copy.</summary>
    public NCollection_Array1<double> VKnots()
    {
        return new NCollection_Array1<double>(myVKnots);
    }

    /// <summary>The multiplicities of the U knots: a copy.</summary>
    public NCollection_Array1<int> UMultiplicities()
    {
        return new NCollection_Array1<int>(myUMults);
    }

    /// <summary>The multiplicities of the V knots: a copy.</summary>
    public NCollection_Array1<int> VMultiplicities()
    {
        return new NCollection_Array1<int>(myVMults);
    }
}
