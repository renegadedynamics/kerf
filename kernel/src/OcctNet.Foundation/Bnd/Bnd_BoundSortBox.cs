// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_BoundSortBox.hxx
// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_BoundSortBox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Compares a bounding box or a plane with a set of bounding boxes, giving the indices of the boxes it intersects.
/// Initialize it with the set of boxes, then call <see cref="Compare(in Bnd_Box)"/>.
/// </summary>
/// <remarks>
/// The boxes are sorted into a voxel grid over their enclosing box (lbr, 27 Feb 1997): each box is projected on the
/// X, Y and Z axes of the grid, a compared box is tested only against the boxes sharing its voxels on all three
/// axes, and boxes spanning a large part of the grid are kept apart and tested directly.
/// </remarks>
public class Bnd_BoundSortBox
{
    private Bnd_Box myEnclosingBox = new();                  // The bounding box that contains all the boxes to be sorted.
    private NCollection_HArray1<Bnd_Box>? myBoxes;            // The set of bounding boxes to be sorted.
    private double myCoeffX;                                  // Coefficient for X direction.
    private double myCoeffY;                                  // Coefficient for Y direction.
    private double myCoeffZ;                                  // Coefficient for Z direction.
    private int myResolution;                                 // The number of voxels in each direction.
    private readonly NCollection_List<int> myLastResult = new(); // The last result of the Compare() method.
    private readonly NCollection_DynamicArray<int> myLargeBoxes = new(); // The list of large boxes.
    private Bnd_VoxelGrid? myVoxelGrid;                       // The voxel grid used for sorting the boxes.

    /// <summary>Constructs an empty algorithm; define the boxes with one of the Initialize methods.</summary>
    public Bnd_BoundSortBox()
    {
        myBoxes = null;
        myCoeffX = 0.0;
        myCoeffY = 0.0;
        myCoeffZ = 0.0;
        myResolution = 0;
        myVoxelGrid = null;
    }

    /// <summary>Initializes the algorithm with the set of boxes (the array is shared, not copied).</summary>
    public void Initialize(NCollection_HArray1<Bnd_Box> theSetOfBoxes)
    {
        myBoxes = theSetOfBoxes;

        for (int aBoxIndex = myBoxes.Lower(); aBoxIndex <= myBoxes.Upper(); ++aBoxIndex)
        {
            Bnd_Box aBox = myBoxes.Value(aBoxIndex);
            if (!aBox.IsVoid())
            {
                myEnclosingBox.Add(aBox);
            }
        }

        myResolution = getBnd_VoxelGridResolution(myBoxes.Length());

        if (myEnclosingBox.IsVoid())
        {
            return;
        }

        calculateCoefficients();
        resetVoxelGrid();

        sortBoxes();
    }

    /// <summary>Initializes the algorithm with the set of boxes and a box known to enclose them all.</summary>
    public void Initialize(in Bnd_Box theEnclosingBox, NCollection_HArray1<Bnd_Box> theSetOfBoxes)
    {
        myBoxes = theSetOfBoxes;
        myEnclosingBox = theEnclosingBox;
        myResolution = getBnd_VoxelGridResolution(myBoxes.Length());

        if (myEnclosingBox.IsVoid())
        {
            return;
        }

        calculateCoefficients();
        resetVoxelGrid();

        sortBoxes();
    }

    /// <summary>
    /// Initializes the algorithm with the enclosing box and the number of boxes, which are then given one by one
    /// with <see cref="Add"/>.
    /// </summary>
    public void Initialize(in Bnd_Box theEnclosingBox, int theNbBoxes)
    {
        Standard_NullValue_Raise_if(theNbBoxes <= 0, "Unexpected: theNbBoxes <= 0");
        myBoxes = new NCollection_HArray1<Bnd_Box>(1, theNbBoxes);
        //***>>> JCD - 04.08.2000 - Array initialization is missing...
        Bnd_Box emptyBox = new();
        myBoxes.Init(emptyBox);
        //***<<< JCD - End
        myEnclosingBox = theEnclosingBox;
        myResolution = getBnd_VoxelGridResolution(theNbBoxes);

        if (myEnclosingBox.IsVoid())
        {
            return;
        }

        calculateCoefficients();
        resetVoxelGrid();
    }

    /// <summary>
    /// Adds <paramref name="theBox"/> at the 1-based position <paramref name="theIndex"/>. Throws
    /// Standard_MultiplyDefined if a box is already there.
    /// </summary>
    public void Add(in Bnd_Box theBox, int theIndex)
    {
        Standard_MultiplyDefined_Raise_if(!(myBoxes!.Value(theIndex).IsVoid()), " This box is already defined !");
        if (theBox.IsVoid())
        {
            return;
        }

        myBoxes.SetValue(theIndex, theBox);

        addBox(theBox, theIndex);
    }

    /// <summary>
    /// The indices of the boxes that intersect <paramref name="theBox"/> or are inside it. The list is the
    /// algorithm's own and is replaced by the next comparison.
    /// </summary>
    public NCollection_List<int> Compare(in Bnd_Box theBox)
    {
        myLastResult.Clear();

        if (theBox.IsVoid() || theBox.IsOut(myEnclosingBox))
        {
            return myLastResult;
        }

        // Processing the large boxes. These boxes are not voxelized and are checked
        // against the given box directly which is presumably faster.
        foreach (int aBoxIndex in myLargeBoxes)
        {
            Bnd_Box aBox = myBoxes!.Value(aBoxIndex);
            if (!aBox.IsOut(theBox))
            {
                myLastResult.Append(aBoxIndex);
            }
        }

        // Obtaining the box voxel coordinates.
        Span<int> aVoxels = stackalloc int[6];
        getBoundingVoxels(theBox, aVoxels);
        int aMinVoxelX = aVoxels[0], aMinVoxelY = aVoxels[1], aMinVoxelZ = aVoxels[2];
        int aMaxVoxelX = aVoxels[3], aMaxVoxelY = aVoxels[4], aMaxVoxelZ = aVoxels[5];

        // This vector is a structure to store the indices of boxes that occupy the same voxels
        // as the given box. Index of element in vector corresponds to the index of the box in
        // the array of boxes. The value of the element is a bit mask indicating which axes
        // the box occupies. The first bit (0b01) indicates the X-axis, and the second bit (0b10)
        // indicates the Y-axis. If both bits are set (0b11), and the box also occupies the Z-axis,
        // we check for actual intersection with the given box and add it to the result if
        // intersection occurs.
        // (OCCT's NCollection_LinearVector<uint8_t> is a plain array here.)
        byte[] aResultIndices = new byte[myBoxes!.Upper() + 1];
        const byte anOccupiedX = 0b01;
        const byte anOccupiedY = 0b10;
        const byte anOccupiedXY = 0b11;

        // Checking the voxels along X-axis.
        for (int aVoxelX = aMinVoxelX; aVoxelX <= aMaxVoxelX; ++aVoxelX)
        {
            NCollection_DynamicArray<int>? aBoxIndices = myVoxelGrid!.GetSliceX(aVoxelX);
            if (aBoxIndices is null)
            {
                continue;
            }

            foreach (int aBoxIndex in aBoxIndices)
            {
                // Set the first bit (0b01) to indicate that the box occupies the X-axis.
                aResultIndices[aBoxIndex] |= anOccupiedX;
            }
        }

        // Checking the voxels along Y-axis.
        for (int aVoxelY = aMinVoxelY; aVoxelY <= aMaxVoxelY; ++aVoxelY)
        {
            NCollection_DynamicArray<int>? aBoxIndices = myVoxelGrid!.GetSliceY(aVoxelY);
            if (aBoxIndices is null)
            {
                continue;
            }

            foreach (int aBoxIndex in aBoxIndices)
            {
                // Set the second bit (0b10) to indicate that the box occupies the Y-axis.
                aResultIndices[aBoxIndex] |= anOccupiedY;
            }
        }

        // Checking the voxels along Z-axis.
        for (int aVoxelZ = aMinVoxelZ; aVoxelZ <= aMaxVoxelZ; ++aVoxelZ)
        {
            NCollection_DynamicArray<int>? aBoxIndices = myVoxelGrid!.GetSliceZ(aVoxelZ);
            if (aBoxIndices is null)
            {
                continue;
            }

            foreach (int aBoxIndex in aBoxIndices)
            {
                // If box also occupies X and Y axes, we will check for intersection with the given box.
                if (aResultIndices[aBoxIndex] == anOccupiedXY)
                {
                    // Clear the bit mask for this box to avoid checking and adding it multiple times.
                    aResultIndices[aBoxIndex] = 0;
                    // Perform actual intersection check.
                    if (!myBoxes.Value(aBoxIndex).IsOut(theBox))
                    {
                        // Add to the result if intersection occurs.
                        myLastResult.Append(aBoxIndex);
                    }
                }
            }
        }

        return myLastResult;
    }

    /// <summary>
    /// The indices of the boxes that intersect <paramref name="thePlane"/>. The list is the algorithm's own and is
    /// replaced by the next comparison.
    /// </summary>
    public NCollection_List<int> Compare(in gp_Pln thePlane)
    {
        myLastResult.Clear();
        for (int aBoxIndex = myBoxes!.Lower(); aBoxIndex <= myBoxes.Upper(); ++aBoxIndex)
        {
            Bnd_Box aBox = myBoxes.Value(aBoxIndex);
            if (!aBox.IsOut(thePlane))
            {
                myLastResult.Append(aBoxIndex);
            }
        }

        return myLastResult;
    }

    // Precalculates the coefficients mapping box coordinates to the voxel grid.
    private void calculateCoefficients()
    {
        myEnclosingBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);
        myCoeffX = (aXmax - aXmin == 0.0 ? 0.0 : myResolution / (aXmax - aXmin));
        myCoeffY = (aYmax - aYmin == 0.0 ? 0.0 : myResolution / (aYmax - aYmin));
        myCoeffZ = (aZmax - aZmin == 0.0 ? 0.0 : myResolution / (aZmax - aZmin));
    }

    // Resets the voxel grid and clears the list of large boxes.
    private void resetVoxelGrid()
    {
        myVoxelGrid = new Bnd_VoxelGrid(myResolution, myBoxes!.Length());
        myLargeBoxes.Clear();
        // OCCT sets a block size here (myLargeBoxes.SetIncrement), an allocation hint with no visible effect.
    }

    // Adds every box of myBoxes to the voxel grid.
    private void sortBoxes()
    {
        for (int aBoxIndex = myBoxes!.Lower(); aBoxIndex <= myBoxes.Upper(); ++aBoxIndex)
        {
            addBox(myBoxes.Value(aBoxIndex), aBoxIndex);
        }
    }

    // The indices of the voxels holding the box's minimum and maximum points, in the order
    // [minX, minY, minZ, maxX, maxY, maxZ] (std::array<int, 6> in OCCT).
    private void getBoundingVoxels(in Bnd_Box theBox, Span<int> theVoxels)
    {
        // Start point of the voxel grid.
        gp_Pnt aGridStart = myEnclosingBox.CornerMin();

        theBox.Get(out double aXMin, out double aYMin, out double aZMin, out double aXMax, out double aYMax, out double aZmax);

        // Calculate the voxel indices for the bounding box: for one dimension, V = (Vmax / Lmax) * L, where
        // Vmax / Lmax is myCoeff(X|Y|Z) and L the distance from the grid start. One is added and subtracted to make
        // sure that the box is inside the voxel coordinates.
        theVoxels[0] = Math.Clamp((int)((aXMin - aGridStart.X()) * myCoeffX) - 1, 0, myResolution - 1);
        theVoxels[1] = Math.Clamp((int)((aYMin - aGridStart.Y()) * myCoeffY) - 1, 0, myResolution - 1);
        theVoxels[2] = Math.Clamp((int)((aZMin - aGridStart.Z()) * myCoeffZ) - 1, 0, myResolution - 1);
        theVoxels[3] = Math.Clamp((int)((aXMax - aGridStart.X()) * myCoeffX) + 1, 0, myResolution - 1);
        theVoxels[4] = Math.Clamp((int)((aYMax - aGridStart.Y()) * myCoeffY) + 1, 0, myResolution - 1);
        theVoxels[5] = Math.Clamp((int)((aZmax - aGridStart.Z()) * myCoeffZ) + 1, 0, myResolution - 1);
    }

    // Adds the box stored in myBoxes to the voxel map.
    private void addBox(in Bnd_Box theBox, int theIndex)
    {
        if (theBox.IsVoid())
        {
            return;
        }

        Span<int> aVoxels = stackalloc int[6];
        getBoundingVoxels(theBox, aVoxels);
        int aMinVoxelX = aVoxels[0], aMinVoxelY = aVoxels[1], aMinVoxelZ = aVoxels[2];
        int aMaxVoxelX = aVoxels[3], aMaxVoxelY = aVoxels[4], aMaxVoxelZ = aVoxels[5];

        int aBoxMinSide = Min(Min(aMaxVoxelX - aMinVoxelX, aMaxVoxelY - aMinVoxelY), aMaxVoxelZ - aMinVoxelZ);

        // If the box spans a significant portion of the grid,
        // it is more efficient to test it normally rather than using voxel-based checks.
        if (aBoxMinSide * 4 > myResolution)
        {
            myLargeBoxes.Append(theIndex);
        }
        else
        {
            myVoxelGrid!.AddBox(theIndex, aVoxels);
        }
    }

    // The resolution of the voxel grid, from the number of boxes.
    private static int getBnd_VoxelGridResolution(int theBoxesCount)
    {
        if (theBoxesCount > 40000)
        {
            return 128;
        }

        if (theBoxesCount > 10000)
        {
            return 64;
        }

        if (theBoxesCount > 1000)
        {
            return 32;
        }

        if (theBoxesCount > 100)
        {
            return 16;
        }

        return 8;
    }

    /// <summary>
    /// A 3D voxel grid, stored as three arrays of slices, each holding the indices of the boxes that occupy it.
    /// A Standard_Transient defined in Bnd_BoundSortBox.cxx in OCCT; its incremental allocator is dropped.
    /// </summary>
    private sealed class Bnd_VoxelGrid
    {
        private readonly NCollection_Array1<NCollection_DynamicArray<int>> mySlicesX; // box indices for each X slice
        private readonly NCollection_Array1<NCollection_DynamicArray<int>> mySlicesY; // box indices for each Y slice
        private readonly NCollection_Array1<NCollection_DynamicArray<int>> mySlicesZ; // box indices for each Z slice

        public Bnd_VoxelGrid(int theResolution, int theExpectedBoxCount)
        {
            mySlicesX = new NCollection_Array1<NCollection_DynamicArray<int>>(0, theResolution - 1);
            mySlicesY = new NCollection_Array1<NCollection_DynamicArray<int>>(0, theResolution - 1);
            mySlicesZ = new NCollection_Array1<NCollection_DynamicArray<int>>(0, theResolution - 1);
            // Estimate boxes per slice: total boxes / resolution, with minimum of 16.
            int anIncrement = Max(theExpectedBoxCount / theResolution, 16);
            for (int i = 0; i < theResolution; ++i)
            {
                mySlicesX[i] = new NCollection_DynamicArray<int>(anIncrement);
                mySlicesY[i] = new NCollection_DynamicArray<int>(anIncrement);
                mySlicesZ[i] = new NCollection_DynamicArray<int>(anIncrement);
            }
        }

        // Adds a box, given its voxel coordinates [minX, minY, minZ, maxX, maxY, maxZ].
        public void AddBox(int theBoxIndex, ReadOnlySpan<int> theVoxelBox)
        {
            int aMinVoxelX = theVoxelBox[0];
            int aMinVoxelY = theVoxelBox[1];
            int aMinVoxelZ = theVoxelBox[2];
            int aMaxVoxelX = theVoxelBox[3];
            int aMaxVoxelY = theVoxelBox[4];
            int aMaxVoxelZ = theVoxelBox[5];

            AppendSliceX(aMinVoxelX, aMaxVoxelX, theBoxIndex);
            AppendSliceY(aMinVoxelY, aMaxVoxelY, theBoxIndex);
            AppendSliceZ(aMinVoxelZ, aMaxVoxelZ, theBoxIndex);
        }

        // The indices of the boxes occupying the X slice, or null if none.
        public NCollection_DynamicArray<int>? GetSliceX(int theVoxelIndex)
        {
            NCollection_DynamicArray<int> aSlice = mySlicesX[theVoxelIndex];
            return aSlice.IsEmpty() ? null : aSlice;
        }

        // The indices of the boxes occupying the Y slice, or null if none.
        public NCollection_DynamicArray<int>? GetSliceY(int theVoxelIndex)
        {
            NCollection_DynamicArray<int> aSlice = mySlicesY[theVoxelIndex];
            return aSlice.IsEmpty() ? null : aSlice;
        }

        // The indices of the boxes occupying the Z slice, or null if none.
        public NCollection_DynamicArray<int>? GetSliceZ(int theVoxelIndex)
        {
            NCollection_DynamicArray<int> aSlice = mySlicesZ[theVoxelIndex];
            return aSlice.IsEmpty() ? null : aSlice;
        }

        private void AppendSliceX(int theVoxelIndexMin, int theVoxelIndexMax, int theBoxIndex)
        {
            for (int i = theVoxelIndexMin; i <= theVoxelIndexMax; ++i)
            {
                mySlicesX[i].Append(theBoxIndex);
            }
        }

        private void AppendSliceY(int theVoxelIndexMin, int theVoxelIndexMax, int theBoxIndex)
        {
            for (int i = theVoxelIndexMin; i <= theVoxelIndexMax; ++i)
            {
                mySlicesY[i].Append(theBoxIndex);
            }
        }

        private void AppendSliceZ(int theVoxelIndexMin, int theVoxelIndexMax, int theBoxIndex)
        {
            for (int i = theVoxelIndexMin; i <= theVoxelIndexMax; ++i)
            {
                mySlicesZ[i].Append(theBoxIndex);
            }
        }
    }
}
