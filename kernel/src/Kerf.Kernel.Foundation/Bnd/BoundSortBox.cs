// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_BoundSortBox.hxx
// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_BoundSortBox.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Engine;

namespace Kerf.Engine.Geometry.Bounds;

/// <summary>
/// Compares a bounding box or a plane with a set of bounding boxes, giving the indices of the boxes it intersects.
/// Initialize it with the set of boxes, then call <see cref="Compare(in Box)"/>.
/// </summary>
/// <remarks>
/// The boxes are sorted into a voxel grid over their enclosing box (lbr, 27 Feb 1997): each box is projected on the
/// X, Y and Z axes of the grid, a compared box is tested only against the boxes sharing its voxels on all three
/// axes, and boxes spanning a large part of the grid are kept apart and tested directly.
/// </remarks>
internal class BoundSortBox
{
    private Box _enclosingBox = new();                  // The bounding box that contains all the boxes to be sorted.
    private NCollection_HArray1<Box>? _boxes;            // The set of bounding boxes to be sorted.
    private double _coeffX;                                  // Coefficient for X direction.
    private double _coeffY;                                  // Coefficient for Y direction.
    private double _coeffZ;                                  // Coefficient for Z direction.
    private int _resolution;                                 // The number of voxels in each direction.
    private readonly NCollection_List<int> _lastResult = new(); // The last result of the Compare() method.
    private readonly List<int> _largeBoxes = new(); // The list of large boxes.
    private Bnd_VoxelGrid? _voxelGrid;                       // The voxel grid used for sorting the boxes.

    /// <summary>Constructs an empty algorithm; define the boxes with one of the Initialize methods.</summary>
    public BoundSortBox()
    {
        _boxes = null;
        _coeffX = 0.0;
        _coeffY = 0.0;
        _coeffZ = 0.0;
        _resolution = 0;
        _voxelGrid = null;
    }

    /// <summary>Initializes the algorithm with the set of boxes (the array is shared, not copied).</summary>
    public void Initialize(NCollection_HArray1<Box> setOfBoxes)
    {
        _boxes = setOfBoxes;

        for (int boxIndex = _boxes.Lower(); boxIndex <= _boxes.Upper(); ++boxIndex)
        {
            Box box = _boxes.Value(boxIndex);
            if (!box.IsVoid)
            {
                _enclosingBox.Add(box);
            }
        }

        _resolution = GetBndVoxelGridResolution(_boxes.Length());

        if (_enclosingBox.IsVoid)
        {
            return;
        }

        CalculateCoefficients();
        ResetVoxelGrid();

        SortBoxes();
    }

    /// <summary>Initializes the algorithm with the set of boxes and a box known to enclose them all.</summary>
    public void Initialize(in Box enclosingBox, NCollection_HArray1<Box> setOfBoxes)
    {
        _boxes = setOfBoxes;
        _enclosingBox = enclosingBox;
        _resolution = GetBndVoxelGridResolution(_boxes.Length());

        if (_enclosingBox.IsVoid)
        {
            return;
        }

        CalculateCoefficients();
        ResetVoxelGrid();

        SortBoxes();
    }

    /// <summary>
    /// Initializes the algorithm with the enclosing box and the number of boxes, which are then given one by one
    /// with <see cref="Add"/>.
    /// </summary>
    public void Initialize(in Box enclosingBox, int nbBoxes)
    {
        Standard_NullValue_Raise_if(nbBoxes <= 0, "Unexpected: theNbBoxes <= 0");
        _boxes = new NCollection_HArray1<Box>(1, nbBoxes);
        //***>>> JCD - 04.08.2000 - Array initialization is missing...
        Box emptyBox = new();
        _boxes.Init(emptyBox);
        //***<<< JCD - End
        _enclosingBox = enclosingBox;
        _resolution = GetBndVoxelGridResolution(nbBoxes);

        if (_enclosingBox.IsVoid)
        {
            return;
        }

        CalculateCoefficients();
        ResetVoxelGrid();
    }

    /// <summary>
    /// Adds <paramref name="box"/> at the 1-based position <paramref name="index"/>. Throws
    /// Standard_MultiplyDefined if a box is already there.
    /// </summary>
    public void Add(in Box box, int index)
    {
        Standard_MultiplyDefined_Raise_if(!(_boxes!.Value(index).IsVoid), " This box is already defined !");
        if (box.IsVoid)
        {
            return;
        }

        _boxes.SetValue(index, box);

        AddBox(box, index);
    }

    /// <summary>
    /// The indices of the boxes that intersect <paramref name="box"/> or are inside it. The list is the
    /// algorithm's own and is replaced by the next comparison.
    /// </summary>
    public NCollection_List<int> Compare(in Box box)
    {
        _lastResult.Clear();

        if (box.IsVoid || box.IsOut(_enclosingBox))
        {
            return _lastResult;
        }

        // Processing the large boxes. These boxes are not voxelized and are checked
        // against the given box directly which is presumably faster.
        foreach (int boxIndex in _largeBoxes)
        {
            Box aBox = _boxes!.Value(boxIndex);
            if (!aBox.IsOut(box))
            {
                _lastResult.Append(boxIndex);
            }
        }

        // Obtaining the box voxel coordinates.
        Span<int> voxels = stackalloc int[6];
        GetBoundingVoxels(box, voxels);
        int minVoxelX = voxels[0], minVoxelY = voxels[1], minVoxelZ = voxels[2];
        int maxVoxelX = voxels[3], maxVoxelY = voxels[4], maxVoxelZ = voxels[5];

        // This vector is a structure to store the indices of boxes that occupy the same voxels
        // as the given box. Index of element in vector corresponds to the index of the box in
        // the array of boxes. The value of the element is a bit mask indicating which axes
        // the box occupies. The first bit (0b01) indicates the X-axis, and the second bit (0b10)
        // indicates the Y-axis. If both bits are set (0b11), and the box also occupies the Z-axis,
        // we check for actual intersection with the given box and add it to the result if
        // intersection occurs.
        // (OCCT's NCollection_LinearVector<uint8_t> is a plain array here.)
        byte[] resultIndices = new byte[_boxes!.Upper() + 1];
        const byte occupiedX = 0b01;
        const byte occupiedY = 0b10;
        const byte occupiedXY = 0b11;

        // Checking the voxels along X-axis.
        for (int voxelX = minVoxelX; voxelX <= maxVoxelX; ++voxelX)
        {
            List<int>? boxIndices = _voxelGrid!.GetSliceX(voxelX);
            if (boxIndices is null)
            {
                continue;
            }

            foreach (int boxIndex in boxIndices)
            {
                // Set the first bit (0b01) to indicate that the box occupies the X-axis.
                resultIndices[boxIndex] |= occupiedX;
            }
        }

        // Checking the voxels along Y-axis.
        for (int voxelY = minVoxelY; voxelY <= maxVoxelY; ++voxelY)
        {
            List<int>? boxIndices = _voxelGrid!.GetSliceY(voxelY);
            if (boxIndices is null)
            {
                continue;
            }

            foreach (int boxIndex in boxIndices)
            {
                // Set the second bit (0b10) to indicate that the box occupies the Y-axis.
                resultIndices[boxIndex] |= occupiedY;
            }
        }

        // Checking the voxels along Z-axis.
        for (int voxelZ = minVoxelZ; voxelZ <= maxVoxelZ; ++voxelZ)
        {
            List<int>? boxIndices = _voxelGrid!.GetSliceZ(voxelZ);
            if (boxIndices is null)
            {
                continue;
            }

            foreach (int boxIndex in boxIndices)
            {
                // If box also occupies X and Y axes, we will check for intersection with the given box.
                if (resultIndices[boxIndex] == occupiedXY)
                {
                    // Clear the bit mask for this box to avoid checking and adding it multiple times.
                    resultIndices[boxIndex] = 0;
                    // Perform actual intersection check.
                    if (!_boxes.Value(boxIndex).IsOut(box))
                    {
                        // Add to the result if intersection occurs.
                        _lastResult.Append(boxIndex);
                    }
                }
            }
        }

        return _lastResult;
    }

    /// <summary>
    /// The indices of the boxes that intersect <paramref name="plane"/>. The list is the algorithm's own and is
    /// replaced by the next comparison.
    /// </summary>
    public NCollection_List<int> Compare(in Plane3D plane)
    {
        _lastResult.Clear();
        for (int boxIndex = _boxes!.Lower(); boxIndex <= _boxes.Upper(); ++boxIndex)
        {
            Box box = _boxes.Value(boxIndex);
            if (!box.IsOut(plane))
            {
                _lastResult.Append(boxIndex);
            }
        }

        return _lastResult;
    }

    // Precalculates the coefficients mapping box coordinates to the voxel grid.
    private void CalculateCoefficients()
    {
        _enclosingBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
        _coeffX = (xmax - xmin == 0.0 ? 0.0 : _resolution / (xmax - xmin));
        _coeffY = (ymax - ymin == 0.0 ? 0.0 : _resolution / (ymax - ymin));
        _coeffZ = (zmax - zmin == 0.0 ? 0.0 : _resolution / (zmax - zmin));
    }

    // Resets the voxel grid and clears the list of large boxes.
    private void ResetVoxelGrid()
    {
        _voxelGrid = new Bnd_VoxelGrid(_resolution, _boxes!.Length());
        _largeBoxes.Clear();
        // OCCT sets a block size here (myLargeBoxes.SetIncrement), an allocation hint with no visible effect.
    }

    // Adds every box of myBoxes to the voxel grid.
    private void SortBoxes()
    {
        for (int boxIndex = _boxes!.Lower(); boxIndex <= _boxes.Upper(); ++boxIndex)
        {
            AddBox(_boxes.Value(boxIndex), boxIndex);
        }
    }

    // The indices of the voxels holding the box's minimum and maximum points, in the order
    // [minX, minY, minZ, maxX, maxY, maxZ] (std::array<int, 6> in OCCT).
    private void GetBoundingVoxels(in Box box, Span<int> voxels)
    {
        // Start point of the voxel grid.
        Point3D gridStart = _enclosingBox.CornerMin();

        box.Get(out double xMin, out double yMin, out double zMin, out double xMax, out double yMax, out double zmax);

        // Calculate the voxel indices for the bounding box: for one dimension, V = (Vmax / Lmax) * L, where
        // Vmax / Lmax is myCoeff(X|Y|Z) and L the distance from the grid start. One is added and subtracted to make
        // sure that the box is inside the voxel coordinates.
        voxels[0] = Math.Clamp((int)((xMin - gridStart.X) * _coeffX) - 1, 0, _resolution - 1);
        voxels[1] = Math.Clamp((int)((yMin - gridStart.Y) * _coeffY) - 1, 0, _resolution - 1);
        voxels[2] = Math.Clamp((int)((zMin - gridStart.Z) * _coeffZ) - 1, 0, _resolution - 1);
        voxels[3] = Math.Clamp((int)((xMax - gridStart.X) * _coeffX) + 1, 0, _resolution - 1);
        voxels[4] = Math.Clamp((int)((yMax - gridStart.Y) * _coeffY) + 1, 0, _resolution - 1);
        voxels[5] = Math.Clamp((int)((zmax - gridStart.Z) * _coeffZ) + 1, 0, _resolution - 1);
    }

    // Adds the box stored in myBoxes to the voxel map.
    private void AddBox(in Box box, int index)
    {
        if (box.IsVoid)
        {
            return;
        }

        Span<int> voxels = stackalloc int[6];
        GetBoundingVoxels(box, voxels);
        int minVoxelX = voxels[0], minVoxelY = voxels[1], minVoxelZ = voxels[2];
        int maxVoxelX = voxels[3], maxVoxelY = voxels[4], maxVoxelZ = voxels[5];

        int boxMinSide = Min(Min(maxVoxelX - minVoxelX, maxVoxelY - minVoxelY), maxVoxelZ - minVoxelZ);

        // If the box spans a significant portion of the grid,
        // it is more efficient to test it normally rather than using voxel-based checks.
        if (boxMinSide * 4 > _resolution)
        {
            _largeBoxes.Add(index);
        }
        else
        {
            _voxelGrid!.AddBox(index, voxels);
        }
    }

    // The resolution of the voxel grid, from the number of boxes.
    private static int GetBndVoxelGridResolution(int boxesCount)
    {
        if (boxesCount > 40000)
        {
            return 128;
        }

        if (boxesCount > 10000)
        {
            return 64;
        }

        if (boxesCount > 1000)
        {
            return 32;
        }

        if (boxesCount > 100)
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
        private readonly NCollection_Array1<List<int>> _slicesX; // box indices for each X slice
        private readonly NCollection_Array1<List<int>> _slicesY; // box indices for each Y slice
        private readonly NCollection_Array1<List<int>> _slicesZ; // box indices for each Z slice

        public Bnd_VoxelGrid(int resolution, int expectedBoxCount)
        {
            _slicesX = new NCollection_Array1<List<int>>(0, resolution - 1);
            _slicesY = new NCollection_Array1<List<int>>(0, resolution - 1);
            _slicesZ = new NCollection_Array1<List<int>>(0, resolution - 1);
            // Estimate boxes per slice: total boxes / resolution, with minimum of 16.
            int increment = Max(expectedBoxCount / resolution, 16);
            for (int i = 0; i < resolution; ++i)
            {
                _slicesX[i] = new List<int>(increment);
                _slicesY[i] = new List<int>(increment);
                _slicesZ[i] = new List<int>(increment);
            }
        }

        // Adds a box, given its voxel coordinates [minX, minY, minZ, maxX, maxY, maxZ].
        public void AddBox(int boxIndex, ReadOnlySpan<int> voxelBox)
        {
            int minVoxelX = voxelBox[0];
            int minVoxelY = voxelBox[1];
            int minVoxelZ = voxelBox[2];
            int maxVoxelX = voxelBox[3];
            int maxVoxelY = voxelBox[4];
            int maxVoxelZ = voxelBox[5];

            AppendSliceX(minVoxelX, maxVoxelX, boxIndex);
            AppendSliceY(minVoxelY, maxVoxelY, boxIndex);
            AppendSliceZ(minVoxelZ, maxVoxelZ, boxIndex);
        }

        // The indices of the boxes occupying the X slice, or null if none.
        public List<int>? GetSliceX(int voxelIndex)
        {
            List<int> slice = _slicesX[voxelIndex];
            return slice.Count == 0 ? null : slice;
        }

        // The indices of the boxes occupying the Y slice, or null if none.
        public List<int>? GetSliceY(int voxelIndex)
        {
            List<int> slice = _slicesY[voxelIndex];
            return slice.Count == 0 ? null : slice;
        }

        // The indices of the boxes occupying the Z slice, or null if none.
        public List<int>? GetSliceZ(int voxelIndex)
        {
            List<int> slice = _slicesZ[voxelIndex];
            return slice.Count == 0 ? null : slice;
        }

        private void AppendSliceX(int voxelIndexMin, int voxelIndexMax, int boxIndex)
        {
            for (int i = voxelIndexMin; i <= voxelIndexMax; ++i)
            {
                _slicesX[i].Add(boxIndex);
            }
        }

        private void AppendSliceY(int voxelIndexMin, int voxelIndexMax, int boxIndex)
        {
            for (int i = voxelIndexMin; i <= voxelIndexMax; ++i)
            {
                _slicesY[i].Add(boxIndex);
            }
        }

        private void AppendSliceZ(int voxelIndexMin, int voxelIndexMax, int boxIndex)
        {
            for (int i = voxelIndexMin; i <= voxelIndexMax; ++i)
            {
                _slicesZ[i].Add(boxIndex);
            }
        }
    }
}
