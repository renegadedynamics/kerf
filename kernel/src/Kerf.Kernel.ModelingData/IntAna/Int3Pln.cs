// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_Int3Pln.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_Int3Pln.lxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_Int3Pln.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Intersection;

/// <summary>
/// Intersection between 3 planes. The algorithm searches for an intersection point. If two of the planes are
/// parallel or identical, IsEmpty returns true.
/// </summary>
/// <remarks>An algorithm object: construct it with the planes, or empty and call <c>Perform</c>.</remarks>
internal class Int3Pln
{
    private bool _done;
    private bool _empt;
    private Point3D _pnt;

    public Int3Pln()
    {
        _done = false;
        _empt = true;
    }

    /// <summary>Determination of the intersection point between 3 planes.</summary>
    public Int3Pln(in Plane3D p1, in Plane3D p2, in Plane3D p3)
    {
        Perform(p1, p2, p3);
    }

    /// <summary>Determination of the intersection point between 3 planes.</summary>
    public void Perform(in Plane3D p1, in Plane3D p2, in Plane3D p3)
    {
        _done = false;
        Matrix m = new(1, 3, 1, 3);
        math_Vector v = new(1, 3);

        p1.Coefficients(out m[1, 1], out m[1, 2], out m[1, 3], out v[1]);
        p2.Coefficients(out m[2, 1], out m[2, 2], out m[2, 3], out v[2]);
        p3.Coefficients(out m[3, 1], out m[3, 2], out m[3, 3], out v[3]);

        Gauss resol = new(m, Constants.Resolution());

        if (!resol.IsDone)
        {
            _empt = true;
        }
        else
        {
            _empt = false;
            v.Initialized(-v);
            resol.Solve(v);
            _pnt = _pnt.WithCoord(v[1], v[2], v[3]);
        }
        _done = true;
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone => _done;

    /// <summary>True if there is no intersection point: two of the planes are identical or parallel.</summary>
    /// <exception cref="NotDoneException">The computation was not done.</exception>
    public bool IsEmpty
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _empt;
        }
    }

    /// <summary>The intersection point.</summary>
    /// <exception cref="NotDoneException">The computation was not done.</exception>
    /// <exception cref="DomainErrorException">There is no intersection point.</exception>
    public Point3D Value()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (_empt)
        {
            throw new DomainErrorException();
        }
        return _pnt;
    }
}
