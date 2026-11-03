// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_Int3Pln.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_Int3Pln.lxx
// Port of: src/ModelingData/TKGeomBase/IntAna/IntAna_Int3Pln.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Intersection between 3 planes. The algorithm searches for an intersection point. If two of the planes are
/// parallel or identical, IsEmpty returns true.
/// </summary>
/// <remarks>An algorithm object: construct it with the planes, or empty and call <c>Perform</c>.</remarks>
public class IntAna_Int3Pln
{
    private bool done;
    private bool empt;
    private Point3D pnt;

    public IntAna_Int3Pln()
    {
        done = false;
        empt = true;
    }

    /// <summary>Determination of the intersection point between 3 planes.</summary>
    public IntAna_Int3Pln(in Plane3D P1, in Plane3D P2, in Plane3D P3)
    {
        Perform(P1, P2, P3);
    }

    /// <summary>Determination of the intersection point between 3 planes.</summary>
    public void Perform(in Plane3D P1, in Plane3D P2, in Plane3D P3)
    {
        done = false;
        Matrix M = new(1, 3, 1, 3);
        math_Vector V = new(1, 3);

        P1.Coefficients(out M[1, 1], out M[1, 2], out M[1, 3], out V[1]);
        P2.Coefficients(out M[2, 1], out M[2, 2], out M[2, 3], out V[2]);
        P3.Coefficients(out M[3, 1], out M[3, 2], out M[3, 3], out V[3]);

        Gauss Resol = new(M, Constants.Resolution());

        if (!Resol.IsDone)
        {
            empt = true;
        }
        else
        {
            empt = false;
            V.Initialized(-V);
            Resol.Solve(V);
            pnt.SetCoord(V[1], V[2], V[3]);
        }
        done = true;
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone() => done;

    /// <summary>True if there is no intersection point: two of the planes are identical or parallel.</summary>
    /// <exception cref="NotDoneException">The computation was not done.</exception>
    public bool IsEmpty()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return empt;
    }

    /// <summary>The intersection point.</summary>
    /// <exception cref="NotDoneException">The computation was not done.</exception>
    /// <exception cref="DomainErrorException">There is no intersection point.</exception>
    public Point3D Value()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        if (empt)
        {
            throw new DomainErrorException();
        }
        return pnt;
    }
}
