// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Line.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Line.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.Contap_IType;
using static Occt.IntSurf_TypeTrans;

namespace Occt;

/// <summary>
/// A contour line: a line, a circle, a walking line (a polyline on the surface) or a part of a restriction, with its
/// vertices sorted by their parameter on the line, and its transition on the surface.
/// </summary>
/// <remarks>
/// OCCT's class is used by value but holds its polyline and its vertices through handles, which its copies share. The
/// port makes it a class with OCCT's (shallow) copy as the copy constructor; ported code copies where C++ copies
/// (decision 1077). <see cref="Vertex"/> returns a reference to the stored vertex, as OCCT's. As in OCCT, the default
/// constructor leaves the type unset (<see cref="Contap_Lin"/> here).
/// </remarks>
public sealed class Contap_Line
{
    private IntSurf_TypeTrans Trans;
    private IntSurf_LineOn2S? curv;
    private NCollection_HSequence<Contap_Point> svtx;
    private Adaptor2d_Curve2d? thearc;
    private Contap_IType typL;
    private Point3D pt = new();
    private Direction3D dir1 = new();
    private Direction3D dir2 = new();
    private double rad;

    public Contap_Line()
    {
        svtx = new NCollection_HSequence<Contap_Point>();
        Trans = IntSurf_Undecided;
    }

    /// <summary>OCCT's implicit copy: the polyline and the vertices are shared, as OCCT's handles.</summary>
    public Contap_Line(Contap_Line theOther)
    {
        Trans = theOther.Trans;
        curv = theOther.curv;
        svtx = theOther.svtx;
        thearc = theOther.thearc;
        typL = theOther.typL;
        pt = theOther.pt;
        dir1 = theOther.dir1;
        dir2 = theOther.dir2;
        rad = theOther.rad;
    }

    /// <summary>Makes the line a walking line on the polyline <paramref name="L"/>.</summary>
    public void SetLineOn2S(IntSurf_LineOn2S L)
    {
        curv = L;
        typL = Contap_Walking;
    }

    /// <summary>Clears the polyline (shared with the line's copies, as in OCCT) and the vertices; a walking line.</summary>
    public void Clear()
    {
        if (curv is not null)
        {
            curv.Clear();
        }
        svtx = new NCollection_HSequence<Contap_Point>();
        typL = Contap_Walking;
    }

    /// <summary>The polyline of a walking line.</summary>
    public IntSurf_LineOn2S LineOn2S() => curv!;

    /// <summary>Starts a new sequence of vertices (the copies keep the old one).</summary>
    public void ResetSeqOfVertex()
    {
        svtx = new NCollection_HSequence<Contap_Point>();
    }

    /// <summary>Adds a point to the polyline.</summary>
    public void Add(in IntSurf_PntOn2S POn2S)
    {
        curv!.Add(POn2S);
    }

    /// <summary>Makes the line the straight line <paramref name="L"/>.</summary>
    public void SetValue(in Line3D L)
    {
        pt = L.Location();
        dir1 = L.Direction();
        typL = Contap_Lin;
    }

    /// <summary>Makes the line the circle <paramref name="C"/>.</summary>
    public void SetValue(in Circle3D C)
    {
        pt = C.Location();
        dir1 = C.Position().Direction();
        dir2 = C.Position().XDirection();
        rad = C.Radius();
        typL = Contap_Circle;
    }

    /// <summary>Makes the line the restriction <paramref name="A"/>.</summary>
    public void SetValue(Adaptor2d_Curve2d A)
    {
        thearc = A;
        typL = Contap_Restriction;
    }

    /// <summary>Adds a vertex, keeping the vertices sorted by their parameter on the line.</summary>
    public void Add(in Contap_Point P)
    {
        int n = svtx.Length();
        if (n == 0)
        {
            svtx.Append(P);
        }
        else
        {
            double prm = P.ParameterOnLine();
            if (prm > svtx.Value(n).ParameterOnLine())
            {
                svtx.Append(P);
            }
            else
            {
                for (int i = n - 1; i > 0; i--)
                {
                    if (prm > svtx.Value(i).ParameterOnLine())
                    {
                        svtx.InsertBefore(i + 1, P);
                        return;
                    }
                }
                svtx.Prepend(P);
            }
        }
    }

    public int NbVertex() => svtx.Length();

    /// <summary>The vertex <paramref name="Index"/>, by reference (OCCT's non-const reference).</summary>
    public ref Contap_Point Vertex(int Index) => ref svtx.ChangeSequence()[Index];

    /// <summary>Contap_Lin, Contap_Circle, Contap_Walking or Contap_Restriction.</summary>
    public Contap_IType TypeContour() => typL;

    /// <summary>The number of points of a walking line; throws Standard_DomainError on another type.</summary>
    public int NbPnts()
    {
        if (typL != Contap_Walking)
        {
            throw new DomainErrorException();
        }
        return (curv!.NbPoints());
    }

    /// <summary>The point <paramref name="Index"/> of a walking line; throws Standard_DomainError on another type.</summary>
    public IntSurf_PntOn2S Point(int Index)
    {
        if (typL != Contap_Walking)
        {
            throw new DomainErrorException();
        }
        return (curv!.Value(Index));
    }

    /// <summary>The straight line; throws Standard_DomainError on another type.</summary>
    public Line3D Line()
    {
        if (typL != Contap_Lin)
        {
            throw new DomainErrorException();
        }
        return new Line3D(pt, dir1);
    }

    /// <summary>The circle; throws Standard_DomainError on another type.</summary>
    public Circle3D Circle()
    {
        if (typL != Contap_Circle)
        {
            throw new DomainErrorException();
        }
        return new Circle3D(new Frame3D(pt, dir1, dir2), rad);
    }

    /// <summary>The restriction; throws Standard_DomainError on another type.</summary>
    public Adaptor2d_Curve2d Arc()
    {
        if (typL != Contap_Restriction)
        {
            throw new DomainErrorException();
        }
        return thearc!;
    }

    /// <summary>Sets the transition of the line.</summary>
    public void SetTransitionOnS(IntSurf_TypeTrans T)
    {
        Trans = T;
    }

    /// <summary>IN if, at the left of the line, the surface's normal is oriented towards the observer.</summary>
    public IntSurf_TypeTrans TransitionOnS() => (Trans);
}
