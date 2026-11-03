// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_LineOn2S.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_LineOn2S.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_LineOn2S.cxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntSurf/IntSurf_Allocator.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The points of a walking line between two surfaces (<see cref="IntSurf_PntOn2S"/>, indexed from 1), with lazily
/// computed boxes of the 3D points and of the parameters on each surface.
/// </summary>
/// <remarks>
/// A working object (decision 21; plan phase 5-6, notes for porters): the walking, purging and splitting algorithms
/// edit it in place, so its mutators are public, and it is never frozen. Maps keyed by it compare by reference.
/// OCCT's allocator argument (IntSurf_Allocator, a handle to an NCollection allocator) is dropped
/// (translation rules, section 9).
/// <para>
/// The boxes follow OCCT: a box is "whole" (unknown) until one of the <c>IsOut*</c> queries computes it from all the
/// points, enlarged by 1% of its largest side; after that <see cref="Add"/>, <see cref="InsertBefore"/> and
/// <see cref="SetUV"/> grow it, and the other edits make it unknown again.
/// </para>
/// </remarks>
public class IntSurf_LineOn2S : Transient
{
    private readonly NCollection_Sequence<IntSurf_PntOn2S> mySeq;
    private Box2d myBuv1;
    private Box2d myBuv2;
    private Box myBxyz;

    public IntSurf_LineOn2S()
    {
        mySeq = new NCollection_Sequence<IntSurf_PntOn2S>();
        myBuv1 = new Box2d();
        myBuv2 = new Box2d();
        myBxyz = new Box();
        myBuv1.SetWhole();
        myBuv2.SetWhole();
        myBxyz.SetWhole();
    }

    /// <summary>Adds a point at the end of the line.</summary>
    public void Add(in IntSurf_PntOn2S P)
    {
        mySeq.Append(P);
        if (!myBxyz.IsWhole)
        {
            myBxyz.Add(P.Value());
        }
        if (!myBuv1.IsWhole)
        {
            myBuv1.Add(P.ValueOnSurface(true));
        }
        if (!myBuv2.IsWhole)
        {
            myBuv2.Add(P.ValueOnSurface(false));
        }
    }

    /// <summary>The number of points.</summary>
    public int NbPoints() => mySeq.Length();

    /// <summary>The point of range <paramref name="Index"/>.</summary>
    public IntSurf_PntOn2S Value(int Index) => mySeq.Value(Index);

    /// <summary>Reverses the order of the points.</summary>
    public void Reverse() => mySeq.Reverse();

    /// <summary>Keeps the points 1 to <paramref name="Index"/> - 1 and returns the others as a new line.</summary>
    public IntSurf_LineOn2S Split(int Index)
    {
        NCollection_Sequence<IntSurf_PntOn2S> SS = new();
        mySeq.Split(Index, SS);
        IntSurf_LineOn2S NS = new();
        int i;
        int leng = SS.Length();
        for (i = 1; i <= leng; i++)
        {
            NS.Add(SS[i]);
        }
        return NS;
    }

    /// <summary>Replaces the point of range <paramref name="Index"/>.</summary>
    public void Value(int Index, in IntSurf_PntOn2S P)
    {
        mySeq[Index] = P;
        myBuv1.SetWhole();
        myBuv2.SetWhole();
        myBxyz.SetWhole();
    }

    /// <summary>Sets the 3D point of the point of range <paramref name="Index"/>.</summary>
    public void SetPoint(int Index, in Point3D thePnt)
    {
        mySeq[Index].SetValue(thePnt);
        myBuv1.SetWhole();
        myBuv2.SetWhole();
        myBxyz.SetWhole();
    }

    /// <summary>Sets the parameters on one of the surfaces of the point of range <paramref name="Index"/>.</summary>
    public void SetUV(int Index, bool OnFirst, double U, double V)
    {
        mySeq[Index].SetValue(OnFirst, U, V);

        if (OnFirst && !myBuv1.IsWhole)
        {
            myBuv1.Add(new Point2D(U, V));
        }
        else if (!OnFirst && !myBuv2.IsWhole)
        {
            myBuv2.Add(new Point2D(U, V));
        }
    }

    public void Clear()
    {
        mySeq.Clear();
        myBuv1.SetWhole();
        myBuv2.SetWhole();
        myBxyz.SetWhole();
    }

    /// <summary>Inserts a point before <paramref name="index"/>, or appends it if the index is past the end.</summary>
    public void InsertBefore(int index, in IntSurf_PntOn2S P)
    {
        if (index > mySeq.Length())
        {
            mySeq.Append(P);
        }
        else
        {
            mySeq.InsertBefore(index, P);
        }

        if (!myBxyz.IsWhole)
        {
            myBxyz.Add(P.Value());
        }

        if (!myBuv1.IsWhole)
        {
            myBuv1.Add(P.ValueOnSurface(true));
        }

        if (!myBuv2.IsWhole)
        {
            myBuv2.Add(P.ValueOnSurface(false));
        }
    }

    public void RemovePoint(int index)
    {
        mySeq.Remove(index);
        myBuv1.SetWhole();
        myBuv2.SetWhole();
        myBxyz.SetWhole();
    }

    /// <summary>True if <paramref name="theP"/> is out of the box of the parameters on the first surface.</summary>
    public bool IsOutSurf1Box(in Point2D theP)
    {
        if (NbPoints() == 0)
        {
            return false;
        }
        if (myBuv1.IsWhole)
        {
            int n = NbPoints();
            double pu1, pu2, pv1, pv2;
            myBuv1.SetVoid();
            for (int i = 1; i <= n; i++)
            {
                mySeq[i].Parameters(out pu1, out pv1, out pu2, out pv2);
                myBuv1.Add(new Point2D(pu1, pv1));
            }
            myBuv1.Get(out pu1, out pv1, out pu2, out pv2);
            pu2 -= pu1;
            pv2 -= pv1;
            if (pu2 > pv2)
            {
                myBuv1.Enlarge(pu2 * 0.01);
            }
            else
            {
                myBuv1.Enlarge(pv2 * 0.01);
            }
        }
        bool @out = myBuv1.IsOut(theP);
        return (@out);
    }

    /// <summary>True if <paramref name="theP"/> is out of the box of the parameters on the second surface.</summary>
    public bool IsOutSurf2Box(in Point2D theP)
    {
        if (NbPoints() == 0)
        {
            return false;
        }
        if (myBuv2.IsWhole)
        {
            int n = NbPoints();
            double pu1, pu2, pv1, pv2;
            myBuv2.SetVoid();
            for (int i = 1; i <= n; i++)
            {
                mySeq[i].Parameters(out pu1, out pv1, out pu2, out pv2);
                myBuv2.Add(new Point2D(pu2, pv2));
            }
            myBuv2.Get(out pu1, out pv1, out pu2, out pv2);
            pu2 -= pu1;
            pv2 -= pv1;
            if (pu2 > pv2)
            {
                myBuv2.Enlarge(pu2 * 0.01);
            }
            else
            {
                myBuv2.Enlarge(pv2 * 0.01);
            }
        }
        bool @out = myBuv2.IsOut(theP);
        return (@out);
    }

    /// <summary>True if <paramref name="theP"/> is out of the box of the 3D points.</summary>
    public bool IsOutBox(in Point3D theP)
    {
        if (NbPoints() == 0)
        {
            return false;
        }
        if (myBxyz.IsWhole)
        {
            int n = NbPoints();
            myBxyz.SetVoid();
            for (int i = 1; i <= n; i++)
            {
                Point3D P = mySeq[i].Value();
                myBxyz.Add(P);
            }
            myBxyz.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
            x1 -= x0;
            y1 -= y0;
            z1 -= z0;
            if (x1 > y1)
            {
                if (x1 > z1)
                {
                    myBxyz.Enlarge(x1 * 0.01);
                }
                else
                {
                    myBxyz.Enlarge(z1 * 0.01);
                }
            }
            else
            {
                if (y1 > z1)
                {
                    myBxyz.Enlarge(y1 * 0.01);
                }
                else
                {
                    myBxyz.Enlarge(z1 * 0.01);
                }
            }
        }
        bool @out = myBxyz.IsOut(theP);
        return (@out);
    }
}
