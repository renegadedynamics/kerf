// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Elements.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dHatch/Geom2dHatch_Elements.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// The elements of a 2D hatcher, indexed by integers, and the face explorer that
/// <see cref="Geom2dHatch_Classifier"/> runs over them: one wire holding every element, and half-lines from the point
/// classified through sample points of the elements.
/// </summary>
/// <remarks>
/// A working object (decision 21). The map is an <see cref="NCollection_IntDataMap{TValue}"/>, so the edges are
/// explored in OCCT's order (decisions 543 and 666).
/// </remarks>
public sealed class Geom2dHatch_Elements : ITopClass_FaceExplorer<Geom2dAdaptor_Curve>
{
    //  Modified by skv - Fri Jul 14 17:03:47 2006 OCC12627
    private const double Probing_Start = 0.123;
    private const double Probing_End = 0.8;
    private const double Probing_Step = 0.2111;

    private readonly NCollection_IntDataMap<Geom2dHatch_Element> myMap = new();
    private readonly NCollection_IntDataMap<Geom2dHatch_Element>.Iterator Iter = new();
    private int NumWire;
#pragma warning disable CS0414 // OCCT resets the edge counter without reading it
    private int NumEdge;
#pragma warning restore CS0414
    private int myCurEdge;
    private double myCurEdgePar;

    /// <summary>An empty set of elements.</summary>
    public Geom2dHatch_Elements()
    {
        NumWire = 0;
        NumEdge = 0;
        myCurEdge = 1;
        myCurEdgePar = Probing_Start;
    }

    /// <summary>
    /// OCCT's copy constructor, which copies nothing: the new set is empty, its current edge 0 and its probing
    /// parameter 0 (OCCT's "magic constructor"; occt-issues.md).
    /// </summary>
    public Geom2dHatch_Elements(Geom2dHatch_Elements Other)
    {
        _ = Other; // OCCT ignores its argument: the copy is empty (occt-issues.md)
        NumWire = 0;
        NumEdge = 0;
        myCurEdge = 0;
        myCurEdgePar = 0.0;
    }

    /// <summary>Removes all the elements.</summary>
    public void Clear()
    {
        myMap.Clear();
    }

    /// <summary>Binds the element <paramref name="I"/> to the index <paramref name="K"/>; false if K was bound (it is rebound).</summary>
    public bool Bind(int K, in Geom2dHatch_Element I) => myMap.Bind(K, I);

    /// <summary>True if the index <paramref name="K"/> is bound.</summary>
    public bool IsBound(int K) => myMap.IsBound(K);

    /// <summary>Unbinds the index <paramref name="K"/>; false if it wasn't bound.</summary>
    public bool UnBind(int K) => myMap.UnBind(K);

    /// <summary>The element bound to <paramref name="K"/>.</summary>
    /// <exception cref="NoSuchObjectException"><paramref name="K"/> is not bound.</exception>
    public Geom2dHatch_Element Find(int K) => myMap.Find(K);

    /// <summary>The element bound to <paramref name="K"/>, for writing.</summary>
    /// <exception cref="NoSuchObjectException"><paramref name="K"/> is not bound.</exception>
    internal ref Geom2dHatch_Element ChangeFind(int K) => ref myMap.ChangeFind(K);

    /// <summary>OCCT's <c>operator()</c>: the element bound to <paramref name="K"/>.</summary>
    public Geom2dHatch_Element this[int K] => Find(K);

    /// <summary>Accepts every point.</summary>
    public bool CheckPoint(ref Point2D P) => true;

    /// <summary>Rejects no point.</summary>
    public bool Reject(in Point2D P) => false;

    /// <summary>The first half-line from <paramref name="P"/>: through the first suitable sample point of an element.</summary>
    public bool Segment(in Point2D P, out Line2D L, out double Par)
    {
        myCurEdge = 1;
        myCurEdgePar = Probing_Start;
        return OtherSegment(P, out L, out Par);
    }

    /// <summary>
    /// The next half-line from <paramref name="P"/>: through the next sample point of the elements, not too close to
    /// tangent to the element and passing through neither end of it; false when there is none.
    /// </summary>
    public bool OtherSegment(in Point2D P, out Line2D L, out double Par)
    {
        NCollection_IntDataMap<Geom2dHatch_Element>.Iterator Itertemp = new();
        int i;
        Par = 0.0;
        for (Itertemp.Initialize(myMap), i = 1; Itertemp.More(); Itertemp.Next(), i++)
        {
            if (i < myCurEdge)
            {
                continue;
            }

            ref Geom2dHatch_Element Item = ref myMap.ChangeFind(Itertemp.Key());
            Geom2dAdaptor_Curve E = Item.ChangeCurve();
            TopAbs_Orientation Or = Item.Orientation();
            if (Or == TopAbs_FORWARD || Or == TopAbs_REVERSED)
            {
                double aFPar = E.FirstParameter(), aLPar = E.LastParameter();
                if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(aFPar))
                {
                    if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(aLPar))
                    {
                        aFPar = -1.0;
                        aLPar = 1.0;
                    }
                    else
                    {
                        aFPar = aLPar - 1.0;
                    }
                }
                else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(aLPar))
                {
                    aLPar = aFPar + 1.0;
                }

                for (; myCurEdgePar < Probing_End; myCurEdgePar += Probing_Step)
                {
                    double aParam = myCurEdgePar * aFPar + (1.0 - myCurEdgePar) * aLPar;
                    E.D1(aParam, out Point2D aPOnC, out Vector2D aTanVec);
                    Vector2D aLinVec = new(P, aPOnC);
                    Par = aLinVec.SquareMagnitude();
                    if (Par > global::Kerf.Engine.Foundation.Precision.SquarePConfusion())
                    {
                        Direction2D aLinDir = new(aLinVec);
                        double aTanMod = aTanVec.SquareMagnitude();
                        if (aTanMod < global::Kerf.Engine.Foundation.Precision.SquarePConfusion())
                        {
                            continue;
                        }

                        aTanVec /= Math.Sqrt(aTanMod);
                        double aSinA = aTanVec.Crossed(new Vector2D(aLinDir));
                        if (Math.Abs(aSinA) < 0.001)
                        {
                            // too small angle - line and edge may be considered
                            // as tangent which is bad for classifier
                            if (myCurEdgePar + Probing_Step < Probing_End)
                            {
                                continue;
                            }
                        }

                        L = new Line2D(P, aLinDir);

                        aPOnC = E.Value(aFPar);
                        if (L.SquareDistance(aPOnC) > global::Kerf.Engine.Foundation.Precision.SquarePConfusion())
                        {
                            aPOnC = E.Value(aLPar);
                            if (L.SquareDistance(aPOnC) > global::Kerf.Engine.Foundation.Precision.SquarePConfusion())
                            {
                                myCurEdgePar += Probing_Step;
                                if (myCurEdgePar >= Probing_End)
                                {
                                    myCurEdge++;
                                    myCurEdgePar = Probing_Start;
                                }
                                Par = Math.Sqrt(Par);
                                return true;
                            }
                        }
                    }
                }
            }
            myCurEdge++;
            myCurEdgePar = Probing_Start;
        }

        Par = RealLast();
        L = new Line2D(P, new Direction2D(Direction2D.D.X));
        return false;
    }

    /// <summary>Starts the exploration of the wires (there is one).</summary>
    public void InitWires()
    {
        NumWire = 0;
    }

    /// <summary>True while there is a wire to explore.</summary>
    public bool MoreWires() => (NumWire == 0);

    /// <summary>Moves to the next wire.</summary>
    public void NextWire()
    {
        NumWire++;
    }

    /// <summary>Rejects no wire.</summary>
    public bool RejectWire(in Line2D L, double Par) => false;

    /// <summary>Starts the exploration of the edges (the elements, in OCCT's map order).</summary>
    public void InitEdges()
    {
        NumEdge = 0;
        Iter.Initialize(myMap);
    }

    /// <summary>True while there is an edge to explore.</summary>
    public bool MoreEdges() => (Iter.More());

    /// <summary>Moves to the next edge.</summary>
    public void NextEdge()
    {
        Iter.Next();
    }

    /// <summary>Rejects no edge.</summary>
    public bool RejectEdge(in Line2D L, double Par) => false;

    /// <summary>
    /// The current edge: a shallow copy of the element's curve (OCCT copies it into <paramref name="E"/>), and its
    /// orientation.
    /// </summary>
    public void CurrentEdge(ref Geom2dAdaptor_Curve E, out TopAbs_Orientation Or)
    {
        Geom2dHatch_Element anItem = myMap.Find(Iter.Key());
        E = (Geom2dAdaptor_Curve)anItem.Curve().ShallowCopy();
        Or = anItem.Orientation();
    }
}
