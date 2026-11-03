// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepMAT2d/BRepMAT2d_LinkTopoBilo.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepMAT2d/BRepMAT2d_LinkTopoBilo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>The links between the edges and vertices of the explorer's face and the basic elements of its
/// bisecting locus.</summary>
/// <remarks>
/// A working object. The links of a contour are made by iterating an integer map (basic element to curve) in OCCT's
/// bucket order (NCollection_IntDataMap, decision 543): it decides the order of the basic elements linked to a vertex.
/// The other maps are only looked up; the map of basic elements compares them by reference, as OCCT's handles.
/// </remarks>
public sealed class BRepMAT2d_LinkTopoBilo
{
    private readonly NCollection_DataMap<Shape, NCollection_Sequence<MAT_BasicElt>> myMap =
        new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<MAT_BasicElt, Shape> myBEShape = new(ReferenceEqualityComparer.Instance);
    private Shape myKey;
    private int current;
    private bool isEmpty;

    public BRepMAT2d_LinkTopoBilo()
    {
        current = 0;
        isEmpty = true;
    }

    /// <summary>The links between the face of Explo and BiLo; throws Standard_ConstructionError if the explorer's
    /// shape is not a face.</summary>
    public BRepMAT2d_LinkTopoBilo(BRepMAT2d_Explorer Explo, BRepMAT2d_BisectingLocus BiLo)
    {
        Perform(Explo, BiLo);
    }

    /// <summary>Builds the links between the face of Explo and BiLo; throws Standard_ConstructionError if the
    /// explorer's shape is not a face.</summary>
    public void Perform(BRepMAT2d_Explorer Explo, BRepMAT2d_BisectingLocus BiLo)
    {
        myMap.Clear();
        myBEShape.Clear();

        Shape S = Explo.Shape();
        int IndContour = 1;

        if (S.ShapeType() == ShapeType.Face)
        {
            Explorer Exp = new(S, ShapeType.Wire);

            while (Exp.More())
            {
                LinkToWire(ShapeCasts.Wire(Exp.Current()), Explo, IndContour, BiLo);
                Exp.Next();
                IndContour++;
            }
        }
        else
        {
            throw new ConstructionErrorException("BRepMAT2d_LinkTopoBilo::Perform");
        }
    }

    /// <summary>Starts an iteration over the basic elements linked to S, an edge or a vertex of the initial face.
    /// </summary>
    public void Init(in Shape S)
    {
        isEmpty = false;
        current = 1;
        if (myMap.IsBound(S))
        {
            myKey = S;
        }
        else
        {
            isEmpty = true;
        }
    }

    /// <summary>True if there is a current basic element.</summary>
    public bool More()
    {
        if (isEmpty)
        {
            return false;
        }
        return current <= myMap.Find(myKey).Length();
    }

    /// <summary>Moves to the next basic element.</summary>
    public void Next() => current++;

    /// <summary>The current basic element.</summary>
    public MAT_BasicElt Value() => myMap.Find(myKey).Value(current);

    /// <summary>The shape linked to BE.</summary>
    public Shape GeneratingShape(MAT_BasicElt BE) => myBEShape.Find(BE);

    private void LinkToWire(in Wire W, BRepMAT2d_Explorer Explo, int IndC, BRepMAT2d_BisectingLocus BiLo)
    {
        WireExplorer TheExp = new(W);
        int KC;
        Vertex VF, VL;
        Shape S;
        MAT_BasicElt BE;
        Type Type;
        NCollection_Sequence<Shape> TopoSeq = new();
        NCollection_IntDataMap<int> LinkBECont = new();

        for (; TheExp.More(); TheExp.Next())
        {
            TopoSeq.Append(TheExp.Current());
        }

        // The links BasicElt => curve of the contour IndC.
        LinkToContour(Explo, IndC, BiLo, LinkBECont);

        // Iteration over the BasicElts (in OCCT's bucket order). The index is the same for the curves of the
        // contour and the edges.
        for (NCollection_IntDataMap<int>.Iterator Ite = new(LinkBECont); Ite.More(); Ite.Next())
        {
            BE = BiLo.Graph().BasicElt(Ite.Key());
            Type = BiLo.GeomElt(BE).GetType();
            KC = Ite.Value();
            S = TopoSeq.Value(Math.Abs(KC));

            if (Type == typeof(CartesianPoint2))
            {
                if (S.Orientation() == Reversed)
                {
                    Exploration.Vertices(ShapeCasts.Edge(S), out VL, out VF);
                }
                else
                {
                    Exploration.Vertices(ShapeCasts.Edge(S), out VF, out VL);
                }
                if (KC > 0)
                {
                    S = VL;
                }
                else
                {
                    S = VF;
                }
            }
            if (!myMap.IsBound(S))
            {
                myMap.Bind(S, new NCollection_Sequence<MAT_BasicElt>());
            }
            myMap[S].Append(BE);

            if (KC < 0)
            {
                myBEShape.Bind(BE, S.Oriented(global::Kerf.Engine.Topology.TopAbs.Reverse(S.Orientation())));
            }
            else
            {
                myBEShape.Bind(BE, S);
            }
        }
    }

    // Associates each basic element of the contour IndC with the curve of the initial contour it comes from (a
    // negative index when met in the reverse sense, on open lines).
    private static void LinkToContour(BRepMAT2d_Explorer Explo, int IndC, BRepMAT2d_BisectingLocus BiLo,
        NCollection_IntDataMap<int> Link)
    {
        MAT_BasicElt BE;
        Geometry2D GeomBE;
        Type Type;
        bool DirectSense = true;
        bool LastPoint = false;
        int NbSect, ISect;

        // NbSect: the number of sections of the current curve; ISect: the counter of sections.
        NCollection_Sequence<Curve2D> Cont = Explo.Contour(IndC);

        // On the first curve of the contour.
        int IndOnCont = 1;
        int PrecIndOnCont = -1;
        NbSect = BiLo.NumberOfSections(IndC, 1);
        ISect = 0;

        // The basic elements of the contour IndC (they are ordered).
        for (int i = 1; i <= BiLo.NumberOfElts(IndC); i++)
        {
            BE = BiLo.BasicElt(IndC, i);
            GeomBE = BiLo.GeomElt(BE);
            Type = GeomBE.GetType();

            if (Type != typeof(CartesianPoint2))
            {
                ISect++;
                // A curve: associated with the current curve.
                if (DirectSense)
                {
                    Link.Bind(BE.Index(), IndOnCont);
                }
                else
                {
                    Link.Bind(BE.Index(), -IndOnCont);
                }
            }
            else
            {
                // A point: associated with the previous curve.
                if (DirectSense || LastPoint)
                {
                    Link.Bind(BE.Index(), PrecIndOnCont);
                }
                else
                {
                    Link.Bind(BE.Index(), -PrecIndOnCont);
                }
            }

            PrecIndOnCont = IndOnCont;

            // To the next curve of Explo, once all the parts of the initial curve are seen.
            if (Type != typeof(CartesianPoint2) && ISect == NbSect)
            {
                if (IndOnCont < Cont.Length() && DirectSense)
                {
                    IndOnCont++;
                    NbSect = BiLo.NumberOfSections(IndC, IndOnCont);
                    ISect = 0;
                }
                else
                {
                    // Open lines restart in the other direction.
                    if (!DirectSense)
                    {
                        IndOnCont--;
                        if (IndOnCont != 0)
                        {
                            NbSect = BiLo.NumberOfSections(IndC, IndOnCont);
                        }
                        LastPoint = false;
                    }
                    else
                    {
                        LastPoint = true;
                    }
                    ISect = 0;
                    DirectSense = false;
                }
            }
        }
    }
}
