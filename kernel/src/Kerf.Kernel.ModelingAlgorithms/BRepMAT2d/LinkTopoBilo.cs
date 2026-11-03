// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepMAT2d/BRepMAT2d_LinkTopoBilo.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepMAT2d/BRepMAT2d_LinkTopoBilo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>The links between the edges and vertices of the explorer's face and the basic elements of its
/// bisecting locus.</summary>
/// <remarks>
/// A working object. The links of a contour are made by iterating an integer map (basic element to curve) in OCCT's
/// bucket order (NCollection_IntDataMap, decision 543): it decides the order of the basic elements linked to a vertex.
/// The other maps are only looked up; the map of basic elements compares them by reference, as OCCT's handles.
/// </remarks>
public sealed class LinkTopoBilo
{
    private readonly Dictionary<Shape, NCollection_Sequence<MedialBasicElt>> _map =
        new(ShapeHasher.Instance);
    private readonly Dictionary<MedialBasicElt, Shape> _beShape = new(ReferenceEqualityComparer.Instance);
    private Shape _key;
    private int _current;
    private bool _isEmpty;

    public LinkTopoBilo()
    {
        _current = 0;
        _isEmpty = true;
    }

    /// <summary>The links between the face of Explo and BiLo; throws Standard_ConstructionError if the explorer's
    /// shape is not a face.</summary>
    public LinkTopoBilo(MedialAxisExplorer explo, BisectingLocus biLo)
    {
        Perform(explo, biLo);
    }

    /// <summary>Builds the links between the face of Explo and BiLo; throws Standard_ConstructionError if the
    /// explorer's shape is not a face.</summary>
    public void Perform(MedialAxisExplorer explo, BisectingLocus biLo)
    {
        _map.Clear();
        _beShape.Clear();

        Shape s = explo.Shape();
        int indContour = 1;

        if (s.ShapeType() == ShapeType.Face)
        {
            Explorer exp = new(s, ShapeType.Wire);

            while (exp.More())
            {
                LinkToWire(ShapeCasts.Wire(exp.Current()), explo, indContour, biLo);
                exp.Next();
                indContour++;
            }
        }
        else
        {
            throw new ConstructionErrorException("BRepMAT2d_LinkTopoBilo::Perform");
        }
    }

    /// <summary>Starts an iteration over the basic elements linked to S, an edge or a vertex of the initial face.
    /// </summary>
    public void Init(in Shape s)
    {
        _isEmpty = false;
        _current = 1;
        if (_map.ContainsKey(s))
        {
            _key = s;
        }
        else
        {
            _isEmpty = true;
        }
    }

    /// <summary>True if there is a current basic element.</summary>
    public bool More()
    {
        if (_isEmpty)
        {
            return false;
        }
        return _current <= _map.GetValue(_key).Length();
    }

    /// <summary>Moves to the next basic element.</summary>
    public void Next() => _current++;

    /// <summary>The current basic element.</summary>
    public MedialBasicElt Value() => _map.GetValue(_key).Value(_current);

    /// <summary>The shape linked to BE.</summary>
    public Shape GeneratingShape(MedialBasicElt be) => _beShape.GetValue(be);

    private void LinkToWire(in Wire w, MedialAxisExplorer explo, int indC, BisectingLocus biLo)
    {
        WireExplorer theExp = new(w);
        int kc;
        Vertex vf, vl;
        Shape s;
        MedialBasicElt be;
        Type type;
        NCollection_Sequence<Shape> topoSeq = new();
        NCollection_IntDataMap<int> linkBECont = new();

        for (; theExp.More(); theExp.Next())
        {
            topoSeq.Append(theExp.Current());
        }

        // The links BasicElt => curve of the contour IndC.
        LinkToContour(explo, indC, biLo, linkBECont);

        // Iteration over the BasicElts (in OCCT's bucket order). The index is the same for the curves of the
        // contour and the edges.
        for (NCollection_IntDataMap<int>.Iterator ite = new(linkBECont); ite.More(); ite.Next())
        {
            be = biLo.Graph().BasicElt(ite.Key());
            type = biLo.GeomElt(be).GetType();
            kc = ite.Value();
            s = topoSeq.Value(Math.Abs(kc));

            if (type == typeof(CartesianPoint2))
            {
                if (s.Orientation() == Reversed)
                {
                    Exploration.Vertices(ShapeCasts.Edge(s), out vl, out vf);
                }
                else
                {
                    Exploration.Vertices(ShapeCasts.Edge(s), out vf, out vl);
                }
                if (kc > 0)
                {
                    s = vl;
                }
                else
                {
                    s = vf;
                }
            }
            if (!_map.ContainsKey(s))
            {
                _map[s] = new NCollection_Sequence<MedialBasicElt>();
            }
            _map.GetValue(s).Append(be);

            if (kc < 0)
            {
                _beShape[be] = s.Oriented(global::Kerf.Engine.Topology.TopAbs.Reverse(s.Orientation()));
            }
            else
            {
                _beShape[be] = s;
            }
        }
    }

    // Associates each basic element of the contour IndC with the curve of the initial contour it comes from (a
    // negative index when met in the reverse sense, on open lines).
    private static void LinkToContour(MedialAxisExplorer explo, int indC, BisectingLocus biLo,
        NCollection_IntDataMap<int> link)
    {
        MedialBasicElt be;
        Geometry2D geomBE;
        Type type;
        bool directSense = true;
        bool lastPoint = false;
        int nbSect, iSect;

        // NbSect: the number of sections of the current curve; ISect: the counter of sections.
        NCollection_Sequence<Curve2D> cont = explo.Contour(indC);

        // On the first curve of the contour.
        int indOnCont = 1;
        int precIndOnCont = -1;
        nbSect = biLo.NumberOfSections(indC, 1);
        iSect = 0;

        // The basic elements of the contour IndC (they are ordered).
        for (int i = 1; i <= biLo.NumberOfElts(indC); i++)
        {
            be = biLo.BasicElt(indC, i);
            geomBE = biLo.GeomElt(be);
            type = geomBE.GetType();

            if (type != typeof(CartesianPoint2))
            {
                iSect++;
                // A curve: associated with the current curve.
                if (directSense)
                {
                    link.Bind(be.Index(), indOnCont);
                }
                else
                {
                    link.Bind(be.Index(), -indOnCont);
                }
            }
            else
            {
                // A point: associated with the previous curve.
                if (directSense || lastPoint)
                {
                    link.Bind(be.Index(), precIndOnCont);
                }
                else
                {
                    link.Bind(be.Index(), -precIndOnCont);
                }
            }

            precIndOnCont = indOnCont;

            // To the next curve of Explo, once all the parts of the initial curve are seen.
            if (type != typeof(CartesianPoint2) && iSect == nbSect)
            {
                if (indOnCont < cont.Length() && directSense)
                {
                    indOnCont++;
                    nbSect = biLo.NumberOfSections(indC, indOnCont);
                    iSect = 0;
                }
                else
                {
                    // Open lines restart in the other direction.
                    if (!directSense)
                    {
                        indOnCont--;
                        if (indOnCont != 0)
                        {
                            nbSect = biLo.NumberOfSections(indC, indOnCont);
                        }
                        lastPoint = false;
                    }
                    else
                    {
                        lastPoint = true;
                    }
                    iSect = 0;
                    directSense = false;
                }
            }
        }
    }
}
