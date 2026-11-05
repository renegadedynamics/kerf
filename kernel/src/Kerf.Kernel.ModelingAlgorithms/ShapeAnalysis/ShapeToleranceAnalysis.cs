// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_ShapeTolerance.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_ShapeTolerance.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// Computes shape tolerances (minimal, maximal, average) and finds the sub-shapes whose tolerance matches given
/// criteria. Only reads the shapes.
/// </summary>
/// <remarks>A working object (decision 21): it cumulates tolerances over the shapes added.</remarks>
internal sealed class ShapeToleranceAnalysis
{
    private readonly double[] _tols = new double[3];
    private int _nbTol;

    /// <summary>Empty constructor.</summary>
    public ShapeToleranceAnalysis()
    {
        _nbTol = 0;
    }

    private static void AddTol(double tol, ref int nbt, ref double cmin, ref double cmoy, ref double cmax)
    {
        nbt++;
        if (nbt == 1)
        {
            cmin = cmoy = cmax = tol;
        }
        else
        {
            if (cmin > tol)
            {
                cmin = tol;
            }
            if (cmax < tol)
            {
                cmax = tol;
            }
            //  Calcul en moyenne geometrique  entre 1 et 1e-7
            int mult = 1;
            // #76 rln 11.03.99 S4135: compute average without weights according to tolerances
            nbt += (mult - 1);
            cmoy += (tol * mult);
        }
    }

    /// <summary>
    /// A tolerance of the shape: <paramref name="mode"/> = 0 the average, &gt; 0 the maximum, &lt; 0 the minimum.
    /// <paramref name="type"/>: SHAPE (vertices, edges and faces), VERTEX, EDGE or FACE. Starts a new cumulation.
    /// </summary>
    public double Tolerance(in Shape shape, int mode, ShapeType type = ShapeType.Shape)
    {
        InitTolerance();
        AddTolerance(shape, type);
        return GlobalTolerance(mode);
    }

    /// <summary>
    /// The sub-shapes whose tolerance is over <paramref name="value"/> (<paramref name="type"/> as in
    /// <see cref="Tolerance"/>, and SHELL: shells and faces whose faces, edges or vertices are over it). A new sequence.
    /// </summary>
    public NCollection_HSequence<Shape> OverTolerance(in Shape shape, double value, ShapeType type = ShapeType.Shape)
    {
        if (value >= 0)
        {
            return InTolerance(shape, value, 0.0, type);
        }
        else
        {
            return InTolerance(shape, 0.0, value, type);
        }
    }

    /// <summary>
    /// The sub-shapes whose tolerance is within [<paramref name="valmin"/>, <paramref name="valmax"/>] (no upper bound
    /// if <paramref name="valmax"/> &lt; <paramref name="valmin"/>). A new sequence.
    /// </summary>
    public NCollection_HSequence<Shape> InTolerance(in Shape shape, double valmin, double valmax,
                                                           ShapeType type = ShapeType.Shape)
    {
        double tol;
        bool over = (valmax < valmin); // pas de liminite max
        var sl = new NCollection_HSequence<Shape>();

        Explorer myExp = new();

        // Iteration sur les Faces

        if (type == ShapeType.Face || type == ShapeType.Shape)
        {
            myExp.Init(shape, ShapeType.Face);
            while (myExp.More())
            {
                tol = Tool.Tolerance(ShapeCasts.Face(myExp.Current()));
                if (tol >= valmin && (over || (tol <= valmax)))
                {
                    sl.Append(myExp.Current());
                }
                myExp.Next();
            }
        }

        // Iteration sur les Edges

        if (type == ShapeType.Edge || type == ShapeType.Shape)
        {
            myExp.Init(shape, ShapeType.Edge);
            while (myExp.More())
            {
                tol = Tool.Tolerance(ShapeCasts.Edge(myExp.Current()));
                if (tol >= valmin && (over || (tol <= valmax)))
                {
                    sl.Append(myExp.Current());
                }
                myExp.Next();
            }
        }

        // Iteration sur les Vertex

        if (type == ShapeType.Vertex || type == ShapeType.Shape)
        {
            myExp.Init(shape, ShapeType.Vertex);
            while (myExp.More())
            {
                tol = Tool.Tolerance(ShapeCasts.Vertex(myExp.Current()));
                // OCCT bug: "tol >= valmax" for vertices, where faces and edges test "tol <= valmax" (occt-issues.md)
                if (tol >= valmin && (over || (tol >= valmax)))
                {
                    sl.Append(myExp.Current());
                }
                myExp.Next();
            }
        }

        // Iteration combinee (cumul) SHELL+FACE+EDGE+VERTEX, on retourne SHELL+FACE

        if (type == ShapeType.Shell)
        {
            //  Exploration des shells
            var mapface = new HashSet<Shape>(ShapeHasher.Instance);
            myExp.Init(shape, ShapeType.Shell);
            while (myExp.More())
            {
                bool iashell = false;
                Shape ash = myExp.Current();
                for (var face = new Explorer(ash, ShapeType.Face); face.More(); face.Next())
                {
                    mapface.Add(face.Current());
                    NCollection_HSequence<Shape> fc = InTolerance(face.Current(), valmin, valmax, type);
                    if (fc.Length() > 0)
                    {
                        sl.Append(fc);
                        iashell = true;
                    }
                }
                if (iashell)
                {
                    sl.Append(ash);
                }
                myExp.Next();
            }

            //  Les faces (libres ou sous shell)
            myExp.Init(shape, ShapeType.Face);
            for (; myExp.More(); myExp.Next())
            {
                bool iaface = false;
                if (mapface.Contains(myExp.Current()))
                {
                    continue;
                }
                tol = Tool.Tolerance(ShapeCasts.Face(myExp.Current()));
                if (tol >= valmin && (over || (tol <= valmax)))
                {
                    iaface = true;
                }
                else
                {
                    // les edges contenues ?
                    NCollection_HSequence<Shape> fl = InTolerance(myExp.Current(), valmin, valmax, ShapeType.Edge);
                    if (fl.Length() > 0)
                    {
                        iaface = true;
                    }
                    else
                    {
                        fl = InTolerance(myExp.Current(), valmin, valmax, ShapeType.Vertex);
                        if (fl.Length() > 0)
                        {
                            iaface = true;
                        }
                    }
                }
                if (iaface)
                {
                    sl.Append(myExp.Current());
                }
            }
        }

        return sl;
    }

    /// <summary>Starts a cumulation of tolerances.</summary>
    public void InitTolerance()
    {
        _nbTol = 0;
        _tols[1] = 0;
    }

    /// <summary>Adds the tolerances of a shape to the cumulation (maximum, average and minimum).</summary>
    public void AddTolerance(in Shape shape, ShapeType type = ShapeType.Shape)
    {
        int nbt = 0;
        double tol, cmin = 0.0, cmoy = 0.0, cmax = 0.0;

        Explorer myExp = new();

        // Iteration sur les Faces

        if (type == ShapeType.Face || type == ShapeType.Shape)
        {
            myExp.Init(shape, ShapeType.Face);
            while (myExp.More())
            {
                tol = Tool.Tolerance(ShapeCasts.Face(myExp.Current()));
                AddTol(tol, ref nbt, ref cmin, ref cmoy, ref cmax);
                myExp.Next();
            }
        }

        // Iteration sur les Edges

        if (type == ShapeType.Edge || type == ShapeType.Shape)
        {
            myExp.Init(shape, ShapeType.Edge);
            while (myExp.More())
            {
                tol = Tool.Tolerance(ShapeCasts.Edge(myExp.Current()));
                AddTol(tol, ref nbt, ref cmin, ref cmoy, ref cmax);
                myExp.Next();
            }
        }

        // Iteration sur les Vertices

        if (type == ShapeType.Vertex || type == ShapeType.Shape)
        {
            myExp.Init(shape, ShapeType.Vertex);
            while (myExp.More())
            {
                tol = Tool.Tolerance(ShapeCasts.Vertex(myExp.Current()));
                AddTol(tol, ref nbt, ref cmin, ref cmoy, ref cmax);
                myExp.Next();
            }
        }

        //  Resultat : attention en mode cumul
        if (nbt == 0)
        {
            return;
        }
        if (_nbTol == 0 || _tols[0] > cmin)
        {
            _tols[0] = cmin;
        }
        if (_nbTol == 0 || _tols[2] < cmax)
        {
            _tols[2] = cmax;
        }
        _tols[1] += cmoy;
        _nbTol += nbt;
    }

    /// <summary>The cumulated tolerance: <paramref name="mode"/> = 0 the average, &gt; 0 the maximum, &lt; 0 the minimum.</summary>
    public double GlobalTolerance(int mode)
    {
        // szv#4:S4163:12Mar99 optimized
        double result = 0.0;
        if (_nbTol != 0.0)
        {
            if (mode < 0)
            {
                result = _tols[0];
            }
            else if (mode == 0)
            {
                if (_tols[0] == _tols[2])
                {
                    result = _tols[0];
                }
                else
                {
                    result = _tols[1] / _nbTol;
                }
            }
            else
            {
                result = _tols[2];
            }
        }

        return result;
    }
}
