// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Occt;

namespace Kerf.Kernel.DrawLite;

/// <summary>
/// DrawTrSurf.cxx's transformations of geometry variables: translate, rotate, pmirror, lmirror, smirror, pscale and
/// their 2D forms; and the display settings <c>dmode</c>, <c>discr</c> and <c>defle</c> (unit 7W), which set and print
/// DrawTrSurf_Params' modal values and change nothing else without a viewer. The other display commands of the file
/// (clpoles, setcurvcolor, ...) need a viewer.
/// </summary>
/// <remarks>
/// OCCT transforms the bound curve or surface in place; geometry is frozen when bound (decision 476), so the port binds
/// the transformed copy under the same name. Points are bound again in OCCT too. As in OCCT, pmirror and pscale (and
/// 2dpmirror, 2dpscale) don't take the point's coordinates off the list of names, so those words are printed with them.
/// </remarks>
internal static class DrawTrSurfCommands
{
    public static void Register(DrawInterpreter theInterpreter)
    {
        foreach (string aName in new[] { "translate", "rotate", "pmirror", "lmirror", "smirror", "pscale" })
        {
            DrawCxx.Add(theInterpreter, aName, Transform);
        }

        foreach (string aName in new[] { "2dtranslate", "2drotate", "2dpmirror", "2dlmirror", "2dpscale" })
        {
            DrawCxx.Add(theInterpreter, aName, D2Transform);
        }

        foreach (string aName in new[] { "dmode", "discr", "defle" })
        {
            DrawCxx.Add(theInterpreter, aName, Draw);
        }
    }

    /// <summary>DrawTrSurf_Params: the display settings, per interpreter.</summary>
    private sealed class Params
    {
        public double Deflection = 0.01;
        public int Discret = 30;
        public int DrawMode;
    }

    private static readonly System.Runtime.CompilerServices.ConditionalWeakTable<DrawInterpreter, Params> s_params = new();

    // dmode [names...] Uniform/Discret; discr [names...] nbintervals; defle [names...] defle
    private static int Draw(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        Params aParams = s_params.GetOrCreateValue(theDI);
        if (n <= 2)
        {
            if (a[0] == "dmode")
            {
                // OCCT: any word sets the uniform mode
                if (n == 2)
                {
                    aParams.DrawMode = 1;
                }
                di.Append(aParams.DrawMode != 0 ? "u" : "d");
            }
            else if (a[0] == "discr")
            {
                if (n == 2)
                {
                    aParams.Discret = theDI.Atoi(a[n - 1]);
                }
                di.Append(aParams.Discret);
            }
            else if (a[0] == "defle")
            {
                if (n == 2)
                {
                    aParams.Deflection = theDI.Atof(a[n - 1]);
                }
                di.Append(aParams.Deflection);
            }
        }
        // with names, OCCT sets the drawable of a[1] (for every name, // OCCT) to the mode, which only changes how a view
        // draws it

        return 0;
    }

    private static int Transform(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 5)
        {
            return 1;
        }

        var T = new Transform3D();
        int last = n - 1;
        if (a[0] == "pscale")
        {
            double s = theDI.Atof(a[last]);
            last--;
            if (last < 4)
            {
                return 1;
            }

            var P = new Point3D(theDI.Atof(a[last - 2]), theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            T = T.WithScale(P, s);
        }
        else if (a[0][1..] == "mirror")
        {
            if (last < 4)
            {
                return 1;
            }

            var P = new Point3D(theDI.Atof(a[last - 2]), theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            if (a[0][0] == 'p')
            {
                T = T.WithMirror(P);
            }
            else
            {
                last -= 3;
                if (last < 4)
                {
                    return 1;
                }

                var O = new Point3D(theDI.Atof(a[last - 2]), theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
                last -= 3;
                var D = new Direction3D(P.X, P.Y, P.Z);
                if (a[0][0] == 'l')
                {
                    T = T.WithMirror(new Axis3D(O, D));
                }
                else if (a[0][0] == 's')
                {
                    var Pl = new Plane3D(O, D);
                    T = T.WithMirror(Pl.Position.Frame);
                }
            }
        }
        else if (a[0] == "translate")
        {
            if (last < 4)
            {
                return 1;
            }

            var V = new Vector3D(theDI.Atof(a[last - 2]), theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            last -= 3;
            T = T.WithTranslation(V);
        }
        else if (a[0] == "rotate")
        {
            if (last < 8)
            {
                return 1;
            }

            double ang = theDI.Atof(a[last]) * (Math.PI / 180.0);
            last--;
            var D = new Direction3D(theDI.Atof(a[last - 2]), theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            last -= 3;
            var P = new Point3D(theDI.Atof(a[last - 2]), theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            last -= 3;
            T = T.WithRotation(new Axis3D(P, D), ang);
        }

        for (int i = 1; i <= last; i++)
        {
            Geometry3D? G = DrawTrSurf.Get(theDI, a[i]);
            if (G is not null)
            {
                DrawTrSurf.Set(theDI, a[i], G.Transformed(T));
            }
            else
            {
                var P = new Point3D();
                if (DrawTrSurf.GetPoint(theDI, a[i], ref P))
                {
                    DrawTrSurf.Set(theDI, a[i], P.Transformed(T));
                }
            }

            di.Append(a[i]).Append(" ");
        }

        return 0;
    }

    private static int D2Transform(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 4)
        {
            return 1;
        }

        var T = new Transform2D();
        int last = n - 1;
        if (a[0] == "2dpscale")
        {
            double s = theDI.Atof(a[last]);
            last--;
            if (last < 3)
            {
                return 1;
            }

            var P = new Point2D(theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            T = T.WithScale(P, s);
        }
        else if (a[0] is "2dpmirror" or "2dlmirror")
        {
            if (last < 3)
            {
                return 1;
            }

            var P = new Point2D(theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            if (a[0] == "2dpmirror")
            {
                T = T.WithMirror(P);
            }
            else
            {
                last -= 2;
                if (last < 3)
                {
                    return 1;
                }

                var O = new Point2D(theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
                last -= 2;
                var D = new Direction2D(P.X, P.Y);
                T = T.WithMirror(new Axis2D(O, D));
            }
        }
        else if (a[0] == "2dtranslate")
        {
            if (last < 3)
            {
                return 1;
            }

            var V = new Vector2D(theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            last -= 2;
            T = T.WithTranslation(V);
        }
        else if (a[0] == "2drotate")
        {
            if (last < 4)
            {
                return 1;
            }

            double ang = theDI.Atof(a[last]) * (Math.PI / 180.0);
            last--;
            var P = new Point2D(theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            last -= 2;
            T = T.WithRotation(P, ang);
        }

        for (int i = 1; i <= last; i++)
        {
            Curve2D? G = DrawTrSurf.GetCurve2d(theDI, a[i]);
            if (G is not null)
            {
                DrawTrSurf.Set(theDI, a[i], (Curve2D)G.Transformed(T));
            }
            else
            {
                var P = new Point2D();
                if (DrawTrSurf.GetPoint2d(theDI, a[i], ref P))
                {
                    DrawTrSurf.Set(theDI, a[i], P.Transformed(T));
                }
            }

            di.Append(a[i]).Append(" ");
        }

        return 0;
    }
}
