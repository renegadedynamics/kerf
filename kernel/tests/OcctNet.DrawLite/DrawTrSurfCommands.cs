// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace OcctNet.DrawLite;

/// <summary>
/// DrawTrSurf.cxx's transformations of geometry variables: translate, rotate, pmirror, lmirror, smirror, pscale and
/// their 2D forms. The display commands of the same file (clpoles, dmode, setcurvcolor, ...) need a viewer.
/// </summary>
/// <remarks>
/// OCCT transforms the bound curve or surface in place; geometry is frozen when bound (decision 3601), so the port binds
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
    }

    private static int Transform(DrawInterpreter theDI, DrawResult di, IReadOnlyList<string> a)
    {
        int n = a.Count;
        if (n < 5)
        {
            return 1;
        }

        var T = new gp_Trsf();
        int last = n - 1;
        if (a[0] == "pscale")
        {
            double s = theDI.Atof(a[last]);
            last--;
            if (last < 4)
            {
                return 1;
            }

            var P = new gp_Pnt(theDI.Atof(a[last - 2]), theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            T.SetScale(P, s);
        }
        else if (a[0][1..] == "mirror")
        {
            if (last < 4)
            {
                return 1;
            }

            var P = new gp_Pnt(theDI.Atof(a[last - 2]), theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            if (a[0][0] == 'p')
            {
                T.SetMirror(P);
            }
            else
            {
                last -= 3;
                if (last < 4)
                {
                    return 1;
                }

                var O = new gp_Pnt(theDI.Atof(a[last - 2]), theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
                last -= 3;
                var D = new gp_Dir(P.X(), P.Y(), P.Z());
                if (a[0][0] == 'l')
                {
                    T.SetMirror(new gp_Ax1(O, D));
                }
                else if (a[0][0] == 's')
                {
                    var Pl = new gp_Pln(O, D);
                    T.SetMirror(Pl.Position().Ax2());
                }
            }
        }
        else if (a[0] == "translate")
        {
            if (last < 4)
            {
                return 1;
            }

            var V = new gp_Vec(theDI.Atof(a[last - 2]), theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            last -= 3;
            T.SetTranslation(V);
        }
        else if (a[0] == "rotate")
        {
            if (last < 8)
            {
                return 1;
            }

            double ang = theDI.Atof(a[last]) * (Math.PI / 180.0);
            last--;
            var D = new gp_Dir(theDI.Atof(a[last - 2]), theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            last -= 3;
            var P = new gp_Pnt(theDI.Atof(a[last - 2]), theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            last -= 3;
            T.SetRotation(new gp_Ax1(P, D), ang);
        }

        for (int i = 1; i <= last; i++)
        {
            Geom_Geometry? G = DrawTrSurf.Get(theDI, a[i]);
            if (G is not null)
            {
                DrawTrSurf.Set(theDI, a[i], G.Transformed(T));
            }
            else
            {
                var P = new gp_Pnt();
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

        var T = new gp_Trsf2d();
        int last = n - 1;
        if (a[0] == "2dpscale")
        {
            double s = theDI.Atof(a[last]);
            last--;
            if (last < 3)
            {
                return 1;
            }

            var P = new gp_Pnt2d(theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            T.SetScale(P, s);
        }
        else if (a[0] is "2dpmirror" or "2dlmirror")
        {
            if (last < 3)
            {
                return 1;
            }

            var P = new gp_Pnt2d(theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            if (a[0] == "2dpmirror")
            {
                T.SetMirror(P);
            }
            else
            {
                last -= 2;
                if (last < 3)
                {
                    return 1;
                }

                var O = new gp_Pnt2d(theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
                last -= 2;
                var D = new gp_Dir2d(P.X(), P.Y());
                T.SetMirror(new gp_Ax2d(O, D));
            }
        }
        else if (a[0] == "2dtranslate")
        {
            if (last < 3)
            {
                return 1;
            }

            var V = new gp_Vec2d(theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            last -= 2;
            T.SetTranslation(V);
        }
        else if (a[0] == "2drotate")
        {
            if (last < 4)
            {
                return 1;
            }

            double ang = theDI.Atof(a[last]) * (Math.PI / 180.0);
            last--;
            var P = new gp_Pnt2d(theDI.Atof(a[last - 1]), theDI.Atof(a[last]));
            last -= 2;
            T.SetRotation(P, ang);
        }

        for (int i = 1; i <= last; i++)
        {
            Geom2d_Curve? G = DrawTrSurf.GetCurve2d(theDI, a[i]);
            if (G is not null)
            {
                DrawTrSurf.Set(theDI, a[i], (Geom2d_Curve)G.Transformed(T));
            }
            else
            {
                var P = new gp_Pnt2d();
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
